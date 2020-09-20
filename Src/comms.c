#include <stdio.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "comms.h"
#include "BLDC_controller.h"      /* BLDC's header file */
#include "energy.h"
#include <FastPID.h>

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

static char uart_buf[255];
static volatile int16_t ch_buf[8];
//volatile char char_buf[300];

void setScopeChannel(uint8_t ch, int16_t val) {
  ch_buf[ch] = val;
}

void consoleScope(void) {
  #if defined DEBUG_SERIAL_SERVOTERM && (defined DEBUG_SERIAL_USART2 || defined DEBUG_SERIAL_USART3)
    uart_buf[0] = 0xff;
    uart_buf[1] = CLAMP(ch_buf[0]+127, 0, 255);
    uart_buf[2] = CLAMP(ch_buf[1]+127, 0, 255);
    uart_buf[3] = CLAMP(ch_buf[2]+127, 0, 255);
    uart_buf[4] = CLAMP(ch_buf[3]+127, 0, 255);
    uart_buf[5] = CLAMP(ch_buf[4]+127, 0, 255);
    uart_buf[6] = CLAMP(ch_buf[5]+127, 0, 255);
    uart_buf[7] = CLAMP(ch_buf[6]+127, 0, 255);
    uart_buf[8] = CLAMP(ch_buf[7]+127, 0, 255);
    uart_buf[9] = '\n';

    if(UART_DMA_CHANNEL_TX->CNDTR == 0) {
      UART_DMA_CHANNEL_TX->CCR  &= ~DMA_CCR_EN;
      UART_DMA_CHANNEL_TX->CNDTR = 10;
      UART_DMA_CHANNEL_TX->CMAR  = (uint32_t)uart_buf;
      UART_DMA_CHANNEL_TX->CCR  |= DMA_CCR_EN;
    }
  #endif

  #if defined DEBUG_SERIAL_ASCII && (defined DEBUG_SERIAL_USART2 || defined DEBUG_SERIAL_USART3)
    // memset((void *)(uintptr_t)uart_buf, 0, sizeof(uart_buf));
    int strLength;
    strLength = sprintf((char *)(uintptr_t)uart_buf,
                "1:%i 2:%i 3:%i 4:%i 5:%i 6:%i 7:%i 8:%i\r\n",
                ch_buf[0], ch_buf[1], ch_buf[2], ch_buf[3], ch_buf[4], ch_buf[5], ch_buf[6], ch_buf[7]);

    consoleLog2(uart_buf,strLength);
  #endif
}

void consoleLog(char *message)
{
  #if (defined DEBUG_SERIAL_ASCII || defined CONTROL_APP_BLUETOOTH) && (defined DEBUG_SERIAL_USART2 || defined DEBUG_SERIAL_USART3)
    if(UART_DMA_CHANNEL_TX->CNDTR == 0) {
      UART_DMA_CHANNEL_TX->CCR  &= ~DMA_CCR_EN;
      UART_DMA_CHANNEL_TX->CNDTR = strlen((char *)(uintptr_t)message);
      UART_DMA_CHANNEL_TX->CMAR  = (uint32_t)message;
      UART_DMA_CHANNEL_TX->CCR  |= DMA_CCR_EN;
    }
  #endif
}
void consoleLog2(char *message, int strlength)
{
  #if (defined DEBUG_SERIAL_ASCII || defined CONTROL_APP_BLUETOOTH) && (defined DEBUG_SERIAL_USART2 || defined DEBUG_SERIAL_USART3)
    if(UART_DMA_CHANNEL_TX->CNDTR == 0) {
      UART_DMA_CHANNEL_TX->CCR  &= ~DMA_CCR_EN;
      UART_DMA_CHANNEL_TX->CNDTR = strlength;
      UART_DMA_CHANNEL_TX->CMAR  = (uint32_t)message;
      UART_DMA_CHANNEL_TX->CCR  |= DMA_CCR_EN;
    }
  #endif
}


#ifdef CONTROL_APP_BLUETOOTH
uint32_t telemetryTimer        = 0;
extern int16_t board_temp_deg_c;
extern int16_t curR_DC, curL_DC;
extern ExtY rtY_Left;                   /* External outputs */
extern ExtY rtY_Right;                  /* External outputs */
extern DW   rtDW_Left;                  /* Observable states */

extern P    rtP_Left;                   /* Block parameters (auto storage) */
extern P    rtP_Right;                  /* Block parameters (auto storage) */
extern ExtU rtU_Left;                   /* External inputs */
extern ExtU rtU_Right;                  /* External inputs */

extern int16_t batVoltage;              // global variable for battery voltage
extern int16_t  speedAvg;               // average speed
extern volatile adc_buf_t adc_buffer;
extern int16_t cmd1;                    // speed input
extern int16_t cmd2;                    // brake input
extern EnergyCounters_struct EnergyCounters;
extern uint8_t BAT_CELLS;

void SendTelemetry() {
    if (telemetryTimer %100 == 0) {  // send Temperature and voltage calibration 
                                    // only every 200th time this function is called
      sprintf((char *)(uintptr_t)uart_buf,
        "*v%i*"  // for voltage calibration
        "*T%i*" // board Temperature
        "*M%i*" // Control Type 0 = Commutation , 1 = Sinusoidal, 2 = FOC
        "*m%i*"
        "*P%li"
        "*I%li"
        "*D%li", 
        adc_buffer.batt1,
        (int16_t)board_temp_deg_c / 10, //board temperature
        rtP_Right.z_ctrlTypSel,         // control type
        rtU_Right.z_ctrlModReq,          // control mode
        (uint32_t)(((float)FastPID__p / PARAM_MULT) * 1000.0),               // speed PID-controller P value  
        (uint32_t)(((float)FastPID__i/ PARAM_MULT) * 1000.0),                // speed PID-controller I value  
        (uint32_t)(((float)FastPID__d/ PARAM_MULT) * 1000.0)                // speed PID-controller D value  
      );
    } else if(telemetryTimer%2 == 0) { // these values are sent every second time
      sprintf((char *)(uintptr_t)uart_buf,
        "*OR%iG0B0*" // Are we in Overdrive? ( is input > FIELD_WEAK_LO and speed > n_fieldWeakAuthLo) sent as RGB values
        "*E%ld*"  // Ah
        "*d%ld*"  // distance (m)
        "*W%ld*"  // Wh
        "*t%ld*"  // current telemetryTimer
        "\n",

        (
          rtP_Right.b_fieldWeakEna && 
          cmd2 > FIELD_WEAK_LO && (    ABS(rtY_Right.n_mot) > (rtP_Right.n_fieldWeakAuthLo >> 4)
                                    || ABS(rtY_Left.n_mot)  > (rtP_Left.n_fieldWeakAuthLo  >> 4) 
                                  )
        )  << 8 ,
        (uint32_t)(EnergyCounters.Ah * 1000.0),
        (uint32_t)(EnergyCounters.distance / (float)1000.0),
        (uint32_t)(EnergyCounters.Wh * 1.0),
        telemetryTimer
      );
    } else{
      sprintf((char *)(uintptr_t)uart_buf,
        "*V%i*" // Battery Voltage
        "*c%i*" // cell voltage
        "*A%i*" // sum of motor currents
        "*i%i*" // input cmd
        "*I%i" // cmd1
        "*S%i*", // average speed
        (batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC), 
        (batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC) / (BAT_CELLS), 
        (ABS(curR_DC) + ABS(curL_DC)) / A2BIT_CONV, 
        adc_buffer.l_rx2,
        cmd1,
        speedAvg
      );
    }
    consoleLog(uart_buf);
    telemetryTimer++;
}




#endif
