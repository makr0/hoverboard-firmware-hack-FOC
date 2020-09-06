
#include <stdbool.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "comms.h"
#include "control.h"
#include "stm32f1xx_ll_dma.h"
#include "BLDC_controller.h"      /* BLDC's header file */
#include <FastPID.h>



TIM_HandleTypeDef TimHandle;
TIM_HandleTypeDef TimHandle2;
uint8_t ppm_count = 0;
uint8_t pwm_count = 0;
uint32_t timeout = 100;
uint8_t nunchuk_data[6] = {0};

uint8_t i2cBuffer[2];

extern I2C_HandleTypeDef hi2c2;
extern DMA_HandleTypeDef hdma_i2c2_rx;
extern DMA_HandleTypeDef hdma_i2c2_tx;

extern uint8_t usart_rx_dma_buffer[255];
extern UART_HandleTypeDef huart3;
uint8_t new_command_available;
extern uint8_t BAT_CELLS;

#ifdef CONTROL_PPM
uint16_t ppm_captured_value[PPM_NUM_CHANNELS + 1] = {500, 500};
uint16_t ppm_captured_value_buffer[PPM_NUM_CHANNELS+1] = {500, 500};
uint32_t ppm_timeout = 0;

bool ppm_valid = true;

void PPM_ISR_Callback(void) {
  // Dummy loop with 16 bit count wrap around
  uint16_t rc_delay = TIM2->CNT;
  TIM2->CNT = 0;

  if (rc_delay > 3000) {
    if (ppm_valid && ppm_count == PPM_NUM_CHANNELS) {
      ppm_timeout = 0;
      memcpy(ppm_captured_value, ppm_captured_value_buffer, sizeof(ppm_captured_value));
    }
    ppm_valid = true;
    ppm_count = 0;
  }
  else if (ppm_count < PPM_NUM_CHANNELS && IN_RANGE(rc_delay, 900, 2100)){
    timeout = 0;
    ppm_captured_value_buffer[ppm_count++] = CLAMP(rc_delay, 1000, 2000) - 1000;
  } else {
    ppm_valid = false;
  }
}

// SysTick executes once each ms
void PPM_SysTick_Callback(void) {
  ppm_timeout++;
  // Stop after 500 ms without PPM signal
  if(ppm_timeout > 500) {
    int i;
    for(i = 0; i < PPM_NUM_CHANNELS; i++) {
      ppm_captured_value[i] = 500;
    }
    ppm_timeout = 0;
  }
}

void PPM_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct;
  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  __HAL_RCC_TIM2_CLK_ENABLE();
  TimHandle.Instance = TIM2;
  TimHandle.Init.Period = UINT16_MAX;
  TimHandle.Init.Prescaler = (SystemCoreClock/DELAY_TIM_FREQUENCY_US)-1;;
  TimHandle.Init.ClockDivision = 0;
  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  HAL_TIM_Base_Init(&TimHandle);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  HAL_TIM_Base_Start(&TimHandle);
}
#endif


#ifdef CONTROL_PWM
uint16_t pwm_captured_ch1_value = 500;
uint16_t pwm_captured_ch2_value = 500;
uint32_t pwm_timeout_ch1 = 0;
uint32_t pwm_timeout_ch2 = 0;

void PWM_ISR_CH1_Callback(void) {
  // Dummy loop with 16 bit count wrap around
  uint16_t rc_signal = TIM3->CNT;
  TIM3->CNT = 0;

  if (IN_RANGE(rc_signal, 900, 2100)){
    timeout = 0;
    pwm_timeout_ch1 = 0;
    pwm_captured_ch1_value = CLAMP(rc_signal, 1000, 2000) - 1000;
  }
}


void PWM_ISR_CH2_Callback(void) {
  // Dummy loop with 16 bit count wrap around
  uint16_t rc_signal = TIM2->CNT;
  TIM2->CNT = 0;

  if (IN_RANGE(rc_signal, 900, 2100)){
    timeout = 0;
    pwm_timeout_ch2 = 0;
    pwm_captured_ch2_value = CLAMP(rc_signal, 1000, 2000) - 1000;
  }
}

// SysTick executes once each ms
void PWM_SysTick_Callback(void) {
  pwm_timeout_ch1++;
  pwm_timeout_ch2++;
  // Stop after 500 ms without PWM signal
  if(pwm_timeout_ch1 > 500) {
    pwm_captured_ch1_value = 500;
    pwm_timeout_ch1 = 500;          // limit the timeout to max timeout value of 500 ms
  }
  if(pwm_timeout_ch2 > 500) {
    pwm_captured_ch2_value = 500;
    pwm_timeout_ch2 = 500;          // limit the timeout to max timeout value of 500 ms
  }
}

void PWM_Init(void) {
  // Channel 1 (steering)
  GPIO_InitTypeDef GPIO_InitStruct2;
  // Configure GPIO pin : PA2
  GPIO_InitStruct2.Pin          = GPIO_PIN_2;
  GPIO_InitStruct2.Mode         = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct2.Speed        = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct2.Pull         = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct2);

  __HAL_RCC_TIM3_CLK_ENABLE();
  TimHandle2.Instance           = TIM3;
  TimHandle2.Init.Period        = UINT16_MAX;
  TimHandle2.Init.Prescaler     = (SystemCoreClock/DELAY_TIM_FREQUENCY_US)-1;;
  TimHandle2.Init.ClockDivision = 0;
  TimHandle2.Init.CounterMode   = TIM_COUNTERMODE_UP;
  HAL_TIM_Base_Init(&TimHandle2);

  // EXTI interrupt init
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  HAL_TIM_Base_Start(&TimHandle2);

  // Channel 2 (speed)
  GPIO_InitTypeDef GPIO_InitStruct;
  /*Configure GPIO pin : PA3 */
  GPIO_InitStruct.Pin             = GPIO_PIN_3;
  GPIO_InitStruct.Mode            = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Speed           = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Pull            = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  __HAL_RCC_TIM2_CLK_ENABLE();
  TimHandle.Instance              = TIM2;
  TimHandle.Init.Period           = UINT16_MAX;
  TimHandle.Init.Prescaler        = (SystemCoreClock/DELAY_TIM_FREQUENCY_US)-1;;
  TimHandle.Init.ClockDivision    = 0;
  TimHandle.Init.CounterMode      = TIM_COUNTERMODE_UP;
  HAL_TIM_Base_Init(&TimHandle);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  HAL_TIM_Base_Start(&TimHandle);

  #ifdef SUPPORT_BUTTONS  
  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin             = BUTTON1_RIGHT_PIN;
  GPIO_InitStruct.Mode            = GPIO_MODE_INPUT;
  GPIO_InitStruct.Speed           = GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct.Pull            = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON1_RIGHT_PORT, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct2.Pin            = BUTTON2_RIGHT_PIN;
  GPIO_InitStruct2.Mode           = GPIO_MODE_INPUT;
  GPIO_InitStruct2.Speed          = GPIO_SPEED_FREQ_MEDIUM;
  GPIO_InitStruct2.Pull           = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON2_RIGHT_PORT, &GPIO_InitStruct2);
  #endif
}
#endif

uint8_t Nunchuk_Ping(void) {
  if (HAL_I2C_Master_Receive(&hi2c2,0xA4,(uint8_t*)nunchuk_data, 1, 10) == HAL_OK) {
    return 1;
  }
  return 0;
}

void Nunchuk_Init(void) {
    //-- START -- init WiiNunchuk
  i2cBuffer[0] = 0xF0;
  i2cBuffer[1] = 0x55;

  HAL_I2C_Master_Transmit(&hi2c2,0xA4,(uint8_t*)i2cBuffer, 2, 100);
  HAL_Delay(10);

  i2cBuffer[0] = 0xFB;
  i2cBuffer[1] = 0x00;

  HAL_I2C_Master_Transmit(&hi2c2,0xA4,(uint8_t*)i2cBuffer, 2, 100);
  HAL_Delay(10);
}

void Nunchuk_Read(void) {
  i2cBuffer[0] = 0x00;
  HAL_I2C_Master_Transmit(&hi2c2,0xA4,(uint8_t*)i2cBuffer, 1, 10);
  HAL_Delay(3);
  if (HAL_I2C_Master_Receive(&hi2c2,0xA4,(uint8_t*)nunchuk_data, 6, 10) == HAL_OK) {
    timeout = 0;
  }

  #ifndef TRANSPOTTER
    if (timeout > 3) {
      HAL_Delay(50);
      Nunchuk_Init();
    }
  #endif

  //setScopeChannel(0, (int)nunchuk_data[0]);
  //setScopeChannel(1, (int)nunchuk_data[1]);
  //setScopeChannel(2, (int)nunchuk_data[5] & 1);
  //setScopeChannel(3, ((int)nunchuk_data[5] >> 1) & 1);
}

#if defined(CONTROL_APP_BLUETOOTH)
extern ExtY rtY_Left;                   /* External outputs */
extern ExtY rtY_Right;                  /* External outputs */

extern P    rtP_Left;                   /* Parameters for left motor */
extern P    rtP_Right;                  /* right motor */
extern ExtU rtU_Left;                   /* External inputs */
extern ExtU rtU_Right;                   /* External inputs */
extern uint8_t  ctrlModReq;             // global variable for control mode request 
extern int16_t  speedAvg;               // average speed
int16_t controlTypeSwitchUpSpeed = 600;   // switch up to sinus control type at this speed
int16_t controlTypeSwitchDownSpeed = 550; // switch down to FOC control type at this speed
extern Setpoints_struct Setpoints;               // setpoints for FastPID (speed,accel)

 void USART3_IRQHandler(void) {
    /* Check for IDLE line interrupt */
    if (READ_BIT(USART3->CR1, USART_CR1_IDLEIE) == (USART_CR1_IDLEIE) &&  READ_BIT(USART3->SR, USART_SR_IDLE) == (USART_SR_IDLE))
    {
        /* Clear IDLE line flag */
        __IO uint32_t tmpreg;
        tmpreg = USART3->SR;
        (void) tmpreg;
        tmpreg = USART3->DR;
        (void) tmpreg;
        app_rx_process_data();                       /* Check for data to process */
    }
}
#endif

void app_rx_process_data() {
  int len;

  len = sizeof(usart_rx_dma_buffer) - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_3);
  usart_rx_dma_buffer[len-1]=0;
  HAL_UART_DMAStop(&huart3);
  HAL_UART_Receive_DMA(&huart3, (uint8_t *)&usart_rx_dma_buffer, sizeof(usart_rx_dma_buffer));   
  new_command_available=1;
}
/**
 * interpret command and execute
 * there are 2 types of command: query and execute
 * query commands start with '?'
 * execute commands start with '!'
 * all commands end with '.'
 * 
 * Setup/Tuning Commands, sent once per value change
 * 
 * PID commands. each component (P,I,D) can be omitted. Values are stored as integer
 * !speedP1234.!speedI42.!speedD23. set speedPID to P=1234 I=42 D=23
 * 
 * simple values. are stored as integers
 * !maxcN20.                set max overall current to 20A (10A per motor)
 * !cellN10.                set number of Cells to 10 (equals BAT_NUMBER_OF_CELLS on startup)
 * !ctrlT2.                 set control type to 'FOC'. equivalent to #define CTRL_TYP_SEL in config.h
 *                          Control type selection: 0 = Commutation ,
 *                                                  1 = Sinusoidal,
 *                                                  2 = FOC Field Oriented Control
 * 
 * !ctrlM3.                 set control mode to TORQUE. equivalent to #define CTRL_MOD_REQ  
 *                          Control mode request: 0 = Open mode
 *                                                1 = VOLTAGE mode
 *                                                2 = SPEED mode
 *                                                3 = TORQUE mode.
 *                                              SPEED and TORQUE modes are only available for FOC!
 * motor model parameters
 * !maxRPM600.              set max RPM for both motors to 600
 * 
 */

void AppExecuteCommand() {
  if(!new_command_available) return;


  if(strStartsWith("!maxRPM", usart_rx_dma_buffer)) {
    int16_t rpm = atoi(usart_rx_dma_buffer+strlen("!maxRPM"));
    if(rpm > 0 && rpm <= 1500) {
      Setpoints.speed = rpm; // fixdt(1,16,4)
    }
    sendNewValue("*R%i*",rpm);
  }

  if(strStartsWith("!fieldWstart", usart_rx_dma_buffer)) {
    int16_t value = atoi(usart_rx_dma_buffer+strlen("!fieldWstart"));
    if(value >= 100 && value < (rtP_Right.n_fieldWeakAuthHi >> 4)) {
      rtP_Right.n_fieldWeakAuthLo = value << 4; // fixdt(1,16,4)
      rtP_Left.n_fieldWeakAuthLo = rtP_Right.n_fieldWeakAuthLo; // fixdt(1,16,4)
    }
    sendNewValue("*f%i*",rtP_Right.n_fieldWeakAuthLo >> 4);
  }
  if(strStartsWith("!fieldWend", usart_rx_dma_buffer)) {
    int16_t value = atoi(usart_rx_dma_buffer+strlen("!fieldWend"));
    if(value > (rtP_Right.n_fieldWeakAuthLo >> 4) && value < 1000) {
      rtP_Right.n_fieldWeakAuthHi = value << 4; // fixdt(1,16,4)
      rtP_Left.n_fieldWeakAuthHi = rtP_Right.n_fieldWeakAuthHi; // fixdt(1,16,4)
    }
    sendNewValue("*F%i*",rtP_Right.n_fieldWeakAuthHi >> 4);
  }
  if(strStartsWith("!fieldWenable", usart_rx_dma_buffer)) {
      rtP_Right.b_fieldWeakEna = 1;
      rtP_Left.b_fieldWeakEna = 1;
  }
  if(strStartsWith("!fieldWdisable", usart_rx_dma_buffer)) {
      rtP_Right.b_fieldWeakEna = 0;
      rtP_Left.b_fieldWeakEna = 0;
  }
  if(strStartsWith("!numcells", usart_rx_dma_buffer)) {
    int16_t value = atoi(usart_rx_dma_buffer+strlen("!numcells"));
    if(value >= 10 && value <= 15) {
      BAT_CELLS = value;
    }
    sendNewValue("*B%i*",BAT_CELLS);
  }
  if(strStartsWith("!numcells", usart_rx_dma_buffer)) {
    int16_t value = atoi(usart_rx_dma_buffer+strlen("!numcells"));
    if(value >= 10 && value <= 15) {
      BAT_CELLS = value;
    }
    sendNewValue("*B%i*",BAT_CELLS);
  }
  if(strStartsWith("!ctrlT", usart_rx_dma_buffer)) {
    int16_t value = atoi(usart_rx_dma_buffer+strlen("!ctrlT"));
    if(value >= 0 && value <= 2) {
      rtP_Right.z_ctrlTypSel = value;
      rtP_Left.z_ctrlTypSel = value;
      sendNewValue("*M%i*",value);
    }
  }
  if(strStartsWith("!ctrlM", usart_rx_dma_buffer)) {
    int16_t value = atoi(usart_rx_dma_buffer+strlen("!ctrlM"));
    if(value >= 0 && value <= 3) {
      if(value == 2) { // speed mode maps to external PID
        ctrlModReq = 3; //FOC
        Setpoints.enabled = true;
        FastPID_clear();
      } else {
        ctrlModReq= value;
        Setpoints.enabled = false;
      }
      sendNewValue("*m%i*",value);
    }
  }
  if(strStartsWith("!switchUp", usart_rx_dma_buffer)) {
    int16_t value = atoi(usart_rx_dma_buffer+strlen("!switchUp"));
    if(value >= 0 && value <= 1000) {
      controlTypeSwitchUpSpeed = value;
      sendNewValue("*U%i*",value);
    }
  }
  if(strStartsWith("!switchDown", usart_rx_dma_buffer)) {
    int16_t value = atoi(usart_rx_dma_buffer+strlen("!switchDown"));
    if(value >= 0 && value <= 1000) {
      controlTypeSwitchDownSpeed = value;
      sendNewValue("*u%i*",value);
    }
  }
  if(strStartsWith("!speedP", usart_rx_dma_buffer)) {
    int16_t value = atoi(usart_rx_dma_buffer+strlen("!speedP"));
    if(value >= 0 && value <= 3000) {
      FastPID_setCoefficient_P(value/1000.0);
      sendNewValue("*P%li*",(uint32_t)(((float)FastPID__p / PARAM_MULT) * 1000.0));
    }
  }
  if(strStartsWith("!speedI", usart_rx_dma_buffer)) {
    int16_t value = atoi(usart_rx_dma_buffer+strlen("!speedI"));
    if(value >= 0 && value <= 3000) {
      FastPID_setCoefficient_I(value/1000.0);
      sendNewValue("*I%li*",(uint32_t)(((float)FastPID__i / PARAM_MULT) * 1000.0));
    }
  }
  if(strStartsWith("!speedD", usart_rx_dma_buffer)) {
    int16_t value = atoi(usart_rx_dma_buffer+strlen("!speedD"));
    if(value >= 0 && value <= 3000) {
      FastPID_setCoefficient_D(value/1000.0);
      sendNewValue("*D%li*",(uint32_t)(((float)FastPID__d / PARAM_MULT) * 1000.0));
    }
  }

  new_command_available=0;
}

void ExecuteAutoControl() {
  if(rtP_Right.z_ctrlTypSel == 1 // sinus
     && speedAvg <= controlTypeSwitchDownSpeed )
  {
      rtP_Right.z_ctrlTypSel = 2; // FOC
      rtP_Left.z_ctrlTypSel = 2;
      return;
  }
  if(rtP_Right.z_ctrlTypSel == 2 // FOC
     && speedAvg >= controlTypeSwitchUpSpeed )
  {
      rtP_Right.z_ctrlTypSel = 1; // sinus
      rtP_Left.z_ctrlTypSel = 1;
      return;
  }
}

void sendNewValue(char *format, int16_t value) {
  static volatile uint8_t uart_buf[255];
  sprintf((char *)(uintptr_t)uart_buf, format, value);
  HAL_Delay(200);
  consoleLog(uart_buf);
}

int strStartsWith(char *pre, char *str)
{
    return strncmp(pre, str, strlen(pre)) == 0;
}
