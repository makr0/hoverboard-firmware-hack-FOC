
#include <stdbool.h>
#include <stdint.h>
#include "control.h"
#include <FastPID.h>

Setpoints_struct Setpoints;      // setpoints for externalPID (speed,accel)

void FastPID_init() {
  FastPID_configure(5,0,0,0,16,true);
  Setpoints.enabled=true;
  Setpoints.speed=100;
}

void FastPID_clear() {
  FastPID__last_sp = 0; 
  FastPID__last_out = 0;
  FastPID__sum = 0; 
  FastPID__last_err = 0;
  FastPID__last_run = 0;
}

bool FastPID_setCoefficients(float kp, float ki, float kd) {
  FastPID__p = FastPID_floatToParam(kp);
  FastPID__i = FastPID_floatToParam(ki);
  FastPID__d = FastPID_floatToParam(kd);
  return ! FastPID__cfg_err;
}

bool FastPID_setDeadband(uint16_t db) {
  if (db != 0 && FastPID__i == 0 && FastPID__d == 0) {
    // Deadband causes permanent offsets in P controllers.
    // don't let a user do this.
    FastPID__db = 0;
    FastPID_setCfgErr();
    return ! FastPID__cfg_err;
  }
  FastPID__db = (uint32_t)db * PARAM_MULT;
  return ! FastPID__cfg_err;
}

bool FastPID_setOutputConfig(int bits, bool sign) {
  // Set output bits
  if (bits > 16 || bits < 1) {
    FastPID__cfg_err = true; 
  }
  else {
    if (sign) {
      FastPID__outmax = ((0x1ULL << (bits - 1)) - 1) * PARAM_MULT;
      FastPID__outmin = -((0x1ULL << (bits - 1))) * PARAM_MULT;
    }
    else {
      FastPID__outmax = ((0x1ULL << bits) - 1) * PARAM_MULT;
      FastPID__outmin = 0;
    }
  }
  return ! FastPID__cfg_err;
}

bool FastPID_configure(float kp, float ki, float kd, uint16_t db, int bits, bool sign) {
  FastPID_clear();
  FastPID_setCoefficients(kp, ki, kd);
  FastPID_setDeadband(db);
  FastPID_setOutputConfig(bits, sign);
  return ! FastPID__cfg_err; 
}

uint32_t FastPID_floatToParam(float in) {
  if (in > PARAM_MAX || in < 0) {
    FastPID__cfg_err = true;
    return 0;
  }
  return in * PARAM_MULT;
}

bool FastPID_err() {
    return FastPID__cfg_err;
}

int16_t FastPID_step(int16_t sp, int16_t fb) {

  uint32_t now = HAL_GetTick();
  // Calculate delta T
  // millis(): Frequencies less than 1Hz become 1Hz. 
  //   max freqency 1 kHz (XXX: is this too low?)
  uint32_t hz = 0;
  if (FastPID__last_run == 0) {
    // Ignore I and D on the first step. They will be 
    // unreliable because no time has really passed.
    hz = 0;
  }
  else {
    hz = (uint32_t)(1000) / (now - FastPID__last_run); 
    if (hz == 0) 
      hz = 1;
  }

  FastPID__last_run = now;

  // int16 + int16 = int17
  int32_t err = (int32_t)(sp) - (int32_t)(fb); 
  int64_t P = 0, I = 0, D = 0;

  if (FastPID__p) {
    // uint23 * int16 = int39
    P = (int64_t)(FastPID__p) * (int64_t)(err);
  }

  if (FastPID__i && hz) {
    // int31 + ( int25 *  int17) / int10  = int43
    FastPID__sum += ((int64_t)(FastPID__i) * (int32_t)(err)) / (int32_t)(hz); 

    // Limit sum to 31-bit signed value so that it saturates, never overflows.
    if (FastPID__sum > INTEG_MAX)
      FastPID__sum = INTEG_MAX;
    else if (FastPID__sum < INTEG_MIN)
      FastPID__sum = INTEG_MIN;

    // int43
    I = (int64_t)(FastPID__sum);
  }

  if (FastPID__d && hz) {
    // int17 - (int16 - int16) = int19
    int32_t deriv = (err - FastPID__last_err) - (sp - FastPID__last_sp);
    FastPID__last_sp = sp; 
    FastPID__last_err = err; 

    // uint23 * int19 * uint16 = int58
    D = (int64_t)(FastPID__d) * (int64_t)(deriv) * (int64_t)(hz);
  }

  // int39 (P) + int43 (I) + int58 (D) = int61
  int64_t out = P + I + D;

  // Apply the deadband. 
  if (FastPID__db != 0 && out != 0) {
    if (out < 0) {
      if (-out < FastPID__db) {
	out = 0;
      }
    }
    else {
      if (out < FastPID__db) {
	out = 0;
      }
    }
  }
  
  // Make the output saturate
  if (out > FastPID__outmax) 
    out = FastPID__outmax;
  else if (out < FastPID__outmin) 
    out = FastPID__outmin;

  // Remove the integer scaling factor. 
  int16_t rval = out >> PARAM_SHIFT;

  // Fair rounding.
  if (out & (0x1ULL << (PARAM_SHIFT - 1))) {
    rval++;
  }

  return rval;
}

void FastPID_setCfgErr() {
  FastPID__cfg_err = true;
  FastPID__p = FastPID__i = FastPID__d = 0;
}
