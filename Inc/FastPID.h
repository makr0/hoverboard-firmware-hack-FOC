#ifndef FastPID_H
#define FastPID_H

#include <stdint.h>
#include <stdbool.h>

#define INTEG_MAX    (INT64_MAX >> 1)
#define INTEG_MIN    (INT64_MIN >> 1)

#define PARAM_SHIFT  21
#define PARAM_BITS   25
#define PARAM_MAX    (((0x1ULL << PARAM_BITS)-1) >> PARAM_SHIFT) 
#define PARAM_MULT   (((0x1ULL << PARAM_BITS)-1) >> (PARAM_BITS - PARAM_SHIFT)) 

/*
  A fixed point PID controller with a 64-bit internal calculation pipeline.
*/

  typedef struct{
    int16_t speed; 
    int16_t accel;
    bool enabled;
  } Setpoints_struct;

  // Configuration
  uint32_t FastPID__p, FastPID__i, FastPID__d;
  uint32_t FastPID__db;
  int64_t FastPID__outmax, FastPID__outmin; 
  bool FastPID__cfg_err; 
  
  // State
  int16_t FastPID__last_sp, FastPID__last_out;
  int64_t FastPID__sum;
  int32_t FastPID__last_err;
  uint32_t FastPID__last_run;

  void FastPID_init();
  bool FastPID_setCoefficients(float kp, float ki, float kd);
  bool FastPID_setCoefficient_P(float kp);
  bool FastPID_setCoefficient_I(float ki);
  bool FastPID_setCoefficient_D(float kd);
  
  bool FastPID_setDeadband(uint16_t db);
  bool FastPID_setOutputConfig(int bits, bool sign);
  void FastPID_clear();
  bool FastPID_configure(float kp, float ki, float kd, uint16_t db, int bits, bool sign);
  int16_t FastPID_step(int16_t sp, int16_t fb);

  bool FastPID_err();

  uint32_t FastPID_floatToParam(float); 
  void FastPID_setCfgErr(); 

#endif
