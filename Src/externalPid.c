/**
  * This file is part of the hoverboard-firmware-hack project.
  *
  * Copyright (C) 2020-2021 Emanuel FERU <aerdronix@gmail.com>
  *
  * This program is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Includes
#include <stdlib.h> // for abs()
#include "defines.h"

#include "config.h"
#include "control.h"
#include "BLDC_controller.h"           /* Model's header file */
#include "FastPID.h"
#include "externalPid.h"


ExternalPid_struct ExternalPid_coefficients;
extern int16_t curR_DC, curL_DC;
extern DW   rtDW_Left;                  /* Observable states */
extern DW   rtDW_Right;                 /* Observable states */
extern ExtY rtY_Left;                   /* External outputs */
extern ExtY rtY_Right;                  /* External outputs */
extern int16_t batVoltage;              // global variable for battery voltage
extern Setpoints_struct Setpoints;      // setpoints for externalPID (speed,accel)


void ExternalPid_Init() {
  ExternalPid_coefficients.P=5;
  ExternalPid_coefficients.I=0;
  ExternalPid_coefficients.D=0;
  FastPID_configure(ExternalPid_coefficients.P,ExternalPid_coefficients.I,ExternalPid_coefficients.D,0,16,true);
  Setpoints.enabled=true;
  Setpoints.speed=100;
}
void ExternalPid_setCoefficients(ExternalPid_struct parameters) {
  FastPID_setCoefficients(parameters.P,parameters.I,parameters.D);
}

uint16_t ExternalPID_Step(int16_t setpoint, int16_t feedback) {
  return FastPID_step(setpoint, feedback);
}
