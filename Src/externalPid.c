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
#include "externalPid.h"
#include "config.h"
#include "BLDC_controller.h"           /* Model's header file */

ExternalPid_struct ExternalPid_coefficients;
extern int16_t curR_DC, curL_DC;
extern DW   rtDW_Left;                  /* Observable states */
extern DW   rtDW_Right;                 /* Observable states */
extern ExtY rtY_Left;                   /* External outputs */
extern ExtY rtY_Right;                  /* External outputs */
extern int16_t batVoltage;              // global variable for battery voltage
FastPID ExternalPid(0, 0, 0, 0, 16, true);

void ExternalPid_Init(ExternalPid_struct parameters) {
  ExternalPid.configure(parameters.P,parameters.I,parameters.D,0,16,true);
}
void ExternalPid_setCoefficients(ExternalPid_struct parameters) {
  ExternalPid.setCoefficients(parameters.P,parameters.I,parameters.D);
}

void ExternalPID_Step(int16_t setpoint, int16_t feedback) {
  ExternalPid.step(setpoint, feedback, HAL_GetTick());
}
