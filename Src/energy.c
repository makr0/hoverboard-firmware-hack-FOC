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
#include "energy.h"
#include "config.h"
#include "BLDC_controller.h"           /* Model's header file */

EnergyCounters_struct EnergyCounters;
extern int16_t curR_DC, curL_DC;
extern DW   rtDW_Left;                  /* Observable states */
extern DW   rtDW_Right;                 /* Observable states */
extern ExtY rtY_Left;                   /* External outputs */
extern ExtY rtY_Right;                  /* External outputs */
extern int16_t batVoltage;              // global variable for battery voltage


void Energycounters_Init(uint16_t c) {
  EnergyCounters.calls_per_second = c;
  EnergyCounters.n=0;
  EnergyCounters.Ah=0;
  EnergyCounters.Wh=0;
  EnergyCounters.distance=0;
  EnergyCounters.i_Ah=0;
  EnergyCounters.i_nmot=0;
  EnergyCounters.wheel_circumfence = WHEEL_CIRCUMFENCE;
}

// Integration step
void Energycounters_Step(void) {
  EnergyCounters.n++;
  EnergyCounters.i_Ah += (ABS(curR_DC) + ABS(curL_DC)) / A2BIT_CONV;
  uint32_t sum = (ABS(rtY_Right.n_mot) + ABS(rtY_Left.n_mot));
  EnergyCounters.i_nmot += sum;
  if(EnergyCounters.n == EnergyCounters.calls_per_second ) {
    float rpm_last_second = (float)EnergyCounters.i_nmot / 2.0 / (float)EnergyCounters.calls_per_second;
    float rotations = rpm_last_second / 6.0;
    EnergyCounters.distance += (uint32_t)rotations * (float)EnergyCounters.wheel_circumfence / 10.0;

    float average_Ampere_last_second = EnergyCounters.i_Ah / (float)EnergyCounters.calls_per_second;

    EnergyCounters.Ah += average_Ampere_last_second / 3600.0;
    EnergyCounters.Wh = (batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC) * EnergyCounters.Ah;
    EnergyCounters.i_Ah=0;
    EnergyCounters.n=0;
    EnergyCounters.i_nmot=0;
  }
}
