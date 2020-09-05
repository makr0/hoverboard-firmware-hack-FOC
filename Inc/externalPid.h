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

// Define to prevent recursive inclusion
#ifndef EXTERNALPID_H
#define EXTERNALPID_H

#include <stdint.h>
#include <FastPID.h>

typedef struct{
  float P; // 
  float I;
  float D;  
} ExternalPid_struct;

// Initialization Functions
void ExternalPid_Init(ExternalPid_struct parameters);
void ExternalPid_setCoefficients(ExternalPid_struct parameters);

// Controller step
void ExternalPID_Step(int16_t setpoint, int16_t feedback, uint32_t timestamp);

#endif

