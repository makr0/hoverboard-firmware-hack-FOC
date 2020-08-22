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
#ifndef ENERGY_H
#define ENERGY_H

#include <stdint.h>

typedef struct{
  uint16_t calls_per_second; // how many times per second this structure is updated 
  uint32_t n;             // elapsed calls, reset once per second
  float   Ah;          // Ampere Hours, updated once per second
  float   Wh;          // Watt Hours (depends on Voltage), updated once per second
  uint32_t  distance;     // distance driven (mm) 32bit is long enough for 4000km
  float i_Ah;          // for integration of current
  uint32_t i_nmot;        // for integration of distance
  uint16_t wheel_circumfence; // in mm
  
} EnergyCounters_struct;

// Initialization Functions
void Energycounters_Init(uint16_t c);

// Integration step
void Energycounters_Step(void);

#endif

