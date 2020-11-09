/*
 * datatypes.h
 *
 *  Created on: 13.04.2011
 *      Author: Mike Shatohin (brunql)
 *     Project: Lightpack
 *
 *  Lightpack is a content-appropriate ambient lighting system for any computer
 *
 *  Copyright (c) 2011 Mike Shatohin, mikeshatohin [at] gmail.com
 *
 *  Lightpack is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  Lightpack is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef DATATYPES_H_INCLUDED
#define DATATYPES_H_INCLUDED

#include <stdint.h>
#include "../CommonHeaders/LEDS_COUNT.h"

//Number of emulated Lightpack devices
#define LIGHTPACKS_COUNT	1

//Uncomment to enable zones resampling
#define RESAMPLE 1

//absolute lightpack zone indices on left edge (range)(zero-based indices)
#define LZ_LEFT1 	1
#define LZ_LEFT2 	3

//absolute lightpack zone indices on right edge
#define LZ_RIGHT1 	6
#define LZ_RIGHT2 	8

//absolute lightpack zone indices on top edge
#define LZ_TOP1		3
#define LZ_TOP2 	6

//absolute lightpack zone indices on bottom edge
#define LZ_BOTTOM1	8
#define LZ_BOTTOM2 	(1+10)

//strip zones on left edge (range)(zero-bazed indices)
#define RZ_LEFT1	0
#define RZ_LEFT2	4

//strip zones on right edge
#define RZ_RIGHT1	14
#define RZ_RIGHT2	18

//strip zones on top edge
#define RZ_TOP1		5
#define RZ_TOP2		13

//strip zones on top edge
#define RZ_BOTTOM1		19
#define RZ_BOTTOM2		27


//==================

#define RZ_LEFT 	( RZ_LEFT2 - RZ_LEFT1 + 1)
#define RZ_RIGHT 	( RZ_RIGHT2 - RZ_RIGHT1 + 1)
#define RZ_TOP 		( RZ_TOP2 - RZ_TOP1 + 1)
#define RZ_BOTTOM 	( RZ_BOTTOM2 - RZ_BOTTOM1 + 1)

#define TOTALRZONES ( RZ_LEFT + RZ_RIGHT + RZ_TOP + RZ_BOTTOM )

#if (LIGHTPACK_HW >= 6)
typedef struct
{
    uint8_t r;
    uint8_t b;
    uint8_t g;

} RGB_t;
#else /*(LIGHTPACK_HW >= 6)*/
typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;

} RGB_t;
#endif

typedef struct
{
    RGB_t start[LEDS_COUNT];
    RGB_t end[LEDS_COUNT];

} Images_t;

typedef struct
{
    RGB_t current[LEDS_COUNT];

} Images_t2;

typedef struct
{
    //uint8_t isSmoothEnabled;
    uint8_t smoothSlowdown;
    //uint8_t brightness;
    //uint8_t maxPwmValue;
    //uint16_t timerOutputCompareRegValue;

} Settings_t;

#endif /* DATATYPES_H_INCLUDED */
