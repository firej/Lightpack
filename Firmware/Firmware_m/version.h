/*
 * version.h
 *
 *  Created on: 03.11.2010
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

#ifndef VERSION_H_INCLUDED
#define VERSION_H_INCLUDED

#ifndef LIGHTPACK_HW
#error LIGHTPACK_HW should be passed as argument to make
#endif

#define SOFTWARE_VERSION 0x06UL

#if(LIGHTPACK_HW == 7)
#define VERSION_OF_FIRMWARE              (0x0700UL + SOFTWARE_VERSION)
#elif(LIGHTPACK_HW == 6)
#define VERSION_OF_FIRMWARE              (0x0600UL + SOFTWARE_VERSION)
#elif (LIGHTPACK_HW == 5)
#define VERSION_OF_FIRMWARE              (0x0500UL + SOFTWARE_VERSION)
#elif (LIGHTPACK_HW == 4)
#define VERSION_OF_FIRMWARE              (0x0400UL + SOFTWARE_VERSION)
#else
#error Unknown Lightpach HW version, check LIGHTPACK_HW value
#endif

#define VERSION_OF_FIRMWARE_MAJOR        ((VERSION_OF_FIRMWARE & 0xff00) >> 8)
#define VERSION_OF_FIRMWARE_MINOR         (VERSION_OF_FIRMWARE & 0x00ff)


// Use Bit Angle Modulation (BAM) instead of Pulse Width Modulation (PWM)
#define USE_BAM 0

#endif /* VERSION_H_INCLUDED */
