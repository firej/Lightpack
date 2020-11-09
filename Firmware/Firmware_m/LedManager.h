/*
 * LedManager.h
 *
 *  Created on: 28.09.2011
 *     Project: Lightpack
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

#ifndef LEDMANAGER_H_INCLUDED
#define LEDMANAGER_H_INCLUDED
#include <stdlib.h>

#define LMMODE_HOST		0
#define LMMODE_MOODLAMP	1

extern void LedManager_UpdateColors(void);
extern void LedManager_FillImages(const uint8_t red, const uint8_t green, const uint8_t blue);
extern void EvalCurrentImage_SmoothlyAlgIteration(void);
extern void LedManager_FlashLedsRed(void);
extern void LedManager_LitLeds( uint8_t count );

extern void LedManager_setMode( uint8_t mode );
extern bool LEDManager_acceptsUpdates(void);
extern void LedManager_adjustGamma( int8_t _delta );
extern const uint8_t* LEDManager_getCurrentGammaTable( void );

#endif /* LEDMANAGER_H_INCLUDED */

