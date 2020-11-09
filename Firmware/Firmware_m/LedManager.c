/*
 * LedManager.c
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
#include <avr/wdt.h>
#include <stdlib.h>
#include <avr/eeprom.h>

#include "Lightpack.h"
#include "LedDriver.h"
#include "LedManager.h"

uint8_t s_mode = LMMODE_HOST;
RGB_t ml_start;
RGB_t ml_end;
uint8_t ml_smoothindex2 = 0;
volatile uint8_t ml_smoothindex = 0;
volatile uint8_t soundBeatIndex = 0;
bool s_BeatOn = false;

//=========================================
uint8_t clamp255( int16_t value )
{
	if ( value > 255 )
	{
		return 255;
	}
	else if ( value < 0 )
	{
		return 0;
	}
	else
	{
		return value;
	}
}

//=========================================
uint8_t clamp255max( int16_t value )
{
	if ( value > 255 )
	{
		return 255;
	}
	else
	{
		return value;
	}
}


void LedManager_FillImages(const uint8_t red, const uint8_t green, const uint8_t blue)
{
	for ( uint8_t j = 0; j < LIGHTPACKS_COUNT; j++ )
	{
		Images_t* g_Images1 = &(g_ImagesEx1[j]);
		Images_t2* g_Images2 = &(g_ImagesEx2[j]);
		for (uint8_t i = 0; i < LEDS_COUNT; i++)
		{

			g_Images1->start[i].r = red;
			g_Images1->start[i].g = green;
			g_Images1->start[i].b = blue;

			g_Images2->current[i].r = red;
			g_Images2->current[i].g = green;
			g_Images2->current[i].b = blue;

			g_Images1->end[i].r = red;
			g_Images1->end[i].g = green;
			g_Images1->end[i].b = blue;
		}
	}
}

#if (LIGHTPACK_HW >= 6)

/*
void EvalCurrentImage_SmoothlyAlg(void)
{
	for ( uint8_t j = 1; j < LIGHTPACKS_COUNT-2; j++ )
	{
		Images_t* g_Images1 = &(g_ImagesEx1[j]);
		Images_t2* g_Images2 = &(g_ImagesEx2[j]);

		uint8_t smoothIndex = g_smoothIndex[j];
		uint8_t smoothIndexMax = g_Settings.smoothSlowdown;
		if ( smoothIndexMax == 0 )
		{
			smoothIndex = 1;
			smoothIndexMax = 1;
		}

		uint16_t coefEnd = ((uint16_t)smoothIndex << 8) / smoothIndexMax;
		uint16_t coefStart = (1UL << 8) - coefEnd;

		for (uint8_t i = 0; i < LEDS_COUNT; i++)
		{
			{
				g_Images2->current[i].r = (
						coefStart * g_Images1->start[i].r +
						coefEnd   * g_Images1->end  [i].r) >> 8;

				g_Images2->current[i].g = (
						coefStart * g_Images1->start[i].g +
						coefEnd   * g_Images1->end  [i].g) >> 8;

				g_Images2->current[i].b = (
						coefStart * g_Images1->start[i].b +
						coefEnd   * g_Images1->end  [i].b) >> 8;

			}
		}
		if ((smoothIndex >= g_Settings.smoothSlowdown) )
		{
			g_smoothIndex[j] = g_Settings.smoothSlowdown;
		}
		else
		{
			g_smoothIndex[j]++;
		}
	}
}
*/

uint8_t sa_j = 0;
uint8_t sa_i = 0;

const uint8_t* LEDManager_getCurrentGammaTable( void )
{
	/*
	if ( TST((D,4)) )
	{
		return &(gammaTable[ getGammaIndex() + 1 ][0]);
	}
	else
	{
		return &(gammaTable[ getGammaIndex() ][0]);
	}
	*/
	return &(gammaTable[ getGammaIndex() ][0]);
}

void EvalCurrentImage_SmoothlyAlgIteration(void)
{
	Images_t* g_Images1 = &(g_ImagesEx1[sa_j]);
	Images_t2* g_Images2 = &(g_ImagesEx2[sa_j]);
	uint8_t smoothIndex;
	uint8_t smoothIndexMax;
	uint16_t coefEnd;
	uint16_t coefStart;
	uint16_t si,si2;
	uint8_t c;

	si = ((uint16_t)soundBeatIndex) * 2;
	si2 = 256 - si;
	si*= 255;

	if ( s_BeatOn == false )
	{
		si = 0;
		si2 = 256;
	}

	coefEnd = (((uint16_t)getSwitchIgnored()) << 8 ) / ON_OFF_IGNORE_PERIOD;

	if ( coefEnd )
	{
		coefStart = (1UL << 8) - coefEnd;
		asm ("cli");

		g_Images2->current[sa_i].r = 0;
		g_Images2->current[sa_i].g = coefEnd;
		g_Images2->current[sa_i].b = 0;

		asm ("sei");
	}
	else if ( s_mode == LMMODE_HOST )
	{
		smoothIndex = g_smoothIndex[sa_j];
		smoothIndexMax = g_Settings.smoothSlowdown;
		if ( smoothIndexMax == 0 )
		{
			smoothIndex = 1;
			smoothIndexMax = 1;
		}

		coefEnd = ((uint16_t)smoothIndex << 8) / smoothIndexMax;
		coefStart = (1UL << 8) - coefEnd;

		{
			asm ("cli");
			c = ( coefStart * g_Images1->start[sa_i].r + coefEnd   * g_Images1->end[sa_i].r ) >> 8;
			g_Images2->current[sa_i].r = ( si + si2 * c ) >> 8;

			c = ( coefStart * g_Images1->start[sa_i].g + coefEnd   * g_Images1->end[sa_i].g ) >> 8;
			g_Images2->current[sa_i].g = ( si + si2 * c ) >> 8;

			c = ( coefStart * g_Images1->start[sa_i].b + coefEnd   * g_Images1->end[sa_i].b ) >> 8;
			g_Images2->current[sa_i].b = ( si + si2 * c ) >> 8;

			asm ("sei");
		}
	}
	else
	{
		coefEnd = ml_smoothindex;
		coefStart = (1UL << 8) - coefEnd;

		//moodlamp
		asm ("cli");
		c = ( coefStart * ml_start.r + coefEnd   * ml_end.r ) >> 8;
		g_Images2->current[sa_i].r = ( si + si2 * c ) >> 8;

		c = ( coefStart * ml_start.g + coefEnd   * ml_end.g ) >> 8;
		g_Images2->current[sa_i].g = ( si + si2 * c ) >> 8;

		c = ( coefStart * ml_start.b + coefEnd   * ml_end.b ) >> 8;
		g_Images2->current[sa_i].b = ( si + si2 * c ) >> 8;
		asm ("sei");
	}

	sa_i++;

	if ( sa_i == LEDS_COUNT )
	{
		sa_i = 0;

		sa_j++;
		if ( sa_j == LIGHTPACKS_COUNT )
		{
			sa_j = 0;
		}
	}

}

void LedManager_UpdateColors(void)
{
//    EvalCurrentImage_SmoothlyAlg();

	if ( soundBeatIndex )
	{
		soundBeatIndex--;
	}

	for ( uint8_t j = 0; j < LIGHTPACKS_COUNT; j++ )
	{
		uint8_t smoothIndex = g_smoothIndex[j];

		if ((smoothIndex >= g_Settings.smoothSlowdown) )
		{
			g_smoothIndex[j] = g_Settings.smoothSlowdown;
		}
		else
		{
			g_smoothIndex[j]++;
		}
	}
/*
    if (g_Settings.isSmoothEnabled)
    	LedDriver_Update(g_Images.current);
    else
    	LedDriver_Update(g_Images.end);
*/
   LedDriver_UpdateCurrent();

   ml_smoothindex2++;
   if ( ml_smoothindex2 == 2 )
   {
	   ml_smoothindex2 = 0;
	   ml_smoothindex++;
	   if ( ml_smoothindex == 0 )
	   {
		   moodLamp_next();
	   }
   }

}


#elif (LIGHTPACK_HW == 5 || LIGHTPACK_HW == 4)


static inline void _StartConstantTime(void)
{
    TCNT1 = 0;
}

static inline void _EndConstantTime(const uint8_t time)
{
    while(TCNT1 < time * 256UL) { }
}

void EvalCurrentImage_SmoothlyAlg(void)
{
    for (uint8_t i = 0; i < LEDS_COUNT; i++)
    {
        if (g_Images.smoothIndex[i] >= g_Settings.smoothSlowdown)
        {
            // Smooth change colors complete, rewrite start image
            g_Images.current[i].r = g_Images.start[i].r = g_Images.end[i].r;
            g_Images.current[i].g = g_Images.start[i].g = g_Images.end[i].g;
            g_Images.current[i].b = g_Images.start[i].b = g_Images.end[i].b;

        } else {
            uint16_t coefEnd = ((uint16_t)g_Images.smoothIndex[i] << 8) / g_Settings.smoothSlowdown;
            uint16_t coefStart = (1UL << 8) - coefEnd;

            g_Images.current[i].r = (
                    coefStart * g_Images.start[i].r +
                    coefEnd   * g_Images.end  [i].r) >> 8;

            g_Images.current[i].g = (
                    coefStart * g_Images.start[i].g +
                    coefEnd   * g_Images.end  [i].g) >> 8;

            g_Images.current[i].b = (
                    coefStart * g_Images.start[i].b +
                    coefEnd   * g_Images.end  [i].b) >> 8;

            g_Images.smoothIndex[i]++;
        }
    }
}

static inline void _PulseWidthModulation(void)
{
    static uint8_t s_pwmIndex = 0; // index of current PWM level

    if (s_pwmIndex == g_Settings.maxPwmValue)
    {
        s_pwmIndex = 0;

        _StartConstantTime();

        // Switch OFF LEDs on time sets in g_Settings.brightness
        LedDriver_OffLeds();

        // Also eval current image
        if (g_Settings.isSmoothEnabled)
        {
            EvalCurrentImage_SmoothlyAlg();
        }

        _EndConstantTime(g_Settings.brightness);
    }


    if (g_Settings.isSmoothEnabled)
    {
        LedDriver_UpdatePWM(g_Images.current, s_pwmIndex);
    } else {
        LedDriver_UpdatePWM(g_Images.end, s_pwmIndex);
    }

    s_pwmIndex++;

    // Clear timer counter
    TCNT1 = 0x0000;
}

void LedManager_UpdateColors(void)
{
	_PulseWidthModulation();
}


#endif

void LedManager_FlashLedsRed(void)
{
	uint8_t i;

	for ( i = 0; i < 4; i++ )
	{
		LedManager_FillImages( 255, 0, 0 );
		LedDriver_UpdateCurrent();
		wdt_reset();
		_delay_ms( 200 );
		wdt_reset();
		_delay_ms( 200 );
		wdt_reset();
		LedManager_FillImages( 0, 0, 0 );
		LedDriver_UpdateCurrent();
		wdt_reset();
		_delay_ms( 200 );
		wdt_reset();
		_delay_ms( 200 );
		wdt_reset();
	}

}

void LedManager_LitLeds( uint8_t count )
{
	LedDriver_LitLeds( count );
}

void LedManager_setMode( uint8_t mode )
{
	if ( s_mode == mode )
	{
		s_BeatOn = !s_BeatOn;
	}
	s_mode = mode;
	moodLamp_next();
}

bool LEDManager_acceptsUpdates(void)
{
	return ( s_mode == LMMODE_HOST) && ( getSwitchIgnored() == 0 );
}

//===============================
//===============================
void LedManager_adjustGamma( int8_t _delta )
{
	_delta += (int8_t)getGammaIndex();

	if ( _delta <= 0 )
	{
		setGammaIndex( 0 );
	}
	else if ( _delta >= 4 )
	{
		setGammaIndex( 4 );
	}
	else
	{
		setGammaIndex( _delta );
	}
}

//===============================
//===============================
void LedManager_Beat()
{
	soundBeatIndex = 16;
}
