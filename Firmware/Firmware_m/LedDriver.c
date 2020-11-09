/*
 * LedDriver.c
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

#include <avr/io.h>
#include "iodefs.h"
#include "Lightpack.h"
#include "LedDriver.h"

#include "ws2811.h"

#include "version.h"

#include "LedManager.h"

#if ( RESAMPLE == 1 )
RGB_t1 resampled[TOTALRZONES];
#endif


#if (LIGHTPACK_HW >= 6)

/*
 *  Hardware 6.x
 */

//#define LATCH_PIN   (B, 0)
//#define SCK_PIN     (B, 1)
//#define MOSI_PIN    (B, 2)

static const uint8_t LedsNumberForOneDriver = 5;

/*
static inline void _SPI_Write12(uint16_t byte)
{
    CLR(MOSI_PIN);

    // 12-bits
    for (uint16_t bit = 0x0800; bit != 0; bit >>= 1)
    {
        if (byte & bit)
        {
            SET(MOSI_PIN);
        } else {
            CLR(MOSI_PIN);
        }

        SET(SCK_PIN);
        CLR(SCK_PIN);
    }
}

static inline void _LedDriver_LatchPulse(void)
{
    SET(LATCH_PIN);
    CLR(LATCH_PIN);
	TOGGLE(LATCH_PIN );
}

*/
void LedDriver_Init(void)
{
    //OUTPUT(SCK_PIN);
    //OUTPUT(MOSI_PIN);
   // OUTPUT(LATCH_PIN);

    //CLR(LATCH_PIN);
    //CLR(SCK_PIN);
   // CLR(MOSI_PIN);

    LedDriver_OffLeds();
}

DEFINE_WS2811_FN(WS2811RGB, PORTB, 0)

//RGB_t1 a[LIGHTPACKS_COUNT*10];

#if ( RESAMPLE == 1 )

void resample( uint8_t lz1, uint8_t lz2, uint8_t rz1, uint8_t rz2 )
{
	if ( lz2 < lz1 )
	{
		lz2 += LIGHTPACKS_COUNT * 10;
	}

	uint8_t c1 = lz2 - lz1;
	uint8_t c2 = rz2 - rz1 + 1;

	uint8_t rd = (((uint16_t)c1) << 8) / c2;

	uint16_t ri = rd >> 1;  //middle of strip zone
	//li1 + ri >> 8 - lightpack zone index where middle of strip zone is,
	//ri & 0xff - position inside that lightpack zone 0..255

	RGB_t1* a = (RGB_t1*)g_ImagesEx2;

	const uint8_t* gt = LEDManager_getCurrentGammaTable();

	for ( uint8_t i = rz1; i <= rz2; i++ )
	{
		//take  ( ri & 0xff ) from (li1 + ri>>8 + 1) zone,
		//the rest from (li1 + ri>>8)  zone

		uint8_t t = ri & 0xff;
		uint8_t t1 = 256 - t;  //not a bug. Due to nature of algorithm, t1 is never =256.

		uint8_t li1 = ( ri >> 8 ) + lz1;

		uint8_t li2 = li1 + 1;
		if ( li2 > ( LIGHTPACKS_COUNT * 10 ) )
		{
			li2 -= LIGHTPACKS_COUNT * 10;
		}

		resampled[ i ].r = pgm_read_byte( &gt[ ( ((uint16_t)a[ li2 ].r) * t + ((uint16_t)a[ li1 ].r) * t1 ) >> 8 ]);
		resampled[ i ].g = pgm_read_byte( &gt[ ( ((uint16_t)a[ li2 ].g) * t + ((uint16_t)a[ li1 ].g) * t1 ) >> 8 ]);
		resampled[ i ].b = pgm_read_byte( &gt[ ( ((uint16_t)a[ li2 ].b) * t + ((uint16_t)a[ li1 ].b) * t1 ) >> 8 ]);

		ri += rd;
	}

}

#endif

void LedDriver_UpdateCurrent(void)
{
#if ( RESAMPLE == 1 )

	resample( LZ_LEFT1, LZ_LEFT2, RZ_LEFT1, RZ_LEFT2 );
	resample( LZ_RIGHT1, LZ_RIGHT2, RZ_RIGHT1, RZ_RIGHT2 );
	resample( LZ_TOP1, LZ_TOP2, RZ_TOP1, RZ_TOP2 );
	resample( LZ_BOTTOM1, LZ_BOTTOM2, RZ_BOTTOM1, RZ_BOTTOM2 );

    WS2811RGB(resampled, TOTALRZONES);

#else

	RGB_t1* a = (RGB_t1*)g_ImagesEx2;
    WS2811RGB(a, LIGHTPACKS_COUNT*10);

#endif

}

void LedDriver_LitLeds( uint8_t count )
{
#if ( RESAMPLE == 1 )

	for (uint8_t i = 0; i < count; i++)
	{
		resampled[ i ].r = 255;
		resampled[ i ].g = 255;
		resampled[ i ].b = 255;
	}

	for (uint8_t i = count; i < TOTALRZONES; i++)
	{
		resampled[ i ].r = 0;
		resampled[ i ].g = 0;
		resampled[ i ].b = 0;
	}

    WS2811RGB(resampled, TOTALRZONES);

#else
	Images_t2* g_Images2 = &(g_ImagesEx2[0]);
	for (uint8_t i = 0; i < count; i++)
	{
		g_Images2->current[i].r = 255;
		g_Images2->current[i].g = 255;
		g_Images2->current[i].b = 255;
	}

	for (uint8_t i = count; i < LIGHTPACKS_COUNT*10; i++)
	{
		g_Images2->current[i].r = 0;
		g_Images2->current[i].g = 0;
		g_Images2->current[i].b = 0;
	}

	RGB_t1* a = (RGB_t1*)g_ImagesEx2;
    WS2811RGB(a, LIGHTPACKS_COUNT*10);

#endif

}

void LedDriver_Update(const RGB_t imageFrame[LEDS_COUNT])
{
	/*
    // ...
    // LedDriver2
    //      10     9     8     7     6
    // 0 B G R B G R B G R B G R B G R
    // LedDriver1
    //       5     4     3     2     1
    // 0 B G R B G R B G R B G R B G R

    _SPI_Write12(0);

    for (uint8_t i = LedsNumberForOneDriver; i < LEDS_COUNT; i++)
    {
        _SPI_Write12(imageFrame[i].b);
        _SPI_Write12(imageFrame[i].g);
        _SPI_Write12(imageFrame[i].r);
    }

    _SPI_Write12(0);


    for (uint8_t i = 0; i < LedsNumberForOneDriver; i++)
    {
        _SPI_Write12(imageFrame[i].b);
        _SPI_Write12(imageFrame[i].g);
        _SPI_Write12(imageFrame[i].r);
    }

    _LedDriver_LatchPulse();
    */
}

void LedDriver_OffLeds(void)
{
	/*
	RGB_t1* a = (RGB_t1*)g_ImagesEx2;

	for ( uint8_t i = 0; i < LIGHTPACKS_COUNT*10; i++ )
	{
		a->r = 0;
		a->b = 0;
		a->g = 0;
		a++;
	}

    WS2811RGB(a, LIGHTPACKS_COUNT*10);
*/
	/*
    for (uint8_t i = 0; i < 16; i++)
        _SPI_Write12(0x0000);

    _LedDriver_LatchPulse();
    */
}

#elif (LIGHTPACK_HW == 5)

/*
 *  Hardware 5.x
 */

#define LATCH_PIN   (B, 0)
#define SCK_PIN     (B, 1)
#define MOSI_PIN    (B, 2)

static const uint8_t LedsNumberForOneDriver = 5;

static inline void _SPI_Write8(const uint8_t byte)
{
    SPDR = byte;
    while ((SPSR & (1 << SPIF)) == false) { }
}

static inline void _SPI_Write16(const uint16_t word)
{
    _SPI_Write8((word >> 8) & 0xff);
    _SPI_Write8(word & 0xff);
}

static inline void _LedDriver_LatchPulse(void)
{
    SET(LATCH_PIN);
    CLR(LATCH_PIN);
}

static inline void _LedDriver_UpdateLedsPWM(
        const uint8_t startIndex, const uint8_t endIndex,
        const RGB_t imageFrame[LEDS_COUNT],
        const uint8_t pwmIndex )
{
    uint16_t sendme = 0x0000;
    uint16_t bit  = 1;

    for (uint8_t i = startIndex; i < endIndex; i++)
    {
        if (imageFrame[i].r > pwmIndex)
            sendme |= bit;
        bit <<= 1;

        if (imageFrame[i].g > pwmIndex)
            sendme |= bit;
        bit <<= 1;

        if (imageFrame[i].b > pwmIndex)
            sendme |= bit;
        bit <<= 1;
    }

    _SPI_Write16(sendme);
}

void LedDriver_Init(void)
{
    OUTPUT(SCK_PIN);
    OUTPUT(MOSI_PIN);
    OUTPUT(LATCH_PIN);

    CLR(LATCH_PIN);

    // Setup SPI Master with max SPI clock speed (F_CPU / 2)
    SPSR = (1 << SPI2X);
    SPCR = (1 << SPE) | (1 << MSTR);

    LedDriver_OffLeds();
}

void LedDriver_UpdatePWM(const RGB_t imageFrame[LEDS_COUNT], const uint8_t pwmIndex)
{
    // ...
    // LedDriver2
    //      10     9     8     7     6
    // 0 B G R B G R B G R B G R B G R
    // LedDriver1
    //       5     4     3     2     1
    // 0 B G R B G R B G R B G R B G R

    //    for (int8_t i = LEDS_COUNT - LedsNumberForOneDriver; i >= 0; i -= LedsNumberForOneDriver)
    //    {
    //        _LedDriver_UpdateLeds(i, i + LedsNumberForOneDriver,
    //                LevelsForPWM, pwmIndex);
    //    }

    _LedDriver_UpdateLedsPWM(LedsNumberForOneDriver, LEDS_COUNT, imageFrame, pwmIndex);
    _LedDriver_UpdateLedsPWM(0, LedsNumberForOneDriver, imageFrame, pwmIndex);

    _LedDriver_LatchPulse();
}

void LedDriver_OffLeds(void)
{
    _SPI_Write16(0x0000);
    _SPI_Write16(0x0000);
    _LedDriver_LatchPulse();
}

#elif (LIGHTPACK_HW == 4)

/*
 *  Hardware 4.x
 */

#define LATCH_1	    (B, 4)
#define CLK_1	    (B, 5)
#define DATA_1 	    (B, 6)

#define LATCH_2	    (B, 0)
#define CLK_2	    (B, 1)
#define DATA_2 	    (B, 2)

enum LEDS { LED1, LED2, LED3, LED4, LED5, LED6, LED7, LED8 };

static inline void _LedDrivers_LatchPulse(void)
{
    SET(LATCH_1);
    SET(LATCH_2);

    CLR(LATCH_1);
    CLR(LATCH_2);
}

static inline void _LedDrivers_ClkUp(void)
{
    SET(CLK_1);
    SET(CLK_2);
}

static inline void _LedDrivers_ClkDown(void)
{
    CLR(CLK_1);
    CLR(CLK_2);
}

// Parallel serial output of the one led for each driver, Pulse Width Modulation
static inline void _LedDrivers_PSO_PWM(
        const RGB_t imageFrame[LEDS_COUNT],
        const uint8_t pwmIndex,
        const uint8_t ledIndex1,
        const uint8_t ledIndex2 )
{
    // LedDriver connection: NC B G R
    _LedDrivers_ClkDown();
    CLR(DATA_1);
    CLR(DATA_2);
    _LedDrivers_ClkUp();

    _LedDrivers_ClkDown();
    if (imageFrame[ledIndex1].b > pwmIndex) SET(DATA_1) else CLR(DATA_1);
    if (imageFrame[ledIndex2].b > pwmIndex) SET(DATA_2) else CLR(DATA_2);
    _LedDrivers_ClkUp();

    _LedDrivers_ClkDown();
    if (imageFrame[ledIndex1].g > pwmIndex) SET(DATA_1) else CLR(DATA_1);
    if (imageFrame[ledIndex2].g > pwmIndex) SET(DATA_2) else CLR(DATA_2);
    _LedDrivers_ClkUp();

    _LedDrivers_ClkDown();
    if (imageFrame[ledIndex1].r > pwmIndex) SET(DATA_1) else CLR(DATA_1);
    if (imageFrame[ledIndex2].r > pwmIndex) SET(DATA_2) else CLR(DATA_2);
    _LedDrivers_ClkUp();

}

void LedDriver_Init(void)
{
    CLR(CLK_1);
    CLR(CLK_2);

    CLR(LATCH_1);
    CLR(LATCH_2);

    CLR(DATA_1);
    CLR(DATA_2);

    OUTPUT(CLK_1);
    OUTPUT(CLK_2);

    OUTPUT(LATCH_1);
    OUTPUT(LATCH_2);

    OUTPUT(DATA_1);
    OUTPUT(DATA_2);
}

void LedDriver_UpdatePWM(const RGB_t imageFrame[LEDS_COUNT], const uint8_t pwmIndex)
{
    _LedDrivers_PSO_PWM(imageFrame, pwmIndex, LED4, LED8);
    _LedDrivers_PSO_PWM(imageFrame, pwmIndex, LED3, LED7);
    _LedDrivers_PSO_PWM(imageFrame, pwmIndex, LED2, LED6);
    _LedDrivers_PSO_PWM(imageFrame, pwmIndex, LED1, LED5);

    _LedDrivers_LatchPulse();
}

void LedDriver_OffLeds(void)
{
    CLR(DATA_1);
    CLR(DATA_2);

    for (uint8_t i = 0; i < 16; i++)
    {
        _LedDrivers_ClkDown();
        _LedDrivers_ClkUp();
    }

    _LedDrivers_LatchPulse();
}

#else
#	error "LIGHTPACK_HW must be defined in '../CommonHeaders/LIGHTPACK_HW.h' to major number of the hardware revision"
#endif /* (LIGHTPACK_HW switch) */

