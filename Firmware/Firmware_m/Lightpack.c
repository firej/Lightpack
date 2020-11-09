/*
 * Lightpack.c
 *
 *  Created on: 11.01.2011
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

#include "Lightpack.h"
#include "LedDriver.h"
#include "LedManager.h"
#include "LightpackUSB.h"
#include <avr/sleep.h>


volatile uint8_t g_Flags = 0;


Images_t g_ImagesEx1[LIGHTPACKS_COUNT];
Images_t2 g_ImagesEx2[LIGHTPACKS_COUNT];

uint8_t g_smoothIndex[LIGHTPACKS_COUNT];

Settings_t g_Settings =
{
        //.isSmoothEnabled = true,

        // Number of intermediate colors between old and new
        .smoothSlowdown = 100/4,

        //.brightness = 50,

        // Maximum number of different colors for each channel (Red, Green and Blue)
        //.maxPwmValue = 128,

        // Timer OCR value
        //.timerOutputCompareRegValue = 20000,
        //2000//with div8 it's 100Hz update
};

/*
 *  Interrupts of the timer that generates PWM
 */

//incremented with 100hz
//volatile uint16_t s_timer100;

//ifnore exthw on/off and 12V on/of until!=0
//decremented with 100hz
volatile uint8_t s_switchIgnore = 0;


//===========================================
//===========================================
ISR( TIMER1_COMPA_vect )
{
    SET((D,6));
    //s_timer100++;

    if ( s_switchIgnore )
    {
    	s_switchIgnore--;
    }

    TCNT1 = 0x0000;

    LedManager_UpdateColors();

    // Clear timer interrupt flag
    TIFR1 = _BV(OCF1A);

    CLR((D,6));
}

//=======================================
//=======================================
ISR( INT1_vect )
{
	LedManager_Beat();
}

//=======================================
//=======================================
// Watchdog interrupt
ISR ( WDT_vect )
{
    _BlinkUsbLed(3, 40);
    for(;;);
}

/*
//=======================================
//=======================================
uint16_t getTimer100(void )
{
	uint16_t res;
	asm ("cli");
	res = s_timer100;
	asm ("sei");

	return res;
}
*/

//=======================================
//=======================================
static inline void Timer_Init(void)
{
    TCCR1A = 0x00;
    TCCR1C = 0x00;
    TCCR1B = 0x00;
//    TCCR0A = 0x00;
//    TCCR0B = 0x00;

    // Setup default value
 //   OCR1A = g_Settings.timerOutputCompareRegValue;
      OCR1A = 20000;

    TIMSK1 = _BV(OCIE1A); // Main timer
//    TIMSK0 = _BV(TOIE0); // Usb led timer

    // Start timer
    //TCCR1B = _BV(CS10); // div1
    TCCR1B = _BV(CS11); // div8


    //    TCCR0B = _BV(CS02); // div by 8

    TCNT1 = 0x0000;
//    TCNT0 = 0x0000;


}

//=======================================
//=======================================
static inline void SetupHardware(void)
{
    /* Disable clock division */
    clock_prescale_set(clock_div_1);

    /* Hardware Initialization */
    DDRB  = 0b11101111;
    PORTB = 0b00000000;

    DDRC  = 0b11111100;
    PORTC = 0b00000000;

    DDRD  = 0b11101000;
    PORTD = 0b00000101;

    //D0 - setup button
    //D2 - RXD1

//    OUTPUT(USBLED);
//    CLR(USBLED);

//    set_sleep_mode(SLEEP_MODE_IDLE);

    // External Interrupt(s) initialization
    // INT0: Off
    // INT1: On
    // INT1 Mode: Rising Edge
    // INT2: Off
    // INT3: Off
    // INT4: Off
    // INT5: Off
    // INT6: Off
    // INT7: Off
    EICRA=0x0C;
    EICRB=0x00;
    EIMSK=0x02;
    EIFR=0x02;
    // PCINT0 interrupt: Off
    // PCINT1 interrupt: Off
    // PCINT2 interrupt: Off
    // PCINT3 interrupt: Off
    // PCINT4 interrupt: Off
    // PCINT5 interrupt: Off
    // PCINT6 interrupt: Off
    // PCINT7 interrupt: Off
    PCMSK0=0x00;
    // PCINT8 interrupt: Off
    // PCINT9 interrupt: Off
    // PCINT10 interrupt: Off
    // PCINT11 interrupt: Off
    // PCINT12 interrupt: Off
    PCMSK1=0x00;
    PCICR=0x00;

}

//=======================================
//=======================================
static inline void enableWatchdog(void)
{
    wdt_reset();
    // Watchdog configuration: interrupt after 250ms and reset after 500ms;
    wdt_enable(WDTO_250MS);
    // Enable watchdog interrupt
    WDTCSR |= _BV(WDIE);
}

//=======================================
//=======================================
static inline void _ProcessFlags(void)
{
    /* if (_FlagProcess(Flag_HaveNewColors)) */

    if (_FlagProcess(Flag_LedsOffAll))
        LedManager_FillImages(0, 0, 0);


    /*
    if (_FlagProcess(Flag_TimerOptionsChanged))
    {
        // Pause timer
        TIMSK1 &= (uint8_t)~_BV(OCIE1A);

        OCR1A = g_Settings.timerOutputCompareRegValue;

        // Restart timer
        TCNT1 = 0x0000;
        TIMSK1 = _BV(OCIE1A);
    }
    */
}

//=======================================
//=======================================
void switchOnOff(void )
{
	if ( s_switchIgnore )
	{
		return;
	}

	s_switchIgnore = ON_OFF_IGNORE_PERIOD;

	if ( ISSET((B,1)) )
	{
		CLR((B,1));
	}
	else
	{
		SET((B,1));
	}
}

//=======================================
//=======================================
void switchExtOnOff(void)
{
	if ( s_switchIgnore )
	{
		return;
	}

	s_switchIgnore = ON_OFF_IGNORE_PERIOD;

	if ( ISSET((B,2)) )
	{
		CLR((B,2));
	}
	else
	{
		SET((B,2));
	}
}

//=======================================
//=======================================
uint8_t getSwitchIgnored(void)
{
	return s_switchIgnore;
}

//=======================================
//=======================================
/*
 *  Main program entry point
 */
__attribute__((OS_main)) int main(void)
{
    enableWatchdog();
    SetupHardware();
    _WaveSwitchOnUsbLed(100, 100);
    // Led driver ports initialization
    LedDriver_Init();

    // Initialize timer for update LedDriver-s
    Timer_Init();

    // Initialize USB
    USB_Init();

    sei();

    for (;;)
    {
        //SET((D,4));
        //CLR((D,4));

        wdt_reset();
        ProcessUsbTasks();
        _ProcessFlags();
//	sleep_mode();
        EvalCurrentImage_SmoothlyAlgIteration();


        if ( !TST((D,0)) )
        {
            TIMSK1 &= (uint8_t)~_BV(OCIE1A);
            TIMSK1 = _BV(OCIE1A);
        }

    }

    // Avoid annoying warning
    return 0;
}

