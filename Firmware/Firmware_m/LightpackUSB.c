/*
 * LightpackUSB.c
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

#include "Lightpack.h"
#include "LightpackUSB.h"
#include "version.h"
#include "LedManager.h"

#include "../CommonHeaders/COMMANDS.h"
#include <LUFA/Drivers/USB/USB.h>

// Buffer to hold the previously generated HID report, for comparison purposes inside the HID class driver.
//uint8_t PrevHIDReportINBuffer0[GENERIC_EPSIZE];
//uint8_t PrevHIDReportINBuffer1[GENERIC_EPSIZE];
//uint8_t PrevHIDReportINBuffer2[GENERIC_EPSIZE];
//uint8_t PrevUsbLedState;

USB_ClassInfo_HID_Device_t Generic_HID_Interface0 =
{
        .Config =
        {
                .InterfaceNumber              = 0,
                .ReportINEndpoint =
                {
                    .Address                  = GENERIC_IN_EPADDR1,
                    .Size                     = GENERIC_EPSIZE,
                    .Banks                    = 1,
                },

                .PrevReportINBuffer           = NULL,
                .PrevReportINBufferSize       = GENERIC_EPSIZE
        },
};


USB_ClassInfo_HID_Device_t Generic_HID_Interface1 =
{
        .Config =
        {
                .InterfaceNumber              = 1,
                .ReportINEndpoint =
                {
                    .Address                  = GENERIC_IN_EPADDR2,
                    .Size                     = GENERIC_EPSIZE,
                    .Banks                    = 1,
                },

                .PrevReportINBuffer           = NULL,
                .PrevReportINBufferSize       = GENERIC_EPSIZE
        },
};

USB_ClassInfo_HID_Device_t Generic_HID_Interface2 =
{
        .Config =
        {
                .InterfaceNumber              = 2,
                .ReportINEndpoint =
                {
                    .Address                  = GENERIC_IN_EPADDR3,
                    .Size                     = GENERIC_EPSIZE,
                    .Banks                    = 1,
                },

                .PrevReportINBuffer           = NULL,
                .PrevReportINBufferSize       = GENERIC_EPSIZE
        },
};

void ProcessUsbTasks(void)
{
    HID_Device_USBTask(&Generic_HID_Interface0);
    HID_Device_USBTask(&Generic_HID_Interface1);
    HID_Device_USBTask(&Generic_HID_Interface2);
    USB_USBTask();
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
    bool ConfigSuccess = true;

    ConfigSuccess &= HID_Device_ConfigureEndpoints(&Generic_HID_Interface0);
    ConfigSuccess &= HID_Device_ConfigureEndpoints(&Generic_HID_Interface1);
    ConfigSuccess &= HID_Device_ConfigureEndpoints(&Generic_HID_Interface2);

    USB_Device_EnableSOFEvents();
}

/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame(void)
{
    HID_Device_MillisecondElapsed(&Generic_HID_Interface0);
    HID_Device_MillisecondElapsed(&Generic_HID_Interface1);
    HID_Device_MillisecondElapsed(&Generic_HID_Interface2);
}


bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                         uint8_t* const ReportID,
                                         const uint8_t ReportType,
                                         void* ReportData,
                                         uint16_t* const ReportSize)
{
    uint8_t *ReportData_u8 = (uint8_t *)ReportData;
    *ReportSize = GENERIC_EPSIZE;

    // Firmware version
    ReportData_u8[INDEX_FW_VER_MAJOR] = VERSION_OF_FIRMWARE_MAJOR;
    ReportData_u8[INDEX_FW_VER_MINOR] = VERSION_OF_FIRMWARE_MINOR;
    return true;
}

/** HID class driver callback function for the processing of HID reports from the host.
 *
 *  \param[in] HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in] ReportID    Report ID of the received report from the host
 *  \param[in] ReportType  The type of report that the host has sent, either HID_REPORT_ITEM_Out or HID_REPORT_ITEM_Feature
 *  \param[in] ReportData  Pointer to a buffer where the created report has been stored
 *  \param[in] ReportSize  Size in bytes of the received HID report
 */
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
                                          const uint8_t ReportID,
                                          const uint8_t ReportType,
                                          const void* ReportData,
                                          const uint16_t ReportSize)
{
    
    SET((D,5));

    uint8_t *ReportData_u8 = (uint8_t *)ReportData;

    uint8_t cmd = ReportData_u8[0]; //[0]; // command from enum COMMANDS{ ... };

    Images_t* g_Images1 = &g_ImagesEx1[HIDInterfaceInfo->Config.InterfaceNumber];
    Images_t2* g_Images2 = &g_ImagesEx2[HIDInterfaceInfo->Config.InterfaceNumber];
    //Images_t* g_Images = &(g_ImagesEx[0]);

    switch (cmd)
    {

    case CMD_UPDATE_LEDS:
    {

        _FlagSet(Flag_ChangingColors);

        uint8_t reportDataIndex = 1; // new data starts form ReportData_u8[1]

        if ( LEDManager_acceptsUpdates() )
        {
			for (uint8_t i = 0; i < LEDS_COUNT; i++)
			{

				g_Images1->start[i].r = g_Images2->current[i].r;
				g_Images1->start[i].g = g_Images2->current[i].g;
				g_Images1->start[i].b = g_Images2->current[i].b;

				g_Images1->end[i].r = ReportData_u8[reportDataIndex++];
				g_Images1->end[i].g = ReportData_u8[reportDataIndex++];
				g_Images1->end[i].b = ReportData_u8[reportDataIndex++];

				reportDataIndex+=3;
			}
        }
        g_smoothIndex[HIDInterfaceInfo->Config.InterfaceNumber] = 0;

        _FlagClear(Flag_ChangingColors);
        _FlagSet(Flag_HaveNewColors);

        break;
    }
    case CMD_OFF_ALL:

        _FlagSet(Flag_LedsOffAll);

        break;

    case CMD_SET_TIMER_OPTIONS:

    	/*
        g_Settings.timerOutputCompareRegValue =
                (((uint16_t)ReportData_u8[2] << 8) | ReportData_u8[1]);
        */

        _FlagSet(Flag_TimerOptionsChanged);

        break;

    case CMD_SET_PWM_LEVEL_MAX_VALUE:

        //g_Settings.maxPwmValue = ReportData_u8[1];

        break;

    case CMD_SET_SMOOTH_SLOWDOWN:

        //g_Settings.isSmoothEnabled = ReportData_u8[1];
        g_Settings.smoothSlowdown  = ReportData_u8[1]/4; /* not a bug */

        break;

    case CMD_SET_BRIGHTNESS:

        //g_Settings.brightness = ReportData_u8[1];

        break;

    case CMD_NOP:
        break;

    }
    CLR((D,5));

}

void EVENT_USB_Device_ControlRequest(void)
{
	HID_Device_ProcessControlRequest(&Generic_HID_Interface0);
	HID_Device_ProcessControlRequest(&Generic_HID_Interface1);
	HID_Device_ProcessControlRequest(&Generic_HID_Interface2);
}
