/*
             LUFA Library
     Copyright (C) Dean Camera, 2013.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2013  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Header file for Descriptors.c.
 */

#ifndef _DESCRIPTORS_H_
#define _DESCRIPTORS_H_

	/* Includes: */
		#include <avr/pgmspace.h>

		#include <LUFA/Drivers/USB/USB.h>
		#include "../CommonHeaders/USB_ID.h"
		#include "datatypes.h"
		
	/* Type Defines: */
		/** Type define for the device configuration descriptor structure. This must be defined in the
		 *  application code, as the configuration descriptor contains several sub-descriptors which
		 *  vary between devices, and which describe the device's usage to the host.
		 */
		typedef struct
		{
			USB_Descriptor_Configuration_Header_t Config;

			// Generic HID Interface
			USB_Descriptor_Interface_t            HID_Interface;
			USB_HID_Descriptor_HID_t              HID_GenericHID;
		        USB_Descriptor_Endpoint_t             HID_ReportINEndpoint;

#if (LIGHTPACKS_COUNT >= 2)

				USB_Descriptor_Interface_t            HID_Interface1;
				USB_HID_Descriptor_HID_t              HID_GenericHID1;
			        USB_Descriptor_Endpoint_t             HID_ReportINEndpoint1;
#endif
#if (LIGHTPACKS_COUNT >= 3)

					USB_Descriptor_Interface_t            HID_Interface2;
					USB_HID_Descriptor_HID_t              HID_GenericHID2;
				        USB_Descriptor_Endpoint_t             HID_ReportINEndpoint2;
#endif
		} USB_Descriptor_Configuration_t;

	/* Macros: */
		/** Endpoint address of the Generic HID reporting IN endpoint. */
		#define GENERIC_IN_EPADDR1         (ENDPOINT_DIR_IN | 1)
		#define GENERIC_IN_EPADDR2         (ENDPOINT_DIR_IN | 2)
		#define GENERIC_IN_EPADDR3         (ENDPOINT_DIR_IN | 3)

		/** Size in bytes of the Generic HID reporting endpoint. */
		#define GENERIC_EPSIZE            8

		#define GENERIC_REPORT_SIZE       64

	/* Function Prototypes: */
		uint16_t CALLBACK_USB_GetDescriptor(const uint16_t wValue,
		                                    const uint8_t wIndex,
		                                    const void** const DescriptorAddress)
		                                    ATTR_WARN_UNUSED_RESULT ATTR_NON_NULL_PTR_ARG(3);

#endif
