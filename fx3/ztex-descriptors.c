/*%
 ZTEX Firmware Kit for EZ-USB FX3 Microcontrollers
 Copyright (C) 2009-2017 ZTEX GmbH.
 http://www.ztex.de

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License version 3 as
 published by the Free Software Foundation.

 This program is distributed in the hope that it will be useful, but
 WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, see http://www.gnu.org/licenses/.
 %*/
/*
 Descriptor definitions.
 */
#ifndef _ZTEX_CONF_H_
#error "Illegal use of `ztex-descriptors.c'. This file must be included in the main firmware source after including `ztex-conf.h' and after the configuration section"
#endif
#ifndef _ZTEX_DESCRIPTORS_C_
#define _ZTEX_DESCRIPTORS_C_

#define W2B(a) (a) & 255, (a) >> 8
#define DIR_IN 128
#define DIR_OUT 0
#define TYPE_ISO 1
#define TYPE_BULK 2
#define TYPE_INT 3


#include "uvc.h"

// device descriptor for USB 3.0
uint8_t ztex_usb3_device_descriptor[] __attribute__ ((aligned (32))) = { 18, // 0, descriptor size
		CY_U3P_USB_DEVICE_DESCR,        		// 1, Device descriptor type
		0x00, 0x03,                      		// 2, USB 3.0
		0xEF,                           		// 4, Device class
		0x02,                          		// 5, Device sub-class
		0x01,                           		// 6, Device protocol
		0x09,                           	// 7, Max packet size for EP0 : 2^9
		W2B(ZTEX_USB_VENDOR_ID),	    		// 8, Vendor ID
		W2B(ZTEX_USB_PRODUCT_ID),			// 10, Product ID
		0x00, 0x00,                      		// 12, Device release number
		0x01,                           		// 14, Manufacture string index
		0x02,                           		// 15, Product string index
		0x03,                           	// 16, Serial number string index
		0x01                            		// 17, Number of configurations
		};

// device descriptor for USB 2.0
uint8_t ztex_usb2_device_descriptor[] __attribute__ ((aligned (32))) = { 18, // 0, Descriptor size
		CY_U3P_USB_DEVICE_DESCR,        		// 1, Device descriptor type
		0x00, 0x02,                      		// 2, USB 2.00
		0xEF,                           		// 4, Device class
		0x02,                           		// 5, Device sub-class
		0x01,                           		// 6, Device protocol
		0x40,                           // 7, Max packet size for EP0 : 64 bytes
		W2B(ZTEX_USB_VENDOR_ID),	    		// 8, Vendor ID
		W2B(ZTEX_USB_PRODUCT_ID),			// 10, Product ID
		0x00, 0x00,                      		// 12, Device release number
		0x01,                           		// 14, Manufacture string index
		0x02,                           		// 15, Product string index
		0x03,                           	// 16, Serial number string index
		0x01                            		// 17, Number of configurations
		};

// Binary device object store descriptor
const uint8_t ztex_bos_descriptor[] __attribute__ ((aligned (32))) = { 5, // 0, Descriptor size
		CY_U3P_BOS_DESCR,              		// 1, Device descriptor type
		22, 0, 	         // 2, Length of this descriptor and all sub descriptors
		0x02,                      // 4, Number of device capability descriptors

		// USB 2.0 extension
		7,// 0, Descriptor size
		CY_U3P_DEVICE_CAPB_DESCR,       // 1, Device capability type descriptor
		CY_U3P_USB2_EXTN_CAPB_TYPE,     // 2, USB 2.0 extension capability type
		0x1E, 0x64, 0x00, 0x00, // 3, Supported device level features: LPM support

		// SuperSpeed device capability
		10,// 0, Descriptor size
		CY_U3P_DEVICE_CAPB_DESCR,       // 1, Device capability type descriptor
		CY_U3P_SS_USB_CAPB_TYPE,        // 2, SuperSpeed device capability type
		0x00,                           // 3, Supported device level features
		0x0E, 0x00,         // 4, Speeds supported by the device : SS, HS and FS
		0x03,                           		// 6, Functionality support
		0x00,                           		// 7, U1 Device Exit latency
		0x00, 0x00                       		// 8, U2 Device Exit latency
		};

// Standard device qualifier descriptor
const uint8_t ztex_device_qualifier_descriptor[] __attribute__ ((aligned (32)))
		= { 10,                           		// 0, Descriptor size
				CY_U3P_USB_DEVQUAL_DESCR, // 1, Device qualifier descriptor type
				0x00, 0x02,                      		// 2, USB 2.0
				0xEF,                           		// 4, Device class
				0x02,                           		// 5, Device sub-class
				0x01,                           		// 6, Device protocol
				0x40,                   // 7, Max packet size for EP0 : 64 bytes
				0x01,                           // 8, Number of configurations
				0x00                            		// 9, Reserved
		};

/* super speed configuration descriptor */
const uint8_t ztex_usb3_config_descriptor[] __attribute__ ((aligned (32))) = {
		/* Configuration descriptor */
		        9, /* 0, Descriptor size */
		        CY_U3P_USB_CONFIG_DESCR, /* 1, Configuration descriptor type */
				0x12, 0x01, /* Length of this descriptor and all sub descriptors */
				0x04, /* Number of interfaces */
				0x01, /* 5, Configuration number */
				0x00, /* 6, Configuration string index */
				0x80, /* 7, attributes: bus */
				0x32, /* 8, Max power consumption of device (in 8mA units) : 400mA */

				/* Interface Descriptors */

				0x09, /* 0, Descriptor size */
				CY_U3P_USB_INTRFC_DESCR, /* 1, Interface Descriptor type */
				0x03, /* 2, Interface number */
				0x00, /* 3, Alternate setting number */
				0x02, /* 4, Number of end points */
				0xFF, /* 5, Interface class */
				0x00, /* 5, Interface sub class */
				0x00, /* 6, Interface protocol code  */
				0x04, /* 7, Interface descriptor string index */

				/* Producer */
				0x07, /* 0, Descriptor size */
				CY_U3P_USB_ENDPNT_DESCR, /* 1, Endpoint descriptor type */
				0x02, /* 2, Endpoint number + direction */
				CY_U3P_USB_EP_BULK, /* 3, endpoint type */
				0x00, 0x04, /* 4, Max packet size */
				0x00, /* 6, Service interval */

				/* Super speed endpoint companion descriptor for producer EP */
				0x06, /* 0, Descriptor size */
				CY_U3P_SS_EP_COMPN_DESCR, /* 1, SS endpoint companion descriptor type  */
				0x0F, /* 2, bursts*/
				0x00, /* 3, attributes */
				0x00, 0x00,

				/* Consumer */
				0x07, /* 0, Descriptor size */
				CY_U3P_USB_ENDPNT_DESCR, /* 1, Endpoint descriptor type */
				0x84, /* 2, Endpoint number + direction */
				CY_U3P_USB_EP_BULK, /* 3, endpoint type */
				0x00, 0x04, /* 4, Max packet size */
				0x00, /* 6, Service interval */

				/* Super speed endpoint companion descriptor for Consumer EP */
				0x06, /* 0, Descriptor size */
				CY_U3P_SS_EP_COMPN_DESCR, /* 1, SS endpoint companion descriptor type  */
				0x0F, /* 2, bursts*/
				0x00, /* 3, attributes */
				0x00, 0x00,

				/* FPGA */

				0x09, /* 0, Descriptor size */
				CY_U3P_USB_INTRFC_DESCR, /* 1, Interface Descriptor type */
				0x02, /* 2, Interface number */
				0x00, /* 3, Alternate setting number */
				0x01, /* 4, Number of end points */
				0xFF, /* 5, Interface class */
				0x00, /* 5, Interface sub class */
				0x00, /* 6, Interface protocol code  */
				0x05, /* 7, Interface descriptor string index */

				/* FGPA Endpoint */
				0x07, /* 0, Descriptor size */
				CY_U3P_USB_ENDPNT_DESCR, /* 1, Endpoint descriptor type */
				0x06, /* 2, Endpoint number + direction */
				CY_U3P_USB_EP_BULK, /* 3, endpoint type */
				0x00, 0x04, /* 4, Max packet size */
				0x00, /* 6, Service interval */

				/* Super speed endpoint companion descriptor for producer EP */
				0x06, /* 0, Descriptor size */
				CY_U3P_SS_EP_COMPN_DESCR, /* 1, SS endpoint companion descriptor type  */
				0x00, /* 2, bursts*/
				0x00, /* 3, attributes */
				0x00, 0x00,

				/* UVC */

				/* Interface Association Descriptor */
				0x08, /* Descriptor Size */
				CY_FX_INTF_ASSN_DSCR_TYPE, /* Interface Association Descr Type: 11 */
				0x00, /* I/f number of first VideoControl i/f */
				0x02, /* Number of Video i/f */
				0x0E, /* CC_VIDEO : Video i/f class code */
				0x03, /* SC_VIDEO_INTERFACE_COLLECTION : Subclass code */
				0x00, /* Protocol : Not used */
				0x00, /* String desc index for interface */

				/* Standard Video Control Interface Descriptor */
				0x09, /* Descriptor size */
				CY_U3P_USB_INTRFC_DESCR, /* Interface Descriptor type */
				0x00, /* Interface number */
				0x00, /* Alternate setting number */
				0x01, /* Number of end points */
				0x0E, /* CC_VIDEO : Interface class */
				0x01, /* CC_VIDEOCONTROL : Interface sub class */
				0x00, /* Interface protocol code */
				0x00, /* Interface descriptor string index */

				/* Class specific VC Interface Header Descriptor */
				0x0D, /* Descriptor size */
				0x24, /* Class Specific I/f Header Descriptor type */
				0x01, /* Descriptor Sub type : VC_HEADER */
				0x00, 0x01, /* Revision of class spec : 1.0 */
				0x50, 0x00, /* Total Size of class specific descriptors (till Output terminal) */
				0x00, 0x6C, 0xDC, 0x02, /* Clock frequency : 48MHz(Deprecated) */
				0x01, /* Number of streaming interfaces */
				0x01, /* Video streaming I/f 1 belongs to VC i/f */

				/* Input (Camera) Terminal Descriptor */
				0x12, /* Descriptor size */
				0x24, /* Class specific interface desc type */
				0x02, /* Input Terminal Descriptor type */
				0x01, /* ID of this terminal */
				0x01, 0x02, /* Camera terminal type */
				0x00, /* No association terminal */
				0x00, /* String desc index : Not used */
		#ifdef UVC_PTZ_SUPPORT
				(uint8_t)(wObjectiveFocalLengthMin&0xFF),
				(uint8_t)((wObjectiveFocalLengthMin>>8)&0xFF),
				(uint8_t)(wObjectiveFocalLengthMax&0xFF),
				(uint8_t)((wObjectiveFocalLengthMax>>8)&0xFF),
				(uint8_t)(wOcularFocalLength&0xFF),
				(uint8_t)((wOcularFocalLength>>8)&0xFF),
		#else
				0x00, 0x00, /* No optical zoom supported */
				0x00, 0x00, /* No optical zoom supported */
				0x00, 0x00, /* No optical zoom supported */
		#endif
				0x03, /* Size of controls field for this terminal : 3 bytes */
				/* A bit set to 1 in the bmControls field indicates that
				 * the mentioned Control is supported for the video stream.
				 * D0: Scanning Mode
				 * D1: Auto-Exposure Mode
				 * D2: Auto-Exposure Priority
				 * D3: Exposure Time (Absolute)
				 * D4: Exposure Time (Relative)
				 * D5: Focus (Absolute)
				 * D6: Focus (Relative)
				 * D7: Iris (Absolute)
				 * D8: Iris (Relative)
				 * D9: Zoom (Absolute)
				 * D10: Zoom (Relative)
				 * D11: PanTilt (Absolute)
				 * D12: PanTilt (Relative)
				 * D13: Roll (Absolute)
				 * D14: Roll (Relative)
				 * D15: Reserved
				 * D16: Reserved
				 * D17: Focus, Auto
				 * D18: Privacy
				 * D19: Focus, Simple
				 * D20: Window
				 * D21: Region of Interest
				 * D22 \96 D23: Reserved, set to zero
				 */
		#ifdef UVC_PTZ_SUPPORT
				0x00,0x0A,0x00, /* bmControls field of camera terminal: PTZ supported */
		#else
				0x00, 0x00, 0x00, /* bmControls field of camera terminal: No controls supported */
		#endif

				/* Processing Unit Descriptor */
				0x0C, /* Descriptor size */
				0x24, /* Class specific interface desc type */
				0x05, /* Processing Unit Descriptor type */
				0x02, /* ID of this terminal */
				0x01, /* Source ID : 1 : Conencted to input terminal */
				0x00, 0x40, /* Digital multiplier */
				0x03, /* Size of controls field for this terminal : 3 bytes */
				0xFF, 0x01, 0x00, /* bmControls field of processing unit: Brightness control supported */
				0x00, /* String desc index : Not used */

				/* Extension Unit Descriptor */
				0x1C, /* Descriptor size */
				0x24, /* Class specific interface desc type */
				0x06, /* Extension Unit Descriptor type */
				0x03, /* ID of this terminal */
				0xFF, 0xFF, 0xFF, 0xFF, /* 16 byte GUID */
				0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
				0x00, /* Number of controls in this terminal */
				0x01, /* Number of input pins in this terminal */
				0x02, /* Source ID : 2 : Connected to Proc Unit */
				0x03, /* Size of controls field for this terminal : 3 bytes */
				/* A bit set to 1 in the bmControls field indicates that
				 * the mentioned Control is supported for the video stream.
				 * D0: Brightness
				 * D1: Contrast
				 * D2: Hue
				 * D3: Saturation
				 * D4: Sharpness
				 * D5: Gamma
				 * D6: White Balance Temperature
				 * D7: White Balance Component
				 * D8: Backlight Compensation
				 * D9: Gain
				 * D10: Power Line Frequency
				 * D11: Hue, Auto
				 * D12: White Balance Temperature, Auto
				 * D13: White Balance Component, Auto
				 * D14: Digital Multiplier
				 * D15: Digital Multiplier Limit
				 * D16: Analog Video Standard
				 * D17: Analog Video Lock Status
				 * D18: Contrast, Auto
				 * D19 \96 D23: Reserved. Set to zero.
				 */
				0x00, 0x00, 0x00, /* No controls supported */
				0x00, /* String desc index : Not used */

				/* Output Terminal Descriptor */
				0x09, /* Descriptor size */
				0x24, /* Class specific interface desc type */
				0x03, /* Output Terminal Descriptor type */
				0x04, /* ID of this terminal */
				0x01, 0x01, /* USB Streaming terminal type */
				0x00, /* No association terminal */
				0x03, /* Source ID : 3 : Connected to Extn Unit */
				0x00, /* String desc index : Not used */

				/* Video Control Status Interrupt Endpoint Descriptor */
				0x07, /* Descriptor size */
				CY_U3P_USB_ENDPNT_DESCR, /* Endpoint Descriptor Type */
				CY_FX_EP_CONTROL_STATUS, /* Endpoint address and description */
				CY_U3P_USB_EP_INTR, /* Interrupt End point Type */
				0x00, 0x04, /* Max packet size = 1024 bytes */
				0x01, /* Servicing interval */

				/* Super Speed Endpoint Companion Descriptor */
				0x06, /* Descriptor size */
				CY_U3P_SS_EP_COMPN_DESCR, /* SS Endpoint Companion Descriptor Type */
				0x00, /* Max no. of packets in a Burst : 1 */
				0x00, /* Attribute: N.A. */
				0x00, /* Bytes per interval:1024 */
				0x04,

				/* Class Specific Interrupt Endpoint Descriptor */
				0x05, /* Descriptor size */
				0x25, /* Class Specific Endpoint Descriptor Type */
				CY_U3P_USB_EP_INTR, /* End point Sub Type */
				0x40, 0x00, /* Max packet size = 64 bytes */

				/* Standard Video Streaming Interface Descriptor (Alternate Setting 0) */
				0x09, /* Descriptor size */
				CY_U3P_USB_INTRFC_DESCR, /* Interface Descriptor type */
				0x01, /* Interface number */
				0x00, /* Alternate setting number */
				0x01, /* Number of end points */
				0x0E, /* Interface class : CC_VIDEO */
				0x02, /* Interface sub class : CC_VIDEOSTREAMING */
				0x00, /* Interface protocol code : Undefined */
				0x00, /* Interface descriptor string index */

				/* Class-specific Video Streaming Input Header Descriptor */
				0x0E, /* Descriptor size */
				0x24, /* Class-specific VS I/f Type */
				0x01, /* Descriptotor Subtype : Input Header */
				0x01, /* 1 format desciptor follows */
				0x47, 0x00, /* Total size of Class specific VS descr */
				CY_FX_EP_BULK_VIDEO, /* EP address for BULK video data */
				0x00, /* No dynamic format change supported */
				0x04, /* Output terminal ID : 4 */
				0x01, /* Still image capture method 1 supported */
				0x00, /* Hardware trigger NOT supported */
				0x00, /* Hardware to initiate still image capture NOT supported */
				0x01, /* Size of controls field : 1 byte */
				0x00, /* D2 : Compression quality supported */

				/* Class specific Uncompressed VS format descriptor */
				0x1B, /* Descriptor size */
				0x24, /* Class-specific VS I/f Type */
				0x04, /* Subtype : uncompressed format I/F */
				0x01, /* Format desciptor index */
				0x01, /* Number of frame descriptor followed */
				//0x59, 0x55, 0x59, 0x32, /* GUID used to identify streaming-encoding format: YUY2  */
				//0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71,

				0x7E, 0xEB, 0x36, 0xE4,         /*MEDIASUBTYPE_RGB32 GUID: E436EB7E-524F-11CE-9F53-0020AF0BA770  */
				0x4F, 0x52, 0xCE, 0x11,
				0x9F, 0x53, 0x00, 0x20,
				0xAF, 0x0B, 0xA7, 0x70,


				0x20, /* Number of bits per pixel */
				0x01, /* Optimum Frame Index for this stream: 1 */
				0x08, /* X dimension of the picture aspect ratio; Non-interlaced */
				0x06, /* Y dimension of the pictuer aspect ratio: Non-interlaced */
				0x00, /* Interlace Flags: Progressive scanning, no interlace */
				0x00, /* duplication of the video stream restriction: 0 - no restriction */

				/* Class specific Uncompressed VS frame descriptor */
				0x1E, /* Descriptor size */
				0x24, /* Descriptor type*/
				0x05, /* Subtype: uncompressed frame I/F */
				0x01, /* Frame Descriptor Index */
				0x03, /* Still image capture method 1 supported, fixed frame rate */
				0xC2, 0x03, /* Width in pixel */
				0x7F, 0x01, /* Height in pixel */
				0x00, 0x51, 0x2A, 0x2A, /* Min bit rate bits/s. */
				0x00, 0x51, 0x2A, 0x2A, /* Max bit rate bits/s. */
				0xF8, 0x7C, 0x16, 0x00, /* Maximum video or still frame size in bytes(Deprecated)*/
				0x0A, 0x8B, 0x02, 0x00, /* 60fps */
				0x01,
				0x0A, 0x8B, 0x02, 0x00,

				/* Endpoint Descriptor for BULK Streaming Video Data */
				0x07, /* Descriptor size */
				CY_U3P_USB_ENDPNT_DESCR, /* Endpoint Descriptor Type */
				CY_FX_EP_BULK_VIDEO, /* Endpoint address and description */
				CY_U3P_USB_EP_BULK, /* BULK End point */
				CY_FX_EP_BULK_VIDEO_PKT_SIZE_L, /* EP MaxPcktSize: 1024B */
				CY_FX_EP_BULK_VIDEO_PKT_SIZE_H, /* EP MaxPcktSize: 1024B */
				0x01, /* Servicing interval for data transfers */

				/* Super Speed Endpoint Companion Descriptor */
				0x06, /* Descriptor size */
				CY_U3P_SS_EP_COMPN_DESCR, /* SS Endpoint Companion Descriptor Type */
				0x0F, /* Max number of packets per burst: 16 */
				0x00, /* Attribute: Streams not defined */
				0x00, /* No meaning for bulk */
				0x00

};

// high speed configuration descriptor
const uint8_t ztex_usb2_config_descriptor[] __attribute__ ((aligned (32))) = {
		// Configuration descriptor
		9,// 0, Descriptor size
		CY_U3P_USB_CONFIG_DESCR,       		// 1, Configuration descriptor type
        0xF4,0x00,                      /* Length of this descriptor and all sub descriptors */
        0x04,                           /* Number of interfaces */
		0x01,                           		// 5, Configuration number
		0x00,                           		// 6, Configuration string index
		0x80,                           		// 7, attributes: bus
		0xFA,         // 8, Max power consumption of device (in 8mA units) : 200mA

		// Interface descriptors

		0x09, /* 0, Descriptor size */
		CY_U3P_USB_INTRFC_DESCR, /* 1, Interface Descriptor type */
		0x03, /* 2, Interface number */
		0x00, /* 3, Alternate setting number */
		0x02, /* 4, Number of end points */
		0xFF, /* 5, Interface class */
		0x00, /* 5, Interface sub class */
		0x00, /* 6, Interface protocol code  */
		0x04, /* 7, Interface descriptor string index */

		/* Producer */
		0x07, /* 0, Descriptor size */
		CY_U3P_USB_ENDPNT_DESCR, /* 1, Endpoint descriptor type */
		0x02, /* 2, Endpoint number + direction */
		CY_U3P_USB_EP_BULK, /* 3, endpoint type */
		0x00, 0x02, /* 4, Max packet size */
		0x00, /* 6, Service interval */

		/* Consumer */
		0x07, /* 0, Descriptor size */
		CY_U3P_USB_ENDPNT_DESCR, /* 1, Endpoint descriptor type */
		0x84, /* 2, Endpoint number + direction */
		CY_U3P_USB_EP_BULK, /* 3, endpoint type */
		0x00, 0x02, /* 4, Max packet size */
		0x00, /* 6, Service interval */

		//FPGA

		0x09, /* 0, Descriptor size */
		CY_U3P_USB_INTRFC_DESCR, /* 1, Interface Descriptor type */
		0x02, /* 2, Interface number */
		0x00, /* 3, Alternate setting number */
		0x01, /* 4, Number of end points */
		0xFF, /* 5, Interface class */
		0x00, /* 5, Interface sub class */
		0x00, /* 6, Interface protocol code  */
		0x05, /* 7, Interface descriptor string index */

		/* FGPA Endpoint */
		0x07, /* 0, Descriptor size */
		CY_U3P_USB_ENDPNT_DESCR, /* 1, Endpoint descriptor type */
		0x06, /* 2, Endpoint number + direction */
		CY_U3P_USB_EP_BULK, /* 3, endpoint type */
		0x00, 0x02, /* 4, Max packet size */
		0x00,

		//UVC START

		  /* Interface Association Descriptor */
		        0x08,                           /* Descriptor Size */
		        CY_FX_INTF_ASSN_DSCR_TYPE,      /* Interface Association Descr Type: 11 */
		        0x00,                           /* I/f number of first VideoControl i/f */
		        0x02,                           /* Number of Video i/f */
		        0x0E,                           /* CC_VIDEO : Video i/f class code */
		        0x03,                           /* SC_VIDEO_INTERFACE_COLLECTION : Subclass code */
		        0x00,                           /* Protocol : Not used */
		        0x00,                           /* String desc index for interface */

		        /* Standard Video Control Interface Descriptor */
		        0x09,                           /* Descriptor size */
		        CY_U3P_USB_INTRFC_DESCR,        /* Interface Descriptor type */
		        0x00,                           /* Interface number */
		        0x00,                           /* Alternate setting number */
		        0x01,                           /* Number of end points */
		        0x0E,                           /* CC_VIDEO : Interface class */
		        0x01,                           /* CC_VIDEOCONTROL : Interface sub class */
		        0x00,                           /* Interface protocol code */
		        0x00,                           /* Interface descriptor string index */

		        /* Class specific VC Interface Header Descriptor */
		        0x0D,                           /* Descriptor size */
		        0x24,                           /* Class Specific I/f Header Descriptor type */
		        0x01,                           /* Descriptor Sub type : VC_HEADER */
		        0x00,0x01,                      /* Revision of class spec : 1.0 */
		        0x50,0x00,                      /* Total Size of class specific descriptors (till Output terminal) */
		        0x00,0x6C,0xDC,0x02,            /* Clock frequency : 48MHz(Deprecated) */
		        0x01,                           /* Number of streaming interfaces */
		        0x01,                           /* Video streaming I/f 1 belongs to VC i/f */

		        /* Input (Camera) Terminal Descriptor */
		        0x12,                           /* Descriptor size */
		        0x24,                           /* Class specific interface desc type */
		        0x02,                           /* Input Terminal Descriptor type */
		        0x01,                           /* ID of this terminal */
		        0x01,0x02,                      /* Camera terminal type */
		        0x00,                           /* No association terminal */
		        0x00,                           /* String desc index : Not used */
		#ifdef UVC_PTZ_SUPPORT
		        (uint8_t)(wObjectiveFocalLengthMin&0xFF),
		        (uint8_t)((wObjectiveFocalLengthMin>>8)&0xFF),
		        (uint8_t)(wObjectiveFocalLengthMax&0xFF),
		        (uint8_t)((wObjectiveFocalLengthMax>>8)&0xFF),
		        (uint8_t)(wOcularFocalLength&0xFF),
		        (uint8_t)((wOcularFocalLength>>8)&0xFF),
		#else
		        0x00,0x00,                      /* No optical zoom supported */
		        0x00,0x00,                      /* No optical zoom supported */
		        0x00,0x00,                      /* No optical zoom supported */
		#endif
		        0x03,                           /* Size of controls field for this terminal : 3 bytes */
		                                        /* A bit set to 1 indicates that the mentioned Control is
		                                         * supported for the video stream in the bmControls field
		                                         * D0: Scanning Mode
		                                         * D1: Auto-Exposure Mode
		                                         * D2: Auto-Exposure Priority
		                                         * D3: Exposure Time (Absolute)
		                                         * D4: Exposure Time (Relative)
		                                         * D5: Focus (Absolute)
		                                         * D6: Focus (Relative)
		                                         * D7: Iris (Absolute)
		                                         * D8: Iris (Relative)
		                                         * D9: Zoom (Absolute)
		                                         * D10: Zoom (Relative)
		                                         * D11: PanTilt (Absolute)
		                                         * D12: PanTilt (Relative)
		                                         * D13: Roll (Absolute)
		                                         * D14: Roll (Relative)
		                                         * D15: Reserved
		                                         * D16: Reserved
		                                         * D17: Focus, Auto
		                                         * D18: Privacy
		                                         * D19: Focus, Simple
		                                         * D20: Window
		                                         * D21: Region of Interest
		                                         * D22 \96 D23: Reserved, set to zero
		                                         */
		#ifdef UVC_PTZ_SUPPORT
		        0x00,0x0A,0x00,                 /* bmControls field of camera terminal: PTZ supported */
		#else
		        0x00,0x00,0x00,                 /* bmControls field of camera terminal: No controls supported */
		#endif

		        /* Processing Unit Descriptor */
		        0x0C,                           /* Descriptor size */
		        0x24,                           /* Class specific interface desc type */
		        0x05,                           /* Processing Unit Descriptor type */
		        0x02,                           /* ID of this terminal */
		        0x01,                           /* Source ID : 1 : Conencted to input terminal */
		        0x00,0x40,                      /* Digital multiplier */
		        0x03,                           /* Size of controls field for this terminal : 3 bytes */
		                                        /* A bit set to 1 in the bmControls field indicates that
		                                         * the mentioned Control is supported for the video stream.
		                                         * D0: Brightness
		                                         * D1: Contrast
		                                         * D2: Hue
		                                         * D3: Saturation
		                                         * D4: Sharpness
		                                         * D5: Gamma
		                                         * D6: White Balance Temperature
		                                         * D7: White Balance Component
		                                         * D8: Backlight Compensation
		                                         * D9: Gain
		                                         * D10: Power Line Frequency
		                                         * D11: Hue, Auto
		                                         * D12: White Balance Temperature, Auto
		                                         * D13: White Balance Component, Auto
		                                         * D14: Digital Multiplier
		                                         * D15: Digital Multiplier Limit
		                                         * D16: Analog Video Standard
		                                         * D17: Analog Video Lock Status
		                                         * D18: Contrast, Auto
		                                         * D19 \96 D23: Reserved. Set to zero.
		                                         */
		        0xFF,0x01,0x00,                 /* bmControls field of processing unit: Brightness control supported */
		        0x00,                           /* String desc index : Not used */

		        /* Extension Unit Descriptor */
		        0x1C,                           /* Descriptor size */
		        0x24,                           /* Class specific interface desc type */
		        0x06,                           /* Extension Unit Descriptor type */
		        0x03,                           /* ID of this terminal */
		        0xFF,0xFF,0xFF,0xFF,            /* 16 byte GUID */
		        0xFF,0xFF,0xFF,0xFF,
		        0xFF,0xFF,0xFF,0xFF,
		        0xFF,0xFF,0xFF,0xFF,
		        0x00,                           /* Number of controls in this terminal */
		        0x01,                           /* Number of input pins in this terminal */
		        0x02,                           /* Source ID : 2 : Connected to Proc Unit */
		        0x03,                           /* Size of controls field for this terminal : 3 bytes */
		        0x00,0x00,0x00,                 /* No controls supported */
		        0x00,                           /* String desc index : Not used */

		        /* Output Terminal Descriptor */
		        0x09,                           /* Descriptor size */
		        0x24,                           /* Class specific interface desc type */
		        0x03,                           /* Output Terminal Descriptor type */
		        0x04,                           /* ID of this terminal */
		        0x01,0x01,                      /* USB Streaming terminal type */
		        0x00,                           /* No association terminal */
		        0x03,                           /* Source ID : 3 : Connected to Extn Unit */
		        0x00,                           /* String desc index : Not used */

		        /* Video Control Status Interrupt Endpoint Descriptor */
		        0x07,                           /* Descriptor size */
		        CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint Descriptor Type */
		        CY_FX_EP_CONTROL_STATUS,        /* Endpoint address and description */
		        CY_U3P_USB_EP_INTR,             /* Interrupt End point Type */
		        0x40,0x00,                      /* Max packet size = 64 bytes */
		        0x08,                           /* Servicing interval : 8ms */

		        /* Class Specific Interrupt Endpoint Descriptor */
		        0x05,                           /* Descriptor size */
		        0x25,                           /* Class Specific Endpoint Descriptor Type */
		        CY_U3P_USB_EP_INTR,             /* End point Sub Type */
		        0x40,0x00,                      /* Max packet size = 64 bytes */

		        /* Standard Video Streaming Interface Descriptor (Alternate Setting 0) */
		        0x09,                           /* Descriptor size */
		        CY_U3P_USB_INTRFC_DESCR,        /* Interface Descriptor type */
		        0x01,                           /* Interface number */
		        0x00,                           /* Alternate setting number */
		        0x01,                           /* Number of end points : Zero Bandwidth */
		        0x0E,                           /* Interface class : CC_VIDEO */
		        0x02,                           /* Interface sub class : CC_VIDEOSTREAMING */
		        0x00,                           /* Interface protocol code : Undefined */
		        0x00,                           /* Interface descriptor string index */

		       /* Class-specific Video Streaming Input Header Descriptor */
		        0x0E,                           /* Descriptor size */
		        0x24,                           /* Class-specific VS I/f Type */
		        0x01,                           /* Descriptotor Subtype : Input Header */
		        0x01,                           /* 1 format desciptor follows */
		        0x29,0x00,                      /* Total size of Class specific VS descr: 41 Bytes */
		        CY_FX_EP_BULK_VIDEO,            /* EP address for BULK video data */
		        0x00,                           /* No dynamic format change supported */
		        0x04,                           /* Output terminal ID : 4 */
		        0x01,                           /* Still image capture method 1 supported */
		        0x00,                           /* Hardware trigger NOT supported */
		        0x00,                           /* Hardware to initiate still image capture NOT supported */
		        0x01,                           /* Size of controls field : 1 byte */
		        0x00,                           /* D2 : Compression quality supported */


		       /* Class specific Uncompressed VS format descriptor */
		        0x1B,                           /* Descriptor size */
		        0x24,                           /* Class-specific VS I/f Type */
		        0x04,                           /* Subtype : uncompressed format I/F */
		        0x01,                           /* Format desciptor index (only one format is supported) */
		        0x01,                           /* number of frame descriptor followed */
		        0x7E,0xEB,0x36,0xE4,            /* GUID used to identify streaming-encoding format: YUY2  */
		        0x4F,0x52,0xCE,0x11,
		        0x9F,0x53,0x00,0x20,
		        0xAF,0x0B,0xA7,0x70,
		        0x20,                           /* Number of bits per pixel used to specify color in the decoded video frame.
		                                           0 if not applicable: 10 bit per pixel */
		        0x01,                           /* Optimum Frame Index for this stream: 1 */
		        0x08,                           /* X dimension of the picture aspect ratio: Non-interlaced in
					        	   progressive scan */
		        0x06,                           /* Y dimension of the picture aspect ratio: Non-interlaced in
							   progressive scan*/
		        0x00,                           /* Interlace Flags: Progressive scanning, no interlace */
		        0x00,                           /* duplication of the video stream restriction: 0 - no restriction */



		       /* Class specific Uncompressed VS Frame descriptor */
		        0x1E,                           /* Descriptor size */
		        0x24,                           /* Descriptor type*/
		        0x05,                           /* Subtype: uncompressed frame I/F */
		        0x01,                           /* Frame Descriptor Index */
		        0x03,                           /* Still image capture method 1 supported, fixed frame rate */
		        0xC2,0x03,                      /* Width in pixel: 320-QVGA */
		        0x7F,0x01,                      /* Height in pixel 240-QVGA */
		        0x00,0x51,0x2A,0x2A,            /* Min bit rate bits/s. Not specified, taken from MJPEG */
		        0x00,0x51,0x2A,0x2A,            /* Max bit rate bits/s. Not specified, taken from MJPEG */
		        0xF8,0x7C,0x16,0x00,            /* Maximum video or still frame size in bytes(Deprecated) */
		        0x0A,0x8B,0x02,0x00,            /* Default Frame Interval */
		        0x01,                           /* Frame interval(Frame Rate) types: Only one frame interval supported */
		        0x0A,0x8B,0x02,0x00,            /* Shortest Frame Interval */

		        /* Endpoint Descriptor for BULK Streaming Video Data */
		        0x07,                           /* Descriptor size */
		        CY_U3P_USB_ENDPNT_DESCR,        /* Endpoint Descriptor Type */
		        CY_FX_EP_BULK_VIDEO,            /* Endpoint address and description */
		        0x02,                           /* BULK End point */
		        (uint8_t)(512 & 0x00FF),        /* High speed max packet size is always 512 bytes. */
		        (uint8_t)((512 & 0xFF00)>>8),
		        0x01                            /* Servicing interval for data transfers */
};

// full speed configuration descriptor
const uint8_t ztex_usb1_config_descriptor[] __attribute__ ((aligned (32))) = {
// Configuration descriptor
		9,// 0, Descriptor size
		CY_U3P_USB_CONFIG_DESCR,       		// 1, Configuration descriptor type
		0x30, 0x00,		// 2, Length of this descriptor and all sub descriptors
		0x02, 0x01,                           		// 5, Configuration number
		0x00,                           		// 6, Configuration string index
		0x80,                           		// 7, attributes: bus
		25,         // 8, Max power consumption of device (in 8mA units) : 200mA

		// Interface descriptors

		0x09, /* 0, Descriptor size */
		CY_U3P_USB_INTRFC_DESCR, /* 1, Interface Descriptor type */
		0x02, /* 2, Interface number */
		0x00, /* 3, Alternate setting number */
		0x02, /* 4, Number of end points */
		0xFF, /* 5, Interface class */
		0x00, /* 5, Interface sub class */
		0x00, /* 6, Interface protocol code  */
		0x04, /* 7, Interface descriptor string index */

		/* Producer */
		0x07, /* 0, Descriptor size */
		CY_U3P_USB_ENDPNT_DESCR, /* 1, Endpoint descriptor type */
		0x02, /* 2, Endpoint number + direction */
		CY_U3P_USB_EP_BULK, /* 3, endpoint type */
		0x00, 0x02, /* 4, Max packet size */
		0x00, /* 6, Service interval */

		/* Consumer */
		0x07, /* 0, Descriptor size */
		CY_U3P_USB_ENDPNT_DESCR, /* 1, Endpoint descriptor type */
		0x84, /* 2, Endpoint number + direction */
		CY_U3P_USB_EP_BULK, /* 3, endpoint type */
		0x00, 0x02, /* 4, Max packet size */
		0x00, /* 6, Service interval */

		//FPGA

		0x09, /* 0, Descriptor size */
		CY_U3P_USB_INTRFC_DESCR, /* 1, Interface Descriptor type */
		0x03, /* 2, Interface number */
		0x00, /* 3, Alternate setting number */
		0x01, /* 4, Number of end points */
		0xFF, /* 5, Interface class */
		0x00, /* 5, Interface sub class */
		0x00, /* 6, Interface protocol code  */
		0x05, /* 7, Interface descriptor string index */

		/* FGPA Endpoint */
		0x07, /* 0, Descriptor size */
		CY_U3P_USB_ENDPNT_DESCR, /* 1, Endpoint descriptor type */
		0x06, /* 2, Endpoint number + direction */
		CY_U3P_USB_EP_BULK, /* 3, endpoint type */
		0x00, 0x02, /* 4, Max packet size */
		0x00, };

// language string descriptor */
const uint8_t ztex_lang_string_descriptor[] __attribute__ ((aligned (32))) = {
		0x04,                           // Descriptor size
		CY_U3P_USB_STRING_DESCR,        // Device descriptor type
		0x04, 0x09 };

/* Place this buffer as the last buffer so that no other variable / code shares
 the same cache line. Do not add any other variables / arrays in this file.
 This will lead to variables sharing the same cache line. */
const uint8_t descriptor_allign_buffer[32] __attribute__ ((aligned (32)));

const uint8_t ztex_descriptor[] __attribute__ ((aligned (32))) = {
		40,                 	// Descriptor size
		0x01, 		  	// Descriptor version
		'Z', 'T', 'E',
		'X', 		// Signature "ZTEX"

		ZTEX_PRODUCT_ID_0,	  	// product ID's
		ZTEX_PRODUCT_ID_1, ZTEX_PRODUCT_ID_2, ZTEX_PRODUCT_ID_3,

		ZTEX_FWVER,		 	// firmware version

		1,				// interface version

		// interface capabilities
		0
#ifdef _ZTEX_FPGA_
		| 2				// FPGA configuration support
#endif
#ifdef ENABLE_SPI_FLASH
		| 4				// SPI Flash support
#endif
#ifdef ZTEX_FPGA_CONF_FAST_EP
		| 32			// fast FPGA configuration support
#endif
#ifdef ENABLE_I2C
		| 64			// MAC EEPROM support
#endif
		, 4 | 8			// FX3, debug2
#ifdef ENABLE_SD_FLASH
				| 2				// SD Flash support
#endif
#ifdef _ZTEX_LSI_
				| 16			// default firmware interface support
#endif
				, 0, 0, 0, 0,

		ZTEX_MODULE_RESERVED_00,	// 11 bytes which can be used by application
		ZTEX_MODULE_RESERVED_01, ZTEX_MODULE_RESERVED_02,
		ZTEX_MODULE_RESERVED_03, ZTEX_MODULE_RESERVED_04,
		ZTEX_MODULE_RESERVED_05, ZTEX_MODULE_RESERVED_06,
		ZTEX_MODULE_RESERVED_07, ZTEX_MODULE_RESERVED_08,
		ZTEX_MODULE_RESERVED_09, ZTEX_MODULE_RESERVED_10,

		207,			// must be 207 for FX3 firmwares

		'0', '0', '0', '0', '0', '0', '0', '0', '0', '0'// 1 bytes serial number string
		};

#undef W2B
#undef DIR_IN
#undef DIR_OUT
#undef TYPE_ISO
#undef TYPE_BULK
#undef TYPE_INT


uint8_t ztex_ep0buf[4096] __attribute__ ((aligned (32)));

const char ztex_manufacturer_string[] = ZTEX_MANUFACTURER_STRING;

char ztex_product_string[64] = ZTEX_PRODUCT_STRING;

char ztex_sn_string[] = "0000000000";

#endif // _ZTEX_DESCRIPTORS_C_
