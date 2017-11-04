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
 Main include file. See the examples for usage.
 */

//Maheswari test

#include "cyu3system.h"
#include "cyu3os.h"
#include "cyu3dma.h"
#include "cyu3error.h"
#include "cyu3usb.h"
#include "cyu3uart.h"
#include "cyu3pib.h"
#include "cyu3gpif.h"
#include "cyu3i2c.h"
#include "cyu3utils.h"
#include "cyu3socket.h"
#include <cyu3types.h>
#include <cyu3gpio.h>

#include "uvc.h"
#include "sensor.h"
#include "sensor.c"
#include "camera_ptzcontrol.h"
#include "cyfxgpif2config.h"

CyU3PPibClock_t ztex_pib_clock = { .clkDiv = 2,	// 69.33 MHz @ 26 MHz external clock
		.clkSrc = CY_U3P_SYS_CLK, .isHalfDiv = CyFalse, .isDllEnable = CyFalse };

CyU3PPibClock_t *ztex_pib_clock2 = &ztex_pib_clock;
#define CY_FX_EP_DEBUG_RSP_SOCKET       0x04    /* USB Consumer socket 4 is used as the debug response pipe. */

/*************************************************************************************************
 Global Variables
 *************************************************************************************************/
static CyU3PThread uvcAppThread; /* UVC video streaming thread. */
static CyU3PThread uvcAppEP0Thread; /* UVC control request handling thread. */
static CyU3PEvent glFxUVCEvent; /* Event group used to signal threads. */
CyU3PDmaChannel glChHandleUVCStream; /* DMA multi-channel handle. */

/* Current UVC control request fields. See USB specification for definition. */
uint8_t bmReqType, bRequest, bType, bTarget; /* bmReqType and bRequest fields. */
uint16_t wValue, wIndex, wLength; /* wValue, wIndex and wLength fields. */

CyU3PUSBSpeed_t usbSpeed = CY_U3P_NOT_CONNECTED; /* Current USB connection speed. */
CyBool_t clearFeatureRqtReceived = CyFalse; /* Whether a CLEAR_FEATURE (stop streaming) request has been
 received. */
CyBool_t streamingStarted = CyFalse; /* Whether USB host has started streaming data */

#ifdef BACKFLOW_DETECT
uint8_t back_flow_detected = 0; /* Whether buffer overflow error is detected. */
#endif

volatile static uint32_t glFrameCount = 0;
volatile static uint32_t glDmaDone = 1;
/* UVC Probe Control Settings for a USB 3.0 connection. */
uint8_t glProbeCtrl[CY_FX_UVC_MAX_PROBE_SETTING] = { 0x00, 0x00, /* bmHint : no hit */
0x01, /* Use 1st Video format index */
0x01, /* Use 1st Video frame index */
0x0A, 0x8B, 0x02, 0x00, /* Desired frame interval in the unit of 100ns: 60 fps */
0x00, 0x00, /* Key frame rate in key frame/video frame units: only applicable
 to video streaming with adjustable compression parameters */
0x00, 0x00, /* PFrame rate in PFrame / key frame units: only applicable to
 video streaming with adjustable compression parameters */
0x00, 0x00, /* Compression quality control: only applicable to video streaming
 with adjustable compression parameters */
0x00, 0x00, /* Window size for average bit rate: only applicable to video
 streaming with adjustable compression parameters */
0x00, 0x00, /* Internal video streaming i/f latency in ms */
0xF8, 0x7C, 0x16, 0x00, /* Max video frame size in bytes */
0x00, 0x40, 0x00, 0x00 /* No. of bytes device can rx in single payload = 16 KB */
};

/* UVC Probe Control Setting for a USB 2.0 connection. */
uint8_t glProbeCtrl20[CY_FX_UVC_MAX_PROBE_SETTING] = { 0x00, 0x00, /* bmHint : no hit */
0x01, /* Use 1st Video format index */
0x01, /* Use 1st Video frame index */
0x0A, 0x8B, 0x02, 0x00, /* Desired frame interval in the unit of 100ns: 15 fps */
0x00, 0x00, /* Key frame rate in key frame/video frame units: only applicable
 to video streaming with adjustable compression parameters */
0x00, 0x00, /* PFrame rate in PFrame / key frame units: only applicable to
 video streaming with adjustable compression parameters */
0x00, 0x00, /* Compression quality control: only applicable to video streaming
 with adjustable compression parameters */
0x00, 0x00, /* Window size for average bit rate: only applicable to video
 streaming with adjustable compression parameters */
0x00, 0x00, /* Internal video streaming i/f latency in ms */
0xF8, 0x7C, 0x16, 0x00, /* Max video frame size in bytes */
0x00, 0x40, 0x00, 0x00 /* No. of bytes device can rx in single payload = 16 KB */
};

/* Video Probe Commit Control. This array is filled out when the host sends down the SET_CUR request. */
static uint8_t glCommitCtrl[CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED];

/* Scratch buffer used for handling UVC class requests with a data phase. */
static uint8_t glEp0Buffer[32];

/* UVC Header to be prefixed at the top of each 16 KB video data buffer. */
uint8_t volatile glUVCHeader[CY_FX_UVC_MAX_HEADER] = { 0x0C, /* Header Length */
0x8C, /* Bit field header field */
0x00, 0x00, 0x00, 0x00, /* Presentation time stamp field */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00 /* Source clock reference field */
};

#define	ZTEX_VENDOR_REQ_MAX 	50			// maximum amount of vendor requests
#define	ZTEX_VENDOR_CMD_MAX 	50			// maximum amount of vendor commands

typedef uint8_t (*ztex_vendor_func)(uint16_t value, uint16_t index,
		uint16_t length);

// global configuration
uint32_t ztex_app_thread_stack = 0x1000;	// stack size of application thread
uint32_t ztex_app_thread_prio = 8;// priority of application thread, should be 7..15
void (*ztex_app_thread_run)() = 0;
void (*ztex_usb_start)() = 0;			// called when USB connection is started
void (*ztex_usb_stop)() = 0;			// called when USB connection is stopped

CyBool_t ztex_allow_lpm = CyFalse;// whether to allow low power mode transitions

// strings for interfaces 0..7, can be overwritten by user
char* ztex_interface_string[] =
		{ NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL };

// called to clean up a transfer when a clear stall request is received, see 
void ep_cleanup_default_handler( uint16_t);
void (*ztex_ep_cleanup_handler)( uint16_t) = ep_cleanup_default_handler;

// system stuff
uint8_t ztex_usb_is_connected = 0;

// super speed error counters
#define ZTEX_USB3_SND_ERROR_COUNT (*((uint16_t*) 0xe0033014))
#define ZTEX_USB3_RCV_ERROR_COUNT (*((uint16_t*) 0xe0033016))

#ifndef EP_SETUP_FPGACONF
#define EP_SETUP_FPGACONF
#endif

#define EP_SETUP_ALL EP_SETUP EP_SETUP_FPGACONF

#include "ztex-descriptors.c"
#include "ztex-debug.c"

#include "ztex-ep0.c"
#include "ztex-gpio.c"

// SPI Flash support
#ifdef ENABLE_SPI_FLASH
#define ENABLE_SPI
#include "ztex-flash.c"
#endif

#include "ztex-i2c.c"

#define _ZTEX_INCLUDE_2_

#ifdef _ZTEX_CONF_UFM_2_14_C1_
#include "ztex-ufm-2_14.c"
#endif

#ifdef _ZTEX_LSI_
#include "ztex-lsi.c"
#endif
//#define ENABLE_I2C 1
void ztex_usb_start_usb() {

	CyU3PUSBSpeed_t usbSpeed = CyU3PUsbGetSpeed();
//	ZTEX_REC(CyU3PPibInit(CyTrue,ztex_pib_clock2));
//	CyU3PThreadResume(&uvcAppThread);
//	CyU3PThreadResume(&uvcAppEP0Thread);

}

void ztex_usb_stop_usb() {
	//ZTEX_REC(CyU3PPibDeInit());
}

void ztex_usb_start_main() {
	ztex_usb_start_usb();

	if (ztex_usb_start != 0)
		ztex_usb_start();

	CyU3PUSBSpeed_t usbSpeed = CyU3PUsbGetSpeed();
	ZTEX_LOG("Info: USB setup finished: %s",
			usbSpeed == CY_U3P_SUPER_SPEED ? "super speed" :
			usbSpeed == CY_U3P_HIGH_SPEED ? "high speed" :
			usbSpeed == CY_U3P_FULL_SPEED ? "full speed" : "not connected");

	ztex_usb_is_connected = 1;

}

void ztex_usb_stop_main() {
	ztex_usb_is_connected = 0;

	if (ztex_usb_stop != 0)
		ztex_usb_stop();

#ifdef ENABLE_SPI_FLASH
	ztex_usb_stop_flash();
#endif

#ifdef _ZTEX_BOARD_
	ztex_board_stop();
#endif    

	ztex_usb_stop_usb();

	ztex_log("Info: USB disconnected.");
}

/* 
 Default handler to clean up a transfer. Since we do no known how endpoints
 are associated we reset the whole connection. This may no be what is
 expected by the host.
 */
void ep_cleanup_default_handler(uint16_t ep) {
	ztex_usb_stop_main();
	CyU3PThreadSleep(1);  // Give a chance for the main thread loop to run
	ztex_usb_start_main();
}
;

// USB event handler
void ztex_usb_event_handler(CyU3PUsbEventType_t evtype, uint16_t evdata) {
//    ZTEX_LOG("Event: %d",evtype);
	switch (evtype) {
	case CY_U3P_USB_EVENT_SETCONF:
		// stop the connection before restarting
		if (ztex_usb_is_connected) {
			ztex_usb_stop_main();
		}
		// start the connection
		ztex_usb_start_main();
		break;

	case CY_U3P_USB_EVENT_RESET:
		CyU3PDebugPrint(4, "RESET encountered...\r\n");
		CyU3PGpifDisable(CyFalse);
		streamingStarted = CyFalse;
		CyFxUVCApplnAbortHandler();
		break;

	case CY_U3P_USB_EVENT_SUSPEND:
		CyU3PDebugPrint(4, "SUSPEND encountered...\r\n");
		CyU3PGpifDisable(CyFalse);
		streamingStarted = CyFalse;
		CyFxUVCApplnAbortHandler();
		break;

	case CY_U3P_USB_EVENT_EP_UNDERRUN:
		CyU3PDebugPrint(4, "CY_U3P_USB_EVENT_EP_UNDERRUN encountered...\r\n");
		break;

	case CY_U3P_USB_EVENT_DISCONNECT:
		// stops the connection
		if (ztex_usb_is_connected) {
			ztex_usb_stop_main();
			CyU3PDebugPrint(4, "USB disconnected...\r\n");
			CyU3PGpifDisable(CyFalse);
			streamingStarted = CyFalse;
			CyFxUVCApplnAbortHandler();
		}
		break;

	default:
		break;
	}
}

// LPM (link power management request). Return value CyTrue allows transitions to low power modes.
CyBool_t ztex_lpm_handler(CyU3PUsbLinkPowerMode link_mode) {
	return ztex_allow_lpm;
}

/* Add the UVC packet header to the top of the specified DMA buffer. */
void CyFxUVCAddHeader(uint8_t *buffer_p, /* Buffer pointer */
uint8_t frameInd /* EOF or normal frame indication */
) {
	/* Copy header to buffer */
	CyU3PMemCopy(buffer_p, (uint8_t *) glUVCHeader, CY_FX_UVC_MAX_HEADER);

	/* The EOF flag needs to be set if this is the last packet for this video frame. */
	if (frameInd & CY_FX_UVC_HEADER_EOF) {
		buffer_p[1] |= CY_FX_UVC_HEADER_EOF;
		glUVCHeader[1] ^= CY_FX_UVC_HEADER_FRAME_ID;
	}
}

/* Application Error Handler */
void CyFxAppErrorHandler(CyU3PReturnStatus_t apiRetStatus /* API return status */
) {
	/* This function is hit when we have hit a critical application error. This is not
	 expected to happen, and the current implementation of this function does nothing
	 except stay in a loop printing error messages through the UART port.

	 This function can be modified to take additional error handling actions such
	 as cycling the USB connection or performing a warm reset.
	 */
	for (;;) {
		ztex_log("Error handler...\r\n");
		CyU3PDebugPrint(4, "Error handler...\r\n");
		CyU3PThreadSleep(1000);
	}
}

/* This function performs the operations for a Video Streaming Abort.
 This is called every time there is a USB reset, suspend or disconnect event.
 */
void CyFxUVCApplnAbortHandler(void) {
	/* Set Video Stream Abort Event */
	CyU3PEventSet(&glFxUVCEvent, CY_FX_UVC_STREAM_ABORT_EVENT, CYU3P_EVENT_OR);
}

///* Callback to handle the USB Setup Requests and UVC Class events */
//static CyBool_t CyFxUVCApplnUSBSetupCB(uint32_t setupdat0, /* SETUP Data 0 */
//uint32_t setupdat1 /* SETUP Data 1 */
//) {
//	CyBool_t uvcHandleReq = CyFalse;
//	uint32_t status;
//
//	/* Obtain Request Type and Request */
//	bmReqType = (uint8_t)(setupdat0 & CY_FX_USB_SETUP_REQ_TYPE_MASK);
//	bRequest = (uint8_t)((setupdat0 & CY_FX_USB_SETUP_REQ_MASK) >> 8);
//	wValue = (uint16_t)((setupdat0 & CY_FX_USB_SETUP_VALUE_MASK) >> 16);
//	wIndex = (uint16_t)(setupdat1 & CY_FX_USB_SETUP_INDEX_MASK);
//	wLength = (uint16_t)((setupdat1 & CY_FX_USB_SETUP_LENGTH_MASK) >> 16);
//
//	/* Check for UVC Class Requests */
//	switch (bmReqType) {
//	case CY_FX_USB_UVC_GET_REQ_TYPE:
//	case CY_FX_USB_UVC_SET_REQ_TYPE:
//		/* UVC Specific requests are handled in the EP0 thread. */
//		switch (wIndex & 0xFF) {
//		case CY_FX_UVC_CONTROL_INTERFACE: {
//			uvcHandleReq = CyTrue;
//			status = CyU3PEventSet(&glFxUVCEvent,
//			CY_FX_UVC_VIDEO_CONTROL_REQUEST_EVENT,
//			CYU3P_EVENT_OR);
//			if (status != CY_U3P_SUCCESS) {
//				CyU3PDebugPrint(4,
//						"Set CY_FX_UVC_VIDEO_CONTROL_REQUEST_EVENT Failed %x\n",
//						status);
//				CyU3PUsbStall(0, CyTrue, CyFalse);
//			}
//		}
//			break;
//
//		case CY_FX_UVC_STREAM_INTERFACE: {
//			uvcHandleReq = CyTrue;
//			status = CyU3PEventSet(&glFxUVCEvent,
//			CY_FX_UVC_VIDEO_STREAM_REQUEST_EVENT,
//			CYU3P_EVENT_OR);
//			if (status != CY_U3P_SUCCESS) {
//				/* Error handling */
//				CyU3PDebugPrint(4,
//						"Set CY_FX_UVC_VIDEO_STREAM_REQUEST_EVENT Failed %x\n",
//						status);
//				CyU3PUsbStall(0, CyTrue, CyFalse);
//			}
//		}
//			break;
//
//		default:
//			break;
//		}
//		break;
//
//	case CY_FX_USB_SET_INTF_REQ_TYPE:
//		if (bRequest == CY_FX_USB_SET_INTERFACE_REQ) {
//			/* Some hosts send Set Interface Alternate Setting 0 command while stopping the video
//			 * stream. The application uses this event to stop streaming. */
//			if ((wIndex == CY_FX_UVC_STREAM_INTERFACE) && (wValue == 0)) {
//				/* Stop GPIF state machine to stop data transfers through FX3 */
//				CyU3PDebugPrint(4, "Alternate setting 0..\r\n");
//
//				CyU3PGpifDisable(CyFalse);
//				streamingStarted = CyFalse;
//
//				/* Place the EP in NAK mode before cleaning up the pipe. */
//				CyU3PUsbSetEpNak(CY_FX_EP_BULK_VIDEO, CyTrue);
//				CyU3PBusyWait(100);
//
//				/* Reset and flush the endpoint pipe. */
//				CyU3PDmaMultiChannelReset(&glChHandleUVCStream);
//				CyU3PUsbFlushEp(CY_FX_EP_BULK_VIDEO);
//				CyU3PUsbSetEpNak(CY_FX_EP_BULK_VIDEO, CyFalse);
//				CyU3PBusyWait(100);
//
//				/* Clear the stall condition and sequence numbers. */
//				CyU3PUsbStall(CY_FX_EP_BULK_VIDEO, CyFalse, CyTrue);
//				uvcHandleReq = CyTrue;
//
//				/* Complete Control request handshake */
//				CyU3PUsbAckSetup();
//
//				/* Indicate stop streaming to main thread */
//				clearFeatureRqtReceived = CyTrue;
//			//	CyFxUVCApplnAbortHandler();
//			}
//		}
//		break;
//
//	case CY_U3P_USB_TARGET_ENDPT:
//		if (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE) {
//			if (wIndex == CY_FX_EP_BULK_VIDEO) {
//				/* Windows OS sends Clear Feature Request after it stops streaming,
//				 * however MAC OS sends clear feature request right after it sends a
//				 * Commit -> SET_CUR request. Hence, stop streaming only if streaming
//				 * has started. */
//				if (streamingStarted == CyTrue) {
//					CyU3PDebugPrint(4, "Clear feature request detected..\r\n");
//
//					/* Disable the GPIF state machine. */
//					CyU3PGpifDisable(CyFalse);
//					streamingStarted = CyFalse;
//
//					/* Place the EP in NAK mode before cleaning up the pipe. */
//					CyU3PUsbSetEpNak(CY_FX_EP_BULK_VIDEO, CyTrue);
//					CyU3PBusyWait(100);
//
//					/* Reset and flush the endpoint pipe. */
//					CyU3PDmaMultiChannelReset(&glChHandleUVCStream);
//					CyU3PUsbFlushEp(CY_FX_EP_BULK_VIDEO);
//					CyU3PUsbSetEpNak(CY_FX_EP_BULK_VIDEO, CyFalse);
//					CyU3PBusyWait(100);
//
//					/* Clear the stall condition and sequence numbers. */
//					CyU3PUsbStall(CY_FX_EP_BULK_VIDEO, CyFalse, CyTrue);
//
//					uvcHandleReq = CyTrue;
//					/* Complete Control request handshake */
//					CyU3PUsbAckSetup();
//					/* Indicate stop streaming to main thread */
//					clearFeatureRqtReceived = CyTrue;
//					CyFxUVCApplnAbortHandler();
//				} else {
//					uvcHandleReq = CyTrue;
//					CyU3PUsbAckSetup();
//				}
//			}
//		}
//		break;
//
//	default:
//		break;
//	}
//
//	/* Return status of request handling to the USB driver */
//	return uvcHandleReq;
//}

///* This is the Callback function to handle the USB Events */
//static void CyFxUVCApplnUSBEventCB(CyU3PUsbEventType_t evtype, /* Event type */
//uint16_t evdata /* Event data */
//) {
//
//	switch (evtype) {
//	case CY_U3P_USB_EVENT_RESET:
//		ztex_log("RESET encountered...\r\n");
//		CyU3PDebugPrint(4, "RESET encountered...\r\n");
//		CyU3PGpifDisable(CyFalse);
//		streamingStarted = CyFalse;
//		CyFxUVCApplnAbortHandler();
//		break;
//
//	case CY_U3P_USB_EVENT_SUSPEND:
//		ztex_log("SUSPEND encountered...\r\n");
//		CyU3PDebugPrint(4, "SUSPEND encountered...\r\n");
//		CyU3PGpifDisable(CyFalse);
//		streamingStarted = CyFalse;
//		CyFxUVCApplnAbortHandler();
//		break;
//
//	case CY_U3P_USB_EVENT_DISCONNECT:
//		ztex_log("RESET encountered...\r\n");
//		CyU3PDebugPrint(4, "RESET disconnected...\r\n");
//		CyU3PGpifDisable(CyFalse);
//		streamingStarted = CyFalse;
//		CyFxUVCApplnAbortHandler();
//		break;
//
//	case CY_U3P_USB_EVENT_EP_UNDERRUN:
//		ztex_log("CY_U3P_USB_EVENT_EP_UNDERRUN encountered...\r\n");
//		CyU3PDebugPrint(4, "CY_U3P_USB_EVENT_EP_UNDERRUN encountered...\r\n");
//		break;
//
//	default:
//		break;
//	}
//}

/* DMA callback providing notification when data buffers are received from the sensor and when they have
 * been drained by the USB host.
 *
 * The UVC headers are attached to the data, and forwarded to the USB host in this callback function.
 */
void CyFxUvcApplnDmaCallback(CyU3PDmaMultiChannel *chHandle,
		CyU3PDmaCbType_t type, CyU3PDmaCBInput_t *input) {
	CyU3PDmaBuffer_t dmaBuffer;
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;


	if (type == CY_U3P_DMA_CB_PROD_EVENT) {
		/* This is a produce event notification to the CPU. This notification is received upon reception of
		 * every buffer. The buffer will not be sent out unless it is explicitly committed. The call shall fail
		 * if there is a bus reset / usb disconnect or if there is any application error.
		 */

		/* There is a possibility that CyU3PDmaMultiChannelGetBuffer will return CY_U3P_ERROR_INVALID_SEQUENCE here.
		 * In such a case, do nothing. We make up for this missed produce event by making repeated commit actions
		 * in subsequent produce event callbacks.
		 */
		status = CyU3PDmaMultiChannelGetBuffer(chHandle, &dmaBuffer,
		CYU3P_NO_WAIT);
		while (status == CY_U3P_SUCCESS) {
			/* Add Headers*/
			if (dmaBuffer.count == CY_FX_UVC_BUF_FULL_SIZE) {
				/* A full buffer indicates there is more data to go in this video frame. */
				CyFxUVCAddHeader(dmaBuffer.buffer - CY_FX_UVC_MAX_HEADER,
				CY_FX_UVC_HEADER_FRAME);
			} else {
				/* A partially filled buffer indicates the end of the ongoing video frame. */
				CyFxUVCAddHeader(dmaBuffer.buffer - CY_FX_UVC_MAX_HEADER,
				CY_FX_UVC_HEADER_EOF);

				glFrameCount++;
				glDmaDone = 0;

			}

			/* Commit Buffer to USB*/
			status = CyU3PDmaMultiChannelCommitBuffer(chHandle,
					(dmaBuffer.count + CY_FX_UVC_MAX_HEADER), 0);
//			/ZTEX_LOG("COMMIT BUFFER = %d",status);
			if (status == CY_U3P_SUCCESS) {

				glDmaDone++;

			} else {
				ztex_log("Error in CyU3PDmaMultiChannelCommitBuffer \r\n");
				CyU3PDebugPrint(4,
						"Error in CyU3PDmaMultiChannelCommitBuffer: code %d\r\n",
						status);
				break;
			}

			/* Check if any more buffers are ready to go, and commit them here. */
			status = CyU3PDmaMultiChannelGetBuffer(chHandle, &dmaBuffer,
			CYU3P_NO_WAIT);
		}
	} else if (type == CY_U3P_DMA_CB_CONS_EVENT) {
		streamingStarted = CyTrue;
	}
}

/* GpifCB callback function is invoked when FV triggers GPIF interrupt */
void CyFxGpifCB(uint8_t currentState /* GPIF state which triggered the interrupt. */
) {
	/* The ongoing video frame has ended. If we have a partial buffer sitting on the socket, we need to forcibly
	 * wrap it up. We also need to toggle the FW_TRG a couple of times to get the state machine ready for the
	 * next frame.
	 *
	 * Note: DMA channel APIs cannot be used here as this is ISR context. We are making use of the raw socket
	 * APIs.
	 */
	//ztex_log("CyFxGpifCB");
	switch (currentState) {
	case PARTIAL_BUF_IN_SCK0:
		//ztex_log(" PARTIAL_BUF_IN_SCK0 ");
		CyU3PDmaSocketSetWrapUp(CY_U3P_PIB_SOCKET_0);
		break;
	case FULL_BUF_IN_SCK0:
		//ztex_log(" FULL_BUF_IN_SCK0 ");
		break;
	case PARTIAL_BUF_IN_SCK1:
		//ztex_log(" PARTIAL_BUF_IN_SCK1 ");
		CyU3PDmaSocketSetWrapUp(CY_U3P_PIB_SOCKET_1);
		break;
	case FULL_BUF_IN_SCK1:
	//	ztex_log(" FULL_BUF_IN_SCK1 ");
		break;

	default:
		ztex_log("default");
		/* This should not happen. Do nothing. */
		return;
	}

	CyU3PGpifControlSWInput(CyTrue);
	CyU3PGpifControlSWInput(CyFalse);
}

/* This function initializes the Debug Module for the UVC Application */
static void CyFxUVCApplnDebugInit(void) {
	CyU3PUartConfig_t uartConfig;
	CyU3PReturnStatus_t apiRetStatus;

	/* Initialize the UART for printing debug messages */
	apiRetStatus = CyU3PUartInit();
	if (apiRetStatus != CY_U3P_SUCCESS) {
		ztex_log("UART failed");
		CyU3PDebugPrint(4, "UART initialization failed!\n");
		CyFxAppErrorHandler(apiRetStatus);
	}

	/* Set UART Configuration */
	uartConfig.baudRate = CY_U3P_UART_BAUDRATE_115200;
	uartConfig.stopBit = CY_U3P_UART_ONE_STOP_BIT;
	uartConfig.parity = CY_U3P_UART_NO_PARITY;
	uartConfig.txEnable = CyTrue;
	uartConfig.rxEnable = CyFalse;
	uartConfig.flowCtrl = CyFalse;
	uartConfig.isDma = CyTrue;

	/* Set the UART configuration */
	apiRetStatus = CyU3PUartSetConfig(&uartConfig, NULL);
	if (apiRetStatus != CY_U3P_SUCCESS) {
		CyFxAppErrorHandler(apiRetStatus);
	}

	/* Set the UART transfer */
	apiRetStatus = CyU3PUartTxSetBlockXfer(0xFFFFFFFF);
	if (apiRetStatus != CY_U3P_SUCCESS) {
		CyFxAppErrorHandler(apiRetStatus);
	}

	/* Initialize the Debug logger module. */
	apiRetStatus = CyU3PDebugInit(CY_U3P_LPP_SOCKET_UART_CONS, 4);
	if (apiRetStatus != CY_U3P_SUCCESS) {
		CyFxAppErrorHandler(apiRetStatus);
	}

	/* Disable log message headers. */
	CyU3PDebugPreamble(CyFalse);
}





///* I2C initialization. */
//static void CyFxUVCApplnI2CInit(void) {
//	CyU3PI2cConfig_t i2cConfig;
//
//	CyU3PReturnStatus_t status;
//
//	status = CyU3PI2cInit();
//	if (status != CY_U3P_SUCCESS) {
//		ztex_log("I2C init failed");
//		CyU3PDebugPrint(4, "I2C initialization failed!\n");
//		CyFxAppErrorHandler(status);
//	}
//
//	/*  Set I2C Configuration */
//	i2cConfig.bitRate = 100000; /*  100 KHz */
//	i2cConfig.isDma = CyFalse;
//	i2cConfig.busTimeout = 0xffffffffU;
//	i2cConfig.dmaTimeout = 0xffff;
//
//	status = CyU3PI2cSetConfig(&i2cConfig, 0);
//	if (CY_U3P_SUCCESS != status) {
//		ztex_log("I2C config failed");
//		CyU3PDebugPrint(4, "I2C configuration failed!\n");
//		CyFxAppErrorHandler(status);
//	}
//}

#ifdef BACKFLOW_DETECT
static void CyFxUvcAppPibCallback (
		CyU3PPibIntrType cbType,
		uint16_t cbArg)
{
	if ((cbType == CYU3P_PIB_INTR_ERROR) && ((cbArg == 0x1005) || (cbArg == 0x1006)))
	{
		if (!back_flow_detected)
		{
			ztex_log("Backflow detected...\r\n");
			CyU3PDebugPrint (4, "Backflow detected...\r\n");
			back_flow_detected = 1;
		}
	}
}
#endif

/*
 * Load the GPIF configuration on the GPIF-II engine. This operation is performed at start-up.
 */
static void CyFxUvcAppGpifInit(void) {
	CyU3PReturnStatus_t apiRetStatus;

	apiRetStatus = CyU3PGpifLoad((CyU3PGpifConfig_t *) &CyFxGpifConfig);
	if (apiRetStatus != CY_U3P_SUCCESS) {
		ztex_log("Loading GPIF configuration failed");
	}
}

/* This function initializes the USB Module, creates event group,
 sets the enumeration descriptors, configures the Endpoints and
 configures the DMA module for the UVC Application */
static void CyFxUVCApplnInit(void) {
	CyU3PDmaMultiChannelConfig_t dmaMultiConfig;
	CyU3PDmaChannelConfig_t channelConfig;
	CyU3PEpConfig_t endPointConfig;
	CyU3PReturnStatus_t apiRetStatus;
	CyU3PGpioClock_t gpioClock;
	CyU3PGpioSimpleConfig_t gpioConfig;

	/* Create UVC event group */
	apiRetStatus = CyU3PEventCreate(&glFxUVCEvent);
	if (apiRetStatus != 0) {
		ztex_log("UVC Create Event failed");
		CyU3PDebugPrint(4, "UVC Create Event failed, Error Code = %d\n",
				apiRetStatus);
		CyFxAppErrorHandler(apiRetStatus);
	}

#ifdef UVC_PTZ_SUPPORT
	CyFxUvcAppPTZInit ();
#endif

	clearFeatureRqtReceived = CyFalse;
//
//    /* Init the GPIO module */
//		gpioClock.fastClkDiv = 2;
//		gpioClock.slowClkDiv = 2;
//		gpioClock.simpleDiv  = CY_U3P_GPIO_SIMPLE_DIV_BY_2;
//		gpioClock.clkSrc     = CY_U3P_SYS_CLK;
//		gpioClock.halfDiv    = 0;
////
////    /* Initialize Gpio interface */
//    apiRetStatus = CyU3PGpioInit (&gpioClock, NULL);
//    if (apiRetStatus != 0)
//    {
//        CyU3PDebugPrint (4, "GPIO Init failed, Error Code = %d\n", apiRetStatus);
//        CyFxAppErrorHandler (apiRetStatus);
//    }

//    /* CTL pins are restricted and cannot be configured using I/O matrix configuration function,
//     * must use GpioOverride to configure it */
//    apiRetStatus = CyU3PDeviceGpioOverride (SENSOR_RESET_GPIO, CyTrue);
//    if (apiRetStatus != 0)
//    {
//        CyU3PDebugPrint (4, "GPIO Override failed, Error Code = %d\n", apiRetStatus);
//        CyFxAppErrorHandler (apiRetStatus);
//    }
//
//    /* SENSOR_RESET_GPIO is the Sensor reset pin */
//    gpioConfig.outValue    = CyTrue;
//    gpioConfig.driveLowEn  = CyTrue;
//    gpioConfig.driveHighEn = CyTrue;
//    gpioConfig.inputEn     = CyFalse;
//    gpioConfig.intrMode    = CY_U3P_GPIO_NO_INTR;
//    apiRetStatus           = CyU3PGpioSetSimpleConfig (SENSOR_RESET_GPIO, &gpioConfig);
//    if (apiRetStatus != CY_U3P_SUCCESS)
//    {
//        CyU3PDebugPrint (4, "GPIO Set Config Error, Error Code = %d\n", apiRetStatus);
//        CyFxAppErrorHandler (apiRetStatus);
//    }

	/* Initialize the P-port. */
	pibclock.clkDiv = 2;
	pibclock.clkSrc = CY_U3P_SYS_CLK;
	pibclock.isDllEnable = CyFalse;
	pibclock.isHalfDiv = CyFalse;
	apiRetStatus = CyU3PPibInit(CyTrue, &pibclock);
	if (apiRetStatus != CY_U3P_SUCCESS) {
		ztex_log("PIB Function Failed to Start");
		CyU3PDebugPrint(4, "PIB Function Failed to Start, Error Code = %d\n",
				apiRetStatus);
		CyFxAppErrorHandler(apiRetStatus);
	}
	CyFxUvcAppGpifInit();
	//ztex_log("Exiting GPIF INIT");

	/* Register the GPIF State Machine callback used to get frame end notifications.
	 * We use the fast callback version which is triggered from ISR context.
	 */
	CyU3PGpifRegisterSMIntrCallback(CyFxGpifCB);
	//ztex_log("Exiting SM INIT");

#ifdef BACKFLOW_DETECT
	back_flow_detected = 0;
	CyU3PPibRegisterCallback (CyFxUvcAppPibCallback, CYU3P_PIB_INTR_ERROR);
#endif

	/* Image sensor initialization. Reset and then initialize with appropriate configuration. */
	CyU3PThreadSleep(100);
	//SensorReset ();
	//SensorInit ();

	/* USB initialization. */
//    apiRetStatus = CyU3PUsbStart ();
//    if (apiRetStatus != CY_U3P_SUCCESS)
//    {
//    	ztex_log("USB failed");
//        CyU3PDebugPrint (4, "USB Function Failed to Start, Error Code = %d\n", apiRetStatus);
//        CyFxAppErrorHandler (apiRetStatus);
//    }
	/* Setup the Callback to Handle the USB Setup Requests */
	//CyU3PUsbRegisterSetupCallback (CyFxUVCApplnUSBSetupCB, CyFalse);
	/* Setup the Callback to Handle the USB Events */
	//CyU3PUsbRegisterEventCallback (CyFxUVCApplnUSBEventCB);
	/* Configure the video streaming endpoint. */
	endPointConfig.enable = 1;
	endPointConfig.epType = CY_U3P_USB_EP_BULK;
	endPointConfig.pcktSize = CY_FX_EP_BULK_VIDEO_PKT_SIZE;
	endPointConfig.isoPkts = 1;
	endPointConfig.burstLen = 16;
	endPointConfig.streams = 0;
	apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_BULK_VIDEO, &endPointConfig);
	if (apiRetStatus != CY_U3P_SUCCESS) {
		ztex_log("USB Set Endpoint config failed");
		/* Error Handling */
		CyU3PDebugPrint(4, "USB Set Endpoint config failed, Error Code = %d\n",
				apiRetStatus);
		CyFxAppErrorHandler(apiRetStatus);
	}

	/* Configure the status interrupt endpoint.
	 Note: This endpoint is not being used by the application as of now. This can be used in case
	 UVC device needs to notify the host about any error conditions. A MANUAL_OUT DMA channel
	 can be associated with this endpoint and used to send these data packets.
	 */
	endPointConfig.enable = 1;
	endPointConfig.epType = CY_U3P_USB_EP_INTR;
	endPointConfig.pcktSize = 64;
	endPointConfig.isoPkts = 0;
	endPointConfig.streams = 0;
	endPointConfig.burstLen = 1;
	apiRetStatus = CyU3PSetEpConfig(CY_FX_EP_CONTROL_STATUS, &endPointConfig);
	if (apiRetStatus != CY_U3P_SUCCESS) {
		/* Error Handling */
		ztex_log("USB Set Endpoint config failed");
		CyU3PDebugPrint(4, "USB Set Endpoint config failed, Error Code = %d\n",
				apiRetStatus);
		CyFxAppErrorHandler(apiRetStatus);
	}


	dmaMultiConfig.size = CY_FX_UVC_STREAM_BUF_SIZE;
	dmaMultiConfig.count = CY_FX_UVC_STREAM_BUF_COUNT;
	dmaMultiConfig.validSckCount = 2;
	dmaMultiConfig.prodSckId[0] = (CyU3PDmaSocketId_t) CY_U3P_PIB_SOCKET_0;
	dmaMultiConfig.prodSckId[1] = (CyU3PDmaSocketId_t) CY_U3P_PIB_SOCKET_1;
	dmaMultiConfig.consSckId[0] = (CyU3PDmaSocketId_t) (CY_U3P_UIB_SOCKET_CONS_0 | CY_FX_EP_VIDEO_CONS_SOCKET);
	dmaMultiConfig.prodAvailCount = 0;
	dmaMultiConfig.prodHeader = 12; /* 12 byte UVC header to be added. */
	dmaMultiConfig.prodFooter = 4; /* 4 byte footer to compensate for the 12 byte header. */
	dmaMultiConfig.consHeader = 0;
	dmaMultiConfig.dmaMode = CY_U3P_DMA_MODE_BYTE;
	dmaMultiConfig.notification = CY_U3P_DMA_CB_PROD_EVENT
			| CY_U3P_DMA_CB_CONS_EVENT;
	dmaMultiConfig.cb = CyFxUvcApplnDmaCallback;
	apiRetStatus = CyU3PDmaMultiChannelCreate(&glChHandleUVCStream,
			CY_U3P_DMA_TYPE_MANUAL_MANY_TO_ONE, &dmaMultiConfig);

	if (apiRetStatus != CY_U3P_SUCCESS) {
		/* Error handling */
		ztex_log("DMA Channel Creation Failed, Error Code");
		CyU3PDebugPrint(4, "DMA Channel Creation Failed, Error Code = %d\n",
				apiRetStatus);
		CyFxAppErrorHandler(apiRetStatus);
	}

	apiRetStatus = CyU3PDmaSocketIsValidConsumer(CY_U3P_UIB_SOCKET_CONS_0 | CY_FX_EP_VIDEO_CONS_SOCKET);
	ZTEX_LOG("CY_U3P_UIB_SOCKET_CONS_0 | CY_FX_EP_VIDEO_CONS_SOCKET = %d",apiRetStatus);
	if(apiRetStatus != CY_U3P_SUCCESS) {
		ztex_log("Not a valid socket");
	}

	apiRetStatus = CyU3PDmaSocketIsValidProducer(CY_U3P_PIB_SOCKET_0);
	if(apiRetStatus != CY_U3P_SUCCESS) {
		ztex_log("CY_U3P_PIB_SOCKET_0 not a valid socket");
	}

	apiRetStatus = CyU3PDmaSocketIsValidProducer(CY_U3P_PIB_SOCKET_1);
	if(apiRetStatus != CY_U3P_SUCCESS) {
		ztex_log("CY_U3P_PIB_SOCKET_0 not a valid socket");
	}

	/* Enable USB connection from the FX3 device, preferably at USB 3.0 speed. */
//    apiRetStatus = CyU3PConnectState (CyTrue, CyTrue);
//    if (apiRetStatus != CY_U3P_SUCCESS)
//    {
//
//        CyU3PDebugPrint (4, "USB Connect failed, Error Code = %d\n", apiRetStatus);
//        CyFxAppErrorHandler (apiRetStatus);
//    }
}

/*
 * Entry function for the UVC Application Thread
 */
void UVCAppThread_Entry(uint32_t input) {
	ztex_log("				UVCAppThread_Entry");
	CyU3PReturnStatus_t apiRetStatus;
	uint32_t flag;

	/* Initialize the Uart Debug Module */
	CyFxUVCApplnDebugInit();
	/* Initialize the I2C interface */
	//CyFxUVCApplnI2CInit ();
	/* Initialize the UVC Application */
	CyFxUVCApplnInit();

	/*
	 The actual data forwarding from sensor to USB host is done from the DMA and GPIF callback
	 functions. The thread is only responsible for checking for streaming start/stop conditions.

	 The CY_FX_UVC_STREAM_EVENT event flag will indicate that the UVC video stream should be started.

	 The CY_FX_UVC_STREAM_ABORT_EVENT event indicates that we need to abort the video streaming. This
	 only happens when we receive a CLEAR_FEATURE request indicating that streaming is to be stopped,
	 or when we have a critical error in the data path.
	 */
	for (;;) {
		apiRetStatus = CyU3PEventGet(&glFxUVCEvent,
				CY_FX_UVC_STREAM_ABORT_EVENT | CY_FX_UVC_STREAM_EVENT,
				CYU3P_EVENT_OR_CLEAR, &flag, LOOP_TIMEOUT);
		if (apiRetStatus == CY_U3P_SUCCESS) {
			if ((flag & CY_FX_UVC_STREAM_EVENT) != 0) {

				glDmaDone = 1;
				glFrameCount = 0;
				//ztex_log("START REQUEST");
				/* Request to start video stream. */

				ztex_log("START STREAM");
				/* Start with frame ID 0. */
				glUVCHeader[1] &= ~CY_FX_UVC_HEADER_FRAME_ID;

				/* Make sure we return to an active USB link state and stay there. */
				CyU3PUsbLPMDisable();
				if (CyU3PUsbGetSpeed() == CY_U3P_SUPER_SPEED) {
					CyU3PUsbSetLinkPowerState(CyU3PUsbLPM_U0);
					CyU3PBusyWait(200);
				} else {
					CyU3PUsb2Resume();
				}

				/* Set DMA Channel transfer size, first producer socket */
				apiRetStatus = CyU3PDmaMultiChannelSetXfer(&glChHandleUVCStream,0, 0);
				//ZTEX_LOG("DMA transfer started STATUS CODE = %d",apiRetStatus)
				if (apiRetStatus != CY_U3P_SUCCESS) {
					ztex_log("DMA Channel Set Transfer Failed");
					/* Error handling */
					CyU3PDebugPrint(4,"DMA Channel Set Transfer Failed, Error Code = %d\r\n",apiRetStatus);
					CyFxAppErrorHandler(apiRetStatus);
				}

				/* Start the state machine from the designated start state. */
				apiRetStatus = CyU3PGpifSMStart(START_SCK0, ALPHA_START_SCK0);
				//ZTEX_LOG("Starting GPIF state machine STATUS CODE = %d",apiRetStatus)
				if (apiRetStatus != CY_U3P_SUCCESS) {
					ztex_log("Starting GPIF state machine failed");

					/* Error Handling */
		CyU3PDebugPrint(4,"Starting GPIF state machine failed, Error Code = %d\r\n",apiRetStatus);
					CyFxAppErrorHandler(apiRetStatus);
				}
			}
			/* Video stream abort requested. */
			if ((flag & CY_FX_UVC_STREAM_ABORT_EVENT) != 0) {
				glDmaDone = 1;
				glFrameCount = 0;
				ztex_log("ABORT STREAM");
				if (!clearFeatureRqtReceived) {
					apiRetStatus = CyU3PDmaMultiChannelReset(
							&glChHandleUVCStream);
					if (apiRetStatus != CY_U3P_SUCCESS) {
						CyFxAppErrorHandler(apiRetStatus);
					}

					/* Flush the Endpoint memory */
					CyU3PUsbFlushEp(CY_FX_EP_BULK_VIDEO);
				}

				clearFeatureRqtReceived = CyFalse;

				/* Allow USB low power link transitions at this stage. */
				CyU3PUsbLPMEnable();
			}
		}
		//ztex_log("				UVCAppThread_Exit");
	//	ZTEX_LOG("UVC completed: %d frames and %d buffers\r\n", glFrameCount,(glDmaDone != 0) ? (glDmaDone - 1) : 0);
	}
}

/*
 * Handler for control requests addressed to the Processing Unit.
 */
static void UVCHandleProcessingUnitRqts(void) {
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	uint16_t readCount;

	switch (wValue) {
	 case CY_FX_UVC_PU_BRIGHTNESS_CONTROL:
	            switch (bRequest)
	            {
	                case CY_FX_USB_UVC_GET_LEN_REQ: /* Length of brightness data = 1 byte. */
	                    glEp0Buffer[0] = 1;
	                    CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
	                    break;
	                case CY_FX_USB_UVC_GET_CUR_REQ: /* Current brightness value. */
	                    glEp0Buffer[0] = SensorGetBrightness ();
	                    CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
	                    break;
	                case CY_FX_USB_UVC_GET_MIN_REQ: /* Minimum brightness = 0. */
	                    glEp0Buffer[0] = 0;
	                    CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
	                    break;
	                case CY_FX_USB_UVC_GET_MAX_REQ: /* Maximum brightness = 255. */
	                    glEp0Buffer[0] = 255;
	                    CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
	                    break;
	                case CY_FX_USB_UVC_GET_RES_REQ: /* Resolution = 1. */
	                    glEp0Buffer[0] = 1;
	                    CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
	                    break;
	                case CY_FX_USB_UVC_GET_INFO_REQ: /* Both GET and SET requests are supported, auto modes not supported */
	                    glEp0Buffer[0] = 3;
	                    CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
	                    break;
	                case CY_FX_USB_UVC_GET_DEF_REQ: /* Default brightness value = 55. */
	                    glEp0Buffer[0] = 55;
	                    CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
	                    break;
	                case CY_FX_USB_UVC_SET_CUR_REQ: /* Update brightness value. */
	                    apiRetStatus = CyU3PUsbGetEP0Data (CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
	                            glEp0Buffer, &readCount);
	                    if (apiRetStatus == CY_U3P_SUCCESS)
	                    {
	                        SensorSetBrightness (glEp0Buffer[0]);
	                    }
	                    break;
	                default:
	                    CyU3PUsbStall (0, CyTrue, CyFalse);
	                    break;
	            }
	            break;



	case CY_FX_UVC_PU_CONTRAST_CONTROL:
			  switch (bRequest)
			  {
				  case CY_FX_USB_UVC_GET_LEN_REQ: /* Length of brightness data = 1 byte. */
					  glEp0Buffer[0] = 1;
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_GET_CUR_REQ: /* Current brightness value. */
					  glEp0Buffer[0] = SensorGetContrast();
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_GET_MIN_REQ: /* Minimum brightness = 0. */
					  glEp0Buffer[0] = 0;
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_GET_MAX_REQ: /* Maximum brightness = 255. */
					  glEp0Buffer[0] = 255;
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_GET_RES_REQ: /* Resolution = 1. */
					  glEp0Buffer[0] = 1;
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_GET_INFO_REQ: /* Both GET and SET requests are supported, auto modes not supported */
					  glEp0Buffer[0] = 3;
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_GET_DEF_REQ: /* Default brightness value = 55. */
					  glEp0Buffer[0] = 55;
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_SET_CUR_REQ: /* Update brightness value. */
					  apiRetStatus = CyU3PUsbGetEP0Data (CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
							  glEp0Buffer, &readCount);
					  if (apiRetStatus == CY_U3P_SUCCESS)
					  {
						  SensorSetContrast(glEp0Buffer[0]);
					  }
					  break;
				  default:
					  CyU3PUsbStall (0, CyTrue, CyFalse);
					  break;
			  }
			  break;



	  case CY_FX_UVC_PU_HUE_CONTROL:
			  switch (bRequest)
			  {
				  case CY_FX_USB_UVC_GET_LEN_REQ: /* Length of brightness data = 1 byte. */
					  glEp0Buffer[0] = 1;
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_GET_CUR_REQ: /* Current brightness value. */
					  glEp0Buffer[0] = SensorGetHue ();
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_GET_MIN_REQ: /* Minimum brightness = 0. */
					  glEp0Buffer[0] = 0;
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_GET_MAX_REQ: /* Maximum brightness = 255. */
					  glEp0Buffer[0] = 255;
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_GET_RES_REQ: /* Resolution = 1. */
					  glEp0Buffer[0] = 1;
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_GET_INFO_REQ: /* Both GET and SET requests are supported, auto modes not supported */
					  glEp0Buffer[0] = 3;
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_GET_DEF_REQ: /* Default brightness value = 55. */
					  glEp0Buffer[0] = 55;
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_SET_CUR_REQ: /* Update brightness value. */
					  apiRetStatus = CyU3PUsbGetEP0Data (CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
							  glEp0Buffer, &readCount);
					  if (apiRetStatus == CY_U3P_SUCCESS)
					  {
						  SensorSetHue (glEp0Buffer[0]);
					  }
					  break;
				  default:
					  CyU3PUsbStall (0, CyTrue, CyFalse);
					  break;
			  }
			  break;



	  case CY_FX_UVC_PU_SATURATION_CONTROL  :
			  switch (bRequest)
			  {
				  case CY_FX_USB_UVC_GET_LEN_REQ: /* Length of brightness data = 1 byte. */
					  glEp0Buffer[0] = 1;
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_GET_CUR_REQ: /* Current brightness value. */
					  glEp0Buffer[0] = SensorGetSaturation ();
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_GET_MIN_REQ: /* Minimum brightness = 0. */
					  glEp0Buffer[0] = 0;
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_GET_MAX_REQ: /* Maximum brightness = 255. */
					  glEp0Buffer[0] = 255;
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_GET_RES_REQ: /* Resolution = 1. */
					  glEp0Buffer[0] = 1;
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_GET_INFO_REQ: /* Both GET and SET requests are supported, auto modes not supported */
					  glEp0Buffer[0] = 3;
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_GET_DEF_REQ: /* Default brightness value = 55. */
					  glEp0Buffer[0] = 55;
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_SET_CUR_REQ: /* Update brightness value. */
					  apiRetStatus = CyU3PUsbGetEP0Data (CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
							  glEp0Buffer, &readCount);
					  if (apiRetStatus == CY_U3P_SUCCESS)
					  {
						  SensorSetSaturation (glEp0Buffer[0]);
					  }
					  break;
				  default:
					  CyU3PUsbStall (0, CyTrue, CyFalse);
					  break;
			  }
			  break;

	  case CY_FX_UVC_PU_SHARPNESS_CONTROL  :
			  switch (bRequest)
			  {
				  case CY_FX_USB_UVC_GET_LEN_REQ: /* Length of brightness data = 1 byte. */
					  glEp0Buffer[0] = 1;
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_GET_CUR_REQ: /* Current brightness value. */
					  glEp0Buffer[0] = SensorGetSharpness ();
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_GET_MIN_REQ: /* Minimum brightness = 0. */
					  glEp0Buffer[0] = 0;
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_GET_MAX_REQ: /* Maximum brightness = 255. */
					  glEp0Buffer[0] = 255;
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_GET_RES_REQ: /* Resolution = 1. */
					  glEp0Buffer[0] = 1;
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_GET_INFO_REQ: /* Both GET and SET requests are supported, auto modes not supported */
					  glEp0Buffer[0] = 3;
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_GET_DEF_REQ: /* Default brightness value = 55. */
					  glEp0Buffer[0] = 55;
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_SET_CUR_REQ: /* Update brightness value. */
					  apiRetStatus = CyU3PUsbGetEP0Data (CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
							  glEp0Buffer, &readCount);
					  if (apiRetStatus == CY_U3P_SUCCESS)
					  {
						  SensorSetSharpness (glEp0Buffer[0]);
					  }
					  break;
				  default:
					  CyU3PUsbStall (0, CyTrue, CyFalse);
					  break;
			  }
			  break;

	case CY_FX_UVC_PU_GAMMA_CONTROL  :
		  switch (bRequest)
		  {
			  case CY_FX_USB_UVC_GET_LEN_REQ: /* Length of brightness data = 1 byte. */
				  glEp0Buffer[0] = 1;
				  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
				  break;
			  case CY_FX_USB_UVC_GET_CUR_REQ: /* Current brightness value. */
				  glEp0Buffer[0] = SensorGetGamma ();
				  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
				  break;
			  case CY_FX_USB_UVC_GET_MIN_REQ: /* Minimum brightness = 0. */
				  glEp0Buffer[0] = 0;
				  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
				  break;
			  case CY_FX_USB_UVC_GET_MAX_REQ: /* Maximum brightness = 255. */
				  glEp0Buffer[0] = 255;
				  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
				  break;
			  case CY_FX_USB_UVC_GET_RES_REQ: /* Resolution = 1. */
				  glEp0Buffer[0] = 1;
				  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
				  break;
			  case CY_FX_USB_UVC_GET_INFO_REQ: /* Both GET and SET requests are supported, auto modes not supported */
				  glEp0Buffer[0] = 3;
				  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
				  break;
			  case CY_FX_USB_UVC_GET_DEF_REQ: /* Default brightness value = 55. */
				  glEp0Buffer[0] = 55;
				  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
				  break;
			  case CY_FX_USB_UVC_SET_CUR_REQ: /* Update brightness value. */
				  apiRetStatus = CyU3PUsbGetEP0Data (CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
						  glEp0Buffer, &readCount);
				  if (apiRetStatus == CY_U3P_SUCCESS)
				  {
					  SensorSetGamma(glEp0Buffer[0]);
				  }
				  break;
			  default:
				  CyU3PUsbStall (0, CyTrue, CyFalse);
				  break;
		  }
		  break;

	case CY_FX_UVC_PU_WHITE_BALANCE_COMPONENT_CONTROL  :
		  switch (bRequest)
		  {
			  case CY_FX_USB_UVC_GET_LEN_REQ: /* Length of brightness data = 1 byte. */
				  glEp0Buffer[0] = 1;
				  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
				  break;
			  case CY_FX_USB_UVC_GET_CUR_REQ: /* Current brightness value. */
				  glEp0Buffer[0] = SensorGetWhiteBalance ();
				  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
				  break;
			  case CY_FX_USB_UVC_GET_MIN_REQ: /* Minimum brightness = 0. */
				  glEp0Buffer[0] = 0;
				  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
				  break;
			  case CY_FX_USB_UVC_GET_MAX_REQ: /* Maximum brightness = 255. */
				  glEp0Buffer[0] = 255;
				  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
				  break;
			  case CY_FX_USB_UVC_GET_RES_REQ: /* Resolution = 1. */
				  glEp0Buffer[0] = 1;
				  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
				  break;
			  case CY_FX_USB_UVC_GET_INFO_REQ: /* Both GET and SET requests are supported, auto modes not supported */
				  glEp0Buffer[0] = 3;
				  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
				  break;
			  case CY_FX_USB_UVC_GET_DEF_REQ: /* Default brightness value = 55. */
				  glEp0Buffer[0] = 55;
				  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
				  break;
			  case CY_FX_USB_UVC_SET_CUR_REQ: /* Update brightness value. */
				  apiRetStatus = CyU3PUsbGetEP0Data (CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
						  glEp0Buffer, &readCount);
				  if (apiRetStatus == CY_U3P_SUCCESS)
				  {
					  SensorSetWhiteBalance(glEp0Buffer[0]);
				  }
				  break;
			  default:
				  CyU3PUsbStall (0, CyTrue, CyFalse);
				  break;
		  }
		  break;

	  case CY_FX_UVC_PU_BACKLIGHT_COMPENSATION_CONTROL  :
			  switch (bRequest)
			  {
				  case CY_FX_USB_UVC_GET_LEN_REQ: /* Length of brightness data = 1 byte. */
					  glEp0Buffer[0] = 1;
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_GET_CUR_REQ: /* Current brightness value. */
					  glEp0Buffer[0] = SensorGetBacklightComp ();
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_GET_MIN_REQ: /* Minimum brightness = 0. */
					  glEp0Buffer[0] = 0;
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_GET_MAX_REQ: /* Maximum brightness = 255. */
					  glEp0Buffer[0] = 255;
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_GET_RES_REQ: /* Resolution = 1. */
					  glEp0Buffer[0] = 1;
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_GET_INFO_REQ: /* Both GET and SET requests are supported, auto modes not supported */
					  glEp0Buffer[0] = 3;
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_GET_DEF_REQ: /* Default brightness value = 55. */
					  glEp0Buffer[0] = 55;
					  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
					  break;
				  case CY_FX_USB_UVC_SET_CUR_REQ: /* Update brightness value. */
					  apiRetStatus = CyU3PUsbGetEP0Data (CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
							  glEp0Buffer, &readCount);
					  if (apiRetStatus == CY_U3P_SUCCESS)
					  {
						  SensorSetBacklightComp(glEp0Buffer[0]);
					  }
					  break;
				  default:
					  CyU3PUsbStall (0, CyTrue, CyFalse);
					  break;
			  }
			  break;

	case CY_FX_UVC_PU_GAIN_CONTROL :
		  switch (bRequest)
		  {
			  case CY_FX_USB_UVC_GET_LEN_REQ: /* Length of brightness data = 1 byte. */
				  glEp0Buffer[0] = 1;
				  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
				  break;
			  case CY_FX_USB_UVC_GET_CUR_REQ: /* Current brightness value. */
				  glEp0Buffer[0] = SensorGetGain ();
				  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
				  break;
			  case CY_FX_USB_UVC_GET_MIN_REQ: /* Minimum brightness = 0. */
				  glEp0Buffer[0] = 0;
				  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
				  break;
			  case CY_FX_USB_UVC_GET_MAX_REQ: /* Maximum brightness = 255. */
				  glEp0Buffer[0] = 255;
				  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
				  break;
			  case CY_FX_USB_UVC_GET_RES_REQ: /* Resolution = 1. */
				  glEp0Buffer[0] = 1;
				  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
				  break;
			  case CY_FX_USB_UVC_GET_INFO_REQ: /* Both GET and SET requests are supported, auto modes not supported */
				  glEp0Buffer[0] = 3;
				  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
				  break;
			  case CY_FX_USB_UVC_GET_DEF_REQ: /* Default brightness value = 55. */
				  glEp0Buffer[0] = 55;
				  CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
				  break;
			  case CY_FX_USB_UVC_SET_CUR_REQ: /* Update brightness value. */
				  apiRetStatus = CyU3PUsbGetEP0Data (CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
						  glEp0Buffer, &readCount);
				  if (apiRetStatus == CY_U3P_SUCCESS)
				  {
					  SensorSetGain(glEp0Buffer[0]);
				  }
				  break;
			  default:
				  CyU3PUsbStall (0, CyTrue, CyFalse);
				  break;
		  }
		  break;

	        default:
	            /*
	             * Only the brightness control is supported as of now. Add additional code here to support
	             * other controls.
	             */
	            CyU3PUsbStall (0, CyTrue, CyFalse);
	            break;
	    }
	}
/*
 * Handler for control requests addressed to the UVC Camera Terminal unit.
 */
static void UVCHandleCameraTerminalRqts(void) {
#ifdef UVC_PTZ_SUPPORT
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	uint16_t readCount;
	uint16_t zoomVal;
	int32_t panVal, tiltVal;
	CyBool_t sendData = CyFalse;
#endif

	switch (wValue) {
#ifdef UVC_PTZ_SUPPORT
	case CY_FX_UVC_CT_ZOOM_ABSOLUTE_CONTROL:
	switch (bRequest)
	{
		case CY_FX_USB_UVC_GET_INFO_REQ:
		glEp0Buffer[0] = 3; /* Support GET/SET queries. */
		CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
		break;
		case CY_FX_USB_UVC_GET_CUR_REQ: /* Current zoom control value. */
		zoomVal = CyFxUvcAppGetCurrentZoom ();
		sendData = CyTrue;
		break;
		case CY_FX_USB_UVC_GET_MIN_REQ: /* Minimum zoom control value. */
		zoomVal = CyFxUvcAppGetMinimumZoom ();
		sendData = CyTrue;
		break;
		case CY_FX_USB_UVC_GET_MAX_REQ: /* Maximum zoom control value. */
		zoomVal = CyFxUvcAppGetMaximumZoom ();
		sendData = CyTrue;
		break;
		case CY_FX_USB_UVC_GET_RES_REQ: /* Resolution is one unit. */
		zoomVal = CyFxUvcAppGetZoomResolution ();
		sendData = CyTrue;
		break;
		case CY_FX_USB_UVC_GET_DEF_REQ: /* Default zoom setting. */
		zoomVal = CyFxUvcAppGetDefaultZoom ();
		sendData = CyTrue;
		break;
		case CY_FX_USB_UVC_SET_CUR_REQ:
		apiRetStatus = CyU3PUsbGetEP0Data (CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
				glEp0Buffer, &readCount);
		if (apiRetStatus == CY_U3P_SUCCESS)
		{
			zoomVal = (glEp0Buffer[0]) | (glEp0Buffer[1] << 8);
			CyFxUvcAppModifyZoom (zoomVal);
		}
		break;
		default:
		CyU3PUsbStall (0, CyTrue, CyFalse);
		break;
	}

	if (sendData)
	{
		/* Send the 2-byte data in zoomVal back to the USB host. */
		glEp0Buffer[0] = CY_U3P_GET_LSB (zoomVal);
		glEp0Buffer[1] = CY_U3P_GET_MSB (zoomVal);
		CyU3PUsbSendEP0Data (wLength, (uint8_t *)glEp0Buffer);
	}
	break;

	case CY_FX_UVC_CT_PANTILT_ABSOLUTE_CONTROL:
	switch (bRequest)
	{
		case CY_FX_USB_UVC_GET_INFO_REQ:
		glEp0Buffer[0] = 3; /* GET/SET requests supported for this control */
		CyU3PUsbSendEP0Data (1, (uint8_t *)glEp0Buffer);
		break;
		case CY_FX_USB_UVC_GET_CUR_REQ:
		panVal = CyFxUvcAppGetCurrentPan ();
		tiltVal = CyFxUvcAppGetCurrentTilt ();
		sendData = CyTrue;
		break;
		case CY_FX_USB_UVC_GET_MIN_REQ:
		panVal = CyFxUvcAppGetMinimumPan ();
		tiltVal = CyFxUvcAppGetMinimumTilt ();
		sendData = CyTrue;
		break;
		case CY_FX_USB_UVC_GET_MAX_REQ:
		panVal = CyFxUvcAppGetMaximumPan ();
		tiltVal = CyFxUvcAppGetMaximumTilt ();
		sendData = CyTrue;
		break;
		case CY_FX_USB_UVC_GET_RES_REQ:
		panVal = CyFxUvcAppGetPanResolution ();
		tiltVal = CyFxUvcAppGetTiltResolution ();
		sendData = CyTrue;
		break;
		case CY_FX_USB_UVC_GET_DEF_REQ:
		panVal = CyFxUvcAppGetDefaultPan ();
		tiltVal = CyFxUvcAppGetDefaultTilt ();
		sendData = CyTrue;
		break;
		case CY_FX_USB_UVC_SET_CUR_REQ:
		apiRetStatus = CyU3PUsbGetEP0Data (CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED,
				glEp0Buffer, &readCount);
		if (apiRetStatus == CY_U3P_SUCCESS)
		{
			panVal = (glEp0Buffer[0]) | (glEp0Buffer[1]<<8) |
			(glEp0Buffer[2]<<16) | (glEp0Buffer[2]<<24);
			tiltVal = (glEp0Buffer[4]) | (glEp0Buffer[5]<<8) |
			(glEp0Buffer[6]<<16) | (glEp0Buffer[7]<<24);

			CyFxUvcAppModifyPan (panVal);
			CyFxUvcAppModifyTilt (tiltVal);
		}
		break;
		default:
		CyU3PUsbStall (0, CyTrue, CyFalse);
		break;
	}

	if (sendData)
	{
		/* Send the 8-byte PAN and TILT values back to the USB host. */
		glEp0Buffer[0] = CY_U3P_DWORD_GET_BYTE0 (panVal);
		glEp0Buffer[1] = CY_U3P_DWORD_GET_BYTE1 (panVal);
		glEp0Buffer[2] = CY_U3P_DWORD_GET_BYTE2 (panVal);
		glEp0Buffer[3] = CY_U3P_DWORD_GET_BYTE3 (panVal);
		glEp0Buffer[4] = CY_U3P_DWORD_GET_BYTE0 (tiltVal);
		glEp0Buffer[5] = CY_U3P_DWORD_GET_BYTE1 (tiltVal);
		glEp0Buffer[6] = CY_U3P_DWORD_GET_BYTE2 (tiltVal);
		glEp0Buffer[7] = CY_U3P_DWORD_GET_BYTE3 (tiltVal);
		CyU3PUsbSendEP0Data (wLength, (uint8_t *)glEp0Buffer);
	}
	break;
#endif

	default:
		CyU3PUsbStall(0, CyTrue, CyFalse);
		break;
	}
}

/*
 * Handler for UVC Interface control requests.
 */
static void UVCHandleInterfaceCtrlRqts(void) {
	/* No requests supported as of now. Just stall EP0 to fail the request. */
	CyU3PUsbStall(0, CyTrue, CyFalse);
}

/*
 * Handler for control requests addressed to the Extension Unit.
 */
static void UVCHandleExtensionUnitRqts(void) {
	/* No requests supported as of now. Just stall EP0 to fail the request. */
	CyU3PUsbStall(0, CyTrue, CyFalse);
}

/*
 * Handler for the video streaming control requests.
 */
static void UVCHandleVideoStreamingRqts(void) {
	//ztex_log("UVCHandleVideoStreamingRqts");
	CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
	uint16_t readCount;

	switch (wValue) {
	case CY_FX_UVC_PROBE_CTRL:
		switch (bRequest) {
		case CY_FX_USB_UVC_GET_INFO_REQ:
			glEp0Buffer[0] = 3; /* GET/SET requests are supported. */
			CyU3PUsbSendEP0Data(1, (uint8_t *) glEp0Buffer);
			break;
		case CY_FX_USB_UVC_GET_LEN_REQ:
			glEp0Buffer[0] = CY_FX_UVC_MAX_PROBE_SETTING;
			CyU3PUsbSendEP0Data(1, (uint8_t *) glEp0Buffer);
			break;
		case CY_FX_USB_UVC_GET_CUR_REQ:
		case CY_FX_USB_UVC_GET_MIN_REQ:
		case CY_FX_USB_UVC_GET_MAX_REQ:
		case CY_FX_USB_UVC_GET_DEF_REQ: /* There is only one setting per USB speed. */
			if (usbSpeed == CY_U3P_SUPER_SPEED) {
				CyU3PUsbSendEP0Data(CY_FX_UVC_MAX_PROBE_SETTING,
						(uint8_t *) glProbeCtrl);
			} else {
				//ztex_log("sending ep0 data");
				CyU3PUsbSendEP0Data(CY_FX_UVC_MAX_PROBE_SETTING,
						(uint8_t *) glProbeCtrl20);
			}
			break;
		case CY_FX_USB_UVC_SET_CUR_REQ:
			apiRetStatus = CyU3PUsbGetEP0Data(
			CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED, glCommitCtrl, &readCount);
			if (apiRetStatus == CY_U3P_SUCCESS) {
				if (usbSpeed == CY_U3P_SUPER_SPEED) {
					/* Copy the relevant settings from the host provided data into the
					 active data structure. */
					glProbeCtrl[2] = glCommitCtrl[2];
					glProbeCtrl[3] = glCommitCtrl[3];
					glProbeCtrl[4] = glCommitCtrl[4];
					glProbeCtrl[5] = glCommitCtrl[5];
					glProbeCtrl[6] = glCommitCtrl[6];
					glProbeCtrl[7] = glCommitCtrl[7];
				}
			}
			break;
		default:
			CyU3PUsbStall(0, CyTrue, CyFalse);
			break;
		}
		break;

	case CY_FX_UVC_COMMIT_CTRL:
		switch (bRequest) {
		case CY_FX_USB_UVC_GET_INFO_REQ:
			glEp0Buffer[0] = 3; /* GET/SET requests are supported. */
			CyU3PUsbSendEP0Data(1, (uint8_t *) glEp0Buffer);
			break;
		case CY_FX_USB_UVC_GET_LEN_REQ:
			glEp0Buffer[0] = CY_FX_UVC_MAX_PROBE_SETTING;
			CyU3PUsbSendEP0Data(1, (uint8_t *) glEp0Buffer);
			break;
		case CY_FX_USB_UVC_GET_CUR_REQ:
			if (usbSpeed == CY_U3P_SUPER_SPEED) {
				CyU3PUsbSendEP0Data(CY_FX_UVC_MAX_PROBE_SETTING,
						(uint8_t *) glProbeCtrl);
			} else {
				CyU3PUsbSendEP0Data(CY_FX_UVC_MAX_PROBE_SETTING,
						(uint8_t *) glProbeCtrl20);
			}
			break;
		case CY_FX_USB_UVC_SET_CUR_REQ:
			/* The host has selected the parameters for the video stream. Check the desired
			 resolution settings, configure the sensor and start the video stream.
			 */
			apiRetStatus = CyU3PUsbGetEP0Data(
			CY_FX_UVC_MAX_PROBE_SETTING_ALIGNED, glCommitCtrl, &readCount);
			if (apiRetStatus == CY_U3P_SUCCESS) {
				if (usbSpeed == CY_U3P_SUPER_SPEED) {
					SensorScaling_HD720p_30fps();
				} else {
					SensorScaling_VGA();
				}

				/* We can start streaming video now. */
				apiRetStatus = CyU3PEventSet(&glFxUVCEvent,
				CY_FX_UVC_STREAM_EVENT, CYU3P_EVENT_OR);
				if (apiRetStatus != CY_U3P_SUCCESS) {
					ztex_log("Set CY_FX_UVC_STREAM_EVENT failed %x\n");
					CyU3PDebugPrint(4, "Set CY_FX_UVC_STREAM_EVENT failed %x\n",
							apiRetStatus);
				}
			}
			break;

		default:
			CyU3PUsbStall(0, CyTrue, CyFalse);
			break;
		}
		break;

	default:
		CyU3PUsbStall(0, CyTrue, CyFalse);
		break;
	}
}

/*
 * Entry function for the UVC control request processing thread.
 */
void UVCAppEP0Thread_Entry(uint32_t input) {
	uint32_t eventMask = (CY_FX_UVC_VIDEO_CONTROL_REQUEST_EVENT
			| CY_FX_UVC_VIDEO_STREAM_REQUEST_EVENT);
	uint32_t eventFlag;

	ztex_log("                   UVCAppEP0Thread_Entry");
	for (;;) {
		/* Wait for a Video control or streaming related request on the control endpoint. */
		if (CyU3PEventGet (&glFxUVCEvent, eventMask, CYU3P_EVENT_OR_CLEAR, &eventFlag,
				CYU3P_WAIT_FOREVER) == CY_U3P_SUCCESS) {
			//ztex_log("             Wait for a Video control or streaming");
			usbSpeed = CyU3PUsbGetSpeed();
			//ZTEX_LOG("USB SPEED = %x",usbSpeed);

			if (eventFlag & CY_FX_UVC_VIDEO_CONTROL_REQUEST_EVENT)

			{
				//ztex_log("             Wait for a Video control ");
				switch ((wIndex >> 8)) {
				case CY_FX_UVC_PROCESSING_UNIT_ID:
					UVCHandleProcessingUnitRqts();
					break;

				case CY_FX_UVC_CAMERA_TERMINAL_ID:
					UVCHandleCameraTerminalRqts();
					break;

				case CY_FX_UVC_INTERFACE_CTRL:
					UVCHandleInterfaceCtrlRqts();
					break;

				case CY_FX_UVC_EXTENSION_UNIT_ID:
					UVCHandleExtensionUnitRqts();
					break;

				default:
					/* Unsupported request. Fail by stalling the control endpoint. */
					CyU3PUsbStall(0, CyTrue, CyFalse);
					break;
				}
			}

			if (eventFlag & CY_FX_UVC_VIDEO_STREAM_REQUEST_EVENT) {

				//ztex_log("             Wait for a STREAM request");
				if (wIndex != CY_FX_UVC_STREAM_INTERFACE) {
					CyU3PUsbStall(0, CyTrue, CyFalse);
				} else {
					UVCHandleVideoStreamingRqts();
				}
			}

		}

		/* Allow other ready threads to run. */
		CyU3PThreadRelinquish();
	//	ztex_log("                   UVCAppEP0Thread_Exit");
	}
}

CyU3PThread ztex_app_thread;	 		// ztex application thread structure

// entry function for the application thread
void ztex_app_thread_entry(uint32_t input) {
	ztex_debug_init();

	ZTEX_REC(CyU3PUsbStart());					// start USB functionality

	CyU3PUsbRegisterSetupCallback(ztex_ep0_handler, CyTrue);// register EP0 handler

	CyU3PUsbRegisterLPMRequestCallback(ztex_lpm_handler); // register link power management handler

	CyU3PUsbRegisterEventCallback(ztex_usb_event_handler); // register USB event handler

	// Set the USB registers
	ZTEX_REC(
			CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_DEVICE_DESCR, 0,
					(uint8_t * )ztex_usb3_device_descriptor)); // Super speed device descriptor
	ZTEX_REC(
			CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_DEVICE_DESCR, 0,
					(uint8_t * )ztex_usb2_device_descriptor)); // High speed device descriptor
	ZTEX_REC(
			CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_BOS_DESCR, 0,
					(uint8_t * )ztex_bos_descriptor)); 	// BOS descriptor
	ZTEX_REC(
			CyU3PUsbSetDesc(CY_U3P_USB_SET_DEVQUAL_DESCR, 0,
					(uint8_t * )ztex_device_qualifier_descriptor)); // Device qualifier descriptor
	ZTEX_REC(
			CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_CONFIG_DESCR, 0,
					(uint8_t * )ztex_usb3_config_descriptor)); // Super speed configuration descriptor
	ZTEX_REC(
			CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_CONFIG_DESCR, 0,
					(uint8_t * )ztex_usb2_config_descriptor)); // High speed configuration descriptor
	ZTEX_REC(
			CyU3PUsbSetDesc(CY_U3P_USB_SET_FS_CONFIG_DESCR, 0,
					(uint8_t * )ztex_usb1_config_descriptor)); // Full speed configuration descriptor
	CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 0,
			(uint8_t *) ztex_lang_string_descriptor); // String descriptor 0 must not be handled by ep0_handler for an undocumented reason

	ztex_gpio_init();

#ifdef _ZTEX_BOARD_
	ztex_board_init();
#endif    

#ifdef ENABLE_SPI_FLASH
	ztex_flash_init();
#endif    

#ifdef ENABLE_I2C
	ztex_i2c_init();
#endif

#ifdef _ZTEX_FLASH_CONFIG_FUNC_
	_ZTEX_FLASH_CONFIG_FUNC_
#endif

	// Connect the USB Pins with super speed operation enabled.
	ZTEX_REC(CyU3PConnectState(CyTrue, CyTrue));
	CyU3PThreadResume(&uvcAppThread);
	CyU3PThreadResume(&uvcAppEP0Thread);

	if (ztex_app_thread_run != 0)
		ztex_app_thread_run();

	for (;;) {
		CyU3PThreadSleep(1000);
	}
}

/* This function is expected and called by OS. */
void CyFxApplicationDefine() {
	/* create and start the application thread */
	if ( CyU3PThreadCreate(&ztex_app_thread, // ztex application thread structure
			"21:ztex_app_thread",// thread ID and name
			ztex_app_thread_entry,// entry function
			0,// no input parameter
			CyU3PMemAlloc(ztex_app_thread_stack),// allocate stack memory
			ztex_app_thread_stack,// stack size
			ztex_app_thread_prio,// thread priority
			ztex_app_thread_prio,// preempt threshold
			CYU3P_NO_TIME_SLICE,// no time slice as recommended
			CYU3P_AUTO_START// start the thread immediately
	) != 0)
		ztex_ec = 105;

	if ( CyU3PThreadCreate(&uvcAppThread, // ztex application thread structure
			"22:UVC App Thread",// thread ID and name
			UVCAppThread_Entry,// entry function
			0,// no input parameter
			CyU3PMemAlloc( UVC_APP_THREAD_STACK),// allocate stack memory
			UVC_APP_THREAD_STACK,// stack size
			UVC_APP_THREAD_PRIORITY,// thread priority
			UVC_APP_THREAD_PRIORITY,// preempt threshold
			CYU3P_NO_TIME_SLICE,// no time slice as recommended
			CYU3P_DONT_START// start the thread immediately
	) != 0)
		ztex_ec = 105;

	if ( CyU3PThreadCreate(&uvcAppEP0Thread, // ztex application thread structure
			"23:UVC App EP0 Thread",// thread ID and name
			UVCAppEP0Thread_Entry,// entry function
			0,// no input parameter
			CyU3PMemAlloc( UVC_APP_THREAD_STACK),// allocate stack memory
			UVC_APP_EP0_THREAD_STACK,// stack size
			UVC_APP_EP0_THREAD_PRIORITY,// thread priority
			UVC_APP_EP0_THREAD_PRIORITY,// preempt threshold
			CYU3P_NO_TIME_SLICE,// no time slice as recommended
			CYU3P_DONT_START// start the thread immediately
	) != 0)
		ztex_ec = 105;

}

/*
 * Main function. Configures the IO-Matrix and starts the OS
 */
void ztex_main(void) {
	ztex_ec = 101;

	/* Initialize the device */
	CyU3PSysClockConfig_t clock_cfg;
	clock_cfg.setSysClk400 = CyFalse;
	clock_cfg.cpuClkDiv = 2;
	clock_cfg.dmaClkDiv = 2;
	clock_cfg.mmioClkDiv = 2;
	clock_cfg.useStandbyClk = CyFalse;
	clock_cfg.clkSrc = CY_U3P_SYS_CLK;
	if (CyU3PDeviceInit(&clock_cfg) != CY_U3P_SUCCESS)
		ztex_ec = 102;

	/* Initialize the caches. Enable both Instruction and Data Caches. */
	if (CyU3PDeviceCacheControl(CyTrue, CyTrue, CyTrue) != CY_U3P_SUCCESS)
		ztex_ec = 103;

	CyU3PIoMatrixConfig_t io_cfg;
	CyU3PMemSet((uint8_t *) &io_cfg, 0, sizeof(io_cfg));
	io_cfg.isDQ32Bit = CyFalse;
#ifdef ENABLE_SPORT0
	io_cfg.s0Mode = CY_U3P_SPORT_4BIT;
#else    
	io_cfg.s0Mode = CY_U3P_SPORT_INACTIVE;
#endif
	io_cfg.s1Mode = CY_U3P_SPORT_INACTIVE;
#ifdef ENABLE_UART    
	io_cfg.useUart = CyFalse;
#else    
	io_cfg.useUart = CyTrue;
#endif    
#ifdef ENABLE_I2C
	io_cfg.useI2C = CyTrue;
#else
	io_cfg.useI2C = CyFalse;
#endif
	io_cfg.useI2S = CyFalse;
#ifdef ENABLE_SPI
	io_cfg.useSpi = CyTrue;
#else
	io_cfg.useSpi = CyFalse;
#endif
	io_cfg.lppMode = CY_U3P_IO_MATRIX_LPP_DEFAULT;
	/* No GPIOs are enabled. */
	io_cfg.gpioSimpleEn[0] = (ZTEX_GPIO_SIMPLE_BITMAP0 | GPIO_SIMPLE_BITMAP0)
			& (~(ZTEX_GPIO_COMPLEX_BITMAP0 | GPIO_COMPLEX_BITMAP0));
	io_cfg.gpioSimpleEn[1] = (ZTEX_GPIO_SIMPLE_BITMAP1 | GPIO_SIMPLE_BITMAP1)
			& (~(ZTEX_GPIO_COMPLEX_BITMAP1 | GPIO_COMPLEX_BITMAP1));
	io_cfg.gpioComplexEn[0] = ZTEX_GPIO_COMPLEX_BITMAP0 | GPIO_COMPLEX_BITMAP0;
	io_cfg.gpioComplexEn[1] = ZTEX_GPIO_COMPLEX_BITMAP1 | GPIO_COMPLEX_BITMAP1;
	if (CyU3PDeviceConfigureIOMatrix(&io_cfg) != CY_U3P_SUCCESS)
		ztex_ec = 104;

	/* This is a non returnable call for initializing the RTOS kernel */
	ztex_ec = 0;
	CyU3PKernelEntry();
}

