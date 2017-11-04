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
    Implementation of the Endpoint 0 functionality.
*/    

/* 
    return values of ztex_vendor_func:
     0     : success
     1..254: RTOS or API error (see cyu3error.h)
     255   : other error
*/    
uint8_t vendor_req_last = 0;				// last vendor request + 1
uint8_t vendor_cmd_last = 0;			// last vendor command + 1
uint8_t vendor_req_idx[ZTEX_VENDOR_REQ_MAX];		// indexes of vendor requests
uint8_t vendor_cmd_idx[ZTEX_VENDOR_CMD_MAX];    	// indexes of vendor commands
ztex_vendor_func vendor_req[ZTEX_VENDOR_REQ_MAX];	// indexes of vendor requests
ztex_vendor_func vendor_cmd[ZTEX_VENDOR_CMD_MAX];   	// indexes of vendor commands

void (*ztex_disable_flash_boot)() = 0;			// this is board specific and called to disable boot from flash 

uint8_t ztex_register_vendor_req(uint8_t idx, ztex_vendor_func f) {
    uint8_t i=0;
    while ( (i<vendor_req_last) && (vendor_req_idx[i]!=idx) ) i++;
    ZTEX_ASSERT_RET((i<ZTEX_VENDOR_REQ_MAX));
    vendor_req_idx[i]=idx;
    vendor_req[i]=f;
    if (i==vendor_req_last) vendor_req_last++;
    return 0;
}

uint8_t ztex_register_vendor_cmd(uint8_t idx, ztex_vendor_func f) {
    uint8_t i=0;
    while ( (i<vendor_cmd_last) && (vendor_cmd_idx[i]!=idx) ) i++;
    ZTEX_ASSERT_RET(i<ZTEX_VENDOR_CMD_MAX);
    vendor_cmd_idx[i]=idx;
    vendor_cmd[i]=f;
    if (i==vendor_cmd_last) vendor_cmd_last++;
    return 0;
}


uint8_t ztex_send_string_descriptor (char* str) {
    uint8_t l = 1;
    if ( str == NULL ) {
	ztex_ep0buf[0]=4;
	ztex_ep0buf[2]=0;
	ztex_ep0buf[3]=0;
    } else {
        l = strlen(str);
	ztex_ep0buf[0]=l*2+2;
	for ( uint8_t i = 0; i<l; i++ ) {
	    ztex_ep0buf[i*2+2] = str[i];
	    ztex_ep0buf[i*2+3] = 0;
	}
    }
    ztex_ep0buf[1]=CY_U3P_USB_STRING_DESCR;
    return CyU3PUsbSendEP0Data(l*2+2, ztex_ep0buf);
}
int cnt=0;
#define SEND_DESCR(d) ZTEX_REC(status=CyU3PUsbSendEP0Data (((uint8_t *)d)[0], (uint8_t *)d));
CyBool_t ztex_ep0_handler ( uint32_t setupdat0, uint32_t setupdat1)
{
    CyBool_t uvcHandleReq = CyFalse;
    // Decode the fields from the setup request. 

cnt=cnt+1;

		bmReqType = (uint8_t)(setupdat0 & CY_FX_USB_SETUP_REQ_TYPE_MASK);
		bRequest  = (uint8_t)((setupdat0 & CY_FX_USB_SETUP_REQ_MASK) >> 8);
		wValue    = (uint16_t)((setupdat0 & CY_FX_USB_SETUP_VALUE_MASK) >> 16);
		wIndex    = (uint16_t)(setupdat1 & CY_FX_USB_SETUP_INDEX_MASK);
		wLength   = (uint16_t)((setupdat1 & CY_FX_USB_SETUP_LENGTH_MASK) >> 16);
		bType     = (uint8_t)(bmReqType & CY_U3P_USB_TYPE_MASK);
		bTarget   = (uint8_t)(bmReqType & CY_U3P_USB_TARGET_MASK);
//if(cnt%5==0)
//{
//	    ZTEX_LOG("cnt .....%d",cnt);
//		ZTEX_LOG("bmReqType.....%x",bmReqType);
//		ZTEX_LOG("bRequest.....%x",bRequest);
//		ZTEX_LOG("wValue.....%x",wValue);
//		ZTEX_LOG("wIndex.....%x",wIndex);
//		ZTEX_LOG("wLength .....%x",wLength);
//		ZTEX_LOG("bType .....%d",bType);
//		ZTEX_LOG("bTarget .....%d",bTarget );
//		ZTEX_LOG("setupdat0 .....%d",setupdat0);
//		ZTEX_LOG("setupdat1 .....%d",setupdat1);
//		ZTEX_LOG("\n\n");
//
//}


//    uint8_t bRequestType = (setupdat0 & CY_U3P_USB_REQUEST_TYPE_MASK);
//    uint8_t bType = (bRequestType & CY_U3P_USB_TYPE_MASK);
//    uint8_t bTarget  = (bRequestType & CY_U3P_USB_TARGET_MASK);
//    uint8_t bRequest = ((setupdat0 & CY_U3P_USB_REQUEST_MASK) >> CY_U3P_USB_REQUEST_POS);
//    uint16_t wValue   = ((setupdat0 & CY_U3P_USB_VALUE_MASK) >> CY_U3P_USB_VALUE_POS);
//    uint16_t wIndex   = ((setupdat1 & CY_U3P_USB_INDEX_MASK) >> CY_U3P_USB_INDEX_POS);
//    uint16_t wIndex2   = (setupdat1 & CY_U3P_USB_INDEX_MASK);
//    uint16_t wLength   = ((setupdat1 & CY_U3P_USB_LENGTH_MASK) >> CY_U3P_USB_LENGTH_POS);
  //  uint8_t isHandled = 0;
    uint32_t status=0;

    // handle strings
    if ( bRequest==6 && (wValue>>8)==3 ) {  
	switch (wValue & 255) {
	    case 1: 
		ZTEX_REC(status = ztex_send_string_descriptor((char *)ztex_manufacturer_string));
                break;
            case 2:
		ZTEX_REC(status = ztex_send_string_descriptor((char *)ztex_product_string));
                break;
            case 3:
		ZTEX_REC(status = ztex_send_string_descriptor((char *)ztex_sn_string));
                break;
            default:
        	// 4..12 descriptions for interfaces 0..7
		ZTEX_REC(status = ztex_send_string_descriptor( (((wValue & 255) >= 4) && ((wValue & 255) < 12)) ? ztex_interface_string[(wValue & 255)-4] : NULL) );
                break;
	}

	uvcHandleReq=1;
//     isHandled = 1;
    }
    else if (bType == CY_U3P_USB_STANDARD_RQT) {
    	ztex_log("CY_U3P_USB_STANDARD_RQT");
    
        if ( (bTarget == CY_U3P_USB_TARGET_INTF) && ((bRequest == CY_U3P_USB_SC_SET_FEATURE) || (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)) && (wValue==0) ) {
            if ( ztex_usb_is_connected )
                CyU3PUsbAckSetup();
            else
                CyU3PUsbStall(0, CyTrue, CyFalse);
            uvcHandleReq=1;
           // isHandled = 1;
        }

        /* Clear stall requests for endpoint is always passed to the setup callback.
         * It's handled by ztex_ep_cleanup_handler */
        if ( (bTarget == CY_U3P_USB_TARGET_ENDPT) && (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)  && (wValue == CY_U3P_USBX_FS_EP_HALT))
        {
            if ( ztex_usb_is_connected )
            {
        	ztex_ep_cleanup_handler(wIndex);
                CyU3PUsbStall (wIndex, CyFalse, CyTrue);
                CyU3PUsbAckSetup ();
                uvcHandleReq= CyTrue;
                //isHandled = CyTrue;
            }
        }

    }
    // Handle vendor requests.
    else if (bmReqType == 0xc0 ) {
    	  uvcHandleReq= 0;
        //isHandled = 0;
        switch (bRequest)
        {
            case 0x22:		// send ZTEX descriptor 
        	SEND_DESCR(ztex_descriptor);

        	  uvcHandleReq= 1;
    		//isHandled = 1;
                break;

            default:
        	for ( int i=0; i<vendor_req_last; i++ ) {
        	    if ( bRequest==vendor_req_idx[i] ) {
        		status = (vendor_req[i])(wValue, wIndex, wLength);
        		  uvcHandleReq= 1;
        		//isHandled = 1;
        	    }
        	}
                // unknown request 
                break;
        }
    }
    // Handle vendor command
    else if (bmReqType == 0x40 ) {
    	  uvcHandleReq= 0;
       // isHandled = 0;
        switch (bRequest)
        {
            case 0xA1:		// system reset
        	if ( (wValue == 0) && (ztex_disable_flash_boot != 0) ) ztex_disable_flash_boot();
        	CyU3PDeviceReset(CyFalse);
        	  uvcHandleReq= 1;
    		//isHandled = 1;
                break;
            default:
        	for ( int i=0; i<vendor_cmd_last; i++ ) {
        	    if ( bRequest==vendor_cmd_idx[i] ) {
        		status = (vendor_cmd[i])(wValue, wIndex, wLength);
        		  uvcHandleReq= 1;
        		//isHandled = 1;
        	    }
        	}
                // unknown request 
                break;
        }
    }
    /* Check for UVC Class Requests */
        switch (bmReqType)
        {
            case CY_FX_USB_UVC_GET_REQ_TYPE:
            case CY_FX_USB_UVC_SET_REQ_TYPE:
                /* UVC Specific requests are handled in the EP0 thread. */
                switch (wIndex & 0xFF)
                {
                    case CY_FX_UVC_CONTROL_INTERFACE:
                        {
                            uvcHandleReq = CyTrue;
                            status = CyU3PEventSet (&glFxUVCEvent, CY_FX_UVC_VIDEO_CONTROL_REQUEST_EVENT,
                                    CYU3P_EVENT_OR);
                            if (status != CY_U3P_SUCCESS)
                            {
                            	ztex_log ("Set CY_FX_UVC_VIDEO_CONTROL_REQUEST_EVENT Failed \n");
                                CyU3PDebugPrint (4, "Set CY_FX_UVC_VIDEO_CONTROL_REQUEST_EVENT Failed %x\n", status);
                                CyU3PUsbStall (0, CyTrue, CyFalse);
                            }
                        }
                        break;

                    case CY_FX_UVC_STREAM_INTERFACE:
                        {
                            uvcHandleReq = CyTrue;
                            status = CyU3PEventSet (&glFxUVCEvent, CY_FX_UVC_VIDEO_STREAM_REQUEST_EVENT,
                                    CYU3P_EVENT_OR);
                            if (status != CY_U3P_SUCCESS)
                            {
                                /* Error handling */
                            	ztex_log ("Set CY_FX_UVC_VIDEO_STREAM_REQUEST_EVENT Failed %x\n");
                                CyU3PDebugPrint (4, "Set CY_FX_UVC_VIDEO_STREAM_REQUEST_EVENT Failed %x\n", status);
                                CyU3PUsbStall (0, CyTrue, CyFalse);
                            }
                        }
                        break;

                    default:
                        break;
                }
                break;

            case CY_FX_USB_SET_INTF_REQ_TYPE:
                if (bRequest == CY_FX_USB_SET_INTERFACE_REQ)
                {
                	/* Some hosts send Set Interface Alternate Setting 0 command while stopping the video
                     * stream. The application uses this event to stop streaming. */
                    if ((wIndex == CY_FX_UVC_STREAM_INTERFACE) && (wValue == 0))
                    {
                        /* Stop GPIF state machine to stop data transfers through FX3 */
                    	ztex_log("Alternate setting 0..\r\n");
                        CyU3PDebugPrint (4, "Alternate setting 0..\r\n");

                        CyU3PGpifDisable (CyFalse);
                        streamingStarted = CyFalse;

                        /* Place the EP in NAK mode before cleaning up the pipe. */
                        CyU3PUsbSetEpNak (CY_FX_EP_BULK_VIDEO, CyTrue);
                        CyU3PBusyWait (100);

                        /* Reset and flush the endpoint pipe. */
                        CyU3PDmaMultiChannelReset (&glChHandleUVCStream);
                        CyU3PUsbFlushEp (CY_FX_EP_BULK_VIDEO);
                        CyU3PUsbSetEpNak (CY_FX_EP_BULK_VIDEO, CyFalse);
                        CyU3PBusyWait (100);

                        /* Clear the stall condition and sequence numbers. */
                        CyU3PUsbStall (CY_FX_EP_BULK_VIDEO, CyFalse, CyTrue);
                        uvcHandleReq = CyTrue;

                        /* Complete Control request handshake */
                        CyU3PUsbAckSetup ();

                        /* Indicate stop streaming to main thread */
                        clearFeatureRqtReceived = CyTrue;
                        CyFxUVCApplnAbortHandler ();
                    }
                }
                break;

            case CY_U3P_USB_TARGET_ENDPT:
                if (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)
                {
                    if (wIndex == CY_FX_EP_BULK_VIDEO)
                    {
                        /* Windows OS sends Clear Feature Request after it stops streaming,
                         * however MAC OS sends clear feature request right after it sends a
                         * Commit -> SET_CUR request. Hence, stop streaming only if streaming
                         * has started. */
                        if (streamingStarted == CyTrue)
                        {	ztex_log("Clear feature request detected..\r\n");
                            CyU3PDebugPrint (4, "Clear feature request detected..\r\n");

                            /* Disable the GPIF state machine. */
                            CyU3PGpifDisable (CyFalse);
                            streamingStarted = CyFalse;

                            /* Place the EP in NAK mode before cleaning up the pipe. */
                            CyU3PUsbSetEpNak (CY_FX_EP_BULK_VIDEO, CyTrue);
                            CyU3PBusyWait (100);

                            /* Reset and flush the endpoint pipe. */
                            CyU3PDmaMultiChannelReset (&glChHandleUVCStream);
                            CyU3PUsbFlushEp (CY_FX_EP_BULK_VIDEO);
                            CyU3PUsbSetEpNak (CY_FX_EP_BULK_VIDEO, CyFalse);
                            CyU3PBusyWait (100);

                            /* Clear the stall condition and sequence numbers. */
                            CyU3PUsbStall (CY_FX_EP_BULK_VIDEO, CyFalse, CyTrue);

                            uvcHandleReq = CyTrue;
                            /* Complete Control request handshake */
                            CyU3PUsbAckSetup ();
                            /* Indicate stop streaming to main thread */
                            clearFeatureRqtReceived = CyTrue;
                            CyFxUVCApplnAbortHandler ();
                        }
                        else
                        {
                            uvcHandleReq = CyTrue;
                            CyU3PUsbAckSetup ();
                        }
                    }
                }
                break;

            default:
                break;
        }

        return uvcHandleReq;
    //return isHandled && (status == 0);
}
