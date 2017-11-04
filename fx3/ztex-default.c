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
    Template for default firmware.
*/

#undef GPIO_SIMPLE_BITMAP0
#define GPIO_SIMPLE_BITMAP0 ( ( 1 << GPIO_GPIO0 ) | ( 1 << GPIO_GPIO1 ) | ( 1 << GPIO_RESET ) | ( 1 << GPIO_CLK ) | ( 1 << GPIO_DATA ) | ( 1 << GPIO_STOP ) )
#undef GPIO_SIMPLE_BITMAP1
#define GPIO_SIMPLE_BITMAP1 ( ( 1 << (GPIO_GPIO2-32) ) | ( 1 << (GPIO_GPIO3-32) ) )

CyU3PDmaChannel dma_out_handle, dma_in_handle, dma_fpga_conf_handle;
CyBool_t communication_started = CyFalse;


/* 
re-using input channel for FPGA configuration fails in super speed mode due to a bug in Cypress SDK, see 
http://www.cypress.com/forum/usb-30-super-speed/dma-channel-gpif-reset-problem-super-speed-mode

#define ZTEX_FPGA_CONF_FAST_EP OUT_ENDPOINT 
#define ZTEX_FPGA_CONF_FAST_IFACE 0
#define ZTEX_FPGA_CONF_FAST_SOCKET EZUSB_IO_IN_SOCKET
*/

// workaround: create a own channel for FPGA configuration
#ifndef ZTEX_FPGA_CONF_FAST_IFACE
#define ZTEX_FPGA_CONF_FAST_IFACE 1
#define ZTEX_FPGA_CONF_FAST_SOCKET CY_U3P_PIB_SOCKET_5
#endif

/* endpount / dma macros
    OUT_ENDPOINT : host --> EZUSB --> FPGA 
      * interface 0
      * bulk size: 16*1024
      * buffer size: 16K
      * buffer count: 4
      * PIB socket: 1
    IN_ENDPOINT : FPGA --> EZUSB --> host
      * interface 0
      * bulk size: 16*1024
      * buffer size: 16K
      * buffer count: 4
      * PIB socket: 0
    ZTEX_FPGA_CONF_FAST_EP : FPGA configuration 
      * interface 1
      * bulk size: 1*1024
      * buffer size: 1K
      * buffer count: 2
      * PIB socket: 5
*/   



#include "ztex.c" 	


void run () {
    uint16_t snd_errors, rcv_errors;
    uint16_t last_snd_errors = 0;
    uint16_t last_rcv_errors = 0;
    
    ztex_log ( "Info: Starting default Firmware" );

    while (1) {
	snd_errors = ZTEX_USB3_SND_ERROR_COUNT;
	rcv_errors = ZTEX_USB3_RCV_ERROR_COUNT;
	if ( (snd_errors != last_snd_errors) || (rcv_errors != last_rcv_errors) ) {
    	    ZTEX_LOG("snd errors: %d,  rcv errors: %d", snd_errors, rcv_errors);
    	    last_snd_errors = snd_errors;
    	    last_rcv_errors = rcv_errors;
        }
/*	if ( CyU3PDmaChannelIsValid(&dma_in_handle) ) {
	    uint32_t p,c;
	    CyU3PDmaState_t s;
	    CyU3PDmaChannelGetStatus(&dma_in_handle, &s, &p,&c);
    	    ZTEX_LOG("Info: Input DMA status status=%d, prod=%d, cons=%d",s,p,c);
        } */
        CyU3PThreadSleep (500);
    }
    
}

void usb_start() {
    // start communication if cable is connected and FPGA is running
    if ( ZTEX_FPGA_CONFIGURED ) {
	ztex_log ( "Info: Starting communication" );
	communication_started = CyTrue;
		//CyU3PThreadResume(&uvcAppThread);
		//CyU3PThreadResume(&uvcAppEP0Thread);
	ztex_gpio_set(GPIO_RESET, CyFalse); 
	ztex_gpio_set(ZTEX_GPIO_LED, CyTrue);
    }
}

void usb_stop() {

    if ( communication_started ) {
	ztex_log ( "Info: Stopping communication" );
	communication_started = CyFalse;
	ztex_gpio_set(ZTEX_GPIO_LED, CyFalse);
    }
}

/*
 * Main function
 */
int main (void)
{
      ztex_pib_clock.clkDiv = 2;		// normal setting: 104 MHz @ 26 MHz external clock
//    ztex_pib_clock.clkDiv = 6;		// conservative setting: 69.33MHz @ 26 MHz external clock

    ztex_app_thread_run = run;
    ztex_usb_start = usb_start;
    ztex_usb_stop = usb_stop;
    
    ztex_interface_string[0] = "ZTEX Default Interface";
    ztex_interface_string[1] = "FPGA configuration interface";
 //   ztex_interface_string[2] = "UVC";

    
    ztex_main();  	// starts the OS and never returns
    return 0;		// makes the compiler happy
}
