/*
 * default.c

 *
 *  Created on: 19-Jul-2017
 *      Author: htic
 */

/*%
   Default firmware and loader for ZTEX USB-FPGA Modules 2.14
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
#include "cyu3system.h"
#include "cyu3os.h"
#include "cyu3dma.h"
#include "cyu3error.h"
#include "cyu3usb.h"
#include "cyu3spi.h"

// loads ztex header files and defeult configuration macros (fixed part of the SDK)
#include "ztex-conf.c"
#include "ztex-ufm-2_14.c"

#define OUT_ENDPOINT	2
#define IN_ENDPOINT	4
#define ZTEX_FPGA_CONF_FAST_EP	6

#define GPIO_RESET 25

#define GPIO_GPIO0 24
#define GPIO_GPIO1 23
#define GPIO_GPIO2 39
#define GPIO_GPIO3 38


#define GPIO_CLK 26
#define GPIO_DATA 21
#define GPIO_STOP 22

#undef ZTEX_PRODUCT_STRING
#define ZTEX_PRODUCT_STRING "Default Firmware for ZTEX USB-FPGA Modules 2.14"


#include "ztex-default.c"




