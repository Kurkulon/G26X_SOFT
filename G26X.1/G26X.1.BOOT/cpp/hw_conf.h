#ifndef HW_CONF_H__11_11_22__17_22
#define HW_CONF_H__11_11_22__17_22

#pragma once

#include "core.h"

#include "G26X_1_HW_CONF.h"
#include "G_RCV.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define BOOT_MAX_NETADR			RCV_MAX_NUM_STATIONS
#define BOOT_TIMEOUT			(MS2CTM(2000))
#define BOOT_MAIN_TIMEOUT		(MS2CTM(100000))
#define BOOT_COM_SPEED			RCV_BOOT_COM_BAUDRATE	
#define BOOT_COM_PARITY			RCV_BOOT_COM_PARITY		
#define BOOT_COM_PRETIMEOUT		(~0)
#define BOOT_COM_POSTTIMEOUT	(US2COM(500))

#define BOOT_MAN_REQ_WORD		RCV_BOOT_REQ_WORD
#define BOOT_MAN_REQ_MASK 		RCV_BOOT_REQ_MASK

#define BOOT_HW_UPDATE 			UpdateADC

#define PIO_RTS					HW::PIOF
#define PIN_RTS					10
#define MASK_RTS				(1UL<<PIN_RTS)

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define MAIN_LOOP_PIN_TGL()		{ HW::PIOF->NOT(PF4);}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define BAUD_RATE_DIVISOR 	5

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#endif // HW_CONF_H__11_11_22__17_22
