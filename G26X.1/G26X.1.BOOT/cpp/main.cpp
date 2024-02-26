#include "hardware.h"
#include "spi.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define BOOT_SGUID RCV_BOOT_SGUID

#define BOOT_COM
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

//#define PIO_RTS					HW::PIOF
//#define PIN_RTS					10
//#define MASK_RTS				(1UL<<PIN_RTS)

#define ADSP_CHECKFLASH

#define	NUM_SMALL_BUF	60      
#define	NUM_MEDIUM_BUF	1
#define	NUM_HUGE_BUF	1

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define MAIN_LOOP_PIN_TGL()		{ HW::PIOF->NOT(PF4);}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define BAUD_RATE_DIVISOR 	5

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u16 SPI_CS_MASK[] = { PF8 };

static S_SPIM	spi(0, HW::PIOF, SPI_CS_MASK, ArraySize(SPI_CS_MASK), SCLK);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifndef BOOT_NEW

	#include "BOOT\boot_imp.h"

#else // #ifndef BOOT_NEW

	#undef BOOT_TIMEOUT			
	#undef BOOT_MAIN_TIMEOUT	
	#define BOOT_TIMEOUT			(2000)
	#define BOOT_MAIN_TIMEOUT		(100000)

	#define Pin_MainLoop_Tgl()		MAIN_LOOP_PIN_TGL()

	#ifdef BOOT_COM

	#include "Comport\ComPort_imp.h"

	static ComPort com;

	#endif

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#include <FLASH\FlashSPI_imp.h>

	FlashSPI bootFlash(spi);

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#include <BOOT\boot_com_emac_imp_v2.h>

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#endif // #else // #ifndef BOOT_NEW
