#include "hw_conf.h" 
#include "spi.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define FLASH_REQ_NUM 	16

#define AT25_SPI_BAUD_RATE 10000000

#ifndef FLASH_NEW

#include <FLASH\at25df021_imp_v2.h> 

#else

#define SMALL_BUF_LEN	(FLASH_PAGE_SIZE+64)
#define MEDIUM_BUF_LEN	(FLASH_PAGE_SIZE+68)
#define HUGE_BUF_LEN	(FLASH_PAGE_SIZE+72)

#define	NUM_SMALL_BUF	4       
#define	NUM_MEDIUM_BUF	4
#define	NUM_HUGE_BUF	4

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u16 SPI_CS_MASK[] = { PF8 };

static S_SPIM	spi(0, HW::PIOF, SPI_CS_MASK, ArraySize(SPI_CS_MASK), SCLK);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define ADSP_CHECKFLASH

#include "FLASH\FlashSPI_imp.h"

FlashSPI bootFlash(spi);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include "MEM\mem_imp.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#endif
