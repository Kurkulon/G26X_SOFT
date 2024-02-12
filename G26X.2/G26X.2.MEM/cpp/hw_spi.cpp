#include "core.h"
#include "time.h"
#include "spi.h"
#include "spis.h"
#include "SEGGER_RTT\SEGGER_RTT.h"
#include "hw_conf.h"


#ifdef CPU_SAME53
static u32 SPI_CS_MASK[] = { CS0, CS1 };
#elif defined(CPU_XMC48)
static byte SPI_CS_PIN[] = { PIN_CS0, PIN_CS1 };
#endif

#ifdef WIN32

#include <windows.h>
#include <Share.h>
#include <conio.h>
#include <stdarg.h>
#include <stdio.h>
#include <intrin.h>
//#include "CRC16_CCIT.h"
#include "list.h"

//static HANDLE handleNandFile;
//static const char nameNandFile[] = "NAND_FLASH_STORE.BIN";
//
//static HANDLE handleWriteThread;
//static HANDLE handleReadThread;
//
//static byte nandChipSelected = 0;
//
//static u64 curNandFilePos = 0;
////static u64 curNandFileBlockPos = 0;
//static u32 curBlock = 0;
//static u32 curRawBlock = 0;
//static u16 curPage = 0;
//static u16 curCol = 0;
//
//static OVERLAPPED	_overlapped;
//static u32			_ovlReadedBytes = 0;
//static u32			_ovlWritenBytes = 0;
//
//static void* nandEraseFillArray;
//static u32 nandEraseFillArraySize = 0;
//static byte nandReadStatus = 0x41;
//static u32 lastError = 0;
//
//
//static byte fram_I2c_Mem[0x10000];
static byte fram_SPI_Mem[0x40000];
//
static bool fram_spi_WREN = false;
//
//static u16 crc_ccit_result = 0;

static S_SPIM	spi;

#elif defined(CPU_SAME53)

static S_SPIM	spi(SPI_SERCOM_NUM, PIO_SPCK, PIO_MOSI, PIO_MISO, PIO_CS, SPCK, MOSI, MISO, SPI_PMUX_SPCK, SPI_PMUX_MOSI, SPI_PMUX_MISO, SPI_CS_MASK, ArraySize(SPI_CS_MASK), 
					SPI_DIPO_BITS, SPI_DOPO_BITS, SPI_GEN_SRC, SPI_GEN_CLK, &SPI_DMA_TX, &SPI_DMA_RX);

#elif defined(CPU_XMC48)

static S_SPIM	spi(SPI_USIC_NUM, PIO_SPCK, PIO_MOSI, PIO_MISO, PIO_CS, PIN_SPCK, PIN_MOSI, PIN_MISO, MUX_SPCK, MUX_MOSI, SPI_CS_PIN, ArraySize(SPI_CS_PIN), &SPI_DMA, SPI_DRL, SPI_DX0CR, SPI_DX1CR, SYSCLK);

#endif 


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static byte *spi_wrPtr = 0;
//static byte *spi_rdPtr = 0;
//static u16 spi_wrCount = 0;
//static u16 spi_count = 0;
//static u16 spi_rdCount = 0;
//static byte *spi_wrPtr2 = 0;
//static u16 spi_wrCount2 = 0;
//static u32 spi_adr = 0;
//static DSCSPI* spi_dsc = 0;
//static DSCSPI* spi_lastDsc = 0;


//static u32 spi_timestamp = 0;

//static bool SPI_Write(DSCSPI *d);
//static bool SPI_Read(DSCSPI *d);
//static bool SPI_WriteRead(DSCSPI *d);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool SPI_AddRequest(DSCSPI *d)
{
	if (d == 0) { return false; };
	//if ((d->wdata == 0 || d->wlen == 0) && (d->rdata == 0 || d->rlen == 0)) { return false; }

#ifndef WIN32

	return spi.AddRequest(d);

#else

	u32 adr;

	switch (d->csnum)
	{
		case 0x0: //Accel

			if (d->rlen >= 2)
			{
				byte *p = (byte*)d->rdata;

				p[0] = 0;
				p[1] = 0;
			};
				
			d->ready = true;

			break;

		case 0x1: // FRAM

			switch(d->adr&0xFF)
			{
				case 0x2: // WRITE

					if (d->alen == 4 && d->wdata != 0 && d->wlen != 0 && fram_spi_WREN)
					{
						adr = ReverseDword(d->adr & ~0xFF);
						adr %= sizeof(fram_SPI_Mem);

						u16 count = d->wlen;
						byte *s = (byte*)d->wdata;
						byte *d = fram_SPI_Mem + adr;

						while (count-- != 0) { *(d++) = *(s++); adr++; if (adr >= sizeof(fram_SPI_Mem)) { adr = 0; d = fram_SPI_Mem; }; };
					};

					fram_spi_WREN = false;

					break;

				case 0x3: // READ

					if (d->alen == 4 && d->rdata != 0 && d->rlen != 0)
					{
						adr = ReverseDword(d->adr & ~0xFF);
						adr %= sizeof(fram_SPI_Mem);

						u16 count = d->rlen;

						byte *p = (byte*)(d->rdata);
						byte *s = fram_SPI_Mem + adr;

						while (count-- != 0) { *(p++) = *(s++); adr++; if (adr >= sizeof(fram_SPI_Mem)) { adr = 0; s = fram_SPI_Mem; }; };
					};

					fram_spi_WREN = false;

					break;

				case 0x6: // WREN

					fram_spi_WREN = (d->alen == 1);

					break;

				default:	fram_spi_WREN = false; break;
			};

			d->ready = true;

			break;
	};

	return true;

#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool SPI_Update()
{
	return spi.Update();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SPI_Init()
{
#ifndef WIN32

	spi.Connect(SPI_BAUDRATE);

#else

	HANDLE h;

	h = CreateFile("FRAM_SPI_STORE.BIN", GENERIC_READ, 0, 0, OPEN_EXISTING, 0, 0);

	if (h == INVALID_HANDLE_VALUE)
	{
		return;
	};

	dword bytes;

	ReadFile(h, fram_SPI_Mem, sizeof(fram_SPI_Mem), &bytes, 0);
	CloseHandle(h);

#endif
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef WIN32

void SPI_Destroy()
{
	HANDLE h;

	h = CreateFile("FRAM_SPI_STORE.BIN", GENERIC_WRITE, 0, 0, OPEN_ALWAYS, 0, 0);

	if (h == INVALID_HANDLE_VALUE)
	{
		return;
	};

	dword bytes;

	if (!WriteFile(h, fram_SPI_Mem, sizeof(fram_SPI_Mem), &bytes, 0))
	{
		dword le = GetLastError();
	};

	CloseHandle(h);
}

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include "spis_imp.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
