#include "types.h"
#include "core.h"
#include "time.h"
#include "CRC16_8005.h"
#include "list.h"
#include "PointerCRC.h"

#include "hardware.h"
#include "SEGGER_RTT.h"
#include "hw_conf.h"
#include "hw_rtm.h"


#ifdef WIN32

#include <windows.h>
#include <Share.h>
#include <conio.h>
#include <stdarg.h>
#include <stdio.h>
#include <intrin.h>
#include "CRC16_CCIT.h"
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
static byte fram_I2c_Mem[0x10000];
//static byte fram_SPI_Mem[0x40000];
//
//static bool fram_spi_WREN = false;
//
//static u16 crc_ccit_result = 0;
//
//
//struct BlockBuffer { BlockBuffer *next; u32 block; u32 prevBlock; u32 writed; u32 data[((NAND_PAGE_SIZE+NAND_SPARE_SIZE) << NAND_PAGE_BITS) >> 2]; };
//
//static BlockBuffer _blockBuf[16];
//
//static List<BlockBuffer> freeBlockBuffer;
//static List<BlockBuffer> rdBlockBuffer;
//static List<BlockBuffer> wrBlockBuffer;
//
//static BlockBuffer *curNandBlockBuffer[4] = { 0 };
//
//static volatile bool busyWriteThread = false;

#elif defined(CPU_SAME53)

static S_I2C i2c(I2C_SERCOM_NUM, PIO_I2C, SCL, I2C_PMUX_SCL, PIO_I2C, SDA, I2C_PMUX_SDA, I2C_GEN_SRC, I2C_GEN_CLK, &I2C_DMA );

#elif defined(CPU_XMC48)

static S_I2C i2c(I2C_USIC_NUM, PIO_SCL, PIN_SCL, MUX_SCL, PIO_SDA, PIN_SDA, MUX_SDA, &I2C_DMA, I2C_DX0CR, I2C_DX1CR, SYSCLK);

#endif 


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//List<DSCI2C>	i2c_ReqList;
//DSCI2C*			i2c_dsc = 0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool I2C_Update()
{
#ifndef WIN32
	return i2c.Update();
#else
	return true;
#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool I2C_AddRequest(DSCI2C *d)
{

#ifndef WIN32

	return i2c.AddRequest(d);

#else

	if (d == 0) { return false; };
	if ((d->wdata == 0 || d->wlen == 0) && (d->rdata == 0 || d->rlen == 0)) { return false; }

	u16 adr;

	switch (d->adr)
	{
		case 0x49: //Temp

			if (d->rlen >= 2)
			{
				byte *p = (byte*)d->rdata;

				p[0] = 0;
				p[1] = 0;
			};
				
			d->readedLen = d->rlen;
			d->ack = true;
			d->ready = true;

			break;

		case 0x50: // FRAM

			d->readedLen = 0;

			if (d->wdata != 0 && d->wlen == 2)
			{
				adr = ReverseWord(*((u16*)d->wdata));

				adr %= sizeof(fram_I2c_Mem);

				if (d->wdata2 != 0 && d->wlen2 != 0)
				{
					u16 count = d->wlen2;
					byte *s = (byte*)d->wdata2;
					byte *d = fram_I2c_Mem + adr;

					while (count-- != 0) { *(d++) = *(s++); adr++; if (adr >= sizeof(fram_I2c_Mem)) { adr = 0; d = fram_I2c_Mem; }; };
				}
				else if (d->rdata != 0 && d->rlen != 0)
				{
					d->readedLen = d->rlen;
					u16 count = d->rlen;

					byte *p = (byte*)(d->rdata);
					byte *s = fram_I2c_Mem + adr;

					while (count-- != 0) { *(p++) = *(s++); adr++; if (adr >= sizeof(fram_I2c_Mem)) { adr = 0; s = fram_I2c_Mem; }; };
				};
			};

			d->ack = true;
			d->ready = true;

			break;
	};

	return true;

#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void I2C_Init()
{
#ifndef WIN32

	using namespace HW;

//	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_CYAN "I2C Init ... ");

	i2c.Connect(I2C_BAUDRATE);

#else

	HANDLE h;

	h = CreateFile("FRAM_I2C_STORE.BIN", GENERIC_READ, 0, 0, OPEN_EXISTING, 0, 0);

	if (h == INVALID_HANDLE_VALUE)
	{
		return;
	};

	dword bytes;

	ReadFile(h, fram_I2c_Mem, sizeof(fram_I2c_Mem), &bytes, 0);
	CloseHandle(h);

#endif

//	SEGGER_RTT_WriteString(0, "OK\n");
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef WIN32

void I2C_Destroy()
{
	HANDLE h;

	h = CreateFile("FRAM_I2C_STORE.BIN", GENERIC_WRITE, 0, 0, OPEN_ALWAYS, 0, 0);

	if (h == INVALID_HANDLE_VALUE)
	{
		return;
	};

	dword bytes;

	if (!WriteFile(h, fram_I2c_Mem, sizeof(fram_I2c_Mem), &bytes, 0))
	{
		dword le = GetLastError();
	};

	CloseHandle(h);
}

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
