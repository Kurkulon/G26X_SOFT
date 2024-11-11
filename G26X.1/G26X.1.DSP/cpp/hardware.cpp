#include "hardware.h"

//#include <bfrom.h>
//#include <sys\exception.h>
//#include <cdefBF592-A.h>
//#include <ccblkfn.h>

//#pragma optimize_for_speed

//#include "SPI.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include "ADSP\system_imp.h"
#include "i2c.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define SPORT_BUF_NUM 2

const u32 coreCLK = CCLK;
const u32 sysCLK = SCLK;

static u16 temp = 0;

static DSCRSP02 *volatile curDscSPORT = 0;

static DSCRSP02 sportdsc[SPORT_BUF_NUM];

#pragma instantiate List<DSCRSP02>
static List<DSCRSP02> freeSPORT;
static List<DSCRSP02> readySPORT;
static DSCRSP02 dscRsp02[1];

//static List<DSCRSP02> freeRSP02;
//static List<DSCRSP02> readyRSP02;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

DMA_CH	dmaRxSp0(SPORT0_RX_DMA);
DMA_CH	dmaRxSp1(SPORT1_RX_DMA);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//S_SPIM spi(1, 

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static bool defSport0_Ready = false, defSport1_Ready = false;
//static bool *sport0_Ready = 0, *sport1_Ready = 0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

DSCRSP02* GetDscSPORT()
{
	return readySPORT.Get();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

DSCRSP02* AllocDscSPORT()
{
	return freeSPORT.Get();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void FreeDscSPORT(DSCRSP02* dsc)
{
	freeSPORT.Add(dsc);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma optimize_for_speed

EX_INTERRUPT_HANDLER(SPORT0_ISR)
{
	HW::PIOF->SET(PF4);

	if (dmaRxSp0.CheckComplete())
	{
		HW::SPORT0->RCR1 = 0;
		dmaRxSp0.Disable();
		curDscSPORT->readyMask |= 1;

		if (curDscSPORT->readyMask == 3) readySPORT.Add(curDscSPORT), curDscSPORT = 0;
	};

	HW::PIOF->CLR(PF4);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

EX_INTERRUPT_HANDLER(SPORT1_ISR)
{
	HW::PIOF->SET(PF4);

	if (dmaRxSp1.CheckComplete())
	{
		HW::SPORT1->RCR1 = 0;
		dmaRxSp1.Disable();
		curDscSPORT->readyMask |= 2;

		if (curDscSPORT->readyMask == 3) readySPORT.Add(curDscSPORT), curDscSPORT = 0;
	};

	HW::PIOF->CLR(PF4);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

EX_INTERRUPT_HANDLER(StartSPORT_ISR)
{
//	*pPORTFIO_SET = 1<<2;

	HW::SPORT0->RCR1 = RCKFE|LARFS|LRFS|RFSR|IRFS|IRCLK|RSPEN;
	HW::SPORT1->RCR1 = RCKFE|LARFS|LRFS|RFSR|IRFS|IRCLK|RSPEN;

	PIO_SYNC->ClrMaskA(BM_SYNC);
	PIO_SYNC->ClearTriggerIRQ(BM_SYNC);

	//*pPORTFIO_MASKA = 0;

	//*pPORTFIO_CLEAR = /*(1<<2)|*/(1<<12);


//	*pPORTFIO_CLEAR = 1<<2;
}

#pragma optimize_as_cmd_line

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitSPORT()
{
	HW::PIOF->SetFER(PF0|PF1|PF2|PF3);
	HW::PIOF->ClrMUX(PF0|PF1|PF2|PF3);

	HW::PIOG->SetFER(PG0|PG1|PG2|PG3);
	HW::PIOG->ClrMUX(PG0|PG1|PG2|PG3);

	dmaRxSp0.Disable();
	dmaRxSp1.Disable();

	HW::SPORT0->RCR1 = 0;
	HW::SPORT1->RCR1 = 0;

	InitIVG(IVG_SPORT0_DMA, PID_DMA1_SPORT0_RX, SPORT0_ISR);
	InitIVG(IVG_SPORT1_DMA, PID_DMA3_SPORT1_RX, SPORT1_ISR);

	PIO_SYNC->EnableIRQA_Low(BM_SYNC);
	PIO_SYNC->ClrMaskA(BM_SYNC);

	PIO_SYNC->ClrMUX(BM_SYNC);
	PIO_SYNC->SetFER(BM_SYNC);

	InitIVG(IVG_PORTF_SYNC, PID_Port_F_Interrupt_A, StartSPORT_ISR);

	for (u16 i = 0; i < ArraySize(sportdsc); i++)
	{
		DSCRSP02 &dsc = sportdsc[i];

		dsc.readyMask = 0;

		freeSPORT.Add(&dsc);
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//void ReadSPORT(void *dst1, void *dst2, u16 len1, u16 len2, u16 clkdiv, bool *ready0, bool *ready1)
//{
//	sport0_Ready = (ready0 != 0) ? ready0 : &defSport0_Ready;
//	sport1_Ready = (ready1 != 0) ? ready1 : &defSport1_Ready;
//
//	*sport0_Ready = false;
//	*sport1_Ready = false;
//
//	*pDMA1_CONFIG = 0;
//	*pDMA3_CONFIG = 0;
//
//	*pSPORT0_RCR1 = 0;
//	*pSPORT0_RCR2 = 15|RXSE;
//	*pSPORT0_RCLKDIV = clkdiv;
//	*pSPORT0_RFSDIV = 49;
//
//	*pSPORT1_RCR1 = 0;
//	*pSPORT1_RCR2 = 15|RXSE;
//	*pSPORT1_RCLKDIV = clkdiv;
//	*pSPORT1_RFSDIV = 49;
//
//	*pDMA1_START_ADDR = dst1;
//	*pDMA1_X_COUNT = len1/2;
//	*pDMA1_X_MODIFY = 2;
//
//	*pDMA3_START_ADDR = dst2;
//	*pDMA3_X_COUNT = len2/2;
//	*pDMA3_X_MODIFY = 2;
//
//	*pDMA1_CONFIG = FLOW_STOP|DI_EN|WDSIZE_16|SYNC|WNR|DMAEN;
//	*pDMA3_CONFIG = FLOW_STOP|DI_EN|WDSIZE_16|SYNC|WNR|DMAEN;
//
//	*pSPORT0_RCR1 = RCKFE|LARFS|LRFS|RFSR|IRFS|IRCLK|RSPEN;
//	*pSPORT1_RCR1 = RCKFE|LARFS|LRFS|RFSR|IRFS|IRCLK|RSPEN;
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SyncReadSPORT(DSCRSP02 *dsc, u16 delay) // (void *dst1, void *dst2, u16 len, u16 clkdiv, u16 delay, bool *ready0, bool *ready1)
{
	if (dsc == 0) return;

	dsc->readyMask = 0;

	dmaRxSp0.Disable();
	dmaRxSp1.Disable();

	//*pDMA1_CONFIG = 0;
	//*pDMA3_CONFIG = 0;

	u16 rclkdiv = dsc->r02.hdr.st*(SCLK_MHz/50)-1;

	HW::SPORT0->RCR1	= 0;							//*pSPORT0_RCR1 = 0;
	HW::SPORT0->RCR2	= 15|RXSE;						//*pSPORT0_RCR2 = 15|RXSE;
	HW::SPORT0->RCLKDIV = rclkdiv;						//*pSPORT0_RCLKDIV = clkdiv;
	HW::SPORT0->RFSDIV	= 24;							//*pSPORT0_RFSDIV = 49;

	HW::SPORT1->RCR1	= 0;
	HW::SPORT1->RCR2	= 15|RXSE;
	HW::SPORT1->RCLKDIV = rclkdiv;
	HW::SPORT1->RFSDIV	= 24;

	//delay = (delay+1)*2;
	u16 len = dsc->sportLen;//*2;

	dmaRxSp0.ReadInterleaved16(dsc->r02.data,			delay+1, len);
	dmaRxSp1.ReadInterleaved16(dsc->r02.data + len*2,	delay+1, len);

	curDscSPORT = dsc;

	PIO_SYNC->ClearTriggerIRQ(BM_SYNC);
	PIO_SYNC->SetMaskA(BM_SYNC);

}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SetGain(u16 v, bool preAmp)
{
	PIO_GAIN->WBIT(BM_GAIN, !preAmp);
	
	HW::SPI1->Baud	= 7; // SCLK=7MHz
	HW::SPI1->Flg	= FLS5;	//FLS5|FLS2;
	HW::SPI1->Ctl	= SPE|MSTR|SIZE|(TIMOD & 1);    // MSTR=1, CPOL=0, CPHA=0, LSBF=0, SIZE=1, EMISO=0, PSSE=0, GM=0, SZ=0, TIMOD=01
	//*pPORTGIO_CLEAR = 1<<11;
	HW::SPI1->TDBR	= 0x2A01|((v&7)<<4);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitGain()
{
	PIO_GAIN->ClrFER(BM_GAIN);
	PIO_GAIN->DirSet(BM_GAIN);

	HW::PIOG->SetFER(PG8|PG9|PG11);
	HW::PIOG->ClrMUX(PG8|PG9|PG11);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include "G26X_1_UpdateADC_imp.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void InitHardware()
{
	LowLevelInit();

	I2C_Init(SCLK_MHz, IVG_TWI, PID_TWI);

	InitSPORT();

	InitGain();

	CTM32 tm;

	tm.Reset();

	while(!tm.Check(MS2CTM(10))) UpdateHardware();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u16 GetTemp()
{
	return temp;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateTemp()
{
	static byte i = 0;

	static DSCI2C dsc;

	static u16 rbuf = 0;
	static byte buf[2];

	static CTM32 tm;

	switch (i)
	{
		case 0:

			if (tm.Check(MS2CTM(100)))
			{
				buf[0] = 0;

				dsc.adr = 0x49;
				dsc.wdata = buf;
				dsc.wlen = 1;
				dsc.rdata = &rbuf;
				dsc.rlen = 2;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				if (I2C_AddRequest(&dsc))
				{
					i++;
				};
			};

			break;

		case 1:

			if (dsc.ready)
			{
				if (dsc.ack && dsc.readedLen == dsc.rlen)
				{
					i32 t = (i16)ReverseWord(rbuf);

					temp = (t * 10 + 64) / 128;
				};

				i = 0;
			};

			break;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void UpdateHardware()
{
		static byte i = 0;

		#define CALL(p) case (__LINE__-S): p; break;

		enum C { S = (__LINE__+3) };
		switch(i++)
		{
			CALL( UpdateADC()	);
			CALL( UpdateTemp()	);
		};

		i = (i > (__LINE__-S-3)) ? 0 : i;

		#undef CALL

	*pWDOG_STAT = 0; //Reset WDT
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
