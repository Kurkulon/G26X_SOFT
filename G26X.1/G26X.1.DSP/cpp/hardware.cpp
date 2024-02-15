#include "hardware.h"

#include <bfrom.h>
#include <sys\exception.h>
//#include <cdefBF592-A.h>
//#include <ccblkfn.h>

//#pragma optimize_for_speed

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <ADSP\system_imp.h>
#include <i2c.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

const u32 coreCLK = CCLK;
const u32 sysCLK = SCLK;

//U32u adcValue;

//u16 pgaValue = 0x2A01;
//bool pgaSet = true;
//bool adcEnable = true;

//byte netAdr = 1;

//u16 netResist = 0;
//u16 fltResist = 0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

DMA_CH	dmaRxSp0(SPORT0_RX_DMA);
DMA_CH	dmaRxSp1(SPORT1_RX_DMA);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool defSport0_Ready = false, defSport1_Ready = false;
static bool *sport0_Ready = 0, *sport1_Ready = 0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma optimize_for_speed

EX_INTERRUPT_HANDLER(SPORT0_ISR)
{
	HW::PIOF->SET(PF4);

	if (dmaRxSp0.CheckComplete())
	{
		HW::SPORT0->RCR1 = 0;
		dmaRxSp0.Disable();
		//*pDMA1_IRQ_STATUS = 1;
		//*pDMA1_CONFIG = 0;
		*sport0_Ready = true;
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
		//*pDMA3_IRQ_STATUS = 1;
		//*pDMA3_CONFIG = 0;
		*sport1_Ready = true;
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

	PIO_SYNC->EnableIRQA_Fall(BM_SYNC);
	PIO_SYNC->ClrMaskA(BM_SYNC);

	PIO_SYNC->ClrMUX(BM_SYNC);
	PIO_SYNC->SetFER(BM_SYNC);

	InitIVG(IVG_PORTF_SYNC, PID_Port_F_Interrupt_A, StartSPORT_ISR);

	//*pEVT9 = (void*)SPORT_ISR;
	//*pIMASK |= EVT_IVG9; 
	//*pSIC_IMASK |= IRQ_DMA1;


	//*pEVT11 = (void*)StartSPORT_ISR;
	//*pIMASK |= EVT_IVG11; 
	//*pSIC_IMASK |= IRQ_PFA_PORTF;

	//*pPORTFIO_INEN = 1<<12;
	//*pPORTFIO_EDGE = 1<<12;
	//*pPORTFIO_BOTH = 0;
	//*pPORTFIO_CLEAR = 1<<12;
	//*pPORTFIO_MASKA = 0;
	//*pPORTFIO_MASKB = 0;
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

void SyncReadSPORT(void *dst1, void *dst2, u16 len, u16 clkdiv, u16 delay, bool *ready0, bool *ready1)
{
	sport0_Ready = (ready0 != 0) ? ready0 : &defSport0_Ready;
	sport1_Ready = (ready1 != 0) ? ready1 : &defSport1_Ready;

	*sport0_Ready = false;
	*sport1_Ready = false;

	dmaRxSp0.Disable();
	dmaRxSp1.Disable();

	//*pDMA1_CONFIG = 0;
	//*pDMA3_CONFIG = 0;

	HW::SPORT0->RCR1	= 0;							//*pSPORT0_RCR1 = 0;
	HW::SPORT0->RCR2	= 15|RXSE;						//*pSPORT0_RCR2 = 15|RXSE;
	HW::SPORT0->RCLKDIV = clkdiv*(SCLK_MHz/50)-1;		//*pSPORT0_RCLKDIV = clkdiv;
	HW::SPORT0->RFSDIV	= 24;							//*pSPORT0_RFSDIV = 49;

	HW::SPORT1->RCR1	= 0;
	HW::SPORT1->RCR2	= 15|RXSE;
	HW::SPORT1->RCLKDIV = clkdiv*(SCLK_MHz/50)-1;
	HW::SPORT1->RFSDIV	= 24;

	if (delay != 0)
	{
		dmaRxSp0.Read16(dst1, delay*2, len);
		dmaRxSp1.Read16(dst2, delay*2, len);
	}
	else
	{
		dmaRxSp0.Read16(dst1, len); 
		dmaRxSp1.Read16(dst2, len); 
	};

	//*pDMA1_START_ADDR = dst1;
	//*pDMA1_X_COUNT = len1/2;
	//*pDMA1_X_MODIFY = 2;

	//*pDMA3_START_ADDR = dst2;
	//*pDMA3_X_COUNT = len2/2;
	//*pDMA3_X_MODIFY = 2;

	//*pDMA1_CONFIG = FLOW_STOP|DI_EN|WDSIZE_16|SYNC|WNR|DMAEN;
	//*pDMA3_CONFIG = FLOW_STOP|DI_EN|WDSIZE_16|SYNC|WNR|DMAEN;


	PIO_SYNC->ClearTriggerIRQ(BM_SYNC);
	PIO_SYNC->SetMaskA(BM_SYNC);

	//*pPORTFIO_CLEAR = 1<<12;
	//*pPORTFIO_MASKA = 1<<12;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//void WritePGA(u16 v)
//{
//	
//	*pSPI1_BAUD = 7; // SCLK=7MHz
//	*pSPI1_FLG = 0;//FLS5|FLS2;
//	*pSPI1_CTL = SPE|MSTR|SIZE|(TIMOD & 1);    // MSTR=1, CPOL=0, CPHA=0, LSBF=0, SIZE=1, EMISO=0, PSSE=0, GM=0, SZ=0, TIMOD=01
//	*pPORTGIO_CLEAR = 1<<11;
//	*pSPI1_TDBR = v;
//
//	while((*pSPI1_STAT&1) == 0) ;
//
//	*pPORTGIO_SET = 1<<11;
//
//	*pSPI1_CTL = 0;
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <G26X_1_UpdateADC_imp.h>

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

	CTM32 tm;

	tm.Reset();

	while(!tm.Check(MS2CTM(10))) UpdateHardware();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void UpdateHardware()
{
	UpdateADC();

	*pWDOG_STAT = 0; //Reset WDT
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
