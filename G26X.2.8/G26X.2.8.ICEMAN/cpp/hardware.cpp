#include "types.h"
#include "core.h"
#include "time.h"
#include "CRC\CRC16_8005.h"
#include "list.h"
#include "PointerCRC.h"

#include "hardware.h"
#include <SEGGER_RTT\SEGGER_RTT.h>
#include "hw_rtm.h"
//#include "hw_nand.h"
#include <DMA\DMA.h>
#include <MANCH\manch.h>
#include <math.h>
//#include "G26X_3_HW_CONF.H"

//#define TRM_SYNC_IRQ

const float pi = 3.14159265358979f;
//extern u16 curFireVoltage;
u16 waveBuffer[1000] = {0};

static i16 sinArr[256] = {0};

#define pwmPeriodUS	6
#define waveTimeUS	300
//u16 waveAmp = 0;
//u16 waveFreq = 3000; //Hz

#define postFirePeriodUS	50
#define postFireCount		(10000/postFirePeriodUS)

u16 waveLen = 2;

byte isrFireNum = 0;
bool postFire = false;

bool dachvInverted = false;


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//static void InitVectorTable();

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifndef WIN32

//__forceinline 	void EnableVCORE()	{ PIO_ENVCORE->CLR(ENVCORE); 	}
//__forceinline 	void DisableVCORE()	{ PIO_ENVCORE->SET(ENVCORE); 	}

#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <ARM\system_imp.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++




//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void WDT_Init()
{
	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_YELLOW "WDT Init ... ");

	#ifdef CPU_SAME53	

		HW::MCLK->APBAMASK |= APBA_WDT;

		HW::WDT->CONFIG = WDT_WINDOW_CYC512|WDT_PER_CYC1024;
	
		#ifndef _DEBUG
		HW::WDT->CTRLA = WDT_ENABLE;
		#else
		HW::WDT->CTRLA = 0;
		#endif

		//while(HW::WDT->SYNCBUSY);

	#elif defined(CPU_XMC48)

		#ifndef _DEBUG
	
		//HW::WDT_Enable();

		//HW::WDT->WLB = OFI_FREQUENCY/2;
		//HW::WDT->WUB = (3 * OFI_FREQUENCY)/2;
		//HW::SCU_CLK->WDTCLKCR = 0|SCU_CLK_WDTCLKCR_WDTSEL_OFI;

		//HW::WDT->CTR = WDT_CTR_ENB_Msk|WDT_CTR_DSP_Msk;

		#else

		HW::WDT_Disable();

		#endif

	#endif

	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_GREEN "OK\n");
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void InitHardware()
{
	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_YELLOW "Hardware Init ... \n");

#ifdef CPU_SAME53	
	
	using namespace HW;

	#ifdef _DEBUG

		SEGGER_RTT_printf(0, "HW::WDT->CTRLA = 0x%02X\n", (u32)(HW::WDT->CTRLA));

		SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_GREEN "Disable WDT ... ");

		HW::MCLK->APBAMASK |= APBA_WDT;

		while(HW::WDT->SYNCBUSY);

		if (HW::WDT->CTRLA & WDT_ENABLE)
		{
			HW::WDT->CTRLA = 0;
			
			while(HW::WDT->SYNCBUSY);
		};

		SEGGER_RTT_WriteString(0, "OK\n");

	#endif

//	HW::PIOA->BSET(13);

	HW::GCLK->GENCTRL[GEN_32K]	= GCLK_DIV(1)	|GCLK_SRC_OSCULP32K	|GCLK_GENEN;

	HW::GCLK->GENCTRL[GEN_1M]	= GCLK_DIV(25)	|GCLK_SRC_XOSC1		|GCLK_GENEN		|GCLK_OE;

	HW::GCLK->GENCTRL[GEN_25M]	= GCLK_DIV(1)	|GCLK_SRC_XOSC1		|GCLK_GENEN;

//	HW::GCLK->GENCTRL[GEN_500K] = GCLK_DIV(50)	|GCLK_SRC_XOSC1		|GCLK_GENEN;

	//PIO_32kHz->SetWRCONFIG(1UL<<PIN_32kHz, PORT_PMUX_M|PORT_WRPINCFG|PORT_PMUXEN|PORT_WRPMUX|PORT_PULLEN);

	//HW::GCLK->GENCTRL[GEN_EXT32K]	= GCLK_DIV(1)	|GCLK_SRC_GCLKIN	|GCLK_GENEN		;


	HW::MCLK->APBAMASK |= APBA_EIC;
	HW::GCLK->PCHCTRL[GCLK_EIC] = GCLK_GEN(GEN_MCK)|GCLK_CHEN;

	EIC->CTRLA = EIC_SWRST; while(EIC->SYNCBUSY);

	HW::MCLK->APBBMASK |= APBB_EVSYS;
	EVSYS->CTRLA = EVSYS_SWRST;

	HW::GCLK->PCHCTRL[GCLK_SERCOM_SLOW]		= GCLK_GEN(GEN_32K)|GCLK_CHEN;	// 32 kHz

#endif

	Init_time(MCK);
	I2C_Init();

	RTT_Init();

#ifndef WIN32

	#ifdef MANCH_REQ
		InitManRecieve();
		InitManTransmit();
	#endif

//	EnableVCORE();
	
	WDT_Init();

#else

	InitDisplay();

#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void UpdateHardware()
{
#ifndef WIN32

	I2C_Update();

#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
