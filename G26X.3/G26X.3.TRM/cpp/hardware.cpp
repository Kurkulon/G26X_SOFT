#include "types.h"
#include "core.h"
#include "time.h"
#include "CRC\CRC16_8005.h"
#include "list.h"
#include "PointerCRC.h"

#include "hardware.h"
#include <SEGGER_RTT\SEGGER_RTT.h>
#include "hw_rtm.h"
#include "hw_nand.h"
#include <DMA\DMA.h>
#include <MANCH\manch.h>
#include <math.h>
#include "G26X_3_HW_CONF.H"


const float pi = 3.14159265358979f;
//extern u16 curFireVoltage;
u16 waveBuffer[1000] = {0};

static i16 sinArr[256] = {0};

#define pwmPeriodUS	6
#define waveTimeUS	300
//u16 waveAmp = 0;
//u16 waveFreq = 3000; //Hz
u16 waveLen = 2;

static Rsp72 rsp72buf[8];

static List<Rsp72>	freeRsp72;
static List<Rsp72>	readyRsp72;

static Rsp72 *hwRsp72 = 0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Rsp72* GetReadyRsp72() { return readyRsp72.Get(); }

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void FreeRsp72(Rsp72 *rsp) { freeRsp72.Add(rsp); }

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//static void InitVectorTable();

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifndef WIN32

__forceinline 	void EnableVCORE()	{ PIO_ENVCORE->CLR(ENVCORE); 	}
__forceinline 	void DisableVCORE()	{ PIO_ENVCORE->SET(ENVCORE); 	}

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

#ifdef PWMDMA_TCC

	#define PWMDMATCC					HW::PWMDMA_TCC
	#define PWMDMA_GEN					CONCAT2(GEN_,PWMDMA_TCC)
	#define PWMDMA_GEN_CLK				CONCAT2(CLK_,PWMDMA_TCC) 
	#define PWMDMA_IRQ					CONCAT2(PWMDMA_TCC,_0_IRQ)
	#define GCLK_PWMDMA					CONCAT2(GCLK_,PWMDMA_TCC)
	#define PID_PWMDMA					CONCAT2(PID_,PWMDMA_TCC)

	#if (PWMDMA_GEN_CLK > 100000000)
			#define PWMDMA_PRESC_NUM	1
	#elif (PWMDMA_GEN_CLK > 50000000)
			#define PWMDMA_PRESC_NUM	1
	#elif (PWMDMA_GEN_CLK > 20000000)
			#define PWMDMA_PRESC_NUM	1
	#elif (PWMDMA_GEN_CLK > 10000000)
			#define PWMDMA_PRESC_NUM	1
	#elif (PWMDMA_GEN_CLK > 5000000)
			#define PWMDMA_PRESC_NUM	1
	#else
			#define PWMDMA_PRESC_NUM	1
	#endif

	#define PWMDMA_PRESC_DIV			CONCAT2(TCC_PRESCALER_DIV,PWMDMA_PRESC_NUM)
	#define US2PWMDMA(v)				(((v)*(PWMDMA_GEN_CLK/PWMDMA_PRESC_NUM/1000)+500)/1000)

	#define PWMDMA0_EVSYS_USER			CONCAT3(EVSYS_USER_, PWMDMA_TCC, _EV_0)
	#define PWMDMA1_EVSYS_USER			CONCAT3(EVSYS_USER_, PWMDMA_TCC, _EV_1)
	#define PWMDMA_EVENT_GEN			CONCAT3(EVGEN_, PWMDMA_TCC, _OVF)

	#define PWMDMA_DMCH_TRIGSRC			CONCAT3(DMCH_TRIGSRC_, PWMDMA_TCC, _OVF)

	inline void PWMDMA_ClockEnable()	{ HW::GCLK->PCHCTRL[GCLK_PWMDMA] = PWMDMA_GEN|GCLK_CHEN; HW::MCLK->ClockEnable(PID_PWMDMA); }

#else
//	#error  Must defined PWMDMA_TCC
#endif


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef PWMCOUNT_TCC

	#define PWMCOUNTTCC					HW::PWMCOUNT_TCC
	#define PWMCOUNT_GEN				CONCAT2(GEN_,PWMCOUNT_TCC)
	#define PWMCOUNT_GEN_CLK			CONCAT2(CLK_,PWMCOUNT_TCC) 
	#define PWMCOUNT_IRQ				CONCAT2(PWMCOUNT_TCC,_0_IRQ)
	#define GCLK_PWMCOUNT				CONCAT2(GCLK_,PWMCOUNT_TCC)
	#define PID_PWMCOUNT				CONCAT2(PID_,PWMCOUNT_TCC)

	#if (PWMCOUNT_GEN_CLK > 100000000)
			#define PWMCOUNT_PRESC_NUM	1
	#elif (PWMCOUNT_GEN_CLK > 50000000)
			#define PWMCOUNT_PRESC_NUM	1
	#elif (PWMCOUNT_GEN_CLK > 20000000)
			#define PWMCOUNT_PRESC_NUM	1
	#elif (PWMCOUNT_GEN_CLK > 10000000)
			#define PWMCOUNT_PRESC_NUM	1
	#elif (PWMCOUNT_GEN_CLK > 5000000)
			#define PWMCOUNT_PRESC_NUM	1
	#else
			#define PWMCOUNT_PRESC_NUM	1
	#endif

	#define PWMCOUNT_PRESC_DIV			CONCAT2(TCC_PRESCALER_DIV,PWMCOUNT_PRESC_NUM)
	#define US2PWMCOUNT(v)				(((v)*(PWMCOUNT_GEN_CLK/PWMCOUNT_PRESC_NUM)+500000)/1000000)

	#define PWMCOUNT_EVENT_GEN			CONCAT3(EVGEN_, PWMCOUNT_TCC, _OVF)
	#define PWMCOUNT0_EVSYS_USER		CONCAT3(EVSYS_USER_, PWMCOUNT_TCC, _EV_0)
	#define PWMCOUNT1_EVSYS_USER		CONCAT3(EVSYS_USER_, PWMCOUNT_TCC, _EV_1)

	inline void PWMCOUNT_ClockEnable()	{ HW::GCLK->PCHCTRL[GCLK_PWMCOUNT] = PWMCOUNT_GEN|GCLK_CHEN; HW::MCLK->ClockEnable(PID_PWMCOUNT); }

#else
	#error  Must defined PWMCOUNT_TCC
#endif


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef PWM_TCC

	#define PWMTCC						HW::PWM_TCC
	#define PWM_GEN						CONCAT2(GEN_,PWM_TCC)
	#define PWM_GEN_CLK					CONCAT2(CLK_,PWM_TCC) 
	#define PWM_IRQ						CONCAT2(PWM_TCC,_0_IRQ)
	#define GCLK_PWM					CONCAT2(GCLK_,PWM_TCC)
	#define PID_PWM						CONCAT2(PID_,PWM_TCC)

	#if (PWM_GEN_CLK > 100000000)
			#define PWM_PRESC_NUM		4
	#elif (PWM_GEN_CLK > 50000000)
			#define PWM_PRESC_NUM		2
	#elif (PWM_GEN_CLK > 20000000)
			#define PWM_PRESC_NUM		1
	#elif (PWM_GEN_CLK > 10000000)
			#define PWM_PRESC_NUM		1
	#elif (PWM_GEN_CLK > 5000000)
			#define PWM_PRESC_NUM		1
	#else
			#define PWM_PRESC_NUM		1
	#endif

	#define PWM_PRESC_DIV				CONCAT2(TCC_PRESCALER_DIV,PWM_PRESC_NUM)
	#define US2PWM(v)					(((v)*(PWM_GEN_CLK/PWM_PRESC_NUM/1000)+500)/1000)


	#define PWM_CC_NUM					CONCAT2(PWM_TCC,_CC_NUM)

	#define PWMLA_CC_NUM				(PWMLA_WO_NUM % PWM_CC_NUM)
	#define PWMHA_CC_NUM				(PWMHA_WO_NUM % PWM_CC_NUM)
	#define PWMLB_CC_NUM				(PWMLB_WO_NUM % PWM_CC_NUM)
	#define PWMHB_CC_NUM				(PWMHB_WO_NUM % PWM_CC_NUM)

	#define PWM_EVENT_GEN				CONCAT3(EVGEN_, PWM_TCC, _OVF)
	#define PWM_EVSYS_USER				CONCAT3(EVSYS_USER_, PWM_TCC, _EV_0)

	#define PWM_DMCH_TRIGSRC			CONCAT3(DMCH_TRIGSRC_, PWM_TCC, _OVF)

	inline void PWM_ClockEnable()		{ HW::GCLK->PCHCTRL[GCLK_PWM] = PWM_GEN|GCLK_CHEN; HW::MCLK->ClockEnable(PID_PWM); }

#else
	#error  Must defined PWM_TCC
#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef WFG_TC

	#define WFGTC						HW::WFG_TC
	#define WFG_GEN						CONCAT2(GEN_,WFG_TC)
	#define WFG_GEN_CLK					CONCAT2(CLK_,WFG_TC) 
	#define WFG_IRQ						CONCAT2(WFG_TC,_IRQ)
	#define GCLK_WFG					CONCAT2(GCLK_,WFG_TC)
	#define PID_WFG						CONCAT2(PID_,WFG_TC)

	#if (WFG_GEN_CLK > 100000000)
			#define WFG_PRESC_NUM		8
	#elif (WFG_GEN_CLK > 50000000)
			#define WFG_PRESC_NUM		4
	#elif (WFG_GEN_CLK > 20000000)
			#define WFG_PRESC_NUM		2
	#elif (WFG_GEN_CLK > 10000000)
			#define WFG_PRESC_NUM		1
	#elif (WFG_GEN_CLK > 5000000)
			#define WFG_PRESC_NUM		1
	#else
			#define WFG_PRESC_NUM		1
	#endif

	#define WFG_PRESC_DIV				CONCAT2(TC_PRESCALER_DIV,WFG_PRESC_NUM)
	#define US2WFG(v)					(((v)*(WFG_GEN_CLK/WFG_PRESC_NUM)+500000)/1000000)

	#define WFG_EVSYS_USER				CONCAT3(EVSYS_USER_, WFG_TC, _EVU)

	inline void WFG_ClockEnable()		{ HW::GCLK->PCHCTRL[GCLK_WFG] = WFG_GEN|GCLK_CHEN; HW::MCLK->ClockEnable(PID_WFG); }

#else
	#error  Must defined PWM_TCC
#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef ADC_TCC

	#define ADCTCC						HW::ADC_TCC
	#define ADC_GEN						CONCAT2(GEN_,ADC_TCC)
	#define ADC_GEN_CLK					CONCAT2(CLK_,ADC_TCC) 
	#define ADC_IRQ						CONCAT2(ADC_TCC,_0_IRQ)
	#define GCLK_ADC					CONCAT2(GCLK_,ADC_TCC)
	#define PID_ADC						CONCAT2(PID_,ADC_TCC)

	#if (ADC_GEN_CLK > 100000000)
			#define ADC_PRESC_NUM		4
	#elif (ADC_GEN_CLK > 50000000)
			#define ADC_PRESC_NUM		2
	#elif (ADC_GEN_CLK > 20000000)
			#define ADC_PRESC_NUM		1
	#elif (ADC_GEN_CLK > 10000000)
			#define ADC_PRESC_NUM		1
	#elif (ADC_GEN_CLK > 5000000)
			#define ADC_PRESC_NUM		1
	#else
			#define ADC_PRESC_NUM		1
	#endif

	#define ADC_PRESC_DIV				CONCAT2(TCC_PRESCALER_DIV,ADC_PRESC_NUM)
	#define US2ADC(v)					(((v)*(ADC_GEN_CLK/ADC_PRESC_NUM/1000)+500)/1000)

	#define ADC_EVSYS_USER				CONCAT3(EVSYS_USER_, ADC_TCC, _EV_0)

	#define ADC_DMCH_TRIGSRC			CONCAT3(DMCH_TRIGSRC_, ADC_TCC, _OVF)

	inline void ADC_ClockEnable()		{ HW::GCLK->PCHCTRL[GCLK_ADC] = ADC_GEN|GCLK_CHEN; HW::MCLK->ClockEnable(PID_ADC); }

#else
	#error  Must defined PWM_TCC
#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void PrepareWFMOSC(u16 fireNum, u16 waveFreq, u16 klen)
{
	u16 N = 64;

	ADCTCC->CTRLA &= ~TCC_ENABLE;

	if (hwRsp72 == 0) hwRsp72 = freeRsp72.Get();

	if (hwRsp72 == 0) return;

	u16 t = ((1000000+waveFreq/2)/waveFreq + N/4)/(N/2);

	Rsp72 &rsp = *hwRsp72;

	rsp.h.rw	= 0;
	rsp.h.num	= fireNum;
	rsp.h.amp	= 0;
	rsp.h.st	= t;
	rsp.h.sl	= N * klen / 2;
	rsp.h.sd	= 0;

	while(ADCTCC->SYNCBUSY);

	ADCTCC->CTRLA = PWM_PRESC_DIV;
	ADCTCC->WAVE = TCC_WAVEGEN_NPWM;
	ADCTCC->PER = US2ADC(t)-1;
	ADCTCC->EVCTRL = TCC_TCEI0|TCC_EVACT0_RETRIGGER;
	ADCTCC->INTENCLR = ~0;

	ADCTCC->CTRLA |= TCC_ENABLE;

	ADC_DMA.ReadPeripheral(&HW::ADC1->RESULT, rsp.h.data, rsp.h.sl, DMCH_TRIGACT_BURST|ADC_DMCH_TRIGSRC, DMDSC_BEATSIZE_HWORD); 
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Update_WFMOSC()
{
	if (hwRsp72 != 0)
	{
		if (ADC_DMA.CheckComplete())
		{
			ADCTCC->CTRLA &= ~TCC_ENABLE;

			readyRsp72.Add(hwRsp72);

			hwRsp72 = 0;
		};
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Init_WFMOSC()
{
	ADC_ClockEnable();

	ADCTCC->CTRLA = PWM_PRESC_DIV;
	ADCTCC->WAVE = TCC_WAVEGEN_NPWM;
	ADCTCC->PER = US2PWM(pwmPeriodUS)-1;
	ADCTCC->EVCTRL = TCC_TCEI0|TCC_EVACT0_RETRIGGER;
	ADCTCC->INTENCLR = ~0;

	HW::EVSYS->USER[ADC_EVSYS_USER] = EVENT_PWM_SYNC+1;

	for (u16 i = 0; i < ArraySize(rsp72buf); i++) freeRsp72.Add(rsp72buf+i);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static u32 pwmStopTime = 0;
static u16 *ppwmdata = waveBuffer;
static bool pwmstat = true;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma push
#pragma O3
#pragma Otime

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool IsFireOK()
{
	return pwmstat;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void DisableFire()
{
	HW::EVSYS->CH[EVENT_PWM_SYNC].CHANNEL = EVGEN_NONE;
	PIO_URXD0->SetWRCONFIG(URXD0, PMUX_URXD0|PORT_WRPMUX|PORT_WRPINCFG|PORT_PMUXEN); 
	PWMTCC->CTRLA &= ~TCC_ENABLE;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static __irq void PwmCountIRQ()
{
	HW::PIOB->BSET(23);

	PWMTCC->CTRLBSET = TCC_CMD_STOP;

	PIO_PWMHM->PINCFG[PIN_PWMHM] = 0;
	PIO_PWMLM->PINCFG[PIN_PWMLM] = 0;
	PIO_PWMHX->PINCFG[PIN_PWMHX] = 0;
	PIO_PWMLX->PINCFG[PIN_PWMLX] = 0;
	PIO_PWMHY->PINCFG[PIN_PWMHY] = 0;
	PIO_PWMLY->PINCFG[PIN_PWMLY] = 0;

	PIO_WF_PWM->EVCTRL.EV[0] = 0;
	PIO_WF_PWM->SET(WF_PWM);
	PIO_DRVEN->CLR(DRVEN);
	//WFGTC->CTRLBSET = TC_CMD_STOP;
//	pwmStopTime = GetMilliseconds();
	PWMCOUNTTCC->INTFLAG = ~0;
	PWMCOUNTTCC->CTRLA &= ~TCC_ENABLE;
	pwmstat = true;
	DisableFire();
	PWM_DMA.Disable();

	HW::PIOB->BCLR(23);
}

#pragma pop

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void PrepareFire(u16 fireNum, u16 waveFreq, u16 waveAmp, u16 fireCount, u16 fireDuty)
{
	u16 hi, mid, lo;
	i16 amp;
	volatile void *dst;

	bool pwm = true;

	HW::PIOA->BSET(21);

	pwmstat = false;
	PIO_DRVEN->CLR(DRVEN);

	PWMTCC->CTRLA &= ~TCC_ENABLE;
	PWMCOUNTTCC->CTRLA &= ~TCC_ENABLE;

	if (waveFreq < 500) waveFreq = 500;

	fireCount = LIM(fireCount, 1, 5);

	if (fireNum <= 1)
	{
		u16 kLen = 2;

		if (waveAmp > 2100)
		{
			u16 t = waveAmp - 2100;
		
			kLen = 3 + (t / 460);

			waveAmp = 2100;
		};

		PrepareWFMOSC(fireNum, waveFreq, kLen);

		PIO_PWMHM->PINCFG[PIN_PWMHM] = PINGFG_PMUXEN;
		PIO_PWMLM->PINCFG[PIN_PWMLM] = PINGFG_PMUXEN;
		PIO_PWMHX->PINCFG[PIN_PWMHX] = 0;
		PIO_PWMLX->PINCFG[PIN_PWMLX] = 0;
		PIO_PWMHY->PINCFG[PIN_PWMHY] = 0;
		PIO_PWMLY->PINCFG[PIN_PWMLY] = 0;

		//ppwmdata = waveBuffer; 

		PIO_WF_PWM->SET(WF_PWM);


		//PWMTCC->PERBUF		= US2PWM(pwmPeriodUS);
		PWMTCC->PER			= US2PWM(pwmPeriodUS);
		//PWMTCC->CCBUF[0]	= US2PWM(pwmPeriodUS/2);
		PWMTCC->CC[0]		= US2PWM(pwmPeriodUS/2);

		waveLen = ((1000000 + waveFreq/2) / waveFreq + pwmPeriodUS/2) / pwmPeriodUS;

		const u16 FV = MAX(GetCurFireVoltage(), 25);

		if (pwm)
		{
			waveAmp = (waveFreq >= 5000) ? (waveAmp * 3 / 16) : (waveAmp/8);
			hi = US2PWM(pwmPeriodUS-0.5);
			mid = US2PWM(pwmPeriodUS/2);
			lo = US2PWM(0.5);
			amp = ((u32)waveAmp*US2PWM(pwmPeriodUS)+FV/2)/FV;
			dst = &(PWMTCC->CCBUF[0]);

			PIO_WF_PWM->EVCTRL.EV[0] = 0;
		}
		else
		{
			hi = 4095;
			mid = 0x7FF;
			lo = 0;
			amp = -(waveAmp*5/16);
			dst = &(HW::DAC->DATA[0]);
			
			HW::EVSYS->USER[EVSYS_USER_PORT_EV_0] = EVENT_PWM_SYNC+1;

			PIO_WF_PWM->EVCTRL.EV[0] = PORT_PORTEI|PORT_EVACT_CLR|PORT_PID(PIN_WF_PWM);
		};

		const u16 ki = 256 * ArraySize(sinArr) / waveLen;

		waveLen = waveLen * kLen / 2;

		for (u32 i = 0; i < waveLen; i++)
		{
			i16 t = mid + (amp*sinArr[(byte)((i*ki+127)>>8)])/2048;		
			waveBuffer[i] = LIM(t, lo, hi);
		};

		waveBuffer[waveLen] = mid;
	
		u16 waveLenMax = (waveTimeUS + pwmPeriodUS/2) / pwmPeriodUS;

		waveLenMax = MAX(waveLenMax, waveLen*2);

		PWMCOUNTTCC->PER = waveLenMax;
		PWMCOUNTTCC->CC[0] = waveLenMax;

		PWMCOUNTTCC->CTRLA |= TCC_ENABLE;

		PWMTCC->CTRLA |= TCC_ENABLE;

		PWM_DMA.WritePeripheral(waveBuffer, dst, waveLen+1, DMCH_TRIGACT_BURST|PWM_DMCH_TRIGSRC, DMDSC_BEATSIZE_HWORD); 
	}
	else
	{
		PIO_PWMHM->PINCFG[PIN_PWMHM] = 0;
		PIO_PWMLM->PINCFG[PIN_PWMLM] = 0;

		if (fireNum == 2)
		{
			PIO_PWMHX->PINCFG[PIN_PWMHX] = PINGFG_PMUXEN;
			PIO_PWMLX->PINCFG[PIN_PWMLX] = PINGFG_PMUXEN;
			PIO_PWMHY->PINCFG[PIN_PWMHY] = 0;
			PIO_PWMLY->PINCFG[PIN_PWMLY] = 0;
		}
		else if (fireNum == 3)
		{
			PIO_PWMHX->PINCFG[PIN_PWMHX] = 0;
			PIO_PWMLX->PINCFG[PIN_PWMLX] = 0;
			PIO_PWMHY->PINCFG[PIN_PWMHY] = PINGFG_PMUXEN;
			PIO_PWMLY->PINCFG[PIN_PWMLY] = PINGFG_PMUXEN;
		};

		u32 per = US2PWM((1000000+waveFreq/2)/waveFreq);
		u32 cc = per - (per * fireDuty + 5000) / 10000;

		//PWMTCC->PERBUF		= per;
		PWMTCC->PER			= per;
		//PWMTCC->CCBUF[0]	= cc;
		PWMTCC->CC[0]		= cc;

		PWMCOUNTTCC->PER	= fireCount-1;
		PWMCOUNTTCC->CC[0]	= fireCount-1;

		PWMCOUNTTCC->CTRLA |= TCC_ENABLE;

		PWMTCC->CTRLA |= TCC_ENABLE;
	}

	PIO_URXD0->SetWRCONFIG(URXD0, PORT_PMUX_A|PORT_WRPMUX|PORT_WRPINCFG|PORT_PMUXEN); 
	HW::EVSYS->CH[EVENT_PWM_SYNC].CHANNEL = (EVGEN_EIC_EXTINT_0+PWM_EXTINT)|EVSYS_PATH_ASYNCHRONOUS;
	//HW::EVSYS->SWEVT = 1;

	HW::PIOA->BCLR(21);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Init_PWM()
{
	PWM_ClockEnable();
	PWMCOUNT_ClockEnable();

	PIO_DRVEN->SET(DRVEN);

	PIO_PWMLX->SET(PWMLX);
	PIO_PWMLY->SET(PWMLY);
	PIO_PWMHX->CLR(PWMHX);
	PIO_PWMHY->CLR(PWMHY);
	PIO_PWMLM->CLR(PWMLM);
	PIO_PWMHM->CLR(PWMHM);

	PIO_DRVEN->DIRSET	= DRVEN;
	PIO_WF_PWM->DIRSET	= WF_PWM;
	PIO_PWMLX->DIRSET	= PWMLX;
	PIO_PWMLY->DIRSET	= PWMLY;
	PIO_PWMHX->DIRSET	= PWMHX;
	PIO_PWMHY->DIRSET	= PWMHY;
	PIO_PWMLM->DIRSET	= PWMLM;
	PIO_PWMHM->DIRSET	= PWMHM;
	PIO_POL->DIRSET		= POLWML|POLWMH;

	PIO_PWMLX->SetWRCONFIG(PWMLX, PMUX_PWMLX|PORT_WRPMUX|PORT_WRPINCFG|PORT_PMUXEN);
	PIO_PWMLY->SetWRCONFIG(PWMLY, PMUX_PWMLY|PORT_WRPMUX|PORT_WRPINCFG|PORT_PMUXEN);
	PIO_PWMHX->SetWRCONFIG(PWMHX, PMUX_PWMHX|PORT_WRPMUX|PORT_WRPINCFG|PORT_PMUXEN);
	PIO_PWMHY->SetWRCONFIG(PWMHY, PMUX_PWMHY|PORT_WRPMUX|PORT_WRPINCFG|PORT_PMUXEN);
	PIO_PWMLM->SetWRCONFIG(PWMLM, PMUX_PWMLM|PORT_WRPMUX|PORT_WRPINCFG|PORT_PMUXEN);
	PIO_PWMHM->SetWRCONFIG(PWMHM, PMUX_PWMHM|PORT_WRPMUX|PORT_WRPINCFG|PORT_PMUXEN);

	PIO_WF_PWM->SET(WF_PWM);
	PIO_POL->CLR(POLWML|POLWMH);

	PWMTCC->CTRLA = PWM_PRESC_DIV;
	PWMTCC->WAVE = TCC_WAVEGEN_NPWM;//|TCC_SWAP0;
	PWMTCC->DRVCTRL = TCC_NRE0|TCC_NRE1|TCC_NRE2|TCC_NRE3|TCC_NRE4|TCC_NRE5|TCC_NRV0|TCC_NRV1|TCC_NRV3|TCC_INVEN3;
	PWMTCC->WEXCTRL = TCC_OTMX(2)|TCC_DTIEN0|TCC_DTIEN1|TCC_DTLS(1)|TCC_DTHS(1); //0x01010F02;
	PWMTCC->PER = US2PWM(pwmPeriodUS)-1;
	PWMTCC->CCBUF[0] = US2PWM(pwmPeriodUS/2); 
	PWMTCC->EVCTRL = TCC_OVFEO|TCC_TCEI0|TCC_EVACT0_RETRIGGER;
	PWMTCC->INTENCLR = ~0;

	HW::GCLK->PCHCTRL[EVENT_PWM_SYNC+GCLK_EVSYS0] = GCLK_GEN(GEN_MCK)|GCLK_CHEN;
	HW::GCLK->PCHCTRL[EVENT_PWM+GCLK_EVSYS0] = GCLK_GEN(GEN_MCK)|GCLK_CHEN;
	HW::GCLK->PCHCTRL[EVENT_PWMCOUNT+GCLK_EVSYS0] = GCLK_GEN(GEN_MCK)|GCLK_CHEN;

	HW::EIC->CTRLA = 0; while(HW::EIC->SYNCBUSY);

	HW::EIC->EVCTRL |= EIC_EXTINT0<<PWM_EXTINT;
	HW::EIC->SetConfig(PWM_EXTINT, 0, EIC_SENSE_FALL);
	HW::EIC->INTENCLR = EIC_EXTINT0<<PWM_EXTINT;
	HW::EIC->CTRLA = EIC_ENABLE;

	HW::EVSYS->CH[EVENT_PWM_SYNC].CHANNEL = (EVGEN_EIC_EXTINT_0+PWM_EXTINT)|EVSYS_PATH_ASYNCHRONOUS;
	HW::EVSYS->USER[PWM_EVSYS_USER] = EVENT_PWM_SYNC+1;

	HW::EVSYS->CH[EVENT_PWM].CHANNEL = PWM_EVENT_GEN|EVSYS_PATH_ASYNCHRONOUS;
	HW::EVSYS->USER[PWMCOUNT0_EVSYS_USER] = EVENT_PWM+1;

	PWMCOUNTTCC->CTRLA = PWMCOUNT_PRESC_DIV;
	PWMCOUNTTCC->WAVE = TCC_WAVEGEN_NPWM;
	PWMCOUNTTCC->PER = waveLen;
	PWMCOUNTTCC->CC[0] = waveLen;
	PWMCOUNTTCC->EVCTRL = TCC_TCEI0|TCC_EVACT0_COUNT|TCC_TCEI1|TCC_EVACT1_RETRIGGER;
	PWMCOUNTTCC->INTENCLR = ~0;
	PWMCOUNTTCC->INTENSET = TCC_OVF;
	PWMCOUNTTCC->CTRLA |= TCC_ENABLE;
	PWMCOUNTTCC->CTRLBSET = TCC_ONESHOT;

	HW::EVSYS->CH[EVENT_PWMCOUNT].CHANNEL = PWMCOUNT_EVENT_GEN|EVSYS_PATH_ASYNCHRONOUS;
	HW::EVSYS->USER[PWMCOUNT1_EVSYS_USER] = EVENT_PWM_SYNC+1;

	VectorTableExt[PWMCOUNT_IRQ] = PwmCountIRQ;
	CM4::NVIC->CLR_PR(PWMCOUNT_IRQ);
	CM4::NVIC->SET_ER(PWMCOUNT_IRQ);

	HW::PIOA->SetWRCONFIG(1<<14,  PORT_PMUX_F|PORT_WRPMUX|PORT_WRPINCFG|PORT_PMUXEN); // PWMDMATCC WO[0]
	HW::PIOB->SetWRCONFIG(1<<16,  PORT_PMUX_F|PORT_WRPMUX|PORT_WRPINCFG|PORT_PMUXEN); // PWMCOUNTTCC WO[0]

	//DACTC_DMA.WritePeripheral(waveBuffer, HW::DAC

	const float k = 2*pi/waveLen;
	const u16 hi = US2PWM(pwmPeriodUS-0.5);
	const u16 mid = US2PWM(pwmPeriodUS/2);
	const u16 lo = US2PWM(0.5);
	const u16 amp = 0;//US2PWM(pwmPeriodUS);

	for (u32 i = 0; i < waveLen; i++) waveBuffer[i] = mid;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Init_HV()
{
	static DSCI2C dsc;
	static byte wbuf[4];
	static TM32 tm;

	wbuf[0] = 3;	
	wbuf[1] = 1;	
	wbuf[2] = 0;	

	dsc.adr = 0x48;
	dsc.wdata = wbuf;
	dsc.wlen = 3;
	dsc.rdata = 0;
	dsc.rlen = 0;
	dsc.wdata2 = 0;
	dsc.wlen2 = 0;

	I2C_AddRequest(&dsc);

	tm.Reset();	while (!dsc.ready && !tm.Check(10)) I2C_Update();

	wbuf[0] = 4;	
	wbuf[1] = 1;	
	wbuf[2] = 1;	

	dsc.adr = 0x48;
	dsc.wdata = wbuf;
	dsc.wlen = 3;
	dsc.rdata = 0;
	dsc.rlen = 0;
	dsc.wdata2 = 0;
	dsc.wlen2 = 0;

	I2C_AddRequest(&dsc);

	tm.Reset();	while (!dsc.ready && !tm.Check(10)) I2C_Update();

	wbuf[0] = 8;	
	wbuf[1] = ~0;
	wbuf[2] = ~0;

	dsc.adr = 0x48;
	dsc.wdata = wbuf;
	dsc.wlen = 3;
	dsc.rdata = 0;
	dsc.rlen = 0;
	dsc.wdata2 = 0;
	dsc.wlen2 = 0;

	I2C_AddRequest(&dsc);

	tm.Reset();	while (!dsc.ready && !tm.Check(10)) I2C_Update();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Init_WaveFormGen()
{
	WFG_ClockEnable();
	//DACTC_ClockEnable();

	PIO_GEN->DIRSET		= GENA|GENB;

	PIO_GEN->SetWRCONFIG(GENA, PMUX_GENA|PORT_WRPMUX|PORT_WRPINCFG|PORT_PMUXEN);
	PIO_GEN->SetWRCONFIG(GENB, PMUX_GENB|PORT_WRPMUX|PORT_WRPINCFG|PORT_PMUXEN);

	WFGTC->CTRLA = WFG_PRESC_DIV|TC_MODE_COUNT8;
	WFGTC->WAVE = TC_WAVEGEN_NPWM;
	WFGTC->DRVCTRL = TC_INVEN0 << GENB_WO_NUM;

	WFGTC->PER8 = US2WFG(pwmPeriodUS)-1;
	WFGTC->CC8[GENA_WO_NUM] = US2WFG(pwmPeriodUS/2); 
	WFGTC->CC8[GENB_WO_NUM] = US2WFG(pwmPeriodUS/2); 

	//WFGTC->EVCTRL = TC_TCEI|TC_EVACT_RETRIGGER;
	//HW::EVSYS->USER[WFG_EVSYS_USER] = EVENT_PWM_SYNC+1;

	WFGTC->INTENCLR = ~0;

	WFGTC->CTRLA |= TC_ENABLE;
	WFGTC->CTRLBSET = TC_CMD_RETRIGGER;


	PIO_DAC0->SetWRCONFIG(DAC0, PORT_PMUX_B|PORT_WRPMUX|PORT_WRPINCFG|PORT_PMUXEN);

	HW::GCLK->GENCTRL[GEN_DAC] = GCLK_DIV(GEN_DAC_DIV)|GCLK_SRC_DPLL0|GCLK_GENEN;
	HW::GCLK->PCHCTRL[GCLK_DAC] = GEN_DAC|GCLK_CHEN; 
	HW::MCLK->ClockEnable(PID_DAC); 

	HW::DAC->CTRLA = DAC_SWRST;

	while(HW::DAC->SYNCBUSY);

	HW::DAC->CTRLB = DAC_REFSEL_VREFAU;
	HW::DAC->EVCTRL = 0;
	HW::DAC->INTENCLR = ~0;
	HW::DAC->DACCTRL[0] = DAC_ENABLE|DAC_CC12M|DAC_RUNSTDBY|DAC_REFRESH(1);
	HW::DAC->DACCTRL[1] = 0;
	HW::DAC->CTRLA = DAC_ENABLE;

	while(HW::DAC->SYNCBUSY);

	HW::DAC->DATA[0] = 0x7FF;


	//DACTC->CTRLA = DACTC_PRESC_DIV|TC_MODE_COUNT8;
	//DACTC->WAVE = TC_WAVEGEN_NPWM;

	//DACTC->PER8 = US2DACTC(10)-1;
	//DACTC->EVCTRL = 0;

	//DACTC->INTENCLR = ~0;

	//DACTC->CTRLA |= TC_ENABLE;
	//DACTC->CTRLBSET = TC_CMD_RETRIGGER;

	const float k = 2*pi/ArraySize(sinArr);

	for (u32 i = 0; i < ArraySize(sinArr); i++)
	{
		sinArr[i] = 1861*sin(i*k);		
	};

}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Init_ADC()
{
	HW::GCLK->PCHCTRL[GCLK_ADC0] = GEN_MCK|GCLK_CHEN; 
	HW::GCLK->PCHCTRL[GCLK_ADC1] = GEN_MCK|GCLK_CHEN; 
	HW::MCLK->ClockEnable(PID_ADC0); 
	HW::MCLK->ClockEnable(PID_ADC1); 

	PIO_ADC800->SetWRCONFIG(ADC800, PORT_PMUX_B|PORT_WRPMUX|PORT_WRPINCFG|PORT_PMUXEN);
	PIO_ADCWFB->SetWRCONFIG(ADCWFB, PORT_PMUX_B|PORT_WRPMUX|PORT_WRPINCFG|PORT_PMUXEN);

	HW::ADC0->CTRLB = ADC_RESSEL_16BIT|ADC_FREERUN;
	HW::ADC1->CTRLB = ADC_RESSEL_12BIT|ADC_FREERUN;

	HW::ADC0->REFCTRL = ADC_REFSEL_VDDANA|ADC_REFCOMP;
	HW::ADC1->REFCTRL = ADC_REFSEL_VDDANA|ADC_REFCOMP;

	HW::ADC0->INPUTCTRL = ADC_MUXPOS_AIN7;
	HW::ADC1->INPUTCTRL = ADC_MUXPOS_AIN6;

	HW::ADC0->AVGCTRL = ADC_SAMPLENUM_1024;
	HW::ADC1->AVGCTRL = ADC_SAMPLENUM_1;

	HW::ADC0->SAMPCTRL = ADC_SAMPLEN(0);
	HW::ADC1->SAMPCTRL = ADC_SAMPLEN(0);

	HW::ADC0->CTRLA = ADC_PRESCALER_DIV16|ADC_ENABLE; 
	HW::ADC1->CTRLA = ADC_PRESCALER_DIV16|ADC_ENABLE; 

	HW::ADC0->SWTRIG = ADC_START;
	HW::ADC1->SWTRIG = ADC_START;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u16 GetCurFireVoltage()
{
	return (HW::ADC0->RESULT * 37361) >> 21;
}

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
	//Init_HV();

	RTT_Init();

#ifndef WIN32

	#ifdef MANCH_REQ
		InitManRecieve();
		InitManTransmit();
	#endif

	EnableVCORE();
	
	Init_ADC();
	Init_PWM();
	Init_WaveFormGen();
	Init_WFMOSC();

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

	if (HW::DAC->SYNCBUSY == 0 && pwmstat) HW::DAC->DATA[0] = 0x7ff;

	Update_WFMOSC();

#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
