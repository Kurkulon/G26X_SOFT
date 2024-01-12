#include "time.h"
#include "core.h"

#include "SEGGER_RTT.h"

#ifdef WIN32

#include <windows.h>
#include <time.h>

LARGE_INTEGER queryPerformanceFrequency = { 0, 0 };

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void RTT_Init()
{
#ifndef WIN32

	using namespace HW;

	#ifdef CPU_SAME53	

		HW::OSC32KCTRL->RTCCTRL = OSC32_RTCSEL_ULP32K;

		HW::MCLK->APBAMASK |= APBA_RTC;

		HW::RTC->CTRLA = 0;

		HW::RTC->CTRLA = RTC_SWRST;

		while(HW::RTC->CTRLA & RTC_SWRST);

		HW::RTC->CTRLA = RTC_MODE_COUNT32|RTC_COUNTSYNC|RTC_PRESC_DIV1;
		HW::RTC->COUNT = 0;
		HW::RTC->CTRLA |= RTC_ENABLE;

		while(HW::RTC->SYNCBUSY != 0);

		u32 t = HW::RTC->COUNT;

		while(t == HW::RTC->COUNT);

	#elif defined(CPU_XMC48)

		T_HW::CCU4_GLOBAL_Type * const module = HW::CCU43;
		T_HW::CCU4_CC4_Type * const slice = HW::CCU43_CC43;

		CCU_Enable(PID_CCU43);

		module->GCTRL = 0;

		module->GIDLC = CCU4_GIDLC_CS3I_Msk|CCU4_GIDLC_SPRB_Msk;

		slice->PRS = 0xFFFF;
		slice->PSC = 11; //20.48us

		module->GCSS = CCU4_GCSS_S3SE_Msk;  

		slice->TCSET = 1;

	#endif

	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_GREEN "RTT Init ... \n");

#else

	if (!QueryPerformanceFrequency(&queryPerformanceFrequency))
	{
		queryPerformanceFrequency.LowPart = 1;
	};

#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

