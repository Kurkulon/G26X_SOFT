#ifndef HW_RTM_H__20_04_2022__16_00
#define HW_RTM_H__20_04_2022__16_00

#ifdef WIN32
#include <windows.h>
#else
#include "core.h"
#endif

#include "types.h"

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

extern void RTT_Init();

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef WIN32	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


extern LARGE_INTEGER queryPerformanceFrequency;

#define US2RT(x) (((u32)(x) * queryPerformanceFrequency.LowPart + 500000UL) / 1000000UL)
#define MS2RT(x) (((u32)(x) * queryPerformanceFrequency.LowPart + 500UL) / 1000UL)

inline u32 GetRTT() 
{ 
	LARGE_INTEGER t;

	QueryPerformanceCounter(&t);

	return t.LowPart; 
}

struct RTM
{
	u32 pt;

//	RTM32() : pt(0) {}
	bool Check(u32 v) { u32 t = GetRTT(); if ((t - pt) >= v) { pt = t; return true; } else { return false; }; }
	bool Timeout(u32 v) { return (GetRTT() - pt) >= v; }
	void Reset() { pt = GetRTT(); }
};

#elif defined(CPU_SAME53)	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


#define US2RT(x) ((x*32768+500000)/1000000)
#define MS2RT(x) ((x*32768+500)/1000)

inline u32 GetRTT() { /*while(HW::RTC->SYNCBUSY & RTC_COUNT);*/ return HW::RTC->COUNT; }

struct RTM
{
	u32 pt;

//	RTM32() : pt(0) {}
	bool Check(u32 v) { u32 t = GetRTT(); if ((t - pt) >= v) { pt = t; return true; } else { return false; }; }
	bool Timeout(u32 v) { return (GetRTT() - pt) >= v; }
	void Reset() { pt = GetRTT(); }
};

#elif defined(CPU_XMC48)	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define US2RT(x) (((x)*MCK_MHz+1024)/2048)
#define MS2RT(x) (((x)*MCK_MHz*1000+1024)/2048)

inline u16 GetRTT() { return HW::CCU43_CC43->TIMER; }

struct RTM
{
	u16 pt;

	//RTM16() : pt(0) {}
	bool Check(u16 v) { if ((u16)(GetRTT() - pt) >= v) { pt = GetRTT(); return true; } else { return false; }; }
	bool Timeout(u16 v) { return (u16)(GetRTT() - pt) >= v; }
	void Reset() { pt = GetRTT(); }
};

#endif	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



#endif // HW_RTM_H__20_04_2022__16_00
