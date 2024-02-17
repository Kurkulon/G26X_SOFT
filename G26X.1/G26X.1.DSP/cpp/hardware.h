#ifndef HARDWARE_H__15_05_2009__14_35
#define HARDWARE_H__15_05_2009__14_35
  
#include "hw_conf.h"
#include "G_RCV.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct DSCSPORT
{
	DSCSPORT	*next;
	u32			vectorCount;
	u16			readyMask;
	u16			fireN;
	u16			gain;
	u16			sl;
	u16			st;
	u16			sd;
	u16			next_fireN;
	u16			next_gain;
#ifdef RCV_WAVEPACK
	u16			packType;
#endif
	u16			spd[2][1024*2];
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

extern void InitHardware();
extern void UpdateHardware();

//inline u32 GetRTT() { return *pTIMER0_COUNTER; }

extern void SyncReadSPORT(DSCSPORT *dsc);
//extern void ReadSPORT(void *dst1, void *dst2, u16 len1, u16 len2, u16 clkdiv, bool *ready0, bool *ready1);
//inline void WritePGA(u16 v) { extern u16 pgaValue; extern bool pgaSet; pgaValue = v; pgaSet = true; }
//inline void SetGain(byte g) { WritePGA(0x2A01|(g<<4)); }
//inline u16 GetADC() { extern U32u adcValue; return adcValue.w[1]; }
//inline void EnableADC() { extern bool adcEnable; adcEnable = true; }
//inline void DisableADC() { extern bool adcEnable; adcEnable = false; }

extern u16 GetNetAdr();
extern DSCSPORT* GetDscSPORT();
extern void FreeDscSPORT(DSCSPORT* dsc);
extern DSCSPORT* AllocDscSPORT();

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//struct TM32
//{
//	u32 pt;
//
//	TM32() : pt(0) {}
//	bool Check(u32 v) { if ((GetRTT() - pt) >= v) { pt = GetRTT(); return true; } else { return false; }; }
//	void Reset() { pt = GetRTT(); }
//};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


#endif // HARDWARE_H__15_05_2009__14_35
