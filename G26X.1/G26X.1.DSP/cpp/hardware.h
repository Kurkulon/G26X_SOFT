#ifndef HARDWARE_H__15_05_2009__14_35
#define HARDWARE_H__15_05_2009__14_35
  
#include "hw_conf.h"
#include "G_RCV.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//struct DSCSPORT
//{
//	DSCSPORT	*next;
//	u32			vectorCount;
//	u16			readyMask;
//	u16			fireN;
//	u16			gain;
//	u16			sl;
//	u16			st;
//	u16			sd;
//	u16			next_fireN;
//	u16			next_gain;
//	u16			packType;
//	u16			math;
//
//	u16			spd[RCV_SAMPLE_LEN*4];
//};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct DSCRSP02
{
	DSCRSP02 *next;

	u16			len;
	u16			readyMask;
	u16			fireN;
	u16			next_fireN;
	u16			next_gain;
	u16			sportLen;

	//u32			vectorCount;
	//u16			gain;
	//u16			sl;
	//u16			st;
	//u16			sd;
	//u16			packType;
	//u16			math;

	union 
	{
		RspRcv02 r02;
		u16 data[sizeof(RspRcv02)/2];
	};
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

extern void InitHardware();
extern void UpdateHardware();

//inline u32 GetRTT() { return *pTIMER0_COUNTER; }

extern void SyncReadSPORT(DSCRSP02 *dsc, u16 delay);
//extern void ReadSPORT(void *dst1, void *dst2, u16 len1, u16 len2, u16 clkdiv, bool *ready0, bool *ready1);
//inline void WritePGA(u16 v) { extern u16 pgaValue; extern bool pgaSet; pgaValue = v; pgaSet = true; }
//inline void SetGain(byte g) { WritePGA(0x2A01|(g<<4)); }
//inline u16 GetADC() { extern U32u adcValue; return adcValue.w[1]; }
//inline void EnableADC() { extern bool adcEnable; adcEnable = true; }
//inline void DisableADC() { extern bool adcEnable; adcEnable = false; }

extern u16 GetNetAdr();
extern DSCRSP02* GetDscSPORT();
extern void FreeDscSPORT(DSCRSP02* dsc);
extern DSCRSP02* AllocDscSPORT();

extern u16 GetTemp();
extern void SetGain(u16 v, bool preAmp);


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
