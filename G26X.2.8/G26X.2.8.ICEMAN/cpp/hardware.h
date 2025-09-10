#ifndef HARDWARE_H__23_12_2013__11_37
#define HARDWARE_H__23_12_2013__11_37

#include <types.h>
//#include "core.h"
#include <time.h>
#include <i2c.h>
//#include "hw_nand.h"
#include "hw_rtm.h"
#include "MANCH\manch.h"
#include "G26X_2_8_HW_CONF.H"
//#include "G_TRM.h"

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//struct HdrRsp20
//{
//	u16		rw;
//	u16		fireNum;
//	u16		voltage;
//	u16		amp;
//	u16		temp;
//	u16		st;
//	u16		sl;
//	u16		sd;
//};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//extern	Rsp72*	GetReadyRsp72();
//extern	void	FreeRsp72(Rsp72 *rsp);
//extern	void	PrepareFire(u16 fireNum, u16 waveFreq, u16 waveAmp, u16 fireCount, u16 fireDuty);

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

extern bool IsFireOK();
extern void DisableFire();

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

extern void InitHardware();
extern void UpdateHardware();
//extern u16 GetCurFireVoltage();

//inline bool DacHvInverted() { extern bool dachvInverted; return dachvInverted; }

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//__forceinline u32 Push_IRQ()
//{
//	register u32 t;
//
//#ifndef WIN32
//
//	register u32 primask __asm("primask");
//
//	t = primask;
//
//	__disable_irq();
//
//#endif
//
//	return t;
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//__forceinline void Pop_IRQ(u32 t)
//{
//#ifndef WIN32
//
//	register u32 primask __asm("primask");
//
//	primask = t;
//
//#endif
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef WIN32

extern void I2C_Destroy();
extern void SPI_Destroy();
extern void UpdateDisplay();
extern int PutString(u32 x, u32 y, byte c, const char *str);
extern int Printf(u32 x, u32 y, byte c, const char *format, ... );

extern void NAND_FlushBlockBuffers();
extern void NAND_ReqFlushBlockBuffers();

#endif

#endif // HARDWARE_H__23_12_2013__11_37
