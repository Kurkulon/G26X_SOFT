#ifndef HARDWARE_H__23_12_2013__11_37
#define HARDWARE_H__23_12_2013__11_37

#include "types.h"
//#include "core.h"
#include "time.h"
#include "i2c.h"
#include "spi.h"
#include "hw_nand.h"
#include "hw_rtm.h"
#include "MANCH\manch.h"
#include "spis.h"
#include "clock.h"

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef DSPSPI
extern S_SPIS	spidsp;
#endif


extern void Hardware_Init();

extern void UpdateHardware();

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

extern void InitHardware();
extern void UpdateHardware();



//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef WIN32

extern void I2C_Destroy();
extern void SPI_Destroy();
extern void UpdateDisplay();
extern int PutString(u32 x, u32 y, byte c, const char *str);
extern int Printf(u32 x, u32 y, byte c, const char *format, ... );

//extern void NAND_FlushBlockBuffers();
//extern void NAND_ReqFlushBlockBuffers();

#endif

#endif // HARDWARE_H__23_12_2013__11_37
