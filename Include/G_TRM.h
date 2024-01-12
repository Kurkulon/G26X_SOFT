#ifndef G_TRM_H__27_12_2023__11_23
#define G_TRM_H__27_12_2023__11_23

#pragma once

#ifdef __CC_ARM

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define TRM_BOOT_SGUID	0X66C41909FA7C4F91 
#define TRM_BOOT_MGUID	0x84F14C8C797FC02C

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifndef BOOTLOADER

__packed struct ReqMoto
{
	u16 	rw;
	u16 	enableMotor; 
	u32		tRPM;		// время 1/6 оборота двигателя в мкс
	u16		limCurrent; // Ограничение тока двигателя (мА)
	u16		maxCurrent; // Аварийный ток двигателя (мА)
	u16 	crc;  
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct RspMoto
{
	u16 	rw;
	u16 	mororStatus; 
	u16		current;
	u16		currentLow;
	u16		rpm;
	u16		motoCounter;
	u16		auxVoltage;
	u16		motoVoltage;
	u16 	crc;  
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct ReqBootMotoHS { unsigned __int64 guid; u16 crc; };
__packed struct RspBootMotoHS { unsigned __int64 guid; u16 crc; };

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct ReqBootMoto
{
	union
	{
		struct { u32 func; u32 len;								u16 align; u16 crc; }	F1; // Get CRC
		struct { u32 func;										u16 align; u16 crc; }	F2; // Exit boot loader
		struct { u32 func; u32 padr; u32 plen; u32 pdata[16];	u16 align; u16 crc; }	F3; // Programm page
	};
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct RspBootMoto
{
	union
	{
		struct { u32 func; u32 pageLen;	u32 len;	u16 sCRC;	u16 crc; }	F1; // Get CRC
		struct { u32 func;							u16 align;	u16 crc; } 	F2; // Exit boot loader
		struct { u32 func; u32 padr;	u32 status; u16 align;	u16 crc; } 	F3; // Programm page
	};
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#endif // BOOTLOADER

#endif // __CC_ARM


#endif //G_TRM_H__27_12_2023__11_23
