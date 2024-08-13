#ifndef G_TRM_H__27_12_2023__11_23
#define G_TRM_H__27_12_2023__11_23

#pragma once

#include "types.h"

#if defined(__CC_ARM) || defined(_MSC_VER)

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define TRM_BOOT_SGUID		0X66C41909FA7C4F91 
#define TRM_BOOT_MGUID		0x84F14C8C797FC02C

#define TRM_COM_BAUDRATE	1250000
#define TRM_COM_PARITY		2
#define TRM_COM_STOPBITS	2

#define TRM_BOOT_REQ_WORD	0X6600
#define TRM_BOOT_REQ_MASK	0xFF00
#define TRM_BOOT_NET_ADR	1

#define TRM_RSP03_RW		0xAA72

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifndef BOOTLOADER

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct ReqTrm01	
{
	__packed struct Req
	{
		byte 	len;
		byte 	func;
		byte 	n;
		byte	fireCount;
		u16		fireFreq;
		u16		fireDuty;
		u16		fireAmp;
		word 	crc;
	}
	r[3];
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct ReqTrm02	
{
	__packed struct Req
	{
		byte 	len;
		byte 	func;
		byte	numDevValid;		// если не ноль, numDev пральный и его нада записать в RAM
		byte 	saveParams;			// если не ноль, то записать параметры во flash
		u16		reqHV;
		u16		numDev;				// номер модуля приёмников
		u16 	crc;  
	}
	r[2];
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct RspTrm02	
{
	byte	func; 
	byte	numDevValid;	// если не ноль, numDev считан из flash правильно или установлен запросом
	u16		numdev;			// номер модуля приёмников
	u16		verdev; 		// версия ПО модуля приёмников
	u16		hv; 
	u16		temp;
	u16		crc;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct ReqTrm03	// Запрос осциллограмм излучателя
{
	__packed struct Req
	{
		byte 	len;
		byte	func; 
		word 	crc;  
	}
	r[2];
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct RspTrm03	
{
	u16		rw;			//1. Ответное слово
	u16		num;		//2. Номер излучателя (0 - Монополь 1, 1 - Монополь 2)
	u16		amp;   		//3. Аплитуда излучателя измеренная (В)
	u16		st;			//4. Шаг, мкс
	u16		sl;  		//5. Длина, отсч
	u16		sd; 		//6. Задержка мкс
	i16		data[128];	//7-?. данные (до 128шт)
 
	u16		crc;	
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//__packed struct ReqMoto
//{
//	u16 	rw;
//	u16 	enableMotor; 
//	u32		tRPM;		// время 1/6 оборота двигателя в мкс
//	u16		limCurrent; // Ограничение тока двигателя (мА)
//	u16		maxCurrent; // Аварийный ток двигателя (мА)
//	u16 	crc;  
//};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//__packed struct RspMoto
//{
//	u16 	rw;
//	u16 	mororStatus; 
//	u16		current;
//	u16		currentLow;
//	u16		rpm;
//	u16		motoCounter;
//	u16		auxVoltage;
//	u16		motoVoltage;
//	u16 	crc;  
//};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//__packed struct ReqBootMotoHS { unsigned __int64 guid; u16 crc; };
//__packed struct RspBootMotoHS { unsigned __int64 guid; u16 crc; };

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//struct ReqBootMoto
//{
//	union
//	{
//		struct { u32 func; u32 len;								u16 align; u16 crc; }	F1; // Get CRC
//		struct { u32 func;										u16 align; u16 crc; }	F2; // Exit boot loader
//		struct { u32 func; u32 padr; u32 plen; u32 pdata[16];	u16 align; u16 crc; }	F3; // Programm page
//	};
//};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//struct RspBootMoto
//{
//	union
//	{
//		struct { u32 func; u32 pageLen;	u32 len;	u16 sCRC;	u16 crc; }	F1; // Get CRC
//		struct { u32 func;							u16 align;	u16 crc; } 	F2; // Exit boot loader
//		struct { u32 func; u32 padr;	u32 status; u16 align;	u16 crc; } 	F3; // Programm page
//	};
//};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#endif // BOOTLOADER

#endif // __CC_ARM


#endif //G_TRM_H__27_12_2023__11_23
