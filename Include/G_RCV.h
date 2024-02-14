#ifndef G_RCV_H__27_12_2023__11_22
#define G_RCV_H__27_12_2023__11_22

#pragma once

//#include "types.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define RCV_MAX_NUM_STATIONS 13

#define RCV_BOOT_SGUID		0X66C41909FA7C4F91 

#define RCV_COM_BAUDRATE	12500000
#define RCV_COM_PARITY		2

#define RCV_MAN_REQ_WORD 0xAA00
#define RCV_MAN_REQ_MASK 0xFF00

#define RCV_BOOT_REQ_WORD	((~(RCV_MAN_REQ_WORD)) & RCV_MAN_REQ_MASK)
#define RCV_BOOT_REQ_MASK	RCV_MAN_REQ_MASK

#define RCV_FltResist(v)	(((v) * 941 + 2048) / 4096)
#define RCV_NetResist(v)	(((v) * 941 + 128) / 256)
#define RCV_NetAdr(v)		(1 + (v)/1024)

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef _ADI_COMPILER
	#pragma pack(1)
	//#ifndef __packed
	//	#define __packed /**/
	//#endif
#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define TRANSMITER_NUM	2	// 0 - Monopole, 1 - DipoleXY

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct ReqRcv01	// старт оцифровки
{
	__packed struct Req
	{
		byte 	len;
		byte 	adr;
		byte 	func;
		byte 	n; 
		byte 	next_n; 
		u16		fc;		// fire count
		word 	crc;  
	}
	r[3];
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct RspRcv01	// старт оцифровки
{
	byte adr;
	byte func;
	word crc;  
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct ReqRcv02	// чтение вектора
{
	__packed struct Req
	{
		byte 	len;
		byte 	adr;
		byte 	func;
		byte 	n; 
		byte 	chnl; 
		word 	crc; 
	}
	r[2];
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct RspRcvHdr02	// чтение вектора
{
	u16 rw;
	u32 cnt;
	u16 gain;
	u16 st;
	u16 len;
	u16 delay;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct RspRcv02	// чтение вектора
{
	RspRcvHdr02 hdr;

	u16 data[1024*4]; 
	u16 crc;
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct  ReqRcv03	// установка периода дискретизации вектора и коэффициента усиления
{ 
	__packed struct Req
	{
		byte 	len;
		byte 	adr;
		byte 	func;
		u16 	st[3]; 
		u16		sl[3]; 
		u16		sd[3];
		word	crc; 
	}
	r[2];
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct  RspRcv03	// установка периода дискретизации вектора и коэффициента усиления
{ 
	byte adr;
	byte func;
	word crc; 
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct  ReqRcv04	// установка коэффициента усиления
{ 
	__packed struct Req
	{
		byte 	len;
		byte 	adr;
		byte 	func;
		byte 	ka[3]; 
		word 	crc; 
	}
	r[2];
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct  RspRcv04	// установка периода дискретизации вектора и коэффициента усиления
{ 
	byte	adr;
	byte	func;
	u16 	maxAmp[4];
	u16		power[4];
	word	crc; 
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//__packed struct  ReqRcv05	// запрос контрольной суммы и длины программы во флэш-памяти
//{ 
//	byte 	len;
//	byte 	adr;
//	byte 	func;
//	word 	crc; 
//};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//__packed struct  RspRcv05	// запрос контрольной суммы и длины программы во флэш-памяти
//{ 
//	byte 	adr;
//	byte 	func;
//	u16		flashLen; 
//	u16		flashCRC;
//	word 	crc; 
//};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//__packed struct  ReqRcv06	// запись страницы во флэш
//{ 
//	byte 	len;
//	byte 	adr;
//	byte 	func;
//	u16		stAdr; 
//	u16		count; 
//	word	crc; 
//	byte	data[258]; 
//};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//__packed struct  RspRcv06	// запись страницы во флэш
//{ 
//	byte 	adr;
//	byte 	func;
//	u16		res; 
//	word	crc; 
//};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//__packed struct  ReqRcv07	// перезагрузить блэкфин
//{ 
//	byte 	len;
//	byte 	adr;
//	byte 	func;
//	word 	crc; 
//};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct Transmiter
{
	u16 gain;
	u16 st;	//sampleTime;
	u16 sl;	//sampleLen;
	u16 sd;	//sampleDelay;
	u16 freq;
	u16 duty;
	u16 amp;
	u16 pulseCount;
	u16 packType;
};
	
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//__packed struct ReqDsp01	// чтение вектора
//{
//	enum { VERSION = 1 };
//
//	u16 	rw;
//	u16		len;				// Длина структуры
//	u16		version;			// Версия структуры
//
//	//u16		com_spi;			// 0 - ответ по UART, 1 - ответ по SPI
//
//	u16 	mode; 
//	u16		ax; 
//	u16		ay; 
//	u16		az; 
//	u16		at;
//	
//	SENS	sens[SENS_NUM];		//SENS	sens1;
//								//SENS	sens2;
//								//SENS	refSens;
//	u16		wavesPerRoundCM;
//	u16		wavesPerRoundIM;
//	u16		fireVoltage;		// Напряжение излучателя (0.1В)
//	u16		sensMask;
//
//	u16 	crc;  
//};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//__packed struct RspHdrCM	// 0xAD40
//{
//	u16 	rw;
//	u32 	time;		//mmsecTime; 
//	u32		hallTime;	//shaftTime; 
//	u16		motoCount; 
//	u16		headCount;
//	u16		ax; 
//	u16		ay; 
//	u16		az; 
//	u16		at;
//	u16		sensType; 
//	u16		angle;
//	u16		maxAmp;
//	u16		fi_amp;
//	u16		fi_time;
//	u16 	gain; 
//	u16 	st;	 
//	u16 	sl; 
//	u16 	sd; 
//	u16		packType;
//	u16		packLen;
//};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//__packed struct RspHdrIM	// 0xAD50
//{
//	u16 	rw;
//	u32 	time;		//mmsecTime; 
//	u32		hallTime;	//shaftTime; 
//	u16		ax; 
//	u16		ay; 
//	u16		az; 
//	u16		at;
//	u16		sensType; 
//	u16 	gain; 
//	u16		refAmp;
//	u16		refTime;
//	u16		dataLen;
//};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//__packed union RspDsp01	// чтение вектора
//{
//	__packed struct { RspHdrCM hdr; u16 data[512]; } CM;
//	__packed struct { RspHdrIM hdr; u16 data[512]; } IM;
//	__packed struct { u16 rw; u16 len; u16 version; u16 fireVoltage; u16 motoVoltage; u16 crc; } v01;
//};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//__packed struct  ReqDsp05	// запрос контрольной суммы и длины программы во флэш-памяти
//{ 
//	u16		rw; 
//	u16 	crc; 
//};  
//
////+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//__packed struct  RspDsp05	// запрос контрольной суммы и длины программы во флэш-памяти
//{ 
//	u16		rw; 
//	u16		flashLen; 
//	u32		startAdr; 
//	u16		flashCRC; 
//	u16		crc;
//};  
//
////+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//__packed struct  ReqDsp06	// запись страницы во флэш
//{ 
//	u16		rw; 
//	u16		stAdr; 
//	u16		count; 
//	byte	data[258]; 
//	u16		crc; 
//};  
//
////+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//__packed struct  RspDsp06	// запись страницы во флэш
//{ 
//	u16		rw; 
//	u16		res; 
//	word	crc; 
//};  
//
////+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//__packed struct  ReqDsp07	// перезагрузить блэкфин
//{ 
//	u16		rw; 
//	word 	crc; 
//};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef _ADI_COMPILER
#pragma pack()
//#undef __packed
#endif



#endif //G_RCV_H__27_12_2023__11_22
