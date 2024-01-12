#ifndef G_RCV_H__27_12_2023__11_22
#define G_RCV_H__27_12_2023__11_22

#pragma once

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef __CC_ARM

#else
	#pragma pack(1)
	#ifndef __packed
		#define __packed
	#endif
#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define SENS_NUM	3

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct SENS
{
	u16 gain;
	u16 st;	//sampleTime;
	u16 sl;	//sampleLen;
	u16 sd;	//sampleDelay;
	u16 deadTime;
	u16 descriminant;
	u16 freq;
	u16 filtrType;
	u16 packType;
	u16 fi_Type;
	u16 fragLen;
};
	
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct ReqDsp01	// чтение вектора
{
	enum { VERSION = 1 };

	u16 	rw;
	u16		len;				// Длина структуры
	u16		version;			// Версия структуры

	//u16		com_spi;			// 0 - ответ по UART, 1 - ответ по SPI

	u16 	mode; 
	u16		ax; 
	u16		ay; 
	u16		az; 
	u16		at;
	
	SENS	sens[SENS_NUM];		//SENS	sens1;
								//SENS	sens2;
								//SENS	refSens;
	u16		wavesPerRoundCM;
	u16		wavesPerRoundIM;
	u16		fireVoltage;		// Напряжение излучателя (0.1В)
	u16		sensMask;

	u16 	crc;  
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct RspHdrCM	// 0xAD40
{
	u16 	rw;
	u32 	time;		//mmsecTime; 
	u32		hallTime;	//shaftTime; 
	u16		motoCount; 
	u16		headCount;
	u16		ax; 
	u16		ay; 
	u16		az; 
	u16		at;
	u16		sensType; 
	u16		angle;
	u16		maxAmp;
	u16		fi_amp;
	u16		fi_time;
	u16 	gain; 
	u16 	st;	 
	u16 	sl; 
	u16 	sd; 
	u16		packType;
	u16		packLen;
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct RspHdrIM	// 0xAD50
{
	u16 	rw;
	u32 	time;		//mmsecTime; 
	u32		hallTime;	//shaftTime; 
	u16		ax; 
	u16		ay; 
	u16		az; 
	u16		at;
	u16		sensType; 
	u16 	gain; 
	u16		refAmp;
	u16		refTime;
	u16		dataLen;
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed union RspDsp01	// чтение вектора
{
	__packed struct { RspHdrCM hdr; u16 data[512]; } CM;
	__packed struct { RspHdrIM hdr; u16 data[512]; } IM;
	__packed struct { u16 rw; u16 len; u16 version; u16 fireVoltage; u16 motoVoltage; u16 crc; } v01;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct  ReqDsp05	// запрос контрольной суммы и длины программы во флэш-памяти
{ 
	u16		rw; 
	u16 	crc; 
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct  RspDsp05	// запрос контрольной суммы и длины программы во флэш-памяти
{ 
	u16		rw; 
	u16		flashLen; 
	u32		startAdr; 
	u16		flashCRC; 
	u16		crc;
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct  ReqDsp06	// запись страницы во флэш
{ 
	u16		rw; 
	u16		stAdr; 
	u16		count; 
	byte	data[258]; 
	u16		crc; 
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct  RspDsp06	// запись страницы во флэш
{ 
	u16		rw; 
	u16		res; 
	word	crc; 
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct  ReqDsp07	// перезагрузить блэкфин
{ 
	u16		rw; 
	word 	crc; 
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef __CC_ARM

#else
#pragma pack()
#undef __packed
#endif



#endif //G_RCV_H__27_12_2023__11_22
