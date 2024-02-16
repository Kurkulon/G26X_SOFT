#ifndef G_RCV_H__27_12_2023__11_22
#define G_RCV_H__27_12_2023__11_22

#pragma once

//#include "types.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define RCV_MAX_NUM_STATIONS 13

#define RCV_BOOT_SGUID			0X66C41909FA7C4F91 
#define RCV_BOOT_REQ_WORD		((~(RCV_MAN_REQ_WORD)) & RCV_MAN_REQ_MASK)
#define RCV_BOOT_REQ_MASK		RCV_MAN_REQ_MASK
#define RCV_BOOT_COM_BAUDRATE	1250000
#define RCV_BOOT_COM_PARITY		2


#define RCV_COM_BAUDRATE		12500000
#define RCV_COM_PARITY			2

#define RCV_MAN_REQ_WORD 		0xAA00
#define RCV_MAN_REQ_MASK 		0xFF00

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

#define TRANSMITER_NUM	3	// 0 - Monopole1, 1 - Monopole1, 2 - DipoleXY
#define RCV_FIRE_NUM	4	// 0 - Mnpl_1, 1 - Mnpl_2, 2 - DplX, 3 - DplY

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct ReqRcv01	// ����� ���������
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

__packed struct RspRcv01	// ����� ���������
{
	byte adr;
	byte func;
	word crc;  
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct ReqRcv02	// ������ �������
{
	__packed struct Req
	{
		byte 	len;
		byte 	adr;
		byte 	func;
		byte 	n; 
		word 	crc; 
	}
	r[2];
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct RspRcvHdr02	// ������ �������
{
	u16 rw;
	u32 cnt;
	u16 gain;
	u16 st;
	u16 len;
	u16 delay;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct RspRcv02	// ������ �������
{
	RspRcvHdr02 hdr;

	u16 data[1024*4]; 
	u16 crc;
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct  ReqRcv03	// ��������� ������� ������������� ������� � ������������ ��������
{ 
	__packed struct Req
	{
		byte 	len;
		byte 	adr;
		byte 	func;
		u16 	st[RCV_FIRE_NUM]; 
		u16		sl[RCV_FIRE_NUM]; 
		u16		sd[RCV_FIRE_NUM];
		word	crc; 
	}
	r[2];
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct  RspRcv03	// ��������� ������� ������������� ������� � ������������ ��������
{ 
	byte adr;
	byte func;
	word crc; 
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct  ReqRcv04	// ��������� ������������ ��������
{ 
	__packed struct Req
	{
		byte 	len;
		byte 	adr;
		byte 	func;
		byte 	ka[RCV_FIRE_NUM]; 
		word 	crc; 
	}
	r[2];
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct  RspRcv04	// ��������� ������� ������������� ������� � ������������ ��������
{ 
	byte	adr;
	byte	func;
	u16 	maxAmp[4];
	u16		power[4];
	word	crc; 
};  

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
};
	
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef _ADI_COMPILER
#pragma pack()
//#undef __packed
#endif



#endif //G_RCV_H__27_12_2023__11_22
