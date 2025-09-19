#ifndef G_RCV_H__27_12_2023__11_22
#define G_RCV_H__27_12_2023__11_22

#pragma once 

#include "types.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef _ADI_COMPILER
	#pragma pack(1)
	//#ifndef __packed
	//	#define __packed /**/
	//#endif
#else

	//#define RCV_8AD

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define RCV_AUTO_GAIN
#define RCV_AUTO_GAIN_LO_AMP	8192
#define RCV_AUTO_GAIN_HI_AMP	24576

#define RCV_MAN_VEC_TIMOUT		10000  //ms

#define RCV_MAX_NUM_STATIONS	13

#ifndef RCV_8AD
	#define RCV_COM_BAUDRATE		6250000
	#define RCV_COM_PARITY			2
#else
	#define RCV_COM_BAUDRATE		6250000
	#define RCV_COM_PARITY			0
#endif

#define RCV_BOOT_SGUID			0X66C41909FA7C4F91 
#define RCV_BOOT_REQ_WORD		((~(0xAA00)) & RCV_MAN_REQ_MASK)
#define RCV_BOOT_REQ_MASK		RCV_MAN_REQ_MASK
#define RCV_BOOT_COM_BAUDRATE	12500000
#define RCV_BOOT_COM_PARITY		2

#define RCV_MAN_REQ_WORD 		0xAB00
#define RCV_MAN_REQ_MASK 		0xFF00

#define RCV_FltResist(v)	(((v) * 941 + 2048) / 4096)
#define RCV_NetResist(v)	(((v) * 941 + 128) / 256)
#define RCV_NetAdr(v)		(1 + (v)/1024)

//#define RCV_TEST_WAVEPACK 16

#define RCV_WAVEPACK

#define RCV_SAMPLE_LEN 1024

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define TRANSMITER_NUM	3	// 0 - Monopole1, 1 - Monopole1, 2 - DipoleXY
#define RCV_FIRE_NUM	4	// 0 - Mnpl_1, 1 - Mnpl_2, 2 - DplX, 3 - DplY

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


#ifndef RCV_8AD

	#define REQRCV_ENUM enum { LEN = sizeof(Req)-sizeof(Req::len)-sizeof(Req::fill) }; enum { CRCLEN = sizeof(Req)-sizeof(Req::len)-sizeof(Req::fill)-sizeof(Req::crc) }

	__packed struct ReqRcv01	// ����� ���������
	{
		__packed struct Req
		{
			byte 	len;
			byte 	adr;
			byte 	func;
			byte 	n; 
			byte 	next_n; 
			byte	next_gain;
			u16		fc;		// fire count
			u16		sl;
			u16		st;
			u16		sd;
			u16		packType;
			u16		math;

			word 	crc;  

			u16		fill;
		}
		r[3];

		REQRCV_ENUM;
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
			u16		fill;
		}
		r[2];

		REQRCV_ENUM;
	};  

#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct RspRcvHdr02	// ������ �������
{
	u16 rw;
	u32 cnt;
	u16 preAmp;
	u16 gain;
	u16 st;
	u16 sl;
	u16 sd;
	u16 packType;
	u16 math;
	u16 packLen1;
	u16 packLen2;
	u16 packLen3;
	u16 packLen4;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct RspRcv02	// ������ �������
{
	RspRcvHdr02 hdr;

	u16 data[(RCV_SAMPLE_LEN+64)*4]; 
	u16 crc;
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifndef RCV_8AD

	__packed struct  ReqRcv03	// ��������� ������� ������������� ������� � ������������ ��������
	{ 
		__packed struct Req
		{
			byte 	len;
			byte 	adr;
			byte 	func;
			byte	numDevValid;		// ���� �� ����, numDev �������� � ��� ���� �������� � RAM
			u16		numDev;				// ����� ������ ���������
			u16 	gain[RCV_FIRE_NUM]; 
			word	crc; 
			u16		fill;
		}
		r[2];

		REQRCV_ENUM;
	};  

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	__packed struct  RspRcv03	// ��������� ������� ������������� ������� � ������������ ��������
	{ 
		byte	adr;
		byte	func;
		u16		temp;			// �����������
		u16		numdev;			// ����� ������ ���������
		u16		verdev; 		// ������ �� ������ ���������
		byte	numDevValid;	// ���� �� ����, numDev ������ �� flash ��������� ��� ���������� ��������
		byte	flashStatus; 	// ��� 0 - ������ � ��������, ��� 1 - ������ ��, ��� 2 - ������ ������
		word	crc; 
	};  

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	__packed struct  ReqRcv04	// ��������� ������������ ��������
	{ 
		__packed struct Req
		{
			byte 	len;
			byte 	adr;
			byte 	func;
			byte 	saveParams; // ���� �� ����, �� �������� ��������� �� flash
			word 	crc; 
			u16		fill;
		}
		r[2];

		REQRCV_ENUM;
	};  

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	__packed struct  RspRcv04	// ��������� ������� ������������� ������� � ������������ ��������
	{ 
		byte	adr;
		byte	func;
		byte	numDevValid;	// ���� �� ����, numDev ������ �� flash ��������� ��� ���������� ��������
		byte	flashStatus; 	// ��� 0 - ������ � ��������, ��� 1 - ������ ��, ��� 2 - ������ ������
		u16		temp;			// �����������
		u16		numdev;			// ����� ������ ���������
		u16		verdev; 		// ������ �� ������ ���������
		u16 	maxAmp[4];
		u16		power[4];
		word	crc; 
	};  

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	__packed struct ReqRcv05	// ������ ����������� ������� ��� � ����������� ��� �������� �� ������
	{
		__packed struct Req
		{
			byte 	len;
			byte 	adr;
			byte 	func;
			byte 	n; 
			word 	crc; 
			u16		fill;
		}
		r[2];

		REQRCV_ENUM;
	};  

	#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef RCV_8AD

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	__packed struct Req8AD_Rcv01	// ����� ���������
	{
		__packed struct Req
		{
			byte 	len;
			byte 	adr;
			byte 	func;
			byte 	n; 
			u16		vc;
			word 	crc;  
		}
		r[3];
	};

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	__packed struct Rsp8AD_Rcv01	// ����� ���������
	{
		byte adr;
		byte func;
		word crc;  
	};

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	__packed struct Req8AD_Rcv02	// ������ �������
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

	__packed struct Rsp8AD_RcvHdr02	// ������ �������
	{
		u16 rw; 
		u32 cnt; 
		u16 gain; 
		u16 st; 
		u16 len; 
		u16 delay; 
	};

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	__packed struct Rsp8AD_Rcv02	// ������ �������
	{
		Rsp8AD_RcvHdr02 hdr;

		u16 data[1024*4]; 
		u16 crc;
	};  

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	__packed struct  Req8AD_Rcv03	// ��������� ������� ������������� ������� � ������������ ��������
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

	__packed struct  Rsp8AD_Rcv03	// ��������� ������� ������������� ������� � ������������ ��������
	{ 
		byte adr;
		byte func;
		word crc; 
	};  

	//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	__packed struct  Req8AD_Rcv04	// ��������� ������������ ��������
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

	__packed struct  Rsp8AD_Rcv04	// ��������� ������� ������������� ������� � ������������ ��������
	{ 
		byte	adr;
		byte	func;
		u16 	maxAmp[4];
		u16		power[4];
		word	crc; 
	};  

#endif // #ifdef RCV_8AD

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct Transmiter
{
	u16 gain;
	u16 st;		//sampleTime;
	u16 sl;		//sampleLen;
	u16 sd;		//sampleDelay;
	u16 freq;
	u16 duty;
	u16 amp;
	u16 pulseCount;
	u16 packType;
	u16 math;

	u16 GetGain()				{ return gain&7; }
	void SetGain(u16 v)			{ gain = (gain & ~7)|(v&7); }
	u16 GetPreAmp()				{ return gain>>7; }
	void SetPreAmp(u16 v)		{ gain = (gain & ~(1<<7))|(v<<7); }

//#ifdef RCV_AUTO_GAIN
//	bool GetAutoGain()			{ return gain&8; }
//	void SetAutoGain(bool v)	{ gain = (v) ? (gain|8) : (gain & ~8); }
//#endif

};
	
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef RCV_AUTO_GAIN

struct AutoGain
{
	u16 gain;
	u16 maxAmp;

	u16 GetGain()			{ return gain&7; }
	void SetGain(u16 v)		{ gain = (gain & ~7)|(v&7); }
	u16 GetPreAmp()			{ return gain>>7; }
	void SetPreAmp(u16 v)	{ gain = (gain & ~(1<<7))|(v<<7); }

	void Inc()
	{ 
		u16 g = GetGain();

		if (GetPreAmp())
		{
			if (g != 7) SetGain(g+1);
		}
		else
		{
			if (g == 3) gain = 1<<7; else if (g != 7) SetGain(g+1);
		};
	}

	void Dec()
	{
		u16 g = GetGain();

		if (GetPreAmp())
		{
			if (g == 0) gain = 3; else SetGain(g-1);
		}
		else
		{
			if (g != 0) SetGain(g-1);
		};
	}
};

#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef _ADI_COMPILER
#pragma pack()
//#undef __packed
#endif



#endif //G_RCV_H__27_12_2023__11_22
