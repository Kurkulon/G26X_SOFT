#include "hardware.h"
//#include "options.h"
//#include "hw_emac.h"
#include "EMAC\xtrap.h"
#include "FLASH\NandFlash.h"
#include "CRC\CRC16.h"
#include "CRC\CRC16_CCIT.h"
#include "BOOT\boot_req.h"
#include "req.h"
#include "list.h"
#include "PointerCRC.h"
#include "SEGGER_RTT\SEGGER_RTT.h"
#include "hw_com.h"
#include "TaskList.h"
#include "FLASH\Nand_ECC.h"
#include "CRC\CRC_CCITT_DMA.h"

//#define RCV_TESTREQ02

#ifdef WIN32

#include <conio.h>
//#include <stdio.h>

//static const bool __WIN32__ = true;

#else

//static const bool __WIN32__ = false;

//#pragma diag_suppress 546,550,177

#endif

#if defined(WIN32) || defined(_DEBUG)
#define __TEST__
#endif

enum { VERSION = 0x10B };

//#pragma O0
//#pragma Otime

#ifndef _DEBUG
	static const bool __debug = false;
#else
	static const bool __debug = true;
#endif

//#define SENS_NUM	3
//#define NS2DSP(v) (((v)+10)/20)
//#define US2DSP(v) ((((v)*1000)+10)/20)

static Ptr<REQ> CreateRcvBootReq03(u16 adr, u16 tryCount);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//#ifdef CPU_SAME53
//
//	#define FPS_PIN_SET()	HW::PIOA->BSET(25)
//	#define FPS_PIN_CLR()	HW::PIOA->BCLR(25)
//
//#elif defined(CPU_XMC48)
//
//	#define FPS_PIN_SET()	HW::P2->BSET(13)
//	#define FPS_PIN_CLR()	HW::P2->BCLR(13)
//
//#elif defined(WIN32)
//
//	#define FPS_PIN_SET()	
//	#define FPS_PIN_CLR()	
//
//#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct MainVars // NonVolatileVars  
{
	u32 timeStamp;

	u16 numDevice;
	u16 numMemDevice;

	Transmiter	trans[TRANSMITER_NUM];

	u16 trmVoltage;
	u16 disableFireNoVibration;
	u16 levelNoVibration;
	u16 firePeriod;
	u16 lfMnplEnabled;

#ifdef RCV_AUTO_GAIN
	u16 autoGain;
#endif
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static MainVars mv;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool runMainMode = false;
static bool startFire = false;
static U32u  fireCounter;
static u32  manCounter = 0;

#ifndef RCV_8AD
static byte numStations = RCV_MAX_NUM_STATIONS;
#else
static byte numStations = 8;
#endif

static u16  resistValue = 0;
static u16  trmVoltage = 0;
static u16  trmVoltageLast = 0;
static u16  trmTemp = 0;
static u16  trmCount = 0;
static byte transIndex[RCV_FIRE_NUM] = {0, 1, 2, 2};

static u16  numDevTrm = 0;
static byte numDevTrmValid = 0;
static u16  verDevTrm = 0;
//static byte verDevTrmValid = 0;

static u16  numDevRcv = 0;
static byte numDevRcvValid = 0;
static u16  verDevRcv = 0;
//static byte verDevRcvValid = 0;

static u16  arrRcvNumDev[RCV_MAX_NUM_STATIONS] = {0};
static u16  arrRcvTemp[RCV_MAX_NUM_STATIONS] = {0};
static u16  arrRcvVerDev[RCV_MAX_NUM_STATIONS] = {0};
static byte arrRcvNumDevValid[RCV_MAX_NUM_STATIONS] = {0};
static byte arrRcvFlashStatus[RCV_MAX_NUM_STATIONS] = {0};

u32 req30_count1 = 0;
u32 req30_count2 = 0;
u32 req30_count3 = 0;

u32 fps;
i16 tempClock = 0;
i16 cpu_temp = 0;
u32 i2cResetCount = 0;

u16 manRcvData[10];
u16 manTrmData[50];
static u16 manTrmBaud = 0;
static u16 memTrmBaud = 0;

u16 txbuf[128 + 512 + 16];

static RequestQuery qTrm(&comTrm);
static RequestQuery qRcv(&comRcv);

static Ptr<MB> manVec30[RCV_FIRE_NUM];
static Ptr<MB> manVecMnpl[2];
static Ptr<MB> tmpVecMnpl[2];
static Ptr<MB> curManVec30;
static Ptr<MB> manVec60;
static Ptr<MB> curManVec60;
static TM32 tmVecMnpl[2];

//static ListPtr<MB> readyR01;

static RspMan71 rspMan71[RCV_FIRE_NUM];
static byte curRcv[RCV_FIRE_NUM] = { 0 };
static byte curRcvMnpl[2] = { 0 };
static byte indRcvMnpl = 0;
static TM32 curRcvTimer[RCV_FIRE_NUM];

#ifdef RCV_AUTO_GAIN
	static AutoGain autoGain[TRANSMITER_NUM] = { 0 };
	//static u16 bufMaxAmp[16] = { 0 };
	//static u16 bufMaxAmpIndex = 0;
#endif

static Ptr<MB> manVec72[2];
static Ptr<MB> curManVec72;
static byte indManVec72 = 0;

static u16 manReqWord = RCV_MAN_REQ_WORD;
static u16 manReqMask = RCV_MAN_REQ_MASK;

static u16 memReqWord = 0x3F00;
static u16 memReqMask = 0xFF00;

static u16 verDevice = VERSION;

static u16 verMemDevice = 0x301;

static byte mainModeState = 0;
static byte fireType = 0;
static u16	fireMask = 0;//(1UL<<RCV_FIRE_NUM)-1;
static byte nextFireType = 1;

static byte cmdWriteStart_00 = 0;
static byte cmdWriteStart_10 = 0;
static bool cmdWriteStart_20 = false;
static bool cmdWriteStart_73 = false;
static bool cmdRcvSaveParams = false;
static bool cmdTrmSaveParams = false;
static bool cmdWriteEnable = false;
static bool cmdWriteDisable = false;

static u16 rcvStatus = 0;
static u16 rcvErrors = 0;
static u16 rcvStatus02 = 0;
static u16 rcvStatus04 = 0;
static u16 rcvMisFire = 0;
static u16 rcvComStatus = 0;
//static u32 crcErr02[RCV_MAX_NUM_STATIONS] = {0};
//static u32 crcErr03 = 0;
//static u32 crcErr04 = 0;
static u32 rcv02rejVec = 0;
static u32 retryRcv02 = 0;
static u32 crcErr02 = 0;
static u32 notRcv02 = 0;
static u16 okRcv02 = 0;
static u16 crcErrLen02 = 0;
static u16 crcErrRW02 = 0;

static u32 rcv05rejVec = 0;
static u32 retryRcv05 = 0;
static u32 crcErr05 = 0;
static u32 notRcv05 = 0;
static u16 okRcv05 = 0;
static u16 crcErrLen05 = 0;
static u16 crcErrRW05 = 0;

//static u32 notRcv02[RCV_MAX_NUM_STATIONS] = {0};
//static u32 lenErr02[RCV_MAX_NUM_STATIONS] = {0};
//static u32 rejRcv02[RCV_MAX_NUM_STATIONS] = {0};
//static u32 retryRcv02[RCV_MAX_NUM_STATIONS] = {0};

//static u32 crcErr06 = 0;
//static u32 wrtErr06 = 0;

static i16 ax = 0, ay = 0, az = 0, at = 0;
static u16 vibration;

i16 temperature = 0;
i16 cpuTemp = 0;
i16 temp = 0;

static byte svCount = 0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static byte GetNextFireType(byte n)
{
	n = (n+1)%RCV_FIRE_NUM; 

	if (fireMask == 0) return n;

	for (byte i = 0; i < RCV_FIRE_NUM; i++)
	{
		if (fireMask & (1UL<<n)) break;
		n = (n+1)%RCV_FIRE_NUM; 
	};

	return n;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SaveMainParams()
{
	svCount = 1;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SetNumDevice(u16 num)
{
	mv.numDevice = num;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u16 GetNumDevice()
{
	return mv.numDevice;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u16 GetVersionDevice()
{
	return verDevice;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static void Response_0(u16 rw, MTB &mtb)
//{
//	__packed struct Rsp {u16 rw; u16 device; u16 session; u32 rcvVec; u32 rejVec; u32 wrVec; u32 errVec; u16 wrAdr[3]; u16 numDevice; u16 version; u16 temp; byte status; byte flags; RTC rtc; };
//
//	Rsp &rsp = *((Rsp*)&txbuf);
//
//	rsp.rw = rw;
//	rsp.device = NandFlash_GetDeviceID();  
//	rsp.session = NandFlash_Session_Get();	  
//	rsp.rcvVec =  NandFlash_Vectors_Recieved_Get();
//	rsp.rejVec = NandFlash_Vectors_Rejected_Get();
//	rsp.wrVec = NandFlash_Vectors_Saved_Get();
//	rsp.errVec = NandFlash_Vectors_Errors_Get();
//	*((__packed u64*)rsp.wrAdr) = NandFlash_Current_Adress_Get();
//	rsp.temp = temp*5/2;
//	rsp.status = NandFlash_Status();
//
//	GetTime(&rsp.rtc);
//
//	mtb.data1 = txbuf;
//	mtb.len1 = sizeof(rsp)/2;
//	mtb.data2 = 0;
//	mtb.len2 = 0;
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateRcvReqFire(byte n, byte next_n, u16 fc)
{
	Ptr<REQ> rq(AllocREQ());
	
	//rq.Alloc();

	if (!rq.Valid()) return rq;

#ifndef RCV_8AD
	ReqRcv01		&req = *((ReqRcv01*)rq->reqData);
#else
	Req8AD_Rcv01	&req = *((Req8AD_Rcv01*)rq->reqData);
#endif

	REQ &q = *rq;

	q.CallBack = 0;
	q.tryCount = 0;
	q.checkCRC = false;
	q.updateCRC = false;
	
	q.wb.data = &req;
	q.wb.len = sizeof(req);

	q.rb.data = 0;
	q.rb.maxLen = 0;

#ifndef RCV_8AD

	Transmiter	&trans = mv.trans[transIndex[n]];

	req.r[2].len		= req.r[1].len			= req.r[0].len			= req.LEN; //sizeof(req.r[0]) - req.OVERLEN;
	req.r[2].adr		= req.r[1].adr			= req.r[0].adr			= 0;
	req.r[2].func		= req.r[1].func			= req.r[0].func			= 1;
	req.r[2].n			= req.r[1].n			= req.r[0].n			= n;
	req.r[2].next_n		= req.r[1].next_n		= req.r[0].next_n		= next_n;

#ifdef RCV_AUTO_GAIN

	if (mv.autoGain)
	{
		req.r[2].next_gain	= req.r[1].next_gain = req.r[0].next_gain = autoGain[transIndex[next_n]].gain;
	}
	else

#endif	

	req.r[2].next_gain	= req.r[1].next_gain	= req.r[0].next_gain	= mv.trans[transIndex[next_n]].gain;

	req.r[2].st 		= req.r[1].st 			= req.r[0].st 			= trans.st;
	req.r[2].sl 		= req.r[1].sl 			= req.r[0].sl 			= trans.sl;
	req.r[2].sd 		= req.r[1].sd 			= req.r[0].sd 			= trans.sd;
	req.r[2].fc			= req.r[1].fc			= req.r[0].fc			= fc;
	req.r[2].packType	= req.r[1].packType		= req.r[0].packType		= trans.packType;
	req.r[2].math		= req.r[1].math			= req.r[0].math			= trans.math;

	req.r[2].crc		= req.r[1].crc			= req.r[0].crc			= GetCRC16(&req.r[0].adr, req.CRCLEN);

	req.r[2].fill		= req.r[1].fill			= req.r[0].fill			= 0xFFFF;

#else

	req.r[2].len		= req.r[1].len			= req.r[0].len			= sizeof(req.r[0]) - 1;
	req.r[2].adr		= req.r[1].adr			= req.r[0].adr			= 0;
	req.r[2].func		= req.r[1].func			= req.r[0].func			= 1;
	req.r[2].n			= req.r[1].n			= req.r[0].n			= (n > 0) ? (n-1) : 0;
	req.r[2].vc			= req.r[1].vc			= req.r[0].vc			= fc;

	req.r[2].crc		= req.r[1].crc			= req.r[0].crc			= GetCRC16(&req.r[0].adr, sizeof(req.r[0])-3);

#endif

	return &q;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifndef RCV_8AD

static bool CallBackRcvReq02(Ptr<REQ> &q)
{
	ReqRcv02 &req = *((ReqRcv02*)q->wb.data);
	RspRcv02 &rsp = *((RspRcv02*)q->rb.data);
	 
	//bool crcOK = q->crcOK;

	q->rsp->len = 0;

	//byte a = (req.r[0].adr-1) & 15;

	u16 mask = 1 << ((req.r[0].adr-1) & 15);

	if (q->crcOK)
	{
		rcvStatus02 |= 1 << (rsp.hdr.rw & 15);

		u16 len = sizeof(rsp.hdr) + sizeof(rsp.crc) + ((rsp.hdr.packLen1+rsp.hdr.packLen2+rsp.hdr.packLen3+rsp.hdr.packLen4) * 2);

		if (q->rb.len < sizeof(rsp.hdr) || (rsp.hdr.rw & manReqMask) != manReqWord || q->rb.len != len)
		{
			//SEGGER_RTT_printf(0, RTT_CTRL_TEXT_BRIGHT_YELLOW "\nRcvReq02 RW:0x%04X rb.len = %-6u rsp.hdr Error ", rsp.hdr.rw, q->rb.len);

			q->crcOK = false;
			q->rsp->len = 0;
			//lenErr02[a]++;
		}
		else
		{
			okRcv02 += 1;

			rcvStatus |= 1 << (rsp.hdr.rw & 15);
			rcvComStatus |= 1 << (rsp.hdr.rw & 15);
			
			q->rsp->len = q->rb.len;
		};
	}
	else 
	{
		if (q->rb.recieved)
		{
			//SEGGER_RTT_printf(0, RTT_CTRL_TEXT_BRIGHT_YELLOW "\nRcvReq02 RW:0x%04X rb.len = %-6u CRC Error ", rsp.hdr.rw, q->rb.len);

			crcErrLen02 = q->rb.len;
			crcErrRW02 = rsp.hdr.rw;
			crcErr02++;
		}
		else
		{
			//SEGGER_RTT_printf(0, RTT_CTRL_TEXT_WHITE "\nRcvReq02 Adr:%u Not Recieved ", a+1);

			notRcv02++;
		};
	};

	if (!q->crcOK)
	{
		//if (q->rb.recieved)
		//{
		//	crcErr02[a]++;
		//}
		//else
		//{
		//	notRcv02[a]++;
		//};

		if (q->tryCount > 0)
		{
			q->tryCount--;
			qRcv.Add(q);

			retryRcv02 += 1;

			//retryRcv02[a] += 1;
		}
		else
		{
			//SEGGER_RTT_printf(0, RTT_CTRL_TEXT_BRIGHT_RED "\nRcvReq02 RW:0x%04X rb.len = %-6u Rejected ", rsp.hdr.rw, q->rb.len);

			rcvStatus &= ~mask; 
			rcvErrors |= mask;

			rcvComStatus = (rcvComStatus & ~mask) | (rcvStatus02 & mask);

			if (rcvStatus02 & mask) rcvMisFire += 1;

			rcv02rejVec += 1;
			//rejRcv02[a] += 1;
		};
	};

	return true;
}

#else //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool CallBackRcvReq02(Ptr<REQ> &q)
{
	Req8AD_Rcv02 &req8 = *((Req8AD_Rcv02*)q->wb.data);
	Rsp8AD_Rcv02 &rsp8 = *((Rsp8AD_Rcv02*)q->rb.data);
	 
	q->rsp->len = 0;

	byte a = (req8.r[0].adr-1) & 7;

	if (q->crcOK)
	{
		u16 len = sizeof(rsp8.hdr) + sizeof(rsp8.crc) + rsp8.hdr.len * 8;

		if (q->rb.len < sizeof(rsp8.hdr) || (rsp8.hdr.rw & manReqMask) != manReqWord || q->rb.len != len)
		{
			q->crcOK = false;
			q->rsp->len = 0;
			//lenErr02[a]++;
		}
		else
		{
			RspRcv02 &rsp = *((RspRcv02*)(q->rsp->GetDataPtr()));

			rsp.hdr.rw			= ((rsp8.hdr.rw & 0xF0) == 0x30) ? rsp8.hdr.rw : (rsp8.hdr.rw+0x10);
			rsp.hdr.cnt			= rsp8.hdr.cnt;
			rsp.hdr.gain		= 1UL << rsp8.hdr.gain;
			rsp.hdr.st 			= rsp8.hdr.st;
			rsp.hdr.sl 			= rsp8.hdr.len;
			rsp.hdr.sd 			= rsp8.hdr.delay;
			rsp.hdr.packType	= 0;
			rsp.hdr.math		= 0;
			rsp.hdr.packLen1 	= rsp.hdr.sl;
			rsp.hdr.packLen2 	= rsp.hdr.sl;
			rsp.hdr.packLen3 	= rsp.hdr.sl;
			rsp.hdr.packLen4 	= rsp.hdr.sl;

			okRcv02 += 1;

			rcvStatus |= 1 << (rsp.hdr.rw & 7);
			
			q->rsp->len = q->rb.len + sizeof(rsp.hdr) - sizeof(rsp8.hdr);
		};
	}
	else if (q->rb.recieved)
	{
		crcErrLen02 = q->rb.len;
		crcErrRW02 = rsp8.hdr.rw;
		crcErr02++;
	}
	else
	{
		notRcv02++;
	};

	if (!q->crcOK)
	{
		//if (q->rb.recieved)
		//{
		//	crcErr02[a]++;
		//}
		//else
		//{
		//	notRcv02[a]++;
		//};

		if (q->tryCount > 0)
		{
			q->tryCount--;
			qRcv.Add(q);

			retryRcv02 += 1;

			//retryRcv02[a] += 1;
		}
		else
		{
			rcvStatus &= ~(1 << (a)); 
			rcvErrors |= (1 << (a));

			rcv02rejVec += 1;
			//rejRcv02[a] += 1;
		};
	};

	return true;
}

#endif
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateRcvReq02(byte adr, byte n, u16 tryCount)
{
	Ptr<REQ> rq(AllocREQ());
	
	if (!rq.Valid()) return rq;

	if (adr != 0)
	{
		rq->rsp = NandFlash_AllocWB(sizeof(RspRcv02)+2);
		if (!rq->rsp.Valid()) { rq.Free(); return rq; };
		
		RspRcv02 &rsp = *((RspRcv02*)(rq->rsp->GetDataPtr()));

		#ifndef RCV_8AD

			rq->rb.data = &rsp;
			rq->rb.maxLen = rq->rsp->GetDataMaxLen();

		#else

			Rsp8AD_Rcv02 &rsp8 = *((Rsp8AD_Rcv02*)(((byte*)&rsp) + sizeof(rsp.hdr) - sizeof(Rsp8AD_Rcv02::hdr)));

			rq->rb.data = &rsp8;
			rq->rb.maxLen = rq->rsp->GetDataMaxLen() + sizeof(rsp.hdr) - sizeof(Rsp8AD_Rcv02::hdr);

		#endif

		rcvStatus02 &= ~(1<<(adr-1));
	}
	else
	{
		rq->rb.data = 0;
		rq->rb.maxLen = 0;
	};

	rq->rsp->len = 0;
	REQ &q = *rq;

#ifndef RCV_8AD

	ReqRcv02 &req = *((ReqRcv02*)rq->reqData);
	adr = (adr-1)&15; 

	q.crcType = REQ::CRC16_CCIT;

#else

	Req8AD_Rcv02 &req = *((Req8AD_Rcv02*)rq->reqData);

	adr = (adr-1)&7; 

	q.crcType = REQ::CRC16;

#endif

	q.CallBack = CallBackRcvReq02;
	q.preTimeOut = MS2COM(1);
	q.postTimeOut = US2COM(100);
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;
	
	q.wb.data = &req;
	q.wb.len = sizeof(req);

	req.r[1].len	= req.r[0].len	= req.LEN;
	req.r[1].adr	= req.r[0].adr	= adr+1;
	req.r[1].func	= req.r[0].func	= 2;

#ifndef RCV_8AD

	req.r[1].n		= req.r[0].n	= n;

#else

	req.r[1].n		= req.r[0].n	= (n > 0) ? (n-1) : 0;
	req.r[1].chnl	= req.r[0].chnl	= 0;

#endif

	req.r[1].crc	= req.r[0].crc	= GetCRC16(&req.r[0].adr, req.CRCLEN);

	req.r[1].fill	= req.r[0].fill	= 0xFFFF;

#ifdef RCV_TESTREQ02

	static u32 seed = 0;

	seed = (seed + 101) * 1001 + GetCYCCNT();

	byte *p = (byte*)&req;

	byte i = seed % (sizeof(req)*8);

	p[i/8] ^= (1<<(i&7));

#endif

	return rq;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool CallBackRcvReq03(Ptr<REQ> &q)
{
	if (!q->crcOK) 
	{
		//crcErr03++;

		if (q->tryCount > 0)
		{
			q->tryCount--;
			qRcv.Add(q);
		};
	}
	else
	{
	#ifndef RCV_8AD

		RspRcv03 &rsp = *((RspRcv03*)q->rb.data);

		byte n = rsp.adr - 1;

		arrRcvTemp[n]			= rsp.temp;
		arrRcvNumDev[n]			= rsp.numdev;
		arrRcvVerDev[n]			= rsp.verdev;
		arrRcvNumDevValid[n]	= rsp.numDevValid;
		arrRcvFlashStatus[n]	= rsp.flashStatus;

	#else


	#endif

	};

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateRcvReq03(byte adr, u16 tryCount)
{
	Ptr<REQ> rq(AllocREQ());
	
	if (!rq.Valid()) return rq;

	#ifndef RCV_8AD
		typedef RspRcv03 RspT;
		typedef ReqRcv03 ReqT;

	#else
		typedef Rsp8AD_Rcv03 RspT;
		typedef Req8AD_Rcv03 ReqT;
	#endif

	if (adr != 0)
	{
		rq->rsp = AllocMemBuffer(sizeof(RspT)+2);

		if (!rq->rsp.Valid()) { rq.Free(); return rq; };

		RspT &rsp = *((RspT*)(rq->rsp->GetDataPtr()));

		rq->rb.data = &rsp;
		rq->rb.maxLen = rq->rsp->GetDataMaxLen();
	}
	else
	{
		rq->rb.data	= 0;
		rq->rb.maxLen	= 0;
	};

	ReqT &req = *((ReqT*)rq->reqData);

	REQ &q = *rq;

	q.CallBack = CallBackRcvReq03;
	q.preTimeOut = US2COM(500);
	q.postTimeOut = US2COM(100);
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;
	q.crcType = REQ::CRC16;
	
	q.wb.data = &req;
	q.wb.len = sizeof(req);

	req.r[1].len			= req.r[0].len			= req.LEN;
	req.r[1].adr			= req.r[0].adr			= adr;
	req.r[1].func			= req.r[0].func			= 3;

#ifndef RCV_8AD

	req.r[1].numDevValid	= req.r[0].numDevValid	= numDevRcvValid;
	req.r[1].numDev			= req.r[0].numDev		= numDevRcv;

	for (u16 i = 0; i < RCV_FIRE_NUM; i++)
	{
		byte n = transIndex[i];

		#ifdef RCV_AUTO_GAIN

			if (mv.autoGain)
			{
				req.r[1].gain[i] = req.r[0].gain[i] = autoGain[n].gain;
			}
			else

		#endif	
				
				req.r[1].gain[i] = req.r[0].gain[i] = mv.trans[n].gain;
	};

#else

	req.r[1].st[0] = req.r[0].st[0] = mv.trans[0].st;
	req.r[1].st[1] = req.r[0].st[1] = mv.trans[2].st;
	req.r[1].st[2] = req.r[0].st[2] = mv.trans[2].st;

	req.r[1].sl[0] = req.r[0].sl[0] = mv.trans[0].sl;
	req.r[1].sl[1] = req.r[0].sl[1] = mv.trans[2].sl;
	req.r[1].sl[2] = req.r[0].sl[2] = mv.trans[2].sl;

	req.r[1].sd[0] = req.r[0].sd[0] = 0;
	req.r[1].sd[1] = req.r[0].sd[1] = 0;
	req.r[1].sd[2] = req.r[0].sd[2] = 0;

#endif

	req.r[1].crc	= req.r[0].crc	= GetCRC16(&req.r[0].adr, req.CRCLEN);

	req.r[1].fill	= req.r[0].fill	= 0xFFFF;

	return &q;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool CallBackRcvReq04(Ptr<REQ> &q)
{
	if (!q->crcOK) 
	{
		if(!q->rb.recieved) qRcv.Add(CreateRcvBootReq03(0, 0));

		//crcErr04++;

		if (q->tryCount > 0)
		{
			q->tryCount--;
			qRcv.Add(q);
		}
		else
		{
			RspRcv04 &rsp = *((RspRcv04*)q->rb.data);

			rcvStatus04 &= ~(1<<(rsp.adr-1));
		};
	}
	else
	{
		#ifndef RCV_8AD

			RspRcv04 &rsp = *((RspRcv04*)q->rb.data);

			u16 i = rsp.adr - 1;

			rcvStatus04 |= 1<<i;

			arrRcvTemp[i]			= rsp.temp;
			arrRcvNumDev[i]			= rsp.numdev;
			arrRcvVerDev[i]			= rsp.verdev;
			arrRcvNumDevValid[i]	= rsp.numDevValid;
			arrRcvFlashStatus[i]	= rsp.flashStatus;

			if (fireType < RCV_FIRE_NUM && i < RCV_MAX_NUM_STATIONS)
			{
				i = i*4;

				rspMan71[fireType].maxAmp[i + 0] = rsp.maxAmp[0];
				rspMan71[fireType].maxAmp[i + 1] = rsp.maxAmp[1];
				rspMan71[fireType].maxAmp[i + 2] = rsp.maxAmp[2];
				rspMan71[fireType].maxAmp[i + 3] = rsp.maxAmp[3];

				rspMan71[fireType].power[i + 0] = rsp.power[0];
				rspMan71[fireType].power[i + 1] = rsp.power[1];
				rspMan71[fireType].power[i + 2] = rsp.power[2];
				rspMan71[fireType].power[i + 3] = rsp.power[3];

				#ifdef RCV_AUTO_GAIN

				if (mv.autoGain)
				{
					byte n = transIndex[fireType];

					if (rsp.maxAmp[0] > autoGain[n].maxAmp) autoGain[n].maxAmp = rsp.maxAmp[0];
					if (rsp.maxAmp[1] > autoGain[n].maxAmp) autoGain[n].maxAmp = rsp.maxAmp[1];
					if (rsp.maxAmp[2] > autoGain[n].maxAmp) autoGain[n].maxAmp = rsp.maxAmp[2];
					if (rsp.maxAmp[3] > autoGain[n].maxAmp) autoGain[n].maxAmp = rsp.maxAmp[3];
				};

				#endif		
			};

		#else

			Rsp8AD_Rcv04 &rsp = *((Rsp8AD_Rcv04*)q->rb.data);

			u16 i = rsp.adr - 1;

			if (fireType < RCV_FIRE_NUM && i < RCV_MAX_NUM_STATIONS)
			{
				i = i*4;

				rspMan71[fireType].maxAmp[i + 0] = rsp.maxAmp[0];
				rspMan71[fireType].maxAmp[i + 1] = rsp.maxAmp[1];
				rspMan71[fireType].maxAmp[i + 2] = rsp.maxAmp[2];
				rspMan71[fireType].maxAmp[i + 3] = rsp.maxAmp[3];

				rspMan71[fireType].power[i + 0] = rsp.power[0];
				rspMan71[fireType].power[i + 1] = rsp.power[1];
				rspMan71[fireType].power[i + 2] = rsp.power[2];
				rspMan71[fireType].power[i + 3] = rsp.power[3];
			};

		#endif
	};

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateRcvReq04(byte adr, byte saveParams, u16 tryCount)
{
	Ptr<REQ> rq(AllocREQ());
	
	if (!rq.Valid()) return rq;

	#ifndef RCV_8AD
		typedef RspRcv04 RspT;
		typedef ReqRcv04 ReqT;

	#else
		typedef Rsp8AD_Rcv04 RspT;
		typedef Req8AD_Rcv04 ReqT;
	#endif

	rq->rsp = NandFlash_AllocWB(sizeof(RspT)+2);

	if (!rq->rsp.Valid()) { rq.Free(); return rq; };

	ReqT &req = *((ReqT*)rq->reqData);
	RspT &rsp = *((RspT*)(rq->rsp->GetDataPtr()));

	REQ &q = *rq;

	q.CallBack = CallBackRcvReq04;
	q.preTimeOut = US2COM(500);
	q.postTimeOut = US2COM(100);
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;
	q.crcType = REQ::CRC16;
	
	q.wb.data = &req;
	q.wb.len = sizeof(req);

	q.rb.data = (adr != 0) ? &rsp : 0;
	q.rb.maxLen = rq->rsp->GetDataMaxLen();

	req.r[1].len		= req.r[0].len			= req.LEN;
	req.r[1].adr		= req.r[0].adr			= adr;
	req.r[1].func		= req.r[0].func			= 4;

#ifndef RCV_8AD

	req.r[1].saveParams	= req.r[0].saveParams	= saveParams;

#else

	req.r[1].ka[0] = req.r[0].ka[0] = mv.trans[0].sd;	//(mv.trans[0].gain) ? 4 : 1;
	req.r[1].ka[1] = req.r[0].ka[1] = mv.trans[2].sd;	//(mv.trans[2].gain) ? 4 : 1;
	req.r[1].ka[2] = req.r[0].ka[2] = mv.trans[2].sd;	//(mv.trans[2].gain) ? 4 : 1;

#endif
						 
	req.r[1].crc	= req.r[0].crc	= GetCRC16(&req.r[0].adr, req.CRCLEN);

	req.r[1].fill	= req.r[0].fill	= 0xFFFF;

	return &q;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool CallBackRcvReq05(Ptr<REQ> &q)
{
	ReqRcv05 &req = *((ReqRcv05*)q->wb.data);
	RspRcv02 &rsp = *((RspRcv02*)q->rb.data);

	//bool crcOK = q->crcOK;

	q->rsp->len = 0;

	byte a = (req.r[0].adr-1) & 15;

	if (q->crcOK)
	{
		u16 len = sizeof(rsp.hdr) + sizeof(rsp.crc) + ((rsp.hdr.packLen1+rsp.hdr.packLen2+rsp.hdr.packLen3+rsp.hdr.packLen4) * 2);

		if (q->rb.len < sizeof(rsp.hdr) || (rsp.hdr.rw & manReqMask) != manReqWord || q->rb.len != len)
		{
			q->crcOK = false;
			q->rsp->len = 0;
		}
		else
		{
			okRcv05 += 1;

			q->rsp->len = q->rb.len;

			u16 n = ((rsp.hdr.rw >> 4) & 0xF) - 3;
			u16 r = rsp.hdr.rw & 0xF;

			if ((rsp.hdr.rw & manReqMask) == manReqWord && n < RCV_FIRE_NUM && r < RCV_MAX_NUM_STATIONS)
			{
				bool c = false;

				Ptr<MB> *manVec = 0;

				if (n == 0) // Monopole HF
				{
					for (byte i = 0; i < ArraySize(curRcvMnpl); i++)
					{
						if (curRcvMnpl[i] == (r+1))
						{
							manVec = &(tmpVecMnpl[i]);
							c = true;
							break;
						};
					};
				}
				else if (curRcv[n] == (r+1))
				{
					c = true;

					manVec = &(manVec30[n]);
				};

				if (c) 
				{
					*manVec = q->rsp;
				};
			};

		};
	}
	else if (q->rb.recieved)
	{
		crcErrLen05 = q->rb.len;
		crcErrRW05 = rsp.hdr.rw;
		crcErr05++;
	}
	else
	{
		notRcv05++;
	};

	if (!q->crcOK)
	{
		if (q->tryCount > 0)
		{
			q->tryCount--;
			qRcv.Add(q);

			retryRcv05 += 1;
		}
		else
		{
			rcv05rejVec += 1;
		};
	};

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateRcvReq05(byte adr, byte n, u16 tryCount)
{
	Ptr<REQ> rq(AllocREQ());

	if (!rq.Valid()) return rq;

	if (adr != 0)
	{
		rq->rsp = AllocMemBuffer(sizeof(RspRcv02)+2);
		if (!rq->rsp.Valid()) { rq.Free(); return rq; };

		RspRcv02 &rsp = *((RspRcv02*)(rq->rsp->GetDataPtr()));

		rq->rb.data = &rsp;
		rq->rb.maxLen = rq->rsp->GetDataMaxLen();

	}
	else
	{
		rq->rb.data = 0;
		rq->rb.maxLen = 0;
	};

	rq->rsp->len = 0;
	REQ &q = *rq;

	ReqRcv05 &req = *((ReqRcv05*)rq->reqData);
	adr = (adr-1)&15; 

	q.crcType = REQ::CRC16_CCIT;

	q.CallBack = CallBackRcvReq05;
	q.preTimeOut = MS2COM(1);
	q.postTimeOut = US2COM(100);
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;

	q.wb.data = &req;
	q.wb.len = sizeof(req);

	req.r[1].len	= req.r[0].len	= req.LEN;
	req.r[1].adr	= req.r[0].adr	= adr+1;
	req.r[1].func	= req.r[0].func	= 5;
	req.r[1].n		= req.r[0].n	= n;
	req.r[1].crc	= req.r[0].crc	= GetCRC16(&req.r[0].adr, req.CRCLEN);
	req.r[1].fill	= req.r[0].fill	= 0xFFFF;

	return rq;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateTrmReqFire(byte n)
{
	Ptr<REQ> rq(AllocREQ());
	
	if (!rq.Valid()) return rq;

	ReqTrm01 &req = *((ReqTrm01*)rq->reqData);

	REQ &q = *rq;

	q.CallBack = 0;
	q.checkCRC = false;
	q.updateCRC = false;
	
	q.wb.data = &req;
	q.wb.len = sizeof(req);

	q.rb.data = 0;
	q.rb.maxLen = 0;

	byte t = transIndex[n];

	req.r[2].len		= req.r[1].len			= req.r[0].len			= sizeof(req.r[0]) - 1;
	req.r[2].func		= req.r[1].func			= req.r[0].func			= 1;
	req.r[2].n			= req.r[1].n			= req.r[0].n			= n;
	req.r[2].fireCount	= req.r[1].fireCount	= req.r[0].fireCount	= mv.trans[t].pulseCount;
	req.r[2].fireFreq	= req.r[1].fireFreq		= req.r[0].fireFreq		= mv.trans[t].freq;
	req.r[2].fireDuty	= req.r[1].fireDuty		= req.r[0].fireDuty		= mv.trans[t].duty;
	req.r[2].fireAmp	= req.r[1].fireAmp		= req.r[0].fireAmp		= mv.trans[t].amp;
	req.r[2].crc		= req.r[1].crc			= req.r[0].crc			= GetCRC16(&req.r[0].func, sizeof(req.r[0])-3);

	return &q;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool CallBackTrmReq02(Ptr<REQ> &q)
{
	if (!q->crcOK) 
	{
		if (q->tryCount > 0)
		{
			q->tryCount--;
			qTrm.Add(q);
		};
	}
	else
	{
		RspTrm02 &rsp = *((RspTrm02*)q->rb.data);

		numDevTrmValid			= rsp.numDevValid;
		numDevTrm				= rsp.numdev;
		verDevTrm				= rsp.verdev;
		trmVoltage				= rsp.hv;
		trmTemp					= rsp.temp;

		cmdTrmSaveParams		= false;

		trmCount++;
	};

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateTrmReq02(u16 tryCount)
{
	typedef ReqTrm02 ReqT;
	typedef RspTrm02 RspT;

	Ptr<REQ> rq(AllocREQ());
	
	if (!rq.Valid()) return rq;

	rq->rsp = AllocMemBuffer(sizeof(RspT)+2);

	if (!rq->rsp.Valid()) { rq.Free(); return rq; };
	
	RspT &rsp = *((RspT*)(rq->rsp->GetDataPtr()));

	rq->rb.data = &rsp;
	rq->rb.maxLen = rq->rsp->GetDataMaxLen();

	ReqT &req = *((ReqT*)rq->reqData);

	rq->rsp->len = 0;
	
	REQ &q = *rq;

	q.CallBack = CallBackTrmReq02;
	q.preTimeOut = MS2COM(1);
	q.postTimeOut = US2COM(100);
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;
	q.crcType = REQ::CRC16;
	
	q.wb.data = &req;
	q.wb.len = sizeof(req);

	req.r[1].len			= req.r[0].len			= sizeof(req.r[0]) - 1;
	req.r[1].func			= req.r[0].func			= 2;
	req.r[1].numDevValid	= req.r[0].numDevValid	= numDevTrmValid;
	req.r[1].saveParams		= req.r[0].saveParams	= cmdTrmSaveParams;
	req.r[1].reqHV			= req.r[0].reqHV		= mv.trmVoltage;
	req.r[1].numDev			= req.r[0].numDev		= numDevTrm;
	req.r[1].crc			= req.r[0].crc			= GetCRC16(&req.r[0].func, sizeof(req.r[0])-3);

	return rq;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool CallBackTrmReq03(Ptr<REQ> &q)
{
	if (q->crcOK) 
	{
		RspTrm03 &rsp = *((RspTrm03*)q->rb.data);

		if (q->rb.len == (sizeof(rsp) - sizeof(rsp.data) + rsp.sl*2))
		{
			q->rsp->len = q->rb.len-2;

			rsp.rw = manReqWord|0x72;

			if (rsp.num < 2) manVec72[rsp.num] = q->rsp;

			NandFlash_RequestWrite(q->rsp, rsp.rw, true);
	
			trmCount++;
		};
	};

	if (!q->crcOK) 
	{
		if (q->tryCount > 0)
		{
			q->tryCount--;
			qTrm.Add(q);
		};
	}

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateTrmReq03(u16 tryCount)
{
	typedef ReqTrm03 ReqT;
	typedef RspTrm03 RspT;

	Ptr<REQ> rq(AllocREQ());
	
	if (!rq.Valid()) return rq;

	rq->rsp = NandFlash_AllocWB(sizeof(RspTrm03)+2);
	if (!rq->rsp.Valid()) { rq.Free(); return rq; };
	
	RspT &rsp = *((RspT*)(rq->rsp->GetDataPtr()));

	rq->rb.data = &rsp;
	rq->rb.maxLen = rq->rsp->GetDataMaxLen();

	ReqT &req = *((ReqT*)rq->reqData);

	rq->rsp->len = 0;
	
	REQ &q = *rq;

	q.CallBack = CallBackTrmReq03;
	q.preTimeOut = MS2COM(1);
	q.postTimeOut = US2COM(100);
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;
	q.crcType = REQ::CRC16;
	
	q.wb.data = &req;
	q.wb.len = sizeof(req);

	req.r[1].len	= req.r[0].len	= sizeof(req.r[0]) - 1;
	req.r[1].func	= req.r[0].func	= 3;
	req.r[1].crc	= req.r[0].crc	= GetCRC16(&req.r[0].func, sizeof(req.r[0])-3);

	return rq;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool CallBackRcvBootReq00(Ptr<REQ> &q)
{
	if (!q->crcOK) 
	{
		if (q->tryCount > 0)
		{
			q->tryCount--;
			qRcv.Add(q);

			return false;
		};
	};

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateRcvBootReq00(u16 adr, BootRspV1::SF0 *rspdata, u16 tryCount)
{
	Ptr<REQ> rq(AllocREQ());
	
	//rq.Alloc();//= REQ::Alloc();

	if (!rq.Valid()) return rq;

	if (rspdata == 0 && adr != 0)
	{
		rq->rsp = AllocMemBuffer(sizeof(*rspdata));
		if (!rq->rsp.Valid()) { rq.Free(); return rq; };
		rspdata = (BootRspV1::SF0*)(rq->rsp->GetDataPtr());
	};

	BootReqV1::SF0 &req = *((BootReqV1::SF0*)rq->reqData);
	BootRspV1::SF0 &rsp = *rspdata;
	
	REQ &q = *rq;

	q.CallBack = CallBackRcvBootReq00;
	q.preTimeOut = MS2COM(10);
	q.postTimeOut = US2COM(100);
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;
	
	q.wb.data = &req;
	q.wb.len = sizeof(req);
	
	q.rb.data = (adr == 0) ? 0 : &rsp;
	q.rb.maxLen = sizeof(rsp);

	req.adr	= adr;
	req.rw	= RCV_BOOT_REQ_WORD|0;
	req.crc	= GetCRC16(&req, sizeof(req)-sizeof(req.crc));

	return rq;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool CallBackRcvBootReq01(Ptr<REQ> &q)
{
	if (!q->crcOK) 
	{
		if (q->tryCount > 0)
		{
			q->tryCount--;
			qRcv.Add(q);

			return false;
		};
	};

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateRcvBootReq01(u16 adr, u32 len, u16 tryCount)
{
	Ptr<REQ> rq(AllocREQ());
	
	//rq.Alloc();//= REQ::Alloc();

	if (!rq.Valid()) return rq;

	if (adr != 0)
	{
		rq->rsp = AllocMemBuffer(sizeof(BootRspV1::SF1));
		if (!rq->rsp.Valid()) { rq.Free(); return rq; };
	};

	BootReqV1::SF1 &req = *((BootReqV1::SF1*)rq->reqData);
	BootRspV1::SF1 &rsp = *((BootRspV1::SF1*)(rq->rsp->GetDataPtr()));
	
	REQ &q = *rq;

	q.CallBack = CallBackRcvBootReq01;
	q.preTimeOut = MS2COM(10);
	q.postTimeOut = US2COM(100);
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;
	
	q.wb.data = &req;
	q.wb.len = sizeof(req);
	
	q.rb.data = (adr == 0) ? 0 : &rsp;
	q.rb.maxLen = sizeof(rsp);

	req.adr	= adr;
	req.rw	= RCV_BOOT_REQ_WORD|1;
	req.len = len;
	req.crc	= GetCRC16(&req, sizeof(req)-sizeof(req.crc));

	return rq;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool CallBackRcvBootReq02(Ptr<REQ> &q)
{
	bool retry = false;

	if (!q->crcOK) 
	{
		//crcErr06++;

		retry = true;
	}
	else
	{
		BootRspV1::SF2 &rsp = *((BootRspV1::SF2*)q->rb.data);

		if (rsp.res != 0) /*wrtErr06++,*/ retry = true;
	};

	if (retry && q->tryCount > 0)
	{
		q->tryCount--;
		qRcv.Add(q);

		return false;
	};

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateRcvBootReq02(u16 adr, u16 stAdr, u16 count, void* data, u16 count2, void* data2, u16 pageLen, u16 tryCount)
{
	Ptr<REQ> rq(AllocREQ());
	
	//rq.Alloc();//= REQ::Alloc();

	if (!rq.Valid()) return rq;

	rq->rsp = AllocMemBuffer(sizeof(BootReqV1::SF2) + pageLen);

	if (!rq->rsp.Valid()) { rq.Free(); return rq; };

	BootRspV1::SF2 &rsp = *((BootRspV1::SF2*)rq->reqData);
	BootReqV1::SF2 &req = *((BootReqV1::SF2*)(rq->rsp->GetDataPtr()));
	
	REQ &q = *rq;

	q.CallBack = CallBackRcvBootReq02;
	q.preTimeOut = MS2COM(50);
	q.postTimeOut = US2COM(100);
	q.ready = false;
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;
	
	q.rb.data = &rsp;
	q.rb.maxLen = sizeof(rsp);

	req.adr	= adr;
	req.rw	= RCV_BOOT_REQ_WORD|2;

	u16 max = pageLen;

	if (count > max)
	{
		count = max;
		count2 = 0;
	}
	else if ((count + count2) > max)
	{
		count2 = max - count;
	};

	req.padr = stAdr;
	req.plen = count+count2;

	byte *d = (byte*)req.pdata;
	byte *s = (byte*)data;

	while(count > 0) *d++ = *s++, count--;

	if (data2 != 0)	{ s = (byte*)data2;	while(count2 > 0) *d++ = *s++, count2--; };

	u16 len = sizeof(req) - sizeof(req.pdata) + req.plen;

	u16 crc = GetCRC16(&req, len);

	*(d++) = crc;
	*(d++) = crc>>8;

	q.wb.data = &req;
	q.wb.len = len+2;

	return rq;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool CallBackRcvBootReq03(Ptr<REQ> &q)
{
	if (!q->crcOK) 
	{
		if (q->rb.len >=4)
		{
			BootReqV1::SF3 &req = *((BootReqV1::SF3*)q->wb.data);
			BootRspV1::SF3 &rsp = *((BootRspV1::SF3*)q->rb.data);

			if (rsp.adr == req.adr && rsp.rw == req.rw) return q->crcOK = true;
		};

		if (q->tryCount > 0)
		{
			q->tryCount--;
			qRcv.Add(q);

			return false;
		};
	};

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateRcvBootReq03(u16 adr, u16 tryCount)
{
	Ptr<REQ> rq(AllocREQ());
	
	//rq.Alloc();//= REQ::Alloc();

	if (!rq.Valid()) return rq;

	if (adr != 0)
	{
		rq->rsp = AllocMemBuffer(sizeof(BootRspV1::SF3));
		if (!rq->rsp.Valid()) { rq.Free(); return rq; };
	};

	BootReqV1::SF3 &req = *((BootReqV1::SF3*)rq->reqData);
	BootRspV1::SF3 &rsp = *((BootRspV1::SF3*)(rq->rsp->GetDataPtr()));
	
	REQ &q = *rq;

	q.CallBack = CallBackRcvBootReq03;
	q.preTimeOut = MS2COM(10);
	q.postTimeOut = US2COM(100);
	q.ready = false;
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;
	
	q.wb.data = &req;
	q.wb.len = sizeof(req);
	
	q.rb.data = (adr == 0) ? 0 : &rsp;
	q.rb.maxLen = sizeof(rsp);

	req.adr	= adr;
	req.rw	= RCV_BOOT_REQ_WORD|3;
	req.crc	= GetCRC16(&req, sizeof(req)-sizeof(req.crc));

	return rq;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool CallBackTrmBootReq(Ptr<REQ> &q)
{
	if (!q->crcOK) 
	{
		if (q->tryCount > 0)
		{
			q->tryCount--;
			qTrm.Add(q);
		};
	};

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateTrmBootReq00(u16 adr, BootRspV1::SF0 *rspdata, u16 tryCount)
{
	Ptr<REQ> rq(AllocREQ());
	
	if (!rq.Valid()) return rq;

	if (rspdata == 0 && adr != 0)
	{
		rq->rsp = AllocMemBuffer(sizeof(*rspdata));
		if (!rq->rsp.Valid()) { rq.Free(); return rq; };
		rspdata = (BootRspV1::SF0*)(rq->rsp->GetDataPtr());
	};

	BootReqV1::SF0 &req = *((BootReqV1::SF0*)rq->reqData);
	BootRspV1::SF0 &rsp = *rspdata;
	
	REQ &q = *rq;

	q.CallBack = CallBackTrmBootReq;
	q.preTimeOut = MS2COM(10);
	q.postTimeOut = US2COM(100);
	q.ready = false;
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;
	
	q.wb.data = &req;
	q.wb.len = sizeof(req);
	
	q.rb.data = (adr == 0) ? 0 : &rsp;
	q.rb.maxLen = sizeof(rsp);

	req.adr	= adr;
	req.rw	= TRM_BOOT_REQ_WORD|0;
	req.crc	= GetCRC16(&req, sizeof(req)-sizeof(req.crc));

	return rq;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateTrmBootReq01(u16 adr, u32 len, u16 tryCount)
{
	Ptr<REQ> rq(AllocREQ());
	
	if (!rq.Valid()) return rq;

	if (adr != 0)
	{
		rq->rsp = AllocMemBuffer(sizeof(BootRspV1::SF1));
		if (!rq->rsp.Valid()) { rq.Free(); return rq; };
	};

	BootReqV1::SF1 &req = *((BootReqV1::SF1*)rq->reqData);
	BootRspV1::SF1 &rsp = *((BootRspV1::SF1*)(rq->rsp->GetDataPtr()));
	
	REQ &q = *rq;

	q.CallBack = CallBackTrmBootReq;
	q.preTimeOut = MS2COM(10);
	q.postTimeOut = US2COM(100);
	q.ready = false;
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;
	
	q.wb.data = &req;
	q.wb.len = sizeof(req);
	
	q.rb.data = (adr == 0) ? 0 : &rsp;
	q.rb.maxLen = sizeof(rsp);

	req.adr	= adr;
	req.rw	= TRM_BOOT_REQ_WORD|1;
	req.len = len;
	req.crc	= GetCRC16(&req, sizeof(req)-sizeof(req.crc));

	return rq;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateTrmBootReq02(u16 adr, u16 stAdr, u16 count, void* data, u16 pageLen, u16 tryCount)
{
	Ptr<REQ> rq(AllocREQ());
	
	if (!rq.Valid()) return rq;

	rq->rsp = AllocMemBuffer(sizeof(BootReqV1::SF2) + pageLen);

	if (!rq->rsp.Valid()) { rq.Free(); return rq; };

	BootRspV1::SF2 &rsp = *((BootRspV1::SF2*)rq->reqData);
	BootReqV1::SF2 &req = *((BootReqV1::SF2*)(rq->rsp->GetDataPtr()));
	
	REQ &q = *rq;

	q.CallBack = CallBackTrmBootReq;
	q.preTimeOut = MS2COM(50);
	q.postTimeOut = US2COM(100);
	q.ready = false;
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;
	
	q.rb.data = &rsp;
	q.rb.maxLen = sizeof(rsp);

	req.adr	= adr;
	req.rw	= TRM_BOOT_REQ_WORD|2;

	u16 max = pageLen;

	if (count > max)
	{
		count = max;
//		count2 = 0;
	}
	//else if ((count + count2) > max)
	//{
	//	count2 = max - count;
	//};

	req.padr = stAdr;
	req.plen = count;//+count2;

	byte *d = (byte*)req.pdata;
	byte *s = (byte*)data;

	while(count > 0) *d++ = *s++, count--;

	//if (data2 != 0)	{ s = (byte*)data2;	while(count2 > 0) *d++ = *s++, count2--; };

	u16 len = sizeof(req) - sizeof(req.pdata) + req.plen;

	u16 crc = GetCRC16(&req, len);

	*(d++) = crc;
	*(d++) = crc>>8;

	q.wb.data = &req;
	q.wb.len = len+2;

	return rq;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateTrmBootReq03(u16 adr, u16 tryCount)
{
	Ptr<REQ> rq(AllocREQ());
	
	if (!rq.Valid()) return rq;

	if (adr != 0)
	{
		rq->rsp = AllocMemBuffer(sizeof(BootRspV1::SF3));
		if (!rq->rsp.Valid()) { rq.Free(); return rq; };
	};

	BootReqV1::SF3 &req = *((BootReqV1::SF3*)rq->reqData);
	BootRspV1::SF3 &rsp = *((BootRspV1::SF3*)(rq->rsp->GetDataPtr()));
	
	REQ &q = *rq;

	q.CallBack = CallBackTrmBootReq;
	q.preTimeOut = MS2COM(10);
	q.postTimeOut = US2COM(100);
	q.ready = false;
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;
	
	q.wb.data = &req;
	q.wb.len = sizeof(req);
	
	q.rb.data = (adr == 0) ? 0 : &rsp;
	q.rb.maxLen = sizeof(rsp);

	req.adr	= adr;
	req.rw	= TRM_BOOT_REQ_WORD|3;
	req.crc	= GetCRC16(&req, sizeof(req)-sizeof(req.crc));

	return rq;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateTrmBootReq04(u16 adr, u32 timeOut, u16 tryCount)
{
	Ptr<REQ> rq(AllocREQ());
	
	if (!rq.Valid()) return rq;

	if (adr != 0)
	{
		rq->rsp = AllocMemBuffer(sizeof(BootRspV1::SF4));
		if (!rq->rsp.Valid()) { rq.Free(); return rq; };
	};

	BootReqV1::SF4 &req = *((BootReqV1::SF4*)rq->reqData);
	BootRspV1::SF4 &rsp = *((BootRspV1::SF4*)(rq->rsp->GetDataPtr()));
	
	REQ &q = *rq;

	q.CallBack = CallBackTrmBootReq;
	q.preTimeOut = MS2COM(10);
	q.postTimeOut = US2COM(100);
	q.ready = false;
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;
	
	q.wb.data = &req;
	q.wb.len = sizeof(req);
	
	q.rb.data = (adr == 0) ? 0 : &rsp;
	q.rb.maxLen = sizeof(rsp);

	req.adr			= adr;
	req.rw			= TRM_BOOT_REQ_WORD|4;
	req.timeOutMS	= timeOut;
	req.crc			= GetCRC16(&req, sizeof(req)-sizeof(req.crc));

	return rq;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u32 InitRspMan_00(__packed u16 *data)
{
	__packed u16 *start = data;

	*(data++)	= (manReqWord & manReqMask) | 0;	//1. ответное слово
	*(data++)	= mv.numDevice;						//2. номер прибора
	*(data++)	= verDevice;						//3. версия прибора 

	*(data++)	= numDevRcv;						//4. номер модуля приёмников
	*(data++)	= verDevRcv;						//5. версия модуля приёмников
	*(data++)	= numDevTrm;						//6. номер модуля излучателей
	*(data++)	= verDevTrm;						//7. версия модуля излучателей

	return data - start;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void RequestFlashWrite_00(Ptr<MB> &flwb)
{
	__packed u16* data = (__packed u16*)(flwb->GetDataPtr());

	flwb->len = InitRspMan_00(data) * 2;

	NandFlash_RequestWrite(flwb, data[0], true);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_00(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	len = InitRspMan_00(manTrmData);

	mtb->data1 = manTrmData;
	mtb->len1 = len;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u32 InitRspMan_10(__packed u16 *data)
{
	__packed u16 *start = data;
	
	*(data++)	= (manReqWord & manReqMask) | 0x10;		//1.  Ответное слово

	*(data++)  	= mv.trans[0].freq;						//2.  Монополь1. Частота излучателя (1000..30000 шаг 1Гц)
	*(data++)  	= mv.trans[0].amp;						//3.  Монополь1. Амплитуда излучателя (0..3000 вольт)
	*(data++)  	= mv.trans[1].freq;						//4.  Монополь2. Частота импульсов излучателя (10000..30000 шаг 1Гц)
	*(data++)  	= mv.trans[1].amp;						//5.  Монополь2. Амплитуда излучателя (0..3000 вольт)
	*(data++)  	= mv.trans[2].pulseCount;				//6.  Диполь. Количество импульсов (1..5)
	*(data++)  	= mv.trans[2].freq;						//7.  Диполь. Частота импульсов излучателя (1000..10000 шаг 1Гц)
	*(data++)  	= mv.trans[2].duty;						//8.  Диполь. Скважность импульсов излучателя (0..60 шаг 0.01%)
	*(data++)  	= mv.trans[0].GetPreAmp();				//9.  Монополь1. Предусилитель(0..1)
	*(data++)  	= mv.trans[0].GetGain();				//10. Монополь1. КУ[0..7]-(1,2,4,8,16,32,64,128)
	*(data++)  	= mv.trans[0].st;						//11. Монополь1. Шаг оцифровки (2..50)
	*(data++)  	= mv.trans[0].sl;						//12. Монополь1. Длина оцифровки (16..1024)
	*(data++)  	= mv.trans[0].sd;						//13. Монополь1. Задержка оцифровки 
	*(data++)  	= mv.trans[0].packType;					//14. Монополь1. Тип упаковки (0-нет, 1 - uLaw 12 бит, 2 - uLaw 16 бит, 3 - ADPCMIMA, 4 - ДКП Низкое, 5 - ДКП Среднее, 6 - ДКП Высокое, 7 - ДКП Максимальное)
	*(data++)  	= mv.trans[0].math;						//15. Монополь1. Математика (0 - нет, 1 - Среднее по каналам) 
	*(data++)  	= mv.trans[1].GetPreAmp();				//16. Монополь2. Предусилитель(0..1)
	*(data++)  	= mv.trans[1].GetGain();				//17. Монополь2. КУ[0..7]-(1,2,4,8,16,32,64,128)
	*(data++)  	= mv.trans[1].st;						//18. Монополь2. Шаг оцифровки (2..50)
	*(data++)  	= mv.trans[1].sl;						//19. Монополь2. Длина оцифровки (16..1024)
	*(data++)  	= mv.trans[1].sd;						//20. Монополь2. Задержка оцифровки 
	*(data++)  	= mv.trans[1].packType;					//21. Монополь2. Тип упаковки (0-нет, 1 - uLaw 12 бит, 2 - uLaw 16 бит, 3 - ADPCMIMA, 4 - ДКП Низкое, 5 - ДКП Среднее, 6 - ДКП Высокое, 7 - ДКП Максимальное)
	*(data++)  	= mv.trans[1].math;						//22. Монополь2. Математика (0 - нет, 1 - Среднее по каналам) 
	*(data++)  	= mv.trans[2].GetPreAmp();				//23. Диполь. Предусилитель(0..1)
	*(data++)  	= mv.trans[2].GetGain();				//24. Диполь. КУ[0..7]-(1,2,4,8,16,32,64,128)
	*(data++)  	= mv.trans[2].st;						//25. Диполь. Шаг оцифровки (2..50)
	*(data++)  	= mv.trans[2].sl;						//26. Диполь. Длина оцифровки (16..1024)
	*(data++)  	= mv.trans[2].sd;						//27. Диполь. Задержка оцифровки 
	*(data++)  	= mv.trans[2].packType;					//28. Диполь. Тип упаковки (0-нет, 1 - uLaw 12 бит, 2 - uLaw 16 бит, 3 - ADPCMIMA, 4 - ДКП Низкое, 5 - ДКП Среднее, 6 - ДКП Высокое, 7 - ДКП Максимальное)
	*(data++)  	= mv.trans[2].math;						//29. Диполь. Математика (0 - нет, 2 - Разница 1-3 2-4) 
	*(data++)  	= mv.trmVoltage;						//30. Напряжение излучателя (0...950 вольт)
	*(data++)  	= mv.disableFireNoVibration;			//31. Отключение регистрации на стоянке(0 - нет, 1 - да)
	*(data++)  	= mv.levelNoVibration;					//32. Уровень вибрации режима отключения регистрации на стойнке(у.е)(ushort)
	*(data++)  	= mv.firePeriod;						//33. Период опроса(мс)(ushort)
#ifdef RCV_AUTO_GAIN
	*(data++)  	= mv.autoGain;							//34. Автоматическое усиление (0 - выкл, 1 - вкл)
#endif

	return data - start;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void RequestFlashWrite_10(Ptr<MB> &flwb)
{
	__packed u16* data = (__packed u16*)(flwb->GetDataPtr());

	flwb->len = InitRspMan_10(data) * 2;

	NandFlash_RequestWrite(flwb, data[0], true);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_10(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	len = InitRspMan_10(manTrmData);

	mtb->data1 = manTrmData;
	mtb->len1 = len;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u32 InitRspMan_20(u16 rw, __packed u16 *data)
{
	i16 max = -32767;
	i16 min = 32767;

	for (byte i = 0; i < RCV_MAX_NUM_STATIONS; i++)
	{
		if (rcvStatus & (1UL<<i))
		{
			i16 t = arrRcvTemp[i];

			if (t > max) max = t;
			if (t < min) min = t;
		};
	};

	__packed u16 *start = data;

	*(data++) = rw; //manReqWord|0x20;			//	1. 	ответное слово
	*(data++) = fireCounter.w[0];				//	2. 	счётчик. младшие 2 байта
	*(data++) = fireCounter.w[1];				//	3. 	счётчик. старшие 2 байта
	*(data++) = RCV_MAX_NUM_STATIONS;			//	4.	к-во приемников
	*(data++) = rcvStatus; 						//	5.	статус приёмников (бит 0 - П1, бит 1 - П2, ... , бит 12 - П13)
	*(data++) = rcvErrors;			 			//	6.	статус ошибок линии приёмников (бит 0 - П1, бит 1 - П2, ... , бит 12 - П13)
	*(data++) = Get_NetResist();	 			//	7.	сопротивление IdLine
	*(data++) = okRcv02;			 			//	8.	Счётчик запросов приёмников
	*(data++) = ax;								//	9.	AX
	*(data++) = ay;								//	10.	AY
	*(data++) = az;								//	11.	AZ
	*(data++) = at;					 			//	12.	AT
	*(data++) = temp;				 			//	13.	Температура в приборе (short)(0.1гр)
	*(data++) = min;				 			//	14.	Минимальное значение температуры в приёмниках (short)(0.1гр)
	*(data++) = max;				 			//	15.	Максимальное значение температуры в приёмниках (short)(0.1гр)
	*(data++) = vibration;			 			//	16.	Вибрация (у.е)(ushort)
	*(data++) = trmVoltageLast;			 		//	17.	напряжение излучателя (1 Вольт)
	*(data++) = trmTemp;			 			//	18.	Температура излучателя (short)(0.1гр)
	*(data++) = trmCount;			 			//	19.	Счётчик запросов излучателя
	*(data++) = GetRcvManQuality();	 			//	20.	Качество сигнала запроса телеметрии (%)
	*(data++) = rcvMisFire;			 			//	21. Счётчик пропусков оцифровки

	return data - start;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void RequestFlashWrite_20(Ptr<MB> &flwb)
{
	__packed u16* data = (__packed u16*)(flwb->GetDataPtr());

	flwb->len = InitRspMan_20(manReqWord|0x20|(fireMask&0xF), data) * 2;

	NandFlash_RequestWrite(flwb, data[0], true);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_20(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	fireMask = data[0] & 0xF;

	len = InitRspMan_20(data[0], manTrmData);

	rcvErrors = 0;

	mtb->data1 = manTrmData;
	mtb->len1 = len;
	mtb->data2 = 0;
	mtb->len2 = 0;

	runMainMode = true;

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_30(u16 *data, u16 reqlen, MTB* mtb)
{
	__packed struct Req { u16 rw; u16 off; u16 len; };

	Req &req = *((Req*)data);

	if (data == 0 || reqlen == 0 || reqlen > 4 || mtb == 0) return false;

	byte nf = ((req.rw >> 4) - 3) & 3;
	byte nr = req.rw & 15;

	Ptr<MB> *manVec = 0;

	if (nf == 0)
	{
		bool c = false;

		for (byte i = 0; i < ArraySize(curRcvMnpl); i++)
		{
			if (curRcvMnpl[i] == (nr+1))
			{
				tmVecMnpl[i].Reset();
				manVec = &(manVecMnpl[i]);
				c = true; //manVec->Valid();
				break;
			};
		};

		if (!c)
		{
			manVec = &(manVecMnpl[indRcvMnpl]);
			curRcvMnpl[indRcvMnpl] = (nr+1);
			indRcvMnpl = (indRcvMnpl+1) % ArraySize(curRcvMnpl);
		};
	}
	else
	{
		manVec = &manVec30[nf];
		curRcv[nf] = nr+1;
		curRcvTimer[nf].Reset();
	};

	struct Rsp { u16 rw; };
	static Rsp rsp; 
	
	static u16 prevOff = 0;
	static u16 prevLen = 0;
	static u16 maxLen = 200;

	//static byte sensInd = 0;

	rsp.rw = req.rw;

	mtb->data1 = (u16*)&rsp;
	mtb->len1 = sizeof(rsp)/2;
	mtb->data2 = 0;
	mtb->len2 = 0;

	if (reqlen == 1 || (reqlen >= 2 && data[1] == 0))
	{
		curManVec30.Free();

		if (manVec->Valid())
		{
			RspRcv02& rsp = *((RspRcv02*)(*manVec)->GetDataPtr());

			if (rsp.hdr.rw == req.rw) curManVec30 = *manVec;
			
			manVec->Free();
		};

		if (curManVec30.Valid())
		{
			RspRcv02 &rsp = *((RspRcv02*)(curManVec30->GetDataPtr()));

			u16 sz = (sizeof(rsp.hdr)-sizeof(rsp.hdr.rw))/2 + (rsp.hdr.packLen1+rsp.hdr.packLen2+rsp.hdr.packLen3+rsp.hdr.packLen4);

			mtb->data2 = ((u16*)&rsp)+1;

			prevOff = 0;

			if (reqlen == 1)
			{
				mtb->len2 = sz;
				prevLen = sz;
			}
			else 
			{
				req30_count1++;

				if (reqlen == 3) maxLen = data[2];

				u16 len = maxLen;

				if (len > sz) len = sz;

				mtb->len2 = len;

				prevLen = len;
			};
		};
	}
	else if (curManVec30.Valid())
	{
		RspRcv02& rsp = *((RspRcv02*)(curManVec30->GetDataPtr()));

		req30_count2++;

		u16 off = prevOff + prevLen;
		u16 len = prevLen;

		if (reqlen == 3)
		{
			off = data[1];
			len = data[2];
		};

		u16 sz = (sizeof(rsp.hdr)-sizeof(rsp.hdr.rw))/2 + (rsp.hdr.packLen1+rsp.hdr.packLen2+rsp.hdr.packLen3+rsp.hdr.packLen4);

		if (sz >= off)
		{
			req30_count3++;

			u16 ml = sz - off;

			if (len > ml) len = ml;

			mtb->data2 = (u16*)&rsp + data[1]+1;
			mtb->len2 = len;
		};
	};

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_71(u16 *data, u16 len, MTB* mtb)
{
	struct Rsp { u16 rw; };
	static Rsp rsp; 

	static byte index = 0;

	if (data == 0 || len == 0 || len > 4 || mtb == 0) return false;

	mtb->data2 = 0;
	mtb->len2 = 0;

	if (len < 3)
	{
		index = GetNextFireType(index);

		rspMan71[index].rw	= manReqWord|0x71;
		rspMan71[index].cnt = fireCounter;
		rspMan71[index].n	= index;

		mtb->data1 = (u16*)&rspMan71[index];
		mtb->len1 = sizeof(rspMan71[index])/2;
	}
	else if (data[1] == 0)
	{
		index = GetNextFireType(index);

		rspMan71[index].rw = manReqWord|0x71;
		rspMan71[index].cnt = fireCounter;
		rspMan71[index].n	= index;

		u16 maxlen = sizeof(rspMan71[index])/2;
		u16 len = data[2]+1;

		if (len > maxlen) len = maxlen;

		mtb->data1 = (u16*)&rspMan71[index];
		mtb->len1 = len;
	}
	else
	{
		rsp.rw = manReqWord|0x71;

		mtb->data1 = (u16*)&rsp;
		mtb->len1 = sizeof(rsp)/2;

		if (sizeof(rspMan71[index])/2 > data[1])
		{
			u16 maxlen = sizeof(rspMan71[index])/2 - data[1] - 1;
			u16 len = data[2];

			if (len > maxlen) len = maxlen;

			mtb->data2 = ((u16*)&rspMan71[index]) + data[1]+1;
			mtb->len2 = len;
		};
	};

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_72(u16 *data, u16 len, MTB* mtb)
{
	static u16 buf; 

	if (data == 0 || len == 0 || len > 4 || mtb == 0) return false;

	if (curManVec72.Valid()) curManVec72.Free();

	curManVec72 = manVec72[indManVec72];

	if (!curManVec72.Valid())
	{
		indManVec72 = (indManVec72+1) & 1;

		curManVec72 = manVec72[indManVec72];
	};

	if (curManVec72.Valid())
	{
		indManVec72 = (indManVec72+1) & 1;

		mtb->data1 = (u16*)curManVec72->GetDataPtr();
		mtb->len1 = curManVec72->len/2;
	}
	else
	{
		buf = manReqWord|0x72;

		mtb->data1 = &buf;
		mtb->len1 = sizeof(buf)/2;
	};

	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u32 InitRspMan_73(u16 rw, __packed u16 *data)
{
	__packed u16 *start = data;

	*(data++) = rw; 																	//	1. 	ответное слово

	for (byte i = 0; i < RCV_MAX_NUM_STATIONS; i++) 	*(data++) = arrRcvTemp[i];		//	2. Температура приёмника 1

	return data - start;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void RequestFlashWrite_73(Ptr<MB> &flwb)
{
	__packed u16* data = (__packed u16*)(flwb->GetDataPtr());

	flwb->len = InitRspMan_73(manReqWord|0x73, data) * 2;

	NandFlash_RequestWrite(flwb, data[0], true);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_73(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 4 || mtb == 0) return false;

	len = InitRspMan_73(data[0], manTrmData);

	mtb->data1 = manTrmData;
	mtb->len1 = len;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_80(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len < 3 || len > 4 || mtb == 0) return false;

	switch (data[1])
	{
		case 1:		mv.numDevice	= data[2];							break;
		case 2:		manTrmBaud		= data[2] - 1;						break;
		case 3:		numDevRcv		= data[2]; numDevRcvValid = true;	break;
		case 4:		numDevTrm		= data[2]; numDevTrmValid = true;	break;
	};

	manTrmData[0] = (manReqWord & manReqMask) | 0x80;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static bool RequestMan_90(u16 *data, u16 len, MTB* mtb)
//{
//	if (data == 0 || len < 3 || len > 4 || mtb == 0) return false;
//
//	cmdWriteStart_10 = true;


static bool RequestMan_90(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len < 3 || len > 4 || mtb == 0) return false;

	u16 nr = data[1];

	switch(nr)
	{
		case 0x00:	mv.trans[0].freq			= LIM(data[2],	10000,	30000			);	break;	//	0x00 - Монополь1. Частота излучателя (1000..30000 шаг 1Гц)
		case 0x01:	mv.trans[0].amp				= MIN(data[2],			3000			);	break;	//	0x01 - Монополь1. Амплитуда излучателя (0..3000 вольт)
																									
		case 0x10:	mv.trans[1].freq			= LIM(data[2],	1000,	10000			);	break;	//	0x10 - Монополь2. Частота импульсов излучателя (10000..30000 шаг 1Гц)
		case 0x11:	mv.trans[1].amp				= MIN(data[2],			2100			);	break;	//	0x11 - Монополь2. Амплитуда излучателя (0..3000 вольт)
																									
		case 0x20:	mv.trans[2].pulseCount		= LIM(data[2],	1,		5				);	break;	//	0x20 - Диполь. Количество импульсов (1..5)
		case 0x21:	mv.trans[2].freq			= LIM(data[2],	500,	10000			);	break;	//	0x21 - Диполь. Частота импульсов излучателя (1000..10000 шаг 1Гц)
		case 0x22:	mv.trans[2].duty			= MIN(data[2],			6000			);	break;	//	0x22 - Диполь. Скважность импульсов излучателя (0..60 шаг 0.01%)
																									
		case 0x30:	mv.trans[0].SetPreAmp(		  MIN(data[2],			1				));	break;	//	0x30 - Монополь1. Предусилитель(0..1)
		case 0x31:	mv.trans[0].SetGain(		  MIN(data[2],			7				));	break;	//	0x31 - Монополь1. КУ[0..7]-(1,2,4,8,16,32,64,128)
		case 0x32:	mv.trans[0].st				= LIM(data[2],	2,		50				);	break;	//	0x32 - Монополь1. Шаг оцифровки (2..50)
		case 0x33:	mv.trans[0].sl				= LIM(data[2],	16,		RCV_SAMPLE_LEN	);	break;	//	0x33 - Монополь1. Длина оцифровки (16..1024)
		case 0x34:	mv.trans[0].sd				= MIN(data[2],			1024			);	break;	//	0x34 - Монополь1. Задержка оцифровки 
		case 0x35:	mv.trans[0].packType		= MIN(data[2],			7				);	break;	//	0x35 - Монополь1. Тип упаковки (0-нет, 1 - uLaw 12 бит, 2 - uLaw 16 бит, 3 - ADPCMIMA, 4 - ДКП Низкое, 5 - ДКП Среднее, 6 - ДКП Высокое, 7 - ДКП Максимальное)
		case 0x36:	mv.trans[0].math			= data[2];									break;	//	0x36 - Монополь1. Математика (0 - нет, 1 - Среднее по каналам) 
																									
		case 0x40:	mv.trans[1].SetPreAmp(		  MIN(data[2],			1				));	break;	//	0x40 - Монополь1. Предусилитель(0..1)
		case 0x41:	mv.trans[1].SetGain(		  MIN(data[2],			7				));	break;	//	0x41 - Монополь1. КУ[0..7]-(1,2,4,8,16,32,64,128)
		case 0x42:	mv.trans[1].st				= LIM(data[2],	2,		50				);	break;	//	0x42 - Монополь2. Шаг оцифровки (2..50)
		case 0x43:	mv.trans[1].sl				= LIM(data[2],	16,		RCV_SAMPLE_LEN	);	break;	//	0x43 - Монополь2. Длина оцифровки (16..1024)
		case 0x44:	mv.trans[1].sd				= MIN(data[2],			1024			);	break;	//	0x44 - Монополь2. Задержка оцифровки 
		case 0x45:	mv.trans[1].packType		= MIN(data[2],			7				);	break;	//	0x45 - Монополь2. Тип упаковки (0-нет, 1 - uLaw 12 бит, 2 - uLaw 16 бит, 3 - ADPCMIMA, 4 - ДКП Низкое, 5 - ДКП Среднее, 6 - ДКП Высокое, 7 - ДКП Максимальное)
		case 0x46:	mv.trans[1].math			= data[2];									break;	//	0x46 - Монополь2. Математика (0 - нет, 1 - Среднее по каналам) 
																									
		case 0x50:	mv.trans[2].SetPreAmp(		  MIN(data[2],			1				));	break;	//	0x50 - Диполь. Предусилитель(0..1)
		case 0x51:	mv.trans[2].SetGain(		  MIN(data[2],			7				));	break;	//	0x51 - Диполь. КУ[0..8]-(1,2,4,8,16,32,64,128,Авто)
		case 0x52:	mv.trans[2].st				= LIM(data[2],	2,		50				);	break;	//	0x52 - Диполь. Шаг оцифровки (2..50)
		case 0x53:	mv.trans[2].sl				= LIM(data[2],	16,		RCV_SAMPLE_LEN	);	break;	//	0x53 - Диполь. Длина оцифровки (16..1024)
		case 0x54:	mv.trans[2].sd				= MIN(data[2],			1024			);	break;	//	0x54 - Диполь. Задержка оцифровки 
		case 0x55:	mv.trans[2].packType		= MIN(data[2],			7				);	break;	//	0x55 - Диполь. Тип упаковки (0-нет, 1 - uLaw 12 бит, 2 - uLaw 16 бит, 3 - ADPCMIMA, 4 - ДКП Низкое, 5 - ДКП Среднее, 6 - ДКП Высокое, 7 - ДКП Максимальное)
		case 0x56:	mv.trans[2].math			= data[2];									break;	//	0x56 - Диполь. Математика (0 - нет, 2 - Разница 1-3 2-4) 
																									
		case 0x60:	mv.trmVoltage				= MIN(data[2],			600				);	break;	//	0x60 - Напряжение излучателя (0...950 вольт)
		case 0x61:	mv.disableFireNoVibration	= data[2];									break;	//	0x61 - Отключение регистрации на стоянке(0 - нет, 1 - да)
		case 0x62:	mv.levelNoVibration			= data[2];									break;	//	0x62 - Уровень вибрации режима отключения регистрации на стойнке(у.е)(ushort)
		case 0x63:	mv.firePeriod				= MAX(data[2],			300				);	break;	//	0x63 - Период опроса(мс)(ushort)

#ifdef RCV_AUTO_GAIN
		case 0x70:	mv.autoGain					= MIN(data[2],			1				);	break;	//	0x70 - Автоматическое усиление (0 - выкл, 1 - вкл)
#endif

		default:	return false;
	};

	//if (nr >= 0x10 && nr < 0x30) qRcv.Add(CreateRcvReq03(0, 2));

	manTrmData[0] = (manReqWord & manReqMask) | 0x90;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_F0(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	SaveMainParams();

	cmdRcvSaveParams = true;
	cmdTrmSaveParams = true;

	manTrmData[0] = (manReqWord & manReqMask) | 0xF0;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan(u16 *buf, u16 len, MTB* mtb)
{
	if (buf == 0 || len == 0 || mtb == 0) return false;

	bool r = false;

	byte i = buf[0] & ~manReqMask;

	if ((i & 0xF0) == 0x20) i = 0x20;
	if (i >= 0x30 && i <= 0x6F) i = 0x30;

	switch (i)
	{
		case 0x00: 	r = RequestMan_00(buf, len, mtb); break;
		case 0x10: 	r = RequestMan_10(buf, len, mtb); break;
		case 0x20: 	r = RequestMan_20(buf, len, mtb); break;
		case 0x30:	r = RequestMan_30(buf, len, mtb); break;
		case 0x71: 	r = RequestMan_71(buf, len, mtb); break;
		case 0x72: 	r = RequestMan_72(buf, len, mtb); break;
		case 0x73: 	r = RequestMan_73(buf, len, mtb); break;
		case 0x80: 	r = RequestMan_80(buf, len, mtb); break;
		case 0x90:	r = RequestMan_90(buf, len, mtb); break;
		case 0xF0:	r = RequestMan_F0(buf, len, mtb); break;
	};

	if (r)
	{ 
		mtb->baud = manTrmBaud; 
	
	#if defined(_DEBUG) && !defined(WIN32)
		if (mtb->data1 != 0 && !HW::RamCheck(mtb->data1)) __breakpoint(0);
		if (mtb->data2 != 0 && !HW::RamCheck(mtb->data2)) __breakpoint(0);
	#endif
	};

	return r;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_00(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	manTrmData[0] = (memReqWord & memReqMask) | 0;
	manTrmData[1] = mv.numMemDevice;
	manTrmData[2] = verMemDevice;

	mtb->data1 = manTrmData;
	mtb->len1 = 3;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_10(u16 *data, u16 len, MTB* mtb)
{
	//__packed struct T { u16 g[8]; u16 st; u16 len; u16 delay; u16 voltage; };
	//__packed struct Rsp { u16 hdr; u16 rw; T t1, t2, t3; };
	
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	manTrmData[0] = (memReqWord & memReqMask) | 0x10;
	manTrmData[1] = NandFlash_Chip_Mask_Get();
	manTrmData[2] = NandFlash_Full_Size_Get() >> 20;

	mtb->data1 = manTrmData;
	mtb->len1 = 3;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_20(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	__packed struct Rsp 
	{
		u16 rw;						//	1. ответное слово
		u16 device;					//	2.Идентификатор прибора
		u16 session;				//	3.Идентификатор сессии
		u32 rcvVec;					//	4.Количество принятых векторов. младшие 2 байта
		u32 rejVec;					//	6.Количество пропущенных векторов. младшие 2 байта
		u32 wrVec;					//	8.Количество записанных векторов. младшие 2 байта
		u32 errVec;					//	10.Количество ошибок записи векторов. младшие 2 байта
		u16 wrAdr[3];				//	12.Текущий адрес памяти. младшие 2 байта
		u16 temp;					//	15.Температура внутри модуля (гр)
		byte status;				//	16.Статус(мл) + Флаги(ст) статус: 0 - ожидание, 1 - запись, 2 - чтение, 3 - стирание
		byte flags;
		RTC rtc;					//	17. ДатаВремя
		u16 blockErr;				//	21. Счётчик ошибок стирания блоков
		u16 pageErr;				//	22. Счётчик ошибок записи страниц
		u16 percentFree;			//	23. Свободно памяти (0.01%)
		u16 eccErrCount;			//	24. Счётчик неисправимых ошибок ECC
		u16 eccCorrErrCount;		//	25. Счётчик коррекций ECC 
		u16 eccParityErrCount;		//	26. Счётчик ошибок в коде ECC 
	};

	if (len != 1) return false;

	Rsp &rsp = *((Rsp*)&manTrmData);

	rsp.rw = (memReqWord & memReqMask)|0x20;
	rsp.device = NandFlash_GetDeviceID();  
	rsp.session = NandFlash_Session_Get();	  
	rsp.rcvVec =  NandFlash_Vectors_Recieved_Get();
	rsp.rejVec = rcv02rejVec; //NandFlash_Vectors_Rejected_Get();
	rsp.wrVec = NandFlash_Vectors_Saved_Get();
	rsp.errVec = NandFlash_Vectors_Errors_Get();
	*((__packed u64*)rsp.wrAdr) = NandFlash_Current_Adress_Get();
	rsp.temp = (temp+5)/10;
	rsp.status = NandFlash_Status();

	GetTime(&rsp.rtc);

	rsp.blockErr	= NandFlash_BlockErr_Get();
	rsp.pageErr		= NandFlash_PageErr_Get();

	u32 used = NandFlash_Used_Size_Get() >> 20;
	u32 full = NandFlash_Full_Size_Get() >> 20;
	
	if (full < used) used = full;

	rsp.percentFree	= ((full - used) * 10000 + full/2) / full;

	rsp.eccErrCount			= NandFlash_ErrECC_Get();		
	rsp.eccCorrErrCount		= NandFlash_CorrectedErrECC_Get();	
	rsp.eccParityErrCount	= NandFlash_ParityErrECC_Get();

	mtb->data1 = manTrmData;
	mtb->len1 = sizeof(rsp)/2;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_30(u16 *data, u16 len, MTB* mtb)
{
	if (len != 5) return false;

	SetClock(*(RTC*)(&data[1]));

	manTrmData[0] = (memReqWord & memReqMask)|0x30;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_31(u16 *data, u16 len, MTB* mtb)
{
	if (len != 1) return false;

	cmdWriteEnable = true;

	manTrmData[0] = (memReqWord & memReqMask)|0x31;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_32(u16 *data, u16 len, MTB* mtb)
{
	if (len != 1) return false;

	cmdWriteDisable = true;

	manTrmData[0] = (memReqWord & memReqMask)|0x32;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_33(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 4 || mtb == 0) return false;

	// Erase all

	manTrmData[0] = (memReqWord & memReqMask)|0x33;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_80(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len < 3 || len > 4 || mtb == 0) return false;

	switch (data[1])
	{
		case 1:

			mv.numMemDevice = data[2];

			break;

		case 2:

			memTrmBaud = data[2] - 1;	//SetTrmBoudRate(data[2]-1);

			break;
	};

	manTrmData[0] = (memReqWord & memReqMask) | 0x80;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_90(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len < 1 || mtb == 0) return false;

	manTrmData[0] = (memReqWord & memReqMask) | 0x90;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_F0(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	SaveMainParams();

	manTrmData[0] = (memReqWord & memReqMask) | 0xF0;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


static bool RequestMem(u16 *buf, u16 len, MTB* mtb)
{
	if (buf == 0 || len == 0 || mtb == 0) return false;

	bool r = false;

	byte i = buf[0]&0xFF;

	switch (i)
	{
		case 0x00: 	r = RequestMem_00(buf, len, mtb); break;
		case 0x10: 	r = RequestMem_10(buf, len, mtb); break;
		case 0x20: 	r = RequestMem_20(buf, len, mtb); break;
		case 0x30: 	r = RequestMem_30(buf, len, mtb); break;
		case 0x31: 	r = RequestMem_31(buf, len, mtb); break;
		case 0x32: 	r = RequestMem_32(buf, len, mtb); break;
		case 0x33: 	r = RequestMem_33(buf, len, mtb); break;
		case 0x80: 	r = RequestMem_80(buf, len, mtb); break;
		case 0x90: 	r = RequestMem_90(buf, len, mtb); break;
		case 0xF0: 	r = RequestMem_F0(buf, len, mtb); break;
	};

	if (r) { mtb->baud = memTrmBaud; };

	return r;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateMan()
{
	static MTB mtb;
	static MRB mrb;

	static byte i = 0;

	static CTM32 tm;


//	u16 c;

	switch (i)
	{
		case 0:

//			HW::P5->BSET(7);

			mrb.data = manRcvData;
			mrb.maxLen = ArraySize(manRcvData);
			RcvManData(&mrb);

			i++;

			break;

		case 1:

			ManRcvUpdate();

			if (mrb.ready)
			{
				tm.Reset();

				if (mrb.OK && mrb.len > 0 &&	(((manRcvData[0] & manReqMask) == manReqWord && RequestMan(manRcvData, mrb.len, &mtb)) 
										||		((manRcvData[0] & memReqMask) == memReqWord && RequestMem(manRcvData, mrb.len, &mtb))))
				{
					//HW::P5->BCLR(7);

					i++;
				}
				else
				{
					i = 0;
				};
			}
			else if (mrb.len > 0)
			{

			};

			break;

		case 2:

			if (tm.Check(US2CTM(100)))
			{
				//manTrmData[0] = 1;
				//manTrmData[1] = 0;
				//mtb.len1 = 2;
				//mtb.data1 = manTrmData;

			#if defined(_DEBUG) && !defined(WIN32)
				if (mtb.data1 != 0 && !HW::RamCheck(mtb.data1)) __breakpoint(0);
				if (mtb.data2 != 0 && !HW::RamCheck(mtb.data2)) __breakpoint(0);
			#endif	

				SendManData(&mtb);

				i++;
			};

			break;

		case 3:

			if (mtb.ready)
			{
				i = 0;
			};

			break;

	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void StartTrmFire()
{
#ifndef WIN32

	__disable_irq();

	PIO_RTS4->BSET(PIN_RTS4);
	PIO_RTS7->BSET(PIN_RTS7);

	HW::USART4->CTRLB |= USART_TXEN;
	HW::USART7->CTRLB |= USART_TXEN;

	HW::USART4->DATA = 0;
	HW::USART7->DATA = 0;

	__enable_irq();

#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void StopTrmFire()
{
#ifndef WIN32

	__disable_irq();

	PIO_RTS4->BCLR(PIN_RTS4);
	PIO_RTS7->BCLR(PIN_RTS7);

	HW::USART4->CTRLB &= ~USART_TXEN;
	HW::USART7->CTRLB &= ~USART_TXEN;

	__enable_irq();

#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateRcvTrm()
{
	static byte i = 0, n = 0, next_n = 0;
	//static u16 sd = 0;
	static CTM32 ctm;

	static u32 fireTime = MS2CTM(15);

	Ptr<REQ> reqr;
	Ptr<REQ> reqt;
			
	switch(i)
	{
		case 0:
			
			qRcv.Update();

			i = (startFire) ? 2 : 1;

			break;

		case 1:

			qTrm.Update();
		
			if (!startFire && ctm.Check(MS2CTM(20)))
			{
				qTrm.Add(CreateTrmReq02(2));
				qTrm.Add(CreateTrmReq03(2));
			};

			i = (startFire) ? 2 : 0;

			break;

		case 2:

			qRcv.Stop();
			qTrm.Stop();
			n = fireType;
			next_n = nextFireType;

			i++;

			break;


		case 3:

			qRcv.Update();
			qTrm.Update();
 
			if (qRcv.Stoped() && qTrm.Stoped())
			{
				trmVoltageLast = trmVoltage;

				ctm.Reset();
				i++;
			};

			break;

		case 4:

			if (ctm.Check(US2CTM(200)))
			{
				reqr = CreateRcvReqFire(n, next_n, fireCounter);

				if (reqr.Valid())
				{
					comRcv.Write(&reqr->wb);
					
					i++;
				};
			};

			break;

		case 5:

			if (mv.trmVoltage > 0)
			{
				reqt = CreateTrmReqFire(n);

				if (reqt.Valid())
				{
					comTrm.Write(&reqt->wb);

					i++;
				};
			}
			else
			{
				i++;
			};

			break;

		case 6:

			if (!comRcv.Update() && !comTrm.Update())
			{
				reqr.Free();
				reqt.Free();

				ctm.Reset();
				i++;
			};

			break;

		case 7:

			if (ctm.Check(US2CTM(200)))
			{
				StartTrmFire();

				byte tn = transIndex[n];

				u32 t = 2000 + ((u32)mv.trans[tn].sl+64) * mv.trans[tn].st + mv.trans[tn].sd;

				fireTime = US2CTM(t);

				i++;
			};

			break;

		case 8:

			if (ctm.Check(fireTime))
			{
				StopTrmFire();

				startFire = false;

				qRcv.Start();
				qTrm.Start();
				i = 0;
			};

			break;
	};

}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void MainMode()
{
	//fireType

	static byte rcv = 0;
	//static byte chnl = 0;
	//static REQ *req = 0;
	//static R02 *r02 = 0;
	static CTM32 ctm;
	static CTM32 ctm2;
	static CTM32 ctm3;

	static byte saveRcvParams = 0;

	static Ptr<REQ> req;
//	REQ *rm = 0;

	switch (mainModeState)
	{
		case 0:

			if (runMainMode && (mv.disableFireNoVibration == 0 || vibration > mv.levelNoVibration))
			{
				req = CreateRcvReq03(0, 0);

				if (req.Valid())
				{
					qRcv.Add(req);

					if (cmdRcvSaveParams) saveRcvParams = 1, cmdRcvSaveParams = false;

					mainModeState++;
				};
			}
			else if (cmdRcvSaveParams) 
			{
				req = CreateRcvReq04(0, 1, 0);

				if (req.Valid())
				{
					qRcv.Add(req);

					cmdRcvSaveParams = false;
				};
			}
			else
			{
				trmVoltageLast = trmVoltage;	
			};

			break;

		case 1:

			if (req->ready)
			{
				startFire = true;

				req.Free();
				
				mainModeState++;
			};

			break;

		case 2:

			if (!startFire)
			{
				rcv = 1;

				mainModeState++;
			};

			break;

		case 3: // Считывание вектора без сжатия и без математики

			req = CreateRcvReq02(rcv, fireType, 5);

			if (req.Valid())
			{
				qRcv.Add(req);

				mainModeState++;
			};

			break;

		case 4:

			if (req->ready)
			{
				if (req->crcOK && req->rsp.Valid())
				{
					RspRcv02 &r02 = *((RspRcv02*)(req->rsp->GetDataPtr()));

				#ifndef RCV_8AD
					bool crc = false;
				#else
					bool crc = true;
				#endif

					u16 rw = manReqWord | ((3+fireType) << 4) | (rcv - 1);
					
					//if (r02.hdr.rw != rw || r02.hdr.cnt != fireCounter)
					//{
					//	//__breakpoint(0);

					//	//r02.hdr.rw = manReqWord | ((3+fireType) << 4) | (rcv - 1);
					//	//r02.hdr.cnt = fireCounter;
					//	//crc = true;
					//};

					NandFlash_RequestWrite(req->rsp, r02.hdr.rw, crc);

					u16 n = ((r02.hdr.rw >> 4) & 0xF) - 3;
					u16 r = r02.hdr.rw & 0xF;

					if ((r02.hdr.rw & manReqMask) == manReqWord && n < RCV_FIRE_NUM && r < RCV_MAX_NUM_STATIONS)
					{
						bool c = false;

						Ptr<MB> *manVec = 0;

						if (n == 0) // Monopole HF
						{
							for (byte i = 0; i < ArraySize(curRcvMnpl); i++)
							{
								if (curRcvMnpl[i] == (r+1))
								{
									manVec = &(tmpVecMnpl[i]);
									c = true;
									break;
								};
							};
						}
						else if (curRcv[n] == (r+1))
						{
							c = true;

							manVec = &(manVec30[n]);
						};

						if (c) 
						{
							if (mv.trans[transIndex[n]].packType == 0 && mv.trans[transIndex[n]].math == 0)
							{
								*manVec = req->rsp;
							}
							else
							{
								qRcv.Add(CreateRcvReq05(rcv, fireType, 3));
							};

						};
					};

					manCounter++;
				};

				if (rcv < numStations)
				{
					rcv += 1;

					mainModeState++;
				}
				else
				{
					rcv = 1;
					mainModeState = 6;
				};

				ctm3.Reset();
			};

			qRcv.Update();

			break;

		case 5:

			if (ctm3.Check(US2CTM(100)))
			{
				mainModeState = 3;
			};

			break;

		case 6:

			req = CreateRcvReq04(rcv, saveRcvParams, 2);

			if (req.Valid())
			{
				qRcv.Add(req);

				mainModeState++;
			};

			break;

		case 7:

			if (req->ready)
			{
				if (rcv < numStations)
				{
					rcv++;

					mainModeState -= 1;
				}
				else
				{
					saveRcvParams = 0;

					mainModeState++;
				};
			};

			break;

		case 8:
		{
			#ifdef RCV_AUTO_GAIN

			if (mv.autoGain && fireType != 2) // DipoleX
			{
				byte n = transIndex[fireType];

				//if (fireType > 1) bufMaxAmp[(bufMaxAmpIndex++)&15] = autoGain[n].maxAmp;

				if (autoGain[n].maxAmp > RCV_AUTO_GAIN_HI_AMP)
				{
					autoGain[n].Dec();
				}
				else if (autoGain[n].maxAmp < RCV_AUTO_GAIN_LO_AMP)
				{
					autoGain[n].Inc();
				};

				autoGain[n].maxAmp = 0;
			};

			#endif

			if (fireType == 0)
			{
				if (!(manVecMnpl[0].Valid() || manVecMnpl[1].Valid()))
				{
					manVecMnpl[0] = tmpVecMnpl[0]; tmpVecMnpl[0].Free();
					manVecMnpl[1] = tmpVecMnpl[1]; tmpVecMnpl[1].Free();
				};

				if (tmVecMnpl[0].Check(RCV_MAN_VEC_TIMOUT)) curRcvMnpl[0] = 0, manVecMnpl[0].Free(), tmpVecMnpl[0].Free();
				if (tmVecMnpl[1].Check(RCV_MAN_VEC_TIMOUT)) curRcvMnpl[1] = 0, manVecMnpl[1].Free(), tmpVecMnpl[1].Free();
			}
			else if (curRcvTimer[fireType].Check(RCV_MAN_VEC_TIMOUT)) curRcv[fireType] = 0, manVec30[fireType].Free();

			byte pft = fireType;

			fireType = nextFireType; 

			nextFireType = GetNextFireType(nextFireType); //do nextFireType = (nextFireType+1)%RCV_FIRE_NUM; while((fireMask & (1UL<<nextFireType)) == 0);

			fireCounter.d += 1;

			if (fireType > pft)
			{
				mainModeState++;
			}
			else
			{
				mainModeState = 10;
			};

			break;
		};

		case 9:

			if (ctm.Check(MS2CTM(mv.firePeriod/4)))
			{
				mainModeState = 0;
			};

			break;

		case 10:

			if (ctm2.Check(MS2CTM(mv.firePeriod)))
			{
				ctm.Reset();
				mainModeState = 0;

				if (cmdWriteEnable)
				{
					NandFlash_WriteEnable();

					cmdWriteStart_00 = cmdWriteStart_10 = 3;

					cmdWriteEnable = false;
				}
				else if (cmdWriteDisable)
				{
					NandFlash_WriteDisable();

					cmdWriteDisable = false;
				}
				else
				{
					cmdWriteStart_00 = 1;
				};

				cmdWriteStart_20 = true;
				cmdWriteStart_73 = true;
			};

			break;
	};

	if (cmdWriteStart_00)
	{
		Ptr<MB> b(NandFlash_AllocWB(6));

		if (b.Valid())
		{
			RequestFlashWrite_00(b);

			cmdWriteStart_00 -= 1;
		};
	}
	else if (cmdWriteStart_10)
	{
		Ptr<MB> b(NandFlash_AllocWB(44));

		if (b.Valid())
		{
			RequestFlashWrite_10(b);

			cmdWriteStart_10 -= 1;
		};
	}
	else if (cmdWriteStart_20)
	{
		Ptr<MB> b(NandFlash_AllocWB(60));

		if (b.Valid())
		{
			RequestFlashWrite_20(b);

			cmdWriteStart_20 = false;
		};
	}
	else if (cmdWriteStart_73)
	{
		Ptr<MB> b(NandFlash_AllocWB(32));

		if (b.Valid())
		{
			RequestFlashWrite_73(b);

			cmdWriteStart_73 = false;
		};
	};

	//else if (tm.Check(1001))
	//{
	//	cmdWriteStart_20 = true;
	//};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static DSCSPI dscAccel;

//static i16 ax = 0, ay = 0, az = 0, at = 0;


static u8 txAccel[25] = { 0 };
static u8 rxAccel[50];

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool AccelReadReg(byte reg, u16 count)
{
	dscAccel.adr = (reg<<1)|1;
	dscAccel.alen = 1;
	//dscAccel.baud = 8000000;
	dscAccel.csnum = 0;
	dscAccel.wdata = 0;
	dscAccel.wlen = 0;
	dscAccel.rdata = rxAccel;
	dscAccel.rlen = count;

	return SPI_AddRequest(&dscAccel);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool AccelWriteReg(byte reg, u16 count)
{
	dscAccel.adr = (reg<<1)|0;
	dscAccel.alen = 1;
	dscAccel.csnum = 0;
	dscAccel.wdata = txAccel;
	dscAccel.wlen = count;
	dscAccel.rdata = 0;
	dscAccel.rlen = 0;

	return SPI_AddRequest(&dscAccel);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateAccel()
{
	static byte i = 0; 
	static i32 fx = 0, fy = 0, fz = 0, fv = 0, ft = 0;

	static TM32 tm;

	switch (i)
	{
		case 0:

			txAccel[0] = 0x52;
			AccelWriteReg(0x2F, 1); // Reset

			i++;

			break;

		case 1:

			if (dscAccel.ready)
			{
				tm.Reset();

				i++;
			};

			break;

		case 2:

			if (tm.Check(35))
			{
				AccelReadReg(0x1E, 18);

				i++;
			};

			break;

		case 3:

			if (dscAccel.ready)
			{
				txAccel[0] = 0;
				AccelWriteReg(0x28, 1); // FILTER SETTINGS REGISTER

				i++;
			};

			break;

		case 4:

			if (dscAccel.ready)
			{
				txAccel[0] = 0;
				AccelWriteReg(0x2D, 1); // CTRL Set PORST to zero

				i++;
			};

			break;

		case 5:

			if (dscAccel.ready)
			{
				AccelReadReg(0x2D, 1);

				i++;
			};

			break;

		case 6:

			if (dscAccel.ready)
			{
				if (rxAccel[0] != 0)
				{
					txAccel[0] = 0;
					AccelWriteReg(0x2D, 1); // CTRL Set PORST to zero
					i--; 
				}
				else
				{
					txAccel[0] = 0;
					AccelWriteReg(0x2E, 1); // Self Test

					tm.Reset();
					i++;
				};
			};

			break;

		case 7:

			if (dscAccel.ready)
			{
				i++;
			};

			break;

		case 8:

			if (tm.Check(10))
			{
				AccelReadReg(6, 11); // X_MSB 

				i++;
			};

			break;

		case 9:

			if (dscAccel.ready)
			{
				i32 t = (rxAccel[0] << 8)  | rxAccel[1];
				i32 x = (rxAccel[2] << 24) | (rxAccel[3] << 16) | (rxAccel[4]  <<8);
				i32 y = (rxAccel[5] << 24) | (rxAccel[6] << 16) | (rxAccel[7]  <<8);
				i32 z = (rxAccel[8] << 24) | (rxAccel[9] << 16) | (rxAccel[10] <<8);

				fx += (x - fx) / 16;
				fy += (y - fy) / 16;
				fz += (z - fz) / 16;
				ft += (t - ft) / 4;

				ay = -(fz / 65536); 
				ax = -(fy / 65536); 
				az =  (fx / 65536);

				//at = 2500 + ((1852 - t) * 2000 + 91) / 181;
				at = 2500 + ((1852 - ft) * 11315 + 512) / 1024;

				i32 vx = ABS(x - fx) / 64;
				i32 vy = ABS(y - fy) / 64;
				i32 vz = ABS(z - fz) / 64;

				fv += ((i32)(vx+vy+vz)-fv)/256;

				t = fv/1024;

				vibration = LIM(t, 0, 0xFFFF);

				i--;
			};

			break;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateTemp()
{
	static byte i = 0;

	static DSCI2C dsc, dsc2;

//	static byte reg = 0;
	static u16 rbuf = 0;
	static byte buf[10];

	static TM32 tm;

	switch (i)
	{
		case 0:

			if (tm.Check(100))
			{
#ifndef WIN32
				if (!__debug) { HW::WDT->Update(); };
#endif

				buf[0] = 0x11;

				dsc.adr = 0x68;
				dsc.wdata = buf;
				dsc.wlen = 1;
				dsc.rdata = &rbuf;
				dsc.rlen = 2;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				if (I2C_AddRequest(&dsc))
				{
					i++;
				};
			};

			break;

		case 1:

			if (dsc.ready)
			{
				if (dsc.ack && dsc.readedLen == dsc.rlen)
				{
					i32 t = (i16)ReverseWord(rbuf);
					
					t = (t * 10 + 128) / 256;

					if (t < (-600))
					{
						t += 2560;
					};

					tempClock = t;
				};
				//else
				//{
				//	tempClock = -2730;
				//};

				i++;
			};

			break;

		case 2:

			buf[0] = 0x0E;
			buf[1] = 0x20;
			buf[2] = 0xC8;

			dsc2.adr = 0x68;
			dsc2.wdata = buf;
			dsc2.wlen = 3;
			dsc2.rdata = 0;
			dsc2.rlen = 0;
			dsc2.wdata2 = 0;
			dsc2.wlen2 = 0;

			if (I2C_AddRequest(&dsc2))
			{
				i++;
			};

			break;

		case 3:

			if (dsc2.ready)
			{
				buf[0] = 0;

				dsc.adr = 0x49;
				dsc.wdata = buf;
				dsc.wlen = 1;
				dsc.rdata = &rbuf;
				dsc.rlen = 2;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				if (I2C_AddRequest(&dsc))
				{
					i++;
				};
			};

			break;

		case 4:

			if (dsc.ready)
			{
				if (dsc.ack && dsc.readedLen == dsc.rlen)
				{
					i32 t = (i16)ReverseWord(rbuf);

					temp = (t * 10 + 64) / 128;
				};
				//else
				//{
				//	temp = -2730;
				//};

#ifdef CPU_SAME53	

				i = 0;
			};

			break;

#elif defined(CPU_XMC48)

				HW::SCU_GENERAL->DTSCON = SCU_GENERAL_DTSCON_START_Msk;
				
				i++;
			};

			break;

		case 5:

			if (HW::SCU_GENERAL->DTSSTAT & SCU_GENERAL_DTSSTAT_RDY_Msk)
			{
				cpu_temp = ((i32)(HW::SCU_GENERAL->DTSSTAT & SCU_GENERAL_DTSSTAT_RESULT_Msk) - 605) * 1000 / 205;

				i = 0;
			};

			break;

#elif defined(WIN32)

				i = 0;
			};

			break;
#endif
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateI2C()
{
	I2C_Update();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateSPI()
{
	SPI_Update();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

Ptr<MB> CreateTestRspRcv02()
{
	Ptr<MB> rq;
	
	rq = NandFlash_AllocWB(sizeof(RspRcv02));

	if (!rq.Valid()) { return rq; };

	RspRcv02 &rsp = *((RspRcv02*)(rq->GetDataPtr()));

	rsp.hdr.rw			= (manReqWord|0x30)+((fireCounter.d&3)<<4);
	rsp.hdr.cnt			= fireCounter.d++;
	rsp.hdr.preAmp		= 0;
	rsp.hdr.gain		= 0;
	rsp.hdr.st			= 10;
	rsp.hdr.sl			= RCV_SAMPLE_LEN;
	rsp.hdr.sd			= 0;
	rsp.hdr.packType	= 0;
	rsp.hdr.math		= 0;
	rsp.hdr.packLen1	= RCV_SAMPLE_LEN;
	rsp.hdr.packLen2	= RCV_SAMPLE_LEN;
	rsp.hdr.packLen3	= RCV_SAMPLE_LEN;
	rsp.hdr.packLen4	= RCV_SAMPLE_LEN;

	rq->len = sizeof(rsp.hdr) + RCV_SAMPLE_LEN*8;
	
	return rq;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateTestFlashWrite()
{
	static Ptr<MB> ptr;
	static u32 count = 0;

	//static CTM32 rtm;

	if (/*rtm.Check(MS2CTM(1)) &&*/ NandFlash_Status() != 0 && mv.disableFireNoVibration != 0 && mv.levelNoVibration == 0xFFFF)
	{
		ptr = CreateTestRspRcv02();

		if (ptr.Valid())
		{
			count--;

			RspRcv02 *rsp = (RspRcv02*)(ptr->GetDataPtr());
			NandFlash_RequestWrite(ptr, rsp->hdr.rw, true);

		};
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateBootFlashMisc()
{
	qRcv.Update();
	qTrm.Update();
	UpdateEMAC();
	UpdateTraps();

#ifndef WIN32
	if (!__debug) { HW::WDT->Update(); };
#endif

}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifndef WIN32

static const u32 rcvFlashPages[] = {
#include "G26X.1.DSP.LDR.H"
};

//u16 dspFlashLen = 0;
//u16 dspFlashCRC = 0;

//static const u32 dspBootFlashPages[] = {
//#include "G26K_2_BOOT_BF592.LDR.H"
//};

//u16 dspBootFlashLen = 0;
//u16 dspBootFlashCRC = 0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void FlashRcv()
{
	Ptr<REQ> rq;

	u32 flashLen = sizeof(rcvFlashPages);
	u16 flashCRC = GetCRC16(rcvFlashPages, flashLen);

	BootRspV1::SF0	rspF0[RCV_MAX_NUM_STATIONS];

	u16 tryCount = 4;
	
	rcvStatus = 0;
	byte N = RCV_MAX_NUM_STATIONS;

	CTM32 tm;

	//comRcv.Disconnect();

	//comRcv.Connect(ComPort::ASYNC, RCV_BOOT_COM_BAUDRATE, RCV_BOOT_COM_PARITY, 1);

	while (tryCount > 0)
	{
		for (byte i = 1; i <= N; i++)
		{
			if ((rcvStatus & (1 << (i-1))) == 0)
			{
				rq = CreateRcvBootReq00(i, &rspF0[i-1], 2);

				if (rq.Valid())
				{
					qRcv.Add(rq); while(!rq->ready) UpdateBootFlashMisc();

					if (rq->crcOK)
					{
						BootReqV1::SF0 &req = *((BootReqV1::SF0*)rq->wb.data);
						BootRspV1::SF0 &rsp = rspF0[i-1];

						if (rsp.adr == i && rsp.rw == req.rw)
						{
							rcvStatus |= 1 << (i-1);
						};

						if (tryCount > 2) tryCount = 2;
					};

					rq.Free();
				};
			};
		};
		
		tryCount--;
	};

	for (byte i = 1; i <= N; i++)
	{
		if (rcvStatus & (1 << (i-1)))
		{
			rq = CreateRcvBootReq01(i, flashLen, 2);

			qRcv.Add(rq); while(!rq->ready) UpdateBootFlashMisc();

			BootRspV1::SF1 &rsp = *((BootRspV1::SF1*)rq->rb.data);

			if (rsp.flashCRC != flashCRC || rsp.flashLen != flashLen)
			{
				u16 count = flashLen+2;
				u16 adr = 0;
				byte *p = (byte*)rcvFlashPages;

				u16 max = rspF0[i-1].pageLen;

				u16 len;

				while (count > 0)
				{
					if (count > max)
					{
						len = max;

						rq = CreateRcvBootReq02(i, adr, len, p, 0, 0, max, 2);
					}
					else
					{
						len = count;

						if (len > 2)
						{
							rq = CreateRcvBootReq02(i, adr, len-2, p, sizeof(flashCRC), &flashCRC, max, 2);
						}
						else
						{
							rq = CreateRcvBootReq02(i, adr, sizeof(flashCRC), &flashCRC, 0, 0, max, 2);
						};
					};

					qRcv.Add(rq); while(!rq->ready) UpdateBootFlashMisc();

					count -= len;
					p += len;
					adr += len;
				};
			};

			tm.Reset();	while (!tm.Check(MS2CTM(1)));

			//rq = CreateRcvBootReq03(i, 2);

			//qRcv.Add(rq); while(!rq->ready) qRcv.Update();

			//rq.Free();
		};
	};

	tm.Reset(); while (!tm.Check(MS2CTM(1)));

	rq = CreateRcvBootReq03(0, 2);

	qRcv.Add(rq); while(!rq->ready) UpdateBootFlashMisc();

	rq.Free();

	tm.Reset();	while (!tm.Check(MS2CTM(10)));
	
	//comRcv.Disconnect();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static const u32 trmFlashPages[] = {
#include "G26X.3.TRM.BIN.H"
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void FlashTrm()
{
	CTM32 tm;

	Ptr<REQ> rq;

	u16 flashLen = sizeof(trmFlashPages);
	u16 flashCRC = GetCRC16(trmFlashPages, flashLen);

	BootRspV1::SF0	rspF0;

	u16 tryCount = 4;

	bool hs = false;

	while (tryCount > 0)
	{
		rq = CreateTrmBootReq00(TRM_BOOT_NET_ADR, &rspF0, 2);

		if (rq.Valid())
		{
			qTrm.Add(rq); while(!rq->ready) UpdateBootFlashMisc();

			if (rq->crcOK)
			{
				BootReqV1::SF0 &req = *((BootReqV1::SF0*)rq->wb.data);
				BootRspV1::SF0 &rsp = rspF0;

				if (rsp.adr == TRM_BOOT_NET_ADR && rsp.rw == req.rw)
				{
					hs = true;
					break;
				};

				if (tryCount > 2) tryCount = 2;
			};

			rq.Free();
		};
		
		tryCount--;
	};

	if (hs)
	{
		rq = CreateTrmBootReq01(TRM_BOOT_NET_ADR, flashLen, 2);

		qTrm.Add(rq); while(!rq->ready) UpdateBootFlashMisc();

		BootRspV1::SF1 &rsp = *((BootRspV1::SF1*)rq->rb.data);

		if (rsp.flashCRC != flashCRC || rsp.flashLen != flashLen)
		{
			u16 count = flashLen;
			u16 adr = 0;
			byte *p = (byte*)trmFlashPages;

			u16 max = rspF0.pageLen;

			u16 len;

			while (count > 0)
			{
				len = (count > max) ? max : count;

				rq = CreateTrmBootReq02(TRM_BOOT_NET_ADR, adr, len, p, max, 2);

				qTrm.Add(rq); while(!rq->ready) UpdateBootFlashMisc();

				count -= len;
				p += len;
				adr += len;
			};
		};

		tm.Reset();	while (!tm.Check(MS2CTM(1)));
	};

	tm.Reset(); while (!tm.Check(MS2CTM(1)));

	rq = CreateTrmBootReq03(0, 2);

	qTrm.Add(rq); while(!rq->ready) UpdateBootFlashMisc();

	rq.Free();

	tm.Reset();	while (!tm.Check(MS2CTM(10)));
}

#endif
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void ReadNumDevRcvTrm()
{
	Ptr<REQ> rq;

	u16 tryCount = 6;
	
	rcvStatus = 0;
	
	bool trmStatus = false;

	while (tryCount > 0)
	{
		for (byte i = 1; i <= RCV_MAX_NUM_STATIONS; i++)
		{
			if ((rcvStatus & (1 << (i-1))) == 0)
			{
				rq = CreateRcvReq03(i, 2);

				if (rq.Valid())
				{
					qRcv.Add(rq); while(!rq->ready) UpdateBootFlashMisc();

					if (rq->crcOK) rcvStatus |= 1 << (i-1);

					rq.Free();
				};
			};
		};

		if (!trmStatus)
		{
			rq = CreateTrmReq02(2);

			if (rq.Valid())
			{
				qTrm.Add(rq); while(!rq->ready) UpdateBootFlashMisc();

				if (rq->crcOK) trmStatus = true;

				rq.Free();
			};
		};

		if (trmStatus && rcvStatus && tryCount > 2) tryCount = 2;

		tryCount--;
	};

	u16 nums[RCV_MAX_NUM_STATIONS] = {0};
	u16 vers[RCV_MAX_NUM_STATIONS] = {0};
	u16 count[RCV_MAX_NUM_STATIONS] = {0};

	byte im = 0;

	for (byte i = 0; i < RCV_MAX_NUM_STATIONS; i++)
	{
		if (arrRcvNumDevValid[i])
		{
			bool c = false;

			for (byte j = 0; j < im; j++)
			{
				if (nums[j] == arrRcvNumDev[i]) { count[j]++; c = true; break; };
			};

			if (!c)
			{
				nums[im] = arrRcvNumDev[i];
				vers[im] = arrRcvVerDev[i];
				count[im++]++;
			};
		};
	};

	byte imax = 0;
	u16 max = count[0];

	for (byte i = 1; i < im; i++)
	{
		if (count[i] > max) max = count[i], imax = i;
	};

	numDevRcv = nums[imax];
	verDevRcv = vers[imax];
	numDevRcvValid = (max != 0);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void FlashInitBoot()
{
	Ptr<REQ> rq;		
						
	u16 tryCount = 4;
	
	rcvStatus = 0;
	byte N = RCV_MAX_NUM_STATIONS;

	CTM32 tm;

	BootRspV1::SF0	rspF0;

	bool hs = false;

	while (tryCount > 0)
	{
	#ifndef RCV_8AD

		for (byte i = 1; i <= N; i++)
		{
			if ((rcvStatus & (1 << (i-1))) == 0)
			{
				rq = CreateRcvBootReq00(i, &rspF0, 2);

				if (rq.Valid())
				{
					qRcv.Add(rq); while(!rq->ready) UpdateBootFlashMisc();

					if (rq->crcOK)
					{
						BootReqV1::SF0 &req = *((BootReqV1::SF0*)rq->wb.data);
						BootRspV1::SF0 &rsp = rspF0;

						if (rsp.adr == i && rsp.rw == req.rw)
						{
							rcvStatus |= 1 << (i-1);
						};

						if (tryCount > 2) tryCount = 2;
					};

					rq.Free();
				};
			};
		};
	#endif

		if (!hs)
		{
			rq = CreateTrmBootReq04(TRM_BOOT_NET_ADR, 100000, 2);

			if (rq.Valid())
			{
				qTrm.Add(rq); while(!rq->ready) UpdateBootFlashMisc();

				if (rq->crcOK)
				{
					BootReqV1::SF4 &req = *((BootReqV1::SF4*)rq->wb.data);
					BootRspV1::SF4 &rsp = *((BootRspV1::SF4*)rq->rb.data);

					if (rsp.adr == TRM_BOOT_NET_ADR && rsp.rw == req.rw) hs = true;
				};

				rq.Free();
			};
		};

		if (tryCount > 2 && rcvStatus != 0 && hs) tryCount = 2;
		
		tryCount--;
	};

}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitMainVars()
{
	mv.numDevice			= 11111;
	mv.numMemDevice			= 11111;

	mv.trans[0].gain		= 128;
	mv.trans[0].st 			= 5;	
	mv.trans[0].sl 			= 512;	
	mv.trans[0].sd 			= 200;	
	mv.trans[0].freq		= 16000;
	mv.trans[0].duty		= 50;
	mv.trans[0].amp			= 2100;
	mv.trans[0].pulseCount	= 1;
	mv.trans[0].packType	= 7;	// PACK_DCT3
	mv.trans[0].math		= 0;

	mv.trans[1].gain		= 128;
	mv.trans[1].st 			= 10;	
	mv.trans[1].sl 			= 512;	
	mv.trans[1].sd 			= 200;	
	mv.trans[1].freq		= 1000;
	mv.trans[1].duty		= 50;
	mv.trans[1].amp			= 2100;
	mv.trans[1].pulseCount	= 1;
	mv.trans[1].packType	= 7;	// PACK_DCT3
	mv.trans[1].math		= 0;

	mv.trans[2].gain		= 128;
	mv.trans[2].st 			= 10;	
	mv.trans[2].sl 			= 512;	
	mv.trans[2].sd 			= 500;	
	mv.trans[2].freq		= 3000;
	mv.trans[2].duty		= 5000;
	mv.trans[2].amp			= 1200;
	mv.trans[2].pulseCount	= 2;
	mv.trans[2].packType	= 7;	// PACK_DCT3
	mv.trans[2].math		= 0;

	mv.trmVoltage				= 600;
	mv.disableFireNoVibration	= 0;
	mv.levelNoVibration			= 100;
	mv.firePeriod				= 1000;
	mv.lfMnplEnabled			= 0;

#ifdef RCV_AUTO_GAIN
	mv.autoGain					= 0;
#endif

	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_CYAN "Init Main Vars Vars ... OK\n");
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void LoadVars()
{
	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_WHITE "Load Vars ... ");

	static DSCI2C dsc;
	static DSCSPI spi;
	static u16 romAdr = 0;
	
	byte buf[sizeof(mv)*2+4];

	MainVars mv1, mv2;

	bool c1 = false, c2 = false;

	bool loadVarsOk = false;

	spi.adr = (ReverseDword(FRAM_SPI_MAINVARS_ADR) & ~0xFF) | 3;
	spi.alen = 4;
	spi.csnum = 1;
	spi.wdata = 0;
	spi.wlen = 0;
	spi.rdata = buf;
	spi.rlen = sizeof(buf);

	if (SPI_AddRequest(&spi))
	{
		while (!spi.ready) { SPI_Update(); };
	};

	PointerCRC p(buf);

	for (byte i = 0; i < 2; i++)
	{
		p.CRC.w = 0xFFFF;
		p.ReadArrayB(&mv1, sizeof(mv1));
		p.ReadW();

		if (p.CRC.w == 0) { c1 = true; break; };
	};

	romAdr = ReverseWord(FRAM_I2C_MAINVARS_ADR);

	dsc.wdata = &romAdr;
	dsc.wlen = sizeof(romAdr);
	dsc.wdata2 = 0;
	dsc.wlen2 = 0;
	dsc.rdata = buf;
	dsc.rlen = sizeof(buf);
	dsc.adr = 0x50;


	if (I2C_AddRequest(&dsc))
	{
		while (!dsc.ready) { I2C_Update(); };
	};

//	bool c = false;

	p.b = buf;

	for (byte i = 0; i < 2; i++)
	{
		p.CRC.w = 0xFFFF;
		p.ReadArrayB(&mv2, sizeof(mv2));
		p.ReadW();

		if (p.CRC.w == 0) { c2 = true; break; };
	};

	SEGGER_RTT_WriteString(0, "FRAM SPI - "); SEGGER_RTT_WriteString(0, (c1) ? (RTT_CTRL_TEXT_BRIGHT_GREEN "OK") : (RTT_CTRL_TEXT_BRIGHT_RED "ERROR"));

	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_WHITE " ... FRAM I2C - "); SEGGER_RTT_WriteString(0, (c2) ? (RTT_CTRL_TEXT_BRIGHT_GREEN "OK\n") : (RTT_CTRL_TEXT_BRIGHT_RED "ERROR\n"));

	if (c1 && c2)
	{
		if (mv1.timeStamp > mv2.timeStamp)
		{
			c2 = false;
		}
		else if (mv1.timeStamp < mv2.timeStamp)
		{
			c1 = false;
		};
	};

	if (c1)	{ mv = mv1; } else if (c2) { mv = mv2; };

	loadVarsOk = c1 || c2;

	if (!c1 || !c2)
	{
		if (!loadVarsOk) InitMainVars();

		svCount = 2;
	};

#ifdef RCV_AUTO_GAIN

	for (u16 i = 0; i < RCV_FIRE_NUM; i++)
	{
		autoGain[transIndex[i]].gain = mv.trans[transIndex[i]].gain;
	};

#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void SaveVars()
{
	static DSCI2C dsc;
	static DSCSPI spi,spi2;
	static u16 romAdr = 0;
	static byte buf[sizeof(mv) * 2 + 8];

	static byte i = 0;
	static TM32 tm;

	PointerCRC p(buf);

	switch (i)
	{
		case 0:

			if (svCount > 0)
			{
				svCount--;
				i++;
			};

			break;

		case 1:

			mv.timeStamp = GetMilliseconds();

			for (byte j = 0; j < 2; j++)
			{
				p.CRC.w = 0xFFFF;
				p.WriteArrayB(&mv, sizeof(mv));
				p.WriteW(p.CRC.w);
			};

			spi.adr = (ReverseDword(FRAM_SPI_MAINVARS_ADR) & ~0xFF) | 2;
			spi.alen = 4;
			spi.csnum = 1;
			spi.wdata = buf;
			spi.wlen = p.b-buf;
			spi.rdata = 0;
			spi.rlen = 0;

			romAdr = ReverseWord(FRAM_I2C_MAINVARS_ADR);

			dsc.wdata = &romAdr;
			dsc.wlen = sizeof(romAdr);
			dsc.wdata2 = buf;
			dsc.wlen2 = p.b-buf;
			dsc.rdata = 0;
			dsc.rlen = 0;
			dsc.adr = 0x50;

			spi2.adr = 6;
			spi2.alen = 1;
			spi2.csnum = 1;
			spi2.wdata = 0;
			spi2.wlen = 0;
			spi2.rdata = 0;
			spi2.rlen = 0;

			tm.Reset();

			SPI_AddRequest(&spi2);

			i++;

			break;

		case 2:

			if (spi2.ready || tm.Check(200))
			{
				SPI_AddRequest(&spi);

				i++;
			};

			break;

		case 3:

			if (spi.ready || tm.Check(200))
			{
				I2C_AddRequest(&dsc);
				
				i++;
			};

			break;

		case 4:

			if (dsc.ready || tm.Check(100))
			{
				i = 0;
			};

			break;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static TaskList taskList;

static void InitTaskList()
{
	static Task tsk[] =
	{
		Task(MainMode,				US2CTM(100)	),
		Task(UpdateTemp,			MS2CTM(1)	),
		Task(UpdateMan,				US2CTM(100)	),
		Task(NandFlash_Update,		MS2CTM(1)	),
		Task(UpdateAccel,			MS2CTM(1)	),
		Task(UpdateI2C,				US2CTM(20)	),
		Task(SaveVars,				MS2CTM(1)	),
		Task(UpdateEMAC,			MS2CTM(1)	),
		Task(UpdateRcvTrm,			US2CTM(1)	),
		Task(UpdateSPI,				US2CTM(20)	),
		Task(UpdateTestFlashWrite,	MS2CTM(1)	)
	};

	for (u16 i = 0; i < ArraySize(tsk); i++) taskList.Add(tsk+i);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateParams()
{
	static byte i = 0;

	#define CALL(p) case (__LINE__-S): p; break;

	enum C { S = (__LINE__+3) };
	switch(i++)
	{
		CALL( MainMode()				);
		CALL( UpdateTemp()				);
		CALL( UpdateMan(); 				);
		CALL( NandFlash_Update();		);
		CALL( UpdateHardware();			);
		CALL( UpdateAccel();			);
		CALL( UpdateI2C();				);
		CALL( SaveVars();				);
	};

	i = (i > (__LINE__-S-3)) ? 0 : i;

	#undef CALL
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateMisc()
{
	static byte i = 0;

	#define CALL(p) case (__LINE__-S): p; break;

	enum C { S = (__LINE__+3) };
	switch(i++)
	{
		CALL( I2C_Update();			); 
		CALL( SPI_Update();			);
		CALL( NandFlash_Update();	);
	};

	i = (i > (__LINE__-S-3)) ? 0 : i;

	#undef CALL
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Update()
{
	NAND_Idle();	

	if (EmacIsConnected())
	{
		UpdateEMAC();
		UpdateTraps();

#ifndef WIN32
		if (!__debug) { HW::WDT->Update(); };
#endif
	};
	
	if (!(IsComputerFind() && EmacIsConnected()))
	{
		taskList.Update(); //UpdateMisc();
	}
	else
	{
		UpdateMisc();
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef __TEST__

static void Test_Ptr_MB()
{
	const u32 buflen = 1UL<<4;
	const u32 bufmask = buflen-1;

	Ptr<MB> buf[16];

	Ptr<MB> mb = AllocSmallBuffer();

	if (!mb.Valid()) __breakpoint(0);

	u32 count = mb.Count();

	u32 freeCount = mb->FreeCount();

	for (u32 i = 0; i < buflen; i++)
	{
		if (buf[i].Valid()) __breakpoint(0);
	};

	Ptr<MB> ptr;// = AllocSmallBuffer();

	for (u32 i = 0, n = 0; i < 100000; i++)
	{
		n = n * 7727 + (n>>16) + 1;

		if (((n>>8)&0xF) == 0)
		{
			ptr = AllocSmallBuffer();
		}
		else
		{
			buf[n&bufmask] = ptr;
		};
	};

	for (u32 i = 0; i < buflen; i++)
	{
		buf[i].Free();

		if (buf[i].Valid()) __breakpoint(0);
	};

	ptr.Free();

	if (ptr.Valid()) __breakpoint(0);

	u32 freeCount2 = mb->FreeCount();

	if (freeCount2 != freeCount) __breakpoint(0);

	u32 count2 = mb.Count();

	if (count2 != count) __breakpoint(0);

	mb.Free();

	if (mb.Valid()) __breakpoint(0);
}

#endif // __TEST__

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef __TEST__

static void Test_ListPtr()
{
	static ListPtr<MB> list;

	for (u32 i = 0; i < 100; i++)
	{
		Ptr<MB> mb = AllocSmallBuffer();

		list.Add(mb);
	};

	for (u32 i = 0; i < 100; i++)
	{
		Ptr<MB> mb = list.Get();
	};

	if(!list.Empty()) __breakpoint(0);
}

#endif // __TEST__

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef __TEST__

static void Test_ListRef()
{
	static ListRef<MB> list;

	for (u32 i = 0; i < 100; i++)
	{
		Ptr<MB> mb = AllocSmallBuffer();

		list.Add(mb);
	};

	for (u32 i = 0; i < 100; i++)
	{
		Ptr<MB> mb = list.Get();
	};

	if(!list.Empty()) __breakpoint(0);
}

#endif // __TEST__

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#if defined(__TEST__) && defined(WIN32) 

static ListPtr<MB> mt_listptr;

DWORD WINAPI Thread_Test_ListPtr(LPVOID lpParam) 
{
	Ptr<MB> mb;

	while(1)
	{
		mb = mt_listptr.Get();
	};
}

static void Test_MT_ListPtr()
{
	HANDLE ht = CreateThread(0, 0, Thread_Test_ListPtr, 0, 0, 0);

	DEBUG_ASSERT(ht != INVALID_HANDLE_VALUE);

	Ptr<MB> mb;

	//mb = AllocSmallBuffer(); mt_listptr.Add(mb);
	//mb = AllocSmallBuffer(); mt_listptr.Add(mb);
	//mb = AllocSmallBuffer(); mt_listptr.Add(mb);
	//mb = AllocSmallBuffer(); mt_listptr.Add(mb);

	u32 t = GetMilliseconds();

	while ((GetMilliseconds() - t) < 2000) 
	{
		mb = AllocSmallBuffer();

		mt_listptr.Add(mb);
	};

	t = GetMilliseconds();

	while (!mt_listptr.Empty() && (GetMilliseconds() - t) < 1000) 
	{
		Sleep(1);
	};

	if(!mt_listptr.Empty()) __breakpoint(0);

	DEBUG_ASSERT(TerminateThread(ht, 0) != 0);
}

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#if defined(__TEST__) && defined(WIN32) 

static ListRef<MB> mt_listref;

DWORD WINAPI Thread_Test_ListRef(LPVOID lpParam) 
{
	Ptr<MB> mb;

	while(1)
	{
		mb = mt_listref.Get();
	};
}

static void Test_MT_ListRef()
{
	HANDLE ht = CreateThread(0, 0, Thread_Test_ListRef, 0, 0, 0);

	DEBUG_ASSERT(ht != INVALID_HANDLE_VALUE);

	Ptr<MB> mb;

	//mb = AllocSmallBuffer(); mt_listref.Add(mb);
	//mb = AllocSmallBuffer(); mt_listref.Add(mb);
	//mb = AllocSmallBuffer(); mt_listref.Add(mb);
	//mb = AllocSmallBuffer(); mt_listref.Add(mb);

	u32 t = GetMilliseconds();

	while ((GetMilliseconds() - t) < 2000) 
	{
		mb = AllocSmallBuffer();

		mt_listref.Add(mb);
	};

	t = GetMilliseconds();

	while (!mt_listref.Empty() && (GetMilliseconds() - t) < 1000) 
	{
		Sleep(1);
	};

	if(!mt_listref.Empty()) __breakpoint(0);

	DEBUG_ASSERT(TerminateThread(ht, 0) != 0);
}

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int main()
{
	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_WHITE "main() start ...\n");

	#ifdef __TEST__

		Test_Ptr_MB();
		Test_ListPtr();
		Test_ListRef(); 

		#ifdef WIN32
			Test_MT_ListPtr();
			Test_MT_ListRef();
		#endif

	#endif

	TM32 tm;

	InitHardware();

	//__breakpoint(0);

	//static u16 crc1, crc2, crc3;
	//crc1 = GetCRC16_CCIT_refl(0, 16384);
	//crc2 = CRC_CCITT_DMA(0, 16384);
	//CRC_CCITT_PIO(0, 16384, &crc3);

	LoadVars();

	InitEMAC();

	NandFlash_Init();
 
#ifndef WIN32

	comTrm.Connect(ComPort::ASYNC, TRM_COM_BAUDRATE,		TRM_COM_PARITY,			TRM_COM_STOPBITS);
	comRcv.Connect(ComPort::ASYNC, RCV_BOOT_COM_BAUDRATE,	RCV_BOOT_COM_PARITY,	2);

	//__breakpoint(0);

	#ifndef RCV_8AD
	
		FlashInitBoot();

		FlashRcv();

		comRcv.Disconnect();
		comRcv.Connect(ComPort::ASYNC, RCV_COM_BAUDRATE,		RCV_COM_PARITY,			2);
	
		FlashTrm();

		ReadNumDevRcvTrm();

	#endif

	InitTaskList();

#endif

	u32 fc = 0;

	while (1)
	{
		Pin_MainLoop_Set();

		Update();

		Pin_MainLoop_Clr();

		fc++;

		if (tm.Check(1000))
		{ 
			fps = fc; fc = 0; 

			//if ((fps & 3) == 0) SEGGER_RTT_printf(0, RTT_CTRL_TEXT_WHITE "%u\n", fps);


#ifdef WIN32

			extern u32 txThreadCount;

			Printf(0, 0, 0xFC, "FPS=%9i", fps);
//			Printf(0, 1, 0xF0, "%lu", testDspReqCount);
			Printf(0, 2, 0xF0, "%lu", txThreadCount);
#endif
		};

#ifdef WIN32

		UpdateDisplay();

		static TM32 tm2;

		byte key = 0;

		if (tm2.Check(50))
		{
			if (_kbhit())
			{
				key = _getch();

				if (key == 27) break;
			};

			if (key == 'w')
			{
				NandFlash_WriteEnable();
			}
			else if (key == 'e')
			{
				NandFlash_WriteDisable();
			}
			else if (key == 'p')
			{
				NandFlash_FullErase();
			};
		};

		Sleep(0);

#endif

	}; // while (1)

#ifdef WIN32

	NAND_FlushBlockBuffers();

	I2C_Destroy();
	SPI_Destroy();

	//_fcloseall();

#endif

}
