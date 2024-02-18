#include "hardware.h"
//#include "options.h"
//#include "hw_emac.h"
#include "EMAC\xtrap.h"
#include "FLASH\flash.h"
#include "CRC\CRC16.h"
#include "CRC\CRC16_CCIT.h"
#include "BOOT\boot_req.h"
#include "req.h"
#include "list.h"
#include "PointerCRC.h"
#include "SEGGER_RTT\SEGGER_RTT.h"
#include "hw_com.h"

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

enum { VERSION = 0x105 };

//#pragma O3
//#pragma Otime

#ifndef _DEBUG
	static const bool __debug = false;
#else
	static const bool __debug = true;
#endif

//#define SENS_NUM	3
//#define NS2DSP(v) (((v)+10)/20)
//#define US2DSP(v) ((((v)*1000)+10)/20)

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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
	u16 packType;
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static MainVars mv;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool runMainMode = true;
static bool startFire = false;
static u32 fireCounter = 0;
static u32 manCounter = 0;
static byte numStations = RCV_MAX_NUM_STATIONS;
static u16 resistValue = 0;
static u16 voltage = 0;
static byte transIndex[RCV_FIRE_NUM] = {0, 1, 2, 2};

static u16  numDevTrm = 0;
static byte numDevTrmValid = 0;
static u16  verDevTrm = 0;
static byte verDevTrmValid = 0;

static u16  numDevRcv = 0;
static byte numDevRcvValid = 0;
static u16  verDevRcv = 0;
static byte verDevRcvValid = 0;

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

static Ptr<MB> manVec30[3];
static Ptr<MB> curManVec30;
static Ptr<MB> manVec60;
static Ptr<MB> curManVec60;

//static ListPtr<MB> readyR01;

static RspMan60 rspMan60;
static byte curRcv[3] = { 0 };



//static u16 motoEnable = 0;		// двигатель включить или выключить
//static u16 motoTargetRPS = 0;		// заданные обороты двигателя
//static u16 motoRPS = 0;				// обороты двигателя, об/сек
//static u16 motoCur = 0;				// ток двигателя, 0...9400 мА
//static u16 motoCurLow = 0;			// ток двигателя, 0...4500 мА
//static u16 motoStat = 0;			// статус двигателя
//static u16 motoCounter = 0;			// счётчик оборотов двигателя 1/6 оборота
//static u16 cmSPR = 32;			// Количество волновых картин на оборот головки в режиме цементомера
//static u16 imSPR = 100;			// Количество точек на оборот головки в режиме имиджера
//static u16 *curSPR = &cmSPR;		// Количество импульсов излучателя на оборот в текущем режиме
//static u16 auxVoltage = 0;
//static u16 motoVoltage = 90;
//static u16 motoRcvCount = 0;

//static u16 curFireVoltage = 500;

//static u32 dspMMSEC = 0;
//static u32 shaftMMSEC = 0;

//const u16 dspReqWord = 0xAD00;
//const u16 dspReqMask = 0xFF00;

static u16 manReqWord = RCV_MAN_REQ_WORD;
static u16 manReqMask = RCV_MAN_REQ_MASK;

static u16 memReqWord = 0x3D00;
static u16 memReqMask = 0xFF00;

//static u16 numDevice = 0;
static u16 verDevice = VERSION;

//static u16 numMemDevice = 0;
static u16 verMemDevice = 0x100;

//static u32 manCounter = 0;
//static u32 fireCounter = 0;

static byte mainModeState = 0;
static byte fireType = 0;
static u16	fireMask = 13;//(1UL<<RCV_FIRE_NUM)-1;
static byte nextFireType = 1;
//static byte dspStatus = 0;

static bool cmdWriteStart_00 = false;
static bool cmdWriteStart_10 = false;
static bool cmdWriteStart_20 = false;
static bool cmdRcvSaveParams = false;

//static u32 dspRcv40 = 0;
//static u32 dspRcv50 = 0;
//static u16 dspRcvCount = 0;
//static u32 dspRcvErr = 0;
//static u16 dspNotRcv = 0;

static u16 rcvStatus = 0;
static u32 crcErr02[13] = {0};
static u32 crcErr03 = 0;
static u32 crcErr04 = 0;
static u32 rcv02rejVec = 0;

static u32 notRcv02[13] = {0};
static u32 lenErr02[13] = {0};

//static u32 rcvCRCER = 0;

//static u32 chnlCount[4] = {0};

//static u32 crcErr02 = 0;
//static u32 crcErr03 = 0;
static u32 crcErr06 = 0;
static u32 wrtErr06 = 0;

//static u32 notRcv02 = 0;
//static u32 lenErr02 = 0;

static i16 ax = 0, ay = 0, az = 0, at = 0;
static u16 vibration;

i16 temperature = 0;
i16 cpuTemp = 0;
i16 temp = 0;

static byte svCount = 0;


//struct AmpTimeMinMax { bool valid; u16 ampMax, ampMin, timeMax, timeMin; };
//static AmpTimeMinMax sensMinMax[SENS_NUM]		= { {0, 0, ~0, 0, ~0}, {0, 0, ~0, 0, ~0}, {0, 0, ~0, 0, ~0} };
//static AmpTimeMinMax sensMinMaxTemp[SENS_NUM]	= { {0, 0, ~0, 0, ~0}, {0, 0, ~0, 0, ~0}, {0, 0, ~0, 0, ~0} };

//static u32 testDspReqCount = 0;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void SaveMainParams()
{
	svCount = 1;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static void Response_0(u16 rw, MTB &mtb)
//{
//	__packed struct Rsp {u16 rw; u16 device; u16 session; u32 rcvVec; u32 rejVec; u32 wrVec; u32 errVec; u16 wrAdr[3]; u16 numDevice; u16 version; u16 temp; byte status; byte flags; RTC rtc; };
//
//	Rsp &rsp = *((Rsp*)&txbuf);
//
//	rsp.rw = rw;
//	rsp.device = GetDeviceID();  
//	rsp.session = FLASH_Session_Get();	  
//	rsp.rcvVec =  FLASH_Vectors_Recieved_Get();
//	rsp.rejVec = FLASH_Vectors_Rejected_Get();
//	rsp.wrVec = FLASH_Vectors_Saved_Get();
//	rsp.errVec = FLASH_Vectors_Errors_Get();
//	*((__packed u64*)rsp.wrAdr) = FLASH_Current_Adress_Get();
//	rsp.temp = temp*5/2;
//	rsp.status = FLASH_Status();
//
//	GetTime(&rsp.rtc);
//
//	mtb.data1 = txbuf;
//	mtb.len1 = sizeof(rsp)/2;
//	mtb.data2 = 0;
//	mtb.len2 = 0;
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateRcvReqFire(byte n, byte next_n, u16 fc)
{
	Ptr<REQ> rq(AllocREQ());
	
	//rq.Alloc();

	if (!rq.Valid()) return rq;

	ReqRcv01 &req = *((ReqRcv01*)rq->reqData);

	REQ &q = *rq;

	q.CallBack = 0;
	q.ready = false;
	q.tryCount = 0;
	q.checkCRC = false;
	q.updateCRC = false;
	
	q.wb.data = &req;
	q.wb.len = sizeof(req);

	q.rb.data = 0;
	q.rb.maxLen = 0;

	Transmiter	&trans = mv.trans[transIndex[n]];

	req.r[2].len		= req.r[1].len			= req.r[0].len			= sizeof(req.r[0]) - 1;
	req.r[2].adr		= req.r[1].adr			= req.r[0].adr			= 0;
	req.r[2].func		= req.r[1].func			= req.r[0].func			= 1;
	req.r[2].n			= req.r[1].n			= req.r[0].n			= n;
	req.r[2].next_n		= req.r[1].next_n		= req.r[0].next_n		= next_n;
	req.r[2].next_gain	= req.r[1].next_gain	= req.r[0].next_gain	= mv.trans[transIndex[next_n]].gain;
	req.r[2].st 		= req.r[1].st 			= req.r[0].st 			= trans.st;
	req.r[2].sl 		= req.r[1].sl 			= req.r[0].sl 			= trans.sl;
	req.r[2].sd 		= req.r[1].sd 			= req.r[0].sd 			= trans.sd;
	req.r[2].fc			= req.r[1].fc			= req.r[0].fc			= fc;

#ifdef RCV_WAVEPACK
	req.r[2].packType	= req.r[1].packType		= req.r[0].packType		= mv.packType;
#endif

	req.r[2].crc		= req.r[1].crc			= req.r[0].crc			= GetCRC16(&req.r[0].adr, sizeof(req.r[0])-3);

	return &q;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool CallBackRcvReq02(Ptr<REQ> &q)
{
	ReqRcv02 &req = *((ReqRcv02*)q->wb.data);
	RspRcv02 &rsp = *((RspRcv02*)q->rb.data);
	 
	bool crcOK = q->crcOK;

	if (crcOK)
	{
		rcvStatus |= 1 << (rsp.hdr.rw & 15);
		
		q->rsp->len = q->rb.len;
	}
	else
	{
		byte a = (req.r[0].adr-1) & 15;

		if (q->rb.recieved)
		{
			if ((rsp.hdr.rw & manReqMask) != manReqWord || (sizeof(rsp.hdr) + rsp.hdr.len * 8) != q->rb.len)
			{
				lenErr02[a]++;
			}
			else
			{
				crcErr02[a]++;
			};
		}
		else
		{
			notRcv02[a]++;
		};

		if (q->tryCount > 0)
		{
			q->tryCount--;
			qRcv.Add(q);
		}
		else
		{
			rcvStatus &= ~(1 << (a)); 
			rcv02rejVec += 1;
		};
	};

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateRcvReq02(byte adr, byte n, u16 tryCount)
{
	Ptr<REQ> rq(AllocREQ());
	
	if (!rq.Valid()) return rq;

	if (adr != 0)
	{
		rq->rsp = AllocFlashWriteBuffer(sizeof(RspRcv02)+2);
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

	ReqRcv02 &req = *((ReqRcv02*)rq->reqData);

	adr = (adr-1)&15; 
	//n &= 3; //chnl &= 3; 

	rq->rsp->len = 0;
	
	REQ &q = *rq;

	q.CallBack = CallBackRcvReq02;
	q.preTimeOut = MS2COM(1);
	q.postTimeOut = US2COM(100);
	q.ready = false;
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;
	q.crcType = REQ::CRC16_CCIT;
	
	q.wb.data = &req;
	q.wb.len = sizeof(req);

	req.r[1].len	= req.r[0].len	= sizeof(req.r[0]) - 1;
	req.r[1].adr	= req.r[0].adr	= adr+1;
	req.r[1].func	= req.r[0].func	= 2;
	req.r[1].n		= req.r[0].n	= n;
	req.r[1].crc	= req.r[0].crc	= GetCRC16(&req.r[0].adr, sizeof(req.r[0])-3);

	return rq;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool CallBackRcvReq03(Ptr<REQ> &q)
{
	if (!q->crcOK) 
	{
		crcErr03++;

		if (q->tryCount > 0)
		{
			q->tryCount--;
			qRcv.Add(q);
		};
	}
	else
	{
		RspRcv03 &rsp = *((RspRcv03*)q->rb.data);

		byte n = rsp.adr - 1;

		arrRcvTemp[n]			= rsp.temp;
		arrRcvNumDev[n]			= rsp.numdev;
		arrRcvVerDev[n]			= rsp.verdev;
		arrRcvNumDevValid[n]	= rsp.numDevValid;
		arrRcvFlashStatus[n]	= rsp.flashStatus;
	};

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateRcvReq03(byte adr, u16 tryCount)
{
	Ptr<REQ> rq(AllocREQ());
	
	if (!rq.Valid()) return rq;

	rq->rsp = AllocFlashWriteBuffer(sizeof(RspRcv03)+2);

	if (!rq->rsp.Valid()) { rq.Free(); return rq; };

	ReqRcv03 &req = *((ReqRcv03*)rq->reqData);
	RspRcv03 &rsp = *((RspRcv03*)(rq->rsp->GetDataPtr()));

	REQ &q = *rq;

	q.CallBack = CallBackRcvReq03;
	q.preTimeOut = US2COM(500);
	q.postTimeOut = US2COM(100);
	q.ready = false;
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;
	q.crcType = REQ::CRC16;
	
	q.wb.data = &req;
	q.wb.len = sizeof(req);

	q.rb.data = (adr != 0) ? &rsp : 0;
	q.rb.maxLen = rq->rsp->GetDataMaxLen();

	req.r[1].len			= req.r[0].len			= sizeof(req.r[0]) - 1;
	req.r[1].adr			= req.r[0].adr			= adr;
	req.r[1].func			= req.r[0].func			= 3;
	req.r[1].numDevValid	= req.r[0].numDevValid	= numDevRcvValid;
	req.r[1].numDev			= req.r[0].numDev		= numDevRcv;

	for (u16 i = 0; i < RCV_FIRE_NUM; i++)
	{
		byte n = transIndex[i];

		req.r[1].gain[i] = req.r[0].gain[i] = mv.trans[n].gain;
	};

	req.r[1].crc = req.r[0].crc = GetCRC16(&req.r[0].adr, sizeof(req.r[0])-3);
	
	return &q;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool CallBackRcvReq04(Ptr<REQ> &q)
{
	if (!q->crcOK) 
	{
		crcErr04++;

		if (q->tryCount > 0)
		{
			q->tryCount--;
			qRcv.Add(q);
		};
	}
	else
	{
		RspRcv04 &rsp = *((RspRcv04*)q->rb.data);

		u16 i = rsp.adr - 1;

		arrRcvTemp[i]			= rsp.temp;
		arrRcvNumDev[i]			= rsp.numdev;
		arrRcvVerDev[i]			= rsp.verdev;
		arrRcvNumDevValid[i]	= rsp.numDevValid;
		arrRcvFlashStatus[i]	= rsp.flashStatus;

		if (fireType < RCV_FIRE_NUM && i < RCV_MAX_NUM_STATIONS)
		{
			i = fireType*(RCV_MAX_NUM_STATIONS*4) + i*4;

			rspMan60.maxAmp[i + 0] = rsp.maxAmp[0];
			rspMan60.maxAmp[i + 1] = rsp.maxAmp[1];
			rspMan60.maxAmp[i + 2] = rsp.maxAmp[2];
			rspMan60.maxAmp[i + 3] = rsp.maxAmp[3];

			rspMan60.power[i + 0] = rsp.power[0];
			rspMan60.power[i + 1] = rsp.power[1];
			rspMan60.power[i + 2] = rsp.power[2];
			rspMan60.power[i + 3] = rsp.power[3];
		};
	};

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateRcvReq04(byte adr, byte saveParams, u16 tryCount)
{
	Ptr<REQ> rq(AllocREQ());
	
	if (!rq.Valid()) return rq;

	rq->rsp = AllocFlashWriteBuffer(sizeof(RspRcv04)+2);

	if (!rq->rsp.Valid()) { rq.Free(); return rq; };

	ReqRcv04 &req = *((ReqRcv04*)rq->reqData);
	RspRcv04 &rsp = *((RspRcv04*)(rq->rsp->GetDataPtr()));

	REQ &q = *rq;

	q.CallBack = CallBackRcvReq04;
	q.preTimeOut = US2COM(500);
	q.postTimeOut = US2COM(100);
	q.ready = false;
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;
	q.crcType = REQ::CRC16;
	
	q.wb.data = &req;
	q.wb.len = sizeof(req);

	q.rb.data = (adr != 0) ? &rsp : 0;
	q.rb.maxLen = rq->rsp->GetDataMaxLen();

	req.r[1].len		= req.r[0].len			= sizeof(req.r[0]) - 1;
	req.r[1].adr		= req.r[0].adr			= adr;
	req.r[1].func		= req.r[0].func			= 4;
	req.r[1].saveParams	= req.r[0].saveParams	= saveParams;
						 
	req.r[1].crc	= req.r[0].crc = GetCRC16(&req.r[0].adr, sizeof(req.r[0])-3);

	return &q;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateTrmReqFire(byte n)
{
	Ptr<REQ> rq(AllocREQ());
	
	if (!rq.Valid()) return rq;

	ReqTrm01 &req = *((ReqTrm01*)rq->reqData);

	REQ &q = *rq;

	const byte fn[3] = {0, 1, 1};

	q.CallBack = 0;
	q.ready = false;
	q.checkCRC = false;
	q.updateCRC = false;
	
	q.wb.data = &req;
	q.wb.len = sizeof(req);

	q.rb.data = 0;
	q.rb.maxLen = 0;

	byte t = fn[n];

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

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<MB> CreateTestDspReq01()
{
	Ptr<MB> rq;
	
	//rq = AllocFlashWriteBuffer(sizeof(RspDsp01));

	//if (!rq.Valid()) { return rq; };

	//RspDsp01 &rsp = *((RspDsp01*)(rq->GetDataPtr()));

	//rsp.CM.hdr.rw = manReqWord|0x40;
	//rsp.CM.hdr.time = 1;
	//rsp.CM.hdr.hallTime = 2;
	//rsp.CM.hdr.motoCount = 3;
	//rsp.CM.hdr.headCount = 4;
	//rsp.CM.hdr.ax = 5;
	//rsp.CM.hdr.ay = 6;
	//rsp.CM.hdr.az = 7;
	//rsp.CM.hdr.at = 8;
	//rsp.CM.hdr.sensType = 0;
	//rsp.CM.hdr.angle = 9;
	//rsp.CM.hdr.maxAmp = 10;
	//rsp.CM.hdr.fi_amp = 11;
	//rsp.CM.hdr.fi_time = 12;
	//rsp.CM.hdr.gain = 13;
	//rsp.CM.hdr.st = 14;
	//rsp.CM.hdr.sl = ArraySize(rsp.CM.data);
	//rsp.CM.hdr.sd = 0;
	//rsp.CM.hdr.packType = 0;
	//rsp.CM.hdr.packLen = 0;

	//for (u32 i = 0; i < rsp.CM.hdr.sl; i++)
	//{
	//	rsp.CM.data[i] = 0;
	//};

	//rq->len = sizeof(rsp.CM);
	
	return rq;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateRcvBootReq00(u16 adr, BootRspV1::SF0 *rspdata, u16 tryCount)
{
	Ptr<REQ> rq(AllocREQ());
	
	//rq.Alloc();//= REQ::Alloc();

	if (!rq.Valid()) return rq;

	if (rspdata == 0)
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
	q.ready = false;
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;
	
	q.wb.data = &req;
	q.wb.len = sizeof(req);
	
	q.rb.data = &rsp;
	q.rb.maxLen = sizeof(rsp);

	req.adr	= adr;
	req.rw	= RCV_BOOT_REQ_WORD|0;
	req.crc	= GetCRC16(&req, sizeof(req)-sizeof(req.crc));

	return rq;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateRcvBootReq01(u16 adr, u32 len, u16 tryCount)
{
	Ptr<REQ> rq(AllocREQ());
	
	//rq.Alloc();//= REQ::Alloc();

	if (!rq.Valid()) return rq;

	rq->rsp = AllocMemBuffer(sizeof(BootRspV1::SF1));

	if (!rq->rsp.Valid()) { rq.Free(); return rq; };

	BootReqV1::SF1 &req = *((BootReqV1::SF1*)rq->reqData);
	BootRspV1::SF1 &rsp = *((BootRspV1::SF1*)(rq->rsp->GetDataPtr()));
	
	REQ &q = *rq;

	q.CallBack = CallBackRcvBootReq01;
	q.preTimeOut = MS2COM(10);
	q.postTimeOut = US2COM(100);
	q.ready = false;
	q.tryCount = tryCount;
	q.checkCRC = true;
	q.updateCRC = false;
	
	q.wb.data = &req;
	q.wb.len = sizeof(req);
	
	q.rb.data = &rsp;
	q.rb.maxLen = sizeof(rsp);

	req.adr	= adr;
	req.rw	= RCV_BOOT_REQ_WORD|1;
	req.len = len;
	req.crc	= GetCRC16(&req, sizeof(req)-sizeof(req.crc));

	return rq;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool CallBackRcvBootReq02(Ptr<REQ> &q)
{
	bool retry = false;

	if (!q->crcOK) 
	{
		crcErr06++;

		retry = true;
	}
	else
	{
		BootRspV1::SF2 &rsp = *((BootRspV1::SF2*)q->rb.data);

		if (rsp.res != 0) wrtErr06++, retry = true;
	};

	if (retry && q->tryCount > 0)
	{
		q->tryCount--;
		qRcv.Add(q);

		return false;
	};

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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
	q.preTimeOut = MS2COM(500);
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

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateRcvBootReq03(u16 adr, u16 tryCount)
{
	Ptr<REQ> rq(AllocREQ());
	
	//rq.Alloc();//= REQ::Alloc();

	if (!rq.Valid()) return rq;

	rq->rsp = AllocMemBuffer(sizeof(BootRspV1::SF3));

	if (!rq->rsp.Valid()) { rq.Free(); return rq; };

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

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void CallBackMotoReq(Ptr<REQ> &q)
{
	//if (!q->crcOK) 
	//{
	//	if (q->tryCount > 0)
	//	{
	//		q->tryCount--;
	//		qTrm.Add(q);
	//	};
	//}
	//else
	//{
	//	RspMoto &rsp = *((RspMoto*)q->rb.data);

	//	if (rsp.rw == 0x101)
	//	{
	//		motoRPS		= rsp.rpm;
	//		motoCur		= rsp.current;
	//		motoCurLow	= rsp.currentLow;
	//		motoStat	= rsp.mororStatus;
	//		motoCounter = rsp.motoCounter;
	//		auxVoltage	= rsp.auxVoltage;
	//		motoVoltage	= rsp.motoVoltage;
	//		motoRcvCount++;
	//	};
	//};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateMotoReq()
{
	Ptr<REQ> rq(AllocREQ());
	
	//rq.Alloc();

	if (!rq.Valid()) return rq;

	//rq->rsp = AllocMemBuffer(sizeof(RspMoto));

	//if (!rq->rsp.Valid()) { rq.Free(); return rq; };

	//ReqMoto &req = *((ReqMoto*)rq->reqData);
	//RspMoto &rsp = *((RspMoto*)(rq->rsp->GetDataPtr()));
	//
	//REQ &q = *rq;

	//q.CallBack = CallBackMotoReq;
	////q.rb = &rb;
	////q.wb = &wb;
	//q.preTimeOut = MS2COM(1);
	//q.postTimeOut = US2COM(100);
	//q.ready = false;
	//q.checkCRC = true;
	//q.updateCRC = false;
	//q.tryCount = 1;
	//
	//q.wb.data = &req;
	//q.wb.len = sizeof(req);

	//q.rb.data = &rsp;
	//q.rb.maxLen = sizeof(rsp);
	//
	//req.rw = 0x101;
	//req.enableMotor	= 1;
	//req.tRPM = motoTargetRPS;
	//req.limCurrent = mv.motoLimCur;
	//req.maxCurrent = mv.motoMaxCur;
	//req.crc	= GetCRC16(&req, sizeof(req)-2);

	return rq;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void CallBackBootMotoReq01(Ptr<REQ> &q)
{
	if (!q->crcOK) 
	{
		if (q->tryCount > 0)
		{
			q->tryCount--;
			qTrm.Add(q);
		};
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateBootMotoReq01(u16 flashLen, u16 tryCount)
{
	Ptr<REQ> rq(AllocREQ());
	
	//rq.Alloc();

	if (!rq.Valid()) return rq;

	//rq->rsp = AllocMemBuffer(sizeof(RspBootMoto));

	//if (!rq->rsp.Valid()) { rq.Free(); return rq; };

	//ReqBootMoto &req = *((ReqBootMoto*)rq->reqData);
	//RspBootMoto &rsp = *((RspBootMoto*)(rq->rsp->GetDataPtr()));
	//
	//REQ &q = *rq;

	//q.CallBack = CallBackBootMotoReq01;
	//q.preTimeOut = MS2COM(10);
	//q.postTimeOut = US2COM(100);
	////q.rb = &rb;
	////q.wb = &wb;
	//q.ready = false;
	//q.tryCount = tryCount;
	//q.checkCRC = true;
	//q.updateCRC = false;
	//
	//q.wb.data = &req;
	//q.wb.len = sizeof(req.F1);
	//
	//q.rb.data = &rsp;
	//q.rb.maxLen = sizeof(rsp);

	//req.F1.func = 1;
	//req.F1.len = flashLen;
	//req.F1.align = ~flashLen;

	//req.F1.crc	= GetCRC16(&req, sizeof(req.F1) - sizeof(req.F1.crc));

	return rq;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void CallBackBootMotoReq03(Ptr<REQ> &q)
{
	if (!q->crcOK) 
	{
		if (q->tryCount > 0)
		{
			q->tryCount--;
			qTrm.Add(q);
		};
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateBootMotoReq03(u16 stAdr, u16 count, const u32* data, u16 tryCount)
{
	Ptr<REQ> rq(AllocREQ());
	
	//rq.Alloc();

	if (!rq.Valid()) return rq;

	//rq->rsp = AllocMemBuffer(sizeof(RspBootMoto));

	//if (!rq->rsp.Valid()) { rq.Free(); return rq; };

	//ReqBootMoto &req = *((ReqBootMoto*)rq->reqData);
	//RspBootMoto &rsp = *((RspBootMoto*)(rq->rsp->GetDataPtr()));
	//
	//REQ &q = *rq;

	//q.CallBack = CallBackBootMotoReq03;
	//q.preTimeOut = MS2COM(300);
	//q.postTimeOut = US2COM(100);
	////q.rb = &rb;
	////q.wb = &wb;
	//q.ready = false;
	//q.tryCount = tryCount;
	//q.checkCRC = true;
	//q.updateCRC = false;
	//
	//q.rb.data = &rsp;
	//q.rb.maxLen = sizeof(rsp);

	//req.F3.func = 3;

	//u16 max = ArraySize(req.F3.pdata);

	//if (count > max)
	//{
	//	count = max;
	//};

	//u32 count2 = max - count;

	//req.F3.padr = stAdr;

	//u32 *d = req.F3.pdata;

	//while(count > 0)
	//{
	//	*d++ = *data++;
	//	count--;
	//};

	//if (count2 > 0)
	//{
	//	*d++ = ~0;
	//	count2--;
	//};

	//u16 len = sizeof(req.F3) - sizeof(req.F3.crc);

	//req.F3.align = 0xAAAA;
	//req.F3.crc = GetCRC16(&req, len);

	//q.wb.data = &req;
	//q.wb.len = len+sizeof(req.F3.crc);

	return rq;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Ptr<REQ> CreateBootMotoReq02()
{
	Ptr<REQ> rq(AllocREQ());
	
	//rq.Alloc();

	if (!rq.Valid()) return rq;

	//rq->rsp = AllocMemBuffer(sizeof(RspBootMoto));

	//if (!rq->rsp.Valid()) { rq.Free(); return rq; };

	//ReqBootMoto &req = *((ReqBootMoto*)rq->reqData);
	//RspBootMoto &rsp = *((RspBootMoto*)(rq->rsp->GetDataPtr()));
	//
	//REQ &q = *rq;

	//q.CallBack = 0;
	//q.preTimeOut = MS2COM(10);
	//q.postTimeOut = US2COM(100);
	////q.rb = &rb;
	////q.wb = &wb;
	//q.ready = false;
	//q.tryCount = 1;
	//q.checkCRC = true;
	//q.updateCRC = false;
	//
	//q.rb.data = &rsp;
	//q.rb.maxLen = sizeof(rsp);
	//
	//req.F2.func = 2;
	//req.F2.align += 1; 

	//u16 len = sizeof(req.F2) - sizeof(req.F2.crc);

	//req.F2.crc = GetCRC16(&req, len);

	//q.wb.data = &req;
	//q.wb.len = len + sizeof(req.F2.crc);

	return rq;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static void InitRmemList()
//{
//	for (u16 i = 0; i < ArraySize(r02); i++)
//	{
//		freeR01.Add(&r02[i]);
//	};
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u32 InitRspMan_00(__packed u16 *data)
{
	__packed u16 *start = data;

	*(data++)	= (manReqWord & manReqMask) | 0;
	*(data++)	= mv.numDevice;
	*(data++)	= verDevice;
	
	return data - start;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void RequestFlashWrite_00(Ptr<MB> &flwb)
{
	__packed u16* data = (__packed u16*)(flwb->GetDataPtr());

	flwb->len = InitRspMan_00(data) * 2;

	RequestFlashWrite(flwb, data[0], true);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_00(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	InitRspMan_00(manTrmData);

	mtb->data1 = manTrmData;
	mtb->len1 = 3;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u32 InitRspMan_10(__packed u16 *data)
{
	__packed u16 *start = data;

	*(data++)	= (manReqWord & manReqMask) | 0x10;		//	1.	Ответное слово		
	*(data++)  	= mv.trmVoltage;						//	2.	Напряжение излучателей (0..1000 В)
	*(data++)  	= mv.trans[0].pulseCount;				//	3.	Количество импульсов монополь (1..5)
	*(data++)  	= mv.trans[1].pulseCount;				//	4.	Количество импульсов диполь (1..5)
	*(data++)  	= mv.trans[0].freq;						//	5.	Монополь. Частота импульсов излучателя (1Гц)
	*(data++)  	= mv.trans[1].freq;						//	6.	Диполь. Частота импульсов излучателя (1Гц)
	*(data++)  	= mv.trans[0].duty;						//	7.	Монополь. Скважность импульсов излучателя (0.01%)
	*(data++)  	= mv.trans[1].duty;						//	8.	Диполь. Скважность импульсов излучателя (0.01%)
	*(data++)  	= mv.trans[0].gain;						//	9.	Монополь. КУ
	*(data++)  	= mv.trans[0].st;						//	10.	Монополь. Шаг оцифровки
	*(data++)  	= mv.trans[0].sl;						//	11.	Монополь. Длина оцифровки
	*(data++)  	= mv.trans[0].sd;						//	12.	Монополь. Задержка оцифровки
	*(data++)  	= mv.trans[1].gain;						//	13.	Диполь. КУ
	*(data++)  	= mv.trans[1].st;						//	14.	Диполь. Шаг оцифровки
	*(data++)  	= mv.trans[1].sl;						//	15.	Диполь. Длина оцифровки
	*(data++)  	= mv.trans[1].sd;						//	16.	Диполь. Задержка оцифровки
	*(data++)  	= mv.disableFireNoVibration;			//	17.	Отключение регистрации на стоянке(0 - нет, 1 - да)
	*(data++)  	= mv.levelNoVibration;					//	18.	Уровень вибрации режима отключения регистрации на стойнке(у.е)(ushort)
	*(data++)  	= mv.firePeriod;						//	19.	Период опроса(мс)(ushort)

	return data - start;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void RequestFlashWrite_10(Ptr<MB> &flwb)
{
	__packed u16* data = (__packed u16*)(flwb->GetDataPtr());

	flwb->len = InitRspMan_10(data) * 2;

	RequestFlashWrite(flwb, data[0], true);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u32 InitRspMan_20(__packed u16 *data)
{
	__packed u16 *start = data;

	*(data++) = manReqWord|0x20;						//	1. 	ответное слово
	*(data++) = GD(&fireCounter, u16, 0);				//	2. 	счётчик. младшие 2 байта
	*(data++) = GD(&fireCounter, u16, 1);				//	3. 	счётчик. старшие 2 байта
	*(data++) = voltage;								//	4. 	напряжение передатчика
	*(data++) = numStations|(((u16)rcvStatus)<<8);		//	5. 	мл.байт - к-во приемников, ст.байт-статус приёмников
	*(data++) = rcvStatus;//resistValue;							//	6. 	сопротивление IdLine
	*(data++) = (temp+5)/10;							//	7. 	ТЕМПЕРАТУРА CPU
	*(data++) = ax;										//	8. 	AX
	*(data++) = ay;										//	9. 	AY
	*(data++) = az;										//	10.	AZ
	*(data++) = at;										//	11.	AT
	*(data++) = temp;									//	12.	Температура в приборе (short)(0.1гр)
	*(data++) = vibration;								//	13.	Вибрация (у.е)(ushort)

	return data - start;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void RequestFlashWrite_20(Ptr<MB> &flwb)
{
	__packed u16* data = (__packed u16*)(flwb->GetDataPtr());

	flwb->len = InitRspMan_20(data) * 2;

	RequestFlashWrite(flwb, data[0], true);
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_20(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	len = InitRspMan_20(manTrmData);

	mtb->data1 = manTrmData;
	mtb->len1 = len;
	mtb->data2 = 0;
	mtb->len2 = 0;

	runMainMode = true;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_30(u16 *data, u16 reqlen, MTB* mtb)
{
	__packed struct Req { u16 rw; u16 off; u16 len; };

	Req &req = *((Req*)data);

	if (data == 0 || reqlen == 0 || reqlen > 4 || mtb == 0) return false;

	byte nf = ((req.rw >> 4) - 3) & 3;
	byte nr = req.rw & 7;

	curRcv[nf] = nr+1;

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
		if (manVec30[nf].Valid())
		{
			RspRcv02& rsp = *((RspRcv02*)manVec30[nf]->GetDataPtr());

			if (rsp.hdr.rw == req.rw) curManVec30 = manVec30[nf];
		}
		
		manVec30[nf].Free();

		if (curManVec30.Valid())
		{
			RspRcv02 &rsp = *((RspRcv02*)(curManVec30->GetDataPtr()));

			u16 sz = (sizeof(rsp.hdr)-sizeof(rsp.hdr.rw))/2 + rsp.hdr.len * 4;

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

		u16 sz = (sizeof(rsp.hdr) - sizeof(rsp.hdr.rw)) / 2 + rsp.hdr.len * 4;

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

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_60(u16 *data, u16 len, MTB* mtb)
{
	struct Rsp { u16 rw; };
	
	static Rsp rsp; 

	if (data == 0 || len == 0 || len > 4 || mtb == 0) return false;

	mtb->data2 = 0;
	mtb->len2 = 0;

	if (len < 3)
	{
		rspMan60.rw = manReqWord|0x60;
		rspMan60.cnt = fireCounter;

		mtb->data1 = (u16*)&rspMan60;
		mtb->len1 = sizeof(rspMan60)/2;
	}
	else if (data[1] == 0)
	{
		rspMan60.rw = manReqWord|0x60;
		rspMan60.cnt = fireCounter;

		u16 maxlen = sizeof(rspMan60)/2;
		u16 len = data[2]+1;

		if (len > maxlen) len = maxlen;

		mtb->data1 = (u16*)&rspMan60;
		mtb->len1 = len;
	}
	else
	{
		rsp.rw = manReqWord|0x60;

		mtb->data1 = (u16*)&rsp;
		mtb->len1 = sizeof(rsp)/2;

		if (sizeof(rspMan60)/2 > data[1])
		{
			u16 maxlen = sizeof(rspMan60)/2 - data[1] - 1;
			u16 len = data[2];

			if (len > maxlen) len = maxlen;

			mtb->data2 = ((u16*)&rspMan60) + data[1]+1;
			mtb->len2 = len;
		};
	};

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_80(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len < 3 || len > 4 || mtb == 0) return false;

	switch (data[1])
	{
		case 1:

			mv.numDevice = data[2];

			break;

		case 2:

			manTrmBaud = data[2] - 1;	//SetTrmBoudRate(data[2]-1);

			break;
	};

	manTrmData[0] = (manReqWord & manReqMask) | 0x80;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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
		case 0x0:	mv.trmVoltage				= MIN(data[2],			950		);	break;	//0x00 - Напряжение излучателей (0..1000 В)
		case 0x1:	mv.trans[0].pulseCount		= LIM(data[2],	1,		5		);	break;	//0x01 - Монополь. Количество импульсов (1..5)
		case 0x2:	mv.trans[1].pulseCount		= LIM(data[2],	1,		5		);	break;	//0x02 - Диполь. Количество импульсов (1..5)
		case 0x3:	mv.trans[0].freq			= LIM(data[2],	10000,	30000	);	break;	//0x03 - Монополь. Частота импульсов излучателя (10000..30000 шаг 1Гц)
		case 0x4:	mv.trans[1].freq			= LIM(data[2],	1000,	10000	);	break;	//0x04 - Диполь. Частота импульсов излучателя (1000..10000 шаг 1Гц)
		case 0x5:	mv.trans[0].duty			= MIN(data[2],			6000	);	break;	//0x05 - Монополь. Скважность импульсов излучателя (0..60 шаг 0.01%)
		case 0x6:	mv.trans[1].duty			= MIN(data[2],			6000	);	break;	//0x06 - Диполь. Скважность импульсов излучателя (0..60 шаг 0.01%)

		case 0x10:	mv.trans[0].gain			= MIN(data[2],			7		);	break;	//0x10 - Монополь. КУ (0..7 = 1..128)
		case 0x18:	mv.trans[0].st				= MIN(data[2],			255		);	break;	//0x18 - Монополь. Шаг оцифровки (1..50)
		case 0x19:	mv.trans[0].sl				= MIN(data[2],			1024	);	break;	//0x19 - Монополь. Длина оцифровки (0..512)
		case 0x1A:	mv.trans[0].sd				= MIN(data[2],			1024	);	break;	//0x1A - Монополь. Задержка оцифровки 

		case 0x20:	mv.trans[2].gain			= MIN(data[2],			7		);	break;	//0x20 - Диполь. КУ (0..7 = 1..128)
		case 0x28:	mv.trans[2].st				= MIN(data[2],			255		);	break;	//0x28 - Диполь. Шаг оцифровки (1..50)
		case 0x29:	mv.trans[2].sl				= MIN(data[2],			1024	);	break;	//0x29 - Диполь. Длина оцифровки (0..512)
		case 0x2A:	mv.trans[2].sd				= MIN(data[2],			1024	);	break;	//0x2A - Диполь. Задержка оцифровки 

		case 0x30:	mv.disableFireNoVibration	= data[2];							break;	//0x30 - Отключение регистрации на стоянке(0 - нет, 1 - да)
		case 0x31:	mv.levelNoVibration			= data[2];							break;	//0x31 - Уровень вибрации режима отключения регистрации на стойнке(у.е)(ushort)
		case 0x32:	mv.firePeriod				= MAX(data[2],	300);				break;	//0x32 - Период опроса(мс)(ushort)

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

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_F0(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	SaveMainParams();

	manTrmData[0] = (manReqWord & manReqMask) | 0xF0;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan(u16 *buf, u16 len, MTB* mtb)
{
	if (buf == 0 || len == 0 || mtb == 0) return false;

	bool r = false;

	byte i = (buf[0]>>4)&0xF;

	switch (i)
	{
		case 0: 	r = RequestMan_00(buf, len, mtb); break;
		case 1: 	r = RequestMan_10(buf, len, mtb); break;
		case 2: 	r = RequestMan_20(buf, len, mtb); break;
		case 3:
		case 4:
		case 5:		r = RequestMan_30(buf, len, mtb); break;

		case 6: 	r = RequestMan_60(buf, len, mtb); break;
		case 8: 	r = RequestMan_80(buf, len, mtb); break;
		case 9:		r = RequestMan_90(buf, len, mtb); break;
		case 0xF:	r = RequestMan_F0(buf, len, mtb); break;
	};

	if (r) { mtb->baud = manTrmBaud; };

	return r;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_10(u16 *data, u16 len, MTB* mtb)
{
	//__packed struct T { u16 g[8]; u16 st; u16 len; u16 delay; u16 voltage; };
	//__packed struct Rsp { u16 hdr; u16 rw; T t1, t2, t3; };
	
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	manTrmData[0] = (memReqWord & memReqMask) | 0x10;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_20(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	__packed struct Rsp {u16 rw; u16 device; u16 session; u32 rcvVec; u32 rejVec; u32 wrVec; u32 errVec; u16 wrAdr[3]; u16 temp; byte status; byte flags; RTC rtc; };

	if (len != 1) return false;

	Rsp &rsp = *((Rsp*)&manTrmData);

	rsp.rw = (memReqWord & memReqMask)|0x20;
	rsp.device = GetDeviceID();  
	rsp.session = FLASH_Session_Get();	  
	rsp.rcvVec =  FLASH_Vectors_Recieved_Get();
	rsp.rejVec = rcv02rejVec; //FLASH_Vectors_Rejected_Get();
	rsp.wrVec = FLASH_Vectors_Saved_Get();
	rsp.errVec = FLASH_Vectors_Errors_Get();
	*((__packed u64*)rsp.wrAdr) = FLASH_Current_Adress_Get();
	rsp.temp = (temp+5)/10;
	rsp.status = FLASH_Status();

	GetTime(&rsp.rtc);

	mtb->data1 = manTrmData;
	mtb->len1 = 20;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_31(u16 *data, u16 len, MTB* mtb)
{
	if (len != 1) return false;

	cmdWriteStart_00 = cmdWriteStart_10 = FLASH_WriteEnable();

	manTrmData[0] = (memReqWord & memReqMask)|0x31;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_32(u16 *data, u16 len, MTB* mtb)
{
	if (len != 1) return false;

	FLASH_WriteDisable();

	manTrmData[0] = (memReqWord & memReqMask)|0x32;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMem_F0(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	//SaveParams();

	manTrmData[0] = (memReqWord & memReqMask) | 0xF0;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


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

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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
				i++;
			};

			break;

		case 4:

			reqr = CreateRcvReqFire(n, next_n, fireCounter);

			if (reqr.Valid())
			{
				comRcv.Write(&reqr->wb);
				
				i++;
			};

			break;

		case 5:

			if (mv.trmVoltage > 100)
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

				u32 t = 1000 + (u32)mv.trans[tn].sl * mv.trans[tn].st + mv.trans[tn].sd;

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

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void MainMode()
{
	//fireType

	static byte rcv = 0;
	//static byte chnl = 0;
	//static REQ *req = 0;
	//static R02 *r02 = 0;
	static CTM32 ctm;
	static CTM32 ctm2;

	static byte saveRcvParams = 0;

	static Ptr<REQ> req;
//	REQ *rm = 0;

	switch (mainModeState)
	{
		case 0:

			if (runMainMode && (mv.disableFireNoVibration == 0 || vibration > mv.levelNoVibration))
			{
				req = CreateRcvReq03(1, 0);

				if (req.Valid())
				{
					qRcv.Add(req);

					if (cmdRcvSaveParams) saveRcvParams = 1;

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

		case 3:

			req = CreateRcvReq02(rcv, fireType, 1);

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

					bool crc = false;
					//bool free = true;

					u16 rw = manReqWord | ((3+fireType) << 4) | (rcv - 1);
					
					if (r02.hdr.rw != rw || r02.hdr.cnt != fireCounter)
					{
						//__breakpoint(0);

						//r02.hdr.rw = manReqWord | ((3+fireType) << 4) | (rcv - 1);
						//r02.hdr.cnt = fireCounter;
						//crc = true;
					};

					if (!RequestFlashWrite(req->rsp, r02.hdr.rw, crc)) __breakpoint(0);

					if (curRcv[fireType] == rcv) manVec30[fireType] = req->rsp;

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

				ctm.Reset();
			};

			qRcv.Update();

			break;

		case 5:

			if (ctm.Check(US2CTM(100)))
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
					cmdRcvSaveParams = false;
					saveRcvParams = 0;

					mainModeState++;
				};
			};

			break;

		case 8:

			if (ctm.Check(MS2CTM(mv.firePeriod/16)))
			{
				byte pft = fireType;

				fireType = nextFireType; 

				do nextFireType = (nextFireType+1)%RCV_FIRE_NUM; while((fireMask & (1UL<<nextFireType)) == 0);

				fireCounter += 1;

				if (fireType > pft)
				{
					mainModeState = 9;
				}
				else
				{
					mainModeState = 10;

					cmdWriteStart_20 = true;
				};
			};

			break;

		case 9:

			if (ctm.Check(MS2CTM(mv.firePeriod/4)))
			{
				mainModeState = 0;
			};

			break;

		case 10:

			if (ctm2.Check(MS2CTM(mv.firePeriod)))
			{
				mainModeState = 0;
			};

			break;
	};

	//static Ptr<MB> mb;
	//static TM32 tm;
	//static RspDsp01 *rsp = 0;

	//switch (mainModeState)
	//{
	//	case 0:

	//		mb = readyR01.Get();

	//		if (mb.Valid())
	//		{
	//			rsp = (RspDsp01*)(mb->GetDataPtr());

	//			RequestFlashWrite(mb, rsp->CM.hdr.rw, true);

	//			mainModeState++;
	//		};

	//		break;

	//	case 1:

	//		if ((rsp->CM.hdr.rw & 0xFF) == 0x40)
	//		{
	//			byte n = rsp->CM.hdr.sensType;

	//			if (n < SENS_NUM)
	//			{
	//				manVec40[n] = mb;

	//				AmpTimeMinMax& mm = sensMinMaxTemp[n];

	//				u16 amp		= rsp->CM.hdr.maxAmp;
	//				u16 time	= rsp->CM.hdr.fi_time;

	//				if (amp > mm.ampMax)	mm.ampMax = amp;
	//				if (amp < mm.ampMin)	mm.ampMin = amp;
	//				if (time > mm.timeMax)	mm.timeMax = time;
	//				if (time < mm.timeMin)	mm.timeMin = time;

	//				mm.valid = true;
	//			};
	//		}
	//		else if ((rsp->IM.hdr.rw & 0xFF) == 0x50)
	//		{
	//			byte n = rsp->IM.hdr.sensType;

	//			if (n < (SENS_NUM-1))
	//			{
	//				manVec50[n] = mb;
	//			};
	//		};

	//		if (imModeTimeout.Check(10000))
	//		{
	//			SetModeCM();
	//		};

	//		mb.Free();

	//		mainModeState++;

	//		break;

	//	case 2:

	if (cmdWriteStart_00)
	{
		Ptr<MB> b(AllocFlashWriteBuffer(6));

		if (b.Valid())
		{
			RequestFlashWrite_00(b);

			cmdWriteStart_00 = false;
		};
	}
	else if (cmdWriteStart_10)
	{
		Ptr<MB> b(AllocFlashWriteBuffer(44));

		if (b.Valid())
		{
			RequestFlashWrite_10(b);

			cmdWriteStart_10 = false;
		};
	}
	else if (cmdWriteStart_20)
	{
		Ptr<MB> b(AllocFlashWriteBuffer(60));

		if (b.Valid())
		{
			RequestFlashWrite_20(b);

			cmdWriteStart_20 = false;
		};
	};
	//else if (tm.Check(1001))
	//{
	//	cmdWriteStart_20 = true;
	//};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static DSCSPI dscAccel;

//static i16 ax = 0, ay = 0, az = 0, at = 0;


static u8 txAccel[25] = { 0 };
static u8 rxAccel[50];

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateAccel()
{
	static byte i = 0; 
	static i32 fx = 0, fy = 0, fz = 0, fv = 0;

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

				ay = -(fz / 65536); 
				ax = -(fy / 65536); 
				az =  (fx / 65536);

				//at = 2500 + ((1852 - t) * 2000 + 91) / 181;
				at = 2500 + ((1852 - t) * 11315 + 512) / 1024;

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

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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
#ifdef CPU_SAME53	

	I2C_Update();

#elif defined(CPU_XMC48)

	//if (!comdsp.Update())
	//{
	//	if (I2C_Update())
	//	{
	//		comdsp.InitHW();

	//		i2cResetCount++;
	//	};
	//};

#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static void UpdateMoto()
//{
//	static Ptr<REQ> rq;
//	static TM32 tm;
//
//	static byte i = 0;
//
//	switch (i)
//	{
//		case 0:
//
//			rq = CreateMotoReq();
//
//			if (rq.Valid())
//			{
//				qTrm.Add(rq);
//
//				i++;
//			};
//
//			break;
//
//		case 1:
//
//			if (rq->ready)
//			{
//				rq.Free();
//
//				i++;
//			};
//
//			break;
//
//		case 2:
//
//			if (tm.Check(101)) i = 0;
//
//			break;
//	};
//
//	qTrm.Update();
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateTestFlashWrite()
{
//	static Ptr<MB> ptr;
//	static u32 count = 0;
//
//	static CTM32 rtm;
//
//	if (rtm.Check(MS2CTM(1)))
//	{
//		testDspReqCount++;
//
//		count = 1000;
//	};
//
////	if (count != 0)
//	{
//		ptr = CreateTestDspReq01();
//
//		if (ptr.Valid())
//		{
//			count--;
//
//			RspDsp01 *rsp = (RspDsp01*)(ptr->GetDataPtr());
//			RequestFlashWrite(ptr, rsp->CM.hdr.rw, true);
//
//		};
//	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateDSP_Com()
{
//	static Ptr<REQ> rq;
//
//	static byte i = 0;
////	static TM32 tm;
//
//	switch (i)
//	{
//		case 0:
//
//			if ((mv.fireVoltage == 0 && motoTargetRPS == 1500) || __WIN32__)
//			{
//				if (FLASH_Status() != 0) UpdateTestFlashWrite();
//			}
//			else
//			{
//				rq = CreateDspReq01(1);
//
//				if (rq.Valid())	qRcv.Add(rq), i++;
//			};
//
//			break;
//
//		case 1:
//
//			if (rq->ready)
//			{
//				if (rq->crcOK)
//				{
//					readyR01.Add(rq->rsp);
//				};
//				
//				i = 0;
//
//				rq.Free();
//			};
//
//			break;
//	};

	qRcv.Update();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef DSPSPI

static void UpdateDSP_SPI()
{
	static Ptr<MB> mb;

	//static RspDsp01 rsp;

	static byte i = 0;
	static u16 len = 0;
	static bool crc = false;
	//static u32 ptime = 0;

	static S_SPIS::RBUF rb;

	switch (i)
	{
		case 0:

			if (!mb.Valid())
			{
				mb = AllocFlashWriteBuffer(sizeof(RspDsp01)+6);
			};

			if (mb.Valid())
			{
				mb->dataOffset = (mb->dataOffset + 3) & ~3;

				rb.data = mb->GetDataPtr();
				rb.maxLen = mb->GetDataMaxLen() & ~3;

				spidsp.Read(&rb, ~0, US2SPIS(50));

				HW::PIOC->BSET(19);
				PIO_SS->BCLR(PIN_SS);

				i++;
			};

			break;

		case 1:

			if (!spidsp.Update())
			{
				PIO_SS->BSET(PIN_SS);
				HW::PIOC->BCLR(19);

				bool c = false;

				RspDsp01 &rsp = *((RspDsp01*)rb.data);

				if (rsp.CM.hdr.rw == (dspReqWord|0x40))
				{
					if (rsp.CM.hdr.packType == 0)
					{
						len = rsp.CM.hdr.sl*2 + sizeof(rsp.CM.hdr) + 2;
						crc = (GetCRC16(&rsp.CM.hdr, sizeof(rsp.CM.hdr)) == rsp.CM.data[rsp.CM.hdr.sl]);
					}
					else
					{
						len = rsp.CM.hdr.packLen*2 + sizeof(rsp.CM.hdr) + 2;
						crc = (GetCRC16(&rsp.CM.hdr, sizeof(rsp.CM.hdr)) == rsp.CM.data[rsp.CM.hdr.packLen]);
					};

					c = /*(rb.len >= len) &&*/ crc;

					if (c)
					{
						mb->len = rb.len - 2;

						dspRcv40++;
						//dspRcvCount++;
					};
				}
				else if (rsp.IM.hdr.rw == (dspReqWord|0x50))
				{
					if (rb.len == (rsp.IM.hdr.dataLen*4 + sizeof(rsp.IM.hdr) + 2))
					{
						if (rsp.IM.data[rsp.IM.hdr.dataLen] == GetCRC16(&rsp.IM.hdr, sizeof(rsp.IM.hdr)))
						{
							mb->len = rb.len - 2;
							
							dspRcv50++;
							//dspRcvCount++;

							//dspMMSEC = rsp.IM.hdr.time;
							//shaftMMSEC = rsp.IM.hdr.hallTime;

							c = true;
						};
					};
				};

				if (c)
				{
					readyR01.Add(mb);

					mb.Free();
				}
				else if (rb.len != 0)
				{
					HW::PIOC->BSET(18);
					dspRcvErr++;
					HW::PIOC->BCLR(18);
				}
				else
				{
					HW::PIOC->BSET(17);
					dspNotRcv++;
					HW::PIOC->BCLR(17);
				};

				i = 0;

			};

			break;
	};
}
#endif
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

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void FlashRcv()
{
	Ptr<REQ> rq;

	u32 flashLen = sizeof(rcvFlashPages);
	u16 flashCRC = GetCRC16(rcvFlashPages, flashLen);

	BootRspV1::SF0	rspF0[RCV_MAX_NUM_STATIONS];

	u16 tryCount = 100;
	
	rcvStatus = 0;
	byte N = RCV_MAX_NUM_STATIONS;

	comRcv.Disconnect();

	comRcv.Connect(ComPort::ASYNC, RCV_BOOT_COM_BAUDRATE, RCV_BOOT_COM_PARITY, 1);

	while (tryCount > 0)
	{
		for (byte i = 1; i <= N; i++)
		{
			if ((rcvStatus & (1 << (i-1))) == 0)
			{
				rq = CreateRcvBootReq00(i, &rspF0[i-1], 2);

				if (rq.Valid())
				{
					qRcv.Add(rq); while(!rq->ready) qRcv.Update();

					if (rq->crcOK)
					{
						BootReqV1::SF0 &req = *((BootReqV1::SF0*)rq->wb.data);
						BootRspV1::SF0 &rsp = rspF0[i-1];

						if (rsp.adr == i && rsp.rw == req.rw)
						{
							rcvStatus |= 1 << (i-1);
						};

						if (tryCount > RCV_MAX_NUM_STATIONS) tryCount = RCV_MAX_NUM_STATIONS;
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

			qRcv.Add(rq); while(!rq->ready) { qRcv.Update(); };

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

					qRcv.Add(rq); while(!rq->ready) qRcv.Update();

					count -= len;
					p += len;
					adr += len;
				};
			};

			rq = CreateRcvBootReq03(i, 2);

			qRcv.Add(rq); while(!rq->ready) qRcv.Update();

			rq.Free();
		};
	};

	rq = CreateRcvBootReq03(0, 2);

	qRcv.Add(rq); while(!rq->ready) qRcv.Update();

	rq.Free();
	
	comRcv.Disconnect();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
static void FlashDSP()
{
	TM32 tm;

	Ptr<REQ> req;

	tm.Reset();

	while (!tm.Check(100)) HW::WDT->Update();

	while (!req.Valid()) req = CreateRcvBootReq00(2);

	qRcv.Add(req); while(!req->ready) { qRcv.Update(); HW::WDT->Update(); };

	if (req->crcOK)
	{
		RspDsp05 *rsp = (RspDsp05*)req->rb.data;

		u16 flen = sizeof(dspFlashPages);
		const u32 *fpages = dspFlashPages;

		u16 fcrc = GetCRC16(fpages, flen);

		if (rsp->flashCRC != fcrc || rsp->flashLen != flen)
		{
			u16 count = flen+2;
			u16 adr = 0;
			byte *p = (byte*)fpages;

			while (count > 0)
			{
				u16 len;
				
				if (count > 256)
				{
					len = 256;

					req = CreateDspReq06(adr, len, p, 0, 0, 10);
				}
				else
				{
					len = count;

					if (len > 2)
					{
						req = CreateDspReq06(adr, len-2, p, sizeof(fcrc), &fcrc, 10);
					}
					else
					{
						req = CreateDspReq06(adr, sizeof(fcrc), &fcrc, 0, 0, 10);
					};
				};

				qRcv.Add(req); while(!req->ready) { qRcv.Update(); HW::WDT->Update(); };

				count -= len;
				p += len;
				adr += len;
			};

			req = CreateDspReq07();

			qRcv.Add(req); while(!req->ready) { qRcv.Update();	};
		};
	};
}
*/
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static const u32 motoFlashPages[] = {
//#include "G26K2_V2_LPC824.BIN.H"
};

u16 motoFlashLen = 0;
u16 motoFlashCRC = 0;


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void FlashMoto()
{
	static BootReqHS	reqHS;
	static BootRspHS	rspHS;
	static ComPort::WriteBuffer wb;
	static ComPort::ReadBuffer	rb;

	const unsigned __int64 masterGUID = TRM_BOOT_MGUID;
	const unsigned __int64 slaveGUID = TRM_BOOT_SGUID;

	CTM32 ctm;

	Ptr<REQ> req;

	motoFlashLen = sizeof(motoFlashPages);
	motoFlashCRC = GetCRC16(motoFlashPages, motoFlashLen);

	ctm.Reset();

	bool hs = false;

	while (!ctm.Check(MS2CTM(200)))
	{
		reqHS.guid = masterGUID;
		reqHS.crc = GetCRC16(&reqHS, sizeof(reqHS) - sizeof(reqHS.crc));
		wb.data = &reqHS;
		wb.len = sizeof(reqHS);

		comTrm.Write(&wb);

		while (comTrm.Update()) HW::WDT->Update(); 

		rb.data = &rspHS;
		rb.maxLen = sizeof(rspHS);
		comTrm.Read(&rb, MS2COM(5), US2COM(100));

		while (comTrm.Update()) HW::WDT->Update();;

		if (rb.recieved && rb.len == sizeof(rspHS) && GetCRC16(&rspHS, sizeof(rspHS)) == 0 && rspHS.guid == slaveGUID)
		{
			hs = true;
			break;
		};
	};

	if (hs)
	{
		req = CreateBootMotoReq01(motoFlashLen, 2);

		qTrm.Add(req); while(!req->ready) { qTrm.Update(); HW::WDT->Update(); };

		if (req->crcOK)
		{
			BootRspMes *rsp = (BootRspMes*)req->rb.data;

			if (rsp->F1.sCRC != motoFlashCRC || rsp->F1.len != motoFlashLen)
			{
				u16 count = motoFlashLen/4;
				u32 adr = 0;
				const u32 *p = motoFlashPages;

				while (count > 0)
				{
					u16 len = (count > 16) ? 16 : count;

					for(u32 i = 3; i > 0; i--)
					{
						req = CreateBootMotoReq03(adr, len, p, 3);

						qTrm.Add(req); while(!req->ready) { qTrm.Update(); HW::WDT->Update(); };

						BootRspMes *rsp = (BootRspMes*)req->rb.data;

						if (req->crcOK && rsp->F3.status) { break;	}
					};

					ctm.Reset();

					while (!ctm.Check(MS2CTM(1))) HW::WDT->Update();

					count -= len;
					p += len;
					adr += len*4;
				};
			};
		};

		req = CreateBootMotoReq02();

		qTrm.Add(req); while(!req->ready) { qTrm.Update(); HW::WDT->Update();	};

		ctm.Reset();

		while (!ctm.Check(MS2CTM(1))) HW::WDT->Update();
	};
}

#endif
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitMainVars()
{
	mv.numDevice		= 11111;
	mv.numMemDevice		= 11111;

	mv.trans[0].gain		= 0;
	mv.trans[0].st 		= 5;	
	mv.trans[0].sl 		= 512;	
	mv.trans[0].sd 		= 200;	
	mv.trans[0].freq		= 16000;
	mv.trans[0].duty		= 50;
	mv.trans[0].amp		= 1200;
	mv.trans[0].pulseCount = 1;

	mv.trans[1].gain		= 1;
	mv.trans[1].st 		= 10;	
	mv.trans[1].sl 		= 512;	
	mv.trans[1].sd 		= 200;	
	mv.trans[1].freq		= 3000;
	mv.trans[1].duty		= 50;
	mv.trans[1].amp		= 1200;
	mv.trans[1].pulseCount = 1;

	mv.trans[2].gain		= 1;
	mv.trans[2].st 		= 10;	
	mv.trans[2].sl 		= 512;	
	mv.trans[2].sd 		= 500;	
	mv.trans[2].freq		= 1000;
	mv.trans[2].duty		= 50;
	mv.trans[2].amp		= 1200;
	mv.trans[2].pulseCount = 1;

	mv.trmVoltage				= 800;
	mv.disableFireNoVibration	= 1;
	mv.levelNoVibration		= 100;
	mv.firePeriod				= 1000;
	mv.lfMnplEnabled			= 0;
	mv.packType				= 0;

	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_CYAN "Init Main Vars Vars ... OK\n");
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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
		CALL( FLASH_Update();			);
		CALL( UpdateHardware();			);
		CALL( UpdateAccel();			);
		CALL( UpdateI2C();				);
		CALL( SaveVars();				);
	};

	i = (i > (__LINE__-S-3)) ? 0 : i;

	#undef CALL
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateMisc()
{
	static byte i = 0;

	#define CALL(p) case (__LINE__-S): p; break;

	enum C { S = (__LINE__+3) };
	switch(i++)
	{
		CALL( UpdateEMAC();		);
		CALL( UpdateRcvTrm();	);
		CALL( SPI_Update();		);
		CALL( UpdateParams();	);
		CALL( I2C_Update();		); 
	};

	i = (i > (__LINE__-S-3)) ? 0 : i;

	#undef CALL
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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
		UpdateMisc();
	}
	else
	{
		I2C_Update();
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

//	static bool c = true;

//	static byte buf[16384];

	//volatile byte * const FLD = (byte*)0x60000000;	
	
	//static ComPort commem;

	DSCSPI dsc, dsc2;

	TM32 tm;

	//__breakpoint(0);

	InitHardware();

	LoadVars();

	InitEMAC();

	FLASH_Init();
 
//	Update_RPS_SPR();

#ifndef WIN32

	comTrm.Connect(ComPort::ASYNC, TRM_COM_BAUDRATE, TRM_COM_PARITY, TRM_COM_STOPBITS);


	//__breakpoint(0);

	//FlashMoto();

	FlashRcv();

	comRcv.Connect(ComPort::ASYNC, RCV_COM_BAUDRATE, RCV_COM_PARITY, 1);

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

			//static MTB mtb;
			//static u16 buf[20];

			//mtb.baud = 1;
			//mtb.data1 = buf;
			//mtb.len1 = ArraySize(buf);
			//mtb.data2 = 0;
			//mtb.len2 = 0;

			//SendManData(&mtb);

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
				FLASH_WriteEnable();
			}
			else if (key == 'e')
			{
				FLASH_WriteDisable();
			}
			else if (key == 'p')
			{
				NAND_FullErase();
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
