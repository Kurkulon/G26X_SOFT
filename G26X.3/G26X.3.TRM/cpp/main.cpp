#include "hardware.h"
//#include "options.h"
//#include "hw_emac.h"
#include <CRC\CRC16.h>
#include <ComPort\ComPort.h>
#include <CRC\CRC16_CCIT.h>
#include <list.h>
#include <PointerCRC.h>
#include <SEGGER_RTT\SEGGER_RTT.h>
#include "hw_com.h"
#include "G_TRM.h"

//#include "G_TRM.H"


enum { VERSION = 0x101 };

//#pragma O3
//#pragma Otime

#ifndef _DEBUG
	static const bool __debug = false;
#else
	static const bool __debug = true;
#endif

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct MainVars // NonVolatileVars  
{
	u32 timeStamp;

	u16 numDevice;

	u16 fireVoltage;
	u16 fireType;
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static MainVars mv;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u32 fps;

static u16 manRcvData[10];
static u16 manTrmData[50];
static u16 manTrmBaud = 0;
//static u16 memTrmBaud = 0;

u16 txbuf[128 + 512 + 16];

static u16 curFireVoltage = 300;

static const u16 manReqWord = 0xA700;
static const u16 manReqMask = 0xFF00;

static u16 verDevice = VERSION;

i16 temp = 0;

static byte svCount = 0;

static Rsp20 *curRsp20 = 0;

u16 fireAmp = 0;
u16 fireFreq = 3000;

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

extern u16 GetNumDevice()
{
	return mv.numDevice;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

extern u16 GetVersionDevice()
{
	return verDevice;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u32 InitRspMan_00(__packed u16 *data)
{
	__packed u16 *start = data;

	*(data++)	= (manReqWord & manReqMask) | 0;
	*(data++)	= mv.numDevice;
	*(data++)	= verDevice;
	
	return data - start;
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

	*(data++)	= (manReqWord & manReqMask) | 0x10;		//	1. ответное слово
	*(data++)	= mv.fireVoltage;						//	2. Напряжение излучателя (0...300 В)
	*(data++)	= mv.fireType;							//	3. Формирование излучения (тип, 0,1,2.. )

	return data - start;
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

static bool RequestMan_20(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	curRsp20 = GetReadyRsp20();
	
	if (curRsp20 == 0)
	{
		manTrmData[0] = data[0];		//	1. Ответное слово	

		mtb->data1 = manTrmData;
		mtb->len1 = 1;
	}
	else
	{
		Rsp20 &rsp = *curRsp20;

		rsp.hdr.rw = data[0];							//	1. Ответное слово	
		rsp.hdr.amp = 0;								//	4. Аплитуда излучателя измеренная (В)
		rsp.hdr.temp = temp;							//	5. Температура излучателя измеренная (0.1гр) (short)

		mtb->data1 = &rsp.hdr.rw;

		data = rsp.data;

		i16 amp = 0;

		for (u16 i = 0; i < rsp.osc.sl; i++, data++)
		{
			i16 t = data[0]-0x7FF;
			t = t * 5 / 4;
			data[0] = t;

			if (t > amp) amp = t;
		};

		rsp.hdr.amp = amp;

		mtb->len1 = sizeof(rsp.hdr)/2 + ((rsp.hdr.rw&1) ? (sizeof(rsp.osc)/2 + curRsp20->osc.sl) : 0);
	};

	mtb->data2 = 0;
	mtb->len2 = 0;

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

static bool RequestMan_90(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len < 3 || len > 4 || mtb == 0) return false;

	switch(data[1])
	{
		case 0x00:	mv.fireVoltage		= MIN(data[2], 300);	break;	//	0x00. Напряжение излучателя (0...300 вольт)
		case 0x01:	mv.fireType			= MIN(data[2], 1);		break;	//	0x01. Формирование излучения (тип, 0,1,2.. )

		default:

			return false;
	};

	manTrmData[0] = (manReqWord & manReqMask) | 0x90;

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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

	if ((buf[0] & manReqMask) != manReqWord) return false;

	bool r = false;

	byte i = (buf[0]>>4)&0xF;

	switch (i)
	{
		case 0: 	r = RequestMan_00(buf, len, mtb); break;
		case 1: 	r = RequestMan_10(buf, len, mtb); break;
		case 2: 	r = RequestMan_20(buf, len, mtb); break;
		case 8: 	r = RequestMan_80(buf, len, mtb); break;
		case 9:		r = RequestMan_90(buf, len, mtb); break;
		case 0xF:	r = RequestMan_F0(buf, len, mtb); break;
	};

	if (r) { mtb->baud = manTrmBaud; };

	return r;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateMan()
{
	static MTB mtb;
	static MRB mrb;

	static byte i = 0;

	static RTM tm;

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

				if (mrb.OK && mrb.len > 0 && ((manRcvData[0] & manReqMask) == manReqWord && RequestMan(manRcvData, mrb.len, &mtb)))
				{
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

			if (tm.Check(US2RT(200)))
			{
				SendManData(&mtb);

				i++;
			};

			break;

		case 3:

			if (mtb.ready)
			{
				if (curRsp20 != 0) FreeRsp20(curRsp20), curRsp20 = 0;

				i = 0;
			};

			break;

	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

typedef bool (*REQF)(byte *req, u16 len, ComPort::WriteBuffer *wb);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool Request01(byte *data, u16 len, ComPort::WriteBuffer *wb)
{
	// Запуск импульса излучателя

	ReqTrm01 &req = *((ReqTrm01*)data);
	
	PrepareFire(req.r[0].n, req.r[0].fireFreq, req.r[0].fireAmp, req.r[0].fireCount, req.r[0].fireDuty);

	wb->data = 0;
	wb->len = 0;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool Request02(byte *data, u16 len, ComPort::WriteBuffer *wb)
{
	// Чтение текущего значения высокого напряжения

	//__packed struct Rsp { byte f; u16 hv; u16 crc; };

	//static Rsp rsp;
	//
	//rsp.f = 2;
	//rsp.hv = GetCurHV();
	//rsp.crc = GetCRC(&rsp, 3);
	//wb->data = &rsp;
	//wb->len = sizeof(rsp);

	return false;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool Request03(byte *data, u16 len, ComPort::WriteBuffer *wb)
{
	// Установка значения требуемого высокого напряжения

	//__packed struct Rsp { byte f; u16 crc; };

	//static Rsp rsp;
	//
	//SetReqFireCountM(req->f3.fireCountM);
	//SetReqFireCountXY(req->f3.fireCountXY);
	//SetReqHV(req->f3.hv);
	//SetReqFireFreqM(req->f3.fireFreqM, req->f3.fireDutyM);
	//SetReqFireFreqXY(req->f3.fireFreqXY, req->f3.fireDutyXY);

	//rsp.f = 3;
	//rsp.crc = GetCRC(&rsp, 1);
	//wb->data = &rsp;
	//wb->len = sizeof(rsp);

	return false;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static REQF listReq[3] = { Request01, Request02, Request03 };

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool UpdateRequest(ComPort::WriteBuffer *wb, ComPort::ReadBuffer *rb)
{
	//static Req nulReq;

	static const byte fl[3] = { sizeof(ReqTrm01::r[0])-1, sizeof(ReqTrm02::r[0])-1, sizeof(ReqTrm03::r[0])-1 };

	if (rb == 0 || rb->len < 4) return false;

	bool result = false;

	u16 rlen = rb->len;

	byte *p = (byte*)rb->data;

	while(rlen > 3)
	{
		byte len = p[0];
		byte func = p[1]-1;

		if (func < 4 && len == fl[func] && len < rlen && GetCRC16(p+1, len) == 0)
		{
			//Req *req = (Req*)p;

			result = listReq[func](p, len+1, wb);

			break;
		}
		else
		{
			p += 1;
			rlen -= 1;
		};
	};

	return result;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateCom()
{
	static byte i = 0;
	static CTM32 ctm;
	static ComPort::WriteBuffer wb;
	static ComPort::ReadBuffer rb;
	static MTB mtb;
	static u16 buf[16];
	
	switch(i)
	{
		case 0:

			rb.data = buf;
			rb.maxLen = sizeof(buf);
			comdsp.Read(&rb, MS2COM(5000), US2COM(100));
			i++;

			break;

		case 1:

			if (!comdsp.Update())
			{
				if (rb.recieved && rb.len >= 6)
				{
					ctm.Reset();

					if (UpdateRequest(&wb, &rb))
					{
						i += 2;
					}
					else if (wb.data != 0 && wb.len != 0)
					{
						comdsp.Write(&wb);

						i++;
					}
					else
					{
						i = 0;
					};
				}
				else
				{
					i = 0;
				};
			};

			break;

		case 2:

			if (!comdsp.Update())
			{
				i = 0;
			};

			break;

		case 3:

			if (IsFireOK() || ctm.Check(MS2CTM(50)))
			{
				DisableFire();

				i = 0;
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
				if (!__debug) { HW::ResetWDT(); };

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

		case 1:

			if (dsc.ready)
			{
				if (dsc.ack && dsc.readedLen == dsc.rlen)
				{
					i32 t = (i16)ReverseWord(rbuf);

					temp = (t * 10 + 64) / 128;
				};

				i = 0;
			};

			break;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateHV()
{
	static byte i = 0;
	static DSCI2C dsc;
	static byte wbuf[4];
	static byte rbuf[4];
	static TM32 tm;
	//static CTM32 ctm;
	static i32 filtFV = 0;
	//static i32 filtMV = 0;
	static u16 correction = 0x200;
	static u16 dstFV = 0;

	//if (!ctm.Check(US2CCLK(50))) return;

	switch (i)
	{
		case 0:

			if (tm.Check(10))
			{
				wbuf[0] = 2;
				wbuf[1] = 0;
				wbuf[2] = 0;

				dsc.adr = 0x48;
				dsc.wdata = wbuf;
				dsc.wlen = 3;
				dsc.rdata = 0;
				dsc.rlen = 0;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				I2C_AddRequest(&dsc);

				i++;
			};

			break;

		case 1:

			if (dsc.ready)
			{
				wbuf[0] = 3;	
				wbuf[1] = 1;	
				wbuf[2] = 0;	

				dsc.adr = 0x48;
				dsc.wdata = wbuf;
				dsc.wlen = 3;
				dsc.rdata = 0;
				dsc.rlen = 0;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				I2C_AddRequest(&dsc);

				i++;
			};

			break;

		case 2:

			if (dsc.ready)
			{
				wbuf[0] = 4;	
				wbuf[1] = 1;	
				wbuf[2] = 1;	

				dsc.adr = 0x48;
				dsc.wdata = wbuf;
				dsc.wlen = 3;
				dsc.rdata = 0;
				dsc.rlen = 0;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				I2C_AddRequest(&dsc);

				i++;
			};

			break;

		case 3:

			if (dsc.ready)
			{
				wbuf[0] = 5;	
				wbuf[1] = 0;	
				wbuf[2] = 0;	

				dsc.adr = 0x48;
				dsc.wdata = wbuf;
				dsc.wlen = 3;
				dsc.rdata = 0;
				dsc.rlen = 0;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				I2C_AddRequest(&dsc);

				i++;
			};

			break;

		case 4:

			if (dsc.ready)
			{
				curFireVoltage = GetCurFireVoltage();

				u16 t = mv.fireVoltage;

				if (t > curFireVoltage)
				{
					if (correction < 0x3FF)
					{
						correction += 1;
					};
				}
				else if (t < curFireVoltage)
				{
					if (correction > 0)
					{
						correction -= 1;
					};
				};

				dstFV += (i16)mv.fireVoltage - (dstFV+4)/8;

				t = (dstFV+4)/8;

				u32 k = (0x1E00 + correction) >> 3;

				t = (k*t+128) >> 10;

				if (t > 955) t = 955;

				t = ~(((u32)t * (65535*16384/955)) / 16384); 

				wbuf[0] = 8;	
				wbuf[1] = t>>8;
				wbuf[2] = t;

				dsc.adr = 0x48;
				dsc.wdata = wbuf;
				dsc.wlen = 3;
				dsc.rdata = rbuf;
				dsc.rlen = 0;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				I2C_AddRequest(&dsc);

				i++;
			};

			break;

		case 5:

			if (dsc.ready)
			{
				i = 0;
			};

			break;


	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitMainVars()
{
	mv.numDevice		= 0;
	mv.fireVoltage		= 0;
	mv.fireType			= 0;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void LoadVars()
{
	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_CYAN "Load Vars ... ");

	static DSCI2C dsc;
	static u16 romAdr = 0;
	
	byte buf[sizeof(mv)*2+4];

	bool c2 = false;

	bool loadVarsOk = false;

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

	PointerCRC p(buf);

	for (byte i = 0; i < 2; i++)
	{
		p.CRC.w = 0xFFFF;
		p.ReadArrayB(&mv, sizeof(mv));
		p.ReadW();

		if (p.CRC.w == 0) { c2 = true; break; };
	};

	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_WHITE "FRAM I2C - "); SEGGER_RTT_WriteString(0, (c2) ? (RTT_CTRL_TEXT_BRIGHT_GREEN "OK\n") : (RTT_CTRL_TEXT_BRIGHT_RED "ERROR\n"));

	loadVarsOk = c2;

	if (!loadVarsOk)
	{
		InitMainVars();

		svCount = 2;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void SaveVars()
{
	static DSCI2C dsc;
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

			romAdr = ReverseWord(FRAM_I2C_MAINVARS_ADR);

			dsc.wdata = &romAdr;
			dsc.wlen = sizeof(romAdr);
			dsc.wdata2 = buf;
			dsc.wlen2 = p.b-buf;
			dsc.rdata = 0;
			dsc.rlen = 0;
			dsc.adr = 0x50;

			tm.Reset();

			I2C_AddRequest(&dsc);

			i++;

			break;

		case 2:

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
		CALL( UpdateMan(); 				);
		CALL( UpdateTemp()				);
		CALL( SaveVars();				);
		CALL( UpdateHV();				);
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
		CALL( UpdateCom(); 		);
		CALL( UpdateHardware();	);
		CALL( UpdateParams();	);
	};

	i = (i > (__LINE__-S-3)) ? 0 : i;

	#undef CALL
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int main()
{
	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_WHITE "main() start ...\n");

	TM32 tm;

	//__breakpoint(0);

	InitHardware();

	LoadVars();

#ifndef WIN32

	comdsp.Connect(ComPort::ASYNC, TRM_COM_BAUDRATE, TRM_COM_PARITY, TRM_COM_STOPBITS);

#endif

	u32 fc = 0;

	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_WHITE "Main Loop start ...\n");

	while (1)
	{
		Pin_MainLoop_Set();

		UpdateMisc();

		Pin_MainLoop_Clr();

		fc++;

		if (tm.Check(1000))
		{
			fps = fc; fc = 0; 

			//PrepareFire(0, fireFreq, fireAmp, true);
		};
	}; // while (1)
}
