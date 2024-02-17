#include "hardware.h"
#include "ComPort\ComPort.h"
#include "CRC\CRC16.h"
#include "CRC\CRC16_CCIT.h"
#include "G_RCV.h"
#include "list.h"
#include "pack.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//#include <bfrom.h>

static ComPort com;
//static ComPort::WriteBuffer wb;
//
//static byte data[256*48];

//static u16 spd[2][1024*2];
static byte spTime[RCV_FIRE_NUM];
static byte spGain[RCV_FIRE_NUM];
static u16	spLen[RCV_FIRE_NUM];
static u16	spDelay[RCV_FIRE_NUM];

static u16	maxAmp[4];
static u16	power[4];


//static u16 spd2[512*2];
//
//static i16 ch1[512];
//static i16 ch2[512];
//static i16 ch3[512];
//static i16 ch4[512];

//static bool ready1 = false, ready2 = false;

//static u32 CRCOK = 0;
//static u32 CRCER = 0;

static byte sampleTime[RCV_FIRE_NUM]	= { 10	};
static byte gain[RCV_FIRE_NUM]			= { 0	};
static u16 sampleLen[RCV_FIRE_NUM]		= { 512	};
static u16 sampleDelay[RCV_FIRE_NUM]	= { 200	};

//static byte netAdr = 1;

static U32u fadc = 0;

static byte fireN = 0;

static u16 flashCRC = 0;
static u32 flashLen = 0;
static bool flashOK = false;
static bool flashChecked = false;

static u16 packType = 0;

static u16 lastErasedBlock = ~0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct DSCRSP02
{
	DSCRSP02 *next;

	u16 len;

	union 
	{
		RspRcv02 r02;
		u16 data[sizeof(RspRcv02)/2];
	};
};

static DSCRSP02 dscRsp02[1];

#pragma instantiate List<DSCRSP02>
static List<DSCRSP02> freeRSP02;
static List<DSCRSP02> readyRSP02;

static DSCRSP02 *wrDscRSP02 = 0;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

union RequestUnion { ReqRcv01	req01; ReqRcv02	req02; ReqRcv03	req03; ReqRcv04	req04; };
				
//static RspRcv02 rsp02;

static byte rspBuf[64];

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static byte build_date[sizeof(RequestUnion)+32] = "\n" "G26X_1_DSP" "\n" __DATE__ "\n" __TIME__ "\n";

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestFunc01(byte *data, u16 len, ComPort::WriteBuffer *wb)
{
	ReqRcv01::Req &req = *((ReqRcv01::Req*)data);
	RspRcv01 &rsp = *((RspRcv01*)rspBuf);

	byte n = req.n;
	if (n > RCV_FIRE_NUM) n = RCV_FIRE_NUM;

	DSCSPORT *dsc = AllocDscSPORT();

	if (dsc != 0)
	{
		dsc->vectorCount	= req.fc;
		dsc->fireN			= n;
		dsc->st				= sampleTime[n];
		dsc->gain			= gain[n];
		dsc->sl				= sampleLen[n];

		u16 delay = sampleDelay[n] / sampleTime[n];

		dsc->sd	= delay * sampleTime[n];

		SyncReadSPORT(dsc);
	};

	fireN = n;
	//sportState = 0;

	if (req.adr == 0) return  false;

	rsp.adr = req.adr;
	rsp.func = req.func;
	rsp.crc = GetCRC16(&rsp, sizeof(rsp)-sizeof(rsp.crc));

	wb->data = &rsp;
	wb->len = sizeof(rsp);

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestFunc02(byte *data, u16 len, ComPort::WriteBuffer *wb)
{
	static u16 buf[2];

	ReqRcv02::Req &req = *((ReqRcv02::Req*)data);

	if (req.n > 2)
	{
		return false;
	};

	if (req.adr == 0) return  false;

	byte n = req.n;
	
	if (wrDscRSP02 == 0) wrDscRSP02 = readyRSP02.Get();

	if (wrDscRSP02 != 0)
	{
		RspRcv02 &rsp = wrDscRSP02->r02;

		wb->data = &rsp;
		wb->len = wrDscRSP02->len;
	}
	else
	{
		buf[0] = 0xAA30 + (n<<4) + req.adr-1;
		buf[1] = GetCRC16_CCIT(buf, 2);

		wb->data = buf;
		wb->len = sizeof(buf);
	};

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestFunc03(byte *data, u16 len, ComPort::WriteBuffer *wb)
{
	ReqRcv03::Req &req	= *((ReqRcv03::Req*)data);
	RspRcv03 &rsp		= *((RspRcv03*)rspBuf);

	sampleTime[0] = MAX(req.st[0], 2);
	sampleTime[1] = MAX(req.st[1], 2);
	sampleTime[2] = MAX(req.st[2], 2);

	sampleLen[0] = LIM(req.sl[0], 16, 1024);
	sampleLen[1] = LIM(req.sl[1], 16, 1024);
	sampleLen[2] = LIM(req.sl[2], 16, 1024);

	//if (sampleLen[0] > 1024) { sampleLen[0] = 1024; };
	//if (sampleLen[1] > 1024) { sampleLen[1] = 1024; };
	//if (sampleLen[2] > 1024) { sampleLen[2] = 1024; };

	sampleDelay[0] = req.sd[0];
	sampleDelay[1] = req.sd[1];
	sampleDelay[2] = req.sd[2];

	if (req.adr == 0) return  false;

	rsp.adr		= req.adr;
	rsp.func	= req.func;
	rsp.crc		= GetCRC16(&rsp, sizeof(rsp)-sizeof(rsp.crc));

	wb->data = &rsp;
	wb->len = sizeof(rsp);

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestFunc04(byte *data, u16 len, ComPort::WriteBuffer *wb)
{
	ReqRcv04::Req	&req = *((ReqRcv04::Req*)data);
	RspRcv04		&rsp = *((RspRcv04*)rspBuf);

	gain[0] = req.ka[0];
	gain[1] = req.ka[1];
	gain[2] = req.ka[2];

	if (req.adr == 0) return  false;

	rsp.adr = req.adr;
	rsp.func = 4;

	rsp.maxAmp[0] = maxAmp[0];
	rsp.maxAmp[1] = maxAmp[1];
	rsp.maxAmp[2] = maxAmp[2];
	rsp.maxAmp[3] = maxAmp[3];

	rsp.power[0] = power[0];
	rsp.power[1] = power[1];
	rsp.power[2] = power[2];
	rsp.power[3] = power[3];

	rsp.crc = GetCRC16(&rsp, sizeof(rsp)-sizeof(rsp.crc));

	wb->data = &rsp;
	wb->len = sizeof(rsp);

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

typedef bool (*REQF)(byte *req, u16 len, ComPort::WriteBuffer *wb);

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static REQF listReq[4] = { RequestFunc01, RequestFunc02, RequestFunc03, RequestFunc04 };

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestFunc(const ComPort::ReadBuffer *rb, ComPort::WriteBuffer *wb)
{
	//static NewRequest nulReq;
	static const byte fl[4] = { sizeof(ReqRcv01::r[0])-1, sizeof(ReqRcv02::r[0])-1, sizeof(ReqRcv03::r[0])-1, sizeof(ReqRcv04::r[0])-1 };

	if (rb == 0 || rb->len < 4) return false;

	bool result = false;

	u16 rlen = rb->len;

	byte *p = (byte*)rb->data; 

	while(rlen > 3)
	{
		byte len = p[0];
		byte adr = p[1];
		byte func = p[2]-1;

		if (func < 4 && len == fl[func] && len < rlen && GetCRC16(p+1, len) == 0)
		{
			if (adr != 0 && adr != GetNetAdr()) return false;

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

static void UpdateBlackFin()
{
	static byte i = 0;
	static ComPort::WriteBuffer wb;
	static ComPort::ReadBuffer rb;

	switch(i)
	{
		case 0:

			rb.data = build_date;
			rb.maxLen = sizeof(build_date);
			com.Read(&rb, ~0, US2COM(45+62500000/RCV_COM_BAUDRATE));
			i++;

			break;

		case 1:

			if (!com.Update())
			{
				if (rb.recieved && rb.len > 0)
				{
					if (RequestFunc(&rb, &wb))
					{
						com.Write(&wb);
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

			if (!com.Update())
			{
				if (wrDscRSP02 != 0) freeRSP02.Add(wrDscRSP02), wrDscRSP02 = 0;

				i = 0;
			};

			break;
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateSport()
{
	static byte sportState = 0;
	//static byte n = 0;
	//static byte chnl = 0;
	//static u16 len = 0;
	//static byte st = 0;
	//static byte sg = 0;
	//static u16 sd = 0;

	static DSCSPORT *dsc = 0;
	static DSCRSP02 *prsp = 0;

	//RspRcv02 &rsp = rsp02;

	switch(sportState)
	{
		case 0:
			
			dsc = GetDscSPORT();

			if (dsc != 0)
			{
				//n = fireN;
				//len = spLen[n];
				//st = spTime[n];
				//sg = spGain[n];
				//sd = spDelay[n];

				sportState++;
			};

			break;

		case 1:

			prsp = freeRSP02.Get();

			if (prsp != 0)
			{
				RspRcv02 &rsp = prsp->r02;

				rsp.hdr.rw			= 0xAA30 + (dsc->fireN<<4) + GetNetAdr()-1;
				rsp.hdr.cnt			= dsc->vectorCount;
				rsp.hdr.gain		= dsc->gain; 
				rsp.hdr.st			= dsc->st; 
				rsp.hdr.len			= dsc->sl; 
				rsp.hdr.delay		= dsc->sd;
				rsp.hdr.packType	= packType;

				u16 *p1 = rsp.data + rsp.hdr.len*0;
				u16 *p2 = rsp.data + rsp.hdr.len*1;
				u16 *p3 = rsp.data + rsp.hdr.len*2;
				u16 *p4 = rsp.data + rsp.hdr.len*3;

				u16 max[4] = {0, 0, 0, 0};
				u16 min[4] = {65535, 65535, 65535, 65535 };

				u32 pow[4] = {0, 0, 0, 0};

				u16 t;

				i16 x;

				for (u16 i = 0; i < rsp.hdr.len; i++)
				{
					t = dsc->spd[0][i*2+0];

					if (t > max[0]) { max[0] = t; };
					if (t < min[0]) { min[0] = t; };

					*p1++ = x = t - 0x8000;

					pow[0] += (x > 0) ? x : (-x);

					t = dsc->spd[0][i*2+1];

					if (t > max[1]) { max[1] = t; };
					if (t < min[1]) { min[1] = t; };

					*p2++ = x = t - 0x8000;

					pow[1] += (x > 0) ? x : (-x);

					t = dsc->spd[1][i*2+0];

					if (t > max[2]) { max[2] = t; };
					if (t < min[2]) { min[2] = t; };

					*p3++ = x = t - 0x8000;

					pow[2] += (x > 0) ? x : (-x);

					t = dsc->spd[1][i*2+1];

					if (t > max[3]) { max[3] = t; };
					if (t < min[3]) { min[3] = t; };

					*p4++ = x = t - 0x8000;

					pow[3] += (x > 0) ? x : (-x);
				};

				for (byte i = 0; i < 4; i++)
				{
					maxAmp[i] = max[i] - min[i];
					power[i] = (rsp.hdr.len > 0) ? (pow[i] / rsp.hdr.len) : 0;
				};
			
				FreeDscSPORT(dsc);

				dsc = 0;

				prsp->len = sizeof(rsp.hdr) + rsp.hdr.len*8;

				sportState = (rsp.hdr.packType == PACK_NO) ? 3 : (sportState+1);
			};

			break;

		case 2:
		{
			RspRcv02 &rsp = prsp->r02;

			if (rsp.hdr.packType == PACK_ULAW16)
			{
				Pack_uLaw_16Bit((i16*)rsp.data, (byte*)rsp.data, rsp.hdr.len*4);
				rsp.hdr.packLen = rsp.hdr.len*2;
				prsp->len = sizeof(rsp.hdr) + rsp.hdr.len*4;
			}
			else if (rsp.hdr.packType == PACK_ADPCMIMA)
			{
				i16	*src	= (i16*)rsp.data;
				byte *dst	= (byte*)rsp.data;

				Pack_ADPCMIMA(src, dst, rsp.hdr.len); src += rsp.hdr.len; dst += rsp.hdr.len;
				Pack_ADPCMIMA(src, dst, rsp.hdr.len); src += rsp.hdr.len; dst += rsp.hdr.len;
				Pack_ADPCMIMA(src, dst, rsp.hdr.len); src += rsp.hdr.len; dst += rsp.hdr.len;
				Pack_ADPCMIMA(src, dst, rsp.hdr.len); 

				rsp.hdr.packLen = rsp.hdr.len;
				prsp->len = sizeof(rsp.hdr) + rsp.hdr.len*2;
			}
			else if (rsp.hdr.packType >= PACK_DCT0)
			{
				u16 OVRLAP = (rsp.hdr.packType > PACK_DCT0) ? 7 : 3;
				u16 shift = 4 - (rsp.hdr.packType - PACK_DCT0);

				i16	*src	= (i16*)rsp.data;
				byte *dst	= (byte*)rsp.data;
				u16 packedLen = 0;
				u16 pl = 0;

				u16 sl =	Pack_FDCT(src, dst, rsp.hdr.len, shift, OVRLAP, &packedLen); src += rsp.hdr.len; dst += packedLen; pl += packedLen;
							Pack_FDCT(src, dst, rsp.hdr.len, shift, OVRLAP, &packedLen); src += rsp.hdr.len; dst += packedLen; pl += packedLen;
							Pack_FDCT(src, dst, rsp.hdr.len, shift, OVRLAP, &packedLen); src += rsp.hdr.len; dst += packedLen; pl += packedLen;
							Pack_FDCT(src, dst, rsp.hdr.len, shift, OVRLAP, &packedLen); pl += packedLen;

				rsp.hdr.len = sl;

				rsp.hdr.packLen = pl/2;
				prsp->len = sizeof(rsp.hdr) + rsp.hdr.packLen*2;
			};

			sportState += 1;

			break;
		};

		case 3:
		{
			RspRcv02 &rsp = prsp->r02;

			prsp->data[prsp->len/2] = GetCRC16_CCIT(&prsp->r02, prsp->len);
			prsp->len += 2;

			sportState = 0;

			break;
		};
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static void UpdateNetAdr()
//{
//	netAdr = (GetADC() / 398) + 1;
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static void InitNetAdr()
//{
//	u32 t = GetRTT();
//
//	while ((GetRTT()-t) < 10000000)
//	{
//		UpdateHardware();
//	};
//
////	UpdateNetAdr();
//}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void main( void )
{
//	static byte s = 0;

//	static u32 pt = 0;

	InitHardware();

	Pack_Init();

	com.Connect(RCV_COM_BAUDRATE, RCV_COM_PARITY);

	for (u16 i = 0; i < ArraySize(dscRsp02); i++)
	{
		DSCRSP02 &dsc = dscRsp02[i];

		dsc.next = 0;

		freeRSP02.Add(&dsc);
	};


//	InitNetAdr();

	while (1)
	{
		MAIN_LOOP_PIN_SET();

		static byte i = 0;

		#define CALL(p) case (__LINE__-S): p; break;

		enum C { S = (__LINE__+3) };
		switch(i++)
		{
			CALL( UpdateBlackFin()	);
			CALL( UpdateHardware()	);
			CALL( UpdateSport()		);
		};

		i = (i > (__LINE__-S-3)) ? 0 : i;

		#undef CALL

		MAIN_LOOP_PIN_CLR();
	};

//	return 0;
}
