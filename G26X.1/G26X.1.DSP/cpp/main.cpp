#include "hardware.h"
#include "ComPort\ComPort.h"
#include "CRC\CRC16.h"
#include "G_RCV.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//#include <bfrom.h>

static ComPort com;
//static ComPort::WriteBuffer wb;
//
//static byte data[256*48];

static u16 spd[2][1024*2];
static byte spTime[3];
static byte spGain[3];
static u16	spLen[3];
static u16	spDelay[3];

static u16	maxAmp[4];
static u16	power[4];


//static u16 spd2[512*2];
//
//static i16 ch1[512];
//static i16 ch2[512];
//static i16 ch3[512];
//static i16 ch4[512];

static bool ready1 = false, ready2 = false;

//static u32 CRCOK = 0;
//static u32 CRCER = 0;

static byte sampleTime[3] = { 5, 10, 10};
static byte gain[3] = { 0, 1, 1 };
static u16 sampleLen[3] = {1024, 1024, 1024};
static u16 sampleDelay[3] = { 200, 500, 500};


//static byte netAdr = 1;

static U32u fadc = 0;

static byte sportState = 0;
static byte fireN = 0;

static u16 flashCRC = 0;
static u32 flashLen = 0;
static bool flashOK = false;
static bool flashChecked = false;

static u16 vectorCount = 0;

static u16 lastErasedBlock = ~0;

static void CheckFlash();


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//struct Request
//{
//	byte adr;
//	byte func;
//	
//	union
//	{
//		struct  { byte n; word crc; } f1;  // старт оцифровки
//		struct  { byte n; byte chnl; word crc; } f2;  // чтение вектора
//		struct  { u16 st[3]; u16 sl[3]; u16 sd[3]; word crc; } f3;  // установка периода дискретизации вектора и коэффициента усиления
//		struct  { byte ka[3]; word crc; } f4;  // старт оцифровки с установкой периода дискретизации вектора и коэффициента усиления
//
////		struct  { byte subfunc; ka[3]; word crc; } fAD;  // Перепрошивка флэшки
//	};
//};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma pack(1)

//struct NewRequest
//{
//	byte len;
//	byte adr;
//	byte func;
//	
//	union
//	{
//		struct  { byte n; u16 vc; word crc; } f1;  // старт оцифровки
//		struct  { byte n; byte chnl; word crc; } f2;  // чтение вектора
//		struct  { u16 st[3]; u16 sl[3]; u16 sd[3]; word crc; } f3;  // установка шага и длины оцифровки вектора
//		struct  { byte ka[3]; word crc; } f4;  // установка коэффициента усиления
//	};
//};

#pragma pack()

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//struct Response
//{
//	byte adr;
//	byte func;
//	
//	union
//	{
//		struct  { word crc; } f1;  // старт оцифровки
//		struct  { word crc; } f3;  // установка периода дискретизации вектора и коэффициента усиления
//		struct  { u16 maxAmp[4]; u16 power[4]; word crc; } f4;  // старт оцифровки с установкой периода дискретизации вектора и коэффициента усиления
//	};
//};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma pack(1)

//struct Response02 { u16 rw; u32 cnt; u16 gain; u16 st; u16 len; u16 delay; u16 data[1024*4]; word crc; }; // чтение вектора

#pragma pack()

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

union RequestUnion { ReqRcv01	req01; ReqRcv02	req02; ReqRcv03	req03; ReqRcv04	req04; };
				
static RspRcv02 rsp02;

static byte rspBuf[64];

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static byte build_date[sizeof(RequestUnion)+32] = "\n" "G26X_1_DSP" "\n" __DATE__ "\n" __TIME__ "\n";

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestFunc01(byte *data, u16 len, ComPort::WriteBuffer *wb)
{
	ReqRcv01::Req &req = *((ReqRcv01::Req*)data);
	RspRcv01 &rsp = *((RspRcv01*)rspBuf);

	byte n = req.n;
	if (n > 2) n = 2;

	vectorCount = req.fc;

	spTime[n]	= sampleTime[n];
	spGain[n]	= gain[n];
	spLen[n]	= sampleLen[n];
	spDelay[n]	= sampleDelay[n];

	//SetGain(spGain[n]);
	SyncReadSPORT(spd[0], spd[1], spLen[n]*4, spLen[n]*4, spTime[n]-1, &ready1, &ready2);
	//DisableADC();

	fireN = n;
	sportState = 0;

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
	ReqRcv02::Req &req = *((ReqRcv02::Req*)data);

	if (req.n > 2)
	{
		return false;
	};

	if (req.adr == 0) return  false;

	byte n = req.n;
//	byte chnl = (req->f2.chnl)&3;

	byte ch = 0;
	byte cl = 0;
	
	len = sampleLen[n];

	RspRcv02 &rsp = rsp02;

	u16 rsplen = sizeof(rsp) - sizeof(rsp.data) - sizeof(rsp.crc) + len*8;

	if (rsp.hdr.rw == 0)
	{
		rsp.hdr.rw = 0xAA30 + (n<<4) + req.adr-1;
		rsp.hdr.cnt		= vectorCount++;
		rsp.hdr.gain	= gain[n]; 
		rsp.hdr.st		= sampleTime[n]; 
		rsp.hdr.len		= len; 
		rsp.hdr.delay	= sampleDelay[n];

		u16 *p1 = rsp.data+len*0;
		u16 *p2 = rsp.data+len*1;
		u16 *p3 = rsp.data+len*2;
		u16 *p4 = rsp.data+len*3;

		for (u16 i = 0; i < len; i++)
		{
			*p1++ = spd[0][i*2+0] - 0x8000;
			*p2++ = spd[0][i*2+1] - 0x8000;
			*p3++ = spd[1][i*2+0] - 0x8000;
			*p4++ = spd[1][i*2+1] - 0x8000;
		};

		rsp.data[len*4] = GetCRC16(&rsp, rsplen);
	};

	//rsp.adr = netAdr;
	//rsp.func = 2;

	wb->data = &rsp;
	wb->len = rsplen + 2;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestFunc03(byte *data, u16 len, ComPort::WriteBuffer *wb)
{
	ReqRcv03::Req &req	= *((ReqRcv03::Req*)data);
	RspRcv03 &rsp		= *((RspRcv03*)rspBuf);

	sampleTime[0] = req.st[0];
	sampleTime[1] = req.st[1];
	sampleTime[2] = req.st[2];

	sampleLen[0] = req.sl[0];
	sampleLen[1] = req.sl[1];
	sampleLen[2] = req.sl[2];

	if (sampleLen[0] > 1024) { sampleLen[0] = 1024; };
	if (sampleLen[1] > 1024) { sampleLen[1] = 1024; };
	if (sampleLen[2] > 1024) { sampleLen[2] = 1024; };

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
			com.Read(&rb, (u32)-1, US2SCLK(50));
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
				i = 0;
			};

			break;
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateSport()
{
	static byte n = 0;
	static byte chnl = 0;
	static u16 len = 0;
	static byte st = 0;
	static byte sg = 0;
	static u16 sd = 0;

	RspRcv02 &rsp = rsp02;

	switch(sportState)
	{
		case 0:
			
			if (ready1 && ready2)
			{
				//EnableADC();

				n = fireN;
				chnl = 0;
				len = spLen[n];
				st = spTime[n];
				sg = spGain[n];
				sd = spDelay[n];

				sportState++;
			};

			break;

		case 1:

			rsp.hdr.rw = 0xAA30 + (n<<4) + GetNetAdr()-1;
			rsp.hdr.cnt = vectorCount++;
			rsp.hdr.gain = sg; 
			rsp.hdr.st = st; 
			rsp.hdr.len = len; 
			rsp.hdr.delay = spDelay[n];

			*pPORTFIO_SET = 1<<8;

			{
				spd[0][0] = spd[0][2];
				spd[0][1] =	spd[0][3];
				spd[1][0] =	spd[1][2];
				spd[1][1] =	spd[1][3];

				u16 *p1 = rsp.data+len*0;
				u16 *p2 = rsp.data+len*1;
				u16 *p3 = rsp.data+len*2;
				u16 *p4 = rsp.data+len*3;

				u16 max[4] = {0, 0, 0, 0};
				u16 min[4] = {65535, 65535, 65535, 65535 };

				u32 pow[4] = {0, 0, 0, 0};

				u16 t;

				i16 x;

				for (u16 i = 0; i < len; i++)
				{
					t = spd[0][i*2+0];

					if (t > max[0]) { max[0] = t; };
					if (t < min[0]) { min[0] = t; };

					*p1++ = x = t - 0x8000;

					pow[0] += (x > 0) ? x : (-x);

					t = spd[0][i*2+1];

					if (t > max[1]) { max[1] = t; };
					if (t < min[1]) { min[1] = t; };

					*p2++ = x = t - 0x8000;

					pow[1] += (x > 0) ? x : (-x);

					t = spd[1][i*2+0];

					if (t > max[2]) { max[2] = t; };
					if (t < min[2]) { min[2] = t; };

					*p3++ = x = t - 0x8000;

					pow[2] += (x > 0) ? x : (-x);

					t = spd[1][i*2+1];

					if (t > max[3]) { max[3] = t; };
					if (t < min[3]) { min[3] = t; };

					*p4++ = x = t - 0x8000;

					pow[3] += (x > 0) ? x : (-x);
				};

				for (byte i = 0; i < 4; i++)
				{
					maxAmp[i] = max[i] - min[i];
					power[i] = (len > 0) ? (pow[i] / len) : 0;
				};
			};

			*pPORTFIO_CLEAR = 1<<8;

			sportState++;

			break;

		case 2:

			rsp.data[len*4] = GetCRC16(&rsp, sizeof(rsp) - sizeof(rsp.data) - sizeof(rsp.crc) + len*8);

			sportState++;

			break;

		case 3:

			break;

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

	com.Connect(RCV_COM_BAUDRATE, RCV_COM_PARITY);

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
