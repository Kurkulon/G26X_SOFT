#include "hardware.h"
#include "ComPort.h"
#include "CRC16.h"

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

struct NewRequest
{
	byte len;
	byte adr;
	byte func;
	
	union
	{
		struct  { byte n; u16 vc; word crc; } f1;  // старт оцифровки
		struct  { byte n; byte chnl; word crc; } f2;  // чтение вектора
		struct  { u16 st[3]; u16 sl[3]; u16 sd[3]; word crc; } f3;  // установка шага и длины оцифровки вектора
		struct  { byte ka[3]; word crc; } f4;  // установка коэффициента усиления
		struct  { word crc; } f5;  // запрос контрольной суммы и длины программы во флэш-памяти
		struct  { u16 stAdr; u16 len; word crc; byte data[258]; } f6;  // запись страницы во флэш
		struct  { word crc; } f7;  // перезагрузить блэкфин
	};
};

#pragma pack()

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct Response
{
	byte adr;
	byte func;
	
	union
	{
		struct  { word crc; } f1;  // старт оцифровки
		struct  { word crc; } f3;  // установка периода дискретизации вектора и коэффициента усиления
		struct  { u16 maxAmp[4]; u16 power[4]; word crc; } f4;  // старт оцифровки с установкой периода дискретизации вектора и коэффициента усиления
		struct  { u16 flashLen; u16 flashCRC; word crc; } f5;  // запрос контрольной суммы и длины программы во флэш-памяти
		struct  { u16 res; word crc; } f6;  // запись страницы во флэш
	};
};


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#pragma pack(1)

struct Response02 { u16 rw; u32 cnt; u16 gain; u16 st; u16 len; u16 delay; u16 data[1024*4]; word crc; }; // чтение вектора

#pragma pack()

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static Response02 rsp02;

static byte rspBuf[64];

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestFunc01(const NewRequest *req, ComPort::WriteBuffer *wb)
{
//	const Request *req = (Request*)rb->data;
	Response &rsp = *((Response*)rspBuf);

	byte n = req->f1.n;
	if (n > 2) n = 2;

	vectorCount = req->f1.vc;

	spTime[n] = sampleTime[n];
	spGain[n] = gain[n];
	spLen[n] = sampleLen[n];
	spDelay[n] = sampleDelay[n];

	SetGain(spGain[n]);
	SyncReadSPORT(spd[0], spd[1], spLen[n]*4, spLen[n]*4, spTime[n]-1, &ready1, &ready2);
	DisableADC();

	fireN = n;
	sportState = 0;

	if (req->adr == 0) return  false;

	rsp.adr = GetNetAdr();
	rsp.func = 1;
	rsp.f1.crc = GetCRC16(&rsp, 2);

	wb->data = &rsp;
	wb->len = sizeof(rsp.f1)+2;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestFunc02(const NewRequest *req, ComPort::WriteBuffer *wb)
{
//	const Request *req = (Request*)rb->data;

	if (req->f2.n > 2)
	{
		return false;
	};

	if (req->adr == 0) return  false;

	byte n = req->f2.n;
//	byte chnl = (req->f2.chnl)&3;

	byte ch = 0;
	byte cl = 0;
	u16 len = sampleLen[n];

	Response02 &rsp = rsp02;

	if (rsp.rw == 0)
	{
		rsp.rw = 0xAA30 + (n<<4) + req->adr-1;
		rsp.cnt = vectorCount++;
		rsp.gain = gain[n]; 
		rsp.st = sampleTime[n]; 
		rsp.len = len; 
		rsp.delay = sampleDelay[n];

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

		rsp.data[len*4] = GetCRC16(&rsp, sizeof(rsp) - sizeof(rsp.data) - sizeof(rsp.crc) + len*8);
	};

	//rsp.adr = netAdr;
	//rsp.func = 2;

	wb->data = &rsp;
	wb->len = sizeof(rsp) - sizeof(rsp.data) + rsp.len*8;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestFunc03(const NewRequest *req, ComPort::WriteBuffer *wb)
{
//	const Request *req = (Request*)rb->data;
	Response &rsp = *((Response*)rspBuf);

	sampleTime[0] = req->f3.st[0];
	sampleTime[1] = req->f3.st[1];
	sampleTime[2] = req->f3.st[2];

	sampleLen[0] = req->f3.sl[0];
	sampleLen[1] = req->f3.sl[1];
	sampleLen[2] = req->f3.sl[2];

	if (sampleLen[0] > 1024) { sampleLen[0] = 1024; };
	if (sampleLen[1] > 1024) { sampleLen[1] = 1024; };
	if (sampleLen[2] > 1024) { sampleLen[2] = 1024; };

	sampleDelay[0] = req->f3.sd[0];
	sampleDelay[1] = req->f3.sd[1];
	sampleDelay[2] = req->f3.sd[2];

	if (req->adr == 0) return  false;

	rsp.adr = req->adr;
	rsp.func = req->func;
	rsp.f3.crc = GetCRC16(&rsp, 2);

	wb->data = &rsp;
	wb->len = sizeof(rsp.f3)+2;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestFunc04(const NewRequest *req, ComPort::WriteBuffer *wb)
{
//	const Request *req = (Request*)rb->data;
	Response &rsp = *((Response*)rspBuf);

	gain[0] = req->f4.ka[0];
	gain[1] = req->f4.ka[1];
	gain[2] = req->f4.ka[2];

	if (req->adr == 0) return  false;

	rsp.adr = req->adr;
	rsp.func = 4;

	rsp.f4.maxAmp[0] = maxAmp[0];
	rsp.f4.maxAmp[1] = maxAmp[1];
	rsp.f4.maxAmp[2] = maxAmp[2];
	rsp.f4.maxAmp[3] = maxAmp[3];

	rsp.f4.power[0] = power[0];
	rsp.f4.power[1] = power[1];
	rsp.f4.power[2] = power[2];
	rsp.f4.power[3] = power[3];

	rsp.f4.crc = GetCRC16(&rsp, sizeof(rsp.f4));

	wb->data = &rsp;
	wb->len = sizeof(rsp.f4) + 2;

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestFunc(const ComPort::ReadBuffer *rb, ComPort::WriteBuffer *wb)
{
	static NewRequest nulReq;
	static const byte fl[7] = { sizeof(nulReq.f1)+2, sizeof(nulReq.f2)+2, sizeof(nulReq.f3)+2, sizeof(nulReq.f4)+2, sizeof(nulReq.f5)+2, sizeof(nulReq.f6)+2-sizeof(nulReq.f6.data), sizeof(nulReq.f7)+2 };

	if (rb == 0 || rb->len < 2) return false;

	bool result = false;

	u16 rlen = rb->len;

	byte *p = (byte*)rb->data;

	while(rlen > 5)
	{
		byte len = p[0];
		byte func = p[2]-1;

		if (func < 7 && len == fl[func] && len < rlen && GetCRC16(p+1, len) == 0)
		{
			NewRequest *req = (NewRequest*)p;

			if (req->adr != 0 && req->adr != GetNetAdr()) return false;

			switch(req->func)
			{
				case 1: result = RequestFunc01 (req, wb); break;
				case 2: result = RequestFunc02 (req, wb); break;
				case 3: result = RequestFunc03 (req, wb); break;
				case 4: result = RequestFunc04 (req, wb); break;
			};

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
	static byte buf[sizeof(NewRequest)+10];

	switch(i)
	{
		case 0:

			rb.data = buf;
			rb.maxLen = sizeof(buf);
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

	Response02 &rsp = rsp02;

	switch(sportState)
	{
		case 0:
			
			if (ready1 && ready2)
			{
				EnableADC();

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

			rsp.rw = 0xAA30 + (n<<4) + GetNetAdr()-1;
			rsp.cnt = vectorCount++;
			rsp.gain = sg; 
			rsp.st = st; 
			rsp.len = len; 
			rsp.delay = spDelay[n];

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

	com.Connect(6250000, 0);

//	InitNetAdr();

	while (1)
	{
		*pPORTFIO_TOGGLE = 1<<5;

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

		*pPORTFIO_TOGGLE = 1<<5;
	};

//	return 0;
}
