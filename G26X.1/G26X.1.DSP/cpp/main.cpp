#include "hardware.h"
#include "ComPort\ComPort.h"
#include "CRC\CRC16.h"
#include "CRC\CRC16_CCIT.h"
#include "FLASH\at25df021.h"
#include "BOOT\boot_req.h"
//#include "G_RCV.h"
#include "list.h"
#include "pack.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
static void CheckFlash();

enum { VERSION = 0x101 };

static u16 numDevice = 0;
static u16 numDevValid = 0;
//static u16 temp = 0;
static u16 flashStatus = 0;

static const u16 bootReqWord = RCV_BOOT_REQ_WORD;
static const u16 bootReqMask = RCV_BOOT_REQ_MASK;

//#include <bfrom.h>

static ComPort com;
//static ComPort::WriteBuffer wb;
//
//static byte data[256*48];

//static u16 spd[2][1024*2];
//static byte spTime[RCV_FIRE_NUM];
//static byte spGain[RCV_FIRE_NUM];
//static u16	spLen[RCV_FIRE_NUM];
//static u16	spDelay[RCV_FIRE_NUM];

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

//static byte sampleTime[RCV_FIRE_NUM]	= { 10	};
static byte ngain[RCV_FIRE_NUM]			= { 0	};
//static u16 sampleLen[RCV_FIRE_NUM]		= { 512	};
//static u16 sampleDelay[RCV_FIRE_NUM]	= { 200	};

//static byte netAdr = 1;

//static U32u fadc = 0;

static byte fireN = 0;

static u16 flashCRC = 0;
static u32 flashLen = 0;
static bool flashOK = false;
static bool flashChecked = false;
static bool flashCRCOK = false;
static bool cmdSaveParams = false;
static bool cmdReboot = false;

static u16 packType = 0;

static u16 lastErasedBlock = ~0;

static u32 curWriteReqAdr = 0;

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

//union RequestUnion { ReqRcv01	req01; ReqRcv02	req02; ReqRcv03	req03; ReqRcv04	req04; };
				
//static RspRcv02 rsp02;

static byte rspBuf[64] = "\n" "G26X_1_DSP" "\n" __DATE__ "\n" __TIME__ "\n";

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static byte build_date[sizeof(RequestUnion)+32] = "\n" "G26X_1_DSP" "\n" __DATE__ "\n" __TIME__ "\n";

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

inline void SaveParams()
{
	cmdSaveParams = true;
}

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
		if (wrDscRSP02 != 0) freeRSP02.Add(wrDscRSP02), wrDscRSP02 = 0;

		dsc->vectorCount	= req.fc;
		dsc->fireN			= n;
		dsc->gain			= ngain[n];
		dsc->next_fireN		= req.next_n;
		dsc->next_gain		= req.next_gain;
		dsc->sl				= LIM(req.sl, 16, 1024);
		dsc->st				= MAX(req.st, 2);

		#ifdef RCV_WAVEPACK
		dsc->packType		= req.packType;
		#endif

		u16 delay = req.sd / dsc->st;

		dsc->sd	= delay * dsc->st;

		SyncReadSPORT(dsc, delay);
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

	if (req.n > RCV_FIRE_NUM)
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
		buf[1] = GetCRC16_CCIT_refl(buf, 2);

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

	for (byte i = 0; i < ArraySize(ngain); i++)	ngain[i] = req.gain[i];

	if (req.numDevValid) numDevice = req.numDev, numDevValid = req.numDevValid;

	if (req.adr == 0) return  false;

	rsp.adr			= req.adr;
	rsp.func		= req.func;
	rsp.temp		= GetTemp();		// температура
	rsp.numdev		= numDevice;		// номер модуля приёмников
	rsp.verdev		= VERSION; 			// версия ПО модуля приёмников
	rsp.numDevValid	= numDevValid;		// если не ноль, numDev считан из flash правильно или установлен запросом
	rsp.flashStatus	= flashStatus; 		// бит 0 - запись в процессе, бит 1 - запись ОК, бит 2 - ошибка записи

	rsp.crc			= GetCRC16(&rsp, sizeof(rsp)-sizeof(rsp.crc));

	wb->data = &rsp;
	wb->len = sizeof(rsp);

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestFunc04(byte *data, u16 len, ComPort::WriteBuffer *wb)
{
	ReqRcv04::Req	&req = *((ReqRcv04::Req*)data);
	RspRcv04		&rsp = *((RspRcv04*)rspBuf);

	if (req.saveParams) SaveParams();

	if (req.adr == 0) return  false;

	rsp.adr			= req.adr;
	rsp.func		= req.func;
	rsp.temp		= GetTemp();		// температура
	rsp.numdev		= numDevice;		// номер модуля приёмников
	rsp.verdev		= VERSION; 			// версия ПО модуля приёмников
	rsp.numDevValid	= numDevValid;		// если не ноль, numDev считан из flash правильно или установлен запросом
	rsp.flashStatus	= flashStatus; 		// бит 0 - запись в процессе, бит 1 - запись ОК, бит 2 - ошибка записи

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

static void Reboot()
{
	if (!flashChecked)
	{
		CheckFlash();

		if (flashOK && flashCRCOK) bfrom_SysControl(SYSCTRL_SOFTRESET, NULL, NULL);
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestBoot_00(ReqAT25 *r, ComPort::WriteBuffer *wb)
{
	const BootReqV1 &req = *((BootReqV1*)(r->GetDataPtr()));
	static BootRspV1 rsp;

	if (req.F0.adr == 0 || r->len < sizeof(req.F0)) return  false;

	rsp.F0.adr		= req.F0.adr;
	rsp.F0.rw		= req.F0.rw;
	rsp.F0.ver		= rsp.VERSION;
	rsp.F0.maxFunc	= rsp.FUNC_MAX;
	rsp.F0.guid		= RCV_BOOT_SGUID;
	rsp.F0.startAdr = FLASH_START_ADR;
	rsp.F0.pageLen	= FLASH_PAGE_SIZE;

	rsp.F0.crc = GetCRC16(&rsp.F0, sizeof(rsp.F0)-sizeof(rsp.F0.crc));

	wb->data = &rsp.F0;
	wb->len = sizeof(rsp.F0);

	FreeReqAT25(r);

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestBoot_01(ReqAT25 *r, ComPort::WriteBuffer *wb)
{
	const BootReqV1 &req = *((BootReqV1*)(r->GetDataPtr()));
	static BootRspV1 rsp;

	if (req.F1.adr == 0 || r->len < sizeof(req.F1)) return  false;

	rsp.F1.adr		= req.F1.adr;
	rsp.F1.rw		= req.F1.rw;
	rsp.F1.flashLen	= flashLen;
	rsp.F1.flashCRC = flashCRC;

	rsp.F1.crc = GetCRC16(&rsp.F1, sizeof(rsp.F1)-sizeof(rsp.F1.crc));

	wb->data = &rsp.F1;
	wb->len = sizeof(rsp.F1);

	FreeReqAT25(r);

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestBoot_02(ReqAT25 *r, ComPort::WriteBuffer *wb)
{
	const BootReqV1 &req = *((BootReqV1*)(r->GetDataPtr()));
	static BootRspV1 rsp;

	u16 xl = req.F2.plen + sizeof(req.F2) - sizeof(req.F2.pdata);

	if (r->len < xl) return  false;

	u16 adr = req.F2.adr;

	if (req.F2.padr >= curWriteReqAdr)
	{
		curWriteReqAdr = req.F2.padr + req.F2.plen;
	
		r->dataOffset = (byte*)req.F2.pdata - r->data;
		r->stAdr = req.F2.padr;
		r->len = req.F2.plen;

		FlashWriteReq(r);
	}
	else
	{
		FreeReqAT25(r);
	};

	if (adr == 0) return false;

	rsp.F2.adr	= req.F2.adr;
	rsp.F2.rw	= req.F2.rw;
	rsp.F2.res	= GetLastError();

	rsp.F2.crc = GetCRC16(&rsp.F2, sizeof(rsp.F2)-sizeof(rsp.F2.crc));

	wb->data = &rsp.F2;
	wb->len = sizeof(rsp.F2);

	flashChecked = flashOK = flashCRCOK = false;

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestBoot_03(ReqAT25 *r, ComPort::WriteBuffer *wb)
{
	const BootReqV1 &req = *((BootReqV1*)(r->GetDataPtr()));
	static BootRspV1 rsp;

	if (r->len < sizeof(req.F3)) return  false;

	cmdReboot = true;

	if (req.F3.adr == 0) return  false;

	rsp.F3.adr		= req.F3.adr;
	rsp.F3.rw		= req.F3.rw;

	rsp.F3.crc = GetCRC16(&rsp.F3, sizeof(rsp.F3)-sizeof(rsp.F3.crc));

	wb->data = &rsp.F3;
	wb->len = sizeof(rsp.F3);

	FreeReqAT25(r);

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestBoot(ReqAT25 *r, ComPort::WriteBuffer *wb)
{
	bool result = false;

	BootReqV1 &req = *((BootReqV1*)(r->GetDataPtr()));

	u16 t = req.F0.rw;
	u16 adr = GetNetAdr();

	bool cm = (t & bootReqWord) == bootReqWord;
	bool ca = req.F0.adr == adr || req.F0.adr == 0;

	if (!ca || !cm || r->len < 2)
	{
		FreeReqAT25(r);
		return false;
	};

	t &= 0xFF;

	switch (t)
	{
		case 0: 	result = RequestBoot_00(r, wb); break;
		case 1: 	result = RequestBoot_01(r, wb); break;
		case 2: 	result = RequestBoot_02(r, wb); break;
		case 3: 	result = RequestBoot_03(r, wb); break;

		default:	FreeReqAT25(r); break;
	};

	return result;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateBlackFin()
{
	static byte i = 0;
	static ComPort::WriteBuffer wb;
	static ComPort::ReadBuffer rb;

	static ReqAT25 *req = 0;

	switch(i)
	{
		case 0:

			req = AllocReqAT25();

			if (req != 0)
			{
				req->len = 0;
				req->dataOffset = 0;
				rb.data = req->GetDataPtr();
				rb.maxLen = req->MaxLen();
				com.Read(&rb, ~0, US2COM(45+62500000/RCV_COM_BAUDRATE));
				i++;
			};

			break;

		case 1:

			if (!com.Update())
			{
				if (rb.recieved && rb.len > 0)
				{
					req->len = rb.len;
					
					if (GetCRC16(rb.data, rb.len) == 0 && RequestBoot(req, &wb))
					{
						com.Write(&wb);
						i++;
					}
					else 
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

						FreeReqAT25(req);
					};
				}
				else
				{
					FreeReqAT25(req);
					i = 0;
				};
			};

			break;

		case 2:

			if (!com.Update())
			{
				if (cmdReboot) Reboot(), cmdReboot = false;

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

				SetGain(dsc->next_gain);

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

				#ifdef RCV_WAVEPACK
				rsp.hdr.packType	= packType;
				#endif

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

				#ifdef RCV_WAVEPACK
				sportState = (rsp.hdr.packType == PACK_NO) ? 3 : (sportState+1);
				#else
				sportState = 3;
				#endif
			};

			break;

	#ifdef RCV_WAVEPACK

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

	#endif

		case 3:
		{
			RspRcv02 &rsp = prsp->r02;

			prsp->data[prsp->len/2] = GetCRC16_CCIT_refl(&prsp->r02, prsp->len);
			prsp->len += 2;

			readyRSP02.Add(prsp); prsp = 0;

			sportState = 0;

			break;
		};
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateSaveParams()
{
	static byte i = 0;

	static ReqAT25 *req = 0;

	switch (i)
	{
		case 0:

			if (cmdSaveParams)
			{
				i++;
			};

			break;

		case 1:

			req = AllocReqAT25();

			if (req != 0)
			{
				req->dataOffset = 0;
				req->stAdr = 0;
				//req->data
			};

			break;
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void CheckFlash()
{
	static ADI_BOOT_HEADER bh;
	static u16 crc = 0;

	while (FlashBusy()) FlashUpdate();

	u32 *p = (u32*)&bh;

	u32 adr = 0;
	
	flashOK = flashChecked = flashCRCOK = false;

	//at25df021_Read(buf, 0, sizeof(buf));

	while (1)
	{
		at25df021_Read(&bh, FLASH_START_ADR + adr, sizeof(bh));

		u32 x = p[0] ^ p[1] ^ p[2] ^ p[3];
		x ^= x >> 16; 
		x = (x ^ (x >> 8)) & 0xFF; 

		if (((u32)(bh.dBlockCode) >> 24) == 0xAD && x == 0)
		{
			adr += sizeof(bh);

			if ((bh.dBlockCode & BFLAG_FILL) == 0)
			{
				adr += bh.dByteCount;	
			};

			if (bh.dBlockCode & BFLAG_FINAL)
			{
				flashOK = true;

				break;
			};
		}
		else
		{
			break;
		};
	};

	flashLen = adr;

	at25df021_Read(&crc, FLASH_START_ADR + adr, sizeof(crc));

	if (flashLen > 0) flashCRC = at25df021_GetCRC16(FLASH_START_ADR, flashLen), flashCRCOK = (flashCRC == crc);

	if (!flashCRCOK) flashLen = 0;

	flashChecked = true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void main( void )
{
//	static byte s = 0;

//	static u32 pt = 0;

	InitHardware();

	FlashInit();

	CheckFlash();

#ifdef RCV_WAVEPACK
	Pack_Init();
#endif

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
			CALL( FlashUpdate()		);
		};

		i = (i > (__LINE__-S-3)) ? 0 : i;

		#undef CALL

		MAIN_LOOP_PIN_CLR();
	};

//	return 0;
}
