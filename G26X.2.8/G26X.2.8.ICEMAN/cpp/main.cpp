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
//#include "G_TRM.h"
#include "TaskList.h"

//#include "G_TRM.H"

#define TRANSMITER_NUM			3	// 0 - Monopole1, 1 - Monopole1, 2 - DipoleXY
#define RCV_FIRE_NUM			4	// 0 - Mnpl_1, 1 - Mnpl_2, 2 - DplX, 3 - DplY
#define RCV_MAX_NUM_STATIONS	13
#define RCV_SAMPLE_LEN 1024 //1456

enum { VERSION = (0x100+18) };

//#pragma O3
//#pragma Otime

#ifndef _DEBUG
	static const bool __debug = false;
#else
	static const bool __debug = true;
#endif

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

	u16 autoGain;
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static MainVars mv;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u32 fps;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static u16 manRcvData[10];
static u16 manTrmData[50];
static u16 manTrmBaud = 0;

static const u16 manReqWord = 0xAB00; // 13АД
static const u16 manReqMask = 0xFF00;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u16 txbuf[128 + 512 + 16];

static u16 reqFireVoltage = 0;
static u16 curFireVoltage = 300;


static u16 verDevice = VERSION;
static bool numDevValid = false;
static u16  numDevRcv = 0;
static u16  verDevRcv = 0;
static u16  numDevTrm = 0;
static u16  verDevTrm = 0;

i16 temp = 0;

static byte svCount = 0;


//u16 fireAmp = 0;
//u16 fireFreq = 3000;

static u16  arrRcvTemp[RCV_MAX_NUM_STATIONS] = {0};

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
	*(data++)  	= mv.autoGain;							//34. Автоматическое усиление (0 - выкл, 1 - вкл)

	return data - start;
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


	__packed u16 *start = data;

	*(data++) = rw; //manReqWord|0x20;			//	1. 	ответное слово
	*(data++) = 0;								//	2. 	счётчик. младшие 2 байта
	*(data++) = 0;								//	3. 	счётчик. старшие 2 байта
	*(data++) = RCV_MAX_NUM_STATIONS;			//	4.	к-во приемников
	*(data++) = 0; 								//	5.	статус приёмников (бит 0 - П1, бит 1 - П2, ... , бит 12 - П13)
	*(data++) = 0;			 					//	6.	статус ошибок линии приёмников (бит 0 - П1, бит 1 - П2, ... , бит 12 - П13)
	*(data++) = 0;	 							//	7.	сопротивление IdLine
	*(data++) = 0;			 					//	8.	Счётчик запросов приёмников
	*(data++) = 0;								//	9.	AX
	*(data++) = 0;								//	10.	AY
	*(data++) = 0;								//	11.	AZ
	*(data++) = 0;					 			//	12.	AT
	*(data++) = temp;				 			//	13.	Температура в приборе (short)(0.1гр)
	*(data++) = min;				 			//	14.	Минимальное значение температуры в приёмниках (short)(0.1гр)
	*(data++) = max;				 			//	15.	Максимальное значение температуры в приёмниках (short)(0.1гр)
	*(data++) = 0;			 					//	16.	Вибрация (у.е)(ushort)
	*(data++) = 0;			 					//	17.	напряжение излучателя (1 Вольт)
	*(data++) = 0;			 					//	18.	Температура излучателя (short)(0.1гр)
	*(data++) = 0;			 					//	19.	Счётчик запросов излучателя
	*(data++) = GetRcvManQuality();	 			//	20.	Качество сигнала запроса телеметрии (%)
	*(data++) = 0;			 					//	21. Счётчик пропусков оцифровки
	*(data++) = 0;								//	22. статус ошибок CRC приёмников			(бит 0 - П1, бит 1 - П2, ... , бит 12 - П13)
	*(data++) = 0;								//	23. статус неправильный ответ приёмников	(бит 0 - П1, бит 1 - П2, ... , бит 12 - П13)
	*(data++) = 0;								//	24. статус нет ответа приёмников			(бит 0 - П1, бит 1 - П2, ... , бит 12 - П13)

	return data - start;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_20(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 2 || mtb == 0) return false;

	//fireMask = data[0] & 0xF;

	len = InitRspMan_20(data[0], manTrmData);

	//ClearRcvErrors();

	mtb->data1 = manTrmData;
	mtb->len1 = len;
	mtb->data2 = 0;
	mtb->len2 = 0;

	//runMainMode = true;

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_30(u16 *data, u16 reqlen, MTB* mtb)
{
	if (data == 0 || reqlen == 0 || reqlen > 4 || mtb == 0) return false;

	manTrmData[0] = data[0];

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_71(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 4 || mtb == 0) return false;

	manTrmData[0] = data[0];

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
	mtb->data2 = 0;
	mtb->len2 = 0;

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestMan_72(u16 *data, u16 len, MTB* mtb)
{
	if (data == 0 || len == 0 || len > 4 || mtb == 0) return false;

	manTrmData[0] = data[0];

	mtb->data1 = manTrmData;
	mtb->len1 = 1;
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
		case 1:		mv.numDevice	= data[2];		break;
		case 2:		manTrmBaud		= data[2] - 1;	break;
		case 3:		numDevRcv		= data[2];		break;
		case 4:		numDevTrm		= data[2];		break;
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

		case 0x70:	mv.autoGain					= MIN(data[2],			1				);	break;	//	0x70 - Автоматическое усиление (0 - выкл, 1 - вкл)

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

			if (tm.Check(US2CTM(200)))
			{
				SendManData(&mtb);

				i++;
			};

			break;

		case 3:

			if (mtb.ready)
			{
				//if (curRsp72 != 0) FreeRsp20(curRsp72), curRsp72 = 0;

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


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void InitMainVars()
{
	mv.numDevice		= 0;
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

	numDevValid = true;
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

//static void UpdateParams()
//{
//	static byte i = 0;
//
//	#define CALL(p) case (__LINE__-S): p; break;
//
//	enum C { S = (__LINE__+3) };
//	switch(i++)
//	{
//		CALL( UpdateTemp()				);
//		CALL( SaveVars();				);
//		CALL( UpdateHV();				);
//	};
//
//	i = (i > (__LINE__-S-3)) ? 0 : i;
//
//	#undef CALL
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static void UpdateMisc()
//{
//	static byte i = 0;
//
//	#define CALL(p) case (__LINE__-S): p; break;
//
//	enum C { S = (__LINE__+3) };
//	switch(i++)
//	{
//		CALL( UpdateCom(); 		);
//		CALL( UpdateHardware();	);
//		CALL( UpdateParams();	);
//	};
//
//	i = (i > (__LINE__-S-3)) ? 0 : i;
//
//	#undef CALL
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static TaskList taskList;

static void InitTaskList()
{
	static Task tsk[] =
	{
		Task(UpdateTemp,		US2CTM(100)	),
		Task(SaveVars,			US2CTM(100)	),
		Task(UpdateMan,			US2CTM(100)	),
		Task(UpdateHardware,	US2CTM(1)	)
	};

	for (u16 i = 0; i < ArraySize(tsk); i++) taskList.Add(tsk+i);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int main()
{
	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_WHITE "main() start ...\n");

	TM32 tm;

	//__breakpoint(0);

	InitHardware();

	//LoadVars();

	InitTaskList();

#ifndef WIN32

//	comdsp.Connect(ComPort::ASYNC, TRM_COM_BAUDRATE, TRM_COM_PARITY, TRM_COM_STOPBITS);

#endif

	u32 fc = 0;
	u16 n = 0;

	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_WHITE "Main Loop start ...\n");

	MTB mtb;

	while (1)
	{
		Pin_MainLoop_Set();

		taskList.Update(); //UpdateMisc();

		Pin_MainLoop_Clr();

		fc++;

		if (tm.Check(1000))
		{
			fps = fc; fc = 0; 

			//mtb.baud = 0;
			//mtb.data1 = manTrmData;
			//mtb.len1 = 2;
			//mtb.data2 = 0;
			//mtb.len2 = 0;

			//SendManData(&mtb);
		};
	}; // while (1)
}
