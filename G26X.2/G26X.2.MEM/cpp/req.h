#ifndef REQ_H__09_10_2014__10_31
#define REQ_H__09_10_2014__10_31

#pragma once

#include <RequestQuery.h>
#include <ComPort.h>

#include <list.h>
#include <flash.h>
#include <boot_req.h>

#include "G_TRM.h"
#include "G_RCV.h"

//struct Request
//{
//	byte adr;
//	byte func;
//	
//	union
//	{
//		struct  { byte n; word crc; } f1;  // старт оцифровки
//		struct  { byte n; byte chnl; word crc; } f2;  // чтение вектора
//		struct  { byte dt[3]; byte ka[3]; word crc; } f3;  // установка периода дискретизации вектора и коэффициента усиления
//	};
//};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
__packed struct ReqMoto
{
	u16 	rw;
	u16 	enableMotor; 
	u32		tRPM;		// время 1/6 оборота двигателя в мкс
	u16		limCurrent; // Ограничение тока двигателя (мА)
	u16		maxCurrent; // Аварийный ток двигателя (мА)
	u16 	crc;  
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct RspMoto
{
	u16 	rw;
	u16 	mororStatus; 
	u16		current;
	u16		currentLow;
	u16		rpm;
	u16		motoCounter;
	u16		auxVoltage;
	u16		motoVoltage;
	u16 	crc;  
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct ReqBootMotoHS { unsigned __int64 guid; u16 crc; };
__packed struct RspBootMotoHS { unsigned __int64 guid; u16 crc; };

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct ReqBootMoto
{
	union
	{
		struct { u32 func; u32 len;								u16 align; u16 crc; }	F1; // Get CRC
		struct { u32 func;										u16 align; u16 crc; }	F2; // Exit boot loader
		struct { u32 func; u32 padr; u32 plen; u32 pdata[16];	u16 align; u16 crc; }	F3; // Programm page
	};
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

struct RspBootMoto
{
	union
	{
		struct { u32 func; u32 pageLen;	u32 len;	u16 sCRC;	u16 crc; }	F1; // Get CRC
		struct { u32 func;							u16 align;	u16 crc; } 	F2; // Exit boot loader
		struct { u32 func; u32 padr;	u32 status; u16 align;	u16 crc; } 	F3; // Programm page
	};
};
*/
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
__packed struct SENS
{
	u16 gain;
	u16 sampleTime;
	u16 sampleLen;
	u16 sampleDelay;
	u16 deadTime;
	u16 descriminant;
	u16 freq;
	u16 filtrType;
	u16 packType;
	u16 fi_Type;
	u16 fragLen;
};
	
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct ReqDsp01	// чтение вектора
{
	enum { VERSION = 1 };

	u16 	rw;
	u16		len;				// Длина структуры
	u16		version;			// Версия структуры

	u16 	mode; 
	u16		ax; 
	u16		ay; 
	u16		az; 
	u16		at;
	SENS	sens1;
	SENS	sens2;
	SENS	refSens;
	u16		vavesPerRoundCM;
	u16		vavesPerRoundIM;
	u16		fireVoltage;		// Напряжение излучателя (0.1В)
	u16		sensMask;

	u16 	crc;  
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct RspHdrCM	// 0xAD40
{
	u16 	rw;
	u32 	time;		//mmsecTime; 
	u32		hallTime;	//shaftTime; 
	u16		motoCount; 
	u16		headCount;
	u16		ax; 
	u16		ay; 
	u16		az; 
	u16		at;
	u16		sensType; 
	u16		angle;
	u16		maxAmp;
	u16		fi_amp;
	u16		fi_time;
	u16 	gain; 
	u16 	st;	 
	u16 	sl; 
	u16 	sd; 
	u16		packType;
	u16		packLen;
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct RspHdrIM	// 0xAD50
{
	u16 	rw;
	u32 	time;		//mmsecTime; 
	u32		hallTime;	//shaftTime; 
	u16		ax; 
	u16		ay; 
	u16		az; 
	u16		at;
	u16		sensType; 
	u16 	gain; 
	u16		refAmp;
	u16		refTime;
	u16		dataLen;
};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed union RspDsp01	// чтение вектора
{
	__packed struct { RspHdrCM hdr; u16 data[512]; } CM;
	__packed struct { RspHdrIM hdr; u16 data[512]; } IM;
	__packed struct { u16 rw; u16 len; u16 version; u16 fireVoltage; u16 motoVoltage; u16 crc; } v01;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct  ReqDsp05	// запрос контрольной суммы и длины программы во флэш-памяти
{ 
	u16		rw; 
	u16 	crc; 
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct  RspDsp05	// запрос контрольной суммы и длины программы во флэш-памяти
{ 
	u16		rw; 
	u16		flashLen; 
	u32		startAdr; 
	u16		flashCRC; 
	u16		crc;
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct  ReqDsp06	// запись страницы во флэш
{ 
	u16		rw; 
	u16		stAdr; 
	u16		count; 
	byte	data[258]; 
	u16		crc; 
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct  RspDsp06	// запись страницы во флэш
{ 
	u16		rw; 
	u16		res; 
	word	crc; 
};  

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct  ReqDsp07	// перезагрузить блэкфин
{ 
	u16		rw; 
	word 	crc; 
};  
*/
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

union ReqUnion
{
	ReqRcv01 	rcv01;
	ReqRcv02	rcv02;
	ReqRcv03	rcv03;
	ReqRcv04	rcv04;
	ReqDsp05 	dsp05;	
	ReqDsp06 	dsp06;	
	ReqDsp07 	dsp07;	
	ReqTrm01	trm01;	
	ReqTrm02	trm02;
	ReqTrm03	trm03;
	BootReqMes	bootTrm;
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

__packed struct RspMan60
{
	u16 rw; 
	u32 cnt; 
	u16 maxAmp[96]; 
	u16 power[96];
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//__packed struct ReqTrm01	
//{
//	byte 	len;
//	byte 	func;
//	byte 	n; 
//	word 	crc;  
//};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//__packed struct ReqTrm02	
//{
//	byte 	len;
//	byte 	f;
//	word 	crc;  
//};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//__packed struct RspTrm02	
//{
//	byte f; 
//	u16 hv; 
//	u16 crc;
//};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//__packed struct ReqTrm03	
//{
//	byte 	len;
//	byte	f; 
//	byte	fireCountM; 
//	byte	fireCountXY; 
//	u16		hv;
//	word 	crc;  
//};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//__packed struct RspTrm03	
//{
//	byte f; 
//	u16 crc;
//};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//__packed struct ReqMem
//{
//	u16 rw; 
//	u32 cnt; 
//	u16 gain; 
//	u16 st; 
//	u16 len; 
//	u16 delay; 
//	u16 data[1024*4]; 
//	u16 crc;
//
//	//byte adr;
//	//byte func;
//	
//	//__packed union
//	//{
//	//	__packed struct  { word crc; } f1;  // Старт новой сессии
//	//	__packed struct  { word crc; } f3;  
//	//};
//};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//__packed struct RspMem
//{
//	u16 rw; 
//	u16 crc; 
//
//	//byte	adr;
//	//byte	func;
//	
//	//__packed union
//	//{
//	//	__packed struct  { word crc; } f1;  // Старт новой сессии
//	//	__packed struct  { word crc; } f2;  // Запись вектора
//	//	__packed struct  { word crc; } f3;  // 
//	//	__packed struct  { word crc; } fFE;  // Ошибка CRC
//	//	__packed struct  { word crc; } fFF;  // Неправильный запрос
//	//};
//};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//__packed struct RspMan60
//{
//	u16 rw; 
//	u32 cnt; 
//	u16 maxAmp[96]; 
//	u16 power[96];
//};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//struct REQ : public PtrItem<REQ>
//{
//	PTR_LIST_FRIENDS(REQ);
//
//	bool	ready;
//	bool	crcOK;
//	bool	checkCRC;
//	bool	updateCRC;
//
//	typedef void tRsp(Ptr<REQ> &q);
//
//	u16		tryCount;
//	
//	//REQ *next;
//
//	tRsp		*CallBack;
//	Ptr<MB>	rsp;
//
//	ComPort::WriteBuffer wb;
//	ComPort::ReadBuffer rb;
//
//	u32		preTimeOut, postTimeOut;
//
//	byte	reqData[(sizeof(ReqUnion)+64) & ~3];
//
//protected:
//
//	virtual void _FreeCallBack() { rsp.Free(); }
//
//public:
//
//	//void	Free() { if (this != 0) rsp.Free(), PtrItem<REQ>::Free(); }
//
//	REQ() : tryCount(0) { }
//};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//class RequestQuery
//{
//	//REQ*			_first;
//	//REQ*			_last;
//	Ptr<REQ>		_req;
//
//	ListPtr<REQ>	reqList;
//	
//	byte			_state;
//
//	u16				_crc;
//	u16 			_crcLen;
//
//	byte*			_crcPtr;
//
//
//	ComPort			*com;
//
//	//u32			count;
//
//	bool			_run;
//
//public:
//
//				RequestQuery(ComPort *p) : _state(0), com(p), _run(true) {}
//	void		Add(const Ptr<REQ>& req)	{ reqList.Add(req); }
//	Ptr<REQ>	Get()						{ return reqList.Get(); }
//	//bool Empty() { return reqList.Empty(); }
//	//bool Idle() { return (_first == 0) && (_req == 0); }
//	bool Stoped() { return !_req.Valid(); }
//	void Update();
//	void Stop() { _run = false; }
//	void Start() { _run = true; }
//};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//struct R01
//{
//	R01*	next;
//
//protected:
//
//	u32		count;
//
//	static List<R01> freeList;
//
//public:
//
////	bool memNeedSend;
//	//ComPort::WriteBuffer	wb;
//	//ComPort::ReadBuffer		rb;
//	REQ			q;
//	ReqDsp01	req;
//	RspDsp01	rsp;
//
//	R01() : next(0) { freeList.Add(this); }
//
//	static	R01*	Alloc()	{ R01* p = freeList.Get(); if (p != 0) p->count = 1; return p; };
//			void	Free()	{ if (this != 0 && count != 0) { count--; if (count == 0) freeList.Add(this); }; }
//};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


/*struct RMEM
{
	RMEM* next;

//	R01*	r02;

	ComPort::WriteBuffer	wb;
	ComPort::ReadBuffer		rb;
	REQ		q;
	ReqMem	req;
	RspMem	rsp;
};*/

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++






#endif //REQ_H__09_10_2014__10_31
