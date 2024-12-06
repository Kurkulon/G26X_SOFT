#include "pack.h"

#pragma optimize_for_speed

#ifdef WAVEPACK_V2

#define FDCT_LOG2N		8
#define FDCT_TRIG		u16
#define FDCT_TRIGBITS	(17-FDCT_LOG2N)

#else

#define FDCT_LOG2N		6
#define FDCT_TRIG		u16
#define FDCT_TRIGBITS	(17-FDCT_LOG2N)

#endif

#define FDCT_BAND_DIV 2

//#include "WAVEPACK\fdct_imp.h"
#include "WAVEPACK\wavepack_imp.h"

//#include "PackWave.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//void Pack_uLaw_12Bit(i16* src, byte* dst, u16 len)
//{
//	WavePack_uLaw_12Bit((u16*)src, dst, len);
//}
//
////++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//void Pack_uLaw_16Bit(i16* src, byte* dst, u16 len)
//{
//	WavePack_uLaw_16Bit((u16*)src, dst, len);
//}
//
////++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//void Pack_ADPCMIMA(i16* src, byte* dst, u16 len)
//{
//	WavePack_ADPCMIMA((u16*)src, dst, len);
//}
//
////++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//u16 Pack_FDCT(i16* src, byte* dst, u16 len, u16 shift, u16 OVRLAP, u16* packedLen)
//{
//	return WavePack_FDCT12(src, dst, len, shift, OVRLAP, FDCT_N, packedLen);
//}
//
////++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//void Pack_Init()
//{
//	FDCT_Init();
//}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
