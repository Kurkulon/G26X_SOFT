#ifndef PACK_H__17_02_2024__00_00
#define PACK_H__17_02_2024__00_00

#include "types.h"
//#include "..\..\G_LIBS_BF592\cpp\fdct.h"
//#include "..\..\G_LIBS_BF592\cpp\mdct.h"
//#include "G.722.1\defs.h"

enum PackType { PACK_NO = 0, PACK_ULAW12, PACK_ULAW16, PACK_ADPCMIMA, PACK_DCT0, PACK_DCT1, PACK_DCT2, PACK_DCT3 };

extern void Pack_uLaw_12Bit(i16* src, byte* dst, u16 len);
extern void Pack_uLaw_16Bit(i16* src, byte* dst, u16 len);
extern void Pack_ADPCMIMA(i16* src, byte* dst, u16 len);
extern	u16	Pack_FDCT(i16* src, byte* dst, u16 len, u16 shift, u16 OVRLAP, u16* packedLen);
extern void Pack_Init();

#endif //PACK_H__17_02_2024__00_00
