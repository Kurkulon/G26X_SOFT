#ifndef MEMORY_AT91SAM7X256_H
#define MEMORY_AT91SAM7X256_H

#define VERSION			0x0102

//#define MAIN_MODE_CHECK_MS	500
//
//typedef enum __attribute__ ((packed)) 
//{
//	MAIN_MODE_NONE = 0,
//	MAIN_MODE_EMAC,
//	MAIN_MODE_MEMORY,
//	MAIN_MODE_AUTONOM,
//} main_mode_type;

extern void SetNumDevice(u16 num);
extern u16 GetNumDevice();
extern void SaveMainParams();

#endif
