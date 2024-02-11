#ifndef HW_COM_H__24_06_2022__13_39
#define HW_COM_H__24_06_2022__13_39

#ifdef WIN32
#include <windows.h>
#else
#include "core.h"
#endif

#include "types.h"
#include "ComPort\ComPort.h"

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

extern ComPort comdsp;	

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef WIN32	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


#elif defined(CPU_SAME53)	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


#elif defined(CPU_XMC48)	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


#endif	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



#endif // HW_COM_H__24_05_2022__13_39
