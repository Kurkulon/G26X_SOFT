//#pragma O3
//#pragma Otime

//#include <stdio.h>
//#include <conio.h>

#include <ComPort.h>
#include "hw_conf.h"
#include "hw_com.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef CPU_SAME53

ComPort comTrm	(UART4_TRM_NUM, 0, PIO_UTXD4, PIO_URXD4, PIO_RTS4, 0, PIN_UTXD4, PIN_URXD4, PIN_RTS4, 0, PMUX_UTXD4, PMUX_URXD4, UART4_TXPO, UART4_RXPO, UART4_GEN_SRC, UART4_GEN_CLK, &UART4_DMA);
ComPort comRcv	(UART7_RCV_NUM, 0, PIO_UTXD7, PIO_URXD7, PIO_RTS7, 0, PIN_UTXD7, PIN_URXD7, PIN_RTS7, 0, PMUX_UTXD7, PMUX_URXD7, UART7_TXPO, UART7_RXPO, UART7_GEN_SRC, UART7_GEN_CLK, &UART7_DMA);

#elif defined(WIN32)

ComPort comTrm;	
ComPort comRcv;	

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <ComPort_imp.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
