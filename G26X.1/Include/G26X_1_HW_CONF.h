#ifndef G26X_1_HW_CONF_H__13_02_2024__23_17
#define G26X_1_HW_CONF_H__13_02_2024__23_17

#pragma once

#include <types.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define CLKIN_MHz			25
#define CLKIN_DIV			1	// 1, 2

#define PLL_MUL				16	// 5...64
#define SCLK_DIV			2	// 1...15
#define CCLK_CSEL			0	// 0...3
#define CCLK_DIV			(1UL<<CCLK_CSEL)

#define VCO_CLK_MHz 		(CLKIN_MHz*PLL_MUL/CLKIN_DIV)
#define CCLK_MHz			VCO_CLK_MHz/CCLK_DIV
#define SCLK_MHz			VCO_CLK_MHz/SCLK_DIV

#define VRCTL_VALUE         0x0000

#if CLKIN_DIV == 2
#define PLLCTL_VALUE        (SET_MSEL(PLL_MUL)|DF)
#else
#define PLLCTL_VALUE        (SET_MSEL(PLL_MUL))
#endif

#define PLLDIV_VALUE        (SET_SSEL(SCLK_DIV))
#define PLLLOCKCNT_VALUE    0x0000
#define PLLSTAT_VALUE       0x0000
//#define RSICLK_DIV          0x0001

#define SCLK (SCLK_MHz*1000000)
#define CCLK (CCLK_MHz*1000000)

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define MS2CLK(x) ((u32)(x*1.0*SCLK/1e3+0.5))
#define US2CLK(x) ((u32)(x*1.0*SCLK/1e6+0.5))
#define NS2CLK(x) ((u32)(x*1.0*SCLK/1e9+0.5))

#define MS2SCLK(x) MS2CLK(x)
#define US2SCLK(x) US2CLK(x)
#define NS2SCLK(x) NS2CLK(x)

#define MS2CCLK(x) ((u32)(x*1.0*CCLK/1e3+0.5))
#define US2CCLK(x) ((u32)(x*1.0*CCLK/1e6+0.5))
#define NS2CCLK(x) ((u32)(x*1.0*CCLK/1e9+0.5))

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// 4  - PF4  - Main Loop
// 5  - PF5  - 
// 6  - PF6  - 
// 7  - PF7  - 
// 37 - PG5  -
// 38 - PG6  -
// 39 - PG7  -
// 39 - PG8  -
// 39 - PG9  -
// 39 - PG10 -
// 39 - PG11 -
// 39 - PG12 -
// 39 - PG13 -
// 39 - PG14 -
// 39 - PG15 -


//#define Pin_UpdateMode_Set()	HW::PIOG->BSET(3)
//#define Pin_UpdateMode_Clr()	HW::PIOG->BCLR(3)
//
//#define Pin_SPORT0_ISR_Set()	HW::PIOG->BSET(5)
//#define Pin_SPORT0_ISR_Clr()	HW::PIOG->BCLR(5)
//
//#define Pin_SPORT1_ISR_Set()	HW::PIOG->BSET(12)
//#define Pin_SPORT1_ISR_Clr()	HW::PIOG->BCLR(12)
//
//#define Pin_ProcessSPORT_Set()	HW::PIOG->BSET(13)
//#define Pin_ProcessSPORT_Clr()	HW::PIOG->BCLR(13)

#define Pin_MainLoop_Set()		HW::PIOF->BSET(4)
#define Pin_MainLoop_Clr()		HW::PIOF->BCLR(4)


// Вектора прерываний
// IVG7		- 
// IVG8 	- DMA0 (PPI)
// IVG9 	- PORTF PF4 SYNC
// IVG10 	- GPTIMER0 FIRE
// IVG11 	- GPTIMER2 RTT
// IVG12 	- TWI


// CoreTimer - PPI delay

// TIMER0 	- Fire2
// TIMER1 	- Fire1
// TIMER2 	- RTT

// UART0	- 
// SPI0		- Boot flash
// SPI1 	- 
// TWI		- 

#define IVG_EMULATION		0
#define IVG_RESET			1
#define IVG_NMI				2
#define IVG_EXEPTIONS		3
#define IVG_HW_ERROR		5
#define IVG_CORETIMER		6
//#define IVG_PORTF_SYNC		7
//#define IVG_PORTF_SHAFT		8
//#define IVG_GPTIMER2_RTT	9
#define IVG_SPORT0_DMA		10
#define IVG_SPORT1_DMA		11
//#define IVG_PORTG_ROT		12
#define IVG_TWI				13

// ++++++++++++++	DMA	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//#define	PPI_DMA				0
#define	SPORT0_RX_DMA		1
//#define	SPORT0_TX_DMA		2
#define	SPORT1_RX_DMA		3
//#define	SPORT1_TX_DMA		4
#define	SPI0_DMA			5
#define	SPI1_DMA			6
#define	UART_RX_DMA			7
#define	UART_TX_DMA			8

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define PIO_FIRE			HW::PIOF

#define PIN_FIRE1			9
#define PIN_FIRE2			10

#define BM_FIRE1			(1UL << PIN_FIRE1)
#define BM_FIRE2			(1UL << PIN_FIRE2)

#define FIRE1_TIMER			HW::TIMER1
#define FIRE2_TIMER			HW::TIMER0

#define FIRE1_TIMEN			TIMEN1
#define FIRE2_TIMEN			TIMEN0

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define PIO_GAIN			HW::PIOF
#define PIN_GAIN			3
#define BM_GAIN				(1U << PIN_GAIN)	

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define PIO_RTS					HW::PIOF
#define PIN_RTS					5
#define MASK_RTS				(1UL<<PIN_RTS)

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define INIT_PORTF_MUX			0xFEC3		//  1111 1110 1100 0011
#define INIT_PORTG_MUX			0x03C3		//  0000 0011 1100 0011

#define INIT_PORTF_FER 			0x0000		//  0000 0000 0000 0000
#define INIT_PORTG_FER 			0x0000		//  0000 0000 0000 0000

#define INIT_PORTFIO_DIR 		0x0124		//  0000 0001 0010 0100
#define INIT_PORTGIO_DIR 		0x7C3C		//  0111 1100 0011 1100

#define INIT_PORTFIO_INEN 		0x0000		//  0000 0000 0000 0000
#define INIT_PORTGIO_INEN 		0x0000		//  0000 0000 0000 0000

#define INIT_PORTFIO 			MASK_RTS
#define INIT_PORTGIO 			0

#define INIT_PORTFIO_POLAR		0
#define INIT_PORTFIO_EDGE 		0
#define INIT_PORTFIO_BOTH 		0
#define INIT_PORTFIO_MASKA		0
#define INIT_PORTFIO_MASKB		0

#define INIT_PORTGIO_POLAR		0
#define INIT_PORTGIO_EDGE 		0
#define INIT_PORTGIO_BOTH 		0
#define INIT_PORTGIO_MASKA		0
#define INIT_PORTGIO_MASKB		0

#ifndef __DEBUG
#define INIT_WDOG_CNT			MS2CLK(100)
#define INIT_WDOG_CTL			WDEV_RESET|WDEN
#else
#define INIT_WDOG_CNT			MS2CLK(100)
#define INIT_WDOG_CTL			WDEV_RESET|WDDIS
#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define MAIN_LOOP_PIN_SET()		{*pPORTFIO_SET = 1<<5;}
#define MAIN_LOOP_PIN_CLR()		{*pPORTFIO_CLEAR = 1<<5;}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#endif // G26X_1_HW_CONF_H__13_02_2024__23_17
