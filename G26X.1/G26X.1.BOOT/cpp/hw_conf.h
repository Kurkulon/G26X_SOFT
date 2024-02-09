#ifndef HW_CONF_H__11_11_22__17_22
#define HW_CONF_H__11_11_22__17_22

#define	CORETYPE_BF592

#include <bf592.h>
#include <G_RCV.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define CLKIN_MHz			25
#define CLKIN_DIV			1	// 1, 2

#define PLL_MUL				8	// 5...64
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

#define MS2SCLK(x)	((u32)(x*1.0*SCLK/1e3+0.5))
#define US2SCLK(x)	((u32)(x*1.0*SCLK/1e6+0.5))
#define NS2SCLK(x)	((u32)(x*1.0*SCLK/1e9+0.5))

#define MS2CLK(x)	MS2SCLK(x) 
#define US2CLK(x)	US2SCLK(x)
#define NS2CLK(x)	NS2SCLK(x)

#define MS2CCLK(x)	((u32)(x*1.0*CCLK/1e3+0.5))
#define US2CCLK(x)	((u32)(x*1.0*CCLK/1e6+0.5))
#define NS2CCLK(x)	((u32)(x*1.0*CCLK/1e9+0.5))

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

// Вектора прерываний
// IVG7		- 
// IVG8 	- DMA0 (PPI)
// IVG9 	- PORTF PF4 SYNC
// IVG10 	- GPTIMER0 FIRE
// IVG11 	- GPTIMER2 RTT
// IVG12 	- TWI


// CoreTimer - PPI delay

// TIMER0 	- Fire
// TIMER1 	- PPI CLK
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
//#define IVG_PPI_DMA0		10
//#define IVG_PORTG_ROT		11
#define IVG_TWI				12
//#define IVG_GPTIMER0_FIRE	10

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define INIT_PORTF_MUX			0														
#define INIT_PORTG_MUX			0														

#define INIT_PORTF_FER 			0														
#define INIT_PORTG_FER 			0														

#define INIT_PORTFIO_DIR 		PF4|PF5|PF6|PF7|PF10									
#define INIT_PORTGIO_DIR 		PG4|PG5|PG6|PG7|PG8|PG9|PG10|PG11|PG12|PG13|PG14		

#define INIT_PORTFIO_INEN 		0													
#define INIT_PORTGIO_INEN 		0													

#define INIT_PORTGIO 			0
#define INIT_PORTFIO 			0

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

#define INIT_WDOG_CNT			MS2CLK(1000)

#ifndef __DEBUG
#define INIT_WDOG_CTL			WDEV_RESET|WDEN
#else
#define INIT_WDOG_CTL			WDEV_RESET|WDEN
#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define BOOT_NETADR
#define BOOT_COM_SPEED			RCV_COM_BAUDRATE
#define BOOT_COM_PARITY			RCV_COM_PARITY
#define BOOT_COM_PRETIMEOUT		(~0)
#define BOOT_COM_POSTTIMEOUT	(US2COM(500))

#define BOOT_MAN_REQ_WORD		0xAD00
#define BOOT_MAN_REQ_MASK 		0xFF00

#define BOOT_HW_UPDATE 			UpdateHardware

#define PIO_RTS					HW::PIOF
#define PIN_RTS					10
#define MASK_RTS				(1UL<<PIN_RTS)

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define MAIN_LOOP_PIN_SET()		{ HW::PIOF->SET(PF4);}
#define MAIN_LOOP_PIN_CLR()		{ HW::PIOF->CLR(PF4);}
#define MAIN_LOOP_PIN_TGL()		{ HW::PIOF->NOT(PF4);}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define BAUD_RATE_DIVISOR 	5

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#endif // HW_CONF_H__11_11_22__17_22
