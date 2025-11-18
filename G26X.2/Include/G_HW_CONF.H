#ifndef G_HW_CONF_H__27_12_2023__12_00
#define G_HW_CONF_H__27_12_2023__12_00

#pragma once

#include <types.h>
#include <core.h>


#define FRAM_SPI_MAINVARS_ADR 0
#define FRAM_SPI_SESSIONS_ADR 0x200

#define FRAM_I2C_MAINVARS_ADR 0
#define FRAM_I2C_SESSIONS_ADR 0x200

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define TRAP_PACKET_VERSION	0x7
#define REAL_TIME_CLOCK_ENABLE

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef CPU_SAME53	
	
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define MCK_MHz 200
	#define MCK (MCK_MHz*1000000)
	#define NS2CLK(x) (((x)*MCK_MHz+500)/1000)
	#define US2CLK(x) ((x)*MCK_MHz)
	#define MS2CLK(x) ((x)*MCK_MHz*1000)

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	// Test Pins
	// 37	- PB15	- SPI_Handler
	// 42	- PC12
	// 43	- PC13
	// 52	- PA16
	// 57	- PC17
	// 58	- PC18
	// 59	- PC19	- SPI DSP
	// 66	- PB18	- ManRcvIRQ sync true
	// 74	- PA24	- ManRcvIRQ
	// 75	- PA25	- main loop


	// ++++++++++++++	GEN	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define GEN_MCK		0
	#define GEN_32K		1
	#define GEN_25M		2
	#define GEN_1M		3
	//#define GEN_500K	4
	//#define GEN_EXT32K	5

	#define GEN_MCK_CLK			MCK
	#define GEN_32K_CLK			32768
	#define GEN_25M_CLK			25000000
	#define GEN_1M_CLK			1000000

	// ++++++++++++++	SERCOM	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define SPI_SERCOM_NUM		0
	//#define UART2_LPC			1
//	#define SERCOM_2			2
	#define I2C_SERCOM_NUM		3
	#define UART4_TRM_NUM		4
	//#define UART1_DSP			5
	//#define SERCOM_6			6
	#define UART7_RCV_NUM		7

	// ++++++++++++++	DMA	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define	SPI_DMA_TX_CH		0
	#define	SPI_DMA_RX_CH		1
	#define	I2C_DMA_CH			2
	#define	NAND_DMA_CH			3
	#define	UART7_DMA_CH		4
	#define	UART4_DMA_CH		5
	#define	NAND_MEMCOPY_DMA_CH	6
	//#define	DSP_SPI_DMA_RX	8

	//#define	DSP_SPI_DMA_TX	30
	#define	CRC_DMA_CH			31

	// ++++++++++++++	EVENT	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define EVENT_NAND_1	0
	//#define EVENT_NAND_2	1
	//#define EVENT_NAND_3	2
	#define EVENT_MANR_1	3
	#define EVENT_MANR_2	4

	// ++++++++++++++	TC	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define MANI_TC			TC0
	//#define CRCDMA_TC		TC1
	//#define 				TC2
	//#define 				TC3
	//#define 				TC4
	//#define 				TC5
	//#define 				TC6
	//#define 				TC7

	#define GEN_TC0_TC1			GEN_MCK
	#define GEN_TC2_TC3			GEN_MCK
	#define GEN_TC4_TC5			GEN_MCK
	#define GEN_TC6_TC7			GEN_MCK

	#define CLK_TC0_TC1			GEN_MCK_CLK
	#define CLK_TC2_TC3			GEN_MCK_CLK
	#define CLK_TC4_TC5			GEN_MCK_CLK
	#define CLK_TC6_TC7			GEN_MCK_CLK

	#define GEN_TC0				GEN_TC0_TC1
	#define GEN_TC1				GEN_TC0_TC1
	#define GEN_TC2				GEN_TC2_TC3
	#define GEN_TC3				GEN_TC2_TC3
	#define GEN_TC4				GEN_TC4_TC5
	#define GEN_TC5				GEN_TC4_TC5
	#define GEN_TC6				GEN_TC6_TC7
	#define GEN_TC7				GEN_TC6_TC7

	#define GCLK_TC0			GCLK_TC0_TC1
	#define GCLK_TC1			GCLK_TC0_TC1
	#define GCLK_TC2			GCLK_TC2_TC3
	#define GCLK_TC3			GCLK_TC2_TC3
	#define GCLK_TC4			GCLK_TC4_TC5
	#define GCLK_TC5			GCLK_TC4_TC5
	#define GCLK_TC6			GCLK_TC6_TC7
	#define GCLK_TC7			GCLK_TC6_TC7

	#define CLK_TC0				CLK_TC0_TC1
	#define CLK_TC1				CLK_TC0_TC1
	#define CLK_TC2				CLK_TC2_TC3
	#define CLK_TC3				CLK_TC2_TC3
	#define CLK_TC4				CLK_TC4_TC5
	#define CLK_TC5				CLK_TC4_TC5
	#define CLK_TC6				CLK_TC6_TC7
	#define CLK_TC7				CLK_TC6_TC7

	// ++++++++++++++	TCC	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define MANT_TCC		TCC0
	//#define				TCC1
	#define MANR_TCC		TCC2
	#define SYNC_TCC		TCC3
	#define NAND_TCC		TCC4
	//#define MltTmr		TCC4

	#define GEN_TCC0_TCC1		GEN_MCK
	#define GEN_TCC2_TCC3		GEN_MCK
	#define GEN_TCC4			GEN_MCK

	#define CLK_TCC0_TCC1		GEN_MCK_CLK
	#define CLK_TCC2_TCC3		GEN_MCK_CLK
	#define CLK_TCC4			GEN_MCK_CLK

	#define GEN_TCC0			GEN_TCC0_TCC1
	#define GEN_TCC1			GEN_TCC0_TCC1
	#define GEN_TCC2			GEN_TCC2_TCC3
	#define GEN_TCC3			GEN_TCC2_TCC3


	#define GCLK_TCC0			GCLK_TCC0_TCC1
	#define GCLK_TCC1			GCLK_TCC0_TCC1
	#define GCLK_TCC2			GCLK_TCC2_TCC3
	#define GCLK_TCC3			GCLK_TCC2_TCC3


	#define CLK_TCC0			CLK_TCC0_TCC1
	#define CLK_TCC1			CLK_TCC0_TCC1
	#define CLK_TCC2			CLK_TCC2_TCC3
	#define CLK_TCC3			CLK_TCC2_TCC3

	// ++++++++++++++	I2C	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define PIO_I2C				HW::PIOA 
	#define PIN_SDA				22 
	#define PIN_SCL				23 
	#define SDA					(1<<PIN_SDA) 
	#define SCL					(1<<PIN_SCL) 
	#define I2C_PMUX_SDA		PORT_PMUX_C 
	#define I2C_PMUX_SCL		PORT_PMUX_C 
	#define I2C_GEN_SRC			GEN_MCK
	#define I2C_GEN_CLK			GEN_MCK_CLK
	#define I2C_BAUDRATE		400000 

	// ++++++++++++++	SPI	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//#define SPI					HW::SPI0
	#define PIO_SPCK			HW::PIOA
	#define PIO_MOSI			HW::PIOA
	#define PIO_MISO			HW::PIOA
	#define PIO_CS				HW::PIOB

	#define PIN_SPCK			9
	#define PIN_MOSI			8 
	#define PIN_MISO			10 
	#define PIN_CS0				10 
	#define PIN_CS1				11

	#define SPCK				(1<<PIN_SPCK) 
	#define MOSI				(1<<PIN_MOSI) 
	#define MISO				(1<<PIN_MISO) 
	#define CS0					(1<<PIN_CS0) 
	#define CS1					(1<<PIN_CS1) 

	#define SPI_PMUX_SPCK		PORT_PMUX_C 
	#define SPI_PMUX_MOSI		PORT_PMUX_C 
	#define SPI_PMUX_MISO		PORT_PMUX_C 
	#define SPI_DIPO_BITS		SPI_DIPO(2)
	#define SPI_DOPO_BITS		SPI_DOPO(0) 

	#define SPI_GEN_SRC			GEN_MCK
	#define SPI_GEN_CLK			GEN_MCK_CLK
	#define SPI_BAUDRATE		8000000

	#define Pin_SPI_IRQ_Set() HW::PIOB->BSET(15)		
	#define Pin_SPI_IRQ_Clr() HW::PIOB->BCLR(15)		

	// ++++++++++++++	MANCH	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	
	#define MAN_TRANSMIT_V2

	#define PIO_MANCH		HW::PIOC
	#define PIN_L1			13 
	#define PIN_L2			14 
	#define PIN_H1			 
	#define PIN_H2			
	#define MANCH_PMUX		PORT_PMUX_F
	#define L1_WO_NUM		3
	#define L2_WO_NUM		4

	#define L1				(1UL<<PIN_L1)
	#define L2				(1UL<<PIN_L2)
	#define H1				0
	#define H2				0

	#define PIO_RXD			HW::PIOB
	#define PIN_RXD			23
	#define RXD				(1UL<<PIN_RXD)

	#define Pin_ManRcvIRQ_Set()	HW::PIOA->BSET(24)
	#define Pin_ManRcvIRQ_Clr()	HW::PIOA->BCLR(24)

	#define Pin_ManTrmIRQ_Set()	HW::PIOB->BSET(21)		
	#define Pin_ManTrmIRQ_Clr()	HW::PIOB->BCLR(21)		

	#define Pin_ManRcvSync_Set()	HW::PIOB->BSET(18)		
	#define Pin_ManRcvSync_Clr()	HW::PIOB->BCLR(18)		

	// ++++++++++++++	NAND Flash	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define PIO_WP			HW::PIOB 
	#define PIO_FLREADY		HW::PIOB
	#define PIO_FCS			HW::PIOB
	#define PIO_NAND_DATA	HW::PIOA
	#define PIO_ALE			HW::PIOB 
	#define PIO_CLE			HW::PIOB 
	#define PIO_WE_RE		HW::PIOB 

	#define PIN_WP			24 
	#define PIN_FLREADY		25 
	#define PIN_FCS0		2 
	#define PIN_FCS1		3 
	#define PIN_FCS2		4 
	#define PIN_FCS3		5 
	#define PIN_WE			30 
	#define PIN_RE			31 
	#define PIN_ALE			0 
	#define PIN_CLE			1 

	#define WP				(1<<PIN_WP) 
	#define FLREADY			(1UL<<PIN_FLREADY) 
	#define FCS0			(1<<PIN_FCS0) 
	#define FCS1			(1<<PIN_FCS1) 
	#define FCS2			(1<<PIN_FCS2) 
	#define FCS3			(1<<PIN_FCS3) 
	#define WE				(1UL<<PIN_WE) 
	#define RE				(1UL<<PIN_RE) 
	#define ALE				(1UL<<PIN_ALE) 
	#define CLE				(1UL<<PIN_CLE) 

	#define PIN_WE_CFG		PINGFG_DRVSTR 
	#define PIN_RE_CFG		PINGFG_DRVSTR 
	#define PIN_ALE_CFG		PINGFG_DRVSTR 
	#define PIN_CLE_CFG		PINGFG_DRVSTR 

	#define NAND_DELAY_WP()		{ delay(4);				}
	#define NAND_DELAY_WH()		{ delay(4);				}
	#define NAND_DELAY_RP()		{ delay(4);				}
	#define NAND_DELAY_REH()	{ delay(4);				}
	#define NAND_DELAY_WHR()	{ delay(10);			}
	#define NAND_DELAY_ADL()	{ delay(20);			}
	#define NAND_DELAY_PR()		{ delay(4);				}
	#define NAND_DELAY_FEAT()	{ delay(NS2CLK(1000));	}

	#define NAND_WE_PER		NS2CLK(200)-1	
	#define NAND_WE_CC0		NS2CLK(100) 
	#define NAND_WE_CC1		NS2CLK(100)

	#define nandTCC			HW::NAND_TCC
	//#define nandTC			HW::NAND_TC

	#ifdef nandTCC
	
		#define NAND_RE_PER_SLOW	(NS2CLK(200)-1)
		#define NAND_RE_CC0_SLOW	NS2CLK(110) 
		#define NAND_RE_CC1_SLOW	NS2CLK(100)

		#define NAND_RE_PER_FAST	(NS2CLK(100)-1)
		#define NAND_RE_CC0_FAST	NS2CLK(55) 
		#define NAND_RE_CC1_FAST	NS2CLK(50)

		#define WE_PORT_PMUX	(PORT_PMUX_F) 
		#define RE_PORT_PMUX	(PORT_PMUX_F) 

		inline void NAND_ClockEnable()  { HW::GCLK->PCHCTRL[CONCAT2(GCLK_,NAND_TCC)] = GCLK_GEN(CONCAT2(GEN_,NAND_TCC))|GCLK_CHEN; HW::MCLK->ClockEnable(CONCAT2(PID_,NAND_TCC)); }

		#define NAND_TRIGSRC_MC0  CONCAT3(DMCH_TRIGSRC_,NAND_TCC,_MC0)
		#define NAND_TRIGSRC_MC1  CONCAT3(DMCH_TRIGSRC_,NAND_TCC,_MC1)

		#define NAND_EVENT_GEN		CONCAT2(EVGEN_DMAC_CH_, NAND_DMA_CH)
		#define NAND_EVSYS_USER		CONCAT3(EVSYS_USER_,NAND_TCC,_EV_1)

	#else

		#define NAND_RE_PER		250	
		//#define NAND_RE_CC0		NS2CLK(35) 
		#define NAND_RE_CC1		NS2CLK(25)

		#define WE_PORT_PMUX	(PORT_PMUX_E) 
		#define RE_PORT_PMUX	(PORT_PMUX_E) 

		inline void NAND_ClockEnable()  { HW::GCLK->PCHCTRL[CONCAT2(GCLK_,NAND_TC)] = GCLK_GEN(CONCAT2(GEN_,NAND_TC))|GCLK_CHEN; HW::MCLK->ClockEnable(CONCAT2(PID_,NAND_TC)); }

		#define NAND_TRIGSRC_MC0	CONCAT3(DMCH_TRIGSRC_,NAND_TC,_MC0)
		#define NAND_TRIGSRC_MC1	CONCAT3(DMCH_TRIGSRC_,NAND_TC,_MC1)

		#define NAND_EVENT_GEN		EVGEN_DMAC_CH_0
		#define NAND_EVSYS_USER		CONCAT3(EVSYS_USER_,NAND_TC,_EVU)

	#endif

	// ++++++++++++++	VCORE	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//#define PIO_ENVCORE		HW::PIOC
	//#define PIN_ENVCORE		14 
	//#define ENVCORE			(1<<PIN_ENVCORE) 
	
	// ++++++++++++++	RESET	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//#define PIN_RESET		10
	//#define PIO_RESET		HW::PIOC
	//#define RESET			(1<<PIN_RESET)

	// ++++++++++++++	UART TRM	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define PIO_UTXD4			HW::PIOB 
	#define PIO_URXD4			HW::PIOB 
	#define PIO_RTS4			HW::PIOB 
	#define PMUX_UTXD4			PORT_PMUX_C
	#define PMUX_URXD4			PORT_PMUX_C 
	#define UART4_TXPO			USART_TXPO_0 
	#define UART4_RXPO			USART_RXPO_1 
	#define PIN_UTXD4			12 
	#define PIN_URXD4			13 
	#define PIN_RTS4			14
	#define UTXD4				(1<<PIN_UTXD4) 
	#define URXD4				(1<<PIN_URXD4) 
	#define RTS4				(1<<PIN_RTS4) 
	#define UART4_GEN_SRC		GEN_MCK
	#define UART4_GEN_CLK		GEN_MCK_CLK

	// ++++++++++++++	UART RCV	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define PIO_UTXD7			HW::PIOB
	#define PIO_URXD7			HW::PIOB 
	#define PIO_RTS7			HW::PIOC 
	#define PMUX_UTXD7			PORT_PMUX_D 
	#define PMUX_URXD7			PORT_PMUX_D 
	#define UART7_TXPO			USART_TXPO_0 
	#define UART7_RXPO			USART_RXPO_1 
	#define PIN_UTXD7			21 
	#define PIN_URXD7			20
	#define PIN_RTS7			5
	#define UTXD7				(1<<PIN_UTXD7) 
	#define URXD7				(1<<PIN_URXD7) 
	#define RTS7				(1<<PIN_RTS7) 
	#define UART7_GEN_SRC		GEN_MCK
	#define UART7_GEN_CLK		GEN_MCK_CLK

	// ++++++++++++++	CLOCK	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define PIO_RTCINT		HW::PIOC
	#define PIN_RTCINT		25 
	#define CLOCK_EXTINT	(PIN_RTCINT&15)
	#define CLOCK_IRQ		(EIC_0_IRQ+(PIN_RTCINT&15))
	#define RTCINT			(1UL<<PIN_RTCINT) 

	// ++++++++++++++	EMAC	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define EMAC_PHYA		0

	#define PIO_RESET_PHY	HW::PIOC
	#define PIN_RESET_PHY	15

	#define PIO_GMD			HW::PIOA
	#define PIN_GMDC		20
	#define PIN_GMDIO		21

	#define GMDC			(1UL<<PIN_GMDC) 
	#define GMDIO			(1UL<<PIN_GMDIO) 

	// ++++++++++++++	ADC		++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define PIO_ARID			HW::PIOB
	#define PIO_AVMAN 			HW::PIOB

	#define PIN_ARID			8
	#define PIN_AVMAN			9 
	
	#define ARID				(1UL<<PIN_ARID	) 
	#define AVMAN				(1UL<<PIN_AVMAN	) 

	// ++++++++++++++	PIO INIT	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define PIOA_INIT_DIR		(PA11|PA16|PA24|PA25|PA27)
	#define PIOA_INIT_SET		(0)
	#define PIOA_INIT_CLR		(PA11|PA16|PA24|PA25|PA27)
	#define PIOA_TEST_MASK		(~(PA12|PA13|PA14|PA15|PA26|PA28|PA29|PA30|PA31))

	#define PIOB_INIT_DIR		(PB06|PB07|RTS4|PB15|PB16|PB17|PB18)
	#define PIOB_INIT_SET		(0)
	#define PIOB_INIT_CLR		(PB06|PB07|RTS4|PB15|PB16|PB17|PB18)
	#define PIOB_TEST_MASK		(~(PB08|PB09|PB13|PB19|PB20|PB22|PB23|PB26|PB27|PB28|PB29))

	#define PIOC_INIT_DIR		(PC00|PC01|PC02|PC03|RTS7|PC06|PC07|PC10|PC11|PC12|L1|L2|PC16|PC17|PC18|PC19|PC21|PC24|PC26|PC27|PC28)
	#define PIOC_INIT_SET		(0)
	#define PIOC_INIT_CLR		(PC00|PC01|PC02|PC03|RTS7|PC06|PC07|PC10|PC11|PC12|L1|L2|PC16|PC17|PC18|PC19|PC21|PC24|PC26|PC27|PC28)
	#define PIOC_TEST_MASK		(~(PC04|PC08|PC09|PC20|PC21|PC22|PC23|PC25|PC29|PC30|PC31))

	#define TEST_PIN_DELAY		(MCK_MHz*1)

	#define Pin_MainLoop_Set()	HW::PIOA->BSET(25)
	#define Pin_MainLoop_Clr()	HW::PIOA->BCLR(25)
	#define Pin_MainLoop_Tgl()	HW::PIOA->BTGL(25)
	

#elif defined(CPU_BF607) //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define FLASHSPI_EXTWDT_TIMEOUT	(MS2CTM(100))

	#define CLKIN_MHz			25

	#define CLKIN_DIV			1	// 1, 2

	#define PLL_MUL				16	// 1...128
	#define CCLK_DIV			1   // 1...32
	#define SCLK_DIV			2	// 1...32
	#define SCLK0_DIV			1	// 1...8
	#define SCLK1_DIV			1	// 1...8
	#define DCLK_DIV			32	// 1...32	DRAM clock
	#define OCLK_DIV			1	// 1...128  Output clock

	#define VCO_CLK_MHz 		(CLKIN_MHz*PLL_MUL/CLKIN_DIV)
	#define CCLK_MHz			(VCO_CLK_MHz/CCLK_DIV)
	#define SCLK_MHz			(VCO_CLK_MHz/SCLK_DIV)
	#define SCLK0_MHz			(SCLK_MHz/SCLK0_DIV)
	#define SCLK1_MHz			(SCLK_MHz/SCLK1_DIV)

	#if (VCO_CLK_MHz < 250) || (VCO_CLK_MHz > 1000)
	#error VCO_CLK_MHz must be 250...1000
	#endif

	#if (CCLK_MHz > 500)
	#error CCLK_MHz must be <= 500
	#endif

	#if (SCLK_MHz > 250)
	#error SCLK_MHz must be <= 250
	#endif

	//#define VRCTL_VALUE         0x0000

	#if CLKIN_DIV == 2
	#define CGUCTL_VALUE        (CGU_CTL_MSEL(PLL_MUL)|CGU_CTL_DF)
	#else
	#define CGUCTL_VALUE        (CGU_CTL_MSEL(PLL_MUL))
	#endif

	#define CCLK (CCLK_MHz*1000000)
	#define SCLK (SCLK_MHz*1000000)
	#define SCLK0 (SCLK0_MHz*1000000)
	#define SCLK1 (SCLK1_MHz*1000000)

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define MS2SCLK(x) ((u64)((x)*SCLK_MHz*1000))
	#define US2SCLK(x) ((u64)((x)*SCLK_MHz))
	#define NS2SCLK(x) ((u64)(((x)*SCLK_MHz+500)/1000))

	#define MS2SCLK0(x) ((u64)((x)*SCLK0_MHz*1000))
	#define US2SCLK0(x) ((u64)((x)*SCLK0_MHz))
	#define NS2SCLK0(x) ((u64)(((x)*SCLK0_MHz+500)/1000))

	#define MS2SCLK1(x) ((u64)((x)*SCLK1_MHz*1000))
	#define US2SCLK1(x) ((u64)((x)*SCLK1_MHz))
	#define NS2SCLK1(x) ((u64)(((x)*SCLK1_MHz+500)/1000))

	#define MS2CCLK(x) ((u64)((x)*CCLK_MHz*1000))
	#define US2CCLK(x) ((u64)((x)*CCLK_MHz))
	#define NS2CCLK(x) ((u64)(((x)*CCLK_MHz+500)/1000))

	#define MS2CLK(x) MS2SCLK(x)
	#define US2CLK(x) US2SCLK(x)
	#define NS2CLK(x) NS2SCLK(x)

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



	//  1 - PC10 - UpdateMode 
	//  2 - PC09 - SPORT0_ISR
	//  3 - PC08 - SPORT1_ISR
	//  5 - PC07 - SPORT1_ISR
	//  6 - PC06 - ProcessSPORT
	//  7 - PC05 - 
	//  8 - PC04 - PreProcessSPORT
	// 44 - PB05 - Main Loop
	// 45 - PB04 - 
	// 58 - PA13 - ProcessSPORT

	#define Pin_UpdateMode_Set()		HW::PIOC->BSET(10)
	#define Pin_UpdateMode_Clr()		HW::PIOC->BCLR(10)

	#define Pin_SPORT0_ISR_Set()		HW::PIOC->BSET(9)
	#define Pin_SPORT0_ISR_Clr()		HW::PIOC->BCLR(9)

	#define Pin_SPORT1_ISR_Set()		HW::PIOC->BSET(8)
	#define Pin_SPORT1_ISR_Clr()		HW::PIOC->BCLR(8)

	#define Pin_ProcessSPORT_Set()		HW::PIOA->BSET(13)
	#define Pin_ProcessSPORT_Clr()		HW::PIOA->BCLR(13)

	#define Pin_PreProcessSPORT_Set()	HW::PIOC->BSET(4)
	#define Pin_PreProcessSPORT_Clr()	HW::PIOC->BCLR(4)

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	// ++++++++++++++	DMA	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define	SPI_DMA_TX_CH		0
	#define	SPI_DMA_RX_CH		1
	#define	I2C_DMA_CH			2
	#define	NAND_DMA_CH			3
	#define	UART7_DMA_CH		4
	#define	UART4_DMA_CH		5
	#define	NAND_MEMCOPY_DMA_CH	6
	//#define	DSP_SPI_DMA_RX	8

	//#define	DSP_SPI_DMA_TX	30
	#define	CRC_DMA_CH			31

	// ++++++++++++++	TIMER	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define MANT_TMR		0
	#define MANR_TMR		1
	//#define 				2
	//#define 				3
	//#define 				4
	//#define 				5
	//#define 				6
	//#define 				7

	// ++++++++++++++	I2C	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define I2C_GEN_SRC			GEN_MCK
	#define I2C_GEN_CLK			GEN_MCK_CLK
	#define I2C_BAUDRATE		400000 

	// ++++++++++++++	SPI	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	//#define SPI					HW::SPI0
	#define PIO_SPCK			HW::PIOD
	#define PIO_MOSI			HW::PIOD
	#define PIO_MISO			HW::PIOD
	#define PIO_CS0				HW::PIOC
	#define PIO_CS1				HW::PIOC

	#define PIN_SPCK			5
	#define PIN_MOSI			13 
	#define PIN_MISO			14 
	#define PIN_CS0				5 
	#define PIN_CS1				1

	#define SPCK				(1<<PIN_SPCK) 
	#define MOSI				(1<<PIN_MOSI) 
	#define MISO				(1<<PIN_MISO) 
	#define CS0					(1<<PIN_CS0) 
	#define CS1					(1<<PIN_CS1) 

	#define SPI_GEN_SRC			GEN_MCK
	#define SPI_GEN_CLK			GEN_MCK_CLK
	#define SPI_BAUDRATE		8000000
	#define SPI_CS0_BAUD		8000000
	#define SPI_CS1_BAUD		8000000

	#define SPI_CS0_MODE		0
	#define SPI_CS1_MODE		0

	// ++++++++++++++	MANCH	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	
	#define MAN_TRANSMIT_V1

	#define PIO_MANCH		HW::PIOG
	#define PIN_L1			12 
	#define PIN_L2			 
	#define PIN_H1			 
	#define PIN_H2			

	#define L1				(1UL<<PIN_L1)
	#define L2				0
	#define H1				0
	#define H2				0

	#define PIO_RXD			HW::PIOG
	#define PINT_RXD		HW::PINT5
	#define PIN_RXD			10
	#define RXD				(1UL<<PIN_RXD)

	#define Pin_ManRcvIRQ_Set()	//HW::PIOA->BSET(24)
	#define Pin_ManRcvIRQ_Clr()	//HW::PIOA->BCLR(24)

	#define Pin_ManTrmIRQ_Set()	//HW::PIOB->BSET(21)		
	#define Pin_ManTrmIRQ_Clr()	//HW::PIOB->BCLR(21)		

	#define Pin_ManRcvSync_Set()	//HW::PIOB->BSET(18)		
	#define Pin_ManRcvSync_Clr()	//HW::PIOB->BCLR(18)		

	// ++++++++++++++	NAND Flash	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define PIO_WP			HW::PIOB 
	#define PIO_FLREADY		HW::PIOB
	#define PIO_FCS			HW::PIOA

	#define PIN_WP			1 
	#define PIN_FLREADY		2 
	#define PIN_FCS0		0 
	#define PIN_FCS1		1 
	#define PIN_FCS2		2 
	#define PIN_FCS3		3 

	#define WP				(1<<PIN_WP) 
	#define FLREADY			(1UL<<PIN_FLREADY) 
	#define FCS0			(1<<PIN_FCS0) 
	#define FCS1			(1<<PIN_FCS1) 
	#define FCS2			(1<<PIN_FCS2) 
	#define FCS3			(1<<PIN_FCS3) 

	#define PIN_WE_CFG		PINGFG_DRVSTR 
	#define PIN_RE_CFG		PINGFG_DRVSTR 
	#define PIN_ALE_CFG		PINGFG_DRVSTR 
	#define PIN_CLE_CFG		PINGFG_DRVSTR 

	#define NAND_DELAY_WP()		{ delay(4);				}
	#define NAND_DELAY_WH()		{ delay(4);				}
	#define NAND_DELAY_RP()		{ delay(4);				}
	#define NAND_DELAY_REH()	{ delay(4);				}
	#define NAND_DELAY_WHR()	{ delay(10);			}
	#define NAND_DELAY_ADL()	{ delay(20);			}
	#define NAND_DELAY_PR()		{ delay(4);				}
	#define NAND_DELAY_FEAT()	{ delay(NS2CLK(1000));	}

	#define NAND_WE_PER		NS2CLK(200)-1	
	#define NAND_WE_CC0		NS2CLK(100) 
	#define NAND_WE_CC1		NS2CLK(100)

	#define nandTCC			HW::NAND_TCC
	//#define nandTC			HW::NAND_TC

	#ifdef nandTCC
	
		#define NAND_RE_PER_SLOW	(NS2CLK(200)-1)
		#define NAND_RE_CC0_SLOW	NS2CLK(110) 
		#define NAND_RE_CC1_SLOW	NS2CLK(100)

		#define NAND_RE_PER_FAST	(NS2CLK(100)-1)
		#define NAND_RE_CC0_FAST	NS2CLK(55) 
		#define NAND_RE_CC1_FAST	NS2CLK(50)

		#define WE_PORT_PMUX	(PORT_PMUX_F) 
		#define RE_PORT_PMUX	(PORT_PMUX_F) 

		//inline void NAND_ClockEnable()  { HW::GCLK->PCHCTRL[CONCAT2(GCLK_,NAND_TCC)] = GCLK_GEN(CONCAT2(GEN_,NAND_TCC))|GCLK_CHEN; HW::MCLK->ClockEnable(CONCAT2(PID_,NAND_TCC)); }

		#define NAND_TRIGSRC_MC0  CONCAT3(DMCH_TRIGSRC_,NAND_TCC,_MC0)
		#define NAND_TRIGSRC_MC1  CONCAT3(DMCH_TRIGSRC_,NAND_TCC,_MC1)

		#define NAND_EVENT_GEN		CONCAT2(EVGEN_DMAC_CH_, NAND_DMA_CH)
		#define NAND_EVSYS_USER		CONCAT3(EVSYS_USER_,NAND_TCC,_EV_1)

	#else

		#define NAND_RE_PER		250	
		//#define NAND_RE_CC0		NS2CLK(35) 
		#define NAND_RE_CC1		NS2CLK(25)

		#define WE_PORT_PMUX	(PORT_PMUX_E) 
		#define RE_PORT_PMUX	(PORT_PMUX_E) 

		inline void NAND_ClockEnable()  { HW::GCLK->PCHCTRL[CONCAT2(GCLK_,NAND_TC)] = GCLK_GEN(CONCAT2(GEN_,NAND_TC))|GCLK_CHEN; HW::MCLK->ClockEnable(CONCAT2(PID_,NAND_TC)); }

		#define NAND_TRIGSRC_MC0	CONCAT3(DMCH_TRIGSRC_,NAND_TC,_MC0)
		#define NAND_TRIGSRC_MC1	CONCAT3(DMCH_TRIGSRC_,NAND_TC,_MC1)

		#define NAND_EVENT_GEN		EVGEN_DMAC_CH_0
		#define NAND_EVSYS_USER		CONCAT3(EVSYS_USER_,NAND_TC,_EVU)

	#endif

	// ++++++++++++++	UART TRM	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define PIO_UTXD4			HW::PIOD 
	#define PIO_URXD4			HW::PIOD 
	#define PIO_RTS4			HW::PIOD 
	//#define PMUX_UTXD4			PORT_PMUX_C
	//#define PMUX_URXD4			PORT_PMUX_C 
	//#define UART4_TXPO			USART_TXPO_0 
	//#define UART4_RXPO			USART_RXPO_1 
	#define PIN_UTXD4			7 
	#define PIN_URXD4			8 
	#define PIN_RTS4			15
	#define UTXD4				(1<<PIN_UTXD4) 
	#define URXD4				(1<<PIN_URXD4) 
	#define RTS4				(1<<PIN_RTS4) 
	#define UART4_GEN_SRC		GEN_MCK
	#define UART4_GEN_CLK		GEN_MCK_CLK

	// ++++++++++++++	UART RCV	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define PIO_UTXD7			HW::PIOG
	#define PIO_URXD7			HW::PIOG 
	#define PIO_RTS7			HW::PIOB
	//#define PMUX_UTXD7			PORT_PMUX_D 
	//#define PMUX_URXD7			PORT_PMUX_D 
	//#define UART7_TXPO			USART_TXPO_0 
	//#define UART7_RXPO			USART_RXPO_1 
	#define PIN_UTXD7			15 
	#define PIN_URXD7			14
	#define PIN_RTS7			12
	#define UTXD7				(1<<PIN_UTXD7) 
	#define URXD7				(1<<PIN_URXD7) 
	#define RTS7				(1<<PIN_RTS7) 
	#define UART7_GEN_SRC		GEN_MCK
	#define UART7_GEN_CLK		GEN_MCK_CLK


	// ++++++++++++++	CLOCK	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define PIO_RTCINT		HW::PIOC
	#define PIN_RTCINT		25 
	#define CLOCK_EXTINT	(PIN_RTCINT&15)
	#define CLOCK_IRQ		(EIC_0_IRQ+(PIN_RTCINT&15))
	#define RTCINT			(1UL<<PIN_RTCINT) 

	// ++++++++++++++	EMAC	++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define EMAC_PHYA		0
	#define HWEMAC_NUM		1

	#define PIO_RESET_PHY	HW::PIOF
	#define PIN_RESET_PHY	0
	#define RESET_PHY		(1<<PIN_RESET_PHY) 

	#define PIO_GMD			HW::PIOA
	#define PIN_GMDC		20
	#define PIN_GMDIO		21

	#define GMDC			(1UL<<PIN_GMDC) 
	#define GMDIO			(1UL<<PIN_GMDIO) 


	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define INIT_PORTA_MUX				0x0000						
	#define INIT_PORTB_MUX				0x0000						
	#define INIT_PORTC_MUX				0x0000						
	#define INIT_PORTD_MUX				0x0000						
	#define INIT_PORTE_MUX				0x0000						
	#define INIT_PORTF_MUX				0x0000						
	#define INIT_PORTG_MUX				0x0000						

	#define INIT_PORTA_FER 				0x0000						
	#define INIT_PORTB_FER 				0x0000						
	#define INIT_PORTC_FER 				0x0000						
	#define INIT_PORTD_FER 				0x0000						
	#define INIT_PORTE_FER 				0x0000						
	#define INIT_PORTF_FER 				0x0000						
	#define INIT_PORTG_FER 				0x0000						

	#define INIT_PORTA_DIR 				(FCS0|FCS1|FCS2|FCS3|PA4|PA6|PA8|PA10|PA12|PA14)
	#define INIT_PORTB_DIR 				(RTS7|PB7|PB10)
	#define INIT_PORTC_DIR 				(CS0|CS1|PC0|PC2|PC4|PC6|PC8|PC10|PC12|PC14)
	#define INIT_PORTD_DIR 				(RTS4)
	#define INIT_PORTE_DIR 				(PE0|PE1|PE4)
	#define INIT_PORTF_DIR 				(RESET_PHY|PF1|PF2|PF3|PF4|PF5|PF7|PF9|PF11|PF13|PF15)
	#define INIT_PORTG_DIR 				0

	#define INIT_PORTA_INEN 			0x0000		
	#define INIT_PORTB_INEN 			0x0000		
	#define INIT_PORTC_INEN 			0x0000		
	#define INIT_PORTD_INEN 			0x0000		
	#define INIT_PORTE_INEN 			0x0000		
	#define INIT_PORTF_INEN 			0x0000		
	#define INIT_PORTG_INEN 			0x0000		

	#define INIT_PORTA_DATA 			(FCS0|FCS1|FCS2|FCS3)
	#define INIT_PORTB_DATA 			0
	#define INIT_PORTC_DATA 			(CS0|CS1)
	#define INIT_PORTD_DATA 			(RTS4)
	#define INIT_PORTE_DATA 			0
	#define INIT_PORTF_DATA 			0
	#define INIT_PORTG_DATA 			0

	#define INIT_PADS_EMAC_PTP_CLKSEL	0
	#define INIT_PADS_PORTS_HYST		0
	#define INIT_PADS_TWI_VSEL			0

	#ifndef __DEBUG
	#define INIT_WDOG_CNT			MS2SCLK0(100)
	#define INIT_WDOG_CTL			WDOG_WDEN
	#else
	#define INIT_WDOG_CNT			MS2SCLK0(100)
	#define INIT_WDOG_CTL			WDOG_WDDIS
	#endif

	#define PIO_MAINLOOP			HW::PIOA
	#define PIN_MAINLOOP			4

	#define PIO_CORE1MAINLOOP		HW::PIOB
	#define PIN_CORE1MAINLOOP		10

	#define PIO_VERIFY_PAGE_ERROR	HW::PIOC
	#define PIN_VERIFY_PAGE_ERROR	4

	#define Pin_MainLoop_Set()	PIO_MAINLOOP->BSET(PIN_MAINLOOP)
	#define Pin_MainLoop_Clr()	PIO_MAINLOOP->BCLR(PIN_MAINLOOP)
	#define Pin_MainLoop_Tgl()	PIO_MAINLOOP->BTGL(PIN_MAINLOOP)

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


#elif defined(WIN32) //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	#define BAUD2CLK(x)				(x)
	#define MT(v)					(v)
	#define Pin_MainLoop_Set()	
	#define Pin_MainLoop_Clr()	

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++




#endif // G_HW_CONF_H__27_12_2023__12_00
