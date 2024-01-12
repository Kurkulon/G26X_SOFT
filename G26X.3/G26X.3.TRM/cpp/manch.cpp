#include "types.h"
#include "core.h"
//#include "COM_DEF.h"
//#include "CRC16_8005.h"

//#include "hardware.h"

#include "SEGGER_RTT.h"
#include "hardware.h"

#ifdef CPU_SAME53	
#define MAN_TRANSMIT_V2
#elif defined(CPU_XMC48)
#define MAN_TRANSMIT_V2
#endif


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef MAN_SERCOM_NUM

	#ifdef CPU_SAME53	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

		#ifdef MAN_SERCOM_NUM

			#define MANT_GEN					GEN_1M
			#define MANT_GEN_CLK				GEN_1M_CLK 
			#define GCLK_MANT					CONCAT3(GCLK_SERCOM,MAN_SERCOM_NUM,_CORE)
			#define PID_MANT					CONCAT2(PID_SERCOM,MAN_SERCOM_NUM)

			#define MANTSPI						CONCAT2(HW::SPI,MAN_SERCOM_NUM)

		#else
			#error  Must defined MANT_TC or MANT_TCC
		#endif
		
		#define BAUD2CLK(x)						((u32)(MANT_GEN_CLK/(2*(x))+0.5)-1)

		inline void MANT_ClockEnable()			{ HW::GCLK->PCHCTRL[GCLK_MANT] = MANT_GEN|GCLK_CHEN; HW::MCLK->ClockEnable(PID_MANT); }

	#endif

#endif // #ifdef MAN_SERCOM_NUM

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <manch_imp.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef MAN_SERCOM_NUM

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define BIT2M(v,b) ((((v)>>(b))&1+1)<<((b)*2))
#define B2M(v) (BIT2M(v,0)|BIT2M(v,1)|BIT2M(v,2)|BIT2M(v,3)|BIT2M(v,4)|BIT2M(v,5)|BIT2M(v,6)|BIT2M(v,7))

static u16 byte2man[256] = { 
	
	B2M(0x00), B2M(0x01), B2M(0x02), B2M(0x03), B2M(0x04), B2M(0x05), B2M(0x06), B2M(0x07), B2M(0x08), B2M(0x09), B2M(0x0A), B2M(0x0B), B2M(0x0C), B2M(0x0D), B2M(0x0E), B2M(0x0F), 
	B2M(0x10), B2M(0x11), B2M(0x12), B2M(0x13), B2M(0x14), B2M(0x15), B2M(0x16), B2M(0x17), B2M(0x18), B2M(0x19), B2M(0x1A), B2M(0x1B), B2M(0x1C), B2M(0x1D), B2M(0x1E), B2M(0x1F), 
	B2M(0x20), B2M(0x21), B2M(0x22), B2M(0x23), B2M(0x24), B2M(0x25), B2M(0x26), B2M(0x27), B2M(0x28), B2M(0x29), B2M(0x2A), B2M(0x2B), B2M(0x2C), B2M(0x2D), B2M(0x2E), B2M(0x2F), 
	B2M(0x30), B2M(0x31), B2M(0x32), B2M(0x33), B2M(0x34), B2M(0x35), B2M(0x36), B2M(0x37), B2M(0x38), B2M(0x39), B2M(0x3A), B2M(0x3B), B2M(0x3C), B2M(0x3D), B2M(0x3E), B2M(0x3F), 
	B2M(0x40), B2M(0x41), B2M(0x42), B2M(0x43), B2M(0x44), B2M(0x45), B2M(0x46), B2M(0x47), B2M(0x48), B2M(0x49), B2M(0x4A), B2M(0x4B), B2M(0x4C), B2M(0x4D), B2M(0x4E), B2M(0x4F), 
	B2M(0x50), B2M(0x51), B2M(0x52), B2M(0x53), B2M(0x54), B2M(0x55), B2M(0x56), B2M(0x57), B2M(0x58), B2M(0x59), B2M(0x5A), B2M(0x5B), B2M(0x5C), B2M(0x5D), B2M(0x5E), B2M(0x5F), 
	B2M(0x60), B2M(0x61), B2M(0x62), B2M(0x63), B2M(0x64), B2M(0x65), B2M(0x66), B2M(0x67), B2M(0x68), B2M(0x69), B2M(0x6A), B2M(0x6B), B2M(0x6C), B2M(0x6D), B2M(0x6E), B2M(0x6F), 
	B2M(0x70), B2M(0x71), B2M(0x72), B2M(0x73), B2M(0x74), B2M(0x75), B2M(0x76), B2M(0x77), B2M(0x78), B2M(0x79), B2M(0x7A), B2M(0x7B), B2M(0x7C), B2M(0x7D), B2M(0x7E), B2M(0x7F), 
	B2M(0x80), B2M(0x81), B2M(0x82), B2M(0x83), B2M(0x84), B2M(0x85), B2M(0x86), B2M(0x87), B2M(0x88), B2M(0x89), B2M(0x8A), B2M(0x8B), B2M(0x8C), B2M(0x8D), B2M(0x8E), B2M(0x8F), 
	B2M(0x90), B2M(0x91), B2M(0x92), B2M(0x93), B2M(0x94), B2M(0x95), B2M(0x96), B2M(0x97), B2M(0x98), B2M(0x99), B2M(0x9A), B2M(0x9B), B2M(0x9C), B2M(0x9D), B2M(0x9E), B2M(0x9F), 
	B2M(0xA0), B2M(0xA1), B2M(0xA2), B2M(0xA3), B2M(0xA4), B2M(0xA5), B2M(0xA6), B2M(0xA7), B2M(0xA8), B2M(0xA9), B2M(0xAA), B2M(0xAB), B2M(0xAC), B2M(0xAD), B2M(0xAE), B2M(0xAF), 
	B2M(0xB0), B2M(0xB1), B2M(0xB2), B2M(0xB3), B2M(0xB4), B2M(0xB5), B2M(0xB6), B2M(0xB7), B2M(0xB8), B2M(0xB9), B2M(0xBA), B2M(0xBB), B2M(0xBC), B2M(0xBD), B2M(0xBE), B2M(0xBF), 
	B2M(0xC0), B2M(0xC1), B2M(0xC2), B2M(0xC3), B2M(0xC4), B2M(0xC5), B2M(0xC6), B2M(0xC7), B2M(0xC8), B2M(0xC9), B2M(0xCA), B2M(0xCB), B2M(0xCC), B2M(0xCD), B2M(0xCE), B2M(0xCF), 
	B2M(0xD0), B2M(0xD1), B2M(0xD2), B2M(0xD3), B2M(0xD4), B2M(0xD5), B2M(0xD6), B2M(0xD7), B2M(0xD8), B2M(0xD9), B2M(0xDA), B2M(0xDB), B2M(0xDC), B2M(0xDD), B2M(0xDE), B2M(0xDF), 
	B2M(0xE0), B2M(0xE1), B2M(0xE2), B2M(0xE3), B2M(0xE4), B2M(0xE5), B2M(0xE6), B2M(0xE7), B2M(0xE8), B2M(0xE9), B2M(0xEA), B2M(0xEB), B2M(0xEC), B2M(0xED), B2M(0xEE), B2M(0xEF), 
	B2M(0xF0), B2M(0xF1), B2M(0xF2), B2M(0xF3), B2M(0xF4), B2M(0xF5), B2M(0xF6), B2M(0xF7), B2M(0xF8), B2M(0xF9), B2M(0xFA), B2M(0xFB), B2M(0xFC), B2M(0xFD), B2M(0xFE), B2M(0xFF)
};

#ifndef WIN32

#pragma push
//#pragma O3
//#pragma Otime

static __irq void ManTrmIRQ_1()
{
	static u32 tw = 0;
	static u16 count = 0;
	//static byte i = 0;
	static const u16* data = 0;
	static u16 len = 0;
	static bool cmd = false;

	Pin_ManTrmIRQ_Set();

	switch (stateManTrans)
	{
		case 0:	// 1-st sync imp 

			data = manTB->data1;
			len = manTB->len1;
			cmd = false;
			stateManTrans++;

		case 1:

			if (len == 0)
			{
				data = manTB->data2;
				len = manTB->len2;
				manTB->len2 = 0;
			};

			if (len != 0)
			{
				tw = ((u32)(*data) << 1) | (CheckParity(*data) & 1);

				data++;
				len--;

				count = 17;

				//u32 tadd = (cmd) ? trmHalfPeriod : 0;

				//MANT_SET_CR(trmHalfPeriod3+tadd);

				//if (tw & 0x10000)
				//{
				//	MANT_SET_PR(trmHalfPeriod7+tadd); //US2MT(96);
				//	stateManTrans += 2;
				//}
				//else
				//{
				//	MANT_SET_PR(trmHalfPeriod6+tadd); //US2MT(72);
				//	stateManTrans++;
				//};

				//MANT_SHADOW_SYNC();

				tw <<= 1;
				count--;
			}
			else
			{
				stateManTrans = 4;
			};

			break;

		case 2:	

			//MANT_SET_CR(trmHalfPeriod);

			if (count == 0)
			{
			//	MANT_SET_PR(trmHalfPeriod2);
				cmd = false;
				stateManTrans = 1;
			}
			else
			{
				if (tw & 0x10000)
				{
					//MANT_SET_PR(trmHalfPeriod3);

					if (count == 1)
					{
						cmd = true;
						stateManTrans = 1;
					}
					else
					{
						stateManTrans++;
					};
				}
				else
				{
					//MANT_SET_PR(trmHalfPeriod2);
				};

				tw <<= 1;
				count--;
			};

			//MANT_SHADOW_SYNC();

			break;

		case 3: 

			if (tw & 0x10000)
			{
				//MANT_SET_CR(trmHalfPeriod);
				//MANT_SET_PR(trmHalfPeriod2);

				tw <<= 1;
				count--;

				if (count == 0)
				{
					cmd = true;
					stateManTrans = 1;
				};
			}
			else
			{
				tw <<= 1;
				count--;

				//MANT_SET_CR(trmHalfPeriod2);

				if (tw & 0x10000)
				{
					//MANT_SET_PR(trmHalfPeriod4);
					
					if (count == 1)
					{
						cmd = true;
						stateManTrans = 1;
					};
				}
				else
				{
					//MANT_SET_PR(trmHalfPeriod3);

					if (count == 0)
					{
						cmd = false;
						stateManTrans = 1;
					}
					else
					{
						stateManTrans--;
					}
				};

//				if (count != 0)
				{
					tw <<= 1;
					count--;
				};
			};

			//MANT_SHADOW_SYNC();

			break;

		case 4:

			stateManTrans++;
			break;

		case 5:

			stateManTrans = 0;

			//ManDisableTransmit();

			manTB->ready = true;
			trmBusy = false;

			break;
	}; // 	switch (stateManTrans)

	//ManEndIRQ();

	Pin_ManTrmIRQ_Clr();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static __irq void ManTrmIRQ_2()
{

}

#pragma pop

#endif
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

bool SendManData(MTB* mtb)
{
	#ifndef WIN32

		if (trmBusy || rcvBusy || mtb == 0 || mtb->data1 == 0 || mtb->len1 == 0)
		{
			return false;
		};

		mtb->ready = false;

		manTB = mtb;

		stateManTrans = 0;

		#ifdef CPU_SAME53	

			MANTSPI->BAUD = GetTrmBaudRate(mtb->baud);

			//MNTTCC->CTRLA = MANT_PRESC_DIV;
			//MNTTCC->WAVE = TCC_WAVEGEN_NPWM;//|TCC_POL0;
			//MNTTCC->DRVCTRL = TCC_INVEN0 << L2_WO_NUM;

			//MNTTCC->PER = US2MT(50)-1;
			//MNTTCC->CC[L1_CC_NUM] = 0; 
			//MNTTCC->CC[L2_CC_NUM] = ~0; 

			//MNTTCC->EVCTRL = 0;

			//MNTTCC->INTENCLR = ~0;
			//MNTTCC->INTENSET = TCC_OVF;
			//MNTTCC->INTFLAG = TCC_OVF;

			//MNTTCC->CTRLA |= TCC_ENABLE;
			//MNTTCC->CTRLBSET = TCC_CMD_RETRIGGER;

			//PIO_MANCH->SetWRCONFIG(L1|L2, MANCH_PMUX|PORT_WRPMUX|PORT_WRPINCFG|PORT_PMUXEN);
			//PIO_MANCH->CLR(L1|L2);
			//PIO_MANCH->DIRSET = L1|L2;

		#endif

		return trmBusy = true;

	#else

		mtb->ready = true;

		return true;

	#endif
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef WIN32

void InitManTransmit()
{
	using namespace HW;

	SEGGER_RTT_WriteString(0, RTT_CTRL_TEXT_BRIGHT_WHITE "Manchester Transmiter Init ... ");

#ifdef CPU_SAME53	

	byte MANT_IRQ = SERCOM0_0_IRQ + (MAN_SERCOM_NUM * 4);

	VectorTableExt[MANT_IRQ] = ManTrmIRQ_1;
	CM4::NVIC->CLR_PR(MANT_IRQ);
	CM4::NVIC->SET_ER(MANT_IRQ);

	VectorTableExt[MANT_IRQ+1] = ManTrmIRQ_2;
	CM4::NVIC->CLR_PR(MANT_IRQ+1);
	CM4::NVIC->SET_ER(MANT_IRQ+1);

	MANT_ClockEnable();

	PIO_MAN->SetWRCONFIG(1UL<<PIN_MAN, PORT_PMUX_D|PORT_PMUXEN|PORT_WRPINCFG|PORT_WRPMUX);

	MANTSPI->CTRLA = SPI_SWRST;

	while (MANTSPI->SYNCBUSY);

	MANTSPI->CTRLA = MAN_DOPO|SERCOM_MODE_SPI_MASTER;
	MANTSPI->CTRLB = 0;
	MANTSPI->CTRLC = 0;
	MANTSPI->BAUD = BAUD2CLK(20833);

#elif defined(CPU_XMC48)


#endif

	SEGGER_RTT_WriteString(0, "OK\n");
}

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#endif // #ifndef MAN_SERCOM_NUM
