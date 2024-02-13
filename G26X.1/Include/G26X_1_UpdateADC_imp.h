#ifndef G26X_1_UPDATEADC_IMP_H__13_02_2024__18_16
#define G26X_1_UPDATEADC_IMP_H__13_02_2024__18_16

#pragma once

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u16 netResist = 0;
u16 fltResist = 0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u16 GetNetAdr()
{
	return 1 + netResist/1024;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void UpdateADC()
{
	static byte i = 0;
	static DSCI2C dsc;
	static byte wbuf[4];
	static byte rbuf[4];
	static byte adr = 0x28;
	static CTM32 tm;
	static u32 filtFV = ~0;

	switch (i)
	{
		case 0:

			if (tm.Check(US2CTM(500)))
			{
				wbuf[0] = 0x20;	

				dsc.adr = 0x28|(adr&1);
				dsc.wdata = wbuf;
				dsc.wlen = 1;
				dsc.rdata = rbuf;
				dsc.rlen = 2;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				I2C_AddRequest(&dsc);

				i++;
			};

			break;

		case 1:

			if (dsc.ready)
			{
				if (dsc.ack)
				{
					byte *p = rbuf;

					for (u32 i = dsc.readedLen; i > 0; i -= 2, p += 2)
					{
						byte ch = (p[0] >> 4) & 3;

						u16 res = ((p[0]<<8)|p[1]) & 0xFFF;

						if (ch == 1)
						{
							if (filtFV == ~0) 
							{
								filtFV = res << 4;
							}
							else
							{
								filtFV += res - (filtFV>>4);
							};

							fltResist = (filtFV * 941 + 2048) / 4096; //51869
							netResist = (res * 941 + 128) / 256; 
						};
					};
				}
				else
				{
					adr ^= 1;
				};

				i = 0;
			};

			break;
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#endif // G26X_1_UPDATEADC_IMP_H__13_02_2024__18_16
