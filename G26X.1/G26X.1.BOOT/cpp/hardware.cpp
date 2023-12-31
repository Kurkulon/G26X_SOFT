#include "hardware.h"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <system_imp.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <twi_imp.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

u16 netResist = 0;
u16 fltResist = 0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void InitHardware()
{
	LowLevelInit();

	InitTWI();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef BOOT_HW_UPDATE

void UpdateHardware()
{
	static byte i = 0;
	static DSCTWI dsc;
	static byte wbuf[4];
	static byte rbuf[4];
	static byte adr = 0x28;
	static RTM32 tm;
	static i32 filtFV = 0;

	switch (i)
	{
		case 0:

			if (tm.Check(US2RT(500)))
			{
				wbuf[0] = 0x20;	

				dsc.adr = 0x28|(adr&1);
				dsc.wdata = wbuf;
				dsc.wlen = 1;
				dsc.rdata = rbuf;
				dsc.rlen = 2;
				dsc.wdata2 = 0;
				dsc.wlen2 = 0;

				TWI_AddRequest(&dsc);

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

						i32 res = ((p[0]<<8)|p[1]) & 0xFFF;

						if (ch == 1)
						{
							filtFV += res - (filtFV+32)/64;

							fltResist = (filtFV * 15179 + 32768) / 65536; //51869
							netResist = (res * 15179 + 512) / 1024; 
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

#endif
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef BOOT_NETADR

u16 GetNetAdr()
{
	return netResist/1024;
}

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
