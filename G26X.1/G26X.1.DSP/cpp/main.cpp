#include "hardware.h"
#include "ComPort\ComPort.h"
#include "CRC\CRC16.h"
#include "CRC\CRC16_CCIT.h"
#include "FLASH\at25df021.h"
#include "BOOT\boot_req.h"
//#include "G_RCV.h"
#include "list.h"
#include "pack.h"
#include <builtins.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
static void CheckFlash();

enum { VERSION = 0x101 };

static u16 numDevice = 0;
static u16 numDevValid = 0;
//static u16 temp = 0;
static u16 flashStatus = 0;

static const u16 bootReqWord = RCV_BOOT_REQ_WORD;
static const u16 bootReqMask = RCV_BOOT_REQ_MASK;

//#include <bfrom.h>

static ComPort com;
//static ComPort::WriteBuffer wb;
//
//static byte data[256*48];

//static u16 spd[2][1024*2];
//static byte spTime[RCV_FIRE_NUM];
//static byte spGain[RCV_FIRE_NUM];
//static u16	spLen[RCV_FIRE_NUM];
//static u16	spDelay[RCV_FIRE_NUM];

static u16	maxAmp[4];
static u16	power[4];


//static u16 spd2[512*2];
//
//static i16 ch1[512];
//static i16 ch2[512];
//static i16 ch3[512];
//static i16 ch4[512];

//static bool ready1 = false, ready2 = false;

//static u32 CRCOK = 0;
//static u32 CRCER = 0;

//static byte sampleTime[RCV_FIRE_NUM]	= { 10	};
static byte ngain[RCV_FIRE_NUM]			= { 0	};
//static u16 sampleLen[RCV_FIRE_NUM]		= { 512	};
//static u16 sampleDelay[RCV_FIRE_NUM]	= { 200	};

//static byte netAdr = 1;

//static U32u fadc = 0;

static byte fireN = 0;

static u16 flashCRC = 0;
static u32 flashLen = 0;
static bool flashOK = false;
static bool flashChecked = false;
static bool flashCRCOK = false;
static bool cmdSaveParams = false;
static bool cmdReboot = false;

//static u16 packType = 0;

static u16 lastErasedBlock = ~0;

static u32 curWriteReqAdr = 0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#if defined(RCV_TEST_WAVEPACK) && RCV_TEST_WAVEPACK == 12

static i16 wave1[1024] = { -1,-1,0,0,0,1,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,0,1,1,2,0,-6,-17,-34,-45,-33,25,132,261,339,281,37,-354,-747,-954,-836,-395,225,808,1170,1222,978,513,
     -61,-608,-992,-1106,-927,-523,-35,392,645,674,528,323,160,67,-6,-156,-392,-597,-673,-615,-468,-291,-84,232,658,1042,1201,1027,521,-162,-739,-942,-654,-55,419,
     376,-192,-843,-1024,-561,208,709,580,38,-344,-225,287,787,859,461,-63,-394,-425,-214,64,267,361,276,-50,-543,-1016,-1220,-968,-315,465,1076,1298,1074,518,-176,
     -742,-947,-745,-278,225,509,497,314,76,-129,-183,-16,343,745,874,460,-409,-1294,-1732,-1532,-908,-255,251,666,1082,1470,1617,1307,645,21,-255,-173,-73,-377,
     -1106,-1804,-1941,-1542,-482,462,1012,1225,1272,1249,1096,652,-88,-804,-1081,-800,-242,133,87,-140,-132,171,448,347,-158,-631,-568,15,622,754,248,-509,-796,
     -247,860,1843,1754,1255,-94,-1224,-1824,-1921,-1635,-1329,-1156,-1226,-1425,-1245,-1077,-921,-780,845,2048,2048,2048,1819,1608,1411,1229,1061,907,765,635,515,
     407,308,205,-881,-1879,-1935,-1935,-1935,-1940,-1817,-1572,-1371,-1179,-1005,-845,96,1284,2048,2048,1572,600,-195,-416,-90,466,841,812,480,105,-106,-58,207,534,
     781,873,802,603,341,98,-29,76,460,1037,1640,1967,1737,1536,652,-132,-450,-138,486,1007,940,791,157,-558,-1046,-1208,-1040,-714,-392,-141,-76,-282,-686,-1140,
     -1499,-1675,-1690,-1587,-1425,-1295,-1287,-1497,-1695,-1454,-1256,-1075,-908,-756,-567,-88,42,78,247,662,1190,1680,2048,2048,1783,1117,455,124,269,761,1399,2045,
     2048,2048,1821,1595,1385,1193,1016,852,702,565,452,-268,-775,-1195,-1649,-1937,-1935,-1935,-1609,-734,-20,228,44,-283,-460,-459,-467,-598,-825,-1099,-1445,-1881,
     -1921,-1630,-1433,-1239,-1061,-897,-747,-610,-258,492,1101,1633,2012,2048,2048,2048,2048,2048,1676,807,-121,-744,-921,-658,-180,269,669,1021,1214,1171,899,512,285,
     446,943,870,731,603,485,-218,-1343,-1934,-1935,-1935,-1935,-1937,-1556,-1512,-1759,-1534,-1334,-1152,-644,671,1682,2048,2038,1579,901,111,-609,-1113,-1276,-1033,
     -578,-203,-36,-109,-331,-414,-93,680,1768,2048,2048,1957,1740,1525,1335,230,-511,-760,-575,-233,-70,-288,-829,-1356,-1518,-1190,-477,346,996,1398,1258,1097,948,
     440,-412,-1189,-1665,-1820,-1811,-1753,-1733,-1783,-1838,-1806,-1606,-1399,-884,-76,862,1597,1797,1377,641,31,-217,-117,114,212,122,0,-17,75,209,319,514,1013,1824,
     2048,2041,1808,1595,1399,751,-17,-530,-856,-1026,-1071,-1088,-1226,-1593,-1935,-1941,-1802,-1584,-616,830,1959,2048,2048,1925,1709,1107,432,-161,-585,-825,-984,
     -1199,-1529,-1833,-1904,-1460,-700,153,767,929,678,168,-425,-916,-1218,-1334,-1170,-775,-145,491,977,1216,1296,1439,1758,2048,2048,2016,1795,1578,1376,1191,1020,
     863,720,588,86,-630,-1434,-1935,-1935,-1935,-1940,-1760,-1575,-1377,-383,354,850,1076,1009,740,448,239,134,58,-118,-427,-766,-1009,-1053,-870,-548,-199,107,362,593,
     800,939,1013,1090,1276,1631,2048,2048,2048,1835,1603,1312,552,-165,-720,-954,-761,-212,431,860,920,717,483,382,424,334,232,-83,-623,-1191,-1737,-1937,-1935,-1935,
     -1935,-1537,-831,-446,-499,-878,-1307,-1596,-1488,-1293,-1111,-943,-789,-648,-519,-401,-293,-196,-107,-28,41,688,1472,2014,2048,2048,2048,2048,2048,2048,1833,1596,
     1381,1184,1004,839,-151,-573,-498,-147,222,494,397,289,191,102,23,-509,-970,-1221,-1386,-1636,-1927,-1935,-1935,-1935,-1944,-1930,-1671,-1438,-1239,-1056,-730,-525,
     -549,-504,-382,-275,280,910,1386,1559,1423,1066,619,262,148,358,869,1522,2048,2048,2048,2048,2048,1822,1584,1371,1174,994,841,387,-64,-574,-1063,-1372,-1367,-1050,
     -560,-94,180,217,55,-283,-776,-1357,-1859,-1938,-1937,-1941,-1920,-1757,-1708,-1643,-1487,-1236,-943,-639,-292,130,561,843,851,622,361,292,475,807,1145,1366,1403,
     1280,1072,861,748,787,943,1172,1421,1601,1458,1265,843,195,-361,-712,-836,-812,-766,-771,-832,-920,-1001,-1060,-1069,-987,-798,-567,-410,-416,-590,-811,-909,-797,
     -505,-129,195,345,291,88,-169,-354,-382,-256,-39,185,341,391,329,206,109,112,233,423,608,762,906,1040,1118,1088,939,730,564,516,550,571,520,356,94,-207,-515,-818,
     -1067,-1237,-1350,-1426,-1475,-1492,-1430,-1221,-869,-455,-67,207,309,278,197,139,171,290,409,488,538,525,418,199,-138,-502,-751,-791,-627,-349,-37,297,655,996,1249,
     1355,1312,1198,1084,947,728,442,137,-134,-353,-554,-789,-1023,-1156,-1083,-769,-331,29,205,218,145,81,58,29,-40,-144,-283,-469,-694,-902,-998,-907,-650,-345,-122,
     -12,54,145,274,410,518,612,745,928,1087,1119,966,653,280,-51,-325,-580,-806,-912,-803,-493,-137,106,206,225,222,224,212,154,60,-44,-152,-268,-404,-561,-700,-779,
     -791,-764,-737,-706,-620,-449,-205,62,318,580,871,1141,1332,1415,1362,1157,824,431,87,-111,-149,-128,-147,-238,-363,-443,-401,-216,79,423,739,945,993,885,644,316,
     -34,-341,-574,-755,-934,-1145,-1368,-1515,-1485,-1256,-898,-502,-138,169,410,555,566,449,258,66,-98,-279,-533,-857,-1153,-1301,-1248,-1032,-716,-326,155,720,1272,
     1669,1826,1749,1516,1210,858,467,92,-200,-360,-399,-365,-298,-204,-58,176,484,809,1101,1324,1464,1457,1263,1090,944,362,-204,-685,-1055,-1345,-1552,-1654
};

#elif defined(RCV_TEST_WAVEPACK) && RCV_TEST_WAVEPACK == 16

i16 wave1[996] = { 235,281,279,205,65,-99,-227,-273,-225,-110,16,97,106,46,-45,-122,-147,-107,-18,81,143,142,85,15,-20,1,52,78,39,-52,-125,-110,12,174,271,229,64,
    -128,-234,-202,-69,73,143,121,46,-21,-44,-25,7,25,22,5,-11,-16,-10,0,5,-1,-20,-49,-74,-78,-40,48,176,308,389,366,214,-44,-335,-562,-641,-543,-310,
    -38,163,218,122,-63,-234,-301,-218,-10,253,493,657,737,755,726,635,438,91,-405,-971,-1450,-1651,-1431,-769,183,1130,1724,1719,1087,56,-982,-1649,
    -1748,-1337,-658,19,531,866,1083,1206,1172,875,295,-429,-1013,-1172,-784,8,852,1359,1300,723,-77,-722,-946,-722,-251,168,314,156,-157,-422,-522,
    -508,-543,-771,-1198,-1662,-1923,-1806,-1281,-453,553,1686,2982,4468,6027,7338,7931,7372,5446,2270,-1727,-5929,-9686,-12440,-13798,-13563,-11747,
    -8573,-4455,59,4380,7982,10501,11785,11904,11092,9648,7815,5693,3216,231,-3356,-7430,-11577,-15118,-17265,-17338,-14972,-10257,-3748,3627,10764,
    16614,20385,21668,20439,16968,11687,5096,-2248,-9675,-16349,-21298,-23571,-22498,-17957,-10522,-1422,7725,15309,20135,21684,20135,16172,10673,4452,
    -1845,-7677,-12490,-15679,-16722,-15415,-12066,-7493,-2764,1194,3930,5544,6485,7201,7851,8242,8006,6881,4904,2432,-13,-1957,-3117,-3446,-3087,-2310,
    -1439,-792,-611,-984,-1816,-2861,-3813,-4420,-4541,-4142,-3251,-1931,-279,1536,3259,4593,5284,5224,4505,3392,2233,1324,804,614,547,368,-65,-723,-1447,
    -2057,-2471,-2752,-3048,-3470,-3993,-4454,-4639,-4382,-3625,-2389,-733,1259,3424,5447,6873,7229,6238,4003,1039,-1881,-4006,-4865,-4387,-2850,-703,1603,
    3690,5246,5977,5629,4087,1506,-1634,-4631,-6795,-7710,-7379,-6168,-4603,-3118,-1911,-950,-93,780,1708,2672,3649,4634,5600,6433,6910,6748,5732,3842,1315,
    -1409,-3828,-5554,-6423,-6513,-6051,-5284,-4372,-3352,-2178,-798,777,2454,4066,5422,6344,6689,6363,5332,3646,1468,-918,-3138,-4807,-5654,-5608,-4832,
    -3647,-2393,-1301,-430,299,1016,1810,2696,3620,4479,5123,5358,4972,3812,1887,-565,-3101,-5204,-6446,-6617,-5760,-4125,-2065,45,1862,3113,3628,3373,2474,
    1197,-114,-1123,-1590,-1418,-668,463,1677,2641,3059,2755,1729,165,-1620,-3268,-4470,-5028,-4865,-4020,-2642,-981,650,1942,2694,2883,2672,2323,2062,1978,
    2001,1961,1684,1052,28,-1361,-3018,-4752,-6265,-7183,-7162,-6011,-3795,-842,2343,5216,7318,8356,8216,6954,4776,2021,-863,-3371,-5043,-5583,-4960,-3446,
    -1536,215,1387,1827,1667,1200,710,337,47,-272,-691,-1151,-1491,-1539,-1223,-608,154,919,1631,2317,3001,3605,3901,3593,2475,605,-1636,-3654,-4873,-4982,
    -4049,-2447,-654,962,2224,3096,3553,3506,2843,1554,-153,-1861,-3086,-3477,-2966,-1782,-331,979,1871,2255,2181,1769,1149,439,-250,-805,-1132,-1194,-1036,
    -772,-531,-380,-285,-121,245,867,1658,2404,2837,2738,2020,753,-866,-2580,-4143,-5347,-6015,-5983,-5109,-3345,-832,2041,4680,6471,7001,6230,4475,2251,43,
    -1832,-3222,-4030,-4120,-3343,-1667,680,3188,5192,6104,5621,3811,1059,-2081,-5039,-7339,-8644,-8772,-7681,-5475,-2403,1139,4628,7485,9187,9400,8087,5528,
    2243,-1152,-4090,-6155,-7106,-6878,-5557,-3381,-725,1944,4156,5536,5878,5164,3542,1283,-1258,-3675,-5556,-6560,-6496,-5370,-3386,-879,1770,4203,6112,7242,
    7400,6486,4548,1812,-1327,-4374,-6832,-8299,-8536,-7497,-5330,-2355,979,4162,6706,8235,8553,7667,5768,3181,303,-2458,-4727,-6216,-6753,-6313,-5025,-3140,
    -983,1112,2847,3982,4352,3885,2645,875,-1008,-2498,-3149,-2738,-1354,638,2729,4432,5401,5473,4642,3026,854,-1543,-3761,-5400,-6168,-5969,-4925,-3322,-1507,
    210,1611,2580,3078,3117,2740,2024,1082,70,-828,-1416,-1537,-1122,-229,940,2087,2898,3142,2740,1767,404,-1138,-2676,-4057,-5127,-5706,-5605,-4701,-3023,-800,
    1591,3738,5329,6221,6414,5960,4893,3230,1059,-1363,-3601,-5148,-5626,-4960,-3424,-1522,230,1497,2188,2382,2188,1666,847,-164,-1113,-1648,-1475,-552,832,
    2150,2875,2712,1711,201,-1401,-2772,-3752,-4301,-4397,-3976,-2972,-1408,520,2456,4000,4853,4928,4348,3350,2166,942,-253,-1377,-2359,-3099,-3511,-3579,-3367,
    -2978,-2492,-1930,-1266,-489,346,1123,1709,2027,2091,1984,1803,1615,1443,1299,1211,1209,1290,1369,1283,859,10,-1192,-2522,-3677,-4385,-4483,-3940,-2831,-1303,
    440,2147,3549,4405,4579,4077,3041,1689,249,-1077,-2120,-2728,-2780,-2236,-1182,158,1486,2523,3109,3227,2957,2401,1628,680,-382,-1426,-2256,-2679,-2595,-2062,
    -1269,-445,254,802,1268,1715,2100,2267,2034,1334,301,-768,-1561,-1915,-1891,-1714,-1610,-1670,-1823,-1923,-1883,-1741,-1616,-1578,-1555,-1340,-707,416,1860,
    3251,4163,4300,3608,2253,521,-1305,-3005,-4403,-5326,-5581,-5019,-3631,-1618,630,2659,4103,4794,4763,4147,3089,1694,59,-1662,-3227,-4359,-4834,-4572,-3648,
    -2232,-511,1364,3267,5044,6491,7366,7467,6711,5179,3105,808,-1386,-3207,-4475,-5104,-5093,-4505,-3462,-2133,-721,560,1519,2028,2057,1690,1104,523,153,124,459,
    1079,1841,2581,3153,3440,3346,2791,1729,181,-1710,-3660,-5291,-6226,-6220,-5252,-3538,-1456,581,2234,3293,3666,3344,2380,898,-879,-2646,-4070,-4893,-5018,
    -4528,-3625,-2532,-1403,-301,755,1723,2503,2963,3006,2628,1921,1027,76,-851,-1696,-2384,-2798,-2807,-2344,-1465,-362,705,1508,1918,1921,1574,948,107,-864,
    -1826,-2575,-2896,-2650,-1839,-618,761,2048,3069,3756,4117,4185,3985,3529,2838,1968,1013,84,-719,-1327,-1706,-1850,-1776,-1530,-1179,-805,-471,-194,62,364,
    756,1221,1676,2011,2143,2050,1759,1307,707,-34,-872,-1676,-2240,-2363,-1951,-1099,-63,840,1378,1490,1279,919,542,179,-218,-689,-1197,-1616,-1805,-1697,-1357
};

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static List<DSCRSP02> readyRSP02;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static DSCRSP02 *wrDscRSP02 = 0;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//union RequestUnion { ReqRcv01	req01; ReqRcv02	req02; ReqRcv03	req03; ReqRcv04	req04; };
				
//static RspRcv02 rsp02;

static byte rspBuf[64] = "\n" "G26X_1_DSP" "\n" __DATE__ "\n" __TIME__ "\n";

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//static byte build_date[sizeof(RequestUnion)+32] = "\n" "G26X_1_DSP" "\n" __DATE__ "\n" __TIME__ "\n";

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

inline void SaveParams()
{
	cmdSaveParams = true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestFunc01(byte *data, u16 len, ComPort::WriteBuffer *wb)
{
	ReqRcv01::Req &req = *((ReqRcv01::Req*)data);
	RspRcv01 &rsp = *((RspRcv01*)rspBuf);

	byte n = req.n;
	if (n > RCV_FIRE_NUM) n = RCV_FIRE_NUM;

	if (wrDscRSP02 != 0) FreeDscSPORT(wrDscRSP02), wrDscRSP02 = 0;

	DSCRSP02 *dsc = AllocDscSPORT();

	if (dsc != 0)
	{
		dsc->r02.hdr.cnt		= req.fc;
		dsc->fireN				= n;
		dsc->r02.hdr.gain		= ngain[n];
		dsc->next_fireN			= req.next_n;
		dsc->next_gain			= req.next_gain;
		dsc->r02.hdr.st			= MAX(req.st, 2);
		dsc->r02.hdr.sl			= LIM(req.sl, 16, RCV_SAMPLE_LEN);

		dsc->r02.hdr.packType	= req.packType;
		dsc->r02.hdr.math		= req.math;
		
		dsc->r02.hdr.sl = (dsc->r02.hdr.sl+3) & ~3;	

		dsc->sportLen = dsc->r02.hdr.sl;

		if (req.packType >= PACK_DCT0)
		{
			dsc->sportLen += 64;
		};

		u16 delay = req.sd / dsc->r02.hdr.st;

		dsc->r02.hdr.sd	= delay * dsc->r02.hdr.st;

		SyncReadSPORT(dsc, delay);
	};

	fireN = n;
	//sportState = 0;

	if (req.adr == 0) return  false;

	rsp.adr = req.adr;
	rsp.func = req.func;
	rsp.crc = GetCRC16(&rsp, sizeof(rsp)-sizeof(rsp.crc));

	wb->data = &rsp;
	wb->len = sizeof(rsp);

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestFunc02(byte *data, u16 len, ComPort::WriteBuffer *wb)
{
	static u16 buf[2];

	ReqRcv02::Req &req = *((ReqRcv02::Req*)data);

	if (req.n > RCV_FIRE_NUM)
	{
		return false;
	};

	if (req.adr == 0) return  false;

	byte n = req.n;
	
	if (wrDscRSP02 == 0) wrDscRSP02 = readyRSP02.Get();

	if (wrDscRSP02 != 0)
	{
		RspRcv02 &rsp = wrDscRSP02->r02;

		wb->data = &rsp;
		wb->len = wrDscRSP02->len;
	}
	else
	{
		buf[0] = 0xAA30 + (n<<4) + req.adr-1;
		buf[1] = GetCRC16_CCIT_refl(buf, 2);

		wb->data = buf;
		wb->len = sizeof(buf);
	};

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestFunc03(byte *data, u16 len, ComPort::WriteBuffer *wb)
{
	ReqRcv03::Req &req	= *((ReqRcv03::Req*)data);
	RspRcv03 &rsp		= *((RspRcv03*)rspBuf);

	for (byte i = 0; i < ArraySize(ngain); i++)	ngain[i] = req.gain[i];

	if (req.numDevValid) numDevice = req.numDev, numDevValid = req.numDevValid;

	if (req.adr == 0) return  false;

	rsp.adr			= req.adr;
	rsp.func		= req.func;
	rsp.temp		= GetTemp();		// температура
	rsp.numdev		= numDevice;		// номер модуля приёмников
	rsp.verdev		= VERSION; 			// версия ПО модуля приёмников
	rsp.numDevValid	= numDevValid;		// если не ноль, numDev считан из flash правильно или установлен запросом
	rsp.flashStatus	= flashStatus; 		// бит 0 - запись в процессе, бит 1 - запись ОК, бит 2 - ошибка записи

	rsp.crc			= GetCRC16(&rsp, sizeof(rsp)-sizeof(rsp.crc));

	wb->data = &rsp;
	wb->len = sizeof(rsp);

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestFunc04(byte *data, u16 len, ComPort::WriteBuffer *wb)
{
	ReqRcv04::Req	&req = *((ReqRcv04::Req*)data);
	RspRcv04		&rsp = *((RspRcv04*)rspBuf);

	if (req.saveParams) SaveParams();

	if (req.adr == 0) return  false;

	rsp.adr			= req.adr;
	rsp.func		= req.func;
	rsp.temp		= GetTemp();		// температура
	rsp.numdev		= numDevice;		// номер модуля приёмников
	rsp.verdev		= VERSION; 			// версия ПО модуля приёмников
	rsp.numDevValid	= numDevValid;		// если не ноль, numDev считан из flash правильно или установлен запросом
	rsp.flashStatus	= flashStatus; 		// бит 0 - запись в процессе, бит 1 - запись ОК, бит 2 - ошибка записи

	rsp.maxAmp[0] = maxAmp[0];
	rsp.maxAmp[1] = maxAmp[1];
	rsp.maxAmp[2] = maxAmp[2];
	rsp.maxAmp[3] = maxAmp[3];

	rsp.power[0] = power[0];
	rsp.power[1] = power[1];
	rsp.power[2] = power[2];
	rsp.power[3] = power[3];

	rsp.crc = GetCRC16(&rsp, sizeof(rsp)-sizeof(rsp.crc));

	wb->data = &rsp;
	wb->len = sizeof(rsp);

	return true;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

typedef bool (*REQF)(byte *req, u16 len, ComPort::WriteBuffer *wb);

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static REQF listReq[4] = { RequestFunc01, RequestFunc02, RequestFunc03, RequestFunc04 };

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestFunc(const ComPort::ReadBuffer *rb, ComPort::WriteBuffer *wb)
{
	//static NewRequest nulReq;
	static const byte fl[4] = { sizeof(ReqRcv01::r[0])-1, sizeof(ReqRcv02::r[0])-1, sizeof(ReqRcv03::r[0])-1, sizeof(ReqRcv04::r[0])-1 };

	if (rb == 0 || rb->len < 4) return false;

	bool result = false;

	u16 rlen = rb->len;

	byte *p = (byte*)rb->data; 

	while(rlen > 3)
	{
		byte len = p[0];
		byte adr = p[1];
		byte func = p[2]-1;

		if (func < 4 && len == fl[func] && len < rlen && GetCRC16(p+1, len) == 0)
		{
			if (adr != 0 && adr != GetNetAdr()) return false;

			result = listReq[func](p, len+1, wb);

			break;
		}
		else
		{
			p += 1;
			rlen -= 1;
		};
	};

	return result;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void Reboot()
{
	if (!flashChecked)
	{
		CheckFlash();

		if (flashOK && flashCRCOK) bfrom_SysControl(SYSCTRL_SOFTRESET, NULL, NULL);
	};
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestBoot_00(ReqAT25 *r, ComPort::WriteBuffer *wb)
{
	const BootReqV1 &req = *((BootReqV1*)(r->GetDataPtr()));
	static BootRspV1 rsp;

	if (req.F0.adr == 0 || r->len < sizeof(req.F0)) return  false;

	rsp.F0.adr		= req.F0.adr;
	rsp.F0.rw		= req.F0.rw;
	rsp.F0.ver		= rsp.VERSION;
	rsp.F0.maxFunc	= rsp.FUNC_MAX;
	rsp.F0.guid		= RCV_BOOT_SGUID;
	rsp.F0.startAdr = FLASH_START_ADR;
	rsp.F0.pageLen	= FLASH_PAGE_SIZE;

	rsp.F0.crc = GetCRC16(&rsp.F0, sizeof(rsp.F0)-sizeof(rsp.F0.crc));

	wb->data = &rsp.F0;
	wb->len = sizeof(rsp.F0);

	FreeReqAT25(r);

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestBoot_01(ReqAT25 *r, ComPort::WriteBuffer *wb)
{
	const BootReqV1 &req = *((BootReqV1*)(r->GetDataPtr()));
	static BootRspV1 rsp;

	if (req.F1.adr == 0 || r->len < sizeof(req.F1)) return  false;

	rsp.F1.adr		= req.F1.adr;
	rsp.F1.rw		= req.F1.rw;
	rsp.F1.flashLen	= flashLen;
	rsp.F1.flashCRC = flashCRC;

	rsp.F1.crc = GetCRC16(&rsp.F1, sizeof(rsp.F1)-sizeof(rsp.F1.crc));

	wb->data = &rsp.F1;
	wb->len = sizeof(rsp.F1);

	FreeReqAT25(r);

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestBoot_02(ReqAT25 *r, ComPort::WriteBuffer *wb)
{
	const BootReqV1 &req = *((BootReqV1*)(r->GetDataPtr()));
	static BootRspV1 rsp;

	u16 xl = req.F2.plen + sizeof(req.F2) - sizeof(req.F2.pdata);

	if (r->len < xl) return  false;

	u16 adr = req.F2.adr;

	if (req.F2.padr >= curWriteReqAdr)
	{
		curWriteReqAdr = req.F2.padr + req.F2.plen;
	
		r->dataOffset = (byte*)req.F2.pdata - r->data;
		r->stAdr = req.F2.padr;
		r->len = req.F2.plen;

		FlashWriteReq(r);
	}
	else
	{
		FreeReqAT25(r);
	};

	if (adr == 0) return false;

	rsp.F2.adr	= req.F2.adr;
	rsp.F2.rw	= req.F2.rw;
	rsp.F2.res	= GetLastError();

	rsp.F2.crc = GetCRC16(&rsp.F2, sizeof(rsp.F2)-sizeof(rsp.F2.crc));

	wb->data = &rsp.F2;
	wb->len = sizeof(rsp.F2);

	flashChecked = flashOK = flashCRCOK = false;

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestBoot_03(ReqAT25 *r, ComPort::WriteBuffer *wb)
{
	const BootReqV1 &req = *((BootReqV1*)(r->GetDataPtr()));
	static BootRspV1 rsp;

	if (r->len < sizeof(req.F3)) return  false;

	cmdReboot = true;

	if (req.F3.adr == 0) return  false;

	rsp.F3.adr		= req.F3.adr;
	rsp.F3.rw		= req.F3.rw;

	rsp.F3.crc = GetCRC16(&rsp.F3, sizeof(rsp.F3)-sizeof(rsp.F3.crc));

	wb->data = &rsp.F3;
	wb->len = sizeof(rsp.F3);

	FreeReqAT25(r);

	return true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static bool RequestBoot(ReqAT25 *r, ComPort::WriteBuffer *wb)
{
	bool result = false;

	BootReqV1 &req = *((BootReqV1*)(r->GetDataPtr()));

	u16 t = req.F0.rw;
	u16 adr = GetNetAdr();

	bool cm = (t & bootReqWord) == bootReqWord;
	bool ca = req.F0.adr == adr || req.F0.adr == 0;

	if (!ca || !cm || r->len < 2)
	{
		FreeReqAT25(r);
		return false;
	};

	t &= 0xFF;

	switch (t)
	{
		case 0: 	result = RequestBoot_00(r, wb); break;
		case 1: 	result = RequestBoot_01(r, wb); break;
		case 2: 	result = RequestBoot_02(r, wb); break;
		case 3: 	result = RequestBoot_03(r, wb); break;

		default:	FreeReqAT25(r); break;
	};

	return result;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateBlackFin()
{
	static byte i = 0;
	static ComPort::WriteBuffer wb;
	static ComPort::ReadBuffer rb;

	static ReqAT25 *req = 0;

	switch(i)
	{
		case 0:

			req = AllocReqAT25();

			if (req != 0)
			{
				req->len = 0;
				req->dataOffset = 0;
				rb.data = req->GetDataPtr();
				rb.maxLen = req->MaxLen();
				com.Read(&rb, ~0, US2COM(45+62500000/RCV_COM_BAUDRATE));
				i++;
			};

			break;

		case 1:

			if (!com.Update())
			{
				if (rb.recieved && rb.len > 0)
				{
					req->len = rb.len;
					
					if (GetCRC16(rb.data, rb.len) == 0 && RequestBoot(req, &wb))
					{
						com.Write(&wb);
						i++;
					}
					else 
					{
						if (RequestFunc(&rb, &wb))
						{
							com.Write(&wb);
							i++;
						}
						else
						{
							i = 0;
						};

						FreeReqAT25(req);
					};
				}
				else
				{
					FreeReqAT25(req);
					i = 0;
				};
			};

			break;

		case 2:

			if (!com.Update())
			{
				if (cmdReboot) Reboot(), cmdReboot = false;

				i = 0;
			};

			break;
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateSport()
{
	static byte sportState = 0;
	//static byte n = 0;
	//static byte chnl = 0;
	//static u16 len = 0;
	//static byte st = 0;
	//static byte sg = 0;
	//static u16 sd = 0;

	static DSCRSP02 *dsc = 0;
	//static DSCRSP02 *prsp = 0;

	//RspRcv02 &rsp = rsp02;

	switch(sportState)
	{
		case 0:
			
			dsc = GetDscSPORT();

			if (dsc != 0)
			{
				SetGain(dsc->next_gain);

				#ifdef RCV_TEST_WAVEPACK

					dsc->sl = ArraySize(wave1);

					for (u16 i = 0; i < dsc->sl; i++)
					{
						dsc->spd[0][i*2+0] = wave1[i] + 0x8000;
						dsc->spd[0][i*2+1] = wave1[i] + 0x8000;
						dsc->spd[1][i*2+0] = wave1[i] + 0x8000;
						dsc->spd[1][i*2+1] = wave1[i] + 0x8000;
					};

				#endif

				sportState++;
			};

			break;

		case 1:
		{
			RspRcv02 &rsp = dsc->r02;

			rsp.hdr.rw			= 0xAA30 + (dsc->fireN<<4) + GetNetAdr()-1;

			rsp.hdr.packLen1	= dsc->sportLen;

			if (rsp.hdr.math == 0)
			{
				rsp.hdr.packLen2	= dsc->sportLen;
				rsp.hdr.packLen3	= dsc->sportLen;
				rsp.hdr.packLen4	= dsc->sportLen;
			}
			else if (rsp.hdr.math == 1) // Среднее
			{
				rsp.hdr.packLen2	= 0;
				rsp.hdr.packLen3	= 0;
				rsp.hdr.packLen4	= 0;
			}
			else // Разница
			{
				rsp.hdr.packLen2	= dsc->sportLen;
				rsp.hdr.packLen3	= 0;
				rsp.hdr.packLen4	= 0;
			};

			u16 *p1 = rsp.data + dsc->sportLen*0;
			u16 *p2 = rsp.data + dsc->sportLen*1;
			u16 *p3 = rsp.data + dsc->sportLen*2;
			u16 *p4 = rsp.data + dsc->sportLen*3;

			u16 max[4] = {0, 0, 0, 0};
			u16 min[4] = {65535, 65535, 65535, 65535 };

			u32 pow[4] = {0, 0, 0, 0};

			u16 t[4];

			//i16 x;

			for (u16 i = 0; i < dsc->sportLen; i++)
			{
				t[0] = *p1;
				t[1] = *p2;
				t[2] = *p3;
				t[3] = *p4;

				for (byte n = 0; n < 4; n++)
				{
					max[n] = __builtin_max(t[n], max[n]);
					min[n] = __builtin_min(t[n], min[n]);
					pow[n] += __builtin_abs((i16)(t[n] - 0x8000));
				};

				if (rsp.hdr.math == 0)
				{
					*p1 = t[0] - 0x8000;
					*p2 = t[1] - 0x8000;
					*p3 = t[2] - 0x8000;
					*p4 = t[3] - 0x8000;
				}
				else if (rsp.hdr.math == 1) // Среднее
				{
					*p1 = (t[0]+t[1]+t[2]+t[3])/4 - 0x8000;
				}
				else // Разница
				{
					*p1 = subsat16(t[2], t[0]); //(t[2]-t[0])/2;
					*p2 = subsat16(t[3], t[1]); //2;
				};

				p1++;
				p2++;
				p3++;
				p4++;
			};

			for (byte i = 0; i < 4; i++)
			{
				maxAmp[i] = max[i] - min[i];
				power[i] = (rsp.hdr.sl > 0) ? (pow[i] / rsp.hdr.sl) : 0;
			};
		
			//FreeDscSPORT(dsc);

			//dsc = 0;

			dsc->len = sizeof(rsp.hdr) + (rsp.hdr.packLen1+rsp.hdr.packLen2+rsp.hdr.packLen3+rsp.hdr.packLen4)*2;

			sportState = (rsp.hdr.packType == PACK_NO) ? 3 : (sportState+1);

			break;

		};

		case 2:
		{
			HW::PIOF->BSET(7);

			RspRcv02 &rsp = dsc->r02;

			if (rsp.hdr.packType == PACK_ULAW12 || rsp.hdr.packType == PACK_ULAW16)
			{
				u16 plen = rsp.hdr.packLen1+rsp.hdr.packLen2+rsp.hdr.packLen3+rsp.hdr.packLen4;

				if (rsp.hdr.packType == PACK_ULAW12)
				{
					WavePack_uLaw_12Bit((i16*)rsp.data, (byte*)rsp.data, plen);
				}
				else
				{
					WavePack_uLaw_16Bit((i16*)rsp.data, (byte*)rsp.data, plen);
				};

				rsp.hdr.packLen1 /= 2;
				rsp.hdr.packLen2 /= 2;
				rsp.hdr.packLen3 /= 2;
				rsp.hdr.packLen4 /= 2;

				dsc->len = sizeof(rsp.hdr) + plen;
			}
			else if (rsp.hdr.packType == PACK_ADPCMIMA)
			{
				i16	*src	= (i16*)rsp.data;
				byte *dst	= (byte*)rsp.data;

				WavePack_ADPCMIMA(src, dst, rsp.hdr.packLen1); src += rsp.hdr.packLen1; dst += rsp.hdr.packLen1/2;
				WavePack_ADPCMIMA(src, dst, rsp.hdr.packLen2); src += rsp.hdr.packLen2; dst += rsp.hdr.packLen2/2;
				WavePack_ADPCMIMA(src, dst, rsp.hdr.packLen3); src += rsp.hdr.packLen3; dst += rsp.hdr.packLen3/2;
				WavePack_ADPCMIMA(src, dst, rsp.hdr.packLen4); 

				rsp.hdr.packLen1 /= 4;
				rsp.hdr.packLen2 /= 4;
				rsp.hdr.packLen3 /= 4;
				rsp.hdr.packLen4 /= 4;

				dsc->len = sizeof(rsp.hdr) + (rsp.hdr.packLen1+rsp.hdr.packLen2+rsp.hdr.packLen3+rsp.hdr.packLen4)*2;
			}
			else if (rsp.hdr.packType >= PACK_DCT0)
			{
				i16	*src	= (i16*)rsp.data;
				byte *dst	= (byte*)rsp.data;

				rsp.hdr.packLen1 = WavePack_FDCT(rsp.hdr.packType, src, dst, rsp.hdr.packLen1); src += dsc->sportLen; dst += rsp.hdr.packLen1;
				rsp.hdr.packLen2 = WavePack_FDCT(rsp.hdr.packType, src, dst, rsp.hdr.packLen2); src += dsc->sportLen; dst += rsp.hdr.packLen2; 
				rsp.hdr.packLen3 = WavePack_FDCT(rsp.hdr.packType, src, dst, rsp.hdr.packLen3); src += dsc->sportLen; dst += rsp.hdr.packLen3; 
				rsp.hdr.packLen4 = WavePack_FDCT(rsp.hdr.packType, src, dst, rsp.hdr.packLen4); 

				dsc->len = sizeof(rsp.hdr) + rsp.hdr.packLen1 + rsp.hdr.packLen2 + rsp.hdr.packLen3 + rsp.hdr.packLen4;

				rsp.hdr.packLen1 /= 2;
				rsp.hdr.packLen2 /= 2;
				rsp.hdr.packLen3 /= 2;
				rsp.hdr.packLen4 /= 2;
			};

			HW::PIOF->BCLR(7);

			sportState += 1;

			break;
		};

		case 3:
		{
			RspRcv02 &rsp = dsc->r02;

			dsc->data[dsc->len/2] = GetCRC16_CCIT_refl(&dsc->r02, dsc->len);
			dsc->len += 2;

			readyRSP02.Add(dsc); dsc = 0;

			sportState = 0;

			break;
		};
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void UpdateSaveParams()
{
	static byte i = 0;

	static ReqAT25 *req = 0;

	static byte count = 0;
	static u32 adr = 0;

	switch (i)
	{
		case 0:

			if (numDevValid == 0)
			{
				cmdSaveParams = false;
			};

			if (cmdSaveParams)
			{
				adr = 0x30000-FLASH_START_ADR;
				count = 4;
				i++;
			};

			break;

		case 1:

			req = AllocReqAT25();

			if (req != 0)
			{
				req->dataOffset = 0;
				req->stAdr = adr;
				req->len = 4;

				u16 *p = (u16*)req->data;

				p[0] = numDevice;
				p[1] = GetCRC16(p, 2);

				FlashWriteReq(req);

				adr += FLASH_SECTOR_SIZE;

				if (count > 0)
				{
					count -= 1;
				}
				else
				{
					cmdSaveParams = false;
					i = 0;
				};
			};

			break;
	};

	FlashUpdate();
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void LoadParams()
{
	u32 adr = 0x30000;

	static u16 aaa[2];

	numDevice = 0;
	numDevValid = 0;

	while (FlashBusy()) FlashUpdate();

	for (byte i = 0; i < 4; i++)
	{
		at25df021_Read(aaa, adr, sizeof(aaa));

		if (GetCRC16(aaa, sizeof(aaa)) == 0)
		{
			numDevice = aaa[0];
			numDevValid = true;
			break;
		};

		adr += FLASH_SECTOR_SIZE;
	};
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static void CheckFlash()
{
	static ADI_BOOT_HEADER bh;
	static u16 crc = 0;

	while (FlashBusy()) FlashUpdate();

	u32 *p = (u32*)&bh;

	u32 adr = 0;
	
	flashOK = flashChecked = flashCRCOK = false;

	//at25df021_Read(buf, 0, sizeof(buf));

	while (1)
	{
		at25df021_Read(&bh, FLASH_START_ADR + adr, sizeof(bh));

		u32 x = p[0] ^ p[1] ^ p[2] ^ p[3];
		x ^= x >> 16; 
		x = (x ^ (x >> 8)) & 0xFF; 

		if (((u32)(bh.dBlockCode) >> 24) == 0xAD && x == 0)
		{
			adr += sizeof(bh);

			if ((bh.dBlockCode & BFLAG_FILL) == 0)
			{
				adr += bh.dByteCount;	
			};

			if (bh.dBlockCode & BFLAG_FINAL)
			{
				flashOK = true;

				break;
			};
		}
		else
		{
			break;
		};
	};

	flashLen = adr;

	at25df021_Read(&crc, FLASH_START_ADR + adr, sizeof(crc));

	if (flashLen > 0) flashCRC = at25df021_GetCRC16(FLASH_START_ADR, flashLen), flashCRCOK = (flashCRC == crc);

	if (!flashCRCOK) flashLen = 0;

	flashChecked = true;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

void main( void )
{
//	static byte s = 0;

//	static u32 pt = 0;

	InitHardware();

	FlashInit();

	CheckFlash();

	WavePack_Init();

	com.Connect(RCV_COM_BAUDRATE, RCV_COM_PARITY);

	LoadParams();

//	InitNetAdr();

	while (1)
	{
		MAIN_LOOP_PIN_SET();

		static byte i = 0;

		#define CALL(p) case (__LINE__-S): p; break;

		enum C { S = (__LINE__+3) };
		switch(i++)
		{
			CALL( UpdateBlackFin()		);
			CALL( UpdateHardware()		);
			CALL( UpdateSport()			);
			CALL( UpdateSaveParams()	);
		};

		i = (i > (__LINE__-S-3)) ? 0 : i;

		#undef CALL

		MAIN_LOOP_PIN_CLR();
	};

//	return 0;
}
