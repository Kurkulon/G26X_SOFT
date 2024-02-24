#include "G26X_3_HW_CONF.H"
#include "G_TRM.H"

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#define BOOT_COM
//#define BOOT_EMAC
//#define BOOT_HANDSHAKE
//#define BOOT_COM_MODE					ComPort::ASYNC
#define BOOT_COM_SPEED				TRM_COM_BAUDRATE
#define BOOT_COM_PARITY				TRM_COM_PARITY
#define BOOT_COM_STOPBITS			TRM_COM_STOPBITS
#define BOOT_COM_PRETIMEOUT			(~0)
#define BOOT_COM_POSTTIMEOUT		(US2COM(400))
#define BOOT_HANDSHAKE_PRETIMEOUT	(MS2COM(100))
#define BOOT_HANDSHAKE_POSTTIMEOUT	(US2COM(400))
#define BOOT_HANDSHAKE_TIMEOUT		(2000)
#define BOOT_SGUID					TRM_BOOT_SGUID
#define BOOT_MGUID					TRM_BOOT_MGUID
//#define BOOT_START_SECTOR				8
//#define BOOT_START_BREAKPOINT
//#define BOOT_EXIT_BREAKPOINT

#define BOOT_MAN_REQ_WORD			TRM_BOOT_REQ_WORD
#define BOOT_MAN_REQ_MASK 			TRM_BOOT_REQ_MASK

#define BOOT_MAX_NETADR				TRM_BOOT_NET_ADR
#define BOOT_TIMEOUT				(2000)
#define BOOT_MAIN_TIMEOUT			(10000)
#define BOOT_COM_ERROR_TIMEOUT		(2000)

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

inline u16 GetNetAdr() { return TRM_BOOT_NET_ADR; }

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef BOOT_COM

//#include <DMA\DMA.h>
#include <ComPort\ComPort_imp.h>

static ComPort com(0, 0, PIO_UTXD0, PIO_URXD0, PIO_RTS0, 0, PIN_UTXD0, PIN_URXD0, PIN_RTS0, 0, PMUX_UTXD0, PMUX_URXD0, UART0_TXPO, UART0_RXPO, UART0_GEN_SRC, GEN_MCK_CLK, &UART0_DMA);

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#ifdef BOOT_EMAC

#include <emac_imp_v2.h>
#include <tftp_imp.h>

#endif

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <BOOT\boot_com_emac_imp_v2.h>

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++



//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
