/*
 * Driver for OMAP-UART controller.
 * Based on drivers/serial/8250.c
 *
 * Copyright (C) 2010 Texas Instruments.
 *
 * Authors:
 *	Govindraj R	<govindraj.raja@ti.com>
 *	Thara Gopinath	<thara@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Note: This driver is made separate from 8250 driver as we cannot
 * over load 8250 driver with omap platform specific configuration for
 * features .like DMA, it makes easier to implement features like DMA and
 * hardware flow control and software flow control configuration with
 * this driver as required for the omap-platform.
 */

#define EXOR_MPI
#define USE_WAIT_TX_EMPTY

#define RETRY_IN_SAME_TOKEN
//#define	MANAGE_RR_ANSWERS

#ifdef EXOR_MPI
#include "linux/types.h"
#include <linux/hrtimer.h>
#include "../../../include/linux/time.h"
#include "../../../arch/arm/include/asm/ftrace.h"


#define UART_OMAP_TLR		0x07	/* FIFO trigger level register */
#define UART_RXFIFO_LVL		0x19	/* number of chars in RX fifo */
#define OMAP_TX_FIFO_LVL	0x1A	/* number of chars in TX fifo */

typedef unsigned char byte;
typedef unsigned short word;
typedef unsigned int dword;

#define MPI_MASTER_STACK 1
#define MPI_SLAVE_STACK 0
#define BUF_LEN  512

#define SET_MPI_MODE 0x54EF
#define GET_MPI_DIAG 0x54EE
#define SET_MPI_DIAG 0x54ED
#define MPI_OPEN	 0x54EC
#define MPI_RUN		 0x54EB
#define MPI_IS_OPEN	 0x54EA
#define MPI_CLOSE	 0x54E9
#define SET_MPI_DATA 0x54E8
#define SET_MPI_REQ	 0x54E7
#define GET_MPI_RQST 0x54E6
#define GET_MPI_RESP 0X54E5

struct s_MPIparams
{
	const char *uart;
	unsigned char panelNode;
	unsigned char maxNode;
	unsigned short applTimeout;
	unsigned short guardTimeoutConstant;
	unsigned short guardTimeoutFactor;
	unsigned short ackTimeout;
	unsigned short fdlTimeout;
	unsigned short tokTimeout;
	unsigned short selfTokTimeout;
};

/* Frame Control Functions                                              */
#define  D_SD1          0x10           /* Start delimeter FDL STATUS    */
#define  D_SD2          0x68           /* Start delimeter SRD Request Data Transfer */
#define  D_SD3          0xA2           /* Start delimeter               */
#define  D_ED           0x16           /* End delimeter                 */
#define  D_SC           0xE5           /* Short acknowledge             */
#define  D_TOK          0xDC           /* Token frame     (SD4)         */

/* Frame Control Functions                                              */
#define FCFC_SDAL            0x03      /* Send Data with Acknowledge Low*/
#define FCFC_SRDL            0x0C      /* Send and Request Data Low     */
#define FCFC_RFDL            0x09      /* Request FDL status            */

/* Frame Control byte masks                                             */
#define FCMSK_FCB            0x20      /* FCB mask                      */
#define FCMSK_FCV            0x10      /* FCV mask                      */
#define FCMSK_ST             0x30      /* Station Type mask             */
#define FCMSK_FT             0x40      /* Frame Type mask               */

#define FCST_SLAVE           0x00      /* Slave station                 */
#define FCST_MSNR            0x10      /* Master not ready for ring     */
#define FCST_MSRD            0x20      /* Master ready for ring         */
#define FCST_MSIR            0x30      /* Master in for ring            */

/* Reply Frame Control bytes (FC)                                       */
#define RFCMSK_CT            0x0F      /* Mask for frame control resp   */
#define RFC_OK               0x00      /* Acknowledge positive          */
#define RFC_DL               0x08      /* Response FDL/FMA1/2 Data low  */
#define RFC_DH               0x0A      /* Response FDL/FMA1/2 Data high */
#define RFC_TTNAK            0x02      /* NAK no resource */
#define RFC_RS_NAK           0x03      /* NAK no service activated */

#define NR_MAX_STATIONS 127

// master line states
#define NONE        0
#define SKIP        1

/* Config Par ----------------------------*/
#define N_MACCHINE   8
#define N_EVPOST	 256

#define FSM_TOK      0
#define FSM_LOG      1
#define FSM_JB0      2
#define FSM_JOB      3
#define FSM_LGS      4
#define FSM_J0S      5
#define FSM_JBS      6
#define FSM_LGF      7

/* Log Activation Flags ------------------*/
#define EVENTLOG	       0
#if EVENTLOG
   #define SUBSET_EVENTLOG  0
#endif
#define FUNCTLOG         0
#define SHORT_FORM       1

/* Events --------------------------------*/
#define EV__NULL 0
#define EV_BRC_ENABLE 1
#define EV_BRC_ACK 2
#define EV_BRC_TOK 3
#define EV_BRC_SD1 4
#define EV_BRC_SD2 5
#define EV_BRC_SDX 6
#define EV_BRC_EOTX 7
#define EV_BRC_APPL_TIMEOUT 8
#define EV_BRC_ACK_TIMEOUT 9
#define EV_BRC_SL_SESS_TIMEOUT 10
#define EV_BRC_STOP_SESS 11
#define EV_BRC_J0S_SESS_TIMEOUT 12
#define EV_BRC_JBS_SESS_TIMEOUT 13
#define EV_SES_ACK 14
#define EV_SES_SD2 15
#define EV_SES_ERR 16
#define EV_SES_SD1 17
#define EV_SES_RETRY 18
#define EV_LOG_RUN 19
#define EV_LOG_START 20
#define EV_LOG_OFF 21
#define EV_TOK_RUN 22
#define EV_TOK_ACTIVEIDLE 23
#define EV_TOK_FDLSTATUS 24
#define EV_TOK_WAITSESSRX 25
#define EV_TOK_FDLTIMEOUT 26
#define EV_TOK_TIMEOUT 27
#define EV_TOK_TXFRAME 28
#define EV_TOK_PASSTOKEN 29
#define EV_TOK_SELFTOKEN 30
#define EV_TOK_TIMEOUT_TOK 31
#define EV_TOK_SELFTOK_TIMEOUT 32
#define EV_TOK_NO_RING_ACTIVITY 33
#define EV_JB0_RUN 34
#define EV_JB0_SEND_REQ 35
#define EV_JOB_RUN 36
#define EV_JOB_SEND_REQ 37
#define EV_LGS_RUN 38
#define EV_LGS_SEND_SAPRSP 39
#define EV_LGS_SEND_ACKSAPRSPACK 40
#define EV_J0S_RUN 41
#define EV_J0S_SEND_JOBACK 42
#define EV_J0S_SEND_JOBRSP 43
#define EV_JBS_RUN 44
#define EV_JBS_SEND_JOBACK 45
#define EV_JBS_SEND_JOBRSP 46
#define EV_LGF_RUN 47
#define EV_LGF_SEND_LOGOFFACK 48
#define EV_TX_DELAY_DONE 49
#define N_EVENTI 50

/* States --------------------------------*/
#define _NULL 0
#define TOK_IDLE 1
#define TOK_LISTENTOKEN 2
#define TOK_ACTIVEIDLE 3
#define TOK_WAITRX 4
#define TOK_TOKEN_RETRY 5
#define TOK_WAITFDLSTATUS 6
#define TOK_WAITFDLSTATUS2 7
#define TOK_WAITSESSRX 8
#define TOK_SELFTOKEN 9
#define LOG_IDLE 10
#define LOG_RUN 11
#define LOG_WAITSAPREQACK 12
#define LOG_WAITSAPRSP 13
#define LOG_WAITSAPRSPACK 14
#define LOG_WAITACKSAPRSPACK 15
#define JB0_IDLE 16
#define JB0_RUN 17
#define JB0_WAITSHORTACK0 18
#define JB0_WAITJOBACK0 19
#define JB0_WAITJOBRESP0 20
#define JB0_WAITSHORTACK02 21
#define JOB_IDLE 22
#define JOB_RUN 23
#define JOB_WAITSHORTACK 24
#define JOB_WAITJOBACK 25
#define JOB_WAITJOBRESP 26
#define JOB_WAITSHORTACK2 27
#define LGS_IDLE 28
#define LGS_RUN 29
#define LGS_SEND_SAPRSP 30
#define LGS_ACKSAPRSP 31
#define LGS_SEND_ACKSAPRSPACK 32
#define J0S_IDLE 33
#define J0S_RUN 34
#define J0S_SEND_JOBACK 35
#define J0S_SEND_JOBRSP 36
#define J0S_WAIT_ACK_JOBRSP 37
#define JBS_IDLE 38
#define JBS_RUN 39
#define JBS_SEND_JOBACK 40
#define JBS_SEND_JOBRSP 41
#define JBS_WAIT_ACK_JOBRSP 42
#define LGF_IDLE 43
#define LGF_RUN 44
#define LGF_SENT_LOGOFF_ACK 45
#define N_STATI 46

#define WR 0
#define RD 1
#define NR_JOB_RETRY 3

#define JOBACTION_REQ_LEN0 31


typedef enum
{
   NO_ERROR       =  0x00, /* initial value, before comm. starts        */
   NOT_ACCEPTED   =  0x01, /* request refused                           */
   M_PROC_RUNNING,
   M_PROC_OK,
   RESPONSE_NAK,           /* repeated NACKs from slave                 */
   TIMEOUT_ERR,            /* Not getting polled by Master              */
   RESPONSE_ERR,           /* error in response from slave              */
   GEN_COMM_ERR,           /* general communication error               */
   TIMEOUT_ERR2,           /* no response from slave to data request    */
   TIMEOUT_ERR3,           /* Timed out while sending request to PLC.   */
   RESPONSE_NAK2,          /* NAK from PLC.                             */
   LINE_ERROR,             /* Bad baud rate, parity, data bits etc      */
   RESPONSE_ERR2,          /* ill formed response from PLC              */
   RESPONSE_XOFF,          /* Timed out while waiting for XON           */
   RESPONSE_CTS,           /* Timed out while waiting for CTS           */
   COMM_CTRL_BLOCKED,      /* Comm blocked by FrameCommControlBlocked   */
   DRIVER_ERR              /* error in protocol driver                  */
} sendType;


byte const JobActionReq0[JOBACTION_REQ_LEN0] =
   {D_SD2, 0x19, 0x19, D_SD2, 0, 0, 0x7C, 0, 0,  //SD2 LE LER SD2 DA SA FC DAE SAE
	0xF1, 0,                                     //F1 jobnr
	0x32, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,    //APPL FIX PART
	0x08, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x01,    //
	0x00, 0x03, 0x00, 0xF0,                      //
	0, D_ED};                                    //FCS ED

#define JOBACTION2_REQ_LEN0 33
byte const JobAction2Req0[JOBACTION2_REQ_LEN0] =
   {D_SD2, 0x1B, 0x1B, D_SD2, 0, 0, 0x5C, 0, 0,  //SD2 LE LER SD2 DA SA FC DAE SAE
	0xF1, 0,                                     //F1 jobnr
	0x32, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00,    //APPL FIX PART
	0x08, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x00,    //
	0x00, 0x01, 0x00, 0x03, 0x00, 0xF0,          //
	0, D_ED};                                    //FCS ED

#define JOBACTION_REQ_LEN 37
byte const JobActionReq[JOBACTION_REQ_LEN] =
   {D_SD2, 0x1F, 0x1F, D_SD2, 0, 0, 0x7C, 0, 0,  //SD2 LE LER SD2 DA SA FC DAE SAE
	0xF1, 0,                                     //F1 jobnr
	0x32, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00,    //APPL FIX PART
	0x0E, 0x00, 0x00, 0x04, 0x01, 0x12, 0x0a,    //
	0x10, 0x05, 0x00, 0x01, 0x00, 0x00, 0x83,    //
	0x00, 0x00, 0x00,                            //
	0, D_ED};                                    //FCS ED

#define JOBACTION_ACK_LEN 14
byte const JobActionAck[JOBACTION_ACK_LEN] =
   {D_SD2, 0x08, 0x08, D_SD2, 0, 0, 0x7C, 0, 0,  //SD2 LE LER SD2 DA SA FC DAE SAE
	0xB0, 0x01, 0,                               //BO 1 jobnr
	0, D_ED};                                    //FCS ED

#define LOGON_SAPREQ_LEN 23
byte const LogOnSAPReq[LOGON_SAPREQ_LEN] =
   {D_SD2, 0x11, 0x11, D_SD2, 0, 0, 0x6D, 0, 0, //SD2 LE LER SD2 DA SA FC DAE SAE
	0xE0, 4, 0, 0x80, 0, 2, 0, 2, 2, 0, 2, 0,    //FIX PART
	0, D_ED};                                    //FCS ED

#define LOGON_SAPRSP_LEN 23
byte const LogOnSAPRsp[LOGON_SAPRSP_LEN] =
   {D_SD2, 0x11, 0x11, D_SD2, 0, 0, 0x6C, 0, 0, //SD2 LE LER SD2 DA SA FC DAE SAE
	0xD0, 4, 0, 0x80, 0, 2, 0, 2, 1, 0, 1, 0,    //FIX PART
//    0xD0, 4, 0, 0x80, 0, 2, 0, 2, 2, 0, 2, 0,    //FIX PART
	0, D_ED};                                    //FCS ED

#define LOGOFF_ACK 12
byte const LogOffAck[LOGOFF_ACK] =
   {D_SD2, 0x06, 0x06, D_SD2, 0, 0, 0x6D, 0, 0, //SD2 LE LER SD2 DA SA FC DAE SAE
	0xC0,                                       //FIX PART
	0, D_ED};                                   //FCS ED

#define LOGON_SAPRESPACK_LEN 13
byte const LogOnSAPRespAck[LOGON_SAPRESPACK_LEN] =
   {D_SD2, 0x07, 0x07, D_SD2, 0, 0, 0x5C, 0, 0, //SD2 LE LER SD2 DA SA FC DAE SAE
	0x05, 0x01,                                  //FIX PART
	0, D_ED};                                    //FCS ED

typedef struct
{
   byte  IsActive;         // 0 - station not present
						   // 1 - station present
   byte  StationType;      // 0 - slave station
						   // 1 - Master not ready to enter token ring
						   // 2 - Master ready to enter token ring (Ready FC=0x20)
						   // 3 - Master in token ring (Active FC=0x30)
   byte  FCV;              // 0 - alternating function of FCB is invalid
						   // 1 - alternating function of FCB is valid
   byte  FCB;              // Alternates between 0 and 1 for each new
						   // action frame
   byte  Logged;           // 0/1 LoggedOn/LoggedOff
   byte  LogStatus;        // status of logging machine	//MG001
   byte  Job;              // Job counter
   byte  LogOn_SAE;        // LogOnSAE
   byte  LogOn_DAE;        // LogOnDAE
} sStationStatus;

#define MAX_LENGTH_SEND 270                   /* max frame data length  */
#define MAX_LENGTH_RECV 270                   /* max frame data length  */

#define PASS_TOKEN_REPLY 4
#define FDLSTATUS        0
#define PASSTOKEN        1

struct s_MPIdata
{

	struct uart_omap_port *mpiUp;
	bool m_isOpen;
	struct hrtimer hrt;
	byte UltiUart1_TxTimeout;
	int UltiUart1_TxNunUSec;


	int generateEOTEvent;
	int shortACK_EOTEvent;

	// from action.c
	byte NumTokenRotations;          /* num token rotations          */
	byte TokenNotForMe;
	byte LastStation;

	byte SessReqPending;
	byte SessionStarted;
	byte JobRetry;
	byte LogSessReqPending;
	byte LgfSessReqPending;
	byte Job0SessReqPending;
	byte JobSessReqPending;

	byte LgsSessReqPending;
	byte J0sSessReqPending;
	byte JbsSessReqPending;

	byte MyOperFlag;
	byte WrMyBuff[256];
	byte FrameMyBuff[256];
	word WrMyBuffLength;
	byte FrameMyBuffLength;
	word MyLen;

	unsigned char countArray[7];
	unsigned char interruptsCount[3];

	unsigned char ev_queue[N_EVPOST];
	unsigned char ev_queue_rd;
	unsigned char ev_queue_wr;
	unsigned char queue_empty;
	unsigned char event;
	unsigned char tok_state;
	unsigned char lgf_state;

	#if MPI_MASTER_STACK
	byte LogOn_Retry;
	unsigned char log_state;
	unsigned char jb0_state;
	unsigned char job_state;
	#endif
	#if MPI_SLAVE_STACK
	unsigned char lgs_state;
	unsigned char j0s_state;
	unsigned char jbs_state;
	byte JbsOnSlave_Job;
	#endif

	signed char  FrameUniopNodeNum;       /* Node of UniOP in network      */
	byte  FrameSendBufferLen1;
	byte  FrameSendBufferLen;
	byte  FrameSendBuffer[MAX_LENGTH_SEND + 10];       /* TX buffer */
	byte  FrameSendBuffer1[MAX_LENGTH_SEND + 10];       /* TX buffer */

	word  FrameSessionSendLength;
	byte  FrameSessionSendBuffer[MAX_LENGTH_SEND + 10];     //MG001
	word  FrameSessionReceiveLength;
	byte  FrameSessionReceiveBuffer[MAX_LENGTH_RECV + 10];  //MG001

	byte ProcedureApplMStatus;                 /* procedure global status      */
	bool ProcedureApplMRequestPending;         /* Waiting to send PLC a request*/
	bool ProcedureApplMResponsePending;        /* Waiting for answer from PLC  */
	sStationStatus StationStatus[NR_MAX_STATIONS]; /* the station statuses    */
	byte LowestAddress;                        /* Station for next GAP         */
	signed char NextGAP;
	bool AreWeInRing;
	signed char NextStation;
	signed char MaxStationAddress;
	word applResponseTime;
	word guardTimeConstant;
	word guardTimeFactor;
	word ackGuardTime;
	word FDLTimeout;
	word tokTimeout;
	word selfTokTimeout;
	byte Source;
	byte Dest;
	byte FlowCtrl;
	byte MyFrameResponseBuff[MAX_LENGTH_RECV + 10];
	word MyFrameResponseBuffLen;
	byte MyFrameRequestBuff[MAX_LENGTH_SEND + 10];
	word FrameJobSessionLen;
	byte FrameJobSessionBuff[MAX_LENGTH_SEND + 10];
	byte ReadyForConfigMode;
	byte SlaveSession;

	byte GapUpdateFactor;
	byte RingActivityCounter;

	byte LogOn_DA;
	byte LogOff_DA;
	byte Last_DA;
	byte LogOn_DAE;
	byte LogOn_SAE;
	byte CurrentShortAck;
	byte Start_LogOn_SAE;

	byte Start_LogOn_DAE;
	byte LogOnSlave_DA;

	byte startStopTrace;
	byte traceStatus;
	byte tracePostStopCnt;

	byte PassTokenReply;
	byte FlagSendSelfToken;
	byte FlagPassToken;
	byte Sd1RespGuard;

	byte GapUpdateFactorCnt;
	signed char NxtStat;
	byte cnt;
	unsigned char m_taskBuf[BUF_LEN];
	int m_taskLen;
	int applTryCnt;
	bool MPIenabled;
	bool MPImode;
	int rxCnt;
	unsigned char mpiRxBuf[1000];
	int txCnt;
	int txIdx;
	unsigned char mpiTxBuf[280];
//	int txfullflag;
};
void ev_move(struct s_MPIdata *pMPIdata, unsigned char ev);

#endif

#if defined(CONFIG_SERIAL_OMAP_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/module.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/serial_reg.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/serial_core.h>
#include <linux/irq.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_data/serial-omap.h>
#include <linux/of_platform.h>

#include <dt-bindings/gpio/gpio.h>
#include <linux/plxx_manager.h>

#define OMAP_MAX_HSUART_PORTS	6

#define UART_BUILD_REVISION(x, y)	(((x) << 8) | (y))

#define OMAP_UART_REV_42 0x0402
#define OMAP_UART_REV_46 0x0406
#define OMAP_UART_REV_52 0x0502
#define OMAP_UART_REV_63 0x0603

#define OMAP_UART_TX_WAKEUP_EN		BIT(7)

/* Feature flags */
#define OMAP_UART_WER_HAS_TX_WAKEUP	BIT(0)

#define UART_ERRATA_i202_MDR1_ACCESS	BIT(0)
#define UART_ERRATA_i291_DMA_FORCEIDLE	BIT(1)

#define DEFAULT_CLK_SPEED 48000000 /* 48Mhz*/

/* SCR register bitmasks */
#define OMAP_UART_SCR_RX_TRIG_GRANU1_MASK		(1 << 7)
#define OMAP_UART_SCR_TX_TRIG_GRANU1_MASK		(1 << 6)
#define OMAP_UART_SCR_TX_EMPTY			(1 << 3)

/* FCR register bitmasks */
#define OMAP_UART_FCR_RX_FIFO_TRIG_MASK			(0x3 << 6)
#define OMAP_UART_FCR_TX_FIFO_TRIG_MASK			(0x3 << 4)

/* MVR register bitmasks */
#define OMAP_UART_MVR_SCHEME_SHIFT	30

#define OMAP_UART_LEGACY_MVR_MAJ_MASK	0xf0
#define OMAP_UART_LEGACY_MVR_MAJ_SHIFT	4
#define OMAP_UART_LEGACY_MVR_MIN_MASK	0x0f

#define OMAP_UART_MVR_MAJ_MASK		0x700
#define OMAP_UART_MVR_MAJ_SHIFT		8
#define OMAP_UART_MVR_MIN_MASK		0x3f

#define OMAP_UART_DMA_CH_FREE	-1

#define MSR_SAVE_FLAGS		UART_MSR_ANY_DELTA
#define OMAP_MODE13X_SPEED	230400

/* WER = 0x7F
 * Enable module level wakeup in WER reg
 */
#define OMAP_UART_WER_MOD_WKUP	0X7F

/* Enable XON/XOFF flow control on output */
#define OMAP_UART_SW_TX		0x08

/* Enable XON/XOFF flow control on input */
#define OMAP_UART_SW_RX		0x02

#define OMAP_UART_SW_CLR	0xF0

#define OMAP_UART_TCR_TRIG	0x0F

struct uart_omap_dma {
	u8			uart_dma_tx;
	u8			uart_dma_rx;
	int			rx_dma_channel;
	int			tx_dma_channel;
	dma_addr_t		rx_buf_dma_phys;
	dma_addr_t		tx_buf_dma_phys;
	unsigned int		uart_base;
	/*
	 * Buffer for rx dma.It is not required for tx because the buffer
	 * comes from port structure.
	 */
	unsigned char		*rx_buf;
	unsigned int		prev_rx_dma_pos;
	int			tx_buf_size;
	int			tx_dma_used;
	int			rx_dma_used;
	spinlock_t		tx_lock;
	spinlock_t		rx_lock;
	/* timer to poll activity on rx dma */
	struct timer_list	rx_timer;
	unsigned int		rx_buf_size;
	unsigned int		rx_poll_rate;
	unsigned int		rx_timeout;
};

struct uart_omap_port {
	struct uart_port	port;
	struct uart_omap_dma	uart_dma;
	struct device		*dev;

	unsigned char		ier;
	unsigned char		lcr;
	unsigned char		mcr;
	unsigned char		fcr;
	unsigned char		efr;
	unsigned char		dll;
	unsigned char		dlh;
	unsigned char		mdr1;
	unsigned char		scr;
	unsigned char		wer;

	int			use_dma;
	/*
	 * Some bits in registers are cleared on a read, so they must
	 * be saved whenever the register is read but the bits will not
	 * be immediately processed.
	 */
	unsigned int		lsr_break_flag;
	unsigned char		msr_saved_flags;
	char			name[20];
	unsigned long		port_activity;
	int			context_loss_cnt;
	u32			errata;
	u8			wakeups_enabled;
	u32			features;

	int			DTR_gpio;
	int			DTR_inverted;
	int			DTR_active;

	struct serial_rs485	rs485;
	int			rts_gpio;
	int 			mode_gpio;    /* If a valid gpio is mapped here, it means we have a programmable RS485/RS232 phy */
	int                     rxen_gpio;    /* If a valid gpio is mapped here, we will use it for disabling the RX echo while in RS485 mode */

	struct pm_qos_request	pm_qos_request;
	u32			latency;
	u32			calc_latency;
	struct work_struct	qos_work;
	bool			is_suspending;
	struct platform_device* plugin1dev;
	struct platform_device* plugin2dev;
#ifdef EXOR_MPI
	struct s_MPIdata MPIdata;
#endif
};

#define to_uart_omap_port(p)	((container_of((p), struct uart_omap_port, port)))

static struct uart_omap_port *ui[OMAP_MAX_HSUART_PORTS];

/* Forward declaration of functions */
static void serial_omap_mdr1_errataset(struct uart_omap_port *up, u8 mdr1);

static struct workqueue_struct *serial_omap_uart_wq;

static inline unsigned int serial_in(struct uart_omap_port *up, int offset)
{
	offset <<= up->port.regshift;
	return readw(up->port.membase + offset);
}

static inline void serial_out(struct uart_omap_port *up, int offset, int value)
{
	offset <<= up->port.regshift;
	writew(value, up->port.membase + offset);
}

static inline void serial_omap_clear_fifos(struct uart_omap_port *up)
{
	serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO);
	serial_out(up, UART_FCR, UART_FCR_ENABLE_FIFO |
		       UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
	serial_out(up, UART_FCR, 0);
}

static int serial_omap_get_context_loss_count(struct uart_omap_port *up)
{
	struct omap_uart_port_info *pdata = dev_get_platdata(up->dev);

	if (!pdata || !pdata->get_context_loss_count)
		return -EINVAL;

	return pdata->get_context_loss_count(up->dev);
}

static void serial_omap_enable_wakeup(struct uart_omap_port *up, bool enable)
{
	struct omap_uart_port_info *pdata = dev_get_platdata(up->dev);

	if (!pdata || !pdata->enable_wakeup)
		return;

	pdata->enable_wakeup(up->dev, enable);
}

/*
 * serial_omap_baud_is_mode16 - check if baud rate is MODE16X
 * @port: uart port info
 * @baud: baudrate for which mode needs to be determined
 *
 * Returns true if baud rate is MODE16X and false if MODE13X
 * Original table in OMAP TRM named "UART Mode Baud Rates, Divisor Values,
 * and Error Rates" determines modes not for all common baud rates.
 * E.g. for 1000000 baud rate mode must be 16x, but according to that
 * table it's determined as 13x.
 */
static bool serial_omap_baud_is_mode16(struct uart_port *port, unsigned int baud)
{
	unsigned int n13 = port->uartclk / (13 * baud);
	unsigned int n16 = port->uartclk / (16 * baud);
	int baudAbsDiff13 = n13 ? (baud - (port->uartclk / (13 * n13))) : INT_MAX;
	int baudAbsDiff16 = n16 ? (baud - (port->uartclk / (16 * n16))) : INT_MAX;
	if(baudAbsDiff13 < 0)
		baudAbsDiff13 = -baudAbsDiff13;
	if(baudAbsDiff16 < 0)
		baudAbsDiff16 = -baudAbsDiff16;

	return (baudAbsDiff13 > baudAbsDiff16);
}

/*
 * serial_omap_get_divisor - calculate divisor value
 * @port: uart port info
 * @baud: baudrate for which divisor needs to be calculated.
 */
static unsigned int serial_omap_get_divisor(struct uart_port *port, unsigned int baud)
{
	unsigned int divisor;

	if (!serial_omap_baud_is_mode16(port, baud))
		divisor = 13;
	else
		divisor = 16;
	return port->uartclk/(baud * divisor);
}

static void serial_omap_enable_ms(struct uart_port *port)
{
	struct uart_omap_port *up = to_uart_omap_port(port);

	dev_dbg(up->port.dev, "serial_omap_enable_ms+%d\n", up->port.line);

	pm_runtime_get_sync(up->dev);
	up->ier |= UART_IER_MSI;
	serial_out(up, UART_IER, up->ier);
	pm_runtime_mark_last_busy(up->dev);
	pm_runtime_put_autosuspend(up->dev);
}

static void serial_omap_stop_tx(struct uart_port *port)
{
	struct uart_omap_port *up = to_uart_omap_port(port);
	struct circ_buf *xmit = &up->port.state->xmit;
	int res;

	pm_runtime_get_sync(up->dev);
#ifdef EXOR_MPI
#ifdef USE_WAIT_TX_EMPTY
	if (up->MPIdata.MPIenabled) {
		/* if there's no more data to send, turn off rts */
		//wait untill TX is EMPTY
		while((serial_in(up, UART_LSR) & UART_LSR_TEMT) == 0)
			;	//wait max 53us at 187500 baud
		if(gpio_is_valid(up->rts_gpio)) {
			/* if rts not already disabled */
			res = (up->rs485.flags & SER_RS485_RTS_AFTER_SEND) ? 1 : 0;
			if (gpio_get_value(up->rts_gpio) != res) {
				if (up->rs485.delay_rts_after_send > 0) {
					mdelay(up->rs485.delay_rts_after_send);
				}
				gpio_set_value(up->rts_gpio, res);
			}
		} else {
			//Enabled RS485/422 mode, but no rts_gpio pin available, use RTS native pin
			unsigned char tmpmcr;
			if(!(up->rs485.flags & SER_RS485_RTS_AFTER_SEND)) {
				tmpmcr = serial_in(up, UART_MCR);
				tmpmcr &= ~UART_MCR_RTS;
				serial_out(up, UART_MCR, tmpmcr);
			}
		}
	} else 
#endif	
	{
		/* Handle RS-485 */
		if (up->rs485.flags & SER_RS485_ENABLED) {
			if (up->scr & OMAP_UART_SCR_TX_EMPTY) {
				/* THR interrupt is fired when both TX FIFO and TX
				 * shift register are empty. This means there's nothing
				 * left to transmit now, so make sure the THR interrupt
				 * is fired when TX FIFO is below the trigger level,
				 * disable THR interrupts and toggle the RS-485 GPIO
				 * data direction pin if needed.
				 */
				up->scr &= ~OMAP_UART_SCR_TX_EMPTY;
				serial_out(up, UART_OMAP_SCR, up->scr);

				/* if there's no more data to send, turn off rts */
#ifndef	USE_WAIT_TX_EMPTY 
				if (up->MPIdata.MPIenabled || uart_circ_empty(xmit))
#else
				if (uart_circ_empty(xmit))
#endif
				{
					if(gpio_is_valid(up->rts_gpio))
					{
						/* if rts not already disabled */
						res = (up->rs485.flags & SER_RS485_RTS_AFTER_SEND) ? 1 : 0;
						if (gpio_get_value(up->rts_gpio) != res) {
							if (up->rs485.delay_rts_after_send > 0) {
								mdelay(up->rs485.delay_rts_after_send);
							}
							gpio_set_value(up->rts_gpio, res);
						}
					}
					else
					{
						//Enabled RS485/422 mode, but no rts_gpio pin available, use RTS native pin
						unsigned char tmpmcr;
						if(!(up->rs485.flags & SER_RS485_RTS_AFTER_SEND)) {
							tmpmcr = serial_in(up, UART_MCR);
							tmpmcr &= ~UART_MCR_RTS;
							serial_out(up, UART_MCR, tmpmcr);
						}
					}
				}
			} else {
				/* We're asked to stop, but there's still stuff in the
				 * UART FIFO, so make sure the THR interrupt is fired
				 * when both TX FIFO and TX shift register are empty.
				 * The next THR interrupt (if no transmission is started
				 * in the meantime) will indicate the end of a
				 * transmission. Therefore we _don't_ disable THR
				 * interrupts in this situation.
				 */
				up->scr |= OMAP_UART_SCR_TX_EMPTY;
				serial_out(up, UART_OMAP_SCR, up->scr);
				return;
			}
		}
	}
	if (up->ier & UART_IER_THRI) {
		up->ier &= ~UART_IER_THRI;
		serial_out(up, UART_IER, up->ier);
	}

	if ((up->rs485.flags & SER_RS485_ENABLED) &&
		!(up->rs485.flags & SER_RS485_RX_DURING_TX))
	{
	  //RX enable by using the prg phy dedicated gpio pin
	  if (gpio_is_valid(up->rxen_gpio))
		gpio_set_value(up->rxen_gpio, 1);

	  up->ier = UART_IER_RLSI | UART_IER_RDI;
	  serial_out(up, UART_IER, up->ier);
	}
	if (up->MPIdata.MPIenabled) {
		ev_move(&up->MPIdata, EV_BRC_EOTX);
	}
#else
	/* Handle RS-485 */
	if (up->rs485.flags & SER_RS485_ENABLED) {
		if (up->scr & OMAP_UART_SCR_TX_EMPTY) {
			/* THR interrupt is fired when both TX FIFO and TX
			 * shift register are empty. This means there's nothing
			 * left to transmit now, so make sure the THR interrupt
			 * is fired when TX FIFO is below the trigger level,
			 * disable THR interrupts and toggle the RS-485 GPIO
			 * data direction pin if needed.
			 */
			up->scr &= ~OMAP_UART_SCR_TX_EMPTY;
			serial_out(up, UART_OMAP_SCR, up->scr);

			/* if there's no more data to send, turn off rts */
			if (uart_circ_empty(xmit))
			{
			  if(gpio_is_valid(up->rts_gpio))
			  {
				  /* if rts not already disabled */
				  res = (up->rs485.flags & SER_RS485_RTS_AFTER_SEND) ? 1 : 0;
				  if (gpio_get_value(up->rts_gpio) != res) {
					  if (up->rs485.delay_rts_after_send > 0) {
						  mdelay(up->rs485.delay_rts_after_send);
					  }
					  gpio_set_value(up->rts_gpio, res);
				  }
			  }
			  else
			  {
				//Enabled RS485/422 mode, but no rts_gpio pin available, use RTS native pin
				unsigned char tmpmcr;
				if(!(up->rs485.flags & SER_RS485_RTS_AFTER_SEND))
				{
				  tmpmcr = serial_in(up, UART_MCR);
				  tmpmcr &= ~UART_MCR_RTS;
				  serial_out(up, UART_MCR, tmpmcr);
				}
			  }
			}
		} else {
			/* We're asked to stop, but there's still stuff in the
			 * UART FIFO, so make sure the THR interrupt is fired
			 * when both TX FIFO and TX shift register are empty.
			 * The next THR interrupt (if no transmission is started
			 * in the meantime) will indicate the end of a
			 * transmission. Therefore we _don't_ disable THR
			 * interrupts in this situation.
			 */
			up->scr |= OMAP_UART_SCR_TX_EMPTY;
			serial_out(up, UART_OMAP_SCR, up->scr);
			return;
		}
	}
	if (up->ier & UART_IER_THRI) {
		up->ier &= ~UART_IER_THRI;
		serial_out(up, UART_IER, up->ier);
	}

	if ((up->rs485.flags & SER_RS485_ENABLED) &&
	    !(up->rs485.flags & SER_RS485_RX_DURING_TX)) 
	{
	  //RX enable by using the prg phy dedicated gpio pin
	  if (gpio_is_valid(up->rxen_gpio)) 
	    gpio_set_value(up->rxen_gpio, 1);
	  
	  up->ier = UART_IER_RLSI | UART_IER_RDI;
	  serial_out(up, UART_IER, up->ier);
	}
#endif

	pm_runtime_mark_last_busy(up->dev);
	pm_runtime_put_autosuspend(up->dev);
}

static void serial_omap_stop_rx(struct uart_port *port)
{
	struct uart_omap_port *up = to_uart_omap_port(port);

	pm_runtime_get_sync(up->dev);
	up->ier &= ~UART_IER_RLSI;
	up->port.read_status_mask &= ~UART_LSR_DR;
	serial_out(up, UART_IER, up->ier);
	pm_runtime_mark_last_busy(up->dev);
	pm_runtime_put_autosuspend(up->dev);
}

static void transmit_chars(struct uart_omap_port *up, unsigned int lsr)
{
#ifdef EXOR_MPI
	if (up->MPIdata.MPIenabled) {
		int count;

		int i = serial_in(up, OMAP_TX_FIFO_LVL) & 0xFF;
		if (up->MPIdata.txCnt == 0) {
			serial_omap_stop_tx(&up->port);
			return;
		}

		count = up->port.fifosize - i;	//fill txFIFO up to size if possible
		local_irq_disable();
		{
			do {
				serial_out(up, UART_TX, up->MPIdata.mpiTxBuf[up->MPIdata.txIdx++]);
				up->MPIdata.txCnt--;
				if (up->MPIdata.txCnt == 0)
					break;
			} while (--count > 0);
		}
		local_irq_enable();
//		up->MPIdata.txfullflag=0;

//		if (up->MPIdata.txCnt != 0)
//			up->MPIdata.txfullflag=1;
	}
	else
#endif
	{
		struct circ_buf *xmit = &up->port.state->xmit;
		int count;

		if (up->port.x_char) {
			serial_out(up, UART_TX, up->port.x_char);
			up->port.icount.tx++;
			up->port.x_char = 0;
			return;
		}
		if (uart_circ_empty(xmit) || uart_tx_stopped(&up->port)) {
			serial_omap_stop_tx(&up->port);
			return;
		}
		count = up->port.fifosize / 4;
		do {
			serial_out(up, UART_TX, xmit->buf[xmit->tail]);
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
			up->port.icount.tx++;
			if (uart_circ_empty(xmit))
				break;
		} while (--count > 0);

		if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS) {
			spin_unlock(&up->port.lock);
			uart_write_wakeup(&up->port);
			spin_lock(&up->port.lock);
		}

		if (uart_circ_empty(xmit))
			serial_omap_stop_tx(&up->port);
	}
}

static inline void serial_omap_enable_ier_thri(struct uart_omap_port *up)
{
	if (!(up->ier & UART_IER_THRI)) {
		up->ier |= UART_IER_THRI;
		serial_out(up, UART_IER, up->ier);
	}
}

static void serial_omap_start_tx(struct uart_port *port)
{
	struct uart_omap_port *up = to_uart_omap_port(port);
	int res;

	pm_runtime_get_sync(up->dev);

	/* handle rs485 */
	if (up->rs485.flags & SER_RS485_ENABLED)
	{
	  /* Fire THR interrupts when FIFO is below trigger level */
	  up->scr &= ~OMAP_UART_SCR_TX_EMPTY;
	  serial_out(up, UART_OMAP_SCR, up->scr);
	  
	  if(gpio_is_valid(up->rts_gpio))
	  {
		/* if rts not already enabled */
		res = (up->rs485.flags & SER_RS485_RTS_ON_SEND) ? 1 : 0;
		if (gpio_get_value(up->rts_gpio) != res) {
			gpio_set_value(up->rts_gpio, res);
			if (up->rs485.delay_rts_before_send > 0) {
				mdelay(up->rs485.delay_rts_before_send);
			}
		}
	  }
	  else
	  {
	    //Enabled RS485/422 mode, but no rts_gpio pin available, use RTS native pin
	    unsigned char tmpmcr;
	    tmpmcr = serial_in(up, UART_MCR);
	    tmpmcr |= UART_MCR_RTS;
	    serial_out(up, UART_MCR, tmpmcr);
	  }
	}
	else
	{
	  /*
	  * If we are in RS232 mode and we have a programmable phy, enable the TX if not yet done.
	  */
		if (gpio_is_valid(up->mode_gpio)) {
			if (gpio_is_valid(up->rts_gpio))
			{
				res = (up->rs485.flags & SER_RS485_RTS_ON_SEND) ? 1 : 0;
				if (gpio_get_value(up->rts_gpio) != res)
				{
					gpio_set_value(up->rts_gpio, res);
				}
			}
		}
	}

	if ((up->rs485.flags & SER_RS485_ENABLED) &&
	    !(up->rs485.flags & SER_RS485_RX_DURING_TX))
	{
	  //RX disable by using the prg phy dedicated gpio pin
	  if (gpio_is_valid(up->rxen_gpio)) 
	    gpio_set_value(up->rxen_gpio, 0);
	  
	  serial_omap_stop_rx(port);
	}

	serial_omap_enable_ier_thri(up);
	pm_runtime_mark_last_busy(up->dev);
	pm_runtime_put_autosuspend(up->dev);
}

static void serial_omap_throttle(struct uart_port *port)
{
	struct uart_omap_port *up = to_uart_omap_port(port);
	unsigned long flags;

	pm_runtime_get_sync(up->dev);
	spin_lock_irqsave(&up->port.lock, flags);
	up->ier &= ~(UART_IER_RLSI | UART_IER_RDI);
	serial_out(up, UART_IER, up->ier);
	spin_unlock_irqrestore(&up->port.lock, flags);
	pm_runtime_mark_last_busy(up->dev);
	pm_runtime_put_autosuspend(up->dev);
}

static void serial_omap_unthrottle(struct uart_port *port)
{
	struct uart_omap_port *up = to_uart_omap_port(port);
	unsigned long flags;

	pm_runtime_get_sync(up->dev);
	spin_lock_irqsave(&up->port.lock, flags);
	up->ier |= UART_IER_RLSI | UART_IER_RDI;
	serial_out(up, UART_IER, up->ier);
	spin_unlock_irqrestore(&up->port.lock, flags);
	pm_runtime_mark_last_busy(up->dev);
	pm_runtime_put_autosuspend(up->dev);
}

static unsigned int check_modem_status(struct uart_omap_port *up)
{
	unsigned int status;

	status = serial_in(up, UART_MSR);
	status |= up->msr_saved_flags;
	up->msr_saved_flags = 0;
	if ((status & UART_MSR_ANY_DELTA) == 0)
		return status;

	if (status & UART_MSR_ANY_DELTA && up->ier & UART_IER_MSI &&
	    up->port.state != NULL) {
		if (status & UART_MSR_TERI)
			up->port.icount.rng++;
		if (status & UART_MSR_DDSR)
			up->port.icount.dsr++;
		if (status & UART_MSR_DDCD)
			uart_handle_dcd_change
				(&up->port, status & UART_MSR_DCD);
		if (status & UART_MSR_DCTS)
			uart_handle_cts_change
				(&up->port, status & UART_MSR_CTS);
		wake_up_interruptible(&up->port.state->port.delta_msr_wait);
	}

	return status;
}

static void serial_omap_rlsi(struct uart_omap_port *up, unsigned int lsr)
{
	unsigned int flag;
	unsigned char ch = 0;

	if (likely(lsr & UART_LSR_DR))
		ch = serial_in(up, UART_RX);

	up->port.icount.rx++;
	flag = TTY_NORMAL;

	if (lsr & UART_LSR_BI) {
		flag = TTY_BREAK;
		lsr &= ~(UART_LSR_FE | UART_LSR_PE);
		up->port.icount.brk++;
		/*
		 * We do the SysRQ and SAK checking
		 * here because otherwise the break
		 * may get masked by ignore_status_mask
		 * or read_status_mask.
		 */
		if (uart_handle_break(&up->port))
			return;

	}

	if (lsr & UART_LSR_PE) {
		flag = TTY_PARITY;
		up->port.icount.parity++;
	}

	if (lsr & UART_LSR_FE) {
		flag = TTY_FRAME;
		up->port.icount.frame++;
	}

	if (lsr & UART_LSR_OE)
		up->port.icount.overrun++;

#ifdef CONFIG_SERIAL_OMAP_CONSOLE
	if (up->port.line == up->port.cons->index) {
		/* Recover the break flag from console xmit */
		lsr |= up->lsr_break_flag;
	}
#endif
	uart_insert_char(&up->port, lsr, UART_LSR_OE, 0, flag);
}
#ifdef EXOR_MPI
struct ValueName
{
	int value;
	const char *name;
};

// event names
struct ValueName event_name[] =
{
	{ EV__NULL, "EV__NULL" },
	{ EV_BRC_ENABLE, "EV_BRC_ENABLE" },
	{ EV_BRC_ACK, "EV_BRC_ACK" },
	{ EV_BRC_TOK, "EV_BRC_TOK" },
	{ EV_BRC_SD1, "EV_BRC_SD1" },
	{ EV_BRC_SD2, "EV_BRC_SD2" },
	{ EV_BRC_SDX, "EV_BRC_SDX" },
	{ EV_BRC_EOTX, "EV_BRC_EOTX" },
	{ EV_BRC_APPL_TIMEOUT, "EV_BRC_APPL_TIMEOUT" },
	{ EV_BRC_ACK_TIMEOUT, "EV_BRC_ACK_TIMEOUT" },
	{ EV_BRC_SL_SESS_TIMEOUT, "EV_BRC_SL_SESS_TIMEOUT" },
	{ EV_BRC_STOP_SESS, "EV_BRC_STOP_SESS" },
	{ EV_BRC_J0S_SESS_TIMEOUT, "EV_BRC_J0S_SESS_TIMEOUT" },
	{ EV_BRC_JBS_SESS_TIMEOUT, "EV_BRC_JBS_SESS_TIMEOUT" },
	{ EV_SES_ACK, "EV_SES_ACK" },
	{ EV_SES_SD2, "EV_SES_SD2" },
	{ EV_SES_ERR, "EV_SES_ERR" },
	{ EV_SES_SD1, "EV_SES_SD1" },
	{ EV_SES_RETRY, "EV_SES_RETRY" },
	{ EV_LOG_RUN, "EV_LOG_RUN" },
	{ EV_LOG_START, "EV_LOG_START" },
	{ EV_LOG_OFF, "EV_LOG_OFF" },
	{ EV_TOK_RUN, "EV_TOK_RUN" },
	{ EV_TOK_ACTIVEIDLE, "EV_TOK_ACTIVEIDLE" },
	{ EV_TOK_FDLSTATUS, "EV_TOK_FDLSTATUS" },
	{ EV_TOK_WAITSESSRX, "EV_TOK_WAITSESSRX" },
	{ EV_TOK_FDLTIMEOUT, "EV_TOK_FDLTIMEOUT" },
	{ EV_TOK_TIMEOUT, "EV_TOK_TIMEOUT" },
	{ EV_TOK_TXFRAME, "EV_TOK_TXFRAME" },
	{ EV_TOK_PASSTOKEN, "EV_TOK_PASSTOKEN" },
	{ EV_TOK_SELFTOKEN, "EV_TOK_SELFTOKEN" },
	{ EV_TOK_TIMEOUT_TOK, "EV_TOK_TIMEOUT_TOK" },
	{ EV_TOK_SELFTOK_TIMEOUT, "EV_TOK_SELFTOK_TIMEOUT" },
	{ EV_TOK_NO_RING_ACTIVITY, "EV_TOK_NO_RING_ACTIVITY" },
	{ EV_JB0_RUN, "EV_JB0_RUN" },
	{ EV_JB0_SEND_REQ, "EV_JB0_SEND_REQ" },
	{ EV_JOB_RUN, "EV_JOB_RUN" },
	{ EV_JOB_SEND_REQ, "EV_JOB_SEND_REQ" },
	{ EV_LGS_RUN, "EV_LGS_RUN" },
	{ EV_LGS_SEND_SAPRSP, "EV_LGS_SEND_SAPRSP" },
	{ EV_LGS_SEND_ACKSAPRSPACK, "EV_LGS_SEND_ACKSAPRSPACK" },
	{ EV_J0S_RUN, "EV_J0S_RUN" },
	{ EV_J0S_SEND_JOBACK, "EV_J0S_SEND_JOBACK" },
	{ EV_J0S_SEND_JOBRSP, "EV_J0S_SEND_JOBRSP" },
	{ EV_JBS_RUN, "EV_JBS_RUN" },
	{ EV_JBS_SEND_JOBACK, "EV_JBS_SEND_JOBACK" },
	{ EV_JBS_SEND_JOBRSP, "EV_JBS_SEND_JOBRSP" },
	{ EV_LGF_RUN, "EV_LGF_RUN" },
	{ EV_LGF_SEND_LOGOFFACK, "EV_LGF_SEND_LOGOFFACK" },
	{ EV_TX_DELAY_DONE, "EV_TX_DELAY_DONE" },
};
// state names
struct ValueName state_name[] =
{
{ _NULL, "_NULL" },
{ TOK_IDLE, "TOK_IDLE" },
{ TOK_LISTENTOKEN, "TOK_LISTENTOKEN" },
{ TOK_ACTIVEIDLE, "TOK_ACTIVEIDLE" },
{ TOK_WAITRX, "TOK_WAITRX" },
{ TOK_TOKEN_RETRY, "TOK_TOKEN_RETRY" },
{ TOK_WAITFDLSTATUS, "TOK_WAITFDLSTATUS" },
{ TOK_WAITFDLSTATUS2, "TOK_WAITFDLSTATUS2" },
{ TOK_WAITSESSRX, "TOK_WAITSESSRX" },
{ TOK_SELFTOKEN, "TOK_SELFTOKEN" },
{ LOG_IDLE, "LOG_IDLE" },
{ LOG_RUN, "LOG_RUN" },
{ LOG_WAITSAPREQACK, "LOG_WAITSAPREQACK" },
{ LOG_WAITSAPRSP, "LOG_WAITSAPRSP" },
{ LOG_WAITSAPRSPACK, "LOG_WAITSAPRSPACK" },
{ LOG_WAITACKSAPRSPACK, "LOG_WAITACKSAPRSPACK" },
{ JB0_IDLE, "JB0_IDLE" },
{ JB0_RUN, "JB0_RUN" },
{ JB0_WAITSHORTACK0, "JB0_WAITSHORTACK0" },
{ JB0_WAITJOBACK0, "JB0_WAITJOBACK0" },
{ JB0_WAITJOBRESP0, "JB0_WAITJOBRESP0" },
{ JB0_WAITSHORTACK02, "JB0_WAITSHORTACK02" },
{ JOB_IDLE, "JOB_IDLE" },
{ JOB_RUN, "JOB_RUN" },
{ JOB_WAITSHORTACK, "JOB_WAITSHORTACK" },
{ JOB_WAITJOBACK, "JOB_WAITJOBACK" },
{ JOB_WAITJOBRESP, "JOB_WAITJOBRESP" },
{ JOB_WAITSHORTACK2, "JOB_WAITSHORTACK2" },
{ LGS_IDLE, "LGS_IDLE" },
{ LGS_RUN, "LGS_RUN" },
{ LGS_SEND_SAPRSP, "LGS_SEND_SAPRSP" },
{ LGS_ACKSAPRSP, "LGS_ACKSAPRSP" },
{ LGS_SEND_ACKSAPRSPACK, "LGS_SEND_ACKSAPRSPACK" },
{ J0S_IDLE, "J0S_IDLE" },
{ J0S_RUN, "J0S_RUN" },
{ J0S_SEND_JOBACK, "J0S_SEND_JOBACK" },
{ J0S_SEND_JOBRSP, "J0S_SEND_JOBRSP" },
{ J0S_WAIT_ACK_JOBRSP, "J0S_WAIT_ACK_JOBRSP" },
{ JBS_IDLE, "JBS_IDLE" },
{ JBS_RUN, "JBS_RUN" },
{ JBS_SEND_JOBACK, "JBS_SEND_JOBACK" },
{ JBS_SEND_JOBRSP, "JBS_SEND_JOBRSP" },
{ JBS_WAIT_ACK_JOBRSP, "JBS_WAIT_ACK_JOBRSP" },
{ LGF_IDLE, "LGF_IDLE" },
{ LGF_RUN, "LGF_RUN" },
{ LGF_SENT_LOGOFF_ACK, "LGF_SENT_LOGOFF_ACK" },
};

#ifdef DEBUG_STATE
#include <sys/time.h>
#include <string>       // std::string
#include <sstream>      // std::stringstream
#include <fstream>      // std::stringstream
#include <deque>

static const char *eventName(int ev)
{
	for (int i=0; i<sizeof(event_name)/sizeof(ValueName); ++i)
	{
		if (event_name[i].value == ev)
			return event_name[i].name;
	}
	return "unknown";
};

static const char *stateName(int state)
{
	for (int i=0; i<sizeof(state_name)/sizeof(ValueName); ++i)
	{
		if (state_name[i].value == state)
			return state_name[i].name;
	}
	return "unknown";
};

class MsgContainer
{
	class UsTimer
	{
	public:
		typedef unsigned int Value;
		UsTimer()
		{
			timespec time;
			clock_gettime( CLOCK_MONOTONIC, &time );
			m_startUs = time.tv_sec * 1000000 + time.tv_nsec/1000;
		}
		Value getUs() const
		{
			timespec time;
			clock_gettime( CLOCK_MONOTONIC, &time );
			Value us = time.tv_sec * 1000000 + time.tv_nsec/1000;
			return us - m_startUs;
		}
	private:
		Value m_startUs;
	};

	struct DiagMsg
	{
		UsTimer::Value time;
		std::string type;
		std::string msg;

		std::ostream &dump(std::ostream &ostr) const
		{
			ostr << std::setfill('0') << std::setw(8) << time << " " << type << " = " << msg << "\n";
			return ostr;
		}
	};

public:
	MsgContainer(int maxSize, bool cycling, bool running)
		: m_maxSize(maxSize),
		  m_cycling(cycling),
		  m_running(running)
	{
	}

	void run()
	{
		m_running = true;
		append("setMode", "run");
	}

	void stop()
	{
		append("setMode", "stop");
		m_running = false;
	}

	void append(const char *type, const char *msg)
	{
		if (m_running)
		{
			if (m_cycling && (m_messages.size() >= m_maxSize))
			{
				// make new place
				m_messages.pop_front();
			}
			if (m_messages.size() < m_maxSize)
			{
				// add new item
				DiagMsg diagMsg;
				diagMsg.time = usTimer.getUs();
				diagMsg.type = type;
				diagMsg.msg = msg;
				m_messages.push_back(diagMsg);
			}
		}
	}

	void append(const char *type, int value)
	{
		std::stringstream stream;
		stream << value;
		std::string str = stream.str();
		append(type, str.c_str());
	}

	void append(const char *type, unsigned char *arr, int len)
	{
		std::stringstream stream;
		for (int i=0; i<len; ++i)
		{
			stream << " " << std::hex << std::setfill('0') << std::setw(2) << (int)arr[i];
		}
		std::string str = stream.str();
		append(type, str.c_str());
	}

	friend std::ostream &operator <<(std::ostream &ostr, const MsgContainer &container);

private:
	UsTimer usTimer;
	std::deque<DiagMsg> m_messages;
	int m_maxSize;
	bool m_cycling;
	bool m_running;
};

inline std::ostream &operator <<(std::ostream &ostr, const MsgContainer &container)
{
	for (std::deque<MsgContainer::DiagMsg>::const_iterator it = container.m_messages.begin(); it != container.m_messages.end(); ++it)
		it->dump(ostr);
	return ostr;
}

static MsgContainer msgContainer(10000, true, true);

static void dumpRun()
{
	msgContainer.run();
}

static void dumpStop()
{
	msgContainer.stop();
}

static void dumpEvent(int ev)
{
	msgContainer.append("EVENT", eventName(ev));
}

static void dumpTokState(int tokState)
{
	static int old = -1;
	if (old != tokState)
	{
		old = tokState;
		msgContainer.append("TOK_STATE", stateName(tokState));
	}
}

static void dumpLogState(int state)
{
	static int old = -1;
	if (old != state)
	{
		old = state;
		msgContainer.append("LOG_STATE", stateName(state));
	}
}

static void dumpJb0State(int state)
{
	static int old = -1;
	if (old != state)
	{
		old = state;
		msgContainer.append("JB0_STATE", stateName(state));
	}
}

static void dumpJobState(int state)
{
	static int old = -1;
	if (old != state)
	{
		old = state;
		msgContainer.append("JOB_STATE", stateName(state));
	}
}

static void dumpInteger(const char *name, int n)
{
	msgContainer.append(name, n);
}

static void dumpTimeout(int type, int value)
{
	std::stringstream stream;
	stream << "TIMEOUT_" << type;
	std::string str = stream.str();
	msgContainer.append(str.c_str(), value);
}

static void dumpArray(const char *type, unsigned char *arr, int len)
{
	msgContainer.append(type, arr, len);
}

static void dumpDebug(const char *debug)
{
	msgContainer.append("DEBUG", debug);
}

static void printDiagMessages()
{
	// std::cout << msgContainer;
	std::ofstream file("diag.txt", std::ofstream::out);
	if (file.good())
	{
		file << msgContainer;
		file.close();
	}
}

#else
#define dumpRun()
#define dumpStop()
#define dumpEvent(x)
#define dumpTokState(x)
#define dumpLogState(x)
#define dumpJb0State(x)
#define dumpJobState(x)
#define dumpInteger(x, y)
#define dumpTimeout(x, y)
#define dumpDebug(x)
#define dumpArray(x, y, z)
#define printDiagMessages()
#endif // DEBUG_STATE

static void CalcCheckSum(unsigned char *buf, int start, int fcsPos)
{
	unsigned char sum = 0;
	int i;
	for (i=start; i<fcsPos; ++i)
		sum += buf[i];
	buf[fcsPos] = sum;
}

static void RtlCopyMemory(unsigned char *dest, const unsigned char *src, int len)
{
	memcpy(dest, src, len);
}
void UltiUART1_StartTimer(struct s_MPIdata *pMPIdata, unsigned char ev, int timeoutUSec);

enum hrtimer_restart hrtCallBack(struct hrtimer *phrt)
{
	struct s_MPIdata *pMPIdata = container_of(phrt, struct s_MPIdata, hrt);
	unsigned long flags=0;
	struct uart_omap_port *up = container_of(pMPIdata, struct uart_omap_port, MPIdata);
	spin_lock_irqsave(&pMPIdata->mpiUp->port.lock, flags);
	if (pMPIdata->UltiUart1_TxTimeout != EV__NULL && pMPIdata->UltiUart1_TxTimeout != EV_TOK_RUN && (serial_in(up, UART_RXFIFO_LVL) & 0xff) != 0)
		UltiUART1_StartTimer(pMPIdata, pMPIdata->UltiUart1_TxTimeout, pMPIdata->UltiUart1_TxNunUSec);
	else
		ev_move(pMPIdata, pMPIdata->UltiUart1_TxTimeout);
	spin_unlock_irqrestore(&pMPIdata->mpiUp->port.lock, flags);
	return HRTIMER_NORESTART;
}

void MPIDriverInit(struct s_MPIdata *pMPIdata)
{
	hrtimer_init(&pMPIdata->hrt, CLOCK_MONOTONIC,HRTIMER_MODE_REL);
	pMPIdata->hrt.function = hrtCallBack;
	pMPIdata->MPIenabled = false;

	// initialize variables (previous globals)
	pMPIdata->NumTokenRotations = 0;
	pMPIdata->TokenNotForMe = 0;
	pMPIdata->LastStation = 0;

	pMPIdata->SessReqPending = false;
	pMPIdata->SessionStarted = false;
	pMPIdata->JobRetry = 0;
	pMPIdata->LogSessReqPending = false;
	pMPIdata->LgfSessReqPending = false;
	pMPIdata->Job0SessReqPending = false;
	pMPIdata->JobSessReqPending = false;

	pMPIdata->LgsSessReqPending = false;
	pMPIdata->J0sSessReqPending = false;
	pMPIdata->JbsSessReqPending = false;

	pMPIdata->MyOperFlag = 0;
	memset(pMPIdata->WrMyBuff, 0, sizeof(pMPIdata->WrMyBuff));
	memset(pMPIdata->FrameMyBuff, 0, sizeof(pMPIdata->FrameMyBuff));
	pMPIdata->WrMyBuffLength = 0;
	pMPIdata->FrameMyBuffLength = 0;
	pMPIdata->MyLen = 0;

	memset(pMPIdata->countArray, 0, sizeof(pMPIdata->countArray));
	memset(pMPIdata->interruptsCount, 0, sizeof(pMPIdata->interruptsCount));

	memset(pMPIdata->ev_queue, 0, sizeof(pMPIdata->ev_queue));
	pMPIdata->ev_queue_rd = 0;
	pMPIdata->ev_queue_wr = 0;
	pMPIdata->queue_empty = 1;
	pMPIdata->event = 0;
	pMPIdata->tok_state = TOK_IDLE;
	pMPIdata->lgf_state = LGF_IDLE;

#if MPI_MASTER_STACK
	pMPIdata->LogOn_Retry = 0;
	pMPIdata->log_state = LOG_IDLE;
	pMPIdata->jb0_state = JB0_IDLE;
	pMPIdata->job_state = JOB_IDLE;
#endif
#if MPI_SLAVE_STACK
	pMPIdata->lgs_state = LGS_IDLE;
	pMPIdata->j0s_state = J0S_IDLE;
	pMPIdata->jbs_state = JBS_IDLE;
	pMPIdata->JbsOnSlave_Job = 0;
#endif

	pMPIdata->FrameUniopNodeNum = 0;
	pMPIdata->FrameSendBufferLen1 = 0;
	pMPIdata->FrameSendBufferLen = 0;
	pMPIdata->FrameSessionSendLength = 0;
	pMPIdata->FrameSessionReceiveLength = 0;

	pMPIdata->ProcedureApplMStatus = 0;
	pMPIdata->ProcedureApplMRequestPending = 0;
	pMPIdata->ProcedureApplMResponsePending = 0;
	memset(pMPIdata->StationStatus, 0, sizeof(pMPIdata->StationStatus));
	pMPIdata->LowestAddress = 0;
	pMPIdata->NextGAP = 0;
	pMPIdata->AreWeInRing = 0;
	pMPIdata->NextStation = 0;
	pMPIdata->MaxStationAddress = 0;
	pMPIdata->applResponseTime = 0;
	pMPIdata->guardTimeConstant = 0;
	pMPIdata->guardTimeFactor = 0;
	pMPIdata->ackGuardTime = 0;
	pMPIdata->FDLTimeout = 0;
	pMPIdata->tokTimeout = 0;
	pMPIdata->selfTokTimeout = 0;
	pMPIdata->Source = 0;
	pMPIdata->Dest = 0;
	pMPIdata->FlowCtrl = 0;
	pMPIdata->MyFrameResponseBuffLen = 0;
	pMPIdata->FrameJobSessionLen = 0;
	pMPIdata->ReadyForConfigMode = 0;
	pMPIdata->SlaveSession = 0;

	pMPIdata->GapUpdateFactor = 0;
	pMPIdata->job_state = 0;
	pMPIdata->RingActivityCounter = 0;

	pMPIdata->LogOn_DA = 0;
	pMPIdata->LogOff_DA = 0;
	pMPIdata->Last_DA = 0;
	pMPIdata->LogOn_DAE = 0;
	pMPIdata->LogOn_SAE = 0;
	pMPIdata->CurrentShortAck = 0;
	pMPIdata->Start_LogOn_SAE = 0;

	pMPIdata->Start_LogOn_DAE = 0;
	pMPIdata->LogOnSlave_DA = 0;

#if MPI_SLAVE_STACK
	pMPIdata->lgs_state = 0;
	pMPIdata->j0s_state = 0;
	pMPIdata->jbs_state = 0;
#endif
	pMPIdata->job_state = 0;
	pMPIdata->log_state = 0;
	pMPIdata->lgf_state = 0;
	pMPIdata->jb0_state = 0;

	pMPIdata->startStopTrace = 0;
	pMPIdata->traceStatus = 0;
	pMPIdata->tracePostStopCnt = 0;

	pMPIdata->PassTokenReply    = 0;
	pMPIdata->FlagSendSelfToken = 0;
	pMPIdata->FlagPassToken     = 0;
	pMPIdata->Sd1RespGuard      = 0;

	pMPIdata->GapUpdateFactorCnt = 0;
	pMPIdata->NxtStat = 0;
	pMPIdata->cnt = 0;
	pMPIdata->generateEOTEvent = 0;
	pMPIdata->shortACK_EOTEvent = 0;
	pMPIdata->MPImode = true;
}


#if 0
void DBGReg(byte event, byte status, word param)
{
}
#else
#define DBGReg(A,B,C)
#endif
#define MAKEWORD(a, b)     ((word)(((byte)(a)) | ((word)((byte)(b))) << 8))
void ev_post(struct s_MPIdata *pMPIdata, unsigned char ev);

static void sendData(struct s_MPIdata *pMPIdata, char * buf, unsigned int len)
{
	struct uart_omap_port *up = container_of (pMPIdata, struct uart_omap_port, MPIdata);

	if (len == 1)
		pMPIdata->shortACK_EOTEvent = 1;

	memcpy(pMPIdata->mpiTxBuf, buf, len);
	pMPIdata->txCnt = len;
	pMPIdata->txIdx = 0;
	serial_omap_start_tx(&up->port);
}


void sendShortAck(struct s_MPIdata *pMPIdata)
{
	unsigned char c = D_SC;
	sendData(pMPIdata, &c, 1);
}

void UltiUART1_StartTimer(struct s_MPIdata *pMPIdata, unsigned char ev, int timeoutUSec)
{
	ktime_t kt = ktime_set(timeoutUSec / 1000000, (timeoutUSec % 1000000)*1000);
   pMPIdata->UltiUart1_TxTimeout = ev;
   pMPIdata->UltiUart1_TxNunUSec = timeoutUSec;
   hrtimer_start( &pMPIdata->hrt, kt, HRTIMER_MODE_REL );
}

void UltiUart1_StopTimer(struct s_MPIdata *pMPIdata)
{
	hrtimer_try_to_cancel(&pMPIdata->hrt);
	pMPIdata->UltiUart1_TxTimeout = EV__NULL;
}

/************************************************************************
*
* mInitEngine
*
*  This function Init the Stations and state machines status
*
* Parameters:
*  void
*
* Returns:
***********************************************************************/
void mInitEngine(struct s_MPIdata *pMPIdata)
{
   int i;
   // Initialise all the stations to inactive
   for (i = 0; i < NR_MAX_STATIONS; i++)
   {
	  pMPIdata->StationStatus[i].IsActive = 0;
	  pMPIdata->StationStatus[i].StationType = 0;
	  pMPIdata->StationStatus[i].FCV = 0;
	  pMPIdata->StationStatus[i].FCB = 1;
	  pMPIdata->StationStatus[i].Logged = 0;
	  pMPIdata->StationStatus[i].LogStatus = 0;   //MG001
	  pMPIdata->StationStatus[i].Job = 0;
	  pMPIdata->StationStatus[i].LogOn_SAE = pMPIdata->Start_LogOn_SAE++;
	  pMPIdata->StationStatus[i].LogOn_DAE = 0;
   }
   //reset all other state machines
   pMPIdata->lgf_state = LGF_IDLE;
   pMPIdata->log_state = LOG_IDLE;
   pMPIdata->jb0_state = JB0_IDLE;
   pMPIdata->job_state = JOB_IDLE;
#if MPI_SLAVE_STACK
   pMPIdata->lgs_state = LGS_IDLE;
   pMPIdata->j0s_state = J0S_IDLE;
   pMPIdata->jbs_state = JBS_IDLE;
#endif
   pMPIdata->tok_state = TOK_IDLE;
}

void Init_ev_queue(struct s_MPIdata *pMPIdata)
{
	memset(pMPIdata->ev_queue, 0, sizeof(pMPIdata->ev_queue));
	pMPIdata->ev_queue_rd = 0;
	pMPIdata->ev_queue_wr = 0;
	pMPIdata->queue_empty = 1;
}


// Post event
void ev_post(struct s_MPIdata *pMPIdata, unsigned char ev)
{
	pMPIdata->ev_queue[pMPIdata->ev_queue_rd] = ev;
	pMPIdata->ev_queue_rd = (pMPIdata->ev_queue_rd + 1) % N_EVPOST;
	pMPIdata->queue_empty = 0;
}

/************************************************************************
*
* ResetStations
*
*  This function reset the Stations Status
*
* Parameters:
*  void
*
* Returns:
***********************************************************************/
void ResetStations(struct s_MPIdata *pMPIdata)
{
byte i;

   for (i = 0; i < NR_MAX_STATIONS; i++)
   {
	  pMPIdata->StationStatus[i].IsActive = 0;
	  pMPIdata->StationStatus[i].StationType = 0;
	  pMPIdata->StationStatus[i].Job = 0;
	  pMPIdata->StationStatus[i].Logged = 0;
	  pMPIdata->StationStatus[i].LogOn_DAE = 0;
	  pMPIdata->StationStatus[i].LogStatus = 0; //MG001
   }
   pMPIdata->NextGAP = pMPIdata->FrameUniopNodeNum;
   pMPIdata->NextStation = -1;
   pMPIdata->AreWeInRing = false;
   pMPIdata->m_taskLen = 0;
   Init_ev_queue(pMPIdata);
}

#ifdef RETRY_IN_SAME_TOKEN
void ResetLastDAStation(struct s_MPIdata *pMPIdata)
{
	pMPIdata->StationStatus[pMPIdata->Last_DA].IsActive = 0;
	pMPIdata->StationStatus[pMPIdata->Last_DA].StationType = 0;
	pMPIdata->StationStatus[pMPIdata->Last_DA].Job = 0;
	pMPIdata->StationStatus[pMPIdata->Last_DA].Logged = 0;
	pMPIdata->StationStatus[pMPIdata->Last_DA].LogOn_DAE = 0;
	pMPIdata->StationStatus[pMPIdata->Last_DA].LogStatus = 0; //MG001
}
#endif
/************************************************************************
*
* PrepareNextGAP
*
*  This function returns the next station address in the GAP between
*  us and the next active station. The NextGAP station is a static
*  variable that is incremented by this function.
*
* Parameters:
*  void
*
* Returns:
*  -1 - no GAP
*  else - next GAP station
***********************************************************************/
#define MON_DEBUG 0
signed char PrepareNextGAP(struct s_MPIdata *pMPIdata)
{
	int i;

   #if MON_DEBUG
	  putCh('a');
   #endif

   if (pMPIdata->NextGAP == pMPIdata->MaxStationAddress)
	  pMPIdata->NextGAP = -1;

   for (i = 0; i < pMPIdata->MaxStationAddress; i++)
   {
	  #if MON_DEBUG
		 putHexByte(pMPIdata->NextGAP);
	  #endif
	  if (pMPIdata->StationStatus[pMPIdata->NextGAP + 1].IsActive && (pMPIdata->StationStatus[pMPIdata->NextGAP + 1].StationType >= 2))
	  {
		 if ((pMPIdata->NextGAP + 1) == (pMPIdata->FrameUniopNodeNum + 1))
		 {
			#if MON_DEBUG
			   putCh('d');
			#endif
			pMPIdata->NextGAP = pMPIdata->FrameUniopNodeNum;
			return -1;
		 }
		 pMPIdata->NextGAP = pMPIdata->FrameUniopNodeNum + 1;
		 #if MON_DEBUG
			putCh('b');
		 #endif
		 return pMPIdata->NextGAP;
	  }
	  else
	  {
		 if ((pMPIdata->NextGAP + 1) > pMPIdata->MaxStationAddress)
		 {
			pMPIdata->NextGAP = -1;
			#if MON_DEBUG
			   putCh('e');
			#endif
		 }
		 else
		 {
			if ((pMPIdata->NextGAP + 1) == pMPIdata->FrameUniopNodeNum)
			{
			   pMPIdata->NextGAP++;
			   #if MON_DEBUG
				  putCh('f');
			   #endif
			}
			else
			{
			   #if MON_DEBUG
				  putCh('g');
			   #endif
			   pMPIdata->NextGAP += 1;
			   return pMPIdata->NextGAP;
			}
		 }
	  }
   }
   return -1;	//MG001
}

/************************************************************************
*
* GetNextActiveStation
*
*  This function checks the LAS to determine the next active station
*  relative to us
*
* Parameters:
*  void
*
* Returns:
*  next active station
************************************************************************/
signed char GetNextActiveStation(struct s_MPIdata *pMPIdata)
{
	signed char  i;

   // Search over the whole array of station addresses starting from
   // StartNode to find the next master station, i.e. a station
   // registered as In the token ring or ready for it
   i = pMPIdata->FrameUniopNodeNum + 1;
   while (i != pMPIdata->FrameUniopNodeNum)
   {
	  if (i > pMPIdata->MaxStationAddress)
	  {
		 if (pMPIdata->FrameUniopNodeNum == 0)
			break;
		 else
			i = 0;
	  }
	  if (pMPIdata->StationStatus[i].IsActive && (pMPIdata->StationStatus[i].StationType >= 2))
		 return i;
	  i++;
   }
   return pMPIdata->FrameUniopNodeNum;
}

#if MPI_MASTER_STACK
//MG003 Start
void mExitFromRing(struct s_MPIdata *pMPIdata)
{

   pMPIdata->SessReqPending     = false;
   pMPIdata->SessionStarted     = false;
   pMPIdata->LgsSessReqPending  = false;
   pMPIdata->Job0SessReqPending = false;
   pMPIdata->JobSessReqPending  = false;

//   ProcedureApplMStatus = TIMEOUT_ERR;
#ifdef FATAL_ERRORS
   pMPIdata->ProcedureApplMStatus = GEN_COMM_ERR;
   DBGReg(239, pMPIdata->ProcedureApplMStatus, 0x0001);
   pMPIdata->ProcedureApplMRequestPending  = false;
   pMPIdata->ProcedureApplMResponsePending = false;
#else
   if (pMPIdata->ProcedureApplMResponsePending)
	  pMPIdata->ProcedureApplMRequestPending = true;
#endif

   ResetStations(pMPIdata);

   //reset all other state machines
   pMPIdata->lgf_state = LGF_IDLE;
   pMPIdata->log_state = LOG_IDLE;
   pMPIdata->jb0_state = JB0_IDLE;
   pMPIdata->job_state = JOB_IDLE;
#if MPI_SLAVE_STACK
   pMPIdata->lgs_state = LGS_IDLE;
   pMPIdata->j0s_state = J0S_IDLE;
   pMPIdata->jbs_state = JBS_IDLE;
#endif
   DBGReg(242, pMPIdata->tok_state, TOK_IDLE);
   pMPIdata->tok_state = TOK_IDLE;
   dumpTokState(pMPIdata->tok_state);

   UltiUART1_StartTimer(pMPIdata, EV_TOK_RUN,1000*1000);
}
//MG003 End


/*----LogOn--------------------------------*/
#define MAX_LOGON_RETRY 5

void mLogSendSAPReqFrame(struct s_MPIdata *pMPIdata)
{
   RtlCopyMemory(pMPIdata->FrameSessionSendBuffer, LogOnSAPReq, LOGON_SAPREQ_LEN);
   pMPIdata->FrameSessionSendBuffer[4] = pMPIdata->LogOn_DA | 0x80;
   pMPIdata->FrameSessionSendBuffer[5] = pMPIdata->FrameUniopNodeNum | 0x80;
   pMPIdata->FrameSessionSendBuffer[7] = pMPIdata->StationStatus[pMPIdata->LogOn_DA].LogOn_DAE;
   pMPIdata->FrameSessionSendBuffer[8] = pMPIdata->StationStatus[pMPIdata->LogOn_DA].LogOn_SAE;
   pMPIdata->FrameSessionSendLength = LOGON_SAPREQ_LEN;

   //Calcola FCS
   CalcCheckSum(pMPIdata->FrameSessionSendBuffer, 4, pMPIdata->FrameSessionSendLength - 2);

   pMPIdata->LogSessReqPending = true;
   pMPIdata->SessReqPending    = true;
}

byte LogOn_RespFrameOk(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->FrameSessionReceiveBuffer[9] != 0xD0)
   {
	  pMPIdata->StationStatus[pMPIdata->LogOn_DA].Job = 0;
	  return 0;
   }
   pMPIdata->StationStatus[pMPIdata->LogOn_DA].LogOn_DAE = pMPIdata->FrameSessionReceiveBuffer[8];
   return 1;
}

byte LogOn_AckRespFrameOk(struct s_MPIdata *pMPIdata)
{
   pMPIdata->StationStatus[pMPIdata->LogOn_DA].Logged = 1;
   pMPIdata->StationStatus[pMPIdata->LogOn_DA].Job   = 0;
   return 1;
}


unsigned char mLogAbortSession(struct s_MPIdata *pMPIdata)
{
   mExitFromRing(pMPIdata); //MG003
   return LOG_IDLE; //MG003
}


unsigned char mLogCheckLogOff(struct s_MPIdata *pMPIdata)
{
   //Exit from Ring!!!!
   if (pMPIdata->LogOn_Retry > MAX_LOGON_RETRY)
   {
	  pMPIdata->LogOn_Retry = 0;
	  ev_post(pMPIdata, EV_LOG_OFF);
	  pMPIdata->NumTokenRotations = 0;
	  pMPIdata->LowestAddress = 0xFF;
	  return LOG_RUN;
   }

   if (pMPIdata->LogSessReqPending && (pMPIdata->FrameSessionReceiveBuffer[9] == 0x80))
	  pMPIdata->LogOn_Retry++;

	mLogAbortSession(pMPIdata);
	return LOG_RUN;
}

unsigned char mLogSendRespAck(struct s_MPIdata *pMPIdata)
{
   if (LogOn_RespFrameOk(pMPIdata))
   {
	  //Send Short Ack
	  sendShortAck(pMPIdata);

	  pMPIdata->SessReqPending = true;

	  //Prepare next frame
	  pMPIdata->FrameSessionSendLength = LOGON_SAPRESPACK_LEN;
	  RtlCopyMemory(pMPIdata->FrameSessionSendBuffer, LogOnSAPRespAck, LOGON_SAPRESPACK_LEN);
	  pMPIdata->FrameSessionSendBuffer[4] = pMPIdata->LogOn_DA | 0x80;
	  pMPIdata->FrameSessionSendBuffer[5] = pMPIdata->FrameUniopNodeNum | 0x80;
	  pMPIdata->FrameSessionSendBuffer[7] = pMPIdata->StationStatus[pMPIdata->LogOn_DA].LogOn_DAE;
	  pMPIdata->FrameSessionSendBuffer[8] = pMPIdata->StationStatus[pMPIdata->LogOn_DA].LogOn_SAE;

	  //Calcola FCS
	  CalcCheckSum(pMPIdata->FrameSessionSendBuffer, 4, pMPIdata->FrameSessionSendLength - 2);

	  return LOG_WAITSAPRSPACK;
   }
   return mLogCheckLogOff(pMPIdata);
}

unsigned char mLogWaitAckRespAck(struct s_MPIdata *pMPIdata)
{
	if (LogOn_AckRespFrameOk(pMPIdata))
	{
	  sendShortAck(pMPIdata);

	  //Success!!!
	  pMPIdata->LogSessReqPending = false;
	  pMPIdata->LogOn_Retry = 0;
	  return LOG_RUN;
	}
	return mLogCheckLogOff(pMPIdata);
}

/*----LogOff----------------------------*/
unsigned char mLgfCheckLogOff(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->SlaveSession)   //MG001
	  return LGF_RUN;  //MG001

   if (pMPIdata->FrameSessionReceiveBuffer[9] == 0x80)
   {
	  mExitFromRing(pMPIdata); //MG003
	  return LGF_IDLE; //MG003

   }
   return LGF_RUN;
}

void mLgfSendLogOffAck(struct s_MPIdata *pMPIdata)
{
   pMPIdata->SessReqPending = true;
}

void mLgfAbortSession(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->LgfSessReqPending)
   {
	  pMPIdata->LgfSessReqPending = false;
	  pMPIdata->SessReqPending    = false;
	  pMPIdata->SessionStarted    = false;
	  pMPIdata->StationStatus[pMPIdata->LogOff_DA].Logged = 0;
	  pMPIdata->StationStatus[pMPIdata->LogOff_DA].Job   = 0;
	  pMPIdata->StationStatus[pMPIdata->LogOff_DA].LogOn_DAE = 0;
	  pMPIdata->StationStatus[pMPIdata->LogOff_DA].LogStatus = 0;	//MG001
   }
}

unsigned char mLgfCheckSD1(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->Dest == pMPIdata->FrameUniopNodeNum)
   {
	  pMPIdata->LgfSessReqPending = false;
	  pMPIdata->SessReqPending    = false;
	  pMPIdata->SessionStarted    = false;
	  pMPIdata->StationStatus[pMPIdata->LogOff_DA].Logged = 0;
	  pMPIdata->StationStatus[pMPIdata->LogOff_DA].Job   = 0;
	  pMPIdata->StationStatus[pMPIdata->LogOff_DA].LogOn_DAE = 0;
	  pMPIdata->StationStatus[pMPIdata->LogOff_DA].LogStatus = 0;	//MG001
	  return LGF_RUN;
   }
   return LGF_SENT_LOGOFF_ACK;
}

/*--- JobAction & JobAction0 common functions -------------*/
char JobAckFrameOk(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->FrameSessionReceiveBuffer[9] != 0xB0)
   {
	  pMPIdata->StationStatus[pMPIdata->FrameSessionSendBuffer[4] & 0x7F].Job = 0;
	  return 0;
   }
   return 1;
}

/*----JobAction0----------------------------*/
void mJob0ApplicationResponseErr(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->Job0SessReqPending)
   {
	  pMPIdata->Job0SessReqPending = false;
	  pMPIdata->SessReqPending     = false;
	  pMPIdata->SessionStarted     = false;
	  pMPIdata->LogSessReqPending  = false;
	  pMPIdata->ProcedureApplMStatus = GEN_COMM_ERR;
	  DBGReg(239, pMPIdata->ProcedureApplMStatus, 0x0002);
	  pMPIdata->ProcedureApplMRequestPending  = false;
	  pMPIdata->ProcedureApplMResponsePending = false;
	  pMPIdata->StationStatus[pMPIdata->Last_DA].LogStatus = 1;
   }
}

void mJob0SendReq(struct s_MPIdata *pMPIdata)
{
   RtlCopyMemory(pMPIdata->FrameSessionSendBuffer, JobActionReq0, JOBACTION_REQ_LEN0);
   pMPIdata->FrameSessionSendBuffer[4]  = pMPIdata->Last_DA | 0x80;
   pMPIdata->FrameSessionSendBuffer[5]  = pMPIdata->FrameUniopNodeNum | 0x80;
   pMPIdata->FrameSessionSendBuffer[7]  = pMPIdata->StationStatus[pMPIdata->Last_DA].LogOn_DAE;
   pMPIdata->FrameSessionSendBuffer[8]  = pMPIdata->StationStatus[pMPIdata->Last_DA].LogOn_SAE;
   pMPIdata->FrameSessionSendBuffer[10] = 0; //Job = 0
   pMPIdata->FrameSessionSendLength = JOBACTION_REQ_LEN0;

   //Calcola FCS
   CalcCheckSum(pMPIdata->FrameSessionSendBuffer, 4, pMPIdata->FrameSessionSendLength - 2);

   pMPIdata->Job0SessReqPending = true;
   pMPIdata->SessReqPending = true;
}

unsigned char mJob0SendShortAck(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->FrameSessionReceiveBuffer[9] == 0xB0)
   {
	  sendShortAck(pMPIdata);
	  return JB0_WAITJOBRESP0;
   }

   pMPIdata->StationStatus[pMPIdata->FrameSessionSendBuffer[4] & 0x7F].Job = 0;
   mJob0ApplicationResponseErr(pMPIdata);
   return JB0_RUN;
}

unsigned char mJob0SendJobAck(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->FrameSessionReceiveBuffer[9] == 0xF1)
   {
	  sendShortAck(pMPIdata);

	  pMPIdata->FrameSessionSendBuffer[0]  = D_SD2;
	  pMPIdata->FrameSessionSendBuffer[1]  = 0x08;
	  pMPIdata->FrameSessionSendBuffer[2]  = 0x08;
	  pMPIdata->FrameSessionSendBuffer[3]  = D_SD2;
	  pMPIdata->FrameSessionSendBuffer[4]  = pMPIdata->Last_DA | 0x80;
	  pMPIdata->FrameSessionSendBuffer[5]  = pMPIdata->FrameUniopNodeNum | 0x80;
	  pMPIdata->FrameSessionSendBuffer[6]  = 0x5C;
	  pMPIdata->FrameSessionSendBuffer[7]  = pMPIdata->StationStatus[pMPIdata->Last_DA].LogOn_DAE;
	  pMPIdata->FrameSessionSendBuffer[8]  = pMPIdata->StationStatus[pMPIdata->Last_DA].LogOn_SAE;
	  pMPIdata->FrameSessionSendBuffer[9]  = 0xB0;
	  pMPIdata->FrameSessionSendBuffer[10] = 0x01;
	  pMPIdata->FrameSessionSendBuffer[11] = 0;
	  pMPIdata->FrameSessionSendBuffer[13] = D_ED;
	  pMPIdata->FrameSessionSendLength = JOBACTION_ACK_LEN;

	  //Calcola FCS
	  CalcCheckSum(pMPIdata->FrameSessionSendBuffer, 4, pMPIdata->FrameSessionSendLength - 2);

	  pMPIdata->SessReqPending = true;
	  return JB0_WAITSHORTACK02;
   }

   mJob0ApplicationResponseErr(pMPIdata);
   return JB0_RUN;
}
void mJob0Ok(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->StationStatus[pMPIdata->Last_DA].Job == 0xFF) pMPIdata->StationStatus[pMPIdata->Last_DA].Job = 1;
   else pMPIdata->StationStatus[pMPIdata->Last_DA].Job++;

   pMPIdata->Job0SessReqPending = false;
}
/*----JobAction----------------------------*/
void mJobApplicationResponse(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->JobSessReqPending)
   {
	  pMPIdata->JobSessReqPending = false;
	  pMPIdata->SessReqPending    = false;
	  pMPIdata->SessionStarted    = false;
	  if (pMPIdata->MyFrameResponseBuff[14] != 0xFF)
	  {
		 pMPIdata->ProcedureApplMStatus = RESPONSE_NAK2;
		 DBGReg(239, pMPIdata->ProcedureApplMStatus, 0x0003);
	  }
	  else
		 pMPIdata->ProcedureApplMStatus = NO_ERROR;
	  pMPIdata->ProcedureApplMRequestPending  = false;
	  pMPIdata->ProcedureApplMResponsePending = false;
   }
}

void mJobApplicationResponseErr(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->JobSessReqPending)
   {
	  pMPIdata->Job0SessReqPending = false;
	  pMPIdata->SessReqPending     = false;
	  pMPIdata->SessionStarted     = false;
	  pMPIdata->LogSessReqPending  = false;
	  pMPIdata->ProcedureApplMStatus = GEN_COMM_ERR;
	  DBGReg(239, pMPIdata->ProcedureApplMStatus, 0x0004);
	  pMPIdata->ProcedureApplMRequestPending  = false;
	  pMPIdata->ProcedureApplMResponsePending = false;
   }
}

void mJobSendReq(struct s_MPIdata *pMPIdata)
{
   pMPIdata->FrameJobSessionBuff[7]  = pMPIdata->StationStatus[pMPIdata->Last_DA].LogOn_DAE;
   pMPIdata->FrameJobSessionBuff[8]  = pMPIdata->StationStatus[pMPIdata->Last_DA].LogOn_SAE;
   pMPIdata->FrameJobSessionBuff[10] = pMPIdata->StationStatus[pMPIdata->Last_DA].Job;

   //Calcola FCS
   CalcCheckSum(pMPIdata->FrameJobSessionBuff, 4, pMPIdata->FrameJobSessionLen - 2);
   pMPIdata->FrameJobSessionBuff[pMPIdata->FrameJobSessionLen-1] = D_ED;

   RtlCopyMemory(pMPIdata->FrameSessionSendBuffer, pMPIdata->FrameJobSessionBuff, pMPIdata->FrameJobSessionLen);   //MG001
   pMPIdata->FrameSessionSendLength =pMPIdata-> FrameJobSessionLen;                               //MG001

   pMPIdata->JobSessReqPending = true;
   pMPIdata->SessReqPending    = true;
   pMPIdata->JobRetry = 0;
}

unsigned char  mJobRetryReq(struct s_MPIdata *pMPIdata)
{
   if (++pMPIdata->JobRetry > NR_JOB_RETRY)
   {
	  mJobApplicationResponseErr(pMPIdata);
	  return JOB_RUN;
   }
   else
   {
	  RtlCopyMemory(pMPIdata->FrameSessionSendBuffer, pMPIdata->FrameJobSessionBuff, pMPIdata->FrameJobSessionLen);   //MG001
	  pMPIdata->FrameSessionSendLength =pMPIdata->FrameJobSessionLen;                               //MG001
	  pMPIdata->JobSessReqPending = true;
	  pMPIdata->SessReqPending    = true;
	  return JOB_WAITSHORTACK;
   }
}


unsigned char mJobSendShortAck(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->FrameSessionReceiveBuffer[9] == 0x80)  //MG001
	  return JOB_WAITJOBACK;                  //MG001

   if (pMPIdata->FrameSessionReceiveBuffer[9] == 0xB0)
   {
	  sendShortAck(pMPIdata);
	  return JOB_WAITJOBRESP;
   }
   DBGReg(241, pMPIdata->FrameSessionReceiveBuffer[9], 0);
   return JOB_WAITJOBACK;                  //MG001
}

unsigned char mJobSendJobAck(struct s_MPIdata *pMPIdata)
{
	if (pMPIdata->FrameSessionReceiveBuffer[9] != 0xB0 && pMPIdata->FrameSessionReceiveBuffer[9] != 0xF1)
		printk("mJobSendJobAck: not ACK sent %02X \n", pMPIdata->FrameSessionReceiveBuffer[9]);

   if (pMPIdata->FrameSessionReceiveBuffer[9] == 0x80)  //MG001
	  return JOB_WAITJOBRESP;                 //MG001

   if (pMPIdata->FrameSessionReceiveBuffer[9] == 0xB0)
   {
	  sendShortAck(pMPIdata);
	  DBGReg(240, pMPIdata->FrameSessionReceiveBuffer[9], 0);
	  return JOB_WAITJOBRESP;
   }
   if (pMPIdata->FrameSessionReceiveBuffer[9] == 0xF1)
   {
	  pMPIdata->MyFrameResponseBuffLen = pMPIdata->FrameSessionReceiveBuffer[1]-7;
	  RtlCopyMemory(pMPIdata->MyFrameResponseBuff, &pMPIdata->FrameSessionReceiveBuffer[11], pMPIdata->MyFrameResponseBuffLen);

	  sendShortAck(pMPIdata);

	  pMPIdata->FrameSessionSendBuffer[0]  = D_SD2;
	  pMPIdata->FrameSessionSendBuffer[1]  = 0x08;
	  pMPIdata->FrameSessionSendBuffer[2]  = 0x08;
	  pMPIdata->FrameSessionSendBuffer[3]  = D_SD2;
	  pMPIdata->FrameSessionSendBuffer[4]  = pMPIdata->Last_DA | 0x80;
	  pMPIdata->FrameSessionSendBuffer[5]  = pMPIdata->FrameUniopNodeNum | 0x80;
	  pMPIdata->FrameSessionSendBuffer[6]  = 0x5C;
	  pMPIdata->FrameSessionSendBuffer[7]  = pMPIdata->StationStatus[pMPIdata->Last_DA].LogOn_DAE;
	  pMPIdata->FrameSessionSendBuffer[8]  = pMPIdata->StationStatus[pMPIdata->Last_DA].LogOn_SAE;
	  pMPIdata->FrameSessionSendBuffer[9]  = 0xB0;
	  pMPIdata->FrameSessionSendBuffer[10] = 0x01;
	  pMPIdata->FrameSessionSendBuffer[11] = pMPIdata->StationStatus[pMPIdata->Last_DA].Job;
	  pMPIdata->FrameSessionSendBuffer[13] = D_ED;
	  pMPIdata->FrameSessionSendLength = JOBACTION_ACK_LEN;

	  //Calcola FCS
	  CalcCheckSum(pMPIdata->FrameSessionSendBuffer, 4, pMPIdata->FrameSessionSendLength - 2);

	  if (pMPIdata->StationStatus[pMPIdata->Last_DA].Job == 0xFF) pMPIdata->StationStatus[pMPIdata->Last_DA].Job = 1;
	  else pMPIdata->StationStatus[pMPIdata->Last_DA].Job++;

	  pMPIdata->SessReqPending = true;
	  pMPIdata->JobRetry = 0;
	  return JOB_WAITSHORTACK2;
   }

   DBGReg(240, pMPIdata->FrameSessionReceiveBuffer[9], 0);

   return JOB_WAITJOBRESP;                 //MG001
}

unsigned char mJobRetryJobAck(struct s_MPIdata *pMPIdata)
{
   if (++pMPIdata->JobRetry > NR_JOB_RETRY)
   {
	  mJobApplicationResponseErr(pMPIdata);
	  return JOB_RUN;
   }
   else
   {
	  //retransmit previous Job number
	  if (pMPIdata->StationStatus[pMPIdata->Last_DA].Job == 1)   pMPIdata->StationStatus[pMPIdata->Last_DA].Job = 0xFF;
	  else                                   pMPIdata->StationStatus[pMPIdata->Last_DA].Job--;

	  pMPIdata->FrameSessionSendBuffer[0]  = D_SD2;
	  pMPIdata->FrameSessionSendBuffer[1]  = 0x08;
	  pMPIdata->FrameSessionSendBuffer[2]  = 0x08;
	  pMPIdata->FrameSessionSendBuffer[3]  = D_SD2;
	  pMPIdata->FrameSessionSendBuffer[4]  = pMPIdata->Last_DA | 0x80;
	  pMPIdata->FrameSessionSendBuffer[5]  = pMPIdata->FrameUniopNodeNum | 0x80;
	  pMPIdata->FrameSessionSendBuffer[6]  = 0x5C;
	  pMPIdata->FrameSessionSendBuffer[7]  = pMPIdata->StationStatus[pMPIdata->Last_DA].LogOn_DAE;
	  pMPIdata->FrameSessionSendBuffer[8]  = pMPIdata->StationStatus[pMPIdata->Last_DA].LogOn_SAE;
	  pMPIdata->FrameSessionSendBuffer[9]  = 0xB0;
	  pMPIdata->FrameSessionSendBuffer[10] = 0x01;
	  pMPIdata->FrameSessionSendBuffer[11] = pMPIdata->StationStatus[pMPIdata->Last_DA].Job;
	  pMPIdata->FrameSessionSendBuffer[13] = D_ED;
	  pMPIdata->FrameSessionSendLength = JOBACTION_ACK_LEN;

	  //Calcola FCS
	  CalcCheckSum(pMPIdata->FrameSessionSendBuffer, 4, pMPIdata->FrameSessionSendLength - 2);

	  if (pMPIdata->StationStatus[pMPIdata->Last_DA].Job == 0xFF) pMPIdata->StationStatus[pMPIdata->Last_DA].Job = 1;
	  else pMPIdata->StationStatus[pMPIdata->Last_DA].Job++;

	  pMPIdata->SessReqPending = true;
	  return JOB_WAITSHORTACK2;
   }
}

void mJobStopSess(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->JobSessReqPending)
   {
	  pMPIdata->JobSessReqPending = false;
   }
}
#endif

/*----Token--------------------------------*/

//MG002 Start
void mTokFSMRestart(struct s_MPIdata *pMPIdata)
{
   #if MPI_MASTER_STACK
	  ev_post(pMPIdata, EV_LOG_RUN);
	  ev_post(pMPIdata, EV_JB0_RUN);
	  ev_post(pMPIdata, EV_JOB_RUN);
   #endif
   #if MPI_SLAVE_STACK
	  ev_post(pMPIdata, EV_LGS_RUN);
   #endif
   ev_post(pMPIdata, EV_LGF_RUN);
}
//MG002 End
void mTokListenTokenStartTimeout(struct s_MPIdata *pMPIdata)
{
   UltiUART1_StartTimer(pMPIdata, EV_TOK_TIMEOUT, (word)pMPIdata->FrameUniopNodeNum * pMPIdata->guardTimeFactor + pMPIdata->guardTimeConstant); //guard timeout
}

unsigned char mTokListenToken(struct s_MPIdata *pMPIdata)
{
   // Since SA is passing the token it must be an active master
   pMPIdata->StationStatus[pMPIdata->Source].StationType = 3; // Master in token ring

   // When we have listened to 2 complete token rotations
   // then we are ready to join the token ring
   if (2 <= pMPIdata->NumTokenRotations)
   {
	  mTokFSMRestart(pMPIdata); //MG002
	  return TOK_ACTIVEIDLE;
   }

   if (pMPIdata->LowestAddress > pMPIdata->Source)
   {
	  pMPIdata->LowestAddress = pMPIdata->Source;
	  pMPIdata->NumTokenRotations = 0;
   }

   if (pMPIdata->LowestAddress == pMPIdata->Dest)
	  pMPIdata->NumTokenRotations++;

   return TOK_LISTENTOKEN;
}

void mTokActiveIdle(struct s_MPIdata *pMPIdata)
{
   // Since SA is passing the token it must be an active master
   pMPIdata->StationStatus[pMPIdata->Source].StationType = 2; // Master Master ready to enter token ring (Ready FC=0x20)
   if (pMPIdata->FrameUniopNodeNum == pMPIdata->Dest)
   {
	   dumpDebug("mTokActiveIdle");
	  // We have been addressed with an FDL status request
	  // If we are already in the token ring then tell the
	  // requester so, otherwise tell them that we are ready
	  // to join the token ring!
	  pMPIdata->NextGAP = pMPIdata->FrameUniopNodeNum;
	  if (pMPIdata->NextStation == (signed char)-1)
		 pMPIdata->NextStation = pMPIdata->Source;

	  pMPIdata->FrameSendBuffer[0] = D_SD1;
	  pMPIdata->FrameSendBuffer[1] = pMPIdata->Source;
	  pMPIdata->FrameSendBuffer[2] = pMPIdata->FrameUniopNodeNum;
	  pMPIdata->FrameSendBuffer[3] = (pMPIdata->AreWeInRing)? FCST_MSIR | RFC_OK : FCST_MSRD | RFC_OK;
	  //Calcola FCS
	  CalcCheckSum(pMPIdata->FrameSendBuffer, 1, 4);
	  pMPIdata->FrameSendBuffer[5] = D_ED;
	  pMPIdata->FrameSendBufferLen = 6;
	  sendData(pMPIdata, pMPIdata->FrameSendBuffer, (byte)pMPIdata->FrameSendBufferLen);

	  pMPIdata->FlagSendSelfToken = 0;
	  mTokFSMRestart(pMPIdata);  //MG002
	  return;
   }
}

unsigned char GotoActiveIdle(struct s_MPIdata *pMPIdata)
{
   pMPIdata->SessReqPending     = false;
   pMPIdata->SessionStarted     = false;
   pMPIdata->LgsSessReqPending  = false;
   pMPIdata->Job0SessReqPending = false;
   pMPIdata->JobSessReqPending  = false;

#ifdef FATAL_ERRORS
   pMPIdata->ProcedureApplMStatus = GEN_COMM_ERR;
   DBGReg(239, pMPIdata->ProcedureApplMStatus, 0x0005);
   pMPIdata->ProcedureApplMRequestPending  = false;
   pMPIdata->ProcedureApplMResponsePending = false;
#else
   if (pMPIdata->ProcedureApplMResponsePending)
	  pMPIdata->ProcedureApplMRequestPending = true;
#endif

   ResetStations(pMPIdata);

//MG001 start
   //reset all other state machines
   mLgfAbortSession(pMPIdata);
   pMPIdata->lgf_state = LGF_RUN;
   mLogAbortSession(pMPIdata);
   pMPIdata->log_state = LOG_RUN;
   mJob0ApplicationResponseErr(pMPIdata);
   pMPIdata->jb0_state = JB0_RUN;
   mJobApplicationResponseErr(pMPIdata);
   pMPIdata->job_state = JOB_RUN;
#if MPI_SLAVE_STACK
   mLgsAbortSession(pMPIdata);
   pMPIdata->lgs_state = LGS_RUN;
   mJ0sAbortSession(pMPIdata);
   pMPIdata->j0s_state = J0S_IDLE;
   mJbsAbortSession(pMPIdata);
   pMPIdata->jbs_state = JBS_IDLE;
#endif
   DBGReg(243, pMPIdata->tok_state, TOK_IDLE);
   UltiUART1_StartTimer(pMPIdata, EV_TOK_RUN,100*1000);
   return TOK_IDLE;
}
void StopTimeoutFDL(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->UltiUart1_TxTimeout == EV_TOK_FDLTIMEOUT)
   {
	  UltiUart1_StopTimer(pMPIdata);
   }
}

void mTokSD1Resp(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->FrameUniopNodeNum == pMPIdata->Dest)
   {
	  StopTimeoutFDL(pMPIdata);       //MG001
	  // Since SA is passing the token it must be an active master
	  pMPIdata->NextGAP = pMPIdata->FrameUniopNodeNum;
	  pMPIdata->StationStatus[pMPIdata->Source].StationType = 2; // Master in token ring
	  pMPIdata->FlagSendSelfToken = 0;
	  pMPIdata->PassTokenReply++;
	  pMPIdata->NextStation = pMPIdata->Source;

	  // Pass the token onto the next active master
	  pMPIdata->FrameSendBuffer[0] = D_TOK;
	  pMPIdata->FrameSendBuffer[1] = pMPIdata->Source;
	  pMPIdata->FrameSendBuffer[2] = pMPIdata->FrameUniopNodeNum;
	  pMPIdata->FrameSendBufferLen = 3;
	  sendData(pMPIdata, pMPIdata->FrameSendBuffer, (byte)pMPIdata->FrameSendBufferLen);

	  return;
   }
}

byte FDLStatus(struct s_MPIdata *pMPIdata, signed char *MyNextGap)
{
   if ((*MyNextGap = PrepareNextGAP(pMPIdata)) != (signed char)-1)
   {
	  dumpDebug("FDLStatus");
	  pMPIdata->FrameSendBuffer[0] = D_SD1;
	  pMPIdata->FrameSendBuffer[1] = *MyNextGap;
	  pMPIdata->FrameSendBuffer[2] = pMPIdata->FrameUniopNodeNum;
	  pMPIdata->FrameSendBuffer[3] = FCFC_RFDL | FCMSK_FT;

	  //Calcola FCS
	  CalcCheckSum(pMPIdata->FrameSendBuffer, 1, 4);
	  pMPIdata->FrameSendBuffer[5] = D_ED;
	  pMPIdata->FrameSendBufferLen = 6;
	  sendData(pMPIdata, pMPIdata->FrameSendBuffer, (byte)pMPIdata->FrameSendBufferLen);

	  pMPIdata->PassTokenReply = 0;
	  return 1;
   }
   return 0;
}

byte PassToken(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->PassTokenReply < PASS_TOKEN_REPLY)
   {
	  // If the NextStation hasn't been calculated yet do it now
	  if (pMPIdata->NextStation == (signed char)-1)
		 pMPIdata->NextStation = pMPIdata->FrameUniopNodeNum;
	  else
		 pMPIdata->NextStation = GetNextActiveStation(pMPIdata);

	  // Pass the token onto the next active master
	  pMPIdata->FrameSendBuffer[0] = D_TOK;
	  pMPIdata->FrameSendBuffer[1] = pMPIdata->NextStation;
	  pMPIdata->FrameSendBuffer[2] = pMPIdata->FrameUniopNodeNum;
	  pMPIdata->FrameSendBufferLen = 3;
	  sendData(pMPIdata, pMPIdata->FrameSendBuffer, (byte)pMPIdata->FrameSendBufferLen);

	  pMPIdata->PassTokenReply++;
	  return 1;
   }

   //Reset station Status
   pMPIdata->StationStatus[(int)pMPIdata->NextStation].IsActive = 0;
   pMPIdata->StationStatus[(int)pMPIdata->NextStation].StationType = 0;
   pMPIdata->PassTokenReply = 0;

   // Try with the next station if exists
   pMPIdata->NextStation = GetNextActiveStation(pMPIdata);
   if ((pMPIdata->NextStation == (signed char)-1) || (pMPIdata->NextStation == pMPIdata->FrameUniopNodeNum)) return 0;

   // Pass the token onto the next active master
   pMPIdata->FrameSendBuffer[0] = D_TOK;
   pMPIdata->FrameSendBuffer[1] = pMPIdata->NextStation;
   pMPIdata->FrameSendBuffer[2] = pMPIdata->FrameUniopNodeNum;
   pMPIdata->FrameSendBufferLen = 3;
   sendData(pMPIdata, pMPIdata->FrameSendBuffer, (byte)pMPIdata->FrameSendBufferLen);

   pMPIdata->PassTokenReply++;
   return 1;
}

unsigned char mTokTxFrame(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->FrameUniopNodeNum != pMPIdata->Dest)
   {
	  if (pMPIdata->Dest == pMPIdata->Source)  //it is a self-token, the ring is broken
	  {
		 mExitFromRing(pMPIdata);
		 return TOK_IDLE;
	  }
	  //Too token not for me
	  if (pMPIdata->TokenNotForMe++ > 60)
	  {
		 pMPIdata->TokenNotForMe = 0;
		 return GotoActiveIdle(pMPIdata);
	  }
	  mTokListenTokenStartTimeout(pMPIdata);
	  return TOK_WAITRX;
   }

   //disable any active timeout
   UltiUart1_StopTimer(pMPIdata);

   pMPIdata->PassTokenReply = 0;
   pMPIdata->AreWeInRing = true;
   pMPIdata->TokenNotForMe = 0;
   if (pMPIdata->GapUpdateFactorCnt <= pMPIdata->GapUpdateFactor)   //MG001
	  pMPIdata->GapUpdateFactorCnt++;                     //MG001

   if (pMPIdata->SessReqPending && !pMPIdata->SessionStarted)//Session Request Pending
   {
	  if (pMPIdata->LgfSessReqPending || pMPIdata->LgsSessReqPending || pMPIdata->J0sSessReqPending || pMPIdata->JbsSessReqPending)
	  {
		 pMPIdata->FrameSessionSendLength = pMPIdata->FrameSendBufferLen1;
		 sendData(pMPIdata, pMPIdata->FrameSendBuffer1, (byte)pMPIdata->FrameSessionSendLength);
	  }
	  else
		 sendData(pMPIdata, pMPIdata->FrameSessionSendBuffer, (byte)pMPIdata->FrameSessionSendLength);

	  pMPIdata->SessionStarted = true;
	  return TOK_WAITRX;
   }

   if (pMPIdata->GapUpdateFactorCnt > pMPIdata->GapUpdateFactor)  //MG001
   {
	  if (FDLStatus(pMPIdata, &pMPIdata->NxtStat))
	  {
		 if (pMPIdata->NxtStat == pMPIdata->NextStation - 1)
			pMPIdata->GapUpdateFactorCnt = 0;
		 return TOK_WAITRX;// Send an FDL Status request to the next GAP
	  }
   }
   if (PassToken(pMPIdata))
	   return TOK_WAITRX;// Pass Token
   return GotoActiveIdle(pMPIdata);
}
//MG001 end


void mTokStartTimeoutTok(struct s_MPIdata *pMPIdata)
{
   UltiUART1_StartTimer(pMPIdata, EV_TOK_TIMEOUT_TOK, pMPIdata->tokTimeout); //guard timeout
 //  dumpInteger("TOKEN TIMEOUT", tokTimeout);
}

void StopTimeoutTok(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->UltiUart1_TxTimeout == EV_TOK_TIMEOUT_TOK)
   {
	  pMPIdata->PassTokenReply = 0;
	  UltiUart1_StopTimer(pMPIdata);
   }
}
void mTokStartFDLTimeout(struct s_MPIdata *pMPIdata)
{
   #if EVENTLOG
	  UltiUART1_StartTimer(pMPIdata, EV_TOK_FDLTIMEOUT, 25*11);
   #else
	  UltiUART1_StartTimer(pMPIdata, EV_TOK_FDLTIMEOUT, pMPIdata->FDLTimeout);
   #endif
   pMPIdata->PassTokenReply = 0;
}
//MG001 start
void mTokStopTimeoutTok(struct s_MPIdata *pMPIdata)
{
   StopTimeoutTok(pMPIdata);
}
//MG001 end

void StopTimeoutAck(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->UltiUart1_TxTimeout == EV_BRC_ACK_TIMEOUT)
   {
	  UltiUart1_StopTimer(pMPIdata);
   }
}

void mTokStartTimeout10(struct s_MPIdata *pMPIdata)
{
   UltiUART1_StartTimer(pMPIdata, EV_TOK_TIMEOUT, (word)pMPIdata->FrameUniopNodeNum * pMPIdata->guardTimeFactor + pMPIdata->guardTimeConstant); //guard timeout
   pMPIdata->PassTokenReply = 0;
}

void mTokStartSelfTokTimeout(struct s_MPIdata *pMPIdata)
{
   UltiUART1_StartTimer(pMPIdata, EV_TOK_SELFTOK_TIMEOUT, pMPIdata->selfTokTimeout);  //timeout di funzionamento  //MG001
   pMPIdata->PassTokenReply = 0;
}


unsigned char mTokCheckReply(struct s_MPIdata *pMPIdata)
{
   //Sessione Slave - Su EOTX di ACK
   if (pMPIdata->ReadyForConfigMode == 2)
   {
	  pMPIdata->ReadyForConfigMode = 3;
	  return TOK_IDLE;
   }

   if (pMPIdata->LgfSessReqPending || pMPIdata->LgsSessReqPending || pMPIdata->J0sSessReqPending || pMPIdata->JbsSessReqPending) {
	  if (pMPIdata->SessReqPending && (pMPIdata->SessionStarted == 0)) {
		 pMPIdata->CurrentShortAck = SKIP;
		 if (PassToken(pMPIdata))
			return TOK_WAITRX;// Pass Token
		 return GotoActiveIdle(pMPIdata);
	  }
   }

   //Session Master
   if (pMPIdata->SessionStarted)
   {
	  pMPIdata->SessReqPending = false;
	  pMPIdata->SessionStarted = false;
	  UltiUART1_StartTimer(pMPIdata, EV_BRC_ACK_TIMEOUT, pMPIdata->ackGuardTime); //timeout di guardia
	  return TOK_WAITSESSRX;
   }

   if (pMPIdata->Sd1RespGuard)
   {
	  pMPIdata->Sd1RespGuard = 0;
	  return TOK_WAITRX;
   }

   //it's a pass token
   if (pMPIdata->PassTokenReply > 0)
   {
	  mTokStartTimeoutTok(pMPIdata);
	  return TOK_WAITRX;
   }
   if (pMPIdata->shortACK_EOTEvent)
   {
	   pMPIdata->shortACK_EOTEvent = 0;
	   mTokStartTimeoutTok(pMPIdata);
	   return TOK_WAITRX;
   }
   mTokStartFDLTimeout(pMPIdata);
   return TOK_WAITFDLSTATUS;
}

void mTokReloadTimer(struct s_MPIdata *pMPIdata)
{
}

unsigned char mTokWaitSessRx(struct s_MPIdata *pMPIdata, int stat)
{
   StopTimeoutAck(pMPIdata);
   if (stat) 
   {
#ifdef RETRY_IN_SAME_TOKEN
	  if (++pMPIdata->JobRetry < NR_JOB_RETRY)
      {
         if (pMPIdata->job_state == JOB_WAITSHORTACK)
         {
           RtlCopyMemory(pMPIdata->FrameSessionSendBuffer, pMPIdata->FrameJobSessionBuff, pMPIdata->FrameJobSessionLen);   //MG001
           pMPIdata->FrameSessionSendLength =pMPIdata->FrameJobSessionLen;                               //MG001
           pMPIdata->JobSessReqPending = true;
           pMPIdata->SessReqPending    = true;
         }
         else
         {
           //retransmit previous Job number
           if (pMPIdata->StationStatus[pMPIdata->Last_DA].Job == 1)   
              pMPIdata->StationStatus[pMPIdata->Last_DA].Job = 0xFF;
           else
              pMPIdata->StationStatus[pMPIdata->Last_DA].Job--;
           pMPIdata->FrameSessionSendBuffer[0]  = D_SD2;
           pMPIdata->FrameSessionSendBuffer[1]  = 0x08;
           pMPIdata->FrameSessionSendBuffer[2]  = 0x08;
           pMPIdata->FrameSessionSendBuffer[3]  = D_SD2;
           pMPIdata->FrameSessionSendBuffer[4]  = pMPIdata->Last_DA | 0x80;
           pMPIdata->FrameSessionSendBuffer[5]  = pMPIdata->FrameUniopNodeNum | 0x80;
           pMPIdata->FrameSessionSendBuffer[6]  = 0x5C;
           pMPIdata->FrameSessionSendBuffer[7]  = pMPIdata->StationStatus[pMPIdata->Last_DA].LogOn_DAE;
           pMPIdata->FrameSessionSendBuffer[8]  = pMPIdata->StationStatus[pMPIdata->Last_DA].LogOn_SAE;
           pMPIdata->FrameSessionSendBuffer[9]  = 0xB0;
           pMPIdata->FrameSessionSendBuffer[10] = 0x01;
           pMPIdata->FrameSessionSendBuffer[11] = pMPIdata->StationStatus[pMPIdata->Last_DA].Job;
           pMPIdata->FrameSessionSendBuffer[13] = D_ED;
           pMPIdata->FrameSessionSendLength = JOBACTION_ACK_LEN;
           //Calcola FCS
           CalcCheckSum(pMPIdata->FrameSessionSendBuffer, 4, pMPIdata->FrameSessionSendLength - 2);
           if (pMPIdata->StationStatus[pMPIdata->Last_DA].Job == 0xFF) pMPIdata->StationStatus[pMPIdata->Last_DA].Job = 1;
           else pMPIdata->StationStatus[pMPIdata->Last_DA].Job++;
           pMPIdata->SessReqPending = true;            
         }
         //resend
         return mTokTxFrame(pMPIdata);
      }
	  else {
		  ResetLastDAStation(pMPIdata);
		  ev_post(pMPIdata, EV_SES_ERR);
	  }
#else
	  ev_post(pMPIdata, EV_SES_RETRY);
#endif
   }
   else
	  ev_post(pMPIdata, EV_SES_ACK);

   //Pass Token
   {
	  if (PassToken(pMPIdata))
	  {
		 pMPIdata->FlagPassToken = 1;
		 return TOK_WAITSESSRX;
	  }
   }
   pMPIdata->FlagPassToken = 0;
   return GotoActiveIdle(pMPIdata);
}

#ifdef MANAGE_RR_ANSWERS
unsigned char mTokAnswerRR(struct s_MPIdata *pMPIdata)
{
   StopTimeoutAck(pMPIdata);
   ev_post(pMPIdata, EV_SES_RETRY);

   //Pass Token
   {
	  if (PassToken(pMPIdata))
	  {
		 pMPIdata->FlagPassToken = 1;
		 return TOK_WAITSESSRX;
	  }
   }
   pMPIdata->FlagPassToken = 0;
   return GotoActiveIdle(pMPIdata);
}
#endif

unsigned char mTokWaitSessRxSD1(struct s_MPIdata *pMPIdata)
{
   StopTimeoutAck(pMPIdata);

   if (pMPIdata->FrameSessionReceiveBuffer[3] == (FCST_MSIR | RFC_TTNAK) ||  /* Master in for ring + NAK no resource */
	   pMPIdata->FrameSessionReceiveBuffer[3] == (FCST_MSIR | RFC_RS_NAK))   /* Master in for ring + NAK no service activated */
   {
	   mExitFromRing(pMPIdata);
	   return TOK_IDLE;
   }

   ev_post(pMPIdata, EV_SES_ERR);
   //Pass Token
   if (pMPIdata->FrameUniopNodeNum == pMPIdata->Dest)
   {
	  if (PassToken(pMPIdata))
	  {
		 pMPIdata->FlagPassToken = 1;
		 return TOK_WAITSESSRX;
	  }
   }
   pMPIdata->FlagPassToken = 0;
   return GotoActiveIdle(pMPIdata);
}
//MG001 end

unsigned char mTokCheckGuard(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->FlagPassToken)
   {
	 pMPIdata->FlagPassToken = 0;
	 mTokStartTimeoutTok(pMPIdata);
	 return TOK_WAITRX;
   }
   return TOK_WAITSESSRX;
}

unsigned char mTokPassToken1(struct s_MPIdata *pMPIdata)
{
   if (PassToken(pMPIdata)) return TOK_TOKEN_RETRY;
   return GotoActiveIdle(pMPIdata);
}

unsigned char mTokPassToken(struct s_MPIdata *pMPIdata)
{
   if (PassToken(pMPIdata)) return TOK_WAITFDLSTATUS;
   return GotoActiveIdle(pMPIdata);
}

unsigned char mTokCheckFDLStatusResp(struct s_MPIdata *pMPIdata)
{
   if ((pMPIdata->Source == pMPIdata->NextGAP) && (pMPIdata->FrameUniopNodeNum == pMPIdata->Dest))
   {
	  // Copy the station type from the StationType bits in the FC
	  // to our internal variable
	  pMPIdata->StationStatus[pMPIdata->Source].StationType =
		 (pMPIdata->FrameSessionReceiveBuffer[3] & FCMSK_ST) >> 4;

	  pMPIdata->NextStation = pMPIdata->Source;
	  pMPIdata->StationStatus[pMPIdata->Source].StationType = 3;

	  // Pass the token onto the next active master
	  pMPIdata->FrameSendBuffer[0] = D_TOK;
	  pMPIdata->FrameSendBuffer[1] = pMPIdata->NextStation;
	  pMPIdata->FrameSendBuffer[2] = pMPIdata->FrameUniopNodeNum;
	  pMPIdata->FrameSendBufferLen = 3;
	  sendData(pMPIdata, pMPIdata->FrameSendBuffer, (byte)pMPIdata->FrameSendBufferLen);
	  pMPIdata->NextGAP = pMPIdata->FrameUniopNodeNum;

	  pMPIdata->FlagSendSelfToken = 0;
	  pMPIdata->PassTokenReply++;

	  return TOK_WAITRX;
   }
   return GotoActiveIdle(pMPIdata);
}

void mTokSendSelfToken(struct s_MPIdata *pMPIdata)
{
   // Pass the token to ourselves
   pMPIdata->FrameSendBuffer[0] = D_TOK;
   pMPIdata->FrameSendBuffer[1] = pMPIdata->FrameUniopNodeNum;
   pMPIdata->FrameSendBuffer[2] = pMPIdata->FrameUniopNodeNum;
   pMPIdata->FrameSendBufferLen = 3;
   sendData(pMPIdata, pMPIdata->FrameSendBuffer, (byte)pMPIdata->FrameSendBufferLen);

   pMPIdata->FlagSendSelfToken = 1;
   pMPIdata->NextStation = - 1;
}

unsigned char mTokSendFDLStatus(struct s_MPIdata *pMPIdata)
{
	signed char nxtgap;

   if (pMPIdata->cnt++ == 255)//to recovery error
   {
	  ResetStations(pMPIdata);
	  pMPIdata->cnt = 0;
   }
   // Send an FDL Status request to the next GAP
   if (FDLStatus(pMPIdata, &nxtgap))
		 return TOK_SELFTOKEN;
   return GotoActiveIdle(pMPIdata);
}

unsigned char mTokCheckResp(struct s_MPIdata *pMPIdata)
{
   if (pMPIdata->FlagSendSelfToken)
   {
	  pMPIdata->FlagSendSelfToken = 0;
	  mTokStartSelfTokTimeout(pMPIdata);
	  return TOK_SELFTOKEN;
   }
   //Return from self token
   mTokFSMRestart(pMPIdata);         //MG002
   mTokStartTimeoutTok(pMPIdata);
   return TOK_WAITRX;
}

void mTokSelfToken(struct s_MPIdata *pMPIdata)
{
   ResetStations(pMPIdata);
   mTokStartFDLTimeout(pMPIdata);  //MG001
}

void mTokSendFDLStatusRsp(struct s_MPIdata *pMPIdata)
{
	if (pMPIdata->FrameUniopNodeNum == pMPIdata->Dest)
	{
		if (pMPIdata->cnt++ > 5)//to avoid dead lock
		{
			pMPIdata->cnt = 0;
			mTokStartTimeout10(pMPIdata);
			return;
		}

		dumpDebug("mTokSendFDLStatusRsp");
		pMPIdata->NextGAP = pMPIdata->FrameUniopNodeNum;
		pMPIdata->FrameSendBuffer[0] = D_SD1;
		pMPIdata->FrameSendBuffer[1] = pMPIdata->Source;
		pMPIdata->FrameSendBuffer[2] = pMPIdata->FrameUniopNodeNum;
		pMPIdata->FrameSendBuffer[3] = (pMPIdata->AreWeInRing)? FCST_MSIR | RFC_OK : FCST_MSRD | RFC_OK;
		pMPIdata->StationStatus[pMPIdata->Source].StationType = 3;

		//Calcola FCS
		CalcCheckSum(pMPIdata->FrameSendBuffer, 1, 4);
		pMPIdata->FrameSendBuffer[5] = D_ED;
		pMPIdata->FrameSendBufferLen = 6;
		sendData(pMPIdata, pMPIdata->FrameSendBuffer, (byte)pMPIdata->FrameSendBufferLen);

		pMPIdata->FlagSendSelfToken = 0;
		pMPIdata->Sd1RespGuard = 1;
		return;
	}
}

unsigned char mTokGotoActiveIdle(struct s_MPIdata *pMPIdata)
{
   return GotoActiveIdle(pMPIdata);
}

#if MPI_SLAVE_STACK
/************************************************************************
*
* PrepareSwitchToConfigFrames
*
*  This function builds the frames for manage the "switch to config" process
*
* Parameters:
*  void
*
* Returns:
*  0 - error
*  1 - ok
***********************************************************************/
byte PrepareSwitchToConfigFrames()
{
   //Prepare Response Frame : WrMyBuff
   if (FrameSessionReceiveBuffer[14+7] == 5)
   {//WR
	  RtlCopyMemory(WrMyBuff, &FrameSessionReceiveBuffer[11], 7);
	  //Prepare WR Ack
	  WrMyBuff[1]  = 0x03;
	  WrMyBuff[7] = 0x02;
	  WrMyBuff[8] = 0x00;
	  WrMyBuff[9] = 0x01;
	  WrMyBuff[10] = 0x00;
	  WrMyBuff[11] = 0x00;
	  WrMyBuff[12] = 0x05;
	  WrMyBuff[13] = 0x01;
	  WrMyBuff[14] = 0xFF;
	  FrameMyBuffLength = 19;
	  MyOperFlag = WR;
	  //Check if the cmd goto_config is ok and prepare answer
	  if (FrameSessionReceiveBuffer[32+4+7] == 0x01 ||
		  FrameSessionReceiveBuffer[32+4+7] == 0x02)
	  {
		 FrameMyBuff[0] = 0x81;
		 FrameMyBuff[1] = 0x01;
		 FrameMyBuff[2] = FrameSessionReceiveBuffer[32+2+7];
		 FrameMyBuff[3] = FrameSessionReceiveBuffer[32+3+7];
		 FrameMyBuff[4] = 0x80 | FrameSessionReceiveBuffer[32+4+7];
		 FrameMyBuff[5] = 0;
		 FrameMyBuff[6] = 0;
		 FrameMyBuff[7] = 1;

//MG001 start
		 if (FrameSessionReceiveBuffer[32+4+7] == 1)
			FrameMyBuff[8] = 1;
		 else
			FrameMyBuff[8] = FrameSessionReceiveBuffer[32+4+7+4];
//MG001 end
	  }
	  else
		 return 0; //error!!
   }
   if (FrameSessionReceiveBuffer[14+7] == 4)
   {//RD
	  MyOperFlag = 1;
	  WrMyBuffLength = MAKEWORD(FrameSessionReceiveBuffer[21+7], FrameSessionReceiveBuffer[20+7]);
	  FrameMyBuffLength = 22 + WrMyBuffLength;
	  //Prepare PDU-Frame Header
	  RtlCopyMemory(WrMyBuff, &FrameSessionReceiveBuffer[11], 7);
	  WrMyBuff[1] = 0x03;
	  WrMyBuff[7] = 0x02;
	  WrMyBuff[8] = 0x00;
	  WrMyBuff[9] = 0x0D;
	  WrMyBuff[10] = 0x00;
	  WrMyBuff[11] = 0x00;
	  WrMyBuff[12] = 0x04;
	  WrMyBuff[13] = 0x01;  //NrOfVars
	  WrMyBuff[14] = 0xFF;
	  WrMyBuff[15] = 0x04;
	  MyLen = MAKEWORD(FrameSessionReceiveBuffer[21+7], FrameSessionReceiveBuffer[20+7]);
	  MyLen = MyLen << 3;
	  WrMyBuff[16] = HIBYTE(MyLen);
	  WrMyBuff[17] = LOBYTE(MyLen);
//MG001      if (FrameMyBuff[4] == 0x82 && MyLen == 9*8)
	  if (FrameMyBuff[4] == 0x82 && FrameMyBuff[8] == 0 && MyLen == 9*8)   //MG001
		 ReadyForConfigMode = 1;
	  else
		 ReadyForConfigMode = 0;
   }
   return 1;
}

void InsertSwitchToConfigFrame(void)
{
   if (MyOperFlag == WR)
	  RtlCopyMemory(&FrameSendBuffer1[11], WrMyBuff, 19);
   else
   {
	  RtlCopyMemory(&WrMyBuff[18], FrameMyBuff, WrMyBuffLength);
	  RtlCopyMemory(&FrameSendBuffer1[11], WrMyBuff, 18 + WrMyBuffLength + 2);
   }
   FrameSendBufferLen1 = FrameSendBuffer1[1]+6;
}

/*----LogSlave---------------------------*/
unsigned char mLgsCheckSAPReqFrame()
{
   //Rx Frame Ok
   if (FrameSessionReceiveBuffer[9] == 0xE0)
   {
	  sendShortAck();

	  //Prepare next frame
	  FrameSendBufferLen1 = LOGON_SAPRSP_LEN;
	  RtlCopyMemory(FrameSendBuffer1, LogOnSAPRsp, LOGON_SAPRSP_LEN);
	  LogOnSlave_DA = FrameSessionReceiveBuffer[5] & 0x7F;
	  StationStatus[LogOnSlave_DA].LogOn_DAE = FrameSessionReceiveBuffer[8];
	  StationStatus[LogOnSlave_DA].LogOn_SAE = Start_LogOn_DAE;
	  FrameSendBuffer1[4] = LogOnSlave_DA | 0x80;
	  FrameSendBuffer1[5] = FrameUniopNodeNum | 0x80;
	  FrameSendBuffer1[7] = StationStatus[LogOnSlave_DA].LogOn_DAE;
	  FrameSendBuffer1[8] = StationStatus[LogOnSlave_DA].LogOn_SAE;

	  //Calcola FCS
	  CalcCheckSum(FrameSendBuffer1, 4, FrameSendBufferLen1 - 2);

	  ev_post(EV_LGS_SEND_SAPRSP);
	  timer_start(EV_BRC_SL_SESS_TIMEOUT, 10000);

	  LgsSessReqPending = true;
	  SlaveSession = true;
	  ReadyForConfigMode = 0;

	  ev_post(EV_BRC_STOP_SESS); //Abort eventually JobSession
	  return LGS_SEND_SAPRSP;
   }
   return LGS_RUN;
}

void mLgsSendRequestedSap()
{
   //Because it's possible a JOB abort
   SessReqPending = true;
   SessionStarted = false;
}

unsigned char mLgsCheckAckSAPRspFrame()
{
   //Rx Frame Ok
   Source = FrameSessionReceiveBuffer[5] & 0x7f;
   if (Source == LogOnSlave_DA)
   {
	  if (FrameSessionReceiveBuffer[9] == 0x05) {
		 sendShortAck();

		 //Prepare next frame
		 FrameSendBufferLen1 = LOGON_SAPRESPACK_LEN;
		 RtlCopyMemory(FrameSendBuffer1, LogOnSAPRespAck, LOGON_SAPRESPACK_LEN);
		 FrameSendBuffer1[4] = LogOnSlave_DA | 0x80;
		 FrameSendBuffer1[5] = FrameUniopNodeNum | 0x80;
		 FrameSendBuffer1[7] = StationStatus[LogOnSlave_DA].LogOn_DAE;
		 FrameSendBuffer1[8] = StationStatus[LogOnSlave_DA].LogOn_SAE;

		 //Calcola FCS
		 CalcCheckSum(FrameSendBuffer1, 4, FrameSendBufferLen1 - 2);

		 ev_post(EV_LGS_SEND_ACKSAPRSPACK);
		 return LGS_SEND_ACKSAPRSPACK;
	  }
	  mLgsAbortSession();
	  return LGS_RUN;
   }
   return LGS_ACKSAPRSP;
}

void mLgsSendAckSAPRspAck()
{
   SessReqPending = true;
   SessionStarted = false;
}

void mLgsLogOnSlaveOk()
{
   timer_stop(EV_BRC_SL_SESS_TIMEOUT); //MG002
   Start_LogOn_DAE++;
   LgsSessReqPending = false;
   StationStatus[LogOnSlave_DA].Logged = 1;
   StationStatus[LogOnSlave_DA].Job   = 0;
   ev_post(EV_J0S_RUN);
}

void mLgsAbortSession()
{
   if (LgsSessReqPending)
   {
	  LgsSessReqPending = false;
	  SessReqPending    = false;
	  StationStatus[LogOnSlave_DA].Logged = 0;
	  StationStatus[LogOnSlave_DA].Job   = 0;
	  StationStatus[LogOnSlave_DA].LogOn_DAE = 0;
	  StationStatus[LogOnSlave_DA].LogStatus = 0;	//MG001
   }
   SlaveSession = false;
}

/*----Job0Slave---------------------------*/
void mJ0sRun()
{
   timer_start(EV_BRC_J0S_SESS_TIMEOUT, 1000);
}

unsigned char mJ0sCheckJobReqFrame()
{
   if (!StationStatus[LogOnSlave_DA].Logged)    //MG001
	  return J0S_RUN;                           //MG001

   Source = FrameSessionReceiveBuffer[5] & 0x7f;   //MG001
   if (Source == LogOnSlave_DA)                    //MG001
   {                                               //MG001
	  //Rx Frame Ok
	  if ((FrameSessionReceiveBuffer[9] == 0xF1) &&
		  (FrameSessionReceiveBuffer[10] == 0x00))
	  {
		 sendShortAck();
		 //Prepare next frame
		 FrameSendBufferLen1 = JOBACTION_ACK_LEN;
		 RtlCopyMemory(FrameSendBuffer1, JobActionAck, JOBACTION_ACK_LEN);
		 FrameSendBuffer1[4] = LogOnSlave_DA | 0x80;
		 FrameSendBuffer1[5] = FrameUniopNodeNum | 0x80;
		 FrameSendBuffer1[7] = StationStatus[LogOnSlave_DA].LogOn_DAE;
		 FrameSendBuffer1[8] = StationStatus[LogOnSlave_DA].LogOn_SAE;

		 //Calcola FCS
		 CalcCheckSum(FrameSendBuffer1, 4, FrameSendBufferLen1 - 2);

		 ev_post(EV_J0S_SEND_JOBACK);
		 J0sSessReqPending = true;
		 timer_start(EV_BRC_J0S_SESS_TIMEOUT, 1000);	//MG001
		 return J0S_SEND_JOBACK;
	  }
   }                                               //MG001
   return J0S_RUN;
}

void mJ0sSendJobAck()
{
   SessReqPending = true;
}

void mJ0sBuildJobRsp()
{
   RtlCopyMemory(FrameSendBuffer1, JobAction2Req0, JOBACTION2_REQ_LEN0);

   FrameSendBuffer1[4]  = LogOnSlave_DA | 0x80;
   FrameSendBuffer1[5]  = FrameUniopNodeNum | 0x80;
   FrameSendBuffer1[7]  = StationStatus[LogOnSlave_DA].LogOn_DAE;
   FrameSendBuffer1[8]  = StationStatus[LogOnSlave_DA].LogOn_SAE;
   FrameSendBufferLen1 = FrameSendBuffer1[1]+6;

   //Calcola FCS
   CalcCheckSum(FrameSendBuffer1, 4, FrameSendBufferLen1 - 2);

   ev_post(EV_J0S_SEND_JOBRSP);
   timer_start(EV_BRC_J0S_SESS_TIMEOUT, 1000);	//MG001
}

void mJ0sSendJobRsp()
{
   SessReqPending = true;
}

void mJ0sAbortSession()
{
   if (J0sSessReqPending)
   {
	  J0sSessReqPending = false;
	  SessReqPending    = false;
	  SessionStarted    = false;
	  StationStatus[LogOnSlave_DA].Logged = 0;
	  StationStatus[LogOnSlave_DA].Job   = 0;
	  StationStatus[LogOnSlave_DA].LogOn_DAE = 0;
	  StationStatus[LogOnSlave_DA].LogStatus = 0;  //MG001
   }
   SlaveSession = false;
}

//MG001 void mJ0sSlaveOk(void)
unsigned char mJ0sSlaveOk()
{
   Source = FrameSessionReceiveBuffer[5] & 0x7f;   //MG001
   if (Source == LogOnSlave_DA)                    //MG001
   {                                               //MG001
	  J0sSessReqPending = false;
	  //Rx Frame Ok
	  if ((FrameSessionReceiveBuffer[9] == 0xB0) &&
		  (FrameSessionReceiveBuffer[10] == 0x01))
	  {
		 sendShortAck();

		 StationStatus[LogOnSlave_DA].Logged = 1;
		 StationStatus[LogOnSlave_DA].Job   = 1;
		 ev_post(EV_JBS_RUN);
		 timer_stop(EV_BRC_J0S_SESS_TIMEOUT);
		 return J0S_IDLE;                          //MG001
	  }
	  ev_post(EV_SES_ERR);
	  return J0S_IDLE;                             //MG001
   }                                               //MG001
   return J0S_WAIT_ACK_JOBRSP;                     //MG001
}

/*----JobSlave---------------------------*/
void mJbsRun()
{
   timer_start(EV_BRC_JBS_SESS_TIMEOUT, 1000);
}

unsigned char mJbsCheckJobReqFrame()
{
   if (!StationStatus[LogOnSlave_DA].Logged) return JBS_RUN;

   //Rx Frame Ok
   Source = FrameSessionReceiveBuffer[5] & 0x7f;
   if (Source == LogOnSlave_DA)
   {
	  if (FrameSessionReceiveBuffer[9] == 0xF1)
	  {
		 sendShortAck();

		 if (!PrepareSwitchToConfigFrames())
			return JBS_RUN;

		 //Prepare JobAck frame
		 FrameSendBufferLen1 = JOBACTION_ACK_LEN;
		 RtlCopyMemory(FrameSendBuffer1, JobActionAck, JOBACTION_ACK_LEN);

		 FrameSendBuffer1[4] = LogOnSlave_DA | 0x80;
		 FrameSendBuffer1[5] = FrameUniopNodeNum | 0x80;
		 FrameSendBuffer1[7] = StationStatus[LogOnSlave_DA].LogOn_DAE;
		 FrameSendBuffer1[8] = StationStatus[LogOnSlave_DA].LogOn_SAE;
		 JbsOnSlave_Job = FrameSessionReceiveBuffer[10];
		 FrameSendBuffer1[11] = JbsOnSlave_Job;

		 //Calcola FCS
		 CalcCheckSum(FrameSendBuffer1, 4, FrameSendBufferLen1 - 2);

		 ev_post(EV_JBS_SEND_JOBACK);
		 JbsSessReqPending = true;
		 timer_start(EV_BRC_JBS_SESS_TIMEOUT, 1000);	//MG001
		 return JBS_SEND_JOBACK;
	  }
   }
   return JBS_RUN;
}

void mJbsSendJobAck()
{
   SessReqPending = true;
}

void mJbsBuildJobRsp()
{
   FrameSendBuffer1[0]  = D_SD2;
   FrameSendBuffer1[1]  = FrameMyBuffLength + 7;
   FrameSendBuffer1[2]  = FrameMyBuffLength + 7;
   FrameSendBuffer1[3]  = D_SD2;
   FrameSendBuffer1[4]  = LogOnSlave_DA | 0x80;
   FrameSendBuffer1[5]  = FrameUniopNodeNum | 0x80;
   FrameSendBuffer1[9]  = 0x5C;
   FrameSendBuffer1[7]  = StationStatus[LogOnSlave_DA].LogOn_DAE;
   FrameSendBuffer1[8]  = StationStatus[LogOnSlave_DA].LogOn_SAE;
   FrameSendBuffer1[9]  = 0xF1;
   FrameSendBuffer1[10] = JbsOnSlave_Job;

   InsertSwitchToConfigFrame();
   FrameSendBufferLen1 = FrameSendBuffer1[1]+6;

   //Calcola FCS
   CalcCheckSum(FrameSendBuffer1, 4, FrameSendBufferLen1 - 2);
   FrameSendBuffer1[FrameSendBufferLen1-1] = D_ED;

   ev_post(EV_JBS_SEND_JOBRSP);
   timer_start(EV_BRC_JBS_SESS_TIMEOUT, 1000);	//MG001
}

void mJbsSendJobRsp()
{
   SessReqPending = true;
}

void mJbsAbortSession()
{
   if (JbsSessReqPending)
   {
	  JbsSessReqPending = false;
	  SessReqPending    = false;
	  SessionStarted    = false;
	  StationStatus[LogOnSlave_DA].Logged = 0;
	  StationStatus[LogOnSlave_DA].Job   = 0;
	  StationStatus[LogOnSlave_DA].LogOn_DAE = 0;
	  StationStatus[LogOnSlave_DA].LogStatus = 0;	//MG001
   }
   SlaveSession = false;
}

unsigned char mJbsSlaveOk()
{
   Source = FrameSessionReceiveBuffer[5] & 0x7f;   //MG001
   if (Source == LogOnSlave_DA)                    //MG001
   {                                               //MG001
	  JbsSessReqPending = false;
	  //Rx Frame Ok
	  if ((FrameSessionReceiveBuffer[9] == 0xB0) &&
		  (FrameSessionReceiveBuffer[10] == 0x01))
	  {

		 sendShortAck();

		 if (ReadyForConfigMode == 1)
		 {
			ReadyForConfigMode = 2;
			timer_stop(EV_BRC_JBS_SESS_TIMEOUT);
			SlaveSession = false;
			//GOTO_CONFIG_MODE();         //switch to config mode!!!
			return JBS_IDLE;
		 }
		 return JBS_RUN;
	  }
	  ev_post(EV_SES_ERR);
	  SlaveSession = false;
   }                                               //MG001
   return JBS_WAIT_ACK_JOBRSP;
}

#endif // MPI_SLAVE_STACK

// Get Event
unsigned char ev_get(struct s_MPIdata *pMPIdata)
{
  unsigned char ev;

	ev = pMPIdata->ev_queue[pMPIdata->ev_queue_wr];
	pMPIdata->ev_queue_wr = (pMPIdata->ev_queue_wr + 1) % N_EVPOST;
	if (pMPIdata->ev_queue_wr == pMPIdata->ev_queue_rd) pMPIdata->queue_empty = 1;
	dumpEvent(ev);
	return ev;
}

// Move The Machines

void Token(struct s_MPIdata *pMPIdata)
{
unsigned char next_state = _NULL;
//unsigned char last_state = pMPIdata->tok_state;
#if EVENTLOG
	#if !SHORT_FORM
		unsigned char s[80];
	#endif
#endif
	switch (pMPIdata->tok_state)
	{
		case TOK_IDLE:
			switch (pMPIdata->event) {
				case EV_BRC_ENABLE:
					next_state = TOK_IDLE; break;
				case EV_TOK_RUN:
					mTokListenTokenStartTimeout(pMPIdata);next_state = TOK_LISTENTOKEN; break;
			} break;
		case TOK_LISTENTOKEN:
			switch (pMPIdata->event) {
				case EV_BRC_TOK:
					mTokListenTokenStartTimeout(pMPIdata);next_state = mTokListenToken(pMPIdata); break;//MG001
				case EV_BRC_SD1:
					mTokListenTokenStartTimeout(pMPIdata);next_state = mTokListenToken(pMPIdata); break;//MG001
				case EV_TOK_TIMEOUT:
					mTokSendSelfToken(pMPIdata);next_state = TOK_LISTENTOKEN; break;
				case EV_BRC_EOTX:
					mTokStartSelfTokTimeout(pMPIdata);next_state = TOK_SELFTOKEN; break;
			} break;
		case TOK_SELFTOKEN:
			switch (pMPIdata->event) {
				case EV_TOK_SELFTOK_TIMEOUT : next_state = mTokSendFDLStatus(pMPIdata); break;
				case EV_BRC_EOTX : mTokStartFDLTimeout(pMPIdata);next_state = TOK_WAITFDLSTATUS2; break;
				case EV_BRC_TOK : mTokFSMRestart(pMPIdata);next_state = TOK_ACTIVEIDLE; break;//MG002
			} break;
		case TOK_WAITFDLSTATUS2:
			switch (pMPIdata->event) {
				case EV_TOK_FDLTIMEOUT : mTokSendSelfToken(pMPIdata);next_state = TOK_WAITFDLSTATUS2; break;
				case EV_BRC_EOTX : next_state = mTokCheckResp(pMPIdata); break;
				case EV_BRC_SD1 : mTokSD1Resp(pMPIdata);next_state = TOK_WAITFDLSTATUS2; break;
				case EV_BRC_TOK : mTokFSMRestart(pMPIdata);next_state = TOK_ACTIVEIDLE; break;//MG002
			} break;
		case TOK_ACTIVEIDLE:
			switch (pMPIdata->event) {
				case EV_BRC_SD1:
					mTokActiveIdle(pMPIdata);next_state = TOK_ACTIVEIDLE; break;
				case EV_BRC_EOTX:
					next_state = TOK_WAITRX; break; //MG001
				case EV_BRC_TOK:
					mTokStartTimeout10(pMPIdata);next_state = TOK_ACTIVEIDLE; break;   //MG001
				case EV_SES_SD2:
					mTokStartTimeout10(pMPIdata);next_state = TOK_ACTIVEIDLE; break;   //MG001
				case EV_TOK_TIMEOUT:
					mTokSelfToken(pMPIdata);next_state = TOK_WAITFDLSTATUS2; break; //MG002
			} break;
		case TOK_TOKEN_RETRY:
			switch (pMPIdata->event) {
				case EV_BRC_EOTX : mTokStartTimeoutTok(pMPIdata);next_state = TOK_WAITRX; break;
				case EV_BRC_TOK : next_state = TOK_ACTIVEIDLE; break;
			} break;
		case TOK_WAITRX:
			switch (pMPIdata->event) {
				case EV_BRC_SD1 : mTokListenTokenStartTimeout(pMPIdata);mTokSendFDLStatusRsp(pMPIdata);next_state = TOK_WAITRX; break;   //MG001
				case EV_SES_SD2 : mTokListenTokenStartTimeout(pMPIdata);next_state = TOK_WAITRX; break;        //MG001
				case EV_BRC_TOK : next_state = mTokTxFrame(pMPIdata); break;
				case EV_BRC_EOTX : next_state = mTokCheckReply(pMPIdata); break;
				case EV_TOK_TIMEOUT_TOK : next_state = mTokPassToken1(pMPIdata); break;
				case EV_TOK_TIMEOUT : mTokSelfToken(pMPIdata);next_state = TOK_WAITFDLSTATUS2; break; //MG002
			} break;
		case TOK_WAITFDLSTATUS:
			switch (pMPIdata->event) {
				case EV_TOK_FDLTIMEOUT : next_state = mTokPassToken(pMPIdata); break;
				case EV_BRC_EOTX : mTokStartTimeoutTok(pMPIdata);next_state = TOK_WAITRX; break;
				case EV_BRC_SD1 : next_state = mTokCheckFDLStatusResp(pMPIdata); break;
				case EV_BRC_TOK : next_state = mTokListenToken(pMPIdata); break;
			} break;
		case TOK_WAITSESSRX:
			switch (pMPIdata->event) {
				case EV_BRC_ACK : next_state = mTokWaitSessRx(pMPIdata, 0); break;
				case EV_BRC_SD1 : next_state = mTokWaitSessRxSD1(pMPIdata); break;
				case EV_BRC_EOTX : next_state = mTokCheckGuard(pMPIdata); break;
				case EV_BRC_ACK_TIMEOUT: next_state = mTokWaitSessRx(pMPIdata, 1);  break;
#ifdef MANAGE_RR_ANSWERS
				case EV_BRC_SDX: next_state = mTokAnswerRR(pMPIdata);  break;
#endif
			} break;
	}
	if (next_state != _NULL)
	{
		#if EVENTLOG
		  #if SUBSET_EVENTLOG
			if (msgEventsLog[event]) {
		  #endif
		   #if SHORT_FORM
				dbg_byte = tok_state+LOG_STATE_OFFSET;
				UltiUART2_FifoWrite(&dbg_byte, 1);
				dbg_byte = next_state+LOG_STATE_OFFSET;
				  UltiUART2_FifoWrite(&dbg_byte, 1);
			#else
					sprintf(s, "(%s)->(%s)", msgStates[tok_state], msgStates[next_state]);
				   DbgStrPrint(s);
			#endif
			#if SUBSET_EVENTLOG
			 }
		#endif
		#endif
		pMPIdata->tok_state = next_state;
		DBGReg(pMPIdata->event, last_state, pMPIdata->tok_state);
	}
	dumpTokState(pMPIdata->tok_state);
}

void LogOff(struct s_MPIdata *pMPIdata)
{
unsigned char next_state = _NULL;
//unsigned char last_state = pMPIdata->lgf_state;
#if EVENTLOG
	#if !SHORT_FORM
		unsigned char s[80];
	#endif
#endif
	switch (pMPIdata->lgf_state) {
		case LGF_IDLE:
			switch (pMPIdata->event) {
				case EV_BRC_ENABLE : next_state = LGF_IDLE; break;
				case EV_LGF_RUN : next_state = LGF_RUN; break;
			} break;
		case LGF_RUN:
			switch (pMPIdata->event) {
				case EV_SES_SD2 : next_state = mLgfCheckLogOff(pMPIdata); break;
			} break;
		case LGF_SENT_LOGOFF_ACK:
			switch (pMPIdata->event) {
				case EV_LGF_SEND_LOGOFFACK : mLgfSendLogOffAck(pMPIdata);next_state = LGF_SENT_LOGOFF_ACK; break;
//MG002				case EV_BRC_SD1 : next_state = mLgfCheckSD1(); break;
				case EV_SES_ACK : mLgfAbortSession(pMPIdata);next_state = LGF_RUN; break;
				case EV_SES_ERR : mLgfAbortSession(pMPIdata);next_state = LGF_RUN; break;
				case EV_BRC_APPL_TIMEOUT : mLgfAbortSession(pMPIdata);next_state = LGF_RUN; break;
			} break;
	}
	if (next_state != _NULL) {
	#if EVENTLOG
	#if SUBSET_EVENTLOG
	if (msgEventsLog[event]) {
	#endif
		#if SHORT_FORM
			DebugPrint(lgf_state+LOG_STATE_OFFSET);DebugPrint(next_state+LOG_STATE_OFFSET);
		#else
				sprintf(s, "(%s)->(%s)", msgStates[lgf_state], msgStates[next_state]);
			DbgStrPrint(s);
		#endif
	#if SUBSET_EVENTLOG
	}
	#endif
	#endif
	pMPIdata->lgf_state = next_state;
	DBGReg(pMPIdata->event, last_state, pMPIdata->lgf_state);
	}
}


#if MPI_MASTER_STACK
void Log(struct s_MPIdata *pMPIdata)
{
unsigned char next_state = _NULL;
//unsigned char last_state = pMPIdata->log_state;
#if EVENTLOG
	#if !SHORT_FORM
		unsigned char s[80];
	#endif
#endif
	switch (pMPIdata->log_state) {
		case LOG_IDLE:
			switch (pMPIdata->event) {
				case EV_BRC_ENABLE : next_state = LOG_IDLE; break;
				case EV_LOG_RUN : next_state = LOG_RUN; break;
			} break;
		case LOG_RUN:
			switch (pMPIdata->event) {
				case EV_LOG_START : mLogSendSAPReqFrame(pMPIdata);next_state = LOG_WAITSAPREQACK; break;
				//case EV_SES_SD2 : next_state = LOG_RUN; break;
			} break;
		case LOG_WAITSAPREQACK:
			switch (pMPIdata->event) {
				case EV_SES_SD2 : next_state = mLogCheckLogOff(pMPIdata); break;
				case EV_SES_ACK : next_state = LOG_WAITSAPRSP; break;
				//MG003 case EV_SES_ERR : mLogAbortSession();next_state = LOG_RUN; break;
				//MG003 case EV_BRC_APPL_TIMEOUT : mLogAbortSession();next_state = LOG_RUN; break;
				case EV_SES_ERR : next_state = mLogAbortSession(pMPIdata); break;//MG003
				case EV_BRC_APPL_TIMEOUT : next_state = mLogAbortSession(pMPIdata); break;//MG003
			} break;
		case LOG_WAITSAPRSP:
			switch (pMPIdata->event) {
				case EV_SES_SD2 : next_state = mLogSendRespAck(pMPIdata); break;
				//MG003 case EV_BRC_APPL_TIMEOUT : mLogAbortSession();next_state = LOG_RUN; break;
				case EV_BRC_APPL_TIMEOUT : next_state = mLogAbortSession(pMPIdata); break;//MG003
			} break;
		case LOG_WAITSAPRSPACK:
			switch (pMPIdata->event) {
				case EV_SES_SD2 : next_state = mLogCheckLogOff(pMPIdata); break;
				case EV_SES_ACK : next_state = LOG_WAITACKSAPRSPACK; break;
				//MG003 case EV_SES_ERR : mLogAbortSession();next_state = LOG_RUN  ; break;
				//MG003 case EV_BRC_APPL_TIMEOUT : mLogAbortSession();next_state = LOG_RUN ; break;
				case EV_SES_ERR : next_state = mLogAbortSession(pMPIdata); break;//MG003
				case EV_BRC_APPL_TIMEOUT : next_state = mLogAbortSession(pMPIdata); break;//MG003
			} break;
		case LOG_WAITACKSAPRSPACK:
			switch (pMPIdata->event) {
				case EV_SES_SD2 : next_state = mLogWaitAckRespAck(pMPIdata); break;
				//MG003 case EV_SES_ERR : mLogAbortSession();next_state = LOG_RUN         ; break;
				//MG003 case EV_BRC_APPL_TIMEOUT : mLogAbortSession();next_state = LOG_RUN  ; break;
				case EV_SES_ERR : next_state = mLogAbortSession(pMPIdata); break;//MG003
				case EV_BRC_APPL_TIMEOUT : next_state = mLogAbortSession(pMPIdata);break;//MG003
			} break;
	}
	if (next_state != _NULL) {
   #if EVENTLOG
		#if SUBSET_EVENTLOG
		if (msgEventsLog[event]) {
		#endif
	   #if SHORT_FORM
			dbg_byte = log_state+LOG_STATE_OFFSET;
			UltiUART2_FifoWrite(&dbg_byte, 1);
			dbg_byte = next_state+LOG_STATE_OFFSET;
			  UltiUART2_FifoWrite(&dbg_byte, 1);
		#else
				sprintf(s, "(%s)->(%s)", msgStates[log_state], msgStates[next_state]);
			   DbgStrPrint(s);
		#endif
		#if SUBSET_EVENTLOG
		 }
		#endif
	#endif
		pMPIdata->log_state = next_state;
		dumpLogState(pMPIdata->log_state);
		DBGReg(pMPIdata->event, last_state, pMPIdata->log_state);
   }
}

void Job0(struct s_MPIdata *pMPIdata)
{
unsigned char next_state = _NULL;
//unsigned char last_state = pMPIdata->jb0_state;
#if EVENTLOG
	#if !SHORT_FORM
		unsigned char s[80];
	#endif
#endif
	switch (pMPIdata->jb0_state) {
		case JB0_IDLE:
			switch (pMPIdata->event) {
				case EV_BRC_ENABLE : next_state = JB0_IDLE; break;
				case EV_JB0_RUN : next_state = JB0_RUN; break;
			} break;
		case JB0_RUN:
			switch (pMPIdata->event) {
				case EV_JB0_SEND_REQ : mJob0SendReq(pMPIdata);next_state = JB0_WAITSHORTACK0; break;
			} break;
		case JB0_WAITSHORTACK0:
			switch (pMPIdata->event) {
				case EV_LOG_OFF : mJob0ApplicationResponseErr(pMPIdata);next_state = JB0_RUN; break;
				case EV_SES_ACK : next_state = JB0_WAITJOBACK0; break;
				case EV_SES_ERR : mJob0ApplicationResponseErr(pMPIdata);next_state = JB0_RUN; break;
				case EV_BRC_APPL_TIMEOUT : mJob0ApplicationResponseErr(pMPIdata);next_state = JB0_RUN; break;
			} break;
		case JB0_WAITJOBACK0:
			switch (pMPIdata->event) {
				case EV_LOG_OFF : mJob0ApplicationResponseErr(pMPIdata);next_state = JB0_RUN; break;
				case EV_SES_SD2 : next_state = mJob0SendShortAck(pMPIdata); break;
				case EV_SES_ERR : mJob0ApplicationResponseErr(pMPIdata);next_state = JB0_RUN; break;
				case EV_BRC_APPL_TIMEOUT : mJob0ApplicationResponseErr(pMPIdata);next_state = JB0_RUN; break;
			} break;
		case JB0_WAITJOBRESP0:
			switch (pMPIdata->event) {
				case EV_LOG_OFF : mJob0ApplicationResponseErr(pMPIdata);next_state = JB0_RUN; break;
				case EV_SES_SD2 : next_state = mJob0SendJobAck(pMPIdata); break;
				case EV_SES_ERR : mJob0ApplicationResponseErr(pMPIdata);next_state = JB0_RUN; break;
				case EV_BRC_APPL_TIMEOUT : mJob0ApplicationResponseErr(pMPIdata);next_state = JB0_RUN; break;
			} break;
		case JB0_WAITSHORTACK02:
			switch (pMPIdata->event) {
				case EV_LOG_OFF : mJob0ApplicationResponseErr(pMPIdata);next_state = JB0_RUN; break;
				case EV_SES_ACK : mJob0Ok(pMPIdata);next_state = JB0_RUN; break;
				case EV_SES_ERR : mJob0ApplicationResponseErr(pMPIdata);next_state = JB0_RUN; break;
				case EV_BRC_APPL_TIMEOUT : mJob0ApplicationResponseErr(pMPIdata);next_state = JB0_RUN; break;
			} break;
	}
	if (next_state != _NULL) {
	#if EVENTLOG
	#if SUBSET_EVENTLOG
	if (msgEventsLog[event]) {
	#endif
		#if SHORT_FORM
			dbg_byte = jb0_state+LOG_STATE_OFFSET;
			UltiUART2_FifoWrite(&dbg_byte, 1);
			dbg_byte = next_state+LOG_STATE_OFFSET;
			  UltiUART2_FifoWrite(&dbg_byte, 1);
		#else
				sprintf(s, "(%s)->(%s)", msgStates[jb0_state], msgStates[next_state]);
			DbgStrPrint(s);
		#endif
	#if SUBSET_EVENTLOG
	}
	#endif
	#endif
	pMPIdata->jb0_state = next_state;
	dumpJb0State(pMPIdata->jb0_state);
	DBGReg(pMPIdata->event, last_state, pMPIdata->jb0_state);
	}
}

void Job(struct s_MPIdata *pMPIdata)
{
unsigned char next_state = _NULL;
//unsigned char last_state = pMPIdata->job_state;
#if EVENTLOG
	#if !SHORT_FORM
		unsigned char s[80];
	#endif
#endif
	switch (pMPIdata->job_state) {
		case JOB_IDLE:
			switch (pMPIdata->event) {
				case EV_BRC_ENABLE : next_state = JOB_IDLE; break;
				case EV_JOB_RUN : next_state = JOB_RUN; break;
			} break;
		case JOB_RUN:
			switch (pMPIdata->event) {
				case EV_JOB_SEND_REQ : mJobSendReq(pMPIdata);next_state = JOB_WAITSHORTACK; break;
			} break;
		case JOB_WAITSHORTACK:
			switch (pMPIdata->event) {
				case EV_LOG_OFF : mJobApplicationResponseErr(pMPIdata);next_state = JOB_RUN; break;
				case EV_SES_ACK : next_state = JOB_WAITJOBACK; break;
				case EV_SES_ERR : mJobApplicationResponseErr(pMPIdata);next_state = JOB_RUN; break;
				case EV_BRC_APPL_TIMEOUT : mJobApplicationResponseErr(pMPIdata);next_state = JOB_RUN; break;
				case EV_BRC_STOP_SESS : mJobStopSess(pMPIdata);next_state = JOB_RUN; break;
				case EV_SES_RETRY : next_state = mJobRetryReq(pMPIdata); break;
			} break;
		case JOB_WAITJOBACK:
			switch (pMPIdata->event) {
				case EV_LOG_OFF : mJobApplicationResponseErr(pMPIdata);next_state = JOB_RUN; break;
				case EV_SES_SD2 : next_state = mJobSendShortAck(pMPIdata); break;
				case EV_SES_ERR : mJobApplicationResponseErr(pMPIdata);next_state = JOB_RUN; break;
				case EV_BRC_APPL_TIMEOUT : mJobApplicationResponseErr(pMPIdata);next_state = JOB_RUN; break;
				case EV_BRC_STOP_SESS : mJobStopSess(pMPIdata);next_state = JOB_RUN; break;
			} break;
		case JOB_WAITJOBRESP:
			switch (pMPIdata->event) {
				case EV_LOG_OFF : mJobApplicationResponseErr(pMPIdata);next_state = JOB_RUN; break;
				case EV_SES_SD2 : next_state = mJobSendJobAck(pMPIdata); break;
				case EV_SES_ERR : mJobApplicationResponseErr(pMPIdata);next_state = JOB_RUN; break;
				case EV_BRC_APPL_TIMEOUT : mJobApplicationResponseErr(pMPIdata);next_state = JOB_RUN; break;
				case EV_BRC_STOP_SESS : mJobStopSess(pMPIdata);next_state = JOB_RUN; break;
			} break;
		case JOB_WAITSHORTACK2:
			switch (pMPIdata->event) {
				case EV_LOG_OFF : mJobApplicationResponseErr(pMPIdata);next_state = JOB_RUN; break;
				case EV_SES_ACK : mJobApplicationResponse(pMPIdata);next_state = JOB_RUN; break;
				case EV_SES_ERR : mJobApplicationResponseErr(pMPIdata);next_state = JOB_RUN; break;
				case EV_BRC_APPL_TIMEOUT : mJobApplicationResponseErr(pMPIdata);next_state = JOB_RUN; break;
				case EV_BRC_STOP_SESS : mJobStopSess(pMPIdata);next_state = JOB_RUN; break;
				case EV_SES_RETRY : next_state = mJobRetryJobAck(pMPIdata); break;
			} break;
	}
	if (next_state != _NULL) {
	#if EVENTLOG
	#if SUBSET_EVENTLOG
	if (msgEventsLog[event]) {
	#endif
		#if SHORT_FORM
			dbg_byte = job_state+LOG_STATE_OFFSET;
			UltiUART2_FifoWrite(&dbg_byte, 1);
			dbg_byte = next_state+LOG_STATE_OFFSET;
			  UltiUART2_FifoWrite(&dbg_byte, 1);
		#else
				sprintf(s, "(%s)->(%s)", msgStates[job_state], msgStates[next_state]);
			DbgStrPrint(s);
		#endif
	#if SUBSET_EVENTLOG
	}
	#endif
	#endif
		pMPIdata->job_state = next_state;
		dumpJobState(pMPIdata->job_state);
		DBGReg(pMPIdata->event, last_state, pMPIdata->job_state);
	}
}
#endif

#if MPI_SLAVE_STACK
void LogSlave()
{
unsigned char next_state = _NULL;
#if EVENTLOG
	#if !SHORT_FORM
		unsigned char s[80];
	#endif
#endif
	switch (lgs_state) {
		case LGS_IDLE:
			switch (event) {
				case EV_BRC_ENABLE : next_state = LGS_IDLE; break;
				case EV_LGS_RUN : next_state = LGS_RUN; break;
			} break;
		case LGS_RUN:
			switch (event) {
				case EV_SES_SD2 : next_state = mLgsCheckSAPReqFrame(); break;
				case EV_BRC_SL_SESS_TIMEOUT : mLgsAbortSession();next_state = LGS_RUN; break;
			} break;
		case LGS_SEND_SAPRSP:
			switch (event) {
				case EV_SES_ERR : mLgsAbortSession();next_state = LGS_RUN; break;
				case EV_LGS_SEND_SAPRSP : mLgsSendRequestedSap();next_state = LGS_SEND_SAPRSP; break;
				case EV_SES_ACK : next_state = LGS_ACKSAPRSP; break;
				case EV_BRC_SL_SESS_TIMEOUT : mLgsAbortSession();next_state = LGS_RUN; break;
			} break;
		case LGS_ACKSAPRSP:
			switch (event) {
				case EV_SES_SD2 : next_state = mLgsCheckAckSAPRspFrame(); break;
				case EV_BRC_SL_SESS_TIMEOUT : mLgsAbortSession();next_state = LGS_RUN; break;
			} break;
		case LGS_SEND_ACKSAPRSPACK:
			switch (event) {
				case EV_BRC_SL_SESS_TIMEOUT : mLgsAbortSession();next_state = LGS_RUN; break;
				case EV_SES_ERR : mLgsAbortSession();next_state = LGS_RUN; break;
				case EV_LGS_SEND_ACKSAPRSPACK : mLgsSendAckSAPRspAck();next_state = LGS_SEND_ACKSAPRSPACK; break;
				case EV_SES_ACK : mLgsLogOnSlaveOk();next_state = LGS_RUN; break;
			} break;
	}
	if (next_state != _NULL) {
	#if EVENTLOG
	#if SUBSET_EVENTLOG
	if (msgEventsLog[event]) {
	#endif
		#if SHORT_FORM
			DebugPrint(lgs_state+LOG_STATE_OFFSET);DebugPrint(next_state+LOG_STATE_OFFSET);
		#else
				sprintf(s, "(%s)->(%s)", msgStates[lgs_state], msgStates[next_state]);
			DbgStrPrint(s);
		#endif
	#if SUBSET_EVENTLOG
	}
	#endif
	#endif
	lgs_state = next_state;
	}
}

void Job0Slave()
{
unsigned char next_state = _NULL;
#if EVENTLOG
	#if !SHORT_FORM
		unsigned char s[80];
	#endif
#endif
	switch (j0s_state) {
		case J0S_IDLE:
			switch (event) {
				case EV_BRC_ENABLE : next_state = J0S_IDLE; break;
				case EV_J0S_RUN : mJ0sRun();next_state = J0S_RUN; break;
			} break;
		case J0S_RUN:
			switch (event) {
				case EV_SES_SD2 : next_state = mJ0sCheckJobReqFrame(); break;
				case EV_BRC_J0S_SESS_TIMEOUT : mJ0sAbortSession();next_state = J0S_IDLE; break;
			} break;
		case J0S_SEND_JOBACK:
			switch (event) {
				case EV_SES_ERR : mJ0sAbortSession();next_state = J0S_IDLE; break;
				cavoidse EV_J0S_SEND_JOBACK : mJ0sSendJobAck();next_state = J0S_SEND_JOBACK; break;
				case EV_SES_ACK : mJ0sBuildJobRsp();next_state = J0S_SEND_JOBRSP; break;
				case EV_BRC_J0S_SESS_TIMEOUT : mJ0sAbortSession();next_state = J0S_IDLE; break;
			} break;
		case J0S_SEND_JOBRSP:
			switch (event) {
				case EV_SES_ERR : mJ0sAbortSession();next_state = J0S_IDLE; break;
				case EV_J0S_SEND_JOBRSP : mJ0sSendJobRsp();next_state = J0S_SEND_JOBRSP; break;
				case EV_SES_ACK : next_state = J0S_WAIT_ACK_JOBRSP; break;
				case EV_BRC_J0S_SESS_TIMEOUT : mJ0sAbortSession();next_state = J0S_IDLE; break;
			} break;
		case J0S_WAIT_ACK_JOBRSP:
			switch (event) {
//MG001				case EV_SES_SD2 : mJ0sSlaveOk();next_state = J0S_IDLE; break;
				case EV_SES_SD2 : next_state = mJ0sSlaveOk();break;   //MG001
				case EV_SES_ERR : mJ0sAbortSession();next_state = J0S_IDLE; break;
				case EV_BRC_J0S_SESS_TIMEOUT : mJ0sAbortSession();next_state = J0S_IDLE; break;
			} break;
	}
	if (next_state != _NULL) {
	#if EVENTLOG
	#if SUBSET_EVENTLOG
	if (msgEventsLog[event]) {
	#endif
		#if SHORT_FORM
			dbg_byte = j0s_state+LOG_STATE_OFFSET;
			UltiUART2_FifoWrite(&dbg_byte, 1);
			dbg_byte = next_state+LOG_STATE_OFFSET;
			  UltiUART2_FifoWrite(&dbg_byte, 1);
		#else
				sprintf(s, "(%s)->(%s)", msgStates[j0s_state], msgStates[next_state]);
			DbgStrPrint(s);
		#endif
	#if SUBSET_EVENTLOG
	}
	#endif
	#endif
	j0s_state = next_state;
	}
}

void JobSlave()
{
unsigned char next_state = _NULL;
#if EVENTLOG
	#if !SHORT_FORM
		unsigned char s[80];
	#endif
#endif
	switch (jbs_state) {
		case JBS_IDLE:
			switch (event) {
				case EV_BRC_ENABLE : next_state = JBS_IDLE; break;
				case EV_JBS_RUN : mJbsRun();next_state = JBS_RUN; break;
			} break;
		case JBS_RUN:
			switch (event) {
				case EV_SES_SD2 : next_state = mJbsCheckJobReqFrame(); break;
				case EV_BRC_JBS_SESS_TIMEOUT : mJbsAbortSession();next_state = JBS_IDLE; break;
			} break;
		case JBS_SEND_JOBACK:
			switch (event) {
				case EV_SES_ERR : mJbsAbortSession();next_state = JBS_IDLE; break;
				case EV_JBS_SEND_JOBACK : mJbsSendJobAck();next_state = JBS_SEND_JOBACK; break;
				case EV_SES_ACK : mJbsBuildJobRsp();next_state = JBS_SEND_JOBRSP; break;
				case EV_BRC_JBS_SESS_TIMEOUT : mJbsAbortSession();next_state = JBS_IDLE; break;
			} break;
		case JBS_SEND_JOBRSP:
			switch (event) {
				case EV_SES_ERR : mJbsAbortSession();next_state = JBS_IDLE; break;
				case EV_JBS_SEND_JOBRSP : mJbsSendJobRsp();next_state = JBS_SEND_JOBRSP; break;
				case EV_SES_ACK : next_state = JBS_WAIT_ACK_JOBRSP; break;
				case EV_BRC_JBS_SESS_TIMEOUT : mJbsAbortSession();next_state = JBS_IDLE; break;
			} break;
		case JBS_WAIT_ACK_JOBRSP:
			switch (event) {
				case EV_SES_SD2 : next_state = mJbsSlaveOk(); break;
				case EV_SES_ERR : mJbsAbortSession();next_state = JBS_IDLE; break;
				case EV_BRC_JBS_SESS_TIMEOUT : mJbsAbortSession();next_state = JBS_IDLE; break;
			} break;
	}
	if (next_state != _NULL) {
	#if EVENTLOG
	#if SUBSET_EVENTLOG
	if (msgEventsLog[event]) {
	#endif
		#if SHORT_FORM
			DebugPrint(jbs_state+LOG_STATE_OFFSET);DebugPrint(next_state+LOG_STATE_OFFSET);
		#else
				sprintf(s, "(%s)->(%s)", msgStates[jbs_state], msgStates[next_state]);
			DbgStrPrint(s);
		#endif
	#if SUBSET_EVENTLOG
	}
	#endif
	#endif
	jbs_state = next_state;
	}
}



// Post event
void ev_post(unsigned char ev)
{
	ev_queue[ev_queue_rd] = ev;
	ev_queue_rd = (ev_queue_rd + 1) % N_EVPOST;
	queue_empty = 0;
}
#endif
// Get Event

void ev_move(struct s_MPIdata *pMPIdata, unsigned char ev)
{
#if EVENTLOG
   #if !SHORT_FORM
	  char s[20];
   #endif
#endif

   ev_post(pMPIdata, ev);
   pMPIdata->event = ev_get(pMPIdata);
	while ( pMPIdata->event )
	{
	  #if EVENTLOG
		#if SUBSET_EVENTLOG
			   if (msgEventsLog[event])
			   {
		 #endif
		 #if SHORT_FORM
				  dbg_byte = event + LOG_EVENT_OFFSET;
					UltiUART2_FifoWrite(&dbg_byte, 1);
		#else
					sprintf(s, "\n\r%s:", msgEvents[event]);
				 DbgStrPrint(s);
	   #endif
		#if SUBSET_EVENTLOG
			   }
		 #endif
		#endif

	  Token(pMPIdata);
	  LogOff(pMPIdata);

	  #if MPI_SLAVE_STACK
		 LogSlave();
	  #endif

	  #if MPI_MASTER_STACK
		 Job(pMPIdata);
		 Log(pMPIdata);
		 Job0(pMPIdata);
	  #endif

	  #if MPI_SLAVE_STACK
		 Job0Slave(pMPIdata);
		 JobSlave(pMPIdata);
	  #endif

	  if (pMPIdata->queue_empty)
	  {
		 return;
	  }
	  pMPIdata->event = ev_get(pMPIdata);
   }
}

void setRxFifoThrForMPI(struct uart_omap_port *up, int status)
{
	unsigned int TLRtmp;
	/* FIFOs and DMA Settings */

	/* FCR can be changed only when the
	 * baud clock is not running
	 * DLL_REG and DLH_REG set to 0.
	 */

	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_A);
	serial_out(up, UART_DLL, 0);
	serial_out(up, UART_DLM, 0);
	serial_out(up, UART_LCR, 0);

	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);

	up->efr = serial_in(up, UART_EFR) & ~UART_EFR_ECB;
	up->efr &= ~UART_EFR_SCD;
	serial_out(up, UART_EFR, up->efr | UART_EFR_ECB);

	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_A);
	up->mcr = serial_in(up, UART_MCR) & ~UART_MCR_TCRTLR;
	serial_out(up, UART_MCR, up->mcr | UART_MCR_TCRTLR);
	/* FIFO ENABLE, DMA MODE */

	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);

	TLRtmp = serial_in(up, UART_OMAP_TLR) & 0x0f;
	serial_out(up, UART_OMAP_TLR, ((status)?0x8F:0) | TLRtmp);	//rxFIFOth=33, txFIFth=60

	/* Reset UART_MCR_TCRTLR: this must be done with the EFR_ECB bit set */
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_A);
	serial_out(up, UART_MCR, up->mcr);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);
	serial_out(up, UART_EFR, up->efr);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_A);
}

void MPIDriverOpen(struct s_MPIdata *pMPIdata, struct s_MPIparams init)
{
	if (pMPIdata->m_isOpen)
	{
		pMPIdata->FrameUniopNodeNum = init.panelNode;	//????????????? do it in open
		ResetStations(pMPIdata);
	}
	else
	{
		pMPIdata->FrameUniopNodeNum = init.panelNode;	//????????????? do it in open
		pMPIdata->MaxStationAddress = init.maxNode;
		pMPIdata->applResponseTime = init.applTimeout;
		pMPIdata->guardTimeConstant = init.guardTimeoutConstant;
		pMPIdata->guardTimeFactor = init.guardTimeoutFactor;
		pMPIdata->ackGuardTime = init.ackTimeout;
		pMPIdata->FDLTimeout = init.fdlTimeout;
		pMPIdata->tokTimeout = init.tokTimeout;
		pMPIdata->selfTokTimeout = init.selfTokTimeout;
		pMPIdata->SlaveSession = false;
		pMPIdata->ProcedureApplMRequestPending  = false;
		pMPIdata->ProcedureApplMResponsePending = false;

		pMPIdata->LowestAddress = 0xFF;
		pMPIdata->NextGAP = pMPIdata->FrameUniopNodeNum;
		pMPIdata->NextStation = -1;
		pMPIdata->AreWeInRing = false;
		pMPIdata->ReadyForConfigMode = 0;
		pMPIdata->GapUpdateFactor = 5;
		pMPIdata->Start_LogOn_SAE = 20;
		pMPIdata->Start_LogOn_DAE = 20;
		mInitEngine(pMPIdata);
		mTokStartTimeout10(pMPIdata);
		Init_ev_queue(pMPIdata);
		ev_move(pMPIdata, EV_BRC_ENABLE);
		pMPIdata->m_taskLen = 0;
		pMPIdata->m_isOpen = true;
		pMPIdata->MPIenabled = true;
		pMPIdata->rxCnt = 0;
	}
}

static void mpiReceiveFrame(struct s_MPIdata *pMPIdata, unsigned char *buf, unsigned int len)
{
	unsigned char l;
	StopTimeoutTok(pMPIdata);
	while (len) {
		switch (buf[0])
		{
			case D_SD1:
				if (len < 6) {
					len = 0;
					break;
				}
				pMPIdata->FrameSessionReceiveLength = 6;
				memcpy(pMPIdata->FrameSessionReceiveBuffer, buf, 6);
				pMPIdata->Dest = buf[1];
				pMPIdata->Source = buf[2];
				DBGReg(255, D_SD1, MAKEWORD(pMPIdata->Source, pMPIdata->Dest));
				pMPIdata->FlowCtrl = buf[3];
				pMPIdata->StationStatus[pMPIdata->Source].IsActive = 1;
	#ifdef MANAGE_RR_ANSWERS
				if (pMPIdata->Dest == pMPIdata->FrameUniopNodeNum && (pMPIdata->FlowCtrl & 0x4F) == 2)
					ev_move(pMPIdata, EV_BRC_SDX);
				else
	#endif
				ev_move(pMPIdata, EV_BRC_SD1);
				len -= 6;
				buf += 6;
				pMPIdata->rxCnt -= 6;
				break;
			case D_SD2:
				l =  buf[1]+6;
				if (len < l) {
					len = 0;
					break;
				}
				pMPIdata->FrameSessionReceiveLength = l;
				memcpy(pMPIdata->FrameSessionReceiveBuffer, buf, l);
				pMPIdata->FlowCtrl = buf[6];
				pMPIdata->Dest = buf[4] & 0x7F;
				pMPIdata->Source = buf[5] & 0x7F;
#if 0
				{
					byte AM_JobNr = pMPIdata->FrameSessionReceiveBuffer[9] == 0xB0 ? pMPIdata->FrameSessionReceiveBuffer[11] : pMPIdata->FrameSessionReceiveBuffer[10];
					DBGReg(255, D_SD2, MAKEWORD(((pMPIdata->Source << 4) | (pMPIdata->Dest & 0x0F)),
												((pMPIdata->FrameSessionReceiveBuffer[9] & 0xF0) |
												(AM_JobNr & 0x0F))));
				}
#endif
				len -= l;
				buf += l;
				pMPIdata->rxCnt -= l;
				pMPIdata->StationStatus[pMPIdata->Source].IsActive = 1;
				if (pMPIdata->Dest != pMPIdata->FrameUniopNodeNum)
				{
					if (pMPIdata->tok_state == TOK_WAITRX ||         //MG001
						pMPIdata->tok_state == TOK_LISTENTOKEN ||    //MG001
						pMPIdata->tok_state == TOK_ACTIVEIDLE)       //MG001
						mTokListenTokenStartTimeout(pMPIdata);      //MG001
					break;
				}
	#ifdef MANAGE_RR_ANSWERS
				if ((pMPIdata->FlowCtrl & 0x4F) == 2)
					ev_move(pMPIdata, EV_BRC_SDX);
				else
	#endif
					ev_move(pMPIdata, EV_SES_SD2);
				break;
			case D_TOK:
				if (len < 3) {
					len = 0;
					break;
				}
				pMPIdata->Dest   = buf[1] & 0x7F;                        //MG002
				pMPIdata->Source = buf[2] & 0x7F;                        //MG002
				len -= 3;
				buf += 3;
				pMPIdata->rxCnt -= 3;
				DBGReg(255, D_TOK, MAKEWORD(pMPIdata->Source, pMPIdata->Dest));
				if ((pMPIdata->Source > NR_MAX_STATIONS - 1) || (pMPIdata->Dest > NR_MAX_STATIONS - 1)) {  //MG002
					break;                                                            //MG002
				}
				pMPIdata->StationStatus[pMPIdata->Source].IsActive = 1;
				pMPIdata->StationStatus[pMPIdata->Source].StationType = 3;
				ev_move(pMPIdata, EV_BRC_TOK);
				break;
			case D_SC:
				DBGReg(255, D_SC, 0);
				len -= 1;
				buf += 1;
				pMPIdata->rxCnt -= 1;
				if (pMPIdata->tok_state == TOK_WAITSESSRX)
					ev_move(pMPIdata, EV_BRC_ACK); // consider ack only if I sent SD2 req
				break;
			default:
				len--;
				buf++;
				pMPIdata->rxCnt--;
				break;
		} //switch(FrameSessionReceiveBuffer[0])
	}
	if (pMPIdata->rxCnt) {
		memcpy(pMPIdata->mpiRxBuf, buf, pMPIdata->rxCnt);
	}
}

#endif

static void serial_omap_rdi(struct uart_omap_port *up, unsigned int lsr)
{
	unsigned char ch = 0;
	unsigned int flag;

	if (!(lsr & UART_LSR_DR))
		return;

	ch = serial_in(up, UART_RX);
	flag = TTY_NORMAL;
	up->port.icount.rx++;

	if (uart_handle_sysrq_char(&up->port, ch))
		return;

	uart_insert_char(&up->port, lsr, UART_LSR_OE, ch, flag);
}

/**
 * serial_omap_irq() - This handles the interrupt from one port
 * @irq: uart port irq number
 * @dev_id: uart port info
 */
#ifndef EXOR_MPI
static irqreturn_t serial_omap_irq(int irq, void *dev_id)
{
	struct uart_omap_port *up = dev_id;
	unsigned int iir, lsr;
	unsigned int type;
	irqreturn_t ret = IRQ_NONE;
	int max_count = 256;

	spin_lock(&up->port.lock);
	pm_runtime_get_sync(up->dev);

	do {
		iir = serial_in(up, UART_IIR);
		if (iir & UART_IIR_NO_INT)
			break;

		ret = IRQ_HANDLED;
		lsr = serial_in(up, UART_LSR);

		/* extract IRQ type from IIR register */
		type = iir & 0x3e;

		switch (type) {
		case UART_IIR_MSI:
			check_modem_status(up);
			break;
		case UART_IIR_THRI:
			transmit_chars(up, lsr);
			break;
		case UART_IIR_RX_TIMEOUT:
			/* FALLTHROUGH */
		case UART_IIR_RDI:
			serial_omap_rdi(up, lsr);
			break;
		case UART_IIR_RLSI:
			serial_omap_rlsi(up, lsr);
			break;
		case UART_IIR_CTS_RTS_DSR:
			/* simply try again */
			break;
		case UART_IIR_XOFF:
			/* FALLTHROUGH */
		default:
			break;
		}
	} while (!(iir & UART_IIR_NO_INT) && max_count--);

	spin_unlock(&up->port.lock);

	tty_flip_buffer_push(&up->port.state->port);

	pm_runtime_mark_last_busy(up->dev);
	pm_runtime_put_autosuspend(up->dev);
	up->port_activity = jiffies;

	return ret;
}
#else 
static irqreturn_t serial_omap_irq(int irq, void *dev_id)
{
	struct uart_omap_port *up = dev_id;
	unsigned int iir, lsr;
	unsigned int type;
	unsigned int i;
	irqreturn_t ret = IRQ_NONE;
	int max_count = 256;

	spin_lock(&up->port.lock);
	pm_runtime_get_sync(up->dev);
	if (up->MPIdata.MPIenabled)	{
		do {
			iir = serial_in(up, UART_IIR);
			if (iir & UART_IIR_NO_INT)
				break;

			ret = IRQ_HANDLED;
			lsr = serial_in(up, UART_LSR);

			/* extract IRQ type from IIR register */
			type = iir & 0x3e;

			switch (type) {
			case UART_IIR_MSI:
				check_modem_status(up);
				break;
			case UART_IIR_THRI:
				transmit_chars(up, lsr);
				break;
			case UART_IIR_RX_TIMEOUT:
				i = serial_in(up, UART_RXFIFO_LVL) & 0xff;
				if ((up->MPIdata.rxCnt+i) > 1000) {
					printk("MPI: OVER UART_IIR_RX_TIMEOUT %d,%d\n", up->MPIdata.rxCnt, i);
					while (i--)
						serial_in(up, UART_RX);
					up->MPIdata.rxCnt = 0;
					break;
				}
//				while(i--)
//					up->MPIdata.mpiRxBuf[up->MPIdata.rxCnt++] = serial_in(up, UART_RX);
				while(serial_in(up, UART_RXFIFO_LVL) & 0xff)
					up->MPIdata.mpiRxBuf[up->MPIdata.rxCnt++] = serial_in(up, UART_RX);

				if (up->MPIdata.rxCnt)
					mpiReceiveFrame(&up->MPIdata, up->MPIdata.mpiRxBuf, up->MPIdata.rxCnt);

//				up->MPIdata.rxCnt = 0;
				break;
			case UART_IIR_RDI:
				i = (serial_in(up, UART_RXFIFO_LVL) & 0xff);
				if (i == 0)
					break;
				i--;
				if ((up->MPIdata.rxCnt+i) > 1000) {
					printk("MPI: UART_IIR_RDI %d,%d\n", up->MPIdata.rxCnt, i);
					while (i--)
						serial_in(up, UART_RX);
					up->MPIdata.rxCnt = 0;
					break;
				}
				while(i--)
					up->MPIdata.mpiRxBuf[up->MPIdata.rxCnt++] = serial_in(up, UART_RX);

				if (up->MPIdata.UltiUart1_TxTimeout != EV__NULL && up->MPIdata.UltiUart1_TxTimeout != EV_TOK_RUN){
					UltiUART1_StartTimer(&up->MPIdata, up->MPIdata.UltiUart1_TxTimeout, up->MPIdata.UltiUart1_TxNunUSec);
				}
				break;
			case UART_IIR_RLSI:
				serial_omap_rlsi(up, lsr);
				break;
			case UART_IIR_CTS_RTS_DSR:
				/* simply try again */
				break;
			case UART_IIR_XOFF:
				/* FALLTHROUGH */
			default:
				break;
			}
		} while (!(iir & UART_IIR_NO_INT) && max_count--);
	}
	else
	{	
		do {
			iir = serial_in(up, UART_IIR);
			if (iir & UART_IIR_NO_INT)
				break;

			ret = IRQ_HANDLED;
			lsr = serial_in(up, UART_LSR);

			/* extract IRQ type from IIR register */
			type = iir & 0x3e;

			switch (type) {
			case UART_IIR_MSI:
				check_modem_status(up);
				break;
			case UART_IIR_THRI:
				transmit_chars(up, lsr);
				break;
			case UART_IIR_RX_TIMEOUT:
				/* FALLTHROUGH */
			case UART_IIR_RDI:
				serial_omap_rdi(up, lsr);
				break;
			case UART_IIR_RLSI:
				serial_omap_rlsi(up, lsr);
				break;
			case UART_IIR_CTS_RTS_DSR:
				/* simply try again */
				break;
			case UART_IIR_XOFF:
				/* FALLTHROUGH */
			default:
				break;
			}
		} while (!(iir & UART_IIR_NO_INT) && max_count--);
	}
	spin_unlock(&up->port.lock);

	tty_flip_buffer_push(&up->port.state->port);
	pm_runtime_mark_last_busy(up->dev);
	pm_runtime_put_autosuspend(up->dev);
	up->port_activity = jiffies;

	return ret;
}
#endif

static unsigned int serial_omap_tx_empty(struct uart_port *port)
{
	struct uart_omap_port *up = to_uart_omap_port(port);
	unsigned long flags = 0;
	unsigned int ret = 0;

	pm_runtime_get_sync(up->dev);
	dev_dbg(up->port.dev, "serial_omap_tx_empty+%d\n", up->port.line);
	spin_lock_irqsave(&up->port.lock, flags);
	ret = serial_in(up, UART_LSR) & UART_LSR_TEMT ? TIOCSER_TEMT : 0;
	spin_unlock_irqrestore(&up->port.lock, flags);
	pm_runtime_mark_last_busy(up->dev);
	pm_runtime_put_autosuspend(up->dev);
	return ret;
}

static unsigned int serial_omap_get_mctrl(struct uart_port *port)
{
	struct uart_omap_port *up = to_uart_omap_port(port);
	unsigned int status;
	unsigned int ret = 0;

	pm_runtime_get_sync(up->dev);
	status = check_modem_status(up);
	pm_runtime_mark_last_busy(up->dev);
	pm_runtime_put_autosuspend(up->dev);

	dev_dbg(up->port.dev, "serial_omap_get_mctrl+%d\n", up->port.line);

	if (status & UART_MSR_DCD)
		ret |= TIOCM_CAR;
	if (status & UART_MSR_RI)
		ret |= TIOCM_RNG;
	if (status & UART_MSR_DSR)
		ret |= TIOCM_DSR;
	if (status & UART_MSR_CTS)
		ret |= TIOCM_CTS;
	return ret;
}

static void serial_omap_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_omap_port *up = to_uart_omap_port(port);
	unsigned char mcr = 0, old_mcr;

	dev_dbg(up->port.dev, "serial_omap_set_mctrl+%d\n", up->port.line);
	if (mctrl & TIOCM_RTS)
		mcr |= UART_MCR_RTS;
	if (mctrl & TIOCM_DTR)
		mcr |= UART_MCR_DTR;
	if (mctrl & TIOCM_OUT1)
		mcr |= UART_MCR_OUT1;
	if (mctrl & TIOCM_OUT2)
		mcr |= UART_MCR_OUT2;
	if (mctrl & TIOCM_LOOP)
		mcr |= UART_MCR_LOOP;

	pm_runtime_get_sync(up->dev);
	old_mcr = serial_in(up, UART_MCR);
	old_mcr &= ~(UART_MCR_LOOP | UART_MCR_OUT2 | UART_MCR_OUT1 |
		     UART_MCR_DTR | UART_MCR_RTS);
	up->mcr = old_mcr | mcr;
	serial_out(up, UART_MCR, up->mcr);
	pm_runtime_mark_last_busy(up->dev);
	pm_runtime_put_autosuspend(up->dev);

	if (gpio_is_valid(up->DTR_gpio) &&
	    !!(mctrl & TIOCM_DTR) != up->DTR_active) {
		up->DTR_active = !up->DTR_active;
		if (gpio_cansleep(up->DTR_gpio))
			schedule_work(&up->qos_work);
		else
			gpio_set_value(up->DTR_gpio,
				       up->DTR_active != up->DTR_inverted);
	}
}

static void serial_omap_break_ctl(struct uart_port *port, int break_state)
{
	struct uart_omap_port *up = to_uart_omap_port(port);
	unsigned long flags = 0;

	dev_dbg(up->port.dev, "serial_omap_break_ctl+%d\n", up->port.line);
	pm_runtime_get_sync(up->dev);
	spin_lock_irqsave(&up->port.lock, flags);
	if (break_state == -1)
		up->lcr |= UART_LCR_SBC;
	else
		up->lcr &= ~UART_LCR_SBC;
	serial_out(up, UART_LCR, up->lcr);
	spin_unlock_irqrestore(&up->port.lock, flags);
	pm_runtime_mark_last_busy(up->dev);
	pm_runtime_put_autosuspend(up->dev);
}

static int serial_omap_startup(struct uart_port *port)
{
	struct uart_omap_port *up = to_uart_omap_port(port);
	unsigned long flags = 0;
	int retval;

	/*
	 * Allocate the IRQ
	 */
	retval = request_irq(up->port.irq, serial_omap_irq, up->port.irqflags, up->name, up);
	if (retval)
		return retval;

	dev_dbg(up->port.dev, "serial_omap_startup+%d\n", up->port.line);

	pm_runtime_get_sync(up->dev);
	/*
	 * Clear the FIFO buffers and disable them.
	 * (they will be reenabled in set_termios())
	 */
	serial_omap_clear_fifos(up);
	/* For Hardware flow control */
	serial_out(up, UART_MCR, UART_MCR_RTS);

	/*
	 * Clear the interrupt registers.
	 */
	(void) serial_in(up, UART_LSR);
	if (serial_in(up, UART_LSR) & UART_LSR_DR)
		(void) serial_in(up, UART_RX);
	(void) serial_in(up, UART_IIR);
	(void) serial_in(up, UART_MSR);

	/*
	 * Now, initialize the UART
	 */
	serial_out(up, UART_LCR, UART_LCR_WLEN8);
	spin_lock_irqsave(&up->port.lock, flags);
	/*
	 * Most PC uarts need OUT2 raised to enable interrupts.
	 */
	up->port.mctrl |= TIOCM_OUT2;
	serial_omap_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);

	up->msr_saved_flags = 0;
	/*
	 * Finally, enable interrupts. Note: Modem status interrupts
	 * are set via set_termios(), which will be occurring imminently
	 * anyway, so we don't enable them here.
	 */
	up->ier = UART_IER_RLSI | UART_IER_RDI;
	serial_out(up, UART_IER, up->ier);

	/* Enable module level wake up */
	up->wer = OMAP_UART_WER_MOD_WKUP;
	if (up->features & OMAP_UART_WER_HAS_TX_WAKEUP)
		up->wer |= OMAP_UART_TX_WAKEUP_EN;

	serial_out(up, UART_OMAP_WER, up->wer);

	pm_runtime_mark_last_busy(up->dev);
	pm_runtime_put_autosuspend(up->dev);
	up->port_activity = jiffies;
#ifdef EXOR_MPI
	up->MPIdata.MPIenabled = false;
#endif
	return 0;
}

static void serial_omap_shutdown(struct uart_port *port)
{
	struct uart_omap_port *up = to_uart_omap_port(port);
	unsigned long flags = 0;

	dev_dbg(up->port.dev, "serial_omap_shutdown+%d\n", up->port.line);

	pm_runtime_get_sync(up->dev);
	/*
	 * Disable interrupts from this port
	 */
	up->ier = 0;
	serial_out(up, UART_IER, 0);

	spin_lock_irqsave(&up->port.lock, flags);
	up->port.mctrl &= ~TIOCM_OUT2;
	serial_omap_set_mctrl(&up->port, up->port.mctrl);
	spin_unlock_irqrestore(&up->port.lock, flags);

	/*
	 * Disable break condition and FIFOs
	 */
	serial_out(up, UART_LCR, serial_in(up, UART_LCR) & ~UART_LCR_SBC);
	serial_omap_clear_fifos(up);

	/*
	 * Read data port to reset things, and then free the irq
	 */
	if (serial_in(up, UART_LSR) & UART_LSR_DR)
		(void) serial_in(up, UART_RX);

	pm_runtime_mark_last_busy(up->dev);
	pm_runtime_put_autosuspend(up->dev);
	free_irq(up->port.irq, up);
	if(gpio_is_valid(up->rts_gpio)) {
		gpio_set_value(up->rts_gpio, 0);
	}
#ifdef EXOR_MPI
	hrtimer_try_to_cancel(&up->MPIdata.hrt);
	up->MPIdata.m_isOpen = false;
	up->MPIdata.MPIenabled = false;
	up->MPIdata.MPImode = false;
#endif
}



static void serial_omap_uart_qos_work(struct work_struct *work)
{
	struct uart_omap_port *up = container_of(work, struct uart_omap_port,
						qos_work);

	pm_qos_update_request(&up->pm_qos_request, up->latency);
	if (gpio_is_valid(up->DTR_gpio))
		gpio_set_value_cansleep(up->DTR_gpio,
					up->DTR_active != up->DTR_inverted);
}

static void serial_omap_set_termios(struct uart_port *port, struct ktermios *termios,
			struct ktermios *old)
{
	struct uart_omap_port *up = to_uart_omap_port(port);
	unsigned char cval = 0;
	unsigned long flags = 0;
	unsigned int baud, quot;

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		cval = UART_LCR_WLEN5;
		break;
	case CS6:
		cval = UART_LCR_WLEN6;
		break;
	case CS7:
		cval = UART_LCR_WLEN7;
		break;
	default:
	case CS8:
		cval = UART_LCR_WLEN8;
		break;
	}

	if (termios->c_cflag & CSTOPB)
		cval |= UART_LCR_STOP;
	if (termios->c_cflag & PARENB)
		cval |= UART_LCR_PARITY;
	if (!(termios->c_cflag & PARODD))
		cval |= UART_LCR_EPAR;
	if (termios->c_cflag & CMSPAR)
		cval |= UART_LCR_SPAR;

	/*
	 * Ask the core to calculate the divisor for us.
	 */

	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk/13);
	quot = serial_omap_get_divisor(port, baud);

	/* calculate wakeup latency constraint */
	up->calc_latency = (USEC_PER_SEC * up->port.fifosize) / (baud / 8);
	up->latency = up->calc_latency;
	schedule_work(&up->qos_work);

	up->dll = quot & 0xff;
	up->dlh = quot >> 8;
	up->mdr1 = UART_OMAP_MDR1_DISABLE;

	up->fcr = UART_FCR_R_TRIG_01 | UART_FCR_T_TRIG_01 |
			UART_FCR_ENABLE_FIFO;

	/*
	 * Ok, we're now changing the port state. Do it with
	 * interrupts disabled.
	 */
	pm_runtime_get_sync(up->dev);
	spin_lock_irqsave(&up->port.lock, flags);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	up->port.read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if (termios->c_iflag & INPCK)
		up->port.read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		up->port.read_status_mask |= UART_LSR_BI;

	/*
	 * Characters to ignore
	 */
	up->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		up->port.ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	if (termios->c_iflag & IGNBRK) {
		up->port.ignore_status_mask |= UART_LSR_BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			up->port.ignore_status_mask |= UART_LSR_OE;
	}

	/*
	 * ignore all characters if CREAD is not set
	 */
	if ((termios->c_cflag & CREAD) == 0)
		up->port.ignore_status_mask |= UART_LSR_DR;

	/*
	 * Modem status interrupts
	 */
	up->ier &= ~UART_IER_MSI;
	if (UART_ENABLE_MS(&up->port, termios->c_cflag))
		up->ier |= UART_IER_MSI;
	serial_out(up, UART_IER, up->ier);
	serial_out(up, UART_LCR, cval);		/* reset DLAB */
	up->lcr = cval;
	up->scr = 0;

	/* FIFOs and DMA Settings */

	/* FCR can be changed only when the
	 * baud clock is not running
	 * DLL_REG and DLH_REG set to 0.
	 */
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_A);
	serial_out(up, UART_DLL, 0);
	serial_out(up, UART_DLM, 0);
	serial_out(up, UART_LCR, 0);

	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);

	up->efr = serial_in(up, UART_EFR) & ~UART_EFR_ECB;
	up->efr &= ~UART_EFR_SCD;
	serial_out(up, UART_EFR, up->efr | UART_EFR_ECB);

	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_A);
	up->mcr = serial_in(up, UART_MCR) & ~UART_MCR_TCRTLR;
	serial_out(up, UART_MCR, up->mcr | UART_MCR_TCRTLR);
	/* FIFO ENABLE, DMA MODE */

	up->scr |= OMAP_UART_SCR_RX_TRIG_GRANU1_MASK;
	/*
	 * NOTE: Setting OMAP_UART_SCR_RX_TRIG_GRANU1_MASK
	 * sets Enables the granularity of 1 for TRIGGER RX
	 * level. Along with setting RX FIFO trigger level
	 * to 1 (as noted below, 16 characters) and TLR[3:0]
	 * to zero this will result RX FIFO threshold level
	 * to 1 character, instead of 16 as noted in comment
	 * below.
	 */

	/* Set receive FIFO threshold to 16 characters and
	 * transmit FIFO threshold to 16 spaces
	 */
	up->fcr &= ~OMAP_UART_FCR_RX_FIFO_TRIG_MASK;
	up->fcr &= ~OMAP_UART_FCR_TX_FIFO_TRIG_MASK;
	up->fcr |= UART_FCR6_R_TRIGGER_16 | UART_FCR6_T_TRIGGER_24 | UART_FCR_ENABLE_FIFO;	//rxFIFOth=1 txFIFOth=56
#ifdef EXOR_MPI
	setRxFifoThrForMPI(up, (up->MPIdata.MPImode)?true:false);
#endif
	serial_out(up, UART_FCR, up->fcr);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);

	serial_out(up, UART_OMAP_SCR, up->scr);

	/* Reset UART_MCR_TCRTLR: this must be done with the EFR_ECB bit set */
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_A);
	serial_out(up, UART_MCR, up->mcr);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);
	serial_out(up, UART_EFR, up->efr);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_A);

	/* Protocol, Baud Rate, and Interrupt Settings */

	if (up->errata & UART_ERRATA_i202_MDR1_ACCESS)
		serial_omap_mdr1_errataset(up, up->mdr1);
	else
		serial_out(up, UART_OMAP_MDR1, up->mdr1);

	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);
	serial_out(up, UART_EFR, up->efr | UART_EFR_ECB);

	serial_out(up, UART_LCR, 0);
	serial_out(up, UART_IER, 0);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);

	serial_out(up, UART_DLL, up->dll);	/* LS of divisor */
	serial_out(up, UART_DLM, up->dlh);	/* MS of divisor */

	serial_out(up, UART_LCR, 0);
	serial_out(up, UART_IER, up->ier);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);

	serial_out(up, UART_EFR, up->efr);
	serial_out(up, UART_LCR, cval);

	if (!serial_omap_baud_is_mode16(port, baud))
		up->mdr1 = UART_OMAP_MDR1_13X_MODE;
	else
		up->mdr1 = UART_OMAP_MDR1_16X_MODE;

	if (up->errata & UART_ERRATA_i202_MDR1_ACCESS)
		serial_omap_mdr1_errataset(up, up->mdr1);
	else
		serial_out(up, UART_OMAP_MDR1, up->mdr1);

	/* Configure flow control */
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);

	/* XON1/XOFF1 accessible mode B, TCRTLR=0, ECB=0 */
	serial_out(up, UART_XON1, termios->c_cc[VSTART]);
	serial_out(up, UART_XOFF1, termios->c_cc[VSTOP]);

	/* Enable access to TCR/TLR */
	serial_out(up, UART_EFR, up->efr | UART_EFR_ECB);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_A);
	serial_out(up, UART_MCR, up->mcr | UART_MCR_TCRTLR);

	serial_out(up, UART_TI752_TCR, OMAP_UART_TCR_TRIG);

	if (termios->c_cflag & CRTSCTS && up->port.flags & UPF_HARD_FLOW) {
		/* Enable AUTORTS and AUTOCTS */
		up->efr |= UART_EFR_CTS | UART_EFR_RTS;

		/* Ensure MCR RTS is asserted */
		up->mcr |= UART_MCR_RTS;
	} else {
		/* Disable AUTORTS and AUTOCTS */
		up->efr &= ~(UART_EFR_CTS | UART_EFR_RTS);
	}

	if (up->port.flags & UPF_SOFT_FLOW) {
		/* clear SW control mode bits */
		up->efr &= OMAP_UART_SW_CLR;

		/*
		 * IXON Flag:
		 * Enable XON/XOFF flow control on input.
		 * Receiver compares XON1, XOFF1.
		 */
		if (termios->c_iflag & IXON)
			up->efr |= OMAP_UART_SW_RX;

		/*
		 * IXOFF Flag:
		 * Enable XON/XOFF flow control on output.
		 * Transmit XON1, XOFF1
		 */
		if (termios->c_iflag & IXOFF)
			up->efr |= OMAP_UART_SW_TX;

		/*
		 * IXANY Flag:
		 * Enable any character to restart output.
		 * Operation resumes after receiving any
		 * character after recognition of the XOFF character
		 */
		if (termios->c_iflag & IXANY)
			up->mcr |= UART_MCR_XONANY;
		else
			up->mcr &= ~UART_MCR_XONANY;
	}
	serial_out(up, UART_MCR, up->mcr);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);
	serial_out(up, UART_EFR, up->efr);
	serial_out(up, UART_LCR, up->lcr);

	serial_omap_set_mctrl(&up->port, up->port.mctrl);
	
	if (up->rs485.flags & SER_RS485_ENABLED)
	  if (!gpio_is_valid(up->rts_gpio))
	  {
	    //Enabled RS485/422 mode, but no rts_gpio pin available, use RTS native pin
	    unsigned char tmpmcr;
	    tmpmcr = serial_in(up, UART_MCR);
	    tmpmcr &= ~UART_MCR_RTS;
	    serial_out(up, UART_MCR, tmpmcr);
	  }

	spin_unlock_irqrestore(&up->port.lock, flags);
	pm_runtime_mark_last_busy(up->dev);
	pm_runtime_put_autosuspend(up->dev);
	dev_dbg(up->port.dev, "serial_omap_set_termios+%d\n", up->port.line);
}

static int serial_omap_set_wake(struct uart_port *port, unsigned int state)
{
	struct uart_omap_port *up = to_uart_omap_port(port);

	serial_omap_enable_wakeup(up, state);

	return 0;
}

static void serial_omap_pm(struct uart_port *port, unsigned int state,
	       unsigned int oldstate)
{
	struct uart_omap_port *up = to_uart_omap_port(port);
	unsigned char efr;

	dev_dbg(up->port.dev, "serial_omap_pm+%d\n", up->port.line);

	pm_runtime_get_sync(up->dev);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);
	efr = serial_in(up, UART_EFR);
	serial_out(up, UART_EFR, efr | UART_EFR_ECB);
	serial_out(up, UART_LCR, 0);

	serial_out(up, UART_IER, (state != 0) ? UART_IERX_SLEEP : 0);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B);
	serial_out(up, UART_EFR, efr);
	serial_out(up, UART_LCR, 0);

	if (!device_may_wakeup(up->dev)) {
		if (!state)
			pm_runtime_forbid(up->dev);
		else
			pm_runtime_allow(up->dev);
	}

	pm_runtime_mark_last_busy(up->dev);
	pm_runtime_put_autosuspend(up->dev);
}

static void serial_omap_release_port(struct uart_port *port)
{
	dev_dbg(port->dev, "serial_omap_release_port+\n");
}

static int serial_omap_request_port(struct uart_port *port)
{
	dev_dbg(port->dev, "serial_omap_request_port+\n");
	return 0;
}

static void serial_omap_config_port(struct uart_port *port, int flags)
{
	struct uart_omap_port *up = to_uart_omap_port(port);

	dev_dbg(up->port.dev, "serial_omap_config_port+%d\n",
							up->port.line);
	up->port.type = PORT_OMAP;
	up->port.flags |= UPF_SOFT_FLOW | UPF_HARD_FLOW;
}

static int
serial_omap_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	/* we don't want the core code to modify any port params */
	dev_dbg(port->dev, "serial_omap_verify_port+\n");
	return -EINVAL;
}

static const char *serial_omap_type(struct uart_port *port)
{
	struct uart_omap_port *up = to_uart_omap_port(port);

	dev_dbg(up->port.dev, "serial_omap_type+%d\n", up->port.line);
	return up->name;
}

#define BOTH_EMPTY (UART_LSR_TEMT | UART_LSR_THRE)

static inline void wait_for_xmitr(struct uart_omap_port *up)
{
	unsigned int status, tmout = 10000;

	/* Wait up to 10ms for the character(s) to be sent. */
	do {
		status = serial_in(up, UART_LSR);

		if (status & UART_LSR_BI)
			up->lsr_break_flag = UART_LSR_BI;

		if (--tmout == 0)
			break;
		udelay(1);
	} while ((status & BOTH_EMPTY) != BOTH_EMPTY);

	/* Wait up to 1s for flow control if necessary */
	if (up->port.flags & UPF_CONS_FLOW) {
		tmout = 1000000;
		for (tmout = 1000000; tmout; tmout--) {
			unsigned int msr = serial_in(up, UART_MSR);

			up->msr_saved_flags |= msr & MSR_SAVE_FLAGS;
			if (msr & UART_MSR_CTS)
				break;

			udelay(1);
		}
	}
}

#ifdef CONFIG_CONSOLE_POLL

static void serial_omap_poll_put_char(struct uart_port *port, unsigned char ch)
{
	struct uart_omap_port *up = to_uart_omap_port(port);

	pm_runtime_get_sync(up->dev);
	wait_for_xmitr(up);
	serial_out(up, UART_TX, ch);
	pm_runtime_mark_last_busy(up->dev);
	pm_runtime_put_autosuspend(up->dev);
}

static int serial_omap_poll_get_char(struct uart_port *port)
{
	struct uart_omap_port *up = to_uart_omap_port(port);
	unsigned int status;

	pm_runtime_get_sync(up->dev);
	status = serial_in(up, UART_LSR);
	if (!(status & UART_LSR_DR)) {
		status = NO_POLL_CHAR;
		goto out;
	}

	status = serial_in(up, UART_RX);

out:
	pm_runtime_mark_last_busy(up->dev);
	pm_runtime_put_autosuspend(up->dev);

	return status;
}

#endif /* CONFIG_CONSOLE_POLL */

#ifdef CONFIG_SERIAL_OMAP_CONSOLE

static struct uart_omap_port *serial_omap_console_ports[OMAP_MAX_HSUART_PORTS];

static struct uart_driver serial_omap_reg;

static void serial_omap_console_putchar(struct uart_port *port, int ch)
{
	struct uart_omap_port *up = to_uart_omap_port(port);

	wait_for_xmitr(up);
	serial_out(up, UART_TX, ch);
}

static void serial_omap_console_write(struct console *co, const char *s,
		unsigned int count)
{
	struct uart_omap_port *up = serial_omap_console_ports[co->index];
	unsigned long flags;
	unsigned int ier;
	int locked = 1;

	pm_runtime_get_sync(up->dev);

	if (up->port.sysrq || oops_in_progress)
		locked = spin_trylock_irqsave(&up->port.lock, flags);
	else
		spin_lock_irqsave(&up->port.lock, flags);

	/*
	 * First save the IER then disable the interrupts
	 */
	ier = serial_in(up, UART_IER);
	serial_out(up, UART_IER, 0);

	uart_console_write(&up->port, s, count, serial_omap_console_putchar);

	/*
	 * Finally, wait for transmitter to become empty
	 * and restore the IER
	 */
	wait_for_xmitr(up);
	serial_out(up, UART_IER, ier);
	/*
	 * The receive handling will happen properly because the
	 * receive ready bit will still be set; it is not cleared
	 * on read.  However, modem control will not, we must
	 * call it if we have saved something in the saved flags
	 * while processing with interrupts off.
	 */
	if (up->msr_saved_flags)
		check_modem_status(up);

	pm_runtime_mark_last_busy(up->dev);
	pm_runtime_put_autosuspend(up->dev);
	if (locked)
		spin_unlock_irqrestore(&up->port.lock, flags);
}

static int __init
serial_omap_console_setup(struct console *co, char *options)
{
	struct uart_omap_port *up;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (serial_omap_console_ports[co->index] == NULL)
		return -ENODEV;
	up = serial_omap_console_ports[co->index];

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(&up->port, co, baud, parity, bits, flow);
}

static struct console serial_omap_console = {
	.name		= OMAP_SERIAL_NAME,
	.write		= serial_omap_console_write,
	.device		= uart_console_device,
	.setup		= serial_omap_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &serial_omap_reg,
};

static void serial_omap_add_console_port(struct uart_omap_port *up)
{
	serial_omap_console_ports[up->port.line] = up;
}

#define OMAP_CONSOLE	(&serial_omap_console)

#else

#define OMAP_CONSOLE	NULL

static inline void serial_omap_add_console_port(struct uart_omap_port *up)
{}

#endif

/* Enable or disable the rs485 support */
static void serial_omap_config_rs485(struct uart_port *port, struct serial_rs485 *rs485conf)
{
	struct uart_omap_port *up = to_uart_omap_port(port);
	unsigned long flags;
	unsigned int mode;
	int val;

	pm_runtime_get_sync(up->dev);
	spin_lock_irqsave(&up->port.lock, flags);

	/* Disable interrupts from this port */
	mode = up->ier;
	up->ier = 0;
	serial_out(up, UART_IER, 0);

	/* store new config */
	up->rs485 = *rs485conf;

	/*
	 * Set initial value of tx_enable pin
	 * 
	 */
	if (gpio_is_valid(up->rts_gpio)) {
		/* enable / disable rts_gpio pin */
		val = (up->rs485.flags & SER_RS485_ENABLED) ?
			SER_RS485_RTS_AFTER_SEND : SER_RS485_RTS_ON_SEND;
		val = (up->rs485.flags & val) ? 1 : 0;
		gpio_set_value(up->rts_gpio, val);
	} 
	else if (up->rs485.flags & SER_RS485_ENABLED)
	{
	  //Enabled RS485/422 mode, but no rts_gpio pin available, use RTS native pin
	  unsigned char tmpmcr;
	  tmpmcr = serial_in(up, UART_MCR);
	  tmpmcr &= ~UART_MCR_RTS;
	  serial_out(up, UART_MCR, tmpmcr);
	}
	
	/*
	 * If we have a programmable phy, set the mode accordingly
	 */
	if (gpio_is_valid(up->mode_gpio)) 
	{
	  if(up->rs485.flags & SER_RS485_ENABLED)
	    gpio_set_value(up->mode_gpio, 1);
	  else
	    gpio_set_value(up->mode_gpio, 0);
	}
	
	//RX enable by using the prg phy dedicated gpio pin
	if (gpio_is_valid(up->rxen_gpio)) 
	  gpio_set_value(up->rxen_gpio, 1);
	
	/* Enable interrupts */
	up->ier = mode;
	serial_out(up, UART_IER, up->ier);

	/* If RS-485 is disabled, make sure the THR interrupt is fired when
	 * TX FIFO is below the trigger level.
	 */
	if (!(up->rs485.flags & SER_RS485_ENABLED) &&
	    (up->scr & OMAP_UART_SCR_TX_EMPTY)) {
		up->scr &= ~OMAP_UART_SCR_TX_EMPTY;
		serial_out(up, UART_OMAP_SCR, up->scr);
	}
	
	spin_unlock_irqrestore(&up->port.lock, flags);
	pm_runtime_mark_last_busy(up->dev);
	pm_runtime_put_autosuspend(up->dev);
}

#ifdef EXOR_MPI
static int serial_omap_ioctl_mpi(struct uart_port *port, unsigned int cmd, unsigned long arg)
{
	struct uart_omap_port *up = to_uart_omap_port(port);
	unsigned long flags = 0;
	int ret = 0;
	switch (cmd) {
		case SET_MPI_MODE:
			spin_lock_irqsave(&up->port.lock, flags);
			up->MPIdata.mpiUp = up;
			MPIDriverInit(&up->MPIdata);
			spin_unlock_irqrestore(&up->port.lock, flags);
			break;
		case MPI_OPEN:
			{
				struct s_MPIparams init;
				if (copy_from_user(&init, (struct s_MPIparams *) arg, sizeof(init)))
					ret = -EFAULT;
				else {
					spin_lock_irqsave(&up->port.lock, flags);
					MPIDriverOpen(&up->MPIdata, init);
					spin_unlock_irqrestore(&up->port.lock, flags);
				}
			}
			break;

		case MPI_RUN:
			{
				spin_lock_irqsave(&up->port.lock, flags);
				ev_move(&up->MPIdata, EV_TOK_RUN);					//do it in run
				spin_unlock_irqrestore(&up->port.lock, flags);
			}
			break;

		case MPI_IS_OPEN:
			if (copy_to_user((unsigned int *)arg, &up->MPIdata.m_isOpen, sizeof(up->MPIdata.m_isOpen)))
				ret = -EFAULT;
			break;

		case GET_MPI_DIAG:
//			if (copy_to_user((unsigned int *)arg, &(sport->SCNKdata.diag_cnt), sizeof(sport->SCNKdata.diag_cnt)))
				ret = -EFAULT;
			break;

		case SET_MPI_DIAG:
			{
				unsigned int tmp[2];
				if (copy_from_user(tmp, (unsigned int*) arg, sizeof(tmp)))
					ret = -EFAULT;
			}
			break;

		case MPI_CLOSE:
			spin_lock_irqsave(&up->port.lock, flags);
			mExitFromRing(&up->MPIdata);
			UltiUart1_StopTimer(&up->MPIdata);
			usleep_range(10000,11000);
			up->MPIdata.m_isOpen = false;
			up->MPIdata.MPIenabled = false;
			up->MPIdata.MPImode = false;
			spin_unlock_irqrestore(&up->port.lock, flags);
			break;

		case SET_MPI_DATA:
			{
				unsigned int tmp;
				if (copy_from_user(&tmp, (unsigned int *) arg, sizeof(tmp)))
					ret = -EFAULT;
				else {
					up->MPIdata.FrameJobSessionLen = tmp;
					if (copy_from_user(up->MPIdata.FrameJobSessionBuff, (unsigned char*) arg + sizeof(tmp), tmp))
						ret = -EFAULT;
				}
			}
			break;

		case SET_MPI_REQ:
			{
				int tmp;
				if (copy_from_user(&tmp, (int *) arg, sizeof(tmp)))
					ret = -EFAULT;
				else {
					spin_lock_irqsave(&up->port.lock, flags);
					up->MPIdata.ProcedureApplMRequestPending = tmp;
					up->MPIdata.ProcedureApplMResponsePending = tmp;
					up->MPIdata.ProcedureApplMStatus = M_PROC_RUNNING;
					up->MPIdata.applTryCnt = up->MPIdata.applResponseTime / 5; //init timeout counting
					spin_unlock_irqrestore(&up->port.lock, flags);
				}
			}
			break;
		case GET_MPI_RQST:
			{
				unsigned char plcIndex;
				if (copy_from_user(&plcIndex, (unsigned char *) arg, sizeof(plcIndex)))
					ret = -EFAULT;
				else {
					byte tmp;
					spin_lock_irqsave(&up->port.lock, flags);
					if (up->MPIdata.ProcedureApplMRequestPending)
					{
						// Check whether there really was no traffic on the line at all !!!!!
						if (up->MPIdata.AreWeInRing && up->MPIdata.StationStatus[plcIndex].IsActive &&
							up->MPIdata.StationStatus[plcIndex].StationType >= 2)
						{
							if (up->MPIdata.SlaveSession == false)
							{
								if (!up->MPIdata.StationStatus[plcIndex].Logged)
								{
									if (up->MPIdata.StationStatus[plcIndex].LogStatus == 0)
									{
										up->MPIdata.LogOn_DA = plcIndex;
										ev_post(&up->MPIdata, EV_LOG_START);
										up->MPIdata.StationStatus[plcIndex].LogStatus = 1;
									}
								}
								else
								{
									up->MPIdata.Last_DA = plcIndex;
									if (up->MPIdata.StationStatus[up->MPIdata.Last_DA].Job == 0)
									{
										if (up->MPIdata.StationStatus[plcIndex].LogStatus == 1)
										{
											ev_post(&up->MPIdata, EV_JB0_SEND_REQ);
											up->MPIdata.StationStatus[plcIndex].LogStatus = 2;
										}
									}
									else
									{
										ev_post(&up->MPIdata, EV_JOB_SEND_REQ);
										up->MPIdata.ProcedureApplMRequestPending = false;
									}
								}
							}
						}
					}

					/* Comm sequence is over when ProcedureMResponsePending == false     */
					if (up->MPIdata.ProcedureApplMResponsePending)
					{
						// check application timeout
						if (--up->MPIdata.applTryCnt == 0)
						{
							up->MPIdata.ProcedureApplMStatus = TIMEOUT_ERR;
							DBGReg(239, up->MPIdata.ProcedureApplMStatus, 0x0006);
							up->MPIdata.ProcedureApplMRequestPending = false;
							up->MPIdata.ProcedureApplMResponsePending = false;
							ev_post(&up->MPIdata, EV_BRC_APPL_TIMEOUT);
						}
					}

					if (NO_ERROR == up->MPIdata.ProcedureApplMStatus) {
						up->MPIdata.ProcedureApplMStatus = M_PROC_OK;
					}
					tmp = up->MPIdata.ProcedureApplMStatus;
					spin_unlock_irqrestore(&up->port.lock, flags);
					if (copy_to_user((unsigned char *)arg, &tmp, sizeof(up->MPIdata.ProcedureApplMStatus)))
						ret = -EFAULT;
				}
			}
			break;
		case GET_MPI_RESP:
			{
				unsigned int len = up->MPIdata.MyFrameResponseBuffLen;
				if (copy_to_user((unsigned char *)arg, &len, sizeof(len)))
					ret = -EFAULT;
				else if (copy_to_user((unsigned char *)arg+sizeof(len), &up->MPIdata.MyFrameResponseBuff, len))
					ret = -EFAULT;
			}
			break;
		default:
			ret = -ENOIOCTLCMD;
			break;
	}
	return ret;
}
#endif
static int serial_omap_ioctl(struct uart_port *port, unsigned int cmd, unsigned long arg)
{
	struct serial_rs485 rs485conf;
	struct uart_omap_port *up = to_uart_omap_port(port);
	
	switch (cmd) {
	case TIOCSRS485:
		if (copy_from_user(&rs485conf, (struct serial_rs485 *) arg,
					sizeof(rs485conf)))
			return -EFAULT;

		serial_omap_config_rs485(port, &rs485conf);

		//Set duplex mode for RS485/422 plugin modules, according with RS485 or RS422 mode set
		if (up->rs485.flags & SER_RS485_ENABLED)
		    if (!gpio_is_valid(up->rxen_gpio)) 
		    {
		      unsigned int cmd;
		      
		      if(up->rs485.flags & SER_RS485_RX_DURING_TX)
			cmd = RS422_485_IF_SETFD;
		      else
			cmd = RS422_485_IF_SETHD;
		      if(up->plugin1dev) 
			plxx_manager_sendcmd(up->plugin1dev , cmd); 
		      if(up->plugin2dev) 
			plxx_manager_sendcmd(up->plugin2dev , cmd); 
		    } 
		break;

	case TIOCGRS485:
		if (copy_to_user((struct serial_rs485 *) arg,
					&(to_uart_omap_port(port)->rs485),
					sizeof(rs485conf)))
			return -EFAULT;
		break;

	default:
#ifdef EXOR_MPI
		return serial_omap_ioctl_mpi(port, cmd, arg);
#else
		return -ENOIOCTLCMD;
#endif
	}
	return 0;
}


static struct uart_ops serial_omap_pops = {
	.tx_empty	= serial_omap_tx_empty,
	.set_mctrl	= serial_omap_set_mctrl,
	.get_mctrl	= serial_omap_get_mctrl,
	.stop_tx	= serial_omap_stop_tx,
	.start_tx	= serial_omap_start_tx,
	.throttle	= serial_omap_throttle,
	.unthrottle	= serial_omap_unthrottle,
	.stop_rx	= serial_omap_stop_rx,
	.enable_ms	= serial_omap_enable_ms,
	.break_ctl	= serial_omap_break_ctl,
	.startup	= serial_omap_startup,
	.shutdown	= serial_omap_shutdown,
	.set_termios	= serial_omap_set_termios,
	.pm		= serial_omap_pm,
	.set_wake	= serial_omap_set_wake,
	.type		= serial_omap_type,
	.release_port	= serial_omap_release_port,
	.request_port	= serial_omap_request_port,
	.config_port	= serial_omap_config_port,
	.verify_port	= serial_omap_verify_port,
	.ioctl		= serial_omap_ioctl,
#ifdef CONFIG_CONSOLE_POLL
	.poll_put_char  = serial_omap_poll_put_char,
	.poll_get_char  = serial_omap_poll_get_char,
#endif
};

static struct uart_driver serial_omap_reg = {
	.owner		= THIS_MODULE,
	.driver_name	= "OMAP-SERIAL",
	.dev_name	= OMAP_SERIAL_NAME,
	.nr		= OMAP_MAX_HSUART_PORTS,
	.cons		= OMAP_CONSOLE,
};

#ifdef CONFIG_PM_SLEEP
static int serial_omap_prepare(struct device *dev)
{
	struct uart_omap_port *up = dev_get_drvdata(dev);

	up->is_suspending = true;

	return 0;
}

static void serial_omap_complete(struct device *dev)
{
	struct uart_omap_port *up = dev_get_drvdata(dev);

	up->is_suspending = false;
}

static int serial_omap_suspend(struct device *dev)
{
	struct uart_omap_port *up = dev_get_drvdata(dev);

	uart_suspend_port(&serial_omap_reg, &up->port);
	flush_work(&up->qos_work);

	/* Select sleep pin state */
	pinctrl_pm_select_sleep_state(dev);

	return 0;
}

static int serial_omap_resume(struct device *dev)
{
	struct uart_omap_port *up = dev_get_drvdata(dev);

	/* Select default pin state */
	pinctrl_pm_select_default_state(dev);

	uart_resume_port(&serial_omap_reg, &up->port);

	return 0;
}
#else
#define serial_omap_prepare NULL
#define serial_omap_complete NULL
#endif /* CONFIG_PM_SLEEP */

static void omap_serial_fill_features_erratas(struct uart_omap_port *up)
{
	u32 mvr, scheme;
	u16 revision, major, minor;

	mvr = readl(up->port.membase + (UART_OMAP_MVER << up->port.regshift));

	/* Check revision register scheme */
	scheme = mvr >> OMAP_UART_MVR_SCHEME_SHIFT;

	switch (scheme) {
	case 0: /* Legacy Scheme: OMAP2/3 */
		/* MINOR_REV[0:4], MAJOR_REV[4:7] */
		major = (mvr & OMAP_UART_LEGACY_MVR_MAJ_MASK) >>
					OMAP_UART_LEGACY_MVR_MAJ_SHIFT;
		minor = (mvr & OMAP_UART_LEGACY_MVR_MIN_MASK);
		break;
	case 1:
		/* New Scheme: OMAP4+ */
		/* MINOR_REV[0:5], MAJOR_REV[8:10] */
		major = (mvr & OMAP_UART_MVR_MAJ_MASK) >>
					OMAP_UART_MVR_MAJ_SHIFT;
		minor = (mvr & OMAP_UART_MVR_MIN_MASK);
		break;
	default:
		dev_warn(up->dev,
			"Unknown %s revision, defaulting to highest\n",
			up->name);
		/* highest possible revision */
		major = 0xff;
		minor = 0xff;
	}

	/* normalize revision for the driver */
	revision = UART_BUILD_REVISION(major, minor);

	switch (revision) {
	case OMAP_UART_REV_46:
		up->errata |= (UART_ERRATA_i202_MDR1_ACCESS |
				UART_ERRATA_i291_DMA_FORCEIDLE);
		break;
	case OMAP_UART_REV_52:
		up->errata |= (UART_ERRATA_i202_MDR1_ACCESS |
				UART_ERRATA_i291_DMA_FORCEIDLE);
		up->features |= OMAP_UART_WER_HAS_TX_WAKEUP;
		break;
	case OMAP_UART_REV_63:
		up->errata |= UART_ERRATA_i202_MDR1_ACCESS;
		up->features |= OMAP_UART_WER_HAS_TX_WAKEUP;
		break;
	default:
		break;
	}
}

static struct omap_uart_port_info *of_get_uart_port_info(struct device *dev)
{
	struct omap_uart_port_info *omap_up_info;

	omap_up_info = devm_kzalloc(dev, sizeof(*omap_up_info), GFP_KERNEL);
	if (!omap_up_info)
		return NULL; /* out of memory */

	of_property_read_u32(dev->of_node, "clock-frequency",
					 &omap_up_info->uartclk);
	return omap_up_info;
}

static int serial_omap_probe_rs485(struct uart_omap_port *up,
				   struct device_node *np)
{
	struct serial_rs485 *rs485conf = &up->rs485;
	u32 rs485_delay[2];
	enum of_gpio_flags flags;
	int ret;
	struct device_node *plxxnp;

	rs485conf->flags = 0;
	up->rts_gpio = -EINVAL;
	up->mode_gpio = -EINVAL;
	up->rxen_gpio = -EINVAL;

	if (!np)
		return 0;

	if (of_property_read_bool(np, "rs485-rts-active-high"))
	{
	  rs485conf->flags |= SER_RS485_RTS_ON_SEND;
	}
	else
	  rs485conf->flags |= SER_RS485_RTS_AFTER_SEND;

	/* check for tx enable gpio */
	up->rts_gpio = of_get_named_gpio_flags(np, "rts-gpio", 0, &flags);
	if (gpio_is_valid(up->rts_gpio)) {
		ret = gpio_request(up->rts_gpio, "omap-serial");
		if (ret < 0)
			return ret;
		ret = gpio_direction_output(up->rts_gpio,
					    flags & SER_RS485_RTS_AFTER_SEND);
		if (ret < 0)
			return ret;
	} else
		up->rts_gpio = -EINVAL;
	
	/* check for mode gpio, which is used to switch from RS485 <-> RS232 on programmable phys */
	up->mode_gpio = of_get_named_gpio(np, "mode-gpio", 0);
	if (gpio_is_valid(up->mode_gpio)) 
	{
	  ret = gpio_request(up->mode_gpio, "omap-serial");
	  if (ret < 0)
	    return ret;
	  ret = gpio_direction_output(up->mode_gpio,0);
	  if (ret < 0)
	    return ret;
	} 
	else
	  up->mode_gpio = -EINVAL;

	/* check for rxen gpio, which is used to enable/disable the rx on programmable phys */
	up->rxen_gpio = of_get_named_gpio(np, "rxen-gpio", 0);
	if (gpio_is_valid(up->rxen_gpio)) 
	{
	  ret = gpio_request(up->rxen_gpio, "omap-serial");
	  if (ret < 0)
	    return ret;
	  ret = gpio_direction_output(up->rxen_gpio,0);
	  if (ret < 0)
	    return ret;
	  
	  gpio_set_value(up->rxen_gpio, 1);
	} 
	else
	  up->rxen_gpio = -EINVAL;
	
		
	if (of_property_read_u32_array(np, "rs485-rts-delay",
				    rs485_delay, 2) == 0) {
		rs485conf->delay_rts_before_send = rs485_delay[0];
		rs485conf->delay_rts_after_send = rs485_delay[1];
	}

	if (of_property_read_bool(np, "rs485-rx-during-tx"))
		rs485conf->flags |= SER_RS485_RX_DURING_TX;

	if (of_property_read_bool(np, "linux,rs485-enabled-at-boot-time"))
	{
		rs485conf->flags |= SER_RS485_ENABLED;
		if (gpio_is_valid(up->mode_gpio)) 
		  gpio_set_value(up->mode_gpio, 1);
	}
	
	/* Get handle to plugin plxx manager drivers (if any) */
	up->plugin1dev = NULL;
	up->plugin2dev = NULL;
	
	plxxnp = of_parse_phandle(np, "plugin1", 0);
	if (plxxnp) 
	{
	  up->plugin1dev = of_find_device_by_node(plxxnp);
	  of_node_put(plxxnp);
	}

	plxxnp = of_parse_phandle(np, "plugin2", 0);
	if (plxxnp) 
	{
	  up->plugin2dev = of_find_device_by_node(plxxnp);
	  of_node_put(plxxnp);
	}

	return 0;
}

static int serial_omap_probe(struct platform_device *pdev)
{
	struct uart_omap_port	*up;
	struct resource		*mem, *irq;
	struct omap_uart_port_info *omap_up_info = dev_get_platdata(&pdev->dev);
	int ret;

	if (pdev->dev.of_node) {
		omap_up_info = of_get_uart_port_info(&pdev->dev);
		pdev->dev.platform_data = omap_up_info;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "no mem resource?\n");
		return -ENODEV;
	}

	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return -ENODEV;
	}

	if (!devm_request_mem_region(&pdev->dev, mem->start, resource_size(mem),
				pdev->dev.driver->name)) {
		dev_err(&pdev->dev, "memory region already claimed\n");
		return -EBUSY;
	}

	if (gpio_is_valid(omap_up_info->DTR_gpio) &&
	    omap_up_info->DTR_present) {
		ret = gpio_request(omap_up_info->DTR_gpio, "omap-serial");
		if (ret < 0)
			return ret;
		ret = gpio_direction_output(omap_up_info->DTR_gpio,
					    omap_up_info->DTR_inverted);
		if (ret < 0)
			return ret;
	}

	up = devm_kzalloc(&pdev->dev, sizeof(*up), GFP_KERNEL);
	if (!up)
		return -ENOMEM;

	if (gpio_is_valid(omap_up_info->DTR_gpio) &&
	    omap_up_info->DTR_present) {
		up->DTR_gpio = omap_up_info->DTR_gpio;
		up->DTR_inverted = omap_up_info->DTR_inverted;
	} else
		up->DTR_gpio = -EINVAL;
	up->DTR_active = 0;

	up->dev = &pdev->dev;
	up->port.dev = &pdev->dev;
	up->port.type = PORT_OMAP;
	up->port.iotype = UPIO_MEM;
	up->port.irq = irq->start;

	up->port.regshift = 2;
	up->port.fifosize = 64;
	up->port.ops = &serial_omap_pops;

	if (pdev->dev.of_node)
		up->port.line = of_alias_get_id(pdev->dev.of_node, "serial");
	else
		up->port.line = pdev->id;

	if (up->port.line < 0) {
		dev_err(&pdev->dev, "failed to get alias/pdev id, errno %d\n",
								up->port.line);
		ret = -ENODEV;
		goto err_port_line;
	}

	ret = serial_omap_probe_rs485(up, pdev->dev.of_node);
	if (ret < 0)
		goto err_rs485;

	sprintf(up->name, "OMAP UART%d", up->port.line);
	up->port.mapbase = mem->start;
	up->port.membase = devm_ioremap(&pdev->dev, mem->start,
						resource_size(mem));
	if (!up->port.membase) {
		dev_err(&pdev->dev, "can't ioremap UART\n");
		ret = -ENOMEM;
		goto err_ioremap;
	}

	up->port.flags = omap_up_info->flags;
	up->port.uartclk = omap_up_info->uartclk;
	if (!up->port.uartclk) {
		up->port.uartclk = DEFAULT_CLK_SPEED;
		dev_warn(&pdev->dev, "No clock speed specified: using default:"
						"%d\n", DEFAULT_CLK_SPEED);
	}

	up->latency = PM_QOS_CPU_DMA_LAT_DEFAULT_VALUE;
	up->calc_latency = PM_QOS_CPU_DMA_LAT_DEFAULT_VALUE;
	pm_qos_add_request(&up->pm_qos_request,
		PM_QOS_CPU_DMA_LATENCY, up->latency);
	serial_omap_uart_wq = create_singlethread_workqueue(up->name);
	INIT_WORK(&up->qos_work, serial_omap_uart_qos_work);

	platform_set_drvdata(pdev, up);
	if (omap_up_info->autosuspend_timeout == 0)
		omap_up_info->autosuspend_timeout = -1;
	device_init_wakeup(up->dev, true);
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_set_autosuspend_delay(&pdev->dev,
			omap_up_info->autosuspend_timeout);

	pm_runtime_irq_safe(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	pm_runtime_get_sync(&pdev->dev);

	omap_serial_fill_features_erratas(up);

	ui[up->port.line] = up;
	serial_omap_add_console_port(up);

	ret = uart_add_one_port(&serial_omap_reg, &up->port);
	if (ret != 0)
		goto err_add_port;

	pm_runtime_mark_last_busy(up->dev);
	pm_runtime_put_autosuspend(up->dev);
	return 0;

err_add_port:
	pm_runtime_put(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
err_ioremap:
err_rs485:
err_port_line:
	dev_err(&pdev->dev, "[UART%d]: failure [%s]: %d\n",
				pdev->id, __func__, ret);
	return ret;
}

static int serial_omap_remove(struct platform_device *dev)
{
	struct uart_omap_port *up = platform_get_drvdata(dev);

	if (gpio_is_valid(up->mode_gpio))
		gpio_free(up->mode_gpio);
	if (gpio_is_valid(up->rts_gpio))
		gpio_free(up->rts_gpio);
	if (gpio_is_valid(up->rxen_gpio))
		gpio_free(up->rxen_gpio);
	if (gpio_is_valid(up->DTR_gpio))
		gpio_free(up->DTR_gpio);

	pm_runtime_put_sync(up->dev);
	pm_runtime_disable(up->dev);
	uart_remove_one_port(&serial_omap_reg, &up->port);
	pm_qos_remove_request(&up->pm_qos_request);

	return 0;
}

/*
 * Work Around for Errata i202 (2430, 3430, 3630, 4430 and 4460)
 * The access to uart register after MDR1 Access
 * causes UART to corrupt data.
 *
 * Need a delay =
 * 5 L4 clock cycles + 5 UART functional clock cycle (@48MHz = ~0.2uS)
 * give 10 times as much
 */
static void serial_omap_mdr1_errataset(struct uart_omap_port *up, u8 mdr1)
{
	u8 timeout = 255;

	serial_out(up, UART_OMAP_MDR1, mdr1);
	udelay(2);
	serial_out(up, UART_FCR, up->fcr | UART_FCR_CLEAR_XMIT |
			UART_FCR_CLEAR_RCVR);
	/*
	 * Wait for FIFO to empty: when empty, RX_FIFO_E bit is 0 and
	 * TX_FIFO_E bit is 1.
	 */
	while (UART_LSR_THRE != (serial_in(up, UART_LSR) &
				(UART_LSR_THRE | UART_LSR_DR))) {
		timeout--;
		if (!timeout) {
			/* Should *never* happen. we warn and carry on */
			dev_crit(up->dev, "Errata i202: timedout %x\n",
						serial_in(up, UART_LSR));
			break;
		}
		udelay(1);
	}
}

#ifdef CONFIG_PM_RUNTIME
static void serial_omap_restore_context(struct uart_omap_port *up)
{
	if (up->errata & UART_ERRATA_i202_MDR1_ACCESS)
		serial_omap_mdr1_errataset(up, UART_OMAP_MDR1_DISABLE);
	else
		serial_out(up, UART_OMAP_MDR1, UART_OMAP_MDR1_DISABLE);

	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B); /* Config B mode */
	serial_out(up, UART_EFR, UART_EFR_ECB);
	serial_out(up, UART_LCR, 0x0); /* Operational mode */
	serial_out(up, UART_IER, 0x0);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B); /* Config B mode */
	serial_out(up, UART_DLL, up->dll);
	serial_out(up, UART_DLM, up->dlh);
	serial_out(up, UART_LCR, 0x0); /* Operational mode */
	serial_out(up, UART_IER, up->ier);
	serial_out(up, UART_FCR, up->fcr);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_A);
	serial_out(up, UART_MCR, up->mcr);
	serial_out(up, UART_LCR, UART_LCR_CONF_MODE_B); /* Config B mode */
	serial_out(up, UART_OMAP_SCR, up->scr);
	serial_out(up, UART_EFR, up->efr);
	serial_out(up, UART_LCR, up->lcr);
	if (up->errata & UART_ERRATA_i202_MDR1_ACCESS)
		serial_omap_mdr1_errataset(up, up->mdr1);
	else
		serial_out(up, UART_OMAP_MDR1, up->mdr1);
	serial_out(up, UART_OMAP_WER, up->wer);
}

static int serial_omap_runtime_suspend(struct device *dev)
{
	struct uart_omap_port *up = dev_get_drvdata(dev);

	if (!up)
		return -EINVAL;

	/*
	* When using 'no_console_suspend', the console UART must not be
	* suspended. Since driver suspend is managed by runtime suspend,
	* preventing runtime suspend (by returning error) will keep device
	* active during suspend.
	*/
	if (up->is_suspending && !console_suspend_enabled &&
	    uart_console(&up->port))
		return -EBUSY;

	up->context_loss_cnt = serial_omap_get_context_loss_count(up);

	if (device_may_wakeup(dev)) {
		if (!up->wakeups_enabled) {
			serial_omap_enable_wakeup(up, true);
			up->wakeups_enabled = true;
		}
	} else {
		if (up->wakeups_enabled) {
			serial_omap_enable_wakeup(up, false);
			up->wakeups_enabled = false;
		}
	}

	up->latency = PM_QOS_CPU_DMA_LAT_DEFAULT_VALUE;
	schedule_work(&up->qos_work);

	return 0;
}

static int serial_omap_runtime_resume(struct device *dev)
{
	struct uart_omap_port *up = dev_get_drvdata(dev);

	int loss_cnt = serial_omap_get_context_loss_count(up);

	if (loss_cnt < 0) {
		dev_dbg(dev, "serial_omap_get_context_loss_count failed : %d\n",
			loss_cnt);
		serial_omap_restore_context(up);
	} else if (up->context_loss_cnt != loss_cnt) {
		serial_omap_restore_context(up);
	}
	up->latency = up->calc_latency;
	schedule_work(&up->qos_work);

	return 0;
}
#endif

static const struct dev_pm_ops serial_omap_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(serial_omap_suspend, serial_omap_resume)
	SET_RUNTIME_PM_OPS(serial_omap_runtime_suspend,
				serial_omap_runtime_resume, NULL)
	.prepare        = serial_omap_prepare,
	.complete       = serial_omap_complete,
};

#if defined(CONFIG_OF)
static const struct of_device_id omap_serial_of_match[] = {
	{ .compatible = "ti,omap2-uart" },
	{ .compatible = "ti,omap3-uart" },
	{ .compatible = "ti,omap4-uart" },
	{},
};
MODULE_DEVICE_TABLE(of, omap_serial_of_match);
#endif

static struct platform_driver serial_omap_driver = {
	.probe          = serial_omap_probe,
	.remove         = serial_omap_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.pm	= &serial_omap_dev_pm_ops,
		.of_match_table = of_match_ptr(omap_serial_of_match),
	},
};

static int __init serial_omap_init(void)
{
	int ret;

	ret = uart_register_driver(&serial_omap_reg);
	if (ret != 0)
		return ret;
	ret = platform_driver_register(&serial_omap_driver);
	if (ret != 0)
		uart_unregister_driver(&serial_omap_reg);
	return ret;
}

static void __exit serial_omap_exit(void)
{
	platform_driver_unregister(&serial_omap_driver);
	uart_unregister_driver(&serial_omap_reg);
}

module_init(serial_omap_init);
module_exit(serial_omap_exit);

MODULE_DESCRIPTION("OMAP High Speed UART driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Texas Instruments Inc");
