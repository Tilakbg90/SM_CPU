/*********************************************************************************************************************
	Project		: 	Single Section Digital Axle Counter
	Version		: 	2.0 
	Revision	:	1	
	Filename	: 	comm_smc.h
	Target MCU	: 	PIC24FJ256GB210   
    Compiler	: 	XC16 Compiler V1.31  
	Author		:	EM003 
	Date		:	
	Company Name: 	Insys Digital Systems
	Modification History:
					Rev No			Date		Description
					 --				 --				--    
*********************************************************************************************************************/
#ifndef _COMM_SMC_H_
#define _COMM_SMC_H_
/*
 *      Comm_SM routines header file
 */

/*
 * I/O Ports - Communication related
 */
#define M0						LATBbits.LATB14         /*Set RJ0 port to MODEM Request to Send Signal*/
#define MODEM_CD				PORTBbits.RB13         /*Set RJ1 port to MODEM Carrier detect Signal */
#define M1						LATBbits.LATB15         /*Set RJ2 port to MODEM Clear to Send Signal*/

/* SMC Comm Process time-outs in milli-seconds */
#define MODEM_TRC_ON_TIMEOUT			(100)		/* This is Maximum Time Scheduler Waits to Get Clear to Send Signal from Modem for the 1st time*/
#define TRANSMIT_WAIT_TIME				(800)		/* Pre Transmission Delay Time */
#define RS_HOLD_LOW_TIMEOUT				(100)		/* Once All Bytes are Transmitted, Release RS Signal after this time Completes */

#define SMC_COMMAND_RECEIVE_TIMEOUT     (18)		/* (18millsec)Maximum time allowed between Bytes of one Command from Station Master Unit*/
#define TIME_BETWEEN_RECORDS_FOR_SMC	(20)		/* Transmission delay Between Records - 10ms */
#define SMC_RESPONSE_WAIT_TIME			(2000)		/* (2Sec)- Max time Smc scheduler wait for SM Reply */

#define SMC_COMM_SCHEDULER_SCAN_RATE	(875)
#define SMC_XMIT_MESSAGE_LENGTH			(80)


#define RX_STABLE_WAIT_TIME                 (10)
#define CD_LOW_TO_HIGH_STABLE_WAIT_TIME     (10)
#define RX_QUERY_WAIT_TIME                  (70)
#define WAIT_TIME_FOR_CD_LOW_TIME           (100)
#define ERROR_WAIT_TIME                     (300)
#define OTHER_COM_WAIT_TIME                 (50+533+50)
#define WAIT_FOR_TX_TRANSITION_TIME         (20)
#define WAIT_TX_STABLE_TIME                 (100)
#define PACKET_TX_MAX_WAIT_TIME             (550)
#define SMC_COM_TX_END_WAIT                 (20)
#define SMC_TRANSMIT_WAIT_TIME              (20)
#define NEXT_QUERY_WAIT                     (100)
#define NEXT_ERR_QUERY_WAIT                 (40)

/* Smc Scheduler -States */
typedef enum {
			SMC_SCHEDULER_NOT_STARTED = 0,		/* Default State */
			SMC_COMM_SCHEDULER_IDLE,			/* when there is no commands to service scheduler will be in this State */ 
            RX_STABLE_WAIT,
            WAIT_FOR_SMC_CD_HIGH,
            WAIT_FOR_SMC_CD_HIGH_STABLE,
            WAIT_FOR_RX_QUERY,
            WAIT_FOR_CD_LOW,
            WAIT_FOR_TX_TRANSITION,
            WAIT_TX_STABLE,
            TRANSMIT_SMC_DATA,
            WAIT_AFTER_TX,
            ERROR_SMC_MODEM
} smc_sch_state_t;

/* Smc Scheduler */
#define RCV_QUERY_LEN   5
typedef struct {	
			smc_sch_state_t	State;			/* Smc scheduler present state */	
			UINT16			Timeout_ms;		/* One milliSecond Variable for Smc scheduler*/
			BYTE			Rx_query_len;		/* Transmission Time of One Byte depends on Baud Rate */
            BYTE            Recvd_query[RCV_QUERY_LEN];
            BYTE            Req_CPU_Addr;
} smc_sch_info_t;

typedef struct {
			BYTE	Msg_Length;						
			BYTE	Index;			
			BYTE	Msg_Buffer[SMC_XMIT_MESSAGE_LENGTH];
} smc_info_t;

extern void Initialise_Smc_CommSch(void);
extern void SetupCOM1BaudRate(BYTE uchBaudRate);
extern void Start_Smc_Communication(void);
extern void Update_Smc_Sch_State_QR(void);
extern void Set_Smc_Sch_Idle(void);
extern void Decrement_Smc_Sch_msTmr(void);
extern void Clear_Com1_Error(void);
extern void Build_smc_broadcast_message(void);
extern void Initialise_Modem(void);
extern void Set_Modem_TX_Mode(void);
extern void Set_Modem_RX_Mode(void);

#endif
