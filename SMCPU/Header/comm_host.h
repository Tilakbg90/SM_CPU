/*********************************************************************************************************************
	Project		: 	Single Section Digital Axle Counter
	Version		: 	2.0 
	Revision	:	1	
	Filename	: 	comm_host.h
	Target MCU	: 	PIC24FJ256GB210   
    Compiler	: 	XC16 Compiler V1.31  
	Author		:	EM003
	Date		:	
	Company Name: 	Insys Digital Systems
	Modification History:
					Rev No			Date		Description
					 --				 --				--    
*********************************************************************************************************************/
#ifndef _COMM_HOST_H_
#define _COMM_HOST_H_
/*
 *      Comm_Host routines header file
 */

/* Timeouts in ms */
#define HOST_COMMAND_RECEIVE_TIMEOUT			  	(10000)  /* (10Sec)Maximum time allowed between Bytes of one Command from Host*/
#define ASYNCHRONOUS_MODE							(0xFF)	 /* Mode of communication selected -Asynchronous Mode */
#define SYNCHRONOUS_MODE							(0x55)	 /* Mode of communication selected -Synchronous Mode */

#define HOST_REPLY_WAIT_TIME 						(200)    /* (200ms)- Max time host scheduler wait for Host Reply */
#define RECORD_GAP_TIME								(10)     /* Transmission delay Between Records - 10ms */

/* Host Scheduler -States */
typedef enum {
			HOST_SCHEDULER_NOT_STARTED = 0,			/* Default State */
			HOST_COMM_SCHEDULER_IDLE,				/* when there is no commands to service scheduler will be in this State */ 
			BUILD_RECORDS_FOR_HOST,                 /* It Builds the Records in Xmit Queue to Transmit */
			TRANSMIT_RECORDS_FOR_HOST,				/* It handles the transmision of All record filled in Xmit Queue */
			HOST_WAIT_FOR_REPLY,					/* In this state scheduler waits for Reply from Host */
			PROCESS_HOST_REPLY_COMMAND,				/* Process the Reply command Received and Retransmit the Corrupted Records at Receiving End */
			TRANSMIT_SINGLE_RECORD_FOR_HOST			/* Transmit Single Record */
} host_sch_state_t;


typedef struct {
			host_sch_state_t	State;			/* Host scheduler present state */
			host_sch_state_t    Next_State;		/* Host scheduler Next state */
			UINT16				Timeout_ms;		/* One milliSecond Variable for Host scheduler*/
			BYTE				BytePeriod;		/* Transmission Time of One Byte deponds on Baud Rate */
                        UINT16				USB_Timeout_ms;		/* One milliSecond Variable for Host scheduler*/
                        BYTE                            USB_Connection_status;
                        BYTE                            Rx_USB_Data_available;
                        BYTE                            Tx_USB_Data_available;
                        BYTE                            Tx_USB_Byte_Count;
                        BYTE                            Tx_USB_Logged_events;
} host_sch_info_t;


extern void Initialise_Host_CommSch(void);
extern void Start_Host_Communication(void);
extern void Update_Host_Sch_State(void);
extern void Decrement_Host_Sch_msTmr(void);
extern void SetupCOM2BaudRate(BYTE);
extern void Receive_COM2_Message(void);
extern void Clear_Com2_Error(void);
extern void Clear_COM2_Receive_Buffer(void);


#endif
