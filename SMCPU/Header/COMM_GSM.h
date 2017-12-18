/* 
 * File:   COMM_GSM.h
 * Author: I15076
 *
 * Created on November 9, 2015, 11:48 PM
 */

#ifndef COMM_GSM_H
#define	COMM_GSM_H

/*
 *      Comm_GSM routines header file
 */

/* Timeouts in ms */
#define GSM_MAX_RECV_MESSAGE_LENGTH                             256
#define GSM_CHECK_INCOMMING_MSG_TIMEOUT			  	(20000)  /* (10Sec)Maximum time allowed between Bytes of one Command from GSM*/
#define ASYNCHRONOUS_MODE							(0xFF)	 /* Mode of communication selected -Asynchronous Mode */
#define SYNCHRONOUS_MODE							(0x55)	 /* Mode of communication selected -Synchronous Mode */

#define GSM_REPLY_WAIT_TIME 						(1000)    /* (200ms)- Max time GSM scheduler wait for GSM Reply */
#define RECORD_GAP_TIME								(10)     /* Transmission delay Between Records - 10ms */

/* GSM Scheduler -States */
typedef enum {
                        GSM_SCHEDULER_NOT_STARTED = 0,			/* Default State */
                        GSM_INITIALIZE,
			GSM_COMM_SCHEDULER_WAIT,				/* when there is no commands to service scheduler will be in this State */
			BUILD_RECORDS_FOR_GSM_INIT,                 /* It Builds the Records in Xmit Queue to Transmit */
			TRANSMIT_RECORDS_FOR_GSM,				/* It handles the transmision of All record filled in Xmit Queue */
			GSM_WAIT_FOR_REPLY,					/* In this state scheduler waits for Reply from GSM */
			PROCESS_GSM_REPLY_COMMAND,				/* Process the Reply command Received and Retransmit the Corrupted Records at Receiving End */
                        GSM_SEND_MESSAGE,
                        GSM_FORM_MESSAGE
} GSM_sch_state_t;

typedef enum {
    GSM_ATE0 = 0,
            GSM_CPIN,
            GSM_CFUN,
            GSM_CMGF,
            GSM_CSMP,
            GSM_CSCS,
            GSM_CMGS,
            GSM_CMGL,
            GSM_CMGD
} GSM_AT_Commands;

typedef struct {
			GSM_sch_state_t	State;			/* GSM scheduler present state */
			UINT16				Timeout_ms;		/* One milliSecond Variable for GSM scheduler*/
			BYTE				BytePeriod;		/* Transmission Time of One Byte deponds on Baud Rate */
                        GSM_AT_Commands                 AT_command;
                        BYTE                            GSM_Init_Success;
                        BYTE                            Error_Occured;
                        BYTE                            Error_Reported;
                        BYTE                            Error_Number;
                        char                            Error_Message[40];
                        BYTE                            Change_number_Success;
                        BYTE                            GSM_Enable;
                        BYTE                            Ph_number_index;
                        BYTE                            Form_message;
} GSM_sch_info_t;

/* COM Process states - MESSAGE RECEIVER which is common to Both Host and Smc Receiver  */
typedef enum {
 			 GSM_IGNORE_RECV_DATA= 0,  /* when no Bytes are Received , Scheduler will be in this State */
			 GSM_CONSIDER_RECV_DATA
			}GSM_rece_state_t;

typedef struct {
			GSM_rece_state_t	State;			/*Comm Receiver States */
			UINT16				Timeout_ms;		/* One milliSecond Variable for Receive scheduler */
			UINT16  			Msg_Length;		/* length of command that is receiving */
			UINT16				Index;			/* Pointer to access Bytes in Message Buffer */
			char				Msg_Buffer[GSM_MAX_RECV_MESSAGE_LENGTH]; /* Buffer to Store the Received Data */
} GSM_recv_msg_info_t;

extern void Initialise_GSM_CommSch(void);
extern void Start_GSM_Communication(void);
extern void Update_GSM_Sch_State(void);
extern void Decrement_GSM_Sch_msTmr(void);
void SetupCOM3BaudRate(BYTE);
void Receive_COM3_Message(void);
void Clear_Com3_Error(void);
void Clear_COM3_Receive_Buffer(void);
void Clear_COM3_Transmit_Buffer(void);
void Build_GSM_Init_Command(void);
void Check_Error(void);
void Check_change_number(void);

extern GSM_sch_info_t GSM_Sch_Info;			/* Structure holds GSM comm scheduler information */
extern disp_info_t Disp_Info;
extern BYTE var_count, SF_Error,EF_Error;
extern BYTE inspect_event_done;
extern BYTE feedback_error,FB_error_ID;
#endif	/* COMM_GSM_H */

