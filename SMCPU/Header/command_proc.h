/*********************************************************************************************************************
	Project		: 	Single Section Digital Axle Counter
	Version		: 	2.0 
	Revision	:	1	
	Filename	: 	command_proc.h
	Target MCU	: 	PIC24FJ256GB210   
    Compiler	: 	XC16 Compiler V1.31  
	Author		:	EM003
	Date		:	
	Company Name: 	Insys Digital Systems
	Modification History:
					Rev No			Date		Description
					 --				 --				--    
*********************************************************************************************************************/
#ifndef _COMMAND_PROC_H_
#define _COMMAND_PROC_H_
/*
 *      Command_Proc routines header file
 */


#define MAX_HOST_SEND_MESSAGE_LENGTH    (20)    /* Max length of the Record send to Host/Smc */
#define MAX_RECV_MESSAGE_LENGTH			(12)	/* Host/Smc command Receive Buffer Size */
#define MAX_XMIT_QUEUE_SIZE 			(1)		/* Host/Smc Reply Data Queue Size */

#define NO_OF_HOST_COMMANDS				(7)		/* No of Host/Smc Commands Available*/
#define ALL_RECORDS_SUCCESSFULL			(0xFF)  /* indicates that All records Received at Host/Smc correctly */
#define ALL_EVENTS						(0xFF)  /* indicates that all events in serial EEPROM has to be Transmitted to Host/Smc */

/* Message offsets */
#define HOST_ADDR_OFFSET				(0)		/* index of Recive Buffer identifies the Host Address */
#define HOST_COMMAND_OFFSET				(1)		/* index of Recive Buffer identifies the Command Id */
#define HOST_DATA_BYTE1_OFFSET			(2)		/* index of Recive Buffer identifies the 1'st Data Byte */
#define HOST_DATA_BYTE2_OFFSET			(3)		/* index of Recive Buffer identifies the 2nd  Data Byte */
#define HOST_DATA_BYTE3_OFFSET			(4)		/* index of Recive Buffer identifies the 3'rd Data Byte */
#define HOST_DATA_BYTE4_OFFSET			(5)		/* index of Recive Buffer identifies the 4'th Data Byte */
#define HOST_DATA_BYTE5_OFFSET			(6)		/* index of Recive Buffer identifies the 5'th Data Byte */
#define HOST_DATA_BYTE6_OFFSET			(7)		/* index of Recive Buffer identifies the 6'th Data Byte */
#define HOST_DATA_BYTE7_OFFSET			(8)		/* index of Recive Buffer identifies the 7'th Data Byte */


#define GET_LOGGED_EVENTS				(0)		/* Type: Command for getting logged events */
#define SET_RTC_DATE_AND_TIME			(1)		/* Type: Command for Setting Date and Time */ 
#define GET_RTC_DATE_TIME				(2)		/* Type: Command for getting system Date and Time */
#define ERASE_EVENTS_EEPROM				(3)		/* Type: Command for Erasing all logged events */
#define GET_DAC_STATUS					(4)		/* Type: Command for getting Dac Status	 */ 
#define REPLY_TO_RECORDS				(5)		/* Type: Reply of Record Reception status from Host/Smc */ 
#define CANCEL_GET_LOGGED_EVENTS		(18)	/* Type: Command to cancel when logged data is being transmitted */
#define GET_EVENT_COUNTS				(6)		/* Type: Command for getting number of events start date and end date	 */

#define LOGGED_EVENTS_REPLY				(0)		/* Type: Command Id for Logged events */
#define DATE_TIME_REPLY					(1)		/* Type: Command Id for Date and Time reply */
#define EEPROM_ERASED_REPLY				(2)		/* Type: Command Id for reply to "Erase command" */
#define DAC_STATUS_REPLY				(3)		/* Type: Command Id for Dac Status reply */
#define END_OF_EVENT_REPLY				(4)		/* Type: Command Id to indicate the end of Event Transmission */ 
#define END_EEPROM_ERASED_REPLY			(5)		/* Type: Command Id to indicate the end of EEPROM Erase Transmission */ 
#define REPLY_EVENTS_COUNT                              (6)     /*sending event details*/
#define BUSY_REPLY						(8)		/* Type Reply when unit is busy */


/* Structure used to Store Date and Time which is avialable in Sec (LWord).
   Also to reterive Time Byte by Byte to Transmit to Host. 
   Used in Build_Time_Record_Message() function of command_proc.h */	 
typedef union {
	long LWord;
	struct	{
		BYTE	Byte1;
		BYTE	Byte2;
		BYTE	Byte3;
		BYTE	Byte4;
	} Byte;
} long_t;      


/* COM Process states - MESSAGE RECEIVER which is common to Both Host and Smc Receiver  */
typedef enum {
 			 RECV_BUFFER_EMPTY =0,  /* when no Bytes are Received , Scheduler will be in this State */ 
			 RECE_COMMAND,			/* When Second Byte Received  , Scdeduler will come to this State */ 
									/* In all type of Commands Second Byte is Command Id */		
			 RECV_DATA,				/* it will Start Receiving remaining Data Bytes */	
			 CHECK_CRC16			/* Validate the Received  Data */
			}com_rece_state_t;

/* Query Parameter Structure */
typedef struct {
			INT32	Read_Count;			  /* This Variable Maintains the No of Record Read from Serial EEPROM */
			INT32	Max_Permitted_Reads;  /* This Variable Holds the Max permittable read of Records */ 
			INT32	Read_Pointer;		  /* it is Used to maintain the Record Number which is to be read next */	
			time_t	TimeStamp_SetPoint;   /* Time Stamp Point determines how many days events has to be Transmitted */
			BOOL    uchIs_Next_Available; /* This Variable Indicates the availablity of Next of Record */ 	
} query_param_t;

/* Xmit Queue -> it can store 8 event records for Transmission.
   Both Host Scheduler and Smc Scheduler transmitt  the Event Record present in Xmit Queue */	
typedef struct {			
				BYTE  Record_Index;      /* Pointer to Access the Records in Queue */
				BYTE  Record_Count; 	 /* indicates the Number of Records present in Queue */	
				BYTE  Mode;				 /* indicates mode of Transmission- Async or Sync */	
				BYTE  Index;			 /* Pointer to Access the Bytes in Particular Record */
				BYTE  Record_Rx_Status;	 /* The Reception Status of Records send by Host/Smc */		
				struct {	
						BYTE  Host_Msg_Length;	/* Length of Message for Host Transmission */				
						BYTE  Smc_Msg_Length;	/* Length of Message for Smc Transmission */				
						BYTE  Msg_Buffer[MAX_HOST_SEND_MESSAGE_LENGTH]; /* Message Buffer to Store Bytes of the Record */
			   } Record[MAX_XMIT_QUEUE_SIZE];			
} xmit_queue_info_t;


/* Comm Receiver Scheduler - This structure is common to Both Receiver Comm1 and Comm2 */
typedef struct {
			com_rece_state_t	State;			/*Comm Receiver States */
			UINT16				Timeout_ms;		/* One milliSecond Variable for Receive scheduler */	
			BYTE				Msg_Length;		/* length of command that is receiving */
			BYTE				Index;			/* Pointer to access Bytes in Message Buffer */
			BYTE				Msg_Buffer[MAX_RECV_MESSAGE_LENGTH]; /* Buffer to Store the Received Data */
} host_recv_msg_info_t;


extern void Process_Host_Command(void);
extern void Setup_Get_Logged_Events_Params(void);
extern void Build_Next_Logged_Event_Reply(void);
extern void Build_Time_Record_Message(void);
extern void Build_Event_Record_Message(void);
extern void Notify_End_of_Events_Reply(void);
extern void Process_Set_Date_And_Time_Command(void);
extern void Build_Reply_To_Erase_Command(void);
extern void Build_End_EEPROM_Erase_Reply_Command(void);
extern void Build_Reply_To_Status_Command(void);
extern void Build_Reply_To_Get_Event_Command(void);

extern BYTE Compute_LRC(event_record_t,BYTE);
#endif
