/*********************************************************************************************************************
*	Project		: 	Single Section Digital Axle Counter
*	Version		: 	2.0 
*	Revision	:	1	
*	Filename	: 	comm_dac.c
*	Target MCU	: 	PIC24FJ256GB210   
*    Compiler	: 	XC16 Compiler V1.31  
*	Author		:	EM003
*	Date		:	
*	Company Name: 	Insys Digital Systems
*	Modification History:
*					Rev No			Date		Description
*					 --				 --				--    
*	Functions	:   
*                 void Process_Host_Command(void);
*                 void Setup_Get_Logged_Events_Params(void);
*                 void Build_Next_Logged_Event_Reply(void);
*                 void Build_Time_Record_Message(void);
*                 void Build_Event_Record_Message(void);
*                 void Notify_End_of_Events_Reply(void);
*                 void Process_Set_Date_And_Time_Command(void);
*                 void Build_Reply_To_Erase_Command(void);
*                 void Build_End_EEPROM_Erase_Reply_Command(void);
*                 void Build_Reply_To_Status_Command(void);
*                 void Build_Reply_To_Get_Event_Command(void);
*********************************************************************************************************************/
#include <xc.h>
#include <time.h>

#include "COMMON.h"
#include "comm_host.h"
#include "DRV_MEM.h"
#include "EVENTS.h"
#include "CRC16.h"
#include "command_proc.h"

extern dac_sysinfo_t DAC_sysinfo;
extern time_t SystemClock,SystemDate;						/* from cpu_sm.c */
extern sm_status_t Status;						/* from cpu_sm.c */
extern event_register_t Shadow[MAXIMUM_NO_OF_CPU];	/* comm_dac.c	 */
extern BYTE CPU1_Address;						/* from cpu_sm.c */
extern BYTE CPU2_Address;						/* from cpu_sm.c */
extern host_sch_info_t   Host_Sch_Info;				/* from comm_host.c */			
extern UINT16 Page;
extern wordtype_t Erase_address;
extern EEPROM_Erase_State Block_erase_status;
extern host_recv_msg_info_t Com2RecvObject;
extern ring_buffer_t  Events_Ring_Buffer;
extern event_record_t Event_H, Event_T, Event_0;

/* Message Length Table for Different Commands from PC/SMC */	 
   /* D     T	  G	    E     S     R */		
const BYTE uchCommand_Length_Table[7][2] = {
    {0,6},{1,11},{2,4},{3,4},{4,4},{5,5},{6,4}
};    
query_param_t     		 Query_Param;				/* this structure holds query param information */
event_record_t    	     Event_Record;				/* this structure holds event record information */
xmit_queue_info_t 		 Xmit_Queue;				/* this structure holds Xmit queue information */
BYTE  Command_Info[12];								/* received data from host command info */ 

/*********************************************************************************
*File name 			:comm_dac.c
*Function Name		:void Process_Host_Command(void)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			: Process the command from  Host and Station Master Unit.
*Algorithm			:
*					 *.SM external Interface is designed to Service one command at a time either
*					   from Host or Smc. When SM-CPU  is servicing one, the command comes from 
*					   other side is rejected.	  					
*Description		: 
*Input Element		:None 
*					 
*Output Element		:void
*
**********************************************************************************/
void Process_Host_Command(void)
{
	Command_Info[HOST_ADDR_OFFSET]      = Com2RecvObject.Msg_Buffer[HOST_ADDR_OFFSET];	

	Command_Info[HOST_COMMAND_OFFSET] = Com2RecvObject.Msg_Buffer[HOST_COMMAND_OFFSET];

	if (Status.Flags.Query_Pending == TRUE && Command_Info[HOST_COMMAND_OFFSET] != REPLY_TO_RECORDS )
	{	
	  /* Pervious Query is not Completed. So it won't take New Commands */	
      return;		
	}	

	switch (Command_Info[HOST_COMMAND_OFFSET])
	{	
		case SET_RTC_DATE_AND_TIME:			
			/* Collect the Date and Time Information send by Host / SMC to Command Buffer*/	
			Command_Info[HOST_DATA_BYTE1_OFFSET] = Com2RecvObject.Msg_Buffer[HOST_DATA_BYTE1_OFFSET];	
			Command_Info[HOST_DATA_BYTE2_OFFSET] = Com2RecvObject.Msg_Buffer[HOST_DATA_BYTE2_OFFSET];	
			Command_Info[HOST_DATA_BYTE3_OFFSET] = Com2RecvObject.Msg_Buffer[HOST_DATA_BYTE3_OFFSET];	
			Command_Info[HOST_DATA_BYTE4_OFFSET] = Com2RecvObject.Msg_Buffer[HOST_DATA_BYTE4_OFFSET];	
			Command_Info[HOST_DATA_BYTE5_OFFSET] = Com2RecvObject.Msg_Buffer[HOST_DATA_BYTE5_OFFSET];	
			Command_Info[HOST_DATA_BYTE6_OFFSET] = Com2RecvObject.Msg_Buffer[HOST_DATA_BYTE6_OFFSET];	
			Command_Info[HOST_DATA_BYTE7_OFFSET] = Com2RecvObject.Msg_Buffer[HOST_DATA_BYTE7_OFFSET];					 
			Process_Set_Date_And_Time_Command();   /* This Function executes the Date and Time Updation */  
 			Build_Time_Record_Message();			  /* Build the Date and Time Reply to unit who has send the Query */ 		
			/* Reply from Host */	
			Host_Sch_Info.State      = TRANSMIT_SINGLE_RECORD_FOR_HOST;
			Host_Sch_Info.Next_State = HOST_COMM_SCHEDULER_IDLE;		
			break;		
		case GET_RTC_DATE_TIME:
			Build_Time_Record_Message();	/* Build the Date and Time Reply to unit who has send the Query */ 		
			/* Reply from Host */		
		    Host_Sch_Info.State      = TRANSMIT_SINGLE_RECORD_FOR_HOST;
		    Host_Sch_Info.Next_State = HOST_COMM_SCHEDULER_IDLE;		
			break;
		case GET_LOGGED_EVENTS:
			Command_Info[HOST_DATA_BYTE1_OFFSET] = Com2RecvObject.Msg_Buffer[HOST_DATA_BYTE1_OFFSET]; /* No of Days Events Required  */
			Command_Info[HOST_DATA_BYTE2_OFFSET] = Com2RecvObject.Msg_Buffer[HOST_DATA_BYTE2_OFFSET]; /* Mode Selection Async / Sync */	
			Setup_Get_Logged_Events_Params();  /*it determines no of Events to be Transmitted */							  
			/* Reply to Host */	
			Xmit_Queue.Record_Count = 0;	 
			Xmit_Queue.Record_Index = 0;	
			Host_Sch_Info.Tx_USB_Logged_events = 1;
            Host_Sch_Info.State      = BUILD_RECORDS_FOR_HOST;
			Host_Sch_Info.Next_State = HOST_COMM_SCHEDULER_IDLE;							   
			break;
		case REPLY_TO_RECORDS:				
			/* This command is received when CRC Corruption Occured at Receiving End - Station Master Unit */
			Xmit_Queue.Record_Rx_Status = Com2RecvObject.Msg_Buffer[HOST_DATA_BYTE1_OFFSET];
			/* Reply from Host */	
			if(Xmit_Queue.Record_Rx_Status == ALL_RECORDS_SUCCESSFULL)
		 	{
			   /* All records are received at Host Successfully 
				  So, go for Building of Next Batch of Records */		
			   Xmit_Queue.Record_Count = 0;	 
			   Xmit_Queue.Record_Index = 0;
			   Host_Sch_Info.State      = BUILD_RECORDS_FOR_HOST;
		       Host_Sch_Info.Next_State = HOST_COMM_SCHEDULER_IDLE;		
		 	}
			else
			{
			  /* Some Records received at Host corrupted, So for Retransmission of those Records */	
			   Host_Sch_Info.Timeout_ms = 0;
			   Host_Sch_Info.State      = PROCESS_HOST_REPLY_COMMAND;
		       Host_Sch_Info.Next_State = HOST_COMM_SCHEDULER_IDLE;			
			}			   
			break;				
		case ERASE_EVENTS_EEPROM:		
			if(Setup_Events_Erasure())  /* setup the Event Logger for ERASE Operation */
			{
                Page = 0;
                Erase_address.Word = 0;
                Block_erase_status = START;
				/* Erase Command Accepted, So Send a Reply to Command Sender */
			    Build_Reply_To_Erase_Command();
                //Build_End_EEPROM_Erase_Reply_Command();
				/* Reply to Host */	
			    Host_Sch_Info.State      = TRANSMIT_SINGLE_RECORD_FOR_HOST;
			    Host_Sch_Info.Next_State = HOST_COMM_SCHEDULER_IDLE;		
			}									
            break;
		case GET_DAC_STATUS:
			Build_Reply_To_Status_Command();	/* Build Dac Status for Reply to Get_Dac_status Command */
			/* Reply to Host */	
            Host_Sch_Info.State      = TRANSMIT_SINGLE_RECORD_FOR_HOST;
			Host_Sch_Info.Next_State = HOST_COMM_SCHEDULER_IDLE;		
			break;
        case GET_EVENT_COUNTS:
			Build_Reply_To_Get_Event_Command();	/* Build Dac Status for Reply to Get_Dac_status Command */
			/* Reply to Host */
			Host_Sch_Info.State      = TRANSMIT_SINGLE_RECORD_FOR_HOST;
			Host_Sch_Info.Next_State = HOST_COMM_SCHEDULER_IDLE;
			break;
        default:
            break;
	}
}
/*********************************************************************************
*File name 			:comm_dac.c
*Function Name		:void Setup_Get_Logged_Events_Params(void)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			: Sets The Parameter reading Events From Serial EEPROM
*					  Parameters :No of Events to be Retrived, Read Pointer,Read 
*                       Count,Record Count  	
*Algorithm			:
*Description		: 
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Setup_Get_Logged_Events_Params(void)
{
	/* By making query pending status to True,
	 other Commands are rejected unless this status is set to False */ 								
	Status.Flags.Query_Pending = TRUE;

	Query_Param.Max_Permitted_Reads = MAXIMUM_EVENTS;
	Query_Param.Read_Count   = 0;
	
	if ((Get_Current_Event_Number()-1)>MAXIMUM_EVENTS)
	{
		Query_Param.Read_Pointer=0;
	}
	else
	{
		Query_Param.Read_Pointer = Get_Current_Event_Number()-1;
	}	
	Query_Param.uchIs_Next_Available = (BOOL) TRUE;
	Xmit_Queue.Mode = 0xFF;     /* Transmission Mode Async / Sync */
    Xmit_Queue.Record_Count = 0;
    Xmit_Queue.Record_Index = 0;						
	Clear_Events_Counter();										/* from events.c */

	/* first data byte contains number of days of data to be retreived */
	if (Command_Info[HOST_DATA_BYTE1_OFFSET] != 0 &&
        Command_Info[HOST_DATA_BYTE1_OFFSET] != ALL_EVENTS )
	{
		Query_Param.TimeStamp_SetPoint = ( SystemDate - 
								(Command_Info[HOST_DATA_BYTE1_OFFSET]* SECONDS_PER_DAY));
	}
	else
	{
		/* All the events has to be transferred. Set maximum possible date */
		Query_Param.TimeStamp_SetPoint = (long) 0 ;
	}
}

/*********************************************************************************
*File name 			:comm_dac.c
*Function Name		:void Build_Next_Logged_Event_Reply(void)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			: Read the Events from Serial EEPROM and Builds the Xmit Queue
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Build_Next_Logged_Event_Reply(void)
{
	BYTE uchBuf=0;

	/* Ignore any new event which has occured after GET_LOGGED_EVENTS command */
	uchBuf = Get_Events_Counter_Value();        /* returns No of events Occured */
	Query_Param.Max_Permitted_Reads -= uchBuf;  
	Clear_Events_Counter();						/* from events.c */

	if(Query_Param.Read_Count >= Query_Param.Max_Permitted_Reads) /* Check Whether All events are read from Memory */
	{
        Notify_End_of_Events_Reply();		/* Build End of Events Reply */					
        Query_Param.uchIs_Next_Available = (BOOL) FALSE;	
        return;
	}
	/* Fetch the data */
    for (uchBuf = 0; uchBuf <	EVENT_RECORD_SIZE; uchBuf++)
	{
		Event_Record_R.Byte[uchBuf] = 0;
	}
	if(Read_Event_from_Serial_EEProm((UINT16)Query_Param.Read_Pointer) == FALSE)
	{
		return;	/* Read failed */
	}
	for (uchBuf = 0; uchBuf <	EVENT_RECORD_SIZE; uchBuf++)
	{
		Event_Record.Byte[uchBuf] = Event_Record_R.Byte[uchBuf];
	}
	uchBuf = Compute_LRC(Event_Record, 15);

	/* Update Read Pointer */
	if(Query_Param.Read_Pointer <= 0)
	{
       Query_Param.Read_Pointer = MAXIMUM_EVENTS - 1;
	}
	else
	{		
	   Query_Param.Read_Pointer = Query_Param.Read_Pointer - 1;	
	}

	Query_Param.Read_Count = Query_Param.Read_Count + 1;	   /* Increment Read Count */

	if (Event_Record.Field.LRC != uchBuf)
	{
		/* Checksum does not match. Record corrupted */
		/* Record read was out of Time Stamp, So No need to Transmit the record */
		Notify_End_of_Events_Reply();		/* Build End of Events Reply */
		Query_Param.uchIs_Next_Available = FALSE;
        return;
	}
	if (Event_Record.Field.Token == LATEST_EVENT_TOKEN || 
		Event_Record.Field.Token == OLD_EVENT_TOKEN)
	{		
		if (Event_Record.Field.Date_Time >= Query_Param.TimeStamp_SetPoint)
		{				 
			Build_Event_Record_Message();	/* Build the Record in Xmit_Queue for Transmission */			 	
		}
		else
		{
			/* Record read was out of Time Stamp, So No need to Transmit the record */		
		 	Notify_End_of_Events_Reply();		/* Build End of Events Reply */		
			Query_Param.uchIs_Next_Available = FALSE;			
		}
	}
}

/*********************************************************************************
*File name 			:comm_dac.c
*Function Name		:void Build_Time_Record_Message(void)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			: Build Date and Time Message Record 
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Build_Time_Record_Message(void)
{
	wordtype_t CheckSum;
	long_t L_Time;
	//BYTE *puchMsg;
 
	L_Time.LWord = SystemClock;                    /* Read the System Clock */
	//puchMsg  = & Xmit_Queue.Record[0].Msg_Buffer[0];
	Xmit_Queue.Record[0].Msg_Buffer[0]  = CPU1_Address;				   /* Put the Source Id - Address of CPU1 */	
	Xmit_Queue.Record[0].Msg_Buffer[1]  = DATE_TIME_REPLY;			   /* Command Id */	
	Xmit_Queue.Record[0].Msg_Buffer[2]  = L_Time.Byte.Byte1; 			   /* System Time in sec -4 Bytes */	
	Xmit_Queue.Record[0].Msg_Buffer[3]  = L_Time.Byte.Byte2;
	Xmit_Queue.Record[0].Msg_Buffer[4]  = L_Time.Byte.Byte3;
	Xmit_Queue.Record[0].Msg_Buffer[5]  = L_Time.Byte.Byte4;
	CheckSum.Word = Crc16(XMIT_QUEUE,6);				/* Calculate CRC 16 for Xmit Buffer */
	Xmit_Queue.Record[0].Msg_Buffer[6]  = CheckSum.Byte.Lo;				/* CRC low Byte */ 
	Xmit_Queue.Record[0].Msg_Buffer[7]  = CheckSum.Byte.Hi;				/* CRC High Byte */
	Xmit_Queue.Record[0].Msg_Buffer[8]  = 0x0D;  							/* Add CR */
	Xmit_Queue.Record[0].Msg_Buffer[9]  = 0x0A;  							/* Add LF */
	Xmit_Queue.Record[0].Host_Msg_Length = 10;  
	Xmit_Queue.Record[0].Smc_Msg_Length = 8;  
	Xmit_Queue.Index = 0;
	Xmit_Queue.Record_Index =0;
    Xmit_Queue.Record_Count =1;	
	Status.Flags.Query_Pending = FALSE;
}

/*********************************************************************************
*File name 			:comm_dac.c
*Function Name		:void Build_Event_Record_Message(void)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Build Event  Message Record 
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Build_Event_Record_Message(void)
{
	wordtype_t CheckSum;

	if(Xmit_Queue.Record_Index >= MAX_XMIT_QUEUE_SIZE)
	{
	   Xmit_Queue.Record_Index = 0;
	}
	Xmit_Queue.Record[Xmit_Queue.Record_Index].Msg_Buffer[0]   = Event_Record.Byte[5];		/* Source Id   */
	Xmit_Queue.Record[Xmit_Queue.Record_Index].Msg_Buffer[1]  	= LOGGED_EVENTS_REPLY;		/* Command  */	
	Xmit_Queue.Record[Xmit_Queue.Record_Index].Msg_Buffer[2]   = Event_Record.Byte[6];		/* Event Id	   */												
	Xmit_Queue.Record[Xmit_Queue.Record_Index].Msg_Buffer[3]   = Event_Record.Byte[1];		/* System Clock 4 Bytes */
	Xmit_Queue.Record[Xmit_Queue.Record_Index].Msg_Buffer[4]   = Event_Record.Byte[2];
	Xmit_Queue.Record[Xmit_Queue.Record_Index].Msg_Buffer[5]   = Event_Record.Byte[3];
	Xmit_Queue.Record[Xmit_Queue.Record_Index].Msg_Buffer[6]   = Event_Record.Byte[4];		
	Xmit_Queue.Record[Xmit_Queue.Record_Index].Msg_Buffer[7]   = Event_Record.Byte[7];		/* US FWD Axle Count 2 Bytes */
	Xmit_Queue.Record[Xmit_Queue.Record_Index].Msg_Buffer[8]   = Event_Record.Byte[8];	
	Xmit_Queue.Record[Xmit_Queue.Record_Index].Msg_Buffer[9]   = Event_Record.Byte[9];		/* US REV Axle Count 2 Bytes */
	Xmit_Queue.Record[Xmit_Queue.Record_Index].Msg_Buffer[10]   = Event_Record.Byte[10];
	Xmit_Queue.Record[Xmit_Queue.Record_Index].Msg_Buffer[11]   = Event_Record.Byte[11];	/* DS FWD Axle Count 2 Bytes */
	Xmit_Queue.Record[Xmit_Queue.Record_Index].Msg_Buffer[12]   = Event_Record.Byte[12];
	Xmit_Queue.Record[Xmit_Queue.Record_Index].Msg_Buffer[13]   = Event_Record.Byte[13];	/* DS REV Axle Count 2 Bytes */
	Xmit_Queue.Record[Xmit_Queue.Record_Index].Msg_Buffer[14]   = Event_Record.Byte[14];
	CheckSum.Word   = Crc16(XMIT_QUEUE,15);		/* Calculate CRC 16 for Xmit Buffer */
	Xmit_Queue.Record[Xmit_Queue.Record_Index].Msg_Buffer[15]   = CheckSum.Byte.Lo;			/* CRC low Byte */ 
	Xmit_Queue.Record[Xmit_Queue.Record_Index].Msg_Buffer[16]   = CheckSum.Byte.Hi;			/* CRC high Byte */ 
	Xmit_Queue.Record[Xmit_Queue.Record_Index].Msg_Buffer[17]   = 0x0D;  /* Add CR */
	Xmit_Queue.Record[Xmit_Queue.Record_Index].Msg_Buffer[18]   = 0x0A;  /* Add LF */
	Xmit_Queue.Record[Xmit_Queue.Record_Index].Host_Msg_Length = 19;  
	Xmit_Queue.Record[Xmit_Queue.Record_Index].Smc_Msg_Length  = 17;  
	Xmit_Queue.Index  = 0;
	if(Xmit_Queue.Record_Count >= MAX_XMIT_QUEUE_SIZE)
	{
        Xmit_Queue.Record_Index = 0;
	}
	else
	{
        Xmit_Queue.Record_Index = Xmit_Queue.Record_Index + 1;
	}
    Xmit_Queue.Record_Count = Xmit_Queue.Record_Count + 1;	
}

/*********************************************************************************
*File name 			:comm_dac.c
*Function Name		:void Notify_End_of_Events_Reply(void)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Build End of Events Reply
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Notify_End_of_Events_Reply(void)
{
	wordtype_t CheckSum;
 
	if(Xmit_Queue.Record_Index >= MAX_XMIT_QUEUE_SIZE)
	{
	   Xmit_Queue.Record_Index = 0;
	}	
	Xmit_Queue.Record[Xmit_Queue.Record_Index].Msg_Buffer[0]  = CPU1_Address;				/* Source Id   */
	Xmit_Queue.Record[Xmit_Queue.Record_Index].Msg_Buffer[1]  = END_OF_EVENT_REPLY;			/* Command  */
	CheckSum.Word = Crc16(XMIT_QUEUE, 2);			/* Calculate CRC 16 for Xmit Buffer */
	Xmit_Queue.Record[Xmit_Queue.Record_Index].Msg_Buffer[2]  = CheckSum.Byte.Lo;			/* CRC low Byte */ 
	Xmit_Queue.Record[Xmit_Queue.Record_Index].Msg_Buffer[3]  = CheckSum.Byte.Hi;			/* CRC high Byte */ 
	Xmit_Queue.Record[Xmit_Queue.Record_Index].Msg_Buffer[4]  = 0x0D;  						/* Add CR */
	Xmit_Queue.Record[Xmit_Queue.Record_Index].Msg_Buffer[5]  = 0x0A;  						/* Add LF */
	Xmit_Queue.Record[Xmit_Queue.Record_Index].Host_Msg_Length = 6;  
	Xmit_Queue.Record[Xmit_Queue.Record_Index].Smc_Msg_Length = 4;  
	Xmit_Queue.Index = 0;
	if( Xmit_Queue.Record_Count >= MAX_XMIT_QUEUE_SIZE)
	{
		Xmit_Queue.Record_Index =0;
	}
	else
	{
		Xmit_Queue.Record_Index = Xmit_Queue.Record_Index + 1;
	}
    Xmit_Queue.Record_Count = Xmit_Queue.Record_Count + 1;
}

/*********************************************************************************
*File name 			:comm_dac.c
*Function Name		:void Process_Set_Date_And_Time_Command(void)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:This Function executes the Date and Time Updation
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Process_Set_Date_And_Time_Command(void)
{
	wordtype_t year;
	BYTE month,date;
	BYTE hour,minute,sec;

	year.Byte.Lo = 	Command_Info[HOST_DATA_BYTE1_OFFSET];
	year.Byte.Hi = 	Command_Info[HOST_DATA_BYTE2_OFFSET];
    month 		 =  Command_Info[HOST_DATA_BYTE3_OFFSET];
    date 		 =  Command_Info[HOST_DATA_BYTE4_OFFSET];
    hour 		 =  Command_Info[HOST_DATA_BYTE5_OFFSET];
    minute 		 =  Command_Info[HOST_DATA_BYTE6_OFFSET];
    sec 		 =  Command_Info[HOST_DATA_BYTE7_OFFSET];

	if(Set_RTC_Date(date,month,year.Word))
	{
        /*This function is used to change day, Month and Year of Real Time Clock IC */
        if(Set_RTC_Time(hour,minute,sec))
		{   /*This function is used to change hour, Minute and Sec of Real Time Clock IC */
            Add_SM_Event_to_Queue((event_id_t) EVENT_DATE_TIME_CHANGED); /* DATE_TIME_Changed event is added to Event Queue */
		}
	}
}

/*********************************************************************************
*File name 			:comm_dac.c
*Function Name		:void Build_Reply_To_Erase_Command(void)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Build erase command reply 
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Build_Reply_To_Erase_Command(void)
{
	wordtype_t CheckSum;

	Xmit_Queue.Record[0].Msg_Buffer[0]  = CPU1_Address;				/* Source Id   */
	Xmit_Queue.Record[0].Msg_Buffer[1]  = EEPROM_ERASED_REPLY;		/* Command  */
	CheckSum.Word = Crc16(XMIT_QUEUE, 2);			/* Calculate CRC 16 for Xmit Buffer */
	Xmit_Queue.Record[0].Msg_Buffer[2]  = CheckSum.Byte.Lo;			/* CRC low Byte */ 
	Xmit_Queue.Record[0].Msg_Buffer[3]  = CheckSum.Byte.Hi;			/* CRC high Byte */ 
	Xmit_Queue.Record[0].Msg_Buffer[4]  = 0x0D;  						/* Add CR */
	Xmit_Queue.Record[0].Msg_Buffer[5]  = 0x0A;  						/* Add LF */
	Xmit_Queue.Record[0].Host_Msg_Length = 6;  
	Xmit_Queue.Record[0].Smc_Msg_Length = 4;  
	Xmit_Queue.Index = 0;
    Xmit_Queue.Record_Count =1;
	Xmit_Queue.Record_Index =0;	
}

/*********************************************************************************
*File name 			:comm_dac.c
*Function Name		:Build_End_EEPROM_Erase_Reply_Command(void)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Build erase command reply 
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Build_End_EEPROM_Erase_Reply_Command(void)
{
	wordtype_t CheckSum;

	Xmit_Queue.Record[0].Msg_Buffer[0] = CPU1_Address;						/* Source Id   */
	Xmit_Queue.Record[0].Msg_Buffer[1] = END_EEPROM_ERASED_REPLY;			/* Command  */
	CheckSum.Word = Crc16(XMIT_QUEUE, 2);					/* Calculate CRC 16 for Xmit Buffer */
	Xmit_Queue.Record[0].Msg_Buffer[2] = CheckSum.Byte.Lo;					/* CRC low Byte */ 
	Xmit_Queue.Record[0].Msg_Buffer[3] = CheckSum.Byte.Hi;					/* CRC high Byte */ 
	Xmit_Queue.Record[0].Msg_Buffer[4] = 0x0D;  								/* Add CR */
	Xmit_Queue.Record[0].Msg_Buffer[5] = 0x0A;  								/* Add LF */
	Xmit_Queue.Record[0].Host_Msg_Length = 6;  
	Xmit_Queue.Record[0].Smc_Msg_Length = 4;  
	Xmit_Queue.Index = 0;
    Xmit_Queue.Record_Count =1;
	Xmit_Queue.Record_Index =0;	
	Status.Flags.Query_Pending = FALSE;
}

/*********************************************************************************
*File name 			:comm_dac.c
*Function Name		:void Build_Reply_To_Status_Command(void)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Build status command reply
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Build_Reply_To_Status_Command(void)
{
	wordtype_t CheckSum;

	Xmit_Queue.Record[0].Msg_Buffer[0]  = CPU1_Address;		/* Source Id   */
	Xmit_Queue.Record[0].Msg_Buffer[1]  = DAC_STATUS_REPLY;					/* Command  */
    Xmit_Queue.Record[0].Msg_Buffer[2]  = DAC_sysinfo.Unit_Type;
    Xmit_Queue.Record[0].Msg_Buffer[3]  = Event_Logger_ID;
	if(Command_Info[HOST_ADDR_OFFSET] == CPU1_Address)
	{	
		Xmit_Queue.Record[0].Msg_Buffer[4]  = (BYTE) Shadow[0].Id.US_DAC;		/* US unit status */
		Xmit_Queue.Record[0].Msg_Buffer[5]  = (BYTE) Shadow[0].Id.DS_DAC;		/* DS unit status */
		Xmit_Queue.Record[0].Msg_Buffer[6]  = (BYTE) Shadow[0].Id.US_Block;   /* US Track_status */
		Xmit_Queue.Record[0].Msg_Buffer[7]  = (BYTE) Shadow[0].Id.DS_Block;	/* DS Track_status */
	}
	else
	{
		Xmit_Queue.Record[0].Msg_Buffer[4]  = (BYTE) Shadow[1].Id.US_DAC;		/* US unit status */
		Xmit_Queue.Record[0].Msg_Buffer[5]  = (BYTE) Shadow[1].Id.DS_DAC;		/* DS unit status */
		Xmit_Queue.Record[0].Msg_Buffer[6]  = (BYTE) Shadow[1].Id.US_Block;	/* US Track_status */
		Xmit_Queue.Record[0].Msg_Buffer[7]  = (BYTE) Shadow[1].Id.DS_Block;	/* DS Track_status */
	}				
	CheckSum.Word = Crc16(XMIT_QUEUE, 8);					/* Calculate CRC 16 for Xmit Buffer */
	Xmit_Queue.Record[0].Msg_Buffer[8] = CheckSum.Byte.Lo;					/* CRC low Byte */
	Xmit_Queue.Record[0].Msg_Buffer[9] = CheckSum.Byte.Hi;					/* CRC high Byte */
	Xmit_Queue.Record[0].Msg_Buffer[10]  = 0x0D;  								/* Add CR */
	Xmit_Queue.Record[0].Msg_Buffer[11]  = 0x0A;  								/* Add LF */
	Xmit_Queue.Record[0].Host_Msg_Length = 12;
	Xmit_Queue.Record[0].Smc_Msg_Length = 10;
	Xmit_Queue.Index = 0;
    Xmit_Queue.Record_Count =1;
	Xmit_Queue.Record_Index =0;	
	Status.Flags.Query_Pending = FALSE;
}

/*********************************************************************************
*File name 			:comm_dac.c
*Function Name		:void Build_Reply_To_Status_Command(void)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:void Build_Reply_To_Get_Event_Command(void)
*Algorithm			:
*Description			:
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Build_Reply_To_Get_Event_Command(void)
{

    long_t S_Time, E_Time, Event_count;
	wordtype_t CheckSum;

	Xmit_Queue.Record[0].Msg_Buffer[0]    = CPU1_Address;		/* Source Id   */
	Xmit_Queue.Record[0].Msg_Buffer[1]    = REPLY_EVENTS_COUNT;					/* Command  */
        
    E_Time.LWord = Event_H.Field.Date_Time;
    if(Event_T.Field.Token == OLD_EVENT_TOKEN)
    {
        S_Time.LWord = Event_T.Field.Date_Time;
        Event_count.LWord = MAXIMUM_EVENTS;
    }
    else
    {
        S_Time.LWord = Event_0.Field.Date_Time;
        Event_count.LWord = Mem_addr_ref.mem_ref.Tail;            
    }
    Xmit_Queue.Record[0].Msg_Buffer[2] = Event_count.Byte.Byte1; 			   /* Send nuber of events */
	Xmit_Queue.Record[0].Msg_Buffer[3] = Event_count.Byte.Byte2;
	Xmit_Queue.Record[0].Msg_Buffer[4] = Event_count.Byte.Byte3;
	Xmit_Queue.Record[0].Msg_Buffer[5] = Event_count.Byte.Byte4;
    Xmit_Queue.Record[0].Msg_Buffer[6] = S_Time.Byte.Byte1; 			   /* First event time */
	Xmit_Queue.Record[0].Msg_Buffer[7] = S_Time.Byte.Byte2;
	Xmit_Queue.Record[0].Msg_Buffer[8] = S_Time.Byte.Byte3;
	Xmit_Queue.Record[0].Msg_Buffer[9] = S_Time.Byte.Byte4;
    Xmit_Queue.Record[0].Msg_Buffer[10]  = E_Time.Byte.Byte1; 			   /* End event time */
	Xmit_Queue.Record[0].Msg_Buffer[11]  = E_Time.Byte.Byte2;
	Xmit_Queue.Record[0].Msg_Buffer[12]  = E_Time.Byte.Byte3;
	Xmit_Queue.Record[0].Msg_Buffer[13]  = E_Time.Byte.Byte4;
    CheckSum.Word = Crc16(XMIT_QUEUE,14);				/* Calculate CRC 16 for Xmit Buffer */
	Xmit_Queue.Record[0].Msg_Buffer[14] = CheckSum.Byte.Lo;				/* CRC low Byte */
	Xmit_Queue.Record[0].Msg_Buffer[15] = CheckSum.Byte.Hi;				/* CRC High Byte */
	Xmit_Queue.Record[0].Msg_Buffer[16] = 0x0D;  								/* Add CR */
	Xmit_Queue.Record[0].Msg_Buffer[17] = 0x0A;  								/* Add LF */
	Xmit_Queue.Record[0].Host_Msg_Length = 18;
	Xmit_Queue.Record[0].Smc_Msg_Length = 16;
	Xmit_Queue.Index = 0;
    Xmit_Queue.Record_Count =1;
	Xmit_Queue.Record_Index =0;
	Status.Flags.Query_Pending = FALSE;
}
