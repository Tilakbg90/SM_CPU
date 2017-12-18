/*********************************************************************************************************************
* Project		: 	Single Section Digital Axle Counter
* Version		: 	2.0 
* Revision      :	1	
* Filename      : 	comm_host.c
* Target MCU	: 	PIC24FJ256GB210   
*  Compiler     : 	XC16 Compiler V1.31
* Author		:	EM003
* Date          :	
* Company Name  : 	Insys Digital Systems
* Modification History:
* 				Rev No			Date		Description
* 				 --				 --				--    
* Functions	:   
*               void Initialise_Host_CommSch(void);
* 				void SetupCOM2BaudRate(BYTE uchBaudRate);
* 				void Update_Host_Sch_State(void);
* 				void Decrement_Host_Sch_msTmr(void);
* 				void Receive_COM2_Message(void);
* 				void Clear_COM2_Receive_Buffer(void);
* 				void Clear_Com2_Error(void);
*********************************************************************************************************************/
#include <xc.h>
#include <time.h>

#include "COMMON.h"
#include "command_proc.h"
#include "DRV_MEM.h"
#include "EVENTS.h"
#include "CRC16.h"
#include "comm_host.h"

extern char USB_Out_Buffer[64];
extern char USB_In_Buffer[64];
extern sm_status_t Status;					/* From cpu_sm.c */
extern const BYTE uchCommand_Length_Table[7][2];	/* From command_proc.c */
extern const BYTE BitMask_List[8];					/* From cpu_sm.c */
extern query_param_t        Query_Param;			/* From command_proc.c */
extern xmit_queue_info_t    Xmit_Queue;				/* From command_proc.c */

host_sch_info_t      		Host_Sch_Info;			/* Structure holds host comm scheduler information */
host_recv_msg_info_t 		Com2RecvObject;			/* COM2: Message Receiving Buffer  */
BYTE Bytes_read;

/*********************************************************************************
* File name 			:comm_host.c
* Function Name		:void Initialise_Host_CommSch(void)
* Created By			:EM003
* Date Created		:
* Modification History:
* 					Rev No			Date		Description
* 					 --				 --				--
* Tracability:
* 		SRS()    	:
* 
* Abstract			: Initialise host communication scheduler.
* Algorithm			:
* Description			: 
* Input Element		:None
* Output Element		:void
* 
***********************************************************************************/
void Initialise_Host_CommSch(void)
{
	Host_Sch_Info.State      = HOST_SCHEDULER_NOT_STARTED;  /*  set host comm scheduler to "HOST_SCHEDULER_NOT_STARTED" state  */ 
	Host_Sch_Info.BytePeriod = BYTE_PERIOD_1MS;				/*  set byte PERIOD to 1ms for 9600 Baud Rate*/ 
	Com2RecvObject.State = RECV_BUFFER_EMPTY;				/*  initialise host comm receiving scheduler state to "buffer empty"*/ 
	Com2RecvObject.Index = 0;								/*  intialise receiving scheduler index value to zero */ 
	Com2RecvObject.Timeout_ms = 0;							/*  intialise receiving scheduler timeout value to zero */ 
	Clear_COM2_Receive_Buffer();							/*  clear receiving buffer */ 
	SetupCOM2BaudRate((BYTE)BAUDRATE_9600);						/*  set host comm baudrate to 9600bps */ 
}
/*********************************************************************************
* File name 			:comm_host.c
* Function Name		:void SetupCOM2BaudRate(BYTE uchBaudRate)
* Created By			:EM003
* Date Created		:
* Modification History:
* 					Rev No			Date		Description
* 					 --				 --				--
* Tracability:
* 		SRS()    	:
* 
* Abstract			: sets Baudrate for host communication 
* Algorithm			:
* Description			: 
* Input Element		:uchBaudRate
* Output Element		:void
* 
**********************************************************************************/

void SetupCOM2BaudRate(BYTE uchBaudRate)
{
    TRISFbits.TRISF3 = SET_LOW;       // RF3 is Tx, Output
    TRISFbits.TRISF2 = SET_HIGH;       // RF2 is Rx, Input

    /* Configure Remappable pins */
    // Unlock Registers
    __builtin_write_OSCCONL(OSCCON & 0xbf); //clear the bit 6 of OSCCONL to unlock Pin Re-map
    RPINR19bits.U2RXR = 30;
    RPOR8bits.RP16R = 5;
    // Lock Registers
    
    __builtin_write_OSCCONL(OSCCON | 0x40); //set the bit 6 of OSCCONL to lock Pin Re-map
    //******************************************************************

    U2MODE = 0;
    U2STA = 0;
    U2MODEbits.RTSMD = SET_HIGH;   // Operate in Simplex Mode
    U2MODEbits.BRGH =  SET_HIGH;    // Low Speed Clocks to BRG @ 16x of Fp
    /*
     * FCY = FOSC/2 = 32MHz/2 = 16Mz
     * U1BRG =FCY/(4 * Baud Rate) - 1
     * U1BRG = 16000000/(4*1200) - 1
     * U1BRG = 3332
     */
    switch (uchBaudRate)
	{	
		case BAUDRATE_1200:
			U2BRG = SPBRG_FOR_1200;
			break;
		case BAUDRATE_2400:
			U2BRG = SPBRG_FOR_2400;
            break;
		case BAUDRATE_9600:
			U2BRG = SPBRG_FOR_9600;
			break;
		case BAUDRATE_19200:
			U2BRG = SPBRG_FOR_19200;
			break;	
        default:
            break;
	}
    U2MODEbits.UARTEN = SET_HIGH;  // Enable UART
    U2STA = 0x0000;
}
/*********************************************************************************
* File name 		:comm_host.c
* Function Name		:void Start_Host_Communication(void)
* Created By		:EM003
* Date Created		:
* Modification History:
* 					Rev No			Date		Description
* 					 --				 --				--
* Tracability:
* 		SRS()    	:
* 
* Abstract			:This function starts Host comm scheduler.
* Algorithm			:
* Description		: 
* Input Element		:None
* Output Element	:void
* 
***********************************************************************************/
void Start_Host_Communication(void)
{
	U2STAbits.UTXEN = SET_HIGH;	/* Enable Transmitter */
    Host_Sch_Info.State = HOST_COMM_SCHEDULER_IDLE;
}
/*********************************************************************************
File name 			:comm_host.c
Function Name		:void Update_Host_Sch_State(void)
Created By			:EM003
Date Created		:
Modification History:
					Rev No			Date		Description
					 --				 --				--
Tracability:
		SRS()    	:

Abstract			:Host communication scheduler to Interface with PC through RS232 port
Algorithm			:
					 This Scheduler have two state, One refers to Scheduler current State and another 
					 to Next State of Scheduler.					
                      
					*.Responding to Single Record Reply -Query i.e Get Time,Set Time,Erase Events, Dac Status.
					 1.Initially host comm scheduler will be in Idle state
					 2.When a single data query comes from PC, the response message is build by 
					   Process_Host_Command() function.
					 3.Then it sets host comm Scheduler state to "TRANSMIT_SINGLE_RECORD_FOR_HOST"
					   and Scheduler next State to "HOST_COMM_SCHEDULER_IDLE"
					 4.After one Record is transmitted to PC, it goes to idle State.
				
					
					*.Responding to Multiple Record Reply - Query	i.e (Read Events)	
					 1.Initially host comm scheduler will be in Idle state
					 2.when a Multiple Record Query comes from PC, 
					   Process_Host_Command() function sets scheduler to  "BUILD_RECORDS" state.
					 3.It will build 8 Records and fill the Xmit_Queue.if Last Record detected or
					   Xmit Queue is Filled with 8 Records, it will Change scheduler state to "TRANSMIT_RECORDS".
					   If All records are read from Events Memory and there is no msg in Xmit Queue to transmit, 
					   then it Change scheduler state  to "HOST_COMM_SCHEDULER_IDLE".
					 4.In this State All Records available in Xmit Queue is Transmitted. For transmitting 
					   each Record, Scheduler will goto "TRANSMIT_SINGLE_RECORD_FOR_HOST" state and 
					   next state of Scheduler is set to "TRANSMIT_RECORDS".So After Transmitting the Record ,
					   Scheduler Comebacks to "TRANSMIT_RECORDS" State.
					   Once All Records Transmitted, it will check the Mode of Transfer required for Host (PC).
					   if it is "ASYNCHRONOUS_MODE" , Scheduler will goto "BUILD_RECORDS" for Transmitting Next Batch of Records.  	 	
					   otherwise the mode Selected is "SYNCHRONOUS_MODE" and then it will goto "WAIT_FOR_REPLY" state.
					5. When the REPLY command comes from the Host, Process_Host_Command() function analyse the Reply Status.
					   If Host received all records correctly, then it will set the Scheduler state to  "BUILD_RECORDS" for Transmitting 
					   Next Batch of Records. otherwise it sets Scheduler state to "PROCESS_HOST_REPLY_COMMAND".
					6. In "PROCESS_HOST_REPLY_COMMAND" State, all corrupted records are retransmitted.
					   For transmitting each Record, Scheduler will goto "TRANSMIT_SINGLE_RECORD_FOR_HOST" state and 
					   next state of Scheduler is set to "PROCESS_HOST_REPLY_COMMAND".So After Transmitting the Record ,
					   Scheduler Comebacks to "PROCESS_HOST_REPLY_COMMAND" State.
					   After all corrupted records are retransmitted, Scheduler state is set to "WAIT_FOR_REPLY".  		 
Description			: 
Input Element		:None
Output Element		:void

**********************************************************************************/
void Update_Host_Sch_State(void)
{
    BYTE count_byte, Max_len;
	Receive_COM2_Message();					/* Receive data from RS232 port which connects to PC */

	switch (Host_Sch_Info.State)
	{
        case HOST_SCHEDULER_NOT_STARTED:
            break;
		case HOST_COMM_SCHEDULER_IDLE:							
			break;
		case BUILD_RECORDS_FOR_HOST:			
			if(Xmit_Queue.Record_Index == 0 &&
			   Query_Param.uchIs_Next_Available == FALSE)   
			{
				/* No more events to transmit */
				/* By Setting Query_Pending Status Bit, the Command Processor is made ready for Next Command */
				Status.Flags.Query_Pending = FALSE;																							  
   			    Host_Sch_Info.State = HOST_COMM_SCHEDULER_IDLE;
                            Host_Sch_Info.Tx_USB_Logged_events = 0;
				break;
			}
			if(Xmit_Queue.Record_Index >= MAX_XMIT_QUEUE_SIZE ||
			   Query_Param.uchIs_Next_Available == FALSE)
			{
				/* Transmit Queue Filled, So, go for Transmission */
				Host_Sch_Info.State = TRANSMIT_RECORDS_FOR_HOST;
				Host_Sch_Info.Timeout_ms = 0;
				Xmit_Queue.Record_Index  = 0;
				break;
			}			
			if(Is_Serial_EEPROM_Write_Complete())			
			{
				/*Read the events from Serial eeprom and copy the Event Record to Xmit Queue. */
				Build_Next_Logged_Event_Reply(); 				    	
			}
			break;
		case TRANSMIT_RECORDS_FOR_HOST:
			if( Xmit_Queue.Record_Index >= MAX_XMIT_QUEUE_SIZE ||
				Xmit_Queue.Record_Index >= Xmit_Queue.Record_Count)
			{
 				Xmit_Queue.Record_Index = 0;	
				if(Xmit_Queue.Mode == ASYNCHRONOUS_MODE)
				{
				  /*In this Mode, Scheduler won't wait for Host Reply, It will go for Next Record Transmission */	
				  Host_Sch_Info.State = BUILD_RECORDS_FOR_HOST;
				  Xmit_Queue.Record_Count = 0;	
				}
				else
				{   				 
				  Host_Sch_Info.State = HOST_WAIT_FOR_REPLY;	
				  Host_Sch_Info.Timeout_ms = HOST_REPLY_WAIT_TIME;
				}
				break;			
			}
			if(Host_Sch_Info.Timeout_ms == TIMEOUT_EVENT)
			{
			   Xmit_Queue.Index  = 0;
		       Host_Sch_Info.State      = TRANSMIT_SINGLE_RECORD_FOR_HOST;
			   Host_Sch_Info.Next_State = TRANSMIT_RECORDS_FOR_HOST;				
			}
			break;				
		case HOST_WAIT_FOR_REPLY:
			if(Host_Sch_Info.Timeout_ms == TIMEOUT_EVENT)
			{
			  /* Host System failed to Reply the Status of Reception of Records,
				 So Stop transmitting further Records and Scheduler will goto Idle State */		
			  Status.Flags.Query_Pending = FALSE;	
   			  Host_Sch_Info.State = HOST_COMM_SCHEDULER_IDLE;								
			}
			break;
		case PROCESS_HOST_REPLY_COMMAND:
			if(Host_Sch_Info.Timeout_ms == TIMEOUT_EVENT)
			{
				if(Xmit_Queue.Record_Index >= MAX_XMIT_QUEUE_SIZE ||
				   Xmit_Queue.Record_Index >= Xmit_Queue.Record_Count)
				{
				   /* All Corrupted Records ReTransmitted, So Scheduler goes to "Wait For Reply" State*/	
				   Xmit_Queue.Record_Index = 0;			
				   Host_Sch_Info.State = HOST_WAIT_FOR_REPLY;
				   break;	
				}				
				if(Xmit_Queue.Record_Rx_Status & BitMask_List[Xmit_Queue.Record_Index])
				{
				  /* Data received at remote side correctly, So no need to Retransmit it */
			 	 	Xmit_Queue.Record_Index = Xmit_Queue.Record_Index + 1;				
				}
				else
				{			
					Xmit_Queue.Index  = 0;
					Host_Sch_Info.State      = TRANSMIT_SINGLE_RECORD_FOR_HOST;
					Host_Sch_Info.Next_State = PROCESS_HOST_REPLY_COMMAND;
				}
			}
			break;			
		case TRANSMIT_SINGLE_RECORD_FOR_HOST:
			if (Xmit_Queue.Index >= Xmit_Queue.Record[Xmit_Queue.Record_Index].Host_Msg_Length ||
				Xmit_Queue.Index >= MAXIMUM_MESSAGE_LENGTH)
			{
				/* All Bytes of the Record are Transmitted, So Scheduler will goto Next State value */
				Host_Sch_Info.State      = Host_Sch_Info.Next_State;
				Host_Sch_Info.Next_State = HOST_COMM_SCHEDULER_IDLE;
				Xmit_Queue.Record_Index = Xmit_Queue.Record_Index + 1;
				Host_Sch_Info.Timeout_ms = RECORD_GAP_TIME;
				break;
			}
            if(Host_Sch_Info.USB_Connection_status == 1)
            {
                Max_len = Xmit_Queue.Record[Xmit_Queue.Record_Index].Host_Msg_Length;
                for(count_byte = 0; count_byte<Max_len;count_byte++)
                    USB_In_Buffer[count_byte] = Xmit_Queue.Record[Xmit_Queue.Record_Index].Msg_Buffer[count_byte];
                
                Host_Sch_Info.Tx_USB_Byte_Count = Xmit_Queue.Record[Xmit_Queue.Record_Index].Host_Msg_Length;
                Host_Sch_Info.Tx_USB_Data_available = 1;
                Host_Sch_Info.State      = Host_Sch_Info.Next_State;
				Host_Sch_Info.Next_State = HOST_COMM_SCHEDULER_IDLE;
				Xmit_Queue.Record_Index = Xmit_Queue.Record_Index + 1;
				Host_Sch_Info.Timeout_ms = RECORD_GAP_TIME;
				break;
            }
            else
            {
                if (U2STAbits.TRMT)
                {
                        U2TXREG = Xmit_Queue.Record[Xmit_Queue.Record_Index].Msg_Buffer[Xmit_Queue.Index];
                        Xmit_Queue.Index = Xmit_Queue.Index + 1;
                }
            }
			break;	
        default:
            break;
	}
}
/*********************************************************************************
* File name 		:comm_host.c
* Function Name		:void Decrement_Host_Sch_msTmr(void)
* Created By		:EM003
* Date Created		:
* Modification History:
* 					Rev No			Date		Description
* 					 --				 --				--
* Tracability:
* 		SRS()    	:
* 
* Abstract			:One milliSecond Timer Variables present Host_sch_info and
* 					 com2 receive object are decremented.
* Algorithm			:
* Description		: 
* Input Element		:None
* Output Element	:void
* 
**********************************************************************************/
void Decrement_Host_Sch_msTmr(void)
{
	if (Host_Sch_Info.Timeout_ms > 0)
	{
		Host_Sch_Info.Timeout_ms = Host_Sch_Info.Timeout_ms - 1;
	}
    if (Host_Sch_Info.USB_Timeout_ms > 0)
	{
		Host_Sch_Info.USB_Timeout_ms = Host_Sch_Info.USB_Timeout_ms - 1;
	}
	if (Com2RecvObject.Timeout_ms > 0)
	{
		Com2RecvObject.Timeout_ms = Com2RecvObject.Timeout_ms - 1;
	}
}
/*********************************************************************************
*File name 			:comm_host.c
*Function Name		:void Receive_COM2_Message(void)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Receive Command from RS232 Port (PC). 
*Algorithm			:1.Intialise receive scheduler to buffer empty state.
*				     2.Check for any data received.If data received, store into Receive Buffer.
*					   If Data received is command id, then get the message length for the command from the table.
*					 3.If No of Bytes Received is greater than or equal to Message lengh, then send Receive Buffer for crc check.
*					 4.If crc function return value is zero, then received message is Vaild. So go for process host command.
*					 5.If crc function return value is non zero, then received message is InVaild. So go for buffer empty state.
*Description			: 
*Input Element		:None
*Output Element		:void
*
***********************************************************************************/

void Receive_COM2_Message(void)
{
	BYTE uchData=0, count = 0;
    if (U2STAbits.OERR)
    	Clear_Com2_Error();
    if(Host_Sch_Info.USB_Connection_status == 1 && Host_Sch_Info.Rx_USB_Data_available == 1)
    {
        Host_Sch_Info.Rx_USB_Data_available = 0;
        while (Bytes_read!=0)
        {
            uchData = USB_Out_Buffer[count++];
            Bytes_read--;
            Com2RecvObject.Timeout_ms = HOST_COMMAND_RECEIVE_TIMEOUT;    /* maximum time allowed between data bytes */
            switch (Com2RecvObject.State)
            {
                case CHECK_CRC16:
                    break;
                case RECV_BUFFER_EMPTY:
                    Com2RecvObject.Msg_Buffer[0] = uchData;
                    Com2RecvObject.Index = 1;
                    Com2RecvObject.State = RECE_COMMAND;
                    break;
                case RECE_COMMAND:
                    Com2RecvObject.Msg_Buffer[Com2RecvObject.Index] = uchData;
                    Com2RecvObject.Index = Com2RecvObject.Index + 1;
                    if(uchData < NO_OF_HOST_COMMANDS)
                    {
                       Com2RecvObject.Msg_Length = uchCommand_Length_Table[uchData][1];
                       Com2RecvObject.State = RECV_DATA;
                    }
                    break;
                case RECV_DATA:
                    Com2RecvObject.Msg_Buffer[Com2RecvObject.Index] = uchData;
                    Com2RecvObject.Index = Com2RecvObject.Index + 1;
                    if(Com2RecvObject.Index >= Com2RecvObject.Msg_Length)
                    {
                        Com2RecvObject.State = CHECK_CRC16;
                    }
                    break;
                default:
                    break;
            }
        }
    }
    if(Host_Sch_Info.USB_Connection_status != 1)
    {
        while (IFS1bits.U2RXIF)
        {
            uchData = (BYTE)U2RXREG;
            IFS1bits.U2RXIF = 0;
            Com2RecvObject.Timeout_ms = HOST_COMMAND_RECEIVE_TIMEOUT;    /* maximum time allowed between data bytes */
            switch (Com2RecvObject.State)
            {
                case CHECK_CRC16:
                    break;
                case RECV_BUFFER_EMPTY:
                    Com2RecvObject.Msg_Buffer[0] = uchData;
                    Com2RecvObject.Index = 1;
                    Com2RecvObject.State = RECE_COMMAND;
                    break;
                case RECE_COMMAND:
                    Com2RecvObject.Msg_Buffer[Com2RecvObject.Index] = uchData;
                    Com2RecvObject.Index = Com2RecvObject.Index + 1;
                    if(uchData < NO_OF_HOST_COMMANDS)
                    {
                       Com2RecvObject.Msg_Length = uchCommand_Length_Table[uchData][1];
                       Com2RecvObject.State = RECV_DATA;
                    }
                    break;
                case RECV_DATA:
                    Com2RecvObject.Msg_Buffer[Com2RecvObject.Index] = uchData;
                    Com2RecvObject.Index = Com2RecvObject.Index + 1;
                    if(Com2RecvObject.Index >= Com2RecvObject.Msg_Length)
                    {
                        Com2RecvObject.State = CHECK_CRC16;
                    }
                    break;
                default:
                    break;
            }
        }
    }
	if(Com2RecvObject.State == CHECK_CRC16)
	{
	   if (Crc16(COM2_RECV_OBJ,Com2RecvObject.Msg_Length-2) == (UINT16)((UINT16)Com2RecvObject.Msg_Buffer[Com2RecvObject.Msg_Length-2]<<8)+Com2RecvObject.Msg_Buffer[Com2RecvObject.Msg_Length-1])
       {
	      /* Command received from Host Computer */
		  Process_Host_Command();
	   }
	  Com2RecvObject.State = RECV_BUFFER_EMPTY;
	}
	if(Com2RecvObject.Timeout_ms == TIMEOUT_EVENT)
	{
     /* There is no reception of data for last 25Sec , So Reset FSA for New Packet */
	  Com2RecvObject.State = RECV_BUFFER_EMPTY;
	}
}

/*********************************************************************************
*File name 			:comm_host.c
*Function Name		:void Clear_COM2_Receive_Buffer(void)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Function that clears receiving buffer.
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Clear_COM2_Receive_Buffer(void)
{
	BYTE uchCnt =0;

	for (uchCnt = 0; uchCnt < MAX_RECV_MESSAGE_LENGTH; uchCnt++)
	{
		Com2RecvObject.Msg_Buffer[uchCnt] = (BYTE) 0;
	}
}
/*********************************************************************************
*File name 			:comm_host.c
*Function Name		:void Clear_Com2_Error(void)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:This function Clears the communication errors,if occured.
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Clear_Com2_Error(void)
{
  BYTE uchCnt=0;

	if (U2STAbits.OERR)
	{
		/* Overrun Error! Clear the error */
		U2MODEbits.UARTEN = SET_LOW;
        U2MODEbits.UARTEN = SET_HIGH;
	}
	if (U2STAbits.FERR)
	{
		/* Framing Error! Clear the error */
		uchCnt= (BYTE)U2RXREG;
        U2MODEbits.UARTEN = SET_LOW;
        U2MODEbits.UARTEN = SET_HIGH;
	}

}

