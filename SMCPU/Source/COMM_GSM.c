/*********************************************************************************************************************
 *	Project		: 	Single Section Digital Axle Counter
 *	Version		: 	2.0
 *	Revision	:	1
 *	Filename	: 	COMM_GSM.c
 *	Target MCU	: 	PIC24FJ256GB210
 *    Compiler	: 	XC16 Compiler V1.31
 *	Author		:	EM003
 *	Date		:
 *	Company Name: 	Insys Digital Systems
 *	Modification History:
 *					Rev No			Date		Description
 *					 --				 --				--
 *	Functions	:   
 *                  void Initialise_GSM_CommSch(void);
 *                  void SetupCOM3BaudRate(BYTE uchBaudRate);
 *                  void Start_GSM_Communication(void);
 *                  void Update_GSM_Sch_State(void);
 *                  void Check_change_number(void);
 *                  void Check_Error(void);
 *                  void Decrement_GSM_Sch_msTmr(void);
 *                  void Receive_COM3_Message(void);
 *                  void Clear_COM3_Receive_Buffer(void);
 *                  void Clear_COM3_Transmit_Buffer(void);
 *                  void Clear_Com3_Error(void);
 * 
 **********************************************************************************************************************/
#include <xc.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <limits.h>

#include "COMMON.h"
#include "command_proc.h"
#include "DRV_MEM.h"
#include "EVENTS.h"
#include "CRC16.h"
#include "COMM_GSM.h"

BYTE System_GSM_on =0, Check_msg = 0, Delete_message = 0, send_confirmation =0;
GSM_sch_info_t  GSM_Sch_Info;			/* Structure holds GSM comm scheduler information */
GSM_recv_msg_info_t Com3RecvObject,Com3TransObject;			/* COM2: Message Receiving Buffer  */
BOOL P_status; /*lint -e552 */
char Error_message[70][35] =
{
"Prelimnary Relay 2 Failure",
"Prelimnary Relay 1 Failure",
"Critical Relay 2 Failure",
"Critical Relay 1 Failure",
"Transient Power Failure at DS1",
"Transient Power Failure at DS2",
"Transient Power Failure at US1",
"Transient Power Failure at US2",
"Inoperative Network address",
"Incorrect CRC of Code",
"Inoperative Configuration",
"Inoperative Counts",
"Ram test Failed",
"Direct Out Count",
"AD SUP Missing",
"",
"AD1 SUP Low",
"AD2 SUP Low",
"AD1 Pulsing",
"AD2 Pulsing",
"AD State Missing",
"AD SUP Pulsating",
"AD State Fail",
"AD not Pulsating",
"AD1 Board Missing",
"AD2 Board Missing",
"Event Logger Board Missing",
"Modem Board Missing",
"VRPR1 Board Missing",
"VRPR1 Board Missing",
"VRPR2 Board Missing",
"Associate CPU not Active",
"No Carrier in Modem",
"Interprocess Failure",
"Failure at DS",
"Failure at US",
"AD Pulse Mismatch",
"Bootup AD Fail",
"Axle Decptive",
"Double Coil Influence",
"Com Fail - LU1 to US1",
"Com Fail - LU1 to US2",
"Com Fail - LU1 to DS1",
"Com Fail - LU1 to DS2",
"Com Fail - US1 to LU1",
"Com Fail - US2 to LU1",
"Com Fail - DS1 to LU1",
"Com Fail - DS2 to LU1",
"Com Fail - LU2 to US1",
"Com Fail - LU2 to US2",
"Com Fail - LU2 to DS1",
"Com Fail - LU2 to DS2",
"Com Fail - US1 to LU2",
"Com Fail - US2 to LU2",
"Com Fail - DS1 to LU2",
"Com Fail - DS2 to LU2"
};

char Feedback_message[20][35] =
{
    "YLED feedback incorrect",
    "RLED feedback incorrect",
    "SPK feedback incorrect",
    "System Theft detected",
    "System Door Open",
    "YLED feedback restored",
    "RLED feedback restored",
    "SPK feedback restored",
    "System Theft restored",
    "System Door Closed"
};

char GSM_AT_Enter[3] = {0x0D,0x0A};
char GSM_end_msg = 0x1A;

char GSM_AT_Command[20][25] =
{
"ATE0",
"AT+CPIN?",
"AT+CFUN?",
"AT+CMGF=1",
"AT+CSMP=17,167,0,0",
"AT+CSCS=\"GSM\"",
"AT+CMGS=",
"AT+CMGL=\"REC UNREAD\"",
"AT+CMGD=1,4"
};

char String_Change_Number[] = "ADD NUMBER";
char Phone_number[6][15]=
{
    "+919740629290",
    "+919740629290",
    "+919740629290",
    "+919740629290",
    "+919740629290",
};



/*********************************************************************************
 * File name 			:comm_GSM.c
 * Function Name		:void Initialise_GSM_CommSch(void)
 * Created By			:EM003
 * Date Created		:
 * Modification History:
 * 					Rev No			Date		Description
 * 					 --				 --				--
 * Tracability:
 * 		SRS()    	:
 * 
 * Abstract			: Initialise GSM communication scheduler.
 * Algorithm			:
 * Description			:
 * Input Element		:None
 * Output Element		:void
 * 
 ***********************************************************************************/
void Initialise_GSM_CommSch(void)
{
    TRISDbits.TRISD6 = SET_HIGH;
    TRISDbits.TRISD7 = SET_HIGH;    
    TRISFbits.TRISF0 = SET_HIGH;
    TRISFbits.TRISF1 = SET_HIGH;
    
    TRISEbits.TRISE2 = SET_LOW;
    TRISEbits.TRISE3 = SET_LOW;
    TRISEbits.TRISE4 = SET_LOW;

    LATEbits.LATE2 = SET_HIGH;
    LATEbits.LATE3 = SET_HIGH;
    LATEbits.LATE4 = SET_LOW;

    
    if(GSM_Sch_Info.GSM_Enable == 0)
        return;

	GSM_Sch_Info.State      = GSM_INITIALIZE;  /*  set GSM comm scheduler to "GSM_SCHEDULER_NOT_STARTED" state  */
    GSM_Sch_Info.AT_command = GSM_ATE0;
	GSM_Sch_Info.BytePeriod = BYTE_PERIOD_1MS;				/*  set byte PERIOD to 1ms for 9600 Baud Rate*/
    GSM_Sch_Info.GSM_Init_Success = 0;
	Com3RecvObject.State = GSM_IGNORE_RECV_DATA;				/*  initialise GSM comm receiving scheduler state to "buffer empty"*/
	Com3RecvObject.Index = 0;								/*  intialise receiving scheduler index value to zero */
	Com3RecvObject.Timeout_ms = 0;							/*  intialise receiving scheduler timeout value to zero */
	Clear_COM3_Receive_Buffer();							/*  clear receiving buffer */
    Clear_COM3_Transmit_Buffer();
	SetupCOM3BaudRate((BYTE)BAUDRATE_1200);						/*  set GSM comm baudrate to 9600bps */
    P_status = Read_Phone_Number();
}
/*********************************************************************************
* File name 		:comm_GSM.c
* Function Name		:void SetupCOM3BaudRate(BYTE uchBaudRate)
* Created By		:EM003
* Date Created		:
* Modification History:
* 					Rev No			Date		Description
* 					 --				 --				--
* Tracability:
* 		SRS()    	:
* 
* Abstract			: sets Baudrate for GSM communication
* Algorithm			:
* Description		:
* Input Element		:uchBaudRate
* Output Element	:void
* 
************************************************************************************/
void SetupCOM3BaudRate(BYTE uchBaudRate)
{
    TRISFbits.TRISF13 = SET_LOW;       // RF5 is Tx, Output
    TRISFbits.TRISF12 = SET_HIGH;       // RF4 is Rx, Input

    /* Configure Remappable pins */
    // Unlock Registers
    __builtin_write_OSCCONL(OSCCON & 0xbf); /*lint -e526 -e628*/
    RPINR17bits.U3RXR = 32;
    RPOR15bits.RP31R = 28;
    // Lock Registers
    __builtin_write_OSCCONL(OSCCON | 0x40); /*lint -e526 -e628*/

    U3MODE = 0;
    U3STA =  0;
    U3MODEbits.RTSMD = SET_HIGH;   // Operate in Simplex Mode
    U3MODEbits.BRGH = SET_HIGH;    // Low Speed Clocks to BRG @ 16x of Fp
    /*
     * FCY = FOSC/2 = 32MHz/2 = 16Mz
     * U1BRG =FCY/(4 * Baud Rate) - 1
     * U1BRG = 16000000/(4*1200) - 1
     * U1BRG = 3332
     */
	switch (uchBaudRate)
	{
		case BAUDRATE_1200:
			U3BRG = SPBRG_FOR_1200;
			break;
		case BAUDRATE_2400:
			U3BRG = SPBRG_FOR_2400;
            break;
		case BAUDRATE_9600:
			U3BRG = SPBRG_FOR_9600;
			break;
		case BAUDRATE_19200:
			U3BRG = SPBRG_FOR_19200;
			break;
        default:
            break;
	}

    IEC5bits.U3ERIE = SET_LOW;
    IEC5bits.U3RXIE = SET_LOW;
    IEC5bits.U3TXIE = SET_LOW;
    IFS5bits.U3ERIF = SET_LOW;
    IFS5bits.U3RXIF = SET_LOW;
    IFS5bits.U3TXIF = SET_LOW;

    U3MODEbits.UARTEN = SET_HIGH;  // Enable UART
    U3STAbits.UTXEN = SET_HIGH;
    U3STAbits.UTXISEL0 = SET_HIGH;
    U3STAbits.UTXISEL1 = SET_LOW;
    U3STAbits.URXISEL  = SET_LOW;
    IFS5bits.U3RXIF = SET_LOW;
}
/*********************************************************************************
* File name 		:comm_GSM.c
* Function Name		:void Start_GSM_Communication(void)
* Created By		:EM003
* Date Created		:
* Modification History:
* 					Rev No			Date		Description
* 					 --				 --				--
* Tracability:
* 		SRS()    	:
* 
* Abstract			:This function starts GSM comm scheduler.
* Algorithm			:
* Description		:
* Input Element		:None
* Output Element	:void
* 
**********************************************************************************/
void Start_GSM_Communication(void)
{
    U3STAbits.UTXEN = SET_HIGH;	/* Enable Transmitter */
    GSM_Sch_Info.State = GSM_COMM_SCHEDULER_WAIT;
}
/*********************************************************************************
* File name 		:comm_GSM.c
* Function Name		:void Update_GSM_Sch_State(void)
* Created By		:EM003
* Date Created		:
* Modification History:
* 					Rev No			Date		Description
* 					 --				 --				--
* Tracability:
* 		SRS()    	:
* 
* Abstract			:GSM communication scheduler to Interface with PC through RS232 port
* Algorithm			:
* 					 This Scheduler have two state, One refers to Scheduler current State and another
* 					 to Next State of Scheduler.
* 
* 					*.Responding to Single Record Reply -Query i.e Get Time,Set Time,Erase Events, Dac Status.
* 					 1.Initially GSM comm scheduler will be in Idle state
* 					 2.When a single data query comes from PC, the response message is build by
* 					   Process_GSM_Command() function.
* 					 3.Then it sets GSM comm Scheduler state to "TRANSMIT_SINGLE_RECORD_FOR_GSM"
* 					   and Scheduler next State to "GSM_COMM_SCHEDULER_IDLE"
* 					 4.After one Record is transmitted to PC, it goes to idle State.
* 
* 
* 					*.Responding to Multiple Record Reply - Query	i.e (Read Events)
* 					 1.Initially GSM comm scheduler will be in Idle state
* 					 2.when a Multiple Record Query comes from PC,
* 					   Process_GSM_Command() function sets scheduler to  "BUILD_RECORDS" state.
* 					 3.It will build 8 Records and fill the Xmit_Queue.if Last Record detected or
* 					   Xmit Queue is Filled with 8 Records, it will Change scheduler state to "TRANSMIT_RECORDS".
* 					   If All records are read from Events Memory and there is no msg in Xmit Queue to transmit,
* 					   then it Change scheduler state  to "GSM_COMM_SCHEDULER_IDLE".
* 					 4.In this State All Records available in Xmit Queue is Transmitted. For transmitting
* 					   each Record, Scheduler will goto "TRANSMIT_SINGLE_RECORD_FOR_GSM" state and
* 					   next state of Scheduler is set to "TRANSMIT_RECORDS".So After Transmitting the Record ,
* 					   Scheduler Comebacks to "TRANSMIT_RECORDS" State.
* 					   Once All Records Transmitted, it will check the Mode of Transfer required for GSM (PC).
* 					   if it is "ASYNCHRONOUS_MODE" , Scheduler will goto "BUILD_RECORDS" for Transmitting Next Batch of Records.
* 					   otherwise the mode Selected is "SYNCHRONOUS_MODE" and then it will goto "WAIT_FOR_REPLY" state.
* 					5. When the REPLY command comes from the GSM, Process_GSM_Command() function analyse the Reply Status.
* 					   If GSM received all records correctly, then it will set the Scheduler state to  "BUILD_RECORDS" for Transmitting
* 					   Next Batch of Records. otherwise it sets Scheduler state to "PROCESS_GSM_REPLY_COMMAND".
* 					6. In "PROCESS_GSM_REPLY_COMMAND" State, all corrupted records are retransmitted.
* 					   For transmitting each Record, Scheduler will goto "TRANSMIT_SINGLE_RECORD_FOR_GSM" state and
* 					   next state of Scheduler is set to "PROCESS_GSM_REPLY_COMMAND".So After Transmitting the Record ,
* 					   Scheduler Comebacks to "PROCESS_GSM_REPLY_COMMAND" State.
* 					   After all corrupted records are retransmitted, Scheduler state is set to "WAIT_FOR_REPLY".
* Description		:
* Input Element     :None
* Output Element	:void
* 
***********************************************************************************/
void Update_GSM_Sch_State(void)
{
    BYTE temp_uart2, temp_uart;
    if(inspect_event_done == 0)
        return;
    if(GSM_Sch_Info.GSM_Enable == 0)
        return;
    
	Receive_COM3_Message();					/* Receive data from RS232 port which connects to PC */

	switch (GSM_Sch_Info.State)
	{
        case GSM_SCHEDULER_NOT_STARTED:
            break;
        case GSM_COMM_SCHEDULER_WAIT:
            if(GSM_Sch_Info.Timeout_ms == 0)
            {
                Clear_COM3_Transmit_Buffer();
                Clear_COM3_Receive_Buffer();
                Com3TransObject.Index = 0;
                GSM_Sch_Info.AT_command = GSM_CMGL;
                sprintf(&Com3TransObject.Msg_Buffer[Com3TransObject.Index],"%s%s",&GSM_AT_Command[GSM_CMGL][0],&GSM_AT_Enter[0]);
                Com3TransObject.Msg_Length = strlen(&Com3TransObject.Msg_Buffer[0]);
                GSM_Sch_Info.State = TRANSMIT_RECORDS_FOR_GSM;
                Com3RecvObject.State = GSM_CONSIDER_RECV_DATA;
                Com3RecvObject.Index = 0;
                Check_msg = 1;
                GSM_Sch_Info.Ph_number_index = 0;
            }
            else
            {
                Check_Error();
                if(GSM_Sch_Info.Error_Occured==1 && GSM_Sch_Info.Error_Reported==0)
                {
                    GSM_Sch_Info.State = GSM_SEND_MESSAGE;
                    GSM_Sch_Info.Ph_number_index = 0;
                }
            }
            break;
        case GSM_INITIALIZE:
            if(GSM_Sch_Info.AT_command > GSM_CSCS)
            {    
                GSM_Sch_Info.State = GSM_SEND_MESSAGE;
            }
            else if(GSM_Sch_Info.AT_command <= GSM_CSCS)
            {
                GSM_Sch_Info.State = BUILD_RECORDS_FOR_GSM_INIT;
            }
            break;
        case BUILD_RECORDS_FOR_GSM_INIT:
            Clear_COM3_Transmit_Buffer();
            Com3TransObject.Index = 0;
            sprintf(&Com3TransObject.Msg_Buffer[Com3TransObject.Index],"%s%s%s",&GSM_AT_Enter[0],&GSM_AT_Command[GSM_Sch_Info.AT_command][0],&GSM_AT_Enter[0]);
            Com3TransObject.Msg_Length = strlen(&Com3TransObject.Msg_Buffer[0]);
            GSM_Sch_Info.State = TRANSMIT_RECORDS_FOR_GSM;
            Com3RecvObject.State = GSM_CONSIDER_RECV_DATA;
            GSM_Sch_Info.Ph_number_index = 0;
            break;
        case TRANSMIT_RECORDS_FOR_GSM:
            if(Com3TransObject.Index>=(Com3TransObject.Msg_Length))
            {
                Com3RecvObject.Timeout_ms = GSM_REPLY_WAIT_TIME;
                GSM_Sch_Info.State = GSM_WAIT_FOR_REPLY;
            }
            else
            {
                if(IFS5bits.U3TXIF==1)//if (U1STAbits.TRMT==1 && U1STAbits.UTXBF == 0 && Smc1XmitObject.Index < (Smc1XmitObject.Msg_Length))
                {
                    IFS5bits.U3TXIF = 0;
                    while(U3STAbits.UTXBF == 0 && Com3TransObject.Index<(Com3TransObject.Msg_Length))
                    {
                        if(U3STAbits.TRMT == 1)
                        {
                            temp_uart2 = (BYTE)Com3TransObject.Index;
                            temp_uart = Com3TransObject.Msg_Buffer[temp_uart2];
                            U3TXREG = temp_uart;//Smc1XmitObject.Msg_Buffer[temp_uart];
                            Com3TransObject.Index = Com3TransObject.Index + 1;
                        }
                    }
                   while(U3STAbits.TRMT==0);
                }
            }
            break;
        case GSM_SEND_MESSAGE:
            Com3TransObject.Index = 0;
            Clear_COM3_Transmit_Buffer();
            GSM_Sch_Info.AT_command = GSM_CMGS;
            sprintf(&Com3TransObject.Msg_Buffer[Com3TransObject.Index],"%s\"%s\"%s",&GSM_AT_Command[GSM_Sch_Info.AT_command][0],&Phone_number[GSM_Sch_Info.Ph_number_index][0],&GSM_AT_Enter[0]);
            Com3TransObject.Msg_Length = strlen(&Com3TransObject.Msg_Buffer[0]);
            GSM_Sch_Info.State = TRANSMIT_RECORDS_FOR_GSM;
            GSM_Sch_Info.Form_message = 0;
            break;
        case GSM_WAIT_FOR_REPLY:
            if(Com3RecvObject.Timeout_ms == TIMEOUT_EVENT)
            {
                if(GSM_Sch_Info.AT_command <= GSM_CSCS)
                {
                    GSM_Sch_Info.AT_command++;
                    GSM_Sch_Info.State = GSM_INITIALIZE;
                }
                else if(GSM_Sch_Info.AT_command == GSM_CMGS )
                {
                    if(send_confirmation)
                    {
                        send_confirmation = 0;
                        GSM_Sch_Info.State = GSM_COMM_SCHEDULER_WAIT;
                        GSM_Sch_Info.Timeout_ms = GSM_CHECK_INCOMMING_MSG_TIMEOUT;
                        GSM_Sch_Info.AT_command++;
                    }
                    else if(GSM_Sch_Info.Ph_number_index<5)
                    {
                        if(GSM_Sch_Info.Form_message == 0)
                        {
                            GSM_Sch_Info.State = GSM_FORM_MESSAGE;
                            GSM_Sch_Info.Ph_number_index++;
                        }
                        else
                        {
                            GSM_Sch_Info.State = GSM_SEND_MESSAGE;
                        }
                    }
                    else
                    {
                        GSM_Sch_Info.AT_command++;
                        GSM_Sch_Info.Error_Reported = 1;
                        System_GSM_on = 1;
                    }
                }
                else if(GSM_Sch_Info.AT_command > GSM_CSCS)
                {
                    GSM_Sch_Info.State = GSM_COMM_SCHEDULER_WAIT;
                    GSM_Sch_Info.Timeout_ms = GSM_CHECK_INCOMMING_MSG_TIMEOUT;
                }
                if(Check_msg)
                {
                    GSM_Sch_Info.State = PROCESS_GSM_REPLY_COMMAND;
                    Check_msg = 0;
                }
                if(Delete_message)
                {
                    Delete_message = 0;
                    GSM_Sch_Info.State = GSM_COMM_SCHEDULER_WAIT;
                    GSM_Sch_Info.Timeout_ms = GSM_CHECK_INCOMMING_MSG_TIMEOUT;
                    if(GSM_Sch_Info.Change_number_Success == 1)
                    {
                        GSM_Sch_Info.State = GSM_SEND_MESSAGE;
                        break;
                    }
                }
            }
            break;
        case GSM_FORM_MESSAGE:
            Com3TransObject.Index = 0;
            Clear_COM3_Transmit_Buffer();
            GSM_Sch_Info.Form_message = 1;
            if(System_GSM_on == 0)
            {
                if(GSM_Sch_Info.Ph_number_index==6)
                    System_GSM_on = 1;
                sprintf(&Com3TransObject.Msg_Buffer[0],"SSDAC System is turned on %c",GSM_end_msg);
            }
            else if(GSM_Sch_Info.Error_Occured == 1 && GSM_Sch_Info.Error_Reported==0)
            {
                if(GSM_Sch_Info.Ph_number_index==6)
                   GSM_Sch_Info.Error_Reported = 1;
                sprintf(&Com3TransObject.Msg_Buffer[0],"%s %c",&GSM_Sch_Info.Error_Message[0],GSM_end_msg);
            }
            else if(GSM_Sch_Info.Change_number_Success==1)
            {
                GSM_Sch_Info.Change_number_Success = 0;
                sprintf(&Com3TransObject.Msg_Buffer[0],"Number Added successfully.%c",GSM_end_msg);
                send_confirmation = 1;
            }
            Com3TransObject.Msg_Length = strlen(&Com3TransObject.Msg_Buffer[0]);
            GSM_Sch_Info.State = TRANSMIT_RECORDS_FOR_GSM;
            break;
        case PROCESS_GSM_REPLY_COMMAND:
            Check_change_number();
            Clear_COM3_Transmit_Buffer();
            Clear_COM3_Receive_Buffer();
            Com3TransObject.Index = 0;
            GSM_Sch_Info.AT_command = GSM_CMGD;
            sprintf(&Com3TransObject.Msg_Buffer[Com3TransObject.Index],"%s%s",&GSM_AT_Command[GSM_Sch_Info.AT_command][0],&GSM_AT_Enter[0]);
            Com3TransObject.Msg_Length = strlen(&Com3TransObject.Msg_Buffer[0]);
            GSM_Sch_Info.State = TRANSMIT_RECORDS_FOR_GSM;
            Com3RecvObject.Index = 0;
            Delete_message = 1;
            break;
        default:
            break;
	}
}

/*********************************************************************************
* File name 		:comm_GSM.c
* Function Name		:void Check_change_number(void)
* Created By		:EM003
* Date Created		:
* Modification History:
* 					Rev No			Date		Description
* 					 --				 --				--
* Tracability:
* 		SRS()    	:
* 
* Abstract			:Change phone number saved and add it the EEPROM
* Algorithm			:
* Description		:
* Input Element		:None
* Output Element	:void
* 
**********************************************************************************/

void Check_change_number(void)
{
    char refernce_array[14] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    unsigned int Message_start;
    unsigned int temp, track =0;
    BYTE ph_index = 0;
    char temp_array[10] = {0,0,0,0,0,0,0,0,0,0};
//    for(temp=0;temp<strlen(temp_array);temp++)
//        temp_array[temp]=0;
    memcpy(temp_array,&Com3RecvObject.Msg_Buffer[3],4);
    if(strcmp(temp_array,"CMGL")==0)
    {
        Message_start = strlen(&Com3RecvObject.Msg_Buffer[0]) - strlen(&String_Change_Number[0]) -8 -2;
        track = strlen(&String_Change_Number[0]);
        for(temp=0;temp<track;temp++)
            refernce_array[temp] = Com3RecvObject.Msg_Buffer[temp+Message_start];
        ph_index = Com3RecvObject.Msg_Buffer[temp+Message_start + 1] - '0';
        if((strcmp(&refernce_array[0],&String_Change_Number[0]))==0)
        {
            for (temp = 0, track = 0; temp<strlen(&Com3RecvObject.Msg_Buffer[0]); temp++)
            {
                if(Com3RecvObject.Msg_Buffer[temp]=='"')
                {
                    track++;
                }
                if(track==3)
                {
                    break;
                }
            }
            if(ph_index >= 1 && ph_index <= 5)
            {
                memcpy(&Phone_number[ph_index - 1][0],&Com3RecvObject.Msg_Buffer[temp+1],13);
                GSM_Sch_Info.Change_number_Success = 1;
                for(temp = 0; temp<0xFFF0; temp++);
                P_status = Save_Phone_Number(ph_index);
                GSM_Sch_Info.Ph_number_index = ph_index - 1;
            }
        }
    }
}
/*********************************************************************************
* File name 		:comm_GSM.c
* Function Name		:void Check_Error(void)
* Created By		:EM003
* Date Created		:
* Modification History:
* 					Rev No			Date		Description
* 					 --				 --				--
* Tracability:
* 		SRS()    	:
* 
* Abstract			:Check for error particular to configuration and trigger to 
* send message
* Algorithm			:
* Description		:
* Input Element		:None
* Output Element	:void
* 
**********************************************************************************/
void Check_Error(void)
{
    switch(Disp_Info.Unit_Type)
    {
        case DAC_UNIT_TYPE_SF:
            break;
        case DAC_UNIT_TYPE_EF:
            break;
        case DAC_UNIT_TYPE_LCWS:
        case DAC_UNIT_TYPE_LCWS_DL:
                switch(Disp_Info.DS_mode)
                {
                    case WAITING_FOR_RESET_AT_BOTH_UNITS:
                        if(EF_Error!=0)
                        {
                            sprintf(&GSM_Sch_Info.Error_Message[0],"%d %s",EF_Error-1,&Error_message[EF_Error-1][0]);
                            GSM_Sch_Info.Error_Occured = 1;
                            GSM_Sch_Info.Error_Number = EF_Error - 1;
                            return;
                        }
                        break;
                    case SECTION_ERROR_AT_BOTH_UNITS:
                        if(var_count != 255)
                        {
                            sprintf(&GSM_Sch_Info.Error_Message[0],"%d %s\n",var_count,&Error_message[var_count][0]);
                            GSM_Sch_Info.Error_Occured = 1;
                            GSM_Sch_Info.Error_Number = var_count;
                            return;
                        }
                        else if(EF_Error!=0)
                        {
                            sprintf(&GSM_Sch_Info.Error_Message[0],"%d %s\n",EF_Error - 1,&Error_message[EF_Error - 1][0]);
                            GSM_Sch_Info.Error_Occured = 1;
                            GSM_Sch_Info.Error_Number = EF_Error - 1;
                            return;
                        }
                        break;
                    default:
                            GSM_Sch_Info.Error_Occured = 0;
                            GSM_Sch_Info.Error_Reported = 0;
                            GSM_Sch_Info.Error_Number = 0;
                        break;
                }
            break;
        default:
            break;
    }
    if(feedback_error)
    {
        sprintf(&GSM_Sch_Info.Error_Message[0],"%d %s\n",FB_error_ID - 1,&Feedback_message[FB_error_ID - 1][0]);
        GSM_Sch_Info.Error_Occured = 1;
        GSM_Sch_Info.Error_Number = FB_error_ID - 1;
        feedback_error = 0;
    }
}
/*********************************************************************************
* File name 		:comm_GSM.c
* Function Name		:void Decrement_GSM_Sch_msTmr(void)
* Created By		:EM003
* Date Created		:
* Modification History:
* 					Rev No			Date		Description
* 					 --				 --				--
* Tracability:
* 		SRS()    	:
* 
* Abstract			:One milliSecond Timer Variables present GSM_sch_info and
* 					 com3 receive object are decremented.
* Algorithm			:
* Description		:
* Input Element		:None
* Output Element	:void
* 
**********************************************************************************/
void Decrement_GSM_Sch_msTmr(void)
{
	if (GSM_Sch_Info.Timeout_ms > 0)
	{
		GSM_Sch_Info.Timeout_ms = GSM_Sch_Info.Timeout_ms - 1;
	}
	if (Com3RecvObject.Timeout_ms > 0)
	{
		Com3RecvObject.Timeout_ms = Com3RecvObject.Timeout_ms - 1;
	}
}
/*********************************************************************************
* File name 		:comm_GSM.c
* Function Name		:void Receive_COM3_Message(void)
* Created By		:EM003
* Date Created		:
* Modification History:
* 					Rev No			Date		Description
* 					 --				 --				--
* Tracability:
* 		SRS()    	:
* 
* Abstract			:Receive Command from RS232 Port (PC).
* Algorithm			:1.Intialise receive scheduler to buffer empty state.
* 				     2.Check for any data received.If data received, store into Receive Buffer.
* 					   If Data received is command id, then get the message length for the command from the table.
* 					 3.If No of Bytes Received is greater than or equal to Message lengh, then send Receive Buffer for crc check.
* 					 4.If crc function return value is zero, then received message is Vaild. So go for process GSM command.
* 					 5.If crc function return value is non zero, then received message is InVaild. So go for buffer empty state.
* Description		:
* Input Element		:None
* Output Element	:void
* 
**********************************************************************************/

void Receive_COM3_Message(void)
{
	BYTE uchData=0;
    if (U3STAbits.OERR)
    	Clear_Com3_Error();
    if(GSM_Sch_Info.GSM_Enable && IFS5bits.U3RXIF)
    {
        while(U3STAbits.URXDA)
        {
            uchData = (BYTE)U3RXREG;
            switch(Com3RecvObject.State)
            {
                case GSM_IGNORE_RECV_DATA:
                    break;
                case GSM_CONSIDER_RECV_DATA:
                    Com3RecvObject.Msg_Buffer[Com3RecvObject.Index] = uchData;
                    Com3RecvObject.Index++;
                    break;
                default:
                    break;
            }
        }
    }
}

/*********************************************************************************
* File name 		:comm_GSM.c
* Function Name		:void Clear_COM3_Receive_Buffer(void)
* Created By		:EM003
* Date Created		:
* Modification History:
* 					Rev No			Date		Description
* 					 --				 --				--
* Tracability:
* 		SRS()    	:
* 
* Abstract			:Function that clears receiving buffer.
* Algorithm			:
* Description		:
* Input Element		:None
* Output Element	:void
* 
**********************************************************************************/
void Clear_COM3_Receive_Buffer(void)
{
	unsigned int uchCnt =0;

	for (uchCnt = 0; uchCnt < GSM_MAX_RECV_MESSAGE_LENGTH; uchCnt++)
	{
		Com3RecvObject.Msg_Buffer[uchCnt] = (BYTE) 0;
	}
}
/*********************************************************************************
* File name 		:comm_GSM.c
* Function Name		:void Clear_COM3_Transmit_Buffer(void)
* Created By		:EM003
* Date Created		:
* Modification History:
* 					Rev No			Date		Description
* 					 --				 --				--
* Tracability:
* 		SRS()    	:
* 
* Abstract			:Function that clears transmit buffer.
* Algorithm			:
* Description		:
* Input Element		:None
* Output Element	:void
* 
**********************************************************************************/
void Clear_COM3_Transmit_Buffer(void)
{
	unsigned int uchCnt =0;

	for (uchCnt = 0; uchCnt < GSM_MAX_RECV_MESSAGE_LENGTH; uchCnt++)
	{
		Com3TransObject.Msg_Buffer[uchCnt] = (BYTE) 0;
	}
}
/*********************************************************************************
* File name 		:comm_GSM.c
* Function Name		:void Clear_Com3_Error(void)
* Created By		:EM003
* Date Created		:
* Modification History:
* 					Rev No			Date		Description
* 					 --				 --				--
* Tracability:
* 		SRS()    	:
* 
* Abstract			:This function Clears the communication errors,if occured.
* Algorithm			:
* Description		:
* Input Element		:None
* Output Element	:void
* 
**********************************************************************************/
void Clear_Com3_Error(void)
{
    volatile BYTE uchCnt=0;

	if (U3STAbits.OERR)
	{
		/* Overrun Error! Clear the error */
		U3MODEbits.UARTEN = SET_LOW;
        U3MODEbits.UARTEN = SET_HIGH;
	}
	if (U3STAbits.FERR)
	{
		/* Framing Error! Clear the error */
		uchCnt= (BYTE)U3RXREG;
        U3MODEbits.UARTEN = SET_LOW;
        U3MODEbits.UARTEN = SET_HIGH;
	}
    U3STAbits.UTXEN = 1;
    uchCnt++;
}

