/*********************************************************************************************************************
*	Project		: 	Single Section Digital Axle Counter
*	Version		: 	2.0 
*	Revision	:	1	
*	Filename	: 	comm_smc.c
*	Target MCU	: 	PIC24FJ256GB210   
*    Compiler	: 	XC16 Compiler V1.31  
*	Author		:	EM003 
*	Date		:	
*	Company Name: 	Insys Digital Systems
*	Modification History:
*					Rev No			Date		Description
*					 --				 --				--    
*	Functions	:   void Initialise_Smc_CommSch(void)
*					void SetupCOM1BaudRate(BYTE uchBaudRate)
*					void Start_Smc_Communication(void)
*					void Update_Smc_Sch_State_QR(void)
*					void Decrement_Smc_Sch_msTmr(void)
*					void Clear_COM1_Receive_Buffer(void)
*					void Clear_Com1_Error(void)
*			
*********************************************************************************************************************/
#include <xc.h>

#include "COMMON.h"
#include "CRC16.h"
#include "COMM_GSM.h"
#include "COMM_SMC.h"


extern  sm_status_t Status;					/* From cpu_sm.c */
extern const BYTE uchCommand_Length_Table[7][2];	/* From command_proc.c */
extern const BYTE BitMask_List[8];					/* From cpu_sm.c */
extern BYTE CPU1_data_GLCD[GCPU_SPI_MESSAGE_LENGTH];			/* Buffer to store data recived from Cpu1 */
extern BYTE CPU2_data_GLCD[GCPU_SPI_MESSAGE_LENGTH];			/* Buffer to store data recived from Cpu2 */
extern dac_sysinfo_t DAC_sysinfo;					/* from cpu_sm.c */
extern BYTE CPU1_data[SPI_MESSAGE_LENGTH];			/* Buffer to store data recived from Cpu1 */			
extern BYTE CPU2_data[SPI_MESSAGE_LENGTH];			/* Buffer to store data recived from Cpu2 */
extern BYTE CPU1_Address;					/* holds cpu1 address */
extern BYTE CPU2_Address;					/* holds cpu2 address */


smc_sch_info_t             Smc_Sch_Info;			/* structure holds station master interface communication scheduler */
smc_info_t					Smc1XmitObject;
BYTE Latest_DAC_data[SPI_MESSAGE_LENGTH];	/* From comm_dac.c */

/*********************************************************************************
*File name 			:comm_smc.c
*Function Name		:void Initialise_Modem(void)
*Created By			:EM003 
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Initialize modem by controlling M0 and M1 pins
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Initialise_Modem(void)
{
    LATB = 0xC000 | (LATB & 0x3FFF);
}
/*********************************************************************************
*File name 			:comm_smc.c
*Function Name		:void Set_Modem_TX_Mode(void)
*Created By			:EM003 
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Set modem to transmission by controlling M0 and M1 pins
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Set_Modem_TX_Mode(void)
{
    LATB = 0x4000 | (LATB & 0x3FFF);
}
/*********************************************************************************
*File name 			:comm_smc.c
*Function Name		:void Set_Modem_RX_Mode(void)
*Created By			:EM003 
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Set modem to Reception by controlling M0 and M1 pins
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Set_Modem_RX_Mode(void)
{
    LATB = 0x8000 | (LATB & 0x3FFF);
}


/*********************************************************************************
*File name 			:comm_smc.c
*Function Name		:void Initialise_Smc_CommSch(void)
*Created By			:EM003 
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Initialise smc communication scheduler .
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:void
*
***********************************************************************************/
void Initialise_Smc_CommSch(void)
{
	Smc_Sch_Info.State      = SMC_SCHEDULER_NOT_STARTED;		 /*  set smc comm scheduler to "SMC_SCHEDULER_NOT_STARTED" state  */ 
	SetupCOM1BaudRate((BYTE)BAUDRATE_1200);						     /*  set smc comm baudrate to 1200bps */ 
	Set_Modem_RX_Mode();
	Smc_Sch_Info.Timeout_ms = NEXT_QUERY_WAIT;
	Status.Flags.MDM_TX_Mode = SET_LOW;
}
/*********************************************************************************
File name 			:comm_smc.c
Function Name		:void SetupCOM1BaudRate(BYTE uchBaudRate)
Created By			:EM003 
Date Created		:
Modification History:
					Rev No			Date		Description
					 --				 --				--
Tracability:
		SRS()    	:

Abstract			:sets Baudrate for smc communication 
Algorithm			:
Description			: 
Input Element		:uchBaudRate
Output Element		:void

**********************************************************************************/
void SetupCOM1BaudRate(BYTE uchBaudRate)
{
    TRISFbits.TRISF5 = SET_LOW;       // RF5 is Tx, Output
    TRISFbits.TRISF4 = SET_HIGH;       // RF4 is Rx, Input
    
    //CD as input
    TRISBbits.TRISB13 = SET_HIGH;
    ANSBbits.ANSB13 = SET_LOW;

    TRISBbits.TRISB14 = SET_LOW;      //M0
    TRISBbits.TRISB15 = SET_LOW;      //M1

    /* Configure Remappable pins */
    __builtin_write_OSCCONL(OSCCON & 0xbf); //clear the bit 6 of OSCCONL to unlock Pin Re-map
    RPINR18bits.U1RXR = 10;
    RPOR8bits.RP17R = 3;
    __builtin_write_OSCCONL(OSCCON | 0x40); //set the bit 6 of OSCCONL to lock Pin Re-map

    U1MODE = 0;
    U1STA = 0;

    U1MODEbits.RTSMD = SET_HIGH;   // Operate in Simplex Mode
    U1MODEbits.BRGH  = SET_HIGH;    // Low Speed Clocks to BRG @ 16x of Fp

    /*
     * FCY = FOSC/2 = 32MHz/2 = 16Mz
     * U1BRG =FCY/(4 * Baud Rate) - 1
     * U1BRG = 16000000/(4*1200) - 1
     * U1BRG = 3332
     */
	switch (uchBaudRate)
	{
		case BAUDRATE_1200:
			U1BRG = SPBRG_FOR_1200;
			break;
		case BAUDRATE_2400:
			U1BRG = SPBRG_FOR_2400;
                        break;
		case BAUDRATE_9600:
			U1BRG = SPBRG_FOR_9600;
			break;
		case BAUDRATE_19200:
			U1BRG = SPBRG_FOR_19200;
			break;
        default:
            break;
	}

    IEC4bits.U1ERIE = SET_LOW;
    IEC0bits.U1RXIE = SET_LOW;
    IEC0bits.U1TXIE = SET_LOW;

    IFS4bits.U1ERIF = SET_LOW;
    IFS0bits.U1RXIF = SET_LOW;
    IFS0bits.U1TXIF = SET_LOW;
    U1MODEbits.UARTEN = SET_HIGH;  // Enable UART
    U1STAbits.UTXEN = SET_HIGH;
    U1STAbits.UTXISEL0 = SET_HIGH;
    U1STAbits.UTXISEL1 = SET_LOW;
    
}
/*********************************************************************************
*File name 			:comm_smc.c
*Function Name		:void Start_Smc_Communication(void)
*Created By			:EM003 
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:This function starts smc comm scheduler.
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:void
*
***********************************************************************************/
void Start_Smc_Communication(void)
{
    Smc_Sch_Info.State =  SMC_COMM_SCHEDULER_IDLE;
}
/*********************************************************************************
*File name 			:comm_smc.c
*Function Name		:void Update_Smc_Sch_State_QR(void)
*Created By			:EM003 
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Smc communication scheduler to Interface with Station Master through FSK Modem.
*Algorithm			:				             
*Stage			State									Action
*1			SMC_SCHEDULER_NOT_STARTED					NA
*2			SMC_COMM_SCHEDULER_IDLE						Wait to get into RX mode
*3			RX_STABLE_WAIT								Wait for RX Stablize after switching
*4			WAIT_FOR_SMC_CD_HIGH						Wait fir CD to get High from reset box
*5			(CD==1)WAIT_FOR_SMC_CD_HIGH_STABLE			Wait for CD to stabilize from Low to High
*6			WAIT_FOR_RX_QUERY							Wait to recieve 5 bytes query from reset(0x3C,N_CONF,CPU1/2,CRCL,CRCH)
*7			WAIT_FOR_CD_LOW								Wait for CD to get low if query is matched	
*8			WAIT_FOR_TX_TRANSITION						After CD is low, wait before switching to TX mode
*9			WAIT_TX_STABLE								After switching to TX wait for it to stabilize
*10			TRANSMIT_SMC_DATA							Transmit 80 bytes of data matching CPU1/2
*11			WAIT_AFTER_TX								Wait to get out TX mode. After Timeout, enter RX mode and switch to SCHEDULER_IDLE
*12			ERROR_SMC_MODEM								Enter here if query does not match  
*Description			: 
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Update_Smc_Sch_State_QR(void)
{   
    if(inspect_event_done == 0)
        return;
    BYTE uchByte;
    UINT16 temp_crc;
	switch (Smc_Sch_Info.State)
	{
		case SMC_SCHEDULER_NOT_STARTED:
			break;
		case SMC_COMM_SCHEDULER_IDLE:
			if (Smc_Sch_Info.Timeout_ms == TIMEOUT_EVENT)
			{
				Set_Modem_RX_Mode();
                Smc_Sch_Info.State = RX_STABLE_WAIT;
                Smc_Sch_Info.Timeout_ms = RX_STABLE_WAIT_TIME;//10ms
			}
			break;
        case RX_STABLE_WAIT:
            if (Smc_Sch_Info.Timeout_ms == TIMEOUT_EVENT)
            {
				Smc_Sch_Info.State = WAIT_FOR_SMC_CD_HIGH;
            }
            break;
		case WAIT_FOR_SMC_CD_HIGH:
			if (MODEM_CD == 1)
			{
				//CD has been detected, wait for it to get stable
				Smc_Sch_Info.State = WAIT_FOR_SMC_CD_HIGH_STABLE;
                Smc_Sch_Info.Timeout_ms = CD_LOW_TO_HIGH_STABLE_WAIT_TIME;//10ms
			}
			else
			{
				//Other SMCPU is sending data
				//Wait till next query
				// CD is not getting low
                while(U1STAbits.URXDA)
					uchByte = (BYTE)U1RXREG; /*Clear old Junk Data*/
				SetupCOM1BaudRate((BYTE)BAUDRATE_1200);/*  set smc comm baudrate to 1200bps */ 
				Set_Modem_RX_Mode();
				Clear_Com1_Error();
			}
			break;
		case WAIT_FOR_SMC_CD_HIGH_STABLE:
			if (Smc_Sch_Info.Timeout_ms == TIMEOUT_EVENT)
			{
				Smc_Sch_Info.State = WAIT_FOR_RX_QUERY;
                //1 byte takes 6.66ms, 5 bytes will take ~33.3ms, 33+10(CD TX wait) +(10) => ~70ms
				Smc_Sch_Info.Timeout_ms = RX_QUERY_WAIT_TIME; //100 ms
				Smc_Sch_Info.Rx_query_len = 0;
			}
			break;
		case WAIT_FOR_RX_QUERY:
            Clear_Com1_Error();
			if(Smc_Sch_Info.Rx_query_len>4)
			{
				if(Smc_Sch_Info.Recvd_query[0] == 0x3C)
				{
					//check CRC
                    temp_crc = Crc16(SMC_QUERY,RCV_QUERY_LEN-2);
                    if (temp_crc == ((UINT16)((UINT16)Smc_Sch_Info.Recvd_query[RCV_QUERY_LEN-1]<<8)+(Smc_Sch_Info.Recvd_query[RCV_QUERY_LEN-2])))
					{
						// Valid CRC. check for match of CPU & Ntw addess;
						if((Smc_Sch_Info.Recvd_query[1]==Event_Logger_ID) && ((Smc_Sch_Info.Recvd_query[2]==0x01) || (Smc_Sch_Info.Recvd_query[2]==0x02)))
						{
                            Smc_Sch_Info.Req_CPU_Addr = Smc_Sch_Info.Recvd_query[2];
							Smc_Sch_Info.State = WAIT_FOR_CD_LOW;//Valid Query
							Smc_Sch_Info.Timeout_ms = WAIT_TIME_FOR_CD_LOW_TIME;//Wait for CD to get low 30ms buffer
						}
						else{
                            //CRC is correct but this query is not for me
                            Smc_Sch_Info.State = SMC_COMM_SCHEDULER_IDLE;//Query for different CPU
                            Smc_Sch_Info.Timeout_ms  = OTHER_COM_WAIT_TIME;//Wait for CPU to send the data. (50+533+50))
						}
					}
					else
					{
                        //Incorrect CRC
						Smc_Sch_Info.State = ERROR_SMC_MODEM;//Query for different CPU
						Smc_Sch_Info.Timeout_ms  = ERROR_WAIT_TIME;//Wait for 300ms to get to IDLE state (Ideally 80bytes requires 533ms)
					}
				}
				else{
					//Invalid Packet
					Smc_Sch_Info.State = ERROR_SMC_MODEM;//Query for different CPU
					Smc_Sch_Info.Timeout_ms  = ERROR_WAIT_TIME;//Wait for 300ms to get to IDLE state (Ideally 80bytes requires 533ms)						
				}
			}
			else{
				if(IFS0bits.U1RXIF)
				{
					while(U1STAbits.URXDA)
					{
						Smc_Sch_Info.Recvd_query[Smc_Sch_Info.Rx_query_len] = (BYTE)U1RXREG;
						Smc_Sch_Info.Rx_query_len++;
					}
                    IFS0bits.U1RXIF = 0;
				}
			}
			break;
		case WAIT_FOR_CD_LOW:
			if (Smc_Sch_Info.Timeout_ms == TIMEOUT_EVENT)
			{
				//CD did not get to low, go to end. This is an error state.
				Smc_Sch_Info.State = ERROR_SMC_MODEM;//Query for different CPU
                Smc_Sch_Info.Timeout_ms  = ERROR_WAIT_TIME;//Wait for 300ms to get to IDLE state (Ideally 80bytes requires 533ms)										
			}
			if(MODEM_CD == 0)
			{
				//CD got Low
				Smc_Sch_Info.State = WAIT_FOR_TX_TRANSITION;//Wait for TX mode transition
				Smc_Sch_Info.Timeout_ms =WAIT_FOR_TX_TRANSITION_TIME;//Wait for 20ms before getting into TX mode
			}
			break;
		case WAIT_FOR_TX_TRANSITION:
			if(Smc_Sch_Info.Timeout_ms == TIMEOUT_EVENT)
			{
				Set_Modem_TX_Mode();
				Smc_Sch_Info.State = WAIT_TX_STABLE;
				Smc_Sch_Info.Timeout_ms = WAIT_TX_STABLE_TIME;//10 ms
			}
			break;
		case WAIT_TX_STABLE:
			if(Smc_Sch_Info.Timeout_ms == TIMEOUT_EVENT)
			{
                //TX and CD are stable
 				Set_Modem_TX_Mode();
				Build_smc_broadcast_message();
				Clear_Com1_Error();
                SetupCOM1BaudRate((BYTE)BAUDRATE_1200);
				Smc_Sch_Info.State  = TRANSMIT_SMC_DATA;				
				Smc_Sch_Info.Timeout_ms = PACKET_TX_MAX_WAIT_TIME;//550
			}
			break;
		case TRANSMIT_SMC_DATA:
			if (Smc1XmitObject.Index >= (Smc1XmitObject.Msg_Length) && U1STAbits.TRMT==1)
			{
				Smc_Sch_Info.State      = WAIT_AFTER_TX;				
				Smc_Sch_Info.Timeout_ms = SMC_COM_TX_END_WAIT;//20ms
				break;
			}
			if (U1STAbits.TRMT==1 && U1STAbits.UTXBF == 0 && Smc1XmitObject.Index < (Smc1XmitObject.Msg_Length))
			{
				IFS0bits.U1TXIF = 0;	
				while(U1STAbits.UTXBF == 0 && Smc1XmitObject.Index < (Smc1XmitObject.Msg_Length))
				{
					U1TXREG = Smc1XmitObject.Msg_Buffer[Smc1XmitObject.Index];
					Smc1XmitObject.Index = Smc1XmitObject.Index + 1;
				}
				Receive_COM3_Message();
			}
			break;
        case WAIT_AFTER_TX:
            if(Smc_Sch_Info.Timeout_ms == TIMEOUT_EVENT)
            {
                Smc_Sch_Info.State      = SMC_COMM_SCHEDULER_IDLE;				
				Smc_Sch_Info.Timeout_ms = NEXT_QUERY_WAIT;//100ms
				SetupCOM1BaudRate((BYTE)BAUDRATE_1200);						     /*  set smc comm baudrate to 1200bps */ 
                Set_Modem_RX_Mode();	
            }
            break;
		case ERROR_SMC_MODEM:
			if (Smc_Sch_Info.Timeout_ms == TIMEOUT_EVENT)
			{
				//Total CD capture failure!, restart scheduler
                Smc_Sch_Info.Timeout_ms = NEXT_ERR_QUERY_WAIT;//40ms
				Smc_Sch_Info.State      = SMC_COMM_SCHEDULER_IDLE;				
				SetupCOM1BaudRate((BYTE)BAUDRATE_1200);						     /*  set smc comm baudrate to 1200bps */ 
                Set_Modem_RX_Mode();					
			}
			break;			
        default:
            break;
	}
}

/*********************************************************************************
*File name 			:comm_smc.c
*Function Name		:void Decrement_Smc_Sch_msTmr(void)
*Created By			:EM003 
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:one milliSecond Timer Variables present Smc_sch_info and
*					 com1 receive object are decremented.
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:void
*
***********************************************************************************/
void Decrement_Smc_Sch_msTmr(void)
{
	if (Smc_Sch_Info.Timeout_ms > 0)
	{
		Smc_Sch_Info.Timeout_ms = Smc_Sch_Info.Timeout_ms - 1;
	}	

}
/*********************************************************************************
*File name 			:comm_smc.c
*Function Name		:void Clear_Com1_Error(void)
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
void Clear_Com1_Error(void)
{
  BYTE uchCnt=0;

	if (U1STAbits.OERR)
	{
		/* Overrun Error! Clear the error */
        U1STAbits.OERR = 0;
		U1MODEbits.UARTEN = SET_LOW;
        U1MODEbits.UARTEN = SET_HIGH;
        IFS0bits.U1RXIF = 0;
	}
	if (U1STAbits.FERR)
	{
		/* Framing Error! Clear the error */
		uchCnt= (BYTE)U1RXREG;
        IFS0bits.U1RXIF = 0;
        U1MODEbits.UARTEN = SET_LOW;
        U1MODEbits.UARTEN = SET_HIGH;
	}
//    U1STAbits.UTXEN = SET_HIGH;
}
/*********************************************************************************
*File name 			:comm_smc.c
*Function Name		:void Build_smc_broadcast_message(void)
*Created By			:EM003 
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:This function builds the message to broadcast to remote display.
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Build_smc_broadcast_message(void)
{
	BYTE uchCnt=0;
	wordtype_t CheckSum;

	for (uchCnt = 0; uchCnt < SPI_MESSAGE_LENGTH; uchCnt++)
	{
        if(Smc_Sch_Info.Req_CPU_Addr == 0x01)
        {
            Smc1XmitObject.Msg_Buffer[uchCnt]  = CPU1_data_GLCD[uchCnt];
        }
        else
        {
            Smc1XmitObject.Msg_Buffer[uchCnt]  = CPU2_data_GLCD[uchCnt];
        }
	}
    Smc1XmitObject.Msg_Buffer[70] = DAC_sysinfo.Unit_Type;
	Smc1XmitObject.Msg_Buffer[71] = DAC_sysinfo.SW_Version;
	Smc1XmitObject.Msg_Buffer[72] = DAC_sysinfo.Checksum.DWord.HiWord.Byte.Hi;
	Smc1XmitObject.Msg_Buffer[73] = DAC_sysinfo.Checksum.DWord.HiWord.Byte.Lo;
	Smc1XmitObject.Msg_Buffer[74] = DAC_sysinfo.Checksum.DWord.LoWord.Byte.Hi;
	Smc1XmitObject.Msg_Buffer[75] = DAC_sysinfo.Checksum.DWord.LoWord.Byte.Lo;    
    Smc1XmitObject.Msg_Buffer[76] = Event_Logger_ID;

	CheckSum.Word = Crc16(SMC_OBJ, SMC_XMIT_MESSAGE_LENGTH - 2);
	Smc1XmitObject.Msg_Buffer[78] = CheckSum.Byte.Lo;				/* CRC low Byte */
	Smc1XmitObject.Msg_Buffer[79] = CheckSum.Byte.Hi;				/* CRC High Byte */
	Smc1XmitObject.Msg_Length = SMC_XMIT_MESSAGE_LENGTH;
	Smc1XmitObject.Index = 0;
}

