/*********************************************************************************************************************
*	Project		: 	Single Section Digital Axle Counter
*	Version		: 	2.0
*	Revision	:	1
*	Filename	: 	DRV_GLCD_SPI.c
*	Target MCU	: 	PIC24FJ256GB210
*    Compiler	: 	XC16 Compiler V1.31
*	Author		:	EM003 
*	Date		:
*	Company Name: 	Insys Digital Systems.
*	Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*	Functions	:   void Initialise_GLCD_Display(void);
*                   void Build_packet_GLCD(void);
*                   void Update_SMPU_data(void);
*                   void Update_GLCD_State(void);
*                   void Decrement_GLCD_msTmr(void);
*
*********************************************************************************************************************/
#include <xc.h>
#include <stddef.h>
#include <time.h>

#include "COMMON.h"
#include "DRV_GLCD_SPI.h"
#include "command_proc.h"
#include "DRV_MEM.h"
#include "COMM_GSM.h"
#include "CRC16.h"
#include "EVENTS.h"


extern glcd_info_t GLCD_Info;
extern dac_sysinfo_t DAC_sysinfo;
extern BYTE CPU1_data_GLCD[GCPU_SPI_MESSAGE_LENGTH];			/* Buffer to store data recived from Cpu1 */
extern BYTE CPU2_data_GLCD[GCPU_SPI_MESSAGE_LENGTH];			/* Buffer to store data recived from Cpu2 */
extern ring_buffer_t     Events_Ring_Buffer;
extern time_t SystemClock;					/* holds system clock time   */
extern UINT32 SMCPU_CRC_checksum;
extern BOOL E_status;/*lint -e552 */
glcd_info_t GLCD_Info;		/* structure that handles Lcd scheduler and holds lcd Information */
BYTE count;



void Build_packet_GLCD(void);



/*********************************************************************************
*File name 			:DRV_GLCD_SPI.c
*Function Name		:void Initialise_LCD_Driver(void)
*Created By			:EM003 
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Initialisation of lcd driver module.
*Algorithm			:
*Description			:
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Initialise_GLCD_Driver(void)
{
    UINT16 temp;
    if(SPI1STATbits.SPIRBF==1)
        while(!SPI1STATbits.SRXMPT)
            temp = SPI1BUF;
    
    SPI1CON1 = 0;
    SPI1CON2 = 0;
    SPI1STAT = 0;
    
    /* Configure Remappable pins */
    __builtin_write_OSCCONL(OSCCON & 0xbf); //clear the bit 6 of OSCCONL to unlock Pin Re-map
    RPINR20bits.SDI1R = 22;
    RPOR10bits.RP20R = 8;
    RPOR12bits.RP25R = 7;
    __builtin_write_OSCCONL(OSCCON | 0x40); //set the bit 6 of OSCCONL to lock Pin Re-map

    TRISDbits.TRISD4 = SET_LOW;            // set  SDI1 as an input
    TRISDbits.TRISD3 = SET_HIGH;            // set  SDO1 as an output
    TRISDbits.TRISD5 = SET_LOW;             // set  SCK1 as an output
    TRISDbits.TRISD13 = SET_LOW;             //  set  SS as an input
    
    SPI1CON2bits.SPIBEN = SET_HIGH;
    IFS0bits.SPI1IF = SET_LOW;

    IEC0bits.SPI1IE = SET_LOW;
    SPI1CON1 = SPI_MASTER;  // select mode
    SPI1STAT = SPI_ENABLE;  // enable the peripheral

    G_SPI_SS = SET_HIGH;
    temp = SPI1BUF;
}

/*********************************************************************************
File name 			:DRV_GLCD_SPI.c
Function Name		:void Update_LCD_State(void)
Created By			:EM003 
Date Created		:
Modification History:
					Rev No			Date		Description
					 --				 --				--
Tracability:
		SRS()    	:

Abstract			:Module for updating the values in the lcd display
Algorithm			:1.check for lcd module plugged in or not.
					 2.Go for initialisation if module is plugged in  else go to step1.
					 3.After Initialisation of LCD completed, Send packet from SPI
Description			:
Input Element		:None
Output Element		:void

**********************************************************************************/
void Update_GLCD_State(void)
{

    if(inspect_event_done == 0)
        return;
    if(0)
    {
        GLCD_Info.State = GLCD_MODULE_NOT_ACTIVE;
        return;
    }
	switch (GLCD_Info.State)
	{
		case GLCD_MODULE_NOT_ACTIVE:
            if (1)
			{
                count = 0;
                GLCD_Info.Comm_Timeout_ms = SS_TIMEOUT; //wait for 5 ms for SS to stabilize
                GLCD_Info.State = GLCD_SS_WAIT;
			}
			break;
        case GLCD_IDLE:
            if(GLCD_Info.Comm_Timeout_ms == 0)
            {
                G_SPI_SS = SET_LOW;
                count = 0;
                GLCD_Info.State = GLCD_COMPARE_DATA;
            }
            break;
        case GLCD_COMPARE_DATA:
            G_SPI_SS = SET_LOW;
            GLCD_Info.Comm_Timeout_ms = SS_TIMEOUT; //wait for 10 ms for SS to stabilize
            GLCD_Info.State = GLCD_SS_WAIT;
            break;
        case GLCD_SS_WAIT:
            if(GLCD_Info.Comm_Timeout_ms == 0)
            {
                GLCD_Info.State = GLCD_INIT_DATA;
            }
            break;
		case GLCD_INIT_DATA:
            Build_packet_GLCD();
            GLCD_Info.Packet_Max_length = MAX_G_PACKET_LEN;
            GLCD_Info.State = GLCD_TX_DATA;
            GLCD_Info.Packet_index = 0;
			break;
		case GLCD_TX_DATA:            
            //send 8 bytes through SPI1
            //SPI1BUF = data;
            if(GLCD_Info.Comm_Timeout_ms != 0)
                break;
            while(SPI1STATbits.SPITBF == 0)//if (BF == 0)					   /* Still all Bytes are not completely transmitted */
            {
                /* SSPBUF empty - Transmit Status, So we can Put the Next Byte */
                SPI1BUF = GLCD_Info.Message_Buffer[GLCD_Info.Packet_index];//GLCD_Info.Packet_index; for testing
                GLCD_Info.Packet_index++;
            }
            //GLCD_Info.Comm_Timeout_ms = TX_TIMEOUT;
            GLCD_Info.State = GLCD_CM_WAIT;
            break;
        case GLCD_CM_WAIT:
            if(SPI1STATbits.SPIRBF==1)
            {
                while(SPI1STATbits.SRXMPT==0)
                {
                    GLCD_Info.Rx_Message_Buffer[count] = (BYTE)SPI1BUF;
                    count++;
                }
                if((count%8)==0)
                {
                    if(GLCD_Info.Packet_index >= GLCD_Info.Packet_Max_length)
                    {
                        GLCD_Info.Comm_Timeout_ms = MAX_COMM_TIMEOUT;
                        GLCD_Info.State = GLCD_IDLE;
                        G_SPI_SS = SET_HIGH;
                    }
                    else
                    {
                        GLCD_Info.Comm_Timeout_ms = TX_TIMEOUT;
                        GLCD_Info.State = GLCD_TX_DATA;
                    }

                }
            }
            break;
        default:
			break;
	}
}

/*********************************************************************************
File name 			:DRV_GLCD_SPI.c
Function Name		:void Decrement_LCD_msTmr(void)
Created By			:EM003 
Date Created		:
Modification History:
					Rev No			Date		Description
					 --				 --				--
Tracability:
		SRS()    	:

Abstract			:For every 1ms,lcd info timeout variable is decremented.
Algorithm			:
Description			:
Input Element		:None
Output Element		:void

**********************************************************************************/
void Decrement_GLCD_msTmr(void)
{
	if (GLCD_Info.Comm_Timeout_ms > 0)
	{
		GLCD_Info.Comm_Timeout_ms = GLCD_Info.Comm_Timeout_ms - 1;
	}
}

/*********************************************************************************
*File name 			:DRV_GLCD_SPI.c
*Function Name		:void Build_packet_GLCD(void)
*Created By			:EM003 
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Construct packet to populate 144 bytes for GLCD
*Algorithm			:
*Description		:
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Build_packet_GLCD(void)
{
    BYTE track,u_count;
    wordtype_t CheckSum_G;
    //clear the buffer
    for(u_count = 0;u_count<(MAX_G_PACKET_LEN);u_count++)
    {
        GLCD_Info.Message_Buffer[u_count] = 0;
    }

    for(u_count = 0;u_count<8;u_count++) 
    {
        GLCD_Info.Message_Buffer[u_count]    = CPU1_data_GLCD[u_count];
        GLCD_Info.Message_Buffer[u_count+CPU1_PACKET_LEN] = CPU2_data_GLCD[u_count];
    }
    track = u_count;//8 - 21
    for(;u_count<21;u_count++) 
    {
        GLCD_Info.Message_Buffer[u_count]    = CPU1_data_GLCD[u_count + 24 - track];
        GLCD_Info.Message_Buffer[u_count + CPU1_PACKET_LEN] = CPU2_data_GLCD[u_count + 24 - track];
    }
        GLCD_Info.Message_Buffer[u_count]    = CPU1_data_GLCD[77];                  //Message ID
        GLCD_Info.Message_Buffer[u_count + CPU1_PACKET_LEN] = CPU2_data_GLCD[77];   //Message ID
        u_count++;
    track = u_count;
    for(;u_count<55;u_count++)     //u_count will start from 22 to 54 //Disp Count
    {
        GLCD_Info.Message_Buffer[u_count]    = CPU1_data_GLCD[u_count + 37 - track];
        GLCD_Info.Message_Buffer[u_count + CPU1_PACKET_LEN] = CPU2_data_GLCD[u_count + 37 - track];
    }
    GLCD_Info.Message_Buffer[OFFSET_CODE_CHECKSUM] = DAC_sysinfo.Checksum.DWord.HiWord.Byte.Hi;
    GLCD_Info.Message_Buffer[OFFSET_CODE_CHECKSUM + 1] = DAC_sysinfo.Checksum.DWord.HiWord.Byte.Lo;
    GLCD_Info.Message_Buffer[OFFSET_CODE_CHECKSUM + 2] = DAC_sysinfo.Checksum.DWord.LoWord.Byte.Hi;
    GLCD_Info.Message_Buffer[OFFSET_CODE_CHECKSUM + 3] = DAC_sysinfo.Checksum.DWord.LoWord.Byte.Lo;
    GLCD_Info.Message_Buffer[OFFSET_UNIT_TYPE] = DAC_sysinfo.Unit_Type;
    GLCD_Info.Message_Buffer[OFFSET_SW_V] = DAC_sysinfo.SW_Version;
    Update_SMPU_data();
    GLCD_Info.Message_Buffer[OFFSET_UNUSED + 1] = CPU1_data_GLCD[71];
    GLCD_Info.Message_Buffer[OFFSET_UNUSED + 2] = CPU1_data_GLCD[72];
    GLCD_Info.Message_Buffer[OFFSET_UNUSED + 3] = CPU1_data_GLCD[73];
    GLCD_Info.Message_Buffer[OFFSET_UNUSED + 4] = 0xAA;
    GLCD_Info.Message_Buffer[OFFSET_UNUSED + 5] = 0xAA;
    CheckSum_G.Word = Crc16(GLCD_INFO, MAX_G_PACKET_LEN - 2);
    GLCD_Info.Message_Buffer[OFFSET_CRC]   = CheckSum_G.Byte.Lo;
    GLCD_Info.Message_Buffer[OFFSET_CRC + 1]   = CheckSum_G.Byte.Hi;
}
/*********************************************************************************
*File name 			:DRV_GLCD_SPI.c
*Function Name		:void Update_SMPU_data(void)
*Created By			:EM003 
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Construct packet to populate 144 bytes for GLCD
*Algorithm			:
*Description		:
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Update_SMPU_data(void)
{
    BYTE uchBuf;
    event_record_t count_record;
    long_t S_Time, E_Time, Event_count, P_Time, SMCPU_CRC;
    for (uchBuf = 0; uchBuf <	EVENT_RECORD_SIZE; uchBuf++)
    {
        Event_Record_R.Byte[uchBuf] = 0;
    }
    E_status = Read_Event_from_Serial_EEProm(Events_Ring_Buffer.Head);
    for (uchBuf = 0; uchBuf <	EVENT_RECORD_SIZE; uchBuf++)
    {
        count_record.Byte[uchBuf] = Event_Record_R.Byte[uchBuf];
    }
    if(count_record.Field.Token == LATEST_EVENT_TOKEN)
    {
        // This is the latest recorded event
        E_Time.LWord = count_record.Field.Date_Time;
    }
    else
    {   // 0 if there is any error
        E_Time.LWord = 0x0000;
    }
    for (uchBuf = 0; uchBuf <	EVENT_RECORD_SIZE; uchBuf++)
    {
        Event_Record_R.Byte[uchBuf] = 0;
    }
    E_status = Read_Event_from_Serial_EEProm(Events_Ring_Buffer.Head + 1);
    for (uchBuf = 0; uchBuf <	EVENT_RECORD_SIZE; uchBuf++)
    {
        count_record.Byte[uchBuf] = Event_Record_R.Byte[uchBuf];
    }
    if(count_record.Field.Token != LATEST_EVENT_TOKEN || count_record.Field.Token != OLD_EVENT_TOKEN)
    {
        // record 0 is the 1st event.
        for (uchBuf = 0; uchBuf <	EVENT_RECORD_SIZE; uchBuf++)
        {
            Event_Record_R.Byte[uchBuf] = 0;
        }
        E_status = Read_Event_from_Serial_EEProm(0);
        for (uchBuf = 0; uchBuf <	EVENT_RECORD_SIZE; uchBuf++)
        {
            count_record.Byte[uchBuf] = Event_Record_R.Byte[uchBuf];
        }
        S_Time.LWord = count_record.Field.Date_Time;
        Event_count.LWord = Events_Ring_Buffer.Tail;
    }
    else if(count_record.Field.Token == OLD_EVENT_TOKEN)
    {
        // Tail record or next of head record is the 1st event. Memory has been rouded to use from the starting.
        S_Time.LWord = count_record.Field.Date_Time;
        Event_count.LWord = MAXIMUM_EVENTS;
    }
    else
    {
        // 0 if there is any error
        S_Time.LWord = 0x0000;
        Event_count.LWord = 0;
    }
    GLCD_Info.Message_Buffer[OFFSET_EVENT_COUNT]     = Event_count.Byte.Byte1; 			   /* Send nuber of events */
    GLCD_Info.Message_Buffer[OFFSET_EVENT_COUNT + 1] = Event_count.Byte.Byte2;
    GLCD_Info.Message_Buffer[OFFSET_EVENT_COUNT + 2] = Event_count.Byte.Byte3;
    GLCD_Info.Message_Buffer[OFFSET_EVENT_COUNT + 3] = Event_count.Byte.Byte4;
    GLCD_Info.Message_Buffer[OFFSET_START_TIME   ]   = S_Time.Byte.Byte1; 			   /* First event time */
	GLCD_Info.Message_Buffer[OFFSET_START_TIME + 1]  = S_Time.Byte.Byte2;
	GLCD_Info.Message_Buffer[OFFSET_START_TIME + 2]  = S_Time.Byte.Byte3;
	GLCD_Info.Message_Buffer[OFFSET_START_TIME + 3]  = S_Time.Byte.Byte4;
    GLCD_Info.Message_Buffer[OFFSET_END_TIME      ]  = E_Time.Byte.Byte1; 			   /* End event time */
	GLCD_Info.Message_Buffer[OFFSET_END_TIME + 1  ]  = E_Time.Byte.Byte2;
	GLCD_Info.Message_Buffer[OFFSET_END_TIME + 2  ]  = E_Time.Byte.Byte3;
	GLCD_Info.Message_Buffer[OFFSET_END_TIME + 3  ]  = E_Time.Byte.Byte4;

    P_Time.LWord = SystemClock;
    GLCD_Info.Message_Buffer[OFFSET_PRESENT_TIME     ]  = P_Time.Byte.Byte1;
    GLCD_Info.Message_Buffer[OFFSET_PRESENT_TIME + 1 ]  = P_Time.Byte.Byte2;
    GLCD_Info.Message_Buffer[OFFSET_PRESENT_TIME + 2 ]  = P_Time.Byte.Byte3;
    GLCD_Info.Message_Buffer[OFFSET_PRESENT_TIME + 3 ]  = P_Time.Byte.Byte4;

    SMCPU_CRC.LWord = (long)SMCPU_CRC_checksum;//substitute with code crc
    GLCD_Info.Message_Buffer[OFFSET_SMCPU_CRC     ]  = SMCPU_CRC.Byte.Byte1;
    GLCD_Info.Message_Buffer[OFFSET_SMCPU_CRC + 1 ]  = SMCPU_CRC.Byte.Byte2;
    GLCD_Info.Message_Buffer[OFFSET_SMCPU_CRC + 2 ]  = SMCPU_CRC.Byte.Byte3;
    GLCD_Info.Message_Buffer[OFFSET_SMCPU_CRC + 3 ]  = SMCPU_CRC.Byte.Byte4;
    GLCD_Info.Message_Buffer[OFFSET_SMCPU_CRC + 4 ]  = Event_Logger_ID;

}

