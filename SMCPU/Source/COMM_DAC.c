/*********************************************************************************************************************
 *	Project		: 	Single Section Digital Axle Counter
 *	Version		: 	2.0
 *	Revision	:	1
 *	Filename	: 	comm_dac.c
 *	Target MCU	: 	PIC24FJ256GB210
 *  Compiler	: 	XC16 V1.31
 *	Author		:	EM003
 *	Date		:
 *	Company Name: 	Insys
 *	Modification History:
 *					Rev No			Date		Description
 *					 --				 --				--
 *	Functions	:   void Initialise_SPI(void);
 *					void Update_SPI_State(void);
 *					void Set_SPI_Sch_Idle(void);
 *					void Decrement_SPI_Sch_msTmr(void);
 *					void Process_SPI_Message(BYTE uchCPU_ID);
 *					void Build_Message(BYTE uchCPU_ID);
 *					void Update_Shadow_Register(BYTE uchCPU);
 *					void Synchronise_Shadow_Register(void);
 *                  void Update_Comm_Err_Counter(BYTE uchCPU_ID);                        `
 *
 ******************************************************************************************************************/
#include <xc.h>

#include "COMMON.h"
#include "COMM_DAC.h"
#include "CRC16.h"
#include "EEPROM.h"
#include "EVENTS.h"

BYTE CPU1_data[SPI_MESSAGE_LENGTH];			/* Buffer to store data recived from Cpu1 */			
BYTE CPU2_data[SPI_MESSAGE_LENGTH];			/* Buffer to store data recived from Cpu2 */
BYTE CPU1_data_GLCD[GCPU_SPI_MESSAGE_LENGTH];			/* Buffer to store data recived from Cpu1 */
BYTE CPU2_data_GLCD[GCPU_SPI_MESSAGE_LENGTH];			/* Buffer to store data recived from Cpu2 */




BYTE Transmit_Queue[SPI_MESSAGE_LENGTH];	/* Buffer which holds the data to be transmitted to cpu */
BYTE var_count, SF_Error,EF_Error;

disp_info_t Disp_Info;
comm_sch_info_t SPI_Sch_Info;			/* structure to hold inforamtion of SPI scheduler */
spirecvobject SPIRecvObject;					/* SPI Message Receiving Buffer */
event_register_t Shadow[MAXIMUM_NO_OF_CPU]; /* shadow will have present status of Both Cpus */


UINT16 temp_crc_SPI;
BYTE log_event, System_error_code;

/* The Transmission from CPU to the SM_CPU is through the SPI. The software routine shall
 *  initilaize the ports either as output or input based on the usage of the ports. The Software
 *  shall also intialize the schedulers state, timeout and scan rate. */
/*********************************************************************************
 *File name 		:comm_dac.c
 *Function Name		:void Initialise_SPI(void)
 *Created By		:EM003
 *Date Created		:
 *Modification History:
 *					Rev No			Date		Description
 *					 --				 --				--
 *Tracability:
 *		SRS()    	:
 *
 *Abstract			:SPI configuration and SPI scheduler Initilaistion will be done.
 *Algorithm			:
 *Description		:Initilaise SPI and synchronize shadow register.
 *Input Element		:None
 *Output Element	:void
 *
 **********************************************************************************/
void Initialise_SPI(void)
{
	SS_CPU1_PORT_DIR = OUTPUT_PORT;				     /* configure SS1 - Slave Select CPU1 is Output */
	SS_CPU2_PORT_DIR = OUTPUT_PORT;				     /* configure SS2 - Slave Select CPU2 is Output */
	SS_CPU1_PORT = SPI_DESELECT_SLAVE_DEVICE;        /* Deselect SPI Slave -CPU1 */
	SS_CPU2_PORT = SPI_DESELECT_SLAVE_DEVICE;		 /* Deselect SPI Slave -CPU2 */

	SPI_Sch_Info.State = SPI_SCHEDULER_IDLE;		 /* set SPI Scheduler to Idle State */
	SPI_Sch_Info.Timeout_ms = 0;					 /* set SPI Scheduler timeout to zero */
	SPI_Sch_Info.ScanRate = SPI_SCHEDULER_SCAN_RATE; /* set SPI Scheduler Scan rate to SPI_SCHEDULER_SCAN_RATE */

    __builtin_write_OSCCONL(OSCCON & 0xbf); //clear the bit 6 of OSCCONL to unlock Pin Re-map
    RPINR22bits.SDI2R = 26;
    RPOR10bits.RP21R  = 11;
    RPOR9bits.RP19R   = 10;

    __builtin_write_OSCCONL(OSCCON | 0x40); //set the bit 6 of OSCCONL to lock Pin Re-map

    SPI2CON2bits.SPIBEN = SET_HIGH;
    IFS2bits.SPI2IF = SET_LOW;
    IEC2bits.SPI2IE = SET_LOW;
    SPI2CON1 = SPI_MASTER;  // select mode
    SPI2STAT = SPI_ENABLE;  // enable the peripheral

	Synchronise_Shadow_Register();					 /* copy Shadow_Register with EEPROM Stored Value at Boot Time */
    Erase_Start = 0;

}

/*********************************************************************************
 *File name 		:comm_dac.c
 *Function Name		:void Update_SPI_State(void)
 *Created By		:EM003
 *Date Created		:
 *Modification History:
 *					Rev No			Date		Description
 *					 --				 --				--
 *Tracability:
 *		SRS()    	:
 *
 *Abstract			: SPI Scheduler which implements SPI protocol to exchange data with both Cpus
 *Algorithm			:
 *						1.Initially SPI scheduler will be in Idle State.
 *						2.After Spi_Scan time completed, it will select Appropriate Cpu to which it wants to
 *						   Exchange the data.
 *						3.After Pre_Transmission delay, it will goto step 4.
 *						4.Transmit One Byte and Receive One Byte.
 *						5.Check for All Bytes are Transmitted. If Not it will repeat Step 4.
 *						6.It will check CRC for recived Bytes. If Crc matches it will process the message.
 *						7.It will deselect the selected cpu and Select the other Cpu for Communication.
 *						8.It will goto Idle State with calculated Scan time.
 *                      9.Indicate valid and invalid CPU packect
 *                      10. Track erase  
 *Description		:
 *Input Element		:None
 *Output Element	:void
 *
 **********************************************************************************/
void Update_SPI_State(void)
{
	static BYTE uchSelectedCPU = 1,index;                    /* uchSelectedCPU holds selected Cpu Id to which SM cpu is communicating, index will indicate the Byte No */
	static BYTE uchBytes_to_Transmit;   /* uchBytes_to_Transmit holds No of Bytes to be exchange*/

    if(Erase_Start == 1)
    {
        LATDbits.LATD12 = SET_HIGH;
    }
    if(inspect_event_done == 0)
    {
        return;
    }
	switch (SPI_Sch_Info.State)
	{
	case SPI_SCHEDULER_IDLE:
		if (SPI_Sch_Info.Timeout_ms == TIMEOUT_EVENT)	/* SPI- scheduler scan time completed, So, Select the appropriate cpu */
		{
			if (uchSelectedCPU == 1)
			{
				/* Select  CPU1 */
				SS_CPU1_PORT = SPI_SELECT_SLAVE_DEVICE;			/* CPU1 is Selceted */
				SS_CPU2_PORT = SPI_DESELECT_SLAVE_DEVICE;		/* CPU2 is De-Selceted */
			}
			else
			{
				/* Select CPU2 */
				SS_CPU2_PORT = SPI_SELECT_SLAVE_DEVICE;			/* CPU2 is Selceted */
				SS_CPU1_PORT = SPI_DESELECT_SLAVE_DEVICE;		/* CPU1 is De-Selceted */
			}
			index =0;
			Build_Message(uchSelectedCPU);						  /* Bulid the message to transmit */
			SPI_Sch_Info.Timeout_ms = SPI_PRE_TRANSMISSION_DELAY; /* Set pre transmission delay */
			SPI_Sch_Info.State = SPI_BREATHING_TIME_FOR_SLAVE;
		}
		break;
	case SPI_BREATHING_TIME_FOR_SLAVE:
		if (SPI_Sch_Info.Timeout_ms == TIMEOUT_EVENT)
		{
			SPI_Sch_Info.State = SPI_WAIT_FOR_XMIT_TO_FINISH;	  /* pre transmission delay completed, so state is changed to transmission */
			uchBytes_to_Transmit = (BYTE) SPI_MESSAGE_LENGTH + 1; /* No of Bytes to be exchanged is assigned.one Byte is added to Actual Length.
  																   * Becuse First Byte received from Cpu is Dummy Byte */
			SPIRecvObject.Index = 0;							  /* Receive Buffer index is assigned to 0*/
		}
		break;
	case SPI_WAIT_FOR_XMIT_TO_FINISH:        
        if(Erase_Start == 1)
        {

            Erase_Start = 0;
        }
		if (SPI_Sch_Info.Timeout_ms == TIMEOUT_EVENT)
		{
			if (uchBytes_to_Transmit > 1)
			{
				while(SPI2STATbits.SPITBF == 0)//if (BF == 0)					   /* Still all Bytes are not completely transmitted */
				{
                    /* SSPBUF empty - Transmit Status, So we can Put the Next Byte */
                    SPI2BUF = Transmit_Queue[index];
                    index = index + 1;
                    uchBytes_to_Transmit = uchBytes_to_Transmit - 1;
                }
				SPI_Sch_Info.State = SPI_WAIT_FOR_RECEIVE_TO_COMPLETE;  /* Go to Receiving State */
			}
			else
			{
				SPI_Sch_Info.State = SPI_CHECK_CRC;	/* All Bytes transmission and Reception Over, So go to CRC Check to validate Received Data */
			}
		}
		break;
	case SPI_WAIT_FOR_RECEIVE_TO_COMPLETE:
        if(SPI2STATbits.SPIRBF)
        {
            while(!SPI2STATbits.SRXMPT)
			{
                /* SSPBUF full - Received data from Slave */
                SPIRecvObject.Msg_Buffer[SPIRecvObject.Index] = (BYTE)SPI2BUF;
                SPIRecvObject.Index = SPIRecvObject.Index + 1;
            }
			SPI_Sch_Info.State = SPI_WAIT_FOR_XMIT_TO_FINISH;             /* go to Transmitting State */
			SPI_Sch_Info.Timeout_ms = SPI_DELAY_BETWEEN_BYTES;			  /* Introduce A time delay Between Bytes */
		}
		break;
	case SPI_CHECK_CRC:
        temp_crc_SPI = Crc16(SPI_RECV_OBJ, SPI_MESSAGE_LENGTH-2);
		if (temp_crc_SPI == ((UINT16)((UINT16)SPIRecvObject.Msg_Buffer[SPI_MESSAGE_LENGTH-1]<<8)+(SPIRecvObject.Msg_Buffer[SPI_MESSAGE_LENGTH-2])))
		{
			/* valid CRC-16 Checksum */
	 	    Process_SPI_Message(uchSelectedCPU);						     /* Received  Data is Valid, Process the Data */
            LATDbits.LATD12 = SET_LOW;
            LATDbits.LATD2  = SET_HIGH;
		}
        else
        {
            LATDbits.LATD12 = SET_HIGH;
            LATDbits.LATD2  = SET_LOW;
        }
		if (uchSelectedCPU == 1)
		{
			/* CPU1 over, now select CPU2 */
			uchSelectedCPU = 2;											/* Select Cpu 2 */
		}
		else
		{
			/* CPU2 over, now select CPU1 */
			uchSelectedCPU = 1;											/* Select Cpu 1 */
		}
		SPI_Sch_Info.Timeout_ms = SPI_HOLD_SS_LOW_TIMEOUT;
		SPI_Sch_Info.State = SPI_HOLD_SS_LOW;
		break;
	case SPI_HOLD_SS_LOW:
		if (SPI_Sch_Info.Timeout_ms == TIMEOUT_EVENT)
		{
			Set_SPI_Sch_Idle();											/* this function is called to Set the time gap for Next Transmission */
		}
		break;
    default:
        break;
	}
}

/*********************************************************************************
 *File name 		:comm_dac.c
 *Function Name		:void Set_SPI_Sch_Idle(void)
 *Created By		:EM003
 *Date Created		:
 *Modification History:
 *					Rev No			Date		Description
 *					 --				 --				--
 *Tracability:
 *		SRS()    	:
 *
 *Abstract			: Make the SPI Scheduler ready for Next Transmission
 *Algorithm			:
 *Description		:
 *Input Element		:None
 *Output Element	:void
 *
 ***********************************************************************************/

void Set_SPI_Sch_Idle(void)
{
	UINT16 uiTmp;

	SPI_Sch_Info.State = SPI_SCHEDULER_IDLE;          /* Set SPI Scheduler to Idle State */
	/* Elapsed time is Time Taken By Sm CPU to Exchange ALL Bytes with one Cpu */
	if (SPI_Sch_Info.ElapsedTime > SPI_Sch_Info.ScanRate)
	{
		uiTmp = (SPI_Sch_Info.ElapsedTime % SPI_SCHEDULER_SCAN_RATE);
	}
	else
	{
		uiTmp = SPI_Sch_Info.ElapsedTime;
	}
	SPI_Sch_Info.Timeout_ms = SPI_Sch_Info.ScanRate - uiTmp;
	/* Release SS signal */
	SS_CPU1_PORT = SPI_DESELECT_SLAVE_DEVICE;		/* De-Select CPU1 */
	SS_CPU2_PORT = SPI_DESELECT_SLAVE_DEVICE;		/* De-Select CPU2 */
}

/*********************************************************************************
 *File name 		:comm_dac.c
 *Function Name		:void Decrement_SPI_Sch_msTmr(void)
 *Created By		:EM003
 *Date Created		:
 *Modification History:
 *					Rev No			Date		Description
 *					 --				 --				--
 *Tracability:
 *		SRS()    	:
 *
 *Abstract			: For Every 1 Ms, the 1ms Timer-variables are Updated.
 *Algorithm			:
 *Description		:
 *Input Element		:None
 *Output Element	:void
 *
 **********************************************************************************/

void Decrement_SPI_Sch_msTmr(void)
{
	if (SPI_Sch_Info.Timeout_ms > 0)
	{
        SPI_Sch_Info.Timeout_ms = SPI_Sch_Info.Timeout_ms - 1;              /* SPI_Sch_Info.Timeout_ms is decremented */
	}
	if (SPI_Sch_Info.State == SPI_SCHEDULER_IDLE)
	/* when SPI_Scheduler is in a state other than Idle State it is in the Transmission,
	   So Increment Elapsed Time */
	{
        SPI_Sch_Info.ElapsedTime = 0;
	}
	else
	{
        SPI_Sch_Info.ElapsedTime = SPI_Sch_Info.ElapsedTime + 1;
	}
}

/*********************************************************************************
 *File name 		:comm_dac.c
 *Function Name		:void Process_SPI_Message(BYTE uchCPU_ID)
 *Created By		:EM003
 *Date Created		:
 *Modification History:
 *					Rev No			Date		Description
 *					 --				 --				--
 *Tracability:
 *		SRS()    	:
 *
 *Abstract			: Process the Message Received
 *Algorithm			:
 *Description		:
 *Input Element		:uchCPU_ID - represent the CPU No (CPU1 or Cpu2)
 *Output Element	:void
 *
 ***********************************************************************************/
void Process_SPI_Message(BYTE uchCPU_ID)
{
	BYTE uchbyte=0, uchCommand;

	if (uchCPU_ID == CPU1_ID)
	{
		/* If Data received from CPU1, then store into CPU1 Buffer */
		for(uchbyte =0 ;uchbyte<SPI_MESSAGE_LENGTH;uchbyte++)
		{
            CPU1_data[uchbyte] = SPIRecvObject.Msg_Buffer[uchbyte];
            CPU1_data_GLCD[uchbyte] = SPIRecvObject.Msg_Buffer[uchbyte];
			Latest_DAC_data[uchbyte] = SPIRecvObject.Msg_Buffer[uchbyte];
		}
    	CPU1_Address = CPU1_data[0];		/* Save the address of CPU1 into Cpu1_Addrss Variable */
        System_error_code = CPU1_data[69];
	}
	if (uchCPU_ID == CPU2_ID)
	{
		/* If Data received from CPU2, then store into CPU2 Buffer */
		for(uchbyte =0 ;uchbyte<SPI_MESSAGE_LENGTH;uchbyte++)
		{
		    CPU2_data[uchbyte] = SPIRecvObject.Msg_Buffer[uchbyte];
            CPU2_data_GLCD[uchbyte] = SPIRecvObject.Msg_Buffer[uchbyte];
            Latest_DAC_data[uchbyte] = SPIRecvObject.Msg_Buffer[uchbyte];
		}		
	    CPU2_Address = CPU2_data[0];		/* Save the address of CPU2 into Cpu2_Addrss Variable */
        System_error_code = CPU2_data[69];
	}
        
    uchCommand = SPIRecvObject.Msg_Buffer[MESSAGE_TYPE_OFFSET];
    if(uchCommand == READ_RESET_INFO)
    {
        DAC_sysinfo.Unit_Type = SPIRecvObject.Msg_Buffer[UNIT_TYPE_OFFSET];
        DAC_sysinfo.SW_Version = SPIRecvObject.Msg_Buffer[SOFWARE_VERSION_OFFSET];
        DAC_sysinfo.Checksum.DWord.HiWord.Byte.Hi = SPIRecvObject.Msg_Buffer[VALIDATED_CHECKSUM_OFFSET];
        DAC_sysinfo.Checksum.DWord.HiWord.Byte.Lo = SPIRecvObject.Msg_Buffer[VALIDATED_CHECKSUM_OFFSET + 1];
        DAC_sysinfo.Checksum.DWord.LoWord.Byte.Hi = SPIRecvObject.Msg_Buffer[VALIDATED_CHECKSUM_OFFSET + 2];
        DAC_sysinfo.Checksum.DWord.LoWord.Byte.Lo = SPIRecvObject.Msg_Buffer[VALIDATED_CHECKSUM_OFFSET + 3];
        if (DAC_sysinfo.Checksum.LWord == 0L)
        {
            DAC_sysinfo.Checksum.DWord.HiWord.Byte.Hi = SPIRecvObject.Msg_Buffer[LOCAL_CHECKSUM_OFFSET];
            DAC_sysinfo.Checksum.DWord.HiWord.Byte.Lo = SPIRecvObject.Msg_Buffer[LOCAL_CHECKSUM_OFFSET + 1];
            DAC_sysinfo.Checksum.DWord.LoWord.Byte.Hi = SPIRecvObject.Msg_Buffer[LOCAL_CHECKSUM_OFFSET + 2];
            DAC_sysinfo.Checksum.DWord.LoWord.Byte.Lo = SPIRecvObject.Msg_Buffer[LOCAL_CHECKSUM_OFFSET + 3];
        }
    }
	Update_Comm_Err_Counter(uchCPU_ID);	/* update Count for the Crc error Occured in External Communication */
	Update_Shadow_Register(uchCPU_ID);	/* Update our copy of event register */
    Disp_Info.Unit_Type = (Unit_Type_info)DAC_sysinfo.Unit_Type;
    Disp_Info.MessageID = CPU1_data_GLCD[77];//Message ID
    Disp_Info.Reset_mode = (Reset_info)CPU1_data_GLCD[26];//Local and Remote System Mode
    Disp_Info.DS_mode = (Reset_info)(CPU1_data_GLCD[26] & 0x0F);
    Disp_Info.US_mode = (Reset_info)((BYTE)(CPU1_data_GLCD[26] & 0xF0)>>4);
    var_count = CPU1_data_GLCD[69];//ErrorCodeForSPI
    SF_Error = CPU1_data_GLCD[28];//Up stream Error Code
    EF_Error = CPU1_data_GLCD[27];//Down stream Error Code
	Detect_DAC_Events(uchCPU_ID, Shadow[(uchCPU_ID - 1)]);	/* Event detection */
}

/*********************************************************************************
 *File name 		:comm_dac.c
 *Function Name		:void Build_Message(BYTE uchCPU_ID)
 *Created By		:EM003
 *Date Created		:
 *Modification History:
 *					Rev No			Date		Description
 *					 --				 --				--
 *Tracability:
 *		SRS()    	:
 *
 *Abstract			: Builds The Transmit Queue with Cpu Info to Transmit to CPu
 *Algorithm			:
 *Description		:
 *Input Element		:uchCPU_ID - represent the CPU No (CPU1 or Cpu2)
 *Output Element	:void
 *
 ***********************************************************************************/

void Build_Message(BYTE uchCPU_ID)
{
	BYTE uchbyte=0;

	if (uchCPU_ID == CPU1_ID)
	{
		/* Build CPU2 information to transmit to CPU1  */
		for(uchbyte =0;uchbyte < SPI_MESSAGE_LENGTH;uchbyte++)
		{
	   	  Transmit_Queue[uchbyte] = CPU2_data[uchbyte];
		  /* After Building Cpu2 data to sent to Cpu1, Cpu2 data has to be cleaned.
		   * otherwise the same message will get retransmitted in case of Cpu2 failed to send New Data */
		  CPU2_data[uchbyte] = (BYTE) 0;
		}
	}
	if (uchCPU_ID == CPU2_ID)
	{
		/* Build CPU1 information to transmit to CPU2  */
		for(uchbyte =0;uchbyte < SPI_MESSAGE_LENGTH;uchbyte++)
		{
		  Transmit_Queue[uchbyte] = CPU1_data[uchbyte];
           /* After Building Cpu1 data to sent to Cpu2, Cpu1 data has to cleaned
	       * otherwise the same message will get retransmitted in case of Cpu1 failed to send New Data */
		  CPU1_data[uchbyte] = (BYTE) 0;
		}
	}
}

/*********************************************************************************
 *File name 		:comm_dac.c
 *Function Name		:void Update_Shadow_Register(BYTE uchCPU)
 *Created By		:EM003
 *Date Created		:
 *Modification History:
 *					Rev No			Date		Description
 *					 --				 --				--
 *Tracability:
 *		SRS()    	:
 *
 *Abstract			: Shadow Register will be updated by New Data Received from CPU
 *Algorithm			:
 *Description		:
 *Input Element		:uchCPU - represent the CPU No (CPU1 or Cpu2)
 *Output Element	:void
 *
 ***********************************************************************************/
void Update_Shadow_Register(BYTE uchCPU)
{
	BYTE uchbyte1, uchbyte2;
	dac_status_t DAC_Status;
	BOOL bUS_Healthy = FALSE;
	BOOL bDS_Healthy = FALSE;

	if (uchCPU != CPU1_ID && uchCPU != CPU2_ID)
	{
		return;		/* Invalid range */
	}
	if (Status.Flags.Inhibit_Logging_P == SET_HIGH && Status.Flags.Inhibit_Logging_N == SET_LOW)
	{
		return;		/* Data logging is INHIBITED, no point in updating */
	}
	uchbyte1 = (uchCPU - 1);

	/* Initialise the pointer to Appropriate Cpu Buffers location*/

	if (uchCPU == CPU1_ID)
	{
        for (uchbyte2 = 0; uchbyte2 < 7; uchbyte2++)
        {
            DAC_Status.Byte[uchbyte2] = CPU1_data[uchbyte2+1];
        }
	}
	else
	{
        for (uchbyte2 = 0; uchbyte2 < 7; uchbyte2++)
        {
            DAC_Status.Byte[uchbyte2] = CPU2_data[uchbyte2+1];
        }
	}

	/* MAP Status data to Shadow event register */
	Shadow[uchbyte1].Id.US_Block  = DAC_Status.Flags.US_Track_Status;
	Shadow[uchbyte1].Id.DS_Block  = DAC_Status.Flags.DS_Track_Status;
	Shadow[uchbyte1].Id.DAC_DS_Reset = DAC_Status.Flags.Local_Reset_Done;
	Shadow[uchbyte1].Id.DAC_US_Reset = DAC_Status.Flags.Local_Reset_Done2;
	Shadow[uchbyte1].Id.System    = DAC_Status.Flags.System_Status;
	Shadow[uchbyte1].Id.Direct_Out_Count = DAC_Status.Flags.Direct_Out_Count;
	Shadow[uchbyte1].Id.Board     = DAC_Status.Flags.Unit_Board_Status;
	Shadow[uchbyte1].Id.PD1_Board = DAC_Status.Flags.PD1_Board_Present;
	Shadow[uchbyte1].Id.PD2_Board = DAC_Status.Flags.PD2_Board_Present;
	Shadow[uchbyte1].Id.Modem_Board_A = DAC_Status.Flags.Modem_Card_A_Present;
	Shadow[uchbyte1].Id.Modem_Board_B = DAC_Status.Flags.Modem_Card_B_Present;
	Shadow[uchbyte1].Id.Relay_Board_A = DAC_Status.Flags.Relay_Drive_A_Present;
	Shadow[uchbyte1].Id.Relay_Board_B = DAC_Status.Flags.Relay_Drive_B_Present;
	Shadow[uchbyte1].Id.Peer_CPU_Board = DAC_Status.Flags.Peer_Cpu_Present;
	Shadow[uchbyte1].Id.LU1_US1_Comm = DAC_Status.Flags.LU1_to_US1_Link;
	Shadow[uchbyte1].Id.LU1_US2_Comm = DAC_Status.Flags.LU1_to_US2_Link;
	Shadow[uchbyte1].Id.LU1_DS1_Comm = DAC_Status.Flags.LU1_to_DS1_Link;
	Shadow[uchbyte1].Id.LU1_DS2_Comm = DAC_Status.Flags.LU1_to_DS2_Link;
	Shadow[uchbyte1].Id.LU2_US1_Comm = DAC_Status.Flags.LU2_to_US1_Link;
	Shadow[uchbyte1].Id.LU2_US2_Comm = DAC_Status.Flags.LU2_to_US2_Link;
	Shadow[uchbyte1].Id.LU2_DS1_Comm = DAC_Status.Flags.LU2_to_DS1_Link;
	Shadow[uchbyte1].Id.LU2_DS2_Comm = DAC_Status.Flags.LU2_to_DS2_Link;
	Shadow[uchbyte1].Id.US1_LU1_Comm = DAC_Status.Flags.US1_to_LU1_Link;
	Shadow[uchbyte1].Id.US2_LU1_Comm = DAC_Status.Flags.US2_to_LU1_Link;
	Shadow[uchbyte1].Id.DS1_LU1_Comm = DAC_Status.Flags.DS1_to_LU1_Link;
	Shadow[uchbyte1].Id.DS2_LU1_Comm = DAC_Status.Flags.DS2_to_LU1_Link;
	Shadow[uchbyte1].Id.US1_LU2_Comm = DAC_Status.Flags.US1_to_LU2_Link;
	Shadow[uchbyte1].Id.US2_LU2_Comm = DAC_Status.Flags.US2_to_LU2_Link;
	Shadow[uchbyte1].Id.DS1_LU2_Comm = DAC_Status.Flags.DS1_to_LU2_Link;
	Shadow[uchbyte1].Id.DS2_LU2_Comm = DAC_Status.Flags.DS2_to_LU2_Link;
	Shadow[uchbyte1].Id.PD1 = DAC_Status.Flags.PD1_Status;
	Shadow[uchbyte1].Id.PD2 = DAC_Status.Flags.PD2_Status;
	Shadow[uchbyte1].Id.Peer_CPU = DAC_Status.Flags.Peer_System_Status;
	Shadow[uchbyte1].Id.Peer_CPU_Comm = DAC_Status.Flags.Peer_CPU_Link;
	Shadow[uchbyte1].Id.Modem_A = DAC_Status.Flags.Modem_A;
	Shadow[uchbyte1].Id.Modem_B = DAC_Status.Flags.Modem_B;
	Shadow[uchbyte1].Id.US_DAC  = DAC_Status.Flags.US_System_Status;
	Shadow[uchbyte1].Id.DS_DAC  = DAC_Status.Flags.DS_System_Status;
	Shadow[uchbyte1].Id.US1_Power = DAC_Status.Flags.Power_Fail_at_US1;
	Shadow[uchbyte1].Id.US2_Power = DAC_Status.Flags.Power_Fail_at_US2;
	Shadow[uchbyte1].Id.DS1_Power = DAC_Status.Flags.Power_Fail_at_DS1;
	Shadow[uchbyte1].Id.DS2_Power = DAC_Status.Flags.Power_Fail_at_DS2;
	if (DAC_Status.Flags.System_Status == SET_HIGH && DAC_Status.Flags.US_System_Status == SET_HIGH)
	{
		bUS_Healthy = TRUE;			/* UP Stream section healthy */
	}
	if (DAC_Status.Flags.System_Status == SET_HIGH && DAC_Status.Flags.DS_System_Status == SET_HIGH)
	{
		bDS_Healthy = TRUE;			/* DOWN Stream section healthy */
	}
	if (bUS_Healthy == SET_LOW)
	{
		/* Failure in UP Stream section */
		if (DAC_Status.Flags.Vital_Relay_A != SET_HIGH)
		{
			Shadow[uchbyte1].Id.Vital_Relay_A = DAC_Status.Flags.Vital_Relay_A;
		}
		if (DAC_Status.Flags.Preparatory_Relay1 != SET_HIGH)
		{
			Shadow[uchbyte1].Id.Prep_Relay_A = DAC_Status.Flags.Preparatory_Relay1;
		}
	}
	else
	{
		/* UP Stream section healthy */
		if (DAC_Status.Flags.Vital_Relay_A != SET_LOW)
		{
			Shadow[uchbyte1].Id.Vital_Relay_A = DAC_Status.Flags.Vital_Relay_A;
		}
		if (DAC_Status.Flags.Preparatory_Relay1 != SET_LOW)
		{
			Shadow[uchbyte1].Id.Prep_Relay_A = DAC_Status.Flags.Preparatory_Relay1;
		}
	}

	if (bDS_Healthy == SET_LOW)
	{
		/* Failure in DOWN Stream section */
		if (DAC_Status.Flags.Vital_Relay_B != SET_HIGH)
		{
			Shadow[uchbyte1].Id.Vital_Relay_B = DAC_Status.Flags.Vital_Relay_B;
		}
		if (DAC_Status.Flags.Preparatory_Relay != SET_HIGH)
		{
			Shadow[uchbyte1].Id.Prep_Relay_B = DAC_Status.Flags.Preparatory_Relay;
		}
	}
	else
	{
		/* DOWN Stream section healthy */
		if (DAC_Status.Flags.Vital_Relay_B != SET_LOW)
		{
			Shadow[uchbyte1].Id.Vital_Relay_B = DAC_Status.Flags.Vital_Relay_B;
		}
		if (DAC_Status.Flags.Preparatory_Relay != SET_LOW)
		{
			Shadow[uchbyte1].Id.Prep_Relay_B = DAC_Status.Flags.Preparatory_Relay;
		}
	}
}

/*********************************************************************************
File name 			:comm_dac.c
Function Name		:void Synchronise_Shadow_Register(void)
Created By			:EM003
Date Created		:
Modification History:
					Rev No			Date		Description
					 --				 --				--
Tracability:
		SRS()    	:

Abstract			: Shadow Register will be updated by Status information Stored On onchip EEPROM
Algorithm			:
Description			:
Input Element		:None
Output Element		:void

**********************************************************************************/

void Synchronise_Shadow_Register(void)
{
	BYTE uchbyte=0;

	if (Is_CPU1_EEPROM_Record_Valid())  /* Check the CPU1 Data stored on EEPROM is Valid or Not */
	{
		/* The stored data is Valid, So Load value from on-chip EEPROM */
		for (uchbyte = 0; uchbyte < NO_OF_EVENT_REGISTERS; uchbyte++)
		{
			Shadow[0].Byte[uchbyte] = EEPROM_Sch_Info.CPU1_Data[uchbyte];//*pEEPROM_data;
		}
	}
	else
	{
		/* The stored data is not Valid, So Load with Default Values */
		Shadow[0].Byte[0] = (BYTE) EVENT_REGISTER1_DEFAULT;
		Shadow[0].Byte[1] = (BYTE) EVENT_REGISTER2_DEFAULT;
		Shadow[0].Byte[2] = (BYTE) EVENT_REGISTER3_DEFAULT;
		Shadow[0].Byte[3] = (BYTE) EVENT_REGISTER4_DEFAULT;
		Shadow[0].Byte[4] = (BYTE) EVENT_REGISTER5_DEFAULT;
		Shadow[0].Byte[5] = (BYTE) EVENT_REGISTER6_DEFAULT;
	}
	if (Is_CPU2_EEPROM_Record_Valid()) /* Check the CPU1 Data stored on EEPROM is Valid or Not */
	{
		/* The stored data is Valid, So Load value from on-chip EEPROM */
		for (uchbyte = 0; uchbyte < NO_OF_EVENT_REGISTERS; uchbyte++)
		{
			Shadow[1].Byte[uchbyte] = EEPROM_Sch_Info.CPU2_Data[uchbyte];
		}
	}
	else
	{
		/* The stored data is not Valid, So Load with Default Values */
		Shadow[1].Byte[0] = (BYTE) EVENT_REGISTER1_DEFAULT;
		Shadow[1].Byte[1] = (BYTE) EVENT_REGISTER2_DEFAULT;
		Shadow[1].Byte[2] = (BYTE) EVENT_REGISTER3_DEFAULT;
		Shadow[1].Byte[3] = (BYTE) EVENT_REGISTER4_DEFAULT;
		Shadow[1].Byte[4] = (BYTE) EVENT_REGISTER5_DEFAULT;
		Shadow[1].Byte[5] = (BYTE) EVENT_REGISTER6_DEFAULT;
	}
}

/*********************************************************************************
 *File name 		:comm_dac.c
 *Function Name		:void Update_Comm_Err_Counter(BYTE uchCPU_ID)
 *Created By		:EM003
 *Date Created		:
 *Modification History:
 *					Rev No			Date		Description
 *					 --				 --				--
 *Tracability:
 *		SRS()    	:
 *
 *Abstract			: Update communication Crc error counts of Both CPU
 *Algorithm			:
 *					 1.For Every Crc error Dac cpu will toggle error Bit.
 *					 2.each CPU will have two error Bits, one For Modem A and Modem B
 *					 3.For Every State Change of these bits, this fuction will increment the error Count.
 *
 *Description		:
 *Input Element		:uchCPU_ID - represent the CPU No (CPU1 or Cpu2)
 *Output Element	:void
 *
 ***********************************************************************************/

void Update_Comm_Err_Counter(BYTE uchCPU_ID)
{
	bitadrb_t Buffer;

	if (uchCPU_ID != CPU1_ID && uchCPU_ID != CPU2_ID)
	{
		return;		/* Invalid range */
	}

	if(uchCPU_ID == CPU1_ID)
	{
		Buffer.Byte = CPU1_data[7];
		/* Update New state of CPU1 Comm A and Comm B Error Bit*/
		Dac_Comm_Err.Flags.New_Cpu1_CommA_Err_State = Buffer.Bit.b2;
		Dac_Comm_Err.Flags.New_Cpu1_CommB_Err_State = Buffer.Bit.b3;

		if( Dac_Comm_Err.Flags.New_Cpu1_CommA_Err_State != Dac_Comm_Err.Flags.Old_Cpu1_CommA_Err_State)
		{
			 /* New state is not same as old State, Update the counter.*/
			if(Dac_Comm_Err.CPU1_CommA_Error_Count > LARGEST_CONVERTABLE_INTEGER)
			{
            	Dac_Comm_Err.CPU1_CommA_Error_Count =0;
			}
			else
			{
				Dac_Comm_Err.CPU1_CommA_Error_Count = Dac_Comm_Err.CPU1_CommA_Error_Count + 1;
			}
		}
		if( Dac_Comm_Err.Flags.New_Cpu1_CommB_Err_State != Dac_Comm_Err.Flags.Old_Cpu1_CommB_Err_State)
		{
			 /* New state is not same as old State, Update the counter.*/
			if(Dac_Comm_Err.CPU1_CommB_Error_Count > LARGEST_CONVERTABLE_INTEGER)
			{
            	Dac_Comm_Err.CPU1_CommB_Error_Count =0;
			}
			else
			{
				Dac_Comm_Err.CPU1_CommB_Error_Count = Dac_Comm_Err.CPU1_CommB_Error_Count + 1;
			}
		 }
		/* Update old state with New state*/
		Dac_Comm_Err.Flags.Old_Cpu1_CommA_Err_State = Dac_Comm_Err.Flags.New_Cpu1_CommA_Err_State;
		Dac_Comm_Err.Flags.Old_Cpu1_CommB_Err_State = Dac_Comm_Err.Flags.New_Cpu1_CommB_Err_State;
	}
	else
	{
		Buffer.Byte = CPU2_data[7];
		/* Update New state of CPU1 Comm A and Comm B Error Bit*/
		Dac_Comm_Err.Flags.New_Cpu2_CommA_Err_State = Buffer.Bit.b2;
		Dac_Comm_Err.Flags.New_Cpu2_CommB_Err_State = Buffer.Bit.b3;

		if( Dac_Comm_Err.Flags.New_Cpu2_CommA_Err_State !=  Dac_Comm_Err.Flags.Old_Cpu2_CommA_Err_State)
		  {
                /* New state is not same as old State, Update the counter.*/
               if(Dac_Comm_Err.CPU2_CommA_Error_Count > LARGEST_CONVERTABLE_INTEGER)
               {
                   Dac_Comm_Err.CPU2_CommA_Error_Count =0;
               }
               else
               {
                   Dac_Comm_Err.CPU2_CommA_Error_Count = Dac_Comm_Err.CPU2_CommA_Error_Count + 1;
               }
		  }
		if( Dac_Comm_Err.Flags.New_Cpu2_CommB_Err_State != Dac_Comm_Err.Flags.Old_Cpu2_CommB_Err_State)
		  {
                /* New state is not same as old State, Update the counter.*/
                if(Dac_Comm_Err.CPU2_CommB_Error_Count > LARGEST_CONVERTABLE_INTEGER)
                {
                    Dac_Comm_Err.CPU2_CommB_Error_Count =0;
                }
                else
                {
                    Dac_Comm_Err.CPU2_CommB_Error_Count = Dac_Comm_Err.CPU2_CommB_Error_Count + 1;
                }
		  }
		/* Update old state with New state*/
		Dac_Comm_Err.Flags.Old_Cpu2_CommA_Err_State = Dac_Comm_Err.Flags.New_Cpu2_CommA_Err_State;
		Dac_Comm_Err.Flags.Old_Cpu2_CommB_Err_State = Dac_Comm_Err.Flags.New_Cpu2_CommB_Err_State;
	}
}
