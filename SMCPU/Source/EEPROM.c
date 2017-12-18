/*********************************************************************************************************************
*	Project		: 	Single Section Digital Axle Counter
*	Version		: 	2.0 
*	Revision	:	1	
*	Filename	: 	eeprom.c
*	Target MCU	: 	PIC24FJ256GB210   
*    Compiler	: 	XC16 Compiler V1.31  
*	Author		:	EM003
*	Date		:	
*	Company Name: 	Insys Digital Systems
*	Modification History:
*					Rev No			Date		Description
*					 --				 --				--    
*	Functions	:   
*                    void Initialise_EEPROM_State(void);
*                    void Start_EEPROM_Sch(void);
*                    void Update_EEPROM_State(void);
*                    void Decrement_EEPROM_msTmr(void);
*                    void Save_Event_Register_on_EEPROM(BYTE uchCPU, event_register_t Event_Reg);
*                    BOOL Is_CPU1_EEPROM_Record_Valid(void);
*                    BOOL Is_CPU2_EEPROM_Record_Valid(void);
*
*			
*********************************************************************************************************************/
#include "COMMON.h"
#include "EEPROM.h"
#include "CRC16.h"

eeprom_sch_info_t EEPROM_Sch_Info;				/* Structure holds eeprom scheduler information */

/*********************************************************************************
*File name 			:eeprom.c
*Function Name		:void Initialise_EEPROM_State(void)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Initialise of EEPROM states and Scheduler.
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Initialise_EEPROM_State(void)
{
	BYTE uchByte=0;
	UINT16 uiCrc16=0;

	EEPROM_Sch_Info.State = EEPROM_SCHEDULER_NOT_STARTED; /* Set eeprom scheduler  to EEPROM_SCHEDULER_NOT_STARTED */
	EEPROM_Sch_Info.Timeout_ms = 0;						  /* Set eeprom scheduler timeout to zero */
	EEPROM_Sch_Info.Save_CPU1_Register = SET_LOW;		  /* Set eeprom scheduler info Save_CPU1_Register to low */
	EEPROM_Sch_Info.Save_CPU2_Register = SET_LOW;		  /* Set eeprom scheduler info Save_CPU2_Register to low */
	EEPROM_Sch_Info.CPU1_Data_Good = SET_LOW;			  /* Set eeprom scheduler info CPU1_Data_Good to low */
	EEPROM_Sch_Info.CPU2_Data_Good = SET_LOW;			  /* Set eeprom scheduler info CPU2_Data_Good to low */

	EEPROM_Sch_Info.Address = CPU1_DATA_SAVE_ADDRESS;	  /* Set eeprom scheduler info address to  CPU1_DATA_SAVE_ADDRESS */						
	for (uchByte = 0; uchByte < (NO_OF_EVENT_REGISTERS + 2); uchByte++)
	{
		EEPROM_Sch_Info.Address = EEPROM_Sch_Info.Address + 1;
	}

	EEPROM_Sch_Info.Address = CPU2_DATA_SAVE_ADDRESS;	  /* Set eeprom scheduler info address to  CPU2_DATA_SAVE_ADDRESS */
	for (uchByte = 0; uchByte < (NO_OF_EVENT_REGISTERS + 2); uchByte++)
	{
		EEPROM_Sch_Info.Address = EEPROM_Sch_Info.Address + 1;
	}
	EEPROM_Sch_Info.Address = EEPROM_PARKING_ADDRESS;        

	uiCrc16 = Crc16(EEPROM_CPU1, (NO_OF_EVENT_REGISTERS + 2));  /* Validate Cpu1 Event Registers read from EEPROM */
	if (uiCrc16 == 0)
	{
		EEPROM_Sch_Info.CPU1_Data_Good = SET_HIGH;  /*Cpu1 data is Valid */
	}
	uiCrc16 = Crc16(EEPROM_CPU2, (NO_OF_EVENT_REGISTERS + 2));  /* Validate Cpu2 Event Registers read from EEPROM*/	
	if (uiCrc16 == 0)  
	{
		EEPROM_Sch_Info.CPU2_Data_Good = SET_HIGH;	/*Cpu2 data is Valid */
	}
}

/*********************************************************************************
*File name 			:eeprom.c
*Function Name		:void Start_EEPROM_Sch(void)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:This function starts EEPROM  scheduler.
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Start_EEPROM_Sch(void)
{
	EEPROM_Sch_Info.State = EEPROM_SCHEDULER_IDLE;	/* Set eeprom scheduler info to idle state */	
	EEPROM_Sch_Info.Timeout_ms = 0;				    /* Set eeprom scheduler info timeout to zero */
}

/*********************************************************************************
*File name 			:eeprom.c
*Function Name		:void Update_EEPROM_State(void)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:This function updates EEPROM scheduler
*Algorithm			:1.Initially eeprom scheduler in idle state.
*					 2.Check Save_CPU1_Register Flag. if it  is triggered,change Scheduler state to 
*					   "EEPROM_WRITE_CPU1_DATA"
*						Check Save_CPU2_Register Flag.If it  is triggered,change Scheduler state to 
*					   "EEPROM_WRITE_CPU2_DATA"			   
*					 3.In "EEPROM_WRITE_CPU1_DATA" state , Cpu1 data is Written into EEPROM
*						Byte by Byte with 5ms interval between Bytes.Once all bytes are written
*						into EEPROM, Scheduler goes to Idle State.  
*					 4.In "EEPROM_WRITE_CPU2_DATA" state , Cpu2 data is Written into EEPROM
*						Byte by Byte with 5ms interval between Bytes.Once all bytes are written
*						into EEPROM, Scheduler goes to Idle State.  
*Description			: 
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Update_EEPROM_State(void)
{
	switch (EEPROM_Sch_Info.State)
	{
		case EEPROM_SCHEDULER_NOT_STARTED:
			break;
		case EEPROM_SCHEDULER_IDLE:
			if (EEPROM_Sch_Info.Save_CPU1_Register == SET_HIGH)
			{
				EEPROM_Sch_Info.State = EEPROM_WRITE_CPU1_DATA;
				EEPROM_Sch_Info.Timeout_ms = 0;
				EEPROM_Sch_Info.Byte_Count = 0;
				EEPROM_Sch_Info.Address = CPU1_DATA_SAVE_ADDRESS;
				break;
			}
			if (EEPROM_Sch_Info.Save_CPU2_Register == SET_HIGH)
			{
				EEPROM_Sch_Info.State = EEPROM_WRITE_CPU2_DATA;
				EEPROM_Sch_Info.Timeout_ms = 0;
				EEPROM_Sch_Info.Byte_Count = 0;
				EEPROM_Sch_Info.Address = CPU2_DATA_SAVE_ADDRESS;
			}
			break;
		case EEPROM_WRITE_CPU1_DATA:
			if (EEPROM_Sch_Info.Byte_Count >= (NO_OF_EVENT_REGISTERS+2))
			{
				EEPROM_Sch_Info.Save_CPU1_Register = SET_LOW;
				EEPROM_Sch_Info.State = EEPROM_SCHEDULER_IDLE;
				EEPROM_Sch_Info.Timeout_ms = 0;
				EEPROM_Sch_Info.Byte_Count = 0;
				EEPROM_Sch_Info.Address = EEPROM_PARKING_ADDRESS;
				break;
			}
			EEPROM_Sch_Info.Address = EEPROM_Sch_Info.Address + 1;
			EEPROM_Sch_Info.Byte_Count = EEPROM_Sch_Info.Byte_Count + 1;
			EEPROM_Sch_Info.State = EEPROM_WRITE_CPU1_DATA_WAIT;
			EEPROM_Sch_Info.Timeout_ms = EEPROM_DATA_WRITE_WAIT;
			break;
		case EEPROM_WRITE_CPU1_DATA_WAIT:
			if (EEPROM_Sch_Info.Timeout_ms == TIMEOUT_EVENT)
			{
				EEPROM_Sch_Info.State = EEPROM_WRITE_CPU1_DATA;
			}
			break;
		case EEPROM_WRITE_CPU2_DATA:
			if (EEPROM_Sch_Info.Byte_Count >= (NO_OF_EVENT_REGISTERS+2))
			{
				EEPROM_Sch_Info.Save_CPU2_Register = SET_LOW;
				EEPROM_Sch_Info.State = EEPROM_SCHEDULER_IDLE;
				EEPROM_Sch_Info.Timeout_ms = 0;
				EEPROM_Sch_Info.Byte_Count = 0;
				EEPROM_Sch_Info.Address = EEPROM_PARKING_ADDRESS;
				break;
			}
			EEPROM_Sch_Info.Address = EEPROM_Sch_Info.Address + 1;
			EEPROM_Sch_Info.Byte_Count = EEPROM_Sch_Info.Byte_Count + 1;
			EEPROM_Sch_Info.State = EEPROM_WRITE_CPU2_DATA_WAIT;
			EEPROM_Sch_Info.Timeout_ms = EEPROM_DATA_WRITE_WAIT;
			break;
		case EEPROM_WRITE_CPU2_DATA_WAIT:
			if (EEPROM_Sch_Info.Timeout_ms == TIMEOUT_EVENT)
			{
				EEPROM_Sch_Info.State = EEPROM_WRITE_CPU2_DATA;
			}
			break;
        default:
            break;
		}
}

/*********************************************************************************
*File name 			:eeprom.c
*Function Name		:void Decrement_EEPROM_msTmr(void)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:one milliSecond Timer Variables present EEPROM_Sch_Info timeout 
*					  decremented.
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Decrement_EEPROM_msTmr(void)
{
	if (EEPROM_Sch_Info.Timeout_ms > 0)
	{
		EEPROM_Sch_Info.Timeout_ms = EEPROM_Sch_Info.Timeout_ms - 1;
	}
}

/*********************************************************************************
*File name 			:eeprom.c
*Function Name		:void Save_Event_Register_on_EEPROM(BYTE uchCPU, event_register_t Event_Reg)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Function to save the event Record on EEPROM
*Algorithm			:
*Description			: 
*Input Element		:uchCPU -Select CPU1 or CPU2 ID
*					 Event_Reg - Save latest event record into event register buffer 
*Output Element		:void
*
**********************************************************************************/
void Save_Event_Register_on_EEPROM(BYTE uchCPU, event_register_t Event_Reg)
{
	BYTE uchByte=0;
	wordtype_t TempBuf;

	if (uchCPU != CPU1_ID && uchCPU != CPU2_ID)
	{
		return;		/* out of range */
	}
	if (uchCPU == CPU1_ID)
	{
		if (EEPROM_Sch_Info.Save_CPU1_Register == SET_LOW)
		{
			for (uchByte = 0; uchByte < NO_OF_EVENT_REGISTERS; uchByte++)
			{
				/* Copy the data from Event Reg to EEPROM Scheduler Data Register */
				EEPROM_Sch_Info.CPU1_Data[uchByte] = Event_Reg.Byte[uchByte];   
			}
			TempBuf.Word = Crc16(EEPROM_CPU1, NO_OF_EVENT_REGISTERS);
			EEPROM_Sch_Info.CPU1_Data[NO_OF_EVENT_REGISTERS] = TempBuf.Byte.Lo;
			EEPROM_Sch_Info.CPU1_Data[NO_OF_EVENT_REGISTERS + 1] = TempBuf.Byte.Hi;
			EEPROM_Sch_Info.Save_CPU1_Register = SET_HIGH; /* Trigger the Save_Cpu1_Register Flag */ 
		}
	}
	else
	{
		if (EEPROM_Sch_Info.Save_CPU2_Register == SET_LOW)
		{
			for (uchByte = 0; uchByte < NO_OF_EVENT_REGISTERS; uchByte++)
			{
				/* Copy the data from Event Reg to EEPROM Scheduler Data Register */
				EEPROM_Sch_Info.CPU2_Data[uchByte] = Event_Reg.Byte[uchByte];
			}
			TempBuf.Word = Crc16(EEPROM_CPU2, NO_OF_EVENT_REGISTERS);
			EEPROM_Sch_Info.CPU2_Data[NO_OF_EVENT_REGISTERS] = TempBuf.Byte.Lo;
			EEPROM_Sch_Info.CPU2_Data[NO_OF_EVENT_REGISTERS + 1] = TempBuf.Byte.Hi;
			EEPROM_Sch_Info.Save_CPU2_Register = SET_HIGH; /* Trigger the Save_Cpu2_Register Flag */
		}
	}
}
/*********************************************************************************
*File name 			:eeprom.c
*Function Name		:BOOL Is_CPU1_EEPROM_Record_Valid(void)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Returns the Status of CPU1 Record read from EEPROM is Valid or Not. 
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:CPU1_Event_Record Status 
*
**********************************************************************************/
BOOL Is_CPU1_EEPROM_Record_Valid(void)
{
	return (EEPROM_Sch_Info.CPU1_Data_Good);
}
/*********************************************************************************
*File name 			:eeprom.c
*Function Name		:BOOL Is_CPU2_EEPROM_Record_Valid(void)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Returns the Status of CPU2 Record read from EEPROM is Valid or Not. 
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:CPU2_Event_Record Status 
*
**********************************************************************************/
BOOL Is_CPU2_EEPROM_Record_Valid(void)
{
	return (EEPROM_Sch_Info.CPU2_Data_Good);
}
