/*********************************************************************************************************************
*	Project		: 	Single Section Digital Axle Counter
*	Version		: 	2.0 
*	Revision	:	1	
*	Filename	: 	drv_mem.c
*	Target MCU	: 	PIC24FJ256GB210   
*    Compiler	: 	XC16 Compiler V1.31  
*	Author		:	EM003
*	Date		:	
*	Company Name: 	Insys Digital Systems
*	Modification History:
*					Rev No			Date		Description
*					 --				 --				--    
*	Functions	:   
*                    BOOL Erase_Serial_EEProm_Block(UINT16 uiBlockNo);
*                    BOOL Write_Event_on_Serial_EEProm(UINT16 uiRecordNo);
*                    BOOL Read_Event_from_Serial_EEProm(UINT16 uiRecordNo);
*                    BOOL Is_Serial_EEPROM_Write_Complete(void);
*                    BOOL Save_Phone_Number(BYTE num);
*                    BOOL Read_Phone_Number(void);
*                    BOOL Save_Event_Ref(void);
*                    BOOL Read_Event_ref(void);
*		
*********************************************************************************************************************/
#include <xc.h>
#include <time.h>

#include "COMMON.h"
#include "DRV_I2C.h"
#include "DRV_MEM.h"
#include "EVENTS.h"
#include "command_proc.h"

extern sm_status_t Status;	/* from cpu_sm.c */
extern char Phone_number[6][15];
extern BOOL E_status;
/* Addressing Table To Select Particular serial EEPROM Chip */
const ser_eeprom_addr_mapping_t Serial_EEPROM_Addr_Map[NO_OF_SERIAL_EEPROMS] = {
		/* CHIP No:0               CHIP No:1            */
		{0xA1,0xA0}, {0xA9,0xA8}, {0xA5,0xA4}, {0xAD,0xAC}
};

/* In EEProm Chip Address First one is Read Address and Second one is Write Address.
i.e 0xA1 - Read Add, 0xA0 -Write Address */ 

BYTE Previous_Write_Addr;  /* it holds the Address of last Record  Written into serial EEPROM */
Location_Ref_t Mem_addr_ref;
event_record_t Event_Record_R;
event_record_t Event_Record_W;
UINT16 Page;
wordtype_t Erase_address;
EEPROM_Erase_State Block_erase_status;
BYTE temp_E = 0;


/*********************************************************************************
*File name 			:drv_mem.c
*Function Name		:BOOL Erase_Serial_EEProm_Block(UINT16 uiBlockNo)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			: This Funtion erase the Data of one Block. The Default value Zero is Written to 
*					  all Locations of that Block.	
*Algorithm			:
*					 * No of Bytes per Block is 64. 
*					 * No of Blocks Per CHIP is 512.
*					 * Total No of Blocks to be Erased is 512 * 7 = 3584
*					
*					1.Find The EEPROM Chip No in which Block to be erased.
*					   EEPROM chip No = Block_No to be Erased / No of Blocks Per CHIP;
*					2.Find the Offset address in selected EEPROM CHIP.
*					   offset address = ((Block_No % Blocks Per CHIP) * EEPROM Block Size) 		
*					3.Generate "START Condition" and select Particular EEPROM Chip.
*					4.Write the starting address of the Block into EEPROM Address Register.
*					5.Write zero in all locations of that Block 	
*					6.Generate "STOP condition" and return the Status of Erase function.		
*
*
*Description			: 
*Input Element		: uiBlockNo- Block No to be Erased
*Output Element		: Status of Erase Function.
*
**********************************************************************************/
BOOL Erase_Serial_EEProm_Block(UINT16 uiBlockNo)
{
    LATDbits.LATD12 = 0;
	long int uchOffset=0;
	wordtype_t	Address;
    UINT16  PG_count;
	BYTE uchChipWrAdr=0;

	if (uiBlockNo > (MAXIMUM_BLOCKS+1))
	{
		return (FALSE);				/* Range check failed */
	}

	uchOffset = (uiBlockNo / BLOCKS_PER_CHIP);
    if(Block_erase_status == START)
    {
    	Address.Word = ((uiBlockNo % BLOCKS_PER_CHIP) * EEPROM_BLOCK_SIZE);
        Block_erase_status = IN_PROGESS;
        Page = 0;
    }
    else
    {
        Address.Word = Erase_address.Word;
    }

    uchChipWrAdr = Serial_EEPROM_Addr_Map[uchOffset].Physical_Write_Addr;/*lint -e661 */
    for(PG_count = 0;PG_count<0x1;PG_count++)
    {
        if (Generate_I2C_Start() == FALSE)
        {
            I2C2STATbits.IWCOL = SET_LOW;			/* Clear Write collision bit */
            I2C2CONbits.RCEN = SET_LOW;			/* Put MSSP in Transmit mode */
            E_status = Generate_I2C_Stop();		/* Free the I2C BUS */
            Status.Flags.MEM_I2C_Failed = TRUE;
            return (FALSE);				/* Communication failed */
        }
        Status.Flags.MEM_I2C_Failed = FALSE;
        if (Put_I2C_Byte(uchChipWrAdr) == FALSE)	/* Select the Particular EEPROM CHIP */
        {
            I2C2STATbits.IWCOL = SET_LOW;			/* Clear Write collision bit */
            I2C2CONbits.RCEN = SET_LOW;			/* Put MSSP in Transmit mode */
            Status.Flags.MEM_I2C_Failed = TRUE;
            E_status = Generate_I2C_Stop();					/* Free the I2C BUS */
            return (FALSE);							/* Device Busy */
        }
        Nop ();
        Nop ();

        E_status = Put_I2C_Byte (Address.Byte.Hi);			   /* Write Starting Address of the Block */
        E_status = Put_I2C_Byte (Address.Byte.Lo);
        MEM_WRITE_PROTECT = FALSE;				   /* Enable Write operation */
        for (uchOffset = 0; uchOffset < 0x10; uchOffset+=1)
        {
            if(Put_I2C_Byte(0xFF)== FALSE)
            {
                I2C2STATbits.IWCOL = SET_LOW;			/* Clear Write collision bit */
                I2C2CONbits.RCEN = SET_LOW;			/* Put MSSP in Transmit mode */
                Status.Flags.MEM_I2C_Failed = TRUE;
                E_status = Generate_I2C_Stop();					/* Free the I2C BUS */
                return (FALSE);							/* Device Busy */
            }					  /* Write Zero In all locations */ 	
        }
        E_status = Generate_I2C_Stop();
        Address.Word = Address.Word + 0x10;

        temp_E++;
    }
    Erase_address.Word = Address.Word;
    if(temp_E == 4)
    {
        Page = Page + 0x01;
        temp_E = 0;
    }
    if(Page>=0x400)
    {
        Block_erase_status = START;
    }
	MEM_WRITE_PROTECT = TRUE;			     /* Prevent accidental write */
	Previous_Write_Addr = uchChipWrAdr;
	return (TRUE);						     /* memory block succesfully erased */
}

/*********************************************************************************
*File name 			:drv_mem.c
*Function Name		:BOOL Write_Event_on_Serial_EEProm(UINT16 uiRecordNo)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:This Funtion writes One Record of Information into Serial EEPROM.	
*Algorithm			:
*					 * No of Bytes per Event Record is 16.
*					 * No of Events per CHIP is  2048 
*					
*					1.Find The EEPROM Chip No in which Record is to be written.
*					   EEPROM chip No = Record No / No of Events per CHIP;
*					2.Find the Offset address in selected EEPROM CHIP.
*					   offset address = ((Record No % No of Events per CHIP) * No of Bytes per Record)
*					3.Generate "START Condition" and select Particular EEPROM Chip.
*					4.Write the starting address of the Record into EEPROM Address Register.
*					5.Write All data of that Record  	
*					6.Generate "STOP condition" and return the Status of Write_Event function.		
*
*Description			: 
*Input Element		:RecordNo - indcates the Location where the new record has to be written
*					 Event_Info - Structure holds the data to be written.
*
*Output Element		:Status of Write_Event function.		
*
**********************************************************************************/
BOOL Write_Event_on_Serial_EEProm(UINT16 uiRecordNo)
{
	BYTE uchOffset=0;
	wordtype_t	Address;
	BYTE uchChipWrAdr=0;

	if (uiRecordNo >= MAXIMUM_EVENTS)
	{
		return (FALSE);				/* Range check failed */
	}
	uchOffset = (BYTE)(uiRecordNo / EVENTS_PER_CHIP);
	Address.Word = ((uiRecordNo % EVENTS_PER_CHIP) * EVENT_RECORD_SIZE);
	uchChipWrAdr = Serial_EEPROM_Addr_Map[uchOffset].Physical_Write_Addr;

	if (Generate_I2C_Start() == FALSE)
	{
        I2C2STATbits.IWCOL = SET_LOW;			/* Clear Write collision bit */
		I2C2CONbits.RCEN = SET_LOW;			/* Put MSSP in Transmit mode */
		E_status = Generate_I2C_Stop();		/* Free the I2C BUS */
		Status.Flags.MEM_I2C_Failed = TRUE;
		return (FALSE);				/* Communication failed */
	}
	Status.Flags.MEM_I2C_Failed = FALSE;
	if (Put_I2C_Byte(uchChipWrAdr) == FALSE)    /* Select the Particular EEPROM CHIP */
	{
		E_status = Generate_I2C_Stop();	    /* Free the I2C BUS */
		return (FALSE);				/* Device Busy */
	}
	Nop();
	Nop();
	E_status = Put_I2C_Byte(Address.Byte.Hi);	 /* Write Starting Address of that Record */	
	E_status = Put_I2C_Byte(Address.Byte.Lo);
	MEM_WRITE_PROTECT = FALSE;		/* Enable Write operation */
	for (uchOffset = 0; uchOffset < EVENT_RECORD_SIZE; uchOffset+=1)
	{
		E_status = Put_I2C_Byte(Event_Record_W.Byte[uchOffset]);	/* Write all bytes of that Record */ 	
	}
	E_status = Generate_I2C_Stop();			 /* Free the I2C BUS */
	MEM_WRITE_PROTECT = TRUE;		 /* Prevent accidental write */
	Previous_Write_Addr = uchChipWrAdr;
	return (TRUE);					/* Event info succesfully read */
}
/*********************************************************************************
*File name 			:drv_mem.c
*Function Name		:BOOL Read_Event_from_Serial_EEProm(UINT16 uiRecordNo)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			: This Funtion reads One Record of Information from Serial EEPROM.	
*Algorithm			:
*					 * No of Bytes per Event Record is 16.
*					 * No of Events per CHIP is  2048 
*					
*					1.Find The EEPROM Chip No in which Record is to be read.
*					   EEPROM chip No = Record No / No of Events per CHIP;
*					2.Find the Offset address in selected EEPROM CHIP.
*					   offset address = ((Record No % No of Events per CHIP) * No of Bytes per Record)
*					3.Generate "START Condition" and select Particular EEPROM Chip.
*					4.Write the starting address of the Record into EEPROM Address Register.
*					5.Read All data of that Record  and Store in the Receive Buffer.	
*					6.Generate "STOP condition" 
*					7.Calculate LRC For Received Data and compare with Stored LRC Value for data Validity. 
*					8.Return the Status of Read_Event function.	
*
*Description			: 
*Input Element		:RecordNo - indcates the Location where the new record has to be written
*					 Event_Info - Structure holds the data to be read. i.e Event Receive Buffer.
*Output Element		:Status of Read_Event function.		
*
*
**********************************************************************************/
BOOL Read_Event_from_Serial_EEProm(UINT16 uiRecordNo)
{
	BYTE uchBuf=0;
	BYTE uchOffset=0;
	wordtype_t	Address;
	BYTE uchChipWrAdr=0;
	BYTE uchChipRdAdr=0;

	if (uiRecordNo >= MAXIMUM_EVENTS)
	{
	  return (FALSE);					/* Range check failed */
	}

	MEM_WRITE_PROTECT = TRUE;			/* Prevent accidental write */
    uchBuf = (BYTE)(uiRecordNo / EVENTS_PER_CHIP);
	Address.Word = ((uiRecordNo % EVENTS_PER_CHIP) * EVENT_RECORD_SIZE);
	uchChipWrAdr = Serial_EEPROM_Addr_Map[uchBuf].Physical_Write_Addr;
	uchChipRdAdr = Serial_EEPROM_Addr_Map[uchBuf].Physical_Read_Addr;

	if (Generate_I2C_Start() == FALSE)
	{
        I2C2STATbits.IWCOL = SET_LOW;			/* Clear Write collision bit */
		I2C2CONbits.RCEN = SET_LOW;			/* Put MSSP in Transmit mode */

		E_status = Generate_I2C_Stop();		/* Free the I2C BUS */
		Status.Flags.MEM_I2C_Failed = TRUE;
		return (FALSE);				/* Communication failed */
	}
	Status.Flags.MEM_I2C_Failed = FALSE;
	if (Put_I2C_Byte(uchChipWrAdr) == FALSE)
	{
		E_status = Generate_I2C_Stop();	    	/* Free the I2C BUS */
		return (FALSE);					/* Device Busy */
	}
	E_status = Put_I2C_Byte(Address.Byte.Hi);		/* Write Starting Address of that Record */	
	E_status = Put_I2C_Byte(Address.Byte.Lo);
	E_status = Generate_I2C_ReStart();
	E_status = Put_I2C_Byte(uchChipRdAdr);
	uchBuf = (EVENT_RECORD_SIZE - 1);
	for (uchOffset = 0; uchOffset < EVENT_RECORD_SIZE; uchOffset++)
	{
		I2C2CONbits.RCEN =SET_HIGH;					/* Put MSSP in Receive mode */
		Event_Record_R.Byte[uchOffset] =  Get_I2C_Byte(uchBuf--);		/* reads all bytes of that Record */ 	
	}
	E_status = Generate_I2C_Stop();
	if (Event_Record_R.Field.Token == LATEST_EVENT_TOKEN || 
		Event_Record_R.Field.Token == OLD_EVENT_TOKEN)
	{
		uchBuf = Compute_LRC(Event_Record_R,15);
		if (Event_Record_R.Field.LRC != uchBuf)
		{
			Status.Flags.MEM_Corrupted = TRUE ;
		}
		else
		{
			Status.Flags.MEM_Corrupted = FALSE ;			
		}
	}
	else
	{
		Status.Flags.MEM_Corrupted = FALSE;
	}
	return (TRUE);		/* Event info succesfully read */	
}
/*********************************************************************************
*File name 			:drv_mem.c
*Function Name		:BOOL Is_Serial_EEPROM_Write_Complete(void)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			: It will set Serial EEPROM Address Register to Last Written Record Starting Address.
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:Status indicates that Address of  "Last written Record" is set or Not.		
*
**********************************************************************************/
BOOL Is_Serial_EEPROM_Write_Complete(void)
{
	BOOL uchAck=0;

	if (Previous_Write_Addr == 0)
	{
		return (TRUE);
	}
	if (Generate_I2C_Start() == FALSE)
	{
		I2C2STATbits.IWCOL = SET_LOW;			/* Clear Write collision bit */
		I2C2CONbits.RCEN = SET_LOW;			/* Put MSSP in Transmit mode */
		E_status = Generate_I2C_Stop();		/* Unable to generate Start condition */
		return (TRUE);
	}
	uchAck = Put_I2C_Byte(Previous_Write_Addr); /* Write Starting Address of "last written Record" */	
	E_status = Generate_I2C_Stop();						/* Free the I2C BUS */
	return (uchAck);
}

/*********************************************************************************
*File name 			:drv_mem.c
*Function Name		:BOOL Save_Phone_Number(BYTE num)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Save Phone number to be pinged in EEPROM	
*Algorithm			:
*
*Description			: 
*Input Element		:num - Phone number to indicate address location
*
*Output Element		:Status of function.		
*
**********************************************************************************/
BOOL Save_Phone_Number(BYTE num)
{
	BYTE uchOffset=0;
	wordtype_t	Address;
	BYTE uchChipWrAdr=0;

	Address.Word = 0xFFB0 + ((num-1)*0x10);
	uchChipWrAdr = Serial_EEPROM_Addr_Map[3].Physical_Write_Addr;

	if (Generate_I2C_Start() == FALSE)
	{
        I2C2STATbits.IWCOL = SET_LOW;			/* Clear Write collision bit */
		I2C2CONbits.RCEN = SET_LOW;			/* Put MSSP in Transmit mode */
		E_status = Generate_I2C_Stop();		/* Free the I2C BUS */
		Status.Flags.MEM_I2C_Failed = TRUE;
		return (FALSE);				/* Communication failed */
	}
	Status.Flags.MEM_I2C_Failed = FALSE;
	if (Put_I2C_Byte(uchChipWrAdr) == FALSE)    /* Select the Particular EEPROM CHIP */
	{
		E_status = Generate_I2C_Stop();	    /* Free the I2C BUS */
		return (FALSE);				/* Device Busy */
	}
	Nop();
	Nop();
	E_status = Put_I2C_Byte(Address.Byte.Hi);	 /* Write Starting Address of that Record */
	E_status = Put_I2C_Byte(Address.Byte.Lo);
	MEM_WRITE_PROTECT = FALSE;		/* Enable Write operation */
	for (uchOffset = 0; uchOffset < 14; uchOffset+=1)
	{
		E_status = Put_I2C_Byte(Phone_number[num-1][uchOffset]);	/* Write all bytes of that Record */
	}
	E_status = Generate_I2C_Stop();			 /* Free the I2C BUS */
	MEM_WRITE_PROTECT = TRUE;		 /* Prevent accidental write */
	Previous_Write_Addr = uchChipWrAdr;
	return (TRUE);					/* Event info succesfully read */
}
/*********************************************************************************
*File name 			:drv_mem.c
*Function Name		:BOOL Read_Phone_Number(void)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Save Phone number to be pinged in EEPROM	
*Algorithm			:
*
*Description			: 
*Input Element		:void
*
*Output Element		:Status of function.		
*
**********************************************************************************/
BOOL Read_Phone_Number(void)
{
	BYTE uchBuf=0;
	BYTE uchOffset=0;
	wordtype_t	Address;
	BYTE uchChipWrAdr=0;
	BYTE uchChipRdAdr=0;
    BYTE track_ph = 0;

	MEM_WRITE_PROTECT = TRUE;			/* Prevent accidental write */
	Address.Word = 0xFFB0;
    uchChipWrAdr = Serial_EEPROM_Addr_Map[3].Physical_Write_Addr;
    uchChipRdAdr = Serial_EEPROM_Addr_Map[3].Physical_Read_Addr;

    for(track_ph = 0; track_ph <5;track_ph++, Address.Word += 0x10)
    {
        if (Generate_I2C_Start() == FALSE)
        {
                I2C2STATbits.IWCOL = SET_LOW;			/* Clear Write collision bit */
                I2C2CONbits.RCEN = SET_LOW;			/* Put MSSP in Transmit mode */

                E_status = Generate_I2C_Stop();		/* Free the I2C BUS */
                Status.Flags.MEM_I2C_Failed = TRUE;
                return (FALSE);				/* Communication failed */
        }
        Status.Flags.MEM_I2C_Failed = FALSE;
        if (Put_I2C_Byte(uchChipWrAdr) == FALSE)
        {
                E_status = Generate_I2C_Stop();	    	/* Free the I2C BUS */
                return (FALSE);					/* Device Busy */
        }
        E_status = Put_I2C_Byte(Address.Byte.Hi);		/* Write Starting Address of that Record */
        E_status = Put_I2C_Byte(Address.Byte.Lo);
        E_status = Generate_I2C_ReStart();
        E_status = Put_I2C_Byte(uchChipRdAdr);
        uchBuf = (14 - 1);
        for (uchOffset = 0; uchOffset < 14; uchOffset++)
        {
                I2C2CONbits.RCEN =SET_HIGH;					/* Put MSSP in Receive mode */
                Phone_number[track_ph][uchOffset] =  Get_I2C_Byte(uchBuf--);		/* reads all bytes of that Record */
        }
        E_status = Generate_I2C_Stop();
    }
    return (TRUE);		/* Event info succesfully read */
}

/*********************************************************************************
*File name 			:drv_mem.c
*Function Name		:BOOL Save_Event_Ref(void)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Save start/head and end/tail address of events in memory
*Algorithm			:
*
*Description			: 
*Input Element		:void
*
*Output Element		:Status of function.		
*
**********************************************************************************/
BOOL Save_Event_Ref(void)
{
	BYTE uchOffset=0;
	wordtype_t	Address;
	BYTE uchChipWrAdr=0;

	Address.Word = 0xFFA0;
	uchChipWrAdr = Serial_EEPROM_Addr_Map[3].Physical_Write_Addr;

	if (Generate_I2C_Start() == FALSE)
	{
        I2C2STATbits.IWCOL = SET_LOW;			/* Clear Write collision bit */
		I2C2CONbits.RCEN = SET_LOW;			/* Put MSSP in Transmit mode */
		E_status = Generate_I2C_Stop();		/* Free the I2C BUS */
		Status.Flags.MEM_I2C_Failed = TRUE;
		return (FALSE);				/* Communication failed */
	}
	Status.Flags.MEM_I2C_Failed = FALSE;
	if (Put_I2C_Byte(uchChipWrAdr) == FALSE)    /* Select the Particular EEPROM CHIP */
	{
		E_status = Generate_I2C_Stop();	    /* Free the I2C BUS */
		return (FALSE);				/* Device Busy */
	}
	Nop();
	Nop();
	E_status = Put_I2C_Byte(Address.Byte.Hi);	 /* Write Starting Address of that Record */
	E_status = Put_I2C_Byte(Address.Byte.Lo);
	MEM_WRITE_PROTECT = FALSE;		/* Enable Write operation */
	for (uchOffset = 0; uchOffset < 6; uchOffset+=1)
	{
		E_status = Put_I2C_Byte(Mem_addr_ref.Memory_ref_array[uchOffset]);	/* Write all bytes of that Record */
	}
	E_status = Generate_I2C_Stop();			 /* Free the I2C BUS */
	MEM_WRITE_PROTECT = TRUE;		 /* Prevent accidental write */
	Previous_Write_Addr = uchChipWrAdr;
	return (TRUE);					/* Event info succesfully read */
}

/*********************************************************************************
*File name 			:drv_mem.c
*Function Name		:BOOL Read_Event_Ref(void)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Read start/head and end/tail address of events in memory
*Algorithm			:
*
*Description			: 
*Input Element		:void
*
*Output Element		:Status of function.		
*
**********************************************************************************/
BOOL Read_Event_ref(void)
{
	BYTE uchBuf=0;
	BYTE uchOffset=0;
	wordtype_t	Address;
	BYTE uchChipWrAdr=0;
	BYTE uchChipRdAdr=0;

	MEM_WRITE_PROTECT = TRUE;			/* Prevent accidental write */
	Address.Word = 0xFFA0;
    uchChipWrAdr = Serial_EEPROM_Addr_Map[3].Physical_Write_Addr;
    uchChipRdAdr = Serial_EEPROM_Addr_Map[3].Physical_Read_Addr;

    if (Generate_I2C_Start() == FALSE)
    {
        I2C2STATbits.IWCOL = SET_LOW;			/* Clear Write collision bit */
        I2C2CONbits.RCEN = SET_LOW;			/* Put MSSP in Transmit mode */
        E_status = Generate_I2C_Stop();		/* Free the I2C BUS */
        Status.Flags.MEM_I2C_Failed = TRUE;
        return (FALSE);				/* Communication failed */
    }
    Status.Flags.MEM_I2C_Failed = FALSE;
    if (Put_I2C_Byte(uchChipWrAdr) == FALSE)
    {
        E_status = Generate_I2C_Stop();	    	/* Free the I2C BUS */
        return (FALSE);					/* Device Busy */
    }
    E_status = Put_I2C_Byte(Address.Byte.Hi);		/* Write Starting Address of that Record */
    E_status = Put_I2C_Byte(Address.Byte.Lo);
    E_status = Generate_I2C_ReStart();
    E_status = Put_I2C_Byte(uchChipRdAdr);
    uchBuf = 5;
    for (uchOffset = 0; uchOffset < 6; uchOffset++)
    {
            I2C2CONbits.RCEN =SET_HIGH;					/* Put MSSP in Receive mode */
            Mem_addr_ref.Memory_ref_array[uchOffset] =  Get_I2C_Byte(uchBuf--);		/* reads all bytes of that Record */
    }
    E_status = Generate_I2C_Stop();
    return (TRUE);		/* Event info succesfully read */
}
