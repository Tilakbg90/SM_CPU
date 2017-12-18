/*********************************************************************************************************************
	Project		: 	Single Section Digital Axle Counter
	Version		: 	2.0 
	Revision	:	1	
	Filename	: 	drv_lcd.c
	Target MCU	: 	PIC24FJ256GB210   
    Compiler	: 	XC16 Compiler V1.31  
	Author		:	EM003 
	Date		:	
	Company Name: 	Insys Digital Systems.
	Modification History:
					Rev No			Date		Description
					 --				 --				--    
	Functions	:   void Initialise_LCD_Driver(void)
					void Initialise_LCD_Display(void)
					void Update_LCD_State(void)
					void Decrement_LCD_msTmr(void)
					void WriteShiftRegister(BYTE uchLCDdata)
					void Write_LCD_data(BYTE uchData)
					void Write_LCD_command(BYTE uchCommand)
					void Display_on_LCD(BYTE uchRow, BYTE uchColumn, BYTE *String)
			
*********************************************************************************************************************/
#include <xc.h>

#include "COMMON.h"
#include "DRV_LCD.h"

const BYTE uch4094AddressMask[8] = {
0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80 };  /*this table is used address mask all bits,except one bit */

lcd_info_t LCD_Info;		/* structure that handles Lcd scheduler and holds lcd Information */

void Initialise_LCD_Display(void);
void WriteShiftRegister(BYTE);
void Write_LCD_data(BYTE);
void Write_LCD_command(BYTE);
/*********************************************************************************
File name 			:drv_lcd.c
Function Name		:void Initialise_LCD_Driver(void)
Created By			:EM003 
Date Created		:
Modification History:
					Rev No			Date		Description
					 --				 --				--
Tracability:
		SRS()    	:

Abstract			:Initialisation of lcd driver module.
Algorithm			:
Description			: 
Input Element		:None
Output Element		:void

**********************************************************************************/
void Initialise_LCD_Driver(void)
{
	BYTE uchRow, uchColumn;

	CD4094_STROBE = 0;									/* Strobe = LOW */
	CD4094_DATA = 0;									/* Data   = LOW */
	CD4094_CLOCK = 0;									/* Clock  = LOW */
	LCD_Info.State = LCD_MODULE_NOT_PLUGGED_IN;			/* Initialise lcd module info state in not plugged in mode */ 
	LCD_Info.Timeout_ms = 0;							/* Set timeout event is zero */
	LCD_Info.Line = 0;									/* set lcd line no.  is zero */
	LCD_Info.Column = 0;								/* set lcd column is zero */
	for (uchRow = 0; uchRow < LCD_MAX_ROWS; uchRow++)  
	{
		for (uchColumn = 0; uchColumn < LCD_MAX_COLS; uchColumn++)
		{
			/* load the line buffer with default value i.e space ' ' */          
			LCD_Info.Line_Buffer[uchRow][uchColumn] = ' ';   
		}
	}
}
/*********************************************************************************
File name 			:drv_lcd.c
Function Name		:void Initialise_LCD_Display(void)
Created By			:EM003 
Date Created		:
Modification History:
					Rev No			Date		Description
					 --				 --				--
Tracability:
		SRS()    	:

Abstract			:Initialise lcd display module
Algorithm			:1.Turn on the power ,wait for 250 ms after vdd rises to 4.5v.
					 2.Set interface as 8 bits long(function Set).wait for 5ms then go to step3.
					 3.Set interface as 8 bits long(function Set).wait for 5ms then go to step4.
					 4.Set interface as 8 bits long(function Set).
					 6.Set interface as 8 bits,number of display lines and character font(Function Set 4*20)
					 7.Set display ON 
				     8.clear display and give a time delay of 2ms.
					 9.Set cursor move direction and specify display shift(entry mode Set).give a time delay of 2ms.
					 10.Initialisation complete.
Description			: 
					* All data sent to the lcd module which passed through a shiftregister  
					* present in the lcd module.
Input Element		:None
Output Element		:void

**********************************************************************************/
void Initialise_LCD_Display(void)
{
	switch (LCD_Info.Init_State)
	{
            case LCD_INIT_COMPLETE:
                break;
		case LCD_INIT_BEGIN:
			LCD_RS = 0;
			LCD_Info.Init_State = WAIT_FOR_Vdd_TO_STABILISE;
			LCD_Info.Timeout_ms = DELAY_250MS;
			break;
		case WAIT_FOR_Vdd_TO_STABILISE:
			if (LCD_Info.Timeout_ms == TIMEOUT_EVENT)
			{
				LCD_Info.Init_State = LCD_INIT_BYTE1_WRITTEN;   /*Set interface as 8 bits long */
				WriteShiftRegister((BYTE)0x30);
				LCD_STROBE();
				LCD_Info.Timeout_ms = DELAY_5MS;				/*Give a time delay of 5ms*/
			}
			break;
		case LCD_INIT_BYTE1_WRITTEN:
			if (LCD_Info.Timeout_ms == TIMEOUT_EVENT)
			{
				LCD_Info.Init_State = LCD_INIT_BYTE2_WRITTEN;   /*Set interface as 8 bits long */
				WriteShiftRegister((BYTE)0x30);
				LCD_STROBE();
				LCD_Info.Timeout_ms = DELAY_5MS;
			}
			break;
		case LCD_INIT_BYTE2_WRITTEN:
			if (LCD_Info.Timeout_ms == TIMEOUT_EVENT)
			{
				LCD_Info.Init_State = LCD_INIT_FUNCTION_SET;
				WriteShiftRegister((BYTE)0x30);
				LCD_STROBE();
			}
			break;
		case LCD_INIT_FUNCTION_SET:
			WriteShiftRegister((BYTE)0x38);
			LCD_STROBE();
			LCD_Info.Init_State = LCD_INIT_DISPLAY_CONTROL;
			break;
		case LCD_INIT_DISPLAY_CONTROL:
			WriteShiftRegister((BYTE)0x08);
			LCD_STROBE();
			LCD_Info.Init_State = LCD_INIT_CLEAR_DISPLAY;
			break;
		case LCD_INIT_CLEAR_DISPLAY:
			WriteShiftRegister((BYTE)0x1);
			LCD_STROBE();
			LCD_Info.Init_State = LCD_INIT_ENTRY_MODE_SET;
			LCD_Info.Timeout_ms = DELAY_2MS;    					    /*Give a time delay of 2ms*/
			break;
		case LCD_INIT_ENTRY_MODE_SET:
			if (LCD_Info.Timeout_ms == TIMEOUT_EVENT)
			{
				WriteShiftRegister((BYTE)0x6);
				LCD_STROBE();
				LCD_Info.Init_State = LCD_INIT_TURN_DISPLAY_ON;
				LCD_Info.Timeout_ms = DELAY_2MS;                       /*Give a time delay of 2ms*/
			}
			break;
		case LCD_INIT_TURN_DISPLAY_ON:
			if (LCD_Info.Timeout_ms == TIMEOUT_EVENT)
			{
				WriteShiftRegister((BYTE)0x0C);
				LCD_STROBE();
				LCD_Info.Init_State = LCD_INIT_COMPLETE;
			}
			break;
	}
}
/*********************************************************************************
File name 			:drv_lcd.c
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
					 3.After Initialisation of LCD completed,it will go for display the characters
					 4.All lcd info four lines are written character by character into display with
				       a time interval of 2ms for each character.
					 5.After go to step4.
Description			: 
Input Element		:None
Output Element		:void

**********************************************************************************/
void Update_LCD_State(void)
{
	BYTE uchBuf=0;
	
	Initialise_LCD_Display();
	
	switch (LCD_Info.State)
	{
		case LCD_MODULE_NOT_PLUGGED_IN:
			if (LCD_MDL_DETECT_PORT == LCD_MDL_PRESENT)
			{
				//CLRWDT();
				//CLRWDT();
				//CLRWDT();
				if (LCD_MDL_DETECT_PORT == LCD_MDL_PRESENT)
				{
					LCD_Info.Init_State = LCD_INIT_BEGIN;
					LCD_Info.State = INITIALISE_LCD_MODULE;
					LCD_Info.Timeout_ms = 0;
				}
			}
			break;
		case INITIALISE_LCD_MODULE:
			if (LCD_MDL_DETECT_PORT != LCD_MDL_PRESENT)
			{
				LCD_Info.Init_State = LCD_INIT_COMPLETE;
				LCD_Info.State = LCD_MODULE_NOT_PLUGGED_IN;
				LCD_Info.Timeout_ms = 0;
				break;
			}
			if (LCD_Info.Init_State == LCD_INIT_COMPLETE)
			{
				Write_LCD_command((BYTE) BEGINNING_OF_LINE1);
				LCD_Info.Line = 0;
				LCD_Info.Column = 0;
				LCD_Info.State = DISPLAY_CHARACTER_ON_LCD;
				LCD_Info.Timeout_ms = LCD_WRITE_TIMEOUT;
			}
			break;
		case DISPLAY_CHARACTER_ON_LCD:
			if (LCD_MDL_DETECT_PORT != LCD_MDL_PRESENT)
			{
				LCD_Info.State = LCD_MODULE_NOT_PLUGGED_IN;
				LCD_Info.Timeout_ms = 0;
				break;
			}
			if (LCD_Info.Timeout_ms == TIMEOUT_EVENT)
			{
				uchBuf = LCD_Info.Line_Buffer[LCD_Info.Line][LCD_Info.Column];
				LCD_Info.Column = LCD_Info.Column + 1;
				Write_LCD_data(uchBuf);
				if (LCD_Info.Column >= LCD_MAX_COLS)
				{
					LCD_Info.Column = 0;
					switch (LCD_Info.Line)
					{
						case 0:
							LCD_Info.Line = LCD_Info.Line + 1;
							LCD_Info.State = SET_DDRAM_ADDRESS_TO_LINE2;
							break;
						case 1:
							LCD_Info.Line = LCD_Info.Line + 1;
							LCD_Info.State = SET_DDRAM_ADDRESS_TO_LINE3;
							break;
						case 2:
							LCD_Info.Line = LCD_Info.Line + 1;
							LCD_Info.State = SET_DDRAM_ADDRESS_TO_LINE4;
							break;
						case 3:
							LCD_Info.Line = 0;
							LCD_Info.State = SET_DDRAM_ADDRESS_TO_LINE1;
							break;
					}
					LCD_Info.Timeout_ms = 0;
				}
				else
				{
					LCD_Info.Timeout_ms = LCD_WRITE_TIMEOUT;
				}
			}
			break;
		case SET_DDRAM_ADDRESS_TO_LINE1:
			Write_LCD_command((BYTE) BEGINNING_OF_LINE1);
			LCD_Info.State = DISPLAY_CHARACTER_ON_LCD;
			LCD_Info.Timeout_ms = LCD_WRITE_TIMEOUT;
			break;
		case SET_DDRAM_ADDRESS_TO_LINE2:
			Write_LCD_command((BYTE) BEGINNING_OF_LINE2);
			LCD_Info.State = DISPLAY_CHARACTER_ON_LCD;
			LCD_Info.Timeout_ms = LCD_WRITE_TIMEOUT;
			break;
		case SET_DDRAM_ADDRESS_TO_LINE3:
			Write_LCD_command((BYTE) BEGINNING_OF_LINE3);
			LCD_Info.State = DISPLAY_CHARACTER_ON_LCD;
			LCD_Info.Timeout_ms = LCD_WRITE_TIMEOUT;
			break;
		case SET_DDRAM_ADDRESS_TO_LINE4:
			Write_LCD_command((BYTE) BEGINNING_OF_LINE4);
			LCD_Info.State = DISPLAY_CHARACTER_ON_LCD;
			LCD_Info.Timeout_ms = LCD_WRITE_TIMEOUT;
			break;
	}
}
/*********************************************************************************
File name 			:drv_lcd.c
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
void Decrement_LCD_msTmr(void)
{
	if (LCD_Info.Timeout_ms > 0)
	{
		LCD_Info.Timeout_ms = LCD_Info.Timeout_ms - 1;
	}

}
/*********************************************************************************
File name 			:drv_lcd.c
Function Name		:void WriteShiftRegister(BYTE uchLCDdata)
Created By			:EM003 
Date Created		:
Modification History:
					Rev No			Date		Description
					 --				 --				--
Tracability:
		SRS()    	:

Abstract			:Writes the data to the shift register  bit by bit.
Algorithm			:1.Make shift register strobe low to enable the shift register for taking input.
					 2.Make shift register clock low,Read bit position value and shift it out.
					 3.Make shift register clock high.
					 4.Repeat the above 2 steps until all bits are transferred.
					 5.Make shift register strobe high, will latch the data.
					 6.Make shift register clock low,after some delay (100ns) Make shift register strobe low.
Description			: 
Input Element		:uchLCDdata-input data to the shift register
Output Element		:void

**********************************************************************************/
void WriteShiftRegister(BYTE uchLCDdata)
{
	signed char chBitPosition;

	CD4094_STROBE = 0;									 /* Strobe = LOW, data propagates */

	//CLRWDT(); 											 /* Clear Watchdog Timer */

	for (chBitPosition = 7; chBitPosition >= 0; chBitPosition--)
	{
		CD4094_CLOCK = 0; 								 /* Clock = LOW */
		if (uchLCDdata & uch4094AddressMask[(BYTE)chBitPosition])
		{
			CD4094_DATA = 1;							 /* Data   = HIGH */
		}
		else
		{
			CD4094_DATA = 0;							 /* Data   = LOW */
		}
		CD4094_CLOCK = 1; 								 /* Clock = HIGH */
	}

	CD4094_STROBE = 1;									 /* Strobe = HIGH, will Latch the data */
	//CLRWDT();											 /* Clear Watchdog Timer */
	CD4094_CLOCK = 0; 									 /* Clock = LOW */
	asm("NOP");
	CD4094_STROBE = 0;									/* Strobe = LOW */
}
/*********************************************************************************
File name 			:drv_lcd.c
Function Name		:void Write_LCD_data(BYTE uchData)
Created By			:EM003 
Date Created		:
Modification History:
					Rev No			Date		Description
					 --				 --				--
Tracability:
		SRS()    	:

Abstract			:Write one byte of data  to the lcd.
Algorithm			:
Description			: 
Input Element		:uchData-data to be written into LCD
Output Element		:void

**********************************************************************************/
void Write_LCD_data(BYTE uchData)
{
	LCD_RS = 1;						/* Register select-Data register */
	WriteShiftRegister(uchData);
	LCD_STROBE();
}
/*********************************************************************************
File name 			:drv_lcd.c
Function Name		:void Write_LCD_command(BYTE uchCommand)
Created By			:EM003 
Date Created		:
Modification History:
					Rev No			Date		Description
					 --				 --				--
Tracability:
		SRS()    	:

Abstract			:Write a command byte to the lcd.
Algorithm			:
Description			: 
Input Element		:uchCommand- The Command to LCD
Output Element		:void

**********************************************************************************/
void Write_LCD_command(BYTE uchCommand)
{
	LCD_RS = 0;							/* Register select-Instruction register */
	WriteShiftRegister(uchCommand);	
	LCD_STROBE();
	LCD_RS = 1;							/* Register select-Data register */
}

/*********************************************************************************
File name 			:drv_lcd.c
Function Name		:void Display_on_LCD(BYTE uchRow, BYTE uchColumn, BYTE *String)
Created By			:EM003 
Date Created		:
Modification History:
					Rev No			Date		Description
					 --				 --				--
Tracability:
		SRS()    	:

Abstract			:It copies all the characters of passed string argument to the lcd line buffer. 
Algorithm			:
Description			: 
Input Element		:uchRow-line no to be updated.
					 uchColumn - the starting character of the line to be updated.
					 String - it holds characters to be copied.
Output Element		:void

**********************************************************************************/
void Display_on_LCD(BYTE uchRow, BYTE uchColumn, BYTE *String)
{
	BYTE uchBuf=0;
	BYTE *uchDestination;
	
	if (uchRow >=  LCD_MAX_ROWS || uchColumn >= LCD_MAX_COLS)
	{
		return;	/* Out of range */
	}								 

	uchBuf = *String;
	String +=1;	
	uchDestination = &LCD_Info.Line_Buffer[uchRow][uchColumn];
	while (uchBuf != '\0' && uchColumn < LCD_MAX_COLS)
	{
		*uchDestination = uchBuf;
		 uchDestination = uchDestination + 1;
		 uchBuf = *String;
		 String = String + 1;	
		 uchColumn = uchColumn + 1;
	}
}


