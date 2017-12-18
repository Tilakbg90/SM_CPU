/*********************************************************************************************************************
	Project		: 	Single Section Digital Axle Counter
	Version		: 	2.0 
	Revision	:	1	
	Filename	: 	display.c
	Target MCU	: 	PIC24FJ256GB210   
    Compiler	: 	XC16 Compiler V1.31  
	Author		:	EM003 
	Date		:	
	Company Name: 	Insys Digital Systems
	Modification History:
					Rev No			Date		Description
					 --				 --				--    
	Functions	:   void Initialise_Display_State(void)
					void Update_itoa_Display_State(void)
					void Decrement_itoa_msTmr(void)
			
*********************************************************************************************************************/
#include <xc.h>

#include "COMMON.h"
#include "DISPLAY.h"
#include "DRV_LCD.h"


extern dac_comm_error Dac_Comm_Err;    /* From cpu_sm.c */
/* Variable Holds information to display  Last Line of Lcd Display */
BYTE uchErrorCount_Line [21] = {" 0000 0000 0000 0000" };  
itoa_info_t ItoA_Info; /* Display update Scheduler */

/*********************************************************************************
File name 			:display.c
Function Name		:void Initialise_Display_State(void)
Created By			:EM003 
Date Created		:
Modification History:
					Rev No			Date		Description
					 --				 --				--
Tracability:
		SRS()    	:

Abstract			:Intialise the display state.
Algorithm			:
Description			: 
Input Element		:None
Output Element		:void

**********************************************************************************/
void Initialise_Display_State(void)
{
	ItoA_Info.State = CPU1_COMMA_ERROR_COUNT;				/* Default State */
	ItoA_Info.Timeout_ms = (BYTE) ITOA_SCHEDULER_SCAN_RATE;  
}	
/*********************************************************************************
File name 			:display.c
Function Name		:void Update_itoa_Display_State(void)
Created By			:EM003 
Date Created		:
Modification History:
					Rev No			Date		Description
					 --				 --				--
Tracability:
		SRS()    	:

Abstract			:It updates the fourth line of LCD display with comm error counts
Algorithm			:
					1.It will update all of the following counts with regular time Interval.
						* Cpu1 Communication Modem A error Count
						* Cpu2 Communication Modem A error Count
						* Cpu1 Communication Modem B error Count
						* Cpu2 Communication Modem B error Count
					2.It calls display function for displaying fourth Line of LCD and repeat the
					  above two steps.  
					 
Description			: 
Input Element		:None
Output Element		:void

**********************************************************************************/
void Update_itoa_Display_State(void)
{
	BYTE *puchString;

	switch (ItoA_Info.State)
	{
		case CPU1_COMMA_ERROR_COUNT:
			if (ItoA_Info.Timeout_ms == TIMEOUT_EVENT)
			{				
				puchString = &uchErrorCount_Line[1];
				Itoac(Dac_Comm_Err.CPU1_CommA_Error_Count,puchString);				
				ItoA_Info.State = (BYTE) CPU2_COMMA_ERROR_COUNT;
				ItoA_Info.Timeout_ms = (BYTE) ITOA_SCHEDULER_SCAN_RATE;
			}
			break;
		case CPU2_COMMA_ERROR_COUNT:
			if (ItoA_Info.Timeout_ms == TIMEOUT_EVENT)
			{
				puchString = &uchErrorCount_Line[6];
				Itoac(Dac_Comm_Err.CPU2_CommA_Error_Count, puchString);
				ItoA_Info.State = (BYTE) CPU1_COMMB_ERROR_COUNT;
				ItoA_Info.Timeout_ms = (BYTE) ITOA_SCHEDULER_SCAN_RATE;
			}
			break;		
		case CPU1_COMMB_ERROR_COUNT:
			if (ItoA_Info.Timeout_ms == TIMEOUT_EVENT)
			{
				puchString = &uchErrorCount_Line[11];
				Itoac(Dac_Comm_Err.CPU1_CommB_Error_Count,puchString);
				ItoA_Info.State = (BYTE) CPU2_COMMB_ERROR_COUNT;
				ItoA_Info.Timeout_ms = (BYTE) ITOA_SCHEDULER_SCAN_RATE;
			}
			break;
		case CPU2_COMMB_ERROR_COUNT:
			if (ItoA_Info.Timeout_ms == TIMEOUT_EVENT)
			{
				puchString = &uchErrorCount_Line[16];
			    Itoac(Dac_Comm_Err.CPU2_CommB_Error_Count, puchString);
				ItoA_Info.State = (BYTE) UPDATE_ERROR_COUNT_DISPLAY;
				ItoA_Info.Timeout_ms = (BYTE) ITOA_SCHEDULER_SCAN_RATE;
			}
			break;
		case UPDATE_ERROR_COUNT_DISPLAY:
			ItoA_Info.State = (BYTE) CPU1_COMMA_ERROR_COUNT;
			ItoA_Info.Timeout_ms = (BYTE) ITOA_SCHEDULER_SCAN_RATE;
			break;						
        default:
            break;
	}
}
/*********************************************************************************
File name 			:display.c
Function Name		:void Decrement_itoa_msTmr(void)
Created By			:EM003 
Date Created		:
Modification History:
					Rev No			Date		Description
					 --				 --				--
Tracability:
		SRS()    	:

Abstract			:It decrements One millisecond variable present in itoa Scheduler.
Algorithm			:
Description			: 
Input Element		:None
Output Element		:void

**********************************************************************************/

void Decrement_itoa_msTmr(void)
{
	if (ItoA_Info.Timeout_ms > 0)	
		{
		ItoA_Info.Timeout_ms = ItoA_Info.Timeout_ms - 1;
		}
}


