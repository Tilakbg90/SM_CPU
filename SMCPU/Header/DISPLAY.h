/*********************************************************************************************************************
	Project		: 	Single Section Digital Axle Counter
	Version		: 	2.0 
	Revision	:	1	
	Filename	: 	display.h
	Target MCU	: 	PIC24FJ256GB210   
    Compiler	: 	XC16 Compiler V1.31  
	Author		:	EM003 
	Date		:	
	Company Name: 	Insys Digital Systems
	Modification History:
					Rev No			Date		Description
					 --				 --				--    
*********************************************************************************************************************/
#ifndef _DISPLAY_H_
#define _DISPLAY_H_

/* ITOA Process time-outs in 1 milli-seconds */
#define ITOA_SCHEDULER_SCAN_RATE			(10)		/* Wait Time for Scheduler in Each State */

/* ITOA Scheduler States */
typedef enum
			{
		    CPU1_COMMA_ERROR_COUNT=0,		/*Updates "CPU1 communication A Modem Error count" in Display */		
		    CPU2_COMMA_ERROR_COUNT,			/*Updates "CPU2 communication A Modem Error count" in Display */		
		    CPU1_COMMB_ERROR_COUNT,			/*Updates "CPU1 communication B Modem Error count" in Display */		
		    CPU2_COMMB_ERROR_COUNT,         /*Updates "CPU2 communication B Modem Error count" in Display */		
			UPDATE_ERROR_COUNT_DISPLAY		/*Updates the fouth line of LCD display and then goto State 0 */
			}itoa_states;

/* ITOA Scheduler for Displaying the Communication ERRor Counts of Both Cpu's on LCD Display */
typedef struct {
            	itoa_states State ;  		  /* ITOA Scheduler State */	    		
  			    BYTE Timeout_ms;		      /* One millisec variable of ITOA Scheduler Timing */
} itoa_info_t;

extern void Initialise_Display_State(void);   
extern void Update_itoa_Display_State(void);  
extern void Decrement_itoa_msTmr(void);       
#endif
