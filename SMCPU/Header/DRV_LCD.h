/*********************************************************************************************************************
	Project		: 	Single Section Digital Axle Counter
	Version		: 	2.0 
	Revision	:	1	
	Filename	: 	drv_lcd.h
	Target MCU	: 	PIC24FJ256GB210   
    Compiler	: 	XC16 Compiler V1.31  
	Author		:	EM003 
	Date		:	
	Company Name: 	Insys Digital Systems
	Modification History:
					Rev No			Date		Description
					 --				 --				--    
*********************************************************************************************************************/
#ifndef _LCD_DRV_H_
#define _LCD_DRV_H_
/*
 *      Header file for LCD_DRV.C
 */
 
 /*
 * Port I/O
 */
#define CD4094_CLOCK				LATAbits.LATA0    		/* Clock output pin for the Shift Register CD4094 */
#define CD4094_DATA					LATAbits.LATA0  		/* data port to the Shift Register CD4094 */
#define CD4094_STROBE				LATAbits.LATA0   		/* strobe signal to Shift Register CD4094 */
#define LCD_MDL_DETECT_PORT			LATAbits.LATA0   		/* Input port to detect the LCD module */
#define LCD_RS						LATAbits.LATA0   		/* Set LCD register as instruction register */
#define LCD_EN						LATAbits.LATA0    		/* LCD enable Port */

#define BEGINNING_OF_LINE1			0x80        /* starting address of line1 in DDR RAM from where the data has to written on LCD */
#define BEGINNING_OF_LINE2			0xC0	    /* starting address of line2 in DDR RAM from where the data has to written on LCD */
#define BEGINNING_OF_LINE3			0x94        /* starting address of line3 in DDR RAM from where the data has to written on LCD */
#define BEGINNING_OF_LINE4			0xD4        /* starting address of line4 in DDR RAM from where the data has to written on LCD */

#define LCD_MDL_PRESENT				(0)			/* Logic Zero in LCD_MDL_DETECT_PORT indicates that LCD Module is connected */
#define LCD_MAX_ROWS				(4)   		/* Maximum number of rows can be displayed in LCD*/
#define LCD_MAX_COLS			    (20)  		/* Maximum number of colummns can be displayed in LCD*/
#define LCD_WRITE_TIMEOUT			(2)			/* Time Gap between Writing characters into Lcd */

/* Macro Function that Enables and Disable the LCD Module */
#define LCD_STROBE()	{asm("NOP"); LCD_EN = 1; /*CLRWDT();*/LCD_EN = 0;}

/* LCD Driver States*/
typedef enum
			{
			LCD_MODULE_NOT_PLUGGED_IN =0,   /* Check whether the LCD module is connected or not*/
			INITIALISE_LCD_MODULE,          /* Initialise the LCD module if it is connected*/
			DISPLAY_CHARACTER_ON_LCD,       /* Displaying the characters*/
			SET_DDRAM_ADDRESS_TO_LINE1,     /* Set DDR Address Register to line1 */
			SET_DDRAM_ADDRESS_TO_LINE2,     /* Set DDR Address Register to line2 */
			SET_DDRAM_ADDRESS_TO_LINE3,     /* Set DDR Address Register to line3 */
			SET_DDRAM_ADDRESS_TO_LINE4      /* Set DDR Address Register to line4 */
			}LCD_Driver_State;

/* LCD display initialisation States */
typedef enum
			{
			 LCD_INIT_COMPLETE =0,          /* After LCD initialisation completed Lcd_init_State will come to this state*/
			 LCD_INIT_BEGIN,                /* Starts LCD Initialisation */
			 WAIT_FOR_Vdd_TO_STABILISE,     /* In this Scheduler will 250ms wait for VDD to Stabilise.*/		
			 LCD_INIT_BYTE1_WRITTEN,		/* In this 8 bit Interface Mode is set */
			 LCD_INIT_BYTE2_WRITTEN,		/* In this 8 bit Interface Mode is set */
			 LCD_INIT_FUNCTION_SET,			/* Font,Character Size, No of Lines are set for the LCD */
			 LCD_INIT_DISPLAY_CONTROL,		/* Switch On Display */
			 LCD_INIT_CLEAR_DISPLAY,		/* Clear Display */
			 LCD_INIT_ENTRY_MODE_SET,		/* Set the Entry Mode i.e Curser Movement Right to Left*/
			 LCD_INIT_TURN_DISPLAY_ON       /* Switch On Display */
			}LCD_Init_State;

/* Lcd_info_Scheduler */
typedef struct {
			LCD_Init_State 	    Init_State;	/*LCD display initialisation States */
			LCD_Driver_State	State;		/*LCD display driver States */	
			BYTE	Timeout_ms;				/*One millisec Timer Variable to Lcd info Scheduler */	
			BYTE	Line;					/* Line Number to be updated */
			BYTE	Column;					/* Coloumn Number to be updated */
			BYTE	Line_Buffer[LCD_MAX_ROWS][LCD_MAX_COLS]; /* Buffer to Store character which is to be writtern into Lcd */
} lcd_info_t;


extern void Initialise_LCD_Driver(void);  
extern void Update_LCD_State(void);
extern void Decrement_LCD_msTmr(void);

#endif

