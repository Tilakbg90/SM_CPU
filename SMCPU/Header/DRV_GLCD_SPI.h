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
#ifndef _GLCD_DRV_H_
#define _GLCD_DRV_H_
/*
 *      Header file for GLCD_DRV.C
 */

 /*
 * Port I/O
 */
#include <xc.h>

#define G_SPI_SS					LATDbits.LATD13  		/* data port to the Shift Register CD4094 */
#define G_ACTIVE					PORTEbits.RE5    		/* LCD enable Port */

//#define G_SPI_SS2					LATGbits.LATG13  		/* data port to the Shift Register CD4094 */
//#define G_ACTIVE2					PORTGbits.RG12    		/* LCD enable Port */

#define CPU1_PACKET_LEN             55
#define CPU_DATA_LEN                110
#define MAX_G_PACKET_LEN            144                 /*Number of bytes from SMCPU*/
#define OFFSET_CODE_CHECKSUM    (CPU_DATA_LEN)
#define OFFSET_UNIT_TYPE        (OFFSET_CODE_CHECKSUM+4)
#define OFFSET_SW_V             (OFFSET_UNIT_TYPE + 1)
#define OFFSET_EVENT_COUNT      (OFFSET_SW_V + 1)
#define OFFSET_START_TIME       (OFFSET_EVENT_COUNT + 4)
#define OFFSET_END_TIME         (OFFSET_START_TIME + 4)
#define OFFSET_PRESENT_TIME     (OFFSET_END_TIME+ 4)
#define OFFSET_SMCPU_CRC        (OFFSET_PRESENT_TIME+ 4)
#define OFFSET_UNUSED           (OFFSET_SMCPU_CRC + 4)
#define OFFSET_CRC              (OFFSET_UNUSED + 6)
#define MAX_COMM_TIMEOUT        200                      /*updated for every 250ms*/
#define SS_TIMEOUT              50                     /*updated for every 5ms*/
#define TX_TIMEOUT              20                     /*updated for every 2ms*/
/* LCD Driver States*/
typedef enum
			{
        			GLCD_MODULE_NOT_ACTIVE =0,   /* Check whether the LCD module is connected or not*/
                                GLCD_SS_WAIT,
                                GLCD_WAIT_FOR_DATA,
                                GLCD_INIT_DATA,
                                GLCD_RCV_DATA,
                                GLCD_COMPARE_DATA,
                                GLCD_TX_DATA,
                                GLCD_IDLE,
                                GLCD_BUSY,
                                        GLCD_CM_WAIT,
			}GLCD_Driver_State;

                        typedef enum
                        {
                                GLCD_DATA_CPU1,
                                GLCD_DATA_CPU2,
                                GLCD_DATA_SMCPU
                        }GLCD_Data;
///* LCD display initialisation States */
//typedef enum
//			{
//			 LCD_INIT_COMPLETE =0,          /* After LCD initialisation completed Lcd_init_State will come to this state*/
//			 LCD_INIT_BEGIN,                /* Starts LCD Initialisation */
//			 WAIT_FOR_Vdd_TO_STABILISE,     /* In this Scheduler will 250ms wait for VDD to Stabilise.*/
//			 LCD_INIT_BYTE1_WRITTEN,		/* In this 8 bit Interface Mode is set */
//			 LCD_INIT_BYTE2_WRITTEN,		/* In this 8 bit Interface Mode is set */
//			 LCD_INIT_FUNCTION_SET,			/* Font,Character Size, No of Lines are set for the LCD */
//			 LCD_INIT_DISPLAY_CONTROL,		/* Switch On Display */
//			 LCD_INIT_CLEAR_DISPLAY,		/* Clear Display */
//			 LCD_INIT_ENTRY_MODE_SET,		/* Set the Entry Mode i.e Curser Movement Right to Left*/
//			 LCD_INIT_TURN_DISPLAY_ON       /* Switch On Display */
//			}LCD_Init_State;

/* Lcd_info_Scheduler */
#define     CPU_PACKET_LEN      55

                        //SPK_FB_IN -> RG12 -> G_ACTIVE2
//RLED_FB_IN -> RG13 -> G_SPI_SS2


typedef struct {
	GLCD_Driver_State		State;	         	/*LCD display driver States */
    GLCD_Data               Data_pack;
	BYTE                    GLCD1_On_state;
	BYTE                    GLCD2_On_state;
    unsigned int            Comm_Timeout_ms;				/*One millisec Timer Variable to Lcd info Scheduler */
    BYTE                    Off_Timeout_ms;
    BYTE                    Packet_index;					/* Line Number to be updated */
	BYTE                    Packet_Max_length;					/* Coloumn Number to be updated */
    BYTE                    CPU_Data[2][CPU_PACKET_LEN];
    BYTE                    Message_Buffer[MAX_G_PACKET_LEN]; /* Buffer to Store character which is to be writtern into Lcd */
    BYTE                    Rx_Message_Buffer[MAX_G_PACKET_LEN]; /* Buffer to Store character which is to be writtern into Lcd */
} glcd_info_t;



#define SPI_MASTER  0x0060	// select 8-bit master mode, CKE=1, CKP=0
#define SPI_ENABLE  0x8000	// enable SPI port, clear status

extern void Initialise_GLCD_Driver(void);
extern void Update_GLCD_State(void);
extern void Decrement_GLCD_msTmr(void);
void Update_SMPU_data(void);

#endif

