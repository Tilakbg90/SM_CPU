/*********************************************************************************************************************
*	Project		: 	Single Section Digital Axle Counter
*	Version		: 	2.0 
*	Revision	:	1	
*	Filename	: 	cpu_sm.c
*	Target MCU	: 	PIC24FJ256GB210   
*    Compiler	: 	XC16 Compiler V1.31  
*	Author		:	EM003 
*	Date		:	
*	Company Name: 	Insys Digital Systems
*	Modification History:
*					Rev No			Date		Description
*					 --				 --				--    
*	Functions	:   
*                 void Initialise_System(void);
*                 void Check_Boards(void);
*                 void Update_Event_logger_ID(void);
*                 void Check_High_Availabilty_Config(void);
*                 void Check_Flash(void);
*                 void Update_Feedback_State(void);
*                 BYTE Check_Theft_Detect(void);
*                 BYTE Check_Door_Open(void);
*                 BYTE Check_YLED_State(void);
*                 BYTE Check_RLED_State(void);
*                 BYTE Check_SPK_State(void);
*                 void Start_Sub_Systems(void);
*                 int main(void);
*		
*********************************************************************************************************************/
#include <xc.h>
#include <stdio.h>
#include <time.h>
#include <libpic30.h>

#include "COMMON.h"
#include "CPU_SM.h"
#include "COMM_DAC.h"
#include "comm_host.h"
#include "COMM_SMC.h"
#include "CRC32.h"
#include "DRV_A2D.h"
#include "DRV_I2C.h"
#include "DRV_RTC.h"
#include "EVENTS.h"
#include "EEPROM.h"
#include "DRV_GLCD_SPI.h"
#include "COMM_GSM.h"

extern int main(void);
void Update_USB_Sch_State(void);

void USBDeviceInit(void);

extern glcd_info_t GLCD_Info;		/* structure that handles Lcd scheduler and holds lcd Information */

sm_status_t Status;					/* sm status information */
time_t SystemClock;					/* holds system clock time   */
time_t SystemDate;                  /*System time with HH:MM:SS as 00:00:00 */
dac_comm_error Dac_Comm_Err;		/* Dac external communication CRC error count information */
dac_sysinfo_t DAC_sysinfo;			/* Structure to hold DAC System Information */

const BYTE BitMask_List[8] = { 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80 }; /* this table is used mask all bits,except one bit */
BYTE CPU1_Address;					/* holds cpu1 address */
BYTE CPU2_Address;					/* holds cpu2 address */

BYTE Event_Logger_ID;
BYTE High_Availability, GSM_Enable; 
BYTE YLED_FB_ERROR = 0, RLED_FB_ERROR = 0, SPK_FB_ERROR = 0, THFT_FB_ERROR = 0, DOOR_FB_ERROR = 0, feedback_error = 0, FB_error_ID =0;
BYTE inspect_event_done;
unsigned int init_delay_count = 0;
UINT32 SMCPU_CRC_checksum = 0;
/*********************************************************************************
*File name 			:cpu_sm.c
*Function Name		:void Initialise_System(void)
*Created By			:EM003 
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Initialise all I/O ports,timer configuration,port direction,disable all priority interrupt levels,
*					 disable A/D converter,Disable Capture/Comparator/PWM  Modules,default status  values ,
*					 calls all initialisation function modules.	
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Initialise_System(void)
{
    unsigned int pll_startup_counter = 600;
    /*Set clock frequency*/
    CLKDIVbits.CPDIV = SET_LOW;   // 8MHz input, 32MHz System Clock
    //8MHz crystal, Fosc = 32MHz, Fcy = FOSC/2 = 16Mhz
    CLKDIVbits.PLLEN = SET_HIGH;
    while(pll_startup_counter--);

	/* Clear ports */
	PORTA = CLEAR;
	PORTB = CLEAR;
	PORTC = CLEAR;
	PORTD = CLEAR;
	PORTE = CLEAR;
    PORTF = CLEAR;
    PORTG = CLEAR;
	/* set direction registers */
	TRISA = PORT_A_DIRECTION;
	TRISB = PORT_B_DIRECTION;
	TRISC = PORT_C_DIRECTION;
	TRISD = PORT_D_DIRECTION;
	TRISE = PORT_E_DIRECTION;
	TRISF = PORT_F_DIRECTION;
	TRISG = PORT_G_DIRECTION;

    ANSA   = 0x0400;
    ANSB   = 0x0038;
    ANSC   = DIGITAL_CONFIG;
    ANSD   = DIGITAL_CONFIG;
    ANSE   = DIGITAL_CONFIG;
    ANSF   = DIGITAL_CONFIG;
    ANSG   = DIGITAL_CONFIG;


	AD1CON1 = ADCON1_CONFIG;
	AD1CON2 = ADCON2_CONFIG;
	AD1CON3 = ADCON3_CONFIG;
    AD1CHS  = A2D_CS_DEFAULT;
    ANCFG   = A2D_BG_DEFAULT;
    AD1CSSL = A2D_SC_DEFAULT;
    AD1CSSH = A2D_SC_DEFAULT;

    CVRCON = CLEAR;
	CM1CON = CLEAR;
    CM2CON = CLEAR;
    CM3CON = CLEAR;
    /* Turn Comparators Off */

    IC1CON1 = CLEAR;
    IC1CON2 = CLEAR;
    IC2CON1 = CLEAR;
    IC2CON2 = CLEAR;
    IC3CON1 = CLEAR;
    IC3CON2 = CLEAR;
    IC4CON1 = CLEAR;
    IC4CON2 = CLEAR;
    IC5CON1 = CLEAR;
    IC5CON2 = CLEAR;
    IC6CON1 = CLEAR;
    IC6CON2 = CLEAR;
    IC7CON1 = CLEAR;
    IC7CON2 = CLEAR;
    IC8CON1 = CLEAR;
    IC8CON2 = CLEAR;
    IC9CON1 = CLEAR;
    IC9CON2 = CLEAR;

    OC1CON1 = CLEAR;
    OC1CON2 = CLEAR;
    OC2CON1 = CLEAR;
    OC2CON2 = CLEAR;
    OC3CON1 = CLEAR;
    OC3CON2 = CLEAR;
    OC4CON1 = CLEAR;
    OC4CON2 = CLEAR;
    OC5CON1 = CLEAR;
    OC5CON2 = CLEAR;
    OC6CON1 = CLEAR;
    OC6CON2 = CLEAR;
    OC7CON1 = CLEAR;
    OC7CON2 = CLEAR;
    OC8CON1 = CLEAR;
    OC8CON2 = CLEAR;
    OC9CON1 = CLEAR;
    OC9CON2 = CLEAR;
	/* Disable Capture/Comparator/PWM  Modules */


	/* Load Default PORT Values */
	LATA = PORT_A_DEFAULT;
	LATB = PORT_B_DEFAULT;
	LATC = PORT_C_DEFAULT;
	LATD = PORT_D_DEFAULT;
	LATE = PORT_E_DEFAULT;
	LATF = PORT_F_DEFAULT;
	LATG = PORT_G_DEFAULT;

	/* Configure Timers, CCP Module */
	T1CON = TIMER1_CONFIG;
	T2CON = TIMER2_CONFIG;
    PR1   = PR1_SET;
    PR2   = PR2_SET;
    
	TMR1 = TIMER1_SETPOINT;
	TMR2 = TIMER2_SETPOINT;

	
    INTCON1 = CLEAR;
    INTCON2 = CLEAR;

    IEC0 = CLEAR;
    IEC1 = CLEAR;
    IEC2 = CLEAR;
    IEC3 = CLEAR;
    IEC4 = CLEAR;
    IEC5 = CLEAR;

	Status.Byte[0] = (BYTE) 0b11011111;			/* Initialise Status Register */
	Status.Byte[1] = (BYTE) 0b01001000;			/* Initialise Status Register */

	Check_Boards();			    				/* Check whether CPU1 and CPU2 boards are present in rack */
	Initialise_Smc_CommSch();	/* from comm_smc.c */
    Check_Flash();			    				/* Check for corruption of On-Board FLASH */
    if(Status.Flags.Flash_CheckSum == CRC32_CHECKSUM_BAD)
    {
        LATDbits.LATD12 = SET_HIGH;
        LATDbits.LATD2  = SET_LOW;        
        //while(1);
    }
    else
    {
        LATDbits.LATD12 = SET_LOW;
        LATDbits.LATD2  = SET_HIGH;        
    }

	/*
	 * Initialise sub-systems
	 *
	 * Please ensure that Initialise_EEPROM_State() 
	 * is called before calling following functions
	 * - Initialse_SPI()
	 * - Initialise_Events_Sch() 
	 */
	Initialise_A2D_Driver();	/* from drv_a2d.c */
	Initialise_I2C_Driver();	/* from drv_i2c.c */
	Initialise_RTC_Sch();		/* from drv_rtc.c */
	Initialise_Host_CommSch();	/* from comm_host.c */	
	Initialise_EEPROM_State();	/* from eeprom.c */
	Initialise_SPI();			/* from comm_dac.c */
	Initialise_Events_Sch();	/* from events.c */
    Initialise_GSM_CommSch();
	USBDeviceInit();	//usb_device.c.  Initializes USB module SFRs and firmware variables to known states.//Initialise_Display_State(); /* from display.c */
    Initialise_GLCD_Driver();
    
	//Read_RTC_Registers();
	Dac_Comm_Err.CPU1_CommB_Error_Count = CLEAR;     /*Initialise all Communication Err Counts to zero */
	Dac_Comm_Err.CPU1_CommA_Error_Count = CLEAR;
	Dac_Comm_Err.CPU2_CommB_Error_Count = CLEAR;
	Dac_Comm_Err.CPU2_CommA_Error_Count = CLEAR;	

    TRISDbits.TRISD12 = SET_LOW;
    LATDbits.LATD12 = SET_HIGH;
    
    //FB lines
    TRISGbits.TRISG0  = SET_HIGH;
    TRISGbits.TRISG1  = SET_HIGH;
    TRISGbits.TRISG12 = SET_HIGH;
    TRISGbits.TRISG13 = SET_HIGH;
    TRISGbits.TRISG14 = SET_HIGH;
        
    Update_Event_logger_ID();
    Check_High_Availabilty_Config();
}

/*********************************************************************************
*File name 			:cpu_sm.c
*Function Name		:void Check_Boards(void)
*Created By			:EM003 
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Check whether CPU1 and CPU2 boards are present in rack
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Check_Boards(void)
{
	if (CPU_DETECT_PORT == BOARD_MISSING)
	{
		Status.Flags.B1_Board_Status = CPU_DETECT_PORT;
		Status.Flags.System_Status = CATASTROPHIC_ERROR;
	}
}
/*********************************************************************************
*File name 			:cpu_sm.c
*Function Name		:void Update_Event_logger_ID(void)
*Created By			:EM003 
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Get Event logger ID which is set on the board 
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Update_Event_logger_ID(void)
{
    BYTE DIP_val, uchTemp;
    TRISDbits.TRISD6 = SET_HIGH;
    TRISDbits.TRISD7 = SET_HIGH;    
    TRISFbits.TRISF0 = SET_HIGH;
    TRISFbits.TRISF1 = SET_HIGH;
    
    TRISEbits.TRISE2 = SET_LOW;
    TRISEbits.TRISE3 = SET_LOW;
    TRISEbits.TRISE4 = SET_LOW;

    LATEbits.LATE2 = SET_LOW;
    LATEbits.LATE3 = SET_HIGH;
    LATEbits.LATE4 = SET_HIGH;
    
    DIP_val = 0;
    for(uchTemp=0;uchTemp<100;uchTemp++);
    DIP_val = (~(((BYTE)PORTDbits.RD6) | (BYTE)((((BYTE)PORTDbits.RD7))<<1u) | 
            (BYTE)((((BYTE)PORTFbits.RF0))<<2u) | (BYTE)((((BYTE)PORTFbits.RF1))<<3u))) & 0x0F;
    
    LATEbits.LATE2 = SET_HIGH;
    LATEbits.LATE3 = SET_LOW;
    LATEbits.LATE4 = SET_HIGH;
    
    for(uchTemp=0;uchTemp<100;uchTemp++);
    DIP_val |= (BYTE)((~((PORTDbits.RD6) | (BYTE)((((BYTE)PORTDbits.RD7))<<1u) | 
            (BYTE)((((BYTE)PORTFbits.RF0))<<2u) | (BYTE)((((BYTE)PORTFbits.RF1))<<3u))) & 0x0f) <<4u;
    
    Event_Logger_ID = DIP_val;
}

/*********************************************************************************
*File name 			:cpu_sm.c
*Function Name		:void Check_High_Availabilty_Config(void)
*Created By			:EM003 
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Check if high availability is set or not
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Check_High_Availabilty_Config(void)
{
    BYTE  uchTemp;
    LATEbits.LATE2 = SET_HIGH;
    LATEbits.LATE3 = SET_HIGH;
    LATEbits.LATE4 = SET_LOW;
    
    High_Availability = 0;
    for(uchTemp=0;uchTemp<100;uchTemp++);
    High_Availability = !(PORTDbits.RD6);
    GSM_Enable = PORTDbits.RD7;    
    GSM_Sch_Info.GSM_Enable = !GSM_Enable;

}
/*********************************************************************************
*File name 			:cpu_sm.c
*Function Name		:void Check_Flash(void)
*Created By			:EM003 
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:This function is used to detect the Rom corruption.
*					 It performs crc32 bit operation on entire rom.
*					 It compares the calculated checksum with stored checksum value. 
*
*Algorithm			:1.Read the saved checksum from the system ID locations.
*					 2.Calculate crc32 bit for entire Rom.
*					 3.Compare stored checksum and calculated checksum.
*					 4.It sets the checksum status bit to "OK or BAD" depends on the comparison result.
*Description			: 
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Check_Flash(void)
{
	longtype_t CalculatedSum;
	longtype_t SavedSum;
    _prog_addressT CK_addr;

    CK_addr = CHECKSUM_LOCATION;
    CK_addr = _memcpy_p2d16(&SavedSum.DWord.HiWord.Byte.Hi,CK_addr,1);
    CK_addr = _memcpy_p2d16(&SavedSum.DWord.HiWord.Byte.Lo,CK_addr,1);
    CK_addr = _memcpy_p2d16(&SavedSum.DWord.LoWord.Byte.Hi,CK_addr,1);
    CK_addr = _memcpy_p2d16(&SavedSum.DWord.LoWord.Byte.Lo,CK_addr,1);;

    CalculatedSum.LWord = 0;
	CalculatedSum.LWord = crc32(CalculatedSum.LWord);
	if (CalculatedSum.LWord == SavedSum.LWord)
	{
		/* CRC-32 Check sum computed matches with the one stored in ID LOCATION */
		Status.Flags.Flash_CheckSum = CRC32_CHECKSUM_OK;
        SMCPU_CRC_checksum = SavedSum.LWord;
	}
	else	
	{
		Status.Flags.Flash_CheckSum = CRC32_CHECKSUM_BAD;
	}	
}

/*********************************************************************************
*File name 			:cpu_sm.c
*Function Name		:void Update_Feedback_State(void)
*Created By			:EM003 
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Check the feedback signals YLED, RLED, SPK, THEFT, DOOR 
*                     and log events appropriately
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Update_Feedback_State(void)
{
    if(Disp_Info.DS_mode== SECTION_CLEAR_AT_BOTH_UNITS)
    {
        if(YLED_FB_ERROR == 0)
        {
            if(Check_YLED_State() == 0) //YLED should have been ON
            {
                YLED_FB_ERROR = 1;
                Add_SM_Event_to_Queue(EVENT_YLED_BAD);
                feedback_error = 1;
                FB_error_ID = 1;
            }
        }
        else
        {
            if(Check_YLED_State() == 1) //YLED should have been ON
            {
                YLED_FB_ERROR = 0;
                Add_SM_Event_to_Queue(EVENT_YLED_RESTORED);
                feedback_error = 1;
                FB_error_ID = 6;
            }
        }
        if(RLED_FB_ERROR == 0)
        {
            if(Check_RLED_State() == 1) //RLED should have been OFF
            {
                RLED_FB_ERROR = 1;
                Add_SM_Event_to_Queue(EVENT_RLED_BAD);
                feedback_error = 1;
                FB_error_ID = 2;
            }
        }
        else
        {
            if(Check_RLED_State() == 0) //RLED should have been OFF
            {
                RLED_FB_ERROR = 0;
                Add_SM_Event_to_Queue(EVENT_RLED_RESTORED);
                feedback_error = 1;
                FB_error_ID = 7;
            }
        }
        if(SPK_FB_ERROR == 0)
        {
            if(Check_SPK_State() == 1) //SPK should have been OFF
            {
                SPK_FB_ERROR = 1;
                Add_SM_Event_to_Queue(EVENT_SPEAKER_BAD);
                feedback_error = 1;
                FB_error_ID = 3;
            }
        }
        else
        {
            if(Check_SPK_State() == 0) //SPK should have been OFF
            {
                SPK_FB_ERROR = 0;
                Add_SM_Event_to_Queue(EVENT_SPEAKER_RESTORED);
                feedback_error = 1;
                FB_error_ID = 8;
            }
        }
    }
    else if(Disp_Info.DS_mode == SECTION_OCCUPIED_AT_BOTH_UNITS)
    {
        if(YLED_FB_ERROR == 0)
        {
            if(Check_YLED_State() == 1)
            {
                YLED_FB_ERROR = 1;
                Add_SM_Event_to_Queue(EVENT_YLED_BAD);
                feedback_error = 1;
                FB_error_ID = 1;
            }
        }
        else
        {
            if(Check_YLED_State() == 0)
            {
                YLED_FB_ERROR = 0;
                Add_SM_Event_to_Queue(EVENT_YLED_RESTORED);
                feedback_error = 1;
                FB_error_ID = 6;
            }
        }
        if(RLED_FB_ERROR == 0)
        {
            if(Check_RLED_State() == 0)
            {
                RLED_FB_ERROR = 1;
                Add_SM_Event_to_Queue(EVENT_RLED_BAD);
                feedback_error = 1;
                FB_error_ID = 2;
            }
        }
        else
        {
            if(Check_RLED_State() == 1)
            {
                RLED_FB_ERROR = 0;
                Add_SM_Event_to_Queue(EVENT_RLED_RESTORED);
                feedback_error = 1;
                FB_error_ID = 7;
            }
        }
        if(SPK_FB_ERROR == 0)
        {
            if(Check_SPK_State() == 0)
            {
                SPK_FB_ERROR = 1;
                Add_SM_Event_to_Queue(EVENT_SPEAKER_BAD);
                feedback_error = 1;
                FB_error_ID = 3;
            }
        }
        else
        {
            if(Check_SPK_State() == 1)
            {
                SPK_FB_ERROR = 0;
                Add_SM_Event_to_Queue(EVENT_SPEAKER_RESTORED);
                feedback_error = 1;
                FB_error_ID = 8;
            }
        }
    }
    if(THFT_FB_ERROR == 0)
    {
        if(Check_Theft_Detect() == 0)
        {
            //Log Theft
            THFT_FB_ERROR = 1;
            Add_SM_Event_to_Queue(EVENT_THEFT);
            feedback_error = 1;
            FB_error_ID = 4;
        }
    }
    else
    {
        if(Check_Theft_Detect() == 1)
        {
            //Log Theft
            THFT_FB_ERROR = 0;
            Add_SM_Event_to_Queue(EVENT_THEFT_RESTORED);
            feedback_error = 1;
            FB_error_ID = 9;
        }
    }
    if(DOOR_FB_ERROR == 0)
    {
        if(Check_Door_Open() == 0)
        {
            //Log Door Open
            DOOR_FB_ERROR = 1;
            Add_SM_Event_to_Queue(EVENT_SYSTEM_DOOR_OPEN);
            feedback_error = 1;
            FB_error_ID = 5;
        }
    }
    else
    {
        if(Check_Door_Open() == 1)
        {
            //Log Door Open
            DOOR_FB_ERROR = 0;
            Add_SM_Event_to_Queue(EVENT_SYSTEM_DOOR_CLOSED);
            feedback_error = 1;
            FB_error_ID = 10;
        }
    }
}
/*********************************************************************************
*File name 			:cpu_sm.c
*Function Name		:BYTE Check_Theft_Detect(void)
*Created By			:EM003 
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Check if THEFT pin is 0 or 1 
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:BYTE
*
**********************************************************************************/
BYTE Check_Theft_Detect(void)
{
    if(THEFT_PIN == 0)
    {
        return 1;
    }
    return 0;
}
/*********************************************************************************
*File name 			:cpu_sm.c
*Function Name		:BYTE Check_Door_Open(void)
*Created By			:EM003 
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Check if DOOR pin is 0 or 1 
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:BYTE
*
**********************************************************************************/
BYTE Check_Door_Open(void)
{
    if(DOOR_PIN == 0)
    {
        return 1;
    }
    return 0;
}
/*********************************************************************************
*File name 			:cpu_sm.c
*Function Name		:BYTE Check_YLED_State(void)
*Created By			:EM003 
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Check if YLED pin is 0 or 1 
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:BYTE
*
**********************************************************************************/
BYTE Check_YLED_State(void)
{
    if(YLED_PIN == 0)
    {
        return 1;
    }
    return 0;
}
/*********************************************************************************
*File name 			:cpu_sm.c
*Function Name		:BYTE Check_RLED_State(void)
*Created By			:EM003 
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Check if RLED pin is 0 or 1 
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:BYTE
*
**********************************************************************************/
BYTE Check_RLED_State(void)
{
    if(RLED_PIN == 0)
    {
        return 1;
    }
    return 0;
}

/*********************************************************************************
*File name 			:cpu_sm.c
*Function Name		:BYTE Check_SPK_State(void)
*Created By			:EM003 
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Check if SPK pin is 0 or 1 
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:BYTE
*
**********************************************************************************/
BYTE Check_SPK_State(void)
{
    if(SPK_PIN == 0)
    {
        return 1;
    }
    return 0;
}


/*********************************************************************************
*File name 			:cpu_sm.c
*Function Name		:void Start_Sub_Systems(void)
*Created By			:EM003 
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Starts the RTC, EEPROM, Host communication and SMC communication shecduler.
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Start_Sub_Systems(void)
{
	Start_RTC_Sch();			/* from drv_rtc.c */
	Start_EEPROM_Sch();			/* from eeprom.c */
	Start_Host_Communication();	/* from comm_host.c */	
	Start_Smc_Communication();	/* from comm_smc.c */
	Start_A2D_Driver();			/* from drv_a2d.c */
    
    G_SPI_SS= SET_HIGH;
	T1CONbits.TON = 1;	/* Enable Timer 1 */
	T2CONbits.TON = 1;	/* Enable Timer 2 */

}

/*********************************************************************************
*File name 			:cpu_sm.c
*Function Name		:void main(void)
*Created By			:EM003 
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:main function which calls all other modules.
*Algorithm			:
*Description			:
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
int main(void)
{
    BYTE Always = 1;
    inspect_event_done = 0;
    GLCD_Info.State = GLCD_IDLE;
    GLCD_Info.Comm_Timeout_ms = MAX_COMM_TIMEOUT;
    
	Initialise_System();					 /* Initialise Ports and All Schedulers */
    Start_Sub_Systems();					 /* Start Schedulers */
    Add_SM_Event_to_Queue(EVENT_SM_POWERED_ON);	/* POWER ON event is added to Event Queue */
    
	do {
		if (IFS0bits.T1IF)
		{
			/* 10-mS Timer has overflowed */
			IFS0bits.T1IF = 0;
			TMR1 = TIMER1_SETPOINT;
			Decrement_RTC_10msTmr();		/* from drv_rtc.c */
			Decrement_A2D_10msTmr();		/* from drv_a2d.c */
			Decrement_Events_Sch_10msTmr();	/* from events.c */
            if(inspect_event_done == 0)
                init_delay_count++;
            if(init_delay_count == 5)
                inspect_event_done = 1;
		}
		if (IFS0bits.T2IF)
        {
			/* 1-mS Timer has overflowed */
			IFS0bits.T2IF = 0;
			TMR2 = TIMER2_SETPOINT;
			Decrement_SPI_Sch_msTmr();		/* from comm_dac.c */
			Decrement_Host_Sch_msTmr();		/* from comm_host.c */
			Decrement_Smc_Sch_msTmr();		/* from comm_smc.c */
			Decrement_GSM_Sch_msTmr();
			Decrement_EEPROM_msTmr();		/* from eeprom.c */
			Decrement_GLCD_msTmr();
        }
		Update_Vdd_Mon_State();				/* from drv_a2d.c */
		Update_EEPROM_State();				/* from eeprom.c */
		Update_RtcSch_State();				/* from drv_rtc.c */
		Update_Events_Sch_State();			/* from events.c */
		Update_SPI_State();					/* from comm_dac.c */
		Update_Host_Sch_State();			/* from comm_host.c */
        Update_Smc_Sch_State_QR();          /* from comm_smc.c */
        Update_USB_Sch_State();
        Update_GLCD_State();
        Update_GSM_Sch_State();
        if(DAC_sysinfo.Unit_Type == (BYTE)DAC_UNIT_TYPE_LCWS || DAC_sysinfo.Unit_Type == (BYTE)DAC_UNIT_TYPE_LCWS_DL )
        {
            Update_Feedback_State();
        }
	} while (Always == 1);
    return 0;
}
