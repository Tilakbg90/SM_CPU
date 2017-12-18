/*********************************************************************************************************************
	Project		: 	Single Section Digital Axle Counter
	Version		: 	2.0 
	Revision	:	1	
	Filename	: 	eeprom.h
	Target MCU	: 	PIC24FJ256GB210   
    Compiler	: 	XC16 Compiler V1.31  
	Author		:	EM003
	Date		:	
	Company Name: 	Insys Digital Systems
	Modification History:
					Rev No			Date		Description
					 --				 --				--    
*********************************************************************************************************************/
#ifndef _EEPROM_H_
#define _EEPROM_H_

#define CPU1_DATA_SAVE_ADDRESS			(0)       /* Address in On chip EEPROM where the CPU1 data get stored*/
#define CPU2_DATA_SAVE_ADDRESS			(8)       /* Address in On chip EEPROM where the CPU2 data get stored*/
#define EEPROM_PARKING_ADDRESS			(1023)    /* Default Address Value to which EEPROM Address Pointer is set*/
#define EEPROM_DATA_WRITE_WAIT			(5)       /* Time delay between the writes of each data bytes*/


/* EEPROM scheduler state */
typedef enum {
			EEPROM_SCHEDULER_NOT_STARTED = 0,     /* EEPROM scheduler Default state*/ 
			EEPROM_SCHEDULER_IDLE,                /* Initialise scheduler to idle state*/
			EEPROM_WRITE_CPU1_DATA,               /* Write CPU1 data into on-chip EEPROM*/
			EEPROM_WRITE_CPU1_DATA_WAIT,          /* Wait for 5ms before going to write Next Byte of Cpu1 Data */
			EEPROM_WRITE_CPU2_DATA,               /* Write CPU2 data into on-chip EEPROM*/
			EEPROM_WRITE_CPU2_DATA_WAIT           /* Wait for 5ms before going to write Next Byte of Cpu2 Data*/
} eeprom_state_t;

/* EEPROM Scheduler */
typedef struct {
		eeprom_state_t State;					  /* EEPROM Scheduler state */	
		BYTE Timeout_ms;						  /* one millisecond Timer for EEPROM Scheduler */	
		BYTE Byte_Count;						  /* No of Bytes to be stored for Each Cpu */		
		UINT16 Address;							  /* Pointer to Access the on-chip EEPROM data */			
		unsigned Save_CPU1_Register :1;			  /* Flag which triggers the saving Cpu1 Data into onchip EEPROM */	
		unsigned Save_CPU2_Register :1;			  /* Flag which triggers the saving Cpu2 Data into onchip EEPROM */	
		unsigned CPU1_Data_Good :1;        		  /* Flag indicates the Cpu1 Data Read from onchip EEprom is Vaild or Not*/
		unsigned CPU2_Data_Good :1;        		  /* Flag indicates the Cpu2 Data Read from onchip EEprom is Vaild or Not*/
		unsigned :4;
		BYTE CPU1_Data[NO_OF_EVENT_REGISTERS+2];  /* Buffer that stores the Cpu1 Data */		
		BYTE CPU2_Data[NO_OF_EVENT_REGISTERS+2];  /* Buffer that stores the Cpu2 Data */	
} eeprom_sch_info_t;


extern void Initialise_EEPROM_State(void);  
extern void	Start_EEPROM_Sch(void);         
extern void Update_EEPROM_State(void);      
extern void Decrement_EEPROM_msTmr(void);   
extern void Save_Event_Register_on_EEPROM(BYTE uchCPU, event_register_t Event_Reg);
extern BOOL Is_CPU1_EEPROM_Record_Valid(void);  
extern BOOL Is_CPU2_EEPROM_Record_Valid(void);  
    

extern eeprom_sch_info_t EEPROM_Sch_Info;				/* Structure holds eeprom scheduler information */
#endif
