/*********************************************************************************************************************
	Project		: 	Single Section Digital Axle Counter
	Version		: 	2.0 
	Revision	:	1	
	Filename	: 	comm_dac.h
	Target MCU	: 	PIC24FJ256GB210   
    Compiler	: 	XC16 Compiler V1.31  
	Author		:	EM003 
	Date		:	
	Company Name: 	Insys Digital Systems
	Modification History:
					Rev No			Date		Description
					 --				 --				--    
*********************************************************************************************************************/
#ifndef _COMMDAC_H_
#define _COMMDAC_H_

/*
 *      Comm_DAC routines header file
 */

#define SSP1STAT_DEFAULT 	0b01000000      /* b7- SMP - 0 -Input Data sampled at middle of data output Time */
											/* b6- CKE - 1 -Transmission occurs when clock transists from active to idle state*/
#define SSP1CON1_DEFAULT 	0b00110011		/* b5 -SSPEN -1 -enabled*/
											/* b4 -CKP  -1 -Idle state for Clock is High Level */ 	
											/* b3 to b0 -  SPI Master mode, clock = TMR2 output/2 */

#define SS_CPU1_PORT		LATCbits.LATC3				/* Output Port to Select Cpu1 */
#define SS_CPU2_PORT		LATCbits.LATC2				/* Output port to select Cpu2 */
#define SS_CPU1_PORT_DIR	TRISCbits.TRISC3          /* set CPU1 portJ4 data direction as output */
#define SS_CPU2_PORT_DIR	TRISCbits.TRISC2          /* set CPU2 portJ5 data direction as output */
#define SPI_MASTER  0x0060	// select 8-bit master mode, CKE=1, CKP=0
#define SPI_ENABLE  0x8000	// enable SPI port, clear status
#define SPI_SLAVE   0x0040	// select 8-bit slave mode, CKE=1, CKP=0

#define SPI_SELECT_SLAVE_DEVICE				0  /* set low for selecting the slave device*/
#define SPI_DESELECT_SLAVE_DEVICE			1  /* set high for deselecting the slave device*/

/* SPI Process time-outs in milli-seconds */
#define SPI_SCHEDULER_SCAN_RATE				(125) /* SPI transmitting schedule time for each CPU*/
#define SPI_PRE_TRANSMISSION_DELAY			(5)	  /* SPI Pre transmission Time */	
#define SPI_DELAY_BETWEEN_BYTES				(2)   /* Time delay between each byte of transmission*/ 
#define SPI_HOLD_SS_LOW_TIMEOUT				(2)   /* set slave select pin low Hold Time */



/* spi Receive Scheduler states */
typedef struct {
			BYTE	State;				/* SPI Receive state */
			UINT16	Timeout_ms;			/* one ms Timer */ 			
			BYTE	Msg_Length;			/* SPI receive message length */		
			BYTE	Index;				/* pointer to access receive message buffer */
			BYTE	Msg_Buffer[SPI_MESSAGE_LENGTH];  /* Message buffer to store received bytes of data */
} spirecvobject;

/* Process states  - SPI scheduler States*/
typedef enum {
			 SPI_SCHEDULER_IDLE	=0,                  /* set scheduler in idle state*/
 			 SPI_BREATHING_TIME_FOR_SLAVE,	         /* Pre transmission Time wait*/      
			 SPI_WAIT_FOR_XMIT_TO_FINISH,		     /* Transmit Byte State*/
			 SPI_WAIT_FOR_RECEIVE_TO_COMPLETE,		 /* Receive Byte State*/
			 SPI_CHECK_CRC,							 /* Validate Data */
			 SPI_HOLD_SS_LOW                         /*  SS Hold Low time Wait*/
			}spi_sch_state_t;					


/* SPI scheduler */
typedef struct {
			spi_sch_state_t	State;					 /* Spi scheduler state */
			UINT16	        Timeout_ms;				 /* one millisecond Timer */
			UINT16			ScanRate;				 /* Scan Time */
			UINT16			ElapsedTime;			 /* Time Required for Transmision with One cpu */
} comm_sch_info_t;

extern sm_status_t Status;				/* from cpu_sm.c */
extern dac_comm_error Dac_Comm_Err;			/* from cpu_sm.c */
extern BYTE CPU1_Address;				/* from cpu_sm.c */
extern BYTE CPU2_Address;				/* from cpu_sm.c */
extern dac_sysinfo_t DAC_sysinfo;			/* from cpu_sm.c */
extern BYTE Latest_DAC_data[SPI_MESSAGE_LENGTH];
extern BYTE Erase_Start;
extern BYTE inspect_event_done;

extern void	Initialise_SPI(void);
extern void Update_SPI_State(void);
extern void Set_SPI_Sch_Idle(void);
extern void Process_SPI_Message(BYTE);
extern void Build_Message(BYTE);
extern void Update_Shadow_Register(BYTE);
extern void Synchronise_Shadow_Register(void);
extern void Update_Comm_Err_Counter(BYTE);
extern void setup_SPI (void);
#endif
