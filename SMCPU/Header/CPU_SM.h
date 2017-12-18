/*********************************************************************************************************************
	Project		: 	Single Section Digital Axle Counter
	Version		: 	2.0 
	Revision	:	1	
	Filename	: 	cpu_sm.h
	Target MCU	: 	PIC24FJ256GB210   
    Compiler	: 	XC16 Compiler V1.31  
	Author		:	EM003 
	Date		:	
	Company Name: 	Insys Digital Systems
	Modification History:
					Rev No			Date		Description
					 --				 --				--    
*********************************************************************************************************************/
#ifndef _CPU_SM_H_
#define _CPU_SM_H_
/*
 *      Header file for CPU_SM.C
 */
/* I/O Ports */


/* ID Locations from where CRC-32 CheckSum will be read */




/* Port Allocation Table and it's Default Values and Directions 
	PORTA	Port allocation -   Direction - Default Value 	PORTB	Port allocation - Direction -Default Value 
PORT RA0 -  Vin_Sense Signal  -    1    -     0		  PORT RB0- CPU1 Detect		  -    1    -     0	
PORT RA1 - 	Rtc Battery Signal-    1    -     0		  PORT RB1- CPU2 Detect		  -    1    -     0	
PORT RA2 - 	Not Used		  -	   1    -     1		  PORT RB2- Not Used		  -    0    -     0	
PORT RA3 - 	+ Vref for A2d    -    1    -     1		  PORT RB3- Not Used		  -    0    -     0	
PORT RA4 -	Not Used		  -    0    -     1  	  PORT RB4- Not Used		  -    0    -     0	
PORT RA5 -	+5v Monitor Signal-    1    -     0		  PORT RB5- Not Used 		  -    0    -     0	
PORT RA6 -	Osc1 Pin		  -    1    -     0		  PORT RB6- Not Used		  -    0    -     0	
PORT RA7 -  Osc2 Pin  		  -    1    -     0		  PORT RB7-	Not Used		  -    0    -     0	


	PORTC	Port allocation -   Direction - Default Value 	PORTD	Port allocation - Direction -Default Value 
PORT RC0 -  EEPROM Write Protect-  0    -     1		  PORT RD0- Not Used		  -    0    -     0		  
PORT RC1 - 	Not Used          -    0    -     0		  PORT RD1- Not Used		  -    0    -     0	
PORT RC2 - 	Not Used		  -	   0    -     0		  PORT RD2- Not Used		  -    0    -     0	
PORT RC3 - 	SCK of SPI		  -    0    -     0		  PORT RD3- Not Used		  -    0    -     0	
PORT RC4 -	SDI of SPI		  -    1    -     0  	  PORT RD4- Not Used		  -    0    -     0	
PORT RC5 -	SDO of SPI		  -    0    -     0		  PORT RD5- SDA of i2c		  -    0    -     0	
PORT RC6 -	Tx of USART 1	  -    0    -     0		  PORT RD6- SCL of i2c		  -    0    -     0	
PORT RC7 -  RX of USART 1  	  -    1    -     0		  PORT RD7-	Not Used		  -    0    -     0	


	PORTE	Port allocation -   Direction - Default Value 	PORTF	Port allocation - Direction -Default Value 
PORT RE0 -  Not Used		  -    0    -     0		  PORT RF0- Not Used		  -    0    -     0		  
PORT RE1 - 	Not Used          -    0    -     0		  PORT RF1- Not Used		  -    0    -     0	
PORT RE2 - 	Not Used		  -	   0    -     0		  PORT RF2- Not Used		  -    0    -     0	
PORT RE3 - 	Not Used		  -    0    -     0		  PORT RF3- Not Used		  -    0    -     0	
PORT RE4 -	Not Used		  -    0    -     0  	  PORT RF4- Not Used		  -    0    -     0	
PORT RE5 -	Not Used		  -    0    -     0		  PORT RF5- Not Used		  -    0    -     0	
PORT RE6 -	Not Used		  -    0    -     0		  PORT RF6- Not Used		  -    0    -     0	
PORT RE7 -  Not Used  		  -    0    -     0		  PORT RF7-	Not Used		  -    0    -     0	



PORTG	Port allocation -   Direction - Default Value 	PORTH	Port allocation - Direction -Default Value 
PORT RG0 -  Not Used		  -    0    -     0		  PORT RH0- LCD- RS 		  -    0    -     0		  
PORT RG1 - 	Tx of USART 1     -    0    -     0		  PORT RH1- LCD Detect		  -    1    -     0	
PORT RG2 - 	Rx of USART 1	  -	   1    -     0		  PORT RH2- Lcd Data Enable   -    0    -     0	
PORT RG3 - 	Not Used		  -    0    -     0		  PORT RH3- Lcd Strobe		  -    0    -     0	
PORT RG4 -	Not Used		  -    0    -     0  	  PORT RH4- Lcd Data		  -    0    -     0	
PORT RG5 -	MCLR pin		  -    0    -     0		  PORT RH5- Lcd Clock		  -    0    -     0	
													  PORT RH6- Not Used		  -    0    -     0	
													  PORT RH7-	Not Used		  -    0    -     0	

PORTJ	Port allocation -   Direction - Default Value 	
PORT RJ0 -  Modem RS		  -    0    -     1		 
PORT RJ1 - 	Modem CD          -    1    -     0		  
PORT RJ2 - 	Modem CS 		  -	   1    -     0		  
PORT RJ3 - 	Not Used		  -    0    -     0		  
PORT RJ4 -	Slave Select1 for SPI- 0    -     1  	  
PORT RJ5 -	Slave Select2 for SPI- 0    -     1		  
PORT RJ6 -	Not Used		  -    0    -     0		  
PORT RJ7 -  Not Used  		  -    0    -     0		  

*/



/* Default values to be loaded on to I/O PORTS on Power-up */
#define PORT_A_DEFAULT	0b0000000000011100
#define PORT_B_DEFAULT	0b1110000000000000
#define PORT_C_DEFAULT	0b0000000000001100
#define PORT_D_DEFAULT	0b0000000000000000
#define PORT_E_DEFAULT	0b0000000000011100
#define PORT_F_DEFAULT	0b0000000000000000
#define PORT_G_DEFAULT	0b0000000111000000

/* Default PORT Directions to be loaded on to DIRECTION REGISERS on Power-up */
#define PORT_A_DIRECTION	0b0000010010001100
#define PORT_B_DIRECTION	0b0000000000111000
#define PORT_C_DIRECTION	0b0000000011000000
#define PORT_D_DIRECTION	0b0000000000010000
#define PORT_E_DIRECTION	0b0000000000100000
#define PORT_F_DIRECTION	0b0001000000010111
#define PORT_G_DIRECTION	0b0000000010000000

#define DIGITAL_CONFIG      0x0000
#define TIMER1_CONFIG		0x0020	/* Set as 16-bit Timer, 1:64 Pre-Scaler, 1 cnt= 4us */
#define TIMER2_CONFIG		0x0020	/* Set as 16-bit Timer, 1:64 Pre-Scaler, 1 cnt= 4us */
#define PR1_SET             2500  /*Timer On and Pre-Scale=1:64, 1 cnt= 4us, 4*2500 = 10ms*/
#define PR2_SET             250   /*Timer On and Pre-Scale=1:64, 1 cnt= 4us, 4*250  = 1ms*/

#define TIMER3_CONFIG		0b0000000000000000	/* 1:4 Pre-Scaler, 1:1 Post-Scaler */
#define TIMER1_SETPOINT		0			/* 10 mSec - 60 for 20MHz */
#define TIMER2_SETPOINT		0 		/* 1 mSec - 60535 for 20MHz */
#define TIMER3_SETPOINT     65			/* 9600 Baud for SPI COMMUNICATION, 65 for 20MHz */

#define DIP_EN1             LATEbits.LATE2
#define DIP_EN2             LATEbits.LATE3
#define DIP_EN3             LATEbits.LATE3

#define CLEAR               0

void Initialise_System(void);
void Start_Sub_Systems(void);
void Check_Boards(void);
void Check_Flash(void);

#endif
