/*********************************************************************************************************************
	Project		: 	Single Section Digital Axle Counter
	Version		: 	2.0 
	Revision	:	1	
	Filename	: 	drv_i2c.h
	Target MCU	: 	PIC24FJ256GB210   
    Compiler	: 	XC16 Compiler V1.31  
	Author		:	EM003
	Date		:	
	Company Name: 	Insys Digital Systems
	Modification History:
					Rev No			Date		Description
					 --				 --				--    
********************************************************************************************************************/
#ifndef _DRV_I2C_H_
#define _DRV_I2C_H_

extern BOOL Has_SSPIF_gone_high(void);
extern void Initialise_I2C_Driver(void);
extern BOOL Generate_I2C_Start(void);
extern BOOL Generate_I2C_ReStart(void);
extern BOOL Generate_I2C_Stop(void);
extern void Give_I2C_ACK(void);
extern void Give_I2C_NAK(void);
extern BOOL Read_I2C_ACK_Status(void);
extern BOOL Put_I2C_Byte(BYTE uchData);
extern BYTE Get_I2C_Byte(BOOL uchReadContinue);
extern BOOL Has_SSPIF_gone_high(void);

#define I2C_CONFIG      0b1000000000000000
#define I2C_HIGH_SPEED_MODE  37

#endif

