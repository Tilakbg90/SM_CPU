/*********************************************************************************************************************
	Project		: 	Single Section Digital Axle Counter
	Version		: 	2.0 
	Revision	:	1	
	Filename	: 	crc16.h
	Target MCU	: 	PIC24FJ256GB210   
    Compiler	: 	XC16 Compiler V1.31  
	Author		:	EM003
	Date		:	
	Company Name: 	Insys Digital Systems
	Modification History:
					Rev No			Date		Description
					 --				 --				--    
*********************************************************************************************************************/
#ifndef _CRC16_H_
#define _CRC16_H_
/*
 *      CRC-16 routines header file
 */

typedef enum{
    SPI_RECV_OBJ = 0,
    SMC_OBJ,
    EEPROM_CPU1,
    EEPROM_CPU2,
    COM2_RECV_OBJ,
    XMIT_QUEUE,
    GLCD_INFO,
    SMC_QUERY
} CRC_PACK;

extern UINT16 Crc16(CRC_PACK, INT16);
#endif
