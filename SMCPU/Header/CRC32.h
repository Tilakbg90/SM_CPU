/*********************************************************************************************************************
	Project		: 	Single Section Digital Axle Counter
	Version		: 	2.0 
	Revision	:	1	
	Filename	: 	crc32.h
	Target MCU	: 	PIC24FJ256GB210   
    Compiler	: 	XC16 Compiler V1.31  
	Author		:	EM003
	Date		:	
	Company Name: 	Insys Digital Systems
	Modification History:
					Rev No			Date		Description
					 --				 --				--    
*********************************************************************************************************************/
#ifndef _CRC32_H_
#define _CRC32_H_
/*
 *      CRC-32 routines header file
 */
#define CHECKSUM_LOCATION	0x2ABF0     /* System ID Location  0x200000L  to 0x200007L  */


typedef unsigned long int resultType;
#define POLYNOMIAL 0x04C11DB7
#define WIDTH (8 * sizeof(resultType))
#define MSb ((resultType)1 << (WIDTH - 1))
resultType crc32(resultType remainder);
#endif
