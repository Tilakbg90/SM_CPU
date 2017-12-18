/*********************************************************************************************************************
*	Project		: 	Single Section Digital Axle Counter
*	Version		: 	2.0 
*	Revision	:	1	
*	Filename	: 	crc16.c
*	Target MCU	: 	PIC24FJ256GB210   
*    Compiler	: 	XC16 Compiler V1.31  
*	Author		:	EM003
*	Date		:	
*	Company Name: 	Insys Digital Systems
*	Modification History:
*					Rev No			Date		Description
*					 --				 --				--    
*	Functions	:   UINT16 Crc16(const BYTE *puchMsg, INT16 iMsgLen)			
*********************************************************************************************************************/
#include <time.h>

#include "COMMON.h"
#include "CRC16.h"
#include "COMM_DAC.h"
#include "COMM_SMC.h"
#include "EEPROM.h"
#include "command_proc.h"
#include "DRV_GLCD_SPI.h"

extern spirecvobject SPIRecvObject;
extern smc_info_t Smc1XmitObject;
extern eeprom_sch_info_t EEPROM_Sch_Info;

extern host_recv_msg_info_t 		Com2RecvObject;
extern xmit_queue_info_t 		 Xmit_Queue;
extern glcd_info_t GLCD_Info;

extern smc_sch_info_t             Smc_Sch_Info;			/* structure holds station master interface communication scheduler */
/* Table of CRC values for high-order byte */
const unsigned char uchCRC16TableHi[256] = {
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40 } ;

/* Table of CRC values for low-order byte */
const unsigned char uchCRC16TableLo[256] = {
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
0x40 } ;
/*********************************************************************************
*File name 			:crc16.c
*Function Name		:UINT16 Crc16(CRC_PACK Pack, INT16 iMsgLen)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:
*Algorithm			:
*Description			: 
*Input Element		:CRC_PACK Pack:Which array should be considered for crc calculation
*                    INT16 iMsgLen: Array length
*Output Element		:Calculated CRC value
*
**********************************************************************************/
UINT16 Crc16(CRC_PACK Pack, INT16 iMsgLen)
{
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	 * Computes 16-bit cyclic redundancy check (CRC) value. Look-up table is   *
	 * for Polynomial 0xA001 and initial value of CRC is 0xFFFF.               *
	 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	BYTE uchCRCHi = 0xFF ; /* high byte of CRC initialized */
	BYTE uchCRCLo = 0xFF ; /* low byte of CRC initialized */
	BYTE uchIndex ;        /* will index into CRC lookup table */
	BYTE uchMsgIndex =0;

	while (iMsgLen)	/* pass through message buffer */
	{
		iMsgLen = iMsgLen - 1;
        switch(Pack)
        {
            case SPI_RECV_OBJ:
                uchIndex = uchCRCHi ^ SPIRecvObject.Msg_Buffer[uchMsgIndex];
                break;
            case SMC_OBJ:
                uchIndex = uchCRCHi ^ Smc1XmitObject.Msg_Buffer[uchMsgIndex];
                break;
            case EEPROM_CPU1:
                uchIndex = uchCRCHi ^ EEPROM_Sch_Info.CPU1_Data[uchMsgIndex];
                break;
            case EEPROM_CPU2:
                uchIndex = uchCRCHi ^ EEPROM_Sch_Info.CPU2_Data[uchMsgIndex];
                break;
            case COM2_RECV_OBJ:
                uchIndex = uchCRCHi ^ Com2RecvObject.Msg_Buffer[uchMsgIndex];
                break;
            case XMIT_QUEUE:
                uchIndex = uchCRCHi ^ Xmit_Queue.Record[0].Msg_Buffer[uchMsgIndex];
                break;
            case GLCD_INFO:
                uchIndex = uchCRCHi ^ GLCD_Info.Message_Buffer[uchMsgIndex];
                break;
            case SMC_QUERY:
                uchIndex = uchCRCHi ^ Smc_Sch_Info.Recvd_query[uchMsgIndex];
                break;
            default:
                uchIndex = 0;
                break;
        }
		uchMsgIndex = uchMsgIndex + 1;
		uchCRCHi = uchCRCLo ^ uchCRC16TableHi[uchIndex];
		uchCRCLo = uchCRC16TableLo[uchIndex];
	}
	return (UINT16)((UINT16) uchCRCLo << 8 | uchCRCHi);
}

