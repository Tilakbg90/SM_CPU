/*********************************************************************************************************************
*	Project		: 	Single Section Digital Axle Counter
*	Version		: 	2.0
*	Revision	:	1
*	Filename	: 	drv_i2c.c
*	Target MCU	: 	PIC24FJ256GB210
*    Compiler	: 	XC16 V1.31
*	Author		:	EM003
*	Date		:
*	Company Name: 	Insys
*	Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*	Functions	:   
*                    void Initialise_I2C_Driver(void);
*                    BOOL Generate_I2C_Start(void);
*                    BOOL Generate_I2C_ReStart(void);
*                    BOOL Generate_I2C_Stop(void);
*                    void Give_I2C_ACK(void);
*                    void Give_I2C_NAK(void);
*                    BOOL Read_I2C_ACK_Status(void);
*                    BOOL Put_I2C_Byte(BYTE uchData);
*                    BYTE Get_I2C_Byte(BOOL uchReadContinue);
*                    BOOL Has_SSPIF_gone_high(void);
*********************************************************************************************************************/

#include <xc.h>

#include "COMMON.h"
#include "DRV_I2C.h"
#include "COMM_GSM.h"

extern BOOL E_status;
/*********************************************************************************
*File name 			:drv_i2c.c
*Function Name		:void Initialise_I2C_Driver(void)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			: Initialise the i2c driver.
*Algorithm			:
*Description			:
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Initialise_I2C_Driver(void)
{
	I2C2CON = I2C_CONFIG;
	I2C2BRG = I2C_HIGH_SPEED_MODE;
}
/*********************************************************************************
*File name 			:drv_i2c.c
*Function Name		:BOOL Generate_I2C_Start(void)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			: This function Generates START condition for Transmission.
*Algorithm			:
*Description			:
*Input Element		:None
*Output Element		:Return the status of START condition generated successfull or Not.
*
**********************************************************************************/
BOOL Generate_I2C_Start(void)
{
	IFS3bits.MI2C2IF = SET_LOW;
	I2C2CONbits.SEN = SET_HIGH;				/* Initiate START condition */
	return (Has_SSPIF_gone_high());
}
/*********************************************************************************
*File name 			:drv_i2c.c
*Function Name		:BOOL Generate_I2C_ReStart(void)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:This function Generates RE-START condition .
*Algorithm			:
*Description			:
*Input Element		:None
*Output Element		:Return the status of RE-START condition generated successfull or Not.
*
**********************************************************************************/
BOOL Generate_I2C_ReStart(void)
{
	IFS3bits.MI2C2IF = SET_LOW;
	I2C2CONbits.RSEN  = SET_HIGH;			/* Initiate Repeated START condition */
	return (Has_SSPIF_gone_high());
}
/*********************************************************************************
*File name 			:drv_i2c.c
*Function Name		:BOOL Generate_I2C_Stop(void)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:This function Generates STOP condition.
*Algorithm			:
*Description			:
*Input Element		:None
*Output Element		:Return the status of STOP condition generated successfull or Not.
**********************************************************************************/
BOOL Generate_I2C_Stop(void)
{
	IFS3bits.MI2C2IF = SET_LOW;
	I2C2CONbits.PEN   = SET_HIGH;				/* Initiate STOP condition */
	return (Has_SSPIF_gone_high());
}
/*********************************************************************************
*File name 			:drv_i2c.c
*Function Name		:void Give_I2C_ACK(void)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:This function send Ack signal to Slave which is transmitting.
*					 Only if Ack is given, Slave will continue to Transmit further Bytes.
*Algorithm			:
*Description			:
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Give_I2C_ACK(void)
{
	IFS3bits.MI2C2IF = SET_LOW;
	I2C2CONbits.ACKDT = SET_LOW;
	I2C2CONbits.ACKEN = SET_HIGH;
	E_status = Has_SSPIF_gone_high();
}

/*********************************************************************************
*File name 			:drv_i2c.c
*Function Name		:void Give_I2C_NAK(void)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:This function Generates Not-Ack signal to Slave which is transmitting.
*					 This Not-Ack is given to stop the slave from the further transmission.
*Algorithm			:
*Description			:
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Give_I2C_NAK(void)
{
	IFS3bits.MI2C2IF = SET_LOW;
	I2C2CONbits.ACKDT = SET_HIGH;
	I2C2CONbits.ACKEN = SET_HIGH;
	E_status = Has_SSPIF_gone_high();
}
/*********************************************************************************
*File name 			:drv_i2c.c
*Function Name		:BOOL Read_I2C_ACK_Status(void)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:This function reads Ack signal to detect that Slave response.
*Algorithm			:
*Description			:
*Input Element		:None
*Output Element		:return the Ack signal status.
*
**********************************************************************************/
BOOL Read_I2C_ACK_Status(void)
{
	Nop();	Nop();	Nop();	Nop();
	Nop();	Nop();	Nop();	Nop();
    Nop();	Nop();	Nop();	Nop();
    Nop();	Nop();	Nop();	Nop();
    return (I2C2STATbits.ACKSTAT);
}

/*********************************************************************************
*File name 			:drv_i2c.c
*Function Name		:BOOL Put_I2C_Byte(BYTE uchData)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Writes One Byte of Data into I2c Bus
*Algorithm			:
*Description			:
*Input Element		:uchData-Data to be transmitted
*Output Element		:Transmission Status
**********************************************************************************/
BOOL Put_I2C_Byte(BYTE uchData)
{
	BYTE uchAck=0;
    Receive_COM3_Message();
	IFS3bits.MI2C2IF = SET_LOW;
	I2C2TRN = uchData;
	E_status = Has_SSPIF_gone_high();		/* wait for SSPIF to go HIGH */
	uchAck = Read_I2C_ACK_Status();
	if(uchAck == SET_LOW)
	{
	   return (TRUE);
	}
	else
	{
	   return (FALSE);
	}
}
/*********************************************************************************
*File name 			:drv_i2c.c
*Function Name		:BYTE Get_I2C_Byte(BOOL uchReadContinue)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:read One Byte of Data received from I2c Slave
*Algorithm			:
*Description			:
*Input Element		:uchReadContinue- it indicates that that Read operation to be contiuned or Not.
*Output Element		:one Byte of Data read from the slave.
**********************************************************************************/
BYTE Get_I2C_Byte(BOOL uchReadContinue)
{
	BYTE uchData=0;
    IFS3bits.MI2C2IF = SET_LOW;
    I2C2CONbits.RCEN = 1;
	E_status = Has_SSPIF_gone_high();		/* wait for SSPIF to go HIGH */
	uchData = (BYTE)I2C2RCV;
	IFS3bits.MI2C2IF = SET_LOW;
	if(uchReadContinue)
	{
	   Give_I2C_ACK();
	}
	else
	{
	   Give_I2C_NAK();
	}
	return (uchData);
}
/*********************************************************************************
*File name 			:drv_i2c.c
*Function Name		:BOOL Has_SSPIF_gone_high(void)
*Created By			:EM003
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			: Wait for a Delay time for SSPIF to Set.Then SSPIF status is read and returned.
*Algorithm			:
*Description			:
*Input Element		:None
*Output Element		:returns the Status of SSPIF
**********************************************************************************/
BOOL Has_SSPIF_gone_high(void)
{
	UINT16 uchCnt=0;
	BOOL ReturnValue = FALSE;

	for (uchCnt = 0; uchCnt < 200; uchCnt++)
	{
		if (IFS3bits.MI2C2IF == SET_HIGH)
		{
		  ReturnValue = TRUE;
		  break;
		}
	}
	return (ReturnValue);
}
