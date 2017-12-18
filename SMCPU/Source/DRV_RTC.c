/*********************************************************************************************************************
*	Project		: 	Single Section Digital Axle Counter
*	Version		: 	2.0 
*	Revision	:	1	
*	Filename	: 	drv_rtc.c
*	Target MCU	: 	PIC24FJ256GB210   
*    Compiler	: 	XC16 Compiler V1.31  
*	Author		:	EM003 
*	Date		:	
*	Company Name: 	Insys Digital Systems
*	Modification History:
*					Rev No			Date		Description
*					 --				 --				--    
*	Functions	:   
*                    void Initialise_RTC_Sch(void);
*                    void Start_RTC_Sch(void);
*                    void Update_RtcSch_State(void);
*                    void Decrement_RTC_10msTmr(void);
*                    BOOL Read_RTC_Registers(void);
*                    void Update_System_Date_and_Time(void);
*                    BOOL Set_RTC_Date(BYTE uchDay, BYTE uchMonth, UINT16 uiYear);
*                    BOOL Set_RTC_Time(BYTE uchHour, BYTE uchMinute, BYTE uchSecond);
*********************************************************************************************************************/

#include <xc.h>
#include <time.h>

#include "COMMON.h"
#include "DRV_I2C.h"
#include "DRV_RTC.h"

extern sm_status_t Status;		/* from cpu_sm.c */
extern time_t SystemClock,SystemDate;		/* from cpu_sm.c */
extern BYTE uchDateTime[21];		/* from cpu_sm.c */
extern BOOL E_status;
rtc_info_t RTC_Info;				/* Structue information that handles the RTC Scheduler */
								   /* Maximum Days of All months in any year*/ 							
const BYTE uchDayUpperLimit[13] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};


BOOL Read_RTC_Registers(void);
void Update_System_Clock(void);


/*********************************************************************************
*File name 			:drv_rtc.c
*Function Name		:void Initialise_RTC_Sch(void)
*Created By			:EM003 
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			: Initialise The RTC Scheduler.
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Initialise_RTC_Sch(void)
{
	RTC_Info.State = RTC_SCHEDULER_NOT_STARTED;
	RTC_Info.Timeout_10ms = 0;
}
/*********************************************************************************
*File name 			:drv_rtc.c
*Function Name		:void Start_RTC_Sch(void)
*Created By			:EM003 
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:Start the RTC scheduler.	
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Start_RTC_Sch(void)
{
	RTC_Info.State = RTC_SCHEDULER_IDLE;
	RTC_Info.Timeout_10ms = 0;
}
/*********************************************************************************
*File name 			:drv_rtc.c
*Function Name		:void Update_RtcSch_State(void)
*Created By			:EM003 
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			: It Reads the Date and Time from RTC Chip and 
*					  Update the System Clock and Display with Regular Interval. 
*						
*Algorithm			:
*					 1.Initially RTC scheduler will be Idle State.
*					 2.When RTC_SCHEDULER_IDLE_TIMEOUT over, it will goto Step 3
*					 3.Read RTC Registers which Contain date and Time.
*					 4.Update the System Clock.
*					 5.Update the Display Date and Time.
*					 6.Set RTC_SCHEDULER_IDLE_TIMEOUT and goto step 2.
*
*Description			: 
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Update_RtcSch_State(void)
{
	switch (RTC_Info.State)
	{
        case RTC_SCHEDULER_NOT_STARTED:
            break;
		case RTC_SCHEDULER_IDLE:
			if (RTC_Info.Timeout_10ms == TIMEOUT_EVENT)
			{
				RTC_Info.State = RTC_READ_REGISTERS;
			}
			break;
		case RTC_READ_REGISTERS:
			if (Read_RTC_Registers())
			{
				Status.Flags.RTC_I2C_Failed = FALSE;
				RTC_Info.State = RTC_UPDATE_SYS_CLOCK;
			}
			else
			{
				RTC_Info.State = RTC_I2C_FAILURE;
			}
			break;
		case RTC_UPDATE_SYS_CLOCK:
			Update_System_Clock();
			RTC_Info.State = RTC_UPDATE_SYS_DATE_TIME;
			break;
		case RTC_UPDATE_SYS_DATE_TIME:
			RTC_Info.State = RTC_UPDATE_DISPLAY;
			break;
		case RTC_UPDATE_DISPLAY:
			RTC_Info.State = RTC_SCHEDULER_IDLE;
			RTC_Info.Timeout_10ms = RTC_SCHEDULER_IDLE_TIMEOUT;
			break;
		case RTC_I2C_FAILURE:
            I2C2STATbits.IWCOL = SET_LOW;			/* Clear Write collision bit */
            I2C2CONbits.RCEN = SET_LOW;			/* Put MSSP in Transmit mode */
			E_status = Generate_I2C_Stop();		/* Free the I2C BUS */
			Status.Flags.RTC_I2C_Failed = TRUE;
			RTC_Info.State = RTC_SCHEDULER_IDLE;
			RTC_Info.Timeout_10ms = RTC_SCHEDULER_IDLE_TIMEOUT;
			break;
        default:
            break;
	}
}
/*********************************************************************************
*File name 			:drv_rtc.c
*Function Name		:void Decrement_RTC_10msTmr(void)
*Created By			:EM003 
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			: For every 10ms, RTC Scheduler timeout is decremented.
*Algorithm			:
*Description			: 
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Decrement_RTC_10msTmr(void)
{
	if (RTC_Info.Timeout_10ms > 0)
	{
		RTC_Info.Timeout_10ms = RTC_Info.Timeout_10ms - 1;
	}
}
/*********************************************************************************
*File name 			:drv_rtc.c
*Function Name		:BOOL Read_RTC_Registers(void)
*Created By			:EM003 
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:This function reads RTC Registers and copies to RTC_info variable.
*
*Algorithm			:1.Generate "START Condition" 
*				     2.Select RTC IC in Write Mode
*					   and write the STATUS Register Address into RTC Address Register.  
*					 3.Generate "ReStart Condition" and Select RTC IC in Read Mode.
*					 4.Read out All Bytes of RTC Registers and store in Rtc_info Variable.
*					 5.Generate "STOP Condition"
*	
*Description			:  
*Input Element		:None
*Output Element		:Status of read RTC register function 
*
**********************************************************************************/
BOOL Read_RTC_Registers(void)
{
	BYTE uchCnt = 9;
	BYTE uchOffset = 0;	
	/*
	 * Read 9 registers starting with Status1 to Years from RTC and 
	 * populate RTC_Info.Data.Byte[0] to RTC_Info.Data.Byte[8]
	 */
	if (Generate_I2C_Start() == FALSE)
	{
		return (FALSE);				/* Communication failed */
	}
	E_status = Put_I2C_Byte(RTC_WRITE_ADDRESS);
	E_status = Put_I2C_Byte(RTC_STATUS_REGISTER);
	if (Generate_I2C_ReStart() == FALSE)
	{
		return (FALSE);				/* Communication failed */
	}
	E_status = Put_I2C_Byte(RTC_READ_ADDRESS);
		
	while (uchCnt > 0)
	{
		uchCnt = uchCnt - 1;
		I2C2CONbits.RCEN = SET_HIGH;
		RTC_Info.Data.Byte[uchOffset] = Get_I2C_Byte(uchCnt);
		uchOffset = uchOffset + 1;
	}
	E_status = Generate_I2C_Stop();
	return (TRUE);		/* RTC registers succesfully read */
}
/*********************************************************************************
*File name 			:drv_rtc.c
*Function Name		:void Update_System_Clock(void)
*Created By			:EM003 
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			: Converts the Data read from RTC registers into Ctime Format and 
*					  update to System_Clock.
*Algorithm			:					 
*Description			:
*					  years  -  represented one Register in BCD Format ( 00 -99)
*					  Months -  b7 - Century flag ( 1 indicates - 19xx, 0 indicates- 20xx),
*							    b0 to b4 - Month   (01-12) in BCD Format
*					  Days   -  b0 to b5 - (01 - 31) in BCD Format
*					  hour   -  b0 to b5 - (00 - 23) in Bcd Format
*					  Minute -  b0 to b6 - (00 - 59) in BCD Format 			  	 
*					  Second -  B7 - VL(voltage Low Sense) b0 to b6 - (00 - 59) in BCD Format
*
*					  The Above Data has to be converted to Time Format.
*									 
*Input Element		:None
*Output Element		:void
*
**********************************************************************************/
void Update_System_Clock(void)
{
	struct tm rtctime;
	BYTE uchBcd;
		
	rtctime.tm_year = BCD_TO_BIN(RTC_Info.Data.Register.Years);   
	if ((RTC_Info.Data.Register.Months &(BYTE)0x80) == 0)       /* Bit B7 is Checked for Century Updation */
	{
		rtctime.tm_year += 100;		/* Century is 20xx */
	}

	uchBcd = (RTC_Info.Data.Register.Months &(BYTE)0x1F);       /* Bit b5 to b7 is masked, Because b0 to b4 represents the Month*/
	rtctime.tm_mon = BCD_TO_BIN(uchBcd);						
	rtctime.tm_mon = rtctime.tm_mon - 1;											/* Month is 0 to 11 in struct tm */

	uchBcd = (RTC_Info.Data.Register.Days &(BYTE)0x3F);			/* Bit b6 and b7 is masked, Because b0 to b5 represents the day*/
	rtctime.tm_mday = BCD_TO_BIN(uchBcd);

	uchBcd = (RTC_Info.Data.Register.Hours &(BYTE)0x3F);        /* Bit b6 and b7 is masked, Because b0 to b5 represents the Hour*/
	rtctime.tm_hour = BCD_TO_BIN(uchBcd);

	uchBcd = (RTC_Info.Data.Register.Minutes &(BYTE)0x7F);      /* Bit b7 is masked, Because b0 to b6 represents the Minute*/	
	rtctime.tm_min = BCD_TO_BIN(uchBcd);

	uchBcd = (RTC_Info.Data.Register.Seconds &(BYTE)0x7F);		/* Bit b7 is masked, Because b0 to b6 represents the Seconds*/	
	rtctime.tm_sec = BCD_TO_BIN(uchBcd);

	/*
	 * mktime returns number of seconds elapsed since 1970.
	 */
	SystemClock = mktime(&rtctime);	

    
    rtctime.tm_sec = 0;
    rtctime.tm_min = 0;
    rtctime.tm_hour = 0;
    SystemDate = mktime(&rtctime);

}

/*********************************************************************************
*File name 			:drv_rtc.c
*Function Name		:BOOL Set_RTC_Date(BYTE uchDay, BYTE uchMonth, UINT16 uiYear)
*Created By			:EM003 
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			: This function is used to change day, Month and Year of Real Time Clock IC
*Algorithm			:
*					1.Check DAY, Month and Year for Valid Range.
*					2.If Not a valid Date return from funtion.
*					3.Convert DAY, Month and Year to BCD format   
*					4.Generate "START Condition" for i2c
*					5.Select RTC Ic in Write Mode.
*					6.Set RTC Address Pointer to RTC_DAYS_REGISTER.
*					7.Write Day into RTC_DAY Register.
*					8.Generate "STOP Condition" for i2c
*					9.Generate "START Condition" for i2c
*					10.Select RTC Ic in Write Mode.
*					11.Set RTC Address Pointer to RTC_MONTHS_REGISTER.
*					12.Write month into RTC_MONTH Register.
*					13.Write year into  RTC_YEAR Register.
*					14.Generate "STOP Condition" for i2c
*				
*Description			: 
*Input Element		:Day,Month,Year
*Output Element		:Status of Set_RTC_Date function
*
**********************************************************************************/
BOOL Set_RTC_Date(BYTE uchDay, BYTE uchMonth, UINT16 uiYear)
{
	BYTE uchLeapYear;
	
	uchLeapYear = (BYTE)(((uiYear % 4) == 0) && (((uiYear %100) != 0) || ((uiYear % 400) == 0)));

	if (uiYear < 2000 || uiYear > 2099 || uchMonth == 0 || uchMonth > 12)
	{
		return (BYTE)(FALSE);
	}
	if ((uchLeapYear) && uchMonth == 2 && uchDay > 29)
	{
		return (BYTE)(FALSE);
	}
	if (uchDay > (uchDayUpperLimit[uchMonth]))
	{
		return (BYTE)(FALSE);
	}
	/*
	 * Valid date, now convert to BCD
	 */
	uchDay = (BYTE)(uchDay + (BYTE)((uchDay/10)*6));		
	uchMonth = (BYTE)(uchMonth + (BYTE)((uchMonth/10)*6));
	uiYear = uiYear - 2000;
	uiYear = uiYear + (BYTE)((uiYear/10)*6);
	/*
	 * Proceed to set RTC date
	 */
	E_status = Generate_I2C_Start();
	E_status = Put_I2C_Byte(RTC_WRITE_ADDRESS);
	E_status = Put_I2C_Byte(RTC_DAYS_REGISTER);	
	E_status = Put_I2C_Byte(uchDay);
	/* RTC day and Month ,Year register addresses are not in Sequence. So i2c communication has to be 
	   stopped and restart for Saving Month and Year */	
	E_status = Generate_I2C_Stop();  
	Nop();		Nop();		Nop();		Nop();
	E_status = Generate_I2C_Start();
	E_status = Put_I2C_Byte(RTC_WRITE_ADDRESS);
	E_status = Put_I2C_Byte(RTC_MONTHS_REGISTER);	
	E_status = Put_I2C_Byte(uchMonth);
	E_status = Put_I2C_Byte((BYTE) uiYear);
	E_status = Generate_I2C_Stop();
	return (BYTE)(TRUE);
}
/*********************************************************************************
*File name 			:drv_rtc.c
*Function Name		:BOOL Set_RTC_Time(BYTE uchHour, BYTE uchMinute, BYTE uchSecond)
*Created By			:EM003 
*Date Created		:
*Modification History:
*					Rev No			Date		Description
*					 --				 --				--
*Tracability:
*		SRS()    	:
*
*Abstract			:This function is used to change Hour, Minute and Seconds of Real Time Clock IC
*Algorithm			:
*					1.Check hour, Minute and Second for Valid Range.
*					2.If Not a valid Time return from funtion.
*					3.Convert hour, Minute and Seconds to BCD format   
*					4.Generate "START Condition" for i2c
*					5.Select RTC Ic in Write Mode.
*					6.Set RTC Address Pointer to RTC_SECONDS_REGISTER.
*					7.Write Seconds,Minute,Hour into RTC Registers.
*					8.Generate "STOP Condition" for i2c
*Description			: 
*Input Element		:Hour,Minute,Second 
*Output Element		:Status of Set_RTC_Time function
*
**********************************************************************************/
BOOL Set_RTC_Time(BYTE uchHour, BYTE uchMinute, BYTE uchSecond)
{
	if (uchHour > 23 || uchMinute > 59 || uchSecond > 59)
	{
		return (BYTE)(FALSE);
	}
	/*
	 * Valid time, now convert to BCD
	 */
	uchHour   = (BYTE)(uchHour + (BYTE)((uchHour/10)*6));
	uchMinute = (BYTE)(uchMinute + (BYTE)((uchMinute/10)*6));
	uchSecond = (BYTE)(uchSecond + (BYTE)((uchSecond/10)*6));
	/*
	 * Proceed to set RTC Time
	 */
	E_status = Generate_I2C_Start();
	E_status = Put_I2C_Byte(RTC_WRITE_ADDRESS);
	E_status = Put_I2C_Byte(RTC_SECONDS_REGISTER);	
	E_status = Put_I2C_Byte(uchSecond);
	E_status = Put_I2C_Byte(uchMinute);
	E_status = Put_I2C_Byte(uchHour);
	E_status = Generate_I2C_Stop();
	return (BYTE)(TRUE);
}
