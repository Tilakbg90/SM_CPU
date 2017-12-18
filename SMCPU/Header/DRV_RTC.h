/*********************************************************************************************************************
	Project		: 	Single Section Digital Axle Counter
	Version		: 	2.0 
	Revision	:	1	
	Filename	: 	drv_rtc.h
	Target MCU	: 	PIC24FJ256GB210   
    Compiler	: 	XC16 Compiler V1.31  
	Author		:	EM003 
	Date		:	
	Company Name: 	Insys Digital Systems
	Modification History:
					Rev No			Date		Description
					 --				 --				--    
*********************************************************************************************************************/
#ifndef _DRV_RTC_H_
#define _DRV_RTC_H_

#define RTC_WRITE_ADDRESS			(0xA2)	/* this address to be sent to Select RTC in Write Mode */
#define RTC_READ_ADDRESS			(0xA3)	/* this address to be sent to Select RTC in Read Mode */
#define RTC_STATUS_REGISTER			(0)		/* Address of RTC Status Register */
#define RTC_SECONDS_REGISTER		(2)		/* Address of RTC Seconds Register */
#define RTC_DAYS_REGISTER			(5)		/* Address of RTC Days Register */
#define RTC_MONTHS_REGISTER			(7)		/* Address of RTC Month Register */
#define RTC_YEARS_REGISTER			(8)		/* Address of RTC Year Register */

/* Process Time-outs in multiples of 10-Ms */
#define RTC_SCHEDULER_IDLE_TIMEOUT	75  
/* For Every 750ms Date and Time is read from RTC Register and System Clock is Updated */

/* RTC Scheduler States */
typedef enum {
			RTC_SCHEDULER_NOT_STARTED = 0,		/* Default State */
			RTC_SCHEDULER_IDLE,					/* when scan time completed, it will go to RTC READ REGISTER State */
			RTC_READ_REGISTERS,					/* In this State all RTC Register are Read */
			RTC_UPDATE_SYS_CLOCK,				/* updates the system Clock */
			RTC_UPDATE_SYS_DATE_TIME,			/* Update the Date and Time Display Variable */
			RTC_UPDATE_DISPLAY,					/* Calls the Display Function with updated Date and Time Variable */
			RTC_I2C_FAILURE						/* Whenever RTC i2c communication Failed, Scheduler will come to this State */ 
} rtc_state_t;


/* RTC Scheduler  */
typedef struct {
	rtc_state_t State;		/* RTC Scheduler State */
	BYTE Timeout_10ms;		/* 10 milliSecond timer for RTC Scheduler */ 
	union {
		struct {
			BYTE	Status1;	/* Address 0: Control/Status 1 */
			BYTE	Status2;	/* Address 1: Control/Status 2 */
			BYTE	Seconds;	/* Address 2: Seconds/VL */
			BYTE	Minutes;	/* Address 3: Minutes */
			BYTE	Hours;		/* Address 4: Hours */
			BYTE	Days;		/* Address 5: Days */
			BYTE	Weekdays;	/* Address 6: Weekdays */
			BYTE	Months;		/* Address 7: Months/Century */
			BYTE	Years;		/* Address 8: Years */
		} Register ;
	BYTE Byte[9];
	} Data;
} rtc_info_t;


extern void Initialise_RTC_Sch(void);
extern void	Start_RTC_Sch(void);
extern void Update_RtcSch_State(void);
extern void Decrement_RTC_10msTmr(void);

#endif

