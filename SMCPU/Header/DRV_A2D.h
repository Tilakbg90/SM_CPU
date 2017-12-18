/*********************************************************************************************************************
	Project		: 	Single Section Digital Axle Counter
	Version		: 	2.0 
	Revision	:	1	
	Filename	: 	drv_a2d.h
	Target MCU	: 	PIC24FJ256GB210   
    Compiler	: 	XC16 Compiler V1.31  
	Author		:	EM003 
	Date		:	25/12/2005
	Company Name: 	Insys Digital Systems
	Modification History:
					Rev No			Date		Description
					 --				 --				--    
*********************************************************************************************************************/
#ifndef _DRV_A2D_H_
#define _DRV_A2D_H_

/* A/D configuration */
#define ADCON1_CONFIG		0x00E4; /*   auto conversion,sample after conversion ends  */
#define ADCON2_CONFIG		0x207D	/* Vref+ external, Vref- AVSS, interrupt on 32nd completion, 1  word 32 buffers */
#define ADCON3_CONFIG		0x1F00		/* System clock, TAD = 31, Tcy = 0 */

#define A2D_CS_DEFAULT          0x0004     /*Negative input is Vr- for Channel A as well as channel B, AN0*/
#define A2D_BG_DEFAULT          0b0000000000000000      /*Band gap is kept 0*/
#define A2D_SC_DEFAULT          0b0000000000000000      /*Default scan select is kept 0*/

#define CHANNEL4_SELECT		0b00010001		/* Select AN4 and Switch on A/D converter */

#define NO_OF_READINGS_TO_BE_AVGD	(10)		
/*
 */
//#define VDD_HIGH_SET_VALUE		(854)	/* VDD >  5.2V, AN4 = 2.108V */
//#define VDD_HIGH_CLEAR_VALUE		(839)	/* VDD <= 5.1V, AN4 = 2.071V */
//#define VDD_LOW_SET_VALUE			(772)	/* VDD <  4.7V, AN4 = 1.905V */
//#define VDD_LOW_CLEAR_VALUE		(805)	/* VDD >= 4.9V, AN4 = 1.987V */
//#define VDD_DEFAULT_VALUE			(821)	/* VDD >= 5.0V, AN4 = 2.027V */

/*
 * 511/2.45 = 208.5
 Ip	Mon     Vref    ADC_Mon
3.5	1.305	2.44    271.1
3.4	1.28	2.45    266.8
3.3	1.24	2.45    258.5
3.2	1.21	2.45    252.8
3.1	1.18	2.46    246.0
3	1.15	2.46    239.7
2.9	1.12	2.47    233.5
2.8	1.08	2.45    225.18
 */

//Vref = 2.45, 1023/2.45 = 417.5
#define VDD_HIGH_SET_VALUE			(542)	/* VDD >= 3.5V, AN3 = 1.30V */
#define VDD_HIGH_CLEAR_VALUE        (534)	/* VDD <= 3.4V, AN3 = 1.28V */
#define VDD_LOW_SET_VALUE			(451)	/* VDD <= 2.8V, AN3 = 1.08V */
#define VDD_LOW_CLEAR_VALUE			(480)	/* VDD >= 3.0V, AN3 = 1.15V */
#define VDD_DEFAULT_VALUE			(517)	/* VDD >= 3.3V, AN3 = 1.24V */

#define RTC_HIGH_SET_VALUE			(893)	/* RTC >= 3.6V, AN4 = 2.25V */
#define RTC_HIGH_CLEAR_VALUE        (879)	/* RTC <= 3.5V, AN4 = 2.16V */
#define RTC_LOW_SET_VALUE			(368)	/* RTC <= 1.5V, AN4 = 0.92V */
#define RTC_LOW_CLEAR_VALUE			(490)	/* RTC >= 2.0V, AN4 = 1.23V */
#define RTC_DEFAULT_VALUE			(840)	/* RTC >= 3.3V, AN4 = 2.04V */


#define A2D_MAX_ITERATIONS			(15)	/* Number of iteration for averaging (32*15 = 480 samples) */

typedef enum {
			VDD_MON_NOT_STARTED = 0,
			VDD_OK,
			VDD_HIGH,
			VDD_LOW,
} vdd_mon_state_t;

typedef enum {
			A2D_NOT_STARTED = 0,
			A2D_IDLE,
			A2D_WAIT_FOR_CONVERSION,
			A2D_SAVE_RESULT
} a2d_state_t;

typedef struct {
			a2d_state_t	State;
			BYTE		Timeout_10ms;
			UINT16		Iteration;
			wordtype_t	Result;
			UINT32		Summation;
} a2d_data_t;

typedef enum{
    ADC_WAIT,
    ADC_DONE
}ADC_CON;

typedef struct {
			vdd_mon_state_t State;
            ADC_CON Conv;
			UINT16	Vdd_Value;
} vdd_mon_info_t;

typedef enum{
    VDD,
    BAT
}ADC_USAGE;

#define ADC_SCHEDULER_IDLE_TIMEOUT	50

extern void Initialise_A2D_Driver(void);
extern void Start_A2D_Driver(void);
extern void Update_Vdd_Mon_State(void);
extern void Update_Feedback_State(void);
extern void Decrement_A2D_10msTmr(void);
extern BYTE Check_Theft_Detect(void);
extern BYTE Check_Door_Open(void);
extern BYTE Check_YLED_State(void);
extern BYTE Check_RLED_State(void);
extern BYTE Check_SPK_State(void);
#endif

