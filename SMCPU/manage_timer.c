
/*
 * File:   main.c
 * Author:
 *
 * Created on August 14, 2014, 6:24 PM
 */
#include <xc.h>
#define M0 LATBbits.LATB14
#define M1 LATBbits.LATB15
#include <PPS.h>
#if defined(__PIC24FJ256DA210__)

#endif

void init_ports(void);
void init_1ms_timer(void);
void manage_timer(void);

#define DIP_EN1_DIR     TRISEbits.TRISE2
#define DIP_EN2_DIR     TRISEbits.TRISE3
#define DIP_EN3_DIR     TRISEbits.TRISE4
#define KEY_A_DIR       TRISDbits.TRISD6
#define KEY_B_DIR       TRISDbits.TRISD7
#define KEY_C_DIR       TRISFbits.TRISF0
#define KEY_D_DIR       TRISFbits.TRISF1

#define DIP_EN1         LATEbits.LATE2
#define DIP_EN2         LATEbits.LATE3
#define DIP_EN3         LATEbits.LATE4
#define KEY_A           PORTDbits.RD6
#define KEY_B           PORTDbits.RD7
#define KEY_C           PORTFbits.RF0
#define KEY_D           PORTFbits.RF1

#define GS_LED          LATCbits.LATC2 //SS2
#define BS_LED          LATCbits.LATC3 //SS1

typedef struct
{
    unsigned int ms_1;
    unsigned int ms_10;
    unsigned int ms_100;
    unsigned int ms_200;
    unsigned int ms_500;
    unsigned int ms_1000;
    unsigned int ms_5000;
}def_sys_timer;

def_sys_timer sys_timer_counter;
def_sys_timer sys_timer_flags;

#define COM_BUFFER_MAX_BYTES 50

typedef struct
{
    unsigned char tx_buffer[COM_BUFFER_MAX_BYTES];
    unsigned int tx_rd_ptr;
    unsigned int tx_wr_ptr;
    unsigned int rx_rd_ptr;
    unsigned int rx_wr_ptr;
    unsigned char tx_in_progress;
    unsigned char enable_transmission;
    unsigned char rx_buffer[COM_BUFFER_MAX_BYTES];
    unsigned char rx_enable;
    unsigned char tx_enable;
    unsigned char rx_stat;
}com_data_x;
com_data_x com_data;
/*
 *
 */
int main2()
{
    CLKDIVbits.CPDIV = 0;   // 8MHz input, 32MHz System Clock
    init_ports();

    init_1ms_timer();
    com_data.tx_rd_ptr = 0;
    com_data.tx_wr_ptr = 0;
    com_data.tx_enable = 0;
    com_data.rx_enable = 0;
    com_data.rx_stat = 0;
    while(1)
    {
        manage_timer();
        if(sys_timer_flags.ms_1 == 1)
        {
            sys_timer_flags.ms_1 = 0;
        }

        if(sys_timer_flags.ms_500)
        {
            // some action for every 500 ms
            sys_timer_flags.ms_500 = 0;
        }
    }
    return 0;
}


void manage_timer(void)
{
    if(IFS0bits.T2IF)
    {
        IFS0bits.T2IF = 0;
        sys_timer_flags.ms_1 = 1;
        sys_timer_counter.ms_10 = sys_timer_counter.ms_10 + 1;
        if(sys_timer_counter.ms_10 >= 10)
        {
            sys_timer_counter.ms_10 = 0;
            sys_timer_flags.ms_10 = 1;
        }

        sys_timer_counter.ms_100 = sys_timer_counter.ms_100 + 1;
        if(sys_timer_counter.ms_100 >= 100)
        {
            sys_timer_counter.ms_100 = 0;
            sys_timer_flags.ms_100 = 1;
        }

        sys_timer_counter.ms_200 = sys_timer_counter.ms_200 + 1;
        if(sys_timer_counter.ms_200 >= 200)
        {
            sys_timer_counter.ms_200 = 0;
            sys_timer_flags.ms_200 = 1;
        }
        sys_timer_counter.ms_500 = sys_timer_counter.ms_500 + 1;
        if(sys_timer_counter.ms_500 >= 500)
        {
            sys_timer_counter.ms_500 = 0;
            sys_timer_flags.ms_500 = 1;
        }

        sys_timer_counter.ms_1000 = sys_timer_counter.ms_1000 + 1;
        if(sys_timer_counter.ms_1000 >= 1000)
        {
            sys_timer_counter.ms_1000 = 0;
            sys_timer_flags.ms_1000 = 1;
        }

        sys_timer_counter.ms_5000 = sys_timer_counter.ms_5000 + 1;
        if(sys_timer_counter.ms_5000 >= 5000)
        {
            sys_timer_counter.ms_5000 = 0;
            sys_timer_flags.ms_5000 = 1;
        }

    }
}




void init_ports(void)
{
    // Configure All ports as digital I/O
    ANSA = 0x0000;
    ANSB = 0x0000;
    ANSC = 0x0000;
    ANSD = 0x0000;
    ANSE = 0x0000;
    ANSF = 0x0000;
    ANSG = 0x0000;

    TRISC = 0x0000;

    DIP_EN1_DIR = 0;
    DIP_EN2_DIR = 0;
    DIP_EN3_DIR = 0;

    KEY_A_DIR = 1;
    KEY_B_DIR = 1;
    KEY_C_DIR = 1;
    KEY_D_DIR = 1;

    DIP_EN1 = 1;
    DIP_EN2 = 1;
    DIP_EN3 = 0;

}

void init_1ms_timer(void)
{
    // Configure Timer 2 to run at 16MHz, interrupt flag to set once in 1mS

    T2CON = 0;
    TMR2 = 0;               // Reset Timer to 0
    PR2 = 16000;            // 1mS / 62.5nS = 16000.
    IEC0bits.T2IE = 0;      // Disable Interrupts for Timer 2
    IFS0bits.T2IF = 0;      // Clear Interrupt Flag
    T2CONbits.TON = 1;      // Start Timer
}
