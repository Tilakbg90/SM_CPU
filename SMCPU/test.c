#include<xc.h>
#include <PPS.h>

#define SS1_M          LATCbits.LATC3
#define SS2_M          LATCbits.LATC2

#define SPI_MASTER  0x0060	// select 8-bit master mode, CKE=1, CKP=0
#define SPI_ENABLE  0x8000	// enable SPI port, clear status
#define SPI_SLAVE   0x0040	// select 8-bit slave mode, CKE=1, CKP=0

struct def_sys_timer
{
    unsigned int ms_1;
    unsigned int ms_10;
    unsigned int ms_100;
    unsigned int ms_200;
    unsigned int ms_500;
    unsigned int ms_1000;
    unsigned int ms_5000;
};

struct def_sys_timer sys_timer_counter;
struct def_sys_timer sys_timer_flags;

#define COM_BUFFER_MAX_BYTES 50

struct
{
    unsigned char tx_buffer[COM_BUFFER_MAX_BYTES];
    unsigned char rx_buffer[COM_BUFFER_MAX_BYTES];
    unsigned int tx_rd_ptr;
    unsigned int tx_wr_ptr;
    unsigned int rx_rd_ptr;
    unsigned int rx_wr_ptr;
    unsigned char tx_in_progress;
    unsigned char ss1_state;
    unsigned char ss2_state;
}spi_data1, spi_data2;

unsigned char tx[20] = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xAA,0xBB,0xCC,0xDD,0xEE,0xFF,0x00};    // Array of Data to be sent and received
unsigned char test_t[20],test_r[20];

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

void manage_timer(void)
{
    if(IFS0bits.T2IF)
    {
        IFS0bits.T2IF = 0;
        sys_timer_flags.ms_1 = 1;
        sys_timer_counter.ms_10 = sys_timer_counter.ms_10 + 1;
        if(sys_timer_counter.ms_10 >= 100)
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

void init_SPI1_IO (void) //MASTER GCPU
{

    #ifdef  __PIC24FJ256GB210__

        iPPSInput(IN_FN_PPS_SDI1,IN_PIN_PPS_RP25);        //RP25 - RD4 - SDI
        iPPSOutput(OUT_PIN_PPS_RP20,OUT_FN_PPS_SCK1OUT);    //RP20 - RD5 - SCK
        iPPSOutput(OUT_PIN_PPS_RP22,OUT_FN_PPS_SDO1);    //RP22 - RD3 - SDO
        //iPPSOutput(OUT_PIN_PPS_RP27,OUT_FN_PPS_SS1OUT);  //RP25 - RD13 - SS

    #endif

TRISDbits.TRISD4 = 1;            // set  SDI1 as an input
TRISDbits.TRISD3 = 0;            // set  SDO1 as an output
TRISDbits.TRISD5 = 0;             // set  SCK1 as an output
TRISDbits.TRISD13 = 0;             //  set  SS as an input

}

void init_Channel (void)
{
    TRISA = 0;
    PORTA=0;
    LATA=0;
    ANSA = 0x0000;
    ANSD = 0x0000;
    ANSG = 0x0000;
    ANSF = 0x0000;

}


void init_SPI2_IO (void) //SLAVE SMCPU
{

    #ifdef  __PIC24FJ256GB210__

        iPPSOutput(OUT_PIN_PPS_RP19,OUT_FN_PPS_SDO2);  //RP19 - RG8 - SDO
        iPPSOutput(OUT_PIN_PPS_RP21,OUT_FN_PPS_SCK2OUT);    //RP21 - RG6 - SCK
        iPPSInput(IN_FN_PPS_SDI2,IN_PIN_PPS_RP26);    //RP26 - RG7 - SDI
        //iPPSInput(IN_FN_PPS_SDI1,IN_PIN_PPS_RPI40);  //RPI40 - RC3 - SS

    #endif

TRISGbits.TRISG7 = 1;            // set  SDI1 as an input
TRISGbits.TRISG8 = 0;            // set  SDO1 as an output
TRISGbits.TRISG6 = 0;             // set  SCK1 as an output
TRISCbits.TRISC3 = 0;             //  set  SS1 as an input
TRISCbits.TRISC2 = 0;             //  set  SS2 as an input
}

unsigned char writeSPI2( unsigned char i )
{
    SPI2BUF = i;					// write to buffer for TX
    while(!SPI2STATbits.SRMPT);	// wait for transfer to complete
    return SPI2BUF;    				// read the received value
}//writeSPI2

unsigned char temp;
void writeSPI2_array()
{
    temp = 0;
    while(temp<8)
    {
    SPI2BUF = 0xF0;					// write to buffer for TX
    temp++;
    }
    while(SPI2STATbits.SRMPT==0);	// wait for transfer to complete

}//writeSPI2

unsigned char readSPI2( unsigned char i )
{
    unsigned char read_val  = 0;
    if(!SPI2STATbits.SRXMPT)	// wait for transfer to complete
    {
    read_val = SPI2BUF;
    }
    return read_val;    				// read the received value
}

void readSPI2_array( unsigned char *arr)
{
    while(!SPI2STATbits.SPIRBF);
    while(!SPI2STATbits.SRXMPT)
    {
    *arr = SPI2BUF;
    arr++;
    }
}

void SPI2INTInit()
{
          SPI2CON2bits.SPIBEN = 1;
  IFS2bits.SPI2IF = 0;

  IEC2bits.SPI2IE = 0;
    SPI2CON1 = SPI_MASTER;  // select mode
    SPI2STAT = SPI_ENABLE;  // enable the peripheral

} // SPI2INTInit
unsigned char CPU_selected = 0;
#define CPU1_LED LATFbits.LATF5
#define CPU2_LED LATFbits.LATF4
int main2 (void)
{
   unsigned char n=0,i=0;
   CLKDIVbits.CPDIV = 0;   // 8MHz input, 32MHz System Clock
    SPI1BUF=0;

    init_Channel();
    init_SPI2_IO();
    init_1ms_timer();
    SPI2INTInit();
    TRISFbits.TRISF4 = 0;
    TRISFbits.TRISF5 = 0;
    CPU1_LED = 0;
    CPU2_LED = 0;
    SS1_M = 1;
    SS2_M = 1;
    spi_data1.ss1_state = 0;
    spi_data1.tx_in_progress = 0;
    spi_data2.ss1_state = 0;
    spi_data2.tx_in_progress = 0;
    CPU_selected = 2;
    while(1)
    {
        if(CPU_selected == 1)
        {
            SS1_M = 0;
            SS2_M = 1;
        }
        else            //CPU 2 will be selected
        {
            SS2_M = 0;
            SS1_M = 1;
        }

        manage_timer();
        if(sys_timer_flags.ms_1000 == 1)
        {
            if(CPU_selected == 1)
            {
                if(!spi_data1.ss1_state && !spi_data1.tx_in_progress)
                {
                    spi_data1.ss1_state = 1; //activate
                    spi_data1.tx_in_progress = 1; //initiate tx
                }
            }
            else
            {
                if(!spi_data2.ss1_state && !spi_data2.tx_in_progress)
                {
                    spi_data2.ss1_state = 1; //activate
                    spi_data2.tx_in_progress = 1; //initiate tx
                }
            }
            sys_timer_flags.ms_1000 = 0;
        }
        if(spi_data1.tx_in_progress)
        {
            for (spi_data1.rx_wr_ptr = 0;spi_data1.rx_wr_ptr<=7;spi_data1.rx_wr_ptr++)
                spi_data1.rx_buffer[spi_data1.rx_wr_ptr] = 0;
            writeSPI2_array(); //Slave

            spi_data1.ss1_state = 0; //de-activate
            spi_data1.tx_in_progress = 0; //end of transmission
            readSPI2_array(&spi_data1.rx_buffer[0]); //What slave has recv
            CPU1_LED = 1;
            for(spi_data1.rx_rd_ptr = 0; spi_data1.rx_rd_ptr<=7;spi_data1.rx_rd_ptr++)
            {
                if(spi_data1.rx_buffer[spi_data1.rx_rd_ptr] != 0x11)
                {
                    CPU1_LED = 0;
                    break;
                }
            }
            SS1_M = 1;
            CPU_selected = 2;
        }
        if(spi_data2.tx_in_progress)
        {
            for (spi_data2.rx_wr_ptr = 0;spi_data2.rx_wr_ptr<=7;spi_data2.rx_wr_ptr++)
                spi_data2.rx_buffer[spi_data2.rx_wr_ptr] = 0;
            writeSPI2_array(); //Slave

            spi_data2.ss1_state = 0; //de-activate
            spi_data2.tx_in_progress = 0; //end of transmission
            readSPI2_array(&spi_data2.rx_buffer[0]); //What slave has recv
            CPU2_LED = 1;
            for(spi_data2.rx_rd_ptr = 0; spi_data2.rx_rd_ptr<=7;spi_data2.rx_rd_ptr++)
            {
                if(spi_data2.rx_buffer[spi_data2.rx_rd_ptr] != 0x11)
                {
                    CPU2_LED = 0;
                    break;
                }
            }
            SS2_M = 1;
            CPU_selected = 1;

        }
    }
}

