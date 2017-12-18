#include "usb.h"


BOOL USBSleepOnSuspend(void)
{
    unsigned int U1EIE_save, U1IE_save, U1OTGIE_save;
    unsigned char USB1IE_save;

    #if defined(USB_POLLING)
        //If IPL is equal to 0 then there is no way for the USB module to
        //  generate an interrupt to wake up the device.  
        if(_IPL == 0)
        {
            return FALSE;
        }

        //Set the interrupt priority to a level that will wake up the part (>0)
        //  but will not cause a interrupt vector jump (USB1IP<=IPL)
        _USB1IP = 1;
    #endif 

    //Save the old interrupt and CPU settings
    U1EIE_save = U1EIE;
    U1IE_save = U1IE;
    U1OTGIE_save = U1OTGIE;
    USB1IE_save = IEC5bits.USB1IE;

    //Disable all USB interrupts
    U1EIE = 0;
    U1IE = 0;
    U1OTGIE = 0; 

    //Enable the interrupt
    IFS5bits.USB1IF = 0;
    U1OTGIEbits.ACTVIE = 1;
    USBClearInterruptFlag(USBActivityIFReg,USBActivityIFBitNum);
    IEC5bits.USB1IE = 1;

    Sleep();/*lint -e526 -e628*/

    #if defined(USB_POLLING)
        //Disable the interrupt
        _USB1IP = 0;
    #endif  

    //restore the previous interrupt settings
    IEC5bits.USB1IE = USB1IE_save;
    U1EIE = U1EIE_save;
    U1IE = U1IE_save;
    U1OTGIE = U1OTGIE_save;

    return TRUE;
}
