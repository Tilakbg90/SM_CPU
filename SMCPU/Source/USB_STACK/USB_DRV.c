#include "usb.h"
#include "usb_function_cdc.h"
#include <xc.h>


#include "comm_host.h"

char USB_In_Buffer[64];
char USB_Out_Buffer[64];




extern host_sch_info_t      		Host_Sch_Info;			/* Structure holds host comm scheduler information */
extern BOOL USB_Status;/*lint -e552 */
void ProcessIO(void);
void USBCBSendResume(void);


void USB_Process(void);

volatile long int Debug_flag = 0;
int USB_Check(void)
{   
        if(Host_Sch_Info.USB_Timeout_ms==0  || (USBDeviceState != CONFIGURED_STATE && USBDeviceState != DETACHED_STATE))
        {
            USBDeviceTasks();
            ProcessIO();
            Nop();
            if(Host_Sch_Info.USB_Timeout_ms==0)
                Host_Sch_Info.USB_Timeout_ms = USB_DELAY_TIME;
        }
        return 0;
}//end main

void Update_USB_Sch_State(void)
{
        if(Host_Sch_Info.USB_Timeout_ms==0  || (USBDeviceState != CONFIGURED_STATE && USBDeviceState != DETACHED_STATE ) || Host_Sch_Info.Tx_USB_Logged_events == 1)
        {
            USBDeviceTasks();
            USB_Process();
            if(Host_Sch_Info.USB_Timeout_ms==0)
                Host_Sch_Info.USB_Timeout_ms = USB_DELAY_TIME;
        }
}//end main
    BYTE numBytesRead = 0;
void ProcessIO(void)
{   

    numBytesRead = 0;
    if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) return;     // User Application USB tasks
    Host_Sch_Info.USB_Connection_status = 1;
    if(USBUSARTIsTxTrfReady())
    {
		numBytesRead = getsUSBUSART(64);
		if(numBytesRead != 0)
		{
			BYTE i;
	        
			for(i=0;i<numBytesRead;i++)
			{
				switch(USB_Out_Buffer[i])
				{
					case 0x0A:
					case 0x0D:
						USB_In_Buffer[i] = USB_Out_Buffer[i];
						break;
					default:
						USB_In_Buffer[i] = USB_Out_Buffer[i] + 1;
						break;
				}

			}

			putUSBUSART(numBytesRead);
		}
	}

    CDCTxService();
}		//end ProcessIO
extern BYTE Bytes_read;
void USB_Process(void)
{
    numBytesRead = 0;
    if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) 
    {
        Host_Sch_Info.USB_Connection_status = 0;
        return;
    }
    else
    {
        Host_Sch_Info.USB_Connection_status = 1;
    }
    if(USBUSARTIsTxTrfReady())
    {
		numBytesRead = getsUSBUSART(64);
		if(numBytesRead != 0)
		{
			Bytes_read = numBytesRead;
                        Host_Sch_Info.Rx_USB_Data_available = 1;
		}
                else
                {
                        Bytes_read = numBytesRead;
                        Host_Sch_Info.Rx_USB_Data_available = 0;
                }
                Update_Host_Sch_State();
                if(Host_Sch_Info.Tx_USB_Data_available && USBUSARTIsTxTrfReady())
                {
                    putUSBUSART(Host_Sch_Info.Tx_USB_Byte_Count );
                    Host_Sch_Info.Tx_USB_Data_available = 0;
                }
	}
    CDCTxService();
}		//end ProcessIO


void USBCB_SOF_Handler(void)
{
    Nop();
}



void USBCBSendResume(void)
{   
    static WORD delay_count;
    if(USBGetRemoteWakeupStatus() == TRUE) 
    {
        if(USBIsBusSuspended() == TRUE)
        {
            USBMaskInterrupts();
            
            //Clock switch to settings consistent with normal USB operation.
            USBSuspendControl = 0; 
            USBBusIsSuspended = FALSE;  //So we don't execute this code again, 
                                        //until a new suspend condition is detected.
            delay_count = 3600U;        
            do
            {
                delay_count--;
            }while(delay_count);
            
            //Now drive the resume K-state signalling onto the USB bus.
            USBResumeControl = 1;       // Start RESUME signaling
            delay_count = 1800U;        // Set RESUME line for 1-13 ms
            do
            {
                delay_count--;
            }while(delay_count);
            USBResumeControl = 0;       //Finished driving resume signalling

            USBUnmaskInterrupts();
        }
    }
}

BOOL USER_USB_CALLBACK_EVENT_HANDLER(int event)
{
    switch( event )
    {
        case EVENT_TRANSFER:
            //Add application specific callback task or callback function here if desired.
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USB_Status = USBSleepOnSuspend();
            break;
        case EVENT_RESUME:
            
            break;
        case EVENT_CONFIGURED: 
            CDCInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            //USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCheckCDCRequest();
            break;
        case EVENT_BUS_ERROR:
            break;
        case EVENT_TRANSFER_TERMINATED:
            break;
        default:
            break;
    }      
    return TRUE; 
}

