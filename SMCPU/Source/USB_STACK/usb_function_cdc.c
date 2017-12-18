#include "usb.h"
#include "usb_function_cdc.h"


volatile FAR unsigned char cdc_data_tx[CDC_DATA_IN_EP_SIZE] ;
volatile FAR unsigned char cdc_data_rx[CDC_DATA_OUT_EP_SIZE] ;

LINE_CODING line_coding ;    // Buffer to store line coding information
volatile FAR CDC_NOTICE cdc_notice ;
extern char USB_In_Buffer[64];
extern char USB_Out_Buffer[64];
#define BDT_NUM_ENTRIES      ((USB_MAX_EP_NUMBER + 1) * 4)
extern volatile BDT_ENTRY BDT[BDT_NUM_ENTRIES];
extern BYTE Track_pipe;
BYTE cdc_rx_len;            // total rx length

BYTE cdc_trf_state;         // States are defined cdc.h
char CDCSrc[64];
BYTE Buf_len;
BYTE cdc_tx_len;            // total tx length
BYTE cdc_mem_type;          // _ROM, _RAM

//USB_HANDLE CDCDataOutHandle;
//USB_HANDLE CDCDataInHandle;

BYTE CDCOutHandle;
BYTE CDCInHandle;

//CONTROL_SIGNAL_BITMAP control_signal_bitmap;


#define dummy_length    0x08
BYTE_VAL dummy_encapsulated_cmd_response[dummy_length];

extern RAM_Src IN_RAM_src;

extern BYTE IN_RAM_src_offset;


extern BYTE Track_OUT_pipe;
void USBCheckCDCRequest(void)
{
    if(SetupPkt.Recipient != USB_SETUP_RECIPIENT_INTERFACE_BITFIELD) return;

    if(SetupPkt.RequestType != USB_SETUP_TYPE_CLASS_BITFIELD) return;

    if((SetupPkt.bIntfID != CDC_COMM_INTF_ID)&&
       (SetupPkt.bIntfID != CDC_DATA_INTF_ID)) return;
    
    switch(SetupPkt.bRequest)
    {
        //****** These commands are required ******//
        case SEND_ENCAPSULATED_COMMAND:
         //send the packet
            //inPipes[0].pSrc.bRam = (BYTE*)&dummy_encapsulated_cmd_response;
            Track_pipe = 0;
            IN_RAM_src = SRC_DUMMY;
            IN_RAM_src_offset = 0;
            inPipes[0].wCount.Val = dummy_length;
            inPipes[0].info.bits.ctrl_trf_mem = USB_EP0_RAM;
            inPipes[0].info.bits.busy = 1;
            break;
        case GET_ENCAPSULATED_RESPONSE:
            // Populate dummy_encapsulated_cmd_response first.
            //inPipes[0].pSrc.bRam = (BYTE*)&dummy_encapsulated_cmd_response;
            Track_pipe = 0;
            IN_RAM_src = SRC_DUMMY;
            IN_RAM_src_offset = 0;            
            inPipes[0].info.bits.busy = 1;
            break;
        //****** End of required commands ******//

        #if defined(USB_CDC_SUPPORT_ABSTRACT_CONTROL_MANAGEMENT_CAPABILITIES_D1)
        case SET_LINE_CODING:
            outPipes[0].wCount.Val = SetupPkt.wLength;
            //outPipes[0].pDst.bRam = (BYTE*)LINE_CODING_TARGET;
            Track_OUT_pipe = 0;
            outPipes[0].info.bits.busy = 1;
            break;
            
        case GET_LINE_CODING:
            //inPipes[0].pSrc.bRam = (BYTE *)&line_coding;
            Track_pipe = 0;
            IN_RAM_src = SRC_LINE;
            IN_RAM_src_offset = 0;
            inPipes[0].wCount.Val = LINE_CODING_LENGTH;
            inPipes[0].info.Val = USB_EP0_INCLUDE_ZERO | USB_EP0_BUSY | USB_EP0_RAM;
            break;

        case SET_CONTROL_LINE_STATE:
            //control_signal_bitmap._byte = (BYTE)SetupPkt.W_Value.v[0];
            
            inPipes[0].info.bits.busy = 1;
            break;
        #endif

        default:
            break;
    }//end switch(SetupPkt.bRequest)

}//end USBCheckCDCRequest

void CDCInitEP(void)
{
   	//Abstract line coding information
   	line_coding.dwDTERate.Val = 9600;      // baud rate
   	line_coding.bCharFormat = 0x00;             // 1 stop bit
   	line_coding.bParityType = 0x00;             // None
   	line_coding.bDataBits = 0x08;               // 5,6,7,8, or 16

    cdc_rx_len = 0;
    
    USBEnableEndpoint(CDC_COMM_EP,USB_IN_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
    USBEnableEndpoint(CDC_DATA_EP,USB_IN_ENABLED|USB_OUT_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);

//    USBRxOnePacket(CDC_DATA_EP,(BYTE*)&cdc_data_rx,sizeof(cdc_data_rx));
    USBTransferOnePacket(CDC_DATA_EP,OUT_FROM_HOST,2,sizeof(cdc_data_rx));
    //CDCDataInHandle = NULL;
    CDCInHandle = NULL;
    cdc_trf_state = CDC_TX_READY;
}//end CDCInitEP


BYTE getsUSBUSART(BYTE len)
{
    cdc_rx_len = 0;
    if(!BDT[CDCOutHandle].STAT.UOWN)
    //if(!USBHandleBusy(CDCDataOutHandle))
    {
//        if(len > USBHandleGetLength(CDCDataOutHandle))
//            len = USBHandleGetLength(CDCDataOutHandle);
        
        if(len > BDT[CDCOutHandle].CNT)
            len = BDT[CDCOutHandle].CNT;
        
        for(cdc_rx_len = 0; cdc_rx_len < len; cdc_rx_len++)
            USB_Out_Buffer[cdc_rx_len] = cdc_data_rx[cdc_rx_len];


        //USBRxOnePacket(CDC_DATA_EP,(BYTE*)&cdc_data_rx,sizeof(cdc_data_rx));
        USBTransferOnePacket(CDC_DATA_EP,OUT_FROM_HOST,2,sizeof(cdc_data_rx));
    }//end if
    
    return cdc_rx_len;
    
}//end getsUSBUSART

extern char CDCSrc[64];
BYTE temp_len;
void putUSBUSART(BYTE length)
{
    USBMaskInterrupts();
    if(cdc_trf_state == CDC_TX_READY)
    {
        for(temp_len = 0; temp_len<length; temp_len++)
            CDCSrc[temp_len]= USB_In_Buffer[temp_len];

        cdc_tx_len = length;
        cdc_mem_type = USB_EP0_RAM;
        cdc_trf_state = CDC_TX_BUSY;
    }
    USBUnmaskInterrupts();
}//end putUSBUSART

BYTE byte_to_send_cdc;
BYTE i_cnt_cdc;
 
void CDCTxService(void)
{
    
    USBMaskInterrupts();
    
    if(BDT[CDCInHandle].STAT.UOWN)
    //if(USBHandleBusy(CDCDataInHandle)) 
    {
        USBUnmaskInterrupts();
        return;
    }

    if(cdc_trf_state == CDC_TX_COMPLETING)
        cdc_trf_state = CDC_TX_READY;
    
    if(cdc_trf_state == CDC_TX_READY)
    {
        USBUnmaskInterrupts();
        return;
    }
    
    if(cdc_trf_state == CDC_TX_BUSY_ZLP)
    {
//        USBTxOnePacket(CDC_DATA_EP,NULL,0);
        USBTransferOnePacket(CDC_DATA_EP,IN_TO_HOST,0,0);
        //CDC_DATA_BD_IN.CNT = 0;
        cdc_trf_state = CDC_TX_COMPLETING;
    }
    else if(cdc_trf_state == CDC_TX_BUSY)
    {
        
    	if(cdc_tx_len > sizeof(cdc_data_tx))
    	    byte_to_send_cdc = sizeof(cdc_data_tx);
    	else
    	    byte_to_send_cdc = cdc_tx_len;

        
    	cdc_tx_len = cdc_tx_len - byte_to_send_cdc;
    	  

        
        i_cnt_cdc = byte_to_send_cdc;
        Buf_len = 0;
            while(i_cnt_cdc)
            {
                cdc_data_tx[Buf_len] = CDCSrc[Buf_len];
                Buf_len++;
                i_cnt_cdc--;
            }//end while(byte_to_send._word)
        
        
        
        if(cdc_tx_len == 0)
        {
            if(byte_to_send_cdc == CDC_DATA_IN_EP_SIZE)
                cdc_trf_state = CDC_TX_BUSY_ZLP;
            else
                cdc_trf_state = CDC_TX_COMPLETING;
        }//end if(cdc_tx_len...)
//        USBTxOnePacket(CDC_DATA_EP,(BYTE*)&cdc_data_tx,byte_to_send_cdc);
        USBTransferOnePacket(CDC_DATA_EP,IN_TO_HOST,1,byte_to_send_cdc);
    }//end if(cdc_tx_sate == CDC_TX_BUSY)
    
    USBUnmaskInterrupts();
}//end CDCTxService

