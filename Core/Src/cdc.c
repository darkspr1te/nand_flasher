/*  Copyright (C) 2020 NANDO authors
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 3.
 */

#include "log.h"
#include "nand_programmer.h"
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
//#include <hw_config.h>

 
 extern USBD_HandleTypeDef hUsbDeviceFS;
#define SEND_TIMEOUT 0x1000000

static int cdc_send(uint8_t *data, uint32_t len)
{


    printf("\n\r cdc send Length %d\n\r",len);

   /* for (uint8_t x=0;x<len;x++)
    {
        printf(" 0x%x,",(uint8_t *)data[x]);
    }*/
    printf("\n\r");
    if (CDC_Transmit_FS(data,len)!=USBD_OK)
    //if (!CDC_Transmit_FS(&data,len))
    {
        ERROR_PRINT("Failed to send data\r\n");
        return -1;
    }
 
    return 0;
   
}

static int cdc_send_ready()
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
    return 0;
    //return CDC_IsPacketSent();
}

static uint32_t cdc_peek(uint8_t **data)
{
    /* uint8_t rxData[8];

 return 0;
 //   CDC_PeekRxBuffer_FS(data,1);
    uint16_t bytesAvailable = CDC_GetRxBufferBytesAvailable_FS();
    if (bytesAvailable > 0) {
    	uint16_t bytesToRead = bytesAvailable >= 8 ? 8 : bytesAvailable;
    	if (CDC_ReadRxBuffer_FS(data, bytesToRead) == USB_CDC_RX_BUFFER_OK) {
            //*data = rxData;
    	}
    }*/
    return USB_Data_Peek(data);
}

static void cdc_consume()
{
   uint8_t *data;
  // CDC_ReadRxBuffer_FS(data,1);
    USB_Data_Get(&data);

}

static np_comm_cb_t cdc_comm_cb = 
{
    .send = cdc_send,
    .send_ready = cdc_send_ready,
    .peek = cdc_peek,
    .consume = cdc_consume,
};

void cdc_init()
{
#if defined (STM32F407xx)
 np_comm_register(&cdc_comm_cb);

#else
    np_comm_register(&cdc_comm_cb);

    /* Enable receive of data */
    CDC_Receive_DATA();
#endif
}
