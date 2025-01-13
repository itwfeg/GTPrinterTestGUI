#ifndef GLOBAL_USB_TASK_H
#define GLOBAL_USB_TASK_H 
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#include "usb.h"
#include "usb_device_class.h"
#include "compositeHobart.h"

/* image to print transfer manager */
typedef struct
{       
     unsigned long      labelImageSize;         /* label image size in bytes */  
     unsigned long      frameTotalRcvd;         /* total usb frames received */       
     unsigned long      frameTotal;             /* number of usb frames to 
                                                complete transfer of label image */
     bool               fitInImageBfr;          /* will the label image size fit 
                                                   into our image buffer */
     unsigned short     numImageBfrs;           /* number of image buffers to 
                                                complete the printed image */
}ITransMgr;

#define USB_DEVICE_INTERRUPT_PRI                                3U
#define CONTROLLER_ID                                           kUSB_ControllerEhci0


#define usb_task_PRIORITY ( configMAX_PRIORITIES - 1 )

/* prototypes */
BaseType_t createUsbTask( QueueHandle_t prSendQueue, QueueHandle_t wrSendQueue );
void USB_OTG1_IRQHandler( void );
void USB_DeviceClockInit( void );
void USB_DeviceIsrEnable( void );
void usbTask( void *parameter );
void setTotalLabelSize( unsigned long size );
unsigned long getPacketTransferTotal( void );

/* private */
static bool initUSB( void );
static usb_status_t USB_DeviceCallback( usb_device_handle handle, uint32_t event, void *param );
static usb_status_t USB_DeviceAppCallback( class_handle_t classHandle, uint32_t event, void *param );

usb_status_t USB_DevicePrinterInit( usb_device_composite_struct_t *deviceComposite );
usb_status_t USB_DevicePrinterSetConfigure(class_handle_t handle, uint8_t configure);
usb_status_t USB_DevicePrinterSetInterface(class_handle_t handle, uint8_t interface, uint8_t alternateSetting);
usb_status_t USB_DeviceWeigherInit(usb_device_composite_struct_t *deviceComposite);
usb_status_t USB_DeviceWeigherSetConfigure(class_handle_t handle, uint8_t configure);
usb_status_t USB_DeviceWeigherSetInterface(class_handle_t handle, uint8_t interface, uint8_t alternateSetting);
#endif 
