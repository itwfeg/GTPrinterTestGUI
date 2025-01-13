#include "usbTask.h"
#include "usbHobartPrinter.h"
#include "usbHobartWeigher.h"
#include "queueManager.h"
#include "translator.h"
#include "printHead.h"
#include "usb_device_config.h"
#include "usb_device.h"
#include "usb_device_class.h"
#include "usbPrinter.h"
#include "usbWeigher.h"
#include "usb_device_ch9.h"
#include "usb_device_descriptor.h"
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "usb_phy.h"

static TaskHandle_t     pUsbHandle_             = NULL;
static QueueHandle_t    pMsgInPrQHandle_        = NULL;
static QueueHandle_t    pMsgInWrQHandle_        = NULL;

static QueueHandle_t    pMsgOutPrQHandle_       = NULL;
static QueueHandle_t    pMsgOutWrQHandle_       = NULL;

QueueSetHandle_t        uQueueSet_              = NULL;

static bool             suspend_                = false;
static ITransMgr        iTfansMgr_;

extern usb_device_class_struct_t g_UsbDevicePrinterConfig;
extern usb_device_class_struct_t g_UsbDeviceWeigherConfig;

usb_device_composite_struct_t usbMgr;
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) uint8_t s_PrinterRcvBuffer[ FS_PRINTER_INTERRUPT_OUT_PACKET_SIZE ];
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) uint8_t s_PrinterSendBuffer[ FS_PRINTER_INTERRUPT_IN_PACKET_SIZE ];
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) uint8_t s_imageRcvBuffer[FS_PRINTER_BULK_OUT_PACKET_SIZE ];

USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) uint8_t s_WeigherRcvBuffer[ FS_WEIGHER_INTERRUPT_OUT_PACKET_SIZE ];
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) uint8_t s_WeigherSendBuffer[ FS_WEIGHER_INTERRUPT_IN_PACKET_SIZE ];

/* printer class specifice transfer buffer 
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_PrinterClassBuffer[ 64 ];*/

usb_device_class_config_struct_t g_CompositeClassConfig[USB_COMPOSITE_INTERFACE_COUNT] = {
    {
        USB_DevicePrinterCallback, /* Printer class callback pointer */
        (class_handle_t)NULL,          /* The hobart printer class handle, This field is set by USB_DeviceClassInit */
        &g_UsbDevicePrinterConfig, /* The hobart printer keyboard configuration, including class code, subcode, and protocol,
                                  class
                                  type, transfer type, endpoint address, max packet size, etc.*/
    },
    {
        USB_DeviceWeigherCallback, /* Weigher class callback pointer */
        (class_handle_t)NULL,       /* The hobart Weigher class handle, This field is set by USB_DeviceClassInit */
        &g_UsbDeviceWeigherConfig, /* The hobart  weigher configuration, including class code, subcode, and protocol, class
                               type,
                               transfer type, endpoint address, max packet size, etc.*/
    }};

/* Set class configuration list */
usb_device_class_config_list_struct_t usbMgrConfigList = {
    g_CompositeClassConfig,        /* Class configurations */
    USB_DeviceCallback,            /* Device callback pointer */
    2,                             /* Class count */
};

/******************************************************************************/
/*!   \fn BaseType_t createUsbTask( QueueHandle_t prSendQueue, QueueHandle_t wrSendQueue )

      \brief
        This function initializes resources and spawns the usb task.
   
      \author
          Aaron Swift
*******************************************************************************/
BaseType_t createUsbTask( QueueHandle_t prSendQueue, QueueHandle_t wrSendQueue )
{
    BaseType_t result = pdFAIL;
    
    /* clear our image transfer manager */
    memset( &iTfansMgr_, 0, sizeof( ITransMgr ) ); 
    
    /* get translator queues */ 
    pMsgOutPrQHandle_ = getUsbOutPrQueueHandle();
    pMsgOutWrQHandle_ = getUsbOutWrQueueHandle();
    
    /* create queue set for printer and weigher messages */
    if( ( prSendQueue != NULL ) && ( wrSendQueue != NULL ) ) { 
        /* pipes are based on host perspective */
        pMsgInPrQHandle_  = prSendQueue;
        pMsgInWrQHandle_  = wrSendQueue;


        /* determine length of all message queues combined */
        unsigned long wQSetLength = ( getUsbInQueueLength() + getUsbInQueueLength() );
        
        /* create local queue set */
        uQueueSet_ = xQueueCreateSet( wQSetLength );

        if( ( pMsgInPrQHandle_ != NULL ) && ( pMsgInWrQHandle_ != NULL ) ) {
            /* add queues to local set */
            xQueueAddToSet( pMsgInPrQHandle_, uQueueSet_ );
            xQueueAddToSet( pMsgInWrQHandle_, uQueueSet_ );
        } else {       
            PRINTF("createUsbTask(): Critical Error queue set not created!\r\n" );
        }
               
        if( initUSB() ) {
            /* create printer task thread */
            result = xTaskCreate( usbTask,  "UsbTask", 5000L / sizeof(portSTACK_TYPE),
                                                usbMgr.deviceHandle, 5U, &pUsbHandle_ );    
        } else {
            PRINTF("createUsbTask(): Failed to start task!\r\n" );
        }
    } else {
            PRINTF("createUsbTask(): Queues Null!\r\n" );    
    }
    return result;
}

/******************************************************************************/
/*!   \fn static bool initUSB( void )

      \brief
        This function initializes the hobart composite driver and usb stack.         
   
      \author
          Aaron Swift
*******************************************************************************/
static bool initUSB( void )
{
    bool result = false;
    
    USB_DeviceClockInit();
    /* set composite device to default state */
    usbMgr.speed                        = USB_SPEED_FULL;
    usbMgr.attach                       = 0U;
    usbMgr.printer.deviceHandle         = NULL;
    usbMgr.weigher.deviceHandle         = NULL;
    usbMgr.applicationTaskHandle        = (TaskHandle_t)usbTask;
    usbMgr.deviceTaskHandle             = (TaskHandle_t)USB_DeviceCallback;
    usbMgr.currentConfiguration         = 0;            
    usbMgr.deviceHandle                 = NULL;
   

    /* initialize the usb stack and class drivers */
    if( kStatus_USB_Success == USB_DeviceClassInit( kUSB_ControllerEhci0, 
                                                    &usbMgrConfigList, 
                                                    &usbMgr.deviceHandle) ) {
        
        /* get the class handle */
        usbMgr.printer.deviceHandle = usbMgrConfigList.config[0].classHandle;
        usbMgr.weigher.deviceHandle = usbMgrConfigList.config[1].classHandle;
        
        USB_DevicePrinterInit( &usbMgr );
        USB_DeviceWeigherInit( &usbMgr );
        /* install isr, set priority, and enable IRQ. */
        USB_DeviceIsrEnable();
        
        /*add one delay here to make the DP pull down long enough to allow host to detect the previous disconnection.*/
        SDK_DelayAtLeastUs( 5000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY );
        USB_DeviceRun( usbMgr.deviceHandle );
        result = true;
    } 
    return result;
}

/******************************************************************************/
/*!   \fn void usbTask( void *parameter )

      \brief
        This function handles sending queue'd weigher / printer messages 
        to the device's endpoints.
   
      \author
          Aaron Swift
*******************************************************************************/
void usbTask( void *parameter )
{
    usb_device_composite_struct_t *pUsbMgr      = (usb_device_composite_struct_t *)&usbMgr;
    usb_status_t status                         = kStatus_USB_Error;
    static uint32_t prMsgSize = 0, wrMsgSize = 0, prMsgCnt = 0, wrMsgCnt = 0;
    static bool prHdr = true, wrHdr = true; 
    static uint8_t prevMsgType_ = 0;
    static bool wgFlag_ = false;      
    QueueSetMemberHandle_t setHandle = NULL;
    
    
    PRINTF("usbTask(): Task starting.......\r\n" );
    while( !suspend_ ) {
        if( pUsbMgr->attach ) {
           
            /* wait for object in one of the queues within the set portMAX_DELAY*/  
            setHandle = xQueueSelectFromSet( uQueueSet_,  portMAX_DELAY ); 
            if( setHandle == pMsgInPrQHandle_ ) {                      
                /* how many transactions are queued? */
                prMsgCnt = uxQueueMessagesWaiting( setHandle );
                /* do not process until we have a head and body */
                if( prMsgCnt  ) {
                    /* only process one message at a time */
					
#if 0	//removed due to issues with odd number of total transfers 										
                    wrMsgCnt = 2;
#endif					
                    while( prMsgCnt ) {
                        if( xQueueReceive( pMsgInPrQHandle_, (void *)&s_PrinterSendBuffer[0], portMAX_DELAY ) ) {
                            if( prHdr ) {
                                if( ( s_PrinterSendBuffer[0] != 0x46 ) || ( s_PrinterSendBuffer[1] != 0x46 ) ) {
                                    //PRINTF("pr$\r\n");
                                    xQueueReset( pMsgInPrQHandle_ );
                                
                                    resetPrinter();
                                    postRequestStatus();
                                    
                                    asm("nop");
                                }
                                /* get the body size for next transfer */
                                prMsgSize = s_PrinterSendBuffer[8];
                                prMsgSize <<= 8;
                                prMsgSize |= s_PrinterSendBuffer[9];
                                                                                               
                                status = USB_DeviceHBPrinterSend( pUsbMgr->printer.deviceHandle, USB_PRINTER_INTERRUPT_ENDPOINT_IN,
                                                                  &s_PrinterSendBuffer[0], 10 ); 
                               
                                /* keep trying until sent */
                                while( ( status != kStatus_USB_Success )  ) {
                                    taskYIELD(); 
                                    status = USB_DeviceHBPrinterSend( pUsbMgr->printer.deviceHandle, USB_PRINTER_INTERRUPT_ENDPOINT_IN,
                                                                      &s_PrinterSendBuffer[0], 10 ); 
                                }
                                
                                prHdr = false;                                                        
                            } else {
                                if( ( s_PrinterSendBuffer[0] == 0x46 ) || ( s_PrinterSendBuffer[1] == 0x46 ) ) {
                                    //PRINTF("pr~\r\n");
                                    
                                    
                                    xQueueReceive( pMsgInPrQHandle_, (void *)&s_PrinterSendBuffer[0], portMAX_DELAY );
                                    prMsgSize = 0x0e;
                                    xQueueReset( pMsgInPrQHandle_ );
                                    
                                    resetPrinter();
                                    postRequestStatus();
                                }

                                if( prMsgSize < FS_PRINTER_INTERRUPT_IN_PACKET_SIZE ) {  
                                        
                                    status = USB_DeviceHBPrinterSend( pUsbMgr->printer.deviceHandle, USB_PRINTER_INTERRUPT_ENDPOINT_IN,
                                                                      &s_PrinterSendBuffer[0], prMsgSize );                        
                                    /* keep trying until sent */
                                    while( status != kStatus_USB_Success )   {
                                        taskYIELD();
                                        status = USB_DeviceHBPrinterSend( pUsbMgr->printer.deviceHandle, USB_PRINTER_INTERRUPT_ENDPOINT_IN,
                                                                          &s_PrinterSendBuffer[0], prMsgSize );                        

                                    }                                   

                                    /* entire message sent */
                                    prHdr = true;
                                    prMsgSize = 0;
									break;
                                } else {                                
                                    status = USB_DeviceHBPrinterSend( pUsbMgr->printer.deviceHandle, USB_PRINTER_INTERRUPT_ENDPOINT_IN,
                                                                      &s_PrinterSendBuffer[0], FS_PRINTER_INTERRUPT_IN_PACKET_SIZE );                        
                                    /* keep trying until sent */
                                    while( status != kStatus_USB_Success ) {
                                        taskYIELD();
                                        status = USB_DeviceHBPrinterSend( pUsbMgr->printer.deviceHandle, USB_PRINTER_INTERRUPT_ENDPOINT_IN,
                                                                          &s_PrinterSendBuffer[0], FS_PRINTER_INTERRUPT_IN_PACKET_SIZE );                        
         
                                    }
                                    prMsgSize -= FS_PRINTER_INTERRUPT_IN_PACKET_SIZE; 
                                }             
                            }
                            prMsgCnt--;
                        } else {
                            PRINTF("usbTask(): failed to unqueue message!\r\n");
                        }
                    }
                }
            } else if( setHandle == pMsgInWrQHandle_ ) {
                /* how many transactions are queued? */
                wrMsgCnt = uxQueueMessagesWaiting( setHandle );
                /* do not process until we have a head and body */
                if( wrMsgCnt  ) {
                    /* only process one message at a time */

#if 0	//removed due to issues with odd number of total transfers 										
                    wrMsgCnt = 2;
#endif
					
                    while( wrMsgCnt ) {
                        if( xQueueReceive( pMsgInWrQHandle_, (void *)&s_WeigherSendBuffer[0], portMAX_DELAY ) ) {
                            if( wrHdr ) {

                                if( ( s_WeigherSendBuffer[4] != 0x46 ) || ( s_WeigherSendBuffer[5] != 0x46 ) ||
                                    ( s_WeigherSendBuffer[7] != 0x06 ) ) {
                                    PRINTF("Wg$\r\n");
                                    PRINTF("Wg s_WeigherSendBuffer[0] %d\r\n", s_WeigherSendBuffer[0] );
                                    PRINTF("Wg s_WeigherSendBuffer[1] %d\r\n", s_WeigherSendBuffer[1] );
                                    PRINTF("Wg s_WeigherSendBuffer[2] %d\r\n", s_WeigherSendBuffer[2] );
                                    PRINTF("Wg s_WeigherSendBuffer[3] %d\r\n", s_WeigherSendBuffer[3] );
                                    PRINTF("Wg s_WeigherSendBuffer[4] %d\r\n", s_WeigherSendBuffer[4] );
                                    PRINTF("Wg s_WeigherSendBuffer[5] %d\r\n", s_WeigherSendBuffer[5] );
                                    PRINTF("Wg s_WeigherSendBuffer[6] %d\r\n", s_WeigherSendBuffer[6] );
                                    PRINTF("Wg s_WeigherSendBuffer[7] %d\r\n", s_WeigherSendBuffer[7] );
                                    PRINTF("Wg s_WeigherSendBuffer[8] %d\r\n", s_WeigherSendBuffer[8] );
                                    PRINTF("Wg s_WeigherSendBuffer[9] %d\r\n", s_WeigherSendBuffer[9] );
                                    wgFlag_ = true;
                                    //xQueueReset( pMsgInWrQHandle_ );                                   
                                    //asm("nop");
                                }

                                /* get the body size for next transfer */
                                wrMsgSize = s_WeigherSendBuffer[8];
                                wrMsgSize <<= 8;
                                wrMsgSize |= s_WeigherSendBuffer[9];

                                status = USB_DeviceHBWeigherSend( pUsbMgr->weigher.deviceHandle, USB_WEIGHER_INTERRUPT_ENDPOINT_IN,
                                                                  &s_WeigherSendBuffer[0], 10 ); 
                                /* keep trying until sent */
                                while( ( status != kStatus_USB_Success ) ) {
                                    taskYIELD();
                                    status = USB_DeviceHBWeigherSend( pUsbMgr->weigher.deviceHandle, USB_WEIGHER_INTERRUPT_ENDPOINT_IN,
                                                                      &s_WeigherSendBuffer[0], 10 );
                                }
                                wrHdr = false;                        
                            } else {
                                if( wgFlag_ ) {
                                    PRINTF("wg~\r\n");
                                    PRINTF("Wg s_WeigherSendBuffer[0] %d\r\n", s_WeigherSendBuffer[0] );
                                    PRINTF("Wg s_WeigherSendBuffer[1] %d\r\n", s_WeigherSendBuffer[1] );
                                    PRINTF("Wg s_WeigherSendBuffer[2] %d\r\n", s_WeigherSendBuffer[2] );
                                    PRINTF("Wg s_WeigherSendBuffer[3] %d\r\n", s_WeigherSendBuffer[3] );
                                    
                                    wgFlag_ = false;
                                    
                                    //xQueueReceive( pMsgInWrQHandle_, (void *)&s_WeigherSendBuffer[0], portMAX_DELAY );
                                    //xQueueReset( pMsgInWrQHandle_ );
                                }

                                if( wrMsgSize < FS_WEIGHER_INTERRUPT_IN_PACKET_SIZE ) {
                                    status = USB_DeviceHBWeigherSend( pUsbMgr->weigher.deviceHandle, USB_WEIGHER_INTERRUPT_ENDPOINT_IN,
                                                                        &s_WeigherSendBuffer[0], wrMsgSize ); 
                                    /* keep trying until sent */
                                    while( status != kStatus_USB_Success ) {
                                        taskYIELD();
                                        status = USB_DeviceHBWeigherSend( pUsbMgr->weigher.deviceHandle, USB_WEIGHER_INTERRUPT_ENDPOINT_IN,
                                                                          &s_WeigherSendBuffer[0], wrMsgSize ); 
                                    }
                                    /* entire message sent */
                                    wrHdr = true;
                                    wrMsgSize = 0;
									break;
                                } else {                          
                                    status = USB_DeviceHBWeigherSend( pUsbMgr->weigher.deviceHandle, USB_WEIGHER_INTERRUPT_ENDPOINT_IN,
                                                                      &s_WeigherSendBuffer[0], FS_WEIGHER_INTERRUPT_IN_PACKET_SIZE ); 
                                    /* keep trying until sent */
                                    while( status != kStatus_USB_Success ) {
                                        taskYIELD();
                                        status = USB_DeviceHBWeigherSend( pUsbMgr->weigher.deviceHandle, USB_WEIGHER_INTERRUPT_ENDPOINT_IN,
                                                                          &s_WeigherSendBuffer[0], FS_WEIGHER_INTERRUPT_IN_PACKET_SIZE );                             
                                    }
                                    wrMsgSize -=  FS_WEIGHER_INTERRUPT_IN_PACKET_SIZE;                           
                                }
                            }
                            wrMsgCnt--;
                        }
                    }
                }
            } else {
                PRINTF("usbTask(): invalid select from queue set! %d  \r\n", setHandle );                                       
            }
        }    
        taskYIELD();
    }
    vTaskSuspend(NULL);
}

void USB_OTG1_IRQHandler(void)
{
    USB_DeviceEhciIsrFunction( usbMgr.deviceHandle );
}

/******************************************************************************/
/*!   \fn void USB_DeviceClockInit(void)

      \brief
        This function initializes the USB clock.          
   
      \author
          Aaron Swift
*******************************************************************************/
void USB_DeviceClockInit(void)
{
    usb_phy_config_struct_t phyConfig = {
        BOARD_USB_PHY_D_CAL,
        BOARD_USB_PHY_TXCAL45DP,
        BOARD_USB_PHY_TXCAL45DM,
    };

    CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_Usbphy480M, 480000000U);
    CLOCK_EnableUsbhs0Clock(kCLOCK_Usb480M, 480000000U);
    USB_EhciPhyInit(CONTROLLER_ID, BOARD_XTAL0_CLK_HZ, &phyConfig);
}

/******************************************************************************/
/*!   \fn void USB_DeviceIsrEnable(void)

      \brief
        This function initializes the USB clock.          
   
      \author
          Aaron Swift
*******************************************************************************/
void USB_DeviceIsrEnable(void)
{
    uint8_t irqNumber;

    uint8_t usbDeviceEhciIrq[] = USBHS_IRQS;
    irqNumber                  = usbDeviceEhciIrq[CONTROLLER_ID - kUSB_ControllerEhci0];

    /* Install isr, set priority, and enable IRQ. */
    NVIC_SetPriority((IRQn_Type)irqNumber, USB_DEVICE_INTERRUPT_PRIORITY);
    EnableIRQ((IRQn_Type)irqNumber);
}

/******************************************************************************/
/*!   \fn void setTotalLabelSize( unsigned long size )

      \brief
        This function sets the label image size to be transfered.        
   
      \author
          Aaron Swift
*******************************************************************************/
void setTotalLabelSize( unsigned long size )
{
    iTfansMgr_.labelImageSize = size;    
}

/******************************************************************************/
/*!   \fn unsigned long getPacketTransferTotal( void )

      \brief
        This function returns the total bytes of label image received.        
   
      \author
          Aaron Swift
*******************************************************************************/
unsigned long getPacketTransferTotal( void )
{
    return iTfansMgr_.frameTotalRcvd;    
}


/******************************************************************************/
/*!   \fn unsigned long getPacketTransferTotal( void )

      \brief
        This function initializes the hobart composite printer.        
   
      \author
          Aaron Swift
*******************************************************************************/
usb_status_t USB_DevicePrinterInit( usb_device_composite_struct_t *deviceComposite )
{
    usb_status_t error = kStatus_USB_Error;
    if(  deviceComposite->printer.deviceHandle != NULL ) { 
      deviceComposite->printer.interruptOutEp             = 0;
      deviceComposite->printer.interruptInPipeDataBuffer  = &s_PrinterSendBuffer[0];
      deviceComposite->printer.interruptInPipeDataLen     = FS_PRINTER_INTERRUPT_IN_PACKET_SIZE;
      deviceComposite->printer.interruptOutPipeDataBuffer = &s_PrinterRcvBuffer[0];
      deviceComposite->printer.interruptOutPipeDataLen    = FS_PRINTER_INTERRUPT_OUT_PACKET_SIZE;
      deviceComposite->printer.interruptInPipeBusy        = false;
      deviceComposite->printer.interruptOutPipeBusy       = false;
      deviceComposite->printer.interruptInPipeStall       = false;
      deviceComposite->printer.interruptOutPipeStall      = false;
      deviceComposite->printer.msgLength                  = 0;
      deviceComposite->printer.bulkOutPipeDataBuffer      = &s_imageRcvBuffer[0];
      deviceComposite->printer.bulkOutPipeDataLen         = FS_PRINTER_BULK_OUT_PACKET_SIZE;
      deviceComposite->printer.bulkOutEp                  = 0;
      deviceComposite->printer.bulkOutPipeBusy            = false;
      deviceComposite->printer.bulkOutPipeStall           = false; 
            
      error = kStatus_USB_Success;    
    }
    return error;
}

/******************************************************************************/
/*!   \fn usb_status_t USB_DevicePrinterSetConfigure(class_handle_t handle, uint8_t configure)

      \brief
        This function schedules the receive buffers for interrupt and bulk out
        endpoints of teh printer.        
   
      \author
          Aaron Swift
*******************************************************************************/
usb_status_t USB_DevicePrinterSetConfigure(class_handle_t handle, uint8_t configure)
{
    if( USB_COMPOSITE_CONFIGURE_INDEX == configure ) {
        usbMgr.printer.attach = 1;
        
        usbMgr.printer.interruptOutEp = USB_PRINTER_INTERRUPT_ENDPOINT_OUT;
          
        /* schedule interrupt buffer for receive */
        USB_DeviceHBPrinterRecv( usbMgr.printer.deviceHandle, USB_PRINTER_INTERRUPT_ENDPOINT_OUT, &s_PrinterRcvBuffer[0],
                             FS_PRINTER_INTERRUPT_OUT_PACKET_SIZE);
        
        usbMgr.printer.bulkOutEp = USB_PRINTER_BULK_ENDPOINT_OUT;
        
        /* schedule bulk buffer for receive */
        USB_DeviceHBPrinterRecv( usbMgr.printer.deviceHandle, USB_PRINTER_BULK_ENDPOINT_OUT, &s_imageRcvBuffer[0],
                             FS_PRINTER_BULK_OUT_PACKET_SIZE);

    }
    return kStatus_USB_Success;  
}

usb_status_t USB_DevicePrinterSetInterface(class_handle_t handle, uint8_t interface, uint8_t alternateSetting)
{
    usb_status_t error = kStatus_USB_Error;
    if( usbMgr.printer.attach ) {
    
    }
    return error;
}

/******************************************************************************/
/*!   \fn usb_status_t USB_DeviceWeigherInit(usb_device_composite_struct_t *deviceComposite)

      \brief
        This function initializes the hobart composite weigher.
          
      \author
          Aaron Swift
*******************************************************************************/
usb_status_t USB_DeviceWeigherInit(usb_device_composite_struct_t *deviceComposite)
{
    usb_status_t error = kStatus_USB_Error;
    if(  deviceComposite->weigher.deviceHandle != NULL ) { 
      deviceComposite->weigher.interruptInPipeDataBuffer  = &s_WeigherSendBuffer[0];
      deviceComposite->weigher.interruptInPipeDataLen     = FS_WEIGHER_INTERRUPT_IN_PACKET_SIZE;
      deviceComposite->weigher.interruptOutPipeDataBuffer = &s_WeigherRcvBuffer[0];
      deviceComposite->weigher.interruptOutPipeDataLen    = FS_WEIGHER_INTERRUPT_OUT_PACKET_SIZE;
      deviceComposite->weigher.interruptInPipeBusy        = false;
      deviceComposite->weigher.interruptOutPipeBusy       = false;
      deviceComposite->weigher.interruptInPipeStall       = false;
      deviceComposite->weigher.interruptOutPipeStall      = false;
      error = kStatus_USB_Success;    
    }
    return error;
}

/******************************************************************************/
/*!   \fn usb_status_t USB_DeviceWeigherSetConfigure(class_handle_t handle, uint8_t configure)

      \brief
        This function schedules the receive buffer for interrupt out
        endpoint of the weigher.        
   
      \author
          Aaron Swift
*******************************************************************************/
usb_status_t USB_DeviceWeigherSetConfigure(class_handle_t handle, uint8_t configure)
{
    if( USB_COMPOSITE_CONFIGURE_INDEX == configure ) {
        usbMgr.weigher.attach = 1;
        /* schedule buffer for receive */
        USB_DeviceHBWeigherRecv( usbMgr.weigher.deviceHandle, USB_WEIGHER_INTERRUPT_ENDPOINT_OUT, &s_WeigherRcvBuffer[0],
                             FS_WEIGHER_INTERRUPT_OUT_PACKET_SIZE);
    }
    return kStatus_USB_Success;    
}

usb_status_t USB_DeviceWeigherSetInterface(class_handle_t handle, uint8_t interface, uint8_t alternateSetting)
{
    usb_status_t error = kStatus_USB_Error;
    if( usbMgr.weigher.attach ) {
    
    }
    return error;
}

/******************************************************************************/
/*!   \fn usb_status_t USB_DeviceWeigherSetConfigure(class_handle_t handle, uint8_t configure)

      \brief
        This function handles control pipe events for the hobart composite device.       
   
      \author
          Aaron Swift
*******************************************************************************/
static usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint16_t *temp16   = (uint16_t *)param;
    uint8_t *temp8     = (uint8_t *)param;

    switch (event)
    {
        case kUSB_DeviceEventBusReset: {
            /* USB bus reset signal detected */
            usbMgr.attach               = 0U;
            usbMgr.currentConfiguration = 0U;
            error = kStatus_USB_Success;
#if (defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)) || \
    (defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U))
            /* get USB speed to configure the device, including max packet size and interval of the endpoints. */
            if( kStatus_USB_Success == USB_DeviceClassGetSpeed( CONTROLLER_ID, &usbMgr.speed ) ) {
                USB_DeviceSetSpeed(handle, usbMgr.speed);
            }
#endif
        }
        break;
        case kUSB_DeviceEventSetConfiguration: {
            if( 0U == (*temp8) ) {
                usbMgr.attach               = 0U;
                usbMgr.currentConfiguration = 0U;
                error = kStatus_USB_Success;
                
            } else if( USB_COMPOSITE_CONFIGURE_INDEX == (*temp8) ) {
                /* set device configuration request */
                usbMgr.attach = 1U;
                usbMgr.currentConfiguration = *temp8;              
                USB_DevicePrinterSetConfigure( usbMgr.printer.deviceHandle, *temp8 );
                USB_DeviceWeigherSetConfigure( usbMgr.weigher.deviceHandle, *temp8 );
                
                error = kStatus_USB_Success;
            } else {
                /* no action, return kStatus_USB_InvalidRequest. */
            }
        }       
        break;
        case kUSB_DeviceEventSetInterface: {
            if( usbMgr.attach ) {
                /* set device interface request */
                uint8_t interface        = (uint8_t)((*temp16 & 0xFF00U) >> 0x08U);
                uint8_t alternateSetting = (uint8_t)(*temp16 & 0x00FFU);
                
                if( USB_PRINTER_INTERFACE_INDEX == interface ) {
                    if( alternateSetting < USB_PRINTER_INTERFACE_ALTERNATE_COUNT ) {
                        usbMgr.currentInterfaceAlternateSetting[interface] = alternateSetting;                        
                        error = kStatus_USB_Success;
                    }
                } else if( USB_WEIGHER_INTERFACE_INDEX == interface ) {
                    if( alternateSetting < USB_WEIGHER_INTERFACE_ALTERNATE_COUNT ) {
                        usbMgr.currentInterfaceAlternateSetting[interface] = alternateSetting;                                               
                        error = kStatus_USB_Success;
                    }
                } else {
                    /* no action, return kStatus_USB_InvalidRequest. */
                }
            }
        }
        break;
        case kUSB_DeviceEventGetConfiguration: {
            if( param ) {
                /* get current configuration request */
                *temp8 = usbMgr.currentConfiguration;
                error  = kStatus_USB_Success;
            }
            break;
        }
        case kUSB_DeviceEventGetInterface: {
            if( param ) {
                /* get current alternate setting of the interface request */
                uint8_t interface = (uint8_t)((*temp16 & 0xFF00U) >> 0x08U);
                if( interface < USB_COMPOSITE_INTERFACE_COUNT ){
                    *temp16 = (*temp16 & 0xFF00U) | usbMgr.currentInterfaceAlternateSetting[interface];
                    error   = kStatus_USB_Success;
                }
            }
            break;
        }
        case kUSB_DeviceEventGetDeviceDescriptor: {
            if( param ) {
                /* get device descriptor request */
                error = USB_DeviceGetDeviceDescriptor( handle, (usb_device_get_device_descriptor_struct_t *)param );
            }
            break;
        }
        case kUSB_DeviceEventGetConfigurationDescriptor: {
            if( param ) {
                /* get device configuration descriptor request */
                error = USB_DeviceGetConfigurationDescriptor( handle,
                                                             (usb_device_get_configuration_descriptor_struct_t *)param );
            }
            break;
        }
        case kUSB_DeviceEventGetStringDescriptor: {
            if( param ) {
                /* get device string descriptor request */
                error = USB_DeviceGetStringDescriptor( handle, (usb_device_get_string_descriptor_struct_t *)param );
            }
            break;
        }
        default:
            break;
    }

    return error;
}

/******************************************************************************/
/*!   \fn usb_status_t USB_DevicePrinterCallback( class_handle_t handle, 
                                                 uint32_t event, void *param )

      \brief
        This function handles control pipe events for the hobart composite device.       
   
      \param handle          The hobart printer class handle.
      \param event           The hobart printer event type.
      \param param           The parameter of the class specific request.

      \author
          Aaron Swift
*******************************************************************************/
usb_status_t USB_DevicePrinterCallback( class_handle_t handle, uint32_t event, void *param )
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    usb_device_endpoint_callback_message_struct_t *epCbParam;

    switch (event)
    {
      case kUSB_DevicePrinterEventSendResponse: {
        error = kStatus_USB_Success;
        usbMgr.printer.interruptInPipeBusy = 0;
        break;
      }        
      case kUSB_DevicePrinterEventRecvResponse: {
          if( usbMgr.printer.attach ) {       
              usbMgr.printer.msgLength =  epCbParam->length; 
            
              if( usbMgr.printer.msgLength != 0 ) {
                /* queue the received frame */
                BaseType_t xHigherPriorityTaskWoken = false;
                
                /* post to the translator queue */
                BaseType_t result = xQueueSendToBackFromISR( pMsgOutPrQHandle_, (void *)&s_PrinterRcvBuffer[0], &xHigherPriorityTaskWoken );  
                if( result != pdPASS ) {
                    PRINTF("USB_DevicePrinterCallback: Failed to queue rx transaction!\r\n" );
                }
              } else {
                  PRINTF("USB_DevicePrinterCallback: transaction lenght 0!\r\n" );
              }
              
              error = USB_DeviceHBPrinterRecv( usbMgr.printer.deviceHandle, USB_PRINTER_INTERRUPT_ENDPOINT_OUT, &s_PrinterRcvBuffer[0],
              FS_PRINTER_INTERRUPT_OUT_PACKET_SIZE ); 
              if( error != kStatus_USB_Success ) {
                  PRINTF("USB_DevicePrinterCallback: error: %d \r\n", error );  
              }
          }
          break;
      }
      case kUSB_DevicePrinterEventRecvBulkResponse: {
        
            /* copy label image to print engine image buffer */
            copyBulkToImageBfr( &s_imageRcvBuffer[0], epCbParam->length );
            
            error = USB_DeviceHBPrinterRecv( usbMgr.printer.deviceHandle, USB_PRINTER_BULK_ENDPOINT_OUT, &s_imageRcvBuffer[0],
            FS_PRINTER_BULK_OUT_PACKET_SIZE ); 
        
          break;
      }
      default: {
          PRINTF("USB_DevicePrinterCallback: default: \r\n");  
          error = USB_DeviceHBPrinterRecv( usbMgr.printer.deviceHandle, USB_PRINTER_INTERRUPT_ENDPOINT_OUT, &s_PrinterRcvBuffer[0],
          FS_PRINTER_INTERRUPT_OUT_PACKET_SIZE ); 
        break;
      }
    }
    return error;        
}


/******************************************************************************/
/*!   \fn usb_status_t USB_DeviceWeigherCallback( class_handle_t handle, 
                                                 uint32_t event, void *param )

      \brief
        This function handles control pipe events for the hobart composite device.       
   
      \param handle          The hobart weigher class handle.
      \param event           The hobart weigher event type.
      \param param           The parameter of the class specific request.

      \author
          Aaron Swift
*******************************************************************************/
usb_status_t USB_DeviceWeigherCallback(class_handle_t handle, uint32_t event, void *param)
{
    usb_device_endpoint_callback_message_struct_t *epCbParam;
    usb_status_t error = kStatus_USB_InvalidRequest;
    switch (event)
    {
      case kUSB_DeviceWeigherEventSendResponse: {
          error = kStatus_USB_Success;
          usbMgr.weigher.interruptInPipeBusy = 0;
          break;              
      }

      case kUSB_DeviceWeigherEventRecvResponse: {
        if( usbMgr.weigher.attach ) {
            usbMgr.weigher.msgLength =  epCbParam->length;             
            if( usbMgr.weigher.msgLength != 0 ) {
                /* queue the received frame */
                BaseType_t xHigherPriorityTaskWoken = false;
                BaseType_t result = xQueueSendToBackFromISR( pMsgOutWrQHandle_, (void *)&s_WeigherRcvBuffer[0], &xHigherPriorityTaskWoken );                         
                if( result != pdPASS ) {
                    PRINTF("USB_DeviceWeigherCallback: Failed to queue rx transaction!\r\n" );
                }
            } else {
                PRINTF("USB_DeviceWeigherCallback: transaction lenght 0!\r\n" );
            }         
            error = USB_DeviceHBWeigherRecv(handle, USB_WEIGHER_INTERRUPT_ENDPOINT_OUT, &s_WeigherRcvBuffer[0],
                                                 FS_WEIGHER_INTERRUPT_OUT_PACKET_SIZE ); 
            if( error != kStatus_USB_Success ) {
                PRINTF("USB_DevicePrinterCallback: error: %d\r\n", error );  
            }            
        }
        break;
      }
      default: {
          error = USB_DeviceHBWeigherRecv(handle, USB_WEIGHER_INTERRUPT_ENDPOINT_OUT, &s_WeigherRcvBuffer[0],
                                               FS_WEIGHER_INTERRUPT_OUT_PACKET_SIZE ); 
        break;
      }
    }
    return error;        
}


