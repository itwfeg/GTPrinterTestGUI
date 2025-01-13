#include "usbTask.h"
#include "usbHobartBootloader.h"
#include "usbBootloader.h"
#include "queueManager.h"
#include "translator.h"
#include "usb_device_config.h"
#include "usb_device.h"
#include "usb_device_class.h"
#include "usb_device_ch9.h"
#include "usb_device_descriptor.h"
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "usb_phy.h"

static TaskHandle_t     pUsbHandle_             = NULL;
static QueueHandle_t    pMsgInBootQHandle_        = NULL;
static QueueHandle_t    pMsgOutBootQHandle_       = NULL;
static QueueHandle_t    pBQHandle_              = NULL;/* messages to the bootloader task */
QueueSetHandle_t        uQueueSet_              = NULL;

static bool             suspend_                = false;
static ITransMgr        iTransMgr_;

AT_NONCACHEABLE_SECTION( unsigned char imageBuffer[ USB_BULK_PAYLOAD ] );

extern usb_device_class_struct_t g_UsbDeviceBootloaderConfig;

usb_device_composite_struct_t usbMgr;

USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) uint8_t s_BootloaderRcvBuffer[FS_BOOTLOADER_INTERRUPT_OUT_PACKET_SIZE];
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) uint8_t s_BootloaderSendBuffer[FS_BOOTLOADER_INTERRUPT_IN_PACKET_SIZE];
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) uint8_t s_applicationRcvBuffer[FS_BOOTLOADER_BULK_OUT_PACKET_SIZE];


usb_device_class_config_struct_t g_CompositeClassConfig[USB_COMPOSITE_INTERFACE_COUNT] = {
    {
        USB_DeviceBootloaderCallback,   /* Bootloader class callback pointer */
        (class_handle_t)NULL,           /* The hobart printer class handle, This field is set by USB_DeviceClassInit */
        &g_UsbDeviceBootloaderConfig,   /* The hobart printer keyboard configuration, including class code, subcode, and protocol,
                                           class
                                           type, transfer type, endpoint address, max packet size, etc. */
    }
};

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
BaseType_t createUsbTask( QueueHandle_t bootSendQueue )
{
    BaseType_t result = pdFAIL;
	
	//handle to the bootloader task queue */
	pBQHandle_ = getBootQueueHandle();
    
    /* clear our image transfer manager */
    memset( &iTransMgr_, 0, sizeof( ITransMgr ) ); 
    
    /* get translator queues */ 
    pMsgOutBootQHandle_ = getUsbOutBootQueueHandle();
          
    /* create queue set for messages */
    if( ( bootSendQueue != NULL ) ) { 
        /* pipes are based on host perspective */
        pMsgInBootQHandle_  = bootSendQueue;

        /* determine length of all message queues combined */
        unsigned long wQSetLength = ( getUsbInQueueLength() + getUsbInQueueLength() );
        
        /* create local queue set */
        uQueueSet_ = xQueueCreateSet( wQSetLength );

        if( ( pMsgInBootQHandle_ != NULL ) ) {
            /* add queues to local set */
            xQueueAddToSet( pMsgInBootQHandle_, uQueueSet_ );
        } else {       
            PRINTF("createUsbTask(): Critical Error queue set not created!\r\n" );
        }
               
        if( initUSB() ) {
            /* create usb task thread */
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
    usbMgr.bootloader.deviceHandle      = NULL;
    usbMgr.applicationTaskHandle        = (TaskHandle_t)usbTask;
    usbMgr.deviceTaskHandle             = (TaskHandle_t)USB_DeviceCallback;
    usbMgr.currentConfiguration         = 0;            
    usbMgr.deviceHandle                 = NULL;
   

    /* initialize the usb stack and class drivers */
    if( kStatus_USB_Success == USB_DeviceClassInit( kUSB_ControllerEhci0, 
                                                    &usbMgrConfigList, 
                                                    &usbMgr.deviceHandle) ) {
        
        /* get the class handle */
        usbMgr.bootloader.deviceHandle = usbMgrConfigList.config[0].classHandle;
        
        USB_DeviceBootloaderInit( &usbMgr );
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
    static uint32_t blMsgSize = 0, blMsgCnt = 0;
    static bool blHdr = false; 
    QueueSetMemberHandle_t setHandle = NULL;
	uint16_t blMsgType;
    
    PRINTF("usbTask(): Task starting.......\r\n" );
    while( !suspend_ ) {
        if( pUsbMgr->attach ) {
           
            /* wait for object in one of the queues within the set portMAX_DELAY*/  
            setHandle = xQueueSelectFromSet( uQueueSet_, portMAX_DELAY ); 
            if( setHandle == pMsgInBootQHandle_ ) {                      
                /* how many transactions are queued? */
                blMsgCnt = uxQueueMessagesWaiting( setHandle );
                
				while( blMsgCnt ) {
                    if( xQueueReceive( pMsgInBootQHandle_, (void *)&s_BootloaderSendBuffer[0], portMAX_DELAY ) ) {
						
						blMsgType = s_BootloaderSendBuffer[0];
						blMsgType <<= 8;
						blMsgType |= s_BootloaderSendBuffer[1];								
						
						//based on the message type get message length
						blMsgSize = getBlMsgTypeSize((KPCBootEvent_t)blMsgType);
						
						if( blMsgSize <= FS_BOOTLOADER_INTERRUPT_IN_PACKET_SIZE ) {  
							status = USB_DeviceHBBootloaderSend( pUsbMgr->bootloader.deviceHandle, USB_BOOTLOADER_INTERRUPT_ENDPOINT_IN,
															  &s_BootloaderSendBuffer[0], blMsgSize );                        
							/* keep trying until sent */
							while( status != kStatus_USB_Success )   {
								taskYIELD();
								status = USB_DeviceHBBootloaderSend( pUsbMgr->bootloader.deviceHandle, USB_BOOTLOADER_INTERRUPT_ENDPOINT_IN,
																  &s_BootloaderSendBuffer[0], blMsgSize );                        
							}
							
							blMsgSize = 0;                        
							
						} else {
							PRINTF("usbTask(): Send msg greater than %d  \r\n", FS_BOOTLOADER_INTERRUPT_IN_PACKET_SIZE );                                       

						}             
					
                        blMsgCnt--;
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
    iTransMgr_.labelImageSize = size;    
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
    return iTransMgr_.frameTotalRcvd;    
}


/******************************************************************************/
/*!   \fn unsigned long getPacketTransferTotal( void )

      \brief
        This function initializes the hobart composite printer.        
   
      \author
          Aaron Swift
*******************************************************************************/
usb_status_t USB_DeviceBootloaderInit( usb_device_composite_struct_t *deviceComposite )
{
    usb_status_t error = kStatus_USB_Error;
    if(  deviceComposite->bootloader.deviceHandle != NULL ) { 
      deviceComposite->bootloader.interruptOutEp             = 0;
      deviceComposite->bootloader.interruptInPipeDataBuffer  = &s_BootloaderSendBuffer[0];
      deviceComposite->bootloader.interruptInPipeDataLen     = FS_BOOTLOADER_INTERRUPT_IN_PACKET_SIZE;
      deviceComposite->bootloader.interruptOutPipeDataBuffer = &s_BootloaderRcvBuffer[0];
      deviceComposite->bootloader.interruptOutPipeDataLen    = FS_BOOTLOADER_INTERRUPT_OUT_PACKET_SIZE;
      deviceComposite->bootloader.interruptInPipeBusy        = false;
      deviceComposite->bootloader.interruptOutPipeBusy       = false;
      deviceComposite->bootloader.interruptInPipeStall       = false;
      deviceComposite->bootloader.interruptOutPipeStall      = false;
      deviceComposite->bootloader.msgLength                  = 0;
      deviceComposite->bootloader.bulkOutPipeDataBuffer      = &s_applicationRcvBuffer[0];
      deviceComposite->bootloader.bulkOutPipeDataLen         = FS_BOOTLOADER_BULK_OUT_PACKET_SIZE;
      deviceComposite->bootloader.bulkOutEp                  = 0;
      deviceComposite->bootloader.bulkOutPipeBusy            = false;
      deviceComposite->bootloader.bulkOutPipeStall           = false; 
            
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
usb_status_t USB_DeviceBootloaderSetConfigure(class_handle_t handle, uint8_t configure)
{
    if( USB_COMPOSITE_CONFIGURE_INDEX == configure ) {
        usbMgr.bootloader.attach = 1;
        
        usbMgr.bootloader.interruptOutEp = USB_BOOTLOADER_INTERRUPT_ENDPOINT_OUT;
          
        /* schedule interrupt buffer for receive */
        USB_DeviceHBBootloaderRecv( usbMgr.bootloader.deviceHandle, USB_BOOTLOADER_INTERRUPT_ENDPOINT_OUT, &s_BootloaderRcvBuffer[0],
                             FS_BOOTLOADER_INTERRUPT_OUT_PACKET_SIZE);
        
        usbMgr.bootloader.bulkOutEp = USB_BOOTLOADER_BULK_ENDPOINT_OUT;
        
        /* schedule bulk buffer for receive */
        USB_DeviceHBBootloaderRecv( usbMgr.bootloader.deviceHandle, USB_BOOTLOADER_BULK_ENDPOINT_OUT, &s_applicationRcvBuffer[0],
                             FS_BOOTLOADER_BULK_OUT_PACKET_SIZE);

    }
    return kStatus_USB_Success;  
}

usb_status_t USB_DeviceBootloaderSetInterface(class_handle_t handle, uint8_t interface, uint8_t alternateSetting)
{
    usb_status_t error = kStatus_USB_Error;
    if( usbMgr.bootloader.attach ) {
    
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
                USB_DeviceBootloaderSetConfigure( usbMgr.bootloader.deviceHandle, *temp8 );
                
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
                
                if( USB_BOOTLOADER_INTERFACE_INDEX == interface ) {
                    if( alternateSetting < USB_BOOTLOADER_INTERFACE_ALTERNATE_COUNT ) {
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
usb_status_t USB_DeviceBootloaderCallback( class_handle_t handle, uint32_t event, void *param )
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    usb_device_endpoint_callback_message_struct_t *epCbParam;

    switch (event)
    {
      case kUSB_DeviceBootloaderEventSendResponse: {
        error = kStatus_USB_Success;
        usbMgr.bootloader.interruptInPipeBusy = 0;
        break;
      }        
      case kUSB_DeviceBootloaderEventRecvResponse: {
          if( usbMgr.bootloader.attach ) {       
              usbMgr.bootloader.msgLength =  epCbParam->length; 
            
              if( usbMgr.bootloader.msgLength != 0 ) {
                /* queue the received frame */
                BaseType_t xHigherPriorityTaskWoken = false;
                
                /* post to the translator queue */
                BaseType_t result = xQueueSendToBackFromISR( pMsgOutBootQHandle_, (void *)&s_BootloaderRcvBuffer[0], &xHigherPriorityTaskWoken );  
                if( result != pdPASS ) {
                    PRINTF("USB_DevicePrinterCallback: Failed to queue rx transaction!\r\n" );
                }
              } else {
                  PRINTF("USB_DevicePrinterCallback: transaction lenght 0!\r\n" );
              }
              
              error = USB_DeviceHBBootloaderRecv( usbMgr.bootloader.deviceHandle, USB_BOOTLOADER_INTERRUPT_ENDPOINT_OUT, &s_BootloaderRcvBuffer[0],
              FS_BOOTLOADER_INTERRUPT_OUT_PACKET_SIZE ); 
              if( error != kStatus_USB_Success ) {
                  PRINTF("USB_DevicePrinterCallback: error: %d \r\n", error );  
              }
          }
          break;
      }
      case kUSB_DeviceBootloaderEventRecvBulkResponse: {
		  
		  	if(epCbParam->length <  USB_BULK_PAYLOAD)
			{
				PRINTF("USB_DevicePrinterCallback: transaction lenght less than 512!\r\n" );
			}
				
		  	memcpy( imageBuffer, &s_applicationRcvBuffer[0], USB_BULK_PAYLOAD );
            
			error = USB_DeviceHBBootloaderRecv( usbMgr.bootloader.deviceHandle, USB_BOOTLOADER_BULK_ENDPOINT_OUT, &s_applicationRcvBuffer[0],
            FS_BOOTLOADER_BULK_OUT_PACKET_SIZE );
			
			if( error != kStatus_USB_Success ) {
                PRINTF("USB_DevicePrinterCallback: Bulk error: %d \r\n", error );  
			}
			else
			{	//notify boot task of arrived bulk payload
				BootGenericMsg genericMsg;
				genericMsg.function_code = BOOT_USB_BULK_PAYLOAD_RCV;
				BaseType_t xHigherPriorityTaskWoken = false;

				BaseType_t result = xQueueSendToBackFromISR( pBQHandle_, (void *)&genericMsg, &xHigherPriorityTaskWoken );
				
				if( result != pdTRUE ) {
					PRINTF("t_handleReqStatus(): Boot Message Queue full! \r\n");  
				}
			}
        
          break;
      }
      default: {
          PRINTF("USB_DeviceBootloaderCallback: default: \r\n");  
          error = USB_DeviceHBBootloaderRecv( usbMgr.bootloader.deviceHandle, USB_BOOTLOADER_INTERRUPT_ENDPOINT_OUT, &s_BootloaderRcvBuffer[0],
          FS_BOOTLOADER_INTERRUPT_OUT_PACKET_SIZE ); 
        break;
      }
    }
    return error;        
}