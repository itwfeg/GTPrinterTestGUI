#include "queueManager.h"
#include "fsl_debug_console.h"
#include "usb_misc.h"

USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static QueueHandle_t pUSBInBootQueue_ = NULL;
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static QueueHandle_t pUSBOutBootQueue_ = NULL;

static QueueHandle_t pBootQueue_ = NULL;
static QueueHandle_t pThreadManagerQueue_ = NULL;

QueueHandle_t getBootQueueHandle( void )
{
    return pBootQueue_;
}

QueueHandle_t getUsbInBootQueueHandle( void )
{
    return pUSBInBootQueue_;
}

QueueHandle_t getUsbOutBootQueueHandle( void )
{
    return pUSBOutBootQueue_;
}

QueueHandle_t getThreadManagerQueueHandle( void )
{
    return pThreadManagerQueue_;
}

int getBootMsgQueueLength( void )
{
    return MAX_BOOT_MSG_LENGTH;
}

int getUsbInQueueLength( void )
{
    return MAX_USB_IN_LENGTH;    
}

int getUsbOutQueueLength( void )
{
    return MAX_USB_OUT_LENGTH;    
}

unsigned int getQueueDepth( QueueHandle_t handle_ )
{       
    int depth = 0;
    if( handle_ ==  pUSBInBootQueue_ )
        depth = MAX_USB_IN_DEPTH;
    if( handle_ == pUSBOutBootQueue_ )
        depth = MAX_USB_OUT_DEPTH;
    if( handle_ ==  pBootQueue_ )
        depth = MAX_BOOT_MSG_LENGTH;    
 
    return depth;
}

unsigned int getQueueSize( QueueHandle_t handle_ )
{
    int size = 0;
    if( handle_ ==  pUSBInBootQueue_ )
        size = MAX_USB_IN_LENGTH;
    if( handle_ == pUSBOutBootQueue_ )
        size = MAX_USB_OUT_LENGTH;
    if( handle_ ==  pBootQueue_ )
        size = MAX_BOOT_MSG_LENGTH; 
    
    return size;  
}

void initializeQueueManager( void )
{
  
  pThreadManagerQueue_  = xQueueCreate( MAX_TM_MSG_LENGTH, MAX_TM_MSG_DEPTH );
  if( pThreadManagerQueue_ == NULL ) {
    PRINTF("initializeQueueManager(): Failed to create Thread Manager message queue!\r\n" );
  }
    pUSBOutBootQueue_ = xQueueCreate( MAX_USB_OUT_LENGTH, MAX_USB_OUT_DEPTH );   
    pUSBInBootQueue_ = xQueueCreate( MAX_USB_IN_LENGTH, MAX_USB_IN_DEPTH ); 
    
    if( ( pUSBOutBootQueue_ != NULL ) && ( pUSBInBootQueue_ != NULL ) ) {        
        /* create Weigher message queue */
        pBootQueue_  = xQueueCreate( MAX_BOOT_MSG_LENGTH, MAX_BOOT_MSG_DEPTH );
        if( pBootQueue_ == NULL ) {
            PRINTF("initializeQueueManager(): Failed to create Bootloader message queue!\r\n" );
        }
    } else {
        PRINTF("initializeQueueManager(): Failed to create USB message queue!\r\n" );
    }
}

bool addThreadManagerMsg( unsigned char * pMsg )
{
    bool result = false;
    BaseType_t x;
    
    if( pThreadManagerQueue_ != NULL ) {
        x = xQueueSend(pThreadManagerQueue_, pMsg, 0);
        if( x == pdPASS ) {
            result = true;  
        }
    }
    return result;  
}

void createBootQueue( void ){
  pBootQueue_ = xQueueCreate( MAX_BOOT_MSG_LENGTH, MAX_BOOT_MSG_DEPTH );
  if( pBootQueue_ == NULL ) {
    PRINTF("initializeQueueManager(): Failed to create Boot transaction queue!\r\n" );
  }  
}

void deleteBootUpgraderQueue( void ){
  vQueueDelete( pBootQueue_ );
  if( pBootQueue_ != NULL ) {
    PRINTF("initializeQueueManager(): Failed to delete Boot transaction queue!\r\n" );
  }  
}