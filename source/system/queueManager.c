#include "queueManager.h"
#include "fsl_debug_console.h"
#include "usb_misc.h"

USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static QueueHandle_t    pUSBInPrQueue_          = NULL;
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static QueueHandle_t    pUSBInWrQueue_          = NULL;
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static QueueHandle_t    pUSBOutPrQueue_         = NULL;
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static QueueHandle_t    pUSBOutWrQueue_         = NULL;
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static QueueHandle_t    pUSBOutBlkQueue_        = NULL;
static QueueHandle_t    pWgQueue_               = NULL;
static QueueHandle_t    pPrQueue_               = NULL;
static QueueHandle_t    pPrCmdQueue_            = NULL;
static QueueHandle_t    pWadQueue_              = NULL;
static QueueHandle_t    pThreadManagerQueue_    = NULL;
static QueueHandle_t    pValueMaxQueue_         = NULL;
static QueueHandle_t    pDotQueue_              = NULL;
static QueueHandle_t    pUTQueue_               = NULL;
static QueueHandle_t    pCRQueue_               = NULL;
static QueueHandle_t    pIPrQueue_              = NULL;

#if 0 /* TO DO: add when ready */
/*****************************************************************************/
static QueueHandle_t    pBootUpgraderQueue_     = NULL;
#endif

QueueHandle_t getUsbInPrQueueHandle( void )
{
    return pUSBInPrQueue_;
}

QueueHandle_t getUsbInWrQueueHandle( void )
{
    return pUSBInWrQueue_;
}

QueueHandle_t getUsbOutPrQueueHandle( void )
{
    return pUSBOutPrQueue_;
}

QueueHandle_t getUsbOutWrQueueHandle( void )
{
    return pUSBOutWrQueue_;
}

QueueHandle_t getUsbOutBlkQueueHandle( void )
{
    return pUSBOutBlkQueue_;
}

QueueHandle_t getUsbOutWeigherQueueHandle( void )
{
    return pUSBOutWrQueue_;
}

QueueHandle_t getWeigherQueueHandle( void )
{
    return pWgQueue_;
}

int getWeigherMsgQueueLength( void ) 
{ 
    return MAX_WG_MSG_LENGTH;
}

QueueHandle_t getWeigherCntsQueueHandle( void )
{
    return pWadQueue_;
}

int getWeigherCntsQueueLength( void ) 
{ 
    return MAX_WG_CNT_LENGTH;
}

QueueHandle_t getDotWearQueueHandle( void )
{
    return pDotQueue_;
}

#if 0 /* TO DO: add when ready */
/*****************************************************************************/

QueueHandle_t getBootUpgraderQueueHandle( void )
{
    return pBootUpgraderQueue_;
}
#endif

int getPrinterMsgQueueLength( void )
{
    return MAX_PR_MSG_LENGTH;
}

QueueHandle_t getThreadManagerQueueHandle( void )
{
    return pThreadManagerQueue_;
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
    if( ( handle_ ==  pUSBInPrQueue_ ) || ( handle_ ==  pUSBInWrQueue_ ) )
        depth = MAX_USB_IN_DEPTH;
    if( ( handle_ == pUSBOutPrQueue_ ) || ( handle_ == pUSBOutWrQueue_ ) )
        depth = MAX_USB_OUT_DEPTH;
    if( handle_ ==  pWgQueue_ )
        depth = MAX_WG_MSG_DEPTH;    
    if( handle_ ==  pWgQueue_ )
        depth = MAX_WG_MSG_DEPTH;
    if( handle_ ==  pWadQueue_ )
        depth = MAX_WG_CNT_DEPTH;
    if( handle_ == pPrCmdQueue_ ) 
        depth = MAX_PR_CMD_DEPTH; 
    if( handle_ == pValueMaxQueue_ )
        depth = MAX_VM_QUEUE_DEPTH; 
    if( handle_ ==  pUTQueue_ )
        depth = MAX_UT_MSG_DEPTH;      
    if( handle_ == pCRQueue_ )
        depth = MAX_CR_MSG_DEPTH;
    if( handle_ == pIPrQueue_ )
        depth = MAX_IPR_MSG_DEPTH;    
    return depth;
}

unsigned int getQueueSize( QueueHandle_t handle_ )
{
    int size = 0;
    if( ( handle_ ==  pUSBInPrQueue_ ) || ( handle_ ==  pUSBInWrQueue_ ) )
        size = MAX_USB_IN_LENGTH;
    if( ( handle_ == pUSBOutPrQueue_ ) || ( handle_ == pUSBOutWrQueue_ ) )
        size = MAX_USB_OUT_LENGTH;
    if( handle_ ==  pWgQueue_ )
        size = MAX_WG_MSG_LENGTH;
    if( handle_ ==  pWadQueue_ )
        size = MAX_WG_CNT_LENGTH;
    if( handle_ == pPrCmdQueue_ ) 
        size = MAX_PR_CMD_LENGTH;
    if( handle_ ==  pUTQueue_ )
        size = MAX_UT_MSG_LENGTH;
    if( handle_ == pCRQueue_ )
        size = MAX_CR_MSG_LENGTH;
    if( handle_ == pIPrQueue_ )
        size = MAX_IPR_MSG_LENGTH;
    return size;  
}

unsigned int getWeigherCountsQueueSize( void )
{
    return MAX_WG_CNT_DEPTH;
}

QueueHandle_t getPrinterQueueHandle( void )
{
    return pPrQueue_;
}

QueueHandle_t getPrCommandQueueHandle( void )
{
    return pPrCmdQueue_;
}


QueueHandle_t getCutterQueueHandle( void )
{
    return pCRQueue_;    
}

QueueHandle_t getInternalPrinterQueueHandle( void )
{
    return pIPrQueue_;
}
    
QueueHandle_t getVMQueueHandle( void )
{
    return pValueMaxQueue_;
}

QueueHandle_t getTestQueueHandle( void )
{
    return pUTQueue_;
}

#if 0 /* TO DO: add when ready */
void createBootUpgraderQueue( void ){
  pBootUpgraderQueue_ = xQueueCreate( MAX_BOOT_UPGRADER_MSG_LENGTH, MAX_BOOT_UPGRADER_MSG_DEPTH );
  if( pBootUpgraderQueue_ == NULL ) {
    PRINTF("initializeQueueManager(): Failed to create Boot Upgrader transaction queue!\r\n" );
  }  
}

void deleteBootUpgraderQueue( void ){
  vQueueDelete( pBootUpgraderQueue_ );
  if( pBootUpgraderQueue_ != NULL ) {
    PRINTF("initializeQueueManager(): Failed to delete Boot Upgrader transaction queue!\r\n" );
  }  
}
#endif

void initializeQueueManager( void )
{
  
  pThreadManagerQueue_  = xQueueCreate( MAX_TM_MSG_LENGTH, MAX_TM_MSG_DEPTH );
  if( pThreadManagerQueue_ == NULL ) {
    PRINTF("initializeQueueManager(): Failed to create Thread Manager message queue!\r\n" );
  }
    /* create USB in weigher and printer coming message queues */   
    pUSBOutPrQueue_ = xQueueCreate( MAX_USB_OUT_LENGTH, MAX_USB_OUT_DEPTH );   
    pUSBOutWrQueue_ = xQueueCreate( MAX_USB_OUT_LENGTH, MAX_USB_OUT_DEPTH );   
    
    /* create USB out going weigher and printer message queues */ 
    pUSBInPrQueue_ = xQueueCreate( MAX_USB_IN_LENGTH, MAX_USB_IN_DEPTH ); 
    pUSBInWrQueue_ = xQueueCreate( MAX_USB_IN_LENGTH, MAX_USB_IN_DEPTH ); 
    
    if( ( pUSBOutPrQueue_ != NULL ) && ( pUSBInPrQueue_ != NULL ) && 
        ( pUSBOutWrQueue_ != NULL ) && ( pUSBInWrQueue_ != NULL ) ) {        
        /* create Weigher message queue */
        pWgQueue_  = xQueueCreate( MAX_WG_MSG_LENGTH, MAX_WG_MSG_DEPTH );
        if( pWgQueue_ == NULL ) {
            PRINTF("initializeQueueManager(): Failed to create Weigher message queue!\r\n" );
        }
        /* create Weigher a/d counts queue */
        pWadQueue_ = xQueueCreate( MAX_WG_CNT_LENGTH, MAX_WG_CNT_DEPTH );
        if( pWadQueue_ == NULL ) {
            PRINTF("initializeQueueManager(): Failed to create Weigher counts queue!\r\n" );
        }
        
        /* create Printer message queue */
        pPrQueue_  = xQueueCreate( MAX_PR_MSG_LENGTH, MAX_PR_MSG_DEPTH );
        if( pPrQueue_ == NULL ) {
            PRINTF("initializeQueueManager(): Failed to create Printer message queue!\r\n" );
        }        
        /* create printer command queue */
        pPrCmdQueue_ = xQueueCreate( MAX_PR_CMD_LENGTH, MAX_PR_CMD_DEPTH );
        if( pPrCmdQueue_ == NULL ) {
            PRINTF("initializeQueueManager(): Failed to create Printer command queue!\r\n" );
        }
        /* create internal Printer message queue */
        pIPrQueue_ = xQueueCreate( MAX_IPR_MSG_LENGTH, MAX_IPR_MSG_DEPTH );
        if( pIPrQueue_ == NULL ) {
            PRINTF("initializeQueueManager(): Failed to create internal Printer message queue!\r\n" );
        } 
        
        /* create value Max transaction queue */
        pValueMaxQueue_ = xQueueCreate( MAX_VM_QUEUE_LENGTH, MAX_VM_QUEUE_DEPTH );
        if( pValueMaxQueue_ == NULL ) {
            PRINTF("initializeQueueManager(): Failed to create Value Max transaction queue!\r\n" );
        }
               
        /* create cutter internal message queue */
        pCRQueue_ = xQueueCreate( MAX_CR_MSG_LENGTH, MAX_CR_MSG_DEPTH );
        if( pCRQueue_ == NULL ) {
            PRINTF("initializeQueueManager(): Failed to create Cutter internal message queue!\r\n" );
        }
        
        pUTQueue_ = xQueueCreate( MAX_UT_MSG_LENGTH, MAX_UT_MSG_DEPTH );
        if( pUTQueue_ == NULL ) {
            PRINTF("initializeQueueManager(): Failed to create Unit Test message queue!\r\n" );
        }
        pDotQueue_ = xQueueCreate( MAX_DW_MSG_LENGTH, MAX_DW_MSG_DEPTH );
        if( pDotQueue_ == NULL ) {
            PRINTF("initializeQueueManager(): Failed to create dot wear queue!\r\n" );
        }
    } else {
        PRINTF("initializeQueueManager(): Failed to create CAN message queue!\r\n" );
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

