#include "translator.h"
#include "queueManager.h"
#include "semphr.h"
#include "fsl_debug_console.h"
#include "bootloader.h"
#include "bootTask.h"



AT_NONCACHEABLE_SECTION_ALIGN_INIT( unsigned char bootRcvUsbBfr[ 74 ], 4U ) = { 0 };

static bool             suspend_                = false;
static TaskHandle_t     tHandle_                = NULL;
static QueueHandle_t    pQUSBSendBootHandle_      = NULL;         /* USB tx messages printer */
static QueueHandle_t    pQUSBRcvBootHandle_       = NULL;         /* USB rx messages printer */

static QueueSetHandle_t tQueueSet_              = NULL;         /* in and out set */
static QueueHandle_t    pBQHandle_              = NULL;         /* messages to the weigher task */ 

static SemaphoreHandle_t tMutex_;

extern BootFrameHeader bFrameHeader;

/*!   \fn BaseType_t createTranslator( QueueHandle_t bootMsgQueue, 
                                       QueueHandle_t UsbInBootMsgQueue, 
                                       QueueHandle_t UsbOutBootMsgQueue )

      \brief
        This function intializes the USB message translator.
   
      \author
          Aaron Swift / Joseph DiCarlantonio
*******************************************************************************/
BaseType_t createTranslator( QueueHandle_t bootMsgQueue, QueueHandle_t UsbInBootMsgQueue, QueueHandle_t UsbOutBootMsgQueue )
{
    BaseType_t result;

    if( ( bootMsgQueue != NULL ) && ( UsbInBootMsgQueue != NULL ) && ( UsbOutBootMsgQueue != NULL ) ) {
        pBQHandle_                = bootMsgQueue;
        pQUSBSendBootHandle_      = UsbInBootMsgQueue;
        pQUSBRcvBootHandle_       = UsbOutBootMsgQueue;
        
        PRINTF("createTranslator(): Starting...\r\n" );
        /* determine length of transmit and receive queues combined */
        unsigned long wQSetLength = ( getUsbInQueueLength() * 2 );
        
        /* create local queue set */
        tQueueSet_ = ( QueueHandle_t )xQueueCreateSet( ( UBaseType_t )wQSetLength );
        /* add queues to local set */
        xQueueAddToSet( pQUSBRcvBootHandle_, tQueueSet_ );
                
        /* create mutex for translateFrameHdr()  */
        tMutex_ = xSemaphoreCreateMutex(); 
        if( !tMutex_ ) {     
            PRINTF("createTranslator(): Failed to create mutex!\r\n" );
        }     
        
        /*create our thread */
        result = xTaskCreate( 
            translatorTask, 
            "TranslatorTask", 
            configMINIMAL_STACK_SIZE, 
            NULL, 
            translator_task_PRIORITY, 
            &tHandle_ 
        );
        
    } else {
        PRINTF("translateMessage(): Queues are null!\r\n" );
    }
    return result;        
}

/******************************************************************************/
/*!   \fn static void translatorTask( void *pvParameters )

      \brief
        This function converts usb frames into hobart messages for manager 
        processing.
   
      \author
          Aaron Swift / Joseph DiCarlantonio
*******************************************************************************/
static void translatorTask( void *pvParameters )
{
    ( void ) pvParameters;
    
    unsigned int msgCnt                 = 0;
    QueueSetMemberHandle_t setHandle    = NULL;
    
    PRINTF("translatorTask(): Thread running...\r\n" ); 
    
    while( !suspend_ ) {           
        /* wait for object in one of the queues within the set*/  
        setHandle = xQueueSelectFromSet( tQueueSet_, portMAX_DELAY );
		
        /* determine which queue */
        if( setHandle ==  pQUSBRcvBootHandle_ ) {
            
			/* how many transactions are queued? */
            msgCnt = uxQueueMessagesWaiting( setHandle );
           
			while( msgCnt ) {          
				/* process message from host */
				if( xQueueReceive( setHandle, &bootRcvUsbBfr[0], 0 ) ) {     //portMAX_DELAY				  
				  translateBootMessage( &bootRcvUsbBfr[0] );   
				  msgCnt--;
				} else {
				  PRINTF("printerTask(): Failed to OUT message from USB queue!\r\n" );              
				}
            }
			
        } else if( setHandle ==  pQUSBSendBootHandle_ ) {
            PRINTF("translatorTask(): Unknown queue set handle!\r\n" );
        } 
		
        taskYIELD();
    }
    vTaskSuspend(NULL);    
}

void assignBootOutGoingMsgQueue( QueueHandle_t pHandle )
{
    pBQHandle_ = pHandle;
}

/******************************************************************************/
/*!   \fn static void translatePrinterMessage( unsigned char *pBfr  )

      \brief
        private function translates bootloader usb frames.
   
      \author
          Aaron Swift
*******************************************************************************/
static void translateBootMessage( unsigned char *pBfr )
{        
    if( pBfr != NULL ) {      
		t_handleBootMessage( pBfr );
		        
    } else {
        PRINTF("translateMessage(): pBfr is null!\r\n" );
    }
    
}

/******************************************************************************/
/*!   \fn static void t_handleBootMessage( unsigned char *pMsg_ )

      \brief
        handles incomming usb hobart printer messages.
         
      \author
          Joseph DiCarlantonio
*******************************************************************************/          
static void t_handleBootMessage( unsigned char *pMsg_ )
{   
	if( pMsg_ != NULL ) {
			
		/* first byte is always the message type */	
		KPCBootEvent_t event = (KPCBootEvent_t)pMsg_[0];//1st parameter is msgType
			
		switch( event )
		{
			case KpcBootReqStatus: {
				PRINTF("t_handlePrinterMessage(): KpcBootReqStatus type: %d!\r\n", pMsg_[0] );
				t_handleReqBootStatus();
				break;
			}

			case KpcBootCmd: {
				PRINTF("t_handlePrinterMessage(): KpcBootCmd type: %d!\r\n", pMsg_[0] );
				t_handleBootCmd( pMsg_ );
				break;
			}
			
			case KpcBootReqVersion: {
				PRINTF("t_handlePrinterMessage(): KpcBootReqVersion type: %d!\r\n", pMsg_[0] );					
				t_handleReqBootVersion();
				break;
			}
			
			case KpcBootReqAppVersion: {
				PRINTF("t_handlePrinterMessage(): KpcBootReqAppVersion type: %d!\r\n", pMsg_[0] );					
				t_handleReqAppVersion();
				break;
			}

			default: {
				//PRINTF("t_handlePrinterMessage(): Unknown message type: %d!\r\n", *pMsg_ );
			}
		}
		
    } else {
        PRINTF("t_handlePrinterMessage(): pMsg_ is NULL!\r\n" );    
    }

}

/******************************************************************************/
/*!   \fn unsigned short charToShort( unsigned char *pData  )

      \brief
        public creates unsigned word from char array
   
      \author
          Aaron Swift
*******************************************************************************/
unsigned short charToShort( unsigned char *pData )
{
    unsigned short data;
    data =  *pData++;
    data <<= 8;
    data |= *pData;
    return data;
}

/******************************************************************************/
/*!   \fn unsigned short nCharToShort( unsigned char *pData )

      \brief
        public creates unsigned word from byte swapped char array
   
      \author
          Aaron Swift
*******************************************************************************/
unsigned short nCharToShort( unsigned char *pData )
{
    unsigned short data;
    data =  *( pData + 1 );
    data <<= 8;
    data |= *pData;
    return data;    
}

/******************************************************************************/
/*!   \fn unsigned long charToLong( unsigned char *pData )

      \brief
        public converts char data into long.
   
      \author
          Aaron Swift
*******************************************************************************/
unsigned long charToLong( unsigned char *pData )
{
    
    unsigned long data;
  
    data =  *pData++;
    data <<= 8;
    data |= *pData++;
    data <<= 8;
    data |= *pData++;
    data <<= 8;
    data |= *pData;    

    return data;
}

/******************************************************************************/
/*!   \fn unsigned long nCharToLong( unsigned char *pData )

      \brief
        public creates unsigned long from byte swapped char array
   
      \author
          Aaron Swift
*******************************************************************************/
unsigned long nCharToLong( unsigned char *pData )
{
    
    unsigned long data;
  
    data =  *( pData + 3 );
    data <<= 8;
    data |= *( pData + 2 );
    data <<= 8;
    data |= *( pData + 1 );
    data <<= 8;
    data |= *pData;    

    return data;
}

/******************************************************************************/
/*!   \fn unsigned char getCharFromShort( unsigned short *pWord, unsigned char index )

      \brief
        public get character from indexed unsigned word.
   
      \author
          Aaron Swift
*******************************************************************************/
unsigned char getCharFromShort( unsigned short *pWord, unsigned char index )
{
    unsigned char char_ = 0;
    if( index == 0 ) {
      char_ = (unsigned char)( *pWord & 0x00ff );
    }
    else if( index == 1 ) {
      char_ = (unsigned char)( ( *pWord & 0xff00 ) >> 8 );
    }
    else {
        PRINTF("getCharFromShort(): Conversion error!\r\n");        
    }
    return char_;
}

/******************************************************************************/
/*!   \fn unsigned char getCharFromLong( unsigned long *pLong, unsigned char index )

      \brief
        public get character from indexed unsigned long.

      \author
          Aaron Swift
*******************************************************************************/
unsigned char getCharFromLong( unsigned long *pLong, unsigned char index )
{
    unsigned char char_ = 0;
    if( index == 0 ) {
    	char_ = (unsigned char)( *pLong & 0x00ff );
    }
    else if( index == 1 ) {
    	char_ = (unsigned char)( ( *pLong & 0xff00 ) >> 8 );
    }
    else if(index == 2 ) {
    	char_ = (unsigned char)( ( *pLong & 0xff0000 ) >> 16 );
    }
    else if(index == 3 ) {
    	char_ = (unsigned char)( ( *pLong & 0xff000000 ) >> 24 );
    }
    else {
        PRINTF("getCharFromLong(): Conversion error!\r\n");
    }
    return char_;

}

////////////////////////
////////////////////////
///////////	Translator Receive handler functions
////////////////////////
////////////////////////
static void t_handleReqBootStatus(void)
{
    BootGenericMsg genericMsg;
    genericMsg.function_code = BOOT_REQ_STATUS;
    
    BaseType_t result = xQueueSendToBack( pBQHandle_, (void *)&genericMsg, 0 );
    if( result != pdTRUE ) {
        PRINTF("t_handleReqStatus(): Boot Message Queue full! \r\n");  
    }
}

static void t_handleBootCmd( unsigned char * BootCmd )
{
    BootCmdMsg command;
    command.function_code	= BOOT_CMD;
    command.boot_cmd		= BootCmd[1];
    command.dest			= BootCmd[3];
	command.src				= BootCmd[4];	
    command.addr			= BootCmd[5];
    
    if( command.boot_cmd == JUMP ) {
        PRINTF("jumping\r\n");
    }
        
    BaseType_t result = xQueueSendToBack( pBQHandle_, ( void * )&command, 0);
    if( result != pdTRUE ) {
        PRINTF("t_handleBootCommand(): could not send boot command\r\n");
    }
    
}

static void t_handleReqAppVersion(void){
    
    BootGenericMsg bootMsg;
    bootMsg.function_code = BOOT_REQ_APP_VERSION;

    /* add message to the boot queue */
    BaseType_t result = xQueueSendToBack( pBQHandle_, (void *)&bootMsg, 0 );
    if( result != pdTRUE ) {
        PRINTF("t_handleReqAppVersion(): Boot Message Queue full! \r\n");  
    }
}

static void t_handleReqBootVersion(void){
    
    BootGenericMsg bootMsg;
    bootMsg.function_code = BOOT_REQ_BOOT_VERSION;

    /* add message to the boot queue */
    BaseType_t result = xQueueSendToBack( pBQHandle_, (void *)&bootMsg, 0 );
    if( result != pdTRUE ) {
        PRINTF("t_handleReqVersion(): Boot Message Queue full! \r\n");  
    }
}


////////////////////////
///////////////////////
/////		send functions
/////////////////////////
////////////////////////
/******************************************************************************/
/*!   \fn void sendPrWakeup( PrWakeup *pWakeMsg)

      \brief
        handles converting printer wakeup message into usb frames and 
        adding to usb transmit queue for sending. 
      \author
          Aaron Swift
*******************************************************************************/
void sendBootReady(BootReadyMsg * rdy)
{
    unsigned char body[ sizeof( BootReadyMsg ) ]; 
    unsigned int index = 0;
    
	memset(&body, 0, sizeof(body) );
	
	/* initialize body of the usb message */
    body[index++] = getCharFromShort( (unsigned short *)&rdy->msgType, 1 );
	body[index++] = getCharFromShort( (unsigned short *)&rdy->msgType, 0 );
    body[index++] = getCharFromShort( (unsigned short *)&rdy->product_id, 1 ); 
    body[index++] = getCharFromShort( (unsigned short *)&rdy->product_id, 0 );
    
    /* post message to queue */    
    if( pQUSBSendBootHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendBootHandle_, (void *)&body[0], portMAX_DELAY );
        if( result != pdPASS ) {
            PRINTF("sendWakeup(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendWakeup(): pQUSBSendBootHandle_ is null!\r\n");
    }
   
}  

void sendBootAck(void)
{
	BootGenericTypeMsg ack;
    unsigned char body[ sizeof( BootGenericTypeMsg ) ]; 
    unsigned int index = 0;
    
	memset(&body, 0, sizeof(body) );
	
	//set ack msg type
	ack.msgType = KpcBootAck;
	
	/* initialize body of the usb message */
    body[index++] = getCharFromShort( (unsigned short *)&ack.msgType, 1 );
	body[index++] = getCharFromShort( (unsigned short *)&ack.msgType, 0 );
    
    /* post message to queue */    
    if( pQUSBSendBootHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendBootHandle_, (void *)&body[0], portMAX_DELAY );
        if( result != pdPASS ) {
            PRINTF("sendWakeup(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendWakeup(): pQUSBSendBootHandle_ is null!\r\n");
    }
   
}

void sendBootStatus(BootStatusMsg * status)
{
    unsigned char body[ sizeof( BootStatusMsg ) ]; 
    unsigned int index = 0;
    
	memset(&body, 0, sizeof(body) );
	
	/* initialize body of the usb message */
    body[index++] = getCharFromShort( (unsigned short *)&status->msgType, 1 );
	body[index++] = getCharFromShort( (unsigned short *)&status->msgType, 0 );
    body[index++] = getCharFromShort( (unsigned short *)&status->model, 1 ); 
    body[index++] = getCharFromShort( (unsigned short *)&status->model, 0 );
	body[index++] = getCharFromShort( (unsigned short *)&status->status, 1 ); 
    body[index++] = getCharFromShort( (unsigned short *)&status->status, 0 );
	body[index++] = getCharFromShort( (unsigned short *)&status->fault, 1 ); 
    body[index++] = getCharFromShort( (unsigned short *)&status->fault, 0 );
	body[index++] = getCharFromLong( (unsigned long *)&status->fault_info, 3 );
    body[index++] = getCharFromLong( (unsigned long *)&status->fault_info, 2 );
    body[index++] = getCharFromLong( (unsigned long *)&status->fault_info, 1 );
    body[index++] = getCharFromLong( (unsigned long *)&status->fault_info, 0 );  
    
    
    /* post message to queue */    
    if( pQUSBSendBootHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendBootHandle_, (void *)&body[0], portMAX_DELAY );
        if( result != pdPASS ) {
            PRINTF("sendWakeup(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendWakeup(): pQUSBSendBootHandle_ is null!\r\n");
    }
   
}  

void sendBootVersion(BootVerMsg *version)
{
	unsigned char body[ sizeof( BootVerMsg ) ]; 
    unsigned int index = 0;
    
	memset(&body, 0, sizeof(body) );
	
	/* initialize body of the usb message */
    body[index++] = getCharFromShort( (unsigned short *)&version->msgType, 1 );
	body[index++] = getCharFromShort( (unsigned short *)&version->msgType, 0 );
	body[index++] = getCharFromShort( (unsigned short *)&version->boot_version.major_rev, 1 );
	body[index++] = getCharFromShort( (unsigned short *)&version->boot_version.major_rev, 0 );
    body[index++] = getCharFromShort( (unsigned short *)&version->boot_version.minor_rev, 1 ); 
    body[index++] = getCharFromShort( (unsigned short *)&version->boot_version.minor_rev, 0 );
	body[index++] = getCharFromShort( (unsigned short *)&version->boot_version.build_number, 1 ); 
    body[index++] = getCharFromShort( (unsigned short *)&version->boot_version.build_number, 0 );
	
    /* post message to queue */    
    if( pQUSBSendBootHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendBootHandle_, (void *)&body[0], portMAX_DELAY );
        if( result != pdPASS ) {
            PRINTF("sendBootVersion(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendBootVersion(): pQUSBSendBootHandle_ is null!\r\n");
    }
	
}

void sendBootAppVersion(BootAppVerMsg *version)
{
	unsigned char body[ sizeof( BootAppVerMsg ) ]; 
    unsigned int index = 0;
    
	memset(&body, 0, sizeof(body) );
	
	/* initialize body of the usb message */
    body[index++] = getCharFromShort( (unsigned short *)&version->msgType, 1 );
	body[index++] = getCharFromShort( (unsigned short *)&version->msgType, 0 );
	body[index++] = getCharFromShort( (unsigned short *)&version->printer_app_version.major_rev, 1 );
	body[index++] = getCharFromShort( (unsigned short *)&version->printer_app_version.major_rev, 0 );
    body[index++] = getCharFromShort( (unsigned short *)&version->printer_app_version.minor_rev, 1 ); 
    body[index++] = getCharFromShort( (unsigned short *)&version->printer_app_version.minor_rev, 0 );
	body[index++] = getCharFromShort( (unsigned short *)&version->printer_app_version.build_number, 1 ); 
    body[index++] = getCharFromShort( (unsigned short *)&version->printer_app_version.build_number, 0 );
	body[index++] = getCharFromShort( (unsigned short *)&version->weigher_app_version.major_rev, 1 ); 
    body[index++] = getCharFromShort( (unsigned short *)&version->weigher_app_version.major_rev, 0 );
	body[index++] = getCharFromShort( (unsigned short *)&version->weigher_app_version.minor_rev, 1 );
    body[index++] = getCharFromShort( (unsigned short *)&version->weigher_app_version.minor_rev, 0 );
    body[index++] = getCharFromShort( (unsigned short *)&version->weigher_app_version.build_number, 1 );
    body[index++] = getCharFromShort( (unsigned short *)&version->weigher_app_version.build_number, 0 );      
    
    /* post message to queue */    
    if( pQUSBSendBootHandle_ != NULL ) {
        BaseType_t result = xQueueSend( pQUSBSendBootHandle_, (void *)&body[0], portMAX_DELAY );
        if( result != pdPASS ) {
            PRINTF("sendBootAppVersion(): USB tx message queue is full!\r\n");
        }
    } else {
        PRINTF("sendBootAppVersion(): pQUSBSendBootHandle_ is null!\r\n");
    }
	
}

uint32_t getBlMsgTypeSize(KPCBootEvent_t blMsgType)
{
	uint32_t size = 0;
	
	switch( blMsgType )
	{
		case KpcBootReady:
		{
			size = sizeof( BootReadyMsg );
			PRINTF("getBlMsgTypeSize(): sizeof( BootReadyMsg ) = %d\r\n", size );
		}
		break;
		
		case KpcBootStatus:
		{
			size = sizeof( BootStatusMsg );
			PRINTF("getBlMsgTypeSize(): sizeof( KpcBootStatus ) = %d\r\n", size );
		}
		break;
		
		case KpcBootVersion:
		{
			size = sizeof( BootVerMsg );
			PRINTF("getBlMsgTypeSize(): sizeof( BootVerMsg ) = %d\r\n", size );
		}
		break;
		
		case KpcBootAppVersion:
		{
			size = sizeof( BootAppVerMsg );
			PRINTF("getBlMsgTypeSize(): sizeof( BootAppVerMsg ) = %d\r\n", size );
		}
		break;
		
		case KpcBootAck:
		{
			size = sizeof( BootGenericTypeMsg );
			PRINTF("getBlMsgTypeSize(): sizeof( KpcBootAck ) = %d\r\n", size );
		}
		break;
		
		
	}
	
	return size;
}