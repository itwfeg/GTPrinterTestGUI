#include "averyCutter.h"
#include "semphr.h"
#include "fsl_debug_console.h"


SemaphoreHandle_t       pCutDoneSemaphore       = NULL;

static TaskHandle_t     cHandle_                = NULL;
static QueueHandle_t    pMsgQHandle_            = NULL;
static QueueHandle_t    pIPMsgQueue             = NULL;
static bool             suspend_                = false;



AT_NONCACHEABLE_SECTION_ALIGN( lpuart_handle_t cutterHandle_, 4U ); 
AT_NONCACHEABLE_SECTION_ALIGN( unsigned char uartTxBfr[ MAX_MESSGAE_SIZE ], 4U );
AT_NONCACHEABLE_SECTION_ALIGN( unsigned char uartRxBfr[ MAX_MESSGAE_SIZE ], 4U );
AT_NONCACHEABLE_SECTION_ALIGN( static CutterMgr cutterMgr_, 4U );

const unsigned short crc16[CRC_TABLE_SIZE] = 
                            {   0xABCD, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 
                                0x0280, 0xC241, 0xC601, 0x06C0, 0x0780, 0xC741, 
                                0x0500, 0xC5C1, 0xC481, 0x0440, 0xCC01, 0x0CC0, 
                                0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
                                0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 
                                0x0880, 0xC841, 0xD801, 0x18C0, 0x1980, 0xD941, 
                                0x1B00, 0xDBC1, 0xDA81, 0x1A40, 0x1E00, 0xDEC1, 
                                0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
                                0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 
                                0x1680, 0xD641, 0xD201, 0x12C0, 0x1380, 0xD341, 
                                0x1100, 0xD1C1, 0xD081, 0x1040, 0xF001, 0x30C0, 
                                0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
                                0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 
                                0x3480, 0xF441, 0x3C00, 0xFCC1, 0xFD81, 0x3D40, 
                                0xFF01, 0x3FC0, 0x3E80, 0xFE41, 0xFA01, 0x3AC0, 
                                0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
                                0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 
                                0x2A80, 0xEA41, 0xEE01, 0x2EC0, 0x2F80, 0xEF41, 
                                0x2D00, 0xEDC1, 0xEC81, 0x2C40, 0xE401, 0x24C0, 
                                0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
                                0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 
                                0x2080, 0xE041, 0xA001, 0x60C0, 0x6180, 0xA141, 
                                0x6300, 0xA3C1, 0xA281, 0x6240, 0x6600, 0xA6C1, 
                                0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
                                0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 
                                0x6E80, 0xAE41, 0xAA01, 0x6AC0, 0x6B80, 0xAB41, 
                                0x6900, 0xA9C1, 0xA881, 0x6840, 0x7800, 0xB8C1, 
                                0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
                                0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 
                                0xBC81, 0x7C40, 0xB401, 0x74C0, 0x7580, 0xB541, 
                                0x7700, 0xB7C1, 0xB681, 0x7640, 0x7200, 0xB2C1, 
                                0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
                                0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 
                                0x5280, 0x9241, 0x9601, 0x56C0, 0x5780, 0x9741, 
                                0x5500, 0x95C1, 0x9481, 0x5440, 0x9C01, 0x5CC0, 
                                0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
                                0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 
                                0x5880, 0x9841, 0x8801, 0x48C0, 0x4980, 0x8941, 
                                0x4B00, 0x8BC1, 0x8A81, 0x4A40, 0x4E00, 0x8EC1, 
                                0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41, 
                                0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 
                                0x4680, 0x8641, 0x8201, 0x42C0, 0x4380, 0x8341, 
                                0x4100, 0x81C1, 0x8081, 0x4040 };


const TickType_t                lightSleep = 100 / portTICK_PERIOD_MS;
const TickType_t                deepSleep = 1000 / portTICK_PERIOD_MS;


SemaphoreHandle_t               cMutex_;
static ACTSTATES                actState_;



extern uint32_t BOARD_DebugConsoleSrcFreq(void);

/******************************************************************************/
/*!   \fn BaseType_t createAveryCutterTask( QueueHandle_t msgQueue, 
                                            QueueHandle_t printerMsgQueue )

      \brief
        This function creates a cutter task to manage messaging between the 
        device and host and printer task.   
       
      \author
          Aaron Swift
*******************************************************************************/                
BaseType_t createAveryCutterTask( QueueHandle_t msgQueue, 
                                  QueueHandle_t printerMsgQueue )
{
    BaseType_t result;
    PRINTF("createAveryCutterTask(): Starting...\r\n" );
    /* set our task state */
    actState_ = AC_INIT_INTERFACE_;
    /* assign my internal message queue */
    pMsgQHandle_ = msgQueue;
    /* assign the printers internal message queue */
    pIPMsgQueue = printerMsgQueue;
    
    pCutDoneSemaphore = xSemaphoreCreateBinary();
    
    if( ( pMsgQHandle_ != NULL ) && ( pIPMsgQueue != NULL ) ) {
        /* create printer task thread */
        result = xTaskCreate( averyCutterTask,  "CutterTask", configMINIMAL_STACK_SIZE,
                                            NULL, 1, &cHandle_ );
    } else {
        PRINTF("createAveryCutterTask(): Failed to create Cutter task. Queue is null!\r\n" );
    }
    return result;
}

/******************************************************************************/
/*!   \fn bool initCutter( void ) 

      \brief
        This function initializes the cutter manager and opens the cutter 
        interface. Returns  
       
      \author
          Aaron Swift
*******************************************************************************/                
bool initCutter( void )
{
    bool status = false;
    /* reset the manager */
    memset( (void *)&cutterMgr_, 0, sizeof(cutterMgr_) );
    
    cutterMgr_.version.productName[20] = '\0';
    cutterMgr_.version.firmware[12] = '\0';
    cutterMgr_.version.issueDate[8] = '\0';
    cutterMgr_.version.date[11] = '\0';   
    
    /* clear the mgs buffers */
    memset( (void *)&uartTxBfr[0], 0, MAX_MESSGAE_SIZE );
    memset( (void *)&uartRxBfr[0], 0xff, MAX_MESSGAE_SIZE );
 
    /* create mutex for interface */
    vSemaphoreCreateBinary( cMutex_ )
    //cMutex_ = xSemaphoreCreateMutex(); 
    if( cMutex_ ) {             
        /* open the interface */
        if( openCutterInterface( CUTTER_UART_BAUD ) ) {                                    
            status = true;
        }
    } else {
        PRINTF("initCutter(): Failed to create mutex!\r\n" );
    }
    return status;
}

/******************************************************************************/
/*!   \fn bool sendACCut( CutterMgr *pMgr, ACCutPaper *pMsg )

      \brief
        This function builds and sends the cut command message to the cutter 
        interface. 
       
      \author
          Aaron Swift
*******************************************************************************/                
bool sendACCut( CutterMgr *pMgr, ACCutPaper *pMsg )
{
    ACutterMsgs msg;
    lpuart_transfer_t xfer;
    bool sent = false;
    
    if( cMutex_ ) {
        /* wait for mutex */
        if( xSemaphoreTake( cMutex_, ( TickType_t )portMAX_DELAY ) == pdTRUE ) {
            pMgr->txReady = false;       
            buildCutterMsg( AC_CUT_, &msg, pMsg->distance, pMsg->speed );
            stuffTxBuffer( (unsigned char *)&msg, (unsigned char)( HEADER_SIZE + sizeof(ACCutPaper) ) );
            
            /* append crc and ext */
            prepTransmitBfr( &uartTxBfr[1], ( getMessageSize( msg.header.command ) - 1 ) );

            /* fill out transfer info */           
            xfer.data = &uartTxBfr[0];
            xfer.dataSize = ( sizeof(ACHeader) + sizeof(ACCutPaper) + 5 );
            /* send the command */
            status_t result = LPUART_TransferSendNonBlocking( LPUART1, &cutterHandle_, &xfer );
            if( result != kStatus_Success ) {
                PRINTF("sendACCut(): Failed to send AC_CUT_!\r\n" );
            } else {
                sent = true;
            }
        } else {
            PRINTF("sendACCut(): Failed to take mutex!\r\n" );
        }
    } else {
        PRINTF("sendACCut(): Mutex is null!\r\n" );
    }
    return sent;
}

/******************************************************************************/
/*!   \fn bool sendACReqStatus( CutterMgr *pMgr, ACCutPaper *pMsg )

      \brief
        This function builds and sends a request for status command message 
        to the cutter interface. 
       
      \author
          Aaron Swift
*******************************************************************************/                
bool sendACReqStatus( CutterMgr *pMgr )
{
    ACutterMsgs msg;
    lpuart_transfer_t xfer;
    bool sent = false;
        
    if( cMutex_ ) {
        /* wait for mutex */
        if( xSemaphoreTake( cMutex_, ( TickType_t )portMAX_DELAY ) == pdTRUE ) {
            pMgr->txReady = false;       
            /* build the cutter message */ 
            buildCutterMsg( AC_REQ_STATUS_, &msg, NULL, NULL );
            stuffTxBuffer( (unsigned char *)&msg, (unsigned char)HEADER_SIZE );
            /* append crc and ext */
            prepTransmitBfr( &uartTxBfr[1], getMessageSize( msg.header.command ) );
            uartTxBfr[5] = '1';
            uartTxBfr[6] = '1';
            uartTxBfr[7] = '1';
            uartTxBfr[8] = '1';
            uartTxBfr[14] = 'E';
            uartTxBfr[15] = '1';
            /* fill out transfer info */           
            xfer.data = &uartTxBfr[0];
            xfer.dataSize = (size_t)( getMessageSize( msg.header.command ) + 6 ); /* crc + ext */
            /* send the command */
            status_t result = LPUART_TransferSendNonBlocking( LPUART1, &cutterHandle_, &xfer );
            if( result != kStatus_Success ) {
                PRINTF("sendACReqStatus(): Failed to send AC_REQ_STATUS_!\r\n" );
            } else {
                sent = true;
            }
        } else {
            PRINTF("sendACReqStatus(): Failed to take mutex!\r\n" );
        }
    } else {
        PRINTF("sendACReqStatus(): Mutex is null!\r\n" );
    }
    return sent;    
}

/******************************************************************************/
/*!   \fn bool sendACReqStatus( CutterMgr *pMgr, ACCutPaper *pMsg )

      \brief
        This function builds and sends a request for version command message 
        to the cutter interface. The version recieved is based on the version
        index.
        
        0: Product name
        1: Firmware number
        2: Firmware issue
        3: Firmware date
        4: Time
       
      \author
          Aaron Swift
*******************************************************************************/                
bool sendACReqVersion( CutterMgr *pMgr, ACRVersion *pMsg )
{
    ACutterMsgs msg;
    lpuart_transfer_t xfer;
    bool sent = false;
    
    if( cMutex_ ) {
        /* wait for mutex */
        if( xSemaphoreTake( cMutex_, ( TickType_t )portMAX_DELAY ) == pdTRUE ) {
            pMgr->txReady = false;       
            buildCutterMsg( AC_VERSION_, &msg, pMsg->index, NULL );
            stuffTxBuffer( (unsigned char *)&msg, (unsigned char)( sizeof(ACHeader) + sizeof(ACRVersion) ) );

            /* append crc and ext */
            prepTransmitBfr( &uartTxBfr[1], getMessageSize( msg.header.command ) );

            /* fill out transfer info */           
            xfer.data = &uartTxBfr[0];
            xfer.dataSize = (size_t)( getMessageSize( msg.header.command ) + 6 ); /* crc + ext */
            /* send the command */
            status_t result = LPUART_TransferSendNonBlocking( LPUART1, &cutterHandle_, &xfer );
            if( result != kStatus_Success ) {
                PRINTF("sendACReqVersion(): Failed to send AC_VERSION_!\r\n" );
            } else {
                sent = true;
            }
        } else {
            PRINTF("sendACReqVersion(): Failed to take mutex!\r\n" );
        }
    } else {
        PRINTF("sendACReqVersion(): Mutex is null!\r\n" );
    } 
    return sent;    
}

/******************************************************************************/
/*!   \fn bool sendACHome( CutterMgr *pMgr, ACHome *pMsg )

      \brief
        This function builds and sends a home command message 
        to the cutter interface. 
       
      \author
          Aaron Swift
*******************************************************************************/                
bool sendACHome( CutterMgr *pMgr, ACHome *pMsg )
{
    ACutterMsgs msg;
    lpuart_transfer_t xfer;
    bool sent = false;
    
    if( cMutex_ ) {
        /* wait for mutex */
        if( xSemaphoreTake( cMutex_, ( TickType_t )portMAX_DELAY ) == pdTRUE ) {
            pMgr->txReady = false;       
            buildCutterMsg( AC_HOME_, &msg, pMsg->speed, NULL );
            stuffTxBuffer( (unsigned char *)&msg, ( sizeof(ACHeader) + sizeof(ACHome) ) );
            
            /* append crc and ext */
            prepTransmitBfr( &uartTxBfr[1], getMessageSize( msg.header.command ) );
            
            /* fill out transfer info */           
            xfer.data = &uartTxBfr[0];
            xfer.dataSize = (size_t)( getMessageSize( msg.header.command ) + 6 ); /* crc + ext */
            /* send the command */
            status_t result = LPUART_TransferSendNonBlocking( LPUART1, &cutterHandle_, &xfer );
            if( result != kStatus_Success ) {
                PRINTF("sendACHome(): Failed to send AC_HOME_!\r\n" );
            } else {
                sent = true;
            }
        } else {
            PRINTF("sendACHome(): Failed to take mutex!\r\n" );
        }
    } else {
        PRINTF("sendACHome(): Mutex is null!\r\n" );
    }
    return sent;        
}

/******************************************************************************/
/*!   \fn bool sendACCutSteps( CutterMgr *pMgr, ACCutSteps *pMsg )

      \brief
        This function builds and sends a cut command message 
        with the distance to travel and how fast to the cutter interface. 
       
      \author
          Aaron Swift
*******************************************************************************/                
bool sendACCutSteps( CutterMgr *pMgr, ACCutSteps *pMsg )
{
    ACutterMsgs msg;
    lpuart_transfer_t xfer;
    bool sent = false;
    
    if( cMutex_ ) {
        /* wait for mutex */
        if( xSemaphoreTake( cMutex_, ( TickType_t )portMAX_DELAY ) == pdTRUE ) {
            pMgr->txReady = false;       
            buildCutterMsg( AC_CUT_STEPS_, &msg, pMsg->distance, pMsg->speed );
            stuffTxBuffer( (unsigned char *)&msg, ( sizeof(ACHeader) + sizeof(ACCutSteps) ) );
            
            /* append crc and ext */
            prepTransmitBfr( &uartTxBfr[1], getMessageSize( msg.header.command ) );

            /* fill out transfer info */           
            xfer.data = &uartTxBfr[0];
            xfer.dataSize = (size_t)( getMessageSize( msg.header.command ) + 6 ); /* crc + ext */
            /* send the command */
            status_t result = LPUART_TransferSendNonBlocking( LPUART1, &cutterHandle_, &xfer );
            if( result != kStatus_Success ) {
                PRINTF("sendACCutSteps(): Failed to send AC_CUT_STEPS_!\r\n" );
            } else {
                sent = true;
            }
        } else {
            PRINTF("sendACCutSteps(): Failed to take mutex!\r\n" );
        }
    } else {
        PRINTF("sendACCutSteps(): Mutex is null!\r\n" );
    } 
    return sent;    
}

/******************************************************************************/
/*!   \fn bool sendACReadProfile( CutterMgr *pMgr, ACReadProfile *pMsg )

      \brief
        This function builds and sends a profile command message 
         to the cutter interface. 
       
      \author
          Aaron Swift
*******************************************************************************/                
bool sendACReadProfile( CutterMgr *pMgr, ACReadProfile *pMsg )
{
    ACutterMsgs msg;
    lpuart_transfer_t xfer;
    bool sent = false;
    
    if( cMutex_ ) {
        /* wait for mutex */
        if( xSemaphoreTake( cMutex_, ( TickType_t )portMAX_DELAY ) == pdTRUE ) {
            pMgr->txReady = false;       
            buildCutterMsg( AC_READ_SPEED_, &msg, pMsg->data, NULL );
            stuffTxBuffer( (unsigned char *)&msg, ( sizeof(ACHeader) + sizeof(ACReadProfile) ) );

            /* append crc and ext */
            prepTransmitBfr( &uartTxBfr[1], getMessageSize( msg.header.command ) );
            
            /* fill out transfer info */           
            xfer.data = &uartTxBfr[0];
            xfer.dataSize = (size_t)( getMessageSize( msg.header.command ) + 6 ); /* crc + ext */
            /* send the command */
            status_t result = LPUART_TransferSendNonBlocking( LPUART1, &cutterHandle_, &xfer );
            if( result != kStatus_Success ) {
                PRINTF("sendACReadProfile(): Failed to send AC_READ_SPEED_!\r\n" );
            } else {
                sent = true;
            }
        } else {
            PRINTF("sendACReadProfile(): Failed to take mutex!\r\n" );
        }
    } else {
        PRINTF("sendACReadProfile(): Mutex is null!\r\n" );
    } 
    return sent;    
}

/******************************************************************************/
/*!   \fn bool sendACTestMode( CutterMgr *pMgr )

      \brief
        This function builds and sends a test mode command message 
         to the cutter interface. 
       
      \author
          Aaron Swift
*******************************************************************************/                
bool sendACTestMode( CutterMgr *pMgr )
{
    ACutterMsgs msg;
    lpuart_transfer_t xfer;
    bool sent = false;
    
    if( cMutex_ ) {
        /* wait for mutex */
        if( xSemaphoreTake( cMutex_, ( TickType_t )portMAX_DELAY ) == pdTRUE ) {
            pMgr->txReady = false;       
            buildCutterMsg( AC_TEST_MODE_, &msg, NULL, NULL );
            stuffTxBuffer( (unsigned char *)&msg, sizeof(ACHeader) );

            /* append crc and ext */
            prepTransmitBfr( &uartTxBfr[1], getMessageSize( msg.header.command ) );

            /* fill out transfer info */           
            xfer.data = &uartTxBfr[0];
            xfer.dataSize = (size_t)( getMessageSize( msg.header.command ) + 6 ); /* crc + ext */
            /* send the command */
            status_t result = LPUART_TransferSendNonBlocking( LPUART1, &cutterHandle_, &xfer );
            if( result != kStatus_Success ) {
                PRINTF("sendACTestMode(): Failed to send AC_TEST_MODE_!\r\n" );
            } else {
                sent = true;
            }
        } else {
            PRINTF("sendACTestMode(): Failed to take mutex!\r\n" );
        }
    } else {
        PRINTF("sendACTestMode(): Mutex is null!\r\n" );
    } 
    return sent;    
}

/******************************************************************************/
/*!   \fn bool sendACReset( CutterMgr *pMgr )

      \brief
        This function builds and sends a reset command message 
         to the cutter interface. 
       
      \author
          Aaron Swift
*******************************************************************************/                
bool sendACReset( CutterMgr *pMgr )
{
    ACutterMsgs msg;
    lpuart_transfer_t xfer;
    bool sent = false;
    
    if( cMutex_ ) {
        /* wait for mutex */
        if( xSemaphoreTake( cMutex_, ( TickType_t )portMAX_DELAY ) == pdTRUE ) {
            pMgr->txReady = false;       
            buildCutterMsg( AC_RESET_, &msg, NULL, NULL );
            stuffTxBuffer( (unsigned char *)&msg, sizeof(ACHeader) );

            /* append crc and ext */
            prepTransmitBfr( &uartTxBfr[1], getMessageSize( msg.header.command ) );

            /* fill out transfer info */           
            xfer.data = &uartTxBfr[0];
            xfer.dataSize = (size_t)( getMessageSize( msg.header.command ) + 6 ); /* crc + ext */
            /* send the command */
            status_t result = LPUART_TransferSendNonBlocking( LPUART1, &cutterHandle_, &xfer );
            if( result != kStatus_Success ) {
                PRINTF("sendACTestMode(): Failed to send AC_RESET_!\r\n" );
            } else {
                sent = true;
            }
        } else {
            PRINTF("sendACTestMode(): Failed to take mutex!\r\n" );
        }
    } else {
        PRINTF("sendACTestMode(): Mutex is null!\r\n" );
    } 
    return sent;    
    
}

/******************************************************************************/
/*!   \fn unsigned char getMessageSize( unsigned char type )

      \brief
        This function returns the fixed size of the message based on the message 
        type. 
               
      \author
          Aaron Swift
*******************************************************************************/                
unsigned char getMessageSize( unsigned char type )
{
    unsigned char size = 0;
   
    if( type == CUTTER_CUT_PAPER ) {
        size = PAPER_CUT_MSG_SIZE_;
    } else if( type == CUTTER_RETURN_HOME ) {
        size = RETURN_HOME_MSG_SIZE_;
    } else if( type == CUTTER_CUT_PAPER_STEPS ) {
        size = PAPER_CUT_STEPS_MSG_SIZE_;
    } else if( type == CUTTER_READ_SPEED ) {
        size = READ_PROFILE_MSG_SIZE_;
    } else if( type == CUTTER_RESET ) {
        size = DEFAULT_MSG_SIZE_;
    } else if( type == CUTTER_STATUS ) {
        size = DEFAULT_MSG_SIZE_;
    } else if( type == CUTTER_TEST_MODE ) {
        size = DEFAULT_MSG_SIZE_;
    } else if( type == CUTTER_VERSION ) {
        size = READ_VERSION_MSG_SIZE_;
    } else {
        PRINTF("getMessageSize(): Unknown message type: %d!\r\n", type );
    }
    return size;
}

/******************************************************************************/
/*!   \fn static void averyCutterTask( void *pvParameters )

      \brief
        This function handles initializing the cutter mechanism and processes  
        requests from the printer manager.
       
      \author
          Aaron Swift
*******************************************************************************/                
static void averyCutterTask( void *pvParameters )
{
    static ACRVersion version;
    version.index = 0;
    
    PRINTF("averyCutterTask(): Thread running...\r\n" ); 
    while( !suspend_ ) {
        switch( actState_ )
        {
            case AC_INIT_INTERFACE_: {
                /* open interface to cutter mechanism */
                if( initCutter() ) {
                    /* set initialization sequence flag */
                    cutterMgr_.initSeq = true;
                    /* request the cutter status */
                    if( sendACReqStatus( &cutterMgr_ ) ) { 
                        receiveCutterMsg( DEFAULT_MSG_RX_SIZE_ );  
                        actState_ = AC_INIT_DEVICE_STATUS_;
                    } else {
                        PRINTF("averyCutterTask(): sendACReqStatus() failed!\r\n" ); 
                    }
                } else {
                    PRINTF("averyCutterTask(): initCutter() failed!\r\n" ); 
                }
                break;
            }
            case AC_INIT_DEVICE_STATUS_: {
                
                if( cutterMgr_.msgReady ) {
                    handleCutterMessage( &cutterMgr_, &uartRxBfr[0] );                   
                                       
                    if( ( cutterMgr_.error == AC_ERR_NONE_ ) || ( cutterMgr_.error == AC_ERR_DOOR_OPEN_ ) ) {                                              
                        /* status is good request version info from device */
                        if( sendACReqVersion( &cutterMgr_, &version ) ) {                            
                            receiveCutterMsg( getVersionRxMsgSize( version.index ) );       
                            cutterMgr_.initSeq = false;
                            actState_ = AC_INIT_DEVICE_RX_VERSION_;
                        } else {
                            PRINTF("averyCutterTask(): sendACReqVersion() failed!\r\n" ); 
                        }
                    } else {
                        /* crtitical errors */
                        if( ( cutterMgr_.error == AC_ERR_POWER_FAILURE_ ) ||                        
                            ( cutterMgr_.error == AC_ERR_MSG_FAILURE_ ) ) {                            
                                PRINTF("averyCutterTask(): initCutter() Critical Error!\r\n" );
                                actState_ = AC_ERROR_;
                            } else {
                                /* blade not home, home and continue with intialization */
                                actState_ = AC_PROCESS_HOME_;
                            }
                    }
                }
                break;
            }
            case AC_INIT_DEVICE_TX_VERSION_: {
                if( sendACReqVersion( &cutterMgr_, &version ) ) {                         
                    actState_ = AC_INIT_DEVICE_RX_VERSION_;
                } else {
                    PRINTF("averyCutterTask(): sendACReqVersion() failed!\r\n" ); 
                }
                break;
            }
            case AC_INIT_DEVICE_RX_VERSION_: {
                
                if( cutterMgr_.msgReady ) {
                    version.index++;
                    if( version.index != MAX_VERSION_INDEX ) {
                        receiveCutterMsg( getVersionRxMsgSize( version.index ) );                           
                    }
                    
                    handleCutterMessage( &cutterMgr_, &uartRxBfr[0] );                                       
                    
                    /* do we have all version info? */
                    if( version.index == MAX_VERSION_INDEX ) {
                        version.index = 0;
                        /* wait for door to close */
                        if( cutterMgr_.error == AC_ERR_DOOR_OPEN_ ) {
                            actState_ = AC_INIT_SEND_DOOR_STATUS_;
                            vTaskDelay( lightSleep );
                        } else {
                            /* tell the printer that we are ready to process cmds */
                            ICutterGeneric imsg;
                            imsg.msgType = _I_CUTTER_READY_FOR_CMD;
                            BaseType_t result = xQueueSend( pIPMsgQueue, (void *)&imsg, 0 );
                            if( result != pdPASS ) {
                                PRINTF("printerTask(): Failed to post Cutter message!\r\n" );       
                            }                            
                            actState_ = AC_WAIT_FOR_COMMAND_;
                             /* add to cycle cutter 
                            actState_ = AC_PROCESS_CUT_; */
                        }                        
                    } else {
                        /* get the next */                        
                        actState_ = AC_INIT_DEVICE_TX_VERSION_;
                    }
                }                
                break;
            }
            case AC_INIT_SEND_DOOR_STATUS_: {
                if( sendACReqStatus( &cutterMgr_ ) ) { 
                    receiveCutterMsg( DEFAULT_MSG_RX_SIZE_ );  
                    actState_ = AC_INIT_WAIT_DOOR_STATUS_;
                }                
                break;
            }
            
            case AC_INIT_WAIT_DOOR_STATUS_: {
                if( cutterMgr_.msgReady ) {
                    handleCutterMessage( &cutterMgr_, &uartRxBfr[0] );
                    /* if we have homed the blade then get version info */
                    if( cutterMgr_.error == AC_ERR_NONE_ ) {
                        receiveCutterMsg( getVersionRxMsgSize( version.index ) );       
                        actState_ = AC_WAIT_FOR_COMMAND_;
                    } else {
                        /* wait 100mS before sending next request */
                        vTaskDelay( lightSleep ); 
                        actState_ = AC_INIT_SEND_DOOR_STATUS_; 
                    }
                }
                break;
            }
            
            case AC_WAIT_FOR_COMMAND_: {
#if 1        
                ICMessages msg;
                /* wait for  message */
                //if( xQueueReceive( pMsgQHandle_, &msg, portMAX_DELAY ) ) {           
                //    handleInternalMessage( &msg );    
                //} else {
                //    PRINTF("averyCutterTask(): Failed to get cutter message from queue!\r\n" );  
                //}
#else 
                /* TO DO: remove when ready */
                //vTaskDelay( deepSleep );               
#endif 
                break;
            }            
            case AC_PROCESS_CUT_ : {
                ACCutPaper steps;
                steps.distance = 80;
                steps.speed = 9;            
                if( sendACCut( &cutterMgr_, &steps ) )  {
                    receiveCutterMsg( PAPER_CUT_RX_STEPS_MSG_SIZE_ );                    
                    actState_ = AC_PROCESS_CUT_RESPONSE_;
                } else {
                    PRINTF("averyCutterTask(): Failed to send cut command!\r\n" );
                }                
                break;
            }
            
            case AC_PROCESS_CUT_RESPONSE_: {
                if( cutterMgr_.msgReady ) {
                    handleCutterMessage( &cutterMgr_, &uartRxBfr[0] );                            
                    /* have the cutter performed the cut */
                    if( cutterMgr_.error == AC_ERR_BLADE_NOT_HOME_ ) {
                        actState_ = AC_PROCESS_HOME_;
                        PRINTF("averyCutterTask(): Cutter has cut!\r\n" );
                    } else {
                        PRINTF("averyCutterTask(): Cutter failed to cut!\r\n" );
                    }
                }
                break;
            }
            
            case AC_PROCESS_HOME_: {
                ACHome home;
                home.speed = 9;
                /* home.speed = 4; real slow */ 
                if( sendACHome( &cutterMgr_, &home ) ) {
                    receiveCutterMsg( RETURN_HOME_RX_MSG_SIZE_  ); 
                    actState_ = AC_PROCESS_HOME_RESPONSE_;                    
                } else {
                    PRINTF("averyCutterTask(): Failed to send home command!\r\n" );
                }                
                break;
            }
            
            case AC_PROCESS_HOME_RESPONSE_: {
                //cutterMgr_.msgReady = true;
                if( cutterMgr_.msgReady ) {
                    handleCutterMessage( &cutterMgr_, &uartRxBfr[0] );
                    if( cutterMgr_.error == AC_ERR_NONE_ ) {
                        /* if we need to home cutter while in init */
                        if( cutterMgr_.initSeq ) {
                            actState_ =  AC_INIT_DEVICE_RX_VERSION_;
                        } else {
                            vTaskDelay( pdMS_TO_TICKS(300) );
                            /* tell the printer that we are ready to process cmds */
                            ICutterGeneric imsg;
                            imsg.msgType = _I_CUTTER_READY_FOR_CMD;
                            //BaseType_t result = xQueueSend( pIPMsgQueue, (void *)&imsg, 0 );
                            //if( result != pdPASS ) {
                            //    PRINTF("printerTask(): Failed to post Cutter message!\r\n" );       
                            //}                                                        
                            actState_ =  AC_WAIT_FOR_COMMAND_;
                        }
                        /* add to cycle cutter 
                        vTaskDelay( lightSleep ); 
                        actState_ = AC_PROCESS_CUT_; */
                    //} else {
                        //vTaskDelay( lightSleep );   
                        //actState_ = AC_PROCESS_HOME_RESPONSE_; 
                        /* request status */
                        //if( sendACReqStatus( &cutterMgr_ ) ) { 
                         //   receiveCutterMsg( DEFAULT_MSG_RX_SIZE_ );  
                        //}                        
                    }                    
                }                                
                break;
            }

            case AC_ERROR_: {
                /* TO DO: replace with task deletion */
                /* sleep for a second */
                vTaskDelay( deepSleep );
                break;
            }
            
        }
        taskYIELD();
    }
    vTaskSuspend(NULL);  
}

/******************************************************************************/
/*!   \fn void handleInternalMessage( ICMessages *pMsg )

      \brief
        This function handles internal cutter messsages from the Printer task.
       
      \author
          Aaron Swift
*******************************************************************************/      
void handleInternalMessage( ICMessages *pMsg )
{
    switch( pMsg->generic.msgType ) 
    {
        case _I_CUTTER_CUT_CMD: {
            actState_ = AC_PROCESS_CUT_;
            break;
        }
        case _I_CUTTER_HOME_CMD: {
            actState_ = AC_PROCESS_HOME_;
            break;
        }
        case _I_CUTTER_REQ_STATUS: {

            break;
        }
        case _I_CUTTER_REQ_VERSION: {

            break;
        }
        default: {
            break;
        }
    }
}

/******************************************************************************/
/*!   \fn static bool receiveCutterMsg( unsigned char rxSize  )

      \brief
        This function receives cutter message from uart1 non blocking. 
       
      \author
          Aaron Swift
*******************************************************************************/      
static bool receiveCutterMsg( unsigned char rxSize  )
{
    lpuart_transfer_t xfer;    
    size_t result, rxBytes = 0;
    bool ready = false;
    xfer.rxData = &uartRxBfr[0];
    xfer.dataSize = rxSize;
    

    result = LPUART_TransferReceiveNonBlocking( LPUART1, &cutterHandle_, &xfer, &rxBytes );
    if( result == kStatus_LPUART_RxBusy ) {
        PRINTF("receiveCutterMsg(): Receiver is busy!\r\n" );
    } else {
        ready = true;
    }
    return ready;
}

/******************************************************************************/
/*!   \fn static size_t getVersionRxMsgSize( unsigned char index )

      \brief
        This function returns version string size for given index. 
       
      \author
          Aaron Swift
*******************************************************************************/                
static size_t getVersionRxMsgSize( unsigned char index )
{
    size_t size = 0;
    if( index == 0 ) 
        size = READ_VERSION0_RX_MSG_SIZE_;
    if( index == 1 ) 
        size = READ_VERSION1_RX_MSG_SIZE_;
    if( index == 2 ) 
        size = READ_VERSION2_RX_MSG_SIZE_;
    if( index == 3 ) 
        size = READ_VERSION3_RX_MSG_SIZE_;
    if( index == 4 ) 
        size = READ_VERSION4_RX_MSG_SIZE_;
    return size;
}

/******************************************************************************/
/*!   \fn static void stuffTxBuffer( unsigned char *pMsg, unsigned char size )

      \brief
        This function copies the data into the transmit buffer for a given 
        message size.
       
      \author
          Aaron Swift
*******************************************************************************/                
static void stuffTxBuffer( unsigned char *pMsg, unsigned char size )
{
    if( pMsg != NULL ) {
        memcpy( &uartTxBfr[0], pMsg, size );
    } else {
        PRINTF("stuffTxBuffer(): pMsg is null!\r\n" );
    }
}

/******************************************************************************/
/*!   \fn static void buildCutterMsg( ACCCMDS cmd, ACutterMsgs *pMsg, 
                                      unsigned short arg1, unsigned short arg2 )
      \brief
        This function copies the data into the transmit buffer for a given 
        message size.
       
      \author
          Aaron Swift
*******************************************************************************/                
static void buildCutterMsg( ACCCMDS cmd, ACutterMsgs *pMsg, unsigned short arg1, 
                            unsigned short arg2 )
{
    unsigned char *pChar = (unsigned char *)&pMsg->header.config; 
    /* same args for all messages */
    pMsg->header.start          = AC_STX;      
    pMsg->header.slaveId        = CUTTER_SLAVE_ID;
    pMsg->header.type           = 'Q';
    /* all outgoing messages */
    pMsg->header.cr             = 'C';
    
    for( int i = 0; i < sizeof(long); i++ ) { 
        *pChar++                = '0';
    }
                    
    switch( cmd ) 
    {
        case AC_CUT_: {
            pMsg->header.type               = 'U';
            pMsg->header.command            = CUTTER_CUT_PAPER;
            pMsg->header.length             = PAPER_CUT_MSG_SIZE;
            
            /* bounds check our data */
            if( arg1 <= MAX_DISTANCE ) {
                hexToAscii( (unsigned char *)&pMsg->body.cut.distance, (unsigned char)arg1 ); 
            } else {
                PRINTF( "buildCutterMsg(): AC_CUT_ distance argument is out of bounds %d\r\n", arg1);
            }              
            if( arg2 <= MAX_SPEED ) {
                arg2 <<= 4;
                hexToAscii( (unsigned char *)&pMsg->body.cut.speed, (unsigned char)arg2  ); 
            } else {
                PRINTF( "buildCutterMsg(): AC_CUT_ speed argument is out of bounds %d\r\n", arg2);
            }                
            break;
        }
        case AC_HOME_: {
            pMsg->header.type               = 'U';
            pMsg->header.command            = CUTTER_RETURN_HOME;
            pMsg->header.length             = RETURN_HOME_MSG_SIZE;
            /* bounds check our data */
            if( arg1 <= MAX_SPEED ) {                
                //hexToAscii( (unsigned char *)&pMsg->body.home.speed, arg1 );
                pMsg->body.home.speed = ( '30' + arg1 );
            } else {
                PRINTF( "buildCutterMsg(): AC_HOME_ speed argument is out of bounds %d\r\n", arg1);
            }
            break;
        }
        case AC_CUT_STEPS_: {
            pMsg->header.type               = 'U';
            pMsg->header.command            = CUTTER_CUT_PAPER_STEPS;
            pMsg->header.length             = PAPER_CUT_STEPS_MSG_SIZE;
            /* bounds check our data */
            if( arg1 <= MAX_DISTANCE ) {
                hexToAscii( (unsigned char *)&pMsg->body.cutSteps.distance, arg1 ); 
            } else {
                PRINTF( "buildCutterMsg(): AC_CUT_STEPS_ distance argument is out of bounds %d\r\n", arg1);
            }                
            if( arg2 <= MAX_SPEED ) {
                arg2 <<= 4;
                hexToAscii( (unsigned char *)&pMsg->body.cutSteps.speed, arg2 ); 
            } else {
                PRINTF( "buildCutterMsg(): AC_CUT_STEPS_ speed argument is out of bounds %d\r\n", arg2);
            }                                
            break;
        }
        case AC_READ_SPEED_: {           
            pMsg->header.command            = CUTTER_READ_SPEED;
            pMsg->header.length             = READ_PROFILE_MSG_SIZE;
            if( arg1 <= MAX_MARK ) {                
                hexToAscii( (unsigned char *)&pMsg->body.profile.data, arg1 );
            } else {
                PRINTF( "buildCutterMsg(): AC_READ_SPEED_ mark argument is out of bounds %d\r\n", arg1);
            }                
            break;
        }
        case AC_RESET_: {
            pMsg->header.type               = 'U';
            pMsg->header.command            = CUTTER_RESET;
            pMsg->header.length             = DEFAULT_MSG_SIZE;
            break;
        }
        case AC_REQ_STATUS_: {
            pMsg->header.command            = CUTTER_STATUS;
            pMsg->header.length             = DEFAULT_MSG_SIZE;
            break;
        }
        case AC_TEST_MODE_: {
            pMsg->header.command            = CUTTER_TEST_MODE;
            pMsg->header.length             = DEFAULT_MSG_SIZE;                
            break;
        }
        case AC_VERSION_: {
            pMsg->header.command            = CUTTER_VERSION;
            pMsg->header.length             = READ_VERSION_MSG_SIZE;
            if( arg1 <= MAX_VERSION_INDEX ) {
                pMsg->body.readVersion.index = ( '30' + arg1 );
            } else {
                PRINTF( "buildCutterMsg(): AC_VERSION_ version index argument is out of bounds %d\r\n", arg1);
            }
            break;
        }
        default: {
            PRINTF( "buildCutterMsg(): Unknown command %d\r\n", cmd );
        }
    }
}

/******************************************************************************/
/*!   \fn static void prepTransmitBfr( unsigned char *pBfr, unsigned char length )
                                     
      \brief
        This function preps the transmit buffer to send to device by calculating 
        payload crc and appending ending message character EXT.
       
      \author
          Aaron Swift
*******************************************************************************/                
static void prepTransmitBfr( unsigned char *pBfr, unsigned char length )
{
    unsigned short crc = calcCrc16( pBfr, length );
    /* offset to end of message */
    pBfr += length;
    unsigned char a = 0;
    a = (unsigned char)( ( crc & 0xff00 ) >> 8 );
    hexToAscii( pBfr++, a );
    pBfr++;
    a = (unsigned char)( crc & 0x00ff );
    hexToAscii( pBfr++, a );
    pBfr++;
    *pBfr = AC_ETX;
    
}

/******************************************************************************/
/*!   \fn static void hexToAscii( unsigned char *pChar, unsigned char data )
                                     
      \brief
        This function converts the data character into ascii and assigns to the
        character pointer.
       
      \author
          Aaron Swift
*******************************************************************************/                
static void hexToAscii( unsigned char *pChar, unsigned char data )
{
    unsigned char nibble = ( ( data & 0xF0 ) >> 4 );
    int i = 0;
    while( i < 2 ) {
        if( nibble < 10 ) {
            *pChar++ = '0' + nibble; 
        } else if( nibble < 16 ) {
            *pChar++ = nibble + 'A' - 10;
        } else {
            *pChar++ = '?'; 
        }
        i++;
        nibble = ( data & 0x0F );
    }
}

/******************************************************************************/
/*!   \fn static void asciiToChar( unsigned char c, unsigned char *pChar )
                                     
      \brief
        This function converts the nibblized ascii to binary
       
      \author
          Aaron Swift
*******************************************************************************/                
static unsigned char asciiToChar( unsigned char *pChar )
{
    unsigned char result;
    if( *(pChar + 1) >= '0' && *(pChar + 1) <= '9' ) {
        result = *(pChar + 1) - 0x30;
    }
    else if( *(pChar + 1) >= 'A' && *(pChar + 1) <= 'F' ) {										 		
        result = *(pChar + 1) - 0x37;
    } else {
        result = 0;
    }
      
    if( *pChar >= '0' && *pChar <= '9' ) {
        result += ( ( *pChar - 0x30 ) << 4 );
    } else if( *pChar >= 'A' && *pChar <= 'F' ) {										 		
        result += ( ( *pChar - 0x37 ) << 4 );
    } else {
        result = 0;
    }
    return result;       
}

/******************************************************************************/
/*!   \fn static void handleCutterMessage( CutterMgr *pMgr, unsigned char *pBfr )
                                     
      \brief
        This function parses the received cutter message.
               
      \author
          Aaron Swift
*******************************************************************************/                
static void handleCutterMessage( CutterMgr *pMgr, unsigned char *pBfr )
{
    ACHeader header;   
    if( parseMsgHeader( &header, pBfr ) ){
        /* index to the body of the message */
        pBfr += ( DEFAULT_MSG_SIZE_ + 1);
        switch( header.command )
        {
            case 'c': {
                PRINTF( "handleCutterMessage(): Cut c message %d\r\n", header.length );
                break;
            }
            case 'h': {
                PRINTF( "handleCutterMessage(): Home message %d\r\n", header.length );
                if( pMgr->error == AC_ERR_NONE_ ) {
                    PRINTF( "Cutter blade is home!\r\n");
                } else if( pMgr->error == AC_ERR_BLADE_NOT_HOME_ ) {
                    PRINTF( "Error: Cutter blade failed to home!\r\n");
                } else {
                
                }
                break;
            }
            case 'k': {
                PRINTF( "handleCutterMessage(): Cut k message %d\r\n", header.length );
                break;
            }
            case 'p': {
                PRINTF( "handleCutterMessage(): Profile message %d\r\n", header.length );
                break;
            }
            case 'r': {
                PRINTF( "handleCutterMessage(): Reset message %d\r\n", header.length );
                break;
            }
            case 's': {
                PRINTF( "handleCutterMessage(): Status message %d\r\n", header.length ); 
                showDeviceStatus( pMgr->deviceStatus );
                break;
            }
            case 't': {
                PRINTF( "handleCutterMessage(): Test mode message %d\r\n", header.length ); 
                break;
            }
            case 'v': {
                PRINTF( "handleCutterMessage(): Version message %d\r\n", header.length );                
                
                /* parse the version message based on message length */
                if( header.length == READ_VERSION0_RX_MSG_SIZE_ ) {
                    parseVerPoductMsg( pMgr, pBfr );
                } else if( header.length == READ_VERSION1_RX_MSG_SIZE_ ) {
                    parseVerFirmNumbMsg( pMgr, pBfr );
                } else if( header.length == READ_VERSION2_RX_MSG_SIZE_ ) {
                    parseVerIssueMsg( pMgr, pBfr );
                } else if( header.length == READ_VERSION3_RX_MSG_SIZE_ ) {
                    parseVerDateMsg( pMgr, pBfr );
                    showCutterVersion( pMgr );
                } else {
                    PRINTF( "handleCutterMessage(): Unknown version msg! %d\r\n", header.length );
                }
                break;
            }
            default: {
                PRINTF( "handleCutterMessage(): Unknown msg! %d\r\n", header.length );
            }

        }
    }
    /* clear our msg flag */
    pMgr->msgReady = false;
    /* finished with message, clear our message buffer */
    memset( &uartRxBfr[0], 0xff, MAX_MESSGAE_SIZE );   
}

/******************************************************************************/
/*!   \fn static bool parseMsgHeader( ACHeader *pHeader, unsigned char *pMsg )
                                     
      \brief
        This function parses the received message header.
               
      \author
          Aaron Swift
*******************************************************************************/                
static bool parseMsgHeader( ACHeader *pHeader, unsigned char *pMsg )
{   
    bool valid = false;
    pHeader->start      = *pMsg++; 
    
    unsigned char b[] = { 0, 0 };    
    b[0] = *pMsg++;
    pHeader->slaveId    = atoi( (const char *)&b[0] );   
    
    pHeader->command    = *pMsg++;    
    pHeader->type       = *pMsg++;    
    pHeader->cr         = *pMsg++;    
    
    //unsigned char c = asciiToChar( pMsg++ );
    unsigned char c = *pMsg++ - '30'; 
    *pMsg++;
    *pMsg++;
    //unsigned char x = asciiToChar( pMsg++ );
    unsigned char x = *pMsg++ - '30';        
    *pMsg++;
    *pMsg++;
    
    pHeader->config = c;
    pHeader->config <<= 12;
    pHeader->config |= x;
    
    parseHeaderStatus( &cutterMgr_, pHeader );
    
    pHeader->length = (unsigned long)asciiToChar( pMsg++ );    
    
    if( ( pHeader->start == AC_STX ) && ( pHeader->slaveId == 1 ) && 
        (  pHeader->cr == 'R' ) ) {
        valid = true;
    }
    return valid;
}

/******************************************************************************/
/*!   \fn static void parseHeaderStatus( CutterMgr *pMgr, ACHeader *pHeader )
                                     
      \brief
        This function parses the status and updates the cutter manager.
               
      \author
          Aaron Swift
*******************************************************************************/                
static void parseHeaderStatus( CutterMgr *pMgr, ACHeader *pHeader )
{
    /* set any errors recorded in the status */
    if( ( pHeader->config & AC_POWER_FAILURE ) == AC_POWER_FAILURE ) {
        pMgr->error = AC_ERR_POWER_FAILURE_;    
    } else {
        if( pMgr->error == AC_ERR_POWER_FAILURE_ )
            pMgr->error = AC_ERR_NONE_;
    }
    
    if( ( pHeader->config & AC_MSG_FAILURE ) == AC_MSG_FAILURE ) {
        if( pMgr->error == AC_ERR_NONE_ )
            pMgr->error = AC_ERR_MSG_FAILURE_;
    } else {
        if( pMgr->error == AC_ERR_MSG_FAILURE_ )
            pMgr->error = AC_ERR_NONE_;    
    }
    
    if( ( pHeader->config & AC_BLADE_HOME ) != AC_BLADE_HOME ) {
        if( pMgr->error == AC_ERR_NONE_ )
            pMgr->error = AC_ERR_BLADE_NOT_HOME_;
    } else {
        if( pMgr->error == AC_ERR_BLADE_NOT_HOME_ )
            pMgr->error = AC_ERR_NONE_;        
    }
    
    if( ( pHeader->config & AC_TIME_OUT_ERROR ) == AC_TIME_OUT_ERROR ) {        
        if( pMgr->error == AC_ERR_NONE_ )
            pMgr->error = AC_ERR_TIMEOUT_;
    } else {
        if( pMgr->error == AC_ERR_TIMEOUT_ )
            pMgr->error = AC_ERR_NONE_;            
    }
    
    if( ( pHeader->config & AC_DOOR_OPEN ) == AC_DOOR_OPEN ) {
        if( pMgr->error == AC_ERR_NONE_ )
            pMgr->error = AC_ERR_DOOR_OPEN_;
    } else {
        if( pMgr->error == AC_ERR_DOOR_OPEN_ )
            pMgr->error = AC_ERR_NONE_;                
    }

    pMgr->deviceStatus = (unsigned short)pHeader->config;
    
    /* if we have errors then show */
    if( pMgr->error != AC_ERR_NONE_ ) {        
        showDeviceStatus( pMgr->deviceStatus );
    } else {
        PRINTF("Cutter status:  ready\r\n");    
    }
    
    /* determine our platten type */
    if( ( pHeader->config & PLATTEN_FULL_TYPE ) == PLATTEN_FULL_TYPE ) {
        pMgr->platten = FULL_PLATTEN_;
    } else {
        pMgr->platten = SLOTTED_PLATTEN_;
    }    
}

/******************************************************************************/
/*!   \fn static void parseVerPoductMsg( CutterMgr *pMgr, unsigned char *pMsg )

      \brief
        This function parses the product portion of the version message and 
        updates the cutter manager. 
      \note
        This function assumes the message buffer has been indexed to the start 
        of the body of the message. 
      \author
          Aaron Swift
*******************************************************************************/                        
static void parseVerPoductMsg( CutterMgr *pMgr, unsigned char *pMsg )
{    
    memcpy( &pMgr->version.productName[0], pMsg, MAX_PRODUCT_NAME_SIZE );    
}

/******************************************************************************/
/*!   \fn static void parseVerFirmNumbMsg( CutterMgr *pMgr, unsigned char *pMsg )

      \brief
        This function parses the firmware number portion of the version message and 
        updates the cutter manager. 
      \note
        This function assumes the message buffer has been indexed to the start 
        of the body of the message. 
       
      \author
          Aaron Swift
*******************************************************************************/                        
static void parseVerFirmNumbMsg( CutterMgr *pMgr, unsigned char *pMsg )
{
    memcpy( &pMgr->version.firmware[0], pMsg, MAX_FIRMWARE_NUM_SIZE );     
}

/******************************************************************************/
/*!   \fn static void parseVerIssueMsg( CutterMgr *pMgr, unsigned char *pMsg )

      \brief
        This function parses the issue portion of the version message and 
        updates the cutter manager. 
      \note
        This function assumes the message buffer has been indexed to the start 
        of the body of the message. 
       
      \author
          Aaron Swift
*******************************************************************************/                        
static void parseVerIssueMsg( CutterMgr *pMgr, unsigned char *pMsg )
{
    memcpy( &pMgr->version.issueDate[0], pMsg, MAX_FIRMWARE_ISSUE_SIZE );     
}

/******************************************************************************/
/*!   \fn static void parseVerDateMsg( CutterMgr *pMgr, unsigned char *pMsg )

      \brief
        This function parses the date portion of the version message and 
        updates the cutter manager. 
      \note
        This function assumes the message buffer has been indexed to the start 
        of the body of the message. 
       
      \author
          Aaron Swift
*******************************************************************************/                        
static void parseVerDateMsg( CutterMgr *pMgr, unsigned char *pMsg )
{
    memcpy( &pMgr->version.date[0], pMsg, MAX_DATE_SIZE );     
}
        
/******************************************************************************/
/*!   \fn static void showDeviceStatus( unsigned short status )

      \brief
        This function parses and prints the status bits from the cutter. 
       
      \author
          Aaron Swift
*******************************************************************************/                
static void showDeviceStatus( unsigned short status )
{
    PRINTF("showDeviceStatus(): cutter status 0x%02x\r\n", status );
    if( ( status & AC_POWER_FAILURE ) == AC_POWER_FAILURE ) 
        PRINTF("Cutter Error: ac power failure\r\n");
    
    if( ( status & AC_MSG_FAILURE ) == AC_MSG_FAILURE )
        PRINTF("Cutter Error: message failure\r\n");
    
    if( ( status & AC_REBOOT_FLAG ) == AC_REBOOT_FLAG ) 
        PRINTF("Cutter Warning: boot flag\r\n");
    
    if( ( status & AC_BLADE_HOME ) == AC_BLADE_HOME ) 
        PRINTF("Cutter Blade: is home\r\n");
    
    if( ( status & AC_TIME_OUT_ERROR ) == AC_TIME_OUT_ERROR ) 
        PRINTF("Cutter Error: cutter timeout\r\n");
        
    if( ( status & PLATTEN_FULL_TYPE ) == PLATTEN_FULL_TYPE ) {
        PRINTF("Cutter Platten Type: full\r\n");
    } else {
        PRINTF("Cutter Platten Type: slotted \r\n");
    }
            
    if( ( status & AC_DOOR_OPEN ) == AC_DOOR_OPEN ) {
        PRINTF("Cutter Error: door is open\r\n");
    } 
}

/******************************************************************************/
/*!   \fn static void showDeviceStatus( unsigned short status )

      \brief
        This function prints the cutter version strings. 
       
      \author
          Aaron Swift
*******************************************************************************/                
static void showCutterVersion( CutterMgr *pMgr )
{
    PRINTF("Cutter product name: %s\r\n", &(pMgr->version.productName[0]) );    
    PRINTF("Cutter model: %s\r\n", &(pMgr->version.firmware[0]) );    
    PRINTF("Cutter firmware version: %s\r\n", &(pMgr->version.issueDate[0]) );    
    PRINTF("Cutter issue date: %s\r\n", &(pMgr->version.date[0]) );    
}

/******************************************************************************/
/*!   \fn static bool openCutterInterface( unsigned short baud ) 

      \brief
        This function opens the uart interface to the cutter. 
       
      \author
          Aaron Swift
*******************************************************************************/                
static bool openCutterInterface( unsigned short baud )
{
    lpuart_config_t     config;    
    bool                status = false;
    
    LPUART_GetDefaultConfig( &config );
    config.baudRate_Bps         = baud;
    config.dataBitsCount        = kLPUART_EightDataBits;
    config.parityMode           = kLPUART_ParityEven;
    config.stopBitCount         = kLPUART_OneStopBit;
    config.enableTx             = true;
    config.enableRx             = true;

    long initStatus = LPUART_Init( LPUART1, &config, BOARD_DebugConsoleSrcFreq() );
    if( initStatus == kStatus_Success ) {
        LPUART_TransferCreateHandle( LPUART1, &cutterHandle_, uart1Callback, NULL );
        status = true;
    } else {
        PRINTF("openCutterInterface(): Could not open interface!\r\n" );   
    }
    NVIC_SetPriority( LPUART1_IRQn, 5 );
    return status;        
}

/******************************************************************************/
/*!   \fn bool isCutterInterlockClosed( void )

        \brief
        This function returns true if the label cutter interlock is closed.

        \author
        Aaron Swift
*******************************************************************************/
bool isCutterInterlockClosed( void )
{  
    return ( ( ( cutterMgr_.deviceStatus & AC_DOOR_OPEN ) != AC_DOOR_OPEN ) );
}

/******************************************************************************/
/*!   \fn static void uart1Callback( LPUART_Type *base, lpuart_handle_t *pHandle, 
                                     status_t status, void *pData )

      \brief
        This function is the callback for uart2. 
       
      \author
          Aaron Swift
*******************************************************************************/                
void uart1Callback( LPUART_Type *base, lpuart_handle_t *pHandle, 
                           status_t status, void *pData )
{
    if ( status == kStatus_LPUART_TxIdle ) {
        BaseType_t reschedule;
        
        cutterMgr_.txReady = true;
        memset( (void *)&uartTxBfr[0], 0, MAX_MESSGAE_SIZE );
        xSemaphoreGiveFromISR( cMutex_, &reschedule );
        
        portYIELD_FROM_ISR( reschedule );
    } else if( status == kStatus_LPUART_RxIdle ) {
        cutterMgr_.msgReady = true;

    } else if( ( status != kStatus_LPUART_TxBusy ) || 
              ( status != kStatus_LPUART_RxBusy ) ) {
#if 1                  
        if( status == kStatus_LPUART_Error  )
            PRINTF("uart1Callback(): Error: %d!\r\n", status );
        if( status == kStatus_LPUART_Error  )
            PRINTF("uart1Callback(): Error: %d!\r\n", status );            
        if( status == kStatus_LPUART_RxRingBufferOverrun  )
            PRINTF("uart1Callback(): kStatus_LPUART_RxRingBufferOverrun: %d!\r\n", status );            
        if( status == kStatus_LPUART_RxHardwareOverrun  )
            PRINTF("uart1Callback(): kStatus_LPUART_RxHardwareOverrun: %d!\r\n", status );            
        if( status == kStatus_LPUART_NoiseError  )
            PRINTF("uart1Callback(): kStatus_LPUART_NoiseError: %d!\r\n", status );            
        if( status == kStatus_LPUART_FramingError  )
            PRINTF("uart1Callback(): kStatus_LPUART_FramingError: %d!\r\n", status );            
        if( status == kStatus_LPUART_ParityError  )
            PRINTF("uart1Callback(): kStatus_LPUART_ParityError: %d!\r\n", status );            
        if( status == kStatus_LPUART_Timeout  )
            PRINTF("uart1Callback(): kStatus_LPUART_Timeout: %d!\r\n", status );                        
#endif    
      }
}

/******************************************************************************/
/*!   \fn static unsigned short calcCrc16( unsigned char *pBlob, unsigned short length )

      \brief
        This function calculates the message 16 bit crc value.
        The value is calculated starting after the STX and excluding the 
        CRC and ETX fields.
       
      \author
          Aaron Swift
*******************************************************************************/                
static unsigned short calcCrc16( unsigned char *pBlob, unsigned short length )
{
    unsigned short checkSum = 0;
    unsigned char index = 0;
    while( length-- ) {              
        /* reverse order checksum */
        checkSum = ( checkSum >> 8 )^crc16[ (unsigned char)( checkSum^ *pBlob++ ) ];    
    }
    return checkSum;       
}