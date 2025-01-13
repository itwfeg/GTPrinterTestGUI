#include "printerTask.h"
#include <stdlib.h>
#include "fsl_debug_console.h"
#include "fsl_flexcan.h"
#include "fsl_uart.h"
#include "serialFlash.h"
#include "sst25vf010a.h"
#include "threadManager.h"
#include "queueManager.h"
#include "systemTimer.h"
#include "translator.h"
#include "printHead.h"
#include "commandTable.h"
#include "printEngine.h"
#include "rs485.h"
#include "lp5521.h"
#include "sensors.h"
#include "cutter.h"
#include "vendor.h"
#include "k64_api.h"
#include "dotWearTask.h"
#include "sram23lcx.h"


Pr_Config config_;
int labelSize;
static bool suspend_ = false;
static TaskHandle_t pHandle_    = NULL;
static QueueHandle_t pMsgQHandle_   = NULL;
static SemaphoreHandle_t pCutSemaphore      = NULL;
static QueueSetHandle_t pQueueSet_      = NULL;

static bool configValid         = false;
static bool hostConnected_      = false;

static PrVersion prVersion_;
static PrinterStyle style_;
static PrinterOrder instance_;
static PrintMode printMode_;
static TimerHandle_t pWTimer_;

static bool activeMonitor_                      = false;
static bool stopTest_                           = false;
static ActiveTest maintenanceTest_              = _UNKNOWN_ACTIVE_TEST;

extern PrStatusInfo currentStatus, prevStatus;
extern void initializeCmdSequence( CMDId id, PrStatusInfo *pStatus );
extern void setOperation( unsigned char operation, PrStatusInfo *pStatus );
extern void clearPrevVertOffset( void );
extern Driver_Instance_Table_t inst;
extern const unsigned char testLabel[];
extern const unsigned char testLabel2[];
extern const unsigned char testLabel3[];
extern const unsigned char testLabel4[];
extern const unsigned char testLabel5[];


#ifdef CUTTER_LIFE_TEST
extern SemaphoreHandle_t pCutDoneSemaphore;
#endif


/******************************************************************************/
/*!   \fn BaseType_t createPrinterTask( void )                                                            
 
      \brief
        This function initializes printer resources and creates 
        printer task thread.
       
      \author
          Aaron Swift
*******************************************************************************/ 
BaseType_t createPrinterTask( PrinterStyle style )
{
    BaseType_t result;
    FPMBLC3Checksums    checkSums;
    UINT checksum = 0;
    labelSize = 0;
    /* set our style */
    style_ = style;
    
    /* set our version information */
    getPrVersion( &prVersion_ );
    
    /* clear our configuration and status */
    memset( &config_, 0, sizeof(PrConfig) );
    memset( &currentStatus, 0, sizeof(PrStatusInfo) );
    memset( &prevStatus, 0, sizeof(PrStatusInfo) );

    /* set our print mode to portrait */
    printMode_ = PRINTER_PORTRAIT_MODE;
    /* added to remove compiler warning. print mode is always set to portrait */
    if( printMode_ != PRINTER_PORTRAIT_MODE ) {
        PRINTF("printerTask(): printMode_  Not Portrait!" );
    }
    
    /* clear our command tables */
    initializeTable();

    currentStatus.mask.sensor2 = ( OUT_OF_DATA_BUFFERS | LOW_STOCK_REACHED | OUT_OF_STOCK );
    prevStatus.mask.sensor2 = ( OUT_OF_DATA_BUFFERS | LOW_STOCK_REACHED | OUT_OF_STOCK );

    if( style_ == K_PRINTER_SERVICE_SCALE ) {
        createFanTimer();
    }
      
    if( createAveryCutterTask() != pdPASS ) {
        PRINTF("createPrinterTask(): failed to create cutter task!\r\n");
    }

    /*read the printer configuration from the serial flash */
    if( getSerialPrConfiguration( &config_ ) ) {                
        if( getPageChecksums( &checkSums ) ) {
            checksum = calculateChecksum((void *)&config_, sizeof(Pr_Config));
            configValid = true;
        } else {
            /*failed to read checksums*/
            checkSums.prConfigSum = 0;
            configValid = false;
        }
    } else {
        sendPrError( _PR_CFG_READ_FAILURE, false );
        /*failed to read configuration from serial flash*/
        PRINTF("printerTask(): failed to read configuration from serial flash!\r\n"); 
        
        checkSums.prConfigSum = 0;
        configValid = false;
    }

    if ( ( checksum != checkSums.prConfigSum ) || ( !configValid ) ) {
        
        /*set printer configuration to it's defaults */
        if( setSerialPrDfltConfiguration( &config_ ) ){
            checkSums.prConfigSum = calculateChecksum ( &config_, sizeof(Pr_Config) );
            
            /*set the new default checksum to the serial flash*/
            setPageChecksums(&checkSums);
            configValid  = true;
            
            PRINTF("printerTask(): Default configuration loaded!\r\n" );
        } else {
            configValid  = false;
        }
    } else {
        PRINTF("printerTask(): Configuration valid!\r\n" );
        configValid = true;
        instance_ = config_.instance; 
    }
    if( ( config_.contrast_adjustment > 7 ) || ( config_.contrast_adjustment < 0 ) ) {
        config_.contrast_adjustment = 3;
    }
    /* initialize our printhead */
    if( style_ == K_PRINTER_SERVICE_SCALE )
        intializePrintHead( config_.contrast_adjustment, K_SERVICE );
    else
        intializePrintHead( config_.contrast_adjustment, K_PREPACK );
    
    /* am i a freestanding scale */
    if( getMyModel() == K_FSSS ) {
        initLp5521();

        /* set label taken bias */
        if( !setLabelTakenCurrent( CC_THREE_POINT_ZERO  ) ) {   
            PRINTF("printerTask(): failed to setup gap sensor!\r\n" );
        }
        
        if( !setGapCurrent( config_.media_sensor_adjustment ) ) {
            PRINTF("printerTask(): failed to setup gap sensor!\r\n" );
        }
        
        setLowStockCurrent( CC_TWENTY_FIVE_POINT_FIVE );
        setLowStockDutyCycle( DC_90_PRECENT );           
    }        
            
    /* assigning the local task queues */
    assignPrinterMsgQueue( getPrinterQueueHandle());
    
    /* Create the semaphore that is being added to the set. */
    pCutSemaphore = xSemaphoreCreateBinary();
    
    /* determine length of all printer queues combined */
    unsigned long pQSetLength = ( getPrinterMsgQueueLength() + 1 + 1);
    
    /* create printer queue set */
    pQueueSet_ = xQueueCreateSet( pQSetLength );
    
    if((pMsgQHandle_ != NULL) && (pCutSemaphore != NULL) ){
        xQueueAddToSet(pMsgQHandle_, pQueueSet_);
        xQueueAddToSet(pCutSemaphore, pQueueSet_);
    }else{
        sendPrError( _PR_QUEUE_SET_FAILURE, true);
        PRINTF("createPrinterTask(): Critical Error queue set not created!\r\n" );
    }
    
    /* initialize the print engine */
    initializePrintEngine( config_.contrast_adjustment, config_.out_of_media_count, getPrCommandQueueHandle() );
   
    initializePwm();
        
    /* assigning the local task queues */
    assignPrinterMsgQueue( getPrinterQueueHandle() );
    
    /* open rs485 channel to host */
    initializeRS485();

    /* create timer for wakeup message */  
    pWTimer_  = xTimerCreate( "prWakeTimer", (TickType_t)500, pdTRUE, ( void * ) 0, prWakeupCallBack );
    if( pWTimer_ == NULL ) {
        sendPrError( _PR_WAKE_TIMER_INIT_FAILURE, false );
        PRINTF("printerTask(): Critical Error wakeup timer not created!\r\n" );
    }   
    
    if( getPrCommandQueueHandle() != NULL ) {
        /* create printer task thread */
        result = xTaskCreate( printerTask,  "PrinterTask", 2000/* configMINIMAL_STACK_SIZE */,
                                            NULL, printer_task_PRIORITY, &pHandle_ );
    } else {
        sendPrError( _PR_CMD_QUEUE_NULL, false );
        PRINTF("printerTask(): Command queue is NULL! Printer task not created!\r\n" );
    }
    initializeCutter();
    

    PrInfo prInfo;
    if( getSerialPrInfo( &prInfo ) ) {
        /* has the printer info been programmed? */ 
        if( prInfo.cutterInstalled != CUTTER_MAGIC_KEY_BANK ) {
            if( prInfo.cutterInstalled == CUTTER_MAGIC_KEY_INSTALLED ) {
                setCutterInstalled();            
            } 
        } else { 
            /*printer info has not been programmed. 
              detect if a cutter is installed and set magic key*/
            if( isCutterInstalled() ) {
                prInfo.cutterInstalled = CUTTER_MAGIC_KEY_INSTALLED;
                prInfo.dateOfManufacture = 1602603274;
                prInfo.numOfPrints = 0;    
                
                if( !setSerialPrInfo( &prInfo ) ) {
                    PRINTF("printerTask(): Critical Error printer info not saved!\r\n" );
                }                 
            } else {
                prInfo.cutterInstalled = CUTTER_MAGIC_KEY_NOT_INSTALLED;
                prInfo.dateOfManufacture = 1602603274;
                prInfo.numOfPrints = 0;    

                if( !setSerialPrInfo( &prInfo ) ) {
                    PRINTF("printerTask(): Critical Error printer info not saved!\r\n" );                    
                }                 
            }            
        }
    } else {
        PRINTF("printerTask(): failed to read printer info from serial flash!\r\n"); 
    }
    /* setup fan control */
    gpio_pin_config_t config = { kGPIO_DigitalOutput, 0, };
    GPIO_PinInit( FAN_OFF_GPIOx, FAN_OFF_PINx, &config );
    GPIO_PinInit( FAN_FULL_SPEED_GPIOx, FAN_FULL_SPEED_PINx, &config );

    GPIO_WritePinOutput(FAN_OFF_GPIOx, FAN_OFF_PINx, true);
    GPIO_WritePinOutput(FAN_FULL_SPEED_GPIOx, FAN_FULL_SPEED_PINx, true);

    GPIO_WritePinOutput(FAN_FULL_SPEED_GPIOx, FAN_FULL_SPEED_PINx, false);
   
    /* load zero into the printhead to avoid current spikes after boot. */
    setHeadPower(1);
    loadZeroPrintLine();
    setHeadPower(0);
        
    return result; 
}

/******************************************************************************/
/*!   \fn SemaphoreHandle_t getCutSemaphore( void );                                                           
 
      \brief
        This function returns the handle to the cutSemaphore  
       
      \author
          Eric Landes
*******************************************************************************/ 
SemaphoreHandle_t getCutSemaphore( void )
{
  return pCutSemaphore;
}

SemaphoreHandle_t getCutSemaphore( void );
/******************************************************************************/
/*!   \fn void assignPrinterMsgQueue( QueueHandle_t pQHandle )                                                           
 
      \brief
        This function returns true if our style ( model ) is service scale.  
       
      \author
          Aaron Swift
*******************************************************************************/ 
bool isServiceScale( void )
{
    bool serviceScale = false;
    if( style_ == K_PRINTER_SERVICE_SCALE ) {
        serviceScale = true;
    }
    return serviceScale;
}

/******************************************************************************/
/*!   \fn void assignPrinterMsgQueue( QueueHandle_t pQHandle )                                                           
 
      \brief
        This function sets the printer message queue 
       
      \author
          Aaron Swift
*******************************************************************************/ 
void assignPrinterMsgQueue( QueueHandle_t pQHandle )
{
    if( pQHandle != NULL ) {
        pMsgQHandle_ = pQHandle;
    } else {
        sendPrError( _PR_MSG_QUEUE_NULL, false );
        PRINTF("assignPrinterMsgQueue(): Msg queue is NULL!\r\n" );
    }
}

/******************************************************************************/
/*!   \fn TaskHandle_t getPrinterHandle( void )                                                          
 
      \brief
        This function returns the printer task handle.
       
      \author
          Aaron Swift
*******************************************************************************/ 
TaskHandle_t getPrinterHandle( void )
{
    return pHandle_;
}

/******************************************************************************/
/*!   \fn static void printerTask( void *pvParameters )                                                         
 
      \brief
        This function is the task thread for the printer. The task waits for 
        a select from the queue set and evaluates and processes.
       
      \author
          Aaron Swift
*******************************************************************************/ 
static void printerTask( void *pvParameters )
{
    PrMessage prMsg; 
    QueueSetMemberHandle_t setHandle = NULL;
    
    ( void ) pvParameters;
    
    PRINTF("printerTask(): Thread running...\r\n" );        
    
    /* start timer for wake message */
    if( pWTimer_ ) {
        startOneShot( pWTimer_ );
    }
#if CUTTER_LIFE_TEST
    long long cuts = 0;
    while(CUTTER_LIFE_TEST){
      if(currentStatus.error == NO_ERROR && (currentStatus.sensor & OUT_OF_MEDIA) != OUT_OF_MEDIA 
         && (currentStatus.sensor & LABEL_TAKEN) == LABEL_TAKEN && isCutterInterlockClosed() ){
          initializeCmdSequence( 5, &currentStatus );
          while(currentStatus.state != ENGINE_IDLE){
            taskYIELD();
          }
          cutterCut(true);
          xSemaphoreTake(pCutDoneSemaphore, portMAX_DELAY );
          initializeCmdSequence( 2, &currentStatus );
          while(currentStatus.state != ENGINE_IDLE){
            taskYIELD();
          }
          cuts++;
          PRINTF("-----------------------Number of Cuts %lld-----------------------\r\n",cuts);
          vTaskDelay(pdMS_TO_TICKS(1000));
      }
      readHeadUpSensor(&currentStatus);
      readLabelTakenSensor();
      processLabelTakenSensor(&currentStatus);
      taskYIELD();
    }
#endif 
    
#if FSSS_DRIVETRAIN_TEST
long long cntr_ = 0;

while( FSSS_DRIVETRAIN_TEST ){
    if( ( currentStatus.error == NO_ERROR ) && 
        ( ( currentStatus.sensor & OUT_OF_MEDIA ) != OUT_OF_MEDIA ) ) {
        if( isCutterInterlockClosed() ) {  
            initializeCmdSequence( 5, &currentStatus );
            while( currentStatus.state != ENGINE_IDLE ){
                taskYIELD();
            }
            
            vTaskDelay( pdMS_TO_TICKS( 100 ) );

            initializeCmdSequence( 2, &currentStatus );
            while(currentStatus.state != ENGINE_IDLE){
                taskYIELD();
            }
            cntr_++;
            PRINTF( "-----------------------Number of Cycles %lld-----------------------\r\n", cntr_ );
            vTaskDelay( pdMS_TO_TICKS( 1000 ) );
        } else {
            taskYIELD();    
        }
    }
    readHeadUpSensor(&currentStatus);
    readLabelTakenSensor();
    processLabelTakenSensor(&currentStatus);
    taskYIELD();    
}
#endif    
    

    while( !suspend_ ) {
        /* active monitoring is set when we need to monitor the print engine.
           this is used for monitoring cleaning printhead and life tests for QA.*/
        if( !activeMonitor_ ) {
            /* block until message */
            setHandle = xQueueSelectFromSet( pQueueSet_, portMAX_DELAY );
            if( setHandle == pMsgQHandle_){
              /* wait for host message */        
              if( xQueueReceive( setHandle, &prMsg, portMAX_DELAY ) ) { 
                  PrMessage newMsg;
                  memcpy( &newMsg, &prMsg, sizeof(PrMessage) );
                  handlePrinterMsg( &newMsg );                
              } else {
                  sendPrError( _PR_MSG_QUEUE_FAILURE, false );
                  PRINTF("printerTask(): Failed to get Printer message from queue!\r\n" );              
              } 
            } else if( setHandle == pCutSemaphore ) {
                if( xSemaphoreTake( setHandle, 0 ) == pdTRUE ) {
                    cutterCut(true);
                }                
            } else {
                PRINTF("PrinterTask(): Unknown queue handle!\r\n" );
            }              
        } else {
            /* non blocking  */
            unsigned long qItems = 0;
            /* do we have an incomming message to process */
            qItems = uxQueueMessagesWaiting( pMsgQHandle_ );
            if( qItems > 0 ) {
                if( xQueueReceive( setHandle, &prMsg, portMAX_DELAY ) ) { 
                    PrMessage newMsg;
                    memcpy( &newMsg, &prMsg, sizeof(PrMessage) );
                    handlePrinterMsg( &newMsg );                
                } else {
                    PRINTF("printerTask(): Failed to get Printer message from queue!\r\n" );              
                }             
            }
            monitorMaintenance();
        }
        taskYIELD();
    }
    vTaskSuspend(NULL);
}

/******************************************************************************/
/*!   \fn static void handlePrinterMsg( PrMessage *pMsg )                                                         
 
      \brief
        This function handles printer messages from the host.
       
      \author
          Aaron Swift
*******************************************************************************/ 
static void handlePrinterMsg( PrMessage *pMsg )
{
    static unsigned int rePostCntr_ = 0;    
    static GAPSteps step_ = _INIT;
    switch( pMsg->generic.function_code )
    {
        case PR_WAKEUP:
        {            
            PrWakeup wakeMsg;
            wakeMsg.pid = getProductId();
            wakeMsg.order = instance_;
            wakeMsg.printhead_size = PRINTER_HEAD_SIZE;
            wakeMsg.buffer_size = PRINTER_BUFFER_SIZE;
            wakeMsg.transfer_size = MAX_RS485_TRANAFER_SIZE;
           
            sendPrWakeup( &wakeMsg );
            break;
        }
        case PR_REQ_CONFIG:
        {
            PRINTF("handlePrinterMsg(): Processing message: PR_REQ_CONFIG \r\n");
            sendPrConfig( &config_ );
            break;
        }
        case PR_CONFIG:
        {
            PRINTF("handlePrinterMsg(): Processing message: PR_CONFIG \r\n");
            FPMBLC3Checksums sums;           
            if( pMsg->config.disposition == DEFAULT ) {
                
                if( setSerialPrDfltConfiguration( &config_ ) ) {
                    getPageChecksums( &sums );
                    sums.prConfigSum  = calculateChecksum( (void *)&config_, sizeof (Pr_Config) );
                    setPageChecksums( &sums );
                    
                    PRINTF("handlePrinterMsg(): default configuration saved.\r\n" );
                    
                    configValid = true;
                } else {                  
                    configValid = false;
                    sendPrError( _PR_DFLT_CFG_SAVE_FAILURE, false );
                    PRINTF("handlePrinterMsg(): default configuration failed to saved.\r\n" );
                }
                
            } else if( pMsg->config.disposition == PERMANENT ) {
                /* copy new config to our working config */
                config_.contrast_adjustment     = pMsg->config.config.contrast_adjustment;
                config_.printPosition           = pMsg->config.config.printPosition;
                config_.config_3                = pMsg->config.config.config_3;
                
                if( getMyModel() == K_FSSS ) {
                    /* make sure we write back our shootthrough values. */
                    pMsg->config.config.backingPaper        = config_.backingPaper;
                    pMsg->config.config.backingAndlabel     = config_.backingAndlabel;
                    pMsg->config.config.media_sensor_adjustment = config_.media_sensor_adjustment;
                }
                
                if( getPageChecksums( &sums ) ) {
                    /* copy the new configuration to the serial flash section for configuration */
                    if ( ! setSerialPrConfiguration( &(pMsg->config.config) ) ) {
                        /* serial flash write failed.... load defaults values */
                        setSerialPrDfltConfiguration( &config_ );
                        sums.prConfigSum  = 0;    /* section corruption indication. */
                        setPageChecksums( &sums );
                    } else {                     
                        sums.prConfigSum  = calculateChecksum( (void *)&config_, sizeof (Pr_Config) );
                        /* save the new checksum */
                        if( setPageChecksums( &sums ) ) {
                            configValid = true;
                            PRINTF("handlePrinterMsg(): configuration saved.\r\n" );
                        } else {
                            sendPrError( _PR_CFG_SAVE_FAILURE, false );
                            PRINTF("handlePrinterMsg(): configuration failed to saved.\r\n" );
                        }
                    }
                } else {
                    sendPrError( _PR_CHKSUM_READ_FAILURE, false );
                    PRINTF("handlePrinterMsg(): failed to get checksums!\r\n" );
                }
            }
            /* update print contrast */
            setEngineContrast( config_.contrast_adjustment );
            /* update the vertical print position */
            updateLabelAlignment();
            break;
        }
        case PR_FACTORY_DFLTS:
        {
            PRINTF("handlePrinterMsg(): Processing message: PR_FACTORY_DFLTS \r\n");
            /* erase our cutter installed bit */
            eraseSerialCutterBit();
            
            /* set default printer configuration */    
            FPMBLC3Checksums sums;
            if( setSerialPrDfltConfiguration( &config_ ) ) {
                /* update our checksums */
                getPageChecksums( &sums );
                sums.prConfigSum  = calculateChecksum( (void *)&config_, sizeof (Pr_Config) );
                /* set new checksum for defaults */
                setPageChecksums( &sums );
                /* clear and read config from flash */
                memset( &config_, 0, sizeof( Pr_Config ) );
                getSerialPrConfiguration( &config_ );
                if( getMyModel() == K_FSSS ) { 
                    if( ( config_.media_sensor_adjustment == 0 ) && ( config_.out_of_media_count == 200  ) &&
                        ( config_.config_3 == 150 ) ) {                
                        PRINTF("handlePrinterMsg(): factory default configuration saved.\r\n" );
                        sendPrFactoryDlftsComplete(true);
                    } else {
                        PRINTF("handlePrinterMsg(): factory default configuration failed.\r\n" );               
                        sendPrFactoryDlftsComplete(false);                
                    }

                } else {    
                    if( ( config_.media_sensor_adjustment == 31 ) && ( config_.out_of_media_count == 200  ) &&
                        ( config_.config_3 == 100 ) ) {                
                        PRINTF("handlePrinterMsg(): factory default configuration saved.\r\n" );
                        sendPrFactoryDlftsComplete(true);
                    } else {
                        PRINTF("handlePrinterMsg(): factory default configuration failed.\r\n" );               
                        sendPrFactoryDlftsComplete(false);                
                    }
                }
            } else {                
                PRINTF("handlePrinterMsg(): factory default configuration failed.\r\n" );               
                sendPrFactoryDlftsComplete(false);
            }
            /* update the vertical print position */
            updateLabelAlignment();
            break;
        }
        case PR_REQ_STATUS:
        {
            PRINTF("handlePrinterMsg(): Processing message: PR_REQ_STATUS \r\n");
            
            sendPrStatus( &currentStatus, false );  
            break;
        }
        case PR_REQ_SENSORS:
        {
            /* PRINTF("handlePrinterMsg(): Processing message: PR_REQ_SENSORS \r\n"); */
            
            PrSensors sensors;
            getCurrentSensors( &sensors );
            sendPrSensors( &sensors );
            break;
        }
        case PR_REQ_CALIBRATE:
        {
            PRINTF("handlePrinterMsg(): Processing message: PR_REQ_CALIBRATE \r\n");
            
            calibratePrinter( pMsg->calibrate.calibration );
            break;
        }
        case PR_REQ_VERSION:
        {
            PrHead head; 
            head.headType = getHeadStyleType();
            head.headSize = getHeadStyleSize();		  
		  
            PRINTF("handlePrinterMsg(): Processing message: PR_REQ_VERSION \r\n");
            
            sendPrVersion( &prVersion_ );	
            sendPrHeadType( &head );
			
            break;
        }
        case PR_REQ_HEAD_POWER:
        {
            PRINTF("handlePrinterMsg(): Processing message: PR_REQ_HEAD_POWER \r\n");            
            break;
        }
        case PR_MODE:
        {
            PRINTF("handlePrinterMsg(): Processing message: PR_MODE \r\n");
            
            printMode_ = pMsg->mode.mode;
            break;
        }
        case PR_RESET:
        {
            PRINTF("handlePrinterMsg(): Processing message: PR_RESET \r\n");
          
            resetPrinter(); 
            break;
        }
        case PR_ENABLE:
        {
            PRINTF("handlePrinterMsg(): Processing message: PR_ENABLE \r\n");
            
            currentStatus.command = ENABLE_COMMAND;
            /* set the idle operation. */
            setOperation( IDLE_DIRECTIVE, &currentStatus );
            break;
        }
        case PR_DISABLE:
        {
            PRINTF("handlePrinterMsg(): Processing message: PR_DISABLE \r\n");
            
            currentStatus.command = DISABLE_COMMAND;            
            /* set the disable operation. */
            setOperation( DISABLE_DIRECTIVE, &currentStatus );
            break;
        }
        case PR_POWER: 
        {            
            PRINTF("handlePrinterMsg(): Processing message: PR_POWER \r\n");
            
            setHeadPower( pMsg->power.state );          
            break;
        }
        case PR_SIZE:
        {
            PRINTF("handlePrinterMsg(): Processing message: PR_SIZE measured = %d actual %d\r\n", pMsg->size.measured_size, pMsg->size.actual_size );
            
            setLabelAlignment( pMsg->size.measured_size, pMsg->size.actual_size );
            labelSize = pMsg->size.actual_size;
            break;
        }
        case PR_LABEL_SIZE:
        {
            PRINTF("handlePrinterMsg(): Processing message: PR_MULT_BUFFERS %d \r\n", pMsg->labelSize.size );             
            setTotalLabelSize( pMsg->labelSize.size );
            break;
        }
        case PR_REQ_485_TRANSFER:
        {
            PRINTF("handlePrinterMsg(): Processing message: PR_REQ_485_TRANSFER \r\n");            
            
            /* clear our flag if set */
            currentStatus.sensor2 &= ~OUT_OF_DATA_BUFFERS; 
            compareStatus( &currentStatus, &prevStatus );

            if( openInterface() ) {

                /* set uart/dma channel to receive label image */
                status_t result = setUartDmaTransfer( pMsg->transfer.transferSize, pMsg->transfer.imageSize );
                if( ( result == kStatus_UART_RxBusy ) || ( result == kStatus_UART_RxRingBufferOverrun ) ||
                    ( result == kStatus_UART_RxRingBufferOverrun ) || ( result == kStatus_UART_RxHardwareOverrun ) ||
                    ( result == kStatus_UART_NoiseError ) || ( result == kStatus_UART_FramingError ) || 
                    ( result == kStatus_UART_ParityError ) ){
                    PRINTF("handlePrinterMsg(): Error: dma channel not open! %d \r\n", result); 
                }
                /* inform host we are ready for label image */
                sendPrTransferReady( true );
            } else {
                sendPrError( _PR_RS485_OPEN_FAILURE, false );
                PRINTF("handlePrinterMsg(): Error: unable to open interface! \r\n");     
            }
            break;
        }
        case PR_FLUSH:
        {
            PRINTF("handlePrinterMsg(): Processing message: PR_FLUSH \r\n");              
            break;
        }
        case PR_TEACH:
        {
            PRINTF("handlePrinterMsg(): Processing message: PR_TEACH\r\n");
            
            CmdOp *pOper = &pMsg->teach.operation[0];  
            /* freestanding scale */
            if( pMsg->teach.identifier != 8 ) {
                if( pMsg->teach.identifier != 7 ) { 
                    for(int i = 0; i < pMsg->teach.entries; i++ ) {                
                        buildCmdTable( pMsg->teach.identifier, i, pOper );              
                        pOper++;
                    }
                }
            }
            break;
        }
        case PR_MASK:
        {
            PRINTF("handlePrinterMsg(): Processing message: PR_MASK \r\n");

            currentStatus.mask.sensor = pMsg->mask.mask.sensor;
            currentStatus.mask.user   = pMsg->mask.mask.user;                    
            break;
        }
        case PR_TEST:
        {
            PRINTF("handlePrinterMsg(): Processing message: PR_TEST \r\n");
            PrCommand cmd;
            memset( &cmd, 0, sizeof(PrCommand) );
            /* check for shoot through gap test */ 
            if( pMsg->test.data == 1 ) {
                /* copy test table into reserved entry space */ 
                setTableTestCmds( _TESTGAP );
                cmd.identifier = 0;     /* reserved index */
                addCmdToQueue( &cmd );
            } else if( pMsg->test.data == 2 ) {
                /* copy test table into reserved entry space */ 
                setTableTestCmds( _TESTEDGE );
                cmd.identifier = 0;    /* reserved index */
                addCmdToQueue( &cmd );
            } else if( ( pMsg->test.data == _CUTTER_LIFE_TEST ) || 
                       ( pMsg->test.data == _DRIVETRAIN_LIFE_TEST ) || 
                       ( pMsg->test.data == _CLEANING_HEAD_TEST ) ) {
                maintenanceTest_ = (ActiveTest)pMsg->test.data;
                activeMonitor_ = true;
                stopTest_ = false;
                /* no table change for cutter and drivetrain tests */                
                if( pMsg->test.data == _CLEANING_HEAD_TEST ) {
                    setTableTestCmds( _TESTCLEANING );
                }
            } else {
                PRINTF("handlePrinterMsg(): unknown test: %d!\r\n", pMsg->test.data);
            }
            break;
        }
        case PR_RAM:
        {
            PRINTF("handlePrinterMsg(): Processing message: PR_RAM \r\n");
            
            updateRam( &(pMsg->ram) );
            break;
        }
        case PR_COMMAND:
        {
            if( ( ( pMsg->command.identifier == 4 ) && ( getPacketTransferTotal() <= 1 ) ) && ( rePostCntr_ <= 10 ) ) {                  
                /* give the rs485 transfer time to transfer at least 2 payloads before we start printing. */
                /* PRINTF("handlePrinterMsg(): repost print cmd!\r\n"); */
                rePostCntr_++;
                xQueueSend( pMsgQHandle_, pMsg, 0 );
            } else { 
                if( rePostCntr_ >= 10 ) {
                    rePostCntr_ = 0;
                }
                PRINTF("handlePrinterMsg(): Processing message: PR_COMMAND command.identifier %d\r\n", pMsg->command.identifier );
                PRINTF("handlePrinterMsg(): Processing message: PR_COMMAND command.function_code %d\r\n", pMsg->command.function_code );
                PRINTF("handlePrinterMsg(): Processing message: PR_COMMAND command.data_item %d\r\n", pMsg->command.data_item );
                PRINTF("handlePrinterMsg(): Processing message: PR_COMMAND command.options %d\r\n", pMsg->command.options );
                PRINTF("handlePrinterMsg(): Processing message: PR_COMMAND command.value %d\r\n", pMsg->command.value );
               
                if( getMyModel() != K_FSSS ) { 
                    addCmdToQueue( &pMsg->command );
                } else { 
                    if( pMsg->command.identifier == 8 ) {
                        /* change command for shoot through sizing */
                        pMsg->command.identifier = 12;

                        /* re-apply vertical offset since we are sizing */
                        clearPrevVertOffset();
                        addCmdToQueue( &pMsg->command );
                    } else {
                        addCmdToQueue( &pMsg->command );
                    }
                } 
            }
            break;
        }
        case PR_REQ_HEAD_TYPE:
        {
            PRINTF("handlePrinterMsg(): Processing message: PR_REQ_HEAD_TYPE \r\n");
            
            PrHead head;
            head.headType = getHeadStyleType();
            head.headSize = getHeadStyleSize();
            sendPrHeadType( &head );
            break;
        }
        case PR_SET_BOARDTEST_MODE:
        {
            PRINTF("handlePrinterMsg(): Processing message: PR_SET_BOARDTEST_MODE \r\n");
            
            /* start the unit test thread */
            startUnitTest();
            break;
        }
        case PR_CUTTER_CUT: 
        {
            PRINTF("handlePrinterMsg(): Processing message: PR_CUTTER_CUT \r\n");
            cutterCut(false);
            break;
        }
        case PR_CUTTER_HOME:
        {
            PRINTF("handlePrinterMsg(): Processing message: PR_CUTTER_HOME \r\n"); 
            cutterHome();
            break;
        }
        case PR_REQ_CUTTER_STATUS:
        {
            PRINTF("handlePrinterMsg(): Processing message: PR_REQ_CUTTER_STATUS \r\n");
            PrCutterStatus msg;
            
            msg.function_code = PR_CUTTER_STATUS;
            getCutterStatus(&msg);
            sendPrCutterStatus( &msg );
            break;
        } 
        case PR_REQ_DOT_WEAR:        
        {
            PRINTF("handlePrinterMsg(): Processing message: PR_REQ_DOT_WEAR \r\n");
            /*The test is run when we request dot wear status. This should only
              get sent if dot wear status says there are bad dots*/
            sendDotWear( getHeadStyleSize() );
            break;
        }
        case PR_ORDER_ID:
        {
            PRINTF("handlePrinterMsg(): Processing message: PR_ORDER_ID \r\n");
            
            FPMBLC3Checksums sums;
            config_.instance = pMsg->orderId.order;
            
            if( getPageChecksums( &sums ) ) {
                /* set the new configuration to the serial flash section for configuration */
                if ( ! setSerialPrConfiguration( &config_ ) ) {
                    /* serial flash write failed.... load defaults values */
                    setSerialPrDfltConfiguration( &config_ );
                    sums.prConfigSum  = 0;    /* section corruption indication. */
                    setPageChecksums( &sums );
                } else {                     
                    sums.prConfigSum  = calculateChecksum( (void *)&config_, sizeof (Pr_Config) );
                    /* save the new checksum */
                    if( setPageChecksums( &sums ) ) {
                        configValid = true;
                        PRINTF("handlePrinterMsg(): configuration saved.\r\n" );
                    } else {
                        sendPrError( _PR_CFG_SAVE_FAILURE, false );
                        PRINTF("handlePrinterMsg(): configuration failed to saved.\r\n" );
                    }
                }
            } else {
                sendPrError( _PR_CHKSUM_READ_FAILURE, false );
                PRINTF("handlePrinterMsg(): failed to get checksums!\r\n" );
            }            
            break;
        }
        case PR_REQ_DOT_STATUS: 
        {
            PRINTF("handlePrinterMsg(): Processing message: PR_REQ_DOT_STATUS \r\n");
            /* Start the Dot Wear Checking task. When it is done we will get a 
               semaphore and send the response */
            ManagerMsg msg;
            msg.cmd = CREATE;
            msg.task = DOT_WEAR;
            addThreadManagerMsg(&msg);    
            break;
        }
        case PR_PS_FAN_CNTRL:
        {
            PRINTF("handlePrinterMsg(): Processing message: PR_PS_FAN_CNTRL \r\n");
            if( pMsg->psFanCntl.state ==  0 ) {
                /* turn power supply fan off */
                GPIO_WritePinOutput(FAN_OFF_GPIOx, FAN_OFF_PINx, false);   
                /* turn off head power */
                setHeadPower( 0 );
            } else {
                /* turn power supply fan on */
                GPIO_WritePinOutput(FAN_OFF_GPIOx, FAN_OFF_PINx, true);   
                /* turn on head power */
                setHeadPower( 1 );                
            }
            break;
        }
        case PR_ERASE_CUTTER_BIT: 
        {
            PRINTF("handlePrinterMsg(): Processing message: PR_ERASE_CUTTER_BIT \r\n");
            /* erase our cutter installed bit */
            if( !eraseSerialCutterBit() ) {
                PRINTF("handlePrinterMsg(): failed to erase cutter bit!\r\n" );
            }
            break;
        }
        case PR_START_GAP_CALIBRATION: 
        { 
            PRINTF("handlePrinterMsg(): Processing message: PR_START_GAP_CALIBRATION \r\n");
            unsigned char calPoint = 0;
            /* check we are successful in initializing */
            if( gapSensorCal( step_, &calPoint ) ) {
                /* switch to next step */
                step_ = _CALBACKING;
            } else {
                PRINTF("handlePrinterMsg(): failed to initialize for gap calibration!\r\n" );
                step_ = _CALDONE;
            }                
            break;
        }
        case PR_CAL_GAP_SENSOR_NEXT: 
        {              
            PRINTF("handlePrinterMsg(): Processing message: PR_CAL_GAP_SENSOR_NEXT \r\n");
            unsigned char calPoint = 0;
            FPMBLC3Checksums sums;
            
            PRINTF("printerTask(): Current gap cal state: %d!\r\n", step_ );
            /* if the cal step completed then setup for the next step */
            if( gapSensorCal( step_, &calPoint ) ) {
                /* switch to the next state based on current state */
                if( step_ == _CALBACKING )  
                    step_ = _CALLABEL;
                else if( step_ == _CALLABEL )
                    step_ = _CALRESULTS;
                else if( step_ == _CALRESULTS ) {
                    config_.media_sensor_adjustment = calPoint;
                    if( setGapCurrent( calPoint ) ) {                        
                        /* save set point */
                        if( setSerialPrConfiguration( &config_ ) ) {   
                            /* SOF-5109 */
                            getPageChecksums( &sums );
                            sums.prConfigSum  = calculateChecksum( (void *)&config_, sizeof (Pr_Config) );
                            
                            /* save the new checksum */
                            if( setPageChecksums( &sums ) ) {
                                configValid = true;
                            } else {
                                /* failed to save new checksum! */
                                PRINTF("printerTask(): Failed to save gap cal new checksum!\r\n" );
                            }
                        } else {
                            /* failed to save cal point into configuration! */
                            PRINTF("printerTask(): Failed to save gap cal!\r\n" );                        
                        }                                                       
                        gapSensorCal( _CALDONE, &calPoint );
                        step_ = _INIT;
                    } else {
                        PRINTF("printerTask(): failured to set gap drive current!\r\n" );
                        // TO DO: inform host of failure.
                    }
                }
                
                if( step_ == _CALDONE )
                    step_ = _INIT;
            } else {
                /* problem, next step cleanup */
                step_ = _CALDONE;
            }
            //PRINTF("printerTask(): Switching to cal state: %d!\r\n", step_ );
            break;
        }
        case PR_USE_CONTINUOUS_STOCK: {            
            setContinuousStock();
            PRINTF("printerTask(): continuous stock in use!\r\n" );
        }
        default:
            break;
    }
    hostConnected_ = true;
}

/******************************************************************************/
/*!   \fn void prWakeupCallBack( TimerHandle_t timer_  )                                                      
 
      \brief
         This function is the callback for the wakeup timer.
       
      \author
          Aaron Swift
*******************************************************************************/ 
void prWakeupCallBack( TimerHandle_t timer_  )
{
  
    /* send wakeup to host */
    PrWakeup wake;
    
    wake.function_code = PR_WAKEUP;
    wake.pid = getProductId();  
    wake.order = instance_;
    wake.printhead_size = PRINTER_HEAD_SIZE;
    wake.buffer_size = PRINTER_BUFFER_SIZE;
    wake.transfer_size = MAX_RS485_TRANAFER_SIZE;
      
    if( style_ == K_PRINTER_SERVICE_SCALE ) {
        /* freestanding scale printer does not use narrow stock */
        if( getMyModel() !=  K_FSSS ) {
            /* sensors have been processed before wakeup is sent! */
            if( ( currentStatus.sensor & WIDE_LABEL ) == WIDE_LABEL ) {
                wake.label_width = WIDE_LABEL_STOCK;
            } else {
                wake.label_width = NARROW_LABEL_STOCK;
            }
        } else {
            wake.label_width = WIDE_LABEL_STOCK;
        }
    } else {
        /* prepack printer stores label width in configuration */
        wake.label_width = config_.label_width;
    }
    
    sendPrWakeup( &wake ); 
    
    if( hostConnected_ ) {
        /* delete our timer */
        if( xTimerDelete( timer_, 0 ) != pdPASS ) {
            sendPrError( _PR_ONESHOT_TIMER_DELETE_FAILURE, true );
            PRINTF("oneShotPrWakeupCallBack(): Failed to delete timer!\r\n" );  
        }
    }
}

/******************************************************************************/
/*!   \fn static void resetPrinter( void )                                                       
 
      \brief
        This function resets the print engine and state machine.
       
      \author
          Aaron Swift
*******************************************************************************/ 
static void resetPrinter( void )
{
  
    shutdownPrintEngine();
    
    /* reset the error history and user area. */
    currentStatus.history = 0;
    currentStatus.user    = 0;
    
    /* set the reset command. */
    currentStatus.command = RESET_COMMAND;
    /* notify the host of the change */
    sendPrStatus( &currentStatus, false );
    
    /* clear the command queue */
    clearCmdQueue();
    
    /* clear label image buffer */
    clearLabelImageBuffer();
    
    /* set the idle operation. */
    setOperation( IDLE_DIRECTIVE, &currentStatus ); 
}

/******************************************************************************/
/*!   \fn static void getPrVersion( PrVersion *pVersion )                                                       
 
      \brief
        This function retrieves the printer version information from flash.
       
      \author
          Aaron Swift
*******************************************************************************/ 
static void getPrVersion( PrVersion *pVersion )
{
    /* TO DO: add product id and version to program flash*/ 
    pVersion->pid = getProductId();
    pVersion->major = getPrinterSoftwareIDMajor();
    pVersion->minor = getPrinterSoftwareIDMinor();
    pVersion->build = getPrinterSoftwareIDEng();  
    pVersion->hwMajor = getHardwareIDMajor();
    pVersion->hwMinor = getHardwareIDMinor();
}

/******************************************************************************/
/*!   \fn PrStatusInfo *getCurrentStatus( void )                                                        
 
      \brief
        This function returns the current printer status.
       
      \author
          Aaron Swift
*******************************************************************************/ 
PrStatusInfo *getCurrentStatus( void ) 
{
    return( &currentStatus );  
}

int getLabelSize(){
    return labelSize;
}

void setConfigBackingValue(unsigned char val )
{
    config_.backingPaper = val;
}

void setConfigLabelValue(unsigned char val )
{
    config_.backingAndlabel = val;
}

/******************************************************************************/
/*!   \fn static void monitorMaintenance( void )                                                       
 
      \brief
        This function will monitor specific maintenance activities.
       
      \author
          Aaron Swift
*******************************************************************************/ 
static void monitorMaintenance( void )
{
    static long long cntr_ = 0;    
    switch( maintenanceTest_ )
    {
        case _CUTTER_LIFE_TEST: {
            while( !stopTest_ ) {
                if( currentStatus.error == NO_ERROR && (currentStatus.sensor & OUT_OF_MEDIA) != OUT_OF_MEDIA 
                    && (currentStatus.sensor & LABEL_TAKEN) == LABEL_TAKEN && 
                        isCutterInterlockClosed() ) {
                    initializeCmdSequence( 5, &currentStatus );
                    while(currentStatus.state != ENGINE_IDLE) {
                        taskYIELD();
                    }

                    cutterCut(true);
                    xSemaphoreTake( pCutDoneSemaphore, portMAX_DELAY );
                    initializeCmdSequence( 2, &currentStatus );
                    while(currentStatus.state != ENGINE_IDLE){
                        taskYIELD();
                    }
                
                    cntr_++;
                    PRINTF("-----------------------Number of Cuts %lld-----------------------\r\n",cntr_);
                    vTaskDelay(pdMS_TO_TICKS(1000));                
                }
                readHeadUpSensor(&currentStatus);
                readLabelTakenSensor();
                processLabelTakenSensor(&currentStatus);
                taskYIELD();
            } 
            cntr_= 0;
            break;
        }
        case _DRIVETRAIN_LIFE_TEST: {
            while( !stopTest_ ) {
                if( ( currentStatus.error == NO_ERROR ) && 
                    ( ( currentStatus.sensor & OUT_OF_MEDIA ) != OUT_OF_MEDIA ) ) {
                    if( isCutterInterlockClosed() ) {  
                        initializeCmdSequence( 5, &currentStatus );
                        while( currentStatus.state != ENGINE_IDLE ){
                            taskYIELD();
                        }
                        
                        vTaskDelay( pdMS_TO_TICKS( 100 ) );

                        initializeCmdSequence( 2, &currentStatus );
                        while( currentStatus.state != ENGINE_IDLE ) {
                            taskYIELD();
                        }
                        cntr_++;
                        PRINTF( "-----------------------Number of Cycles %lld-----------------------\r\n", cntr_ );
                        vTaskDelay( pdMS_TO_TICKS( 1000 ) );
                    } else {
                        taskYIELD();    
                    }
                }
                readHeadUpSensor(&currentStatus);
                readLabelTakenSensor();
                processLabelTakenSensor(&currentStatus);
                taskYIELD();    
            } 
            cntr_= 0;
            break;
        }
        case _CLEANING_HEAD_TEST: {
            while( !stopTest_ ) {            
                if( ( currentStatus.error == NO_ERROR ) && 
                    ( ( currentStatus.sensor & OUT_OF_MEDIA ) != OUT_OF_MEDIA ) ) {
                    if( isCutterInterlockClosed() ) {  
                        initializeCmdSequence( 0, &currentStatus );
                        while( currentStatus.state != ENGINE_IDLE ){
                            taskYIELD();
                        }
                        cntr_++;
                        /* finish cleaning cycle */
                        if( cntr_ >= 5 ) {
                            /* switch out current table with expel table */
                            setTableTestCmds( _TESTCLEANINGEXPEL );
                            initializeCmdSequence( 0, &currentStatus );
                            while( currentStatus.state != ENGINE_IDLE ){
                                taskYIELD();
                            }                                                        
                            maintenanceTest_ = _UNKNOWN_ACTIVE_TEST;
                            stopTest_ = true;
                            activeMonitor_ = false;
                        }
                        vTaskDelay( pdMS_TO_TICKS( 1000 ) );
                    } else {
                        taskYIELD();    
                    }
                }
                readHeadUpSensor(&currentStatus);
                readLabelTakenSensor();
                processLabelTakenSensor(&currentStatus);
                taskYIELD();                                             
            }
            cntr_= 0;
            break;
        }
        default: {
            break;
        }
    }        
}