#include "globalPrinterTask.h"
#include "internalMessages.h"
#include "threadManager.h"
#include "queueManager.h"
#include "averyCutter.h"
#include "usbTask.h"
#include "systemTimer.h"
#include "translator.h"
#include "serialFlash.h"
#include "w25x10cl.h"
#include "printHead.h"
#include "commandTable.h"
#include "printEngine.h"
#include "lp5521.h"
#include "sensors.h"
#include "averyCutter.h"
#include "dotWearTask.h"
#include "vendor.h"
#include "pin_mux.h"
#include "fsl_gpt.h"
#include "fsl_debug_console.h"
#include <stdbool.h>
#include <stdlib.h>


/* task management */
static bool                     suspend_        = false;
static TaskHandle_t             pHandle_        = NULL;
static QueueHandle_t            pMsgQHandle_    = NULL;
static SemaphoreHandle_t        pCutSemaphore   = NULL;
static QueueSetHandle_t         pQueueSet_      = NULL;
static QueueHandle_t            pCRMsgQHandle_  = NULL;
static QueueHandle_t            pIMsgQHandle_   = NULL;
static TimerHandle_t            pWTimer_        = NULL;

/* label and sensor timers */
static TimerHandle_t            pTTakeLabel_     = NULL;
static TimerHandle_t            pTContinuous_    = NULL;
static TimerHandle_t            pTSensors_       = NULL;

/* application */
static PrVersion        prVersion_;
static PrinterStyle     style_;
static StationID        instance_;
static PrintMode        printMode_;

static bool             configValid             = false;
static bool             hostConnected_          = false;
static bool             cutterInstalled_        = false;

static bool             activeMonitor_          = false;
static bool             stopTest_               = false;
static bool             ltTimerStarted_         = false;

static ActiveTest       maintenanceTest_        = _UNKNOWN_ACTIVE_TEST;

Pr_Config               config_;
int                     labelSize;

static bool waitForLabelTaken = true;

extern PrStatusInfo     currentStatus, prevStatus;
extern const unsigned char testLabel[];
extern bool             paused_;

extern void initializeCmdSequence( CMDId id, PrStatusInfo *pStatus );
extern void setOperation( unsigned char operation, PrStatusInfo *pStatus );
extern void clearPrevVertOffset( void );
extern void testPrintHeadTransfer( void );
extern void weigherCountsStartStop( bool start );

/******************************************************************************/
/*!   \fn BaseType_t createGlobalPrinterTask( PrinterStyle style, 
                                              QueueHandle_t msgQueue )                                                           
      \brief
        This function initializes printer resources and creates 
        a global printer task.
       
      \author
          Aaron Swift
*******************************************************************************/ 
BaseType_t createGlobalPrinterTask( PrinterStyle style, QueueHandle_t msgQueue )
{
    BaseType_t                  result;
    FPMBLC3Checksums            checkSums;
    unsigned int                checksum = 0;
    
    
    labelSize = 0;
        
    /* set our style */
    style_ = style;
    /* set our message queue */
    pMsgQHandle_ = msgQueue;
    
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
        PRINTF("createGlobalPrinterTask(): printMode_  Not Portrait!" );
    }
    
    /* clear our command tables */
    initializeTable();

    /* currently we have no way to detect stock width */
    currentStatus.sensor |= WideLabelBit;
    
    currentStatus.mask.sensor2 = ( OUT_OF_DATA_BUFFERS | LOW_STOCK_REACHED | OUT_OF_STOCK );
    prevStatus.mask.sensor2 = ( OUT_OF_DATA_BUFFERS | LOW_STOCK_REACHED | OUT_OF_STOCK );
    
    /* before accessing serial flash, we must first get a lock on it */
    getLockSerialFlash();
    
    /*read the printer configuration from the serial flash */
    if( getSerialPrConfiguration( &config_ ) ) { 
        if( getPageChecksums( &checkSums ) ) {
            checksum = calculateChecksum((void *)&config_, sizeof(Pr_Config));
            configValid = true;
        } else {
            PRINTF("createGlobalPrinterTask(): failed to read page checksum from serial flash!\r\n"); 
          
            /* force reboot of scale */
            assert( 0 );
            
            /*failed to read checksums*/
            checkSums.prConfigSum = 0;
            configValid = false;
        }
    } else {
        /*failed to read configuration from serial flash*/
        PRINTF("createGlobalPrinterTask(): failed to read configuration from serial flash!\r\n"); 
        
        /* force reboot of scale */
        assert( 0 );
        
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
            
            PRINTF("createGlobalPrinterTask(): Default configuration loaded!\r\n" );
        } else {
            configValid  = false;
        }
    } else {
        PRINTF("createGlobalPrinterTask(): Configuration valid!\r\n" );
        configValid = true;
        instance_ = config_.instance; 
    }

    /* release the lock onthe serial flash */
    releaseLockSerialFlash();
    
    //config_.contrast_adjustment = 3;
    
    if(config_.verticalPosition > 99)
    {
       config_.verticalPosition = 99;
    }
    else if(config_.verticalPosition < 0)
    {
       config_.verticalPosition = 0;
    }
    
    /* initialize our printhead */
    if( ( style_ == RT_PRINTER_SERVICE_SCALE_72MM ) || 
        ( style_ == RT_PRINTER_FRESH_SERVE_SCALE_72MM ) ||
        ( style_ == RT_PRINTER_STAND_ALONE_72MM ) ) {
        intializePrintHead( config_.contrast_adjustment, RT_SERVICE_72MM );
    } else if( ( style_ == RT_PRINTER_SERVICE_SCALE_80MM ) || 
               ( style_ == RT_PRINTER_FRESH_SERVE_SCALE_80MM ) ||
               ( style_ == RT_PRINTER_STAND_ALONE_80MM ) ) {
        intializePrintHead( config_.contrast_adjustment, RT_SERVICE_80MM );    
    } else if( style_ ==  RT_PRINTER_PREPACK ) {
        PRINTF("createGlobalPrinterTask(): Unsupported printer style_: %d !\r\n", style_ );
    }
    
    /* cutter is only installed on better and best models */
    if( ( getMyModel() == RT_GLOBAL_SCALE_BETTER ) || 
        ( getMyModel() == RT_GLOBAL_SCALE_BEST ) || 
          ( getMyModel() == RT_GLOBAL_FSS ) ) {
        cutterInstalled_ = true;           //TFinkCutter (need to be sure cutterInstalled is true                
    }
    
    if( cutterInstalled_ ) {
        /* get my internal message queue */
        pIMsgQHandle_ = getInternalPrinterQueueHandle();       
        pCRMsgQHandle_ = getCutterQueueHandle();
        if( ( pCRMsgQHandle_ != NULL ) && ( pIMsgQHandle_ != NULL ) ) {
            if( createAveryCutterTask( pCRMsgQHandle_, pIMsgQHandle_ ) != pdPASS ) {
                PRINTF("createAveryPrinterTask(): failed to create cutter task!\r\n");
            }
        } else {
            PRINTF("createAveryPrinterTask(): Cutter msg queue was not created!\r\n");    
        }    
    
    }
        
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
        PRINTF("createGlobalPrinterTask(): Critical Error queue set not created!\r\n" );
    }
    
    /* initialize the print engine */
    initializePrintEngine( config_.contrast_adjustment, config_.out_of_media_count, getPrCommandQueueHandle() );
               
    GPIO_WritePinOutput( LOGO_EN1_LED_GPIO, LOGO_EN1_LED_PIN, false );
    GPIO_WritePinOutput( LOGO_EN_LED_GPIO, LOGO_EN_LED_PIN, false );
     
    if( getPrCommandQueueHandle() != NULL ) {
        /* create printer task thread */
        result = xTaskCreate( printerTask,  "PrinterTask", 2000,
                                            NULL, printer_task_PRIORITY, &pHandle_ );
    } else {
        PRINTF("createGlobalPrinterTask(): Command queue is NULL! Printer task not created!\r\n" );
    }
    
    if( cutterInstalled_ ) {
        initCutter();
    } else {
      config_.cutterEnabled = false;
    }
    
    /* load zero into the printhead to avoid current spikes after boot. */ 
    loadZeroPrintLine();

    PrSysInfo info;
    info.msgType = PR_SYS_INFO;
    info.pid = getProductId();
    info.id = instance_;        /* first, second or third */
    if( style_ ==  RT_PRINTER_SERVICE_SCALE_72MM ) {
        info.printhead_size = HEAD_DOTS_72MM;
        info.buffer_size = PRINTER_BUFFER_SIZE_72MM;
    } else if( style_ ==  RT_PRINTER_SERVICE_SCALE_80MM ) {
        info.printhead_size = HEAD_DOTS_80MM;
        info.buffer_size = PRINTER_BUFFER_SIZE_80MM;    
    } else {
        PRINTF("createGlobalPrinterTask(): unsupported style_!\r\n" );
    }
    info.transfer_size = 512;
    info.headType = getPrintHeadType();
    info.cutterInstalled = cutterInstalled_;
    info.cutterEnabled = config_.cutterEnabled;
    info.configValid = configValid;   
    
    sendPrSysInfo( &info );              
    
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
    if( ( style_ == RT_PRINTER_SERVICE_SCALE_72MM ) || 
        ( style_ == RT_PRINTER_SERVICE_SCALE_80MM ) ) {
        serviceScale = true;
    }
    return serviceScale;
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
    
    /* 
    torqueTest(); 
    print3InchLabel(); */
    
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
                    PRINTF("printerTask(): Failed to get Printer message from queue!\r\n" );              
                } 
            } else if( setHandle == pCutSemaphore ) {
                if( xSemaphoreTake( setHandle, 0 ) == pdTRUE ) {
                    #if 0   /* TO DO: finish integration to averyCutter. This should be a message sent to the cutter task */                    
                    cutterCut(true);
                    #endif
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

void postRequestStatus( void )
{
  
    PrGeneric msg;
    msg.msgType = PR_REQ_STATUS;
      
    if( pMsgQHandle_ != NULL ) {
        BaseType_t xHigherPriorityTaskWoken = false;
        BaseType_t result = xQueueSendToBackFromISR( pMsgQHandle_, (void *)&msg, &xHigherPriorityTaskWoken );
        if( result != pdTRUE ) {
            PRINTF("postRequestStatus(): Failed to post message to queue!\r\n" );      
        }    
    } else {
        PRINTF("postRequestStatus(): message queue NULL!\r\n" );        
    }  
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
    static TUSteps TUstep_ = _INIT_TU_CAL;
        
    unsigned long headSize = 0;
    unsigned long buffSize = 0;
    
    if(getPrintHeadType() == ROHM_72MM_800_OHM)
    {
        headSize = PRINTER_HEAD_SIZE_72MM;
        buffSize = PRINTER_BUFFER_SIZE_72MM;
    }
    else
    {
        headSize = PRINTER_HEAD_SIZE_80MM;
        buffSize = PRINTER_BUFFER_SIZE_80MM;
    }
  
    //PRINTF("PM: %d\r\n", pMsg->generic.msgType);
    
    switch( pMsg->generic.msgType )
    {
        
      
        case PR_WAKEUP:
        {              
            //PRINTF("handlePrinterMsg(): Processing message: PR_WAKEUP \r\n");
            PrWakeup wakeMsg;
            wakeMsg.msgType = PR_WAKEUP;
            wakeMsg.pid = getProductId();
            wakeMsg.id = instance_;         /* first, second or third */
            wakeMsg.printhead_size = headSize;
            wakeMsg.buffer_size = buffSize;
            wakeMsg.transfer_size = 512;
            wakeMsg.configValid = configValid;
            sendPrWakeup( &wakeMsg );
            break;
        }
        case PR_REQ_SYS_INFO: 
        {
            //PRINTF("handlePrinterMsg(): Processing message: PR_REQ_SYS_INFO \r\n");
            PrSysInfo info;
            info.msgType = PR_SYS_INFO;
            info.pid = getProductId();
            info.id = instance_;        /* first, second or third */
            
            if( getHeadStyleSize() == HEAD_DOTS_72MM  ) {
              info.printhead_size = HEAD_DOTS_72MM;
              info.buffer_size = PRINTER_BUFFER_SIZE_72MM;
            } else {            
              info.printhead_size = HEAD_DOTS_80MM;
              info.buffer_size = PRINTER_BUFFER_SIZE_80MM;            
            }
            
            info.transfer_size = 512;
            info.headType = getPrintHeadType();
            info.cutterInstalled = cutterInstalled_;
            info.cutterEnabled = config_.cutterEnabled;
            info.configValid = configValid;   
            
            sendPrSysInfo( &info );              
            break;
        }
        case PR_REQ_CONFIG:
        {
            //PRINTF("handlePrinterMsg(): Processing message: PR_REQ_CONFIG \r\n");
            sendPrConfig( &config_ );
            break;
        }
        case PR_CONFIG:
        {
            /* before accessing serial flash, we must first get a lock on it */
            getLockSerialFlash();

            //PRINTF("handlePrinterMsg(): Processing message: PR_CONFIG \r\n");
            //PRINTF("handlePrinterMsg(): disposition: %d\r\n", pMsg->config.disposition);
            FPMBLC3Checksums sums, tSum;           
            if( pMsg->config.disposition == DEFAULT_CFG ) {
                
                if( setSerialPrDfltConfiguration( &config_ ) ) {
                    getPageChecksums( &sums );
                    sums.prConfigSum  = calculateChecksum( (void *)&config_, sizeof (Pr_Config) );
                    setPageChecksums( &sums );
                    
                    //PRINTF("handlePrinterMsg(): default configuration saved.\r\n" );
                    
                    configValid = true;
                } else {                  
                    configValid = false;
                    //PRINTF("handlePrinterMsg(): default configuration failed to saved.\r\n" );
                }
                
            } else if( pMsg->config.disposition == PERMANENT_CFG ) {
                /* copy new config to our working config */
              
                /* breaks print engine if anything other than 3 contrast */
                //pMsg->config.config.contrast_adjustment = 3;  
                config_.contrast_adjustment     = pMsg->config.config.contrast_adjustment; 
              
                config_.verticalPosition        = pMsg->config.config.verticalPosition;
                config_.expel_position          = pMsg->config.config.expel_position;
                config_.peel_position           = pMsg->config.config.peel_position;
                config_.retract_position        = pMsg->config.config.retract_position;
				config_.label_width				= pMsg->config.config.label_width;
                              
                /* make sure we write back our shootthrough values. */
                pMsg->config.config.backingPaper        = config_.backingPaper;
                pMsg->config.config.backingAndlabel     = config_.backingAndlabel;
                pMsg->config.config.media_sensor_adjustment = config_.media_sensor_adjustment;
                
                if( getPageChecksums( &sums ) ) {
                    sums.prConfigSum  = calculateChecksum( (void *)&config_, sizeof (Pr_Config) );
                    /* copy the new configuration to the serial flash section for configuration */
                    if ( ! setSerialPrConfiguration( &config_ ) ) {
                        //PRINTF("setSerialPrConfiguration() failed.\r\n");
                        /* serial flash write failed.... load defaults values */
                        setSerialPrDfltConfiguration( &config_ );
                        sums.prConfigSum  = 0;    /* section corruption indication. */
                        setPageChecksums( &sums );
                    } else {     
                        Pr_Config tCfg;
                        bool configValid = false;
                        /* verify our configuration was written correctly.*/
                        getSerialPrConfiguration( &tCfg );
                        if( !verifyPrConfiguration( &config_, &tCfg) ) {
                            PRINTF("printer config re-written......\r\n");
                            /* failed to write configuration */
                            setSerialPrConfiguration( &config_ ); 
                            sums.prConfigSum  = calculateChecksum((void *)&config_, sizeof (Pr_Config) );
                        } else {
                            configValid = true;
                        }
                        /* save the new checksum */
                        if( setPageChecksums( &sums ) ) {
                            //configValid = true;
                            //PRINTF("handlePrinterMsg(): configuration saved.\r\n" );
                        } else {
                            configValid = false;
                            PRINTF("handlePrinterMsg(): configuration failed to saved.\r\n" );
                        }
                        
                        if(getPageChecksums(&tSum)) {
                            if(tSum.prConfigSum != sums.prConfigSum) {
                                if(configValid) {
                                    PRINTF("handleWeigherMsg(): wg config checksum mismatch, rewritting...\r\n");
                                    setPageChecksums( &sums );
                                } else {
                                    PRINTF("handleWeigherMsg(): both config and checksums were mismatched\r\n");
                                }
                            }
                        } else {
                            PRINTF("handleWeigherMsg(): could not get checksum from flash\r\n");
                        }                        
                    }
                } else {
                    PRINTF("handlePrinterMsg(): failed to get checksums!\r\n" );
                }
            }
            /* update print contrast */
            setEngineContrast( config_.contrast_adjustment );
            /*  TO DO:  update the vertical print position 
            updateLabelAlignment();*/
            
            /* release the lock onthe serial flash */
            releaseLockSerialFlash();
            break;
        }
        case PR_FACTORY_DFLTS:
        {
            /* before accessing serial flash, we must first get a lock on it */
            getLockSerialFlash();
          
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
                if( getMyModel() == RT_GLOBAL_FSS ) { 
                    if( ( config_.media_sensor_adjustment == 0 ) && ( config_.out_of_media_count == 200  ) &&
                        ( config_.expel_position == 150 ) ) {                
                        PRINTF("handlePrinterMsg(): factory default configuration saved.\r\n" );
                        sendPrFactoryDlftsComplete(true);
                    } else {
                        PRINTF("handlePrinterMsg(): factory default configuration failed.\r\n" );               
                        sendPrFactoryDlftsComplete(false);                
                    }

                } else {    
                    if( ( config_.media_sensor_adjustment == 31 ) && ( config_.out_of_media_count == 200  ) &&
                        ( config_.expel_position == 100 ) ) {                
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
            /* TO DO: update the vertical print position
            updateLabelAlignment(); */
            /* release the lock onthe serial flash */
            releaseLockSerialFlash();
            break;
        }
        case PR_REQ_STATUS:
        {
            //PRINTF("handlePrinterMsg(): Processing message: PR_REQ_STATUS \r\n");
            /* added for rollover error with interrupt pipes during bulk transfer */
            if( !paused_ ) {
                sendPrStatus( &currentStatus, false );  
            }
            break;
        }
        case PR_REQ_SENSORS:
        {
             //PRINTF("handlePrinterMsg(): Processing message: PR_REQ_SENSORS \r\n"); 
            
            PrSensors sensors;
            getCurrentSensors( &sensors );
            
            /* convert to new global sensors message */
            if( ( currentStatus.error & HEAD_UP ) == HEAD_UP ) {
                sensors.headup_reading = 1;
            } else {
                sensors.headup_reading = 0;
            } 
            
            if( ( currentStatus.sensor2 & OUT_OF_STOCK ) == OUT_OF_STOCK ) {
                //sensors.lowStock = 1;
                sensors.lowStock = 0;
            } else {
                sensors.lowStock = 0;
            }  
                                 
            sendPrSensors( &sensors );
            break;
        }
        case PR_REQ_CALIBRATE:
        {
            //PRINTF("handlePrinterMsg(): Processing message: PR_REQ_CALIBRATE \r\n");
            
            calibratePrinter( pMsg->calibrate.calibration );
            break;
        }
        case PR_REQ_VERSION:
        {		  
            //PRINTF("handlePrinterMsg(): Processing message: PR_REQ_VERSION \r\n");
            prVersion_.msgType = PR_VERSION;
            sendPrVersion( &prVersion_ );				
            break;
        }
        case PR_REQ_HEAD_POWER:
        {
            //PRINTF("handlePrinterMsg(): Processing message: PR_REQ_HEAD_POWER \r\n"); 
                        
            break;
        }
        case PR_MODE:
        {
            //PRINTF("handlePrinterMsg(): Processing message: PR_MODE \r\n");            
            printMode_ = pMsg->mode.mode;
            break;
        }
        case PR_RESET:
        {
            //PRINTF("handlePrinterMsg(): Processing message: PR_RESET \r\n");          
            resetPrinter(); 
            break;
        }
        case PR_ENABLE:
        {
            //PRINTF("handlePrinterMsg(): Processing message: PR_ENABLE \r\n");            
            currentStatus.command = ENABLE_COMMAND;
            /* set the idle operation. */
            setOperation( IDLE_DIRECTIVE, &currentStatus );
            break;
        }
        case PR_DISABLE:
        {
            //PRINTF("handlePrinterMsg(): Processing message: PR_DISABLE \r\n");            
            currentStatus.command = DISABLE_COMMAND;            
            /* set the disable operation. */
            setOperation( DISABLE_DIRECTIVE, &currentStatus );
            break;
        }
        case PR_POWER: 
        {            
            //PRINTF("handlePrinterMsg(): Processing message: PR_POWER \r\n");            
            setHeadPower( pMsg->power.state );          
            break;
        }
        case PR_SIZE:
        { 
            //PRINTF("handlePrinterMsg(): Processing message: PR_SIZE measured = %d actual %d\r\n", pMsg->size.measured_size, pMsg->size.actual_size );
            if( pMsg->size.actual_size == 4999 ) {
                setContinuousStock();
                resetPrinter();
            } else {
                clrContinuousStock();
            }
            //setLabelAlignment( pMsg->size.measured_size, pMsg->size.actual_size );
            labelSize = pMsg->size.actual_size;
            break;
        }
        case PR_LABEL_SIZE:
        {
            //PRINTF("handlePrinterMsg(): Processing message: PR_MULT_BUFFERS %d \r\n", pMsg->labelSize.size );                         
            setTotalLabelSize( pMsg->labelSize.size );     
            break;
        }
        case PR_TEACH:
        {
            //PRINTF("handlePrinterMsg(): Processing message: PR_TEACH\r\n");
            
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
            //PRINTF("handlePrinterMsg(): Processing message: PR_MASK \r\n");
            currentStatus.mask.sensor = pMsg->mask.mask.sensor;
            currentStatus.mask.user   = pMsg->mask.mask.user; 
            
            //PRINTF("MASK SENSOR: %d\r\n", pMsg->mask.mask.sensor);
            //PRINTF("MASK USER %d\r\n", pMsg->mask.mask.user);
            break;
        }
        case PR_TEST:
        {
            //PRINTF("handlePrinterMsg(): Processing message: PR_TEST \r\n");
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
            //PRINTF("handlePrinterMsg(): Processing message: PR_RAM \r\n");            
            /* updateRam( &(pMsg->ram) ); TO DO: is this needed??*/
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
                /* */
                //PRINTF("handlePrinterMsg(): Processing message: PR_COMMAND command.identifier %d\r\n", pMsg->command.identifier );
                //PRINTF("handlePrinterMsg(): Processing message: PR_COMMAND command.msgType %d\r\n", pMsg->command.msgType );
                //PRINTF("handlePrinterMsg(): Processing message: PR_COMMAND command.data_item %d\r\n", pMsg->command.data_item );
                //PRINTF("handlePrinterMsg(): Processing message: PR_COMMAND command.options %d\r\n", pMsg->command.options );
                //PRINTF("handlePrinterMsg(): Processing message: PR_COMMAND command.value %d\r\n", pMsg->command.value ); 
                
                //use the command option to determine whether we should waiyt for label taken when streaming 
                if(pMsg->command.options == PrWaitForLabelTaken)
                {
                    //wait for label taken when streaming
                    waitForLabelTaken = true;
                }
                else if(pMsg->command.options == PrStreamLabel)
                {
                    //do NOT wait for label taken when streaming
                    waitForLabelTaken = false; 
                }
                else
                {
                    waitForLabelTaken = false;
                }
                
                if( pMsg->command.identifier == 1 ) {
                    PRINTF("handlePrinterMsg(): Processing message: PR_COMMAND findNoLabel\r\n");  
                } else if( pMsg->command.identifier == 2 ) {
                    PRINTF("handlePrinterMsg(): Processing message: PR_COMMAND fsbackup\r\n");    
                } else if( pMsg->command.identifier == 3 ) {
                   PRINTF("handlePrinterMsg(): Processing message: PR_COMMAND gAdvance\r\n"); 
                } else if( pMsg->command.identifier == 4 ) {
                    PRINTF("handlePrinterMsg(): Processing message: PR_COMMAND gPrint\r\n"); 
                } else if( pMsg->command.identifier == 5 ) {
                    PRINTF("handlePrinterMsg(): Processing message: PR_COMMAND gEject\r\n"); 
                } else if( pMsg->command.identifier == 6 ) {
                    PRINTF("handlePrinterMsg(): Processing message: PR_COMMAND gSync\r\n"); 
                } else if( pMsg->command.identifier == 7 ) {
                    PRINTF("handlePrinterMsg(): Processing message: PR_COMMAND gFind\r\n"); 
                } else if( pMsg->command.identifier == 8 ) {
                    PRINTF("handlePrinterMsg(): Processing message: PR_COMMAND gSizing\r\n"); 
                   //PRINTF("TU Torqe Cal Peak: %d\r\n", (unsigned short)((float) config_.takeup_sensor_max_tension_counts/MAX_TENSION_MULTIPLIER));
                } else if( pMsg->command.identifier == 9 ) {
                    PRINTF("handlePrinterMsg(): Processing message: PR_COMMAND fsCutRetract\r\n"); 
                } else if( pMsg->command.identifier == 10 ) {
                    PRINTF("handlePrinterMsg(): Processing message: PR_COMMAND gHeadTest\r\n"); 
                } else if( pMsg->command.identifier == 11 ) {
                    PRINTF("handlePrinterMsg(): Processing message: PR_COMMAND fsCutRetractVirt\r\n"); 
                } else  {
                    PRINTF("handlePrinterMsg(): Processing message: PR_COMMAND %d\r\n", pMsg->command.identifier); 
                }
               
                if( getMyModel() != RT_GLOBAL_FSS ) {
                    addCmdToQueue( &pMsg->command );
                } else { 
                    if( pMsg->command.identifier == 8 ) {
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
        case PR_REQ_TRANSFER: 
        {
            //PRINTF("handlePrinterMsg(): transfer size: %d \r\n", pMsg->transfer.transferSize );
            //PRINTF("handlePrinterMsg(): label image size to print: %d \r\n", pMsg->transfer.imageSize );
            /* inform the image manager of the incomming label image size */
            setLabelImageSizeMgr( pMsg->transfer.imageSize );
            sendPrTransferReady( true );
            break;
        }
        case PR_REQ_HEAD_TYPE:
        {
            //PRINTF("handlePrinterMsg(): Processing message: PR_REQ_HEAD_TYPE \r\n");
            
            PrHead head;
            head.msgType  = PR_HEAD_TYPE;
            head.headType = getHeadStyleType();
            head.headSize = getHeadStyleSize();
            sendPrHeadType( &head );
            break;
        }
        case PR_CUTTER_CUT: 
        {
            PRINTF("handlePrinterMsg(): Processing message: PR_CUTTER_CUT \r\n");
            #if 0   /* TO DO: finish integration to averyCutter. This should be a message sent to the cutter task */
            cutterCut(false);
            #endif
            break;
        }
        case PR_CUTTER_HOME:
        {
            PRINTF("handlePrinterMsg(): Processing message: PR_CUTTER_HOME \r\n");
            #if 0   /* TO DO: finish integration to averyCutter. This should be a message sent to the cutter task */
            cutterHome();
            #endif
            break;
        }
        case PR_REQ_CUTTER_STATUS:
        {
            PRINTF("handlePrinterMsg(): Processing message: PR_REQ_CUTTER_STATUS \r\n");
            PrCutterStatus msg;
            
            msg.msgType = PR_CUTTER_STATUS;
            #if 0   /* TO DO: finish integration to averyCutter. This should be a message sent to the cutter task */
            getCutterStatus(&msg);
            sendPrCutterStatus( &msg );
            #endif
            break;
        } 
        case PR_REQ_DOT_WEAR:        
        {
            //PRINTF("handlePrinterMsg(): Processing message: PR_REQ_DOT_WEAR \r\n");
            /*The test is run when we request dot wear status. This should only
              get sent if dot wear status says there are bad dots*/
            sendDotWear( getHeadStyleSize() );
            break;
        }
        case PR_STATION_ID:
        {
            //PRINTF("handlePrinterMsg(): Processing message: PR_STATION_ID \r\n");
            
            FPMBLC3Checksums sums;
            config_.instance = pMsg->id.station;
            
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
                        //PRINTF("handlePrinterMsg(): configuration saved.\r\n" );
                    } else {
                        PRINTF("handlePrinterMsg(): configuration failed to saved.\r\n" );
                    }
                }
            } else {
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
            msg.task = T_DOT_WEAR;
            addThreadManagerMsg((unsigned char *)&msg);    
            break;
        }
        case PR_START_GAP_CALIBRATION: 
        { 
            PRINTF("handlePrinterMsg(): Processing message: PR_START_GAP_CALIBRATION \r\n");
            
            /* stop processing weigher counts until cal is done. */ 
            weigherCountsStartStop( false );
            
            unsigned char calPoint = 0;
            /* check we are successful in initializing */
            if( gapSensorCal( step_, &calPoint ) ) {
                 PRINTF("handlePrinterMsg(): _CALBACKING \r\n");
                /* switch to next step */
                step_ = _CALBACKING;
            } else {
                PRINTF("handlePrinterMsg(): failed to initialize for gap calibration!\r\n" );
                step_ = _CALDONE;
                 weigherCountsStartStop( true );
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
                        /* allow weigher to continue processing weigher counts */
                        weigherCountsStartStop( true );
                        
                        step_ = _INIT;
                    } else {
                        PRINTF("printerTask(): failured to set gap drive current!\r\n" );
                        /* allow weigher to continue processing weigher counts */
                        weigherCountsStartStop( true );

                        // TO DO: inform host of failure.
                    }
                }
                
                if( step_ == _CALDONE ) {
                    /* allow weigher to continue processing weigher counts */
                    weigherCountsStartStop( true );
                    
                    step_ = _INIT;
                }
            } else {
                /* allow weigher to continue processing weigher counts */
                weigherCountsStartStop( true );

                /* problem, next step cleanup */
                step_ = _CALDONE;
            }
            //PRINTF("printerTask(): Switching to cal state: %d!\r\n", step_ );
            break;
        }
        case PR_START_TU_CALIBRATION: 
        { 
            PRINTF("handlePrinterMsg(): Processing message: PR_START_TU_CALIBRATION \r\n");
            
            step_ = _INIT;
            
            /* stop processing weigher counts until cal is done. */ 
            weigherCountsStartStop( false );
            
            unsigned char calPoint = 0;
            /* check we are successful in initializing */
            if( TUSensorCal( step_, &calPoint ) ) 
            {
                PRINTF("handlePrinterMsg(): TU cal _INIT done, transition to _CALBACKING state \r\n");
                /* switch to next step */
                step_ = _CALRESULTS;
            } 
            else 
            {
                PRINTF("handlePrinterMsg(): failed to initialize for TU calibration!\r\n" );
                step_ = _INIT;
                
                setTUCalHeadUpFlag(false);
                weigherCountsStartStop( true );
            }                
            break;
        }
        case PR_CAL_TU_SENSOR_NEXT: 
        {              
            
            PRINTF("handlePrinterMsg(): Processing message: PR_CAL_TU_SENSOR_NEXT \r\n");
            unsigned char calPoint = 0;
            FPMBLC3Checksums sums;
            
            PRINTF("printerTask(): Current TU cal state: %d!\r\n", step_ );
            
            if(step_ == _CALRESULTS)
            {
                TUSensorCal( step_, &calPoint );
                step_ = _CALDONE;
            }
            else if(step_ == _CALDONE)
            {
                TUSensorCal( step_, &calPoint );
                step_ = _INIT;
    
                /* save set point */
                if( setSerialPrConfiguration( &config_ ) ) 
                {   
                    /* SOF-5109 */
                    getPageChecksums( &sums );
                    sums.prConfigSum  = calculateChecksum( (void *)&config_, sizeof (Pr_Config) );
                    
                    /* save the new checksum */
                    if( setPageChecksums( &sums ) ) 
                    {
                        PRINTF("printerTask(): saved TU cal new checksum!\r\n" );
                        configValid = true;
                    } 
                    else 
                    {
                        /* failed to save new checksum! */
                        PRINTF("printerTask(): Failed to save TU cal new checksum!\r\n" );
                    }
                } 
                else 
                {
                    /* failed to save cal point into configuration! */
                    PRINTF("printerTask(): Failed to save TU cal!\r\n" );                        
                }  

                weigherCountsStartStop( true );
            }

            break;
        }
        case PR_USE_CONTINUOUS_STOCK: {            
            setContinuousStock();
            PRINTF("printerTask(): continuous stock in use!\r\n" );
            break;
        }
        case PR_SET_TIMER: {
          if( pMsg->setTimer.timer == _LABEL_TAKEN_TIMER ) {
              /* should we be ignoring label taken sensor? */
              if( pMsg->setTimer.milliseconds != 0 ) {
                  //PRINTF("printerTask(): set label taken timer: %dmS\r\n", pMsg->setTimer.milliseconds ); 
                  if( pTTakeLabel_ == NULL ) {
                      //pTTakeLabel_ = xTimerCreate( "labelTimer", (TickType_t)pMsg->setTimer.milliseconds, pdTRUE, ( void * ) 0, takeLabelCallBack );
                      if( pTTakeLabel_ != NULL ) {        
                          //PRINTF("printerTask(): take label timer created\r\n" );                      
                      } else {
                          //PRINTF("printerTask(): Critical Error label timer not created!\r\n" );
                      }
                  }
              } else {
                  /* tell print engine to ignore label taken sensor during print */
                  setSkipLabelTakenCheck();
                  PRINTF("printerTask(): ignore label taken sensor!\r\n" );
              }
          } else if( pMsg->setTimer.timer == _CONTINUOUS_LABEL_TIMER ) {
              PRINTF("printerTask(): set continuous label timer: %dmS\r\n", pMsg->setTimer.milliseconds );  
              if( pTContinuous_ == NULL ) {
                  pTContinuous_ = xTimerCreate( "continuousTimer", (TickType_t)pMsg->setTimer.milliseconds, pdTRUE, ( void * ) 0, continuousLabelCallBack );
                  if( pTContinuous_ != NULL ) {        
                      if( xTimerStart( pTContinuous_, 0 ) != pdPASS ) {
                          PRINTF("printerTask(): Critical Error failed continuous timer start!\r\n" );
                      }
                  } else {
                      PRINTF("printerTask(): Critical Error continuous timer not created!\r\n" );
                  }
              }
          } else if( pMsg->setTimer.timer == _SENSOR_TIMER ) {
              PRINTF("printerTask(): set sensor timer: %dmS\r\n", pMsg->setTimer.milliseconds );  
              if( pTSensors_ == NULL ) {
                  pMsg->setTimer.milliseconds = 500;    /* 500mS */
                  pTSensors_ = xTimerCreate( "sensorTimer", (TickType_t)pMsg->setTimer.milliseconds, pdTRUE, ( void * ) 0, sensorTimerCallBack );
                  if( pTSensors_ != NULL ) {        
                      if( xTimerStart( pTSensors_, 0 ) != pdPASS ) {
                          PRINTF("printerTask(): Critical Error failed sensor timer start!\r\n" );
                      }
                      /* refresh the sensors or config page */
                      PrSensors sensors;
                      getCurrentSensors( &sensors );
                      sendPrSensors( &sensors );                          
                  } else {
                      PRINTF("printerTask(): Critical Error sensor timer not created!\r\n" );
                  }
              }
          } else {
              PRINTF("printerTask(): error setting timer = _UNKNOWN_TIMER!\r\n" );  
          }          
          break;
        }
        case PR_STOP_TIME: {
            PRINTF("handlePrinterMsg(): Processing message: PR_STOP_TIME\r\n" );  
            if( pMsg->stopTimer.timer == _LABEL_TAKEN_TIMER ) {
                if( pTTakeLabel_ != NULL ) {
                    xTimerDelete( pTTakeLabel_, 0 );
                    pTTakeLabel_ = NULL;
                }
            } else if( pMsg->stopTimer.timer == _CONTINUOUS_LABEL_TIMER ) {
                 if( pTContinuous_ != NULL ) {
                    xTimerDelete( pTContinuous_, 0 );
                    pTContinuous_ = NULL;
                }              
            } else if( pMsg->stopTimer.timer == _SENSOR_TIMER ) {                 
                 if( pTSensors_ != NULL ) {
                    xTimerDelete( pTSensors_, 0 );
                    pTSensors_ = NULL;
                    PRINTF("handlePrinterMsg(): deleting _SENSOR_TIMER\r\n" ); 
                }             
            } else {
                PRINTF("printerTask(): error stopping timer = _UNKNOWN_TIMER!\r\n" );  
            }
            break;
        }
        default:
            PRINTF("UNKNOWN PRINTER MESSAGE\r\n");
            break;
    }
    hostConnected_ = true;
}

/******************************************************************************/
/*!   \fn void startLabelTakenTimer( void )                                                      
 
      \brief
         This function starts the label taken timer. Note, should only be called 
         from the print engine printOp function.

      \author
          Aaron Swift
*******************************************************************************/ 
bool startLabelTakenTimer( void )
{
    bool result = false;
    if( !ltTimerStarted_ ) {
        if( pTTakeLabel_ != NULL ) {        
            if( xTimerStartFromISR( pTTakeLabel_, 0 ) == pdPASS ) {
                ltTimerStarted_ = true;  
                result = true;
            } else {
                PRINTF("startLabelTakenTimer(): Critical Error failed label timer start!\r\n" );
            }
        } else {
            PRINTF("startLabelTakenTimer(): pTTakeLabel_ not created!\r\n" );
        }  
    }
    return result;
}

/******************************************************************************/
/*!   \fn void stopLabelTakenTimer( void )                                                      
 
      \brief
         This function stops the label taken timer. Note, should only be called 
         from the sensors processLabelTakenSensor function.

      \author
          Aaron Swift
*******************************************************************************/ 
void stopLabelTakenTimer( void )
{
    if( pTTakeLabel_ != NULL ) {        
        if( xTimerStopFromISR( pTTakeLabel_, 0 ) == pdPASS ) {
            ltTimerStarted_ = false;                
        } else {
            PRINTF("stopLabelTakenTimer(): Critical Error failed label timer stop!\r\n" );
        }
    } else {
        PRINTF("stopLabelTakenTimer(): pTTakeLabel_ not created!\r\n" );
    }  
  
}

/******************************************************************************/
/*!   \fn bool isLabelTakenTimerStarted( void )                                                      
 
      \brief
         This function retruns true if timer has been started.

      \author
          Aaron Swift
*******************************************************************************/ 
bool isLabelTakenTimerStarted( void )
{
    return ltTimerStarted_;
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
    
    wake.msgType = PR_WAKEUP;
    wake.pid = getProductId();  
    wake.id = instance_;
    if( getHeadStyleSize() == HEAD_DOTS_72MM  ) {
        wake.printhead_size = PRINTER_HEAD_SIZE_72MM;
        wake.buffer_size = PRINTER_BUFFER_SIZE_72MM;
    } else {
        wake.printhead_size = PRINTER_HEAD_SIZE_80MM;
        wake.buffer_size = PRINTER_BUFFER_SIZE_80MM;    
    }
    wake.transfer_size = 512; 
    /* freestanding scale printer does not use narrow stock */
    if( ( style_ == RT_PRINTER_SERVICE_SCALE_72MM ) || 
    ( style_ == RT_PRINTER_FRESH_SERVE_SCALE_72MM ) ||
    ( style_ == RT_PRINTER_STAND_ALONE_72MM ) ) {        
        /* sensors have been processed before wakeup is sent! */
        if( ( currentStatus.sensor & WIDE_LABEL ) == WIDE_LABEL ) {
            wake.label_width = WIDE_LABEL_STOCK;
        } else {
            wake.label_width = WIDE_LABEL_STOCK;
        }
        
        //PRINTF("\r\n\r\n\r\n\r\nstyle == %d\r\n\r\n\r\n\r\n", style_);
        
        
        //wake.label_width = UFW_LABEL_STOCK;
        
    } else {
        /* prepack printer stores label width in configuration */
        wake.label_width = WIDE_LABEL_STOCK;
    }
    
    sendPrWakeup( &wake ); 
    
    if( hostConnected_ ) {
        /* delete our timer */
        if( xTimerDelete( timer_, 0 ) != pdPASS ) {
            PRINTF("oneShotPrWakeupCallBack(): Failed to delete timer!\r\n" );  
        }
    }
}

/******************************************************************************/
/*!   \fn void takeLabelCallBack( TimerHandle_t timer_ )                                                   
 
      \brief
         This function is the callback for label taken timer.
       
      \author
          Aaron Swift
*******************************************************************************/ 
void takeLabelCallBack( TimerHandle_t timer_ )
{
    /* label has not been removed before timeout.*/
    //sendTakeLabelError( true );
}

/******************************************************************************/
/*!   \fn void continuousLabelCallBack( TimerHandle_t timer_ )                                                   
 
      \brief
         This function is the callback for continuous timer.
       
      \author
          Aaron Swift
*******************************************************************************/ 
void continuousLabelCallBack( TimerHandle_t timer_ )
{
    /* TO DO: send continuous timer expired */        
}

/******************************************************************************/
/*!   \fn void sensorTimerCallBack( TimerHandle_t timer_ )                                                  
 
      \brief
         This function is the callback for sensors timer.
       
      \author
          Aaron Swift
*******************************************************************************/ 
void sensorTimerCallBack( TimerHandle_t timer_ )
{
  
    PrSensors sensors;
    getCurrentSensors( &sensors );
    
    sendPrSensors( &sensors );    
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
        #if 0 /* TO DO: port this here */                
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
        #endif
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
#if 0 /* TO DO: port this here */        
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
#endif
        
        default: {
            break;
        }
    }        
}

/******************************************************************************/
/*!   \fn static void verifyConfiguration( Pr_Config *pTemp, Pr_Config *pSF )                                                       
 
      \brief
        verify printer config.
       
      \author
          Joseph DiCarlantonio
*******************************************************************************/ 
static bool verifyPrConfiguration( Pr_Config *pTemp, Pr_Config *pSF )
{
    bool result = false;

    if( memcmp( pTemp, pSF, sizeof(Pr_Config) ) == 0 ) {
        result = true;
    } else {
        PRINTF("verifyConfiguration(): failed!\r\n" );        
        if( pTemp->instance != pSF->instance ) {
            PRINTF("instance: %d  %d\r\n", pTemp->instance, pSF->instance );
        }
        if( pTemp->label_width != pSF->label_width ) {
            PRINTF("label_width: %d  %d\r\n", pTemp->label_width, pSF->label_width );
        }
        if( pTemp->media_sensor_adjustment != pSF->media_sensor_adjustment ) {
            PRINTF("media_sensor_adjustment: %d  %d\r\n", pTemp->media_sensor_adjustment, pSF->media_sensor_adjustment );          
        }
        if( pTemp->out_of_media_count != pSF->out_of_media_count ) {
            PRINTF("out_of_media_count: %d  %d\r\n", pTemp->out_of_media_count, pSF->out_of_media_count );          
        }
        if( pTemp->contrast_adjustment != pSF->contrast_adjustment ) {
            PRINTF("contrast_adjustment: %d  %d\r\n", pTemp->contrast_adjustment, pSF->contrast_adjustment );          
        }
        if( pTemp->expel_position != pSF->expel_position ) {
            PRINTF("expel_position: %d  %d\r\n", pTemp->expel_position, pSF->expel_position );          
        }
        if( pTemp->peel_position != pSF->peel_position ) {
            PRINTF("peel_position: %d  %d\r\n", pTemp->peel_position, pSF->peel_position );        
        }
        if( pTemp->retract_position != pSF->retract_position ) {
            PRINTF("retract_position: %d  %d\r\n", pTemp->retract_position, pSF->retract_position );        
        }
        if( pTemp->media_sensor_type != pSF->media_sensor_type ) {
            PRINTF("media_sensor_type: %d  %d\r\n", pTemp->media_sensor_type, pSF->media_sensor_type );        
        }
        if( pTemp->sla_eject_line != pSF->sla_eject_line ) {
            PRINTF("sla_eject_line: %d  %d\r\n", pTemp->sla_eject_line, pSF->sla_eject_line );        
        }
        if( pTemp->verticalPosition != pSF->verticalPosition ) {
            PRINTF("verticalPosition: %d  %d\r\n", pTemp->verticalPosition, pSF->verticalPosition );        
        }
        if( pTemp->backingPaper != pSF->backingPaper ) {
            PRINTF("backingPaper: %d  %d\r\n", pTemp->backingPaper, pSF->backingPaper );        
        }
        if( pTemp->backingAndlabel != pSF->backingAndlabel ) {
            PRINTF("backingAndlabel: %d  %d\r\n", pTemp->backingAndlabel, pSF->backingAndlabel );        
        }
        if( pTemp->labelCalCnts != pSF->labelCalCnts ) {
            PRINTF("labelCalCnts: %d  %d\r\n", pTemp->labelCalCnts, pSF->labelCalCnts );        
        }
        if( pTemp->noLabelCalCnts != pSF->noLabelCalCnts ) {
            PRINTF("noLabelCalCnts: %d  %d\r\n", pTemp->noLabelCalCnts, pSF->noLabelCalCnts );        
        }
        if( pTemp->cutterEnabled != pSF->cutterEnabled ) {
            PRINTF("cutterEnabled: %d  %d\r\n", pTemp->cutterEnabled, pSF->cutterEnabled );        
        }
    }
    return result;
}

/******************************************************************************/
/*!   \fn static void resetPrinter( void )                                                       
 
      \brief
        This function resets the print engine and state machine.
       
      \author
          Aaron Swift
*******************************************************************************/ 
void resetPrinter( void )
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

void setConfigBackingValue(unsigned short val )
{
    PRINTF("BACKING VAL %d\r\n", val);
    config_.backingPaper = val;
}

void setConfigLabelValue(unsigned short val )
{
    PRINTF("BACKING_AND_LABEL VAL %d\r\n", val);
    config_.backingAndlabel = val;
}

void setConfigTakeupSensorValue(unsigned char val, unsigned short max, unsigned short min )
{
    config_.takeup_sensor_drive_current = val;
    config_.takeup_sensor_max_tension_counts = max;
    config_.takeup_sensor_min_tension_counts = min;
}

void setConfigMediaCurrentValue(unsigned short val )
{
    PRINTF("MEDIA SENSOR ADJUSTMENT %d\r\n", val);
    config_.media_sensor_adjustment = val;
}

int getLabelSizeFromPrinterTask( void )
{
    return labelSize;
}

bool getWaitForLabelTaken( void )
{
    return waitForLabelTaken;
}