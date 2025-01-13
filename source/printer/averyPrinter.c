#include "averyPrinter.h"
#include "averyCutter.h"
#include "translator.h"
#include "lp5521.h"
#include "vendor.h"
#include "fsl_debug_console.h"


static TaskHandle_t     pHandle_                = NULL;
static QueueHandle_t    pMsgQHandle_            = NULL;

static bool             suspend_                = false;

static PrVersion        prVersion_;




BaseType_t createAveryPrinterTask( QueueHandle_t msgQueue )
{
    BaseType_t result;
#if 0   /* add when hardware attached */    
    if( createAveryCutterTask() != pdPASS ) {
        PRINTF("createAveryPrinterTask(): failed to create cutter task!\r\n");
    }

    if( initLp5521() ) {
        PRINTF("averyPrinterTask(): LP5521 initialized.\r\n" );
    } else {
        PRINTF("averyPrinterTask(): Failed to initialize LP5521.\r\n" );
    }
#endif  
    if( msgQueue != NULL  ) {
        PRINTF("createAveryPrinterTask(): Starting...\r\n" );
        pMsgQHandle_ = msgQueue;
        
        /* create printer task thread */
        result = xTaskCreate( averyPrinterTask,  "PrinterTask", configMINIMAL_STACK_SIZE,
                                            NULL, printer_task_PRIORITY, &pHandle_ );
    }
    return result;
}

static void averyPrinterTask( void *pvParameters )
{
    PRINTF("printerTask(): Thread running...\r\n" ); 
 
    while( !suspend_ ) {        
        PrMessage msg;
        /* wait for  message */
        if( xQueueReceive( pMsgQHandle_, &msg, portMAX_DELAY ) ) {           
            handlePrinterMsg( &msg );    
        } else {
            PRINTF("averyPrinterTask(): Failed to get printer message from queue!\r\n" );  
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
    switch( pMsg->generic.msgType )
    {
            case PR_WAKEUP: {                
                //PRINTF("averyPrinterTask(): received ReqWakeup from host. \r\n\");
                PRINTF("handlePrinterMsg(): received ReqWakeup from host.\r\n" );  
                PrWakeup wakeMsg;
                wakeMsg.msgType = PR_WAKEUP;
                wakeMsg.pid = getProductId();
                wakeMsg.id = 1;
                wakeMsg.label_width = UFW_LABEL_STOCK;
                wakeMsg.printhead_size = 56;     //PRINTER_HEAD_SIZE;
                wakeMsg.buffer_size = 56952;    //PRINTER_BUFFER_SIZE;
                wakeMsg.transfer_size = 512;
                wakeMsg.cutterInstalled = true; /* TO DO: */
                sendPrWakeup( &wakeMsg );               
                break;  
            }
            case PR_REQ_CONFIG: {                                                
                PRINTF("handlePrinterMsg(): received ReqConfig from host.\r\n" );       
                Pr_Config cfg;
                /* test message, close to defaults*/        
                cfg.instance = 1;
                cfg.label_width = 3;
                cfg.media_sensor_adjustment = 0x37;
                cfg.out_of_media_count   = 200;
                cfg.contrast_adjustment  = 3;
                cfg.expel_position = 150;
                cfg.peel_position  = 0;        
                cfg.retract_position = 50;
                cfg.media_sensor_type = 2;
                cfg.sla_eject_line = 0;
                cfg.verticalPosition = 0;
                cfg.backingPaper = 0x95;
                cfg.backingAndlabel = 0x15;
                cfg.labelCalCnts = 0x75;
                cfg.noLabelCalCnts = 0x10;
                
                sendPrConfig( &cfg );
                break;  
            }
            case PR_CONFIG: {
                PRINTF("handlePrinterMsg(): received WriteConfig from host.\r\n" );       


                break;
            }
            case PR_RESET: {

                break;  
            }
            case PR_REQ_STATUS: {               
                PRINTF("handlePrinterMsg(): received ReqStatus from host.\r\n" );        
                /* test message, fake status*/               
                PrStatusInfo  info;
                info.state = 5;     /* switching */
                info.command = 0;   
                info.error = 0;
                info.history = 99;
                info.sensor = 0x55;
                info.user = 0xA5;
                info.counter = 1024;
                info.mask.sensor = 0;
                info.mask.user = 0;
                info.mask.sensor2 = 0;    
                info.sla = 0;
                info.sensor2 = 0;
                         
                sendPrStatus( &info, false );
                break;  
            }
            case PR_ENABLE: {

                break;  
            }
            case PR_DISABLE: {

                break;  
            }
            case PR_MODE: {
               
                break;  
            }
            case PR_REQ_SENSORS: {
                PRINTF("handlePrinterMsg(): received ReqSensors from host.\r\n" );  
                PrSensors msg;    
                msg.msgType = PR_REQ_SENSORS;                        
                msg.headup_reading = 0x1122;
                msg.label_width_reading = 0x3344;
                msg.lowStock = 0x5566; 
                msg.head_temperature_reading = 0x7788;
                msg.head_voltage_reading  = 0x99AA;
                msg.head_current_reading  = 0xBBCC;
                msg.label_high_average  = 0xDDEE; 
                msg.label_low_average = 0xFF00;
                msg.label_threshold = 0x1122; 
                msg.label_reading = 0x3344; 
                msg.media_high_average = 0x5566;
                msg.media_low_average = 0x7788;
                msg.media_threshold = 0x99AA;
                msg.media_reading = 0xBBCC; 
                sendPrSensors( &msg );
                break;  
            }
            case PR_COMMAND: {
                PRINTF("handlePrinterMsg(): received Command from host.\r\n" );
                
                #if 0
                addCmdToQueue( &pMsg->command );
                #endif
                break;  
            }
            case PR_REQ_CALIBRATE: {

                break;  
            }
            case PR_SIZE: {
                break;  
            }
            case PR_ENABLE_TAKEUP: {

                break;  
            }
            case PR_REQ_VERSION: {
                PRINTF("handlePrinterMsg(): received ReqVersion from host.\r\n" );                 
                getPrVersion( &prVersion_ );                
                sendPrVersion( &prVersion_ );
                break;  
            }
            case PR_REQ_HEAD_TYPE: {

                break;  
            }
            case PR_CUTTER_CUT: {

                break;  
            }
            case PR_REQ_CUTTER_STATUS: {
                break;  
            }
            case PR_REQ_DOT_WEAR: {
                break;  
            }
            case PR_FACTORY_DFLTS: {
                break;  
            }
            case PR_START_GAP_CALIBRATION: {
                break;  
            }
            case PR_CAL_GAP_SENSOR_NEXT: {
                break;  
            }
            case PR_USE_CONTINUOUS_STOCK: {
                #if 0
                setContinuousStock();
                #endif                
                break;  
            }
            case PR_MASK: {
                break;
            }
            case PR_STATION_ID: {
                break;
            }
            case PR_POWER: {
                break;
            }
            default: {
                PRINTF("t_handlePrinterMessage(): Unknown message type: %d !\r\n", pMsg->generic.msgType );
            }
    }
    
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
