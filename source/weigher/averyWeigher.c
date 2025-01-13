#include "averyWeigher.h"
#include "threadManager.h"
#include "queueManager.h"
#include "translator.h"
#include "cat3Audit.h"
#include "serialFlash.h"
#include "w25x10cl.h"
#include "cs5530.h"
#include "valueMax.h"
#include "sca3300Accel.h"
#include "math.h"
#include "vendor.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_gpio.h"
#include "fsl_gpt.h"
#include "pin_mux.h"
#include "boxcarFilter.h"
#include "boxcarFilterF.h"
#include "varGainFilter.h"
#include "fsl_debug_console.h"
#include "developmentSettings.h"
#include "eep93C56.h"

// define PI here for now
#define PI                 3.14159265358979f

static TaskHandle_t     pHandle_                = NULL;
static QueueHandle_t    pMsgQHandle_            = NULL;
static QueueHandle_t    adQHandle_              = NULL;
static QueueSetHandle_t wQueueSet_              = NULL;

static TimerHandle_t    switchBounce_           = NULL;

WeightStateType currentWeigher_;
WeightStateType prevWeigher_;

static bool suspend_                    = false;
static bool powerUp_                    = false;
static bool serviceSwitch_              = false;
static bool hostConnected_              = false;
static bool debounce_                   = false;

static WgVersion wgVersion_;
static WeigherStyle style_;
static WgConfiguration config;

static unsigned char OneHalfGraduation;
static unsigned char OneQuarterGraduation;

static WeighMode weigherMode_ = SCALE_DUMB_MODE;
static WeigherStates state_ = POWERUP_;
static bool initTracking_ = false;
static bool errorState_  = false;
static bool configValid_ = false;
static AD_Counts counts_;


bool weigherDebugOnce = false;

/* current weight */
static long weight_ = 0;


/* Flag used to indicate whether the weigher is just powering up
   after reset or after the power key has been pressed. When TRUE, +/- 10%
   zero drift limit applies. When false, +/-2% zero drift limit applies. */
bool weigherPowerUp = true;

/* last zero reference*/
long measuredZeroReference;

/* current zero of the scale*/
long currentZero;
long currentZeroAvgFilterCounts;

/* offset for strain gauge irregularities */
long electronicOffset;

/* value of the center of zero maintenance zone */
long zeroCenter;

/* value max (tilt correction) enabled */
bool valueMaxEnabled = true;

fit fitParam;

/* weigher filter */
boxcarFilter filteredReadings;
bool filteredReadingsInit = false;

/* variable gain filter and history filter for raw counts */
static boxcarFilterF hist0;
static boxcarFilterF hist1;
static boxcarFilterF hist2;

static boxcarFilterF min;
static boxcarFilterF max;

static varGainFilter filt;
static double uncertainty;

static bool init = false;

/******************************************************************************/
/*!   \fn BaseType_t createAveryWeigherTask( WeigherStyle style, QueueHandle_t msgQueue )                                                        
 
      \brief
      
      \author
          Aaron Swift / Joseph 
*******************************************************************************/ 
BaseType_t createAveryWeigherTask( WeigherStyle style, QueueHandle_t msgQueue )
{
    BaseType_t result;

    /* set our style */
    style_ = style; 
    
    /* get our weigher configuration from serial flash */    
    getSerialConfiguration( &config );
    PRINTF("createAveryWeigherTask(): value_max_on_off = %d\r\n", config.value_max_on_off);
    
    #ifdef TFinkForceVMConfigEnable
    config.value_max_on_off = true;
    #endif
    
	/* Based on Config, Initialize the following parameters:
	 	- AutoZeroMaintenanceLimit, 
	 	- OneHalfGraduation
	 	- OneQuarterGraduation 
	*/
	updateParameters();
	
	/* intialize our version information */
    setWeigherVersion( &wgVersion_ );
    
    /* clear our weigher status */
    memset( &currentWeigher_, 0, sizeof(WeightStateType) );
    memset( &prevWeigher_, 0, sizeof(WeightStateType) );
    memset( &counts_, 0, sizeof(AD_Counts) );
    
    /* clear our current weight */
    weight_ = 0;
    
	/* set the powerup flag */
    weigherPowerUp = true;
	    
    /* intialize the service switch */
    initCalibrationSwitch();

    /* create a timer for dealing with service switch bounce */
    switchBounce_ = xTimerCreate( "serviceDebounce", (TickType_t)SWITCH_DEBOUNCE_TICKS, pdTRUE, ( void * ) 0, debounceCallback );
    
    /* initialize our filter */
    initializeFilter( config.filter_speed );

    /* assign local task queues */    
    pMsgQHandle_ = msgQueue;
    adQHandle_ = getWeigherCntsQueueHandle();
    
    /* initialize our external a/d */
    if( initializeCs5530( adQHandle_ ) ) {    
        PRINTF("\r\ncreateAveryWeigherTask(): initializing CS5530()\r\n" );
        /* only best model has valuemax */ 
        if(config.value_max_on_off == true)
        {
            PRINTF("createAveryWeigherTask(): createValueMaxTask()\r\n" );
            /* spinup our accelerometer task */
            if( createValueMaxTask() != pdTRUE ) {            
                PRINTF("createAveryWeigherTask(): Failed to create accelerometer task!\r\n" );
            }
        }

        #if 0        
        eraseAuditLog();
        setSerialWgDfltConfig( &config ); 
        #endif
        
        /* before accessing serial flash, we must first get a lock on it */
        getLockSerialFlash();
        
        /* intialize cat3 audit manager */
        initializeAuditMGR();  
        
        /* Init audit tracking events */
        initializeEvents( &config );

        /* release the lock onthe serial flash */
        releaseLockSerialFlash();  
        
        /* determine length of all weigher queues combined without value max */
        unsigned long wQSetLength = ( getWeigherMsgQueueLength() + getWeigherCntsQueueLength() );
        
        /* create local queue set */
        wQueueSet_ = xQueueCreateSet( wQSetLength );
        
        if( ( pMsgQHandle_ != NULL ) && ( adQHandle_ != NULL ) && 
           ( wQueueSet_ != NULL ) ) {
              /* add queues to local set */
              xQueueAddToSet( pMsgQHandle_, wQueueSet_ );
              xQueueAddToSet( adQHandle_, wQueueSet_ );                          
           } else {
              PRINTF("createWeigherTask(): Critical Error queue set not created!\r\n" );
           }        
        
        PRINTF("createAveryWeigherTask(): Starting...\r\n" );
        /* create printer task thread */
        result = xTaskCreate( averyWeigherTask,  "WeigherTask", configMINIMAL_STACK_SIZE,
                                            NULL, weigher_task_PRIORITY, &pHandle_ );
    } else {
        PRINTF("createAveryWeigherTask(): Failed to start external weigher a/d!\r\n" );
    }
    return result;
}

/******************************************************************************/
/*!   \fn static void getSerialConfiguration( WgConfiguration *pCfg  )

      \brief
        This function read weigher configuration from serial flash and determines 
        if configuration is valid.
      \author
          Aaron Swift
*******************************************************************************/
static void getSerialConfiguration( WgConfiguration *pCfg  )
{
    int checkSum = 0;
    FPMBLC3Checksums sums;
    bool blankDevice = false;
    
    /* before accessing serial flash, we must first get a lock on it */
    getLockSerialFlash();

    /*get the weigher configuration data from the serial flash */
    if( getSerialWgConfiguration( pCfg ) ) {
        if( getPageChecksums( &sums ) ) {
            if( sums.wgConfigSum == 65535 ) {
              
                PRINTF("getSerialConfiguration(): weigher section of serial flash blank!\r\n" ); 
                /* force reboot of scale 
                assert( 0 );*/

                blankDevice = true;
            }

            checkSum = calculateChecksum((void *)pCfg, (unsigned long)sizeof(WgConfiguration) );
        } else {   
            PRINTF("getSerialConfiguration():failed to read page checksums !\r\n" ); 
          
            /* force reboot of scale */
            assert( 0 );

            /*failed to read checksums*/
            sums.wgConfigSum = 0;
            errorState_ = true;
            configValid_ = false;
        }
    } else {        
        PRINTF("createAveryWeigherTask(): failed to read configuration from serial flash!\r\n" ); 

        /* force reboot of scale */
        assert( 0 );

        setSerialWgDfltConfig( pCfg );        
        sums.wgConfigSum = 0;
        errorState_ = true;
        configValid_ = false;
    }

    if( checkSum != sums.wgConfigSum ) {

        PRINTF("createAveryWeigherTask(): Setting default configuration!\r\n" );
        PRINTF("createAveryWeigherTask(): checksum = %d\r\n", checkSum);
        PRINTF("createAveryWeigherTask(): sums.wgConfigSum = %d\r\n", sums.wgConfigSum);
        
        setSerialWgDfltConfig( pCfg );
        sums.wgConfigSum  = calculateChecksum((void *)pCfg, (unsigned long)sizeof (WgConfiguration) );

        if( !blankDevice ) {
            setPageChecksums( &sums );
            PRINTF("createAveryWeigherTask(): Default configuration loaded!\r\n" );
        } else {
            setPageChecksums( &sums );
        }
        configValid_ = true;
    } else {    	
        PRINTF("createAveryWeigherTask(): Configuration valid!\r\n" );        
        errorState_ = false;
        configValid_ = true;
    } 
    /* release the lock onthe serial flash */
    releaseLockSerialFlash();
}

static bool verifyConfiguration( WgConfiguration *pTemp, WgConfiguration *pSF )
{
    bool result = false; 

    if( memcmp( pTemp, pSF, sizeof(WgConfiguration) ) == 0 ) {
        result = true;
    } else {
        PRINTF("verifyConfiguration(): failed!\r\n" );        
        if( pTemp->center_of_maintenance_zone != pSF->center_of_maintenance_zone ) {
            PRINTF("center_of_maintenance_zone: %d  %d\r\n", pTemp->center_of_maintenance_zone, pSF->center_of_maintenance_zone );
        }
        if( pTemp->scale_factor != pSF->scale_factor ) {
            PRINTF("scale_factor: %d  %d\r\n", pTemp->scale_factor, pSF->scale_factor );
        }
        if( pTemp->max_weight != pSF->max_weight ) {
            PRINTF("max_weight: %d  %d\r\n", pTemp->max_weight, pSF->max_weight );          
        }
        if( pTemp->gain_factor != pSF->gain_factor ) {
            PRINTF("gain_factor: %d  %d\r\n", pTemp->gain_factor, pSF->gain_factor );          
        }
        if( pTemp->prepack_motion_count != pSF->prepack_motion_count ) {
            PRINTF("prepack_motion_count: %d  %d\r\n", pTemp->prepack_motion_count, pSF->prepack_motion_count );          
        }
        if( pTemp->initialize_zero_time != pSF->initialize_zero_time ) {
            PRINTF("initialize_zero_time: %d  %d\r\n", pTemp->initialize_zero_time, pSF->initialize_zero_time );          
        }
        if( pTemp->small_motion_limit != pSF->small_motion_limit ) {
            PRINTF("small_motion_limit: %d  %d\r\n", pTemp->small_motion_limit, pSF->small_motion_limit );        
        }
        if( pTemp->large_motion_limit != pSF->large_motion_limit ) {
            PRINTF("large_motion_limit: %d  %d\r\n", pTemp->large_motion_limit, pSF->large_motion_limit );        
        }
        if( pTemp->large_motion_count != pSF->large_motion_count ) {
            PRINTF("large_motion_count: %d  %d\r\n", pTemp->large_motion_count, pSF->large_motion_count );        
        }
        if( pTemp->small_motion_count != pSF->small_motion_count ) {
            PRINTF("small_motion_count: %d  %d\r\n", pTemp->small_motion_count, pSF->small_motion_count );        
        }
        if( pTemp->no_motion_count != pSF->no_motion_count ) {
            PRINTF("no_motion_count: %d  %d\r\n", pTemp->no_motion_count, pSF->no_motion_count );        
        }
        if( pTemp->filter_speed != pSF->filter_speed ) {
            PRINTF("filter_speed: %d  %d\r\n", pTemp->filter_speed, pSF->filter_speed );        
        }
        if( pTemp->last_calibration_date != pSF->last_calibration_date ) {
            PRINTF("last_calibration_date: %d  %d\r\n", pTemp->last_calibration_date, pSF->last_calibration_date );        
        }
        if( pTemp->number_of_calibrations != pSF->number_of_calibrations ) {
            PRINTF("number_of_calibrations: %d  %d\r\n", pTemp->number_of_calibrations, pSF->number_of_calibrations );        
        }
        if( pTemp->last_configuration_date != pSF->last_configuration_date ) {
            PRINTF("last_configuration_date: %d  %d\r\n", pTemp->last_configuration_date, pSF->last_configuration_date );        
        }
        if( pTemp->number_of_configurations != pSF->number_of_configurations ) {
            PRINTF("number_of_configurations: %d  %d\r\n", pTemp->number_of_configurations, pSF->number_of_configurations );        
        }
        if( pTemp->weigher_type != pSF->weigher_type ) {
            PRINTF("weigher_type: %d  %d\r\n", pTemp->weigher_type, pSF->weigher_type );        
        }
        if( pTemp->flags != pSF->flags ) {
            PRINTF("flags: %d  %d\r\n", pTemp->flags, pSF->flags );        
        }
        if( pTemp->min_weight_to_print != pSF->min_weight_to_print ) {
            PRINTF("verifyConfiguration: %d  %d\r\n", pTemp->min_weight_to_print, pSF->min_weight_to_print );        
        }
        if( pTemp->weigher_model != pSF->weigher_model ) {
            PRINTF("weigher_model: %d  %d\r\n", pTemp->weigher_model, pSF->weigher_model );        
        }
        if( pTemp->value_max_on_off != pSF->value_max_on_off ) {
            PRINTF("value_max_on_off: %d  %d\r\n", pTemp->value_max_on_off, pSF->value_max_on_off );        
        }
    }
    return result;
}

/******************************************************************************/
/*!   \fn static void initCalibrationSwitch( void )

      \brief
        This function sets gpio pin for input external interrupt.
      \note
        Priority level of 0-255 for each interrupt.  A higher level corresponds 
        to a lower priority, so level 0 is the highest interrupt priority.
   
      \author
          Aaron Swift
*******************************************************************************/
static void initCalibrationSwitch( void )
{
    gpio_pin_config_t gpioConfig;
    
    gpioConfig.direction        = kGPIO_DigitalInput;
    gpioConfig.outputLogic      = 0;
    /* gpioConfig.interruptMode    = kGPIO_IntFallingEdge; */
    
    /* init input switch gpio. */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_06_GPIO2_IO06, 0U );
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_06_GPIO2_IO06, 0x70A0U );
    GPIO_PinInit( SERVICE_SWITCH_GPIO, SERVICE_SWITCH_PIN, &gpioConfig );
    
    /* enable the interrupt 
    GPIO_PortEnableInterrupts( SERVICE_SWITCH_GPIO, 1U << SERVICE_SWITCH_PIN );
    NVIC_SetPriority( GPIO2_Combined_0_15_IRQn, 10 );    
    EnableIRQ( GPIO2_Combined_0_15_IRQn );    */
}

/******************************************************************************/
/*!   \fn void debounceCallback( TimerHandle_t timer )

      \brief
        This function deletes the service switch debounce timer and  
        sets debounce flag to allow monitor weigher to check service 
        switch status.
      \author
          Aaron Swift
*******************************************************************************/
void debounceCallback( TimerHandle_t timer )
{
    debounce_ = false;
    if( xTimerStop( timer, 0 ) != pdPASS ) {
        PRINTF("debounceCallback(): cannot stop debounce timer!\r\n" );    
    }
}

/******************************************************************************/
/*!   \fn static void averyWeigherTask( void *pvParameters )

      \brief
        This function handles initialization of value max and handles the 
        processing of the message and a/d count queue.
   
      \author
          Aaron Swift
*******************************************************************************/
static void averyWeigherTask( void *pvParameters )
{
    ( void ) pvParameters;
    
    WgMessage wgMsg;
    //SCA3300 acclData;
    unsigned long adCounts; 
    QueueSetMemberHandle_t setHandle = NULL;
    
    PRINTF("weigherTask(): Thread running...\r\n" );  
    
    while( !suspend_ ) {
           
        #ifdef TFinkStartCS5530woApp  
        //We were getting out of sync. ADCounts Queue was full, but WeigherTask was suspended_. Returns NULL if all Queues in set empty
        setHandle = xQueueSelectFromSet( wQueueSet_, pdMS_TO_TICKS(10)); 
        #else
        setHandle = xQueueSelectFromSet( wQueueSet_, portMAX_DELAY ); 
        #endif
        if( setHandle == pMsgQHandle_ ) {   
            #ifdef TFinkStartCS5530woApp  
            if( xQueueReceive( setHandle, &wgMsg, pdMS_TO_TICKS(10)  ) ) {
            //We were getting out of sync. ADCounts Queue was full, but WeigherTask was suspended_. Returns NULL if all Queues in set empty
            #else
            if( xQueueReceive( setHandle, &wgMsg, portMAX_DELAY ) ) {
            #endif
                handleWeigherMsg( &wgMsg );
            } else {
                PRINTF("weigherTask(): Failed to get Weigher message from queue!\r\n" );
            }
            } else if( setHandle == adQHandle_ ) {
               int cnt_ = uxQueueMessagesWaiting( setHandle );
               while( cnt_ ) {
                  #ifdef TFinkStartCS5530woApp  
                  //We were getting out of sync. ADCounts Queue was full, but WeigherTask was suspended_. Returns NULL if all Queues in set empty
                  if( xQueueReceive( setHandle, &adCounts, pdMS_TO_TICKS(10)  ) ) { 
                  #else
                  if( xQueueReceive( setHandle, &adCounts, portMAX_DELAY  ) ) { 
                  #endif
                        counts_.rawCounts = adCounts;
                        handleAdConversion( &counts_ );
                        monitorWeigher( &counts_ );
                     } else {
                        PRINTF("weigherTask(): Failed to get Weigher counts from queue!\r\n" );
                     }
                     cnt_--;
                  } 
                  
                /* cnt_ should be 0 unless messages were queued while above code executed */
                cnt_ = uxQueueMessagesWaiting( setHandle );
                if(cnt_ != 0)
                  PRINTF("WeigherTask(): adQueue Messages: %d\r\n",cnt_);
           }
        taskYIELD();
    }
    PRINTF("\r\n\r\nWeigherTask(): Suspending! \r\r\r\n");  
    vTaskSuspend(NULL);  
    
}

/******************************************************************************/
/*!   \fn static void handleWeigherMsg( void )

      \brief
        This function handles Weigher messages.
   
      \author
          Aaron Swift
*******************************************************************************/
static void handleWeigherMsg( WgMessage *pMsg )
{
    /* two config messages for one calibration event  */ 
    static bool calFinished = false; 
    switch( pMsg->generic.msgType )
    {

        case WG_WAKEUP: {
            //PRINTF("handleWeigherMsg(): received ReqWakeup from host.\r\n" );
            WgWakeup wakeup;
            wakeup.msgType = WG_WAKEUP;
            wakeup.pid = getProductId();
            wakeup.weigherType = 1; //primary or secondary?
            sendWgWakeup( &wakeup );
            break;
        }
        case WG_REQ_CONFIG: {
            //PRINTF("handleWeigherMsg(): received ReqConfig from host.\r\n");            
            WgCfg msg;

            msg.msgType = WG_REQ_CONFIG;
            msg.disposition = 1;
            msg.config.center_of_maintenance_zone       = config.center_of_maintenance_zone;
            msg.config.scale_factor                     = config.scale_factor;
            msg.config.max_weight                       = config.max_weight;            //in weigher config page
            msg.config.gain_factor                      = config.gain_factor;
            msg.config.prepack_motion_count             = config.prepack_motion_count;
            msg.config.initialize_zero_time             = config.initialize_zero_time;
            msg.config.small_motion_limit               = config.small_motion_limit;        
            msg.config.large_motion_limit               = config.large_motion_limit;
            msg.config.large_motion_count               = config.large_motion_count;
            msg.config.small_motion_count               = config.small_motion_count;   
            msg.config.no_motion_count                  = config.no_motion_count;
            msg.config.filter_speed                     = config.filter_speed;          //in weigher config page
            msg.config.last_calibration_date            = config.last_calibration_date;
            msg.config.number_of_calibrations           = config.number_of_calibrations;
            msg.config.last_configuration_date          = config.last_configuration_date;
            msg.config.number_of_configurations         = config.number_of_configurations;
            msg.config.weigher_type                     = config.weigher_type; //primary or secondary?
            msg.config.flags                            = config.flags;
            msg.config.min_weight_to_print              = config.min_weight_to_print;
            msg.config.weigher_model                    = config.weigher_model;   /* global legacy weigher protocol */
            msg.config.value_max_on_off                 = config.value_max_on_off;
            
            #if 0
            PRINTF("\r\n\r\n CONFIG DATA ----------- FROM ------------------- PERPHERAL PROCESSOR\r\n\r\n");
            
            PRINTF("msg.config.center_of_maintenance_zon %d \r\n",msg.config.center_of_maintenance_zone);
            PRINTF("msg.config.scale_factor %d\r\n",msg.config.scale_factor);
            PRINTF("msg.config.max_weight %d\r\n",msg.config.max_weight);
            PRINTF("msg.config.gain_factor %d\r\n",msg.config.gain_factor);
            PRINTF("msg.config.prepack_motion_count %d\r\n",msg.config.prepack_motion_count);
            PRINTF("msg.config.initialize_zero_tim %d\r\n",msg.config.initialize_zero_time);
            PRINTF("msg.config.small_motion_limit %d\r\n",msg.config.small_motion_limit);
            PRINTF("msg.config.large_motion_limit %d\r\n",msg.config.large_motion_limit);
            PRINTF("msg.config.large_motion_count %d\r\n",msg.config.large_motion_count);
            PRINTF("msg.config.small_motion_count %d \r\n",msg.config.small_motion_count);
            PRINTF("msg.config.no_motion_count %d \r\n",msg.config.no_motion_count);
            PRINTF("msg.config.filter_speed %d\r\n",msg.config.filter_speed);

            PRINTF("msg.config.last_calibration_date %d\r\n",msg.config.last_calibration_date);
            PRINTF("msg.config.number_of_calibrations %d\r\n",msg.config.number_of_calibrations);
            PRINTF("msg.config.last_configuration_date %d\r\n",msg.config.last_configuration_date);
            PRINTF("msg.config.number_of_configurations %d\r\n",msg.config.number_of_configurations);

            PRINTF("msg.config.weigher_type %d \r\n",msg.config.weigher_type);
            PRINTF("msg.config.flags %d \r\n",msg.config.flags);
            PRINTF("msg.config.min_weight_to_print %d \r\n",msg.config.min_weight_to_print);
            PRINTF("msg.config.weigher_model %d \r\n",msg.config.weigher_model);
            PRINTF("msg.config.value_max_on_off %d \r\n",msg.config.value_max_on_off);
            #endif
            
            if( !serviceSwitch_  )
                sendWgConfig( &msg );
            break;
        }
        case WG_CONFIG: {
            PRINTF("handleWeigherMsg(): received write of configuration from host. \r\n");
            
            #if 0
            PRINTF("pMsg->config.disposition %d \r\n",pMsg->config.disposition);
            PRINTF("pMsg->config.config.center_of_maintenance_zon %d \r\n",pMsg->config.config.center_of_maintenance_zone);
            PRINTF("pMsg->config.config.scale_factor %d\r\n",pMsg->config.config.scale_factor);
            PRINTF("pMsg->config.config.max_weight %d\r\n",pMsg->config.config.max_weight);

            PRINTF("\r\n\r\nconfig.gain_factor %d\r\n",pMsg->config.config.gain_factor);
            PRINTF("pMsg->config.config.gain_factor %d\r\n",pMsg->config.config.gain_factor);

            PRINTF("pMsg->config.config.prepack_motion_count %d\r\n",pMsg->config.config.prepack_motion_count);
            PRINTF("pMsg->config.config.initialize_zero_tim %d\r\n",pMsg->config.config.initialize_zero_time);
            PRINTF("pMsg->config.config.small_motion_limit %d\r\n",pMsg->config.config.small_motion_limit);
            PRINTF("pMsg->config.config.large_motion_limit %d\r\n",pMsg->config.config.large_motion_limit);
            //azsm no longer used in message. PRINTF("pMsg->config.config.azsm_motion_limit %d\r\n",pMsg->config.config.azsm_motion_limit);
            PRINTF("pMsg->config.config.large_motion_count %d\r\n",pMsg->config.config.large_motion_count);
            PRINTF("pMsg->config.config.small_motion_count %d \r\n",pMsg->config.config.small_motion_count);
            PRINTF("pMsg->config.config.no_motion_count %d \r\n",pMsg->config.config.no_motion_count);
            PRINTF("pMsg->config.config.filter_speed %d\r\n",pMsg->config.config.filter_speed);
            PRINTF("pMsg->config.last_calibration_date %d\r\n",pMsg->config.config.last_calibration_date);
            PRINTF("pMsg->config.number_of_calibrations %d\r\n",pMsg->config.config.number_of_calibrations);
            PRINTF("pMsg->config.last_configuration_date %d\r\n",pMsg->config.config.last_configuration_date);
            PRINTF("pMsg->config.number_of_configurations %d\r\n",pMsg->config.config.number_of_configurations);
            PRINTF("pMsg->config.config.weigher_type %d \r\n",pMsg->config.config.weigher_type);
            PRINTF("pMsg->config.config.flags %d \r\n",pMsg->config.config.flags);
            PRINTF("pMsg->config.config.min_weight_to_print %d \r\n",pMsg->config.config.min_weight_to_print);
            PRINTF("pMsg->config.config.weigher_model %d \r\n",pMsg->config.config.weigher_model);
            #ifdef TFinkForceVMConfigEnable
            pMsg->config.config.value_max_on_off = true;
            #endif
            PRINTF("pMsg->config.config.value_max_on_off %d \r\n",pMsg->config.config.value_max_on_off);
            #endif
            
            /*  clear error status field in case any ValueMax errors where set, prior to setting new config. */        
            currentWeigher_.status = 0;
            prevWeigher_.status = 0;
                        
            if( ( pMsg->config.disposition & CALIBRATION_IN_PROGRESS ) == CALIBRATION_IN_PROGRESS ) {
                //PRINTF("handleWeigherMsg(): config.disposition CALIBRATION_IN_PROGRESS. \r\n");    
                /********************************************
                * Turn off the CALIBRATION_IN_PROGRESS bit. *
                ********************************************/
                pMsg->config.disposition &= ~CALIBRATION_IN_PROGRESS;

                /* set our cat3 flag */
                calFinished = true;
            }

            if( pMsg->config.disposition == _DEFAULT ) {
              
                /* before accessing serial flash, we must first get a lock on it */
                getLockSerialFlash();

                PRINTF("handleWeigherMsg(): config.disposition _DEFAULT. \r\n");    
                FPMBLC3Checksums sums;
                setSerialWgDfltConfig( &config );

                getPageChecksums( &sums );
                sums.wgConfigSum  = calculateChecksum((void *)&config, (unsigned long)sizeof (WgConfiguration) );
                setPageChecksums( &sums );

                PRINTF("handleWeigherMsg(): default configuration saved.\r\n" );
                errorState_ = false;
                
                /* release the lock onthe serial flash */
                releaseLockSerialFlash();

            }

            initializeFilter( config.filter_speed );
        
            if( pMsg->config.disposition == _PERMANENT ) {
                /* disable weigher counts until we finish processing config */
                disableWeigher();
                
                memcpy((void *)&config, (void *)&(pMsg->config.config), (size_t)(sizeof(WgConfiguration)));
                
                /* before accessing serial flash, we must first get a lock on it */
                getLockSerialFlash();

                PRINTF("handleWeigherMsg(): config.disposition _PERMANENT. \r\n");  
                FPMBLC3Checksums chkSums;
                /*read the checksum page from the serial flash so we can update the weigher config
                checksum.  */
                if( getPageChecksums(&chkSums) ) {
                    chkSums.wgConfigSum  = calculateChecksum((void *)&config, sizeof (WgConfiguration) );
                    PRINTF("calculated checksum: %d\r\n", chkSums.wgConfigSum);
                    
                    /* copy the new configuration to the serial flash section for configuration */
                    if ( ! setSerialWgConfiguration( &config ) ) {
                        PRINTF("wg config serial flash write failed\r\n");
                        /* serial flash write failed.... load defaults values */
                        setSerialWgDfltConfig( &config );
                        chkSums.wgConfigSum  = 0;    /* section corruption indication. */
                        setPageChecksums( &chkSums );
                    } else {
                          
                        WgConfiguration tCfg;
                        FPMBLC3Checksums tSum;
                        bool configValid = false;
                        
                        int retryconf = 3;
                        int retrysum = 3;
                        
                        do
                        {                          
                            /* verify our configuration was written correctly.*/
                            if( getSerialWgConfiguration( &tCfg ) ) {
                                if( !verifyConfiguration( &config, &tCfg) ) {
                                    PRINTF("weigher config re-written......\r\n");
                                    /* failed to write configuration */
                                    if( setSerialWgConfiguration( &config ) ) {
                                        chkSums.wgConfigSum  = calculateChecksum((void *)&config, sizeof (WgConfiguration) );
                                    } else {
                                        PRINTF("handleWeigherMsg(): write of serial flash failed.\r\n"); 
                                    }
                                } else {
                                    configValid = true;
                                    PRINTF("handleWeigherMsg(): Verified WConfig write valid\r\n");
                                    break;
                                }
                            } else {
                                PRINTF("handleWeigherMsg(): read of serial flash failed.\r\n");  
                            }
                        } while ( --retryconf );

                        /* update our weighing parameters based on config changes */ 
                        updateParameters();
                        
                        /* save the new checksum */
                        if( setPageChecksums( &chkSums ) ) {
                            errorState_ = false;
                            PRINTF("handleWeigherMsg(): configuration saved.\r\n" );
                        } else {
                            errorState_ = true;
                            PRINTF("handleWeigherMsg(): configuration failed to saved.\r\n" );
                        }
                        if(getPageChecksums(&tSum)) {
                            if(tSum.wgConfigSum != chkSums.wgConfigSum) {
                                if(configValid) {
                                    PRINTF("handleWeigherMsg(): wg config checksum mismatch, rewritting...\r\n");
                                    setPageChecksums( &chkSums );
                                } else {
                                    break;
                                }
                            } else {
                               PRINTF("handleWeigherMsg(): Wg Chksum Err: from Flash %d, calculated %d: \r\n", tSum.wgConfigSum,chkSums.wgConfigSum );
                            }
                        } while ( --retrysum );
                        
                        if( calFinished ) {
                            /* save the calibration event into the audit trail */
                            saveCalibEvents( &config );
                            calFinished = false;
                        } else {
                            saveConfigEvents( &config );
                        }                         
                    }
                } else {
                    PRINTF("handleWeigherMsg(): failed to get checksums!\r\n" );
                }   
                /* release the lock onthe serial flash */
                releaseLockSerialFlash();

                enableWeigher();
            }
            break;
        } 
        case WG_FACTORY_DEFAULTS: {
            PRINTF("\r\nhandleWeigherMsg(): Processing message: WG_FACTORY_DEFAULTS \r\n");              
            FPMBLC3Checksums sums;
            
            /* before accessing serial flash, we must first get a lock on it */
            getLockSerialFlash();
            
            if( setSerialWgDfltConfig( &config ) ) {
                /* update our checksums */
                getPageChecksums( &sums );
                sums.wgConfigSum  = calculateChecksum((void *)&config, (unsigned long)sizeof (WgConfiguration) );
                /* set new checksum for defaults */
                setPageChecksums( &sums );
                PRINTF("handleWeigherMsg(): factory default configuration saved.\r\n" );
                
                memset( &config, 0, sizeof( WgConfiguration ) );
                getSerialWgConfiguration( &config );                          
                sendWgFactoryDlftsComplete( true );                                        
                
                /* save to cat3 log */
                saveConfigEvents( &config );
            } else {               
                PRINTF("handleWeigherMsg(): factory default configuration failed.\r\n" );
                sendWgFactoryDlftsComplete(false);
            }
            errorState_ = false;
            
            /* release the lock onthe serial flash */
            releaseLockSerialFlash();
            break;
        }
        case WG_RESET: {
            //PRINTF("handleWeigherMsg(): Processing message: WG_RESET \r\n");
            adReset(currentWeigher_.avgFilterCounts);
            break;
        }
        case WG_STATUS: {
            //PRINTF("handleWeigherMsg(): Processing message: WG_STATUS \r\n"); 
            
            WgStatus msg;
            short x_deg, y_deg;

            msg.status                  = currentWeigher_.status;
            msg.avgFilterCounts         = (long)currentWeigher_.avgFilterCounts;
            msg.nonZeroCalibratedCounts = (long)currentWeigher_.calibratedCounts;	
            msg.zeroedCalibratedCounts  = (long)labs( weight_ );
            msg.rawCounts               = (unsigned long)currentWeigher_.rawCounts;
            
            if( getMyModel() == RT_GLOBAL_SCALE_BEST ) {
               ValueMax *pVM = getPtrToValueMax();
               
               msg.acclAvgXCounts          = pVM->accel_x_avg_raw_reading;
               msg.acclAvgYCounts          = pVM->accel_y_avg_raw_reading;
               msg.acclAvgZCounts          = pVM->accel_z_avg_raw_reading;
               x_deg                       = (short)((getXRadians() * 18000.0 / PI));
               msg.tiltX                   = x_deg;
               y_deg                       = (short)((getYRadians() * 18000.0 / PI));
               msg.tiltY                   = y_deg;
            } else {
               msg.acclAvgXCounts          = 0;
               msg.acclAvgYCounts          = 0;
               msg.acclAvgZCounts          = 0;            
               msg.tiltX                   = 0;           
               msg.tiltY                   = 0;
            }
               
            sendWgStatus( &msg );
            memcpy( &prevWeigher_, &currentWeigher_, sizeof( WeightStateType ) );           
            break;
        }
        case WG_ENABLE: {
            PRINTF("handleWeigherMsg(): Processing message: WG_ENABLE \r\n");   
            enableWeigher();
            break;
        }
        case WG_DISABLE: {
            PRINTF("handleWeigherMsg(): Processing message: WG_DISABLE \r\n");   
            disableWeigher();
            break;
        }
        case WG_MODE: {
            //PRINTF("handleWeigherMsg(): Processing message: WG_MODE \r\n"); 
            /* if we are tracking, before changing mode clear tracking flag */
            if( weigherMode_ == SCALE_WEIGHT_TRACKING_MODE ) {
                initTracking_ = false;
            }
            
            weigherMode_ = pMsg->mode.mode;
			
            /*
            *	Setting weigherMode to SCALE_PREPACK_MODE
            * 	will cause issues like multiple labels being
            * 	printed for a single stable weight. For now,
            *	we're going to reset it to SCALE_MANUAL_MODE.
            */
            if(weigherMode_ == SCALE_PREPACK_MODE) {	
              weigherMode_ = SCALE_MANUAL_MODE;
            }
			   				
            /* if we are changing modes due to the service switch then clear. */
            if( ( currentWeigher_.status & WEIGHER_KEY_PRESSED ) == WEIGHER_KEY_PRESSED ) 
                currentWeigher_.status &= ~WEIGHER_KEY_PRESSED;             
            break;
        }
        case WG_REZERO: {
            //PRINTF("handleWeigherMsg(): Processing message: WG_REZERO \r\n");                             
            state_ = REZERO_;
            break;
        }
        case WG_REQ_VERSION: {
            //PRINTF("handleWeigherMsg(): Processing message: WG_REQ_VERSION \r\n");
            sendWgVersion( &wgVersion_ );
            break;
        }
        case WG_CAT3_REQ_STATISTICS: {
            //PRINTF("handleWeigherMsg(): Processing message: WG_CAT3_REQ_STATISTICS \r\n"); 
                          
            WgCat3Statistics stat;
            /* get a copy of the audit manager's statistics */
            getAuditMgrStats( &stat );
            sendCat3Statistics( &stat );
            break;
        }
        case WG_CAT3_DATE_TIME: {
            //PRINTF("handleWeigherMsg(): Processing message: WG_CAT3_DATE_TIME \r\n");                 
            updateDateTimeEvents( pMsg->dateTime.epochTime );
            break;
        }
        case WG_CAT3_REQ_PAGE: {
            //PRINTF("handleWeigherMsg(): Processing message: WG_CAT3_REQ_PAGE \r\n");

            /* before accessing serial flash, we must first get a lock on it */
            getLockSerialFlash();
            
            /* set the page number scale backend will be reading from */
            setReadPageNumber(pMsg->pageRequest.pageNumber);
            /* have the audit manager read the requested page */
            readAuditPage( ( pMsg->pageRequest.pageNumber * PAGE_SIZE )  );
            /* set the index for the creation of the record message */
            setReadPageIndex( pMsg->pageRequest.index );
             
            WgCat3Records records;
            memset( &records, 0, sizeof(WgCat3Records) );
            
            records.pageNumber = getReadPageNumber();
            records.index = getReadPageIndex();
            /* is the audit trail page is blank? */
            if( isAuditPageBlank() ) {
                 /* clear the 10 log records instead of sending all 0xFF's */
                 memset( &records.records[0], 0,   (sizeof(AuditRecord) * 10) );
            }
            else {
                /* copy the records into the message starting at the readPageIndex. */
                readRecords( &records.records[0] );                     

                /* check for blank records and null before sending */
                for( int i = 0; i < 10; i++ ) {
                    AuditRecord *pRecord = (AuditRecord *)&( records.records[ i * sizeof(AuditRecord) ] );
                    
                    if( pRecord->recordTag ==  BLANK_RECORD ) {
                        memset( pRecord, 0, sizeof(AuditRecord) );
                    }
                }
            }
            /* release the lock onthe serial flash */
            releaseLockSerialFlash();

            sendCat3Records( &records );          
            break;
        }
        case WG_CAT3_REQ_NEXT_RECORDS: {
            //PRINTF("handleWeigherMsg(): Processing message: WG_CAT3_REQ_NEXT_RECORDS \r\n"); 
            WgCat3Records records;
            
           /* clear the record memory (packing issue)*/
            memset( &records, 0, sizeof(WgCat3Records) );
                     
            /* before accessing serial flash, we must first get a lock on it */
            getLockSerialFlash();
            
            records.pageNumber = getReadPageNumber();
            records.index = getReadPageIndex();
            /* is the audit trail page is blank? */
            if( isAuditPageBlank() ) {
                 /* clear the 10 log records instead of sending all 0xFF's */
                 memset( &records.records[0], 0,   (sizeof(AuditRecord) * 10) );
            } else {
                /* copy the records into the message starting at the readPageIndex. */
                readRecords( &records.records[0] );                     

                /* check for blank records and null before sending */
                for( int i = 0; i < 10; i++ ) {
                    AuditRecord *pRecord = (AuditRecord *)&( records.records[ i * sizeof(AuditRecord) ] );
                    if( pRecord->recordTag ==  BLANK_RECORD ) {
                        memset( pRecord, 0, sizeof(AuditRecord) );
                    }
                }
            }                       
            /* release the lock onthe serial flash */
            releaseLockSerialFlash();

            sendCat3Records( &records );      
            break;
        }
        case WG_CAT3_WRITE_RECORD: {
            //PRINTF("handleWeigherMsg(): Processing message: WG_CAT3_WRITE_RECORD \r\n"); 
           
            /* before accessing serial flash, we must first get a lock on it */
            getLockSerialFlash();

            /* update Cat3 date and time events */ 
            updateDateTimeEvents( pMsg->writeRecord.epochTime );
                 
            /* release the lock onthe serial flash */
            releaseLockSerialFlash();

            /* create new audit record from message and have the audit manager save to audit trail */
            AuditRecord record;
            /* clear the record memory (packing issue)*/
            memset( &record, 0, sizeof(AuditRecord) );
            
            record.recordTag = 0;
            record.event = (AuditEvent)pMsg->writeRecord.event;
            record.parameterId = (CFGParamID)pMsg->writeRecord.parameterId;
            /* we only save the none weigher sealable parameters listed below */
            if(	(record.parameterId == FRACTIONAL_PRICING)              || (record.parameterId == POUNDS_FOR_MULTIPLIER)     ||
                (record.parameterId == SOFTWARE_UPDATE)                 || (record.parameterId == DECIMAL_SEPARATOR)         ||
                (record.parameterId == MONETARY_RULE)                   || (record.parameterId == TOTAL_PRICE_DIGITS)        ||
                (record.parameterId == UNIT_PRICE_DIGITS)               || (record.parameterId == ROUNDING_FACTOR)           ||
                (record.parameterId == ROUNDING_METHOD)			|| (record.parameterId == PRIMARY_MONETARY_SYMBOL)   ||
                (record.parameterId == PRIMARY_MONETARY_DECIMAL_DIGITS) || (record.parameterId == SECONDARY_MONETARY_SYMBOL) ||
                (record.parameterId == SECONDARY_MONETARY_DECIMAL_DIGITS) )
            {
                if( record.parameterId == SOFTWARE_UPDATE ) {
                    record.event = UPGRADE;
                } else {
                    record.event = CONFIGURATION;
                }
                record.eventType = (AuditType)pMsg->writeRecord.eventType;
                memset( (&record.unused[0]), 0, sizeof(record.unused) );     
                record.epochTime = pMsg->writeRecord.epochTime;
                record.newParamValue = pMsg->writeRecord.newValue;
                record.oldParamValue = pMsg->writeRecord.oldValue;
                record.checksum = calcRecordCheckSum( &record );

                WgCat3RecordStatus status;   

                /* before accessing serial flash, we must first get a lock on it */
                getLockSerialFlash();

                if( auditManagerSaveRecord( &record ) ) {                        
                    /*audit record saved successfully */
                    status.saved = true;
                } else {
                    status.saved = false;
                }
                
                /* release the lock onthe serial flash */
                releaseLockSerialFlash();

                sendCat3RecordStatus( &status );
            }  
            break;
        }
        case WG_CAT3_REQ_ERASE_LOG: {
            //PRINTF("handleWeigherMsg(): Processing message: WG_CAT3_WRITE_RECORD \r\n");                        
            /* before accessing serial flash, we must first get a lock on it */
            getLockSerialFlash();

            eraseAuditLog();

            /* release the lock onthe serial flash */
            releaseLockSerialFlash();

            sendCat3LogEraseComplete();
            break;
        }
        
        /**************************************************************************************************
         * TFink 5/16/24 - none of these Value Max messages are used by the scale application. They are only
         * used by the Mfr Test Fixture which is based on HT/FS code. So these messages may never be used.
         * They were tested as best as possible, mostly the sub functions (such as readVmAssemblyNumber() or 
         * writeFitCalibrationAll() that are used elsewhere 
         *************************************************************************************************/
        case WG_VM_REQUEST:
        {   
           
            switch(pMsg->vmRequest.req )
            {
                case VMREQ_ASSEMBLY: {
                    queueVMTransactionRequest(VM_EEP_READ_ASSEMBLY_SEND);
                    break;
                }
                case VMREQ_HWVERSION: {
                    queueVMTransactionRequest(VM_EEP_READ_VERSION_SEND);       
                    break;
                }
                case VMREQ_SERIALNUM: {
                    queueVMTransactionRequest(VM_EEP_READ_SERIAL_SEND);            
                    break;
                }
                case VMREQ_DATE: {
                    queueVMTransactionRequest(VM_EEP_READ_DATE_SEND);    
                    break;
                }
                case VMREQ_FIT_VALUES: {
                    /* send data that was retreived at bootup */
                    ValueMax *pVM = getPtrToValueMax();
                    sendWgVMFitValues(&(pVM->fit_data)); 
                    break;
                }
            }
            break;
        }
        case WG_VM_WRITE_ASSEMBLY: {
            ValueMax *pVM = getPtrToValueMax();
            memset(&pVM->assemblyNumber, 0, sizeof(pVM->assemblyNumber));
            memcpy(&pVM->assemblyNumber, &pMsg->vmAssembly.assemblyNumber[0], sizeof(pVM->assemblyNumber));
            queueVMTransactionRequest(VM_EEP_WRITE_ASSEMBLY);
            
            break;
        }
        case WG_VM_WRITE_VERSION: {           
            ValueMax *pVM = getPtrToValueMax();
            memset(&pVM->version, 0, sizeof(pVM->version));            
            memcpy(&pVM->version, &pMsg->vmVersion.version, sizeof(pVM->version));            
            queueVMTransactionRequest(VM_EEP_WRITE_VERSION);
            break;
        }
        case WG_VM_WRITE_SERIAL: {	              
          ValueMax *pVM = getPtrToValueMax();  
          memset(&pVM->serial, 0, sizeof(pVM->serial));
          memcpy(&pVM->serial, &pMsg->vmSerial.serial[0], sizeof(pVM->serial));
          queueVMTransactionRequest(VM_EEP_WRITE_SERIAL);          
          break;          
        }
        case WG_VM_WRITE_DATE: {
            ValueMax *pVM = getPtrToValueMax();           
            pVM->epoch = pMsg->vmDate.epoch;
            queueVMTransactionRequest(VM_EEP_WRITE_DATE);
           break;
        }
        case WG_CONTROL: {
            //PRINTF("handleWeigherMsg(): Message no longer used: WG_CONTROL \r\n" ); 
            break;
        }
        case WG_REQ_SYSTEM_INFO: {                        
            WgInfo info;
            info.msgType = WG_SYSTEM_INFO;
            info.serialFlashValid = configValid_;
            info.eepromValid = true;
            info.softwareVersion[0] = wgVersion_.swMajor;
            info.softwareVersion[1] = wgVersion_.swMinor;
            info.softwareVersion[2] = wgVersion_.swBuild;
            info.hardwareVersion[0] = wgVersion_.hwMajor;
            info.hardwareVersion[1] = wgVersion_.hwMinor;
            sendWgSystemInfo( &info );
            /* host requests this when manager is created */
            hostConnected_ = true;
            
            #ifndef TFinkStartCS5530woApp
            /* backend weigher manager started so start our ext a/d */
            startcs5530PollTimer( CS5530_RATE );
            #endif
            break;
        }
        
        case WG_VM_WRITE_FIT_VALUES: {
            ValueMax *pVM = getPtrToValueMax();             
            memset(&pVM->fit_data, 0, sizeof(fit_flash_section));            
            /* copy cal data into temp buffer */
            memcpy(&pVM->fit_data, &pMsg->vmFit.fit, sizeof(fit_flash_section)); 
            queueVMTransactionRequest(VM_EEP_WRITE_FIT_CALIBRATION);         
            break;
        }
        
        case WG_REQ_SYS_INFO: {
            WgSysInfo info;
            info.msgType = WG_SYS_INFO;
            info.pid = getProductId();
            info.valueMaxEnabled = (getMyModel() == RT_GLOBAL_SCALE_BEST);
            
            sendWgSysInfo( &info );   
            break;
        }
        
        default: {
            PRINTF("handleWeigherMsg(): Unknown Weigher message from queue!\r\n" ); 
            break;
        }
    }
}

/******************************************************************************/
/*!   \fn void weigherCountsStartStop( bool start )

      \brief
        This function enables or disables the interrupt to the external a/d.
   
      \author
          Aaron Swift
*******************************************************************************/
void weigherCountsStartStop( bool start )
{
    if( start ) {
        enableWeigher();
    } else {
        disableWeigher();
    }  
}


/******************************************************************************/
/*!   \fn static void initializeFilter( int speed  )

      \brief
        This function sets the filter depth based on speed.
   
      \author
          Aaron Swift
*******************************************************************************/
static void initializeFilter( int speed )
{
    if ( !filteredReadingsInit )
    {
        boxcarFilterInit( &filteredReadings, MAX_FILTER_READINGS );
        filteredReadingsInit = true;
    }

    switch ( speed )
    {
    case FAST_FILTER_SPEED:
        boxcarFilterSetMaxLength( &filteredReadings, 6 );
        break;
    case SLOW_FILTER_SPEED:
        boxcarFilterSetMaxLength( &filteredReadings, 10 );
        break;
    case NORMAL_FILTER_SPEED:
    default:
        boxcarFilterSetMaxLength( &filteredReadings, 8 );
        break;
    }
}

/******************************************************************************/
/*!   \fn WeigherStyle getWeigherStyle( void )

      \brief
        This function returns the weigher task handle.
   
      \author
          Aaron Swift
*******************************************************************************/
TaskHandle_t getWeigherHandle( void )
{
    return pHandle_;
}

/******************************************************************************/
/*!   \fn WeigherStyle getWeigherStyle( void )

      \brief
        This function returns the weigher style (model).
   
      \author
          Aaron Swift
*******************************************************************************/
WeigherStyle getWeigherStyle( void )
{
    return style_;
}

/******************************************************************************/
/*!   \fn void setWeigherVersion( WgVersion *pVersion )

      \brief
        This function sets the weigher version information
   
      \author
          Aaron Swift
*******************************************************************************/
static void setWeigherVersion( WgVersion *pVersion )
{
    pVersion->pid = getProductId();
    pVersion->swMajor = getWeigherSoftwareIDMajor();
    pVersion->swMinor = getWeigherSoftwareIDMinor();
    pVersion->swBuild = getWeigherSoftwareIDEng();
    pVersion->firmware = 0;  
    pVersion->hwMajor = getHardwareIDMajor();
    pVersion->hwMinor = getHardwareIDMinor();
}


void enableWeigher( void )
{
    /* set weigher reset signal (active-low) */
    GPIO_WritePinOutput(WGR_RESET_GPIO, WGR_RESET_PIN, true);
    currentWeigher_.status &= ~WEIGHER_DISABLED;
    
    /* set the gpio for edge interrupt */
    setConversionInterrupt(); 
}

void disableWeigher( void )
{
    /* clear weigher reset signal (active-low) */
    GPIO_WritePinOutput(WGR_RESET_GPIO, WGR_RESET_PIN, false);
    
    /* Clear external data ready interrupt flag. */
    GPIO_PortClearInterruptFlags(GPIO2, 1 << 13U);

    /* Disable GPIO edge triggered interrupt */
    DisableIRQ( GPIO2_Combined_0_15_IRQn );
    GPIO_DisableInterrupts( GPIO2, 1U << 13U );

    currentWeigher_.status |= WEIGHER_DISABLED;

    /* empty the a/d count queue */
    xQueueReset( adQHandle_ );
}

/******************************************************************************/
/*!   \fn void valueMaxHandleAcclMeasurement( ValueMax *VM, SCA3300 *acclData )

      \brief
        This function handles calculating the scale's degrees off level
  
      \author
          Carlos Guzman / Joseph DiCarlantonio
*******************************************************************************/
void valueMaxHandleAcclMeasurement( ValueMax *VM, SCA3300 *acclData )  
{
    static bool buffer_filled = false;
    long oldest_x_raw_reading;
    long oldest_y_raw_reading;
    long oldest_z_raw_reading;
    double avg_x_raw_reading,avg_y_raw_reading,avg_z_raw_reading;
    double avg_x_raw_reading_scaled,avg_y_raw_reading_scaled,avg_z_raw_reading_scaled;
    double x_prime, y_prime, z_prime;
    
    /** load x,y,z readings. Swap from what they are called in SCA3300 because in HT scale, which this code is based on,
       y axis is vertical. FIT values were calculated with Y axis being vertical **/
    VM->accel_x_raw_reading = -acclData->xAccel;
    VM->accel_y_raw_reading =  acclData->zAccel; 
    VM->accel_z_raw_reading = -acclData->yAccel;
   
    if( !buffer_filled ) {
    /*** Fill Buffer *****/   
        /* buffer is filling up for 1st time still */
        VM->accel_x_raw_reading_buffer[VM->acceleromter_sample_counter - 1] = VM->accel_x_raw_reading;
        VM->accel_x_accum_raw_reading += VM->accel_x_raw_reading;
        
        VM->accel_y_raw_reading_buffer[VM->acceleromter_sample_counter - 1] = VM->accel_y_raw_reading;
        VM->accel_y_accum_raw_reading += VM->accel_y_raw_reading;
        
        VM->accel_z_raw_reading_buffer[VM->acceleromter_sample_counter - 1] = VM->accel_z_raw_reading;
        VM->accel_z_accum_raw_reading += VM->accel_z_raw_reading;

        
        if(VM->acceleromter_sample_counter == MAX_ACCEL_RUNNING_SAMPLES) {
            buffer_filled = true;
        }   
    } else {
    /*** Now treat buffer as a FIFO and calculate average value ***/
        VM->accel_minimum_samples_reached = true;
        
        /* x */
        oldest_x_raw_reading = VM->accel_x_raw_reading_buffer[0];
        memmove(  //TFink why are we copying the whole buffer. Save bandwidth by using a pointer to a rotating buffer
            &VM->accel_x_raw_reading_buffer[0], 
            &VM->accel_x_raw_reading_buffer[1], 
            ( ( MAX_ACCEL_RUNNING_SAMPLES - 1 ) * 4 )
        );
        VM->accel_x_raw_reading_buffer[MAX_ACCEL_RUNNING_SAMPLES - 1] = VM->accel_x_raw_reading;
        
        VM->accel_x_accum_raw_reading -= oldest_x_raw_reading;
        VM->accel_x_accum_raw_reading += VM->accel_x_raw_reading;
        VM->accel_x_avg_raw_reading    = VM->accel_x_accum_raw_reading / MAX_ACCEL_RUNNING_SAMPLES;
        avg_x_raw_reading                = (double)VM->accel_x_accum_raw_reading / MAX_ACCEL_RUNNING_SAMPLES;
        
        /* y */
        oldest_y_raw_reading = VM->accel_y_raw_reading_buffer[0];
        memmove( 
            &VM->accel_y_raw_reading_buffer[0], 
            &VM->accel_y_raw_reading_buffer[1], 
            ( ( MAX_ACCEL_RUNNING_SAMPLES - 1 ) * 4 ) 
        );
        VM->accel_y_raw_reading_buffer[MAX_ACCEL_RUNNING_SAMPLES - 1] = VM->accel_y_raw_reading;
        
        VM->accel_y_accum_raw_reading -= oldest_y_raw_reading;
        VM->accel_y_accum_raw_reading += VM->accel_y_raw_reading;
        VM->accel_y_avg_raw_reading    = VM->accel_y_accum_raw_reading / MAX_ACCEL_RUNNING_SAMPLES;
        avg_y_raw_reading                = (double)VM->accel_y_accum_raw_reading / MAX_ACCEL_RUNNING_SAMPLES;
        
        /* z */
        oldest_z_raw_reading = VM->accel_z_raw_reading_buffer[0];
        memmove( 
            &VM->accel_z_raw_reading_buffer[0], 
            &VM->accel_z_raw_reading_buffer[1], 
            ( ( MAX_ACCEL_RUNNING_SAMPLES - 1 ) * 4 )
        );
        VM->accel_z_raw_reading_buffer[MAX_ACCEL_RUNNING_SAMPLES - 1] = VM->accel_z_raw_reading;
        
        VM->accel_z_accum_raw_reading -= oldest_z_raw_reading;
        VM->accel_z_accum_raw_reading += VM->accel_z_raw_reading;
        VM->accel_z_avg_raw_reading    = VM->accel_z_accum_raw_reading / MAX_ACCEL_RUNNING_SAMPLES;
        avg_z_raw_reading                = (double)VM->accel_z_accum_raw_reading / MAX_ACCEL_RUNNING_SAMPLES;
        
    }
 
    /* Scale readings to G */
    //TFink: the divide isn't necessary for the math to work. I tried it. But it works real well as-is
    avg_x_raw_reading_scaled = avg_x_raw_reading / SCA3300_LSB_G;
    avg_y_raw_reading_scaled = avg_y_raw_reading / SCA3300_LSB_G;
    avg_z_raw_reading_scaled = avg_z_raw_reading / SCA3300_LSB_G;
   
    /* Rotate frame */
    x_prime = avg_x_raw_reading_scaled * fitParam.xrot[0] + avg_y_raw_reading_scaled * fitParam.xrot[1] + avg_z_raw_reading_scaled * fitParam.xrot[2];
    y_prime = avg_x_raw_reading_scaled * fitParam.yrot[0] + avg_y_raw_reading_scaled * fitParam.yrot[1] + avg_z_raw_reading_scaled * fitParam.yrot[2];
    z_prime = avg_x_raw_reading_scaled * fitParam.zrot[0] + avg_y_raw_reading_scaled * fitParam.zrot[1] + avg_z_raw_reading_scaled * fitParam.zrot[2];
    
    /* Calculate Tilt in radians */
    VM->accel_x_radians = atan(x_prime / sqrt(y_prime*y_prime + z_prime*z_prime)) * fitParam.xgain;
    VM->accel_y_radians = atan(y_prime / sqrt(x_prime*x_prime + z_prime*z_prime)) * fitParam.ygain;

 
    /*  check to see if tilt is greater then 3 degrees */
    VM->x_degrees   = VM->accel_x_radians * 180 / PI;    
    VM->y_degrees   = VM->accel_y_radians * 180 / PI;
    
    /* Print calculated data - debug only */
    if(buffer_filled) {
       static unsigned short printfThrottle = 0;
       printfThrottle++;
    }

    
    /* determine if max tilt is reached */
    if( (fabs(VM->x_degrees) > VM_MAX_TILT  || fabs(VM->y_degrees) > VM_MAX_TILT) && config.value_max_on_off == true ) {
        /* Send error to UI */
        currentWeigher_.status |= WEIGHER_VM_MAX_TILT_REACHED;
        if(buffer_filled)
           PRINTFThrottle(140,"\r\n\r\nWeigher Max Tilt Reached!!!\r\n\r\n");  //angle isn't calculated until buffer is filled
    }
    else {
        currentWeigher_.status &= ~WEIGHER_VM_MAX_TILT_REACHED;
    }
}



/******************************************************************************/
/*!   \fn static static void initFit(fit_flash_section *fitParamFlash)

      \brief
        This function converts the fit values stored in Serial EEP to double 
        values used by the Global Application struct: fitParam.
   
      \author
          Carlos Guzman
*******************************************************************************/
void initFit(fit_flash_section *fitParamFlash) 
{
    /** See weigher.h" for original definition of these constants **/
    
    /* Gravity */
    fitParam.g = (double) fitParamFlash->g / RESOLUTION_1M; 
     /* Pounds/Counts conversion factor.  "gravConv = gainFactor / fitParam.m" used as part of loadcell front/back and left/right compensation and to calculate deadWeight */
    fitParam.m = (double) fitParamFlash->m / RESOLUTION_10M;       

    /* Counts at zero load - not used */
    fitParam.zerovalue = (double) fitParamFlash->zerovalue;
    /* Cell span (max load - zero load) - not used */
    fitParam.spanvalue = (double) fitParamFlash->spanvalue;
    /* Electronic offset - not used */
    fitParam.Z0 = (double) fitParamFlash->Z0;
    /* The deadweight (as measured at factory?) */
    fitParam.D = (double) fitParamFlash->D;                        

    /* used to calc Sx_error (correction for loadcell front-back tilt) */
    fitParam.x2 = (double) fitParamFlash->x2 / RESOLUTION_10M;      
    fitParam.x1 = (double) fitParamFlash->x1 / RESOLUTION_100K;
    fitParam.x0 = (double) fitParamFlash->x0 / RESOLUTION_100;

    /* used to calc Sy_error (correction for loadcell front-back tilt) */
    fitParam.y2 = (double) fitParamFlash->y2 / RESOLUTION_10M;
    fitParam.y1 = (double) fitParamFlash->y1 / RESOLUTION_100K;
    fitParam.y0 = (double) fitParamFlash->y0 / RESOLUTION_100;

    /* Used to rotate reference frame (to "true north"?) */
    fitParam.xrot[0] = (double) fitParamFlash->xrot[0] / RESOLUTION_10M;
    fitParam.xrot[1] = (double) fitParamFlash->xrot[1] / RESOLUTION_10M;
    fitParam.xrot[2] = (double) fitParamFlash->xrot[2] / RESOLUTION_10M;

    fitParam.yrot[0] = (double) fitParamFlash->yrot[0] / RESOLUTION_10M;
    fitParam.yrot[1] = (double) fitParamFlash->yrot[1] / RESOLUTION_10M;
    fitParam.yrot[2] = (double) fitParamFlash->yrot[2] / RESOLUTION_10M;

    fitParam.zrot[0] = (double) fitParamFlash->zrot[0] / RESOLUTION_10M;
    fitParam.zrot[1] = (double) fitParamFlash->zrot[1] / RESOLUTION_10M;
    fitParam.zrot[2] = (double) fitParamFlash->zrot[2] / RESOLUTION_10M;

    /* Used to scale the X & Y angles after frame rotation */
    fitParam.xgain = (double) fitParamFlash->xgain / RESOLUTION_100K;  //X Gain Factor
    fitParam.ygain = (double) fitParamFlash->ygain / RESOLUTION_100K;  //Y Gain Factor

    /* Not used? Gravity Coefficient*/
    fitParam.g_coeff = (double) fitParamFlash->g_coeff / RESOLUTION_100K;
    
    #if 0  //The following "zeros out" the rotation - for debug purposes
    fitParam.xrot[0] = (double) 1;
    fitParam.xrot[1] = (double) 0;
    fitParam.xrot[2] = (double) 0;

    fitParam.yrot[0] = (double) 0;
    fitParam.yrot[1] = (double) 1;
    fitParam.yrot[2] = (double) 0;

    fitParam.zrot[0] = (double) 0;
    fitParam.zrot[1] = (double) 0;
    fitParam.zrot[2] = (double) 1;

    fitParam.xgain = (double) 1;
    fitParam.ygain = (double) 1;

    fitParam.g_coeff = (double) 992266;
    #endif   
}

/******************************************************************************/
/*!   \fn void serviceSwitchPressed( )

      \brief
       
   
      \author
          Aaron Swift
*******************************************************************************/
void serviceSwitchPressed( void )
{
    /* start to debounce the switch */
    debounce_ = true; 
    if( switchBounce_ ) {
        if( xTimerStart( switchBounce_, 0 ) == pdFAIL ) {
            PRINTF("serviceSwitchPressed(): failed to start debounce timer!\r\n");  
        }
    }
    serviceSwitch_ = true;
    currentWeigher_.status |= WEIGHER_KEY_PRESSED;
    //PRINTF("Service Switch Pressed\r\n");
}

/******************************************************************************/
/*!   \fn void isServiceSwitchPressed( )

      \brief
         Returns true if the service switch has been pressed.
         As of 3/20/24 I'm using this for test/debug purposes only
   
      \author
          Tom Fink
*******************************************************************************/
bool isServiceSwitchPressed(void)
{
   if((currentWeigher_.status & WEIGHER_KEY_PRESSED) == WEIGHER_KEY_PRESSED)
      return(true);
   else
      return(false);
}


/******************************************************************************/
/*!   \fn static void clearServiceSwitch( void )

      \brief
        This function clears the service switch flag.

      \author
          Aaron Swift
*******************************************************************************/
static void clearServiceSwitch( void )
{
    serviceSwitch_ = false;
}

/******************************************************************************/
/*!   \fn static long calibrateCounts( unsigned long counts )

      \brief
        This function calibrates the A/D counts.       
   
      \author
          Aaron Swift
*******************************************************************************/
static long calibrateCounts( unsigned long counts )
{
   long long calibrated_count;
    
    double offset_counts = (double)counts - electronicOffset;
    double gainFactor = ((double)config.gain_factor / RESOLUTION_10M);

    double Wb_adj;

    // Print Weight with VM OFF
    calibrated_count = (int32_t) (offset_counts * gainFactor);
	
    if( isAccelerometerInitialized() == true && config.value_max_on_off == true  )
    {
        double angle;
        double gravConvert;
        double Wb,Sx_load,Sx_error,Sx,Sy_load,Sy_error,Sy;
        //PRINTFThrottle(300,"\r\n*** calibrateCounts(): calc counts with VM ***\r\n\r\n");
         
        gravConvert = gainFactor / fitParam.m;

        angle = 1.0 / sqrt(1+pow(tan(getXRadians()),2.0)+pow(tan(getYRadians()),2.0));  //convert X and Y angles to one "combined angle"

        /* Wb = load + dead load adjusted for cosine error */
        Wb = offset_counts / angle;

        /* Sx = correction for front-back tilt */
        Sx_load = ((getXRadians() < 0.0) ? -1.0 : 1.0) * (Wb/gravConvert);

        Sx_error = fitParam.x2/RESOLUTION_10M*pow(Sx_load,2) + fitParam.x1/RESOLUTION_100K*Sx_load + fitParam.x0;

        Sx =  sin( getXRadians() ) * Sx_error;

        /* Sy = correction for left-right tilt */
        Sy_load = ((getYRadians() < 0.0) ? -1.0 : 1.0) * (Wb/gravConvert);

        Sy_error = fitParam.y2/RESOLUTION_10M*pow(Sy_load,2) + fitParam.y1/RESOLUTION_100K*Sy_load + fitParam.y0;

        Sy =  fabs(sin( getYRadians() ))  * Sy_error;

        Wb_adj = Wb + Sx + Sy ;
    }
    else
    {
        //PRINTFThrottle(300,"\r\n*** calibrateCounts(): calc counts wo VM ***\r\n\r\n");
        Wb_adj = offset_counts;
    }

    // adjusted counts
    calibrated_count = (int32_t) (Wb_adj * gainFactor);

    return (calibrated_count);
}

/******************************************************************************/
/*!   \fn static void setZeroTracking( void )

      \brief
              
   
      \author
          Aaron Swift
*******************************************************************************/
static void setZeroTracking( void )
{
        /*************************************************************
        * Force the powerup values into the zero tracking variables. *
        *************************************************************/
        zeroCenter = config.center_of_maintenance_zone;
}

/******************************************************************************/
/*!   \fn static UCHAR calculateSlope( WeightStateType *current, WeightStateType *previous )

      \brief
        This function determines if the counts have a positive or negative slope.      
   
      \author
          Aaron Swift
*******************************************************************************/
static unsigned char calculateSlope( WeightStateType *current, WeightStateType *previous )
{
    long delta = current->zeroedCalibrCounts - previous->zeroedCalibrCounts;

    if( delta > 0 ) 
        /* slope is positive */ 
        return( PLUS );
    else if( delta < 0 )
        /* slope is negative */ 
        return( MINUS );  
    else   
        /* slope is the opposite sign of the last slope */ 
        return( ~(previous->slope) );
}

/******************************************************************************/
/*!   \fn static bool validZero( void )

      \brief
        This function determines if the current zero is within operational maint
        limit.      
   
      \author
          Aaron Swift
*******************************************************************************/
static bool validZero( void )
{
    double gainFactor = ((double)config.gain_factor / RESOLUTION_10M);
    double limit;
    
    bool rv = false;

    /***********************************************************
    * The Maintence Zone Limit is 10 % of scale capacity at    *
    * power up and 2 % of scale capacity the rest of the time. *
    ***********************************************************/
    if( weigherPowerUp ) {
        limit = (double) POWERUP_MAINTENANCE_ZONE_LIMIT / gainFactor;
      
        /****************************************************************
        * Perform this test only at power up until the first valid zero *
        * is selected                                                   *
        ****************************************************************/
        if( labs( currentZeroAvgFilterCounts - zeroCenter ) <= (long) limit ) {			
            weigherPowerUp = false;

            /**************************************************************
            * A valid zero has been selected, now change all of the valid *
            * zero criteria to support the 2 % of scale capacity          *
            * maintenance zone requirement.                               *
            **************************************************************/
            setZeroTracking();

            if ( powerUp_ )
                powerUp_ = false;
 
            rv = true;
        }
    }
    else
    {
        limit = (double) OPERATE_MAINTENANCE_ZONE_LIMIT / gainFactor;
      
        if ( labs( currentZeroAvgFilterCounts - zeroCenter ) <= (long) limit ) {
            rv = true;
        }
    }
    return rv;
}

/******************************************************************************/
/*!   \fn static bool autoZero( void )

      \brief
        This function
        limit.      
   
      \author
          Aaron Swift
*******************************************************************************/
static bool autoZero( void )
{
    static int timer = 3 * N_READINGS_PER_SECOND;

    long azsm_limit;

    if ( ( config.flags & WEIGH_MODE_AVOIR ) == WEIGH_MODE_AVOIR)
    {
        azsm_limit = AUTO_ZERO_MAINTENANCE_LIMIT_AVOIR_DR;
    }
    else
    {
        azsm_limit = AUTO_ZERO_MAINTENANCE_LIMIT_METRIC_DR;
    }
    
    if ((labs(weight_) >= azsm_limit) ||   
        (labs(weight_ - currentWeigher_.zeroedCalibrCounts) > AUTO_ZERO_MOTION_LIMIT) ||
        ((prevWeigher_.status & SMALL_MOTION) == SMALL_MOTION) )
    {
        // Reset the timer
        timer = 3 * N_READINGS_PER_SECOND;
        return (false);
    }
    else if (--timer < 0)
    {
        // Reset the timer.
        timer = 3 * N_READINGS_PER_SECOND;
        return (true);
    }
    else
    {
        return (false);
    }	

}

/******************************************************************************/
/*!   \fn static void updateParameters( void )

      \brief
        This function
        limit.      
   
      \author
          Aaron Swift
*******************************************************************************/
static void updateParameters( void )
{
    /* all weigher models that support dual range */
    if( ( ( config.flags & WEIGH_MODE_AVOIR ) == WEIGH_MODE_AVOIR ) /*&&
        ( ( config.weigher_model == GL_SERVICE_SCALE_WEIGHER ) )*/ ){
        /* avoir dual range */
        //AutoZeroMaintenanceLimit = AUTO_ZERO_MAINTENANCE_LIMIT_AVOIR_DR; //TFink 5/2/24 set but not used
        OneHalfGraduation = ONE_HALF_GRADUATION_AVOIR_DR;
        OneQuarterGraduation = ONE_QUARTER_GRADUATION_AVOIR_DR;

    } else if( ( ( config.flags & WEIGH_MODE_METRIC ) == WEIGH_MODE_METRIC ) /*&&
                 ( ( config.weigher_model == GL_SERVICE_SCALE_WEIGHER ) )*/ ) {
        /* metric dual range */
        //AutoZeroMaintenanceLimit = AUTO_ZERO_MAINTENANCE_LIMIT_METRIC_DR; //TFink 5/2/24 set but not used
        OneHalfGraduation = ONE_HALF_GRADUATION_METRIC_DR;
        OneQuarterGraduation = ONE_QUARTER_GRADUATION_METRIC_DR;
    } else {
        PRINTF("updateParameters(): unknown weigher");
        /* an unsupported weigher model has been selected! */
        config.weigher_model = G_UNKNOWN_WEIGHER19;
    }
}

/******************************************************************************/
/*!   \fn static void initializeZero( unsigned long counts )

      \brief
        This function
        limit.      
   
      \author
          Aaron Swift
*******************************************************************************/
static void initializeZero( unsigned long counts )
{
    static int timer = 3 * N_READINGS_PER_SECOND;
    double gainFactor = ((double)config.gain_factor / RESOLUTION_10M);
    
    double gravConv;
    double deadWeight;
    double deadWeightTiltComp = 0;
 
    currentWeigher_.status |= SCALE_WARMUP;

    currentWeigher_.calibratedCounts = calibrateCounts( counts );
    currentWeigher_.zeroedCalibrCounts  = currentWeigher_.calibratedCounts - currentZero;
    currentWeigher_.slope = calculateSlope( &currentWeigher_, &prevWeigher_ );

       
	if ( ( state_ != INIT_ZERO_ ) || 
	   ( labs( weight_ - currentWeigher_.calibratedCounts ) > AUTO_ZERO_MOTION_LIMIT ) ) {
		   
          /* is Accl working */
          if(isAccelerometerInitialized() == true && config.value_max_on_off == true) {     
             /* Ok Accl is present, make sure we've taken enough readings to normaize Tilt, and compensate counts */
             if(accelMinSamplesReached()) {
                gravConv = gainFactor / fitParam.m;
                deadWeight = fitParam.D * gravConv;
                PRINTF("gainF: %.4f, fitPm: %.4f, fitPd: %.4f\r\n", gainFactor, fitParam.m, fitParam.D);
                deadWeightTiltComp = deadWeight / sqrt( 1 + pow( tan( getXRadians() ), 2.0 ) + pow( tan( getYRadians() ), 2.0 ) );
                PRINTF("dwTiltComp %.4f deadWeight %.4f\r\n",deadWeightTiltComp, deadWeight);
                state_ = INIT_ZERO_;
                PRINTF("initializeZero(): finding zero w Value Max on\r\n");
             }
		}
		else {	
		   /* No Accl, just use uncompensated counts */
		   state_ = INIT_ZERO_;	 
           //PRINTF("initializeZero(): finding zero w Value Max off\r\n");
		}

        electronicOffset = ((long)counts - (long)deadWeightTiltComp); //TFink 5/15/24 Was "(long)(counts - deadWeightTiltComp)". I verifed both give same result
        if(accelMinSamplesReached())
           PRINTF("EO calc: counts: %d, DW: %.2f, Elec Offset: %d\r\n",counts, deadWeightTiltComp, electronicOffset);
             
		/* update our current weight */
		weight_ = currentWeigher_.calibratedCounts;

		/* set timer to full amount */
		timer = config.initialize_zero_time * N_READINGS_PER_SECOND;            
		
	} else if( --timer <= 0 ) {
        //PRINTF("initializeZero(): eOS: %d  counts: %d  dwTiltComp: %.4f \r\n", electronicOffset, counts, deadWeightTiltComp);
		
		/* get a new zero */
		currentZero = weight_;
		currentZeroAvgFilterCounts = counts;

		/* reset our current weight */
		weight_ = 0;

		currentWeigher_.status |= NEW_ZERO;

		if( validZero() == false ) {
			 currentWeigher_.status |= ZERO_OUT_OF_MAINTENANCE_ZONE;
		}

		if( errorState_ ) {			
			   /* weigher serial is corrupt, so lock up the scale. */    
			   state_ = LOCKUP_;
			   PRINTF("\r\n\r\n\r\ninitializeZero(): Serial flash corrupt! Lock state set! \r\n\r\n\r\n" ); 
		} else {
			/* begin detecting motion. */ 
			state_ = DETECTMOTION_;
		}
	}

}

/******************************************************************************/
/*!   \fn static void rezeroWeigher( unsigned long counts )

      \brief
        This function     
   
      \author
          Aaron Swift
*******************************************************************************/
static void rezeroWeigher( unsigned long counts )
{
    currentWeigher_.status |= SCALE_WARMUP;

    currentWeigher_.calibratedCounts = calibrateCounts( counts );
    currentWeigher_.zeroedCalibrCounts  = currentWeigher_.calibratedCounts - currentZero;
    currentWeigher_.slope = calculateSlope( &currentWeigher_, &prevWeigher_ );

    /* check if full calibration is needed  */	
    if (( powerUp_ ) || ( weigherPowerUp ) ) {    
        /* do a full hardware calibration. */ 
        state_ = ADRESET_;
    }
    else {
        /* just select a new zero. */ 
        state_ = INIT_ZERO_;  
    }
}

/******************************************************************************/
/*!   \fn static void lockUp( unsigned long counts )

      \brief
        This function
        limit.      
   
      \author
          Aaron Swift
*******************************************************************************/
static void lockUp( unsigned long counts )
{
    if ( state_ != LOCKUP_ )
        state_ = LOCKUP_;

    currentWeigher_.calibratedCounts = calibrateCounts( counts );
    currentWeigher_.zeroedCalibrCounts = currentWeigher_.calibratedCounts - currentZero;
    currentWeigher_.slope = calculateSlope( &currentWeigher_, &prevWeigher_ );

    weight_ = currentWeigher_.zeroedCalibrCounts;

    if (( !largeMotion() ) && ( !smallMotion() ))
        if ( autoZero() )
        {
            PRINTF("auto zero on lock up\r\n");
            currentZero += currentWeigher_.zeroedCalibrCounts;
            weight_ = 0;
        }

    if ( errorState_ )
        currentWeigher_.status |= WEIGHER_EEP_READ_FAILED;
}


/******************************************************************************/
/*!   \fn static void detectMotion( unsigned long counts )

      \brief
        This function
        limit.      
   
      \author
          Aaron Swift
*******************************************************************************/
static void detectMotion( unsigned long counts )
{
    currentWeigher_.calibratedCounts = calibrateCounts( counts );
    currentWeigher_.zeroedCalibrCounts  = currentWeigher_.calibratedCounts - currentZero;
    currentWeigher_.slope = calculateSlope( &currentWeigher_, &prevWeigher_ );
    
    #if 1
    static unsigned short printfThrottle = 1;
    if(!printfThrottle--) {
       //PRINTF("detectMotion(): zCalCnts: %d \r\n", currentWeigher_.zeroedCalibrCounts);
       printfThrottle = 20;      
    }
    #endif

    if( state_ != DETECTMOTION_ )
        state_ = DETECTMOTION_;

    if( largeMotion() )
    {
        /* there is large motion, so update the weight. */ 
        weight_ = currentWeigher_.zeroedCalibrCounts;

        /* the weight could also be updated with the old count, but was done in 
           this manner for this scale  in order to reach a stable weight reading faster */
        currentWeigher_.status |= LARGE_MOTION;

    }
    else
    {
        if ( smallMotion() )
        {
            /* There is small motion, so update the weight */ 
            weight_ = currentWeigher_.zeroedCalibrCounts;
            currentWeigher_.status |= SMALL_MOTION;
        }
        else
        {
            if ( autoZero() )
            {
                currentZero += currentWeigher_.zeroedCalibrCounts;
                currentWeigher_.status |= NEW_ZERO;
                weight_ = 0;                               
            }
            else
            {
                /* check for negative weight */ 
                if ( weight_ < 0 && labs( weight_ ) > (OneHalfGraduation) )
                    currentWeigher_.status |= NEGATIVE_GROSS_WEIGHT;
            }

        }
        /* check for weight within quarter graduation of zero */ 
        if( labs( weight_ ) <= OneQuarterGraduation ) {
            currentWeigher_.status |= WITHIN_QUARTER_GRADUATION_OF_ZERO;            
            /* we are no longer in warmup! */
            if( ( currentWeigher_.status & SCALE_WARMUP) == SCALE_WARMUP ) {
                currentWeigher_.status &= ~SCALE_WARMUP;
            }                
            
        }
    }

    /* check for over gross weight */
    if ( weight_ > config.max_weight )
        currentWeigher_.status |= OVER_GROSS_WEIGHT_LIMIT;

    /* check for zero within the maintenance zone.*/
    if( !validZero() ) {
        currentWeigher_.status |= ZERO_OUT_OF_MAINTENANCE_ZONE;
        //PRINTFThrottle(100,"detectMotion(): ZERO_OUT_OF_MAINTENANCE_ZONE\r\n");
    }
}

/******************************************************************************/
/*!   \fn static BOOL largeMotion()
      \brief
        This function
        limit.      
   
      \author
          Aaron Swift
*******************************************************************************/
static bool largeMotion()
{
    /* not sure how this will work? unless counter never gets dec for the first time?? */
    static short half_cycle_counter = 36;       /* default config value */

    /* Detect a gradual change in the same direction  over a period of time. */
    if( prevWeigher_.slope == currentWeigher_.slope )
        --half_cycle_counter;    
    else
        half_cycle_counter = config.large_motion_count;
    

    /* once in large motion, stay there until the weights stabilize. */
    if(( prevWeigher_.status & LARGE_MOTION ) == LARGE_MOTION )
    {
        if( labs( weight_ - currentWeigher_.zeroedCalibrCounts ) > config.small_motion_limit )         
            /* there is large motion. */
            return( true );        
        else            
            /* there is no large motion. */
            return( false );
        
    }
    else
    {
        /* if there is a gradual change in the same direction or the 
           difference between the new count and the Target is        
           greater than the large motion limit.*/
        if( ( half_cycle_counter <= 0 ) || ( labs( weight_ - 
            currentWeigher_.zeroedCalibrCounts ) > config.large_motion_limit ) )        
            /* there is large motion. */
            return(true);
        else
            /* there is no large motion */ 
            return( false );      
    }
}

/******************************************************************************/
/*!   \fn static BOOL smallMotion()
      \brief
        This function
        limit.      
   
      \author
          Aaron Swift
*******************************************************************************/
static bool smallMotion()
{
    static int timer = 12;      /* default configuration value */
    static bool motion = false;

     /* if the absolute difference between the weight and the new counts 
        is greater than the small motion limit or the scale was          
        previously in large motion. */

     /* the test for previously in large motion requires that the scale  
        always produce at least one small motion reading when going out  
        of motion. This also allows the weight value to be averaged with 
        the new counts for a more accurate weight when out of motion 
        condition is established.*/
         
    if( ( prevWeigher_.status & LARGE_MOTION ) == LARGE_MOTION ) {
        timer = ( weigherMode_ == SCALE_PREPACK_MODE ) ?  config.prepack_motion_count : config.small_motion_count;	
        motion = true;
    }	
	 
    if( motion ) {
      
        if( ( labs( weight_ - currentWeigher_.zeroedCalibrCounts ) > config.small_motion_limit ) || 
        ( ( prevWeigher_.status & LARGE_MOTION ) == LARGE_MOTION ) ) 
            timer = ( weigherMode_ == SCALE_PREPACK_MODE ) ? 
            config.prepack_motion_count : config.small_motion_count;	
        		
        if( --timer > 0 ) {
            return(true);
        } else {
            motion = false;
            timer = config.no_motion_count;
            return(false);
        }
     } else {
        /* movement greater than the small_motion_limit can be tolerated for
           no_motion_count long before "small motion" is true */
        if( labs( weight_ - currentWeigher_.zeroedCalibrCounts ) <= config.small_motion_limit)
            timer = config.no_motion_count;

        if( --timer > 0 ) {
            return(false);
        } else {
            motion = true;
            timer = ( weigherMode_ == SCALE_PREPACK_MODE ) ? 
            config.prepack_motion_count : config.small_motion_count;	
            return(true);
        }
    }  
}

/******************************************************************************/
/*!   \fn void handleAdConversion( void )

      \brief
        This function handles A/D readings.
   
      \author
          Aaron Swift
*******************************************************************************/
static void handleAdConversion( AD_Counts *pCounts )
{
    readFilteredWeigher( pCounts );
}

/******************************************************************************/
/*!   \fn void readFilteredWeigher( void )

      \brief
        This function applys filter to A/D readings.
   
      \author
          Aaron Swift
*******************************************************************************/
static void readFilteredWeigher( AD_Counts *pCounts )
{
    
    filter( pCounts );
    
    pCounts->avgFilterCounts = ( long )boxcarFilterAppend( &filteredReadings, pCounts->firCounts );
    
  
    //filter( pCounts );
    
    //pCounts->avgFilterCounts = ( long )boxcarFilterAppend( &filteredReadings, pCounts->firCounts );    
    //pCounts->avgFilterCounts = pCounts->rawCounts;

    /*
    // raw, filtered, avg, non-zero, weight, large motion, small motion
    bool largeMotion = false;
    bool smallMotion = false;
    bool qGrad = false;
    bool negGross = false;
    bool nZero = false;
    bool sWarm = false;
    bool vM = false;
    
    if((currentWeigher_.status & LARGE_MOTION) == LARGE_MOTION)
    {
        largeMotion = true;
    }
        
    if((currentWeigher_.status & SMALL_MOTION) == SMALL_MOTION)
    {
        smallMotion = true;
    }
        
    if((currentWeigher_.status & NEGATIVE_GROSS_WEIGHT) == NEGATIVE_GROSS_WEIGHT)
    {
        negGross = true;
    }
    
    if((currentWeigher_.status & WITHIN_QUARTER_GRADUATION_OF_ZERO) == WITHIN_QUARTER_GRADUATION_OF_ZERO)
    {
        qGrad = true;
    }
    
    if((currentWeigher_.status & NEW_ZERO) == NEW_ZERO)
    {
        nZero = true;
    }
    
    if((currentWeigher_.status & SCALE_WARMUP ) == SCALE_WARMUP )
    {
        sWarm = true;
    }
     
    
    if(weigherDebugOnce == false)
    {
        weigherDebugOnce = true;
        PRINTF("Time, Raw, Avg, Zero, Weight,  LM, SM, quarter, neg, newZero, warmup");
    }
    
    PRINTF("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d\r\n", counts_.rawCounts, counts_.avgFilterCounts, currentWeigher_.zeroedCalibrCounts, labs(weight_), vM, largeMotion, smallMotion, qGrad, negGross, nZero, sWarm );
    */
}

/******************************************************************************/
/*!   \fn static void filter(AD_Counts *pCounts)

      \brief
        This function filters low level load cell counts using variable gain filtering.

      \author
          Randy Blankley
*******************************************************************************/
static void filter( AD_Counts *pCounts )
{
    // load cell standard deviation = 8.6601
    static const double stddev_var = 75.0; // stddev^2          // load cell standard deviation variance
    static const double log_stddev = 2.158726; // log(stddev)   // natural log of standard deviation

    /*
     * This filter is a modified Kalman filter. It uses variable gain to determine how reactive
     * it should be to each successive count. We measure how much deviation is in the load cell
     * output to determine how much gain the filter should have. Higher deviation equals more
     * uncertainty and higher gain.... lower deviation equals less uncertainty and lower gain.
     *
     * At a high level there are three steps to this filter:
     * 1) Adjust our current state using the latest measurement (counts) and gain, this is our
     *    filtered value
     * 2) Make a prediction about the next measurement
     * 3) Compute how much deviation is in the system (uncertainty) for use in calculating gain
     *
     * WHAT WE ARE TRYING TO ACCOMPLISH
     *
     * Load cells have a lot of oscillation... that is they will read above true value then below
     * true value then above and then below and so on and so forth. Below is a diagram of load cell
     * output over time:
     *
     * --------
     * *       \                       UPPER LINE IS MAX
     *          ------------
     *         *            \                   ----
     *     *                 --------          /   *\
     *                 *   *         \        - *--  \          ----
     *                                \      /  /  \ *\        / *  \      ----
     *             *                *  ------ *-    \  \      -* --  \    / *  \
     *                         *         *  * /      \  \    /  /  \  \  /  --  \      ---
     *         COUNTS                      ---        \* \  /  /    \ *--  /  \  \    /  *
     *                   *       *    *   /            -  --  /      \  * /    \  \  /  --
     *                             -------              \* * /        ----      \ *--  /
     *                       *    /                      ----                    \  * /
     *       *           ---------                                                ----
     *                  /
     *           *   * /
     *           ------                LOWER LINE IS MIN
     *   *      /
     * ---------
     *
     * When I place weight on the load cell it will oscillate up and down with huge swings and
     * finally converge after some time has passed. After convergence a low frequency ripple will
     * occur where counts move slowly up and down over time. Our goal is to have high gain at the
     * beginning to average out the swings and to have low gain at the end as to not react to
     * ripple.
     *
     * HOW WE ARE FILTERING
     *
     * The first thing this filter does is take an averaged value of the oscillation. This is the
     * purpose of hist0, hist1, and hist2 filters. Each successive filter reduces oscillation by
     * about 90% or so. After three such iterations we have a pretty good estimate value of true.
     *
     * We also measure the min and max of this this output. By looking at the range of these two
     * values we have a reasonable idea of how much oscillation is in our system. Higher range is
     * more oscillation and lower range is less oscillation.
     *
     * This is part of step 1. We take the estimate value, plug it into our filter, and out comes
     * our result. When the filter is less uncertain (i.e. stable) it wont move very much, when the
     * filter is more uncertain (i.e. unstable) it will move a lot.
     *
     * In step 2 we make a prediction of the next value. For load cells this is easy... if we are
     * in motion we predict the value we have otherwise we dont predict anything because the filter
     * should not be moving (i.e. our prediction is our current value). We can determine if we are
     * in motion by looking at our min/max range. When range is more than a few standard deviations
     * or so we are moving. When range is tiny then we are stable.
     *
     * For step 3 we must determine the uncertainty of our system. This is the hardest part of our
     * filter logic and can border on art more than science.
     *
     * Kalman uncertainty is a measurement of how much trust we have in our current filter state.
     * When uncertainty is high we are saying that our filtered value is changing a lot over time;
     * conversely, when uncertainty is low we are saying that our filtered value should remain
     * more static over time.
     *
     * Uncertainty is based off load cell standard deviation and variance. When uncertainty is
     * equal to variance we get about a 50% gain factor. A little uncertainty goes a long way...
     *
     * Kalman filters assume stability over time (which is true for load cells as well) so they
     * remove uncertainty over time. The amount of uncertainty that gets removed is based on our
     * current gain factor (which is based on uncertainty). So this means when uncertainty is high
     * it decays fast and when uncertainty is low it will decay slow.
     *
     * So how do we calculate uncertainty? It needs to be a measurement of how much error we
     * estimate is present in our current state. Also, when the load cell is moving fast (i.e.
     * stabalizing) there should be more error.
     *
     * We determine uncertainty by looking at how much error is present between our current filter
     * value and our estimate value. The estimate value moves around a lot but should generally be
     * within a few deviations of our filter value. When its not within a few devaiations then
     * something is going on and we become more uncertain.
     *
     * Actually we look at two things. If there is a little error but we have motion (i.e. min/max
     * range is high) then we add uncertainty into the system. The other thing we look for is simply
     * just a lot of error. If we detect a lot of error (more than a few deviations) we also will
     * add uncertainty.
     *
     * The last thing we do is add a touch of uncertainty due to process noise. No
     * algorithm/process is perfect so we always add a little uncertainty in. This allows the filter
     * to move slowly over time to adjust for A/D drift.
     *
     * NOTES AND COMMENTS
     *
     * Why not just use a boxcar filter? Boxcar filters work reasonably well when we are stabalizing
     * but not so well for the ripple. You would need a really large filter to get rid of ripple but
     * that would introduce a ton of lag.
     *
     * If you need to tune the performance of this filter there are three knobs you can turn:
     * 1) Predict Next State
     *    This controls how reactive the filter is to movement. Its great when we are first
     *    stabalizing but not so much later on because its not very accurate. If you need to react to
     *    huge swings you could lower the rlog value it uses to do this.
     * 2) Estimate Uncertainty
     *    Bigger uncertainty number equals more movement to the next count. A little really goes a
     *    long way too. There is a balancing act here between too little movement during stabalization
     *    and too much movement during ripple. By fixing one you may be breaking the other. Be very
     *    careful in this area!
     * 3) Process Noise
     *    This adjusts how well we cope with ripple and drift over time. Higher values will react
     *    faster and produce ripple. Lower values react slower and may not allow the A/D to drift over
     *    time. Again there is a balance act here but not so severe as previous comment.
     */

    const double c = pCounts->rawCounts;

    // initialize filter(s)
    if ( !init )
    {
        boxcarFilterInitF( &hist0, 3 );
        boxcarFilterInitF( &hist1, 3 );
        boxcarFilterInitF( &hist2, 3 );

        boxcarFilterInitF( &min, 6 );
        boxcarFilterInitF( &max, 6 );

        varGainFilterInit( &filt, c, 0.5 );
        uncertainty = stddev_var;

        init = true;
    }

    // calculate gain (K)
    varGainFilterSetGain( &filt, uncertainty / (uncertainty + stddev_var) );

    // update current state
    const double h0 = boxcarFilterAppendF( &hist0, c );
    const double h1 = boxcarFilterAppendF( &hist1, h0 );
    const double h2 = boxcarFilterAppendF( &hist2, h1 );

    const double hmin = boxcarFilterAppendF( &min, hist2.min );
    const double hmax = boxcarFilterAppendF( &max, hist2.max );

    const double result = varGainFilterAppend( &filt, h2 );

    // predict next state
    const double r = hmax - hmin;
    const double rlog = log( r / 2.0 ) / log_stddev;

    if ( 3.0 <= rlog )
    {
        // when min/max range is high we are in motion and should make prediction
        filt.value = h2;
    }

    // estimate uncertainty (p)
    const double err = fabs( h2 - result );
    const double errlog = log( err ) / log_stddev;

    uncertainty *= ( 1.0 - filt.gain );

    // 2.51 = ~3*stddev standard deviations
    // 2.0  = stddev standard deviations (variance)
    // 1.83 = ~6 standard deviations
    // 1.51 = ~3 standard deviations
    // 1.32 = ~2 standard deviations
    // 1.0  = 1 standard deviation
    if ((( 1.32 <= rlog ) && ( 2.0 <= errlog )) || ( 2.51 <= errlog ))
    {
        // add uncertainty into system when:
        // 1) motion present and a little error
        // 2) a lot of error
        uncertainty += pow( errlog - 1.0, 2.7182818 );
    }

    // update uncertainty with process noise (q)
    const double q = 0.0001;

    uncertainty += q;

    pCounts->firCounts = (unsigned long) result;  //TFink 5/2/24 added cast to unsigned long to eliminate warning. 
}

/******************************************************************************/
/*!   \fn static void monitorWeigher( AD_Counts *pCounts  )

      \brief
        This function handles the counts based on the weigher state.
   
      \author
          Aaron Swift
*******************************************************************************/
static void monitorWeigher( AD_Counts *pCounts )
{
    static unsigned long startCounter_ = 0;
    //PRINTF("monitorWeigher():\r\n");

    #ifndef WEIGHER_DEBUG   
    static WeigherStates previous_state;
        
    if( state_ != previous_state ) {
        //PRINTF("WeigherState: %d, \r\n",state_ );
        previous_state = state_;
    }
    #endif 
    
    /* check state of service switch */
    if( ( !GPIO_ReadPinInput( SERVICE_SWITCH_GPIO, SERVICE_SWITCH_PIN ) ) && !debounce_ ) {
        serviceSwitchPressed();
    } 
        
    #if 0
    // raw, filtered, avg, non-zero, weight, large motion, small motion
    bool largeMotion = false;
    bool smallMotion = false;
    bool qGrad = false;
    bool negGross = false;
    bool nZero = false;
    bool sWarm = false;
    bool vM = false;
    
    
     
    
    if(weigherDebugOnce == false)
    {
        weigherDebugOnce = true;
        PRINTF("Time, Raw, Avg, Zero, Weight, WeightVM, LM, SM, quarter, neg, newZero, warmup");
    }
    
    //PRINTF("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d\r\n", pCounts->rawCounts, pCounts->avgFilterCounts, currentZero, labs(weight_), vM, largeMotion, smallMotion, qGrad, negGross, nZero, sWarm );
    PRINTF("%d, %d, %d,", pCounts->rawCounts, pCounts->avgFilterCounts, currentZero);
    #endif 
    
    /* handle the counts based on the weigher state */
    switch( state_ ) {
        case POWERUP_:
            adReset( pCounts->avgFilterCounts );
            break;
        case PRE_INIT_ZERO: 
        case INIT_ZERO_:
            initializeZero( pCounts->avgFilterCounts );
            break;
        case LOCKUP_:
            lockUp( pCounts->avgFilterCounts );
            break;
        case DETECTMOTION_:      
            detectMotion( pCounts->avgFilterCounts );        
            break;
        case ADRESET_:
            adReset( pCounts->avgFilterCounts );
            break;
        case REZERO_:
            rezeroWeigher( pCounts->avgFilterCounts );
            break;
        default:
            break;
    }
    
    #if 0
    if((currentWeigher_.status & LARGE_MOTION) == LARGE_MOTION)
    {
        largeMotion = true;
    }
        
    if((currentWeigher_.status & SMALL_MOTION) == SMALL_MOTION)
    {
        smallMotion = true;
    }
        
    if((currentWeigher_.status & NEGATIVE_GROSS_WEIGHT) == NEGATIVE_GROSS_WEIGHT)
    {
        negGross = true;
    }
    
    if((currentWeigher_.status & WITHIN_QUARTER_GRADUATION_OF_ZERO) == WITHIN_QUARTER_GRADUATION_OF_ZERO)
    {
        qGrad = true;
    }
    
    if((currentWeigher_.status & NEW_ZERO) == NEW_ZERO)
    {
        nZero = true;
    }
    
    if((currentWeigher_.status & SCALE_WARMUP ) == SCALE_WARMUP )
    {
        sWarm = true;
    }
    
    
    PRINTF("%d, %d, %d, %d, %d, %d, %d, %d\r\n", labs(weight_), vM, largeMotion, smallMotion, qGrad, negGross, nZero, sWarm);
    #endif
    
    
    /* ProcessCounts clears "currentWeigher", so update after Process Counts */
    currentWeigher_.avgFilterCounts = pCounts->avgFilterCounts;
    currentWeigher_.rawCounts = pCounts->rawCounts;
    
    switch( weigherMode_ ) {
        case SCALE_PREPACK_MODE:
        case SCALE_MANUAL_MODE:
            if( ( ( currentWeigher_.status != prevWeigher_.status ) && ( statusMsgFilter(&prevWeigher_, &currentWeigher_ ) == true ) ) || 
                ( serviceSwitch_ == true ) )
            {
                WgStatus msg;

                if( serviceSwitch_ == true ) {
                    currentWeigher_.status |= WEIGHER_KEY_PRESSED;
                }
                
                msg.status                  = currentWeigher_.status;	
                msg.avgFilterCounts         = (long)currentWeigher_.avgFilterCounts;
                msg.nonZeroCalibratedCounts = (long)currentWeigher_.calibratedCounts;	
                msg.zeroedCalibratedCounts  = (long)labs( weight_ );     
                msg.rawCounts               = (unsigned long)currentWeigher_.rawCounts;
                
                if(config.value_max_on_off == true) {            
                   ValueMax *pVM = getPtrToValueMax();
                   
                   msg.acclAvgXCounts          = pVM->accel_x_avg_raw_reading;
                   msg.acclAvgYCounts          = pVM->accel_y_avg_raw_reading;
                   msg.acclAvgZCounts          = pVM->accel_z_avg_raw_reading;
                   msg.tiltX                   = (short)(pVM->x_degrees*100);
                   msg.tiltY                   = (short)(pVM->y_degrees*100);
                } else {
                   
                   msg.acclAvgXCounts          = 0;
                   msg.acclAvgYCounts          = 0;
                   msg.acclAvgZCounts          = 0;
                   msg.tiltX                   = 0;  
                   msg.tiltY                   = 0;  
                }
                
                   #if 1
                   //if(config.value_max_on_off == false)
                    //PRINTFThrottle(8,"\r\nmonitorWeigher(): This Model does not support Value Max\r\n");
                
                   ValueMax *pVM = getPtrToValueMax();
                   //PRINTF("\r\nMANUAL_MODE Status:\t%4x\r\n",msg.status);
                   //PRINTF("Raw Weigher Counts\t%d\r\n",currentWeigher_.rawCounts);
                   //PRINTF("Avg Filter Counts\t%d\r\n",msg.avgFilterCounts);
                   //PRINTF("Non Zero Cal Counts\t%d\r\n",msg.nonZeroCalibratedCounts);
                   //PRINTF("Zeroed Cal Counts\t\t\t%d\r\n",msg.zeroedCalibratedCounts);
                   //PRINTF("Accl Avg X Counts\t%d\r\n",msg.acclAvgXCounts);
                   //PRINTF("Accl Avg Y Counts\t%d\r\n",msg.acclAvgYCounts);
                   //PRINTF("Accl Avg Z Counts\t%d\r\n",msg.acclAvgZCounts);
                   //PRINTF("Tilt X\t\t\t%.3f \r\n",pVM->x_degrees);  
                   //PRINTF("Tilt Y\t\t\t%.3f \r\n\r\n\r\n",pVM->y_degrees);  
                   #endif
                
                /* update our status if changed */
                if( currentWeigher_.status != prevWeigher_.status ) {
                    /* do not spam backend with status updates quarter zero, zero 
                    if( ( currentWeigher_.status != WITHIN_QUARTER_GRADUATION_OF_ZERO ) && 
                        ( prevWeigher_.status != ( WITHIN_QUARTER_GRADUATION_OF_ZERO | NEW_ZERO ) ) ) { */
                          sendWgStatus( &msg );
                          memcpy( &prevWeigher_, &currentWeigher_, sizeof( WeightStateType ) );
                    /* } */
                }    
                
                if( serviceSwitch_ == true ) {
                    clearServiceSwitch();
                    currentWeigher_.status &= ~WEIGHER_KEY_PRESSED;	                        
                }
            }
            break;
        case SCALE_WEIGHT_TRACKING_MODE:            
            /* one message every 250mS.*/
            if( !initTracking_ ) {
                initTracking_ = true;
                startCounter_ = xTaskGetTickCount();                
            } else {
                unsigned long count = xTaskGetTickCount();                
                if( count - startCounter_ >= TRACKING_COUNT ) {                    
                    startCounter_ = xTaskGetTickCount();
                    WgStatus msg;
                    
                    msg.status                  = currentWeigher_.status;	
                    msg.avgFilterCounts         = (long)currentWeigher_.avgFilterCounts;
                    msg.nonZeroCalibratedCounts = (long)currentWeigher_.calibratedCounts;	
                    msg.zeroedCalibratedCounts  = (long)labs( weight_ );   
                    msg.rawCounts               = (unsigned long)currentWeigher_.rawCounts;
                    
                if(config.value_max_on_off == true) {             
                   ValueMax *pVM = getPtrToValueMax();
                   
                   msg.acclAvgXCounts          = pVM->accel_x_avg_raw_reading;
                   msg.acclAvgYCounts          = pVM->accel_y_avg_raw_reading;
                   msg.acclAvgZCounts          = pVM->accel_z_avg_raw_reading;
                   msg.tiltX                   = (short)(pVM->x_degrees*100);
                   msg.tiltY                   = (short)(pVM->y_degrees*100);
                   
                } else {
                   
                   msg.acclAvgXCounts          = 0;
                   msg.acclAvgYCounts          = 0;
                   msg.acclAvgZCounts          = 0;
                   msg.tiltX                   = 0;  
                   msg.tiltY                   = 0;  
                }
                
                   #if 1
                   //if(config.value_max_on_off == false)
                    //PRINTFThrottle(8,"\r\nmonitorWeigher(): This Model does not support Value Max\r\n");
                
                   ValueMax *pVM = getPtrToValueMax();
                   //PRINTF("\r\nWT_TRACK: Status: \t%4x\r\n",msg.status);
                   //PRINTF("Raw Weigher Counts\t%d\r\n",currentWeigher_.rawCounts);
                   //PRINTF("Avg Filter Counts\t%d\r\n",msg.avgFilterCounts);
                   //PRINTF("Non Zero Cal Counts\t%d\r\n",msg.nonZeroCalibratedCounts);
                   //PRINTF("Zeroed Cal Counts\t\t\t%d\r\n",msg.zeroedCalibratedCounts);
                   //PRINTF("Accl Avg X Counts\t%d\r\n",msg.acclAvgXCounts);
                   //PRINTF("Accl Avg Y Counts\t%d\r\n",msg.acclAvgYCounts);
                   //PRINTF("Accl Avg Z Counts\t%d\r\n",msg.acclAvgZCounts);
                   //PRINTF("Tilt X\t\t\t%.3f\r\n",pVM->x_degrees);  
                   //PRINTF("Tilt Y\t\t\t%.3f\r\n\r\n\r\n",pVM->y_degrees);  
                   #endif
                                          
                    sendWgStatus( &msg ) ;
                    memcpy( &prevWeigher_, &currentWeigher_, sizeof( WeightStateType ) );                
                }
            }
            break;
        case SCALE_DUMB_MODE:
            /* the Weigher Calibration Switch also serves as the service
            switch. In HLX, a USB Thumb Drive is used, so Scale Dumb Mode doesn'
            return status when the  WEIGHER KEY is pressed */
            if( ( ( currentWeigher_.status != prevWeigher_.status ) && ( statusMsgFilter( &prevWeigher_, &currentWeigher_ ) == true ) ) ||
                ( ( currentWeigher_.status & WEIGHER_KEY_PRESSED ) != ( prevWeigher_.status & WEIGHER_KEY_PRESSED ) ) ) 
            {
				  
                WgStatus msg;

                msg.status                  = currentWeigher_.status;
                msg.avgFilterCounts         = (long)currentWeigher_.avgFilterCounts;
                msg.nonZeroCalibratedCounts = (long)currentWeigher_.calibratedCounts;	
                msg.zeroedCalibratedCounts  = (long)labs( weight_ );   
                msg.rawCounts               = (unsigned long)currentWeigher_.rawCounts;
                
                if(config.value_max_on_off == true) {             
                   ValueMax *pVM = getPtrToValueMax();
                   
                   msg.acclAvgXCounts          = pVM->accel_x_avg_raw_reading;
                   msg.acclAvgYCounts          = pVM->accel_y_avg_raw_reading;
                   msg.acclAvgZCounts          = pVM->accel_z_avg_raw_reading;
                   msg.tiltX                   = (short)(pVM->x_degrees*100);
                   msg.tiltY                   = (short)(pVM->y_degrees*100);
                   
                } else {
                   
                   msg.acclAvgXCounts          = 0;
                   msg.acclAvgYCounts          = 0;
                   msg.acclAvgZCounts          = 0;
                   msg.tiltX                   = 0;  
                   msg.tiltY                   = 0;  
                }
                
                   #if 1
                   if(config.value_max_on_off == false)
                    PRINTFThrottle(8,"\r\nmonitorWeigher(): This Model does not support Value Max\r\n");
                
                   ValueMax *pVM = getPtrToValueMax();
                   PRINTF("\r\nDUMB_MODE: Status:\t%4x\r\n",msg.status);
                   //PRINTF("Raw Weigher Counts\t%d\r\n",currentWeigher_.rawCounts);
                   PRINTF("Avg Filter Counts\t%d\r\n",msg.avgFilterCounts);
                   PRINTF("Non Zero Cal Counts\t%d\r\n",msg.nonZeroCalibratedCounts);
                   PRINTF("Zeroed Cal Counts\t\t\t%d\r\n",msg.zeroedCalibratedCounts);
                   PRINTF("Accl Avg X Counts\t%d\r\n",msg.acclAvgXCounts);
                   PRINTF("Accl Avg Y Counts\t%d\r\n",msg.acclAvgYCounts);
                   PRINTF("Accl Avg Z Counts\t%d\r\n",msg.acclAvgZCounts);
                   PRINTF("Tilt X\t\t\t%.3f\r\n",pVM->x_degrees);  
                   PRINTF("Tilt Y\t\t\t%.3f\r\n\r\n\r\n",pVM->y_degrees);  
                   #endif
                
                sendWgStatus( &msg );
                memcpy( &prevWeigher_, &currentWeigher_, sizeof( WeightStateType ) );
            }
            break;
        default:
            break;
    }
    
    /*update the weigher history */
    updateHistory(&currentWeigher_, &prevWeigher_);
}

/******************************************************************************/
/*!   \fn static void adReset( unsigned long counts )

      \brief
       
   
      \author
          Aaron Swift
*******************************************************************************/
static void adReset( unsigned long counts )
{
    currentWeigher_.status |= SCALE_WARMUP;
    currentWeigher_.zeroedCalibrCounts = currentWeigher_.calibratedCounts = counts;
    currentWeigher_.slope = calculateSlope( &currentWeigher_, &prevWeigher_ );

    if ( state_ != ADRESET_ ) {
        state_ = ADRESET_;
        setZeroTracking();
    } else {
        
        #ifdef TFinkStartCS5530woApp
        /* Want Peripheral Processor to run wo App */
        if( isAccelerometerInitialized() || config.value_max_on_off == false  ) {
            state_ = PRE_INIT_ZERO; 
            PRINTF("adReset(): Jumping to wgh state PRE_INIT_ZERO\r\n");
        }     
        #else
        /* jump to initializeZero(); once we've connected with Host controller
           and have ran Accl init. Just incase we need to report errors. */
        if( hostConnected_ == true && (isAccelerometerInitialized() || config.value_max_on_off == false ) ) {
            state_ = PRE_INIT_ZERO;        	
        }
        #endif

        weight_ = currentWeigher_.calibratedCounts; 
    }
}

/******************************************************************************/
/*!   \fn static bool statusMsgFilter( WeightStateType *previous, WeightStateType *current )

      \brief
        Function to control sending of status message based on what status bits are set
		returns - true to send 
		returns - false to not send
   
      \author
          Carlos Guzman
*******************************************************************************/
static bool statusMsgFilter( WeightStateType *previous, WeightStateType *current )
{
    bool send = true;
  
   /* TFink 5/17/24: Per Chris King, Application guys want this message to be sent
      continuously and they will filter it out. */
    
   #if 0
    /*
    * 	If ZERO_OUT_OF_MAINTENANCE_ZONE is set, don't bother sending any other 
    *   new statuses until this one is corrected. This is to handle condition when
    *	Scale Platter is removed and small motion bits are getting set and cleared
    *   constantly, causing multiple error prompts to continously be displayed.
    */
    
   
    if( ( ( current->status & ZERO_OUT_OF_MAINTENANCE_ZONE ) == ZERO_OUT_OF_MAINTENANCE_ZONE ) &&
        ( ( previous->status & ZERO_OUT_OF_MAINTENANCE_ZONE ) == ZERO_OUT_OF_MAINTENANCE_ZONE ) ) 
    {
        send = false;
    }
    */

    /*
    *	Add more cases...
    */
   #endif
      
    return send;
}

/******************************************************************************/
/*!   \fn static void updateHistory( WeightStateType *current, WeightStateType *previous )

      \brief
        This function updates the previous weigher counts.       
   
      \author
          Aaron Swift
*******************************************************************************/
static void updateHistory( WeightStateType *current, WeightStateType *previous )
{
	 
    previous->calibratedCounts      = current->calibratedCounts;
    previous->zeroedCalibrCounts    = current->zeroedCalibrCounts;
    previous->slope                 = current->slope;
    previous->status                = current->status;
    previous->avgFilterCounts       = current->avgFilterCounts;
    previous->rawCounts             = current->rawCounts;
    
    if( ( current->status & SCALE_WARMUP ) != SCALE_WARMUP ) {
        /* initialize the new weigher state. */
        current->calibratedCounts      = 0;
        current->zeroedCalibrCounts    = 0;
        current->slope                 = 0;
        current->status                = 0;
        current->avgFilterCounts       = 0;
        current->rawCounts             = 0;
    }
	
    /*
    * 	If WEIGHER_VM_MAX_TILT_REACHED bit is set, dont clear it
    * 	here, keep it set and wait for valueMaxHandleAccelMeasurement() to 
    * 	clear it when tilt is < than 3 degrees.
    */
    if( ( previous->status & WEIGHER_VM_MAX_TILT_REACHED ) == WEIGHER_VM_MAX_TILT_REACHED ) {
        current->status |= WEIGHER_VM_MAX_TILT_REACHED;
    }

    /*
    *	Dont clear - WEIGHER_VM_ACCL_COMM_FAILURE, WEIGHER_VM_EEP_CHECKSUM_FAILURE and
    *	WEIGHER_VM_EEP_COMM_FAILURE - Keep them set until operator resets scale.
    */
    if( ( previous->status & WEIGHER_VM_ACCL_COMM_FAILURE ) == WEIGHER_VM_ACCL_COMM_FAILURE ) {
        current->status |= WEIGHER_VM_ACCL_COMM_FAILURE;
    } 

    if( ( previous->status & WEIGHER_VM_EEP_CHECKSUM_FAILURE) == WEIGHER_VM_EEP_CHECKSUM_FAILURE ) {
        current->status |= WEIGHER_VM_EEP_CHECKSUM_FAILURE;
    }

    if( ( previous->status & WEIGHER_VM_EEP_COMM_FAILURE ) == WEIGHER_VM_EEP_COMM_FAILURE ) {
        current->status |= WEIGHER_VM_EEP_COMM_FAILURE;
    }
    /* - */
}

/********** Getter/Setter Functions ***************************/
bool isValueMaxOn(void)
{  /* Getting from the Weigher Config, which is on the main board configuration EEP - as opposed to the Acceleromter PCBA EEP */
   return(config.value_max_on_off);
}

bool isHostConnected(void)
{
   #ifdef TFinkStartCS5530woApp
   return true;
   #else
   return hostConnected_;
   #endif
}

void setWeigherStatusBit(unsigned short wgStatusBit)
{
  currentWeigher_.status |= wgStatusBit;
}


