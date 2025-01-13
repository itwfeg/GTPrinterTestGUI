#include "sensors.h"
#include "lp5521.h"
#include "w25x10cl.h"
#include <stdlib.h>
#include <math.h>
#include "filter.h"
#include "globalPrinter.h"
#include "printHead.h"
#include "printEngine.h"
#include "threadManager.h"
#include "queueManager.h"
#include "serialflash.h"
#include "label.h"
#include "fsl_gpio.h"
#include "fsl_gpt.h"
#include "pin_mux.h"
#include "fsl_pit.h"
#include "fsl_dmamux.h"
#include "fsl_adc_etc.h"
#include "fsl_edma.h"
#include "fsl_dmamux.h"
#include "fsl_xbara.h"
#include "fsl_qtmr.h"
#include "fsl_pwm.h"
#include "fsl_debug_console.h"
#include "takeupMotor.h"
#include "virtualScope.h"


#define TFINKMEDIABUFFERSIZE 16   /* Must be 2^x where x is an integer */
#define TFINKMEDIABUFFERPWR 4

static int TFinkMediaBuffer[TFINKMEDIABUFFERSIZE] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};  //TF4
static unsigned short TFinkMediaBufferPtr = 0;
static int TFinkMediaBufferSum = 0;
static int TFinkMediaSensorAvg = 0;


static bool suspend_                    = false;
static TaskHandle_t pHandle_            = NULL;
static ADConfig mode_                   = AD_INIT;
static unsigned int measuredLength_     = 0;
static unsigned int actualLength_       = 0;
static bool motorStopGap_               = false;
static bool motorStopEdge_              = false;
static TimerHandle_t ledOnPwm_          = NULL;
unsigned long lTick                     = 0;
static bool backwindAfterContExpel      = false;

extern PrStatusInfo                     currentStatus;
extern PrStatusInfo                     prevStatus;
extern Pr_Config                        config_;
extern LabelPosition                    *pCurrentLabel;
extern int                              labelAlignment;
extern bool                             mainMotorStopped_;

extern void powerOffStepper( void );
extern bool isLabelTakenTimerStarted( void );
extern void stopLabelTakenTimer( void );
extern void sendTakeLabelError( bool error );

HeadSensors  headSensors;

uint8_t pitTimerLength = 1;

AT_NONCACHEABLE_SECTION_INIT( uint32_t adcMux[9] ) = { SHOOT_SENSOR_CHANNEL, HEAD_UP_CHANNEL, HEAD_DOT_CHANNEL, 
                                                       HEAD_TEMPERATURE_CHANNEL, SHOOT_SENSOR_CHANNEL, 
                                                       TAKEN_SENSOR_CHANNEL, PAPER_TAKEUP_CHANNEL, 
                                                       LOW_STOCK_CHANNEL, HEAD_DETECT_CHANNEL };
volatile ADCManager adcManager;

/*  testing dot measurement */
unsigned short indx_ = 0;

/* track if label is present */
static int ghostM_ = 0;

bool prevHeadUp = false;
bool currentHeadUp = false;

volatile unsigned char chCntr_ = 0;
unsigned char headTemperature = 0;
volatile bool tracking_ = false;

static bool TUCalHeadUpFlag = false;

#define LABEL_TAKEN_DELAY_LENGTH 10500


/******************************************************************************/
/*!   \fn BaseType_t createSensorsTask( ADConfig mode )

      \brief
        This function intializes and creates the sensors thread.
        If the mode is set to AD_MANUAL the sensors task will manually
        intiate A/D conversions and muxing A/D channels. If the mode is set to
        AD_AUTO then no sensors task is created and the trigger for A/D conversions
        and channel muxing is handled through hardware (PDB, ADC, DMA).
      \author
          Aaron Swift
*******************************************************************************/
BaseType_t createSensorsTask(  ADConfig mode  )
{
    BaseType_t result;
    mode_ = mode;
    
    /* clear our manager */
    memset( (void *)&adcManager, 0, sizeof(ADCManager) );
    initManager( (ADCManager *)&adcManager );
    
    /* do not create managment thread if AUTO mode is used */
    if( mode_ != AD_AUTO ) {
               
        /* init sensors */ 
        initializeSensors();
        PRINTF("\r\n\r\ninit sensors\r\n\r\n");
        
        /* create sensors task thread */
        result = xTaskCreate( sensorsTask,  "SensorsTask", configMINIMAL_STACK_SIZE,
                                            NULL, sensors_task_PRIORITY, &pHandle_ );
        /* give the thread manager the handle */
        updateTaskHandle( T_SENSORS );
    } else {
        /* init sensors */ 
        initializeSensors();
        result = pdPASS;
    }
    return result;
}

/******************************************************************************/
/*!   \fn static void sensorsTask( void )

      \brief
        This function is the sensors run thread which manually intiates the 
        A/D conversions. 400uS for all channels to be processed and another 
        channel scan started. 
   
      \author
          Aaron Swift
*******************************************************************************/
static void sensorsTask( void *pvParameters )
{
    
    ( void ) pvParameters;
        
    PRINTF("sensorsTask(): Thread running...\r\n" ); 
    #if 0   
    static int dbCnt_ = 0;
    #endif
    static adc_channel_config_t adcChanCfg;
    
    adcManager.adc0Channel = CHANNEL_TAKEN;
    adcChanCfg.channelNumber = TAKEN_SENSOR_CHANNEL;
    adcChanCfg.enableInterruptOnConversionCompleted = true;    
   
    ADC_SetChannelConfig( ADC2, 0, &adcChanCfg );
               
    static bool adcOnce_ = false;
    while( !suspend_ ) {
                
        if( !adcManager.adcPaused ) {      
            /* is conversion complete? */
            if( adcManager.adcComplete ) {
                            
                if( adcManager.adc0Channel == CHANNEL_TAKEN ) {
                    /* change the channel and start the conversion */
                    adcChanCfg.channelNumber = LOW_STOCK_CHANNEL;
                    adcChanCfg.enableInterruptOnConversionCompleted = true;                    
                    adcManager.adc0Channel = CHANNEL_LOW_STOCK;
                    adcManager.adcComplete = false;    
                    
                    ADC_SetChannelConfig( ADC2, 0, &adcChanCfg );     
                    
                } else if( adcManager.adc0Channel == CHANNEL_LOW_STOCK ) {
                    adcChanCfg.channelNumber = SHOOT_SENSOR_CHANNEL;
                    adcChanCfg.enableInterruptOnConversionCompleted = true;    
                    adcManager.adc0Channel = CHANNEL_SHOOT;
                    adcManager.adcComplete = false;    
                                    
                    ADC_SetChannelConfig( ADC2, 0, &adcChanCfg );
                    
                } else if( adcManager.adc0Channel == CHANNEL_SHOOT ) {
                  
                    if( motorStopGap_ ) {
                        if( adcManager.value[CHANNEL_SHOOT] <= BACKING_PAPER_THRESHOLD ) {
                            powerOffStepper();
                        }
                    }
                    
                    if( motorStopEdge_ ) {
                        if( adcManager.value[CHANNEL_SHOOT] >= LABEL_EDGE_THRESHOLD ) {
                            powerOffStepper();
                        }
                    }
                  
                    adcChanCfg.channelNumber = PAPER_TAKEUP_CHANNEL;
                    adcChanCfg.enableInterruptOnConversionCompleted = true;    
                    adcManager.adc0Channel = CHANNEL_PAPER_TAKEUP;
                    adcManager.adcComplete = false;    
                    
                    ADC_SetChannelConfig( ADC2, 0, &adcChanCfg );    
                    
                } else if( adcManager.adc0Channel == CHANNEL_PAPER_TAKEUP ) {
                    adcChanCfg.channelNumber = HEAD_TEMPERATURE_CHANNEL;
                    adcChanCfg.enableInterruptOnConversionCompleted = true;    
                    adcManager.adc0Channel = CHANNEL_HEAD_TEMP;                
                    adcManager.adcComplete = false;    
                    
                    ADC_SetChannelConfig( ADC2, 0, &adcChanCfg );      
                    
                } else if( adcManager.adc0Channel == CHANNEL_HEAD_TEMP ) {
                    adcChanCfg.channelNumber = HEAD_UP_CHANNEL;
                    adcChanCfg.enableInterruptOnConversionCompleted = true;    
                    adcManager.adc0Channel = CHANNEL_HEAD_STATE;
                    adcManager.adcComplete = false;    
                    
                    ADC_SetChannelConfig( ADC2, 0, &adcChanCfg );   
                    
                } else if( adcManager.adc0Channel == CHANNEL_HEAD_STATE ) {
                    adcChanCfg.channelNumber = TAKEN_SENSOR_CHANNEL;
                    adcChanCfg.enableInterruptOnConversionCompleted = true;    
                    adcManager.adc0Channel = CHANNEL_TAKEN;
                    
                    adcManager.adcComplete = false;    
                    adcOnce_ = true;
                    
                    ADC_SetChannelConfig( ADC2, 0, &adcChanCfg );   
                    
                } else {
                    PRINTF("sensorsTask(): Unknown channel!\r\n" );
                }          
            }        
        }
        /* do not read until after we are through the chain */ 
        if( adcOnce_ ) {
            readADChannels();
            #if 0   
            dbCnt_++;
            if( dbCnt_ == 10000 ) {
                showADCReadings();
                dbCnt_ = 0;
            }
            #endif
            adcOnce_ = false;
        }
        
        /* check to see if any state has changed and notify the host */         
        taskYIELD();
    }
    vTaskSuspend(NULL); 
}

/******************************************************************************/
/*!   \fn static void initManager( ADCManager *pMgr )

      \brief
        This function initializes the ADC manager for manual mode operation. 
        Setup of current working channel and channel list.

      \author
          Aaron Swift
*******************************************************************************/
static void initManager( ADCManager *pMgr )
{
    /* setup current channel */    
    pMgr->adc0Channel = CHANNEL_TAKEN;
     
    /* setup our channel list */
    pMgr->channelList[ 0 ] =  CHANNEL_TAKEN;
    pMgr->channelList[ 1 ] =  CHANNEL_LOW_STOCK;
    pMgr->channelList[ 2 ] =  CHANNEL_SHOOT;
    pMgr->channelList[ 3 ] =  CHANNEL_PAPER_TAKEUP;
    pMgr->channelList[ 4 ] =  CHANNEL_HEAD_TEMP;
    pMgr->channelList[ 5 ] =  CHANNEL_HEAD_STATE;
    pMgr->channelList[ 6 ] =  CHANNEL_HEAD_DOT;
    pMgr->channelList[ 7 ] =  CHANNEL_HEAD_DETECT;
    pMgr->adcComplete      = false;
    pMgr->adcPaused        = false;
}

TaskHandle_t getSensorsHandle(){
    return pHandle_;
}

void closeAdc1( void )
{
    ADC_Deinit( ADC2 );  
}

/******************************************************************************/
/*!   \fn void initializeSensors( void )                                                             
 
      \brief
        This function initializes the lp5521 sensor driver and sets current 
        values for shoot through and label taken sensors.
          
      \author
          Aaron Swift
*******************************************************************************/ 
void initializeSensors( void )
{
    Pr_Config cfg;
    FPMBLC3Checksums            checkSums;
    unsigned int                checksum = 0;
    
    /* initialize sensor driver */
    if( initLp5521() ) {
        PRINTF("initializeSensors(): LP5521 initialized.\r\n" );
    } 
    else 
    {
        PRINTF("initializeSensors(): Failed to initialize LP5521.\r\n" );
    }

    /* read printer configuration for bias settings of shoot through and label
       taken sensors */    
    if( getSerialPrConfiguration( &cfg ) ) 
    {   
        /* check for valid configuration */
        if( getPageChecksums( &checkSums ) ) 
        {
            checksum = calculateChecksum((void *)&cfg, sizeof(Pr_Config));
            if( checksum == checkSums.prConfigSum ) 
            {
                /* set shoot through sensor bias */
                if( !setGapCurrent( cfg.media_sensor_adjustment ) ) {
                    PRINTF("initializeSensors(): failed to set gap bias!\r\n" );
                }
                else
                {
                    PRINTF("Gap sensor current set to %d\r\n", cfg.media_sensor_adjustment);
                }
                    /* set paper take up sensor bias */
                    if( !setPaperTakeupCurrent( cfg.takeup_sensor_drive_current ) ) 
                    {   
                        PRINTF("initializeSensors(): failed to set label taken bias!\r\n" );
                    }
                    else
                    {
                        PRINTF("paper takeup sensor current set to %d\r\n", cfg.takeup_sensor_drive_current);
                        PRINTF("max tension set at %d ADC counts\r\n", cfg.takeup_sensor_max_tension_counts);
                        PRINTF("min tension set at %d ADC counts\r\n", cfg.takeup_sensor_min_tension_counts);
                    }
                /* set low stock sensor bias */
                if( !setLowStockCurrent( /*CC_TWENTY_FIVE_POINT_ZERO*/ 35 ) ) 
                {
                    PRINTF("initializeSensors(): failed to set low stock bias!\r\n" );
                }
                /* set low stock sensor duty cycle DC_90_PRECENT*/
                if( !setLowStockDutyCycle( DC_90_PRECENT ) ) 
                {
                    PRINTF("initializeSensors(): failed to set low stock duty cycle!\r\n" );
                }
            } 
            else 
            {
                PRINTF("initializeSensors(): Printer config unusable, settting sensor defaults\r\n" );                 
            }
        } 
        else 
        {
            PRINTF("initializeSensors(): Failed to read checksums, settting sensor defaults\r\n" );
        }    
    } 
    else 
    {
        PRINTF("initializeSensors(): Failed to read config, settting sensor defaults\r\n" );
    }

    /* connect flextimer 1 output to gpio label taken clock */
    initXbar( mode_ );

    /* start pwm clock to label taken sensor */  
    initLabelTakenLED();    
    
    /* pit timer required if running adc in auto mode */
    if( mode_ == AD_AUTO ) 
    {
        initPitADC();
    }
    
    headTemperature = getPrintheadTemperatureInCelsius(); 
    
    initializeAdcs( mode_ );     
}


/******************************************************************************/
/*!   \fn void initializeAdcs( void )                                                            
 
      \brief
        This function initializes module 0 A/D converter.
          
      \author
          Aaron Swift
*******************************************************************************/ 
void initializeAdcs( ADConfig configuration )
{
    adc_config_t adcConfig;    
    static bool calOnce_ = false;
  
    if( configuration == AD_MANUAL ) { 
        ADC_GetDefaultConfig( &adcConfig ); 
        
        adcConfig.resolution = kADC_Resolution12Bit;
        adcConfig.enableOverWrite = true;
        adcConfig.enableHighSpeed = true;
        adcConfig.samplePeriodMode = kADC_SamplePeriodShort2Clocks;
    
        /* initialize the adc modules */
        ADC_Init( ADC2, &adcConfig );
      
        adc_channel_config_t adcChansCfg[7];
        ADC_EnableHardwareTrigger( ADC2, false );
        
        adcChansCfg[0].channelNumber = LOW_STOCK_CHANNEL;
        adcChansCfg[0].enableInterruptOnConversionCompleted = true;

        adcChansCfg[1].channelNumber = SHOOT_SENSOR_CHANNEL;
        adcChansCfg[1].enableInterruptOnConversionCompleted = true;

        adcChansCfg[2].channelNumber = PAPER_TAKEUP_CHANNEL;
        adcChansCfg[2].enableInterruptOnConversionCompleted = true;

        adcChansCfg[3].channelNumber = HEAD_TEMPERATURE_CHANNEL;
        adcChansCfg[3].enableInterruptOnConversionCompleted = true;

        adcChansCfg[4].channelNumber = HEAD_DOT_CHANNEL;
        adcChansCfg[4].enableInterruptOnConversionCompleted = true;
        
        adcChansCfg[5].channelNumber = HEAD_UP_CHANNEL;
        adcChansCfg[5].enableInterruptOnConversionCompleted = true;

        adcChansCfg[6].channelNumber = HEAD_DETECT_CHANNEL;
        adcChansCfg[6].enableInterruptOnConversionCompleted = true;
        
        ADC_SetChannelConfig( ADC2, 0, &adcChansCfg[0] );   
        
    } else {

        adc_channel_config_t adcChannelConfigStruct;
        
        ADC_GetDefaultConfig( &adcConfig );
        ADC_Init( ADC2, &adcConfig );
        ADC_EnableHardwareTrigger( ADC2, true );

        adcChannelConfigStruct.channelNumber = 10U; /* head up */
        adcChannelConfigStruct.enableInterruptOnConversionCompleted = false;
        ADC_SetChannelConfig( ADC2, 0U, &adcChannelConfigStruct );
        
        adcChannelConfigStruct.channelNumber = 11U; /* head dot */
        ADC_SetChannelConfig( ADC2, 1U, &adcChannelConfigStruct );

        adcChannelConfigStruct.channelNumber = 9U; /* head temp */
        ADC_SetChannelConfig( ADC2, 2U, &adcChannelConfigStruct );

        adcChannelConfigStruct.channelNumber = 8U; /* paper takeup */
        ADC_SetChannelConfig( ADC2, 3U, &adcChannelConfigStruct );
        
        adcChannelConfigStruct.channelNumber = 7U; /* shoot through */
        ADC_SetChannelConfig( ADC2, 4U, &adcChannelConfigStruct );

        adcChannelConfigStruct.channelNumber = 0U; /* label taken */
        ADC_SetChannelConfig( ADC2, 5U, &adcChannelConfigStruct );

        adcChannelConfigStruct.channelNumber = 1U; /* low stock */
        ADC_SetChannelConfig( ADC2, 6U, &adcChannelConfigStruct );
        
        adcChannelConfigStruct.channelNumber = 2U; /* head detection */
        ADC_SetChannelConfig( ADC2, 7U, &adcChannelConfigStruct );

    }

    if( !calOnce_ ) {
        /* auto hardware calibration adc module. */
        if( configuration == AD_AUTO ) {                                           
            if( kStatus_Success != ADC_DoAutoCalibration( ADC2 ) ) {
                PRINTF("ADC_DoAutoCalibration() Failed.\r\n");
            } else {
                calOnce_ = true;
            }                
        } else {
            if( kStatus_Success != ADC_DoAutoCalibration( ADC2 ) ) {
                PRINTF("ADC_DoAutoCalibration() Failed.\r\n");
            } else {
                calOnce_ = true;
            }
        }
    }

    if( configuration == AD_AUTO ) {                                     
        /* initialize adc etc module */  
        initializeEtcAdc( configuration );
    }

    if( configuration == AD_AUTO ) {
        EnableIRQ( ADC_ETC_IRQ0_IRQn );
        EnableIRQ( ADC_ETC_ERROR_IRQ_IRQn );
        
        /* start pit timer channel0. */
        PIT_StartTimer( PIT, kPIT_Chnl_0 );            
    } else {
        NVIC_SetPriority( ADC2_IRQn, 5 );        
        EnableIRQ( ADC2_IRQn );           
    }
}

/******************************************************************************/
/*!   \fn void initLabelTakenLED( void )                                                            
 
      \brief
        This function initializes the quad flex timer for generating a 10Khz
        20% duty cycle signal to the label taken sensor. The xbar is used to 
        connect the timer output to the gpio pin of the sensor.
          
      \author
          Aaron Swift
*******************************************************************************/ 
void initLabelTakenLED( void )
{
    /* enable the label taken sensor */
    //enable pin must be held high for >= 50ns before going low to allow C3 to discharge fully
    GPIO_WritePinOutput( LABEL_TAKEN_EN_GPIO, LABEL_TAKEN_EN_PIN, false ); 
  
    for(int wait = 0; wait < 30000; wait++)
    {
        for(int wait2 = 0; wait2 < 20000; wait2++)
        {
            __NOP();
        }
    }
    
    GPIO_WritePinOutput( LABEL_TAKEN_EN_GPIO, LABEL_TAKEN_EN_PIN, true ); 
  
    for(int wait = 0; wait < 30000; wait++)
    {
        for(int wait2 = 0; wait2 < 20000; wait2++)
        {
            __NOP();
        }
    }
    
    GPIO_WritePinOutput( LABEL_TAKEN_EN_GPIO, LABEL_TAKEN_EN_PIN, false );
  
    qtmr_config_t qtmrConfig;
    
    QTMR_GetDefaultConfig( &qtmrConfig );
    /* initial the output channel. */
    qtmrConfig.primarySource = kQTMR_ClockDivide_4;
    QTMR_Init( TMR1, kQTMR_Channel_1, &qtmrConfig );

    /* generate a 10Khz PWM signal with 20U% dutycycle */
    QTMR_SetupPwm( TMR1, kQTMR_Channel_1, 10000U, 20U, false,
                   ( CLOCK_GetFreq( kCLOCK_IpgClk ) / 4U ) );
    
    QTMR_StartTimer( TMR1, kQTMR_Channel_1, kQTMR_PriSrcRiseEdge); 
}

/******************************************************************************/
/*!   \fn void initializeEtcAdc( ADConfig configuration )                                                     
 
      \brief
        This function initializes the ETC ADC interface for chaining channels  
        into one trigger group which is triggered every 100mS from the pit timer.
          
      \author
          Aaron Swift
*******************************************************************************/ 
void initializeEtcAdc( ADConfig configuration )
{
    adc_etc_config_t adcEtcConfig;
    adc_etc_trigger_config_t adcEtcTriggerConfig;
    adc_etc_trigger_chain_config_t adcEtcTriggerChainConfig;

    ADC_ETC_GetDefaultConfig( &adcEtcConfig );
    adcEtcConfig.XBARtriggerMask = 16U;          /* enable the external XBAR trigger0. */
    adcEtcConfig.enableTSCBypass = false;
    ADC_ETC_Init( ADC_ETC, &adcEtcConfig );

    /* Set the external XBAR trigger0 configuration. */
    adcEtcTriggerConfig.enableSyncMode      = false;
    adcEtcTriggerConfig.enableSWTriggerMode = false;
    adcEtcTriggerConfig.triggerChainLength  = 6U; 
    adcEtcTriggerConfig.triggerPriority     = 0U;
    adcEtcTriggerConfig.sampleIntervalDelay = 0U;
    adcEtcTriggerConfig.initialDelay        = 0U;
    ADC_ETC_SetTriggerConfig( ADC_ETC, 4U, &adcEtcTriggerConfig );
    
    /* Set the external XBAR trigger0 chain configuration. */
    adcEtcTriggerChainConfig.enableB2BMode       = true;
    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 0U; 
    adcEtcTriggerChainConfig.ADCChannelSelect = 10U; /* head up */
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_InterruptDisable;
    ADC_ETC_SetTriggerChainConfig( ADC_ETC, 4U, 0U, &adcEtcTriggerChainConfig);
    
    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 1U;
    adcEtcTriggerChainConfig.ADCChannelSelect = 11U;  /* head dot */ 
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_InterruptDisable; 
    ADC_ETC_SetTriggerChainConfig( ADC_ETC, 4U, 1U, &adcEtcTriggerChainConfig); 

    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 2U;
    adcEtcTriggerChainConfig.ADCChannelSelect = 9U;  /* head temp */ 
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_InterruptDisable; 
    ADC_ETC_SetTriggerChainConfig( ADC_ETC, 4U, 2U, &adcEtcTriggerChainConfig); 

    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 3U; 
    adcEtcTriggerChainConfig.ADCChannelSelect = 8U; /* paper takeup */
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_InterruptDisable; 
    ADC_ETC_SetTriggerChainConfig( ADC_ETC, 4U, 3U, &adcEtcTriggerChainConfig); 

    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 4U; 
    adcEtcTriggerChainConfig.ADCChannelSelect = 7U;  /* shoot through */
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_InterruptDisable; 
    ADC_ETC_SetTriggerChainConfig( ADC_ETC, 4U, 4U, &adcEtcTriggerChainConfig); 

    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 5U; 
    adcEtcTriggerChainConfig.ADCChannelSelect = 0U;  /* label taken */ 
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_InterruptDisable; 
    ADC_ETC_SetTriggerChainConfig( ADC_ETC, 4U, 5U, &adcEtcTriggerChainConfig); 
    
    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 6U; 
    adcEtcTriggerChainConfig.ADCChannelSelect = 1U; /* low stock */ 
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_Done0InterruptEnable; 
    ADC_ETC_SetTriggerChainConfig( ADC_ETC, 4U, 6U, &adcEtcTriggerChainConfig);

    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 2U << 7U; 
    adcEtcTriggerChainConfig.ADCChannelSelect = 2U; /* head detect */ 
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_Done0InterruptEnable; 
    ADC_ETC_SetTriggerChainConfig( ADC_ETC, 4U, 7U, &adcEtcTriggerChainConfig);    
    
}

/******************************************************************************/
/*!   \fn void initXbar( ADConfig configuration )                                                         
 
      \brief
        This function initializes the the XBAR interface and connect flex timer1 
        signal to gpio pin for label taken pwm and connect pit timer to 
        adc etc module if in auto mode. 
          
      \author
          Aaron Swift
*******************************************************************************/ 
void initXbar( ADConfig configuration )
{   
    XBARA_Init( XBARA );
    /* configure the xbara signal connections. */ 
    XBARA_SetSignalsConnection( XBARA, kXBARA1_InputQtimer1Tmr1, kXBARA1_OutputIomuxXbarInout09 ); 
    if( configuration == AD_AUTO ) {
        /* connect pit timer output to adc1 conversion */
        XBARA_SetSignalsConnection( XBARA, kXBARA1_InputPitTrigger0, kXBARA1_OutputAdcEtcTrig10 );        
    }
}

/******************************************************************************/
/*!   \fn void initPitADC( void )                                                          
 
      \brief
        This function initializes the pit timer to initiate a conversion every
        100mS. 
          
      \required xbar connection. 

      \author
          Aaron Swift
*******************************************************************************/ 
void initPitADC( void )
{
    pit_config_t pitConfig;

    /* init pit module */
    PIT_GetDefaultConfig( &pitConfig );
    PIT_Init( PIT, &pitConfig );

    
    PIT_SetTimerPeriod( PIT, kPIT_Chnl_0, USEC_TO_COUNT( 100U, CLOCK_GetFreq( kCLOCK_PerClk ) ) );
    
    initializeVScopeTimer();

    PIT_EnableInterrupts( PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable );   
    EnableIRQ( PIT_IRQn );
}

/******************************************************************************/
/*!   \fn void pitHandleIsr( void )                                                          
 
      \brief
        This function handles the pit timer interrupt.
                
      \author
          Aaron Swift
*******************************************************************************/ 
void pitHandleIsr( void )
{
    PIT_ClearStatusFlags( PIT, kPIT_Chnl_0, kPIT_TimerFlag );    
    /* added for arm errata 838869 */
    SDK_ISR_EXIT_BARRIER;
}

/******************************************************************************/
/*!   \fn void adcEtc0HandleIsr( void )                                                          
 
      \brief
        This function handles etc adc interrupt.
                
      \author
          Aaron Swift
*******************************************************************************/ 
void adcEtc0HandleIsr( void )
{    
    //GPIO_WritePinOutput( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, true );
 
    uint32_t flags = ADC_ETC_GetInterruptStatusFlags( ADC_ETC, kADC_ETC_Trg4TriggerSource  );
    ADC_ETC_ClearInterruptStatusFlags( ADC_ETC, kADC_ETC_Trg4TriggerSource, kADC_ETC_Done0StatusFlagMask );
    ADC_ETC_ClearInterruptStatusFlags( ADC_ETC, kADC_ETC_Trg4TriggerSource, kADC_ETC_Done1StatusFlagMask );
    /* get trigger 0 group 0 chain 0 - 6 results. */
    adcManager.value[CHANNEL_HEAD_STATE] = ADC_ETC_GetADCConversionValue( ADC_ETC, 4U, 0U);  /* head up */
    adcManager.value[CHANNEL_HEAD_DOT] = ADC_ETC_GetADCConversionValue( ADC_ETC, 4U, 1U);  /* head dot */ 
    adcManager.value[CHANNEL_HEAD_TEMP] = ADC_ETC_GetADCConversionValue( ADC_ETC, 4U, 2U);  /* head temp */ 
    adcManager.value[CHANNEL_PAPER_TAKEUP] = ADC_ETC_GetADCConversionValue( ADC_ETC, 4U, 3U);  /* paper takeup */
    adcManager.value[CHANNEL_SHOOT] = ADC_ETC_GetADCConversionValue( ADC_ETC, 4U, 4U);  /* shoot through */
    adcManager.value[CHANNEL_TAKEN] = ADC_ETC_GetADCConversionValue( ADC_ETC, 4U, 5U);  /* label taken */ 
    adcManager.value[CHANNEL_LOW_STOCK] = ADC_ETC_GetADCConversionValue( ADC_ETC, 4U, 6U);  /* low stock */
    adcManager.value[CHANNEL_HEAD_DETECT] = ADC_ETC_GetADCConversionValue( ADC_ETC, 4U, 7U);  /* head detect */ 
    if( motorStopGap_ ) {
        if( adcManager.value[CHANNEL_SHOOT] <= BACKING_PAPER_THRESHOLD ) {
            mainMotorStopped_ = true;
            powerOffStepper();
        }
    }
    
    if( motorStopEdge_ ) {
        if( adcManager.value[CHANNEL_SHOOT] >= LABEL_EDGE_THRESHOLD ) {
            mainMotorStopped_ = true;     
            powerOffStepper();
        }
    }
    
    /*
    TFinkMediaBufferSum = TFinkMediaBufferSum - TFinkMediaBuffer[TFinkMediaBufferPtr];
    TFinkMediaBuffer[TFinkMediaBufferPtr] = adcManager.value[CHANNEL_SHOOT];
    TFinkMediaBufferSum += TFinkMediaBuffer[TFinkMediaBufferPtr];  
    TFinkMediaSensorAvg = TFinkMediaBufferSum/TFINKMEDIABUFFERSIZE;
    
    TFinkMediaBufferPtr++;
    if(TFinkMediaBufferPtr >= TFINKMEDIABUFFERSIZE)
       TFinkMediaBufferPtr = 0;
        
    adcManager.value[CHANNEL_SHOOT] = TFinkMediaSensorAvg;
    */

    adcManager.adcComplete = true;
    
    //GPIO_WritePinOutput( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, false );
    /* added for arm errata 838869 */
    SDK_ISR_EXIT_BARRIER;
}

/******************************************************************************/
/*!   \fn void adcEtcErrorHandleIsr( void )                                                          
 
      \brief
        This function handles etc adc error flags.
                
      \author
          Aaron Swift
*******************************************************************************/ 
void adcEtcErrorHandleIsr( void )
{
    uint32_t flags = ADC_ETC_GetInterruptStatusFlags( ADC_ETC, kADC_ETC_Trg4TriggerSource  );
    PRINTF( "adcEtcErrorHandleIsr: flags %d\r\n", flags );
    ADC_ETC_ClearInterruptStatusFlags( ADC_ETC, kADC_ETC_Trg4TriggerSource, kADC_ETC_ErrorStatusFlagMask );

    /* added for arm errata 838869 */
    SDK_ISR_EXIT_BARRIER;        
}

void pauseResumeConversions( bool pause )
{
    adcManager.adcPaused = pause;
    PRINTF("ADC MANAGER PAUSE = %d\r\n", pause);
}

/******************************************************************************/
/*!   \fn unsigned short getShootThroughConversion( void )                                                           
 
      \brief
        This function sets up the shoot through channel and starts a conversion.
        This function blocks until the channel conversion is complete.
        

      \author
          Aaron Swift
*******************************************************************************/ 
unsigned short getShootThroughConversion( void )
{
    if( mode_ != AD_AUTO ) {  
        adc_channel_config_t adcChanCfg;  
        
        adcChanCfg.channelNumber = SHOOT_SENSOR_CHANNEL;
        adcChanCfg.enableInterruptOnConversionCompleted = true;    
        adcManager.adc0Channel = CHANNEL_SHOOT;
        adcManager.adcComplete = false;    
                                        
        ADC_SetChannelConfig( ADC2, 0, &adcChanCfg );
        while( !adcManager.adcComplete ) {
            taskYIELD();      
        }
    } 
    return adcManager.value[ CHANNEL_SHOOT ];
}

/******************************************************************************/
/*!   \fn unsigned short getShootThroughConversion( void )                                                           
 
      \brief
        This function sets up the shoot through channel and starts a conversion.

      \author
          Aaron Swift
*******************************************************************************/ 
unsigned short pollMediaCounts( void )
{

    if( mode_ != AD_AUTO ) {
        adc_channel_config_t adcChanCfg;  
        
        adcChanCfg.channelNumber = SHOOT_SENSOR_CHANNEL;
        adcChanCfg.enableInterruptOnConversionCompleted = false;    
        adcManager.adc0Channel = CHANNEL_SHOOT;
        adcManager.adcComplete = true;    
                                        
        ADC_SetChannelConfig( ADC2, 0, &adcChanCfg );
        while ( 0U == ADC_GetChannelStatusFlags(ADC2, 0) )  {
        
        }
        unsigned short result = ADC_GetChannelConversionValue( ADC2, 0 );
        return result;
    } else {
        return adcManager.value[CHANNEL_SHOOT];
    }
    
}

/******************************************************************************/
/*!   \fn bool isADCAutoMode( void )                                                           
 
      \brief
        This function return true if adc is setup for auto mode of operation.

      \author
          Aaron Swift
*******************************************************************************/ 
bool isADCAutoMode( void )
{
    bool result = false;
    if( mode_ == AD_AUTO ) {
        result = true;
    }
    return result;
}

/******************************************************************************/
/*!   \fn void readADChannels( void )                                                            
 
      \brief
        This function reads all A/D convertor channels from the ADCManager.
        The ADCManager is automaticaly updated through the pdb/adc/edma chain.
        The pdb generates a channel interrupt every 333uS.

      \author
          Aaron Swift
*******************************************************************************/ 
void readADChannels( void )
{ 
    /* intiate new conversion */
    headSensors.headVoltage   = adcManager.value[CHANNEL_HEAD_DOT];
    headTemperature           = getPrintheadTemperatureInCelsius();           
    headSensors.mediaDetector = adcManager.value[CHANNEL_SHOOT];
    headSensors.headUp        = adcManager.value[CHANNEL_HEAD_STATE];
    headSensors.labelTaken    = readLabelTakenSensor();      
    
    readMediaSensor( &currentStatus );
    readHeadTemperatureSensor( &currentStatus );
    readHeadUpSensor( &currentStatus );
    processLabelTakenSensor( &currentStatus );
  
    prevHeadUp = currentHeadUp;
    
    if(headSensors.headUp > 1000)
    {
        currentHeadUp = true;
        
        currentStatus.sensor2 &= ~JAMMED_LABEL;
        currentStatus.sensor2 &= ~LOW_STOCK_REACHED;
        currentStatus.sensor &= ~OUT_OF_MEDIA;
        
        compareStatus(&currentStatus, &prevStatus);
    }
    else
    {
        currentHeadUp = false;
    }

    if(currentHeadUp == false && prevHeadUp == true)
    {
        //currentStatus.sensor2 &= ~OUT_OF_STOCK;
        currentStatus.sensor2 &= ~JAMMED_LABEL;
        currentStatus.sensor2 &= ~LOW_STOCK_REACHED;
        currentStatus.sensor &= ~OUT_OF_MEDIA;
        
        compareStatus(&currentStatus, &prevStatus);
        
        taskYIELD();

        if(adcManager.value[CHANNEL_SHOOT] < ((float)config_.backingPaper * 0.94)) 
        { 
            currentStatus.sensor |= OUT_OF_MEDIA;
        }
        else
        {
            //checkForPaper(((float)config_.takeup_sensor_max_tension_counts * 0.90), 820);
            
            setHalfStepMode(_MAIN_STEPPER);
            setHalfStepMode(_TAKEUP_STEPPER);
          
            setTakeUpMotorDirection( FORWARDM_ ); 
            setMainMotorDirection( BACKWARDM_ );
            
            if(getTakingUpPaper() == true)
            {
                loosenStock(50, 1700);
            }           
            
            takeupDelay();
            
            if( ( currentStatus.mask.user == 1 ) && ( getGapCalStatus() == false ) )
            {                
                stepToLt(3000, 600);
                
                stepToNextLabel(1500, 600);
                
                backwindAfterContExpel = true;

                //PRINTF("\r\nJAMMED_LABEL cleared\r\n");
            }
        }
    }
    
    /* SOF-4912 */                              
    if( ( currentStatus.error & HEAD_UP ) && currentStatus.command == 0x89 ) {  /* cut retract command complete*/
        currentStatus.command = 0;    
    }
    
    if(headSensors.mediaDetector < ((float)config_.backingPaper * 0.94)) 
    {
        currentStatus.sensor |= OUT_OF_MEDIA;
    }
    else
    {
        currentStatus.sensor &= ~OUT_OF_MEDIA;
    }
    
    
    compareStatus(&currentStatus, &prevStatus);
}

/******************************************************************************/
/*!   \fn UINT readHeadVoltage(void)                                                            
 
      \brief
        This function returns the adc reading of the print head voltage.                            
          
      \author
          Aaron Swift
*******************************************************************************/ 
unsigned int readHeadVoltage( void )
{  
    return( headSensors.headVoltage );
}

/******************************************************************************/
/*!   \fn void readMediaSensor( PrStatusInfo *pStatus )                                                           
 
      \brief
        This function sets and clears the associated media sensor bits 
        in the PrStatusInfo structure.    
                                           
      \author
          Aaron Swift
*******************************************************************************/ 
void readMediaSensor( PrStatusInfo *pStatus )
{  
 
    /* leave in for printing routines */
    if( getOutOfMediaCounts() < getMaxOutOfMediaCounts() ) {
        //pStatus->sensor &= ~OUT_OF_MEDIA;
        //pStatus->error &= ~MEDIA_SHUTDOWN;
    } else {
        /* removed for freestanding scale -- replace with shoot-through
        pStatus->sensor |= OUT_OF_MEDIA;
        */        
    }
}

/******************************************************************************/
/*!   \fn unsigned long getLabelTaken( void ) 

      \brief
        This function returns the a/d channel conversion for the label taken
        sensor.

      \return raw a/d value for the label taken sensor.
      
      \author
          Aaron Swift
*******************************************************************************/
unsigned long getLabelTaken( void ) 
{
  
  if( mode_ != AD_AUTO ) {
        adc_channel_config_t adcChanCfg;  
        
        adcChanCfg.channelNumber = TAKEN_SENSOR_CHANNEL;
        adcChanCfg.enableInterruptOnConversionCompleted = false;    
        adcManager.adc0Channel = CHANNEL_TAKEN;
        adcManager.adcComplete = true;    
                                        
        ADC_SetChannelConfig( ADC2, 0, &adcChanCfg );
        while ( 0U == ADC_GetChannelStatusFlags(ADC2, 0) )  {
        
        }
        unsigned short result = ADC_GetChannelConversionValue( ADC2, 0 );
        return result;
    } else {
        return adcManager.value[CHANNEL_TAKEN];
    }
     
}

/******************************************************************************/
/*!   \fn void readLabelTakenSensor( void )

      \brief
        This function reads the label taken sensor gpio pin. Modified this 
        function for freestanding self service scale due to ESD coupling to   
        the sensor detector which is causing motor chatter. This filtering 
        of the signal should improve the issue.

      \return true when label is present.
      
      \author
          Aaron Swift
*******************************************************************************/
bool readLabelTakenSensor( void )
{
    if( mode_ != AD_AUTO ) {
        adc_channel_config_t adcChanCfg;  
    
        adcChanCfg.channelNumber = TAKEN_SENSOR_CHANNEL;
        adcChanCfg.enableInterruptOnConversionCompleted = false;    
        adcManager.adc0Channel = CHANNEL_TAKEN;
        adcManager.adcComplete = true;    
                                        
        ADC_SetChannelConfig( ADC2, 0, &adcChanCfg );
        while ( 0U == ADC_GetChannelStatusFlags(ADC2, 0) )  {};
        unsigned short result = ADC_GetChannelConversionValue( ADC2, 0 );
        
        return result;
    } else {
       static bool previousState_ = false; 

        /* only accept values above and below threshold throw all 
        other values out due to noise. */
        if( adcManager.value[CHANNEL_TAKEN] <= LABEL_TAKEN_THRESHOLD_NO_LABEL ) {
            previousState_ = true;
            return( true );
        } else if( adcManager.value[CHANNEL_TAKEN] > LABEL_TAKEN_THRESHOLD_LABEL ) {
            previousState_ = false;
            return( false );
        } else {
            return( previousState_ ); 
        }  
    }  
}

/******************************************************************************/
/*!   \fn void processLabelTakenSensor( PrStatusInfo *pStatus )

      \brief
        This function sets and clears the associated
         label taken sensor bits in the PrStatusInfo structure.

      \author
          Aaron Swift
*******************************************************************************/
void processLabelTakenSensor( PrStatusInfo *pStatus )
{    
    if( headSensors.labelTaken ) { 
        //NO LABEL IN FRONT OF SENSOR
        setLTWaitCount_(0);
      
        currentStatus.sensor |= LABEL_TAKEN;          
    } 
    else 
    {        
        //LABEL IN FRONT OF SENSOR
        if(getLTWaitCount_() < LABEL_TAKEN_DELAY_LENGTH)
        {
            setLTWaitCount_(getLTWaitCount_() + 1);
        }
        else
        {
            currentStatus.sensor &= ~LABEL_TAKEN;
        }   
    }
    
    compareStatus(&currentStatus, &prevStatus);
}

/******************************************************************************/
/*!   \fn int getGhostMCntr( void )

      \brief
        This function returns the counter that tracks the number of label 
        present bits while printing a label. This is used to detect false or
        ghost missing label occurences.

      \author
          Aaron Swift
*******************************************************************************/
int getGhostMCntr( void )
{
    return ghostM_;
}

void clearGhostMCntr( void )
{
    ghostM_ = 0;   
}


/******************************************************************************/
/*!   \fn void readHeadUpSensor( PrStatusInfo *pStatus )                                                           
 
      \brief
        This function sets and clears the associated head up 
        sensor bits in the PrStatusInfo structure.  
  
                                           
      \author
          Aaron Swift
*******************************************************************************/ 
bool readHeadUpSensor( PrStatusInfo *pStatus )
{
    static unsigned char cnt_ = 0;
    
    /* if not a freestanding scale */
    if( getMyModel() != RT_GLOBAL_FSS ) 
    { 
        if(TUCalHeadUpFlag == false)
        {
            if( headSensors.headUp < NO_CASSET_THRESHOLD ) 
            {  
                pStatus->error &= ~HEAD_UP;   
            } 
            else 
            { 
                pStatus->error |= HEAD_UP;
                resetTUTorqueControlVars();
            }
        }
        else
        {
            pStatus->error &= ~HEAD_UP;
        }
    } 
    else 
    {
        if( headSensors.headUp < FREESTAND_HEADUP_THRESHOLD ) 
        {
            cnt_ = 0;
            pStatus->error &= ~HEAD_UP;   
        } 
        else 
        { 
            cnt_++;
            if( cnt_ >= HEAD_UP_DEBOUNCE ) 
            {
                cnt_ = 0;                
                pStatus->error |= HEAD_UP;
                resetTUTorqueControlVars();
            }
        }
    }
    
    /* gloabl and freestanding does not support narrow label */
    pStatus->sensor |= WIDE_LABEL;    
    
    if(headSensors.headUp < NO_CASSET_THRESHOLD)
    {
        return false;
    }
    else
    {
        return true;
    }
}

/******************************************************************************/
/*!   \fn void readLowStockSensor( PrStatusInfo *pStatus )                                                          
 
      \brief
        This function sets and clears the associated low stock 
        sensor bits in the PrStatusInfo structure.  
  
                                           
      \author
          Aaron Swift
*******************************************************************************/ 
void readLowStockSensor( PrStatusInfo *pStatus )
{
  //TODO: This needs fixed later when we figure out where and when we want to sample this sensor,
  //we are meant to be measuring the frequency of the change in the big and little dip as they rotate  
  
    /* only report out of stock if cassette is installed */
    if( ( pStatus->error & HEAD_UP ) != HEAD_UP ) {
        /* check if we are out of stock */
        if( adcManager.value[CHANNEL_LOW_STOCK] >= MEDIA_OUT_OF_THRESHOLD  ) {
            //pStatus->sensor2 |= OUT_OF_STOCK;
        } else {
            //pStatus->sensor2 &= ~OUT_OF_STOCK;
        }
    }
}

/******************************************************************************/
/*!   \fn unsigned short getPrintheadThermistorCounts()                                                         
 
      \brief
        Returns the ADC counts for the printhead thermistor 
                             
      \author
          Chris King
*******************************************************************************/ 
unsigned short getPrintheadThermistorCounts() 
{
    return adcManager.value[CHANNEL_HEAD_TEMP];
}

unsigned short getTakeUpTorque( void )
{
    if( mode_ != AD_AUTO ) {
        adc_channel_config_t adcChanCfg;  
    
        adcChanCfg.channelNumber = PAPER_TAKEUP_CHANNEL;
        adcChanCfg.enableInterruptOnConversionCompleted = false;    
        adcManager.adc0Channel = CHANNEL_PAPER_TAKEUP;
        adcManager.adcComplete = true;    
                                        
        ADC_SetChannelConfig( ADC2, 0, &adcChanCfg );
        while ( 0U == ADC_GetChannelStatusFlags(ADC2, 0) )  {};
        unsigned short result = ADC_GetChannelConversionValue( ADC2, 0 );
        
        return result;
    } else {
       return adcManager.value[CHANNEL_PAPER_TAKEUP]; 
    }
}

unsigned short getHeadVoltage( void )
{
    return adcManager.value[CHANNEL_HEAD_DOT];  
}
/******************************************************************************/
/*!   \fn void readHeadTemperatureSensor( PrStatusInfo *pStatus )                                                         
 
      \brief
        This function sets and clears the associated head temperature 
        sensor bits in the PrStatusInfo structure.  
                                             
      \author
          Aaron Swift
*******************************************************************************/ 
void readHeadTemperatureSensor( PrStatusInfo *pStatus )
{
    /* are we a global printhead? */
    if( ( getPrintHeadType() == ROHM_72MM_800_OHM ) || ( getPrintHeadType() == ROHM_80MM_650_OHM ) ) 
    {  
        if( getPrintheadTemperatureInCelsius() >= HIGH_HEAD_TEMPERATURE_LIMIT_ROHM ) 
        {          
            /* set the sensor flag */
            pStatus->sensor |= THERMAL_OVERLOAD; 
            //PRINTF("sensor == themal overload 1\r\n");

            if( pStatus->state == ENGINE_PRINTING || pStatus->state == ENGINE_CALIBRATING ) 
            {
                /* thermal overload only becomes a fatal error when 
                   the printer is in the printing state. */
                pStatus->error |= THERMAL_SHUTDOWN;
                //PRINTF("sensor == themal overload 1\r\n");
            } 
            else 
            {
                /* otherwise, thermal overload is not reported as a 
                   fatal error. */                                     
                pStatus->error &= ~THERMAL_SHUTDOWN; 
            }
        } 
        else 
        {
            /* clear the sensor and error flag */
            pStatus->sensor &= ~THERMAL_OVERLOAD; 
            pStatus->error &= ~THERMAL_SHUTDOWN;       
        }        
    } 
    else 
    {
        if( headTemperature <= HIGH_HEAD_TEMPERATURE_LIMIT ) 
        {
            /* set the sensor flag */
            //pStatus->sensor |= THERMAL_OVERLOAD; 

            if( pStatus->state == ENGINE_PRINTING || pStatus->state == ENGINE_CALIBRATING ) 
            {
                /* thermal overload only becomes a fatal error when 
                   the printer is in the printing state. */
               //pStatus->error |= THERMAL_SHUTDOWN;
            } 
            else 
            {
                /* otherwise, thermal overload is not reported as a 
                   fatal error. */                                     
                pStatus->error &= ~THERMAL_SHUTDOWN; 
            }
        } 
        else 
        {
            /* clear the sensor and error flag */
            pStatus->sensor &= ~THERMAL_OVERLOAD; 
            pStatus->error &= ~THERMAL_SHUTDOWN; 
        }
    }
}

/******************************************************************************/
/*!   \fn unsigned char getHeadSupplyCurrent( void )                                                         
 
      \brief
        This function returns the power supply current.         
                                             
      \author
          Aaron Swift
*******************************************************************************/ 
unsigned char getHeadSupplyCurrent( void )
{
    return headSensors.supplyCurrent;
}

/******************************************************************************/
/*!   \fn getCurrentSensors( PrSensors *pSensors )                                              
 
      \brief
        This function returns the current sensor readings.      
                                             
      \author
          Aaron Swift
*******************************************************************************/ 
void getCurrentSensors( PrSensors *pSensors )
{
  
    pSensors->headup_reading                    = headSensors.headUp;

    pSensors->head_current_reading              = adcManager.value[CHANNEL_HEAD_STATE];
        
    pSensors->head_temperature_reading          = getPrintheadTemperatureInCelsius();

    pSensors->head_voltage_reading              = adcManager.value[CHANNEL_HEAD_DOT];
    
    pSensors->label_high_average                = 0;
    pSensors->label_low_average                 = 0;
    
    /* freestanding scale  */
    if( getMyModel() != RT_GLOBAL_FSS ) {
        if( !headSensors.labelTaken ) {
            pSensors->label_reading             = (int)255;
        } else {
            pSensors->label_reading             = 0;  
        }
    } else {
        if( headSensors.labelTaken ) {
            pSensors->label_reading             = 0;
        } else {
            pSensors->label_reading             = (int)255;  
        }
    }
    
    pSensors->label_threshold                   = 0;
    
    /* freestanding scale  has no sensor for width*/
    if( getMyModel() != RT_GLOBAL_FSS ) {    
        pSensors->label_width_reading           = adcManager.value[CHANNEL_HEAD_STATE];
    } else {
        pSensors->label_width_reading           = 60; /* wide width stock only */
    }
    
    pSensors->media_high_average                = 0;
    pSensors->media_low_average                 = 0;
    pSensors->media_threshold                   = 0;
    pSensors->media_reading                     = adcManager.value[CHANNEL_SHOOT];
    
    #if 0       /* TO DO: port low label stock sensor */
    /* unused field now being used for low stock status and engine state */
    if( GPIO_ReadPinInput( LTS_GPIOx, LTS_PINx ) ) {    
        pSensors->regulator_temperature_reading &= ~LOW_STOCK_REACHED;
    } else {
        pSensors->regulator_temperature_reading |= LOW_STOCK_REACHED;
    }
    /* report current engine state */
    if( currentStatus.state != ENGINE_IDLE ) {
        pSensors->regulator_temperature_reading |= ENGINE_BUSY_STATE;
    } else {
        pSensors->regulator_temperature_reading &= ~ENGINE_BUSY_STATE;
    }
    #endif
}
/******************************************************************************/
/*!   \fn void adcModule2IsrHandler(void)                                            
 
      \brief
        This function handles adc2 interrupts. Handler currently read the 
        conversion value for each adc channel.

      \author
          Aaron Swift
*******************************************************************************/ 
void adcModule2IsrHandler( void )
{
    
    if( mode_ != AD_AUTO ) {
        if( adcManager.adc0Channel == CHANNEL_TAKEN ) {
            adcManager.value[0] = ADC_GetChannelConversionValue( ADC2, 0 );
        } else if( adcManager.adc0Channel == CHANNEL_LOW_STOCK ) {
            adcManager.value[1] = ADC_GetChannelConversionValue( ADC2, 0 );
        } else if( adcManager.adc0Channel == CHANNEL_SHOOT ) {
            adcManager.value[2] = ADC_GetChannelConversionValue( ADC2, 0 );
            
            if( motorStopGap_ ) {
                if( adcManager.value[2] <= BACKING_PAPER_THRESHOLD ) {
                    powerOffStepper();
                }
            }
            
            if( motorStopEdge_ ) {
                if( adcManager.value[2] >= LABEL_EDGE_THRESHOLD ) {
                    powerOffStepper();
                }
            }
            
        } else if( adcManager.adc0Channel == CHANNEL_PAPER_TAKEUP ) {
            adcManager.value[3] = ADC_GetChannelConversionValue( ADC2, 0 );
        } else if( adcManager.adc0Channel == CHANNEL_HEAD_TEMP ) {
            adcManager.value[4] = ADC_GetChannelConversionValue( ADC2, 0 );
        } else if( adcManager.adc0Channel == CHANNEL_HEAD_STATE )  {
            adcManager.value[5] = ADC_GetChannelConversionValue( ADC2, 0 );
        } else if( adcManager.adc0Channel == CHANNEL_HEAD_DOT )  {
            adcManager.value[6] = ADC_GetChannelConversionValue( ADC2, 0 );
        } 
    }
    adcManager.adcComplete = true;
    SDK_ISR_EXIT_BARRIER;
}

/******************************************************************************/
/*!   \fn unsigned short sampleHeadDotADC( void )

      \brief
        This function samples one channel of ADC0


      \author
          Aaron Swift
*******************************************************************************/
unsigned short sampleHeadDotADC( void )
{
    
    
    #if 0 /* TO DO: Port to rt1024 */
    adc16_channel_config_t adc16ChannelConfig;
    adc16ChannelConfig.channelNumber = HEAD_DOT_CHANNEL;
    adc16ChannelConfig.enableInterruptOnConversionCompleted = true;
    adc16ChannelConfig.enableDifferentialConversion = false;

    adcManager.adcComplete = false;    
    /* start conversion of head dot value */
    ADC16_SetChannelConfig( ADC16_BASE, ADC16_CHANNEL_GROUP0, &adc16ChannelConfig );
    /* wait for the adc to complete conversion (conversion time:1.489uS) */
    while( !adcManager.adcComplete ) {
        asm("nop");
    }
    
    adcManager.results[indx_].dotPosition = indx_ + 1;
    /* added for dot measurement test */
    if( indx_ == HEAD_DOTS_PREPACK ) {
        adcManager.results[indx_].sample1  = adcManager.value[CHANNEL_HEAD_DOT];
    } else {
        adcManager.results[indx_].sample1 = adcManager.value[CHANNEL_HEAD_DOT];
    }

    adcManager.adcComplete = false;
    GPIO_WritePinOutput( EX_P2_GPIOx, EX_P2_PINx, true );
    /* start conversion of head dot value */
    ADC16_SetChannelConfig( ADC16_BASE, ADC16_CHANNEL_GROUP0, &adc16ChannelConfig );
    /* wait for the adc to complete conversion (conversion time:1.489uS) */
    while( !adcManager.adcComplete ) {
        asm("nop");
    }
    
    /* added for dot measurement test */
    if( indx_ == HEAD_DOTS_PREPACK ) {
        adcManager.results[indx_].sample2  = adcManager.value[CHANNEL_HEAD_DOT];
    } else {
        adcManager.results[indx_].sample2 = adcManager.value[CHANNEL_HEAD_DOT];
    }

    adcManager.adcComplete = false;
    GPIO_WritePinOutput( EX_P2_GPIOx, EX_P2_PINx, true );
    /* start conversion of head dot value */
    ADC16_SetChannelConfig( ADC16_BASE, ADC16_CHANNEL_GROUP0, &adc16ChannelConfig );
    /* wait for the adc to complete conversion (conversion time:1.489uS) */
    while( !adcManager.adcComplete ) {
        asm("nop");
    }

    /* added for dot measurement test */
    if( indx_ == HEAD_DOTS_PREPACK ) {
        adcManager.results[indx_].sample3  = adcManager.value[CHANNEL_HEAD_DOT];
    } else {
        adcManager.results[indx_].sample3 = adcManager.value[CHANNEL_HEAD_DOT];
    }
    incrementIndexDotTest();
    #endif
    return adcManager.value[CHANNEL_HEAD_DOT];
}


/******************************************************************************/
/*!   \fn unsigned short sampleHeadDot( void )

      \brief
        This function samples one channel of ADC0


      \author
          Aaron Swift
*******************************************************************************/
unsigned short sampleHeadDot( void )
{
    #if 0 /* TO DO: port to rt1024 */
    adc16_channel_config_t adc16ChannelConfig;
    adc16ChannelConfig.channelNumber = HEAD_DOT_CHANNEL;
    adc16ChannelConfig.enableInterruptOnConversionCompleted = true;
    adc16ChannelConfig.enableDifferentialConversion = false;

    adcManager.adcComplete = false;
    /* start conversion of head dot value */
    ADC16_SetChannelConfig( ADC16_BASE, ADC16_CHANNEL_GROUP0, &adc16ChannelConfig );
    /* wait for the adc to complete conversion (conversion time:1.489uS) */
    while( !adcManager.adcComplete ) {
        asm("nop");
    }
    #endif    
    return adcManager.value[CHANNEL_HEAD_DOT];
}

/******************************************************************************/
/*!   \fn unsigned short sampleHeadVoltage( void )

      \brief
        This function samples one channel of ADC0


      \author
          Aaron Swift
*******************************************************************************/
unsigned short sampleHeadVoltage( void )
{
    #if 0 /* port to rt1024 */
    adc16_channel_config_t adc16ChannelConfig;
    adc16ChannelConfig.channelNumber = HEAD_VOLTAGE_CHANNEL;
    adc16ChannelConfig.enableInterruptOnConversionCompleted = true;
    adc16ChannelConfig.enableDifferentialConversion = false;

    adcManager.adcComplete = false;
    /* start conversion of head dot value */
    ADC16_SetChannelConfig( ADC16_BASE, ADC16_CHANNEL_GROUP0, &adc16ChannelConfig );
    /* wait for the adc to complete conversion (conversion time:1.489uS) */
    while( !adcManager.adcComplete ) {
        asm("nop");
    }
    #endif
    return adcManager.value[CHANNEL_HEAD_DOT];
}

/******************************************************************************/
/*!   \fn unsigned short getMediaCounts()                                           
 
      \brief
        This function returns the raw ad counts for the shoot through / 
        reflective channel. 

      \author
          Aaron Swift
*******************************************************************************/ 
unsigned short getMediaCounts() 
{    
    return adcManager.value[CHANNEL_SHOOT];
}

void showLabelTakenCounts( void )
{
    PRINTF( "label taken counts: %d\r\n", adcManager.value[CHANNEL_TAKEN] );
}

void showPaperTakeup( void )
{
    PRINTF( "paper takeup counts: %d\r\n", adcManager.value[CHANNEL_PAPER_TAKEUP] );   
}

/******************************************************************************/
/*!   \fn unsigned short getLowStockSensor( void )                                     
 
      \brief
        This function returns the raw ad counts for the low stock channel.
        
      \author
          Aaron Swift
*******************************************************************************/ 
unsigned short getLowStockSensor( void )
{
    return adcManager.value[CHANNEL_LOW_STOCK];  
}

void setMotorStopOnGap( void )
{
    motorStopGap_ = true;
}

void clrMotorStopOnGap( void )
{
    motorStopGap_ = false;    
}

void setMotorStopOnEdge( void )
{
   motorStopEdge_ = true;
}

void clrMotorStopOnEdge( void )
{
    motorStopEdge_ = false;
}


/******************************************************************************/
/*!   \fn void setLabelAlignment( unsigned int actualSize )                                          
 
      \brief
        This function sets the label vertical alignment value used to synchronize the 
        label stock.

      \author
          Aaron Swift
*******************************************************************************/ 
void setLabelAlignment( unsigned int measuredSize, unsigned int actualSize )
{
#if 0 /* TO DO: look into if this is still needed for diecut labels? */
    measuredLength_ = measuredSize;
    actualLength_ = labelAlignment = actualSize;  
    /* decide what the alignment should be based on the actual size of the 
       label used as the measured distance from sync bar to sync bar. */ 
    if( actualSize > MEDIA_SENSOR_TO_ALIGNMENT ) {
        PRINTF( "labelAlignment == MEDIA_SENSOR_TO_ALIGNMENT: %d\r\n", MEDIA_SENSOR_TO_ALIGNMENT );
        labelAlignment = MEDIA_SENSOR_TO_ALIGNMENT;
    } else {        
        /* this calculation must be adjusted by a fudge factor for labels 
           shorter than the media sensor to alignment position distance 
           versus those longer (possibly due to slip when stopping). */ 

        /* enable this to get labels <= 3.00" to align correctly C.G. */
	  	measuredSize += 60;
		PRINTF( "labelAlignment abs: %d\r\n", MEDIA_SENSOR_TO_ALIGNMENT );
        PRINTF( "labelAlignment abs: %d\r\n", measuredSize );
        labelAlignment = abs( MEDIA_SENSOR_TO_ALIGNMENT % measuredSize );
  
        PRINTF( "labelAlignment: %d\r\n", labelAlignment );
		
		labelAlignment += config_.verticalPosition;
	} 
    PRINTF( "adjusting label position by: %d\r\n", config_.verticalPosition );
    PRINTF( "label position: %d\r\n", labelAlignment );
	
#if 1//Enable this to get labels <= 3.00" to alingn correctly C.G
	labelAlignment += config_.verticalPosition;
#else	
    /* add the vertical position adjustment to this value. */
    if( getPrintEngine()->labelOrientation == HEAD_FIRST ) {       
        labelAlignment += config_.verticalPosition;
    } else {
        labelAlignment += abs( config_.verticalPosition );        
    }
#endif	
    PRINTF( "label position after adjust: %d\r\n", labelAlignment );
#endif
}

/******************************************************************************/
/*!   \fn void updateLabelAlignment( void )                                          
 
      \brief
        This function updates the print position of the label.

      \author
          Aaron Swift
*******************************************************************************/ 
void updateLabelAlignment( void )
{
    setLabelAlignment( measuredLength_, actualLength_ );
}


/******************************************************************************/
/*!   \fn void incrementIndexDotTest( void )                                          
 
      \brief
        This function increments the dot wear index.

      \author
          Aaron Swift
*******************************************************************************/ 
void incrementIndexDotTest( void )
{
    if( indx_ <= HEAD_DOTS_72MM ) {
        indx_++;
    }
}

/******************************************************************************/
/*!   \fn void updateMediaSensor( void )                                        
 
      \brief
        This function updates the shoot through sensor value. 

      \author
          Aaron Swift
*******************************************************************************/ 
void updateMediaSensor( void )
{
    headSensors.mediaDetector = adcManager.value[CHANNEL_SHOOT];
}

/******************************************************************************/
/*!   \fn void showADCReadings( void )                                        
 
      \brief
        This function shows the adc values measured. 

      \author
          Aaron Swift
*******************************************************************************/ 
void showADCReadings( void )
{
    PRINTF("showADCReadings(): ***********************\r\n" );
    float resolution = .00080566, value = 0.0;

    PRINTF( "label taken counts: %d\r\n", adcManager.value[CHANNEL_TAKEN] ); 
    value = (float)(adcManager.value[CHANNEL_TAKEN] * resolution);
    PRINTF( "label taken voltage: %2.3fV\r\n", value );    
    
    if( adcManager.value[CHANNEL_TAKEN] >= LABEL_TAKEN_THRESHOLD_LABEL ) {
        PRINTF( "label taken at sensor \r\n");
    } else {
        PRINTF( "label taken\r\n");
    }
    
    PRINTF( "low stock counts: %d\r\n", adcManager.value[CHANNEL_LOW_STOCK] ); 
    value = (float)(adcManager.value[CHANNEL_LOW_STOCK] * resolution);
    PRINTF( "low stock voltage: %2.3fV\r\n", value );    
    if( adcManager.value[CHANNEL_LOW_STOCK] >= MEDIA_OUT_OF_THRESHOLD  ) {
        PRINTF( "out of stock\r\n"); 
    } 

    PRINTF( "shoot through counts: %d\r\n", adcManager.value[CHANNEL_SHOOT] ); 
    value = (float)(adcManager.value[CHANNEL_SHOOT] * resolution);
    PRINTF( "shoot through voltage: %2.3fV\r\n", value );    

    
    PRINTF( "paper takeup counts: %d\r\n", adcManager.value[CHANNEL_PAPER_TAKEUP] ); 
    value = (float)(adcManager.value[CHANNEL_PAPER_TAKEUP] * resolution);
    PRINTF( "paper takeup voltage: %2.3fV\r\n", value );    

    PRINTF( "head temp counts: %d\r\n", adcManager.value[CHANNEL_HEAD_TEMP] ); 
    value = (float)(adcManager.value[CHANNEL_HEAD_TEMP] * resolution);
    PRINTF( "head temp voltage: %2.3fV\r\n", value );      
    PRINTF( "head temperature: %dC\r\n", getPrintheadTemperatureInCelsius() );             

    
    PRINTF( "head state counts: %d\r\n", adcManager.value[CHANNEL_HEAD_STATE] ); 
    value = (float)(adcManager.value[CHANNEL_HEAD_STATE] * resolution);
    PRINTF( "head state voltage: %2.3fV\r\n", value );    
    if( adcManager.value[CHANNEL_HEAD_STATE] >= NO_CASSET_THRESHOLD  ) {
        PRINTF( "head is up\r\n"); 
    } 
          

    #if 0
    float resolution = .0130, value = 0.0;
    PRINTF("showADCReadings(): ***********************\r\n" );

    PRINTF( "head voltage counts: %d\r\n", adcManager.value[CHANNEL_HEAD_VOLT] ); 
    value = (float)(adcManager.value[CHANNEL_HEAD_VOLT] * resolution);
    PRINTF( "head voltage: %2.3fV\r\n", value );    
    /* PRINTF( "head shoot counts: %d\r\n", adcManager.value[CHANNEL_SHOOT] );  
    value = (float)(adcManager.value[CHANNEL_SHOOT] * resolution); */ 
    PRINTF( "head shoot voltage: %2.3fV\r\n", value );    

    PRINTF( "head state counts: %d\r\n", adcManager.value[CHANNEL_HEAD_STATE] );   
    value = (float)(adcManager.value[CHANNEL_HEAD_STATE] * resolution);
    PRINTF( "head state voltage: %2.3fV\r\n", value );    
    PRINTF( "head dot counts: %d\r\n", adcManager.value[CHANNEL_HEAD_DOT] );    
    value = (float)(adcManager.value[CHANNEL_HEAD_DOT] * resolution);
    PRINTF( "head dot voltage: %2.3fV\r\n", value );    
   
    if( headSensors.headUp > NO_CASSET_THRESHOLD ) {
        PRINTF( "Head is up. \r\n" ); 
    }

    if( headSensors.headUp > WIDE_CASSET_THRESHOLD ) {
        PRINTF( "Wide label casset installed. \r\n" ); 
    }

    PRINTF( "media counts: %d\r\n", adcManager.value[CHANNEL_SHOOT] );
    value = (float)(adcManager.value[CHANNEL_SHOOT] * resolution);
    PRINTF( "media voltage: %2.3fV\r\n", value ); 
    
    if( headSensors.mediaDetector > MEDIA_SYNC_BAR_THRESHOLD ) {
        PRINTF( "label sync bar detected. \r\n" ); 
    }
    if( headSensors.labelTaken ) {
        PRINTF( "take label. \r\n" );       
    }

    PRINTF( "head temperature counts: %d\r\n", adcManager.value[CHANNEL_HEAD_TEMP] );
    int degrees = 0;
    if( adcManager.value[CHANNEL_HEAD_TEMP] >= ZERO_DEGREES_COUNT ) {
        degrees = ZERO_DEGREES_C;
    } else if( adcManager.value[CHANNEL_HEAD_TEMP] >= TEN_DEGREES_COUNT ) {
        degrees = TEN_DEGREES_C;
    } else if( adcManager.value[CHANNEL_HEAD_TEMP] >= TWENTY_DEGREES_COUNT ) {
        degrees = TWENTY_DEGREES_C;      
    } else if( adcManager.value[CHANNEL_HEAD_TEMP] >= THIRTY_DEGREES_COUNT ) {
          degrees = THIRTY_DEGREES_C;      
    } else if( adcManager.value[CHANNEL_HEAD_TEMP] >= FOURTY_DEGREES_COUNT ) {
        degrees = FOURTY_DEGREES_C;      
    } else if( adcManager.value[CHANNEL_HEAD_TEMP] >= FIFTY_DEGREES_COUNT ) {
        degrees = FIFTY_DEGREES_C;
    } else {
        degrees = SIXTY_DEGREES_C;
    }      
    PRINTF("showADCReadings(): ***********************\r\n" );
    #endif
}

/******************************************************************************/
/*!   \fn void setADCMode( ADConfig mode )                                    
 
      \brief
        This function sets both adc modules into either auto or manual mode
        of operation. 

      \author
          Aaron Swift
*******************************************************************************/ 
void setADCMode( ADConfig mode )
{
    mode_ = mode;
}

/******************************************************************************/
/*!   \fn unsigned short sampleMediaSensor( void )

      \brief
        This function samples one channel of ADC0


      \author
          Aaron Swift
*******************************************************************************/
unsigned short sampleMediaSensor( void )
{       
    if( mode_ != AD_AUTO ) {
        adc_channel_config_t adcChanCfg;
        adcChanCfg.channelNumber = TAKEN_SENSOR_CHANNEL;
        adcChanCfg.enableInterruptOnConversionCompleted = true;

        adcManager.adcComplete = false;
        /* start conversion of head dot value */
        ADC_SetChannelConfig( ADC2, 5, &adcChanCfg );
        
        /* wait for the adc to complete conversion (conversion time:1.489uS) */
        while( !adcManager.adcComplete ) {
            asm("nop");
        }
        /*
          Since we are running 12-bit conversions we need to convert to 8-bit to play
          nice with others
        */
        adcManager.value[CHANNEL_SHOOT] = (int) round((float)adcManager.value[CHANNEL_SHOOT] / 16.0);
    }
    return adcManager.value[CHANNEL_SHOOT];
}

bool getBackwindAfterContExpel( void )
{
    return backwindAfterContExpel;
}

void setBackwindAfterContExpel( bool backwind )
{
    backwindAfterContExpel = backwind;
}

bool getTUCalHeadUpFlag( void )
{
    return TUCalHeadUpFlag;
}

void setTUCalHeadUpFlag( bool cal )
{
    TUCalHeadUpFlag = cal;
}

