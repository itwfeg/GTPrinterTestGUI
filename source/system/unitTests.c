#include "unitTest.h"
#include "prMessages.h"
#include "commandTable.h"
#include "printEngine.h"
#include "serialFlash.h"
#include "globalPrinter.h"
#include "dvr8818.h"
#include "lp5521.h"
#include "valueMax.h"
#include "cs5530.h"
#include "sensors.h"
#include "takeupMotor.h"
#include "fsl_gpt.h"
#include "fsl_iomuxc.h"
#include "fsl_gpio.h"
#include "fsl_adc.h"
#include "fsl_adc_etc.h"
#include "fsl_pit.h"
#include "fsl_pwm.h"
#include "fsl_xbara.h"
#if 1
#include "fsl_lpspi_edma.h"
#include "fsl_edma.h"
#include "fsl_dmamux.h"
#endif
#include "fsl_common.h"
#include "pin_mux.h"
#include "fsl_common_arm.h"
#include "fsl_debug_console.h"


#if 1
void LPSPI4_MasterUserCallback( LPSPI_Type *base, lpspi_master_edma_handle_t *handle, status_t status, void *userData );
void LPSPI4_MMasterUserCallback( LPSPI_Type *base, lpspi_master_handle_t *handle, status_t status, void *userData );
extern volatile bool spiDmaTransferComplete;
extern edma_handle_t eDmaHandle_0, eDmaHandle_1, eDmaHandle_2;
lpspi_master_edma_handle_t spi4HeadMasterHandle;
lpspi_master_handle_t   spi4ManualHandle;
#endif

//#define generalTimerIsr GPT2_IRQHandler
//#define adc1HandleIsr ADC1_IRQHandler
//#define weigherDataRdyIsr GPIO1_Combined_0_15_IRQHandler

//#define adcEtc0HandleIsr ADC_ETC_IRQ0_IRQHandler
//#define adcEtcErrorHandleIsr ADC_ETC_ERROR_IRQ_IRQHandler

//#define pitTimerHandleIsr PIT_IRQHandler

volatile uint32_t adcChannels[4] = { 0 };
volatile bool adcComplete_ = false; 
ADCMode mode_ = _AUTO;//_MANUAL; //_AUTO;

extern lpspi_master_edma_handle_t spiHeadMasterHandle;
extern PrStatusInfo currentStatus;
void lpspiCSCallback(LPSPI_Type *base, lpspi_master_handle_t *handle, status_t status, void *userData);


static TaskHandle_t     utHandle_               = NULL;
static bool             suspend_                = false;
static UTests           uTest_                  = _UT_INIT;
static UTSubTst         usTest_                 = _UT_SUB_INIT;

Pr_Config               utPConfig_;
Pr_Config               utPTConfig_;

WgConfiguration         utWconfig_;
WgConfiguration         utWTConfig_;

FPMBLC3Checksums        utSums_;
FPMBLC3Checksums        utTestSums_;
unsigned long           rCksCnt_                = 0;


bool _ready_ = false;


//#define ADC_ETC_DONE0_FLAG (0x1U)
//#define ADC_ETC_DONE1_FLAG (0x2U)
//#define adcMod2IsrHandler ADC2_IRQHandler
void testPaperTakeup( void )
{
    initializeMotors(FORWARDM_); 
            
    initTakeupIntr(); 
    
    powerOnMotors();
    
    stepTUMotor( 10000, 820 );

}



void testADC2Auto( void )
{
    adc_config_t adcConfig;    

    ADC_GetDefaultConfig( &adcConfig );
      
    adcConfig.resolution = kADC_Resolution12Bit;
    //adcConfig.enableHighSpeed = true;
    
    /* initialize the adc modules */
    ADC_Init( ADC2, &adcConfig );
    ADC_EnableHardwareTrigger( ADC2, false );
    adc_channel_config_t adcChansCfg;
  
    
    /* auto hardware calibration both adc modules. */
    if( kStatus_Success != ADC_DoAutoCalibration( ADC2 ) ) {
        PRINTF("ADC_DoAutoCalibration() Failed.\r\n");
    } 
   // GPIO_WritePinOutput( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, true );
    NVIC_SetPriority( ADC2_IRQn, 10 );        
    EnableIRQ( ADC2_IRQn );           

    adcChansCfg.channelNumber = SHOOT_SENSOR_CHANNEL;
    adcChansCfg.enableInterruptOnConversionCompleted = true;
    //GPIO_WritePinOutput( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, true );
    //
   // GPIO_WritePinOutput( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, true );
    ADC_SetChannelConfig( ADC2, 0, &adcChansCfg );  
   // GPIO_WritePinOutput( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, false );
//    while( !_ready_ ) {
//        
//        asm("nop");
//    }

    
    //while( 1 ) {
    //    asm("nop");
    //}

}
#if 0
void adcMod2IsrHandler( void )
{
    //unsigned long flags = ADC_GetChannelStatusFlags( ADC2,  0 );
    unsigned long value = ADC_GetChannelConversionValue( ADC2, 0 );
    
    _ready_ = true;
   // GPIO_WritePinOutput( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, true );
    SDK_ISR_EXIT_BARRIER;
}
#endif 
void testADCAuto( void )
{
    /* Set PERCLK_CLK source to OSC_CLK*/
    CLOCK_SetMux(kCLOCK_PerclkMux, 1U);
    /* Set PERCLK_CLK divider to 1 */
    CLOCK_SetDiv(kCLOCK_PerclkDiv, 0U);
    
    testInitADC2();
    testInitXbar();    
    testinitPitADC();
    
    testInitEtcAdc();
    
    /* enable the NVIC. */
    EnableIRQ( ADC_ETC_IRQ0_IRQn );
    EnableIRQ( ADC_ETC_ERROR_IRQ_IRQn );
    EnableIRQ( PIT_IRQn );
    
    /* start pit timer channel0. */
    PIT_StartTimer( PIT, kPIT_Chnl_0 );
    
}

void testInitADC2( void )
{
  
#if 1   /* from my etc demo */
    adc_config_t k_adcConfig;
    adc_channel_config_t adcChannelConfigStruct;
    
    ADC_GetDefaultConfig( &k_adcConfig );
    ADC_Init( ADC1, &k_adcConfig );
    ADC_EnableHardwareTrigger( ADC1, true );

    adcChannelConfigStruct.channelNumber = 10U; /* External channel selection from ADC_ETC. */
    adcChannelConfigStruct.enableInterruptOnConversionCompleted = false;
    ADC_SetChannelConfig( ADC1, 0U, &adcChannelConfigStruct );
    
    adcChannelConfigStruct.channelNumber = 11U; /* External channel selection from ADC_ETC. */
    ADC_SetChannelConfig( ADC1, 1U, &adcChannelConfigStruct );

    adcChannelConfigStruct.channelNumber = 9U; /* External channel selection from ADC_ETC. */
    ADC_SetChannelConfig( ADC1, 2U, &adcChannelConfigStruct );

    adcChannelConfigStruct.channelNumber = 8U; /* External channel selection from ADC_ETC. */
    ADC_SetChannelConfig( ADC1, 3U, &adcChannelConfigStruct );
    
    adcChannelConfigStruct.channelNumber = 7U; /* External channel selection from ADC_ETC. */
    ADC_SetChannelConfig( ADC1, 4U, &adcChannelConfigStruct );

    adcChannelConfigStruct.channelNumber = 0U; /* External channel selection from ADC_ETC. */
    ADC_SetChannelConfig( ADC1, 5U, &adcChannelConfigStruct );

    adcChannelConfigStruct.channelNumber = 1U; /* External channel selection from ADC_ETC. */
    ADC_SetChannelConfig( ADC1, 6U, &adcChannelConfigStruct );
  
#else 
    
    adc_config_t adcConfig;
    adc_channel_config_t adcChannelConfigStruct;

    ADC_GetDefaultConfig(&adcConfig);
    
    ADC_Init(ADC2, &adcConfig);
    ADC_EnableHardwareTrigger(ADC2, true);
    
    adcChannelConfigStruct.enableInterruptOnConversionCompleted = false;
    adcChannelConfigStruct.channelNumber = 7U; //shoot through
    ADC_SetChannelConfig(ADC2, SHOOT_THROUGH_GROUP, &adcChannelConfigStruct);
    
    adcChannelConfigStruct.channelNumber = 8U; //paper takeup
    ADC_SetChannelConfig(ADC2, PAPER_TAKEUP_GROUP, &adcChannelConfigStruct);
     
    adcChannelConfigStruct.channelNumber = 1U; //low stock
    ADC_SetChannelConfig(ADC2, LOW_STOCK_GROUP, &adcChannelConfigStruct);

    adcChannelConfigStruct.channelNumber = 0U; //label taken
    ADC_SetChannelConfig(ADC2, LABEL_TAKEN_GROUP, &adcChannelConfigStruct);
   
    adcChannelConfigStruct.channelNumber = 10U; //head up
    ADC_SetChannelConfig(ADC2, HEAD_UP_GROUP, &adcChannelConfigStruct);
    
    adcChannelConfigStruct.channelNumber = 11U; //head dot
    ADC_SetChannelConfig(ADC2, HEAD_DOT_GROUP, &adcChannelConfigStruct);
    
    adcChannelConfigStruct.channelNumber = 9U; //head temp
    ADC_SetChannelConfig(ADC2, HEAD_TEMP_GROUP, &adcChannelConfigStruct);
#endif
    
    /* Do auto hardware calibration. */
    if( kStatus_Success != ADC_DoAutoCalibration( ADC1 ) ) {
        PRINTF("ADC_DoAutoCalibration(ADC1) Failed.\r\n");
    }  
}

void testInitEtcAdc( void )
{

#if 1   /* from my etc demo */
    adc_etc_config_t adcEtcConfig;
    adc_etc_trigger_config_t adcEtcTriggerConfig;
    adc_etc_trigger_chain_config_t adcEtcTriggerChainConfig;

    ADC_ETC_GetDefaultConfig( &adcEtcConfig );
    adcEtcConfig.XBARtriggerMask = 1U;          /* enable the external XBAR trigger0. */
    adcEtcConfig.enableTSCBypass = false;
    ADC_ETC_Init( ADC_ETC, &adcEtcConfig);

    /* Set the external XBAR trigger0 configuration. */
    adcEtcTriggerConfig.enableSyncMode      = false;
    adcEtcTriggerConfig.enableSWTriggerMode = false;
    adcEtcTriggerConfig.triggerChainLength  = 6U; 
    adcEtcTriggerConfig.triggerPriority     = 0U;
    adcEtcTriggerConfig.sampleIntervalDelay = 0U;
    adcEtcTriggerConfig.initialDelay        = 0U;
    ADC_ETC_SetTriggerConfig( ADC_ETC, 0U, &adcEtcTriggerConfig);
    
    /* Set the external XBAR trigger0 chain configuration. */
    adcEtcTriggerChainConfig.enableB2BMode       = true;
    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 0U; 
    adcEtcTriggerChainConfig.ADCChannelSelect = 10U; 
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_Done0InterruptEnable;
    ADC_ETC_SetTriggerChainConfig( ADC_ETC, 0U, 0U, &adcEtcTriggerChainConfig);
    
    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 1U;
    adcEtcTriggerChainConfig.ADCChannelSelect = 11U;  /* paper takeup */
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_Done0InterruptEnable; 
    ADC_ETC_SetTriggerChainConfig( ADC_ETC, 0U, 1U, &adcEtcTriggerChainConfig); 

    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 2U;
    adcEtcTriggerChainConfig.ADCChannelSelect = 1U;  /* low stock */
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_Done0InterruptEnable; 
    ADC_ETC_SetTriggerChainConfig( ADC_ETC, 0U, 2U, &adcEtcTriggerChainConfig); 

    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 3U; 
    adcEtcTriggerChainConfig.ADCChannelSelect = 0U; /* label taken */
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_Done0InterruptEnable; 
    ADC_ETC_SetTriggerChainConfig( ADC_ETC, 0U, 3U, &adcEtcTriggerChainConfig); 

    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 4U; 
    adcEtcTriggerChainConfig.ADCChannelSelect = 10U;  /* head up */
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_Done0InterruptEnable; 
    ADC_ETC_SetTriggerChainConfig( ADC_ETC, 0U, 4U, &adcEtcTriggerChainConfig); 

    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 5U; 
    adcEtcTriggerChainConfig.ADCChannelSelect = 11U;  /* head dot */
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_Done0InterruptEnable; 
    ADC_ETC_SetTriggerChainConfig( ADC_ETC, 0U, 5U, &adcEtcTriggerChainConfig); 
    
    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << 6U; 
    adcEtcTriggerChainConfig.ADCChannelSelect = 9U;  /* head temp */
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_Done0InterruptEnable; 
    ADC_ETC_SetTriggerChainConfig( ADC_ETC, 0U, 6U, &adcEtcTriggerChainConfig);   

  
#else   
    adc_etc_config_t adcEtcConfig;
    adc_etc_trigger_config_t adcEtcTriggerConfig;
    adc_etc_trigger_chain_config_t adcEtcTriggerChainConfig;
    
    /* Initialize the ADC_ETC. */
    ADC_ETC_GetDefaultConfig(&adcEtcConfig);
    adcEtcConfig.XBARtriggerMask = 16U; /* Enable the external XBAR trigger0 and trigger4. 17 */
    adcEtcConfig.enableTSCBypass = false; //without this bit set to 0, ADC2 will not work
    //adcEtcConfig.dmaMode = kADC_ETC_TrigDMAWithLatchedSignal;
    ADC_ETC_Init(ADC_ETC, &adcEtcConfig);

    /* Set the external XBAR trigger0 configuration. */
    adcEtcTriggerConfig.enableSyncMode = false; //sync mode makes trigger 0 and 4 fire together when trigger0 is tripped from PIT 
    adcEtcTriggerConfig.enableSWTriggerMode = false; //we want a hardware trigger
    adcEtcTriggerConfig.initialDelay = 0x60002000;
    adcEtcTriggerConfig.sampleIntervalDelay = 0x2000ffb0;
    adcEtcTriggerConfig.triggerPriority = 0x8f410001;
    
    /* Set the external XBAR trigger4 configuration. */
    adcEtcTriggerConfig.triggerChainLength  = 6; //chain starts at 0
    ADC_ETC_SetTriggerConfig( ADC_ETC, 4U, &adcEtcTriggerConfig );

    /* Set the external XBAR trigger chain configuration. */
    /*back2back mode fires the next trigger in the chain as soon as the previous 
    conversion is complete instead of waiting for the interval time*/
    adcEtcTriggerChainConfig.enableB2BMode = true; 
    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << SHOOT_THROUGH_GROUP; /* Select ADC_HC0 register to trigger. */
    adcEtcTriggerChainConfig.ADCChannelSelect = 0U; /* ADC_HC0 will be triggered to sample Corresponding channel. */
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_Done0InterruptEnable; /* Enable the Done0 interrupt. */                                 
    ADC_ETC_SetTriggerChainConfig(ADC_ETC, 4U, 0U, &adcEtcTriggerChainConfig); /* Configure the trigger0 chain0. */
    
    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << PAPER_TAKEUP_GROUP; /* Select ADC_HC1 register to trigger. */
    adcEtcTriggerChainConfig.ADCChannelSelect = 1U; /* ADC_HC1 will be triggered to sample Corresponding channel. */
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_Done0InterruptEnable; /* Enable the Done0 interrupt. */
    ADC_ETC_SetTriggerChainConfig(ADC_ETC, 4U, 1U, &adcEtcTriggerChainConfig); /* Configure the trigger0 chain1. */
    
    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << LOW_STOCK_GROUP; /* Select ADC_HC2 register to trigger. */
    adcEtcTriggerChainConfig.ADCChannelSelect = 2U; /* ADC_HC2 will be triggered to sample Corresponding channel. */
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_Done0InterruptEnable; /* Enable the Done0 interrupt. */
    ADC_ETC_SetTriggerChainConfig(ADC_ETC, 4U, 2U, &adcEtcTriggerChainConfig); /* Configure the trigger0 chain2. */
    
    /* Set the external XBAR trigger4 chain configuration. */
    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << LABEL_TAKEN_GROUP; /* Select ADC_HC3 register to trigger. */
    adcEtcTriggerChainConfig.ADCChannelSelect = 3U; /* ADC_HC3 will be triggered to sample Corresponding channel. */
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_Done1InterruptEnable; /* Enable the Done1 interrupt. */
    ADC_ETC_SetTriggerChainConfig(ADC_ETC, 4U, 3U, &adcEtcTriggerChainConfig); /* Configure the trigger4 chain0. */
    
    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << HEAD_UP_GROUP; /* Select ADC_HC4 register to trigger. */
    adcEtcTriggerChainConfig.ADCChannelSelect = 4U; /* ADC_HC4 will be triggered to sample Corresponding channel. */
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_Done1InterruptEnable; /* Enable the Done1 interrupt. */
    ADC_ETC_SetTriggerChainConfig(ADC_ETC, 4U, 4U, &adcEtcTriggerChainConfig); /* Configure the trigger4 chain1. */
    
    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << HEAD_DOT_GROUP; /* Select ADC_HC5 register to trigger. */
    adcEtcTriggerChainConfig.ADCChannelSelect = 5U; /* ADC_HC5 will be triggered to sample Corresponding channel. */
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_Done1InterruptEnable; /* Enable the Done1 interrupt. */
    ADC_ETC_SetTriggerChainConfig(ADC_ETC, 4U, 5U, &adcEtcTriggerChainConfig); /* Configure the trigger4 chain2. */
    
    adcEtcTriggerChainConfig.ADCHCRegisterSelect = 1U << HEAD_TEMP_GROUP; /* Select ADC_HC6 register to trigger. */
    adcEtcTriggerChainConfig.ADCChannelSelect = 6U; /* ADC_HC6 will be triggered to sample Corresponding channel. */
    adcEtcTriggerChainConfig.InterruptEnable = kADC_ETC_Done1InterruptEnable; /* Enable the Done1 interrupt. */
    ADC_ETC_SetTriggerChainConfig(ADC_ETC, 4U, 6U, &adcEtcTriggerChainConfig); /* Configure the trigger4 chain3. */
    
#endif    
  
}
#if 0
void testInitXbar( void )
{
    XBARA_Init( XBARA );
    /* configure the xbara signal connections. */ 
    XBARA_SetSignalsConnection( XBARA, kXBARA1_InputQtimer1Tmr1, kXBARA1_OutputIomuxXbarInout09 ); 
    /* connect pit timer output to adc1 conversion */
    XBARA_SetSignalsConnection( XBARA, kXBARA1_InputPitTrigger0, kXBARA1_OutputAdcEtcTrig00 );    
}

void testinitPitADC( void )
{
    pit_config_t pitConfig;

    /* init pit module */
    PIT_GetDefaultConfig( &pitConfig );
    PIT_Init( PIT, &pitConfig );

    /* Set timer period 100mS for adc conversion */
    PIT_SetTimerPeriod( PIT, kPIT_Chnl_0, USEC_TO_COUNT( 100000U, CLOCK_GetFreq( kCLOCK_OscClk ) ) );        

    PIT_EnableInterrupts( PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable );   
}

void adcEtc0HandleIsr( void )
{
    uint32_t flags = ADC_ETC_GetInterruptStatusFlags( ADC_ETC, kADC_ETC_Trg0TriggerSource  );
    
    ADC_ETC_ClearInterruptStatusFlags( ADC_ETC, kADC_ETC_Trg0TriggerSource, kADC_ETC_Done0StatusFlagMask );
    ADC_ETC_ClearInterruptStatusFlags( ADC_ETC, kADC_ETC_Trg0TriggerSource, kADC_ETC_Done1StatusFlagMask);
    ADC_ETC_ClearInterruptStatusFlags( ADC_ETC, kADC_ETC_Trg0TriggerSource, kADC_ETC_Done2StatusFlagMask);
    
    /* get trigger0 chain0 results. */
    adcChannels[0] = ADC_ETC_GetADCConversionValue( ADC_ETC, 0U, 0U);  
    adcChannels[1] = ADC_ETC_GetADCConversionValue( ADC_ETC, 0U, 1U);  
    adcChannels[2] = ADC_ETC_GetADCConversionValue( ADC_ETC, 0U, 2U);  
    adcChannels[3] = ADC_ETC_GetADCConversionValue( ADC_ETC, 0U, 3U);  
    adcChannels[4] = ADC_ETC_GetADCConversionValue( ADC_ETC, 0U, 4U);  
    adcChannels[5] = ADC_ETC_GetADCConversionValue( ADC_ETC, 0U, 5U);  
    adcChannels[6] = ADC_ETC_GetADCConversionValue( ADC_ETC, 0U, 6U);
    adcComplete_ = true;
    

    /* added for arm errata 838869 */
    SDK_ISR_EXIT_BARRIER;

}

void adcEtcErrorHandleIsr( void )
{
    uint32_t flags = ADC_ETC_GetInterruptStatusFlags( ADC_ETC, kADC_ETC_Trg0TriggerSource  );
    PRINTF( "adcEtcErrorHandleIsr: flags %d\r\n", flags );
    ADC_ETC_ClearInterruptStatusFlags( ADC_ETC, kADC_ETC_Trg4TriggerSource, kADC_ETC_ErrorStatusFlagMask );

    /* added for arm errata 838869 */
    SDK_ISR_EXIT_BARRIER;        
}


void pitTimerHandleIsr( void )
{
    PIT_ClearStatusFlags( PIT, kPIT_Chnl_0, kPIT_TimerFlag );
    
    /* added for arm errata 838869 */
    SDK_ISR_EXIT_BARRIER;
}

#endif


BaseType_t createUnitTest( UTests test)
{
    BaseType_t result;  
    /*create our thread */
    result = xTaskCreate( unitTestTask, "UnitTestTask", 
                          configMINIMAL_STACK_SIZE , NULL, uTest_task_PRIORITY, 
                         &utHandle_ );
    if( result == pdPASS ) {
        uTest_= _UT_SERIAL;
    }
    
    return result;          
}

static void unitTestTask( void *pvParameters )
{
    while( !suspend_ ) { 
        switch( uTest_ ) 
        {
            case _UT_SERIAL: {
                switch( usTest_ ) 
                {
                    case _UT_SUB_INIT: {
                        if( getPageChecksums( &utTestSums_ ) ) {
                            if( getSerialPrConfiguration( &utPTConfig_ ) ) {
                                if( getSerialConfiguration( &utWTConfig_ ) ) {
                                    usTest_  = _UT_CHECKSUM_READ;
                                } else {
                                   PRINTF("unitTestTask(): Failed -- reading  printer config! \r\n");
                                  uTest_ = _UT_END_SERIAL;                              
                                }
                            } else {
                                PRINTF("unitTestTask(): Failed -- reading  printer config! \r\n");
                                uTest_ = _UT_END_SERIAL;                              
                            }                          
                        } else {
                            PRINTF("unitTestTask(): Failed  -- _UT_SUB_INIT! \r\n");
                            uTest_ = _UT_END_SERIAL;
                        }
                        break;
                    }
                    case _UT_CHECKSUM_READ: {
                        while( rCksCnt_ <= 500000 ) {
                            if( getPageChecksums( &utSums_ ) ) {
                                if( ( utSums_.prConfigSum != utTestSums_.prConfigSum ) || 
                                    ( utSums_.wgConfigSum != utTestSums_.wgConfigSum ) ) {
                                    PRINTF("unitTestTask(): Failed   _UT_CHECKSUM_READ: %d\r\n", uTest_ );  
                                    taskYIELD();  
                                    break;
                                } else {
                                    rCksCnt_++;
                                    taskYIELD();
                                }
                            } else {
                                PRINTF("unitTestTask(): Failed to read page checksums: %d\r\n", rCksCnt_ ); 
                                break;
                            }
                        }
                        if( rCksCnt_ >= 500000 ) {
                            uTest_ = _UT_END_SERIAL;
                        } else {
                            usTest_ = _UT_CHECKSUM_WRITE;
                        }
                        rCksCnt_ = 0;                        
                        break;
                    }
                    case _UT_CHECKSUM_WRITE: {
                        utTestSums_.pagesum = 0;
                        utTestSums_.sysConfig = 0;
                        utTestSums_.USBDescSum = 0;
                        utTestSums_.prConfigSum = 0xaa55;
                        utTestSums_.wgConfigSum =0x55aa;
                        
                        setPageChecksums( &utTestSums_ );
                        
                        
                        break;
                    }
                    case _UT_PRINTER_CFG_READ: {
                      
                        break;
                    }
                    case _UT_PRINTER_CFG_WRITE: {
                        break;
                    }
                    case _UT_WEIGHER_CFG_READ: {
                      
                        break;
                    }
                    case _UT_WEIGHER_CFG_WRITE: {
                        break;
                    }
                    default: {
                        PRINTF("unitTestTask(): Unkown sub test: %d\r\n", uTest_ );
                    }
                }  
                break;
            }
            case _UT_WAIT_SERIAL: {
                break;
            }
            default: {
                PRINTF("unitTestTask(): Unkown test: %d\r\n", uTest_ );      
            }
        }
        taskYIELD();  
    }
    vTaskSuspend(NULL);
  
  
}



void print3InchLabel( void )
{
    while( 1 ) {
        
#define AVERY_XT_PRINT_TEST        
#ifdef AVERY_XT_PRINT_TEST
#ifdef AVERY_XT_MOTORS_TEST
   unitTestSpinMotors();
   powerOffMotors();
    while( 1 ) {   
        
        powerOnMotors();
        delay_uS(100);
        for( int i = 0; i < 609; i++ ) {       //2436
            stepMotors();
            delay_uS(800);
        }
        powerOffMotors();
        taskYIELD();
    }
#endif    
#endif    
       vTaskDelay( pdMS_TO_TICKS(500) ); 
#if 0        
        ICMessages msg;
        /* wait for cutter to tell us it's ready. */
        if( xQueueReceive( pIMsgQHandle_, &msg, portMAX_DELAY ) ) {           
            if( msg.generic.msgType == _I_CUTTER_READY_FOR_CMD ) {
                PRINTF("printerTask(): Cutter ready for cmd\r\n" );  
                msg.generic.msgType = 0;
            }
        }
#endif
        
        
        //extern const unsigned short testLabel2[];
       
        //memcpy( getImageBuffer(), &testLabel2[0], getImageBufferSize() );         

        initializeMotors( FORWARDM_ );            
        powerOnMotors();
        setHeadPower(1);
        
        createCheckerBoardLabel( 0, STEPS_PER_LENGTH3_00 );
//ifdef AVERY_XT_PRINT_TEST_SINGLE           
        /* set label alignment */              
        setLabelAlignment( 772, 772 );            
        setIndirectData( RAM_0, STEPS_PER_LENGTH3_00 ); 

        /* start the print engine  */
        GPT_StartTimer( ENGINE_TIMER_BASE );                  
        
        /* intiate generic printing command */
        initializeCmdSequence( 4, &currentStatus );

        /* wait until done... */
        while( (currentStatus.state != ENGINE_IDLE) || 
               ( !getPrintEngine()->linePrintDone ) ) { taskYIELD(); }
        
        /* eject the label */
        initializeCmdSequence( 5, &currentStatus );

        /* wait until eject done... */
        while( (currentStatus.state != ENGINE_IDLE) || 
               ( !getPrintEngine()->linePrintDone ) ) { taskYIELD(); }

//#define AVERY_XT_PRINT_TEST_CONTINUOUS
        /* stop engine to set print operation command 
        GPT_StopTimer( ENGINE_TIMER_BASE );*/
        powerOffMotors();
        setHeadPower( 0 );   
#if 0        
        /* command cutter to cut the label */
        ICutterGeneric imsg;
        imsg.msgType = _I_CUTTER_CUT_CMD;
        BaseType_t result = xQueueSend( pCRMsgQHandle_, (void *)&imsg, 0 );
        if( result != pdPASS ) {
            PRINTF("printerTask(): Failed to post Cutter message!\r\n" );       
        }
#endif  
        while( ( currentStatus.sensor & LABEL_TAKEN ) == LABEL_TAKEN ) { taskYIELD(); }
        /* wait a little bit before we start the next label      
        vTaskDelay( pdMS_TO_TICKS(3000) ); */
      }
      vTaskSuspend(NULL); 
}


void torqueTest( void )
{
  
    while( 1 ) {  
      
        PrCommand cmd;
        /* copy test table into reserved entry space */ 
        setTableTestCmds( _TESTPAPERTAKEUP );
        cmd.identifier = 0;     /* reserved index */
        addCmdToQueue( &cmd );

        initializeMotors( FORWARDM_ );            
        powerOnMotors();
        /* start the print engine  */
        GPT_StartTimer( ENGINE_TIMER_BASE );                  
        
        initializeCmdSequence( 0, &currentStatus );
        while( currentStatus.state == ENGINE_IDLE ) { taskYIELD(); }
        
        while( currentStatus.state != ENGINE_IDLE ) { taskYIELD(); }
        powerOffMotors();
        
        vTaskDelay( pdMS_TO_TICKS( 1000 ) );
    }  
}

void testSizing( void )
{
  
}

void testGapDetection( void )
{
    /* testing shoot through sensor preformance */
    while( 1 ) {
        
        PrCommand cmd;
        /* copy test table into reserved entry space */ 
        setTableTestCmds( _TESTGAP );
        cmd.identifier = 0;     /* reserved index */
        addCmdToQueue( &cmd );

        initializeMotors( FORWARDM_ );
        powerOnMotors();
                
        /* start the print engine  */
        GPT_StartTimer( ENGINE_TIMER_BASE );                  
        
        initializeCmdSequence( 0, &currentStatus );
        while( currentStatus.state == ENGINE_IDLE ) { taskYIELD(); }
        
        while( currentStatus.state != ENGINE_IDLE ) { taskYIELD(); }
        powerOffMotors();
          
        vTaskDelay( pdMS_TO_TICKS( 1000 ) );
    }
  
}






#if 0
bool here = false;
lpspi_master_handle_t lpspi2CSHandle;

extern CS5530Mgr                       cs5530Mgr_;
void testCSADCInt( void )
{
    lpspi_master_config_t masterConfig;
    
    LPSPI_MasterGetDefaultConfig( &masterConfig );
   
    /* interface is now claimed. we can safely use. */
    masterConfig.baudRate                      = CS5530_MAX_CLK_FREQ; //500000U;
    masterConfig.bitsPerFrame                  = 8;
    masterConfig.cpol                          = kLPSPI_ClockPolarityActiveHigh;
    masterConfig.cpha                          = kLPSPI_ClockPhaseFirstEdge;
    masterConfig.direction                     = kLPSPI_MsbFirst;
    //masterConfig.pcsToSckDelayInNanoSec        = 500;             
    //masterConfig.lastSckToPcsDelayInNanoSec    = 500;
    masterConfig.betweenTransferDelayInNanoSec = 1000000000;
    masterConfig.whichPcs                      = kLPSPI_Pcs0;
    masterConfig.pcsActiveHighOrLow            = kLPSPI_PcsActiveLow;
    masterConfig.pinCfg                        = kLPSPI_SdiInSdoOut;
    masterConfig.dataOutConfig                 = kLpspiDataOutTristate;


    /* configured as lpspi2 interface */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_13_LPSPI2_SDI, 0U );
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_13_LPSPI2_SDI, 0x10B0U ); 
    
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_12_LPSPI2_SDO, 0U );                                    
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_12_LPSPI2_SDO, 0x10B0U );                                 
    
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_11_LPSPI2_PCS0, 0U );
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_11_LPSPI2_PCS0, 0x10B0U ); 
 
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_10_LPSPI2_SCK, 0U );
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_10_LPSPI2_SCK, 0x10B0U ); 

    /* initialize the spi master */
    LPSPI_MasterInit( LPSPI2, &masterConfig, ( CLOCK_GetFreq(kCLOCK_Usb1PllPfd0Clk) / 8 ) );
    /* setup our interrupts */
    LPSPI_EnableInterrupts( LPSPI2, ( kLPSPI_TxInterruptEnable | 
                               kLPSPI_RxInterruptEnable | 
                               kLPSPI_TransmitErrorInterruptEnable | 
                               kLPSPI_ReceiveErrorInterruptEnable ) );

    /* create interrupt handle */
    LPSPI_MasterTransferCreateHandle( LPSPI2, &lpspi2CSHandle, (lpspi_master_transfer_callback_t)lpspiCSCallback, NULL );
    EnableIRQ( LPSPI2_IRQn );
    NVIC_SetPriority( LPSPI2_IRQn, WEIGHER_SPI_INT_PRIORITY );
    
    LPSPI_Enable( LPSPI2, false );
    LPSPI2->CFGR1 &= (~LPSPI_CFGR1_NOSTALL_MASK);
    LPSPI_Enable( LPSPI2, true );  
    
    
    memset( &cs5530Mgr_, 0, sizeof( cs5530Mgr_ ) );
    /* setup sync transfer */
    buildSyncTransfer( &cs5530Mgr_ );
    /* sync the interface of the cs5530 */
    if( cs5530SyncSerialPort( &cs5530Mgr_ ) ) {
        /* setup the reset transfer */
        buildResetTransfer( &cs5530Mgr_ );
        /* reset the cs5530 */
        if( cs5530Reset( &cs5530Mgr_ ) ) {
            /* cs5530 requires 8 clocks to fully reset p13 of data sheet */
            startWaitTimer( 5 );        
            buildReleaseRstTransfer( &cs5530Mgr_ );
            /* release the soft reset */
            if( cs5530Reset( &cs5530Mgr_ ) ) {
                /* validate reset */
                cs5530ReadCfgResiter( &cs5530Mgr_ );
                if( isResetValid( &cs5530Mgr_ ) ) {
                    PRINTF("initializeCs5530(): CS5530 initialized!\r\n");   
                }
            } else {
                PRINTF("initializeCs5530(): failed to release CS5530 from reset!\r\n");        
            }
        } else {
            PRINTF("initializeCs5530(): failed to reset the CS5530!\r\n");        
        }
    } else {
        PRINTF("initializeCs5530(): failed to synchronize interface!\r\n");        
    }    
}

void lpspiCSCallback( LPSPI_Type *base, lpspi_master_handle_t *handle, status_t status, void *userData )
{
    here = true;
    cs5530SpiCallBack();    
}
#endif

void testStrobePWM( void )
{
    pwm_config_t pwmConfig;
    pwm_signal_param_t pwmSignal; 
    pwm_fault_param_t faultConfig;
    uint32_t pwmFrequency = 25265U;
    
    CLOCK_SetDiv( kCLOCK_AhbDiv, 0x2 );
    CLOCK_SetDiv( kCLOCK_IpgDiv, 0x3 );


         GPIO_WritePinOutput( PHEAD_STROBE_EN_GPIO, PHEAD_STROBE_EN_PIN, false ); 
#if 0
         GPIO_WritePinOutput(PHEAD_STROBE_B_GPIO, PHEAD_STROBE_B_PIN, false );
         GPIO_WritePinOutput( PHEAD_STROBE_A_GPIO, PHEAD_STROBE_A_PIN, true );
    
        GPIO_WritePinOutput(PHEAD_STROBE_B_GPIO, PHEAD_STROBE_B_PIN, true);
        GPIO_WritePinOutput( 
                            PHEAD_STROBE_A_GPIO, PHEAD_STROBE_A_PIN, false );
#endif

    /* set deadtime */
    uint16_t deadTimeVal = 0;
    
    /* disable GPIO control of the pin select ftm0 pwm control for printhead strobe */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_38_FLEXPWM2_PWMA00, 0 );
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_38_FLEXPWM2_PWMA00, 0x10B0U ); 

    PWM_GetDefaultConfig( &pwmConfig );
         
   
    /* use full cycle reload */
    pwmConfig.reloadLogic               = kPWM_ReloadImmediate;    
    pwmConfig.pairOperation             = kPWM_Independent;
    pwmConfig.enableDebugMode           = true;
    pwmConfig.clockSource               = kPWM_Submodule0Clock;
    pwmConfig.prescale                  = kPWM_Prescale_Divide_16;
    pwmConfig.initializationControl     = kPWM_Initialize_MasterSync;

    /* initialize submodule 0 */
    if( PWM_Init( PWM2, kPWM_Module_0, &pwmConfig ) == kStatus_Fail ) {
        PRINTF("testPWMWeigher(): PWM_Init Failed.\r\n");    
    }

    
    pwmSignal.pwmChannel                = kPWM_PwmA;    
    pwmSignal.level                     = kPWM_HighTrue;
    pwmSignal.dutyCyclePercent          = 50;
    pwmSignal.deadtimeValue             = deadTimeVal;
    pwmSignal.faultState                = kPWM_PwmFaultState0;
    
    PWM_SetupPwm( PWM2, kPWM_Module_0, &pwmSignal, 1, kPWM_SignedCenterAligned, pwmFrequency,
                 CLOCK_GetFreq( kCLOCK_IpgClk ) );
    
    
    
    /* set the load okay bit for submodule to load register from their buffer */
    PWM_SetPwmLdok( PWM2, kPWM_Control_Module_0, true );
    PWM2->SM[kPWM_Module_0].DISMAP[kPWM_Module_0] = 0x00;
    
    /* start the pwm generation from submodules 0*/
    PWM_StartTimer( PWM2, kPWM_Control_Module_0 );    
    while(1) {
        asm("nop");
    }
      
}

void initEdma( void )
{
    lpspi_master_config_t masterConfig;
#if 0    
    edma_config_t edmaConfig;
    
    /* intialize the dma mux */
    DMAMUX_Init( DMAMUX );
    /* setup the dma channel 0 for source 79: lpspi4 rx this channel will not be used but needs setup! */
    DMAMUX_SetSource( DMAMUX, 0, kDmaRequestMuxLPSPI4Rx );
    DMAMUX_EnableChannel( DMAMUX, 0 );

    /* setup the dma channel 1 for source 80: lpspi4 tx  */
    DMAMUX_SetSource( DMAMUX, 1, kDmaRequestMuxLPSPI4Tx );
    DMAMUX_EnableChannel( DMAMUX, 1 );
    
    /* edma already intialized in sensors.c  */
    EDMA_GetDefaultConfig( &edmaConfig );   
    EDMA_Init( DMA0, &edmaConfig );
       
    /* setup spi4 master configuration kjt-56 max clock rate 12Mhz */     
    masterConfig.baudRate                      = 12000000u; /* 10Mhz */
    masterConfig.bitsPerFrame                  = 64;        
    masterConfig.cpol                          = kLPSPI_ClockPolarityActiveHigh;        
    masterConfig.cpha                          = kLPSPI_ClockPhaseFirstEdge;

    masterConfig.direction                 = kLPSPI_MsbFirst; 
    masterConfig.pcsToSckDelayInNanoSec        = 50;
    masterConfig.lastSckToPcsDelayInNanoSec    = 50;
    masterConfig.betweenTransferDelayInNanoSec = 50;
    masterConfig.whichPcs                                 = kLPSPI_Pcs0;
    masterConfig.pcsActiveHighOrLow                       = kLPSPI_PcsActiveLow;
    masterConfig.pinCfg                                   = kLPSPI_SdiInSdoOut;    //kLPSPI_SdoInSdiOut//kLPSPI_SdiInSdoOut; -- switch on schematic!!
    masterConfig.dataOutConfig                            = kLpspiDataOutTristate;

    /* intialize the spi interface */
    LPSPI_MasterInit( LPSPI4, &masterConfig, ( CLOCK_GetFreq(kCLOCK_Usb1PllPfd0Clk) / 8 ) );
    NVIC_SetPriority( LPSPI4_IRQn, 1 );
    
    GPIO_WritePinOutput( ACCEL_INTB_GPIO, ACCEL_INTB_PIN, true );
    memset( &eDmaHandle_0, 0, sizeof(edma_handle_t) );
    memset( &eDmaHandle_1, 0, sizeof(edma_handle_t) );
    
    /* create handles for all three dma channels */ 
    EDMA_CreateHandle( &eDmaHandle_0, DMA0, 0 ); 
    EDMA_CreateHandle( &eDmaHandle_1, DMA0, 1 );
    
    LPSPI_MasterTransferCreateHandleEDMA( LPSPI4, &spi4HeadMasterHandle, LPSPI4_MasterUserCallback, 
                                         NULL, &eDmaHandle_0, &eDmaHandle_1 );
#else 

    /*Master config*/
    masterConfig.baudRate                      = 12000000u; /* 10Mhz */
    masterConfig.bitsPerFrame                  = 64;        
    masterConfig.cpol                          = kLPSPI_ClockPolarityActiveHigh;        
    masterConfig.cpha                          = kLPSPI_ClockPhaseFirstEdge;

    masterConfig.direction                 = kLPSPI_MsbFirst; 
    masterConfig.pcsToSckDelayInNanoSec        = 50;
    masterConfig.lastSckToPcsDelayInNanoSec    = 50;
    masterConfig.betweenTransferDelayInNanoSec = 50;
    masterConfig.whichPcs                                 = kLPSPI_Pcs0;
    masterConfig.pcsActiveHighOrLow                       = kLPSPI_PcsActiveLow;
    masterConfig.pinCfg                                   = kLPSPI_SdiInSdoOut;    //kLPSPI_SdoInSdiOut//kLPSPI_SdiInSdoOut; -- switch on schematic!!
    masterConfig.dataOutConfig                            = kLpspiDataOutTristate;

    /* intialize the spi interface */
    LPSPI_MasterInit( LPSPI4, &masterConfig, ( CLOCK_GetFreq(kCLOCK_Usb1PllPfd0Clk) / 8 ) );
    NVIC_SetPriority( LPSPI4_IRQn, 1 );
    LPSPI_MasterTransferCreateHandle( LPSPI4, &spi4ManualHandle, LPSPI4_MMasterUserCallback, NULL );
#endif    
}

void testPrintHeadTransfer( void )
{
    lpspi_transfer_t masterXfer;  
    /* added to debug data out */
    unsigned char bfr[56] = { 0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,
                            0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,
                            0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55};

#if 0 
    /* setup master transfer */
    masterXfer.txData = &bfr[0];
    masterXfer.rxData = NULL;
    masterXfer.dataSize = 0x38;
    masterXfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous;
    spiDmaTransferComplete = false;    
    
    
    GPIO_WritePinOutput( ACCEL_INTA_GPIO, ACCEL_INTA_PIN, false );
    unsigned long result  = LPSPI_MasterTransferEDMA( LPSPI4, &spi4HeadMasterHandle, &masterXfer );    
    GPIO_WritePinOutput( ACCEL_INTA_GPIO, ACCEL_INTA_PIN, true );
    if( kStatus_Success != result ) {
        PRINTF("testPrintHeadTransfer(): Dma transfer failed. critical error: %d \r\n", result );
    }
//    while( !spiDmaTransferComplete ) {
//        asm("nop");
//    }
    
#else
        spiDmaTransferComplete = false;    
        masterXfer.txData   = &bfr[0];
        masterXfer.rxData   = NULL;
        masterXfer.dataSize = 0x38;
        masterXfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous;
        GPIO_WritePinOutput( ACCEL_INTA_GPIO, ACCEL_INTA_PIN, false );    
        unsigned long result = LPSPI_MasterTransferNonBlocking( LPSPI4, &spi4ManualHandle, &masterXfer);
        if( kStatus_Success != result ) {
            while( !spiDmaTransferComplete ) {
                asm("nop");
            }
        }
        GPIO_WritePinOutput( ACCEL_INTA_GPIO, ACCEL_INTA_PIN, true );

#endif
}
#if 1
/******************************************************************************/
/*!   \fn void DSPI_MasterUserCallback( SPI_Type *base, 
                                        dspi_master_edma_handle_t *handle, 
                                        status_t status, void *userData )
      \brief
        This function is called after one complete dma transfer has occured.
       
      \author
          Aaron Swift
*******************************************************************************/

void LPSPI4_MasterUserCallback( LPSPI_Type *base, lpspi_master_edma_handle_t *handle, status_t status, void *userData )
{
    if( status == kStatus_Success ) {
        spiDmaTransferComplete = true;
    } else {
        PRINTF("LPSPI4_MasterUserCallback(): Dma transfer failed. critical error!\r\n" );
    }
}
#endif

void LPSPI4_MMasterUserCallback( LPSPI_Type *base, lpspi_master_handle_t *handle, status_t status, void *userData )
{
   if( status == kStatus_Success ) {
        spiDmaTransferComplete = true;
    }      
}
/******************************************************************************/
/*!   \fn void testPrintEngineTimer( void )

      \brief
        This function test if general purpose timer can support print engine.
        We simulate an actual line burn time and verified the timer will support  
        our print engine. 
 
        gpt frequency is 1.953Mhz = .512uSec
        
        slt time:               756uS
        history time:           151uS
        current line time:      295uS
        pwm start time:         426uS

      \author
          Aaron Swift
*******************************************************************************/
void testPrintEngineTimer( void )
{
    gpt_config_t gptConfig;
    
    GPT_GetDefaultConfig( &gptConfig );
    
    gptConfig.enableFreeRun = true;
    GPIO_WritePinOutput( ACCEL_INTB_GPIO, ACCEL_INTB_PIN, true );
    /* initialize gpt module */
    GPT_Init( GPT2, &gptConfig );    
    GPT_SetClockDivider( GPT2, 32 );

    /* get gpt clock frequency */
    uint32_t freq = CLOCK_GetFreq( kCLOCK_PerClk );

    /* gpt frequency is 1.953Mhz = .512uSec */
  
    /* set channel 1 for slt time of 756uS duration */
    GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel1, 4603 ); 
    /* set channel 2 for history time of 151uS duration */
    GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel2, 295 ); 
    /* set channel 3 for current line time of 295uS duration */
    GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel3, 576 ); 

    GPT_SetOutputOperationMode( GPT2, kGPT_OutputCompare_Channel1, kGPT_OutputOperation_Disconnected );
    GPT_SetOutputOperationMode( GPT2, kGPT_OutputCompare_Channel2, kGPT_OutputOperation_Disconnected );
    GPT_SetOutputOperationMode( GPT2, kGPT_OutputCompare_Channel3, kGPT_OutputOperation_Disconnected );
    
    /* enable gpt output compare1 interrupt */
    GPT_EnableInterrupts( GPT2, kGPT_OutputCompare1InterruptEnable );
    GPT_EnableInterrupts( GPT2, kGPT_OutputCompare2InterruptEnable );
    GPT_EnableInterrupts( GPT2, kGPT_OutputCompare3InterruptEnable );

    
    EnableIRQ( GPT2_IRQn );
    /* mark start of line burn */

    initEdma();
    testPrintHeadTransfer();
    GPIO_WritePinOutput( ACCEL_INTB_GPIO, ACCEL_INTB_PIN, false );
    GPT_StartTimer( GPT2 ); 
    //while(1) {;}
    
}

/******************************************************************************/
/*!   \fn void testPrintEngineTimer( void )

      \brief
        This function handles the general purpose timer interrupts.
        Compare 2 is dual purpose history and pwm start time.

      \author
          Aaron Swift
*******************************************************************************/
void generalTimerIsr( void )
{
    static bool strobe_ = false;
    if( kGPT_OutputCompare1Flag == GPT_GetStatusFlags( GPT2, kGPT_OutputCompare1Flag ) ) {
            GPIO_WritePinOutput( ACCEL_INTB_GPIO, ACCEL_INTB_PIN, true );            
            
            /* clear interrupt flag.*/
            GPT_ClearStatusFlags( GPT2, kGPT_OutputCompare1Flag );      
            GPT_DisableInterrupts( GPT2, kGPT_OutputCompare1Flag );
           
    }
    
    if( kGPT_OutputCompare2Flag == GPT_GetStatusFlags( GPT2, kGPT_OutputCompare2Flag ) ) {
            if( !strobe_ ) {
                /* clear interrupt flag.*/
                GPT_ClearStatusFlags( GPT2, kGPT_OutputCompare2Flag );
                //GPT_DisableInterrupts( GPT2, kGPT_OutputCompare2Flag );
             
                GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel2, 832 ); 
            } else {
                /* clear interrupt flag.*/
                GPT_ClearStatusFlags( GPT2, kGPT_OutputCompare2Flag );
                GPT_DisableInterrupts( GPT2, kGPT_OutputCompare2Flag );            
            }
    }

    if( kGPT_OutputCompare3Flag == GPT_GetStatusFlags( GPT2, kGPT_OutputCompare3Flag ) ) {          
            
            /* clear interrupt flag.*/
            GPT_ClearStatusFlags( GPT2, kGPT_OutputCompare3Flag ); 
            GPT_DisableInterrupts( GPT2, kGPT_OutputCompare3Flag );
               
    }   
    
    /* added for arm errata 838869 */
    SDK_ISR_EXIT_BARRIER;    
}

#if 0
void testADCAuto( void )
{
    adc_etc_config_t adcEtcCfg;
    adc_etc_trigger_config_t adcEtcTrgrCfg;
    adc_etc_trigger_chain_config_t adcEtcTrgrChainCfg;
    
    initADC0( mode_ );  
    if( mode_ == _AUTO ) {
        initXbar();
        initPitADC();
    }
    
    ADC_ETC_GetDefaultConfig( &adcEtcCfg );
    
    /* enable the external xbar trigger0. */
    adcEtcCfg.XBARtriggerMask = 1U;     
    ADC_ETC_Init( ADC_ETC, &adcEtcCfg );

    /* Set the external XBAR trigger0 configuration. */
    adcEtcTrgrCfg.enableSyncMode      = false;
    if( mode_ == _AUTO ) {
        adcEtcTrgrCfg.enableSWTriggerMode = false;
    } else {
        adcEtcTrgrCfg.enableSWTriggerMode = true;
    }
    /* not channels but number of chains which is 3 */
    adcEtcTrgrCfg.triggerChainLength  = 3; 
    adcEtcTrgrCfg.triggerPriority     = 0U;
    adcEtcTrgrCfg.sampleIntervalDelay = 0U;
    adcEtcTrgrCfg.initialDelay        = 0U;
    ADC_ETC_SetTriggerConfig( ADC_ETC, 0U, &adcEtcTrgrCfg );
    
    /* set the external xbar trigger 0 chain configuration. */
    if( mode_ == _AUTO ) {
        adcEtcTrgrChainCfg.enableB2BMode       = true;
    } else {
        adcEtcTrgrChainCfg.enableB2BMode       = false;
    }
    
    /* select adc_hc0 register to trigger. */
    adcEtcTrgrChainCfg.ADCHCRegisterSelect = 1U  << 0;                                                   
    /* adc_hc0 trigger used to sample adc0 channel 10 */  
    adcEtcTrgrChainCfg.ADCChannelSelect = 10U;    
    adcEtcTrgrChainCfg.InterruptEnable = kADC_ETC_Done0InterruptEnable; 
    /* configure the trigger 0 chain0. */
    ADC_ETC_SetTriggerChainConfig ( ADC_ETC, 0U, 0U, &adcEtcTrgrChainCfg ); 
   
    /* select adc_hc1 register to trigger. */
    adcEtcTrgrChainCfg.ADCHCRegisterSelect = 1U << 1; 
    /* adc_hc1 trigger used to sample adc0 channel 11 */  
    adcEtcTrgrChainCfg.ADCChannelSelect = 11U;        
    adcEtcTrgrChainCfg.InterruptEnable = kADC_ETC_InterruptDisable; 
    /* configure the trigger 0 chain1. */
    ADC_ETC_SetTriggerChainConfig( ADC_ETC, 0U, 1U, &adcEtcTrgrChainCfg ); 
                                  
    adcEtcTrgrChainCfg.ADCHCRegisterSelect = 1U << 2; 
    /* adc_hc2 trigger used to sample adc0 channel 12 */  
    adcEtcTrgrChainCfg.ADCChannelSelect = 12U;        
    adcEtcTrgrChainCfg.InterruptEnable = kADC_ETC_InterruptDisable; 
    /* configure the trigger 0 chain1. */
    ADC_ETC_SetTriggerChainConfig( ADC_ETC, 0U, 2U, &adcEtcTrgrChainCfg ); 

    adcEtcTrgrChainCfg.ADCHCRegisterSelect = 1U << 3; 
    /* adc_hc3 trigger used to sample adc0 channel 13*/  
    adcEtcTrgrChainCfg.ADCChannelSelect = 13U;        
    adcEtcTrgrChainCfg.InterruptEnable = kADC_ETC_InterruptDisable; 
    /* configure the trigger 0 chain1. */
    ADC_ETC_SetTriggerChainConfig( ADC_ETC, 0U, 3U, &adcEtcTrgrChainCfg ); 
   
    /* Enable the NVIC. */
    EnableIRQ( ADC_ETC_IRQ0_IRQn );
    EnableIRQ( ADC_ETC_ERROR_IRQ_IRQn );
    
    if( mode_ == _AUTO ) {       
        /* Start PIT channel0. */
        PIT_StartTimer( PIT, kPIT_Chnl_0 );    
        //while(1) { asm("nop"); }
    } else {
        while(1) {            
            adcComplete_ = false;
            /* trigger conversion */
            ADC_ETC_DoSoftwareTrigger( ADC_ETC, 0U );
            while( !adcComplete_) { asm("nop"); }
           
        }
    }   
}

void initADC0( ADCMode mode )
{
    
    adc_config_t adcConfig;
    adc_channel_config_t adcChanCfg;

    /* setup adc clock */
    CLOCK_SetMux( kCLOCK_PerclkMux, 1U );    
    CLOCK_SetDiv( kCLOCK_PerclkDiv, 0U );
    
    ADC_GetDefaultConfig( &adcConfig );
    
    if( mode == _MANUAL ) {                
        /* initialize the adc module for manual conversion */
        //adcConfig.enableContinuousConversion = true;
        ADC_Init( ADC1, &adcConfig );
        ADC_EnableHardwareTrigger( ADC1, true );

        adcChanCfg.channelNumber = 16U;
        adcChanCfg.enableInterruptOnConversionCompleted = false;
        ADC_SetChannelConfig( ADC1, 0, &adcChanCfg );
        ADC_SetChannelConfig( ADC1, 1, &adcChanCfg );    
        ADC_SetChannelConfig( ADC1, 2, &adcChanCfg );
        ADC_SetChannelConfig( ADC1, 3, &adcChanCfg );            
        
    } else {
                
        /* initialize the adc module for auto conversion */
        ADC_Init( ADC1, &adcConfig );
        ADC_EnableHardwareTrigger( ADC1, true );
        
        adcChanCfg.channelNumber = 16U;         
        adcChanCfg.enableInterruptOnConversionCompleted = false;
        ADC_SetChannelConfig( ADC1, 0, &adcChanCfg );
        ADC_SetChannelConfig( ADC1, 1, &adcChanCfg );    
        ADC_SetChannelConfig( ADC1, 2, &adcChanCfg );
        ADC_SetChannelConfig( ADC1, 3, &adcChanCfg );            
    }

    /* auto hardware calibration. */
    if( kStatus_Success == ADC_DoAutoCalibration( ADC1 ) ) {
        PRINTF("ADC_DoAutoCalibration() Done.\r\n");
    } else {
        PRINTF("ADC_DoAutoCalibration() Failed.\r\n");
    }                        
}

void initXbar( void )
{
    XBARA_Init( XBARA );

    /* configure the xbara signal connections. */
    XBARA_SetSignalsConnection( XBARA, kXBARA1_InputPitTrigger0, kXBARA1_OutputAdcEtcTrig00 );
    
}

void initPitADC( void )
{
    pit_config_t pitConfig;

    /* init pit module */
    PIT_GetDefaultConfig( &pitConfig );
    PIT_Init( PIT, &pitConfig );

    /* Set timer period 100mS for channel 4 adc conversion */
    PIT_SetTimerPeriod( PIT, kPIT_Chnl_0, MSEC_TO_COUNT( 100U, CLOCK_GetFreq( kCLOCK_OscClk ) ) );        

    PIT_EnableInterrupts( PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable );   
    EnableIRQ(PIT_IRQn);

}


#if 0
void adc1HandleIsr( void )
{
    uint32_t chFlag = ADC_GetChannelStatusFlags( ADC1, 0 );        
    //if( chFlag )
    
    SDK_ISR_EXIT_BARRIER;
}
#endif
/******************************************************************************/
/*!   \fn void testPrintEngineTimer( void )

      \brief
        This function handles the general purpose timer interrupts.
        Compare 2 is dual purpose history and pwm start time.

      \author
          Aaron Swift
*******************************************************************************/
void testWeigherSpi( void )
{
    setConversionInterrupt();
    startWeigherClock();
    while(1) { asm("nop"); }
    
}

/******************************************************************************/
/*!   \fn void startWeigherClock( void )

      \brief
        This function sets up and starts a free running pwm clk 2.45Mhz for the 
        weigher. This function is used to evaluate gpio isr / spi weigher. 


      \author
          Aaron Swift
*******************************************************************************/
void startWeigherClock( void )
{
    pwm_config_t pwmConfig;
    pwm_signal_param_t pwmSignal; 
    pwm_fault_param_t faultConfig;
    uint32_t pwmFrequency = 2450000UL;
    
    CLOCK_SetDiv( kCLOCK_AhbDiv, 0x2 );
    CLOCK_SetDiv( kCLOCK_IpgDiv, 0x3 );

    /* set deadtime */
    uint16_t deadTimeVal = 0;
    PWM_GetDefaultConfig( &pwmConfig );
         
   
    /* use full cycle reload */
    pwmConfig.reloadLogic               = kPWM_ReloadPwmFullCycle;    
    pwmConfig.pairOperation             = kPWM_Independent;
    pwmConfig.enableDebugMode           = true;
    pwmConfig.clockSource               = kPWM_Submodule0Clock;
    pwmConfig.prescale                  = kPWM_Prescale_Divide_1;
    pwmConfig.initializationControl     = kPWM_Initialize_MasterSync;

    /* initialize submodule 0 */
    if( PWM_Init( PWM1, kPWM_Module_1, &pwmConfig ) == kStatus_Fail ) {
        PRINTF("testPWMWeigher(): PWM_Init Failed.\r\n");    
    }

    
    pwmSignal.pwmChannel                = kPWM_PwmA;    
    pwmSignal.level                     = kPWM_HighTrue;
    pwmSignal.dutyCyclePercent          = 50;
    pwmSignal.deadtimeValue             = deadTimeVal;
    pwmSignal.faultState                = kPWM_PwmFaultState0;
    
    PWM_SetupPwm( PWM1, kPWM_Module_0, &pwmSignal, 1, kPWM_SignedCenterAligned, pwmFrequency,
                 CLOCK_GetFreq( kCLOCK_IpgClk ) );
    
    
    
    /* set the load okay bit for submodule to load register from their buffer */
    PWM_SetPwmLdok( PWM1, kPWM_Control_Module_0, true );

    /* start the pwm generation from submodules 0*/
    PWM_StartTimer( PWM1, kPWM_Control_Module_0 );    
}

/******************************************************************************/
/*!   \fn void startWeigherClock( void )

      \brief
        This function sets gpio for external interrupt. 
        This function is used to evaluate gpio isr / spi weigher. 

      \author
          Aaron Swift
*******************************************************************************/
void setConversionInterrupt( void )
{
    gpio_pin_config_t gpioConfig;
    
    gpioConfig.direction        = kGPIO_DigitalInput;
    gpioConfig.outputLogic      = 0;
    gpioConfig.interruptMode    = kGPIO_IntFallingEdge;
    
    /* intialize the spi miso pin as gpio input interrupt for data ready signal */
    IOMUXC_SetPinMux( IOMUXC_GPIO_AD_B0_13_GPIO1_IO13, 0U );
    IOMUXC_SetPinConfig( IOMUXC_GPIO_AD_B0_13_GPIO1_IO13, 0x70A0U );
    GPIO_PinInit( WEIGHER_DATA_RDY_GPIO, WEIGHER_DATA_RDY_PIN, &gpioConfig );
    
    /* enable the interrupt */
    GPIO_PortEnableInterrupts( WEIGHER_DATA_RDY_GPIO, 1U << WEIGHER_DATA_RDY_PIN );
    EnableIRQ( GPIO1_Combined_0_15_IRQn );
}

/******************************************************************************/
/*!   \fn void startWeigherClock( void )

      \brief
        This function handles the gpio interrupt from the weigher.  
        This function is used to evaluate gpio isr / spi weigher. 

      \author
          Aaron Swift
*******************************************************************************/
void weigherDataRdyIsr( void )
{
    uint8_t dummyBytes[3] = { 0x00, 0x0B, 0x00 };
    uint8_t cnts[3] = { 0 };
    uint32_t result = 0, msb = 0, mid = 0, lsb = 0;
    
    GPIO_PortClearInterruptFlags( WEIGHER_DATA_RDY_GPIO, 1U << WEIGHER_DATA_RDY_PIN ); 
    DisableIRQ( GPIO5_Combined_0_15_IRQn );
    /* ensure that data ready signal is low before clocking data out */
    if( GPIO_ReadPinInput( WEIGHER_DATA_RDY_GPIO, WEIGHER_DATA_RDY_PIN ) == false ) {
        /* setup spi for data transfer */
        intializeSpiForWeigher();
        /* setup the master transfer */
        lpspi_transfer_t mXfer;
        mXfer.txData   = &dummyBytes[0];
        mXfer.rxData   = &cnts[0];
        mXfer.dataSize = sizeof(dummyBytes);
        mXfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous | kLPSPI_MasterByteSwap;
        
        /* get the weigher counts */
        LPSPI_MasterTransferBlocking( LPSPI1, &mXfer );
    
        /* byte order */
        msb  = (uint32_t)cnts[0];
        msb = msb << 16;
        
        mid = (uint32_t)cnts[1];
        mid = mid << 8;
        
        lsb  = (uint32_t)cnts[2];
        
        result =  ( msb | mid | lsb );
       
        /* strip off pos/neg bit indicator. see AD7191 datasheet */    
        result = result & 0x7fffff;                  
    }
    setConversionInterrupt();
    
    /* added for arm errata 838869 */
    SDK_ISR_EXIT_BARRIER;
}

/******************************************************************************/
/*!   \fn void startWeigherClock( void )

      \brief
        This function sets up the spi interface to the external weigher adc.  
        This function is used to evaluate gpio isr / spi weigher. 

      \author
          Aaron Swift
*******************************************************************************/
void intializeSpiForWeigher( void )
{
    lpspi_master_config_t lpspiConfig;
    
    /*set clock source for lpspi1*/
    CLOCK_SetMux( kCLOCK_LpspiMux, 1 );
    CLOCK_SetDiv( kCLOCK_LpspiDiv, 7 );

    /* configured as gpio1 pin13 */
    IOMUXC_SetPinMux( IOMUXC_GPIO_AD_B0_13_LPSPI1_SDI, 0U );
    IOMUXC_SetPinConfig( IOMUXC_GPIO_AD_B0_13_LPSPI1_SDI, 0x10B0U ); 
    
    IOMUXC_SetPinMux( IOMUXC_GPIO_AD_B0_11_LPSPI1_PCS0, 0U );                                    
    IOMUXC_SetPinConfig( IOMUXC_GPIO_AD_B0_11_LPSPI1_PCS0, 0x10B0U );                                 

    LPSPI_MasterGetDefaultConfig( &lpspiConfig );
    
    lpspiConfig.baudRate      = 1500000U;
    lpspiConfig.whichPcs      = kLPSPI_Pcs0;
    lpspiConfig.cpol          = kLPSPI_ClockPolarityActiveLow,
    lpspiConfig.cpha          = kLPSPI_ClockPhaseSecondEdge,
    lpspiConfig.direction     = kLPSPI_MsbFirst,
    
    /* setup spi interface blocking */
    LPSPI_MasterInit( LPSPI1, &lpspiConfig, ( CLOCK_GetFreq(kCLOCK_Usb1PllPfd0Clk) / 8 ) );              
}

/******************************************************************************/
/*!   \fn void startWeigherClock( void )

      \brief
        This function used to test the pwm to supply an exernal clock to the  
        weigher adc.

      \author
          Aaron Swift
*******************************************************************************/
void testPWMWeigher( void )
{
    pwm_config_t pwmConfig;
    pwm_signal_param_t pwmSignal; 
    pwm_fault_param_t faultConfig;
    uint32_t pwmFrequency = 2450000UL;
    
    CLOCK_SetDiv( kCLOCK_AhbDiv, 0x2 );
    CLOCK_SetDiv( kCLOCK_IpgDiv, 0x3 );
    
    XBARA_Init(XBARA);
    XBARA_SetSignalsConnection(XBARA, kXBARA1_InputLogicHigh, kXBARA1_OutputFlexpwm1Fault0);
    XBARA_SetSignalsConnection(XBARA, kXBARA1_InputLogicHigh, kXBARA1_OutputFlexpwm1Fault1);
    XBARA_SetSignalsConnection(XBARA, kXBARA1_InputLogicHigh, kXBARA1_OutputFlexpwm12Fault2);
    XBARA_SetSignalsConnection(XBARA, kXBARA1_InputLogicHigh, kXBARA1_OutputFlexpwm12Fault3);


    /* set deadtime */
    uint16_t deadTimeVal = 0;
    PWM_GetDefaultConfig( &pwmConfig );
         
   
    /* use full cycle reload */
    pwmConfig.reloadLogic               = kPWM_ReloadPwmFullCycle;    
    pwmConfig.pairOperation             = kPWM_Independent;
    pwmConfig.enableDebugMode           = true;
    pwmConfig.clockSource               = kPWM_Submodule0Clock;
    pwmConfig.prescale                  = kPWM_Prescale_Divide_1;
    pwmConfig.initializationControl     = kPWM_Initialize_MasterSync;

    /* initialize submodule 0 */
    if( PWM_Init( PWM1, kPWM_Module_1, &pwmConfig ) == kStatus_Fail ) {
        PRINTF("testPWMWeigher(): PWM_Init Failed.\r\n");    
    }

    
    pwmSignal.pwmChannel                = kPWM_PwmA;    
    pwmSignal.level                     = kPWM_HighTrue;
    pwmSignal.dutyCyclePercent          = 50;
    pwmSignal.deadtimeValue             = deadTimeVal;
    pwmSignal.faultState                = kPWM_PwmFaultState0;
    
    PWM_SetupPwm( PWM1, kPWM_Module_0, &pwmSignal, 1, kPWM_SignedCenterAligned, pwmFrequency,
                 CLOCK_GetFreq( kCLOCK_IpgClk ) );
    
    
    
    /* set the load okay bit for submodule to load register from their buffer */
    PWM_SetPwmLdok( PWM1, kPWM_Control_Module_0, true );

    /* start the pwm generation from submodules 0*/
    PWM_StartTimer( PWM1, kPWM_Control_Module_0 );
    while(1) { asm("nop"); }
}

#if 0
#define adcEtc1HandleIsr ADC_ETC_IRQ1_IRQHandler
#define adcEtc2HandleIsr ADC_ETC_IRQ2_IRQHandler

void adcEtc1HandleIsr( void )
{
    uint32_t flags = ADC_ETC_GetInterruptStatusFlags( ADC_ETC, kADC_ETC_Trg0TriggerSource  );
    PRINTF( "adcEtc1HandleIsr: flags %d\r\n", flags );

    ADC_ETC_ClearInterruptStatusFlags( ADC_ETC, kADC_ETC_Trg0TriggerSource, kADC_ETC_Done1StatusFlagMask );
    /* get trigger0 chain1 result. */
    ADC_ETC_GetADCConversionValue( ADC_ETC, 0U, 1U);     
    /* added for arm errata 838869 */
    SDK_ISR_EXIT_BARRIER;    
}

void adcEtc2HandleIsr( void )
{
    uint32_t flags = ADC_ETC_GetInterruptStatusFlags( ADC_ETC, kADC_ETC_Trg0TriggerSource  );
    PRINTF( "adcEtc2HandleIsr: flags %d\r\n", flags );

    ADC_ETC_ClearInterruptStatusFlags( ADC_ETC, kADC_ETC_Trg0TriggerSource, kADC_ETC_Done2StatusFlagMask );
    /* get trigger0 chain1 result. */
    ADC_ETC_GetADCConversionValue( ADC_ETC, 0U, 1U);     
    /* added for arm errata 838869 */
    SDK_ISR_EXIT_BARRIER;    
}
#endif

void testI2CLP5521( void )
{
    initLp5521();    
    
}
#endif
