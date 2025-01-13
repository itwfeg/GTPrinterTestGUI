#include "cs5530.h"
#include "fsl_lpspi.h"
#include "fsl_pwm.h"
#include "fsl_xbara.h"
#include "fsl_gpio.h"
#include "fsl_gpt.h"
#include "fsl_iomuxc.h"
#include "pin_mux.h"
#include "fsl_debug_console.h"
#include "developmentSettings.h"
#include "threadManager.h"

QueueHandle_t                   cs5530ADQueue_         = NULL; 
TimerHandle_t                   cs5530WaitTimer_       = NULL;
TimerHandle_t                   cs5530PollTimer_       = NULL;
CS5530Mgr                       cs5530Mgr_;
lpspi_master_handle_t           cs5530SpiHandle_;

extern void serviceSwitchPressed( void );
/******************************************************************************/
/*!   \fn bool initializeCs5530( QueueHandle_t adQueue )

      \brief
        This function intializes the CS5530 external A/D. This initialization
        takes multiple steps to put the CS5530 into a usable state. 

        1.      Synchronize the serial port.
        2.      Soft reset the CS5530.          p12 - 13 of data sheet
        3.      Release from reset
        4.      Verify valid CS5530 reset.

      \author
          Aaron Swift
*******************************************************************************/
bool initializeCs5530( QueueHandle_t adQueue )
{
    bool init = false;
    /* pull CS low - keep it low*/
    assertCS5530();
    /* initialize our manager */
    memset( &cs5530Mgr_, 0, sizeof( cs5530Mgr_ ) );
    /* assign our weigher's queue */
    cs5530ADQueue_ = adQueue;
    /* setup the spi interface to the cs5530 */    
    initSpiInterface();
    /* setup sync transfer */
    buildSyncTransfer( &cs5530Mgr_ );
    /* sync the interface of the cs5530 */
    PRINTF("Sync serial port\r\n");
    if( cs5530SyncSerialPort( &cs5530Mgr_ ) ) {
        /* setup the reset transfer */
        buildResetTransfer( &cs5530Mgr_ );
        /* reset the cs5530 */
        PRINTF("Reset cs5530\r\n");
        if( cs5530Reset( &cs5530Mgr_ ) ) {
            /* cs5530 requires 8 clocks to fully reset p13 of data sheet */
            delay_uS(6000);
            buildReleaseRstTransfer( &cs5530Mgr_ );
            /* release the soft reset */
            PRINTF("Release soft reset\r\n");
            if( cs5530Reset( &cs5530Mgr_ ) ) {
                /* validate reset */
                PRINTF("Validate reset\r\n");
                cs5530ReadCfgResiter( &cs5530Mgr_ );
                if( isResetValid( &cs5530Mgr_ ) ) {
                    /* word rate and polarity */
                    PRINTF("Building conversion transfer\r\n");
                    buildConvTransfer( &cs5530Mgr_ );
                    if( cs5530ConvConfig( &cs5530Mgr_ ) ) { 
                        /* setup for continuous conversion mode */     
                        PRINTF("Setup continuous conversion\r\n");
                        if( cs5530ContConversion( &cs5530Mgr_ ) ) {
                            PRINTF("set interrupt and start polltimer\r\n");
                            #ifdef TFinkStartCS5530woApp
                            //poll timer starts conversion interrupt - if poll timer is removed call this function here
                            init = startcs5530PollTimer( CS5530_RATE ); 
                            #endif
                            init = true;
                            if( !init ) {
                                PRINTF("Could not start CS5530 poll timer\r\n");
                            }
                        } else {
                            PRINTF("initializeCs5530(): failed to set continuous conversion mode\r\n");    
                        }
                    } else {
                        PRINTF("initializeCs5530(): failed to config continuous conversion!\r\n");
                    }
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
    return init;
}

/******************************************************************************/
/*!   \fn void buildResetTransfer( CS5530Mgr *pMgr )

      \brief
        This function sets up the transmit buffer to issue a sync of the CS5530
        interface.
   
      \author
          Aaron Swift
*******************************************************************************/
void buildSyncTransfer( CS5530Mgr *pMgr )
{
    pMgr->currentCmd = _Sync_Interface;
    memset( &pMgr->txBfr[0], SYNC1, SYNC_XFER_SIZE - 1 );
    pMgr->txBfr[ SYNC_XFER_SIZE - 1 ] = SYNC0;    
}

/******************************************************************************/
/*!   \fn void buildResetTransfer( CS5530Mgr *pMgr )

      \brief
        This function sets up the transmit buffer to issue a reset of the CS5530.
   
      \author
          Aaron Swift
*******************************************************************************/
void buildResetTransfer( CS5530Mgr *pMgr )
{
    /* set the command followed by the cfg register value */
    pMgr->currentCmd = _Write_Cfg_Reg;
    /* msb first */
    pMgr->txBfr[0] = WRITE_CFG_REGISTER;    
    pMgr->txBfr[1] = (unsigned char)( ( CS5530_SOFT_RESET & 0xFF000000 ) >> 24 );
    pMgr->txBfr[2] = (unsigned char)( ( CS5530_SOFT_RESET & 0x00FF0000 ) >> 16 );
    pMgr->txBfr[3] = (unsigned char)( ( CS5530_SOFT_RESET & 0x0000FF00 ) >> 8 );
    pMgr->txBfr[4] = (unsigned char)  ( CS5530_SOFT_RESET & 0x000000FF );
     
}

/******************************************************************************/
/*!   \fn void buildReleaseRstTransfer( CS5530Mgr *pMgr )

      \brief
        This function sets up the transmit buffer to release the CS5530 from
        a software reset.
   
      \author
          Aaron Swift
*******************************************************************************/
void buildReleaseRstTransfer( CS5530Mgr *pMgr )
{
    /* set the command followed by the cfg register value */
    pMgr->currentCmd = _Write_Cfg_Reg;
    pMgr->txBfr[0] = WRITE_CFG_REGISTER;
    pMgr->txBfr[1] = CS5530_NULL;
    pMgr->txBfr[2] = CS5530_NULL;
    pMgr->txBfr[3] = CS5530_NULL;
    pMgr->txBfr[4] = CS5530_NULL;
}


/******************************************************************************/
/*!   \fn bool startcs5530PollTimer( CS5530PollFreq freq )

      \brief
        This function starts the cs5530 poll timer. Period is based off of desired 
        frequency. Timer time base is 1mS tick rate.
   
      \author
          Aaron Swift
*******************************************************************************/
bool startcs5530PollTimer( CS5530PollFreq freq )
{
    bool result = false;

    if( freq > _0HZ ) {
        if( cs5530PollTimer_ == NULL ) {
            cs5530PollTimer_  = xTimerCreate( "csPollTimer", pdMS_TO_TICKS(1000/freq), pdTRUE, (void *)0, csPollTimerCallBack );
            if( cs5530PollTimer_ == NULL ) {        
                PRINTF("startcs5530PollTimer(): Critical Error poll timer not created!\r\n" );
            } else {
                if( xTimerStart( cs5530PollTimer_, 0 ) == pdPASS ) {
                    result = true;
                    //PRINTF("startcs5530PollTimer(): Poll Timer Started!\r\n" );
                } else {
                    PRINTF("startcs5530PollTimer(): Critical Error poll timer not started!\r\n" );
                }
            }
        } else {
            result = true;        
        }
    }
    return result;    
}

/******************************************************************************/
/*!   \fn void csPollTimerCallBack( TimerHandle_t cs5530PollTimer_  )                                                      
 
      \brief
         This function is the callback for the poll timer.
       
      \author
          Aaron Swift
*******************************************************************************/ 
void csPollTimerCallBack( TimerHandle_t cs5530PollTimer_  )
{
    //PRINTF("csPollTimerCallBack():\r\n" );
    setConversionInterrupt();
    xTimerStop( cs5530PollTimer_, 0 );    
}

/******************************************************************************/
/*!   \fn void buildConfigTransfer( CS5530Mgr *pMgr  )                                                      
 
      \brief
         Sets up the transmit buffer for writing word rate and polarity to the 
         config register for continuous conversion.
       
      \author
          Aaron Swift
*******************************************************************************/ 
void buildConvTransfer( CS5530Mgr *pMgr )
{
    /* set the command followed by the cfg register value */
    pMgr->currentCmd = _Write_Cfg_Reg;
    /* msb first */
    pMgr->txBfr[0] = (unsigned char)WRITE_CFG_REGISTER;   

    unsigned long convConfig = CS5530_BIPOLAR_MODE;
#ifdef CS5530_RATE
    unsigned long rate = CS5530_RATE;
    switch(rate) {
    case _15HZ: 
        convConfig |= CS5530_15SPS_RATE;
        break;
    case _30HZ: 
        convConfig |= CS5530_30SPS_RATE;
        break;
    case _60HZ: 
        convConfig |= CS5530_60SPS_RATE;
        break;
    case _100HZ: 
        convConfig |= CS5530_120SPS_RATE;
        convConfig |= CS5530_FRS_SCALE5_6;
        break;
    case _120HZ: 
        convConfig |= CS5530_120SPS_RATE;
        break;
    default:
        PRINTF("buildConvTransfer(): Unsupported convert frequence request!");
        break;
    }
#else
    // default to 30Hz
    convConfig |= CS5530_30SPS_RATE;
#endif

    pMgr->txBfr[1] = (unsigned char)( ( convConfig & 0xFF000000 ) >> 24 );
    pMgr->txBfr[2] = (unsigned char)( ( convConfig & 0x00FF0000 ) >> 16 );
    pMgr->txBfr[3] = (unsigned char)( ( convConfig & 0x0000FF00 ) >> 8  );
    pMgr->txBfr[4] = (unsigned char)( ( convConfig & 0x000000FF ) );
}

/******************************************************************************/
/*!   \fn void setConversionInterrupt( void )

      \brief
        This function sets the spi.sdo to gpio interrupt on falling edge trigger.
        The sdo signal falls low to signal conversion complete to the host. 

      \note
        Priority level of 0-255 for each interrupt.  A higher level corresponds 
        to a lower priority, so level 0 is the highest interrupt priority.

      \author
          Aaron Swift - CKing
*******************************************************************************/
void setConversionInterrupt( void )
{   
    gpio_pin_config_t weigherIntrPinCfg;
    
    weigherIntrPinCfg.direction = kGPIO_DigitalInput;
    weigherIntrPinCfg.outputLogic = 0U;
    weigherIntrPinCfg.interruptMode = kGPIO_IntFallingEdge;
      
    GPIO_PinInit( GPIO2, 13U, &weigherIntrPinCfg );
    
    NVIC_SetPriority( GPIO2_Combined_0_15_IRQn, WEIGHER_DATA_RDY_PRIORITY ); 
    
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_13_GPIO2_IO13, 0U );                                    
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_13_GPIO2_IO13, 0xB0U );   
       
    GPIO_EnableInterrupts( GPIO2, 1U << 13U );
    EnableIRQ( GPIO2_Combined_0_15_IRQn );
    
}

/******************************************************************************/
/*!   \fn static void removeConversionInterrupt( void )

      \brief
        This function removes the GPIO interrupt monitoring CS5530 SDO and
        remuxes the pin for use in the SPI driver

      \author
          CKing
*******************************************************************************/
static void removeConversionInterrupt( void )
{
    DisableIRQ( GPIO2_Combined_0_15_IRQn );
    GPIO_DisableInterrupts( GPIO2, 1U << 13U );
      
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_13_LPSPI2_SDI, 0U );   
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_13_LPSPI2_SDI, 0x10B0U );
}


/******************************************************************************/
/*!   \fn void weigherDataRdyIsr( void )

      \brief
        This function handles the gpio interrupt from the weigher and read the 
        conversion result from the cs5530 and set the gpio back for next 
        conversion.

        This function is actually "GPIO2_Combined_0_15_IRQHandler" See
        "#define weigherDataRdyIsr GPIO2_Combined_0_15_IRQHandler" in cs5530.h

      \author
          Aaron Swift - Cking
*******************************************************************************/

void weigherDataRdyIsr( void )
{
    removeConversionInterrupt();  
  
    //PRINTFThrottle(100,"wISR\r\n");
    if( cs5530ReadConversion( &cs5530Mgr_ ) ) {
        uint32_t msb = 0, mid = 0, lsb = 0;
        BaseType_t result = pdFAIL;
          
        /* byte order */
        msb  = (uint32_t)cs5530Mgr_.rxBfr[1];
        msb = msb << 16;
            
        mid = (uint32_t)cs5530Mgr_.rxBfr[2];
        mid = mid << 8;
            
        lsb  = (uint32_t)cs5530Mgr_.rxBfr[3];
            
        cs5530Mgr_.conversion =  ( msb | mid | lsb );
            
        /* strip off pos/neg bit indicator. */
        cs5530Mgr_.conversion = cs5530Mgr_.conversion & 0x7fffff;
            
        /* post to weigher counts queue, if queue not already full */
        if( xQueueIsQueueFullFromISR(cs5530ADQueue_) ) {            
            result = pdFAIL;
            PRINTFThrottle(100,"cs5530ADQueue_ Full\r\n");
        } else {    
            BaseType_t xHigherPriorityTaskWoken = pdTRUE;
            result = xQueueSendToBackFromISR( cs5530ADQueue_, (void *)&cs5530Mgr_.conversion, &xHigherPriorityTaskWoken );
        }
                
        if( result != pdPASS ) {
            PRINTF("weigherDataRdyIsr(): Failed to post cs5530 conversion: result = %ld\r\n", result );
            PRINTF("weigherDataRdyIsr(): number of queued counts: %d \r\n", uxQueueMessagesWaitingFromISR( cs5530ADQueue_ ) );
        } 
    } 
    else {
        PRINTF("weigherDataRdyIsr(): failed to read cs5530 conversion.\r\n"); 
    }
      
    GPIO_PortClearInterruptFlags(GPIO2, 1 << 13U);
    setConversionInterrupt();
    
    /* added for arm errata 838869 */
    SDK_ISR_EXIT_BARRIER;
}

/******************************************************************************/
/*!   \fn static bool cs5530ReadConversion( CS5530Mgr *pMgr )                                                     
 
      \brief
         This function is called when the cs5530 signals conversion complete.
         5 bytes are required to read the conversion from the cs5530. First 
         8 bits clear the sdo flag and the last 4 bytes read the conversion 
         value. Please see section 2.5.2 of the datasheet.

      \author
          Aaron Swift
*******************************************************************************/ 
static bool cs5530ReadConversion( CS5530Mgr *pMgr )
{
    /* clear ready flag */
    pMgr->spiReady = false;
    /* prep the transfer buffer */
    pMgr->currentCmd = _Perform_Mult_Conv;
    pMgr->txBfr[0] = CS5530_NULL;
    pMgr->txBfr[1] = CS5530_NULL;
    pMgr->txBfr[2] = CS5530_NULL;
    pMgr->txBfr[3] = CS5530_NULL;
    pMgr->txBfr[4] = CS5530_NULL;
    /* send it */
    // writeCS5530 returns true or false, so a success would be true, rather than 0 (kStatus_Success)
    bool sent = writeCS5530( &pMgr->txBfr[0], &pMgr->rxBfr[0], READ_CONV_XFER_SIZE );
    if( !sent ) {
        PRINTF("cs5530ReadCfgResiter(): failed to read cs5530 conversion register.\r\n"); 
    }
    return sent;       
}
/******************************************************************************/
/*!   \fn static bool isResetValid( CS5530Mgr *pMgr )

      \brief
        This function validates if the cs5520 has been properly reset.
        a software reset.
   
      \author
          Aaron Swift
*******************************************************************************/
bool isResetValid( CS5530Mgr *pMgr )
{
    bool result = false;
    if( pMgr->currentCmd == _Read_Cfg_Reg ) {
        /* is the reset valid bit set in the configuration register */
        if( pMgr->rxBfr[1] == CS5530_RESET_VALID ) {
            result = true;
        }
    } else {
        PRINTF("isResetValid(): failed to validate configuration register.\r\n");     
    }
    return result;
}

/******************************************************************************/
/*!   \fn static bool cs5530ReadCfgResiter(  CS5530Mgr *pMgr )

      \brief
        This function sets up the transmit buffer to release the CS5530 from
        a software reset.
   
      \author
          Aaron Swift
*******************************************************************************/
static bool cs5530ReadCfgResiter(  CS5530Mgr *pMgr )
{
    bool result = false;
    /* clear ready flag */
    pMgr->spiReady = false;
    /* prep the transfer buffer */
    pMgr->currentCmd = _Read_Cfg_Reg;
    pMgr->txBfr[0] = READ_CFG_REGISTER;
    pMgr->txBfr[1] = CS5530_NULL;
    pMgr->txBfr[2] = CS5530_NULL;
    pMgr->txBfr[3] = CS5530_NULL;
    pMgr->txBfr[4] = CS5530_NULL;
    /* send it */
    result = writeCS5530( &pMgr->txBfr[0], &pMgr->rxBfr[0], READ_CFG_XFER_SIZE );
    if( !result ) {
        PRINTF("cs5530ReadCfgResiter() could not write to cfg register\r\n");
    }

    return result;
}

#if 0  //TFink 5/2/24 removed to eliminate warning
/******************************************************************************/
/*!   \fn static void cs5530StartClk( void )

      \brief
        This function sets up a PWM channel to provide a clock source to the 
        CS5530 external A/D.   
   
        Note: @3Mhz the PWM frequencey is 3.457Mhz and  @4.9Mhz the PWM
              frequencey is 5.2Mhz which is above the spec for the cs5530.

        Rev A boards have the XTAL on the board. This would be cost savings if
        implemented.

      \author
          Aaron Swift
*******************************************************************************/
static void cs5530StartClk( void )
{
    pwm_config_t pwmConfig;
    pwm_signal_param_t pwmSignal; 
    uint32_t pwmFrequency = 300000UL;
    
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
        PRINTF("cs5530StartClk(): PWM_Init Failed.\r\n");    
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
#endif


/******************************************************************************/
/*!   \fn void initSpiInterface( void )

      \brief
        This function sets up the SPI interface to the CS5530 external A/D.   
   
      \author
          Aaron Swift
*******************************************************************************/
static void initSpiInterface( void )
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

    /* initialize the spi master */
    LPSPI_MasterInit( LPSPI2, &masterConfig, ( CLOCK_GetFreq(kCLOCK_Usb1PllPfd0Clk) / 8 ) );
    /* setup our interrupts */
    LPSPI_EnableInterrupts( LPSPI2, ( kLPSPI_TxInterruptEnable | 
                               kLPSPI_RxInterruptEnable | 
                               kLPSPI_TransmitErrorInterruptEnable | 
                               kLPSPI_ReceiveErrorInterruptEnable ) );

    /* create interrupt handle */
    LPSPI_MasterTransferCreateHandle( LPSPI2, &cs5530SpiHandle_, (lpspi_master_transfer_callback_t)cs5530SpiCallBack, NULL );
    EnableIRQ( LPSPI2_IRQn );
    NVIC_SetPriority( LPSPI2_IRQn, WEIGHER_SPI_INT_PRIORITY );
    
    LPSPI_Enable( LPSPI2, false );
    LPSPI2->CFGR1 &= (~LPSPI_CFGR1_NOSTALL_MASK);
    LPSPI_Enable( LPSPI2, true );       

    cs5530Mgr_.spiReady = true;
}

/******************************************************************************/
/*!   \fn static bool cs5530SyncSerialPort( CS5530Mgr *pMgr )

      \brief
        This function synchronize the SPI interface on the CS5530 external A/D.   
        The interface is synchronized by sending at least 15 sync1 commands
        followed by a sync0 command.
      \author
          Aaron Swift
*******************************************************************************/
static bool cs5530SyncSerialPort( CS5530Mgr *pMgr )
{    
    bool result = false; 
    /* sync the interface */
    result = writeCS5530( &pMgr->txBfr[0], &pMgr->rxBfr[0], SYNC_XFER_SIZE ); 

    if( !result ) {
        PRINTF("cs5530SyncSerialPort(): could not write to register\r\n");
    }

    return result;   
}

/******************************************************************************/
/*!   \fn static bool cs5530Reset( CS5530Mgr *pMgr )

      \brief
        This function commands the CS5530 to perform a soft reset or release 
        from reset.

      \author
          Aaron Swift
*******************************************************************************/
static bool cs5530Reset( CS5530Mgr *pMgr )
{

    bool result = false;     
    result = writeCS5530( &pMgr->txBfr[0], &pMgr->rxBfr[0], RESET_XFER_SIZE ); 
    if( !result ) {
        PRINTF("cs5530Reset(): could not write to config register\r\n");
    }

    return result;   
}

/******************************************************************************/
/*!   \fn static bool cs5530ConvConfig( CS5530Mgr *pMgr )

      \brief
        This function writes the continuous conversion config to the config reg.

      \author
          Joseph DiCarlantonio
*******************************************************************************/
static bool cs5530ConvConfig( CS5530Mgr *pMgr )
{
    bool result = false;     
    result = writeCS5530( &pMgr->txBfr[0], &pMgr->rxBfr[0], WRITE_CFG_XFER_SIZE ); 
    if( !result ) {
        PRINTF("cs5530ConvConfig(): could not write conversion config\r\n");
    }

    return result;  
}

static void handleTransfer( CS5530Mgr *pMgr )
{
    switch( pMgr->currentCmd )
    {
        case _Write_Offset_Reg: {
            
            break;
        }
        case _Read_Offset_Reg: {
            
            break;
        }
        case _Write_Gain_Reg: {
            
            break;
        }
        case _Read_Gain_Reg: {
            
            break;
        }
        case _Write_Cfg_Reg: {
            
            break;
        }
        case _Read_Cfg_Reg: {
            
            break;
        }
        case _Perform_Single_Conv: {
            
            break;
        }
        case _Perform_Mult_Conv: {
            
            break;
        }
        case _Perform_Offset_Cal: {
            
            break;
        }
        case _Perform_Gain_Cal: {
            
            break;
        }
        case _Sync_Interface: {
            /* clear both transmit and recieve buffers */
            memset( &pMgr->txBfr[0], 0, MAX_CS5530_PAYLOAD );
            memset( &pMgr->rxBfr[0], 0, MAX_CS5530_PAYLOAD );
            pMgr->currentCmd = _CS5530_No_Command;
            break;
        }
    }
}

/******************************************************************************/
/*!   \fn static bool cs5530ContConversion( CS5530Mgr *pMgr )

      \brief
        This function commands the CS5530 to set into continuous conversion mode

      \author
          Joseph DiCarlantonio
*******************************************************************************/
static bool cs5530ContConversion( CS5530Mgr *pMgr )
{
    bool result = false;

    pMgr->currentCmd = _Perform_Mult_Conv;
    pMgr->txBfr[0] = PERFORM_MULT_CONVERSION; 
    
    result = writeCS5530( &pMgr->txBfr[0], &pMgr->rxBfr[0], 1 );  
    if( !result ) {
        PRINTF("cs5530ContConversion(): could not write to register\r\n");
    }
    return result;
}

/******************************************************************************/
/*!   \fn void cs5530SpiCallBack( void )

      \brief
        This function is the callback for the external a/d spi interface.    
      \author
          Aaron Swift
*******************************************************************************/
void cs5530SpiCallBack( void )
{   
    /* releaseCS5530(); we can leave CS low and use the CS5530 as a 3 wire SPI device - CKing */
    handleTransfer( &cs5530Mgr_ );
    cs5530Mgr_.spiReady = true;     
}


static void assertCS5530( void )
{
    GPIO_WritePinOutput( WEIGHER_SPI_CS_GPIO, WEIGHER_SPI_CS_PIN, false );     
}


/*! ****************************************************************************   
      \fn static bool writeCS5530( unsigned char *pTxData, unsigned char *pRxData, 
                              unsigned char size )
 
      \brief
        This function will write a stream of data to the cs5530. 
                        
      \author
          Aaron Swift
*******************************************************************************/ 
static bool writeCS5530( unsigned char *pTxData, unsigned char *pRxData, unsigned char size )
{
    lpspi_transfer_t masterXfer;
    status_t status;
    bool result = false;
   
    cs5530Mgr_.spiReady = false;

    masterXfer.txData = (unsigned char *)pTxData;
    masterXfer.rxData = pRxData;
    masterXfer.dataSize = size; 
    masterXfer.configFlags = kLPSPI_Pcs0 | kLPSPI_MasterPcsContinuous;
   
    status = LPSPI_MasterTransferBlocking( LPSPI2, &masterXfer );
    if( status != kStatus_Success ) {
        PRINTF("writeCS5530(): write to cs5530 failed: %d\r\n", status );
    } else {
        result = true;
    }
    return result;
}
