#include "lp5521.h"
#include <stdlib.h>
#include "fsl_lpi2c.h"
#include "pin_mux.h"
#include "fsl_gpio.h"
#include "sensors.h"
#include "translator.h"
#include "prMessages.h"
#include "threadManager.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "dvr8818.h"
#include "takeupMotor.h"

#define LPI2C_CLOCK_FREQUENCY ((CLOCK_GetFreq(kCLOCK_Usb1PllClk) / 8) / (6U))

static LP5521CFG lp5521Cfg;
static bool initEnablePin_      = false;

extern void delay_uS ( unsigned int time );
extern void setConfigBackingValue(unsigned short val );
extern void setConfigLabelValue(unsigned short val );

static unsigned short x[ CC_TWENTY_FIVE_POINT_FIVE + 1 ];
static unsigned short y[ CC_TWENTY_FIVE_POINT_FIVE + 1 ];
static unsigned short z[ CC_TWENTY_FIVE_POINT_FIVE + 1 ];

static uint16_t relaxedVals[255] = { 0 };
static uint16_t tightenedVals[255] = { 0 };
static uint16_t deltaVals[255] = { 0 };

static bool gapCal_ = false;
static bool TUCal_ = false;

static uint16_t TUDrive = 0;

extern Pr_Config config_;
extern PrStatusInfo currentStatus;
/******************************************************************************/
/*!   \fn bool initLp5521( void )

      \brief
        This function initializes the constant current source driver IC.
   
      \author
          Aaron Swift
*******************************************************************************/
bool initLp5521( void )
{
    bool result = false;
    gpio_pin_config_t config = { kGPIO_DigitalOutput, 1, };
    
    memset( (void *)&lp5521Cfg, 0, sizeof(LP5521CFG) );
    
    /* initialize interface to device */
    initializeI2C();
        
    /* enable the device */
    GPIO_PinInit( LP5521_EN_GPIO, LP5521_EN_PIN, &config );
    GPIO_WritePinOutput(LP5521_EN_GPIO, LP5521_EN_PIN, true);
    initEnablePin_ = true;
    
    LP5521delay();    
    
    result = configure();
    
    return result;
}


/******************************************************************************/
/*!   \fn static bool increaseShootCurrent( void )

      \brief
        This function will intialize the i2c interface for the lp5521.
                  
      \author
          Aaron Swift
*******************************************************************************/
static void initializeI2C( void )
{
    lpi2c_master_config_t i2cConfig;
    /* select the usb1 pll (480Mhz )as master clock for lpi2c */
    CLOCK_SetMux(kCLOCK_Lpi2cMux, 0U );
    CLOCK_SetDiv(kCLOCK_Lpi2cDiv, 5U );
    
    LPI2C_MasterGetDefaultConfig( &i2cConfig );
    i2cConfig.baudRate_Hz = 100000U;    /* 100Khz */
    LPI2C_MasterInit( (LPI2C_Type *)LPI2C1_BASE, &i2cConfig, LPI2C_CLOCK_FREQUENCY );
    LPI2C_MasterEnable( (LPI2C_Type *)LPI2C1_BASE, true );
}

/******************************************************************************/
/*!   \fn static bool increaseShootCurrent( void )

      \brief
        This function will increase the drive current 0.1mA to the gap sensor 
        each time function is called.
           
      \author
          Aaron Swift
*******************************************************************************/
bool increaseGapCurrent( void )
{
    bool result = false;
    /* have we reached the max current? */
    if( lp5521Cfg.redCurrentReg < CC_TWENTY_FIVE_POINT_FIVE ) {
        lp5521Cfg.redCurrentReg++;  
        /* increase drive current by 0.1mA */ 
        if( writeRegister( _LP5521_GAP_CUR_REG, lp5521Cfg.redCurrentReg ) ) {
            result = true;               
        } else {
            PRINTF("increaseGapCurrent(): Failed to write LP5521 current register!\r\n");
        }
    } else {
        PRINTF("increaseGapCurrent(): Max sensor current reached!\r\n");
    } 
    return result;
}

/******************************************************************************/
/*!   \fn static bool increaseShootCurrent( void )

      \brief
        This function will increase the drive current 0.1mA to the low stock sensor 
        each time function is called.
           
      \author
          Aaron Swift
*******************************************************************************/
static bool increaseLowStockCurrent( void )
{
    bool result = false;
    /* have we reached the max current? */
    if( lp5521Cfg.blueCurrentReg < CC_TWENTY_FIVE_POINT_FIVE ) {
        lp5521Cfg.blueCurrentReg++;  
        /* increase drive current by 0.1mA */ 
        if( writeRegister( _LP5521_LOW_STOCK_CUR_REG, lp5521Cfg.blueCurrentReg ) ) { //is this the right register? was set to green before - CK
            result = true;               
        } else {
            PRINTF("increaseLowStockCurrent(): Failed to write LP5521 current register!\r\n");
        }
    } else {
        PRINTF("increaseLowStockCurrent(): Max sensor current reached!\r\n");
    } 
    return result;    
}

/******************************************************************************/
/*!   \fn static bool decrementGapCurrent( void )

      \brief
        This function will decrease the drive current 0.1mA to the gap sensor 
        each time function is called.
           
      \author
          Aaron Swift
*******************************************************************************/
bool decrementGapCurrent( void )
{
    bool result = false;
    /* have we reached the max current? */
    if( lp5521Cfg.redCurrentReg != CC_Zero ) {
        lp5521Cfg.redCurrentReg--;  
        /* increase drive current by 0.1mA */ 
        if( writeRegister( _LP5521_GAP_CUR_REG, lp5521Cfg.redCurrentReg ) ) {
            result = true;            
        } else {
            PRINTF("increaseGapCurrent(): Failed to write LP5521 current register!\r\n");
        }
    } else {
        PRINTF("increaseGapCurrent(): Max sensor current reached!\r\n");
    }
    return result;    
}

/******************************************************************************/
/*!   \fn static bool decreaseLowStockCurrent( void )

      \brief
        This function will decrease the drive current 0.1mA to the low stock sensor 
        each time function is called.
           
      \author
          Aaron Swift
*******************************************************************************/
static bool decreaseLowStockCurrent( void )
{
    bool result = false;
    /* have we reached the max current? */
    if( lp5521Cfg.blueCurrentReg != CC_Zero ) {
        lp5521Cfg.blueCurrentReg--;  
        /* decrease drive current by 0.1mA */ 
        if( writeRegister( _LP5521_LOW_STOCK_CUR_REG, lp5521Cfg.blueCurrentReg ) ) {
            result = true;            
        } else {
            PRINTF("decreaseLowStockCurrent(): Failed to write LP5521 current register!\r\n");
        }
    } else {
        PRINTF("decreaseLowStockCurrent(): Max sensor current reached!\r\n");
    }
    return result;        
}

/******************************************************************************/
/*!   \fn bool setLowStockCurrent( unsigned char value )

      \brief
        This function sets the low stock sensor drive current by the amount of the  
        value argument.
           
      \author
          Aaron Swift
*******************************************************************************/
bool setLowStockCurrent( unsigned char value )
{
    bool result = false;
    if( value <= (unsigned char)CC_TWENTY_FIVE_POINT_FIVE ) {
        lp5521Cfg.blueCurrentReg = value;
        if( writeRegister( _LP5521_LOW_STOCK_CUR_REG, value ) ) {  
            result = true;
        } else {
            PRINTF("setGapCurrent(): Failed to write LP5521 current register!\r\n");
        }
    } else {
        PRINTF("setGapCurrent(): Max sensor current reached!\r\n");
    }
    return result;    
}

/******************************************************************************/
/*!   \fn bool setGapCurrent( unsigned char value )

      \brief
        This function sets the gap sensor drive current by the amount of the  
        value argument.
           
      \author
          Aaron Swift
*******************************************************************************/
bool setGapCurrent( unsigned char value )
{
    bool result = false;
    if( value < (unsigned char)CC_TWENTY_FIVE_POINT_FIVE ) {
        if( writeRegister( _LP5521_GAP_CUR_REG, value ) ) {  
            result = true;
        } else {
            PRINTF("setGapCurrent(): Failed to write LP5521 current register!\r\n");
        }
    } else {
        //PRINTF("setGapCurrent(): Max sensor current reached!\r\n");
    }
    return result;    
}

/******************************************************************************/
/*!   \fn bool setLowStockDutyCycle( unsigned char value )

      \brief
        This function sets the low stock sensor duty cycle by the amount of the  
        value argument.
           
      \author
          Aaron Swift
*******************************************************************************/
bool setLowStockDutyCycle( unsigned char value )
{
    bool result = false;
    if( value <= (unsigned char)DC_100_PERCENT ) {
        if( writeRegister( _LP5521_LOW_STOCK_PWM_REG, value ) ) {  
            result = true;
        } else {
            PRINTF("setLowStockDutyCycle(): Failed to write LP5521 current register!\r\n");
        }
    } else {
        PRINTF("setLowStockDutyCycle(): Max duty cycle reached!\r\n");
    }
    return result;        
}

/******************************************************************************/
/*!   \fn void shutdownGapSensor( void )

      \brief
        This function shutsdown the gap sensor drive current by removing the 
        chip enable. This effectively resets the lp5521 and needs to be 
        intialized to restart. 
           
      \author
          Aaron Swift
*******************************************************************************/
void shutdownGapSensor( void )
{
    gpio_pin_config_t config = { kGPIO_DigitalOutput, 1, };
    if( !initEnablePin_ ) {
        GPIO_PinInit( LP5521_EN_GPIO, LP5521_EN_PIN, &config );
    }
    
    GPIO_WritePinOutput( LP5521_EN_GPIO, LP5521_EN_PIN, false );    
}
/******************************************************************************/
/*!   \fn bool setPaperTakeupCurrent( unsigned char value )

      \brief
        This function sets the paper takeup sensor drive current by the amount 
        of the  value argument.
           
      \author
          Aaron Swift
*******************************************************************************/
bool setPaperTakeupCurrent( unsigned char value )
{
    bool result = false;
    if( writeRegister( _LP5521_PAPER_TAKE_UP_CUR_REG, value ) ) {  
        result = true;
    }
    return result;        
}

/******************************************************************************/
/*!   \fn bool gapSensorCal( GAPSteps step, unsigned char *pBias )

      \brief
        This function sets up the drive current level to the Gap sensor.
        To use this function label stock must be between the gap sensor and 
        ensure that backing paper only is between the sensor. 
        The function starts from the highest drive current and works down until
        light is just detected. 

        Sensor: backing paper           => 300mV
                backing + label         => 1.30V
  
      \author
       Aaron Swift
*******************************************************************************/
bool gapSensorCal( GAPSteps step, unsigned char *pBias )
{
    bool result = false;
    static unsigned short *pBacking = NULL, *pLabel = NULL, *pDeflection = NULL;
    static unsigned short *pBackingS = NULL, *pLabelS = NULL, *pDeflectionS = NULL;
    static PrGapCalStatus calStatus;
    
    switch( step )
    {
        case _INIT: {
            PRINTF("gapSensorCal(): _INIT\r\n");  
            
            gapCal_ = true;
             
            /* make sure we have not been allocated before */
            if( !pBacking || !pLabel || !pDeflection ) {
                /* clear our message to the host */
                memset( &calStatus, 0, sizeof(PrGapCalStatus) );
                
                /* allocate buffers for storing measurements */
                pBackingS = pBacking        = &x[0];    /* pvPortMalloc( CC_TWENTY_FIVE_POINT_FIVE + 1 ); */
                pLabelS = pLabel            = &y[0];    /* pvPortMalloc( CC_TWENTY_FIVE_POINT_FIVE + 1 ); */
                pDeflectionS = pDeflection  = &z[0];    /* pvPortMalloc( CC_TWENTY_FIVE_POINT_FIVE + 1 ); */   
                if( pBacking && pLabel && pDeflection ) {
                     
                     memset( pBackingS, 0, ( CC_TWENTY_FIVE_POINT_FIVE + 1 ) );
                     pBacking = pBackingS;
                     
                     memset( pLabelS, 0, ( CC_TWENTY_FIVE_POINT_FIVE + 1 ) );
                     pLabel = pLabelS;
                     
                     memset( pDeflectionS, 0, ( CC_TWENTY_FIVE_POINT_FIVE + 1 ) );
                     pDeflection = pDeflectionS;
                     
                    /* notify host ready for next step in calibration */
                    calStatus.state =_CalInit;
                    sendPrGapCalStatus( &calStatus ); 
                    result = true;
                } else {
                    /*inform the host we failed to initialize */        
                    calStatus.state =_FailureMemory;
                    sendPrGapCalStatus( &calStatus );         
                                        
                    pBacking = 0, pLabel = 0, pDeflection = 0;
                    
                    PRINTF("gapSensorCal(): Failed to allocate memory for calibration!\r\n");            
                }
            } else {
                     memset( pBackingS, 0, ( CC_TWENTY_FIVE_POINT_FIVE + 1 ) );
                     pBacking = pBackingS;
                     
                     memset( pLabelS, 0, ( CC_TWENTY_FIVE_POINT_FIVE + 1 ) );
                     pLabel = pLabelS;
                     
                     memset( pDeflectionS, 0, ( CC_TWENTY_FIVE_POINT_FIVE + 1 ) );
                     pDeflection = pDeflectionS;

                     /* notify host ready for next step in calibration */
                    calStatus.state =_CalInit;
                    sendPrGapCalStatus( &calStatus ); 
                    result = true;                     
            }
            break;
        }
        case _CALBACKING: {
            if( pBacking && pLabel && pDeflection ) {   
                PRINTF("gapSensorCal(): _CALBACKING start\r\n");
                /* pause the sensors thread from doing conversions */
                pauseResumeConversions( true );

                /* set gap bias current to min */
                lp5521Cfg.redCurrentReg = CC_Zero; 
                setGapCurrent( lp5521Cfg.redCurrentReg );
                
                /* give the adc a chance to sample 
                delay_uS( FIVE_MILISECONDS );*/
                delay_uS( TEN_MILISECONDS );
                
                /* read the voltage across the detector */
                *pBacking = getMediaCounts();  
                       
                /* keep decreasing current until at end of range*/
                while( lp5521Cfg.redCurrentReg < CC_TWENTY_FIVE_POINT_FIVE ) {
                    /* if fails to set register then abort */
                    if( increaseGapCurrent() ) {
                        /* get the average of 10 sensor readings */
                        *pBacking = getShootAverage();
                        PRINTF("gapSensorCal():backing bias: %d backing cnts: %d\r\n", lp5521Cfg.redCurrentReg, *pBacking );
                        taskYIELD(); 
                        pBacking++;
                    } else {
                        /* inform host of failure   */     
                        calStatus.state =_FailureSettingI;
                        sendPrGapCalStatus( &calStatus );  

                        PRINTF("gapSensorCal(): Failed to set gap current!\r\n");
                    }                     
                }
                
                if( calStatus.state !=_FailureSettingI ) {
                    PRINTF("gapSensorCal(): backing paper cal done.\r\n"); 
                    result = true;
                }
                /* reset our pointer */
                pBacking = pBackingS; 
                
                /* resume the sensors thread from doing conversions */
                pauseResumeConversions( false );
                
                /* notify host we are finished with step */ 
                calStatus.state =_BackingPaper;
                sendPrGapCalStatus( &calStatus );
            } else {
                PRINTF("gapSensorCal(): Abort cal backing paper!\r\n");            
            }
            break;
        }
        case _CALLABEL: {
            if( pBacking && pLabel && pDeflection ) {
              
                checkForPaper(((float)config_.takeup_sensor_max_tension_counts * 0.90), 820);
              
                /* set gap bias current to min */
                lp5521Cfg.redCurrentReg = CC_Zero; 
                setGapCurrent( lp5521Cfg.redCurrentReg );
                
                /* give the adc a chance to sample */
                delay_uS( TEN_MILISECONDS );
                
                /* read the voltage across the detector */
                *pLabel = getMediaCounts();  

                /* pause the sensors thread from doing conversions */
                pauseResumeConversions( true );
                
                /* keep decreasing current until at end of range*/
                while( lp5521Cfg.redCurrentReg < CC_TWENTY_FIVE_POINT_FIVE ) {
                    /* if fails to set register then abort */
                    if( increaseGapCurrent() ) {
                        /* get the average of 10 sensor readings */
                        *pLabel = getShootAverage(); 
                         PRINTF("gapSensorCal():label bias: %d label cnts: %d\r\n", lp5521Cfg.redCurrentReg, *pLabel );
                         taskYIELD(); 
                        pLabel++;
                    } else {
                        /* inform host of failure   */        
                        calStatus.state =_FailureSettingI;
                        sendPrGapCalStatus( &calStatus ); 

                        PRINTF("gapSensorCal(): Failed to set gap current!\r\n");
                        return result;
                    }            
                }
                
                if( calStatus.state !=_FailureSettingI ) {
                    PRINTF("gapSensorCal(): label cal done.\r\n"); 
                    result = true;
                }

                /* reset our pointer */
                pLabel = pLabelS;

                /* resume the sensors thread from doing conversions */
                pauseResumeConversions( false );

                /* notify host we are finished with step */ 
                calStatus.state =_LabelPlusBacking;
                sendPrGapCalStatus( &calStatus ); 
            } else {
                PRINTF("gapSensorCal(): Abort cal label paper!\r\n");            
            }
            break;
        }
        case _CALRESULTS: {
            if( pBacking && pLabel && pDeflection ) {
                /* calculate deflections */
                for( int i = 0; i < CC_TWENTY_FIVE_POINT_FIVE; i++ ) {
                    *pDeflection = abs( *pLabel - *pBacking );
                    /*
                    PRINTF("gapSensorCal(): index: %d deflection: %d \r\n", i, *pDeflection); 
                    taskYIELD(); */
                    pDeflection++;
                    pLabel++;
                    pBacking++;            
                }
                
                /* reset our pointers and calculate deflection */
                pBacking = pBackingS; 
                pLabel = pLabelS;
                pDeflection = pDeflectionS;
                
                /* skip first 10 readings */
                pDeflection += 10;
                /* now find the greatest deflection point */
                unsigned short dmax = *pDeflection++;    
                PRINTF("gapSensorCal(): 1) deflection: %d \r\n", dmax ); 
                for( int i = 11; i < CC_TWENTY_FIVE_POINT_FIVE; i++ ) {
                    if( *pDeflection > dmax ) {
                        /* drive current for our greatest deflection value */
                        dmax = *pDeflection; 
                        PRINTF("gapSensorCal(): cal point: %d deflection: %d \r\n", i, z[i]); 
                        taskYIELD();
                        *pBias = i;
                    } 
                    pDeflection++;
                }
                /* reset our pointer */
                pDeflection = pDeflectionS; 


                *pBias = ((float)*pBias); 

                
                PRINTF("\r\n\r\nDrive current set to: %d\r\n\r\n", *pBias);
                PRINTF("\r\n\r\npBacking = %d\r\n\r\n", pBacking[*pBias]);
                PRINTF("\r\n\r\npLabel = %d\r\n\r\n", pLabel[*pBias]);
                setConfigBackingValue( pBacking[*pBias] );
                setConfigLabelValue( pLabel[*pBias] );
                
                
                /* inform host or results */
                calStatus.state =_CalcDeflections;
                
                calStatus.deflectionVoltage = (int)config_.takeup_sensor_drive_current;
                calStatus.driveCurrent = (*pBias);
                /* add backing paper val at calibration point to config */
                //setConfigBackingValue( pBacking[*pBias] );
                calStatus.backingVoltage = *pBacking;  
                /* add backing paper plus label val at calibration point to config */
                //setConfigLabelValue( pLabel[*pBias] );
                calStatus.labelBackVoltage = *pLabel;
               
                sendPrGapCalStatus( &calStatus );
                
                pBacking += *pBias;    
                pLabel += *pBias;
                pDeflection += *pBias;
                              
                float w = (float)( (float)*pDeflection * AD_RESOLUTION );
                float x  = (float)( (float)*pBias / 10);
                float y  = (float)( (float)*pBacking * AD_RESOLUTION );               
                float z = (float)( (float)*pLabel * AD_RESOLUTION );
                                
                //PRINTF("gapSensorCal(): calibrated values: \r\n");
                //PRINTF("gapSensorCal(): detectorVoltage: %2.3fV\r\n", w );
                //PRINTF("gapSensorCal(): driveCurrent: %2.3fV\r\n", x );
                //PRINTF("gapSensorCal(): backingVoltage: %2.3fV\r\n", y );
                //PRINTF("gapSensorCal(): labelBackVoltage: %2.3fV\r\n", z );
                taskYIELD();
                /* cleanup */
                pBacking = pBackingS; 
                pLabel = pLabelS;
                pDeflection = pDeflectionS;                
                result = true;
            } else {
                PRINTF("gapSensorCal(): Abort cal results!\r\n");
            }
            break;
        }
        case _CALDONE: {
            PRINTF("gapSensorCal(): here 4\r\n");          
            result = true;
            calStatus.state =_Done;            
            sendPrGapCalStatus( &calStatus ); 
            
            gapCal_ = false;
            
            break;
        }
        default: {
            PRINTF("gapSensorCal(): Unknown cal step!\r\n");
            break;
        }
    }
    
    
    
    return result;
}

/******************************************************************************/
/*!   \fn bool TUSensorCal( GAPSteps step, unsigned char *pBias )

      \brief

  
      \author
       Chris King
*******************************************************************************/
bool TUSensorCal( GAPSteps step, unsigned char *pBias )
{
    bool result = false;
    static unsigned short *pBacking = NULL, *pLabel = NULL, *pDeflection = NULL;
    static unsigned short *pBackingS = NULL, *pLabelS = NULL, *pDeflectionS = NULL;
    static PrGapCalStatus calStatus;
    
    uint8_t TUCurrent = 0;
    uint8_t TUIndex = 0;
    uint16_t TUVal = 0;
    
    switch( step )
    {
        case _INIT: 
        {
            //spoof head up 
            //setTUCalHeadUpFlag(true);

            memset( &calStatus, 0, sizeof(PrGapCalStatus) );
            calStatus.state =_CalTakeupMinTension;
            sendPrGapCalStatus( &calStatus ); 
            
            gapCal_ = true;
            
            PRINTF("TU calibration start\r\n");
    
            setPaperTakeupCurrent( TUCurrent );
                        
            powerOnMotors();
                    
            while(TUCurrent < 255)
            {
                powerOnMotors();
              
                TUVal = getTakeUpTorque();
                relaxedVals[TUIndex] = TUVal;
              
                //PRINTF("TUVal = %d     current = %d\r\n", TUVal, TUCurrent);
                TUCurrent++;
                TUIndex++;
                
                setPaperTakeupCurrent( TUCurrent ); 
            }
            
            TUCurrent = 0;
            TUIndex = 0;
            
            setPaperTakeupCurrent( TUCurrent );

            result = true;
        
            break;
        }
        case _CALRESULTS: 
        {
            PRINTF("TUSensorCal(): _CALRESULTS\r\n");
                                                            
            //continue on to gap sensor calibration
            memset( &calStatus, 0, sizeof(PrGapCalStatus) );
            
            calStatus.deflectionVoltage = TUDrive;
            calStatus.driveCurrent = config_.media_sensor_adjustment;
            calStatus.backingVoltage = 0;  
            calStatus.labelBackVoltage = 0;
            
            calStatus.state = _CalcDeflections;            
            sendPrGapCalStatus( &calStatus );
            
            taskYIELD();
            
            result = true;
          
            break;
        }
        case _CALDONE: {
            PRINTF("TUSensorCal(): _Done\r\n"); 
            PRINTF("TU calibration done\r\n");
            PRINTF("\r\n\r\n\r\n");
            
            powerOnMotors();
         
            while(TUCurrent < 255)
            {
                powerOnMotors();
              
                TUVal = getTakeUpTorque();
                tightenedVals[TUIndex] = TUVal;
              
                //PRINTF("TUVal2 = %d     current2 = %d\r\n", TUVal, TUCurrent);
                TUCurrent++;
                TUIndex++;
                
                setPaperTakeupCurrent( TUCurrent ); 
            }
            
            TUCurrent = 0;
            TUIndex = 0;

            setPaperTakeupCurrent( TUCurrent );    
             
            
            int size = sizeof(relaxedVals) / sizeof(relaxedVals[0]);
            
            for (int i = 0; i < size; ++i) 
            {
                deltaVals[i] = tightenedVals[i] - relaxedVals[i];
            }
            
            /*
            for(int i = 0; i < 255; i++)
            {
                //PRINTF("deltaVals = %d \r\n", deltaVals[i], i);
                
                for(int i = 0; i < 1000; i++)
                {
                    __NOP();
                    for(int i2 = 0; i2 < 1000; i2++)
                    {
                        __NOP();
                    }
                }
            }
            */
            
            if (size <= 0) 
            {
                // Handle invalid size
                return 0;  
            }

            uint16_t largestValue = 0;
            uint16_t largestValueIndex = 0;

            for (int i = 10; i < size; ++i) 
            {
                if (deltaVals[i] > largestValue) 
                {
                    largestValue = deltaVals[i];
                    largestValueIndex = i;
                }
            }
            
            //PRINTF("largest deflection detected at current level %d\r\n", largestValueIndex);
            //PRINTF("deflection value = %d\r\n", largestValue);
            //PRINTF("TU arm resting torque value = %d\r\n", relaxedVals[largestValueIndex]);
            //PRINTF("TU arm tightened torque value = %d\r\n", tightenedVals[largestValueIndex]);
                           
            //set our calibrated torque sensor current
            setPaperTakeupCurrent( largestValueIndex );
        
            //DVT 3 springs
            unsigned short max = (int)(MAX_TENSION_MULTIPLIER * (float)tightenedVals[largestValueIndex]);
            unsigned short min = (int)(MIN_TENSION_MULTIPLIER * (float)tightenedVals[largestValueIndex]);
            
            //randys new springs
            //unsigned short max = (int)(0.60 * tightenedVals[largestValueIndex]);
            //unsigned short min = (int)(0.55 * tightenedVals[largestValueIndex]);
            
            setConfigTakeupSensorValue( largestValueIndex,  max, min);
            
            TUDrive = largestValueIndex;
            
            PRINTF("TU sensor drive current set to: %d\r\n", largestValueIndex);
            
            PRINTF("Max tightness = %d\r\n", max);
            PRINTF("Min tightness = %d\r\n", min);
            
            calStatus.state =_Done;
            
            calStatus.deflectionVoltage = TUDrive;
            calStatus.driveCurrent = config_.media_sensor_adjustment;
            calStatus.backingVoltage = config_.backingPaper;  
            calStatus.labelBackVoltage = config_.backingAndlabel;
           
            sendPrGapCalStatus( &calStatus );
            
            taskYIELD();
            
            gapCal_ = false;
            
            result = true;
                    
            break;
        }
        default: {
            break;
        }
    }
    
    
    return result;
}

void testlabelTaken( void )
{
    float takenVoltage = 0.0;
   
    for( int i = (int)CC_ONE_POINT_ZERO;  i < (int)CC_TWENTY_FIVE_POINT_FIVE; i++ ) {
        
        setMediaCurrent( i );    
        delay_uS( FIVE_MILISECONDS );
        delay_uS( FIVE_MILISECONDS ); //for test only
        
        unsigned long cnts = getLabelTaken();
        takenVoltage =  (float)( (float)cnts * AD_RESOLUTION );
        PRINTF("testlabelTaken(): takenVoltage: %2.3fV\r\n", takenVoltage );  
    }
    
}

/******************************************************************************/
/*!   \fn static bool configure( void )

      \brief
        This function configures the driver IC based on Avery's hardware 
        platform.
   
      \author
          Aaron Swift
*******************************************************************************/
static bool configure( void )
{
    bool result = false;
    
    if( getMyModel() == RT_GLOBAL_SCALE_GOOD ) {
        /* setup for freestanding scale */
        configureHobart( &lp5521Cfg );
    } else {
        /* setup for avery configuration */ 
        configureAvery( &lp5521Cfg );
    }

    /* initialize enable register */
    if( writeRegister( _LP5521_EN_REG, lp5521Cfg.enableReg ) ) {      
        if( writeRegister( _LP5521_OP_MODE_REG, lp5521Cfg.opModeReg ) ) {          
            if( writeRegister( _LP5521_GAP_PWM_REG, lp5521Cfg.redPwmReg ) ) {
                if( writeRegister( _LP5521_LOW_STOCK_PWM_REG, lp5521Cfg.bluePwmReg ) ) {
                    if( writeRegister( _LP5521_PAPER_TAKE_UP_PWM_REG, lp5521Cfg.greenPwmReg ) ) {
                        if( writeRegister( _LP5521_GAP_CUR_REG, lp5521Cfg.redCurrentReg ) ) {
                            if( writeRegister( _LP5521_LOW_STOCK_CUR_REG, lp5521Cfg.blueCurrentReg ) ) {
                                if( writeRegister( _LP5521_PAPER_TAKE_UP_CUR_REG, lp5521Cfg.greenCurrentReg ) ) {
                                    if( writeRegister( _LP5521_CFG_REG, lp5521Cfg.configReg ) ) {
                                        result = true;
                                    } else {
                                        PRINTF("configure(): Failed to write LP5521 config register!\r\n");
                                    }                                                                                                
                                } else {
                                    PRINTF("configure(): Failed to write LP5521 current register!\r\n");
                                }                                                                
                            } else {
                                PRINTF("configure(): Failed to write LP5521 current register!\r\n");
                            }                                                            
                        } else {
                            PRINTF("configure(): Failed to write LP5521 current register!\r\n");
                        }                                    
                    } else {
                        PRINTF("configure(): Failed to write LP5521 pwm register!\r\n");
                    }                
                } else {
                    PRINTF("configure(): Failed to write LP5521 pwm register!\r\n");
                }            
            } else {
                PRINTF("configure(): Failed to write LP5521 pwm register!\r\n");
            }
        } else {
            PRINTF("configure(): Failed to write LP5521 op mode register!\r\n");
        }        
    } else {
        PRINTF("configure(): Failed to write LP5521 enable register!\r\n");
    }    
    return result;
}

/******************************************************************************/
/*!   \fn static void configureAvery( LP5521CFG *pLP5521Cfg )

      \brief
        This function sets up the configuration based on Avery's hardware 
        platform.
   
      \author
          Aaron Swift
*******************************************************************************/
static void configureAvery( LP5521CFG *pLP5521Cfg )
{
    /* setup the register according to Avery's configuration */
    pLP5521Cfg->deviceAddr = LP5521_I2C_ADDR;
    pLP5521Cfg->opModeReg = ( CH_R_DIRECT_CONTROL | CH_G_DIRECT_CONTROL | CH_B_DIRECT_CONTROL );
    pLP5521Cfg->enableReg = LP5521_OPERATE;
    pLP5521Cfg->configReg = ( PWM_256_FREQ | CP_AUTO_MODE | LED_SOURCE_DEFAULT | CLK_INTERNAL_A );
    pLP5521Cfg->redCurrentReg = CC_THREE_POINT_FIVE;   
    pLP5521Cfg->greenCurrentReg = CC_THREE_POINT_FIVE; 
    pLP5521Cfg->blueCurrentReg = CC_THREE_POINT_FIVE;  
    pLP5521Cfg->redPwmReg = DC_100_PERCENT;
    pLP5521Cfg->greenPwmReg = DC_100_PERCENT;
    pLP5521Cfg->bluePwmReg = DC_100_PERCENT;   
}

/******************************************************************************/
/*!   \fn static void configureHobart( LP5521CFG *pLP5521Cfg )

      \brief
        This function sets up the configuration based on Hobart's hardware 
        platform.
   
      \author
          Aaron Swift
*******************************************************************************/
static void configureHobart( LP5521CFG *pLP5521Cfg )
{
    /*
    typedef enum
    {
        DC_0_PRECENT,
        DC_5_PRECENT =                      0x05,
        DC_10_PRECENT =                     0x10,
        DC_25_PRECENT =                     0x3f,
        DC_50_PRECENT =                     0x7f,
        DC_75_PRECENT =                     0xbf,
        DC_90_PRECENT =                     0xe6,
        DC_97_PERCENT =                     0xFc,
        DC_100_PERCENT =                    0xff    
    }PWMDutyCycle;
    */
  
    /* setup the register according to Hobart's configuration */
    pLP5521Cfg->deviceAddr = LP5521_I2C_ADDR;
    pLP5521Cfg->opModeReg = ( CH_R_DIRECT_CONTROL | CH_G_DIRECT_CONTROL | CH_B_DIRECT_CONTROL );
    pLP5521Cfg->enableReg = LP5521_OPERATE;
    pLP5521Cfg->configReg = ( PWM_558_FREQ | /*CP_AUTO_MODE*/ CP_FORCED_1_MODE | LED_SOURCE_DEFAULT | CLK_INTERNAL_A );
    //PRINTF("\r\n\r\nconfig_.media_sensor_adjustment = %d\r\n\r\n", config_.media_sensor_adjustment);
    pLP5521Cfg->redCurrentReg = CC_Zero;
    pLP5521Cfg->greenCurrentReg = CC_Zero;
    pLP5521Cfg->blueCurrentReg = CC_Zero;
    //pLP5521Cfg->redPwmReg = DC_90_PRECENT;
    pLP5521Cfg->redPwmReg = DC_100_PERCENT;  
    pLP5521Cfg->greenPwmReg = DC_100_PERCENT;  
    pLP5521Cfg->bluePwmReg = DC_100_PERCENT;
}

/******************************************************************************/
/*!   \fn static unsigned short getShootAverage( void )

      \brief
        This function converts 10 shoot through channel samples from the a/d 
        and returns the average raw count.
   
      \author
          Aaron Swift
*******************************************************************************/
unsigned short getShootAverage( void )
{
    unsigned int dCnts = 0;
    /* take 10 readings fromt he shoot through sensor and return average */
    for( int i = 0; i < 25; i++ ) {
        /* read the voltage across the detector */
        dCnts += getShootThroughConversion();   
        if( isADCAutoMode() ) {
            delay_uS( ONE_MILISECONDS );
        }
    }
    dCnts = dCnts / 25;
    return (unsigned short)dCnts;
}

/******************************************************************************/
/*!   \fn static bool writeRegister( unsigned char reg, unsigned char data )

      \brief
        This function writes data to the lp5521 register selected by the 
        variable reg and returns result of the i2c write.
        
        returns 0 on kStatus_Success

      \author
          Aaron Swift
*******************************************************************************/
static bool writeRegister( unsigned char reg, unsigned char data )
{
    status_t result = kStatus_Fail;
    size_t txCnts = 0xFFU;              /* full */ 
    uint8_t txBffr[3] = { 0 };    
    bool x = false;
    
    #if 0
    txBffr[0] = LP5521_I2C_ADDR;
    txBffr[1] = reg;
    txBffr[2] = data;
    #else
    txBffr[0] = reg;
    txBffr[1] = data;    
    #endif
    /* send the start of frame */
    if( kStatus_Success == LPI2C_MasterStart( (LPI2C_Type *)LPI2C1_BASE, LP5521_I2C_ADDR, kLPI2C_Write ) ) {
        /* check master tx fifo */
        LPI2C_MasterGetFifoCounts( (LPI2C_Type *)LPI2C1_BASE, NULL, &txCnts );
        /* wait till empty */
        while( txCnts ) {            
            LPI2C_MasterGetFifoCounts( (LPI2C_Type *)LPI2C1_BASE, NULL, &txCnts );
        }
        /* slave device failed to ack start of frame */
        if( LPI2C_MasterGetStatusFlags( (LPI2C_Type *)LPI2C1_BASE ) & kLPI2C_MasterNackDetectFlag )
        {
            return x;
        }
        /* send device address sub-address and data */
        result = LPI2C_MasterSend( (LPI2C_Type *)LPI2C1_BASE, (void *)&txBffr[0], 2 );
        if( result != kStatus_Success ) {
            if( result == kStatus_LPI2C_Nak ) {
                /* send stop of frame */
                LPI2C_MasterStop( (LPI2C_Type *)LPI2C1_BASE );
            }
            return x;
        }
        /* send stop of frame */
        result = LPI2C_MasterStop( (LPI2C_Type *)LPI2C1_BASE );
        if( result == kStatus_Success ) {
            x = true;            
        } 
        return x;        
    }     
    return x;
}

/******************************************************************************/
/*!   \fn uint8_t readRegister( char slaveAdd, char reg )

      \brief
        This function reads data from the lp5521 register selected by the 
        variable reg and returns the result of the i2c read.
     
      \author
          Chris King
*******************************************************************************/
uint8_t readRegister( char slaveAdd, char reg )
{
    uint8_t g_master_rxBuff[1];
    g_master_rxBuff[0] = 0;

    status_t reVal        = kStatus_Fail;
    size_t txCount        = 0xFFU;
    
    
    if (kStatus_Success == LPI2C_MasterStart((LPI2C_Type *)LPI2C1_BASE, slaveAdd, kLPI2C_Write))
    {
        /* Check master tx FIFO empty or not */
        LPI2C_MasterGetFifoCounts((LPI2C_Type *)LPI2C1_BASE, NULL, &txCount);
        while (txCount)
        {
            LPI2C_MasterGetFifoCounts((LPI2C_Type *)LPI2C1_BASE, NULL, &txCount);
        }
        /* Check communicate with slave successful or not */
        if (LPI2C_MasterGetStatusFlags((LPI2C_Type *)LPI2C1_BASE) & kLPI2C_MasterNackDetectFlag)
        {
            return -1;
        }

        reVal = LPI2C_MasterSend((LPI2C_Type *)LPI2C1_BASE, (void *)&reg, 1);
        if (reVal != kStatus_Success)
        {
            if (reVal == kStatus_LPI2C_Nak)
            {
                LPI2C_MasterStop((LPI2C_Type *)LPI2C1_BASE);
            }
            return -1;
        }

        reVal = LPI2C_MasterRepeatedStart((LPI2C_Type *)LPI2C1_BASE, slaveAdd, kLPI2C_Read);
        if (reVal != kStatus_Success)
        {
            return -1;
        }

        reVal = LPI2C_MasterReceive((LPI2C_Type *)LPI2C1_BASE, g_master_rxBuff, 1);
        if (reVal != kStatus_Success)
        {
            if (reVal == kStatus_LPI2C_Nak)
            {
                LPI2C_MasterStop((LPI2C_Type *)LPI2C1_BASE);
            }
            return -1;
        }

        reVal = LPI2C_MasterStop((LPI2C_Type *)LPI2C1_BASE);
        if (reVal != kStatus_Success)
        {
            return -1;
        }
    }

    return g_master_rxBuff[0];
}

static void LP5521delay() 
{
    for( uint32_t i = 0U; i < 1000000; i++ ) {
        __NOP();
    }
}

bool getGapCalStatus( void )
{
    return gapCal_;
}

bool getTUCalStatus( void )
{
    return TUCal_;
}
