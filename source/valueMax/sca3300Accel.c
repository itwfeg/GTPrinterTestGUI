#include "valueMax.h"
#include "sca3300Accel.h"
#include "fsl_lpspi.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "fsl_debug_console.h"
#include "fsl_lpspi_freertos.h"
#include "threadManager.h"
#include "developmentSettings.h"


static bool     accelSPIReady    = true;  //serial EEP and accelerometer share the SPI port. EEP CS = high, Accel CS = low.

/******************************************************************************/
/*!   \fn static bool initSpiAccelerometer( void )

      \brief
        This function initializes the spi interface to the accelerometer ic.
        Since this interface is shared with the serial flash the function checks 
        for ownership and claims the interface. Function returns a false if 
        claiming of the interface fails.
      \author
          Aaron Swift
*******************************************************************************/
static bool initSpiAccelerometer( void )
{
    lpspi_master_config_t masterConfig;
    bool result = false;
    
    LPSPI_Reset(LPSPI3); //TFink 5/13/24. Before adding reset, baud rate didn't change
       
    LPSPI_MasterGetDefaultConfig( &masterConfig );
   
    /* interface is now claimed. we can safely use. */
    masterConfig.baudRate                      = SCA3300_MAX_CLK_FREQ;
    masterConfig.bitsPerFrame                  = 8;
    masterConfig.cpol                          = kLPSPI_ClockPolarityActiveHigh;
    masterConfig.cpha                          = kLPSPI_ClockPhaseFirstEdge;
    masterConfig.direction                     = kLPSPI_MsbFirst;
    masterConfig.pcsToSckDelayInNanoSec        = 250;             
    masterConfig.lastSckToPcsDelayInNanoSec    = 250;
    masterConfig.betweenTransferDelayInNanoSec = 250;
    masterConfig.whichPcs                      = kLPSPI_Pcs0;
    masterConfig.pcsActiveHighOrLow            = kLPSPI_PcsActiveLow;
    masterConfig.pinCfg                        = kLPSPI_SdiInSdoOut;
    masterConfig.dataOutConfig                 = kLpspiDataOutTristate;

    /* initialize the spi master */
    LPSPI_MasterInit( LPSPI3, &masterConfig, ( CLOCK_GetFreq(kCLOCK_Usb1PllPfd0Clk) / 8 ) );
    /* setup our interrupts */
    LPSPI_EnableInterrupts( LPSPI3, (kLPSPI_TxInterruptEnable | 
                               kLPSPI_RxInterruptEnable | 
                               kLPSPI_TransmitErrorInterruptEnable | 
                               kLPSPI_ReceiveErrorInterruptEnable ) );

    /* create interrupt handle */
    LPSPI_MasterTransferCreateHandle( LPSPI3, getVmSPIHandle(), (lpspi_master_transfer_callback_t)sca3300SpiCallBack, NULL );
    EnableIRQ( LPSPI3_IRQn );
    
    LPSPI_Enable( LPSPI3, false );
    LPSPI3->CFGR1  &= (~LPSPI_CFGR1_NOSTALL_MASK);
    LPSPI_Enable( LPSPI3, true );    
    
    result = true;
    setVmSPIConfigState(VM_SPI_INITIALIZED_FOR_SCA3300);
    
    return result;

}

/******************************************************************************/
/*!   \fn bool initSCA3300( SCA3300 *pSca )

      \brief
        This function initializes the SCA3300 accelerometer ic.
        To initialize the ic the following steps are needed.
    
        1. write sw reset cmd and wait 1mS 
        2. select operating mode and wait 15mS 
        3. if mode 4 wait 100mS 
        4. read status 3 times, last should be 01 
        5. read who am i regster 
        6. read serial number 
        
      \author
          Aaron Swift
*******************************************************************************/
bool initSCA3300( SCA3300 *pSca )
{
    bool result = false;
    
    /* #1 software reset the device.  */
    resetSca3300( pSca );

    /* #2 set mode. */
    changeMode( pSca, _MODE_4_1r5g );  
         
    /* #4  read status 3 times to clear error flags from powerup*/
    getDeviceStatus( pSca );
    getDeviceStatus( pSca );                      
    getDeviceStatus( pSca );
    
    /* check device status */
    if( pSca->dStatus == SCA_NORMAL_OP ) {    
        /* #5 check chip id */
        getDeviceId( pSca );        
        if( pSca->chipId == SCA3300_DEVICE_ID ) {
            /* #6 read the device temperature */
            getTemperature( pSca );
            if( pSca->vFrame )
                convertRawTemp( pSca );    
                result = true;
                
                PRINTF("initSCA3300(): SCA3300 initialized. Temperature  %.2f C\r\n", pSca->deviceTemp );
        } else {
            PRINTF("initSCA3300(): Unknown device id!: %d\r\n", pSca->chipId );    
        }            
    } else { 
        PRINTF("\r\n\r\n SCA3300 dStatus = %d, != SCA_NORMAL_OP!!!\r\n\r\n\r\n",  pSca->dStatus);
    }
    return result;
}

/******************************************************************************/
/*!   \fn static void resetSca3300( SCA3300 *pSca )

      \brief
        This function sends a software reset command through the spi interface 
        to the accelerometer ic and waits the required 1mS before returning.

      \author
          Aaron Swift
*******************************************************************************/
static void resetSca3300( SCA3300 *pSca )
{
    if( pSca != NULL ) {
        writeSCA3300( SCA_CMD_SW_RESET, NULL, sizeof(unsigned long) ); 
        delayTicks(1);  //Wait one tick. That's the minimum time we can Delay.
        PRINTF("resetSca3300 cmd complete\r\n");
    }    
}


/******************************************************************************/
/*!   \fn static void changeMode( SCA3300 *pSca, SCA_MODE sensitivity )

      \brief
        This function sends a change mode command through the spi interface 
        to the accelerometer ic and waits the required 15mS before returning.

      \author
          Aaron Swift
*******************************************************************************/
static void changeMode( SCA3300 *pSca, SCA_MODE sensitivity )
{    
    unsigned long cmd = 0;
    TickType_t waitTime = 20;  //mS Mode1,2,3 all = 15mS Mode4 = 100mS
    if( pSca != NULL ) {
        if( sensitivity == _MODE_1_3g ) {
            cmd = SCA_CMD_CHANGE_MODE1;
        } else if( sensitivity == _MODE_2_6g ) {
            cmd = SCA_CMD_CHANGE_MODE2;
        } else if( sensitivity == _MODE_3_15g ) {
            cmd = SCA_CMD_CHANGE_MODE3;
        } else {
            cmd = SCA_CMD_CHANGE_MODE4;
            waitTime = 105;
        }        
        writeSCA3300( cmd, NULL, sizeof(cmd) );
        /* allow settling of signal path */
        delayTicks(pdMS_TO_TICKS(waitTime));       
    } else {
        PRINTF("changeMode(): manager object is null!\r\n" );
    }        
}


/******************************************************************************/
/*!   \fn static void getDeviceStatus( SCA3300 *pSca )

      \brief
        This function sends a dummy command through the spi interface 
        to read the results from the pervious command. The frame crc is checked 
        and if valid the status is returned.

      \author
          Aaron Swift
*******************************************************************************/
static void getDeviceStatus( SCA3300 *pSca )
{
    unsigned long cmd = SCA_CMD_READ_STATUS, status = 0;
    if( pSca != NULL ) {
        writeSCA3300( cmd, NULL, sizeof(cmd) ); 
        
        delayTicks(1);
        
        cmd = SCA_READ_REPLY;
        writeSCA3300( cmd, (unsigned char *)&status, sizeof(cmd) );        
        parseReply( pSca, &status );
        /* if not a valid frame */
        if( !pSca->vFrame ) {
            pSca->dStatus = SCA_ERROR;   
        }            
    } else {
        PRINTF("getDeviceId(): manager object is null!\r\n" );
    }        
}

/******************************************************************************/
/*!   \fn void getDeviceId( SCA3300 *pSca )

      \brief
        This function sends a who am i command through the spi interface 
        to read the results from the pervious command. The frame crc is checked 
        and if valid the chip id is returned. 

      \author
          Aaron Swift
*******************************************************************************/
void getDeviceId( SCA3300 *pSca )
{
    unsigned long cmd = SCA_CMD_WHOAMI, id = 0;
    if( pSca != NULL ) {
        writeSCA3300( cmd, NULL, sizeof(cmd) ); 
    
        cmd = SCA_READ_REPLY;
        writeSCA3300( cmd, (unsigned char *)&id, sizeof(cmd) );        
        unsigned short data = parseReply( pSca, &id );

        if( pSca->vFrame ) {
            pSca->chipId = (unsigned char)( data & 0x00FF );
           // PRINTF("SCA3300 Chip ID = %2x\r\n", pSca->chipId);
        } else {
            pSca->chipId = 0;
            PRINTFThrottle(50,"getDeviceId(): invalid id response!\r\n" );
        }        
    } else {
        PRINTF("getDeviceId(): manager object is null!\r\n" );
    }    
}

/******************************************************************************/
/*!   \fn static void getTemperature( SCA3300 *pSca )

      \brief
        This function sends a temperature command through the spi interface 
        to read the results from the pervious command. The frame crc is checked 
        and if valid the raw device temperature is returned. 

      \author
          Aaron Swift
*******************************************************************************/
static void getTemperature( SCA3300 *pSca )
{
    unsigned long cmd = SCA_CMD_READ_TEMP, temp = 0;    
    
    if( pSca != NULL ) {
        writeSCA3300( cmd, NULL, sizeof(cmd) ); 
        
        delayTicks(1);
        
        cmd = SCA_READ_REPLY;
        writeSCA3300( cmd, (unsigned char *)&temp, sizeof(cmd) );        
        unsigned short data = parseReply( pSca, &temp );

        if( pSca->vFrame ) {
            pSca->temperature = data;
        } else {            
            pSca->temperature = 0;
            PRINTF("getTemperature(): invalid temperature response!\r\n" );
        }        
    
    } else {
        PRINTF("getTemperature(): manager object is null!\r\n" );
    }    
}

/******************************************************************************/
/*!   \fn static void convertRawTemp( SCA3300 *pSca )

      \brief
        This function converts the raw temperature to degrees °C.
        Temperature [°C] = -273 + (TEMP / 18.9)         p.4 of datasheet
 
      \author
          Aaron Swift
*******************************************************************************/
static void convertRawTemp( SCA3300 *pSca )
{
    pSca->deviceTemp = (float)( ( (float)pSca->temperature / 18.9 ) -273 );     
}

/******************************************************************************/
/*!   \fn void getAcceleration( SCA3300 *pSca, SCA_AXIS_TYPE axis )

      \brief
        This function reads the acceleration measured by the device for the 
        given axis. 
        
 
      \author
          Aaron Swift
*******************************************************************************/
void getAcceleration( SCA3300 *pSca, SCA_AXIS_TYPE axis )
{
    unsigned long accel = 0;
    
    if( pSca != NULL ) {
        if( axis == SCA_X_AXIS_ ) {
            writeSCA3300( SCA_CMD_READ_ACC_X, NULL, sizeof(accel) ); 
                     
            writeSCA3300( SCA_READ_REPLY, (unsigned char *)&accel, sizeof(accel) );        
            unsigned short data = parseReply( pSca, &accel );

            if( pSca->vFrame ) {
                pSca->xAccel = data;
            } else {            
                pSca->xAccel = 0;
                PRINTFThrottle(300,"getAcceleration(): invalid X acceleration response!\r\n" );
            }                    
        } else if( axis == SCA_Y_AXIS_ ) {
            writeSCA3300( SCA_CMD_READ_ACC_Y, NULL, sizeof(accel) );
                 
            writeSCA3300( SCA_READ_REPLY, (unsigned char *)&accel, sizeof(accel) );        
            unsigned short data = parseReply( pSca, &accel );

            if( pSca->vFrame ) {
                pSca->yAccel = data;
            } else {            
                pSca->yAccel = 0;
                PRINTFThrottle(300,"getAcceleration(): invalid Y acceleration response!\r\n" );
            }                                
        } else {
            writeSCA3300( SCA_CMD_READ_ACC_Z, NULL, sizeof(accel) );
                      
            writeSCA3300( SCA_READ_REPLY, (unsigned char *)&accel, sizeof(accel) );        
            unsigned short data = parseReply( pSca, &accel );

            if( pSca->vFrame ) {
                pSca->zAccel = data;
            } else {            
                pSca->zAccel = 0;
                PRINTFThrottle(300,"getAcceleration(): invalid Z acceleration response!\r\n");
            }                                
        }        
    } else {
        PRINTF("getAcceleration(): manager object is null!\r\n" );
    }
}

/******************************************************************************/
/*!   \fn static bool runSCASelfTest( SCA3300 *pSca )

      \brief
        This function send the self test command and reads the results. Self test
        provides information on signal saturation during vibration or shock events.
        At this time runSCASelfTest is only being used for debug (PRINTF). More
        tests will be required if we want to disable VM due to a failed self test
      
 
      \author
          Aaron Swift
*******************************************************************************/
bool runSCASelfTest( SCA3300 *pSca )
{
    unsigned long testResult = 0;
    bool result = false; 
    
    if( pSca != NULL ) {
        writeSCA3300( SCA_CMD_READ_STO, NULL, sizeof(testResult) ); 
        delayTicks(1);
                    
        writeSCA3300( SCA_READ_REPLY, (unsigned char *)&testResult, sizeof(testResult) );        
        short data = parseReply( pSca, &testResult );

        if( pSca->vFrame ) {
            /* check test results */
            if( ( data < 1400 ) && ( data > -1400 ) ) {
                pSca->selfTestOutput = data;
                PRINTF("runSCASelfTest(): Test passed!\r\n" );
                result = true;
            } else {
                PRINTF("runSCASelfTest(): Test failed: %d!\r\n", data );
            }
        } else {            
            pSca->selfTestOutput = 0;
            PRINTF("runSCASelfTest(): invalid self test response!\r\n" );
        }                    
    } else {
        PRINTF("runSCASelfTest(): manager object is null!\r\n" );
    } 
    return result;
}
 

/******************************************************************************/
/*!   \fn static void negateChipSelectSCA3300( void )

      \brief
        This function set the sca3300 chip select line high, which disables
        the SCA3300 interface
        
      \author
          Aaron Swift
*******************************************************************************/
static void negateChipSelectSCA3300( void )
{
    /* Negates SCA3300 CS, asserts 93C56 CS */
    GPIO_WritePinOutput( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, true );
    delay_uS(10);
}

/******************************************************************************/
/*!   \fn static void assertChipSelectSCA3300( void )

      \brief
        This function set the sca3300 chip select line low, which enables the 
        SCA3300 SPI interface.
        
      \author
          Aaron Swift
*******************************************************************************/
static void assertChipSelectSCA3300( void )
{    
    /* Asserts SCA3300 CS, negates 93C56 CS */
    GPIO_WritePinOutput( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, false ); 
}



/*! ****************************************************************************   
      \fn static void writeSCA3300( SCA3300 *pSca, unsigned long cmd, 
                                    unsigned char *pRxData, unsigned char size )
 
      \brief
        This function will write a command ( 32 bits ) through the spi interface 
        to the sca3300. 
                        
      \param   pSca: device manager, cmd: command, pRxData: recv data, size:
               frame size

      \author
          Aaron Swift
*******************************************************************************/ 
static void writeSCA3300(unsigned long cmd, unsigned char *pRxData, unsigned char size )
{  
    lpspi_transfer_t masterXfer;
    status_t status;
    
    if(!getLockVMSPI3Flash()) {
       PRINTF("\r\n\r\n\r\n writeSCA3300(); getLockVMSPI3Flash() Failed!\r\n\r\n\r\n");
       return;
    }
    
    if(getVmSPIConfigState() != VM_SPI_INITIALIZED_FOR_SCA3300)
       initSpiAccelerometer();
    
    assertChipSelectSCA3300();
    accelSPIReady = false;
    
    masterXfer.txData = (unsigned char *)&cmd;
    masterXfer.rxData = pRxData;
    masterXfer.dataSize = size; 
    masterXfer.configFlags = (uint32_t)kLPSPI_Pcs0 | (uint32_t)kLPSPI_MasterPcsContinuous; 

    status = LPSPI_MasterTransferNonBlocking( LPSPI3, getVmSPIHandle(), &masterXfer );
    if( status != kStatus_Success ) {
        PRINTF("writeSCA3300(): write to sca3300 failed: %d\r\n", status );
    }  
    
    unsigned short spiReadyTimeout = SCA3300_SPI_READY_TIMEOUT; 
    while( !getAccelSPIReady() && spiReadyTimeout) {
       spiReadyTimeout--; //At 8Mbps, we spin wheels for 4uS waiting for spiReady. Could add taskYield or possibly group multiple transfers using RT1024 capabilities
    }
    if(spiReadyTimeout == 0) 
       PRINTFThrottle(10,"writeSCA3300(): spi write timed out\r\n");
            
    negateChipSelectSCA3300();
    
    releaseLockVMSPI3Flash();
}

/******************************************************************************/
/*!   \fn static unsigned short parseReply(  SCA3300 *pSca, unsigned char *pRxData )

      \brief
        This function parses the reply frame from the sca3300 and updates the
        SCA3300 manager object and returns the message data if message valid.

      \author
          Aaron Swift
*******************************************************************************/
static unsigned short parseReply(  SCA3300 *pSca, unsigned long *pRxData )
{
    unsigned short data = 0;
    unsigned long frame = 0;
    
    /* re-order the frame */
    frame = ( ( *pRxData & 0x000000FF ) << 24 );
    frame |= ( ( ( *pRxData & 0x0000FF00 ) >> 8 ) << 16 );
    frame |= ( ( ( *pRxData & 0x00FF0000 ) >> 16 ) << 8 );
    frame |= ( ( *pRxData & 0xFF000000 ) >> 24 );

    /* validate the message first */
    pSca->rCrc = calcFrameCrc( pSca, frame );
    if( pSca->vFrame ) {
        /* get frame operational code */
        pSca->fOpCode = (unsigned char)( ( frame & SCA_OPC_MASK ) >> 24 );
        pSca->dStatus = (unsigned char)( ( frame & SCA_RS_MASK ) >> 24 );
        data = (unsigned short)( ( frame & SCA_REPLY_DATA_MASK ) >> 8 );
    } else {
        PRINTFThrottle(400,"parseReply(): SCA3300 Frame invalid!\r\n" );
    }
    return data;
}

/******************************************************************************/
/*!   \fn static unsigned char calcFrameCrc( SCA3300 *pSca, unsigned long frame )

      \brief
        This function calculates and verify the frame crc. 
        
      \author
          Aaron Swift
*******************************************************************************/
static unsigned char calcFrameCrc( SCA3300 *pSca, unsigned long frame )
{
    /* get the frame crc */
    unsigned char fCrc = (unsigned char)( frame & SCA_REPLY_CRC_MASK );
    /* calculate the frame crc */
    unsigned char crc = calculateSCA_CRC( frame );
    /* do they match? */
    if( fCrc == crc ) {
        pSca->vFrame = true;
    } else {
        pSca->vFrame = false;
    }
    return crc;
}

/******************************************************************************/
/*!   \fn static unsigned char calcFrameCrc( SCA3300 *pSca, unsigned long frame )

      \brief
        This function calculates crc for 24 MSB's of the 32 bit frame data. 
        
      \author
          Aaron Swift
*******************************************************************************/
static unsigned char calculateSCA_CRC( unsigned long data )
{
    /*Note: bits 0-7 is the CRC field and are not included in CRC calculation. */
    unsigned char index = 0x00, bitValue = 0x00, crc = 0xff;

    for( index = 31; index > 7; index-- ) {
        bitValue = (uint8_t)((data >> index) & 0x01);
        crc = calculateSCA_CRC8( bitValue, crc);
    }
    crc = (unsigned char)~crc;
    return crc;   
}

/******************************************************************************/
/*!   \fn static unsigned char calculateSCA_CRC8( unsigned char bitValue, unsigned char crc ))

      \brief
        This function calculates crc for byte of frame data. 
        
      \author
          Aaron Swift
*******************************************************************************/
static unsigned char calculateSCA_CRC8( unsigned char bitValue, unsigned char crc )
{
    unsigned char temp = 0;
    
    temp = (unsigned char)(crc & 0x80);
    if( bitValue == 0x01 ) {
        temp ^= 0x80;
    }
    crc <<= 1;
    if( temp > 0 ) {
        crc ^= 0x1D;
    }
    return crc;    
}


/******************************************************************************/
/*!   \fn void sca3300SpiCallBack( void )

      \brief
        This function is the callback for the accelerometer spi interface.    
      \author
          Aaron Swift
*******************************************************************************/
static void sca3300SpiCallBack( void )
{
    accelSPIReady = true;    
}

/******************************************************************************/
/*!   \fn void delay_mS(TickType_t delayTicks)

      \brief
         Relinquishes control for "delayTicks". In this system delayTicks = 5mS.
         This is the minimum delay. After 5mS the acceleromter task will be
         scheduled according to the task priorities. 

         A quick test showed the typical delay is 5-6mS when delayTicks = 1.
      \author
         T Fink
*******************************************************************************/
void delayTicks(TickType_t delayTicks)
{ 
   #if 0 
   /* TFink: The following code was caused a "return from task" error (we ended up
   in "vListInsert"). I couldn't figure out why. Probably good idea to
   understand why so I can avoid this issue in the future. It did it 
   occasionally from debugger with Accel board plugged in. It did it
   all the time when Accell board not plugged in. Have no idea why this 
   is because its just a delay. The accel interface is just an SPI. There
   are no interrupt pins.... */  
   
   startWaitTimer( waitTime );  
   while( !timeOut_ ) { taskYIELD(); } 
   
   #else
   vTaskDelay(delayTicks);
   #endif
}


/******************************************************************************/
/*!   \fn bool getAccelSPIReady(void)
          void setAccelSPIReadyFalse(void)
          lpspi_master_handle_t* getVmSPIHandle(void)

      \brief
        Getter and Setter functions to allow the Accel SPI channel to be used
        by eep93C56.c    
      \author
          Aaron Swift
*******************************************************************************/
bool getAccelSPIReady(void)
{
   return accelSPIReady;
}

void setAccelSPIReadyFalse(void)
{
   accelSPIReady = false;
}



