#include "valueMax.h"
#include "fsl_lpspi.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "fsl_debug_console.h"
#include "eep93C56.h"
#include "wgMessages.h"
#include "queueManager.h"
#include "developmentSettings.h"
#include "sca3300Accel.h"


/************ #defines ************************/
#define EEP_SPI_READY_TIMEOUT 20       //typical delay is 1 "vTaskDelay(1)"
#define MISO_LOW_TIMEOUT  100      //typical delay is 0 cycles.
#define MISO_HIGH_TIMEOUT 20       //typical delay is 2 cycles for write93C56Word , ?? for 

/*********** Variables ****************************/
static unsigned char            EEPPageBuffer[PAGE_SIZE_93C56];
static bool                     isEEPWriteEnabled = false; //93C56 has to be enabled before it can be written or erased
static bool                     eepSPIReady       = true;  //serial EEP and accelerometer share the SPI port. EEP CS = high, Accel CS = low. 

/******** Getter/Setter Functions ***************/
unsigned char *get93C56EEPPageBuffer( void )
{
   return &EEPPageBuffer[0];
}

 
/*************** Global Variables *****************/
static lpspi_master_config_t    masterConfig;            //Holds the SPI Master Configuration

/******************************************************************************/
/*!   \fn static bool initSpi93C56EEP( void )

      \brief
        This function initializes the spi interface for the 93C56 EEP.
        Since this interface is shared with the accelerometer the function checks 
        for ownership and claims the interface. Function returns a false if 
        claiming of the interface fails.
      \author
          Aaron Swift
*******************************************************************************/

bool initSpi93C56EEP( void )
{
    bool result = false;
    
    LPSPI_Reset(LPSPI3);
    
    LPSPI_MasterGetDefaultConfig( &masterConfig );
   
    /* Modify the default configuration */
    masterConfig.baudRate                      = EEP_93C56_CLK_FREQ;
    masterConfig.bitsPerFrame                  = 8;  
    masterConfig.cpol                          = kLPSPI_ClockPolarityActiveHigh;
    masterConfig.cpha                          = kLPSPI_ClockPhaseFirstEdge;
    masterConfig.direction                     = kLPSPI_MsbFirst;
    masterConfig.pcsToSckDelayInNanoSec        = 500;             
    masterConfig.lastSckToPcsDelayInNanoSec    = 500;
    masterConfig.betweenTransferDelayInNanoSec = 750;                  //A little more than normal clock period to make it easy to see byte boundry on logic analyzer.
    masterConfig.whichPcs                      = kLPSPI_Pcs0;          //We're not using automatic CS. 
    masterConfig.pcsActiveHighOrLow            = kLPSPI_PcsActiveLow;  //We're not using automatic CS. 
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
    LPSPI_MasterTransferCreateHandle( LPSPI3, getVmSPIHandle(), (lpspi_master_transfer_callback_t)eep93C56SpiCallBack, NULL );
    EnableIRQ( LPSPI3_IRQn );
    
    LPSPI_Enable( LPSPI3, false );
    LPSPI3->CFGR1  &= (~LPSPI_CFGR1_NOSTALL_MASK);
    LPSPI_Enable( LPSPI3, true ); 
     
    setVmSPIConfigState(VM_SPI_INITIALIZED_FOR_93C56);
    result = true;
    
    return result;

}


/*! ****************************************************************************   
      \fn static unsigned short read93C56EEPMultiByte (unsigned char addr)
 
      \brief
        Reads one word from the 93C56 from address "addr"

      \author
          Tom Fink
*******************************************************************************/ 
int32_t read93C56EEPMultiByte (unsigned char addr, unsigned char *pData, unsigned char numBytes)
{
    lpspi_transfer_t masterXfer;
    status_t status;
    uint32_t readStatus = kStatus_Success;
    unsigned char dataOut[2] = {_93C56_READ_CMD, addr/2};   //93C56 reads words, not bytes
    
    if(!getLockVMSPI3Flash()) {
       PRINTF("\r\n\r\n\r\n read93C56EEPMultiByte(); getLockVMSPI3Flash() Failed!\r\n\r\n\r\n");
       return (kStatus_Fail);
    }
    
    
    if(getVmSPIConfigState() != VM_SPI_INITIALIZED_FOR_93C56) 
      initSpi93C56EEP();
       
    /** Send the Command and Address Bytes **/
    assertChipSelect93C56();
    setEEPSPIReadyFalse();
    
    masterXfer.txData = dataOut;
    masterXfer.rxData = pData;
    masterXfer.dataSize = 2; 
    masterXfer.configFlags = (uint32_t)kLPSPI_Pcs0 | (uint32_t)kLPSPI_MasterPcsContinuous;   

    status = LPSPI_MasterTransferNonBlocking( LPSPI3, getVmSPIHandle(), &masterXfer );
    if( status != kStatus_Success ) {
        PRINTF("read93C56Word(): command write to 93C56 EEP failed: %d\r\n", status );
        readStatus = kStatus_Fail;
    }  
   
    unsigned long spiReadyTimeout = EEP_SPI_READY_TIMEOUT; 
    while( !getEEPSPIReady() && spiReadyTimeout) {
      spiReadyTimeout--;
      vTaskDelay(1); 
    }
    if(spiReadyTimeout == 0) {
       PRINTFThrottle(10,"read93C56Word(): spi timed out\r\n");
       readStatus = kStatus_Fail;
    }
      
    /** Read the words back **/
    /* 93C56 clocks data out on the RISING edge, so change the phase to data
       is clocked into the RT1024 on the falling edge. The Read command is the 
       only command that requires this phase setting, so the phase is changed back
       to the 1st edge before exiting this function */
    changeClkPhase2ndEdge();
    
    /* Clock the data word in from the 93C56 */
    setEEPSPIReadyFalse();
    dataOut[0] = 0x00;  //Data clocked out is "don't care"
    dataOut[1] = 0x00; 
    masterXfer.txData = dataOut;
    masterXfer.rxData = pData;
    masterXfer.dataSize = numBytes; 
    masterXfer.configFlags = (uint32_t)kLPSPI_Pcs0 | (uint32_t)kLPSPI_MasterPcsContinuous;   //| kLPSPI_MasterByteSwap
    
    
    status = LPSPI_MasterTransferNonBlocking( LPSPI3, getVmSPIHandle(), &masterXfer );
    if( status != kStatus_Success ) {
       PRINTF("read93C56Word(): read from 93C56 EEP failed: %d\r\n", status );
       readStatus = kStatus_Fail;
    }  
    
    spiReadyTimeout = EEP_SPI_READY_TIMEOUT; 
    while( !getEEPSPIReady() && spiReadyTimeout) {
       spiReadyTimeout--;
       vTaskDelay(1); 
    }
    if(spiReadyTimeout == 0) {
       PRINTFThrottle(10,"read93C56Word(): spi timed out\r\n");
       readStatus = kStatus_Fail;
    }
        
    negateChipSelect93C56();
    
    /* The 93C56 clocks data IN on the rising edge, so change phase back so the RT1024
       clocks data OUT on the falling edge. */
    changeClkPhase1stEdge(); 
    
    releaseLockVMSPI3Flash();
    return(readStatus);
  
}

#if 0  //depracated by read93C56EEPMultiByte
/*! ****************************************************************************   
      \fn static unsigned short read93C56Word (unsigned char addr)
 
      \brief
        Reads one word from the 93C56 from address "addr"

      \author
          Tom Fink
*******************************************************************************/ 
unsigned short read93C56Word (unsigned char addr)
{
    lpspi_transfer_t masterXfer;
    status_t status;
    unsigned char dataOut[2] = {_93C56_READ_CMD, addr/2};  //93C56 reads words, not bytes
    union charShort dataIn;
    
    if(!getLockVMSPI3Flash()) {
       PRINTF("\r\n\r\n\r\n read93C56Word(); getLockVMSPI3Flash() Failed!\r\n\r\n\r\n");
       return(0);
    }
      
    if(getVmSPIConfigState() != VM_SPI_INITIALIZED_FOR_93C56) 
      initSpi93C56EEP();
    
    /* Send the Command and Address Bytes */
    assertChipSelect93C56();
    setEEPSPIReadyFalse();
    
    masterXfer.txData = dataOut;
    masterXfer.rxData = dataIn.uc;
    masterXfer.dataSize = 2; 
    masterXfer.configFlags = (uint32_t)kLPSPI_Pcs0 | (uint32_t)kLPSPI_MasterPcsContinuous; 

    status = LPSPI_MasterTransferNonBlocking( LPSPI3, getVmSPIHandle(), &masterXfer );
    if( status != kStatus_Success ) {
        PRINTF("read93C56Word(): command write to 93C56 EEP failed: %d\r\n", status );
    }  
   
    unsigned long spiReadyTimeout = EEP_SPI_READY_TIMEOUT; 
    while( !getEEPSPIReady() && spiReadyTimeout) {
      spiReadyTimeout--;
      vTaskDelay(1); 
    }
    if(spiReadyTimeout == 0)
       PRINTFThrottle(10,"read93C56Word(): spi timed out\r\n");
    
    /* 93C56 clocks data out on the RISING edge, so change the phase to data
       is clocked into the RT1024 on the falling edge. The Read command is the 
       only command that requires this phase setting, so the phase is changed back
       to the 1st edge before exiting this function */
    changeClkPhase2ndEdge();
    
    /* Clock the data word in from the 93C56 */
    setEEPSPIReadyFalse();
    dataOut[0] = 0x00;  //Data clocked out is "don't care"
    dataOut[1] = 0x00; 
    dataIn.uc[0] =  0x00;  //Set data in to a known value so we recognize if it changes
    dataIn.uc[1] =  0x00;
    masterXfer.txData = dataOut;
    masterXfer.rxData = dataIn.uc;
    masterXfer.dataSize = 2; 
    masterXfer.configFlags = (uint32_t)kLPSPI_Pcs0 | (uint32_t)kLPSPI_MasterPcsContinuous; 
    
    
    status = LPSPI_MasterTransferNonBlocking( LPSPI3, getVmSPIHandle(), &masterXfer );
    if( status != kStatus_Success ) {
        PRINTF("read93C56Word(): read from 93C56 EEP failed: %d\r\n", status );
    }  
    
    spiReadyTimeout = EEP_SPI_READY_TIMEOUT; 
    while( !getEEPSPIReady() && spiReadyTimeout) {
      spiReadyTimeout--;
      vTaskDelay(1); 
    }
    if(spiReadyTimeout == 0)
       PRINTFThrottle(10,"read93C56Word(): spi timed out\r\n");
    
    negateChipSelect93C56();
    
    /* The 93C56 clocks data IN on the rising edge, so change phase back so the RT1024
       clocks data OUT on the falling edge. */
    changeClkPhase1stEdge();
    
    //PRINTF("EEP Addr %d: %x |%x, %x\r\n", addr,dataIn.us,dataIn.uc[1],dataIn.uc[0]);  
    
    releaseLockVMSPI3Flash();
    return (unsigned short) dataIn.us;  
}
#endif


/*! ****************************************************************************   
      \fn static void enable93C56WriteAndErase (void)
 
      \brief
         Enables 93C56 writes and erase cycles. Note that the 93C56 defaults 
         to the locked state on power up

      \author
          Tom Fink
*******************************************************************************/ 
void enable93C56WriteAndErase (void)
{
    lpspi_transfer_t masterXfer;
    status_t status;
    bool cmdSuccess = true;
    unsigned char dataOut[2] = {_93C56_ENABLE_CMD_1, _93C56_ENABLE_CMD_2}; 
    unsigned char dataIn[4];
    
    if(!getLockVMSPI3Flash()) {
       PRINTF("\r\n\r\n\r\n enable93C56WriteAndErase(); getLockVMSPI3Flash() Failed!\r\n\r\n\r\n");
       return;
    }
    
    if(getVmSPIConfigState() != VM_SPI_INITIALIZED_FOR_93C56) 
      initSpi93C56EEP();
    
    assertChipSelect93C56();
    setEEPSPIReadyFalse();
    
    masterXfer.txData = dataOut;
    masterXfer.rxData = dataIn;
    masterXfer.dataSize = 2; 
    masterXfer.configFlags = (uint32_t)kLPSPI_Pcs0 | (uint32_t)kLPSPI_MasterPcsContinuous; 

    status = LPSPI_MasterTransferNonBlocking( LPSPI3, getVmSPIHandle(), &masterXfer );
    if( status != kStatus_Success ) 
       cmdSuccess = false;
      
    unsigned long spiReadyTimeout = EEP_SPI_READY_TIMEOUT; 
    while( !getEEPSPIReady() && spiReadyTimeout) {
      spiReadyTimeout--;
      vTaskDelay(1); 
    }
    if(spiReadyTimeout == 0)
       cmdSuccess = false;
    
    /* Negate CS then implement delay so CS minimum negation time of 250nS isn't violated */
    negateChipSelect93C56();
    unsigned short delayCount = 50;   //Measured delay of about 500nS
    while(delayCount){delayCount--;}  
    
    if(cmdSuccess)
       isEEPWriteEnabled = true;
    else
       PRINTFThrottle(10,"enable93C56WriteAndErase (): command failed\r\n");
    
    releaseLockVMSPI3Flash();
}


/*! ****************************************************************************   
      \fn static void disable93C56WriteAndErase (void)
 
      \brief
         Disables 93C56 writes and erase cycles. Note that the 93C56 defaults 
         to the locked state on power up

      \author
          Tom Fink
*******************************************************************************/ 
void disable93C56WriteAndErase (void)
{
    lpspi_transfer_t masterXfer;
    unsigned char dataOut[2] = {_93C56_DISABLE_CMD_1, _93C56_DISABLE_CMD_2}; 
    unsigned char dataIn[4];
    
    if(!getLockVMSPI3Flash()) {
       PRINTF("\r\n\r\n\r\n disable93C56WriteAndErase(); getLockVMSPI3Flash() Failed!\r\n\r\n\r\n");
       return;
    }
    
    if(getVmSPIConfigState() != VM_SPI_INITIALIZED_FOR_93C56) 
      initSpi93C56EEP();
    
    assertChipSelect93C56();
    setEEPSPIReadyFalse();
    
    masterXfer.txData = dataOut;
    masterXfer.rxData = dataIn;
    masterXfer.dataSize = 2; 
    masterXfer.configFlags = (uint32_t)kLPSPI_Pcs0 | (uint32_t)kLPSPI_MasterPcsContinuous; 

    LPSPI_MasterTransferNonBlocking( LPSPI3, getVmSPIHandle(), &masterXfer );
      
    unsigned long spiReadyTimeout = EEP_SPI_READY_TIMEOUT; 
    while( !getEEPSPIReady() && spiReadyTimeout) {
      spiReadyTimeout--;
      vTaskDelay(1); 
    }
    
    /* Negate CS then implement delay so CS minimum negation time of 250nS isn't violated */
    negateChipSelect93C56();
    unsigned short delayCount = 50;   //Measured delay of about 500nS
    while(delayCount){delayCount--;}  

    releaseLockVMSPI3Flash();    
  
    isEEPWriteEnabled = false;
}

/*! ****************************************************************************   
      \fn void write93C56MultiWord (void)
 
      \brief
         Writes multiple words to the 93C56. Note that they have to be whole
         words (can't write an odd# of bytes) 

      \author
          Tom Fink
*******************************************************************************/ 
int write93C56MultiWord  (unsigned char addr, unsigned char *pData, unsigned char numBytes)
{
   int writeStatus = kStatus_Success;
   unsigned char i = 0;
   union charShort dataToWrite;
     
   enable93C56WriteAndErase();  //93C56 has to be enabled before data can be written to it.
   if(isEEPWriteEnabled == false)
      writeStatus = kStatus_Fail;
   
   while(i < numBytes)
   { 
      dataToWrite.uc[1] = pData[i];
      dataToWrite.uc[0] = pData[i+1];
        
      if(!write93C56Word (addr, dataToWrite.us)) 
         writeStatus = kStatus_Fail;  
      
      i +=2; 
      addr +=2;  
   }
     
   /* 99.999% of the time we will not be writting to the EEP. Since EEP shares a Chip Select with
      the SCA3300 accelerometer, play it safe and disable EEP Write */
   disable93C56WriteAndErase();  
      
   return(writeStatus);
   
}




/*! ****************************************************************************   
      \fn void write93C56Word (void)
 
      \brief
         Writes one word to the 93C56 

      \author
          Tom Fink
*******************************************************************************/ 

static bool write93C56Word (unsigned char addr, unsigned short data)
{
    union charShort dataToWrite; 
    lpspi_transfer_t masterXfer;
    status_t status;
    bool cmdSuccess = true;
    unsigned char dataOut[4];
    unsigned char dataIn[4];
    
    if(!getLockVMSPI3Flash()) {
       PRINTF("\r\n\r\n\r\n write93C56Word(); getLockVMSPI3Flash() Failed!\r\n\r\n\r\n");
       return(false);
    }                

   
    if(getVmSPIConfigState() != VM_SPI_INITIALIZED_FOR_93C56) 
      initSpi93C56EEP();
    
    if(!isEEPWriteEnabled)
       enable93C56WriteAndErase();  //93C56 has to be enabled before data can be written to it.
    
    dataToWrite.us = data;
    dataOut[0] = _93C56_WRITE_CMD;
    dataOut[1] = addr/2;       //93C56 reads words, not bytes
    dataOut[2] = dataToWrite.uc[1];
    dataOut[3] = dataToWrite.uc[0];
      
    assertChipSelect93C56();
    setEEPSPIReadyFalse();
    
    masterXfer.txData = dataOut;
    masterXfer.rxData = dataIn;
    masterXfer.dataSize = 4; 
    masterXfer.configFlags = (uint32_t)kLPSPI_Pcs0 | (uint32_t)kLPSPI_MasterPcsContinuous; 

    status = LPSPI_MasterTransferNonBlocking( LPSPI3, getVmSPIHandle(), &masterXfer );
    if( status != kStatus_Success ) 
        cmdSuccess = false;
     
    unsigned long spiReadyTimeout = EEP_SPI_READY_TIMEOUT; 
    while( !getEEPSPIReady() && spiReadyTimeout) {
      spiReadyTimeout--;
      vTaskDelay(1); 
    }
    
    if(spiReadyTimeout == 0)
        cmdSuccess = false; 
      
    /* The SPI write is done, so now Use the MISO pin as Ready/Busy input to determine
       when EEP is done with the write cycle */
    configurePinAD_B1_15AsGPIO();  
    
    negateChipSelect93C56();
    //Hold CS negated for at least 250nS
    unsigned short delayCount = 50;
    while(delayCount){delayCount--;}   
    assertChipSelect93C56(); //measured 500nS negative pulse with delay count set to 50. TFink 2/23/24
          
    /* Look for EEP MISO pin to go low indicating the  EEP is busy executing the command */
    unsigned long misoLowTimeout = MISO_LOW_TIMEOUT;  
    while((GPIO_ReadPinInput( ACCEL_SPI_MISO_GPIO, ACCEL_SPI_MISO_PIN ) == 1) && (misoLowTimeout != 0)) 
       misoLowTimeout--;
    
    if(misoLowTimeout == 0)
       cmdSuccess = false;

    /* Now wait for MISO input to go high. This will indicate the EEP has completed the write command */
    unsigned long misoHighTimeout = MISO_HIGH_TIMEOUT; 
    while((GPIO_ReadPinInput( ACCEL_SPI_MISO_GPIO, ACCEL_SPI_MISO_PIN ) == 0) && (misoHighTimeout != 0))
    {
       vTaskDelay(1);   //Delay 1 tick - the minimum
       misoHighTimeout--;
    }
    
    if(misoHighTimeout == 0)
       cmdSuccess = false;
          
    /* Cycle has finished executing */
    negateChipSelect93C56();

    /* Switch pin AD_B1_15 back to MISO function for next SPI command */
    configurePinAD_B1_15AsSPI3MISO();  
       
    if(!cmdSuccess)
      PRINTF("\r\nwrite93C56Word() command failed!\r\n\r\n"); 
    
    //debug only
    PRINTF("%d while loop cycles for EEP Write Command to finish\r\n", EEP_SPI_READY_TIMEOUT-spiReadyTimeout);
    PRINTF("%d while loop cycles for EEP MISO pin to go low\r\n", MISO_LOW_TIMEOUT-misoLowTimeout);
    PRINTF("%d while loop cycles from MISO low to MISO high\r\n\r\n\r\n", MISO_HIGH_TIMEOUT-misoHighTimeout);
    
    releaseLockVMSPI3Flash();
    return(cmdSuccess);
}


#if 0 
/*! ****************************************************************************   
      \fn void writeAll93C56 (void)
 
      \brief
         Writes all words of 93C56 with the last two bytes of "dataOut". Use this 
         function for development test. Change data to 0xFFFF to make it an
         "Erase All" function.

      \author
          Tom Fink
*******************************************************************************/ 
bool writeAll93C56 (void)
{
    lpspi_transfer_t masterXfer;
    status_t status;
    bool cmdSuccess = true;
    unsigned char dataOut[4] = {_93C56_WRITE_ALL_CMD_1, _93C56_WRITE_ALL_CMD_2, 0x98, 0x76};  
    unsigned char dataIn[4];
    
    if(getVmSPIConfigState() != VM_SPI_INITIALIZED_FOR_93C56) 
      initSpi93C56EEP();
    
    assertChipSelect93C56();
    setEEPSPIReadyFalse();
    
    masterXfer.txData = dataOut;
    masterXfer.rxData = dataIn;
    masterXfer.dataSize = 4; 
    masterXfer.configFlags = (uint32_t)kLPSPI_Pcs0 | (uint32_t)kLPSPI_MasterPcsContinuous; 

    status = LPSPI_MasterTransferNonBlocking( LPSPI3, getVmSPIHandle(), &masterXfer );
    if( status != kStatus_Success ) 
      cmdSuccess = false;
    
    unsigned long spiReadyTimeout = EEP_SPI_READY_TIMEOUT; 
    while( !getEEPSPIReady() && spiReadyTimeout) {
       spiReadyTimeout--;
       vTaskDelay(1);
    } 
    
    if(spiReadyTimeout == 0)
      cmdSuccess = false;
        
    /* The SPI write is done, so now Use the MISO pin as Ready/Busy input to determine
       when EEP is done with the write cycle */   
    configurePinAD_B1_15AsGPIO();
    
    negateChipSelect93C56();
    //Hold CS negated for at least 250nS
    unsigned short delayCount = 50;
    while(delayCount){delayCount--;}   
    assertChipSelect93C56(); //measured 500nS negative pulse with delay count set to 50. TFink 2/23/24
    
    /* Look for EEP MISO pin to go low */
    unsigned long misoLowTimeout = MISO_LOW_TIMEOUT;  
    while((GPIO_ReadPinInput( ACCEL_SPI_MISO_GPIO, ACCEL_SPI_MISO_PIN ) == 1) && (misoLowTimeout != 0))
       misoLowTimeout--;
    
    if(misoLowTimeout == 0)
       cmdSuccess = false;

    /* Now wait for MISO input to go high. This will indicate the EEP has completed its command */
    unsigned long misoHighTimeout = MISO_HIGH_TIMEOUT; 
    while((GPIO_ReadPinInput( ACCEL_SPI_MISO_GPIO, ACCEL_SPI_MISO_PIN ) == 0) && (misoHighTimeout != 0)) {
       vTaskDelay(1);   //Delay 1 tick - the minimum
       misoHighTimeout--;
    }
    
    if(misoHighTimeout == 0)
       cmdSuccess = false;
          
    /* Cycle has finished executing */
    negateChipSelect93C56();

    /* Switch pin AD_B1_15 back to MISO function for next SPI command */
    configurePinAD_B1_15AsSPI3MISO();  
    
    if(!cmdSuccess)
      PRINTF("\r\nwriteAll93C56() command failed!!\r\n\r\n"); 
    
    //debug only
    //PRINTF("%d while loop cycles for EEP Write Command to finish\r\n", EEP_SPI_READY_TIMEOUT-spiReadyTimeout);
    //PRINTF("%d while loop cycles for EEP MISO pin to go low\r\n", MISO_LOW_TIMEOUT-misoLowTimeout);
    //PRINTF("%d while loop cycles from MISO low to MISO high\r\n\r\n\r\n", MISO_HIGH_TIMEOUT-misoHighTimeout);
    
    return(cmdSuccess);
}
#endif


/******************************************************************************/
/*!   \fn static void assert/negateChipSelect93C56( void )

      \brief
          93C56 is enabled when CS is high

      \author
          Tom Fink
*******************************************************************************/
static void assertChipSelect93C56( void )
{
     /* Negates SCA3300 CS, asserts 93C56 CS */
    GPIO_WritePinOutput( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, true );
}

static void negateChipSelect93C56( void )
{    
     /* Asserts SCA3300 CS, negates 93C56 CS */
    GPIO_WritePinOutput( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, false ); 
}

/******************************************************************************/
/*!   \fn static void changeClkPhase2ndEdge(void)

      \brief
          Clock data in on the second clock edge (high to low edge in this
          case)

      \author
          Tom Fink
*******************************************************************************/
static void changeClkPhase2ndEdge(void)
{  
    masterConfig.cpha = kLPSPI_ClockPhaseSecondEdge; //kLPSPI_ClockPhaseFirstEdge;

    /* initialize the spi master */
    LPSPI_MasterInit( LPSPI3, &masterConfig, ( CLOCK_GetFreq(kCLOCK_Usb1PllPfd0Clk) / 8 ) );
}

/******************************************************************************/
/*!   \fn static void changeClkPhase1stEdge(void)

      \brief
           Clock data in on the first clock edge (low to high edge in this
          case)
      \author
          Tom Fink
*******************************************************************************/
static void changeClkPhase1stEdge(void)
{  
    masterConfig.cpha = kLPSPI_ClockPhaseFirstEdge;

    /* initialize the spi master */
    LPSPI_MasterInit( LPSPI3, &masterConfig, ( CLOCK_GetFreq(kCLOCK_Usb1PllPfd0Clk) / 8 ) );
}

/******************************************************************************/
/*!   \fn void 93C56SpiCallBack( void )

      \brief
        This function is the callback for the 93C56 EEP spi interface.    
      \author
          Aaron Swift
*******************************************************************************/
static void eep93C56SpiCallBack( void )
{
    eepSPIReady = true;    
}


static bool getEEPSPIReady(void)
{
   return eepSPIReady;
}

static void setEEPSPIReadyFalse(void)
{
   eepSPIReady = false;
}