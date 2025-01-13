#include <math.h>
#include "valueMax.h"
#include "sca3300Accel.h"
#include "fsl_lpspi.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "eep93C56.h"
#include "averyWeigher.h"
#include "queueManager.h"
#include "translator.h"
#include "developmentSettings.h"


#ifndef PI
  #define PI                    3.14159265358979f
#endif
static unsigned char            vmEEPPageBuffer[PAGE_SIZE_93C56] = "0123456789";
static TaskHandle_t             vmTaskHandle       = NULL;
static bool                     suspendVMTask_     = false;
static SCA3300                  sca3300_;
static ValueMax                 valueMax_;
static vmSPIInitStates          vmSPIConfigState   = VM_SPI_UNINITIALIZED;
static lpspi_master_handle_t    vmSpiHandle_;
static QueueHandle_t            valueMaxQHandle = NULL;
            
#define SEND_MSG_TO_APP         true      
#define DO_NOT_SEND_MSG         false

/******************************************************************************/
/*!   \fn BaseType_t createValueMaxTask( void )

      \brief
        This function creates a task to manager the sca3300 accelerometer. 
   
      \author
          Aaron Swift
*******************************************************************************/
BaseType_t createValueMaxTask( void )
{  
    BaseType_t result = pdFALSE;
    
    memset( &sca3300_, 0, sizeof(SCA3300) );

    PRINTF("createValueMaxTask(): Starting...\r\n" );
    /* initialize the chip select signal */    
    
    memset( &valueMax_, 0, sizeof(ValueMax) );
    
    valueMaxQHandle = getVMQueueHandle();  
         
    /* create accelerometer task thread */
    result = xTaskCreate( valueMaxTask,  "ValueMaxTask", vmTaskStackSize,
                                        NULL, vm_task_PRIORITY, &vmTaskHandle );
        
    return result;
}


/******************************************************************************/
/*!   \fn void valueMaxTask( void *pvParameters )

      \brief
        This function manages the sca3300 accelerometer. 
   
      \author
          Aaron Swift
*******************************************************************************/
static void valueMaxTask( void *pvParameters )
{
    ( void ) pvParameters;     
    PRINTF("valueMaxTask(): Thread running...\r\n" ); 
    
    GPIO_WritePinOutput( ACCEL_PWR_EN_GPIO, ACCEL_PWR_EN_PIN, true ); //Enable VM Power
    vTaskDelay(pdMS_TO_TICKS(30)); //Give the linear power supply time to ramp up. Measured time is 6mS
      
    initVMSPI3Mutex();
    
    while( !suspendVMTask_ ) {  
         
      
       /******************* Initialize VM EEP *************************/
       if(valueMax_.vmEEP_initialized != true) {
    
          bool vmEEPInitSucceeded = initValueMaxEEP();
          
          if(vmEEPInitSucceeded != true) 
             vmEEPInitSucceeded = initValueMaxEEP();           //try one more time.
          
          if(vmEEPInitSucceeded == true)
             valueMax_.vmEEP_initialized = true;
          else {
             handleVMEEPError();
          }
          
          #ifdef TFinkWriteEEPDefaults
          write_VM_EEP_Defaults();
          #endif
       }
       

             
       /***********  Initialize SCA3300 Accelerometer ****************************/
       if(valueMax_.vmEEP_initialized == true && valueMax_.acceleromter_initialized == false) {
          if(initSCA3300( &sca3300_ )) {
             if(runSCASelfTest( &sca3300_ ) == true) {
                valueMax_.acceleromter_initialized 	= true;
             }
          }
          else
             handleVMAccelError();
       } 
       

       /********** SCA3300 is initialized. Now read it indefinitely *************/
       if(valueMax_.vmEEP_initialized == true && valueMax_.acceleromter_initialized == true &&isValueMaxOn() == true ) {     
          
         //Read a known value to verify the connection to the SCA3300
         getDeviceId( &sca3300_);        
         if(sca3300_.chipId != SCA3300_DEVICE_ID ){
            /* see "monitorValueMax()" in FS code. This code implements that functionality. */
            handleVMAccelError(); 
            PRINTFThrottle(10,"SCA3300 ChipID read failed!\r\n");
         }
         else
         {
            getAcceleration( &sca3300_, SCA_X_AXIS_ );
            getAcceleration( &sca3300_, SCA_Y_AXIS_ );          
            getAcceleration( &sca3300_, SCA_Z_AXIS_ );
            
            /* track samples for use in filter */
            if(valueMax_.acceleromter_sample_counter == MAX_ACCEL_RUNNING_SAMPLES)
               valueMax_.acceleromter_sample_counter = 0;                 
            
            /* increment sample counter */
            valueMax_.acceleromter_sample_counter++;
            
            valueMaxHandleAcclMeasurement(&valueMax_,&sca3300_ );          
         }
           
         #ifdef TFinkAlternateAngleCalc
         alternateAngleCalculation();  
         #endif
       }
       else
          PRINTFThrottle(0,"ValueMax not initialized or enabled\r\n");
       
       valueMaxTransactionID id = VM_EEP_NO_TRANSACTION;
       if( xQueueReceive( valueMaxQHandle, &id, 0 ) ) { 
          handleVMQueueTransactionRequest(id);                          
       } 
 
       vTaskDelay(pdMS_TO_TICKS(20)); //TFink 5/13/24 - 20mS seems to give fast enough acclerometer response. With 8MHz clock Accel Data read takes about 400uS
    
    }
    taskYIELD();
    vTaskSuspend(NULL); 
}

/******************************************************************************/
/*!   \fn static void handleVMQueueTransactionRequest(valueMaxTransactionID id)

      \brief
         Calls the appropriate function placed on the VM Queue by other tasks.
         It's important other tasks don't call these functions directly because
         they aren't necessarily re-entrant.
   
      \author
          Tom Fink
*******************************************************************************/
static void handleVMQueueTransactionRequest(valueMaxTransactionID id)
{    
   if(id == VM_EEP_READ_ASSEMBLY_SEND) {
      PRINTF("handleVMQueueTransactionRequest(): readVmAssy \r\n");
      readVmAssemblyNumber( SEND_MSG_TO_APP );
   }
   else if(id == VM_EEP_WRITE_ASSEMBLY) {
      PRINTF("handleVMQueueTransactionRequest(): writeVmAssy \r\n");
      writeVmAssemblyNumber( valueMax_.assemblyNumber, VM_ASSEMBLY_MAX );
   }
   else if(id == VM_EEP_READ_VERSION_SEND) {
      PRINTF("handleVMQueueTransactionRequest(): readVMVer \r\n");
      readVmVersion( SEND_MSG_TO_APP );
   }
   else if(id == VM_EEP_WRITE_VERSION) {
      PRINTF("handleVMQueueTransactionRequest(): writeVMVer \r\n");
      writeVmVersion( valueMax_.version, sizeof(valueMax_.version) );
   }
   else if(id == VM_EEP_READ_SERIAL_SEND) {
      PRINTF("handleVMQueueTransactionRequest(): readVMSer \r\n");
      readVmSerialNumber( SEND_MSG_TO_APP );
   }
   else if(id == VM_EEP_WRITE_SERIAL) {
      PRINTF("handleVMQueueTransactionRequest(): writeVMSer \r\n");
      writeVmSerialNumber( &valueMax_.serial[0], VM_SERIAL_MAX );   
   } 
   else if(id == VM_EEP_READ_DATE_SEND) {
      PRINTF("handleVMQueueTransactionRequest(): readVMDate \r\n");
      readVmDate( SEND_MSG_TO_APP );
   }
   else if(id == VM_EEP_WRITE_DATE) {
      PRINTF("handleVMQueueTransactionRequest(): writeVMDate \r\n");
      writeVmDate( (unsigned char *)&valueMax_.epoch, sizeof(valueMax_.epoch) );  //Storing as unsigned long in EEP. But have to cast to uChar....
   }
   else if(id == VM_EEP_WRITE_FIT_CALIBRATION) {
      PRINTF("handleVMQueueTransactionRequest(): writeVMFit\r\n");
      writeFitCalibrationAll(); 
   }  
}

 
/******************************************************************************/
/*!   \fn void initValueMaxEEP(void)

      \brief
         Read and validate Manufacturing Data and FIT values from the 
         Value Max EEPROM on the ValueMax PCBA (separate from the Weigher EEP)
   
      \author
          Tom Fink
*******************************************************************************/
static bool initValueMaxEEP(void)
{
   bool valueMaxEEPInitSucceeded = true;
   
    /*****************  Read Manufacturing Data ***************************/
   if( readVmAssemblyNumber(DO_NOT_SEND_MSG) != kStatus_Success ) 
      valueMaxEEPInitSucceeded = false;
   
   if( readVmVersion(DO_NOT_SEND_MSG) != kStatus_Success )
      valueMaxEEPInitSucceeded = false;
    
   if( readVmSerialNumber(DO_NOT_SEND_MSG) != kStatus_Success )
      valueMaxEEPInitSucceeded = false;
   
   if( readVmDate(DO_NOT_SEND_MSG) != kStatus_Success ) 
      valueMaxEEPInitSucceeded = false;
  
   /*****************  Read FIT values ***************************/
   unsigned char i                = 0;
   long checksum_val              = 0;
   unsigned char *vmFitPtr        = (unsigned char*)&valueMax_.fit_data;
   
   memset(&valueMax_.fit_data, 0, sizeof(fit_flash_section));
     
   for(i = 0; i<VM_EEP_TOTAL_PAGES; i++) {
      if( readFitCalibration(i) == kStatus_Success) {
         if(i < VM_EEP_TOTAL_PAGES-1) {
           memcpy(&vmFitPtr[i*PAGE_SIZE_93C56], (void*)vmEEPPageBuffer, PAGE_SIZE_93C56 );         
         }
         else {
            unsigned char last_payload_data_length;
            
            last_payload_data_length = sizeof(fit_flash_section) % PAGE_SIZE_93C56;
            // Copy last payload data size
            memcpy(&vmFitPtr[i*PAGE_SIZE_93C56], (void*)vmEEPPageBuffer, last_payload_data_length );           
         }     
      }
      else     
        valueMaxEEPInitSucceeded = false;     
   }
   
   /* Now let's verify checksum */
   while ( vmFitPtr < (unsigned char *) &valueMax_.fit_data.checksum )
      checksum_val += *vmFitPtr++;
   
   if (valueMax_.fit_data.checksum == checksum_val) {  //checksum is part of fit data we just loaded
      /* load global fit struct with received values */
      PrintVMFitValues();
      PRINTF("\r\nVM Checksums Matched!!\r\n\r\n");
      /* not convert discrete values to floating point */
      initFit(&valueMax_.fit_data);
      
   } else 
      PRINTF("\r\nBad FIT checksum! \r\n");
         
   return(valueMaxEEPInitSucceeded);
}


/******************************************************************************/
/*!   \fn void handleVMEEPError(void)

      \brief
         disable value max functionality due to an EEP read error. Report the 
         failure to the application.
   
      \author
          Tom Fink
*******************************************************************************/
static void handleVMEEPError(void)
{

   valueMax_.vmEEP_initialized = false;
   valueMax_.acceleromter_initialized =  false; //This forces calibrateCounts() to not use valueMax in calculating weight
   
   PRINTF("\r\nhandleVMEEPError(): WEIGHER_VM_EEP_COMM_FAILURE. \r\n");
 
   /* Send error to UI - Either of these failures should have same end result */
   setWeigherStatusBit(WEIGHER_VM_EEP_CHECKSUM_FAILURE);  
   setWeigherStatusBit(WEIGHER_VM_EEP_COMM_FAILURE); 
}

/******************************************************************************/
/*!   \fn void handleVMEEPError(void)

      \brief
         disable value max functionality due to an EEP or Accel read error. 
         Report the failure to the application.
   
      \author
          Tom Fink
*******************************************************************************/
static void handleVMAccelError(void)
{
   /* TFink 5/14/24 - pulled Accel PCBA cable during operatin and App displayed
      "weigher has encountered a VM EEP Comm Failure. Reboot/Cancel */
   valueMax_.acceleromter_initialized = false; 
   
   setWeigherStatusBit(WEIGHER_VM_ACCL_COMM_FAILURE);
   
   PRINTF("\r\n\r\nhandleVMAccelError(): WEIGHER_VM_ACCL_COMM_FAILURE\r\n\r\n");
}


/******************************************************************************/
/*!   \fn isVmPCBAssyNumValid

      \brief
        Validates the assy#. In Fresh Select, an I2C EEPROM was used. I2C was
        able to detect if the Accel EEP was not present (NACK error). SPI can't do
        this. Because we don't have a pullup on MISO, we can't rely on an input
        value of 0xFF. So do a bit more validation.

        Does the RT1024 have a configurable pullup? Doesn't seem to
        have one in the SPI configuration. Will the GPIO pullup work? If can
        pull up, then no EEPROM will give a blank char. In the meantime, look
        for the '-' which should be the third character  
   
      \author
          Tom Fink
*******************************************************************************/
bool isVmPCBAssyNumValid(unsigned char *assyNum)
{
   
   /* The GT never asks for VM Assy#, Rev, etc. They are used for nothing as of 5/2024
      No point in disabling VM because of bad Assy#. Only thing that really matters
      at this point is if the FIT checksum matches! */
#ifdef TFinkToDo1  
   bool assyNumValid = true;  
   
   if( assyNum[0] == BLANK_CHAR_93C56 )
      assyNumValid = false;
   else if ( assyNum[2] != '-' )     //All Hobart Ass#s have a '-' as the third char
      assyNumValid = false;
   else if (assyNum[3] == 0 && assyNum[4] == 0 && assyNum[5] == 0 && assyNum[6] == 0 && assyNum[7] == 0 && assyNum[8] == 0)
      assyNumValid = false; 
   
   return(assyNumValid);
#else
   return true;
#endif
}


/*********************************************************************************************************
***************************** Read VM Config from EEP Functions ******************************************
**********************************************************************************************************/
/******************************************************************************/
/*!   \fn void readVmAssemblyNumber( bool send )

      \brief
        This function reads the value max serial eeprom page of the assembly number.
   
      \author
          Aaron Swift
*******************************************************************************/
static status_t readVmAssemblyNumber( bool send )
{   
    status_t result = kStatus_Success;
    
    clearAccelEEPPageBuffer();
    
    result = read93C56EEPMultiByte(VM_EEP_BD_ASSY_NUM_ADDR, vmEEPPageBuffer, VM_ASSEMBLY_MAX);
   
 
    if( isVmPCBAssyNumValid(vmEEPPageBuffer)) {
      strncpy((char *) &valueMax_.assemblyNumber[0],(const char *) vmEEPPageBuffer, VM_ASSEMBLY_MAX );
      PRINTF("value max assembly number: %s \r\n", vmEEPPageBuffer ); 
    }
    else
       result = kStatus_Fail;
    
    if(send) {
       WgVMWrAssembly msg;
       strncpy((char *) &msg.assemblyNumber[0], (const char *)vmEEPPageBuffer, VM_ASSEMBLY_MAX );
       sendWgVmAssembly( &msg );
    }
                    
    if( result != kStatus_Success) 
      PRINTF("readVmAssemblyNumber(): failed to read value max assembly number!\r\n" );            
                
    return result;
}


/******************************************************************************/
/*!   \fn status_t readVmVersion( bool send )

      \brief
        This function reads the value max serial eeprom page of the version number.
   
      \author
          Aaron Swift
*******************************************************************************/
static status_t readVmVersion( bool send )
{    
    status_t result = kStatus_Success;
    
    clearAccelEEPPageBuffer();

    result = read93C56EEPMultiByte(VM_EEP_BD_HW_VER_ADDR, vmEEPPageBuffer, VM_VER_MAX );
    
    if( vmEEPPageBuffer[0] != BLANK_CHAR_93C56 ) {
       strncpy((char *) &valueMax_.version[0],(const char *) vmEEPPageBuffer, VM_VER_MAX );
       vmEEPPageBuffer[2] = '\0';
       PRINTF("value max HW version is: %s \r\n", vmEEPPageBuffer ); 
    }
    
    if(send) {
       WgVMWrVersion msg;
       strncpy((char *)&msg.version[0], (const char *)vmEEPPageBuffer, VM_VER_MAX );
       sendWgVmVersion( &msg ); 
    }
    
    if( result != kStatus_Success)  
        PRINTF("readVmVersion(): failed to read value max version!\r\n" );     
    
    
    return result;
}



/******************************************************************************/
/*!   \fn status_t readVmSerialNumber( bool send )

      \brief
        This function reads the value max serial eeprom page of the serial number.
   
      \author
          Aaron Swift
*******************************************************************************/
static status_t readVmSerialNumber( bool send )
{    
    status_t result = kStatus_Success;
    
    clearAccelEEPPageBuffer();
   
    result = read93C56EEPMultiByte(VM_EEP_BD_SER_NUM_ADDR, vmEEPPageBuffer, VM_SERIAL_MAX );
    
    if( vmEEPPageBuffer[0] != BLANK_CHAR_93C56 ) {                
       strncpy((char *) &valueMax_.serial[0],(const char *) vmEEPPageBuffer, VM_SERIAL_MAX );              
       PRINTF("value max serial number: %s \r\n", vmEEPPageBuffer );           
    }
    
    if(send) {
       WgVMWrSerial msg;
       strncpy((char *) &msg.serial[0], (const char *)vmEEPPageBuffer, VM_SERIAL_MAX );
       sendWgVmSerial( &msg );
    }
    
    if( result != kStatus_Success ) 
        PRINTF("readVmSerialNumber(): failed to read value max serial number!\r\n" );    
     
    return result;
}


/******************************************************************************/
/*!   \fn status_t readVmDate( bool send )

      \brief
        This function reads the value max serial eeprom page of the date.
   
      \author
          Aaron Swift
*******************************************************************************/
static status_t readVmDate( bool send )
{    
    status_t result = kStatus_Success;
   
    clearAccelEEPPageBuffer();
    result = read93C56EEPMultiByte(VM_EEP_BD_MFR_DATE_ADDR, vmEEPPageBuffer, sizeof(unsigned long) );
     
    strncpy((char *) &valueMax_.epoch, (const char *)vmEEPPageBuffer, sizeof(unsigned long) );                
    PRINTF("value max date: %d \r\n", valueMax_.epoch);           

    if(send) {
       WgVMWrDate msg;        
       memcpy( &msg.epoch, (void *)vmEEPPageBuffer, sizeof(msg.epoch) );
       sendWgVmDate( &msg ); 
    }
    
    if( result != kStatus_Success ) {
        handleVMEEPError(); 
        PRINTF("readVmDate(): failed to read value max date!\r\n" );    
    }
    
    return result;
}

/******************************************************************************/
/*!   \fn void readFitCalibration( unsigned char pageIndex )

      \brief
        This function reads ValueMax FIT data from serial eeprom
   
      \author
          Carlos Guzman
*******************************************************************************/
static status_t readFitCalibration( unsigned char pageIndex )
{  
    unsigned char startingAddress;
    status_t result                = kStatus_Success;
   
    switch(pageIndex)
    {
        case 0:
        {
            startingAddress = VM_EEP_FIT_PAGE_0_CAL;
            break;
        }
        case 1:
        {
            startingAddress = VM_EEP_FIT_PAGE_1;
            break;
        }
        case 2:
        {
            startingAddress = VM_EEP_FIT_PAGE_2;
            break;
        }
        case 3:
        {
            startingAddress = VM_EEP_FIT_PAGE_3_X_GAIN;
            break;
        }
        case 4:
        {
            startingAddress =VM_EEP_FIT_PAGE_4_Y_GAIN;
            break;
        }
        case 5:
        {
            startingAddress = VM_EEP_FIT_PAGE_5_Z_GAIN;
            break;
        }
        case 6:
        {
            startingAddress = VM_EEP_FIT_PAGE_6;
            break;
        }
        default:
        {
            PRINTF( "readFitCalibration(): Invalid page index %d \r\n", pageIndex ); 
            return kStatus_Fail;
        }
        
    } 
    
    result = read93C56EEPMultiByte(startingAddress, vmEEPPageBuffer, PAGE_SIZE_93C56 );
   
    return result;
}



/*********************************************************************************************************
***************************** Write VM Config to EEP Functions *******************************************
**********************************************************************************************************/

/******************************************************************************/
/*!   \fn void writeVmAssemblyNumber( unsigned char *pData, unsigned char length )

      \brief
        This function write the value max serial eeprom page of the assembly number.
   
      \author
          Aaron Swift
*******************************************************************************/
static void writeVmAssemblyNumber( unsigned char *pData, unsigned char length )
{ 
    if( length <= VM_ASSEMBLY_MAX ) { 
        if( write93C56MultiWord(VM_EEP_BD_ASSY_NUM_ADDR, pData, length ) ) {
           handleVMEEPError(); 
           PRINTF("writeVmAssemblyNumber(): failed to write value max assembly number!\r\n" );         
        }
    } else {
        PRINTF( "writeVmAssemblyNumber(): length %d is greater than VM_ASSEMBLY_MAX\r\n", length ); 
    }
}

/******************************************************************************/
/*!   \fn void writeVmVersion( unsigned char *pData, unsigned char length )

      \brief
        This function write the value max serial eeprom page of the version number.
   
      \author
          Aaron Swift
*******************************************************************************/
static void writeVmVersion( unsigned char *pData, unsigned char length )
{
    if( length <= VM_VER_MAX ) {  
        if( write93C56MultiWord(VM_EEP_BD_HW_VER_ADDR, pData, length ) ) {
            handleVMEEPError(); 
            PRINTF("writeVmVersion(): failed to write value max version!\r\n" );         
        }
    } else {
        PRINTF( "writeVmVersion(): length %d is greater than VM_VER_MAX\r\n", length ); 
    }  
}

/******************************************************************************/
/*!   \fn void writeVmSerialNumber( unsigned char *pData, unsigned char length )

      \brief
        This function write the value max serial eeprom page of the serial number.
   
      \author
          Aaron Swift
*******************************************************************************/
static void writeVmSerialNumber( unsigned char *pData, unsigned char length )
{
    if( length <= VM_SERIAL_MAX ) { 
        if( write93C56MultiWord(VM_EEP_BD_SER_NUM_ADDR, pData, length ) ) {
            handleVMEEPError(); 
            PRINTF("writeVmSerialNumber(): failed to write value max serial number!\r\n" );         
        }
    } else {
        PRINTF( "writeVmSerialNumber(): length %d is greater than VM_SERIAL_MAX\r\n", length ); 
    }        
}

/******************************************************************************/
/*!   \fn void writeVmDate( unsigned char *pData, unsigned char length )

      \brief
        This function write the value max serial eeprom page of the date.
   
      \author
          Aaron Swift
*******************************************************************************/
static void writeVmDate( unsigned char *pData, unsigned char length )
{
    if( length <= sizeof(unsigned long) ) { 
        if( write93C56MultiWord(VM_EEP_BD_MFR_DATE_ADDR, pData, length ) ) {
            handleVMEEPError(); 
            PRINTF("writeVmDate(): failed to write value max date!\r\n" );         
        }
    } else {
        PRINTF( "writeVmDate(): length %d is greater than sizeof(unsigned long)\r\n", length ); 
    }
    
}


/******************************************************************************/
/*!   \fn void writeFitCalibrationAll( unsigned char *pData)

      \brief
        This function writes ValueMax FIT data into serial eeprom

      \details
        TFink 5/16/24: This function takes about 500mS to execute. Note that for every word
        written to the eep, vTaskDelay(1) is called twice, meaning the majority of this
        500mS, other tasks are running
   
      \author
          Tom Fink
*******************************************************************************/
static void writeFitCalibrationAll(void)
{  
    if( write93C56MultiWord(VM_EEP_FIT_PAGE_0_CAL, (unsigned char *)&valueMax_.fit_data, sizeof(fit_flash_section))) {
        handleVMEEPError(); 
        PRINTF("writeVmZgain(): failed to write fit cal values!\r\n" );         
    }        
}


/***********************************************************************************************************
************************************     Getter/Setter  Functions ******************************************
************************************************************************************************************/
static SemaphoreHandle_t valueMaxSPI3Mutex_;

static bool initVMSPI3Mutex( void )
{
    bool result = false;
    
    valueMaxSPI3Mutex_ = xSemaphoreCreateMutex(); 
    if( !valueMaxSPI3Mutex_ ) {         
        PRINTF("initVMSPI3Mutex(): Failed to create mutex!\r\n" );
        assert( 0 );
    } else {
        result = true;
    }
    return result;  
}

bool getLockVMSPI3Flash( void )
{
    bool result = false;
    if( xSemaphoreTake( valueMaxSPI3Mutex_, ( pdMS_TO_TICKS(10) )) == pdTRUE ) { 
        result = true;  
    }
    return result;    
}

bool releaseLockVMSPI3Flash( void )
{
    xSemaphoreGive( valueMaxSPI3Mutex_ ); 
    return(true);
}


BaseType_t queueVMTransactionRequest(valueMaxTransactionID id)
{
   BaseType_t result = xQueueSendToBack( valueMaxQHandle, (void *)&id, 0 );
   if( result != pdTRUE ) {
      PRINTF("queueVMTransactionRequest(): ValueMax Queue full! \r\n");  
   } 
   return(result);
}

bool isAccelerometerInitialized(void)
{
   return(valueMax_.acceleromter_initialized);
}

bool accelMinSamplesReached(void)
{
   return(valueMax_.accel_minimum_samples_reached);
}
          
double getXRadians(void)
{
  return(valueMax_.accel_x_radians);
}
          
double getYRadians(void)
{
  return(valueMax_.accel_y_radians);
}

vmSPIInitStates getVmSPIConfigState(void)
{
   return vmSPIConfigState;
}

void setVmSPIConfigState(vmSPIInitStates newState)
{
   vmSPIConfigState = newState;
}

ValueMax *getPtrToValueMax(void)
{
   return &valueMax_;  
}

static void clearAccelEEPPageBuffer(void)
{
   unsigned char i = 0;
   while(i< PAGE_SIZE_93C56)
   {
     vmEEPPageBuffer[i] = BLANK_CHAR_93C56;
     i++;
   }
}

lpspi_master_handle_t* getVmSPIHandle(void)
{
   return &vmSpiHandle_;
} 


/***********************************************************************************************************
***************************     Development / Test Functions      ******************************************
************************************************************************************************************/

static void PrintVMFitValues(void)
{   
    PRINTF("valueMax_.fit_data.g\t\t%d\r\n",valueMax_.fit_data.g);                                                          
	PRINTF("valueMax_.fit_data.m\t\t%d\r\n",valueMax_.fit_data.m);                                                         
	PRINTF("valueMax_.fit_data.zerovalue\t%d\r\n",valueMax_.fit_data.zerovalue);                                                         
	PRINTF("valueMax_.fit_data.spanvalue\t%d\r\n",valueMax_.fit_data.spanvalue);                                                         
	PRINTF("valueMax_.fit_data.Z0\t\t%d\r\n",valueMax_.fit_data.Z0);                                                         
	PRINTF("valueMax_.fit_data.D\t\t%d\r\n",valueMax_.fit_data.D);
	PRINTF("valueMax_.fit_data.x2\t\t%d\r\n",valueMax_.fit_data.x2);
	PRINTF("valueMax_.fit_data.x1\t\t%d\r\n",valueMax_.fit_data.x1);
	PRINTF("valueMax_.fit_data.x0\t\t%d\r\n",valueMax_.fit_data.x0);
	PRINTF("valueMax_.fit_data.y2\t\t%d\r\n",valueMax_.fit_data.y2);
	PRINTF("valueMax_.fit_data.y1\t\t%d\r\n",valueMax_.fit_data.y1);
	PRINTF("valueMax_.fit_data.y0\t\t%d\r\n",valueMax_.fit_data.y0);
	PRINTF("valueMax_.fit_data.xrot[0]\t%d\r\n",valueMax_.fit_data.xrot[0]);
	PRINTF("valueMax_.fit_data.xrot[1]\t%d\r\n",valueMax_.fit_data.xrot[1]);
	PRINTF("valueMax_.fit_data.xrot[2]\t%d\r\n",valueMax_.fit_data.xrot[2]);
	PRINTF("valueMax_.fit_data.yrot[0]\t%d\r\n",valueMax_.fit_data.yrot[0]);
	PRINTF("valueMax_.fit_data.yrot[1]\t%d\r\n",valueMax_.fit_data.yrot[1]);
	PRINTF("valueMax_.fit_data.yrot[2]\t%d\r\n",valueMax_.fit_data.yrot[2]);
	PRINTF("valueMax_.fit_data.zrot[0]\t%d\r\n",valueMax_.fit_data.zrot[0]);
	PRINTF("valueMax_.fit_data.zrot[1]\t%d\r\n",valueMax_.fit_data.zrot[1]);
	PRINTF("valueMax_.fit_data.zrot[2]\t%d\r\n",valueMax_.fit_data.zrot[2]);
	PRINTF("valueMax_.fit_data.xgain\t%d\r\n",valueMax_.fit_data.xgain);
	PRINTF("valueMax_.fit_data.ygain\t%d\r\n",valueMax_.fit_data.ygain);
	PRINTF("valueMax_.fit_data.g_coeff\t%d\r\n",valueMax_.fit_data.g_coeff);
	PRINTF("valueMax_.fit_data.checksum\t%d\r\n\r\n",valueMax_.fit_data.checksum);   
}


#ifdef TFinkWriteEEPDefaults  //Development Only
static void write_VM_EEP_Defaults(void)     
{  
   
  /* Write Defaults */
  PRINTF("Queuing Write VM EEP Defaults....\r\n");
  #if 1
  writeVmAssemblyNumberDefault();  
  writeVmVersionDefault();
  writeVmSerialNumberDefault();
  writeVmDateDefault();

  
  #else
  
  //memcpy(valueMax_.assemblyNumber, "00-TFink!", sizeof(valueMax_.assemblyNumber));
  memcpy(valueMax_.assemblyNumber, "00-449407", sizeof(valueMax_.assemblyNumber));
  queueVMTransactionRequest(VM_EEP_WRITE_ASSEMBLY);
  

  //memcpy(valueMax_.version, "TF", sizeof(valueMax_.version)); 
  memcpy(valueMax_.version, "A2", sizeof(valueMax_.version));  
  queueVMTransactionRequest(VM_EEP_WRITE_VERSION); 
  
  //memcpy(valueMax_.serial, "TFinkTFinkTFink", sizeof(valueMax_.serial));
  memcpy(valueMax_.serial, "0123456789ABCDE", sizeof(valueMax_.serial)); 
  queueVMTransactionRequest(VM_EEP_WRITE_SERIAL);
  
  //valueMax_.epoch =  1112223334;  //test data
  valueMax_.epoch  = 1505844683;    /* 9/17/17 18:11:22 GMT */
  queueVMTransactionRequest(VM_EEP_WRITE_DATE);
  
  #endif
  
  PRINTF("Queuing Read VM EEP Defaults....\r\n"); 
  #if 1
  readVmAssemblyNumber(SEND_MSG_TO_APP);
  readVmVersion(SEND_MSG_TO_APP); 
  readVmSerialNumber(SEND_MSG_TO_APP);
  readVmDate(SEND_MSG_TO_APP);

  #else
  queueVMTransactionRequest(VM_EEP_READ_ASSEMBLY_SEND);
  queueVMTransactionRequest(VM_EEP_READ_VERSION_SEND);
  queueVMTransactionRequest(VM_EEP_READ_SERIAL_SEND);
  queueVMTransactionRequest(VM_EEP_READ_DATE_SEND);
  #endif
  
  //writeFitCalibration() has been deprecated in favor of writeFitCalibrationAll()
    //writeFitCalibration((unsigned char*)&valueMax_.fit_data+2*PAGE_SIZE_93C56, 0);
  
  /* Test Code only. Will only write if testWriteFitValues is modified in debugger. 
     Since each loadcell has unique FIT values, best to modify values for test and
     then change them back */
  static bool testWriteFitValues = false;
  if(testWriteFitValues) {
     initFitDefaults();
     writeFitCalibrationAll();
  }
  
}


/******************************************************************************/
/*!   \fn void writeVmAssemblyNumberDefault( void )

      \brief
        This function write factory default assembly number value to the value 
        max serial eeprom.
        
      \author
          Aaron Swift
*******************************************************************************/
static void writeVmAssemblyNumberDefault( void )
{
    //unsigned char str[VM_ASSEMBLY_MAX] = {"00-TFink!"};
    unsigned char str[VM_ASSEMBLY_MAX] = {"00-449407"};
    writeVmAssemblyNumber( &str[0], VM_ASSEMBLY_MAX );    
}

/******************************************************************************/
/*!   \fn void writeVmVersionDefault( void )

      \brief
        This function write factory default hardware version number value to the 
        value max serial eeprom.
        
      \author
          Aaron Swift
*******************************************************************************/
void writeVmVersionDefault( void )
{
    //unsigned char str[VM_VER_MAX] = {"TF"};
    unsigned char str[VM_VER_MAX] = {"A2"}; 
    writeVmVersion( &str[0], VM_VER_MAX );
}


/******************************************************************************/
/*!   \fn void writeVmSerialNumberDefault( void )

      \brief
        This function write factory default serial number value to the value 
        max serial eeprom.
        
      \author
          Aaron Swift
*******************************************************************************/
static void writeVmSerialNumberDefault( void )
{
    //unsigned char str[VM_SERIAL_MAX] =   {"TFinkTFinkTFink"};
    unsigned char str[VM_SERIAL_MAX] = {"0123456789ABCDE"}; 
    writeVmSerialNumber( &str[0], VM_SERIAL_MAX );
  
}

/******************************************************************************/
/*!   \fn void writeVmDateDefault( void )

      \brief
        This function write factory default date value to the value max serial 
        eeprom.
        
      \author
          Aaron Swift
*******************************************************************************/
static void writeVmDateDefault( void )
{
    //unsigned long date =   1112223334;  //test data
    unsigned long date = 1505844683;    /* 9/17/17 18:11:22 GMT */
    writeVmDate( (unsigned char *)&date, sizeof(unsigned long) );
}

/******************************************************************************/
/*!   \fn static static void initFitDefaults(fit_flash_section *fitParamFlash)

      \brief
        This function will initialize Application Global struct fitParam with 
        default values.Stictly for development test. These values will never be
        correct for a random Value Max board/loadcell combination.
   
      \author
          Tom Fink
*******************************************************************************/
static void initFitDefaults( void )
{
    ValueMax *pVM = getPtrToValueMax();
    valueMax_.fit_data.g = 9801040;
    valueMax_.fit_data.g = 9801040;
    valueMax_.fit_data.m = 682499;

    valueMax_.fit_data.zerovalue = 645734;
    valueMax_.fit_data.spanvalue = 4395609;
    valueMax_.fit_data.Z0 = 42891;
    valueMax_.fit_data.D = 602843;

    valueMax_.fit_data.x2 = -172848;
    valueMax_.fit_data.x1 = -13277880;
    valueMax_.fit_data.x0 = -363926;

    valueMax_.fit_data.y2 = 0;
    valueMax_.fit_data.y1 = -25137042;
    valueMax_.fit_data.y0 = -408211;

    valueMax_.fit_data.xrot[0] = 9995297;
    valueMax_.fit_data.xrot[1] = -306492;
    valueMax_.fit_data.xrot[2] = 9459;

    valueMax_.fit_data.yrot[0] = -5426;
    valueMax_.fit_data.yrot[1] = 131628;
    valueMax_.fit_data.yrot[2] = 9999132;

    valueMax_.fit_data.zrot[0] = -306590;
    valueMax_.fit_data.zrot[1] = -9994435;
    valueMax_.fit_data.zrot[2] = 131400;

    valueMax_.fit_data.xgain = 102244;
    valueMax_.fit_data.ygain = 98661;

    valueMax_.fit_data.g_coeff = 992266;

    valueMax_.fit_data.checksum = 9552;
}


#endif  //TFinkWriteEEPDefaults




#ifdef TFinkAlternateAngleCalc 
static DebugAngleCalcStruct  sca3300HWTest_;

/******************************************************************************/
/*!   \fn void alternateAngleCalculation(void)

      \brief
        Development Function 
   
      \author
          Tom Fink
*******************************************************************************/
void alternateAngleCalculation(void) 
{
   static bool zeroCaptured = false;
   static unsigned short printfThrottle = 0;
   
   sca3300HWTest_.xAccel = sca3300_.xAccel;
   sca3300HWTest_.yAccel = sca3300_.yAccel;
   sca3300HWTest_.zAccel = sca3300_.zAccel;
   
   runAveragingFilter();
   
   /*************************************************************************
    * Note - this calculation is only accurrate if the loadcell is placed on
    * a level table and then the weigher calibration button is pressed to 
    * capture a zeroed value. This zeros out the angle of the sensor. At that
    * point, it should produce relatively accurate values 
    *************************************************************************/
   
   if(!zeroCaptured)
   {
      if(!GPIO_ReadPinInput( SERVICE_SWITCH_GPIO, SERVICE_SWITCH_PIN ))
      {
         sca3300HWTest_.xZero = sca3300HWTest_.xAccelFiltered; 
         sca3300HWTest_.yZero = sca3300HWTest_.yAccelFiltered; 
         //sca3300HWTest_.zZero = sca3300HWTest_.zAccel;
         sca3300HWTest_.zZero = 0;
         PRINTF("Zero Captured x,y,z: %d %d %d\r\n",sca3300HWTest_.xZero,sca3300HWTest_.yZero,sca3300HWTest_.zZero ); 
         zeroCaptured = true;
      }
   }
   
   printfThrottle++;
   if(printfThrottle == 10)
   {
      calcAngleAndPrint();
      printfThrottle = 0;
   }
}
/******************************************************************************/
/*!   \fn void calcAngleAndPrint(void)

      \brief
        Development Function to verify calculated tilt vs actual tilt
   
      \author
          Tom Fink
*******************************************************************************/
void calcAngleAndPrint(void)
{
   double sca3300X, sca3300Y, sca3300Z;
   double xPrimeInput, yPrimeInput,x_prime_rad, y_prime_rad, x_degrees, y_degrees; 
   const double scaleFactorSCA3300 = 47.407407; 

   /* All operations are ratios of these variables */
   sca3300HWTest_.zeroedXAccel = (sca3300HWTest_.xAccelFiltered - sca3300HWTest_.xZero);
   sca3300X = (double)sca3300HWTest_.zeroedXAccel; 
   sca3300X = sca3300X*scaleFactorSCA3300;
   sca3300HWTest_.zeroedYAccel = sca3300HWTest_.yAccelFiltered - sca3300HWTest_.yZero;
   sca3300Y = (double)sca3300HWTest_.zeroedYAccel; 
   sca3300Y = sca3300Y*scaleFactorSCA3300;
   sca3300HWTest_.zeroedZAccel = sca3300HWTest_.zAccelFiltered - sca3300HWTest_.zZero;
   sca3300Z = (double)sca3300HWTest_.zeroedZAccel; 
   sca3300Z = sca3300Z*scaleFactorSCA3300;
   
   xPrimeInput = (sca3300X / sqrt(sca3300Y*sca3300Y + sca3300Z*sca3300Z));
   yPrimeInput = (sca3300Y / sqrt(sca3300X*sca3300X + sca3300Z*sca3300Z));
   
   x_prime_rad = atan(xPrimeInput); 
   y_prime_rad = atan(yPrimeInput);
   
   x_degrees   = x_prime_rad * 180 / PI;    
   y_degrees   = y_prime_rad * 180 / PI; 
   
   
   //PRINTF("Raw: %6d %6d %6d Zeroed: %6d %6d %6d      X: %.3f Y %.3f degrees\r\n", sca3300HWTest_.xAccel, sca3300HWTest_.yAccel, sca3300HWTest_.zAccel, sca3300HWTest_.zeroedXAccel, sca3300HWTest_.zeroedYAccel, sca3300HWTest_.zeroedZAccel, x_degrees, y_degrees ); 
   PRINTF("alt Avg: %6d %6d %6d Zrd: %6d %6d %6d      X: %.3f Y %.3f degrees\r\n", sca3300HWTest_.xAccelFiltered, sca3300HWTest_.yAccelFiltered, sca3300HWTest_.zAccelFiltered, sca3300HWTest_.zeroedXAccel, sca3300HWTest_.zeroedYAccel, sca3300HWTest_.zeroedZAccel, x_degrees, y_degrees ); 

}

/******************************************************************************/
/*!   \fn runAveragingFilter(void)

      \brief
        Development Averaging Filter. refactor if decide to keep for production.
   
      \author
          Tom Fink
*******************************************************************************/
#define AVG_FILTER_NUM_ELEMENTS 16
static short xFIFO[AVG_FILTER_NUM_ELEMENTS];
static unsigned short xAxisPointer = 0;
static unsigned long xAxisAccumulator = 0; 
static bool xAxisFIFOFull = false;

static short yFIFO[AVG_FILTER_NUM_ELEMENTS];
static unsigned short yAxisPointer = 0;
static unsigned long yAxisAccumulator = 0; 
static bool yAxisFIFOFull = false;

static short zFIFO[AVG_FILTER_NUM_ELEMENTS];
static unsigned short zAxisPointer = 0;
static unsigned long zAxisAccumulator = 0; 
static bool zAxisFIFOFull = false;


void runAveragingFilter(void)
{
   /******* XAxis ************/
   xAxisPointer++;
   if(xAxisPointer >= AVG_FILTER_NUM_ELEMENTS)  {
      xAxisPointer = 0;
      xAxisFIFOFull = true;
   }
   
   xAxisAccumulator += sca3300HWTest_.xAccel;      //add newest value to accumulator
   if(xAxisFIFOFull) {   
     xAxisAccumulator -= xFIFO[xAxisPointer];  //remove the oldest value from the sum
     sca3300HWTest_.xAccelFiltered = xAxisAccumulator/AVG_FILTER_NUM_ELEMENTS;
   } else {
     sca3300HWTest_.xAccelFiltered =  sca3300HWTest_.xAccel;   //filter not operational until the FIFO filled
   }
     
   xFIFO[xAxisPointer] = sca3300HWTest_.xAccel;
   
    /******* YAxis ************/
   yAxisPointer++;
   if(yAxisPointer >= AVG_FILTER_NUM_ELEMENTS)  {
      yAxisPointer = 0;
      yAxisFIFOFull = true;
   }
   
   yAxisAccumulator += sca3300HWTest_.yAccel;      //add newest value to accumulator
   if(yAxisFIFOFull) {   
     yAxisAccumulator -= yFIFO[yAxisPointer];  //remove the oldest value from the sum
     sca3300HWTest_.yAccelFiltered = yAxisAccumulator/AVG_FILTER_NUM_ELEMENTS;
   } else {
     sca3300HWTest_.yAccelFiltered =  sca3300HWTest_.yAccel;   //filter not operational until the FIFO filled
   }
     
   yFIFO[yAxisPointer] = sca3300HWTest_.yAccel;   
   
   /******* ZAxis ************/
   zAxisPointer++;
   if(zAxisPointer >= AVG_FILTER_NUM_ELEMENTS)  {
      zAxisPointer = 0;
      zAxisFIFOFull = true;
   }
   
   zAxisAccumulator += sca3300HWTest_.zAccel;      //add newest value to accumulator
   if(zAxisFIFOFull) {   
     zAxisAccumulator -= zFIFO[zAxisPointer];  //remove the oldest value from the sum
     sca3300HWTest_.zAccelFiltered = zAxisAccumulator/AVG_FILTER_NUM_ELEMENTS;
   } else {
     sca3300HWTest_.zAccelFiltered =  sca3300HWTest_.zAccel;   //filter not operational until the FIFO filled
   }
     
   zFIFO[zAxisPointer] = sca3300HWTest_.zAccel;   
}
#endif //ifdef TFinkAlternateAngleCalc 


/***********************************************************************************************************
*********** Deprecated Functions that may be used in the future   ******************************************
************************************************************************************************************/


#if 0 //TFink 5/16/24 deprecated in favor of writeFitCalibrationAll
/******************************************************************************/
/*!   \fn void writeFitCalibration( unsigned char *pData, unsigned char length )

      \brief
        This function writes ValueMax FIT data into serial eeprom
   
      \author
          Carlos Guzman
*******************************************************************************/
void writeFitCalibration( unsigned char *pData, unsigned char pageIndex )
{
    unsigned char addr;
    
    switch(pageIndex)
    {
        case 0:
        {
            addr = VM_EEP_FIT_PAGE_0_CAL;
            break;
        }
        case 1:
        {
            addr = VM_EEP_FIT_PAGE_1;
            break;
        }
        case 2:
        {
            addr = VM_EEP_FIT_PAGE_2;
            break;
        }
        case 3:
        {
            addr = VM_EEP_FIT_PAGE_3_X_GAIN;
            break;
        }
        case 4:
        {
            addr = VM_EEP_FIT_PAGE_4_Y_GAIN;
            break;
        }
        case 5:
        {
            addr = VM_EEP_FIT_PAGE_5_Z_GAIN;
            break;
        }
        case 6:
        {
            addr = VM_EEP_FIT_PAGE_6;
            break;
        }
        default:
        {
            PRINTF( "writeFitCalibration(): Invalid page index %d \r\n", pageIndex ); 
            return;
        }
        
    }
    
    if( write93C56MultiWord(addr, pData, PAGE_SIZE_93C56)) {
        handleVMEEPError();
        PRINTF("writeVmZgain(): failed to write fit cal values!\r\n" );         
    }         
}
#endif


         
