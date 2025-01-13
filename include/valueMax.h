#ifndef SCA3300_H
#define SCA3300_H
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "fsl_lpspi.h"
#include "threadManager.h"

/************************* Defines **************************************/
#define vmTaskStackSize   1024
#define vm_task_PRIORITY ( configMAX_PRIORITIES -1 )

#define MAX_ACCEL_RUNNING_SAMPLES	100 /* during normal operations */


  
/********** VM EEP defines **************************/    
#define VM_ASSEMBLY_MAX             10
#define VM_VER_MAX                  2
#define VM_SERIAL_MAX               16 


/******* VM EEP Memory Starting Addresses *****************/
#define VM_EEP_TOTAL_PAGES           7

/* Note - all data started on page  boundry */
#define VM_EEP_BD_ASSY_NUM_ADDR      0       /* Page 0: board assembly number */
#define VM_EEP_BD_HW_VER_ADDR        16      /* Page 1: board HW revision */
#define VM_EEP_BD_SER_NUM_ADDR       32      /* Page 2: board serial number */ 
#define VM_EEP_BD_MFR_DATE_ADDR      48      /* Page 3: board date of manufacture */
#define VM_EEP_FIT_PAGE_0_CAL        64      /* Page 4: Fit Page 0 */
#define VM_EEP_FIT_PAGE_1            80      /* Page 5: reverved */
#define VM_EEP_FIT_PAGE_2            96      /* Page 6: reverved */
#define VM_EEP_FIT_PAGE_3_X_GAIN     112     /* Page 7: adxl312 X gain factor */
#define VM_EEP_FIT_PAGE_4_Y_GAIN     128     /* Page 8: adxl312 Y gain factor */
#define VM_EEP_FIT_PAGE_5_Z_GAIN     144     /* Page 9: adxl312 Z gain factor */
#define VM_EEP_FIT_PAGE_6            160     /* Page 10: reverved */
#define VM_EEP_PAGE11_START_ADDR     176     /* Page 11: reverved */
#define VM_EEP_PAGE12_START_ADDR     192     /* Page 12: reverved */
#define VM_EEP_PAGE13_START_ADDR     208     /* Page 13: reverved */
#define VM_EEP_PAGE14_START_ADDR     224     /* Page 14: reverved */
#define VM_EEP_PAGE15_START_ADDR     240     /* Page 15: reverved */


/************************* ENUMs **************************************/

typedef enum
{
  VM_SPI_UNINITIALIZED,
  VM_SPI_INITIALIZED_FOR_93C56,
  VM_SPI_INITIALIZED_FOR_SCA3300  
}vmSPIInitStates;

/* Value Max Transactions - Serial EEP and Accelerometer */
typedef enum
{  
    VM_EEP_NO_TRANSACTION,
    VM_EEP_READ_ASSEMBLY,
    VM_EEP_READ_ASSEMBLY_SEND,
    VM_EEP_WRITE_ASSEMBLY,
    VM_EEP_READ_VERSION,
    VM_EEP_READ_VERSION_SEND,
    VM_EEP_WRITE_VERSION,
    VM_EEP_READ_SERIAL,
    VM_EEP_READ_SERIAL_SEND,
    VM_EEP_WRITE_SERIAL,
    VM_EEP_READ_DATE,
    VM_EEP_READ_DATE_SEND,
    VM_EEP_WRITE_DATE,
    VM_EEP_WRITE_FIT_CALIBRATION,
    VM_EEP_READ_FIT_CALIBRATION    
}valueMaxTransactionID;

typedef enum {
    VMREQ_ASSEMBLY,		
    VMREQ_HWVERSION,	
    VMREQ_SERIALNUM,	
    VMREQ_DATE,		
    VMREQ_FIT_VALUES,	
    VMREQ_MAX,		
}ValueMaxReq;

typedef struct   
{
    long g;     
    long m;    
    long zerovalue;
    long spanvalue;

    long Z0;
    long D;
    long x2;
    long x1;
    
    long x0;
    long y2;
    long y1;
    long y0;

    long xrot[3];
    long yrot[3];
    long zrot[3];

    long xgain;
    long ygain;

    long g_coeff;

    long checksum;
}fit_flash_section;

typedef struct 
{
    /* Accel EEPROM */
    bool                        vmEEP_initialized;
    unsigned char               assemblyNumber[VM_ASSEMBLY_MAX];    
    unsigned char               version[VM_VER_MAX];
    unsigned char               serial[VM_SERIAL_MAX];  
    fit_flash_section           fit_data;
    unsigned long               epoch;   /* date */
           
    bool                	    acceleromter_initialized;
    unsigned short		        acceleromter_sample_counter;	 /* used as part of avg filter */	
    long			            accel_x_raw_reading;   /* data loaded in here before being copied to the buffer */
    long			            accel_x_raw_reading_buffer[MAX_ACCEL_RUNNING_SAMPLES+1]; //The running buffer of samples that's averaged
    long                        accel_x_accum_raw_reading; //The sum of the readings in the buffer
    long                        accel_x_avg_raw_reading;  //accum_raw_reading divided by MAX_ACCEL_RUNNING_SAMPLES+1
    long			            accel_y_raw_reading;
    long			            accel_y_raw_reading_buffer[MAX_ACCEL_RUNNING_SAMPLES+1];
    long                        accel_y_accum_raw_reading;
    long                        accel_y_avg_raw_reading;
    long			            accel_z_raw_reading;
    long			            accel_z_raw_reading_buffer[MAX_ACCEL_RUNNING_SAMPLES+1];
    long                        accel_z_accum_raw_reading;
    long                        accel_z_avg_raw_reading;
    float                       x_degrees;
    float                       y_degrees;
    float                       accel_x_radians;
    float                       accel_y_radians;
    bool                        accel_minimum_samples_reached;  //true when raw_reading_buffers are full
}ValueMax;

/* Debug Only. Used when TFinkAlternateAngleCalc is #defined*/
typedef struct      
{
    short               xAccel;
    short               yAccel;
    short               zAccel;
    short               xAccelFiltered;
    short               yAccelFiltered;
    short               zAccelFiltered;
    short               xZero;
    short               yZero;
    short               zZero; 
    short               zeroedXAccel;
    short               zeroedYAccel;
    short               zeroedZAccel;                  
}DebugAngleCalcStruct;


/* public prototypes */
bool getLockVMSPI3Flash( void );
bool releaseLockVMSPI3Flash( void );
BaseType_t createValueMaxTask( void );
BaseType_t queueVMTransactionRequest(valueMaxTransactionID id);
QueueHandle_t getVMQueueHandle( void );
ValueMax *getPtrToValueMax(void);
void setVmSPIConfigState(vmSPIInitStates newState);
lpspi_master_handle_t* getVmSPIHandle(void);
bool isAccelerometerInitialized(void);
bool accelMinSamplesReached(void);          
double getXRadians(void);      
double getYRadians(void);
vmSPIInitStates getVmSPIConfigState(void);






/* private prototypes */
static void initFitDefaults( void );
static bool initVMSPI3Mutex( void );
static void clearAccelEEPPageBuffer(void);
static void writeVmAssemblyNumber( unsigned char *pData, unsigned char length );
static void writeFitCalibrationAll(void);
static void writeVmDate( unsigned char *pData, unsigned char length );
static void writeVmVersion( unsigned char *pData, unsigned char length );
static void writeVmSerialNumber( unsigned char *pData, unsigned char length );

static status_t readVmAssemblyNumber( bool send );
static status_t readVmVersion( bool send );       
static status_t readVmSerialNumber( bool send );
static status_t readVmDate( bool send );
static status_t readFitCalibration( unsigned char pageIndex );


static void handleVMQueueTransactionRequest(valueMaxTransactionID id);
static bool initValueMaxEEP(void);
static void PrintVMFitValues(void);
static void writeVmAssemblyNumberDefault( void );
static void alternateAngleCalculation(void);
static void writeVmVersionDefault( void );
static void writeVmSerialNumberDefault( void );
static void writeVmDateDefault( void );
static void write_VM_EEP_Defaults(void);
static void handleVMAccelError(void);
static void handleVMEEPError(void);
bool isVmPCBAssyNumValid(unsigned char *assyNum);
static void calcAngleAndPrint(void);
static void runAveragingFilter(void);
static void valueMaxTask( void *pvParameters );
#endif
