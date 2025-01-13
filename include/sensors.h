#ifndef SENSORS_H
#define SENSORS_H
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "fsl_adc.h"
#include "globalPrinter.h"
#include "prMessages.h"
#include <stdbool.h>
#include "fsl_common.h" 


#define sensors_task_PRIORITY ( configMAX_PRIORITIES - 1 )

#define BY_TWO          2       /* shifting by two = divide by four */
#define HEAD_DELTA      60      /* about 1/2Volt */
#define HEAD_PERIOD     4       /* average four readings */

/* Sensor Bits */
#define MOTOR_FORWARD                   0x01
#define LABEL_TAKEN                     0x02
#define SYNC_BAR                        0x04
#define SYNC_BAR_EDGE                   0x08
#define WIDE_LABEL                      0x10
#define THERMAL_OVERLOAD                0x20
#define OUT_OF_MEDIA                    0x40
#define SYNCHRONIZED                    0x80


#define HIGH_HEAD_TEMPERATURE_LIMIT      165     /* Approx 70C for Kyocera,TDK, and Toshiba */
#define HIGH_HEAD_TEMPERATURE_LIMIT_ROHM 65//50     /* 50 �C */ 
#if 1

/* 12bit adc setting for global */ 
#define AD_RESOLUTION                    .00080566      /* voltage per count */

//#define NO_CASSET_THRESHOLD              3413           /* 2.75Vdc ( 2.75 / .00080566 )*/  
#define NO_CASSET_THRESHOLD              2000           /* 2.75Vdc ( 2.75 / .00080566 )*/  
#define FREESTAND_HEADUP_THRESHOLD       2482           /* 2.0Vdc */
#define MEDIA_CAL_VOLTAGE                372            /* 300mV  ( .300 / .00080566 ) */
#define MEDIA_SYNC_BAR_THRESHOLD         1241           /* 1.00Vdc ( 1.0 / .00080566 ) */
#define MEDIA_OUT_OF_THRESHOLD           2978           /* 2.4Vdc ( 2.4 / .00080566 ) */             
/* different bias on label taken board 220K pulldown */ 
//#define LABEL_TAKEN_THRESHOLD_LABEL      3200            
//#define LABEL_TAKEN_THRESHOLD_NO_LABEL   2700  

/* different bias on label taken board 100K pulldown */
#define LABEL_TAKEN_THRESHOLD_LABEL      101            
#define LABEL_TAKEN_THRESHOLD_NO_LABEL   100 

#define LABEL_EDGE_THRESHOLD             1300     /* was 840 */
#define BACKING_PAPER_THRESHOLD          1000    /* 700 was 450 */        /* backing paper threshold is 60% of label edge threshold */
#define SHOOT_THROUGH_NOISE_FLOOR        20

#else

#define AD_RESOLUTION                    .01294         /* voltage per count */

#define LABEL_WIDTH_THRESHOLD            116     /* 1.5Vdc ( 1.5 / .01294 ) */                      /*28788    1.5Vdc ( 1.5 / 65535 ) */
#define NO_CASSET_THRESHOLD              212     /* 2.75Vdc ( 2.75 / .01294 )*/                       /*59577    3.0Vdc ( 3.0 / 65535 ) */
#define FREESTAND_HEADUP_THRESHOLD       155
#define WIDE_CASSET_THRESHOLD            96      /* 1.25Vdc ( 1.25 / .01294 ) */
#define NARROW_CASSET_THRESHOLD          39      /* 50mVdc ( .05 / .01294 ) noise floor */          /*992      50mVdc ( .05 / 65535 ) noise floor */
#define HEAD_POWER_ON_THRESHOLD          116     /* 1.5Vdc ( 1.5 / .01294 ) */   
#define MEDIA_CAL_VOLTAGE                23      /* 300mV  ( .300 / .01294 ) */
#define MEDIA_SYNC_BAR_THRESHOLD         77      /* 1.00Vdc ( 1.0 / .01294 ) */
#define MEDIA_OUT_OF_THRESHOLD           185     /* 2.4Vdc ( 2.4 / .01294 ) */                      /*35746    1.8Vdc ( 1.8 / 65535 ) */
#define LABEL_TAKEN_THRESHOLD_LABEL      100
#define LABEL_TAKEN_THRESHOLD_NO_LABEL   25
#define LABEL_EDGE_THRESHOLD             84 
#define BACKING_PAPER_THRESHOLD          63     /* backing paper threshold is 60% of label edge threshold */
#define SHOOT_THROUGH_NOISE_FLOOR        20
#endif

typedef enum
{
    AD_INIT,
    AD_MANUAL,
    AD_AUTO
}ADConfig;

/* internal a/d */
#define SHOOT_SENSOR_CHANNEL 7U                 /* ad2 channel 7  */
#define TAKEN_SENSOR_CHANNEL 0U                 /* ad2 channel 0 */
#define HEAD_TEMPERATURE_CHANNEL 9U             /* ad2 channel 9  */
#define HEAD_UP_CHANNEL 10U                     /* ad2 channel 10 */
#define HEAD_DOT_CHANNEL 11U                    /* ad2 channel 11 */
#define LOW_STOCK_CHANNEL 1U                    /* ad2 channel 1 */
#define PAPER_TAKEUP_CHANNEL 8U                 /* ad2 channel 8  */ 
#define HEAD_DETECT_CHANNEL 2U


/* isr handlers for auto mode */
#define adcEtc0HandleIsr ADC_ETC_IRQ0_IRQHandler
#define adcEtcErrorHandleIsr ADC_ETC_ERROR_IRQ_IRQHandler
#define pitHandleIsr PIT_IRQHandler 

/* isr handler for manual mode */
#define adcModule2IsrHandler ADC2_IRQHandler

/* eDMA */
#define ADC0_DMA_MAJOR_CHANNEL          5U
#define ADC0_DMA_MINOR_CHANNEL          6U
#define ADC1_DMA_MAJOR_CHANNEL          7U
#define ADC1_DMA_MINOR_CHANNEL          8U

//#define USE_EDMA      
#define TRANSFER_BYTES                  16U//14U


#define HEAD_UP_DEBOUNCE                        5
#define SAMPLING_FREQ           10000   /* 10Khz */
#define NUMBER_OF_SAMPLES       8

/* print head temperature no longer counts but actual  */       
#define ZERO_DEGREES_C                          0
#define TEN_DEGREES_C                           10
#define TWENTY_DEGREES_C                        20
#define THIRTY_DEGREES_C                        30
#define FOURTY_DEGREES_C                        40
#define FIFTY_DEGREES_C                         50
#define SIXTY_DEGREES_C                         60

#define ENGINE_BUSY_STATE                       0x01

typedef enum
{
    CHANNEL_TAKEN,
    CHANNEL_LOW_STOCK,    
    CHANNEL_SHOOT,
    CHANNEL_PAPER_TAKEUP,  
    CHANNEL_HEAD_TEMP,
    CHANNEL_HEAD_STATE,             
    CHANNEL_HEAD_DOT,
    CHANNEL_HEAD_DETECT,
    MAX_ADC_CHANNELS
}ADChannles;

typedef struct pr_ad_counts
{
    bool labelTaken;
    unsigned short thermistor;
    unsigned short thermistor2;
    unsigned short headVoltage;
    unsigned short mediaDetector;
    unsigned short boardTemp;
    unsigned short supplyCurrent;
    unsigned short headUp;
}HeadSensors;

typedef struct 
{
    unsigned short dotPosition;
    unsigned short sample1;
    unsigned short sample2;
    unsigned short sample3;
}DotTest;

/* same size as the prepack head */
#define MAX_DOT_HEAD_SIZE 680 //640? CKing

typedef struct
{
    unsigned int  adc0Channel;                          /* current channel adc0 */
    unsigned int  chIndex;                              /* channel list index */
    ADChannles  channelList[MAX_ADC_CHANNELS];          /* channel list */
    unsigned short value[MAX_ADC_CHANNELS];             /* channel conversions */
    //DotTest results[MAX_DOT_HEAD_SIZE];                 /* dot wear */
    bool adcComplete;
    bool adcPaused;
}ADCManager;

#define MAX_CHANNEL_SAMPLE      65535

BaseType_t createSensorsTask( ADConfig mode );
static void sensorsTask( void *pvParameters );
TaskHandle_t getSensorsHandle( void );
static void initManager( ADCManager *pMgr );
void closeAdc2( void );
void initializeSensors( void );
void initializeAdcs( ADConfig configuration );
void initializeEtcAdc( ADConfig configuration );
void initXbar( ADConfig configuration );
void initPitADC( void );
void suspendPDB( bool resume );
void initLabelTakenLED( void );
void pauseResumeConversions( bool pause );
unsigned short getShootThroughConversion( void );
bool isADCAutoMode( void );
void showLabelTakenCounts( void );
void showPaperTakeup( void );

typedef enum 
{
    FAN_OFF,
    HALF_SPEED,
    FULL_SPEED
}FanState;

AT_QUICKACCESS_SECTION_CODE(void readADChannels( void ));
AT_QUICKACCESS_SECTION_CODE(unsigned int readHeadVoltage( void ));
AT_QUICKACCESS_SECTION_CODE(void readMediaSensor( PrStatusInfo *pStatus ));
AT_QUICKACCESS_SECTION_CODE(bool readLabelTakenSensor( void ));
AT_QUICKACCESS_SECTION_CODE(void processLabelTakenSensor( PrStatusInfo *pStatus ));
AT_QUICKACCESS_SECTION_CODE(bool readHeadUpSensor( PrStatusInfo *pStatus ));
AT_QUICKACCESS_SECTION_CODE(void readLowStockSensor( PrStatusInfo *pStatus ));
AT_QUICKACCESS_SECTION_CODE(unsigned short getPrintheadThermistorCounts( void ));
AT_QUICKACCESS_SECTION_CODE(void readHeadTemperatureSensor( PrStatusInfo *pStatus ));
AT_QUICKACCESS_SECTION_CODE(unsigned char getHeadSupplyCurrent( void ));
AT_QUICKACCESS_SECTION_CODE(void getCurrentSensors( PrSensors *pSensors ));
AT_QUICKACCESS_SECTION_CODE(unsigned short getMediaCounts( void ));
AT_QUICKACCESS_SECTION_CODE(unsigned short pollMediaCounts( void ));
AT_QUICKACCESS_SECTION_CODE(unsigned short getLowStockSensor( void ));
AT_QUICKACCESS_SECTION_CODE(unsigned long getLabelTaken( void ));
AT_QUICKACCESS_SECTION_CODE(unsigned short getTakeUpTorque( void ));
AT_QUICKACCESS_SECTION_CODE(unsigned short getHeadVoltage( void ));
AT_QUICKACCESS_SECTION_CODE(void setMotorStopOnGap( void ));
AT_QUICKACCESS_SECTION_CODE(void clrMotorStopOnGap( void ));
AT_QUICKACCESS_SECTION_CODE(void setMotorStopOnEdge( void ));
AT_QUICKACCESS_SECTION_CODE(void clrMotorStopOnEdge( void ));

void updateMediaSensor( void );
unsigned short sampleMediaSensor( void );
unsigned short sampleHeadDotADC( void );
unsigned short sampleHeadDot( void );
unsigned short sampleHeadVoltage( void );
void incrementIndexDotTest( void );
void showADCReadings( void );
void setADCMode( ADConfig mode );
int getGhostMCntr( void );
void clearGhostMCntr( void );
bool getBackwindAfterContExpel( void );
void setBackwindAfterContExpel( bool backwind );
bool getTUCalHeadUpFlag( void );
void setTUCalHeadUpFlag( bool cal );

#endif