#ifndef GLOBALPRINTERTASK_H
#define GLOBALPRINTERTASK_H
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#include "prMessages.h"

#define printer_task_PRIORITY ( configMAX_PRIORITIES - 1 )

typedef enum 
{
    RT_PRINTER_UNKNOWN,
    RT_PRINTER_SERVICE_SCALE_72MM,
    RT_PRINTER_SERVICE_SCALE_80MM,
    RT_PRINTER_FRESH_SERVE_SCALE_72MM,
    RT_PRINTER_FRESH_SERVE_SCALE_80MM,
    RT_PRINTER_STAND_ALONE_72MM,
    RT_PRINTER_STAND_ALONE_80MM,    
    RT_PRINTER_PREPACK,    
}PrinterStyle;

typedef enum
{
    _UNKNOWN_ACTIVE_TEST,
    _LABEL_GAP_TEST,
    _LABEL_EDGE_TEST,
    _CUTTER_LIFE_TEST,
    _DRIVETRAIN_LIFE_TEST,
    _CLEANING_HEAD_TEST,
    _STOP_TEST
}ActiveTest;




/* prototypes */
BaseType_t createGlobalPrinterTask( PrinterStyle style, QueueHandle_t msgQueue );
TaskHandle_t getPrinterHandle( void );
//PrStatusInfo *getCurrentStatus( void );
bool startLabelTakenTimer( void );
void stopLabelTakenTimer( void );
void postRequestStatus( void );
void resetPrinter( void );

int getLabelSizeFromPrinterTask( void );
bool getWaitForLabelTaken( void );

/* timer callback functions */
void prWakeupCallBack( TimerHandle_t timer_  ); 
void takeLabelCallBack( TimerHandle_t timer_ );
void continuousLabelCallBack( TimerHandle_t timer_ );
void sensorTimerCallBack( TimerHandle_t timer_ );

bool isServiceScale( void );
void setConfigBackingValue(unsigned short val );
void setConfigLabelValue(unsigned short val );
void setConfigTakeupSensorValue(unsigned char val, unsigned short max, unsigned short min );
void setConfigMediaCurrentValue(unsigned short val );

/* private functions */
static void printerTask( void *pvParameters );
static void handlePrinterMsg( PrMessage *pMsg );
static void getPrVersion( PrVersion *pVersion );
static void monitorMaintenance( void );
static bool verifyPrConfiguration( Pr_Config *pTemp, Pr_Config *pSF );
#endif