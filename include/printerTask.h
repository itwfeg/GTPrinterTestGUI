#ifndef PRINTERTASK_H
#define PRINTERTASK_H
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
//#include "printer.h"
#include "semphr.h"

#define printer_task_PRIORITY ( configMAX_PRIORITIES -3 )

typedef enum 
{
    K_PRINTER_UNKNOWN,
    K_PRINTER_SERVICE_SCALE,
    K_PRINTER_PREPACK,    
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


/* public functions */
BaseType_t createPrinterTask( PrinterStyle style );
void assignPrinterMsgQueue( QueueHandle_t pQHandle );
TaskHandle_t getPrinterHandle( void );
PrStatusInfo *getCurrentStatus( void );
void prWakeupCallBack( TimerHandle_t timer_  ); 
bool isServiceScale( void );
SemaphoreHandle_t getCutSemaphore( void );
void setConfigBackingValue(unsigned char val );
void setConfigLabelValue(unsigned char val );
/* private functions */
static void printerTask( void *pvParameters );
static void handlePrinterMsg( PrMessage *pMsg );
static void resetPrinter( void );
static void getPrVersion( PrVersion *pVersion );
int getLabelSize();
static void testHistory( void );
static void monitorMaintenance( void );
#endif