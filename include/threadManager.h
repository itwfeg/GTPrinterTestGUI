#ifndef THREADMANAGER_H
#define THREADMANAGER_H
#include "FreeRTOS.h"
#include "queue.h"
#include "timers.h"
#include <stdbool.h>

/* add define to turn on thread statistics
#define STATISTICS_ENABLE
#define PRINT_STATS                
*/


#define TASK_COUNT 7

#define manager_task_PRIORITY ( configMAX_PRIORITIES -1 )

typedef enum
{
    F_WEIGHERTHREAD,
    F_PRINTERTHREAD,
    F_TRANSLATORTHREAD,
    F_FLEXCANTHREAD,
    F_BOARDTESTTHREAD
}F_Threads;  

      
typedef enum
{
    RT_UNKNOWN,
    RT_GLOBAL_SCALE_GOOD,
    RT_GLOBAL_SCALE_BETTER,
    RT_GLOBAL_SCALE_BEST,  
    RT_GLOBAL_FSS,
    RT_GLOBAL_SCALE_PRINTER_ONLY,
    RT_GLOBAL_PREPACK_PRINTER,
    RT_GLOBAL_SCALE_WEIGHER_ONLY,
    RT_GLOBAL_PREPACK_WEIGHER,
}PeripheralModel;

typedef enum{
  SUSPEND,
  RESUME,
  TERMINATE,
  LIST,
  CREATE
}ManagerCommand;

typedef enum{
  T_WEIGHER,
  T_PRINTER,
  T_USB,
  T_TRANSLATOR,
  T_SENSORS,
  T_UNIT_TEST,
  T_DOT_WEAR,
  LAST_TASK 
}ManagedThreads;

/* These define which bit correlates to which permission */
#define SUSPEND_RESUMEMASK      (0x01 << 0)
#define TEMINATEMASK            (0x01 << 1)
#define CREATEMASK              (0x01 << 2)

/*Permission fields to allow the thread manager to manipulate tasks*/
#define WEIGHERPERMISSIONS              (TEMINATEMASK | SUSPEND_RESUMEMASK)
#define PRINTERPERMISSIONS              (TEMINATEMASK | SUSPEND_RESUMEMASK)
#define USBPERMISSIONS                  (SUSPEND_RESUMEMASK)
#define TRANSLATORPERMISSIONS           (SUSPEND_RESUMEMASK)
#define SENSORSPERMISSIONS              (TEMINATEMASK | SUSPEND_RESUMEMASK | CREATEMASK)
#define UNITTESTPERMISSIONS             (TEMINATEMASK | SUSPEND_RESUMEMASK)
#define DOTWEARPERMISSIONS              (TEMINATEMASK | SUSPEND_RESUMEMASK | CREATEMASK)

struct manager_msg
{
    ManagerCommand cmd;
    ManagedThreads task; 
};
typedef struct manager_msg ManagerMsg;

/* public functions */
void systemStartup( void );
void startUnitTest( void );
void terminateThread( F_Threads thread_ );
void initializeThreadStatistics( void );
PeripheralModel getMyModel( void );
BaseType_t createThreadManagerTask( void );
void assignThreadManagerMsgQueue( QueueHandle_t pQHandle );
void updateTaskHandle(ManagedThreads thread);
void delay_uS ( unsigned int time );


/* private functions */
static void startSystemThreads( void );
static void initializeMemoryPool( void );
static void initializeModelIO( void );
static PeripheralModel getPeripheralModel( void );

static void threadManager( void *pvParameters );
static void handleManagerMsg(ManagerMsg * msg);
static void tm_createSensorTask();
static bool tm_createDotWearTask();

#ifdef PRINT_STATS
void statsTimerCallback( TimerHandle_t xTimer );
#endif
#endif