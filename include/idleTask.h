#ifndef IDLETASK_H
#define IDLETASK_H
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#include <stdbool.h>




//#define idle_task_PRIORITY ( configMAX_PRIORITIES -3 )
#define idle_task_PRIORITY ( configMAX_PRIORITIES - 1 )

/* prototypes */
BaseType_t createIdleTask( void );
static void idleTask( void *pvParameters );

static void initTest( void );
static void calTest( void );
static void startTest( void );
static void sizeAndPrintTest( void );
static void resultsTest( void );

void waitForCassetteOpen( void );
void waitForCassetteClose( void );

void cycleRedLED( void );
void cycleGreenLED( void );

void setTUStuckTestPass( bool testStatus );
bool getTUStuckTestPass( void );

void setTUChangeTestPass( bool testStatus );

void setSHOOTChangeTestPass( bool testStatus );

void setLOWSTOCKChangeTestPass( bool testStatus );

#endif