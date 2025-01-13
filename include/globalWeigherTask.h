#ifndef GLOBALWEIGHERTASK_H
#define GLOBALWEIGHERTASK_H
#include "wgMessages.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

#define weigher_task_PRIORITY ( configMAX_PRIORITIES - 1 )




/* prototypes */
BaseType_t createGlobalWeigherTask( QueueHandle_t msgQueue );


/* private functions */
static void globalWeigherTask( void *pvParameters );
static void handleWeigherMsg( WgMessage *pMsg );
static void setWeigherVersion( WgVersion *pVersion );


#endif