#ifndef AVERYPRINTER_H
#define AVERYPRINTER_H
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#include "prMessages.h"

#define printer_task_PRIORITY ( configMAX_PRIORITIES - 1 )

/* prototypes */
BaseType_t createAveryPrinterTask( QueueHandle_t msgQueue );


/* private functions */
static void averyPrinterTask( void *pvParameters );
static void handlePrinterMsg( PrMessage *pMsg );
static void getPrVersion( PrVersion *pVersion );
#endif