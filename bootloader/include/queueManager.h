#ifndef QUEUEMANAGER_H
#define QUEUEMANAGER_H
#include "FreeRTOS.h"
#include "queue.h"
#include "threadManager.h"
#include "bootloader.h"

/* USB IN queue dimensions */
#define MAX_USB_IN_LENGTH       90
#define MAX_USB_IN_DEPTH        64

/* USB OUT queue dimensions */
#define MAX_USB_OUT_LENGTH      30
#define MAX_USB_OUT_DEPTH       64

/* USB OUT BLK queue dimensions */
#define MAX_USB_BLK_OUT_LENGTH  2
#define MAX_USB_BLK_OUT_DEPTH   512

/* Thread Manager queue dimensions */
#define MAX_TM_MSG_LENGTH       20
#define MAX_TM_MSG_DEPTH        sizeof(ManagerMsg)

/*printer queue dimensions */
#define MAX_BOOT_MSG_LENGTH       20
#define MAX_BOOT_MSG_DEPTH        sizeof(BootMsg)

/* public functions */
QueueHandle_t getBootQueueHandle( void );
QueueHandle_t getUsbInBootQueueHandle( void );
QueueHandle_t getUsbOutBootQueueHandle( void );
QueueHandle_t getThreadManagerQueueHandle( void );

int getBootMsgQueueLength( void );
int getUsbInQueueLength( void );
int getUsbOutQueueLength( void );
unsigned int getQueueDepth( QueueHandle_t handle_ );
unsigned int getQueueSize( QueueHandle_t handle_ );

void initializeQueueManager( void );
bool addThreadManagerMsg( unsigned char * pMsg );

/*****************************************************************************/
void deleteBootQueue( void );
void createBootQueue( void );
QueueHandle_t getBootQueueHandle( void );
/*****************************************************************************/

#endif
