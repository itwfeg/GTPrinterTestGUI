#ifndef SYSTEMTIMER_H
#define SYSTEMTIMER_H
#include "FreeRTOS.h"
#include "timers.h"
#include "board.h"
#include <stdbool.h>


/* Timer callback functions execute in the context of the timer service task. 
    It is therefore essential that timer callback functions never attempt to block. 
    For example, a timer callback function must not call vTaskDelay(), 
    vTaskDelayUntil(), or specify a non zero block time when 
    accessing a queue or a semaphore. 
*/ 


#define HEARTBEAT       15//150


typedef struct
{
    int id;
    TimerHandle_t handle;
    TickType_t startTime;
    bool timeout;
}OneShot;

#define MAX_LIST_DEPTH  10
#define MUTEX_TIMEOUT   50      /*~  50mSec */

/* public functions */
void initTimers( void );
bool createHeartBeatTimer( void );
bool startHeartBeatTimer( void );
void heartBeatCallback( TimerHandle_t heartBeat_ );
unsigned long getSystemTick( void );
TimerHandle_t createOneShot( unsigned long mSec, void (*f)( TimerHandle_t timer_ )  );
bool startOneShot( TimerHandle_t timer );
bool deleteOneShot( TimerHandle_t timer );
void initializeExpansionPort( void );
#endif
