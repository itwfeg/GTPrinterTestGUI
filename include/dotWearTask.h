#ifndef DOTWEARTASK_H
#define DOTWEARTASK_H
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "globalPrinter.h"
#include <stdbool.h>

#define dot_wear_task_PRIORITY ( configMAX_PRIORITIES -0 )

typedef enum
{
  DOT_GOOD,
  DOT_BAD,
  DOT_MARGINAL,
}headDotStatus;

#define HEAD_CATEGORIES         8

/* Median Resistance Values Correlate To The Head Resistance ID ; */
//                                                           716ohm          812ohm         764ohm         860ohm
static const int MEDIAN_RESISTANCE[HEAD_CATEGORIES][3] = {{716,1432,2750}, {812,1624,2750}, {764,1528,2750}, {860,1720,2750}, //{DOT_RESISTANCE_NOMINAL, DOT_RESISTANCE_MARGINAL, DOT_RESISTANCE_BAD}
//                                                             740ohm        836ohm         788ohm         884ohm
                                                          {740,1480,2750}, {836,1672,2750}, {788,1576,2750}, {884,1768,2750}};

static int DOT_RESISTANCE_NOMINAL = 0;
static int DOT_RESISTANCE_MARGINAL = 0;
static int DOT_RESISTANCE_BAD = 0;

#define DOT_SAMPLE      1UL
#define DOT_DONE        2UL
#define DOT_RUN_COUNT 10

BaseType_t createDotWearTask( void );
void cleanupDotWearTask( void );
static void dotWearTask(void *pvParameters );
bool showHeadDotStatistics( void );
void sendHeadDotStatistics( void );
unsigned short getADCHeadWearDot( int x );
unsigned short getHeadWearDot( int x );
headDotStatus getHeadWearDotStatus( int x );
HeadStatus getHeadWearStatus( int size );
TaskHandle_t getDotWearHandle();
void recordADC();
void sendDotWear( int size );
void assignDotWearMsgQueue( QueueHandle_t pQHandle );
float calculateDotResistance(float driveVoltage, int counts);
bool readyToNotify(void);
//unsigned short max(unsigned short * list, int size);
//unsigned short min(unsigned short * list, int size);

#endif //DOTWEARTASK_H