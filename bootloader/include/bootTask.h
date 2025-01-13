#ifndef BOOTTASK_H
#define BOOTTASK_H
#include "bootloader.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "threadmanager.h"

#define boot_task_PRIORITY ( configMAX_PRIORITIES - 3 )
#define BOOT_READY_SEND_RATE pdMS_TO_TICKS(10000)//1500////500

typedef struct boot_state
{
    unsigned short      model;
    Version             boot_version;
    Version             printer_app_version;
	Version             weigher_app_version;
    BootStatusCode      status;
    unsigned long       fault_info;
    BootLoaderFaults    fault;	  
} BootState;

/*
typedef struct boot_frame_header
{
  bool                upgradeInProgress;
  bool                writeReady;
  unsigned short      number_of_frames;
  unsigned short      segmentOrder[MAX_FRAMES]; // the order in which the segments were received
  unsigned short      final_segment_length;
  uint32_t            *pFrameBuffer;
  unsigned short      total_frame_transfers;
  unsigned short      current_frame;
  unsigned long       totalNumBytes;
  uint32_t            current_address;
} BootFrameHeader;
*/

typedef struct
{
    bool        upgradeInProgress;
    uint16_t    numFrames;
    uint16_t    currentFrame;
    uint32_t    currentAddr;
    uint32_t    length;
} BootFrameHeader;

BaseType_t createBootTask( QueueHandle_t bootMsgQueue );

static void bootTask( void *pvParameters );
void bootReadyCallback(TimerHandle_t boot_ready);
void resetTmrCallback(TimerHandle_t tmr);
bool stopBootReadyTimer(void);
bool startBootReadyTimer(void);
bool startResetTmr(void);
bool stopResetTmr(void);

static void handleBootloaderMsg( BootMsg * btMsg );
static void sendStateStatus( void );
static void handleBootCmd( BootCmdMsg * msg );
static void jumpToApp( void );
static void initializeAppUpgrade( void );
static void handleFrameData( void );
bool verifyApp( void );
uint32_t calculateChecksumBoot( uint32_t start, uint32_t end );

#if 0
void assignBootMsgQueue( QueueHandle_t pQHandle );
static void handleFrameHeader( BootFrameHeaderMsg * frameHeader );
static void handleFrameData( BootFrameMsg * frame );
static void handleWrapperBootVersion( BootGenericMsg * btMsg );
//uint8_t calculateChecksumBoot( uint32_t start, uint32_t end );
uint16_t getCanIDfromBootFrame( BootMsgCode msgID );
#endif
#endif