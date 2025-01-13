#ifndef AVERY_WEIGHER_H
#define AVERY_WEIGHER_H
#include "weigher.h"
#include "wgMessages.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#include "valueMax.h"
#include "sca3300Accel.h"

#define weigher_task_PRIORITY ( configMAX_PRIORITIES - 1 )

#define TRACKING_COUNT          150

typedef enum 
{
    RT_WEIGHER_UNKNOWN,
    RT_WEIGHER_CS5530_30LBS    
}WeigherStyle;

typedef enum {

	POWERUP_,
        PRE_INIT_ZERO,
        INIT_ZERO_,
	LOCKUP_,
	DETECTMOTION_,
	ADRESET_,
	REZERO_
}WeigherStates;

#define SWITCH_DEBOUNCE_TICKS                   1000

//#define serviceSwitchPressed GPIO2_Combined_0_15_IRQHandler
/* prototypes */
BaseType_t createAveryWeigherTask( WeigherStyle style, QueueHandle_t msgQueue );
TaskHandle_t getWeigherHandle( void );
WeigherStyle getWeigherStyle( void );
void serviceSwitchPressed( void );
void debounceCallback( TimerHandle_t timer );
void weigherCountsStartStop( bool start );
bool isServiceSwitchPressed(void);
bool isValueMaxOn(void);
void setValueMaxOff(void);
bool isHostConnected(void);
void setWeigherStatusBit(unsigned short wgStatusBit);
void initFitDefaults( void );
void initFit(fit_flash_section *fitParamFlash);
void valueMaxHandleAcclMeasurement( ValueMax *VM, SCA3300 *acclData );
void initFitDebug(fit_flash_section *fitParamFlash);


/* private functions */
static void getSerialConfiguration( WgConfiguration *pCfg );
static bool verifyConfiguration( WgConfiguration *pTemp, WgConfiguration *pSF );
static void initCalibrationSwitch( void );
static void averyWeigherTask( void *pvParameters );
static void handleWeigherMsg( WgMessage *pMsg );
static void clearServiceSwitch( void );
static void initializeFilter( int speed );
static long calibrateCounts( unsigned long counts );
static unsigned char calculateSlope( WeightStateType *current, WeightStateType *previous );
static void setZeroTracking( void );
static bool validZero( void );
static bool autoZero( void );
static void updateParameters(void);
static void initializeZero( unsigned long counts );
static void rezeroWeigher( unsigned long counts );
static void lockUp( unsigned long counts );
static void detectMotion( unsigned long counts );
static bool largeMotion( void );
static bool smallMotion( void );
static void handleAdConversion( AD_Counts *pCounts );
static void readFilteredWeigher( AD_Counts *pCounts );
static void filter( AD_Counts *pCounts );
static void monitorWeigher( AD_Counts *pCounts );
static void adReset( unsigned long counts );
static bool statusMsgFilter( WeightStateType *previous, WeightStateType *current );
static void updateHistory( WeightStateType *current, WeightStateType *previous );
static void setWeigherVersion( WgVersion *pVersion );
void enableWeigher( void );
void disableWeigher( void );




#endif
