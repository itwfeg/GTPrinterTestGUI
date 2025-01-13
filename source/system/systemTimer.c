#include "systemTimer.h"
#include "semphr.h"
#include "fsl_gpio.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"


static TimerHandle_t heartBeat_; 
static SemaphoreHandle_t xMutex_;
unsigned long sTick = 0;

void initTimers( void )
{
    heartBeat_ = NULL;
        
    /* create mutex for one shot timers */
    xMutex_ = xSemaphoreCreateMutex(); 
    if( !xMutex_ ) {     
      PRINTF("initTimers(): Failed to create mutex!\r\n" );
    }
}

/******************************************************************************/
/*!   \fn bool createHeartBeatTimer( void )

      \brief
        This function creates a software timer for a heart beat indicator.
   
      \author
          Aaron Swift
*******************************************************************************/
bool createHeartBeatTimer( void )
{
    bool result = false;
    heartBeat_  = xTimerCreate( "heartBeat", (TickType_t)HEARTBEAT, pdTRUE, ( void * ) 0, heartBeatCallback );
    if( heartBeat_ != NULL ) {
        PRINTF("createHeartBeatTimer(): Timer created!\r\n" );
        result = true;
    }
    
    gpio_pin_config_t config = { kGPIO_DigitalOutput, 0, };

    GPIO_PinInit( HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, &config );
    return result;
}

/******************************************************************************/
/*!   \fn bool startHeartBeatTimer( void )

      \brief
        This function starts the software timer.
   
      \author
          Aaron Swift
*******************************************************************************/
bool startHeartBeatTimer( void )
{
    bool result = false;
    if( xTimerStart( heartBeat_, 0 ) == pdPASS ) {
        PRINTF("startHeartBeatTimer(): Timer started!\r\n" );
        result = true;
    }
    return result;
}


/******************************************************************************/
/*!   \fn bool heartBeatCallback( void )

      \brief
        This function software timer callback function.
   
      \author
          Aaron Swift
*******************************************************************************/
void heartBeatCallback( TimerHandle_t heartBeat_ )
{

    /* the number of callbacks is stored in the timer object */
    sTick = ( uint32_t ) pvTimerGetTimerID( heartBeat_ );
    
    sTick++;
    if( sTick % 2 == 0 ) { 
        /* turn off heartbeat LED */
        GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, false);
        GPIO_WritePinOutput(STATUS_LED_GPIO, STATUS_LED_PIN, true);
    } else {
        /* turn on heartbeat LED */
        GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, true);
        GPIO_WritePinOutput(STATUS_LED_GPIO, STATUS_LED_PIN, false);
    }
    vTimerSetTimerID( heartBeat_, ( void * )sTick );
}

/******************************************************************************/
/*!   \fn unsigned long getSystemTick( void )

      \brief
        This function returns the system tick count.
   
      \author
          Aaron Swift
*******************************************************************************/
unsigned long getSystemTick( void )
{
    return sTick;
}

/******************************************************************************/
/*!   \fn TimerHandle_t createOneShot( unsigned long mSec, void (*f)( void ) )

      \brief
        This function creates a software one shot timer. It is protected with a 
        mutex since this function can be accessed by several threads at the same 
        time. The mutex will block for 50mSec waiting for the mutex to be available.
        Up to 10 one shot timers can be created at any givin time. 
   
      \author
          Aaron Swift
*******************************************************************************/
TimerHandle_t createOneShot( unsigned long mSec, void (*f)( TimerHandle_t timer_ ) )
{
    TimerHandle_t handle;
    
    if( xMutex_ ) {
        if( xSemaphoreTake( xMutex_, ( TickType_t )MUTEX_TIMEOUT ) == pdTRUE ) {
            handle  = xTimerCreate( "oneShot", (TickType_t)mSec, pdFALSE, ( void * ) 0, (*f) );
            if( handle != NULL ) {
                    //PRINTF("createOneShot(): Timer created and added to list!\r\n" );
                } else {                    
                    PRINTF("createOneShot(): failed to create timer!\r\n" );                    
                }
            xSemaphoreGive( xMutex_ );
        } else {
            handle = 0;                        
            PRINTF("createOneShot(): failed to acquire mutex!\r\n" );
        }
    }
    return handle;  
}

/******************************************************************************/
/*!   \fn bool startOneShot( TimerHandle_t timer )

      \brief
        This function starts the one shot timer.
   
      \author
          Aaron Swift
*******************************************************************************/
bool startOneShot( TimerHandle_t timer )
{ 
    bool result = false;
    if( timer != NULL ) { 
        if( xTimerStart( timer, 0 ) == pdPASS ) 
            result = true;                    
    }
    return result;
}

/******************************************************************************/
/*!   \fn bool deleteOneShot( TimerHandle_t timer )

      \brief
        This function deletes the one shot timer.
   
      \author
          Aaron Swift
*******************************************************************************/
bool deleteOneShot( TimerHandle_t timer )
{
    bool result = false;

    if( xTimerDelete( timer, 0 ) == pdPASS ) {
        result = true;
    }
    return result;
}

/******************************************************************************/
/*!   \fn void initializeExpansionPort( void )

      \brief
        This function initializes all expansion port io as output active low.
   
      \author
          Aaron Swift
*******************************************************************************/
void initializeExpansionPort( void )
{
    /* TO DO */
    /* initialize all extra io as output low */
    
}

