#include "threadManager.h"
#include "queueManager.h"
#include "task.h"
#include "portable.h"
#include "fsl_clock.h"
#include "fsl_gpt.h"
#include "systemTimer.h"
#include "sensors.h"
#include "globalPrinterTask.h"
#include "averyWeigher.h"
#include "usbTask.h"
#include "idleTask.h"
#include "vendor.h"
#include "pin_mux.h"
#include "fsl_pit.h"
#include "unitTest.h"
#include "fsl_debug_console.h"
#include "translator.h"
#include "takeupMotor.h"
#include "developmentSettings.h"
#include "w25x10cl.h"   
#include "serialFlash.h"



static TaskHandle_t     managerHandle_                          = NULL;
static QueueHandle_t    managerQHandle_                         = NULL;
static TaskHandle_t     managedTaskHandles_[LAST_TASK]          = NULL;
/* here is how this works each index in the array corelates to the ManagedThreads
   enmum in threadManager.h The masks determine what permissions correlate to each
   bit. So if (taskPermissions[WEIGHER] & TEMINATEMASK) then you can terminate
   the managedTaskHandles_[WEIGHER] task
*/
static uint8_t taskPermissions[LAST_TASK]               = { WEIGHERPERMISSIONS, 
                                                            PRINTERPERMISSIONS,
                                                            USBPERMISSIONS, 
                                                            TRANSLATORPERMISSIONS,
                                                            SENSORSPERMISSIONS, 
                                                            UNITTESTPERMISSIONS,
                                                            DOTWEARPERMISSIONS };
/* set to 1 to print out the task list at a 1 sec interval */
#ifdef PRINT_STATS 
TimerHandle_t statsTimer                =  NULL;
#endif

static bool suspend_                    = false;

volatile unsigned long  statCntr        = 0;
static PeripheralModel  model_          = RT_UNKNOWN;
static int              totalHeap_      = 0;

extern Pr_Config config_;
extern PrStatusInfo                     currentStatus;
extern PrStatusInfo                     prevStatus;
extern volatile ADCManager              adcManager;

/******************************************************************************/
/*!   \fn void systemStartup( void )

      \brief
        This function intializes kernel resources, starts up system resources 
        and determines system type to start peripheral threads.

      \author
          Aaron Swift
*******************************************************************************/
void systemStartup( void )
{
    initializeQueueManager();

    //PRINTF("systemStartup(): Global Scale Controller RT1024DAG5A\r\n");
    printSWVersions();
    uint32_t freq = CLOCK_GetPllFreq(kCLOCK_PllSys);
    //PRINTF( "System clk: %dMhz\r\n", ( freq / 1000000 ) );
    uint32_t clock = CLOCK_GetSysPfdFreq( kCLOCK_Pfd0 );
    //PRINTF( "Pfd0 clk: %dMhz\r\n", ( clock / 1000000 ) );
    clock = CLOCK_GetSysPfdFreq( kCLOCK_Pfd1 );
    //PRINTF( "Pfd1 clk: %dMhz\r\n", ( clock / 1000000 ) );
    clock = CLOCK_GetSysPfdFreq( kCLOCK_Pfd2 );
    //PRINTF( "Pfd2 clk: %dMhz\r\n", ( clock / 1000000 ) );
    clock = CLOCK_GetSysPfdFreq( kCLOCK_Pfd3 );
    //PRINTF( "Pfd3 clk: %dMhz\r\n", ( clock / 1000000 ) );    
    size_t space = xPortGetFreeHeapSize();
    //PRINTF( "Heap allocation: %dK\r\n", ( space / 1000 ) );
    //PRINTF( "\r\n" );
        
    /* intialize timer resources*/
    /*
    PRINTF("systemStartup(): initialize timer resources\r\n");
    initTimers(); 
    if( createHeartBeatTimer() )
        startHeartBeatTimer();
    */

    /* initialize the serial flash interface */
    if( initializeSerialFlash() ) {
        //PRINTF("systemStartup(): serial flash interface initialized\r\n");
        if( !initSerialFlashMutex() ) {
            //PRINTF("systemStartup(): failed to initialize serial flash mutex!\r\n");
            /* force reboot of scale */
            assert( 0 );            
        }
    } else {
        //PRINTF("systemStartup(): failed to initialize serial flash interface!\r\n");        
        /* force reboot of scale */
        assert( 0 );        
    }
    /* determine our model */
    model_ = getPeripheralModel();  
    
    /* setup the sensors in auto mode */
    //PRINTF("systemStartup(): initialize sensors interface\r\n");
    createSensorsTask( AD_AUTO );    // AD_AUTO AD_MANUAL
    //createSensorsTask( AD_MANUAL );    
    
    //char IDF_1 = GPIO_PinRead(GPIO1, 7U);
    char IDF_2 = GPIO_PinRead(GPIO1, 8U);
    
    //PRINTF("IDF1 = %d\r\n", IDF_1);
    //PRINTF("IDF2 = %d\r\n", IDF_2);

    //GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, true);
    
    
    HEADTYPE printheadType = getPrintHeadType();
    PrinterStyle printerStyle = RT_PRINTER_UNKNOWN;
 
    
    if(printheadType == ROHM_80MM_650_OHM)
    {
        //PRINTF("80MM printhead detected\r\n");
        //set global printer task env_ here
        printerStyle = RT_PRINTER_SERVICE_SCALE_80MM;
      
        //put the stepper current limit GPIO toggle here?
        GPIO_WritePinOutput( GPIO3, 5U, true ); //stepper current limit set A
        GPIO_WritePinOutput( GPIO3, 7U, true ); //stepper current limti set B
      
    }
    else if(printheadType == ROHM_72MM_800_OHM)
    {
        //PRINTF("72MM printhead detected\r\n");
        //set global printer task env_ here 
        printerStyle = RT_PRINTER_SERVICE_SCALE_72MM;
      
        //put the stepper current limit GPIO toggle here?
        GPIO_WritePinOutput( GPIO3, 5U, false ); //stepper current limit set A
        GPIO_WritePinOutput( GPIO3, 7U, false ); //stepper current limit set B
      
    }

    
    
    
    //PRINTF( "spawning global tasks\r\n" );    
    /* make sure we have our message queues */
    if( ( getUsbInPrQueueHandle() != NULL ) && ( getUsbInWrQueueHandle() != NULL ) && 
        ( getUsbOutPrQueueHandle() != NULL ) && ( getUsbOutWrQueueHandle() != NULL ) &&
       ( getWeigherQueueHandle() != NULL ) && ( getPrinterQueueHandle() != NULL ) ) {        
        if( createTranslator( getWeigherQueueHandle(), getPrinterQueueHandle(),
                              getUsbInPrQueueHandle(), getUsbInWrQueueHandle(),
                              getUsbOutPrQueueHandle(), getUsbOutWrQueueHandle() ) == pdTRUE ) {                                                           
            if( /*createUsbTask( (QueueHandle_t)getUsbInPrQueueHandle(), 
                               (QueueHandle_t)getUsbInWrQueueHandle()) == pdTRUE*/ 1 ) {
                 
                
                                 
                /* TO DO 80MM: get head type before creating the global printer task */  
                if( createGlobalPrinterTask(/* RT_PRINTER_SERVICE_SCALE_80MM */ printerStyle, (QueueHandle_t)getPrinterQueueHandle() ) == pdTRUE ) {                   
               
                
                //PRINTF("backing paper only - %d\r\n", config_.backingPaper);
                //PRINTF("backing paper and label - %d\r\n", config_.backingAndlabel);
                                 
               /* if( createGlobalPrinterTask( RT_PRINTER_SERVICE_SCALE_72MM, (QueueHandle_t)getPrinterQueueHandle() ) == pdTRUE ) {  */               
                    if( /*createAveryWeigherTask( RT_WEIGHER_CS5530_30LBS, (QueueHandle_t)getWeigherQueueHandle() ) == pdTRUE*/ 1 ) {
                      
                        createIdleTask();
                        
                        /* start created threads */
                        startSystemThreads(); 
                             
                    } else {
                        //PRINTF("systemStartup(): Falied to create Weigher task!\r\n");
                    }        
                } else {
                    //PRINTF("systemStartup(): Falied to create Printer task!\r\n");  
                }    
            }
        }        
    }
}

/******************************************************************************/
/*!   \fn BaseType_t createThreadManagerTask( void )

\brief
	This function creates a freeRTOS thread for the thread manager.

\author
	Eric Landes
*******************************************************************************/
BaseType_t createThreadManagerTask( void )
{
    BaseType_t result = xTaskCreate( threadManager,  "Thread Manager", configMINIMAL_STACK_SIZE,
                                  NULL, manager_task_PRIORITY, &managerHandle_ );
#ifdef PRINT_STATS
    statsTimer = xTimerCreate("Stat Timer", pdMS_TO_TICKS(1000), pdTRUE, (void *) 0, statsTimerCallback);
    xTimerStart( statsTimer, 0);
#endif
    return result;
}

/******************************************************************************/
/*!   \fn static void threadManager( void *pvParameters )

\brief
	This function is the thread manager run thread.

\author
	Eric Landes
*******************************************************************************/
static void threadManager( void *pvParameters )
{
    PRINTF("threadManager(): Thread running...\r\n" );
    while( !suspend_ ) {
        ManagerMsg MMsg;
        if( xQueueReceive(managerQHandle_, &MMsg, portMAX_DELAY  ) ) {
            ManagerMsg newMsg;
            memcpy( &newMsg, &MMsg, sizeof(ManagerMsg) );
            handleManagerMsg( &newMsg );
        } else {
            PRINTF("threadManager(): Failed to get manager message from queue!\r\n" );
        }
        taskYIELD();
    }
    vTaskSuspend(NULL);
}

/******************************************************************************/
/*!   \fn static void handleManagerMsg( void *pvParameters )

\brief
    handles messages for the thread manager task

\author
    Eric Landes

*******************************************************************************/
static void handleManagerMsg(ManagerMsg * msg)
{
    
    PRINTF("handleManagerMsg(): New Manager Message. Cmd: %d Task: %d\r\n",msg->cmd, msg->task);
    if( msg->cmd == LIST ) {
        PRINTF("ThreadManager statistics ***************************************\r\n");
        PRINTF("Task list ******************************************************\r\n");
        char buffer[ 40 * TASK_COUNT ] = {0};
        vTaskList( (char *)&buffer );
        PRINTF( buffer );
        PRINTF("****************************************************************\r\n");
        PRINTF("Heap ***********************************************************\r\n");
        int size = xPortGetFreeHeapSize();
        PRINTF("Total heap size: %d\r\n", totalHeap_ );
        PRINTF("Free heap: %d\r\n", size );
        PRINTF("****************************************************************\r\n");
    } else if(msg->task < LAST_TASK ) {
        switch(msg->cmd ) 
        {
            case SUSPEND:
            if( managedTaskHandles_[msg->task] != NULL ) {
                if( ( taskPermissions[msg->task] & SUSPEND_RESUMEMASK ) ) {
                    vTaskSuspend(managedTaskHandles_[msg->task]);
                    PRINTF("handleManagerMsg(): %s Task Suspended\r\n",
                    pcTaskGetName( managedTaskHandles_[msg->task]));
                } else {
                    PRINTF("handleManagerMsg(): Insuffecient privileges to suspend %s \r\n",
                            pcTaskGetName( managedTaskHandles_[msg->task]));
                }
            } else {
                PRINTF("handleManagerMsg(): The given task was not initialized\r\n");
            }
            break;
            case RESUME:
            if( managedTaskHandles_[msg->task] != NULL ) {
                if( ( taskPermissions[msg->task] & SUSPEND_RESUMEMASK ) ) {
                    vTaskResume(managedTaskHandles_[msg->task]);
                    PRINTF("handleManagerMsg(): %s Task Resumed\r\n",
                    pcTaskGetName( managedTaskHandles_[msg->task]));
                } else {
                    PRINTF("handleManagerMsg(): Insuffecient privileges to resume %s \r\n",
                            pcTaskGetName( managedTaskHandles_[msg->task]));
                }
            } else {
                PRINTF("handleManagerMsg(): The given task was not initialized\r\n");
            }
            break;
            case TERMINATE:
            if( managedTaskHandles_[msg->task] != NULL ) {
                if( ( taskPermissions[msg->task] & TEMINATEMASK ) ) {
                    PRINTF("handleManagerMsg(): %s Task Terminated\r\n",
                    pcTaskGetName( managedTaskHandles_[msg->task]));
                    vTaskDelete(managedTaskHandles_[msg->task]);
                    managedTaskHandles_[msg->task] = NULL;
                } else {
                    PRINTF("handleManagerMsg(): Insuffecient privileges to kill %s \r\n",
                            pcTaskGetName( managedTaskHandles_[msg->task]));
                }
            } else {
                PRINTF("handleManagerMsg(): The given task was not initialized\r\n");
            }
            break;
            case CREATE:
            if( managedTaskHandles_[msg->task] == NULL ) {
                if( ( taskPermissions[msg->task] & CREATEMASK ) ) {
                    if( msg->task == T_SENSORS ) {
                        tm_createSensorTask();
                        PRINTF("handleManagerMsg(): %s Task Created\r\n",
                        pcTaskGetName( managedTaskHandles_[msg->task]));
                    }else if(msg->task == T_DOT_WEAR){
                        /* only allow dotwear if best model */
                        if( model_ == RT_GLOBAL_SCALE_BEST) {
                          tm_createDotWearTask();
                          PRINTF("handleManagerMsg(): %s Task Created\r\n",
                          pcTaskGetName( managedTaskHandles_[msg->task]));
                        }
                    }
                }else{
                    PRINTF("handleManagerMsg(): Insuffecient privileges to create task\r\n");
                }
            } else {
                PRINTF("handleManagerMsg(): The given task was already created!\r\n");
                char buffer[40* TASK_COUNT] = {0};
                vTaskList((char *)&buffer);
                PRINTF(buffer);
            }
            break;
          case LIST:
            break;
        }
    } else {
        PRINTF("handleManagerMsg(): Invalid Task Handle Command!\r\n" );
    }
}

void updateTaskHandle(ManagedThreads task)
{
    switch (task)
    {
        case T_SENSORS:
            managedTaskHandles_[T_SENSORS] = (TaskHandle_t)getSensorsHandle();
            break;
        default:
            PRINTF("handleManagerMsg(): Failed to update task handle!\r\n" );
        break;
    }
}

/******************************************************************************/
/*!   \fn assignThreadManagerMsgQueue( QueueHandle_t pQHandle )

      \brief
        This function assigns the transmit queue handle.

      \author
            Eric Landes
*******************************************************************************/
void assignThreadManagerMsgQueue( QueueHandle_t pQHandle )
{
    managerQHandle_ = pQHandle;
}

#if 0
/******************************************************************************/
/*!   \fn static void startUnitTest( void )

      \brief
        This function creates and starts unit test thread.

      \author
          Aaron Swift
*******************************************************************************/
void startUnitTest( void )
{
    if( createUnitTestTask() ) {
        PRINTF("startUnitTest(): Created Unit test thread.\r\n" );
        if( getCanQueueHandle() != NULL ) {
            assignMsgQueue( getTestQueueHandle() );
        }
        else {
            PRINTF("systemStartup(): CAN Queue handle NULL!\r\n" );
        }
    }
    else {
        PRINTF("systemStartup(): Failed to create Unit test thread!\r\n" );
    }
}
#endif


#if 0 /* TO DO: might not be needed */
/******************************************************************************/
/*!   \fn static void initializeMemoryPool( void )

      \brief
        This function intializes heap space for kernel objects.

      \author
          Aaron Swift
*******************************************************************************/
static void initializeMemoryPool( void )
{
    /* define the heap reagion with a depth of 2 sections of 40k bytes in the top of the
       upper section of the SRAM just below the stack. Linker file was modified
       increase the heap size to 60K.  */
    const HeapRegion_t xHeapRegions[] =
    {
        { ( uint8_t * ) 0x2001bbffUL, 0xa000 },
        { ( uint8_t * ) 0x20025bffUL, 0x5000 },
        { NULL, 0 }
    };
    vPortDefineHeapRegions( xHeapRegions );
    totalHeap_ = xPortGetFreeHeapSize();
    if( totalHeap_ == 0 ) {
        PRINTF("initializeMemoryPool(): Failed to create Heap space!\r\n" );
    }
    else {
        PRINTF("initializeMemoryPool(): Heap size: %d\r\n", totalHeap_ );
    } 
}
#endif 

/******************************************************************************/
/*!   \fn static void startSystemThreads( void )

      \brief
        This function starts the kernel scheduler.

      \author
          Aaron Swift
*******************************************************************************/
static void startSystemThreads()
{
    /* once the threads are started the scheduler should not return unless
    we run out of heap space */
    vTaskStartScheduler();
    while( 1 ) {
        PRINTF("startSystemThreads(): Critical error out of heap space!\r\n");
    }
}

/******************************************************************************/
/*!   \fn static void startSystemThreads( void )

      \brief
        This function returns what type of device we are based on logic 
        of the model pins.
        different configurations:
        15 (default) = Global Scale Good
        14 = Global Scale Better
        13 = Global Scale Best
        12 = Global Scale Free Standing
        11 = Global Scale Printer Only
        10 = Global PrePack Printer
        9  = Global PrePack Weigher
        8  = Reserved
        7  = Reserved
        6  = Reserved
        5  = Reserved
        4  = Reserved
        3  = Reserved
        2  = Reserved
        1  = Reserved
        0  = Reserved

      \author
          Aaron Swift
*******************************************************************************/
static PeripheralModel getPeripheralModel( void )
{
    PeripheralModel model = RT_UNKNOWN;
    unsigned char modelPins = 0;
    /* read input pins to determine model type */
    if( GPIO_ReadPinInput( MODEL_TYPE_A_GPIO, MODEL_TYPE_A_PIN ) ) {
        modelPins  += 1;
    }
    if( GPIO_ReadPinInput( MODEL_TYPE_B_GPIO, MODEL_TYPE_B_PIN ) ) {
        modelPins  += 2;
    }
    if( GPIO_ReadPinInput( MODEL_TYPE_C_GPIO, MODEL_TYPE_C_PIN ) ) {
        modelPins  += 4;  
    }
    if( GPIO_ReadPinInput( MODEL_TYPE_D_GPIO, MODEL_TYPE_D_PIN ) ) {
        modelPins  += 8;  
    }
      
    
    if( modelPins == 15 ) {
        model = RT_GLOBAL_SCALE_GOOD;
        PRINTF("getPeripheralModel(): RT_GLOBAL_SCALE_GOOD\r\n");
    } else if( modelPins == 14 ) {
        model = RT_GLOBAL_SCALE_BETTER;
        PRINTF("getPeripheralModel(): RT_GLOBAL_SCALE_BETTER\r\n");
    } else if( modelPins == 13 ) {
        model = RT_GLOBAL_SCALE_BEST; 
        PRINTF("getPeripheralModel(): RT_GLOBAL_SCALE_BEST\r\n");
    } else if( modelPins == 12 ) {
        model = RT_GLOBAL_FSS;
        PRINTF("getPeripheralModel(): RT_GLOBAL_FSS\r\n");
    } else if( modelPins == 11 ) {
        model = RT_GLOBAL_SCALE_PRINTER_ONLY;
        PRINTF("getPeripheralModel(): RT_GLOBAL_SCALE_PRINTER_ONLY\r\n");
    } else if( modelPins == 10 ) {
        model = RT_GLOBAL_PREPACK_PRINTER;
        PRINTF("getPeripheralModel(): RT_GLOBAL_PREPACK_PRINTER\r\n");
    } else if( modelPins == 9 ) {
        model = RT_GLOBAL_SCALE_WEIGHER_ONLY;
        PRINTF("getPeripheralModel(): RT_GLOBAL_SCALE_WEIGHER_ONLY\r\n");
    } else if( modelPins == 8 ) {
        model = RT_GLOBAL_PREPACK_WEIGHER; 
        PRINTF("getPeripheralModel(): RT_GLOBAL_PREPACK_WEIGHER\r\n");
    } else if( modelPins < 8 ) {
        model = RT_UNKNOWN;
        PRINTF("getPeripheralModel(): RT_UNKNOWN\r\n");
    }
    
    return model;
}

/******************************************************************************/
/*!   \fn PeripheralModel getMyModel( void )

      \brief
        This function returns the peripheral model.

      \author
          Aaron Swift
*******************************************************************************/
PeripheralModel getMyModel( void )
{
    return model_;
}

/******************************************************************************/
/*!   \fn static void tm_createSensorTask()

      \brief
        Helper to allow the tread manager to create the sensors task.

      \author
          Eric Landes
*******************************************************************************/
static void tm_createSensorTask(){
    /* configuration management determine what are we? ( FPC, Printer, Weigher ) */
    if( createSensorsTask(false) ) {
    	managedTaskHandles_[T_SENSORS] = (TaskHandle_t)getSensorsHandle();
    } else {
        PRINTF("systemStartup(): Failed to create sensors thread!\r\n" );
    }
}

/******************************************************************************/
/*!   \fn static void tm_createDotWearTask()

      \brief
        Helper to allow the thread manager to create the dot wear task.

      \author
          Eric Landes
*******************************************************************************/
static bool tm_createDotWearTask(){
    /* configuration management determine what are we? ( FPC, Printer, Weigher ) */
    if( createDotWearTask() ) {
    	managedTaskHandles_[T_DOT_WEAR] = (TaskHandle_t)getDotWearHandle();
        return true;
    } else {
        PRINTF("systemStartup(): Failed to create dot wear task!\r\n" );
        return false;
    }
}


#define TIMING_VALUE_uS 300   //k64 @96Mhz clk
/*! ****************************************************************************   
      \fn void delay_uS (unsigned int time)                                                              
 
      \author
          Aaron Swift
*******************************************************************************/ 
void delay_uS( unsigned int time )
{
    unsigned int i, j;
    
    if( time > 0 ) {
      /* the overhead of the function takes about 1uS */
      time--;
    }
    
    for( i = 0; i < time; i++ ) { 
      j = TIMING_VALUE_uS;
      while(j--){}
    }
}

#ifdef PRINT_STATS
void statsTimerCallback( TimerHandle_t xTimer )
{
    ManagerMsg msg;
    msg.cmd = LIST;
    addThreadManagerMsg(&msg);
}
#endif

