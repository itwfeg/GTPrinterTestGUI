#include "dotWearTask.h"
#include <stdlib.h>
#include "fsl_debug_console.h"
#include "serialFlash.h"
#include "w25x10cl.h"
#include "threadManager.h"
#include "queueManager.h"
#include "systemTimer.h"
#include "AveryPrinter.h"
#include "translator.h"
#include "printHead.h"
#include "commandTable.h"
#include "printEngine.h"
#include "sensors.h"
#include "averyCutter.h"
#include "vendor.h"
#include "queueManager.h"
#include "semphr.h"


//Set to 1 to continuously run the dot wear task
#define DOT_WEAR_LOOP 0

//-----------------------------Dot Wear Constants------------------------------
#define ADC_BUFFER_GAIN                 25.3
#define SENSE_GAIN                      50.0
#define SENSE_RESISTANCE                0.015
#define ADC_MAX_COUNTS                  4096.0
#define ADC_REFERENCE_VOLTAGE           3.30
//------------------------------------------------------------------------------

static bool suspend_ = false;
static TaskHandle_t pHandle_    = NULL;
static int ADCBaseline;
static QueueHandle_t pMsgQHandle_   = NULL;

static bool startTest_ = false;
static bool testPrint_ = false;
static int dotWearContrast = 7;

//#pragma default_variable_attributes = @ "DEFAULT_RAM0"

//static unsigned short DotWearResults[MAX_DOT_HEAD_SIZE][DOT_RUN_COUNT];

//#pragma default_variable_attributes =

extern PrStatusInfo currentStatus, prevStatus;
extern void initializeCmdSequence( CMDId id, PrStatusInfo *pStatus);
extern volatile ADCManager adcManager;
extern Pr_Config config_;
extern unsigned short indx_;

extern void disableWeighInterrupts( void );
extern void enableWeighInterrupts( void );

BaseType_t createDotWearTask( void ){
    BaseType_t result;
    
    startTest_ = false;
    testPrint_ = false;
    
    result = xTaskCreate( dotWearTask,  "dotWearTask", ( configMINIMAL_STACK_SIZE ) ,
			 NULL, dot_wear_task_PRIORITY, &pHandle_ );

    return result;
}

void cleanupDotWearTask( void ){

    /* reset line timer interrupt level for normal print operation */
    setLineTimerIntLevel( 2 );
    
    /* turn the fan back on TO DO:??
    GPIO_WritePinOutput(FAN_OFF_GPIOx, FAN_OFF_PINx, true); */
    
    /* set a/d module back to auto mode */
    setADCMode(AD_AUTO);
    
    startTest_ = false;
    testPrint_ = false;
    setHistoryEnabled(true);
    
    setEngineContrast( config_.contrast_adjustment );
    setHeadTimings();

#if DOT_WEAR_LOOP
    PrMessage prMsg;
    prMsg.generic.function_code = PR_REQ_DOT_STATUS;
    BaseType_t result = xQueueSendToBack( pMsgQHandle_, (void *)&prMsg, 0 );
#endif
}

/******************************************************************************/
/*!     \fn bool showHeadDotStatistics( void )

        \brief This function is the dotwear task.

        \author
        Aaron Swift
*******************************************************************************/
static void dotWearTask( void *pvParameters )
{
    ( void ) pvParameters;

    PRINTF("dotWearTask(): Thread running...\r\n" );
    PrMessage prMsg; 
    while( !suspend_ ) {
        
        xQueueReceive( pMsgQHandle_, &prMsg, portMAX_DELAY );
        
        switch(prMsg.generic.msgType) 
        {
            case PR_REQ_DOT_WEAR:       {
                sendDotWear( getHeadStyleSize() );
                break;
            }
            case PR_REQ_DOT_STATUS: {

            #if 0 /* TO DO: port */
            /* make sure cassette is engaged before starting test, 
               else 24v PH will be off. */  
            PrSensors sensors;
            readHeadUpSensor( &sensors );
           
            
            if( ( sensors.headup_reading & HEAD_UP ) == HEAD_UP ) {
                PrDotStatus stat;
                stat.msgType	        = PR_DOT_STATUS;
                stat.source 		= RT_GLOBAL_SCALE;
                stat.destination 	= 0;
                stat.head_status 	= DOT_WEAR_ABORTED_CASSETTE_OPEN;
                
                sendPrHeadDotStatus( stat );
                cleanupDotWearTask();
                break;
            }	  
            #endif  
            /* wait for the engine to become idle before starting test */
            while( getPrintEngine()->currentCmd.generic.directive != IDLE_DIRECTIVE ){
                taskYIELD();
            }

            /* set the line timer interrupt level for dotwear test*/
            setLineTimerIntLevel( 5 );
            
            /* turn off the fan to eliminate noise TO DO: ??
            GPIO_WritePinOutput(FAN_OFF_GPIOx, FAN_OFF_PINx, false); */           
            
            /* initialize dotwear variables */
            int runCounter = 0;
            float headVoltage = 0.0;
            bool firstSample = true;
            bool wearDone = false;
            startTest_ = false;
                        
            /* set print head contrast to highest value giving us longest dot 
               current pulse to measure */
            setEngineContrast( dotWearContrast );
            
            setHeadTimings();
            
            /* setup the ADC to manual mode so we can control the trigger point */
            setADCMode(AD_MANUAL);

            while( !wearDone ) {
              
                #if 0   /* TO DO: port timer to RT1024 timer */
                if( !startTest_ && ( getPrintEngine()->currentCmd.generic.directive == IDLE_DIRECTIVE ) ) {
                    FTM_StopTimer( ENGINE_TIMER_BASE );
                    /* intiate generic sizing */
                    //initializeCmdSequence( 8, &currentStatus );
                    FTM_StartTimer( ENGINE_TIMER_BASE, kFTM_SystemClock );
                    //vTaskDelay(pdMS_TO_TICKS(500)); 
                    startTest_ = true;
                    firstSample = true;
                    indx_ = 0;
                    setHistoryEnabled(false);
                    /* shutdown weigher interrupts to external ADC */
                    disableWeighInterrupts();
                }
                #endif
                
                if( ( getPrintEngine()->currentCmd.generic.directive == IDLE_DIRECTIVE ) && ( startTest_ ) && ( !testPrint_ ) )  {

                    /* fixed to 800Ohms for rhom print head */
                    DOT_RESISTANCE_NOMINAL = MEDIAN_RESISTANCE[1][0];
                    DOT_RESISTANCE_MARGINAL =  MEDIAN_RESISTANCE[1][1];
                    DOT_RESISTANCE_BAD = MEDIAN_RESISTANCE[1][2];
                   
                  
                    /* we are not driving the printhead so now is a good time to create a zero value for our readings */
                    /* enable head power */
                    setHeadPower( 1 );
                    /* Let Power Come Up*/
                    vTaskDelay(200);
                    /* clear var*/
                    ADCBaseline = 0;
                    /* average 3 samples */
                    headVoltage = sampleHeadVoltage() * .0130;
                    PRINTF("dotWearTask(): Head Voltage: %f counts %fV\r\n", headVoltage / .0130 , headVoltage);
                    
                    /* added i/o for time stamp debug 
                    GPIO_WritePinOutput( EX_P1_GPIOx, EX_P1_PINx, true ); */
                    
                    ADCBaseline = ( sampleHeadDot() + sampleHeadDot() + sampleHeadDot() +
                                    sampleHeadDot() + sampleHeadDot() + sampleHeadDot() ) / 6;
                                    
                     /* added i/o for time stamp debug
                    GPIO_WritePinOutput( EX_P1_GPIOx, EX_P1_PINx, false ); */
                    
                    PRINTF( "dotWearTask(): ADCBaseline: %d\r\n", ADCBaseline );
                    
                    initializeSelfCheck();
                  
                    /* To DO: set label alignment 
                    updateLabelAlignment(); */
                    PRINTF("dotWearTask(): label size %d\r\n" , getLabelSize());
                    if( 4999 == getLabelSize() ){
                        setIndirectData( RAM_0, STEPS_PER_LENGTH3_50 );
                    } else {
                        setIndirectData( RAM_0, getLabelSize() );
                    }

                    /* turn on print head */
                    //setHeadPower(1);

                    /* test pin to time samples with strobe line 
                    gpio_pin_config_t config = { kGPIO_DigitalOutput, 0, };
                    GPIO_PinInit( EX_P2_GPIOx, EX_P2_PINx, &config );
                    */
                    
                    /* intiate generic dot wear test command */
                    PRINTF("DotWear Running cycle %d\r\n",runCounter);
                    initializeCmdSequence( 10, &currentStatus );
                    testPrint_ = true;                   
                }
                if( testPrint_ ) {
                    
                    unsigned long gotIt = 0;
                    BaseType_t result = xTaskNotifyWait(0, 0xffffffff,&gotIt, pdMS_TO_TICKS(10000));
                    if ((gotIt & DOT_SAMPLE) == DOT_SAMPLE){
                        delay_uS(300); // added in to keep our measurements in the dot wear bubble
                        if(!firstSample ){
                            
                            /* added i/o for time stamp debug
                            GPIO_WritePinOutput( EX_P1_GPIOx, EX_P1_PINx, true ); */
                            
                            sampleHeadDotADC();
                            
                            /* added i/o for time stamp debug
                            GPIO_WritePinOutput( EX_P1_GPIOx, EX_P1_PINx, false ); */
                        }else{
                            firstSample = false;
                        }
                    }
                    if( (gotIt & DOT_DONE) == DOT_DONE ) {
                                                
                        /* re-enable weigher interrupts to external ADC */
                        enableWeighInterrupts();
                        testPrint_ = false;
                        startTest_ = false;
                        
                        /* removed -- ATS
                        for(int i =0; i < MAX_DOT_HEAD_SIZE; i++){
                            DotWearResults[i][runCounter] = calculateDotResistance(headVoltage, getADCHeadWearDot(i));
                        }
                        */
                        
                        if(runCounter >= (DOT_RUN_COUNT - 1)){
                            PrDotStatus stat;
                            stat.msgType = PR_DOT_STATUS;
                            stat.source = RT_GLOBAL_SCALE_GOOD;
                            stat.destination = 0;
                            stat.head_status = getHeadWearStatus(getHeadStyleSize());
                            sendPrHeadDotStatus( stat );                       
                            showHeadDotStatistics();
                            PRINTF("ANY DOTS BAD? %d\r\n", getHeadWearStatus(getHeadStyleSize()));
                           
                            cleanupDotWearTask();
                            wearDone = true;
                        }
                        indx_ = 0;
                        runCounter++;
                        firstSample=true;                        
                    }
                    if(result != pdTRUE){
                        PRINTF("DOT WEAR FAILED: Task Timed Out\r\n");

                        PrDotStatus stat;
                        stat.msgType = PR_DOT_STATUS;
                        stat.source = RT_GLOBAL_SCALE_GOOD;
                        stat.destination = 0;
                        stat.head_status = DOT_FAILED;
                        sendPrHeadDotStatus( stat );
                        cleanupDotWearTask();
                        wearDone = true;
                    }
                }
                taskYIELD();
            }
            break;
        }
        default:
          break;
        }
    }
    vTaskSuspend(NULL);
}

/******************************************************************************/
/*!     \fn bool showHeadDotStatistics( void )

        \brief This function shows the results of the print head dot resistance
        test.

        \author
        Aaron Swift
*******************************************************************************/
bool showHeadDotStatistics( void )
{
    bool done = false;
    float resolution = 0.0008057;
#if 0
    PRINTF( "*********************** HT Plus Dot Measurement ***********************\r\n"  );
    PRINTF( "dot position:dot Sample 1:dot Sample 2:dot Sample 3:dot average reading:dot average voltage\r\n");
    for( int i = 0; i < HEAD_DOTS_72MM + 5; i++ ) {

        int average = getHeadWearDot(i);
        PRINTF( "%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d\r\n", adcManager.results[i].dotPosition, DotWearResults[i][0], DotWearResults[i][1], DotWearResults[i][2],
               DotWearResults[i][3], DotWearResults[i][4], DotWearResults[i][5], DotWearResults[i][6], DotWearResults[i][7], DotWearResults[i][8], DotWearResults[i][9],
               average);
        done = true;
        //PRINTF( "***********************************************************************\r\n"  );
        //No Time Slicing so we have to yeild so other tasks can run
        if(i % 20 == 0){
        taskYIELD();
        }
    }
#endif    
    return done;
}

/******************************************************************************/
/*!     \fn unsigned char getHeadWearDot( int x )

        \brief
        This function returns unsigned char value at location x in the
        head dot wear buffer.

        \author
        Aaron Swift
*******************************************************************************/
unsigned short getADCHeadWearDot( int x )
{
#if 0    
    return (( (adcManager.results[x].sample1) + (adcManager.results[x].sample2) +
	     (adcManager.results[x].sample3)) / 3);
#endif
}

/******************************************************************************/
/*!     \fn unsigned char getHeadWearDot( int x )

        \brief
        This function returns the average value for the dot at the index X.

        \author
        Aaron Swift
*******************************************************************************/
unsigned short getHeadWearDot( int x )
{
    unsigned long accumulator = 0;
    /*
    for(int i = 0; i < DOT_RUN_COUNT; i++){
        accumulator += (DotWearResults[x][i]);
    }
    //remove largest and smallest reading
    accumulator -= max( (unsigned short *)&( DotWearResults[x] ), DOT_RUN_COUNT );
    accumulator -= min( (unsigned short *)&( DotWearResults[x] ), DOT_RUN_COUNT );
    return (unsigned short)( accumulator / ( DOT_RUN_COUNT - 2 ) );
    */
    return 0;
}

/******************************************************************************/
/*!    \fn unsigned short max( unsigned short *list, int size ) 

        \brief
        This function finds and returns the highest value in the list.

        \author
        Aaron Swift
*******************************************************************************/
unsigned short max( unsigned short *list, int size ) 
{
    unsigned short max = 0;
    for( int i = 0; i < size; i++){
        if( max < list[i])
          max = list[i];
    }
    return max;
}

/******************************************************************************/
/*!    \fn unsigned short min( unsigned short *list, int size ) 

        \brief
        This function finds and returns the lowest value in the list.

        \author
        Aaron Swift
*******************************************************************************/
unsigned short min( unsigned short *list, int size ) 
{
    unsigned short min = 0xFFFF;
    for( int i = 0; i < size; i++){
        if( min > list[i])
          min = list[i];
    }
    return min;
}

/******************************************************************************/
/*!    \fn unsigned char getHeadWearDotStatus( int x )

        \brief
        This function calculates if a dot is good bad or marginal.

        \author
        Aaron Swift
*******************************************************************************/
headDotStatus getHeadWearDotStatus( int x )
{
    int avg = getHeadWearDot(x);
    
    //Higher counts is lower resistance
    if( avg >= DOT_RESISTANCE_BAD){
	return DOT_BAD;
    }else if ( avg >= DOT_RESISTANCE_MARGINAL){
	return DOT_MARGINAL;
    }else {
	return DOT_GOOD;
    }
}


/******************************************************************************/
/*!     \fn unsigned char getHeadWearDotStatus( int x )

        \brief
        This function checks the head for bad or marginal dots and returns true
        if a dot bad or marginal dot is found.

        \author
        Aaron Swift
*******************************************************************************/
HeadStatus getHeadWearStatus( int size ){
    bool bad = false;
    bool good = false;
    // run through the dots to see if they are good or bad
    for(int i = 0; i < size; i++){
	if(getHeadWearDotStatus(i) == DOT_GOOD){
	    good = true;
	}else if(getHeadWearDotStatus(i) != DOT_GOOD){
            bad = true;
        }
        // we can stop because we know we have a mixture of dots
        if(good && bad){
            break;
        }
    }
    //The loop is done how did we do 
    if(good && bad)
    {
        return DOTS_MIXED;
    }
    else if(good && !bad)
    {
        return ALL_DOTS_GOOD;
    }
    else
    {
        return ALL_DOTS_BAD;      
    }
}

/******************************************************************************/
/*!     \fn TaskHandle_t getDotWearHandle())

        \brief
        Retruns the task handle for this task

        \author
        Eric Landes
*******************************************************************************/
TaskHandle_t getDotWearHandle(){
    return pHandle_;
}

#if 0 /* remove when finished porting */
/******************************************************************************/
/*!   \fn void sendDotWear( PrDotWear *pDot )

      \brief
        handles converting dot wear message into can frames and 
        adding to flexcan transmit queue for sending. The head dot information
        is sent in two transfers (service scale) of frames 0x175 - 0x191.
        Each transfer contains half of the print head dots.
      \author
          Aaron Swift
*******************************************************************************/                          
void sendDotWear( int size )
{
    flexcan_frame_t  frame_;
    BaseType_t result;
#if 1
    /* first frame include head size and sequence number */
    frame_.id = FLEXCAN_ID_STD( CANID_PR_DOT_WEAR );
    frame_.length = 8;
    frame_.dataByte0 = getHeadWearDotStatus(0);
    frame_.dataByte4 = getHeadWearDot(0);
    frame_.dataByte1 = getHeadWearDotStatus(1);
    frame_.dataByte5 = getHeadWearDot(1);
    frame_.dataByte2 = getHeadWearDotStatus(2);
    frame_.dataByte6 = getHeadWearDot(2);
    frame_.dataByte3 = getHeadWearDotStatus(3);
    frame_.dataByte7 = getHeadWearDot(3);
    frame_.format = kFLEXCAN_FrameFormatStandard;

    result = xQueueSend( getFlexCanTxQueueHandle(), (void *)&frame_, 0 );
    if( result != pdPASS ) {
        PRINTF("sendDotWear(): USB tx message queue is full!\r\n");
    }
    ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
#endif
    int x = 0;
    int i = 4;
    int id_offset = 1;
    /* send the remaining first half of the print head dots */
    for( x = 0; x <= (size / 4); x++) {

        frame_.id = FLEXCAN_ID_STD( CANID_PR_DOT_WEAR + id_offset );
        frame_.length = 8;
        frame_.dataByte0 = getHeadWearDotStatus(i);
        frame_.dataByte4 = getHeadWearDot(i++);
        frame_.dataByte1 = getHeadWearDotStatus(i);
        frame_.dataByte5 = getHeadWearDot(i++);
        frame_.dataByte2 = getHeadWearDotStatus(i);
        frame_.dataByte6 = getHeadWearDot(i++);
        frame_.dataByte3 = getHeadWearDotStatus(i);
        frame_.dataByte7 = getHeadWearDot(i++);
        frame_.format = kFLEXCAN_FrameFormatStandard;

        BaseType_t result = xQueueSend( getFlexCanTxQueueHandle(), (void *)&frame_, 0 );
        if( result != pdPASS ) {
            PRINTF("sendDotWear(): USB tx message queue is full!\r\n");
        }else{
          //PRINTF("sendDotWear(): Sending canID: 0x%x\r\n",CANID_PR_DOT_WEAR + id_offset);
        }
        // remove once time slicing. We need to give the flexcan task some time to empty the message queue
        ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
        if( x % 10){
          taskYIELD();
        }
        if ((CANID_PR_DOT_WEAR + id_offset) == CANID_PR_DOT_WEAR_LAST){
          id_offset = 1;
        }else{
          id_offset++;
        }
    }

}
#endif

/******************************************************************************/
/*!   \fn void assignDotWearMsgQueue( QueueHandle_t pQHandle )                                                           
 
      \brief
        This function sets the printer message queue 
       
      \author
          Aaron Swift
*******************************************************************************/ 
void assignDotWearMsgQueue( QueueHandle_t pQHandle )
{
    if( pQHandle != NULL ) {
        pMsgQHandle_ = pQHandle;
    } else {
        PRINTF("assignDotWearMsgQueue(): Msg queue is NULL!\r\n" );
    }
}


/******************************************************************************/
/*!   \fn calculateDotResistance(int driveVoltage, int counts)                                                           
 
      \brief
        This function calculates the resistance in Ohms for a given adc count
       
      \author
          Aaron Swift
*******************************************************************************/
float calculateDotResistance(float driveVoltage, int counts){
    int delta = counts - ADCBaseline;
    //make sure the delta is greater than 0 to avoid a divide by 0 condition
    if(delta < 1)
        delta = 1;
    return (driveVoltage * ADC_BUFFER_GAIN * SENSE_GAIN * SENSE_RESISTANCE) /
      ((float)(delta)/(ADC_MAX_COUNTS) * (ADC_REFERENCE_VOLTAGE));
}

bool readyToNotify(void){
    //PRINTF("%d\r\n",testPrint_);
    return testPrint_;
}