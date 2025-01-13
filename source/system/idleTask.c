#include "idleTask.h"
#include "fsl_debug_console.h"
#include "sensors.h"
#include "lp5521.h"
#include "takeupMotor.h"
#include "virtualScope.h"
#include "developmentSettings.h"
#include "globalPrinterTask.h"


typedef enum
{
    TEST_INIT,
    TEST_CAL,
    TEST_START,
    TEST_SIZE_AND_PRINT,
    TEST_RESULTS
}TEST_STATE;

typedef enum
{
    TEST_INIT_INIT,
    TEST_INIT_START
}TEST_INIT_STATE;

typedef enum
{
    TEST_CAL_TU_RELAXED_SHOOT_BACKING_ONLY,
    TEST_CAL_TU_TIGHTENED_SHOOT_BACKING_AND_LABEL,
    TEST_CAL_DELTA_CALC
}TEST_CAL_STATE;

typedef enum
{
    TEST_START_INIT,
    TEST_START_START
}TEST_START_STATE;

typedef enum
{
    TEST_SIZE_AND_PRINT_INIT,
    TEST_SIZE_AND_PRINT_START
}TEST_SIZE_AND_PRINT_STATE;

typedef enum
{
    TEST_RESULTS_INIT,
    TEST_RESULTS_START
}TEST_RESULTS_STATE;


static TaskHandle_t     pHandle_                = NULL;
static bool             suspend_                = false;

//state entry flags
static char             testState               = TEST_INIT;
static bool             initTestOnce            = false;
static bool             calTestOnce             = false;
static bool             startTestOnce           = false;
static bool             sizeAndPrintTestOnce    = false;
static bool             resultsTestOnce         = false;

static char             calTestState          = TEST_CAL_TU_RELAXED_SHOOT_BACKING_ONLY;


//TU cal variables
static uint8_t          TUCurrent               = 0;
static uint8_t          TUIndex                 = 0;
static uint16_t         TUVal                   = 0;
static uint16_t         relaxedVals[255]        = { 0 };
static uint16_t         tightenedVals[255]      = { 0 };
static uint16_t         TUdeltaVals[255]        = { 0 };   
static uint16_t         TUDrive                 = 0;

//SHOOT cal variables
static uint8_t          SHOOTCurrent               = 0;
static uint8_t          SHOOTIndex                 = 0;
static uint16_t         SHOOTVal                   = 0;
static uint16_t         backingVals[255]           = { 0 };
static uint16_t         backingAndLabelVals[255]   = { 0 };
static uint16_t         SHOOTdeltaVals[255]        = { 0 };   
static uint16_t         SHOOTDrive                 = 0;

//SENSOR test variables
static bool             TUStuckTestPass            = true;
static bool             TUChangeTestPass           = true;
static bool             TUOutOfRangeTestPass       = true;

static bool             SHOOTChangeTestPass        = true;
static bool             SHOOTOutOfRangeTestPass    = true;

static bool             LOWSTOCKChangeTestPass     = true;
static bool             LOWSTOCKOutOfRangeTestPass = true;

static bool             PHTempTestPass             = true;
static bool             EightyMMPHTestPass         = true;

static bool             allTestsPass               = true;

//LED variables
static bool redLEDOn = false;
static bool greenLEDOn = false;

//external functions
extern void setConfigBackingValue(unsigned short val );
extern void setConfigLabelValue(unsigned short val );
extern void delay_uS ( unsigned int time );

//external variables
extern                  Pr_Config               config_;
extern                  PrStatusInfo            currentStatus;

static LP5521CFG lp5521Cfg;

BaseType_t createIdleTask( void )
{
    BaseType_t result;

    //PRINTF( "idleTask(): Starting...\r\n" );
    
     /* create printer task thread */
    result = xTaskCreate( idleTask,  "IdleTask", configMINIMAL_STACK_SIZE,NULL, idle_task_PRIORITY, &pHandle_ );

    return result; 
}


static void idleTask( void *pvParameters )
{
    //PRINTF( "idleTask(): Thread running...\r\n" );
    
    for (int i = 0; i < 100; ++i) 
    {
        PRINTF("\n");
    }
  
    
    while( !suspend_ ) 
    {
        while( true )
        {
            switch(testState)
            {
                case TEST_INIT:
                {
                    initTest();
                    break;
                }
                case TEST_CAL:
                {
                    calTest();
                    break;
                }
                case TEST_START:
                {
                    startTest();
                    break;
                }
                case TEST_SIZE_AND_PRINT:
                {
                    sizeAndPrintTest();
                    break;
                }
                case TEST_RESULTS:
                {
                    resultsTest();
                    break;
                }
                default:
                {
                    PRINTF("ERROR: undefined test state - testState == %d\r\n", testState);
                    break;
                }
            }
        }
    }
    
    vTaskSuspend(NULL);  
}


void initTest( void )
{
    if( initTestOnce == false )
    {
        initTestOnce = true;
        //PRINTF("initTest() - start\r\n");
        
        //GREEN ON RED OFF
        //GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, false);
        //GPIO_WritePinOutput(STATUS_LED_GPIO, STATUS_LED_PIN, true); 
      
        //GREEN OFF RED OFF
        //GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, false);
        //GPIO_WritePinOutput(STATUS_LED_GPIO, STATUS_LED_PIN, false);
          
        //GREEN OFF RED ON
        greenLEDOn = false;
        redLEDOn = true;
        GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, true);
        GPIO_WritePinOutput(STATUS_LED_GPIO, STATUS_LED_PIN, false);
    }
    else
    {
        char userInput[1];
        userInput[0] = 0;
        
        for (int i = 0; i < 100; ++i) 
        {
            PRINTF("\n");
        }
        
        PRINTF("THE GT PRINTER TEST SHOULD BE STARTED WITH THE BACKING PAPER ONLY OVER THE SHOOT THROUGH SENSOR AND NO PAPER WRAPPED AROUND THE TU HUB OF THE CASSETTE\n");
        PRINTF("\n");
        PRINTF("PRESS THE START TEST BUTTON TO BEGIN THE GT PRINTER TEST\n");
        SCANF("%1hhc", userInput); 
        
        PRINTF("!!!\n");
        
        PRINTF("STARTING TEST...\n");
        
        //next test state
        initTestOnce = false;
        testState = TEST_CAL;
        //PRINTF("initTest() - end - goto test state %d\r\n", testState);
    }
}


void calTest( void )
{       
    if( calTestOnce == false )
    {
        calTestOnce = true;
        
        TUCurrent = 0;
        TUIndex = 0;
        TUVal = 0;
        TUDrive = 0;
        
        SHOOTCurrent = 0;
        SHOOTIndex = 0;
        SHOOTVal = 0;
        SHOOTDrive = 0;
        
        memset( &relaxedVals, 0, sizeof(uint16_t) );
        memset( &tightenedVals, 0, sizeof(uint16_t) );
        memset( &TUdeltaVals, 0, sizeof(uint16_t) );
        
        memset( &backingVals, 0, sizeof(uint16_t) );
        memset( &backingAndLabelVals, 0, sizeof(uint16_t) );
        memset( &SHOOTdeltaVals, 0, sizeof(uint16_t) );
        
        //PRINTF("calTest() - start\r\n");
    }
    else
    {
        switch( calTestState )
        {
            case TEST_CAL_TU_RELAXED_SHOOT_BACKING_ONLY:
            {
                //PRINTF("CAL RELAXED TU AND BACKING PAPER ONLY - START\r\n");
              
                for (int i = 0; i < 100; ++i) 
                {
                    PRINTF("\n");
                }
              
                PRINTF("WORKING...\n");
                
                
              
                //cal relaxed TU motor
                setPaperTakeupCurrent( TUCurrent );
        
                while(TUCurrent < 255)
                {
                    powerOnMotors();
                  
                    TUVal = getTakeUpTorque();
                    relaxedVals[TUIndex] = TUVal;
                    
                    //PRINTF("TUVal1 = %d     current1 = %d\r\n", TUVal, TUCurrent);
                    
                    TUCurrent++;
                    TUIndex++;
                    
                    setPaperTakeupCurrent( TUCurrent ); 
                }
                
                TUCurrent = 0;
                TUIndex = 0;
                
                setPaperTakeupCurrent( TUCurrent );
               
                //cal shoot backing only here
                setGapCurrent( SHOOTCurrent );
        
                while(SHOOTCurrent < 255)
                {
                    powerOnMotors();
                  
                    SHOOTVal = getShootAverage();
                    backingVals[SHOOTIndex] = SHOOTVal;
                    
                    //PRINTF("SHOOTVal1 = %d     current1 = %d\r\n", SHOOTVal, SHOOTCurrent);
                    
                    SHOOTCurrent++;
                    SHOOTIndex++;
                    
                    setGapCurrent( SHOOTCurrent ); 
                }
                
                SHOOTCurrent = 0;
                SHOOTIndex = 0;
                
                setGapCurrent( SHOOTCurrent );
                
                
                
                
                //PRINTF("CAL RELAXED TU AND BACKING PAPER ONLY - END\r\n");
                
                calTestState = TEST_CAL_TU_TIGHTENED_SHOOT_BACKING_AND_LABEL;
                
                break;
                
            }
            case TEST_CAL_TU_TIGHTENED_SHOOT_BACKING_AND_LABEL:
            { 
                //PRINTF("CAL TIGHTENED TU - START\r\n");
                
                for (int i = 0; i < 100; ++i) 
                {
                    PRINTF("\n");
                }
                
                
                PRINTF("OPEN THE CASSETTE\n");
                
                waitForCassetteOpen();
                
                for (int i = 0; i < 100; ++i) 
                {
                    PRINTF("\n");
                }
                
                //RED ON
                greenLEDOn = false;
                redLEDOn = true;
                GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, true);
                GPIO_WritePinOutput(STATUS_LED_GPIO, STATUS_LED_PIN, false);
              
                
                //PRINTF("CAL TIGHTENED TU - RUNNING\r\n");
         
                PRINTF("WHEN THE START TEST BUTTON IS PRESSED A COUNTDOWN WILL BEGIN FROM 10, DURING THIS COUNTDOWN TURN THE TU MOTOR ARM CLOCKWISE UNTIL TU CLUTCH SPRING IS FULLY EXTENDED AND HOLD THE TU ARM IN THIS POSITION UNTIL INSTRUCTED TO STOP\n");
                PRINTF("\n");
                
                char userInput[1];
                userInput[0] = 0;
                PRINTF("PRESS THE START TEST BUTTON TO CONTINUE THE GT PRINTER TEST\n");
                SCANF("%1hhc", userInput); 
                
                for (int i = 10; i > 0; --i) 
                {
                    powerOnMotors();
                    takeupDelay();
                    PRINTF("COUNTDOWN %d\n", i);
                }
                
                
                //cal tightened TU motor here
                while(TUCurrent < 255)
                {
                    powerOnMotors();
                  
                    TUVal = getTakeUpTorque();
                    tightenedVals[TUIndex] = TUVal;
                  
                    //PRINTF("TUVal2 = %d     current2 = %d\r\n", TUVal, TUCurrent);
                    
                    TUCurrent++;
                    TUIndex++;
                    
                    setPaperTakeupCurrent( TUCurrent ); 
                }
                
                TUCurrent = 0;
                TUIndex = 0;

                setPaperTakeupCurrent( TUCurrent ); 
                
                //PRINTF("CAL TIGHTENED TU - END\r\n");
                
                //PRINTF("CAL BACKING AND LABEL SHOOT - START\r\n");
                
                for (int i = 0; i < 100; ++i) 
                {
                    PRINTF("\r\n");
                }
                PRINTF("YOU MAY RELEASE THE TU ARM NOW\n");
                PRINTF("\n");
                PRINTF("POSITION THE LABELS IN THE CASSETTE WITH THE BACKING PAPER AND LABEL OVER THE SHOOT THROUGH SENSOR AND BACKING PAPER STREAMING FROM FRONT OF PRINTER\n");
                PRINTF("\n");
                PRINTF("RE-INSERT CASSETTE\n");
                
                waitForCassetteClose();
                
                for (int i = 0; i < 100; ++i) 
                {
                    PRINTF("\n");
                }
                
                PRINTF("WORKING...\n");
                
                //RED ON
                greenLEDOn = false;
                redLEDOn = true;
                GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, true);
                GPIO_WritePinOutput(STATUS_LED_GPIO, STATUS_LED_PIN, false);
                
                //PRINTF("CAL BACKING AND LABEL SHOOT - RUNNING\r\n");
                
                //cal shoot backing and label here
                setGapCurrent( SHOOTCurrent );
        
                while(SHOOTCurrent < 255)
                {
                    powerOnMotors();
                  
                    SHOOTVal = getShootAverage();
                    backingAndLabelVals[SHOOTIndex] = SHOOTVal;
                    
                    //PRINTF("SHOOTVal2 = %d     current2 = %d\r\n", SHOOTVal, SHOOTCurrent);
                    
                    SHOOTCurrent++;
                    SHOOTIndex++;
                    
                    setGapCurrent( SHOOTCurrent ); 
                }
                
                SHOOTCurrent = 0;
                SHOOTIndex = 0;
                
                setGapCurrent( SHOOTCurrent );
                
                //PRINTF("CAL BACKING AND LABEL SHOOT - END\r\n");
                
                calTestState = TEST_CAL_DELTA_CALC;
                
                break;
            }
            case TEST_CAL_DELTA_CALC:
            {      
                int size = sizeof(relaxedVals) / sizeof(relaxedVals[0]);
            
                for (int i = 0; i < size; ++i) 
                {
                    TUdeltaVals[i] = tightenedVals[i] - relaxedVals[i];
                }
                
                for (int i = 0; i < size; ++i) 
                {
                    SHOOTdeltaVals[i] = backingAndLabelVals[i] - backingVals[i];
                }
                
                
                for(int i = 0; i < 255; i++)
                {
                    //PRINTF("TUdeltaVals = %d \r\n", TUdeltaVals[i], i);
                    
                    for(int i = 0; i < 1000; i++)
                    {
                        __NOP();
                        for(int i2 = 0; i2 < 1000; i2++)
                        {
                            __NOP();
                        }
                    }
                }
                
                for(int i = 0; i < 255; i++)
                {
                    //PRINTF("SHOOT deltaVals = %d \r\n", SHOOTdeltaVals[i], i);
                    
                    for(int i = 0; i < 1000; i++)
                    {
                        __NOP();
                        for(int i2 = 0; i2 < 1000; i2++)
                        {
                            __NOP();
                        }
                    }
                }
                
                for(int i = 0; i < 9; i++)
                {   
                    SHOOTdeltaVals[i] = 0;
                    TUdeltaVals[i] = 0;
                }

                uint16_t largestValue = 0;
                uint16_t largestValueIndex = 0;

                for (int i = 10; i < size; ++i) 
                {
                    if (TUdeltaVals[i] > largestValue) 
                    {
                        largestValue = TUdeltaVals[i];
                        largestValueIndex = i;
                    }
                }
              
                /*
                PRINTF("TU largest deflection detected at current level %d\r\n", largestValueIndex);
                PRINTF("TU deflection value = %d\r\n", largestValue);
                PRINTF("TU arm resting torque value = %d\r\n", relaxedVals[largestValueIndex]);
                PRINTF("TU arm tightened torque value = %d\r\n", tightenedVals[largestValueIndex]);
                */
                               
                unsigned short max = (int)(MAX_TENSION_MULTIPLIER * (float)tightenedVals[largestValueIndex]);
                unsigned short min = (int)(MIN_TENSION_MULTIPLIER * (float)tightenedVals[largestValueIndex]);

                //set our calibrated torque sensor current
                setPaperTakeupCurrent( largestValueIndex );

                setConfigTakeupSensorValue( largestValueIndex,  max, min);
                
                
                largestValue = 0;
                largestValueIndex = 0;

                for (int i = 10; i < size; ++i) 
                {
                    if (SHOOTdeltaVals[i] > largestValue) 
                    {
                        largestValue = SHOOTdeltaVals[i];
                        largestValueIndex = i;
                    }
                }
                
                /*
                PRINTF("SHOOT largest deflection detected at current level %d\r\n", largestValueIndex);
                PRINTF("SHOOT deflection value = %d\r\n", largestValue);
                PRINTF("SHOOT value = %d\r\n", backingVals[largestValueIndex]);
                PRINTF("SHOOT value = %d\r\n", backingAndLabelVals[largestValueIndex]);
                */
                
                setConfigBackingValue( backingVals[largestValueIndex] );
                setConfigLabelValue( backingAndLabelVals[largestValueIndex] );
                setGapCurrent(largestValueIndex);

                calTestOnce = false;
                calTestState = TEST_CAL_TU_RELAXED_SHOOT_BACKING_ONLY;
                testState = TEST_START;
                //PRINTF("calTest() - end - goto test state %d\r\n", testState);
                break;
            }
        }     
    }
}


void startTest( void )
{
    if( startTestOnce == false )
    {
        TUStuckTestPass            = true;
        TUChangeTestPass           = true;
        TUOutOfRangeTestPass       = true;

        SHOOTChangeTestPass        = true;
        SHOOTOutOfRangeTestPass    = true;

        LOWSTOCKChangeTestPass     = true;
        LOWSTOCKOutOfRangeTestPass = true;

        PHTempTestPass             = true;
        EightyMMPHTestPass         = true;

        allTestsPass               = true;
        
        startTestOnce = true;
        //PRINTF("startTest() - start\r\n");
    }
    else
    {
        int TUTestVal = getTakeUpTorque();
        int SHOOTTestVal = getShootAverage();
        int LOWSTOCKTestVal = getLowStockSensor();
        int PRINTHEADTemperatureVal = getPrintheadTemperatureInCelsius();
        int PRINTHEADTypeVal = getPrintHeadType(); //80mm == 8
        
        PRINTF("\r\n");
        PRINTF("TUTestVal - %d\n", TUTestVal);
        PRINTF("SHOOTTestVal - %d\n", SHOOTTestVal);
        PRINTF("LOWSTOCKTestVal - %d\n", LOWSTOCKTestVal);
        PRINTF("PRINTHEADTemperatureVal - %d\n", PRINTHEADTemperatureVal);
        PRINTF("PRINTHEADTypeVal - %d\n", PRINTHEADTypeVal);
        
        
        PrintEngine *pEngineTest = getPrintEngine();
        pEngineTest->linePrintDone = false;
        
        
        //stuck TU hub test
        //PRINTF("STUCK TU HUB TEST - START\n");

        checkForPaper(TUTestVal + 200, 820);
        
        if(getTakingUpPaper() == true)
        {
            PRINTF("STUCK TU HUB TEST TEST FAIL\n");
            
            setTakingUpPaper(false);
            
            TUStuckTestPass = false;
            allTestsPass = false;
        }
        else
        {
            PRINTF("STUCK TU HUB TEST TEST PASS\n");
            TUStuckTestPass = true;
        }
        
        //PRINTF("STUCK TU HUB TEST - END\n");
        
        for (int i = 0; i < 100; ++i) 
        {
            PRINTF("\n");
        }
        
        PRINTF("OPEN CASSETTE\n");
        
        waitForCassetteOpen();
        
        for (int i = 0; i < 100; ++i) 
        {
            PRINTF("\n");
        }
        
        PRINTF("THREAD LABELS AROUND TU HUB OF CASSETTE, THEN RE-INSERT CASSETTE\n");
        
        waitForCassetteClose();
        
        for (int i = 0; i < 100; ++i) 
        {
            PRINTF("\n");
        }
        
        PRINTF("WORKING...\n");
        
        //RED ON
        greenLEDOn = false;
        redLEDOn = true;
        GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, true);
        GPIO_WritePinOutput(STATUS_LED_GPIO, STATUS_LED_PIN, false);
        
        
        //sensors out of range test
        if(TUTestVal < 50 || TUTestVal > 4045)
        {
            PRINTF("TU out of range TEST FAIL - VALUE = %d\n", TUTestVal);
            TUOutOfRangeTestPass = false;
            allTestsPass = false;
        }
        
        if(SHOOTTestVal < 50 || SHOOTTestVal > 4045)
        {
            PRINTF("SHOOT out of range TEST FAIL - VALUE = %d\n", SHOOTTestVal);
            SHOOTOutOfRangeTestPass = false;
            allTestsPass = false;
        }
        
        if(LOWSTOCKTestVal < 50 || LOWSTOCKTestVal > 4045)
        {
            PRINTF("LOW STOCK out of range TEST FAIL - VALUE = %d\n", LOWSTOCKTestVal);
            LOWSTOCKOutOfRangeTestPass = false;
            allTestsPass = false;
        }
        
        
        //80mm vs 72mm printhead test
        if(PRINTHEADTypeVal != 8)
        {
            PRINTF("PRINTHEAD TYPE TEST FAIL\n");
            EightyMMPHTestPass = false;
            allTestsPass = false;
        }
        else
        {
            PRINTF("PRINTHEAD TYPE TEST PASS\n");
        }
        

        //printhead temperature test
        if(PRINTHEADTemperatureVal < 0 || PRINTHEADTemperatureVal > 100)
        {
            PRINTF("PRINTHEAD TEMPERATURE TEST FAIL\n");
            PHTempTestPass = false;
            allTestsPass = false;  
        }
        else
        {
            PRINTF("PRINTHEAD TEMPERATURE TEST PASS\n");
        }
        
 
        //next test state
        startTestOnce = false;
        testState = TEST_SIZE_AND_PRINT;
        //PRINTF("startTest() - end - goto test state %d\r\n", testState);
    }
}


void sizeAndPrintTest( void )
{
    if( sizeAndPrintTestOnce == false )
    {
        sizeAndPrintTestOnce = true;
        //PRINTF("sizeAndPrintTest() - start\r\n");
        PRINTF("SIZING START\n");
    }
    else
    {
        loadZeroPrintLine();
              
        //SIZE LABEL
        uint16_t stepsToLt = 2999;
        uint16_t stepsToSize = 10000;
        
        takeupDelay();
        takeupDelay();
        takeupDelay();
        
        if(TUStuckTestPass == true)
        {
            checkForPaper(((float)config_.takeup_sensor_max_tension_counts * 0.90), 820);
        }
        
        setHalfStepMode(_MAIN_STEPPER);
        setHalfStepMode(_TAKEUP_STEPPER); 
        
        
        
        if(getTakingUpPaper() == true && TUStuckTestPass == true)
        {
            tightenStock(((float)config_.takeup_sensor_min_tension_counts * 1.0), 920, false, HALF_STEP);
        }

        //takeupDelay();

        //stepToLt(stepsToLt, 575);

        sizeLabels(stepsToSize, 575);
        
        //takeupDelay();
        
        #define GAP_SENSOR_TO_PEEL_BAR_GT 900
        #define GAP_SENSOR_TO_PEEL_BAR_HT 840
        #define GAP_SENSOR_TO_TEAR_BAR_GT 700
        #define GAP_SENSOR_TO_TEAR_BAR_HT 940
        
        if(getLargeGapFlag() == true)
        {
            if(getTakingUpPaper() == true)
            {
                //PRINTF("stepsBackToGap - %d\r\n", getStepsBackToGap());
                //PRINTF("firstGapIndex - %d\r\n", getFirstGapIndex());
                //PRINTF("secondGapIndex - %d\r\n", getSecondGapIndex());
                //PRINTF("labelSizeInQuarterSteps - %d\r\n", getLabelSizeInQuarterSteps());
                stepToNextLabel(GAP_SENSOR_TO_PEEL_BAR_HT - getStepsBackToGap(), 575);
            }
            else
            {
                //PRINTF("stepsBackToGap - %d\r\n", getStepsBackToGap());
                //PRINTF("firstGapIndex - %d\r\n", getFirstGapIndex());
                //PRINTF("secondGapIndex - %d\r\n", getSecondGapIndex());
                //PRINTF("labelSizeInQuarterSteps - %d\r\n", getLabelSizeInQuarterSteps());
                stepToNextLabel(GAP_SENSOR_TO_TEAR_BAR_HT - getStepsBackToGap(), 575);
            }
        }
        else
        {
            if(getTakingUpPaper() == true)
            {
                //PRINTF("stepsBackToGap - %d\r\n", getStepsBackToGap());
                //PRINTF("firstGapIndex - %d\r\n", getFirstGapIndex());
                //PRINTF("secondGapIndex - %d\r\n", getSecondGapIndex());
                //PRINTF("labelSizeInQuarterSteps - %d\r\n", getLabelSizeInQuarterSteps());
                stepToNextLabel(GAP_SENSOR_TO_PEEL_BAR_GT - getStepsBackToGap(), 575);
            }
            else
            {
                //PRINTF("stepsBackToGap - %d\r\n", getStepsBackToGap());
                //PRINTF("firstGapIndex - %d\r\n", getFirstGapIndex());
                //PRINTF("secondGapIndex - %d\r\n", getSecondGapIndex());
                //PRINTF("labelSizeInQuarterSteps - %d\r\n", getLabelSizeInQuarterSteps());
                stepToNextLabel(GAP_SENSOR_TO_TEAR_BAR_GT - getStepsBackToGap(), 575);
            }
        }

        /*
        while(getLabelTaken() >= 150)
        {
            //PRINTF("WAITING FOR LT\r\n");
        }
        */
        
        takeupDelay();
        
        if(getTakingUpPaper() == true && TUStuckTestPass == true)
        {
            loosenStock(300, 1700);
            backwindStock(calculateSizingBackwindSteps(), 1500);
        } 
        
        //else
        //{
        //    backwindStock(calculateSizingBackwindSteps(), 1300);
        //}
        
        PRINTF("SIZING END\n");
        
        takeupDelay();
      
        PRINTF("PRINT START\n");
        
        //PRINT LABEL
        createCheckerBoardLabel( 0, STEPS_PER_LENGTH3_00 );
        
        setIndirectData( RAM_0, STEPS_PER_LENGTH3_00 );                  
        
        /* intiate generic printing command */
        initializeCmdSequence( 4, &currentStatus );
      
        //next test state
        sizeAndPrintTestOnce = false;
        testState = TEST_RESULTS;
        //PRINTF("sizeAndPrintTest() - end - goto test state %d\r\n", testState);
        
        PRINTF("PRINT END\n");
    }
}


void resultsTest( void )
{
    PrintEngine *pEngineTest = getPrintEngine();
    bool printDone = pEngineTest->linePrintDone;
  
    if( resultsTestOnce == false )
    {
        resultsTestOnce = true;
        
        //PRINTF("resultsTest() - start\r\n");
    }
    else if(resultsTestOnce == true && printDone == true)
    {   
        //next test state
        resultsTestOnce = false;
        testState = TEST_INIT;
        //PRINTF("resultsTest() - end - goto test state %d\r\n", testState);
        
        
        if(TUChangeTestPass == false || SHOOTChangeTestPass == false || LOWSTOCKChangeTestPass == false )
        {
            PRINTF("CHANGE TEST FAILED\n");
            allTestsPass = false;
        }
        
        
        for (int i = 0; i < 100; ++i) 
        {
            PRINTF("\n");
        }
        
        if(allTestsPass == true)
        {
            PRINTF("ALL TESTS PASSED\n");
          
            //GREEN ON
            GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, false);
            GPIO_WritePinOutput(STATUS_LED_GPIO, STATUS_LED_PIN, true);
        }
        else
        {
            PRINTF("A TEST HAS FAILED\n");
          
            //RED ON GREEN OFF
            greenLEDOn = false;
            redLEDOn = true;
            GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, true);
            GPIO_WritePinOutput(STATUS_LED_GPIO, STATUS_LED_PIN, false);
            
            
            
            
            
            
            
            //SENSOR test variables
            //static bool             TUStuckTestPass            = true;
            //static bool             TUChangeTestPass           = true;
            //static bool             TUOutOfRangeTestPass       = true;

            //static bool             SHOOTChangeTestPass        = true;
            //static bool             SHOOTOutOfRangeTestPass    = true;

            //static bool             LOWSTOCKChangeTestPass     = true;
            //static bool             LOWSTOCKOutOfRangeTestPass = true;

            //static bool             PHTempTestPass             = true;
            //static bool             EightyMMPHTestPass         = true;
            
            
            
            
            
            if( TUStuckTestPass == false || TUChangeTestPass == false || TUOutOfRangeTestPass == false )
            {
                PRINTF("TU SENSOR FAILED A TEST\n");
              
                for(int i2 = 0; i2 < 4; i2++)
                {
                      for(int i = 0; i < 2; i++)
                      {
                          GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, true); //turn LED off
                          //vTaskDelay( pdMS_TO_TICKS( 500 ) );
                          GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, false); //turn LED on
                          //vTaskDelay( pdMS_TO_TICKS( 500 ) );
                      }
                      
                      //vTaskDelay( pdMS_TO_TICKS( 2000 ) );
                }
            }
            else if( SHOOTChangeTestPass == false || SHOOTOutOfRangeTestPass == false )
            {
                PRINTF("SHOOT SENSOR FAILED A TEST\n");
              
                for(int i2 = 0; i2 < 4; i2++)
                {
                      for(int i = 0; i < 3; i++)
                      {
                          GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, true); //turn LED off
                          //vTaskDelay( pdMS_TO_TICKS( 500 ) );
                          GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, false); //turn LED on
                          //vTaskDelay( pdMS_TO_TICKS( 500 ) );
                      }
                      
                      //vTaskDelay( pdMS_TO_TICKS( 2000 ) );
                }
            }
            else if( LOWSTOCKChangeTestPass == false || LOWSTOCKOutOfRangeTestPass == false )
            {
                PRINTF("LOWSTOCK SENSOR FAILED A TEST\n");
              
                for(int i2 = 0; i2 < 4; i2++)
                {
                      for(int i = 0; i < 4; i++)
                      {
                          GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, true); //turn LED off
                          //vTaskDelay( pdMS_TO_TICKS( 500 ) );
                          GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, false); //turn LED on
                          //vTaskDelay( pdMS_TO_TICKS( 500 ) );
                      }
                      
                      //vTaskDelay( pdMS_TO_TICKS( 2000 ) );
                }
            }
            else if( PHTempTestPass == false )
            {
                PRINTF("PRINTHEAD THERMISTOR TEST FAILED\n");
              
                for(int i2 = 0; i2 < 4; i2++)
                {
                      for(int i = 0; i < 5; i++)
                      {
                          GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, true); //turn LED off
                          //vTaskDelay( pdMS_TO_TICKS( 500 ) );
                          GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, false); //turn LED on
                          //vTaskDelay( pdMS_TO_TICKS( 500 ) );
                      }
                      
                      //vTaskDelay( pdMS_TO_TICKS( 2000 ) );
                }
            }
            else if( EightyMMPHTestPass == false )
            {
                PRINTF("PRINTHEAD TYPE DETECTION TEST FAILED\n");
              
                for(int i2 = 0; i2 < 4; i2++)
                {
                      for(int i = 0; i < 6; i++)
                      {
                          GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, true); //turn LED off
                          //vTaskDelay( pdMS_TO_TICKS( 500 ) );
                          GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, false); //turn LED on
                          //vTaskDelay( pdMS_TO_TICKS( 500 ) );
                      }
                      
                      //vTaskDelay( pdMS_TO_TICKS( 2000 ) );
                }
            }
            

            //RED ON GREEN OFF
            greenLEDOn = false;
            redLEDOn = true;
            GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, true);
            GPIO_WritePinOutput(STATUS_LED_GPIO, STATUS_LED_PIN, false);
            
           
        }
        
        
        //PRINTF("\n");
        //PRINTF("OPEN CASSETTE AND THREAD LABELS WITH ONLY BACKING PAPER OVER THE SHOOT THROUGH SENSOR TO START NEXT TEST\n");
        
        
        //restart test
        //while( readHeadUpSensor( &currentStatus ) == false )
        //{              
            //PRINTF("?");
            //__NOP();
        //}
       
        //waitForCassetteClose();
        
        loadZeroPrintLine();
    }
    else
    {
        //PRINTF("WAITING FOR printDone\r\n");
        __NOP();
    }
}


void waitForCassetteOpen( void )
{
    while( readHeadUpSensor( &currentStatus ) == false )
    {              
        cycleRedLED();
    }
}


void waitForCassetteClose( void )
{
    while( readHeadUpSensor( &currentStatus ) == true )
    {              
       cycleRedLED();
    }
}


void cycleRedLED( void )
{
    takeupDelay();
  
    greenLEDOn = false;
    GPIO_WritePinOutput(STATUS_LED_GPIO, STATUS_LED_PIN, false);
    
    if(redLEDOn == false)
    {
        redLEDOn = true;
        GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, true);
    }
    else
    {
        redLEDOn = false;
        GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, false);
    } 
}


void cycleGreenLED( void )
{
    takeupDelay();
    
    redLEDOn = false;
    GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, false);
    
    if(greenLEDOn == false)
    {
        greenLEDOn = true;
        GPIO_WritePinOutput(STATUS_LED_GPIO, STATUS_LED_PIN, true);
    }
    else
    {
        greenLEDOn = false;
        GPIO_WritePinOutput(STATUS_LED_GPIO, STATUS_LED_PIN, false);
    }
}


void setTUStuckTestPass( bool testStatus )
{
    TUStuckTestPass = testStatus;
}


bool getTUStuckTestPass( void )
{
    return TUStuckTestPass;
}


void setTUChangeTestPass( bool testStatus )
{
    TUChangeTestPass = testStatus;
}


void setSHOOTChangeTestPass( bool testStatus )
{
    SHOOTChangeTestPass = testStatus;
}


void setLOWSTOCKChangeTestPass( bool testStatus )
{
    LOWSTOCKChangeTestPass = testStatus;
}