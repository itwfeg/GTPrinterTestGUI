#include "TPHMotor.h"

uint8_t (*GPT2Intr) (void); 

AT_NONCACHEABLE_SECTION_INIT(static uint16_t TPHMotorStepCount) = 0;
AT_NONCACHEABLE_SECTION_INIT(static uint16_t TUMotorStepCount) = 0;

AT_NONCACHEABLE_SECTION_INIT(static uint16_t stepsToTake) = 0;

AT_NONCACHEABLE_SECTION_INIT(static uint16_t TUVal) = 0;
AT_NONCACHEABLE_SECTION_INIT(static uint16_t TUSpeed) = 0;
AT_NONCACHEABLE_SECTION_INIT(static uint16_t TPHSpeed) = 0;

AT_NONCACHEABLE_SECTION_INIT(static unsigned short maxTension) = 0;
AT_NONCACHEABLE_SECTION_INIT(static unsigned short minTension) = 0;

AT_NONCACHEABLE_SECTION_INIT(static uint16_t rampSpeed) = 1800;
AT_NONCACHEABLE_SECTION_INIT(static uint16_t rampTarget) = 1000;
//static bool rampFlag = false;
AT_NONCACHEABLE_SECTION_INIT(static char rampFlag) = 0;

AT_NONCACHEABLE_SECTION_INIT(bool startTUOnce_) = false;
AT_NONCACHEABLE_SECTION_INIT(bool TUFlopFlag) = false;

static uint16_t tensionModifier = 0;

AT_NONCACHEABLE_SECTION_INIT(static uint16_t lastTUStepsDuringSize) = 0;

AT_NONCACHEABLE_SECTION_INIT(static bool measureOnce_) = false;
static bool firstTakeupMeasure_ = false;

extern Pr_Config config_;
extern PrStatusInfo currentStatus;
extern PrStatusInfo prevStatus;

#define TUHISTORYSIZE_GPT2 16
#define TU_SLOPE_GPT2 15
#define MAX_TU_STEP_PERIOD_CHANGE_GPT2 60

static uint16_t TUValHistory_GPT2[TUHISTORYSIZE_GPT2] = {0};
static int16_t TUTensionSlope_GPT2 = 0;
static unsigned char TUHistoryIndex_GPT2 = 0;
static unsigned char newestReading_GPT2 = 0;
static unsigned char oldestReading_GPT2 = 0;
static unsigned char numValidReadings_GPT2 = 0;


void MotorIntrGPT2( void )
{  
    uint8_t intr = GPT2Intr();
    
    
    #if defined __CORTEX_M && (__CORTEX_M == 4U || __CORTEX_M == 7U)
        __DSB();
    #endif
    /* added for arm errata 838869 */
    SDK_ISR_EXIT_BARRIER;        
}

uint8_t stepTPHMotorIntrGPT2( void )
{
    GPT_ClearStatusFlags( GPT2, kGPT_OutputCompare1Flag );
  
    if(TPHMotorStepCount >= stepsToTake)
    {
        GPT_StopTimer( GPT2 );
        
        setTakeupBusy(false);
        
        stopTPHMotorIntrGPT2();
        stopTakeupIntr();
        stepsToTake = 0;
    }
    else
    {      
        stepMainMotor();
          
        TPHMotorStepCount++;
    }
     
    return 0;
}

uint8_t stepTUMotorIntrGPT2( void )
{
    GPT_ClearStatusFlags( GPT2, kGPT_OutputCompare1Flag );
    
    int16_t TUSpeedChange = 0;
    
    TUVal = getTakeUpTorque();
    TUValHistory_GPT2[TUHistoryIndex_GPT2] = TUVal;
    
    newestReading_GPT2 = TUHistoryIndex_GPT2;
    TUHistoryIndex_GPT2++;
    if(TUHistoryIndex_GPT2 >= TUHISTORYSIZE_GPT2)
      TUHistoryIndex_GPT2 = 0;
    if(numValidReadings_GPT2 >= TUHISTORYSIZE_GPT2-1)
      oldestReading_GPT2 = TUHistoryIndex_GPT2;
    else
    {
      oldestReading_GPT2 = 0;
      numValidReadings_GPT2++;
    }
    
     TUTensionSlope_GPT2 = (int16_t)(TUValHistory_GPT2[newestReading_GPT2] - TUValHistory_GPT2[oldestReading_GPT2]);
    
    /* Differential Compensation */
    if(TUTensionSlope_GPT2 > 10*TU_SLOPE_GPT2)
       TUSpeedChange = 40;
    if(TUTensionSlope_GPT2 > 8*TU_SLOPE_GPT2)
       TUSpeedChange = 30;
    else if(TUTensionSlope_GPT2 > 4*TU_SLOPE_GPT2)
       TUSpeedChange = 15;
    else if(TUTensionSlope_GPT2 > 2*TU_SLOPE_GPT2)
       TUSpeedChange = 5;
    
    if (TUTensionSlope_GPT2 < -10*TU_SLOPE_GPT2)
       TUSpeedChange = -60;
    if (TUTensionSlope_GPT2 < -8*TU_SLOPE_GPT2)
       TUSpeedChange = -40;
    else if (TUTensionSlope_GPT2 < -4*TU_SLOPE_GPT2)
       TUSpeedChange = -20;
    else if (TUTensionSlope_GPT2 < -2*TU_SLOPE_GPT2)
       TUSpeedChange = -5;
    
    
    if(TUVal > (maxTension + 40)) 
    {
         if(TUVal > ((float)maxTension * 1.15))
         {
            TUSpeedChange += 15;
         }
         else if(TUVal > ((float)maxTension * 1.1))
         {
            TUSpeedChange += 10;
         }
         else if(TUVal > ((float)maxTension * 1.05))
         {
            TUSpeedChange += 6;
         }
         else
         {
            TUSpeedChange += 3;
         }
    }
    else if(TUVal < (maxTension)) 
    {
         if(TUVal < ((float)minTension * 0.94))
         {
            TUSpeedChange += -30;
         }
         else if(TUVal < ((float)minTension * 0.96))
         {
            TUSpeedChange += -25;
         }
         else if(TUVal < ((float)minTension * 0.98))
         {
            TUSpeedChange += -20;
         }
         else
         {
            TUSpeedChange += -15;
         }
    }
    
    if(TUSpeedChange > MAX_TU_STEP_PERIOD_CHANGE_GPT2)
       TUSpeedChange = MAX_TU_STEP_PERIOD_CHANGE_GPT2;
    else if (TUSpeedChange < -MAX_TU_STEP_PERIOD_CHANGE_GPT2)
       TUSpeedChange = -MAX_TU_STEP_PERIOD_CHANGE_GPT2;
    
    TUSpeed = (uint16_t)((int16_t)TUSpeed + TUSpeedChange);

    
    if(TUSpeed > 4000)
    {
       TUSpeed = 4000;
    }
    else if(TUSpeed < 1000)
    {
       TUSpeed = 1000;
    }
   
    /*
    if(getReadyToRecordTakeupSteps() == true && measureOnce_ == true)
    {
        measureOnce_ = false;
        
        //PRINTF("%d,", TUMotorStepCount);        
        setTensionModifier(TUMotorStepCount);
    }
    */
    
    
    
    if(getReadyToRecordTakeupSteps() == true && measureOnce_ == true)
    {
        measureOnce_ = false;
        
        //PRINTF("TUStep sample during sizing: %d\r\n", TUMotorStepCount);
        //PRINTF("Percentage of roll: %d\r\n", getPercentageOfRollEstimate(TUMotorStepCount));
        
        if(TUMotorStepCount >= 490)
        {
            //PRINTF("setting tensionModifier to 0\r\n");
            setTensionModifier(0);
        }
        else if(TUMotorStepCount >= 460 && TUMotorStepCount < 490)
        {
            //PRINTF("setting tensionModifier to 25\r\n");
            setTensionModifier(25);
        }
        else if(TUMotorStepCount >= 440 && TUMotorStepCount < 460)
        {
            //PRINTF("setting tensionModifier to 50\r\n");
            setTensionModifier(50);
        }
        else if(TUMotorStepCount >= 400 && TUMotorStepCount < 440)
        {
            //PRINTF("setting tensionModifier to 75\r\n");
            setTensionModifier(75);
        }
        else if(TUMotorStepCount >= 375 && TUMotorStepCount < 400)
        {
            //PRINTF("setting tensionModifier to 100\r\n");
            setTensionModifier(100);
        }
        else if(TUMotorStepCount >= 355 && TUMotorStepCount < 375)
        {
            //PRINTF("setting tensionModifier to 125\r\n");
            setTensionModifier(140);
        }
        else if(TUMotorStepCount >= 345 && TUMotorStepCount < 355)
        {
            //PRINTF("setting tensionModifier to 150\r\n");
            setTensionModifier(180);
        }
        else if(TUMotorStepCount >= 335 && TUMotorStepCount < 345)
        {
            //PRINTF("setting tensionModifier to 175\r\n");
            setTensionModifier(200);
        }
        else if(TUMotorStepCount >= 325 && TUMotorStepCount < 335)
        {
            //PRINTF("setting tensionModifier to 200\r\n");
            setTensionModifier(220);
        }
        else if(TUMotorStepCount >= 315 && TUMotorStepCount < 325)
        {
            //PRINTF("setting tensionModifier to 225\r\n");
            setTensionModifier(250);
        }
        else if(TUMotorStepCount >= 305 && TUMotorStepCount < 315)
        {
            //PRINTF("setting tensionModifier to 250\r\n");
            setTensionModifier(280);
        }
        else if(TUMotorStepCount >= 295 && TUMotorStepCount < 305)
        {
            //PRINTF("setting tensionModifier to 275\r\n");
            setTensionModifier(310);
        }
        else if(TUMotorStepCount >= 285 && TUMotorStepCount < 295)
        {
            //PRINTF("setting tensionModifier to 300\r\n");
            setTensionModifier(340);
        }
        else if(TUMotorStepCount >= 275 && TUMotorStepCount < 285)
        {
            //PRINTF("setting tensionModifier to 325\r\n");
            setTensionModifier(370);
        }
        else if(TUMotorStepCount < 275)
        {
            //PRINTF("setting tensionModifier to 340\r\n");
            setTensionModifier(400);
        }
    }
   
    if(TUMotorStepCount >= stepsToTake)
    {
        GPT_StopTimer( GPT2 );

        stopTPHMotorIntrGPT2();
        
        setLastSpeed(TUSpeed);  
        
        TUMotorStepCount = 0;
        TUSpeed = 0;
        
        stepsToTake = 0;
    }
    else
    {
        stepTakeUpMotor();
          
        TUMotorStepCount++;
        
        GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel1, TUSpeed ); 
    }
    
  
    return 0;
}

uint8_t lineTimerIntr( void )
{       
    static bool pwmStartTime_ = false, pwmSltTime_ = false;
    PrintEngine *engine  = getPrintEngine();

    /* latch hist/adj load and burn, start pwm to hold line temperature */
    if( ( GPT_GetStatusFlags( LINE_PRINTER_TIMER_BASE, kGPT_OutputCompare1Flag ) & kGPT_OutputCompare1Flag ) == kGPT_OutputCompare1Flag ) 
    {
        GPT_ClearStatusFlags( LINE_PRINTER_TIMER_BASE, kGPT_OutputCompare1Flag );		
        /* setup for pwm start time after history isr */
        if( !pwmStartTime_ ) 
        {       
            /* latch control during history */ 
            if( !isCurrentLine() ) 
            {    
                /* latch was set Low in LPSPI_MasterUserCallback(); lets set it back to High now.
                    this should give us around ~150Us pulse width. */ 
                GPIO_WritePinOutput( PHEAD_LATCH_GPIO, PHEAD_LATCH_PIN, true ); //true
            }

            /* enable the strobe*/            

            GPIO_WritePinOutput( PHEAD_STROBE_EN_GPIO, PHEAD_STROBE_EN_PIN, false );  
            
            GPIO_WritePinOutput(PHEAD_STROBE_A_GPIO, PHEAD_STROBE_A_PIN, true);  
            GPIO_WritePinOutput(PHEAD_STROBE_B_GPIO, PHEAD_STROBE_B_PIN, true);
            
            /* get the current line loaded in the head but don't latch until current line time */
            lineTimerBurn(); 

            /* setup for pwm start time */
            GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel1, engine->pwmStartTime );
            pwmStartTime_ = true;
        } 
        else 
        {
            /* PWM'ing the strobe pins doesn't seem to offer much benefit with current hardware*/
            //lineTimerStrobe();
            
            /* latch control during current line. 
               data latch was set low above, lets set it back high now. 
               this should give us enough pulse width */ 
            GPIO_WritePinOutput( PHEAD_LATCH_GPIO, PHEAD_LATCH_PIN, true );                      
            pwmStartTime_ = false;
            
            /* setup for history ( next line ) */
            GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel1, engine->histAdj[0].time );            
        }        
    }  
	 
    /* latch control during Current line load and burn, slt end */
    if( ( GPT_GetStatusFlags( LINE_PRINTER_TIMER_BASE, kGPT_OutputCompare2Flag ) & kGPT_OutputCompare2Flag ) == kGPT_OutputCompare2Flag ) 
    {
        GPT_ClearStatusFlags( LINE_PRINTER_TIMER_BASE, kGPT_OutputCompare2Flag );
        if( !pwmSltTime_ ) 
        {
            /* current line data is transfered to the head, assert the data latch LOW*/ 
            GPIO_WritePinOutput( PHEAD_LATCH_GPIO, PHEAD_LATCH_PIN, false );    //false
            /* setup for end of slt time */
             GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel2, engine->sltTime );

             /* calc history line */
             historyAdjacency();  
             
             pwmSltTime_ = true;
        } 
        else 
        { 
            /* reset for current line ( next line ) */
            GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel2, engine->histAdj[1].time );            
            pwmSltTime_ = false; 
            
            /* increment steps (actual steps on motor counted elsewhere) */
            engine->steps++;

            /* end of line burn. reset for next line */
            lineTimerSLT();
        }
    }
    
    /* half way through line burn (half slt time) */
    if( ( GPT_GetStatusFlags( LINE_PRINTER_TIMER_BASE, kGPT_OutputCompare3Flag ) & kGPT_OutputCompare3Flag ) == kGPT_OutputCompare3Flag ) {     
        GPT_ClearStatusFlags( LINE_PRINTER_TIMER_BASE, kGPT_OutputCompare3Flag );
        
        /*
        if( getDotWearHandle() != NULL && readyToNotify()) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xTaskNotifyFromISR( getDotWearHandle(), DOT_SAMPLE, eSetBits, &xHigherPriorityTaskWoken );  
        } 
        */  
        
    }
    
    return 0;
}



void setTPHMotorIntLevel( unsigned int level )
{ 
    NVIC_SetPriority( GPT2_IRQn, level );
    EnableIRQ( GPT2_IRQn );
}



void startTPHMotorIntrGPT2( uint16_t steps, uint32_t speed )
{ 
    setGPT2IntrType(STEP_TPH_GPT2);
  
    TPHMotorStepCount = 0;
    
    stepsToTake = steps;
    TPHSpeed = speed;

    maxTension = config_.takeup_sensor_max_tension_counts;
    minTension = config_.takeup_sensor_min_tension_counts;
    
    /* initialize timer compare interrupts */    
    gpt_config_t gptConfig;
    
    /* obtain default configuration for general purpose timer module */
    GPT_GetDefaultConfig( &gptConfig );
   
    gptConfig.enableRunInWait = true; 
    gptConfig.enableRunInStop = true;
    gptConfig.enableRunInDoze = true;  
    gptConfig.enableRunInDbg = true;   
    gptConfig.enableFreeRun = false;

    /* initialize gpt module */
    GPT_Init( GPT2, &gptConfig );    
    GPT_SetClockDivider( GPT2, 32 );    
    
    /* load the compare registers with their initial values and setup the output 
       compare mode to toggle output on a match*/

    /* set channel 1 time */
    GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel1, speed ); 
    
    GPT_SetOutputOperationMode( GPT2, kGPT_OutputCompare_Channel1, kGPT_OutputOperation_Disconnected );

    /* enable channel interrupt flags. */
    GPT_EnableInterrupts( GPT2, kGPT_OutputCompare1InterruptEnable );     
    
    setTPHMotorIntLevel( 1 );
    
    GPT_StartTimer( GPT2 );
}



void startTUMotorIntrGPT2( uint16_t steps, uint32_t speed )
{ 
    setGPT2IntrType(STEP_TU_GPT2);
  
    TUMotorStepCount = 0;
    
    stepsToTake = steps;
    TUSpeed = speed;
    
    TUFlopFlag = false;
    measureOnce_ = true;

    maxTension = config_.takeup_sensor_max_tension_counts;
    minTension = config_.takeup_sensor_min_tension_counts;

    /* initialize timer compare interrupts */    
    gpt_config_t gptConfig;
    
    /* obtain default configuration for general purpose timer module */
    GPT_GetDefaultConfig( &gptConfig );
   
    gptConfig.enableRunInWait = true; 
    gptConfig.enableRunInStop = true;
    gptConfig.enableRunInDoze = true;  
    gptConfig.enableRunInDbg = true;   
    gptConfig.enableFreeRun = false;

    /* initialize gpt module */
    GPT_Init( GPT2, &gptConfig );    
    GPT_SetClockDivider( GPT2, 32 );    

    /* load the compare registers with their initial values and setup the output 
       compare mode to toggle output on a match*/

    /* set channel 1 time */
    GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel1, speed ); 

    GPT_SetOutputOperationMode( GPT2, kGPT_OutputCompare_Channel1, kGPT_OutputOperation_Disconnected );

    /* enable channel interrupt flags. */
    GPT_EnableInterrupts( GPT2, kGPT_OutputCompare1InterruptEnable );     
    
    setTPHMotorIntLevel( 1 );
    
    GPT_StartTimer( GPT2 );
}


void stopTPHMotorIntrGPT2( void )
{
    /* stop the timer */
    GPT_StopTimer( GPT2 );
    
    /* reset our count */
    GPT_SoftwareReset( GPT2 );

    /* disable channel interrupt flags. */    
    GPT_DisableInterrupts(LINE_PRINTER_TIMER_BASE, kGPT_OutputCompare1InterruptEnable);     
    
    /* clear any flags that might be set */
    GPT_ClearStatusFlags( LINE_PRINTER_TIMER_BASE, kGPT_OutputCompare1Flag );
}


void setGPT2IntrType(TPHIntrType intrType)
{
    TPHMotorStepCount = 0;
  
    if( intrType == STEP_TPH_GPT2 )
    {   
        GPT2Intr = stepTPHMotorIntrGPT2; 
    }
    else if( intrType == BURN_LINE )
    {
        GPT2Intr = lineTimerIntr;
    }
    else if(intrType == STEP_TU_GPT2)
    {
        GPT2Intr = stepTUMotorIntrGPT2;
    }
    else
    {
        GPT2Intr = lineTimerIntr;
    }
}


void setRampSpeed(uint16_t speed)
{
    rampSpeed = speed;
}

void setStartTUOnce(bool start)
{
    startTUOnce_ = start;
}

void setTensionModifier(uint16_t counts)
{
    tensionModifier = counts;
}

uint16_t getTensionModifier(void)
{
    return tensionModifier;
}