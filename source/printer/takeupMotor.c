#include "takeupMotor.h"
#include "fsl_pit.h"
#include "virtualScope.h"
#include "developmentSettings.h"
#include "idleTask.h"

//#define LARGE_GAP_HARDCODING

#define LARGE_GAP_THRESHOLD 70
#define SMALL_GAP_THRESHOLD 70

#define AVERAGING_WINDOW_SIZE 31

#define LARGE_GAP_DIP_THRESHOLD 5
#define SMALL_GAP_DIP_THRESHOLD 5

#define SMALLEST_LABEL_LENGTH_GT_GAP 500
#define SMALLEST_LABEL_LENGTH_HT_GAP 1500

#define GAP_WIDTH_THRESHOLD 45

#define STEP_TO_NEXT_LABEL_MOD_LARGE_GAP 84
#define STEP_TO_NEXT_LABEL_MOD_SMALL_GAP 34

#define LARGE_GAP_SIZING_MOD 60

#ifdef TFinkDeleteChrisArrays
#define TIGHTEN_COUNTS_ARRAY_SIZE 500
#define TENSION_COUNTS_ARRAY_SIZE 500

#define TIGHTEN_SPEED_COUNTS_ARRAY_SIZE 500
#define TENSION_SPEED_COUNTS_ARRAY_SIZE 500
#endif

#define SHOOT_COUNT_ARRAY_SIZE 10000

#define DIPS_ARRAY_SIZE 10

#define SHOOT_TO_PH_DISTANCE_OFFSET 700
#define SWITCH_TO_PID_LINES_LEFT 80


/* stepTUMotorIntr() and support functions */
#define TUHISTORYSIZE 8
#define TU_SLOPE 5
#define TU_RUNNING_AVG_ARRAY_SIZE 8

#ifdef TFinkFullStep
#define TU_MOTOR_MIN_STEP_TIME_US 1000   
#define TU_MOTOR_MAX_STEP_TIME_US 6000 
#define MAX_TU_STEP_PERIOD_CHANGE 400
#else
#define TU_MOTOR_MIN_STEP_TIME_US 620   
#define TU_MOTOR_MAX_STEP_TIME_US 3000 
#define MAX_TU_STEP_PERIOD_CHANGE 200     //TFinkToDo Carlos thinks this should be 60 max
#endif


/* stepTUMotorIntr() and support functions */
static short             TUTensionSlope = 0;     
static bool              torqueSensorSaturated = false;   /* Torque reading close to 100% */
static unsigned short    desiredTorque = 0;   
/* An array of the average takeup motor average times. Used to calculate an average of average times */
static uint16_t          tuRunningAvgArray[TU_RUNNING_AVG_ARRAY_SIZE] = {0};
static unsigned char     tuRunningAvgArrayIndex = 0;
static unsigned short    tuRunningAvgNumValidReadings = 0;
static unsigned long     tuRunningAvgSum = 0;
static unsigned short    tuTimeRunningAverage = 0;     /* Average of the last X average takeup Step times */
static short             adjustTightenTorque_CloseToMaxTorque = 0;  /* adjustment to starting torque. Used to avoid torque going too high */
static short             adjustTightenTorqueForAccuracy = 0;


AT_NONCACHEABLE_SECTION_INIT(static unsigned short maxTension) = 0;
AT_NONCACHEABLE_SECTION_INIT(static unsigned short minTension) = 0;
AT_NONCACHEABLE_SECTION_INIT(static unsigned short tensionSensorSaturate) = 0;



static bool debugOnce_ = true;

/* TMR2 Clock source divider for Ipg clock source */
#define TMR2_CLOCK_SOURCE_DIVIDER (128U)
/* The frequency of the source clock after divided. */
#define TMR2_SOURCE_CLOCK (CLOCK_GetFreq(kCLOCK_IpgClk) / TMR2_CLOCK_SOURCE_DIVIDER)

/* uSec between each of the TMR2 interrupt handler calls that make up a full TU motor step */
static uint8_t timeBetweenStepperIntr = 20; 

/* uSec between each TU motor step */
AT_NONCACHEABLE_SECTION_INIT(static uint16_t TUSpeed) = 0; 
AT_NONCACHEABLE_SECTION_INIT(static uint16_t TPHSpeed) = 0; 

/* uSec to add to the next TUSpeed timer intr if the intialTUSpeed is greater than TUSpeed */
AT_NONCACHEABLE_SECTION_INIT(static uint16_t TUSpeedModifier) = 10; 

/* total amount of steps to take in a stepTUMotor(() or loosenStock() operation */
AT_NONCACHEABLE_SECTION_INIT(static uint16_t stepsToTake) = 0;

/* amount of steps taken so far in a stepTUMotor() or loosenStock() operation */
AT_NONCACHEABLE_SECTION_INIT(static uint16_t stepsTaken) = 0; 
AT_NONCACHEABLE_SECTION_INIT(static uint16_t stepsTakenTPHIntr) = 0; 

/* how many TU sensor ADC counts to tighten to */
static uint16_t torqueThreshold = 0; 

/* function pointer declaration, used in takeupIntrHandler() */
uint8_t (*tmr2Intr) (void); 

static uint16_t lastTensionMeasurement = 0;



static bool stillFindingDesriredTension = true;
static uint16_t tensionCount = 0;
static unsigned short staticTUMotorStepTime = 0;   
static unsigned short lastLabelAverageTUStepTime;
static unsigned short immediateTUSpeedAdjustDueToHighTorque = 0;
static unsigned short TUSpeedAdjustDueToHighTorque = 0;
static tuControlMethodEnum tuControlState = INVALID;

static uint16_t lastSpeed = 0;

static bool rampDone = false;
static uint16_t rampTarget = 0;
static uint16_t rampStart = 0;
static uint16_t rampStepModifier = 1;

static bool takeupBusy = false;
static bool TPHBusy = false;
static bool takingUpPaper = true;
static uint16_t checkForPaperTimerCount = 0;

static int shootCounts[SHOOT_COUNT_ARRAY_SIZE] = {0};
AT_NONCACHEABLE_SECTION_INIT(static int dipIndices[DIPS_ARRAY_SIZE]) = {0};

AT_NONCACHEABLE_SECTION_INIT(static uint16_t totalStepsTaken) = 0;
AT_NONCACHEABLE_SECTION_INIT(static char numDips) = 0;

AT_NONCACHEABLE_SECTION_INIT(static int startFilterIdx) = 0;
//static int endFilterIdx = 31;
AT_NONCACHEABLE_SECTION_INIT(static int endFilterIdx) = AVERAGING_WINDOW_SIZE;

static int labelSizeInQuarterSteps = 4999;
AT_NONCACHEABLE_SECTION_INIT(static int firstGapIndex) = 0;
AT_NONCACHEABLE_SECTION_INIT(static int secondGapIndex) = 0;
AT_NONCACHEABLE_SECTION_INIT(static int stepsToNextLabel) = 0;
AT_NONCACHEABLE_SECTION_INIT(static int stepsBackToGap) = 0;
AT_NONCACHEABLE_SECTION_INIT(static bool largeGapFlag) = false;


AT_NONCACHEABLE_SECTION_INIT(static uint16_t LTVal) = 0;
AT_NONCACHEABLE_SECTION_INIT(static uint16_t TUVal) = 0;

AT_NONCACHEABLE_SECTION_INIT(static int printDip) = 0;

AT_NONCACHEABLE_SECTION_INIT(static bool readyToRecordTakeupSteps) = false;

static bool continueSteppingAfterTighten = false;

AT_NONCACHEABLE_SECTION_INIT(bool readyToRecordShootVal) = false;

extern void delay_uS( unsigned int time );

extern Pr_Config config_;
extern PrStatusInfo currentStatus;
extern PrStatusInfo prevStatus;

int* previousArray = NULL;
int totalLength = 0;
#define BLOCK_SIZE 50

AT_NONCACHEABLE_SECTION_INIT(int lastDip) = 0;
AT_NONCACHEABLE_SECTION_INIT(int lastDipStart) = 0;
AT_NONCACHEABLE_SECTION_INIT(int firstDip) = -1;
AT_NONCACHEABLE_SECTION_INIT(int firstDipStart) = -1;

static bool rampFlag = false;
uint16_t TPHSteps = 0;
uint16_t TPHStepsToTake = 0;
bool TPHIntrDone = false;


static uint16_t TPHStepsThisPrint = 0;
static uint16_t continuousDetectionSteps = 0;



int calculateAverage(int array[], int length) 
{
    int sum = 0;
    int average;

    // Calculate the sum of all elements in the array
    for (int i = 0; i < length; i++) 
    {
        sum += array[i];
    }

    // Calculate the average
    if (length != 0) 
    {
        average = sum / length;
    } 
    else 
    {
        // Handle the case where the array is empty to avoid division by zero
        average = 0.0;
    }

    return average;
}


// Function to shift elements of an array to the left
void shiftLeft(int arr[], int n, int shiftAmount) 
{
    for (int i = 0; i < n - shiftAmount; i++) 
    {
        arr[i] = arr[i + shiftAmount];
    }
    // Fill the remaining slots with 0 or any other default value
    for (int i = n - shiftAmount; i < n; i++) 
    {
        arr[i] = 0; // or any other default value
    }
}


// Function to add a smaller array to the end of a larger array and shift elements
void addAndShift(int smaller[], int larger[], int smallerSize, int largerSize) 
{
    // Shift elements of the larger array to the left by the size of the smaller array
    shiftLeft(larger, largerSize, smallerSize);
    
    // Copy elements from the smaller array to the end of the larger array
    for (int i = 0; i < smallerSize; i++) 
    {
        larger[largerSize - smallerSize + i] = smaller[i];
    }
}


int findDips(int waveform[], int length, double threshold) 
{
    int i;
    int in_dip = 0; // Flag to indicate whether currently in a dip
    int dip_start_index = -1;
    int dipsCount = 0; // Variable to store the total number of dips found

    for (i = 0; i < length; i++) 
    {
        if (waveform[i] < threshold) 
        {
            if (!in_dip) 
            {
                in_dip = 1; // Enter dip
                dip_start_index = i;
            }
        } 
        else 
        {
            if (in_dip) 
            {
                in_dip = 0; // Exit dip
                int dip_end_index = i - 1;
                int middle_index = (dip_start_index + dip_end_index) / 2;
                //PRINTF("Dip found: Start index = %d, Middle index = %d, End index = %d\r\n", 
                       //dip_start_index, middle_index, dip_end_index);
                dipsCount++; // Increment the total number of dips found
            }
        }
    }

    // Check if dip continued till the end of the waveform
    if (in_dip) 
    {
        int dip_end_index = length - 1;
        int middle_index = (dip_start_index + dip_end_index) / 2;
        //PRINTF("Dip found: Start index = %d, Middle index = %d, End index = %d\r\n", 
               //dip_start_index, middle_index, dip_end_index);
        dipsCount++; // Increment the total number of dips found
    }

    // Print the total number of dips found
    //PRINTF("Total number of dips found: %d\r\n", dipsCount);
    return dipsCount;
}


void find_lowest_points(const int* waveform, int length, int threshold_value, int dip_threshold, int width_threshold) 
{
    int dip_start = -1; // Variable to store the start index of a dip
    lastDip = 0;
    lastDipStart = 0;
    firstDip = -1;
    firstDipStart = -1;
    bool firstDipFound = false;

    // Iterate through the waveform
    for (int i = 1; i < length; ++i) 
    {
        if (waveform[i] <= threshold_value) 
        {
            // Found a dip below the threshold
            if (dip_start == -1) 
            {
                // Set the start index of the dip
                dip_start = i;
            }
        }
        else 
        {
            // Check if we were in a dip
            if (dip_start != -1) 
            {
                // Calculate dip width
                int dip_width = i - dip_start;

                // Only process dips that meet the width threshold
                if (dip_width >= width_threshold) 
                {
                    // Find the lowest point in the dip
                    int lowest_index = dip_start;
                    int lowest_value = waveform[dip_start];

                    // Loop to find the lowest value and its index
                    for (int j = dip_start + 1; j < i; ++j) 
                    {
                        if (waveform[j] < lowest_value) 
                        {
                            lowest_value = waveform[j];
                            lowest_index = j;
                        }
                    }

                    // Check if the dip goes low enough compared to dip start
                    if (waveform[dip_start] - lowest_value >= dip_threshold) 
                    {
                        // Set firstDip and firstDipStart if this is the first valid dip
                        if (!firstDipFound) 
                        {
                            firstDip = lowest_index;
                            firstDipStart = dip_start;
                            firstDipFound = true;
                        }

                        // Track the last dip
                        lastDip = lowest_index;
                        lastDipStart = dip_start;

                        // Store the index of the lowest point
                        dipIndices[numDips] = lowest_index;
                        numDips++;
                    }
                }

                // Reset dip_start for the next dip
                dip_start = -1;
            }
        }
    }
}


int find_lowest_points_start(const int* waveform, int length, int threshold_value) {
    int dip_start = -1; // Variable to store the start index of a dip
    int lowest_dip_start = -1; // Variable to store the index where the lowest dip started

    // Iterate through the waveform
    for (int i = 1; i < length; ++i) 
    {
        if (waveform[i] <= threshold_value) 
        {
            // Found a dip below the threshold
            if (dip_start == -1) 
            {
                // Set the start index of the dip
                dip_start = i;
            }
        } 
        else 
        {
            // Check if we were in a dip
            if (dip_start != -1) 
            {
                // Find the lowest point in the dip
                int lowest_index = dip_start;
                for (int j = dip_start + 1; j < i; ++j) 
                {
                    if (waveform[j] < waveform[lowest_index]) 
                    {
                        lowest_index = j;
                        //printDip = lowest_index;
                    }
                }

                // Update lowest_dip_start if this dip is lower than previous dips
                if (lowest_dip_start == -1 || waveform[lowest_index] < waveform[lowest_dip_start]) 
                {
                    lowest_dip_start = dip_start;
                    printDip = dip_start;
                }

                // Reset the dip_start for the next dip
                dip_start = -1;
            }
        }
    }

    // Return the index where the lowest dip started
    printDip = lowest_dip_start;
    return lowest_dip_start;
}


int find_lowest_points_lowest(const int* waveform, int length, int threshold_value) 
{
    int dip_start = -1; // Variable to store the start index of a dip
    int lowest_index = -1; // Variable to store the index of the lowest point within the dip

    // Iterate through the waveform
    for (int i = 1; i < length; ++i) 
    {
        if (waveform[i] <= threshold_value) 
        {
            // Found a dip below the threshold
            if (dip_start == -1) 
            {
                // Set the start index of the dip
                dip_start = i;
            }
            // Check if the current point is lower than the previous lowest point
            if (lowest_index == -1 || waveform[i] < waveform[lowest_index]) 
            {
                lowest_index = i;
            }
        } 
        else 
        {
            // Check if we were in a dip
            if (dip_start != -1) 
            {
                // Reset the dip_start for the next dip
                dip_start = -1;
            }
        }
    }

    // Return the index of the lowest point within the dip
    printDip = lowest_index;
    return lowest_index;
}


int countDipsBelowThreshold(int waveform[], int length, int threshold) 
{
    int count = 0;
    int inDip = 0; // Flag to track whether currently inside a dip

    for (int i = 0; i < length; i++) 
    {
        if (waveform[i] < threshold) 
        {
            if (!inDip) 
            {
                inDip = 1; // Start of a new dip
                count++;
            }
        } 
        else 
        {
            inDip = 0; // End of dip
        }
    }
    return count;
}


void condenseAppendAndResize(int originalArray[], int originalArrayLength, int secondArray[], int secondArrayLength) 
{
    // Condense the second array to the size of the first array
    int blockSize = secondArrayLength / originalArrayLength;
    int* condensedSecondArray = (int*)malloc(originalArrayLength * sizeof(int));
    for (int i = 0; i < originalArrayLength; i++) 
    {
        int sum = 0;

        for (int j = 0; j < blockSize; j++) 
        {
            sum += secondArray[i * blockSize + j];
        }
        
        condensedSecondArray[i] = sum / blockSize;
    }

    // Append the condensed second array to the original array
    for (int i = 0; i < originalArrayLength; i++) 
    {
        originalArray[originalArrayLength + i] = condensedSecondArray[i];
    }

    // Condense the combined array to the size of the original array
    for (int i = 0; i < originalArrayLength; i++) 
    {
        int sum = 0;
        for (int j = 0; j < blockSize; j++) 
        {
            sum += originalArray[i * blockSize + j];
        }
        originalArray[i] = sum / blockSize;
    }

    // Free memory allocated for the condensed second array
    free(condensedSecondArray);
}


// Function to average values in an array from start to end
void averageAndStore(int* array, int start, int end) 
{
    printDip = 0;

    if (array == NULL || start < 0 || end < start) 
    {
        // Handle invalid input
        return;
    }

    // Calculate the sum of values from start to end
    int sum = 0;
    int count = 0;

    for (int i = start; i <= end; ++i) 
    {
        sum += array[i];
        count++;
    }

    if (count == 0) 
    {
        // Avoid division by zero
        return;
    }

    // Calculate the average
    int average = sum / count;

    // Store the average value in the array from start to end
    for (int i = start; i <= end; ++i) 
    {
        array[i] = average;
    }
}


// Function to find a percentage of the average of an array
double find_percentage_of_average(const int* array, int length, double percentage) 
{
    // Calculate the sum of all values in the array
    int sum = 0;
    for (int i = 0; i < length; ++i) 
    {
        sum += array[i];
    }

    // Calculate the average
    double average = (double)sum / length;

    // Calculate the desired percentage of the average
    double result = percentage / 100.0 * average;

    return result;
}


/******************************************************************************/
/*!   \fn 
        void takeupIntrHandler( void )  

      \brief  
        This is the interrupt handler for TMR2, a function pointer is used to 
        change the behavior of the interrupt. setTmr2IntrType() is used to change
        the function that is called in this interrupt.
      \author
        Chris King
*******************************************************************************/
void takeupIntrHandler( void )
{    
    /* if TMR2 channel 1 timer period is expired, this channel's behavior is modified using setTmr2IntrType()*/
    if( (QTMR_GetStatus(TMR2, kQTMR_Channel_1) & kQTMR_CompareFlag) == kQTMR_CompareFlag )
    {
        /* use setTmr2IntrType(TUIntrType intrType) to choose which Intr function to call here */
        uint8_t intr = tmr2Intr();
    }
  
    /* if TMR2 channel 2 timer period is expired, this channel is currently being used for the 
    print roller motor interrupt handler */
    if( (QTMR_GetStatus(TMR2, kQTMR_Channel_2) & kQTMR_Compare1Flag) == kQTMR_Compare1Flag )
    {
        PrintEngine* engine = getPrintEngine();
      
        /* set our target speed to ramp the print roller motor intr to based on the contrast setting */
        switch(config_.contrast_adjustment)
        {
            case 0: 
                rampTarget = 616;
                break;
            case 1: 
                rampTarget = 616;
                break;
            case 2: 
                rampTarget = 709;
                break;
            case 3: 
                rampTarget = 862;
                break;
            case 4: 
                rampTarget = 862;
                break;
            case 5: 
                rampTarget = 1077;
                break;
            case 6: 
                rampTarget = 1077;
                break;
            case 7: 
                rampTarget = 1077;
                break;
            default: 
                rampTarget = 1231;
                break;
        }
        
        /* ramp at start of print */
        if(TPHSteps < 10)
        {
            if((TPHSpeed > rampTarget))
            {
                TPHSpeed = (TPHSpeed - 20);

                rampFlag = 0;
            }
        }
        else if(TPHSteps >= 10 && TPHSteps < 20) 
        {
            if((TPHSpeed >= rampTarget))
            {
                TPHSpeed = (TPHSpeed - 30);

                rampFlag = 0;
            }
        }
        else if(TPHSteps >= 20 && TPHSteps < 30) 
        {
            if((TPHSpeed >= rampTarget))
            {
                TPHSpeed = (TPHSpeed - 40);

                rampFlag = 0;
            }
        }
        
        /* limit lower bounds of TPHSpeed based on contrast setting */
        switch(config_.contrast_adjustment)
        {
            case 0: 
                if(TPHSpeed < 616)
                {
                    TPHSpeed = 616;
                }
                break;
            case 1: 
                if(TPHSpeed < 616)
                {
                    TPHSpeed = 616;
                }
                break;
            case 2: 
                if(TPHSpeed < 709)
                {
                    TPHSpeed = 709;
                }
                break;
            case 3: 
                if(TPHSpeed < 862)
                {
                    TPHSpeed = 862;
                }
                break;
            case 4: 
                if(TPHSpeed < 862)
                {
                    TPHSpeed = 862;
                }
                break;
            case 5: 
                if(TPHSpeed < 1077)
                {
                    TPHSpeed = 1077;
                }
                break;
            case 6: 
                if(TPHSpeed < 1077)
                {
                    TPHSpeed = 1077;
                }
                break;
            case 7: 
                if(TPHSpeed < 1077)
                {
                    TPHSpeed = 1077;
                }
                break;
            default: 
                if(TPHSpeed < 1231)
                {
                    TPHSpeed = 1231;
                }
        }
        
        /* limit upper bounds of TPHSpeed */
        if(TPHSpeed > 1300)
        {
            TPHSpeed = 1300;
        }
        
        /* step print roller motor */
        stepMainMotor();

        /* increment our step count */
        TPHSteps++;
        stepsTakenTPHIntr++;   //TFink  is TPHSteps the same as stepsTakenTPHIntr?
        
        TPHStepsThisPrint = TPHSteps;

        readyToRecordShootVal = true;
        

    
        /* if TPHSteps >= TPHStepsToTake, end print roller intr */
        if(TPHSteps >= TPHStepsToTake)
        {
            TPHStepsThisPrint = TPHSteps;
            TPHSteps = 0;
            TPHStepsToTake = 0;
            TPHIntrDone = true;
        
            /* clear compare flag */
            QTMR_ClearStatusFlags(TMR2, kQTMR_Channel_2, kQTMR_Compare1Flag);
            
            /* stop TMR2 intr */
            stopTPHIntr();
        }
        else
        {
            /* set timer period */
            QTMR_SetTimerPeriod( TMR2, kQTMR_Channel_2, USEC_TO_COUNT(TPHSpeed, TMR2_SOURCE_CLOCK) );
        
            /* clear compare flag */
            QTMR_ClearStatusFlags(TMR2, kQTMR_Channel_2, kQTMR_Compare1Flag);
        }
    } 
      
#if defined __CORTEX_M && (__CORTEX_M == 4U || __CORTEX_M == 7U)
    __DSB();
#endif
}


/******************************************************************************/
/*!   \fn 
        static uint8_t stepTUMotorIntr( void )
      \brief  
        Controls takeup motor torque for most of the label (pretighten handled
        by tightenStockIntr). This function is called in the TMR2 interrupt 
        handler when stepTUMotor() is called.
      \author
        Chris King
*******************************************************************************/
static uint8_t stepTUMotorIntr( void )
{ 
   #ifdef TFinkFullStep
   #define AVERAGE_TIME_ADDER 6
   #else
   #define AVERAGE_TIME_ADDER 5  /*TFinkToDo! This needs to change based on step size. Higher
                                   constast tend to ramp up. Lower ramp down. */ 
   #endif
   
   
   short TUSpeedChange = 0;
   static uint16_t TUTorqueReading = 0;
   /*********  Handle Transition Between TU Control States *****************/
   switch(tuControlState) {
    case HOLD_TIGHTEN_STEP_SPEED:
      if (stepsTakenTPHIntr > 12) {   
         /* staticTUMotorStepTime = 0 if first label since head up */
         if(staticTUMotorStepTime > TU_MOTOR_MIN_STEP_TIME_US && staticTUMotorStepTime < TU_MOTOR_MAX_STEP_TIME_US) {
            tuControlState = RAMP_TO_STATIC_STEP;
            sendStepsTakenTPHIntrToPrintf(stepsTakenTPHIntr);  //TFinkToDo remove this debug statement.
            toggleDebugPin10uS();
         }  
         else {
            tuControlState = PID_CONTROL;
            toggleDebugPin10uS();
         }
      }
      break;
      
    case RAMP_TO_STATIC_STEP:
      if(TUSpeed == (staticTUMotorStepTime + AVERAGE_TIME_ADDER + TUSpeedAdjustDueToHighTorque)
         || TUSpeed >= TU_MOTOR_MAX_STEP_TIME_US) {  /* should never be > MAX_STEP_TIME */
         tuControlState = STATIC_STEP;  
         adjustTightenTorqueForAccuracy += (desiredTorque - getTakeUpTorque())/2;
         toggleDebugPin10uS();
         //queuePrintStringFromISR(SWITCHED_TO_STATIC_CONTROL);
      }
      break;
      
    case STATIC_STEP:
      {
         static bool switchStatesDueToTorqueDrop = false;
         if ((getTakeUpTorque() < desiredTorque-(desiredTorque/4)) && (stepsTakenTPHIntr > 700)) {                  
            /* Switch to PID control if we're at the end of a label OR if we're printing a long label and torque has fallen. If staticTUMotorStepTime
            is off slightly, on long labels, torque can drop over time. OK to switch to PID, since we've already peeled. */
            switchStatesDueToTorqueDrop = true;
            queuePrintStringFromISR(SWITCHED_TO_PID_DUE_TO_LOW_TORQUE);
         }
         
         if((getNumPrintLinesLeft() < SWITCH_TO_PID_LINES_LEFT) || switchStatesDueToTorqueDrop) {   
            
            tuControlState = PID_CONTROL;
            /* PID will control to the torque value at the end of STATIC_STEP. */
            desiredTorque = calculateTUTorqueSetpoint(); 
            toggleDebugPin10uS();
            switchStatesDueToTorqueDrop = false;
         }
      }
      break;
      
    case PID_CONTROL:
      if(TPHIntrDone == true && TUTorqueReading > desiredTorque-20) {         
        static char stepTUStopSamples = 0;
        /* sample the TUTorqueReading a number of times to make sure that we are actually tight */
        if(stepTUStopSamples >= 5) {
           tuControlState = NORMAL_TU_MOTOR_INTR_EXIT;
           stepTUStopSamples = 0;
           toggleDebugPin10uS();
        }
        else
          stepTUStopSamples++; 
      }
      else if(stepsTaken >= stepsToTake )    
         tuControlState = ABNORMAL_TU_MOTOR_INTR_EXIT;
      
      break;
   }
    
    /*********  ALL Control States *****************/
    sendControlStateToPrintf((unsigned short)tuControlState);
    singleStepTUMotor();
          
    /* Calculate tension slope */
    TUTorqueReading = getTakeUpTorque();
    TUTensionSlope = calculateTUTorqueSlope(TUTorqueReading);
    
    /*****************Handle Control States *********************/
    if(tuControlState == HOLD_TIGHTEN_STEP_SPEED) {
      /* do nothing. use same TUSpeed as we were using in tighten stock interrupt */
    }
    
    if(tuControlState == RAMP_TO_STATIC_STEP) { 
       /* ramp from "Tighten Step Speed" to "Static Step Speed" */
       if((abs(TUSpeed-(staticTUMotorStepTime + AVERAGE_TIME_ADDER + TUSpeedAdjustDueToHighTorque) ))<=MAX_TU_STEP_PERIOD_CHANGE+AVERAGE_TIME_ADDER) {
          TUSpeed = staticTUMotorStepTime + AVERAGE_TIME_ADDER + TUSpeedAdjustDueToHighTorque; /*AVERAGE_TIME_ADDER - prefer downslope to upslope */ 
       }
       else if(TUSpeed > staticTUMotorStepTime)
          TUSpeedChange -=MAX_TU_STEP_PERIOD_CHANGE/2; 
       else if (TUSpeed<staticTUMotorStepTime)
          TUSpeedChange +=MAX_TU_STEP_PERIOD_CHANGE/2; 
       
       setTUSpeed(TUSpeedChange);
    }
    
    if(tuControlState == STATIC_STEP) {
      /* Do nothing. TU Speed fixed */
    }
    
    if(tuControlState == PID_CONTROL) {
       TUSpeedChange = calculatePIDControlTimeChange(TUTensionSlope,TUTorqueReading);
       setTUSpeed(TUSpeedChange);
       
       calculateAverageStepTime();
    }
    
    if(tuControlState == NORMAL_TU_MOTOR_INTR_EXIT) {
         staticTUMotorStepTime = getTUStepTimeRunningAvg(lastLabelAverageTUStepTime);
         shutDownStepTUMotorIntr(); 
    }
     
    if(tuControlState == ABNORMAL_TU_MOTOR_INTR_EXIT) {
        shutDownStepTUMotorIntr(); 
    }
    
    
    /***********   Final TU Speed Checks           ***********/
    /* Verify we're not going to saturate the takeup torque sensor */
    //TPHIntrDone == false means roller stepper not turning which means tension will go high quickly (and we'll shut down - see below)
    if(TPHIntrDone == false && TUTorqueReading>tensionSensorSaturate-20) { 
       if(stepsTakenTPHIntr < 70) {  
          torqueSensorSaturated = true;  // Overshooting early. lower desiredTorque in tightenStock             
          //queuePrintStringFromISR(STEPTUMOTORINTR_TORQUE_OVERSHOOT);  
       }
       else {
          if(immediateTUSpeedAdjustDueToHighTorque <=10) {
             immediateTUSpeedAdjustDueToHighTorque +=2; //immediate adjustment for this label
             TUSpeed = TUSpeed+2;  //This only affects the current label. 
          }else if (immediateTUSpeedAdjustDueToHighTorque <=20){  //Slow down so we don't overcorrect
             immediateTUSpeedAdjustDueToHighTorque +=1; 
             TUSpeed = TUSpeed+1; 
          }
          //toggleDebugPin();
          //queuePrintStringFromISR(STEPTUMOTORINTR_HIGHTORQ_SLOWSTEP);
       }
    } 
    
    /* Final check. Verify TUSpeed within allowable bounds */
    if(TUSpeed > TU_MOTOR_MAX_STEP_TIME_US) 
       TUSpeed = TU_MOTOR_MAX_STEP_TIME_US;
    else if(TUSpeed < TU_MOTOR_MIN_STEP_TIME_US)
       TUSpeed = TU_MOTOR_MIN_STEP_TIME_US; 
    
    
    /***********  Reload the Timer and go again    ***********/
    QTMR_SetTimerPeriod( TMR2, kQTMR_Channel_1, USEC_TO_COUNT(TUSpeed, TMR2_SOURCE_CLOCK) );
    QTMR_ClearStatusFlags(TMR2, kQTMR_Channel_1, kQTMR_CompareFlag);
        
    return 0;
}


/******************************************************************************/
/*!   \fn calculateAverageStepTime
        
      \brief  Returns the average TU Motor step time while TPHIntr is active.
         The purpose of this function is to provide an average step time while
         the takeup torque is held steady. This value will be used as input to
         staticTUMotorStepTime
        
      \author
        Tom Fink
*******************************************************************************/
void calculateAverageStepTime(void)
{
   /* Sum the step times and number of steps once we switch to Dynamic Control */
   static unsigned long stepTimeSum = 0; 
   static unsigned short stepTimeSumNumElements = 0;  
   if(TPHIntrDone == false) {
      if(stepTimeSum < 0xFFFFFFF0) {
         stepTimeSum += TUSpeed;
         stepTimeSumNumElements++;
      }
   }   
   else {
      /* Calculate the average step time while under dynamic control */
      if(stepTimeSumNumElements >0) {
         lastLabelAverageTUStepTime = (unsigned short)(stepTimeSum/stepTimeSumNumElements); 
         stepTimeSumNumElements = 0;
         stepTimeSum = 0;
         // sendLLAverageTimeToPrintf( USEC_TO_COUNT(lastLabelAverageTUStepTime, TMR2_SOURCE_CLOCK));
         toggleDebugPin10uS();
      }
   }
}



/******************************************************************************/
/*!   \fn setTUSpeed
        
      \brief  Bounds checks changeToTUSpeed then calculates a new TUSpeed
        
      \author
        Tom Fink
*******************************************************************************/
void setTUSpeed(short changeToTUSpeed){
       
       /* Ensure we never accelerate or decelerate the TU Motor more than allowed */ 
       if(changeToTUSpeed > MAX_TU_STEP_PERIOD_CHANGE)
          changeToTUSpeed = MAX_TU_STEP_PERIOD_CHANGE;
       else if (changeToTUSpeed < -MAX_TU_STEP_PERIOD_CHANGE)
          changeToTUSpeed = -MAX_TU_STEP_PERIOD_CHANGE;
       
       /* Calculate TUSpeed */
       TUSpeed = (uint16_t)((int16_t)TUSpeed + changeToTUSpeed);
}

/******************************************************************************/
/*!   \fn toggleDebugPin
        
      \brief  Toggle pin for test and development.
        TFinkToDo! - delete prior to production!
        
      \author
        Tom Fink
*******************************************************************************/
void toggleDebugPin(void)
{
   GPIO_WritePinOutput( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, true );
   delay_uS(4);
   GPIO_WritePinOutput( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, false );
}

/******************************************************************************/
/*!   \fn toggleDebugPin
        
      \brief  Toggle pin for test and development.
        TFinkToDo! - delete prior to production!
        
      \author
        Tom Fink
*******************************************************************************/
void toggleDebugPin10uS(void)
{
   GPIO_WritePinOutput( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, true );
   delay_uS(10);
   GPIO_WritePinOutput( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, false );
}

/******************************************************************************/
/*!   \fn calculatePIDControlTimeChange
        
      \brief  This is a PID control loop without the 'I' (integral). 
        
      \author
        Tom Fink
*******************************************************************************/
short calculatePIDControlTimeChange(short tuTensionSlope, unsigned short tuTorqReading)
{    
    short tuSpeedChange = 0;
   
    #ifdef TFinkFullStep
    /* Differential Compensation */
    if(tuTensionSlope > 10*TU_SLOPE)
      tuSpeedChange = 60;                       
    if(tuTensionSlope > 8*TU_SLOPE)
       tuSpeedChange = 40;  
    else if(tuTensionSlope > 4*TU_SLOPE)
       tuSpeedChange = 20;
    else if(tuTensionSlope > 2*TU_SLOPE)
       tuSpeedChange = 10; 
    
    if (tuTensionSlope < -10*TU_SLOPE)
       tuSpeedChange = -80; 
    if (tuTensionSlope < -8*TU_SLOPE)
       tuSpeedChange = -60; 
    else if (tuTensionSlope < -4*TU_SLOPE)
       tuSpeedChange = -40;  
    else if (tuTensionSlope < -2*TU_SLOPE)
       tuSpeedChange = -20;  
    #else
        /* Differential Compensation */
    if(tuTensionSlope > 10*TU_SLOPE)
      tuSpeedChange = 30;                       
    if(tuTensionSlope > 8*TU_SLOPE)
       tuSpeedChange = 20;  
    else if(tuTensionSlope > 4*TU_SLOPE)
       tuSpeedChange = 10;
    else if(tuTensionSlope > 2*TU_SLOPE)
       tuSpeedChange = 5; 
    
    if (tuTensionSlope < -10*TU_SLOPE)
       tuSpeedChange = -40; 
    if (tuTensionSlope < -8*TU_SLOPE)
       tuSpeedChange = -30; 
    else if (tuTensionSlope < -4*TU_SLOPE)
       tuSpeedChange = -20;  
    else if (tuTensionSlope < -2*TU_SLOPE)
       tuSpeedChange = -10; 
    #endif
      
    
    #ifdef TFinkFullStep
    /* Proportional Compensation */        
    if(tuTorqReading > (desiredTorque+20)) //need to change the window in calibration
    {    
       if(tuTorqReading > ((float)desiredTorque * 1.15))
          tuSpeedChange += 60; 
       else if(tuTorqReading > ((float)desiredTorque * 1.1))
          tuSpeedChange += 30;  
       else if(tuTorqReading > ((float)desiredTorque * 1.05))
          tuSpeedChange += 12; 
       else
          tuSpeedChange += 6;   
    }
    else if(tuTorqReading < (desiredTorque)) 
    {
       if(tuTorqReading < ((float)desiredTorque * 0.94))
          tuSpeedChange += -60;
       else if(tuTorqReading < ((float)desiredTorque * 0.96))
          tuSpeedChange += -50; 
       else if(tuTorqReading < ((float)desiredTorque * 0.98))
          tuSpeedChange += -40; 
       else
          tuSpeedChange += -30; 
    }
    #else
       /* Proportional Compensation */        
    if(tuTorqReading > (desiredTorque+20)) //need to change the window in calibration
    {    
       if(tuTorqReading > ((float)desiredTorque * 1.15))
          tuSpeedChange += 30; 
       else if(tuTorqReading > ((float)desiredTorque * 1.1))
          tuSpeedChange += 15;  
       else if(tuTorqReading > ((float)desiredTorque * 1.05))
          tuSpeedChange += 6; 
       else
          tuSpeedChange += 3;   
    }
    else if(tuTorqReading < (desiredTorque)) 
    {
       if(tuTorqReading < ((float)desiredTorque * 0.94))
          tuSpeedChange += -30;
       else if(tuTorqReading < ((float)desiredTorque * 0.96))
          tuSpeedChange += -25; 
       else if(tuTorqReading < ((float)desiredTorque * 0.98))
          tuSpeedChange += -20; 
       else
          tuSpeedChange += -15; 
    }  
    #endif
    
    return(tuSpeedChange);
}


/******************************************************************************/
/*!   \fn singleStepTUMotor

      \brief Pulse the TU Motor Stepper IC. Also pulls in some other related
             functions.
              
      \author
          Tom Fink
*******************************************************************************/
void singleStepTUMotor(void)
{
   /* ping motor enable pin */
   powerOnMotors();
   
   /* check for error status before stepping takeup motor */
   if((((currentStatus.error & HEAD_UP ) != HEAD_UP ) && ((currentStatus.sensor2 & JAMMED_LABEL) != JAMMED_LABEL))) 
   {
      GPIO_WritePinOutput( TAKEUP_MOTOR_STEP_GPIO, TAKEUP_MOTOR_STEP_PIN, true );
      vScopeRecordTakeUp();
      delay_uS(6);  //TFink - stepper IC requirement is 1uS but Chris recommends 6 based on experience. 
      #ifdef TFinkFullStep
      stepsTaken +=2;  //1 full step = 2 half steps.
      #else
      stepsTaken++;
      #endif
      GPIO_WritePinOutput( TAKEUP_MOTOR_STEP_GPIO, TAKEUP_MOTOR_STEP_PIN, false );
   }
}



/******************************************************************************/
/*!   \fn shutDownStepTUMotorIntr

      \brief stops the Takeup Interrupt timer (TMR2) and clears variables
              
      \author
          Tom Fink
*******************************************************************************/
void shutDownStepTUMotorIntr(void)
{
   stopTakeupIntr(); 
   powerOnMotors(); 
   startPrintVScope();  
   TUSpeedAdjustDueToHighTorque += (immediateTUSpeedAdjustDueToHighTorque/4);  //Adjust for rest of roll or until "head up"
   sendTorqeAdmustmentsToPrintf(adjustTightenTorque_CloseToMaxTorque,immediateTUSpeedAdjustDueToHighTorque,TUSpeedAdjustDueToHighTorque,adjustTightenTorqueForAccuracy);
   takeupBusy = false;  //This lets the calling interrupt know stepTUMotorIntr done
   stepsToTake = 0;
   stepsTaken = 0;
   TUSpeed = 0;
   lastLabelAverageTUStepTime = 0; 
   
   immediateTUSpeedAdjustDueToHighTorque = 0;
}


/******************************************************************************/
/*!   \fn resetTUTorqueControlVars

      \brief stepTUMotorIntr() uses the previous label's step speed to calulate
         the next labels step speed (staticTUMotorStepTime). But if contrast 
         is changed or a new label roll used, the previous step times are invalid.

         This function is called resets all the appropriate variables to "reset"
         the control. It uses PID control for the whole first label and then uses
         static control after that.
              
      \author
          Tom Fink
*******************************************************************************/
void resetTUTorqueControlVars(void)
{
   unsigned short i = 0;
   for(i = 0; i<TU_RUNNING_AVG_ARRAY_SIZE;i++)
      tuRunningAvgArray[i] = 0;
   
   tuRunningAvgArrayIndex = 0;
   tuRunningAvgNumValidReadings = 0;
   tuRunningAvgSum = 0;
   tuTimeRunningAverage = 0;
   
   adjustTightenTorque_CloseToMaxTorque = 0; 
   TUSpeedAdjustDueToHighTorque = 0;
   adjustTightenTorqueForAccuracy = 0;
   
   staticTUMotorStepTime = 0;
}


/******************************************************************************/
/*!   \fn getTUStepTimeRunningAvg

      \brief This function returns an average of the last TU_RUNNING_AVG_ARRAY_SIZE
         inputs. The input is the average step speed from the previous label,
         so this function returns the average tu speed of the last 
         TU_RUNNING_AVG_ARRAY_SIZE labels. 

         Prior to being full, it only returns the average of the actual number
         of valid inputs. For example after it is called three times, it returns 
         the average of three inputs rather than TU_RUNNING_AVG_ARRAY_SIZE inptus.     
      \author
          Tom Fink
*******************************************************************************/
unsigned short getTUStepTimeRunningAvg(unsigned short prevTUAvgStepTime)
{           
   /* Is prevTUAvgStepTime within a reasonable range? If not, use the previous good value plus an offset  */
   /* Don't subtract from tuTimeRunningAverage because its unsigned and could roll over to a large positive number */
   if(((prevTUAvgStepTime > tuTimeRunningAverage + 300) || (prevTUAvgStepTime +300  < tuTimeRunningAverage))
      && (tuRunningAvgNumValidReadings > 2))
   {        
      prevTUAvgStepTime = staticTUMotorStepTime+50;  //Risk stepping to slow. If we step too fast motor could lose sync
      queuePrintStringFromISR(STEPTUMOTORINTR_BAD_LAST_LABLEL_TIME);
   }
   
   /* Buffer now full, so subtract oldest value */
   if(tuRunningAvgNumValidReadings >= TU_RUNNING_AVG_ARRAY_SIZE)
      tuRunningAvgSum -=tuRunningAvgArray[tuRunningAvgArrayIndex];
   
   tuRunningAvgSum += prevTUAvgStepTime;
   tuRunningAvgArray[tuRunningAvgArrayIndex] = prevTUAvgStepTime;
   
   tuRunningAvgArrayIndex++;
   if(tuRunningAvgArrayIndex > TU_RUNNING_AVG_ARRAY_SIZE-1)
      tuRunningAvgArrayIndex = 0;
   if(tuRunningAvgNumValidReadings >= TU_RUNNING_AVG_ARRAY_SIZE-1) {
      tuRunningAvgNumValidReadings++;
      if(tuRunningAvgNumValidReadings > TU_RUNNING_AVG_ARRAY_SIZE)
         tuRunningAvgNumValidReadings = TU_RUNNING_AVG_ARRAY_SIZE;
   }
   else {
      tuRunningAvgNumValidReadings++;
   }
   
   if(tuRunningAvgNumValidReadings > 0)
      tuTimeRunningAverage = (unsigned short)(tuRunningAvgSum/tuRunningAvgNumValidReadings);
   
   sendLLAverageTimeToPrintf( USEC_TO_COUNT(tuTimeRunningAverage, TMR2_SOURCE_CLOCK));
   return(tuTimeRunningAverage);
}



/*************************** stepTUMotorIntr Takeup Torque Functions ****************/
unsigned long tuTorqueSum = 0;
static uint16_t TUValHistory[TUHISTORYSIZE] = {0};
static unsigned char TUTorqHistoryIndex = 0;
static unsigned char newestTUTorqueReading = 0;
static unsigned char oldestTUTorqReading = 0;
static unsigned short numValidTUTorqReadings = 0;

/******************************************************************************/
/*!   \fn calculateTUTorqueSlope

      \brief returns the difference of the latest torqueReading minus the oldest
         torqueReading. This represents the slope of the TU Torqe sensor (torque
         increasing or decreasing).
              
      \author
          Tom Fink
*******************************************************************************/
unsigned short  calculateTUTorqueSlope(unsigned short torqueReading)
{
    if(numValidTUTorqReadings >= TUHISTORYSIZE)
      tuTorqueSum -=TUValHistory[TUTorqHistoryIndex];
    
    tuTorqueSum += torqueReading;
    TUValHistory[TUTorqHistoryIndex] = torqueReading;
    
    newestTUTorqueReading = TUTorqHistoryIndex;
    TUTorqHistoryIndex++;
    if(TUTorqHistoryIndex >= TUHISTORYSIZE)
      TUTorqHistoryIndex = 0;
    if(numValidTUTorqReadings >= TUHISTORYSIZE-1) {
      oldestTUTorqReading = TUTorqHistoryIndex;
      numValidTUTorqReadings++;
      if(numValidTUTorqReadings > TUHISTORYSIZE)
         numValidTUTorqReadings = TUHISTORYSIZE;
    }
    else
    {
      oldestTUTorqReading = 0;
      numValidTUTorqReadings++;
    }
    
    unsigned short slope = (int16_t)(TUValHistory[newestTUTorqueReading] - TUValHistory[oldestTUTorqReading]);
    return(slope);
}

/******************************************************************************/
/*!   \fn calculateTUTorqueSetpoint
        
      \brief calculates average TU Torqe based on TUHISTORYSIZE samples. Also
         does some bounds checking.
        
      \author
        Tom Fink
*******************************************************************************/
unsigned short calculateTUTorqueSetpoint(void)
{
   unsigned short tuTorque;
   tuTorque = ((unsigned short)(tuTorqueSum/TUHISTORYSIZE)-50); //make a little lower than we've been having.
   if(tuTorque < 1250)  //is this the right setpoint? 
      tuTorque = 1250;  //About 1V
   else if (tuTorque > tensionSensorSaturate-100)
      tuTorque = tensionSensorSaturate-100;
   
   return(tuTorque);
}




/******************************************************************************/
/*!   \fn 
        static uint8_t stepTPHMotorIntr( void )
      \brief  
        This function is called in the TMR2 interrupt handler when stepTPHMotor()
        is called.
      \author
        Chris King
*******************************************************************************/
static uint8_t stepTPHMotorIntr( void )
{
    powerOnMotors();
  
    GPIO_WritePinOutput( MAIN_MOTOR_STEP_GPIO, MAIN_MOTOR_STEP_PIN, true );
    
    delay_uS(6);
    
    GPIO_WritePinOutput( MAIN_MOTOR_STEP_GPIO, MAIN_MOTOR_STEP_PIN, false );
    
    QTMR_SetTimerPeriod( TMR2, kQTMR_Channel_1, USEC_TO_COUNT(TUSpeed, TMR2_SOURCE_CLOCK) );
    
    stepsTaken++;

    
    if(stepsTaken >= stepsToTake)
    { 
        stopTakeupIntr();
        stepsToTake = 0;
        stepsTaken = 0;
        TUSpeed = 0;
        
        takeupBusy = false;
        
        setTakeUpMotorDirection( BACKWARDM_ );
        setMainMotorDirection( FORWARDM_ );    
    }
    
    
    QTMR_ClearStatusFlags(TMR2, kQTMR_Channel_1, kQTMR_CompareFlag);
    
    return 0;
}


/******************************************************************************/
/*!   \fn 
        static uint8_t stepToLtIntr( void )
      \brief  
        This function is called in the TMR2 interrupt handler when stepToLt()
        is called.
      \author
        Chris King
*******************************************************************************/
static uint8_t stepToLtIntr( void )
{
    powerOnMotors();
    
    GPIO_WritePinOutput( MAIN_MOTOR_STEP_GPIO, MAIN_MOTOR_STEP_PIN, true );

    delay_uS(8);
    
    GPIO_WritePinOutput( MAIN_MOTOR_STEP_GPIO, MAIN_MOTOR_STEP_PIN, false );

    QTMR_SetTimerPeriod( TMR2, kQTMR_Channel_1, USEC_TO_COUNT(TUSpeed, TMR2_SOURCE_CLOCK) );
    
  
    stepsTaken++;
    
    LTVal = getLabelTaken();
    
    //PRINTF("\r\nLT VAL: %d\r\n", LTVal);
    
    if(LTVal >= 150 /*LABEL_TAKEN_THRESHOLD_LABEL*/)
    {
      //PRINTF("steps taken to LT: %d\r\n", stepsTaken);
      
        stopTPHMotorIntrGPT2();
        stopTakeupIntr();
        
        stepsToTake = 0;
        stepsTaken = 0;
        TUSpeed = 0;
        
        takeupBusy = false;
        
        setTakeUpMotorDirection( BACKWARDM_ );
        setMainMotorDirection( FORWARDM_ ); 
    }
    
    if(stepsTaken >= stepsToTake)
    { 
        stopTPHMotorIntrGPT2();
        stopTakeupIntr();
        stepsToTake = 0;
        stepsTaken = 0;
        TUSpeed = 0;
        
        takeupBusy = false;
        
        setTakeUpMotorDirection( BACKWARDM_ );
        setMainMotorDirection( FORWARDM_ );    
    }
    
    
    QTMR_ClearStatusFlags(TMR2, kQTMR_Channel_1, kQTMR_CompareFlag);
    
    return 0;
}


/******************************************************************************/
/*!   \fn 
        static uint8_t sizeLabelIntr( void )
      \brief  
        This function is called in the TMR2 interrupt handler when sizeLabel()
        is called.
      \author
        Chris King
*******************************************************************************/
static uint8_t sizeLabelIntr( void )
{    
    powerOnMotors();
    
    GPIO_WritePinOutput( MAIN_MOTOR_STEP_GPIO, MAIN_MOTOR_STEP_PIN, true );
    
    delay_uS(5);
    
    GPIO_WritePinOutput( MAIN_MOTOR_STEP_GPIO, MAIN_MOTOR_STEP_PIN, false );
    
    QTMR_SetTimerPeriod( TMR2, kQTMR_Channel_1, USEC_TO_COUNT(TUSpeed, TMR2_SOURCE_CLOCK) );
    
    if(totalStepsTaken < SHOOT_COUNT_ARRAY_SIZE)
    {        
        shootCounts[totalStepsTaken] = pollMediaCounts();
        
        if(shootCounts[totalStepsTaken] > (config_.backingAndlabel * 1.1))  
        {
            shootCounts[totalStepsTaken] = (config_.backingAndlabel * 1.1);
        }
        
    }
    
    if(totalStepsTaken > AVERAGING_WINDOW_SIZE && totalStepsTaken < SHOOT_COUNT_ARRAY_SIZE)
    { 
        double desiredPercentage = LARGE_GAP_THRESHOLD;
        
        if(numDips >= 1)
        {
            //determine the length of gap between labels
            if( (lastDip - lastDipStart) >= GAP_WIDTH_THRESHOLD)
            {
                desiredPercentage = LARGE_GAP_THRESHOLD;
                largeGapFlag = true;
            }
            else
            {
                desiredPercentage = SMALL_GAP_THRESHOLD;
                largeGapFlag = false;
            }
        }
        
        if(numDips >= 1)
        {
            //determine the length of gap between labels
            if( (firstDip - firstDipStart) >= GAP_WIDTH_THRESHOLD || (lastDip - lastDipStart) >= GAP_WIDTH_THRESHOLD)
            {
                desiredPercentage = LARGE_GAP_THRESHOLD;
                largeGapFlag = true;
            }
            else
            {
                desiredPercentage = SMALL_GAP_THRESHOLD;
                largeGapFlag = false;
            }
        }
        
        numDips = 0;
        
        // Call the function to average and store values
        averageAndStore(shootCounts, startFilterIdx, endFilterIdx);
        startFilterIdx++;
        endFilterIdx++;
        
        //Call the function to find the percentage of the average
        double result = find_percentage_of_average(shootCounts, totalStepsTaken, desiredPercentage);
        
        //Call the function to find lowest points of dips
        if(largeGapFlag == true)
        {
            find_lowest_points(shootCounts, totalStepsTaken, result, LARGE_GAP_DIP_THRESHOLD, 5);
        }
        else
        {
            find_lowest_points(shootCounts, totalStepsTaken, result, SMALL_GAP_DIP_THRESHOLD, 5);
        }
        
        if(numDips >= 1)
        {
            continuousDetectionSteps = 0;
        }
    }

    if(numDips > 1 && (dipIndices[1] - dipIndices[0]) >= SMALLEST_LABEL_LENGTH_GT_GAP && (dipIndices[1] - dipIndices[0]) <= 4999)
    {
        stepsToTake = stepsTaken + 1; 
            
        firstGapIndex = dipIndices[0];
        secondGapIndex = dipIndices[1];

        if(largeGapFlag == true)
        {
            //PRINTF("WalMart GT RFID large gap labels detected\r\n");
            labelSizeInQuarterSteps = (dipIndices[1] - dipIndices[0]) - STEP_TO_NEXT_LABEL_MOD_LARGE_GAP;
            stepsBackToGap = ((totalStepsTaken - secondGapIndex)); //steps since second gap
        }
        else
        {
            //PRINTF("Normal GT small gap labels detected\r\n");
            labelSizeInQuarterSteps = (dipIndices[1] - dipIndices[0]) - STEP_TO_NEXT_LABEL_MOD_SMALL_GAP;
            stepsBackToGap = (totalStepsTaken - secondGapIndex); //steps since second gap
        }
    }

    stepsTaken++;
    totalStepsTaken++;
    continuousDetectionSteps++;
    
    if( stepsToTake == stepsTaken )
    {    
        /*
        PRINTF("\r\n");
        PRINTF("shoot through counts:");
        PRINTF("\r\n");
        for(uint16_t ind = 0; ind < endFilterIdx; ind++)
        {
            PRINTF("%d,", shootCounts[ind]);
            takeupDelayShort();
            takeupDelayShort();
        }
        PRINTF("\r\n");
        */
        PRINTF("\r\n");
        PRINTF("largeGapFlag - %d\r\n", largeGapFlag);
        PRINTF("lastDip = %d        lastDipStart = %d\r\n", lastDip, lastDipStart);
        PRINTF("last gap length = %d\r\n", (lastDip - lastDipStart));
        PRINTF("firstDip = %d        firstDipStart = %d\r\n", firstDip, firstDipStart);
        PRINTF("first gap length = %d\r\n", (firstDip - firstDipStart));
        PRINTF("totalStepsTaken - %d\r\n", totalStepsTaken);
        PRINTF("secondGapIndex - %d\r\n", secondGapIndex);
        PRINTF("stepsBackToGap - %d\r\n", stepsBackToGap);
        PRINTF("labelSizeInQuarterSteps - %d\r\n", labelSizeInQuarterSteps);
        PRINTF("cut off line - %f\r\n", (config_.backingAndlabel * 1.1));
        PRINTF("\r\n");
        
      
        if(numDips < 2)
        {
            labelSizeInQuarterSteps = 4999;
            PRINTF("SIZING FAILED - DEFAULTED TO CONTINUOUS\r\n");
        } 
        
        stopTPHMotorIntrGPT2();
        stopTakeupIntr();

        setTakeUpMotorDirection( BACKWARDM_ );
        setMainMotorDirection( FORWARDM_ );   

        stepsToTake = 0;
        stepsTaken = 0;
        totalStepsTaken = 0;
        TUSpeed = 0;
        takeupBusy = false;
        startFilterIdx = 0;
        endFilterIdx = AVERAGING_WINDOW_SIZE;
        numDips = 0;
        printDip = 0;

        for(uint16_t dipsIndex = 0; dipsIndex < DIPS_ARRAY_SIZE; dipsIndex++)
        {
          dipIndices[dipsIndex] = 0; 
        }
        
        for(uint16_t countsIndex = 0; countsIndex < SHOOT_COUNT_ARRAY_SIZE; countsIndex++)
        {
          shootCounts[countsIndex] = 0; 
        } 
    }
    
    
    if(numDips == 0 && continuousDetectionSteps >= 4999)
    {  
        labelSizeInQuarterSteps = 4999; 
        stepsToTake = 0;
        stepsTaken = 0;
        totalStepsTaken = 0;
        TUSpeed = 0;
        takeupBusy = false;
        startFilterIdx = 0;
        endFilterIdx = AVERAGING_WINDOW_SIZE;
        numDips = 0;  
        stepsToNextLabel = 0;
        firstGapIndex = 0;
        secondGapIndex = 0;
        stepsBackToGap = 0;

        PRINTF("continuousDetectionSteps >= 4999\r\n");
        
        for(uint16_t ind = 0; ind < endFilterIdx; ind++)
        {
            takeupDelayShort();
        }
        
        stopTakeupIntr();
        stopTPHMotorIntrGPT2();
    }
        

    QTMR_ClearStatusFlags(TMR2, kQTMR_Channel_1, kQTMR_CompareFlag);
    
    return 0;
}


/******************************************************************************/
/*!   \fn 
        static uint8_t stepToNextLabel( void )
      \brief  
        This function is called in the TMR2 interrupt handler when stepToNextLabel()
        is called.
      \author
        Chris King
*******************************************************************************/
static uint8_t stepToNextLabelIntr( void )
{
    powerOnMotors();
  
    GPIO_WritePinOutput( MAIN_MOTOR_STEP_GPIO, MAIN_MOTOR_STEP_PIN, true );

    delay_uS(8);
    
    GPIO_WritePinOutput( MAIN_MOTOR_STEP_GPIO, MAIN_MOTOR_STEP_PIN, false );
    
    QTMR_SetTimerPeriod( TMR2, kQTMR_Channel_1, USEC_TO_COUNT(TUSpeed, TMR2_SOURCE_CLOCK) );

    stepsTaken++;
    
    if(stepsTaken == 499)
    {
        readyToRecordTakeupSteps = true;
        //PRINTF("ready to record TU\r\n");
    }
    
    if(stepsTaken >= stepsToTake)
    { 
        stopTPHMotorIntrGPT2();
        stopTakeupIntr();
        
        readyToRecordTakeupSteps = false;
        stepsToTake = 0;
        stepsTaken = 0;
        TUSpeed = 0;
        
        takeupBusy = false;
        
        setTakeUpMotorDirection( BACKWARDM_ );
        setMainMotorDirection( FORWARDM_ );    
    }
    
    
    QTMR_ClearStatusFlags(TMR2, kQTMR_Channel_1, kQTMR_CompareFlag);
    
    return 0;
}


/******************************************************************************/
/*!   \fn 
        static uint8_t tightenStockIntr( void )
      \brief  
        This function is called in the TMR2 interrupt handler when tightenStock()
        is called.
      \author
        Chris King
*******************************************************************************/
static uint8_t tightenStockIntr( void )
{  
   
    static unsigned short targetSpeed;
    static unsigned short rampStep;
    /* ping motor enable pin */
    powerOnMotors();

    /*step */  
    GPIO_WritePinOutput( TAKEUP_MOTOR_STEP_GPIO, TAKEUP_MOTOR_STEP_PIN, true );
    delay_uS(6);
    GPIO_WritePinOutput( TAKEUP_MOTOR_STEP_GPIO, TAKEUP_MOTOR_STEP_PIN, false );
    
    static unsigned short maxDecelTUSpeed;
    static bool startUp = true;
    if(startUp) {
       maxDecelTUSpeed = TUSpeed + 900;
       targetSpeed = TUSpeed;
       TUSpeed = 2*TUSpeed;
       rampStep = (TUSpeed-targetSpeed)/20;
       startUp = false;
    }
      
    if(TUSpeed > targetSpeed)
       TUSpeed = TUSpeed-rampStep;   
    
    if(continueSteppingAfterTighten) {   //TFink - only true when tightenStock called from startPrintEngine
       /* Slow down as we reach desiredTorque. Don't want to overshoot */

       if(getTakeUpTorque() >= (desiredTorque+adjustTightenTorqueForAccuracy-500)) {
          GPIO_WritePinOutput( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, true );  //TFinkToDo! - delete prior to production!
          TUSpeed = TUSpeed + 100;
          if(TUSpeed > maxDecelTUSpeed)
             TUSpeed = maxDecelTUSpeed;
          
          targetSpeed = TUSpeed;
       }            
       
       if(getTakeUpTorque() > 800)   //don't start recording until tension rising
          vScopeRecordTakeUp(); 
    }
    
	
	
    /* set timer period*/
    QTMR_SetTimerPeriod( TMR2, kQTMR_Channel_1, USEC_TO_COUNT(TUSpeed, TMR2_SOURCE_CLOCK) );

    /* increment steps */
    stepsTaken++;

    /* if takeup sensor clutch is greater than our desiredTorque or we have passed our 
    750 steps tighten timeout either stop the tighten intr or allow it to continue pulsing 
    without blocking further execution */
    if( getTakeUpTorque() >= (desiredTorque+adjustTightenTorqueForAccuracy) || (stepsTaken > 750) )
    {      
        if(continueSteppingAfterTighten == true)
        {
            stepsToTake = 0;
            stepsTaken = 0;
            takeupBusy = false; 
            startUp = true;
        }
        else
        {
            stopTakeupIntr();
            
            setHalfStepMode(_MAIN_STEPPER);
            setHalfStepMode(_TAKEUP_STEPPER);
            
            stepsToTake = 0;
            stepsTaken = 0;
            //TUSpeed = 0;                   //TFink - have stepTUMotorIntr start at speed tightenStockInter ended with
            takeupBusy = false;  
            startUp = true;
        }
    }
    
    /* clear compare flag */
    QTMR_ClearStatusFlags(TMR2, kQTMR_Channel_1, kQTMR_CompareFlag);
    
    return 0;
}


/******************************************************************************/
/*!   \fn 
        static uint8_t loosenStockIntr( void )
      \brief  
        This function is called in the TMR2 interrupt handler when loosenStock()
        is called.
      \author
        Chris King
*******************************************************************************/
static uint8_t loosenStockIntr( void )
{
    powerOnMotors();
    
    GPIO_WritePinOutput( TAKEUP_MOTOR_STEP_GPIO, TAKEUP_MOTOR_STEP_PIN, true );
    
    delay_uS(6);
    
    GPIO_WritePinOutput( TAKEUP_MOTOR_STEP_GPIO, TAKEUP_MOTOR_STEP_PIN, false );
    
    stepsTaken++;

    if(stepsTaken < 10)
    {
        if((TUSpeed > 1300))
        {
            TUSpeed = (TUSpeed - 1);
        }
    }
    else if(stepsTaken >= 10 && stepsTaken < 20) 
    {
        if((TUSpeed > 1300))
        {
            TUSpeed = (TUSpeed - 3);
        }
    }
    else if(stepsTaken >= 20 && stepsTaken < 30) 
    {
        if((TUSpeed > 1300))
        {
            TUSpeed = (TUSpeed - 5);
        }
    }
    else
    {
        if((TUSpeed > 1300))
        {
            TUSpeed = (TUSpeed - 10);
        }
    }
    
    if(TUSpeed <= 1300)
    {
        TUSpeed = 1300;
    }

    QTMR_SetTimerPeriod( TMR2, kQTMR_Channel_1, USEC_TO_COUNT(TUSpeed, TMR2_SOURCE_CLOCK) );
      
    if( stepsToTake == stepsTaken )
    {
        stopTakeupIntr();
        stepsToTake = 0;
        stepsTaken = 0;
        TUSpeed = 0;
        takeupBusy = false;
        
        setTakeUpMotorDirection( BACKWARDM_ );
        setMainMotorDirection( FORWARDM_ );
    }
    
    QTMR_ClearStatusFlags(TMR2, kQTMR_Channel_1, kQTMR_CompareFlag);
    
    return 0;
}


/******************************************************************************/
/*!   \fn 
        static uint8_t rampMotorsIntr( void )
      \brief  
        This function is called in the TMR2 interrupt handler when rampMotors()
        is called.
      \author
        Chris King
*******************************************************************************/
static uint8_t rampMotorsIntr( void )
{       
    rampStepMotors();
    
    rampStart -= rampStepModifier;
    
    if(rampStart <= rampTarget)
    {
        rampDone = true;
        rampStart = 0;
        rampTarget = 0;
        
        stopTakeupIntr();
    }
    else
    {
        QTMR_SetTimerPeriod(TMR2, kQTMR_Channel_1, USEC_TO_COUNT(rampStart, TMR2_SOURCE_CLOCK)); 
    }

    QTMR_ClearStatusFlags(TMR2, kQTMR_Channel_1, kQTMR_CompareFlag);
    
    return 0;
}


/******************************************************************************/
/*!   \fn 
        static uint8_t rampMainMotorIntr( void )
      \brief  
        This function is called in the TMR2 interrupt handler when rampMotors()
        is called.
      \author
        Chris King
*******************************************************************************/
static uint8_t rampMainMotorIntr( void )
{       
    stepMainMotor();
    
    rampStart -= rampStepModifier;
    
    if(rampStart <= rampTarget)
    {
        rampDone = true;
        rampStart = 0;
        rampTarget = 0;
        
        stopTakeupIntr();
    }
    else
    {
        QTMR_SetTimerPeriod(TMR2, kQTMR_Channel_1, USEC_TO_COUNT(rampStart, TMR2_SOURCE_CLOCK)); 
    }
    
    QTMR_ClearStatusFlags(TMR2, kQTMR_Channel_1, kQTMR_CompareFlag);

    return 0;
}


/******************************************************************************/
/*!   \fn 
        static uint8_t rampMainMotorIntr( void )
      \brief  
        This function is called in the TMR2 interrupt handler when rampMotors()
        is called.
      \author
        Chris King
*******************************************************************************/
static uint8_t rampMainMotorQuarterStepsIntr( void )
{       
    stepMainMotor();
    
    rampStart -= rampStepModifier;
    
    if(rampStart <= rampTarget)
    {
        rampDone = true;
        rampStart = 0;
        rampTarget = 0;
        
        stopTakeupIntr();
    }
    else
    {
        QTMR_SetTimerPeriod(TMR2, kQTMR_Channel_1, USEC_TO_COUNT(rampStart, TMR2_SOURCE_CLOCK)); 
    }
    
    QTMR_ClearStatusFlags(TMR2, kQTMR_Channel_1, kQTMR_CompareFlag);

    return 0;
}


/******************************************************************************/
/*!   \fn 
        static uint8_t backwindStockIntr( void )
      \brief  
        This function is called in the TMR2 interrupt handler when backwindStock()
        is called.
      \author
        Chris King
*******************************************************************************/
static uint8_t backwindIntr( void )
{  
    powerOnMotors();

    GPIO_WritePinOutput( MAIN_MOTOR_STEP_GPIO, MAIN_MOTOR_STEP_PIN, true );
    
    if(getTakingUpPaper() == true)
    {
        GPIO_WritePinOutput( TAKEUP_MOTOR_STEP_GPIO, TAKEUP_MOTOR_STEP_PIN, true );
    }
     
    delay_uS(6);
    
    GPIO_WritePinOutput( MAIN_MOTOR_STEP_GPIO, MAIN_MOTOR_STEP_PIN, false );
    
    if(getTakingUpPaper() == true)
    {
        GPIO_WritePinOutput( TAKEUP_MOTOR_STEP_GPIO, TAKEUP_MOTOR_STEP_PIN, false );
    }

    stepsTaken++;
    
    if(stepsTaken < 10)
    {
        if((TUSpeed > 1000))
        {
            TUSpeed = (TUSpeed - 1);
        }
    }
    else if(stepsTaken >= 10 && stepsTaken < 20) 
    {
        if((TUSpeed > 1000))
        {
            TUSpeed = (TUSpeed - 3);
        }
    }
    else if(stepsTaken >= 20 && stepsTaken < 30) 
    {
        if((TUSpeed > 1000))
        {
            TUSpeed = (TUSpeed - 5);
        }
    }
    else
    {
        if((TUSpeed > 1000))
        {
            TUSpeed = (TUSpeed - 10);
        }
    }
    
    if(TUSpeed <= 1000)
    {
        TUSpeed = 1000;
    }

    QTMR_SetTimerPeriod(TMR2, kQTMR_Channel_1, USEC_TO_COUNT(TUSpeed, TMR2_SOURCE_CLOCK));
    
    if( stepsToTake == stepsTaken )
    {
        stopTakeupIntr();
        stepsToTake = 0;
        stepsTaken = 0;
        TUSpeed = 0;
        takeupBusy = false;
        
        setTakeUpMotorDirection( BACKWARDM_ );
        setMainMotorDirection( FORWARDM_ );
    }
    
    QTMR_ClearStatusFlags(TMR2, kQTMR_Channel_1, kQTMR_CompareFlag);
    
    return 0;
}


/******************************************************************************/
/*!   \fn 
        static uint8_t checkForPaperIntr( void )
      \brief  
        checks for paper on the takeup
      \author
        Chris King
*******************************************************************************/
static uint8_t checkForPaperIntr( void )
{
    powerOnMotors();
    
    GPIO_WritePinOutput( TAKEUP_MOTOR_STEP_GPIO, TAKEUP_MOTOR_STEP_PIN, true );
    
    delay_uS(10);
    
    GPIO_WritePinOutput( TAKEUP_MOTOR_STEP_GPIO, TAKEUP_MOTOR_STEP_PIN, false );
    
    QTMR_SetTimerPeriod(TMR2, kQTMR_Channel_1, USEC_TO_COUNT(TUSpeed, TMR2_SOURCE_CLOCK));
    
    stepsTaken++;

    if( getTakeUpTorque() >= 3950 )
    {
        stopTakeupIntr();
        
        PRINTF("CHECK FOR PAPER - stepsTaken - %d\r\n", stepsTaken);
        
        stepsToTake = 0;
        stepsTaken = 0;
        TUSpeed = 0;
        desiredTorque = 0; 
        takeupBusy = false;            
        checkForPaperTimerCount = 0;
        
        takingUpPaper = false; 
    }
    
    if( getTakeUpTorque() >= desiredTorque )
    {
        stopTakeupIntr();
        
        PRINTF("CHECK FOR PAPER - stepsTaken - %d\r\n", stepsTaken);
        
        stepsToTake = 0;
        stepsTaken = 0;
        TUSpeed = 0;
        desiredTorque = 0; 
        takeupBusy = false;            
        checkForPaperTimerCount = 0;
        
        takingUpPaper = true;
        
        
    }
        
    if(takeupBusy == true)
    {
        checkForPaperTimerCount++;
        
        if(checkForPaperTimerCount >= 2500)
        {
            stopTakeupIntr();
            stepsToTake = 0;
            stepsTaken = 0;
            TUSpeed = 0;
            desiredTorque = 0;
            takeupBusy = false;
            checkForPaperTimerCount = 0;
            
            takingUpPaper = false;
        }
    }
    
    
    QTMR_ClearStatusFlags(TMR2, kQTMR_Channel_1, kQTMR_CompareFlag);
    
    return 0;
}


/******************************************************************************/
/*!   \fn 
        static void setTmr2IntrType(TUIntrType intrType)
      \brief  
        This function is called in stepTUMotor(), tightenStock(), and loosenStock()
        to set what function is called in the TMR2 interrupt.
      \author
        Chris King
*******************************************************************************/
static void setTmr2IntrType(TUIntrType intrType)
{
    if( intrType == STEP_TU )
    {   
        tmr2Intr = stepTUMotorIntr; 
    }
    else if( intrType == STEP_TPH )
    {   
        tmr2Intr = stepTPHMotorIntr; 
    }
    else if( intrType == TIGHTEN_STOCK )
    {      
        tmr2Intr = tightenStockIntr;
    }
    else if( intrType == LOOSEN_STOCK )
    {
        tmr2Intr = loosenStockIntr;
    } 
    else if( intrType == BACKWIND )
    {
        tmr2Intr = backwindIntr;
    }
    else if( intrType == CHECK_FOR_PAPER )
    {
        tmr2Intr = checkForPaperIntr;
    }
    else if( intrType == RAMP_MOTORS )
    {
        tmr2Intr = rampMotorsIntr;
    }
    else if( intrType == RAMP_MAIN)
    {
        tmr2Intr = rampMainMotorIntr;
    }
    else if(intrType == RAMP_MAIN_QUARTER_STEPS)
    {
        tmr2Intr = rampMainMotorQuarterStepsIntr;
    }
    else if( intrType == RAMP_TAKEUP )
    {
        //TODO when needed 
    }
    else if( intrType == SIZE_LABELS )
    {
        tmr2Intr = sizeLabelIntr; 
    }
    else if( intrType == STEP_TO_LT)
    {
        tmr2Intr = stepToLtIntr;
    }
    else if( intrType == STEP_TO_NEXT_LABEL)
    {
        tmr2Intr = stepToNextLabelIntr;
    }
}


/******************************************************************************/
/*!   \fn 
        void initTakeupIntr( void )
      \brief  
        This function inits TMR2, sets the initial timer period, and enables the 
        compare interrupt.
      \author
        Chris King
*******************************************************************************/
void initTakeupIntr( void )
{   
    qtmr_config_t qtmrConfig;
    
    QTMR_GetDefaultConfig( &qtmrConfig );
    qtmrConfig.primarySource = kQTMR_ClockDivide_128;

    QTMR_Init( TMR2, kQTMR_Channel_1, &qtmrConfig );

    NVIC_SetPriority( TMR2_IRQn, 1 );        
    EnableIRQ( TMR2_IRQn );

    QTMR_EnableInterrupts( TMR2, kQTMR_Channel_1, kQTMR_CompareInterruptEnable );
}


/******************************************************************************/
/*!   \fn 
        void initTPHIntr( void )
      \brief  
        This function inits TMR2, sets the initial timer period, and enables the 
        compare interrupt.
      \author
        Chris King
*******************************************************************************/
void initTPHIntr( void )
{      
    qtmr_config_t qtmrConfig;
    
    QTMR_GetDefaultConfig( &qtmrConfig );
    qtmrConfig.primarySource = kQTMR_ClockDivide_128;

    QTMR_Init( TMR2, kQTMR_Channel_2, &qtmrConfig );

    NVIC_SetPriority( TMR2_IRQn, 1 );        
    EnableIRQ( TMR2_IRQn );

    QTMR_EnableInterrupts( TMR2, kQTMR_Channel_2, kQTMR_Compare1InterruptEnable );
}


/******************************************************************************/
/*!   \fn 
        void initPrintLineIntr( void )
      \brief  
        This function inits TMR2, sets the initial timer period, and enables the 
        compare interrupt.
      \author
        Chris King
*******************************************************************************/
void initPrintLineIntr( void )
{   
    qtmr_config_t qtmrConfig;
    
    QTMR_GetDefaultConfig( &qtmrConfig );
    qtmrConfig.primarySource = kQTMR_ClockDivide_128;

    QTMR_Init( TMR2, kQTMR_Channel_3, &qtmrConfig );

    NVIC_SetPriority( TMR2_IRQn, 1 );        
    EnableIRQ( TMR2_IRQn );
    
    QTMR_EnableInterrupts( TMR2, kQTMR_Channel_3, kQTMR_CompareInterruptEnable );
}


/******************************************************************************/
/*!   \fn 
        static void startTakeupIntr( void )
      \brief  
        This function starts the TMR2 timer.
      \author
        Chris King
*******************************************************************************/
static void startTakeupIntr( void )
{      
    QTMR_SetTimerPeriod( TMR2, kQTMR_Channel_1, USEC_TO_COUNT(TUSpeed, TMR2_SOURCE_CLOCK) ); 
    
    QTMR_StartTimer( TMR2, kQTMR_Channel_1, kQTMR_PriSrcRiseEdge );
}


/******************************************************************************/
/*!   \fn 
        static void startTPHIntr( void )
      \brief  
        This function starts the print roller motor interrupt TMR2 timer.
      \author
        Chris King
*******************************************************************************/
void startTPHIntr( uint16_t TPHStepsInput )
{    
    /* init print roller motor intr parameters */
    TPHSteps = 0; // steps taken during print roller motor intr 
    
    if(TPHStepsInput > 0 && TPHStepsInput < 20000)
    {
        TPHStepsToTake = TPHStepsInput; // steps to take during print roller motor intr
        TPHIntrDone = false; // is the print roller motor intr done?
        //TPHStepsThisPrint = 0;
        stepsTakenTPHIntr = 0;

        /* set print roller motor start speed based on our contrast setting */
        switch(config_.contrast_adjustment)
        {
            case 0: 
                TPHSpeed = 1231;
                break;
            case 1: 
                TPHSpeed = 1231;
                break;
            case 2: 
                TPHSpeed = 1231;
                break;
            case 3: 
                TPHSpeed = 1231;
                break;
            case 4: 
                TPHSpeed = 1231;
                break;
            case 5: 
                TPHSpeed = 1231;
                break;
            case 6: 
                TPHSpeed = 1231;
                break;
            case 7: 
                TPHSpeed = 1231;
                break;
            default: 
                TPHSpeed = 1231;
                break;
        }
        
        /* set initial timer period*/
        QTMR_SetTimerPeriod( TMR2, kQTMR_Channel_2, USEC_TO_COUNT(TPHSpeed, TMR2_SOURCE_CLOCK) ); 

        /* start TMR2 */
        QTMR_StartTimer( TMR2, kQTMR_Channel_2, kQTMR_PriSrcRiseEdge );
    }
    else
    {
        PRINTF("TPH steps OOB \r\n");
    }
}

/******************************************************************************/
/*!   \fn 
        static void startTPHIntr( void )
      \brief  
        This function starts the TMR2 timer.
      \author
        Chris King
*******************************************************************************/
void startPrintLineIntr( void )
{      
    //PRINTF("startPrintLineIntr()\r\n");
     
    QTMR_SetTimerPeriod( TMR2, kQTMR_Channel_3, USEC_TO_COUNT(620, TMR2_SOURCE_CLOCK) ); 
    
    QTMR_StartTimer( TMR2, kQTMR_Channel_3, kQTMR_PriSrcRiseEdge );
}


/******************************************************************************/
/*!   \fn 
        static void stopTakeupIntr( void )
      \brief  
        This function stops the TMR2 timer.
      \author
        Chris King
*******************************************************************************/
void stopTakeupIntr( void )
{
    //PRINTF("stopTakeupIntr()\r\n");
  
    QTMR_StopTimer( TMR2, kQTMR_Channel_1 );
   
    takeupBusy = false;
}

/******************************************************************************/
/*!   \fn 
        static void stopTakeupIntr( void )
      \brief  
        This function stops the TMR2 timer.
      \author
        Chris King
*******************************************************************************/
void stopTPHIntr( void )
{
    //PRINTF("stopTPHIntr()\r\n");
    
    QTMR_StopTimer( TMR2, kQTMR_Channel_2 );
   
    TPHBusy = false;
}


/******************************************************************************/
/*!   \fn 
        static void stopPrintLineIntr( void )
      \brief  
        This function stops the TMR2 timer.
      \author
        Chris King
*******************************************************************************/
void stopPrintLineIntr( void )
{
    //PRINTF("stopPrintLineIntr()\r\n");
    
    QTMR_StopTimer( TMR2, kQTMR_Channel_3 );
}


/******************************************************************************/
/*!   \fn 
        void stepTUMotor( uint16_t steps, uint16_t speedInUs )
      \brief  
        This function initializes and starts the TMR2 intr handler for stepping the takeup 
        motor during printing
      \author
        Chris King
*******************************************************************************/
void stepTUMotor( uint16_t steps, uint16_t speedInUs )
{     
    /* init stepTUMotorIntr parameters */
    tuControlState = HOLD_TIGHTEN_STEP_SPEED;
    TPHIntrDone = false; // is the printhead roller motor intr done?
    stepsToTake = steps; // how many steps to take before stopping the takeup motor intr
    stepsTaken = 0; // how many steps have been taken in the takeup motor intr
    maxTension = config_.takeup_sensor_max_tension_counts; // the maximum desired tension during a print
    minTension = config_.takeup_sensor_min_tension_counts; // the minimum desired tension during a print 
    tensionSensorSaturate = (unsigned short)((float)maxTension/MAX_TENSION_MULTIPLIER);
    //TUSpeed = speedInUs; //TFink - have stepTUMotorIntr start at speed tightenStockInter ended with
    takeupBusy = true; // is the takeup motor intr done?
    
    
    /* Clear TU History Variables */
    memset(TUValHistory, 0, sizeof(TUValHistory));
    TUTensionSlope = 0;
    TUTorqHistoryIndex = 0;
    newestTUTorqueReading = 0;
    oldestTUTorqReading = 0;
    numValidTUTorqReadings = 0;
    tuTorqueSum = 0;
    
    /* stop the previous intr */
    stopTakeupIntr();
    
    #ifdef TFinkFullStep
    setFullStepMode(_TAKEUP_STEPPER);  
    #endif
    
    /* init takeup motor intr */
    initTakeupIntr();

    /* set TMR2 intr handler type to step takeup motor*/
    setTmr2IntrType( STEP_TU );
    
    /* start the takeup motor intr timer*/
    startTakeupIntr();
}


/******************************************************************************/
/*!   \fn 
        void stepTPHMotor( uint16_t steps, uint16_t speedInUs )
      \brief  
        This function should be called to step the TPH motor
      \author
        Chris King
*******************************************************************************/
void stepTPHMotor( uint16_t steps, uint16_t speedInUs )
{ 
    initTakeupIntr(); 
    
    stepsToTake = 0;
    stepsTaken = 0;
    
    maxTension = config_.takeup_sensor_max_tension_counts;
    minTension = config_.takeup_sensor_min_tension_counts;
    
    tensionCount = 0;
    stillFindingDesriredTension = true;

    setHalfStepMode(_MAIN_STEPPER);
    setHalfStepMode(_TAKEUP_STEPPER);
    
    setTakeUpMotorDirection( BACKWARDM_ ); 
    setMainMotorDirection( FORWARDM_ );
    
    TUSpeed = speedInUs;
    stepsToTake = steps;
    takeupBusy = true;
    
    setTmr2IntrType( STEP_TPH );
    
    startTakeupIntr();
    
    while(getTakeupBusy() == true){/*PRINTF(".");*/};
}


/******************************************************************************/
/*!   \fn 
        void stepToLt( uint16_t steps, uint16_t speedInUs )
      \brief  
        This function should be called to step the TPH motor
      \author
        Chris King
*******************************************************************************/
void stepToLt( uint16_t steps, uint16_t speedInUs )
{ 
    initTakeupIntr(); 
    
    stepsToTake = 0;
    stepsTaken = 0;
    
    maxTension = config_.takeup_sensor_max_tension_counts;
    minTension = config_.takeup_sensor_min_tension_counts;
    
    tensionCount = 0;
    stillFindingDesriredTension = true;
  
    setQuarterStepMode( _MAIN_STEPPER ); 
    setQuarterStepMode( _TAKEUP_STEPPER );
    
    setTakeUpMotorDirection( BACKWARDM_ ); 
    setMainMotorDirection( FORWARDM_ );
   
    
    TUSpeed = speedInUs;
    stepsToTake = steps;
    takeupBusy = true;
    
    setTmr2IntrType( STEP_TO_LT );

    startTakeupIntr();
    
    if(takingUpPaper == true)
    {
        startTUMotorIntrGPT2(4000, 1700);
    }
    
    while(getTakeupBusy() == true){/*PRINTF(".");*/};
}


/******************************************************************************/
/*!   \fn 
        void sizeLabel( uint16_t steps, uint16_t speedInUs )
      \brief  
        This function should be called to size labels
      \author
        Chris King
*******************************************************************************/
void sizeLabels( uint16_t steps, uint16_t speedInUs )
{ 
    //initTakeupIntr(); 
    
    stepsToTake = 0;
    stepsTaken = 0;
    stepsToNextLabel = 0;
    stepsBackToGap = 0;
    
    maxTension = config_.takeup_sensor_max_tension_counts;
    minTension = config_.takeup_sensor_min_tension_counts;
  
    tensionCount = 0;
    stillFindingDesriredTension = true;
    
    largeGapFlag = false;
    
    setQuarterStepMode( _MAIN_STEPPER ); 
    setQuarterStepMode( _TAKEUP_STEPPER );
    
    setTakeUpMotorDirection( BACKWARDM_ ); 
    setMainMotorDirection( FORWARDM_ );
    
    TUSpeed = speedInUs;
    stepsToTake = steps;
    stepsTaken = 0;
    takeupBusy = true;
    labelSizeInQuarterSteps = 0;
    firstGapIndex = 0;
    secondGapIndex = 0;
    stepsToNextLabel = 0;
    stepsBackToGap = 0;
    continuousDetectionSteps = 0;
    
    setTmr2IntrType( SIZE_LABELS );
    
    startTakeupIntr();
    
    if(takingUpPaper == true)
    {
        startTUMotorIntrGPT2(10000, 1700);
    }
    
    
    while(getTakeupBusy() == true){/*PRINTF(".");*/};
}


/******************************************************************************/
/*!   \fn 
        void stepToNextLabel( uint16_t steps, uint16_t speedInUs )
      \brief  
        This function should be called to step to the next label afer doing a sizeLabel()
      \author
        Chris King
*******************************************************************************/
void stepToNextLabel( uint16_t steps, uint16_t speedInUs )
{ 
    PRINTF("stepToNextLabel() - steps - %d\r\n", steps);
  
    if(steps >= 0 && steps <= 8000)
    { 
        //initTakeupIntr(); 
        
        stepsToTake = 0;
        stepsTaken = 0;
        
        maxTension = config_.takeup_sensor_max_tension_counts;
        minTension = config_.takeup_sensor_min_tension_counts;
        
        tensionCount = 0;
        stillFindingDesriredTension = true;
        
        readyToRecordTakeupSteps = false;
      
        setQuarterStepMode( _MAIN_STEPPER ); 
        setQuarterStepMode( _TAKEUP_STEPPER );
        
        setTakeUpMotorDirection( BACKWARDM_ ); 
        setMainMotorDirection( FORWARDM_ );
        
        TUSpeed = speedInUs;
        stepsToTake = steps;
        takeupBusy = true;
        
        setTmr2IntrType( STEP_TO_NEXT_LABEL );
        
        startTakeupIntr();
        
        if(takingUpPaper == true)
        {
          startTUMotorIntrGPT2(10000, 1700);
        }
        
        while(getTakeupBusy() == true){};
    } 
    else
    {
       PRINTF("stepToNextLabel() was passed negative steps or too many steps - sizing failed\r\n"); 
    }    
}


/******************************************************************************/
/*!   \fn 
        void tightenStock( uint16_t tension, uint16_t speedInUs )
      \brief  
        This function should be called to tighten the label stock at the end of
        a print cycle.
      \author
        Chris King
*******************************************************************************/
void tightenStock( uint16_t tension, uint16_t speedInUs, bool continueStepping, StepSizeEnum tuStepSize)
{
    /* init the takeup motor intr TMR*/
   // GPIO_WritePinOutput( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, true );  //TFinkToDo! - delete prior to production!
    initTakeupIntr();

    /* set up our TMR2 interrupt handler type */
    setTmr2IntrType( TIGHTEN_STOCK );
    
    /* init tighten parameters */
    stepsToTake = 0; // the amount of steps to take, not used during tighten intr
    stepsTaken = 0; // the amount of steps taken
    TUSpeed = speedInUs; // the speed we are tensioning at

    if(torqueSensorSaturated ) {  /* last label was close to full torque */
      torqueSensorSaturated = false;
      if(adjustTightenTorque_CloseToMaxTorque > -500)
         adjustTightenTorque_CloseToMaxTorque -=50;   //adjustTightenTorque_CloseToMaxTorque reset when PH is up
    }
    desiredTorque =  (unsigned short)((short)tension+adjustTightenTorque_CloseToMaxTorque); // the counts we are tensioning to
       
       
    takeupBusy = true; // is the takeup motor still tightening?
    continueSteppingAfterTighten = continueStepping; //if true continue pulsing step pin when intr is done
    
    setHalfStepMode(_MAIN_STEPPER); 
    
    if(tuStepSize == FULL_STEP)
       setFullStepMode(_TAKEUP_STEPPER);
    else
       setHalfStepMode(_TAKEUP_STEPPER);

    /* set motor direction to tighten paper*/
    setTakeUpMotorDirection( BACKWARDM_ ); 

    /* ping the motor enable pin */
    powerOnMotors();

    /* start the takeup motor tighten interrupt*/
    startTakeupIntr();
    
    /* block until tighten intr is done, takeupBusy is set to false inside of
    tighten intr handler */
    while(getTakeupBusy() == true){};
    GPIO_WritePinOutput( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, false );  //TFinkToDo! - delete prior to production!
}


/******************************************************************************/
/*!   \fn 
        void loosenStock( uint16_t steps, uint16_t speedInUs )
      \brief  
        
      \author
          Chris King
*******************************************************************************/
void loosenStock( uint16_t steps, uint16_t speedInUs ) 
{    
    initTakeupIntr();   
  
    stepsToTake = 0;
    stepsTaken = 0;
    
    maxTension = config_.takeup_sensor_max_tension_counts;
    minTension = config_.takeup_sensor_min_tension_counts;
    
    tensionCount = 0;
    stillFindingDesriredTension = true;

    setHalfStepMode(_MAIN_STEPPER);
    setHalfStepMode(_TAKEUP_STEPPER);
  
    setTakeUpMotorDirection( FORWARDM_ ); 
    setMainMotorDirection( BACKWARDM_ );
    
    powerOnMotors();
    takeupDelayShort();
    powerOnMotors();
    
    TUSpeed = speedInUs;
    
    stepsToTake = steps;
    takeupBusy = true;
    
    setTmr2IntrType( LOOSEN_STOCK );
    
    startTakeupIntr();
    
    while(getTakeupBusy() == true){};
}


/******************************************************************************/
/*!   \fn 
        void backwindStock( uint16_t steps, uint16_t speedInUs )
      \brief  
        
      \author
          Chris King
*******************************************************************************/
void backwindStock( uint16_t steps, uint16_t speedInUs ) 
{         
    if(steps > 0 && steps < 1000)
    {
        initTakeupIntr(); 
    
        stepsToTake = 0;
        stepsTaken = 0;
        
        maxTension = config_.takeup_sensor_max_tension_counts;
        minTension = config_.takeup_sensor_min_tension_counts;
        
        tensionCount = 0;
        stillFindingDesriredTension = true;

        setHalfStepMode(_MAIN_STEPPER);
        setHalfStepMode(_TAKEUP_STEPPER);
        
        setTakeUpMotorDirection( FORWARDM_ ); 
        setMainMotorDirection( BACKWARDM_ );
        
        powerOnMotors();
        takeupDelayShort();
        powerOnMotors();
        
        TUSpeed = speedInUs;
        
        stepsToTake = steps;
        takeupBusy = true;
        
        setTmr2IntrType( BACKWIND );

        startTakeupIntr();

        while(getTakeupBusy() == true){};
    }
    else
    {
        setHalfStepMode(_MAIN_STEPPER);
        setHalfStepMode(_TAKEUP_STEPPER);
      
        PRINTF("backwindStock() PASSED BAD STEP COUNT\r\n");
    }
    
}


/******************************************************************************/
/*!   \fn 
        void checkForPaper( uint16_t tension, uint16_t speedInUs )
      \brief  
        checks for paper on the takeup, sets takingUpPaper accordingly
      \author
        Chris King
*******************************************************************************/
void checkForPaper( uint16_t tension, uint16_t speedInUs )
{
    PRINTF("CHECK FOR PAPER - tension = %d\r\n", tension);
  
    /* set up TMR2 */
    qtmr_config_t qtmrConfig;
    
    QTMR_GetDefaultConfig( &qtmrConfig );
    qtmrConfig.primarySource = kQTMR_ClockDivide_128;

    QTMR_Init( TMR2, kQTMR_Channel_1, &qtmrConfig );

    NVIC_SetPriority( TMR2_IRQn, 1 );        
    EnableIRQ( TMR2_IRQn );

    QTMR_EnableInterrupts( TMR2, kQTMR_Channel_1, kQTMR_CompareInterruptEnable );
    
    /* init check for paper intr handler parameters */
    stepsToTake = 0; // steps to take during the takeup motor intr handler, not used during checking for paper
    stepsTaken = 0;  // the current amount of steps taken during the takeup motor intr handler
    TUSpeed = speedInUs; // the speed to pulse the takeup motor step pin during the takeup motor intr handler
    desiredTorque = tension; // the tension threshold used to determine if there is paper wrapped around the takeup hub
    takeupBusy = true; // is the takeup motor intr handler done?
    
    /* set motors to half steps*/
    setHalfStepMode(_MAIN_STEPPER);
    setHalfStepMode(_TAKEUP_STEPPER);
  
    /* set takeup motor direction for tightening */
    setTakeUpMotorDirection( BACKWARDM_ ); 
    
    /* ping motor enable pin */
    powerOnMotors();

    /* set TMR2 intr handler type to check for paper */
    setTmr2IntrType( CHECK_FOR_PAPER );
    
    /* set timer period */
    QTMR_SetTimerPeriod( TMR2, kQTMR_Channel_1, USEC_TO_COUNT(TUSpeed, TMR2_SOURCE_CLOCK) ); 

    /* start TMR 2*/
    QTMR_StartTimer( TMR2, kQTMR_Channel_1, kQTMR_PriSrcRiseEdge );
    
    /* block until takeupBusy is set to false in takeup motor intr handler */
    while(getTakeupBusy() == true){};
}


/******************************************************************************/
/*!   \fn 
        void rampMotors( uint16_t startSpeed, uint16_t endSpeed )
      \brief  
        ramps both motors
      \author
          Chris King
*******************************************************************************/
void rampMotors(uint16_t startSpeed, uint16_t endSpeed)
{
    qtmr_config_t qtmrConfig;
  
    rampDone = false;
    rampStart = startSpeed;
    rampTarget = endSpeed;
    
    maxTension = config_.takeup_sensor_max_tension_counts;
    minTension = config_.takeup_sensor_min_tension_counts;
    
    tensionCount = 0;
    stillFindingDesriredTension = true;
    
    setTmr2IntrType(RAMP_MOTORS);
    
    setHalfStepMode(_MAIN_STEPPER);
    setHalfStepMode(_TAKEUP_STEPPER);

    QTMR_GetDefaultConfig(&qtmrConfig);
    qtmrConfig.primarySource = kQTMR_ClockDivide_128;

    QTMR_Init(TMR2, kQTMR_Channel_1, &qtmrConfig);

    EnableIRQ(TMR2_IRQn);

    QTMR_EnableInterrupts(TMR2, kQTMR_Channel_1, kQTMR_CompareInterruptEnable);
  
    QTMR_SetTimerPeriod(TMR2, kQTMR_Channel_1, USEC_TO_COUNT(rampStart, TMR2_SOURCE_CLOCK)); 
    
    QTMR_StartTimer(TMR2, kQTMR_Channel_1, kQTMR_PriSrcRiseEdge);
  
    while(rampDone == false) {};
    
}


/******************************************************************************/
/*!   \fn 
        void rampMainMotor( uint16_t startSpeed, uint16_t endSpeed )
      \brief  
        ramps just the TPH motor
      \author
          Chris King
*******************************************************************************/
void rampMainMotor(uint16_t startSpeed, uint16_t endSpeed)
{
    qtmr_config_t qtmrConfig;
  
    rampDone = false;
    rampStart = startSpeed;
    rampTarget = endSpeed;
    
    maxTension = config_.takeup_sensor_max_tension_counts;
    minTension = config_.takeup_sensor_min_tension_counts;
    
    tensionCount = 0;
    stillFindingDesriredTension = true;
    
    setTmr2IntrType(RAMP_MAIN);
    
    setHalfStepMode(_MAIN_STEPPER);
    setHalfStepMode(_TAKEUP_STEPPER);
    
    QTMR_GetDefaultConfig(&qtmrConfig);
    qtmrConfig.primarySource = kQTMR_ClockDivide_128;

    QTMR_Init(TMR2, kQTMR_Channel_1, &qtmrConfig);

    EnableIRQ(TMR2_IRQn);

    QTMR_EnableInterrupts(TMR2, kQTMR_Channel_1, kQTMR_CompareInterruptEnable);
  
    QTMR_SetTimerPeriod(TMR2, kQTMR_Channel_1, USEC_TO_COUNT(rampStart, TMR2_SOURCE_CLOCK)); 
    
    QTMR_StartTimer(TMR2, kQTMR_Channel_1, kQTMR_PriSrcRiseEdge);
  
    while(rampDone == false) {};
}


/*!   \fn 
        void rampMainMotor( uint16_t startSpeed, uint16_t endSpeed )
      \brief  
        ramps just the TPH motor
      \author
          Chris King
*******************************************************************************/
void rampMainMotorQuarterSteps(uint16_t startSpeed, uint16_t endSpeed)
{
    qtmr_config_t qtmrConfig;
  
    rampDone = false;
    rampStart = startSpeed;
    rampTarget = endSpeed;
    
    maxTension = config_.takeup_sensor_max_tension_counts;
    minTension = config_.takeup_sensor_min_tension_counts;
    
    tensionCount = 0;
    stillFindingDesriredTension = true;
    
    setTmr2IntrType(RAMP_MAIN_QUARTER_STEPS);
    
    setQuarterStepMode( _MAIN_STEPPER ); 
    //setQuarterStepMode( _TAKEUP_STEPPER );
    
    QTMR_GetDefaultConfig(&qtmrConfig);
    qtmrConfig.primarySource = kQTMR_ClockDivide_128;

    QTMR_Init(TMR2, kQTMR_Channel_1, &qtmrConfig);

    EnableIRQ(TMR2_IRQn);

    QTMR_EnableInterrupts(TMR2, kQTMR_Channel_1, kQTMR_CompareInterruptEnable);
  
    QTMR_SetTimerPeriod(TMR2, kQTMR_Channel_1, USEC_TO_COUNT(rampStart, TMR2_SOURCE_CLOCK)); 
    
    QTMR_StartTimer(TMR2, kQTMR_Channel_1, kQTMR_PriSrcRiseEdge);
  
    while(rampDone == false) {};
}

/******************************************************************************/
/*!   \fn 
        void rampMotorsBack( uint16_t startSpeed, uint16_t endSpeed )
      \brief  
        ramps both motors backwards
      \author
          Chris King
*******************************************************************************/
void rampMotorsBack(uint16_t startSpeed, uint16_t endSpeed)
{
    qtmr_config_t qtmrConfig;
  
    rampDone = false;
    rampStart = startSpeed;
    rampTarget = endSpeed;
    
    maxTension = config_.takeup_sensor_max_tension_counts;
    minTension = config_.takeup_sensor_min_tension_counts;
    
    tensionCount = 0;
    stillFindingDesriredTension = true;
    
    setTmr2IntrType(RAMP_MOTORS);
    
    setHalfStepMode(_MAIN_STEPPER);
    setHalfStepMode(_TAKEUP_STEPPER);
    
    setTakeUpMotorDirection( FORWARDM_ ); 
    setMainMotorDirection( BACKWARDM_ );
    
    QTMR_GetDefaultConfig(&qtmrConfig);
    qtmrConfig.primarySource = kQTMR_ClockDivide_128;

    QTMR_Init(TMR2, kQTMR_Channel_1, &qtmrConfig);

    EnableIRQ(TMR2_IRQn);

    QTMR_EnableInterrupts(TMR2, kQTMR_Channel_1, kQTMR_CompareInterruptEnable);
  
    QTMR_SetTimerPeriod(TMR2, kQTMR_Channel_1, USEC_TO_COUNT(rampStart, TMR2_SOURCE_CLOCK)); 
    
    QTMR_StartTimer(TMR2, kQTMR_Channel_1, kQTMR_PriSrcRiseEdge);
  
    while(rampDone == false) {};
}


/******************************************************************************/
/*!   \fn 
        static void setTUSpeedModifier( uint16_t amountToSlowInUs)
      \brief  
        sets the TUSpeed modifier 
      \author
          Chris King
*******************************************************************************/
void setTUSpeedModifier( uint16_t amountToSlowInUs)
{
    TUSpeedModifier = amountToSlowInUs;
}


/******************************************************************************/
/*!   \fn 
        static uint16_t getTUSpeedModifier( void )
      \brief  
        gets the TUSpeed modifier 
      \author
          Chris King
*******************************************************************************/
uint16_t getTUSpeedModifier( void )
{
    return TUSpeedModifier;
}

/******************************************************************************/
/*!   \fn 
        static uint16_t setTakeupBusy( void )
      \brief  
        
      \author
          Chris King
*******************************************************************************/
void setTakeupBusy( bool busy )
{
    takeupBusy = busy;
}

/******************************************************************************/
/*!   \fn 
        static uint16_t getTakeupBusy( void )
      \brief  
        returns true if the takeup intr is currently running 
      \author
          Chris King
*******************************************************************************/
bool getTakeupBusy( void )
{
    return takeupBusy;
}


/******************************************************************************/
/*!   \fn 
        static uint16_t getTakingUpPaper( void )
      \brief  
        returns true if there is paper on takeup
      \author
          Chris King
*******************************************************************************/
bool getTakingUpPaper( void )
{
    return takingUpPaper;
}

void setTakingUpPaper( bool paperState )
{
    takingUpPaper = paperState;
}


/******************************************************************************/
/*!   \fn 
        static uint16_t getLabelSizeInQuarterSteps( void )
      \brief  
        returns label size in quarter steps
      \author
          Chris King
*******************************************************************************/
int getLabelSizeInQuarterSteps( void )
{
    return labelSizeInQuarterSteps;
}

/******************************************************************************/
/*!   \fn 
        static uint16_t setLabelSizeInQuarterSteps( void )
      \brief  
        sets label size in quarter steps
      \author
          Chris King
*******************************************************************************/
void setLabelSizeInQuarterSteps( uint16_t labelLength )
{
    labelSizeInQuarterSteps = labelLength;
}


/******************************************************************************/
/*!   \fn 
        static uint16_t getStepsToNextLabel( void )
      \brief  
        returns the amount of steps to reach the start of the next label after a sizeLabel()
        in quarter steps
      \author
          Chris King
*******************************************************************************/
int getStepsToNextLabel( void )
{
    return stepsToNextLabel;
}


/******************************************************************************/
/*!   \fn 
        static uint16_t getStepsBackToGap( void )
      \brief  
        
      \author
          Chris King
*******************************************************************************/
int getStepsBackToGap( void )
{
    return stepsBackToGap;
}


/******************************************************************************/
/*!   \fn 
        void takeupDelay( void )
      \brief  
        delay before backwind or loosen to ensure a clean rip on paper before 
        paper retracts
      \author
          Chris King
*******************************************************************************/
void takeupDelay( void )
{
    for(uint16_t ticks1 = 0; ticks1 < 100; ticks1++) 
    {
        powerOnMotors();
      
        for(uint16_t ticks2 = 0; ticks2 < 1000; ticks2++)
        {
            for(uint16_t ticks3 = 0; ticks3 < 1000; ticks3++)
            {
                __NOP();
            }
            __NOP();
        }
        __NOP();
    }
}


/******************************************************************************/
/*!   \fn 
        void takeupDelayMid( void )
      \brief  
        delay before backwind or loosen to ensure a clean rip on paper before 
        paper retracts
      \author
          Chris King
*******************************************************************************/
void takeupDelayMid( void )
{
    for(uint16_t ticks1 = 0; ticks1 < 50; ticks1++) 
    {
        powerOnMotors();
      
        for(uint16_t ticks2 = 0; ticks2 < 600; ticks2++)
        {
            for(uint16_t ticks3 = 0; ticks3 < 1000; ticks3++)
            {
                __NOP();
            }
            __NOP();
        }
        __NOP();
    }
}


/******************************************************************************/
/*!   \fn 
        void takeupDelayShort( void )
      \brief  
        delay before backwind or loosen to ensure a clean rip on paper before 
        paper retracts
      \author
          Chris King
*******************************************************************************/
void takeupDelayShort( void )
{
    for(uint16_t ticks1 = 0; ticks1 < 250; ticks1++) 
    {
        powerOnMotors();
      
        for(uint16_t ticks2 = 0; ticks2 < 100; ticks2++)
        {
            __NOP();
        }
        __NOP();
    }
}


uint16_t getLastSpeed( void )
{
    return lastSpeed;
}

void setLastSpeed( uint16_t speed )
{
    lastSpeed = speed;
}

int* getShootThroughBuffer( void)
{
    return shootCounts;
}

/*
uint8_t* getTensionBuffer( void)
{
    return tensionCounts;
}

uint8_t* getTightenBuffer( void)
{
    return tightenCounts;
}

uint8_t* getTensionSpeedBuffer( void)
{
    return tensionSpeedCounts;
}

uint8_t* getTightenSpeedBuffer( void)
{
    return tightenSpeedCounts;
}
*/

int getPrintDip(void)
{
    return printDip;
}

uint16_t getLastTensionMeasurement(void)
{
    return lastTensionMeasurement;
}

bool getReadyToRecordTakeupSteps(void)
{
    return readyToRecordTakeupSteps;
}

bool getLargeGapFlag(void)
{
    return largeGapFlag;
}

uint16_t getTPHStepsThisPrint(void)
{
    return TPHStepsThisPrint;
}

bool getReadyToRecordShootVal(void)
{
    return readyToRecordShootVal;
}

void setReadyToRecordShootVal(bool ready)
{
    readyToRecordShootVal = ready;
}

bool getTPHIntrDone( void )
{
    return TPHIntrDone;
}
  
uint16_t getFirstGapIndex( void )
{
    return firstGapIndex;
}

uint16_t getSecondGapIndex( void )
{
    return secondGapIndex;
}