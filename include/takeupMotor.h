#ifndef PAPER_TAKEUP_MOTOR_H
#define PAPER_TAKEUP_MOTOR_H

#include "fsl_debug_console.h"
#include "fsl_qtmr.h"
#include "fsl_gpio.h"
#include "pin_mux.h"

#include "dvr8818.h"
#include "sensors.h"
#include "TPHMotor.h"
#include "developmentSettings.h"



#define takeupIntrHandler TMR2_IRQHandler
#define MAX_TENSION_MULTIPLIER 0.72  //successfuly ran 1.6 RFID rolls at  0.62 @800mA
#define MIN_TENSION_MULTIPLIER 0.70  //successfuly ran 1.6 RFID rolls at  0.60 @800mA

typedef enum
{
    STEP_TU,
    STEP_TPH,
    TIGHTEN_STOCK,
    LOOSEN_STOCK,
    BACKWIND,
    RAMP_MOTORS,
    RAMP_MAIN,
    RAMP_MAIN_QUARTER_STEPS,
    RAMP_TAKEUP,
    CHECK_FOR_PAPER,
    SIZE_LABELS,
    STEP_TO_LT,
    STEP_TO_NEXT_LABEL
}TUIntrType;

typedef enum
{
    QUARTER_STEP,
    HALF_STEP,
    FULL_STEP
}StepSizeEnum;



typedef enum
{
  HOLD_TIGHTEN_STEP_SPEED,
  RAMP_TO_STATIC_STEP,
  STATIC_STEP,
  PID_CONTROL,
  NORMAL_TU_MOTOR_INTR_EXIT,
  ABNORMAL_TU_MOTOR_INTR_EXIT,
  INVALID  
}tuControlMethodEnum;


/* stepTUMotorIntr() and support functions */
AT_QUICKACCESS_SECTION_CODE(static uint8_t stepTUMotorIntr( void )); //if setTmr2IntrType(STEP_TU), this function is called in takeupIntrHandler()
AT_QUICKACCESS_SECTION_CODE(unsigned short  calculateTUTorqueSlope(unsigned short torqueReading));
AT_QUICKACCESS_SECTION_CODE(void singleStepTUMotor(void));
AT_QUICKACCESS_SECTION_CODE(void controlDebugOutputPin(void));
AT_QUICKACCESS_SECTION_CODE(unsigned short calculateTUTorqueSetpoint(void));
AT_QUICKACCESS_SECTION_CODE(short calculatePIDControlTimeChange(short tuTensionSlope, unsigned short tuTorqReading));
AT_QUICKACCESS_SECTION_CODE(void toggleDebugPin(void));
AT_QUICKACCESS_SECTION_CODE(void toggleDebugPin10uS(void));
AT_QUICKACCESS_SECTION_CODE(void setTUSpeed(short changeToTUSpeed));
AT_QUICKACCESS_SECTION_CODE(void calculateAverageStepTime(void));


unsigned short getTUStepTimeRunningAvg(unsigned short prevTUAvgStepTime);
void shutDownStepTUMotorIntr(void);
void resetTUTorqueControlVars(void);

/* interrupt handlers */
AT_QUICKACCESS_SECTION_CODE(void takeupIntrHandler( void ));

/* private functions */
AT_QUICKACCESS_SECTION_CODE(static void setTmr2IntrType(TUIntrType intrType)); //sets the funtion that is called in takeupIntrHandler()

AT_QUICKACCESS_SECTION_CODE(static void startTakeupIntr( void ));
AT_QUICKACCESS_SECTION_CODE(void stopTakeupIntr( void ));
AT_QUICKACCESS_SECTION_CODE(void startTPHIntr( uint16_t TPHStepsInput ));
AT_QUICKACCESS_SECTION_CODE(void stopTPHIntr( void ));
AT_QUICKACCESS_SECTION_CODE(void startPrintLineIntr( void ));
AT_QUICKACCESS_SECTION_CODE(void stopPrintLineIntr( void ));




AT_QUICKACCESS_SECTION_CODE(static uint8_t stepTPHMotorIntr( void )); 
AT_QUICKACCESS_SECTION_CODE(static uint8_t sizeLabelIntr( void ));
AT_QUICKACCESS_SECTION_CODE(static uint8_t tightenStockIntr( void )); //if setTmr2IntrType(TIGHTEN_STOCK), this function is called in takeupIntrHandler()
AT_QUICKACCESS_SECTION_CODE(static uint8_t loosenStockIntr( void )); //if setTmr2IntrType(LOOSEN_STOCK), this function is called in takeupIntrHandler()
static uint8_t rampMotorsIntr( void );
static uint8_t rampMainMotorIntr( void );
static uint8_t rampMainMotorQuarterStepsIntr( void );
AT_QUICKACCESS_SECTION_CODE(static uint8_t backwindIntr( void ));
AT_QUICKACCESS_SECTION_CODE(static uint8_t checkForPaperIntr( void ));
AT_QUICKACCESS_SECTION_CODE(static uint8_t stepToLtIntr( void ));
AT_QUICKACCESS_SECTION_CODE(static uint8_t stepToNextLabelIntr( void ));

/* public functions */
AT_QUICKACCESS_SECTION_CODE(void initTakeupIntr( void )); //called in the functions below to init TMR2 interrupt handler function pointer
AT_QUICKACCESS_SECTION_CODE(void initTPHIntr( void )); //called in the functions below to init TMR2 interrupt handler function pointer
AT_QUICKACCESS_SECTION_CODE(void initPrintLineIntr( void ));
AT_QUICKACCESS_SECTION_CODE(void stepTUMotor( uint16_t steps, uint16_t speedInUs )); //call this function to step the takeup motor during print cycle
AT_QUICKACCESS_SECTION_CODE(void stepTPHMotor( uint16_t steps, uint16_t speedInUs )); 
AT_QUICKACCESS_SECTION_CODE(void tightenStock( uint16_t tension, uint16_t speedInUs, bool continueStepping, StepSizeEnum tuStepSize)); //call this function to tighten stock until the TU sensor reaches the value set in tension
AT_QUICKACCESS_SECTION_CODE(void loosenStock( uint16_t steps, uint16_t speedInUs )); //call this function to backwind takeup motor until steps taken == steps
AT_QUICKACCESS_SECTION_CODE(void backwindStock( uint16_t steps, uint16_t speedInUs ));
AT_QUICKACCESS_SECTION_CODE(void checkForPaper( uint16_t tension, uint16_t speedInUs ));
AT_QUICKACCESS_SECTION_CODE(void stepToLt( uint16_t steps, uint16_t speedInUs )); 
AT_QUICKACCESS_SECTION_CODE(void sizeLabels( uint16_t steps, uint16_t speedInUs ));
AT_QUICKACCESS_SECTION_CODE(void stepToNextLabel( uint16_t steps, uint16_t speedInUs ));

void rampMotors( uint16_t startSpeed, uint16_t endSpeed );
void rampMotorsBack( uint16_t startSpeed, uint16_t endSpeed );
void rampMainMotor( uint16_t startSpeed, uint16_t endSpeed );
void rampMainMotorQuarterSteps(uint16_t startSpeed, uint16_t endSpeed);

AT_QUICKACCESS_SECTION_CODE(void setTUSpeedModifier( uint16_t amountToSlowInUs )); //sets the amount to modify TUSpeed based on the amount of times it has been modified so far this print cycle
AT_QUICKACCESS_SECTION_CODE(uint16_t getTUSpeedModifier( void )); //sets the amount to modify TUSpeed based on the amount of times it has been modified so far this print cycle

AT_QUICKACCESS_SECTION_CODE(bool getTakeupBusy( void ));
AT_QUICKACCESS_SECTION_CODE(void setTakeupBusy( bool busy ));
AT_QUICKACCESS_SECTION_CODE(bool getTakingUpPaper( void ));
void setTakingUpPaper( bool paperState );
AT_QUICKACCESS_SECTION_CODE(int getLabelSizeInQuarterSteps( void ));
AT_QUICKACCESS_SECTION_CODE(void setLabelSizeInQuarterSteps( uint16_t labelSize ));
AT_QUICKACCESS_SECTION_CODE(int getStepsToNextLabel( void ));
AT_QUICKACCESS_SECTION_CODE(int getStepsBackToGap( void ));

AT_QUICKACCESS_SECTION_CODE(void takeupDelay( void ));
AT_QUICKACCESS_SECTION_CODE(void takeupDelayShort( void ));
AT_QUICKACCESS_SECTION_CODE(void takeupDelayMid( void ));

AT_QUICKACCESS_SECTION_CODE(void find_lowest_points(const int* waveform, int length, int threshold_value, int dip_threshold, int width_threshold));
AT_QUICKACCESS_SECTION_CODE(int find_lowest_points_lowest(const int* waveform, int length, int threshold_value));
AT_QUICKACCESS_SECTION_CODE(int find_lowest_points_start(const int* waveform, int length, int threshold_value));
AT_QUICKACCESS_SECTION_CODE(void print_waveform_csv_single_row(const char* label, const int* waveform, int length));
AT_QUICKACCESS_SECTION_CODE(void averageAndStore(int* array, int start, int end));
AT_QUICKACCESS_SECTION_CODE(double find_percentage_of_average(const int* array, int length, double percentage));
AT_QUICKACCESS_SECTION_CODE(uint16_t getLastSpeed( void ));
AT_QUICKACCESS_SECTION_CODE(void setLastSpeed( uint16_t speed ));
AT_QUICKACCESS_SECTION_CODE(int* getShootThroughBuffer( void));
uint8_t* getTensionBuffer( void );
uint8_t* getTightenBuffer( void );
uint8_t* getTensionSpeedBuffer( void );
uint8_t* getTightenSpeedBuffer( void );
AT_QUICKACCESS_SECTION_CODE(int getPrintDip( void ));
AT_QUICKACCESS_SECTION_CODE(int countDipsBelowThreshold(int waveform[], int length, int threshold));
AT_QUICKACCESS_SECTION_CODE(void condenseAppendAndResize(int firstArray[], int firstArrayLength, int secondArray[], int secondArrayLength));
int findDips(int waveform[], int length, double threshold);
AT_QUICKACCESS_SECTION_CODE(void shiftLeft(int arr[], int n, int shiftAmount));
AT_QUICKACCESS_SECTION_CODE(void addAndShift(int smaller[], int larger[], int smallerSize, int largerSize));
AT_QUICKACCESS_SECTION_CODE(int calculateAverage(int array[], int length));
uint16_t getLastTensionMeasurement(void);
AT_QUICKACCESS_SECTION_CODE(bool getReadyToRecordTakeupSteps(void));
AT_QUICKACCESS_SECTION_CODE(bool getLargeGapFlag(void));
uint16_t getTPHStepsThisPrint( void );
AT_QUICKACCESS_SECTION_CODE(bool getReadyToRecordShootVal( void ));
AT_QUICKACCESS_SECTION_CODE(void setReadyToRecordShootVal( bool ready ));
AT_QUICKACCESS_SECTION_CODE(bool getTPHIntrDone( void ));
AT_QUICKACCESS_SECTION_CODE(uint16_t getSecondGapIndex( void ));
AT_QUICKACCESS_SECTION_CODE(uint16_t getFirstGapIndex( void ));

#endif