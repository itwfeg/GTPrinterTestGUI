#ifndef PRINTENGINE_H
#define PRINTENGINE_H
#include "MIMXRT1024.h"
#include "printHead.h"
#include "fsl_common.h"
#include "commandTable.h"
#include "globalPrinter.h"
#include "prMessages.h"
#include "FreeRTOS.h"
#include "queue.h"
#include <stdbool.h>

typedef enum PrinterCommandOptions
{
   PrNoSecurityLabel,
   PrApplySecurityLabel,
   PrWaitForLabelTaken,
   PrStreamLabel      
}   PrinterCommandOptions_t;

#define PRINT_TIME                              25      /* 25.0 msec. */
#define STEP_TIME                               1       /*  1.0 msec. */
#define WAIT_TIME                               1       /*  2.0 msec. */
#define IDLE_TIME                               20      /* 20.0 msec. */  
#define DELAY_TIME                              20      /* 20.0 msec. */
#define CALIBRATION_TIME                        2       /*  2.0 msec. */
#define STEP_ISR_COUNT                          20      /* 20.0 msec. */
#define REVERSE_STEP_TIME                       2       /*  2.0 msec. */
#define FIRST_REVERSE_STEP_TIME                 2       /*  2.0 msec  */
#define SWITCH_TIME                             0       /*  0.0 msec  */

#define USEC_DELAY_COUNT                        17



#define printEngineISR                          (GPT1_IRQHandler)
#define ENGINE_TIMER_BASE                       GPT1       

#define ENGINE_TIMER_PRIORITY                   2
#define ENGINE_TIMER_COUNT                      (ENGINE_TIMER_BASE->CNT & 0x0000FFFF)       
#define ENGINE_TIMER_IRQ                        (GPT1_IRQn)
#define ENGINE_TIMER_PRESCALE                   (16U) //(64U)//(32U)//(1U)
#define DEFAULT_ENGINE_COUNT                    1476U     /* 1476U  current: changed from this to increase printspeed freestanding scale */                 

/*****************************************************************************
* The distance between the print line and the center of the media sensor.    *
*                2.914" / (0.00655"/step) = 445 steps                        *
* Now! Allow 10% of these steps as adustment after the sync is accomplished. *
*                         445 - 45 = 400                                     *
*****************************************************************************/
#define MEDIA_SENSOR_TO_PEEL_BAR                771  /*media sensor entry to peel bar 3.794" x 203.2 = 771*/ /* 3.661" = 744 lines @ 203.2 lines/inch */
#define BLACK_BAR_TO_EDGE_OF_LABEL               64  /* 5/16" = 63 lines @203.2 lines/inch */
#define ADJUSTMENT_FACTOR                        51  /* 1/4" - want label to stop short of expelling so "print position"
							 can adjust it (print position can only adust it forward */

#define MEDIA_SENSOR_TO_ALIGNMENT               (MEDIA_SENSOR_TO_PEEL_BAR - BLACK_BAR_TO_EDGE_OF_LABEL - ADJUSTMENT_FACTOR)

/* range from 1" to 5" */                        
#define  LOW_RANGE_START_SIZING                 203                                                                                            
#define  LOW_RANGE_END_SIZING                   1065
                                                /* apply this offset if sizing in low range */                        
#define  LOW_RANGE_OFFSET                       100     

                                                /* range from 5.5" to 7.5" */                                                                                                
#define  MID_RANGE_START_SIZING                 1066                                                                                            
#define  MID_RANGE_END_SIZING                   1573
                                                /* apply this offset if sizing in mid range */                        
#define  MID_RANGE_OFFSET                       80

                                                /* range from 8.0" to 9.5" */                                                                                                
#define  HIGH_RANGE_START_SIZING                1574                                                                                            
#define  HIGH_RANGE_END_SIZING                  2000
                                                /* apply this offset if sizing in mid range */                        
#define  HIGH_RANGE_OFFSET                      60
                                                /* 10" * 203 steps per inch */
#define CONTINUOUS_STOCK_MIN                    2030                                                                        

/* no defined in stepper driver */
typedef enum
{
    FORWARD_,
    BACKWARD_
}StepDir;
 

typedef enum
{
    FIRST_LEVEL_HIST,
    SECOND_LEVEL_HIST,
    FIRST_LEVEL_ADJ,
    SECOND_LEVEL_ADJ,
    CURRENT_LINE,
    UNKNOWN_COMPENSATION_TYPE
}HistAdjType;



typedef struct hist_adj_level_struct
{
    unsigned long      *pLinePointer;     /* PH Data, Source Address Low Word */
    unsigned short     time;
    HistAdjType compType;
}HistAdj;

#define MAX_HIST_ADJ_LEVELS           3

//#define lineTimerIsr                            (GPT2_IRQHandler)
//#define LINE_PRINTER_TIMER_BASE                 (GPT2) 
//#define LINE_PRINTER_TIMER_PRIORITY             1        /* was 2 */   
//#define LINE_PRINTER_TIMER_COUNT                (GPT2->CNT & 0x0000FFFF)       
//#define LINE_PRINTER_TIMER_IRQ                  (GPT2_IRQn)



#define STROBE_FTM_BASE                       (FTM0)   
#define STROBE_PWM_Hz                         (25265U)          /* 25.265Khz */        
#define STROBE_PWM_PERIOD                     ( 1000000000U / STROBE_PWM_Hz ) 
#define STROBE_FTM_CHANNEL                    (kFTM_Chnl_3)
#define STROBE_PWM_DEFAULT_DUTY_CYCLE         (50U)             /* [%] */
#define STROBE_PWM_POLARITY                   (kFTM_HighTrue)
#define STROBE_PWM_INTERRUPT_MASK             (kFTM_Chnl3InterruptEnable)
#define STROBE_PWM_ALIGNMENT                  (kFTM_EdgeAlignedPwm)
#define STROBE_FTM_SOURCE_CLOCK               CLOCK_GetFreq(kCLOCK_BusClk)
 

#define PRECENT_MS_CAL                        3 

#define EDGE_MARGIN     0
#define BMP_PITCH       16
#define BAR_HEIGHT      5


typedef struct
{
    CmdOp currentCmd;
    
   /* The following three variables are used to select which Hist/Adj calculations
      are done. This is done so that only the types
      that are being used will be calculated (which saves lots of processing time).
      There is no 1st history variable, because it is always calculated. */
  
   bool                 calc2ndHistory;
   bool                 calc1stAdjacency;
   bool                 calc2ndAdjacency;
   bool                 pause;

   unsigned short       contrast;
   HEADTYPE             headType;
   unsigned char        levels;                     /* number of levels of compensation. i.e. 1st level history, 2nd level adjacency, and
                                                       2nd level history = 3 levels of PH compensation */
   unsigned long        lineCounter;                /* counts the Print Lines as they're loaded */
   unsigned long        lineCounter2;               /* used when label image is greater than 5 " */
   unsigned char        burnSequence;               /* tracks where we are in the line burn sequence   */
   unsigned short       pwmStartTime;               /* time that PWMing is started */
   unsigned char        pwmDutyCycle;
   unsigned short       sltTime;                    /* overall line printing time */
   unsigned short       sltHalfTime;                /* half of the SLT time  */
   bool                 linePrintDone;
   HistAdj              histAdj[MAX_HIST_ADJ_LEVELS];
   unsigned char        *pHistory;                   /* history for current print line */
   unsigned char        *pImage;                    /* print image buffer */
   signed short         numSteps;                   /* number of steps for the motor */ 
   StepDir              direction;                  /* direction of the stepper motor */
   signed short         numPrintLines;		    /* number of print lines */
   signed short         totalLinesToPrint;          /* total number of lines to print label */
   unsigned char        labelOrientation;	    /* head first or heel first */
   signed short         steps;                      /* number of steps for the step operations */
   unsigned short       outOfMediaCnt;              /* number of steps past media */
   unsigned short       maxMediaCount;              /* configuration value for cntr compare */
   unsigned short       stepsOffset;                /* number of steps used to determine sync bar */
   signed short         labelTracking;              /* number of steps to correct next label */
}PrintEngine;





AT_QUICKACCESS_SECTION_CODE( void initializePrintEngine( unsigned int contrast, unsigned int mediaCount, QueueHandle_t pHandle ) );
AT_QUICKACCESS_SECTION_CODE(unsigned short getNumPrintLinesLeft(void));
void addCmdToQueue( PrCommand *pCmd );
void setSkipLabelTakenCheck( void );
void startPrintEngine( void );
AT_QUICKACCESS_SECTION_CODE( void setLineTimerIntLevel( unsigned int level ) );
AT_QUICKACCESS_SECTION_CODE( void startLineTimer( bool start ) );
AT_QUICKACCESS_SECTION_CODE( void stopLineTimer( void ) );
AT_QUICKACCESS_SECTION_CODE( void stopLineTimer( void ) );
//AT_QUICKACCESS_SECTION_CODE( void shutdownPrintEngine( void ) );
void shutdownPrintEngine( void );
AT_QUICKACCESS_SECTION_CODE( void printHeadPowerOff( void ) );
AT_QUICKACCESS_SECTION_CODE( void printHeadPowerOn( void ) );
AT_QUICKACCESS_SECTION_CODE( void initializeStepper( StepDir direction ) );
AT_QUICKACCESS_SECTION_CODE( void powerOffStepper( void ) );
AT_QUICKACCESS_SECTION_CODE( void powerOnStepper( void ) );
AT_QUICKACCESS_SECTION_CODE( void setStepperDirection( StepDir direction ) );
AT_QUICKACCESS_SECTION_CODE( void halfStepMotor( void ) );
AT_QUICKACCESS_SECTION_CODE( void motorStep( StepDir dir, PrStatusInfo *pStatus ) );
AT_QUICKACCESS_SECTION_CODE( void motorStepFast( PrStatusInfo *pStatus ) );
AT_QUICKACCESS_SECTION_CODE( void initializePrintEngineTimer( uint16_t period_us ) );
AT_QUICKACCESS_SECTION_CODE( void setPrintEngineTimerSlt( void ) );
AT_QUICKACCESS_SECTION_CODE( void setPrintEngineTimer( unsigned short time ) );
AT_QUICKACCESS_SECTION_CODE( void setEngineContrast( unsigned short contrast ) );
AT_QUICKACCESS_SECTION_CODE( void resetPrintEngineTimer( void ) );
AT_QUICKACCESS_SECTION_CODE( void resetEngine( void ) );
AT_QUICKACCESS_SECTION_CODE( void pauseEngine( void ) );
AT_QUICKACCESS_SECTION_CODE( void initializePrintHeadPwm( void ) );
AT_QUICKACCESS_SECTION_CODE( PrintEngine *getPrintEngine( void ) );
AT_QUICKACCESS_SECTION_CODE( bool isEnginePaused() );
AT_QUICKACCESS_SECTION_CODE( unsigned long getPrintEngineLineCntr( void ) );
AT_QUICKACCESS_SECTION_CODE( void historyAdjacency( void ) );
AT_QUICKACCESS_SECTION_CODE( void loadHistory( void ) );
AT_QUICKACCESS_SECTION_CODE( void loadPrintLine( void ) );
AT_QUICKACCESS_SECTION_CODE( void loadZeroPrintLine( void ) );
AT_QUICKACCESS_SECTION_CODE( bool isCurrentLine( void ) );
AT_QUICKACCESS_SECTION_CODE( void clearBurnSequence( void ) );
AT_QUICKACCESS_SECTION_CODE( void clearPrevVertOffset( void ) );
AT_QUICKACCESS_SECTION_CODE( void lineTimerStrobe( void ) );
AT_QUICKACCESS_SECTION_CODE( void compareStatus( PrStatusInfo *pCurrent, PrStatusInfo *pPrevoius ) );
AT_QUICKACCESS_SECTION_CODE( bool testCondition( PrStatusInfo *pStatus, TestOperator oper, unsigned char bits, unsigned char result ) );
AT_QUICKACCESS_SECTION_CODE( void calibratePrinter( PrinterCal cal ) );
AT_QUICKACCESS_SECTION_CODE(void createCheckerBoardLabel( unsigned char offset, unsigned long length ) );
AT_QUICKACCESS_SECTION_CODE(void createVerticalLinesLabel( unsigned char offset, unsigned long length ) );
AT_QUICKACCESS_SECTION_CODE(void createSingleVerticalLineLabel(  unsigned char offset, unsigned long length ) );
AT_QUICKACCESS_SECTION_CODE(void createHorizontalLinesLabel( unsigned char offset, unsigned long length ) );
AT_QUICKACCESS_SECTION_CODE( void bitSet( unsigned short startBit, unsigned short numBits, unsigned char *pBuffer ) );
AT_QUICKACCESS_SECTION_CODE( int getLabelSize() );
AT_QUICKACCESS_SECTION_CODE( int getOutOfMediaCounts() );
AT_QUICKACCESS_SECTION_CODE( int getMaxOutOfMediaCounts() );
AT_QUICKACCESS_SECTION_CODE( void setOutOfMediaCounts(int val) );
AT_QUICKACCESS_SECTION_CODE( void lineTimerBurn( void ) );
AT_QUICKACCESS_SECTION_CODE( void lineTimerSLT( void ) ); 

AT_QUICKACCESS_SECTION_CODE( void idleOp( void ) );
AT_QUICKACCESS_SECTION_CODE( void printOp( CmdOp *pOperation ) );
void stepOp( StepOperation *pOperation );
void stepUntilOp( StepUntilOperation *pOperation );
AT_QUICKACCESS_SECTION_CODE( void stepGapOp( StepUntilOperation *pOperation ) );
void stepTakeupOp( StepUntilOperation *pOperation );
void stepEdgeOp( StepUntilOperation *pOperation );
void testForSyncOp( StepOperation *pOperation );
void testForLabelOp( StepOperation *pOperation );
void testForContinuous( StepOperation *pOperation );
void stepTakeupTightenOp( StepOperation *pOperation );
void reverseStepOp( StepOperation *pOperation );
AT_QUICKACCESS_SECTION_CODE( void waitOp( WaitOperation *pOperation ) );
AT_QUICKACCESS_SECTION_CODE( void waitUntilOp( WaitUntilOperation *pOperation ) );
void waitUntilSizingOp( WaitUntilOperation *pOperation );
void testOp( TestOperation *pOperation );
AT_QUICKACCESS_SECTION_CODE( void statusOp( StatusOperation *pOperation ) );
AT_QUICKACCESS_SECTION_CODE( void counterOp( CounterOperation *pOperation ) );
void calibrateOp( CmdOp *pOperation );
void disableOp( CmdOp *pOperation );
AT_QUICKACCESS_SECTION_CODE( void clearCmdQueue( void ) );
AT_QUICKACCESS_SECTION_CODE( void clearLabelImageBuffer( void ) );
void printerTests( void );
AT_QUICKACCESS_SECTION_CODE( void cutOp( GenericOperation *pOperation ) );
AT_QUICKACCESS_SECTION_CODE( void printDotWearOp( CmdOp *pOperation ) );
AT_QUICKACCESS_SECTION_CODE( void setHistoryEnabled(bool enabled) );
void detectionOp( StepUntilOperation *pOperation ); 
AT_QUICKACCESS_SECTION_CODE( int calculateSizingOffset( unsigned int wSteps ) ); 
AT_QUICKACCESS_SECTION_CODE( void setContinuousStock( void ) );
AT_QUICKACCESS_SECTION_CODE( void clrContinuousStock( void ) );
AT_QUICKACCESS_SECTION_CODE( bool getIDF2( void) );

bool getLabelPositionedAfterSizing( void );
bool getUsingContinuous(void);
bool getSizingLabels(void);
void setSizingLabels(bool sizing);
int getBackwindOffset( void );
bool getBackwindAfterSizing( void );
void setBackwindAfterSizing( bool backwind );
uint32_t getLTWaitCount_( void );
void setLTWaitCount_( uint32_t waitCount );
bool getExpelDone( void );
int* getLabelLowHistoryBuffer( void);
int getShootIndex( void );
void setShootIndex( int index );

uint16_t calculateSizingBackwindSteps( void );
uint16_t calculateStreamingBackwindSteps( void );
uint16_t calculatePeelingBackwindSteps( void );

#endif