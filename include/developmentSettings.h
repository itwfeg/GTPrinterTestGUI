#ifndef DEVELOPMENT_SETTINGS_H
#define DEVELOPMENT_SETTINGS_H

#include "takeupMotor.h"

/** Accelerometer Test/Deubg Defines **/
 //#define TFinkAlternateAngleCalc            //Alternate Angle calc - validated on granite table. Not as accurate as Randy's method.
 //#define TFinkWriteEEPDefaults     
 //#define TFinkForceVMConfigEnable  
 //#define TFinkStartCS5530woApp   //5530 is normally started when WG_REQ_SYSTEM_INFO msg is received from App. Restarting from debugger confuses App
 
#define PRINTFROMINTERRUPT
typedef enum printMessageList
{
   NO_PRINTF_MESSAGE,
   VSCOPE_INITIALIZED_MSG,
   VSCOPE_UNINITIALIZED_MSG,
   VSCOPE_FULL_MSG,
   VSCOPE_RECORDING_MSG,
   VSCOPE_STARTPRINTING_MSG,
   STEPTUMOTORINTR_TORQUE_OVERSHOOT,
   STEPTUMOTORINTR_HIGHTORQ_SLOWSTEP,
   STEPTUMOTORINTR_BAD_LAST_LABLEL_TIME,
   SWITCHED_TO_PID_DUE_TO_LOW_TORQUE,
   SWITCHED_TO_STATIC_CONTROL,
   TU_CONTROL_HOLD_TIGHTEN_STEP_SPEED,
   TU_CONTROL_RAMP_TO_STATIC_STEP,
   TU_CONTROL_STATIC_STEP,
   TU_CONTROL_PID_CONTROL,
   TU_CONTROL_NORMAL_TU_MOTOR_INTR_EXIT,
   TU_CONTROL_ABNORMAL_TU_MOTOR_INTR_EXIT,
   TU_CONTROL_UNKNOWN_STATE
      
} PrintMessageList;

//#define TFinkFullStep


void createPrintFromInterruptQueue(void);
void queuePrintStringFromISR(PrintMessageList stringIdentifier);
void printFromInterrupt(void);
void printLLAverageTime(void);
void sendLLAverageTimeToPrintf(unsigned short time);
void printStepsTakenTPHIntr(void);
void sendStepsTakenTPHIntrToPrintf(unsigned short stepsTaken);
void sendTorqeAdmustmentsToPrintf(short tightenAjust, unsigned short immPrintAdjust, unsigned short ltPrintAdjust, short tightenTorqOvershootAjustment);
void sendControlStateToPrintf(unsigned short state);
void queueControlState(void);







/******************************************************************************/
/*!   \fn PRINTFThrottle

      \brief
        Prints pString every "throttle" times its called.
        The purpose of this function is to prevent a particular PRINTF from 
        flooding the terminal. 

        This function is a #define because it uses static variables that can't
        be shared if the function is called by different functions

      \author
          Tom Fink
*******************************************************************************/
#define PRINTFThrottle(THROTTLE,STRING) {                                             \
                                           static unsigned short printfThrottle = 1;  \
                                           printfThrottle--;                          \
                                           if(!printfThrottle) {                      \
                                             PRINTF(STRING);                          \
                                             printfThrottle = (THROTTLE);             \
                                          }                                           \
                                        }




#endif