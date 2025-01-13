#ifndef DRV8818_H
#define DRV8818_H

#include "fsl_common.h"
#include "MIMXRT1024.h"


typedef enum
{
   _MAIN_STEPPER,
   _TAKEUP_STEPPER    
}DVR8818Type;

typedef enum
{
    FORWARDM_,
    BACKWARDM_
}StepDirM;

void initializeMotors( StepDirM direction );
AT_QUICKACCESS_SECTION_CODE(void setMainMotorDirection( StepDirM direction ));
AT_QUICKACCESS_SECTION_CODE(void setTakeUpMotorDirection( StepDirM direction ));
AT_QUICKACCESS_SECTION_CODE(void powerOnMotors( void ));
AT_QUICKACCESS_SECTION_CODE(void powerOffMotors( void ));
AT_QUICKACCESS_SECTION_CODE(void stepMotors( void ));
AT_QUICKACCESS_SECTION_CODE(void rampStepMotors( void ));
AT_QUICKACCESS_SECTION_CODE(void stepMainMotor( void ));
AT_QUICKACCESS_SECTION_CODE(void stepTakeUpMotor( void ));
void unitTestMotorsIO( void );
void unitTestSpinMotors( void );


AT_QUICKACCESS_SECTION_CODE(static void setStepDirection( DVR8818Type type, StepDirM dir )); 
static void initIODRV8818( DVR8818Type type );
AT_QUICKACCESS_SECTION_CODE(void setFullStepMode( DVR8818Type type ));
AT_QUICKACCESS_SECTION_CODE(void setHalfStepMode( DVR8818Type type ));
AT_QUICKACCESS_SECTION_CODE(void setQuarterStepMode( DVR8818Type type ));
AT_QUICKACCESS_SECTION_CODE(void setEighthStepMode( DVR8818Type type ));
void sleepMotor( DVR8818Type type );
void wakeupMotor( DVR8818Type type );
static void releaseFromReset( DVR8818Type type );
static void resetMotor(  DVR8818Type type );
bool getMotorPowerStatus( void );
#endif