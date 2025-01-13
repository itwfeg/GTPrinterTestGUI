#ifndef TPHMOTOR_H
#define TPHMOTOR_H

#include "fsl_debug_console.h"
#include "fsl_qtmr.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "fsl_gpt.h"

#include "dvr8818.h"
#include "sensors.h"
#include "printEngine.h"
#include "dotWearTask.h"
#include "takeupMotor.h"
#include "lp5521.h"


#define MotorIntrGPT2                           (GPT2_IRQHandler)
#define LINE_PRINTER_TIMER_BASE                 (GPT2) 
#define LINE_PRINTER_TIMER_PRIORITY             1        /* was 2 */   
#define LINE_PRINTER_TIMER_COUNT                (GPT2->CNT & 0x0000FFFF)       
#define LINE_PRINTER_TIMER_IRQ                  (GPT2_IRQn)


typedef enum
{
    STEP_TPH_GPT2,
    STEP_TU_GPT2,
    BURN_LINE
}TPHIntrType;


AT_QUICKACCESS_SECTION_CODE(uint8_t lineTimerIntr( void ));
AT_QUICKACCESS_SECTION_CODE(uint8_t stepTPHMotorIntrGPT2( void ));
AT_QUICKACCESS_SECTION_CODE(uint8_t stepTUMotorIntrGPT2( void ));

AT_QUICKACCESS_SECTION_CODE(void setGPT2MotorIntLevel( unsigned int level ));
AT_QUICKACCESS_SECTION_CODE(void setGPT2IntrType(TPHIntrType intrType ));

AT_QUICKACCESS_SECTION_CODE(void startTPHMotorIntrGPT2( uint16_t steps, uint32_t speed ));
AT_QUICKACCESS_SECTION_CODE(void startTUMotorIntrGPT2( uint16_t steps, uint32_t speed ));
AT_QUICKACCESS_SECTION_CODE(void stopTPHMotorIntrGPT2( void ));

AT_QUICKACCESS_SECTION_CODE(void setRampSpeed(uint16_t speed));
AT_QUICKACCESS_SECTION_CODE(void setStartTUOnce(bool start));
AT_QUICKACCESS_SECTION_CODE(void setTensionModifier( uint16_t counts ));
AT_QUICKACCESS_SECTION_CODE(uint16_t getTensionModifier( void ));

#endif