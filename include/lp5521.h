#ifndef LP5521_H
#define LP5521_H

#include <stdbool.h>
#include "FreeRTOS.h"


#define LP5521_I2C_ADDR                 0x32

/* lp5521 register addresses */
#define _LP5521_EN_REG                  0x00
#define _LP5521_OP_MODE_REG             0x01

#if 0
#define _LP5521_RED_PWM_REG             0x02
#define _LP5521_GREEN_PWM_REG           0x03
#define _LP5521_BLUE_PWM_REG            0x04

#define _LP5521_RED_CUR_REG             0x05
#define _LP5521_GREEN_CUR_REG           0x06
#define _LP5521_BLUE_CUR_REG            0x07
#else
#define _LP5521_GAP_PWM_REG             0x02
#define _LP5521_LOW_STOCK_PWM_REG       0x04
#define _LP5521_PAPER_TAKE_UP_PWM_REG   0x03

#define _LP5521_GAP_CUR_REG             0x05
#define _LP5521_LOW_STOCK_CUR_REG       0x07
#define _LP5521_PAPER_TAKE_UP_CUR_REG   0x06
#endif

#define _LP5521_CFG_REG                 0x08

#define _LP5521_RED_PC_REG              0x09
#define _LP5521_GREEN_PC_REG            0x0A
#define _LP5521_BLUE_PC_REG             0x0B

#define _LP5521_STATUS_REG              0x0C
#define _LP5521_RESET_REG               0x0D  
#define _LP5521_GPIO_REG                0x0E

#define _LP5521_RED_SRAM_START_REG      0x10
#define _LP5521_RED_SRAM_END_REG        0x2F

#define _LP5521_GREEN_SRAM_START_REG    0x30
#define _LP5521_GREEN_SRAM_END_REG      0x4F

#define _LP5521_BLUE_SRAM_START_REG     0x50
#define _LP5521_BLUE_SRAM_END_REG       0x6F



/********* enable register bit settings *********/
/* chip enable bit 6 */
#define LP5521_STANDBY                  ( 0 << 6 )
#define LP5521_OPERATE                  ( 1 << 6 )

/* pwm adjustments bit 7 */
#define ADJ_LINEAR                      ( 0 << 7 )
#define ADJ_LOGARITHMATIC               ( 1 << 7 )
/************************************************/


/********* operation mode register bit settings */
/* blue channel mode bits 0:1 */
#define CH_B_DISABLED                   ( 0 << 0 )
#define CH_B_LOAD_PROG                  ( 1 << 0 )
#define CH_B_RUN_PROG                   ( 2 << 0 )
#define CH_B_DIRECT_CONTROL             ( 3 << 0 )

/* green channel mode bits 2:3 */
#define CH_G_DISABLED                   ( 0 << 2 )
#define CH_G_LOAD_PROG                  ( 1 << 2 )
#define CH_G_RUN_PROG                   ( 2 << 2 )
#define CH_G_DIRECT_CONTROL             ( 3 << 2 )

/* red channel mode bits 4:5 */
#define CH_R_DISABLED                   ( 0 << 4 )
#define CH_R_LOAD_PROG                  ( 1 << 4 )
#define CH_R_RUN_PROG                   ( 2 << 4 )
#define CH_R_DIRECT_CONTROL             ( 3 << 4 )
/************************************************/


/******** r,g,b pwm register bit settings *******/ 
/* pwm duty cycle bits 0:7 */
typedef enum
{
    DC_0_PRECENT,
    DC_5_PRECENT =                      0x05,
    DC_10_PRECENT =                     0x10,
    DC_25_PRECENT =                     0x3f,
    DC_50_PRECENT =                     0x7f,
    DC_75_PRECENT =                     0xbf,
    DC_90_PRECENT =                     0xe6,
    DC_97_PERCENT =                     0xFc,
    DC_100_PERCENT =                    0xff    
}PWMDutyCycle;
/************************************************/

/****** r,g,b current adj register bit settings */ 
/* const current values adjust from 0 - 25.5mA  */
typedef enum
{
   CC_Zero,
   CC_POINT_ONE,
   CC_POINT_TWO,    
   CC_POINT_THREE,    
   CC_POINT_FOUR,
   CC_POINT_FIVE,
   CC_POINT_SIX,
   CC_POINT_SEVEN,
   CC_POINT_EIGHT,
   CC_POINT_NINE,
   CC_ONE_POINT_ZERO                    = 10,
   CC_ONE_POINT_ONE,
   CC_ONE_POINT_TWO,
   CC_ONE_POINT_THREE,
   CC_ONE_POINT_FOUR,
   CC_ONE_POINT_FIVE,
   CC_TWO_POINT_ZERO                    = 20,
   CC_TWO_POINT_FOUR                    = 24,
   CC_THREE_POINT_ZERO                  = 30,
   CC_THREE_POINT_FIVE                  = 35,
   CC_THREE_POINT_SIX,
   CC_THREE_POINT_SEVEN,
   CC_THREE_POINT_EIGHT,
   CC_THREE_POINT_NINE,
   CC_FOUR_POINT_ZERO,
   CC_FOUR_POINT_ONE,
   CC_FIVE_POINT_ZERO                   = 50,
   CC_SIX_POINT_ZERO                    = 60,
   CC_SIX_POINT_FIVE                    = 65,
   CC_SEVEN_POINT_ZERO                  = 70,
   CC_EIGHT_POINT_ZERO                  = 80,
   CC_TEN_POINT_ZERO                    = 100,
   CC_TWELVE_POINT_EIGHT                = 128,
   CC_TEN_POINT_FIVE                    = 150,
   CC_SEVENTEEN_POINT_FIVE              = 175,
   CC_TWENTY_FIVE_POINT_ZERO            = 250,
   CC_TWENTY_FIVE_POINT_FIVE            = 255        
}CCSettings;
/************************************************/

/********* config register bit settings *********/
/* bits clock selection 0:1 */
#define CLK_EXTERNAL_32K                ( 0 << 0 )
#define CLK_INTERNAL                    ( 1 << 0 )
#define CLK_AUTO                        ( 2 << 0 )
#define CLK_INTERNAL_A                  ( 3 << 0 )

/* led source selection bit 2 */
#define LED_SOURCE_DEFAULT              ( 0 << 2 )
#define LED_SOURCE_BATTERY              ( 1 << 2 )

/* charge pump values bits 3:4 */
#define CP_OFF                          ( 0 << 3 )
#define CP_FORCED_BYPASS_MODE           ( 1 << 3 )
#define CP_FORCED_1_MODE                ( 2 << 3 )
#define CP_AUTO_MODE                    ( 3 << 3 )

/* power savings bit 5 */
#define PWR_SAVE_OFF                    ( 0 << 5 )
#define PWR_SAVE_ON                     ( 1 << 5 )

/* pwm clock frequency bit 6 */
#define PWM_256_FREQ                    ( 0 << 6 )
#define PWM_558_FREQ                    ( 1 << 6 )
/************************************************/


/********* status register bit settings *********/
#define BLUE_INTERRUPT_MASK              1
#define GREEN_INTERRUPT_MASK             2
#define RED_INTERRUPT_MASK               4
#define EXT_CLK_USED_MASK                8
/************************************************/


/********* reset register bit settings **********/
#define LP5521_DEVICE_RESET              255
/************************************************/

#define ONE_MILISECONDS         1000
#define FIVE_MILISECONDS        5000
#define TEN_MILISECONDS         10000

typedef struct
{
    unsigned char deviceAddr;
    unsigned char enableReg;
    unsigned char opModeReg;
    unsigned char configReg;
    PWMDutyCycle redPwmReg;
    PWMDutyCycle greenPwmReg;
    PWMDutyCycle bluePwmReg;
    CCSettings redCurrentReg;
    CCSettings greenCurrentReg;
    CCSettings blueCurrentReg;   
}LP5521CFG;


typedef enum
{
    _INIT,
    _CALBACKING,
    _CALLABEL,
    _CALRESULTS,
    _CALDONE
}GAPSteps;


typedef enum
{
    _INIT_TU_CAL,
    _CAL_MIN,
    _CAL_MAX,
    _CALRESULTS_TU,
    _CALDONE_TU
}TUSteps;

/************** public prototypes ***************/
bool initLp5521( void );
uint8_t readRegister( char slaveAdd, char reg );
void osc32InterruptHandler( void );
bool setGapCurrent( unsigned char value );
bool setLowStockCurrent( unsigned char value );
bool setLowStockDutyCycle( unsigned char value );
void shutdownGapSensor( void );
bool setPaperTakeupCurrent( unsigned char value );
bool gapSensorCal( GAPSteps step, unsigned char *pBias );
bool TUSensorCal( GAPSteps step, unsigned char *pBias );
void testlabelTaken( void );
bool getGapCalStatus( void );
bool getTUCalStatus( void );
/************************************************/

/************** private prototypes **************/
static void initializeI2C( void );
static void initExternalClock( void );
static bool configure( void );
static void configureAvery( LP5521CFG *pLP5521Cfg );
static void configureHobart( LP5521CFG *pLP5521Cfg );
bool increaseGapCurrent( void );
static bool increaseLowStockCurrent( void );
bool decrementGapCurrent( void );
static bool decreaseLowStockCurrent( void );
unsigned short getShootAverage( void );
static bool writeRegister( unsigned char reg, unsigned char data );
static void LP5521delay( void );
/************************************************/
#endif