#ifndef SCA3300ACCEL_H
#define SCA3300ACCEL_H
#include "fsl_lpspi_freertos.h"
#include <stdbool.h>

/************************* Defines **************************************/
/* max spi frequency of the ic */
#define SCA3300_MAX_CLK_FREQ          8000000U        /* 8 Mhz */
#define SCA3300_BELOW_MAX_CLK_FREQ    4000000U        /* 4 Mhz */
#define SCA3300_DEVICE_ID             0x51
#define SCA3300_LSB_G                 5400            /* LSB/g */

/* device protocol ( note: values are swapped from datasheet p.17 */
/* r/w bit: 31, op bits: 30 - 26, rs bits: 25-24, data bits: 23 - 8, crc bits: 7-0 */

/*  operational commands ( 1 command per frame ) */
#define SCA_READ_REPLY              0x00000000

#define SCA3300_SPI_READY_TIMEOUT   10000 

/*******************************************************************************
                from data sheet p.17
#define SCA_CMD_READ_ACC_X          0x040000F7         
#define SCA_CMD_READ_ACC_Y          0x080000FD
#define SCA_CMD_READ_ACC_Z          0x0C0000FB
#define SCA_CMD_READ_STO            0x100000E9 
#define SCA_CMD_READ_TEMP           0x140000EF 
#define SCA_CMD_READ_STATUS         0x180000E5 
#define SCA_CMD_READ_CMD            0x340000DF
#define SCA_CMD_CHANGE_MODE1        0xB400001F
#define SCA_CMD_CHANGE_MODE2        0xB4000102
#define SCA_CMD_CHANGE_MODE3        0xB4000225
#define SCA_CMD_CHANGE_MODE4        0xB4000338
#define SCA_CMD_SET_POWER_DOWN      0xB400046B
#define SCA_CMD_WAKE_UP             0xB400001F
#define SCA_CMD_SW_RESET            0xB4002098 
#define SCA_CMD_WHOAMI              0x40000091
#define SCA_CMD_READ_SERIAL1        0x640000A7
#define SCA_CMD_READ_SERIAL2        0x680000AD
#define SCA_CMD_READ_CUR_BANK       0x7C0000B3
#define SCA_CMD_SWITCH_BANK0        0xFC000073
#define SCA_CMD_SWITCH_BANK1        0xFC00016E
********************************************************************************/

#define SCA_CMD_READ_ACC_X          0xF7000004
#define SCA_CMD_READ_ACC_Y          0xFD000008
#define SCA_CMD_READ_ACC_Z          0xFB00000C
#define SCA_CMD_READ_STO            0xE9000010
#define SCA_CMD_READ_TEMP           0xEF000014
#define SCA_CMD_READ_STATUS         0xE5000018
#define SCA_CMD_READ_CMD            0xDF000034    
#define SCA_CMD_CHANGE_MODE1        0x1F0000B4  
#define SCA_CMD_CHANGE_MODE2        0x020100B4
#define SCA_CMD_CHANGE_MODE3        0x250200B4
#define SCA_CMD_CHANGE_MODE4        0x380300B4
#define SCA_CMD_SET_POWER_DOWN      0x6B0400B4
#define SCA_CMD_WAKE_UP             0x1F0000B4
#define SCA_CMD_SW_RESET            0x982000B4
#define SCA_CMD_WHOAMI              0x91000040
#define SCA_CMD_READ_SERIAL1        0xA7000064
#define SCA_CMD_READ_SERIAL2        0xAD000068
#define SCA_CMD_READ_CUR_BANK       0xB300007C
#define SCA_CMD_SWITCH_BANK0        0x730000FC
#define SCA_CMD_SWITCH_BANK1        0x6E0100FC

/* "Return Status" masks */
#define SCA_OPC_MASK                0xFC000000
#define SCA_RS_MASK                 0x03000000
#define SCA_REPLY_DATA_MASK         0x00FFFF00
#define SCA_REPLY_CRC_MASK          0x000000FF

/* return status */
#define SCA_STARTUP_IN_PROGRESS     0x00
#define SCA_NORMAL_OP               0x01
#define SCA_RESERVED                0x02
#define SCA_ERROR                   0x03

/* status information masks */
#define SCA_UNUSED_STATUS_MASK      0x03FF
#define SCA_DIGI1_STATUS_MASK       0x0200
#define SCA_DIGI2_STATUS_MASK       0x0100
#define SCA_CLOCK_STATUS_MASK       0x0080
#define SCA_SAT_STATUS_MASK         0x0040
#define SCA_TEMP_STATUS_MASK        0x0020
#define SCA_POWER_STATUS_MASK       0x0010
#define SCA_MEM_STATUS_MASK         0x0008
#define SCA_DIGI3_STATUS_MASK       0x0004
#define SCA_MODE_CHANGE_STATUS_MASK 0x0002
#define SCA_PIN_STATUS_MASK         0x0001
                   
#define SCA3300_MAX_DATA_TRANFER    16      

/************************* ENUMs **************************************/

typedef enum 
{
    _MODE_1_3g,         /* 2700 LSB/g */
    _MODE_2_6g,         /* 1350 LSB/g */
    _MODE_3_15g,        /* 5400 LSB/g */ /* low pass filter 70Hz */
    _MODE_4_1r5g        /* 5400 LSB/g */ /* low pass filter 10Hz */
}SCA_MODE;


typedef enum
{
   SCA_X_AXIS_,
   SCA_Y_AXIS_,
   SCA_Z_AXIS_
}SCA_AXIS_TYPE;


/*********************** Structs **************************************/
typedef struct 
{
    unsigned char       chipId;
    unsigned short      selfTestOutput;
    short               xAccel;
    short               yAccel;
    short               zAccel;     
    unsigned short      temperature;    /* raw device temperature */
    float               deviceTemp;     /* device temperature °C */
    bool                vFrame;         /* frame valid */
    unsigned char       fOpCode;        /* frame operation code */
    unsigned char       dStatus;        /* device return status */   
    unsigned char       rCrc;           /* frame crc value */              
}SCA3300;


/* public prototypes */
bool getAccelSPIReady(void);
void setAccelSPIReadyFalse(void); 
bool initSCA3300( SCA3300 *pSca );
void getDeviceId( SCA3300 *pSca );
void getAcceleration( SCA3300 *pSca, SCA_AXIS_TYPE accel );
bool runSCASelfTest( SCA3300 *pSca );

/* private prototypes */
static void sca3300SpiCallBack( void );
static void delayTicks(TickType_t delayTicks);
static void negateChipSelectSCA3300( void );
static void assertChipSelectSCA3300( void );
static void valueMaxTask( void *pvParameters );
static void accelPwrOn( void );
static bool initSpiAccelerometer( void );
static void resetSca3300( SCA3300 *pSca );
static void changeMode( SCA3300 *pSca, SCA_MODE sensitivity );
static void getDeviceStatus( SCA3300 *pSca );
static void getTemperature( SCA3300 *pSca );
static void convertRawTemp( SCA3300 *pSca );
static void writeSCA3300(unsigned long cmd, unsigned char *pRxData, unsigned char size );
static unsigned short parseReply( SCA3300 *pSca, unsigned long *pRxData );
static unsigned char calcFrameCrc( SCA3300 *pSca, unsigned long frame);  
static unsigned char calculateSCA_CRC( unsigned long data );
static unsigned char calculateSCA_CRC8( unsigned char bitValue, unsigned char crc );
#endif
