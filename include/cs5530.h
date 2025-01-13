#ifndef CS5530_H
#define CS5530_H
#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "timers.h"



/* command bytes */
#define CS5530_NULL                     0x00
#define WRITE_OFFSET_REGISTER           0x01
#define READ_OFFSET_REGISTER            0x09
#define WRITE_GAIN_REGISTER             0x02
#define READ_GAIN_REGISTER              0x0A
#define WRITE_CFG_REGISTER              0x03
#define READ_CFG_REGISTER               0x0B
#define PERFORM_SINGLE_CONVERSION       0x80
#define PERFORM_MULT_CONVERSION         0xC0
#define PERFORM_OFFSET_CALIBRATION      0x85
#define PERFORM_GAIN_CALIBRATION        0x86
#define SYNC0                           0xFE
#define SYNC1                           0xFF


/*                      config register 
 * [ PSS ] [ PDW ] [ RS ] [ RV ] [ IS ] [ NU ] [ VRS ] [ A1 ]
 *   b31                                                b24
 * [ A0 ] [ NU ] [ NU ] [ NU ] [ FRS ] [ NU ] [ NU ] [ NU ]
 *   b23                                               b16
 * [ NU ] [ WR3 ] [ WR2 ] [ WR1 ] [ WR0 ] [ UP/BP ] [ OCD ] [ NU ]
 *   b15                                                      b8
 * [ NU ] [ NU ] [ NU ] [ NU ] [ NU ] [ NU ] [ NU ] [ NU ]
 *   b7                                               b0
 *      See page 19 of data sheet for bit definitions
 *      PSS: power save select
 *      PDW: power down 
 *      RS: soft reset
 *      RV: reset valid
 *      IS: input short
 *      NU: not used
 *      VRS: voltage reference select
 *      A1-A0 output latch bits
 *      FRS: filter rate select
 *      WR3-WR0: conversion rates
 *      U/B: unipolar bipolar
 *      OCD: open circuit detect 
*/

/* configuration bits */
#define CS5530_PSS_SLEEP                0x80000000
#define CS5530_PDW_POWER_SAVE           0x40000000
#define CS5530_SOFT_RESET               0x20000000
#define CS5530_RESET_VALID              0x10
#define CS5530_INPUT_SHORT              0x08000000
#define CS5530_VREF_SELECT              0x02000000      /* 1v <= Vref <= 2.5v */
#define CS5530_FRS_SCALE5_6             0x00200000
#define CS5530_120SPS_RATE              0x00000000      /* default */
#define CS5530_60SPS_RATE               0x00000800      /* note: ad7191 on ARK */  
#define CS5530_1920SPS_RATE             0x00000900      /* note: ad7191 on ARK */  
#define CS5530_30SPS_RATE               0x00001000      /* is set for 60Hz */
#define CS5530_7_5SPS_RATE              0x00002000      
#define CS5530_15SPS_RATE               0x00001800     
#define CS5530_UNI_MODE                 0x00000400
#define CS5530_BIPOLAR_MODE             0x00000000
#define CS5530_OPEN_DETECT              0x00000200

/* transfer sizes */
#define SYNC_XFER_SIZE                  16
#define RESET_XFER_SIZE                 5               /* command + cfg register */
#define READ_CFG_XFER_SIZE              5               /* command + cfg register */
#define WRITE_CFG_XFER_SIZE             5               /* command + cfg register */
#define READ_CONV_XFER_SIZE             5               /* clear flag + conversion */
/* interface clk frequency */
#define CS5530_MAX_CLK_FREQ             2000000U        /* 2 Mhz */

#define MAX_CS5530_PAYLOAD              20

/* conversion complete interrupt */
#define weigherDataRdyIsr               GPIO2_Combined_0_15_IRQHandler

#define WEIGHER_DATA_RDY_PRIORITY       7
#define WEIGHER_SPI_INT_PRIORITY        7
/* command types */
typedef enum
{
    _CS5530_No_Command,
    _Write_Offset_Reg,
    _Read_Offset_Reg,    
    _Write_Gain_Reg,
    _Read_Gain_Reg,
    _Write_Cfg_Reg,
    _Read_Cfg_Reg,
    _Perform_Single_Conv,
    _Perform_Mult_Conv,
    _Perform_Offset_Cal,
    _Perform_Gain_Cal,
    _Sync_Interface,
}CS5530CMDS;
    
typedef struct 
{
    unsigned long       offsetRegister;
    unsigned long       gainRegister;
    unsigned long       configRegister;
    CS5530CMDS          currentCmd;
    unsigned char       txBfr[MAX_CS5530_PAYLOAD];
    unsigned char       rxBfr[MAX_CS5530_PAYLOAD];  
    unsigned long       conversion;
    bool                spiReady;        
}CS5530Mgr;

typedef enum
{
    _0HZ,
    _15HZ  = 15,
    _30HZ  = 30,
    _60HZ  = 60,
    _100HZ = 100,
    _120HZ = 120
}CS5530PollFreq;

#define CS5530_RATE _30HZ

/* public functions */
bool initializeCs5530( QueueHandle_t adQueue );
void buildSyncTransfer( CS5530Mgr *pMgr );
void buildResetTransfer( CS5530Mgr *pMgr );
void buildReleaseRstTransfer( CS5530Mgr *pMgr );
void cs5530SpiCallBack( void );
bool startcs5530PollTimer( CS5530PollFreq freq );
bool startCSWaitTimer( unsigned long mSec );
void csWaitCallBack( TimerHandle_t cs5530WaitTimer_ );
void csPollTimerCallBack( TimerHandle_t cs5530PollTimer_  );
void buildConvTransfer( CS5530Mgr *pMgr );
void setConversionInterrupt( void );
/* private functions */

static void cs5530StartClk( void );
static void initSpiInterface( void );
static void removeConversionInterrupt( void );
static bool cs5530ReadConversion( CS5530Mgr *pMgr );
bool isResetValid( CS5530Mgr *pMgr );
static bool cs5530ReadCfgResiter( CS5530Mgr *pMgr );
static bool cs5530SyncSerialPort( CS5530Mgr *pMgr );
static bool cs5530Reset( CS5530Mgr *pMgr );
static bool cs5530ConvConfig( CS5530Mgr *pMgr );
static void assertCS5530( void );
static void releaseCS5530( void );
static bool writeCS5530( unsigned char *pTxData, unsigned char *pRxData, unsigned char size );
static void handleTransfer( CS5530Mgr *pMgr );
static bool cs5530ContConversion( CS5530Mgr *pMgr );
#endif