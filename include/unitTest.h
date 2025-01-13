#ifndef UNITTESTS_H
#define UNITTESTS_H
#include "MIMXRT1024.h"
#include "fsl_lpspi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdbool.h>

#define uTest_task_PRIORITY ( configMAX_PRIORITIES -1 )

typedef enum
{
    _AUTO,
    _MANUAL
    
}ADCMode;

typedef enum
{
    _UT_INIT,
    _UT_SERIAL,
    _UT_WAIT_SERIAL,
    _UT_END_SERIAL
}UTests;

typedef enum
{
     _UT_SUB_INIT,
     _UT_CHECKSUM_READ,
     _UT_CHECKSUM_WRITE,
     _UT_PRINTER_CFG_READ,
     _UT_PRINTER_CFG_WRITE,
     _UT_WEIGHER_CFG_READ,
     _UT_WEIGHER_CFG_WRITE,
}UTSubTst;


BaseType_t createUnitTest( UTests test);
static void unitTestTask( void *pvParameters );

void testPaperTakeup( void );

void testADC2Auto( void );

void print3InchLabel( void );
void testSizing( void );
void testGapDetection( void );
void torqueTest( void );

void testCSADCInt( void );
void testStrobePWM( void );
void initEdma( void );
void testPrintHeadTransfer( void );
void testPrintEngineTimer( void );
void testWeigherADC( void );
void testADCAuto( void );
void testInitADC2( void );
void testInitEtcAdc( void );
void testInitXbar( void );
void testinitPitADC( void );
void testPWMWeigher( void );
void testWeigherSpi( void );
void startWeigherClock( void );
void intializeSpiForWeigher( void );
void lpspiWeigherCallback( LPSPI_Type *base, lpspi_master_handle_t *handle, 
                           status_t status, void *userData );
void testI2CLP5521( void );
void testsca3300( void );
#endif
