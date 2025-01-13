#ifndef TRANSLATOR_H
#define TRANSLATOR_H
#include "prMessages.h"
#include "wgMessages.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdbool.h>


#define translator_task_PRIORITY ( configMAX_PRIORITIES -1 )

typedef struct
{
    unsigned int msgSize; 
    bool handlingMsg;
}TransMSG;

/* transaction weigher config container */
//typedef struct 
//{
//    TransMSG    tracker;
//    WgCfg       config;    
//}TWGCfgMsg;

/* this list must be same as in backend */
typedef enum
{
    UNKNOWNTYPE_,
    G_WEIGHER_,
    G_PRINTER_,
    G_DBOOT_,
    G_WRAPPER_,
    G_CLA_           
}HDeviceType;

/* LEGACY_PROTOCOL describes the USB messaging communication scheme which is made up of a header packet followed by a data packet. 
   This is the messaging format used on legacy peripherals, Atmel based.*/
#define LEGACY_PROTOCOL                         1       
/* GLOBAL_PROTOCOL omits the use of header packet and only sends data packet */
/* #define GLOBAL_PROTOCOL                      1 */


#pragma pack( 1 )
#ifdef GLOBAL_PROTOCOL
typedef struct
{
    HDeviceType         device;
    unsigned char       stationId;              /* unique system id (ex. first printer, second printer ) */
    unsigned char       msgType;
    unsigned long       msgSize;                /* message body size */
    unsigned char       frameNumber;            /* current frame number */
    unsigned char       numberOfFrames;         /* total frames needed to complete the message */
    unsigned short      lastFrameSize;          /* total bytes in the last frame */
}USBHeader;

typedef struct
{
    USBHeader hdr;    
    unsigned char *pMsg;
}HUSBMessage;

#else 
typedef struct
{
    unsigned short       sourcePhysAddr;         /* unused */
    unsigned short       sourceLogicalAddr;
    unsigned short       DestPhysAddr;           /* unused */
    unsigned short       DestLogicalAddr;
    unsigned short       msgSize;        
}USBHeader;

typedef union
{
    PrMessage prMsg;
    WgMessage wgMsg;    
}HBMessage;

typedef struct
{
    USBHeader hdr;    
    unsigned char body[ sizeof(HBMessage) ];
}HUSBMessage;

#define LGCY_LOGICAL_ADDR_PRNTR                 0x24
#define LGCY_LOGICAL_ADDR_GPRNTR                0x25
#define LGCY_LOGICAL_ADDR_WGR                   0x06
#define LGCY_LOGICAL_ADDR_GWGR                  0x66
#endif
#pragma pack( push,1 )
#pragma pack( pop )

#define MAX_USB_INT_FRAME_SIZE                  64
#define MAX_USB_FRAME_SIZE                      512     /* this must be the same size af the pipe! */
/* public functions */
BaseType_t createTranslator( QueueHandle_t wgMsgQueue, 
                             QueueHandle_t prMsgQueue, 
                             QueueHandle_t UsbInPrMsgQueue,
                             QueueHandle_t UsbInWrMsgQueue,
                             QueueHandle_t UsbOutPrMsgQueue, 
                             QueueHandle_t UsbOutWrMsgQueue);


/* printer messages */
void sendPrWakeup( PrWakeup *pWakeMsg);
void sendFakePrWakeup( void );
void sendPrSysInfo( PrSysInfo *pSysInfo );
void sendPrConfig( Pr_Config *pConfig );
void sendPrStatus( PrStatusInfo *pStatus, bool interrupt );
void sendPrSensors( PrSensors *pSensors );
void sendPrVersion( PrVersion *pVersion );
void sendPrHeadType( PrHead *pType );
void sendPrCutterStatus( PrCutterStatus *pStatus );
BaseType_t sendPrCutterStatusFromISR( PrCutterStatus *pStatus );
void sendPrTransferReady( bool ready );
void sendPrFactoryDlftsComplete( bool status );
void sendPrHeadDotStatus( PrDotStatus stat );
void sendPrGapCalStatus( PrGapCalStatus *pPrGapCalStatus );
void sendPrTUCalStatus( PrGapCalStatus *pPrGapCalStatus );
void sendDotWear( int size );
void sendTakeLabelError( bool error );



/* weigher messages */
void sendWgWakeup( WgWakeup *pWakeMsg );
void sendWgSysInfo( WgSysInfo *pSysInfo );
void sendWgStatus( WgStatus *pStatusMsg );
void sendWgConfig( WgCfg *pConfigMsg );
void sendWgSystemInfo( WgInfo *pInfo );
void sendWgVersion( WgVersion *pVersion );
void sendCat3Statistics( WgCat3Statistics *pStatsMsg );
void sendCat3Records( WgCat3Records *pRecordsMsg );
void sendCat3RecordStatus( WgCat3RecordStatus *pStatusMsg );
void sendCat3LogEraseComplete( void );
void sendWgVmAssembly( WgVMWrAssembly *pVmAssembly );
void sendWgVmVersion( WgVMWrVersion *pVmVersion );
void sendWgVmSerial( WgVMWrSerial *pVmSerial );
void sendWgVmDate( WgVMWrDate *pVmDate );
void sendWgVmXGain( WgVMWrOffsetGain *pXFactor );
void sendWgVmYGain( WgVMWrOffsetGain *pYFactor );
void sendWgVmZGain( WgVMWrOffsetGain *pZFactor );
void sendWgVmXZeroRef( float *zero_ref );
void sendWgVmYZeroRef( float *zero_ref );
void sendWgVmZZeroRef( float *zero_ref );
void sendWgCurrentGains( WgVMCurrent *pCurrent );
void sendWgFactoryDlftsComplete( bool status );
void sendWgVMFitValues( fit_flash_section *fitValues );


/* private functions */
static void translatorTask( void *pvParameters );
static void translatePrinterMessage( unsigned char *pBfr );
static void translateWeigherMessage( unsigned char *pBfr );
static unsigned int translateFrameHdr( unsigned char *pBfr, USBHeader *pHdr );
static bool isWeigherMessage( HDeviceType type );
static bool isPrinterMessage( HDeviceType type );
#ifdef LEGACY_PROTOCOL
static void t_handleWeigherMessage( unsigned char *pMsg_, unsigned int msgSize );
static void t_handlePrinterMessage( unsigned char *pMsg_, unsigned int msgSize );
//static void initHeader( USBHeader *pHdr, HDeviceType type, unsigned short logicalAddr, int size );
#else
static void t_handleWeigherMessage( HUSBMessage *pMsg_ );
static void t_handlePrinterMessage( HUSBMessage *pMsg_ );
#endif
/* handle printer messages */
static void t_handlePrReqWakeup( void );
static void t_handlePrWriteConfig( unsigned char *pFrame );
static void t_handlePrReqConfig( void );
static void t_handlePrReqStatus( void );
static void t_handlePrReqSensors( void );
static void t_handlePrReqVersion( void );
static void t_handlePrReqCalibration( void );
static void t_handlePrReqHeadType( void );
static void t_handlePrMode( unsigned char mode );
static void t_handlePrDisable( void );
static void t_handlePrEnable( void );
static void t_handlePrReset( void );
static void t_handlePrEnableTakeup( void );
static void t_handlePrCommand( unsigned char *pFrame );
static void t_handlePrCutterCut( void );
static void t_handlePrTeach( unsigned char *pFrame );
static void t_handlePrReqLabelTransfer( unsigned char *pFrame  );
static void t_handlePrMask( unsigned char *pFrame );
static void t_handlePrNomSize( unsigned char *pFrame );
static void t_handlePrTest( unsigned char *pFrame );
static void t_handlePrRam( unsigned char *pFrame );
static void t_handlePrCutterHome( void );
static void t_handlePrReqCutterStatus( void );
static void t_handlePrReqDotWear( void );
static void t_handlePrFactoryDefaults( void );
static void t_handlePrStartGapCal( void );
static void t_handlePrGapCalNext( void );
static void t_handlePrStartTUCal( void );
static void t_handlePrTUCalNext( void );
static void t_handlePrUseContinuousStock( void );
static void t_handleStationId( unsigned char order );
static void t_handlePrCutterDefault( void );
static void t_handlePrLabelSize( unsigned char *pFrame );
static void t_handlePrReqDotStatus( void );
static void t_handlePrHeadPower( unsigned char *pFrame );
static void t_handleWgReqSysInfo( void );
static void t_handleReqSysInfo( void );
static void t_handlePrSetTime( unsigned char *pFrame );
static void t_handlePrStopTime( unsigned char *pFrame );

/* handle weigher messages */
static void t_handleReqWakeup( void );
static void t_handleReqConfig( void );
static void t_handleModeMsg( unsigned char mode );
static void t_handleWriteConfig( unsigned char *pFrame );
static void t_handleReqStatus( void );
static void t_handleEnable( void );
static void t_handleDisable( void );
static void t_handleRezero( void );
static void t_handleReset( void );
static void t_handleReqWeight( void );
static void t_handleControl( unsigned char cntrl );
static void t_handleReqCat3Statistics( void );
static void t_handleCat3DateTime( unsigned char *pFrame );
static void t_handleReqCat3NextRecord( void );
static void t_handleReqCat3EraseLog( void );
static void t_handleReqCat3Page( unsigned char page, unsigned char index );
static void t_handleCat3RecordWrite( unsigned char *pFrame );
static void t_handleReqVersion( void );
static void t_handleValueMaxRequest( unsigned char *pFrame );
static void t_handleValueMaxWriteAssembly( unsigned char *pFrame );
static void t_handleValueMaxWriteVersion( unsigned char *pFrame );
static void t_handleValueMaxWriteSerial( unsigned char *pFrame );
static void t_handleValueMaxWriteDate( unsigned char *pFrame );
static void t_handleValueMaxWriteXGain( unsigned char *pFrame );
static void t_handleValueMaxWriteYGain( unsigned char *pFrame );
static void t_handleValueMaxWriteZGain( unsigned char *pFrame );
static void t_handleValueMaxWriteCal( unsigned char *pFrame );
static void t_handleReqInfo( void );
static void t_handleResetDefaultCfg( void );
static void t_handleWriteVMFit( unsigned char *pFrame );

void initPrHeader( unsigned char *pHdr, unsigned short size );
void initWrHeader( unsigned char *pHdr, unsigned short size );
unsigned short charToShort( unsigned char *pData );
unsigned short nCharToShort( unsigned char *pData );
unsigned long charToLong( unsigned char *pData );
unsigned long nCharToLong( unsigned char *pData );
unsigned char getCharFromShort( unsigned short *pWord, unsigned char index );
unsigned char getCharFromLong( unsigned long *pLong, unsigned char index );
#endif