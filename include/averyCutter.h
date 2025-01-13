#ifndef AVERYCUTTER_H
#define AVERYCUTTER_H
#include "internalMessages.h"
#include "fsl_lpuart.h"
#include "MIMXRT1024.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdbool.h>



#define cutter_task_PRIORITY ( configMAX_PRIORITIES - 2 )

#define CUTTER_UART_BAUD                        19200
#define CUTTER_UART_PRIORITY                    8

/* messages */
#define AC_STX                                  0x02
#define AC_ETX                                  0x03                                  
#define AC_EMPTY                                0xFF

#define CUTTER_SLAVE_ID                         0x31

/* largest msg packet size */
#define MAX_MESSGAE_SIZE                        256

#define CRC_TABLE_SIZE                          256

#pragma pack( 1 )
typedef struct
{
    unsigned char       start;
    unsigned char       slaveId;
    unsigned char       command;
    unsigned char       type;
    unsigned char       cr;
    unsigned long       config;
    unsigned long       length;
}ACHeader;
#pragma pack( push,1 )
#pragma pack( pop )

#define HEADER_SIZE                             13

/* status bits first byte field*/
#define AC_POWER_FAILURE                        0x0001
#define AC_MSG_FAILURE                          0x0002
#define AC_REBOOT_FLAG                          0x0004
#define AC_BLADE_HOME                           0x0008
#define AC_NOT_USED_A                           0x0010
#define AC_NOT_USED_B                           0x0020
#define AC_NOT_USED_C                           0x0040
#define AC_TIME_OUT_ERROR                       0x0080

typedef enum
{
    AC_ERR_NONE_,
    AC_ERR_POWER_FAILURE_,
    AC_ERR_MSG_FAILURE_,
    AC_ERR_BLADE_NOT_HOME_,
    AC_ERR_TIMEOUT_,
    AC_ERR_DOOR_OPEN_,    
}AC_ERRORS;

/* status bits second byte field*/
#define AC_NOT_USED_D                           0x0100
#define AC_NOT_USED_E                           0x0200
#define AC_NOT_USED_F                           0x0400
#define AC_NOT_USED_G                           0x0800
#define PLATTEN_TYPE                            0x0000
#define PLATTEN_SLOTTED_TYPE                    0x2000
#define PLATTEN_FULL_TYPE                       0x3000
#define AC_DOOR_OPEN                            0x4000
#define AC_NOT_USED_H                           0x8000

/* version string sizes */
#define MAX_PRODUCT_NAME_SIZE                   20
#define MAX_FIRMWARE_NUM_SIZE                   12
#define MAX_FIRMWARE_ISSUE_SIZE                 8
#define MAX_DATE_SIZE                           11
#define MAX_TIME_SIZE                           8

/* cutter commands */
#define CUTTER_CUT_PAPER                       'C'
#define CUTTER_RETURN_HOME                     'H'
#define CUTTER_CUT_PAPER_STEPS                 'K'
#define CUTTER_READ_SPEED                      'P'
#define CUTTER_RESET                           'R'
#define CUTTER_STATUS                          'S'
#define CUTTER_TEST_MODE                       'T'
#define CUTTER_VERSION                         'V'

typedef enum
{
    AC_CUT_,
    AC_HOME_,
    AC_CUT_STEPS_,
    AC_READ_SPEED_,
    AC_RESET_,
    AC_REQ_STATUS_,
    AC_TEST_MODE_,
    AC_VERSION_
}ACCCMDS;

/* distance ranges */
#define CUTTER_DISTANCE_MIN                     0x1E
#define CUTTER_DISTANCE_MAX                     0x52

/* speed ranges */
#define CUTTER_MIN_SPEED                        '0'
#define CUTTER_MAX_SPEED                        '9'

/* cutter messages */
typedef struct
{  
    unsigned char       byte0;
    unsigned char       byte1;
    unsigned char       byte2;
    unsigned char       byte3;
}ACStatus;

typedef struct
{
    unsigned char       productName[ ( MAX_PRODUCT_NAME_SIZE + 1 )];
    unsigned char       firmware[ ( MAX_FIRMWARE_NUM_SIZE + 1 )];
    unsigned char       issueDate[ ( MAX_FIRMWARE_ISSUE_SIZE + 1 ) ];
    unsigned char       date[ ( MAX_DATE_SIZE + 1 ) ];
    unsigned char       time[ ( MAX_TIME_SIZE + 1 ) ];  
}ACVersion;

#pragma pack( 1 )
typedef struct
{ 
    unsigned short      distance;
    unsigned char       speed;
}ACCutPaper;

typedef struct
{
    unsigned char       speed;
}ACHome;
    
typedef struct
{
    unsigned short      distance;
    unsigned char       speed;
}ACCutSteps;

typedef struct 
{
    unsigned short      data;    
}ACReadProfile;

typedef struct
{
    unsigned char       index;
}ACRVersion;
#pragma pack( push,1 )
#pragma pack( pop )

typedef union
{
    ACCutPaper          cut;
    ACStatus            status; 
    ACVersion           version;
    ACHome              home;
    ACCutSteps          cutSteps;
    ACReadProfile       profile;    
    ACRVersion          readVersion;
}AMsgBody;

/* message sizes in ascii form */
#define PAPER_CUT_MSG_SIZE              0x35313030          /* 15 */
#define RETURN_HOME_MSG_SIZE            0x33313030          /* 13 */
#define PAPER_CUT_STEPS_MSG_SIZE        0x35313030          /* 15 */
#define READ_PROFILE_MSG_SIZE           0x33313030          /* 13 */
#define DEFAULT_MSG_SIZE                0x32313030          /* 12 */
#define READ_VERSION_MSG_SIZE           0x33313030          /* 13 */

/* tx message sizes */
#define PAPER_CUT_MSG_SIZE_             16 
#define RETURN_HOME_MSG_SIZE_           13 
#define PAPER_CUT_STEPS_MSG_SIZE_       15
#define READ_PROFILE_MSG_SIZE_          13 
#define DEFAULT_MSG_SIZE_               12
#define READ_VERSION_MSG_SIZE_          13

/* rx message sizes */
#define PAPER_CUT_RX_MSG_SIZE_          23 
#define RETURN_HOME_RX_MSG_SIZE_        22 
#define PAPER_CUT_RX_STEPS_MSG_SIZE_    22
#define READ_PROFILE_RX_MSG_SIZE_       13 
#define DEFAULT_MSG_RX_SIZE_            18
    
#define READ_VERSION0_RX_MSG_SIZE_      38
#define READ_VERSION1_RX_MSG_SIZE_      30
#define READ_VERSION2_RX_MSG_SIZE_      26
#define READ_VERSION3_RX_MSG_SIZE_      29
#define READ_VERSION4_RX_MSG_SIZE_      18
    
/* argument limits on messages */
#define MAX_DISTANCE                    0x99
#define MAX_SPEED                       9
#define MAX_MARK                        15
#define MAX_VERSION_INDEX               4

typedef enum 
{
    UNKNOWN_PLATTEN_,
    SLOTTED_PLATTEN_,
    FULL_PLATTEN_
}AC_PLATTEN;

typedef struct
{
    ACHeader    header;
    AMsgBody    body;    
}ACutterMsgs;

/* cutter thread states */
typedef enum
{    
    AC_INIT_INTERFACE_,
    AC_INIT_DEVICE_STATUS_,
    AC_INIT_DEVICE_TX_VERSION_,
    AC_INIT_DEVICE_RX_VERSION_,
    AC_INIT_SEND_DOOR_STATUS_,
    AC_INIT_WAIT_DOOR_STATUS_,
    AC_READY_,
    AC_WAIT_FOR_COMMAND_,
    AC_PROCESS_CUT_,
    AC_PROCESS_CUT_RESPONSE_,
    AC_PROCESS_HOME_,
    AC_PROCESS_HOME_RESPONSE_,
    AC_ERROR_
}ACTSTATES;

typedef struct 
{
    AC_ERRORS           error;    
    bool                txReady; 
    bool                msgReady;
    bool                msgComplete;
    bool                initSeq;
    unsigned short      deviceStatus;
    unsigned short      totalBytesRcvd;
    unsigned short      expectedMsgSize;
    unsigned short      totalBytesXfrd;
    unsigned short      msgSize;
    unsigned long       *pTxBfr;
    unsigned long       *pRxBfr;
    ACVersion           version;   
    AC_PLATTEN          platten;
}CutterMgr;


/* public functions */
BaseType_t createAveryCutterTask( QueueHandle_t msgQueue, QueueHandle_t printerMsgQueue );
void handleInternalMessage( ICMessages *pMsg );    
bool initCutter( void );
bool sendACCut( CutterMgr *pMgr, ACCutPaper *pMsg );
bool sendACReqStatus( CutterMgr *pMgr );
bool sendACReqVersion( CutterMgr *pMgr, ACRVersion *pMsg );
bool sendACHome( CutterMgr *pMgr, ACHome *pMsg );
bool sendACCutSteps( CutterMgr *pMgr, ACCutSteps *pMsg );
bool sendACReadProfile( CutterMgr *pMgr, ACReadProfile *pMsg );
bool sendACTestMode( CutterMgr *pMgr );
bool sendACReset( CutterMgr *pMgr );
unsigned char getMessageSize( unsigned char type );
bool isCutterInterlockClosed( void );

/* old cutter functions on fsss 
bool cutterCut( bool notifyEngine );
bool isCutterHome( void );
bool isCutterInstalled( void );
void setCutterInstalled( void );
bool isCutterInterlockClosed( void );
*/


/* private functions */
static void averyCutterTask( void *pvParameters );
static bool receiveCutterMsg( unsigned char rxSize  ); 
static size_t getVersionRxMsgSize( unsigned char index );
static void stuffTxBuffer( unsigned char *pMsg, unsigned char size );
static void buildCutterMsg( ACCCMDS cmd, ACutterMsgs *pMsg, unsigned short arg1, unsigned short arg2 );
static void prepTransmitBfr( unsigned char *pBfr, unsigned char length );
static void hexToAscii( unsigned char *pChar, unsigned char data );
static void handleCutterMessage( CutterMgr *pMgr, unsigned char *pMsg );
static bool parseMsgHeader( ACHeader *pHeader, unsigned char *pMsg );
static void parseHeaderStatus( CutterMgr *pMgr, ACHeader *pHeader );
static void parseVersionMsg( CutterMgr *pMgr, unsigned char *pMsg );
static void parseVerPoductMsg( CutterMgr *pMgr, unsigned char *pMsg );
static void parseVerFirmNumbMsg( CutterMgr *pMgr, unsigned char *pMsg );
static void parseVerIssueMsg( CutterMgr *pMgr, unsigned char *pMsg );
static void parseVerDateMsg( CutterMgr *pMgr, unsigned char *pMsg );

static void showDeviceStatus( unsigned short status );
static void showCutterVersion( CutterMgr *pMgr );
static bool openCutterInterface( unsigned short baud );
void uart1Callback( LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData );
static unsigned short calcCrc16( unsigned char *pBlob, unsigned short length );



#endif