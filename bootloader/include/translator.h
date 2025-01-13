#ifndef TRANSLATOR_H
#define TRANSLATOR_H
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "bootloader.h"
#include <stdbool.h>


#define translator_task_PRIORITY ( configMAX_PRIORITIES - 1 )

/* this list must be same as in backend */
typedef enum
{
    UNKNOWNTYPE_,
    G_BOOTLOADER_          
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
    BootMsg bootMsg;
}HBMessage;

typedef struct
{
    USBHeader hdr;    
    unsigned char body[ sizeof(HBMessage) ];
}HUSBMessage;

#define LGCY_LOGICAL_ADDR_BOOTLOADER                 0x28
#define LGCY_LOGICAL_ADDR_GBOOTLOADER                0x29
#endif
#pragma pack( push, 1 )
#pragma pack( pop )

#define MAX_USB_INT_FRAME_SIZE                  64
#define MAX_USB_FRAME_SIZE                      512     /* this must be the same size af the pipe! */

/* public functions */
BaseType_t createTranslator( QueueHandle_t bootMsgQueue, QueueHandle_t UsbInBootMsgQueue, QueueHandle_t UsbOutBootMsgQueue );

void sendBootStatus(BootStatusMsg * status);
void sendBootVersion(BootVerMsg *version);
void sendBootAppVersion(BootAppVerMsg *version);
void sendBootReady(BootReadyMsg * rdy);
void sendBootAck(void);

uint32_t getBlMsgTypeSize(KPCBootEvent_t blMsgType);

/* private functions */
static void translatorTask( void *pvParameters );
static void translateBootMessage( unsigned char *pBfr  );
static unsigned int translateFrameHdr( unsigned char *pBfr, USBHeader *pHdr );

/* bootloader handler messages */
static void t_handleBootMessage( unsigned char *pMsg_ );
static void t_handleBootCmd( unsigned char * BootCmd );
static void t_handleReqBootStatus(void);
static void t_handleReqBootVersion(void);
static void t_handleReqAppVersion(void);
static void t_handleBootFrameHeader(unsigned char *pMsg_);
static bool t_handleBootFrame(unsigned char *pMsg_);
static void t_handleBootFlush(void);

unsigned short charSwapShort( unsigned char *pData );
unsigned short charToShort( unsigned char *pData );
unsigned long charToLong( unsigned char *pData );
unsigned char getCharFromShort( unsigned short *pWord, unsigned char index );
unsigned char getCharFromLong( unsigned long *pLong, unsigned char index );

#endif