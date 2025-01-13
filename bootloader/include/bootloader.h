#ifndef BOOTLOADER_H
#define BOOTLOADER_H

#include <stdbool.h>
#include "threadManager.h"

#define MAX_FRAMES             8
#define NUM_BYTES_PER_FRAME    8

#define CMD_REQ_BOOT_VER 34
#define CMD_FLASH_UPDATE_NODE 124

typedef enum 
{
	allow_char,
	dont_use_char = 257
}ENUM_MAKE_INT;

#if 1
//backend boot event codes
typedef enum KPCBootEvent{
	KpcBootGeneric 		= 0,	//Must keep this value to be compatible with backend application
	KpcBootStatus 		= 1,	//Must keep this value to be compatible with backend application
	KpcBootVersion 		= 2,	//Must keep this value to be compatible with backend application
	KpcBootAppVersion 	= 3,	//Must keep this value to be compatible with backend application
	KpcBootAck 			= 4,	//Must keep this value to be compatible with backend application
	KpcBootReady 		= 5,	//Must keep this value to be compatible with backend application
	KpcBootReqStatus 	= 6,	//Must keep this value to be compatible with backend application		
	KpcBootCmd 			= 7,	//Must keep this value to be compatible with backend application
	KpcBootFrame,
	KpcBootFrame1,
	KpcBootFrame2,	//10
	KpcBootFrame3,			
	KpcBootFrame4,
	KpcBootFrame5,
	KpcBootFrame6,
	KpcBootFrame7,	//15
	KpcBootFrame8,			
	KpcBootFlush,
	KpcBootReqVersion 		= 18,	//Must keep this value to be compatible with backend application
	KpcBootReqAppVersion 	= 19,	//Must keep this value to be compatible with backend application
	KpcBootDone,			//20
	KpcBootRestart,			
	KpcNoEvent,
	KpcBootBulkMsg,

	KpcBootStartUpgrade,		//    number of nodes for UI to present
	KpcBootUpgradeStatus,		// 25 tell UI the status (nodeId,state,version,error) dont know what triggers it
	KpcBootUpgradeProgress,     //    tell the UI the current progress. Prolly response to KpcStartUpgrade
	KpcBootUpgradeFinished,		//    close UI upgrade page
	KpcBootUpgradeRequired,		//    ask UI to open upgrade page
	KpcBootUpgradeUIReady,		//    UI has signaled the upgrade page is ready
	KpcBootReqUpgradeInfo,		// 30 UI requests upgrade information
	KpcBootUpgradeInfo,			//    upgrade information

	// SOF-3478
	KpcBootUpgradeCancel,		//    cancel upgrade 
	KpcBootUpgradeRequested,	//	  UI made decision to go ahead with upgrade
	KpcBootUpgradeUpdateUI,		//    Update UI upgrade is ready after decision has been made to move forward
	// SOF-3478 END

	KPCBootEventEnd				// 35
}KPCBootEvent_t;
#endif

//Hobart function codes
typedef enum boot_msg_code {
  BOOTLOADER_READY,
  BOOT_REQ_STATUS,
  BOOT_CMD,
  BOOT_FRAME,
  BOOT_FRAME1,
  BOOT_FRAME2,
  BOOT_FRAME3,
  BOOT_FRAME4,
  BOOT_FRAME5,
  BOOT_FRAME6,
  BOOT_FRAME7,
  BOOT_FRAME8,
  BOOT_FLUSH,
  BOOT_REQ_BOOT_VERSION,
  BOOT_REQ_APP_VERSION,
  BOOT_USB_BULK_PAYLOAD_RCV
} BootMsgCode;


typedef enum boot_status_code {
  BOOT_INIT,
  BOOT_READY,
  BOOT_FAULT,
  BOOT_VERIFY,
  BOOT_DONE	
} BootStatusCode;

typedef enum boot_loader_faults {
  NOFAULT,
  TIMEOUT,
  WRITE_FAILURE,
  CHECKSUM,
  UNKNOWN_RECORD_TYPE,
  ERASE_FAILURE_APP,
  ERASE_FAILURE_BOOT,
  COMPONENT_NOT_SET,
  IMAGE_MISMATCH,
  IMAGE_CHECKSUM_MISMATCH,
  MODEL_TYPE_ERROR,
  WRITE_ADDRESS_OUT_OF_RANGE,
  FRAME_OUT_OF_RANGE
} BootLoaderFaults;

typedef enum {
  NO_CMD,
  JUMP,
  INIT_UPGRADE,
  I_HEARD_YOU_NOW_SHUT_UP,
  VERIFY_APP
} BootCmd;


typedef struct {
    unsigned char minor_rev;
    unsigned char major_rev;
} WrapperVersion;

typedef struct version {
  uint16_t major_rev;
  uint16_t minor_rev;
  uint16_t build_number;
} Version;

//usb incomming boot command message
typedef struct {
	uint8_t		boot_msg;
	uint16_t	boot_cmd;
    uint16_t	dest;
    uint16_t    src;
    uint32_t    addr;
} USBBootCmdMsg;

/* Internal Hobart messages */
typedef struct boot_generic {
    BootMsgCode      function_code;
} BootGenericMsg;

typedef struct boot_cmd {
    BootMsgCode	function_code;
	BootCmd		boot_cmd;
    uint16_t	dest;
    uint16_t    src;
    uint32_t    addr;
} BootCmdMsg;

typedef struct boot_frame_header_msg {
    /*
    BootMsgCode         function_code;
    unsigned short      source_id;
    PeripheralModel     destination_model;
    unsigned short	number_of_frames; 
    unsigned short      final_segment_length;
    */
    KPCBootEvent_t         function_code;
    uint8_t             command;
    uint8_t             sourceID;
    uint8_t             nodeID;
    uint16_t            numFrames;
    uint16_t            length;
} BootFrameHeaderMsg;

typedef struct boot_frame {
    KPCBootEvent_t      function_code;
    uint8_t          byte0;
    uint8_t          byte1;
    uint8_t          byte2;
    uint8_t          byte3;
    uint8_t          byte4;
    uint8_t          byte5;
    uint8_t          byte6;
    uint8_t          byte7;
} BootFrameMsg;

typedef union boot_msg {
  BootGenericMsg           generic;
  BootCmdMsg               cmd;
  BootFrameHeaderMsg       frame_header;
  BootFrameMsg             frame;
} BootMsg; 

//////////////////////////////////////////////////////////////////////////
/****************************/
/* Outgoing  messages types */
/****************************/
//////////////////////////////////////////////////////////////////////////
typedef struct boot_status_msg {
 #if 1
	uint16_t			msgType;
#endif	
    uint16_t	        model;		
    //BootStatusCode      status;
	uint16_t			status;
    //BootLoaderFaults    fault;
	uint16_t    		fault;
    uint32_t       fault_info;	  
} BootStatusMsg;

typedef struct boot_app_version_msg {
	uint16_t	msgType;
  	Version		printer_app_version;         
  	Version   	weigher_app_version;         
} BootAppVerMsg;

typedef struct boot_version_msg {
	uint16_t	msgType;
  	Version		boot_version;         
} BootVerMsg;

typedef struct wrapper_version_msg {
    WrapperVersion     app_version;
    WrapperVersion     data_version; // dont know what this is, but its in the backend
    WrapperVersion     boot_version;   
} WrapperVerMsg;

typedef struct boot_ack_msg {
    uint8_t               source;
    uint8_t               destination;
    uint16_t              acked_ID;
} BootAckMsg;

typedef struct boot_rdy_msg {
#if 1
	uint16_t	msgType;
	uint16_t    product_id;
#else	
    uint8_t             source;
    uint8_t             destination;
#endif	
} BootReadyMsg;


typedef struct boot_generic_msg {
#if 1
	uint16_t	msgType;
#endif	
} BootGenericTypeMsg;

#endif