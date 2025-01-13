#ifndef WGMESSAGES_H
#define WGMESSAGES_H
#include <stdbool.h>
#include "globalWeigher.h"
#include "valueMax.h"

typedef enum 
{
    WG_WAKEUP,
    WG_CONFIG,
    WG_REQ_CONFIG,
    WG_RESET,
    WG_STATUS,
    WG_ENABLE,
    WG_DISABLE,
    WG_MODE,
    WG_REZERO,
    WG_CONTROL,                 /* no longer used! */
    WG_COLD_RESET,              /* 10 */            
    WG_CAT3_REQ_STATISTICS,
    WG_CAT3_AUDIT_STATISTICS,
    WG_CAT3_DATE_TIME,
    WG_CAT3_REQ_PAGE,
    WG_CAT3_READ_RECORDS, /* 15 */
    WG_CAT3_REQ_NEXT_RECORDS,  
    WG_CAT3_WRITE_RECORD,
    WG_CAT3_RECORD_STATUS,
    WG_CAT3_REQ_ERASE_LOG,
    WG_CAT3_LOG_ERASE_COMPLETE, /* 20 */
    WG_CAT3_ERROR,
    WG_FACTORY_DEFAULTS,
    WG_FACTORY_DEFAULTS_CMPLT,
    
    
    WG_REQ_WEIGHT,
    WG_REQ_SYSTEM_INFO,
    WG_SYSTEM_INFO,
    
    WG_ERROR,
    WG_REQ_BOARD_TEST,
    WG_BOARD_TEST,
    
    WG_VM_REQUEST,
    WG_VM_WRITE_ASSEMBLY,
    WG_VM_ASSEMBLY,
    WG_VM_WRITE_VERSION,
    WG_VM_VERSION,
    WG_VM_WRITE_SERIAL,
    WG_REQ_WAKEUP = 34,
    WG_VM_SERIAL,
    WG_VM_WRITE_DATE,
    WG_VM_DATE,
    WG_VM_WRITE_FIT_VALUES,
    WG_VM_FIT_VALUES,
    WG_REQ_VERSION = 126,
    WG_REQ_SYS_INFO = 253,
    WG_SYS_INFO
} WgMsgType;
 
typedef struct 
{
    WgMsgType                   msgType;
}WgGeneric;

typedef struct 
{
    WgMsgType                   msgType;
    unsigned short              pid;
    unsigned short              weigherType;
}WgWakeup;
 
typedef struct 
{
    WgMsgType                   msgType;
    unsigned char               pad0;
    unsigned short              disposition;    
    WgConfiguration             config;
}WgCfg;

typedef struct 
{
    WgMsgType           msgType;
    unsigned short      pid;
    bool                valueMaxEnabled;
}WgSysInfo;


typedef struct
{
    WgMsgType                   msgType;
    unsigned short              disposition;
    fit_flash_section           fit;
}WgVMFitMsg;

typedef struct 
{
    WgMsgType                   msgType;
    unsigned short              disposition;
    WgConfiguration             config;
}WgInquire;
 
typedef struct 
{
    WgMsgType                   msgType;
}WgReset;

typedef struct
{
    WgMsgType                   msgType;
}WgEnable;

typedef struct 
{
    WgMsgType                   msgType;
}WgDisable;

typedef struct
{
    WgMsgType                   msgType;
    WeighMode       mode;
}WgMode;

typedef struct
{
    WgMsgType                   msgType;
    unsigned char               pad0;
    unsigned short              status;
    long                        avgFilterCounts;//zeroedCalibrCounts; /* Zeroed counts */
    long                        nonZeroCalibratedCounts;//calibratedCounts;   /* Unzeroed counts */
    unsigned long               rawCounts;
    long                        zeroedCalibratedCounts;//avgFilterCounts;    /* For calibration */
    long                        acclAvgXCounts;//rawCounts;         /* For debug only */
    long                        acclAvgYCounts;
    long                        acclAvgZCounts;
    short                       tiltX;
    short                       tiltY;
}WgStatus;

typedef struct 
{
    WgMsgType                   msgType;
}WgRezero;

typedef struct 
{
    WgMsgType                   msgType;
    unsigned char               control;
}WgControl;

typedef struct
{
    WgMsgType                   msgType;
    bool                        serialFlashValid;
    bool                        eepromValid;
    unsigned char               softwareVersion[3];
    unsigned char               hardwareVersion[3];
}WgInfo;

typedef struct
{
    WgMsgType                   msgType;
    unsigned char               totalPages;                     /* Total number of Cat3 audit pages */
    unsigned char               totalRecordsPerPage;            /* Total number of records per page */
    unsigned char               currentPage;                    /* Audit managers current working page */
    unsigned short              totalFreeRecords;               /* Total number of free records in the serial flash */
    unsigned short              totalRecordedEvents;            /* Total number of CAT3 events saved */
    unsigned short              totalCalibrationRecords;        /* Total number of saved calibration events */
    unsigned short              totalConfigurationRecords;      /* Total number of saved configuration events */
    /* sealable parameter counters (requirement 8.24c, 8.24d ) */
    unsigned short              gainCoefficientCounter;         /* Total number of times gain coefficent has changed */
    unsigned short              offsetCoefficientCounter;       /* Total number of times offset coefficent has changed */
    unsigned short              gainFactorCounter;              /* Total number of times gain factor has changed */
    unsigned short              centerOfMainCounter;            /* Total number of times center of maintenance has changed */
    unsigned short              zeroReferenceCounter;           /* Total number of times zero reference has changed */
    unsigned short              weigherModelCounter;            /* Total number of times weigher model has changed */
    unsigned short              minWeightPrintCounter;          /* Total number of times min weight to print has changed */
    unsigned short              maxWeightCounter;               /* Total number of times max weight has changed */
    unsigned short              filterSpeedCounter;             /* Total number of times filter speed has changed */
    unsigned short              divisionSizeCounter;            /* Total number of times division size has changed */
    unsigned short              weigherModeCounter;             /* Total number of times weigh mode has changed */
}WgCat3Statistics;

typedef struct
{
    WgMsgType                   msgType;
    unsigned char               pageNumber;
    unsigned char               index;          /* record index 0 - 128 */
}WgReqCat3Page;

typedef struct
{
    WgMsgType                   msgType;
    unsigned char               pageNumber;
    unsigned char               index;          /* record index of the first record */
    unsigned char               records[320];
}WgCat3Records;

typedef struct
{
    WgMsgType                   msgType;
    unsigned char               event;
    unsigned char               parameterId;
    unsigned char               eventType;
    unsigned long               epochTime;
    unsigned long               newValue;
    unsigned long               oldValue;
}WgCat3WriteRecord;

typedef struct
{
    WgMsgType                   msgType;
    unsigned char               pad0;
    unsigned char               pad1;
    unsigned char               pad2;
    unsigned long               epochTime;
}WgCat3DateTime;

typedef struct
{
    WgMsgType                   msgType;
    unsigned char               pad0;
    bool                        saved;          /* record saved to Cat3 audit trail */
}WgCat3RecordStatus;

typedef struct
{
    WgMsgType                   msgType;
    unsigned char               pad0;

}WgCat3EraseComplete;

typedef struct
{
    WgMsgType                   msgType;  
    unsigned short              pid;           /* product id */
    unsigned char               swMajor;
    unsigned char               swMinor;
    unsigned char               swBuild;
    unsigned long               firmware;     /* checksum */
    unsigned char               hwMajor;
    unsigned char               hwMinor;
}WgVersion;

typedef struct
{
    WgMsgType                   msgType;
    unsigned short              codeId;                    /* error code number */
    unsigned short              action;                    /* type of action the host should take */
    unsigned long               data;
}WgError;

typedef struct
{
    WgMsgType	                msgType;
    unsigned char               testCode;
    unsigned long               data;
    unsigned long               data1;
    unsigned long               data2;
    unsigned long               data3;
    unsigned long               data4;
}WgBoardTest;

typedef struct
{
    bool                        error;
    unsigned long               data;
    unsigned short              data1;
    unsigned char               data2;
}WgBdTestComplete;

/************ Value Max Messages **************************/ 
typedef struct
{
    WgMsgType                   msgType;  
    ValueMaxReq                 req; 
}WgVMRequest;

typedef struct 
{
    WgMsgType                   msgType; 
    unsigned char               assemblyNumber[VM_ASSEMBLY_MAX];
}WgVMWrAssembly;

typedef struct
{
    WgMsgType                   msgType;   
    unsigned char               version[VM_VER_MAX];
}WgVMWrVersion;

   
typedef struct
{
    WgMsgType                   msgType;   
    unsigned char               serial[VM_SERIAL_MAX];  
}WgVMWrSerial;

typedef struct
{
    WgMsgType                   msgType;   
    unsigned long               epoch;      
}WgVMWrDate;

typedef struct
{
    WgMsgType                   msgType;
    float		        offset;
    float                       gain;
}WgVMWrOffsetGain;

typedef struct 
{
    float                       xGain;  
    float                       yGain;  
    float                       zGain;  
}WgVMCurrent;

typedef struct
{
    WgMsgType                   msgType;
    bool                        result;      
}WgVMCalibrated;
/************************* end Value Max Messages *******************/

typedef union 
{
    WgGeneric           generic;
    WgWakeup            wakeup;
    WgCfg               config;
    WgInquire           inquire;
    WgReset             reset;
    WgStatus            status;
    WgEnable            enable;
    WgDisable           disable;
    WgMode              mode;
    WgRezero            rezero;
    WgControl           control;
    WgInfo              info;
    WgCat3Statistics    statistics;
    WgReqCat3Page       pageRequest;
    WgCat3DateTime      dateTime;
    WgCat3Records       records;
    WgCat3WriteRecord   writeRecord;
    WgCat3EraseComplete eraseComplete;
    WgVersion           version;
    WgBoardTest         boardTest;  
    WgBdTestComplete    testComplete;  
    WgVMRequest         vmRequest;
    WgVMWrAssembly      vmAssembly;
    WgVMWrVersion       vmVersion;
    WgVMWrSerial        vmSerial;
    WgVMWrDate          vmDate;
    WgVMWrOffsetGain    vmFactor;  
    WgVMCurrent         vmCurrent;
    WgVMCalibrated      vmCalibrated;
    WgVMFitMsg          vmFit;
}WgMessage;




#endif