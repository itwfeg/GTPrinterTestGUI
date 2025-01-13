#ifndef PRMESSAGES_H
#define PRMESSAGES_H
#include <stdbool.h>
#include "globalPrinter.h"
#include "printhead.h"
#include "commandTable.h"

/************************* Message Definitions ******************************/
typedef enum  
{ 
    PR_WAKEUP,
    PR_CONFIG, 
	PR_INQUIRE,
    PR_RESET, 
    PR_STATUS,
    PR_ENABLE,
    PR_DISABLE, 
    PR_MODE,
    PR_SENSORS, 
    PR_POWER,
    PR_MASK,                    /* 10 */
    PR_TEACH,
    PR_COMMAND,
    PR_REQ_CALIBRATE,
    PR_RAM,
    PR_TEST,
    /* PR_FLUSH */
    PR_SIZE = 17,
    PR_ENABLE_TAKEUP = 19,     
    PR_REQ_WAKEUP = 21,
	PR_REQ_CONFIG = 22,
    PR_REQ_STATUS = 23,
    PR_REQ_SENSORS,
    PR_REQ_TRANSFER = 25,            
    PR_REQ_HEAD_POWER = 46,
    
    PR_FACTORY_DFLTS = 57,
    PR_FACTORY_DFLTS_CMPLT,
    PR_REQ_HEAD_TYPE,
    PR_HEAD_TYPE,
    PR_CUTTER_CUT,
    PR_CUTTER_HOME,
    PR_REQ_CUTTER_STATUS,
    PR_CUTTER_STATUS,
    
    PR_SET_TIMER = 71,
    PR_GLOBAL_SENSORS,
    PR_GLOBAL_CONFIG,   
    PR_STOP_TIME,
    PR_LABEL_SIZE,
    PR_STATION_ID,    
    PR_USE_CONTINUOUS_STOCK = 161,    
    PR_REQ_VERSION =170,
    PR_VERSION,
    PR_TRANSFER_READY = 173, 
    PR_REQ_DOT_WEAR = 174,
    PR_DOT_WEAR,               
    PR_DOT_STATUS =179,
    PR_REQ_DOT_STATUS,
    

    PR_ERASE_CUTTER_BIT = 184,
    PR_START_GAP_CALIBRATION = 188,
    PR_GAP_CAL_STATUS,          //189
    PR_CAL_GAP_SENSOR_NEXT,     //190
    PR_LABEL_TAKEN_ERROR,       //191
    
    PR_START_TU_CALIBRATION = 204,    //192
    PR_TU_CAL_STATUS = 205,           //193
    PR_CAL_TU_SENSOR_NEXT = 206,      //194
    
    PR_REQ_SYS_INFO = 253,
    PR_SYS_INFO = 254
}PRMsgType;

typedef struct 
{
    unsigned char       sensor;
    unsigned char       user;
    unsigned char       sensor2;
}MaskInfo;

typedef struct 
{  
    EngineStates        state;
    unsigned char       command;
    unsigned char       error;
    unsigned char       history;
    unsigned char       sensor;
    unsigned char       user;
    short               counter;
    MaskInfo            mask;
    unsigned char       sla;
    unsigned char       sensor2;
}PrStatusInfo;

typedef struct 
{
    PRMsgType           msgType;
    IndirectDataItem    ram [MAX_RAM_LOCATIONS]; 
}PrRam;

typedef struct pr_generic
{
    PRMsgType           msgType;
}PrGeneric;
 
typedef struct pr_enabletakeup
{
    PRMsgType           msgType;
}PrEnableTakeup;

typedef struct 
{
    unsigned short      msgType;
    unsigned short      pid;
    StationID           id;
    LabelWidth          label_width;
    unsigned short      buffer_size;
    unsigned short      printhead_size;
    unsigned short      transfer_size;
    bool                configValid;
}PrWakeup;

typedef struct 
{
    unsigned short      msgType;
    unsigned short      pid;
    StationID           id;
    LabelWidth          label_width;
    unsigned short      buffer_size;
    unsigned short      printhead_size;
    unsigned short      transfer_size;
    HEADTYPE            headType;
    bool                cutterInstalled;
    bool                cutterEnabled;
    bool                configValid;    
}PrSysInfo;

typedef enum
{
    PERMANENT_CFG,
    TEMPORARY_CFG,
    DEFAULT_CFG,    
}CFGType;

typedef  struct pr_config
{
    PRMsgType           msgType;
    CFGType             disposition;
    Pr_Config           config;
}PrConfig;


struct pr_inquire
{
    PRMsgType           msgType;
    unsigned short      disposition;
    Pr_Config           config;
};
typedef struct pr_inquire PrInquire;
        
      
struct pr_reset
{
    PRMsgType           msgType;
};
typedef struct pr_reset PrReset;
   
typedef enum
{
  _UNKNOWN_TIMER,
  _LABEL_TAKEN_TIMER,
  _CONTINUOUS_LABEL_TIMER,
  _SENSOR_TIMER
}SysTimerType;

typedef struct
{
  PRMsgType             msgType;
  SysTimerType          timer;
  unsigned long         milliseconds;  
}PrSetTime;

typedef struct
{
  PRMsgType             msgType;
  SysTimerType          timer;
}PrStopTime;

/* Command Bits (note - other commands are sent with the teach tables) */
#define COMMAND_COMPLETE                0x80
#define DISABLE_COMMAND                 0x7C
#define ENABLE_COMMAND                  0x7D
#define CALIBRATE_COMMAND               0x7E
#define RESET_COMMAND                   0x7F

/* Sensor2 Bits */
#define OUT_OF_DATA_BUFFERS             0x01
#define LOW_STOCK_REACHED               0x02
#define OUT_OF_STOCK                    0x04
#define OUT_OF_DATA_BUFFERS2            0x08
#define JAMMED_LABEL                    0x10

/* Fatal Error Bits */
#define NO_ERROR                        0x00
#define HEAD_UP                         0x01
#define THERMAL_SHUTDOWN                0x02
#define MEDIA_SHUTDOWN                  0x04
#define CASSETTE_MISSING                0x08
#define UNKNOWN_PH                      0x20
#define PH_OVERCURRENT                  0x40
#define OUT_OF_PRINTDATA                0x80

typedef struct
{
    PRMsgType           msgType;
    PrStatusInfo        status;
}PrStatus;

    
typedef struct
{
    PRMsgType           msgType;
} PrEnable;

     
typedef struct 
{
    PRMsgType           msgType;
}PrDisable;

typedef struct 
{
    PRMsgType           msgType;
    PrintMode           mode;
}PrMode;

typedef struct 
{
    PRMsgType           msgType;
    unsigned short      headup_reading;
    unsigned short      label_width_reading;
    unsigned short      lowStock; 
    unsigned short      head_temperature_reading;
    unsigned short      head_voltage_reading;
    unsigned short      head_current_reading;
    unsigned short      label_high_average; 
    unsigned short      label_low_average;
    unsigned short      label_threshold; 
    unsigned short      label_reading; 
    unsigned short      media_high_average;
    unsigned short      media_low_average;
    unsigned short      media_threshold;
    unsigned short      media_reading; 
}PrSensors;

/* sensors message just for global scales */
typedef struct 
{
    PRMsgType           msgType;
    bool                headUp;
    LabelWidth          labelWidth;
    bool                lowStock;
    bool                labelTaken; 
    char                headTemperature;
    unsigned short      media_reading;        
    unsigned short      headVoltage;
}GPrSensors;

typedef struct pr_teach
{
    PRMsgType           msgType;
    unsigned short      entries;
    unsigned char       identifier;
    CmdOp               operation[ MAX_TABLE_OPERATIONS ];
}PrTeach;

typedef struct pr_command
{
    PRMsgType           msgType;
    unsigned char       identifier;
    char                options;
    unsigned short      data_item; 
    short               value;
}PrCommand;


typedef struct 
{
    PRMsgType           msgType;
    PrinterCal          calibration;
}PrCalibrate;

typedef struct 
{
    PRMsgType           msgType;
    char                data; 
    char                data_channel;
}PrTest;


typedef struct pr_mask
{
    PRMsgType           msgType;
    MaskInfo            mask;
}PrMask;


typedef struct 
{
    PRMsgType           msgType;
}PrFlush;

typedef struct 
{
    PRMsgType           msgType;
    unsigned short      actual_size;
    unsigned short      measured_size;
}PrSize;

typedef struct
{
    PRMsgType           msgType;
    unsigned short      transferSize;
    unsigned long       imageSize;   
}PrReqTransfer;


typedef struct
{
    PRMsgType	        msgType;
    unsigned char	headType;
    unsigned short      headSize;	
}PrHead;

typedef struct
{
    PRMsgType           msgType;  
    unsigned short      pid;                       /* product id */
    unsigned short      major;                     /* software */
    unsigned short      minor;
    unsigned short      build;
    unsigned short      hwMajor;                   /* hardware */
    unsigned short      hwMinor;
}PrVersion;

typedef struct
{
  PRMsgType             msgType;  
  unsigned long         size;                      /* total size of the label image in bytes */
}PrLabelSize;

typedef struct
{
    PRMsgType           msgType;    
    bool                home;
    bool                installed;
    bool                interlock;
    bool                jammed;
}PrCutterStatus;


#define SERVICE_SCALE_HEAD_SIZE_DOTS    576
typedef struct 
{
    PRMsgType	        msgType;
    unsigned char       dot_status[4];
    unsigned char       dotCounts[4];
}PrDotWear;

typedef struct 
{
    PRMsgType	        msgType;
    unsigned short      source;
    unsigned short      destination;
    unsigned char       head_status;
}PrDotStatus;

typedef struct 
{
    PRMsgType	        msgType;
    bool                state;    
    float               calValue;
}PrGapCalCmplt;

typedef enum
{
    _CalInit,
    _BackingPaper,
    _LabelPlusBacking,
    _CalcDeflections,
    _FailureSettingI,
    _FailureMemory,
    _Done, 
    _CalTakeupMinTension,
    _CalTakeupMaxTension
}GapCalState;

typedef struct
{
    PRMsgType	        msgType;
    GapCalState         state;
    unsigned short      deflectionVoltage;
    unsigned short      driveCurrent;
    unsigned short      backingVoltage;
    unsigned short      labelBackVoltage;
}PrGapCalStatus;

typedef struct 
{
    PRMsgType           msgType;
    bool                state;
    bool                protection_on;
}PrPower;
 

typedef struct 
{
    PRMsgType	        msgType;
    StationID           station;
}PrStationID;

typedef struct
{
    PRMsgType	        msgType;
    bool                error;    
}PrTakeLabelErr;

/* Printer Messages */
typedef union 
{
    PrGeneric           generic;
    PrWakeup            wakeup;
    PrConfig            config; 
    PrInquire           inquire;
    PrReset             reset;
    PrStatus            status;
    PrEnable            enable;
    PrDisable           disable;
    PrMode              mode;
    PrSensors           sensors;
    PrPower             power;
    PrMask              mask;
    PrTeach             teach;
    PrCommand           command;
    PrCalibrate         calibrate;
    PrSize              size;
    PrReqTransfer       transfer;
    PrEnableTakeup      enTakeup;
    PrHead		head;
    PrVersion           version;
    PrTest              test;
    PrCutterStatus      cutterStatus;
    PrStationID         id; 
    PrLabelSize         labelSize;
    PrDotStatus         dotStatus;
    PrDotWear           dotwear;
    PrGapCalCmplt       gapCalCmplt;
    PrSetTime           setTimer;
    PrStopTime          stopTimer;
}PrMessage;

#endif