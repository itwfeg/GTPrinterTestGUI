#ifndef INTERNALMESSAGES_H
#define INTERNALMESSAGES_H
#include <stdbool.h>

typedef enum 
{
    _I_CUTTER_UNDEFINED,
    _I_CUTTER_CUT_CMD,
    _I_CUTTER_HOME_CMD,
    _I_CUTTER_REQ_STATUS,
    _I_CUTTER_REQ_VERSION,
    _I_CUTTER_READY_FOR_CMD    
}ICMsgType;

typedef struct
{
    ICMsgType           msgType;
    
}ICutterGeneric;

typedef struct
{
    ICMsgType           msgType;
    unsigned short      status;
    bool                isHome;
    bool                isError;
}ICutterStatus;


/* version string sizes */
#define MAX_CPRODUCT_SIZE                   20
#define MAX_CFIRMWARE_NUM_SIZE              12
#define MAX_CFIRMWARE_ISSUE_SIZE            8
#define MAX_CDATE_SIZE                      11
#define MAX_CTIME_SIZE                      8

typedef struct
{
    ICMsgType           msgType;
    unsigned char       productName[ ( MAX_CPRODUCT_SIZE + 1 )];
    unsigned char       firmware[ ( MAX_CFIRMWARE_NUM_SIZE + 1 )];
    unsigned char       issueDate[ ( MAX_CFIRMWARE_ISSUE_SIZE + 1 ) ];
    unsigned char       date[ ( MAX_CDATE_SIZE + 1 ) ];
    unsigned char       time[ ( MAX_CTIME_SIZE + 1 ) ];      
}ICutterVersion;

typedef union 
{
    ICutterGeneric      generic;
    ICutterStatus       status;
    ICutterVersion      version;        
}ICMessages;



#endif
