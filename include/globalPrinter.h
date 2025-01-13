#ifndef GPRINTER_H
#define GPRINTER_H
#include <stdbool.h>

/* printEngine states */
typedef enum
{
    ENGINE_IDLE,      
    ENGINE_PRINTING,                        
    ENGINE_STEPPING,                        
    ENGINE_WAITING,                         
    ENGINE_INITIALIZING,
    ENGINE_SWITCHING,
    ENGINE_DISABLED,
    ENGINE_CALIBRATING,
    ENGINE_TEST,
    ENGINE_MEASURING,
    ENGINE_CUTTING,
}EngineStates;

typedef enum 
{
    WIDE_LABEL_STOCK,
    NARROW_LABEL_STOCK,
    UFW_LABEL_STOCK,
    EXT_LABEL,
    GLOBAL_LABEL_STOCK,
    UNKNOWN_STOCK
}LabelWidth;

#define WideLabelBit                    0x10

#define WIDE_STOCK_WIDTH_DOTS           448     /* 2.25 inches */
#define NARROW_STOCK_WIDTH_DOTS         304     /* 1.50 inches */
#define UFW_STOCK_WIDTH_DOTS            640     /* 3.00 inches */

typedef enum 
{
    PRIMARY_PRINTER,
    SECONDARY_PRINTER
}StationID;

/* Fatal Error Bits */
#define NO_ERROR                        0x00
#define HEAD_UP                         0x01
#define THERMAL_SHUTDOWN                0x02
#define MEDIA_SHUTDOWN                  0x04
#define CASSETTE_MISSING                0x08
#define UNKNOWN_PH                      0x20
#define PH_OVERCURRENT                  0x40
#define OUT_OF_PRINTDATA                0x80

/* Printer Calibration Items. */
enum head_status
{
    ALL_DOTS_GOOD,
    DOTS_MIXED,
    ALL_DOTS_BAD,
    DOT_FAILED,
    DOT_WEAR_ABORTED_CASSETTE_OPEN	//4
};
typedef enum head_status HeadStatus;

typedef enum
{
  _REFLECTIVE,
  _SHOOTTHROUGH
}MediaSensorType;
#pragma pack( 1 )
typedef struct pr_configuration 
{
    StationID           instance;                                
    LabelWidth          label_width;                                 
    unsigned char       media_sensor_adjustment;        /* media or shoot sensor cal value  */                     
    unsigned char       out_of_media_count;                          
    unsigned short      contrast_adjustment;                         
    short               expel_position;                 /* was config3 */                                  
    short               peel_position;                  /* was config0 */                                 
    short               retract_position;               /* was config4 */                                 
    short               media_sensor_type;                           
    unsigned short      sla_eject_line;                              
    short               verticalPosition;               /* vertical print position */
    unsigned short      backingPaper;                   /* shoot through backing paper only counts */
    unsigned short      backingAndlabel;                /* shoot through backing paper and label counts */
    unsigned char       labelCalCnts;                   /* label taken label present cal counts */
    unsigned char       noLabelCalCnts;                 /* no label present cal counts */
    bool                cutterEnabled;
	unsigned char       takeup_sensor_drive_current;		/* Takeup sensor drive current cal value  */                     
	unsigned short      takeup_sensor_max_tension_counts;	/* Takeup sensor max tension A/D counts */
	unsigned short      takeup_sensor_min_tension_counts;	/* Takeup sensor min tension A/D counts */ 
}Pr_Config;                                                              
#pragma pack( push,1 )
#pragma pack( pop )

#define CUTTER_MAGIC_KEY_INSTALLED        0x5A
#define CUTTER_MAGIC_KEY_NOT_INSTALLED    0x00
#define CUTTER_MAGIC_KEY_BANK             0xFF

typedef enum 
{
    PRINTER_PORTRAIT_MODE,
    PRINTER_LANDSCAPE_MODE
}PrintMode;

typedef enum 
{
    PRINTHEAD_CALIBRATION,
    MEDIA_SENSOR_CALIBRATION,
    LTS_CALIBRATION
}PrinterCal;


#endif
