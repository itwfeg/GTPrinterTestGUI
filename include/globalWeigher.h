#ifndef GWEIGHER_H
#define GWEIGHER_H
#include <stdbool.h>

typedef enum 
{
    G_UNKNOWN_WEIGHER0 = 0, //WmNone,
    G_UNKNOWN_WEIGHER1 = 1, //WmESS,
    G_UNKNOWN_WEIGHER2 = 2, //WmU2000S1,
    G_UNKNOWN_WEIGHER3 = 3, //WmU2000S3,
    G_UNKNOWN_WEIGHER4 = 4, //WmUWS,
    G_UNKNOWN_WEIGHER5 = 5, //WmQuantHC,
    G_UNKNOWN_WEIGHER6 = 6, //WmESC,
    G_UNKNOWN_WEIGHER7 = 7, //WmEPSA,
    G_UNKNOWN_WEIGHER8 = 8, //WmHLX1,
    G_UNKNOWN_WEIGHER9 = 9, //WmEzUSB,
    G_UNKNOWN_WEIGHER10 = 10, //WmAWS,
    G_UNKNOWN_WEIGHER11 = 11, //WmEPSA2,
    G_UNKNOWN_WEIGHER12 = 12, //WmAWS2,
    G_UNKNOWN_WEIGHER13 = 13, //WmEPSAC2,
    G_UNKNOWN_WEIGHER14 = 14, //WmSASCALE1,
    G_UNKNOWN_WEIGHER15 = 15, //WmSASCALE2,
    G_UNKNOWN_WEIGHER16 = 16, //WmKPCWeigher,
    G_SERVICE_SCALE_WEIGHER  = 17, //WmRTGlobal,
    GL_SERVICE_SCALE_WEIGHER = 18,  //WmRTGlobalLegacy,     /* extended from previous models */
    G_UNKNOWN_WEIGHER19 = 19 //endWeigherModel
}WeigherModel;

typedef struct 
{
    long             center_of_maintenance_zone;        /* set during calibration  */
    long             scale_factor;                      /* no longer used by WmHLX1,WmAWS2DR,WmAWS2 or WmKPCANWeigher */
    long             max_weight;                        /* max weight the scale is rated in calibrated counts */
    long             gain_factor;                       /* from calibration - used to scale the counts */
    unsigned short   prepack_motion_count;              /* time interval to look for small motion when in prepack mode */ 
    unsigned short   initialize_zero_time;              /* time the weight must be stable before a new zero can be captured. increments of 1/3 seconds */
    unsigned short   small_motion_limit;                /* small motion counts limit */
    unsigned short   large_motion_limit;                /* large motion counts limit */
    unsigned short   large_motion_count;                /* time interval to look for largue motion */
    unsigned short   small_motion_count;                /* time interval to look for small motion when not in prepack mode */
    unsigned short   no_motion_count;                   /* time there has to be no motion before leaving the "small motion" state */    
    unsigned char    filter_speed;                      /* configures filter for SLOW, NORMAL, or FAST conversions */ 
    unsigned char    pad0;
    unsigned char    weigher_type;                      /* primary or secondary weigher */    
    unsigned char    flags;                             /* weigh Mode: Metric, Avoir, Dual Range */    
    unsigned short   pad2;
    unsigned long    last_calibration_date;             /* cat1 information */
    unsigned short   number_of_calibrations;            /* cat1 information */
    unsigned short   number_of_configurations;          /* cat1 information */
    unsigned long    last_configuration_date;           /* cat1 information */
    long             min_weight_to_print; 
    unsigned short   azsm_motion_limit;                 /* auto zero maint motion counts limit. TFink 5/14/24 - this is no longer used. Hard coded with AUTO_ZERO_MOTION_LIMIT */   
    WeigherModel     weigher_model;    
    bool             value_max_on_off;                  /* 1 - valueMax feature is Enabled, 0 - valueMax feature is disabled */
    
}WgConfiguration;



typedef enum 
{
    SCALE_PREPACK_MODE,
    SCALE_WEIGHT_TRACKING_MODE,
    SCALE_DUMB_MODE,
    SCALE_MANUAL_MODE
}WeighMode;



#endif