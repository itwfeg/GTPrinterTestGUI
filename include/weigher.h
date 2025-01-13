#ifndef WEIGHER_H
#define WEIGHER_H
#include <stdbool.h>


#define WEIGHT_COMPENSATION

#define RESOLUTION_100K 100000.0 
#define RESOLUTION_1M   1000000.0 
#define RESOLUTION_10M  10000000.0 
#define RESOLUTION_100  100.0


typedef struct
{
    double g;               // Gravity
    double m;               // Pounds/Counts conversion factor

    double zerovalue;       // Counts at zero load.
    double spanvalue;       // Cell span (max load - zero load)
    double Z0;              // Electronic offset
    double D;               // Dead weight.

    double x2;              // Polynomial for Front-Back tilt correction    // RESOLUTION_10M
    double x1;              // Polynomial for Front-Back tilt correction    // RESOLUTION_100K
    double x0;              // Polynomial for Front-Back tilt correction

    double y2;              // Polynomial for Left-Right tilt correction    // RESOLUTION_10M
    double y1;              // Polynomial for Left-Right tilt correction    // RESOLUTION_100K
    double y0;              // Polynomial for Left-Right tilt correction

    double xrot[3];         // Accel. Rotation Matrix
    double yrot[3];         // Accel. Rotation Matrix
    double zrot[3];         // Accel. Rotation Matrix

    double xgain;           // X Gain Factor
    double ygain;           // Y Gain Factor

    double g_coeff;         // Gravity Coefficient
}fit;


/* Changing counts processing for AD7191 (1/2 the counts at given input
     compared to TI1232), and 20Kg loadcell vs 15Kg */
/* AddLaterW2 get rid off this #define after code is stable */
#define AD7191_COUNTS_PROCESS

/* slope types */
#define PLUS  0xFF
#define MINUS 0x00

/* time in mS between unsolicited Weigher status messages when in
   "Weight Tracking Mode" */
#define N_READINGS_PER_SECOND 33

/* Initial Zero Setting Mechanism (IZSM) limit calculation */
#define POWERUP_MAINTENANCE_ZONE_LIMIT  (config.max_weight / 10)

/* Operate Maintenance Zone Bounds is 2% of scale capacity. */
#define OPERATE_MAINTENANCE_ZONE_LIMIT  ((config.max_weight * 2) / 100)


//v0.5 Updated the following three sets of #defines
/* One quarter graduation within zero can be thought of as true zero */
/* 300000 counts = 15Kg or 30lb. 2g = 40 counts, 0.005lb = 50 counts */
#define ONE_QUARTER_GRADUATION_METRIC_DR     10  //Dual Range
#define ONE_QUARTER_GRADUATION_AVOIR_DR      12  //Dual Range, 12.5 is actual value
#define ONE_QUARTER_GRADUATION_METRIC_SR     20  //Single Range
#define ONE_QUARTER_GRADUATION_AVOIR_SR      25  //Single Range


/* One half graduation within zero has no polarity */
/* 300000 counts = 15Kg or 30lb. 2g = 40 counts, 0.005lb = 50 counts */
#define ONE_HALF_GRADUATION_METRIC_DR   19  // Dual Range, 19 is 1/4. Compensate for noise 
#define ONE_HALF_GRADUATION_AVOIR_DR    24  // Dual Range, 25 is 1/4. Compensate for noise 
#define ONE_HALF_GRADUATION_METRIC_SR   38  // Single Range, 40 is 1/4. Compensate for noise 
#define ONE_HALF_GRADUATION_AVOIR_SR    48  // Single Range, 50 is 1/4. Compensate for noise 

/* Automatic zero maintenance limit is 0.5 graduation of a scale dimension */
/* 300000 counts = 15Kg or 30lb. 2g = 40 counts, 0.005lb = 50 counts */
#define AUTO_ZERO_MAINTENANCE_LIMIT_METRIC_DR   17  // Dual Range
#define AUTO_ZERO_MAINTENANCE_LIMIT_AVOIR_DR    26  //Carlos changed based on ATMEL Project  22  // Dual Range
#define AUTO_ZERO_MAINTENANCE_LIMIT_METRIC_SR   34  // Single Range
#define AUTO_ZERO_MAINTENANCE_LIMIT_AVOIR_SR    44  // Single Range
       
/* This value used to be a weigher config that was never changed */
/* As per Kraig/Larry, we can make this value a constant rather than relying on a config
   value which can be changed for whatever reason */
#define AUTO_ZERO_MOTION_LIMIT  15


/* Weigher interrupt states - historically there have been more states. */
enum weigher_interrupt_states
{
    READ_WEIGHER,
};
typedef enum weigher_interrupt_states WeigherInterruptStates;


/* A/D count queue element type definition */
typedef struct {
     unsigned long  avgFilterCounts;  /* Avg Filter <- FIRCounts */
     unsigned long  firCounts;        /* FIR Filter <- AtoDCounts */
     unsigned long  rawCounts;       /* unmodified AtoD Counts */
}AD_Counts;


/* Number of FIR Filter "taps" */
#define NUM_TAPS 15  
#define MAX_FILTER_READINGS 10
/* Weight State elements type definition */
struct weight_state_type
{                           
    unsigned long    zeroedCalibrCounts; /* Zeroed Calibrated Counts */
    unsigned long    calibratedCounts;   /* Unzeroed Calibrated Counts */
    unsigned short   status;         
    unsigned char    slope;
    unsigned long    avgFilterCounts;    /* Filtered, uncalibrated counts */
    unsigned long    rawCounts;         /* Unmodified AtoD Counts */
    unsigned long    zeroedTiltCompCounts;      /* raw + Accl compensation */
};
typedef struct weight_state_type WeightStateType;

/* Weight Status message queue element type definition */
struct weight_status_type
{
    unsigned short  status;
    unsigned long   ZeroedCalibrCounts;
    unsigned long   CalibratedCounts;
};
typedef struct weight_status_type WeightStatusType;

/* Configuration Filter Speed Definitions. */
#define NORMAL_FILTER_SPEED     0 
#define SLOW_FILTER_SPEED       1
#define FAST_FILTER_SPEED       2

/* Weigher Types */
#define PRIMARY_WEIGHER         0
#define SECONDARY_WEIGHER       1

/* Load Cell IDs */
#define LOAD_CELL_15KG          0
#define LOAD_CELL_20KG          1
#define LOAD_CELL_30KG          2
#define LOAD_CELL_45KG          3
#define LOAD_CELL_150KG         4
#define LOAD_CELL_200KG         5

/* Weigher Flags (Configuration) */
#define WEIGH_MODE_AVOIR                 0x01
#define WEIGH_MODE_METRIC                0x00
#define DUALRANGE_NETWEIGHTLIMITCHECK    0x02   
#define DUALRANGE_GROSSWEIGHTLIMITCHECK  0x00

typedef enum 
{
    _PERMANENT,
    _TEMPORARY,
    _DEFAULT,
    _CALINPROGRESS = 0x8000
}ConfigType;


/* Status Bit Masks */
#define SMALL_MOTION                            0x0001
#define LARGE_MOTION                            0x0002
#define NEGATIVE_GROSS_WEIGHT                   0x0004
#define WITHIN_QUARTER_GRADUATION_OF_ZERO       0x0008
#define OVER_GROSS_WEIGHT_LIMIT                 0x0010
#define ZERO_OUT_OF_MAINTENANCE_ZONE            0x0020
#define SCALE_WARMUP                            0x0040
#define NEW_ZERO                                0x0080
#define WEIGHER_CALIBRATION_FAILED              0x0100
#define WEIGHER_EEP_READ_FAILED                 0x0200
#define WEIGHER_DISABLED                        0x0400
#define WEIGHER_KEY_PRESSED                     0x0800
#define WEIGHER_VM_EEP_COMM_FAILURE             0x1000//ValueMax
#define WEIGHER_VM_EEP_CHECKSUM_FAILURE         0x2000//ValueMax
#define WEIGHER_VM_ACCL_COMM_FAILURE            0x4000//ValueMax
#define WEIGHER_VM_MAX_TILT_REACHED             0x8000//ValueMax

#define VM_MAX_TILT                             3.0//degrees


/* Bit definition used in the CONFIG message disposition to */
/* indicate that weigher calibration is in progress.        */
#define CALIBRATION_IN_PROGRESS                 0x8000

#endif

