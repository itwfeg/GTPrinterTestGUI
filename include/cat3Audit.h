#ifndef CAT3_AUDIT
#define CAT3_AUDIT
#include "wgMessages.h"
#include <stdbool.h>

/* *******************Cat 3 audit trail revision history **********************/
/*      changed date time arrays to epoch time:  06282016                     */       
/*                                                                            */
/*                                                                            */           
/******************************************************************************/


typedef enum
{
    CALIBRATION,
    CONFIGURATION,
    UPGRADE  
}AuditEvent;

/* sealable parameters that change on a calibration event */
typedef struct
{
    long center_of_maintenance;
    long previous_center_of_maintenance;
    long gain_factor;
    long previous_gain_factor;
    long zero_reference;
    long previous_zero_reference;
    unsigned long calibration_gain_coefficient;
    unsigned long previous_calibration_gain_coefficient;
    unsigned long calibration_offset_coefficient;
    unsigned long previous_calibration_offset_coefficient;
    unsigned long epochTime;
}CalibrationEvent;   

/* sealable parameters that change on a configuration event */
typedef struct 
{
    unsigned char filterSpeed;
    unsigned char previous_filterSpeed;
    long maxWeight;
    long previous_maxWeight;
    long min_weight_to_print;
    long previous_min_weight_to_print;
    unsigned char weigherModel;
    unsigned char previous_weigherModel;
    unsigned char division_size;
    unsigned char previous_division_size;
    unsigned char weigh_mode;
    unsigned char previous_weigh_mode;
    unsigned long epochTime;
}ConfigurationEvent;

/*configuration fields that can change */
typedef enum
{
    CENTER_OF_MAINTENANCE_ZONE,
    SCALE_FACTOR,
    MAX_WEIGHT,
    GAIN_FACTOR,
    ZERO_REFERENCE,
    CAL_GAIN_COEFFICIENT,
    CAL_OFFSET_COEFFICIENT,
    FILTER_SPEED,
    MIN_WEIGHT_PRINT,
    WEIGHER_MODEL,
    DIVISION_SIZE,
    SOFTWARE_UPDATE,
    GRAVITY_CONSTANT,
    FRACTIONAL_PRICING,
    POUNDS_FOR_MULTIPLIER,
    SERVICE_SWITCH_PRESSED,
    WEIGHER_MODE,
	DECIMAL_SEPARATOR,
	MONETARY_RULE,
	TOTAL_PRICE_DIGITS,
	UNIT_PRICE_DIGITS,
	ROUNDING_FACTOR,
	ROUNDING_METHOD,
	PRIMARY_MONETARY_SYMBOL,
	PRIMARY_MONETARY_DECIMAL_DIGITS,
	SECONDARY_MONETARY_SYMBOL,
	SECONDARY_MONETARY_DECIMAL_DIGITS
}CFGParamID;

typedef enum
{
  LOCAL_EVENT,
  REMOTE_EVENT
}AuditType;

/*record type for cat 3 audit trail */
#pragma pack( 4 )
typedef struct
{
    unsigned char   recordTag;                      /*record present (0) or not (0xff) */
    AuditEvent      event;
    CFGParamID      parameterId;
    unsigned long   epochTime;
    AuditType       eventType;                  
    unsigned char   unused[8];
    unsigned long   oldParamValue;
    unsigned long   newParamValue;
    unsigned long   checksum;
}AuditRecord;
#pragma pack( push,4 )
#pragma pack( pop )

typedef struct
{
    bool blank;                                              /* is page blank */
    AuditRecord records[ (4096 / sizeof(AuditRecord))  ];    /* page buffer */
}AuditPage;

#define NUM_RECORDS_PER_PAGE            (4096 / sizeof(AuditRecord))            
#define BLANK_RECORD                    0xFF
#define TOTAL_NUMBER_SECTORS            2
#define TOTAL_RECORD_STORAGE_SIZE       TOTAL_NUMBER_SECTORS * ( NUM_PAGES_PER_BLOCK * NUM_RECORDS_PER_PAGE )

typedef struct
{
    unsigned char   initialized;
    unsigned long   addr;                           /* current empty record address in the serial flash */
    unsigned char   currentSector;                  /* current sector number we are recording in */
    unsigned char   currentPage;                    /* current page number within the current block we are recording in */
    unsigned short  numOfTotalRecords;              /* total number of records within serial flash */
    unsigned short  numOfCalRecords;                /* total of calibration records within serial flash */    
    unsigned short  numOfConfigRecords;             /* total of configuration records within serial flash */
    unsigned short  numOfUpgradeRecords;            /* total of upgrade records within serial flash */
    unsigned short  centerOfMainCounter;            /* total number of times center of maintenance has changed */       /* requirement  8.24c */     
    unsigned short  gainFactorCounter;              /* total number of times gain factor has changed */         
    unsigned short  zeroReferenceCounter;           /* total number of times zero reference has changed */
    unsigned short  gainCoefficientCounter;         /* total number of times gain coefficient has changed */
    unsigned short  offsetCoefficientCounter;       /* total number of times offset coefficient has changed */
    unsigned short  filterSpeedCounter;             /* total number of times filter speed has changed */    
    unsigned short  maxWeightCounter;               /* total number of times max weight has changed */
    unsigned short  minWeightPrintCounter;          /* total number of times min weight to print has changed */
    unsigned short  weigherModelCounter;            /* total number of times weigher model has changed */
    unsigned short  divisionSizeCounter;            /* total number of times division size has changed */
    unsigned short  weigherModeCounter;             /* total number of times weigher mode has changed */
}AuditMGR;

#define BLANK_INDEX                     0xFFFFFFFF
#define MAX_AUDIT_RECORDS               500    
#define MAX_CAT3_COUNT                  65535

/*********************** public Cat3 Audit Manager functions********************/
bool initializeAuditMGR();
void resetAuditMGR();
void initializeEvents(WgConfiguration *pConfig);
void updateDateTimeEvents(unsigned long time);
void getAuditMgrStats(WgCat3Statistics *pStatistics);
unsigned char getAuditManagerCurrentPage();
unsigned char getReadPageIndex();
unsigned char getReadPageNumber();
bool auditManagerSaveRecord(AuditRecord *pRecord);
void saveCalibEvents(WgConfiguration *pConfig);
void saveConfigEvents(WgConfiguration *pConfig);
void incAuditMGRRecordCount();                                  
void eraseAuditLog();

void setReadPageNumber(unsigned char number);
void setReadPageIndex(unsigned char index);
void readRecords(unsigned char *pRecords);
void readRecord(AuditRecord *pRecord, unsigned int index);
unsigned char getReadPageIndex();
bool readSerialAuditRcd( unsigned long addr, AuditRecord *pAuditRecord  );
void readAuditPage(unsigned long addr);
bool isAuditPageBlank();
unsigned long findEmptyRcd();
unsigned long getAuditStartAddress();
bool isInRange(unsigned long addr);
unsigned short getNumberOfRecords(AuditEvent event);
unsigned short getNumberOfRecordsPage(AuditEvent event);
bool isRecordVaild(AuditRecord *pRecord);
bool isRecordZeroValid();
unsigned long calcRecordCheckSum(AuditRecord *pRecord);



/******************** private Cat3 Audit Manager functions *********************/
static void updateAuditMGR(unsigned long addr, AuditEvent event);
static bool writeAuditMGR(AuditMGR *pAuditMGR);
static void auditTrailRollOver();
static bool writeSerialAuditRcd( unsigned long addr, AuditRecord *pAuditRecord );
static bool eraseAuditPage(unsigned long addr);
static unsigned char getSectorNumber(unsigned long addr);
static unsigned char getPageNumber(unsigned long addr);
static unsigned long getPageBaseAddress();

/*********************Cat3 Unit Testing*****************************************/
void unitCat3TestRollover();
void unitTestResetCat3AuditLog();
void readSectorTest();
void unitCat3Test();

#endif