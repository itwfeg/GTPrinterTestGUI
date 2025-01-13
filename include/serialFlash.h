#ifndef SERIALFLASH_H
#define SERIALFLASH_H
#include "globalWeigher.h"
#include "globalPrinter.h"
#include <stdbool.h>

typedef enum
{
    AD_AD7191,
    AD_TI1232 
}ADType;

#define PART_NUM_SIZE       9
#define BOARD_REV_SIZE      2
#define SERIAL_NUM_SIZE     11

typedef struct
{
    bool                readOnly;          /*once fields are programmed, they will become read only */
    unsigned char       partNumber[PART_NUM_SIZE];
    unsigned char       boardRevision[BOARD_REV_SIZE];
    unsigned long       dateOfManufacture;      /* epoch time */
    unsigned char       serialNumber[SERIAL_NUM_SIZE];
    ADType              type; 
    unsigned long       checkSum;
}WeigerMFGInfo;

/* PrInfo structure added since cutter installed bit is not reliable.
   if the cutter interlock is open when scale boots then the install bit
   is not reliable due to power removal when interlock is opened. */
typedef struct
{
    unsigned char       cutterInstalled;        /* magic key if cutter is installed */  
    unsigned long       dateOfManufacture;      /* epoch time */
    unsigned long       numOfPrints;            /* number of print job on head */         
}PrInfo;

/*checksums need to be placed on seperate page from the data due to messaging issues.
 these checksums with be placed on page 4 within block 3*/
typedef struct 
{
  unsigned short  wgConfigSum;
  unsigned short  wgInfoSum;
  unsigned short  prConfigSum;
  unsigned short  USBDescSum;
  unsigned short  pagesum;            //checksum for this page.
  unsigned short  sysConfig;
}FPMBLC3Checksums;


bool initSerialFlashMutex( void );
bool getLockSerialFlash( void );
void releaseLockSerialFlash( void );
bool getSerialWgConfiguration( WgConfiguration *pWeighConfig );
bool setSerialWgConfiguration( WgConfiguration *pWeighConfig );
bool getSerialWeigherMFGInfo( WeigerMFGInfo *pWeigherMFGInfo );
bool setSerialWeigherMFGInfo( WeigerMFGInfo *pWeigherMFGInfo );
bool setSerialWgDfltConfig( WgConfiguration *pWeighConfig );

bool getPageChecksums(FPMBLC3Checksums *pChecksums);
bool setPageChecksums(FPMBLC3Checksums *pChecksums);


bool getSerialPrConfiguration(Pr_Config *pPrConfig);
bool setSerialPrConfiguration(Pr_Config *pPrConfig);
bool setSerialPrDfltConfiguration(Pr_Config *pPrConfig);

bool getSerialPrInfo(PrInfo *prInfo );
#if 0
bool setSerialPrInfo( PrInfo *prInfo );
#endif
bool eraseSerialCutterBit( void );
unsigned short calculateChecksum (void *buffer, unsigned long size);

void testConfiguration();
#endif