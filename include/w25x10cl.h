#ifndef _W25X10CL_H_
#define _W25X10CL_H_
#include "fsl_lpspi.h"
#include "fsl_common.h"

//#define MANAGE_STATISTICS	/* Enable to count messages and errors */

#define LPSPI1_CLOCK_FREQ       105600000UL
#define SPI_PORT_MASK           0xffffc0ff

/*mfg and device identification */
#define DEVICE_ID             0x3011   // Winbond Serial Flash

#define CHIP_ID_SIZE          3

/* memory characteristics and layout by sectors (units: bytes)*/
#define PAGE_SIZE             4096
#define SECTOR_SIZE           32768     /*the data sheet defines this as a block
                                          but we call it a sector*/
#define NUM_PAGES_PER_BLOCK   8       
#define BUFFER_SIZE           (CMD_SIZE + ADDR_SIZE + (PAGE_SIZE / 16))

#define WRITE_BLOCK_SIZE      256

/*block address layout */
#define SECTOR0_BASE_ADDR     0 
#define SECTOR1_BASE_ADDR     (SECTOR_SIZE * 1)
#define SECTOR2_BASE_ADDR     (SECTOR_SIZE * 2) 
#define SECTOR3_BASE_ADDR     (SECTOR_SIZE * 3)

typedef enum
{
  SECTOR_0,
  SECTOR_1,
  SECTOR_2,
  SECTOR_3 
}FLASH_SECTORS;

typedef enum
{
  NO_SECTOR_PROTECT,
  SECTOR_PROTECT_3,
  SECTOR_PROTECT_2_3,
  ALL_SECTORS 
}SECTOR_PROTECT;

#define PROTECT_BITS_OFFSET     2
#define ADDR_SIZE               3


/* commands:  every command sequences starts with a one byte command code and is sent MSB first*/
typedef enum
{
  FLASH_NO_COMMAND,
  FLASH_READ_IDENT,          
  FLASH_READ_STATUS_REG,
  FLASH_READ_DATA_BYTES,
  FLASH_READ_DATA_BYTES_HS,
  FLASH_READ_ES, 
  FLASH_WRITE_EN,
  FLASH_WRITE_DS,
  FLASH_ENABLE_STATUS_REG,
  FLASH_WRITE_STATUS_REG,
  FLASH_PAGE_PROGRAM,
  FLASH_PAGE_ERASE,
  FLASH_SECTOR_ERASE,
  FLASH_BULK_ERASE,
  FLASH_RELEASE_PWR_DWN, 
}FLASH_CMD;

#define CMD_SIZE                1

/*command codes */
#define READ_IDENT              0x9f    /* 0x9f reads the jedec id: 0xef1031  0x90 reads the identification id of the serial flash */
#define READ_STATUS_REG         0x05    /*reads the status register of the serial flash */
#define READ_DATA_BYTES         0x03    /*reads the identification id of the serial flash */
#define READ_DATA_BYTES_HS      0x0b
#define READ_ES                 0xab
#define RELEASE_POWR_DWN        0xab

#define WRITE_EN                0x06
#define WRITE_DS                0x04
#define WRITE_STATUS_REG        0x01
#define ENABLE_WRITE_STATUS_REG 0x50    /* must enable before setting WRITE_STATUS_REG */

#define PAGE_PROGRAM            0x02
#define SECTOR_ERASE            0x52
#define PAGE_ERASE              0x20
#define BULK_ERASE              0xc7

#define DUMMY_BYTE              0


/*status register format*/
#define SRWD_MASK                0x80           /* write disabled bit */
#define AUTO_INC_MASK            0x40
#define TOP_BOTTOM_PROTECT_MASK  0x20
#define RESERVED_MASK            0x10           /* indicates which blocks are write protected */
#define BLOCK_PROT1_MASK         0x08
#define BLOCK_PROT0_MASK         0x04
#define WRITE_ENABLE_MASK        0x02           /* status of internal write enable latch */
#define FLASH_STATUS_MASK        0x01           /* indicates command in progress */

#define SLAVE_SELECT_FLASH       0x00010000

/*error codes*/
static int _ERRORSPIMODE    =      -1;
static int _ERRORSPIOVER    =      -2;
static int _DATAERRORSIZE   =      -3;
static int _WRITEENABLE     =      -4;
static int _WAITERROR       =      -5;
static int _BUSY            =      -6;
static int _PROCERROR       =      -7;

typedef struct 
{
  unsigned int    chipId;
  uint8_t         statusRegister;               /* flash status register */
  bool            writeEnabled;
  bool            flashBusy;  
  bool            sectorLocked;
  bool            dataReady;
  FLASH_CMD       currentCommand;
  int             transferSize;                 /* number of bytes to complete command sequence */
  uint8_t         transferBuffer[BUFFER_SIZE];  /* data to transmit in transfer */  
  uint8_t         sectorBuffer[BUFFER_SIZE];    /* data received in transfer */
  uint8_t         *pData;                       /* destination of data read */
  int             length;                       /* data length to read or write */
  int             error;   
}SERIAL_FLASH_CNTRL;



/* public functions */
bool initializeSerialFlash();
bool isInterfaceInitialized();
void resetInterface( void ); 
void releasePowerDwn( void );
void lpspi1Callback( LPSPI_Type *base, lpspi_master_handle_t *handle, 
                     status_t status, void *userData );

void lpspi4Callback( LPSPI_Type *base, lpspi_master_handle_t *handle, 
                     status_t status, void *userData );

bool readSerialFlash( uint32_t address,  uint8_t *pData, int size );
void *getReadData();
bool writeSerialFlash( uint8_t *pData, uint32_t address, int size, FLASH_SECTORS sector );
bool writeSerialFlashNE( uint8_t *pData, uint32_t address, int size, FLASH_SECTORS sector );
bool erasePage( uint32_t address, FLASH_SECTORS sector );
bool eraseSector( uint32_t address, FLASH_SECTORS sector );
bool validateErasedPage( uint32_t address, FLASH_SECTORS sector );
int getLastFlashError( void );


int isWriteEnabled( SERIAL_FLASH_CNTRL *pSerialFlash );
int setWriteEnable( SERIAL_FLASH_CNTRL *pSerialFlash );
int isSectorLocked( SERIAL_FLASH_CNTRL *pSerialFlash, FLASH_SECTORS sectorNum );
int setSectorLock( SECTOR_PROTECT sector );
int clearSectorLock( SERIAL_FLASH_CNTRL *pSerialFlash, FLASH_SECTORS sector );
uint32_t getSectorBaseAddress(FLASH_SECTORS sector);
void setupTransfer( FLASH_CMD command, SERIAL_FLASH_CNTRL *pSerialFlash, uint8_t *pAddr, uint8_t *pData, int length );
void processCommand( SERIAL_FLASH_CNTRL *pSerialFlash );
void setDataReady( void );
bool getDataReadyStatus( void );
void setDataTransferIndex( void );

/* private functions */
static void handleReadId( SERIAL_FLASH_CNTRL *pSerialFlash );
static void handleReadStatus( SERIAL_FLASH_CNTRL *pSerialFlash );
static void handleReadData( SERIAL_FLASH_CNTRL *pSerialFlash );
static void handleReadDataHS( SERIAL_FLASH_CNTRL *pSerialFlash );
static void handleReadSignature( SERIAL_FLASH_CNTRL *pSerialFlash ); 
static void handleWriteEnable( SERIAL_FLASH_CNTRL *pSerialFlash );
static void handleWriteDisable( SERIAL_FLASH_CNTRL *pSerialFlash );
static void handleWriteStatus( SERIAL_FLASH_CNTRL *pSerialFlash );
static void handleEnableStatusReg( SERIAL_FLASH_CNTRL *pSerialFlash );
static void handlePageProgram( SERIAL_FLASH_CNTRL *pSerialFlash );
static void handleSectorErase( SERIAL_FLASH_CNTRL *pSerialFlash );
static void handleBulkErase( SERIAL_FLASH_CNTRL *pSerialFlash );
static void waitForCompletion( SERIAL_FLASH_CNTRL *pSerialFlash );
static void handleReleasePowerDwn( SERIAL_FLASH_CNTRL *pSerialFlash );
static int isFlashBusy( SERIAL_FLASH_CNTRL *pSerialFlash );
/* primitives */
static void intializeFlashInterface( void );
static void writeFlashByte( void );
static void eraseFlashSector( SERIAL_FLASH_CNTRL *pSerialFlash );
static void eraseFlash( SERIAL_FLASH_CNTRL *pSerialFlash );

/*unit tests */
static void thisUnitTest();
bool testReadWritePage();
bool testWriteProtection();

#endif /* _SST25VF010A_H_ */
