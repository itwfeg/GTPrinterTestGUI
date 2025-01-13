#include "w25x10cl.h"
#include "pin_mux.h"
#include "fsl_gpio.h"
#include <string.h>
#include "fsl_debug_console.h"

#ifdef VANTRON_SBC_RK3568_NXP24_ARK

static SERIAL_FLASH_CNTRL serialFlash;
static lpspi_master_handle_t  spi1Handle_;
static lpspi_master_handle_t  spi4Handle_;

//#define PROTOTYPE_BOARD
//#define TEST_UNIT_ENABLED

#ifdef TEST_UNIT_ENABLED
static void thisUnitTest();
#endif


/*! ****************************************************************************
      \todo need to determine which flash sector we will allow user to 
            change the write protection. This will be based on what the contents 
            of the section 9/21/12.

      \todo modify file to support the sst serial flash device. The flash has a 
            few slight differences from the m25p80 flash other than size and memory 
            layout 9/21/12.

            differences: writeSerialFlash using PAGE_PROGRAM command Auto Inc. 
                         need to add write disable command at the end of buffer.
                         
                         writing of the status register must be started with the 
                         following command sequence: ENABLE_WRITE_STATUS_REG, WRITE_STATUS_REG
                         setWriteEnable(), setSectorLock(), clearSectorLock() 


*/



/*! ****************************************************************************   
      \fn initializeSerialFlash(SERIAL_FLASH *pSerialFlash)                                                                
 
      \brief
        public: This function initializes the serial flash control object and
         the physical interface to the serial flash device. It verifies that 
         the device and interface are functional by reading the serial flash 
         chip id and returning a pass / fail status. 

      \param pSerialFlash pointer to the serial flash control instance

      \author
          Aaron Swift
*******************************************************************************/ 
bool initializeSerialFlash()
{
    /*intialize the control structure*/
    memset( &serialFlash, 0, sizeof(SERIAL_FLASH_CNTRL));
       
    /*init the SPI interface to the serial flash */
    intializeFlashInterface();

    releasePowerDwn();

    /*read the serial flash id to insure we are initialized */
    setupTransfer( FLASH_READ_IDENT, &serialFlash, NULL, NULL, 0 );
    if( serialFlash.error == 0 )
    {
        processCommand( &serialFlash );
    }
    
    /*read the status register */
    setupTransfer( FLASH_READ_STATUS_REG, &serialFlash, NULL, NULL, 0 );
    if( serialFlash.error == 0 )
    {
        processCommand( &serialFlash );
    }
    
#ifdef TEST_UNIT_ENABLED
    thisUnitTest();
#endif    
    return isInterfaceInitialized();
}


/*! ****************************************************************************  
      \fn bool isInterfaceInitialized()


      \brief
        public: Has the physical interface to the serial flash been initialized.
        The chip id is read from the device during initialization and is compared to 
        the known chip id.
      \return   

      \author
          Aaron Swift
*******************************************************************************/ 
bool isInterfaceInitialized()
{
    bool initialized = false;
  
    if( serialFlash.chipId == DEVICE_ID )
    {
        initialized = true;
    }
    return initialized;
}

/*! ****************************************************************************  
      \fn bool releasePowerDwn()


      \brief
        public: release the serial flash from power down state.
      \return   

      \author
          Aaron Swift
*******************************************************************************/ 
void releasePowerDwn( void )
{       
    setupTransfer( FLASH_RELEASE_PWR_DWN, &serialFlash, NULL, NULL, 0 );
    if( serialFlash.error == 0 ) 
    {
        processCommand( &serialFlash );
    }        
}

/*! ****************************************************************************   
      \fn bool readSerialFlash( uint32_t address, uint8_t *pData, int size )

      \brief
        public: This function reads the number of bytes from the serial flash 
                starting at the given address.                       

      \param address    begining address to start read
      \param pData      destination to copy data 
      \param size       amount of data to read
      
      \return result of read operation  

      \author
          Aaron Swift
*******************************************************************************/ 
bool readSerialFlash( uint32_t address,  uint8_t *pData, int size )
{
    bool status = true;
    while( status && (size > 0)) 
    {
        int sz;
        int bz = BUFFER_SIZE - CMD_SIZE - ADDR_SIZE;

        if( size > bz) 
        {
            sz = bz;
        } else {
            sz = size;
        }
        setupTransfer( FLASH_READ_DATA_BYTES, &serialFlash,  (uint8_t *)&address,  pData, sz );
        if( serialFlash.error == 0 ) 
        {
            processCommand( &serialFlash );
            if( serialFlash.error == 0 ) 
            {
                address += sz;
                pData += sz;
                size -= sz;
            } else {
                status = false;
            }          
        }
    }
    return status;       
}


/*! ****************************************************************************   
      \fn void *getReadData()

      \brief
        public: This function returns a pointer the SERIAL_FLASH_CNTRL's read buffer.

      \return pointer to read buffer 

      \author
          Aaron Swift
*******************************************************************************/ 
void *getReadData(int offset)
{
    if( serialFlash.error == 0 )  
    {  
        if( serialFlash.currentCommand == FLASH_READ_DATA_BYTES)
            return ( void * )&serialFlash.sectorBuffer[CMD_SIZE + ADDR_SIZE + offset];
        
        if( serialFlash.currentCommand == FLASH_READ_DATA_BYTES_HS)
            return ( void * )&serialFlash.sectorBuffer[CMD_SIZE + ADDR_SIZE + 1 + offset];
    }
    return NULL;
}


/*! ****************************************************************************   
      \fn uint64_t readUniqueId()

      \brief
        public: This function reads a factory-set read-only 64-bit number that 
        is unique to each component. If the component do not support this 
        command, the returned value is zero.                     

       \return result of read operation.  

      \author
          Philippe Corbes
*******************************************************************************/ 
uint64_t readUniqueId( void )
{
    uint64_t id = 0xFFFFFFFFFFFFFFFF;
    
    /*setup a read transfer to the serial flash */
    setupTransfer( FLASH_READ_UNIQUE_ID, &serialFlash,  NULL,  (uint8_t *)&id, sizeof(id) );
    if( serialFlash.error == 0 )
    {
        processCommand( &serialFlash );
    }

    return id;
}


/*! ****************************************************************************   
      \fn bool writeSerialFlash( uint8_t *pData, uint32_t address, int size )

      \brief
        public: This function writes bytes to the serial flash at the specified address.                       
                Step 1: check sector lock and unlock
                Step 2: set and verify write enable
                Step 3: erase page
                Step 4: write to serial flash

      \param pData      character pointer to the data
      \param address    begining address to start write
      \param size       amount of data to write
      
      \return result of write operation  

      \author
          Aaron Swift
*******************************************************************************/ 
bool writeSerialFlash( uint8_t *pData, uint32_t address, int size, FLASH_SECTORS sector )
{
    bool status = false;
    
    /*is the sector locked ?*/
    isSectorLocked(&serialFlash, sector);    
    if( serialFlash.sectorLocked )
    {
        /*unlock the write protection */
        clearSectorLock(&serialFlash, sector);        
    }

    /*verify the write enable is set */
    setupTransfer(FLASH_READ_STATUS_REG, &serialFlash, NULL, NULL, 0);
    processCommand(&serialFlash);    
    if( (serialFlash.statusRegister & WRITE_ENABLE_MASK) != WRITE_ENABLE_MASK )
    {
        setupTransfer( FLASH_WRITE_EN, &serialFlash, NULL, NULL, 0 );
        processCommand( &serialFlash );              
    } 
        
    /*erase sector per sector the flash before our write */
    uint32_t page_address = address;
    uint32_t end_address = address + size;
    while (page_address < end_address) {
        setupTransfer(FLASH_PAGE_ERASE, &serialFlash,  (uint8_t *)&page_address, NULL, 0 );
        processCommand(&serialFlash);
        page_address += PAGE_SIZE;
    }

    status = true;
    while( status && (size > 0)) 
    {
        /*check that the write enable is set */
        setupTransfer(FLASH_READ_STATUS_REG, &serialFlash, NULL, NULL, 0);
        processCommand(&serialFlash);
        if( ( serialFlash.statusRegister & WRITE_ENABLE_MASK ) !=  WRITE_ENABLE_MASK)
        {      
           setupTransfer(FLASH_WRITE_EN, &serialFlash, NULL, NULL, 0);
           processCommand(&serialFlash);
        }       

        /* program the serial flash */
        int sz;
        int bz = WRITE_BLOCK_SIZE;

        if( size > bz) 
        {
            sz = bz;
        } else {
            sz = size;
        }
        setupTransfer( FLASH_PAGE_PROGRAM, &serialFlash,  (uint8_t *)&address, pData, sz );
        if( serialFlash.error == 0 ) 
        {
            processCommand( &serialFlash );
            if( serialFlash.error == 0 ) 
            {
                address += sz;
                pData += sz;
                size -= sz;
            } else {
                status = false;
            }          
        }
    }

    return status;       
}


/*! ****************************************************************************   
      \fn bool writeSerialFlashNE( uint8_t *pData, uint32_t address, int size )

      \brief
        public: This function writes bytes to the serial flash at the specified address.
                Note: This function does not preform an erase before the write. It assumes the
                area to write is blank. 
                Step 1: check sector lock and unlock
                Step 2: set and verify write enable
                Step 4: write to serial flash

      \param pData      character pointer to the data
      \param address    begining address to start write
      \param size       amount of data to write
      
      \return result of write operation  

      \author
          Aaron Swift
*******************************************************************************/ 
bool writeSerialFlashNE( uint8_t *pData, uint32_t address, int size, FLASH_SECTORS sector )
{
    bool status = false;
    
    /*is the sector locked ?*/
    isSectorLocked(&serialFlash, sector);    
    if( serialFlash.sectorLocked )
    {
        /*unlock the write protection */
        clearSectorLock(&serialFlash, sector);        
    }

    status = true;
    while( status && (size > 0)) 
    {
        /*verify the write enable is set */
        setupTransfer(FLASH_READ_STATUS_REG, &serialFlash, NULL, NULL, 0);
        processCommand(&serialFlash);    
        if( (serialFlash.statusRegister & WRITE_ENABLE_MASK) != WRITE_ENABLE_MASK )
        {
            setupTransfer( FLASH_WRITE_EN, &serialFlash, NULL, NULL, 0 );
            processCommand( &serialFlash );              
        } 
        
        /* program the serial flash */
        int sz;
        int bz = WRITE_BLOCK_SIZE;

        if( size > bz) 
        {
            sz = bz;
        } else {
            sz = size;
        }
        setupTransfer( FLASH_PAGE_PROGRAM, &serialFlash,  (uint8_t *)&address, pData, sz );
        if( serialFlash.error == 0 ) 
        {
            processCommand( &serialFlash );
            if( serialFlash.error == 0 ) 
            {
                address += sz;
                pData += sz;
                size -= sz;
            } else {
                status = false;
            }          
        }
    }

    return status;           
}


/*! ****************************************************************************   
      \fn bool erasePage( uint32_t address, FLASH_SECTORS sector )

      \brief
        public: This function erases one page (4KB) within the serial flash block. 
                Note: This function will only work within cat3 audit blocks 0 & 1.
                area to write is blank. 
                Step 1: check sector lock and unlock
                Step 2: set and verify write enable
                Step 4: erase serial flash page

      
      \param address  begining address of page to erase
      
      \return result of erase operation  

      \author
          Aaron Swift
*******************************************************************************/ 
bool erasePage( uint32_t address, FLASH_SECTORS sector )
{
    bool status = false;
    /*do not allow users to erase block or (sector) 3 information */ 
    if( ( address >= SECTOR0_BASE_ADDR  ) ) /* && ( address < SECTOR3_BASE_ADDR ) ) */
    {  
        /*is the sector locked ?*/
        isSectorLocked( &serialFlash, sector );    
        if( serialFlash.sectorLocked )
        {
            /*unlock the write protection */
            clearSectorLock( &serialFlash, sector );        
        }
    
        /*verify the write enable is set */
        setupTransfer( FLASH_READ_STATUS_REG, &serialFlash, NULL, NULL, 0 );
        processCommand( &serialFlash );    
        if( (serialFlash.statusRegister & WRITE_ENABLE_MASK) != WRITE_ENABLE_MASK )
        {
            setupTransfer( FLASH_WRITE_EN, &serialFlash, NULL, NULL, 0 );
            processCommand( &serialFlash );              
        } 
            
        /* program the serial flash */
        setupTransfer( FLASH_PAGE_ERASE, &serialFlash,  (uint8_t *)&address, NULL, 0 );
        if( serialFlash.error == 0 )
        {
            processCommand( &serialFlash );
            if( serialFlash.error == 0 )
            {
                status = true;
            }          
        }
    }
    return status;           
  
}

/*! ****************************************************************************   
      \fn bool eraseSector( uint32_t address, FLASH_SECTORS sector )

      \brief
        public: This function erases one sector (32KB) within the serial flash block. 
                Note: This function will only work within cat3 audit blocks 0 & 1.
                area to write is blank. 
                Step 1: check sector lock and unlock
                Step 2: set and verify write enable
                Step 4: erase serial flash sector

      
      \param address  begining address of sector to erase
      
      \return result of erase operation  

      \author
          Aaron Swift
*******************************************************************************/ 
bool eraseSector( uint32_t address, FLASH_SECTORS sector )
{
    bool status = false;
    /*do not allow users to erase block or (sector) 3 information */ 
    if( ( address == SECTOR0_BASE_ADDR ) || ( address == SECTOR1_BASE_ADDR ) )
    {  
        /*is the sector locked ?*/
        isSectorLocked( &serialFlash, sector );    
        if( serialFlash.sectorLocked )
        {
            /*unlock the write protection */
            clearSectorLock( &serialFlash, sector );        
        }
    
        /*verify the write enable is set */
        setupTransfer( FLASH_READ_STATUS_REG, &serialFlash, NULL, NULL, 0 );
        processCommand( &serialFlash );    
        if( (serialFlash.statusRegister & WRITE_ENABLE_MASK) != WRITE_ENABLE_MASK )
        {
            setupTransfer( FLASH_WRITE_EN, &serialFlash, NULL, NULL, 0 );
            processCommand( &serialFlash );              
        } 
            
        /* program the serial flash */
        setupTransfer( FLASH_SECTOR_ERASE, &serialFlash,  (uint8_t *)&address, NULL, 0 );
        if( serialFlash.error == 0 )
        {
            processCommand( &serialFlash );
            if( serialFlash.error == 0 )
            {
                status = true;
            }          
        }
    }
    return status;           
  
}

/*! **************************************************************************** 
      \fn void setDataReady( void )
 
      \brief
        public: This function sets the data ready flag                     
        
      \return error code  

      \author
          Aaron Swift
*******************************************************************************/ 
void setDataReady( void )
{
    serialFlash.dataReady = true;  
}

/*! **************************************************************************** 
      \fn void clearDataReady( void )
 
      \brief
        public: This function clears the data ready flag                     
        
      \return error code  

      \author
          Aaron Swift
*******************************************************************************/ 
void clearDataReady()
{
    serialFlash.dataReady = false;  
}



/*! **************************************************************************** 
      \fn void getDataReadyStatus( void )
 
      \brief
        public: This function returns the status of the data ready flag.                     
        
      \return error code  

      \author
          Aaron Swift
*******************************************************************************/ 
bool getDataReadyStatus( void )
{
    return serialFlash.dataReady;
}

/*! **************************************************************************** 
      \fn int getLastFlashError()
 
      \brief
        public: This function returns the last error                        
        
      \return error code  

      \author
          Aaron Swift
*******************************************************************************/ 
int getLastFlashError()
{
    return serialFlash.error;
}



/*! **************************************************************************** 
      \fn int isWriteEnabled(SERIAL_FLASH_CNTRL *pSerialFlash)
 
      \brief
        public: This function checks the write enable bit within the 
                serial flash status register. 
                

      \param pSerialFlash pointer to the serial flash control instance          
        
      \return error code        
      \author
          Aaron Swift
*******************************************************************************/ 
int isWriteEnabled( SERIAL_FLASH_CNTRL *pSerialFlash )
{
     /*read the serial flash status register */
    setupTransfer( FLASH_READ_STATUS_REG, pSerialFlash, NULL, NULL, 0 );
    if( pSerialFlash->error == 0 )
    {
        processCommand( pSerialFlash );        
    }
    return pSerialFlash->error;

}


/*! ****************************************************************************
      \fn int setWriteEnable(SERIAL_FLASH_CNTRL *pSerialFlash)                                                                
 
      \brief
        public: This function set the serial flash write enable 

      \param pSerialFlash pointer to the serial flash control instance          
        
      \return error code        
      \author
          Aaron Swift
*******************************************************************************/ 
int setWriteEnable( SERIAL_FLASH_CNTRL *pSerialFlash )
{
    /*set the write enable bit in the serial flash status register */ 
    setupTransfer( FLASH_WRITE_EN, pSerialFlash, NULL, NULL, 0 );
    if( pSerialFlash->error == 0 )
    {
        processCommand( pSerialFlash );
    }
    
    /*check that the write enable bit is set */
    isWriteEnabled(pSerialFlash);
    if ( !pSerialFlash->writeEnabled )
    {
        pSerialFlash->error = _WRITEENABLE;
    }
    
    return pSerialFlash->error;  
}


/*! **************************************************************************** 
      \fn int isSectorLocked(SERIAL_FLASH_CNTRL *pSerialFlash, FLASH_SECTORS sectorNum)                                                                
 
      \brief
        public: Which sectors are locked is determined by which block protection 
                bits are set within the serial flash status register. This function 
                reads the status register and determines if the sector number 
                is write protected. 

      \param pSerialFlash pointer to the serial flash control instance          
      \param sectorNum serial flash sector to check  
        
      \return error code        
      \author
          Aaron Swift
*******************************************************************************/  
int isSectorLocked( SERIAL_FLASH_CNTRL *pSerialFlash, FLASH_SECTORS sectorNum )
{
     /*read the serial flash status register */
    setupTransfer( FLASH_READ_STATUS_REG, pSerialFlash, NULL, NULL, 0 );
    if( pSerialFlash->error == 0 )
    {
        processCommand(pSerialFlash);
        if( pSerialFlash->error == 0 )
        {
            pSerialFlash->statusRegister &= ( BLOCK_PROT0_MASK | BLOCK_PROT1_MASK );
            pSerialFlash->statusRegister >>= PROTECT_BITS_OFFSET;
            
            if( pSerialFlash->statusRegister == NO_SECTOR_PROTECT )
            {
                pSerialFlash->sectorLocked = false;
            } 
            else if( pSerialFlash->statusRegister == ALL_SECTORS )
            {
                pSerialFlash->sectorLocked = true;
            }
            else
            {   
                /*only upper half or all of the flash can be protected */
                if( pSerialFlash->statusRegister == SECTOR_PROTECT_3 )
                {
                    if( sectorNum == SECTOR_3 )
                    {
                        pSerialFlash->sectorLocked = true;                                       
                    }
                    else
                    {
                        pSerialFlash->sectorLocked = false;
                    }
                }
                else if( pSerialFlash->statusRegister == SECTOR_PROTECT_2_3 )
                {
                    if( ( sectorNum == SECTOR_3 ) || ( sectorNum == SECTOR_2 ))
                    {
                        pSerialFlash->sectorLocked = true;
                      
                    }
                    else
                    {
                        pSerialFlash->sectorLocked = false;
                    }                                                                                                
                }
            }                          
        }
    }
    return pSerialFlash->error;
}

/*! ****************************************************************************   
      \fn int setSectorLock(SERIAL_FLASH_CNTRL *pSerialFlash, FLASH_SECTORS sectorNum)                                                                
 
      \brief
        public: Which sectors are locked is determined by which block protection 
                bits are set within the serial flash status register. This function 
                writes the protection scheme to the status register. 
          
      \param sectorNum protection scheme  
        
      \return error code        
      \author
          Aaron Swift
*******************************************************************************/  
int setSectorLock( SECTOR_PROTECT sector )
{  
    
    /*enable writing to the status register */
    setupTransfer( FLASH_ENABLE_STATUS_REG, &serialFlash, NULL, NULL, 0 );
    processCommand( &serialFlash );         
    if( serialFlash.error == 0 )
    {
        if( sector != NO_SECTOR_PROTECT )
        {
            sector <<= PROTECT_BITS_OFFSET;
            setupTransfer( FLASH_WRITE_STATUS_REG, &serialFlash, NULL, (uint8_t *)&sector, 1 );
            processCommand( &serialFlash );         
        }
    }    
    return serialFlash.error;
}


/*! ****************************************************************************   
      \fn int clearSectorLock( SERIAL_FLASH_CNTRL *pSerialFlash, FLASH_SECTORS sector )                                                                
 
      \brief
        public: 
 

      \param pSerialFlash pointer to the serial flash control instance          
      \param sectorNum protection scheme  
        
      \return error code        
      \author
          Aaron Swift
*******************************************************************************/
int clearSectorLock( SERIAL_FLASH_CNTRL *pSerialFlash, FLASH_SECTORS sector )
{
    SECTOR_PROTECT lock = ALL_SECTORS;
    /*upper half of the flash sectors can be locked */
    if( sector == SECTOR_3 )
    {
        /*remove write protection on sectors 0 - 3 */ 
        lock = NO_SECTOR_PROTECT;        
    }
    else if( sector == SECTOR_2 )
    {
        /*remove write protection on sectors 0 - 2 */ 
        lock = SECTOR_PROTECT_3;
    }
    else 
    {
         /*remove write protection on sectors 0 - 1 */ 
         lock = SECTOR_PROTECT_2_3;
    }
    lock <<= PROTECT_BITS_OFFSET;
    
    /*enable writing to the status register */
    setupTransfer(FLASH_ENABLE_STATUS_REG, pSerialFlash, NULL, NULL, 0);
    processCommand(pSerialFlash);         
    if(pSerialFlash->error == 0)
    {
        /*write the new sector lock to the status register*/
        setupTransfer(FLASH_WRITE_STATUS_REG, pSerialFlash, NULL, (uint8_t *)&lock, 1);    
        processCommand(pSerialFlash);         
    }        
    return pSerialFlash->error;   
}

/*! ****************************************************************************
      \fn uint32_t getSectorBaseAddress()
                                                                                  
 
      \brief
        public: This function return the base address of a given sector of the serial flash.

      \param sector

      \author
          Aaron Swift
*******************************************************************************/  
uint32_t getSectorBaseAddress(FLASH_SECTORS sector)
{
    uint32_t addr = 0;
    
    if( sector == SECTOR_0 )
    {
        addr = SECTOR0_BASE_ADDR;      
    }
    else if(sector == SECTOR_1 )
    {
        addr = SECTOR1_BASE_ADDR;            
    }
    else if(sector == SECTOR_2 )
    {
        addr = SECTOR2_BASE_ADDR;            
    }
    else 
    {
        addr = SECTOR3_BASE_ADDR;            
    }
    return addr;
}

/*! ****************************************************************************
      \fn setupTransfer(FLASH_CMD command, SERIAL_FLASH_CNTRL *pSerialFlash,
                        uint8_t *pAddr, uint8_t *pData, int length)                                                                
 
      \brief
        public: This function sets up the transfer buffer with the proper sequence 
        of command, address and data to perform an action on the flash. 
        Reads and writes are restricted in size to one page of data (4k bytes).
        This limitation is due to the amount of data that can be erased which is 
        one page (4k bytes).

      \param command
      \param pSerialFlash pointer to the serial flash control instance          
      \param pAddr flash data address
      \param pData data to read from flash or write to flash
      \param length amount of data to read or write

      \author
          Aaron Swift
*******************************************************************************/  
void setupTransfer( FLASH_CMD command, 
                    SERIAL_FLASH_CNTRL *pSerialFlash, 
                    uint8_t *pAddr, 
                    uint8_t *pData, 
                    int length )
{
    pSerialFlash->currentCommand = command;
    pSerialFlash->pData = pData;
    pSerialFlash->length = length;
    
    /* clear our transaction buffers */
    //memset( pSerialFlash->transferBuffer, 0, BUFFER_SIZE);
    //memset( pSerialFlash->sectorBuffer, 0, BUFFER_SIZE);
    
    /*setup the buffer with the command sequence */    
    int i = 0;
    switch(command)
    {
        case FLASH_READ_IDENT:
        {            
            pSerialFlash->transferBuffer[i++] = READ_IDENT;
            int j;
            
            pSerialFlash->transferBuffer[i++] = DUMMY_BYTE;     /* Read M7-M0,    Manufacturer */
            pSerialFlash->transferBuffer[i++] = DUMMY_BYTE;     /* Read ID15-ID8, Memory type */
            pSerialFlash->transferBuffer[i++] = DUMMY_BYTE;     /* Read ID7-ID0,  Capacity */
            pSerialFlash->transferSize = i;
            break;
        }
        case FLASH_READ_STATUS_REG:
        {
            pSerialFlash->transferBuffer[i++] = READ_STATUS_REG;
            pSerialFlash->transferBuffer[i++] = DUMMY_BYTE;     /* Read S7-S0 */
            pSerialFlash->transferSize = i;         
            break;
        }
        case FLASH_READ_DATA_BYTES:
        case FLASH_READ_DATA_BYTES_HS:          
        {
            /*transfer buffer is only the size of one page so if we are reading 
              an entire sector then skip command byte and retransmit until complete.
            */
            if( length <= PAGE_SIZE ) 
            {
                int j;
                if(command == FLASH_READ_DATA_BYTES)
                    pSerialFlash->transferBuffer[i++] = READ_DATA_BYTES;
                else
                    pSerialFlash->transferBuffer[i++] = READ_DATA_BYTES_HS;
                
                /*add the address to the command sequence */
                for(j = 0; j < ADDR_SIZE; j++)
                {                               
                    pSerialFlash->transferBuffer[ADDR_SIZE - j] = *pAddr++;
                    i++;
                }

                /* Insert the dummy field for HS Read */
                if(command == FLASH_READ_DATA_BYTES_HS)
                    pSerialFlash->transferBuffer[i++] = DUMMY_BYTE;

                for(j = 0; j < length; j++)
                {                  
                    pSerialFlash->transferBuffer[i++] = DUMMY_BYTE;
                }
                pSerialFlash->transferSize = i;
            }
            else
            {
                 pSerialFlash->error =  _DATAERRORSIZE;
                 pSerialFlash->currentCommand = FLASH_NO_COMMAND;
            }
            break;
        }          
        case FLASH_WRITE_EN:
        {
            pSerialFlash->transferBuffer[i++] = WRITE_EN;            
            pSerialFlash->transferSize = i;
            break;
        }
        case FLASH_WRITE_DS:
        {
            pSerialFlash->transferBuffer[i++] = WRITE_DS;
            pSerialFlash->transferSize = i;
            break;
        }
        case FLASH_ENABLE_STATUS_REG:
        {
            pSerialFlash->transferBuffer[i++] = ENABLE_WRITE_STATUS_REG; 
            pSerialFlash->transferSize = i; 
            break;
        }
        case FLASH_WRITE_STATUS_REG:
        {           
            pSerialFlash->transferBuffer[i++] = WRITE_STATUS_REG;
            if(pData != NULL)
            {  
                pSerialFlash->transferBuffer[i++] = *pData;
                pSerialFlash->transferSize = i;
            }
            else
            {
                 pSerialFlash->error =  _DATAERRORSIZE;
                 pSerialFlash->currentCommand = FLASH_NO_COMMAND;              
            }
            break;
        }
        case FLASH_PAGE_PROGRAM:
        {
            if(   (length <= (BUFFER_SIZE - CMD_SIZE - ADDR_SIZE))
               && (length <= WRITE_BLOCK_SIZE)
              )
            {
                int j;
                pSerialFlash->transferBuffer[i++] = PAGE_PROGRAM;
                                
                /*add the starting address to the command sequence */
                for(j = 0; j < ADDR_SIZE; j++)
                {                               
                    pSerialFlash->transferBuffer[ADDR_SIZE - j] = *pAddr++;
                    i++;
                }

                /*every data byte is followed by the command */
                for(j = 0; j < length; j++)
                {                  
                    pSerialFlash->transferBuffer[i++] = *pData++;
                }
                
                pSerialFlash->transferSize = i;
            }
            else
            {
                 pSerialFlash->error =  _DATAERRORSIZE;
                 pSerialFlash->currentCommand = FLASH_NO_COMMAND;
            }
            break;
        }
        case FLASH_PAGE_ERASE:
        case FLASH_SECTOR_ERASE:
        {
            if( pAddr != NULL )
            {
                if( command == FLASH_SECTOR_ERASE )
                {
                    pSerialFlash->transferBuffer[i++] = SECTOR_ERASE;
                }
                else
                {
                    pSerialFlash->transferBuffer[i++] = PAGE_ERASE;
                }
                
                int j;
                
                /*add the address to the command sequence */
                for( j = 0; j < ADDR_SIZE; j++ )
                {                               
                    pSerialFlash->transferBuffer[ADDR_SIZE - j] = *pAddr++;
                    i++;
                }                
                pSerialFlash->transferSize = i;
            }
            else
            {
                 pSerialFlash->error =  _DATAERRORSIZE;
                 pSerialFlash->currentCommand = FLASH_NO_COMMAND;              
            }
            break;
        }
        case FLASH_BULK_ERASE:
        {
            pSerialFlash->transferBuffer[i++] = BULK_ERASE;
            pSerialFlash->transferSize = i;
            break;
        }      
        case FLASH_RELEASE_PWR_DWN:
        {
            pSerialFlash->transferBuffer[i++] = RELEASE_POWR_DWN;
            
#if 1
            pSerialFlash->transferBuffer[i++] = DUMMY_BYTE;
            pSerialFlash->transferSize = 1;     
            //pSerialFlash->transferBuffer[i++] = 0x90;
            //pSerialFlash->transferBuffer[i++] = 0x92;
            //pSerialFlash->transferBuffer[i++] = 0x9f;
            //pSerialFlash->transferSize = 4;     
#else            
            /* 4 dummy bytes + 8 data bytes*/
            while (i<13) {
                pSerialFlash->transferBuffer[i++] = DUMMY_BYTE;
            }
            pSerialFlash->transferSize = i;         
#endif            
            break;
        }
        case FLASH_READ_UNIQUE_ID:
        {
            pSerialFlash->transferBuffer[i++] = READ_UNIQUE_ID;
            /* 4 dummy bytes + 8 data bytes*/
            while (i<13) {
                pSerialFlash->transferBuffer[i++] = DUMMY_BYTE;
            }
            pSerialFlash->transferSize = i;         
            break;
        }
        default:
        {
            pSerialFlash->transferBuffer[i++] = FLASH_NO_COMMAND;
            pSerialFlash->transferSize = 0;
        } 
    } 
}


/*! ****************************************************************************  
      \fn processCommand(SERIAL_FLASH_CNTRL *pSerialFlash)                                                                
 
      \brief
        public: This function processes the current command 
      
      \param pSerialFlash pointer to the serial flash control instance          

      \author
          Aaron Swift
*******************************************************************************/ 
void processCommand(SERIAL_FLASH_CNTRL *pSerialFlash)
{
    switch(pSerialFlash->currentCommand)
    {
        case FLASH_READ_IDENT:
        {
            handleReadId( pSerialFlash ); 
            break;
        }
        case FLASH_READ_STATUS_REG:
        {
            handleReadStatus( pSerialFlash );
            break;
        }
        case FLASH_READ_DATA_BYTES:
        {
            handleReadData( pSerialFlash );
            break;
        }
        case FLASH_READ_DATA_BYTES_HS:
        {
            handleReadDataHS( pSerialFlash );            
            break;
        }
        case FLASH_READ_ES:
        {
            handleReadSignature( pSerialFlash );
            break;
        }
        case FLASH_WRITE_EN:
        {
            handleWriteEnable( pSerialFlash );
            break;
        }
        case FLASH_WRITE_DS:
        {
            handleWriteDisable( pSerialFlash );
            break;
        }
        case FLASH_ENABLE_STATUS_REG:
        {
            handleEnableStatusReg( pSerialFlash );
            break;                
        }
        case FLASH_WRITE_STATUS_REG:
        {
            handleWriteStatus( pSerialFlash );
            break;
        }
        case FLASH_PAGE_PROGRAM:
        {
            handlePageProgram( pSerialFlash );
            break;
        }
        case FLASH_PAGE_ERASE:
        case FLASH_SECTOR_ERASE:
        {
            handleSectorErase( pSerialFlash );
            break;
        }
        case FLASH_BULK_ERASE:
        {
            handleBulkErase( pSerialFlash );
            break;
        }      
        case FLASH_RELEASE_PWR_DWN:
        {
            handleReleasePowerDwn( pSerialFlash ); 
            break;
        }
        case FLASH_READ_UNIQUE_ID:
        {
            handleReadUniqueId( pSerialFlash ); 
            break;
        }
        default:
        {
          pSerialFlash->error = _PROCERROR;
        } 
    }   
}


/*! ****************************************************************************
      \fn intializeFlashInterface()                                                                
 
      \brief
        private: This function initializes the physical interface to the serial 
        flash device. 

       \author
          Aaron Swift
*******************************************************************************/  
static void intializeFlashInterface()
{
    lpspi_master_config_t LPSPI1_config = {
    .baudRate =                         500000U,
    .bitsPerFrame =                     8UL,
    .cpol =                             kLPSPI_ClockPolarityActiveHigh,
    .cpha =                             kLPSPI_ClockPhaseFirstEdge,
    .direction =                        kLPSPI_MsbFirst,
    .pcsToSckDelayInNanoSec =           250UL,
    .lastSckToPcsDelayInNanoSec =       250UL,
    .betweenTransferDelayInNanoSec =    250UL,
    .whichPcs =                         kLPSPI_Pcs0,
    .pcsActiveHighOrLow =               kLPSPI_PcsActiveLow,
    .pinCfg =                           kLPSPI_SdiInSdoOut,
    .dataOutConfig =                    kLpspiDataOutTristate
    };

    LPSPI_MasterGetDefaultConfig( &LPSPI1_config );
    LPSPI1_config.baudRate = 500000U;
    LPSPI1_config.whichPcs = kLPSPI_Pcs0;
    /*set clock source for LPSPI*/
    CLOCK_SetMux( kCLOCK_LpspiMux, 1 );
    CLOCK_SetDiv( kCLOCK_LpspiDiv, 7 );
    LPSPI_MasterInit( LPSPI1, &LPSPI1_config, ( CLOCK_GetFreq(kCLOCK_Usb1PllPfd0Clk) / 8 ) );
    LPSPI_EnableInterrupts( LPSPI1, (kLPSPI_TxInterruptEnable | 
                                   kLPSPI_RxInterruptEnable | 
                                   kLPSPI_TransmitErrorInterruptEnable | 
                                   kLPSPI_ReceiveErrorInterruptEnable ) );
    LPSPI_MasterTransferCreateHandle( LPSPI1, &spi1Handle_, lpspi1Callback, NULL );
    EnableIRQ( LPSPI1_IRQn );
    
    LPSPI_Enable( LPSPI1, false );
    LPSPI1->CFGR1 &= (~LPSPI_CFGR1_NOSTALL_MASK);
    LPSPI_Enable( LPSPI1, true );    
}



/*! **************************************************************************** 
      \fn handleReadId(SERIAL_FLASH_CNTRL *pSerialFlash)                                                                
 
      \brief
        private: This function read the id from the serial flash 
      
      \param pSerialFlash pointer to the serial flash control instance          

      \author
          Aaron Swift
*******************************************************************************/ 
static void handleReadId(SERIAL_FLASH_CNTRL *pSerialFlash)
{
    /*read the manufacture id from the serial flash*/
    writeFlashByte( pSerialFlash );    
    if( pSerialFlash->error == 0 )
    {
        pSerialFlash->chipId = (unsigned int)pSerialFlash->sectorBuffer[2];
        pSerialFlash->chipId <<= 8;
        pSerialFlash->chipId |= (unsigned int)pSerialFlash->sectorBuffer[3];

#if 0          
        /*read the device id from the serial flash */
        uint32_t addr = 1;
        setupTransfer( FLASH_READ_IDENT, pSerialFlash, (uint8_t *)&addr, NULL, sizeof(addr) );
        writeFlashByte( pSerialFlash );
        if( pSerialFlash->error == 0 )
        {
            pSerialFlash->chipId |= (unsigned int)pSerialFlash->sectorBuffer[CHIP_ID_SIZE + 1];
        }
        else
        {
            pSerialFlash->chipId = 0;
        }
#endif        
    }       
}


/*! ****************************************************************************
      \fn handleReadStatus(SERIAL_FLASH_CNTRL *pSerialFlash)                                                                
 
      \brief
        private: This function read the contents of the serial flash status register. 
         
      \param pSerialFlash pointer to the serial flash control instance          

      \author
          Aaron Swift
*******************************************************************************/  
static void handleReadStatus(SERIAL_FLASH_CNTRL *pSerialFlash)
{
    writeFlashByte(pSerialFlash);
    if(pSerialFlash->error == 0)
    {
       pSerialFlash->statusRegister = (unsigned int)pSerialFlash->sectorBuffer[1];
       pSerialFlash->flashBusy = (bool)(pSerialFlash->statusRegister & FLASH_STATUS_MASK);
       pSerialFlash->writeEnabled =  (bool)(pSerialFlash->statusRegister & WRITE_ENABLE_MASK);                
    }  
}


/*! ****************************************************************************
      \fn handleReadData(SERIAL_FLASH_CNTRL *pSerialFlash)                                                                
 
      \brief
        private: This function read the contents of the serial flash. 
         
      \param pSerialFlash pointer to the serial flash control instance          

      \author
          Aaron Swift
*******************************************************************************/ 
static void handleReadData(SERIAL_FLASH_CNTRL *pSerialFlash)
{
    writeFlashByte( pSerialFlash );   
    if(pSerialFlash->error == 0)
    {
        /* If a destination pointer is transmitted to the command, copy read data to this address */
        if( pSerialFlash->pData) {
            uint8_t *data = getReadData(0);
            if (data) {
                memcpy(pSerialFlash->pData, data, pSerialFlash->length);
            }
        }
    }  
}


/*! ****************************************************************************
      \fn handleReadDataHS(SERIAL_FLASH_CNTRL *pSerialFlash)                                                                
 
      \brief
        private: This function read the contents of the serial flash at high speed. 
         
      \param pSerialFlash pointer to the serial flash control instance          

      \author
          Aaron Swift
*******************************************************************************/  
static void handleReadDataHS(SERIAL_FLASH_CNTRL *pSerialFlash)
{
    writeFlashByte( pSerialFlash );   
    if(pSerialFlash->error == 0)
    {
        /* If a destination pointer is transmitted to the command, copy read data to this address */
        if( pSerialFlash->pData) {
            uint8_t *data = getReadData(0);
            memcpy(pSerialFlash->pData, data, pSerialFlash->length);
        }
    }  
}


/*! **************************************************************************** 
      \fn handleReadSignature(SERIAL_FLASH_CNTRL *pSerialFlash)                                                                
 
      \brief
        private: This function reads the electronic signature of the serial flash. 
         
      \param pSerialFlash pointer to the serial flash control instance          

      \author
          Aaron Swift
*******************************************************************************/  
static void handleReadSignature(SERIAL_FLASH_CNTRL *pSerialFlash)
{
    /* todo */
  
}


/*! ****************************************************************************
      \fn handleWriteEnable(SERIAL_FLASH_CNTRL *pSerialFlash)                                                                
 
      \brief
        private: This function sets the write enable latch of the serial flash
                 and waits until serial flash has completed command. 
         
      \param pSerialFlash pointer to the serial flash control instance          
        
      \author
          Aaron Swift
*******************************************************************************/ 
static void handleWriteEnable( SERIAL_FLASH_CNTRL *pSerialFlash )
{
    /*write to the serial flash */
    writeFlashByte( pSerialFlash );
    waitForCompletion( pSerialFlash );
}


/*! ****************************************************************************
      \fn handleWriteDisable(SERIAL_FLASH_CNTRL *pSerialFlash)                                                                
 
      \brief
        private: This function clears the write enable latch of the serial flash. 
         
      \param pSerialFlash pointer to the serial flash control instance          

      \author
          Aaron Swift
*******************************************************************************/ 
static void handleWriteDisable(SERIAL_FLASH_CNTRL *pSerialFlash)
{
    writeFlashByte(pSerialFlash);
    waitForCompletion( pSerialFlash );
}


/*! ****************************************************************************
      \fn handleWriteStatus(SERIAL_FLASH_CNTRL *pSerialFlash)                                                                
 
      \brief
        private: This function writes to the status register of the serial flash
                 and waits for serial flash to complete command. 
         
      \param pSerialFlash pointer to the serial flash control instance          

      \author
          Aaron Swift
*******************************************************************************/ 
static void handleWriteStatus( SERIAL_FLASH_CNTRL *pSerialFlash )
{
    writeFlashByte( pSerialFlash );
    waitForCompletion( pSerialFlash );
    
}

/*! ****************************************************************************
      \fn handleEnableStatusReg( SERIAL_FLASH_CNTRL *pSerialFlash )                                                                
 
      \brief
        private: This function writes the status register write enable command to 
                 the serial flash.

      \param pSerialFlash pointer to the serial flash control instance          

      \author
          Aaron Swift
*******************************************************************************/ 
static void handleEnableStatusReg( SERIAL_FLASH_CNTRL *pSerialFlash )
{
    writeFlashByte( pSerialFlash ); 
}

/*! **************************************************************************** 
      \fn handlePageProgram(SERIAL_FLASH_CNTRL *pSerialFlash)                                                                
 
      \brief
        private: This function writes one page (256 bytes) of the serial flash
                 and waits for serial flash to complete command.
         
      \param pSerialFlash pointer to the serial flash control instance          

      \author
          Aaron Swift
*******************************************************************************/ 
static void handlePageProgram(SERIAL_FLASH_CNTRL *pSerialFlash)
{
    writeFlashByte( pSerialFlash );
	waitForCompletion( pSerialFlash );
}


/*! ****************************************************************************
      \fn handleSectorErase(SERIAL_FLASH_CNTRL *pSerialFlash)                                                                
 
      \brief
        private: This function erases one sector () of the serial flash
                 and waits for serial flash to complete command.
         
      \param pSerialFlash pointer to the serial flash control instance          

      \author
          Aaron Swift
*******************************************************************************/ 
static void handleSectorErase( SERIAL_FLASH_CNTRL *pSerialFlash )
{
    writeFlashByte( pSerialFlash ); 
    waitForCompletion( pSerialFlash );
}


/*! ****************************************************************************
      \fn handleBulkErase(SERIAL_FLASH_CNTRL *pSerialFlash)                                                                
 
      \brief
        private: This function erases entire contents of the serial flash. 
         
      \param pSerialFlash pointer to the serial flash control instance          

      \author
          Aaron Swift
*******************************************************************************/ 
static void handleBulkErase(SERIAL_FLASH_CNTRL *pSerialFlash)
{
    writeFlashByte(pSerialFlash);   
    waitForCompletion( pSerialFlash );    
}


/*! ****************************************************************************
      \fn handleReleasePowerDwn(SERIAL_FLASH_CNTRL *pSerialFlash)                                                                
 
      \brief
        private: This function write the instruction to bring the serial flash
                 out of the power down state. 
         
      \param pSerialFlash pointer to the serial flash control instance          

      \author
          Philippe Corbes
*******************************************************************************/ 
static void handleReleasePowerDwn(SERIAL_FLASH_CNTRL *pSerialFlash)
{
    writeFlashByte( pSerialFlash );   
    if(pSerialFlash->error == 0)
    {
        /* If a destination pointer is transmitted to the command, copy read data to this address */
        if( pSerialFlash->pData) {
            memcpy(pSerialFlash->pData, &serialFlash.sectorBuffer[5], pSerialFlash->length);
        }
    }  
}


/*! ****************************************************************************
      \fn handleReadUniqueId(SERIAL_FLASH_CNTRL *pSerialFlash)                                                                
 
      \brief
        private: This function read the contents of the serial flash. 
         
      \param pSerialFlash pointer to the serial flash control instance          

      \author
          Philippe Corbes
*******************************************************************************/ 
static void handleReadUniqueId(SERIAL_FLASH_CNTRL *pSerialFlash)
{
    writeFlashByte( pSerialFlash );   
    if(pSerialFlash->error == 0)
    {
        /* If a destination pointer is transmitted to the command, copy read data to this address */
        if( pSerialFlash->pData) {
            memcpy(pSerialFlash->pData, &serialFlash.sectorBuffer[5], pSerialFlash->length);
        }
    }  
}


/*! **************************************************************************** 
      \fn waitForCompletion(SERIAL_FLASH_CNTRL *pSerialFlash)                                                                
 
      \brief
        private: This function spins until the serial flash busy bit clears. 
         
      \param pSerialFlash pointer to the serial flash control instance          

      \author
          Aaron Swift
*******************************************************************************/ 
static void waitForCompletion( SERIAL_FLASH_CNTRL *pSerialFlash )
{
#ifdef MANAGE_STATISTICS
    pSerialFlash->busyCount = 0;
#endif
    do {
#ifdef MANAGE_STATISTICS
        pSerialFlash->busyCount++;
#endif
        setupTransfer(FLASH_READ_STATUS_REG, pSerialFlash, NULL, NULL, 0);
        processCommand(pSerialFlash);
        if(pSerialFlash->error != 0)
        {
            pSerialFlash->error = _WAITERROR;
            break;
        } 
    } while( pSerialFlash->flashBusy );
}

/*! **************************************************************************** 
      \fn int isFlashBusy(SERIAL_FLASH_CNTRL *pSerialFlash)                                                                
 
      \brief
        private: This function reads the serial flash status register and 
                 sets or clear the flashBusy flag within the serial flash 
                 control structure. 
         
      \param pSerialFlash pointer to the serial flash control instance          
      \return pSerialFlash->error;

      \author
          Aaron Swift
*******************************************************************************/ 
static int isFlashBusy(SERIAL_FLASH_CNTRL *pSerialFlash)
{    
    /*read the serial flash status register */
    setupTransfer(FLASH_READ_STATUS_REG, pSerialFlash, NULL, NULL, 0);
    if(pSerialFlash->error == 0)
    {
        processCommand(pSerialFlash);
//        if(pSerialFlash->error == 0)
//            pSerialFlash->flashBusy = (bool)pSerialFlash->statusRegister & FLASH_STATUS_MASK;
    }
    return pSerialFlash->error;
}


/*! ****************************************************************************   
      \fn writeFlashByte(SERIAL_FLASH_CNTRL *pSerialFlash)                                                                
 
      \brief
        private: this function will write on byte to the serial flash
        
      \param pSerialFlash pointer to the serial flash control instance          

      \author
          Aaron Swift
*******************************************************************************/ 
static void writeFlashByte()
{
#ifdef MIMXRT1024EVK
#else
    GPIO_WritePinOutput( SERIAL_FlASH_CS_GPIO, SERIAL_FlASH_CS_PIN, false);
#endif    

#ifdef MANAGE_STATISTICS
    serialFlash.messageCount++; /* Increment number of message sent for statistics and debug */
#endif
    clearDataReady();
//    spi1Handle_.state = kLPSPI_Idle;

    /* transfer bytes */   
    lpspi_transfer_t transfer;
    transfer.txData   = serialFlash.transferBuffer;
    transfer.rxData   = serialFlash.sectorBuffer;
    transfer.dataSize = serialFlash.transferSize;
    transfer.configFlags = kLPSPI_Pcs0 | kLPSPI_MasterPcsContinuous; 

    LPSPI_MasterTransferNonBlocking( LPSPI1, &spi1Handle_, &transfer );
    
#ifdef MANAGE_STATISTICS
#define WRITE_FLASH_MAX_LOOP 1000000
    uint32_t loop = 0;
    while( !serialFlash.dataReady && loop < WRITE_FLASH_MAX_LOOP) {loop++;} 
    if (loop >= WRITE_FLASH_MAX_LOOP) {
        PRINTF("writeFlashByte()() timeout!\r\n");
    } else {
        PRINTF("Check Serial flash (loop:%d)!!!\r\n", loop);
    }
#else
    while( !serialFlash.dataReady) { asm("nop"); }
#endif
    
#ifdef MIMXRT1024EVK
#else
    GPIO_WritePinOutput( SERIAL_FlASH_CS_GPIO, SERIAL_FlASH_CS_PIN, true);   
#endif    
}


void lpspi1Callback(LPSPI_Type *base, lpspi_master_handle_t *handle, status_t status, void *userData)
{
    if( status == kStatus_Success ) {
        setDataReady();        
    } else {
        PRINTF("lpspi1Callback(): Transfer failure: %d !\r\n", status );
        serialFlash.error = status;
    }
}

void lpspi4Callback(LPSPI_Type *base, lpspi_master_handle_t *handle, status_t status, void *userData)
{
    if( status == kStatus_Success ) {
        setDataReady();        
    } else {
        PRINTF("lpspi4Callback(): Transfer failure: %d !\r\n", status );
        serialFlash.error = status;
    }
}

#ifdef TEST_UNIT_ENABLED
static void thisUnitTest()
{
    /* Display the Chip ID found during initialization */
    PRINTF("Chip ID: %0x\r\n", serialFlash.chipId);
    
#if 0
    uint32_t address;
    uint8_t localBuffer[512]; /*[1024]*/

    uint64_t id = readUniqueId();
    PRINTF("Unique ID: %016llx\r\n", id);
    
    /*lock the write protection */
    setSectorLock(ALL_SECTORS);        

    /*is the sector locked ?*/
    isSectorLocked(&serialFlash, SECTOR_0);    
    if( serialFlash.sectorLocked )
    {
        /*unlock the write protection */
        clearSectorLock(&serialFlash, SECTOR_0);        
    }

    /*is the sector locked ?*/
    isSectorLocked(&serialFlash, SECTOR_1);    
    if( serialFlash.sectorLocked )
    {
        /*unlock the write protection */
        clearSectorLock(&serialFlash, SECTOR_1);        
    }

    /*is the sector locked ?*/
    isSectorLocked(&serialFlash, SECTOR_2);    
    if( serialFlash.sectorLocked )
    {
        /*unlock the write protection */
        clearSectorLock(&serialFlash, SECTOR_2);        
    }

    /*is the sector locked ?*/
    isSectorLocked(&serialFlash, SECTOR_3);    
    if( serialFlash.sectorLocked )
    {
        /*unlock the write protection */
        clearSectorLock(&serialFlash, SECTOR_3);        
    }

    /*### our erase page public function ###*/
    /*verify the write enable is set */
    setupTransfer(FLASH_READ_STATUS_REG, &serialFlash, NULL, NULL, 0);
    if(serialFlash.error == 0)
    {
        processCommand(&serialFlash);
    }
    if( (serialFlash.statusRegister & WRITE_ENABLE_MASK) != WRITE_ENABLE_MASK )
    {
        setupTransfer( FLASH_WRITE_EN, &serialFlash, NULL, NULL, 0 );
        processCommand( &serialFlash );              
    } 
    setupTransfer(FLASH_READ_STATUS_REG, &serialFlash, NULL, NULL, 0);
    processCommand(&serialFlash);

    
    /* #0 (4K) erase the flash before our write */
    address = SECTOR0_BASE_ADDR; 
    setupTransfer(FLASH_PAGE_ERASE, &serialFlash,  (uint8_t *)&address, NULL, 0 );
    processCommand(&serialFlash);

    /* ensure our erase function is working properly */
    for (int i=0; i<sizeof(localBuffer); i++) localBuffer[i] = (uint8_t)i;
    setupTransfer(FLASH_READ_DATA_BYTES, &serialFlash, (uint8_t *)&address, localBuffer, sizeof(localBuffer));
    processCommand(&serialFlash);

    
    /* #0 (32K) erase the flash before our write */
    address = SECTOR0_BASE_ADDR; 
    setupTransfer(FLASH_SECTOR_ERASE, &serialFlash,  (uint8_t *)&address, NULL, 0 );
    processCommand(&serialFlash);

    /* ensure our erase function is working properly */
    for (int i=0; i<sizeof(localBuffer); i++) localBuffer[i] = (uint8_t)i;
    setupTransfer(FLASH_READ_DATA_BYTES, &serialFlash, (uint8_t *)&address, localBuffer, sizeof(localBuffer));
    processCommand(&serialFlash);

    /* #1 erase the flash before our write */
    address = SECTOR1_BASE_ADDR; 
    setupTransfer(FLASH_PAGE_ERASE, &serialFlash,  (uint8_t *)&address, NULL, 0 );
    processCommand(&serialFlash);

    /* ensure our erase function is working properly */
    for (int i=0; i<sizeof(localBuffer); i++) localBuffer[i] = (uint8_t)i;
    setupTransfer(FLASH_READ_DATA_BYTES, &serialFlash, (uint8_t *)&address, localBuffer, sizeof(localBuffer));
    processCommand(&serialFlash);

    /* erase all the flash */
    setupTransfer(FLASH_BULK_ERASE, &serialFlash,  NULL, NULL, 0 );
    processCommand(&serialFlash);

//    address = SECTOR2_BASE_ADDR; 
//    /*erase the flash before our write */
//    setupTransfer(FLASH_PAGE_ERASE, &serialFlash,  (uint8_t *)&address, NULL, 0 );
//    processCommand(&serialFlash);
//
//    /* ensure our erase function is working properly */
//    for (int i=0; i<sizeof(localBuffer); i++) localBuffer[i]= (uint8_t)i;
//    setupTransfer(FLASH_READ_DATA_BYTES, &serialFlash, (uint8_t *)&address, localBuffer, sizeof(localBuffer));
//    processCommand(&serialFlash);
//
//    uint8_t *pIData = getReadData(0);
//    uint8_t localBuffer[20];
//    int i;
//
//    for(i = 0; i < 20; i++)
//    {
//        localBuffer[i] = *pIData++;
//    }

    /*### our erase page public function ###*/
    /*verify the write enable is set */
    setupTransfer(FLASH_READ_STATUS_REG, &serialFlash, NULL, NULL, 0);
    if(serialFlash.error == 0)
    {
        processCommand(&serialFlash);
    }
    if( (serialFlash.statusRegister & WRITE_ENABLE_MASK) != WRITE_ENABLE_MASK )
    {
        setupTransfer( FLASH_WRITE_EN, &serialFlash, NULL, NULL, 0 );
        processCommand( &serialFlash );              
    } 
    setupTransfer(FLASH_READ_STATUS_REG, &serialFlash, NULL, NULL, 0);
    processCommand(&serialFlash);

    /* write data to the first record */
    address = SECTOR0_BASE_ADDR; 
    for (int i=0; i<sizeof(localBuffer); i++) localBuffer[i] = (uint8_t)i*2;
    setupTransfer(FLASH_PAGE_PROGRAM, &serialFlash, (uint8_t *)&address, localBuffer, sizeof(localBuffer));
    processCommand(&serialFlash);

    /* ensure our write function is working properly */
    for (int i=0; i<sizeof(localBuffer); i++) localBuffer[i]= (uint8_t)i;
    setupTransfer(FLASH_READ_DATA_BYTES, &serialFlash, (uint8_t *)&address, localBuffer, sizeof(localBuffer));
    processCommand(&serialFlash);
    
     /*### our write to flash page public function ###*/       
    setupTransfer(FLASH_READ_STATUS_REG, &serialFlash, NULL, NULL, 0);
    processCommand(&serialFlash);

    address = SECTOR3_BASE_ADDR; 
    
    // This will not erase 
    setupTransfer(FLASH_PAGE_ERASE, &serialFlash,  (uint8_t *)&address, NULL, 0 );
    processCommand(&serialFlash);
    setupTransfer(FLASH_READ_DATA_BYTES, &serialFlash, (uint8_t *)&address, localBuffer, sizeof(localBuffer));
    processCommand(&serialFlash);

    /*verify the write enable is set */
    setupTransfer(FLASH_READ_STATUS_REG, &serialFlash, NULL, NULL, 0);
    if(serialFlash.error == 0)
    {
        processCommand(&serialFlash);
        if( (serialFlash.statusRegister & WRITE_ENABLE_MASK) != WRITE_ENABLE_MASK )
        {
            setupTransfer( FLASH_WRITE_EN, &serialFlash, NULL, NULL, 0 );
            processCommand( &serialFlash );              
        } 
    }
//    setupTransfer(FLASH_READ_STATUS_REG, &serialFlash, NULL, NULL, 0);
//    processCommand(&serialFlash);

    // This new try will erase data
    setupTransfer(FLASH_PAGE_ERASE, &serialFlash,  (uint8_t *)&address, NULL, 0 );
    processCommand(&serialFlash);
    setupTransfer(FLASH_READ_DATA_BYTES, &serialFlash, (uint8_t *)&address, localBuffer, sizeof(localBuffer));
    processCommand(&serialFlash);
   
    setupTransfer(FLASH_READ_STATUS_REG, &serialFlash, NULL, NULL, 0);
    if(serialFlash.error == 0)
    {
        processCommand(&serialFlash);
        if( (serialFlash.statusRegister & WRITE_ENABLE_MASK) != WRITE_ENABLE_MASK )
        {
            setupTransfer( FLASH_WRITE_EN, &serialFlash, NULL, NULL, 0 );
            processCommand( &serialFlash );              
        } 
    }
    setupTransfer(FLASH_READ_STATUS_REG, &serialFlash, NULL, NULL, 0);
    processCommand(&serialFlash);
    
    /*setup a few bytes to write */ 
    for(int i=0; i<sizeof(localBuffer); i++) 
        localBuffer[i] = (uint8_t)i*3;

    setupTransfer(FLASH_PAGE_PROGRAM, &serialFlash, (uint8_t *)&address, localBuffer, sizeof(localBuffer));
    processCommand(&serialFlash);
    if(serialFlash.error == 0)
    {
        // Check read
        memset(localBuffer, 0, sizeof(localBuffer));
        setupTransfer(FLASH_READ_DATA_BYTES, &serialFlash, (uint8_t *)&address, localBuffer, sizeof(localBuffer));
        processCommand(&serialFlash);
        // Check high speedr read
        memset(localBuffer, 0, sizeof(localBuffer));
        setupTransfer(FLASH_READ_DATA_BYTES_HS, &serialFlash, (uint8_t *)&address, localBuffer, sizeof(localBuffer));
        processCommand(&serialFlash);
    }
    else
    {
        while(1){ asm("nop"); };
    } 
    
    // Check erasing/writing/reading from upper layer functions
    uint8_t largeBuffer[2048];
    
    /*setup a few bytes to write */ 
    for(int i=0; i<sizeof(largeBuffer); i++) largeBuffer[i] = (uint8_t)(i/512 + i*5);
    // erase, write and read
    if( eraseSector(SECTOR0_BASE_ADDR, SECTOR_0 )) {
        if (writeSerialFlashNE(largeBuffer, SECTOR0_BASE_ADDR, sizeof(largeBuffer), SECTOR_0)) {
            memset(largeBuffer, 0, sizeof(largeBuffer));
            readSerialFlash(SECTOR0_BASE_ADDR, largeBuffer, sizeof(largeBuffer));
        }
    }
    /*setup a few bytes to write */ 
    for(int i=0; i<sizeof(largeBuffer); i++) largeBuffer[i] = (uint8_t)(i/512 + i*7);
    // erase/write and read
    if (writeSerialFlash(largeBuffer, SECTOR1_BASE_ADDR, sizeof(largeBuffer), SECTOR_1)) {
        memset(largeBuffer, 0, sizeof(largeBuffer));
        readSerialFlash(SECTOR1_BASE_ADDR, largeBuffer, sizeof(largeBuffer));
    }
    /* Check if sector 0 not errased */
    readSerialFlash(SECTOR0_BASE_ADDR, largeBuffer, sizeof(largeBuffer));
    
#else

    /* test write upper than 4KB */
    uint8_t extralargeBuffer[4097];
    for(int i=0; i<sizeof(extralargeBuffer); i++) extralargeBuffer[i] = (uint8_t)(i/512 + i);
    // erase/write and read. Write more than the real size. Need to erase more than one page!!
    if (writeSerialFlash(extralargeBuffer, SECTOR2_BASE_ADDR, sizeof(extralargeBuffer), SECTOR_2)) {
        memset(extralargeBuffer, 0, sizeof(extralargeBuffer));
        readSerialFlash(SECTOR2_BASE_ADDR, extralargeBuffer, sizeof(extralargeBuffer));
    }

    /* check other sectors if not modified */
    readSerialFlash(SECTOR0_BASE_ADDR, extralargeBuffer, sizeof(extralargeBuffer));
    readSerialFlash(SECTOR1_BASE_ADDR, extralargeBuffer, sizeof(extralargeBuffer));
    readSerialFlash(SECTOR3_BASE_ADDR, extralargeBuffer, sizeof(extralargeBuffer));
#endif    
    
    PRINTF("End of unit test\n\r"); 
}
#endif

bool testReadWritePage()
{
  return true;
} 

bool testWriteProtection()
{
    bool status = false;
    /*set the write protection for the top most sector */
    setSectorLock( SECTOR_PROTECT_3); 

    /*verify sector is write protected */
    isSectorLocked(&serialFlash, SECTOR_3);
    if( serialFlash.sectorLocked )
    {
       status = true;        
    }

    /*clear the sector protection*/
    clearSectorLock(&serialFlash, SECTOR_3);
    isSectorLocked(&serialFlash, SECTOR_3);
    
    /*sector should be unlocked */
    if( !serialFlash.sectorLocked )
    {
       status = true;              
    }
    return status;
}

#endif  /* VANTRON_SBC_RK3568_NXP24_ARK */