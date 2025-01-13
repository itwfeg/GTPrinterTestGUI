#include "spi_flash_api.h"
#include "MIMXRT1024.h"
#include "fsl_debug_console.h"
#include "fsl_cache.h"
#include "common.h"
#include "clock_config.h"
#include "fsl_flexspi.h"

// from the flexspi_romapi example in the SDK
void FLEXSPI_NorFlash_GetConfig(flexspi_nor_config_t *config)
{
    config->memConfig.tag              = FLEXSPI_CFG_BLK_TAG;
    config->memConfig.version          = FLEXSPI_CFG_BLK_VERSION;
    //config->memConfig.readSampleClkSrc = kFLEXSPIReadSampleClk_LoopbackFromDqsPad;
    config->memConfig.readSampleClkSrc     = kFLEXSPIReadSampleClk_LoopbackInternally,
    config->memConfig.serialClkFreq =
        kFLEXSPISerialClk_60MHz;
        //kFLEXSPISerialClk_133MHz; /* Serial Flash Frequencey.See System Boot Chapter for more details */
    config->memConfig.sflashA1Size   = FLASH_SIZE;
    config->memConfig.csHoldTime     = 3U;                           /* Data hold time, default value: 3 */
    config->memConfig.csSetupTime    = 3U;                           /* Date setup time, default value: 3 */
    config->memConfig.deviceType     = kFLEXSPIDeviceType_SerialNOR; /* Flash device type default type: Serial NOR */
    config->memConfig.deviceModeType = kDeviceConfigCmdType_Generic;
    config->memConfig.columnAddressWidth  = 0U;
    config->memConfig.deviceModeCfgEnable = 0U;
    config->memConfig.waitTimeCfgCommands = 0U;
    config->memConfig.configCmdEnable     = 0U;
    /* Always enable Safe configuration Frequency */
    //config->memConfig.controllerMiscOption = FSL_ROM_FLEXSPI_BITMASK(kFLEXSPIMiscOffset_SafeConfigFreqEnable);
    config->memConfig.controllerMiscOption = (1u << kFLEXSPIMiscOffset_SafeConfigFreqEnable);
    config->memConfig.sflashPadType = kSerialFlash_4Pads; /* Pad Type: 1 - Single, 2 - Dual, 4 - Quad, 8 - Octal */
    config->pageSize                = FLASH_PAGE_SIZE;
    config->sectorSize              = FLASH_SECTOR_SIZE;
    config->blockSize               = FLASH_BLOCK_SIZE;
    //config->ipcmdSerialClkFreq      = kFLEXSPISerialClk_30MHz; /* Clock frequency for IP command */
    config->ipcmdSerialClkFreq      = 1u; /* Clock frequency for IP command */
    
    // Read LUTs
    config->memConfig.lookupTable[0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0xEB, RADDR_SDR, FLEXSPI_4PAD, 0x18);
    config->memConfig.lookupTable[1] = FLEXSPI_LUT_SEQ(DUMMY_SDR, FLEXSPI_4PAD, 0x06, READ_SDR, FLEXSPI_4PAD, 0x04);

    // Read Status LUTs
    config->memConfig.lookupTable[4 * 1 + 0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x05, READ_SDR, FLEXSPI_1PAD, 0x04);

    // Write Enable LUTs
    config->memConfig.lookupTable[4 * 3 + 0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x06, STOP, FLEXSPI_1PAD, 0x0);

    // Erase Sector LUTs
    config->memConfig.lookupTable[4 * 5 + 0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x20, RADDR_SDR, FLEXSPI_1PAD, 0x18);

    // Erase Block LUTs
    config->memConfig.lookupTable[4 * 8 + 0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0xD8, RADDR_SDR, FLEXSPI_1PAD, 0x18);

    // Pape Program LUTs
    config->memConfig.lookupTable[4 * 9 + 0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x02, RADDR_SDR, FLEXSPI_1PAD, 0x18);
    config->memConfig.lookupTable[4 * 9 + 1] = FLEXSPI_LUT_SEQ(WRITE_SDR, FLEXSPI_1PAD, 0x04, STOP, FLEXSPI_1PAD, 0x0);

    // Erase Chip LUTs
    config->memConfig.lookupTable[4 * 11 + 0] = FLEXSPI_LUT_SEQ(CMD_SDR, FLEXSPI_1PAD, 0x60, STOP, FLEXSPI_1PAD, 0x0);
}

#if !(defined(XIP_EXTERNAL_FLASH) && (XIP_EXTERNAL_FLASH == 1))
/* Configure clock for FlexSPI peripheral */
void BOARD_SetupFlexSpiClock(void)
{
    /* Disable FlexSPI peripheral */
    FLEXSPI->MCR0 |= FLEXSPI_MCR0_MDIS_MASK;
    CLOCK_DisableClock(kCLOCK_FlexSpi);

    /* Init Usb1 PLL3 to 480MHZ. */
    CLOCK_InitUsb1Pll(&usb1PllConfig_BOARD_BootClockRUN);
    /* Init Usb1 PLL3->pfd0 360MHZ. */
    CLOCK_InitUsb1Pfd(kCLOCK_Pfd0, 24);
    /* Enable Usb1 PLL output for USBPHY1. */
    CCM_ANALOG->PLL_USB1 |= CCM_ANALOG_PLL_USB1_EN_USB_CLKS_MASK;

    /* Set FLEXSPI_PODF, FlexSPI out clock is 60MHZ. */
    CLOCK_SetDiv(kCLOCK_FlexspiDiv, 5);
    /* Set flexspi clock source to PLL3->pfd0 */
    CLOCK_SetMux(kCLOCK_FlexspiMux, 3);

    CLOCK_EnableClock(kCLOCK_FlexSpi);
    /* Enable FlexSPI peripheral */
    FLEXSPI->MCR0 &= ~FLEXSPI_MCR0_MDIS_MASK;
}
#endif

/******************************************************************************/
/*! \fn bool spi_flash_init()

    \brief
        Initilizes the spi not flash driver to enable application programming

    \return 
        returns true if the driver is sucessfully initialized

    \author
        Joseph DiCarlantonio
*******************************************************************************/
bool spi_flash_init()
{   
#if !(defined(XIP_EXTERNAL_FLASH) && (XIP_EXTERNAL_FLASH == 1))
    BOARD_SetupFlexSpiClock();
#endif

    status_t status;
    uint32_t numSectors;

    // configure nor flash
    memset(&norConfig, 0U, sizeof(flexspi_nor_config_t));
    FLEXSPI_NorFlash_GetConfig(&norConfig);

    status = ROM_FLEXSPI_NorFlash_Init(FlexSpiInstance, &norConfig);
    if(status != kStatus_Success) {
        PRINTF("Could not initialize nor flash\r\n");
        return false;
    }

    // software reset
    ROM_FLEXSPI_NorFlash_ClearCache(FlexSpiInstance);

    return true;
}

/******************************************************************************/
/*! \fn bool spi_flash_erase_app()

    \brief
        Erases the entire program space for the application

    \return 
        returns true the application program space is erased 

    \author
        Joseph DiCarlantonio
*******************************************************************************/
bool spi_flash_erase_app()
{
    status_t status;

    // get the sector aligned address and size
    uint32_t serialNorTotalSize  = norConfig.memConfig.sflashA1Size;
    uint32_t serialNorSectorSize = norConfig.sectorSize;
    uint32_t numSectors = APP_SIZE / serialNorSectorSize;

    uint32_t sectorAlignedSize = numSectors * serialNorSectorSize;
    uint32_t serialNorAddress = serialNorTotalSize - sectorAlignedSize;  
    uint32_t appOffset = (APP_VECTOR_START - FlexSPI_AMBA_BASE);// / serialNorSectorSize;

    status = ROM_FLEXSPI_NorFlash_Erase(FlexSpiInstance, &norConfig, appOffset, APP_SIZE);
    if(status != kStatus_Success) {
        PRINTF("Failed to erase application!\r\n");
        return false;
    }

    return true;
}

/******************************************************************************/
/*! \fn bool spi_flash_program(uint32_t start, uint32_t *src, uint32_t lengthInBytes)

    \brief
        Programs the application, this is a call to ROM_FLEXSPI_NorFlashProgramPage,
        which programs a whole page (256 bytes) at a time.

    \arg
        start:          start address of spi nor flash to be programmed
        src:            pointer to buffer of data being programmed
        length:         length in bytes        

    \return 
        returns true the application program space is erased and verified.

    \author
        Joseph DiCarlantonio
*******************************************************************************/
bool spi_flash_program(uint32_t start, const uint32_t *src, uint32_t size)
{
    status_t status;
    uint32_t serialNorTotalSize = norConfig.memConfig.sflashA1Size;
    uint32_t serialNorPageSize = norConfig.pageSize;
    uint32_t appOffset = (start - FlexSPI_AMBA_BASE); // / serialNorPageSize;
    
    status = ROM_FLEXSPI_NorFlash_ProgramPage(
                FlexSpiInstance, 
                &norConfig, 
                appOffset, // + i * size, 
                src // + i * size
    );
    
    if( status != kStatus_Success ) {
        PRINTF( "Failed to program flash\r\n" );
        return false;
    }
    
    return true;
}