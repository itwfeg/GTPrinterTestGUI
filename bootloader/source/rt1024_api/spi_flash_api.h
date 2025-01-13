/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef SPI_FLASH_API_H
#define SPI_FLASH_API_H

#include <stdbool.h>
#include <stdint.h>

#include "fsl_romapi.h"

#define FlexSpiInstance           0U
#define FLEXSPI_AMBA_BASE FlexSPI_AMBA_BASE
#define FLASH_SIZE                0x400000UL /* 4MBytes */
#define FLASH_PAGE_SIZE           256UL      /* 256Bytes */
#define FLASH_SECTOR_SIZE         0x1000UL   /* 4KBytes */
#define FLASH_BLOCK_SIZE          0x10000UL  /* 64KBytes */
#define BUFFER_LEN FLASH_PAGE_SIZE
#define APP_SIZE ( APP_END - APP_VECTOR_START )

static flexspi_nor_config_t norConfig;

// debugging
static uint8_t buffer_readback[1024];
   
#if 0
static inline void FLEXSPI_ClockInit(void)
{
    const clock_usb_pll_config_t g_ccmConfigUsbPll = {.loopDivider = 0U};

    CLOCK_InitUsb1Pll(&g_ccmConfigUsbPll);
    CLOCK_InitUsb1Pfd(kCLOCK_Pfd0, 24);   /* Set PLL3 PFD0 clock 360MHZ. */
    CLOCK_SetMux(kCLOCK_FlexspiMux, 0x3); /* Choose PLL3 PFD0 clock as flexspi source clock. */
    CLOCK_SetDiv(kCLOCK_FlexspiDiv, 2);   /* flexspi clock 120M. */
}
#endif

#if !(defined(XIP_EXTERNAL_FLASH) && (XIP_EXTERNAL_FLASH == 1))
void BOARD_SetupFlexSpiClock(void);
#endif
void FLEXSPI_NorFlash_GetConfig(flexspi_nor_config_t *config);

/* flexspi flash functionality */
bool spi_flash_init();
//void spi_flash_deinit(); // probs not needed



AT_QUICKACCESS_SECTION_CODE (bool spi_flash_erase(uint32_t address, uint32_t size));
AT_QUICKACCESS_SECTION_CODE (bool spi_flash_erase_app());
AT_QUICKACCESS_SECTION_CODE (bool spi_flash_program(uint32_t start, const uint32_t *src, uint32_t size));

#endif