#ifndef EEP93C56_H
#define EEP93C56_H
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "wgMessages.h"

#define EEP_93C56_CLK_FREQ    2000000U        /* 2MHz */ 

/* 93C56 Command Definition */
#define _93C56_ENABLE_CMD_1         0x04
#define _93C56_ENABLE_CMD_2         0xC0
#define _93C56_WRITE_CMD            0x05
#define _93C56_WRITE_ALL_CMD_1      0x04
#define _93C56_WRITE_ALL_CMD_2      0x40
#define _93C56_DISABLE_CMD_1        0x04
#define _93C56_DISABLE_CMD_2        0x00
#define _93C56_READ_CMD             0x06


#define BLANK_CHAR_93C56            0xFF
#define BLANK_LONG_93C56           -1
#define PAGE_SIZE_93C56             16


/* Data Structure */
union charShort {
   unsigned short us;
   unsigned char  uc[2];   
};

/* public prototypes */
bool initSpi93C56EEP( void );
unsigned short read93C56Word (unsigned char addr);
int write93C56MultiWord  (unsigned char addr, unsigned char *pData, unsigned char numBytes);
bool writeAll93C56 (void);
int32_t read93C56EEPMultiByte (unsigned char addr, unsigned char *pData, unsigned char numBytes);


/* private prototypes */
static bool getEEPSPIReady(void);
static void setEEPSPIReadyFalse(void);
static void eep93C56SpiCallBack( void );
static void disable93C56WriteAndErase (void);
static bool write93C56Word (unsigned char addr, unsigned short data);
static void enable93C56WriteAndErase (void);
static void changeClkPhase2ndEdge(void);
static void changeClkPhase1stEdge(void);
static void assertChipSelect93C56( void );
static void negateChipSelect93C56( void );
#endif