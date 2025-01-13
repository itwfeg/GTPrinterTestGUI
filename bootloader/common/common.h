#ifndef COMMON_H
#define COMMON_H
/* ------------------------------------------------------------------------
 * This is a shared file between the bootloader and application but since
 * they are seperate projects it will not create depedencies. These values
 * must match the linker files
 * ------------------------------------------------------------------------*/

//BOOTLOADER common.h

/* spi flash locations */
#define SPI_FLASH_START                 0x60000000
     
/* Application memory locations */
#define APP_VENDOR_START                0x603FFFF0
#define APP_VENDOR_END                  0x603FFFFB

#define VENDOR_APP_START_OFFSET         0x0
#define VENDOR_PAD_OFFSET               0x0
#define VENDOR_APP_PR_MAJ_OFFSET        0x0
#define VENDOR_APP_PR_MIN_OFFSET        0x1
#define VENDOR_APP_PR_ENG_OFFSET        0x2
#define VENDOR_APP_WG_MAJ_OFFSET        0x3
#define VENDOR_APP_WG_MIN_OFFSET        0x4
#define VENDOR_APP_WG_ENG_OFFSET        0x5
#define VENDOR_APP_HW_MIN_OFFSET        0x6
#define VENDOR_APP_HW_MAJ_OFFSET        0x7

#define APP_VECTOR_START                0x60040000  
#if 0
#define APP_CHECKSUM_START              0x600BFFFA
#define APP_END                         0x600BFFFD
#define SOURCE_APP_END                  0x600444FE
#else
#define APP_CHECKSUM_START              0x603FFFFC
#define APP_END                         0x603FFFFF
#define SOURCE_APP_END                  0x603FFFEF
#endif

#define APP_STACK_ADDR                  0x20200000

/* Bootloader memory locations */
#define BOOT_VECTOR_START               0x60002000
#define BOOT_VENDOR_START               0x6003FFF0
#define BOOT_VENDOR_END                 0x6003FFF9
#define BOOT_CHECKSUM_START             0x6003FFFA
#define BOOT_END                        0x6003FFFF
                                          
#endif                                   
                                         