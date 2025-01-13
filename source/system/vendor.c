#include "vendor.h"
#include "threadManager.h"
#include "fsl_debug_console.h"
#include "developmentSettings.h"

    

const char buildDate[16] =  __DATE__;
const char buildTime[16] =  __TIME__;
   

/* order necessary for flash loading in linker */
/*  
  TFink 5/21/24 - Needed one more byte to fit in allocated area 
  (..F0 to ..FC). I'm guessing there was an "unseen" pad. Only had to 
  allocate one more byte for it to work. 

  In addition, eliminating VENDOR_PAD and VENDOR_VID positions the 
  Vendor data to match bootloader offsets as seen in bootloader "common.h"

#pragma location = "VENDOR_PAD"
const unsigned char pad = 0xFF;
#pragma location = "VENDOR_VID"
const unsigned short deviceVid = 0x128c;
*/
/* TFink 241014 Delete this Comment. Used to fool GIT into allowing a Pull Request */

#pragma location = "VENDOR_PSW_MAJOR"    /* version string: x.x.x */
const unsigned char printerSwM = 0; 
#pragma location = "VENDOR_PSW_MINOR"
const unsigned char printerSwm = 1;
#pragma location = "VENDOR_PSW_ENG"
const unsigned char printerSwe = 1;
#pragma location = "VENDOR_WSW_MAJOR"    /* version string: x.x.x */
const unsigned char weigherSwM = 1; 
#pragma location = "VENDOR_WSW_MINOR"
const unsigned char weigherSwm = 2;
#pragma location = "VENDOR_WSW_ENG"
const unsigned char weigherSwe = 2;
#pragma location = "VENDOR_PAD1"
const unsigned char pad1 = 0xFF;
#pragma location = "VENDOR_HW_MAJOR"
const unsigned char deviceHwM = 0; 
#pragma location = "VENDOR_HW_MINOR"
const unsigned char deviceHwm = 1;
#pragma location="VENDOR_APP_START"
const unsigned long appVectorStart = 0x60040000;




/******************** printer revision history ********************************/
/*  1.0.8: Changed label taken threshold and removed changes for 10K pull     */
/*         down bias.                                                         */
/*  1.0.11 Support bias change to label taken sensor.                         */
/*  1.0.12 Flip strobe pwm from high to low as per Hung ( energy reduction )  */
/*         added expel to print operation for reliable expel distance.        */
/*  1.0.13 Paper takeup at end of print cycle.                                */
/*  1.0.14 Missing print line after print buffer rollover. Chris's paper      */
/*         takeup and MOC values                                              */
/*  1.0.15 Merge of chris's paper takeup                                      */
/*  1.0.17 Weihger usb fix for stall on bootup?                               */
/*  1.0.18 White line on label types over 3.5" ( rollover )                   */
/*  1.0.19 Chris's takeup changes and filter sending weigher status as to     */
/*         not spam the backend with weigher status                           */
/*  1.0.20 Timer not initialized in Weigher autozero and intializeZero        */
/*         functions                                                          */
/*  1.0.21 Timer not initialized in small or large motion detection           */
/*  1.0.22 Backend crash when cassette removed on boot of the scale           */
/*  1.0.24 80mm support                                                       */
/*  1.0.34 03/20/2024                                                         */
/*                                                                            */
/******************************************************************************/


/******************** printer revision history ********************************/
/** ver  date    time   engr     description                                 **/
/*  1.2 5/14/24         T Fink   1st Released version. Tested at NTEP         */
/*  1.3 5/21/24 ~11:06  T Fink   2nd Released version. Major code cleanup &   */
/*      logic rewrite but the code that does calculations didn't change       */
/******************************************************************************/


/* TFink 5/21/24
#pragma required=pad
#pragma required=deviceVid
*/
#pragma required=printerSwM
#pragma required=printerSwm
#pragma required=printerSwe
#pragma required=weigherSwM
#pragma required=weigherSwm
#pragma required=weigherSwe
#pragma required=pad1
#pragma required=deviceHwM
#pragma required=deviceHwm
#pragma required=appVectorStart


void printSWVersions(void)
{
  PRINTF("GT Printer Test");
  PRINTF("Build Date: %s\r\n", buildDate);
  PRINTF("Build Time: %s\r\n", buildTime);
  PRINTF("GT Printer Test version: %d.%d.%d\r\n",printerSwM,printerSwm,printerSwe);
  //PRINTF("WeigherSWVersion: %d.%d.%d\r\n",weigherSwM,weigherSwm,weigherSwe); 
}


#define G_SSRT_PRODUCT_GOOD_ID          0x260   /* global service scale board good configuration */
#define G_SSRT_PRODUCT_BETTER_ID        0x262   /* global service scale board better configuration */
#define G_SSRT_PRODUCT_BEST_ID          0x264   /* global service scale board best configuration */

#define G_FSSRT_PRODUCT_ID              0x270   /* global fresh service scale board */
#define G_SSRT_PRINTER_ONLY_PRODUCT_ID  0x280   /* global scale printer only */
#define G_PPRT_PRINTER_PRODUCT_ID       0x282   /* global prepack printer only */
#define G_SSRT_WEIGHER_ONLY_PRODUCT_ID  0x290   /* global scale weigher only */
#define G_PPRT_WEIGHER_PRODUCT_ID       0x292   /* global prepack weigher only */

unsigned short getProductId( void )
{
    unsigned short id = 0;
    /* our product id is based off of which model we are */
    PeripheralModel model = getMyModel();  

    switch( model )
    {
        case RT_GLOBAL_SCALE_GOOD: {
            id = G_SSRT_PRODUCT_GOOD_ID;
            break;
        }
        case RT_GLOBAL_SCALE_BETTER: {
            id = G_SSRT_PRODUCT_BETTER_ID;
            break;          
        }
        case RT_GLOBAL_SCALE_BEST: {
            id = G_SSRT_PRODUCT_BEST_ID;
            break;          
        }
        case RT_GLOBAL_FSS: {
            id = G_FSSRT_PRODUCT_ID;
            break;
        }
        case RT_GLOBAL_SCALE_PRINTER_ONLY: {
            id = G_SSRT_PRINTER_ONLY_PRODUCT_ID;
            break;
        }
        case RT_GLOBAL_PREPACK_PRINTER: {
            id = G_PPRT_PRINTER_PRODUCT_ID;
            break;
        }
        case RT_GLOBAL_SCALE_WEIGHER_ONLY: {
            id = G_SSRT_WEIGHER_ONLY_PRODUCT_ID;
            break;
        }
        case RT_GLOBAL_PREPACK_WEIGHER: {
            id = G_PPRT_WEIGHER_PRODUCT_ID; 
            break;
        }
        default: {
        }
    }    
    
    //PRINTF("getProductId() - %d\r\n", model);
    
    return id;
}


unsigned char getPrinterSoftwareIDMajor( void )
{
    return printerSwM;
}

unsigned char getPrinterSoftwareIDMinor( void )
{
    return printerSwm;
}

unsigned char getPrinterSoftwareIDEng( void )
{
    return printerSwe;
}

unsigned char getWeigherSoftwareIDMajor( void )
{
    return weigherSwM;
}

unsigned char getWeigherSoftwareIDMinor( void )
{
    return weigherSwm;
}

unsigned char getWeigherSoftwareIDEng( void )
{
    return weigherSwe;
}

unsigned char getHardwareIDMajor( void )
{
    return deviceHwM;  
}
#if 0 /* TO DO: */
unsigned char getBootVerMajor( void )
{
    unsigned char major = 0x0;
    memcpy(&major,(void*)(BOOT_VENDOR_START + B_VENDOR_SW_MAJ_OFFSET),sizeof(major));
    return major;
}

unsigned char getBootVerMinor( void )
{
    unsigned char minor = 0x0;
    memcpy(&minor,(void*)(BOOT_VENDOR_START + B_VENDOR_SW_MIN_OFFSET),sizeof(minor));
    return minor;
}

unsigned char getBootVerEng( void )
{
    unsigned char eng = 0x0;
    memcpy(&eng,(void*)(BOOT_VENDOR_START + B_VENDOR_SW_ENG_OFFSET),sizeof(eng));
    return eng;
}
#endif


unsigned long long getAppVectorStart( void )
{
    return appVectorStart;
}


unsigned char getHardwareIDMinor( void )
{
    return deviceHwm;
}

unsigned char getPad1()
{
    return pad1;
}