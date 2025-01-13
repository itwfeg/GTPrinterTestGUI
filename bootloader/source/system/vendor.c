#include "vendor.h"
#include "threadManager.h"
#include "common.h"

#pragma location = "VENDOR_BOOT_MAJOR"    /* version string: x.x.x */
const unsigned char bootSwM = 1; 
#pragma location = "VENDOR_BOOT_MINOR"
const unsigned char bootSwm = 0;
#pragma location = "VENDOR_BOOT_ENG"
const unsigned char bootSwe = 2;

#pragma required=bootSwM
#pragma required=bootSwm
#pragma required=bootSwe
#if 0
/* order necessary for flash loading in linker */
#pragma location = "VENDOR_PAD"
const unsigned char pad = 0xFF;
#pragma location = "VENDOR_VID"
const unsigned short deviceVid = 0x128c;
#pragma location = "VENDOR_PSW_MAJOR"    /* version string: x.x.x */
const unsigned char printerSwM = 0; 
#pragma location = "VENDOR_PSW_MINOR"
const unsigned char printerSwm = 0;
#pragma location = "VENDOR_PSW_ENG"
const unsigned char printerSwe = 99;
#pragma location = "VENDOR_WSW_MAJOR"    /* version string: x.x.x */
const unsigned char weigherSwM = 0; 
#pragma location = "VENDOR_WSW_MINOR"
const unsigned char weigherSwm = 0;
#pragma location = "VENDOR_WSW_ENG"
const unsigned char weigherSwe = 99;
#pragma location = "VENDOR_PAD1"
const unsigned char pad1 = 0xFF;
#pragma location = "VENDOR_HW_MAJOR"
const unsigned char deviceHwM = 0; 
#pragma location = "VENDOR_HW_MINOR"
const unsigned char deviceHwm = 1;


#pragma location="VENDOR_APP_START"
const unsigned long appVectorStart = 0x0000F000;


#pragma required=pad
#pragma required=deviceVid
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


unsigned short getVendorID( void )
{
    return deviceVid;  
}
#endif


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
    return id;
}

#if 0
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
#endif

//printer
unsigned char getAppPrVerMajor( void )
{
    unsigned char major = 0x0;
    memcpy(&major,(void*)(APP_VENDOR_START + VENDOR_APP_PR_MAJ_OFFSET),sizeof(major));
    return major;
}
unsigned char getAppPrVerMinor( void )
{
    unsigned char minor = 0x0;
    memcpy(&minor,(void*)(APP_VENDOR_START + VENDOR_APP_PR_MIN_OFFSET),sizeof(minor));
    return minor;
}
unsigned char getAppPrVerEng( void )
{
    unsigned char eng = 0x0;
    memcpy(&eng,(void*)(APP_VENDOR_START + VENDOR_APP_PR_ENG_OFFSET),sizeof(eng));
    return eng;
}

//Weigher
unsigned char getAppWgVerMajor( void )
{
    unsigned char major = 0x0;
    memcpy(&major,(void*)(APP_VENDOR_START + VENDOR_APP_WG_MAJ_OFFSET),sizeof(major));
    return major;
}
unsigned char getAppWgVerMinor( void )
{
    unsigned char minor = 0x0;
    memcpy(&minor,(void*)(APP_VENDOR_START + VENDOR_APP_WG_MIN_OFFSET),sizeof(minor));
    return minor;
}
unsigned char getAppWgVerEng( void )
{
    unsigned char eng = 0x0;
    memcpy(&eng,(void*)(APP_VENDOR_START + VENDOR_APP_WG_ENG_OFFSET),sizeof(eng));
    return eng;
}

//hardware
unsigned char getAppHwVerMajor( void )
{
    unsigned char major = 0x0;
    memcpy(&major,(void*)(APP_VENDOR_START + VENDOR_APP_HW_MAJ_OFFSET),sizeof(major));
    return major;
}
unsigned char getAppHwVerMinor( void )
{
    unsigned char minor = 0x0;
    memcpy(&minor,(void*)(APP_VENDOR_START + VENDOR_APP_HW_MIN_OFFSET),sizeof(minor));
    return minor;
}

#if 0 /* TO DO: */
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

#if 0
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
#endif