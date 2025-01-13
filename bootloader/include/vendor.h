#ifndef VENDOR_H
#define VENDOR_H


/* product id's */
#define G_SSRT_PRODUCT_ID               0x260   /* global service scale board */
#define G_FSSRT_PRODUCT_ID              0x270   /* global fresh service scale board */
#define G_PRINTER_PRODUCT_ID            0x280   /* global printer */
#define G_WEIGHER_PRODUCT_ID            0x290   /* global weigher */

/* public functions */
unsigned short getVendorID( void );
unsigned short getProductId( void );
unsigned char getPrinterSoftwareIDMajor( void );
unsigned char getPrinterSoftwareIDMinor( void );
unsigned char getPrinterSoftwareIDEng( void );
unsigned char getWeigherSoftwareIDMajor( void );
unsigned char getWeigherSoftwareIDMinor( void );
unsigned char getWeigherSoftwareIDEng( void );
unsigned char getBootVerMajor( void );
unsigned char getBootVerMinor( void );
unsigned char getBootVerEng( void );
unsigned char getHardwareIDMajor( void );
unsigned char getHardwareIDMinor( void );
unsigned char getPad1( void );
unsigned long long getAppVectorStart( void );
unsigned char getAppPrVerMajor( void );
unsigned char getAppPrVerMinor( void );
unsigned char getAppPrVerEng( void );
unsigned char getAppWgVerMajor( void );
unsigned char getAppWgVerMinor( void );
unsigned char getAppWgVerEng( void );
unsigned char getAppHwVerMajor( void );
unsigned char getAppHwVerMinor( void );
#endif
