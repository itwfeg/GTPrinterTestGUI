#ifndef DRAWINGPRIMITIVES_H
#define DRAWINGPRIMITIVES_H

/* BARCODES */
#define MODS_UPCA               95
#define UPC_LENGTH              10
#define RIGHT_MARGIN_MODS       7
#define LEFT_MARGIN_MODS        11

/* hobart 4 point font metrics */
#define CHARACTER_HEIGHT_4P     9
#define LEADING_4P              1
#define CHARACTER_ASCENT_4P     7
#define CHARACTER_FACE_4P       2


#define PIXELSPERWORD   15

typedef struct 
{
    unsigned short left;
    unsigned short top;
    unsigned short right;
    unsigned short bottom; 
}Rectangle;

typedef struct
{
    unsigned long  baseSize;
    unsigned short pitch;
    unsigned short *pBase;
    Rectangle coordinates;
}BitMap;

typedef enum
{
    DarkModule,
    LightModule  
}ModuleType;

typedef enum
{
    SymbolTableA,
    SymbolTableB,
    SymbolTableC
}SymbolTable;

typedef struct
{
    int x;
    int y;
}Coordinate;

typedef struct 
{
    unsigned short moduleSize;
    unsigned short guardHeight;
    unsigned short symbolHeight;
    unsigned short charOffSet;
    unsigned short charHeight;
    unsigned short leading;
    unsigned short faceMax;
    unsigned short ascent;
    Coordinate startPoint;
}UPCBarcode;

typedef struct
{
    unsigned short      signature;
    unsigned long       fileSize;
    unsigned short      reseverd1;
    unsigned short      reseverd2;
    unsigned long       imageOffset;
    unsigned long       headerSize;
    unsigned long       imageWidth;
    unsigned long       imageHeight;
    unsigned short      numPlanes;
    unsigned short      bitsPerPixel;
    unsigned long       compression;
    unsigned long       imageDataSize;
    unsigned long       horizontalRes;
    unsigned long       verticalRes;
    unsigned long       numColors;
    unsigned long       numImpColors;
    
}BmpHeader;

#endif
void initUPCBarcode( Rectangle *pRect, BitMap *pBitmap );
void blitUPCBarcode();
void byteSwapBuffer( unsigned char *pImage, unsigned long size );
static void drawGuard(UPCBarcode *pBarCode, BitMap *pBitmap);
static void drawModule(UPCBarcode *pBarCode, BitMap *pBitmap, ModuleType type );
static void drawMargin();
static void drawSymbol(UPCBarcode *pBarCode, BitMap *pBitmap, unsigned char symbolCode);
static unsigned char getSymbol( SymbolTable table, unsigned char data );
static void drawCenter(UPCBarcode *pBarCode, BitMap *pBitmap);
static void fillRectangle( BitMap *pBitmap, Rectangle *pRect );
