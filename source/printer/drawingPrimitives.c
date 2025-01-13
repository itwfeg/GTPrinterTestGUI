#include "drawingPrimitives.h"
#include "printhead.h"
#include "fsl_debug_console.h"

static unsigned char symbolTableA[10] = {0x0D,0x19,0x13,0x3d,0x23,0x31,0x2f,0x3b,0x37,0x0b};
static unsigned char symbolTableB[10] = {0x72,0x66,0x6c,0x42,0x5c,0x4e,0x50,0x44,0x48,0x74};
static unsigned char symbolTableC[10] = {0x27,0x33,0x1b,0x21,0x1d,0x39,0x05,0x11,0x09,0x17};

static const unsigned short rightMasks[] = 
{
    0x0000,   /* fill 0th position */
    0x0100, 0x0300, 0x0700, 0x0f00,
    0x1f00, 0x3f00, 0x7f00, 0xff00,
    0xff01, 0xff03, 0xff07, 0xff0f,
    0xff1f, 0xff3f, 0xff7f, 0xffff,
    0xffff
};


/******************************************************************************/
/*!   \fn void initUPCBarcode( Rectangle *pRect,  BitMap *pBitmap )

      \brief
        This function draws UPCBarcode into rectangle. 
   
      \author
          Aaron Swift
*******************************************************************************/
void initUPCBarcode( Rectangle *pRect,  BitMap *pBitmap )
{
    UPCBarcode barcode; 
    unsigned char symbolData[12] = { 0,1,2,3,4,5,6,7,8,9,0,1 };
    /* use hobart 4 point font */
    barcode.charHeight  = CHARACTER_HEIGHT_4P;
    barcode.leading     = LEADING_4P;
    barcode.ascent      = CHARACTER_ASCENT_4P;
    barcode.faceMax     = CHARACTER_FACE_4P;
    
    /* calculate symbol height based on 0 degree rotation */
    barcode.symbolHeight = pRect->bottom - pRect->top - barcode.charHeight - barcode.leading - 1;
    
    /* calculate guard height */
    barcode.guardHeight = barcode.symbolHeight +  ( barcode.charHeight / 2 );
    barcode.charOffSet = 0;
    
    /*calculate the field width */
    unsigned short fieldWidth = pRect->right - pRect->left;
    
    /* calculate the module size */
    barcode.moduleSize = fieldWidth / MODS_UPCA;
    
    /* calculate margin offset */
    unsigned short offset =  barcode.moduleSize * LEFT_MARGIN_MODS;
    
    /* determine the overall barcode width */
    unsigned short width =  barcode.moduleSize * ( LEFT_MARGIN_MODS + MODS_UPCA + RIGHT_MARGIN_MODS);

    /* deterrmine our starting coordinates */
    barcode.startPoint.x = pRect->right + offset;
    barcode.startPoint.y = pRect->top;
    
    /* will the barcode fit into our rectangle? */
    if( width < fieldWidth ) {
        /* draw the UPC barcode */
        drawGuard( &barcode, pBitmap );
        for(int i = 0; i < 6; i++) {
            drawSymbol( &barcode, pBitmap, getSymbol( SymbolTableA, symbolData[i] ) );
        }
        drawCenter( &barcode, pBitmap );
        for(int i = 6; i < 12; i++) {
            drawSymbol( &barcode, pBitmap, getSymbol( SymbolTableB, symbolData[i] ) );
        }
        drawGuard( &barcode, pBitmap );
    }
    
}

void byteSwapBuffer( unsigned char *pImage, unsigned long size )
{
    unsigned char temp = 0, *pSwap = pImage;
    if( pImage != NULL ) {
        int i = 0;
        while( i < size ) {
            /* save the first char */
            temp = *pImage;
            /* point to the next byte */
            pSwap++;
            /* move second byte to first */
            *pImage = *pSwap;
            
            /* move first byte to second */ 
            *pSwap = temp;
            /* next word */
            pImage += 2;
            pSwap = pImage;
            i += 2;
        }
    } else {       
        PRINTF("byteSwapBuffer(): pImage is NULL!\r\n");      
    }
}

/******************************************************************************/
/*!   \fn void reverseOrder64(  unsigned char *pImage, unsigned long size  )

      \brief
        This function places the image data in reverse order 64 bits at a time.
        This mimic the ordering the print head drivers needed to print the image 
        correctly. Without this ordering dot 448 is printed as dot 1 so the image
        on the label is mirrored.
      \author
          Aaron Swift
*******************************************************************************/
void reverseOrder64(  unsigned char *pImage, unsigned long size  )
{
    unsigned long long *pEndLine = (unsigned long long *)pImage + ( PRINTER_HEAD_SIZE_72MM / 8 ) - 1;
    unsigned long long  *pBegin = (unsigned long long *)pImage;
    unsigned long long temp[2] = { 0 };
    unsigned long long headBuffer[PRINTER_HEAD_SIZE_PREPACK / 8] = {0};
    int i = 0;
    while( i < size ) {
        /* reorder line one bank at a time. */
        for( int y = 0; y < PRINTER_HEAD_SIZE_72MM / 8; y++ ) {
            temp[0] =  *pBegin++;
            temp[1] = 0;
            /* reorder 64 bits at a time until done. */
            unsigned long long mask = 0xff00000000000000;
            unsigned long long t = 0;
        
            for( int x = 0; x < 7; x++ ) {                             
                t = temp[0] & mask;
                t <<= ( x * 8 );
                temp[1] |= t;
                temp[1] >>= 8;
                mask >>= 8;
            }
            t = temp[0] & mask;
            t <<= 56;
            /* save result to buffer */
            headBuffer[y] = temp[1] |= t;
        }
        
        /* rewrite the entire line  */
        for(  int x = 0; x < 7; x++ ) {
            *pEndLine = headBuffer[x];
            *pEndLine--;
        }
           
        i += PRINTER_HEAD_SIZE_72MM;
        /* move to the next print line */
        pBegin =(unsigned long long *)( pImage + i );
        pEndLine = (unsigned long long *)pBegin + ( PRINTER_HEAD_SIZE_72MM / 8 ) - 1;               
    }
}

void reverseOrderBuffer( unsigned char *pImage, unsigned long size )
{
    unsigned char right = 0, left = 0;
    unsigned char *pEndLine = pImage + PRINTER_HEAD_SIZE_72MM - 1;
    unsigned char *pBegin = pImage; 
    int i = 0;
    while( i < size ) {
        for( int x = 0; x < PRINTER_HEAD_SIZE_72MM /  2; x++ ) {
            /* grab values from either end of the line */
            right = *pImage;
            left = *pEndLine;
            
            /* re-assign values to opposite ends */
            *pEndLine = right;
            *pImage = left;
            *pEndLine--;
            *pImage++;
        }
        /* advance to the next line */
        i += PRINTER_HEAD_SIZE_80MM;
        /* reset our pointers */
        pImage = pBegin + i;
        pEndLine = pImage + PRINTER_HEAD_SIZE_72MM - 1;        
    }        
}



/******************************************************************************/
/*!   \fn static void drawGuard( UPCBarcode *pBarCode, BitMap *pBitmap )

      \brief
        This function draws UPCBarcode gaurd into the bitmap. 
   
      \author
          Aaron Swift
*******************************************************************************/
static void drawGuard( UPCBarcode *pBarCode, BitMap *pBitmap )
{
    drawModule( pBarCode, pBitmap, DarkModule );
    drawModule( pBarCode, pBitmap, LightModule );
    drawModule( pBarCode, pBitmap, DarkModule );  
}

/******************************************************************************/
/*!   \fn static void drawCenter( UPCBarcode *pBarCode, BitMap *pBitmap )

      \brief
        This function draws UPCBarcode center into the bitmap. 
   
      \author
          Aaron Swift
*******************************************************************************/
static void drawCenter( UPCBarcode *pBarCode, BitMap *pBitmap )
{
    drawModule( pBarCode, pBitmap, LightModule );
    drawModule( pBarCode, pBitmap, DarkModule );
    drawModule( pBarCode, pBitmap, LightModule );
    drawModule( pBarCode, pBitmap, DarkModule );
    drawModule( pBarCode, pBitmap, LightModule );  
}

/******************************************************************************/
/*!   \fn static void drawModule( UPCBarcode *pBarCode, BitMap *pBitmap, 
                                  ModuleType type )

      \brief
        This function draws UPCBarcode module into the bitmap. 
   
      \author
          Aaron Swift
*******************************************************************************/
static void drawModule( UPCBarcode *pBarCode, BitMap *pBitmap, ModuleType type )
{
    Rectangle rect;
    if( type == DarkModule ) {
        /* set starting coordinates for the rectangle  */
        rect.top = pBarCode->startPoint.y;
        rect.left = pBarCode->startPoint.x;
        
        /* get the width of the rectangle to fill */
        rect.right = pBarCode->startPoint.x + pBarCode->moduleSize;
        
        /* get the height of the rectangle to fill */
        rect.bottom = pBarCode->startPoint.y + pBarCode->guardHeight; 
        
        /* blit the rectangle */
        fillRectangle( pBitmap, &rect );
        
        /* move coordinates to the next module */
        pBarCode->startPoint.x = pBarCode->startPoint.x + pBarCode->moduleSize;
    }   
}

/******************************************************************************/
/*!   \fn static void drawSymbol( UPCBarcode *pBarCode, BitMap *pBitmap, 
                                  unsigned char symbolCode )

      \brief
        This function draws UPCBarcode symbol into the bitmap. 
   
      \author
          Aaron Swift
*******************************************************************************/
static void drawSymbol( UPCBarcode *pBarCode, BitMap *pBitmap, unsigned char symbolCode )
{
    ModuleType currentType, lastType;
    short moduleWidth = 1;
    unsigned char testByte = 0x40;	/* seven bit symbol code */

    currentType = (symbolCode & testByte) ? DarkModule : LightModule;
    lastType = currentType;
    testByte = testByte >> 1;

    while(testByte > 0)
    {
            currentType = (symbolCode & testByte) ? DarkModule : LightModule;
            if(currentType == lastType)
            {
                    moduleWidth++;
            }
            else
            {
                    drawModule( pBarCode, pBitmap, lastType );
                    lastType = currentType;
                    moduleWidth = 1;
            }
            testByte = testByte >> 1;
    }
    drawModule( pBarCode, pBitmap, lastType );  
}

/******************************************************************************/
/*!   \fn static unsigned char getSymbol( SymbolTable table, unsigned char data ) 
                                 
      \brief
        This function returns UPCBarcode symbol from symbol table. 
   
      \author
          Aaron Swift
*******************************************************************************/
static unsigned char getSymbol( SymbolTable table, unsigned char data )
{
    unsigned char symbol = 0;
    if( data > 9 ) {
        switch( table )
        {
            case SymbolTableA: {
                symbol = symbolTableA[ data ];
                break;
            }
            case SymbolTableB: {
                symbol = symbolTableB[ data ];
                break;
            }
            case SymbolTableC: {
                symbol = symbolTableC[ data ];              
                break;
            }
        }    
    } else {
        PRINTF("getSymbol(): barcode data out of range: %d valid range is 0 - 9!\r\n", data );
    }
    return symbol;
}

/******************************************************************************/
/*!   \fn static void fillRectangle( BitMap *pBitmap, Rectangle *pRect )

      \brief
       
   
      \author
          Aaron Swift
*******************************************************************************/
static void fillRectangle( BitMap *pBitmap, Rectangle *pRect )
{
    long  srcTop, srcRecLeft;
    short leftMask, rightMask, rowWords,  mask, oldWord, newWord;
    short width, height, delta;
    short *pSrcBitPlane, i;
    unsigned char endPixel, startPixel;

    /* determine if the rectangle dimensions are valid */
    width = pRect->right - pRect->left; 
    height = pRect->bottom - pRect->top;
    if( ( width <= 0 ) || ( height <= 0 ) ){
        PRINTF("fillRectangle(): rectangle dimensions invalid! width: %d height: %d\r\n", width, height);
        return;
    }

    /* verify the rectangle is within the bitmap's bounds */
    if ( (pRect->top < pBitmap->coordinates.top || pBitmap->coordinates.bottom <= pRect->top || pBitmap->coordinates.bottom < pRect->bottom) ||
         (pRect->left < pBitmap->coordinates.left || pBitmap->coordinates.right <= pRect->left || pBitmap->coordinates.right < pRect->right) ) {        
            PRINTF("fillRectangle(): rectangle out of bounds of bitmap. top: %d bottom: %d right: %d left: %d\r\n", pRect->top, pRect->bottom, pRect->right, pRect->left);
            PRINTF("fillRectangle(): bitmap bounds top: %d bottom: %d right: %d left: %d\r\n", pBitmap->coordinates.top, pBitmap->coordinates.bottom, pBitmap->coordinates.right, pBitmap->coordinates.left);
            return;
    }

    /* determine which bit within the ending word is the endpixel */
    endPixel = (unsigned char)( PIXELSPERWORD - ( ( pRect->right - pBitmap->coordinates.left - 1 ) & 0x0f ) );
    rightMask = ~( rightMasks[endPixel] );	//maskoff the slop that is left over in the word

    /* determine which bit within the first word is the startpixel */
    startPixel = (unsigned char)( ( PIXELSPERWORD + 1 ) - ( ( pRect->left - pBitmap->coordinates.left ) & 0x0f ) );
    
    /* maskoff the slop that is left over in the word */
    leftMask = rightMasks[startPixel];		

    if( width <= startPixel ) {
        rowWords = 1;	
        leftMask &= rightMask; 
    } else {
        rowWords = ( ( width - startPixel - 1 ) >> 4 ) + 2;
    }

    delta = pBitmap->pitch - rowWords;

    srcTop = pRect->top - pBitmap->coordinates.top;																	
    srcTop = srcTop * pBitmap->pitch;
    srcRecLeft = pRect->left - pBitmap->coordinates.left;
    srcRecLeft >>= 4;
    pSrcBitPlane  = (short *)pBitmap->pBase + srcTop + srcRecLeft;

    /* complete the bitmap from top to bottom */
    for(; height > 0; height--) {	
        /* starting on the left side */
        mask = leftMask;
        
        for(i = rowWords; i > 0; i--) {
            oldWord = *pSrcBitPlane;
            newWord = ( ( mask & 0xffff ) | ( ~mask & oldWord ) );
            /* copy the new word to the bitmap */
            *pSrcBitPlane++ = newWord;

            /* discard the left mask */
            mask = 0xFFFF;
            /* are we on the last word in the row? */
            if( i == 2 ) {
                mask = rightMask;
            }
        }
        /* move to the start of the next line */
        pSrcBitPlane += delta;
    }  
}