#include "printHead.h"
#include "serialFlash.h"
#include "filter.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "printEngine.h"
#include "sensors.h"
#include "fsl_debug_console.h"
#include "lp5521.h"

HeadStyle headStyle;
static PrinterEnv env_;
extern FilterData    headTempData;
extern bool paused_;
static unsigned short prevContrast_ = 0;
extern Pr_Config config_;
extern PrStatusInfo currentStatus;
extern PrStatusInfo prevStatus;

AT_NONCACHEABLE_SECTION( static ImageBfrMgr imageBfrMgr );
AT_NONCACHEABLE_SECTION( unsigned char imageBuffer[ PRINTER_BUFFER_SIZE_80MM ] );
AT_NONCACHEABLE_SECTION( unsigned char dotWear[HEAD_DOTS_72MM] );

unsigned char lastBfrBytes[512] = { 0 }; 

#if 1   /* 6ips timing values from Hung */
const unsigned short rohm80mmSLTTimes[10] = { 1083, 1083, 1245, 1516, 1516, 1895, 1895, 1895, 2166, 2166 }; 
const unsigned short rohm80mmHistory[8] = { 550, 400, 500, 700, 665, 1020, 975, 935 };
const unsigned short rohm80mmCurrentLine[8] = { 685, 585, 725, 935, 888, 1260, 1229, 1197 };
const unsigned short rohm80mmPwmStart[8] = { 1083, 1083, 1245, 1516, 1516, 1895, 1895, 1895 };
const unsigned short rohm80mmPwmDuty[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
#else
const unsigned short rohm80mmSLTTimes[10] = { 1083, 1083, 1083, 1083, 1083, 1245, 1516, 1895, 2166, 2166 }; 
const unsigned short rohm80mmHistory[8] = { 625, 550, 475, 400, 333, 500, 700, 1025 };
const unsigned short rohm80mmCurrentLine[8] = { 735, 685, 635, 585, 535, 725, 935, 1275};
const unsigned short rohm80mmPwmStart[8] = { 1083, 1083, 1083, 1083, 1083, 1245, 1516, 1895 };
const unsigned short rohm80mmPwmDuty[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
#endif

/* print head timing tables */
unsigned short historyTimes[MAX_TABLE_ENTRIES];
unsigned short adjacencyTimes[MAX_TABLE_ENTRIES];
unsigned short currentLineTimes[MAX_TABLE_ENTRIES];
unsigned short pwmTimes[MAX_TABLE_ENTRIES];
unsigned char pwmDutyCycle[MAX_TABLE_ENTRIES];

/* history line */
unsigned char history1Line[ PRINTER_HEAD_SIZE_80MM ];	

/******************************************************************************/
/*!   \fn void intializePrintHead(  unsigned int contrast  )                                                          
 
      \brief
        This function initalizes the appropriate timing values for the 
        printhead based on head type, temperature and contrast.

      \author
          Aaron Swift
*******************************************************************************/ 
void intializePrintHead(  unsigned int contrast,  PrinterEnv env )
{
    HEADTYPE headType = UNKNOWN_HEAD;
    
    env_ = env;

    headType = getPrintHeadType();
    
    /* intialize our head style */ 
    if( env_ == RT_SERVICE_72MM )    
        headStyle.headSize = HEAD_DOTS_72MM;
    else
        headStyle.headSize = HEAD_DOTS_80MM;
    
    headStyle.headType = headType;
       
    clearHistory( env );
    
    /* initialize image buffer manager */
    initializeImageBfr( &imageBfrMgr );
    
    /* initialize head timings for all temperature ranges */
    switch( headType )
    {
        case ROHM_72MM_800_OHM : 
        {
            for( int i = 0; i <= 6; i++ ) 
            {          
                historyTimes[i]         = rohm80mmHistory[contrast];
                adjacencyTimes[i]       = 0;
                currentLineTimes[i]     = rohm80mmCurrentLine[contrast];
                pwmTimes[i]             = rohm80mmPwmStart[contrast];
                pwmDutyCycle[i]         = rohm80mmPwmDuty[contrast];
            }
            break;
        }
        case ROHM_80MM_650_OHM : 
        {
            for( int i = 0; i <= 6; i++ ) 
            {          
                historyTimes[i]         = rohm80mmHistory[contrast];
                adjacencyTimes[i]       = 0;
                currentLineTimes[i]     = rohm80mmCurrentLine[contrast];
                pwmTimes[i]             = rohm80mmPwmStart[contrast];
                pwmDutyCycle[i]         = rohm80mmPwmDuty[contrast];
            }
            break;
        }
        default: 
        {
            PRINTF("intializePrintHead(): Unknown head type. critical error!\r\n" );
            break;
        } 
    }
    
    /* initialize print head data latch */
    gpio_pin_config_t dataLatch = { kGPIO_DigitalOutput, 1, };
    GPIO_PinInit( PHEAD_LATCH_GPIO, PHEAD_LATCH_PIN, &dataLatch );  
    
    /* initialize print head strobe */
    gpio_pin_config_t strobe = { kGPIO_DigitalOutput, 1, };
    GPIO_PinInit( PHEAD_STROBE_A_GPIO, PHEAD_STROBE_A_PIN, &strobe );  
    GPIO_PinInit( PHEAD_STROBE_B_GPIO, PHEAD_STROBE_B_PIN, &strobe );
    
    /* initialize print head strobe enable */
    gpio_pin_config_t enable = { kGPIO_DigitalOutput, 1, };
    GPIO_PinInit( PHEAD_STROBE_EN_GPIO, PHEAD_STROBE_EN_PIN, &enable );  
    
    /*intialize head power io */
    gpio_pin_config_t headPower = { kGPIO_DigitalOutput, 0, }; 
    GPIO_PinInit( PHEAD_POWER_EN_GPIO, PHEAD_POWER_EN_PIN, &headPower );  
       
    /* turn on the head power  */
    setHeadPower( 1 ); 
    initializePrintHeadSPI();
}

/******************************************************************************/
/*!   \fn void initializeImageBfr( ImageBfrMgr *pMgr )                                                          
 
      \brief
        This function initializes the manager to the image buffer used to 
        manage transfers of usb data to the image buffer for printing. 

      \author
          Aaron Swift
*******************************************************************************/ 
void initializeImageBfr( ImageBfrMgr *pMgr )
{
    static unsigned long buffSize = 0;
    
     if(getPrintHeadType() == ROHM_72MM_800_OHM)
    {
        buffSize = PRINTER_BUFFER_SIZE_72MM;
    }
    else
    {
        buffSize = PRINTER_BUFFER_SIZE_80MM;
    }

    if( pMgr != NULL ) 
    {
        pMgr->pBottom           = &imageBuffer[0];
        pMgr->pTop              = &imageBuffer[ buffSize ];
        pMgr->bufferSize        = buffSize;
        pMgr->index             = 0;
        pMgr->numXfersToFill    = ( buffSize / 512 );
        pMgr->remainder         = buffSize % 512; 
        pMgr->labelImageSize    = 0;
        pMgr->rollOverOffset    = 0;
        pMgr->rollover          = false;
    }  
}

/******************************************************************************/
/*!   \fn void copyBulkToImageBfr( unsinged char *pBulk, , int size  )                                                         
 
      \brief
        This function copy the contents of the bulk transfer to the image buffer.  

      \author
          Aaron Swift
*******************************************************************************/ 
void copyBulkToImageBfr( unsigned char *pBulk, unsigned int size )
{
    if( pBulk != NULL ) 
    {  
        if( paused_ ) 
        {          
            imageBfrMgr.index = 0;
            imageBfrMgr.rollover = false;            
        }

        /* check that our index is not at the top of image buffer */
        if( imageBfrMgr.index <= imageBfrMgr.numXfersToFill ) 
        {          
            /* size is always 0, so use labelImageSize to determine when done */
            if( imageBfrMgr.labelImageSize >= 512 ) 
            {
                memcpy( &imageBuffer[ ( imageBfrMgr.index * 512 ) ], pBulk, 512 );
                imageBfrMgr.index++;
                imageBfrMgr.labelImageSize -= 512;
            } 
            else 
            {
                memcpy( &imageBuffer[ ( imageBfrMgr.index * 512 ) ], pBulk, imageBfrMgr.labelImageSize );
                imageBfrMgr.labelImageSize = 0;
                /* reset index back to the bottom */
                imageBfrMgr.index = 0;
                if( imageBfrMgr.rollover ) 
                {                    
                    imageBfrMgr.rollover = false;                   
                }
                //PRINTF("copyBulkToImageBfr(): transfer done\r\n");
            }
        } 
        else 
        {
            /* we have rolled over, clear   */
            if( imageBfrMgr.rollover ) 
            {
                imageBfrMgr.rollover = false;
                startLineTimer( true ); /* */
                /* where is the print engine when we receive the last chunks... */
                //PRINTF("copyBulkToImageBfr(): lineCounter: %d\r\n", getPrintEngineLineCntr() );
                /* reset index back to the bottom */
                imageBfrMgr.index = 0;                
                if( imageBfrMgr.labelImageSize >= 512 ) 
                {
                    memcpy( &imageBuffer[ ( imageBfrMgr.index * 512 ) ], pBulk, 512 );
                } 
                else 
                {
                    memcpy( &imageBuffer[ ( imageBfrMgr.index * 512 ) ], pBulk, imageBfrMgr.labelImageSize );
                    /* reset index back to the bottom */
                    imageBfrMgr.index = 0;
                    imageBfrMgr.rollover = false;
                    //PRINTF("copyBulkToImageBfr(): transfer done\r\n");
                }
                /* un-pause our print engine */
                paused_ = false;                
            } 
            else 
            {
                PRINTF("copyBulkToImageBfr(): error: image buffer top -- rollover not set!\r\n");
            }
        }
        if( paused_ ) 
        {
            paused_ = false; 
            //PRINTF("copyBulkToImageBfr(): continue print\r\n");
        }
    }    
}

/******************************************************************************/
/*!   \fn void clearHistory( void )                                                             
 
      \brief
        This function sets the total label image size of the image buffer 
        manager. 
        

      \author
          Aaron Swift
*******************************************************************************/ 
void setLabelImageSizeMgr( unsigned long size )
{
    imageBfrMgr.labelImageSize = size; 
    
    imageBfrMgr.index = 0;
    
    if( imageBfrMgr.labelImageSize > imageBfrMgr.bufferSize ) 
    {
        //PRINTF("labelImage is multi transfer!\r\n");
        imageBfrMgr.rollover = true;
    }
}

/******************************************************************************/
/*!   \fn void clearHistory( void )                                                             
 
      \brief
        This function clears all buffers needed to manage the print 
        head data.

      \author
          Aaron Swift
*******************************************************************************/ 
void clearHistory( PrinterEnv env )
{
    if( env == RT_SERVICE_72MM ) 
    {
        memset( &history1Line[0], 0, sizeof( PRINTER_HEAD_SIZE_72MM / sizeof(history1Line) ) );
    } 
    else if ( env == RT_SERVICE_80MM ) 
    {
        memset( &history1Line[0], 0, sizeof( PRINTER_HEAD_SIZE_80MM / sizeof(history1Line) ) );
    } 
    else 
    {
        PRINTF("clearHistory() error: unsupported head type!\r\n");
    }
}


/******************************************************************************/
/*!   \fn unsigned char *getImageBuffer( void )

      \brief
        This function returns the address of the label image buffer.
        

      \author
          Aaron Swift
*******************************************************************************/
unsigned char *getImageBuffer( void )
{
    return( &imageBuffer[0] ); 
}

/******************************************************************************/
/*!   \fn int getImageBufferSize( void )

      \brief
        This function returns the size of the label image buffer.
        

      \author
          Aaron Swift
*******************************************************************************/
int getImageBufferSize( void )
{
    return PRINTER_BUFFER_SIZE_80MM;
}

/******************************************************************************/
/*!   \fn HEADTYPE getPrintHeadType( void )

      \brief
        Returns either 72mm or 80mm headType

      \author
          Chris King
*******************************************************************************/
HEADTYPE getPrintHeadType( void )
{
    HEADTYPE headType = UNKNOWN_HEAD;
    
    if(GPIO_PinRead(GPIO1, 15U) == 1)
    {
        headType = ROHM_72MM_800_OHM;
        return headType;
    }
    else
    {
        headType = ROHM_80MM_650_OHM;
        return headType;
    }
}

/******************************************************************************/
/*!   \fn unsigned int getSltTime( HEADTYPE type, unsigned int contrast )                                                              
 
      \brief
        This function returns the current SLT time based on the contrast.
        
      \author
          Aaron Swift
*******************************************************************************/ 
unsigned int getSltTime( HEADTYPE type, unsigned int contrast )
{
    return( rohm80mmSLTTimes[ contrast ] ); 
}

/******************************************************************************/
/*!   \fn unsigned short getHalfSltTime( void )                                                              
 
      \brief
        This function returns the current half SLT time set in the print engine.
        
      \author
          Aaron Swift
*******************************************************************************/ 
unsigned short getHalfSltTime( void )
{
    PrintEngine *pEngine  = getPrintEngine();
    return pEngine->sltHalfTime;
}

/******************************************************************************/
/*!   \fn unsigned int getSltSizingTime( void )                                                            
 
      \brief
        This function returns the SLT sizing time. NOT USED - Cking
        
      \author
          Aaron Swift
*******************************************************************************/ 
unsigned int getSltSizingTime( HEADTYPE type )
{
    if( ( type == KYOCERA753_OHM ) || ( type == KYOCERA800_OHM  ) || 
        ( type == ROHM_72MM_800_OHM ) || ( type == ROHM_80MM_650_OHM ) )
        return( rohm80mmSLTTimes[ 7 ] );  /* 3 MAX_CONTRAST */
    else 
        PRINTF("getSltTime(): Unknown head type. critical error!\r\n" );
        return( 0 );  
}

/******************************************************************************/
/*!   \fn unsigned int getCompLevel( HEADTYPE type )                                                           
 
      \brief
        This function returns the compensation level based on the head type.
        
      \author
          Aaron Swift
*******************************************************************************/ 
unsigned int getCompLevel( HEADTYPE type )
{
    int compensation;
    if( ( type == KYOCERA753_OHM ) || ( type == KYOCERA800_OHM ) || 
        ( type == KYOCERA849_OHM ) || ( type == ROHM_72MM_800_OHM ) || type == ROHM_80MM_650_OHM )
      compensation = 2;
    else
      compensation = 3;
   return( compensation );  
}


/******************************************************************************/
/*!   \fn int getPrintheadTemperatureInCelsius( void )                                                           
 
      \brief
        Returns the current printhead temperature in C
        
      \author
          Chris King
*******************************************************************************/ 
int getPrintheadTemperatureInCelsius( void )
{   
    int temperatureInCelsius = 0;
    
    temperatureInCelsius = (int)( ((float)getPrintheadThermistorCounts() - 3743.4) / -29.84 ); 

    return temperatureInCelsius;
}

/******************************************************************************/
/*!   \fn unsigned char getHeadStyleType( void )                                                           
 
      \brief
        This function returns the head type
        
      \author
          Aaron Swift
*******************************************************************************/ 
unsigned char getHeadStyleType( void )
{
    return headStyle.headType;
}

/******************************************************************************/
/*!   \fn unsigned short getHeadStyleSize( void )                                                         
 
      \brief
        This function returns the head size in number of dots
        
      \author
          Aaron Swift
*******************************************************************************/ 
unsigned short getHeadStyleSize( void )
{
    if( env_ == RT_SERVICE_72MM )
    { 
        return headStyle.headSize = HEAD_DOTS_72MM;
    }
    else
    {
        return headStyle.headSize = HEAD_DOTS_80MM;
    }      
}
                        
/******************************************************************************/
/*!   \fn void setHeadTimings( void )                                                      
 
      \brief
        This function set the print head timings for the print engine.
        
      \author
          Aaron Swift
*******************************************************************************/ 
void setHeadTimings( void )
{
    PrintEngine *pEngine = getPrintEngine();
    
    if( ( pEngine->headType == ROHM_72MM_800_OHM ) || ( pEngine->headType == ROHM_80MM_650_OHM ) )       
    {
        pEngine->sltTime = rohm80mmSLTTimes[ pEngine->contrast ];
    }

    pEngine->sltHalfTime = pEngine->sltTime / 2;
  
    /* update the timing tables */    
    updatePrintTimings( );
    
    /* only allow history and current line 2loads for 5 ips printing-- ats */
    if( ( pEngine->headType == ROHM_72MM_800_OHM ) || ( pEngine->headType == ROHM_80MM_650_OHM ) ) 
    {    
        pEngine->levels = 2;      
    } 
    else 
    {
        pEngine->levels = 3;
    }
    
    int currentTemperature = getPrintheadTemperatureInCelsius();
    
    float currentLineTimeTemperatureAdjusted = 0;
 
    int difference = currentTemperature - 25; // Calculate the temperature difference from 25C

    // Calculate the number of 5-degree increments
    int increments = difference / 5;

    // Calculate the modification factor based on the number of increments
    float modificationFactor = 1 + ((float)increments * 0.0125);

    // Apply the modification factor to the original value
    currentLineTimeTemperatureAdjusted = ((float)rohm80mmCurrentLine[ pEngine->contrast ] * modificationFactor);
    
    /*
    if(currentStatus.state == ENGINE_PRINTING)
    {
        if(pEngine->steps < 400)
        {
            pEngine->histAdj[0].compType = FIRST_LEVEL_HIST;        
            pEngine->histAdj[0].time = rohm80mmHistory[ pEngine->contrast ] - 100;
            pEngine->histAdj[1].compType = CURRENT_LINE;    
            pEngine->histAdj[1].time = currentLineTimeTemperatureAdjusted + 100; 
            pEngine->pwmStartTime = rohm80mmPwmStart[ pEngine->contrast ];
            pEngine->pwmDutyCycle = ( rohm80mmPwmDuty[ pEngine->contrast ]); 
        }
    }
    else
    {
*/
        pEngine->histAdj[0].compType = FIRST_LEVEL_HIST;        
        pEngine->histAdj[0].time = rohm80mmHistory[ pEngine->contrast ];
        pEngine->histAdj[1].compType = CURRENT_LINE;    
        pEngine->histAdj[1].time = currentLineTimeTemperatureAdjusted; 
        pEngine->pwmStartTime = rohm80mmPwmStart[ pEngine->contrast ];
        pEngine->pwmDutyCycle = ( rohm80mmPwmDuty[ pEngine->contrast ]);
    //}             
}

/******************************************************************************/
/*!   \fn void updatePrintTimings( void )                                                      
 
      \brief
        This function updates the print head timings during run time (printing).
        Not sure this function is needed since the head resistance should not 
        change. 
      \author
          Aaron Swift
*******************************************************************************/ 
void updatePrintTimings( void )
{
     PrintEngine *pEngine = getPrintEngine();
    
    /* printhead type is unkown until printer idle isr has started do not process anything until then */
    if( pEngine->headType == UNKNOWN_HEAD ) 
    {
        PRINTF("UNKNOWN HEAD DETECTED IN updatePrintTimings()\r\n");
        return;
    }

    prevContrast_ = pEngine->contrast;
    
    for( int i = 0; i < MAX_TABLE_ENTRIES; i++ ) 
    { 
        history1Line[i] = rohm80mmHistory[ pEngine->contrast ];
        adjacencyTimes[i] = 0;
        currentLineTimes[i] = rohm80mmCurrentLine[ pEngine->contrast ];
        pwmTimes[i] = rohm80mmPwmStart[ pEngine->contrast ];
        pwmDutyCycle[i] =  rohm80mmPwmDuty[ pEngine->contrast ];                                
    }        
}

/******************************************************************************/
/*!   \fn void updateSLTTime( uint16_t nextSpeed)                                                      
 
      \brief
        This function updates the print head timings during run time that 
        control the line timer speed (printing).
      \author
          Chris King
*******************************************************************************/ 
void updateSLTTime( uint16_t nextSpeed )
{
    PrintEngine *pEngine = getPrintEngine();
    
    pEngine->sltTime = nextSpeed;
    
    pEngine->sltHalfTime = pEngine->sltTime / 2;
}

/******************************************************************************/
/*!   \fn unsigned short getPwmStartTime( unsigned char index )                                                       
 
      \brief
        This function returns the pwm start time based on temperature index.
        
      \author
          Aaron Swift
*******************************************************************************/ 
unsigned short getPwmStartTime( unsigned char index )
{
    if( index < MAX_TABLE_ENTRIES )
        return( pwmTimes[index] );  
    else
        return( pwmTimes[ AVERAGE_TEMP_INDEX ] );
}

/******************************************************************************/
/*!   \fn unsigned short getPwmDutyCycle( unsigned char index )                                                       
 
      \brief
        This function returns the pwm cycle time based on temperature index.
        
      \author
          Aaron Swift
*******************************************************************************/ 
unsigned short getPwmDutyCycle( unsigned char index )
{
    if( index < MAX_TABLE_ENTRIES )
        return( pwmDutyCycle[index] );    
    else
        return( pwmDutyCycle[ AVERAGE_TEMP_INDEX ] );
}

/******************************************************************************/
/*!   \fn unsigned short getHistoryTime( unsigned char index )                                                       
 
      \brief
        This function returns the history time based on temperature index.
        
      \author
          Aaron Swift
*******************************************************************************/ 
unsigned short getHistoryTime( unsigned char index )
{
    if( index < MAX_TABLE_ENTRIES )
        return( historyTimes[ index ] );
    else
        return( historyTimes[ AVERAGE_TEMP_INDEX ] );
}

/******************************************************************************/
/*!   \fn unsigned short getAdjacencyTime( unsigned char index )                                                       
 
      \brief
        This function returns the adjacency time based on temperature index.
        
      \author
          Aaron Swift
*******************************************************************************/ 
unsigned short getAdjacencyTime( unsigned char index )
{
    if( index < MAX_TABLE_ENTRIES )
        return( adjacencyTimes[index] );
    else
        return( adjacencyTimes[AVERAGE_TEMP_INDEX] );
}

/******************************************************************************/
/*!   \fn unsigned short getCurrentLineTime( unsigned char index )                                                       
 
      \brief
        This function returns the current line time based on temperature index.
        
      \author
          Aaron Swift
*******************************************************************************/ 
unsigned short getCurrentLineTime( unsigned char index )
{
    if( index < MAX_TABLE_ENTRIES )
        return( currentLineTimes[index] );
    else
        return( currentLineTimes[AVERAGE_TEMP_INDEX] );
  
}

/******************************************************************************/
/*!   \fn unsigned long byteSwapLong( unsigned long data )

      \brief
        This function converts endian format.

      \author
          Aaron Swift
*******************************************************************************/
unsigned long byteSwapLong( unsigned long data )
{
    unsigned long a, b, c, d, temp = data;

    d = data << 24;
    c = data << 8;
    c &= 0x00FF0000;
    b = data >> 8;
    b &= 0x0000FF00;
    a = data >> 24;
    temp = (d | c | b | a);

    return(temp);
}

/******************************************************************************/
/*!   \fn void setHeadPower( int state )

      \brief
        This function turn on and off power to the print head.

      \author
          Aaron Swift
*******************************************************************************/
void setHeadPower( int state )
{
    if( state == 0 && ((((currentStatus.error & HEAD_UP ) != HEAD_UP ) && ((currentStatus.sensor & OUT_OF_MEDIA) != OUT_OF_MEDIA) && ((currentStatus.sensor2 & JAMMED_LABEL) != JAMMED_LABEL)))) 
    {
        GPIO_WritePinOutput( PHEAD_POWER_EN_GPIO, PHEAD_POWER_EN_PIN, false );  
    }
    else 
    {
        GPIO_WritePinOutput( PHEAD_POWER_EN_GPIO, PHEAD_POWER_EN_PIN, true );  
    }  
}

/******************************************************************************/
/*!   \fn void history( unsigned char *pNewPrintLine )

      \brief
        This function generates first level history line by comparing the
        previous line with the current line. Dots that were previously on will 
        turned off for the history load.

      \author
          Aaron Swift
*******************************************************************************/
void history( unsigned char *pNewPrintLine )
{  
    unsigned char *pPreviousLine = NULL, *pCurrentLine = NULL;
    unsigned char temp = 0;
    
    memset( &history1Line[0], 0, sizeof(history1Line) );

    pCurrentLine = pNewPrintLine;
   
    if( *(pCurrentLine + 1) != 0 ) 
    {
        temp = 0;
    }
    
    if( pCurrentLine != NULL ) 
    {
        if( env_ == RT_SERVICE_72MM ) 
        {
          //PRINTF("\r\n72MM ENV DETECTED\r\n");
            pPreviousLine = getPreviousPrintDataLine();
            if( pPreviousLine != NULL ) 
            {
                /* first level history */ 
                for( int i = 0; i < PRINTER_HEAD_SIZE_72MM; i++ ) 
                {
                    temp = *pPreviousLine;
                    /* invert the dots */
                    temp = ~temp;            
                    /* mask the dots and apply to history buffer */
                    history1Line[i] = temp & *pCurrentLine;  
                    pPreviousLine++;
                    pCurrentLine++;
                }
            }  
        } 
        else if( env_ == RT_SERVICE_80MM ) 
        {
          //PRINTF("\r\n80MM ENV DETECTED\r\n");
            pPreviousLine = getPreviousPrintDataLine();
            if( pPreviousLine != NULL ) 
            {
                /* first level history */ 
                for( int i = 0; i < PRINTER_HEAD_SIZE_80MM; i++ ) 
                {
                    temp = *pPreviousLine;
                    /* invert the dots */
                    temp = ~temp;            
                    /* mask the dots and apply to history buffer */
                    history1Line[i] = temp & *pCurrentLine;  
                    pPreviousLine++;
                    pCurrentLine++;
                }
            }            
        }
    } 
    else 
    {
        PRINTF("history(): Print engine is NULL. critical error!\r\n" );
    }
}

/******************************************************************************/
/*!   \fn unsigned long *getCurrentPrintDataLine( void )

      \brief
        This function returns pointer to the previous printed line.

      \author
          Aaron Swift
*******************************************************************************/
unsigned char *getCurrentPrintDataLine( void )
{
    if( getPrintEngine() != NULL ) 
    {
        if( env_ ==  RT_SERVICE_72MM )
        {
            return( &imageBuffer[ ( ( getPrintEngine()->lineCounter ) *  PRINTER_HEAD_SIZE_72MM ) ] );
        }
        else if( env_ ==  RT_SERVICE_80MM ) 
        {
            return( &imageBuffer[ ( ( getPrintEngine()->lineCounter ) *  PRINTER_HEAD_SIZE_80MM ) ] );
        }  
        else
        {
            return( NULL );
        }   
    } 
    else 
    {
        return( NULL );  
    }    
}

/******************************************************************************/
/*!   \fn unsigned long *getPreviousPrintDataLine( void )

      \brief
        This function returns pointer to the previous printed line.

      \author
          Aaron Swift
*******************************************************************************/
unsigned char *getPreviousPrintDataLine( void )
{
    if( getPrintEngine() != NULL ) 
    {
        if( env_ ==  RT_SERVICE_72MM ) 
        {
            /* if this is the first line printed */
            if( getPrintEngine()->lineCounter != 0 ) 
            {
                return( &imageBuffer[ ( ( getPrintEngine()->lineCounter - 1 ) * PRINTER_HEAD_SIZE_72MM ) ] );
            } 
            else 
            {
                return( &imageBuffer[ 0 ] );    
            }
        } 
        else if( env_ ==  RT_SERVICE_80MM ) 
        {
            /* if this is the first line printed */
            if( getPrintEngine()->lineCounter != 0 ) 
            {
                return( &imageBuffer[ ( ( getPrintEngine()->lineCounter - 1 ) * PRINTER_HEAD_SIZE_80MM ) ] );
            } 
            else 
            {
                return( &imageBuffer[ 0 ] );    
            }
        } 
        else 
        {
            return( NULL );  
        }
    } 
    else 
    {
        return( NULL );  
    }    
}

/******************************************************************************/
/*!   \fn unsigned long *getNextPrintDataLine( void )

      \brief
        This function returns pointer to the next line to print.

      \author
          Aaron Swift
*******************************************************************************/
unsigned char *getNextPrintDataLine( void )
{
    if( getPrintEngine() != NULL ) 
    {  
        if( env_ == RT_SERVICE_72MM ) 
        {
            return( &imageBuffer[ ( getPrintEngine()->lineCounter + 1 ) * PRINTER_HEAD_SIZE_72MM  ] );
        } 
        else if ( env_ == RT_SERVICE_80MM ) 
        {
            return( &imageBuffer[ ( getPrintEngine()->lineCounter + 1 ) * PRINTER_HEAD_SIZE_80MM  ] );
        }
        else 
        {
            return( NULL );    
        }
    } 
    else 
    {
        return( NULL );  
    } 
}

/******************************************************************************/
/*!   \fn void initializeSelfCheck( void )

      \brief
        This function intialize the print buffer for the start of the print head
        dot check(ValueMax). Each line in the print buffer will have one 
        dot set.

      \author
          Aaron Swift
*******************************************************************************/
void initializeSelfCheck( void )
{
    if( headStyle.headSize == HEAD_DOTS_72MM ) 
    {
        unsigned char printLine[ PRINTER_HEAD_SIZE_72MM + 1 ] = { 0 };
        unsigned char *pDot = &printLine[0];
        //unsigned char dot = 1;
        unsigned char dot = 0x80;
        *pDot = dot;
        int i = 0;
        bool firstRun = false;
        memcpy( &imageBuffer[(i) * PRINTER_HEAD_SIZE_72MM], &printLine[0], PRINTER_HEAD_SIZE_PREPACK );//added to print the first dot twice
        while( i <= HEAD_DOTS_72MM +8  ) { // <= because we want to print dot 1 //runs 57 times 

            /* copy the print line into the image buffer */
            memcpy( &imageBuffer[(i + 1) * PRINTER_HEAD_SIZE_72MM], &printLine[0], PRINTER_HEAD_SIZE_72MM ); 
            
            /* rotate the dot through the byte and copy to each position in the image buffer */            
            for( int x = 1; x <= 8; x++ ) 
            {
                //dot <<= 1;
                dot >>= 1;
                *pDot = dot;
                memcpy( &imageBuffer[ ( i + 1 + x ) * PRINTER_HEAD_SIZE_72MM ], &printLine[0], PRINTER_HEAD_SIZE_72MM );
                //memcpy( &imageBuffer[ ((x  * PRINTER_HEAD_SIZE ) * i) ], &printLine[0], PRINTER_HEAD_SIZE );
            }
            /* clear the print line */
            memset( &printLine[0], 0, PRINTER_HEAD_SIZE_72MM);                
            /* next byte */
            pDot++;
            /* reset dot */
            //dot = 1;
            dot = 0x80;
            /* copy dot to the new position */
            *pDot = dot;
             i += 8;
             //i -= 8;
             if(firstRun)
             {
                firstRun =false;
                pDot = &printLine[0];
                dot = 0x80;
                *pDot = dot;
                i = 0;
             }
        }
    } 
    else 
    {
        unsigned char printLine[PRINTER_HEAD_SIZE_PREPACK] = { 0 };
        unsigned char *pDot = &printLine[0];
        unsigned char dot = 1;
          
        *pDot = dot;
        int i = 0;
        /* rotate a dot through the print line buffer and add each 
           dot to the image buffer */
        while( i < HEAD_DOTS_80MM ) 
        {
            /* copy the print line into the image buffer */
            memcpy( &imageBuffer[i * PRINTER_HEAD_SIZE_PREPACK], &printLine[0], PRINTER_HEAD_SIZE_PREPACK ); 
            
            /* rotate the dot through the byte and copy to each position in the image buffer */            
            for( int x = 1; x < 8; x++ ) 
            {
                dot <<= 1;
                *pDot = dot;
                memcpy( &imageBuffer[ ( i + x ) * PRINTER_HEAD_SIZE_PREPACK ], &printLine[0], PRINTER_HEAD_SIZE_PREPACK ); 
            }
            /* clear the print line */
            memset( &printLine[0], 0, PRINTER_HEAD_SIZE_PREPACK);                
            /* next byte */
            pDot++;
            /* reset dot */
            dot = 1;
            /* copy dot to the new position */
            *pDot = dot;
             i += 8;
        }
    }   
}

/******************************************************************************/
/*!   \fn void resetPrintDataLine( void )

      \brief
        This function resets the index into the label image buffer.

      \author
          Aaron Swift
*******************************************************************************/
void resetPrintDataLine( void )
{
    if( getPrintEngine() != NULL ) 
    {  
        unsigned char *pBff = getImageBuffer();
        if( pBff != 0 ) {           
            if( env_ == RT_SERVICE_72MM )
            {
                memset( pBff, 0, PRINTER_HEAD_SIZE_72MM );
            }
            else if( env_ == RT_SERVICE_80MM )
            {
                memset( pBff, 0, PRINTER_HEAD_SIZE_80MM );
            }
            else 
            {
              PRINTF("resetPrintDataLine() error: unsupported head!\r\n" ); 
            } 
            
            getPrintEngine()->lineCounter = 0;    
        }
    } 
    else 
    {
        PRINTF("resetPrintDataLine(): Print engine NULL. critical error!\r\n" );
    }
}

unsigned char *getFirstHistoryLine( void )
{
    return (unsigned char *)&history1Line[0];
}

