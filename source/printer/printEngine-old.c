#include <stdlib.h>
#include <limits.h>
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_lpspi.h"
#include "fsl_lpspi_edma.h"
#include "fsl_dmamux.h"
#include "fsl_clock.h"
#include "fsl_gpt.h"
#include "fsl_pwm.h"
#include "pin_mux.h"
#include "sensors.h"
#include "label.h"
#include "dvr8818.h"
#include "serialFlash.h"
#include "w25x10cl.h"
#include "systemTimer.h"
#include "printEngine.h"
#include "printhead.h"
#include "commandTable.h"
#include "drawingPrimitives.h"
#include "threadManager.h"
#include "queueManager.h"
#include "prMessages.h"
#include "semphr.h"
#include "averyPrinter.h"
#include "dotWearTask.h"
#include "commandTable.h"
#include "averyCutter.h"
#include "fsl_debug_console.h"

PrintEngine         engine;
bool sample_   = false;
bool historyEnabled_ = true;

//unsigned char histAdjLines[MAX_HIST_ADJ_LEVELS][PRINTER_HEAD_SIZE];

static bool stepCounterEnabled_         = false;
static bool skipMissingLabel_           = false;

//#pragma default_variable_attributes = @ "DEFAULT_RAM0"

/* buffers for test labels */
#if 0
static unsigned char pattern1[(PRINTER_HEAD_SIZE * 2 )];
static unsigned char pattern2[(PRINTER_HEAD_SIZE * 2 )];
#else
static unsigned char pattern1[( 72 * 2 )] = {0};
static unsigned char pattern2[( 72 * 2 )];
#endif
#if 0/* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
lpspi_master_edma_handle_t spiHeadMasterHandle;
#else 
lpspi_master_handle_t spiHeadMasterHandle; 
#endif


LabelPosition   label0;
LabelPosition   label1;
LabelPosition   label2;
LabelPosition   *pCurrentLabel;

#pragma default_variable_attributes =

static SemaphoreHandle_t pCutSemaphore  = NULL;
static bool applyVerticalOffset_        = false;
static short prevVertOffset_            = 0;

/* command queue */
static QueueHandle_t pCmdQHandler_      = NULL;

/* line timer interrupt level for print operation and dotwear */
static unsigned int interruptLevel_ = 2; 

bool continuousStock_                   = false;
#if 0 /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
void LPSPI_MasterUserCallback( LPSPI_Type *base, lpspi_master_edma_handle_t *handle, status_t status, void *userData );
#else 
void LPSPI_MasterUserCallback( LPSPI_Type *base, lpspi_master_handle_t *handle, status_t status, void *userData );
#endif

volatile bool spiDmaTransferComplete = false;
edma_handle_t eDmaHandle_0, eDmaHandle_1, eDmaHandle_2;

/* printer status tracking */
PrStatusInfo currentStatus, prevStatus;

extern void setOperation( unsigned char operation, PrStatusInfo *pStatus );
extern void initializeCmdSequence( CMDId id, PrStatusInfo *pStatus);
extern void setNextOperation( PrStatusInfo *pStatus );
extern void skipNextOperation( PrStatusInfo *pStatus ); 
extern void jumpToOperation( PrStatusInfo *pStatus, unsigned char index );
extern int getGhostMCntr( void );
extern void clearGhostMCntr( void );
extern void setTriggerMotorStop( void );
extern void clearTriggerMotorStop( void );
extern void sendPrStatus( PrStatusInfo *pStatus, bool interrupt );

#if 0 /* To Do */
extern HeadSensors  headSensors;
#endif
extern Pr_Config config_;
extern SemaphoreHandle_t pCutDoneSemaphore;
extern volatile ADCManager adcManager;



int             labelAlignment;

bool bank0_ = false, bank1_ = false;

/* added for sizing tests remove when finished -- freestanding scale 
unsigned char *pMediaSizing = NULL;
*/

void delay( unsigned long time );
extern void testPrintHeadTransfer( void );
/******************************************************************************/
/*!   \fn void initializePrintEngine( unsigned int contrast, unsigned int mediaCount, 
                                      QueueHandle_t pHandle )

      \brief
        This function initializes the print engine.
        

      \author
          Aaron Swift
*******************************************************************************/
void initializePrintEngine( unsigned int contrast, unsigned int mediaCount, 
                            QueueHandle_t pHandle )
{
    if( pHandle != NULL ) {
        /* sizing debug - freestanding scale 
        pMediaSizing = pvPortMalloc( 1000 ); */
 
        /* assign our command queue */
        pCmdQHandler_ = pHandle;
        /* get the head temperature index */
        unsigned char tempIndex = getHeadTemp();

        pCutSemaphore = (SemaphoreHandle_t)getCutSemaphore();
                
        engine.headType = getPrintHeadType();
        /* get the line compensation levels */
        engine.levels = getCompLevel( engine.headType );
        /* get the over all line burn time */
        engine.sltTime = getSltTime( engine.headType, contrast );
        
        engine.contrast = config_.contrast_adjustment;
        engine.sltHalfTime = engine.sltTime / 2;
        engine.lineCounter = 0;
        engine.burnSequence = 0; 

        engine.numSteps = 0;
        engine.numPrintLines = 0;
        engine.outOfMediaCnt = 0;
        engine.stepsOffset = 0;
        engine.labelTracking = 0;
        engine.totalLinesToPrint = 0;
        engine.maxMediaCount = 200;
        engine.direction = FORWARD_;
        engine.labelOrientation = HEEL_FIRST;
        engine.pHistory = getFirstHistoryLine();
        engine.pImage        = getImageBuffer();
        
        labelAlignment      = 0;
        /* keep track of three label positions */
        label0.position     = 0;
        label0.next         = &label1;
        label1.position     = 0;
        label1.next         = &label2;
        label2.position     = 0;
        label2.next         = &label0;
                
        pCurrentLabel = &label0;
        
        applyVerticalOffset_ = false;
        prevVertOffset_ = 0;
        
        /* intialize stepper motor */
        initializeStepper( engine.direction );
        // gets rid of whine before running the stepper
        powerOffStepper();
        /* set the operational directive to idle operation */
        engine.currentCmd.generic.directive =  IDLE_DIRECTIVE;
        
        /* setup the start times for history, adjacency and current line */
        if( engine.levels == 3 ) {
            engine.histAdj[0].compType = FIRST_LEVEL_HIST;
            engine.histAdj[0].time = getHistoryTime( contrast );
            engine.histAdj[1].compType = FIRST_LEVEL_ADJ;
            engine.histAdj[1].time = getAdjacencyTime( contrast );
            engine.histAdj[2].compType = CURRENT_LINE;
            engine.histAdj[2].time = getCurrentLineTime( contrast );

            engine.pwmStartTime = getPwmStartTime( contrast );
            engine.pwmDutyCycle = getPwmDutyCycle( contrast );
        } else {
            engine.histAdj[0].compType = FIRST_LEVEL_HIST;
            engine.histAdj[0].time = getHistoryTime( contrast );
            engine.histAdj[1].compType = CURRENT_LINE;
            engine.histAdj[1].time = getCurrentLineTime( contrast );

            engine.pwmStartTime = getPwmStartTime( contrast );
            engine.pwmDutyCycle = getPwmDutyCycle( contrast );
        }
        
        
        /* initialize print engine timer */
        startLineTimer( false );
        initializePrintEngineTimer( DEFAULT_ENGINE_COUNT );    
    } else {
        PRINTF("initializePrintEngine(): PR Command Queue NULL! \r\n");      
    }    
}

/******************************************************************************/
/*!   \fn void initializePrintHeadSPI( void )

      \brief
        This function initializes the SPI peripheral and edma used for data 
        transfer to the print head.
        Note: 4" of label data can be transfered to the print head in 300mS

      \author
          Aaron Swift
*******************************************************************************/
void initializePrintHeadSPI( void )
{
    lpspi_master_config_t masterConfig;
    
    #if 1 /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
    PrintEngine *pEngine  = getPrintEngine();
    if( pEngine != NULL ) {            
        masterConfig.baudRate                      = 12000000u; /* 10Mhz */
        masterConfig.bitsPerFrame                  = 8;      
        masterConfig.cpol                          = kLPSPI_ClockPolarityActiveHigh;        
        masterConfig.cpha                          = kLPSPI_ClockPhaseFirstEdge;

        masterConfig.direction                     = kLPSPI_MsbFirst;
        masterConfig.pcsToSckDelayInNanoSec        = 50;
        masterConfig.lastSckToPcsDelayInNanoSec    = 50;
        masterConfig.betweenTransferDelayInNanoSec = 50;
        masterConfig.whichPcs                                 = kLPSPI_Pcs0;
        masterConfig.pcsActiveHighOrLow                       = kLPSPI_PcsActiveLow;
        masterConfig.pinCfg                                   = kLPSPI_SdiInSdoOut;    //kLPSPI_SdoInSdiOut//kLPSPI_SdiInSdoOut; -- switch on schematic!!
        masterConfig.dataOutConfig                            = kLpspiDataOutTristate;

        /* intialize the spi interface */
        LPSPI_MasterInit( LPSPI4, &masterConfig, ( CLOCK_GetFreq(kCLOCK_Usb1PllPfd0Clk) / 8 ) );
        NVIC_SetPriority( LPSPI4_IRQn, 1 );
        LPSPI_MasterTransferCreateHandle( LPSPI4, &spiHeadMasterHandle, LPSPI_MasterUserCallback, NULL );                
    } else {
        PRINTF("initializePrintHeadSPI(): printEngine is NULL. critical error!\r\n" );
    }
    #else  
    
    edma_config_t edmaConfig;
  
    lpspi_transfer_t masterXfer;
    
    LPSPI_MasterGetDefaultConfig( &masterConfig );
    
    /* intialize the dma mux */
    DMAMUX_Init( DMAMUX );
    /* setup the dma channel 0 for source 79: lpspi4 rx this channel will not be used but needs setup! */
    DMAMUX_SetSource( DMAMUX, 0, kDmaRequestMuxLPSPI4Rx );
    DMAMUX_EnableChannel( DMAMUX, 0 );

    /* setup the dma channel 1 for source 80: lpspi4 tx  */
    DMAMUX_SetSource( DMAMUX, 1, kDmaRequestMuxLPSPI4Tx );
    DMAMUX_EnableChannel( DMAMUX, 1 );
    
    /* edma already intialized in sensors.c  */
    EDMA_GetDefaultConfig( &edmaConfig );   
    EDMA_Init( DMA0, &edmaConfig );
    
    PrintEngine *pEngine  = getPrintEngine();
    if( pEngine != NULL ) {        
        /* setup spi4 master configuration kjt-56 max clock rate 12Mhz */     
        masterConfig.baudRate                      = 12000000u; /* 10Mhz */
        masterConfig.bitsPerFrame                  = 8;      //8  
        masterConfig.cpol                          = kLPSPI_ClockPolarityActiveHigh;        
        masterConfig.cpha                          = kLPSPI_ClockPhaseFirstEdge;
        if( pEngine->labelOrientation ==  HEAD_FIRST ) {
            masterConfig.direction                 = kLPSPI_LsbFirst; 
        } else {
            masterConfig.direction                 = kLPSPI_MsbFirst; 
        }
        masterConfig.pcsToSckDelayInNanoSec        = 50;
        masterConfig.lastSckToPcsDelayInNanoSec    = 50;
        masterConfig.betweenTransferDelayInNanoSec = 50;
        masterConfig.whichPcs                                 = kLPSPI_Pcs0;
        masterConfig.pcsActiveHighOrLow                       = kLPSPI_PcsActiveLow;
        masterConfig.pinCfg                                   = kLPSPI_SdiInSdoOut;    //kLPSPI_SdoInSdiOut//kLPSPI_SdiInSdoOut; -- switch on schematic!!
        masterConfig.dataOutConfig                            = kLpspiDataOutTristate;

        /* intialize the spi interface */
        LPSPI_MasterInit( LPSPI4, &masterConfig, ( CLOCK_GetFreq(kCLOCK_Usb1PllPfd0Clk) / 8 ) );
        NVIC_SetPriority( LPSPI4_IRQn, 1 );
          
        memset( &eDmaHandle_0, 0, sizeof(edma_handle_t) );
        memset( &eDmaHandle_1, 0, sizeof(edma_handle_t) );
        
        /* create handles for all three dma channels */ 
        EDMA_CreateHandle( &eDmaHandle_0, DMA0, 0 ); 
        EDMA_CreateHandle( &eDmaHandle_1, DMA0, 1 );
      
        /* inialize the master handle with the callback */ 
        LPSPI_MasterTransferCreateHandleEDMA( LPSPI4, &spiHeadMasterHandle, LPSPI_MasterUserCallback, 
                                             NULL, &eDmaHandle_0, &eDmaHandle_1 );        
    } else {
        PRINTF("initializePrintHeadSPI(): printEngine is NULL. critical error!\r\n" );
    }
    #endif    
    spiDmaTransferComplete = false;
}

/******************************************************************************/
/*!   \fn void DSPI_MasterUserCallback( SPI_Type *base, 
                                        lpspi_master_handle_t *handle, 
                                        status_t status, void *userData )
      \brief
        This function is called after one complete dma transfer has occured.
       
      \author
          Aaron Swift
*******************************************************************************/
#if 0 /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
void LPSPI_MasterUserCallback( LPSPI_Type *base, lpspi_master_edma_handle_t *handle, status_t status, void *userData )
#else 
void LPSPI_MasterUserCallback( LPSPI_Type *base, lpspi_master_handle_t *handle, status_t status, void *userData )
#endif
{
    /* added to time history */
    GPIO_WritePinOutput( ACCEL_INTB_GPIO, ACCEL_INTB_PIN, true );

    /* assert data latch only on history loads */
    if( !isCurrentLine() ) {
        /* data is transfered to the head, assert the data latch */ 
        GPIO_WritePinOutput( PHEAD_LATCH_GPIO, PHEAD_LATCH_PIN, false );         //false
    }
 
    if( status == kStatus_Success ) {
        spiDmaTransferComplete = true;
    } else {
        PRINTF("DSPI_MasterUserCallback(): Dma transfer failed. critical error!\r\n" );
    }
}


/******************************************************************************/
/*!   \fn void addCmdToQueue( PrCommand *pCmd )

      \brief
        This function adds printer commands to the command queue
        

      \author
          Aaron Swift
*******************************************************************************/
void addCmdToQueue( PrCommand *pCmd )
{
    /* add command message to the command queue */
    BaseType_t result = xQueueSendToBack( pCmdQHandler_, (void *)pCmd, 0 );
    if( result != pdTRUE ) {
        PRINTF("handlePrinterMsg(): PR Command Queue full! \r\n");  
    }               
}


/******************************************************************************/
/*!   \fn void startPrintEngine( void )

      \brief
        This function resets the print engine counters, sets the line 
        history / adjacency for the line and starts the DMA transfer to 
        the print head.    
        
      \author
          Aaron Swift
*******************************************************************************/
void startPrintEngine( void )
{
    
    /* reset engine counters */
    engine.lineCounter = 0;
    engine.burnSequence = 0;
    engine.linePrintDone = false;
    
    /* reset the power supply 
    checkForOverCurrent( true );*/
    
    /* get the line to burn */
    historyAdjacency();  
      
    /* turn on print head power */
    setHeadPower( 1 ); 
    
    /* start dma transfer to the printhead */
    //loadPrintLine();
    
    /* start the line timer to burn the line. */
    startLineTimer( true );
}


/******************************************************************************/
/*!   \fn void setLineTimerIntLevel( unsigned int level )

      \brief
        This function sets the line timer interrupt level.
        Normal printing operations should run at a level of 2 and dot wear 
        should execute with a level of 5. If dotwear runs at level 2 a timing 
        issue occurs with sampling the dot within the current bubble.

      \author
          Aaron Swift
*******************************************************************************/
void setLineTimerIntLevel( unsigned int level )
{
    interruptLevel_ = level;  
    NVIC_SetPriority( GPT2_IRQn, interruptLevel_ );
    EnableIRQ( GPT2_IRQn );
}

/******************************************************************************/
/*!   \fn void startLineTimer(  bool start  )

      \brief
        This function initializes and starts the line print timer if the start
        flag is true.

        Timer period    .512uSec
        Timer Freq      1.953Mhz   
      \author
          Aaron Swift
*******************************************************************************/
void startLineTimer(  bool start  )
{

    /* initialize line printer timer with 3 compare interrupts */    
    gpt_config_t gptConfig;
    
    /* obtain default configuration for general purpose timer module */
    GPT_GetDefaultConfig( &gptConfig );
   
    gptConfig.enableRunInWait = false; 
    gptConfig.enableRunInStop = false;
    gptConfig.enableRunInDoze = false;  
    gptConfig.enableRunInDbg = false;   
    gptConfig.enableFreeRun = true;

    /* initialize gpt module */
    GPT_Init( GPT2, &gptConfig );    
    GPT_SetClockDivider( GPT2, 32 );    

    /* get gpt clock frequency */
    uint32_t freq = CLOCK_GetFreq( kCLOCK_PerClk );
    /* gpt frequency is 1.953Mhz = .512uSec */
    freq =  freq / 32;
    
    /* load the compare registers with their initial values and setup the output 
       compare mode to toggle output on a match*/

    /* set channel 1 for slt time */
    GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel1, engine.histAdj[0].time ); 
    /* set channel 2 for history time */
    GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel2,  engine.histAdj[1].time ); 
    /* set channel 3 for current line time */
    GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel3, engine.sltHalfTime ); 
    
    GPT_SetOutputOperationMode( GPT2, kGPT_OutputCompare_Channel1, kGPT_OutputOperation_Disconnected );
    GPT_SetOutputOperationMode( GPT2, kGPT_OutputCompare_Channel2, kGPT_OutputOperation_Disconnected );
    GPT_SetOutputOperationMode( GPT2, kGPT_OutputCompare_Channel3, kGPT_OutputOperation_Disconnected );
    
    /* enable channel interrupt flags. */
    GPT_EnableInterrupts( GPT2, kGPT_OutputCompare1InterruptEnable );   /* history time channel */
    GPT_EnableInterrupts( GPT2, kGPT_OutputCompare2InterruptEnable );   /* current line channel */  
    GPT_EnableInterrupts( GPT2, kGPT_OutputCompare3InterruptEnable );   /* line burn time channel */       
    
    setLineTimerIntLevel( LINE_PRINTER_TIMER_PRIORITY );
    
   
    if( start ) {
        /* start the timer */
        GPT_StartTimer( GPT2 );
    }
}

/******************************************************************************/
/*!   \fn void stopLineTimer( void )                                                             
 
      \brief
        This function disables all line timer interrupts and stops the print
        timer.
       
      \author
          Aaron Swift
*******************************************************************************/ 
void stopLineTimer( void )
{
    /* stop the timer */
    GPT_StopTimer( GPT2 );
    /* reset our count */
    GPT_SoftwareReset( GPT2 );

    /* disable channel interrupt flags. */    
    GPT_DisableInterrupts(LINE_PRINTER_TIMER_BASE, kGPT_OutputCompare1InterruptEnable);         /* history time channel */ 
    GPT_DisableInterrupts( LINE_PRINTER_TIMER_BASE, kGPT_OutputCompare2InterruptEnable );       /* current line channel */
    GPT_DisableInterrupts(LINE_PRINTER_TIMER_BASE, kGPT_OutputCompare3InterruptEnable);         /* line burn time channel */
    
    /* clear any flags that might be set */
    GPT_ClearStatusFlags( LINE_PRINTER_TIMER_BASE, kGPT_OutputCompare1Flag );
    GPT_ClearStatusFlags( LINE_PRINTER_TIMER_BASE, kGPT_OutputCompare2Flag );
    GPT_ClearStatusFlags( LINE_PRINTER_TIMER_BASE, kGPT_OutputCompare3Flag );

}
  
/******************************************************************************/
/*!   \fn static void lineTimerSLT( void )

      \brief
        This interrupt expires at the SLT time with a rate of .666uSec.
        Turn off PWM of the stobe and reinitialize the strobe line as gpio.
        Prep the next line data by determining history and adjacency and loading
        the next line to the print head.

      \author
          Aaron Swift
*******************************************************************************/
void lineTimerSLT( void ) 
{    
    /* deinitialize strobe pwm channel, we aren't currently PWM'ing */
    //PWM_Deinit( PWM2, kPWM_Module_0 );
    
    /*  re-enable peripheral io control of the strobe pin */
    gpio_pin_config_t strobeConfig = { kGPIO_DigitalOutput, 1 };
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_38_GPIO3_IO06, 0x70A0U );
    /* release data strobe */
    GPIO_PinInit( PHEAD_STROBE_A_GPIO, PHEAD_STROBE_A_PIN, &strobeConfig );
    GPIO_PinInit( PHEAD_STROBE_B_GPIO, PHEAD_STROBE_B_PIN, &strobeConfig );
    /* release strobe enable */
    GPIO_WritePinOutput( PHEAD_STROBE_EN_GPIO, PHEAD_STROBE_EN_PIN, true ); 
    
    engine.burnSequence = 0; // reset burn sequence
    
    /* are we done with the image? */
    if( --engine.numPrintLines <= 0 )   {
        engine.linePrintDone = true;
        if(getDotWearHandle() != NULL && readyToNotify() )
            xTaskNotifyFromISR( getDotWearHandle(), DOT_DONE,eSetBits, pdFALSE );
        stopLineTimer();        
    } else {
        /* stop our line timer */
        stopLineTimer();                      
        /* setup history and adjacency for the next line */
        historyAdjacency();
        loadHistory();
        
        /* restart our timer */
        startLineTimer( true ); 
        
        engine.steps++;
        halfStepMotor();
        motorStepFast( &currentStatus );
        
        /* removed for debug, add back when sensors are fixed!        
        readMediaSensor ( &currentStatus );        
        if( currentStatus.error != NULL ) {                            
            stopLineTimer(); 
        } */
    }
}

/******************************************************************************/
/*!   \fn void loadHistory( void )

      \brief
        This function configures and starts the DMA loading of the history line
        to the print head.

      \author
          Aaron Swift
*******************************************************************************/
void loadHistory( void ) 
{
    if( historyEnabled_ ){
        lpspi_transfer_t masterXfer;  
         
        clearBurnSequence();
        
        /* setup master transfer */
        masterXfer.txData = (unsigned char *)engine.pHistory;
        masterXfer.rxData = NULL;
        masterXfer.dataSize = PRINTER_HEAD_SIZE;
        masterXfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous;
        
        #if 0   /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
        unsigned long result  = LPSPI_MasterTransferEDMA( LPSPI4, &spiHeadMasterHandle, &masterXfer );
        #else
        unsigned long result = LPSPI_MasterTransferNonBlocking( LPSPI4, &spiHeadMasterHandle, &masterXfer);
        #endif
        
        if( kStatus_Success != result ) {
            PRINTF( "loadHistory(): Print head load error! %d\r\n", result ); 
            /* stop engine timer and fix problem */
            stopLineTimer();
            
            #if 0  /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
            /* abort the current transfer and retry */
            LPSPI_MasterTransferAbortEDMA( LPSPI4, &spiHeadMasterHandle );
            #else
            LPSPI_MasterTransferAbort( LPSPI4, &spiHeadMasterHandle );
            #endif            
            
            /* try re-init of SPI/DMA */
            initializePrintHeadSPI();
            #if 0  /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
            result  = LPSPI_MasterTransferEDMA( LPSPI4, &spiHeadMasterHandle, &masterXfer );
            #else 
            result = LPSPI_MasterTransferNonBlocking( LPSPI4, &spiHeadMasterHandle, &masterXfer);
            #endif            
            if( kStatus_Success != result ) {
                PRINTF( "loadHistory(): Nope that doesn't work! %d\r\n", result );    
            }
            /* restart our timer */
            startLineTimer( true );
        }  
    }
}


/******************************************************************************/
/*!   \fn void loadPrintLine( void )

      \brief
        This function configures and starts the DMA loading of the next line
        to the print head.

      \author
          Aaron Swift
*******************************************************************************/
void loadPrintLine( void ) 
{

    lpspi_transfer_t masterXfer;  
    
    unsigned long offset = 0;
    
    /* do not allow the print engine outside the image buffer */
    if( engine.lineCounter >= N_PRINTER_LINES ) {
        /* keep track of total lines in case image greater than 5" */
        engine.lineCounter2 = engine.lineCounter; 
        engine.lineCounter = 0;
        /* label image is greater than 5" */
        offset = engine.lineCounter;
    } else {
        offset = ( engine.lineCounter * PRINTER_HEAD_SIZE );
    }
    
    /* added to debug data out 
    unsigned char bfr[56] = { 0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,
                            0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,
                            0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55,0xaa,0x55}; 
    */
    
    /* setup master transfer */
    masterXfer.txData = (unsigned char *)engine.pImage + offset; 
    masterXfer.rxData = NULL;
    masterXfer.dataSize = 72;   PRINTER_HEAD_SIZE;
    masterXfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous;


    #if 0  /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
    unsigned long result  = LPSPI_MasterTransferEDMA( LPSPI4, &spiHeadMasterHandle, &masterXfer );    
    #else
    unsigned long result = LPSPI_MasterTransferNonBlocking( LPSPI4, &spiHeadMasterHandle, &masterXfer);
    #endif
        
    if( kStatus_Success != result ) {
        PRINTF( "loadPrintLine(): Print head load error! %d\r\n", result );
        /* stop engine timer and fix problem */
        stopLineTimer();
        
        #if 0  /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
        /* abort the current transfer and retry */
        LPSPI_MasterTransferAbortEDMA( LPSPI4, &spiHeadMasterHandle );
        #else
        LPSPI_MasterTransferAbort( LPSPI4, &spiHeadMasterHandle );
        #endif        
        /* try re-init of SPI/DMA */
        initializePrintHeadSPI();
        
        #if 0  /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
        result  = LPSPI_MasterTransferEDMA( LPSPI4, &spiHeadMasterHandle, &masterXfer );
        #else
        result = LPSPI_MasterTransferNonBlocking( LPSPI4, &spiHeadMasterHandle, &masterXfer);
        #endif
        
        if( kStatus_Success != result ) {
            PRINTF( "loadPrintLine(): Print head load error! %d\r\n", result );      
        }
        /* restart our timer */
        startLineTimer( true );
    }
   
    /* NOTE: printhead latch is negated in the SPI-EDMA callback */
    engine.lineCounter++;   
}

/******************************************************************************/
/*!   \fn void loadZeroPrintLine( void )

      \brief
        This function configures and starts the DMA loading of a blank line into
        the printhead

      \author
          Aaron Swift
*******************************************************************************/
void loadZeroPrintLine( void ) 
{
    lpspi_transfer_t masterXfer;  
    
    unsigned long offset = 0;
       
    /* setup master transfer */
    masterXfer.txData = (unsigned char *)pattern1;
    masterXfer.rxData = NULL;
    masterXfer.dataSize = PRINTER_HEAD_SIZE;
    masterXfer.configFlags =  kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous;
    
    #if 0  /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
    unsigned long result  = LPSPI_MasterTransferEDMA( LPSPI4, &spiHeadMasterHandle, &masterXfer );
    #else
    unsigned long result = LPSPI_MasterTransferNonBlocking( LPSPI4, &spiHeadMasterHandle, &masterXfer);
    #endif
    
    if( kStatus_Success != result ) {
        PRINTF( "loadZeroPrintLine(): Print head load error! %d\r\n", result );
        /* stop engine timer and fix problem */
        stopLineTimer();
        
        #if 0  /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */        
        /* abort the current transfer and retry */
        LPSPI_MasterTransferAbortEDMA( LPSPI4, &spiHeadMasterHandle );
        #else
        LPSPI_MasterTransferAbort( LPSPI4, &spiHeadMasterHandle );
        #endif        
        
        /* try re-init of SPI/DMA */
        initializePrintHeadSPI();
        #if 0  /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
        result  = LPSPI_MasterTransferEDMA( LPSPI4, &spiHeadMasterHandle, &masterXfer );
        #else
        result = LPSPI_MasterTransferNonBlocking( LPSPI4, &spiHeadMasterHandle, &masterXfer);
        #endif
        
        if( kStatus_Success != result ) {
            PRINTF( "loadZeroPrintLine(): Print head load error! %d\r\n", result );      
        }
        /* restart our timer */
        startLineTimer( true );                     
    }
    
    delay(150);
    GPIO_WritePinOutput( PHEAD_LATCH_GPIO, PHEAD_LATCH_PIN, true );     
}

bool isCurrentLine( void )
{
    bool result = false;
    if( engine.burnSequence > 0 ) {
        result = true;
    }
    return result;
}

void clearBurnSequence( void ) 
{
    engine.burnSequence = 0;   
}

void clearPrevVertOffset( void )
{
    prevVertOffset_ = 0;    
}

/******************************************************************************/
/*!   \fn void lineTimerStrobe( void )

      \brief
        This function sets up and start pwm of the print head strobe signal.
        PWM of the strobe line regulates the heat to each dot in the print head 
        until the slt time has expired.

      \author
          Aaron Swift
*******************************************************************************/
void lineTimerStrobe( void )
{
    /* added for demo */
    return;
    
    pwm_config_t pwmConfig;
    pwm_signal_param_t pwmSignal;

    pwmSignal.pwmChannel       = kPWM_PwmA;
    pwmSignal.level            = kPWM_HighTrue;
    /* strobe signal is active low! pulse width low is measured 
    pwmSignal.dutyCyclePercent = 100 - engine.pwmDutyCycle;*/
    pwmSignal.dutyCyclePercent = 100 - 15;
    pwmSignal.deadtimeValue    = 0;
    pwmSignal.faultState       = kPWM_PwmFaultState0;

    
    /* disable GPIO control of the pin select ftm0 pwm control for printhead strobe */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_38_FLEXPWM2_PWMA00, 0 );
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_38_FLEXPWM2_PWMA00, 0x10B0U ); 

    PWM_GetDefaultConfig( &pwmConfig );
    pwmConfig.prescale = kPWM_Prescale_Divide_1;
    /* Use full cycle reload */
    pwmConfig.reloadLogic = kPWM_ReloadPwmFullCycle;
    /* PWM A & PWM B operate as 2 independent channels */
    pwmConfig.pairOperation   = kPWM_Independent;
    pwmConfig.enableDebugMode = true;
    
    PWM_Init( PWM2, kPWM_Module_0, &pwmConfig);     
    PWM_SetupPwm( PWM2, kPWM_Module_0, &pwmSignal, 1U, kPWM_SignedCenterAligned, 25265U , CLOCK_GetFreq( kCLOCK_IpgClk ) );
    
    PWM_SetPwmLdok( PWM2, kPWM_Control_Module_0, true );
    PWM2->SM[kPWM_Module_0].DISMAP[kPWM_Module_0] = 0x00;
    
    PWM_StartTimer( PWM2, kPWM_Control_Module_0 );
}

/******************************************************************************/
/*!   \fn static void lineTimerBurn( void )

      \brief
        This function


      \author
          Aaron Swift
*******************************************************************************/
void lineTimerBurn( void )
{    
    if( engine.numPrintLines != 0 ) {
        /* track where we are history / adjacency */
        engine.burnSequence++;
        /* transfer the data to the print head */
        loadPrintLine();         
    }
}

/******************************************************************************/
/*!   \fn void lineTimerIsr( void )

      \brief
         This function handles the general purpose line timer interrupts.
        Compare 2 is dual purpose history and pwm start time.


      \author
          Aaron Swift
*******************************************************************************/
void lineTimerIsr ( void )
{  
    static bool pwmStartTime_ = false, pwmSltTime_ = false;
    
    /* latch hist/adj load and burn, start pwm to hold line temperature */
    if( ( GPT_GetStatusFlags( LINE_PRINTER_TIMER_BASE, kGPT_OutputCompare1Flag ) & kGPT_OutputCompare1Flag ) == kGPT_OutputCompare1Flag ) {
        GPT_ClearStatusFlags( LINE_PRINTER_TIMER_BASE, kGPT_OutputCompare1Flag );		
        /* setup for pwm start time after history isr */
        if( !pwmStartTime_ ) { 
                
            /* latch control during history */ 
            if( !isCurrentLine() ) {    
  
                    /* latch was set Low in LPSPI_MasterUserCallback(); lets set it back to High now.
                       this should give us around ~150Us pulse width. */ 
                    GPIO_WritePinOutput( PHEAD_LATCH_GPIO, PHEAD_LATCH_PIN, true ); //true
                    
                    /* small delay before we assert the strobe, 
                       according to datasheet should be 100ns min 
                    delay(1);*/

                    /* added to time history 
                    GPIO_WritePinOutput( ACCEL_INTB_GPIO, ACCEL_INTB_PIN, true );*/
            }

            /* enable the strobe*/
            GPIO_WritePinOutput( PHEAD_STROBE_EN_GPIO, PHEAD_STROBE_EN_PIN, false );  
            /* assert strobe */
            GPIO_WritePinOutput(PHEAD_STROBE_A_GPIO, PHEAD_STROBE_A_PIN, false);  //false
            GPIO_WritePinOutput(PHEAD_STROBE_B_GPIO, PHEAD_STROBE_B_PIN, false);
            /* get the current line loaded in the head but don't latch until current line time */
            lineTimerBurn();  
            /* setup for pwm start time */
            GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel1, engine.pwmStartTime );
            pwmStartTime_ = true;
        } else {
            lineTimerStrobe();
            /* added for demo: increase dot heat */
            GPIO_WritePinOutput( PHEAD_STROBE_EN_GPIO, PHEAD_STROBE_EN_PIN, false );  
            /* assert strobe */
            GPIO_WritePinOutput(PHEAD_STROBE_A_GPIO, PHEAD_STROBE_A_PIN, false);  //false
            GPIO_WritePinOutput(PHEAD_STROBE_B_GPIO, PHEAD_STROBE_B_PIN, false);

            /* latch control during current line. 
               data latch was set low above, lets set it back high now. 
               this should give us enough pulse width */ 
            GPIO_WritePinOutput( PHEAD_LATCH_GPIO, PHEAD_LATCH_PIN, true );    //true                  
            pwmStartTime_ = false;
            
            /* setup for history ( next line ) */
            GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel1, engine.histAdj[0].time );            
        }        
    }  
	
    
    /* latch control during Current line load and burn, slt end */
    if( ( GPT_GetStatusFlags( LINE_PRINTER_TIMER_BASE, kGPT_OutputCompare2Flag ) & kGPT_OutputCompare2Flag ) == kGPT_OutputCompare2Flag ) {
        GPT_ClearStatusFlags( LINE_PRINTER_TIMER_BASE, kGPT_OutputCompare2Flag );
        if( !pwmSltTime_ ) {
            /* current line data is transfered to the head, assert the data latch LOW*/ 
            GPIO_WritePinOutput( PHEAD_LATCH_GPIO, PHEAD_LATCH_PIN, false );    //false
            /* setup for end of slt time */
             GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel2, engine.sltTime );
             pwmSltTime_ = true;
        } else { 
            /* end of line burn. reset for next line */
            lineTimerSLT();    
            /* reset for current line ( next line ) */
            GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel2, engine.histAdj[1].time );
           pwmSltTime_ = false;
        }
    }
    
	/* half way through line burn (half slt time) */
    if( ( GPT_GetStatusFlags( LINE_PRINTER_TIMER_BASE, kGPT_OutputCompare3Flag ) & kGPT_OutputCompare3Flag ) == kGPT_OutputCompare3Flag ) {     
        GPT_ClearStatusFlags( LINE_PRINTER_TIMER_BASE, kGPT_OutputCompare3Flag );
        
        if( getDotWearHandle() != NULL && readyToNotify()) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xTaskNotifyFromISR( getDotWearHandle(), DOT_SAMPLE, eSetBits, &xHigherPriorityTaskWoken );  
        }   
        /* added for debug */
	engine.steps++;			
        halfStepMotor();                       
    }
    
    /* added for arm errata 838869 */
    SDK_ISR_EXIT_BARRIER;        
}


/******************************************************************************/
/*!   \fn void shutdownPrintEngine( void )

      \brief
        This function


      \author
          Aaron Swift
*******************************************************************************/
void shutdownPrintEngine(void)
{
    stopLineTimer();  
    printHeadPowerOff();
}


/******************************************************************************/
/*!   \fn void printHeadPowerOff( void )                                                             
 
      \brief
        This function turn off 24v power to the print head.
        

      \author
          Aaron Swift
*******************************************************************************/ 
void printHeadPowerOff( void )
{  
    GPIO_WritePinOutput( PHEAD_POWER_EN_GPIO, PHEAD_POWER_EN_PIN, false );  
}

/******************************************************************************/
/*!   \fn void printHeadPowerOn( void )                                                             
 
      \brief
        This function turn on 24v power to the print head. 
        

      \author
          Aaron Swift
*******************************************************************************/ 
void printHeadPowerOn( void )
{ 
    GPIO_WritePinOutput( PHEAD_POWER_EN_GPIO, PHEAD_POWER_EN_PIN, true );  
}


/******************************************************************************/
/*!   \fn void initializeStepper( StepDir direction  )                                                             
 
      \brief
        This function sets the stepper motor direction. 
        

      \author
          Aaron Swift
*******************************************************************************/ 
void initializeStepper( StepDir direction  )
{    
    initializeMotors( (StepDirM)direction );
}


/******************************************************************************/
/*!   \fn void powerOffStepper( void )                                                             
 
      \brief
        This function turns the stepper motor power off.
        

      \author
          Aaron Swift
*******************************************************************************/ 
void powerOffStepper( void )
{
    powerOffMotors();
}


/******************************************************************************/
/*!   \fn void powerOnStepper( void )                                                             
 
      \brief
        This function turns the stepper motor power on.  
        

      \author
          Aaron Swift
*******************************************************************************/ 
void powerOnStepper( void )
{
    powerOnMotors();
}

/******************************************************************************/
/*!   \fn void setStepperDirection( StepDir direction )                                                             
 
      \brief
        This function set the stepper motor direction.
        

      \author
          Aaron Swift
*******************************************************************************/ 
void setStepperDirection( StepDir direction ) 
{
    setMainMotorDirection( (StepDirM)direction );
    setTakeUpMotorDirection( (StepDirM)direction );
}

/******************************************************************************/
/*!   \fn void halfStepMotor( void )                                                             
 
      \brief
        This function half steps the motor  
        

      \author
          Aaron Swift
*******************************************************************************/ 
void halfStepMotor( void )
{    
    stepMotors();
}

/******************************************************************************/
/*!   \fn void motorStep( StepDir dir, PrStatusInfo *pStatus )

      \brief
        This function counts steps of the motor and set status information
        Assumes the motor is going forward (only use while printing)

      \author
          Kurtis Bell
*******************************************************************************/
void motorStep( StepDir dir, PrStatusInfo *pStatus )
{
    if( dir == FORWARD_ ) {
        pStatus->sensor |= MOTOR_FORWARD;
        
        if( stepCounterEnabled_ == true )
            ++pStatus->counter;

        engine.outOfMediaCnt++;

        label0.position++;
        label1.position++;
        label2.position++;
    } else {
        pStatus->sensor &= ~MOTOR_FORWARD;

        if( stepCounterEnabled_ == true )
            --pStatus->counter;
        
        label0.position--;
        label1.position--;
        label2.position--;
    }
  
}

/******************************************************************************/
/*!   \fn void halfStepMotor( void )

      \brief
        This function counts steps of the motor and set status information
        Assumes the motor is going forward (only use while printing)

      \author
          Kurtis Bell
*******************************************************************************/
void motorStepFast( PrStatusInfo *pStatus )
{
    static bool evenSteps_ = false;
    if( evenSteps_ ) {
        /* LinePrinterEngine direction is always FORWARD */
        pStatus->sensor |= MOTOR_FORWARD;

        if( stepCounterEnabled_ == true ) {
            ++pStatus->counter;
        }
        
        engine.outOfMediaCnt++;
        
        label0.position++;
        label1.position++;
        label2.position++;
    } else {
        evenSteps_ = true;
    }
}

/******************************************************************************/
/*!   \fn void initializePrintEngineTimer( void )

      \brief
        This function initializes and starts the printer operation
        timer with the specified period [us]


      \author
          Aaron Swift
*******************************************************************************/
void initializePrintEngineTimer( uint16_t period_us )
{
     gpt_config_t gptConfig; 

    /* obtain default configuration for GPT module */
    GPT_GetDefaultConfig( &gptConfig );
    gptConfig.divider = ENGINE_TIMER_PRESCALE;

    /* initialize FTM module */
    GPT_Init( ENGINE_TIMER_BASE, &gptConfig );

    GPT_SetOutputCompareValue( ENGINE_TIMER_BASE, kGPT_OutputCompare_Channel1, period_us  );
    GPT_EnableInterrupts( ENGINE_TIMER_BASE, kGPT_OutputCompare1InterruptEnable );    

    /* set gpt interrupt priority. */
    NVIC_SetPriority( ENGINE_TIMER_IRQ, ENGINE_TIMER_PRIORITY );
    EnableIRQ( ENGINE_TIMER_IRQ );
    
    /* start the timer  */
    GPT_StartTimer( ENGINE_TIMER_BASE );   
}

/******************************************************************************/
/*!   \fn void setPrintEngineTimerSlt( void )                                                             
 
      \brief
        This function sets the PrinterOperationTimer to SLT time. 
        SLT time is the overall time for one line to print. This is done 
        so that the step_isr or stepUntil_isr move the paper at the same 
        speed as the Line. 
        

      \author
          Aaron Swift
*******************************************************************************/ 
void setPrintEngineTimerSlt( void )
{
    GPT_Deinit( ENGINE_TIMER_BASE );
    initializePrintEngineTimer( engine.sltHalfTime );
}

/******************************************************************************/
/*!   \fn void setPrintEngineTimer( unsigned short time )                                                             
 
      \brief
        This function sets the PrinterOperationTimer to time. 
        This is done so that we can slow the step rate when sizing a label. 
  
      \author
          Aaron Swift
*******************************************************************************/ 
void setPrintEngineTimer( unsigned short time )
{
    GPT_Deinit( ENGINE_TIMER_BASE );
    initializePrintEngineTimer( time );
}

void setEngineContrast( unsigned short contrast )
{
    if( contrast <= MAX_CONTRAST ) {
        engine.contrast = contrast;
    }
}

/******************************************************************************/
/*!   \fn void resetPrintEngineTimer( void )

      \brief
        This function reset the print engine timer back to the 1mS rate.

      \author
          Aaron Swift
*******************************************************************************/
void resetPrintEngineTimer( void )
{
    /* disable Timer before changing values */
    GPT_Deinit( ENGINE_TIMER_BASE );
    initializePrintEngineTimer( DEFAULT_ENGINE_COUNT );
}

/******************************************************************************/
/*!   \fn void resetEngine( void )

      \brief
        This function reset the print engine after it is paused.
        This function is called by the general purpose timer and 
        is not intended to be called directly.                                            

      \author
          Aaron Swift
*******************************************************************************/
void resetEngine( void )
{
    /* setHeadPower( 1 );  removed for print head engine timing measurements*/
   
    setStepperDirection( FORWARD_ );
    powerOnStepper();
        
    startLineTimer( true );  
    
    engine.pause = false;
}

/******************************************************************************/
/*!   \fn void resetEngine( void )

      \brief
        This function pauses the print engine.

      \author
          Aaron Swift
*******************************************************************************/
void pauseEngine( void )
{  
    engine.pause = true;
    
    stopLineTimer();

    /* so motors do not overheat */
    powerOffStepper();
   
    setHeadPower( 0 );  
}

#if 0 /* TO DO: port */
/******************************************************************************/
/*!   \fn void initializePwm( void )

      \brief
        This function configures the three PWMs used by the printhead,
        media sensor, and label taken sensor. The label taken pwm is setup 
        for a 500uS period (2KHz) and 12% active high duty cycle 
       (emitter on 12% of the time). The media sensor. pwm is setup for a 
       66uS period (15KHz) and 1% active high duty cycle
       (emitter on 1% of the time). The duty cycle will be raised when the 
       media sensor is calibrated.                                          
    
      \author
          Aaron Swift
*******************************************************************************/
void initializePwm( void )
{
    if( getMyModel() != K_FSSS ) { 
        /* wait for weigher thread to intialize interface */
        while( !I2C_IsInitialized() ) {
            taskYIELD();
        }
    } else {
        /* no weigher present need to intialize i2c interface */
        I2C_Initialize( I2C_BAUDRATE_100HZ );    
    }
    /* setup the label taken sensor clock -- low stock sensor for freestanding */
    setDutyCycleDS1050( DS1050_LTS_DEVICE_ADDR, PWM_125_PRECENT_DUTY ); 
    wakeDS1050( DS1050_LTS_DEVICE_ADDR );

    /* media sensor not needed for freestanding scale */
    if( getMyModel() != K_FSSS ) {
        /* setup media sensor pwm */
        setDutyCycleDS1050( DS1050_MS_DEVICE_ADDR, config_.media_sensor_adjustment ); 
        wakeDS1050( DS1050_MS_DEVICE_ADDR ); 
        PRINTF("initializePwm(): media sensor at: %d\r\n", config_.media_sensor_adjustment );
    }    
}
#endif

/******************************************************************************/
/*!   \fn void initializePrintHeadPwm( void )                                                             
 
      \brief
        This function configures the PWM channel for the print strobe.
                                                          
      \author
          Aaron Swift
*******************************************************************************/ 
void initializePrintHeadPwm( void )
{

    pwm_signal_param_t pwmSignal;
    
 
    pwmSignal.pwmChannel       = kPWM_PwmA;
    pwmSignal.level            = kPWM_HighTrue;
    /* strobe signal is active low! pulse width low is measured */
    pwmSignal.dutyCyclePercent = 100 - engine.pwmDutyCycle;
    pwmSignal.deadtimeValue    = 0;
    pwmSignal.faultState       = kPWM_PwmFaultState0;    
    /* disable gpio control of the pin */
    /* select pwm control of the printhead strobe pin */
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_38_GPIO3_IO06, 0x10B0U );

    /* period = 39.58uS ==> frequency ~= 25,625 Hz */
    /* duty Cycle updated in start printEngine */
    if( PWM_SetupPwm( PWM2, kPWM_Module_0, &pwmSignal, 1U, kPWM_SignedCenterAligned, 25265U , CLOCK_GetFreq( kCLOCK_IpgClk ) ) != kStatus_Success ) {                       
        PRINTF( "initializePrintHeadPwm(): Error critical print strobe pwm failure! \r\n" );                        
    }
}

/******************************************************************************/
/*!   \fn PrintEngine *getPrintEngine( void )                                                             
 
      \brief
        This function returns a pointer to the print engine. 
                                                          
      \author
          Aaron Swift
*******************************************************************************/ 
PrintEngine *getPrintEngine( void )
{
    return &engine;
}

/******************************************************************************/
/*!   \fn bool isEnginePaused( void )                                                             
 
      \brief
        This function returns . 
                                                          
      \author
          Aaron Swift
*******************************************************************************/ 
bool isEnginePaused()
{
    if(engine.pause) {
        engine.pause = false;  
        return true;
    } else {
        return false;
    }
}

/******************************************************************************/
/*!   \fn PrintEngine *getPrintEngine( void )                                                             
 
      \brief
        This function returns the number of processed print lines. 
                                                          
      \author
          Aaron Swift
*******************************************************************************/                             
unsigned long getPrintEngineLineCntr( void )
{
    return  engine.lineCounter;
}

/******************************************************************************/
/*!   \fn static void printEngineISR( void )

      \brief
        This function is the flex timer interrupt handler used for the 
        print engine. This timer executes at a rate of 400uS or 2.5Khz.
 
      \author
          Aaron Swift
*******************************************************************************/
void printEngineISR( void ) 
{       

    if( ( GPT_GetStatusFlags( ENGINE_TIMER_BASE, kGPT_OutputCompare1Flag ) & 
          kGPT_OutputCompare1Flag ) == kGPT_OutputCompare1Flag ) {
  
        //readADChannels();    

        /* call the current operational command */ 
        switch( engine.currentCmd.generic.directive )
        {              
            case IDLE_DIRECTIVE: {
                idleOp();
                break;
            }
            case PRINT_DIRECTIVE: {
                printOp( &engine.currentCmd );
                break;
            }
            case STEP_DIRECTIVE: {                
                stepOp( (StepOperation *)&engine.currentCmd );
                break;
            }
            case STEP_UNTIL_DIRECTIVE: {
                stepUntilOp( (StepUntilOperation *)&engine.currentCmd );                       
                break;
            }
            case WAIT_DIRECTIVE: {            
                waitOp( (WaitOperation *)&engine.currentCmd );              
                break;
            }
            case WAIT_UNTIL_DIRECTIVE: {            
                waitUntilOp( (WaitUntilOperation *)&engine.currentCmd );
                break;
            }
            case TEST_DIRECTIVE: {            
                testOp( (TestOperation *)&engine.currentCmd );
                break;
            }
            case STATUS_DIRECTIVE: {            
                statusOp( (StatusOperation *)&engine.currentCmd );
                break;
            }
            case COUNTER_DIRECTIVE: {            
                counterOp( (CounterOperation *)&engine.currentCmd );
                break;
            }
            case DISABLE_DIRECTIVE: {          
                disableOp( &engine.currentCmd );
                break;
            }
            case CALIBRATE_DIRECTIVE: {            
                calibrateOp( &engine.currentCmd );
                break;
            }
            case NOOPERATION_DIRECTIVE: {
                break;
            }
            case CUT_DIRECTIVE:{
                cutOp( (GenericOperation *)&engine.currentCmd );
                break;
            }
            case HEAD_TEST_DIRECTIVE: {
                printDotWearOp( &engine.currentCmd );
                break;
            }
            case VIRTUAL_CUT_DIRECTIVE: {
                stepOp( (StepOperation *)&engine.currentCmd );
                break;
            }
            case STEP_GAP_DIRECTIVE: {
                stepGapOp( (StepUntilOperation *)&engine.currentCmd );
                break;
            }
            case STEP_EDGE_DIRECTIVE: {
                stepEdgeOp( (StepUntilOperation *)&engine.currentCmd );
                break;
            }
            case TEST_FOR_SYNC: {
                testForSyncOp( (StepOperation *)&engine.currentCmd );
                break;
            }
            case TEST_FOR_LABEL: {
                testForLabelOp( (StepOperation *)&engine.currentCmd );
                break;
            }
            case WAIT_UNTIL_SIZING: {
                waitUntilSizingOp( (WaitUntilOperation *)&engine.currentCmd );
                break;
            }
            default:
              break;
          
        }
    }

    GPT_ClearStatusFlags( ENGINE_TIMER_BASE,  kGPT_OutputCompare1Flag );   

    /* added for arm errata 838869 */
    SDK_ISR_EXIT_BARRIER;        
}

/******************************************************************************/
/*!   \fn void compareStatus( PrStatusInfo *pCurrent, PrStatusInfo *pPrevoius )

      \brief
        This function compare the current printer status with the prevoius


      \author
          Aaron Swift
*******************************************************************************/
void compareStatus( PrStatusInfo *pCurrent, PrStatusInfo *pPrevious)
{
    /* set error bits to history */
    pCurrent->history |= pCurrent->error;
    if( (pPrevious->error != pCurrent->error) ||
      ( pPrevious->state != pCurrent->state ) ||
      ( pPrevious->command != pCurrent->command ) ||  
    ( ( pPrevious->sensor & pCurrent->mask.sensor ) != ( pCurrent->sensor & pCurrent->mask.sensor ) ) ||
    ( ( pPrevious->sensor2 & pCurrent->mask.sensor2 ) != ( pCurrent->sensor2 & pCurrent->mask.sensor2 ) ) ||
    ( ( pPrevious->user & pCurrent->mask.user) != ( pCurrent->user & pCurrent->mask.user ) ) ) {     
        sendPrStatus( pCurrent, true );
        
        /* set the prevoius to the current */
        memcpy( pPrevious, pCurrent, sizeof(PrStatusInfo) );

    }   
}

/******************************************************************************/
/*!   \fn bool testCondition( PrStatusInfo *pStatus, TestOperator oper, 
                              unsigned char bits, unsigned char result )

      \brief
        This function


      \author
          Aaron Swift
*******************************************************************************/
bool testCondition( PrStatusInfo *pStatus, TestOperator oper, unsigned char bits, 
                    unsigned char result )
{
    bool r = false; 
    if( oper == BITS_EQUAL ) {
        if( ( pStatus->sensor & bits ) == bits ) {
            r = true;
        }     
    } else if( oper == BITS_NOT_EQUAL  ) {
        if( ( pStatus->sensor & bits )!= bits ) {
            r = true;
        }          
    } else {  
        PRINTF("testCondition(): Error: Unknown compare operation!\r\n" ); 
    }
    return r;
}

/******************************************************************************/
/*!   \fn void calibratePrinter( PrinterCal cal )
                             
      \brief
        This function


      \author
          Aaron Swift
*******************************************************************************/
void calibratePrinter( PrinterCal cal )
{ 
    currentStatus.command = CALIBRATE_COMMAND;   
    engine.currentCmd.generic.directive = CALIBRATE_DIRECTIVE;

    if( cal == MEDIA_SENSOR_CALIBRATION ) { 
        setOperation( CALIBRATE_DIRECTIVE, &currentStatus );        
    } 
}

/******************************************************************************/
/*!   \fn createCheckerBoardLabel(  unsigned char offset, unsigned long length )
                             
      \brief
        This function blits the checkerboard test pattern into the 
        printer image buffer.

      \author
          Aaron Swift
*******************************************************************************/
void createCheckerBoardLabel( unsigned char offset, unsigned long length )
{    
#if 1   /* make like cm4 / k64 checkerboard image */
    //unsigned short rowByteWidth = getHeadStyleSize() / 8;
    unsigned short rowByteWidth = 72;
    unsigned short centeringOffset = 0;
    unsigned char bmpPitch = BMP_PITCH;
    /* prepack printer 
    unsigned short centeringOffset = (getHeadStyleSize() - stockLoaded.getWidthDots()) / 2; */
    
    //unsigned short leftMostBit = centeringOffset + EDGE_MARGIN; 
    unsigned short leftMostBit = 0;
    unsigned short rightMostBit = leftMostBit + WIDE_STOCK_WIDTH_DOTS - (8 * 2);
    //unsigned short rightMostBit = leftMostBit + WIDE_STOCK_WIDTH_DOTS - EDGE_MARGIN * 2;
    

    /* our buffer is only big enough for a 5" label*/
    if( length <= STEPS_PER_LENGTH5_00 ) {
      
        /* right most bit position cannot be bigger then the print head 
        if( rightMostBit > getHeadStyleSize() ) {
                rightMostBit = getHeadStyleSize();            
        }*/
        /* clear our pattern buffers 
        memset( &pattern1[0], '\0', ( PRINTER_HEAD_SIZE * 2 ) ); 
        memset( &pattern2[0], '\0', ( PRINTER_HEAD_SIZE * 2 ) );  */
        
        memset( &pattern1[0], '\0', ( 72 * 2 ) ); 
        memset( &pattern2[0], '\0', ( 72 * 2 ) );  

        /* create our first bitmap pattern */
        unsigned short i = leftMostBit;
        while( i < rightMostBit ) {
                unsigned short iMax = ( ( i + bmpPitch ) < rightMostBit ) ? i + bmpPitch : rightMostBit;                
                bitSet( i, iMax - i, &pattern1[0] );
                i = i + 2 * bmpPitch;
        }
        
        /* create our second bitmap pattern */
        i = leftMostBit + bmpPitch;
        while (i < rightMostBit)
        {
                unsigned short iMax = ( ( i + bmpPitch ) < rightMostBit ) ? i + bmpPitch : rightMostBit;               
                bitSet( i, iMax - i, &pattern2[0] );
                i = i + 2 * bmpPitch;
        }
                    
        unsigned char *pImage = getImageBuffer();

        unsigned long rowCount = offset;
        bool isPattern1 = true;
        /* copy the test patterns to the image buffer */
        while( rowCount < length ) {
            if( isPattern1 ) {
                    memcpy( ( pImage + rowCount * rowByteWidth ), &pattern1[0], rowByteWidth );
            } else {
                    memcpy(( pImage + rowCount * rowByteWidth ), &pattern2[0], rowByteWidth );
            }
            rowCount++;

            if ( ( rowCount % 32 ) == 0 ) {       // bmpPitch
                isPattern1 = !isPattern1;
            }
        }

        /* clear our pattern buffers 
        memset( &pattern1[0], '\0', (PRINTER_HEAD_SIZE * 2 ) ); 
        memset( &pattern2[0], '\0', (PRINTER_HEAD_SIZE * 2 ) ); */
        
        memset( &pattern1[0], '\0', ( 72 * 2 ) ); 
        memset( &pattern2[0], '\0', ( 72 * 2 ) ); 
        
    } else {
        PRINTF("createCheckerBoardLabel(): Warning: label length too long for test pattern!\r\n" ); 
    }    
#else    
    unsigned short rowByteWidth = getHeadStyleSize() / 8;
    unsigned short centeringOffset = 0;
    unsigned char bmpPitch = BMP_PITCH;
    /* prepack printer 
    unsigned short centeringOffset = (getHeadStyleSize() - stockLoaded.getWidthDots()) / 2; */
    
    unsigned short leftMostBit = centeringOffset + EDGE_MARGIN; 
    unsigned short rightMostBit = leftMostBit + WIDE_STOCK_WIDTH_DOTS - EDGE_MARGIN * 2;
      
    /* our buffer is only big enough for a 5" label*/
    if( length <= STEPS_PER_LENGTH5_00 ) {
      
        /* right most bit position cannot be bigger then the print head */
        if( rightMostBit > getHeadStyleSize() ) {
                rightMostBit = getHeadStyleSize();            
        }
        /* clear our pattern buffers */
        memset( &pattern1[0], '\0', ( PRINTER_HEAD_SIZE * 2 ) ); 
        memset( &pattern2[0], '\0', ( PRINTER_HEAD_SIZE * 2 ) ); 

        /* create our first bitmap pattern */
        unsigned short i = leftMostBit;
        while( i < rightMostBit ) {
                unsigned short iMax = ( ( i + bmpPitch ) < rightMostBit ) ? i + bmpPitch : rightMostBit;                
                bitSet( i, iMax - i, &pattern1[0] );
                i = i + 2 * bmpPitch;
        }
        
        /* create our second bitmap pattern */
        i = leftMostBit + bmpPitch;
        while (i < rightMostBit)
        {
                unsigned short iMax = ( ( i + bmpPitch ) < rightMostBit ) ? i + bmpPitch : rightMostBit;               
                bitSet( i, iMax - i, &pattern2[0] );
                i = i + 2 * bmpPitch;
        }
                    
        unsigned char *pImage = getImageBuffer();

        unsigned long rowCount = offset;
        bool isPattern1 = true;
        /* copy the test patterns to the image buffer */
        while( rowCount < length ) {
            if( isPattern1 ) {
                    memcpy( ( pImage + rowCount * rowByteWidth ), &pattern1[0], rowByteWidth );
            } else {
                    memcpy(( pImage + rowCount * rowByteWidth ), &pattern2[0], rowByteWidth );
            }
            rowCount++;

            if ( ( rowCount % 32 ) == 0 ) {       //bmpPitch
                isPattern1 = !isPattern1;
            }
        }

        /* clear our pattern buffers */
        memset( &pattern1[0], '\0', (PRINTER_HEAD_SIZE * 2 ) ); 
        memset( &pattern2[0], '\0', (PRINTER_HEAD_SIZE * 2 ) ); 
    } else {
        PRINTF("createCheckerBoardLabel(): Warning: label length too long for test pattern!\r\n" ); 
    }
#endif    
}

/******************************************************************************/
/*!   \fn createVerticalLinesLabel(  unsigned char offset, unsigned long length )
                             
      \brief
        This function blits the vertical lines test pattern into the 
        printer image buffer.


      \author
          Aaron Swift
*******************************************************************************/
void createVerticalLinesLabel(  unsigned char offset, unsigned long length )
{	
    unsigned short pattern[8] = { 0xC0, 0xE0, 0xC0, 0xE0, 0xC0, 0xE0, 0xC0, 0xE0 };
    unsigned char *pImage = getImageBuffer();
    unsigned short centeringOffset = 0;
    /* prepack printer 
    unsigned short centeringOffset = (getHeadStyleSize() - stockLoaded.getWidthDots()) / 2; */
        
    unsigned long rowByteWidth = PRINTER_HEAD_SIZE;
    
    unsigned long leftMarginByte =  ( centeringOffset + EDGE_MARGIN ) / 8;
    unsigned long rightMarginByte = ( centeringOffset + WIDE_STOCK_WIDTH_DOTS - EDGE_MARGIN ) / 8;

    /* our buffer is only big enough for a 4" label*/
    if( length <= STEPS_PER_LENGTH4_00 ) {

        unsigned long rowCount = offset;
        while( rowCount < length ) {
                
                unsigned long startByte = leftMarginByte + ( rowCount * rowByteWidth );
                unsigned long lastByte = rightMarginByte + ( rowCount * rowByteWidth );

                for( unsigned long i = startByte; i < lastByte; i++ ) {
                    pImage[i] = (unsigned char)pattern[i % 8];
                }
                rowCount++;
        }
    } else {
        PRINTF("createVerticalLinesLabel(): Warning: label length too long for test pattern!\r\n" ); 
    }
}

/******************************************************************************/
/*!   \fn createSingleVerticalLineLabel(  unsigned char offset, unsigned long length )
                             
      \brief
        This function blits the vertical lines test pattern into the 
        printer image buffer.


      \author
          Aaron Swift
*******************************************************************************/
void createSingleVerticalLineLabel(  unsigned char offset, unsigned long length )
{	
   
    unsigned char *pImage = getImageBuffer();
    unsigned long rowCount = offset;
    /* our buffer is only big enough for a 4" label*/
    if( length <= STEPS_PER_LENGTH4_00 ) {

        //unsigned long rowCount = offset;
        while( rowCount < length ) {
                
                //unsigned long startByte = leftMarginByte + ( rowCount * rowByteWidth );
                //unsigned long lastByte = rightMarginByte + ( rowCount * rowByteWidth );

                //for( unsigned long i = startByte; i < lastByte; i++ ) {
                    pImage[ rowCount * PRINTER_HEAD_SIZE ] = 0x10;
                //}
                rowCount++;
        }
    } else {
        PRINTF("createVerticalLinesLabel(): Warning: label length too long for test pattern!\r\n" ); 
    }
}

/******************************************************************************/
/*!   \fn createHorizontalLinesLabel( unsigned long length )
                             
      \brief
        This function


      \author
          Aaron Swift
*******************************************************************************/
void createHorizontalLinesLabel( unsigned char offset, unsigned long length )
{
    unsigned short centeringOffset = 0;
    /* prepack printer 
    unsigned short centeringOffset = (getHeadStyleSize() - stockLoaded.getWidthDots()) / 2; */
    
    unsigned short leftMostBit = centeringOffset + EDGE_MARGIN;
    unsigned short rightMostBit = leftMostBit + WIDE_STOCK_WIDTH_DOTS - EDGE_MARGIN * 2;
    
    /* right most bit position cannot be bigger then the print head */
    if( rightMostBit > getHeadStyleSize() ) {
        rightMostBit = getHeadStyleSize();
    }

    unsigned long barHeight = BAR_HEIGHT;
    unsigned long blankLineHeight = BAR_HEIGHT * 2;
    unsigned char *pImage = getImageBuffer();
    unsigned long lineWidthBytes = PRINTER_HEAD_SIZE;
    unsigned long rowCount = offset ;
    /* our buffer is only big enough for a 4" label*/
    if( length <= STEPS_PER_LENGTH4_00 ) {
    
        while( rowCount < length ) {
            for( unsigned long j = rowCount; ( (j < ( rowCount + barHeight ) ) && ( j < (length) ) ); j++ ) {
                    bitSet( leftMostBit, rightMostBit - leftMostBit, &pImage[j * lineWidthBytes] );
            }
            rowCount += ( barHeight + blankLineHeight );

            /* increase the size of the white space as we move down the label */
            blankLineHeight++;
            /* increase the size of the black bar as we move down the label */
            barHeight += 2;
        }  
    } else {
        PRINTF("createHorizontalLinesLabel(): Warning: label length too long for test pattern!\r\n" );     
    }
}

/******************************************************************************/
/*!   \fn void bitSet( unsigned short startBit, unsigned short numBits, 
                                                unsigned char *pBuffer )

      \brief
        This function sets bits in a buffer 

      \author
          Aaron Swift
*******************************************************************************/
void bitSet( unsigned short startBit, unsigned short numBits, unsigned char *pBuffer )
{
    unsigned char  mask = 0x80;
    unsigned char  bitsPerMask = 8;

     
    unsigned short firstFullByte, lastPartialByte;
    unsigned short firstFullBit , lastPartialBit;
    
    /* determine first full byte in the line */
    firstFullByte = startBit / bitsPerMask;

    if (startBit % bitsPerMask)
        firstFullByte += 1;
    
    /* very first bit in the very first full byte */
    firstFullBit = firstFullByte * bitsPerMask;	
 
    lastPartialByte = (startBit + numBits) / bitsPerMask;
    lastPartialBit = lastPartialByte * bitsPerMask;

    /* loop through the first dangling bits. there will be anywhere from 1 to 8 */
    for( unsigned short ii = startBit; ii < firstFullBit; ii++ ) {
        pBuffer[ii / bitsPerMask] |= ( mask >> ( ii % bitsPerMask ) );
    }

    if( firstFullByte < lastPartialByte ) {
        /* do all the full bytes one at a time. */
        for( unsigned short ii = firstFullByte; ii < lastPartialByte; ii++ ) {
            pBuffer[ii] = 0xFF;
        }

        /* Do the last few dangling bits. Might also be 1 - 8 of these */
        for( unsigned short ii = lastPartialBit; ii < (startBit + numBits); ii++ ) {
            pBuffer[ii / bitsPerMask] |= ( mask >> ( ii % bitsPerMask ) );
        }
    }  
}

/******************************************************************************/
/*!   \fn void delay( unsigned long )

      \brief
        This function delays in uSeconds. Please use this function when no 
        other method is ( oneshot timer )available.
      \author
          Aaron Swift
*******************************************************************************/
void delay( unsigned long time )
{
    unsigned long i, j, x;
    for( i = 0; i < time; i++ ) {
        for( j = 0; j < USEC_DELAY_COUNT; j++ ) {
            x += 1;
        }  
    }
}

/******************************************************************************/
/*!   \fn void idleOp( void )

      \brief
        This function handles the teach table idle command operation.

      \author
          Aaron Swift
*******************************************************************************/
void idleOp( void )
{
    /* if this is the first time thru, do some initialization.*/
    if( currentStatus.state != ENGINE_IDLE ) {
        
        PRINTF("currentStatus.state: Entering ENGINE_IDLE!\r\n" );
        PRINTF("currentStatus.state: Counter Enabled! %d \r\n", stepCounterEnabled_); 
        PRINTF("currentStatus.state: steps! %d \r\n", engine.steps); 
        PRINTF( "label count: %d \r\n", getGhostMCntr( ) );
        /* added for freestanding scale */
        engine.stepsOffset = 0;
        engine.steps = 0;         
        
        /* added for freestanding scale */
        if( currentStatus.command == 12 )       /* freestanding scale sizing command */
            currentStatus.command = 8;          /* kpc sizing command */
        currentStatus.state = ENGINE_IDLE;
        /*previous command is complete. */
        currentStatus.command |= COMMAND_COMPLETE;
        /* clear our flag if set */
        currentStatus.sensor2 &= ~OUT_OF_DATA_BUFFERS;         
                
        /* sof-3402: make sure we keep missing label bit set until new command is 
           given */
        if( ( currentStatus.user & MISSING_LABEL ) != MISSING_LABEL )   {
            currentStatus.user    = 0;            
        } else {
            /* clear all other flags */
            currentStatus.user    = 0; 
            /* check for ghost missing label */
            if( getGhostMCntr( ) == 0 ) {              
                /* reset missing label bit so UI prompt stays up */
                currentStatus.user    |= MISSING_LABEL;               
            }
        }
        /* notify the host the printer is idle. */        
        compareStatus( &currentStatus, &prevStatus );
        
        /* freestanding scale */
        if( getMyModel() ==  RT_GLOBAL_FSS ) {
            currentStatus.sensor = 0;
        }
        
        engine.steps = 0;
        setHeadTimings();
        powerOffStepper();        
    } else {        
        /* send requested changes in printer status to the controller. */
        compareStatus( &currentStatus, &prevStatus );

        if( pCmdQHandler_ != NULL ) {
            /* check for commands which may have been issued by the system controller. */
            PrCommand cmdMsg;
            
            int numMgs = uxQueueMessagesWaitingFromISR( pCmdQHandler_ );
            if( numMgs ) {
                if( xQueueReceiveFromISR( pCmdQHandler_, &cmdMsg, 0 ) ) {             
                    /* perform some initialization. */
                    currentStatus.command &= ~COMMAND_COMPLETE;
                    /* SOF-5327 report missing on every occurrence */
                    if( ( currentStatus.user & MISSING_LABEL ) == MISSING_LABEL ) {
                        currentStatus.user &= ~MISSING_LABEL;
                        compareStatus( &currentStatus, &prevStatus );
                    }

                    /* set the commad data */
                    setIndirectData( (CMD_DATA_IDS)cmdMsg.data_item, cmdMsg.value );
                    /* intialize the command list */
                    initializeCmdSequence( cmdMsg.identifier, &currentStatus ); 

                } else {
                    /* timing values change with contrast setting and temperature */
                    setHeadTimings();
                }
            }
        } else { 
            PRINTF("idleOp(): Error: pCmdQHandler_ is null!\r\n" );  
        }
    }
}

/******************************************************************************/
/*!   \fn void printOp( CmdOp *pOperation )

      \brief
        This function handles the teach table print command operation.


      \author
          Aaron Swift
*******************************************************************************/
void printOp( CmdOp *pOperation )
{
    /* added to debug drift and correction */
    static bool onceGap_ = false; 
    static bool onceTaken_ = false;
    
    /* do not think this is needed here.  */
    compareStatus( &currentStatus, &prevStatus ); 
   
    /* clear if there was an error */
    currentStatus.error &= ~OUT_OF_PRINTDATA; 
        
    if( currentStatus.state != ENGINE_PRINTING ) {
        PRINTF("currentStatus.state: Entering ENGINE_PRINTING!\r\n" );  
        
        /* added to debug drift and correction */
        onceGap_ = false;
        onceTaken_ = false;
        engine.steps = 0;
        engine.labelTracking = 0;
        /* clear the ghost missing label counter */
        clearGhostMCntr();

        currentStatus.state = ENGINE_PRINTING;
        /* initialize the printer control parameters. */
        /* TO DO: broke. Not pulling values from indirect data! 
        engine.numPrintLines = getOpData( pOperation->print.type, pOperation->print.data );*/ 
        engine.numPrintLines = 609;
        engine.direction = ( engine.numPrintLines < 0 ) ? BACKWARD_ : FORWARD_;
        engine.numPrintLines = abs( engine.numPrintLines );
        engine.totalLinesToPrint = engine.numPrintLines;
        engine.labelOrientation = pOperation->print.orientation;
        
#if 0   /* does not work! close but does not work */       
        /* vertical print position needs to strip lines from the top of the label 
           image which is the bottom of the printed label */
        if( getMyModel() == K_FSSS ) {
            /* subtract the number of vertical lines from the total image size */            
            engine.totalLinesToPrint -= abs( config_.printPosition ); 
            /* advance the our line counter the vertical offset size */ 
            engine.lineCounter += abs( config_.printPosition );
        }
#endif        
        setStepperDirection( engine.direction  );
        powerOnStepper();
        /* SOF-5305 potential fix for fsss intermittent stalling on label print  
        delay(10000); */
        
        if( engine.headType == UNKNOWN_HEAD ) {
            currentStatus.error |= UNKNOWN_PH;
        } else {
            currentStatus.error &= ~UNKNOWN_PH;
        }

        if( currentStatus.error == NO_ERROR ) {
            startPrintEngine();
        }
   } else {
        /* need to leave this in to accurately size labels. */   
        int mediaValue = getMediaCounts();
        /* added for freestanding scale */
        if( getMyModel() != RT_GLOBAL_FSS ) {
            if( mediaValue > MEDIA_SYNC_BAR_THRESHOLD ) {
                if( ( currentStatus.sensor & SYNC_BAR ) != SYNC_BAR ) {
                    currentStatus.sensor |= SYNC_BAR_EDGE;               
                    pCurrentLabel->next->position = 0;
                } else {
                    currentStatus.sensor &= ~SYNC_BAR_EDGE;
                }
                currentStatus.sensor |= SYNC_BAR;             
            } else {
                engine.outOfMediaCnt = 0;    
                if ( ( currentStatus.sensor & SYNC_BAR ) == SYNC_BAR )
                    currentStatus.sensor |= SYNC_BAR_EDGE;    
                else
                    currentStatus.sensor &= ~SYNC_BAR_EDGE;  
                currentStatus.sensor &= ~SYNC_BAR;           
            }

            if( pCurrentLabel->position == labelAlignment ) {
                pCurrentLabel = pCurrentLabel->next;
                currentStatus.sensor |= SYNCHRONIZED;            
            } else if( pCurrentLabel->position > labelAlignment ) {
                pCurrentLabel = pCurrentLabel->next;
                currentStatus.sensor &= ~SYNCHRONIZED;
            } else {
                currentStatus.sensor &= ~SYNCHRONIZED;
            }
        } else {
            
            /* record when we reach a gap or label taken sensor */
            if( ( mediaValue <= BACKING_PAPER_THRESHOLD  ) && ( mediaValue > 1 ) ) {
                pCurrentLabel->next->position = 0;
                if( !onceGap_ ) {
                    onceGap_ = true;
                    /* */
                    PRINTF( "printOp() gap detected at steps: %d\r\n", engine.steps );
                    PRINTF( "printOp() gap: %d\r\n",  mediaValue );
                }
            }

            if( pCurrentLabel->position == labelAlignment ) {
                pCurrentLabel = pCurrentLabel->next;
                currentStatus.sensor |= SYNCHRONIZED;            
            } else if( pCurrentLabel->position > labelAlignment ) {
                pCurrentLabel = pCurrentLabel->next;
                currentStatus.sensor &= ~SYNCHRONIZED;
            } else {
                currentStatus.sensor &= ~SYNCHRONIZED;
            }
            
            if( readLabelTakenSensor() ) {
                if( !onceTaken_ ) {
                    onceTaken_ = true;                    
                    /* do not track if we are using continuous stock */
                    if( !continuousStock_ ) { 
                        PRINTF( "printOp() label at sensor steps: %d\r\n", engine.steps );
                        int wholeSteps = engine.steps / 2;
                        PRINTF( "printOp() wholeSteps: %d\r\n", wholeSteps );
                        /* distance from print head to label taken sensor */
                        if( wholeSteps >  370 ) {
                            engine.labelTracking = ( wholeSteps - 370 ) * 2;
                            PRINTF( "printOp() label correction half steps: %d\r\n", engine.labelTracking ); /**/
                        } else {
                            engine.labelTracking = ( -370 + wholeSteps   ) * 2;
                            PRINTF( "printOp() label correction half steps: %d\r\n", engine.labelTracking ); /**/                    
                        }
                    }
                }
            }                       
        }
        
        /* are we printing a label bigger than our image buffer? */
        if( engine.totalLinesToPrint > N_PRINTER_LINES ) {
            /* have we used the first 1/2 of buffer yet?  */
            if( ( ( N_PRINTER_LINES - 100 ) < engine.lineCounter ) && !bank0_ ) {        
                /* clear the first half of the image buffer */
                unsigned char *pBff = getImageBuffer();
                memset( pBff, 0, ( PRINTER_BUFFER_SIZE / 2 ) );
                /* notify the host the printer is finished with the first half of the print buffer */
                currentStatus.sensor2 |= OUT_OF_DATA_BUFFERS; 
                compareStatus( &currentStatus, &prevStatus );
                bank0_ = true;
            }
        }
        if( engine.lineCounter < 10 ) {
            bank0_ = false;
        }       
        
        #if 0   /* removed for testing printing */
        /* we are printing */
        if( currentStatus.error != 0 ) {
            powerOffStepper();
            bank1_ = bank0_ = false;
            /* return to the idle state */
            setOperation( IDLE_DIRECTIVE, &currentStatus );
        }
        else if( engine.linePrintDone == true ) {
        #else 
        if( engine.linePrintDone == true ) {    
        #endif
            bank1_ = bank0_ = false;
            resetPrintDataLine();
            //if( prevVertOffset_ != config_.printPosition ) {
                //prevVertOffset_ = config_.printPosition;
                /* set flag for stepOp to adjust for next label */
                applyVerticalOffset_ = true;
            //}
            setNextOperation( &currentStatus );            
        }
   }    
}

/******************************************************************************/
/*!   \fn void printDotWear( CmdOp *pOperation )

      \brief
        This function handles the teach table Dot Wear test command operation.
        Starts the print engine but does not step.

      \author
          Aaron Swift
*******************************************************************************/
void printDotWearOp( CmdOp *pOperation )
{
    if( currentStatus.state != ENGINE_PRINTING ) {
        PRINTF("currentStatus.state: Entering ENGINE_DOT_PRINTING!\r\n" );  
        
        currentStatus.state = ENGINE_PRINTING;
        /* initialize the printer control parameters. */      
        engine.direction = FORWARD_;
        /* set a line for each dot in the head */
        engine.numPrintLines = getHeadStyleSize() + 1; // we print the first dot twice
        engine.totalLinesToPrint = engine.numPrintLines;
        engine.labelOrientation = pOperation->print.orientation;
        
       if( currentStatus.error == NO_ERROR ) {
            startPrintEngine();
       }
   } else {
        engine.outOfMediaCnt = 0;
        /* we are printing */
        if( currentStatus.error != 0 ) {
            /* return to the idle state */
            setOperation( IDLE_DIRECTIVE, &currentStatus );
        }
        else if( engine.linePrintDone == true ) {
            currentStatus.command = _TBL_PRINT;
            resetPrintDataLine();            
            setNextOperation( &currentStatus );            
        }
    }    
}

/******************************************************************************/
/*!   \fn void historyAdjacency( void )
      \brief
        This function generates the history and adjacency lines for the next
        print line.
        
      \author
          Aaron Swift
*******************************************************************************/
void historyAdjacency( void )
{    
    unsigned char *pCurrentLine = getCurrentPrintDataLine();  
    
    if( pCurrentLine != NULL ) {
        history( pCurrentLine );
    } else {
        PRINTF("historyAdjacency(): Warning: Out of print head data!\r\n" );            
    }        
}

/******************************************************************************/
/*!   \fn void stepOp( StepOperation *pOperation )
      \brief
        This function handles the teach table step command operation.
        
      \author
          Aaron Swift
*******************************************************************************/
void stepOp( StepOperation *pOperation )
{
    if (currentStatus.state != ENGINE_STEPPING ) {       
        PRINTF("currentStatus.state: Entering Single ENGINE_STEPPING!\r\n" );  
        /* set the state and enable the stepper motor*/       
        currentStatus.state = ENGINE_STEPPING;
        
        /* sync print line with the step rate */
        setPrintEngineTimerSlt();
        
        engine.numSteps = getOpData( pOperation->type, pOperation->data );
        engine.direction = ( engine.numSteps < 0 ) ? BACKWARD_ : FORWARD_;    
        engine.numSteps = abs(engine.numSteps);
        /* printer has two half steps per print line */
        engine.numSteps = engine.numSteps << 1;
        
        /* PRINTF( "stepOp() engine.numSteps: %d\r\n",  engine.numSteps ); */
        /* added for freestanding scale */
        if( getMyModel() == RT_GLOBAL_FSS ) {       
            /* apply vertical offset for next label */    
            if( applyVerticalOffset_ ) {
                applyVerticalOffset_ = false;
                /* vertical negative until > 50 on ui is reached */
                if( config_.verticalPosition <= 0 ) {
                    /* the backend modifies this number */
                    short x = 50 - abs( config_.verticalPosition );
                    engine.numSteps = engine.numSteps + abs( x /*<< 1*/ );
                } else {                 
                   short x = 50 + abs( config_.verticalPosition );  
                   engine.numSteps =  abs( x /*<< 1 */) + engine.numSteps;
                }
            }
            /* correct for next label print */
            if( engine.labelTracking != 0 ){
                engine.numSteps += engine.labelTracking;
                engine.labelTracking = 0;
            } 
        }  
        /* PRINTF( "stepOp() engine.numSteps: %d\r\n",  engine.numSteps ); */
        if( engine.direction == BACKWARD_ ) {

            setStepperDirection( BACKWARD_ );
            powerOnStepper();
            /* sync print line with the step rate */ 
            resetPrintEngineTimer();
            engine.steps++;
            halfStepMotor();          

            motorStep( engine.direction, &currentStatus );            

        } else {
          
            setStepperDirection( FORWARD_ );
            powerOnStepper();
             /*changed to keep motor from stalling at 6ips when sizing labels -- ats 07102014*/
            if( ( engine.headType == KYOCERA753_OHM ) || ( engine.headType == KYOCERA800_OHM ) || 
                ( engine.headType == KYOCERA849_OHM ) ) {
                delay(10000);         
            } else {
                engine.steps++;
                halfStepMotor();                  
                motorStep( engine.direction, &currentStatus );
            }
        }
    } else {   

        /* if a freestanding scale then the media sensor has been removed */
        if( getMyModel() != RT_GLOBAL_FSS ) {
           
            /* need to leave this in to accurately size labels. */   
            int mediaValue = getMediaCounts();
            
            if( mediaValue > MEDIA_SYNC_BAR_THRESHOLD ) {
                if( ( currentStatus.sensor & SYNC_BAR ) != SYNC_BAR ) {                
                    currentStatus.sensor |= SYNC_BAR_EDGE;               
                    pCurrentLabel->next->position = 0;
                } else {
                    currentStatus.sensor &= ~SYNC_BAR_EDGE;
                }
                currentStatus.sensor |= SYNC_BAR;             
            } else {
                engine.outOfMediaCnt = 0;    
                if ( ( currentStatus.sensor & SYNC_BAR ) == SYNC_BAR )
                    currentStatus.sensor |= SYNC_BAR_EDGE;    
                else
                    currentStatus.sensor &= ~SYNC_BAR_EDGE;  
                currentStatus.sensor &= ~SYNC_BAR;           
            }
        } else {
            /* need to leave this in to accurately size labels. */   
            int mediaValue = getMediaCounts();
            
            if( mediaValue < BACKING_PAPER_THRESHOLD ) {
                if( ( currentStatus.sensor & SYNC_BAR ) != SYNC_BAR ) {                
                    currentStatus.sensor |= SYNC_BAR_EDGE;               
                    pCurrentLabel->next->position = 0;
                } else {
                    currentStatus.sensor &= ~SYNC_BAR_EDGE;
                }
                currentStatus.sensor |= SYNC_BAR;             
            } else {
                engine.outOfMediaCnt = 0;    
                if ( ( currentStatus.sensor & SYNC_BAR ) == SYNC_BAR )
                    currentStatus.sensor |= SYNC_BAR_EDGE;    
                else
                    currentStatus.sensor &= ~SYNC_BAR_EDGE;  
                currentStatus.sensor &= ~SYNC_BAR;           
            }
        }

        if( pCurrentLabel->position == labelAlignment ) {            
            pCurrentLabel = pCurrentLabel->next;
            currentStatus.sensor |= SYNCHRONIZED;
        } else if( pCurrentLabel->position > labelAlignment ) {
            pCurrentLabel = pCurrentLabel->next;
            currentStatus.sensor &= ~SYNCHRONIZED;
        } else {
            currentStatus.sensor &= ~SYNCHRONIZED;
        }
        
        if( engine.outOfMediaCnt < engine.maxMediaCount ) {
            currentStatus.sensor &= ~OUT_OF_MEDIA;
            currentStatus.error &= ~MEDIA_SHUTDOWN;
        } else {
            /* removed for freestanding scale -- replace with shoot-through
            currentStatus.sensor |= OUT_OF_MEDIA; */
            
            if( currentStatus.state == ENGINE_PRINTING || currentStatus.state == ENGINE_CALIBRATING )
                currentStatus.error |= MEDIA_SHUTDOWN;
            else        
                currentStatus.error &= ~MEDIA_SHUTDOWN;
        }
        /* added for freestanding scale */
        if( getMyModel() != RT_GLOBAL_FSS ) {
            /* read label taken sensor */
            if( GPIO_ReadPinInput( LABEL_TAKEN_SENSOR_GPIO, LABEL_TAKEN_SENSOR_PIN ) ) { 
                currentStatus.sensor |= LABEL_TAKEN;
            } else {
                currentStatus.sensor &= ~LABEL_TAKEN;
            }
        } else {
            /* read label taken sensor */            
            if( !readLabelTakenSensor() ) {
                currentStatus.sensor |= LABEL_TAKEN;
            } else {
                currentStatus.sensor &= ~LABEL_TAKEN;                
            }
        }


        /* check for error conditions. */
        if( currentStatus.error != NO_ERROR ) {
            /* return the printer to the idle state */    
            powerOffStepper();
            setOperation( IDLE_DIRECTIVE, &currentStatus ); 
        } else if ( engine.numSteps-- <= 0 ) {
            /* sync the print timer to the motor steps. */
            resetPrintEngineTimer();
            setNextOperation( &currentStatus );
            
            /*eliminate click on start of continuous stock label print*/
            if( ( engine.headType == KYOCERA753_OHM ) || ( engine.headType == KYOCERA800_OHM ) || 
                ( engine.headType == KYOCERA849_OHM ) ) {
                if( pOperation->directive != PRINT_DIRECTIVE ) {
                    powerOffStepper();	
                }
            } else {
                powerOffStepper();	
            }            
        } else {
          
            int even = 0;

            even = engine.numSteps & 0x0001;
            if( even == 0 ) {
                /* Two half steps per printline */  
                motorStep( engine.direction, &currentStatus );
            }
            engine.steps++;
            halfStepMotor(); 
            /* notify host of any status change */
            compareStatus( &currentStatus, &prevStatus ); 
        }
    }
}

/******************************************************************************/
/*!   \fn void stepUntilOp( StepUntilOperation *pOperation )
      \brief
        This function handles the teach table step until command operation.
        
      \author
          Aaron Swift
*******************************************************************************/
void stepUntilOp( StepUntilOperation *pOperation )
{
    /* static int index_ = 0; */

    if( currentStatus.state != ENGINE_STEPPING ) {
        PRINTF("currentStatus.state: Entering Single ENGINE_STEPPING_UNTIL!\r\n" );

        /* added for freestanding scale */
        if( getMyModel() != RT_GLOBAL_FSS ) {        

            if( testCondition( &currentStatus, pOperation->operator, pOperation->bits, 
                      pOperation->result) == true)
            {
                PRINTF( "Here!! steps: %d\r\n",  engine.steps );
                
                /* if we are going to skip stepping then we need to clear the user 
                   status because we do not set label present and taken status bits.
                   when the test is ran (sizing) then a missing label error is generated 
                   in the backend which is not handled correctly and causes the backend to 
                   stop printing labels. This condition is recreated after x number of reboots.
                */
                skipMissingLabel_ = true;
                /* stepping complete */
                resetPrintEngineTimer();
                powerOffStepper();
                setNextOperation( &currentStatus );
            }
        }

        /* freestanding scale        
        index_ = 0; */

        currentStatus.state = ENGINE_STEPPING;
  
        /* notify host of state change */
        compareStatus( &currentStatus, &prevStatus ); 
        

        engine.steps = 0;    
        powerOnStepper();

        /*if we are sizing a label, slow sizing speed to 3ips (motor stall ) -- ats */ 
        if( ( pOperation->bits == MOTOR_FORWARD | SYNC_BAR | SYNC_BAR_EDGE ) && ( ( engine.headType == KYOCERA753_OHM ) ||
            ( engine.headType == KYOCERA800_OHM ) || ( engine.headType == KYOCERA849_OHM ) ) ) {
            currentStatus.sensor |= MOTOR_FORWARD;
            currentStatus.sensor &= ~LABEL_TAKEN;
            
            setPrintEngineTimer( getSltSizingTime( engine.headType ) );
        } else {
            setPrintEngineTimerSlt();
	}
             
        engine.numSteps = getOpData(pOperation->type, pOperation->data);
        engine.direction = ( engine.numSteps < 0 ) ? BACKWARD_ : FORWARD_;    
        engine.numSteps = abs(engine.numSteps);
        engine.numSteps = engine.numSteps << 1;
        
        PRINTF( "number of steps allowed: %d\r\n",  engine.numSteps );
        /* initializeStepper( engine.direction ); */
        setStepperDirection( engine.direction );
        /* removed to keep motor from stalling at 6ips when sizing labels -- ats 07102014 */	     
        if( ( engine.headType == KYOCERA753_OHM ) || ( engine.headType == KYOCERA800_OHM ) ||            
            ( engine.headType == KYOCERA849_OHM ) ) {
            engine.steps++;
            halfStepMotor();          	
            motorStep( engine.direction, &currentStatus );
        }
    } else {
        /* if a freestanding scale then the media sensor has been removed */
        if( getMyModel() != RT_GLOBAL_FSS ) {
            /* need to leave this in to accurately size labels. */   
            int mediaValue = getMediaCounts();
            
            if( mediaValue > MEDIA_SYNC_BAR_THRESHOLD ) {
                if( ( currentStatus.sensor & SYNC_BAR ) != SYNC_BAR ) {
                    currentStatus.sensor |= SYNC_BAR_EDGE;               
                    pCurrentLabel->next->position = 0;
                } else {
                    currentStatus.sensor &= ~SYNC_BAR_EDGE;
                }
                currentStatus.sensor |= SYNC_BAR;             
            } else {
                engine.outOfMediaCnt = 0;    
                if ( ( currentStatus.sensor & SYNC_BAR ) == SYNC_BAR )
                    currentStatus.sensor |= SYNC_BAR_EDGE;    
                else
                    currentStatus.sensor &= ~SYNC_BAR_EDGE;  
                currentStatus.sensor &= ~SYNC_BAR;           
            }
        } else {
            int mediaValue = getMediaCounts();
            /* sizing debug - freestanding scale 
            if( index_ <= 1000 )
                pMediaSizing[index_++] = (unsigned char)mediaValue;*/
            
            if( mediaValue <= BACKING_PAPER_THRESHOLD  ) {
                
                if( ( currentStatus.sensor & MOTOR_FORWARD ) == MOTOR_FORWARD ){
                    pCurrentLabel = pCurrentLabel->next;
                    currentStatus.sensor |= SYNCHRONIZED;    
                } else {
                    if( ( currentStatus.sensor & SYNC_BAR ) != SYNC_BAR ) {
                        currentStatus.sensor |= SYNC_BAR;  
                        currentStatus.sensor &= ~SYNCHRONIZED;
                        pCurrentLabel->next->position = 0;
                    } else {
                        currentStatus.sensor &= ~SYNC_BAR_EDGE;
                    }
                    currentStatus.sensor |= SYNC_BAR;             
                }
            } else { 
                
                engine.outOfMediaCnt = 0; 
                if( mediaValue >= LABEL_EDGE_THRESHOLD ) {
                    currentStatus.sensor &= ~SYNCHRONIZED;
                }
                    /*
                    engine.outOfMediaCnt = 0;    
                    if ( ( currentStatus.sensor & SYNC_BAR ) == SYNC_BAR )
                        currentStatus.sensor |= SYNC_BAR_EDGE;    
                    else
                        currentStatus.sensor &= ~SYNC_BAR_EDGE;  
                    currentStatus.sensor &= ~SYNC_BAR;           
                    */
            }        
        }
        /* added for freestanding scale */
        if(  getMyModel() != RT_GLOBAL_FSS  ) {
            if( pCurrentLabel->position == labelAlignment ) {
                pCurrentLabel = pCurrentLabel->next;
                currentStatus.sensor |= SYNCHRONIZED;            
            } else if( pCurrentLabel->position > labelAlignment ) {
                pCurrentLabel = pCurrentLabel->next;
                currentStatus.sensor &= ~SYNCHRONIZED;
            } else {
                currentStatus.sensor &= ~SYNCHRONIZED;
            }
        }
        if( engine.outOfMediaCnt < engine.maxMediaCount ) {
            currentStatus.sensor &= ~OUT_OF_MEDIA;
            currentStatus.error &= ~MEDIA_SHUTDOWN;
        } else {
            /* removed for freestanding scale -- replace with shoot-through
            currentStatus.sensor |= OUT_OF_MEDIA;
            */
            if( currentStatus.state == ENGINE_PRINTING || currentStatus.state == ENGINE_CALIBRATING )
                currentStatus.error |= MEDIA_SHUTDOWN;
            else        
                currentStatus.error &= ~MEDIA_SHUTDOWN;
        }
        
        /* added for freestanding scale */
        if( getMyModel() != RT_GLOBAL_FSS ) {        
            /* read label taken sensor */
            if( GPIO_ReadPinInput( LABEL_TAKEN_SENSOR_GPIO, LABEL_TAKEN_SENSOR_PIN ) ) { 
                currentStatus.sensor |= LABEL_TAKEN;
            } else {
                currentStatus.sensor &= ~LABEL_TAKEN;
            }
        } else {        
            /* read label taken sensor */
            if( !readLabelTakenSensor() ) {
                currentStatus.sensor |= LABEL_TAKEN;                
            } else {
                currentStatus.sensor &= ~LABEL_TAKEN;
            }
        }


        /* check for error conditions. */
        if( currentStatus.error != NO_ERROR ) {
            /* return the printer to the idle state */    
            powerOffStepper();
            setOperation( IDLE_DIRECTIVE, &currentStatus ); 
        } else if ( ( testCondition( &currentStatus, 
                  pOperation->operator, 
                  pOperation->bits, 
                  pOperation->result) == false) 
                  &&  ( --engine.numSteps > 0 ) ) {    

            int even = 0;
            even = engine.numSteps & 0x0001;

            if( even == 0 ) {
                /* two half steps per print line */            
                motorStep( engine.direction, &currentStatus );
            }
            
            engine.steps++;
            halfStepMotor();
            /* notify host of any status change */
            compareStatus( &currentStatus, &prevStatus ); 
        } else {
                        
            PRINTF( "steps: %d\r\n",  engine.steps );
            PRINTF( "engine.numSteps: %d\r\n",  engine.numSteps );
            /* stepping complete */
            resetPrintEngineTimer();
            powerOffStepper();
                                    
            setNextOperation( &currentStatus );
            /* sizing debug - freestanding scale 
            for( int i = 0; i < index_; i++ ) {
                PRINTF( "media counts: %d\r\n",  pMediaSizing[i] );
            }*/
            
        }
    }	
}

/******************************************************************************/
/*!   \fn void stepGapOp( StepUntilOperation *pOperation )
      \brief
        This function handles the teach table find the label gap command operation.
        
      \author
          Aaron Swift
*******************************************************************************/
void stepGapOp( StepUntilOperation *pOperation )
{
    static int index_ = 0;
    if( currentStatus.state != ENGINE_STEPPING ) {
        PRINTF("currentStatus.state: Entering Single ENGINE_STEPPING_UNTIL_GAP!\r\n" );
        
        /* only for freestanding scale */
        if( getMyModel() != RT_GLOBAL_FSS ) {        
                skipMissingLabel_ = true;
                /* stepping complete */
                resetPrintEngineTimer();
                powerOffStepper();
                setNextOperation( &currentStatus );         
        }
    
        currentStatus.state = ENGINE_STEPPING;
  
        /* notify host of state change */
        compareStatus( &currentStatus, &prevStatus ); 
        /* have the sensor ISR check and stop motor on the gap */
        setMotorStopOnGap();

        /* freestanding scale */        
        index_ = 0;
                  
        powerOnStepper();

        /*we are sizing a label, slow sizing speed to 3ips  */ 
        currentStatus.sensor |= MOTOR_FORWARD;
        currentStatus.sensor &= ~LABEL_TAKEN;
            
        setPrintEngineTimer( getSltSizingTime( engine.headType ) );
             
        engine.numSteps = getOpData(pOperation->type, pOperation->data);
        engine.direction = ( engine.numSteps < 0 ) ? BACKWARD_ : FORWARD_;    
        /* 10" of label travel into steps 
        engine.numSteps = 10 * 203; */
        engine.numSteps = abs(engine.numSteps);
        engine.numSteps = engine.numSteps << 1;
        
        /* PRINTF( "number of steps allowed: %d\r\n",  engine.numSteps ); */
        /* initializeStepper( engine.direction ); */
        setStepperDirection( engine.direction );
        /* removed to keep motor from stalling at 6ips when sizing labels -- ats 07102014 */	     
        if( ( engine.headType == KYOCERA753_OHM ) || ( engine.headType == KYOCERA800_OHM ) ||            
            ( engine.headType == KYOCERA849_OHM ) ) {
            engine.steps++;
            halfStepMotor();          	
            motorStep( engine.direction, &currentStatus );
        }
    } else {
        /* if a freestanding scale then the media sensor has been removed */
        int mediaValue = getMediaCounts();   
        /* sizing debug - freestanding scale 
        if( index_ <= 1000 )
            pMediaSizing[index_++] = (unsigned char)mediaValue;
        */
        if( mediaValue <= BACKING_PAPER_THRESHOLD  ) {
            PRINTF( "gap count: %d\r\n",  mediaValue );
            if( ( currentStatus.sensor & MOTOR_FORWARD ) == MOTOR_FORWARD ){
                pCurrentLabel = pCurrentLabel->next;
                currentStatus.sensor |= SYNCHRONIZED;    
            } else {
                if( ( currentStatus.sensor & SYNC_BAR ) != SYNC_BAR ) {
                    currentStatus.sensor |= SYNC_BAR;  
                    currentStatus.sensor &= ~SYNCHRONIZED;
                    pCurrentLabel->next->position = 0;
                } else {
                    currentStatus.sensor &= ~SYNC_BAR_EDGE;
                }
                currentStatus.sensor |= SYNC_BAR;             
            }
        } else {             
            engine.outOfMediaCnt = 0; 
            if( mediaValue >= LABEL_EDGE_THRESHOLD ) {
                currentStatus.sensor &= ~SYNCHRONIZED;
            }
        }        
        
        if( engine.outOfMediaCnt < engine.maxMediaCount ) {
            currentStatus.sensor &= ~OUT_OF_MEDIA;
            currentStatus.error &= ~MEDIA_SHUTDOWN;
        } else {
            if( currentStatus.state == ENGINE_PRINTING || currentStatus.state == ENGINE_CALIBRATING )
                currentStatus.error |= MEDIA_SHUTDOWN;
            else        
                currentStatus.error &= ~MEDIA_SHUTDOWN;
        }
        
        /* read label taken sensor */
        if( !readLabelTakenSensor() ) {
            currentStatus.sensor |= LABEL_TAKEN;                
        } else {
            currentStatus.sensor &= ~LABEL_TAKEN;
        }
        
        /* check for error conditions. */
        if( currentStatus.error != NO_ERROR ) {
            /* return the printer to the idle state */    
            powerOffStepper();
            setOperation( IDLE_DIRECTIVE, &currentStatus ); 
        } else if ( ( testCondition( &currentStatus, 
                  pOperation->operator, 
                  pOperation->bits, 
                  pOperation->result) == false) 
                  &&  ( --engine.numSteps > 0 ) ) {    

            int even = 0;
            even = engine.numSteps & 0x0001;

            if( even == 0 ) {
                /* two half steps per print line */            
                motorStep( engine.direction, &currentStatus );
            }
            
            engine.steps++;
            halfStepMotor();
            /* notify host of any status change */
            compareStatus( &currentStatus, &prevStatus ); 
        } else {
            
            clrMotorStopOnGap();
            
            PRINTF( "steps: %d\r\n",  engine.steps );
            PRINTF( "engine.numSteps: %d\r\n",  engine.numSteps );
            /* stepping complete */
            resetPrintEngineTimer();
            powerOffStepper();
            
            /* have we detected continuous stock while sizing */
            if( engine.steps > CONTINUOUS_STOCK_MIN ) {
                continuousStock_ = true;
                /*SOF-5183: jump to the end of sizing since we know the stock size already */
                jumpToOperation( &currentStatus,  10 );
                
            } else {
                continuousStock_ = false;
                setNextOperation( &currentStatus ); 
            }
            
            /* sizing debug - freestanding scale 
            for( int i = 0; i < index_; i++ ) {
                PRINTF( "media counts: %d\r\n",  pMediaSizing[i] );
            }*/
        }
    }	
}

/******************************************************************************/
/*!   \fn void stepEdgeOp( StepUntilOperation *pOperation )
      \brief
        This function handles the teach table find the label edge command operation.
        
      \author
          Aaron Swift
*******************************************************************************/
void stepEdgeOp( StepUntilOperation *pOperation )
{
    if( currentStatus.state != ENGINE_STEPPING ) {
        PRINTF("currentStatus.state: Entering Single STEP_EDGE_DIRECTIVE!\r\n" );
        
        /* only for freestanding scale */
        if( getMyModel() != RT_GLOBAL_FSS ) {        
                skipMissingLabel_ = true;
                /* stepping complete */
                resetPrintEngineTimer();
                powerOffStepper();
                setNextOperation( &currentStatus );
        }

        currentStatus.state = ENGINE_STEPPING;
  
        /* notify host of state change */
        compareStatus( &currentStatus, &prevStatus ); 
        /* have the sensor ISR check and stop motor on the label edge */
        setMotorStopOnEdge();

        engine.steps = 0;    
        powerOnStepper();

        /*we are sizing a label, slow sizing speed to 3ips  */ 
        currentStatus.sensor |= MOTOR_FORWARD;
        currentStatus.sensor &= ~LABEL_TAKEN;
        currentStatus.sensor &= ~SYNCHRONIZED;
        
        setPrintEngineTimer( getSltSizingTime( engine.headType ) );
             
        engine.numSteps = getOpData(pOperation->type, pOperation->data);
        engine.direction = ( engine.numSteps < 0 ) ? BACKWARD_ : FORWARD_;    
        engine.numSteps = abs(engine.numSteps);
        engine.numSteps = engine.numSteps << 1;
        
        /* PRINTF( "number of steps allowed: %d\r\n",  engine.numSteps ); */
        /* initializeStepper( engine.direction ); */
        setStepperDirection( engine.direction );
        /* removed to keep motor from stalling at 6ips when sizing labels -- ats 07102014 */	     
        if( ( engine.headType == KYOCERA753_OHM ) || ( engine.headType == KYOCERA800_OHM ) ||            
            ( engine.headType == KYOCERA849_OHM ) ) {
            engine.steps++;
            halfStepMotor();          	
            motorStep( engine.direction, &currentStatus );
        }
    } else {
        /* if a freestanding scale then the media sensor has been removed */
        int mediaValue = getMediaCounts();
        
        if( mediaValue <= BACKING_PAPER_THRESHOLD  ) {
            
            if( ( currentStatus.sensor & MOTOR_FORWARD ) == MOTOR_FORWARD ){
                pCurrentLabel = pCurrentLabel->next;
                currentStatus.sensor |= SYNCHRONIZED;    
            } else {
                if( ( currentStatus.sensor & SYNC_BAR ) != SYNC_BAR ) {
                    currentStatus.sensor |= SYNC_BAR;  
                    currentStatus.sensor &= ~SYNCHRONIZED;
                    pCurrentLabel->next->position = 0;
                } else {
                    currentStatus.sensor &= ~SYNC_BAR_EDGE;
                }
                currentStatus.sensor |= SYNC_BAR;             
            }
        } else { 
            
            engine.outOfMediaCnt = 0; 
            if( mediaValue >= LABEL_EDGE_THRESHOLD ) {
                currentStatus.sensor &= ~SYNCHRONIZED;
            }
        }        
        
        if( engine.outOfMediaCnt < engine.maxMediaCount ) {
            currentStatus.sensor &= ~OUT_OF_MEDIA;
            currentStatus.error &= ~MEDIA_SHUTDOWN;
        } else {
            if( currentStatus.state == ENGINE_PRINTING || currentStatus.state == ENGINE_CALIBRATING )
                currentStatus.error |= MEDIA_SHUTDOWN;
            else        
                currentStatus.error &= ~MEDIA_SHUTDOWN;
        }
        
        /* read label taken sensor */
        if( !readLabelTakenSensor() ) {
            currentStatus.sensor |= LABEL_TAKEN;                
        } else {
            currentStatus.sensor &= ~LABEL_TAKEN;
        }
        
        /* check for error conditions. */
        if( currentStatus.error != NO_ERROR ) {
            /* return the printer to the idle state */    
            powerOffStepper();
            setOperation( IDLE_DIRECTIVE, &currentStatus ); 
        } else if ( ( testCondition( &currentStatus, 
                  pOperation->operator, 
                  pOperation->bits, 
                  pOperation->result) == false) 
                  &&  ( --engine.numSteps > 0 ) ) {    

            int even = 0;
            even = engine.numSteps & 0x0001;

            if( even == 0 ) {
                /* two half steps per print line */            
                motorStep( engine.direction, &currentStatus );
            }
            
            engine.steps++;
            halfStepMotor();
            /* notify host of any status change */
            compareStatus( &currentStatus, &prevStatus ); 
        } else {
            
            clrMotorStopOnEdge();
            
            PRINTF( "steps: %d\r\n",  engine.steps );
            PRINTF( "engine.numSteps: %d\r\n",  engine.numSteps );
            /* stepping complete */
            resetPrintEngineTimer();
            powerOffStepper();
                                    
            setNextOperation( &currentStatus );            
        }
    }	
}

/******************************************************************************/
/*!   \fn void testForSyncOp( StepOperation *pOperation )
      \brief
        This function tests whether we are at a sync bar or label edge.  
        The function will drive out 1/2" looking for a gap meaning we have 
        encountered a sync bar and have driven passed. The function will then 
        proceed to the label edge before exiting. If the function does not 
        encounter a gap then we have driven 1/2" into the label which we will 
        save the 1/2" distance to be used for our sizing measurement and the 
        exits.  
         
      \author
          Aaron Swift
*******************************************************************************/
void testForSyncOp( StepOperation *pOperation )
{
    static bool syncFound_ = false;
    static int index_ = 0;
    
    if (currentStatus.state != ENGINE_STEPPING ) {       
        PRINTF("currentStatus.state: Entering Single TEST_FOR_SYNC!\r\n" );  
                
        /* set the state and enable the stepper motor*/       
        currentStatus.state = ENGINE_STEPPING;
        
        engine.steps = 0;    
        powerOnStepper();
        
        /* freestanding scale */       
        index_ = 0;

        /*we are sizing a label, slow sizing speed to 3ips  */ 
        currentStatus.sensor |= MOTOR_FORWARD;
        currentStatus.sensor &= ~LABEL_TAKEN;
            
        setPrintEngineTimer( getSltSizingTime( engine.headType ) );
                     
        engine.numSteps = getOpData( pOperation->type, pOperation->data );
        engine.direction = ( engine.numSteps < 0 ) ? BACKWARD_ : FORWARD_;    
        engine.numSteps = abs(engine.numSteps);

        /* save steps value to be added back to total steps if sync bar not present */
        engine.stepsOffset = engine.numSteps; 
        
        /* printer has two half steps per print line */
        engine.numSteps = engine.numSteps << 1;

        /* have the sensor ISR check and stop motor on the gap */
        setMotorStopOnGap();
   
        setStepperDirection( FORWARD_ );
        powerOnStepper();
         
    } else {   

        /* if a freestanding scale then the media sensor has been removed */
        if( getMyModel() != RT_GLOBAL_FSS ) {
            /* return the printer to the idle state */    
            powerOffStepper();
            setOperation( IDLE_DIRECTIVE, &currentStatus );            
        } else {           
            int mediaValue = getMediaCounts();
            /* sizing debug - freestanding scale 
            if( index_ <= 1000 )
                pMediaSizing[index_++] = (unsigned char)mediaValue;
            */
            if( !syncFound_ ) {
                if( mediaValue <= BACKING_PAPER_THRESHOLD ) {    
                    /* sync bar found need to drive to edge */
                    syncFound_ = true;
                    /* clear our offset sync bar has been found */
                    engine.stepsOffset = 0;
                    /* add more distance so that we find the label edge */
                    engine.numSteps += 300; /* little over an inch should do it */
                    /* clear flag */
                    clrMotorStopOnGap();
                    /* set flag to stop motor on label edge*/
                    setMotorStopOnEdge();
                    /* turn power back on */
                    powerOnStepper();
                    PRINTF("testForSyncOp(): sync bar found!\r\n" ); 
                }
            }
            /* if we encountered the sync bar then look for the label edge */
            if( syncFound_ ) {
                if( mediaValue >= LABEL_EDGE_THRESHOLD ) {
                    /* clear our flag */                   
                    clrMotorStopOnEdge();
                    /* set our steps to zero */
                    engine.numSteps = 0;
                    PRINTF("testForSyncOp(): label edge found after sync bar!\r\n" ); 
                }
            }
        }

        /* check for error conditions. */
        if( currentStatus.error != NO_ERROR ) {
            /* return the printer to the idle state */    
            powerOffStepper();
            setOperation( IDLE_DIRECTIVE, &currentStatus ); 
        } else if ( engine.numSteps-- <= 0 ) {
            powerOffStepper();	
            /* sync the print timer to the motor steps. */
            resetPrintEngineTimer();                       
                       
            if( syncFound_ ) {
                syncFound_ = false;
#if 1                
                /* at the label edge */
                setNextOperation( &currentStatus ); 
#else   /* for backup */
                skipNextOperation( &currentStatus );            
#endif                
            } else {
                PRINTF("testForSyncOp(): no sync bar, label edge found!\r\n" ); 
                PRINTF("testForSyncOp(): engine.steps %d\r\n", engine.steps );
#if 1                
                /* skip next operation and continue with following operation. */
                skipNextOperation( &currentStatus );            
#else   /* for backup */
                setNextOperation( &currentStatus ); 
#endif                
            }
            /* sizing debug - freestanding scale 
            for( int i = 0; i < index_; i++ ) {
                PRINTF( "shoot counts: %d\r\n",  pMediaSizing[i] );
            }*/

        } else {
          
            int even = 0;

            even = engine.numSteps & 0x0001;
            if( even == 0 ) {
                /* Two half steps per printline */  
                motorStep( engine.direction, &currentStatus );
            }
            engine.steps++;
            halfStepMotor(); 
            /* notify host of any status change */
            compareStatus( &currentStatus, &prevStatus ); 
        }
    }
}

/******************************************************************************/
/*!   \fn void testForLabelOp( StepOperation *pOperation )
      \brief
        The function will drive out 1/2" looking for the label taken sensor 
        to be blocked. The function will then 
        proceed to the label edge before exiting. If the function does not 
        encounter a gap then we have driven 1/2" into the label which we will 
        save the 1/2" distance to be used for our sizing measurement and the 
        exits.  
         
      \author
          Aaron Swift
*******************************************************************************/
void testForLabelOp( StepOperation *pOperation )
{
    
    static bool labelFound_ = false;
    
    if( currentStatus.state != ENGINE_STEPPING ) {       
        PRINTF("currentStatus.state: Entering Single TEST_FOR_LABEL!\r\n" );  
        /* set the state and enable the stepper motor*/       
        currentStatus.state = ENGINE_STEPPING;
        
        powerOnStepper();
        
        /*we are sizing a label, slow sizing speed to 3ips  */ 
        currentStatus.sensor |= MOTOR_FORWARD;
        currentStatus.sensor &= ~LABEL_TAKEN;
            
        setPrintEngineTimer( getSltSizingTime( engine.headType ) );
                     
        engine.numSteps = getOpData( pOperation->type, pOperation->data );
        engine.direction = ( engine.numSteps < 0 ) ? BACKWARD_ : FORWARD_;    
        engine.numSteps = abs(engine.numSteps);

        /* save steps value to be added back to total steps if sync bar not present */
        engine.stepsOffset = engine.numSteps; 
        
        /* printer has two half steps per print line */
        engine.numSteps = engine.numSteps << 1;

        /* have the sensor ISR check and stop motor on the gap */
        setMotorStopOnGap();
   
        setStepperDirection( FORWARD_ );
        powerOnStepper();
         
    } else {   
        
        if( readLabelTakenSensor() ) {
            if( !labelFound_ ) {
                labelFound_ = true;
                /* add more distance to peel the label */
                engine.numSteps += 410; /* 510 little over 2 1/2" should do it */
                /* turn power back on */
                powerOnStepper();
                PRINTF("testForLabelOp(): label at sensor!\r\n" );             
            }
        }
        /* check to see if we have a very short label 1.75" */
        int mediaValue = getMediaCounts();   
        if( mediaValue <= BACKING_PAPER_THRESHOLD  ) {
            engine.numSteps = 0;
        }        
        /* check for error conditions. */
        if( currentStatus.error != NO_ERROR ) {
            /* return the printer to the idle state */    
            powerOffStepper();
            setOperation( IDLE_DIRECTIVE, &currentStatus ); 
        } else if ( engine.numSteps-- <= 0 ) {
            powerOffStepper();	
            /* sync the print timer to the motor steps. */
            resetPrintEngineTimer();                       
                       
            if( labelFound_ ) {
                labelFound_ = false;
                /* label ready to be taken */
                setNextOperation( &currentStatus ); 
            } else {
                PRINTF("testForLabelOp(): no label taken trip!\r\n" ); 
                PRINTF("testForLabelOp(): engine.steps %d\r\n", engine.steps );
                /* skip next operation and continue with following operation. */
                skipNextOperation( &currentStatus );            
            }
        } else {
          
            int even = 0;

            even = engine.numSteps & 0x0001;
            if( even == 0 ) {
                /* Two half steps per printline */  
                motorStep( engine.direction, &currentStatus );
            }
            engine.steps++;
            halfStepMotor(); 
            /* notify host of any status change */
            compareStatus( &currentStatus, &prevStatus ); 
        }
    }    
}

/******************************************************************************/
/*!   \fn void waitOp( WaitOperation *pOperation )
      \brief
        This function waits a specified amount of time (in mS) until executing
        the next operation. Note: FlexTimer runs at a rate of 400uS even timing 
        values will be off (under) by 400uS. 
      \author
          Aaron Swift
*******************************************************************************/
void waitOp( WaitOperation *pOperation )
{  
    static int waitCount_ = 0;
    if( currentStatus.state != ENGINE_WAITING ) {
        PRINTF("currentStatus.state: Entering Single ENGINE_WAITING!\r\n" );
      
        currentStatus.state = ENGINE_WAITING;
        /* get the wait time (mS) */
        waitCount_ = getOpData(pOperation->type, pOperation->data);               
        
        /* adjust count for 400uS timer rate */
        if( waitCount_ >= 10 ) {
            if( waitCount_ / 2 )
                waitCount_ = ( waitCount_ * 2 ) + 4;
            else 
              waitCount_ = ( waitCount_ * 2 ) - 4; 
        } else {
            if( waitCount_ / 2 )
                waitCount_ = ( waitCount_ * 2 ) + 1;                        
            else 
                waitCount_ = ( waitCount_ * 2 ) - 1;                        
        }         
    } else {           
        /* check for error conditions. */
        if ( currentStatus.error != NO_ERROR ) {
            powerOffStepper();            
            setOperation( IDLE_DIRECTIVE, &currentStatus ); 
        } else  {  
            /* notify host of any status change */
            compareStatus( &currentStatus, &prevStatus );             
            if( waitCount_ <= 0 ) {
              setNextOperation(  &currentStatus );         
            } else {
                waitCount_--;
            }   
        }
    }
}

/******************************************************************************/
/*!   \fn void waitUntilOp(  WaitUntilOperation *pOperation  )
      \brief
        This function handles the teach table wait until event operation.
        
      \author
          Aaron Swift
*******************************************************************************/
void waitUntilOp(  WaitUntilOperation *pOperation  )
{
    if ( currentStatus.state != ENGINE_WAITING ) {
        PRINTF("currentStatus.state: Entering ENGINE_WAITING_UNTIL!\r\n" );
        
         currentStatus.state = ENGINE_WAITING;
    } else {
              
        /* check for error conditions. */
        if( currentStatus.error != NULL ) {
            powerOffStepper();           
	    /* return the printer to the idle state */
            setOperation( IDLE_DIRECTIVE, &currentStatus ); 
        } else {   
            /* notify host of any status change */
            compareStatus( &currentStatus, &prevStatus ); 
            //sof-5066
            if( getMyModel() == RT_GLOBAL_FSS ) {
                /* cutter interlock open? */
                if( !isCutterInterlockClosed() ) {
                    /* return the printer to the idle state */
                    setOperation( IDLE_DIRECTIVE, &currentStatus );                                     
                }
            }
            if( testCondition( &currentStatus,
                           pOperation->operator, 
                           pOperation->bits, 
                           pOperation->result ) == true ) {
                setNextOperation( &currentStatus );
            }
        }
    }       
}

/******************************************************************************/
/*!   \fn void waitUntilSizingOp(  WaitUntilOperation *pOperation  )
      \brief
        This function handles the teach table wait until event operation during
        sizing for FSSS.
        
      \author
          Aaron Swift
*******************************************************************************/
void waitUntilSizingOp(  WaitUntilOperation *pOperation  )
{
    /* SOF-5184 */
    static bool interlock_      = false;
    static bool sigCutter_      = false;
    /* SOF-5301 */
    static bool backUp_         = false;
    static bool waitForLabel_   = false;
    
    if ( currentStatus.state != ENGINE_WAITING ) {
        PRINTF("currentStatus.state: Entering ENGINE_WAITING_UNTIL!\r\n" );
        /* if we detected continuous stock then fire the knife. */
        if( continuousStock_ ) {
            //SOF-5184
            interlock_ = isCutterInterlockClosed();
            if( interlock_ ) {
                if( pCutSemaphore != NULL ) {
                    /* make sure the semaphore is empty */
                    xSemaphoreTakeFromISR( pCutDoneSemaphore, pdFALSE );
                    /* called from an ISR so user the ISR give */
                    if( xSemaphoreGiveFromISR( pCutSemaphore, pdFALSE ) != pdTRUE ) {
                        PRINTF("waitUntilSizingOp(): Failed to give cutter Semaphore");
                    } else {
                        sigCutter_ = true;
                    }
                } else {
                    PRINTF("waitUntilSizingOp(): pCutSemaphore is NULL!\r\n" );
                }
            } else {
                PRINTF("waitUntilSizingOp(): interlock is open!\r\n" );
            }
        }        
        currentStatus.state = ENGINE_WAITING;
    } else {
        if( continuousStock_ ) {
            //SOF-5184
            interlock_ = isCutterInterlockClosed();        
            if(  interlock_ ) { 
            /* if we have not signaled the cutter to fire then do it now */ 
                if( !sigCutter_ ) {
                    if( pCutSemaphore != NULL ) {                
                        xSemaphoreTakeFromISR( pCutDoneSemaphore, pdFALSE );                    
                        if( xSemaphoreGiveFromISR( pCutSemaphore, pdFALSE ) != pdTRUE ){
                            PRINTF("waitUntilSizingOp(): Failed to give cutter Semaphore 2");
                        } else {
                            sigCutter_ = true;
                        }
                    } else {
                        PRINTF("waitUntilSizingOp(): pCutSemaphore is NULL!\r\n" );
                    }
                } else {
                    /* SOF-5301 */
                    if( ( !backUp_ ) && ( !waitForLabel_ ) ) {
                        /* wait for the cutter to be done with the cut */
                        if( xSemaphoreTakeFromISR( pCutDoneSemaphore, pdFALSE ) == pdTRUE ){
                            /* backup the label 1/4" to prevent jamming */
                            engine.numSteps = ( 75 * 2 );
                            engine.direction = BACKWARD_;    
                            backUp_ = true;
                            
                            setStepperDirection( engine.direction );
                            powerOnStepper();
                        }
                    }
                    if( backUp_ ) {                    
                        if ( engine.numSteps-- <= 0 ) {
                            powerOffStepper();	
                            /* sync the print timer to the motor steps. */
                            resetPrintEngineTimer();                       
                            
                            backUp_ = false;
                            waitForLabel_ = true;                            
                        } else {          
                            int even = 0;
                            even = engine.numSteps & 0x0001;
                            if( even == 0 ) {
                                /* Two half steps per printline */  
                                motorStep( engine.direction, &currentStatus );
                            }                            
                            halfStepMotor(); 
                        }                    
                    }                    
                }
            }    
        }           
        /* check for error conditions. */
        if( currentStatus.error != NULL ) {
            powerOffStepper();           
            /* return the printer to the idle state */
            setOperation( IDLE_DIRECTIVE, &currentStatus ); 
        } else {
            if( ( waitForLabel_ ) && ( continuousStock_ ) ) {       /* SOF-5301 */
                /* notify host of any status change */
                compareStatus( &currentStatus, &prevStatus ); 
                if( testCondition( &currentStatus,
                               pOperation->operator, 
                               pOperation->bits, 
                               pOperation->result ) == true ) {
                    sigCutter_ = false;
                    waitForLabel_ = false;
                    setNextOperation( &currentStatus );                    
                } 
            } else {
                /* notify host of any status change */
                compareStatus( &currentStatus, &prevStatus ); 
                if( testCondition( &currentStatus,
                               pOperation->operator, 
                               pOperation->bits, 
                               pOperation->result ) == true ) {
                    setNextOperation( &currentStatus );                                
                }
            }
        }
    }      
}

/******************************************************************************/
/*!   \fn void testOp( TestOperation *pOperation )
      \brief
        This function handles the teach table test command operation.
        
      \author
          Aaron Swift
*******************************************************************************/
void testOp( TestOperation *pOperation )
{
    PRINTF("currentStatus.state: Entering ENGINE_TEST!\r\n" );
    if( testCondition( &currentStatus,
                    pOperation->operator, 
                    pOperation->bits, 
                    pOperation->result ) == true ) {
        if( !skipMissingLabel_ ) {              
            currentStatus.user |= pOperation->true_bits; 
            /* check to make sure we are not setting a ghost missing label! */
            if( ( ( currentStatus.user & MISSING_LABEL ) == MISSING_LABEL ) && getGhostMCntr( ) != 0 ) {
                currentStatus.user &= ~MISSING_LABEL;
            }
        } else {
            PRINTF("*********************Skip setting missing label bit -- true ********************\r\n" );
            currentStatus.user = 0;
            skipMissingLabel_ = false;
        }
        sendPrStatus( &currentStatus, true );
        if( pOperation->true_action == CONTINUE_EXECUTION) {
            setNextOperation( &currentStatus );
        } else {
            setOperation( IDLE_DIRECTIVE, &currentStatus ); 
        }
    } else {
        if( !skipMissingLabel_ ) {              
            currentStatus.user |= pOperation->false_bits;
        } else {
            PRINTF("*********************Skip setting missing label bit -- false ********************\r\n" );
            currentStatus.user = 0;
            skipMissingLabel_ = false;
        }
        if( pOperation->false_action == CONTINUE_EXECUTION ) {
            setNextOperation( &currentStatus );
        } else {
            setOperation( IDLE_DIRECTIVE, &currentStatus );
            
            engine.steps = SHRT_MAX;
        }
    }
}

/******************************************************************************/
/*!   \fn void statusOp( StatusOperation *pOperation )
      \brief
        This function handles the teach table status command operation.
        
      \author
          Aaron Swift
*******************************************************************************/
void statusOp( StatusOperation *pOperation )
{   
    PRINTF("currentStatus.state: Entering ENGINE_STATUS!\r\n" );
    if( pOperation->operator == SET_BITS ) {
        currentStatus.user |= pOperation->bits;
        if( currentStatus.user == MISSING_LABEL ) {
            //PRINTF("currentStatus.user == MISSING_LABEL\r\n" );
            currentStatus.user = 0; 
        } else if( currentStatus.user == JAMMED_LABEL ) {
            PRINTF("currentStatus.user == JAMMED_LABEL\r\n" );
        }
    }
    else if( pOperation->operator == CLEAR_BITS ) {
        currentStatus.user &= ~pOperation->bits;
    }
    setNextOperation( &currentStatus );
}

/******************************************************************************/
/*!   \fn void counterOp( CounterOperation *pOperation )
      \brief
        This function handles the teach table counter command operation.
        
      \author
          Aaron Swift
*******************************************************************************/
void counterOp( CounterOperation *pOperation )
{    
    PRINTF("currentStatus.state: Entering ENGINE_COUNTER!\r\n" );
    switch( pOperation->operator )
    {
        case RESET_COUNTER: {
            PRINTF("counterOp(): reset counter!\r\n" );
            currentStatus.counter = 0;
            engine.steps = 0;
            break;
        }
        case ENABLE_COUNTER: {
            PRINTF("counterOp(): enable counter\r\n" );
            PRINTF("counterOp(): currentStatus.counter: %d\r\n", currentStatus.counter );
            stepCounterEnabled_ = true;
            break;
        }
        case DISABLE_COUNTER: {
            PRINTF("counterOp(): disable counter\r\n" );
            PRINTF("counterOp(): half steps currentStatus.counter: %d\r\n", engine.steps );
            /* we are keeping track of half steps and sizing needs whole steps. */
            currentStatus.counter = (  engine.steps / 2 );

            /* if we saved off steps due to sync bar test then add it back into grand total.
            if( engine.stepsOffset != 0 ) {
                PRINTF("counterOp(): whole steps offset: %d\r\n", engine.stepsOffset );
                currentStatus.counter +=  calculateSizingOffset(currentStatus.counter); 
                engine.stepsOffset = 0;
            } */
            PRINTF("counterOp(): whole steps currentStatus.counter: %d\r\n", currentStatus.counter );
            stepCounterEnabled_ = false;
            break;
        }
    }
    setNextOperation( &currentStatus );
}

/******************************************************************************/
/*!   \fn void calibrateOp( CmdOp *pOperation )
      \brief
        This function start with the PWM set to maximum current (99.6%) and  
        slowly reduce the PWM to the minimum current (4.0%) or until the average 
        slope is within the desired tolerance.
      \author
          Aaron Swift
*******************************************************************************/
void calibrateOp( CmdOp *pOperation )
{
#if 0 /* TO DO: convert to LP5521 if needed for media sensor reflective */    
    static unsigned char calDutyCycle = DC_97_PERCENT; 
    FPMBLC3Checksums    checkSums;
    
    if( currentStatus.state != ENGINE_CALIBRATING ) {
      
        calDutyCycle = DC_97_PERCENT;     
        currentStatus.state = ENGINE_CALIBRATING;      
        /* notify host of state change */
        compareStatus( &currentStatus, &prevStatus ); 

        setSamplesCntMedia( 10 );
        
        setDutyCycleIsrDS1050( DS1050_MS_DEVICE_ADDR, calDutyCycle );
    } else {
      
        /* notify host of any status changes */
        compareStatus( &currentStatus, &prevStatus ); 

        /* wait for our sample to be ready */
        if(  isMediaSampleReady() ) {
           
            unsigned long cnts = getMediaCounts();
            /* media sensor threshold 1.0v */
            if( cnts >= MEDIA_CAL_VOLTAGE ) {   
                /* set to just above saturation */  
                calDutyCycle += 1; 
        
                PRINTF("calibrateOp(): cal duty cycle: %fKhz \r\n", calDutyCycle * DUTYCYCLE_RESOLUTION ); 
                PRINTF("calibrateOp(): cal count: %d \r\n", calDutyCycle ); 
                PRINTF("calibrateOp(): cal voltage: %fV \r\n", cnts * AD_RESOLUTION );
                PRINTF("calibrateOp(): cal counts: %d \r\n", cnts );
              
                /* save the media sensor adjustment to configuration */
                config_.media_sensor_adjustment = calDutyCycle;
                setSerialPrConfiguration( &config_ );
                
                /* read the checksum section to maintain weigher checksum */
                getPageChecksums( &checkSums );
                
                /* calculate new checksum for printer section */
                checkSums.prConfigSum = calculateChecksum ( &config_, sizeof (Pr_Config) );
                setPageChecksums(&checkSums);
                
                /* turn off sampling */
                clearSampleCntMedia();
                
                /* return to idle */
                setOperation( IDLE_DIRECTIVE, &currentStatus );               
            } else {
                /* adjust the PWM duty cycle by 3.125% and retest */
                calDutyCycle--;                            
                setSamplesCntMedia( 10 );
                setDutyCycleIsrDS1050( DS1050_MS_DEVICE_ADDR, calDutyCycle );
            }
            /* break out on error */
            if( calDutyCycle == PWM_0_PRECENT_DUTY) {
                /* return to idle */
                setOperation( IDLE_DIRECTIVE, &currentStatus );                           
            }
        }
    }
#endif    
}

/******************************************************************************/
/*!   \fn void disableOp( CmdOp *pOperation )
      \brief
        This function handles the teach table disable command operation.
        
      \author
          Aaron Swift
*******************************************************************************/              
void disableOp( CmdOp *pOperation )
{
              
    if ( currentStatus.state != ENGINE_DISABLED) {
        PRINTF("currentStatus.state: Entering ENGINE_DISABLE!\r\n" );
      /* Set the status bits to show that the printer has      
           just returned to the idle state after the completion  
           one of the commands. The initial command bits that    
           made the printer leave printer idle are still set at  
           this time to indicate what the command was that was   
           just completed. */            
        currentStatus.state = ENGINE_DISABLED;           
        currentStatus.command |= COMMAND_COMPLETE;
        
        /* notify host the printer is disabled. */
        compareStatus( &currentStatus, &prevStatus );
        
        /* perform some initialization. */        
        currentStatus.command &= ~COMMAND_COMPLETE;
        currentStatus.user    = 0;
        
        shutdownPrintEngine();
    }
    
    /* clear the command queue */
    clearCmdQueue();
}

/******************************************************************************/
/*!   \fn void cutOp( WaitOperation *pOperation )
      \brief
        This function signals the cutter to cut the label
      \author
          Eric Landes
*******************************************************************************/
void cutOp( GenericOperation *pOperation )
{  
    if( currentStatus.state != ENGINE_CUTTING ) {
        PRINTF("currentStatus.state: Entering Single ENGINE_CUTTING!\r\n" );
      
        currentStatus.state = ENGINE_CUTTING;
        /* tell the Printer task to cut */
        if(pCutSemaphore != NULL){
            /* make sure the semaphore is empty */
            xSemaphoreTakeFromISR(pCutDoneSemaphore,pdFALSE);
            /* called from an ISR so user the ISR give */
            if(xSemaphoreGiveFromISR( pCutSemaphore, pdFALSE) != pdTRUE){
                PRINTF("cutOp(): Failed to give cutter Semaphore");
            }
        }     
    } else {           
        /* wait until the cutter is done */
        if(xSemaphoreTakeFromISR(pCutDoneSemaphore,pdFALSE) == pdTRUE){
            setNextOperation( &currentStatus );
        }
    }
}


/******************************************************************************/
/*!   \fn void clearCmdQueue( void )
      \brief
        This function clears the printer command queue.
        
      \author
          Aaron Swift
*******************************************************************************/              
void clearCmdQueue( void )
{
    /* clear the command queue */
    xQueueReset( pCmdQHandler_ );  
}

/******************************************************************************/
/*!   \fn void clearLabelImageBuffer( void )
      \brief
        This function clears the printer label image buffer.
        
      \author
          Aaron Swift
*******************************************************************************/              
void clearLabelImageBuffer( void )
{
    unsigned char *pBuffer = getImageBuffer();
    if( pBuffer != NULL) {
        memset( pBuffer, 0, PRINTER_BUFFER_SIZE );    
    } else {
        PRINTF("clearLabelImageBuffer(): Error: pBuffer is null!\r\n" );       
    }
}

/* added for freestanding scale shoot through sensor */
/******************************************************************************/
/*!   \fn int calculateSizingOffset( unsigned int wSteps )
      \brief
        This function will adjust the final label sizing value by an offset 
        amount. This value is based on the charts within the new shoot 
        through algorithm spreadsheet found in the docs section of this project.        
        
      \author
          Aaron Swift
*******************************************************************************/              
int calculateSizingOffset( unsigned int wSteps )
{
    int offset = 0;
    if( ( wSteps >= LOW_RANGE_START_SIZING ) && ( wSteps <= LOW_RANGE_END_SIZING ) ) {
        offset = LOW_RANGE_OFFSET;
    } else if( ( wSteps >= MID_RANGE_START_SIZING ) && ( wSteps <= MID_RANGE_END_SIZING ) ) {
        offset = MID_RANGE_OFFSET;
    } else if( ( wSteps >= HIGH_RANGE_START_SIZING ) && ( wSteps <= HIGH_RANGE_END_SIZING ) ) {
        offset = HIGH_RANGE_OFFSET;
    }
    return offset;
}

/******************************************************************************/
/*!   \fn void setContinuousStock( void )
      \brief
        This function set the flag for continuous stock. This flag signals use 
        when to use tracking for adjusting for die cut labels slip and stalling
        which is not important on continuous stock.

      \author
          Aaron Swift
*******************************************************************************/              
void setContinuousStock( void )
{
    continuousStock_ = true;    
}


void printerTests()
{
    static int testCnt_ = 1; 
    
    /****************** test print engine command operation *******************/
    #if 1
    QueueHandle_t pQHandler_ = NULL; //= getPrCommandQueueHandle();
    if( pQHandler_ != 0 ) {
        switch( testCnt_ ) 
        {
            /* test sizing label */
            case 1: {
                if( engine.currentCmd.generic.directive == NOOPERATION_DIRECTIVE ) {
                    PrCommand cmdMsg;
                    cmdMsg.identifier = _TBL_SIZE_LABELS;  /* size command */
                    cmdMsg.options = 0;             /* no security label */ 
                    cmdMsg.data_item = 0;           /* no indirection */ 
                    cmdMsg.value = 0;               

                    BaseType_t result = xQueueSendToBack( pQHandler_, (void *)&cmdMsg, 0 );
                    if( result == pdTRUE ) {
                        testCnt_++;
                    } else {
                        PRINTF("printerTests(): PR Command Queue full! \r\n");  
                    }
                } else {
                    PRINTF("printerTests(): PR Engine busy! \r\n");  
                }
                break;
            }
            case 2: {
                /* wait for sizing to complete */
                if( currentStatus.state == ENGINE_IDLE ) {
                    testCnt_++;
                }
                break;
            }
            case 3: {
                break;
            }
            default : {
                break;
            }
        }
    } else {
        PRINTF("printerTests(): command queque null!\r\n" );  
    } 
    #endif
    
  
    /********************* test print engine flex timers **********************/
    #if 0
    unsigned char tempIndex = getHeadTemp();

    engine.headType = getPrintHeadType();
    /* get the line compensation levels */
    engine.levels = getCompLevel( engine.headType );
    /* get the over all line burn time */
    engine.sltTime = getSltTime( engine.headType, 3 );

    engine.contrast = 3;
    engine.sltHalfTime = engine.sltTime / 2;
    engine.lineCounter = 0;
    engine.burnCounter = 0;

    engine.numSteps = 0;
    engine.numPrintLines = 0;
    engine.direction = FORWARD_;
    engine.labelOrientation = HEEL_FIRST;  
    
    /* setup the start times for history, adjacency and current line */
    if( engine.levels == 3 ) {
        engine.histAdj[0].compType = FIRST_LEVEL_HIST;
        engine.histAdj[0].time = getHistoryTime( tempIndex );
        engine.histAdj[1].compType = FIRST_LEVEL_ADJ;
        engine.histAdj[1].time = getAdjacencyTime( tempIndex );
        engine.histAdj[2].compType = CURRENT_LINE;
        engine.histAdj[2].time = getCurrentLineTime( tempIndex );

        engine.pwmStartTime = getPwmStartTime( tempIndex );
        engine.pwmDutyCycle = getPwmDutyCycle( tempIndex );
    } else {
        engine.histAdj[0].compType = FIRST_LEVEL_HIST;
        engine.histAdj[0].time = getHistoryTime( tempIndex );
        engine.histAdj[1].compType = CURRENT_LINE;
        engine.histAdj[1].time = getCurrentLineTime( tempIndex );

        engine.pwmStartTime = getPwmStartTime( tempIndex );
        engine.pwmDutyCycle = getPwmDutyCycle( tempIndex );
    }

    /* test test label creation */
    createCheckerBoardLabel( 0, STEPS_PER_LENGTH3_50 ); 
    startLineTimer();
    #endif  
  
    
    /**************** test printer media and label taken PWM ******************/
    #if 0
    /* retrieve FTM default configuration info */
    ftm_config_t ftmInfo;
    FTM_GetDefaultConfig(&ftmInfo);

    /* initialize FTM module */
    FTM_Init(LTS_MS_FTM_BASE, &ftmInfo);

    ftm_chnl_pwm_signal_param_t ms_pwm_ftmParam;

    /* Configure Media Sensor FTM parameters */
    ms_pwm_ftmParam.chnlNumber = MS_PWM_CHANNEL;
    ms_pwm_ftmParam.level = MS_PWM_POLARITY;
    ms_pwm_ftmParam.dutyCyclePercent = MS_PWM_DEFAULT_DUTY_CYCLE;
    ms_pwm_ftmParam.firstEdgeDelayPercent = 0U;

    FTM_SetupPwm( LTS_MS_FTM_BASE, &ms_pwm_ftmParam, 1U, MS_PWM_ALIGNMENT, MS_PWM_Hz,  LTS_MS_FTM_SOURCE_CLOCK);  

    ftm_chnl_pwm_signal_param_t lts_ftmParam;

    /* Configure LTS FTM parameters */
    lts_ftmParam.chnlNumber = LTS_CLK_CHANNEL;
    lts_ftmParam.level = LTS_CLK_POLARITY;
    lts_ftmParam.dutyCyclePercent = LTS_DEFAULT_DUTY_CYCLE;
    lts_ftmParam.firstEdgeDelayPercent = 0U;


    FTM_SetupPwm(LTS_MS_FTM_BASE, &lts_ftmParam, 1U, LTS_CLK_ALIGNMENT, LTS_CLK_Hz, LTS_MS_FTM_SOURCE_CLOCK);

     /* start the FTM module timer */
    FTM_StartTimer(LTS_MS_FTM_BASE, kFTM_SystemClock);
    #endif    
   
}

int getOutOfMediaCounts( void ){
    return engine.outOfMediaCnt;
}

void setOutOfMediaCounts(int val){
    engine.outOfMediaCnt = val;
}

int getMaxOutOfMediaCounts( void ){
    return engine.maxMediaCount;
}

void setHistoryEnabled(bool enabled){
    historyEnabled_ = enabled;
}

