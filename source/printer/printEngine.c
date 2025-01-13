#include <stdlib.h>
#include <limits.h>
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_lpspi.h"
#include "fsl_lpspi_edma.h"
#include "fsl_common_arm.h"
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
#include "takeupMotor.h"
#include "fsl_debug_console.h"
#include "lp5521.h"
#include "translator.h"
#include "TPHMotor.h"
#include "globalPrinterTask.h"
#include "virtualScope.h"
#include "developmentSettings.h"
#include "idleTask.h"

#define SHOOT_COUNT_ARRAY_SIZE 10000

#define GAP_SENSOR_TO_PEEL_BAR_GT 900
#define GAP_SENSOR_TO_PEEL_BAR_HT 840
#define GAP_SENSOR_TO_TEAR_BAR_GT 995
#define GAP_SENSOR_TO_TEAR_BAR_HT 940

PrintEngine         engine;
bool sample_   = false;
bool historyEnabled_ = true;
static bool stepGapOnce_ = false;
static bool continuousBootFlag = false;
static int contBootTimeout = 0;
static bool notSizing = true;
static bool deviationFlag = false;

int* shootCounts;
int maxDeviationFromGapPlus = 0;
int maxDeviationFromGapMinus = 0;

#define LABEL_LOW_ARRAY_SIZE 2500
#define LABEL_LOW_HISTORY_SIZE 2500
static int labelLowAverageCurrent[1] = {0};
static int labelLowAverageHistory[10] = {0};
char labelLowSampleCounter = 0;
int labelLowIndexOffset = 0;
int labelLowCounts[LABEL_LOW_ARRAY_SIZE] = {0};
int labelLowCountsHistory[LABEL_LOW_HISTORY_SIZE] = {0};
int labelLowCountsMini[1000] = {0};
int labelLowBigDipCount = 0;

int currentLabelSize = 0;
int firstLabelGap = 0;
char labelGapOnce_ = 0;
bool labelGapOnceChanged = false;
bool lineTimerOnce_ = false;
uint16_t prevLineCounter = 0;
uint16_t currLineCounter = 0;
int verticalOffsetIndex = 0;
int offsetThisPrint = 0;
uint32_t LTWaitCount_ = 0;
uint32_t sizingLTTimerCount_ = 0;

bool powerTimeout = false;
uint32_t powerTimeoutTicks = 0;
uint8_t tempDelayCounter = 0;

uint16_t prevLabelSize = 0;

int leadInSteps = 0;
bool leadInDone = false;
int expelSteps = 0;
bool expelDone = false;
bool jammedBefore = false;

uint16_t sizingWaitCount = 0;

static uint16_t dieCutExpelLength = 100;
static uint16_t dieCutExpelLengthNoPaper = 170;
static bool PEModeBackwindFlag = false;
static bool firstSize = true;
static int backwindOffsetCount;
static bool sizingAlreadyDone = false;

//unsigned char histAdjLines[MAX_HIST_ADJ_LEVELS][PRINTER_HEAD_SIZE];

static bool stepCounterEnabled_         = false;
static bool skipMissingLabel_           = false;
static bool skipLabelTaken_             = false;
static bool backwindAfterSizing         = false;

/* buffers for test labels */
#if 1
static unsigned char pattern1[(PRINTER_HEAD_SIZE_80MM * 2 )];
static unsigned char pattern2[(PRINTER_HEAD_SIZE_80MM * 2 )];
static unsigned char pattern3[(PRINTER_HEAD_SIZE_80MM * 2 )];
#else
AT_QUICKACCESS_SECTION_DATA( static unsigned char pattern1[( 72 * 2 )] ) = {0};
AT_QUICKACCESS_SECTION_DATA( static unsigned char pattern2[( 72 * 2 )] );
#endif
#if 1   /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
lpspi_master_edma_handle_t spiHeadMasterHandle;
#else 
AT_QUICKACCESS_SECTION_DATA( lpspi_master_handle_t spiHeadMasterHandle ); 
#endif


AT_QUICKACCESS_SECTION_DATA( LabelPosition   label0 );
AT_QUICKACCESS_SECTION_DATA( LabelPosition   label1 );
AT_QUICKACCESS_SECTION_DATA( LabelPosition   label2 );
AT_QUICKACCESS_SECTION_DATA( LabelPosition   *pCurrentLabel );

AT_NONCACHEABLE_SECTION_INIT( static SemaphoreHandle_t pCutSemaphore ) = NULL;
AT_NONCACHEABLE_SECTION_INIT( static bool applyVerticalOffset_ )       = false;
AT_NONCACHEABLE_SECTION_INIT( static short prevVertOffset_ )           = 0;

/* command queue */
AT_NONCACHEABLE_SECTION_INIT( static QueueHandle_t pCmdQHandler_ )     = NULL;

/* line timer interrupt level for print operation and dotwear */
static unsigned int interruptLevel_ = 1; 

bool continuousStock_                   = false;
bool mainMotorStopped_                  = false;

#if 1   /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
void LPSPI_MasterUserCallback( LPSPI_Type *base, lpspi_master_edma_handle_t *handle, status_t status, void *userData );
#else 
AT_QUICKACCESS_SECTION_CODE( void LPSPI_MasterUserCallback( LPSPI_Type *base, lpspi_master_handle_t *handle, status_t status, void *userData ) );
#endif

AT_NONCACHEABLE_SECTION_INIT( volatile bool spiDmaTransferComplete ) = false;
AT_QUICKACCESS_SECTION_DATA( edma_handle_t eDmaHandle_0 );
AT_QUICKACCESS_SECTION_DATA( edma_handle_t eDmaHandle_1 );
AT_QUICKACCESS_SECTION_DATA( edma_handle_t eDmaHandle_2 );

/* printer status tracking */
AT_QUICKACCESS_SECTION_DATA( PrStatusInfo currentStatus );
AT_QUICKACCESS_SECTION_DATA( PrStatusInfo prevStatus );

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
extern bool startLabelTakenTimer( void );

static uint16_t tModifier = 0;

#if 0 /* To Do */
extern HeadSensors  headSensors;
#endif

extern Pr_Config config_;
extern SemaphoreHandle_t pCutDoneSemaphore;
extern volatile ADCManager adcManager;

/*** Chris add this to log shoot through sensor readings during stepGapOp() 
unsigned short ltIndex = 0;
unsigned short ltCounts[4000] = { 0 }; 
***/

int labelAlignment;
bool labelPositionedAfterSizing = true;
bool sizingLabels = false;
int backwindOffset = 0;
char printMovedUp = 0;
char printMovedDown = 0;

uint16_t leadInStepTarget = 0;

uint16_t streamingLabelBackwind = 0;

int shootIndex = 0;

bool paused_ = false;

unsigned short getNumPrintLinesLeft(void)
{
   return engine.numPrintLines;
}

void delay( unsigned long time );
extern void testPrintHeadTransfer( void );

uint16_t TPHStepsPastGapThisPrint = 0;

uint16_t TUSensorTestValueBase = 0;
uint16_t SHOOTSensorTestValueBase = 0;
uint16_t LOWSTOCKSensorTestValueBase = 0;

uint16_t TUSensorTestValue = 0;
uint16_t SHOOTSensorTestValue = 0;
uint16_t LOWSTOCKSensorTestValue = 0;


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
        pTakeup = pvPortMalloc( 1000 ); */
 
        /* assign our command queue */
        pCmdQHandler_ = pHandle;
        /* get the head temperature index */
        unsigned char tempIndex = getPrintheadTemperatureInCelsius();

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
#if 1
        /* intialize stepper motor */
        initializeStepper( engine.direction );
        
        /* initialize the paper takeup motor control 
        initTakeupIntr();       */
        
        /* gets rid of whine before running the stepper */
        powerOffStepper();
#endif        
        /* set the operational directive to idle operation */
        engine.currentCmd.generic.directive =  IDLE_DIRECTIVE;
        
        /* setup the start times for history, adjacency and current line */        
        //PRINTF("initializePrintEngine() - engine.levels == %d\r\n", engine.levels);
        
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
        
        backwindOffsetCount = 0;
        
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
    
    #if 0       /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
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
        masterConfig.whichPcs                      = kLPSPI_Pcs0;
        masterConfig.pcsActiveHighOrLow            = kLPSPI_PcsActiveLow;
        masterConfig.pinCfg                        = kLPSPI_SdiInSdoOut;    
        masterConfig.dataOutConfig                 = kLpspiDataOutTristate;

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
        masterConfig.bitsPerFrame                  = 8;       
        masterConfig.cpol                          = kLPSPI_ClockPolarityActiveHigh;        
        masterConfig.cpha                          = kLPSPI_ClockPhaseFirstEdge;
        if( pEngine->labelOrientation ==  HEAD_FIRST ) {
            masterConfig.direction                 = kLPSPI_LsbFirst; 
        } else {
            masterConfig.direction                 =  kLPSPI_MsbFirst;
        }
        masterConfig.pcsToSckDelayInNanoSec        = 50;
        masterConfig.lastSckToPcsDelayInNanoSec    = 50;
        masterConfig.betweenTransferDelayInNanoSec = 50;
        masterConfig.whichPcs                      = kLPSPI_Pcs0;
        masterConfig.pcsActiveHighOrLow            = kLPSPI_PcsActiveLow;
        masterConfig.pinCfg                        = kLPSPI_SdiInSdoOut;    
        masterConfig.dataOutConfig                 = kLpspiDataOutTristate;

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
#if 1 /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
void LPSPI_MasterUserCallback( LPSPI_Type *base, lpspi_master_edma_handle_t *handle, status_t status, void *userData )
#else 
void LPSPI_MasterUserCallback( LPSPI_Type *base, lpspi_master_handle_t *handle, status_t status, void *userData )
#endif
{
    /* assert data latch only on history loads */
    if( !isCurrentLine() ) {
        /* data is transfered to the head, assert the data latch */ 
        GPIO_WritePinOutput( PHEAD_LATCH_GPIO, PHEAD_LATCH_PIN, false );         //false
        
        /* load history line*/
        //historyAdjacency();
        //loadHistory();
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
/*!   \fn void setSkipLabelTakenCheck( void )

      \brief
        This function set the skip label taken flag. Flag is used in wait till
        engine function which we will skip if bit is set.
        
      \author
          Aaron Swift
*******************************************************************************/
void setSkipLabelTakenCheck( void )
{
    skipLabelTaken_  = true;  
}

/******************************************************************************/
/*!   \fn void startPrintEngine( void )

      \brief
        This function resets the print engine counters and starts the line burn/printhead loading,
        print roller motor and paper takeup motor interrupts.    

      \author
          Aaron Swift
*******************************************************************************/
void startPrintEngine( void )
{ 
    /* set motors to half steps*/
    setHalfStepMode(_MAIN_STEPPER);
    setHalfStepMode(_TAKEUP_STEPPER);
    
    /* set takeup motor direction for tightening */
    setTakeUpMotorDirection( BACKWARDM_ );
    
    //setTUChangeTestPass(false);
    //setSHOOTChangeTestPass(false);
    //setLOWSTOCKChangeTestPass(false);
  
    if(getTUStuckTestPass() == true)
    {
        checkForPaper(((float)config_.takeup_sensor_max_tension_counts * 0.90), 820);
    }
    
    takeupDelay();
    
    config_.contrast_adjustment = 3;
    
    //PRINTF("startPrintEngine()\r\n");
    //PRINTF("streamingLabelBackwind = %d\r\n", streamingLabelBackwind);
    //PRINTF("numPrintLines: %d\r\n", engine.numPrintLines);
    //PRINTF("backwindAfterSizing = %d\r\n", backwindAfterSizing);
    
    //PRINTF("label size: %d\r\n", getLabelSizeInQuarterSteps());
    
    setLabelSizeInQuarterSteps(1200);
  
    //PRINTF("label size: %d\r\n", getLabelSizeInQuarterSteps());
    
    setLTWaitCount_(0);
    /*
    if(backwindAfterSizing == true)
    {
        if(getTakingUpPaper() == true)
        {
            loosenStock(300, 1700);
            backwindStock(calculateSizingBackwindSteps(), 1300);
        }
        else
        {
            backwindStock(calculateSizingBackwindSteps(), 1500);
        }
        
        //PRINTF("sizing backwind steps = %d\r\n", calculateSizingBackwindSteps());
    }

    if(getTakingUpPaper() == false && backwindAfterSizing == false && continuousStock_ == false)
    {
        if(streamingLabelBackwind != 0)
        {
            streamingLabelBackwind = calculateStreamingBackwindSteps();
      
            backwindStock(streamingLabelBackwind, 1500);
            //PRINTF("streaming backwind steps = %d\r\n", calculateStreamingBackwindSteps());
        }    
    }
    
    backwindAfterSizing = false;
    */
    
    /* check if vertical position has changed since the last print, if it has changed then also change
    the firstLabelGap and offsetThisPrint by the amount that config_.verticalPosition has changed from 
    verticalOffsetIndex */
    if(verticalOffsetIndex > config_.verticalPosition)
    {
        /* vertical offset index was greater than config */
        offsetThisPrint = (verticalOffsetIndex - config_.verticalPosition);
        firstLabelGap = firstLabelGap + offsetThisPrint;
        verticalOffsetIndex = config_.verticalPosition;
        
        //PRINTF("vertical offset was less than config - vert pos decreased\r\n");
        //PRINTF("offsetThisPrint = %d\r\n", offsetThisPrint);
    }
    else if(verticalOffsetIndex < config_.verticalPosition)
    {
        /* vertical offset index was less than config */
        offsetThisPrint = (config_.verticalPosition - verticalOffsetIndex);
        firstLabelGap = firstLabelGap - offsetThisPrint;
        verticalOffsetIndex = config_.verticalPosition;
        
        //PRINTF("vertical offset was less than config - vert pos increased\r\n");
        //PRINTF("offsetThisPrint = %d\r\n", offsetThisPrint);
    }
    else if(verticalOffsetIndex == config_.verticalPosition)
    {
        /* vertical offset index == config*/
        offsetThisPrint = 0;
        verticalOffsetIndex = config_.verticalPosition;
    }

    /* reset engine counters */
    engine.lineCounter = 0;
    engine.burnSequence = 0;
    engine.linePrintDone = false; // are we done burning and loading lines?
    leadInSteps = 0; // the amount of steps before lines begin to be loaded in the printhead and burnedd
    leadInDone = false; // are we done with the lead in?
    expelSteps = 0; // the amount of steps to continue stepping after we are done burning and loading llines
    expelDone = false; // are we done with the expel?
    labelLowIndexOffset = 0; // reset the label low buffer index
    setShootIndex(0);
    TPHStepsPastGapThisPrint = 0;
      
    /* turn on print head power */ 
    setHeadPower(false);

    /* initialize the printhead roller motor interrupt, this function is currently defined in 
    takeupMotor.c */
    initTPHIntr();
    
    /* if we are taking up paper, tighten the paper before the print starts */
    if(getTakingUpPaper() == true)
    {
        //PRINTF("takingUpPaper == true\r\n");
      
        /* if(engine.totalLinesToPrint >= 1200) tensionModifier is incremented by 2 each print, 
        otherwise it is incremented by 1 each print, currently it is capped to 400. tensionModifier is
        reset/estimated every cassette open/close*/
#ifdef TFinkFullStep
        uint16_t startingTUSpeed = (1520 + getTensionModifier() * 2);
        
        if(startingTUSpeed > 2320) //1160 uS
        {
            startingTUSpeed = 2320;
        }
#else
        uint16_t startingTUSpeed = (760 + getTensionModifier());
        
        if(startingTUSpeed > 1160) //1160 uS
        {
            startingTUSpeed = 1160;
        }
#endif
      
        enableVScope();
        /*tightenStock starts an interupt that pulses the takeup motor step pin at startingSpeed until
        the takeup clutch tension sensor counts equal the first argument. */
        
        #ifdef TFinkFullStep
        unsigned short tensionSetting = 0;
        tensionSetting = (unsigned short)((float)config_.takeup_sensor_max_tension_counts/0.85);
        tightenStock(   
            tensionSetting,   //TFinkToD Make this 85% of TUSensorThreshold rather than 74%
            startingTUSpeed, 
            true,
			FULL_STEP);    
        #else
        unsigned short tensionSetting = 0;
        tensionSetting = (unsigned short)((float)config_.takeup_sensor_max_tension_counts/0.85);
        PRINTF("tension SP: %d\r\n",tensionSetting);
        tightenStock(   
            tensionSetting,   
            startingTUSpeed, 
            true,
			HALF_STEP);
        #endif
    }        
      
    /* start the printhead roller motor */
    if(continuousStock_ == true)
    {
        /* start the print roller motor interrupt, for a continuous stock print we want to travel the 
        amount of steps equal to (lead in + numPrintLines + expel)*/
        startTPHIntr((getIndirectData( 4 ) + 56) + engine.numPrintLines + (getIndirectData( 4 ) + 136) );
    }
    else
    {
        /* getLargeGapFlag() will return true if during the last sizing the label gap was estimated to
        to be larger than 55 steps (HT/RFID labels with a larger gap than the standard smaller GT gap)*/
        if(getLargeGapFlag() == true)
        {            
            if(getLabelSizeInQuarterSteps() <= 1750 && getLabelSizeInQuarterSteps() >= 1550) //4 inch HT labels 1643 steps
            {
                if(getTakingUpPaper() == true)
                {
                    startTPHIntr(engine.numPrintLines + 170 + 85); //printlines + expel + lead in
                }
                else
                {
                    startTPHIntr(engine.numPrintLines + 115 + 96 ); //printlines + expel + lead in
                }
            }
            else if(getLabelSizeInQuarterSteps() <= 2260 && getLabelSizeInQuarterSteps() >= 2160) //5.5 inch HT labels
            {
                if(getTakingUpPaper() == true)
                {
                    startTPHIntr(engine.numPrintLines + 170 + 85); //printlines + expel + lead in
                }
                else
                {
                    startTPHIntr(engine.numPrintLines + 115 + 96 ); //printlines + expel + lead in
                }
            }
            else if(getLabelSizeInQuarterSteps() <= 2750 && getLabelSizeInQuarterSteps() >= 2550) //6.5 inch
            {
                if(getTakingUpPaper() == true)
                {
                    startTPHIntr(engine.numPrintLines + 190 + 85); //printlines + expel + lead in
                }
                else
                {
                    startTPHIntr(engine.numPrintLines + 135 + 96 ); //printlines + expel + lead in
                }
            }
            else if(getLabelSizeInQuarterSteps() <= 2950 && getLabelSizeInQuarterSteps() >= 2751)
            {
                if(getTakingUpPaper() == true)
                {
                    startTPHIntr(engine.numPrintLines + 165 + 85); //printlines + expel + lead in
                }
                else
                {
                    startTPHIntr(engine.numPrintLines + 110 + 96 ); //printlines + expel + lead in
                }
            }
        }  
        else
        {
            if(getTakingUpPaper() == true)
            {
                if(getLabelSizeInQuarterSteps() <= 780 && getLabelSizeInQuarterSteps() >= 680) //1.75
                {
                    startTPHIntr(engine.numPrintLines + 126 + 45); //printlines + expel + lead in 
                }
                else
                {
                    //PRINTF("SPOOF LENGTH OK 2\r\n");
                    startTPHIntr(engine.numPrintLines + 62 + 45 ); //printlines + expel + lead in 
                }
                  
            }
            else
            {

                if(getLabelSizeInQuarterSteps() <= 780 && getLabelSizeInQuarterSteps() >= 680)
                {
                    // 1.75 inch
                    startTPHIntr(engine.numPrintLines + 130 + 80);
                }
                else if(getLabelSizeInQuarterSteps() <= 1000 && getLabelSizeInQuarterSteps() >= 900)
                {
                    // 2.37 inch
                    startTPHIntr(engine.numPrintLines + 134 + 45);
                }
                else if(getLabelSizeInQuarterSteps() <= 1250 && getLabelSizeInQuarterSteps() >= 1150)
                {
                    // 3 inch
                    //PRINTF("SPOOF LENGTH OK 2\r\n");
                    startTPHIntr(engine.numPrintLines + 118 + 45);
                }
                else if(getLabelSizeInQuarterSteps() <= 1450 && getLabelSizeInQuarterSteps() >= 1350)
                {
                     // 3.5 inch
                    startTPHIntr(engine.numPrintLines + 122 + 45);
                }
                else if(getLabelSizeInQuarterSteps() <= 1650 && getLabelSizeInQuarterSteps() >= 1590)
                {
                    // 4 inch
                    startTPHIntr(engine.numPrintLines + 116 + 45);
                }
                else if(getLabelSizeInQuarterSteps() <= 1870 && getLabelSizeInQuarterSteps() >= 1700)
                {
                    // 4.5 inch
                    startTPHIntr(engine.numPrintLines + 114 + 45);
                }
                else if(getLabelSizeInQuarterSteps() <= 2060 && getLabelSizeInQuarterSteps() >= 1960)
                {
                    // 5 inch
                    startTPHIntr(engine.numPrintLines + 110 + 45);
                }
                else if(getLabelSizeInQuarterSteps() <= 2260 && getLabelSizeInQuarterSteps() >= 2160)
                {
                    // 5.5 inch
                    startTPHIntr(engine.numPrintLines + 105 + 45);
                }
                else if(getLabelSizeInQuarterSteps() <= 2500 && getLabelSizeInQuarterSteps() >= 2380)
                {
                    // 6 inch
                    startTPHIntr(engine.numPrintLines + 94 + 45);
                }
                else if(getLabelSizeInQuarterSteps() <= 2700 && getLabelSizeInQuarterSteps() >= 2600)
                {
                    // 6.5 inch
                    startTPHIntr(engine.numPrintLines + 98 + 45);
                }
                else if(getLabelSizeInQuarterSteps() <=2860 && getLabelSizeInQuarterSteps() >= 2760)
                {
                    // 7 inch
                    startTPHIntr(engine.numPrintLines + 82 + 45);
                }
                else if(getLabelSizeInQuarterSteps() <= 3070 && getLabelSizeInQuarterSteps() >= 2970)
                {
                    // 7.5 inch
                    startTPHIntr(engine.numPrintLines + 80 + 45);
                }
                else if(getLabelSizeInQuarterSteps() <= 3260 && getLabelSizeInQuarterSteps() >= 3160)
                {
                    // 8 inch
                    startTPHIntr(engine.numPrintLines + 82 + 45);
                }
                else if(getLabelSizeInQuarterSteps() <= 3490 && getLabelSizeInQuarterSteps() >= 3400)
                {
                    // 8.5 inch
                    startTPHIntr(engine.numPrintLines + 68 + 45);
                }
                else if(getLabelSizeInQuarterSteps() <= 3690 && getLabelSizeInQuarterSteps() >= 3590) 
                {
                    // 9 inch
                    startTPHIntr(engine.numPrintLines + 64 + 45);
                }
                else if(getLabelSizeInQuarterSteps() <= 3890 && getLabelSizeInQuarterSteps() >= 3700) 
                {
                    // 9.5 inch
                    startTPHIntr(engine.numPrintLines + 65 + 45);
                }
                else if(getLabelSizeInQuarterSteps() <= 5000 && getLabelSizeInQuarterSteps() >= 3960) 
                {
                    // 10 inch
                    startTPHIntr(engine.numPrintLines + 60 + 45);
                }
            }
        }      
    }
    
    /* if we are taking up paper, start the takeup motor interrupt */
    if(getTakingUpPaper() == true)
    {
#ifdef TFinkFullStep
        tModifier = getTensionModifier();
        uint16_t startingSpeed = 1520 + getTensionModifier() * 2;
        
        if(startingSpeed > 2320) //1160 uS
        {
            startingSpeed = 2320;
        }
#else
        tModifier = getTensionModifier();
        uint16_t startingSpeed = 760 + getTensionModifier();
        
        if(startingSpeed > 1500) //1160 uS
        {
            startingSpeed = 1500;
        }
#endif
      
        /* increment the tension modifier more for larger labels  */
        if(engine.totalLinesToPrint >= 1200)
        {
            tModifier += 2;
            setTensionModifier(tModifier);
        }
        else
        {
            tModifier++;
            setTensionModifier(tModifier);
        }

#ifdef TFinkFullStep
         if(tModifier >= 800)
        {
            tModifier = 800;
            setTensionModifier(tModifier);
        }
#else
        if(tModifier >= 400)
        {
            tModifier = 400;
            setTensionModifier(tModifier);
        }
#endif

        /* start the takeup motor intr */
        stepTUMotor(engine.totalLinesToPrint + 6000, startingSpeed);
    }
    
    /* init and start the line timer intr that loads the printhead and burns the lines */
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

        Timer period    dependent on contrast setting
        Timer Freq      1.953Mhz   
      \author
          Aaron Swift
*******************************************************************************/
void startLineTimer(  bool start  )
{ 
    /* set GPT2 intr handler type to printhead loading and line burning */
    setGPT2IntrType(BURN_LINE);
  
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
    GPT_EnableInterrupts( GPT2, kGPT_OutputCompare1InterruptEnable );   
    GPT_EnableInterrupts( GPT2, kGPT_OutputCompare2InterruptEnable );    
    GPT_EnableInterrupts( GPT2, kGPT_OutputCompare3InterruptEnable );         

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
    
    /* clear any flags that might be set */
    GPT_ClearStatusFlags( LINE_PRINTER_TIMER_BASE, kGPT_OutputCompare1Flag );
    GPT_ClearStatusFlags( LINE_PRINTER_TIMER_BASE, kGPT_OutputCompare2Flag );
    GPT_ClearStatusFlags( LINE_PRINTER_TIMER_BASE, kGPT_OutputCompare3Flag );
    
    /* set channel 1 for slt time */
    GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel1, engine.histAdj[0].time ); 
    /* set channel 2 for history time */
    GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel2,  engine.histAdj[1].time ); 
    /* set channel 3 for current line time */
    GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel3, engine.sltTime ); 
}
  
/******************************************************************************/
/*!   \fn static void( void )

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
    /* deinitialize strobe pwm channel, we are not currently PWM'ing the strobe pins */
    //PWM_Deinit( PWM2, kPWM_Module_0 );
    
    /*  re-enable peripheral io control of the strobe pin */
    gpio_pin_config_t strobeConfig = { kGPIO_DigitalOutput, 0 };
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_38_GPIO3_IO06, 0x70A0U );
   
    /* release data strobe */
    GPIO_PinInit( PHEAD_STROBE_A_GPIO, PHEAD_STROBE_A_PIN, &strobeConfig );
    GPIO_PinInit( PHEAD_STROBE_B_GPIO, PHEAD_STROBE_B_PIN, &strobeConfig );
    
    /* release strobe enable */
    GPIO_WritePinOutput( PHEAD_STROBE_EN_GPIO, PHEAD_STROBE_EN_PIN, true ); 
    
    engine.burnSequence = 0; // reset the burnSequence
    
    if(leadInDone == false)
    {
        TUSensorTestValueBase = getTakeUpTorque();
        SHOOTSensorTestValueBase = pollMediaCounts();
        LOWSTOCKSensorTestValueBase = getLowStockSensor();
        
        TUSensorTestValue = 0;
        SHOOTSensorTestValue = 0;
        LOWSTOCKSensorTestValue = 0;
    }

    /* if we aren't done with the lead in yet and we are using continuous stock */
    if(leadInDone == false && continuousStock_ == true)
    {
        /* increment our lead in steps */
        leadInSteps++;
      
        /* if our lead in steps are greater than or equal to our desired lead in (hardcoded to expel + 56) 
        then set leadInDone to true and restart our timer*/
        if(leadInSteps >= (getIndirectData( 4 ) + 56))
        {
            //PRINTF("lead in done\r\n");
            leadInDone = true;
            
            stopLineTimer();
            GPT_StartTimer( GPT2 );
        }
        else /* continue lead in */
        {
            stopLineTimer();
            GPT_StartTimer( GPT2 );
        }
    }/* if we aren't done with the lead in yet and we are using die-cut stock */
    else if(leadInDone == false && continuousStock_ == false)
    {
        /* increment our lead in steps */
        leadInSteps++;
      
        /* Walmart RFID/HT stock lead in */
        if(getLargeGapFlag() == true)
        {
            if(getLabelSizeInQuarterSteps() <= 1750 && getLabelSizeInQuarterSteps() >= 1550) //4 inch HT
            {
                if(getTakingUpPaper() == false)
                {
                    /* if our lead in steps are greater than or equal to our desired lead in step amount
                    then set leadInDone to true and restart our timer*/
                    if(leadInSteps >= 74)
                    {
                        //PRINTF("lead in done 4 inch HT NO PAPER\r\n");
                        leadInDone = true;
                        
                        stopLineTimer();
                        GPT_StartTimer( GPT2 );
                    }
                    else /* continue lead in */
                    {
                        stopLineTimer();
                        GPT_StartTimer( GPT2 );
                    }
                }
                else
                {
                    /* if our lead in steps are greater than or equal to our desired lead in step amount
                    then set leadInDone to true and restart our timer*/
                    if(leadInSteps >= 166)
                    {
                        //PRINTF("lead in done 4 inch HT\r\n");
                        leadInDone = true;
                        
                        stopLineTimer();
                        GPT_StartTimer( GPT2 );
                    }
                    else /* continue lead in */
                    {
                        stopLineTimer();
                        GPT_StartTimer( GPT2 );
                    }
                }
            }
            else if(getLabelSizeInQuarterSteps() <= 2260 && getLabelSizeInQuarterSteps() >= 2160) //5.5 inch HT labels
            {
                if(getTakingUpPaper() == false)
                {
                    /* if our lead in steps are greater than or equal to our desired lead in step amount
                    then set leadInDone to true and restart our timer*/
                    if(leadInSteps >= 74)
                    {
                        //PRINTF("lead in done 5.5 inch HT NO PAPER\r\n");
                        leadInDone = true;
                        
                        stopLineTimer();
                        GPT_StartTimer( GPT2 );
                    }
                    else /* continue lead in */
                    {
                        stopLineTimer();
                        GPT_StartTimer( GPT2 );
                    }
                }
                else
                {
                    /* if our lead in steps are greater than or equal to our desired lead in step amount
                    then set leadInDone to true and restart our timer*/
                    if(leadInSteps >= 175)
                    {
                        //PRINTF("lead in done 5.5 inch HT\r\n");
                        leadInDone = true;
                        
                        stopLineTimer();
                        GPT_StartTimer( GPT2 );
                    }
                    else /* continue lead in */
                    {
                        stopLineTimer();
                        GPT_StartTimer( GPT2 );
                    }
                }
            }
            else if(getLabelSizeInQuarterSteps() <= 2800 && getLabelSizeInQuarterSteps() >= 2550) //6.5 inch HT
            {
                if(getTakingUpPaper() == false)
                {
                    /* if our lead in steps are greater than or equal to our desired lead in step amount
                    then set leadInDone to true and restart our timer*/
                    if(leadInSteps >= 74)
                    {
                        //PRINTF("lead in done 6.5 inch HT NO PAPER\r\n");
                        leadInDone = true;
                        
                        stopLineTimer();
                        GPT_StartTimer( GPT2 );
                    }
                    else /* continue lead in */
                    {
                        stopLineTimer();
                        GPT_StartTimer( GPT2 );
                    }
                }
                else
                {
                    /* if our lead in steps are greater than or equal to our desired lead in step amount
                    then set leadInDone to true and restart our timer*/
                    if(leadInSteps >= 170)
                    {
                        //PRINTF("lead in done 6.5 inch HT\r\n");
                        leadInDone = true;
                        
                        stopLineTimer();
                        GPT_StartTimer( GPT2 );
                    }
                    else /* continue lead in */
                    {
                        stopLineTimer();
                        GPT_StartTimer( GPT2 );
                    }
                }
            }
            else if(getLabelSizeInQuarterSteps() <= 2950 && getLabelSizeInQuarterSteps() >= 2751) //7 inch HT labels
            {
                if(getTakingUpPaper() == false)
                {
                    /* if our lead in steps are greater than or equal to our desired lead in step amount
                    then set leadInDone to true and restart our timer*/
                    if(leadInSteps >= 100)
                    {
                        //PRINTF("lead in done 7 inch HT NO PAPER\r\n");
                        leadInDone = true;
                        
                        stopLineTimer();
                        GPT_StartTimer( GPT2 );
                    }
                    else /* continue lead in */
                    {
                        stopLineTimer();
                        GPT_StartTimer( GPT2 );
                    }
                }
                else
                {
                    /* if our lead in steps are greater than or equal to our desired lead in step amount
                    then set leadInDone to true and restart our timer*/
                    if(leadInSteps >= 204)
                    {
                        //PRINTF("lead in done 7 inch HT\r\n");
                        leadInDone = true;
                        
                        stopLineTimer();
                        GPT_StartTimer( GPT2 );
                    }
                    else /* continue lead in */
                    {
                        stopLineTimer();
                        GPT_StartTimer( GPT2 );
                    }
                }
            }
        }
        else /* GT stock lead in*/
        {       
            if(getTakingUpPaper() == false)
            {                
                if(getLabelSizeInQuarterSteps() <= 780 && getLabelSizeInQuarterSteps() >= 680) //1.75
                {
                    leadInStepTarget = 48;
                }
                else if(getLabelSizeInQuarterSteps() <= 1000 && getLabelSizeInQuarterSteps() >= 900) //2.37
                {
                    leadInStepTarget = 30;
                }
                else if(getLabelSizeInQuarterSteps() <= 1299 && getLabelSizeInQuarterSteps() >= 1100) //3.0 
                {    
                    leadInStepTarget = 20;
                }
                else if(getLabelSizeInQuarterSteps() <= 1499 && getLabelSizeInQuarterSteps() >= 1300) //3.5
                {
                    leadInStepTarget = 32;
                }
                else if(getLabelSizeInQuarterSteps() <= 1699 && getLabelSizeInQuarterSteps() >= 1500) //4.0
                {
                    leadInStepTarget = 24;
                }
                else if(getLabelSizeInQuarterSteps() <= 1900 && getLabelSizeInQuarterSteps() >= 1700) //4.5 
                {
                    leadInStepTarget = 34;
                }
                else if(getLabelSizeInQuarterSteps() <= 2060 && getLabelSizeInQuarterSteps() >= 1960) //5.0 
                {
                    leadInStepTarget = 28;
                }
                else if(getLabelSizeInQuarterSteps() <= 2260 && getLabelSizeInQuarterSteps() >= 2160) //5.5 
                {
                    leadInStepTarget = 32;
                }
                else if(getLabelSizeInQuarterSteps() <= 2500 && getLabelSizeInQuarterSteps() >= 2380) //6.0 
                {
                    leadInStepTarget = 20;
                }
                else if(getLabelSizeInQuarterSteps() <= 2680 && getLabelSizeInQuarterSteps() >= 2600) //6.5 
                {
                    leadInStepTarget = 30;
                }
                else if(getLabelSizeInQuarterSteps() <=2860 && getLabelSizeInQuarterSteps() >= 2780) //7.0 
                {
                    leadInStepTarget = 16;
                }
                else if(getLabelSizeInQuarterSteps() <= 3070 && getLabelSizeInQuarterSteps() >= 2970) //7.5 
                {
                    leadInStepTarget = 16;
                }
                else if(getLabelSizeInQuarterSteps() <= 3300 && getLabelSizeInQuarterSteps() >= 3200) //8.0 
                {  
                    leadInStepTarget = 20;
                }
                else if(getLabelSizeInQuarterSteps() <= 3490 && getLabelSizeInQuarterSteps() >= 3400) //8.5 
                {
                    leadInStepTarget = 20;
                }
                else if(getLabelSizeInQuarterSteps() <= 3690 && getLabelSizeInQuarterSteps() >= 3590) //9.0 
                {
                    leadInStepTarget = 20;
                }
                else if(getLabelSizeInQuarterSteps() <= 3890 && getLabelSizeInQuarterSteps() >= 3700) //9.5 
                {
                    leadInStepTarget = 16;
                }
                else if(getLabelSizeInQuarterSteps() <= 5000 && getLabelSizeInQuarterSteps() >= 3960) //10 
                {
                    leadInStepTarget = 30;
                }
                else //unmatched label length
                {
                    leadInStepTarget = 20;
                }
                
                /* if our lead in steps are greater than or equal to our desired lead in step amount
                then set leadInDone to true and restart our timer*/
                if(leadInSteps >= leadInStepTarget)
                {
                    leadInDone = true;
                    
                    //PRINTF("LEAD IN DONE\r\n");
                    
                    stopLineTimer();
                    GPT_StartTimer( GPT2 );
                }
                else/* continue lead in */
                {
                    stopLineTimer();
                    GPT_StartTimer( GPT2 );
                }
            }
            else
            {
                if(getLabelSizeInQuarterSteps() <= 780 && getLabelSizeInQuarterSteps() >= 680) //1.75
                {
                    leadInStepTarget = 110;
                }
                else if(getLabelSizeInQuarterSteps() <= 1000 && getLabelSizeInQuarterSteps() >= 900) //2.37
                {
                    leadInStepTarget = 34;
                }
                else if(getLabelSizeInQuarterSteps() <= 1299 && getLabelSizeInQuarterSteps() >= 1100) //3.0 
                {    
                    leadInStepTarget = 38;
                }
                else if(getLabelSizeInQuarterSteps() <= 1499 && getLabelSizeInQuarterSteps() >= 1300) //3.5 
                {
                    leadInStepTarget = 34;
                }
                else if(getLabelSizeInQuarterSteps() <= 1699 && getLabelSizeInQuarterSteps() >= 1500) //4.0 
                {
                    leadInStepTarget = 32;
                }
                else if(getLabelSizeInQuarterSteps() <= 1900 && getLabelSizeInQuarterSteps() >= 1700) //4.5 
                {
                    leadInStepTarget = 34;
                }
                else if(getLabelSizeInQuarterSteps() <= 2060 && getLabelSizeInQuarterSteps() >= 1960) //5.0 
                {
                    leadInStepTarget = 24;
                }
                else if(getLabelSizeInQuarterSteps() <= 2260 && getLabelSizeInQuarterSteps() >= 2160) //5.5 
                {
                    leadInStepTarget = 26;
                }
                else if(getLabelSizeInQuarterSteps() <= 2500 && getLabelSizeInQuarterSteps() >= 2380) //6.0 
                {
                    leadInStepTarget = 20;
                }
                else if(getLabelSizeInQuarterSteps() <= 2680 && getLabelSizeInQuarterSteps() >= 2600) //6.5
                {
                    leadInStepTarget = 22;
                }
                else if(getLabelSizeInQuarterSteps() <=2860 && getLabelSizeInQuarterSteps() >= 2780) //7.0 
                {
                    leadInStepTarget = 24;
                }
                else if(getLabelSizeInQuarterSteps() <= 3070 && getLabelSizeInQuarterSteps() >= 2970) //7.5 
                {
                    leadInStepTarget = 60;
                }
                else if(getLabelSizeInQuarterSteps() <= 3300 && getLabelSizeInQuarterSteps() >= 3200) //8.0 
                {  
                    leadInStepTarget = 66;
                }
                else if(getLabelSizeInQuarterSteps() <= 3490 && getLabelSizeInQuarterSteps() >= 3400) //8.5 
                {
                    leadInStepTarget = 64;
                }
                else if(getLabelSizeInQuarterSteps() <= 3690 && getLabelSizeInQuarterSteps() >= 3590) //9.0 
                {
                    leadInStepTarget = 64;
                }
                else if(getLabelSizeInQuarterSteps() <= 3890 && getLabelSizeInQuarterSteps() >= 3700) //9.5 
                {
                    leadInStepTarget = 58;
                }
                else if(getLabelSizeInQuarterSteps() <= 5000 && getLabelSizeInQuarterSteps() >= 3960) //10 
                {
                    leadInStepTarget = 70;
                }
                else //unmatched label length
                {
                    leadInStepTarget = 34;
                }
                            
                /* if our lead in steps are greater than or equal to our desired lead in step amount
                then set leadInDone to true and restart our timer*/
                if(leadInSteps >= leadInStepTarget)
                {
                    leadInDone = true;
                    
                    //PRINTF("LEAD IN DONE\r\n");
                    
                    stopLineTimer();
                    GPT_StartTimer( GPT2 );
                }
                else /* continue lead in */
                {
                    stopLineTimer();
                    GPT_StartTimer( GPT2 );
                }
            }  
        }
    } /* lead in is done, printing is done, continuous stock expel */
    else if(leadInDone == true && engine.linePrintDone == true && expelDone == false && continuousStock_ == true)
    {   
        /* printhead power off */
        setHeadPower(false);
        
        /* increment expel steps */
        expelSteps++;
        
        /* if our expel steps are greater than or equal to our desired expel (hardcoded to expel + 136) 
        then set expelDone to true and restart our timer*/
        if(expelSteps >= (getIndirectData( 4 ) + 136))
        {
            //PRINTF("expel done\r\n");
            expelDone = true;
            
            //PRINTF("EXPEL DONE\r\n");
            
            stopLineTimer();
        }
        else /* continue exxpel */
        {
            stopLineTimer();
            GPT_StartTimer( GPT2 );
        }
    } /* lead in is done, printing is done, die-cut stock expel */
    else if(leadInDone == true && engine.linePrintDone == true && expelDone == false && continuousStock_ == false)
    {
        /* printhead power off */
        setHeadPower(false);
        
        expelDone = true;
        
        //PRINTF("EXPEL DONE\r\n");
                
        stopLineTimer();
    }
    else /* lead in is done, printing in progress, when engine.linePrintDone == true expel will begin */
    {
        /* are we done with the image? */
        if( --engine.numPrintLines <= 0 )   
        {
            //PRINTF("line print done\r\n");
            engine.linePrintDone = true;
            
            if(getDotWearHandle() != NULL && readyToNotify() )
                xTaskNotifyFromISR( getDotWearHandle(), DOT_DONE,eSetBits, pdFALSE );
            
            stopLineTimer(); 
            GPT_StartTimer( GPT2 );
        } 
        else 
        {
            stopLineTimer();       

            setHeadPower(true);
            
            
            //setTUChangeTestPass(false);
            //setSHOOTChangeTestPass(false);
            //setLOWSTOCKChangeTestPass(false);
           
            
            if(TUSensorTestValueBase == getTakeUpTorque())
            {
                TUSensorTestValue++;
                
                if(TUSensorTestValue > 50)
                {
                    setTUChangeTestPass(false);
                }
            }
            
            if(SHOOTSensorTestValueBase == pollMediaCounts())
            {
                SHOOTSensorTestValue++;
                
                if(SHOOTSensorTestValue > 50)
                {
                    setSHOOTChangeTestPass(false);
                }
            }
            
            if(LOWSTOCKSensorTestValueBase == getLowStockSensor())
            {
                LOWSTOCKSensorTestValue++;
                
                if(LOWSTOCKSensorTestValue > 50)
                {
                    setLOWSTOCKChangeTestPass(false);
                }
            }
            
            
            
            /* load history line*/
            loadHistory();  
            
            GPT_StartTimer( GPT2 );
        }
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
        //PRINTF("\r\n\r\nHISTORY ENABLED\r\n\r\n");
        lpspi_transfer_t masterXfer;  
         
        clearBurnSequence();
       
        /* setup master transfer */
        masterXfer.txData = (unsigned char *)engine.pHistory;
        masterXfer.rxData = NULL;
             
        if( getHeadStyleSize() == HEAD_DOTS_72MM ) {
            masterXfer.dataSize = PRINTER_HEAD_SIZE_72MM;
        } else if( getHeadStyleSize() == HEAD_DOTS_80MM ) {
            masterXfer.dataSize = PRINTER_HEAD_SIZE_80MM;        
        } else {
            PRINTF( "loadHistory(): unsupported head style!\r\n"); 
        }
        
        /*
        PRINTF("\r\n\r\nHistory:\r\n");
        for(int i = 0; i < masterXfer.dataSize; i++)
        {
            PRINTF("%b,", masterXfer.txData[i]);
            //takeupDelayShort();
        }
        PRINTF("\r\n\r\n");
        */
        masterXfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous;
        
        #if 1   /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
        unsigned long result  = LPSPI_MasterTransferEDMA( LPSPI4, &spiHeadMasterHandle, &masterXfer );
        #else
        unsigned long result = LPSPI_MasterTransferNonBlocking( LPSPI4, &spiHeadMasterHandle, &masterXfer);
        #endif
        
        if( kStatus_Success != result ) {
            PRINTF( "loadHistory(): Print head load error! %d\r\n", result );
            PRINTF("lines left: %d\r\n", engine.numPrintLines);
            PRINTF("lines to print: %d\r\n", engine.totalLinesToPrint);
            /* stop engine timer and fix problem */
            stopLineTimer();
            
            #if 1  /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
            /* abort the current transfer and retry */
            LPSPI_MasterTransferAbortEDMA( LPSPI4, &spiHeadMasterHandle );
            #else
            LPSPI_MasterTransferAbort( LPSPI4, &spiHeadMasterHandle );
            #endif            
            
            /* try re-init of SPI/DMA */
            initializePrintHeadSPI();
            #if 1  /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
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
    
    unsigned long printLines = 0;
    
    if(getPrintHeadType() == ROHM_72MM_800_OHM)
    {
        printLines = N_PRINTER_LINES_72MM;
    }
    else
    {
        printLines = N_PRINTER_LINES_80MM;
    }
    
    /* do not allow the print engine outside the image buffer */
    //if( engine.lineCounter > N_PRINTER_LINES ) {
    if( engine.lineCounter ==  printLines) {
        engine.lineCounter = 0;
        /* label image is greater than 5" */ 
        offset = engine.lineCounter;        
    } else {
    	
    	if(getHeadStyleSize() == HEAD_DOTS_72MM)
    	{
    		offset = ( engine.lineCounter * PRINTER_HEAD_SIZE_72MM );	 
    	}
    	else
    	{
    		offset = ( engine.lineCounter * PRINTER_HEAD_SIZE_80MM );	
    	}
    	
           
    }
    
        
    /*
    if( engine.lineCounter == ( N_PRINTER_LINES - 2 ) ) {
        memcpy( &lastBfr[0], (unsigned char *)engine.pImage + offset, 144 );   
    } */

    /* setup master transfer */
    masterXfer.txData = (unsigned char *)engine.pImage + offset; 
    masterXfer.rxData = NULL;
    
    
    

    if( getHeadStyleSize() == HEAD_DOTS_72MM ) {
        masterXfer.dataSize = PRINTER_HEAD_SIZE_72MM;
    } else if( getHeadStyleSize() == HEAD_DOTS_80MM ) {
        masterXfer.dataSize = PRINTER_HEAD_SIZE_80MM;        
    } else {
        PRINTF( "loadHistory(): unsupported head style!\r\n"); 
    }
    
    
    masterXfer.configFlags =  kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous;

    #if 1  /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
    unsigned long result  = LPSPI_MasterTransferEDMA( LPSPI4, &spiHeadMasterHandle, &masterXfer );    
    #else
    unsigned long result = LPSPI_MasterTransferNonBlocking( LPSPI4, &spiHeadMasterHandle, &masterXfer);
    #endif
        
    
    if( kStatus_Success != result ) {
        PRINTF( "loadPrintLine(): Print head load error! %d\r\n", result );
        /* stop engine timer and fix problem */
        stopLineTimer();
        
        #if 1  /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
        /* abort the current transfer and retry */
        LPSPI_MasterTransferAbortEDMA( LPSPI4, &spiHeadMasterHandle );
        #else
        LPSPI_MasterTransferAbort( LPSPI4, &spiHeadMasterHandle );
        #endif        
        /* try re-init of SPI/DMA */
        initializePrintHeadSPI();
        
        #if 1  /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
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
    
    //prevLineCounter = currLineCounter;
    engine.lineCounter++;
    currLineCounter = engine.lineCounter;
    
    /* are we about to rollover? ( + 1 still need to print last line ) SOF-5965*/   
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
    
    if( getHeadStyleSize() == HEAD_DOTS_72MM ) {
        masterXfer.dataSize = PRINTER_HEAD_SIZE_72MM;
    } else if( getHeadStyleSize() == HEAD_DOTS_80MM ) {
        masterXfer.dataSize = PRINTER_HEAD_SIZE_80MM;        
    } else {
        PRINTF( "loadZeroPrintLine(): unsupported head style!\r\n"); 
    }
    
    masterXfer.configFlags =  kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous;
    
    #if 1  /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
    unsigned long result  = LPSPI_MasterTransferEDMA( LPSPI4, &spiHeadMasterHandle, &masterXfer );
    #else
    unsigned long result = LPSPI_MasterTransferNonBlocking( LPSPI4, &spiHeadMasterHandle, &masterXfer);
    #endif
    
    if( kStatus_Success != result ) {
        PRINTF( "loadZeroPrintLine(): Print head load error! %d\r\n", result );
        /* stop engine timer and fix problem */
        stopLineTimer();
        
        #if 1  /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */        
        /* abort the current transfer and retry */
        LPSPI_MasterTransferAbortEDMA( LPSPI4, &spiHeadMasterHandle );
        #else
        LPSPI_MasterTransferAbort( LPSPI4, &spiHeadMasterHandle );
        #endif        
        
        /* try re-init of SPI/DMA */
        initializePrintHeadSPI();
        #if 1  /* edma transfers are slower than a manual transfer! 500uS -> edma vs 150us manual */
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
    pwm_config_t pwmConfig;
    pwm_signal_param_t pwmSignal;

    pwmSignal.pwmChannel       = kPWM_PwmA;
    pwmSignal.level            = kPWM_HighTrue;          // kPWM_LowTrue kPWM_HighTrue;
    /* strobe signal is active low! pulse width low is measured*/ 
    pwmSignal.dutyCyclePercent = engine.pwmDutyCycle;      
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
        if(leadInDone == true && engine.linePrintDone == false)
        {
            loadPrintLine();  
        }
        else
        {
            loadZeroPrintLine();
        }
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
#if 0
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
            }

            /* enable the strobe*/
            GPIO_WritePinOutput( PHEAD_STROBE_EN_GPIO, PHEAD_STROBE_EN_PIN, false );  
            
            GPIO_WritePinOutput(PHEAD_STROBE_A_GPIO, PHEAD_STROBE_A_PIN, true);  
            GPIO_WritePinOutput(PHEAD_STROBE_B_GPIO, PHEAD_STROBE_B_PIN, true);

            /* get the current line loaded in the head but don't latch until current line time */
            lineTimerBurn();  
            /* setup for pwm start time */
            GPT_SetOutputCompareValue( GPT2, kGPT_OutputCompare_Channel1, engine.pwmStartTime );
            pwmStartTime_ = true;
        } else {
            lineTimerStrobe();
            
            /* latch control during current line. 
               data latch was set low above, lets set it back high now. 
               this should give us enough pulse width */ 
            GPIO_WritePinOutput( PHEAD_LATCH_GPIO, PHEAD_LATCH_PIN, true );                      
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
        //if(getTakingUpPaper() == true)
        //{
        //    halfStepMotor();    //half step TPH and quarter step takeup here
        //}
        //else
        //{
            stepMainMotor();    //half step TPH here
        //}    
    }
    
    /* added for arm errata 838869 */
    SDK_ISR_EXIT_BARRIER;        
}
#endif

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
    //while(getTakeupBusy() == true) {};
  
    //powerOffMotors();
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
    //powerOnMotors();
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
    /* takeup motor need to run opposite of the main motor */
    if( (StepDirM)direction == FORWARDM_ )
        setTakeUpMotorDirection( BACKWARDM_ );
    else
        setTakeUpMotorDirection( FORWARDM_ );      
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
    resetTUTorqueControlVars();
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
/*!   \fn unsigned long getPrintEngineLineCntr( void )                                                     
 
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
    #if 0   /* added for debugging printEngine */
    //GPIO_WritePinOutput( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, true );
    #endif
    
    if( ( GPT_GetStatusFlags( ENGINE_TIMER_BASE, kGPT_OutputCompare1Flag ) & 
          kGPT_OutputCompare1Flag ) == kGPT_OutputCompare1Flag ) {
  
        /* no thread running in auto mode to update the sensor info */    
        if( isADCAutoMode() ) {
            readADChannels();    
        }

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
            case TEST_FOR_CONTINUOUS: {
                testForContinuous( (StepOperation *)&engine.currentCmd );
                break;
            }
            case WAIT_UNTIL_SIZING: {
                waitUntilSizingOp( (WaitUntilOperation *)&engine.currentCmd );
                break;
            }
            case STEP_TAKEUP_DIRECTIVE: {
                stepTakeupOp( (StepUntilOperation *)&engine.currentCmd );
                break;
            }
            case STEP_TAKEUP_TIGHTEN: {
                stepTakeupTightenOp( (StepOperation *)&engine.currentCmd );
                break;
            }
            case DETECTION_STEP_UNTIL: {
                detectionOp( (StepUntilOperation *)&engine.currentCmd );
                break;
            }
            default:
              break;
          
        }
    }

    GPT_ClearStatusFlags( ENGINE_TIMER_BASE,  kGPT_OutputCompare1Flag );   

    /* added for arm errata 838869 */
    SDK_ISR_EXIT_BARRIER;   
    
    #if 0   /* added for debugging printEngine */    
    //GPIO_WritePinOutput( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, false );
    #endif    
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
      
        /* TO DO: remove missing label until fixed */
        /*
        if( ( pCurrent->user & MISSING_LABEL ) == MISSING_LABEL ) {
            pCurrent->user &= ~MISSING_LABEL;
        }
        
        if( ( pCurrent->user & JAMMED_LABEL ) == JAMMED_LABEL ) {
            pCurrent->user &= ~JAMMED_LABEL;
        }
        */
        
        //pCurrent->user |= JAMMED_LABEL;
        
        #if 0
        PRINTF("\r\n");
        if( pPrevious->error != pCurrent->error ) { 
            PRINTF("compareStatus(): pPrevious->error: %d\r\n", pPrevious->error ); 
            PRINTF("compareStatus(): pCurrent->error: %d\r\n", pCurrent->error );  
        }
        if( pPrevious->state != pCurrent->state ) {
            PRINTF("compareStatus(): pPrevious->state: %d\r\n", pPrevious->state);  
            PRINTF("compareStatus(): pCurrent->state: %d\r\n", pCurrent->state);  
        }
        if( ( pPrevious->sensor & pCurrent->mask.sensor ) != ( pCurrent->sensor & pCurrent->mask.sensor ) ) {
            PRINTF("compareStatus(): pPrevious->sensor: %d\r\n", pPrevious->sensor );  
            PRINTF("compareStatus(): pCurrent->sensor: %d\r\n", pCurrent->sensor );  
        }
        if(( pPrevious->sensor2 & pCurrent->mask.sensor2 ) != ( pCurrent->sensor2 & pCurrent->mask.sensor2 )) {
            PRINTF("compareStatus(): pPrevious->sensor2: %d\r\n", pPrevious->sensor2 );  
            PRINTF("compareStatus(): pCurrent->sensor2: %d\r\n", pCurrent->sensor2 );  
        }
        if(( pPrevious->user & pCurrent->mask.user) != ( pCurrent->user & pCurrent->mask.user )) {        
            PRINTF("compareStatus(): pPrevious->user: %d\r\n", pPrevious->user );  
            PRINTF("compareStatus(): pCurrent->user: %d\r\n", pCurrent->user );            
        }        
        #endif
                
        sendPrStatus( pCurrent, true );
        
        if( ( pCurrent->sensor2 & OUT_OF_DATA_BUFFERS ) == OUT_OF_DATA_BUFFERS ) {
            //PRINTF("compareStatus(): sending out of data buffers\r\n");  
            pCurrent->sensor2 &= ~OUT_OF_DATA_BUFFERS;
            pPrevious->sensor2 &= ~OUT_OF_DATA_BUFFERS;
        }

		if( ( pCurrent->sensor2 & OUT_OF_DATA_BUFFERS2 ) == OUT_OF_DATA_BUFFERS2 ) {
            //PRINTF("compareStatus(): sending out of data buffers\r\n");  
            pCurrent->sensor2 &= ~OUT_OF_DATA_BUFFERS2;
            pPrevious->sensor2 &= ~OUT_OF_DATA_BUFFERS2;
        }
        
        /* set the prevoius to the current */
        memcpy( pPrevious, pCurrent, sizeof(PrStatusInfo) );
        /* SOF-5976 */
        sendPrStatus( pCurrent, true );
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
    //PRINTF("\r\n\r\n-----CREATING CHECKERBOARD LABEL LOCAL------\r\n\r\n");
    //unsigned short rowByteWidth = getHeadStyleSize() / 8;
    unsigned short rowByteWidth = PRINTER_HEAD_SIZE_80MM; /* was 72 */
    unsigned short centeringOffset = 0;
    unsigned char bmpPitch = BMP_PITCH;
    /* prepack printer 
    unsigned short centeringOffset = (getHeadStyleSize() - stockLoaded.getWidthDots()) / 2; */
    
    //unsigned short leftMostBit = centeringOffset + EDGE_MARGIN; 
    unsigned short leftMostBit = 0;
    unsigned short rightMostBit = leftMostBit + UFW_STOCK_WIDTH_DOTS - (8 * 2);
    //unsigned short rightMostBit = leftMostBit + WIDE_STOCK_WIDTH_DOTS - EDGE_MARGIN * 2;
    

    /* our buffer is only big enough for a 3" label*/
    if( length <= STEPS_PER_LENGTH3_00 ) {
      
        /* right most bit position cannot be bigger then the print head 
        if( rightMostBit > getHeadStyleSize() ) {
                rightMostBit = getHeadStyleSize();            
        }*/
        /* clear our pattern buffers  */
        memset( &pattern1[0], '\0', ( PRINTER_HEAD_SIZE_80MM * 2 ) ); 
        memset( &pattern2[0], '\0', ( PRINTER_HEAD_SIZE_80MM * 2 ) ); 
        memset( &pattern3[0], '\0', ( PRINTER_HEAD_SIZE_80MM * 2 ) );
        /*
        memset( &pattern1[0], '\0', ( 72 * 2 ) ); 
        memset( &pattern2[0], '\0', ( 72 * 2 ) );  */

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
        bool isPattern1 = false;
        /* copy the test patterns to the image buffer */
        while( rowCount < (length)) 
        {
            if(rowCount < 15)
            {
                memcpy( ( pImage + rowCount * rowByteWidth ), &pattern3[0], rowByteWidth );
            }
            else
            {
                if( isPattern1 ) 
                {
                    memcpy( ( pImage + rowCount * rowByteWidth ), &pattern1[0], rowByteWidth );
                } 
                else 
                {
                    memcpy(( pImage + rowCount * rowByteWidth ), &pattern2[0], rowByteWidth );
                }
            }
            
            
            rowCount++;

            if ( ( rowCount % 16 ) == 0 ) 
            {      
                isPattern1 = !isPattern1;
            }
        }

        /* clear our pattern buffers */
        memset( &pattern1[0], '\0', (PRINTER_HEAD_SIZE_80MM * 2 ) ); 
        memset( &pattern2[0], '\0', (PRINTER_HEAD_SIZE_80MM * 2 ) );
        memset( &pattern3[0], '\0', ( PRINTER_HEAD_SIZE_80MM * 2 ) );
        
        
        /*
        memset( &pattern1[0], '\0', ( 72 * 2 ) ); 
        memset( &pattern2[0], '\0', ( 72 * 2 ) );  */
        
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
        memset( &pattern1[0], '\0', ( PRINTER_HEAD_SIZE_72MM * 2 ) ); 
        memset( &pattern2[0], '\0', ( PRINTER_HEAD_SIZE_72MM * 2 ) ); 

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
        memset( &pattern1[0], '\0', (PRINTER_HEAD_SIZE_80MM * 2 ) ); 
        memset( &pattern2[0], '\0', (PRINTER_HEAD_SIZE_80MM * 2 ) ); 
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
        
    unsigned long rowByteWidth = PRINTER_HEAD_SIZE_72MM;
    
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
                    pImage[ rowCount * PRINTER_HEAD_SIZE_72MM ] = 0x10;
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
    unsigned long lineWidthBytes = PRINTER_HEAD_SIZE_72MM;
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
    if(getGapCalStatus() == true)
    {
        powerOnMotors();
    }
    else
    {
        powerOffMotors();
    }
  
    /* if this is the first time thru, do some initialization.*/
    if( currentStatus.state != ENGINE_IDLE )
    {
        if( currentStatus.command == 8 ){};
          //PRINTF("currentStatus.command: sizing complete\r\n" );
        if( currentStatus.command == 7 ){};
          //PRINTF("currentStatus.command: find complete\r\n" );

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
        
        if( (currentStatus.sensor2 & JAMMED_LABEL) == JAMMED_LABEL || (currentStatus.sensor & OUT_OF_MEDIA) == OUT_OF_MEDIA) 
        {
            //PRINTF("JAMMED LABEL IN IDLE OP\r\n");

            jammedBefore = true;    
            
            maxDeviationFromGapMinus = 0;
            maxDeviationFromGapPlus = 0;
            
            memset(labelLowCounts, 0, sizeof(labelLowCounts) );
            memset(labelLowCountsHistory, 0, sizeof(labelLowCountsHistory));
            memset(labelLowAverageCurrent, 0, sizeof(labelLowAverageCurrent));
            memset(labelLowAverageHistory, 0, sizeof(labelLowAverageHistory));
            
            prevLabelSize = getLabelSizeInQuarterSteps();
            
            firstLabelGap = 0;
            labelGapOnce_ = 0;
        }
        
        engine.steps = 0;
        setHeadTimings();
        powerOffMotors();
    } 
    else 
    { 
        if(getBackwindAfterContExpel() == true && getLabelTaken() <= LABEL_TAKEN_THRESHOLD_NO_LABEL)
        {
            setBackwindAfterContExpel(false);
            
            setHalfStepMode(_MAIN_STEPPER);
            setHalfStepMode(_TAKEUP_STEPPER);
          
            setTakeUpMotorDirection( FORWARDM_ ); 
            setMainMotorDirection( BACKWARDM_ );
            
            takeupDelay();
            
            if(getTakingUpPaper() == true)
            {
              loosenStock(300 + (getTensionModifier() / 4), 920);
            }
            
            backwindStock(150, 1100); 

            compareStatus(&currentStatus, &prevStatus);
        }
   
        
        if(getBackwindAfterSizing() == true && getLabelTaken() <= LABEL_TAKEN_THRESHOLD_NO_LABEL)
        {
            
            setHalfStepMode(_MAIN_STEPPER);
            setHalfStepMode(_TAKEUP_STEPPER);
          
            setTakeUpMotorDirection( FORWARDM_ ); 
            setMainMotorDirection( BACKWARDM_ );
          
            verticalOffsetIndex = config_.verticalPosition;

            setBackwindAfterSizing(false);
          
            takeupDelay();
  
            if(getTakingUpPaper() == true)
            {
                loosenStock(300, 1700);
                backwindStock(calculateSizingBackwindSteps(), 1300);
            }
            else
            {
                backwindStock(calculateSizingBackwindSteps(), 1500);
            }
        }
        
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
/*!   \fn void ( CmdOp *pOperation )

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
    static bool ltOnce_ = false;
    static bool expelDist_ = false;
    static bool roll_      = false;
    static int cnt_ = 0, rollTicks_ = 0;
    static bool rcrd_ = false;
    static unsigned long printLines = 0;
    
    unsigned long half_image_buffer_line_count = 0;
    
    unsigned short expelSteps = (getIndirectData( 4 ));
    
    if(getPrintHeadType() == ROHM_72MM_800_OHM)
    {
        printLines						= N_PRINTER_LINES_72MM;
		half_image_buffer_line_count 	= HALF_IMAGE_BUFFER_LINE_COUNT_72MM;
    }
    else
    {
        printLines						= N_PRINTER_LINES_80MM;
		half_image_buffer_line_count 	= HALF_IMAGE_BUFFER_LINE_COUNT_80MM;
    }
   
    /* clear if there was an error */
    currentStatus.error &= ~OUT_OF_PRINTDATA; 
    
        
    if( currentStatus.state != ENGINE_PRINTING ) 
    {
        //PRINTF("currentStatus.state: Entering ENGINE_PRINTING!\r\n" );
        //currentStatus.sensor &= ~LABEL_TAKEN;
        
        /* added to debug drift and correction */
        onceGap_ = false;
        onceTaken_ = false;
        ltOnce_ = false;


        /* added to do expel distance after printing */
        expelDist_ = false;
        cnt_ = 0;
        
        rollTicks_ = 0;
        rcrd_ = false;
        
        engine.steps = 0;
        engine.labelTracking = 0;
        
        /* clear the ghost missing label counter */
        clearGhostMCntr();

        shootCounts = getShootThroughBuffer();
        
        engine.numPrintLines = getOpData( pOperation->print.type, pOperation->print.data );
        engine.direction = ( engine.numPrintLines < 0 ) ? BACKWARD_ : FORWARD_;
        engine.numPrintLines = abs( engine.numPrintLines );
        engine.totalLinesToPrint = engine.numPrintLines;
        engine.labelOrientation = pOperation->print.orientation;
        
        //setStepperDirection( engine.direction  );
        powerOnStepper();
        
        tempDelayCounter = 0;
        
        if( engine.headType == UNKNOWN_HEAD ) 
        {
            currentStatus.error |= UNKNOWN_PH;
        } 
        else
        {
            currentStatus.error &= ~UNKNOWN_PH;
        }
        
        if( currentStatus.error == NO_ERROR ) 
        {
            currentStatus.state = ENGINE_PRINTING;
          
            startPrintEngine();
        }
   } 
   else 
   {
        //currentStatus.sensor &= ~LABEL_TAKEN;
     
        if(getLabelSizeInQuarterSteps() <= 1000)
        {
            if(getReadyToRecordShootVal() == true && shootIndex < 300)
            {
                setReadyToRecordShootVal(false);
                
                if(shootIndex < SHOOT_COUNT_ARRAY_SIZE)
                {
                    shootCounts[shootIndex] = pollMediaCounts();
                }
                
                if(labelLowSampleCounter == 3)
                {
                    labelLowCounts[shootIndex - labelLowIndexOffset] = getLowStockSensor();
                    labelLowSampleCounter = 0;
                }
                else
                {
                    labelLowSampleCounter++;
                    labelLowIndexOffset++;
                }
                
                shootIndex++;
            }
        }
        else
        {
            if(getReadyToRecordShootVal() == true)
            {
                setReadyToRecordShootVal(false);
                
                if(shootIndex < SHOOT_COUNT_ARRAY_SIZE)
                {
                    shootCounts[shootIndex] = pollMediaCounts();
                    
                    if(shootCounts[shootIndex] > (config_.backingAndlabel * 1.1))
                    {
                        shootCounts[shootIndex] = (config_.backingAndlabel * 1.1);
                    }
                }
                
                if(labelLowSampleCounter == 3)
                {
                    labelLowCounts[shootIndex - labelLowIndexOffset] = getLowStockSensor();
                    labelLowSampleCounter = 0;
                }
                else
                {
                    labelLowSampleCounter++;
                    labelLowIndexOffset++;
                }
                
                shootIndex++;
            }
        }
        
        /* are we printing a label bigger than our image buffer? */
        if( engine.totalLinesToPrint > printLines ) 
        {
  
            /* Once we use the first half of the image buffer tell the host so we can get more image data */
			if( ( engine.lineCounter == half_image_buffer_line_count && !paused_ ) ) 
                        {   
                
                //PRINTF("\r\nout of buffers 1\r\n");       
                
                /* notify the host that we're finished with the first half of the print buffer */
                currentStatus.sensor2 |= OUT_OF_DATA_BUFFERS; 
                compareStatus( &currentStatus, &prevStatus );
                paused_ = true;
#if 0
				GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, true);
#endif	
				
            }

			/*
			*	Have we used the second half of the image buffer?... before we ask for more data,
			*	make sure that we actually need it.
			*/
            if( engine.lineCounter == printLines && (engine.numPrintLines > half_image_buffer_line_count) )  
            {
                
				// notify host that we want image data to fill the second half (Buffer2)
				currentStatus.sensor2 |= OUT_OF_DATA_BUFFERS; 
				currentStatus.sensor2 |= OUT_OF_DATA_BUFFERS2; 
                
                                //PRINTF("\r\nout of buffers 2\r\n"); 
                                
				//myEngineLineCounterAtRollover++;
#if 0
				GPIO_WritePinOutput(STATUS_LED_GPIO, STATUS_LED_PIN, true);
#endif	
					
				compareStatus( &currentStatus, &prevStatus );
                engine.lineCounter = 0;

            }  
            
        }
		
        
        if( engine.linePrintDone == true && expelDone == true && getTPHIntrDone() == true && getTakeupBusy() == false) //print complete
        {                         
            paused_ = false;
            resetPrintDataLine();   
           
            
            if( continuousStock_ )  //continuous stock print complete
            {
                //currentStatus.sensor &= ~LABEL_TAKEN;
                compareStatus( &currentStatus, &prevStatus );
                setNextOperation(&currentStatus);
            }
            else //die cut print complete  
            {        
                if(getTakingUpPaper() == true)
                {
                    //currentStatus.sensor &= ~LABEL_TAKEN;
                    compareStatus( &currentStatus, &prevStatus );
                    setNextOperation(&currentStatus);
                } 
                else
                {      
                    //currentStatus.sensor &= ~LABEL_TAKEN;
                  
                    //find gap
                    int* shoots = getShootThroughBuffer();
                  
                    int startFilterIdx = 0;
                    int endFilterIdx = 24;
                    
                    while(endFilterIdx < (shootIndex))
                    {
                        averageAndStore(shoots, startFilterIdx, endFilterIdx);
                        startFilterIdx++;
                        endFilterIdx++;
                    }
                  
                    double desiredPercentage;
                    
                    desiredPercentage = 75;
                    
                    double result = find_percentage_of_average(shoots, shootIndex, desiredPercentage);

                    find_lowest_points_lowest(shoots, shootIndex, result);

                    TPHStepsPastGapThisPrint = ( getTPHStepsThisPrint() - getPrintDip() );
                    
                    if(TPHStepsPastGapThisPrint >= 600)
                    {
                        TPHStepsPastGapThisPrint = 600;
                    }
                    
                    if(getPrintDip() < 10)
                    {
                        TPHStepsPastGapThisPrint = 400;
                    }
                    
                    streamingLabelBackwind = 1;
                    
                    
                    
                    
                    
                    
                    //low label
                    double lowLabelThresholdValue = 0;
                    double lowLabelResult = 0;
                    
                    startFilterIdx = 0;
                    endFilterIdx = 31;
                    
                    while(endFilterIdx < (shootIndex / 4))
                    {
                        averageAndStore(labelLowCounts, startFilterIdx, endFilterIdx);
                        startFilterIdx++;
                        endFilterIdx++;
                    }
                       
                    addAndShift(labelLowCounts, labelLowCountsHistory, (shootIndex / 4), LABEL_LOW_HISTORY_SIZE);
                    
                    lowLabelThresholdValue = 99.0;
                    lowLabelResult = find_percentage_of_average(labelLowCountsHistory, LABEL_LOW_HISTORY_SIZE, lowLabelThresholdValue);
                    
                    labelLowAverageCurrent[0] = findDips(labelLowCountsHistory, LABEL_LOW_HISTORY_SIZE, lowLabelResult);
                    
                    addAndShift(labelLowAverageCurrent, labelLowAverageHistory, 1, 10);
                    
                    if(calculateAverage(labelLowAverageHistory, 10) >= 19)
                    {
                        currentStatus.sensor2 |= LOW_STOCK_REACHED;
                    }
                    
                    
                    
                    
                    
                    
                    
                    
                    //print debug                
                    /*
                    if(getPrintDip() > 1)
                    {
                        if(labelGapOnce_ == 0 && labelGapOnceChanged == false )
                        {
                            firstLabelGap = (getPrintDip());
                            
                            if(firstLabelGap < 0)
                            {
                                labelGapOnce_ = 0;
                            }
                            else
                            {
                                labelGapOnce_ = 1;
                            }
                        }
                        else
                        {
                            labelGapOnceChanged = false; 
                        }
                    }
                    */          
                    /*
                    if((maxDeviationFromGapPlus == 0 || maxDeviationFromGapMinus == 0) && deviationFlag == false)
                    {
                        if(((int)firstLabelGap - (int)getPrintDip()) > 0)
                        {
                            maxDeviationFromGapPlus = ((int)firstLabelGap - (int)getPrintDip());
                        }
                        else
                        {
                            maxDeviationFromGapMinus = ((int)firstLabelGap - (int)getPrintDip());
                        } 
                        
                        deviationFlag = true;
                    }
                    else if(((int)firstLabelGap - (int)getPrintDip()) > maxDeviationFromGapPlus)
                    {   
                        maxDeviationFromGapPlus = ((int)firstLabelGap - (int)getPrintDip());
                    }
                    else if(((int)firstLabelGap - (int)getPrintDip()) < maxDeviationFromGapMinus)
                    {
                        maxDeviationFromGapMinus = ((int)firstLabelGap - (int)getPrintDip());
                    }
                    */            
                    /*
                    PRINTF("\r\n");
                    PRINTF("shoot through counts post filter:");
                    PRINTF("\r\n");
                    
                    for(uint16_t ind = 0; ind < shootIndex; ind++)
                    {
                        PRINTF("%d,", shoots[ind]);
                        takeupDelayShort();
                    }
                    
                    PRINTF("\r\n");
                    */          
                    //PRINTF("\r\n\r\n");
                    //PRINTF("firstLabelGap = %d\r\n", firstLabelGap);
                    //PRINTF("calculated gap index during print = %d\r\n", getPrintDip());
                    //PRINTF("deviation from first label gap this print: %d\r\n", ((int)firstLabelGap - (int)getPrintDip()));
                    //PRINTF("max deviation + from first label gap = %d\r\n", maxDeviationFromGapPlus);
                    //PRINTF("max deviation - from first label gap = %d\r\n", maxDeviationFromGapMinus);
                    //PRINTF("label low AVG: %d\r\n", calculateAverage(labelLowAverageHistory, 10));
                    //PRINTF("printhead temperature = %d\r\n", getPrintheadTemperatureInCelsius());
                    
                    shootIndex = 0;
                    memset(shoots, 0, sizeof(&shoots));

                    //if the command option == 2 goto waitUntil, else goto idle
                    if(getWaitForLabelTaken() == false)
                    {
                        skipNextOperation( &currentStatus ); 
                        skipNextOperation( &currentStatus ); 
                        skipNextOperation( &currentStatus ); 
                        skipNextOperation( &currentStatus );
                    }
                    else
                    {
                        setNextOperation(&currentStatus);
                    }
                } 
            }   
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
        //PRINTF("historyAdjacency() pCurrentLine != null\r\n");
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
#if 0
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
            if(getTakingUpPaper() == true)
            {
                halfStepMotor();          
            }
            else
            {
                stepMainMotor();
            }
            
            motorStep( engine.direction, &currentStatus );            

        } else {
          
            setStepperDirection( FORWARD_ );
            powerOnStepper();
             /*changed to keep motor from stalling at 6ips when sizing labels -- ats 07102014*/
            if( ( engine.headType == KYOCERA753_OHM ) || ( engine.headType == KYOCERA800_OHM ) || 
                ( engine.headType == KYOCERA849_OHM ) || ( engine.headType == ROHM_72MM_800_OHM ) ) {
                delay(10000);         
            } else {
                engine.steps++;
                
                if(getTakingUpPaper() == true)
                {
                    halfStepMotor(); 
                }
                else
                {
                    stepMainMotor();
                }        
                
                motorStep( engine.direction, &currentStatus );
            }
        }
    } else {   

        /* if a freestanding scale then the media sensor has been removed */
        if( getMyModel() == RT_GLOBAL_FSS ) {
           
            /* need to leave this in to accurately size labels. */   
            int mediaValue = pollMediaCounts();//getMediaCounts();
            
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
            int mediaValue = pollMediaCounts();//getMediaCounts();
            
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
        if( getMyModel() == RT_GLOBAL_FSS ) {
            /* read label taken sensor */
            if( GPIO_ReadPinInput( LABEL_TAKEN_SENSOR_GPIO, LABEL_TAKEN_SENSOR_PIN ) ) { 
                currentStatus.sensor |= LABEL_TAKEN;
            } else {
                //currentStatus.sensor &= ~LABEL_TAKEN;
            }
        } else {
            /* read label taken sensor */            
            if( !readLabelTakenSensor() ) {
                currentStatus.sensor |= LABEL_TAKEN;
            } else {
                //currentStatus.sensor &= ~LABEL_TAKEN;                
            }
        }


        /* check for error conditions. */
        if( currentStatus.error != NO_ERROR ) {
            /* return the printer to the idle state */    
            powerOffStepper();
            setOperation( IDLE_DIRECTIVE, &currentStatus ); 
        } else if ( engine.numSteps-- <= 0 ) {
          
            /* notify host of any status change */
            compareStatus( &currentStatus, &prevStatus ); 
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
            
            if(getTakingUpPaper() == true)
            {
                halfStepMotor(); 
            }
            else
            {
                stepMainMotor();
            }             
        }
    }
/* notify host of any status change */
compareStatus( &currentStatus, &prevStatus );

setOperation( IDLE_DIRECTIVE, &currentStatus );

#endif

/*
if (currentStatus.state != ENGINE_STEPPING ) 
{       
    PRINTF("currentStatus.state: Entering Single ENGINE_STEPPING!\r\n" ); 
    
    currentStatus.state = ENGINE_STEPPING;
    
    PRINTF("stepOp type: %d\r\n", pOperation->type);
    PRINTF("stepOp data: %d\r\n", pOperation->data);
    
    if(getTakingUpPaper() == false)
    {
        backwindStock(streamingLabelBackwind, 1300);
    }
}    
*/   


compareStatus( &currentStatus, &prevStatus );

setOperation( IDLE_DIRECTIVE, &currentStatus );

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
            //currentStatus.sensor &= ~LABEL_TAKEN;
            
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
            
            if(getTakingUpPaper() == true)
            {
                halfStepMotor(); 
            }
            else
            {
                stepMainMotor();
            }    
            
            motorStep( engine.direction, &currentStatus );
        }
    } else {
        /* if a freestanding scale then the media sensor has been removed */
        if( getMyModel() == RT_GLOBAL_FSS ) {
            /* need to leave this in to accurately size labels. */   
            int mediaValue = pollMediaCounts();//getMediaCounts();
            
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
            int mediaValue = pollMediaCounts();//getMediaCounts();            
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
        }
        /* added for freestanding scale */
        if(  getMyModel() == RT_GLOBAL_FSS  ) {
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
            if( currentStatus.state == ENGINE_PRINTING || currentStatus.state == ENGINE_CALIBRATING )
                currentStatus.error |= MEDIA_SHUTDOWN;
            else        
                currentStatus.error &= ~MEDIA_SHUTDOWN;
        }
        
        /* added for freestanding scale */
        if( getMyModel() == RT_GLOBAL_FSS ) {        
            /* read label taken sensor */
            if( GPIO_ReadPinInput( LABEL_TAKEN_SENSOR_GPIO, LABEL_TAKEN_SENSOR_PIN ) ) { 
                currentStatus.sensor |= LABEL_TAKEN;
            } else {
                //currentStatus.sensor &= ~LABEL_TAKEN;
            }
        } else {        
            /* read label taken sensor */
            if( !readLabelTakenSensor() ) {
                currentStatus.sensor |= LABEL_TAKEN;                
            } else {
                //currentStatus.sensor &= ~LABEL_TAKEN;
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
            
            if(getTakingUpPaper() == true)
            {
                halfStepMotor(); 
            }
            else
            {
                stepMainMotor();
            }    
            
            /* notify host of any status change */
            compareStatus( &currentStatus, &prevStatus ); 
        } else {
                        
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
/*!   \fn void stepGapOp( StepUntilOperation *pOperation )
      \brief
        This function handles the teach table find the label gap command operation.
        
      \author
          Aaron Swift
*******************************************************************************/
void stepGapOp( StepUntilOperation *pOperation )
{   
    
    notSizing = false;
    deviationFlag = false;
    streamingLabelBackwind = 0;
    
    if( sizingWaitCount >= 1000 && (sizingAlreadyDone == false) )
    {
        if(pollMediaCounts() < (((float)config_.backingPaper * 0.94))) 
        {
            PRINTF("\r\n\r\nOUT OF STOCK NO SIZING\r\n\r\n");
            setLabelSizeInQuarterSteps(4999);
            
            sizingWaitCount = 0;
            
            currentStatus.sensor |= OUT_OF_MEDIA;
        }
        else
        {
            currentStatus.sensor2 &= ~JAMMED_LABEL;
            currentStatus.sensor2 &= ~LOW_STOCK_REACHED;
            currentStatus.sensor &= ~OUT_OF_MEDIA;
            
            maxDeviationFromGapMinus = 0;
            maxDeviationFromGapPlus = 0;
            
            memset(labelLowCounts, 0, sizeof(labelLowCounts) );
            memset(labelLowCountsHistory, 0, sizeof(labelLowCountsHistory));
            memset(labelLowAverageCurrent, 0, sizeof(labelLowAverageCurrent));
            memset(labelLowAverageHistory, 0, sizeof(labelLowAverageHistory));
            
            prevLabelSize = getLabelSizeInQuarterSteps();
            sizingLabels = true;
            
            backwindOffsetCount = 0;
          
            currentStatus.counter = 0;
            engine.steps = 0;
            
            firstLabelGap = 0;
            labelGapOnce_ = 0;
          
            uint16_t stepsToLt = 2999;
            uint16_t stepsToSize = 10000;
            
            
            if(getTakingUpPaper() == true)
            {
              tightenStock(((float)config_.takeup_sensor_min_tension_counts * 1.0), 920, false, HALF_STEP);
            }

            //takeupDelay();

            stepToLt(stepsToLt, 575);

            sizeLabels(stepsToSize, 575);
            
            //takeupDelay();
            
            //#define GAP_SENSOR_TO_PEEL_BAR_GT 900
            //#define GAP_SENSOR_TO_PEEL_BAR_HT 840
            //#define GAP_SENSOR_TO_TEAR_BAR_GT 995
            //#define GAP_SENSOR_TO_TEAR_BAR_HT 940
            
            if(getLargeGapFlag() == true)
            {
                if(getTakingUpPaper() == true)
                {
                    PRINTF("stepsBackToGap - %d\r\n", getStepsBackToGap());
                    PRINTF("firstGapIndex - %d\r\n", getFirstGapIndex());
                    PRINTF("secondGapIndex - %d\r\n", getSecondGapIndex());
                    PRINTF("labelSizeInQuarterSteps - %d\r\n", getLabelSizeInQuarterSteps());
                    stepToNextLabel(GAP_SENSOR_TO_PEEL_BAR_HT - getStepsBackToGap(), 575);
                }
                else
                {
                    PRINTF("stepsBackToGap - %d\r\n", getStepsBackToGap());
                    PRINTF("firstGapIndex - %d\r\n", getFirstGapIndex());
                    PRINTF("secondGapIndex - %d\r\n", getSecondGapIndex());
                    PRINTF("labelSizeInQuarterSteps - %d\r\n", getLabelSizeInQuarterSteps());
                    stepToNextLabel(GAP_SENSOR_TO_TEAR_BAR_HT - getStepsBackToGap(), 575);
                }
            }
            else
            {
                if(getTakingUpPaper() == true)
                {
                    PRINTF("stepsBackToGap - %d\r\n", getStepsBackToGap());
                    PRINTF("firstGapIndex - %d\r\n", getFirstGapIndex());
                    PRINTF("secondGapIndex - %d\r\n", getSecondGapIndex());
                    PRINTF("labelSizeInQuarterSteps - %d\r\n", getLabelSizeInQuarterSteps());
                    stepToNextLabel(GAP_SENSOR_TO_PEEL_BAR_GT - getStepsBackToGap(), 575);
                }
                else
                {
                    PRINTF("stepsBackToGap - %d\r\n", getStepsBackToGap());
                    PRINTF("firstGapIndex - %d\r\n", getFirstGapIndex());
                    PRINTF("secondGapIndex - %d\r\n", getSecondGapIndex());
                    PRINTF("labelSizeInQuarterSteps - %d\r\n", getLabelSizeInQuarterSteps());
                    stepToNextLabel(GAP_SENSOR_TO_TEAR_BAR_GT - getStepsBackToGap(), 575);
                }
            }
            
            
            backwindAfterSizing = true;
        }
        
        
        sizingAlreadyDone = true;
        streamingLabelBackwind = 0;
        
        //setNextOperation( &currentStatus );
    }
    else
    {
        sizingWaitCount++;
        //PRINTF("%d\r\n", sizingWaitCount);
    }
    
    if((jammedBefore == true && sizingAlreadyDone == true) || getTakingUpPaper() == true && sizingAlreadyDone == true)
    {
        //PRINTF("JAMMED_BEFORE IN SIZING\r\n");
        //PRINTF("LT: %d\r\n", getLabelTaken());
      
        //currentStatus.sensor &= ~LABEL_TAKEN;
        compareStatus( &currentStatus, &prevStatus );
        
        if(getLabelTaken() <= 150)
        {
            //PRINTF("WAITED FOR LT AFTER POST JAM SIZING\r\n");
            sizingAlreadyDone = false;
            sizingWaitCount = 0;
            setNextOperation( &currentStatus );
        }
    }
    else if(jammedBefore == false && sizingAlreadyDone == true)
    {
        //PRINTF("NOT WAITING FOR LT\r\n");
        sizingAlreadyDone = false;
        sizingWaitCount = 0;
        setNextOperation( &currentStatus );
    }
      
    
    
#if 0
    if( currentStatus.state != ENGINE_STEPPING ) {
        PRINTF("currentStatus.state: Entering Single ENGINE_STEPPING_UNTIL_GAP!\r\n" );
        
        /* pause the adc thread and control manual 
        pauseResumeConversions( true );*/
        
        currentStatus.state = ENGINE_STEPPING;
  
        /* notify host of state change */
        compareStatus( &currentStatus, &prevStatus ); 
        
        /* have the sensor ISR check and stop motor on the gap */
        //setMotorStopOnGap();
        
        engine.steps = 0;
                
        powerOnStepper();

        /*we are sizing a label, slow sizing speed to 3ips  */ 
        currentStatus.sensor |= MOTOR_FORWARD;
        //currentStatus.sensor &= ~LABEL_TAKEN;
            
        setPrintEngineTimer( getSltSizingTime( engine.headType ) );
             
        engine.numSteps = getOpData(pOperation->type, pOperation->data);
        engine.direction = ( engine.numSteps < 0 ) ? BACKWARD_ : FORWARD_;           
        
        engine.numSteps = abs(engine.numSteps);
        engine.numSteps = engine.numSteps << 1;
        
        /* PRINTF( "number of steps allowed: %d\r\n",  engine.numSteps ); */
        /* initializeStepper( engine.direction ); */
        //setStepperDirection( engine.direction );
        /* removed to keep motor from stalling at 6ips when sizing labels -- ats 07102014 */	     
        if( ( engine.headType == KYOCERA753_OHM ) || ( engine.headType == KYOCERA800_OHM ) ||            
            ( engine.headType == KYOCERA849_OHM ) ) {
            engine.steps++;
            
            if(getTakingUpPaper() == true)
            {
                halfStepMotor(); 
            }
            else
            {
                stepMainMotor();
            }    
            
            motorStep( engine.direction, &currentStatus );
        }
    } else {
        /* if a freestanding scale then the media sensor has been removed */
        int mediaValue = pollMediaCounts();//getMediaCounts();     /* was: pollMediaCounts(); */ 
        
        
        if( mediaValue <= BACKING_PAPER_THRESHOLD  ) {
             //PRINTF( "stepGapOp() gap count: %d\r\n",  mediaValue ); 
            if( ( currentStatus.sensor & MOTOR_FORWARD ) == MOTOR_FORWARD ){
                pCurrentLabel = pCurrentLabel->next;
                currentStatus.sensor |= SYNCHRONIZED;    
            } else {
                if( ( currentStatus.sensor & SYNC_BAR ) != SYNC_BAR ) {
                    currentStatus.sensor |= SYNC_BAR;  
                    currentStatus.sensor &= ~SYNCHRONIZED;
                    pCurrentLabel->next->position = 0;
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
                //currentStatus.sensor &= ~LABEL_TAKEN;
            }
        }
        /* check for error conditions. */
        if( currentStatus.error != NO_ERROR ) {
            /* return the printer to the idle state */    
            powerOffStepper();
            setOperation( IDLE_DIRECTIVE, &currentStatus ); 
        } /* Chris: remove this for debug of shoot through sensor */
          else if ( ( testCondition( &currentStatus, 
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
            if(getTakingUpPaper() == true)
            {
                halfStepMotor(); 
            }
            else
            {
                stepMainMotor();
            }    
            
            /* notify host of any status change */
            compareStatus( &currentStatus, &prevStatus ); 
        } else {
            
            //clrMotorStopOnGap();
          
            /* Chris: add this for debug of shoot through sensor         
            ltCounts[ltIndex++] = (unsigned short)mediaValue;
            for( int i = 0; i < 4000; i++ ) {
                PRINTF("%d\r\n", ltCounts[i] ); 
            }  */                

            /* pause the adc thread and control manual
            pauseResumeConversions( false );*/ 
            
            PRINTF( "stepGapOp() pOperation->operator: %d\r\n",  pOperation->operator );
            PRINTF( "stepGapOp() pOperation->bits: %d\r\n",  pOperation->bits );
            PRINTF( "stepGapOp() pOperation->result: %d\r\n",  pOperation->result );
            
            PRINTF( "steps: %d\r\n",  engine.steps );
            PRINTF( "engine.numSteps: %d\r\n",  engine.numSteps );
            /* stepping complete */
            resetPrintEngineTimer();
            powerOffStepper(); 

            /* have we detected continuous stock while sizing */
            if( engine.steps > CONTINUOUS_STOCK_MIN ) {
                /* jump to the end of sizing since we know the stock size already 
                   don't set continuous bit until size message from host */
                jumpToOperation( &currentStatus,  10 );
                
            } else {
                setNextOperation( &currentStatus ); 
            }
            
        }
    }	
    
    
#endif    
    

}

/******************************************************************************/
/*!   \fn void detectionOp( StepUntilOperation *pOperation )
      \brief
        This function handles the setting up the label stock to a known position
        before trying to size.
        
      \author
          Aaron Swift
*******************************************************************************/
void detectionOp( StepUntilOperation *pOperation )
{
    static bool findGap_ = false;
    
    if( currentStatus.state != ENGINE_STEPPING ) {
        PRINTF("currentStatus.state: Entering Single ENGINE_DETECTION_OP!\r\n" );
        
        /* pause the adc thread and control manual 
        pauseResumeConversions( true );*/
        
        currentStatus.state = ENGINE_STEPPING;
  
        /* notify host of state change */
        compareStatus( &currentStatus, &prevStatus ); 
        
        mainMotorStopped_ = false;
        engine.steps = 0;
           
        powerOnStepper();

        /*we are sizing a label, slow sizing speed to 3ips  */ 
        currentStatus.sensor |= MOTOR_FORWARD;
        //currentStatus.sensor &= ~LABEL_TAKEN;
            
        setPrintEngineTimer( getSltSizingTime( engine.headType ) );

        /* determine if we are at rest on gap or label */
        int mediaValue = getMediaCounts(); 
        if( mediaValue > BACKING_PAPER_THRESHOLD  ) {
            findGap_ = true;
            /* have the sensor ISR check and stop motor on the gap */
            setMotorStopOnGap();
            PRINTF( "detectionOp() drive to gap.\r\n");
        } else {
            /* have the sensor ISR check and stop motor on edge */
            setMotorStopOnEdge();
            PRINTF( "detectionOp() drive to label edge.\r\n");
        }
        
        
        engine.numSteps = getOpData(pOperation->type, pOperation->data);
        engine.direction = ( engine.numSteps < 0 ) ? BACKWARD_ : FORWARD_;    
        
        engine.numSteps = abs(engine.numSteps);
        engine.numSteps = engine.numSteps << 1;
        
        setStepperDirection( engine.direction );
        
    } else {
        int mediaValue = getMediaCounts();
        int even = 0;
        
        if( findGap_ ) {
            /* keep stepping until the isr sets the flag or we run out of steps */
            if( ( !mainMotorStopped_ ) &&  ( --engine.numSteps > 0 ) ){
                even = engine.numSteps & 0x0001;
                if( even == 0 ) {
                    /* two half steps per print line */            
                    motorStep( engine.direction, &currentStatus );
                }
                engine.steps++;
                if( getTakingUpPaper() == true ) {
                    halfStepMotor(); 
                } else {
                    stepMainMotor();
                }                    
            } else {                
                clrMotorStopOnGap();
                PRINTF( "detectionOp() steps: %d\r\n", engine.steps );
                /* have we detected continuous stock while sizing */
                if( engine.steps > CONTINUOUS_STOCK_MIN ) {
                    /* jump to the end of sizing since we know the stock size already 
                      don't set continuous bit until size message from host */
                    jumpToOperation( &currentStatus,  10 );                
                } else {
                    skipNextOperation( &currentStatus ); 
                }          
            }   
            /* notify host of any status change */
            compareStatus( &currentStatus, &prevStatus );                                         
        } else {
            /* keep stepping until the isr sets the flag or we run out of steps */
            if( ( !mainMotorStopped_ ) &&  ( --engine.numSteps > 0 ) ){
                even = engine.numSteps & 0x0001;
                if( even == 0 ) {
                    /* two half steps per print line */            
                    motorStep( engine.direction, &currentStatus );
                }
                engine.steps++;
                if( getTakingUpPaper() == true ) {
                    halfStepMotor(); 
                } else {
                    stepMainMotor();
                }                    
            } else {
              
                clrMotorStopOnGap();
                /* stepping complete */
                resetPrintEngineTimer();
                /* have we detected continuous stock while sizing */
                if( engine.steps > CONTINUOUS_STOCK_MIN ) {
                    /* jump to the end of sizing since we know the stock size already 
                      don't set continuous bit until size message from host */
                    jumpToOperation( &currentStatus,  10 );                
                } else {
                    skipNextOperation( &currentStatus ); 
                }          
            }   
            /* notify host of any status change */
            compareStatus( &currentStatus, &prevStatus );                                                 
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
#if 0
    int even = 0;
    if( currentStatus.state != ENGINE_STEPPING ) {
        PRINTF("currentStatus.state: Entering Single STEP_EDGE_DIRECTIVE!\r\n" );
        
        /* only for freestanding scale */
        if( getMyModel() == RT_GLOBAL_FSS ) {        
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

        engine.numSteps = 0;
        engine.steps = 0;    
        powerOnStepper();

        /*we are sizing a label, slow sizing speed to 3ips  */ 
        currentStatus.sensor |= MOTOR_FORWARD;
        //currentStatus.sensor &= ~LABEL_TAKEN;
        currentStatus.sensor &= ~SYNCHRONIZED;
        
        setPrintEngineTimer( getSltSizingTime( engine.headType ) );
             
        engine.numSteps = getOpData(pOperation->type, pOperation->data);
        engine.direction = ( engine.numSteps < 0 ) ? BACKWARD_ : FORWARD_;    
        engine.numSteps = abs(engine.numSteps);
        engine.numSteps = engine.numSteps << 1;
        
        /* PRINTF( "number of steps allowed: %d\r\n",  engine.numSteps ); */        
        setStepperDirection( engine.direction );
        /* removed to keep motor from stalling at 6ips when sizing labels -- ats 07102014 */	     
        if( ( engine.headType == KYOCERA753_OHM ) || ( engine.headType == KYOCERA800_OHM ) ||            
            ( engine.headType == KYOCERA849_OHM ) ) {
            engine.steps++;
            if(getTakingUpPaper() == true)
            {
                halfStepMotor(); 
            }
            else
            {
                stepMainMotor();
            }    
            
            motorStep( engine.direction, &currentStatus );
        }
    } else {
        /* if a freestanding scale then the media sensor has been removed */
        int mediaValue = pollMediaCounts();//getMediaCounts();
        
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
                    PRINTF("stepEdgeOp(): clear SYNCHRONIZED bit\r\n" );
                    PRINTF("stepEdgeOp(): engine.steps %d\r\n", engine.steps ); 
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
                //currentStatus.sensor &= ~LABEL_TAKEN;
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

                even = engine.numSteps & 0x0001;

                if( even == 0 ) {
                    /* two half steps per print line */            
                    motorStep( engine.direction, &currentStatus );
                }
                
                engine.steps++;
                if(getTakingUpPaper() == true)
                {
                    halfStepMotor(); 
                }
                else
                {
                    stepMainMotor();
                }    
                
                /* notify host of any status change 
                compareStatus( &currentStatus, &prevStatus ); */
            } else {
                
                clrMotorStopOnEdge();
                PRINTF( "stepEdgeOp() mediaValue: %d\r\n",  mediaValue );
                PRINTF( "stepEdgeOp() pOperation->operator: %d\r\n",  pOperation->operator );
                PRINTF( "stepEdgeOp() pOperation->bits: %d\r\n",  pOperation->bits );
                PRINTF( "stepEdgeOp() pOperation->result: %d\r\n",  pOperation->result );

                PRINTF( "steps: %d\r\n",  engine.steps );
                PRINTF( "engine.numSteps: %d\r\n",  engine.numSteps );
                /* stepping complete */
                resetPrintEngineTimer();
                powerOffStepper();
                                        
                setNextOperation( &currentStatus );            
            }
        } else {
            even = engine.numSteps & 0x0001;

            if( even == 0 ) {
                /* two half steps per print line */            
                motorStep( engine.direction, &currentStatus );
            }
            
            engine.steps++;
            if(getTakingUpPaper() == true)
            {
                halfStepMotor(); 
            }
            else
            {
                stepMainMotor();
            }            
        }
    }	
#endif
}

void stepTakeupTightenOp( StepOperation *pOperation ) 
{
    if (currentStatus.state != ENGINE_STEPPING ) {       
        PRINTF("currentStatus.state: Entering Tighten ENGINE_STEPPING!\r\n" );  
        /* set the state and enable the stepper motor*/       
        currentStatus.state = ENGINE_STEPPING;
        
        powerOnStepper();
        
        /* sync print line with the step rate */
        setPrintEngineTimerSlt();
        
        engine.numSteps = getOpData( pOperation->type, pOperation->data );
        engine.direction = ( engine.numSteps < 0 ) ? BACKWARD_ : FORWARD_;    
        engine.numSteps = abs(engine.numSteps);
        /* printer has two half steps per print line */
        engine.numSteps = engine.numSteps << 1;
                
        setTakeUpMotorDirection( BACKWARDM_ );
        
        /* sync print line with the step rate */ 
        resetPrintEngineTimer();
        engine.steps++;
        if(getTakingUpPaper() == true)
        {  
            stepTakeUpMotor();                            
        }
    } else {   
        /* check for error conditions. */
        if( currentStatus.error != NO_ERROR ) {
            /* return the printer to the idle state */    
            powerOffStepper();
            setOperation( IDLE_DIRECTIVE, &currentStatus ); 
        } else if( ( engine.numSteps-- <= 0 ) ) {
           
            if( getTakingUpPaper() == true ) {
                tightenStock(2200, 500, false, HALF_STEP);// this function blocks within it now, so does loosen and backwind
            }
            
            /* sync the print timer to the motor steps. */
            resetPrintEngineTimer();
            setNextOperation( &currentStatus );
        } else {
          
            int even = 0;

            even = engine.numSteps & 0x0001;
            if( even == 0 ) {
                /* Two half steps per printline */  
                if(getTakingUpPaper() == true) {
                    stepTakeUpMotor();                            
                }
            }
            engine.steps++;
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
    /* not supported in global scale. global scale stock has no sync bars */
    setNextOperation( &currentStatus ); 
#if 0    
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
        //currentStatus.sensor &= ~LABEL_TAKEN;
            
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
        if( getMyModel() == RT_GLOBAL_FSS ) {
            /* return the printer to the idle state */    
            powerOffStepper();
            setOperation( IDLE_DIRECTIVE, &currentStatus );            
        } else {           
            int mediaValue = pollMediaCounts();//getMediaCounts();
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
                /* at the label edge */
                setNextOperation( &currentStatus ); 
            } else {
                PRINTF("testForSyncOp(): no sync bar, label edge found!\r\n" ); 
                PRINTF("testForSyncOp(): engine.steps %d\r\n", engine.steps );
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
            
            if(getTakingUpPaper() == true)
            {
                halfStepMotor(); 
            }
            else
            {
                stepMainMotor();
            }    
            
            /* notify host of any status change */
            compareStatus( &currentStatus, &prevStatus ); 
        }
    }
#endif    
}

/******************************************************************************/
/*!   \fn void testForLabelOp( StepOperation *pOperation )
      \brief
        The function will drive out 1/2" looking for the label taken sensor 
        to be blocked. The function will then 
        proceed to the label edge before exiting. If the function does not 
        encounter a gap then we have driven 1/2" into the label which we will 
        save the 1/2" distance to be used for our sizing measurement and then 
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
        //currentStatus.sensor &= ~LABEL_TAKEN;
            
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
        int mediaValue = pollMediaCounts();//getMediaCounts();   
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
            
            if(getTakingUpPaper() == true)
            {
                halfStepMotor(); 
            }
            else
            {
                stepMainMotor();
            }     
            
            /* notify host of any status change */
            compareStatus( &currentStatus, &prevStatus ); 
        }
    }    
}

/******************************************************************************/
/*!   \fn void testForContinuous( StepOperation *pOperation )
      \brief
        The function will test for continuous stock bit and if continuous then 
        step to the expel distance. Otherwise go to the next function.
         
      \author
          Aaron Swift
*******************************************************************************/
void testForContinuous( StepOperation *pOperation ) 
{
    if( continuousStock_ ) {
        if( currentStatus.state != ENGINE_STEPPING ) {    
            PRINTF("currentStatus.state: Entering Single TEST_FOR_CONTINUOUS!\r\n" );  
        
            /* set the state and enable the stepper motor*/       
            currentStatus.state = ENGINE_STEPPING;
            
            powerOnStepper();
            
            /* get my expel position */             
            engine.numSteps = getOpData( pOperation->type, pOperation->data );
            engine.direction = ( engine.numSteps < 0 ) ? BACKWARD_ : FORWARD_;    
            engine.numSteps = abs(engine.numSteps);
            engine.numSteps = engine.numSteps << 1;
           
            setStepperDirection( engine.direction );
        } else {
            if( currentStatus.error != NO_ERROR ) {
                /* return the printer to the idle state */    
                powerOffStepper();
                setOperation( IDLE_DIRECTIVE, &currentStatus ); 
            } else if ( engine.numSteps-- <= 0 ) {
                /* stepping complete */
                //resetPrintEngineTimer();
                powerOffStepper();
                                        
                setNextOperation( &currentStatus );                        
            } else {
                int even = engine.numSteps & 0x0001;
                if( even == 0 ) {
                    /* Two half steps per printline */  
                    motorStep( engine.direction, &currentStatus );
                }
                engine.steps++;
                
                if(getTakingUpPaper() == true)
                {
                    halfStepMotor(); 
                }
                else
                {
                    stepMainMotor();
                }    
                
                /* notify host of any status change */
                compareStatus( &currentStatus, &prevStatus );                     
            }
        }      
    } else {
        PRINTF("currentStatus.state: Skip TEST_FOR_CONTINUOUS!\r\n" ); 
        setNextOperation( &currentStatus ); 
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
    currentStatus.state = ENGINE_WAITING;
  
    if(getLabelTaken() <= LABEL_TAKEN_THRESHOLD_NO_LABEL)
    { 
        if(getTakingUpPaper() == true)
        {
            //find gap
            int* shoots = getShootThroughBuffer();
            
            int startFilterIdx = 0;
            int endFilterIdx = 24;
            
            while(endFilterIdx < (shootIndex))
            {
                averageAndStore(shoots, startFilterIdx, endFilterIdx);
                startFilterIdx++;
                endFilterIdx++;
            }
            
            double desiredPercentage;
            
            desiredPercentage = 75;
            
            double result = find_percentage_of_average(shoots, shootIndex, desiredPercentage);
            
            find_lowest_points_lowest(shoots, shootIndex, result);
            
            TPHStepsPastGapThisPrint = ( getTPHStepsThisPrint() - getPrintDip() );
            
            if(TPHStepsPastGapThisPrint >= 600)
            {
                TPHStepsPastGapThisPrint = 600;
            }
            
            if(getPrintDip() < 10)
            {
                TPHStepsPastGapThisPrint = 400;
            }
            
            streamingLabelBackwind = 1;
            
            
            
            
            
            
            //low label
            double lowLabelThresholdValue = 0;
            double lowLabelResult = 0;
            
            startFilterIdx = 0;
            endFilterIdx = 31;
            
            while(endFilterIdx < (shootIndex / 4))
            {
                averageAndStore(labelLowCounts, startFilterIdx, endFilterIdx);
                startFilterIdx++;
                endFilterIdx++;
            }
               
            addAndShift(labelLowCounts, labelLowCountsHistory, (shootIndex / 4), LABEL_LOW_HISTORY_SIZE);
            
            lowLabelThresholdValue = 99.0;
            lowLabelResult = find_percentage_of_average(labelLowCountsHistory, LABEL_LOW_HISTORY_SIZE, lowLabelThresholdValue);
            
            labelLowAverageCurrent[0] = findDips(labelLowCountsHistory, LABEL_LOW_HISTORY_SIZE, lowLabelResult);
            
            addAndShift(labelLowAverageCurrent, labelLowAverageHistory, 1, 10);
            
            if(calculateAverage(labelLowAverageHistory, 10) >= 19)
            {
                currentStatus.sensor2 |= LOW_STOCK_REACHED;
            }

            

          

            


            
            
            
            //peeling backwind
            if(continuousStock_)
            {                      
                if(getTakingUpPaper() == true) 
                {
                    //takeupDelayShort();
                    //loosenStock(300, 1700);

                    //backwindStock(150, 1500);
                }
                else 
                {
                    //takeupDelayShort();
                    //backwindStock(150, 1500);
                }
            }
            else
            {
                takeupDelayShort();
                
                if(getTakingUpPaper() == true)
                {
                    //loosenStock(300, 1700);
                    //backwindStock(calculatePeelingBackwindSteps() , 1500);
                } 
            }
            
           
           
           
           
           
            //print debug
            /*
            if(getPrintDip() > 1)
            {
                if(labelGapOnce_ == 0 && labelGapOnceChanged == false )
                {
                    firstLabelGap = (getPrintDip());
                    
                    if(firstLabelGap < 0)
                    {
                        labelGapOnce_ = 0;
                    }
                    else
                    {
                        labelGapOnce_ = 1;
                    }
                }
                else
                {
                    labelGapOnceChanged = false; 
                }
            }
            */
            /*
            if((maxDeviationFromGapPlus == 0 || maxDeviationFromGapMinus == 0) && deviationFlag == false)
            {
                if(((int)firstLabelGap - (int)getPrintDip()) > 0)
                {
                    maxDeviationFromGapPlus = ((int)firstLabelGap - (int)getPrintDip());
                }
                else
                {
                    maxDeviationFromGapMinus = ((int)firstLabelGap - (int)getPrintDip());
                }    
                
                deviationFlag = true;
            }
            else if(((int)firstLabelGap - (int)getPrintDip()) > maxDeviationFromGapPlus)
            {   
                maxDeviationFromGapPlus = ((int)firstLabelGap - (int)getPrintDip());
            }
            else if(((int)firstLabelGap - (int)getPrintDip()) < maxDeviationFromGapMinus)
            {
                maxDeviationFromGapMinus = ((int)firstLabelGap - (int)getPrintDip());
            }
            */
            /*
            PRINTF("\r\n");
            PRINTF("shoot through counts post filter:");
            PRINTF("\r\n");
            
            for(uint16_t ind = 0; ind < shootIndex; ind++)
            {
                PRINTF("%d,", shoots[ind]);
                takeupDelayShort();
            }
            PRINTF("\r\n");
            */
            //PRINTF("first label gap = %d\r\n", firstLabelGap);
            //PRINTF("calculated gap index during print = %d\r\n", getPrintDip());
            //PRINTF("deviation from first label gap this print: %d\r\n", ((int)firstLabelGap - (int)getPrintDip()));
            //PRINTF("max deviation + from first label gap = %d\r\n", maxDeviationFromGapPlus);
            //PRINTF("max deviation - from first label gap = %d\r\n", maxDeviationFromGapMinus);
            //PRINTF("label low AVG: %d\r\n", calculateAverage(labelLowAverageHistory, 10));
            //PRINTF("printhead temperature = %d\r\n", getPrintheadTemperatureInCelsius());
            
           
            shootIndex = 0;
            memset(shoots, 0, sizeof(&shoots));
            
            setOperation( IDLE_DIRECTIVE, &currentStatus );
        }
        else
        {
            setOperation( IDLE_DIRECTIVE, &currentStatus );
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
        PRINTF("currentStatus.state: Entering ENGINE_WAITING_UNTIL_SIZING!\r\n" );
        
        /* not supported in global better scale ( no cutter ). */
        //setNextOperation( &currentStatus ); 
        
        #if 0         
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
        #endif
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
                                //motorStep( engine.direction, &currentStatus );
                            }                            
                            
                            if(getTakingUpPaper() == true)
                            {
                                //halfStepMotor(); 
                            }
                            else
                            {
                                //stepMainMotor();
                            }    
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
    //PRINTF("currentStatus.state: Entering ENGINE_STATUS!\r\n" );
    if( pOperation->operator == SET_BITS ) {
        currentStatus.user |= pOperation->bits;
        if( ( currentStatus.user & TAKE_LABEL ) == TAKE_LABEL ) {
          PRINTF("statusOp(): setting take label bit\r\n" );
        } else if( ( currentStatus.user & MISSING_LABEL ) == MISSING_LABEL ) {
            PRINTF("statusOp(): setting missing label bit\r\n" );
            currentStatus.user = 0; 
        } else if( ( currentStatus.user & JAMMED_LABEL ) == JAMMED_LABEL ) {
            PRINTF("statusOp(): setting jammed label bit\r\n" );
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
    //PRINTF("currentStatus.state: Entering ENGINE_COUNTER!\r\n" );
    switch( pOperation->operator )
    {
        case RESET_COUNTER: {
            //PRINTF("counterOp(): reset counter!\r\n" );
            currentStatus.counter = 0;
            engine.steps = 0;
            break;
        }
        case ENABLE_COUNTER: {
            //PRINTF("counterOp(): enable counter\r\n" );
            //PRINTF("counterOp(): currentStatus.counter: %d\r\n", currentStatus.counter );
            //stepCounterEnabled_ = true;
            currentStatus.counter = 0;
            engine.steps = 0;
            break;
        }
        case DISABLE_COUNTER: {
            //PRINTF("counterOp(): disable counter\r\n" );
            
            /* we are keeping track of half steps and sizing needs whole steps. */
            engine.steps = getLabelSizeInQuarterSteps();
            //engine.steps = 1460; //test value

            currentStatus.counter = (  engine.steps / 2 );

            
            /* if we saved off steps due to sync bar test then add it back into grand total.
            if( engine.stepsOffset != 0 ) {
                PRINTF("counterOp(): whole steps offset: %d\r\n", engine.stepsOffset );
                currentStatus.counter +=  calculateSizingOffset(currentStatus.counter); 
                engine.stepsOffset = 0;
            } */
            
            //PRINTF("counterOp(): half steps currentStatus.counter: %d\r\n", engine.steps );
            //PRINTF("counterOp(): whole steps currentStatus.counter: %d\r\n", currentStatus.counter );
            stepCounterEnabled_ = false;
            
            //currentStatus.counter = 0;
            //engine.steps = 0;
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
           
            unsigned long cnts = pollMediaCounts();//getMediaCounts();
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
/*!   \fn void stepTakeupOp( StepUntilOperation *pOperation )
      \brief
        This function handles the teach table test of paper takeup command operation.
        
      \author
          Aaron Swift
*******************************************************************************/
void stepTakeupOp( StepUntilOperation *pOperation )
{
    static int index_ = 0;
    if( currentStatus.state != ENGINE_STEPPING ) {
        PRINTF("currentStatus.state: Entering Single ENGINE_STEPPING_UNTIL_TAKEUP!\r\n" );
                
        currentStatus.state = ENGINE_STEPPING;
                  
        powerOnStepper();

        currentStatus.sensor |= MOTOR_FORWARD;
        //currentStatus.sensor &= ~LABEL_TAKEN;
            
        setPrintEngineTimer( getSltSizingTime( engine.headType ) );
             
        engine.numSteps = getOpData(pOperation->type, pOperation->data);
        engine.direction = ( engine.numSteps < 0 ) ? BACKWARD_ : FORWARD_;    
        
        engine.direction = BACKWARD_;
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
              
            if(getTakingUpPaper() == true)
            {   
                stepTakeUpMotor();         	 
            }
            
            motorStep( engine.direction, &currentStatus );
        }
    } else {
        /* if a freestanding scale then the media sensor has been removed */
        unsigned short torque = getTakeUpTorque();
        PRINTF( "torque: %d\r\n",  torque );
        currentStatus.error = NO_ERROR;
        /* check for error conditions. */
        if( currentStatus.error != NO_ERROR ) {
            /* return the printer to the idle state */    
            powerOffStepper();
            setOperation( IDLE_DIRECTIVE, &currentStatus ); 
        } else if( torque <= 2500 ) {
        /*
        else if ( ( testCondition( &currentStatus, 
                  pOperation->operator, 
                  pOperation->bits, 
                  pOperation->result) == false) 
                  &&  ( --engine.numSteps > 0 ) ) {    */

            int even = 0;
            even = engine.numSteps & 0x0001;
            if(getTakingUpPaper() == true)
            {
                if( even == 0 ) {
                    /* two half steps per print line */            
                    motorStep( engine.direction, &currentStatus );
                }
                
                engine.steps++;
                stepTakeUpMotor(); 
            }
            /* notify host of any status change */
            compareStatus( &currentStatus, &prevStatus ); 
        } else {                                  
            PRINTF( "torque: %d\r\n",  torque );
            PRINTF( "engine.numSteps: %d\r\n",  engine.numSteps );
            /* stepping complete */
            resetPrintEngineTimer();
            powerOffStepper();

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
    unsigned long buffSize = 0;
    
    if(getPrintHeadType() == ROHM_72MM_800_OHM)
    {
        buffSize = PRINTER_BUFFER_SIZE_72MM;
    }
    else
    {
        buffSize = PRINTER_BUFFER_SIZE_80MM;
    }
  
  
    unsigned char *pBuffer = getImageBuffer();
    if( pBuffer != NULL) {
        memset( pBuffer, 0, buffSize );    
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
    //sizingLabels = false;
}

/******************************************************************************/
/*!   \fn void clrContinuousStock( void )
      \brief
        This function clears the flag for continuous stock. This flag signals use 
        when to use tracking for adjusting for die cut labels slip and stalling
        which is not important on continuous stock.

      \author
          Aaron Swift
*******************************************************************************/              
void clrContinuousStock( void )
{
    continuousStock_ = false;
    //sizingLabels = true;
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

bool getLabelPositionedAfterSizing( void )
{
    return labelPositionedAfterSizing;
}

bool getUsingContinuous( void )
{
    if(continuousStock_ == true)
    {
        return true;
    }
    else 
    {
        return false;
    }
}

bool getSizingLabels( void )
{
    if(sizingLabels == true)
    {
        return true;
    }
    else 
    {
        return false;
    }
}

void setSizingLabels( bool sizing )
{
    if(sizing == true)
    {
        sizingLabels = true;
    }
    else 
    {
        sizingLabels = false;
    }
}

int getBackwindOffset( void )
{
    return backwindOffset;
}

bool getIDF2( void )
{
    if(GPIO_PinRead(GPIO1, 8U) == 0)
    {
      return true;
    }
    else
    {
      return false;
    }
}

bool getBackwindAfterSizing( void )
{
    return backwindAfterSizing;
}

void setBackwindAfterSizing(bool backwind)
{
    backwindAfterSizing = backwind;
}

uint32_t getLTWaitCount_( void )
{
    return LTWaitCount_;
}

void setLTWaitCount_( uint32_t waitCount )
{
    LTWaitCount_ = waitCount;
}

bool getExpelDone( void )
{
    return expelDone;
}

int* getLabelLowHistoryBuffer( void)
{
    return labelLowCountsHistory;
}

int getShootIndex( void )
{
    return shootIndex;
}

void setShootIndex( int index )
{
    shootIndex = index;
}

uint16_t calculateSizingBackwindSteps( void )
{
    if(getTakingUpPaper() == false)
    {
        if(getLargeGapFlag() == true)
        {
            if(getLabelSizeInQuarterSteps() <= 760 && getLabelSizeInQuarterSteps() >= 660) //1.75 inch HT labels //TODO
            {
                return (175 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 1012 && getLabelSizeInQuarterSteps() >= 912) //2.37 inch HT labels //TODO
            {
                return (175 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 1268 && getLabelSizeInQuarterSteps() >= 1168) //3 inch HT labels //TODO
            {
                return (175 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 1471 && getLabelSizeInQuarterSteps() >= 1371) //3.5 inch HT labels //TODO
            {
                return (175 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 1674 && getLabelSizeInQuarterSteps() >= 1574) //4 inch HT labels 
            {
                return (175 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 1877 && getLabelSizeInQuarterSteps() >= 1777) //4.5 inch HT labels //TODO
            {
                return (175 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 2080 && getLabelSizeInQuarterSteps() >= 1980) //5 inch HT labels //TODO
            {
                return (175 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 2283 && getLabelSizeInQuarterSteps() >= 2183) //5.5 inch HT labels 
            {
                return (175 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 2486 && getLabelSizeInQuarterSteps() >= 2386) //6 inch HT labels //TODO
            {
                return (175 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 2689 && getLabelSizeInQuarterSteps() >= 2589) //6.5 inch HT labels 
            {
                return (175 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 2892 && getLabelSizeInQuarterSteps() >= 2792) //7 inch HT labels 
            {
                return (175 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 3095 && getLabelSizeInQuarterSteps() >= 2995) //7.5 inch HT labels //TODO
            {
                return (175 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 3298 && getLabelSizeInQuarterSteps() >= 3198) //8 inch HT labels //TODO
            {
                return (175 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 3501 && getLabelSizeInQuarterSteps() >= 3401) //8.5 inch HT labels //TODO
            {
                return (175 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 3704 && getLabelSizeInQuarterSteps() >= 3604) //9 inch HT labels //TODO
            {
                return (175 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 3907 && getLabelSizeInQuarterSteps() >= 3807) //9.5 inch HT labels //TODO
            {
                return (175 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 4110 && getLabelSizeInQuarterSteps() >= 4010) //10 inch HT labels //TODO
            {
                return (175 - config_.verticalPosition);
            }
            else 
            {
                return (175 - config_.verticalPosition);
            }
        }
        else
        {
            if(getLabelSizeInQuarterSteps() <= 740 && getLabelSizeInQuarterSteps() >= 680) 
            {
                //backwindStock(223 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 1.75 inch label no paper\r\n");
                return (223 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 1000 && getLabelSizeInQuarterSteps() >= 900)
            {   
                //backwindStock(190 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 2.37 inch label no paper\r\n");
                return (190 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 1299 && getLabelSizeInQuarterSteps() >= 1100)
            {
                //backwindStock(180 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 3 inch label no paper\r\n");
                return (190 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 1499 && getLabelSizeInQuarterSteps() >= 1300)
            {
                //backwindStock(186 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 3.5 inch label no paper\r\n");
                return (186 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 1699 && getLabelSizeInQuarterSteps() >= 1500)
            {
                //backwindStock(178 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 4 inch label no paper\r\n");
                return (178 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 1900 && getLabelSizeInQuarterSteps() >= 1700)
            {
                //backwindStock(180 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 4.5 inch label no paper\r\n");
                return (180 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 2060 && getLabelSizeInQuarterSteps() >= 1960)
            {
                //backwindStock(174 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 5 inch label no paper\r\n");
                return (174 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 2260 && getLabelSizeInQuarterSteps() >= 2190)
            {
                //backwindStock(176 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 5.5 inch label no paper\r\n");
                return (176 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 2500 && getLabelSizeInQuarterSteps() >= 2380)
            {
                //backwindStock(162 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 6 inch label no paper\r\n");
                return (162 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 2700 && getLabelSizeInQuarterSteps() >= 2600) 
            {
                //backwindStock(162 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 6.5 inch label NO PAPER\r\n");
                return (162 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <=2860 && getLabelSizeInQuarterSteps() >= 2760)
            {
                //backwindStock(153 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 7 inch label no paper\r\n");
                return (153 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 3070 && getLabelSizeInQuarterSteps() >= 2970)
            {
                //backwindStock(152 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 7.5 inch label no paper\r\n");
                return (152 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 3300 && getLabelSizeInQuarterSteps() >= 3160)
            {
                //backwindStock(144 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 8 inch label no paper\r\n");
                return (144 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 3490 && getLabelSizeInQuarterSteps() >= 3400)
            {
                //backwindStock(140 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 8.5 inch label no paper\r\n");
                return (140 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 3690 && getLabelSizeInQuarterSteps() >= 3590) 
            {
                //backwindStock(138 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 9 inch label no paper\r\n");
                return (138 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 3890 && getLabelSizeInQuarterSteps() >= 3700) 
            {
                //backwindStock(134 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 9.5 inch label no paper\r\n");
                return (134 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 5000 && getLabelSizeInQuarterSteps() >= 3960) 
            {
                //backwindStock(144 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 10 inch label no paper\r\n");
                return (144 - config_.verticalPosition);
            }
            else 
            {
                //backwindStock(150, 1000);
                PRINTF("FIRST BACKWIND FOR UNMATCHED LABEL LENGTH\r\n");
                return 150;
            }
        }
    }
    else
    {
        if(getLargeGapFlag() == true)
        {
            if(getLabelSizeInQuarterSteps() <= 760 && getLabelSizeInQuarterSteps() >= 660) //1.75 inch HT labels //TODO
            {
                return (225 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 1012 && getLabelSizeInQuarterSteps() >= 912) //2.37 inch HT labels //TODO
            {
                return (225 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 1268 && getLabelSizeInQuarterSteps() >= 1168) //3 inch HT labels //TODO
            {
                return (225 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 1471 && getLabelSizeInQuarterSteps() >= 1371) //3.5 inch HT labels //TODO
            {
                return (225 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 1674 && getLabelSizeInQuarterSteps() >= 1574) //4 inch HT labels
            {
                return (225 - config_.verticalPosition); 
            }
            else if(getLabelSizeInQuarterSteps() <= 1877 && getLabelSizeInQuarterSteps() >= 1777) //4.5 inch HT labels //TODO
            {
                return (225 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 2080 && getLabelSizeInQuarterSteps() >= 1980) //5 inch HT labels //TODO
            {
                return (225 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 2283 && getLabelSizeInQuarterSteps() >= 2183) //5.5 inch HT labels 
            {
                return (225 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 2486 && getLabelSizeInQuarterSteps() >= 2386)  //6 inch HT labels //TODO
            {
                return (225 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 2689 && getLabelSizeInQuarterSteps() >= 2589) //6.5 inch HT labels
            {
                return (225 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 2892 && getLabelSizeInQuarterSteps() >= 2792) //7 inch HT labels
            {
                return (225 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 3095 && getLabelSizeInQuarterSteps() >= 2995) //7.5 inch HT labels //TODO
            {
                return (225 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 3298 && getLabelSizeInQuarterSteps() >= 3198) //8 inch HT labels //TODO
            {
                return (225 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 3501 && getLabelSizeInQuarterSteps() >= 3401) //8.5 inch HT labels //TODO
            {
                return (225 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 3704 && getLabelSizeInQuarterSteps() >= 3604) //9 inch HT labels //TODO
            {
                return (225 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 3907 && getLabelSizeInQuarterSteps() >= 3807) //9.5 inch HT labels //TODO
            {
                return (225 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 4110 && getLabelSizeInQuarterSteps() >= 4010) //10 inch HT labels //TODO
            {
                return (225 - config_.verticalPosition);
            }
            else 
            {
                return (225 - config_.verticalPosition);
            }  
        }
        else
        {
            if(getLabelSizeInQuarterSteps() <= 740 && getLabelSizeInQuarterSteps() >= 680) 
            {
                //backwindStock(225 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 1.75 inch label\r\n");
                return (225 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 1000 && getLabelSizeInQuarterSteps() >= 900)
            {
                //backwindStock(135 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 2.37 inch label\r\n");
                return (135 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 1250 && getLabelSizeInQuarterSteps() >= 1150)
            {
                //backwindStock(137 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 3 inch label\r\n");
                return (160 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 1499 && getLabelSizeInQuarterSteps() >= 1390)
            {
                //backwindStock(129 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 3.5 inch label\r\n");
                return (129 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 1699 && getLabelSizeInQuarterSteps() >= 1500)
            {
                //backwindStock(128 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 4 inch label\r\n");
                return (128 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 1900 && getLabelSizeInQuarterSteps() >= 1700)
            {
                //backwindStock(122 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 4.5 inch label\r\n");
                return (122 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 2060 && getLabelSizeInQuarterSteps() >= 1990)
            {
                //backwindStock(117 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 5 inch label\r\n");
                return (117 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 2260 && getLabelSizeInQuarterSteps() >= 2190)
            {
                //backwindStock(120 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 5.5 inch label\r\n");
                return (120 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <=2500 && getLabelSizeInQuarterSteps() >= 2380)
            {
                //backwindStock(113 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 6 inch label\r\n");
                return (113 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <=2720 && getLabelSizeInQuarterSteps() >= 2600) 
            {
                //backwindStock(110 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 6.5 inch label\r\n");
                return (110 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <=2860 && getLabelSizeInQuarterSteps() >= 2780)
            {
                //backwindStock(104 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 7 inch label\r\n");
                return (104 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 3070 && getLabelSizeInQuarterSteps() >= 2990)
            {
                //backwindStock(138 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 7.5 inch label\r\n");
                return (138 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <=3300 && getLabelSizeInQuarterSteps() >= 3200)
            {
                //backwindStock(138 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 8 inch label\r\n");
                return (138 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 3490 && getLabelSizeInQuarterSteps() >= 3400)
            {
                //backwindStock(138 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 8.5 inch label\r\n");
                return (138 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 3690 && getLabelSizeInQuarterSteps() >= 3590) 
            {
                //backwindStock(134 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 9 inch label\r\n");
                return (134 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 3890 && getLabelSizeInQuarterSteps() >= 3700) 
            {
                //backwindStock(128 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 9.5 inch label\r\n");
                return (128 - config_.verticalPosition);
            }
            else if(getLabelSizeInQuarterSteps() <= 5000 && getLabelSizeInQuarterSteps() >= 3960) 
            {
                //backwindStock(136 - config_.verticalPosition, 1300);
                //PRINTF("FIRST BACKWIND FOR 10 inch label\r\n");
                return (136 - config_.verticalPosition);
            }
            else 
            {
                //backwindStock(150, 1300);
                PRINTF("FIRST BACKWIND FOR UNMATCHED LABEL LENGTH\r\n");
                return 150;
            }
        }
    }  
}

uint16_t calculateStreamingBackwindSteps( void )
{
    if(getLargeGapFlag() == true)
    {
        if(getLabelSizeInQuarterSteps() <= 760 && getLabelSizeInQuarterSteps() >= 660) //1.75 inch HT labels //TODO
        {
            if(TPHStepsPastGapThisPrint >= 390)
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 390) - (config_.verticalPosition - 34);
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 390) + (34 - config_.verticalPosition); 
                    return streamingLabelBackwind;
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (390 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34); 
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (390 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                    return streamingLabelBackwind;
                }
            }
        }
        else if(getLabelSizeInQuarterSteps() <= 1012 && getLabelSizeInQuarterSteps() >= 912) //2.37 inch HT labels //TODO
        {
            if(TPHStepsPastGapThisPrint >= 390)
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 390) - (config_.verticalPosition - 34);
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 390) + (34 - config_.verticalPosition); 
                    return streamingLabelBackwind;
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (390 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34); 
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (390 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                    return streamingLabelBackwind;
                }
            }
        }
        else if(getLabelSizeInQuarterSteps() <= 1268 && getLabelSizeInQuarterSteps() >= 1168) //3 inch HT labels //TODO
        {
            if(TPHStepsPastGapThisPrint >= 390)
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 390) - (config_.verticalPosition - 34);
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 390) + (34 - config_.verticalPosition); 
                    return streamingLabelBackwind;
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (390 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34); 
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (390 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                    return streamingLabelBackwind;
                }
            }
        }
        else if(getLabelSizeInQuarterSteps() <= 1471 && getLabelSizeInQuarterSteps() >= 1371) //3.5 inch HT labels //TODO
        {
            if(TPHStepsPastGapThisPrint >= 390)
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 390) - (config_.verticalPosition - 34);
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 390) + (34 - config_.verticalPosition); 
                    return streamingLabelBackwind;
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (390 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34); 
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (390 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                    return streamingLabelBackwind;
                }
            }
        }
        else if(getLabelSizeInQuarterSteps() <= 1674 && getLabelSizeInQuarterSteps() >= 1574) //4 inch HT labels 
        {
            if(TPHStepsPastGapThisPrint >= 326)
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 326) - (config_.verticalPosition - 34);
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 326) + (34 - config_.verticalPosition); 
                    return streamingLabelBackwind;
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (326 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34); 
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (326 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                    return streamingLabelBackwind;
                }
            } 
        }
        else if(getLabelSizeInQuarterSteps() <= 1877 && getLabelSizeInQuarterSteps() >= 1777) //4.5 inch HT labels //TODO
        {
            if(TPHStepsPastGapThisPrint >= 390)
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 390) - (config_.verticalPosition - 34);
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 390) + (34 - config_.verticalPosition); 
                    return streamingLabelBackwind;
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (390 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34); 
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (390 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                    return streamingLabelBackwind;
                }
            }
        }
        else if(getLabelSizeInQuarterSteps() <= 2080 && getLabelSizeInQuarterSteps() >= 1980) //5 inch HT labels //TODO
        {
            if(TPHStepsPastGapThisPrint >= 390)
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 390) - (config_.verticalPosition - 34);
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 390) + (34 - config_.verticalPosition); 
                    return streamingLabelBackwind;
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (390 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34); 
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (390 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                    return streamingLabelBackwind;
                }
            }
        }
        else if(getLabelSizeInQuarterSteps() <= 2283 && getLabelSizeInQuarterSteps() >= 2183) //5.5 inch HT labels
        {
            if(TPHStepsPastGapThisPrint >= 336)
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 336) - (config_.verticalPosition - 34);
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 336) + (34 - config_.verticalPosition); 
                    return streamingLabelBackwind;
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (336 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34); 
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (336 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                    return streamingLabelBackwind;
                }
            }
        }
        else if(getLabelSizeInQuarterSteps() <= 2486 && getLabelSizeInQuarterSteps() >= 2386) //6 inch HT labels //TODO
        {
            if(TPHStepsPastGapThisPrint >= 390)
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 390) - (config_.verticalPosition - 34);
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 390) + (34 - config_.verticalPosition); 
                    return streamingLabelBackwind;
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (390 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34); 
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (390 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                    return streamingLabelBackwind;
                }
            }
        }
        else if(getLabelSizeInQuarterSteps() <= 2689 && getLabelSizeInQuarterSteps() >= 2589) // 6.5 inch HT labels
        {                     
            if(TPHStepsPastGapThisPrint >= 390)
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 390) - (config_.verticalPosition - 34);
                    return streamingLabelBackwind;
                }
                else
                { 
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 390) + (34 - config_.verticalPosition); 
                    return streamingLabelBackwind;
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (390 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34); 
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (390 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                    return streamingLabelBackwind;
                }
            }
        }
        else if(getLabelSizeInQuarterSteps() <= 2892 && getLabelSizeInQuarterSteps() >= 2792) //7 inch HT labels 
        {
            if(TPHStepsPastGapThisPrint >= 328)
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 328) - (config_.verticalPosition - 34);
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 328) + (34 - config_.verticalPosition); 
                    return streamingLabelBackwind;
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (328 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34); 
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (328 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                    return streamingLabelBackwind;
                }
            }
        }
        else if(getLabelSizeInQuarterSteps() <= 3095 && getLabelSizeInQuarterSteps() >= 2995) //7.5 inch HT labels //TODO
        {
            if(TPHStepsPastGapThisPrint >= 390)
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 390) - (config_.verticalPosition - 34);
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 390) + (34 - config_.verticalPosition); 
                    return streamingLabelBackwind;
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (390 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34); 
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (390 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                    return streamingLabelBackwind;
                }
            }
        }
        else if(getLabelSizeInQuarterSteps() <= 3298 && getLabelSizeInQuarterSteps() >= 3198) //8 inch HT labels //TODO
        {
            if(TPHStepsPastGapThisPrint >= 390)
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 390) - (config_.verticalPosition - 34);
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 390) + (34 - config_.verticalPosition); 
                    return streamingLabelBackwind;
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (390 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34); 
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (390 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                    return streamingLabelBackwind;
                }
            }
        }
        else if(getLabelSizeInQuarterSteps() <= 3501 && getLabelSizeInQuarterSteps() >= 3401) //8.5 inch HT labels //TODO
        {
            if(TPHStepsPastGapThisPrint >= 390)
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 390) - (config_.verticalPosition - 34);
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 390) + (34 - config_.verticalPosition); 
                    return streamingLabelBackwind;
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (390 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34); 
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (390 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                    return streamingLabelBackwind;
                }
            }
        }
        else if(getLabelSizeInQuarterSteps() <= 3704 && getLabelSizeInQuarterSteps() >= 3604) //9 inch HT labels //TODO
        {
            if(TPHStepsPastGapThisPrint >= 390)
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 390) - (config_.verticalPosition - 34);
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 390) + (34 - config_.verticalPosition); 
                    return streamingLabelBackwind;
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (390 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34); 
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (390 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                    return streamingLabelBackwind;
                }
            }
        }
        else if(getLabelSizeInQuarterSteps() <= 3907 && getLabelSizeInQuarterSteps() >= 3807) //9.5 inch HT labels //TODO
        {
            if(TPHStepsPastGapThisPrint >= 390)
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 390) - (config_.verticalPosition - 34);
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 390) + (34 - config_.verticalPosition); 
                    return streamingLabelBackwind;
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (390 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34); 
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (390 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                    return streamingLabelBackwind;
                }
            }
        }
        else if(getLabelSizeInQuarterSteps() <= 4110 && getLabelSizeInQuarterSteps() >= 4010) //10 inch HT labels //TODO
        {
            if(TPHStepsPastGapThisPrint >= 390)
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 390) - (config_.verticalPosition - 34);
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 390) + (34 - config_.verticalPosition); 
                    return streamingLabelBackwind;
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (390 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34); 
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (390 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                    return streamingLabelBackwind;
                }
            }
        }
        else //unmatched HT label length
        {
           if(TPHStepsPastGapThisPrint >= 390)
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 390) - (config_.verticalPosition - 34);
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (TPHStepsPastGapThisPrint - 390) + (34 - config_.verticalPosition);
                    return streamingLabelBackwind;
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    streamingLabelBackwind = (390 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                    return streamingLabelBackwind;
                }
                else
                {
                    streamingLabelBackwind = (390 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                    return streamingLabelBackwind;
                }
            }
           
            PRINTF("WALMART UNMATCHED label BACKWIND either PE mode or not taking up paper\r\n");
        }
    }
    else
    {
        if(getLabelSizeInQuarterSteps() <= 780 && getLabelSizeInQuarterSteps() >= 680) 
        {
            if(config_.verticalPosition > 34)
            {
              //backwindStock((TPHStepsPastGapThisPrint - 349) - (config_.verticalPosition - 34), 1300);
              streamingLabelBackwind = (TPHStepsPastGapThisPrint - 319) - (config_.verticalPosition - 34);
              return streamingLabelBackwind;
            }
            else if(config_.verticalPosition <= 34)
            {
              //backwindStock((TPHStepsPastGapThisPrint - 349) + (34 - config_.verticalPosition), 1300);
              streamingLabelBackwind = (TPHStepsPastGapThisPrint - 319) + (34 - config_.verticalPosition);
              return streamingLabelBackwind;
            }

            //PRINTF("1.75 inch label BACKWIND either PE mode or not taking up paper\r\n");
        }
        else if(getLabelSizeInQuarterSteps() <= 1000 && getLabelSizeInQuarterSteps() >= 900) 
        {
            if(config_.verticalPosition > 34)
            {
              //backwindStock((TPHStepsPastGapThisPrint - 349) - (config_.verticalPosition - 34), 1300);
              streamingLabelBackwind = (TPHStepsPastGapThisPrint - 349) - (config_.verticalPosition - 34);
              return streamingLabelBackwind;
            }
            else if(config_.verticalPosition <= 34)
            {
              //backwindStock((TPHStepsPastGapThisPrint - 349) + (34 - config_.verticalPosition), 1300);
              streamingLabelBackwind = (TPHStepsPastGapThisPrint - 349) + (34 - config_.verticalPosition);
              return streamingLabelBackwind;
            }

            //PRINTF("2.37 inch label BACKWIND either PE mode or not taking up paper\r\n");
        }
        else if(getLabelSizeInQuarterSteps() <= 1299 && getLabelSizeInQuarterSteps() >= 1100) 
        {    
            if(config_.verticalPosition > 34)
            {
              //backwindStock((TPHStepsPastGapThisPrint - 349) - (config_.verticalPosition - 34), 1300);
              streamingLabelBackwind = (TPHStepsPastGapThisPrint - 349) - (config_.verticalPosition - 34);
              return streamingLabelBackwind;
            }
            else if(config_.verticalPosition <= 34)
            {
              //backwindStock((TPHStepsPastGapThisPrint - 349) + (34 - config_.verticalPosition), 1300);
              streamingLabelBackwind = (TPHStepsPastGapThisPrint - 349) + (34 - config_.verticalPosition);
              return streamingLabelBackwind;
            }
            
            //PRINTF("3 inch label BACKWIND either PE mode or not taking up paper\r\n");
        }
        else if(getLabelSizeInQuarterSteps() <= 1499 && getLabelSizeInQuarterSteps() >= 1300)
        {
            if(config_.verticalPosition > 34)
            {
              //backwindStock((TPHStepsPastGapThisPrint - 340) - (config_.verticalPosition - 34), 1300);
              streamingLabelBackwind = (TPHStepsPastGapThisPrint - 340) - (config_.verticalPosition - 34);
              return streamingLabelBackwind;
            }
            else if(config_.verticalPosition <= 34)
            {
              //backwindStock((TPHStepsPastGapThisPrint - 340) + (34 - config_.verticalPosition), 1300);
              streamingLabelBackwind = (TPHStepsPastGapThisPrint - 340) + (34 - config_.verticalPosition);
              return streamingLabelBackwind;
            }

            //PRINTF("3.5 inch label BACKWIND either PE mode or not taking up paper\r\n");
        }
        else if(getLabelSizeInQuarterSteps() <= 1699 && getLabelSizeInQuarterSteps() >= 1500)
        {
            if(config_.verticalPosition > 34)
            {
              //backwindStock((TPHStepsPastGapThisPrint - 352) - (config_.verticalPosition - 34), 1300);
              streamingLabelBackwind = (TPHStepsPastGapThisPrint - 352) - (config_.verticalPosition - 34);
              return streamingLabelBackwind;
            }
            else if(config_.verticalPosition <= 34)
            {
              //backwindStock((TPHStepsPastGapThisPrint - 352) + (34 - config_.verticalPosition), 1300);
              streamingLabelBackwind = (TPHStepsPastGapThisPrint - 352) + (34 - config_.verticalPosition);
              return streamingLabelBackwind;
            }

            //PRINTF("4 inch label BACKWIND either PE mode or not taking up paper\r\n");
        }
        else if(getLabelSizeInQuarterSteps() <= 1900 && getLabelSizeInQuarterSteps() >= 1700) 
        {
            if(config_.verticalPosition > 34)
            {
              //backwindStock((TPHStepsPastGapThisPrint - 349) - (config_.verticalPosition - 34), 1300);
              streamingLabelBackwind = (TPHStepsPastGapThisPrint - 349) - (config_.verticalPosition - 34);
              return streamingLabelBackwind;
            }
            else if(config_.verticalPosition <= 34)
            {
              //backwindStock((TPHStepsPastGapThisPrint - 349) + (34 - config_.verticalPosition), 1300);
              streamingLabelBackwind = (TPHStepsPastGapThisPrint - 349) + (34 - config_.verticalPosition);
              return streamingLabelBackwind;
            }
            
            //PRINTF("4.5 inch label BACKWIND either PE mode or not taking up paper\r\n");
        }
        else if(getLabelSizeInQuarterSteps() <= 2060 && getLabelSizeInQuarterSteps() >= 1960)
        {
            if(config_.verticalPosition > 34)
            {
              //backwindStock((TPHStepsPastGapThisPrint - 350) - (config_.verticalPosition - 34), 1300);
              streamingLabelBackwind = (TPHStepsPastGapThisPrint - 350) - (config_.verticalPosition - 34);
              return streamingLabelBackwind;
            }
            else if(config_.verticalPosition <= 34)
            {
              //backwindStock((TPHStepsPastGapThisPrint - 350) + (34 - config_.verticalPosition), 1300);
              streamingLabelBackwind = (TPHStepsPastGapThisPrint - 350) + (34 - config_.verticalPosition);
              return streamingLabelBackwind;
            }

            //PRINTF("5 inch label BACKWIND either PE mode or not taking up paper\r\n");
        }
        else if(getLabelSizeInQuarterSteps() <= 2260 && getLabelSizeInQuarterSteps() >= 2160)
        {
            if(config_.verticalPosition > 34)
            {
              //backwindStock((TPHStepsPastGapThisPrint - 350) - (config_.verticalPosition - 34), 1300);
              streamingLabelBackwind = (TPHStepsPastGapThisPrint - 350) - (config_.verticalPosition - 34);
              return streamingLabelBackwind;
            }
            else if(config_.verticalPosition <= 34)
            {
              //backwindStock((TPHStepsPastGapThisPrint - 350) + (34 - config_.verticalPosition), 1300);
              streamingLabelBackwind = (TPHStepsPastGapThisPrint - 350) + (34 - config_.verticalPosition);
              return streamingLabelBackwind;
            }

            //PRINTF("5.5 inch label BACKWIND either PE mode or not taking up paper\r\n");
        }
        else if(getLabelSizeInQuarterSteps() <= 2500 && getLabelSizeInQuarterSteps() >= 2380) 
        {
            if(config_.verticalPosition > 34)
            {
              //backwindStock((TPHStepsPastGapThisPrint - 366) - (config_.verticalPosition - 34), 1300);
              streamingLabelBackwind = (TPHStepsPastGapThisPrint - 366) - (config_.verticalPosition - 34);
              return streamingLabelBackwind;
            }
            else if(config_.verticalPosition <= 34)
            {
              //backwindStock((TPHStepsPastGapThisPrint - 366) + (34 - config_.verticalPosition), 1300);
              streamingLabelBackwind = (TPHStepsPastGapThisPrint - 366) + (34 - config_.verticalPosition);
              return streamingLabelBackwind;
            }

            //PRINTF("6 inch label BACKWIND either PE mode or not taking up paper\r\n");
        }
        else if(getLabelSizeInQuarterSteps() <= 2680 && getLabelSizeInQuarterSteps() >= 2600) 
        {
            if(config_.verticalPosition > 34)
            {
                //backwindStock((TPHStepsPastGapThisPrint - 359) - (config_.verticalPosition - 34), 1300);
                streamingLabelBackwind = (TPHStepsPastGapThisPrint - 359) - (config_.verticalPosition - 34);
                return streamingLabelBackwind;
            }
            else if(config_.verticalPosition <= 34)
            {
                //backwindStock((TPHStepsPastGapThisPrint - 359) + (34 - config_.verticalPosition), 1300);
                streamingLabelBackwind = (TPHStepsPastGapThisPrint - 359) + (34 - config_.verticalPosition);
                return streamingLabelBackwind;
            }

            //PRINTF("6.5 inch label BACKWIND either PE mode or not taking up paper\r\n");
        }
        else if(getLabelSizeInQuarterSteps() <=2860 && getLabelSizeInQuarterSteps() >= 2780)
        {
            if(config_.verticalPosition > 34)
            {
                //backwindStock((TPHStepsPastGapThisPrint - 377) - (config_.verticalPosition - 34), 1300);
                streamingLabelBackwind = (TPHStepsPastGapThisPrint - 377) - (config_.verticalPosition - 34);
                return streamingLabelBackwind;
            }
            else if(config_.verticalPosition <= 34)
            {
                //backwindStock((TPHStepsPastGapThisPrint - 377) + (34 - config_.verticalPosition), 1300);
                streamingLabelBackwind = (TPHStepsPastGapThisPrint - 377) + (34 - config_.verticalPosition);
                return streamingLabelBackwind;
            }

            //PRINTF("7 inch label BACKWIND either PE mode or not taking up paper\r\n");
        }
        else if(getLabelSizeInQuarterSteps() <= 3070 && getLabelSizeInQuarterSteps() >= 2970)
        {
            if(config_.verticalPosition > 34)
            {
                //backwindStock((TPHStepsPastGapThisPrint - 377) - (config_.verticalPosition - 34), 1300);
                streamingLabelBackwind = (TPHStepsPastGapThisPrint - 377) - (config_.verticalPosition - 34);
                return streamingLabelBackwind;
            }
            else if(config_.verticalPosition <= 34)
            {
                //backwindStock((TPHStepsPastGapThisPrint - 377) + (34 - config_.verticalPosition), 1300);
                streamingLabelBackwind = (TPHStepsPastGapThisPrint - 377) + (34 - config_.verticalPosition);
                return streamingLabelBackwind;
            }

            //PRINTF("7.5 inch label BACKWIND either PE mode or not taking up paper\r\n");
        }
        else if(getLabelSizeInQuarterSteps() <= 3300 && getLabelSizeInQuarterSteps() >= 3200) 
        {  
            if(config_.verticalPosition > 34)
            {
                //backwindStock((TPHStepsPastGapThisPrint - 377) - (config_.verticalPosition - 34), 1300);
                streamingLabelBackwind = (TPHStepsPastGapThisPrint - 377) - (config_.verticalPosition - 34);
                return streamingLabelBackwind;
            }
            else if(config_.verticalPosition <= 34)
            {
                //backwindStock((TPHStepsPastGapThisPrint - 377) + (34 - config_.verticalPosition), 1300);
                streamingLabelBackwind = (TPHStepsPastGapThisPrint - 377) + (34 - config_.verticalPosition);
                return streamingLabelBackwind;
            }

            //PRINTF("8 inch label BACKWIND either PE mode or not taking up paper\r\n");
        }
        else if(getLabelSizeInQuarterSteps() <= 3490 && getLabelSizeInQuarterSteps() >= 3400)
        {
            if(config_.verticalPosition > 34)
            {
                //backwindStock((TPHStepsPastGapThisPrint - 382) - (config_.verticalPosition - 34), 1300);
                streamingLabelBackwind = (TPHStepsPastGapThisPrint - 382) - (config_.verticalPosition - 34);
                return streamingLabelBackwind;
            }
            else if(config_.verticalPosition <= 34)
            {
                //backwindStock((TPHStepsPastGapThisPrint - 382) + (34 - config_.verticalPosition), 1300);
                streamingLabelBackwind = (TPHStepsPastGapThisPrint - 382) + (34 - config_.verticalPosition);
                return streamingLabelBackwind;
            }

            //PRINTF("8.5 inch label BACKWIND either PE mode or not taking up paper\r\n");
        }
        else if(getLabelSizeInQuarterSteps() <= 3690 && getLabelSizeInQuarterSteps() >= 3590) 
        {
            if(config_.verticalPosition > 34)
            {
                //backwindStock((TPHStepsPastGapThisPrint - 387) - (config_.verticalPosition - 34), 1300);
                streamingLabelBackwind = (TPHStepsPastGapThisPrint - 387) - (config_.verticalPosition - 34);
                return streamingLabelBackwind;
            }
            else if(config_.verticalPosition <= 34)
            {
                //backwindStock((TPHStepsPastGapThisPrint - 387) + (34 - config_.verticalPosition), 1300);
                streamingLabelBackwind = (TPHStepsPastGapThisPrint - 387) + (34 - config_.verticalPosition); 
                return streamingLabelBackwind;
            }

            //PRINTF("9 inch label BACKWIND either PE mode or not taking up paper\r\n");
        }
        else if(getLabelSizeInQuarterSteps() <= 3890 && getLabelSizeInQuarterSteps() >= 3700) 
        {
            if(config_.verticalPosition > 34)
            {
                //backwindStock((TPHStepsPastGapThisPrint - 382) - (config_.verticalPosition - 34), 1300);
                streamingLabelBackwind = (TPHStepsPastGapThisPrint - 382) - (config_.verticalPosition - 34);
                return streamingLabelBackwind;
            }
            else if(config_.verticalPosition <= 34)
            {
                //backwindStock((TPHStepsPastGapThisPrint - 382) + (34 - config_.verticalPosition), 1300);
                streamingLabelBackwind = (TPHStepsPastGapThisPrint - 382) + (34 - config_.verticalPosition);
                return streamingLabelBackwind;
            }

            //PRINTF("9.5 inch label BACKWIND either PE mode or not taking up paper\r\n");
        }
        else if(getLabelSizeInQuarterSteps() <= 5000 && getLabelSizeInQuarterSteps() >= 3960)
        {
            if(config_.verticalPosition > 34)
            {
                //backwindStock((TPHStepsPastGapThisPrint - 382) - (config_.verticalPosition - 34), 1300);
                streamingLabelBackwind = (TPHStepsPastGapThisPrint - 382) - (config_.verticalPosition - 34);
                return streamingLabelBackwind;
            }
            else if(config_.verticalPosition <= 34)
            {
                //backwindStock((TPHStepsPastGapThisPrint - 382) + (34 - config_.verticalPosition), 1300);
                streamingLabelBackwind = (TPHStepsPastGapThisPrint - 382) + (34 - config_.verticalPosition);
                return streamingLabelBackwind;
            }

            //PRINTF("10 inch label BACKWIND either PE mode or not taking up paper\r\n");
        }
        else
        {
            //backwindStock(120 + backwindOffset + backwindOffsetCount /*- offsetThisPrint*/, 1300);
            streamingLabelBackwind = 120 + backwindOffset + backwindOffsetCount;
            PRINTF("UNMATCHED label BACKWIND either PE mode or not taking up paper\r\n");
            return streamingLabelBackwind;
        }
    }
}

uint16_t calculatePeelingBackwindSteps( void )
{          
    if(getLargeGapFlag() == true)
    {
        if(getLabelSizeInQuarterSteps() <= 760 && getLabelSizeInQuarterSteps() >= 660) //1.75 inch HT labels //TODO
        {
            if(TPHStepsPastGapThisPrint >= 282) 
            {
                if(config_.verticalPosition >= 34)
                {
                    return (TPHStepsPastGapThisPrint - 282) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (TPHStepsPastGapThisPrint - 282) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    return (282 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (282 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }
        }
        else if(getLabelSizeInQuarterSteps() <= 1012 && getLabelSizeInQuarterSteps() >= 912) //2.37 inch HT labels //TODO
        {
            if(TPHStepsPastGapThisPrint >= 282) 
            {
                if(config_.verticalPosition >= 34)
                {
                    return (TPHStepsPastGapThisPrint - 282) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (TPHStepsPastGapThisPrint - 282) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    return (282 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (282 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }
        }
        else if(getLabelSizeInQuarterSteps() <= 1268 && getLabelSizeInQuarterSteps() >= 1168) //3 inch HT labels //TODO
        {
            if(TPHStepsPastGapThisPrint >= 282) 
            {
                if(config_.verticalPosition >= 34)
                {
                    return (TPHStepsPastGapThisPrint - 282) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (TPHStepsPastGapThisPrint - 282) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    return (282 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (282 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }
        }
        else if(getLabelSizeInQuarterSteps() <= 1471 && getLabelSizeInQuarterSteps() >= 1371) //3.5 inch HT labels //TODO
        {
            if(TPHStepsPastGapThisPrint >= 282) 
            {
                if(config_.verticalPosition >= 34)
                {
                    return (TPHStepsPastGapThisPrint - 282) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (TPHStepsPastGapThisPrint - 282) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    return (282 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (282 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }
        }
        else if(getLabelSizeInQuarterSteps() <= 1674 && getLabelSizeInQuarterSteps() >= 1574) //4 inch HT labels
        {
            if(TPHStepsPastGapThisPrint >= 238) 
            {
                if(config_.verticalPosition >= 34)
                {
                    return (TPHStepsPastGapThisPrint - 238) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (TPHStepsPastGapThisPrint - 238) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    return (238 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (238 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }
        }
        else if(getLabelSizeInQuarterSteps() <= 1877 && getLabelSizeInQuarterSteps() >= 1777) //4.5 inch HT labels //TODO
        {
            if(TPHStepsPastGapThisPrint >= 282) 
            {
                if(config_.verticalPosition >= 34)
                {
                    return (TPHStepsPastGapThisPrint - 282) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (TPHStepsPastGapThisPrint - 282) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    return (282 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (282 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }
        }
        else if(getLabelSizeInQuarterSteps() <= 2080 && getLabelSizeInQuarterSteps() >= 1980) //5 inch HT labels //TODO
        {
            if(TPHStepsPastGapThisPrint >= 282) 
            {
                if(config_.verticalPosition >= 34)
                {
                    return (TPHStepsPastGapThisPrint - 282) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (TPHStepsPastGapThisPrint - 282) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    return (282 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (282 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }
        }
        else if(getLabelSizeInQuarterSteps() <= 2283 && getLabelSizeInQuarterSteps() >= 2183) //5.5 inch HT labels
        {
            if(TPHStepsPastGapThisPrint >= 242)
            {
                if(config_.verticalPosition >= 34)
                { 
                    return (TPHStepsPastGapThisPrint - 242) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (TPHStepsPastGapThisPrint - 242) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    return (242 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (242 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }
        }
        else if(getLabelSizeInQuarterSteps() <= 2486 && getLabelSizeInQuarterSteps() >= 2386) //6 inch HT labels //TODO
        {
            if(TPHStepsPastGapThisPrint >= 282) 
            {
                if(config_.verticalPosition >= 34)
                {
                    return (TPHStepsPastGapThisPrint - 282) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (TPHStepsPastGapThisPrint - 282) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    return (282 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (282 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }
        }
        else if(getLabelSizeInQuarterSteps() <= 2689 && getLabelSizeInQuarterSteps() >= 2589) //6.5 inch HT labels 
        {         
            if(TPHStepsPastGapThisPrint >= 289)
            {
                if(config_.verticalPosition >= 34)
                {
                    return (TPHStepsPastGapThisPrint - 289) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (TPHStepsPastGapThisPrint - 289) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    return (289 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (289 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }
        }
        else if(getLabelSizeInQuarterSteps() <= 2892 && getLabelSizeInQuarterSteps() >= 2792) //7 inch HT labels
        {
            if(TPHStepsPastGapThisPrint >= 230)
            {
                if(config_.verticalPosition >= 34)
                { 
                    return (TPHStepsPastGapThisPrint - 230) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (TPHStepsPastGapThisPrint - 230) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                { 
                    return (230 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (230 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }
        }
        else if(getLabelSizeInQuarterSteps() <= 3095 && getLabelSizeInQuarterSteps() >= 2995) //7.5 inch HT labels //TODO
        {
            if(TPHStepsPastGapThisPrint >= 282) 
            {
                if(config_.verticalPosition >= 34)
                {
                    return (TPHStepsPastGapThisPrint - 282) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (TPHStepsPastGapThisPrint - 282) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    return (282 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (282 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }
        }
        else if(getLabelSizeInQuarterSteps() <= 3298 && getLabelSizeInQuarterSteps() >= 3198) //8 inch HT labels //TODO
        {
            if(TPHStepsPastGapThisPrint >= 282) 
            {
                if(config_.verticalPosition >= 34)
                {
                    return (TPHStepsPastGapThisPrint - 282) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (TPHStepsPastGapThisPrint - 282) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    return (282 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (282 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }
        }
        else if(getLabelSizeInQuarterSteps() <= 3501 && getLabelSizeInQuarterSteps() >= 3401) //8.5 inch HT labels //TODO
        {
            if(TPHStepsPastGapThisPrint >= 282) 
            {
                if(config_.verticalPosition >= 34)
                {
                    return (TPHStepsPastGapThisPrint - 282) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (TPHStepsPastGapThisPrint - 282) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    return (282 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (282 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }
        }
        else if(getLabelSizeInQuarterSteps() <= 3704 && getLabelSizeInQuarterSteps() >= 3604) //9 inch HT labels //TODO
        {
            if(TPHStepsPastGapThisPrint >= 282) 
            {
                if(config_.verticalPosition >= 34)
                {
                    return (TPHStepsPastGapThisPrint - 282) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (TPHStepsPastGapThisPrint - 282) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    return (282 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (282 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }
        }
        else if(getLabelSizeInQuarterSteps() <= 3907 && getLabelSizeInQuarterSteps() >= 3807) //9.5 inch HT labels //TODO
        {
            if(TPHStepsPastGapThisPrint >= 282) 
            {
                if(config_.verticalPosition >= 34)
                {
                    return (TPHStepsPastGapThisPrint - 282) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (TPHStepsPastGapThisPrint - 282) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    return (282 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (282 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }
        }
        else if(getLabelSizeInQuarterSteps() <= 4110 && getLabelSizeInQuarterSteps() >= 4010) //10 inch HT labels //TODO
        {
            if(TPHStepsPastGapThisPrint >= 282) 
            {
                if(config_.verticalPosition >= 34)
                {
                    return (TPHStepsPastGapThisPrint - 282) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (TPHStepsPastGapThisPrint - 282) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    return (282 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (282 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }
        }
        else //unmatched HT label length
        {
            if(TPHStepsPastGapThisPrint >= 289)
            {
                if(config_.verticalPosition >= 34)
                {
                    return (TPHStepsPastGapThisPrint - 289) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (TPHStepsPastGapThisPrint - 289) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    return (289 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    return (289 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }
        }
    }
    else
    {
        if(getLabelSizeInQuarterSteps() <= 780 && getLabelSizeInQuarterSteps() >= 680) 
        {
            if(TPHStepsPastGapThisPrint >= 258)
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 326) - (config_.verticalPosition - 34) , 1300); 
                    return (TPHStepsPastGapThisPrint - 258) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 326) + (34 - config_.verticalPosition) , 1300);
                    return (TPHStepsPastGapThisPrint - 258) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((326 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34), 1300); 
                    return (258 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((326 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition), 1300);
                    return (258 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }

            //PRINTF("1.75 inch label BACKWIND\r\n");
        }
        else if(getLabelSizeInQuarterSteps() <= 1000 && getLabelSizeInQuarterSteps() >= 900) 
        {
            if(TPHStepsPastGapThisPrint >= 336)
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 326) - (config_.verticalPosition - 34) , 1300); 
                    return (TPHStepsPastGapThisPrint - 336) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 326) + (34 - config_.verticalPosition) , 1300); 
                    return (TPHStepsPastGapThisPrint - 336) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((326 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34), 1300); 
                    return (336 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((326 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition), 1300); 
                    return (336 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }

            //PRINTF("2.37 inch label BACKWIND\r\n");
        }
        else if(getLabelSizeInQuarterSteps() <= 1299 && getLabelSizeInQuarterSteps() >= 1100) 
        {
            if(TPHStepsPastGapThisPrint >= 326)
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 326) - (config_.verticalPosition - 34) , 1300); 
                    return (TPHStepsPastGapThisPrint - 326) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 326) + (34 - config_.verticalPosition) , 1300); 
                    return (TPHStepsPastGapThisPrint - 326) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((326 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34), 1300); 
                    return (326 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((326 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition), 1300); 
                    return (326 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }

            //PRINTF("3 inch label BACKWIND\r\n");
        }
        else if(getLabelSizeInQuarterSteps() <= 1499 && getLabelSizeInQuarterSteps() >= 1300) 
        {
            if(TPHStepsPastGapThisPrint >= 332)
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 332) - (config_.verticalPosition - 34) , 1300);
                    return (TPHStepsPastGapThisPrint - 332) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 332) + (34 - config_.verticalPosition) , 1300); 
                    return (TPHStepsPastGapThisPrint - 332) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((332 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34), 1300); 
                    return (332 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((332 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition), 1300);
                    return (332 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }

            //PRINTF("3.5 inch label BACKWIND\r\n");
        }
        else if(getLabelSizeInQuarterSteps() <= 1699 && getLabelSizeInQuarterSteps() >= 1500)
        {
            if(TPHStepsPastGapThisPrint >= 338)
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 338) - (config_.verticalPosition - 34) , 1300); 
                    return (TPHStepsPastGapThisPrint - 338) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 338) + (34 - config_.verticalPosition) , 1300); 
                    return (TPHStepsPastGapThisPrint - 338) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((338 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34), 1300); 
                    return (338 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((338 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition), 1300); 
                    return (338 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }
            
            //PRINTF("4 inch label BACKWIND\r\n");
        }
        else if(getLabelSizeInQuarterSteps() <= 1900 && getLabelSizeInQuarterSteps() >= 1700) 
        {
            if(TPHStepsPastGapThisPrint >= 340)
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 340) - (config_.verticalPosition - 34) , 1300); 
                    return (TPHStepsPastGapThisPrint - 340) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 340) + (34 - config_.verticalPosition) , 1300); 
                    return (TPHStepsPastGapThisPrint - 340) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((340 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34), 1300); 
                    return (340 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((340 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition), 1300); 
                    return (340 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }

            //PRINTF("4.5 inch label BACKWIND\r\n");
        }
        else if(getLabelSizeInQuarterSteps() <= 2060 && getLabelSizeInQuarterSteps() >= 1960)
        {
            if(TPHStepsPastGapThisPrint >= 346)
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 346) - (config_.verticalPosition - 34) , 1300); 
                    return (TPHStepsPastGapThisPrint - 346) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 346) + (34 - config_.verticalPosition) , 1300);
                    return (TPHStepsPastGapThisPrint - 346) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((346 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34), 1300);
                    return (346 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((346 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition), 1300);
                    return (346 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }

            //PRINTF("5 inch label BACKWIND\r\n");
        }
        else if(getLabelSizeInQuarterSteps() <= 2260 && getLabelSizeInQuarterSteps() >= 2160)
        {
            if(TPHStepsPastGapThisPrint >= 346)
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 346) - (config_.verticalPosition - 34) , 1300); 
                    return (TPHStepsPastGapThisPrint - 346) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 346) + (34 - config_.verticalPosition) , 1300);
                    return (TPHStepsPastGapThisPrint - 346) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((346 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34), 1300); 
                    return (346 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((346 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition), 1300); 
                    return (346 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }

            //PRINTF("5.5 inch label BACKWIND\r\n");
        }
        else if(getLabelSizeInQuarterSteps() <=2500 && getLabelSizeInQuarterSteps() >= 2380) 
        {
            if(TPHStepsPastGapThisPrint >= 355)
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 355) - (config_.verticalPosition - 34) , 1300); 
                    return (TPHStepsPastGapThisPrint - 355) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 355) + (34 - config_.verticalPosition) , 1300);
                    return (TPHStepsPastGapThisPrint - 355) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((355 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34), 1300);
                    return (355 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((355 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition), 1300); 
                    return (355 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }

            //PRINTF("6 inch label BACKWIND\r\n");
        }
        else if(getLabelSizeInQuarterSteps() <=2720 && getLabelSizeInQuarterSteps() >= 2600) 
        {   
            if(TPHStepsPastGapThisPrint >= 355)
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 355) - (config_.verticalPosition - 34) , 1300); 
                    return (TPHStepsPastGapThisPrint - 355) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 355) + (34 - config_.verticalPosition) , 1300); 
                    return (TPHStepsPastGapThisPrint - 355) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((355 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34), 1300); 
                    return (355 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((355 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition), 1300); 
                    return (355 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }

            //PRINTF("6.5 inch label BACKWIND\r\n");
        }
        else if(getLabelSizeInQuarterSteps() <=2860 && getLabelSizeInQuarterSteps() >= 2780)
        {
            if(TPHStepsPastGapThisPrint >= 355)
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 355) - (config_.verticalPosition - 34) , 1300); 
                    return (TPHStepsPastGapThisPrint - 355) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 355) + (34 - config_.verticalPosition) , 1300);
                    return (TPHStepsPastGapThisPrint - 355) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((355 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34), 1300);
                    return (355 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((355 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition), 1300);
                    return (355 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }

            //PRINTF("7 inch label BACKWIND\r\n");
        }
        else if(getLabelSizeInQuarterSteps() <= 3070 && getLabelSizeInQuarterSteps() >= 2990)
        {
            if(TPHStepsPastGapThisPrint >= 326)
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 326) - (config_.verticalPosition - 34) , 1300);
                    return (TPHStepsPastGapThisPrint - 326) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 326) + (34 - config_.verticalPosition) , 1300);
                    return (TPHStepsPastGapThisPrint - 326) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((326 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34), 1300); 
                    return (326 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((326 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition), 1300); 
                    return (326 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }

            //PRINTF("7.5 inch label BACKWIND\r\n");
        }
        else if(getLabelSizeInQuarterSteps() <= 3300 && getLabelSizeInQuarterSteps() >= 3200) 
        {
            if(TPHStepsPastGapThisPrint >= 326)
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 326) - (config_.verticalPosition - 34) , 1300); 
                    return (TPHStepsPastGapThisPrint - 326) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 326) + (34 - config_.verticalPosition) , 1300);
                    return (TPHStepsPastGapThisPrint - 326) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((326 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34), 1300); 
                    return (326 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((326 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition), 1300); 
                    return (326 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }

            //PRINTF("8 inch label BACKWIND\r\n");
        }
        else if(getLabelSizeInQuarterSteps() <= 3490 && getLabelSizeInQuarterSteps() >= 3400)
        {
            if(TPHStepsPastGapThisPrint >= 326)
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 326) - (config_.verticalPosition - 34) , 1300); 
                    return (TPHStepsPastGapThisPrint - 326) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 326) + (34 - config_.verticalPosition) , 1300);
                    return (TPHStepsPastGapThisPrint - 326) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((326 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34), 1300); 
                    return (326 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((326 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition), 1300); 
                    return (326 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }

            //PRINTF("8.5 inch label BACKWIND\r\n");
        }
        else if(getLabelSizeInQuarterSteps() <= 3690 && getLabelSizeInQuarterSteps() >= 3590) 
        {
            if(TPHStepsPastGapThisPrint >= 326)
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 326) - (config_.verticalPosition - 34) , 1300); 
                    return (TPHStepsPastGapThisPrint - 326) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 326) + (34 - config_.verticalPosition) , 1300); 
                    return (TPHStepsPastGapThisPrint - 326) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((326 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34), 1300); 
                    return (326 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((326 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition), 1300);
                    return (326 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }

            //PRINTF("9 inch label BACKWIND\r\n");
        }
        else if(getLabelSizeInQuarterSteps() <= 3890 && getLabelSizeInQuarterSteps() >= 3700) 
        {
            if(TPHStepsPastGapThisPrint >= 342)
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 342) - (config_.verticalPosition - 34) , 1300);
                    return (TPHStepsPastGapThisPrint - 342) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 342) + (34 - config_.verticalPosition) , 1300); 
                    return (TPHStepsPastGapThisPrint - 342) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((342 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34), 1300); 
                    return (342 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((342 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition), 1300); 
                    return (342 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }

            //PRINTF("9.5 inch label BACKWIND\r\n");
        }
        else if(getLabelSizeInQuarterSteps() <= 5000 && getLabelSizeInQuarterSteps() >= 3960) 
        { 
            if(TPHStepsPastGapThisPrint >= 326)
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 326) - (config_.verticalPosition - 34) , 1300); 
                    return (TPHStepsPastGapThisPrint - 326) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 326) + (34 - config_.verticalPosition) , 1300); 
                    return (TPHStepsPastGapThisPrint - 326) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((326 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34), 1300); 
                    return (326 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((326 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition), 1300); 
                    return (326 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }

            //PRINTF("10 inch label BACKWIND\r\n");
        }
        else
        {
            if(TPHStepsPastGapThisPrint >= 326)
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 326) - (config_.verticalPosition - 34) , 1300);
                    return (TPHStepsPastGapThisPrint - 326) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((TPHStepsPastGapThisPrint - 326) + (34 - config_.verticalPosition) , 1300); 
                    return (TPHStepsPastGapThisPrint - 326) + (34 - config_.verticalPosition);
                }
            }
            else
            {
                if(config_.verticalPosition >= 34)
                {
                    //backwindStock((326 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34), 1300);
                    return (326 - TPHStepsPastGapThisPrint) - (config_.verticalPosition - 34);
                }
                else
                {
                    //backwindStock((326 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition), 1300); 
                    return (326 - TPHStepsPastGapThisPrint) + (34 - config_.verticalPosition);
                }
            }
            
            //PRINTF("UNMATCHED label BACKWIND\r\n");
        }
    }
}