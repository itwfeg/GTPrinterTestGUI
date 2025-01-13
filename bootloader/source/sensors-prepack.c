#include "sensors.h"
#include "threadManager.h"
#include "queueManager.h"
#include "wrapperNode.h"
#include "translator.h"
#include "fsl_gpio.h"
#include "fsl_gpt.h"
#include "pin_mux.h"
#include "fsl_adc.h"
#include "fsl_pit.h"
#include "fsl_dmamux.h"
#include "fsl_edma.h"
#include "fsl_dmamux.h"
#include "fsl_debug_console.h"
#include <stdlib.h>
#include "board.h"
#include "switches.h"


/* threading */
static bool             suspend_				= false;
static                  TaskHandle_t pHandle_   = NULL;
static QueueHandle_t    MyQhandle_              = NULL;

/* management */
static                  ADConfig mode_          = AD_MANUAL;    
volatile                ADCManager adcManager;

bool					g_sensorTaskReady 		= false;
uint32_t				g_edmaIter 				= 0;

adc_channel_config_t 	adcChannelConfigStruct;

volatile uint32_t 		g_AdcInterruptCounter;
volatile uint32_t 		adcChannelIndex;

static WNodeID          nodeId					= _UNKNOWN_NODE;

/* Digital Inputs */
NODE_SWITCHES			NodeSensors;

/*  EDMA Variables	*/
edma_handle_t g_EDMA_Handle_0;
edma_handle_t g_EDMA_Handle_1;

/* DMA destination Address to load ADC readings */
AT_NONCACHEABLE_SECTION_INIT(uint32_t destAddr[BUFFER_LENGTH]) = {0x00U, 0x00U, 0x00U, 0x00U,
                                                                  0x00U, 0x00U, 0x00U, 0x00U};

/* DMA CH Sources - will get set during Init */
AT_NONCACHEABLE_SECTION_INIT(uint32_t adcMux[BUFFER_LENGTH]) = {0x00U, 0x00U, 0x00U, 0x00U,
                                                                0x00U, 0x00U, 0x00U, 0x00U};

/* DMA Transfer Descriptors */
AT_QUICKACCESS_SECTION_DATA_ALIGN(edma_tcd_t tcdMemoryPoolPtr_0[TCD_QUEUE_SIZE + 1], sizeof(edma_tcd_t));
AT_QUICKACCESS_SECTION_DATA_ALIGN(edma_tcd_t tcdMemoryPoolPtr_1[TCD_QUEUE_SIZE + 1], sizeof(edma_tcd_t));


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void InitializeADC_DMAMUX_Configuration(void);
static void InitializeADC_EDMATransfer(void);


/******************************************************************************/
/*!   \fn BaseType_t createSensorsTask( ADConfig mode )

      \brief
        This function intializes and creates the sensors thread.
        If the mode is set to AD_MANUAL the sensors task will manually
        intiate A/D conversions and muxing A/D channels. If the mode is set to
        AD_AUTO then no sensors task is created and the trigger for A/D conversions
        and channel muxing is handled through hardware (PIT, ADC, DMA).
      \author
          Aaron Swift
*******************************************************************************/
BaseType_t createSensorsTask(  QueueHandle_t Qhandle  )
{
	BaseType_t result;
    mode_ = AD_AUTO;
    
	MyQhandle_ = Qhandle;
	
    /* clear our manager */
    memset( (void *)&adcManager, 0, sizeof(ADCManager) );
    initManager( (ADCManager *)&adcManager );
	
	
	/* Init ADC Module */
	initializeAdcs( mode_ );       
	
	/* See if ADC Cal passed, if not just return false */
	if(adcManager.adc_calibration_passed == true)
	{
    	/* Init and Link DMA ch0, ch1 to ADC */
		InitializeADC_EDMATransfer();	
		
		/* Start the continuous cycle */
		g_AdcInterruptCounter                                       = 0U; 
		adcChannelConfigStruct.channelNumber                        = ADC_BLDC_VOLTAGE_CHANNEL;
		adcChannelConfigStruct.enableInterruptOnConversionCompleted = false;
		ADC_SetChannelConfig(ADC1, ADC_CHANNEL_GROUP, &adcChannelConfigStruct);		
		/*EnableIRQ(BOARD_ADC_IRQn);*/

		/*	Wait here until we start getting ad readings */
		while(adcManager.adc_raw_readings[ACD_NODEID_READING] == 0);
        
        #ifdef TFinkInterruptWorkAround
        /********** TFinkInterruptWorkAround: *********************************
          if DMA0_DMA16_IRQn is not the highest level interrupt, we 
          never come out of the above while loop. Would be good to understand
          why. For now, leaving at level 0 before here, then setting it to level
          10 at this end of this function 
        ***********************************************************************/
        NVIC_SetPriority(DMA0_DMA16_IRQn,10);
        #endif
		
		/* determine our node id */
		processNodeID();
		
		if(MyQhandle_ != NULL) {
			/* create sensors task thread */
			result = xTaskCreate( sensorsTask,  "SensorsTask", configMINIMAL_STACK_SIZE,
												NULL, sensors_task_PRIORITY, &pHandle_ );
			
			/* enable Digital input interrupts */
			EnableIRQ(GPIO1_0_15_IrqHandler_n);
			EnableIRQ(GPIO1_16_31_IrqHandler_n);
			EnableIRQ(GPIO2_0_15_IrqHandler_n);
			EnableIRQ(GPIO3_0_15_IrqHandler_n);
			
		}
		else {
			PRINTF("createSensorTask(): MyQHandle_ is null!\r\n" );			
		}
			
		/* give the thread manager the handle */
		updateTaskHandle( T_SENSORS );
		
		result = pdPASS;
	}
	else
	{		
		result = pdFAIL;
		PRINTF("createSensorTask(): Failed to calibrate ADC1 module!\r\n" );			
	
	}

	return result;
}


/******************************************************************************/
/*!   \fn void initializeSensors( void )                                                             
 
      \brief
        This function initializes 
          
      \author
          Aaron Swift
*******************************************************************************/ 
void initializeSensors( void )
{   
    /* any intializing of sensors for wrapper node */    
    initializeAdcs( mode_ );       
}


/******************************************************************************/
/*!   \fn WNodeID getNodeId( void )                                                            
 
      \brief
        This function return the node id. 
          
      \author
          Aaron Swift
*******************************************************************************/ 
WNodeID getNodeId( void )
{
	return nodeId;
}

/******************************************************************************/
/*!   \fn static void sensorsTask( void )

      \brief
        This function is the sensors run thread which manually intiates the 
        A/D conversions. 400uS for all channels to be processed and another 
        channel scan started. 
   
      \author
          Aaron Swift
*******************************************************************************/
static void sensorsTask( void *pvParameters )
{
    ( void ) pvParameters;
	
	ISR_SENSOR_MSG sensor_data;
     
    
    /* update or switches */
    NodeSensors.node4_switches.shifter_home_sw 		= ~GPIO_PinRead(GPIO1_11_GPIO, GPIO1_11_PIN);
    NodeSensors.node4_switches.shifter_left_sw 		= ~GPIO_PinRead(GPIO1_10_GPIO, GPIO1_10_PIN);
    NodeSensors.node4_switches.pkg_detect_elevator 	= GPIO_PinRead(GPIO1_7_GPIO, GPIO1_7_PIN);

    NodeSensors.node4_switches.right_side_cover_interlock 	= ~GPIO_PinRead(GPIO1_9_GPIO, GPIO1_9_PIN);
    NodeSensors.node4_switches.canopy_cover_interlock 		= ~GPIO_PinRead(GPIO1_8_GPIO, GPIO1_8_PIN);							

    NodeSensors.node4_switches.rear_folder_home_sw		= ~GPIO_PinRead(GPIO3_13_GPIO, GPIO3_13_PIN);
	
    /* what is this??? it's set but never used... 
    sensor.bits.lvin8		= GPIO_PinRead(GPIO3_15_GPIO, GPIO3_15_PIN); */

    PRINTF("sensorsTask(): Thread running...\r\n" ); 
        
    while( !suspend_ ) {
		
		g_sensorTaskReady = true;
		
		if( xQueueReceive( MyQhandle_, &sensor_data, portMAX_DELAY ) ) {                
            handleSensorMsg( &sensor_data );                            
        } else {
            PRINTF("systemTask(): Failed to get internal message from queue!\r\n" );      
        }                 

		/* check to see if any state has changed and notify the host */         
        taskYIELD();
    }
    vTaskSuspend(NULL); 
}

/******************************************************************************/
/*!   \fn static void handleSensorMsg( unsigned int )

      \brief
        Handles sensor data from interrupt service routine.
		Will populate respective sensor bits based on NODE ID.
           
      \author
          Carlos Guzman
*******************************************************************************/
static void handleSensorMsg( ISR_SENSOR_MSG *sensor_data )
{
    
	/* update this node's sensor data */
    switch( nodeId )
	{
		case _NODE4:
		{
			if( sensor_data->source == GPIO_PORT1_0_15 )
			{
				NodeSensors.node4_switches.shifter_home_sw 		= ~sensor_data->bits.lvin2;
				NodeSensors.node4_switches.shifter_left_sw 		= ~sensor_data->bits.lvin3;
				NodeSensors.node4_switches.pkg_detect_elevator 	= sensor_data->bits.lvin4;
		
				NodeSensors.node4_switches.right_side_cover_interlock 	= sensor_data->bits.interlock1;
				NodeSensors.node4_switches.canopy_cover_interlock 		= sensor_data->bits.interlock2;							
			}
				
			if( sensor_data->source == GPIO_PORT3_0_15 )
			{           
                NodeSensors.node4_switches.rear_folder_home_sw 	= ~sensor_data->bits.lvin1;  								
			}
            
            WrCAN_GNM_SwitchStatus msg;
            msg.switch_status = NodeSensors.generic_switches;
            /* update master with latest changes */
            send_GNM_SwitchStatus( &msg );
		}
		break;
		
		default:
		{
			PRINTF("SensorTask(): message - Invalid Node ID %dr\n", nodeId );      		
		}
		
	}
    
	//PRINTF("SensorTask(): message - Data %d\r\n", sensor_data->bits.lvin1 );      
    
}

/******************************************************************************/
/*!   \fn static void initManager( ADCManager *pMgr )

      \brief
        This function initializes the ADC manager for manual mode operation. 
        Setup of current working channel and channel list.

      \author
          Aaron Swift
*******************************************************************************/
static void initManager( ADCManager *pMgr )
{

}

/******************************************************************************/
/*!   \fn static void processNodeID( void )

      \brief
        This function determines which node type we are by reading the nodeID
        a/d channel.  

      \author
          Aaron Swift
*******************************************************************************/
static void processNodeID( void )
{
    uint32_t node_id_raw_value = adcManager.adc_raw_readings[ACD_NODEID_READING];
	
    if( ( node_id_raw_value >= ID1MinValue ) && ( node_id_raw_value <= ID1MaxValue ) ) {
        PRINTF("processNodeID() Wrapper nodeid is 1\r\n");
        nodeId = _NODE1;        
    } else if( ( node_id_raw_value >= ID2MinValue ) && ( node_id_raw_value <= ID2MaxValue ) ) {
        PRINTF("processNodeID() Wrapper nodeid is 2\r\n");
        nodeId = _NODE2;        
    } else if( ( node_id_raw_value >= ID3MinValue ) && ( node_id_raw_value <= ID3MaxValue ) ) {
        PRINTF("processNodeID() Wrapper nodeid is 3\r\n");
        nodeId = _NODE3;        
    } else if( ( node_id_raw_value >= ID4MinValue ) && ( node_id_raw_value <= ID4MaxValue ) ) {
        PRINTF("processNodeID() Wrapper nodeid is 4\r\n");
        nodeId = _NODE4;        
    } else if( ( node_id_raw_value >= ID5MinValue ) && ( node_id_raw_value <= ID5MaxValue ) ) {
        PRINTF("processNodeID() Wrapper nodeid is 5\r\n");
        nodeId = _NODE5;        
    } else {
        PRINTF("processNodeID() Failed, NodeId Unknown!\r\n");
        nodeId = _UNKNOWN_NODE;    
    }
	    
}



TaskHandle_t getSensorsHandle(){
  return pHandle_;
}

static void closeAdc1( void )
{
    ADC_Deinit( ADC1 );  
}

static void closeAdc2( void )
{
    ADC_Deinit( ADC2 );  
}



/******************************************************************************/
/*!   \fn static void initializeAdcs( ADConfig configuration )                                                            
 
      \brief
        This function initializes A/D converter modules 1 & 2.
          
      \author
          Aaron Swift
*******************************************************************************/ 
static void initializeAdcs( ADConfig configuration )
{
    adc_config_t adcConfig;
	adc_hardware_average_mode_t adcAvgMode;
    static bool calOnce_ = false;
    
    /* setup adc clock */
    CLOCK_SetMux( kCLOCK_PerclkMux, 1U );    
    CLOCK_SetDiv( kCLOCK_PerclkDiv, 0U );
    
    ADC_GetDefaultConfig( &adcConfig );
    
	adcConfig.clockDriver = kADC_ClockDriver8;
                  
    /* initialize the adc modules */
    ADC_Init( ADC1, &adcConfig );
    	
	/* set Hardware Averaging */
	adcAvgMode = kADC_HardwareAverageCount4;
	
	
	/*	based on the kADC_ClockDriver8 and kADC_HardwareAverageCount4,
		we get Ch0..7 results every 800uS. we could go much faster if needed. */ 	
	ADC_SetHardwareAverageConfig( ADC1, adcAvgMode );
	
    if( configuration == AD_MANUAL ) {  
        ADC_EnableHardwareTrigger( ADC1, false );
        ADC_EnableHardwareTrigger( ADC2, false );
    } else {
		
		/* Disable hardware trigger */
		ADC_EnableHardwareTrigger(ADC1, false);
    }
    
    if( !calOnce_ ) {
        /* auto hardware calibration adc module */
        if( kStatus_Success != ADC_DoAutoCalibration( ADC1 ) ) {
			PRINTF("ADC_DoAutoCalibration() Failed.\r\n");
			adcManager.adc_calibration_passed = false;
        
		} else {            
            PRINTF("ADC_DoAutoCalibration() Done.\r\n");
			adcManager.adc_calibration_passed = true;
			calOnce_ = true;
        }
    }

	/* enable DMA */
	ADC_EnableDMA(ADC1, true);
	
	/* Setup ADC Channel list used by DMA */	
	adcMux[0] = ADC_BLDC_VOLTAGE_CHANNEL;		/* Same assignment on all nodes */
	adcMux[1] = ADC_AMBIENT_TEMP_CHANNEL;		/* Only Node 1 uses */
	adcMux[2] = ACD_MOTOR1_CURRENT_CHANNNEL;	/* Same assignment on all nodes */
	adcMux[3] = ACD_MOTOR2_CURRENT_CHANNNEL;	/* Same assignment on all nodes */
	adcMux[4] = ACD_A4_CHANNNEL;				/* Node1-SecVaccum, Node4-MainPsi, Node5-PriVaccum */
	adcMux[5] = ACD_A5_CHANNNEL;				/* Node4-TankPsi, Node5-SealerTemp */
	adcMux[6] = ADC_NODEID_CHANNEL;				/* Same assignment on all nodes */   
}

/******************************************************************************/
/*!   \fn static void EDMA_Callback_0(edma_handle_t *handle, 
									  void *param, bool transferDone, 
									  uint32_t tcds);
 
      \brief
        User callback function for EDMA transfer.
          
      \author
          Carlos Guzman
*******************************************************************************/ 
void EDMA_Callback_0(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
    ISR_SENSOR_MSG sensor;
	
    QueueHandle_t sensor_task_qhandle   = getSensorMsgQueueHandle();
	
	GPIO_WritePinOutput( GPIO1_22_GPIO, GPIO1_22_PIN, false );
	
	sensor.source = ADC_PORT_1;
	
	//g_edmaIter ++;
	
	/* copy adc results locally */
	memcpy( (void*)&adcManager.adc_raw_readings, (void*)&destAddr, sizeof(destAddr));

	/* reset channels and start readings again */
	EDMA_ResetChannel(g_EDMA_Handle_0.base, g_EDMA_Handle_0.channel);
	EDMA_InstallTCD(DMA0, EDMA_CHANNEL_0, tcdMemoryPoolPtr_0);
	
	EDMA_ResetChannel(g_EDMA_Handle_1.base, g_EDMA_Handle_1.channel);
	EDMA_InstallTCD(DMA0, EDMA_CHANNEL_1, tcdMemoryPoolPtr_1);
	
	EDMA_EnableAsyncRequest(DMA0, EDMA_CHANNEL_0, true);

	/* Enable transfer. */
	EDMA_StartTransfer(&g_EDMA_Handle_0);
	
	/* start readings */
	ADC_SetChannelConfig(ADC1, ADC_CHANNEL_GROUP, &adcChannelConfigStruct);	
		
	GPIO_WritePinOutput( GPIO1_22_GPIO, GPIO1_22_PIN, true );
}

void EDMA_Callback_1(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{	
	g_edmaIter ++;
	
	if(g_edmaIter ==7)
		g_edmaIter = 0;	
}

/******************************************************************************/
/*!   \fn static void InitializeADC_DMAMUX_Configuration(void);									  
 
      \brief
        Function Connects ADC1 to EDMA CH0. EDMA CH1 is set to always enable.
		EDMA CH1 will be triggered by EDMA CH0 during operation.
          
      \author
          Carlos Guzman
*******************************************************************************/ 
static void InitializeADC_DMAMUX_Configuration(void)
{
    /* Configure DMAMUX */
    DMAMUX_Init(DMAMUX);
    DMAMUX_SetSource(DMAMUX, EDMA_CHANNEL_0, DMA_ADC_SOURCE); /* Map ADC source to channel 0 */
    DMAMUX_EnableChannel(DMAMUX, EDMA_CHANNEL_0);
	
	DMAMUX_EnableAlwaysOn(DMAMUX, EDMA_CHANNEL_1, true);
	
	DMAMUX_EnableChannel(DMAMUX, EDMA_CHANNEL_1);
}

/******************************************************************************/
/*!   \fn void InitializeADC_EDMATransfer(void)
 
      \brief
        Function will configure EDMA CH0 to handle reading ADC convertions.
		Once EDMA CH0 finishes reading, it will trigger EDMA CH1 to increment 
		ADC Channel and start another conversion.  

		EDMA CH0 will call the User Callback once all ADC channels have been read. 
          
      \author
          Carlos Guzman
*******************************************************************************/ 
void InitializeADC_EDMATransfer(void)
{
    edma_transfer_config_t transferConfig;
    edma_config_t userConfig;
    
    
	/* connect ADC source to DMA ch 0 */
	InitializeADC_DMAMUX_Configuration();
	
	memset(destAddr, 0U, sizeof(destAddr));

    EDMA_GetDefaultConfig(&userConfig);
	userConfig.enableDebugMode = true;
    
	EDMA_Init(DMA0, &userConfig);

	/* clear transfer descriptors */
    memset(&transferConfig, 0, sizeof(edma_transfer_config_t));
    memset(&tcdMemoryPoolPtr_0, 0, sizeof(edma_tcd_t));
    EDMA_CreateHandle(&g_EDMA_Handle_0, DMA0, EDMA_CHANNEL_0);
    EDMA_SetCallback(&g_EDMA_Handle_0, EDMA_Callback_0, NULL);
    EDMA_ResetChannel(g_EDMA_Handle_0.base, g_EDMA_Handle_0.channel);
    /* Configure and submit transfer structure 1 */

	EDMA_PrepareTransfer(&transferConfig, 	(void *)ADC_RESULT_REG_ADDR, 	sizeof(destAddr[0]), 
						 						  	&destAddr[0], 			sizeof(destAddr[0]),
                         						  	sizeof(destAddr[0]),     			/* minor loop bytes */
                         						   	sizeof(destAddr[0]) * BUFFER_LENGTH, /* major loop counts */
                         							kEDMA_PeripheralToMemory);
	
    EDMA_TcdSetTransferConfig(tcdMemoryPoolPtr_0, &transferConfig, NULL);
    EDMA_TcdSetChannelLink(tcdMemoryPoolPtr_0, kEDMA_MinorLink, EDMA_CHANNEL_1);
    EDMA_TcdEnableInterrupts(tcdMemoryPoolPtr_0, kEDMA_MajorInterruptEnable);
    EDMA_InstallTCD(DMA0, EDMA_CHANNEL_0, tcdMemoryPoolPtr_0);
	
	/* Init Chan 1, transfer from mem to peripheral */
    memset(&transferConfig, 0, sizeof(edma_transfer_config_t));
    memset(&tcdMemoryPoolPtr_1, 0, sizeof(edma_tcd_t));
    EDMA_CreateHandle(&g_EDMA_Handle_1, DMA0, EDMA_CHANNEL_1);
    
	EDMA_SetCallback(&g_EDMA_Handle_1, NULL, NULL);
    EDMA_ResetChannel(g_EDMA_Handle_1.base, g_EDMA_Handle_1.channel);
    
	/* Configure and submit transfer structure 2 */
	EDMA_PrepareTransfer(&transferConfig, 	&adcMux[0], 			sizeof(adcMux[0]), 
						 				  	(void *)ADC1, 	sizeof(adcMux[0]),
                         					sizeof(adcMux[0]), /* minor loop bytes*/
                         					sizeof(adcMux[0]), /* Total number of bytes, major loop count 1 */
                         					kEDMA_MemoryToPeripheral);
	
    EDMA_TcdSetTransferConfig(tcdMemoryPoolPtr_1, &transferConfig, NULL);
    
    EDMA_TcdEnableInterrupts(tcdMemoryPoolPtr_1, kEDMA_MajorInterruptEnable);
    EDMA_InstallTCD(DMA0, EDMA_CHANNEL_1, tcdMemoryPoolPtr_1);

	
#if defined(FSL_FEATURE_EDMA_ASYNCHRO_REQUEST_CHANNEL_COUNT) && FSL_FEATURE_EDMA_ASYNCHRO_REQUEST_CHANNEL_COUNT
    /* Enable async DMA request. */
    EDMA_EnableAsyncRequest(DMA0, EDMA_CHANNEL_0, true);
#endif /* FSL_FEATURE_EDMA_ASYNCHRO_REQUEST_CHANNEL_COUNT */
    
	/* Enable transfer. */
    EDMA_StartTransfer(&g_EDMA_Handle_0);
	
}

unsigned short getMainLinePressure( void )
{
    #if 0
    return (unsigned short)getADC1CountsRaw( ACD_MAIN_PSI_CHANNNEL );
    #else     
    return (unsigned short)adcManager.adc_raw_readings[ACD_A5_READING];
    #endif    
}

unsigned short getTankPressure( void )
{
    #if 0
    return (unsigned short)getADC1CountsRaw( ACD_TANK_PSI_CHANNNEL );
    #else
    return (unsigned short)adcManager.adc_raw_readings[ACD_A4_READING];
    #endif
}

/******************************************************************************/
/*!   \fn void showADCReadings( void )                                        
 
      \brief
        This function shows the adc values measured. 

      \author
          Aaron Swift
*******************************************************************************/ 
void showADCReadings( void )
{
    #if 0
    float resolution = .0130, value = 0.0;
    PRINTF("showADCReadings(): ***********************\r\n" );

    PRINTF( "head voltage counts: %d\r\n", adcManager.value[CHANNEL_HEAD_VOLT] ); 
    value = (float)(adcManager.value[CHANNEL_HEAD_VOLT] * resolution);
    PRINTF( "head voltage: %2.3fV\r\n", value );    
    /* PRINTF( "head shoot counts: %d\r\n", adcManager.value[CHANNEL_SHOOT] );  
    value = (float)(adcManager.value[CHANNEL_SHOOT] * resolution); */ 
    PRINTF( "head shoot voltage: %2.3fV\r\n", value );    

    PRINTF( "head state counts: %d\r\n", adcManager.value[CHANNEL_HEAD_STATE] );   
    value = (float)(adcManager.value[CHANNEL_HEAD_STATE] * resolution);
    PRINTF( "head state voltage: %2.3fV\r\n", value );    
    PRINTF( "head dot counts: %d\r\n", adcManager.value[CHANNEL_HEAD_DOT] );    
    value = (float)(adcManager.value[CHANNEL_HEAD_DOT] * resolution);
    PRINTF( "head dot voltage: %2.3fV\r\n", value );    
   
    if( headSensors.headUp > NO_CASSET_THRESHOLD ) {
        PRINTF( "Head is up. \r\n" ); 
    }

    if( headSensors.headUp > WIDE_CASSET_THRESHOLD ) {
        PRINTF( "Wide label casset installed. \r\n" ); 
    }

    PRINTF( "media counts: %d\r\n", adcManager.value[CHANNEL_SHOOT] );
    value = (float)(adcManager.value[CHANNEL_SHOOT] * resolution);
    PRINTF( "media voltage: %2.3fV\r\n", value ); 
    
    if( headSensors.mediaDetector > MEDIA_SYNC_BAR_THRESHOLD ) {
        PRINTF( "label sync bar detected. \r\n" ); 
    }
    if( headSensors.labelTaken ) {
        PRINTF( "take label. \r\n" );       
    }

    PRINTF( "head temperature counts: %d\r\n", adcManager.value[CHANNEL_HEAD_TEMP] );
    int degrees = 0;
    if( adcManager.value[CHANNEL_HEAD_TEMP] >= ZERO_DEGREES_COUNT ) {
        degrees = ZERO_DEGREES_C;
    } else if( adcManager.value[CHANNEL_HEAD_TEMP] >= TEN_DEGREES_COUNT ) {
        degrees = TEN_DEGREES_C;
    } else if( adcManager.value[CHANNEL_HEAD_TEMP] >= TWENTY_DEGREES_COUNT ) {
        degrees = TWENTY_DEGREES_C;      
    } else if( adcManager.value[CHANNEL_HEAD_TEMP] >= THIRTY_DEGREES_COUNT ) {
          degrees = THIRTY_DEGREES_C;      
    } else if( adcManager.value[CHANNEL_HEAD_TEMP] >= FOURTY_DEGREES_COUNT ) {
        degrees = FOURTY_DEGREES_C;      
    } else if( adcManager.value[CHANNEL_HEAD_TEMP] >= FIFTY_DEGREES_COUNT ) {
        degrees = FIFTY_DEGREES_C;
    } else {
        degrees = SIXTY_DEGREES_C;
    }      
    PRINTF( "head temperature: %d°C\r\n", degrees );             
    PRINTF("showADCReadings(): ***********************\r\n" );
    #endif
}


void BOARD_ADC_IRQHandler(void)
{
#if 0/*Enable to determine timing	*/
	GPIO_PinWrite(GPIO1_25_GPIO, GPIO1_25_PIN, false);
    
	g_AdcConversionDoneFlag = true;
    /* Read conversion result to clear the conversion completed flag. */
    g_AdcConversionValue = ADC_GetChannelConversionValue(DEMO_ADC_BASE, DEMO_ADC_CHANNEL_GROUP);
    g_AdcInterruptCounter++;
	
	/* start new conversion */
	adcChannelConfigStruct.channelNumber ++;
	if(adcChannelConfigStruct.channelNumber > 10)
		adcChannelConfigStruct.channelNumber = 0;
	
	ADC_SetChannelConfig(DEMO_ADC_BASE, ADC_CHANNEL_GROUP, &adcChannelConfigStruct);
	
	GPIO_PinWrite(GPIO1_25_GPIO, GPIO1_25_PIN, true);
#endif	
    SDK_ISR_EXIT_BARRIER;
}

/******************************************************************************/
/*!   \fn void GPIO1_0_15_IrqHandler( void )                                        
 
      \brief
        Interrupt Handler for GPIO 1 pins 0-15
		LVIN2		- GPIO 1 pin 11
		LVIN3		- GPIO 1 pin 10
		LVIN4		- GPIO 1 pin 7
		INTERLOCK1 	- GPIO 1 pin 9
		INTERLOCK2 	- GPIO 1 pin 8
		

      \author
          Carlos Guzman
*******************************************************************************/ 
void GPIO1_0_15_IrqHandler(void)
{
	ISR_SENSOR_MSG sensor;
	BaseType_t xHigherPriorityTaskWoken = false;
    QueueHandle_t sensor_task_qhandle   = getSensorMsgQueueHandle();
	uint32_t int_flags 					= GPIO_GetPinsInterruptFlags( GPIO1 );

	/* determine if int source is sensor input */ 
	if( int_flags & (1U << GPIO1_11_PIN) || int_flags & (1U << GPIO1_10_PIN)	||
	    int_flags & (1U << GPIO1_7_PIN)  || int_flags & (1U << GPIO1_9_PIN) 	||
		int_flags & (1U << GPIO1_8_PIN)	)
	{
		/* read all inputs on this port */
		sensor.source			= GPIO_PORT1_0_15;
		sensor.bits.lvin2		= GPIO_PinRead(GPIO1_11_GPIO, GPIO1_11_PIN);
		sensor.bits.lvin3		= GPIO_PinRead(GPIO1_10_GPIO, GPIO1_10_PIN);
		sensor.bits.lvin4		= GPIO_PinRead(GPIO1_7_GPIO, GPIO1_7_PIN);
		sensor.bits.interlock1	= GPIO_PinRead(GPIO1_9_GPIO, GPIO1_9_PIN);
		sensor.bits.interlock2	= GPIO_PinRead(GPIO1_8_GPIO, GPIO1_8_PIN);
		
#if 1
		/* notify Sensor task of Input change */
		if( sensor_task_qhandle != NULL )
		{
			if( xQueueSendToBackFromISR( sensor_task_qhandle, (void *)&sensor, &xHigherPriorityTaskWoken ) != pdPASS ) {
				PRINTF("GPIO1_0_15_IrqHandler: Failed to queue GPIO1_0_15_IrqHandler transaction: %d!\r\n", sensor );
			}
		}
#endif
		/* clear the interrupt flags */
		if( int_flags & (1U << GPIO1_11_PIN) )
			GPIO_PortClearInterruptFlags(GPIO1_11_GPIO, 1U << GPIO1_11_PIN);
		if( int_flags & (1U << GPIO1_10_PIN) )
			GPIO_PortClearInterruptFlags(GPIO1_10_GPIO, 1U << GPIO1_10_PIN);
		if( int_flags & (1U << GPIO1_7_PIN) )
			GPIO_PortClearInterruptFlags(GPIO1_7_GPIO, 1U << GPIO1_7_PIN);
		if( int_flags & (1U << GPIO1_9_PIN) )
			GPIO_PortClearInterruptFlags(GPIO1_9_GPIO, 1U << GPIO1_9_PIN);
		if( int_flags & (1U << GPIO1_8_PIN) )
			GPIO_PortClearInterruptFlags(GPIO1_8_GPIO, 1U << GPIO1_8_PIN);

	}
	
    SDK_ISR_EXIT_BARRIER;
}							

/******************************************************************************/
/*!   \fn void GPIO2_0_15_IrqHandler( void )                                        
 
      \brief
        Interrupt Handler for GPIO 2 pins 0-15
		LVIN5		- GPIO 2 pin 5
		LVIN6		- GPIO 2 pin 8
		LVIN7		- GPIO 2 pin 15
		
		

      \author
          Carlos Guzman
*******************************************************************************/ 
void GPIO2_0_15_IrqHandler( void )
{
	ISR_SENSOR_MSG sensor;
	BaseType_t xHigherPriorityTaskWoken = false;
    QueueHandle_t sensor_task_qhandle   = getSensorMsgQueueHandle();
	uint32_t int_flags 					= GPIO_GetPinsInterruptFlags( GPIO2 );


#ifdef TFinkRevA1PCBA
       //TFinkToDOLVIN5 need to handle lvin5 somewhere..... 
	/* determine if int source is sensor input */ 
	if( int_flags & (1U << GPIO2_5_PIN) || int_flags & (1U << GPIO2_8_PIN)	||
	    int_flags & (1U << GPIO2_15_PIN) )
	{
		/* read all inputs on this port */
		sensor.source			= GPIO_PORT2_0_15;
		sensor.bits.lvin5		= GPIO_PinRead(GPIO2_5_GPIO, GPIO2_5_PIN);
		sensor.bits.lvin6		= GPIO_PinRead(GPIO2_8_GPIO, GPIO2_8_PIN);
		sensor.bits.lvin7		= GPIO_PinRead(GPIO2_15_GPIO, GPIO2_15_PIN);
#elif defined(TFinkRevA2PCBA)
       //TFinkToDOLVIN5 need to handle lvin5 somewhere.....   
    /* determine if int source is sensor input */ 
	if( int_flags & (1U << GPIO2_8_PIN)	|| int_flags & (1U << GPIO2_15_PIN) )
	{
		/* read all inputs on this port */
		sensor.source			= GPIO_PORT2_0_15;
		sensor.bits.lvin6		= GPIO_PinRead(GPIO2_8_GPIO, GPIO2_8_PIN);
		sensor.bits.lvin7		= GPIO_PinRead(GPIO2_15_GPIO, GPIO2_15_PIN);            
#endif 
		
#if 1		
		/* notify Sensor task of Input change */
		if( sensor_task_qhandle != NULL )
		{
			if( xQueueSendToBackFromISR( sensor_task_qhandle, (void *)&sensor, &xHigherPriorityTaskWoken ) != pdPASS ) {
				PRINTF("GPIO2_0_15_IrqHandler: Failed to queue GPIO2_0_15_IrqHandler transaction: %d!\r\n", sensor );
			}
		}	
#endif
		
#ifdef TFinkRevA1PCBA
		/* clear the interrupt flags */
		if( int_flags & (1U << GPIO2_5_PIN) )
			GPIO_PortClearInterruptFlags(GPIO2_5_GPIO, 1U << GPIO2_5_PIN);
#elif defined(TFinkRevA2PCBA)
             //TFinkToDOLVIN5 need to handle lvin5 somewhere.....       
#endif
		if( int_flags & (1U << GPIO2_8_PIN) )
			GPIO_PortClearInterruptFlags(GPIO2_8_GPIO, 1U << GPIO2_8_PIN);
		if( int_flags & (1U << GPIO2_15_PIN) )
			GPIO_PortClearInterruptFlags(GPIO2_15_GPIO, 1U << GPIO2_15_PIN);	
	}
	
    SDK_ISR_EXIT_BARRIER;
}							


/******************************************************************************/
/*!   \fn void GPIO3_0_15_IrqHandler( void )                                        
 
      \brief
        Interrupt Handler for GPIO 3 pins 0-15
		LVIN1		- GPIO 3 pin 13
		LVIN8		- GPIO 3 pin 15
		
		
		

      \author
          Carlos Guzman
*******************************************************************************/ 
void GPIO3_0_15_IrqHandler(void)
{
	ISR_SENSOR_MSG sensor;
	BaseType_t xHigherPriorityTaskWoken = false;
    QueueHandle_t sensor_task_qhandle   = getSensorMsgQueueHandle();
	uint32_t int_flags 					= GPIO_GetPinsInterruptFlags( GPIO3 );

	/* determine if int source is sensor input */ 
	if( int_flags & (1U << GPIO3_13_PIN) || int_flags & (1U << GPIO3_15_PIN) )
	{
		/* read all inputs on this port */
		sensor.source			= GPIO_PORT3_0_15;
		sensor.bits.lvin1		= GPIO_PinRead(GPIO3_13_GPIO, GPIO3_13_PIN);
		sensor.bits.lvin8		= GPIO_PinRead(GPIO3_15_GPIO, GPIO3_15_PIN);
#if 1			
		/* notify Sensor task of Input change */
		if( sensor_task_qhandle != NULL )
		{
			if( xQueueSendToBackFromISR( sensor_task_qhandle, (void *)&sensor, &xHigherPriorityTaskWoken ) != pdPASS ) {
				PRINTF("GPIO3_0_15_IrqHandler: Failed to queue GPIO3_0_15_IrqHandler transaction: %d!\r\n", sensor );
			}
		}
#endif
		/* clear the interrupt flags */
		if( int_flags & (1U << GPIO3_13_PIN) )
			GPIO_PortClearInterruptFlags(GPIO3_13_GPIO, 1U << GPIO3_13_PIN);
		if( int_flags & (1U << GPIO3_15_PIN) )
			GPIO_PortClearInterruptFlags(GPIO3_15_GPIO, 1U << GPIO3_15_PIN);
		
	}
	
    SDK_ISR_EXIT_BARRIER;
}		

TFinkDebugRUFHomeSwitch
/** Initialize RUF Home Sensor. This code not inteneded for production! */
void TFinkInitializeRUFHomeSwitch(void)
{
        ISR_SENSOR_MSG sensor;
	    BaseType_t xHigherPriorityTaskWoken = false;
        QueueHandle_t sensor_task_qhandle   = getSensorMsgQueueHandle();
  
  		sensor.source			= GPIO_PORT3_0_15;
		sensor.bits.lvin1		= GPIO_PinRead(GPIO3_13_GPIO, GPIO3_13_PIN);
		sensor.bits.lvin8		= GPIO_PinRead(GPIO3_15_GPIO, GPIO3_15_PIN);
			
		/* notify Sensor task of Input change */
		if( sensor_task_qhandle != NULL )
		{
			if( xQueueSendToBackFromISR( sensor_task_qhandle, (void *)&sensor, &xHigherPriorityTaskWoken ) != pdPASS ) {
				PRINTF("GPIO3_0_15_IrqHandler: Failed to queue GPIO3_0_15_IrqHandler transaction: %d!\r\n", sensor );
			}
		} 
}


/******************************************************************************/
/*!   \fn void GPIO1_16_31_IrqHandler( void )                                        
 
      \brief
        Interrupt Handler for GPIO 1 pins 0-15

      \author
          Carlos Guzman
*******************************************************************************/
void GPIO1_16_31_IrqHandler(void)
{
	ISR_SENSOR_MSG sensor_data;
	BaseType_t xHigherPriorityTaskWoken = false;
    QueueHandle_t sensor_task_qhandle   = getSensorMsgQueueHandle();
	uint32_t int_flags 					= GPIO_GetPinsInterruptFlags( GPIO1 );
	
	
	/* determine which pin */
	if( int_flags & (1U << GPIO1_22_PIN) )
	{
		sensor_data.source		= GPIO_PORT1_16_31;
		sensor_data.bits.lvin1  = GPIO_PinRead(GPIO1_22_GPIO, GPIO1_22_PIN);
			
		/* clear the interrupt status */
		GPIO_PortClearInterruptFlags(GPIO1_22_GPIO, 1U << GPIO1_22_PIN);	

#if 1
		/* notify Sensor task of Input change */
		if( sensor_task_qhandle != NULL )
		{
			if( xQueueSendToBackFromISR( sensor_task_qhandle, (void *)&sensor_data, &xHigherPriorityTaskWoken ) != pdPASS ) {
				PRINTF("GPIO1_16_31_IrqHandler: Failed to queue GPIO1_16_31_IrqHandler transaction: %d!\r\n", sensor_data );
			}
		}
#endif    		
	
	}

	
    SDK_ISR_EXIT_BARRIER;
}

NODE_SWITCHES getNodeSwitches( void )
{
	return NodeSensors;	
}	

bool isPrimaryHome( void )
{
    return NodeSensors.node1_switches.primary_home_sw;
}

bool isElevatorHome( void )
{
    return NodeSensors.node1_switches.elevator_home_sw;
}

bool isKnifeHome( void )
{
    return NodeSensors.node1_switches.knife_down_sw;
}

bool isShifterHome( void )
{
    return NodeSensors.node4_switches.shifter_home_sw;
}

bool isShifterLimit( void )
{
    return NodeSensors.node4_switches.shifter_left_sw;
}

bool isGripperHome( void )
{
    return NodeSensors.node2_switches.gripper_home_sw;
}

bool isSideClampsHome( void )
{
    return NodeSensors.node2_switches.side_clamp_home_sw;
}

bool isSideFolderHome( void )
{
	return NodeSensors.node5_switches.side_folder_home_sw;
}

bool isFrontFolderHome( void )
{
    return NodeSensors.node3_switches.front_folder_home_sw;
}

bool isRearFolderHome( void )
{
    return NodeSensors.node4_switches.rear_folder_home_sw;
}

bool isPusherHome( void )
{
    return NodeSensors.node3_switches.pusher_home_sw;
}

bool isLabelApplierHome( void )
{
    return NodeSensors.node5_switches.pri_la_rot_home_sw;
}

bool isPriWandUp( void )
{
    bool result = false;
    return result;
}

bool isPriWandDown( void )
{
    bool result = false;
    return result;
}

bool isElevatorSensorTripped( void )
{
    bool result = false;
    /* TO DO: */
    return result;
}

/* Analog functions */
/******************************************************************************/
/*!   \fn uint32_t getADC1CountsRaw( uint32_t Channel )
 
      \brief
        Function returns current ADC1 value for the passed in channel index.

		Defined Channels:
		ADC_BLDC_VOLTAGE_CHANNEL 		-	Common to all nodes
		ADC_AMBIENT_TEMP_CHANNEL		-	Only Node1
		ACD_MOTOR1_CURRENT_CHANNNEL		-	Common to all nodes
		ACD_MOTOR2_CURRENT_CHANNNEL		-	Common to all nodes
		ACD_PRI_LA_VAC_CHANNNEL			-	Only Node5
		ACD_SEC_LA_VAC_CHANNNEL			-	Only Node1
		ACD_MAIN_PSI_CHANNNEL			-	Only Node4
		ACD_TANK_PSI_CHANNNEL			-	Only Node4
		ACD_SEALERBELT_TEMP_CHANNNEL	-	Only Node5
		ADC_NODEID_CHANNEL				-	Common to all nodes

      \author
          Carlos Guzman
*******************************************************************************/
uint32_t getADC1CountsRaw( uint32_t Channel )
{
	uint32_t raw_counts = 0;
	

	/* These channels all resolve to the same ADC channel */
	if( Channel == ACD_PRI_LA_VAC_CHANNNEL 	||
	    Channel == ACD_SEC_LA_VAC_CHANNNEL	||
		Channel == ACD_MAIN_PSI_CHANNNEL )
	{
		Channel = ACD_A4_CHANNNEL; 	
	}
	
	/* These channels too all resolve to the same ADC channel */
	if( Channel == ACD_TANK_PSI_CHANNNEL	||
	    Channel == ACD_SEALERBELT_TEMP_CHANNNEL )
	{
		Channel = ACD_A5_CHANNNEL; 	
	}
	
	switch(Channel)
	{
		case ADC_BLDC_VOLTAGE_CHANNEL:
		{
			raw_counts = adcManager.adc_raw_readings[ADC_BLDC_VOLTAGE_READING];
		}break;
		
		case ADC_AMBIENT_TEMP_CHANNEL:
		{
			raw_counts = adcManager.adc_raw_readings[ADC_AMBIENT_TEMP_READING];
		}break;
		
		case ACD_MOTOR1_CURRENT_CHANNNEL:
		{
			raw_counts = adcManager.adc_raw_readings[ACD_MOTOR1_CURRENT_READING];
		}break;
		
		case ACD_MOTOR2_CURRENT_CHANNNEL:
		{
			raw_counts = adcManager.adc_raw_readings[ACD_MOTOR2_CURRENT_READING];
		}break;
		
		case ACD_A4_CHANNNEL:
		{
			raw_counts = adcManager.adc_raw_readings[ACD_A4_READING];
		}break;
		
		case ACD_A5_CHANNNEL:
		{
			raw_counts = adcManager.adc_raw_readings[ACD_A5_READING];
		}break;
		
		case ADC_NODEID_CHANNEL:
		{
			raw_counts = adcManager.adc_raw_readings[ACD_NODEID_READING];
		}break;
		
		default:
		{
			PRINTF("getADC1CountsRaw(): Invalid channel parameter: %d!\r\n", Channel );			
		}
	}
		
	return raw_counts;	
	
}
