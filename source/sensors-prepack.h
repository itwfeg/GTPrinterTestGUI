#ifndef SENSORS_H
#define SENSORS_H
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "wrapperNode.h"
#include <stdbool.h>
#include "switches.h"
#include "fsl_common.h"  //needed to place code in Tighlty Coupled Memory (TCM)


#define sensors_task_PRIORITY ( configMAX_PRIORITIES - 1 )


/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DMA0_DMA16_DriverIRQHandler DMA_CH_0_16_DriverIRQHandler
#define EDMA_CHANNEL_0 		0
#define EDMA_CHANNEL_1		1
#define DMA_ADC_SOURCE		24U
#define ADC_CHANNEL_GROUP 	0U

#define MAX_ADC_CHANNELS	8U
#define BUFFER_LENGTH      	MAX_ADC_CHANNELS
#define HALF_BUFFER_LENGTH (BUFFER_LENGTH / 2U)
#define TCD_QUEUE_SIZE     1U

#define ADC_RESULT_REG_ADDR    	0x400c4024U
#define BOARD_ADC_IRQn         	ADC1_IRQn
#define DEMO_ADC_USER_CHANNEL  	10U
#define BOARD_ADC_IRQHandler 	ADC1_IRQHandler


/* interrupt handlers */
#define adcEtc1HandleIsr ADC_ETC_IRQ1_IRQHandler
#define adcEtcErrorHandleIsr ADC_ETC_ERROR_IRQ_IRQHandler
#define ADC16_0_IRQn ADC1_IRQn
#define adcModule0IsrHandler ADC1_IRQHandler
#define ADC16_1_IRQn ADC2_IRQn 
#define adcModule1IsrHandler ADC2_IRQHandler 
#define pitHandleIsr PIT_IRQHandler

/* defines */
#define GPIO1_0_15_IrqHandler 	GPIO1_0_15_IRQ_HANDLER
#define GPIO1_0_15_IrqHandler_n GPIO1_0_15_IRQ_HANDLER_N

#define GPIO2_0_15_IrqHandler	GPIO2_0_15_IRQ_HANDLER
#define GPIO2_0_15_IrqHandler_n GPIO2_0_15_IRQ_HANDLER_N

#define GPIO3_0_15_IrqHandler	GPIO3_0_15_IRQ_HANDLER
#define GPIO3_0_15_IrqHandler_n GPIO3_0_15_IRQ_HANDLER_N
															   
															   
															   
#define GPIO1_16_31_IrqHandler GPIO1_16_31_IRQ_HANDLER
#define GPIO1_16_31_IrqHandler_n GPIO1_16_31_IRQ_HANDLER_N
/* eDMA 
#define ADC0_DMA_MAJOR_CHANNEL          5U
#define ADC0_DMA_MINOR_CHANNEL          6U
#define ADC1_DMA_MAJOR_CHANNEL          7U
#define ADC1_DMA_MINOR_CHANNEL          8U
#define TRANSFER_BYTES                  16U
*/

/* adc modes of operation */
typedef enum
{
    AD_MANUAL,
    AD_AUTO
}ADConfig;


typedef struct
{
    unsigned int  adc1Channel;                          /* current channel adc1 */
    unsigned int  adc2Channel;                          /* current channel adc2 */
    unsigned int  chIndex;                              /* channel list index */
    uint32_t adc_raw_readings[MAX_ADC_CHANNELS];        /* channel conversions results */    
    bool adc_calibration_passed;
	bool adcComplete;
}ADCManager;
#if 0
typedef struct
{
	// sensor specific
	SENSOR_ID       sensor_name;
	bool	        start_recording;	//Enable monitor of acutator pressures inside BLDC
	bool	        actuator_state;		//ON or OFF
	bool	        suppress_errors;		//Flag to tell sensor driver not to report error
	bool 	        calibrating;		//Signal to created ON/OFF baseline profiles 
	unsigned short	calibrating_cycle;	
	unsigned short 	fault_occurences;	//Counter that holds the amount of consecutive faults while in monitoring mode
	bool	fault_direction;	//Direction of fault Up/Down
	bool	fault_reached;		//TRUE - if criteria is met to determine failure. 
	unsigned short	one_ms_timebase;	//resets every "start_recoring" cycle
	
	unsigned short	time_to_near_zero_diff;	//Real Time in ms. to reach "near zero diff" meaning actuator PSI is equal to Main PSI
	unsigned short 	min_near_zero_time;		//Min time taken to reach "near zero diff" during Calibration
	unsigned short	max_near_zero_time;		//Max time taken to reach "near zero diff" during Calibration 
	
	unsigned short	time_to_near_full_diff;	//Real Time in ms. to reach "near full diff" meaning actuator PSI is equal to Atmosphere(empty)
	unsigned short 	min_near_full_time;		//Min time taken to reach "near full diff" during Calibration
	unsigned short	max_near_full_time;		//Max time taken to reach "near full diff" during Calibration 
	
	unsigned short 	minimum_ON_pressure;	//Minimum Sensed pressure during ON cycle
	unsigned short	instantaneous_differential;	//Sensed Pressure drop during ON cycle
	unsigned short 	maximum_ON_pressure;	//Maximim Sensed pressure during ON cycle. Used for Vac
	
	unsigned short	buffer_index;
	
    /* Way too much RAM being used here!!!!! 450 is this needed??? */
    unsigned short 	mech_air_cal_pressure_high[PRESSURE_SENSOR_BUFFER_SIZE];//ON High Cal profile
	unsigned short 	mech_air_cal_pressure_low[PRESSURE_SENSOR_BUFFER_SIZE];	//ON Low Cal profile
	unsigned short 	mech_air_off_cal_pressure_high[PRESSURE_SENSOR_BUFFER_SIZE];//OFF High Cal profile
	unsigned short 	mech_air_off_cal_pressure_low[PRESSURE_SENSOR_BUFFER_SIZE];	//OFF Low Cal profile
	unsigned short 	mech_air_pressure[PRESSURE_SENSOR_BUFFER_SIZE];			//Real Time Pressure
	bool	send_sensor_event;	//flag to tell node_mgr.c that sensor event needs to be sent out via USB or CAN
}PRESSURE_SENSOR_CNTRL;

typedef struct
{
	BOOL 		ActuationTestON;	//Actuator Diagnostics test
	ULONG 		ActuationCycles;	
	BOOL		StartUpCycleOnly;	//Flag to cycle actuator once on startup 
	BOOL		SuppressErrors;		//Flag used by application to tell sensor driver not to report error 
	ULONG 		ActuatorOnTime;		
	ULONG 		ActuatorOffTime;	
	TimerEvent 	ActuatorOnTimer;	
	TimerEvent 	ActuatorOffTimer;	
	PRESSURE_SENSOR_CONTROL pressure_sensor;
#ifdef OPTICAL_SENSOR_FEEDBACK	
	OPTICAL_SENSOR_CONTROL  optical_sensor1;
	OPTICAL_SENSOR_CONTROL  optical_sensor2;
#endif	
}PNEUMATIC_ACTUATOR_SENSE_CONTROL;
#endif
typedef enum
{
	INVALID_SOURCE,
	GPIO_PORT1_0_15,
	GPIO_PORT1_16_31,
	GPIO_PORT2_0_15,
	GPIO_PORT2_16_31,
	GPIO_PORT3_0_15,
	GPIO_PORT3_16_31,
	ADC_PORT_1,
	ADC_PORT_2
}SENSOR_SOURCE;

typedef struct
{
	SENSOR_SOURCE source;	/* digital inputs are mapped to GPIO 1, 2, or 3*/
	LVIN_NODE_SWITCHES bits;/* sensor bit data */
}ISR_SENSOR_MSG;

/*public */
BaseType_t createSensorsTask( QueueHandle_t pMsg );
TaskHandle_t getSensorsHandle( void );
void initializeSensors( void );
WNodeID getNodeId( void ); 
NODE_SWITCHES getNodeSwitches( void );
bool isPrimaryHome( void );
bool isElevatorHome( void );
bool isShifterHome( void );
bool isShifterLimit( void );
bool isGripperHome( void );
bool isSideClampsHome( void );
bool isKnifeHome( void );
bool isSideFolderHome( void );
bool isFrontFolderHome( void );
bool isRearFolderHome( void );
bool isPusherHome( void );
bool isSecWandUp( void );
bool isSecWandDown( void );
bool isPriWandUp( void );
bool isPriWandDown( void );
bool isPriRotationHome( void );
bool isSecRotationHome( void );
bool isElevatorSensorTripped( void );

unsigned short getMainLinePressure( void );
unsigned short getTankPressure( void );

AT_QUICKACCESS_SECTION_CODE(uint32_t getADC1CountsRaw( uint32_t Channel ));   //Called from BLDC 1000uS_Tick ISR, so place in Tightly Coupled Memory (TCM)

/* private */
static void sensorsTask( void *pvParameters );
static void initManager( ADCManager *pMgr );
static void handleSensorMsg( ISR_SENSOR_MSG *sensor_data );
static void initializeAdcs( ADConfig configuration );
static void processNodeID( void );
static void closeAdc1( void );
static void closeAdc2( void );
static void initXbar( void );
static void initPitADC( void );



#endif