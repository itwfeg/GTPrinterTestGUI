#include "bootTask.h"
#include "fsl_debug_console.h"
#include "fsl_flexspi.h"
#include "threadManager.h"
#include "queueManager.h"
//#include "sst25vf010a.h"
#include "bootloader.h"
#include "board.h"
#include "queue.h"
#include "vendor.h"
//#include "bootCanMessages.h"
#include "MIMXRT1024.h"
#include "spi_flash_api.h"
#include "fsl_cache.h"
//#include "wrapperNode.h"
//#include "sensors.h"
#include "bootloader_core.h"
#include "common.h"
//#include "fsl_adc.h"
#include "usbTask.h"
#include "translator.h"

#if 1
#define CAN_FRAME_TEST
#endif

/* k64 peripheral instance */
// extern Driver_Instance_Table_t inst;
// extern SPI_Device_t SPI1_Dev;
extern const unsigned char bootSwM;
extern const unsigned char bootSwm;
extern const unsigned char bootSwe;

extern usb_device_composite_struct_t usbMgr;

#define PAGES_TO_WRITE 100


//static uint8_t writeBuffer[numToWrite];
//static uint8_t sendBuffer[FLASH_PAGE_SIZE];
static uint8_t sendBuffer[100];
extern unsigned char imageBuffer[USB_BULK_PAYLOAD];

static TaskHandle_t bootTaskHandle_ = NULL;
static QueueHandle_t bootMsgHandle_ = NULL;
static bool suspend_ = false;
static TimerHandle_t boot_ready_;
static TimerHandle_t reset_tmr_;
static BootState bState;
BootFrameHeader bFrameHeader;


/******************************************************************************/
/*!   \fn BaseType_t createBootTask( void )

      \brief
        This function creates the boot thread.
   
      \author
        Eric Landes, Joseph DiCarlantonio
*******************************************************************************/
BaseType_t createBootTask( QueueHandle_t bootMsgQueue )
{    
	bootMsgHandle_ = bootMsgQueue;
	
    // populate status message
    memset(&bState, 0, sizeof(bState));
    bState.status = BOOT_INIT;
      
    // bootloader upgrades not required
    bState.boot_version.major_rev 			= bootSwM;
    bState.boot_version.minor_rev  			= bootSwm;
	bState.boot_version.build_number		= bootSwe;
    
    bState.printer_app_version.major_rev 	= getAppPrVerMajor();
    bState.printer_app_version.minor_rev 	= getAppPrVerMinor();
    bState.printer_app_version.build_number = getAppPrVerEng();
	
	bState.weigher_app_version.major_rev 	= getAppWgVerMajor();
    bState.weigher_app_version.minor_rev 	= getAppWgVerMinor();
    bState.weigher_app_version.build_number = getAppWgVerEng();
	
    memset(&bFrameHeader, 0, sizeof(BootFrameHeader));

    bState.model = getProductId();
    
    if( !spi_flash_init() ) {
        PRINTF("createBootTask(): Failed to init spi flash\r\n");
    }
		
    /* create boot task thread */
    BaseType_t result = xTaskCreate(bootTask, "bootTask", configMINIMAL_STACK_SIZE, NULL, boot_task_PRIORITY, &bootTaskHandle_);

	/* boot ready timer */
	boot_ready_ = xTimerCreate( "boot ready", (TickType_t)BOOT_READY_SEND_RATE, pdTRUE, ( void * ) 0, bootReadyCallback );
	if( boot_ready_ != NULL ) {
      PRINTF("createBootTask(): Timer created!\r\n" );
    }

	if (!startBootReadyTimer()){
      PRINTF("createBootTask(): Failed to start timer!\r\n" );
    }

	/* reset timer, gets started once upgrade is complete and jump command is issued */
	reset_tmr_ = xTimerCreate( "reset timer", (TickType_t)pdMS_TO_TICKS(3000), pdTRUE, ( void * ) 0, resetTmrCallback );
	if( reset_tmr_ != NULL ) {
      PRINTF("createBootTask(): reset_tmr_ created!\r\n" );
    }


	return result; 
}

#if 0
/******************************************************************************/
/*!   \fn void assignBootMsgQueue( QueueHandle_t pQHandle )

      \brief
        This function assigns the transmit queue handle.
   
      \author
          Eric Landes
*******************************************************************************/
void assignBootMsgQueue( QueueHandle_t pQHandle )
{
    bootMsgHandle_ = pQHandle;
}
#endif

/******************************************************************************/
/*!   \fn static void bootTask( void *pvParameters )

      \brief
      This function is the bootloader run thread.

      \author
          Eric Landes
*******************************************************************************/
static void bootTask( void *pvParameters )
{
    PRINTF("bootTask(): Thread running...\r\n" );
	
    while( !suspend_ ) {

	BootMsg btMsg;
	int numMgs = uxQueueMessagesWaiting( bootMsgHandle_ );
	if( numMgs ) {
		if( xQueueReceive(bootMsgHandle_, &btMsg, 0 ) ) {

			handleBootloaderMsg( &btMsg );
		}
		else {
			PRINTF("BootTask(): Failed to get Boot message from queue!\r\n" );
		}
	}
	taskYIELD();

    }
	
    vTaskSuspend(NULL);
}

/******************************************************************************/
/*!   \fn void bootReadyCallback(TimerHandle_t boot_ready)

      \brief
        callback to send boot ready message to tell the backend we have come out
        of reset and are ready to begin
      \author
          Eric Landes
*******************************************************************************/
void bootReadyCallback(TimerHandle_t boot_ready){
    PRINTF("bootReadyCallback entered\r\n");
	static bool first_expiry = false;
	BootReadyMsg rdy;

	rdy.msgType 	= KpcBootReady;
	rdy.product_id 	= bState.model;

	if(first_expiry == false)
	{
		xTimerChangePeriod( boot_ready, (TickType_t)pdMS_TO_TICKS(5000), 100 );
		first_expiry = true;
	}
	
	sendBootReady(&rdy);	
}

void resetTmrCallback(TimerHandle_t tmr){
    PRINTF("resetTmrCallback entered\r\n");
    
	stopResetTmr();
	jumpToApp();
	
}

/******************************************************************************/
/*!   \fn bool stopBootReadyTimer(void)

      \brief
        stops the boot ready timer
      \author
          Eric Landes
*******************************************************************************/
bool stopBootReadyTimer(void){
    PRINTF("entered stopBootReadyTimer()");
    bool result = false;
    if( xTimerStop( boot_ready_, 0 ) == pdPASS ) {
        PRINTF("stopBootReadyTimer(): Timer stopped!\r\n" );
        result = true;
    }
    return result;
}


/******************************************************************************/
/*!   \fn bool stopBootReadyTimer(void)

      \brief
        starts the boot ready timer
      \author
          Eric Landes
*******************************************************************************/
bool startBootReadyTimer(void){
    bool result = false;
    if( xTimerStart( boot_ready_, 0 ) == pdPASS ) {
        PRINTF("startBootReadyTimer(): Timer started!\r\n" );
        result = true;
    }
    return result;
}

bool stopResetTmr(void){
    PRINTF("entered stopResetTmr()");
    bool result = false;
    if( xTimerStop( reset_tmr_, 0 ) == pdPASS ) {
        PRINTF("stopResetTmr(): Timer stopped!\r\n" );
        result = true;
    }
    return result;
}

bool startResetTmr(void){
    PRINTF("entered startResetTmr()");
    bool result = false;
    if( xTimerStart( reset_tmr_, 0 ) == pdPASS ) {
        PRINTF("startResetTmr(): Time started!\r\n" );
        result = true;
    }
    return result;
}
/******************************************************************************/
/*!   \fn static void handleBootloaderMsg( void *pvParameters )

      \brief
        handles messages for the bootloader

      \author
          Eric Landes, Joseph DiCarlantonio, Carlos Guzman
*******************************************************************************/
static void handleBootloaderMsg(BootMsg * btMsg)
{
    switch( btMsg->generic.function_code ) {
		
        case BOOT_REQ_STATUS:  
		{
            sendStateStatus();
            break;
		}
			
		case BOOT_REQ_BOOT_VERSION:
		{
			BootVerMsg msg;
			
			msg.msgType						= KpcBootVersion;
			msg.boot_version.major_rev		= bState.boot_version.major_rev;
			msg.boot_version.minor_rev 		= bState.boot_version.minor_rev;
			msg.boot_version.build_number 	= bState.boot_version.build_number;

			sendBootVersion(&msg);
			break;
		}
		
		case BOOT_REQ_APP_VERSION:
		{
			BootAppVerMsg msg;
			
			msg.msgType								= KpcBootAppVersion;
			msg.printer_app_version.major_rev		= bState.printer_app_version.major_rev;
			msg.printer_app_version.minor_rev 		= bState.printer_app_version.minor_rev;
			msg.printer_app_version.build_number 	= bState.printer_app_version.build_number;
			msg.weigher_app_version.major_rev 		= bState.weigher_app_version.major_rev;
			msg.weigher_app_version.minor_rev 		= bState.weigher_app_version.minor_rev;
			msg.weigher_app_version.build_number 	= bState.weigher_app_version.build_number;

			sendBootAppVersion(&msg);
			break;
		}
			
        case BOOT_CMD:
		{
            handleBootCmd( ( BootCmdMsg * )btMsg );
            break;
		}
			
		case BOOT_USB_BULK_PAYLOAD_RCV:
		{
			PRINTF("handleBootloaderMsg():	BOOT_USB_BULK_PAYLOAD_RCV\r\n", btMsg->generic.function_code);
			handleFrameData();
			break;
		}
		
        default:
            PRINTF("handleBootloaderMsg(): Unsupported Function Code %04x\r\n", btMsg->generic.function_code);
            break;

    }
	
}


/******************************************************************************/
/*!   \fn static void sendStateStatus()

      \brief
        sends the current state as a status message

      \author
          Eric Landes
*******************************************************************************/
static void sendStateStatus()
{
    BootStatusMsg bStatus;  
	bStatus.msgType		= KpcBootStatus;
    bStatus.model 		= bState.model;
    bStatus.status 		= bState.status;
    bStatus.fault 		= bState.fault;
	bStatus.fault_info 	= bState.fault_info;
    
	//call translator function to send status
    sendBootStatus(&bStatus);
}

/******************************************************************************/
/*!   \fn static void jumpToApp()

      \brief
        jumps from the application to the bootloader

      \author
          Eric Landes, Joseph DiCarlantonio
*******************************************************************************/
static void jumpToApp()
{    
    // Create the function call to the user application.
    // Static variables are needed since changed the stack pointer out from under the compiler
    // we need to ensure the values we are using are not stored on the previous stack
    static void (*farewellBootloader)(void) = 0;
    static uint32_t runApplicationAddress = 0;
    
    taskENTER_CRITICAL();

    __disable_irq();
    NVIC_ClearEnabledIRQs();
    NVIC_ClearAllPendingIRQs();
            
    /* deinit devices for app */
    DbgConsole_Deinit();
    //FLEXSPI_Deinit(FLEXSPI);
	
    //FLEXCAN_Deinit(CAN1);
    //ADC_Deinit(ADC1);
	
    //ADC_EnableDMA(ADC1, false);

    // Get application entry point from application vector table 
    unsigned long addr = APP_VECTOR_START;
    runApplicationAddress = *(uint32_t*)(addr + 0x04);
    farewellBootloader = (void (*)(void))runApplicationAddress; // entry point address is offset of 0x04 from the start of the vector table
    // Set the VTOR to the application vector table address.
    SCB->VTOR = (uint32_t)addr;

    // Set stack pointers to the application stack pointer.
    __set_CONTROL(0); // control register: change from PSP to MSP
    __set_MSP(APP_STACK_ADDR); // main stack pointer
    __set_PSP(APP_STACK_ADDR); // process stack poiner

    // Jump to the application.
    farewellBootloader();
}

/******************************************************************************/
/*!   \fn static void handleBootCmd(BootCmdMsg * msg)

      \brief
        handles boot cmd messages 

      \author
          Eric Landes
*******************************************************************************/
static void handleBootCmd(BootCmdMsg * msg)
{  
    switch( msg->boot_cmd ) {
        case JUMP:
		{
            /* detach usb connection and start reset timer that will call a jump to the APP  */
			USB_DeviceStop( usbMgr.deviceHandle );
			startResetTmr();
            break;
		}
        
		case INIT_UPGRADE:
		{
			initializeAppUpgrade();
            break;
		}
		
        case I_HEARD_YOU_NOW_SHUT_UP:
		{
      		stopBootReadyTimer();
			sendStateStatus();
      		break;
		}
		
        case VERIFY_APP:
		{
            bState.status = BOOT_VERIFY;

            if(!verifyApp()) {
                PRINTF("Could not verify app\r\n");
                bState.status = BOOT_FAULT;
                bState.fault = IMAGE_CHECKSUM_MISMATCH;
            } else {
                PRINTF("App was verified\r\n");
                bState.status = BOOT_VERIFY;                        
            }
			
            PRINTF("Sending state status\r\n");
            sendStateStatus();	
            break;
		}
        default:
            PRINTF("handleBootloaderMsg(): Unsupported Command %04x\r\n", (msg->boot_cmd));
            break;
    }
}

/******************************************************************************/
/*!   \fn static void initializeAppUpgrade()

      \brief
        erases application flash and prepares for application upgrade

      \author
          Eric Landes, Joseph DiCarlantonio
*******************************************************************************/
static void initializeAppUpgrade( void )
{
    if( !bFrameHeader.upgradeInProgress ) {
        bState.status = BOOT_READY;
        bState.fault = NOFAULT;
        bFrameHeader.currentFrame = 0;
        bFrameHeader.upgradeInProgress = true;
        bFrameHeader.currentAddr = APP_VECTOR_START;
        
        memset( sendBuffer, 0x00, sizeof( sendBuffer ) );

        __disable_irq();
        bool res = spi_flash_erase_app();
        if( !res ) {
            bState.status = BOOT_FAULT;
            bState.fault = ERASE_FAILURE_APP;
            bFrameHeader.upgradeInProgress = false;
        } 
        __enable_irq();

        sendStateStatus();
    }
}

/******************************************************************************/
/*!   \fn static void handleFrameData(BootFrameMsg * frame)

      \brief
        stores the given frame in data flash assumes initializeAppUpgrade() has
        been called 

      \author
          Eric Landes, Joseph DiCarlantonio
*******************************************************************************/
static void handleFrameData(/*BootFrameMsg * frame*/void)
{
    // with the fsl romapi, we need to write one page at a time 
    static int count = 0;
    static unsigned long bytesWritten = 0;
    
    
    if( bFrameHeader.upgradeInProgress ) {        		
		count ++;
		
		/*
		*	Note on Firmware Image:
		*	Currently the IAR tool outputs an Application firmware file in binary format.
		*	The binary file contains data to fill the entire size of the RT1024 Application region.
		*	Application region size = 0x3BFFFF(3932159)bytes. See linker file for more information.
		*	File is composed of Application Code + Unused space + vendor data(at the end of the file).
		*	Scale Back-end application will send the firmware file to us in 512 byte payloads via USB. 
		*	It will take 7679 = (3932159/512) payloads to completely tranfer the image. The entire upgrade
		*	process currently takes about 3mins.
		*
		*	We can optimize the upgrade process by reducing the size of the firmware file. The new file would
		*	leave out the unused space contents and just include the Application Code + vendor data. This would
		*	require changes to the Linker file, IAR tool build settings and the bootloader code.
		*
		*	At the time of this initial implementation, it was determined ecceptable, for the 3mins that it currently takes
		*	to perform a firmware upgrade. No optimization was required.   
		*/

	
#if 0		
		if(count >= 7680)
		{		   	  	
			for( int i = 0; i < 512;  ) {

				PRINTF( "%x ",imageBuffer[i++] );
				PRINTF( "%x ",imageBuffer[i++] );
				PRINTF( "%x ",imageBuffer[i++] );
				PRINTF( "%x ",imageBuffer[i++] );
				PRINTF( "%x ",imageBuffer[i++] );
				PRINTF( "%x ",imageBuffer[i++] );
				PRINTF( "%x ",imageBuffer[i++] );
				PRINTF( "%x ",imageBuffer[i++] );
				PRINTF( "%x ",imageBuffer[i++] );
				PRINTF( "%x ",imageBuffer[i++] );
				PRINTF( "%x ",imageBuffer[i++] );
				PRINTF( "%x ",imageBuffer[i++] );
				PRINTF( "%x ",imageBuffer[i++] );
				PRINTF( "%x ",imageBuffer[i++] );
				PRINTF( "%x ",imageBuffer[i++] );
				PRINTF( "%x \r\n",imageBuffer[i++] );	
				}
		}
#endif			
		/*
		*	payload size is 512, Flash driver programs 256(FLASH_PAGE_SIZE) 
			at a time. So we will perform two flash writes. 
		*/
		__disable_irq();
		if( spi_flash_program( bFrameHeader.currentAddr, (const uint32_t *)imageBuffer, FLASH_PAGE_SIZE ) ) {
			//count = 0;
			bFrameHeader.currentAddr += FLASH_PAGE_SIZE;
			bytesWritten += FLASH_PAGE_SIZE;
		} else {
			__enable_irq();
			bState.status = BOOT_FAULT;
			bState.fault = WRITE_FAILURE;
			sendStateStatus();
			PRINTF( "handleFrameData(): Could not write at %d\r\n", bFrameHeader.currentAddr );
			return;
		}
		
		//second write
		if( spi_flash_program( bFrameHeader.currentAddr, (const uint32_t *)&imageBuffer[256], FLASH_PAGE_SIZE ) ) {
			//count = 0;
			bFrameHeader.currentAddr += FLASH_PAGE_SIZE;
			bytesWritten += FLASH_PAGE_SIZE;
		} else {
			__enable_irq();
			bState.status = BOOT_FAULT;
			bState.fault = WRITE_FAILURE;
			sendStateStatus();
			PRINTF( "handleFrameData(): Could not write at %d\r\n", bFrameHeader.currentAddr );
			return;
		}
		__enable_irq();
	    
        // send ack 
		sendBootAck();	
    }
	
}

/******************************************************************************/
/*!   \fn bool verifyApp(void)

      \brief
        validates the application checksum and modelType 

      \author
          Eric Landes
*******************************************************************************/
bool verifyApp(void)
{
    uint32_t checksum = 0;
    uint32_t temp = 0;
    
    memcpy(&temp,(uint32_t *)APP_CHECKSUM_START, sizeof(temp));
    checksum = calculateChecksumBoot(APP_VECTOR_START, APP_VENDOR_END);
    
    if(checksum != temp) {
        bState.fault_info = temp;
        return false;
    }

    return true;
}

/******************************************************************************/
/*!   \fn uint32_t calculateChecksum(uint32_t start, uint32_t end)

      \brief
        calculates the arithmetic sum for the given addresses
      \author
          Eric Landes
*******************************************************************************/
//uint8_t calculateChecksumBoot(uint32_t start, uint32_t end)
uint32_t calculateChecksumBoot(uint32_t start, uint32_t end)
{ 
    uint32_t checksum = 0;
    uint8_t *address = 0;
    uint8_t *endAddress = 0;
    address = (uint8_t *)start;
    endAddress = (uint8_t *)end;
  
    // +1 because we want to include the end address
    while( address != ( endAddress + 1 ) ) { 
        checksum += (uint32_t)*address;
        address++;
    }

    return checksum;
}

