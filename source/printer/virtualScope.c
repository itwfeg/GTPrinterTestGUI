/********************************************************************************
* \name virtualScope.c
*
* \description 
* This module implements a "virtual oscilloscope" that timestamps several
* printer signals. When graphed in excel, the output resembles an oscilloscope
* trace.
*
* This module is designed for minimal bandwidth use. The data is printed is 
* printed in batches from the idleTask which runs at the lowest level interrupt.
*
* \author Tom Fink
*
* \date 8/29/24
********************************************************************************/
#include "takeupMotor.h"
#include "fsl_pit.h"
#include "virtualScope.h"
#include "threadManager.h"   //TFink - only here for "delay_uS". 
#include "developmentSettings.h"

#ifdef VIRTUALSCOPE
/* Takeup Motor and Tension */
unsigned int TUBufIndex = 0;
uint32_t TUTimeBuffer[SCOPEBUFFERSIZE];
unsigned short TUTensionBuffer[SCOPEBUFFERSIZE];

/* PH Motor */
uint32_t PHMotorStepBuffer[SCOPEBUFFERSIZE] = {0};
unsigned short LabelTakenBuffer[SCOPEBUFFERSIZE] = {0};
unsigned short PHMotorIndex = 0;

/* Motor Enable Toggle - enables both stepper motors */
uint32_t previousMtrEnblToggleTime = 0;
uint32_t currentMtrEnblToggleTime = 0;
uint32_t maxTimeBtwnMtrEnblToggle = 0;

enum virtualScopeState vScopeState = UNINITIALIZED;
#endif


/************** Initialization and Enable Functions *********************************/
/******************************************************************************/
/*!     \fn void initializeVScopeTimer(void)

        \brief configures Periodic Interrupt Timer channel 1 period. This function
         is only useful as an "add on" to initPitADC();

        \author
        Tom Fink
*******************************************************************************/
void initializeVScopeTimer(void)
{
    #ifdef VIRTUALSCOPE
    PIT_SetTimerPeriod( PIT, kPIT_Chnl_1, MSEC_TO_COUNT( 5000U, CLOCK_GetFreq( kCLOCK_PerClk ) ) );  
    #endif
}

/******************************************************************************/
/*!     \fn void initializeVScope(void)

        \brief initializes the Virtual Scope variables IF the VS is not currently
         outputing is data from the last print.

        \author
        Tom Fink
*******************************************************************************/
void initializeVScope(void)
{ 
   #ifdef VIRTUALSCOPE
   if(vScopeState == PRINTING) {
      PRINTF("\r\n\r\n!!!!!!!!Failed to initialize vScope. vScopeState = %d !!!!!!!\r\n\r\n", vScopeState);
      return;
   }  
   else
     vScopeState = UNINITIALIZED;
   
   unsigned int i = 0;

   for(i = 0; i<SCOPEBUFFERSIZE;  i++)
   {
      TUTimeBuffer[i] = 0;
      TUTensionBuffer[i] = 0;
      PHMotorStepBuffer[i] = 0; 
      LabelTakenBuffer[i] = 0;
   }
   
   TUBufIndex = 0;
   PHMotorIndex = 0;
   
   PIT_StopTimer( PIT, kPIT_Chnl_1 );  //This should reset the timer.
   PIT_StartTimer( PIT, kPIT_Chnl_1 );   
   currentMtrEnblToggleTime = PIT_GetCurrentTimerCount( PIT, kPIT_Chnl_1 );
   maxTimeBtwnMtrEnblToggle = 0;
        
   vScopeState = INITIALIZED;
   //PRINTF("initializeVScope(): vScopeState = INITIALIZED\r\n");
   queuePrintStringFromISR(VSCOPE_INITIALIZED_MSG);
   #endif
}

/******************************************************************************/
/*!     \fn void enableVScope(void)

        \brief enables the virtual scope.

        \author
        Tom Fink
*******************************************************************************/
void enableVScope(void)
{ 
   #ifdef VIRTUALSCOPE
   if(vScopeState != INITIALIZED) 
      initializeVScope();
        
   if(vScopeState == INITIALIZED) {
    vScopeState = RECORDING;
    //PRINTF("enableVScope(): RECORDING\r\n"); 
    queuePrintStringFromISR(VSCOPE_RECORDING_MSG);
   }
   #endif
}


/******************************************************************************/
/*!     \fn void vScopeRecordTakeUp(void)

        \brief records the value of the PIT channel one (a "timestamp"). Also
               records the value of the takeup tension sensor. Stops recording
               when the buffers are full.

        \author
        Tom Fink
*******************************************************************************/
void vScopeRecordTakeUp(void)
{
   #ifdef VIRTUALSCOPE  
   if(vScopeState == RECORDING)
   {
      if(TUBufIndex < SCOPEBUFFERSIZE) {  //stop recording when we reach the end of the buffer
         TUTimeBuffer[TUBufIndex] = PIT_GetCurrentTimerCount( PIT, kPIT_Chnl_1 ); //Takeup Motor step timestamp
         TUTensionBuffer[TUBufIndex] = getTakeUpTorque();  //Takeup Tension  
         TUBufIndex++;
      }
      
      /* Stop Recording once the TU Buffer is full */
      if(TUBufIndex >= SCOPEBUFFERSIZE-1) {
         vScopeState = FULL; 
         //PRINTF("vScopeRecordTakeUp(): vScopeState = FULL\r\n"); 
         queuePrintStringFromISR(VSCOPE_FULL_MSG);
      }
   } 
   #endif
}

/******************************************************************************/
/*!     \fn void vScopeRecordMtrEnblTime(void)

        \brief records the maximum time between the MOTOR_EN_PIN being toggled
         (assuming its called from powerOnMotors();)

        \author
        Tom Fink
*******************************************************************************/
void vScopeRecordMtrEnblTime(void)
{
   #ifdef VIRTUALSCOPE               
   if(vScopeState == RECORDING) {        
      previousMtrEnblToggleTime = currentMtrEnblToggleTime; 
      currentMtrEnblToggleTime  = PIT_GetCurrentTimerCount( PIT, kPIT_Chnl_1 );
      if((previousMtrEnblToggleTime-currentMtrEnblToggleTime) > maxTimeBtwnMtrEnblToggle)
         maxTimeBtwnMtrEnblToggle = previousMtrEnblToggleTime-currentMtrEnblToggleTime;        
   }
   #endif
}

/******************************************************************************/
/*!     \fn void vScopeRecordPHStep(void)

        \brief Records the time between PH steps until out of buffer.

        \author
        Tom Fink
*******************************************************************************/
void vScopeRecordPHStep(void)
{
   #ifdef VIRTUALSCOPE  
   if(vScopeState == RECORDING) {
      if(PHMotorIndex <= SCOPEBUFFERSIZE) {
         PHMotorStepBuffer[PHMotorIndex] = PIT_GetCurrentTimerCount( PIT, kPIT_Chnl_1 );
         LabelTakenBuffer[PHMotorIndex] = (unsigned short) getLabelTaken();  
         PHMotorIndex++;
      }
   }
    #endif
}
         
         
/******************************************************************************/
/*!     \fn void startPrintVScope(void)

        \brief this function initiates the printing of the Virtual Scope data by
         batchPrintVScopeData().

        \author
        Tom Fink
*******************************************************************************/
void startPrintVScope(void)
{
   #ifdef VIRTUALSCOPE
   if(vScopeState == RECORDING || vScopeState == FULL) {
      vScopeState = START_PRINTING;
      //PRINTF("startPrintVScope(): vScopeState = START_PRINTING\r\n");
      queuePrintStringFromISR(VSCOPE_STARTPRINTING_MSG);
   }  
   #endif
}

/******************************************************************************/
/*!     \fn void batchPrintVScopeData(void)

        \brief prints a small number of lines of vScope data. Keeps track of
         where it left off so multiple calls to this function will print 
         all the VS data. changes the VS state to UNINITIALIZED to indicate
         when all the data has been printing.

        \author
        Tom Fink
*******************************************************************************/
void batchPrintVScopeData(void)
{
   #ifdef VIRTUALSCOPE
   static unsigned short pVSData= 0;
   
   if(vScopeState == START_PRINTING) {
      vScopeState = PRINTING; 
      pVSData= 0;      
      PRINTF("Max time btwn Motor Enable Toggle = %dnS\r\n", maxTimeBtwnMtrEnblToggle*16);  
   }
   
   if(vScopeState != PRINTING)
      return;

   #define BATCH_SIZE 30
   unsigned short i = 0;
   for (i = 0; i < BATCH_SIZE; i++) {
      PRINTF("%d %d %d %d %d\r\n", pVSData, TUTimeBuffer[pVSData], TUTensionBuffer[pVSData], PHMotorStepBuffer[pVSData],LabelTakenBuffer[pVSData]);
      pVSData++;
      if(pVSData >= (SCOPEBUFFERSIZE-1)) {
         i = BATCH_SIZE;
         vScopeState = UNINITIALIZED;
         //PRINTF("batchPrintVScopeData(): vScopeState = UNINITIALIZED\r\n");
         queuePrintStringFromISR(VSCOPE_UNINITIALIZED_MSG);
      }
   }
   #endif
}


