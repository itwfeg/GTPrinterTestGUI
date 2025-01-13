#include "developmentSettings.h"
#include "fsl_debug_console.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

/* Private Functions */
void printSelectedString(PrintMessageList stringIdentifier);

  /* The following assertion will fail if a service routine (ISR) for
             * an interrupt that has been assigned a priority above
             * configMAX_SYSCALL_INTERRUPT_PRIORITY calls an ISR safe FreeRTOS API
             * function.  ISR safe FreeRTOS API functions must *only* be called
             * from interrupts that have been assigned a priority at or below
             * configMAX_SYSCALL_INTERRUPT_PRIORITY.
             *
             * Numerically low interrupt priority numbers represent logically high
             * interrupt priorities, therefore the priority of the interrupt must
             * be set to a value equal to or numerically *higher* than
             * configMAX_SYSCALL_INTERRUPT_PRIORITY.
*/

//#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 2
#define PFISR_QSIZE 14

typedef struct {
    PrintMessageList items[PFISR_QSIZE];
    int front;
    int rear;
} pfISRQueueStuct;

pfISRQueueStuct pfISRQueue;

// Initialize the queue
void initQueue(void) {
    pfISRQueue.front = -1;
    pfISRQueue.rear = -1;
}

// Check if the queue is full
int isFull(void) {
    return pfISRQueue.rear == PFISR_QSIZE - 1;
}

// Check if the queue is empty
int isEmpty(void) {
    return pfISRQueue.front == -1;
}


// Add an element to the queue
void enqueue(PrintMessageList value) {
    if (isFull()) {
        return;
    }
    if (isEmpty()) {
        pfISRQueue.front = 0;
    }
    pfISRQueue.rear++;
    pfISRQueue.items[pfISRQueue.rear] = value;

}

// Remove an element from the queue
PrintMessageList dequeue(void) {
    if (isEmpty()) {
        return NO_PRINTF_MESSAGE;
    }
    PrintMessageList item = pfISRQueue.items[pfISRQueue.front];
    pfISRQueue.front++;
    if (pfISRQueue.front > pfISRQueue.rear) {
        pfISRQueue.front = pfISRQueue.rear = -1;
    }
    return item;
}

/*************************** PUBLIC FUNCTION *****************/
void queuePrintStringFromISR(PrintMessageList stringIdentifier)
{
   #ifdef PRINTFROMINTERRUPT
   enqueue(stringIdentifier);
   #endif   
}

void printFromInterrupt(void)
{
   #ifdef PRINTFROMINTERRUPT
  
   unsigned char i;
   for(i = 0; i<5;i++)  //Print five messages at a time. 
   {
      PrintMessageList msg2Print = dequeue();
      if (msg2Print != NO_PRINTF_MESSAGE) {
         printSelectedString(msg2Print);
      } 
      else
         i = 5; 
   }
   #endif
}

short copyOftightenTorqueOvershootAjustment;
short copyOfTightenTorqueAdjustment; 
unsigned short copyOfImmediateTuSpeedAdjustDueToHighTorque;
unsigned short copyOfTUSpeedAdjustDueToHighTorque;
void sendTorqeAdmustmentsToPrintf(short tightenAjust, unsigned short immPrintAdjust, unsigned short ltPrintAdjust, short tightenTorqOvershootAjustment)
{
   copyOftightenTorqueOvershootAjustment = tightenTorqOvershootAjustment;
   copyOfTightenTorqueAdjustment = tightenAjust;
   copyOfImmediateTuSpeedAdjustDueToHighTorque = immPrintAdjust;
   copyOfTUSpeedAdjustDueToHighTorque = ltPrintAdjust;
}



static unsigned short LLAverageTime = 0;
void sendLLAverageTimeToPrintf(unsigned short time)
{
   LLAverageTime = time;  
}

static tuControlMethodEnum previousControlState = INVALID;
static tuControlMethodEnum currentControlState = INVALID;
void sendControlStateToPrintf(unsigned short state)
{  
   currentControlState = (tuControlMethodEnum)state;  
   queueControlState();
}

void queueControlState(void)
{
   if(currentControlState != previousControlState) {
      previousControlState = currentControlState;
      if(currentControlState == HOLD_TIGHTEN_STEP_SPEED)
         enqueue(TU_CONTROL_HOLD_TIGHTEN_STEP_SPEED);
      else if(currentControlState == RAMP_TO_STATIC_STEP)
         enqueue(TU_CONTROL_RAMP_TO_STATIC_STEP);
      else if(currentControlState == STATIC_STEP)
         enqueue(TU_CONTROL_STATIC_STEP);
      else if(currentControlState == PID_CONTROL)
         enqueue(TU_CONTROL_PID_CONTROL);
      else if(currentControlState == NORMAL_TU_MOTOR_INTR_EXIT )
         enqueue(TU_CONTROL_NORMAL_TU_MOTOR_INTR_EXIT);
      else if(currentControlState == ABNORMAL_TU_MOTOR_INTR_EXIT )
         enqueue(TU_CONTROL_ABNORMAL_TU_MOTOR_INTR_EXIT);
      else
         enqueue(TU_CONTROL_UNKNOWN_STATE);        
   }
   
}

void printLLAverageTime(void)
{
   if(LLAverageTime != 0)
      PRINTF("LLAvgTime: %d, tightAdjust: %d, tightOvrSht: %d, printAdjust, imm: %d, lt: %d\r\n", LLAverageTime, copyOfTightenTorqueAdjustment, 
              copyOftightenTorqueOvershootAjustment, copyOfImmediateTuSpeedAdjustDueToHighTorque,copyOfTUSpeedAdjustDueToHighTorque);
   
   LLAverageTime = 0;
}

static unsigned short  copyOfStepsTakenTPHIntr = 0;
void sendStepsTakenTPHIntrToPrintf(unsigned short stepsTaken)
{
   copyOfStepsTakenTPHIntr = stepsTaken;
}

void printStepsTakenTPHIntr(void)
{
   if(copyOfStepsTakenTPHIntr != 0)
      PRINTF("StepsTaken: %d ,", copyOfStepsTakenTPHIntr);
   
   copyOfStepsTakenTPHIntr = 0;
}




void printSelectedString(PrintMessageList stringIdentifier)
{
   switch (stringIdentifier) {
    case    NO_PRINTF_MESSAGE:
      break;
    case    VSCOPE_INITIALIZED_MSG:
     // PRINTF("initializeVScope():INITIALIZED\r\n");
      break;
    case  VSCOPE_UNINITIALIZED_MSG:
     // PRINTF("batchPrintVScopeData():UNINITIALIZED\r\n");
      break;
    case VSCOPE_FULL_MSG:
      //PRINTF("vScopeRecordTakeUp():FULL\r\n");
      break;
    case VSCOPE_RECORDING_MSG:
     // PRINTF("enableVScope():RECORDING\r\n");
      break;
    case VSCOPE_STARTPRINTING_MSG:
     // PRINTF("startPrintVScope():START_PRINTING\r\n");
      break;
    case STEPTUMOTORINTR_TORQUE_OVERSHOOT:
      //PRINTF("stepTUMtrIntr(): Early Torq Overshoot\r\n");
      break; 
    case STEPTUMOTORINTR_HIGHTORQ_SLOWSTEP:
      //PRINTF("stepTUMtrIntr(): High Torq. Slow TU Step!\r\n");
      break;
    case STEPTUMOTORINTR_BAD_LAST_LABLEL_TIME:
      PRINTF("stepTUMtrIntr(): LastLabelTUStepTime Off\r\n");
      break; 
    case SWITCHED_TO_PID_DUE_TO_LOW_TORQUE:
      PRINTF("stepTUMtrIntr(): Switched to PID control due to low torque\r\n");
      break; 
    case TU_CONTROL_HOLD_TIGHTEN_STEP_SPEED:
      PRINTF("HOLD_TIGHTEN_STEP_SPEED\r\n");
      break;
    case  TU_CONTROL_RAMP_TO_STATIC_STEP:
      PRINTF("RAMP_TO_STATIC_STEP\r\n");
      break;
    case TU_CONTROL_STATIC_STEP:
      PRINTF("STATIC_STEP\r\n");
      break;
    case TU_CONTROL_PID_CONTROL:
      PRINTF("PID_CONTROL\r\n");
      break;
    case TU_CONTROL_NORMAL_TU_MOTOR_INTR_EXIT:
      PRINTF("NORMAL_TU_MOTOR_INTR_EXIT\r\n");
      break;
    case TU_CONTROL_ABNORMAL_TU_MOTOR_INTR_EXIT:
      PRINTF("ABNORMAL_TU_MOTOR_INTR_EXIT\r\n");
      break;
    case TU_CONTROL_UNKNOWN_STATE :
      PRINTF("UNKNOWN_STATE\r\n");
      break;
    default:
      PRINTF("printFromInterrupt(): error\r\n");
   }   
}


#if 0
// Define the queue handle
QueueHandle_t xQueue;


void createPrintFromInterruptQueue(void)
{
  #ifdef PRINTFROMINTERRUPT
  xQueue = xQueueCreate(20, sizeof(PrintMessageList));
  PRINTF("Print from Interrupt Queue created\r\n");
  #endif 
}


void queuePrintStringFromISR(PrintMessageList stringIdentifier)
{
   #ifdef PRINTFROMINTERRUPT
   BaseType_t xHigherPriorityTaskWasWoken;
   xQueueSendFromISR(xQueue, &stringIdentifier, &xHigherPriorityTaskWasWoken);
   #endif   
}

void printFromInterrupt(void)
{
   #ifdef PRINTFROMINTERRUPT
   PrintMessageList receivedStringID = NO_PRINTF_MESSAGE;
   
   if(xQueue == NULL)
      return;
   
   unsigned char i;
   for(i = 0; i<5;i++)  //Print five messages at a time. 
   {
      if (xQueueReceive(xQueue, &receivedStringID, 0) == pdPASS) {
         printSelectedString(receivedStringID);
      } 
      else
         i = 5; 
   }
   #endif
}

#endif