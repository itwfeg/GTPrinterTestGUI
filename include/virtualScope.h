#ifndef VIRTUAL_SCOPE_H
#define VIRTUAL_SCOPE_H


//#define VIRTUALSCOPE     //TFinkToDo!       //comment out this line to disable virtualScope

#define SCOPEBUFFERSIZE 450
void initializeVScopeTimer(void);
void vScopeRecordTakeUp(void);
void initializeVScope(void);
void startPrintVScope(void);
void enableVScope(void);
void printMaxMtrEnableToggleTime(void);
void vScopeRecordMtrEnblTime(void);
void vScopeRecordPHStep(void);
void batchPrintVScopeData(void);


enum virtualScopeState
{
   UNINITIALIZED,
   INITIALIZED,
   RECORDING,
   FULL,
   START_PRINTING,
   PRINTING
};

#endif  

