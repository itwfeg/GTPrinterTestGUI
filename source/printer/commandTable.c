#include "commandTable.h"
#include "sensors.h"
#include "printEngine.h"
#include "threadManager.h"
#include "fsl_debug_console.h"


void initializeCmdSequence( CMDId id, PrStatusInfo *pStatus);
void setOperation( unsigned char operation, PrStatusInfo *pStatus );
void setNextOperation( PrStatusInfo *pStatus );
void skipNextOperation( PrStatusInfo *pStatus );
void jumpToOperation( PrStatusInfo *pStatus, unsigned char index );
  
/*******************************************************************************/
/**************************** basic printer operations *************************/
/*******************************************************************************/

static const IdleOperation      idle_                   = { IDLE_DIRECTIVE };
static const PrintOperation     printLabel_             = { PRINT_DIRECTIVE, INDIRECT_DATA, PRINT_DATA, HEAD_FIRST };
static const PrintOperation     printDotWear            = { HEAD_TEST_DIRECTIVE, INDIRECT_DATA, PRINT_DATA, HEAD_FIRST };
static const StepUntilOperation syncLabel_              = { STEP_UNTIL_DIRECTIVE, DIRECT_DATA, 2030, BITS_EQUAL,
                                                            MOTOR_FORWARD | SYNCHRONIZED, MOTOR_FORWARD | SYNCHRONIZED };
static const StepUntilOperation findLabel_              = { STEP_UNTIL_DIRECTIVE, DIRECT_DATA, 3000, BITS_EQUAL, LABEL_TAKEN, 0 };
static const StepOperation      advanceLabel_           = { STEP_DIRECTIVE, INDIRECT_DATA, ADVANCE_DATA };
static const StepOperation      advanceVirtualLabel_    = { STEP_DIRECTIVE, DIRECT_DATA, 290 };
static const StepOperation      advanceShootVirtualLabel_ = { STEP_DIRECTIVE, DIRECT_DATA, 160 };
static const StepOperation      retractVirtualLabel_    = { STEP_DIRECTIVE, DIRECT_DATA, -260 };
static const StepOperation      retractShootVirtualLabel_ = { STEP_DIRECTIVE, DIRECT_DATA, -203 };
static const StepOperation      advanceVirtLabel_       = { STEP_DIRECTIVE, DIRECT_DATA, 75 };
static const StepOperation      ejectShootLabel_        = { STEP_DIRECTIVE, DIRECT_DATA, 460 }; /*609 */
static const StepOperation      pSyncShoot_             = { STEP_DIRECTIVE, DIRECT_DATA, 75 };  /* 25 good for die cut labels */ /* 51 */
static const StepOperation      tighten_                = { STEP_TAKEUP_TIGHTEN, DIRECT_DATA, 3000 }; /* tighten 3" or torque limit */

static const StepOperation      cleanEject_     = { STEP_DIRECTIVE, DIRECT_DATA, 406 };         /* 2" forward */
static const StepOperation      cleanExpel_     = { STEP_DIRECTIVE, DIRECT_DATA, 3045 };        /* 15" forward */
static const StepOperation      retractLabel_   = { STEP_DIRECTIVE, INDIRECT_DATA, RETRACT_DATA };
static const WaitUntilOperation takeLabel_      = { WAIT_UNTIL_DIRECTIVE, BITS_EQUAL, LABEL_TAKEN, LABEL_TAKEN };
static const WaitUntilOperation takeLabelSzing_ = { WAIT_UNTIL_SIZING,BITS_EQUAL, LABEL_TAKEN, LABEL_TAKEN };
static const WaitOperation      waitSize        = { WAIT_DIRECTIVE, DIRECT_DATA, 150 };
static const WaitOperation      waitCut_        = { WAIT_DIRECTIVE, DIRECT_DATA, 300 };                         /* sof-5061 */
static const WaitOperation      waitClean_      = { WAIT_DIRECTIVE, DIRECT_DATA, 600 };
static const TestOperation      checkStock_     = { TEST_DIRECTIVE, BITS_NOT_EQUAL, OUT_OF_MEDIA,   
                                                    OUT_OF_MEDIA, 0, 0, CONTINUE_EXECUTION,
                                                    CONTINUE_EXECUTION }; /* ABORT_EXECUTION */

static const StepOperation      testForCont_    = { TEST_FOR_CONTINUOUS, INDIRECT_DATA, EJECT_DATA };
static const StepOperation      testForSync_    = { TEST_FOR_SYNC, DIRECT_DATA, 100 }; 
static const StepOperation      testForLabel_   = { TEST_FOR_LABEL, DIRECT_DATA, 150 };
static const StepOperation      cleanBackup_    = { STEP_DIRECTIVE, DIRECT_DATA, -406 };      /* 2" backward */         

static const TestOperation      missingLabelTest_ = { TEST_DIRECTIVE, BITS_EQUAL, LABEL_TAKEN, LABEL_TAKEN,
                                                      MISSING_LABEL, 0, CONTINUE_EXECUTION, CONTINUE_EXECUTION };
static const TestOperation      jammedLabelTest_  = { TEST_DIRECTIVE, BITS_EQUAL, MOTOR_FORWARD | SYNC_BAR | SYNC_BAR_EDGE,
                                                     MOTOR_FORWARD | SYNC_BAR | SYNC_BAR_EDGE, 0, JAMMED_LABEL,
                                                     CONTINUE_EXECUTION, CONTINUE_EXECUTION };  /* ABORT_EXECUTION */
static const CounterOperation   resetCounter_   = { COUNTER_DIRECTIVE, RESET_COUNTER };
static const CounterOperation   enableCounter_  = { COUNTER_DIRECTIVE, ENABLE_COUNTER };
static const CounterOperation   disableCounter_ = { COUNTER_DIRECTIVE, DISABLE_COUNTER };
static const StatusOperation    setTake_        = { STATUS_DIRECTIVE, SET_BITS, TAKE_LABEL };
static const StatusOperation    clearTake_      = { STATUS_DIRECTIVE, CLEAR_BITS, TAKE_LABEL };
static const GenericOperation   cutLabel_       = { CUT_DIRECTIVE };
static const StepUntilOperation findShootGap_   = { STEP_GAP_DIRECTIVE, DIRECT_DATA, 2436, BITS_EQUAL,                
                                                    MOTOR_FORWARD | SYNCHRONIZED, MOTOR_FORWARD | SYNCHRONIZED };       /*3000*/
static const StepUntilOperation findShootEdge_  = { STEP_EDGE_DIRECTIVE, DIRECT_DATA, 3000, BITS_NOT_EQUAL, MOTOR_FORWARD | SYNCHRONIZED };  /*406*/

static const StepUntilOperation paperTakeup_    = { STEP_TAKEUP_DIRECTIVE, DIRECT_DATA, 2436, BITS_EQUAL,                
                                                    MOTOR_FORWARD | SYNCHRONIZED, MOTOR_FORWARD | SYNCHRONIZED };    
static const StepUntilOperation glDetection_    = { DETECTION_STEP_UNTIL, DIRECT_DATA, 2030, BITS_EQUAL,
                                                    MOTOR_FORWARD | SYNCHRONIZED, MOTOR_FORWARD | SYNCHRONIZED };


#if CUTTER_LIFE_TEST
static const StepOperation      ejectLabel_     = { STEP_DIRECTIVE, DIRECT_DATA, 506 };
#else
    #if FSSS_DRIVETRAIN_TEST
    static const StepOperation      ejectLabel_     = { STEP_DIRECTIVE, DIRECT_DATA, 609 };     /* 3" of label */
    #else
    static const StepOperation      ejectLabel_     = { STEP_DIRECTIVE, DIRECT_DATA, 25 };
    #endif
#endif
    
#if CUTTER_LIFE_TEST
static const StepOperation      backup_         = { STEP_DIRECTIVE, DIRECT_DATA, -150 };
#else
static const StepOperation      backup_         = { STEP_DIRECTIVE, DIRECT_DATA, -101 };        //= { STEP_DIRECTIVE, INDIRECT_DATA, BACKUP_DATA };
#endif
#if FSSS_DRIVETRAIN_TEST
static const StepOperation      backupShoot_      = { STEP_DIRECTIVE, DIRECT_DATA, -609 };      
#else 
static const StepOperation      backupShoot_      = { STEP_DIRECTIVE, DIRECT_DATA, -205 };      /* { STEP_DIRECTIVE, DIRECT_DATA, -100 }; */ 
#endif
 

/*******************************************************************************/
/*******************************************************************************/
/*******************************************************************************/




/*******************************************************************************/
/****************************** teach tables ***********************************/
/*******************************************************************************/

/* label sizing: drive to the label gap, 
   drive to edge of label, 
   test if sync bar, 
   backup if not sync, 
   reset counter, drive to gap */ 
/*static CmdOp *gSizing[15]     = { (CmdOp *)&tighten_, (CmdOp *)&findShootGap_, (CmdOp *)&waitSize, (CmdOp *)&findShootEdge_, 
                                  (CmdOp *)&resetCounter_, (CmdOp *)&testForSync_, (CmdOp *)&resetCounter_,
                                  (CmdOp *)&waitSize,(CmdOp *)&findShootGap_,  
                                  (CmdOp *)&disableCounter_, (CmdOp *)&checkStock_, (CmdOp *)&backup_, (CmdOp *)&tighten_, (CmdOp *)&idle_, 0 };*/
   
static CmdOp *gSizing[15]     = { (CmdOp *)&findShootGap_, (CmdOp *)&disableCounter_, (CmdOp *)&idle_, 0 };

/* removed (CmdOp *)&jammedLabelTest_,
static CmdOp *gPrint[9]      = { (CmdOp *)&printLabel_, (CmdOp *)&pSyncShoot_, (CmdOp *)&testForCont_,
                                  (CmdOp *)&missingLabelTest_, (CmdOp *)&setTake_,    
                                  (CmdOp *)&takeLabel_, (CmdOp *)&clearTake_, (CmdOp *)&idle_, 0 }; */

static CmdOp *gPrint[9]      = { (CmdOp *)&printLabel_, /* (CmdOp *)&pSyncShoot_, (CmdOp *)&testForCont_,*/
                                 /*(CmdOp *)&tighten_,*/ (CmdOp *)&setTake_, (CmdOp *)&takeLabel_, (CmdOp *)&clearTake_,
                                 (CmdOp *)&idle_, 0 };

static CmdOp *findNoLabel[8] = { /*(CmdOp *)&findLabel_, (CmdOp *)&checkStock_, 
                                        (CmdOp *)&missingLabelTest_, (CmdOp *)&syncLabel_,
                                        (CmdOp *)&jammedLabelTest_, (CmdOp *)&checkStock_,*/
                                        (CmdOp *)&idle_, 0 };           

/*******************************************************************************/
/******************************** backend tables *******************************/
/*******************************************************************************/
#if 0
	listingPrintHeelCut.push_back(&startLabelOp);           PrinterTeachTable::startLabelOp(DirectCommandData, (PrinterIndirection) 11);
	listingPrintHeelCut.push_back(&reversePrintLabelOp);    PrinterTeachTable::reversePrintLabelOp(IndirectCommandData, PrRam0, HeelFirst);
	listingPrintHeelCut.push_back(&ejectLabelOp);           PrinterTeachTable::ejectLabelOp(IndirectCommandData, PrConfigWord3);
	listingPrintHeelCut.push_back(&cutterEjectLabelOp);     PrinterTeachTable::cutterEjectLabelOp(IndirectCommandData, PrConfigWord5);
	listingPrintHeelCut.push_back(&cutOp);                  PrinterTeachTable::cutOp(CutDirective);
	listingPrintHeelCut.push_back(&retractLabelOp);         PrinterTeachTable::retractLabelOp(IndirectCommandData, PrRam2);
	listingPrintHeelCut.push_back(&idleOp);

	dieCutShootPrintHeel.push_back(&reversePrintLabelOp);
	dieCutShootPrintHeel.push_back(&ejectShootOp);
	dieCutShootPrintHeel.push_back(&missingLabelTestOp);
	dieCutShootPrintHeel.push_back(&jammedLabelTestOp);
	dieCutShootPrintHeel.push_back(&setTakingOp);

	// See createDieCutPrintHead() for explanation
	if( pConfig_ && !pConfig_->isOpModeStreamingLabelTestEnabled())
	{
		dieCutShootPrintHeel.push_back(&takeLabelOp);
	}

	dieCutShootPrintHeel.push_back(&clearTakingOp);
	dieCutShootPrintHeel.push_back(&idleOp);



#endif
/*******************************************************************************/
/*******************************************************************************/
/*******************************************************************************/

/* find command needs to backup the label to park line 1 under the print head. 
static CmdOp *gFind[3]        = { (CmdOp *)&ejectShootLabel_, (CmdOp *)&idle_, 0 };
*/
static CmdOp *gFind[3]        = { (CmdOp *)&backupShoot_, (CmdOp *)&idle_, 0 };

static CmdOp *ssbackup[3]     = { (CmdOp *)&backup_, (CmdOp *)&idle_, 0 };
static CmdOp *fsbackup[3]     = { (CmdOp *)&backupShoot_, (CmdOp *)&idle_, 0 };
static CmdOp *gAdvance[3]     = { (CmdOp *)&advanceLabel_, (CmdOp *)&idle_, 0 };
static CmdOp *gEject[3]       = {(CmdOp *)&ejectLabel_ ,(CmdOp *)&idle_, 0 };           
static CmdOp *gSync[2]        = { (CmdOp *)&idle_, 0 };           

static CmdOp *ssCutRetract[4] = { (CmdOp *)&cutLabel_, (CmdOp *)&retractLabel_, (CmdOp *)&idle_, 0 };
static CmdOp *fsCutRetract[5] = { (CmdOp *)&cutLabel_, (CmdOp *)&waitCut_, (CmdOp *)&retractLabel_, (CmdOp *)&idle_, 0 };
static CmdOp *gHeadTest[3]    = { (CmdOp *)&printDotWear, (CmdOp *)&idle_, 0 };
static CmdOp *testGap[3]      = { (CmdOp *)&findShootGap_, (CmdOp *)&idle_, 0 };
static CmdOp *testEdge[3]     = { (CmdOp *)&findShootEdge_, (CmdOp *)&idle_, 0 };
static CmdOp *cleanHead[5]    = { (CmdOp *)&cleanEject_, (CmdOp *)&waitClean_, (CmdOp *)&cleanBackup_, (CmdOp *)&idle_, 0 };
static CmdOp *cleanExpel[3]   = { (CmdOp *)&cleanExpel_, (CmdOp *)&idle_, 0 };

static CmdOp *ssCutRetractVirt[9] = { (CmdOp *)&advanceVirtualLabel_, (CmdOp *)&cutLabel_, (CmdOp *)&retractVirtualLabel_,
                                      (CmdOp *)&advanceVirtLabel_,    (CmdOp *)&setTake_,  (CmdOp *)&takeLabel_, 
                                      (CmdOp *)&clearTake_,           (CmdOp *)&idle_,         0 };

static CmdOp *fsCutRetractVirt[9] = { (CmdOp *)&advanceShootVirtualLabel_, (CmdOp *)&cutLabel_, (CmdOp *)&retractShootVirtualLabel_, 
                                      (CmdOp *)&advanceVirtLabel_,    (CmdOp *)&setTake_,  (CmdOp *)&takeLabel_, 
                                      (CmdOp *)&clearTake_,           (CmdOp *)&idle_,         0 };

static CmdOp *gPaperTakeup[3]   = { (CmdOp *)&paperTakeup_, (CmdOp *)&idle_, 0 };

/*******************************************************************************/
/*******************************************************************************/
/*******************************************************************************/

unsigned short ram_[ MAX_RAM_LOCATIONS + 1 ] = {0};
CmdTable cmdTable[ _TBL_MAX ];
static CmdOp *pCmdList_ = NULL;


extern Pr_Config config_;
extern PrintEngine *getPrintEngine( void );




/******************************************************************************/
/*!   \fn void initializeTable( CmdTable *pTable )

      \brief
        This function initializes the command tables


      \author
          Aaron Swift
*******************************************************************************/
void initializeTable( void )
{
    /* clear the command table */
    memset( &cmdTable[0], 0, sizeof(CmdTable) );
    
    setTableDefaultCmds();     
}

/******************************************************************************/
/*!   \fn void copyTable( CmdOp *table, CmdOp **operations )

      \brief
        This function copies command operations to a teach table.


      \author
          Aaron Swift
*******************************************************************************/
void copyTable( CmdOp *table, CmdOp **operations )
{
    while (*operations != NULL)
    {
        memcpy((unsigned char *) table++, 
                    (unsigned char *) *operations++, 
                    (unsigned int) sizeof (CmdOp));
    }    
} 


/******************************************************************************/
/*!   \fn void setTableDefaultCmds( void )

      \brief
        This function sets the default teach tables.


      \author
          Aaron Swift
*******************************************************************************/
void setTableDefaultCmds( void )
{
    copyTable( &(cmdTable[1].oper[0]), findNoLabel );    
     /* if not a freestanding scale */
    if( getMyModel() != RT_GLOBAL_FSS ) {
        copyTable( &(cmdTable[2].oper[0]), ssbackup );
    } else {
        copyTable( &(cmdTable[2].oper[0]), fsbackup );
    }    
    
    copyTable( &(cmdTable[3].oper[0]), gAdvance );    
    copyTable( &(cmdTable[4].oper[0]), gPrint ); 
    copyTable( &(cmdTable[5].oper[0]), gEject ); 
    copyTable( &(cmdTable[6].oper[0]), gSync ); 
    copyTable( &(cmdTable[7].oper[0]), gFind );     
    copyTable( &(cmdTable[8].oper[0]), gSizing ); 
    
    if( getMyModel() != RT_GLOBAL_FSS ) {      
        copyTable( &(cmdTable[9].oper[0]), ssCutRetract );
    } else {
        copyTable( &(cmdTable[9].oper[0]), fsCutRetract );
    }
    
    copyTable( &(cmdTable[10].oper[0]), gHeadTest );
    
     /* if not a freestanding scale */
    if( getMyModel() != RT_GLOBAL_FSS ) {      
        copyTable( &(cmdTable[11].oper[0]),ssCutRetractVirt);
    } else {
        copyTable( &(cmdTable[11].oper[0]),fsCutRetractVirt);
    }    
}

void setTableTestCmds( TestTables test )
{
    /* use reserved table entry for test commands */
    if( test == _TESTGAP ) {
        copyTable( &(cmdTable[0].oper[0]), testGap );
    } else if( test == _TESTEDGE ) {
        copyTable( &(cmdTable[0].oper[0]), testEdge );
    } else if( test == _TESTCLEANING ) {
        copyTable( &(cmdTable[0].oper[0]), cleanHead ); 
    } else if( test == _TESTCLEANINGEXPEL ) {
        copyTable( &(cmdTable[0].oper[0]), cleanExpel );        
    } else if( test == _TESTPAPERTAKEUP ) {   
        copyTable( &(cmdTable[0].oper[0]), gPaperTakeup );
    } else {
        PRINTF("setTableTestCmds(): Unknown test teach table command!\r\n" ); 
    }
    
}

/******************************************************************************/
/*!   \fn void buildCmdTable( CMDId id, UINT numEntries, CmdOp *pOper )

      \brief
        Builds a command table.

      \author
          Aaron Swift
*******************************************************************************/
void buildCmdTable( CMDId id, unsigned char index, CmdOp *pOper )
{
    if( pOper != NULL ) {
        cmdTable[id].oper[index].generic.directive = pOper->generic.directive;
        cmdTable[id].oper[index].generic.d0 = pOper->generic.d0;
        cmdTable[id].oper[index].generic.d1 = pOper->generic.d1;
        cmdTable[id].oper[index].generic.d2 = pOper->generic.d2; 
        cmdTable[id].oper[index].generic.d3 = pOper->generic.d3; 
        cmdTable[id].oper[index].generic.d4 = pOper->generic.d4; 
        cmdTable[id].oper[index].generic.d5 = pOper->generic.d5; 
        cmdTable[id].oper[index].generic.d6 = pOper->generic.d6;         
    }
}

/******************************************************************************/
/*!   \fn initializeCmdSequence( CMDId id, PrStatusInfo *pStatus)

      \brief
        This function set the print engine operation directive.

      \author
          Aaron Swift
*******************************************************************************/
void initializeCmdSequence( CMDId id, PrStatusInfo *pStatus)
{
    if( id < _TBL_MAX ) {
        pStatus->command = id;
        pCmdList_ = cmdTable[(int)id].oper;
        if( pCmdList_ != NULL ) {            
            getPrintEngine()->currentCmd.generic.d0 = pCmdList_->generic.d0;
            getPrintEngine()->currentCmd.generic.d1 = pCmdList_->generic.d1;
            getPrintEngine()->currentCmd.generic.d2 = pCmdList_->generic.d2;
            getPrintEngine()->currentCmd.generic.d3 = pCmdList_->generic.d3;
            getPrintEngine()->currentCmd.generic.d4 = pCmdList_->generic.d4;
            getPrintEngine()->currentCmd.generic.d5 = pCmdList_->generic.d5;
            getPrintEngine()->currentCmd.generic.d6 = pCmdList_->generic.d6;
            getPrintEngine()->currentCmd.generic.d7 = pCmdList_->generic.d7;
            getPrintEngine()->currentCmd.generic.d8 = pCmdList_->generic.d8;
            getPrintEngine()->currentCmd.generic.d9 = pCmdList_->generic.d9;
            getPrintEngine()->currentCmd.generic.d10 = pCmdList_->generic.d10;

            pStatus->state = ENGINE_SWITCHING; 
            getPrintEngine()->currentCmd.generic.directive = pCmdList_->generic.directive;            
        } else { 
          PRINTF("initializeCmdSequence(): Warning: command list is null!\r\n" ); 
        }
    }
  
}

/******************************************************************************/
/*!   \fn void setOperation( unsigned char operation, PrStatusInfo *pStatus )

      \brief
        This function set the print engine operation directive.

      \author
          Aaron Swift
*******************************************************************************/
void setOperation( unsigned char operation, PrStatusInfo *pStatus )
{
    PrintEngine *pEngine = getPrintEngine();
    if( pEngine != NULL ) {
        pEngine->currentCmd.generic.directive =  operation;
        pStatus->state = ENGINE_SWITCHING;  
    } else {
        PRINTF("setOperation(): pEngine is null!\r\n" ); 
    }
}

/******************************************************************************/
/*!   \fn void setNextOperation( PrStatusInfo *pStatus )

      \brief
        This function sets the print engine operation directive to the next 
        directive in the command list.

      \author
          Aaron Swift
*******************************************************************************/
void setNextOperation( PrStatusInfo *pStatus )
{
    pCmdList_++;
    
    //PRINTF("setNextOperation(): %d\r\n", pCmdList_->generic.directive );
    
    getPrintEngine()->currentCmd.generic.directive = pCmdList_->generic.directive;
    getPrintEngine()->currentCmd.generic.d0 = pCmdList_->generic.d0;
    getPrintEngine()->currentCmd.generic.d1 = pCmdList_->generic.d1;
    getPrintEngine()->currentCmd.generic.d2 = pCmdList_->generic.d2;
    getPrintEngine()->currentCmd.generic.d3 = pCmdList_->generic.d3;
    getPrintEngine()->currentCmd.generic.d4 = pCmdList_->generic.d4;
    getPrintEngine()->currentCmd.generic.d5 = pCmdList_->generic.d5;
    getPrintEngine()->currentCmd.generic.d6 = pCmdList_->generic.d6;
    pStatus->state = ENGINE_SWITCHING; 
}

/******************************************************************************/
/*!   \fn void skipNextOperation( PrStatusInfo *pStatus )

      \brief
        This function skips the next operation and sets the print engine 
        operation directive to the next directive in the command list.

      \author
          Aaron Swift
*******************************************************************************/
void skipNextOperation( PrStatusInfo *pStatus )
{
    pCmdList_++;
    pCmdList_++;
    
    //PRINTF("skipNextOperation(): %d\r\n", pCmdList_->generic.directive ); 
    
    getPrintEngine()->currentCmd.generic.directive = pCmdList_->generic.directive;
    getPrintEngine()->currentCmd.generic.d0 = pCmdList_->generic.d0;
    getPrintEngine()->currentCmd.generic.d1 = pCmdList_->generic.d1;
    getPrintEngine()->currentCmd.generic.d2 = pCmdList_->generic.d2;
    getPrintEngine()->currentCmd.generic.d3 = pCmdList_->generic.d3;
    getPrintEngine()->currentCmd.generic.d4 = pCmdList_->generic.d4;
    getPrintEngine()->currentCmd.generic.d5 = pCmdList_->generic.d5;
    getPrintEngine()->currentCmd.generic.d6 = pCmdList_->generic.d6;
    pStatus->state = ENGINE_SWITCHING; 
}

/******************************************************************************/
/*!   \fn void jumpToOperation( PrStatusInfo *pStatus, unsigned char index )

      \brief
        This function sets the print engine operation directive to the 
        indexed directive in the command list. SOF-5183

      \author
          Aaron Swift
*******************************************************************************/
void jumpToOperation( PrStatusInfo *pStatus, unsigned char index )
{   /* need a positive jump */
    if( ( index > 0 ) && ( index < sizeof( gSizing ) ) ) {
        for( int i = 0; i < index; i++ ) {
            pCmdList_++;
        }
            
        PRINTF("jumpToOperation(): %d\r\n", pCmdList_->generic.directive ); 
        
        getPrintEngine()->currentCmd.generic.directive = pCmdList_->generic.directive;
        getPrintEngine()->currentCmd.generic.d0 = pCmdList_->generic.d0;
        getPrintEngine()->currentCmd.generic.d1 = pCmdList_->generic.d1;
        getPrintEngine()->currentCmd.generic.d2 = pCmdList_->generic.d2;
        getPrintEngine()->currentCmd.generic.d3 = pCmdList_->generic.d3;
        getPrintEngine()->currentCmd.generic.d4 = pCmdList_->generic.d4;
        getPrintEngine()->currentCmd.generic.d5 = pCmdList_->generic.d5;
        getPrintEngine()->currentCmd.generic.d6 = pCmdList_->generic.d6;
        pStatus->state = ENGINE_SWITCHING;     
    }
}


/******************************************************************************/
/*!   \fn int getIndirectData( CMD_DATA_IDS id )

      \brief
        
      \author
          Aaron Swift
*******************************************************************************/
int getIndirectData( CMD_DATA_IDS id )
{
    int val;
    switch( id )
    {
        case CONFIG_0:
            val = config_.peel_position;
            break;
        case CONFIG_3:
            val = config_.expel_position;
            break;
        case CONFIG_4:
            val = config_.retract_position;
            break;
        case CONFIG_1:       
        case CONFIG_2:
        case CONFIG_5:
        case CONFIG_6:
        case CONFIG_7:
        case CONFIG_8:
        case CONFIG_9:
        case CONFIG_10:
        case CONFIG_11:
        case CONFIG_12:
        case CONFIG_13:
            PRINTF("getIndirectData(): config value not supported %d\r\n", id ); 
            break;
        case CONFIG_14:
            val = config_.media_sensor_type;
            break;
        case CONFIG_15:
            val = config_.sla_eject_line;
            break;

      case RAM_0:
            val = ram_[ RAM_0 - MAX_RAM_LOCATIONS ];
            break;
        case RAM_1:
            val = ram_[ RAM_1 - MAX_RAM_LOCATIONS ];
            break;
        case RAM_2:
            val = ram_[ RAM_2 - MAX_RAM_LOCATIONS ];
            break;
        case RAM_3:
            val = ram_[ RAM_3 - MAX_RAM_LOCATIONS ];
            break;
        case RAM_4:
            val = ram_[ RAM_4 - MAX_RAM_LOCATIONS ];
            break;
        case RAM_5:
            val = ram_[ RAM_5 - MAX_RAM_LOCATIONS ];
            break;
        case RAM_6:
            val = ram_[ RAM_6 - MAX_RAM_LOCATIONS ];
            break;
        case RAM_7:
            val = ram_[ RAM_7 - MAX_RAM_LOCATIONS ];
            break;
        case RAM_8:
            val = ram_[ RAM_8 - MAX_RAM_LOCATIONS ];
            break;
        case RAM_9:
            val = ram_[ RAM_9 - MAX_RAM_LOCATIONS ];
            break;
        case RAM_10:
            val = ram_[ RAM_10 - MAX_RAM_LOCATIONS ];
            break;
        case RAM_11:
            val = ram_[ RAM_11 - MAX_RAM_LOCATIONS ];
            break;
        case RAM_12:
            val = ram_[ RAM_12 - MAX_RAM_LOCATIONS ];
            break;
        case RAM_13:
            val = ram_[ RAM_13 - MAX_RAM_LOCATIONS ];
            break;
        case RAM_14:
            val = ram_[ RAM_14 - MAX_RAM_LOCATIONS ];
            break;
        case RAM_15:
            val = ram_[ RAM_15 - MAX_RAM_LOCATIONS ];
            break;
        default:
            val = 0;
            break;
    }   
    return (val);
}

/******************************************************************************/
/*!   \fn void setIndirectData( CMD_DATA_IDS id, INT val )

      \brief
        
      \author
          Aaron Swift
*******************************************************************************/
void setIndirectData( CMD_DATA_IDS id, int val )
{

    switch( id )
    {
        case RAM_0:
            ram_[ RAM_0 - MAX_RAM_LOCATIONS ] = val;
            break;
        case RAM_1:
            ram_[ RAM_1 - MAX_RAM_LOCATIONS ] = val;
            break;
        case RAM_2:
            ram_[ RAM_2 - MAX_RAM_LOCATIONS ] = val;
            break;
        case RAM_3:
            ram_[ RAM_3 - MAX_RAM_LOCATIONS ] = val;
            break;
        case RAM_4:
            ram_[ RAM_4 - MAX_RAM_LOCATIONS ] = val;
            break;
        case RAM_5:
            ram_[ RAM_5 - MAX_RAM_LOCATIONS ] = val;
            break;
        case RAM_6:
            ram_[ RAM_6 - MAX_RAM_LOCATIONS ] = val;
            break;
        case RAM_7:
            ram_[ RAM_7 - MAX_RAM_LOCATIONS ] = val;
            break;
        case RAM_8:
            ram_[ RAM_8 - MAX_RAM_LOCATIONS ] = val;
            break;
        case RAM_9:
            ram_[ RAM_9 - MAX_RAM_LOCATIONS ] = val;
            break;
        case RAM_10:
            ram_[ RAM_10 - MAX_RAM_LOCATIONS ] = val;
            break;
        case RAM_11:
            ram_[ RAM_11 - MAX_RAM_LOCATIONS ] = val;
            break;
        case RAM_12:
            ram_[ RAM_12 - MAX_RAM_LOCATIONS ] = val;
            break;
        case RAM_13:
            ram_[ RAM_13 - MAX_RAM_LOCATIONS ] = val;
            break;
        case RAM_14:
            ram_[ RAM_14 - MAX_RAM_LOCATIONS ] = val;
            break;
        case RAM_15:
            ram_[ RAM_15 - MAX_RAM_LOCATIONS ] = val;
            break;
        default:
            break;
    } 
}

/******************************************************************************/
/*!   \fn short getOpData( unsigned char type, short data )

      \brief
        
      \author
          Aaron Swift
*******************************************************************************/
short getOpData( unsigned char type, short data )
{    
    if( type == DIRECT_DATA ) {
        return( data );
    } else {
        return( getIndirectData( (CMD_DATA_IDS)data ) );
    }    
}

/******************************************************************************/
/*!   \fn void updateRam( IndirectDataItem *pRam )

      \brief
        
      \author
          Aaron Swift
*******************************************************************************/
void updateRam( IndirectDataItem *pRam )
{    
    for(int i = 0; ( i < MAX_RAM_LOCATIONS && pRam->location != 0 ); ++i ) {
        setIndirectData( (CMD_DATA_IDS)pRam->location,  pRam->value );    
    }  
}