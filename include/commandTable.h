#ifndef COMMANDTABLE_H
#define COMMANDTABLE_H

#define DIRECT_DATA                 0
#define INDIRECT_DATA               1



/* command data identifiers */ 
typedef enum 
{
    NONE_,
    CONFIG_0,                   
    CONFIG_1,                    
    CONFIG_2,                    
    CONFIG_3,                    
    CONFIG_4,                    
    CONFIG_5,                    
    CONFIG_6,                    
    CONFIG_7,                    
    CONFIG_8,                    
    CONFIG_9,                    
    CONFIG_10,                   
    CONFIG_11,                   
    CONFIG_12,                   
    CONFIG_13,                   
    CONFIG_14,                   
    CONFIG_15,                   
    RAM_0,                       
    RAM_1,                       
    RAM_2,                       
    RAM_3,                       
    RAM_4,                       
    RAM_5,                       
    RAM_6,                       
    RAM_7,                       
    RAM_8,                       
    RAM_9,                       
    RAM_10,                      
    RAM_11,                      
    RAM_12,                      
    RAM_13,                      
    RAM_14,                      
    RAM_15                      
}CMD_DATA_IDS;

#define MAX_RAM_LOCATIONS           16

typedef short      IndirectData;
typedef short      DirectData;


typedef struct indirect_data_item
{
    IndirectData    location;
    short             value;
}IndirectDataItem;


/* Test Operators */
#define BITS_EQUAL                  0
#define BITS_NOT_EQUAL              1
typedef unsigned char     TestOperator;

/* Test Actions */
#define CONTINUE_EXECUTION          0
#define ABORT_EXECUTION             1
#define SKIP_CONTINUE_EXECUTION     2    /* skip next command and execute the following */ 
typedef unsigned char     TestAction;

/* Bit Operators */
#define SET_BITS                    0
#define CLEAR_BITS                  1
typedef unsigned char     BitOperator;

/* Counter Operators */
#define RESET_COUNTER               0
#define ENABLE_COUNTER              1
#define DISABLE_COUNTER             2
typedef unsigned char     CounterOperator;

/* Label Orientation Identifiers */
#define HEEL_FIRST                  0
#define HEAD_FIRST                  1
typedef unsigned char     LabelOrientation;



/* Command Directives */
#define IDLE_DIRECTIVE                     0
#define PRINT_DIRECTIVE                    1
#define STEP_DIRECTIVE                     2
#define STEP_UNTIL_DIRECTIVE               3
#define WAIT_DIRECTIVE                     4
#define WAIT_UNTIL_DIRECTIVE               5
#define TEST_DIRECTIVE                     6
#define STATUS_DIRECTIVE                   7
#define MASK_DIRECTIVE                     8
#define COUNTER_DIRECTIVE                  9
#define DISABLE_DIRECTIVE                  10
#define CALIBRATE_DIRECTIVE                11     
#define REVERSE_STEP_DIRECTIVE             12
#define NOOPERATION_DIRECTIVE              13
#define CUT_DIRECTIVE                      14
#define HEAD_TEST_DIRECTIVE                15
#define VIRTUAL_CUT_DIRECTIVE              16 
#define STEP_GAP_DIRECTIVE                 17
#define STEP_EDGE_DIRECTIVE                18
#define TEST_FOR_SYNC                      19
#define TEST_FOR_LABEL                     20
#define WAIT_UNTIL_SIZING                  21
#define STEP_TAKEUP_DIRECTIVE              22
#define TEST_FOR_CONTINUOUS                23
#define STEP_TAKEUP_TIGHTEN                24
#define DETECTION_STEP_UNTIL               25

#define PRINT_DATA            RAM_0
#define ADVANCE_DATA          RAM_1
#define PEEL_DATA             CONFIG_0
#define ALIGN_DATA            CONFIG_1
#define RETRACT_DATA          RAM_2
#define EJECT_DATA            CONFIG_3
#define BACKUP_DATA           CONFIG_4


#define MISSING_LABEL         0x02
#define TAKE_LABEL            0x08




/* Command Operation Structures */
struct generic_operation
{
    unsigned char       directive;
    unsigned char       d0;
    unsigned char       d1;
    unsigned char       d2;
    unsigned char       d3;
    unsigned char       d4;
    unsigned char       d5;
    unsigned char       d6;
    unsigned char       d7;
    unsigned char       d8;
    unsigned char       d9;
    unsigned char       d10;    
};
typedef struct generic_operation GenericOperation;

struct idle_operation
{
    unsigned char    directive;

};
typedef struct idle_operation IdleOperation;

struct print_operation
{
    unsigned char       directive;
    unsigned char       type; 
    short               data; 
    LabelOrientation    orientation;
};
typedef struct print_operation PrintOperation;

struct step_operation
{
    unsigned char       directive;
    unsigned char       type; 
    short               data; 
};
typedef struct step_operation StepOperation;

struct step_until_operation
{
    unsigned char       directive;
    unsigned char       type; 
    short               data; 
    TestOperator        operator;
    unsigned char       bits;
    unsigned char       result;
    char                pad;
};
typedef struct step_until_operation StepUntilOperation;

struct wait_operation
{
    unsigned char       directive;
    unsigned char       type; 
    short               data; 
};
typedef struct wait_operation WaitOperation;

struct wait_until_operation
{
    unsigned char       directive;
    TestOperator        operator;
    unsigned char       bits;
    unsigned char       result;
};
typedef struct wait_until_operation WaitUntilOperation;

struct test_operation
{
    unsigned char       directive;
    TestOperator        operator;
    unsigned char       bits;
    unsigned char       result;
    unsigned char       true_bits;
    unsigned char       false_bits;
    TestAction          true_action;
    TestAction          false_action;
};
typedef struct test_operation TestOperation;

struct status_operation
{
    unsigned char       directive;
    BitOperator         operator;
    unsigned char       temp;           /* SOF-3134: for data packing only */
    unsigned char       bits;
};
typedef struct status_operation StatusOperation;

struct counter_operation
{
    unsigned char       directive;
    CounterOperator     operator;
};
typedef struct counter_operation CounterOperation;

typedef union command_operation
{
    GenericOperation    generic;
    IdleOperation       idle;
    PrintOperation      print;
    StepOperation       step;
    StepUntilOperation  step_until;
    WaitOperation       wait;
    WaitUntilOperation  wait_until;
    TestOperation       test;
    StatusOperation     status;
    CounterOperation    counter;
}CmdOp;
 
/* CommandTable */
#define MAX_TABLE_OPERATIONS    30

/* refactor commands (tables) */
#if 1
typedef enum
{
    _TBL_TEST_SHOOT_THROUGH_GAP_EDGE,
    _TBL_FIND_WO_LABEL_TAKEN,
    _TBL_BACKUP,
    _TBL_ADVANCE,
    _TBL_PRINT,
    _TBL_EJECT,
    _TBL_SYNC,
    _TBL_FIND,
    _TBL_SIZE_LABELS,
    _TBL_CUT_RETRACT,
    _TBL_DOT_WEAR_PRINT,
    _TBL_VIRT_LABEL_TAKEN,
    _TBL_SIZING_SHOOT_THROUGH_LABELS,
    _TBL_RSV0  = 13,    
    /* _TBL_RSV22 = 35,*/
    _TBL_MAX
}CMDTBLType;

#else 
#define COMMAND_0               0       /* shoot through test gap / edge */
#define COMMAND_1               1       /* findWoLabelTakenCmd */
#define COMMAND_2               2       /* backupCmd */
#define COMMAND_3               3       /* advanceCmd */
#define COMMAND_4               4       /* printCmd */
#define COMMAND_5               5       /* ejectCmd */
#define COMMAND_6               6       /* syncCmd */
#define COMMAND_7               7       /* findCmd */
#define COMMAND_8               8       /* sizeCmd */
#define COMMAND_9               9       /* cutRetractCmd */
#define COMMAND_10              10      /* printDotWearCmd */
#define COMMAND_11              11      /* virtualLabelTaken */
#define COMMAND_12              12      /* shoot through sizing */

#define MAX_TABLES              13

#endif

typedef unsigned char    CMDId;

/* command table */
typedef struct 
{
    CmdOp  oper[MAX_TABLE_OPERATIONS];
}CmdTable;

typedef enum
{
    _TESTGAP,
    _TESTEDGE,
    _TESTCUTTER,
    _TESTDRIVETRAIN,
    _TESTCLEANING,
    _TESTCLEANINGEXPEL,
    _TESTPAPERTAKEUP
}TestTables;


void setTableDefaultCmds( void );


void initializeTable( void );
void setTableTestCmds( TestTables test );
void copyTable (CmdOp *table, CmdOp **operations);
void buildCmdTable( CMDId id, unsigned char index, CmdOp *pOper );
short getOpData( unsigned char type, short data );
int getIndirectData( CMD_DATA_IDS id );
void setIndirectData( CMD_DATA_IDS id, int val );
void updateRam( IndirectDataItem *pRam );

#endif