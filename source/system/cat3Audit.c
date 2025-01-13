#include "cat3Audit.h"
#include "w25x10cl.h"
#include "serialFlash.h"
#include "weigher.h"
#include "string.h"
#include "fsl_debug_console.h"



static CalibrationEvent calEvent;
static ConfigurationEvent configEvent;
static char readRecordIndex;         /* index into audit page for read page message */
static char currentPageRead;         /* current page scale backend is reading from */


AT_NONCACHEABLE_SECTION_ALIGN( static AuditMGR auditMgr, 4U );
AT_NONCACHEABLE_SECTION_ALIGN( AuditPage page, 4U ); 
/* ***************************revision history ********************************/
/*                                                                            */
/*      changed date time arrays to epoch time:  06282016                     */       
/*      added weigher mode and changed division size and                      */
/*      added center of maintenance and gain factor to configuration event    */
/*      changed current page in read record message                           */           
/******************************************************************************/


/*! ****************************************************************************   
      \fn bool initializeAuditMGR()                                                                
      \public
      \brief
         This function initializes the audit manager with the last state of
         the manager. If the serial flash is blank, then this function will
         initialize the audit manager's section of the serial flash. The audit 
         manager begins in sector 2 page 0 of the serial flash.

      \return result        
      \author
          Aaron Swift
*******************************************************************************/ 

bool initializeAuditMGR()
{
    AuditMGR *pAuditMGR = NULL;
    AuditMGR aMgr;
    bool result = false;
    unsigned long startAddress = 0;
    AuditRecord firstRecord;
    AuditRecord secondRecord;
    
    /*clear the audit manager */
    memset( &auditMgr, 0, sizeof(AuditMGR) );     
    /* get the starting address of the audit trail. */
    startAddress = getAuditStartAddress();
    /* read the first page of the audit trail. */
    readAuditPage(startAddress);
    startAddress = findEmptyRcd();

    /* is the audit trail full? */
    if( isInRange(startAddress) )
    {      
        pAuditMGR = &aMgr;
        /* get the last state of the audit manager */
        if( readSerialFlash( SECTOR2_BASE_ADDR, (uint8_t *)pAuditMGR, sizeof(AuditMGR) ) )
        {           
            if( pAuditMGR != NULL )
            {
                /*are we dealing with a blank sector 2 device? */
                if ( pAuditMGR->initialized != 0xFF )
                {
                    memcpy( &auditMgr, pAuditMGR, sizeof(AuditMGR) );  
                }
                else
                {  
                    /* Audit manager section is blank, initialize the flash */
                    auditMgr.initialized = 1;
                    /* set the starting address */
                    auditMgr.addr = 0;
                    /* get the current serial flash sector */
                    auditMgr.currentSector = getSectorNumber(startAddress);
                    /* get the current page within the current sector */
                    auditMgr.currentPage = getPageNumber(startAddress);
                    
                    /*get the number of calibration and configuration records from 
                     the audit trail. */
                    auditMgr.numOfCalRecords = 0;       //getNumberOfRecords(CALIBRATION);
                    auditMgr.numOfConfigRecords = 0;    //getNumberOfRecords(CONFIGURATION);
                    auditMgr.numOfUpgradeRecords = 0;   //getNumberOfRecords(UPGRADE);
                    auditMgr.numOfTotalRecords = 0;     //auditMgr.numOfCalRecords + 
                                                        //auditMgr.numOfConfigRecords + auditMgr.numOfUpgradeRecords;
                  
                    auditMgr.centerOfMainCounter        = 0;
                    auditMgr.divisionSizeCounter        = 0;
                    auditMgr.filterSpeedCounter         = 0;
                    auditMgr.gainCoefficientCounter     = 0;
                    auditMgr.gainFactorCounter          = 0;
                    auditMgr.maxWeightCounter           = 0;
                    auditMgr.minWeightPrintCounter      = 0;
                    auditMgr.offsetCoefficientCounter   = 0;
                    auditMgr.weigherModelCounter        = 0;
                    auditMgr.zeroReferenceCounter       = 0;
                    auditMgr.weigherModeCounter         = 0;
                    
                    if( !writeAuditMGR( &auditMgr ) )
                    {                        
                        PRINTF( "initializeAuditMGR(): failed to write cat3 audit manager intialization to serial flash!\r\n" ); 
                    }
                    
                    /* version 3.2 initializes the audit manager at section 0, 
                     * page 0, record 0. This creates an invalid record in 
                     * version 4.0 */
                    
                    /* check for bad record zero */
                    startAddress = getAuditStartAddress();
                    readAuditPage(startAddress);
                    readRecord(&firstRecord , 0);
                    readRecord(&secondRecord, 1);
                    if( (firstRecord.recordTag != BLANK_RECORD) && (secondRecord.recordTag == BLANK_RECORD ) )
                    {
                        if( firstRecord.checksum == 0 )
                        {
                            erasePage(startAddress, SECTOR_0);   
                        }
                    }
                }
            }
        }
        else
        {
            PRINTF( "initializeAuditMGR(): failed to read cat3 audit manager section in serial flash!\r\n" );
          
            auditMgr.initialized = 1;
            /* set the starting address */
            auditMgr.addr = startAddress;
            /* get the current serial flash sector */
            auditMgr.currentSector = getSectorNumber(startAddress);
            /* get the current page within the current sector */
            auditMgr.currentPage = getPageNumber(startAddress);
            
            /*get the number of calibration and configuration records from 
             the audit trail. */
            auditMgr.numOfCalRecords = getNumberOfRecords(CALIBRATION);
            auditMgr.numOfConfigRecords = getNumberOfRecords(CONFIGURATION);
            auditMgr.numOfUpgradeRecords = getNumberOfRecords(UPGRADE);
            auditMgr.numOfTotalRecords = auditMgr.numOfCalRecords + 
            auditMgr.numOfConfigRecords + auditMgr.numOfUpgradeRecords;

            auditMgr.centerOfMainCounter        = 0;
            auditMgr.divisionSizeCounter        = 0;
            auditMgr.filterSpeedCounter         = 0;
            auditMgr.gainCoefficientCounter     = 0;
            auditMgr.gainFactorCounter          = 0;
            auditMgr.maxWeightCounter           = 0;
            auditMgr.minWeightPrintCounter      = 0;
            auditMgr.offsetCoefficientCounter   = 0;
            auditMgr.weigherModelCounter        = 0;
            auditMgr.zeroReferenceCounter       = 0;
            auditMgr.weigherModeCounter         = 0;
            
            if( !writeAuditMGR( &auditMgr ) )
            {
                PRINTF( "initializeAuditMGR(): failed to write intial cat3 audit manager section in serial flash!\r\n" ); 
            }     
        }        
        result = true;
          
    }
    else
    {        
        /* the audit trail is full */
        auditTrailRollOver();
    }    
    return result;
}

/*! ****************************************************************************   
      \fn void resetAuditMGR()                                                                
      \public
      \brief
         This function resets the audit manager.
      
      \author
          Aaron Swift
*******************************************************************************/ 
void resetAuditMGR()
{
    /*clear the audit manager */
    memset( &auditMgr, 0, sizeof(AuditMGR) );  
    auditMgr.initialized = 1;
    writeAuditMGR(&auditMgr);  
}

/*! ****************************************************************************   
      \fn static void updateAuditMGR(unsigned long addr, AuditEvent event)                                                              
      \private
      \brief
         This function saves updates the audit manager  
                  
      \param unsigned long addr, AuditEvent event
      \return result        
      \author
          Aaron Swift
*******************************************************************************/ 
static void updateAuditMGR(unsigned long addr, AuditEvent event)
{
    /* update the record event counters */
    if( event == CALIBRATION )
    {
        auditMgr.numOfCalRecords++;
    }
    if(  event == CONFIGURATION )
    {
        auditMgr.numOfConfigRecords++;
    }
    if( event == UPGRADE )
    {
        auditMgr.numOfUpgradeRecords++;
    }
    auditMgr.numOfTotalRecords++;
    
    /* update the current available record address */
    auditMgr.addr += sizeof(AuditRecord);
    /* update the current page number  */
    auditMgr.currentPage = getPageNumber(auditMgr.addr);
    /* update the current sector number  */
    auditMgr.currentSector = getSectorNumber(auditMgr.addr);
    /* save the current state of the audit manager */
    writeAuditMGR(&auditMgr);
}

/*! ****************************************************************************   
      \fn static void updateAuditMGR(unsigned long addr, AuditEvent event)                                                              
      \private
      \brief
         This function saves updates the audit manager  
                  
      \param unsigned long addr, AuditEvent event
      \return result        
      \author
          Aaron Swift
*******************************************************************************/ 
static bool writeAuditMGR(AuditMGR *pAuditMGR)
{
    bool result = false;
    if( pAuditMGR != NULL )
    {
        /* erase the audit manager's page of the serial flash */ 
        if ( erasePage( SECTOR2_BASE_ADDR, SECTOR_2 ) )
        {
            /* write the audit manager's section of the serial flash */
            if( writeSerialFlash( ( unsigned char * )pAuditMGR, SECTOR2_BASE_ADDR, sizeof(AuditMGR), SECTOR_2 ) )
            {
                result = true;
            }
        }
    }
    return result;
}

/*! ****************************************************************************   
      \fn initializeEvents(WgConfiguration *pConfig)                                                              
      \public
      \brief
         This function initializes the CAT3 event structures so previous and  
         current sealable parameters can be tracked.
                  
      \param WgConfiguration pointer to weigher configuration      
      \author
          Aaron Swift
*******************************************************************************/ 
void initializeEvents(WgConfiguration *pConfig)
{
    /* intialize the sealable calibration events */
    calEvent.center_of_maintenance              = pConfig->center_of_maintenance_zone;
    calEvent.gain_factor                        = pConfig->gain_factor;
    
    /* intialize the sealable configuration events */
    configEvent.filterSpeed                     = pConfig->filter_speed;                 
    configEvent.maxWeight                       = pConfig->max_weight;              
    configEvent.min_weight_to_print             = pConfig->min_weight_to_print;        
    configEvent.weigherModel                    = pConfig->weigher_model;         
    /* this is the scale factor which weights and measures calls division size */
    configEvent.division_size                   = pConfig->scale_factor;
    /* weigh mode is either  avoir (1) or metric (0) */
    configEvent.weigh_mode                      = ( pConfig->flags & ~DUALRANGE_NETWEIGHTLIMITCHECK );
}

/*! ****************************************************************************   
      \fn void saveCalibEvents(WgConfiguration *pConfig)                                                             
      \public
      \brief
         This function saves configuration and calibration events if event 
         changed to the Cat3 audit log.
                  
      \param WgConfiguration *pConfig pointer to Weigher configuration.
      \author
          Aaron Swift
*******************************************************************************/ 
void saveCalibEvents(WgConfiguration *pConfig)
{
    AuditRecord record;
    bool anyChange = false;
    
    /* check all sealable calibration parameters for change and record if found. */
    if( calEvent.gain_factor != pConfig->gain_factor )
    {
        anyChange = true;
        
        /* create audit record and save event to flash */
        record.event = CALIBRATION;
        record.parameterId = GAIN_FACTOR;
        record.newParamValue = pConfig->gain_factor;
        record.oldParamValue = calEvent.gain_factor;
        
        record.epochTime = calEvent.epochTime;
        
        /* save the record to serial flash */
        if( ! writeSerialAuditRcd( auditMgr.addr, &record ) )
        {
            /* did the audit trail rollover? */
            if( ! isInRange(auditMgr.addr) )
            {
                /*handle rollover */
                auditTrailRollOver();        
            }
            else
            {              
                PRINTF( "saveCalibEvents(): failed to log cat3 calibration event to serial flash!\r\n" ); 
            }          
        }
        
        /* update the current and previous values in the event */
        calEvent.previous_gain_factor = calEvent.gain_factor;
        calEvent.gain_factor = pConfig->gain_factor;
        
        /* increment the sealable parameter counter (requirement 8.24c, 8.24d */
        auditMgr.gainFactorCounter += 1;
        if( auditMgr.gainFactorCounter == MAX_CAT3_COUNT )
            auditMgr.gainFactorCounter = 0;

    }
    if( calEvent.center_of_maintenance != pConfig->center_of_maintenance_zone )
    {
        anyChange = true;
        
        /* create audit record and save event to flash */
        record.event = CALIBRATION;
        record.parameterId = CENTER_OF_MAINTENANCE_ZONE;
        record.newParamValue = pConfig->center_of_maintenance_zone;
        record.oldParamValue = calEvent.center_of_maintenance;
        
        record.epochTime = calEvent.epochTime;
        
        /* save the record to serial flash */
        if( ! writeSerialAuditRcd( auditMgr.addr, &record ) )
        {
            /* did the audit trail rollover? */
            if( ! isInRange(auditMgr.addr) )
            {
                /*handle rollover */
                auditTrailRollOver();        
            }
            else
            {
                PRINTF( "saveCalibEvents(): failed to log cat3 calibration event to serial flash!\r\n" ); 
            }          
        }
        
        /* update the current and previous values in the event */
        calEvent.previous_center_of_maintenance = calEvent.center_of_maintenance;
        calEvent.center_of_maintenance = pConfig->center_of_maintenance_zone; 
        
        /* increment the sealable parameter counter (requirement 8.24c, 8.24d */
        auditMgr.centerOfMainCounter += 1;
        if( auditMgr.centerOfMainCounter == MAX_CAT3_COUNT )
            auditMgr.centerOfMainCounter = 0;

    }
    
    /* has any change been detected? */
    if( anyChange )
    {
        /* update the audit manager's serial flash page. */
        if( !writeAuditMGR(&auditMgr) )
        {
            PRINTF( "saveCalibEvents(): failed to write cat3 audit manager page to serial flash!\r\n" );   
        }
    }
}

/*! ****************************************************************************   
      \fn void saveConfigEvents(WgConfiguration *pConfig)                                                             
      \public
      \brief
         This function saves configuration events if sealable configuration  
         values change to the Cat3 audit log.
                  
      \param WgConfiguration *pConfig  pointer to Weigher configuration.
      \author
          Aaron Swift
*******************************************************************************/ 
void saveConfigEvents(WgConfiguration *pConfig)
{
    AuditRecord record;  
    bool anyChange = false;
    
    /* check all sealable configuration parameters for change and record if found. */
    if( configEvent.filterSpeed != pConfig->filter_speed )
    {
        anyChange = true;
        
        /* create audit record and save event to flash */
        record.event = CONFIGURATION;
        record.parameterId = FILTER_SPEED;
        record.newParamValue = pConfig->filter_speed;
        record.oldParamValue = configEvent.filterSpeed;
        
        record.epochTime = configEvent.epochTime;
        
        /* save the record to serial flash */
        if( ! writeSerialAuditRcd( auditMgr.addr, &record ) )
        {
            /* did the audit trail rollover? */
            if( ! isInRange(auditMgr.addr) )
            {
                /*handle rollover */
                auditTrailRollOver();
            }
            else
            {
                PRINTF( "saveConfigEvents(): failed to log cat3 configuration event to serial flash!\r\n" );  
            }          
        }
        
        /* update the current and previous values in the event */
        configEvent.previous_filterSpeed = configEvent.filterSpeed;
        configEvent.filterSpeed = pConfig->filter_speed;            
        
        /* increment the sealable parameter counter (requirement 8.24c, 8.24d */
        auditMgr.filterSpeedCounter += 1;
        if( auditMgr.filterSpeedCounter == MAX_CAT3_COUNT )
            auditMgr.filterSpeedCounter = 0;

    }
    
    if( configEvent.maxWeight != pConfig->max_weight )
    {
        anyChange = true;
        
        /* create audit record and save event to flash */
        record.event = CONFIGURATION;
        record.parameterId = MAX_WEIGHT;
        record.newParamValue = pConfig->max_weight;
        record.oldParamValue = configEvent.maxWeight;
        
        record.epochTime = configEvent.epochTime;
        
        /* save the record to serial flash */
        if( ! writeSerialAuditRcd( auditMgr.addr, &record ) )
        {
            /* did the audit trail rollover? */
            if( ! isInRange(auditMgr.addr) )
            {
                /*handle rollover */
                auditTrailRollOver();        
            }
            else
            {
                PRINTF( "saveConfigEvents(): failed to log cat3 configuration event to serial flash!\r\n" );  
            }       
        }
        
        /* update the current and previous values in the event */
        configEvent.previous_maxWeight = configEvent.maxWeight;
        configEvent.maxWeight = pConfig->max_weight;
        
        /* increment the sealable parameter counter (requirement 8.24c, 8.24d ) */
        auditMgr.maxWeightCounter += 1;
        if( auditMgr.maxWeightCounter == MAX_CAT3_COUNT )
            auditMgr.maxWeightCounter = 0;
      
    }
    
    if( configEvent.min_weight_to_print != pConfig->min_weight_to_print )
    {
        anyChange = true;
        
        /* create audit record and save event to flash */
        record.event = CONFIGURATION;
        record.parameterId = MIN_WEIGHT_PRINT;
        record.newParamValue = pConfig->min_weight_to_print;
        record.oldParamValue = configEvent.min_weight_to_print;
        
        record.epochTime = configEvent.epochTime;
        
        /* save the record to serial flash */
        if( ! writeSerialAuditRcd( auditMgr.addr, &record ) )
        {
            /* did the audit trail rollover? */
            if( ! isInRange(auditMgr.addr) )
            {
                /*handle rollover */
                auditTrailRollOver();        
            }
            else
            {
                PRINTF( "saveConfigEvents(): failed to log cat3 configuration event to serial flash!\r\n" );   
            }            
        }
        
        /* update the current and previous values in the event */
        configEvent.previous_min_weight_to_print = configEvent.min_weight_to_print;
        configEvent.min_weight_to_print = pConfig->min_weight_to_print;            
      
        /* increment the sealable parameter counter (requirement 8.24c, 8.24d ) */
        auditMgr.minWeightPrintCounter += 1;
        if( auditMgr.minWeightPrintCounter == MAX_CAT3_COUNT )
            auditMgr.minWeightPrintCounter = 0;

    }
    
    if( configEvent.weigherModel != pConfig->weigher_model )
    {
        anyChange = true;
        
        /* create audit record and save event to flash */
        record.event = CONFIGURATION;
        record.parameterId = WEIGHER_MODEL;
        record.newParamValue = pConfig->weigher_model;
        record.oldParamValue = configEvent.weigherModel;
        
        record.epochTime = configEvent.epochTime;
        
        /* save the record to serial flash */
        if( ! writeSerialAuditRcd( auditMgr.addr, &record ) )
        {
            /* did the audit trail rollover? */
            if( ! isInRange(auditMgr.addr) )
            {
                /*handle rollover */
                auditTrailRollOver();        
            }
            else
            {
                PRINTF( "saveConfigEvents(): failed to log cat3 configuration event to serial flash!\r\n" );  
            }          
        }
        
        /* update the current and previous values in the event */
        configEvent.previous_weigherModel = configEvent.weigherModel;
        configEvent.weigherModel = pConfig->weigher_model;            
      
        /* increment the sealable parameter counter (requirement 8.24c, 8.24d ) */
        auditMgr.weigherModelCounter += 1;
        if( auditMgr.weigherModelCounter == MAX_CAT3_COUNT )
            auditMgr.weigherModelCounter = 0;
    
    }
    
    if( configEvent.division_size != pConfig->scale_factor )
    {
        anyChange = true;
        
        /* create audit record and save event to flash */
        record.event = CONFIGURATION;
        record.parameterId = DIVISION_SIZE;
        record.newParamValue = pConfig->scale_factor;
        record.oldParamValue = configEvent.division_size;        
        record.epochTime = configEvent.epochTime;
        
        /* save the record to serial flash */
        if( ! writeSerialAuditRcd( auditMgr.addr, &record ) )
        {
            /* did the audit trail rollover? */
            if( ! isInRange(auditMgr.addr) )
            {
                /*handle rollover */
                auditTrailRollOver();        
            }
            else
            {
                PRINTF( "saveConfigEvents(): failed to log cat3 configuration event to serial flash!\r\n" );   
            }          
        }
        
        /* update the current and previous values in the event */
        configEvent.previous_division_size = configEvent.division_size;
        configEvent.division_size =  pConfig->scale_factor;            
      
        /* increment the sealable parameter counter (requirement 8.24c, 8.24d ) */
        auditMgr.divisionSizeCounter += 1;
        if( auditMgr.divisionSizeCounter == MAX_CAT3_COUNT )
            auditMgr.divisionSizeCounter = 0;   
    }
    
    if( configEvent.weigh_mode !=( pConfig->flags & ~DUALRANGE_NETWEIGHTLIMITCHECK ) )
    {
        anyChange = true;
        
        /* create audit record and save event to flash */
        record.event = CONFIGURATION;
        record.parameterId = WEIGHER_MODE;
        record.newParamValue = ( pConfig->flags & ~DUALRANGE_NETWEIGHTLIMITCHECK );
        record.oldParamValue = configEvent.weigh_mode;        
        record.epochTime = configEvent.epochTime;
        
        /* save the record to serial flash */
        if( ! writeSerialAuditRcd( auditMgr.addr, &record ) )
        {
            /* did the audit trail rollover? */
            if( ! isInRange(auditMgr.addr) )
            {
                /*handle rollover */
                auditTrailRollOver();        
            }
            else
            {
                PRINTF( "saveConfigEvents(): failed to log cat3 configuration event to serial flash!\r\n" );  
            }          
        }
        
        /* update the current and previous values in the event */
        configEvent.previous_weigh_mode = configEvent.weigh_mode;
        configEvent.weigh_mode = ( pConfig->flags & ~DUALRANGE_NETWEIGHTLIMITCHECK );            
      
        /* increment the sealable parameter counter (requirement 8.24c, 8.24d ) */
        auditMgr.weigherModeCounter += 1;
        if( auditMgr.weigherModeCounter == MAX_CAT3_COUNT )
            auditMgr.weigherModeCounter = 0;         
    }
    
    /* Note: center of maintenance and gain factor can change during a 
       configuration event. */
    if( calEvent.center_of_maintenance != pConfig->center_of_maintenance_zone )
    {
        anyChange = true;
        
        /* create audit record and save event to flash */
        record.event = CONFIGURATION;
        record.parameterId = CENTER_OF_MAINTENANCE_ZONE;
        record.newParamValue = pConfig->center_of_maintenance_zone;
        record.oldParamValue = calEvent.center_of_maintenance;
        
        record.epochTime = configEvent.epochTime;
        
        /* save the record to serial flash */
        if( ! writeSerialAuditRcd( auditMgr.addr, &record ) )
        {
            /* did the audit trail rollover? */
            if( ! isInRange(auditMgr.addr) )
            {
                /*handle rollover */
                auditTrailRollOver();        
            }
            else
            {
                PRINTF( "saveConfigEvents(): failed to log cat3 calibration event to serial flash!\r\n" );   
            }          
        }
        
        /* update the current and previous values in the event */
        calEvent.previous_center_of_maintenance = calEvent.center_of_maintenance;
        calEvent.center_of_maintenance = pConfig->center_of_maintenance_zone; 
        
        /* increment the sealable parameter counter (requirement 8.24c, 8.24d */
        auditMgr.centerOfMainCounter += 1;
        if( auditMgr.centerOfMainCounter == MAX_CAT3_COUNT )
            auditMgr.centerOfMainCounter = 0;

    }
    
    if( calEvent.gain_factor != pConfig->gain_factor )
    {
        anyChange = true;
        
        /* create audit record and save event to flash */
        record.event = CONFIGURATION;
        record.parameterId = GAIN_FACTOR;
        record.newParamValue = pConfig->gain_factor;
        record.oldParamValue = calEvent.gain_factor;
        
        record.epochTime = configEvent.epochTime;
        
        /* save the record to serial flash */
        if( ! writeSerialAuditRcd( auditMgr.addr, &record ) )
        {
            /* did the audit trail rollover? */
            if( ! isInRange(auditMgr.addr) )
            {
                /*handle rollover */
                auditTrailRollOver();        
            }
            else
            {
                PRINTF( "saveConfigEvents(): failed to log cat3 calibration event to serial flash!\r\n" );   
            }          
        }
        /* update the current and previous values in the event */
        calEvent.previous_gain_factor = calEvent.gain_factor;
        calEvent.gain_factor = pConfig->gain_factor; 
        
        /* increment the sealable parameter counter (requirement 8.24c, 8.24d */
        auditMgr.gainFactorCounter += 1;
        if( auditMgr.gainFactorCounter == MAX_CAT3_COUNT )
            auditMgr.gainFactorCounter = 0;      
    }
    
    /* has any change been detected? */
    if( anyChange )
    {   
        /* update the audit manager's serial flash page */
        if( !writeAuditMGR(&auditMgr) )
        {
            PRINTF( "saveConfigEvents(): failed to write cat3 audit manager page to serial flash!\r\n" );   
        }
    }
}

/*! ****************************************************************************   
      \fn void incAuditMGRRecordCount()                                                              
      \public
      \brief
         This function increments total record count and upgrade record count.
         This function should only be used to record update, fractional price,
         and pounds for events.
      \author
          Aaron Swift
*******************************************************************************/ 
void incAuditMGRRecordCount()
{
    auditMgr.numOfTotalRecords += 1;
    auditMgr.numOfUpgradeRecords += 1;

    /* update the audit manager's serial flash page */
    if( !writeAuditMGR(&auditMgr) )
    {
        PRINTF( "incAuditMGRRecordCount(): failed to write cat3 audit manager page to serial flash!\r\n" );   
    }    
}

/*! ****************************************************************************   
      \fn void eraseAuditLog()                                                                
      \public
      \brief
         This function erases the audit manager from serial flash and write 
         intialized audit manager state to the serial flash. This should only 
         be used for testing.
      
      \author
          Aaron Swift
*******************************************************************************/ 
void eraseAuditLog()
{
    /* get starting address of the audit log  */   
    unsigned long addr = getAuditStartAddress();

    eraseSector( 0, SECTOR_0 );    
    eraseSector( SECTOR1_BASE_ADDR, SECTOR_1 );    
    readAuditPage(0);   
   
    /* reset the audit manager */
    auditMgr.initialized = 1;
    /* set the starting address */
    auditMgr.addr = addr;
    /* get the current serial flash sector */
    auditMgr.currentSector = getSectorNumber(addr);
    /* get the current page within the current sector */
    auditMgr.currentPage = getPageNumber(addr);
    
    /*get the number of calibration and configuration records from 
     the audit trail. */
    auditMgr.numOfCalRecords = 0;
    auditMgr.numOfConfigRecords = 0;
    auditMgr.numOfUpgradeRecords = 0;
    auditMgr.numOfTotalRecords = 0;
    auditMgr.numOfConfigRecords = 0;
  
    auditMgr.centerOfMainCounter        = 0;
    auditMgr.divisionSizeCounter        = 0;
    auditMgr.filterSpeedCounter         = 0;
    auditMgr.gainCoefficientCounter     = 0;
    auditMgr.gainFactorCounter          = 0;
    auditMgr.maxWeightCounter           = 0;
    auditMgr.minWeightPrintCounter      = 0;
    auditMgr.offsetCoefficientCounter   = 0;
    auditMgr.weigherModelCounter        = 0;
    auditMgr.zeroReferenceCounter       = 0;
    auditMgr.weigherModeCounter         = 0;
    
    writeAuditMGR( &auditMgr );  
}

/*! ****************************************************************************   
      \fn void updateDateTimeEvents(unsinged long time)                                                              
      \public
      \brief
         This function initializes the CAT3 event structures date and time.
                  
      \param unsigned char *pDate pointer to date string
      \param unsigned char *pTime pointer to time string
      \author
          Aaron Swift
*******************************************************************************/ 
void updateDateTimeEvents(unsigned long time)
{
    calEvent.epochTime = time;
    configEvent.epochTime = time;
}

/*! ****************************************************************************   
      \fn void getAuditMgrStats(WgCat3Statistics *pStatistics)                                                              
      \public
      \brief
         This function initializes the CAT3 statistics message structure with 
         the contents of the audit manager.               
      \param WgCat3Statistics *pStatistics pointer to the statistics message
                                           structure.
      \author
          Aaron Swift
*******************************************************************************/ 
void getAuditMgrStats(WgCat3Statistics *pStatistics)
{

    pStatistics->currentPage                    = auditMgr.currentPage;
    pStatistics->totalFreeRecords               = TOTAL_RECORD_STORAGE_SIZE - auditMgr.numOfTotalRecords;
    pStatistics->totalPages                     = NUM_PAGES_PER_BLOCK * 2;
    pStatistics->totalRecordsPerPage            = NUM_RECORDS_PER_PAGE;
    pStatistics->totalRecordedEvents            = auditMgr.numOfTotalRecords;
    pStatistics->totalCalibrationRecords        = auditMgr.numOfCalRecords;
    pStatistics->totalConfigurationRecords      = auditMgr.numOfConfigRecords;
    pStatistics->centerOfMainCounter            = auditMgr.centerOfMainCounter;
    pStatistics->filterSpeedCounter             = auditMgr.filterSpeedCounter;  
    pStatistics->gainCoefficientCounter         = auditMgr.gainCoefficientCounter;
    pStatistics->gainFactorCounter              = auditMgr.gainFactorCounter;
    pStatistics->maxWeightCounter               = auditMgr.maxWeightCounter;  
    pStatistics->minWeightPrintCounter          = auditMgr.minWeightPrintCounter;
    pStatistics->offsetCoefficientCounter       = auditMgr.offsetCoefficientCounter;
    pStatistics->weigherModelCounter            = auditMgr.weigherModelCounter;
    pStatistics->zeroReferenceCounter           = auditMgr.zeroReferenceCounter;
    pStatistics->divisionSizeCounter            = auditMgr.divisionSizeCounter;
    pStatistics->weigherModeCounter             = auditMgr.weigherModeCounter;
}

/*! ****************************************************************************   
      \fn unsigned char getAuditManagerCurrentPage()                                                               
      \public
      \brief
         This function returns the current working page of the audit log.         
      \author
          Aaron Swift
*******************************************************************************/ 
unsigned char getAuditManagerCurrentPage()
{
    return auditMgr.currentPage;
}

/*! ****************************************************************************   
      \fn bool auditManagerSaveRecord(AuditRecord *pRecord)                                                             
      \public
      \brief
         This function saves the audit record to serial flash and check 
         for log rollover.         

      \author
          Aaron Swift
*******************************************************************************/ 
bool auditManagerSaveRecord(AuditRecord *pRecord)
{
    bool result = false;
    
    /* save the record to serial flash */
    if( writeSerialAuditRcd( auditMgr.addr, pRecord ) )
    {
        result = true;
    }
    else
    {
        /* did the audit trail rollover? */
        if( ! isInRange(auditMgr.addr) )
        {
            /*handle rollover */
            auditTrailRollOver();
            
            /* save the record to serial flash */
            if( writeSerialAuditRcd( auditMgr.addr, pRecord ) )
            {
                result = true;
            }
        }
        
    }
    return result;
}

/*! ****************************************************************************   
      \fn void setReadPageIndex(unsigned char index)                                                             
      \public
      \brief
         This function sets the read page index

      \author
          Aaron Swift
*******************************************************************************/ 
void setReadPageIndex(unsigned char index)
{
    readRecordIndex  = index;
}

/*! ****************************************************************************   
      \fn unsigned char getReadPageIndex()                                                            
      \public
      \brief
         This function returns the read page index

      \author
          Aaron Swift
*******************************************************************************/ 
unsigned char getReadPageIndex()
{
    return readRecordIndex;
}

/*! ****************************************************************************   
      \fn void setReadPageNumber(unsigned char number)                                                           
      \public
      \brief
         This function sets the read page page number.

      \author
          Aaron Swift
*******************************************************************************/ 
void setReadPageNumber(unsigned char number)
{
    currentPageRead = number;
}

/*! ****************************************************************************   
      \fn void setReadPageNumber(unsigned char number)                                                           
      \public
      \brief
         This function returns the read page page number.

      \author
          Aaron Swift
*******************************************************************************/ 
unsigned char getReadPageNumber()
{
    return currentPageRead;
}

/*! ****************************************************************************   
      \fn void readRecords(unsigned char *pRecords)                                                           
      \public
      \brief
         This function reads all records saved to a page.

      \author
          Aaron Swift
*******************************************************************************/ 
void readRecords(unsigned char *pRecords)
{
    int i = 0, remainder = 0;
    if( readRecordIndex <= NUM_RECORDS_PER_PAGE )
    {
        /* last 8 records */
        if( readRecordIndex < NUM_RECORDS_PER_PAGE - 8 ) 
        {
            /* copy 10 records */
            for( i = 0; i < 10; i++ )
            {
                memcpy(pRecords, &page.records[ readRecordIndex++ ], sizeof(AuditRecord));
                pRecords +=  sizeof(AuditRecord);
            }
        }
        else
        {
            remainder = NUM_RECORDS_PER_PAGE - readRecordIndex;
            if( remainder != 0 )
            {
                /* set record buffer */
                memset( pRecords, 0xff,  (sizeof(AuditRecord) * 10) );
                /* copy the remaining records */
                for( i = 0; i < remainder; i++ )
                {
                    memcpy(pRecords, &page.records[ readRecordIndex++ ], sizeof(AuditRecord));
                    pRecords +=  sizeof(AuditRecord);
                }
            }
            else
            {
               /* error: clear record buffer */
               memset( pRecords, 0,  (sizeof(AuditRecord) * 10) );
            }
                     
        }
    }
    else
    {
      /* error: clear record buffer */
      memset( pRecords, 0,  (sizeof(AuditRecord) * 10) );
    }
}

/*! ****************************************************************************   
      \fn readRecord(AuditRecord *pRecord, unsigned int index)                                                            
      \public
      \brief
         This function read one record from the page buffer for the givin index.
         serial flash page must be read before calling this function.
      \author
          Aaron Swift
*******************************************************************************/ 
void readRecord(AuditRecord *pRecord, unsigned int index)
{
    if( index <= NUM_RECORDS_PER_PAGE )
    {
        memcpy(pRecord, &page.records[ index ], sizeof(AuditRecord));  
    }
}

/*! ****************************************************************************   
      \fn static bool auditTrailRollOver()                                                              
      \private
      \brief
         This function deals with CAT3 requirement 8.29 when audit trail becomes  
         full the audit manager will drop the oldest sector of the flash and copies 
         the latest to the oldest.
               
      \return true if sucessful        
      \author
          Aaron Swift
*******************************************************************************/ 
static void auditTrailRollOver()
{
    unsigned long addr;
    unsigned int sectorPage = 0, i = 0;

    /* erase serial flash sector 0 and replace with the contents of sector 1
       and erase sector 1. */
    if( eraseSector( getAuditStartAddress(), SECTOR_0 ) )
    {
        /* set the write address to the first page of sector 0 */    
        addr = 0;
        /* point to our most recent sector */
        auditMgr.currentSector = SECTOR_1;
        auditMgr.currentPage = 0;
        auditMgr.addr = getSectorBaseAddress( SECTOR_1 );
        for( sectorPage = 0; sectorPage < NUM_PAGES_PER_BLOCK; sectorPage++ )
        {
            /* replace with the contents of sector 1 */
            readAuditPage( auditMgr.addr );
            
            /* erase sector 1 page */
            erasePage(getPageBaseAddress(), SECTOR_1);

            /* copy page */
            for( i = 0; i < NUM_RECORDS_PER_PAGE; i++ )
            {
                writeSerialAuditRcd(addr, &page.records[i]);
                addr += sizeof(AuditRecord);
            }
            
        }
        
        readAuditPage(0);
        
        /* sector 1 of the serial flash is now free update our audit manager */
        auditMgr.currentPage = 0;
        auditMgr.currentSector = SECTOR_1;
        auditMgr.addr = getSectorBaseAddress( SECTOR_1 );
        auditMgr.numOfCalRecords = getNumberOfRecords(CALIBRATION);
        auditMgr.numOfConfigRecords =  getNumberOfRecords(CONFIGURATION);
        auditMgr.numOfUpgradeRecords = getNumberOfRecords(UPGRADE);
        auditMgr.numOfTotalRecords = auditMgr.numOfCalRecords + 
        auditMgr.numOfConfigRecords + auditMgr.numOfUpgradeRecords;
        
        /* do not reset the sealable parameter counters */
        if( !writeAuditMGR( &auditMgr ) )
        {
            PRINTF( "auditTrailRollOver(): failed to write cat3 audit manager page to serial flash!\r\n" );   
        }
          
    }
    else
    {
        PRINTF( "auditTrailRollOver(): failed to erase cat3 audit manager page in serial flash!\r\n" );       
    } 
}

/*! ****************************************************************************   
      \fn bool readSerialAuditRcd( AuditRecord *pAuditRecord )                                                              
      \public
      \brief
         This function reads the audit record at the given address and return 
         the status of the read operation. If the read operation fails, no data 
         will be copied to the record.
                  
      \param pAuditRecord pointer to an empty audit record
      \return result        
      \author
          Aaron Swift
*******************************************************************************/ 
bool readSerialAuditRcd( unsigned long addr, AuditRecord *pAuditRecord  )
{  
    bool result = false;  
    AuditRecord *pRecord = NULL;
    AuditRecord record;
    /*check that the address is in bounds of our audit sector */
    if(( pAuditRecord != NULL ) && ( addr <  SECTOR2_BASE_ADDR ) )
    {      
        pRecord = &record;
        if( readSerialFlash( addr, (uint8_t *)pRecord, sizeof(AuditRecord) ) )
        {
            /*did the controller encounter and error? */
            if( pRecord != NULL )
            {
                /*are we dealing with a blank record */
                if (pRecord->recordTag !=  BLANK_RECORD)
                {
                    memcpy(pAuditRecord, pRecord, sizeof(AuditRecord));  
                    result = true;
                }
  
            }
        }
    }
    return result;

}

/*! ****************************************************************************   
      \fn static bool writeSerialAuditRcd( AuditRecord *pAuditRecord )                                                              
      \private
      \brief
         This function writes a record at the address specified and return status 
         of operation. This function will return a failure if address is out of 
         bounds of the audit section or the record is null.

      \param pAuditRecord pointer to the audit record to be written
      \return result        
      \author
          Aaron Swift
*******************************************************************************/ 
static bool writeSerialAuditRcd( unsigned long addr, AuditRecord *pAuditRecord )
{
    bool result = false;  

    /*check that the address is in bounds of our audit sector */
    if(( pAuditRecord != NULL ) && ( addr <  SECTOR2_BASE_ADDR) )
    {
        if( writeSerialFlashNE ( (unsigned char *)pAuditRecord, addr, sizeof(AuditRecord), (FLASH_SECTORS)auditMgr.currentSector ) )
        {
            updateAuditMGR( addr, pAuditRecord->event );
            result = true;
        }
    }
    return result;
}

/*! ****************************************************************************   
      \fn VOID readAuditPage(unsigned long addr)                                                              
      \public
      \brief
         This function reads and entire page starting from address. 
      \param addr of  page to be read
      \author
          Aaron Swift
*******************************************************************************/ 
void readAuditPage(unsigned long addr)
{
    int i = 0;

    unsigned long endAddr = addr;
    endAddr += PAGE_SIZE;
    
    /*check that the address is in bounds of our audit sector */
    if( addr <  SECTOR2_BASE_ADDR  )
    {      
        while( addr < endAddr )
        {
            AuditRecord record;
            if( readSerialFlash( addr, (uint8_t *)&record, sizeof(AuditRecord) ) )
            {
                memcpy(&(page.records[i++]), &record, sizeof(AuditRecord));
            }
            else 
            {
                PRINTF("readAuditPage(): failed to read serial flash\r\n");
            }
            addr += sizeof(AuditRecord);
         }
    }
    
    /* is the entire page blank? */
    if(page.records[0].event == BLANK_RECORD)
    {
        page.blank = true;
    }
    else
    {
        page.blank = false;
    }
}

/*! ****************************************************************************   
      \fn bool isAuditPageBlank()                                                              
      \public
      \brief
         This function return the page blank flag. 
      \param addr of  page to be read
      \author
          Aaron Swift
*******************************************************************************/ 
bool isAuditPageBlank()
{
    return page.blank;
}

/*! ****************************************************************************   
      \fn static bool eraseAuditPage(unsigned long addr)                                                             
      \private
      \brief
         This function erases a page within the bounds of the audit trail.  
    
      \return true page is erased
      \author
          Aaron Swift
*******************************************************************************/ 
static bool eraseAuditPage(unsigned long addr)
{
    unsigned short calibrationEvents, configurationEvents, upgradeEvents, count;
    bool result = false;
    
    /* before we erase the page, we must update our record counters */
    calibrationEvents = getNumberOfRecordsPage(CALIBRATION);
    configurationEvents = getNumberOfRecordsPage(CONFIGURATION);
    upgradeEvents = getNumberOfRecordsPage(UPGRADE);
    count = 0;
    
    /* if the page erases then update our event counters */
    if( erasePage( getPageBaseAddress(), (FLASH_SECTORS)auditMgr.currentSector ) )
    {
        if( auditMgr.numOfCalRecords != 0 )
        {
            count = calibrationEvents;
            auditMgr.numOfCalRecords -= calibrationEvents;
        }
        if( auditMgr.numOfConfigRecords != 0 )
        {
            count += configurationEvents;
            auditMgr.numOfConfigRecords -= configurationEvents;
        }
        if( auditMgr.numOfUpgradeRecords != 0 )
        {
            count += upgradeEvents;
            auditMgr.numOfUpgradeRecords -= upgradeEvents;          
        }
        auditMgr.numOfCalRecords -= count;
        
        result = true;
    }
    else
    {
        PRINTF( "eraseAuditPage(): failed to erase cat3 audit page %#04x in serial flash!\r\n",  addr );       
    }
    return result;
}

/*! ****************************************************************************   
      \fn unsigned long findEmptyRcd()                                                              
      \public
      \brief
         This function scans the entire audit range of the serial flash for an 
         empty record. 
      \return address of empty record  
      \author
          Aaron Swift
*******************************************************************************/ 
unsigned long findEmptyRcd()
{
    /* start at the beginning of the audit area and read one page at a time 
       until and empty record is found or the end of the audit region is 
       reached. */
    unsigned int i = 0;
    unsigned long addr = SECTOR0_BASE_ADDR;    
    bool blank = false; 
   
    while( addr <= SECTOR2_BASE_ADDR)
    {
        i = 0;
        
        /* read the entire page */
        readAuditPage(addr);
        
        /* is the page blank? */
        if( !page.blank )
        {
            /* determine if page has an empty record */
            while( i <= PAGE_SIZE / sizeof(AuditRecord) )
            {
                /* is the record blank? */
                if( page.records[i].recordTag == BLANK_RECORD )
                {
                    blank = true;
                    /* adjust the address to our current position */
                    addr += i * sizeof(AuditRecord);
                    return addr;
                }
                else
                {
                    i++;
                }
            }
        
            if( !blank )
            {            
                /* index to the next page */
                addr += PAGE_SIZE;
            }
        }
        else
        {
            break;
        }
    }
    return addr;
}

/*! ****************************************************************************   
      \fn bool isInRange(unsigned long addr)                                                            
      \public
      \brief
         This function determines if address is within the bounds of the 
         audit region of the serial flash. 

      \return true if address is withing the range.  
      \author
          Aaron Swift
*******************************************************************************/ 
bool isInRange(unsigned long addr)
{
    if( addr < SECTOR2_BASE_ADDR )
    {
        return true;
    }
    else 
    {
        return false;
    }
}

/*! ****************************************************************************   
      \fn unsigned long get auditStartAddress()                                                          
      \public
      \brief
         This function returns the starting address of the Cat3 audit trail. 
          
      \return unsigned long  
      \author
          Aaron Swift
*******************************************************************************/ 
unsigned long getAuditStartAddress()
{
    return SECTOR0_BASE_ADDR;
}

/*! ****************************************************************************   
      \fn static unsigned long getSectorNumber(unsigned long addr)                                                          
      \private
      \brief
         This function returns the section number for the given address. 
          
      \return unsigned long  
      \author
          Aaron Swift
*******************************************************************************/ 
static unsigned char getSectorNumber(unsigned long addr)
{
    FLASH_SECTORS sector;

    if (addr < SECTOR1_BASE_ADDR)
    {
        sector = SECTOR_0;
    }
    else if ( ( addr >= SECTOR1_BASE_ADDR ) && (addr < SECTOR2_BASE_ADDR) )
    {
        sector = SECTOR_1;      
    }
    else if ( ( addr >= SECTOR2_BASE_ADDR ) && (addr < SECTOR3_BASE_ADDR) )
    {
        sector = SECTOR_2;      
    }
    else
    {
        sector = SECTOR_3;
    }
    return (unsigned char)sector;
}

/*! ****************************************************************************   
      \fn static unsigned char getPageNumber(unsigned long addr)                                                      
      \private
      \brief
         This function returns the page number for the given address. 
          
      \return unsigned long  
      \author
          Aaron Swift
*******************************************************************************/ 
static unsigned char getPageNumber(unsigned long addr)
{
    unsigned char page;
   

    /* determine which section we are in */
    unsigned char sector = getSectorNumber(addr);
    if( sector == SECTOR_1 )
    {
        addr -= SECTOR1_BASE_ADDR;
    }
    else if(sector == SECTOR_2)
    {
        addr -= SECTOR2_BASE_ADDR;
    }
    else if(sector == SECTOR_3)
    {
        addr -= SECTOR3_BASE_ADDR;
    }
    /* determine which page we are in */
    if( ( addr >= ( PAGE_SIZE * 0 ) ) &&  ( addr < ( PAGE_SIZE * 1 )) )
    {
        page = 0;
    }
    else if( ( addr >= ( PAGE_SIZE * 1 ) ) &&  ( addr < ( PAGE_SIZE * 2 )) )
    {
        page = 1;
    }
    else if( ( addr >= ( PAGE_SIZE * 2 ) ) &&  ( addr < ( PAGE_SIZE * 3 )) )
    {
        page = 2;
    }
    else if( ( addr >= ( PAGE_SIZE * 3 ) ) &&  ( addr < ( PAGE_SIZE * 4 )) )
    {
        page = 3;
    }
    else if( ( addr >= ( PAGE_SIZE * 4 ) ) &&  ( addr < ( PAGE_SIZE * 5 )) )
    {
        page = 4;
    }
    else if( ( addr >= ( PAGE_SIZE * 5 ) ) &&  ( addr < ( PAGE_SIZE * 6 )) )
    {
        page = 5;
    }
    else if( ( addr >= ( PAGE_SIZE * 6 ) ) &&  ( addr < ( PAGE_SIZE * 7 )) )
    {
        page = 6;
    }
    else
    {
        page = 7;
    }
    return page;
}

/*! ****************************************************************************   
      \fn static unsigned long getPageBaseAddress()                                                   
      \private
      \brief
         This function returns the base address for the current page the 
         audit manager is working in. 
          
      \return unsigned long base address for the current page 
      \author
          Aaron Swift
*******************************************************************************/ 
static unsigned long getPageBaseAddress()
{
    unsigned long baseAddress;
    baseAddress = ( auditMgr.currentSector * SECTOR_SIZE ) + 
                  ( auditMgr.currentPage * PAGE_SIZE ); 
  return baseAddress;
}

/*! ****************************************************************************   
      \fn unsigned short getNumberOfRecords(AuditEvent event)                                                   
      \public
      \brief
         This function returns the number of records for a given event from the 
         entire audit trail. 
      \return unsigned short 
      \author
          Aaron Swift
*******************************************************************************/ 
unsigned short getNumberOfRecords(AuditEvent event)
{
    AuditRecord record;
    unsigned long addr, endAddr; 
    unsigned short count = 0;
    /* get the address of the first available record */
    endAddr = findEmptyRcd();
    /* is the address in bounds of our audit trail section of the serial flash? */
    if( isInRange(endAddr) )
    {
        /* get the start of our audit trail section of the serial flash */
        addr = getAuditStartAddress();
        while( addr < endAddr ) 
        {
            readSerialAuditRcd(addr, &record); 
            addr += sizeof(AuditRecord);
            if( record.event == event )
            {
                count++;
            }
        }
    }
    else
    {
        PRINTF( "getNumberOfRecords(): end address out of range: %#04x!\r\n",  endAddr );       
    }
    return count;
}

/*! ****************************************************************************   
      \fn unsigned short getNumberOfRecordsPage(AuditEvent event)                                                   
      \public
      \brief
         This function returns the number of records for a given event from the 
         from the given page number. 
      \return unsigned short 
      \author
          Aaron Swift
*******************************************************************************/ 
unsigned short getNumberOfRecordsPage(AuditEvent event)
{
    AuditRecord record;
    unsigned long addr, endAddr; 
    unsigned short count = 0;

    /* get the start of our audit trail page section of the serial flash */
    addr = getPageBaseAddress();

    /* get the address of the end of the page*/
    endAddr = addr + PAGE_SIZE;
    /* is the address in bounds of our audit trail section of the serial flash? */
    if( isInRange(endAddr) )
    {
        while( addr <= endAddr ) 
        {
            readSerialAuditRcd(addr, &record); 
            addr += sizeof(AuditRecord);
            if( record.event == event )
            {
                count++;
            }
        }
    }
    else
    {
        PRINTF( "getNumberOfRecordsPage(): end address out of range: %#04x!\r\n",  endAddr );       
    }
    return count; 
}

/*! ****************************************************************************   
      \fn bool isRecordVaild(AuditRecord *pRecord)                                                  
      \public
      \brief
         This function returns true if record checksun if equal to the recorded
         sum.
      \param AuditRecord *pRecord pointer to record

      \return bool true saved checksum matches calculated sum
      \author
          Aaron Swift
*******************************************************************************/ 
bool isRecordVaild(AuditRecord *pRecord)
{
    unsigned long calcSum = calcRecordCheckSum(pRecord);    
    if( calcSum == pRecord->checksum )
    {
        return true;
    }
    else
    {
        return false;
    }
}

/*! ****************************************************************************   
      \fn unsigned long calcRecordCheckSum(AuditRecord *pRecord)                                                  
      \public
      \brief
         This function returns the byte sum of the record. 
      \param AuditRecord *pRecord pointer to record

      \return unsigned long sum value of record
      \author
          Aaron Swift
*******************************************************************************/ 
unsigned long calcRecordCheckSum(AuditRecord *pRecord)
{
    int i = 0;
    unsigned long sum = 0;
    unsigned char *pChar = NULL; 
    
    if( pRecord != NULL )
    {
        pChar = (unsigned char *)pRecord;
        for( i = 0; i < ( sizeof(AuditRecord) - sizeof(unsigned long) ); i++ )
        {
            sum += *pChar++;
        }
    }
    return sum;
}


void unitCat3TestRollover()
{
    /* setup rollover all but 10 records filled */   
    unsigned long addr = getAuditStartAddress();
    AuditRecord aRecord;
    AuditRecord bRecord;
    int j = 0, i = 0;
    
    readAuditPage(0);   

    eraseSector( 0, SECTOR_0 );    
    eraseSector( 0, SECTOR_1 );    
    readAuditPage(0);   
   
    /* reset the audit manager */
    auditMgr.initialized = 1;
    /* set the starting address */
    auditMgr.addr = addr;
    /* get the current serial flash sector */
    auditMgr.currentSector = getSectorNumber(addr);
    /* get the current page within the current sector */
    auditMgr.currentPage = getPageNumber(addr);
    
    /*get the number of calibration and configuration records from 
     the audit trail. */
    auditMgr.numOfCalRecords = 0;
    auditMgr.numOfConfigRecords = 0;
    auditMgr.numOfUpgradeRecords = 0;
    auditMgr.numOfTotalRecords = 0;
    auditMgr.numOfConfigRecords = 0;
  
    auditMgr.centerOfMainCounter        = 0;
    auditMgr.divisionSizeCounter        = 0;
    auditMgr.filterSpeedCounter         = 0;
    auditMgr.gainCoefficientCounter     = 0;
    auditMgr.gainFactorCounter          = 0;
    auditMgr.maxWeightCounter           = 0;
    auditMgr.minWeightPrintCounter      = 0;
    auditMgr.offsetCoefficientCounter   = 0;
    auditMgr.weigherModelCounter        = 0;
    auditMgr.zeroReferenceCounter       = 0;
    auditMgr.weigherModeCounter         = 0;
    
    writeAuditMGR( &auditMgr );
    
    /* create a fake record */
    aRecord.recordTag       = 0;
    aRecord.event           = CONFIGURATION;
    aRecord.parameterId     = DIVISION_SIZE;
    aRecord.epochTime       = 0x12345678;
         
    aRecord.oldParamValue   = 1;
    aRecord.newParamValue   = 2;
    aRecord.checksum        = calcRecordCheckSum(&aRecord);
 
    bRecord.recordTag       = 0;
    bRecord.event           = CALIBRATION;
    bRecord.parameterId     = GAIN_FACTOR;

    bRecord.oldParamValue   = 222;
    bRecord.newParamValue   = 333;
    bRecord.epochTime       = 0x87654321;
    
    bRecord.checksum = calcRecordCheckSum(&bRecord);
    
    j = 0;
    /* fill all 15 audit pages with fake records */
    while( j < 14 )
    {
        /* fill the entire page with records */
        for(i = 0; i < PAGE_SIZE / sizeof(AuditRecord); i++)
        {
            if(i % 2 == 0)
            {
                writeSerialAuditRcd(addr, &bRecord); 
            }
            else
            {
              writeSerialAuditRcd(addr, &aRecord); 
            }
            addr += sizeof(AuditRecord);
        }
        j++;
    }
 
    /* fill the last page with records except for last 2 */
    while( addr < SECTOR2_BASE_ADDR - (sizeof(AuditRecord) * 2) )
    {
        i++;
        if(i % 2 == 0)
        {
            writeSerialAuditRcd(addr, &bRecord); 
        }
        else
        {
          writeSerialAuditRcd(addr, &aRecord); 
        }
        addr += sizeof(AuditRecord);
    }
    readAuditPage(0);   

    addr = findEmptyRcd();
    isInRange(addr);

    readAuditPage(0);   
  
}
void unitTestResetCat3AuditLog()
{
    /* get starting address of the audit log  */   
    unsigned long addr = getAuditStartAddress();

    eraseSector( 0, SECTOR_0 );    
    eraseSector( 0, SECTOR_1 );    
    readAuditPage(0);   
   
    /* reset the audit manager */
    auditMgr.initialized = 1;
    /* set the starting address */
    auditMgr.addr = addr;
    /* get the current serial flash sector */
    auditMgr.currentSector = getSectorNumber(addr);
    /* get the current page within the current sector */
    auditMgr.currentPage = getPageNumber(addr);
    
    /*get the number of calibration and configuration records from 
     the audit trail. */
    auditMgr.numOfCalRecords = 0;
    auditMgr.numOfConfigRecords = 0;
    auditMgr.numOfUpgradeRecords = 0;
    auditMgr.numOfTotalRecords = 0;
    auditMgr.numOfConfigRecords = 0;
  
    auditMgr.centerOfMainCounter        = 0;
    auditMgr.divisionSizeCounter        = 0;
    auditMgr.filterSpeedCounter         = 0;
    auditMgr.gainCoefficientCounter     = 0;
    auditMgr.gainFactorCounter          = 0;
    auditMgr.maxWeightCounter           = 0;
    auditMgr.minWeightPrintCounter      = 0;
    auditMgr.offsetCoefficientCounter   = 0;
    auditMgr.weigherModelCounter        = 0;
    auditMgr.zeroReferenceCounter       = 0;
    auditMgr.weigherModeCounter         = 0;
    
    writeAuditMGR( &auditMgr );
  
}

void readSectorTest()
{
    unsigned long addr = getAuditStartAddress();
    addr = findEmptyRcd();
    //addr += PAGE_SIZE;
    addr = SECTOR1_BASE_ADDR;
    while( addr < SECTOR2_BASE_ADDR)
    {
        readAuditPage(addr);
        addr += PAGE_SIZE;
    }
}

void unitCat3Test()
{
#if 0
    //initializeAuditMGR();
    AuditMGR *pAuditMGR = NULL;
                    /* Audit manager section is blank, initialize the flash */
                    auditMgr.initialized = 1;
                    /* set the starting address */
                    auditMgr.addr = 0;
                    /* get the current serial flash sector */
                    auditMgr.currentSector = 0;
                    /* get the current page within the current sector */
                    auditMgr.currentPage = 0;
                    
                    /*get the number of calibration and configuration records from 
                     the audit trail. */
                    auditMgr.numOfCalRecords = 0;       //getNumberOfRecords(CALIBRATION);
                    auditMgr.numOfConfigRecords = 0;    //getNumberOfRecords(CONFIGURATION);
                    auditMgr.numOfUpgradeRecords = 0;   //getNumberOfRecords(UPGRADE);
                    auditMgr.numOfTotalRecords = 0;     //auditMgr.numOfCalRecords + 
                                                        //auditMgr.numOfConfigRecords + auditMgr.numOfUpgradeRecords;
                  
                    auditMgr.centerOfMainCounter        = 0;
                    auditMgr.divisionSizeCounter        = 0;
                    auditMgr.filterSpeedCounter         = 0;
                    auditMgr.gainCoefficientCounter     = 0;
                    auditMgr.gainFactorCounter          = 0;
                    auditMgr.maxWeightCounter           = 0;
                    auditMgr.minWeightPrintCounter      = 0;
                    auditMgr.offsetCoefficientCounter   = 0;
                    auditMgr.weigherModelCounter        = 0;
                    auditMgr.zeroReferenceCounter       = 0;
                    auditMgr.weigherModeCounter         = 0;
                    
                   
    extern void setupTransfer( FLASH_CMD command, SERIAL_FLASH_CNTRL *pSerialFlash, unsigned char *pAddr, unsigned char *pData, int length );
    extern SERIAL_FLASH_CNTRL serialFlash;
    /*verify the write enable is set */
    setupTransfer(FLASH_READ_STATUS_REG, &serialFlash, NULL, NULL, 0);
    processCommand(&serialFlash);    
    if( (serialFlash.statusRegister & WRITE_ENABLE_MASK) != WRITE_ENABLE_MASK )
    {
        setupTransfer( FLASH_WRITE_EN, &serialFlash, NULL, NULL, 0 );
        processCommand( &serialFlash );              
    } 

    /*check that the write enable is set */
    setupTransfer(FLASH_READ_STATUS_REG, &serialFlash, NULL, NULL, 0);
    processCommand(&serialFlash);
    if( ( serialFlash.statusRegister & WRITE_ENABLE_MASK ) !=  WRITE_ENABLE_MASK)
    {      
       setupTransfer(FLASH_WRITE_EN, &serialFlash, NULL, NULL, 0);
       processCommand(&serialFlash);
    }       

    /* program the serial flash */
    setupTransfer( FLASH_PAGE_PROGRAM, &serialFlash,  (unsigned char *)65536, &auditMgr, 3 + sizeof(AuditMGR) );
    if( serialFlash.error == 0 )
    {
        processCommand( &serialFlash );
        if( serialFlash.error == 0 )
        {
           
        }          
    }
    readSerialFlash( SECTOR2_BASE_ADDR, sizeof(AuditMGR) );
    if( pAuditMGR != NULL ) {
        erasePage( SECTOR2_BASE_ADDR, SECTOR_2 );
    }
#endif    
#if 0
    AuditRecord bRecord; 
    AuditRecord firstRecord;
    /* create another fake record */
    bRecord.recordTag       = 0;
    bRecord.event           = CALIBRATION;
    bRecord.parameterId     = GAIN_FACTOR;

    bRecord.oldParamValue   = 222;
    bRecord.newParamValue   = 333;
    bRecord.checksum        = 4567;
    
    //memcpy( &bRecord.dateOfChange[0], &bMfgDate[0], DATE_CHANGE_SIZE );
    //memcpy( &bRecord.timeOfChange[0], &bMfgTime[0], TIME_CHANGE_SIZE );     
    readAuditPage(0);
    readRecord(&firstRecord , 0);
    if( isRecordVaild(&firstRecord ) )
        asm("nop");
     else 
        writeSerialAuditRcd(0, &bRecord); 
    //addr += sizeof(AuditRecord);
#endif 
#if 1  
    int a = 0;
    eraseSector( 0, SECTOR_0 );    
    eraseSector( 0, SECTOR_1 );
    erasePage( SECTOR2_BASE_ADDR, SECTOR_2 );
    while(1)
    {
      a++;
    }
    //readSerialFlash( SECTOR2_BASE_ADDR, sizeof(AuditMGR)
#endif  
#if 0   
    unsigned long addr = getAuditStartAddress();
    unsigned char mfgDate[DATE_CHANGE_SIZE] = {'1','0','/','0','8','/','2','0','1','2'};
    unsigned char mfgTime[TIME_CHANGE_SIZE] = {'1','2',':','5','9','P','M'};
    unsigned char bMfgDate[DATE_CHANGE_SIZE] = {'0','6','/','1','8','/','2','0','1','6'};
    unsigned char bMfgTime[TIME_CHANGE_SIZE] = {'0','5',':','1','9','P','M'};
    unsigned int i, j;

    //AuditPage page;    
    AuditRecord aRecord;
    AuditRecord bRecord;
    AuditRecord rRecord;
    AuditRecord sRecord;
    
    /* create a fake record */
    aRecord.recordTag       = 0;
    aRecord.event           = CONFIGURATION;
    aRecord.parameterId     = WEIGHER_TYPE;
    
    memcpy( &aRecord.dateOfChange[0], &mfgDate[0], DATE_CHANGE_SIZE );
    memcpy( &aRecord.timeOfChange[0], &mfgTime[0], TIME_CHANGE_SIZE );     
         
    aRecord.oldParamValue   = 1;
    aRecord.newParamValue   = 2;
    aRecord.checksum        = calcRecordCheckSum(&aRecord);
 
    bRecord.recordTag       = 0;
    bRecord.event           = CALIBRATION;
    bRecord.parameterId     = GAIN_FACTOR;

    bRecord.oldParamValue   = 222;
    bRecord.newParamValue   = 333;
    
    memcpy( &bRecord.dateOfChange[0], &bMfgDate[0], DATE_CHANGE_SIZE );
    memcpy( &bRecord.timeOfChange[0], &bMfgTime[0], TIME_CHANGE_SIZE );     
    
    bRecord.checksum = calcRecordCheckSum(&bRecord);

    eraseSector( 0, SECTOR_0 );
    eraseSector( 0, SECTOR_1 );
#endif
#if 0    
    /* erase the Audit manager */

    readAuditPage(0);
    eraseSector( 0, SECTOR_0 );
    resetAuditMGR();

    readAuditPage(0);
    //eraseSector( 0, SECTOR_1 );
#endif
    
#if 0
 
    j = 0;
    
    while( j < 16 )
    {
      
   
        /* fill the entire page with records */
        for(i = 0; i < PAGE_SIZE / sizeof(AuditRecord); i++)
        {
            if(i % 2 == 0)
            {
                writeSerialAuditRcd(addr, &bRecord); 
            }
            else
            {
              writeSerialAuditRcd(addr, &aRecord); 
            }
            addr += sizeof(AuditRecord);
        }
        j++;
    }
    
    /* read the entire page */
    readAuditPage(0);
    readAuditPage(PAGE_SIZE);
    readAuditPage(PAGE_SIZE + PAGE_SIZE);
    readAuditPage(PAGE_SIZE + PAGE_SIZE + PAGE_SIZE);
    readAuditPage(PAGE_SIZE + PAGE_SIZE + PAGE_SIZE + PAGE_SIZE);
    readAuditPage(PAGE_SIZE + PAGE_SIZE + PAGE_SIZE + PAGE_SIZE + PAGE_SIZE );
    readAuditPage(PAGE_SIZE + PAGE_SIZE + PAGE_SIZE + PAGE_SIZE + PAGE_SIZE + PAGE_SIZE);
    readAuditPage(PAGE_SIZE + PAGE_SIZE + PAGE_SIZE + PAGE_SIZE + PAGE_SIZE + PAGE_SIZE + PAGE_SIZE);
#endif

#if 0    
    /* erase the page */
    erasePage(addr, SECTOR_0); 
    /* erase the page */
    erasePage(addr + PAGE_SIZE, SECTOR_0); 
     /* erase the page */
    erasePage(addr + PAGE_SIZE + PAGE_SIZE, SECTOR_0); 
   
    /* read the entire page */
    readAuditPage(addr);
    
    writeSerialAuditRcd(addr, &aRecord); 
    
    /* read the entire page */
    readAuditPage(addr);

    addr += sizeof(AuditRecord);
  
    /* create another fake record */
    bRecord.recordTag       = 0;
    bRecord.event           = CALIBRATION;
    bRecord.parameterId     = GAIN_FACTOR;

    bRecord.oldParamValue   = 222;
    bRecord.newParamValue   = 333;
    bRecord.checksum        = 4567;
    
    memcpy( &bRecord.dateOfChange[0], &bMfgDate[0], DATE_CHANGE_SIZE );
    memcpy( &bRecord.timeOfChange[0], &bMfgTime[0], TIME_CHANGE_SIZE );     
    
    writeSerialAuditRcd(addr, &bRecord); 
    addr += sizeof(AuditRecord);
    
    /* add 50 additional records 
    for(i = 0; i < 50; i++)
    {
        if(i % 2 == 0)
        {
            writeSerialAuditRcd(addr, &aRecord); 
        }
        else
        {
          writeSerialAuditRcd(addr, &bRecord); 
        }
        addr += sizeof(AuditRecord);
    }
    */
    /* reset the audit record counter */
    addr = getAuditStartAddress();

    /* read the entire page */
    readAuditPage(addr);

    /* reset the audit record counter */
    addr = getAuditStartAddress();
    readSerialAuditRcd(addr, &rRecord);
    /* TO DO: this function is broken will need fixed
    auditMgr.pCurrentRecord = readAuditPage( auditMgr.auditSecAddr ); */
    addr += sizeof(AuditRecord);
    readSerialAuditRcd(addr, &rRecord);
    addr += sizeof(AuditRecord);
    readSerialAuditRcd(addr, &rRecord);
    addr += sizeof(AuditRecord);
    readSerialAuditRcd(addr, &rRecord);
    addr += sizeof(AuditRecord);
    readSerialAuditRcd(addr, &rRecord);
    addr += sizeof(AuditRecord);
    
    /* reset the audit record counter */
    addr = getAuditStartAddress();
    
    /* read the whole page of audit records */
    readAuditPage(addr);

    addr += sizeof(AuditRecord);

    /* find the first empty record */
    addr = findEmptyRcd();
    readSerialAuditRcd(addr, &sRecord);
    
    /* erase the page */
    erasePage(addr, SECTOR_0);
    
    /*reset the address to the start of the audit page*/
    addr = getAuditStartAddress();
    
    /* read the whole page of audit records */
    readAuditPage(addr);

    /* fill the entire page with records */
    for(i = 0; i < PAGE_SIZE / sizeof(AuditRecord); i++)
    {
        if(i % 2 == 0)
        {
            writeSerialAuditRcd(addr, &bRecord); 
        }
        else
        {
          writeSerialAuditRcd(addr, &aRecord); 
        }
        addr += sizeof(AuditRecord);
    }

    /*reset the address to the start of the audit page*/
    addr = getAuditStartAddress();

    /* read the whole page of audit records */
    readAuditPage(addr);

    addr = findEmptyRcd();
    if( isInRange(addr) )
    {
        /* read the whole page of audit records */
        readAuditPage(addr);
        /* read the 2nd page of audit records to make sure it's blank */
        readAuditPage(addr + PAGE_SIZE);
      
    }
    else
    {
       sRecord.recordTag = 1;
    }
#endif   
    

}