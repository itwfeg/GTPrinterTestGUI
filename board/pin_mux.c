/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v8.0
processor: MIMXRT1024xxxxx
package_id: MIMXRT1024DAG5A
mcu_data: ksdk2_0
processor_version: 0.0.0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_gpio.h"     
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 * 
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 * 
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void) {
    BOARD_InitPins();

/* GPIO_AD_B1_00~GPIO_AD_B1_05 can only be configured as flexspi function. Note that it can't be modified here */
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_00_FLEXSPI_A_DATA03,1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_01_FLEXSPI_A_SCLK,1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_02_FLEXSPI_A_DATA00,1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_03_FLEXSPI_A_DATA02,1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_04_FLEXSPI_A_DATA01,1U);
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_05_FLEXSPI_A_SS0_B,1U);
}

void BOARD_InitPins(void) {
    CLOCK_EnableClock( kCLOCK_Iomuxc );         
    CLOCK_EnableClock( kCLOCK_Xbar1 );

    CLOCK_EnableClock( kCLOCK_Lpspi1 ); 
    CLOCK_EnableClock( kCLOCK_Lpspi2 ); 
    CLOCK_EnableClock( kCLOCK_Lpspi3 ); 
    CLOCK_EnableClock( kCLOCK_Lpspi4 ); 
     
    /*set clock source for lpspi1*/
    CLOCK_SetMux( kCLOCK_LpspiMux, 1 );
    CLOCK_SetDiv( kCLOCK_LpspiDiv, 7 );
    
    /*set clock source for lpspi2 */
    CLOCK_SetMux(kCLOCK_LpspiMux, 2);
    CLOCK_SetDiv(kCLOCK_LpspiDiv, 4);
   
    CLOCK_EnableClock( kCLOCK_Adc1 );
    CLOCK_EnableClock( kCLOCK_Adc2 );

    
/******************************************************************************/    
/************************* rt1024-rockchip board ******************************/
/********************* sbc-rk3568-nxp24-ark rev 1.0 ***************************/
/******************************************************************************/        
    /* global board */
    gpio_pin_config_t outCfg, inCfg;
    
    outCfg.direction            = kGPIO_DigitalOutput;
    outCfg.outputLogic          = 1U;
    outCfg.interruptMode        = kGPIO_NoIntmode;
    
    inCfg.direction             = kGPIO_DigitalInput;    
    inCfg.interruptMode         = kGPIO_NoIntmode;
  
/***************************** rev 3 signals **********************************/
    //IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_29_FLEXPWM2_PWMB03, 0U ); //stepper enable PWM      

    /* printhead type pin, 80mm vs 72 */
    GPIO_PinInit( GPIO1, 15U, &inCfg );
    IOMUXC_SetPinConfig( IOMUXC_GPIO_AD_B0_15_GPIO1_IO15, 0x70A0U );
    IOMUXC_SetPinMux( IOMUXC_GPIO_AD_B0_15_GPIO1_IO15, 0U );                                    
    
    /* stepper current limit set A */
    GPIO_PinInit(GPIO3, 5U, &outCfg);
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_37_GPIO3_IO05, 0x70A0U );
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_37_GPIO3_IO05, 0U );  
    
    /* stepper current limit set B */
    GPIO_PinInit(GPIO3, 7U, &outCfg);
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_39_GPIO3_IO07, 0x70A0U ); 
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_39_GPIO3_IO07, 0U); 
    
    /* stepper enable */
    IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_29_GPIO2_IO29, 0U);
    gpio_pin_config_t enableConfig = { kGPIO_DigitalOutput, 0 };
    GPIO_PinInit( MOTOR_EN_GPIO, MOTOR_EN_PIN, &enableConfig );
    
    /* IDF 1 and 2 */

    GPIO_PinInit(GPIO1, 7U, &inCfg);
    GPIO_PinInit(GPIO1, 8U, &inCfg);
    
    IOMUXC_SetPinConfig( IOMUXC_GPIO_AD_B0_07_GPIO1_IO07, 0x70A0U );
    IOMUXC_SetPinConfig( IOMUXC_GPIO_AD_B0_08_GPIO1_IO08, 0x70A0U );

    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_07_GPIO1_IO07, 0U);                                    
    IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_08_GPIO1_IO08, 0U); 
   
    
/*************************** print head signals *******************************/
#if 1
    /* print head lpspi4 clk group3 pin0 configured as lpspi4 clk */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_32_LPSPI4_SCK, 0U );
    IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_32_LPSPI4_SCK, 0x10B0U ); 

    /* print head lpspi4 mosi group3 pin3 configured as lpspi4 mosi ***** miso / mosi switch on schematic! ***** */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_34_LPSPI4_SDO, 0U );
    IOMUXC_SetPinConfig(IOMUXC_GPIO_EMC_34_LPSPI4_SDO, 0x10B0U ); 
    
    /* print head latch enable group2 pin1 configured as ouput 
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_01_GPIO2_IO01, 0U );
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_01_GPIO2_IO01, 0x70A0U ); 
    GPIO_PinInit( PHEAD_STROBE_EN_GPIO, PHEAD_STROBE_EN_PIN, &outCfg );
    */
    
    outCfg.outputLogic          = 1U;
    
    /* print head latch group2 pin0 configured as ouput */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_00_GPIO2_IO00, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_00_GPIO2_IO00, 0x70A0U ); 
    GPIO_PinInit( PHEAD_LATCH_GPIO, PHEAD_LATCH_PIN, &outCfg );
    
    //outCfg.outputLogic          = 1U; //these strobe pins need to be muxed based on printhead type at some point
    
    outCfg.outputLogic          = 1U;
    
    /* print strobe a group3 pin6 configured as ouput */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_38_GPIO3_IO06, 1U );                                                
    //IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_38_FLEXPWM2_PWMA00, 0x10B0U ); 
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_38_GPIO3_IO06, 0x70A0U ); 
    GPIO_PinInit( PHEAD_STROBE_A_GPIO, PHEAD_STROBE_A_PIN, &outCfg );

    /* print strobe b group3 pin22 configured as ouput */
    IOMUXC_SetPinMux( IOMUXC_GPIO_SD_B1_02_GPIO3_IO22, 1U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_SD_B1_02_GPIO3_IO22, 0x70A0U ); 
    GPIO_PinInit( PHEAD_STROBE_B_GPIO, PHEAD_STROBE_B_PIN, &outCfg );
    
    outCfg.outputLogic          = 0U;
    
    /* print power enable group2 pin4 configured as ouput */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_04_GPIO2_IO04, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_04_GPIO2_IO04, 0x70A0U ); 
    GPIO_PinInit( PHEAD_POWER_EN_GPIO, PHEAD_POWER_EN_PIN, &outCfg );

    /* print head dot enable group2 pin5 configured as ouput */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_05_GPIO2_IO05, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_05_GPIO2_IO05, 0x70A0U ); 
    GPIO_PinInit( PHEAD_DOT_ENABLE_GPIO, PHEAD_DOT_ENABLE_PIN, &outCfg );
    
    outCfg.outputLogic          = 1U;
 

#endif
/****************************** motor signals *********************************/
    
#if 1
    /* main motor sleep group3 pin4 configured as ouput */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_36_GPIO3_IO04, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_36_GPIO3_IO04, 0x70A0U ); 
    GPIO_PinInit( MAIN_MOTOR_SLEEP_GPIO, MAIN_MOTOR_SLEEP_PIN, &outCfg );
    
    /* main motor ms0 group2 pin25 configured as ouput */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_25_GPIO2_IO25, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_25_GPIO2_IO25, 0x70A0U );   
    GPIO_PinInit( MAIN_MOTOR_MS0_GPIO, MAIN_MOTOR_MS0_PIN, &outCfg );
    
    /* main motor ms1 group2 pin27 configured as ouput */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_27_GPIO2_IO27, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_27_GPIO2_IO27, 0x70A0U ); 
    GPIO_PinInit( MAIN_MOTOR_MS1_GPIO, MAIN_MOTOR_MS1_PIN, &outCfg );

    /* main motor reset group2 pin28 configured as ouput */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_28_GPIO2_IO28, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_28_GPIO2_IO28, 0x70A0U );
    GPIO_PinInit( MAIN_MOTOR_RESET_GPIO, MAIN_MOTOR_RESET_PIN, &outCfg );

    /* main motor enable group2 pin29 configured as ouput */
    //IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_29_GPIO2_IO29, 1U );                                                
    //IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_29_GPIO2_IO29, 0x70A0U );   
    //GPIO_PinInit( MOTOR_EN_GPIO, MOTOR_EN_PIN, &outCfg );

    //IOMUXC_SetPinMux(IOMUXC_GPIO_EMC_29_GPIO2_IO29, 1U);
    //gpio_pin_config_t enableConfig = { kGPIO_DigitalOutput, 1, };
    //GPIO_PinInit( MOTOR_EN_GPIO, MOTOR_EN_PIN, &enableConfig );
    
    /* main motor direction group2 pin30 configured as ouput */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_30_GPIO2_IO30, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_30_GPIO2_IO30, 0x70A0U );   
    GPIO_PinInit( MAIN_MOTOR_DIR_GPIO, MAIN_MOTOR_DIR_PIN, &outCfg );

    /* main motor step group2 pin31 configured as ouput */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_31_GPIO2_IO31, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_31_GPIO2_IO31, 0x70A0U );       
    GPIO_PinInit( MAIN_MOTOR_STEP_GPIO, MAIN_MOTOR_STEP_PIN, &outCfg );

    
    /* takeup motor ms0 group2 pin16 configured as ouput */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_16_GPIO2_IO16, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_16_GPIO2_IO16, 0x70A0U );
    GPIO_PinInit( TAKEUP_MOTOR_MS0_GPIO, TAKEUP_MOTOR_MS0_PIN, &outCfg );

    /* takeup motor ms1 group2 pin17 configured as ouput */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_17_GPIO2_IO17, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_17_GPIO2_IO17, 0x70A0U ); 
    GPIO_PinInit( TAKEUP_MOTOR_MS1_GPIO, TAKEUP_MOTOR_MS1_PIN, &outCfg );

    /* takeup motor reset group2 pin18 configured as ouput */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_18_GPIO2_IO18, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_18_GPIO2_IO18, 0x70A0U );
    GPIO_PinInit( TAKEUP_MOTOR_RESET_GPIO, TAKEUP_MOTOR_RESET_PIN, &outCfg );

    /* takeup motor enable group2 pin19 configured as ouput */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_19_GPIO2_IO19, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_19_GPIO2_IO19, 0x70A0U );
    GPIO_PinInit( TAKEUP_MOTOR_EN_GPIO, TAKEUP_MOTOR_EN_PIN, &outCfg );

    /* takeup motor direction group2 pin20 configured as ouput */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_20_GPIO2_IO20, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_20_GPIO2_IO20, 0x70A0U );
    GPIO_PinInit( TAKEUP_MOTOR_DIR_GPIO, TAKEUP_MOTOR_DIR_PIN, &outCfg );

    /* takeup motor step group2 pin21 configured as ouput */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_21_GPIO2_IO21, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_21_GPIO2_IO21, 0x70A0U );
    GPIO_PinInit( TAKEUP_MOTOR_STEP_GPIO, TAKEUP_MOTOR_STEP_PIN, &outCfg );

    /* takeup motor sleep group2 pin24 configured as ouput */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_24_GPIO2_IO24, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_24_GPIO2_IO24, 0x70A0U );
    GPIO_PinInit( TAKEUP_MOTOR_SLEEP_GPIO, TAKEUP_MOTOR_SLEEP_PIN, &outCfg );
    
#endif             
/****************************** misc signals **********************************/    
    
    IOMUXC_SetPinMux( IOMUXC_GPIO_SD_B1_05_FLEXSPI_A_DQS, 1U );
    IOMUXC_SetPinConfig( IOMUXC_GPIO_SD_B1_05_FLEXSPI_A_DQS, 0x10F1U );                           
    

    /* serial flash cs group3 pin16 configured as ouput */
    IOMUXC_SetPinMux( IOMUXC_GPIO_SD_B0_03_GPIO3_IO16, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_SD_B0_03_GPIO3_IO16, 0x70A0U );
    GPIO_PinInit( SERIAL_FlASH_CS_GPIO, SERIAL_FlASH_CS_PIN, &outCfg );
    
    #ifdef RK3568_ARK_REV_1_0_MOD    
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_39_FLEXPWM2_PWMB00, 0U ); 
    #else 
    
    #if 1   /* mine */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_09_XBAR1_INOUT09, 0U );                                                
    IOMUXC_GPR->GPR6 = ((IOMUXC_GPR->GPR6 &
      (~(IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_9_MASK))) 
        | IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_9(0x01U) 
      );
    XBARA_SetSignalsConnection( XBARA, kXBARB2_InputQtimer1Tmr0, kXBARA1_OutputIomuxXbarInout09 ); 
    

    /* label taken led group2 pin9 configured as ouput */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_09_XBAR1_INOUT09, 0U );                                                
    IOMUXC_GPR->GPR6 = ((IOMUXC_GPR->GPR6 &
      (~(IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_9_MASK))) 
        | IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_9(0x01U) 
      );
    XBARA_SetSignalsConnection( XBARA, kXBARB2_InputQtimer1Tmr0, kXBARA1_OutputIomuxXbarInout09 ); 

    #else
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_09_GPIO2_IO09, 0U );
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_09_GPIO2_IO09, 0x70A0U );
    
    outCfg.outputLogic          = 0U;
    /* label taken enable group3 pin31 configured as ouput */
    IOMUXC_SetPinMux( IOMUXC_GPIO_SD_B1_11_GPIO3_IO31, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_SD_B1_11_GPIO3_IO31, 0x1088U );
    GPIO_PinInit( LABEL_TAKEN_EN_GPIO, LABEL_TAKEN_EN_PIN, &outCfg );
    outCfg.outputLogic          = 1U;    
    #endif 
    
    outCfg.outputLogic          = 0U;
    /* label taken enable group3 pin31 configured as ouput */
    IOMUXC_SetPinMux( IOMUXC_GPIO_SD_B1_11_GPIO3_IO31, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_SD_B1_11_GPIO3_IO31, 0x1088U );
    GPIO_PinInit( LABEL_TAKEN_EN_GPIO, LABEL_TAKEN_EN_PIN, &outCfg );
    outCfg.outputLogic          = 1U;
    #endif
    

    /* lp5521 enable group2 pin7 configured as ouput */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_07_GPIO2_IO07, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_07_GPIO2_IO07, 0x70A0U ); //0xD0B0U
    GPIO_PinInit( LP5521_EN_GPIO, LP5521_EN_PIN, &outCfg );
    
    /* model a group3 pin17 configured as input */
    IOMUXC_SetPinMux( IOMUXC_GPIO_SD_B1_04_GPIO3_IO24, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_SD_B1_04_GPIO3_IO24, 0x70A0U );
    GPIO_PinInit( MODEL_TYPE_A_GPIO, MODEL_TYPE_A_PIN, &inCfg );

    /* model b group3 pin18 configured as input */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_08_GPIO2_IO08, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_08_GPIO2_IO08, 0x70A0U );
    GPIO_PinInit( MODEL_TYPE_B_GPIO, MODEL_TYPE_B_PIN, &inCfg );

    /* model c group3 pin19 configured as input */
    IOMUXC_SetPinMux( IOMUXC_GPIO_SD_B1_06_GPIO3_IO26, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_SD_B1_06_GPIO3_IO26, 0x70A0U );
    GPIO_PinInit( MODEL_TYPE_C_GPIO, MODEL_TYPE_C_PIN, &inCfg );

    /* model d group3 pin20 configured as input */
    IOMUXC_SetPinMux( IOMUXC_GPIO_SD_B1_07_GPIO3_IO27, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_SD_B1_07_GPIO3_IO27, 0x70A0U );
    GPIO_PinInit( MODEL_TYPE_D_GPIO, MODEL_TYPE_D_PIN, &inCfg );
  
    /* idf1 & 2 signals */
    IOMUXC_SetPinMux( IOMUXC_GPIO_AD_B0_08_GPIO1_IO08, 0U );
    IOMUXC_SetPinConfig( IOMUXC_GPIO_AD_B0_08_GPIO1_IO08, 0x70A0U );
    GPIO_PinInit( IDF1_GPIO, IDF1_PIN, &inCfg );
    
    IOMUXC_SetPinMux( IOMUXC_GPIO_AD_B0_09_GPIO1_IO09, 0U );
    IOMUXC_SetPinConfig( IOMUXC_GPIO_AD_B0_09_GPIO1_IO09, 0x70A0U );
    GPIO_PinInit( IDF2_GPIO, IDF2_PIN, &inCfg );
    
    /* heart beat led group3 pin14 configured as output */
    IOMUXC_SetPinMux( IOMUXC_GPIO_SD_B0_01_GPIO3_IO14, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_SD_B0_01_GPIO3_IO14, 0x70A0U );
    GPIO_PinInit( HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, &outCfg );

    /* logo led enable */
    IOMUXC_SetPinMux( IOMUXC_GPIO_SD_B1_08_GPIO3_IO28, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_SD_B1_08_GPIO3_IO28, 0x70A0U );
    GPIO_PinInit( LOGO_EN_LED_GPIO, LOGO_EN_LED_PIN, &outCfg );
    
    /* logo2 led enable */
    IOMUXC_SetPinMux( IOMUXC_GPIO_SD_B1_09_GPIO3_IO29, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_SD_B1_09_GPIO3_IO29, 0x70A0U );
    GPIO_PinInit( LOGO_EN1_LED_GPIO, LOGO_EN1_LED_PIN, &outCfg );

    IOMUXC_SetPinMux( IOMUXC_GPIO_SD_B1_10_GPIO3_IO30, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_SD_B1_10_GPIO3_IO30, 0x70A0U );
    GPIO_PinInit( LOGO_STATUS_LED_EN_GPIO, LOGO_STATUS_LED_EN_PIN, &outCfg );


    IOMUXC_SetPinMux( IOMUXC_GPIO_AD_B1_06_GPIO1_IO22, 0U );
    IOMUXC_SetPinConfig( IOMUXC_GPIO_SD_B1_10_GPIO3_IO30, 0x70A0U );
    GPIO_PinInit( STATUS_LED_GPIO, STATUS_LED_PIN, &outCfg );

/*************************** interface signals ********************************/    
    
    /* sram cs group3 pin1 configured as output */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_33_GPIO3_IO01, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_33_GPIO3_IO01, 0x70A0U );
    GPIO_PinInit( SRAM_CS_GPIO, SRAM_CS_PIN, &outCfg );

    /* sram lpspi4 miso group3 pin1 configured as lpspi4 miso ***** miso / mosi switch on schematic! ***** */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_35_LPSPI4_SDI, 0U );
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_35_LPSPI4_SDI, 0x10B0U ); 
    
    /* serial flash lpspi1 clk group3 pin15 configured as lpspi1 clk */
    IOMUXC_SetPinMux( IOMUXC_GPIO_SD_B0_02_LPSPI1_SCK, 0U );
    IOMUXC_SetPinConfig( IOMUXC_GPIO_SD_B0_02_LPSPI1_SCK, 0x10B0U ); 
    
    /* serial flash lpspi1 miso group3 pin17 configured as lpspi1 miso */
    IOMUXC_SetPinMux( IOMUXC_GPIO_SD_B0_04_LPSPI1_SDO, 0U );
    IOMUXC_SetPinConfig( IOMUXC_GPIO_SD_B0_04_LPSPI1_SDO, 0x10B0U ); 
    
    /* serial flash lpspi1 mosi group3 pin18 configured as lpspi1 mosi */
    IOMUXC_SetPinMux( IOMUXC_GPIO_SD_B0_05_LPSPI1_SDI, 0U );
    IOMUXC_SetPinConfig( IOMUXC_GPIO_SD_B0_05_LPSPI1_SDI, 0x10B0U ); 
    
    /* weigher lpspi2 clk group2 pin10 configured as lpspi2 clk */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_10_LPSPI2_SCK, 0U );
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_10_LPSPI2_SCK, 0x10B0U ); 
    
    /* weigher lpspi2 miso group2 pin12 configured as lpspi2 miso */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_12_LPSPI2_SDO, 0U );
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_12_LPSPI2_SDO, 0x10B0U ); 

   // IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_11_GPIO2_IO11, 0U );
   // IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_11_GPIO2_IO11, 0x70A0U ); 

    /* weigher lpspi2 mosi group2 pin13 configured as lpspi2 mosi */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_13_LPSPI2_SDI, 0U );
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_13_LPSPI2_SDI, 0x10B0U ); 
    
    
    
/***************************** comm signals ***********************************/    

    /* rs485 group1 pin10 configured as uart5 tx */
    IOMUXC_SetPinMux( IOMUXC_GPIO_AD_B0_10_LPUART5_TX, 0U );                                    
    IOMUXC_SetPinConfig( IOMUXC_GPIO_AD_B0_10_LPUART5_TX, 0x10B0U );                               

    /* rs485 group1 pin11 configured as uart5 rx */
    IOMUXC_SetPinMux( IOMUXC_GPIO_AD_B0_11_LPUART5_RX, 0U );                                    
    IOMUXC_SetPinConfig( IOMUXC_GPIO_AD_B0_11_LPUART5_RX, 0x10B0U ); 
#if 0
    
    /* can interface rx group3 pin21 configured as can rx */
    IOMUXC_SetPinMux( IOMUXC_GPIO_SD_B1_01_FLEXCAN1_RX, 0U );
    IOMUXC_SetPinConfig( IOMUXC_GPIO_SD_B1_01_FLEXCAN1_RX, 0x10B0U );

    /* can interface tx group3 pin20 configured as can tx */
    IOMUXC_SetPinMux( IOMUXC_GPIO_SD_B1_00_FLEXCAN1_TX, 0U );    
    IOMUXC_SetPinConfig( IOMUXC_GPIO_SD_B1_00_FLEXCAN1_TX, 0x10B0U ); 
    
#else 
    
    /* debug interface group3 pin21 configured as uart6 rx */
    IOMUXC_SetPinMux( IOMUXC_GPIO_SD_B1_01_LPUART6_RX, 0U );
    IOMUXC_SetPinConfig( IOMUXC_GPIO_SD_B1_01_LPUART6_RX, 0x10B0U );

    /* debug interface group3 pin20 configured as uart6 tx */
    IOMUXC_SetPinMux( IOMUXC_GPIO_SD_B1_00_LPUART6_TX, 0U );    
    IOMUXC_SetPinConfig( IOMUXC_GPIO_SD_B1_00_LPUART6_TX, 0x10B0U ); 

#endif    
    /* cutter group1 pin6 configured as uart1 tx */
    IOMUXC_SetPinMux( IOMUXC_GPIO_AD_B0_06_LPUART1_TX, 0U );                                    
    IOMUXC_SetPinConfig( IOMUXC_GPIO_AD_B0_06_LPUART1_TX, 0x10B0U ); 
    
    //IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_06_GPIO1_IO06, 0U); 
    //IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_06_GPIO1_IO06, 0x70A0U );  
    //GPIO_PinInit( GPIO1, 6U, &outCfg2 );  //debug pin for hung

    /* cutter group1 pin7 configured as uart1 rx */
    IOMUXC_SetPinMux( IOMUXC_GPIO_AD_B0_07_LPUART1_RX, 0U );                                    
    IOMUXC_SetPinConfig( IOMUXC_GPIO_AD_B0_07_LPUART1_RX, 0x10B0U );
   
    /* do not configure this output to enable the U3920 to switch the usb link! */
    /* usb pwr enable group3 pin9 configured as usb pwr 
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_41_USB_OTG1_PWR, 0U ); */   

    /* usb id group3 pin8 configured as usb id */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_40_USB_OTG1_ID, 0U );       

    /* debug group2 pin22 configured as uart2 tx */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_22_LPUART2_TX, 0U );                                    
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_22_LPUART2_TX, 0x10B0U );                               
    
    /* debug group2 pin23 configured as uart2 rx */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_23_LPUART2_RX, 0U );                                    
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_23_LPUART2_RX, 0x10B0U ); 

/***************************** sensor signals *********************************/    

    /* label taken group1 pin13 configured as input */
    IOMUXC_SetPinMux( IOMUXC_GPIO_AD_B0_13_GPIO1_IO13, 0U );                                                
    
    /* low label stock group1 pin14 configured as adc channel */
    IOMUXC_SetPinMux( IOMUXC_GPIO_AD_B0_14_GPIO1_IO14, 0U );                                    

    /* shoot through group1 pin23 configured as adc channel */
    IOMUXC_SetPinMux( IOMUXC_GPIO_AD_B1_07_GPIO1_IO23, 0U );                                    
    
    /* paper takeup group1 pin24 configured as adc channel */
    IOMUXC_SetPinMux( IOMUXC_GPIO_AD_B1_08_GPIO1_IO24, 0U );                                    

    /* print head temperature group1 pin25 configured as adc channel */
    IOMUXC_SetPinMux( IOMUXC_GPIO_AD_B1_09_GPIO1_IO25, 0U );                                    

    /* print head dot group1 pin27 configured as adc channel */
    IOMUXC_SetPinMux( IOMUXC_GPIO_AD_B1_11_GPIO1_IO27, 0U );                                    

    /* print head up group1 pin26 configured as input */
    IOMUXC_SetPinMux( IOMUXC_GPIO_AD_B1_10_GPIO1_IO26, 0U );     

    /* print head detection group1 pin15 configured as input */
    IOMUXC_SetPinMux( IOMUXC_GPIO_AD_B0_15_GPIO1_IO15, 0U );   
    
/***************************** weigher signals ********************************/

    /* ext weigher adc clk group2 pin26 configured as pwma0  */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_26_FLEXPWM1_PWMA00, 0U );                                    
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_26_FLEXPWM1_PWMA00, 0x10B0U );                                 
   
    /* ext weigher adc reset group3 pin5 configured as ouput */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_37_GPIO3_IO05, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_37_GPIO3_IO05, 0x70A0U );   
    GPIO_PinInit( WGR_RESET_GPIO, WGR_RESET_PIN, &outCfg );   
    
    /* weigher service switch group2 pin6 configured as input */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_06_GPIO2_IO06, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_06_GPIO2_IO06, 0x70A0U ); 
    inCfg.interruptMode         = kGPIO_IntFallingEdge;
    GPIO_PinInit( SERVICE_SWITCH_GPIO, SERVICE_SWITCH_PIN, &inCfg ); 
    
    /* weigher accelerometer int a group2 pin14 configured as input */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_14_GPIO2_IO14, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_14_GPIO2_IO14, 0x70A0U ); 
    outCfg.outputLogic          = 1U;   
    /*
    inCfg.interruptMode         = kGPIO_IntFallingEdge;
    GPIO_PinInit( ACCEL_INTA_GPIO, ACCEL_INTA_PIN, &inCfg ); 
    */
    GPIO_PinInit( ACCEL_INTA_GPIO, ACCEL_INTA_PIN, &outCfg ); 
    
    /* weigher accelerometer int b group2 pin15 configured as output */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_15_GPIO2_IO15, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_15_GPIO2_IO15, 0x70A0U );  
    
    GPIO_PinInit( ACCEL_INTB_GPIO, ACCEL_INTB_PIN, &outCfg );       

    /* weigher accelerometer group2 pin2 configured as lpi2c clk */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_02_LPI2C1_SCL, 1U );
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_02_LPI2C1_SCL, 0xD8B0U );                       

    /* weigher accelerometer group2 pin3 configured as lpi2c sda */
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_03_LPI2C1_SDA, 1U );                            
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_03_LPI2C1_SDA, 0xD8B0U );                           

    /* weigher accelerometer group1 pin28 configured as lpspi3 clk */
    IOMUXC_SetPinMux( IOMUXC_GPIO_AD_B1_12_LPSPI3_SCK, 0U );
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_12_LPSPI3_SCK, 0x10A0U );  

    /* weigher accelerometer group1 pin30 configured as lpspi3 mosi */
    IOMUXC_SetPinMux( IOMUXC_GPIO_AD_B1_14_LPSPI3_SDO, 0U );
    IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_14_LPSPI3_SDO, 0x70B0U ); //TFink 6/24/24 was 0x10B0. PUS=01: 47K PU, PUE=1: Enable Pullup, PKE=1: Pull Enabled. See AN5078
    
    /* weigher accelerometer group1 pin31 configured as lpspi3 miso */
    IOMUXC_SetPinMux( IOMUXC_GPIO_AD_B1_15_LPSPI3_SDI, 0U);    
    IOMUXC_SetPinConfig( IOMUXC_GPIO_AD_B1_15_LPSPI3_SDI, 0x10B0U );     
     
    /* weigher accelerometer spi cs a group1 pin29 configured as output */
    IOMUXC_SetPinMux( IOMUXC_GPIO_AD_B1_13_GPIO1_IO29, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_AD_B1_13_GPIO1_IO29, 0x70A0U );   
    GPIO_PinInit( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, &outCfg ); 
    GPIO_WritePinOutput( ACCEL_SPI_CS_GPIO, ACCEL_SPI_CS_PIN, false );  //Low so serial EEP is not corrupted   

    /* weigher accelerometer pwr enable group3 pin24 configured as output */
    IOMUXC_SetPinMux( IOMUXC_GPIO_SD_B1_03_GPIO3_IO23, 0U );                                                
    IOMUXC_SetPinConfig( IOMUXC_GPIO_SD_B1_03_GPIO3_IO23, 0x70A0U );   
    GPIO_PinInit( ACCEL_PWR_EN_GPIO, ACCEL_PWR_EN_PIN, &outCfg ); 
    GPIO_WritePinOutput( ACCEL_PWR_EN_GPIO, ACCEL_PWR_EN_PIN, false );  //Turn off. Only turn on if VM enabled
    
    //LPSPI2 pins for CS5530
    //CS5530 SPI clk
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_10_LPSPI2_SCK, 0U );
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_10_LPSPI2_SCK, 0x10B0U );
  
    //CS5530 GPIO chip select
    gpio_pin_config_t csCfg; 
    csCfg.direction = kGPIO_DigitalOutput;
    csCfg.outputLogic = 0U;
    csCfg.interruptMode = kGPIO_NoIntmode;
    
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_11_GPIO2_IO11, 0U );                                    
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_11_GPIO2_IO11, 0x1030U ); 
    GPIO_PinInit(GPIO2, 11U, &csCfg);
  
    //CS5530 SPI SDO
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_12_LPSPI2_SDO, 0U );
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_12_LPSPI2_SDO, 0x10B0U );
  
    //CS5530 SPI SDI
    IOMUXC_SetPinMux( IOMUXC_GPIO_EMC_13_LPSPI2_SDI, 0U );                                    
    IOMUXC_SetPinConfig( IOMUXC_GPIO_EMC_13_LPSPI2_SDI, 0x10B0U );
      
/******************************************************************************/    
/******************************************************************************/    
/******************************************************************************/ 
}

/*! ****************************************************************************   
      \fn configurePinAD_B1_15AsSPI3MISO(void)
 
      \brief
        configure pin AD_B1_15 for SPI3 MISO function. This pin has to be 
        dynamically switched from SPI configuration to an Input to accomodate 
        weigher accelerometer serial EEP write and erase cycles.

      \author
          Tom Fink
*******************************************************************************/ 
void configurePinAD_B1_15AsSPI3MISO(void)
{   //GPIO_EMC_35
    IOMUXC_SetPinMux( IOMUXC_GPIO_AD_B1_15_LPSPI3_SDI, 0U);    
    IOMUXC_SetPinConfig( IOMUXC_GPIO_AD_B1_15_LPSPI3_SDI, 0x10B0U ); 
}

/*! ****************************************************************************   
      \fn 
 
      \brief
        Configure pin AD_B1_15 as a GP Input. This pin has to be 
        dynamically switched from SPI configuration to an Input to accomodate 
        weigher accelerometer serial EEP write and erase cycles.

      \author
          Tom Fink
*******************************************************************************/ 
void configurePinAD_B1_15AsGPIO(void)
{
    gpio_pin_config_t inCfg;
    
    inCfg.direction             = kGPIO_DigitalInput;    
    inCfg.interruptMode         = kGPIO_NoIntmode;
   
    /* weigher accelerometer group1 pin31 configured as gpio out */
    IOMUXC_SetPinMux( IOMUXC_GPIO_AD_B1_15_GPIO1_IO31, 0U );                                    
    IOMUXC_SetPinConfig( IOMUXC_GPIO_AD_B1_15_GPIO1_IO31, 0x0190B0U );
    GPIO_PinInit( ACCEL_SPI_MISO_GPIO, ACCEL_SPI_MISO_PIN, &inCfg );
}