/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "threadManager.h"
#include "fsl_debug_console.h"
#include "developmentSettings.h"
   
   


int main(void)
{
    /* init board hardware. */
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
     
    /** //this is only here to prevent bricking boards when you are experimenting with dangerous code in main */
    #if 0 
      #if 1
      char userInput[1];
      userInput[0] = 0;
      PRINTF("Enter character to continue!\r\n");
      SCANF("%1hhc", userInput); 
      #else
      PRINTF("main(): begin startup delay \r\n");
      unsigned long timeout = 0x27FFF;
      while(timeout--){PRINTF(".");}
      PRINTF("\r\nmain(): startup delay finsished\r\n");   
      #endif
    #endif
      
    //turn status LED's off
    GPIO_WritePinOutput(HEART_BEAT_LED_GPIO, HEART_BEAT_LED_PIN, false);
    GPIO_WritePinOutput(STATUS_LED_GPIO, STATUS_LED_PIN, false);
    
    systemStartup();
    while(1) {}
}