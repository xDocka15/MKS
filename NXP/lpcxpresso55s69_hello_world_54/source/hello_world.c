/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017, 2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"

#include "fsl_power.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/


/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
str_safe(char input, char pass, char pass_lenght )
{
	}

int main(void)
{
    char input[20];
    char pass_stored[20]="123";
    uint32_t DWT1,DWT2;
    int status = 0;

    /* Init board hardware. */
    /* set BOD VBAT level to 1.65V */
    POWER_SetBodVbatLevel(kPOWER_BodVbatLevel1650mv, kPOWER_BodHystLevel50mv, false);
    /* attach main clock divide to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
#if !defined(DONT_ENABLE_FLASH_PREFETCH)
    /* enable flash prefetch for better performance */
    SYSCON->FMCCR |= SYSCON_FMCCR_PREFEN_MASK;
#endif

    PRINTF("hello world.\r\n");

    while (1)
    {
    	SCANF("%s", input);
    	 DWT1=DWT->CYCCNT;
    	 status = strcmp(input, pass_stored);
    	 //SysTick_DelayTicks(1000U);
    	 DWT2=DWT->CYCCNT;
    	 PRINTF("\r\nPass %s status %d",input,status);
    	 if (status != 0)
    	 {

    		 PRINTF("\r\n invalid");
    	 } else
    	 {
    		 PRINTF("\r\n correct");
    	 }
    	 PRINTF("\r\nCycles in function %d", DWT2-DWT1);

    }
}
