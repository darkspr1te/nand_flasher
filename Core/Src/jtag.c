/*  Copyright (C) 2020 NANDO authors
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 3.
 */

#if defined (STM32F407xx)
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_flash.h"
#else
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_flash.h"
#endif


void jtag_init()
{
    /* Enable JTAG in low power mode */
    //DBGMCU_Config(DBGMCU_SLEEP | DBGMCU_STANDBY | DBGMCU_STOP, ENABLE);
    //__HAL_AFIO_REMAP_SWJ_ENABLE();
    
}
