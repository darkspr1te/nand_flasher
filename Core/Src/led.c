/*  Copyright (C) 2020 NANDO authors
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 3.
 */

#include "led.h"
#include "main.h"

void led_init()
{
    /*GPIO_InitTypeDef GPIO_InitStruct;

    __
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOG,GPIO_PIN_15,0);
 */
}

static void led_set(GPIO_TypeDef *gpiox, uint16_t pin, bool on)
{
    if (on)
        //GPIO_SetBits(gpiox, pin);
         HAL_GPIO_WritePin(gpiox,pin,1);
    else
        //GPIO_ResetBits(gpiox, pin);
        HAL_GPIO_WritePin(gpiox,pin,0);
        
}

void led_wr_set(bool on)
{
    led_set(GPIOG, GPIO_PIN_15, on);
}

void led_rd_set(bool on)
{
    led_set(GPIOG, GPIO_PIN_15, on);
}

