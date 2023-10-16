/*  Copyright (C) 2020 NANDO authors
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 3.
 */

#ifndef _NAND_PROGRAMMER_H_
#define _NAND_PROGRAMMER_H_


#include "stm32f4xx_hal.h"


typedef struct
{
    int (*send)(uint8_t *data, uint32_t len);
    int (*send_ready)();
    uint32_t (*peek)(uint8_t **data);
    void (*consume)();
} np_comm_cb_t;

int np_comm_register(np_comm_cb_t *cb);
void np_comm_unregister(np_comm_cb_t *cb);
void np_init();
void np_handler();

#endif
