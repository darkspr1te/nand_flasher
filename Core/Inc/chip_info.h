/*  Copyright (C) 2020 NANDO authors
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 3.
 */

#ifndef _CHIP_INFO_H_
#define _CHIP_INFO_H_
#include "main.h"
#include "stdint.h"
typedef struct
{
    uint8_t maker_id;
    uint8_t device_id;
    uint8_t third_id;
    uint8_t fourth_id;
    uint8_t fifth_id;
} chip_id_t;

typedef struct
{
    uint32_t page_size; /* without spare area */
    uint32_t block_size;
    uint64_t total_size;
    uint32_t spare_size;
    uint8_t bb_mark_off;
} chip_info_t;



/*  Copyright (C) 2020 NANDO authors
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 3.
 */



#endif /* _CHIP_H_ */
