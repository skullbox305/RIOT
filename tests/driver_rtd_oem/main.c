/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup tests
 * @{
 *
 * @file
 * @brief       Test application for the Atlas Scientific RTD OEM sensor driver
 *
 * @author      Ting XU <your-email@placeholder.com>
 *
 * @}
 */

#include "xtimer.h"
#include "event/callback.h"

#include "rtd_oem.h"
#include "rtd_oem_params.h"
#include "rtd_oem_regs.h"

#define SLEEP_SEC                   (5)

static rtd_oem_t dev;

int main(void)
{
//    uint16_t data;

    //output is really fast after reset, sleep 2 sec so we see all ouputs
    xtimer_sleep(2);

    puts("Atlas Scientific RTD OEM sensor driver test application\n");

    printf("Initializing RTD OEM sensor at I2C_%i, address 0x%02x...",
           RTD_OEM_PARAM_I2C, RTD_OEM_PARAM_ADDR);

    if (rtd_oem_init(&dev, rtd_oem_params) == RTD_OEM_OK) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }


    while (1) {

        xtimer_sleep(SLEEP_SEC);
    }
    return 0;
}
