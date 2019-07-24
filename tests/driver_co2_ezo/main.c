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
 * @brief       Test application for the Atlas Scientific CO2 EZO sensor driver
 *
 * @author      Ting XU <timtsui@outlook.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 *
 * @}
 */

#include "co2_ezo.h"
#include "co2_ezo_params.h"
#include "co2_ezo_internal.h"
#include "xtimer.h"

#define SLEEP_SEC                   (5)

static co2_ezo_t dev;

int main(void)
{
    xtimer_sleep(2);

    puts("Atlas Scientific CO2 EZO sensor driver test application\n");

    printf("Init CO2 EZO sensor at I2C_%i, address 0x%02x...",
           CO2_EZO_PARAM_I2C, CO2_EZO_PARAM_ADDR);

    if (co2_ezo_init(&dev, co2_ezo_params) == CO2_EZO_OK) {
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
