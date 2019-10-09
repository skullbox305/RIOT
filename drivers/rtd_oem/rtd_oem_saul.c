/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_rtd_oem
 * @{
 *
 * @file
 * @brief       RTD OEM adaption to the sensor/actuator abstraction layer
 *
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 * @author      Ting XU <timtsui@outlook.com>
 *
 * @}
 */

#include <string.h>
#include <stdio.h>

#include "saul.h"
#include "rtd_oem.h"
#include "rtd_oem_regs.h"

static int read_temp(const void *dev, phydat_t *res)
{
    const rtd_oem_t *mydev = dev;
    int32_t rtd_reading = 0;

    if (mydev->oem_dev.params.interrupt_pin != GPIO_UNDEF) {
        puts("interrupt pin not supported with SAUL yet");
        return -ENOTSUP;
    }

    if (oem_common_start_new_reading(&mydev->oem_dev) < 0) {
        return -ECANCELED;
    }

    /* Read raw RTD value */
    if (rtd_oem_read_temp(mydev, &rtd_reading) < 0) {
        return -ECANCELED;
    }
    /* should be 32 Bit. Split into two inidices? But how does phydat dump/print it? */
    res->val[0] = (int16_t)rtd_reading;
    res->unit = UNIT_TEMP_C;
    res->scale = -3;

    return 1;
}


const saul_driver_t rtd_oem_saul_driver = {
    .read = read_temp,
    .write = saul_notsup,
    .type = SAUL_SENSE_TEMP,
};
