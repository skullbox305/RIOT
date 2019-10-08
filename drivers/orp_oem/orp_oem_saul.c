/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_orp_oem
 * @{
 *
 * @file
 * @brief       ORP OEM adaption to the sensor/actuator abstraction layer
 *
 * @author      Ting XU <timtsui@outlook.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 *
 * @}
 */

#include <string.h>
#include <stdio.h>

#include "orp_oem_regs.h"
#include "saul.h"
#include "orp_oem.h"

static int read_orp(const void *dev, phydat_t *res)
{
    const orp_oem_t *mydev = dev;
    int32_t orp_reading;

    if (mydev->oem_dev.params.interrupt_pin != GPIO_UNDEF) {
        puts("interrupt pin not supported with SAUL yet");
        return -ENOTSUP;
    }

    if (oem_common_start_new_reading(&mydev->oem_dev) < 0) {
        return -ECANCELED;
    }

    /* Read raw ORP value */
    if (orp_oem_read_orp(mydev, &orp_reading) < 0) {
        return -ECANCELED;
    }
    res->val[0] = (int16_t)orp_reading;
    res->unit = UNIT_ORP;
    res->scale = -1;

    return 1;
}

const saul_driver_t orp_oem_saul_driver = {
    .read = read_orp,
    .write = saul_notsup,
    .type = SAUL_SENSE_ORP,
};
