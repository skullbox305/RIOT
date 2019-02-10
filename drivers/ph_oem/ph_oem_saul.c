/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_ph_oem
 * @{
 *
 * @file
 * @brief       pH OEM adaption to the sensor/actuator abstraction layer
 *
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 *
 * @}
 */

#include <string.h>
#include <stdio.h>

#include "saul.h"
#include "ph_oem.h"
#include "ph_oem_regs.h"

static int read_ph(const void *dev, phydat_t *res)
{
    const ph_oem_t *mydev = dev;

    if (mydev->params.enable_pin != GPIO_UNDEF) {
        puts("enable pin not supported with SAUL yet");
        return -ENOTSUP;
    }

    if (mydev->params.interrupt_pin != GPIO_UNDEF) {
        puts("interrupt pin not supported with SAUL yet");
        return -ENOTSUP;
    }

    ph_oem_start_new_reading(mydev);

    /* Read raw pH value */
    if (ph_oem_read_ph(mydev, res->val) < 0) {
        return -ECANCELED;
    }
    res->unit = UNIT_PH;
    res->scale = -3;

    return 1;
}

/* Sets the temperature compensation for taking accurate pH readings */
static int set_temp_compensation(const void *dev, phydat_t *res)
{
    const ph_oem_t *mydev = dev;

    ph_oem_set_compensation(mydev, res->val[0]);

    return 1;
}

const saul_driver_t ph_oem_saul_driver = {
    .read = read_ph,
    .write = set_temp_compensation,
    .type = SAUL_SENSE_PH,
};
