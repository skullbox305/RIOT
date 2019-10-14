/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_ec_oem
 * @{
 *
 * @file
 * @brief       EC OEM adaption to the sensor/actuator abstraction layer
 *
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 * @author      Ting Xu <timtsui@outlook.com>
 *
 * @}
 */

#include <string.h>
#include <stdio.h>

#include "include/ec_oem_regs.h"
#include "saul.h"
#include "ec_oem.h"

static int read_ec(const void *dev, phydat_t *res)
{
    const ec_oem_t *mydev = dev;
    uint32_t ec_reading = 0;

    if (mydev->oem_dev.params.interrupt_pin != GPIO_UNDEF) {
        puts("interrupt pin not supported with SAUL yet");
        return -ENOTSUP;
    }

    if (oem_common_start_new_reading(&mydev->oem_dev) < 0) {
        return -ECANCELED;
    }

    /* Read raw EC value */
    if (ec_oem_read_ec(mydev, &ec_reading) < 0) {
        return -ECANCELED;
    }

    /* must be 32 Bit. Split into two indices? But how does phydat dump/print it? */
    res->val[0] = (int16_t)ec_reading;
    res->unit = UNIT_EC;
    res->scale = -2;

    return 1;
}

static int read_tds(const void *dev, phydat_t *res)
{
    const ec_oem_t *mydev = dev;
    uint32_t tds_reading = 0;

    if (mydev->oem_dev.params.interrupt_pin != GPIO_UNDEF) {
        puts("interrupt pin not supported with SAUL yet");
        return -ENOTSUP;
    }

    if (oem_common_start_new_reading(&mydev->oem_dev) < 0) {
        return -ECANCELED;
    }

    /* Read raw TDS value */
    if (ec_oem_read_tds(mydev, &tds_reading) < 0) {
        return -ECANCELED;
    }
    res->val[0] = (int16_t)tds_reading;
    res->unit = UNIT_PPM;
    res->scale = -2;

    return 1;
}

static int read_pss(const void *dev, phydat_t *res)
{
    const ec_oem_t *mydev = dev;
    uint32_t pss_reading = 0;

    if (mydev->oem_dev.params.interrupt_pin != GPIO_UNDEF) {
        puts("interrupt pin not supported with SAUL yet");
        return -ENOTSUP;
    }

    if (oem_common_start_new_reading(&mydev->oem_dev) < 0) {
        return -ECANCELED;
    }

    /* Read raw PSS value */
    if (ec_oem_read_pss(mydev, &pss_reading) < 0) {
        return -ECANCELED;
    }
    res->val[0] = (int16_t)pss_reading;
    res->unit = UNIT_PPT;
    res->scale = -2;

    return 1;
}

/* Sets the temperature compensation for taking accurate EC readings.
 * Valid temperature range is 1 - 20000 (unknown °C  to  unknown °C) */
static int set_temp_compensation(const void *dev, phydat_t *res)
{
    const ec_oem_t *mydev = dev;

    if (!(res->val[0] >= 1 && res->val[0] <= 20000)) {
        return -ECANCELED;
    }

    if (ec_oem_set_compensation(mydev, res->val[0]) < 0) {
        return -ECANCELED;
    }
    return 1;
}

const saul_driver_t ec_oem_ec_saul_driver = {
    .read = read_ec,
    .write = saul_notsup ,
    .type = SAUL_SENSE_EC,
};

const saul_driver_t ec_oem_tds_saul_driver = {
    .read = read_tds,
    .write = saul_notsup,
    .type = SAUL_SENSE_TDS,
};

const saul_driver_t ec_oem_pss_saul_driver = {
    .read = read_pss,
    .write = saul_notsup,
    .type = SAUL_SENSE_PSS,
};

const saul_driver_t ec_oem_temp_comp_saul_driver = {
    .read = saul_notsup,
    .write = set_temp_compensation,
    .type = SAUL_SENSE_TEMP,
};
