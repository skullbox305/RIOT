/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_do_oem
 * @{
 *
 * @file
 * @brief       DO OEM adaption to the sensor/actuator abstraction layer
 *
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 * @author      Ting Xu <timtsui@outlook.com>
 *
 * @}
 */

#include <string.h>
#include <stdio.h>

#include "saul.h"
#include "do_oem.h"
#include "include/do_oem_regs.h"

static int read_do_mg(const void *dev, phydat_t *res)
{
    const do_oem_t *mydev = dev;
    uint32_t do_mg_reading = 0;

    if (mydev->oem_dev.params.interrupt_pin != GPIO_UNDEF) {
        puts("interrupt pin not supported with SAUL yet");
        return -ENOTSUP;
    }

    if (oem_common_start_new_reading(&mydev->oem_dev) < 0) {
        return -ECANCELED;
    }

    /* Read raw D.O. value in mg/L */
    if (do_oem_read_do_mg(mydev, &do_mg_reading) < 0) {
        return -ECANCELED;
    }
    res->val[0] = (int16_t)do_mg_reading;
    res->unit = UNIT_DO_MG;
    res->scale = -2;

    return 1;
}

static int read_do_percent(const void *dev, phydat_t *res)
{
    const do_oem_t *mydev = dev;
    uint32_t do_percent_reading = 0;

    if (mydev->oem_dev.params.interrupt_pin != GPIO_UNDEF) {
        puts("interrupt pin not supported with SAUL yet");
        return -ENOTSUP;
    }

    if (oem_common_start_new_reading(&mydev->oem_dev) < 0) {
        return -ECANCELED;
    }

    /* Read raw D.O. value in % saturation */
    if (do_oem_read_do_percent(mydev, &do_percent_reading) < 0) {
        return -ECANCELED;
    }
    res->val[0] = (int16_t)do_percent_reading;
    res->unit = UNIT_PERCENT;
    res->scale = -2;

    return 1;
}

/* Sets the compensations for taking accurate D.O. readings. */
static int set_sal_compensation(const void *dev, phydat_t *res)
{
    const do_oem_t *mydev = dev;

    if (do_oem_set_sal_compensation(mydev, res->val[0]) < 0) {
        return -ECANCELED;
    }
    return 1;
}

static int set_pres_compensation(const void *dev, phydat_t *res)
{
    const do_oem_t *mydev = dev;

    if (do_oem_set_pres_compensation(mydev, res->val[0]) < 0) {
        return -ECANCELED;
    }
    return 1;
}

static int set_temp_compensation(const void *dev, phydat_t *res)
{
    const do_oem_t *mydev = dev;

    if (do_oem_set_temp_compensation(mydev, res->val[0]) < 0) {
        return -ECANCELED;
    }
    return 1;
}

const saul_driver_t do_oem_do_mg_saul_driver =
{ .read = read_do_mg,
  .write = saul_notsup,
  .type = SAUL_SENSE_DO_MG,
};

const saul_driver_t do_oem_do_percent_saul_driver =
{ .read = read_do_percent,
  .write = saul_notsup,
  .type = SAUL_SENSE_DO_P,
};

const saul_driver_t do_oem_sal_comp_saul_driver =
{ .read = saul_notsup,
  .write = set_sal_compensation,
  .type = SAUL_SENSE_EC,
};

const saul_driver_t do_oem_pres_comp_saul_driver =
{ .read = saul_notsup,
  .write = set_pres_compensation,
  .type = SAUL_SENSE_PRESS,
};

const saul_driver_t do_oem_temp_comp_saul_driver =
{ .read = saul_notsup,
  .write = set_temp_compensation,
  .type = SAUL_SENSE_TEMP,
};
