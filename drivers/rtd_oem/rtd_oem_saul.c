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
 * @author      Ting XU <your-email@placeholder.com>
 *
 * @}
 */

#include <string.h>
#include <stdio.h>

#include "saul.h"
#include "ph_oem.h"

#include "rtd_oem_regs.h"

static int read_temp(const void *dev, phydat_t *res)
{
	(void) dev;
	(void) res;

    return 1;
}


const saul_driver_t ph_oem_saul_driver = {
    .read = read_temp,
    .write = saul_notsup,
    .type = SAUL_SENSE_TEMP,
};
