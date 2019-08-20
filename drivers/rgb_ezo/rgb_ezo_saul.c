/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_rgb_oem
 * @{
 *
 * @file
 * @brief       RGB EZO adaption to the sensor/actuator abstraction layer
 *
 * @author      Ting XU <timtsui@outlook.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 *
 * @}
 */

#include <string.h>
#include <stdio.h>

#include "saul.h"
#include "rgb_ezo.h"


const saul_driver_t rgb_ezo_saul_driver = {
    .read = saul_notsup,
    .write = saul_notsup,
    .type = SAUL_SENSE_RGB,
};
