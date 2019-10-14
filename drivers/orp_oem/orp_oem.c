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
 * @brief       ORP OEM device driver
 *
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 * @author      Ting XU <timtsui@outlook.com>
 * @}
 */

#include "xtimer.h"
#include "assert.h"
#include "periph/i2c.h"
#include "periph/gpio.h"

#include "orp_oem.h"
#include "orp_oem_params.h"
#include "orp_oem_regs.h"

#define ENABLE_DEBUG    (1)
#include "debug.h"

int orp_oem_init(orp_oem_t *dev, const oem_common_params_t *params)
{
    return oem_common_init(&dev->oem_dev, params);
}

int orp_oem_clear_calibration(const orp_oem_t *dev)
{
    return oem_common_set_calibration(&dev->oem_dev,
                                      ORP_OEM_REG_CALIBRATION_REQUEST,
                                      ORP_OEM_CAL_CLEAR,
                                      false);
}

int orp_oem_read_calibration_state(const orp_oem_t *dev,
                                   uint8_t *calibration_state)
{
    return oem_common_read_reg(&dev->oem_dev, ORP_OEM_REG_CALIBRATION_CONFIRM,
                               calibration_state);
}


int orp_oem_set_calibration(const orp_oem_t *dev, int16_t calibration_value,
                            orp_oem_calibration_option_t option)
{
    if (oem_common_set_calibration_value(&dev->oem_dev,
                                         ORP_OEM_REG_CALIBRATION_BASE,
                                         calibration_value) != OEM_COMMON_OK) {
        return OEM_COMMON_WRITE_ERR;
    }

    return oem_common_set_calibration(&dev->oem_dev,
                                      ORP_OEM_REG_CALIBRATION_REQUEST, option,
                                      false);
}

int orp_oem_read_orp(const orp_oem_t *dev, int32_t *orp_value)
{
    return oem_common_read_reg32(&dev->oem_dev, ORP_OEM_REG_ORP_READING_BASE,
                                 true, orp_value);
}
