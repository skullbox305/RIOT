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
 * @brief       pH OEM device driver
 *
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 * @}
 */

#include "xtimer.h"
#include "assert.h"
#include "periph/i2c.h"
#include "periph/gpio.h"

#include "oem_common.h"
#include "ph_oem.h"
#include "ph_oem_params.h"
#include "ph_oem_regs.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

int ph_oem_init(ph_oem_t *dev, const oem_common_params_t *params)
{
    return oem_common_init(&dev->oem_dev, params);
}

int ph_oem_clear_calibration(const ph_oem_t *dev)
{
    return oem_common_set_calibration(&dev->oem_dev,
                                      PH_OEM_REG_CALIBRATION_REQUEST,
									  PH_OEM_CAL_CLEAR,
                                      false);
}

int ph_oem_read_calibration_state(const ph_oem_t *dev,
                                  uint8_t *calibration_state)
{
    return oem_common_read_reg(&dev->oem_dev, PH_OEM_REG_CALIBRATION_CONFIRM,
                               calibration_state);
}

int ph_oem_set_calibration(const ph_oem_t *dev, uint32_t calibration_value,
                           ph_oem_cal_option_t option)
{
    if (oem_common_set_calibration_value(&dev->oem_dev,
                                         PH_OEM_REG_CALIBRATION_BASE,
                                         calibration_value) != OEM_COMMON_OK) {
        return OEM_COMMON_WRITE_ERR;
    }

    return oem_common_set_calibration(&dev->oem_dev,
                                      PH_OEM_REG_CALIBRATION_REQUEST, option,
                                      false);
}

int ph_oem_set_compensation(const ph_oem_t *dev,
                            uint16_t temp_compensation)
{
    if (!(temp_compensation >= 1 && temp_compensation <= 20000)) {
        return OEM_COMMON_VAL_OUT_OF_RANGE;
    }

    return oem_common_write_reg32(&dev->oem_dev,
                                  PH_OEM_REG_TEMP_COMPENSATION_BASE,
                                  temp_compensation);
}

int ph_oem_read_compensation(const ph_oem_t *dev, uint32_t *temp_compensation)
{
    return oem_common_read_reg32(&dev->oem_dev,
                                 PH_OEM_REG_TEMP_CONFIRMATION_BASE,
                                 false, temp_compensation);
}

int ph_oem_read_ph(const ph_oem_t *dev, uint32_t *ph_value)
{
    return oem_common_read_reg32(&dev->oem_dev, PH_OEM_REG_PH_READING_BASE,
                                 false, ph_value);
}
