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
 * @brief       DO OEM device driver
 *
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 * @author      Ting Xu <timtsui@outlook.com>
 * @}
 */

#include "xtimer.h"
#include "assert.h"
#include "periph/i2c.h"
#include "periph/gpio.h"

#include "do_oem.h"
#include "do_oem_params.h"
#include "do_oem_regs.h"

#define ENABLE_DEBUG    (1)
#include "debug.h"

int do_oem_init(do_oem_t *dev, const oem_common_params_t *params)
{
    return oem_common_init(&dev->oem_dev, params);
}

int do_oem_clear_calibration(const do_oem_t *dev)
{
    return oem_common_set_calibration(&dev->oem_dev,
                                      DO_OEM_REG_CALIBRATION,
                                      DO_OEM_CALI_CLEAR,
                                      false);
}

int do_oem_set_calibration(const do_oem_t *dev,
                           do_oem_calibration_option_t option)
{
    return oem_common_set_calibration(&dev->oem_dev, DO_OEM_REG_CALIBRATION,
                                      option, true);
}

int do_oem_read_calibration_state(const do_oem_t *dev,
                                  uint8_t *calibration_state)
{
    return oem_common_read_reg(&dev->oem_dev, DO_OEM_REG_CALIBRATION_CONFIRM,
                               calibration_state);
}

int do_oem_set_sal_compensation(const do_oem_t *dev,
                                uint32_t sali_compensation)
{
    if (!(sali_compensation >= 1 && sali_compensation <= 6562500)) {
        return OEM_COMMON_VAL_OUT_OF_RANGE;
    }

    return oem_common_write_reg32(&dev->oem_dev,
                                  DO_OEM_REG_SALI_COMPENSATION_BASE,
                                  sali_compensation);
}

int do_oem_set_pres_compensation(const do_oem_t *dev,
                                 uint16_t pressure_compensation)
{
    if (!(pressure_compensation >= 3000 && pressure_compensation <= 11000)) {
        return OEM_COMMON_VAL_OUT_OF_RANGE;
    }

    return oem_common_write_reg32(&dev->oem_dev,
                                  DO_OEM_REG_PRES_COMPENSATION_BASE,
                                  pressure_compensation);
}

int do_oem_set_temp_compensation(const do_oem_t *dev,
                                 uint16_t temperature_compensation)
{
    if (!(temperature_compensation >= 1 && temperature_compensation <= 20000)) {
        return OEM_COMMON_VAL_OUT_OF_RANGE;
    }

    return oem_common_write_reg32(&dev->oem_dev,
                                  DO_OEM_REG_TEMP_COMPENSATION_BASE,
                                  temperature_compensation);
}

int do_oem_read_sali_compensation(const do_oem_t *dev,
                                  uint32_t *salinity_compensation)
{
    return oem_common_read_reg32(&dev->oem_dev,
                                 DO_OEM_REG_SALI_COMFIRMATION_BASE,
                                 false, salinity_compensation);
}

int do_oem_read_pres_compensation(const do_oem_t *dev,
                                  uint32_t *pressure_compensation)
{
    return oem_common_read_reg32(&dev->oem_dev,
                                 DO_OEM_REG_PRES_COMFIRMATION_BASE,
                                 false, pressure_compensation);
}

int do_oem_read_temp_compensation(const do_oem_t *dev,
                                  uint32_t *temperature_compensation)
{
    return oem_common_read_reg32(&dev->oem_dev,
                                 DO_OEM_REG_TEMP_COMFIRMATION_BASE,
                                 false, temperature_compensation);
}

int do_oem_read_do_mg(const do_oem_t *dev, uint32_t *do_mg_value)
{
    return oem_common_read_reg32(&dev->oem_dev, DO_OEM_REG_DO_MGL_READING_BASE,
                                 false, do_mg_value);
}

int do_oem_read_do_percent(const do_oem_t *dev, uint32_t *do_percent_value)
{
    return oem_common_read_reg32(&dev->oem_dev, DO_OEM_REG_DO_PERCENT_READING_BASE,
                                 false, do_percent_value);
}
