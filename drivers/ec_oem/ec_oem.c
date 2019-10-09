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
 * @brief       EC OEM device driver
 *
 * @author      Ting Xu <timtsui@outlook.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 * @}
 */

#include "xtimer.h"
#include "assert.h"
#include "periph/i2c.h"
#include "periph/gpio.h"

#include "ec_oem.h"
#include "ec_oem_params.h"
#include "ec_oem_regs.h"


#define ENABLE_DEBUG    (1)
#include "debug.h"

#define DEV_I2C (dev->oem_dev.params.i2c)
#define ADDR (dev->oem_dev.params.addr)
#define IRQ_OPTION (dev->oem_dev.params.irq_option)

int ec_oem_init(ec_oem_t *dev, const oem_common_params_t *params)
{
    return oem_common_init(&dev->oem_dev, params);
}

int ec_oem_set_probe_type(ec_oem_t *dev, uint16_t probe_type)
{
    //TODO check range for probe type

    uint8_t reg_value[2];

    reg_value[0] = (uint8_t)(probe_type >> 8);
    reg_value[1] = (uint8_t)(probe_type & 0x00FF);

    i2c_acquire(DEV_I2C);

    if (i2c_write_regs(DEV_I2C, ADDR, EC_OEM_REG_SET_PROBE_TYPE, &reg_value, 2,
                       0)
        < 0) {
        DEBUG("\n[ec_oem debug] Writing probe type value failed \n");
        i2c_release(DEV_I2C);
        return OEM_COMMON_WRITE_ERR;
    }

    if (i2c_read_regs(DEV_I2C, ADDR, EC_OEM_REG_SET_PROBE_TYPE, &reg_value, 2,
                      0)
        < 0) {
        DEBUG("\n[ec_oem debug] Reading probe type value failed \n");
        i2c_release(DEV_I2C);
        return OEM_COMMON_READ_ERR;
    }

    /* Check if written value is in fact correct */
    uint16_t confirm_value = (int16_t)(reg_value[0] << 8)
                             | (int16_t)(reg_value[1]);

    if (confirm_value != probe_type) {
        DEBUG("\n[ec_oem debug] Setting probe type register to %d "
              "failed \n", probe_type);
        i2c_release(DEV_I2C);
        return OEM_COMMON_WRITE_ERR;
    }

    i2c_release(DEV_I2C);

    return OEM_COMMON_OK;
}

int ec_oem_clear_calibration(const ec_oem_t *dev)
{
    return oem_common_set_calibration(&dev->oem_dev,
                                      EC_OEM_REG_CALIBRATION_REQUEST,
									  EC_OEM_CAL_CLEAR,
                                      false);
}

int ec_oem_set_calibration(const ec_oem_t *dev, uint32_t calibration_value,
                           ec_oem_calibration_option_t option)
{
    if (oem_common_set_calibration_value(&dev->oem_dev,
                                         EC_OEM_REG_CALIBRATION_BASE,
                                         calibration_value) != OEM_COMMON_OK) {
        return OEM_COMMON_WRITE_ERR;
    }

    return oem_common_set_calibration(&dev->oem_dev,
                                      EC_OEM_REG_CALIBRATION_REQUEST, option,
                                      false);
}

int ec_oem_read_calibration_state(const ec_oem_t *dev,
                                  uint8_t *calibration_state)
{
    return oem_common_read_reg(&dev->oem_dev, EC_OEM_REG_CALIBRATION_CONFIRM,
                               calibration_state);
}

int ec_oem_set_compensation(const ec_oem_t *dev, uint16_t temp_compensation)
{
    if (!(temp_compensation >= 1 && temp_compensation <= 20000)) {
        return OEM_COMMON_VAL_OUT_OF_RANGE;
    }

    return oem_common_write_reg32(&dev->oem_dev,
                                  EC_OEM_REG_TEMP_COMPENSATION_BASE,
                                  temp_compensation);
}

int ec_oem_read_compensation(const ec_oem_t *dev, uint32_t *temp_compensation)
{
    return oem_common_read_reg32(&dev->oem_dev,
                                 EC_OEM_REG_TEMP_CONFIRMATION_BASE,
                                 false, temp_compensation);
}

int ec_oem_read_ec(const ec_oem_t *dev, uint32_t *ec_value)
{
    return oem_common_read_reg32(&dev->oem_dev, EC_OEM_REG_EC_READING_BASE,
                                 false, ec_value);
}

int ec_oem_read_tds(const ec_oem_t *dev, uint32_t *tds_value)
{
    return oem_common_read_reg32(&dev->oem_dev, EC_OEM_REG_TDS_READING_BASE,
                                 false, tds_value);
}

int ec_oem_read_pss(const ec_oem_t *dev, uint32_t *pss_value)
{
    return oem_common_read_reg32(&dev->oem_dev, EC_OEM_REG_PSS_READING_BASE,
                                 false, pss_value);
}
