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

#define I2C (dev->params.i2c)
#define ADDR (dev->params.addr)
#define IRQ_OPTION (dev->params.irq_option)

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

    i2c_acquire(I2C);

    if (i2c_write_regs(I2C, ADDR, EC_OEM_REG_SET_PROBE_TYPE, &reg_value, 2, 0)
        < 0) {
        DEBUG("\n[ec_oem debug] Writing probe type value failed \n");
        i2c_release(I2C);
        return OEM_COMMON_WRITE_ERR;
    }

    if (i2c_read_regs(I2C, ADDR, EC_OEM_REG_SET_PROBE_TYPE, &reg_value, 2, 0)
        < 0) {
        DEBUG("\n[ec_oem debug] Reading probe type value failed \n");
        i2c_release(I2C);
        return OEM_COMMON_READ_ERR;
    }

    /* Check if written value is in fact correct */
    uint16_t confirm_value = (int16_t)(reg_value[0] << 8)
                             | (int16_t)(reg_value[1]);

    if (confirm_value != probe_type) {
        DEBUG("\n[ec_oem debug] Setting probe type register to %d "
              "failed \n", probe_type);
        i2c_release(I2C);
        return OEM_COMMON_WRITE_ERR;
    }

    i2c_release(I2C);

    return OEM_COMMON_OK;
}

int ec_oem_clear_calibration(const ec_oem_t *dev)
{
    uint8_t reg_value;

    assert(dev);
    i2c_acquire(I2C);
    if (i2c_write_reg(I2C, ADDR, EC_OEM_REG_CALIBRATION_REQUEST, 0x01, 0) < 0) {
        DEBUG("\n[ec_oem debug] Clearing calibration failed \n");
        i2c_release(I2C);
        return EC_OEM_WRITE_ERR;
    }

    do {
        if (i2c_read_reg(I2C, ADDR, EC_OEM_REG_CALIBRATION_REQUEST, &reg_value,
                         0) < 0) {
            i2c_release(I2C);
            return EC_OEM_READ_ERR;
        }
    } while (reg_value != 0x00);

    i2c_release(I2C);

    return EC_OEM_OK;
}

int ec_oem_set_calibration(const ec_oem_t *dev, uint32_t calibration_value,
                           ec_oem_calibration_option_t option)
{
    assert(dev);

    if (calibration_value > 0) {
        if (_set_calibration_value(dev, calibration_value) != EC_OEM_OK) {
            return EC_OEM_WRITE_ERR;
        }
    }

    uint8_t reg_value;

    i2c_acquire(I2C);

    if (i2c_write_reg(I2C, ADDR, EC_OEM_REG_CALIBRATION_REQUEST, option,
                      0) < 0) {
        DEBUG("\n[ec_oem debug] Sending calibration request failed\n");
        return EC_OEM_WRITE_ERR;
    }

    do {
        if (i2c_read_reg(I2C, ADDR, EC_OEM_REG_CALIBRATION_REQUEST, &reg_value,
                         0) < 0) {
            DEBUG(
                "\n[ec_oem debug] Reading calibration request status failed\n");
            i2c_release(I2C);
            return EC_OEM_READ_ERR;
        }
    } while (reg_value != 0x00);

    i2c_release(I2C);

    return EC_OEM_OK;
}

int ec_oem_read_calibration_state(const ec_oem_t *dev,
                                  uint8_t *calibration_state)
{
    assert(dev);
    i2c_acquire(I2C);

    if (i2c_read_reg(I2C, ADDR, EC_OEM_REG_CALIBRATION_CONFIRM,
                     calibration_state, 0) < 0) {
        DEBUG(
            "\n[ec_oem debug] Failed at reading calibration confirm register\n");
        i2c_release(I2C);
        return EC_OEM_READ_ERR;
    }
    i2c_release(I2C);
    return EC_OEM_OK;
}

int ec_oem_set_compensation(const ec_oem_t *dev,
                            uint16_t temperature_compensation)
{
    if (!(temperature_compensation >= 1 && temperature_compensation <= 20000)) {
        return EC_OEM_TEMP_OUT_OF_RANGE;
    }

    assert(dev);
    uint8_t reg_value[4];

    reg_value[0] = 0x00;
    reg_value[1] = 0x00;
    reg_value[2] = (uint8_t)(temperature_compensation >> 8);
    reg_value[3] = (uint8_t)(temperature_compensation & 0x00FF);

    i2c_acquire(I2C);

    if (i2c_write_regs(I2C, ADDR, EC_OEM_REG_TEMP_COMPENSATION_BASE, &reg_value,
                       4, 0) < 0) {
        DEBUG("\n[ec_oem debug] Setting temperature compensation of device to "
              "%d failed\n", temperature_compensation);
        i2c_release(I2C);
        return EC_OEM_WRITE_ERR;
    }
    i2c_release(I2C);

    return EC_OEM_OK;
}

int ec_oem_read_compensation(const ec_oem_t *dev,
                             uint32_t *temperature_compensation)
{
    uint8_t reg_value[4];

    assert(dev);
    i2c_acquire(I2C);

    if (i2c_read_regs(I2C, ADDR, EC_OEM_REG_TEMP_CONFIRMATION_BASE, &reg_value,
                      4, 0) < 0) {
        DEBUG("[ec_oem debug] Getting temperature compensation value failed\n");
        i2c_release(I2C);
        return EC_OEM_READ_ERR;
    }
    *temperature_compensation = (int16_t)(reg_value[2] << 8)
                                | (int16_t)(reg_value[3]);

    i2c_release(I2C);

    return EC_OEM_OK;
}

int ec_oem_read_ec(const ec_oem_t *dev, uint32_t *ec_value)
{
    uint8_t reg_value[4];

    assert(dev);
    i2c_acquire(I2C);

    if (i2c_read_regs(I2C, ADDR, EC_OEM_REG_EC_READING_BASE, &reg_value, 4, 0)
        < 0) {
        DEBUG("[ec_oem debug] Getting EC value failed\n");
        i2c_release(I2C);
        return EC_OEM_READ_ERR;
    }
    *ec_value = (int32_t)(reg_value[0] << 24) | (int32_t)(reg_value[1] << 16) |
                (int32_t)(reg_value[2] << 8) | (int32_t)(reg_value[3]);

    i2c_release(I2C);

    return EC_OEM_OK;
}

int ec_oem_read_tds(const ec_oem_t *dev, uint32_t *tds_value)
{
    uint8_t reg_value[4];

    assert(dev);
    i2c_acquire(I2C);

    if (i2c_read_regs(I2C, ADDR, EC_OEM_REG_TDS_READING_BASE, &reg_value, 4, 0)
        < 0) {
        DEBUG("[ec_oem debug] Getting TDS value failed\n");
        i2c_release(I2C);
        return EC_OEM_READ_ERR;
    }
    *tds_value = (int32_t)(reg_value[0] << 24) | (int32_t)(reg_value[1] << 16) |
                 (int32_t)(reg_value[2] << 8) | (int32_t)(reg_value[3]);

    i2c_release(I2C);

    return EC_OEM_OK;
}

int ec_oem_read_pss(const ec_oem_t *dev, uint32_t *pss_value)
{
    uint8_t reg_value[4];

    assert(dev);
    i2c_acquire(I2C);

    if (i2c_read_regs(I2C, ADDR, EC_OEM_REG_PSS_READING_BASE, &reg_value, 4, 0)
        < 0) {
        DEBUG("[ec_oem debug] Getting PSS value failed\n");
        i2c_release(I2C);
        return EC_OEM_READ_ERR;
    }
    *pss_value = (int32_t)(reg_value[0] << 24) | (int32_t)(reg_value[1] << 16) |
                 (int32_t)(reg_value[2] << 8) | (int32_t)(reg_value[3]);

    i2c_release(I2C);

    return EC_OEM_OK;
}
