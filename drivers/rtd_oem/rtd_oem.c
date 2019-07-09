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
 * @brief       RTD OEM device driver
 *
 * @author      Ting XU <your-email@placeholder.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 * @}
 */

#include "xtimer.h"
#include "assert.h"
#include "periph/i2c.h"
#include "periph/gpio.h"

#include "rtd_oem.h"

#include "rtd_oem_params.h"
#include "rtd_oem_regs.h"

#define ENABLE_DEBUG    (1)
#include "debug.h"

#define I2C (dev->params.i2c)
#define ADDR (dev->params.addr)
#define IRQ_OPTION (dev->params.irq_option)

int rtd_oem_init(rtd_oem_t *dev, const rtd_oem_params_t *params)
{
    assert(dev && params);

    dev->params = *params;

    uint8_t reg_data;

    i2c_acquire(I2C);

    /* Register read test */
    if (i2c_read_regs(I2C, ADDR, RTD_OEM_REG_DEVICE_TYPE,
                      &reg_data, 1, 0x0) < 0) {
        DEBUG("\n[rtd_oem debug] init - error: unable to read reg %x\n",
              RTD_OEM_REG_DEVICE_TYPE);

        i2c_release(I2C);
        return RTD_OEM_NODEV;
    }

    /* Test if the device ID of the attached RTD OEM sensor equals the
     * value of the RTD_OEM_REG_DEVICE_TYPE register
     * */
    if (reg_data != RTD_OEM_DEVICE_TYPE_ID) {
        DEBUG("\n[rtd_oem debug] init - error: the attached device is not a RTD OEM "
              "Sensor. Read Device Type ID is: %i\n",
              reg_data);
        i2c_release(I2C);
        return RTD_OEM_NOT_RTD;
    }
    i2c_release(I2C);

    return RTD_OEM_OK;
}

int rtd_oem_set_led_state(const rtd_oem_t *dev, rtd_oem_led_state_t state)
{
    assert(dev);
    i2c_acquire(I2C);

    if (i2c_write_reg(I2C, ADDR, RTD_OEM_REG_LED, state, 0x0) < 0) {
        DEBUG("\n[rtd_oem debug] Setting LED state to %d failed.\n", state);
        i2c_release(I2C);
        return RTD_OEM_WRITE_ERR;
    }
    i2c_release(I2C);

    return RTD_OEM_OK;
}

static int _unlock_address_reg(rtd_oem_t *dev)
{
    uint8_t reg_value = 1;

    i2c_acquire(I2C);

    i2c_write_reg(I2C, ADDR, RTD_OEM_REG_UNLOCK, 0x55, 0x0);
    i2c_write_reg(I2C, ADDR, RTD_OEM_REG_UNLOCK, 0xAA, 0x0);
    /* if successfully unlocked the register will equal 0x00 */
    i2c_read_reg(I2C, ADDR, RTD_OEM_REG_UNLOCK, &reg_value, 0x0);

    if (reg_value != 0x00) {
        DEBUG("\n[rtd_oem debug] Failed at unlocking I2C address register. \n");
        i2c_release(I2C);
        return RTD_OEM_WRITE_ERR;
    }
    i2c_release(I2C);

    return RTD_OEM_OK;
}

int rtd_oem_set_i2c_address(rtd_oem_t *dev, uint8_t addr)
{
    assert(dev);

    if (_unlock_address_reg(dev) != RTD_OEM_OK) {
        return RTD_OEM_WRITE_ERR;
    }

    i2c_acquire(I2C);

    if (i2c_write_reg(I2C, ADDR, RTD_OEM_REG_ADDRESS, addr, 0x0) < 0) {
        DEBUG("\n[rtd_oem debug] Setting I2C address to %x failed\n", addr);
        i2c_release(I2C);
        return RTD_OEM_WRITE_ERR;
    }

    dev->params.addr = addr;
    i2c_release(I2C);

    return RTD_OEM_OK;
}

int rtd_oem_clear_calibration(const rtd_oem_t *dev)
{
    uint8_t reg_value;

    assert(dev);
    i2c_acquire(I2C);
    if (i2c_write_reg(I2C, ADDR, RTD_OEM_REG_CALIBRATION_REQUEST, RTD_OEM_CALIBRATE_CLEAR, 0) < 0) {
        DEBUG("\n[rtd_oem debug] Clearing calibration failed \n");
        i2c_release(I2C);
        return RTD_OEM_WRITE_ERR;
    }

    do {
        if (i2c_read_reg(I2C, ADDR, RTD_OEM_REG_CALIBRATION_REQUEST, &reg_value,
                         0) < 0) {
            i2c_release(I2C);
            return RTD_OEM_READ_ERR;
        }
    } while (reg_value != 0x00);

    i2c_release(I2C);

    return RTD_OEM_OK;
}

int rtd_oem_read_calibration_state(const rtd_oem_t *dev, uint16_t *calibration_state)
{
    assert(dev);
    i2c_acquire(I2C);

    if (i2c_read_reg(I2C, ADDR, RTD_OEM_REG_CALIBRATION_CONFIRM,
                     calibration_state, 0) < 0) {
        DEBUG("\n[rtd_oem debug] Failed at reading calibration confirm register\n");
        i2c_release(I2C);
        return RTD_OEM_READ_ERR;
    }

    i2c_release(I2C);
    return RTD_OEM_OK;
}

int rtd_oem_set_calibration(const rtd_oem_t *dev, uint16_t calibration_value,
                           rtd_oem_calibration_option_t option)
{
    assert(dev);

    if (_set_calibration_value(dev, calibration_value) != RTD_OEM_OK) {
        return RTD_OEM_WRITE_ERR;
    }

    uint8_t reg_value;

    i2c_acquire(I2C);

    if (i2c_write_reg(I2C, ADDR, RTD_OEM_REG_CALIBRATION_REQUEST,
                      option, 0) < 0) {
        DEBUG("\n[rtd_oem debug] Sending calibration request failed\n");
        return RTD_OEM_WRITE_ERR;
    }

    do {
        if (i2c_read_reg(I2C, ADDR, RTD_OEM_REG_CALIBRATION_REQUEST, &reg_value,
                         0) < 0) {
            DEBUG("\n[rtd_oem debug] Reading calibration request status failed\n");
            i2c_release(I2C);
            return RTD_OEM_READ_ERR;
        }
    } while (reg_value != 0x00);

    i2c_release(I2C);

    return RTD_OEM_OK;
}
