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
 * @author      Ting XU <timtsui@outlook.com>
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

/**
 * @brief   Unlocks the RTD_OEM_REG_UNLOCK register to be able to change the
 *          I2C device address, by writing 0x55 and 0xAA to the register
 *
 * @param[in] dev device descriptor
 *
 * @return RTD_OEM_OK on success
 * @return RTD_OEM_WRITE_ERR if writing to the device failed
 */
static int _unlock_address_reg(rtd_oem_t *dev);

/**
 * @brief   Setting the RTD OEM interrupt mode to the defined mode provided
 *          in the device descriptor
 *
 * @param[in] dev device descriptor
 *
 * @return RTD_OEM_OK on success
 * @return RTD_OEM_WRITE_ERR if writing to the device failed
 */
static int _set_interrupt_pin(const rtd_oem_t *dev);

/**
 * @brief   Sets the RTD_OEM_REG_CALIBRATION_BASE register to the RTD
 *          @p calibration_value which the device will be calibrated to.
 *
 * @param[in] dev device descriptor
 * @param[in] calibration_value pH value the device will be calibrated to
 *
 * @return RTD_OEM_OK on success
 * @return RTD_OEM_READ_ERR if reading from the register failed
 * @return RTD_OEM_WRITE_ERR if writing the calibration_value to the device failed
 */
static int _set_calibration_value(const rtd_oem_t *dev,
                                  uint32_t calibration_value);

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

int rtd_oem_reset_interrupt_pin(const rtd_oem_t *dev)
{
    /* no reset needed for mode RTD_OEM_IRQ_BOTH */
    if (dev->params.irq_option == RTD_OEM_IRQ_BOTH) {
        return RTD_OEM_OK;
    }

    if (_set_interrupt_pin(dev) < 0) {
        return RTD_OEM_WRITE_ERR;
    }
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

static int _set_calibration_value(const rtd_oem_t *dev,
                                  uint32_t calibration_value)
{
    uint8_t reg_value[4];

    reg_value[0] = 0x00;
    reg_value[1] = (uint8_t)(calibration_value >> 16);
    reg_value[2] = (uint8_t)(calibration_value >> 8);
    reg_value[3] = (uint8_t)(calibration_value & 0x000000FF);

    i2c_acquire(I2C);

    if (i2c_write_regs(I2C, ADDR, RTD_OEM_REG_CALIBRATION_BASE, &reg_value, 4, 0) < 0) {
        DEBUG("\n[rtd_oem debug] Writing calibration value failed \n");
        i2c_release(I2C);
        return RTD_OEM_WRITE_ERR;
    }

    /* Calibration is critical, so check if written value is in fact correct */
    if (i2c_read_regs(I2C, ADDR, RTD_OEM_REG_CALIBRATION_BASE, &reg_value, 4, 0) < 0) {
        DEBUG("\n[rtd_oem debug] Reading the calibration value failed \n");
        i2c_release(I2C);
        return RTD_OEM_READ_ERR;
    }

    uint32_t confirm_value = (int32_t)(reg_value[1] << 16)
    		                 | (int32_t)(reg_value[2] << 8)
                             | (int32_t)(reg_value[3]);

    if (confirm_value != calibration_value) {
        DEBUG("\n[rtd_oem debug] Setting calibration register to RTD raw %lu "
              "failed \n", calibration_value);
        i2c_release(I2C);
        return RTD_OEM_WRITE_ERR;
    }

    i2c_release(I2C);

    return RTD_OEM_OK;
}

int rtd_oem_set_calibration(const rtd_oem_t *dev, uint32_t calibration_value,
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

static int _set_interrupt_pin(const rtd_oem_t *dev)
{
    assert(dev);
    i2c_acquire(I2C);

    if (i2c_write_reg(I2C, ADDR, RTD_OEM_REG_INTERRUPT, IRQ_OPTION, 0x0) < 0) {
        DEBUG("\n[rtd_oem debug] Setting interrupt pin to option %d failed.\n", IRQ_OPTION);
        i2c_release(I2C);
        return RTD_OEM_WRITE_ERR;
    }

    i2c_release(I2C);

    return RTD_OEM_OK;
}

int rtd_oem_enable_interrupt(rtd_oem_t *dev, rtd_oem_interrupt_pin_cb_t cb,
                            void *arg)
{
    if (dev->params.interrupt_pin == GPIO_UNDEF) {
        return RTD_OEM_INTERRUPT_GPIO_UNDEF;
    }

    if (_set_interrupt_pin(dev) < 0) {
        return RTD_OEM_WRITE_ERR;
    }

    int gpio_flank = 0;

    switch (IRQ_OPTION) {
        case RTD_OEM_IRQ_FALLING:
            gpio_flank = GPIO_FALLING;
            break;
        case RTD_OEM_IRQ_RISING:
            gpio_flank = GPIO_RISING;
            break;
        case RTD_OEM_IRQ_BOTH:
            gpio_flank = GPIO_BOTH;
            break;
    }

    dev->arg = arg;
    dev->cb = cb;
    if (gpio_init_int(dev->params.interrupt_pin,
                      dev->params.gpio_mode, gpio_flank, cb, arg) < 0) {
        DEBUG("\n[ph_oem debug] Initializing interrupt gpio pin failed.\n");
        return RTD_OEM_GPIO_INIT_ERR;
    }

    return RTD_OEM_OK;
}

int rtd_oem_set_device_state(const rtd_oem_t *dev, rtd_oem_device_state_t state)
{
    assert(dev);
    i2c_acquire(I2C);

    if (i2c_write_reg(I2C, ADDR, RTD_OEM_REG_HIBERNATE, state, 0x0) < 0) {
        DEBUG("\n[rtd_oem debug] Setting device state to %d failed\n", state);
        i2c_release(I2C);
        return RTD_OEM_WRITE_ERR;
    }
    i2c_release(I2C);

    return RTD_OEM_OK;
}

static int _new_reading_available(const rtd_oem_t *dev)
{
    int8_t new_reading_available;

    assert(dev);
    i2c_acquire(I2C);
    do {
        if (i2c_read_reg(I2C, ADDR, RTD_OEM_REG_NEW_READING,
                         &new_reading_available, 0x0) < 0) {
            DEBUG("\n[rtd_oem debug] Failed at reading RTD_OEM_REG_NEW_READING\n");
            i2c_release(I2C);
            return RTD_OEM_READ_ERR;
        }
        xtimer_usleep(20 * US_PER_MS);
    } while (new_reading_available == 0);

    /* need to manually reset register back to 0x00 */
    if (i2c_write_reg(I2C, ADDR, RTD_OEM_REG_NEW_READING, 0x00, 0x0) < 0) {
        DEBUG("\n[rtd_oem debug] Resetting RTD_OEM_REG_NEW_READING failed\n");
        i2c_release(I2C);
        return RTD_OEM_WRITE_ERR;
    }
    i2c_release(I2C);

    return RTD_OEM_OK;
}

int rtd_oem_start_new_reading(const rtd_oem_t *dev)
{
    if (rtd_oem_set_device_state(dev, RTD_OEM_TAKE_READINGS) < 0) {
        return RTD_OEM_WRITE_ERR;
    }

    /* if interrupt pin is undefined, poll till new reading was taken and stop
     * device form taking further readings */
    if (dev->params.interrupt_pin == GPIO_UNDEF) {
        int result = _new_reading_available(dev);
        if (result < 0) {
            return result;
        }

        if (rtd_oem_set_device_state(dev, RTD_OEM_STOP_READINGS) < 0) {
            return RTD_OEM_WRITE_ERR;
        }
    }
    return RTD_OEM_OK;
}

int rtd_oem_read_temp(const rtd_oem_t *dev, uint32_t *rtd_value)
{
    uint8_t reg_value[4];

    assert(dev);
    i2c_acquire(I2C);

    if (i2c_read_regs(I2C, ADDR, RTD_OEM_REG_RTD_READING_BASE,
                      &reg_value, 4, 0) < 0) {
        DEBUG("[rtd_oem debug] Getting rtd value failed\n");
        i2c_release(I2C);
        return RTD_OEM_READ_ERR;
    }
    *rtd_value = (int32_t)(reg_value[1] << 6) |(int32_t)(reg_value[2] << 8) | (int32_t)(reg_value[3]);

    i2c_release(I2C);

    return RTD_OEM_OK;
}


