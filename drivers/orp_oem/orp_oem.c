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
 * @author      Ting XU <timtsui@outlook.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
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

#define I2C (dev->params.i2c)
#define ADDR (dev->params.addr)
#define IRQ_OPTION (dev->params.irq_option)

/**
 * @brief   Unlocks the ORP_OEM_REG_UNLOCK register to be able to change the
 *          I2C device address, by writing 0x55 and 0xAA to the register
 *
 * @param[in] dev device descriptor
 *
 * @return ORP_OEM_OK on success
 * @return ORP_OEM_WRITE_ERR if writing to the device failed
 */
static int _unlock_address_reg(orp_oem_t *dev);

/**
 * @brief   Setting the ORP OEM interrupt mode to the defined mode provided
 *          in the device descriptor
 *
 * @param[in] dev device descriptor
 *
 * @return ORP_OEM_OK on success
 * @return ORP_OEM_WRITE_ERR if writing to the device failed
 */
static int _set_interrupt_pin(const orp_oem_t *dev);

/**
 * @brief   Polls the ORP_OEM_REG_NEW_READING register as long as it does not
 *          equal 0x01, which indicates that a new pH reading is available.
 *          Polling is done in an interval of 20ms. Estimated completion ~420ms
 *
 * @param[in] dev device descriptor
 *
 * @return ORP_OEM_OK on success
 * @return ORP_OEM_READ_ERR if reading from the register failed
 * @return ORP_OEM_WRITE_ERR if reseting the register failed
 */
static int _new_reading_available(const orp_oem_t *dev);

/**
 * @brief   Sets the PH_OEM_REG_CALIBRATION_BASE register to the pH
 *          @p calibration_value which the device will be calibrated to.
 *
 * @param[in] dev device descriptor
 * @param[in] calibration_value pH value the device will be calibrated to
 *
 * @return PH_OEM_OK on success
 * @return PH_OEM_READ_ERR if reading from the register failed
 * @return PH_OEM_WRITE_ERR if writing the calibration_value to the device failed
 */
static int _set_calibration_value(const orp_oem_t *dev,
                                  uint16_t calibration_value);

int orp_oem_init(orp_oem_t *dev, const orp_oem_params_t *params)
{
    assert(dev && params);

    dev->params = *params;

    uint8_t reg_data;

    i2c_acquire(I2C);

    /* Register read test */
    if (i2c_read_regs(I2C, ADDR, ORP_OEM_REG_DEVICE_TYPE,
                      &reg_data, 1, 0x0) < 0) {
        DEBUG("\n[orp_oem debug] init - error: unable to read reg %x\n",
              ORP_OEM_REG_DEVICE_TYPE);

        i2c_release(I2C);
        return ORP_OEM_NODEV;
    }

    /* Test if the device ID of the attached ORP OEM sensor equals the
     * value of the ORP_OEM_REG_DEVICE_TYPE register
     * */
    if (reg_data != ORP_OEM_DEVICE_TYPE_ID) {
        DEBUG("\n[orp_oem debug] init - error: the attached device is not a ORP OEM "
              "Sensor. Read Device Type ID is: %i\n", reg_data);
        i2c_release(I2C);
        return ORP_OEM_NOT_ORP;
    }
    i2c_release(I2C);

    return ORP_OEM_OK;
}

static int _unlock_address_reg(orp_oem_t *dev)
{
    uint8_t reg_value = 1;

    i2c_acquire(I2C);

    i2c_write_reg(I2C, ADDR, ORP_OEM_REG_UNLOCK, 0x55, 0x0);
    i2c_write_reg(I2C, ADDR, ORP_OEM_REG_UNLOCK, 0xAA, 0x0);
    /* if successfully unlocked the register will equal 0x00 */
    i2c_read_reg(I2C, ADDR, ORP_OEM_REG_UNLOCK, &reg_value, 0x0);

    if (reg_value != 0x00) {
        DEBUG("\n[orp_oem debug] Failed at unlocking I2C address register. \n");
        i2c_release(I2C);
        return ORP_OEM_WRITE_ERR;
    }
    i2c_release(I2C);

    return ORP_OEM_OK;
}

int orp_oem_set_i2c_address(orp_oem_t *dev, uint8_t addr)
{
    assert(dev);

    if (_unlock_address_reg(dev) != ORP_OEM_OK) {
        return ORP_OEM_WRITE_ERR;
    }

    i2c_acquire(I2C);

    if (i2c_write_reg(I2C, ADDR, ORP_OEM_REG_ADDRESS, addr, 0x0) < 0) {
        DEBUG("\n[orp_oem debug] Setting I2C address to %x failed\n", addr);
        i2c_release(I2C);
        return ORP_OEM_WRITE_ERR;
    }

    dev->params.addr = addr;
    i2c_release(I2C);

    return ORP_OEM_OK;
}

static int _set_interrupt_pin(const orp_oem_t *dev)
{
    assert(dev);
    i2c_acquire(I2C);

    if (i2c_write_reg(I2C, ADDR, ORP_OEM_REG_INTERRUPT, IRQ_OPTION, 0x0) < 0) {
        DEBUG("\n[orp_oem debug] Setting interrupt pin to option %d failed.\n", IRQ_OPTION);
        i2c_release(I2C);
        return ORP_OEM_WRITE_ERR;
    }

    i2c_release(I2C);

    return ORP_OEM_OK;
}

int orp_oem_enable_interrupt(orp_oem_t *dev, orp_oem_interrupt_pin_cb_t cb,
                            void *arg)
{
    if (dev->params.interrupt_pin == GPIO_UNDEF) {
        return ORP_OEM_INTERRUPT_GPIO_UNDEF;
    }

    if (_set_interrupt_pin(dev) < 0) {
        return ORP_OEM_WRITE_ERR;
    }

    int gpio_flank = 0;

    switch (IRQ_OPTION) {
        case ORP_OEM_IRQ_FALLING:
            gpio_flank = GPIO_FALLING;
            break;
        case ORP_OEM_IRQ_RISING:
            gpio_flank = GPIO_RISING;
            break;
        case ORP_OEM_IRQ_BOTH:
            gpio_flank = GPIO_BOTH;
            break;
    }

    dev->arg = arg;
    dev->cb = cb;
    if (gpio_init_int(dev->params.interrupt_pin,
                      dev->params.gpio_mode, gpio_flank, cb, arg) < 0) {
        DEBUG("\n[orp_oem debug] Initializing interrupt gpio pin failed.\n");
        return ORP_OEM_GPIO_INIT_ERR;
    }

    return ORP_OEM_OK;
}

int orp_oem_reset_interrupt_pin(const orp_oem_t *dev)
{
    /* no reset needed for mode ORP_OEM_IRQ_BOTH */
    if (dev->params.irq_option == ORP_OEM_IRQ_BOTH) {
        return ORP_OEM_OK;
    }

    if (_set_interrupt_pin(dev) < 0) {
        return ORP_OEM_WRITE_ERR;
    }
    return ORP_OEM_OK;
}

int orp_oem_set_led_state(const orp_oem_t *dev, orp_oem_led_state_t state)
{
    assert(dev);
    i2c_acquire(I2C);

    if (i2c_write_reg(I2C, ADDR, ORP_OEM_REG_LED, state, 0x0) < 0) {
        DEBUG("\n[orp_oem debug] Setting LED state to %d failed.\n", state);
        i2c_release(I2C);
        return ORP_OEM_WRITE_ERR;
    }
    i2c_release(I2C);

    return ORP_OEM_OK;
}

int orp_oem_set_device_state(const orp_oem_t *dev, orp_oem_device_state_t state)
{
    assert(dev);
    i2c_acquire(I2C);

    if (i2c_write_reg(I2C, ADDR, ORP_OEM_REG_HIBERNATE, state, 0x0) < 0) {
        DEBUG("\n[orp_oem debug] Setting device state to %d failed\n", state);
        i2c_release(I2C);
        return ORP_OEM_WRITE_ERR;
    }
    i2c_release(I2C);

    return ORP_OEM_OK;
}

static int _new_reading_available(const orp_oem_t *dev)
{
    int8_t new_reading_available;

    assert(dev);
    i2c_acquire(I2C);
    do {
        if (i2c_read_reg(I2C, ADDR, ORP_OEM_REG_NEW_READING,
                         &new_reading_available, 0x0) < 0) {
            DEBUG("\n[orp_oem debug] Failed at reading ORP_OEM_REG_NEW_READING\n");
            i2c_release(I2C);
            return ORP_OEM_READ_ERR;
        }
        xtimer_usleep(20 * US_PER_MS);
    } while (new_reading_available == 0);

    /* need to manually reset register back to 0x00 */
    if (i2c_write_reg(I2C, ADDR, ORP_OEM_REG_NEW_READING, 0x00, 0x0) < 0) {
        DEBUG("\n[orp_oem debug] Resetting ORP_OEM_REG_NEW_READING failed\n");
        i2c_release(I2C);
        return ORP_OEM_WRITE_ERR;
    }
    i2c_release(I2C);

    return ORP_OEM_OK;
}

int orp_oem_start_new_reading(const orp_oem_t *dev)
{
    if (orp_oem_set_device_state(dev, ORP_OEM_TAKE_READINGS) < 0) {
        return ORP_OEM_WRITE_ERR;
    }

    /* if interrupt pin is undefined, poll till new reading was taken and stop
     * device form taking further readings */
    if (dev->params.interrupt_pin == GPIO_UNDEF) {
        int result = _new_reading_available(dev);
        if (result < 0) {
            return result;
        }

        if (orp_oem_set_device_state(dev, ORP_OEM_STOP_READINGS) < 0) {
            return ORP_OEM_WRITE_ERR;
        }
    }
    return ORP_OEM_OK;
}

int orp_oem_clear_calibration(const orp_oem_t *dev)
{
    uint8_t reg_value;

    assert(dev);
    i2c_acquire(I2C);
    if (i2c_write_reg(I2C, ADDR, ORP_OEM_REG_CALIBRATION_REQUEST, ORP_OEM_CALIBRATE_CLEAR, 0) < 0) {
        DEBUG("\n[orp_oem debug] Clearing calibration failed \n");
        i2c_release(I2C);
        return ORP_OEM_WRITE_ERR;
    }

    do {
        if (i2c_read_reg(I2C, ADDR, ORP_OEM_REG_CALIBRATION_REQUEST, &reg_value,
                         0) < 0) {
            i2c_release(I2C);
            return ORP_OEM_READ_ERR;
        }
    } while (reg_value != 0x00);

    i2c_release(I2C);

    return ORP_OEM_OK;
}

static int _set_calibration_value(const orp_oem_t *dev,
                                  uint16_t calibration_value)
{
    uint8_t reg_value[4];

    reg_value[0] = 0x00;
    reg_value[1] = 0x00;
    reg_value[2] = (uint8_t)(calibration_value >> 8);
    reg_value[3] = (uint8_t)(calibration_value & 0x00FF);

    i2c_acquire(I2C);

    if (i2c_write_regs(I2C, ADDR, ORP_OEM_REG_CALIBRATION_BASE, &reg_value, 4, 0) < 0) {
        DEBUG("\n[orp_oem debug] Writing calibration value failed \n");
        i2c_release(I2C);
        return ORP_OEM_WRITE_ERR;
    }

    /* Calibration is critical, so check if written value is in fact correct */
    if (i2c_read_regs(I2C, ADDR, ORP_OEM_REG_CALIBRATION_BASE, &reg_value, 4, 0) < 0) {
        DEBUG("\n[orp_oem debug] Reading the calibration value failed \n");
        i2c_release(I2C);
        return ORP_OEM_READ_ERR;
    }

    uint16_t confirm_value = (int16_t)(reg_value[2] << 8)
                             | (int16_t)(reg_value[3]);

    if (confirm_value != calibration_value) {
        DEBUG("\n[orp_oem debug] Setting calibration register to ORP raw %d "
              "failed \n", calibration_value);
        i2c_release(I2C);
        return ORP_OEM_WRITE_ERR;
    }

    i2c_release(I2C);

    return ORP_OEM_OK;
}

int orp_oem_set_calibration(const orp_oem_t *dev, uint16_t calibration_value,
                           orp_oem_calibration_option_t option)
{
    assert(dev);

    if (_set_calibration_value(dev, calibration_value) != ORP_OEM_OK) {
        return ORP_OEM_WRITE_ERR;
    }

    uint8_t reg_value;

    i2c_acquire(I2C);

    if (i2c_write_reg(I2C, ADDR, ORP_OEM_REG_CALIBRATION_REQUEST,
                      option, 0) < 0) {
        DEBUG("\n[orp_oem debug] Sending calibration request failed\n");
        return ORP_OEM_WRITE_ERR;
    }

    do {
        if (i2c_read_reg(I2C, ADDR, ORP_OEM_REG_CALIBRATION_REQUEST, &reg_value,
                         0) < 0) {
            DEBUG("\n[orp_oem debug] Reading calibration request status failed\n");
            i2c_release(I2C);
            return ORP_OEM_READ_ERR;
        }
    } while (reg_value != 0x00);

    i2c_release(I2C);

    return ORP_OEM_OK;
}

int orp_oem_read_calibration_state(const orp_oem_t *dev, uint16_t *calibration_state)
{
    assert(dev);
    i2c_acquire(I2C);

    if (i2c_read_reg(I2C, ADDR, ORP_OEM_REG_CALIBRATION_CONFIRM,
                     calibration_state, 0) < 0) {
        DEBUG("\n[orp_oem debug] Failed at reading calibration confirm register\n");
        i2c_release(I2C);
        return ORP_OEM_READ_ERR;
    }
    i2c_release(I2C);
    return ORP_OEM_OK;
}

int orp_oem_read_orp(const orp_oem_t *dev, int16_t *orp_value)
{
    uint8_t reg_value[4];

    assert(dev);
    i2c_acquire(I2C);

    if (i2c_read_regs(I2C, ADDR, ORP_OEM_REG_ORP_READING_BASE,
                      &reg_value, 4, 0) < 0) {
        DEBUG("[orp_oem debug] Getting ORP value failed\n");
        i2c_release(I2C);
        return ORP_OEM_READ_ERR;
    }
    *orp_value = (int16_t)(reg_value[2] << 8) | (int16_t)(reg_value[3]);

    i2c_release(I2C);

    return ORP_OEM_OK;
}
