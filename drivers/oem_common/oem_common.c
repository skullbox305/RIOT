/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_oem_common
 * @{
 *
 * @file
 * @brief       ORP OEM device driver
 *
 * @author      Ting XU <timtsui@outlook.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 * @}
 */

#include "xtimer.h"
#include "assert.h"
#include "periph/i2c.h"
#include "periph/gpio.h"

#include "oem_common.h"
#include "oem_common_regs.h"

#define ENABLE_DEBUG    (1)
#include "debug.h"

#define DEV_I2C (dev->params.i2c)
#define ADDR (dev->params.addr)
#define IRQ_OPTION (dev->params.irq_option)

/**
 * @brief   Unlocks the OEM_COMMON_REG_UNLOCK register to be able to change the
 *          I2C device address
 *
 * @param[in] dev            device descriptor
 *
 * @return OEM_COMMON_OK         on success
 * @return OEM_COMMON_WRITE_ERR if writing to the device failed
 */
static int _unlock_address_reg(oem_common_dev_t *dev);

/**
 * @brief   Setting the XXX_OEM interrupt mode to the defined mode provided
 *          in the device descriptor
 *
 * @param[in] dev              device descriptor
 *
 * @return OEM_COMMON_OK        on success
 * @return OEM_COMMON_WRITE_ERR if writing to the device failed
 */
static int _set_interrupt_pin(const oem_common_dev_t *dev);
//
///**
// * @brief   Polls the OEM_COMMON_REG_NEW_READING register as long as it does not
// *          equal 0x01, which indicates that a new reading is available.
// *          Polling is done in an interval of 20ms. Estimated completion ~420ms
// *
// * @param[in] dev              device descriptor
// *
// * @return OEM_COMMON_OK		 on success
// * @return OEM_COMMON_READ_ERR  if reading from the register failed
// * @return OEM_COMMON_WRITE_ERR if reseting the register failed
// */
//static int _new_reading_available(const oem_common_dev_t *dev);
//


int oem_common_init(oem_common_dev_t *dev, const oem_common_params_t *params)
{
    assert(dev && params);

    dev->params = *params;

    uint8_t reg_data;

    i2c_acquire(DEV_I2C);

    /* Register read test */
    if (i2c_read_regs(DEV_I2C, ADDR, OEM_COMMON_REG_DEVICE_TYPE,
                      &reg_data, 1, 0x0) < 0) {
        DEBUG("\n[oem_common debug] init - error: unable to read reg %x\n",
              OEM_COMMON_REG_DEVICE_TYPE);

        i2c_release(DEV_I2C);
        return OEM_COMMON_NODEV;
    }

    /* Test if the device ID of the attached XXX_OEM sensor equals the
     * value of the OEM_COMMON_REG_DEVICE_TYPE register
     * */
    if (reg_data != dev->params.device_type_id) {
        DEBUG("\n[oem_common debug] init - error: the attached device is not a ORP OEM "
              "Sensor. Read Device Type ID is: %i\n",
              reg_data);
        i2c_release(DEV_I2C);
        return OEM_COMMON_WRONG_DEV;
    }
    i2c_release(DEV_I2C);

    return OEM_COMMON_OK;
}

int oem_common_write_reg(const oem_common_dev_t *dev, uint8_t reg, uint8_t in)
{
    i2c_acquire(DEV_I2C);

    if (i2c_write_reg(DEV_I2C, ADDR, reg, in, 0x0) < 0) {
        i2c_release(DEV_I2C);
        return OEM_COMMON_WRITE_ERR;
    }
    i2c_release(DEV_I2C);

    return OEM_COMMON_OK;
}

int oem_common_read_reg(const oem_common_dev_t *dev, uint8_t reg, uint8_t *out)
{
    i2c_acquire(DEV_I2C);

    if (i2c_read_reg(DEV_I2C, ADDR, reg, out, 0) < 0) {
        i2c_release(DEV_I2C);
        return OEM_COMMON_READ_ERR;
    }
    i2c_release(DEV_I2C);
    return OEM_COMMON_OK;
}

int oem_common_write_reg32(const oem_common_dev_t *dev, uint8_t reg,
                           uint32_t in)
{
    uint8_t reg_values[4];

    reg_values[0] = (uint8_t)(in >> 24);
    reg_values[1] = (uint8_t)(in >> 16);
    reg_values[2] = (uint8_t)(in >> 8);
    reg_values[3] = (uint8_t)(in & 0x00FF);

    i2c_acquire(DEV_I2C);

    if (i2c_write_regs(DEV_I2C, ADDR, reg, &reg_values, 4, 0) < 0) {
        i2c_release(DEV_I2C);
        return OEM_COMMON_WRITE_ERR;
    }
    i2c_release(DEV_I2C);

    return OEM_COMMON_OK;
}

int oem_common_read_reg32(const oem_common_dev_t *dev, uint8_t reg, bool neg,
                          void *out)
{
    uint8_t reg_values[4];

    i2c_acquire(DEV_I2C);

    if (i2c_read_regs(DEV_I2C, ADDR, reg, &reg_values, 4, 0) < 0) {
        i2c_release(DEV_I2C);
        return OEM_COMMON_WRITE_ERR;
    }

    if (neg) {
        *(uint32_t *)out =  (uint32_t)(reg_values[0] << 24)
                          | (uint32_t)(reg_values[1] << 16)
                          | (uint32_t)(reg_values[2] << 8)
                          | (uint32_t)(reg_values[3]);
    }
    else {
        *(uint32_t *)out =  (int32_t)(reg_values[0] << 24)
                          | (int32_t)(reg_values[1] << 16)
                          | (int32_t)(reg_values[2] << 8)
                          | (int32_t)(reg_values[3]);
    }

    i2c_release(DEV_I2C);

    return OEM_COMMON_OK;
}

int oem_common_set_led_state(const oem_common_dev_t *dev,
                             oem_common_led_state_t state)
{
    assert(dev);
    return oem_common_write_reg(dev, OEM_COMMON_REG_LED, state);
}

static int _unlock_address_reg(oem_common_dev_t *dev)
{
    uint8_t reg_value = 1;

    i2c_acquire(DEV_I2C);

    i2c_write_reg(DEV_I2C, ADDR, OEM_COMMON_REG_UNLOCK, 0x55, 0x0);
    i2c_write_reg(DEV_I2C, ADDR, OEM_COMMON_REG_UNLOCK, 0xAA, 0x0);
    /* if successfully unlocked the register will equal 0x00 */
    i2c_read_reg(DEV_I2C, ADDR, OEM_COMMON_REG_UNLOCK, &reg_value, 0x0);

    if (reg_value != 0x00) {
        DEBUG(
            "\n[oem_common debug] Failed at unlocking I2C address register. \n");
        i2c_release(DEV_I2C);
        return OEM_COMMON_WRITE_ERR;
    }
    i2c_release(DEV_I2C);

    return OEM_COMMON_OK;
}

int oem_common_set_i2c_address(oem_common_dev_t *dev, uint8_t addr)
{
    assert(dev);

    if (_unlock_address_reg(dev) != OEM_COMMON_OK) {
        return OEM_COMMON_WRITE_ERR;
    }

    i2c_acquire(DEV_I2C);

    if (i2c_write_reg(DEV_I2C, ADDR, OEM_COMMON_REG_ADDRESS, addr, 0x0) < 0) {
        DEBUG("\n[oem_common debug] Setting I2C address to %x failed\n", addr);
        i2c_release(DEV_I2C);
        return OEM_COMMON_WRITE_ERR;
    }

    dev->params.addr = addr;
    i2c_release(DEV_I2C);

    return OEM_COMMON_OK;
}

int oem_common_set_calibration_value(const oem_common_dev_t *dev, uint8_t reg,
                                     uint32_t calibration_value)
{
    assert(dev);
    uint8_t reg_value[4];

    reg_value[0] = (uint8_t)(calibration_value >> 24);
    reg_value[1] = (uint8_t)(calibration_value >> 16);
    reg_value[2] = (uint8_t)(calibration_value >> 8);
    reg_value[3] = (uint8_t)(calibration_value & 0x00FF);

    i2c_acquire(DEV_I2C);

    if (i2c_write_regs(DEV_I2C, ADDR, reg, &reg_value, 4, 0) < 0) {
        DEBUG("\n[oem_common debug] Writing calibration value failed \n");
        i2c_release(DEV_I2C);
        return OEM_COMMON_WRITE_ERR;
    }

    /* Calibration is critical, so check if written value is in fact correct */
    if (i2c_read_regs(DEV_I2C, ADDR, reg, &reg_value, 4, 0) < 0) {
        DEBUG("\n[oem_common debug] Reading the calibration value failed \n");
        i2c_release(DEV_I2C);
        return OEM_COMMON_READ_ERR;
    }

    uint32_t confirm_value =   (uint32_t)(reg_value[0] << 24)
                             | (uint32_t)(reg_value[1] << 16)
                             | (uint32_t)(reg_value[2] << 8)
                             | (uint32_t)(reg_value[3]);

//    printf("\nconfirm value: %ld\n", confirm_value);
//    printf("\ncal value: %ld\n", calibration_value);

    if (confirm_value != calibration_value) {
        DEBUG("\n[oem_common debug] Setting calibration register to %ld "
              "failed \n", calibration_value);
        i2c_release(DEV_I2C);
        return OEM_COMMON_WRITE_ERR;
    }

    i2c_release(DEV_I2C);

    return OEM_COMMON_OK;
}

int oem_common_set_calibration(const oem_common_dev_t *dev, uint8_t reg,
                               uint8_t option, bool write_only)
{
    assert(dev);
    uint8_t reg_value;

    i2c_acquire(DEV_I2C);
    if (i2c_write_reg(DEV_I2C, ADDR, reg, option, 0) < 0) {
        DEBUG("\n[oem_common debug] Setting calibration failed \n");
        i2c_release(DEV_I2C);
        return OEM_COMMON_WRITE_ERR;
    }

    if (write_only) {
        xtimer_usleep(50 * US_PER_MS);
    }
    else {
        do {
            if (i2c_read_reg(DEV_I2C, ADDR, reg, &reg_value, 0) < 0) {
                DEBUG(
                    "\n[oem_common debug] Reading calibration request status failed \n");
                i2c_release(DEV_I2C);
                return OEM_COMMON_READ_ERR;
            }
        } while (reg_value != 0x00);
    }

    i2c_release(DEV_I2C);

    return OEM_COMMON_OK;
}

static int _set_interrupt_pin(const oem_common_dev_t *dev)
{
    assert(dev);
    i2c_acquire(DEV_I2C);

    if (i2c_write_reg(DEV_I2C, ADDR, OEM_COMMON_REG_INTERRUPT, IRQ_OPTION,
                      0x0) < 0) {
        DEBUG(
            "\n[oem_common debug] Setting interrupt pin to option %d failed.\n",
            IRQ_OPTION);
        i2c_release(DEV_I2C);
        return OEM_COMMON_WRITE_ERR;
    }

    i2c_release(DEV_I2C);

    return OEM_COMMON_OK;
}

int oem_common_enable_interrupt(oem_common_dev_t *dev,
                                oem_common_interrupt_pin_cb_t cb,
                                void *arg)
{
    if (dev->params.interrupt_pin == GPIO_UNDEF) {
        return OEM_COMMON_INT_GPIO_UNDEF;
    }

    if (_set_interrupt_pin(dev) < 0) {
        return OEM_COMMON_WRITE_ERR;
    }

    int gpio_flank = 0;

    switch (IRQ_OPTION) {
        case OEM_COMMON_IRQ_FALLING:
            gpio_flank = GPIO_FALLING;
            break;
        case OEM_COMMON_IRQ_RISING:
            gpio_flank = GPIO_RISING;
            break;
        case OEM_COMMON_IRQ_BOTH:
            gpio_flank = GPIO_BOTH;
            break;
    }

    dev->cb = cb;
    if (gpio_init_int(dev->params.interrupt_pin,
                      dev->params.gpio_mode, gpio_flank, cb, arg) < 0) {
        DEBUG("\n[oem_common debug] Initializing interrupt gpio pin failed.\n");
        return OEM_COMMON_GPIO_INIT_ERR;
    }

    return OEM_COMMON_OK;
}

int oem_common_reset_interrupt_pin(const oem_common_dev_t *dev)
{
    /* no reset needed for mode OEM_COMMON_IRQ_BOTH */
    if (dev->params.irq_option == OEM_COMMON_IRQ_BOTH) {
        return OEM_COMMON_OK;
    }

    if (_set_interrupt_pin(dev) < 0) {
        return OEM_COMMON_WRITE_ERR;
    }
    return OEM_COMMON_OK;
}

int oem_common_set_device_state(const oem_common_dev_t *dev,
                                oem_common_device_state_t state)
{
    assert(dev);
    return oem_common_write_reg(dev, OEM_COMMON_REG_HIBERNATE, state);
}

static int _new_reading_available(const oem_common_dev_t *dev)
{
    int8_t new_reading_available;

    assert(dev);
    i2c_acquire(DEV_I2C);
    do {
        if (i2c_read_reg(DEV_I2C, ADDR, OEM_COMMON_REG_NEW_READING,
                         &new_reading_available, 0x0) < 0) {
            DEBUG(
                "\n[oem_common debug] Failed at reading OEM_COMMON_REG_NEW_READING\n");
            i2c_release(DEV_I2C);
            return OEM_COMMON_READ_ERR;
        }
        xtimer_usleep(20 * US_PER_MS);
    } while (new_reading_available == 0);

    /* need to manually reset register back to 0x00 */
    if (i2c_write_reg(DEV_I2C, ADDR, OEM_COMMON_REG_NEW_READING, 0x00,
                      0x0) < 0) {
        DEBUG(
            "\n[oem_common debug] Resetting OEM_COMMON_REG_NEW_READING failed\n");
        i2c_release(DEV_I2C);
        return OEM_COMMON_WRITE_ERR;
    }
    i2c_release(DEV_I2C);

    return OEM_COMMON_OK;
}

int oem_common_start_new_reading(const oem_common_dev_t *dev)
{
    if (oem_common_set_device_state(dev, OEM_COMMON_TAKE_READINGS) < 0) {
        return OEM_COMMON_WRITE_ERR;
    }

    /* if interrupt pin is undefined, poll till new reading was taken and stop
     * device form taking further readings */
    if (dev->params.interrupt_pin == GPIO_UNDEF) {
        int result = _new_reading_available(dev);
        if (result < 0) {
            return result;
        }

        if (oem_common_set_device_state(dev, OEM_COMMON_STOP_READINGS) < 0) {
            return OEM_COMMON_WRITE_ERR;
        }
    }
    return OEM_COMMON_OK;
}
