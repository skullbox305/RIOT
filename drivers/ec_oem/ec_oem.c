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

#include "include/ec_oem_params.h"
#include "include/ec_oem_regs.h"
#include "periph/i2c.h"
#include "periph/gpio.h"

#include "ec_oem.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

#define I2C (dev->params.i2c)
#define ADDR (dev->params.addr)
#define IRQ_OPTION (dev->params.irq_option)

/**
 * @brief   Unlocks the EC_OEM_REG_UNLOCK register to be able to change the
 *          I2C device address, by writing 0x55 and 0xAA to the register
 *
 * @param[in] dev device descriptor
 *
 * @return EC_OEM_OK on success
 * @return EC_OEM_WRITE_ERR if writing to the device failed
 */
static int _unlock_address_reg(ec_oem_t *dev);

/**
 * @brief   Setting the OEM OEM interrupt mode to the defined mode provided
 *          in the device descriptor
 *
 * @param[in] dev device descriptor
 *
 * @return EC_OEM_OK on success
 * @return EC_OEM_WRITE_ERR if writing to the device failed
 */
static int _set_interrupt_pin(const ec_oem_t *dev);

/**
 * @brief   Polls the EC_OEM_REG_NEW_READING register as long as it does not
 *          equal 0x01, which indicates that a new EC reading is available.
 *          Polling is done in an interval of 20ms. Estimated completion ~640ms
 *
 * @param[in] dev device descriptor
 *
 * @return EC_OEM_OK on success
 * @return EC_OEM_READ_ERR if reading from the register failed
 * @return EC_OEM_WRITE_ERR if reseting the register failed
 */
static int _new_reading_available(const ec_oem_t *dev);

/**
 * @brief   Sets the EC_OEM_REG_CALIBRATION_BASE register to the EC
 *          @p calibration_value which the device will be calibrated to.
 *
 * @param[in] dev device descriptor
 * @param[in] calibration_value EC value the device will be calibrated to
 *
 * @return EC_OEM_OK on success
 * @return EC_OEM_READ_ERR if reading from the register failed
 * @return EC_OEM_WRITE_ERR if writing the calibration_value to the device failed
 */
static int _set_calibration_value(const ec_oem_t *dev,
                                  uint16_t calibration_value);

int ec_oem_init(ec_oem_t *dev, const ec_oem_params_t *params)
{
    assert(dev && params);

    dev->params = *params;

    uint8_t reg_data;

    i2c_acquire(I2C);

    /* Register read test */
    if (i2c_read_regs(I2C, ADDR, EC_OEM_REG_DEVICE_TYPE,
                      &reg_data, 1, 0x0) < 0) {
        DEBUG("\n[ec_oem debug] init - error: unable to read reg %x\n",
              EC_OEM_REG_DEVICE_TYPE);

        i2c_release(I2C);
        return EC_OEM_NODEV;
    }

    /* Test if the device ID of the attached EC OEM sensor equals the
     * value of the EC_OEM_REG_DEVICE_TYPE register
     * */
    if (reg_data != EC_OEM_DEVICE_TYPE_ID) {
        DEBUG("\n[ec_oem debug] init - error: the attached device is not a EC OEM "
              "Sensor. Read Device Type ID is: %i\n", reg_data);
        i2c_release(I2C);
        return EC_OEM_NOT_PH;
    }
    i2c_release(I2C);

    return EC_OEM_OK;
}

static int _unlock_address_reg(ec_oem_t *dev)
{
    uint8_t reg_value = 1;

    i2c_acquire(I2C);

    i2c_write_reg(I2C, ADDR, EC_OEM_REG_UNLOCK, 0x55, 0x0);
    i2c_write_reg(I2C, ADDR, EC_OEM_REG_UNLOCK, 0xAA, 0x0);
    /* if successfully unlocked the register will equal 0x00 */
    i2c_read_reg(I2C, ADDR, EC_OEM_REG_UNLOCK, &reg_value, 0x0);

    if (reg_value != 0x00) {
        DEBUG("\n[ec_oem debug] Failed at unlocking I2C address register. \n");
        i2c_release(I2C);
        return EC_OEM_WRITE_ERR;
    }
    i2c_release(I2C);

    return EC_OEM_OK;
}

int ec_oem_set_i2c_address(ec_oem_t *dev, uint8_t addr)
{
    assert(dev);

    if (_unlock_address_reg(dev) != EC_OEM_OK) {
        return EC_OEM_WRITE_ERR;
    }

    i2c_acquire(I2C);

    if (i2c_write_reg(I2C, ADDR, EC_OEM_REG_ADDRESS, addr, 0x0) < 0) {
        DEBUG("\n[ec_oem debug] Setting I2C address to %x failed\n", addr);
        i2c_release(I2C);
        return EC_OEM_WRITE_ERR;
    }

    dev->params.addr = addr;
    i2c_release(I2C);

    return EC_OEM_OK;
}

static int _set_interrupt_pin(const ec_oem_t *dev)
{
    assert(dev);
    i2c_acquire(I2C);

    if (i2c_write_reg(I2C, ADDR, EC_OEM_REG_INTERRUPT, IRQ_OPTION, 0x0) < 0) {
        DEBUG("\n[ec_oem debug] Setting interrupt pin to option %d failed.\n", IRQ_OPTION);
        i2c_release(I2C);
        return EC_OEM_WRITE_ERR;
    }

    i2c_release(I2C);

    return EC_OEM_OK;
}

int ec_oem_enable_interrupt(ec_oem_t *dev, ec_oem_interrupt_pin_cb_t cb,
                            void *arg)
{
    if (dev->params.interrupt_pin == GPIO_UNDEF) {
        return EC_OEM_INTERRUPT_GPIO_UNDEF;
    }

    if (_set_interrupt_pin(dev) < 0) {
        return EC_OEM_WRITE_ERR;
    }

    int gpio_flank = 0;

    switch (IRQ_OPTION) {
        case EC_OEM_IRQ_FALLING:
            gpio_flank = GPIO_FALLING;
            break;
        case EC_OEM_IRQ_RISING:
            gpio_flank = GPIO_RISING;
            break;
        case EC_OEM_IRQ_BOTH:
            gpio_flank = GPIO_BOTH;
            break;
    }

    dev->arg = arg;
    dev->cb = cb;
    if (gpio_init_int(dev->params.interrupt_pin,
                      dev->params.gpio_mode, gpio_flank, cb, arg) < 0) {
        DEBUG("\n[ec_oem debug] Initializing interrupt gpio pin failed.\n");
        return EC_OEM_GPIO_INIT_ERR;
    }

    return EC_OEM_OK;
}

int ec_oem_reset_interrupt_pin(const ec_oem_t *dev)
{
    /* no reset needed for mode EC_OEM_IRQ_BOTH */
    if (dev->params.irq_option == EC_OEM_IRQ_BOTH) {
        return EC_OEM_OK;
    }

    if (_set_interrupt_pin(dev) < 0) {
        return EC_OEM_WRITE_ERR;
    }
    return EC_OEM_OK;
}

int ec_oem_set_led_state(const ec_oem_t *dev, ec_oem_led_state_t state)
{
    assert(dev);
    i2c_acquire(I2C);

    if (i2c_write_reg(I2C, ADDR, EC_OEM_REG_LED, state, 0x0) < 0) {
        DEBUG("\n[ec_oem debug] Setting LED state to %d failed.\n", state);
        i2c_release(I2C);
        return EC_OEM_WRITE_ERR;
    }
    i2c_release(I2C);

    return EC_OEM_OK;
}

int ec_oem_set_device_state(const ec_oem_t *dev, ec_oem_device_state_t state)
{
    assert(dev);
    i2c_acquire(I2C);

    if (i2c_write_reg(I2C, ADDR, EC_OEM_REG_HIBERNATE, state, 0x0) < 0) {
        DEBUG("\n[ec_oem debug] Setting device state to %d failed\n", state);
        i2c_release(I2C);
        return EC_OEM_WRITE_ERR;
    }
    i2c_release(I2C);

    return EC_OEM_OK;
}

static int _new_reading_available(const ec_oem_t *dev)
{
    int8_t new_reading_available;

    assert(dev);
    i2c_acquire(I2C);
    do {
        if (i2c_read_reg(I2C, ADDR, EC_OEM_REG_NEW_READING,
                         &new_reading_available, 0x0) < 0) {
            DEBUG("\n[ec_oem debug] Failed at reading EC_OEM_REG_NEW_READING\n");
            i2c_release(I2C);
            return EC_OEM_READ_ERR;
        }
        xtimer_usleep(20 * US_PER_MS);
    } while (new_reading_available == 0);

    /* need to manually reset register back to 0x00 */
    if (i2c_write_reg(I2C, ADDR, EC_OEM_REG_NEW_READING, 0x00, 0x0) < 0) {
        DEBUG("\n[ec_oem debug] Resetting EC_OEM_REG_NEW_READING failed\n");
        i2c_release(I2C);
        return EC_OEM_WRITE_ERR;
    }
    i2c_release(I2C);

    return EC_OEM_OK;
}

int ec_oem_start_new_reading(const ec_oem_t *dev)
{
    if (ec_oem_set_device_state(dev, EC_OEM_TAKE_READINGS) < 0) {
        return EC_OEM_WRITE_ERR;
    }

    /* if interrupt pin is undefined, poll till new reading was taken and stop
     * device form taking further readings */
    if (dev->params.interrupt_pin == GPIO_UNDEF) {
        int result = _new_reading_available(dev);
        if (result < 0) {
            return result;
        }

        if (ec_oem_set_device_state(dev, EC_OEM_STOP_READINGS) < 0) {
            return EC_OEM_WRITE_ERR;
        }
    }
    return EC_OEM_OK;
}

int ec_oem_set_probe_type (ec_oem_t *dev, uint16_t probe_type)
{
	uint8_t reg_value[2];

	    reg_value[0] = (uint8_t)(probe_type >> 8);
	    reg_value[1] = (uint8_t)(probe_type & 0x00FF);

	    i2c_acquire(I2C);

	    if (i2c_write_regs(I2C, ADDR, EC_OEM_REG_SET_PROBE_TYPE, &reg_value, 2, 0) < 0) {
	        DEBUG("\n[ec_oem debug] Writing calibration value failed \n");
	        i2c_release(I2C);
	        return EC_OEM_WRITE_ERR;
	    }

	    if (i2c_read_regs(I2C, ADDR, EC_OEM_REG_SET_PROBE_TYPE, &reg_value, 2, 0) < 0) {
	        DEBUG("\n[ec_oem debug] Reading the calibration value failed \n");
	        i2c_release(I2C);
	        return EC_OEM_READ_ERR;
	    }

	    /* Check if written value is in fact correct */

	    uint16_t confirm_value = (int16_t)(reg_value[0] << 8)
	                             | (int16_t)(reg_value[1]);

	    if (confirm_value != probe_type) {
	        DEBUG("\n[ec_oem debug] Setting probe type register to EC raw %d "
	              "failed \n", probe_type);
	        i2c_release(I2C);
	        return EC_OEM_WRITE_ERR;
	    }

	    i2c_release(I2C);

	    return EC_OEM_OK;
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

static int _set_calibration_value(const ec_oem_t *dev,
                                  uint32_t calibration_value)
{
    uint8_t reg_value[4];

    reg_value[0] = 0x00;
    reg_value[1] = (uint8_t)(calibration_value >> 16);
    reg_value[2] = (uint8_t)(calibration_value >> 8);
    reg_value[3] = (uint8_t)(calibration_value & 0x000000FF);

    i2c_acquire(I2C);

    if (i2c_write_regs(I2C, ADDR, EC_OEM_REG_CALIBRATION_BASE, &reg_value, 4, 0) < 0) {
        DEBUG("\n[ec_oem debug] Writing calibration value failed \n");
        i2c_release(I2C);
        return EC_OEM_WRITE_ERR;
    }

    /* Calibration is critical, so check if written value is in fact correct */
    if (i2c_read_regs(I2C, ADDR, EC_OEM_REG_CALIBRATION_BASE, &reg_value, 4, 0) < 0) {
        DEBUG("\n[ec_oem debug] Reading the calibration value failed \n");
        i2c_release(I2C);
        return EC_OEM_READ_ERR;
    }

    uint32_t confirm_value = (int32_t)(reg_value[1] << 16)
        		                 | (int32_t)(reg_value[2] << 8)
                                 | (int32_t)(reg_value[3]);

    if (confirm_value != calibration_value) {
        DEBUG("\n[ec_oem debug] Setting calibration register to EC raw %d "
              "failed \n", calibration_value);
        i2c_release(I2C);
        return EC_OEM_WRITE_ERR;
    }

    i2c_release(I2C);

    return EC_OEM_OK;
}

int ec_oem_set_calibration(const ec_oem_t *dev, uint32_t calibration_value,
                           ec_oem_calibration_option_t option)
{
    assert(dev);

    if (_set_calibration_value(dev, calibration_value) != EC_OEM_OK) {
        return EC_OEM_WRITE_ERR;
    }

    uint8_t reg_value;

    i2c_acquire(I2C);

    if (i2c_write_reg(I2C, ADDR, EC_OEM_REG_CALIBRATION_REQUEST,
                      option, 0) < 0) {
        DEBUG("\n[ec_oem debug] Sending calibration request failed\n");
        return EC_OEM_WRITE_ERR;
    }

    do {
        if (i2c_read_reg(I2C, ADDR, EC_OEM_REG_CALIBRATION_REQUEST, &reg_value,
                         0) < 0) {
            DEBUG("\n[ec_oem debug] Reading calibration request status failed\n");
            i2c_release(I2C);
            return EC_OEM_READ_ERR;
        }
    } while (reg_value != 0x00);

    i2c_release(I2C);

    return EC_OEM_OK;
}

int ec_oem_read_calibration_state(const ec_oem_t *dev, uint16_t *calibration_state)
{
    assert(dev);
    i2c_acquire(I2C);

    if (i2c_read_reg(I2C, ADDR, EC_OEM_REG_CALIBRATION_CONFIRM,
                     calibration_state, 0) < 0) {
        DEBUG("\n[ec_oem debug] Failed at reading calibration confirm register\n");
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

    if (i2c_write_regs(I2C, ADDR, EC_OEM_REG_TEMP_COMPENSATION_BASE,
                       &reg_value, 4, 0) < 0) {
        DEBUG("\n[ec_oem debug] Setting temperature compensation of device to "
              "%d failed\n", temperature_compensation);
        i2c_release(I2C);
        return EC_OEM_WRITE_ERR;
    }
    i2c_release(I2C);

    return EC_OEM_OK;
}

int ec_oem_read_compensation(const ec_oem_t *dev,
                             uint16_t *temperature_compensation)
{
    uint8_t reg_value[4];

    assert(dev);
    i2c_acquire(I2C);

    if (i2c_read_regs(I2C, ADDR, EC_OEM_REG_TEMP_CONFIRMATION_BASE,
                      &reg_value, 4, 0) < 0) {
        DEBUG("[ec_oem debug] Getting temperature compensation value failed\n");
        i2c_release(I2C);
        return EC_OEM_READ_ERR;
    }
    *temperature_compensation = (int16_t)(reg_value[2] << 8) | (int16_t)(reg_value[3]);

    i2c_release(I2C);

    return EC_OEM_OK;
}

int ec_oem_read_ec(const ec_oem_t *dev, uint16_t *ec_value)
{
    uint8_t reg_value[4];

    assert(dev);
    i2c_acquire(I2C);

    if (i2c_read_regs(I2C, ADDR, EC_OEM_REG_EC_READING_BASE,
                      &reg_value, 4, 0) < 0) {
        DEBUG("[ec_oem debug] Getting EC value failed\n");
        i2c_release(I2C);
        return EC_OEM_READ_ERR;
    }
    *ec_value = (int16_t)(reg_value[2] << 8) | (int16_t)(reg_value[3]);

    i2c_release(I2C);

    return EC_OEM_OK;
}

int ec_oem_read_tds(const ec_oem_t *dev, uint16_t *tds_value)
{
    uint8_t reg_value[4];

    assert(dev);
    i2c_acquire(I2C);

    if (i2c_read_regs(I2C, ADDR, EC_OEM_REG_TDS_READING_BASE,
                      &reg_value, 4, 0) < 0) {
        DEBUG("[ec_oem debug] Getting TDS value failed\n");
        i2c_release(I2C);
        return EC_OEM_READ_ERR;
    }
    *tds_value = (int16_t)(reg_value[2] << 8) | (int16_t)(reg_value[3]);

    i2c_release(I2C);

    return EC_OEM_OK;
}

int ec_oem_read_pss(const ec_oem_t *dev, uint16_t *pss_value)
{
    uint8_t reg_value[4];

    assert(dev);
    i2c_acquire(I2C);

    if (i2c_read_regs(I2C, ADDR, EC_OEM_REG_PSS_READING_BASE,
                      &reg_value, 4, 0) < 0) {
        DEBUG("[ec_oem debug] Getting PSS value failed\n");
        i2c_release(I2C);
        return EC_OEM_READ_ERR;
    }
    *pss_value = (int16_t)(reg_value[2] << 8) | (int16_t)(reg_value[3]);

    i2c_release(I2C);

    return EC_OEM_OK;
}
