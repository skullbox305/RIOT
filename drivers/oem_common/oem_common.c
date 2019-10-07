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

#define I2C (dev->params.i2c)
#define ADDR (dev->params.addr)
#define IRQ_OPTION (dev->params.irq_option)

/**
 * @brief   Write to a register (8 Bit) of a XXX_OEM sensor
 *
 * @param[in] dev            device descriptor
 * @param[in] reg            register address
 * @param[in] in             input value, which is written to reg
 *
 * @return OEM_COMMON_OK         on success
 * @return OEM_COMMON_WRITE_ERR if writing to the device failed
 */
static int _write_reg(const oem_common_dev_t *dev, uint8_t reg, uint8_t in);

/**
 * @brief   Read from a register (8 Bit) of a XXX_OEM sensor
 *
 * @param[in] dev            device descriptor
 * @param[in] reg            register address
 * @param[in] out            output value, which is read from reg
 *
 * @return OEM_COMMON_OK         on success
 * @return OEM_COMMON_WRITE_ERR if writing to the device failed
 */
static int _read_reg(const oem_common_dev_t *dev, uint8_t reg, uint16_t *out);

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
//
///**
// * @brief   Setting the XXX_OEM interrupt mode to the defined mode provided
// *          in the device descriptor
// *
// * @param[in] dev              device descriptor
// *
// * @return OEM_COMMON_OK       on success
// * @return OEM_COMMON_WRITE_ERR if writing to the device failed
// */
//static int _set_interrupt_pin(const oem_common_dev_t *dev);
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
///**
// * @brief   Sets the OEM_COMMON_REG_CALIBRATION_BASE register to the ORP
// *          @p calibration_value which the device will be calibrated to.
// *
// * @param[in] dev                 device descriptor
// * @param[in] calibration_value ORP value the device will be calibrated to
// *
// * @return OEM_COMMON_OK       on success
// * @return OEM_COMMON_READ_ERR     if reading from the register failed
// * @return OEM_COMMON_WRITE_ERR  if writing the calibration_value to the device failed
// */
//static int _set_calibration_value(const oem_common_dev_t *dev,
//                                  int16_t calibration_value);

int oem_common_init(oem_common_dev_t *dev, const oem_common_params_t *params)
{
    assert(dev && params);

    dev->params = *params;

    uint8_t reg_data;

    i2c_acquire(I2C);

    /* Register read test */
    if (i2c_read_regs(I2C, ADDR, OEM_COMMON_REG_DEVICE_TYPE,
                      &reg_data, 1, 0x0) < 0) {
        DEBUG("\n[oem_common debug] init - error: unable to read reg %x\n",
              OEM_COMMON_REG_DEVICE_TYPE);

        i2c_release(I2C);
        return OEM_COMMON_NODEV;
    }

    /* Test if the device ID of the attached XXX_OEM sensor equals the
     * value of the OEM_COMMON_REG_DEVICE_TYPE register
     * */
    if (reg_data != dev->params.device_type_id) {
        DEBUG("\n[oem_common debug] init - error: the attached device is not a ORP OEM "
              "Sensor. Read Device Type ID is: %i\n",
              reg_data);
        i2c_release(I2C);
        return OEM_COMMON_WRONG_DEV;
    }
    i2c_release(I2C);

    return OEM_COMMON_OK;
}

static int _write_reg(const oem_common_dev_t *dev, uint8_t reg, uint8_t in)
{
    i2c_acquire(I2C);

    if (i2c_write_reg(I2C, ADDR, reg, in, 0x0) < 0) {
        i2c_release(I2C);
        return OEM_COMMON_WRITE_ERR;
    }
    i2c_release(I2C);

    return OEM_COMMON_OK;
}

static int _read_reg(const oem_common_dev_t *dev, uint8_t reg, uint16_t *out)
{
    i2c_acquire(I2C);

    if (i2c_read_reg(I2C, ADDR, reg, out, 0) < 0) {
        i2c_release(I2C);
        return OEM_COMMON_READ_ERR;
    }
    i2c_release(I2C);
    return OEM_COMMON_OK;
}

int oem_common_set_led_state(const oem_common_dev_t *dev,
                             oem_common_led_state_t state)
{
    assert(dev);
    return _write_reg(dev, OEM_COMMON_REG_LED, state);
}

static int _unlock_address_reg(oem_common_dev_t *dev)
{
    uint8_t reg_value = 1;

    i2c_acquire(I2C);

    i2c_write_reg(I2C, ADDR, OEM_COMMON_REG_UNLOCK, 0x55, 0x0);
    i2c_write_reg(I2C, ADDR, OEM_COMMON_REG_UNLOCK, 0xAA, 0x0);
    /* if successfully unlocked the register will equal 0x00 */
    i2c_read_reg(I2C, ADDR, OEM_COMMON_REG_UNLOCK, &reg_value, 0x0);

    if (reg_value != 0x00) {
        DEBUG(
            "\n[oem_common debug] Failed at unlocking I2C address register. \n");
        i2c_release(I2C);
        return OEM_COMMON_WRITE_ERR;
    }
    i2c_release(I2C);

    return OEM_COMMON_OK;
}

int oem_common_set_i2c_address(oem_common_dev_t *dev, uint8_t addr)
{
    assert(dev);

    if (_unlock_address_reg(dev) != OEM_COMMON_OK) {
        return OEM_COMMON_WRITE_ERR;
    }

    i2c_acquire(I2C);

    if (i2c_write_reg(I2C, ADDR, OEM_COMMON_REG_ADDRESS, addr, 0x0) < 0) {
        DEBUG("\n[oem_common debug] Setting I2C address to %x failed\n", addr);
        i2c_release(I2C);
        return OEM_COMMON_WRITE_ERR;
    }

    dev->params.addr = addr;
    i2c_release(I2C);

    return OEM_COMMON_OK;
}

int oem_common_clear_calibration(const oem_common_dev_t *dev, uint8_t cali_reg,
                                 bool write_only)
{
    uint8_t reg_value;

    assert(dev);
    i2c_acquire(I2C);
    if (i2c_write_reg(I2C, ADDR, cali_reg, 0x1, 0) < 0) {
        DEBUG("\n[oem_common debug] Clearing calibration failed \n");
        i2c_release(I2C);
        return OEM_COMMON_WRITE_ERR;
    }

    if (write_only) {
        xtimer_usleep(50 * US_PER_MS);
    }
    else {
        do {
            if (i2c_read_reg(I2C, ADDR, cali_reg, &reg_value,
                             0) < 0) {
                i2c_release(I2C);
                return OEM_COMMON_READ_ERR;
            }
        } while (reg_value != 0x00);
    }

    i2c_release(I2C);

    return OEM_COMMON_OK;
}

int oem_common_read_calibration_state(const oem_common_dev_t *dev,
                                      uint8_t cal_confirm_reg,
                                      uint16_t *calibration_state)
{
    assert(dev);
    return _read_reg(dev, cal_confirm_reg, calibration_state);
}

//
//static int _set_interrupt_pin(const oem_common_dev_t *dev)
//{
//    assert(dev);
//    i2c_acquire(I2C);
//
//    if (i2c_write_reg(I2C, ADDR, OEM_COMMON_REG_INTERRUPT, IRQ_OPTION, 0x0) < 0) {
//        DEBUG("\n[oem_common debug] Setting interrupt pin to option %d failed.\n", IRQ_OPTION);
//        i2c_release(I2C);
//        return OEM_COMMON_WRITE_ERR;
//    }
//
//    i2c_release(I2C);
//
//    return OEM_COMMON_OK;
//}
//
//int oem_common_enable_interrupt(oem_common_dev_t *dev, oem_common_interrupt_pin_cb_t cb,
//                            void *arg)
//{
//    if (dev->params.interrupt_pin == GPIO_UNDEF) {
//        return OEM_COMMON_INTERRUPT_GPIO_UNDEF;
//    }
//
//    if (_set_interrupt_pin(dev) < 0) {
//        return OEM_COMMON_WRITE_ERR;
//    }
//
//    int gpio_flank = 0;
//
//    switch (IRQ_OPTION) {
//        case OEM_COMMON_IRQ_FALLING:
//            gpio_flank = GPIO_FALLING;
//            break;
//        case OEM_COMMON_IRQ_RISING:
//            gpio_flank = GPIO_RISING;
//            break;
//        case OEM_COMMON_IRQ_BOTH:
//            gpio_flank = GPIO_BOTH;
//            break;
//    }
//
//    dev->arg = arg;
//    dev->cb = cb;
//    if (gpio_init_int(dev->params.interrupt_pin,
//                      dev->params.gpio_mode, gpio_flank, cb, arg) < 0) {
//        DEBUG("\n[oem_common debug] Initializing interrupt gpio pin failed.\n");
//        return OEM_COMMON_GPIO_INIT_ERR;
//    }
//
//    return OEM_COMMON_OK;
//}
//
//int oem_common_reset_interrupt_pin(const oem_common_t *dev)
//{
//    /* no reset needed for mode OEM_COMMON_IRQ_BOTH */
//    if (dev->params.irq_option == OEM_COMMON_IRQ_BOTH) {
//        return OEM_COMMON_OK;
//    }
//
//    if (_set_interrupt_pin(dev) < 0) {
//        return OEM_COMMON_WRITE_ERR;
//    }
//    return OEM_COMMON_OK;
//}

//int oem_common_set_device_state(const oem_common_t *dev, oem_common_device_state_t state)
//{
//    assert(dev);
//    i2c_acquire(I2C);
//
//    if (i2c_write_reg(I2C, ADDR, OEM_COMMON_REG_HIBERNATE, state, 0x0) < 0) {
//        DEBUG("\n[oem_common debug] Setting device state to %d failed\n", state);
//        i2c_release(I2C);
//        return OEM_COMMON_WRITE_ERR;
//    }
//    i2c_release(I2C);
//
//    return OEM_COMMON_OK;
//}
//
//static int _new_reading_available(const oem_common_t *dev)
//{
//    int8_t new_reading_available;
//
//    assert(dev);
//    i2c_acquire(I2C);
//    do {
//        if (i2c_read_reg(I2C, ADDR, OEM_COMMON_REG_NEW_READING,
//                         &new_reading_available, 0x0) < 0) {
//            DEBUG("\n[oem_common debug] Failed at reading OEM_COMMON_REG_NEW_READING\n");
//            i2c_release(I2C);
//            return OEM_COMMON_READ_ERR;
//        }
//        xtimer_usleep(20 * US_PER_MS);
//    } while (new_reading_available == 0);
//
//    /* need to manually reset register back to 0x00 */
//    if (i2c_write_reg(I2C, ADDR, OEM_COMMON_REG_NEW_READING, 0x00, 0x0) < 0) {
//        DEBUG("\n[oem_common debug] Resetting OEM_COMMON_REG_NEW_READING failed\n");
//        i2c_release(I2C);
//        return OEM_COMMON_WRITE_ERR;
//    }
//    i2c_release(I2C);
//
//    return OEM_COMMON_OK;
//}
//
//int oem_common_start_new_reading(const oem_common_t *dev)
//{
//    if (oem_common_set_device_state(dev, OEM_COMMON_TAKE_READINGS) < 0) {
//        return OEM_COMMON_WRITE_ERR;
//    }
//
//    /* if interrupt pin is undefined, poll till new reading was taken and stop
//     * device form taking further readings */
//    if (dev->params.interrupt_pin == GPIO_UNDEF) {
//        int result = _new_reading_available(dev);
//        if (result < 0) {
//            return result;
//        }
//
//        if (oem_common_set_device_state(dev, OEM_COMMON_STOP_READINGS) < 0) {
//            return OEM_COMMON_WRITE_ERR;
//        }
//    }
//    return OEM_COMMON_OK;
//}
//


//static int _set_calibration_value(const oem_common_t *dev,
//                                  int16_t calibration_value)
//{
//    int8_t reg_value[4];
//
//    reg_value[0] = 0x00;
//    reg_value[1] = 0x00;
//    reg_value[2] = (int8_t)(calibration_value >> 8);
//    reg_value[3] = (int8_t)(calibration_value & 0x00FF);
//
//    i2c_acquire(I2C);
//
//    if (i2c_write_regs(I2C, ADDR, OEM_COMMON_REG_CALIBRATION_BASE, &reg_value, 4, 0) < 0) {
//        DEBUG("\n[oem_common debug] Writing calibration value failed \n");
//        i2c_release(I2C);
//        return OEM_COMMON_WRITE_ERR;
//    }
//
//    /* Calibration is critical, so check if written value is in fact correct */
//    if (i2c_read_regs(I2C, ADDR, OEM_COMMON_REG_CALIBRATION_BASE, &reg_value, 4, 0) < 0) {
//        DEBUG("\n[oem_common debug] Reading the calibration value failed \n");
//        i2c_release(I2C);
//        return OEM_COMMON_READ_ERR;
//    }
//
//    uint16_t confirm_value = (int16_t)(reg_value[2] << 8)
//                             | (int16_t)(reg_value[3]);
//
//    if (confirm_value != calibration_value) {
//        DEBUG("\n[oem_common debug] Setting calibration register to ORP raw %d "
//              "failed \n", calibration_value);
//        i2c_release(I2C);
//        return OEM_COMMON_WRITE_ERR;
//    }
//
//    i2c_release(I2C);
//
//    return OEM_COMMON_OK;
//}
//
//int oem_common_set_calibration(const oem_common_t *dev, int16_t calibration_value,
//                           oem_common_calibration_option_t option)
//{
//    assert(dev);
//
//    if (_set_calibration_value(dev, calibration_value) != OEM_COMMON_OK) {
//        return OEM_COMMON_WRITE_ERR;
//    }
//
//    uint8_t reg_value;
//
//    i2c_acquire(I2C);
//
//    if (i2c_write_reg(I2C, ADDR, OEM_COMMON_REG_CALIBRATION_REQUEST,
//                      option, 0) < 0) {
//        DEBUG("\n[oem_common debug] Sending calibration request failed\n");
//        return OEM_COMMON_WRITE_ERR;
//    }
//
//    do {
//        if (i2c_read_reg(I2C, ADDR, OEM_COMMON_REG_CALIBRATION_REQUEST, &reg_value,
//                         0) < 0) {
//            DEBUG("\n[oem_common debug] Reading calibration request status failed\n");
//            i2c_release(I2C);
//            return OEM_COMMON_READ_ERR;
//        }
//    } while (reg_value != 0x00);
//
//    i2c_release(I2C);
//
//    return OEM_COMMON_OK;
//}
//
