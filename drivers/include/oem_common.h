/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_oem_common OEM common code
 * @ingroup     drivers_sensors
 * @brief       Common device driver code for the Atlas Scientific OEM sensor
 *              family (pH, EC, ORP, D.O. and RTD)
 * @{
 *
 * @file
 * @brief       Common device driver code for the Atlas Scientific OEM sensor
 *              family
 *
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 */

#ifndef OEM_COMMON_H
#define OEM_COMMON_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "periph/i2c.h"
#include "periph/gpio.h"

/**
 * @brief   Named return values
 */
typedef enum {
    OEM_COMMON_OK               =  0,       /**< Everything was fine */
    OEM_COMMON_NODEV            = -1,       /**< No device found on the bus */
    OEM_COMMON_READ_ERR         = -2,       /**< Reading to device failed*/
    OEM_COMMON_WRITE_ERR        = -3,       /**< Writing to device failed */
    OEM_COMMON_WRONG_DEV        = -4,       /**< Not an Atlas Scientific XXX OEM device */
    OEM_COMMON_INT_GPIO_UNDEF   = -5,       /**< Interrupt pin is @ref GPIO_UNDEF */
    OEM_COMMON_GPIO_INIT_ERR    = -6,       /**< Error while initializing GPIO PIN */
    OEM_COMMON_VAL_OUT_OF_RANGE = -7,       /**< Input value is out of range */
} oem_common_named_returns_t;

/**
 * @brief   LED state values
 */
typedef enum {
    OEM_COMMON_LED_ON   = 0x01, /**< LED on state */
    OEM_COMMON_LED_OFF  = 0x00, /**< LED off state */
} oem_common_led_state_t;

/**
 * @brief   Device state values
 */
typedef enum {
    OEM_COMMON_TAKE_READINGS    = 0x01, /**< Device active state */
    OEM_COMMON_STOP_READINGS    = 0x00, /**< Device hibernate state */
} oem_common_device_state_t;

/**
 * @brief   Interrupt pin option values
 */
typedef enum {
    OEM_COMMON_IRQ_RISING   = 0x02, /**< Pin high on new reading (manually reset) */
    OEM_COMMON_IRQ_FALLING  = 0x04, /**< Pin low on new reading (manually reset) */
    OEM_COMMON_IRQ_BOTH     = 0x08, /**< Invert state on new reading (automatically reset) */
} oem_common_irq_option_t;

/**
 * @brief   XXX_OEM sensor params
 */
typedef struct oem_common_params {
    i2c_t i2c;                          /**< I2C device the sensor is connected to */
    uint8_t addr;                       /**< the slave address of the sensor on the I2C bus */
    gpio_t interrupt_pin;               /**< interrupt pin (@ref GPIO_UNDEF if not defined) */
    gpio_mode_t gpio_mode;              /**< gpio mode of the interrupt pin */
    oem_common_irq_option_t irq_option; /**< behavior of the interrupt pin, disabled by default */
    uint8_t device_type_id;             /**< Device ID of the @ref PH_OEM_REG_DEVICE_TYPE
                                         *   register of a pH OEM sensor */
} oem_common_params_t;

/**
 * @brief   XXX_OEM interrupt pin callback
 */
typedef void (*oem_common_interrupt_pin_cb_t)(void *);

/**
 * @brief   XXX_OEM device descriptor
 */
typedef struct oem_common_dev {
    oem_common_params_t params;         /**< device driver configuration */
    oem_common_interrupt_pin_cb_t cb;   /**< interrupt pin callback */
} oem_common_dev_t;

/**
 * @brief   Initialize a XXX_OEM sensor
 *
 * @param[in,out]   dev      device descriptor
 * @param[in]       params   device configuration
 *
 * @return @ref OEM_COMMON_OK	     on success
 * @return @ref OEM_COMMON_NODEV     if no device is found on the bus
 * @return @ref OEM_COMMON_WRONG_DEV if the device found at the address is not a XXX_OEM device
 * @return
 */
int oem_common_init(oem_common_dev_t *dev, const oem_common_params_t *params);

/**
 * @brief   Write to a register (8 Bit) of a XXX_OEM sensor
 *
 * @param[in] dev            device descriptor
 * @param[in] reg            register address
 * @param[in] in             input value, which is written to reg
 *
 * @return OEM_COMMON_OK         on success
 * @return OEM_COMMON_WRITE_ERR  if writing to the device failed
 */
int oem_common_write_reg(const oem_common_dev_t *dev, uint8_t reg, uint8_t in);

/**
 * @brief   Read from a register (8 Bit) of a XXX_OEM sensor
 *
 * @param[in] dev     device descriptor
 * @param[in] reg     register address
 * @param[in] out     output value, which is read from reg
 *
 * @return OEM_COMMON_OK         on success
 * @return OEM_COMMON_WRITE_ERR  if writing to the device failed
 */
int oem_common_read_reg(const oem_common_dev_t *dev, uint8_t reg,
                        uint8_t *out);

/**
 * @brief   Write 4x8 Bit to the registers of a XXX_OEM sensor, starting from
 *          the register base address @p reg
 *
 * @param[in] dev    device descriptor
 * @param[in] reg    register base address
 * @param[in] in     input value, which is written to the registers
 *
 * @return OEM_COMMON_OK         on success
 * @return OEM_COMMON_WRITE_ERR  if writing to the device failed
 */
int oem_common_write_reg32(const oem_common_dev_t *dev, uint8_t reg,
                           uint32_t in);

/**
 * @brief   Read 4x8 Bit registers of a XXX_OEM sensor, starting from the register
 *          base address @p reg
 *
 * @param[in] dev    device descriptor
 * @param[in] reg    register base address
 * @param[in] out    output value, which is read from the 4x8 Bit registers
 *                   (must be uint32_t or int32_t)
 *
 * @return OEM_COMMON_OK         on success
 * @return OEM_COMMON_WRITE_ERR  if writing to the device failed
 */
int oem_common_read_reg32(const oem_common_dev_t *dev, uint8_t reg, bool neg,
                          void *out);

/**
 * @brief   Set the LED state of the XXX_OEM sensor by writing to the
 *          @ref OEM_COMMON_REG_LED register
 *
 * @param[in] dev       device descriptor
 * @param[in] state     @ref oem_common_led_state_t
 *
 * @return @ref OEM_COMMON_OK        on success
 * @return @ref OEM_COMMON_WRITE_ERR if writing to the device failed
 */
int oem_common_set_led_state(const oem_common_dev_t *dev,
                             oem_common_led_state_t state);

/**
 * @brief   Sets a new address to the XXX_OEM device by unlocking the
 *          @ref OEM_COMMON_REG_UNLOCK register and  writing a new address to
 *          the @ref OEM_COMMON_REG_ADDRESS register.
 *          The device address will also be updated in the device descriptor so
 *          it is still usable.
 *
 *          Settings are retained in the sensor if the power is cut.
 *
 *          The address in the device descriptor will reverse to the default
 *          address you provided through @ref OEM_COMMON_PARAM_ADDR after the
 *          microcontroller restarts
 *
 * @param[in] dev   device descriptor
 * @param[in] addr  new address for the device. Range: 0x01 - 0x7f
 *
 * @return @ref OEM_COMMON_OK		 on success
 * @return @ref OEM_COMMON_WRITE_ERR if writing to the device failed
 */
int oem_common_set_i2c_address(oem_common_dev_t *dev, uint8_t addr);

/**
 * @brief   Sets the calibration base register (@p reg) to the
 *          @p calibration_value which the device will be calibrated to.
 *
 * @param[in] dev               device descriptor
 * @param[in] calibration_value calibration value
 *
 * @return OEM_COMMON_OK         on success
 * @return OEM_COMMON_READ_ERR   if reading from the register failed
 * @return OEM_COMMON_WRITE_ERR  if writing the calibration_value to the device failed
 */
int oem_common_set_calibration_value(const oem_common_dev_t *dev, uint8_t reg,
                                     uint32_t calibration_value);

/**
 * @brief   Sets the calibration based on the given @p option. The calibration
 *          value must be set before (except for the D.O. sensor or when clearing
 *          the calibration).
 *
 * @param[in] dev          device descriptor
 * @param[in] reg          Calibration Request(R/W) or Calibration(write only) register
 * @param[in] option       Calibration option (e.g. 0x1 = clear, 0x2 = ...)
 * @param[in] write_only   Must be true, if @p reg is write-only (e.g. for D.O. sensor)
 *
 * @return OEM_COMMON_OK         on success
 * @return OEM_COMMON_READ_ERR   if reading from the register failed
 * @return OEM_COMMON_WRITE_ERR  if writing the calibration_value to the device failed
 */
int oem_common_set_calibration(const oem_common_dev_t *dev, uint8_t reg,
                               uint8_t option, bool write_only);

/**
 * @brief   The interrupt pin will not auto reset on option @ref OEM_COMMON_IRQ_RISING
 *          and @ref OEM_COMMON_IRQ_FALLING after interrupt fires,
 *          so call this function again to reset the pin state.
 *
 * @note    The interrupt settings are not retained if the power is cut,
 *          so you have to call this function again after powering on the device.
 *
 * @param[in] dev  device descriptor
 *
 * @return @ref OEM_COMMON_OK         on success
 * @return @ref OEM_COMMON_WRITE_ERR  if writing to the device failed
 */
int oem_common_reset_interrupt_pin(const oem_common_dev_t *dev);

/**
 * @brief   Enable the XXX_OEM interrupt pin if @ref oem_common_params_t.interrupt_pin
 *          is defined.
 *          @note @ref oem_common_reset_interrupt_pin needs to be called in the
 *          interrupt handler if you use @ref OEM_COMMON_IRQ_FALLING or
 *          @ref OEM_COMMON_IRQ_RISING
 *
 *          @note Provide the OEM_COMMON_PARAM_INTERRUPT_OPTION flag in your
 *          makefile. Valid options see: @ref oem_common_irq_option_t.
 *          The default is @ref OEM_COMMON_IRQ_BOTH.
 *
 *          @note Also provide the @ref gpio_mode_t as a CFLAG in your makefile.
 *          Most likely @ref GPIO_IN. If the pin is to sensitive use
 *          @ref GPIO_IN_PU for @ref OEM_COMMON_IRQ_FALLING or
 *          @ref GPIO_IN_PD for @ref OEM_COMMON_IRQ_RISING and
 *          @ref OEM_COMMON_IRQ_BOTH. The default is @ref GPIO_IN_PD
 *
 * @param[in] dev  device descriptor
 * @param[in] cb   callback called when the DO OEM interrupt pin fires
 * @param[in] arg  callback argument
 *
 * @return @ref OEM_COMMON_OK                   on success
 * @return @ref OEM_COMMON_WRITE_ERR            if writing to the device failed
 * @return @ref OEM_COMMON_INTERRUPT_GPIO_UNDEF if the interrupt pin is undefined
 * @return @ref OEM_COMMON_GPIO_INIT_ERR        if initializing the interrupt gpio pin failed
 */
int oem_common_enable_interrupt(oem_common_dev_t *dev,
                                oem_common_interrupt_pin_cb_t cb,
                                void *arg);

/**
 * @brief   Starts a new reading by setting the device state to
 *          @ref OEM_COMMON_TAKE_READINGS.
 *
 * @note    If the @ref oem_common_params_t.interrupt_pin is @ref GPIO_UNDEF
 *          this function will poll every 20ms till a reading is done (~420ms)
 *          and stop the device from taking further readings
 *
 * @param[in] dev   device descriptor
 *
 * @return @ref OEM_COMMON_OK		  on success
 * @return @ref OEM_COMMON_WRITE_ERR  if writing to the device failed
 * @return @ref OEM_COMMON_READ_ERR   if reading from the device failed
 */
int oem_common_start_new_reading(const oem_common_dev_t *dev);

/**
 * @brief   Sets the device state (active/hibernate) of the XXX_OEM sensor by
 *          writing to the @ref OEM_COMMON_REG_HIBERNATE register.
 *
 *          @note Once the device has been woken up it will continuously take
 *          readings every 420ms. Waking the device is the only way to take a
 *          reading. Hibernating the device is the only way to stop taking readings.
 *
 * @param[in] dev                   device descriptor
 * @param[in] state @ref            oem_common_device_state_t
 *
 * @return @ref OEM_COMMON_OK           on success
 * @return @ref OEM_COMMON_WRITE_ERR    if writing to the device failed
 */
int oem_common_set_device_state(const oem_common_dev_t *dev,
                                oem_common_device_state_t state);

#ifdef __cplusplus
}
#endif

#endif /* OEM_COMMON_H */
/** @} */
