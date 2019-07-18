/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_do_oem DO OEM sensor device driver
 * @ingroup     drivers_sensors
 * @ingroup     drivers_saul
 * @brief       Device driver for Atlas Scientific DO OEM sensor with SMBus/I2C interface

 * The Atlas Scientific DO OEM sensor can be used with or without the interrupt
 * pin. Per default this pin is mapped to @ref GPIO_UNDEF if not otherwise defined
 * in your makefile.
 *
 * Once the DO OEM is powered on it will be ready to receive commands and take
 * readings after 1ms.
 *
 * @note This driver provides @ref drivers_saul capabilities.
 * Reading (@ref saul_driver_t.read) from the device returns the current D.O.
 * value.
 *
 * @note Communication is done using SMBus/I2C protocol at speeds
 * of 10-100 kHz. Set your board I2C speed to @ref I2C_SPEED_LOW or
 * @ref I2C_SPEED_NORMAL
 *
 * @{
 *
 * @file
 * @brief       Device driver for Atlas Scientific DO OEM Sensor with SMBus/I2C interface

 * @author      Ting XU <timtsui@outlook.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 */

#ifndef DO_OEM_H
#define DO_OEM_H

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
    DO_OEM_OK                      =  0,   /**< Everything was fine */
    DO_OEM_NODEV                   = -1,   /**< No device found on the bus */
    DO_OEM_READ_ERR                = -2,   /**< Reading to device failed*/
    DO_OEM_WRITE_ERR               = -3,   /**< Writing to device failed */
    DO_OEM_NOT_DO                  = -4,   /**< Not an Atlas Scientific DO OEM device */
    DO_OEM_INTERRUPT_GPIO_UNDEF    = -5,   /**< Interrupt pin is @ref GPIO_UNDEF */
    DO_OEM_GPIO_INIT_ERR           = -6,   /**< Error while initializing GPIO PIN */
} do_oem_named_returns_t;

/**
 * @brief   LED state values
 */
typedef enum {
    DO_OEM_LED_ON  = 0x01, /**< LED on state */
    DO_OEM_LED_OFF = 0x00, /**< LED off state */
} do_oem_led_state_t;

/**
 * @brief   Device state values
 */
typedef enum {
    DO_OEM_TAKE_READINGS   = 0x01, /**< Device active state */
    DO_OEM_STOP_READINGS   = 0x00, /**< Device hibernate state */
} do_oem_device_state_t;

/**
 * @brief   Calibration option values
 */
typedef enum {
    DO_OEM_CALIBRATE_CLEAR         = 0x01, /**< Clear calibration */
	DO_OEM_CALIBRATE_ATMOSPHERIC   = 0x02, /**< Calibrate to atmospheric oxygen content */
    DO_OEM_CALIBRATE_0_DISSOLVED   = 0x03, /**< Calibrate to 0 dissolved oxygen */
} do_oem_calibration_option_t;

/**
 * @brief   Interrupt pin option values
 */
typedef enum {
    DO_OEM_IRQ_RISING  = 0x02, /**< Pin high on new reading (manually reset) */
    DO_OEM_IRQ_FALLING = 0x04, /**< Pin low on new reading (manually reset) */
    DO_OEM_IRQ_BOTH    = 0x08, /**< Invert state on new reading (automatically reset) */
} do_oem_irq_option_t;

/**
 * @brief   DO OEM sensor params
 */
typedef struct do_oem_params {
    i2c_t i2c;                          /**< I2C device the sensor is connected to */
    uint8_t addr;                       /**< the slave address of the sensor on the I2C bus */
    gpio_t interrupt_pin;               /**< interrupt pin (@ref GPIO_UNDEF if not defined) */
    gpio_mode_t gpio_mode;              /**< gpio mode of the interrupt pin */
    do_oem_irq_option_t irq_option;    /**< behavior of the interrupt pin, disabled by default */
} do_oem_params_t;

/**
 * @brief   DO OEM interrupt pin callback
 */
typedef void (*do_oem_interrupt_pin_cb_t)(void *);

/**
 * @brief   DO OEM device descriptor
 */
typedef struct do_oem {
    do_oem_params_t params;        /**< device driver configuration */
    do_oem_interrupt_pin_cb_t cb;  /**< interrupt pin callback */
    void *arg;                      /**< interrupt pin callback param */
} do_oem_t;

/**
 * @brief   Initialize a DO OEM sensor
 *
 * @param[in,out]   dev      device descriptor
 * @param[in]       params   device configuration
 *
 * @return @ref DO_OEM_OK on success
 * @return @ref DO_OEM_NODEV if no device is found on the bus
 * @return @ref DO_OEM_NOT_DO if the device found at the address is not a DO OEM device
 * @return
 */
int do_oem_init(do_oem_t *dev, const do_oem_params_t *params);

/**
 * @brief   Set the LED state of the DO OEM sensor by writing to the
 *          @ref DO_OEM_REG_LED register
 *
 * @param[in] dev       device descriptor
 * @param[in] state     @ref do_oem_led_state_t
 *
 * @return @ref DO_OEM_OK on success
 * @return @ref DO_OEM_WRITE_ERR if writing to the device failed
 */
int do_oem_set_led_state(const do_oem_t *dev, do_oem_led_state_t state);

/**
 * @brief   Sets a new address to the DO OEM device by unlocking the
 *          @ref DO_OEM_REG_UNLOCK register and  writing a new address to
 *          the @ref DO_OEM_REG_ADDRESS register.
 *          The device address will also be updated in the device descriptor so
 *          it is still usable.
 *
 *          Settings are retained in the sensor if the power is cut.
 *
 *          The address in the device descriptor will reverse to the default
 *          address you provided through DO_OEM_PARAM_ADDR after the
 *          microcontroller restarts
 *
 * @param[in] dev   device descriptor
 * @param[in] addr  new address for the device. Range: 0x01 - 0x7f
 *
 * @return @ref DO_OEM_OK on success
 * @return @ref DO_OEM_WRITE_ERR if writing to the device failed
 */
int do_oem_set_i2c_address(do_oem_t *dev, uint8_t addr);

/**
 * @brief   Starts a new reading by setting the device state to
 *          @ref DO_OEM_TAKE_READINGS.
 *
 * @note    If the @ref do_oem_params_t.interrupt_pin is @ref GPIO_UNDEF
 *          this function will poll every 20ms till a reading is done (~420ms)
 *          and stop the device from taking further readings
 *
 * @param[in] dev   device descriptor
 *
 * @return @ref DO_OEM_OK on success
 * @return @ref DO_OEM_WRITE_ERR if writing to the device failed
 * @return @ref DO_OEM_READ_ERR if reading from the device failed
 */
int do_oem_start_new_reading(const do_oem_t *dev);

/**
 * @brief   Clears all calibrations previously done
 *
 * @param[in] dev   device descriptor
 *
 * @return @ref DO_OEM_OK on success
 * @return @ref DO_OEM_WRITE_ERR if writing to the device failed
 * @return @ref DO_OEM_READ_ERR if reading from the device failed
 */
int do_oem_clear_calibration(const do_oem_t *dev);

/**
 * @brief   Read the @ref DO_OEM_REG_CALIBRATION_CONFIRM register.
 *          After a calibration event has been successfully carried out, the
 *          calibration confirmation register will reflect what calibration has
 *          been done, by setting bits 0 - 1.
 *
 * @param[in]  dev                 device descriptor
 * @param[out] calibration_state   calibration state reflected by bits 0 - 1 <br>
 *                                 (0 = no calibration, 1 = calibration)
 *
 * @return @ref DO_OEM_OK on success
 * @return @ref DO_OEM_READ_ERR if reading from the device failed
 */
int do_oem_read_calibration_state(const do_oem_t *dev,
                                   uint16_t *calibration_state);

/**
 * @brief   Sets the @ref DO_OEM_REG_CALIBRATION_BASE register to the
 *          calibration_value which the do OEM sensor will be
 *          calibrated to. Multiply the floating point calibration value of your
 *          solution by 1000 e.g. do calibration solution => 50,5 * 1000 = 50500 = 0x0000C544
 *          The calibration value will be saved based on the given
 *          @ref do_oem_calibration_option_t and retained after the power is cut.
 *
 * @param[in] dev                 device descriptor
 * @param[in] calibration_value   do value multiplied by 1000 e.g 50,5 * 1000 = 50500
 * @param[in] option              @ref do_oem_calibration_option_t
 *
 * @return @ref DO_OEM_OK on success
 * @return @ref DO_OEM_WRITE_ERR if writing to the device failed
 * @return @ref DO_OEM_READ_ERR if reading from the device failed
 */
int do_oem_set_calibration(const do_oem_t *dev, uint32_t calibration_value,
                            do_oem_calibration_option_t option);

/**
 * @brief   The interrupt pin will not auto reset on option @ref DO_OEM_IRQ_RISING
 *          and @ref DO_OEM_IRQ_FALLING after interrupt fires,
 *          so call this function again to reset the pin state.
 *
 * @note    The interrupt settings are not retained if the power is cut,
 *          so you have to call this function again after powering on the device.
 *
 * @param[in] dev    device descriptor
 *
 * @return @ref DO_OEM_OK on success
 * @return @ref DO_OEM_WRITE_ERR if writing to the device failed
 */
int do_oem_reset_interrupt_pin(const do_oem_t *dev);

/**
 * @brief   Enable the DO OEM interrupt pin if @ref do_oem_params_t.interrupt_pin
 *          is defined.
 *          @note @ref do_oem_reset_interrupt_pin needs to be called in the
 *          callback if you use @ref DO_OEM_IRQ_FALLING or @ref DO_OEM_IRQ_RISING
 *
 *          @note Provide the DO_OEM_PARAM_INTERRUPT_OPTION flag in your
 *          makefile. Valid options see: @ref do_oem_irq_option_t.
 *          The default is @ref DO_OEM_IRQ_BOTH.
 *
 *          @note Also provide the @ref gpio_mode_t as a CFLAG in your makefile.
 *          Most likely @ref GPIO_IN. If the pin is to sensitive use
 *          @ref GPIO_IN_PU for @ref DO_OEM_IRQ_FALLING or
 *          @ref GPIO_IN_PD for @ref DO_OEM_IRQ_RISING and
 *          @ref DO_OEM_IRQ_BOTH. The default is @ref GPIO_IN_PD
 *
 *
 * @param[in] dev       device descriptor
 * @param[in] cb        callback called when the DO OEM interrupt pin fires
 * @param[in] arg       callback argument
 *
 * @return @ref DO_OEM_OK on success
 * @return @ref DO_OEM_WRITE_ERR if writing to the device failed
 * @return @ref DO_OEM_INTERRUPT_GPIO_UNDEF if the interrupt pin is undefined
 * @return @ref DO_OEM_GPIO_INIT_ERR if initializing the interrupt gpio pin failed
 */
int do_oem_enable_interrupt(do_oem_t *dev, do_oem_interrupt_pin_cb_t cb,
                             void *arg);

/**
 * @brief   Sets the device state (active/hibernate) of the DO OEM sensor by
 *          writing to the @ref DO_OEM_REG_HIBERNATE register.
 *
 *          @note Once the device has been woken up it will continuously take
 *          readings every 420ms. Waking the device is the only way to take a
 *          reading. Hibernating the device is the only way to stop taking readings.
 *
 * @param[in] dev   device descriptor
 * @param[in] state @ref do_oem_device_state_t
 *
 * @return @ref DO_OEM_OK on success
 * @return @ref DO_OEM_WRITE_ERR if writing to the device failed
 */
int do_oem_set_device_state(const do_oem_t *dev,
                             do_oem_device_state_t state);

/**
 * @brief   Reads the @ref DO_OEM_REG_DO_READING_BASE register to get the current
 *          do reading.
 *
 * @param[in]  dev        device descriptor
 * @param[out] do_value   raw do value <br>
 *                        divide by 1000 for floating point <br>
 *                        e.g 25761 / 1000 = 25.761
 *
 * @return @ref DO_OEM_OK on success
 * @return @ref DO_OEM_READ_ERR if reading from the device failed
 */
int do_oem_read_temp(const do_oem_t *dev, int32_t *do_value);

#ifdef __cplusplus
}
#endif

#endif /* DO_OEM_H */
/** @} */
