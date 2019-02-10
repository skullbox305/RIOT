/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_ph_oem PH OEM sensor device driver
 * @ingroup     drivers_sensors
 * @ingroup     drivers_saul
 * @brief       Device driver for Atlas Scientific pH OEM Sensor with SMBus/I2C interface
 *
 * The Atlas Scientific pH OEM sensor can be used with multiple configurations,
 * for example with interrupt and enable pin, only interrupt or enable pin,
 * or the default configuration with none of these pins. Per default both these
 * pins are mapped to @ref GPIO_UNDEF if not otherwise defined in your makefile.
 *
 * If you use an electrical isolation for most accurate readings
 * e.g. with the ADM3260, keep in mind that its not recommended to use the
 * interrupt pin without also isolating it somehow. The preferred method,
 * if not using an isolation on the interrupt line, would be polling. In this case
 * leave the interrupt pin undefined.
 *
 * The enable pin is a custom PCB feature and not included in the factory
 * default sensor.
 *
 * The Sensor has no integrated temperature sensor and for the highest possible
 * precision it requires another device to provide the temperature for the compensation.
 *
 * Once the pH OEM is powered on it will be ready to receive commands and take
 * readings after 1ms.
 *
 * Communication is done using SMBus/I2C protocol at speeds
 * of 10-100 kHz. Set your I2C speed to @ref I2C_SPEED_LOW or
 * @ref I2C_SPEED_NORMAL
 *
 * This driver provides @ref drivers_saul capabilities.
 * @{
 *
 * @file
 * @brief       Device driver for Atlas Scientific pH OEM Sensor with SMBus/I2C interface

 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 */

#ifndef PH_OEM_H
#define PH_OEM_H

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
    PH_OEM_OK                   = 0,    /**< Everything was fine */
    PH_OEM_NODEV                = -1,   /**< No device found on the bus */
    PH_OEM_READ_ERR             = -2,   /**< Reading to device failed*/
    PH_OEM_WRITE_ERR            = -3,   /**< Writing to device failed */
    PH_OEM_READ_WRITE_ERR       = -4,   /**< Reading or writing to device failed */
    PH_OEM_NOT_PH               = -5,   /**< Not an Atlas Scientific pH OEM device */
    PH_OEM_INTERRUPT_GPIO_UNDEF = -6,   /**< Interrupt pin is @ref GPIO_UNDEF */
    PH_OEM_ENABLE_GPIO_UNDEF    = -7,   /**< Enable pin is @ref GPIO_UNDEF */
    PH_OEM_GPIO_INIT_ERR        = -8    /**< Error while initializing GPIO PIN */
} ph_oem_named_returns_t;

/**
 * @brief   LED state values
 */
typedef enum {
    PH_OEM_LED_ON   = 0x01, /**< LED on state */
    PH_OEM_LED_OFF  = 0x00, /**< LED off state */
} ph_oem_led_state_t;

/**
 * @brief   Device state values
 */
typedef enum {
    PH_OEM_TAKE_READINGS    = 0x01, /**< Device active state */
    PH_OEM_STOP_READINGS    = 0x00, /**< Device hibernate state */
} ph_oem_device_state_t;
/**
 * @brief   Interrupt pin option values
 */
typedef enum {
    PH_OEM_IRQ_DISABLED = 0x00, /**< Interrupt pin option disabled */
    PH_OEM_IRQ_RISING   = 0x02, /**< Pin high on new reading (manually reset) */
    PH_OEM_IRQ_FALLING  = 0x04, /**< Pin low on new reading (manually reset) */
    PH_OEM_IRQ_BOTH     = 0x08, /**< Invert state on new reading (automatically reset) */
} ph_oem_irq_option_t;

/**
 * @brief   Calibration option values
 */
typedef enum {
    PH_OEM_CALIBRATE_LOW_POINT  = 0x02,     /**< Low point calibration option */
    PH_OEM_CALIBRATE_MID_POINT  = 0x03,     /**< Mid point calibration option */
    PH_OEM_CALIBRATE_HIGH_POINT = 0x04,     /**< High point calibration option */
} ph_oem_calibration_option_t;

/**
 * @brief   pH OEM sensor params
 */
typedef struct ph_oem_params {
    i2c_t i2c;              /**< I2C device the sensor is connected to */
    uint8_t addr;           /**< the slave address of the sensor on the I2C bus */
    gpio_t interrupt_pin;   /**< interrupt pin (@ref GPIO_UNDEF if not defined) */
    gpio_t enable_pin;      /**< enable pin (@ref GPIO_UNDEF if not defined) */
} ph_oem_params_t;

/**
 * @brief   pH OEM interrupt pin callback
 */
typedef void (*ph_oem_interrupt_pin_cb_t)(void *);

/**
 * @brief   pH OEM device descriptor
 */
typedef struct ph_oem {
    ph_oem_params_t params;         /**< device driver configuration */
    ph_oem_interrupt_pin_cb_t cb;   /**< interrupt pin callback */
    void *arg;                      /**< interrupt pin callback param */
} ph_oem_t;

/**
 * @brief   Initialize a pH OEM sensor
 *
 * @param[in,out]   dev      device descriptor
 * @param[in]       params   device configuration
 *
 * @return zero on successful initialization, non zero on error
 */
int ph_oem_init(ph_oem_t *dev, const ph_oem_params_t *params);

/**
 * @brief   Sets a new address to the pH OEM device by unlocking the
 *          @ref PH_OEM_REG_UNLOCK register and  writing a new address to
 *          the @ref PH_OEM_REG_ADDRESS register.
 *          The device address will also be updated in the device descriptor so
 *          it is still usable.
 *
 *          Settings are retained in the sensor if the power is cut.
 *
 *          The address in the device descriptor will reverse to the default
 *          address you provided thru PH_OEM_PARAM_ADDR after the
 *          microcontroller restarts
 *
 * @param[in] dev   device descriptor
 * @param[in] addr  new address for the device. Range: 0x01 - 0x7f
 *
 * @return zero on successful write, non zero on error
 */
int ph_oem_set_i2c_address(ph_oem_t *dev, uint8_t addr);

/**
 * @brief   Enable the interrupt gpio pin if @ref ph_oem_params_t.interrupt_pin
 *          is defined
 *
 * @param[in] dev       device descriptor
 * @param[in] cb        callback called when the pH OEM interrupt pin fires
 * @param[in] arg       callback argument
 * @param[in] option    @ref ph_oem_irq_option_t
 *
 * @return zero on successful write, non zero on error
 */
int ph_oem_enable_interrupt(ph_oem_t *dev, ph_oem_interrupt_pin_cb_t cb,
                            void *arg, ph_oem_irq_option_t option);

/**
 * @brief   Set the behavior of the interrupt pin of the pH OEM by setting the
 *          @ref PH_OEM_REG_INTERRUPT register.
 *          Pin is disabled by default.
 *
 *          The pin will not auto reset on option @ref PH_OEM_IRQ_RISING
 *          and @ref PH_OEM_IRQ_FALLING after interrupt fires,
 *          so call this function again to reset the pin state, providing the
 *          same option (@ref PH_OEM_IRQ_RISING or @ref PH_OEM_IRQ_FALLING).
 *
 *          Settings are not retained if the power is cut, so you have to call
 *          this function again after powering on the device.
 *
 * @param[in] dev    device descriptor
 * @param[in] option @ref ph_oem_irq_option_t
 *
 * @return zero on successful write, non zero on error
 */
int ph_oem_set_interrupt_pin(const ph_oem_t *dev, ph_oem_irq_option_t option);

/**
 * @brief   Set the LED state of the pH OEM sensor by writing to the
 *          @ref PH_OEM_REG_LED register
 *
 * @param[in] dev       device descriptor
 * @param[in] state     @ref ph_oem_led_state_t
 *
 * @return zero on successful write, non zero on error
 */
int ph_oem_set_led_state(const ph_oem_t *dev, ph_oem_led_state_t state);

/**
 * @brief   Enables/disables the device by putting a high or low signal on
 *          the GPIO @ref ph_oem_params_t.enable_pin
 *
 *          By default the enable pin is defined as @ref GPIO_UNDEF
 *          if not otherwise defined in the application.
 *
 *          The enable pin is a custom PCB feature and not integrated into the
 *          sensor by default. Your circuit needs an enable pin which controls
 *          some form of power switch. This function is used to turn off the
 *          sensor completely, because even in the state @ref PH_OEM_STOP_READINGS
 *          it still consumes about 3.0mA at 3.3V with the LED turned off.
 *
 *          Recommended process:
 *          1. Enable the device with
 *             @ref ph_oem_enable_device "ph_oem_enable_device(true)"
 *
 *          2. Setup the configurations you need (which are not saved after the
 *             device was turned off e.g. interrupt)
 *
 *          3. Sleep for around 2 minutes for the pH value to stabilize
 *             (if you need the most accurate result)
 *
 *          4. Call @ref ph_oem_set_device_state "ph_oem_set_device_state(PH_OEM_TAKE_READINGS)"
 *             and wait till the reading was taken (interrupt callback or polling
 *             if interrupt pin undefined)
 *
 *          5. Call @ref ph_oem_read_ph to get the pH value
 *
 *          6. Turn the device off again with calling
 *             @ref ph_oem_enable_device "ph_oem_enable_device(false)"
 *
 * @param[in] dev     device descriptor
 * @param[in] enable  True  = Device enabled <br>
 *                    False = Device disabled
 *
 * @return zero on successful write, non zero on error
 */
int ph_oem_enable_device(const ph_oem_t *dev, bool enable);

/**
 * @brief   Sets the device state (active/hibernate) of the pH OEM sensor by
 *          writing to the @ref PH_OEM_REG_HIBERNATE register.
 *          Once the device has been woken up it will continuously take readings
 *          every 420ms. Waking the device is the only way to take a reading.
 *          Hibernating the device is the only way to stop taking readings.
 *
 * @param[in] dev   device descriptor
 * @param[in] state @ref ph_oem_device_state_t
 *
 * @return zero on successful write, non zero on error
 */
int ph_oem_set_device_state(const ph_oem_t *dev, ph_oem_device_state_t state);

/**
 * @brief   Starts a new reading by setting the device state to
 *          @ref PH_OEM_TAKE_READINGS.
 *          If the @ref ph_oem_params_t.interrupt_pin is @ref GPIO_UNDEF
 *          this function will poll every 20ms till a reading is done (~420ms).
 *
 * @param[in] dev   device descriptor
 *
 * @return zero on successful write, non zero on error
 */
int ph_oem_start_new_reading(const ph_oem_t *dev);

/**
 * @brief   Clears all calibrations previously done
 *
 * @param[in] dev   device descriptor
 *
 * @return zero on successfully cleared, non zero on error
 */
int ph_oem_clear_calibration(const ph_oem_t *dev);

/**
 * @brief   Sets the @ref PH_OEM_REG_CALIBRATION_BASE register to the
 *          calibration_value which the pH OEM sensor will be
 *          calibrated to. Multiply the floating point calibration value of your
 *          solution by 1000 e.g. pH calibration solution => 7.002 * 1000 = 7002 = 0x00001B5A
 *
 *          The calibration value will be saved based on the given
 *          @ref ph_oem_calibration_option_t and retained after the power is cut.
 *
 *          Calibrating with @ref PH_OEM_CALIBRATE_MID_POINT will reset the
 *          previous calibrations.
 *          Always start with @ref PH_OEM_CALIBRATE_MID_POINT if you doing
 *          2 or 3 point calibration
 *
 * @param[in] dev                 device descriptor
 * @param[in] calibration_value   pH value multiplied by 1000 e.g 7.002 * 1000 = 7002
 * @param[in] option              @ref ph_oem_calibration_option_t
 *
 * @return zero if successfully calibrated, non zero on error
 */
int ph_oem_set_calibration(const ph_oem_t *dev, uint16_t calibration_value,
                           ph_oem_calibration_option_t option);

/**
 * @brief   Read the @ref PH_OEM_REG_CALIBRATION_CONFIRM register.
 *          After a calibration event has been successfully carried out, the
 *          calibration confirmation register will reflect what calibration has
 *          been done, by setting bits 0 - 2.
 *
 * @param[in]  dev                 device descriptor
 * @param[out] calibration_state   calibration state reflected by bits 0 - 2 <br>
 *                                 (0 = low, 1 = mid, 2 = high)
 *
 * @return zero on successful read, non zero on error
 */
int ph_oem_read_calibration_state(const ph_oem_t *dev, int16_t *calibration_state);

/**
 * @brief   Sets the @ref PH_OEM_REG_TEMP_COMPENSATION_BASE register to the
 *          temperature_compensation value which the pH OEM sensor will use
 *          to compensate the reading error.
 *          Multiply the floating point temperature value by 100
 *          e.g. temperature in degree Celsius = 34.26 * 100 = 3426
 *
 *          The temperature compensation will not be retained if the power is cut
 *
 * @param[in] dev                        device descriptor
 * @param[in] temperature_compensation   temperature value in Celsius multiplied <br>
 *                                       by 100 e.g 34.26 * 100 = 3426
 *
 * @return zero on successful write, non zero on error
 */
int ph_oem_set_compensation(const ph_oem_t *dev,
                            uint16_t temperature_compensation);

/**
 * @brief   Reads the @ref PH_OEM_REG_TEMP_CONFIRMATION_BASE register to verify
 *          the temperature compensation value that was used to take the pH
 *          reading is set to the correct temperature.
 *
 *          Divide the read temperature value by 100 to get the floating point
 *          e.g. temperature raw = 3426 / 100 = 34.26
 *
 * @param[in]  dev                       device descriptor
 * @param[out] temperature_compensation  raw temperature compensation value <br>
 *                                       Devide by 100 for floating point <br>
 *                                       e.g 3426 / 100 = 34.26
 *
 * @return zero if successfully read, non zero on error
 */
int ph_oem_read_compensation(const ph_oem_t *dev,
                             int16_t *temperature_compensation);

/**
 * @brief   Reads the @ref PH_OEM_REG_PH_READING_BASE register to get the current
 *          pH reading.
 *
 *          Divide the raw pH value by 1000 to get the floating point
 *          e.g. pH raw = 8347 / 1000 = 8.347
 *
 * @param[in]  dev        device descriptor
 * @param[out] ph_value   raw pH value <br>
 *                        divide by 1000 for floating point <br>
 *                        e.g 8347 / 1000 = 8.347
 *
 * @return zero if successfully read, non zero on error
 */
int ph_oem_read_ph(const ph_oem_t *dev, int16_t *ph_value);

#ifdef __cplusplus
}
#endif

#endif /* PH_OEM_H */
/** @} */
