/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_rtd_oem RTD OEM sensor device driver
 * @ingroup     drivers_sensors
 * @ingroup     drivers_saul
 * @brief       Device driver for Atlas Scientific RTD OEM sensor with SMBus/I2C interface
 *
 * @{
 *
 * @file
 * @brief       Device driver for Atlas Scientific RTD OEM Sensor with SMBus/I2C interface

 * @author      Ting XU <your-email@placeholder.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 */

#ifndef RTD_OEM_H
#define RTD_OEM_H

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
    RTD_OEM_OK                      =  0,    /**< Everything was fine */
    RTD_OEM_NODEV                   = -1,   /**< No device found on the bus */
    RTD_OEM_READ_ERR                = -2,   /**< Reading to device failed*/
    RTD_OEM_WRITE_ERR               = -3,   /**< Writing to device failed */
    RTD_OEM_NOT_RTD                 = -4,   /**< Not an Atlas Scientific RTD OEM device */
    RTD_OEM_INTERRUPT_GPIO_UNDEF    = -5,   /**< Interrupt pin is @ref GPIO_UNDEF */
    RTD_OEM_GPIO_INIT_ERR           = -6,   /**< Error while initializing GPIO PIN */
} rtd_oem_named_returns_t;

/**
 * @brief   LED state values
 */
typedef enum {
    RTD_OEM_LED_ON  = 0x01, /**< LED on state */
    RTD_OEM_LED_OFF = 0x00, /**< LED off state */
} rtd_oem_led_state_t;

/**
 * @brief   Device state values
 */
typedef enum {
    RTD_OEM_TAKE_READINGS   = 0x01, /**< Device active state */
    RTD_OEM_STOP_READINGS   = 0x00, /**< Device hibernate state */
} rtd_oem_device_state_t;

/**
 * @brief   Calibration option values
 */
typedef enum {
    RTD_OEM_CALIBRATE_CLEAR         = 0x01, /**< Clear calibration */
    RTD_OEM_CALIBRATE_SINGLE_POINT  = 0x02, /**< Single point calibration */
} rtd_oem_calibration_option_t;

/**
 * @brief   Interrupt pin option values
 */
typedef enum {
    RTD_OEM_IRQ_RISING  = 0x02, /**< Pin high on new reading (manually reset) */
    RTD_OEM_IRQ_FALLING = 0x04, /**< Pin low on new reading (manually reset) */
    RTD_OEM_IRQ_BOTH    = 0x08, /**< Invert state on new reading (automatically reset) */
} rtd_oem_irq_option_t;

/**
 * @brief   RTD OEM sensor params
 */
typedef struct rtd_oem_params {
    i2c_t i2c;                          /**< I2C device the sensor is connected to */
    uint8_t addr;                       /**< the slave address of the sensor on the I2C bus */
    gpio_t interrupt_pin;               /**< interrupt pin (@ref GPIO_UNDEF if not defined) */
    gpio_mode_t gpio_mode;              /**< gpio mode of the interrupt pin */
    rtd_oem_irq_option_t irq_option;    /**< behavior of the interrupt pin, disabled by default */
} rtd_oem_params_t;

/**
 * @brief   RTD OEM interrupt pin callback
 */
typedef void (*rtd_oem_interrupt_pin_cb_t)(void *);

/**
 * @brief   RTD OEM device descriptor
 */
typedef struct rtd_oem {
    rtd_oem_params_t params;        /**< device driver configuration */
    rtd_oem_interrupt_pin_cb_t cb;  /**< interrupt pin callback */
    void *arg;                      /**< interrupt pin callback param */
} rtd_oem_t;

/**
 * @brief   Initialize a RTD OEM sensor
 *
 * @param[in,out]   dev      device descriptor
 * @param[in]       params   device configuration
 *
 * @return @ref RTD_OEM_OK on success
 * @return @ref RTD_OEM_NODEV if no device is found on the bus
 * @return @ref RTD_OEM_NOT_RTD if the device found at the address is not a RTD OEM device
 * @return
 */
int rtd_oem_init(rtd_oem_t *dev, const rtd_oem_params_t *params);

/**
 * @brief   Set the LED state of the RTD OEM sensor by writing to the
 *          @ref RTD_OEM_REG_LED register
 *
 * @param[in] dev       device descriptor
 * @param[in] state     @ref ph_oem_led_state_t
 *
 * @return @ref RTD_OEM_OK on success
 * @return @ref RTD_OEM_WRITE_ERR if writing to the device failed
 */
int rtd_oem_set_led_state(const rtd_oem_t *dev, rtd_oem_led_state_t state);

/**
 * @brief   Sets a new address to the RTD OEM device by unlocking the
 *          @ref RTD_OEM_REG_UNLOCK register and  writing a new address to
 *          the @ref RTD_OEM_REG_ADDRESS register.
 *          The device address will also be updated in the device descriptor so
 *          it is still usable.
 *
 *          Settings are retained in the sensor if the power is cut.
 *
 *          The address in the device descriptor will reverse to the default
 *          address you provided through RTD_OEM_PARAM_ADDR after the
 *          microcontroller restarts
 *
 * @param[in] dev   device descriptor
 * @param[in] addr  new address for the device. Range: 0x01 - 0x7f
 *
 * @return @ref RTD_OEM_OK on success
 * @return @ref RTD_OEM_WRITE_ERR if writing to the device failed
 */
int rtd_oem_set_i2c_address(rtd_oem_t *dev, uint8_t addr);

/**
 * @brief   Starts a new reading by setting the device state to
 *          @ref RTD_OEM_TAKE_READINGS.
 *
 * @note    If the @ref rtd_oem_params_t.interrupt_pin is @ref GPIO_UNDEF
 *          this function will poll every 20ms till a reading is done (~420ms)
 *          and stop the device from taking further readings
 *
 * @param[in] dev   device descriptor
 *
 * @return @ref RTD_OEM_OK on success
 * @return @ref RTD_OEM_WRITE_ERR if writing to the device failed
 * @return @ref RTD_OEM_READ_ERR if reading from the device failed
 */
int rtd_oem_start_new_reading(const rtd_oem_t *dev);

/**
 * @brief   Clears all calibrations previously done
 *
 * @param[in] dev   device descriptor
 *
 * @return @ref RTD_OEM_OK on success
 * @return @ref RTD_OEM_WRITE_ERR if writing to the device failed
 * @return @ref RTD_OEM_READ_ERR if reading from the device failed
 */
int rtd_oem_clear_calibration(const rtd_oem_t *dev);

/**
 * @brief   Read the @ref RTD_OEM_REG_CALIBRATION_CONFIRM register.
 *          After a calibration event has been successfully carried out, the
 *          calibration confirmation register will reflect what calibration has
 *          been done, by setting bits 0 - 2.
 *
 * @param[in]  dev                 device descriptor
 * @param[out] calibration_state   calibration state reflected by bits 0 - 2 <br>
 *                                 (0 = low, 1 = mid, 2 = high)
 *
 * @return @ref RTD_OEM_OK on success
 * @return @ref RTD_OEM_READ_ERR if reading from the device failed
 */
int rtd_oem_read_calibration_state(const rtd_oem_t *dev, uint16_t *calibration_state);

/**
 * @brief   Sets the @ref RTD_OEM_REG_CALIBRATION_BASE register to the
 *          calibration_value which the rtd OEM sensor will be
 *          calibrated to. Multiply the floating point calibration value of your
 *          solution by 1000 e.g. rtd calibration solution => 50,5 * 1000 = 50500 = 0x0000C544
 *          The calibration value will be saved based on the given
 *          @ref rtd_oem_calibration_option_t and retained after the power is cut.
 *
 * @param[in] dev                 device descriptor
 * @param[in] calibration_value   rtd value multiplied by 1000 e.g 50,5 * 1000 = 50500
 * @param[in] option              @ref rtd_oem_calibration_option_t
 *
 * @return @ref RTD_OEM_OK on success
 * @return @ref RTD_OEM_WRITE_ERR if writing to the device failed
 * @return @ref RTD_OEM_READ_ERR if reading from the device failed
 */
int rtd_oem_set_calibration(const rtd_oem_t *dev, uint32_t calibration_value,
                           rtd_oem_calibration_option_t option);

/**
 * @brief   The interrupt pin will not auto reset on option @ref RTD_OEM_IRQ_RISING
 *          and @ref RTD_OEM_IRQ_FALLING after interrupt fires,
 *          so call this function again to reset the pin state.
 *
 * @note    The interrupt settings are not retained if the power is cut,
 *          so you have to call this function again after powering on the device.
 *
 * @param[in] dev    device descriptor
 *
 * @return @ref RTD_OEM_OK on success
 * @return @ref RTD_OEM_WRITE_ERR if writing to the device failed
 */
int rtd_oem_reset_interrupt_pin(const rtd_oem_t *dev);

/**
 * @brief   Enable the RTD OEM interrupt pin if @ref RTD_oem_params_t.interrupt_pin
 *          is defined.
 *          @note @ref RTD_oem_reset_interrupt_pin needs to be called in the
 *          callback if you use @ref RTD_OEM_IRQ_FALLING or @ref RTD_OEM_IRQ_RISING
 *
 *          @note Provide the RTD_OEM_PARAM_INTERRUPT_OPTION flag in your
 *          makefile. Valid options see: @ref rtd_oem_irq_option_t.
 *          The default is @ref RTD_OEM_IRQ_BOTH.
 *
 *          @note Also provide the @ref gpio_mode_t as a CFLAG in your makefile.
 *          Most likely @ref GPIO_IN. If the pin is to sensitive use
 *          @ref GPIO_IN_PU for @ref RTD_OEM_IRQ_FALLING or
 *          @ref GPIO_IN_PD for @ref RTD_OEM_IRQ_RISING and
 *          @ref RTD_OEM_IRQ_BOTH. The default is @ref GPIO_IN_PD
 *
 *
 * @param[in] dev       device descriptor
 * @param[in] cb        callback called when the RTD OEM interrupt pin fires
 * @param[in] arg       callback argument
 *
 * @return @ref RTD_OEM_OK on success
 * @return @ref RTD_OEM_WRITE_ERR if writing to the device failed
 * @return @ref RTD_OEM_INTERRUPT_GPIO_UNDEF if the interrupt pin is undefined
 * @return @ref RTD_OEM_GPIO_INIT_ERR if initializing the interrupt gpio pin failed
 */
int rtd_oem_enable_interrupt(rtd_oem_t *dev, rtd_oem_interrupt_pin_cb_t cb,
                            void *arg);

/**
 * @brief   Sets the device state (active/hibernate) of the RTD OEM sensor by
 *          writing to the @ref RTD_OEM_REG_HIBERNATE register.
 *
 *          @note Once the device has been woken up it will continuously take
 *          readings every 420ms. Waking the device is the only way to take a
 *          reading. Hibernating the device is the only way to stop taking readings.
 *
 * @param[in] dev   device descriptor
 * @param[in] state @ref rtd_oem_device_state_t
 *
 * @return @ref RTD_OEM_OK on success
 * @return @ref RTD_OEM_WRITE_ERR if writing to the device failed
 */
int rtd_oem_set_device_state(const rtd_oem_t *dev, rtd_oem_device_state_t state);

/**
 * @brief   Reads the @ref RTD_OEM_REG_RTD_READING_BASE register to get the current
 *          rtd reading.
 *
 * @param[in]  dev        device descriptor
 * @param[out] rtd_value   raw rtd value <br>
 *                        divide by 1000 for floating point <br>
 *                        e.g 25761 / 1000 = 25.761
 *
 * @return @ref RTD_OEM_OK on success
 * @return @ref RTD_OEM_READ_ERR if reading from the device failed
 */
int rtd_oem_read_temp(const rtd_oem_t *dev, uint32_t *rtd_value);

#ifdef __cplusplus
}
#endif

#endif /* RTD_OEM_H */
/** @} */
