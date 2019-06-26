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
    RTD_OEM_OK                      = 0,    /**< Everything was fine */
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

#ifdef __cplusplus
}
#endif

#endif /* RTD_OEM_H */
/** @} */
