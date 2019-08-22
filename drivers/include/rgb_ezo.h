/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_rgb_ezo RGB EZO sensor device driver
 * @ingroup     drivers_sensors
 * @ingroup     drivers_saul
 * @brief       Device driver for Atlas Scientific RGB EZO sensor with SMBus/I2C interface
 *
 * The Atlas Scientific RGB EZO sensor can be used with or without the interrupt
 * pin. Per default this pin is mapped to @ref GPIO_UNDEF if not otherwise defined
 * in your makefile.
 *
 * If you use an electrical isolation for most accurate readings
 * e.g. with the ADM3260, keep in mind that its not recommended to use the
 * interrupt pin without also isolating it somehow. The preferred method,
 * if not using an isolation on the interrupt line, would be polling. In this case
 * leave the interrupt pin undefined.
 *
 * The Sensor has integrated temperature sensor and for the highest possible
 * precision it requires another device to provide the temperature for error
 * compensation.
 *
 * Once the RGB EZO is powered on it will be ready to receive commands but it takes
 * 10s to warm up so it can take readings.
 *
 * @note This driver provides @ref drivers_saul capabilities.
 * Reading (@ref saul_driver_t.read) from the device returns the current RGB value.
 *
 * @note Communication is done using SMBus/I2C protocol at speeds
 * of 10-100 kHz. Set your board I2C speed to @ref I2C_SPEED_LOW or
 * @ref I2C_SPEED_NORMAL
 *
 * @{
 *
 * @file
 * @brief       Device driver for Atlas Scientific RGB EZO Sensor with SMBus/I2C interface

 * @author      Ting XU <timtsui@outlook.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 */

#ifndef RGB_EZO_H
#define RGB_EZO_H

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
    RGB_EZO_OK              = 0,    /**< Everything was fine */
    RGB_EZO_NODEV           = -1,   /**< No device found on the bus */
    RGB_EZO_READ_ERR        = -2,   /**< Reading to device failed*/
    RGB_EZO_WRITE_ERR       = -3,   /**< Writing to device failed */
    RGB_EZO_NOT_RGB         = -4,   /**< Not an Atlas Scientific RGB EZO device */
    RGB_EZO_OUT_OF_RANGE    = -5,   /**< Input value out of accepted range */
    RGB_EZO_WARMING         = -6,   /**< RGB EZO still warming up */
    RGB_EZO_INT_GPIO_UNDEF  = -7,   /**< Alarm interrupt pin undefined */
    RGB_EZO_GPIO_INIT_ERR   = -8    /**< Error while initializing GPIO PIN */
} rgb_ezo_named_returns_t;

/**
 * @brief   LED state values
 */
typedef enum {
    RGB_EZO_LED_ON  = 0x01, /**< LED on state */
    RGB_EZO_LED_OFF = 0x00, /**< LED off state */
} rgb_ezo_led_state_t;


/**
 * @brief  Paremeters of the output string
 */
typedef enum {
    RGB_EZO_RGB = 0x01,     /**< RBG output on state */
    RGB_EZO_LUX = 0x02,     /**< LUX output on state */
    RGB_EZO_CIE =  0x03,    /**< CIE output on state */
} rgb_ezo_parameter_t;

/**
 * @brief   Alarm state values
 */
typedef enum {
    RGB_EZO_ALARM_ON    = 0x01, /**< Alarm on state */
    RGB_EZO_ALARM_OFF   = 0x00, /**< Alarm off state */
} rgb_ezo_alarm_state_t;

/**
 * @brief   RGB EZO sensor params
 */
typedef struct rgb_ezo_params {
    i2c_t i2c;              /**< I2C device the sensor is connected to */
    uint8_t addr;           /**< the slave address of the sensor on the I2C bus */
    gpio_t alarm_int_pin;   /**< interrupt pin (@ref GPIO_UNDEF if not defined) */
} rgb_ezo_params_t;

/**
 * @brief   pH EZO device descriptor
 */
typedef struct rgb_ezo {
    rgb_ezo_params_t params; /**< device driver configuration */
} rgb_ezo_t;

/**
 * @brief   RGB EZO interrupt pin callback
 */
typedef void (*rgb_ezo_interrupt_pin_cb_t)(void *);

/**
 * @brief   Initialize a RGB EZO sensor
 *
 * @param[in,out]   dev      device descriptor
 * @param[in]       params   device configuration
 *
 * @return @ref RGB_EZO_OK      on success
 * @return @ref RGB_EZO_NODEV   if no device is found on the bus
 * @return @ref RGB_EZO_NOT_RGB if the device found at the address is not a RGB EZO device
 */
int rgb_ezo_init(rgb_ezo_t *dev, const rgb_ezo_params_t *params);

/**
 * @brief   Set the LED state of the RGB EZO sensor
 *
 * @param[in] dev       device descriptor
 * @param[in] state     @ref rgb_ezo_led_state_t
 *
 * @return @ref RGB_EZO_OK on success
 * @return @ref RGB_EZO_WRITE_ERR if writing to the device failed
 */
int rgb_ezo_set_indicator_led_state(rgb_ezo_t *dev, rgb_ezo_led_state_t state);

/**
 * @brief   Sets a new I2C address to the RGB EZO device and reboot to I2C mode
 *          The device address will also be updated in the device descriptor so
 *          it is still usable.
 *
 * @note    Changing the I2C address will prevent communication between the circuit
 *          and the CPU until the CPU is updated with the new I2C address.
 *
 * @param[in] dev   device descriptor
 * @param[in] addr  new address for the device. Range: 0x01 - 0x7f
 *
 * @return @ref RGB_EZO_OK on success
 * @return @ref RGB_EZO_WRITE_ERR if writing to the device failed
 */
int rgb_ezo_set_i2c_address(rgb_ezo_t *dev, uint8_t addr);

/**
 * @brief   Calibrate the sensor by placing white object in front of target
 *
 * @param[in,out]   dev      device descriptor
 *
 * @return @ref RGB_EZO_OK      on success
 * @return @ref RGB_EZO_NODEV   if no device is found on the bus
 * @return @ref RGB_EZO_NOT_RGB if the device found at the address is not a RGB EZO device
 */
int rgb_ezo_calibration(rgb_ezo_t *dev);

/**
 * @brief   Setting the gamma correction by sending a floating point number
 *          from 0,01 - 4,99, but send the value after multiplying 100 so the
 *          value would be an integer.
 *
 * @note    The default gamma correction is 1-00 which represents no correction
 *          at all. A gamma correction factor is a floating point number from 0.01 to 4.99.
 *
 * @param[in] dev       device descriptor
 *
 * @return @ref RGB_EZO_OK        on success
 * @return @ref RGB_EZO_WRITE_ERR if writing to the device failed
 */
int rgb_ezo_set_gamma_correction(rgb_ezo_t *dev, uint16_t value);

/**
 * @brief   Gets the gamma correction value
 *
 * @param[in]  dev                   device descriptor
 * @param[out] correction_value      correction value from device.
 *
 * @note    The correction value is in raw so you need to divide it by 100.
 *
 * @return @ref RGB_EZO_OK        on success
 * @return @ref RGB_EZO_WRITE_ERR if writing to the device failed
 */
int rgb_ezo_get_gamma_correction(rgb_ezo_t *dev, uint16_t *correction_value);

/**
 * @brief   Enables the parameters from output string.
 *
 * @param[in] dev                 device descriptor
 * @param[in] parameter           the parameter that you want to enable
 *
 * @return @ref RGB_EZO_OK        on success
 * @return @ref RGB_EZO_WRITE_ERR if writing to the device failed
 */
int rgb_ezo_set_parameters_state(rgb_ezo_t *dev, rgb_ezo_parameter_t parameter,
                                 bool enabled);

/**
 * @brief   Reads the current RGB value by sending command 'R' to device.
 *
 * @param[in]  dev                   device descriptor
 * @param[out] string    			 enabled parameters
 *
 * @return @ref RGB_EZO_OK          on success
 * @return @ref RGB_EZO_READ_ERR    if reading from the device failed
 * @return @ref RGB_EZO_WARMING		if sensor is still under warm-up process
 */
int rgb_ezo_get_parameter_state(rgb_ezo_t *dev, char *string);

/**
 * @brief   Turn the RGB EZO sensor to sleep mode
 *
 * @param[in] dev                 device descriptor
 *
 * @return @ref RGB_EZO_OK        on success
 * @return @ref RGB_EZO_WRITE_ERR if writing to the device failed
 */
int rgb_ezo_sleep_mode(rgb_ezo_t *dev);

/**
 * @brief   Reads the current RGB value by sending command 'R' to device.
 *
 * @param[in]  dev                   device descriptor
 * @param[out] rgb_value             rgb value <br>

 *
 * @return @ref RGB_EZO_OK          on success
 * @return @ref RGB_EZO_READ_ERR    if reading from the device failed
 */
int rgb_ezo_read_rgb(rgb_ezo_t *dev, uint16_t *r_value,uint16_t *g_value,uint16_t *b_value);


#ifdef __cplusplus
}
#endif

#endif /* RGB_EZO_H */
/** @} */
