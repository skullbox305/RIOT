/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_co2_ezo CO2 EZO sensor device driver
 * @ingroup     drivers_sensors
 * @ingroup     drivers_saul
 * @brief       Device driver for Atlas Scientific CO2 EZO sensor with SMBus/I2C interface
 *
 * The Atlas Scientific CO2 EZO sensor can be used with or without the interrupt
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
 * Once the CO2 EZO is powered on it will be ready to receive commands but it takes
 * 10s to warm up so it can take readings.
 *
 * @note This driver provides @ref drivers_saul capabilities.
 * Reading (@ref saul_driver_t.read) from the device returns the current CO2 value.
 *
 * @note Communication is done using SMBus/I2C protocol at speeds
 * of 10-100 kHz. Set your board I2C speed to @ref I2C_SPEED_LOW or
 * @ref I2C_SPEED_NORMAL
 *
 * @{
 *
 * @file
 * @brief       Device driver for Atlas Scientific CO2 EZO Sensor with SMBus/I2C interface

 * @author      Ting XU <timtsui@outlook.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 */

#ifndef CO2_EZO_H
#define CO2_EZO_H

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
    CO2_EZO_OK              =  0,    /**< Everything was fine */
    CO2_EZO_NODEV           = -1,   /**< No device found on the bus */
    CO2_EZO_READ_ERR        = -2,   /**< Reading to device failed*/
    CO2_EZO_WRITE_ERR       = -3,   /**< Writing to device failed */
    CO2_EZO_NOT_CO2         = -4,   /**< Not an Atlas Scientific CO2 EZO device */
    CO2_EZO_OUT_OF_RANGE    = -5,   /**< Input value out of accepted range */
	CO2_EZO_WARMING    		= -6,   /**< CO2 EZO still warming up */
	CO2_EZO_INT_GPIO_UNDEF  = -7,   /**< Alarm interrupt pin undefined */
	CO2_EZO_GPIO_INIT_ERR   = -8    /**< Error while initializing GPIO PIN */
} co2_ezo_named_returns_t;

/**
 * @brief   LED state values
 */
typedef enum {
    CO2_EZO_LED_ON  = 0x01, /**< LED on state */
    CO2_EZO_LED_OFF = 0x00, /**< LED off state */
} co2_ezo_led_state_t;

/**
 * @brief   Alarm state values
 */
typedef enum {
    CO2_EZO_ALARM_ON    = 0x01, /**< Alarm on state */
    CO2_EZO_ALARM_OFF   = 0x00, /**< Alarm off state */
} co2_ezo_alarm_state_t;

/**
 * @brief   CO2 EZO sensor params
 */
typedef struct co2_ezo_params {
    i2c_t i2c;            /**< I2C device the sensor is connected to */
    uint8_t addr;         /**< the slave address of the sensor on the I2C bus */
    gpio_t alarm_int_pin; /**< interrupt pin (@ref GPIO_UNDEF if not defined) */
} co2_ezo_params_t;

/**
 * @brief   pH EZO device descriptor
 */
typedef struct co2_ezo {
    co2_ezo_params_t params; /**< device driver configuration */
} co2_ezo_t;

/**
 * @brief   CO2 EZO interrupt pin callback
 */
typedef void (*co2_ezo_interrupt_pin_cb_t)(void *);

/**
 * @brief   Initialize a CO2 EZO sensor
 *
 * @param[in,out]   dev      device descriptor
 * @param[in]       params   device configuration
 *
 * @return @ref CO2_EZO_OK 		on success
 * @return @ref CO2_EZO_NODEV 	if no device is found on the bus
 * @return @ref CO2_EZO_NOT_CO2 if the device found at the address is not a CO2 EZO device
 * @return
 */
int co2_ezo_init(co2_ezo_t *dev, const co2_ezo_params_t *params);

/**
 * @brief   Set the LED state of the CO2 EZO sensor
 *
 * @param[in] dev       device descriptor
 * @param[in] state     @ref co2_ezo_led_state_t
 *
 * @return @ref CO2_EZO_OK on success
 * @return @ref CO2_EZO_WRITE_ERR if writing to the device failed
 */
int co2_ezo_set_led_state(co2_ezo_t *dev, co2_ezo_led_state_t state);

/**
 * @brief   Sets a new I2C address to the CO2 EZO device and reboot to I2C mode
 *          The device address will also be updated in the device descriptor so
 *          it is still usable.
 *
 * @note    Changing the I2C address will prevent communication between the circuit
 *          and the CPU until the CPU is updated with the new I2C address.
 *
 * @param[in] dev   device descriptor
 * @param[in] addr  new address for the device. Range: 0x01 - 0x7f
 *
 * @return @ref CO2_EZO_OK on success
 * @return @ref CO2_EZO_WRITE_ERR if writing to the device failed
 */
int co2_ezo_set_i2c_address(co2_ezo_t *dev, uint8_t addr);

/**
 * @brief   Enable the CO2 alarm and provide alarm value and tolerance value.
 *
 * @note	The alarm pin will = 1 when CO2 levels are > alarm set point.
 *          Alarm tolerance sets how far below the set point CO2 levels
 *          need to drop before the pin will = 0 again.
 *
 * @param[in] dev       device descriptor
 * @param[in] value     alarm value
 * @param[in] tolerance tolerance value 0 - 499 (not 500 how Atlas said)
 * @param[in] cb        callback called when the CO2 EZO alarm int pin fires
 * @param[in] arg       callback argument
 *
 * @return              @ref CO2_EZO_OK on success
 * @return              @ref CO2_EZO_INT_GPIO_UNDEF
 * @return              @ref CO2_EZO_OUT_OF_RANGE
 * @return              @ref CO2_EZO_GPIO_INIT_ERR
 * @return              @ref CO2_EZO_WRITE_ERR if writing to the device failed
 */
int co2_ezo_enable_alarm(co2_ezo_t *dev, uint16_t value, uint16_t tolerance,
                         co2_ezo_interrupt_pin_cb_t cb,
                         void *arg);

/**
 * @brief   Disable Alarm mode.
 *
 * @param[in] dev       device descriptor
 *
 * @return @ref CO2_EZO_OK 		  on success
 * @return @ref CO2_EZO_WRITE_ERR if writing to the device failed
 */
int co2_ezo_disable_alarm(co2_ezo_t *dev);

/**
 * @brief   Get alarm state if all are enabled.
 *
 * @param[in]  dev              Device descriptor
 * @param[out] alarm_value      Alarm value read from device.
 * @param[out] tolerance_value  Tolerance value read from device.
 * @param[out] enabled          Alarm enabled = true, disabled = false.
 *
 *
 * @return @ref CO2_EZO_OK 		  on success
 * @return @ref CO2_EZO_WRITE_ERR if writing to the device failed
 */
int co2_ezo_get_alarm_state(co2_ezo_t *dev, uint16_t *alarm_value,
                            uint16_t *tolerance_value, bool *enabled);

/**
 * @brief   Turn the CO2 EZO sensor to sleep mode
 *
 * @param[in] dev       		  device descriptor
 *
 * @return @ref CO2_EZO_OK 		  on success
 * @return @ref CO2_EZO_WRITE_ERR if writing to the device failed
 */
int co2_ezo_sleep_mode(co2_ezo_t *dev);

/**
 * @brief   Reads the current CO2 value by sending command 'R' to device.
 *
 * @param[in]  dev                   device descriptor
 * @param[out] co2_value             co2 value in ppm <br>
 * @param[out] internal_temperature  internal temperature of the sensor <br>

 *
 * @return @ref CO2_EZO_OK          on success
 * @return @ref CO2_EZO_READ_ERR    if reading from the device failed
 * @return @ref CO2_EZO_WARMING		if sensor is still under warm-up process
 */
int co2_ezo_read_co2(co2_ezo_t *dev, uint16_t *co2_value,  uint32_t *internal_temperature);


#ifdef __cplusplus
}
#endif

#endif /* CO2_EZO_H */
/** @} */
