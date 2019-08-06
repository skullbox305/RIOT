/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_ph_ezo pH EZO sensor device driver
 * @ingroup     drivers_sensors
 * @ingroup     drivers_saul
 * @brief       Device driver for Atlas Scientific pH EZO sensor with SMBus/I2C interface
 *
 * The Atlas Scientific pH EZO sensor can be used with or without the interrupt
 * pin. Per default this pin is mapped to @ref GPIO_UNDEF if not otherwise defined
 * in your makefile.
 *
 * If you use an electrical isolation for most accurate readings
 * e.g. with the ADM3260, keep in mind that its not recommended to use the
 * interrupt pin without also isolating it somehow. The preferred method,
 * if not using an isolation on the interrupt line, would be polling. In this case
 * leave the interrupt pin undefined.
 *
 * The Sensor has no integrated temperature sensor and for the highest possible
 * precision it requires another device to provide the temperature for error
 * compensation.
 *
 * @note This driver provides @ref drivers_saul capabilities.
 * Reading (@ref saul_driver_t.read) from the device returns the current pH value.
 * Writing (@ref saul_driver_t.write) a temperature value in celsius to the
 * device sets the temperature compensation. A valid temperature range is
 * 0.01 °C  to  200.0 °C
 *
 * @note Communication is done using SMBus/I2C protocol at speeds
 * of 100-400 kHz. Set your board I2C speed to @ref I2C_SPEED_LOW or
 * @ref I2C_SPEED_NORMAL
 *
 * @{
 *
 * @file
 * @brief       Device driver for Atlas Scientific PH EZO Sensor with SMBus/I2C interface

 * @author      Ting XU <timtsui@outlook.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 */

#ifndef PH_EZO_H
#define PH_EZO_H

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
    PH_EZO_OK           	 =  0,   /**< Everything was fine */
    PH_EZO_NODEV         	 = -1,   /**< No device found on the bus */
    PH_EZO_READ_ERR      	 = -2,   /**< Reading to device failed*/
    PH_EZO_WRITE_ERR     	 = -3,   /**< Writing to device failed */
    PH_EZO_NOT_PH         	 = -4,   /**< Not an Atlas Scientific PH EZO device */
	PH_EZO_TEMP_OUT_OF_RANGE = -5, 	 /**< Compensation value out of accepted range  */
	PH_EZO_INT_GPIO_UNDEF  	 = -6,   /**< Interrupt pin undefined */
	PH_EZO_GPIO_INIT_ERR   	 = -7    /**< Error while initializing GPIO PIN */
} ph_ezo_named_returns_t;

/**
 * @brief   LED state values
 */
typedef enum {
    PH_EZO_LED_ON  = 0x01, /**< LED on state */
    PH_EZO_LED_OFF = 0x00, /**< LED off state */
} ph_ezo_led_state_t;

/**
 * @brief   type of calibration
 */
typedef enum {
	PH_EZO_CALIBRATE_LOW_POINT  = 0x00, /**< Low point calibration option */
	PH_EZO_CALIBRATE_MID_POINT  = 0x01, /**< Mid point calibration option */
	PH_EZO_CALIBRATE_HIGH_POINT = 0x02, /**< High point calibration option */
} ph_ezo_calibration_option_t;

/**
 * @brief   state of calibration
 */
typedef enum {
	PH_EZO_NOT_CALIBRATED   = 0x00, /**< Not calibrated */
	PH_EZO_CALIBRATE_ONE    = 0x01, /**< One point calibrated */
	PH_EZO_CALIBRATE_TWO    = 0x02, /**< Two point calibrated */
	PH_EZO_CALIBRATE_THREE  = 0x03, /**< Three point calibrated */
} ph_ezo_calibration_state_t;

/**
 * @brief   pH EZO sensor params
 */
typedef struct ph_ezo_params {
    i2c_t i2c;            /**< I2C device the sensor is connected to */
    uint8_t addr;         /**< the slave address of the sensor on the I2C bus */
    gpio_t alarm_int_pin; /**< interrupt pin (@ref GPIO_UNDEF if not defined) */
} ph_ezo_params_t;

/**
 * @brief   pH EZO device descriptor
 */
typedef struct ph_ezo {
    ph_ezo_params_t params; /**< device driver configuration */
} ph_ezo_t;

/**
 * @brief   pH EZO interrupt pin callback
 */
typedef void (*ph_ezo_interrupt_pin_cb_t)(void *);

/**
 * @brief   Initialize a pH EZO sensor
 *
 * @param[in,out]   dev      device descriptor
 * @param[in]       params   device configuration
 *
 * @return @ref PH_EZO_OK 		on success
 * @return @ref PH_EZO_NODEV 	if no device is found on the bus
 * @return @ref PH_EZO_NOT_PH if the device found at the address is not a pH EZO device
 */
int ph_ezo_init(ph_ezo_t *dev, const ph_ezo_params_t *params);


/**
 * @brief   Enable factory reset
 *
 * @param[in] dev      			  device descriptor
 *
 * @return @ref PH_EZO_OK 		  on success
 * @return @ref PH_EZO_WRITE_ERR  if writing to the device failed
 */
int ph_ezo_reset(ph_ezo_t *dev);


/**
 * @brief   Set the LED state of the pH EZO sensor
 *
 * @param[in] dev       device descriptor
 * @param[in] state     @ref ph_ezo_led_state_t
 *
 * @return @ref PH_EZO_OK on success
 * @return @ref PH_EZO_WRITE_ERR if writing to the device failed
 */
int ph_ezo_set_led_state(ph_ezo_t *dev, ph_ezo_led_state_t state);

/**
 * @brief   Sets a new I2C address to the PH EZO device and reboot to I2C mode
 *          The device address will also be updated in the device descriptor so
 *          it is still usable.
 *
 * @note    Changing the I2C address will prevent communication between the circuit
 *          and the CPU until the CPU is updated with the new I2C address.
 *
 * @param[in] dev   device descriptor
 * @param[in] addr  new address for the device. Range: 0x01 - 0x7f
 *
 * @return @ref PH_EZO_OK on success
 * @return @ref PH_EZO_WRITE_ERR if writing to the device failed
 */
int ph_ezo_set_i2c_address(ph_ezo_t *dev, uint8_t addr);

/**
 * @brief   Clear previous calibration.
 *
 * @param[in]  dev              	Device descriptor
 *
 *
 * @return @ref PH_EZO_OK 		    on success
 * @return @ref PH_EZO_WRITE_ERR    if writing to the device failed
 */
int ph_ezo_clear_calibration(ph_ezo_t *dev);

/**
 * @brief   Enable the PH alarm and provide alarm value and tolerance value.
 *
 * @note	The alarm pin will = 1 when PH levels are > alarm set point.
 *          Alarm tolerance sets how far below the set point PH levels
 *          need to drop before the pin will = 0 again.
 *
 * @param[in] dev       			device descriptor
 * @param[in] calibration_value     calibration value
 * @param[in] option       			calibration option
 *
 * @return  @ref PH_EZO_OK 		  on success
 * @return  @ref PH_EZO_WRITE_ERR if writing to the device failed
 * @return 	@ref PH_EZO_READ_ERR  if reading the device failed
 */
int ph_ezo_set_calibration(ph_ezo_t *dev, uint16_t calibration_value,
		ph_ezo_calibration_option_t option);

/**
 * @brief   Get alarm state if all are enabled.
 *
 * @param[in]  dev              	Device descriptor
 * @param[out] cal_state		    Calibration state
 *
 *
 * @return @ref PH_EZO_OK 		    on success
 * @return @ref PH_EZO_WRITE_ERR    if writing to the device failed
 */
int ph_ezo_get_calibration_state(ph_ezo_t *dev, uint16_t *cal_state);

/**
 * @brief   Turn the PH EZO sensor to sleep mode
 *
 * @param[in] dev       		  device descriptor
 *
 * @return @ref PH_EZO_OK 		  on success
 * @return @ref PH_EZO_WRITE_ERR if writing to the device failed
 */
int ph_ezo_sleep_mode(ph_ezo_t *dev);

/**
 * @brief   Return the slope of the pH probe.
 *
 * @param[in]  dev       		    device descriptor
 * @param[out] acid_slope		    how closely the slope of the acid calibration<br>
 * 									matched the "ideal" pH probe.
 * @param[out] base_slope		    how closely the slope of the base calibration<br>
 * 									matches the "ideal" pH probe.
 *
 * @note 	Acid_slope and base_value are both in raw. Don't forget to divide<br>
 *			by 10 for floating point. e.g 997 / 10 = 99.7
 *
 * @return @ref PH_EZO_OK 		  on success
 * @return @ref PH_EZO_WRITE_ERR  if writing to the device failed
 */
int ph_ezo_read_slope(ph_ezo_t *dev,uint16_t *acid_slope, uint16_t *base_slope);

/**
 * @brief   Reads the current PH value by sending command 'R' to device.
 *
 * @param[in]  dev                   device descriptor
 * @param[out] ph_value            	 raw pH value <br>
 *
 * @note 	pH value is in raw. Don't forget to divide by 10 for floating point.<br>
 * 			 e.g 997 / 10 = 99.7
 *
 * @return @ref PH_EZO_OK           on success
 * @return @ref PH_EZO_READ_ERR     if reading from the device failed
 */
int ph_ezo_read_ph(ph_ezo_t *dev, uint16_t *ph_value);


#ifdef __cplusplus
}
#endif

#endif /* PH_EZO_H */
/** @} */
