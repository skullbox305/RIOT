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
 * If you use an electrical isolation for most accurate readings
 * e.g. with the ADM3260, keep in mind that its not recommended to use the
 * interrupt pin without also isolating it somehow. The preferred method,
 * if not using an isolation on the interrupt line, would be polling. In this case
 * leave the interrupt pin undefined.
 *
 * @note This driver provides @ref drivers_saul capabilities.
 * Reading (@ref saul_driver_t.read) from the device returns the current D.O.
 * value.
 *
 * The Sensor has no integrated temperature sensor,pressure sensor or salinity
 * sensor and for the highest possible precision it requires another device to
 * provide the temperature, the pressure and the salinity for error compensation.
 *
 *
 * Once the DO OEM is powered on it will be ready to receive commands and take
 * readings after 1ms.
 *
 * @note This driver provides @ref drivers_saul capabilities.
 * Reading (@ref saul_driver_t.read) from the device returns the current D.O. values.
 * Writing (@ref saul_driver_t.write) salinity value, the pressure value and the
 * temperature values to the device sets the compensation
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
typedef enum
{
	DO_OEM_OK = 0, /**< Everything was fine */
	DO_OEM_NODEV = -1, /**< No device found on the bus */
	DO_OEM_READ_ERR = -2, /**< Reading to device failed*/
	DO_OEM_WRITE_ERR = -3, /**< Writing to device failed */
	DO_OEM_NOT_DO = -4, /**< Not an Atlas Scientific DO OEM device */
	DO_OEM_INTERRUPT_GPIO_UNDEF = -5, /**< Interrupt pin is @ref GPIO_UNDEF */
	DO_OEM_GPIO_INIT_ERR = -6, /**< Error while initializing GPIO PIN */
	DO_OEM_SALI_OUT_OF_RANGE = -7, /**< Salinity is out of range */
	DO_OEM_PRES_OUT_OF_RANGE = -8, /**< Pressure is out of range */
	DO_OEM_TEMP_OUT_OF_RANGE = -9, /**< Temperature is out of range */
} do_oem_named_returns_t;

/**
 * @brief   LED state values
 */
typedef enum
{
	DO_OEM_LED_ON = 0x01, /**< LED on state */
	DO_OEM_LED_OFF = 0x00, /**< LED off state */
} do_oem_led_state_t;

/**
 * @brief   Device state values
 */
typedef enum
{
	DO_OEM_TAKE_READINGS = 0x01, /**< Device active state */
	DO_OEM_STOP_READINGS = 0x00, /**< Device hibernate state */
} do_oem_device_state_t;

/**
 * @brief   Calibration option values
 */
typedef enum
{
	DO_OEM_CALIBRATE_CLEAR = 0x01, /**< Clear calibration */
	DO_OEM_CALIBRATE_ATMOSPHERIC = 0x02, /**< Calibrate to atmospheric oxygen content */
	DO_OEM_CALIBRATE_0_DISSOLVED = 0x03, /**< Calibrate to 0 dissolved oxygen */
} do_oem_calibration_option_t;

/**
 * @brief   Interrupt pin option values
 */
typedef enum
{
	DO_OEM_IRQ_RISING = 0x02, /**< Pin high on new reading (manually reset) */
	DO_OEM_IRQ_FALLING = 0x04, /**< Pin low on new reading (manually reset) */
	DO_OEM_IRQ_BOTH = 0x08, /**< Invert state on new reading (automatically reset) */
} do_oem_irq_option_t;

/**
 * @brief   DO OEM sensor params
 */
typedef struct do_oem_params
{
	i2c_t i2c; /**< I2C device the sensor is connected to */
	uint8_t addr; /**< the slave address of the sensor on the I2C bus */
	gpio_t interrupt_pin; /**< interrupt pin (@ref GPIO_UNDEF if not defined) */
	gpio_mode_t gpio_mode; /**< gpio mode of the interrupt pin */
	do_oem_irq_option_t irq_option; /**< behavior of the interrupt pin, disabled by default */
} do_oem_params_t;

/**
 * @brief   DO OEM interrupt pin callback
 */
typedef void (*do_oem_interrupt_pin_cb_t)(void *);

/**
 * @brief   DO OEM device descriptor
 */
typedef struct do_oem
{
	do_oem_params_t params; /**< device driver configuration */
	do_oem_interrupt_pin_cb_t cb; /**< interrupt pin callback */
	void *arg; /**< interrupt pin callback param */
} do_oem_t;

/**
 * @brief   Initialize a DO OEM sensor
 *
 * @param[in,out]   dev      device descriptor
 * @param[in]       params   device configuration
 *
 * @return @ref DO_OEM_OK	  on success
 * @return @ref DO_OEM_NODEV  if no device is found on the bus
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
 * @return @ref DO_OEM_OK 		 on success
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
 * @return @ref DO_OEM_OK		 on success
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
 * @return @ref DO_OEM_OK		  on success
 * @return @ref DO_OEM_WRITE_ERR  if writing to the device failed
 * @return @ref DO_OEM_READ_ERR   if reading from the device failed
 */
int do_oem_start_new_reading(const do_oem_t *dev);

/**
 * @brief   Clears all calibrations previously done
 *
 * @param[in] dev   device descriptor
 *
 * @return @ref DO_OEM_OK 		  on success
 * @return @ref DO_OEM_WRITE_ERR  if writing to the device failed
 * @return @ref DO_OEM_READ_ERR	  if reading from the device failed
 */
int do_oem_clear_calibration(const do_oem_t *dev);

/**
 * @brief   Read the @ref DO_OEM_REG_CALIBRATION_SALI_CONFIRM register.
 *          After a calibration event has been successfully carried out, the
 *          calibration confirmation register will reflect what calibration has
 *          been done, by setting bits 0 - 3.
 *
 * @param[in]  dev                 device descriptor
 * @param[out] calibration_state   calibration state reflected by bits 0 - 3 <br>
 *                                 (0 = no calibration, 1 = calibrated to atmosphere
 *                                 2 = calibrated to 0 Dissolved Oxygen 3 = calibrated to
 *                                 both atmospheric and 0 dissolved Oxygen)
 *
 * @return @ref DO_OEM_OK 		on success
 * @return @ref DO_OEM_READ_ERR if reading from the device failed
 */
int do_oem_read_calibration_state(const do_oem_t *dev,
		uint8_t *calibration_state);

/**
 * @brief   Sets the @ref DO_OEM_REG_CALIBRATION_REQUEST register to the
 *          calibration type which the DO OEM sensor will using.
 * @note    The calibration process takes 40ms.
 *
 * @param[in] dev                 device descriptor
 * @param[in] option              @ref do_oem_calibration_option_t
 *
 * @return @ref DO_OEM_OK 		  on success
 * @return @ref DO_OEM_WRITE_ERR  if writing to the device failed
 * @return @ref DO_OEM_READ_ERR   if reading from the device failed
 */
int do_oem_set_calibration(const do_oem_t *dev,
		do_oem_calibration_option_t option);

/**
 * @brief   Sets the @ref DO_OEM_REG_SALI_COMPENSATION_BASE register to the
 *          salinity_compensation value which the DO OEM sensor will use
 *          to compensate the reading error.
 *          Multiply the floating point temperature value by 100
 *          e.g. salinity in microsiemens = 56000 * 100 = 5600000
 *
 * @note   The salinity compensation will not be retained if the power is cut.
 *
 * @param[in] dev                        device descriptor
 * @param[in] salinity_compensation   	 valid salinity range is
 * 										  (Unknown)
 *
 * @return @ref DO_OEM_OK 				on success
 * @return @ref DO_OEM_WRITE_ERR 		if writing to the device failed
 * @return @ref DO_OEM_SAL_OUT_OF_RANGE if the salinity_compensation is not in
 *                                       the valid range
 */
int do_oem_set_sal_compensation(const do_oem_t *dev,
		uint32_t salinity_compensation);

/**
 * @brief   Sets the @ref DO_OEM_REG_PRES_COMPENSATION_BASE register to the
 *          pressure_compensation value which the DO OEM sensor will use
 *          to compensate the reading error.
 *          Multiply the floating point pressure value by 100
 *          e.g. pressure in kilopascals = 34.26 * 100 = 3426
 *
 *  @note   The pressure compensation will not be retained if the power is cut.
 *
 * @param[in] dev                        device descriptor
 * @param[in] pressure_compensation  	 valid pressure range is
 *                                       (Unknown)
 *
 * @return @ref DO_OEM_OK on success
 * @return @ref DO_OEM_WRITE_ERR 		 if writing to the device failed
 * @return @ref DO_OEM_PRES_OUT_OF_RANGE if the pressure_compensation is not in
 *                                       the valid range
 */
int do_oem_set_pres_compensation(const do_oem_t *dev,
		uint32_t presssure_compensation);

/**
 * @brief   Sets the @ref DO_OEM_REG_TEMP_COMPENSATION_BASE register to the
 *          temperature_compensation value which the DO OEM sensor will use
 *          to compensate the reading error.
 *          Multiply the floating point temperature value by 100
 *          e.g. temperature in degree Celsius = 13.78 * 100 = 1378
 *
 *  @note   The temperature compensation will not be retained if the power is cut.
 *
 * @param[in] dev                        device descriptor
 * @param[in] temperature_compensation   valid temperature range is
 *                                       (Unknown)
 *
 * @return @ref DO_OEM_OK                on success
 * @return @ref DO_OEM_WRITE_ERR 		 if writing to the device failed
 * @return @ref DO_OEM_TEMP_OUT_OF_RANGE if the temperature_compensation is not in
 *                                       the valid range
 */
int do_oem_set_temp_compensation(const do_oem_t *dev,
		uint16_t temperature_compensation);
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
 * @return @ref DO_OEM_			 OK on success
 * @return @ref DO_OEM_WRITE_ERR if writing to the device failed
 */
int do_oem_reset_interrupt_pin(const do_oem_t *dev);

/**
 * @brief   Reads the @ref DO_OEM_REG_SAL_CONFIRMATION_BASE register to verify
 *          the salinity compensation value that was used to take the reading
 *          is set to the correct temperature.
 *
 * @param[in]  dev                       device descriptor
 * @param[out] salinity_compensation 	 raw salinity compensation value. <br>
 *                                       Divide by 100 for floating point <br>
 *                                       e.g 3426 / 100 = 34.26
 *
 * @return @ref DO_OEM_OK 			on success
 * @return @ref DO_OEM_READ_ERR 	if reading from the device failed
 */
int do_oem_read_sali_compensation(const do_oem_t *dev,
		uint32_t *salinity_compensation);

/**
 * @brief   Reads the @ref DO_OEM_REG_PRES_CONFIRMATION_BASE register to verify
 *          the pressure compensation value that was used to take the reading
 *          is set to the correct pressure.
 *
 * @param[in]  dev                       device descriptor
 * @param[out] pressure_compensation 	 raw pressure compensation value. <br>
 *                                       Divide by 100 for floating point <br>
 *                                       e.g 3426 / 100 = 34.26
 *
 * @return @ref DO_OEM_OK 				 on success
 * @return @ref DO_OEM_READ_ERR 		 if reading from the device failed
 */
int do_oem_read_pres_compensation(const do_oem_t *dev,
		uint32_t *pressure_compensation);

/**
 * @brief   Reads the @ref DO_OEM_REG_TEMP_CONFIRMATION_BASE register to verify
 *          the temperature compensation value that was used to take the reading
 *          is set to the correct temperature.
 *
 * @param[in]  dev                       device descriptor
 * @param[out] temperature_compensation  raw temperature compensation value. <br>
 *                                       Divide by 100 for floating point <br>
 *                                       e.g 3426 / 100 = 34.26
 *
 * @return @ref DO_OEM_OK 				 on success
 * @return @ref DO_OEM_READ_ERR 		 if reading from the device failed
 */
int do_oem_read_temp_compensation(const do_oem_t *dev,
		uint16_t *temperature_compensation);

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
 * @param[in] dev      			 			device descriptor
 * @param[in] cb        		 			callback called when the DO OEM interrupt pin fires
 * @param[in] arg                			callback argument
 *
 * @return @ref DO_OEM_OK 		 			on success
 * @return @ref DO_OEM_WRITE_ERR            if writing to the device failed
 * @return @ref DO_OEM_INTERRUPT_GPIO_UNDEF if the interrupt pin is undefined
 * @return @ref DO_OEM_GPIO_INIT_ERR 		if initializing the interrupt gpio pin failed
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
 * @param[in] dev   				device descriptor
 * @param[in] state @ref    		do_oem_device_state_t
 *
 * @return @ref DO_OEM_OK 			on success
 * @return @ref DO_OEM_WRITE_ERR 	if writing to the device failed
 */
int do_oem_set_device_state(const do_oem_t *dev, do_oem_device_state_t state);

/**
 * @brief   Reads the @ref DO_OEM_REG_DO_MGL_READING_BASE register to get the current
 *          D.O. reading.
 *
 * @param[in]  dev        	 	 	device descriptor
 * @param[out] do_mg_value 	 	 	raw do value in mg/L <br>
 *                        	 		divide by 100 for floating point <br>
 *                        	  	 	e.g 834 / 100 = 8.34
 *
 * @return @ref DO_OEM_OK 			on success
 * @return @ref DO_OEM_READ_ERR 	if reading from the device failed
 */
int do_oem_read_do_mg(const do_oem_t *dev, uint16_t *do_mg_value);

/**
 * @brief   Reads the @ref DO_OEM_REG_DO_PERCENT_READING_BASE register to get the current
 *          pressure reading.
 *
 * @param[in]  dev        			device descriptor
 * @param[out] do_percent_value 	raw do value in % saturation <br>
 *                        			divide by 100 for floating point <br>
 *                       			e.g 834 / 100 = 8.34
 *
 * @return @ref DO_OEM_OK 			on success
 * @return @ref DO_OEM_READ_ERR 	if reading from the device failed
 */
int do_oem_read_do_percent(const do_oem_t *dev, uint16_t *do_percent_value);

#ifdef __cplusplus
}
#endif

#endif /* DO_OEM_H */
/** @} */
