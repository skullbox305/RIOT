/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_orp_oem ORP OEM sensor device driver
 * @ingroup     drivers_sensors
 * @ingroup     drivers_saul
 * @brief       Device driver for Atlas Scientific ORP OEM sensor with SMBus/I2C interface
 *
 * The Atlas Scientific ORP OEM sensor can be used with or without the interrupt
 * pin. Per default this pin is mapped to @ref GPIO_UNDEF if not otherwise defined
 * in your makefile.
 *
 * If you use an electrical isolation for most accurate readings
 * e.g. with the ADM3260, keep in mind that its not recommended to use the
 * interrupt pin without also isolating it somehow. The preferred method,
 * if not using an isolation on the interrupt line, would be polling. In this case
 * leave the interrupt pin undefined.
 *
 * Once the ORP OEM is powered on it will be ready to receive commands and take
 * readings after 1ms.
 *
 * @note This driver provides @ref drivers_saul capabilities.
 * Reading (@ref saul_driver_t.read) from the device returns the current ORP value.
 *
 * @note Communication is done using SMBus/I2C protocol at speeds
 * of 10-100 kHz. Set your board I2C speed to @ref I2C_SPEED_LOW or
 * @ref I2C_SPEED_NORMAL
 *
 * @{
 *
 * @file
 * @brief       Device driver for Atlas Scientific ORP OEM Sensor with SMBus/I2C interface

 * @author      Ting Xu <timtsui@outlook.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 */

#ifndef ORP_OEM_H
#define ORP_OEM_H

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
    ORP_OEM_OK                   =  0,   /**< Everything was fine */
    ORP_OEM_NODEV                = -1,   /**< No device found on the bus */
    ORP_OEM_READ_ERR             = -2,   /**< Reading to device failed*/
    ORP_OEM_WRITE_ERR            = -3,   /**< Writing to device failed */
    ORP_OEM_NOT_ORP              = -4,   /**< Not an Atlas Scientific ORP OEM device */
    ORP_OEM_INTERRUPT_GPIO_UNDEF = -5,   /**< Interrupt pin is @ref GPIO_UNDEF */
    ORP_OEM_GPIO_INIT_ERR        = -6,   /**< Error while initializing GPIO PIN */
} orp_oem_named_returns_t;

/**
 * @brief   LED state values
 */
typedef enum {
    ORP_OEM_LED_ON   = 0x01, /**< LED on state */
    ORP_OEM_LED_OFF  = 0x00, /**< LED off state */
} orp_oem_led_state_t;

/**
 * @brief   Device state values
 */
typedef enum {
    ORP_OEM_TAKE_READINGS    = 0x01, /**< Device active state */
    ORP_OEM_STOP_READINGS    = 0x00, /**< Device hibernate state */
} orp_oem_device_state_t;

/**
 * @brief   Interrupt pin option values
 */
typedef enum {
    ORP_OEM_IRQ_RISING   = 0x02, /**< Pin high on new reading (manually reset) */
    ORP_OEM_IRQ_FALLING  = 0x04, /**< Pin low on new reading (manually reset) */
    ORP_OEM_IRQ_BOTH     = 0x08, /**< Invert state on new reading (automatically reset) */
} orp_oem_irq_option_t;

/**
 * @brief   Calibration option values
 */
typedef enum {
	 ORP_OEM_CALIBRATE_CLEAR         = 0x01, /**< Clear calibration */
	 ORP_OEM_CALIBRATE_SINGLE_POINT  = 0x02, /**< Single point calibration */
} orp_oem_calibration_option_t;

/**
 * @brief   ORP OEM sensor params
 */
typedef struct orp_oem_params {
    i2c_t i2c;                      /**< I2C device the sensor is connected to */
    uint8_t addr;                   /**< the slave address of the sensor on the I2C bus */
    gpio_t interrupt_pin;           /**< interrupt pin (@ref GPIO_UNDEF if not defined) */
    gpio_mode_t gpio_mode;          /**< gpio mode of the interrupt pin */
    orp_oem_irq_option_t irq_option; /**< behavior of the interrupt pin, disabled by default */
} orp_oem_params_t;

/**
 * @brief   ORP OEM interrupt pin callback
 */
typedef void (*orp_oem_interrupt_pin_cb_t)(void *);

/**
 * @brief   ORP OEM device descriptor
 */
typedef struct orp_oem {
    orp_oem_params_t params;         /**< device driver configuration */
    orp_oem_interrupt_pin_cb_t cb;   /**< interrupt pin callback */
    void *arg;                      /**< interrupt pin callback param */
} orp_oem_t;

/**
 * @brief   Initialize a ORP OEM sensor
 *
 * @param[in,out]   dev       	device descriptor
 * @param[in]       params    	device configuration
 *
 * @return @ref ORP_OEM_OK 	  	on success
 * @return @ref ORP_OEM_NODEV 	if no device is found on the bus
 * @return @ref ORP_OEM_NOT_ORP if the device found at the address is not a ORP OEM device
 * @return
 */
int orp_oem_init(orp_oem_t *dev, const orp_oem_params_t *params);

/**
 * @brief   Sets a new address to the ORP OEM device by unlocking the
 *          @ref ORP_OEM_REG_UNLOCK register and  writing a new address to
 *          the @ref ORP_OEM_REG_ADDRESS register.
 *          The device address will also be updated in the device descriptor so
 *          it is still usable.
 *
 *          Settings are retained in the sensor if the power is cut.
 *
 *          The address in the device descriptor will reverse to the default
 *          address you provided through ORP_OEM_PARAM_ADDR after the
 *          microcontroller restarts
 *
 * @param[in] dev   			  device descriptor
 * @param[in] addr  			  new address for the device. Range: 0x01 - 0x7f
 *
 * @return @ref ORP_OEM_OK  	  on success
 * @return @ref ORP_OEM_WRITE_ERR if writing to the device failed
 */
int orp_oem_set_i2c_address(orp_oem_t *dev, uint8_t addr);

/**
 * @brief   Enable the ORP OEM interrupt pin if @ref orp_oem_params_t.interrupt_pin
 *          is defined.
 *          @note @ref orp_oem_reset_interrupt_pin needs to be called in the
 *          callback if you use @ref ORP_OEM_IRQ_FALLING or @ref ORP_OEM_IRQ_RISING
 *
 *          @note Provide the ORP_OEM_PARAM_INTERRUPT_OPTION flag in your
 *          makefile. Valid options see: @ref orp_oem_irq_option_t.
 *          The default is @ref ORP_OEM_IRQ_BOTH.
 *
 *          @note Also provide the @ref gpio_mode_t as a CFLAG in your makefile.
 *          Most likely @ref GPIO_IN. If the pin is to sensitive use
 *          @ref GPIO_IN_PU for @ref ORP_OEM_IRQ_FALLING or
 *          @ref GPIO_IN_PD for @ref ORP_OEM_IRQ_RISING and
 *          @ref ORP_OEM_IRQ_BOTH. The default is @ref GPIO_IN_PD
 *
 *
 * @param[in] dev       device descriptor
 * @param[in] cb        callback called when the ORP OEM interrupt pin fires
 * @param[in] arg       callback argument
 *
 * @return @ref ORP_OEM_OK 		  		 	 on success
 * @return @ref ORP_OEM_WRITE_ERR 			 if writing to the device failed
 * @return @ref ORP_OEM_INTERRUPT_GPIO_UNDEF if the interrupt pin is undefined
 * @return @ref ORP_OEM_GPIO_INIT_ERR 		 if initializing the interrupt gpio pin failed
 */
int orp_oem_enable_interrupt(orp_oem_t *dev, orp_oem_interrupt_pin_cb_t cb,
                            void *arg);

/**
 * @brief   The interrupt pin will not auto reset on option @ref ORP_OEM_IRQ_RISING
 *          and @ref ORP_OEM_IRQ_FALLING after interrupt fires,
 *          so call this function again to reset the pin state.
 *
 * @note    The interrupt settings are not retained if the power is cut,
 *          so you have to call this function again after powering on the device.
 *
 * @param[in] dev    device descriptor
 *
 * @return @ref ORP_OEM_OK on success
 * @return @ref ORP_OEM_WRITE_ERR if writing to the device failed
 */
int orp_oem_reset_interrupt_pin(const orp_oem_t *dev);

/**
 * @brief   Set the LED state of the ORP OEM sensor by writing to the
 *          @ref ORP_OEM_REG_LED register
 *
 * @param[in] dev       		  device descriptor
 * @param[in] state     		  @ref orp_oem_led_state_t
 *
 * @return @ref ORP_OEM_OK 		  on success
 * @return @ref ORP_OEM_WRITE_ERR if writing to the device failed
 */
int orp_oem_set_led_state(const orp_oem_t *dev, orp_oem_led_state_t state);

/**
 * @brief   Sets the device state (active/hibernate) of the ORP OEM sensor by
 *          writing to the @ref ORP_OEM_REG_HIBERNATE register.
 *
 *          @note Once the device has been woken up it will continuously take
 *          readings every 420ms. Waking the device is the only way to take a
 *          reading. Hibernating the device is the only way to stop taking readings.
 *
 * @param[in] dev   		  	  device descriptor
 * @param[in] state @ref 		  orp_oem_device_state_t
 *
 * @return @ref ORP_OEM_OK 		  on success
 * @return @ref ORP_OEM_WRITE_ERR if writing to the device failed
 */
int orp_oem_set_device_state(const orp_oem_t *dev, orp_oem_device_state_t state);

/**
 * @brief   Starts a new reading by setting the device state to
 *          @ref ORP_OEM_TAKE_READINGS.
 *
 * @note    If the @ref orp_oem_params_t.interrupt_pin is @ref GPIO_UNDEF
 *          this function will poll every 20ms till a reading is done (~420ms)
 *          and stop the device from taking further readings
 *
 * @param[in] dev   				device descriptor
 *
 * @return @ref ORP_OEM_OK 			on success
 * @return @ref ORP_OEM_WRITE_ERR   if writing to the device failed
 * @return @ref ORP_OEM_READ_ERR    if reading from the device failed
 */
int orp_oem_start_new_reading(const orp_oem_t *dev);

/**
 * @brief   Clears all calibrations previously done
 *
 * @param[in] dev  				  device descriptor
 *
 * @return @ref ORP_OEM_OK 		  on success
 * @return @ref ORP_OEM_WRITE_ERR if writing to the device failed
 * @return @ref ORP_OEM_READ_ERR  if reading from the device failed
 */
int orp_oem_clear_calibration(const orp_oem_t *dev);

/**
 * @brief   Sets the @ref ORP_OEM_REG_CALIBRATION_BASE register to the
 *          calibration_value which the ORP OEM sensor will be
 *          calibrated to. Multiply the floating point calibration value of your
 *          solution by 10 e.g. ORP calibration solution => 209.4 * 10 = 2094 = 0x0000082E
 *
 *          The calibration value will be saved based on the given
 *          @ref orp_oem_calibration_option_t and retained after the power is cut.
 *
 * @param[in] dev                 device descriptor
 * @param[in] calibration_value   orp value multiplied by 10 e.g 209.4 * 10 = 2094
 * @param[in] option              @ref orp_oem_calibration_option_t
 *
 * @return @ref ORP_OEM_OK		  on success
 * @return @ref ORP_OEM_WRITE_ERR if writing to the device failed
 * @return @ref ORP_OEM_READ_ERR  if reading from the device failed
 */
int orp_oem_set_calibration(const orp_oem_t *dev, int16_t calibration_value,
                           orp_oem_calibration_option_t option);

/**
 * @brief   Read the @ref ORP_OEM_REG_CALIBRATION_CONFIRM register.
 *          After a calibration event has been successfully carried out, the
 *          calibration confirmation register will by setting bit 1.
 *
 * @param[in]  dev                 device descriptor
 * @param[out] calibration_state   calibration state reflected 1.
 *
 * @return @ref ORP_OEM_OK 		   on success
 * @return @ref ORP_OEM_READ_ERR   if reading from the device failed
 */
int orp_oem_read_calibration_state(const orp_oem_t *dev, uint16_t *calibration_state);

/**
 * @brief   Reads the @ref ORP_OEM_REG_ORP_READING_BASE register to get the current
 *          ORP reading.
 *
 * @param[in]  dev       		 device descriptor
 * @param[out] orp_value 		 raw ORP value <br>
 *                       		 divide by 10 for floating point <br>
 *                       		 e.g 834 / 10 = 83.4
 *
 * @return @ref ORP_OEM_OK 		 on success
 * @return @ref ORP_OEM_READ_ERR if reading from the device failed
 */
int orp_oem_read_orp(const orp_oem_t *dev, int16_t *orp_value);

#ifdef __cplusplus
}
#endif

#endif /* ORP_OEM_H */
/** @} */
