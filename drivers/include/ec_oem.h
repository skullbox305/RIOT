/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_ec_oem EC OEM sensor device driver
 * @ingroup     drivers_sensors
 * @ingroup     drivers_saul
 * @brief       Device driver for Atlas Scientific EC OEM sensor with SMBus/I2C interface
*
 * The Atlas Scientific EC OEM sensor can be used with or without the interrupt
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
 * Once the EC OEM is powered on it will be ready to receive commands and take
 * readings after 1ms.
 *
 * @note This driver provides @ref drivers_saul capabilities.
 * Reading (@ref saul_driver_t.read) from the device returns the current EC,TDS
 * and PSS value.
 * Writing (@ref saul_driver_t.write) a temperature value in celsius to the
 * device sets the temperature compensation. A valid temperature range is
 * 1 - 20000 (0.01 °C  to  200.0 °C)
 *
 * @note Communication is done using SMBus/I2C protocol at speeds
 * of 10-100 kHz. Set your board I2C speed to @ref I2C_SPEED_LOW or
 * @ref I2C_SPEED_NORMAL
 *
 * @{
 *
 * @file
 * @brief       Device driver for Atlas Scientific EC OEM Sensor with SMBus/I2C interface


 * @author      Ting Xu <timtsui@outlook.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 */

#ifndef EC_OEM_H
#define EC_OEM_H

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
    EC_OEM_OK                   =  0,   /**< Everything was fine */
    EC_OEM_NODEV                = -1,   /**< No device found on the bus */
    EC_OEM_READ_ERR             = -2,   /**< Reading to device failed*/
    EC_OEM_WRITE_ERR            = -3,   /**< Writing to device failed */
    EC_OEM_NOT_EC               = -4,   /**< Not an Atlas Scientific EC OEM device */
    EC_OEM_INTERRUPT_GPIO_UNDEF = -5,   /**< Interrupt pin is @ref GPIO_UNDEF */
    EC_OEM_GPIO_INIT_ERR        = -6,   /**< Error while initializing GPIO PIN */
    EC_OEM_TEMP_OUT_OF_RANGE    = -7    /**< Temperature is out of range */
} ec_oem_named_returns_t;

/**
 * @brief   LED state values
 */
typedef enum {
    EC_OEM_LED_ON   = 0x01, /**< LED on state */
    EC_OEM_LED_OFF  = 0x00, /**< LED off state */
} ec_oem_led_state_t;

/**
 * @brief   Device state values
 */
typedef enum {
    EC_OEM_TAKE_READINGS    = 0x01, /**< Device active state */
    EC_OEM_STOP_READINGS    = 0x00, /**< Device hibernate state */
} ec_oem_device_state_t;
/**
 * @brief   Interrupt pin option values
 */
typedef enum {
    EC_OEM_IRQ_RISING   = 0x02, /**< Pin high on new reading (manually reset) */
    EC_OEM_IRQ_FALLING  = 0x04, /**< Pin low on new reading (manually reset) */
    EC_OEM_IRQ_BOTH     = 0x08, /**< Invert state on new reading (automatically reset) */
} ec_oem_irq_option_t;

/**
 * @brief   Calibration option values
 */
typedef enum {
    EC_OEM_CALIBRATE_DRY 			 = 0x02,     /**< Dry point calibration option */
    EC_OEM_CALIBRATE_SINGLE_POINT 	 = 0x03,     /**< Single point calibration option */
    EC_OEM_CALIBRATE_DUAL_LOW		 = 0x04,     /**< Dual point calibration low option */
	EC_OEM_CALIBRATE_DUAL_HIGH		 = 0x05,	 /**< Dual point calibration high option */
} ec_oem_calibration_option_t;

/**
 * @brief   EC OEM sensor params
 */
typedef struct ec_oem_params {
    i2c_t i2c;                      /**< I2C device the sensor is connected to */
    uint8_t addr;                   /**< the slave address of the sensor on the I2C bus */
    gpio_t interrupt_pin;           /**< interrupt pin (@ref GPIO_UNDEF if not defined) */
    gpio_mode_t gpio_mode;          /**< gpio mode of the interrupt pin */
    ec_oem_irq_option_t irq_option; /**< behavior of the interrupt pin, disabled by default */
} ec_oem_params_t;

/**
 * @brief   EC OEM interrupt pin callback
 */
typedef void (*ec_oem_interrupt_pin_cb_t)(void *);

/**
 * @brief   EC OEM device descriptor
 */
typedef struct ec_oem {
    ec_oem_params_t params;         /**< device driver configuration */
    ec_oem_interrupt_pin_cb_t cb;   /**< interrupt pin callback */
    void *arg;                      /**< interrupt pin callback param */
} ec_oem_t;

/**
 * @brief   Initialize a EC OEM sensor
 *
 * @param[in,out]   dev      device descriptor
 * @param[in]       params   device configuration
 *
 * @return @ref EC_OEM_OK on success
 * @return @ref EC_OEM_NODEV if no device is found on the bus
 * @return @ref EC_OEM_NOT_EC if the device found at the address is not a EC OEM
 * device
 * @return
 */
int ec_oem_init(ec_oem_t *dev, const ec_oem_params_t *params);

/**
 * @brief   Sets the probe type to the EC OEM device by writing the probe type to
 *          the @ref EC_OEM_REG_SET_PROBE_TYPE register. Example: K 0.01 - K 600
 *
 *          Settings are retained in the sensor if the power is cut.
 *
 * @param[in] dev         device descriptor
 * @param[in] probe_type  probe type for the device. Range: 0,01-600
 *
 * @return @ref EC_OEM_OK on success
 * @return @ref EC_OEM_WRITE_ERR if writing to the device failed
 */
int ec_oem_set_probe_type (ec_oem_t *dev, uint16_t probe_type);

/**
 * @brief   Sets a new address to the EC OEM device by unlocking the
 *          @ref EC_OEM_REG_UNLOCK register and  writing a new address to
 *          the @ref EC_OEM_REG_ADDRESS register.
 *          The device address will also be updated in the device descriptor so
 *          it is still usable.
 *
 *          Settings are retained in the sensor if the power is cut.
 *
 *          The address in the device descriptor will reverse to the default
 *          address you provided through EC_OEM_PARAM_ADDR after the
 *          microcontroller restarts
 *
 * @param[in] dev   device descriptor
 * @param[in] addr  new address for the device. Range: 0x01 - 0x7f
 *
 * @return @ref EC_OEM_OK on success
 * @return @ref EC_OEM_WRITE_ERR if writing to the device failed
 */
int ec_oem_set_i2c_address(ec_oem_t *dev, uint8_t addr);

/**
 * @brief   Enable the EC OEM interrupt pin if @ref ec_oem_params_t.interrupt_pin
 *          is defined.
 *          @note @ref ec_oem_reset_interrupt_pin needs to be called in the
 *          callback if you use @ref EC_OEM_IRQ_FALLING or @ref EC_OEM_IRQ_RISING
 *
 *          @note Provide the EC_OEM_PARAM_INTERRUPT_OPTION flag in your
 *          makefile. Valid options see: @ref ec_oem_irq_option_t.
 *          The default is @ref EC_OEM_IRQ_BOTH.
 *
 *          @note Also provide the @ref gpio_mode_t as a CFLAG in your makefile.
 *          Most likely @ref GPIO_IN. If the pin is to sensitive use
 *          @ref GPIO_IN_PU for @ref EC_OEM_IRQ_FALLING or
 *          @ref GPIO_IN_PD for @ref EC_OEM_IRQ_RISING and
 *          @ref EC_OEM_IRQ_BOTH. The default is @ref GPIO_IN_PD
 *
 *
 * @param[in] dev       device descriptor
 * @param[in] cb        callback called when the EC OEM interrupt pin fires
 * @param[in] arg       callback argument
 *
 * @return @ref EC_OEM_OK on success
 * @return @ref EC_OEM_WRITE_ERR if writing to the device failed
 * @return @ref EC_OEM_INTERRUPT_GPIO_UNDEF if the interrupt pin is undefined
 * @return @ref EC_OEM_GPIO_INIT_ERR if initializing the interrupt gpio pin failed
 */
int ec_oem_enable_interrupt(ec_oem_t *dev, ec_oem_interrupt_pin_cb_t cb,
                            void *arg);

/**
 * @brief   The interrupt pin will not auto reset on option @ref EC_OEM_IRQ_RISING
 *          and @ref EC_OEM_IRQ_FALLING after interrupt fires,
 *          so call this function again to reset the pin state.
 *
 * @note    The interrupt settings are not retained if the power is cut,
 *          so you have to call this function again after powering on the device.
 *
 * @param[in] dev    device descriptor
 *
 * @return @ref EC_OEM_OK on success
 * @return @ref EC_OEM_WRITE_ERR if writing to the device failed
 */
int ec_oem_reset_interrupt_pin(const ec_oem_t *dev);

/**
 * @brief   Set the LED state of the EC OEM sensor by writing to the
 *          @ref EC_OEM_REG_LED register
 *
 * @param[in] dev       device descriptor
 * @param[in] state     @ref ec_oem_led_state_t
 *
 * @return @ref EC_OEM_OK on success
 * @return @ref EC_OEM_WRITE_ERR if writing to the device failed
 */
int ec_oem_set_led_state(const ec_oem_t *dev, ec_oem_led_state_t state);

/**
 * @brief   Sets the device state (active/hibernate) of the EC OEM sensor by
 *          writing to the @ref EC_OEM_REG_HIBERNATE register.
 *
 *          @note Once the device has been woken up it will continuously take
 *          readings every 640ms. Waking the device is the only way to take a
 *          reading. Hibernating the device is the only way to stop taking readings.
 *
 * @param[in] dev   device descriptor
 * @param[in] state @ref ec_oem_device_state_t
 *
 * @return @ref EC_OEM_OK on success
 * @return @ref EC_OEM_WRITE_ERR if writing to the device failed
 */
int ec_oem_set_device_state(const ec_oem_t *dev, ec_oem_device_state_t state);

/**
 * @brief   Starts a new reading by setting the device state to
 *          @ref EC_OEM_TAKE_READINGS.
 *
 * @note    If the @ref ec_oem_params_t.interrupt_pin is @ref GPIO_UNDEF
 *          this function will poll every 20ms till a reading is done (~640ms)
 *          and stop the device from taking further readings
 *
 * @param[in] dev   device descriptor
 *
 * @return @ref EC_OEM_OK on success
 * @return @ref EC_OEM_WRITE_ERR if writing to the device failed
 * @return @ref EC_OEM_READ_ERR if reading from the device failed
 */
int ec_oem_start_new_reading(const ec_oem_t *dev);

/**
 * @brief   Clears all calibrations previously done
 *
 * @param[in] dev   device descriptor
 *
 * @return @ref EC_OEM_OK on success
 * @return @ref EC_OEM_WRITE_ERR if writing to the device failed
 * @return @ref EC_OEM_READ_ERR if reading from the device failed
 */
int ec_oem_clear_calibration(const ec_oem_t *dev);

/**
 * @brief   Sets the @ref EC_OEM_REG_CALIBRATION_BASE register to the
 *          calibration_value which the EC OEM sensor will be
 *          calibrated to. Multiply the floating point calibration value of your
 *          solution by 100 e.g. EC calibration solution => 150000,00 * 100 = 15000000 = 0x00E4E1C0
 *
 *          The calibration value will be saved based on the given
 *          @ref ec_oem_calibration_option_t and retained after the power is cut.
 *
 * @param[in] dev                 device descriptor
 * @param[in] calibration_value   EC value multiplied by 100 e.g 150000,00 * 100 = 15000000
 * @param[in] option              @ref ec_oem_calibration_option_t
 *
 * @return @ref EC_OEM_OK on success
 * @return @ref EC_OEM_WRITE_ERR if writing to the device failed
 * @return @ref EC_OEM_READ_ERR if reading from the device failed
 */
int ec_oem_set_calibration(const ec_oem_t *dev, uint32_t calibration_value,
                           ec_oem_calibration_option_t option);

/**
 * @brief   Read the @ref EC_OEM_REG_CALIBRATION_CONFIRM register.
 *          After a calibration event has been successfully carried out, the
 *          calibration confirmation register will reflect what calibration has
 *          been done, by setting bits 0 - 3.
 *
 * @param[in]  dev                 device descriptor
 * @param[out] calibration_state   calibration state reflected by bits 0 - 3 <br>
 *                                 (0 = dry, 1 = single, 2 = low , 3 = high)
 *
 * @return @ref EC_OEM_OK on success
 * @return @ref EC_OEM_READ_ERR if reading from the device failed
 */
int ec_oem_read_calibration_state(const ec_oem_t *dev, uint16_t *calibration_state);

/**
 * @brief   Sets the @ref EC_OEM_REG_TEMP_COMPENSATION_BASE register to the
 *          temperature_compensation value which the EC OEM sensor will use
 *          to compensate the reading error.
 *          Multiply the floating point temperature value by 100
 *          e.g. temperature in degree Celsius = 34.26 * 100 = 3426
 *
 *  @note   The temperature compensation will not be retained if the power is cut.
 *
 * @param[in] dev                        device descriptor
 * @param[in] temperature_compensation   valid temperature range is
 *                                       1 - 20000 (0.01 °C  to  200.0 °C)
 *
 * @return @ref EC_OEM_OK on success
 * @return @ref EC_OEM_WRITE_ERR if writing to the device failed
 * @return @ref EC_OEM_TEMP_OUT_OF_RANGE if the temperature_compensation is not in
 *                                       the valid range
 */
int ec_oem_set_compensation(const ec_oem_t *dev,
                            uint16_t temperature_compensation);

/**
 * @brief   Reads the @ref EC_OEM_REG_TEMP_CONFIRMATION_BASE register to verify
 *          the temperature compensation value that was used to take the EC
 *          reading is set to the correct temperature.
 *
 * @param[in]  dev                       device descriptor
 * @param[out] temperature_compensation  raw temperature compensation value. <br>
 *                                       Divide by 100 for floating point <br>
 *                                       e.g 3426 / 100 = 34.26
 *
 * @return @ref EC_OEM_OK on success
 * @return @ref EC_OEM_READ_ERR if reading from the device failed
 */
int ec_oem_read_compensation(const ec_oem_t *dev,
                             uint16_t *temperature_compensation);

/**
 * @brief   Reads the @ref EC_OEM_REG_EC_READING_BASE register to get the current
 *          EC reading.
 *
 * @param[in]  dev        device descriptor
 * @param[out] ec_value   raw EC value <br>
 *                        divide by 100 for floating point <br>
 *                        e.g 7,00 / 100 = 0,07
 *
 * @return @ref EC_OEM_OK on success
 * @return @ref EC_OEM_READ_ERR if reading from the device failed
 */
int ec_oem_read_ec(const ec_oem_t *dev, uint16_t *ec_value);

/**
 * @brief   Reads the @ref EC_OEM_REG_TDS_READING_BASE register to get the current
 *          TDS reading.
 *
 * @param[in]  dev        device descriptor
 * @param[out] tds_value  raw TDS value <br>
 *                        divide by 100 for floating point <br>
 *                        e.g 2250,00 / 100 = 22,5
 *
 * @return @ref EC_OEM_OK on success
 * @return @ref EC_OEM_READ_ERR if reading from the device failed
 */
int ec_oem_read_tds(const ec_oem_t *dev, uint16_t *tds_value);

/**
 * @brief   Reads the @ref EC_OEM_REG_PSS_READING_BASE register to get the current
 *          PSS reading.
 *
 * @param[in]  dev        device descriptor
 * @param[out] tds_value  raw PSS value <br>
 *                        divide by 100 for floating point <br>
 *                        e.g 730 / 100 = 7,30
 *
 * @return @ref EC_OEM_OK on success
 * @return @ref EC_OEM_READ_ERR if reading from the device failed
 */
int ec_oem_read_pss(const ec_oem_t *dev, uint16_t *pss_value);

#ifdef __cplusplus
}
#endif

#endif /* EC_OEM_H */
/** @} */
