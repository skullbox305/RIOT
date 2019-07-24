/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_co2_oem CO2 EZO sensor device driver
 * @ingroup     drivers_sensors
 * @ingroup     drivers_saul
 * @brief       Device driver for Atlas Scientific CO2 EZO sensor with SMBus/I2C interface
 *
 * The Atlas Scientific CO2 EZO sensor can be used with or without the interrupt
 * pin. Per default this pin is mapped to @ref GPIO_UNDEF if not otherwise defined
 * in your makefile.
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
    CO2_EZO_OK                   =  0,   /**< Everything was fine */
    CO2_EZO_NODEV                = -1,   /**< No device found on the bus */
    CO2_EZO_READ_ERR             = -2,   /**< Reading to device failed*/
    CO2_EZO_WRITE_ERR            = -3,   /**< Writing to device failed */
    CO2_EZO_NOT_CO2              = -4,   /**< Not an Atlas Scientific CO2 OEM device */
} co2_ezo_named_returns_t;


/**
 * @brief   CO2 OEM sensor params
 */
typedef struct co2_ezo_params {
    i2c_t i2c;                      /**< I2C device the sensor is connected to */
    uint8_t addr;                   /**< the slave address of the sensor on the I2C bus */
} co2_ezo_params_t;

/**
 * @brief   pH OEM device descriptor
 */
typedef struct co2_ezo {
    co2_ezo_params_t params;         /**< device driver configuration */
} co2_ezo_t;

/**
 * @brief   Initialize a CO2 OEM sensor
 *
 * @param[in,out]   dev      device descriptor
 * @param[in]       params   device configuration
 *
 * @return @ref CO2_EZO_OK on success
 * @return @ref CO2_EZO_NODEV if no device is found on the bus
 * @return @ref CO2_EZO_NOT_CO2 if the device found at the address is not a CO2 EZO device
 * @return
 */
int co2_ezo_init(co2_ezo_t *dev, const co2_ezo_params_t *params);


#ifdef __cplusplus
}
#endif

#endif /* CO2_EZO_H */
/** @} */
