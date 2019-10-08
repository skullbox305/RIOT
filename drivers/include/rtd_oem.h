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

 * The Atlas Scientific RTD OEM sensor can be used with or without the interrupt
 * pin. Per default this pin is mapped to @ref GPIO_UNDEF if not otherwise defined
 * in your makefile.
 *
 * Once the RTD OEM is powered on it will be ready to receive commands and take
 * readings after 1ms.
 *
 * @note This driver provides @ref drivers_saul capabilities.
 * Reading (@ref saul_driver_t.read) from the device returns the current temperature
 * value.
 *
 * @note Communication is done using SMBus/I2C protocol at speeds
 * of 10-100 kHz. Set your board I2C speed to @ref I2C_SPEED_LOW or
 * @ref I2C_SPEED_NORMAL
 *
 * @{
 *
 * @file
 * @brief       Device driver for Atlas Scientific RTD OEM Sensor with SMBus/I2C interface
 *
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 * @author      Ting XU <timtsui@outlook.com>
 */

#ifndef RTD_OEM_H
#define RTD_OEM_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "oem_common.h"
#include "periph/i2c.h"
#include "periph/gpio.h"

/**
 * @brief   Calibration option values
 */
typedef enum {
    RTD_OEM_CAL_CLEAR         = 0x01, /**< Clear calibration */
    RTD_OEM_CAL_SINGLE_POINT  = 0x02, /**< Single point calibration */
} rtd_oem_calibration_option_t;

/**
 * @brief   RTD OEM device descriptor
 */
typedef struct rtd_oem {
    oem_common_dev_t oem_dev;  /**< common OEM device driver configuration */
} rtd_oem_t;

/**
 * @brief   Initialize a RTD OEM sensor
 *
 * @param[in,out]   dev      device descriptor
 * @param[in]       params   device configuration
 *
 * @return @ref RTD_OEM_OK      on success
 * @return @ref RTD_OEM_NODEV   if no device is found on the bus
 * @return @ref RTD_OEM_NOT_RTD if the device found at the address is not a RTD OEM device
 * @return
 */
int rtd_oem_init(rtd_oem_t *dev, const oem_common_params_t *params);

/**
 * @brief   Clears the previous calibration
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
 *          been done, by setting bits 0 - 1.
 *
 * @param[in]  dev                 device descriptor
 * @param[out] calibration_state   calibration state reflected by bits 0 - 1 <br>
 *                                 (0 = no calibration, 1 = calibrated)
 *
 * @return @ref RTD_OEM_OK on success
 * @return @ref RTD_OEM_READ_ERR if reading from the device failed
 */
int rtd_oem_read_calibration_state(const rtd_oem_t *dev,
                                   uint8_t *calibration_state);

/**
 * @brief   Sets the @ref RTD_OEM_REG_CALIBRATION_BASE register to the
 *          @p calibration_value which the RTD OEM sensor will be
 *          calibrated to. Multiply the floating point calibration value by 1000
 *          e.g. 50,5°C * 1000 = 50500 = 0x0000C544
 *          The calibration value will be saved based on the given
 *          @ref rtd_oem_calibration_option_t and retained after the power is cut.
 *
 * @param[in] dev                 device descriptor
 * @param[in] calibration_value   calibration temperature value
 * @param[in] option              @ref rtd_oem_calibration_option_t
 *
 * @return @ref RTD_OEM_OK on success
 * @return @ref RTD_OEM_WRITE_ERR if writing to the device failed
 * @return @ref RTD_OEM_READ_ERR if reading from the device failed
 */
int rtd_oem_set_calibration(const rtd_oem_t *dev, uint32_t calibration_value,
                            rtd_oem_calibration_option_t option);

/**
 * @brief   Reads the @ref RTD_OEM_REG_RTD_READING_BASE register to get the current
 *          temperature reading.
 *
 * @param[in]  dev        device descriptor
 * @param[out] rtd_value  raw temperature value <br>
 *                        divide by 1000 for floating point <br>
 *                        e.g 25761 / 1000 = 25.761<br>
 *                        Range: -126000 – 1254000 (-126.000 °C – 1254 °C)
 *
 * @return @ref RTD_OEM_OK on success
 * @return @ref RTD_OEM_READ_ERR if reading from the device failed
 */
int rtd_oem_read_temp(const rtd_oem_t *dev, int32_t *temp_val);

#ifdef __cplusplus
}
#endif

#endif /* RTD_OEM_H */
/** @} */
