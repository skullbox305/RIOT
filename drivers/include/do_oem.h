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

 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 * @author      Ting XU <timtsui@outlook.com>
 */

#ifndef DO_OEM_H
#define DO_OEM_H

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
    DO_OEM_CALI_CLEAR          = 0x01, /**< Clear calibration */
    DO_OEM_CALI_ATMOSPHERIC    = 0x02, /**< Calibrate to atmospheric oxygen content */
    DO_OEM_CALI_0_DISSOLVED    = 0x03, /**< Calibrate to 0 dissolved oxygen */
} do_oem_calibration_option_t;

/**
 * @brief   DO OEM device descriptor
 */
typedef struct do_oem {
    oem_common_dev_t oem_dev;  /**< common OEM device driver configuration */
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
int do_oem_init(do_oem_t *dev, const oem_common_params_t *params);

/**
 * @brief   Clears all calibrations previously done
 *
 * @param[in] dev   device descriptor
 *
 * @return @ref DO_OEM_OK         on success
 * @return @ref DO_OEM_WRITE_ERR  if writing to the device failed
 * @return @ref DO_OEM_READ_ERR	  if reading from the device failed
 */
int do_oem_clear_calibration(const do_oem_t *dev);

/**
 * @brief   Read the @ref DO_OEM_REG_CALIBRATION_CONFIRM register.
 *          After a calibration event has been successfully carried out, the
 *          calibration confirmation register will reflect what calibration has
 *          been done, by setting bits 0 - 1.
 *
 * @param[in]  dev                 device descriptor
 * @param[out] calibration_state   calibration state reflected by the bits 0 - 1 <br>
 *                                 (0 = no calibration,
 *                                  1 = calibrated to atmosphere,
 *                                  2 = calibrated to 0 Dissolved Oxygen,
 *                                  3 = calibrated to both atmospheric and 0 dissolved Oxygen)
 *
 * @return @ref DO_OEM_OK       on success
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
 * @return @ref DO_OEM_OK         on success
 * @return @ref DO_OEM_WRITE_ERR  if writing to the device failed
 * @return @ref DO_OEM_READ_ERR   if reading from the device failed
 */
int do_oem_set_calibration(const do_oem_t *dev,
                           do_oem_calibration_option_t option);

/**
 * @brief   Sets the @ref DO_OEM_REG_SALI_COMPENSATION_BASE register to the
 *          @p salinity_compensation value which the DO OEM sensor will use
 *          to compensate the reading error.
 *          Multiply the floating point temperature value by 100
 *          e.g. salinity in microsiemens = 56000μS/cm * 100 = 5600000
 *
 * @note   The salinity compensation will not be retained if the power is cut.
 *
 * @param[in] dev                    device descriptor
 * @param[in] sali_compensation      valid salinity range is
 *                                   (0.00 μS/cm - 65625.00 μS/cm)
 *
 * @return @ref DO_OEM_OK                on success
 * @return @ref DO_OEM_WRITE_ERR         if writing to the device failed
 * @return @ref DO_OEM_SALI_OUT_OF_RANGE if the salinity_compensation is not in
 *                                       the valid range
 */
int do_oem_set_sal_compensation(const do_oem_t *dev, uint32_t sali_compensation);

/**
 * @brief   Sets the @ref DO_OEM_REG_PRES_COMPENSATION_BASE register to the
 *          pressure_compensation value which the DO OEM sensor will use
 *          to compensate the reading error.
 *          Multiply the floating point pressure value by 100
 *          e.g. pressure in kilopascals = 34.26 kPa * 100 = 3426
 *
 *  @note   The pressure compensation will not be retained if the power is cut.
 *
 * @param[in] dev                        device descriptor
 * @param[in] pressure_compensation      valid pressure range is
 *                                       (30.00 kPa - 110.00 kPa)
 *
 * @return @ref DO_OEM_OK on success
 * @return @ref DO_OEM_WRITE_ERR         if writing to the device failed
 * @return @ref DO_OEM_PRES_OUT_OF_RANGE if the pressure_compensation is not in
 *                                       the valid range
 */
int do_oem_set_pres_compensation(const do_oem_t *dev,
                                 uint16_t pressure_compensation);

/**
 * @brief   Sets the @ref DO_OEM_REG_TEMP_COMPENSATION_BASE register to the
 *          temperature_compensation value which the DO OEM sensor will use
 *          to compensate the reading error.
 *          Multiply the floating point temperature value by 100
 *          e.g. temperature in degree Celsius = 13.78 ℃ * 100 = 1378
 *
 *  @note   The temperature compensation will not be retained if the power is cut.
 *
 * @param[in] dev                        device descriptor
 * @param[in] temperature_compensation   valid temperature range is
 *                                       (0.01 ℃ - 200.00 ℃)
 *
 * @return @ref DO_OEM_OK                on success
 * @return @ref DO_OEM_WRITE_ERR         if writing to the device failed
 * @return @ref DO_OEM_TEMP_OUT_OF_RANGE if the temperature_compensation is not in
 *                                       the valid range
 */
int do_oem_set_temp_compensation(const do_oem_t *dev,
                                 uint16_t temperature_compensation);

/**
 * @brief   Reads the @ref DO_OEM_REG_SALI_COMFIRMATION_BASE register to verify
 *          the salinity compensation that was used to take the reading
 *          is set to the correct salinity value.
 *
 * @param[in]  dev                       device descriptor
 * @param[out] salinity_compensation     raw salinity compensation value. <br>
 *                                       Divide by 100 for floating point <br>
 *                                       e.g 5600000 / 100 = 56000 μS/cm
 *
 * @return @ref DO_OEM_OK           on success
 * @return @ref DO_OEM_READ_ERR     if reading from the device failed
 */
int do_oem_read_sali_compensation(const do_oem_t *dev,
                                  uint32_t *salinity_compensation);

/**
 * @brief   Reads the @ref DO_OEM_REG_PRES_COMFIRMATION_BASE register to verify
 *          the pressure compensation that was used to take the reading
 *          is set to the correct pressure value.
 *
 * @param[in]  dev                       device descriptor
 * @param[out] pressure_compensation     raw pressure compensation value. <br>
 *                                       Divide by 100 for floating point <br>
 *                                       e.g 3426 / 100 = 34.26 kPa
 *
 * @return @ref DO_OEM_OK                on success
 * @return @ref DO_OEM_READ_ERR          if reading from the device failed
 */
int do_oem_read_pres_compensation(const do_oem_t *dev,
                                  uint32_t *pressure_compensation);

/**
 * @brief   Reads the @ref DO_OEM_REG_TEMP_COMFIRMATION_BASE register to verify
 *          the temperature compensation value that was used to take the reading
 *          is set to the correct temperature.
 *
 * @param[in]  dev                       device descriptor
 * @param[out] temperature_compensation  raw temperature compensation value. <br>
 *                                       Divide by 100 for floating point <br>
 *                                       e.g 3426 / 100 = 34.26
 *
 * @return @ref DO_OEM_OK                on success
 * @return @ref DO_OEM_READ_ERR          if reading from the device failed
 */
int do_oem_read_temp_compensation(const do_oem_t *dev,
                                  uint32_t *temperature_compensation);

/**
 * @brief   Reads the @ref DO_OEM_REG_DO_MGL_READING_BASE register to get the current
 *          D.O. reading.
 *
 * @param[in]  dev                  device descriptor
 * @param[out] do_mg_value          raw do value in mg/L <br>
 *                                  divide by 100 for floating point <br>
 *                                  e.g 834 / 100 = 8.34
 *
 * @return @ref DO_OEM_OK           on success
 * @return @ref DO_OEM_READ_ERR     if reading from the device failed
 */
int do_oem_read_do_mg(const do_oem_t *dev, uint32_t *do_mg_value);

/**
 * @brief   Reads the @ref DO_OEM_REG_DO_PERCENT_READING_BASE register to get the current
 *          pressure reading.
 *
 * @param[in]  dev                  device descriptor
 * @param[out] do_percent_value     raw do value in % saturation <br>
 *                                  divide by 100 for floating point <br>
 *                                  e.g 834 / 100 = 8.34
 *
 * @return @ref DO_OEM_OK           on success
 * @return @ref DO_OEM_READ_ERR     if reading from the device failed
 */
int do_oem_read_do_percent(const do_oem_t *dev, uint32_t *do_percent_value);

#ifdef __cplusplus
}
#endif

#endif /* DO_OEM_H */
/** @} */
