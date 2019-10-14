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


 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 * @author      Ting Xu <timtsui@outlook.com>
 */

#ifndef EC_OEM_H
#define EC_OEM_H

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
	EC_OEM_CAL_CLEAR 	     = 0x01,     /**< Clear calibration */
	EC_OEM_CAL_DRY 			 = 0x02,     /**< Dry point calibration option */
    EC_OEM_CAL_SINGLE_POINT  = 0x03,     /**< Single point calibration option */
    EC_OEM_CAL_DUAL_LOW		 = 0x04,     /**< Dual point calibration low option */
	EC_OEM_CAL_DUAL_HIGH     = 0x05,	 /**< Dual point calibration high option */
} ec_oem_calibration_option_t;

/**
 * @brief   EC OEM device descriptor
 */
typedef struct ec_oem {
    oem_common_dev_t oem_dev;  /**< common OEM device driver configuration */
} ec_oem_t;

/**
 * @brief   Initialize a EC OEM sensor
 *
 * @param[in,out]   dev      device descriptor
 * @param[in]       params   device configuration
 *
 * @return @ref EC_OEM_OK     on success
 * @return @ref EC_OEM_NODEV  if no device is found on the bus
 * @return @ref EC_OEM_NOT_EC if the device found at the address is not a EC OEM
 * device
 * @return
 */
int ec_oem_init(ec_oem_t *dev, const oem_common_params_t *params);

/**
 * @brief   Sets the probe type connected to the EC OEM device, by writing
 *          @p probe_type to the @ref EC_OEM_REG_SET_PROBE_TYPE register.
 *          Supported probes: K 0.01 - K 600 any brand.
 *          Multiply floating point by 100, e.g. K9.8 * 100 = 980
 *
 *          Settings are retained in the sensor if the power is cut.
 *
 * @param[in] dev         device descriptor
 * @param[in] probe_type  probe type for the device. Range: K 0,01-600
 *
 * @return @ref EC_OEM_OK on success
 * @return @ref EC_OEM_WRITE_ERR if writing to the device failed
 */
int ec_oem_set_probe_type (ec_oem_t *dev, uint16_t probe_type);

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
int ec_oem_read_calibration_state(const ec_oem_t *dev, uint8_t *calibration_state);

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
int ec_oem_set_compensation(const ec_oem_t *dev, uint16_t temp_compensation);

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
int ec_oem_read_compensation(const ec_oem_t *dev, uint32_t *temp_compensation);

/**
 * @brief   Reads the @ref EC_OEM_REG_EC_READING_BASE register to get the current
 *          EC reading (electrical conductivity - Unit: μS/cm).
 *
 * @param[in]  dev        device descriptor
 * @param[out] ec_value   raw EC value <br>
 *                        divide by 100 for floating point <br>
 *                        e.g 7,00 / 100 = 0,07
 *
 * @return @ref EC_OEM_OK on success
 * @return @ref EC_OEM_READ_ERR if reading from the device failed
 */
int ec_oem_read_ec(const ec_oem_t *dev, uint32_t *ec_value);

/**
 * @brief   Reads the @ref EC_OEM_REG_TDS_READING_BASE register to get the current
 *          TDS reading (Total dissolved solids. Unit: parts per million: ppm).
 *
 * @param[in]  dev        device descriptor
 * @param[out] tds_value  raw TDS value <br>
 *                        divide by 100 for floating point <br>
 *                        e.g 2250,00 / 100 = 22,5
 *
 * @return @ref EC_OEM_OK on success
 * @return @ref EC_OEM_READ_ERR if reading from the device failed
 */
int ec_oem_read_tds(const ec_oem_t *dev, uint32_t *tds_value);

/**
 * @brief   Reads the @ref EC_OEM_REG_PSS_READING_BASE register to get the current
 *          PSS reading (Practical Salinity Scale. Unit: parts per thousand - ppt).
 *
 * @param[in]  dev        device descriptor
 * @param[out] pss_value  raw PSS value <br>
 *                        divide by 100 for floating point <br>
 *                        e.g 730 / 100 = 7,30
 *
 * @return @ref EC_OEM_OK on success
 * @return @ref EC_OEM_READ_ERR if reading from the device failed
 */
int ec_oem_read_pss(const ec_oem_t *dev, uint32_t *pss_value);

#ifdef __cplusplus
}
#endif

#endif /* EC_OEM_H */
/** @} */
