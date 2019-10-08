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
 *
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 * @author      Ting Xu <timtsui@outlook.com>
 */

#ifndef ORP_OEM_H
#define ORP_OEM_H

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
    ORP_OEM_CAL_CLEAR           = 0x01, /**< Clear calibration */
    ORP_OEM_CAL_SINGLE_POINT    = 0x02, /**< Single point calibration */
} orp_oem_calibration_option_t;

/**
 * @brief   ORP OEM device descriptor
 */
typedef struct orp_oem {
    oem_common_dev_t oem_dev;  /**< common OEM device driver configuration */
} orp_oem_t;

/**
 * @brief   Initialize a ORP OEM sensor
 *
 * @param[in,out]   dev         device descriptor
 * @param[in]       params      device configuration
 *
 * @return @ref ORP_OEM_OK      on success
 * @return @ref ORP_OEM_NODEV   if no device is found on the bus
 * @return @ref ORP_OEM_NOT_ORP if the device found at the address is not a ORP OEM device
 * @return
 */
int orp_oem_init(orp_oem_t *dev, const oem_common_params_t *params);

/**
 * @brief   Clears all calibrations previously done
 *
 * @param[in] dev                 device descriptor
 *
 * @return @ref ORP_OEM_OK        on success
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
 * @return @ref ORP_OEM_OK         on success
 * @return @ref ORP_OEM_READ_ERR   if reading from the device failed
 */
int orp_oem_read_calibration_state(const orp_oem_t *dev,
                                   uint8_t *calibration_state);

/**
 * @brief   Reads the @ref ORP_OEM_REG_ORP_READING_BASE register to get the current
 *          ORP reading.
 *
 * @param[in]  dev               device descriptor
 * @param[out] orp_value         raw ORP value <br>
 *                               divide by 10 for floating point <br>
 *                               e.g 834 / 10 = 83.4
 *
 * @return @ref ORP_OEM_OK       on success
 * @return @ref ORP_OEM_READ_ERR if reading from the device failed
 */
int orp_oem_read_orp(const orp_oem_t *dev, int32_t *orp_value);

#ifdef __cplusplus
}
#endif

#endif /* ORP_OEM_H */
/** @} */
