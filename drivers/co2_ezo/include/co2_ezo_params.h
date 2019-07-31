/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_co2_ezo
 * @{
 *
 * @file
 * @brief       Default configuration for Atlas Scientific CO2 EZO sensors

 * @author      Ting XU <timtsui@outlook.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 */

#ifndef CO2_EZO_PARAMS_H
#define CO2_EZO_PARAMS_H

#include "board.h" /* THIS INCLUDE IS MANDATORY */
#include "saul_reg.h"
#include "co2_ezo.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Set default configuration parameters for the Atlas Scientific CO2 EZO driver
 * @{
 */
#ifndef CO2_EZO_PARAM_I2C
#define CO2_EZO_PARAM_I2C                  (I2C_DEV(0))
#endif
#ifndef CO2_EZO_PARAM_ADDR
#define CO2_EZO_PARAM_ADDR                 (0x69)
#endif
#ifndef CO2_EZO_PARAM_ALARM_INT_PIN
#define CO2_EZO_PARAM_ALARM_INT_PIN        (GPIO_UNDEF)
#endif

#ifndef CO2_EZO_PARAMS
#define CO2_EZO_PARAMS       { .i2c = CO2_EZO_PARAM_I2C,   \
                               .addr = CO2_EZO_PARAM_ADDR, \
                               .alarm_int_pin = CO2_EZO_PARAM_ALARM_INT_PIN   }
#endif
#ifndef CO2_EZO_SAUL_INFO
#define CO2_EZO_SAUL_INFO       { .name = "CO2 EZO sensor" }
#endif
/** @} */
/**
 * @brief   CO2 EZO defaults if not defined for a board or application
 */
static const co2_ezo_params_t co2_ezo_params[] =
{
    CO2_EZO_PARAMS
};

/**
 * @brief   Additional meta information to keep in the SAUL registry
 */
static const saul_reg_info_t co2_ezo_saul_info[] =
{
    CO2_EZO_SAUL_INFO
};

#ifdef __cplusplus
}
#endif

#endif /* CO2_EZO_PARAMS_H */
/** @} */
