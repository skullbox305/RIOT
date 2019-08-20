/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_rgb_ezo
 * @{
 *
 * @file
 * @brief       Default configuration for Atlas Scientific RGB EZO sensors

 * @author      Ting XU <timtsui@outlook.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 */

#ifndef RGB_EZO_PARAMS_H
#define RGB_EZO_PARAMS_H

#include "board.h" /* THIS INCLUDE IS MANDATORY */
#include "saul_reg.h"
#include "rgb_ezo.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Set default configuration parameters for the Atlas Scientific RGB EZO driver
 * @{
 */
#ifndef RGB_EZO_PARAM_I2C
#define RGB_EZO_PARAM_I2C                  (I2C_DEV(0))
#endif
#ifndef RGB_EZO_PARAM_ADDR
#define RGB_EZO_PARAM_ADDR                 (0x70)
#endif
#ifndef RGB_EZO_PARAM_ALARM_INT_PIN
#define RGB_EZO_PARAM_ALARM_INT_PIN        (GPIO_UNDEF)
#endif

#ifndef RGB_EZO_PARAMS
#define RGB_EZO_PARAMS       { .i2c = RGB_EZO_PARAM_I2C,   \
                               .addr = RGB_EZO_PARAM_ADDR, \  }
#endif
#ifndef RGB_EZO_SAUL_INFO
#define RGB_EZO_SAUL_INFO       { .name = "RGB EZO sensor" }
#endif
/** @} */
/**
 * @brief   RGB EZO defaults if not defined for a board or application
 */
static const rgb_ezo_params_t rgb_ezo_params[] =
{
    RGB_EZO_PARAMS
};

/**
 * @brief   Additional meta information to keep in the SAUL registry
 */
static const saul_reg_info_t rgb_ezo_saul_info[] =
{
    RGB_EZO_SAUL_INFO
};

#ifdef __cplusplus
}
#endif

#endif /* RGB_EZO_PARAMS_H */
/** @} */
