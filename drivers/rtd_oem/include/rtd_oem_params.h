/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_rtd_oem
 * @{
 *
 * @file
 * @brief       Default configuration for Atlas Scientific RTD OEM sensors
 *
 * @author      Ting XU <your-email@placeholder.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 */

#ifndef RTD_OEM_PARAMS_H
#define RTD_OEM_PARAMS_H

#include "board.h" /* THIS INCLUDE IS MANDATORY */
#include "saul_reg.h"
#include "rtd_oem.h"
#include "rtd_oem_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Set default configuration parameters for the Atlas Scientific RTD OEM driver
 * @{
 */
#ifndef RTD_OEM_PARAM_I2C
#define RTD_OEM_PARAM_I2C                  (I2C_DEV(0))
#endif
#ifndef RTD_OEM_PARAM_ADDR
#define RTD_OEM_PARAM_ADDR                 (0x68)
#endif
#ifndef RTD_OEM_PARAM_INTERRUPT_PIN
#define RTD_OEM_PARAM_INTERRUPT_PIN        (GPIO_UNDEF)
#endif
#ifndef RTD_OEM_PARAM_INTERRUPT_OPTION
#define RTD_OEM_PARAM_INTERRUPT_OPTION     (RTD_OEM_IRQ_BOTH)
#endif
#ifndef RTD_OEM_PARAM_INTERRUPT_GPIO_MODE
#define RTD_OEM_PARAM_INTERRUPT_GPIO_MODE  (GPIO_IN_PD)
#endif

#ifndef RTD_OEM_PARAMS
#define RTD_OEM_PARAMS       { .i2c = RTD_OEM_PARAM_I2C,        \
                              .addr = RTD_OEM_PARAM_ADDR,       \
                              .interrupt_pin = RTD_OEM_PARAM_INTERRUPT_PIN, \
                              .gpio_mode = RTD_OEM_PARAM_INTERRUPT_GPIO_MODE, \
                              .irq_option = RTD_OEM_PARAM_INTERRUPT_OPTION }
#endif
#ifndef RTD_OEM_SAUL_INFO
#define RTD_OEM_SAUL_INFO       { .name = "RTD OEM sensor" }
#endif
/** @} */
/**
 * @brief   RTD OEM defaults if not defined for a board or application
 */
static const rtd_oem_params_t rtd_oem_params[] =
{
    RTD_OEM_PARAMS
};

/**
 * @brief   Additional meta information to keep in the SAUL registry
 */
static const saul_reg_info_t rtd_oem_saul_info[] =
{
    RTD_OEM_SAUL_INFO
};

#ifdef __cplusplus
}
#endif

#endif /* RTD_OEM_PARAMS_H */
/** @} */
