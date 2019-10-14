/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_do_oem
 * @{
 *
 * @file
 * @brief       Default configuration for Atlas Scientific DO OEM sensors
 *
 * @author      Ting Xu <timtsui@outlook.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 */

#ifndef DO_OEM_PARAMS_H
#define DO_OEM_PARAMS_H

#include "board.h" /* THIS INCLUDE IS MANDATORY */
#include "do_oem_regs.h"
#include "saul_reg.h"
#include "do_oem.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Set default configuration parameters for the Atlas Scientific DO OEM driver
 * @{
 */
#ifndef DO_OEM_PARAM_I2C
#define DO_OEM_PARAM_I2C                  (I2C_DEV(0))
#endif
#ifndef DO_OEM_PARAM_ADDR
#define DO_OEM_PARAM_ADDR                 (0x67)
#endif
#ifndef DO_OEM_PARAM_INTERRUPT_PIN
#define DO_OEM_PARAM_INTERRUPT_PIN        (GPIO_UNDEF)
#endif
#ifndef DO_OEM_PARAM_INTERRUPT_OPTION
#define DO_OEM_PARAM_INTERRUPT_OPTION     (OEM_COMMON_IRQ_BOTH)
#endif
#ifndef DO_OEM_PARAM_INTERRUPT_GPIO_MODE
#define DO_OEM_PARAM_INTERRUPT_GPIO_MODE  (GPIO_IN_PD)
#endif

#ifndef DO_OEM_PARAMS
#define DO_OEM_PARAMS       { .i2c = DO_OEM_PARAM_I2C,        \
                              .addr = DO_OEM_PARAM_ADDR,       \
                              .interrupt_pin = DO_OEM_PARAM_INTERRUPT_PIN, \
                              .gpio_mode = DO_OEM_PARAM_INTERRUPT_GPIO_MODE, \
                              .irq_option = DO_OEM_PARAM_INTERRUPT_OPTION, \
                              .device_type_id = DO_OEM_DEVICE_TYPE_ID }
#endif
#ifndef DO_OEM_SAUL_INFO
#define DO_OEM_SAUL_INFO       { .name = "DO OEM sensor" }
#endif
/** @} */
/**
 * @brief   DO OEM defaults if not defined for a board or application
 */
static const oem_common_params_t do_oem_params[] =
{
    DO_OEM_PARAMS
};

/**
 * @brief   Additional meta information to keep in the SAUL registry
 */
static const saul_reg_info_t do_oem_saul_info[] =
{
    DO_OEM_SAUL_INFO
};

#ifdef __cplusplus
}
#endif

#endif /* DO_OEM_PARAMS_H */
/** @} */
