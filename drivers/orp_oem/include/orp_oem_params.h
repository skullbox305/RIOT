/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_orp_oem
 * @{
 *
 * @file
 * @brief       Default configuration for Atlas Scientific ORP OEM sensors
 *
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 * @author      Ting XU <timtsui@outlook.com>
 */

#ifndef ORP_OEM_PARAMS_H
#define ORP_OEM_PARAMS_H

#include "board.h" /* THIS INCLUDE IS MANDATORY */
#include "saul_reg.h"
#include "orp_oem.h"
#include "orp_oem_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Set default configuration parameters for the Atlas Scientific ORP OEM driver
 * @{
 */
#ifndef ORP_OEM_PARAM_I2C
#define ORP_OEM_PARAM_I2C                  (I2C_DEV(0))
#endif
#ifndef ORP_OEM_PARAM_ADDR
#define ORP_OEM_PARAM_ADDR                 (0x66)
#endif
#ifndef ORP_OEM_PARAM_INTERRUPT_PIN
#define ORP_OEM_PARAM_INTERRUPT_PIN        (GPIO_UNDEF)
#endif
#ifndef ORP_OEM_PARAM_INTERRUPT_OPTION
#define ORP_OEM_PARAM_INTERRUPT_OPTION     (OEM_COMMON_IRQ_BOTH)
#endif
#ifndef ORP_OEM_PARAM_INTERRUPT_GPIO_MODE
#define ORP_OEM_PARAM_INTERRUPT_GPIO_MODE  (GPIO_IN_PD)
#endif

#ifndef ORP_OEM_PARAMS
#define ORP_OEM_PARAMS       { .i2c = ORP_OEM_PARAM_I2C,        \
                              .addr = ORP_OEM_PARAM_ADDR,       \
                              .interrupt_pin = ORP_OEM_PARAM_INTERRUPT_PIN, \
                              .gpio_mode = ORP_OEM_PARAM_INTERRUPT_GPIO_MODE, \
                              .irq_option = ORP_OEM_PARAM_INTERRUPT_OPTION, \
                              .device_type_id = ORP_OEM_DEVICE_TYPE_ID}
#endif
#ifndef ORP_OEM_SAUL_INFO
#define ORP_OEM_SAUL_INFO       { .name = "ORP OEM sensor" }
#endif
/** @} */
/**
 * @brief   ORP OEM defaults if not defined for a board or application
 */
static const oem_common_params_t orp_oem_params[] =
{
    ORP_OEM_PARAMS
};

/**
 * @brief   Additional meta information to keep in the SAUL registry
 */
static const saul_reg_info_t orp_oem_saul_info[] =
{
    ORP_OEM_SAUL_INFO
};

#ifdef __cplusplus
}
#endif

#endif /* ORP_OEM_PARAMS_H */
/** @} */
