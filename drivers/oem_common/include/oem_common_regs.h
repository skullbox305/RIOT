/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_oem_common
 * @{
 *
 * @file
 * @brief       Common register definitions for the Atlas Scientific OEM sensor family.
 *
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 */

#ifndef OEM_COMMON_REGS_H
#define OEM_COMMON_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Common register addresses of the Atlas Scientific OEM sensor family
 *
 * All OEM registers are 8 bit wide and transmitted MSB first.
 *
 */
typedef enum oem_common_reg {
    OEM_COMMON_REG_DEVICE_TYPE      = 0x00, /**< Device type register (read only) */
    OEM_COMMON_REG_FIRMWARE_VERSION = 0x01, /**< Firmware version register (read only) */
    OEM_COMMON_REG_UNLOCK           = 0x02, /**< SMBus/I²C address lock/unlock register (read/write) */
    OEM_COMMON_REG_ADDRESS          = 0x03, /**< SMBus/I²C address register (read/write) */
    OEM_COMMON_REG_INTERRUPT        = 0x04, /**< Interrupt control register (read/write) */
    OEM_COMMON_REG_LED              = 0x05, /**< LED control register (read/write) */
    OEM_COMMON_REG_HIBERNATE        = 0x06, /**< Active/hibernate register (read/write) */
    OEM_COMMON_REG_NEW_READING      = 0x07, /**< New reading available register (read/write)  */
} oem_common_reg_t;

#ifdef __cplusplus
}
#endif

#endif /* OEM_COMMON_REGS_H */
/** @} */
