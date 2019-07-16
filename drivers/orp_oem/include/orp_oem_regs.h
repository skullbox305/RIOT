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
 * @brief       Register definitions for the Atlas Scientific ORP OEM sensor.
 *
 * @author      Ting XU <timtsui@outlook.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 */

#ifndef ORP_OEM_REGS_H
#define ORP_OEM_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Atlas Scientific ORP OEM register addresses
 *
 * All registers in the ORP OEM are 8 bit wide and transmitted MSB first.
 *
 */
typedef enum orp_oem_reg {
    ORP_OEM_REG_DEVICE_TYPE              = 0x00, /**< Device type register (read only) */
    ORP_OEM_REG_FIRMWARE_VERSION         = 0x01, /**< Firmware version register (read only) */
    ORP_OEM_REG_UNLOCK                   = 0x02, /**< SMBus/I²C address lock/unlock register (read/write) */
    ORP_OEM_REG_ADDRESS                  = 0x03, /**< SMBus/I²C address register (read/write) */
    ORP_OEM_REG_INTERRUPT                = 0x04, /**< Interrupt control register (read/write) */
    ORP_OEM_REG_LED                      = 0x05, /**< LED control register (read/write) */
    ORP_OEM_REG_HIBERNATE                = 0x06, /**< Active/hibernate register (read/write) */
    ORP_OEM_REG_NEW_READING              = 0x07, /**< New reading available register (read/write)  */
    ORP_OEM_REG_CALIBRATION_BASE         = 0x08, /**< Calibration value register base address. Register order is: MSB, high byte, low byte, LSB (0x08-0x0B) (read/write) */
    ORP_OEM_REG_CALIBRATION_REQUEST      = 0x0C, /**< Calibration request register (read/write) */
    ORP_OEM_REG_CALIBRATION_CONFIRM      = 0x0D, /**< Calibration confirm register (read/write) */
    ORP_OEM_REG_ORP_READING_BASE         = 0x0E, /**< pH reading register base address, order= MSB, high byte, low byte, LSB (0x0E-0x11) (read only) */

} orp_oem_reg_t;


/**
 * @brief Device ID of the @ref ORP_OEM_REG_DEVICE_TYPE register of a ORP OEM sensor
 */
#define ORP_OEM_DEVICE_TYPE_ID  0x02

#ifdef __cplusplus
}
#endif

#endif /* ORP_OEM_REGS_H */
/** @} */
