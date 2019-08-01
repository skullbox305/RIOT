/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_ec_oem
 * @{
 *
 * @file
 * @brief       Register definitions for the Atlas Scientific EC OEM sensor.
 *
 * @author      Ting Xu <timtsui@outlook.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 */

#ifndef EC_OEM_REGS_H
#define EC_OEM_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Atlas Scientific EC OEM register addresses
 *
 * All registers in the EC OEM are 10 bit wide and transmitted MSB first.
 *
 */
typedef enum ph_oem_reg {
    EC_OEM_REG_DEVICE_TYPE              = 0x00, /**< Device type register (read only) */
    EC_OEM_REG_FIRMWARE_VERSION         = 0x01, /**< Firmware version register (read only) */
    EC_OEM_REG_UNLOCK                   = 0x02, /**< SMBus/I²C address lock/unlock register (read/write) */
    EC_OEM_REG_ADDRESS                  = 0x03, /**< SMBus/I²C address register (read/write) */
    EC_OEM_REG_INTERRUPT                = 0x04, /**< Interrupt control register (read/write) */
    EC_OEM_REG_LED                      = 0x05, /**< LED control register (read/write) */
    EC_OEM_REG_HIBERNATE                = 0x06, /**< Active/hibernate register (read/write) */
    EC_OEM_REG_NEW_READING              = 0x07, /**< New reading available register (read/write)  */
	EC_OEM_REG_SET_PROBE_TYPE			= 0x08, /**< Set probe type available register (read/write)  */
	EC_OEM_REG_CALIBRATION_BASE         = 0x0A, /**< Calibration value register base address. Register order is: MSB, high byte, low byte, LSB (0x08-0x0B) (read/write) */
    EC_OEM_REG_CALIBRATION_REQUEST      = 0x0E, /**< Calibration request register (read/write) */
    EC_OEM_REG_CALIBRATION_CONFIRM      = 0x0F, /**< Calibration confirm register (read only) */
    EC_OEM_REG_TEMP_COMPENSATION_BASE   = 0x10, /**< Temperature compensation register base address. Register order is: MSB, high byte, low byte, LSB (0x0E-0x11) (read only */
    EC_OEM_REG_TEMP_CONFIRMATION_BASE   = 0x14, /**< Temperature confirm register base address. Register order is: MSB, high byte, low byte, LSB (0x12-0x15) (read only) */
    EC_OEM_REG_EC_READING_BASE          = 0x18, /**< EC reading register base address, order= MSB, high byte, low byte, LSB (0x16-0x19) (read only) */
	EC_OEM_REG_TDS_READING_BASE         = 0x1C, /**< TDS reading register base address, order= MSB, high byte, low byte, LSB (0x16-0x19) (read only) */
	EC_OEM_REG_PSS_READING_BASE         = 0x20, /**< PSS reading register base address, order= MSB, high byte, low byte, LSB (0x16-0x19) (read only) */
} ec_oem_reg_t;


/**
 * @brief Device ID of the @ref EC_OEM_REG_DEVICE_TYPE register of an EC OEM sensor
 */
#define EC_OEM_DEVICE_TYPE_ID  0x04

#ifdef __cplusplus
}
#endif

#endif /* EC_OEM_REGS_H */
/** @} */
