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
 * @brief       Register definitions for the Atlas Scientific DO OEM sensor.
 *
 * @author      Ting Xu <timtsui@outlook.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 */

#ifndef DO_OEM_REGS_H
#define DO_OEM_REGS_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Atlas Scientific DO OEM register addresses
 *
 * All registers in the DO OEM are 10 bit wide and transmitted MSB first.
 *
 */
typedef enum ph_oem_reg {
    DO_OEM_REG_CALIBRATION		        = 0x08, /**< Calibration request register (write only)  */
	DO_OEM_REG_CALIBRATION_CONFIRM      = 0x09, /**< Calibration confirm register (read only)*/
	DO_OEM_REG_SALI_COMPENSATION_BASE   = 0x0A, /**< Salinity compensation register base address. Register order is: MSB, high byte, low byte, LSB (0x0A-0x0D) (read/write)  */
	DO_OEM_REG_PRES_COMPENSATION_BASE   = 0x0E, /**< Pressure compensation register base address. Register order is: MSB, high byte, low byte, LSB (0x0E-0x11) (read/write)  */
    DO_OEM_REG_TEMP_COMPENSATION_BASE   = 0x12, /**< Temperature compensation register base address. Register order is: MSB, high byte, low byte, LSB (0x12-0x15) (read/write) */
	DO_OEM_REG_SALI_COMFIRMATION_BASE   = 0x16, /**< Salinity confirm register base address. Register order is: MSB, high byte, low byte, LSB (0x16-0x19) (read only)   */
	DO_OEM_REG_PRES_COMFIRMATION_BASE   = 0x1A, /**< Salinity confirm register base address. Register order is: MSB, high byte, low byte, LSB (0x1A-0x1D) (read only)  */
    DO_OEM_REG_TEMP_COMFIRMATION_BASE   = 0x1E, /**< Salinity confirm register base address. Register order is: MSB, high byte, low byte, LSB (0x1E-0x21) (read only) */
	DO_OEM_REG_DO_MGL_READING_BASE      = 0x22, /**< DO in mg/L reading register base address, order= MSB, high byte, low byte, LSB (0x22-0x25) (read only) */
	DO_OEM_REG_DO_PERCENT_READING_BASE  = 0x26, /**< DO in % saturation reading register base address, order= MSB, high byte, low byte, LSB (0x26-0x29) (read only) */
} do_oem_reg_t;


/**
 * @brief Device ID of the @ref DO_OEM_REG_DEVICE_TYPE register of a DO OEM sensor
 */
#define DO_OEM_DEVICE_TYPE_ID  0x03

#ifdef __cplusplus
}
#endif

#endif /* DO_OEM_REGS_H */
/** @} */
