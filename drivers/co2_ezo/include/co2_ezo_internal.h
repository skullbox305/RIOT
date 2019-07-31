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
 * @brief       Internal definitions for the Atlas Scientific CO2 EZO sensors
 *
 * @author      Ting Xu <timtsui@outlook.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 */

#ifndef CO2_EZO_INTERNAL_H
#define CO2_EZO_INTERNAL_H

#include "board.h" /* THIS INCLUDE IS MANDATORY */
#include "saul_reg.h"
#include "co2_ezo.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @name    CO2 EZO commands
 * @{
 */
#define CO2_EZO_LED                  ("L,")        /**< LED ON (L,1), OFF (L,1) */

#define CO2_EZO_TAKE_READING         ("R")          /**< Return 1 reading from device */

#define CO2_EZO_ALARM_ON             ("Alarm,en,1") /**< Enable alarm */
#define CO2_EZO_ALARM_OFF            ("Alarm,en,0") /**< Disable alarm */
#define CO2_EZO_ALARM_SET            ("Alarm,")     /**< Sets alarm to n = 0 - 10000 */
#define CO2_EZO_ALARM_SET_TOL        ("Alarm,tol,") /**< Sets alarm tolerance(n = 0-499 ppm) */
#define CO2_EZO_ALARM_STATE          ("Alarm,?")    /**< Alarm state */

#define CO2_EZO_DEV_INFO             ("i")          /**< Device information */

#define CO2_EZO_SLEEP_MODE           ("Sleep")      /**< Enter sleep mode/low power */

#define CO2_EZO_I2C_ADDR_SET         ("I2C,")       /**< Sets I2C address (n = 1 - 127) */

/** @} */


/**
 * @name    CO2 EZO response codes
 * @{
 */
#define CO2_EZO_NO_DATA              (255)       /**< Not data to send */
#define CO2_EZO_CMD_PENDING          (254)       /**< Still processing command, not ready */
#define CO2_EZO_SYNTAX_ERR           (2)         /**< Syntax error in command */
#define CO2_EZO_SUCCESS              (1)         /**< Command request successful */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* CO2_EZO_INTERNAL */
/** @} */
