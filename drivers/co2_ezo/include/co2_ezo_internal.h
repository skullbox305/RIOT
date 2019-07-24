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
 * @brief       Default configuration for the Atlas Scientific CO2 EZO sensors
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
#define CO2_EZO_LED_ON                  ("L,1")         /**< LED ON */
#define CO2_EZO_LED_OFF                 ("L,0")         /**< LED OFF */
#define CO2_EZO_LED_STATE               ("L,?")         /**< LED state on/off? */

#define CO2_EZO_FIND                    ("Find")        /**< LED rapidly blinks white, find device */

#define CO2_EZO_TAKE_READING            ("R")           /**< Return 1 reading from device */

#define CO2_EZO_ALARM_ON                ("Alarm,en,1")  /**< Enable alarm */
#define CO2_EZO_ALARM_OFF               ("Alarm,en,0")  /**< Disable alarm */
#define CO2_EZO_ALARM_SET               ("Alarm,")      /**< Sets alarm to n = 0 - 10000 */
#define CO2_EZO_ALARM_SET_TOL           ("Alarm,tol,")  /**< Sets alarm tolerance(n = 0-500 ppm) */
#define CO2_EZO_ALARM_STATE             ("Alarm,?")     /**< Alarm state */

#define CO2_EZO_INTERNAL_TEMP_ON        ("O,t,1")       /**< Enable internal temperature output*/
#define CO2_EZO_INTERNAL_TEMP_OFF       ("O,t,0")       /**< Disable internal temperature output */
#define CO2_EZO_INTERNAL_TEMP           ("O,?")         /**< Read internal temperature output */

#define CO2_EZO_DEV_INFO                ("i")           /**< Device information */

#define CO2_EZO_READ_DEV_STATUS         ("Status")      /**< Reads voltage at Vcc pin and reason for last restart */

#define CO2_EZO_SLEEP_MODE              ("Sleep")       /**< Enter sleep mode/low power */

#define CO2_EZO_PLOCK_ENABLE            ("Plock,1")     /**< Enable Plock */
#define CO2_EZO_PLOCK_DISABLE           ("Plock,0")     /**< Disable Plock */
#define CO2_EZO_PLOCK_STATE             ("Plock,?")     /**< Plock sate on/off? */

#define CO2_EZO_I2C_ADDR_SET            ("I2C,")        /**< Sets I2C address (n = 1 - 127) */

#define CO2_EZO_FACTORY_RESET           ("Factory")     /**< Enable factory reset */

/** @} */


/**
 * @name    CO2 EZO internal definitions
 * @{
 */
#define CO2_EZO_PROCESS_PENDING        (254)

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* CO2_EZO_INTERNAL */
/** @} */
