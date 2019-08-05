/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_ph_ezo
 * @{
 *
 * @file
 * @brief       Internal definitions for the Atlas Scientific pH EZO sensors
 *
 * @author      Ting Xu <timtsui@outlook.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 */

#ifndef PH_EZO_INTERNAL_H
#define PH_EZO_INTERNAL_H

#include "board.h" /* THIS INCLUDE IS MANDATORY */
#include "saul_reg.h"
#include "ph_ezo.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @name    pH EZO commands
 * @{
 */
#define PH_EZO_LED                  ("L,")        	 /**< LED ON (L,1), OFF (L,1) */

#define PH_EZO_TAKE_READING         ("R")          	 /**< Return 1 reading from device */

#define PH_EZO_CALIBRATE_MID        ("Cal,mid,")	 /**< Single point calibration at midpoint */
#define PH_EZO_CALIBRATE_LOW        ("Cal,low,") 	 /**< Two point calibration at lowpoint */
#define PH_EZO_CALIBRATE_HIGH       ("Cal,high,") 	 /**< Three point calibration at hight point */
#define PH_EZO_CALIBRATE_CLEAR      ("Cal,clear") 	 /**< Delete calibration data */
#define PH_EZO_CALIBRATE_STATE      ("Cal,?") 		 /**< Calibration state */

#define PH_EZO_DEV_INFO             ("i")            /**< Device information */

#define PH_EZO_SLEEP_MODE           ("Sleep")        /**< Enter sleep mode/low power */

#define PH_EZO_I2C_ADDR_SET         ("I2C,")         /**< Sets I2C address (n = 1 - 127) */

#define PH_EZO_FACTORY_RESET        ("Factory")      /**< Factory reset */

#define PH_EZO_SLOPE 		        ("Slope,?")      /**< Returns the slope of the pH probe */

/** @} */


/**
 * @name    PH EZO response codes
 * @{
 */
#define PH_EZO_NO_DATA              (255)       /**< Not data to send */
#define PH_EZO_CMD_PENDING          (254)       /**< Still processing cmd, not ready */
#define PH_EZO_SYNTAX_ERR           (2)         /**< Syntax error in command */
#define PH_EZO_SUCCESS              (1)         /**< Command request successful */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* PH_EZO_INTERNAL */
/** @} */
