/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_rgb_ezo
 * @{
 *
 * @file
 * @brief       Internal definitions for the Atlas Scientific RGB EZO sensors
 *
 * @author      Ting Xu <timtsui@outlook.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 */

#ifndef RGB_EZO_INTERNAL_H
#define RGB_EZO_INTERNAL_H

#include "board.h" /* THIS INCLUDE IS MANDATORY */
#include "saul_reg.h"
#include "rgb_ezo.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @name    RGB EZO commands
 * @{
 */
#define RGB_EZO_INDICATOR_LED        ("iL,")        /**< LED ON (L,1), OFF (L,1) */

#define RGB_EZO_TARGET_LED       	 ("L,")         /**< Set target LED brightness */
#define RGB_EZO_TARGET_LED_TRIGGER 	 ("L,,T")       /**< Set target LED brightness/trigger target only when a reading is taken */


#define RGB_EZO_TAKE_READING         ("R")          /**< Return 1 reading from device */

#define RGB_EZO_SET_PARAMETER         ("O,") 		/**< Enable/disable output parameter */
#define RGB_EZO__PARAMETER_STATE      ("O,?") 		/**< Enabled output parameter */

#define RGB_EZO_GAMMA_CORRECTION     ("G,")   	    /**< Sets gamma correction */
#define RGB_EZO_GAMMA_VALUE 	     ("G,?")   	    /**< Gets gamma correction value */

#define RGB_EZO_CALIBRATE      	     ("Cal") 		/**< Calibrates the sensor) */

#define RGB_EZO_DEV_INFO             ("i")          /**< Device information */

#define RGB_EZO_SLEEP_MODE           ("Sleep")      /**< Enter sleep mode/low power */

#define RGB_EZO_I2C_ADDR_SET         ("I2C,")       /**< Sets I2C address (n = 1 - 127) */

/** @} */


/**
 * @name    RGB EZO response codes
 * @{
 */
#define RGB_EZO_NO_DATA              (255)       /**< Not data to send */
#define RGB_EZO_CMD_PENDING          (254)       /**< Still processing command, not ready */
#define RGB_EZO_SYNTAX_ERR           (2)         /**< Syntax error in command */
#define RGB_EZO_SUCCESS              (1)         /**< Command request successful */

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* RGB_EZO_INTERNAL */
/** @} */
