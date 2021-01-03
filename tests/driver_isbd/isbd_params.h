/*
 * Copyright (C) 2020 Igor Knippenberg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_isbd
 * @{
 * @file
 * @brief       Default configuration for Iridium SBD driver
 *
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 */

#ifndef ISBD_PARAMS_H
#define ISBD_PARAMS_H

#include "board.h"
#include "isbd.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Set default configuration parameters for the Iridium SBD
 * @{
 */
#ifndef ISBD_PARAM_UART
#define ISBD_PARAM_UART            (UART_DEV(1))
#endif
#ifndef ISBD_PARAM_BAUDRATE
#define ISBD_PARAM_BAUDRATE        (19200U)
#endif
#ifndef ISBD_PARAM_SLEEP_PIN
#define ISBD_PARAM_SLEEP_PIN       GPIO_PIN(0,25)
#endif
#ifndef ISBD_PARAM_RING_PIN
#define ISBD_PARAM_RING_PIN        GPIO_PIN(0,12)
#endif
#ifndef ISBD_PARAM_NET_AVAIL_PIN
#define ISBD_PARAM_NET_AVAIL_PIN   GPIO_PIN(0,21)
#endif

#ifndef ISBD_PARAMS
#define ISBD_PARAMS           { .uart = ISBD_PARAM_UART,        \
                                .baudrate = ISBD_PARAM_BAUDRATE, \
                                .sleep_pin = ISBD_PARAM_SLEEP_PIN,  \
                                .ring_pin = ISBD_PARAM_RING_PIN, \
                                .network_avail_pin = ISBD_PARAM_NET_AVAIL_PIN }
#endif
/**@}*/

/**
 * @brief   Iridium SBD configuration
 */
static const isbd_params_t isbd_params[] =
{
    ISBD_PARAMS
};

#ifdef __cplusplus
}
#endif

#endif /* ISBD_PARAM_PARAMS_H */
/** @} */
