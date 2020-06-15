/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_isbd
 * @{
 * @file
 * @brief       Netdev driver definitions for Iridium SBD driver
 *
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 */

#ifndef ISBD_NETDEV_H
#define ISBD_NETDEV_H

#include "net/netdev.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Reference to the netdev device driver struct
 */
extern const netdev_driver_t isbd_driver;

#ifdef __cplusplus
}
#endif

#endif /* ISBD_NETDEV_H */
/** @} */
