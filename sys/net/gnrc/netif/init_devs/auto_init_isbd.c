/*
 * Copyright (C) 2020 Igor Knippenberg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 */

/**
 * @ingroup     sys_auto_init_gnrc_netif
 * @{
 *
 * @file
 * @brief       Auto initialization for Iridium SBD interfaces
 *
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 */

#include "log.h"
#include "board.h"
#include "net/gnrc/netif/raw.h"
#include "net/gnrc.h"

#include "isbd.h"
#include "isbd_params.h"

/**
 * @brief   Calculate the number of configured ISBD devices
 */
#define ISBD_NUMOF        ARRAY_SIZE(isbd_params)

/**
 * @brief   Define stack parameters for the MAC layer thread
 */
#define ISBD_STACKSIZE           (THREAD_STACKSIZE_DEFAULT)
#ifndef ISBD_PRIO
#define ISBD_PRIO                (GNRC_NETIF_PRIO)
#endif

/**
 * @brief   Allocate memory for device descriptors, stacks, and GNRC adaption
 */
static isbd_t isbd_devs[ISBD_NUMOF];
static char isbd_stacks[ISBD_NUMOF][ISBD_STACKSIZE];
static gnrc_netif_t _netif[ISBD_NUMOF];

void auto_init_isbd(void)
{
    for (unsigned i = 0; i < ISBD_NUMOF; ++i) {
        LOG_DEBUG("[auto_init_netif] initializing isbd #%u\n", i);

        isbd_setup(&isbd_devs[i], &isbd_params[i], i);
        gnrc_netif_raw_create(&_netif[i], isbd_stacks[i],
                              ISBD_STACKSIZE, ISBD_PRIO,
                              "isbd", (netdev_t *)&isbd_devs[i]);
    }
}
/** @} */
