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
 * @brief       Implementation of get and set functions for the Iridium SBD
 *
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 * @}
 */

#include "isbd.h"
#include "isbd_internal.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

uint8_t isbd_get_state(const isbd_t *dev)
{
    return dev->_internal.state;
}

void isbd_set_state(isbd_t *dev, uint8_t state)
{
#if ENABLE_DEBUG
    printf("[isbd] state: ");
    switch (state) {
        case ISBD_STATE_OFF:
            puts("OFF");
            break;
        case ISBD_STATE_STANDBY:
            puts("STANDBY");
            break;
        case ISBD_STATE_IDLE:
            puts("IDLE");
            break;
        case ISBD_STATE_TX:
            puts("TX");
            break;
        case ISBD_STATE_RX:
            puts("RX");
            break;
        default:
            puts("UNKNOWN");
            break;
    }
#endif

    dev->_internal.state = state;
}


void isbd_set_off(isbd_t *dev)
{
    if (dev->params.sleep_pin == GPIO_UNDEF) {
        DEBUG("[isbd] off: No sleep pin defined.\n");
        return;
    }

    if (isbd_get_state(dev) == ISBD_STATE_TX ||
        isbd_get_state(dev) == ISBD_STATE_RX) {
        DEBUG("[isbd] off: Device still busy\n");
        return;
    }

#if false // recent research suggest this is not what you should do when just sleeping
   // Best Practices Guide suggests this before shutdown
    isbd_flush_eeprom(dev);
#endif

    /* Best Practices Guide suggests waiting at least 2 seconds
     * before powering off again */
    while (xtimer_less(xtimer_diff(xtimer_now(), dev->_internal.power_on_time),
                       xtimer_ticks_from_usec(2 * US_PER_SEC)));

    isbd_set_state(dev, ISBD_STATE_OFF);
    at_dev_poweroff(&dev->at_dev);
    gpio_clear(dev->params.sleep_pin);
}

void isbd_set_standby(isbd_t *dev)
{
    int8_t result = isbd_on(dev);

    isbd_set_at_echo(dev, true);
    isbd_set_dtr_off(dev);
    isbd_set_flow_control_off(dev);

//    isbd_set_sbd_session_timeout(dev, ISBD_DEFAULT_SBD_SESSION_TIMEOUT);
//    isbd_set_ring_alert(dev);

    if (result == ISBD_ERR_GPIO_UNDEF) {
        /* no sleep pin defined, so device is already on. Put directly in stand-by mode */
        isbd_set_state(dev, ISBD_STATE_STANDBY);
    }
}

void isbd_set_idle(isbd_t *dev)
{
    if (dev->_internal.state == ISBD_STATE_IDLE) {
        return;
    }
    if (dev->_internal.state == ISBD_STATE_OFF) {
        isbd_set_standby(dev);
    }

#if !ISBD_TEST_MODE
    isbd_start_network_registration(dev);
#endif
}

void isbd_set_ring_alert(isbd_t *dev)
{
    if (isbd_get_state(dev) == ISBD_STATE_OFF) {
        DEBUG("[isbd] ABORT: Device is in sleep mode\n");
        return;
    }

    if (at_send_cmd_get_resp(&dev->at_dev, "at+sbdmta=1",
                             dev->_internal.resp_buf,
                             sizeof(dev->_internal.resp_buf),
                             2 * US_PER_SEC) < 0) {
        DEBUG("[isbd] enabling ring alert pin failed.\n");
    }
}

void isbd_set_sbd_session_timeout(isbd_t *dev, uint16_t timeout_sec)
{
    if (isbd_get_state(dev) == ISBD_STATE_OFF) {
        DEBUG("[isbd] ABORT: Device is in sleep mode\n");
        return;
    }

    char cmd[14];
    snprintf(cmd, sizeof(cmd) - 1, "at+sbdst=%d ", timeout_sec);

    if (at_send_cmd_get_resp(&dev->at_dev, cmd, dev->_internal.resp_buf,
                             sizeof(dev->_internal.resp_buf),
                             2 * US_PER_SEC) < 0) {
        DEBUG("[isbd] setting sbd session timeout failed.\n");
    }
}

void isbd_set_flow_control_off(isbd_t *dev)
{
    if (at_send_cmd_wait_ok(&dev->at_dev, "AT&K0", 10 * US_PER_SEC) < 0) {
        DEBUG("[isbd] init: Turning off Flow Control failed\n");
    }
}

void isbd_set_dtr_off(isbd_t *dev)
{
    if (at_send_cmd_wait_ok(&dev->at_dev, "AT&D0", 10 * US_PER_SEC) < 0) {
        DEBUG("[isbd] init: Turning off DTR failed\n");
    }
}

void isbd_set_at_echo(isbd_t *dev, bool echo)
{
    (void) echo;
    if (at_send_cmd_wait_ok(&dev->at_dev, "ATE1", 10 * US_PER_SEC) < 0) {
        DEBUG("[isbd] init: Turning off DTR failed\n");
    }
}

