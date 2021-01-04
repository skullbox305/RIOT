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

void isbd_set_state(isbd_t *dev, isbd_states_t state)
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

    // Best Practices Guide suggests this before shutdown
    isbd_flush_eeprom(dev);

    /* Best Practices Guide suggests waiting at least 2 seconds
     * before powering off again */
    while (xtimer_less(xtimer_diff(xtimer_now(), dev->_internal.power_on_time),
                       xtimer_ticks_from_usec(2 * US_PER_SEC))) {}

    isbd_set_state(dev, ISBD_STATE_OFF);
    at_dev_poweroff(&dev->at_dev);
    gpio_clear(dev->params.sleep_pin);
}

void isbd_set_standby(isbd_t *dev)
{
    int8_t result = isbd_on(dev);

    if (result == ISBD_ERR_GPIO_UNDEF) {
        /* no sleep pin defined, so device is already on. Put directly in stand-by mode */
        DEBUG("[isbd] STANDBY\n");
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

#ifndef CONFIG_ISBD_TEST_MODE
    isbd_start_network_registration(dev);
#endif
}

void isbd_set_ring_alert(isbd_t *dev, bool enable)
{
    char command[13];

    snprintf(command, sizeof(command) - 1, ISBD_SBDMTA, enable);

    if (at_send_cmd_get_resp(&dev->at_dev, command,

                             dev->_internal.resp_buf,
                             sizeof(dev->_internal.resp_buf),
                             2 * US_PER_SEC) < 0) {
        DEBUG("[isbd] enabling ring alert pin failed.\n");
    }
}

void isbd_set_sbd_session_timeout(isbd_t *dev, uint16_t timeout_sec)
{
    char cmd[14];

    snprintf(cmd, sizeof(cmd) - 1, ISBD_SBDST, timeout_sec);

    if (at_send_cmd_get_resp(&dev->at_dev, cmd, dev->_internal.resp_buf,
                             sizeof(dev->_internal.resp_buf),
                             2 * US_PER_SEC) < 0) {
        DEBUG("[isbd] setting sbd session timeout failed.\n");
    }
}

void isbd_set_flow_control_off(isbd_t *dev)
{
    if (at_send_cmd_wait_ok(&dev->at_dev, ISBD_FLOW_CTR_OFF,
                            10 * US_PER_SEC) < 0) {
        DEBUG("[isbd] init: Turning off Flow Control failed\n");
    }
}

void isbd_set_dtr_off(isbd_t *dev)
{
    if (at_send_cmd_wait_ok(&dev->at_dev, ISBD_DTR_OFF, 10 * US_PER_SEC) < 0) {
        DEBUG("[isbd] init: Turning off DTR failed\n");
    }
}

void isbd_set_at_echo(isbd_t *dev, bool enable)
{
    char cmd[6];

    snprintf(cmd, sizeof(cmd) - 1, ISBD_AT_ECHO_ON, enable);

    if (at_send_cmd_wait_ok(&dev->at_dev, cmd,
                            10 * US_PER_SEC) < 0) {
        DEBUG("[isbd] init: Turning off at echo failed\n");
    }
}

void isbd_set_store_config(isbd_t *dev, uint8_t profile_nr)
{
    char cmd[7];

    snprintf(cmd, sizeof(cmd) - 1, ISBD_STORE_CFG, profile_nr);

    if (at_send_cmd_wait_ok(&dev->at_dev, cmd, 10 * US_PER_SEC) < 0) {
        DEBUG("[isbd] init: Storing settings in profile %d failed\n",
              profile_nr);
    }
}

void isbd_set_default_reset_config(isbd_t *dev, uint8_t profile_nr)
{
    char cmd[7];

    snprintf(cmd, sizeof(cmd) - 1, ISBD_RST_PROFILE, profile_nr);

    if (at_send_cmd_wait_ok(&dev->at_dev, cmd, 10 * US_PER_SEC) < 0) {
        DEBUG("[isbd] init: Setting default reset profile %d failed\n",
              profile_nr);
    }
}
