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
 * @brief       Netdev adaptation for the Iridium SBD driver
 *
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 * @}
 */

#include <errno.h>

#include "net/netopt.h"
#include "net/netdev.h"

#include "isbd.h"
#include "isbd_internal.h"
#include "isbd_netdev.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

/* Internal helper functions */
static int _set_state(isbd_t *dev, netopt_state_t state);
static int _get_state(isbd_t *dev, void *val);
void _on_network_avail_irq(void *arg);
void _on_new_msg_irq(void *arg);

static int _send(netdev_t *netdev, const iolist_t *iolist)
{
    isbd_t *dev = (isbd_t *)netdev;
    uint8_t state = isbd_get_state(dev);
    int res;

    if (state == ISBD_STATE_TX || state == ISBD_STATE_RX) {
        DEBUG("[isbd] netdev: device still busy with TX/RX\n");
        netdev->event_callback(netdev, NETDEV_EVENT_TX_MEDIUM_BUSY);
        return -EBUSY;
    }

    if(state == ISBD_STATE_OFF) {
    	isbd_set_standby(dev);
    }

    uint8_t size = iolist_size(iolist);

    /* Write payload to tx buf if size > 0, else it is a mailbox check with payload size 0.
     * Used for receiving queued rx messages */
    if (size > 0) {
        if ((res = isbd_write_tx_buf(dev, iolist->iol_base, iolist->iol_len)) < 0) {
            return res;
        }
        dev->_internal.tx_pending = true;
    }
    else {
        dev->_internal.rx_pending = true;
    }

    if ((res = isbd_start_tx(dev)) < 0) {
        return res;
    }

    return size;
}

static int _recv(netdev_t *netdev, void *buf, size_t len, void *info)
{
    (void)info;
    isbd_t *dev = (isbd_t *)netdev;

    uint16_t size = (dev->_internal.resp_buf[0] << 8) |
                    dev->_internal.resp_buf[1];

    if (buf == NULL) {
        return size;
    }

    if (size > len) {
        return -ENOBUFS;
    }

    uint16_t checksum = (dev->_internal.resp_buf[size + 2] << 8) |
                        (dev->_internal.resp_buf[size + 3]);
    uint16_t calc_checksum = 0;

    for (int i = 2; i < size + 2; i++) {
        calc_checksum += dev->_internal.resp_buf[i];
    }

    if (checksum != calc_checksum) {
        DEBUG("[isbd] netdev: received checksum: %d\n", checksum);
        DEBUG("[isbd] netdev: calculated checksum: %d\n", calc_checksum);
        netdev->event_callback(netdev, NETDEV_EVENT_CRC_ERROR);
    }

    memcpy(buf, dev->_internal.resp_buf + 2, size);
    isbd_clear_buffer(dev, ISBD_CLEAR_RX);
//    isbd_set_state(dev, ISBD_STATE_IDLE);

    return 0;
}

static int _init(netdev_t *netdev)
{
    isbd_t *dev = (isbd_t *)netdev;

    /* Launch initialization of driver and device */
    if (isbd_init(dev) != ISBD_OK) {
        DEBUG("[isbd] netdev: init failed.\n");
        return -1;
    }

    DEBUG("[isbd] netdev: init done.\n");

    return 0;
}

static void _isr(netdev_t *netdev)
{
    isbd_t *dev = (isbd_t *)netdev;
    uint8_t irq = dev->irq;

    dev->irq = 0;

    switch (irq) {
        case ISBD_IRQ_NETWORK_AVAILABLE:
            _on_network_avail_irq(dev);
            break;
        case ISBD_IRQ_NEW_MSG:
            _on_new_msg_irq(dev);
            break;
        case ISBD_IRQ_TX_RETRY:
        	DEBUG("[isbd] netdev send retry, retries: %d\n", dev->_internal.tx_retries);
            if (dev->_internal.tx_pending == true || dev->_internal.rx_pending == true) {
                isbd_start_tx(dev);
            }
            else {
                isbd_set_idle(dev);
            }
            break;
        case ISBD_IRQ_TX_TIMEOUT:
            isbd_set_state(dev, ISBD_STATE_STANDBY);
            dev->_internal.tx_retries = 0;
            dev->_internal.tx_pending = false;
            dev->_internal.rx_pending = false; //vlt nicht?
            netdev->event_callback(netdev, NETDEV_EVENT_TX_TIMEOUT);
            break;
        default:
            break;
    }
}

static int _get(netdev_t *netdev, netopt_t opt, void *val, size_t max_len)
{
    (void)max_len;     /* unused when compiled without debug, assert empty */
    isbd_t *dev = (isbd_t *)netdev;

    if (dev == NULL) {
        return -ENODEV;
    }

    switch (opt) {
        case NETOPT_STATE:
            assert(max_len >= sizeof(netopt_state_t));
            return _get_state(dev, val);

        default:
            break;
    }

    return -ENOTSUP;
}

static int _set(netdev_t *netdev, netopt_t opt, const void *val, size_t len)
{
    (void)len;     /* unused when compiled without debug, assert empty */

    isbd_t *dev = (isbd_t *)netdev;
    int res = -ENOTSUP;

    if (dev == NULL) {
        return -ENODEV;
    }

    switch (opt) {
        case NETOPT_STATE:
            assert(len <= sizeof(netopt_state_t));
            return _set_state(dev, *((const netopt_state_t *)val));
        default:
            break;
    }

    return res;
}

static int _set_state(isbd_t *dev, netopt_state_t state)
{
    switch (state) {
        case NETOPT_STATE_OFF:
            isbd_set_off(dev);
            break;

        case NETOPT_STATE_STANDBY:
            isbd_set_standby(dev);
            break;

        case NETOPT_STATE_IDLE:
        	isbd_set_idle(dev);
            break;
        default:
            return -ENOTSUP;
    }
    return sizeof(netopt_state_t);
}

static int _get_state(isbd_t *dev, void *val)
{
    uint8_t isbd_state = isbd_get_state(dev);
    netopt_state_t state = NETOPT_STATE_OFF;

    switch (isbd_state) {
        case ISBD_STATE_OFF:
            state = NETOPT_STATE_OFF;
            break;

        case ISBD_STATE_IDLE:
            state = NETOPT_STATE_IDLE;
            break;

        case ISBD_STATE_STANDBY:
            state = NETOPT_STATE_STANDBY;
            break;

        case ISBD_STATE_TX:
            state = NETOPT_STATE_TX;
            break;

        case ISBD_STATE_RX:
            state = NETOPT_STATE_RX;
            break;

        default:
            break;
    }
    memcpy(val, &state, sizeof(netopt_state_t));
    return sizeof(netopt_state_t);
}

void _on_network_avail_irq(void *arg)
{
    isbd_t *dev = (isbd_t *)arg;
    netdev_t *netdev = (netdev_t *)arg;

    //check if this could be a problem during a transmission, where the
    //network avail pin goes low and then immediately high again. Transmission
    //should be blocking anyway and isr will be called after a failed transmission
    //but who knows

    bool signal_avail;

    puts("net avail");

    if (gpio_read(dev->params.network_avail_pin) > 0) {
        netdev->event_callback(netdev, NETDEV_EVENT_LINK_UP);
        signal_avail = true;
    }
    else {
        netdev->event_callback(netdev, NETDEV_EVENT_LINK_DOWN);
        signal_avail = false;
    }

    if ((isbd_get_state(dev) == ISBD_STATE_TX) && signal_avail && (dev->_internal.is_sending == false)) {
        xtimer_remove(&dev->_internal.timeout_timer);
        dev->_internal.tx_retries++;
        DEBUG("[isbd] network available, start send, retries: %d\n", dev->_internal.tx_retries);
        if (dev->_internal.tx_pending == true || dev->_internal.rx_pending == true) {
            isbd_start_tx(dev);
        }
        else {
            isbd_set_idle(dev);
        }
    }
}

void _on_new_msg_irq(void *arg)
{
    DEBUG("[isbd] netdev: ALERT - new msg!\n");

    netdev_t *dev = (netdev_t *)arg;

    /* we just want to receive the queued rx msg, so send with no payload */
    dev->driver->send(dev, NULL);
}

const netdev_driver_t isbd_driver = {
    .send = _send,
    .recv = _recv,
    .init = _init,
    .isr = _isr,
    .get = _get,
    .set = _set,
};
