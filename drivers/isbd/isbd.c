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
 * @brief       Basic functionality of the Iridium SBD driver
 *
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 * @}
 */

#include "thread.h"

#include "isbd.h"
#include "isbd_internal.h"
#include "isbd_netdev.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

/* Internal functions */
static int _init_at_dev(isbd_t *dev);
static int _init_gpios(isbd_t *dev);

static void _start_timeout_timer(isbd_t *dev, uint32_t seconds,
                                 isbd_timer_cb_t cb);

/* ISBD interrupt handlers initialization */
static void isbd_on_isr(isbd_t *dev, isbd_flags_t flag);
static void isbd_tx_resend_isr(void *arg);
static void isbd_tx_timeout_isr(void *arg);
static void isbd_network_avail_isr(void *arg);
static void isbd_new_msg_isr(void *arg);
//static void isbd_net_register_isr(void *arg);

void isbd_setup(isbd_t *dev, const isbd_params_t *params)
{
    assert(dev && (params->uart < UART_NUMOF));

    netdev_t *netdev = (netdev_t *)dev;
    netdev->driver = &isbd_driver;
    dev->params = *params;
}

int isbd_init(isbd_t *dev)
{
    int res;

    /* Initialize AT device */
    if ((res = _init_at_dev(dev)) != ISBD_OK) {
        return res;
    }


    /* Initialize GPIO pins */
    if (_init_gpios(dev) < 0) {
        DEBUG("[isbd] init: failed to initialize GPIOs\n");
        return ISBD_ERR_GPIO;
    }

    /* Put device in standby mode (wake up) and check if it responds with OK
     * to the 'AT' command */
    if (isbd_on(dev) == ISBD_ERR_WAKE_TIMEOUT) {
        return ISBD_ERR_WAKE_TIMEOUT;
    }

    /* Check if device is an Iridium Transceiver */
    at_send_cmd_get_resp(&dev->at_dev, "at+cgmi", dev->_internal.resp_buf,
                         sizeof(dev->_internal.resp_buf), 10 * US_PER_SEC);

    if (strcmp(dev->_internal.resp_buf, "Iridium") != 0) {
        DEBUG("[isbd] init: not an Iridium device\n");
        return ISBD_ERR_WRONG_DEV;
    }

    /* Enables the AT command echo */
    isbd_set_at_echo(dev);

    /* Both below in case of 3-wire UART (RX,TX,GND) */
    /* Disable RTS/CTS flow control */
    isbd_set_flow_control_off(dev);
    /* Disable Data Terminal Ready control signal */
    isbd_set_dtr_off(dev);

//    isbd_set_sbd_session_timeout(dev, CONFIG_ISBD_SBD_SESSION_TIMEOUT);
//    isbd_set_ring_alert(dev, true);

    /* Initialize the TX timer */
    dev->_internal.tx_retries = 0;
    dev->_internal.timeout_timer.arg = dev;
    dev->irq = 0;

    /* Initialize internal flags */
    dev->_internal.ring_alert_flag = false;
    dev->_internal.rx_pending = false;
    dev->_internal.tx_pending = false;
    dev->_internal.rx_received = false;

    dev->_internal.is_sending = false;

    return ISBD_OK;
}

int isbd_start_tx(isbd_t *dev)
{
    int res;

#if IS_ACTIVE(CONFIG_ISBD_TEST_MODE)
    res = isbd_tx_test(dev);
#else
    if ((res = isbd_tx(dev)) < 0) {
        dev->_internal.is_sending = false;
        if (dev->_internal.tx_retries < CONFIG_ISBD_SBDIX_RETRIES) {

            /* error not due to no network, so start timer and try again in isr.
             * Also check if network avail pin is really high = no signal.
             * If network is still available, then try again too. No need to wait
             * for signal then, even if the error was "no network" */
            if (res != ISBD_ERR_NO_NETWORK ||
                (res == ISBD_ERR_NO_NETWORK &&
                 gpio_read(dev->params.network_avail_pin) > 0)) {
                DEBUG(
                    "[isbd] sending failed with code: %d, trying again in %d sec...\n",
                    res, CONFIG_ISBD_TX_RETRY_INTERVAL);
                _start_timeout_timer(dev, CONFIG_ISBD_TX_RETRY_INTERVAL,
                                     isbd_tx_resend_isr);
            }
            /* no network available, start timeout timer so it won't wait
             * indefinitely for a signal. */
            else {
                DEBUG("[isbd] waiting %d sec for network signal...\n",
                      CONFIG_ISBD_TIMEOUT_IF_NO_SIGNAL);
                _start_timeout_timer(dev, CONFIG_ISBD_TIMEOUT_IF_NO_SIGNAL,
                                     isbd_tx_timeout_isr);
            }
        }
        else {
            netdev_t *netdev = (netdev_t *)dev;
            isbd_set_state(dev, ISBD_STATE_STANDBY);
            netdev->event_callback(netdev, NETDEV_EVENT_TX_TIMEOUT);
            dev->_internal.tx_retries = 0;
            dev->_internal.tx_pending = false;
        }
    }
    else {
//      LED1_OFF;
    }
#endif
    return res;
}
#if IS_ACTIVE(CONFIG_ISBD_TEST_MODE)
int isbd_start_network_registration(isbd_t *dev)
{
    if (isbd_get_state(dev) == ISBD_STATE_OFF) {
        DEBUG("[isbd] ABORT: Device is in sleep mode\n");
        return ISBD_ERR_SLEEP_MODE;
    }

    uint8_t status = 0;
    uint8_t reg_err = 0;
    netdev_t *netdev = (netdev_t *)dev;
    isbd_set_flow_control_off(dev);

    if (dev->_internal.state != ISBD_STATE_TX) {
        netdev->event_callback(netdev, NETDEV_EVENT_TX_STARTED);
        isbd_set_state(dev, ISBD_STATE_TX);
    }

    if (at_send_cmd_get_resp(&dev->at_dev, "at+sbdreg", dev->_internal.resp_buf,
                             sizeof(dev->_internal.resp_buf),
                             60 * US_PER_SEC) < 0) {
        puts("reg fail - at err"); //<----- why?
        isbd_set_state(dev, ISBD_STATE_STANDBY);
        return ISBD_ERR_AT;
    }

    status = atoi(strtok(dev->_internal.resp_buf + 8, ","));
    reg_err = atoi(strtok(NULL, ","));

    // status == 1 and reg_err == 2 is apparently successful too, further tests needed */
    if (status != 2 || reg_err != 0) {

        if (reg_err == 32 && gpio_read(dev->params.network_avail_pin) == 0) {
            DEBUG("[isbd] waiting %d sec for network signal...\n",
                  CONFIG_ISBD_TIMEOUT_IF_NO_SIGNAL);

            _start_timeout_timer(dev, CONFIG_ISBD_TIMEOUT_IF_NO_SIGNAL,
                                 isbd_tx_timeout_isr);
            return ISBD_ERR_NO_NETWORK;
        }
        else if (reg_err == 36) {

            DEBUG("[isbd] need to wait 3 minutes till re-registration...\n");
            _start_timeout_timer(dev, 180, isbd_tx_resend_isr);
            return ISBD_ERR_SBDREG_TRY_LATER;
        }
        else {

            DEBUG(
                "[isbd] registration failed with code: %d, trying again in %d sec...\n", reg_err,
                CONFIG_ISBD_TX_RETRY_INTERVAL);
            _start_timeout_timer(dev, CONFIG_ISBD_TX_RETRY_INTERVAL,
                                 isbd_tx_resend_isr);
            return reg_err;
        }
    }
    isbd_set_state(dev, ISBD_STATE_IDLE);

    return ISBD_OK;
}
#endif

/**
 * IRQ handlers
 */
void isbd_isr(netdev_t *dev)
{
    if (dev->event_callback) {
        dev->event_callback(dev, NETDEV_EVENT_ISR);
    }
}

static void isbd_on_isr(isbd_t *dev, isbd_flags_t flag)
{
    dev->irq |= flag;
    isbd_isr((netdev_t *)dev);
}

static void isbd_tx_resend_isr(void *arg)
{
    isbd_t *dev = (isbd_t *)arg;

    dev->_internal.tx_retries++;
    isbd_on_isr(dev, ISBD_IRQ_TX_RETRY);
}

static void isbd_tx_timeout_isr(void *arg)
{
    isbd_on_isr((isbd_t *)arg, ISBD_IRQ_TX_TIMEOUT);
}

static void isbd_network_avail_isr(void *arg)
{
    isbd_t *dev = (isbd_t *)arg;

    if (isbd_get_state(dev) == ISBD_STATE_TX &&
        dev->_internal.is_sending == false) {
        puts("net");
        isbd_on_isr(dev, ISBD_IRQ_NETWORK_AVAILABLE);
    }
}

static void isbd_new_msg_isr(void *arg)
{
    (void)arg;
    isbd_t *dev = (isbd_t *)arg;

    /* Interrupt pin pulls low after wake up and pulls high immediately again.
     * Catch this case and return, as long as still in SLEEP state */
    if (isbd_get_state(dev) == ISBD_STATE_OFF) {
        return;
    }

    /* ring alert sends two rings. If the first is already being processed skip the second */
    if (dev->_internal.ring_alert_flag == true) {
        return;
    }

    if (isbd_get_state(dev) == ISBD_STATE_IDLE) {
        dev->_internal.ring_alert_flag = false;
        isbd_on_isr(dev, ISBD_IRQ_NEW_MSG);
    }
}

/**
 * Internal functions
 */
static int _init_at_dev(isbd_t *dev)
{
    /* Initialize the AT device */
    int res = at_dev_init(&dev->at_dev, dev->params.uart,
                          dev->params.baudrate, dev->_internal.uart_buf,
                          sizeof(dev->_internal.uart_buf));

    /* check the UART initialization return value */
    if (res == UART_NODEV) {
        DEBUG("[isbd] init: Invalid UART device given!");
        return UART_NODEV;
    }
    if (res == UART_NOBAUD) {
        DEBUG("[isbd] init: Baudrate is not applicable!");
        return UART_NOBAUD;
    }
    return ISBD_OK;
}

static int _init_gpios(isbd_t *dev)
{
    int res;

    /* Init sleep pin if defined*/
    if (dev->params.sleep_pin != GPIO_UNDEF) {
        isbd_set_state(dev, ISBD_STATE_OFF);
        res = gpio_init(dev->params.sleep_pin, GPIO_OUT);

        if (res < 0) {
            DEBUG("[isbd] init: failed to initialize sleep pin\n");
            return res;
        }
    }
    else {
        isbd_set_state(dev, ISBD_STATE_STANDBY);
    }

    /* Init network available pin if defined */
    if (dev->params.network_avail_pin != GPIO_UNDEF) {
        res = gpio_init_int(dev->params.network_avail_pin, GPIO_IN_PU,
                            GPIO_BOTH, isbd_network_avail_isr, dev);
        if (res < 0) {
            DEBUG("[isbd] init: failed to init network available pin\n");
            return res;
        }
    }

    /* Init ring pin if defined*/
    if (dev->params.ring_pin != GPIO_UNDEF) {
        res = gpio_init_int(dev->params.ring_pin, GPIO_IN_PD,
                            GPIO_FALLING, isbd_new_msg_isr, dev);
        if (res < 0) {
            DEBUG("[isbd] init: failed to init ring pin\n");
            return res;
        }
    }

    return res;
}


static void _start_timeout_timer(isbd_t *dev, uint32_t seconds,
                                 isbd_timer_cb_t cb)
{
    dev->_internal.timeout_timer.callback = cb;
    xtimer_set(&dev->_internal.timeout_timer,
               (uint32_t)seconds * US_PER_SEC);
}
