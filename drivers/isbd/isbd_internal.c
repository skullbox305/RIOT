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
 * @brief       Implementation of internal functions for the Iridium SBD
 *
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 * @}
 */

#include "isbd.h"
#include "isbd_internal.h"
#include "isbd_params.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

static int _process_sbdix_tx_response(isbd_t *dev)
{
    int result = -100;
    int mo_status = atoi(strtok(dev->_internal.resp_buf + 8, ","));

    switch (mo_status) {
    case 0:
        result = ISBD_OK;
        DEBUG("[isbd] tx: Successfully transferred.\n");
        break;
    case 1:
        result = ISBD_ERR_SBDIX_MT_OVERSZIZE;
//        result = ISBD_OK;
        DEBUG(
            "[isbd] tx: Successfully transferred, but MT msg to big.\n");
        break;
    case 2:
        result = ISBD_ERR_SBDIX_LOCATION;
//        result = ISBD_OK;
        DEBUG(
            "[isbd] tx: Successfully transferred, but Location Update was not accepted\n");
        break;
    case 10:
        result = ISBD_ERR_SBDIX_GSS_TIMEOUT;
        DEBUG(
            "[isbd] tx err: transmission did not complete in the allowed time.\n");
        break;
    case 11:
        result = ISBD_ERR_SBDIX_MO_QUEUE_FULL;
        DEBUG("[isbd] tx err: MO message queue at the GSS is full.\n");
        break;
    case 12:
        result = ISBD_ERR_SBDIX_MO_SEGMENTS;
        DEBUG("[isbd] tx err: MO message has too many segments.\n");
        break;
    case 13:
        result = ISBD_ERR_SBDIX_INCOMPLETE_SESS;
        DEBUG(
            "[isbd] tx err: GSS reported that the session did not complete.\n");
        break;
    case 14:
        result = ISBD_ERR_SBDIX_INVALID_SEG_SIZ;
        DEBUG("[isbd] tx err: Invalid segment size.\n");
        break;
    case 15:
        result = ISBD_ERR_SBDIX_ACCESS_DENIED;
        DEBUG("[isbd] tx err: Access is denied.\n");
        break;
    case 16:
        result = ISBD_ERR_SBDIX_LOCKED;
        DEBUG("[isbd] tx err: ISU locked the transmission.\n");
        break;
    case 17:
        result = ISBD_ERR_SBDIX_TIMEOUT;
        DEBUG("[isbd] tx err: Gateway not responding.\n");
        break;
    case 18:
        result = ISBD_ERR_SBDIX_RF_DROP;
        DEBUG("[isbd] tx err: Connection lost (RF drop).\n");
        break;
    case 19:
        result = ISBD_ERR_SBDIX_LINK_FAILURE;
        DEBUG("[isbd]] tx err Link failure.\n");
        break;
    case 32:
        result = ISBD_ERR_NO_NETWORK;
        DEBUG("[isbd] tx err: No network service.\n");
        break;
    case 33:
        result = ISBD_ERR_SBDIX_ANTENNA_FAULT;
        DEBUG("[isbd] tx err: Antenna fault.\n");
        break;
    case 34:
        result = ISBD_ERR_SBDIX_RADIO_DISABLED;
        DEBUG("[isbd] tx err: Radio is disabled.\n");
        break;
    case 35:
        result = ISBD_ERR_SBDIX_BUSY;
        DEBUG("[isbd] tx err: ISU is busy, unable to initiate call.\n");
        break;
    case 36:
        result = ISBD_ERR_SBDIX_WAIT;
        DEBUG(
            "[isbd] tx err: Try later, must wait 3 minutes since last registration.\n");
        break;
    case 37:
        result = ISBD_ERR_SBDIX_SBD_SERVICE_OFF;
        DEBUG("[isbd] tx err: SBD service is temporarily disabled.\n");
        break;
    case 38:
        result = ISBD_ERR_SBDIX_TRAFFIC_MANAG;
        DEBUG("[isbd] tx err: Try later, traffic management period.\n");
        break;
    case 64:
        result = ISBD_ERR_SBDIX_BAND_VIOLATION;
        DEBUG("[isbd] tx err: Band violation.\n");
        break;
    case 65:
        result = ISBD_ERR_SBDIX_PLL_LOCK_FAIL;
        DEBUG("[isbd] tx err: PLL lock failure.\n");
        break;
    default:
        result = ISBD_ERR_AT;
        DEBUG("[isbd] tx err: Command failed, unknowm error code: %d\n",
              mo_status);
        break;
    }

    return result;
}

static int _process_sbdix_rx_response(isbd_t *dev)
{
    int i = 0;
    int mt_status = 0;
    int mt_length = 0;
    char *response_code;
    int res = ISBD_OK;

    /* strtok is already initialized in _process_sbdix_tx_response() */
    response_code = strtok(NULL, ",");

    while (response_code != NULL) {
        if (i == 1) {
            mt_status = atoi(response_code);
        }
        if (i == 3) {
            mt_length = atoi(response_code);
        }
        if (i == 4) {
            dev->_internal.rx_queued = atoi(response_code);
        }
        response_code = strtok(NULL, ",");
        i++;
    }

    if (mt_status == 1 && mt_length > 0) {
        DEBUG("[isbd] rx: Message received. Still queued: %d\n",
              dev->_internal.rx_queued);
    }
    else if (mt_status == 2) {
        DEBUG("[isbd] rx err: Error on mailbox check or message reception!\n");
        res = ISBD_ERR_SBDIX_RX_FAILED;
    }

    return res;
}

int isbd_tx(isbd_t *dev)
{
    int res;
    netdev_t *netdev = (netdev_t *)dev;
    isbd_states_t state = isbd_get_state(dev);

    if ((state == ISBD_STATE_STANDBY) || (state == ISBD_STATE_IDLE)) {
        netdev->event_callback(netdev, NETDEV_EVENT_TX_STARTED);
        isbd_set_state(dev, ISBD_STATE_TX);
    }

    /* Flag to prevent the "network avail" interrupt to wrongly start a
     * new transmission, as long as this tranmission is not done */
    dev->_internal.is_sending = true;

#ifdef CONFIG_ISBD_MSSTM_BEFORE_TX
    /* blocking, till system time was received from the network */
    if (isbd_request_sys_time(dev) < 0) {
        return ISBD_ERR_NO_NETWORK;
    }
#endif

    char cmd[10];

    /* if alert flag is set, answer it with +sbdixa, instead of +sbdix */
    if (dev->_internal.ring_alert_flag) {
        snprintf(cmd, sizeof(cmd), ISBD_SBDIXA);
    }
    else {
        snprintf(cmd, sizeof(cmd) - 1, ISBD_SBDIX);
    }

    /* start transmission */
    if (at_send_cmd_get_resp(&dev->at_dev, cmd, dev->_internal.resp_buf,
                             sizeof(dev->_internal.resp_buf),
                             (CONFIG_ISBD_SBD_SESSION_TIMEOUT + 5) *
                             US_PER_SEC) < 0) {
        DEBUG("\nSBDIX timeout\n");
        return ISBD_ERR_AT;
    }

    res = _process_sbdix_tx_response(dev);

    if ((res == ISBD_OK) ||
        (res == ISBD_ERR_SBDIX_MT_OVERSZIZE) ||
        (res == ISBD_ERR_SBDIX_LOCATION)) {
        dev->_internal.tx_retries = 0;

        if (dev->_internal.tx_pending == true) {
            dev->_internal.tx_pending = false;
        }
        else {
            dev->_internal.rx_pending = false;
        }

        dev->_internal.ring_alert_flag = false;
    }

    /* check if a message was received in the response of the transmission */
    if (_process_sbdix_rx_response(dev) == ISBD_OK) {
        isbd_set_state(dev, ISBD_STATE_RX);
        isbd_read_rx_buf(dev);
        dev->_internal.rx_pending = false;
        netdev->event_callback(netdev, NETDEV_EVENT_RX_COMPLETE);
    }

    /* transmission succeeded */
    if (dev->_internal.tx_pending == false &&
        dev->_internal.rx_pending == false) {

        isbd_clear_buffer(dev, ISBD_CLEAR_TX);

        /* set to idle state, because a successful transmission also automatically
         * performs a network registration.
         * If the location update failed, the modem is not registered at the
         * gateway. So the resulting state will be standby. */
        if (res == ISBD_ERR_SBDIX_LOCATION) {
            isbd_set_state(dev, ISBD_STATE_STANDBY);
        }
        else {
            isbd_set_state(dev, ISBD_STATE_IDLE);
        }
        dev->_internal.is_sending = false;

        /* We drop the previous error, if any, from "_process_sbdix_tx_response"
         * in case it was an rx only transmission (netdev->send with payload 0),
         * which still successfully received a message. Therefore no need to handle
         * the error and resend.
         */
        res = ISBD_OK;
        netdev->event_callback(netdev, NETDEV_EVENT_TX_COMPLETE);
    }
    return res;
}

#ifdef CONFIG_ISBD_TEST_MODE
int isbd_tx_test(isbd_t *dev)
{
    if (isbd_get_state(dev) == ISBD_STATE_OFF) {
        DEBUG("[isbd] ABORT: Device is in sleep mode\n");
        return ISBD_ERR_SLEEP_MODE;
    }

    netdev_t *netdev = (netdev_t *)dev;

    isbd_set_state(dev, ISBD_STATE_TX);
    at_send_cmd_get_resp(&dev->at_dev, ISBD_SBDTC, dev->_internal.resp_buf,
                         sizeof(dev->_internal.resp_buf), 2 * US_PER_SEC);

    int len = atoi(dev->_internal.resp_buf + 50);

    if (!(len > 0)) {
        DEBUG("[isbd] tx test: copy tx to rx buffer failed\n");
        return ISBD_ERR_AT;
    }

    netdev->event_callback(netdev, NETDEV_EVENT_TX_COMPLETE);
    isbd_clear_buffer(dev, ISBD_CLEAR_TX);
    isbd_set_state(dev, ISBD_STATE_RX);
    isbd_read_rx_buf(dev);
    netdev->event_callback(netdev, NETDEV_EVENT_RX_COMPLETE);
    isbd_set_state(dev, ISBD_STATE_STANDBY);

    return ISBD_OK;
}
#endif

static int _process_sbdwb_response(isbd_t *dev)
{
    uint8_t res = ISBD_OK;
    ssize_t len =
        at_recv_bytes(&dev->at_dev, dev->_internal.resp_buf, 3,
                      61 * US_PER_SEC);

    if (len == 0) {
        res = ISBD_ERR_AT;
        DEBUG("[isbd] Write TX buf: Device didn't answer after writing bytes\n");
    }
    else if (dev->_internal.resp_buf[2] == '1') {
        res = ISBD_ERR_SBDWB_TIMEOUT;
        DEBUG(
            "[isbd] Write TX buf: Timeout - insufficient number of bytes transfered\n");
    }
    else if (dev->_internal.resp_buf[2] == '2') {
        res = ISBD_ERR_SBDWB_CHKSM;
        DEBUG("[isbd] Write TX buf: Wrong checksum transfered\n");
    }
    else if (dev->_internal.resp_buf[2] == '3') {
        res = ISBD_ERR_SBDWB_INCORRECT_LEN;
        DEBUG("[isbd] Write TX buf: message size not correct.\n");
    }

    return res;
}

static int _append_payload(isbd_t *dev, uint8_t *payload, uint16_t *payload_len)
{
    char cur_char;
    uint16_t checksum = 0;

    /* Append payload and calculate checksum on the fly */
    for (int i = 0; i < *payload_len + 3; i++) {
        if (i < *payload_len) {
            checksum += payload[i];
            cur_char = payload[i];
        }
        else if (i == *payload_len) {
            cur_char = checksum >> 8;
        }
        else if (i == *payload_len + 1) {
            cur_char = checksum & 0xFF;
        }
        else if (i == *payload_len + 2) {
            cur_char = '\r';
        }
        at_send_bytes(&dev->at_dev, &cur_char, 1);
    }

    return _process_sbdwb_response(dev);
}

int isbd_write_tx_buf(isbd_t *dev, uint8_t *payload, uint16_t payload_len)
{
    if (payload_len == 0 || payload_len > CONFIG_ISBD_MAX_TX_LEN) {
        DEBUG("[isbd] write tx: Payload length should be > 0 and <= %d - "
              "but is: %d Bytes\n", CONFIG_ISBD_MAX_TX_LEN, payload_len);
        return ISBD_ERR_SBDWB_INCORRECT_LEN;
    }

    char command[14];
    uint8_t old_state = isbd_get_state(dev);

    snprintf(command, sizeof(command) - 1, ISBD_SBDWB, payload_len);

    /* Initate a binary write */
    at_send_cmd_get_resp(&dev->at_dev, command, dev->_internal.resp_buf,
                         sizeof(dev->_internal.resp_buf), 10 * US_PER_SEC);

    if (strcmp(dev->_internal.resp_buf, "READY") != 0) {
        DEBUG("[isbd] write tx: Initiating write to TX buf failed\n");
        isbd_set_state(dev, old_state);
        return ISBD_ERR_AT;
    }

    return _append_payload(dev, payload, &payload_len);
}

int isbd_clear_buffer(isbd_t *dev, isbd_clear_buf_opt_t option)
{
    DEBUG("[isbd] clearing %s buffer\n", option ? "RX" : "TX");

    char command[10];
    snprintf(command, sizeof(command) - 1, ISBD_SBDD, option);

    at_send_cmd_get_resp(&dev->at_dev, command, dev->_internal.resp_buf,
                         sizeof(dev->_internal.resp_buf), 3 * US_PER_SEC);

    if (dev->_internal.resp_buf[0] != '0') {
        DEBUG("[isbd] clear: failed clearing buffer with option: %d\n", option);
        return ISBD_ERR_AT;
    }

    return ISBD_OK;
}

int isbd_read_rx_buf(isbd_t *dev)
{
    int res;
    size_t cmdlen = strlen(ISBD_SBDRB);
    size_t len = sizeof(dev->_internal.resp_buf);

    at_drain(&dev->at_dev);

    /* special case: echo will not have AT_RECV_EOL_1 and AT_RECV_EOL_2,
     * so do command send manually */
    uart_write(dev->at_dev.uart, (const uint8_t *)ISBD_SBDRB, cmdlen);
    uart_write(dev->at_dev.uart, (const uint8_t *)CONFIG_AT_SEND_EOL,
               AT_SEND_EOL_LEN);


    if (at_expect_bytes(&dev->at_dev, ISBD_SBDRB, 2 * US_PER_SEC)) {
        return -1;
    }

    if (at_expect_bytes(&dev->at_dev, CONFIG_AT_SEND_EOL, 2 * US_PER_SEC)) {
        return -2;
    }

    memset(dev->_internal.resp_buf, 0, sizeof(dev->_internal.resp_buf));

    res = at_recv_bytes_until_string(&dev->at_dev, "\r\n\0",
                                     dev->_internal.resp_buf, &len,
                                     10 * US_PER_SEC);
    if (res < 0) {
        DEBUG("[isbd] read rx buf failed");
        return -1;
    }

    return ISBD_OK;
}

int isbd_on(isbd_t *dev)
{
    if (dev->params.sleep_pin == GPIO_UNDEF) {
        DEBUG("[isbd] on: No sleep pin defined.\n");
        return ISBD_ERR_GPIO_UNDEF;
    }

    /* already awake */
    if (isbd_get_state(dev) != ISBD_STATE_OFF) {
        DEBUG("[isbd] on: Device already awake\n");
        return ISBD_OK;
    }

    gpio_set(dev->params.sleep_pin);
    at_dev_poweron(&dev->at_dev);

    /*  Best practice suggests waiting at least 2 sec to turn off again, so
     *  save timestamp of power on */
    dev->_internal.power_on_time = xtimer_now();

    /* Waiting for the ISBD modem to respond to a simple 'AT' command after
     * wake up. The internal super capacitors need time to charge.
     * Depending on the power supply this can take a couple seconds
     * (especially if they are completely depleted on a cold start). */
    while ((isbd_get_state(dev) != ISBD_STATE_STANDBY) &&
           xtimer_less(xtimer_diff(xtimer_now(), dev->_internal.power_on_time),
                       xtimer_ticks_from_usec(CONFIG_ISBD_STARTUP_TIME
                                              * US_PER_SEC))) {

        if (at_send_cmd_wait_ok(&dev->at_dev, "AT", 1 * US_PER_SEC) == 0) {
            isbd_set_state(dev, ISBD_STATE_STANDBY);
            return ISBD_OK;
        }
    }
    DEBUG("[isbd] on: dev didn't respond. Increase startup max time!\n");

    return ISBD_ERR_WAKE_TIMEOUT;
}

int isbd_flush_eeprom(isbd_t *dev)
{
    if (at_send_cmd_wait_ok(&dev->at_dev, ISBD_FLUSH_EEPROM,
                            10 * US_PER_SEC) < 0) {
        DEBUG("[isbd] flush: Flushing EEPROM failed\n");
        return ISBD_ERR_AT;
    }
    return ISBD_OK;
}

int isbd_request_sys_time(isbd_t *dev)
{
    uint8_t retries;
    int res = ISBD_OK;

    for (retries = 0; retries < CONFIG_ISBD_MSSTM_RETRIES; retries++) {

        if (at_send_cmd_get_resp(&dev->at_dev, ISBD_MSSTM,
                                 dev->_internal.resp_buf,
                                 sizeof(dev->_internal.resp_buf),
								 CONFIG_ISBD_MSSTM_RETRY_INTERVAL * US_PER_SEC) <
            0) {
            DEBUG("[isbd] msstm: System time request failed\n");
            res = ISBD_ERR_AT;
            break;
        }

        if (strcmp(dev->_internal.resp_buf,
                   "-MSSTM: no network service") == 0) {
            if (retries == (CONFIG_ISBD_MSSTM_RETRIES - 1)) {
                return ISBD_ERR_NO_NETWORK;
            }

            DEBUG("[isbd] sys time: %s, waiting %d sec\n",
                  dev->_internal.resp_buf, CONFIG_ISBD_MSSTM_RETRY_INTERVAL);
            xtimer_sleep(CONFIG_ISBD_MSSTM_RETRY_INTERVAL);
        }
        else {
            DEBUG("[isbd] sys time: %s\n", dev->_internal.resp_buf);
            break;
        }
    }

    return res;
}
