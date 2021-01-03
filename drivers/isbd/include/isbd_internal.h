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
 * @brief       Iridium SBD internal functions
 *
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 */

#ifndef ISBD_INTERNAL_H
#define ISBD_INTERNAL_H

#include "isbd.h"
#include "isbd_internal.h"
#include "isbd_params.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Writes a binary payload to the internal Mobile Originated (TX)
 *          buffer of the Transceiver (AT+SBDWB=\<length\>).
 *          The payload size must be > 0 and <= @ref ISBD_MAX_TX_LEN
 *
 * @param[in] dev           ISBD device descriptor
 * @param[in] payload       Payload to transmit
 * @param[in] payload_len   Length of the payload
 *
 * @return                  ISBD_OK on success
 * @return                  ISBD_ERR_AT when AT command failed
 * @return                  ISBD_ERR_SBDWB_TIMEOUT insufficient bytes transfered
 * @return                  ISBD_ERR_SBDWB_CHKSM wrong checksum transfered
 * @return                  ISBD_ERR_SBDWB_INCORRECT_LEN message size is not correct
 */
int isbd_write_tx_buf(isbd_t *dev, uint8_t *payload, uint16_t payload_len);

/**
 * @brief   Reads a binary message from the internal Mobile Terminated (RX)
 *          buffer of the transceiver (AT+SBDRB).
 *
 * @param[in] dev           ISBD device descriptor
 *
 * @return                  ISBD_OK on success
 */
int isbd_read_rx_buf(isbd_t *dev);

/**
 * @brief   Starts an Extended SBD Session (AT+SBDIX) to transmit the payload
 *          (if any) of the TX buffer to the Iridium Gateway. If any RX
 *          messages are queued at the Iridium Gateway, the first queued message
 *          (oldest timestamp) gets transfered as a payload in the response and
 *          is stored in the RX buffer of the transceiver.
 *
 *          If the SBDIX is initiated as a response to a ring alert, the AT+SBDIXA
 *          command will be send instead of AT+SBDIX to acknowledge the ring alert
 *          and download the incoming message into the MT buffer.
 *
 *          Command response:
 *          +SBDIX:\<MO status\>,\<MOMSN\>,\<MT status\>,\<MTMSN\>,\<MT length\>,\<MT queued\>
 *
 * @param[in] dev          The device descriptor
 *
 * @return                 ISBD_OK on success
 */
int isbd_tx(isbd_t *dev);

/**
 * @brief   Copies the Mobile Originated (TX) buffer to the Mobile Terminated (RX)
 *          buffer with the AT+SBDTC command. Used in @ref ISBD_TEST_MODE instead
 *          of AT+SBDIX.
 *
 * @param[in] dev          The device descriptor
 *
 * @return                 ISBD_OK on success
 */
int isbd_tx_test(isbd_t *dev);

/**
 * @brief   Clears the internal modem buffers of the transceiver based on the
 *          provided option (AT+SBDDn with n=0-2)
 *
 * @param[in] dev          The device descriptor
 * @param[in] option       @ref isbd_clear_buf_opt_t
 *
 * @return                 ISBD_OK on success
 */
int isbd_clear_buffer(isbd_t *dev, isbd_clear_buf_opt_t option);

/**
 * @brief   Wakes the ISBD device from sleep mode
 *
 * @param[in] dev           ISBD device descriptor
 *
 * @return                  ISBD_OK on success
 * @return                  ISBD_ERR_GPIO_UNDEF if sleep pin is undefined
 * @return                  ISBD_ERR_WAKE_TIMEOUT if UART communication failed
 */
int isbd_on(isbd_t *dev);

/**
 * @brief   Flush all pending writes to Eeprom, shut down the radio, and prepare
 *          the Data Module to be powered down.
 *
 * @param[in] dev           ISBD device descriptor
 *
 * @return                  ISBD_OK on success
 * @return                  ISBD_ERR_AT when AT command failed
 */
int isbd_flush_eeprom(isbd_t *dev);

/**
 * @brief   Query the latest Iridium system time received from the network.
 *
 * According to Iridium 9602 Product Bulletin of 7 May 2013, to overcome a
 * system erratum:
 *
 * "Before attempting any of the following commands: +SBDDET, +SBDREG, +SBDI,
 * +SBDIX, +SBDIXA the field application should issue the AT command AT-MSSTM
 * to the transceiver and evaluate the response to determine if it is valid or not:
 *
 * Valid Response: "-MSSTM: XXXXXXXX" where XXXXXXXX is an eight-digit hexadecimal number.
 *
 * Invalid Response: "-MSSTM: no network service"
 *
 * If the response is invalid, the field application should wait and recheck
 * system time until a valid response is obtained before proceeding.
 *
 * This will ensure that the Iridium SBD transceiver has received a valid system
 * time before attempting SBD communication. The Iridium SBD transceiver will
 * receive the valid system time from the Iridium network when it has a good
 * link to the satellite. Ensuring that the received signal strength reported
 * in response to AT command +CSQ and +CIER is above 2-3 bars before attempting
 * SBD communication will protect against lockout."
 *
 * @param[in] dev           ISBD device descriptor
 *
 * @return                  ISBD_OK on success
 * @return                  ISBD_ERR_AT when AT command failed
 */
int isbd_request_sys_time(isbd_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* ISBD_INTERNAL_H */
/** @} */
