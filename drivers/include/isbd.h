/*
 * Copyright (C) 2020 Igor Knippenberg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_isbd Iridium Short Burst Data (ISBD) satellite transceiver driver
 * @ingroup     drivers_netdev
 * @brief       Device driver for Iridium SBD transceivers
 *
 * ISBD devices allow you to send and receive short messages (SBD)
 * from anywhere on Earth with a clear view of the sky.
 *
 * It works far beyond the reach of other radio networks.
 * Maybe you want to transmit weather information from mid-ocean?
 * Or use it to control your robot in the middle of the desert?
 * Perhaps you need to communicate in an emergency, when other networks might not be
 * available?
 *
 * It was tested with a 9603 modem, but should work with other ISBD device too.
 * @{
 *
 * @file
 * @brief       Public interface for the Iridium SBD driver
 *
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 */

#ifndef ISBD_H
#define ISBD_H

#include "xtimer.h"
#include "net/netdev.h"
#include "periph/gpio.h"
#include "periph/uart.h"
#include "at.h"
#include <string.h>
#include "stdlib.h"

/**
 * @name    ISBD device default configuration
 * @{
 */

/**
 * @brief   Maximum size of a TX payload supported by the modem
 */
#ifndef CONFIG_ISBD_MAX_TX_LEN
#define CONFIG_ISBD_MAX_TX_LEN              (340U)
#endif

/**
 * @brief   Maximum size of an ISBD response packet (RX).
 *          Usually 270 Bytes + 4 Bytes length and checksum.
 */
#ifndef CONFIG_ISBD_MAX_RX_LEN
#define CONFIG_ISBD_MAX_RX_LEN              (274U)
#endif

/**
 * @brief   Size of the UART buffer
 */
#ifndef CONFIG_ISBD_UART_BUF
#define CONFIG_ISBD_UART_BUF                (512U)
#endif

/**
 * @brief   Startup time in sec for the capacitors to charge after a
 *          cold start/wake up. If your power supply is sufficiently powerful
 *          the value can be decreased.
 */
#ifndef CONFIG_ISBD_STARTUP_TIME
#define CONFIG_ISBD_STARTUP_TIME            (10U)
#endif

/**
 * @brief   Timeout in sec for the response of the AT+SBDIX[A] command
 */
#ifndef CONFIG_ISBD_SBD_SESSION_TIMEOUT
#define CONFIG_ISBD_SBD_SESSION_TIMEOUT     (300U)
#endif

/**
 * @brief   Timeout in sec until a transmission with no network service
 *          gets canceled
 */
#ifndef CONFIG_ISBD_TIMEOUT_IF_NO_SIGNAL
#define CONFIG_ISBD_TIMEOUT_IF_NO_SIGNAL    (300U)
#endif

/**
 * @brief   Interval time between a failed (TX/RX) and a retry.
 *          Retrying to fast depletes the capacitors and if your VCC is not
 *          providing enough power, the device will stop working till the
 *          capacitors are sufficiently charged again. The value can be
 *          decreased if the power supply is able to provide enough burst current.
 */
#ifndef CONFIG_ISBD_TX_RETRY_INTERVAL
#define CONFIG_ISBD_TX_RETRY_INTERVAL       (10U)
#endif

/**
 * @brief   Number of MSSTM retries until the request is considered as failed
 */
#ifndef CONFIG_ISBD_MSSTM_RETRIES
#define CONFIG_ISBD_MSSTM_RETRIES           (3U)
#endif

/**
 * @brief   Interval time between system time requests (in case the system
 *          time was not received on the first attempt)
 */
#ifndef CONFIG_ISBD_MSSTM_RETRY_INTERVAL
#define CONFIG_ISBD_MSSTM_RETRY_INTERVAL    (10U)
#endif

/**
 * @brief   Number of SBDIX retries until a transmission is considered as failed
 */
#ifndef CONFIG_ISBD_SBDIX_RETRIES
#define CONFIG_ISBD_SBDIX_RETRIES           (3U)
#endif

/**
 * @brief   Time increments in seconds between SBDIX retries
 */
#ifndef CONFIG_ISBD_SBDIX_TIME_INCREMENT
#define CONFIG_ISBD_SBDIX_TIME_INCREMENT    (3U)
#endif
/** @} */

/**
 * @name    ISBD AT-Commands
 * @{
 */
/** @brief Initiate an SBD Session Extended */
#define ISBD_SBDIX                   ("at+sbdix")
/** @brief Same as SBDIX, but as answer to a ring alert) */
#define ISBD_SBDIXA                  ("at+sbdixa")
/** @brief Enable/Disable Mobile-Terminated Alert */
#define ISBD_SBDMTA                  ("at+sbdmta=%d")
/** @brief Session Timeout for SBDIX[A] and SBDREG commands */
#define ISBD_SBDST                   ("at+sbdst=%d")
/** @brief Write Binary Data to the modem MO buffer */
#define ISBD_SBDWB                   ("at+sbdwb=%d")
/** @brief Read Binary Data from the modem MT buffer */
#define ISBD_SBDRB                   ("at+sbdrb")
/** @brief TX test command: Transfer MO Buffer directly to MT Buffer */
#define ISBD_SBDTC                   ("at+sbdtc")
/** @brief Clear SBD Message Buffer(s) */
#define ISBD_SBDD                    ("at+sbdd%d")
/** @brief Get the device revision */
#define ISBD_CGMR                    ("at+cgmr")
/** @brief Store Active Configuration */
#define ISBD_STORE_CFG               ("at&w%d")
/** @brief Default Reset Profile after power-up */
#define ISBD_RST_PROFILE              ("at&y%d")
/** @brief Query the latest Iridium system time received from the network */
#define ISBD_MSSTM                   ("at-msstm")
/** @brief Flush the EEPROM */
#define ISBD_FLUSH_EEPROM            ("at*f")
/** @brief Disable Flow Control */
#define ISBD_FLOW_CTR_OFF            ("at&k0")
/** @brief Disable the Data Terminal Ready control signal */
#define ISBD_DTR_OFF                 ("at&d0")
/** @brief Enable the AT command echo.*/
#define ISBD_AT_ECHO_ON              ("ate%d")
/** @} */

/**
 * @name    ISBD IRQ flags
 * @{
 */
#define ISBD_IRQ_NETWORK_AVAILABLE      (1 << 0)    /**< Signal Available IRQ flag */
#define ISBD_IRQ_NEW_MSG                (1 << 1)    /**< Ring Alarm IRQ flag */
#define ISBD_IRQ_TX_RETRY               (1 << 2)    /**< TX Retry IRQ flag */
#define ISBD_IRQ_TX_TIMEOUT             (1 << 3)    /**< TX Timeout IRQ flag */
/** @} */

/**
 * @brief   Radio driver internal state machine states definition.
 */
typedef enum {
    ISBD_STATE_OFF,                 /**< Device is powered off */
    ISBD_STATE_STANDBY,             /**< Awake but not registered at the network.
                                         Waiting for AT commands */
    ISBD_STATE_IDLE,                /**< Registered at the network and will react to
                                         ring alerts and download the corresponding
                                         incoming message. Also waiting for
                                         incoming AT commands */
    ISBD_STATE_TX,                  /**< Device is transmitting */
    ISBD_STATE_RX                   /**< Received a data packet, waiting to be read */
} isbd_states_t;

/**
 * @brief   Named return values
 */
enum {
    ISBD_OK,                                /**< Everything was fine */
    ISBD_ERR_SLEEP_MODE             = -1,   /**< Device still in sleep mode */
    ISBD_ERR_AT                     = -2,   /**< AT command responded with ERROR
                                             *   or timed out */
    ISBD_ERR_NODEV                  = -3,   /**< Invalid UART device given */
    ISBD_ERR_NOBAUD                 = -4,   /**< Baudrate is not applicable */
    ISBD_ERR_WRONG_DEV              = -5,   /**< Not an Iridium device */
    ISBD_ERR_GPIO                   = -6,   /**< Failed to initialize GPIO */
    ISBD_ERR_GPIO_UNDEF             = -7,   /**< GPIO is @ref GPIO_UNDEF */
    ISBD_ERR_WAKE_TIMEOUT           = -8,   /**< Device didn't respond during
                                             *   @ref CONFIG_ISBD_STARTUP_TIME
                                             *   after exiting sleep mode */
    ISBD_ERR_SBDWB_TIMEOUT          = -9,   /**< Write Timeout: insufficient number
                                             *   of bytes were transfered. */
    ISBD_ERR_SBDWB_CHKSM            = -10,  /**< Transfered checksum does not match
                                             *   the internally calculated checksum */
    ISBD_ERR_SBDWB_INCORRECT_LEN    = -11,  /**< When Payload 0 or > @ref CONFIG_ISBD_MAX_TX_LEN */

    /* GSS (gateway)-reported errors: */
    ISBD_ERR_SBDIX_MT_OVERSZIZE     = -12,  /**< MO message, if any, transferred successfully,
                                             *   but the MT message in the queue was too
                                             *   big to be transferred. */
    ISBD_ERR_SBDIX_LOCATION         = -13,  /**< MO message, if any, transferred successfully,
                                             *   but the requested Location Update was not accepted. */
    ISBD_ERR_SBDIX_GSS_TIMEOUT      = -14,  /**< GSS reported that the call did
                                             *   not complete in the allowed time. */
    ISBD_ERR_SBDIX_MO_QUEUE_FULL    = -15,  /**< MO message queue at the GSS is full. */
    ISBD_ERR_SBDIX_MO_SEGMENTS      = -16,  /**< MO message has too many segments. */
    ISBD_ERR_SBDIX_INCOMPLETE_SESS  = -17,  /**< GSS reported that the session did not complete. */
    ISBD_ERR_SBDIX_INVALID_SEG_SIZ  = -18,  /**< Invalid segment size. */
    ISBD_ERR_SBDIX_ACCESS_DENIED    = -19,  /**< Access is denied. */

    /* ISU (transceiver)-reported errors: */
    ISBD_ERR_SBDIX_LOCKED           = -20,  /**< ISU has been locked and may not make SBD calls */
    ISBD_ERR_SBDIX_TIMEOUT          = -21,  /**< Gateway not responding (local session timeout). */
    ISBD_ERR_SBDIX_RF_DROP          = -22,  /**< Connection lost (RF drop). */
    ISBD_ERR_SBDIX_LINK_FAILURE     = -23,  /**< Link failure (A protocol error
                                             *   caused termination of the call). */
    ISBD_ERR_NO_NETWORK             = -24,  /**< No network service, unable to initiate call. */
    ISBD_ERR_SBDIX_ANTENNA_FAULT    = -25,  /**< Antenna fault, unable to initiate call. */
    ISBD_ERR_SBDIX_RADIO_DISABLED   = -26,  /**< Radio is disabled, unable to initiate call */
    ISBD_ERR_SBDIX_BUSY             = -27,  /**< ISU is busy, unable to initiate call. */
    ISBD_ERR_SBDIX_WAIT             = -28,  /**< Try later, must wait 3 minutes since last registration. */
    ISBD_ERR_SBDIX_SBD_SERVICE_OFF  = -29,  /**< SBD service is temporarily disabled. */
    ISBD_ERR_SBDIX_TRAFFIC_MANAG    = -30,  /**< Try later, traffic management period */
    ISBD_ERR_SBDIX_BAND_VIOLATION   = -31,  /**< Band violation (attempt to transmit outside permitted frequency band). */
    ISBD_ERR_SBDIX_PLL_LOCK_FAIL    = -32,  /**< PLL lock failure; hardware error during attempted transmit. */

    ISBD_ERR_SBDIX_RX_FAILED        = -33,  /**< An error occurred while attempting
                                             *   to perform a mailbox check or receive
                                             *   a message from the GSS. */
    ISBD_ERR_SBDREG_TRY_LATER       = -34,  /**< Try later, must wait 3 minutes
                                             *   since last registration. */
	ISBD_ERR_MSSTM_TIMEOUT          = -35,  /**< System time request timeout */
};

/**
 * @brief   ISBD clear buffer options
 */
typedef enum {
    ISBD_CLEAR_TX   = 0,    /**< Clear MO (TX) buffer */
    ISBD_CLEAR_RX   = 1,    /**< Clear MT (RX) buffer */
} isbd_clear_buf_opt_t;


/**
 * @brief   Iridium SBD internal data.
 */
typedef struct {
    uint8_t state;                          /**< Radio state */
    char uart_buf[CONFIG_ISBD_UART_BUF];    /**< UART buffer */
    char resp_buf[CONFIG_ISBD_MAX_RX_LEN];  /**< Response buffer */
    xtimer_ticks32_t power_on_time;         /**< Timestamp of when the device
                                             *   was turned on. */
    xtimer_t timeout_timer;                 /**< Timeout timer */
    bool ring_alert_flag;                   /**< Alert flag, indicating a new msg
                                                 is available to be received */
    bool tx_pending;                        /**< Transmission pending */
    bool rx_pending;                        /**< Receive pending */
    uint8_t rx_queued;                      /**< Amount of messages still queued
                                             *   in the Iridium gateway and ready
                                             *   to be downloaded */
    uint8_t tx_retries;                     /**< Number of attempts of retransmitting
                                             *   the TX buffer */

    bool is_sending;
} isbd_internal_t;

/**
 * @brief   Iridium SBD hardware and global parameters.
 */
typedef struct {
    uart_t uart;                            /**< UART device */
    uint32_t baudrate;                      /**< Baudrate to use */
    gpio_t sleep_pin;                       /**< Sleep pin (low = off) */
    gpio_t network_avail_pin;               /**< Network available pin
                                             *   (high = service available) */
    gpio_t ring_pin;                        /**< Ring Interrupt pin
                                             *   (low = new message available) */
} isbd_params_t;

/**
 * @brief   ISBD IRQ flags.
 */
typedef uint8_t isbd_flags_t;

/**
 * @brief   Iridium SBD device descriptor.
 * @extends netdev_t
 */
typedef struct {
    netdev_t netdev;                        /**< Netdev parent struct */
    at_dev_t at_dev;                        /**< AT device struct */
    isbd_params_t params;                   /**< Device driver parameters */
    isbd_internal_t _internal;              /**< Internal isbd data used within the driver */
    isbd_flags_t irq;                       /**< Device IRQ flags */
} isbd_t;

/**
 * @brief   ISBD timer callback
 */
typedef void (*isbd_timer_cb_t)(void *);

/**
 * @brief   Prepares the given ISBD  device
 *
 * @param[out] dev          ISBD device to initialize
 * @param[in]  params       parameters for device initialization
 * @param[in]  index        Index of @p params in a global parameter struct array.
 *                          If initialized manually, pass a unique identifier instead.
 */
void isbd_setup(isbd_t *dev, const isbd_params_t *params, uint8_t index);

/**
 * @brief   Initializes the isbd device
 *
 * @param[in] dev           ISBD device descriptor
 *
 * @return                  @ref ISBD_OK on success
 * @return                  @ref UART_NODEV or @ref UART_NOBAUD on UART error
 * @return                  @ref ISBD_ERR_GPIO if GPIO init failed
 * @return                  @ref ISBD_ERR_WAKE_TIMEOUT if wake up failed
 * @return                  @ref ISBD_ERR_WRONG_DEV if not an Iridium radio
 * @return                  @ref ISBD_ERR_AT if AT command failed
 */
int isbd_init(isbd_t *dev);

/**
 * @brief   Transmits the message (if any) of the internal TX buffer to the
 *          Iridium Gateway (needs an active subscription, otherwise use the
 *          @ref ISBD_TEST_MODE).
 *          If an incoming message is queued at the Iridium Gateway,
 *          it will be attached to the TX response and written to the internal
 *          RX buffer of the modem.
 *
 * @param[in] dev           ISBD device descriptor
 *
 * @return                  ISBD_OK on success
 */
int isbd_start_tx(isbd_t *dev);

#if defined(DOXYGEN) || !defined(CONFIG_ISBD_TEST_MODE)
/**
 * @brief   Performs a manual network registration in order to use the Ring Alert.
 *          After a successful registration, the ring pin will be pulled low
 *          if any incoming messages arrive at the gateway.
 *          This way no random mailbox checks need to be performed and therefore
 *          no unnecessary fees will be charged.
 *          Alerts will only be send for messages which were send after
 *          the device has registered. You wont get notified for messages which
 *          were queued before the registration. (AT+SBDREG)
 *
 * @param[in] dev          The device descriptor
 *
 * @return                 ISBD_OK on success
 */
int isbd_start_network_registration(isbd_t *dev);
#endif

/**
 * @brief   Gets the current state of the transceiver
 *
 * @param[in] dev            The ISBD device descriptor
 *
 * @return transceiver state [STATE_IDLE, STATE_SLEEP, ...]
 */
uint8_t isbd_get_state(const isbd_t *dev);

/**
 * @brief   Gets the firmware version (AT+CGMR)
 *
 * @param[in] dev            The ISBD device descriptor
 *
 * @return Firmware version
 */
uint8_t isbd_get_fw_version(const isbd_t *dev);

/**
 * @brief   Sets current state of the transceiver.
 *
 * @param[in] dev             The ISBD device descriptor
 * @param[in] state           The new radio state
 */
void isbd_set_state(isbd_t *dev, isbd_states_t state);

/**
 * @brief   Sets the device in off mode. If the @ref dev->params.sleep_pin is
 *          undefined, the off mode is unavailable
 *
 * @param[in] dev             The ISBD device descriptor
 */
void isbd_set_off(isbd_t *dev);

/**
 * @brief   Sets the device in stand-by mode. If the @ref dev->params.sleep_pin is
 *          undefined, the device is assumed to be ON by default
 *
 * @param[in] dev             The ISBD device descriptor
 */
void isbd_set_standby(isbd_t *dev);

/**
 * @brief   Sets the device in idle mode. This will perform a network registration
 *          and enabling the ring alarm pin, to listen to incoming messages.
 *          If the device is already in idle mode, a network re-regristration
 *          will be performed.
 *          If no @ref dev->params.ring_pin is defined, the idle mode is not
 *          available and the device will be set to stand-by mode.
 *
 * @param[in] dev             The ISBD device descriptor
 */
void isbd_set_idle(isbd_t *dev);

/**
 * @brief   Enables/disables the assertion of the ring alert pin of the radio.
 *          Incoming messages will pull the pin low (works only after device
 *          registered to the network).
 *
 *          @note Not saved after power off!
 *
 * @param[in] dev             The ISBD device descriptor
 * @param[in] enable          Enable/disable ring alert
 */
void isbd_set_ring_alert(isbd_t *dev, bool enable);

/**
 * @brief   Set the SBD session timeout in seconds for the +SBDIX, +SBDREG and
 *          +SBDDET commands. Default on power on is 0, which means a command
 *          can take infinite amount of seconds.
 *
 *          @note Not saved after power off!
 *
 * @param[in] dev             The ISBD device descriptor
 * @param[in] timeout_sec     Timeout in seconds
 */
void isbd_set_sbd_session_timeout(isbd_t *dev, uint16_t timeout_sec);

/**
 * @brief   Disable the RTS/CTS flow control in case of 3-wire mode (RX,TX,GND).
 *          @note Not saved after power off!
 *
 * @param[in] dev             The ISBD device descriptor
 */
void isbd_set_flow_control_off(isbd_t *dev);

/**
 * @brief   Disable the Data Terminal Ready control signal, in case of
 *          3-wire mode (RX,TX,GND).
 *
 *          @note Not saved after power off!
 *
 * @param[in] dev             The ISBD device descriptor
 */
void isbd_set_dtr_off(isbd_t *dev);

/**
 * @brief   Enable the AT command echo.
 *          @note Not saved after power off!
 *
 * @param[in] dev             The ISBD device descriptor
 * @param[in] enable          Enable/disable at echo
 */
void isbd_set_at_echo(isbd_t *dev, bool enable);

/**
 * @brief   Store the active profile in non-volatile memory.
 *          This is used to store user configurations for later use.
            0 Store current (active) configuration as profile 0 (AT&W0)
            1 Store current (active) configuration as profile 1(AT&W1)
 *
 * @param[in] dev             The ISBD device descriptor
 */
void isbd_set_store_config(isbd_t *dev, uint8_t profile_nr);

/**
 * @brief   Select profile for use after power-up.
            0 Select profile 0 (default) (AT&Y0)
            1 Select profile 1           (AT&Y1)
 *
 * @param[in] dev             The ISBD device descriptor
 */
void isbd_set_default_reset_config(isbd_t *dev, uint8_t profile_nr);

#ifdef __cplusplus
}
#endif

#endif /* ISBD_H */
/** @} */
