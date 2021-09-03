/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 * @{
 *
 * @file
 * @brief       Test application for the Iridium SBD modem driver
 *
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 *
 * @}
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>

//#include "timex.h"

#include "shell.h"
#include "shell_commands.h"
#include "xtimer.h"
#include "thread.h"

#include "isbd.h"
#include "isbd_netdev.h"
#include "isbd_params.h"
#include "isbd_internal.h"

#define SHELL_BUFSIZE (512U)

#define ISBD_STACKSIZE        (THREAD_STACKSIZE_DEFAULT)

#define MSG_TYPE_ISR          (0x3456)

static char stack[ISBD_STACKSIZE];
static kernel_pid_t _recv_pid;

static char rx_buf[ISBD_MAX_RESP_BUF];

static isbd_t isbd_dev;


static int register_cmd(int argc, char **argv)
{
    (void)argv;
    if (argc > 1) {
        puts("usage: regist");
        return -1;
    }
    isbd_start_network_registration(&isbd_dev);

    return 0;
}

static int off_cmd(int argc, char **argv)
{
    (void)argv;
    if (argc > 1) {
        puts("usage: sys_sleep");
        return -1;
    }
    isbd_set_off(&isbd_dev);

    return 0;
}

static int on_cmd(int argc, char **argv)
{
    (void)argv;
    if (argc > 1) {
        puts("usage: sys_wake");
        return -1;
    }
    isbd_on(&isbd_dev);

    return 0;
}


int cmd_send(int argc, char **argv)
{
    if (argc <= 1) {
        puts("usage: send <payload>");
        return -1;
    }

    printf("sending \"%s\" payload (%u bytes)\n",
           argv[1], (unsigned)strlen(argv[1]));

    iolist_t iolist = {
        .iol_base = argv[1],
        .iol_len = (strlen(argv[1]))
    };

    isbd_set_standby(&isbd_dev);

    netdev_t *netdev = (netdev_t *)&isbd_dev;
    if (netdev->driver->send(netdev, &iolist) == -ENOTSUP) {
        puts("Cannot send: radio is still transmitting");
    }

    return 0;
}



static const shell_command_t shell_commands[] = {
    { "off", "Put device to sleep", off_cmd },
    { "on", "Wake up device", on_cmd },
    { "send", "Send payload as binary message", cmd_send },
    { "regist", "network registration", register_cmd },
    { NULL, NULL, NULL }
};

static void _event_cb(netdev_t *dev, netdev_event_t event)
{
    if (event == NETDEV_EVENT_ISR) {
        msg_t msg;
        msg.type = MSG_TYPE_ISR;
        msg.content.ptr = dev;

        int res;

        if ((res = msg_send(&msg, _recv_pid)) <= 0) {
            printf("gnrc_netdev: possibly lost interrupt. code: %d\n", res);
        }
    }
    else {
        size_t len;
        switch (event) {
            case NETDEV_EVENT_TX_MEDIUM_BUSY:
                printf(
                    "[sat radio] EVENT TX Busy: previous transmission still pending...\n");
                break;
            case NETDEV_EVENT_TX_STARTED:
                printf("[sat radio] EVENT: TX started...\n");
                break;
            case NETDEV_EVENT_TX_COMPLETE:
                printf("[sat radio] EVENT: TX completed...\n");

                /* If more messages are queued at the gateway,
                 * do a send with no payload (mailbox check) to receive the
                 * next message
                 */
                if (isbd_dev._internal.rx_queued > 0) {
                    if (dev->driver->send(dev, NULL) == -ENOTSUP) {
                        puts("Cannot send: radio is busy");
                    }
                }
                else if (isbd_dev._internal.rx_queued == 0) {
                    isbd_set_off(&isbd_dev);
                }
                if (isbd_get_state(&isbd_dev) == ISBD_STATE_IDLE) {
                    isbd_set_off(&isbd_dev);
                }
                break;
            case NETDEV_EVENT_TX_TIMEOUT:
                if (isbd_dev._internal.tx_retries > 0) {
                    printf("[sat radio] EVENT TX Timeout after %d retries\n",
                           ISBD_DEFAULT_SBDIX_MAX_RETRIES);
                }
                else {
                    printf("[sat radio] EVENT TX Timeout: no network signal\n");
                }
                isbd_set_off(&isbd_dev);
                break;
            case NETDEV_EVENT_RX_COMPLETE:
                printf("[sat radio] EVENT RX Completed...\n");
                len = dev->driver->recv(dev, NULL, 0, 0);
                dev->driver->recv(dev, rx_buf, len, NULL);
                printf("[sat radio] Recv. msg: %s\n", rx_buf);

                /* clear rx buffer, so the old message wont overlap with the new */
                memset(rx_buf, 0, sizeof rx_buf);
                break;
            case NETDEV_EVENT_CRC_ERROR:
                printf("[sat radio] EVENT: RX CRC Error\n");
                break;
            case NETDEV_EVENT_LINK_UP:
                printf("[sat radio] EVENT: Signal available\n");
                break;
            case NETDEV_EVENT_LINK_DOWN:
                printf("[sat radio] EVENT: No Signal\n");
                break;
            default:
                printf("[sat radio] Unexpected netdev event received: %d\n",
                       event);
                break;
        }
    }
}

void *_recv_thread(void *arg)
{
    (void)arg;

    static msg_t _msg_q[16U];
    msg_init_queue(_msg_q, 16U);

    while (1) {
        msg_t msg;
        msg_receive(&msg);
        if (msg.type == MSG_TYPE_ISR) {
            netdev_t *dev = msg.content.ptr;
            dev->driver->isr(dev);
        }
        else {
            puts("Unexpected msg type");
        }
    }
}

int main(void)
{
    netdev_t *netdev = (netdev_t *)&isbd_dev;

    netdev->driver = &isbd_driver;
    isbd_dev.params = isbd_params[0];

    if (netdev->driver->init(netdev) < 0) {
        puts("Failed to initialize ISBD device, exiting");
        return 0;
    }

    netdev->event_callback = _event_cb;

    _recv_pid = thread_create(stack, sizeof(stack), THREAD_PRIORITY_MAIN - 1,
                              THREAD_CREATE_STACKTEST, _recv_thread, NULL,
                              "recv_thread");

    if (_recv_pid <= KERNEL_PID_UNDEF) {
        puts("Creation of receiver thread failed");
        return 1;
    }

    /* start the shell */
    puts("Initialization successful - starting the shell now");
    char line_buf[SHELL_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_BUFSIZE);

    return 0;
}
