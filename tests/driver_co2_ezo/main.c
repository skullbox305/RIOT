/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup tests
 * @{
 *
 * @file
 * @brief       Test application for the Atlas Scientific CO2 EZO sensor driver
 *
 * @author      Ting XU <timtsui@outlook.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 *
 * @}
 */

#include "co2_ezo.h"
#include "co2_ezo_params.h"
#include "co2_ezo_internal.h"

#include "xtimer.h"
#include "event/callback.h"
#include "thread.h"

#define SLEEP_SEC                   (5)

kernel_pid_t _event_handler_pid;
static char stack_event_handler[THREAD_STACKSIZE_DEFAULT];

static co2_ezo_t dev;

static void _event_callback(event_t *event);

static event_queue_t event_queue;
static event_t event =
{ .handler = _event_callback };


static void _event_callback(event_t *event)
{
    (void)event;

    puts("\n[EVENT]");

    if (gpio_read(dev.params.alarm_int_pin) > 0) {
        puts("ALARM ON!!!");
    }
    else {
        puts("ALARM OFF");
    }
}


static void alarm_int_pin_callback(void *arg)
{
    puts("\n[IRQ - alarm triggered. Writing event to event queue]");
    (void)arg;

    /* Posting event to the event queue. Main is blocking with "event_wait"
     * and will execute the event callback after posting */
    event_post(&event_queue, &event);
}


static void *_event_handler_thread(void *arg)
{
    (void)arg;

    event_queue_init(&event_queue);
    event_loop(&event_queue);

    return NULL;
}


int main(void)
{
    xtimer_sleep(2);

    uint16_t data = 0;
    uint16_t data2 = 0;
    bool data3 = false;
    uint32_t data4 = 0;

    puts("Atlas Scientific CO2 EZO sensor driver test application\n");

    printf("Init CO2 EZO sensor at I2C_%i, address 0x%02x...",
           CO2_EZO_PARAM_I2C, CO2_EZO_PARAM_ADDR);

    if (co2_ezo_init(&dev, co2_ezo_params) == CO2_EZO_OK) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    /* start event handler thread */
    _event_handler_pid =
        thread_create(stack_event_handler, sizeof(stack_event_handler),
                      THREAD_PRIORITY_MAIN - 1,
                      THREAD_CREATE_STACKTEST,
                      _event_handler_thread,
                      NULL, "event_handler");

    if (_event_handler_pid <= KERNEL_PID_UNDEF) {
        puts("Creation of event handler thread failed");
        return -1;
    }


    printf("Turning LED off... ");
    if (co2_ezo_set_led_state(&dev, CO2_EZO_LED_OFF) == CO2_EZO_OK) {
        puts("[OK]");
        /* Sleep 2 seconds to actually see it turning off */
        xtimer_sleep(2);
    }
    else {
        puts("[Failed]");
        return -1;
    }

    printf("Turning LED on... ");
    if (co2_ezo_set_led_state(&dev, CO2_EZO_LED_ON) == CO2_EZO_OK) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    /* Test the sleep mode */
    printf("Turning device to sleep mode... ");
    if (co2_ezo_sleep_mode(&dev) == CO2_EZO_OK) {
        puts("[OK]");
    /* Sleep 2 seconds to actually see it sleeping */
        xtimer_sleep(2);
    }
    else {
        puts("[Failed]");
        return -1;
    }

    /* Test changing the CO2 EZO i2c address to 0x68 and back to 0x69 in the
     * sensor as well as dev->params.addr
     */
    printf("Setting device address to 0x68... ");
    if (co2_ezo_set_i2c_address(&dev, 0x68) == CO2_EZO_OK) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    printf("Setting device address back to the default address 0x69... ");
    if (co2_ezo_set_i2c_address(&dev, 0x69) == CO2_EZO_OK) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }


    if (dev.params.alarm_int_pin != GPIO_UNDEF) {
        /* Disable alarm */
        printf("Disabling alarm... ");
        if (co2_ezo_disable_alarm(&dev) == CO2_EZO_OK) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        /* Enable alarm and set the alarm value to 380 and tolerance value to 10 */
        printf("Enabling alarm... ");
        if (co2_ezo_enable_alarm(&dev, 380, 10, alarm_int_pin_callback,
                                 NULL) == CO2_EZO_OK) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        /* Get CO2 EZO alarm state */
        printf("Getting alarm state... ");
        if (co2_ezo_get_alarm_state(&dev, &data, &data2,
                                    &data3) == CO2_EZO_OK) {
            puts("[OK]");
            xtimer_sleep(2);
            printf("Alarm value is %d ppm\n", data);
            printf("Tolerance value is %d ppm\n", data2);
            printf("Alarm is %s\n", data3 ? "enabled" : "disabled");

        }
        else {
            puts("[Reading CO2 alarm state Failed]");
            return -1;
        }
    }

    while (1) {
        puts("\n[MAIN - Initiate reading]");

        if (co2_ezo_read_co2(&dev, &data, &data4) == CO2_EZO_OK) {
            printf("CO2 value in ppm : %d\n", data);
            printf("Internal temperature raw(10^-3) : %lu\n", data4);
        }
        else if (co2_ezo_read_co2(&dev, &data, &data4) == CO2_EZO_WARMING) {
            puts("[CO2 EZO still warming]");
        }
        else {
            puts("[Reading CO2 failed]");
        }
        xtimer_sleep(SLEEP_SEC);
    }
    return 0;

}
