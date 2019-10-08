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
 * @brief       Test application for the Atlas Scientific RTD OEM sensor driver
 *
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 * @author      Ting XU <timtsui@outlook.com>
 *
 * @}
 */

#include "xtimer.h"
#include "event/callback.h"

#include "rtd_oem.h"
#include "rtd_oem_params.h"
#include "rtd_oem_regs.h"

#define SLEEP_SEC                   (5)

/* calibration test is off by default, so it won't reset your previous calibration */
#define CALIBRATION_TEST_ENABLED    (false)

static void reading_available_event_callback(event_t *event);

static rtd_oem_t dev;

static event_queue_t event_queue;
static event_t event = { .handler = reading_available_event_callback };

static void reading_available_event_callback(event_t *event)
{
    (void)event;
    int32_t data;

    puts("\n[EVENT - reading RTD value from the device]");

    /* stop rtd sensor from taking further readings*/
    oem_common_set_device_state(&dev.oem_dev, OEM_COMMON_STOP_READINGS);

    /* reset interrupt pin in case of falling or rising flank */
    oem_common_reset_interrupt_pin(&dev.oem_dev);

    rtd_oem_read_temp(&dev, &data);
    printf("rtd value raw: %ld\n", data);
}

static void interrupt_pin_callback(void *arg)
{
    puts("\n[IRQ - Reading done]");
    (void)arg;

    /* Posting event to the event queue. Main is blocking with "event_wait"
     * and will execute the event callback after posting */
    event_post(&event_queue, &event);

    /* initiate new reading with "rtd_oem_start_new_reading()" for this callback
       to be called again */
}

int main(void)
{
    //output is really fast after reset, sleep 2 sec so we see all ouputs
    xtimer_sleep(2);

    uint32_t data = 0;

    puts("Atlas Scientific RTD OEM sensor driver test application\n");

    printf("Initializing RTD OEM sensor at I2C_%i, address 0x%02x...",
           RTD_OEM_PARAM_I2C, RTD_OEM_PARAM_ADDR);

    if (rtd_oem_init(&dev, rtd_oem_params) == 0) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    printf("Turning LED off... ");
    if (oem_common_set_led_state(&dev.oem_dev, OEM_COMMON_LED_OFF) == 0) {
        puts("[OK]");
        /* Sleep 2 seconds to actually see it turning off */
        xtimer_sleep(2);
    }
    else {
        puts("[Failed]");
        return -1;
    }

    printf("Turning LED on... ");
    if (oem_common_set_led_state(&dev.oem_dev, OEM_COMMON_LED_ON) == 0) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    /* Test changing the RTD OEM i2c address to 0x61 and back to 0x68 in the
     * sensor as well as dev->params.addr
     */
    printf("Setting device address to 0x61... ");
    if (oem_common_set_i2c_address(&dev.oem_dev, 0x61) == 0) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    printf("Setting device address back to the default address 0x68... ");
    if (oem_common_set_i2c_address(&dev.oem_dev, 0x68) == 0) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    if (CALIBRATION_TEST_ENABLED) {
        /* Test the calibration */
        printf("Clearing all previous calibrations... ");
        if (rtd_oem_clear_calibration(&dev) == 0) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        printf("Reading calibration state, should be 0... ");
        if (rtd_oem_read_calibration_state(&dev, (uint8_t *)&data) == 0
            && data == 0) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        printf("Single point calibration... ");
        if (rtd_oem_set_calibration(&dev, 100000,
                                    RTD_OEM_CAL_SINGLE_POINT) == 0) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        printf("Reading calibration state, should be 1... ");
        if (rtd_oem_read_calibration_state(&dev, (uint8_t *)&data) == 0 &&
            data == 1) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }
    }

    if (dev.oem_dev.params.interrupt_pin != GPIO_UNDEF) {
        /* initiate an event-queue. An event will be posted by the
         * "interrupt_pin_callback" after an IRQ occurs. */
        event_queue_init(&event_queue);

        /* Setting up and enabling the interrupt pin of the rtd OEM */
        printf("Enabling interrupt pin... ");
        if (oem_common_enable_interrupt(&dev.oem_dev, interrupt_pin_callback,
                                        NULL) == 0) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }
    }
    else {
        puts("Interrupt pin undefined");
    }

    while (1) {
        puts("\n[MAIN - Initiate reading]");

        /* blocking for ~420ms until reading is done, if no interrupt pin defined */
        oem_common_start_new_reading(&dev.oem_dev);

        if (dev.oem_dev.params.interrupt_pin != GPIO_UNDEF) {
            /* when interrupt is defined, wait for the IRQ to fire and
             * the event to be posted, so the "reading_available_event_callback"
             * can be executed after */
            event_t *ev = event_wait(&event_queue);
            ev->handler(ev);
        }

        if (dev.oem_dev.params.interrupt_pin == GPIO_UNDEF) {
            if (rtd_oem_read_temp(&dev, (int32_t *)&data) == 0) {
                printf("RTD value raw: %ld\n", (int32_t)data);
            }
            else {
                puts("[Reading RTD failed]");
            }
        }
        xtimer_sleep(SLEEP_SEC);
    }
    return 0;
}
