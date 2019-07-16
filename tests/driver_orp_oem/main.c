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
 * @brief       Test application for the Atlas Scientific ORP OEM sensor driver
 *
 * @author      Ting XU <timtsui@outlook.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 *
 * @}
 */

#include "xtimer.h"
#include "event/callback.h"

#include "orp_oem.h"
#include "orp_oem_params.h"
#include "orp_oem_regs.h"

#define SLEEP_SEC                   (5)

/* calibration test is off by default, so it won't reset your previous calibration */
#define CALIBRATION_TEST_ENABLED    (false)

static void reading_available_event_callback(event_t *event);

static orp_oem_t dev;

static event_queue_t event_queue;
static event_t event = { .handler = reading_available_event_callback };

static void reading_available_event_callback(event_t *event)
{
    (void)event;
    uint16_t data;

    puts("\n[EVENT - reading ORP value from the device]");

    /* stop ORP sensor from taking further readings*/
    orp_oem_set_device_state(&dev, ORP_OEM_STOP_READINGS);

    /* reset interrupt pin in case of falling or rising flank */
    orp_oem_reset_interrupt_pin(&dev);

    orp_oem_read_orp(&dev, &data);
    printf("ORP value raw: %d\n", data);

}

static void interrupt_pin_callback(void *arg)
{
    puts("\n[IRQ - Reading done. Writing read-event to event queue]");
    (void)arg;

    /* Posting event to the event queue. Main is blocking with "event_wait"
     * and will execute the event callback after posting */
    event_post(&event_queue, &event);

    /* initiate new reading with "orp_oem_start_new_reading()" for this callback
       to be called again */
}

int main(void)
{
    //output is really fast after reset, sleep 2 sec so we see all ouputs
    xtimer_sleep(2);

    uint16_t data = 0;

    puts("Atlas Scientific ORP OEM sensor driver test application\n");

    printf("Initializing ORP OEM sensor at I2C_%i, address 0x%02x...",
           ORP_OEM_PARAM_I2C, ORP_OEM_PARAM_ADDR);

    if (orp_oem_init(&dev, orp_oem_params) == ORP_OEM_OK) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    printf("Turning LED off... ");
    if (orp_oem_set_led_state(&dev, ORP_OEM_LED_OFF) == ORP_OEM_OK) {
        puts("[OK]");
        /* Sleep 2 seconds to actually see it turning off */
        xtimer_sleep(2);
    }
    else {
        puts("[Failed]");
        return -1;
    }

    printf("Turning LED on... ");
    if (orp_oem_set_led_state(&dev, ORP_OEM_LED_ON) == ORP_OEM_OK) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    /* Test changing the ORP OEM i2c address to 0x67 and back to 0x66 in the
     * sensor as well as dev->params.addr
     */
    printf("Setting device address to 0x67... ");
    if (orp_oem_set_i2c_address(&dev, 0x67) == ORP_OEM_OK) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    printf("Setting device address back to the default address 0x66... ");
    if (orp_oem_set_i2c_address(&dev, 0x66) == ORP_OEM_OK) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    /* Test the calibration */
    printf("Clearing all previous calibrations... ");
    if (orp_oem_clear_calibration(&dev) == ORP_OEM_OK) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    printf("Reading calibration state, should be 0... ");
    if (orp_oem_read_calibration_state(&dev, &data) == ORP_OEM_OK
        && data == 0) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    //    printf("Single point calibration... ");
    //    if (rtd_oem_set_calibration(&dev, 100000,
    //                                RTD_OEM_CALIBRATE_SINGLE_POINT) == RTD_OEM_OK) {
    //        puts("[OK]");
    //    }
    //    else {
    //        puts("[Failed]");
    //        return -1;
    //    }

    //    printf("Reading calibration state, should be 1... ");
    //    if (rtd_oem_read_calibration_state(&dev, &data) == RTD_OEM_OK
    //        && data == 1) {
    //        puts("[OK]");
    //    }
    //    else {
    //        puts("[Failed]");
    //        return -1;
    //    }

    if (dev.params.interrupt_pin != GPIO_UNDEF) {
        /* Setting up and enabling the interrupt pin of the ORP OEM */
        printf("Enabling interrupt pin... ");
        if (orp_oem_enable_interrupt(&dev, interrupt_pin_callback,
                                     &data) == ORP_OEM_OK) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        /* initiate an event-queue. An event will be posted by the
         * "interrupt_pin_callback" after an IRQ occurs. */
        event_queue_init(&event_queue);
    }
    else {
        puts("Interrupt pin undefined");
    }

    while (1) {
        puts("\n[MAIN - Initiate reading]");

        /* blocking for ~420ms till reading is done if no interrupt pin defined */
        orp_oem_start_new_reading(&dev);

        if (dev.params.interrupt_pin != GPIO_UNDEF) {
            /* when interrupt is defined, wait for the IRQ to fire and
             * the event to be posted, so the "reading_available_event_callback"
             * can be executed after */
            event_t *ev = event_wait(&event_queue);
            ev->handler(ev);
        }

        if (dev.params.interrupt_pin == GPIO_UNDEF) {

            if (orp_oem_read_orp(&dev, &data) == ORP_OEM_OK) {
                printf("ORP value raw: %d\n", data);
            }
            else {
                puts("[Reading ORP failed]");
            }

        }
        xtimer_sleep(SLEEP_SEC);
    }
    return 0;
}
