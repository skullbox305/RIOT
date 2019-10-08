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
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 * @author      Ting XU <timtsui@outlook.com>
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
    int32_t data = 0;

    puts("\n[EVENT - reading ORP value from the device]");

    /* stop ORP sensor from taking further readings*/
    oem_common_set_device_state(&dev.oem_dev, OEM_COMMON_STOP_READINGS);
    /* reset interrupt pin in case of falling or rising flank */
    oem_common_reset_interrupt_pin(&dev.oem_dev);

    orp_oem_read_orp(&dev, &data);
    printf("ORP value raw: %ld\n", data);

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

	xtimer_sleep(2);
    uint32_t data = 0;

    puts("Atlas Scientific ORP OEM sensor driver test application\n");

    printf("Initializing ORP OEM sensor at I2C_%i, address 0x%02x...",
           ORP_OEM_PARAM_I2C, ORP_OEM_PARAM_ADDR);

    if (orp_oem_init(&dev, orp_oem_params) == 0) {
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

    /* Test changing the ORP OEM i2c address to 0x63 and back to the default
     * 0x66 in the sensor as well as dev->oem_dev.params.addr
     */
    printf("Setting device address to 0x63... ");
    if (oem_common_set_i2c_address(&dev.oem_dev, 0x63) == 0) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    printf("Setting device address back to the default address 0x66... ");
    if (oem_common_set_i2c_address(&dev.oem_dev, 0x66) == 0) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    if (CALIBRATION_TEST_ENABLED) {
        /* Test the calibration */
        printf("Clearing all previous calibrations... ");
        if (orp_oem_clear_calibration(&dev) == 0) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        printf("Reading calibration state, should be 0... ");
        if (orp_oem_read_calibration_state(&dev, (uint8_t *)&data) == 0
            && data == 0) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        printf("Single point calibration... ");
        if (orp_oem_set_calibration(&dev, 10000,
                                    ORP_OEM_CAL_SINGLE_POINT) == 0) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        printf("Reading calibration state, should be 1... ");
        if (orp_oem_read_calibration_state(&dev, (uint8_t *)&data) == 0
            && data == 1) {
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

        /* Setting up and enabling the interrupt pin of the ORP OEM */
        printf("Enabling interrupt pin... ");
        if (oem_common_enable_interrupt(&dev.oem_dev, interrupt_pin_callback,
                                        &data) == 0) {
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
            if (orp_oem_read_orp(&dev, (int32_t *)&data) == 0) {
                printf("ORP value raw: %ld\n", data);
            }
            else {
                puts("[Reading ORP failed]");
            }
        }
        xtimer_sleep(SLEEP_SEC);
    }
    return 0;
}
