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
 * @brief       Test application for the Atlas Scientific DO OEM sensor driver
 *
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 * @author      Ting XU <timtsui@outlook.com>
 *
 * @}
 */

#include "do_oem_params.h"
#include "do_oem_regs.h"
#include "xtimer.h"
#include "event/callback.h"

#include "do_oem.h"

#define SLEEP_SEC                   (5)

/* calibration test is off by default, so it won't reset your previous calibration */
#define CALIBRATION_TEST_ENABLED    (true)

static void reading_available_event_callback(event_t *event);

static do_oem_t dev;

static event_queue_t event_queue;
static event_t event =
{ .handler = reading_available_event_callback };


static void reading_available_event_callback(event_t *event)
{
    (void)event;
    uint32_t data;

    puts("\n[EVENT - reading DO value from the device]");

    /* stop DO sensor from taking further readings*/
    oem_common_set_device_state(&dev.oem_dev, OEM_COMMON_STOP_READINGS);

    /* reset interrupt pin in case of falling or rising flank */
    oem_common_reset_interrupt_pin(&dev.oem_dev);

    do_oem_read_do_mg(&dev, &data);
    printf("D.O. value in mg/L raw: %ld\n", data);
    do_oem_read_do_percent(&dev, &data);
    printf("D.O: value in Percent Saturation raw: %ld\n", data);

    do_oem_read_sali_compensation(&dev, &data);
    printf("DO reading was taken at raw %ld (μs)\n", data);
    do_oem_read_pres_compensation(&dev, &data);
    printf("DO reading was taken at raw %ld (kPa)\n", data);
    do_oem_read_temp_compensation(&dev, &data);
    printf("DO reading was taken at raw %ld (Celsius)\n", data);
}

static void interrupt_pin_callback(void *arg)
{
    puts("\n[IRQ - Reading done. Writing read-event to event queue]");
    (void)arg;

    /* Posting event to the event queue. Main is blocking with "event_wait"
     * and will execute the event callback after posting */
    event_post(&event_queue, &event);

    /* initiate new reading with "do_oem_start_new_reading()" for this callback
       to be called again */
}

int main(void)
{

	xtimer_sleep(3);
    uint32_t data = 0;

    puts("Atlas Scientific DO OEM sensor driver test application\n");

    printf("Initializing DO OEM sensor at I2C_%i, address 0x%02x...",
           DO_OEM_PARAM_I2C, DO_OEM_PARAM_ADDR);

    if (do_oem_init(&dev, do_oem_params) == 0) {
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

    /* Test changing the DO OEM i2c address t0o 0x66 and back to 0x67 in the
     * sensor as well as dev->params.addr
     */
    printf("Setting device address to 0x59... ");
    if (oem_common_set_i2c_address(&dev.oem_dev, 0x59) == 0) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    printf("Setting device address back to the default address 0x67... ");
    if (oem_common_set_i2c_address(&dev.oem_dev, 0x67) == 0) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    /* Test calibration process and if it is applied correctly in the DO OEM register */
    if (CALIBRATION_TEST_ENABLED) {
        printf("Clearing all previous calibrations... ");
        if (do_oem_clear_calibration(&dev) == 0) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        printf("Reading calibration state, should be 0... ");
        if (do_oem_read_calibration_state(&dev,
                                          (uint8_t *)&data) == 0 && data == 0) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        /* Start with atmosphere calibration when doing a new calibration  */
        printf("Calibrate to atmospheric oxygen content... ");
        if (do_oem_set_calibration(&dev, DO_OEM_CALI_ATMOSPHERIC) == 0) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        printf("Reading calibration state, should be 1... ");
        if (do_oem_read_calibration_state(&dev,
                                          (uint8_t *)&data) == 0 && data == 1) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        printf("Calibrate to 0 dissolved oxygen...");
        if (do_oem_set_calibration(&dev, DO_OEM_CALI_0_DISSOLVED) == 0) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        printf("Reading calibration state, should be 3... ");
        if (do_oem_read_calibration_state(&dev,
                                          (uint8_t *)&data) == 0 && data == 3) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }
    }

    if (dev.oem_dev.params.interrupt_pin != GPIO_UNDEF) {
//        /* initiate an event-queue. An event will be posted by the
//         * "interrupt_pin_callback" after an IRQ occurs. */
//        event_queue_init(&event_queue);

        /* Setting up and enabling the interrupt pin of the DO OEM */
        printf("Enabling interrupt pin... ");
        if (oem_common_enable_interrupt(&dev.oem_dev, interrupt_pin_callback,
                                        &data) == 0) {
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

    printf("Setting salinity compensation to 2 μS ... ");
    if (do_oem_set_sal_compensation(&dev, 200) == 0) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    printf("Setting pressure compensation to 100.32 kPa... ");
    if (do_oem_set_pres_compensation(&dev, 10032) == 0) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    printf("Setting temperature compensation to 22 °C... ");
    if (do_oem_set_temp_compensation(&dev, 2200) == 0) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    while (1) {
        puts("\n[MAIN - Initiate reading]");

        /* blocking for ~420ms till reading is done if no interrupt pin defined */
        oem_common_start_new_reading(&dev.oem_dev);

        if (dev.oem_dev.params.interrupt_pin != GPIO_UNDEF) {
            /* when interrupt is defined, wait for the IRQ to fire and
             * the event to be posted, so the "reading_available_event_callback"
             * can be executed after */
            event_t *ev = event_wait(&event_queue);
            ev->handler(ev);
        }

        if (dev.oem_dev.params.interrupt_pin == GPIO_UNDEF) {

            if (do_oem_read_do_mg(&dev, &data) == 0) {
                printf("D.O. value in mg/L raw: %ld\n", data);
            }
            else {
                puts("[Reading D.O. failed]");
            }

            if (do_oem_read_do_percent(&dev, &data) == 0) {
                printf("D.O. value in Percent Saturation raw: %ld\n", data);
            }
            else {
                puts("[Reading D.O. failed]");
            }

            if (do_oem_read_sali_compensation(&dev, &data) == 0) {
                printf("DO reading was taken at raw %ld (μS)\n", data);
            }
            else {
                puts("[Reading salinity compensation failed]");

            }
            if (do_oem_read_pres_compensation(&dev, &data) == 0) {
                printf("DO reading was taken at raw %ld (kPa)\n", data);
            }
            else {
                puts("[Reading pressure compensation failed]");

            }
            if (do_oem_read_temp_compensation(&dev, &data) == 0) {
                printf("DO reading was taken at raw %ld (Celsius)\n", data);
            }
            else {
                puts("[Reading temperature compensation failed]");
            }
        }
        xtimer_sleep(SLEEP_SEC);
    }
    return 0;
}
