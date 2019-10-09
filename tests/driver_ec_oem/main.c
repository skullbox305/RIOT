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
 * @brief       Test application for the Atlas Scientific EC OEM sensor driver
 *
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 * @author      Ting XU <timtsui@outlook.com>
 *
 * @}
 */

#include "ec_oem_params.h"
#include "ec_oem_regs.h"
#include "xtimer.h"
#include "event/callback.h"

#include "ec_oem.h"

#define SLEEP_SEC                   (5)

/* calibration test is off by default, so it won't reset your previous calibration */
#define CALIBRATION_TEST_ENABLED    (false)

static void reading_available_event_callback(event_t *event);

static ec_oem_t dev;

static event_queue_t event_queue;
static event_t event =
{ .handler = reading_available_event_callback };

static void reading_available_event_callback(event_t *event)
{
    (void)event;

    uint32_t data = 0;

    puts("\n[EVENT - reading EC value from the device]");

    /* stop EC sensor from taking further readings*/
    oem_common_set_device_state(&dev.oem_dev, OEM_COMMON_STOP_READINGS);

    /* reset interrupt pin in case of falling or rising flank */
    oem_common_reset_interrupt_pin(&dev.oem_dev);

    ec_oem_read_ec(&dev, &data);
    printf("EC value raw: %ld\n", data);
    ec_oem_read_tds(&dev, &data);
    printf("TDS value raw: %ld\n", data);
    ec_oem_read_pss(&dev, &data);
    printf("PSS value raw: %ld\n", data);

    ec_oem_read_compensation(&dev, &data);
    printf("EC reading was taken at %ld Celsius\n", data);
}

static void interrupt_pin_callback(void *arg)
{
    puts("\n[IRQ - Reading done. Writing read-event to event queue]");
    (void)arg;

    /* Posting event to the event queue. Main is blocking with "event_wait"
     * and will execute the event callback after posting */
    event_post(&event_queue, &event);

    /* initiate new reading with "ec_oem_start_new_reading()" for this callback
       to be called again */
}

int main(void)
{
    xtimer_sleep(3);

    uint32_t data = 0;

    puts("Atlas Scientific EC OEM sensor driver test application\n");

    printf("Initializing EC OEM sensor at I2C_%i, address 0x%02x...",
           EC_OEM_PARAM_I2C, EC_OEM_PARAM_ADDR);

    if (ec_oem_init(&dev, ec_oem_params) == 0) {
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

    /* Test changing the pH OEM i2c address to 0x60 and back to the default
     * 0x64 in the sensor, as well as dev->oem_dev.params.addr
     */
    printf("Setting device address to 0x60... ");
    if (oem_common_set_i2c_address(&dev.oem_dev, 0x60) == 0) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    printf("Setting device address back to the default address 0x64... ");
    if (oem_common_set_i2c_address(&dev.oem_dev, 0x64) == 0) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    printf("Setting EC probe type to K1.0... ");
    if (ec_oem_set_probe_type(&dev, 100) == 0) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    /* Test calibration process and if it is applied correctly in the EC OEM register */
    if (CALIBRATION_TEST_ENABLED) {
        printf("Clearing all previous calibrations... ");
        if (ec_oem_clear_calibration(&dev) == 0) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        printf("Reading calibration state, should be 0... ");
        if (ec_oem_read_calibration_state(&dev, (uint8_t *)&data) == 0
            && data == 0) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            printf("state %ld\n", data);
            return -1;
        }

        /* Don't forget to provide temperature compensation for the calibration */
        printf("Setting temperature compensation to 22 Celsius... ");
        if (ec_oem_set_compensation(&dev, 2200) == 0) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        /* Start with dry calibration when doing a new calibration  */

        //test later with 17
        printf("Dry calibration... ");
        if (ec_oem_set_calibration(&dev, 0, EC_OEM_CAL_DRY) == 0) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        printf("Reading calibration state, should be 1... ");
        if (ec_oem_read_calibration_state(&dev, (uint8_t *)&data) == 0
            && data == 1) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        printf("Single point calibration... ");
        if (ec_oem_set_calibration(&dev, 4000, EC_OEM_CAL_SINGLE_POINT)
            == 0) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        printf("Reading calibration state, should be 3... ");
        if (ec_oem_read_calibration_state(&dev, (uint8_t *)&data) == 0
            && data == 3) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        printf("Calibrating to dual point low 12,880... ");
        if (ec_oem_set_calibration(&dev, 1288000, EC_OEM_CAL_DUAL_LOW) == 0) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        printf("Reading calibration state, should be 7... ");
        if (ec_oem_read_calibration_state(&dev, (uint8_t *)&data) == 0
            && data == 7) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        printf("Calibrating to dual point high 80,000...  ");
        if (ec_oem_set_calibration(&dev, 8000000, EC_OEM_CAL_DUAL_HIGH)
            == 0) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        printf("Reading calibration state, should be 13... ");
        if (ec_oem_read_calibration_state(&dev, (uint8_t *)&data) == 0
            && data == 15) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        printf("Clearing all previous calibrations... ");
        if (ec_oem_clear_calibration(&dev) == 0) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }
    }

    if (dev.oem_dev.params.interrupt_pin != GPIO_UNDEF) {
        /* Setting up and enabling the interrupt pin of the EC OEM */
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

    printf("Setting temperature compensation to 22 °C... ");
    if (ec_oem_set_compensation(&dev, 2500) == 0) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    while (1) {
        puts("\n[MAIN - Initiate reading]");

        /* blocking for ~640ms till reading is done if no interrupt pin defined */
        oem_common_start_new_reading(&dev.oem_dev);

        if (dev.oem_dev.params.interrupt_pin != GPIO_UNDEF) {
            /* when interrupt is defined, wait for the IRQ to fire and
             * the event to be posted, so the "reading_available_event_callback"
             * can be executed after */
            event_t *ev = event_wait(&event_queue);
            ev->handler(ev);
        }

        if (dev.oem_dev.params.interrupt_pin == GPIO_UNDEF) {
//            data = 0;
            if (ec_oem_read_ec(&dev, &data) == 0) {
                printf("EC value raw: %ld\n", data);
            }
            else {
                puts("[Reading EC failed]");
            }

//            data = 0;
            if (ec_oem_read_tds(&dev, &data) == 0) {
                printf("TDS value raw: %ld\n", data);
            }
            else {
                puts("[Reading TDS failed]");
            }

//            data = 0;
            if (ec_oem_read_pss(&dev, &data) == 0) {
                printf("PSS value raw: %ld\n", data);
            }
            else {
                puts("[Reading PSS failed]");
            }

            if (ec_oem_read_compensation(&dev, &data) == 0) {
                printf("EC/TDS/PSS reading was taken at raw %ld Celsius\n",
                       data);
            }
            else {
                puts("[Reading compensation failed]");
            }
        }
        xtimer_sleep(SLEEP_SEC);
    }
    return 0;
}
