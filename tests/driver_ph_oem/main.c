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
 * @brief       Test application for the Atlas Scientific pH OEM sensor driver
 *
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 *
 * @}
 */

#include "xtimer.h"
#include "timex.h"

#include "ph_oem.h"
#include "ph_oem_params.h"
#include "ph_oem_regs.h"

#define SLEEP_SEC                   (5)
#define PH_OEM_IRQ_FLANK            (PH_OEM_IRQ_BOTH)

/* off by default, so it won't reset your previous calibration */
#define CALIBRATION_TEST_ENABLED    (false)

static ph_oem_t dev;

static void interrupt_pin_cb(void *arg)
{
    puts("\n[IRQ - Reading done]");

    /* stop pH sensor from taking further readings*/
    ph_oem_set_device_state(&dev, PH_OEM_STOP_READINGS);

    /* reset interrupt pin in case of falling or rising flank */
    if (PH_OEM_IRQ_FLANK != PH_OEM_IRQ_BOTH) {
        ph_oem_set_interrupt_pin(&dev, PH_OEM_IRQ_FLANK);
    }

    ph_oem_read_ph(&dev, (int16_t *)arg);
    printf("pH value raw: %d\n", *((int16_t *)arg));

    ph_oem_read_compensation(&dev, (int16_t *)arg);
    printf("pH reading was taken at %d Celsius\n", *((int16_t *)arg));

    if (dev.params.enable_pin != GPIO_UNDEF) {
        ph_oem_enable_device(&dev, false);
    }

    /* initiate new reading with "ph_oem_start_new_reading()" for this callback
       to be called again */
}

int main(void)
{
    int16_t data;

    puts("Atlas Scientific pH OEM sensor driver test application\n");

    printf("Initializing pH OEM sensor at I2C_%i, address 0x%02x...",
           PH_OEM_PARAM_I2C, PH_OEM_PARAM_ADDR);

    if (ph_oem_init(&dev, ph_oem_params) == PH_OEM_OK) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    printf("Turning LED off... ");
    if (ph_oem_set_led_state(&dev, PH_OEM_LED_OFF) == PH_OEM_OK) {
        puts("[OK]");
        /* Sleep 2 seconds to actually see it turning off */
        xtimer_sleep(2);
    }
    else {
        puts("[Failed]");
        return -1;
    }

    printf("Turning LED on... ");
    if (ph_oem_set_led_state(&dev, PH_OEM_LED_ON) == PH_OEM_OK) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    /* Test changing the pH OEM i2c address to 0x66 and back to 0x65 in the
     * sensor as well as dev->params.addr
     */
    printf("Setting device address to 0x66... ");
    if (ph_oem_set_i2c_address(&dev, 0x66) == PH_OEM_OK) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    printf("Setting device address back to the default address 0x65... ");
    if (ph_oem_set_i2c_address(&dev, 0x65) == PH_OEM_OK) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    /* Test calibration process and if it is applied correctly in the pH OEM register */
    if (CALIBRATION_TEST_ENABLED) {
        printf("Clearing all previous calibrations... ");
        if (ph_oem_clear_calibration(&dev) == PH_OEM_OK) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        printf("Reading calibration state, should be 0... ");
        if (ph_oem_read_calibration_state(&dev, &data) == PH_OEM_OK
            && data == 0) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        /* Don't forget to provide temperature compensation for the calibration */
        printf("Setting temperature compensation to 22 Celcius... ");
        if (ph_oem_set_compensation(&dev, 2200)) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        /* Always start with mid point when doing a new calibration  */
        printf("Calibrating to midpoint... ");
        if (ph_oem_set_calibration(&dev, 6870, PH_OEM_CALIBRATE_MID_POINT)
            == PH_OEM_OK) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        printf("Reading calibration state, should be 2... ");
        if (ph_oem_read_calibration_state(&dev, &data) == PH_OEM_OK
            && data == 2) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        printf("Calibrating to lowpoint... ");
        if (ph_oem_set_calibration(&dev, 4000, PH_OEM_CALIBRATE_LOW_POINT)
            == PH_OEM_OK) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        printf("Reading calibration state, should be 3... ");
        if (ph_oem_read_calibration_state(&dev, &data) == PH_OEM_OK
            && data == 3) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        printf("Calibrating to highpoint... ");
        if (ph_oem_set_calibration(&dev, 9210, PH_OEM_CALIBRATE_HIGH_POINT)
            == PH_OEM_OK) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        printf("Reading calibration state, should be 7... ");
        if (ph_oem_read_calibration_state(&dev, &data) == PH_OEM_OK
            && data == 7) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }
    }

    if (dev.params.interrupt_pin != GPIO_UNDEF) {
        /* Setting up and enabling the interrupt pin of the pH OEM */
        printf("Setting interrupt pin to option %x...", PH_OEM_IRQ_FLANK);
        if (ph_oem_set_interrupt_pin(&dev, PH_OEM_IRQ_FLANK) == PH_OEM_OK) {
            puts("[OK]");
        }
        else {
            puts("[Failed]");
            return -1;
        }

        printf("Enabling interrupt pin... ");
        if (ph_oem_enable_interrupt(&dev, interrupt_pin_cb,
                                    &data, PH_OEM_IRQ_FLANK) == PH_OEM_OK) {
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

    printf("Enabling device... ");
    int error = ph_oem_enable_device(&dev, true);

    if (error == PH_OEM_OK) {
        puts("[OK]");
    }
    else if (error == PH_OEM_ENABLE_GPIO_UNDEF) {
        puts("[Enable pin undefined]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    while (1) {
        /* Note if you use the enable pin:
         * After powering on the device, it takes a while for the pH value
         * to stabilize. In my tests the value stabilized after around 2 minutes.
         * So sleep 120 seconds and then start a new reading if you want
         * the most accurate reading
         */
        if (dev.params.enable_pin != GPIO_UNDEF) {
            ph_oem_enable_device(&dev, true);

            /* wait for the device to boot up. Here it is recommended to
             * sleep 120 seconds for accurate results */
            xtimer_sleep(1);

            /* need to setup interrupt again, because device was off and
               settings are not retained after power cut */
            if (dev.params.interrupt_pin != GPIO_UNDEF) {
                ph_oem_set_interrupt_pin(&dev, PH_OEM_IRQ_FLANK);
            }
        }

        ph_oem_set_compensation(&dev, 2200);

        /* blocking for ~420ms till reading is done if no interrupt pin defined */
        ph_oem_start_new_reading(&dev);

        if (dev.params.interrupt_pin == GPIO_UNDEF) {

            if (ph_oem_read_ph(&dev, &data) == PH_OEM_OK) {
                printf("pH value raw: %d\n", data);
            }
            else {
                puts("[Reading pH failed]");
            }

            if (ph_oem_read_compensation(&dev, &data) == PH_OEM_OK) {
                printf("pH reading was taken at %d Celsius\n", data);
            }
            else {
                puts("[Reading compensation failed]");
            }

            if (dev.params.enable_pin != GPIO_UNDEF) {
                ph_oem_enable_device(&dev, false);
            }
        }
        xtimer_sleep(SLEEP_SEC);
    }
    return 0;
}
