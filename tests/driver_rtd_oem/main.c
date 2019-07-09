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
 * @author      Ting XU <your-email@placeholder.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 *
 * @}
 */

#include "xtimer.h"
#include "event/callback.h"

#include "rtd_oem.h"
#include "rtd_oem_params.h"
#include "rtd_oem_regs.h"

#define SLEEP_SEC                   (5)

static rtd_oem_t dev;

int main(void)
{
    //output is really fast after reset, sleep 2 sec so we see all ouputs
    xtimer_sleep(2);

    uint16_t data = 0;

    puts("Atlas Scientific RTD OEM sensor driver test application\n");

    printf("Initializing RTD OEM sensor at I2C_%i, address 0x%02x...",
           RTD_OEM_PARAM_I2C, RTD_OEM_PARAM_ADDR);

    if (rtd_oem_init(&dev, rtd_oem_params) == RTD_OEM_OK) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    printf("Turning LED off... ");
    if (rtd_oem_set_led_state(&dev, RTD_OEM_LED_OFF) == RTD_OEM_OK) {
        puts("[OK]");
        /* Sleep 2 seconds to actually see it turning off */
        xtimer_sleep(2);
    }
    else {
        puts("[Failed]");
        return -1;
    }

    printf("Turning LED on... ");
    if (rtd_oem_set_led_state(&dev, RTD_OEM_LED_ON) == RTD_OEM_OK) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    /* Test changing the RTD OEM i2c address to 0x67 and back to 0x68 in the
     * sensor as well as dev->params.addr
     */
    printf("Setting device address to 0x67... ");
    if (rtd_oem_set_i2c_address(&dev, 0x67) == RTD_OEM_OK) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    printf("Setting device address back to the default address 0x68... ");
    if (rtd_oem_set_i2c_address(&dev, 0x68) == RTD_OEM_OK) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    /* Test the calibration */
    printf("Clearing all previous calibrations... ");
    if (rtd_oem_clear_calibration(&dev) == RTD_OEM_OK) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    printf("Reading calibration state, should be 0... ");
    if (rtd_oem_read_calibration_state(&dev, &data) == RTD_OEM_OK
        && data == 0) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }

    printf("Single point calibration... ");
    if (ph_oem_set_calibration(&dev, 100, RTD_OEM_CALIBRATE_SINGLE_POINT) == PH_OEM_OK) {
        puts("[OK]");
    }
    else {
        puts("[Failed]");
        return -1;
    }




    while (1) {

        xtimer_sleep(SLEEP_SEC);
    }
    return 0;
}
