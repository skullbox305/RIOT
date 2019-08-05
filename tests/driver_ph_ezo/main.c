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
 * @brief       Test application for the Atlas Scientific pH EZO sensor driver
 *
 * @author      Ting XU <timtsui@outlook.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 *
 * @}
 */

#include "ph_ezo.h"
#include "ph_ezo_params.h"
#include "ph_ezo_internal.h"

#include "xtimer.h"
#include "event/callback.h"
#include "thread.h"

#define SLEEP_SEC                   (5)

/* calibration test is off by default, so it won't reset your previous calibration */
#define CALIBRATION_TEST_ENABLED    (false)

static ph_ezo_t dev;

int main(void)
{
	xtimer_sleep(2);

	uint16_t data = 0;
	uint16_t data2 = 0;

	puts("Atlas Scientific PH EZO sensor driver test application\n");

	printf("Init PH EZO sensor at I2C_%i, address 0x%02x...",
	PH_EZO_PARAM_I2C, PH_EZO_PARAM_ADDR);

	if (ph_ezo_init(&dev, ph_ezo_params) == PH_EZO_OK)
	{
		puts("[OK]");
	}
	else
	{
		puts("[Failed]");
		return -1;
	}

	puts("Enable factory reset...\n");

	if (ph_ezo_reset(&dev) == PH_EZO_OK)
	{
		puts("[OK]");
	}
	else
	{
		puts("[Failed]");
		return -1;
	}

	printf("Turning LED off... ");
	if (ph_ezo_set_led_state(&dev, PH_EZO_LED_OFF) == PH_EZO_OK)
	{
		puts("[OK]");
		/* Sleep 2 seconds to actually see it turning off */
		xtimer_sleep(2);
	}
	else
	{
		puts("[Failed]");
		return -1;
	}

	printf("Turning LED on... ");
	if (ph_ezo_set_led_state(&dev, PH_EZO_LED_ON) == PH_EZO_OK)
	{
		puts("[OK]");
	}
	else
	{
		puts("[Failed]");
		return -1;
	}

	/* Test the sleep mode */
	printf("Turning device to sleep mode... ");
	if (ph_ezo_sleep_mode(&dev) == PH_EZO_OK)
	{
		puts("[OK]");
		/* Sleep 2 seconds to actually see it sleeping */
		xtimer_sleep(2);
	}
	else
	{
		puts("[Failed]");
		return -1;
	}

	/* Test changing the PH EZO i2c address to 0x68 and back to 0x69 in the
	 * sensor as well as dev->params.addr
	 */
	printf("Setting device address to 0x64... ");
	if (ph_ezo_set_i2c_address(&dev, 0x64) == PH_EZO_OK)
	{
		puts("[OK]");
	}
	else
	{
		puts("[Failed]");
		return -1;
	}

	printf("Setting device address back to the default address 0x63... ");
	if (ph_ezo_set_i2c_address(&dev, 0x63) == PH_EZO_OK)
	{
		puts("[OK]");
	}
	else
	{
		puts("[Failed]");
		return -1;
	}

	if (CALIBRATION_TEST_ENABLED)
	{
		printf("Clearing all previous calibrations... ");
		if (ph_ezo_clear_calibration(&dev) == PH_EZO_OK)
		{
			puts("[OK]");
		}
		else
		{
			puts("[Failed]");
			return -1;
		}

		printf("Reading calibration state, should be 0... ");
		if (ph_ezo_get_calibration_state(&dev, &data) == PH_EZO_OK
				&& data == PH_EZO_NOT_CALIBRATED)
		{
			puts("[OK]");
		}
		else
		{
			puts("[Failed]");
			return -1;
		}

		/* Always start with mid point when doing a new calibration  */
		printf("Calibrating to midpoint... ");
		if (ph_ezo_set_calibration(&dev, 6870, PH_EZO_CALIBRATE_MID_POINT)
				== PH_EZO_OK)
		{
			puts("[OK]");
		}
		else
		{
			puts("[Failed]");
			return -1;
		}

		printf("Reading calibration state, should be 1... ");
		if (ph_ezo_read_calibration_state(&dev, &data) == PH_EZO_OK
				&& data == PH_EZO_CALIBRATE_ONE)
		{
			puts("[OK]");
		}
		else
		{
			puts("[Failed]");
			return -1;
		}

		printf("Calibrating to lowpoint... ");
		if (ph_ezo_set_calibration(&dev, 4000, PH_EZO_CALIBRATE_LOW_POINT)
				== PH_EZO_OK)
		{
			puts("[OK]");
		}
		else
		{
			puts("[Failed]");
			return -1;
		}

		printf("Reading calibration state, should be 2... ");
		if (ph_ezo_read_calibration_state(&dev, &data) == PH_EZO_OK
				&& data == PH_EZO_CALIBRATE_TWO)
		{
			puts("[OK]");
		}
		else
		{
			puts("[Failed]");
			return -1;
		}

		printf("Calibrating to highpoint... ");
		if (ph_ezo_set_calibration(&dev, 9210, PH_EZO_CALIBRATE_HIGH_POINT)
				== PH_EZO_OK)
		{
			puts("[OK]");
		}
		else
		{
			puts("[Failed]");
			return -1;
		}

		printf("Reading calibration state, should be 3... ");
		if (ph_oem_read_calibration_state(&dev, &data) == PH_EZO_OK
				&& data == PH_EZO_CALIBRATE_THREE)
		{
			puts("[OK]");
		}
		else
		{
			puts("[Failed]");
			return -1;
		}
	}


	/* Get the slope of the pH probe */
	puts("Getting the slope of the pH probe... ");
	if (ph_ezo_read_slope(&dev, &data, &data2) == PH_OEM_OK)
	{
		printf("slope of acid calibration in raw: %d\n", data);
		printf("slope of base calibration in raw: %d\n", data2);
	}
	else
	{
		puts("[Reading slope failed]");
		return -1;
	}

	while (1)
	{
		puts("\n[MAIN - Initiate reading]");

		if (ph_ezo_read_ph(&dev, &data) == PH_EZO_OK)
		{
			printf("PH value in raw : %d\n", data);
		}
		else
		{
			puts("[Reading PH failed]");
		}
		xtimer_sleep(SLEEP_SEC);
	}
	return 0;

}
