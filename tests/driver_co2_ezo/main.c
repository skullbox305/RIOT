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

#define SLEEP_SEC                   (5)

static co2_ezo_t dev;

int main(void)
{
	xtimer_sleep(2);

	uint16_t data = 0;
//	uint16_t data2 = 0;

	puts("Atlas Scientific CO2 EZO sensor driver test application\n");

	printf("Init CO2 EZO sensor at I2C_%i, address 0x%02x...",
	CO2_EZO_PARAM_I2C, CO2_EZO_PARAM_ADDR);

	if (co2_ezo_init(&dev, co2_ezo_params) == CO2_EZO_OK)
	{
		puts("[OK]");
	}
	else
	{
		puts("[Failed]");
		return -1;
	}

	printf("Turning LED off... ");
	if (co2_ezo_set_led_state(&dev, CO2_EZO_LED_OFF) == CO2_EZO_OK)
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
	if (co2_ezo_set_led_state(&dev, CO2_EZO_LED_ON) == CO2_EZO_OK)
	{
		puts("[OK]");
	}
	else
	{
		puts("[Failed]");
		return -1;
	}

	xtimer_sleep(1);

	/* Test changing the CO2 EZO i2c address to 0x68 and back to 0x69 in the
	 * sensor as well as dev->params.addr
	 */
	printf("Setting device address to 0x68... ");
	if (co2_ezo_set_i2c_address(&dev, 0x68) == CO2_EZO_OK)
	{
		puts("[OK]");
	}
	else
	{
		puts("[Failed]");
		return -1;
	}

	xtimer_sleep(2);

	printf("Setting device address back to the default address 0x69... ");
	if (co2_ezo_set_i2c_address(&dev, 0x69) == CO2_EZO_OK)
	{
		puts("[OK]");
	}
	else
	{
		puts("[Failed]");
		return -1;
	}

	xtimer_sleep(2);

//	/* Enable alarm and set the alarm value to 1500 and tolerance value to 100 */
//	printf("Enabling alarm... ");
//	if (co2_ezo_enable_alarm(&dev,1300,100) == CO2_EZO_OK)
//	{
//		puts("[OK]");
//	}
//	else
//	{
//		puts("[Failed]");
//		return -1;
//	}
//
//	xtimer_sleep(2);
//
//	/* Gets CO2 EZO alarm state */
//	printf("Getting alarm state... ");
//	if (co2_ezo_get_alarm_state(&dev, &data, &data2) == CO2_EZO_OK)
//	{
//		puts("[OK]");
//		xtimer_sleep(2);
//		printf("\nAlarm value is %d ppm", data);
//		printf("\nTolerance value is %d ppm \n", data2);
//	}
//	else
//	{
//		puts("[Reading CO2 alarm state Failed]");
//		return -1;
//	}
//
//	xtimer_sleep(2);

	/* Disable alarm */
	printf("Disabling alarm... ");
	if (co2_ezo_disable_alarm(&dev) == CO2_EZO_OK)
	{
		puts("[OK]");
	}
	else
	{
		puts("[Failed]");
		return -1;
	}

	/* Test sleep mode */
	printf("Turning device to sleep mode... ");
	if (co2_ezo_sleep_mode(&dev) == CO2_EZO_OK)
	{
		puts("[OK]");
	}
	else
	{
		puts("[Failed]");
		return -1;
	}

	/* Sleep 2 seconds to actually see it sleeping */
	xtimer_sleep(2);

	while (1)
	{
		puts("\n[MAIN - Initiate reading]");

		if (co2_ezo_read_co2(&dev, &data) == CO2_EZO_OK)
		{
			printf("CO2 value in ppm : %d\n", data);
		}
		else
		{
			puts("[Reading CO2 failed]");
		}
		xtimer_sleep(SLEEP_SEC);
	}
	return 0;

}
