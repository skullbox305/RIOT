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
 * @author      Ting XU <timtsui@outlook.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
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
#define CALIBRATION_TEST_ENABLED    (false)

static void reading_available_event_callback(event_t *event);

static do_oem_t dev;

static event_queue_t event_queue;
static event_t event =
{ .handler = reading_available_event_callback };

static void reading_available_event_callback(event_t *event)
{
	(void) event;
	uint16_t data;

	puts("\n[EVENT - reading DO value from the device]");

	/* stop DO sensor from taking further readings*/
	do_oem_set_device_state(&dev, DO_OEM_STOP_READINGS);

	/* reset interrupt pin in case of falling or rising flank */
	do_oem_reset_interrupt_pin(&dev);

	do_oem_read_do_mg(&dev, &data);
	printf("D.O. value in mg/L raw: %d\n", data);
	do_oem_read_do_percent(&dev, &data);
	printf("D.O: value in % saturation raw: %d\n", data);

	do_oem_read_sali_compensation(&dev, &data);
	printf("DO reading was taken at %d μs\n", data);
	do_oem_read_pres_compensation(&dev, &data);
	printf("DO reading was taken at %d kPa\n", data);
	do_oem_read_temp_compensation(&dev, &data);
	printf("DO reading was taken at %d Celsius\n", data);
}

static void interrupt_pin_callback(void *arg)
{
	puts("\n[IRQ - Reading done. Writing read-event to event queue]");
	(void) arg;

	/* Posting event to the event queue. Main is blocking with "event_wait"
	 * and will exdoute the event callback after posting */
	event_post(&event_queue, &event);

	/* initiate new reading with "do_oem_start_new_reading()" for this callback
	 to be called again */
}

int main(void)
{
	xtimer_sleep(2);

	uint16_t data = 0;

	puts("Atlas Scientific DO OEM sensor driver test application\n");

	printf("Initializing DO OEM sensor at I2C_%i, address 0x%02x...",
	DO_OEM_PARAM_I2C, DO_OEM_PARAM_ADDR);

	if (do_oem_init(&dev, do_oem_params) == DO_OEM_OK)
	{
		puts("[OK]");
	}
	else
	{
		puts("[Failed]");
		return -1;
	}

	printf("Turning LED off... ");
	if (do_oem_set_led_state(&dev, DO_OEM_LED_OFF) == DO_OEM_OK)
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
	if (do_oem_set_led_state(&dev, DO_OEM_LED_ON) == DO_OEM_OK)
	{
		puts("[OK]");
	}
	else
	{
		puts("[Failed]");
		return -1;
	}

	/* Test changing the DO OEM i2c address t0o 0x66 and back to 0x67 in the
	 * sensor as well as dev->params.addr
	 */
	printf("Setting device address to 0x66... ");
	if (do_oem_set_i2c_address(&dev, 0x66) == DO_OEM_OK)
	{
		puts("[OK]");
	}
	else
	{
		puts("[Failed]");
		return -1;
	}

	printf("Setting device address back to the default address 0x67... ");
	if (do_oem_set_i2c_address(&dev, 0x67) == DO_OEM_OK)
	{
		puts("[OK]");
	}
	else
	{
		puts("[Failed]");
		return -1;
	}

	/* Test calibration process and if it is applied correctly in the DO OEM register */
	if (CALIBRATION_TEST_ENABLED)
	{
		printf("Clearing all previous calibrations... ");
		if (do_oem_clear_calibration(&dev) == DO_OEM_OK)
		{
			puts("[OK]");
		}
		else
		{
			puts("[Failed]");
			return -1;
		}

		printf("Reading calibration state, should be 0... ");
		if (do_oem_read_calibration_state(&dev, &data) == DO_OEM_OK
				&& data == 0)
		{
			puts("[OK]");
		}
		else
		{
			puts("[Failed]");
			return -1;
		}

		/* Don't forget to provide the compensations for the calibration */

		printf("Setting salinity compensation to ?? μS... ");
		if (do_oem_set_sal_compensation(&dev, 2200) == DO_OEM_OK)
		{
			puts("[OK]");
		}
		else
		{
			puts("[Failed]");
			return -1;
		}

		printf("Setting pressure compensation to ?? kPa... ");
		if (do_oem_set_pres_compensation(&dev, 2200) == DO_OEM_OK)
		{
			puts("[OK]");
		}
		else
		{
			puts("[Failed]");
			return -1;
		}

		printf("Setting temperature compensation to ?? Celsius... ");
		if (do_oem_set_temp_compensation(&dev, 2200) == DO_OEM_OK)
		{
			puts("[OK]");
		}
		else
		{
			puts("[Failed]");
			return -1;
		}

		/* Start with atmosphere calibration when doing a new calibration  */

		//test later with 17
		printf("Calibrate to atmospheric oxygen content... ");
		if (do_oem_set_calibration(&dev, DO_OEM_CALIBRATE_ATMOSPHERIC)
				== DO_OEM_OK)
		{
			puts("[OK]");
		}
		else
		{
			puts("[Failed]");
			return -1;
		}

		printf("Reading calibration state, should be 1... ");
		if (do_oem_read_calibration_state(&dev, &data) == DO_OEM_OK
				&& data == 1)
		{
			puts("[OK]");
		}
		else
		{
			puts("[Failed]");
			return -1;
		}

		printf("Calibrate to atmospheric oxygen content... ... ");
		if (do_oem_set_calibration(&dev, DO_OEM_CALIBRATE_0_DISSOLVED)
				== DO_OEM_OK)
		{
			puts("[OK]");
		}
		else
		{
			puts("[Failed]");
			return -1;
		}

		printf("Reading calibration state, should be 3... ");
		if (do_oem_read_calibration_state(&dev, &data) == DO_OEM_OK
				&& data == 3)
		{
			puts("[OK]");
		}
		else
		{
			puts("[Failed]");
			return -1;
		}

	}

	if (dev.params.interrupt_pin != GPIO_UNDEF)
	{
		/* Setting up and enabling the interrupt pin of the DO OEM */
		printf("Enabling interrupt pin... ");
		if (do_oem_enable_interrupt(&dev, interrupt_pin_callback, &data)
				== DO_OEM_OK)
		{
			puts("[OK]");
		}
		else
		{
			puts("[Failed]");
			return -1;
		}

		/* initiate an event-queue. An event will be posted by the
		 * "interrupt_pin_callback" after an IRQ occurs. */
		event_queue_init(&event_queue);
	}
	else
	{
		puts("Interrupt pin undefined");
	}

	printf("Setting salinity compensation to ?? μS ... ");
	if (do_oem_set_sal_compensation(&dev, 2200) == DO_OEM_OK)
	{
		puts("[OK]");
	}
	else
	{
		puts("[Failed]");
		return -1;
	}

	printf("Setting pressure compensation to ?? kPa... ");
	if (do_oem_set_pres_compensation(&dev, 2200) == DO_OEM_OK)
	{
		puts("[OK]");
	}
	else
	{
		puts("[Failed]");
		return -1;
	}

	printf("Setting temperature compensation to ?? °C... ");
	if (do_oem_set_temp_compensation(&dev, 2200) == DO_OEM_OK)
	{
		puts("[OK]");
	}
	else
	{
		puts("[Failed]");
		return -1;
	}

	while (1)
	{
		puts("\n[MAIN - Initiate reading]");

		/* blocking for ~420ms till reading is done if no interrupt pin defined */
		do_oem_start_new_reading(&dev);

		if (dev.params.interrupt_pin != GPIO_UNDEF)
		{
			/* when interrupt is defined, wait for the IRQ to fire and
			 * the event to be posted, so the "reading_available_event_callback"
			 * can be executed after */
			event_t *ev = event_wait(&event_queue);
			ev->handler(ev);
		}

		if (dev.params.interrupt_pin == GPIO_UNDEF)
		{

			if (do_oem_read_do_mg(&dev, &data) == DO_OEM_OK)
			{
				printf("D.O. value in mg/L raw: %d\n", data);
			}
			else
			{
				puts("[Reading D.O. failed]");
			}

			if (do_oem_read_do_percent(&dev, &data) == DO_OEM_OK)
			{
				printf("D.O. value in % saturation raw: %d\n", data);
			}
			else
			{
				puts("[Reading D.O. failed]");
			}

			if (do_oem_read_sal_compensation(&dev, &data) == DO_OEM_OK)
			{
				printf("DO reading was taken at %d μS\n", data);
			}
			else
			{
				puts("[Reading salinity compensation failed]");

			}
			if (do_oem_read_pres_compensation(&dev, &data) == DO_OEM_OK)
			{
				printf("DO reading was taken at %d kPa\n", data);
			}
			else
			{
				puts("[Reading pressure compensation failed]");

			}
			if (do_oem_read_temp_compensation(&dev, &data) == DO_OEM_OK)
			{
				printf("DO reading was taken at %d Celsius\n", data);
			}
			else
			{
				puts("[Reading temperature compensation failed]");

			}
		}
		xtimer_sleep(SLEEP_SEC);
	}
	return 0;
}
