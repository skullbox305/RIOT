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
 * @brief       Test application for the Atlas Scientific RGB EZO sensor driver
 *
 * @author      Ting XU <timtsui@outlook.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 *
 * @}
 */

#include "rgb_ezo.h"
#include "rgb_ezo_params.h"
#include "rgb_ezo_internal.h"

#include "xtimer.h"
#include "string.h"
//#include "event/callback.h"
#include "thread.h"

#define SLEEP_SEC                   (5)

//kernel_pid_t _event_handler_pid;
//static char stack_event_handler[THREAD_STACKSIZE_DEFAULT];

static rgb_ezo_t dev;

//static void _event_callback(event_t *event);
//
//static event_queue_t event_queue;
//static event_t event =
//{ .handler = _event_callback };
//
//static void _event_callback(event_t *event)
//{
//	(void) event;
//
//	puts("\n[EVENT]");
//
//	if (gpio_read(dev.params.alarm_int_pin) > 0)
//	{
//		puts("ALARM ON!!!");
//	}
//	else
//	{
//		puts("ALARM OFF!!!");
//	}
//}

//static void alarm_int_pin_callback(void *arg)
//{
//	puts("\n[IRQ - alarm triggered. Writing event to event queue]");
//	(void) arg;
//
//	/* Posting event to the event queue. Main is blocking with "event_wait"
//	 * and will execute the event callback after posting */
//	event_post(&event_queue, &event);
//}

//static void *_event_handler_thread(void *arg)
//{
//	(void) arg;
//
//	event_queue_init(&event_queue);
//	event_loop(&event_queue);
//
//	return NULL;
//}

int main(void)
{
	xtimer_sleep(2);

	uint16_t data1 = 0;
	uint16_t data2 = 0;
	uint16_t data3 = 0;

	char string_data[30];

	puts("Atlas Scientific RGB EZO sensor driver test application\n");

	printf("Initiate RGB EZO sensor at I2C_%i, address 0x%02x...",
	RGB_EZO_PARAM_I2C, RGB_EZO_PARAM_ADDR);

	if (rgb_ezo_init(&dev, rgb_ezo_params) == RGB_EZO_OK)
	{
		puts("[OK]");
	}
	else
	{
		puts("[Failed]");
		return -1;
	}

//	/* start event handler thread */
//	_event_handler_pid = thread_create(stack_event_handler,
//			sizeof(stack_event_handler),
//			THREAD_PRIORITY_MAIN - 1,
//			THREAD_CREATE_STACKTEST, _event_handler_thread,
//			NULL, "event_handler");
//
//	if (_event_handler_pid <= KERNEL_PID_UNDEF)
//	{
//		puts("Creation of event handler thread failed");
//		return -1;
//	}

	printf("Turning LED off... ");
	if (rgb_ezo_set_indicator_led_state(&dev, RGB_EZO_LED_OFF) == RGB_EZO_OK)
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
	if (rgb_ezo_set_indicator_led_state(&dev, RGB_EZO_LED_ON) == RGB_EZO_OK)
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
	if (rgb_ezo_sleep_mode(&dev) == RGB_EZO_OK)
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

	/* Test changing the RGB EZO i2c address to 0x69 and back to 0x70 in the
	 * sensor as well as dev->params.addr
	 */
	printf("Setting device address to 0x69... ");
	if (rgb_ezo_set_i2c_address(&dev, 0x69) == RGB_EZO_OK)
	{
		puts("[OK]");
	}
	else
	{
		puts("[Failed]");
		return -1;
	}

	printf("Setting device address back to the default address 0x70... ");
	if (rgb_ezo_set_i2c_address(&dev, 0x70) == RGB_EZO_OK)
	{
		puts("[OK]");
	}
	else
	{
		puts("[Failed]");
		return -1;
	}

	printf("Calibrating the sensor to a white object... ");
	if (rgb_ezo_calibration(&dev) == RGB_EZO_OK)
	{
		puts("[OK]");
	}
	else
	{
		puts("[Failed]");
		return -1;
	}

	printf("Setting gamma correction to 1,99... ");
	if (rgb_ezo_set_gamma_correction(&dev, 199) == RGB_EZO_OK)
	{
		puts("[OK]");
	}
	else
	{
		puts("[Failed]");
		return -1;
	}

	data1 = 0;
	/* Check if the gamma correction value is correctly set */
	printf("Getting gamma correction value, should be 199... ");
	if (rgb_ezo_get_gamma_correction(&dev, &data1) == RGB_EZO_OK && data1 == 199)
	{
		puts("[OK]");
	}
	else
	{
		puts("[Failed]");
		return -1;
	}

	printf("Disabling all the parameters... ");
	if (rgb_ezo_set_parameters_state(&dev, RGB_EZO_RGB, false) == RGB_EZO_OK
			&& rgb_ezo_set_parameters_state(&dev, RGB_EZO_LUX, false) == RGB_EZO_OK
			&& rgb_ezo_set_parameters_state(&dev, RGB_EZO_CIE, false) == RGB_EZO_OK)
	{
		puts("[OK]");
	}
	else
	{
		puts("[Failed]");
		return -1;
	}

//	printf("Reading enabled parameters, should be 'No parameter enabled'... ");
//	if (rgb_ezo_get_parameter_state(&dev, string_data) == RGB_EZO_OK
//			&& strcmp(string_data, "No parameter enabled") == 0)
//	{
//		puts("[OK]");
//	}
//	else
//	{
//		puts("[Failed]");
//		return -1;
//	}

	printf("Enabling all the parameters... ");
	if (rgb_ezo_set_parameters_state(&dev, RGB_EZO_RGB, true) == RGB_EZO_OK
			&& rgb_ezo_set_parameters_state(&dev, RGB_EZO_LUX, true) == RGB_EZO_OK
			&& rgb_ezo_set_parameters_state(&dev, RGB_EZO_CIE, true) == RGB_EZO_OK)
	{
		puts("[OK]");
	}
	else
	{
		puts("[Failed]");
		return -1;
	}

	printf("Reading enabled parameters... ");
	if (rgb_ezo_get_parameter_state(&dev, string_data) == RGB_EZO_OK)
	{
		puts("[OK]");
		printf("enabled %s\n", string_data);
	}
	else
	{
		puts("[Failed]");
		return -1;
	}

//    if (dev.params.alarm_int_pin != GPIO_UNDEF) {
//        /* Disable alarm */
//        printf("Disabling alarm... ");
//        if (rgb_ezo_disable_alarm(&dev) == RGB_EZO_OK) {
//            puts("[OK]");
//        }
//        else {
//            puts("[Failed]");
//            return -1;
//        }
//
//        /* Enable alarm and set the alarm value to 380 and tolerance value to 10 */
//        printf("Enabling alarm... ");
//        if (rgb_ezo_enable_alarm(&dev, 380, 10, alarm_int_pin_callback,
//                                 NULL) == RGB_EZO_OK) {
//            puts("[OK]");
//        }
//        else {
//            puts("[Failed]");
//            return -1;
//        }
//
//        /* Get RGB EZO alarm state */
//        printf("Getting alarm state... ");
//        if (rgb_ezo_get_alarm_state(&dev, &data, &data2,
//                                    &data3) == RGB_EZO_OK) {
//            puts("[OK]");
//            xtimer_sleep(2);
//            printf("Alarm value is %d ppm\n", data);
//            printf("Tolerance value is %d ppm\n", data2);
//            printf("Alarm is %s\n", data3 ? "enabled" : "disabled");
//
//        }
//        else {
//            puts("[Reading RGB alarm state Failed]");
//            return -1;
//        }
//    }

	while (1)
	{
		puts("\n[MAIN - Initiate reading]");

		if (rgb_ezo_read_rgb(&dev, &data1, &data2, &data3) == RGB_EZO_OK)
		{
			printf("R:%d, G:%d, B:%d\n", data1, data2, data3);
		}

		else
		{
			puts("[Reading RGB failed]");
		}
		xtimer_sleep(SLEEP_SEC);
	}
	return 0;

}
