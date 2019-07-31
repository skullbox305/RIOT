/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_ezo_oem
 * @{
 *
 * @file
 * @brief       CO2 EZO device driver
 *
 * @author      Ting XU <timtsui@outlook.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 * @}
 */

#include "xtimer.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "periph/gpio.h"

#include "assert.h"
#include "periph/i2c.h"

#include "co2_ezo.h"
#include "co2_ezo_internal.h"
#include "co2_ezo_params.h"

#define ENABLE_DEBUG    (1)
#include "debug.h"

#define I2C (dev->params.i2c)
#define ADDR (dev->params.addr)

/* private function declaration */
static int _cmd_process_wait(co2_ezo_t *dev);
static int _read_i2c_ezo(co2_ezo_t *dev, char *result_data);
static int _write_i2c_ezo(co2_ezo_t *dev, uint8_t *command, size_t *size);
static int _set_alarm_value(co2_ezo_t *dev, uint16_t *value);
static int _set_alarm_tolerance(co2_ezo_t *dev, uint16_t *tolerance);
static int _enable_alarm(co2_ezo_t *dev);
int _parse_string_to_array(char *input_string, char *delimiter,
		char **output_string_array, uint8_t amount_of_values);

/* Public functions */
int co2_ezo_init(co2_ezo_t *dev, const co2_ezo_params_t *params)
{
	/* check if dev and params == NULL */
	assert(dev && params);

	int result = 0;
	char result_data_substring[3];

	/* assign params argument to params of the device descriptor */
	dev->params = *params;

	/* allocate a char array with size 20 to hold the co2 ezo data which is read from
	 * the device */
	char *result_data = malloc(20 * sizeof(char));

	/* Lock the i2c bus so other code parts cant use it (mutex) */
	i2c_acquire(I2C);

	/* define an int array for the co2 ezo command */
	uint8_t cmd[] = CO2_EZO_DEV_INFO;
	size_t size = sizeof(cmd) / sizeof(uint8_t);

	/* write the command to the co2 ezo */
	if (_write_i2c_ezo(dev, (uint8_t *) cmd, &size) < 0)
	{
		result = CO2_EZO_WRITE_ERR;
		goto RETURN;
	}

	/* enable the internal temperature*/
	char cmd_temp[6] = "O,t,1";
	size_t size_temp = sizeof(cmd_temp) / sizeof(uint8_t);

	if (_write_i2c_ezo(dev, (uint8_t *) cmd_temp, &size_temp) < 0)
	{
		result = CO2_EZO_WRITE_ERR;
		goto RETURN;
	}

	/* read the data from the co2 ezo */
	if (_read_i2c_ezo(dev, result_data) < 0)
	{
		result = CO2_EZO_READ_ERR;
		goto RETURN;
	}

	strncpy(result_data_substring, result_data + 4, 3);
	char device_type[] = "CO2";
	if (strcmp(result_data_substring, device_type) == 0)
	{
		result = CO2_EZO_OK;
	}
	else
	{
		DEBUG("data: %.*s\n", 3, result_data + 4);
		result = CO2_EZO_NOT_CO2;
	}

	RETURN: i2c_release(I2C);
	free(result_data);

	return result;
}

int co2_ezo_set_led_state(co2_ezo_t *dev, co2_ezo_led_state_t state)
{
	assert(dev);
	i2c_acquire(I2C);

	char cmd[3];
	sprintf(cmd, "%s%d", CO2_EZO_LED, state);
	size_t size = strlen(cmd);

	if (_write_i2c_ezo(dev, (uint8_t *) cmd, &size) < 0)
	{
		i2c_release(I2C);
		return CO2_EZO_WRITE_ERR;
	}

	/** blocks till command finishes */
	if (_cmd_process_wait(dev) < 0)
	{
		i2c_release(I2C);
		return CO2_EZO_READ_ERR;
	}

	i2c_release(I2C);

	return CO2_EZO_OK;
}

int co2_ezo_set_i2c_address(co2_ezo_t *dev, uint8_t addr)
{
	assert(dev);
	i2c_acquire(I2C);

	char command[7];
	sprintf(command, "%s%d", CO2_EZO_I2C_ADDR_SET, addr);
	size_t size = strlen(command);

	if (_write_i2c_ezo(dev, (uint8_t *) command, &size) < 0)
	{
		DEBUG("\n[co2_ezo debug] Setting I2C address to %x failed\n", addr);
		i2c_release(I2C);
		return CO2_EZO_WRITE_ERR;
	}
	dev->params.addr = addr;
	i2c_release(I2C);

	xtimer_usleep(2000 * US_PER_MS);

	return CO2_EZO_OK;
}

int co2_ezo_enable_alarm(co2_ezo_t *dev, uint16_t value, uint16_t tolerance,
		co2_ezo_interrupt_pin_cb_t cb, void *arg)
{
	assert(dev);
	if (dev->params.alarm_int_pin == GPIO_UNDEF)
	{
		return CO2_EZO_INT_GPIO_UNDEF;
	}

	if (value > 10000)
	{
		DEBUG("[co2_ezo debug] Value out of range\n");
		return CO2_EZO_OUT_OF_RANGE;
	}

	if (tolerance >= 500)
	{
		DEBUG("[co2_ezo debug] Tolerance out of range\n");
		return CO2_EZO_OUT_OF_RANGE;
	}

	i2c_acquire(I2C);

	_set_alarm_value(dev, &value);
	_set_alarm_tolerance(dev, &tolerance);
	_enable_alarm(dev);

	if (gpio_init_int(dev->params.alarm_int_pin, GPIO_IN_PD, GPIO_BOTH, cb, arg)
			< 0)
	{
		DEBUG("\n[co2 ezo debug] Initializing interrupt gpio pin failed.\n");
		return CO2_EZO_GPIO_INIT_ERR;
	}

	i2c_release(I2C);
	return CO2_EZO_OK;
}

int co2_ezo_get_alarm_state(co2_ezo_t *dev, uint16_t *alarm_value,
		uint16_t *tolerance_value, bool *enabled)
{
	assert(dev);

	char *result_data = malloc(20 * sizeof(char));

	uint8_t alarm_state_cmd[] = CO2_EZO_ALARM_STATE;
	size_t size = sizeof(alarm_state_cmd) / sizeof(uint8_t);

	i2c_acquire(I2C);

	if (_write_i2c_ezo(dev, (uint8_t *) alarm_state_cmd, &size) < 0)
	{
		i2c_release(I2C);
		free(result_data);
		return CO2_EZO_WRITE_ERR;
	}

	/* reads the data from the co2 ezo */
	if (_read_i2c_ezo(dev, result_data) < 0)
	{
		i2c_release(I2C);
		free(result_data);
		return CO2_EZO_WRITE_ERR;
	}

	char *parsed_values[3];
	_parse_string_to_array(result_data + 8, ",", parsed_values, 3);
	*alarm_value = atoi(parsed_values[0]);
	*tolerance_value = atoi(parsed_values[1]);

	if (strcmp(parsed_values[2], "1") == 0)
	{
		*enabled = true;
	}
	else
	{
		*enabled = false;
	}

	i2c_release(I2C);
	free(result_data);
	return CO2_EZO_OK;

}

int co2_ezo_disable_alarm(co2_ezo_t *dev)
{
	assert(dev);
	i2c_acquire(I2C);

	uint8_t cmd[] = CO2_EZO_ALARM_OFF;
	size_t size = sizeof(cmd) / sizeof(uint8_t);

	if (_write_i2c_ezo(dev, (uint8_t *) cmd, &size) < 0)
	{
		DEBUG("\n[co2_ezo debug] Disabling alarm failed\n");
		i2c_release(I2C);
		return CO2_EZO_WRITE_ERR;
	}

	/** blocks till command finishes */
	if (_cmd_process_wait(dev) < 0)
	{
		i2c_release(I2C);
		return CO2_EZO_READ_ERR;
	}
	i2c_release(I2C);

	return CO2_EZO_OK;
}

int co2_ezo_sleep_mode(co2_ezo_t *dev)
{
	assert(dev);
	i2c_acquire(I2C);

	uint8_t cmd[] = CO2_EZO_SLEEP_MODE;
	size_t size = sizeof(cmd) / sizeof(uint8_t);

	if (_write_i2c_ezo(dev, (uint8_t *) cmd, &size) < 0)
	{
		DEBUG("\n[co2_ezo debug] Enable sleep mode failed\n");
		i2c_release(I2C);
		return CO2_EZO_WRITE_ERR;
	}

	/** blocks till command finishes */
	if (_cmd_process_wait(dev) < 0)
	{
		i2c_release(I2C);
		return CO2_EZO_READ_ERR;
	}

	i2c_release(I2C);

	return CO2_EZO_OK;
}

int co2_ezo_read_co2(co2_ezo_t *dev, uint16_t *co2_value,
		uint32_t *internal_temperature)
{
	assert(dev);

	int8_t result = CO2_EZO_OK;

	char *result_data = malloc(5 * sizeof(char));

	uint8_t read_cmd[] = CO2_EZO_TAKE_READING;
	size_t size = sizeof(read_cmd) / sizeof(uint8_t);

	i2c_acquire(I2C);

	if (_write_i2c_ezo(dev, (uint8_t *) read_cmd, &size) < 0)
	{
		result = CO2_EZO_WRITE_ERR;
		goto RETURN;
	}

	if (_read_i2c_ezo(dev, result_data) < 0)
	{
		result = CO2_EZO_READ_ERR;
		goto RETURN;
	}

	/* the sensor must warm-up before it can output readings, return the data
	 * while CO2 sensor is warmed-up */
	if (strcmp(result_data + 1, "*WARM") == 0)
	{
		result = CO2_EZO_WARMING;
		goto RETURN;
	}

	else
	{
		char *parsed_values[2] =
		{ "" };
		_parse_string_to_array(result_data + 1, ",", parsed_values, 2);
		*co2_value = atoi(parsed_values[0]);

		char *parsed_temp_values[2] =
		{ "" };
		_parse_string_to_array(parsed_values[1], ".", parsed_temp_values, 2);
		*internal_temperature = atoi(parsed_temp_values[0]) * 1000
				+ atoi(parsed_temp_values[1]);
	}

	RETURN: i2c_release(I2C);
	free(result_data);

	return result;
}

/* Private functions */
static int _cmd_process_wait(co2_ezo_t *dev)
{
	char *response_code = malloc(2 * sizeof(char));

	if (_read_i2c_ezo(dev, response_code))
	{
		return CO2_EZO_READ_ERR;
	}
	free(response_code);

	return CO2_EZO_OK;
}

static int _read_i2c_ezo(co2_ezo_t *dev, char *result_data)
{
	uint8_t response_code;

	do
	{
		xtimer_usleep(100 * US_PER_MS);
		if (i2c_read_bytes(I2C, ADDR, result_data, 20, 0x00) < 0)
		{
			DEBUG("\n[co2_ezo debug] Read from co2 ezo failed \n");
			return CO2_EZO_READ_ERR;
		}
		response_code = result_data[0];
	} while (response_code != CO2_EZO_SUCCESS);

	return CO2_EZO_OK;
}

static int _write_i2c_ezo(co2_ezo_t *dev, uint8_t *command, size_t *size)
{
	if (i2c_write_bytes(I2C, ADDR, command, *size, 0x0) < 0)
	{
		DEBUG("\n[co2_ezo debug] Writing to co2 ezo failed \n");
		return CO2_EZO_WRITE_ERR;
	}

	return CO2_EZO_OK;
}

static int _set_alarm_value(co2_ezo_t *dev, uint16_t *value)
{
	/*sets the alarm value after the alarm mode is enabled */
	char alarm_cmd[12];

	sprintf(alarm_cmd, "%s%d", CO2_EZO_ALARM_SET, *value);
	size_t value_size = strlen(alarm_cmd);

	if (_write_i2c_ezo(dev, (uint8_t *) alarm_cmd, &value_size) < 0)
	{
		DEBUG("\n[co2_ezo debug] Setting alarm to %d failed\n", *value);
		i2c_release(I2C);
		return CO2_EZO_WRITE_ERR;
	}

	/** blocks till command finishes */
	if (_cmd_process_wait(dev) < 0)
	{
		i2c_release(I2C);
		return CO2_EZO_READ_ERR;
	}

	return CO2_EZO_OK;
}

static int _set_alarm_tolerance(co2_ezo_t *dev, uint16_t *tolerance)
{
	/*sets tolerance value after alarm is enabled */
	char tol_cmd[14];

	sprintf(tol_cmd, "%s%d", CO2_EZO_ALARM_SET_TOL, *tolerance);
	size_t tol_size = strlen(tol_cmd);

	if (_write_i2c_ezo(dev, (uint8_t *) tol_cmd, &tol_size) < 0)
	{
		DEBUG("\n[co2_ezo debug] Setting alarm tolerance to %d failed\n",
				*tolerance);
		i2c_release(I2C);
		return CO2_EZO_WRITE_ERR;
	}

	/** blocks till command finishes */
	if (_cmd_process_wait(dev) < 0)
	{
		i2c_release(I2C);
		return CO2_EZO_READ_ERR;
	}

	return CO2_EZO_OK;
}

static int _enable_alarm(co2_ezo_t *dev)
{
	/*enable alarm mode*/
	uint8_t command[] = CO2_EZO_ALARM_ON;
	size_t size = sizeof(command) / sizeof(uint8_t);

	if (_write_i2c_ezo(dev, (uint8_t *) command, &size) < 0)
	{
		DEBUG("\n[co2_ezo debug] Enabling alarm failed\n");
		i2c_release(I2C);
		return CO2_EZO_WRITE_ERR;
	}

	/** blocks till command finishes */
	if (_cmd_process_wait(dev) < 0)
	{
		i2c_release(I2C);
		return CO2_EZO_READ_ERR;
	}

	return CO2_EZO_OK;
}

int _parse_string_to_array(char *input_string, char *delimiter,
		char **output_string_array, uint8_t amount_of_values)
{
	char *ptr;
	int i = 0;

	ptr = strtok(input_string, delimiter);

	while (ptr != NULL && i < amount_of_values)
	{
		output_string_array[i] = ptr;
		ptr = strtok(NULL, delimiter);
		i++;
	}

	return CO2_EZO_OK;
}
