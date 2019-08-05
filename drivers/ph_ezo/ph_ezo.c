/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_ph_ezo_oem
 * @{
 *
 * @file
 * @brief       pH EZO device driver
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
#include "fmt.h"

#include "ph_ezo.h"
#include "ph_ezo_internal.h"
#include "ph_ezo_params.h"

#define ENABLE_DEBUG    (1)
#include "debug.h"

#define I2C (dev->params.i2c)
#define ADDR (dev->params.addr)

/* private function declaration */
static int _cmd_process_wait(ph_ezo_t *dev);
static int _read_i2c_ezo(ph_ezo_t *dev, char *result_data);
static int _write_i2c_ezo(ph_ezo_t *dev, uint8_t *command, size_t *size);
int _parse_string_to_array(char *input_string, char *delimiter,
		char **output_string_array, uint8_t amount_of_values);

/* Public functions */
int ph_ezo_init(ph_ezo_t *dev, const ph_ezo_params_t *params)
{
	/* check if dev and params == NULL */
	assert(dev && params);

	int result = 0;
	char result_data_substring[2];

	/* assign params argument to params of the device descriptor */
	dev->params = *params;

	/* allocate a char array with size 20 to hold the ph ezo data which is read from
	 * the device */
	char *result_data = malloc(20 * sizeof(char));

	/* Lock the i2c bus so other code parts cant use it (mutex) */
	i2c_acquire(I2C);

	/* define an int array for the ph ezo command */
	uint8_t cmd[] = PH_EZO_DEV_INFO;
	size_t size = sizeof(cmd) / sizeof(uint8_t);

	/* write the command to the ph ezo */
	if (_write_i2c_ezo(dev, (uint8_t *) cmd, &size) < 0)
	{
		result = PH_EZO_WRITE_ERR;
		goto RETURN;
	}

	/* read the data from the ph ezo */
	if (_read_i2c_ezo(dev, result_data) < 0)
	{
		result = PH_EZO_READ_ERR;
		goto RETURN;
	}

	strncpy(result_data_substring, result_data + 4, 2);
	char device_type[] = "pH";
	if (strcmp(result_data_substring, device_type) == 0)
	{
		result = PH_EZO_OK;
	}
	else
	{
		DEBUG("data: %.*s\n", 2, result_data + 4);
		result = PH_EZO_NOT_PH;
	}

	RETURN: i2c_release(I2C);
	free(result_data);

	return result;
}

int ph_ezo_reset(const ph_ezo_t *dev)
{
	assert(dev);
	i2c_acquire(I2C);

	uint8_t cmd[] = PH_EZO_FACTORY_RESET;
	size_t size = sizeof(cmd) / sizeof(uint8_t);

	if (_write_i2c_ezo(dev, (uint8_t *) cmd, &size) < 0)
	{
		DEBUG("\n[ph_ezo debug] Enable factory reset failed\n");
		i2c_release(I2C);
		return PH_EZO_WRITE_ERR;
	}

	i2c_release(I2C);
	return PH_EZO_OK;
}

int ph_ezo_set_led_state(ph_ezo_t *dev, ph_ezo_led_state_t state)
{
	assert(dev);
	i2c_acquire(I2C);

	char cmd[3];
	sprintf(cmd, "%s%d", PH_EZO_LED, state);
	size_t size = strlen(cmd);

	if (_write_i2c_ezo(dev, (uint8_t *) cmd, &size) < 0)
	{
		i2c_release(I2C);
		return PH_EZO_WRITE_ERR;
	}

	/** blocks till command finishes */
	if (_cmd_process_wait(dev) < 0)
	{
		i2c_release(I2C);
		return PH_EZO_READ_ERR;
	}

	i2c_release(I2C);

	return PH_EZO_OK;
}

int ph_ezo_set_i2c_address(ph_ezo_t *dev, uint8_t addr)
{
	assert(dev);
	i2c_acquire(I2C);

	char command[7];
	sprintf(command, "%s%d", PH_EZO_I2C_ADDR_SET, addr);
	size_t size = strlen(command);

	if (_write_i2c_ezo(dev, (uint8_t *) command, &size) < 0)
	{
		DEBUG("\n[ph_ezo debug] Setting I2C address to %x failed\n", addr);
		i2c_release(I2C);
		return PH_EZO_WRITE_ERR;
	}
	dev->params.addr = addr;
	i2c_release(I2C);

	xtimer_usleep(2000 * US_PER_MS);

	return PH_EZO_OK;
}

int ph_ezo_clear_calibration(const ph_ezo_t *dev)
{
	assert(dev);
	i2c_acquire(I2C);

	uint8_t cmd[] = PH_EZO_CALIBRATE_CLEAR;
	size_t size = sizeof(cmd) / sizeof(uint8_t);

	if (_write_i2c_ezo(dev, (uint8_t *) cmd, &size) < 0)
	{
		DEBUG("\n[ph_ezo debug] Clearing calibration failed\n");
		i2c_release(I2C);
		return PH_EZO_WRITE_ERR;
	}

	i2c_release(I2C);
	return PH_EZO_OK;
}

int ph_ezo_set_calibration(const ph_ezo_t *dev, uint16_t calibration_value,
		ph_ezo_calibration_option_t option)
{
	assert(dev);

	char calibration_cmd[15];

	if (option == PH_EZO_CALIBRATE_MID_POINT)
	{
//    	float value;
//    	value = calibration_value / 100;

//    	char calibration_value_float_string[7];
//    	fmt_float(&calibration_value_float_string, ((float)calibration_value) / 100.0, 3)

		sprintf(calibration_cmd, "%s%f", PH_EZO_CALIBRATE_MID,
				((float) calibration_value) / 100.0);
		size_t value_size = strlen(calibration_cmd);

		if (_write_i2c_ezo(dev, (uint8_t *) calibration_cmd, &value_size) < 0)
		{
			DEBUG(
					"\n[ph_ezo debug] Setting calibration at midpoint %f failed\n",
					((float )calibration_value) / 100.0);
			i2c_release(I2C);
			return PH_EZO_WRITE_ERR;
		}

		/** blocks till command finishes */
		if (_cmd_process_wait(dev) < 0)
		{
			i2c_release(I2C);
			return PH_EZO_READ_ERR;
		}

	}

	if (option == PH_EZO_CALIBRATE_LOW_POINT)
	{
//    	float value;
//    	value = calibration_value / 100;

//    	char calibration_value_float_string[7];
//    	fmt_float(&calibration_value_float_string, ((float)calibration_value) / 100.0, 3)

		sprintf(calibration_cmd, "%s%f", PH_EZO_CALIBRATE_LOW,
				((float) calibration_value) / 100.0);
		size_t value_size = strlen(calibration_cmd);

		if (_write_i2c_ezo(dev, (uint8_t *) calibration_cmd, &value_size) < 0)
		{
			DEBUG(
					"\n[ph_ezo debug] Setting calibration at lowpoint %f failed\n",
					((float )calibration_value) / 100.0);
			i2c_release(I2C);
			return PH_EZO_WRITE_ERR;
		}

		/** blocks till command finishes */
		if (_cmd_process_wait(dev) < 0)
		{
			i2c_release(I2C);
			return PH_EZO_READ_ERR;
		}

	}

	if (option == PH_EZO_CALIBRATE_HIGH_POINT)
	{
//    	float value;
//    	value = calibration_value / 100;

//    	char calibration_value_float_string[7];
//    	fmt_float(&calibration_value_float_string, ((float)calibration_value) / 100.0, 3)

		sprintf(calibration_cmd, "%s%f", PH_EZO_CALIBRATE_HIGH,
				((float) calibration_value) / 100.0);
		size_t value_size = strlen(calibration_cmd);

		if (_write_i2c_ezo(dev, (uint8_t *) calibration_cmd, &value_size) < 0)
		{
			DEBUG(
					"\n[ph_ezo debug] Setting calibration at midpoint %f failed\n",
					((float )calibration_value) / 100.0);
			i2c_release(I2C);
			return PH_EZO_WRITE_ERR;
		}

		/** blocks till command finishes */
		if (_cmd_process_wait(dev) < 0)
		{
			i2c_release(I2C);
			return PH_EZO_READ_ERR;
		}

	}
	return PH_EZO_OK;

}

int ph_ezo_get_calibration_state(ph_ezo_t *dev,
		uint16_t *cal_state)
{
	assert(dev);

	char *result_data = malloc(20 * sizeof(char));
	char result_data_substring[1];

	uint8_t calibration_state_cmd[] = PH_EZO_CALIBRATE_STATE;
	size_t size = sizeof(calibration_state_cmd) / sizeof(uint8_t);

	i2c_acquire(I2C);

	if (_write_i2c_ezo(dev, (uint8_t *) calibration_state_cmd, &size) < 0)
	{
		i2c_release(I2C);
		free(result_data);
		return PH_EZO_WRITE_ERR;
	}

	/* reads the data from the ph ezo */
	if (_read_i2c_ezo(dev, result_data) < 0)
	{
		i2c_release(I2C);
		free(result_data);
		return PH_EZO_WRITE_ERR;
	}

	strncpy(result_data_substring, result_data + 6, 1);
	*cal_state = atoi(result_data_substring[0]);

	i2c_release(I2C);
	free(result_data);
	return PH_EZO_OK;

}

int ph_ezo_sleep_mode(ph_ezo_t *dev)
{
	assert(dev);
	i2c_acquire(I2C);

	uint8_t cmd[] = PH_EZO_SLEEP_MODE;
	size_t size = sizeof(cmd) / sizeof(uint8_t);

	if (_write_i2c_ezo(dev, (uint8_t *) cmd, &size) < 0)
	{
		DEBUG("\n[ph_ezo debug] Enable sleep mode failed\n");
		i2c_release(I2C);
		return PH_EZO_WRITE_ERR;
	}

	/** blocks till command finishes */
	if (_cmd_process_wait(dev) < 0)
	{
		i2c_release(I2C);
		return PH_EZO_READ_ERR;
	}

	i2c_release(I2C);

	return PH_EZO_OK;
}

int ph_ezo_read_slope(ph_ezo_t *dev, uint16_t *acid_slope, uint16_t *base_slope)
{
	assert(dev);

	char *result_data = malloc(20 * sizeof(char));

	uint8_t slope_cmd[] = PH_EZO_SLOPE;
	size_t size = sizeof(slope_cmd) / sizeof(uint8_t);

	i2c_acquire(I2C);

	if (_write_i2c_ezo(dev, (uint8_t *) slope_cmd, &size) < 0)
	{
		i2c_release(I2C);
		free(result_data);
		return PH_EZO_WRITE_ERR;
	}

	/* reads the data from the ph ezo */
	if (_read_i2c_ezo(dev, result_data) < 0)
	{
		i2c_release(I2C);
		free(result_data);
		return PH_EZO_WRITE_ERR;
	}

	char parsed_values[2] =
	{ "" };
	_parse_string_to_array(result_data + 8, ",", parsed_values, 2);

	char *parsed_acid_values[2] =
	{ "" };
	_parse_string_to_array(parsed_values[0], ".", parsed_acid_values, 2);
	*acid_slope = atoi(parsed_acid_values[0]) | atoi(parsed_acid_values[1]);

	char *parsed_base_values[2] =
	{ "" };
	_parse_string_to_array(parsed_values[1], ".", parsed_base_values, 2);
	*base_slope = atoi(parsed_acid_values[0]) | atoi(parsed_acid_values[1]);

	i2c_release(I2C);
	free(result_data);
	return PH_EZO_OK;
}

int ph_ezo_read_ph(ph_ezo_t *dev, uint16_t *ph_value)
{
	assert(dev);

	int8_t result = PH_EZO_OK;

	char *result_data = malloc(6 * sizeof(char));

	uint8_t read_cmd[] = PH_EZO_TAKE_READING;
	size_t size = sizeof(read_cmd) / sizeof(uint8_t);

	i2c_acquire(I2C);

	if (_write_i2c_ezo(dev, (uint8_t *) read_cmd, &size) < 0)
	{
		result = PH_EZO_WRITE_ERR;
		goto RETURN;
	}

	if (_read_i2c_ezo(dev, result_data) < 0)
	{
		result = PH_EZO_READ_ERR;
		goto RETURN;
	}

	char *parsed_values[2] =
	{ "" };

	_parse_string_to_array(result_data + 1, ".", parsed_values, 2);
	*ph_value = atoi(parsed_values[0])|atoi(parsed_values[1]);

	RETURN: i2c_release(I2C);
	free(result_data);

	return result;
}

/* Private functions */
static int _cmd_process_wait(ph_ezo_t *dev)
{
	char *response_code = malloc(2 * sizeof(char));

	if (_read_i2c_ezo(dev, response_code))
	{
		return PH_EZO_READ_ERR;
	}
	free(response_code);

	return PH_EZO_OK;
}

static int _read_i2c_ezo(ph_ezo_t *dev, char *result_data)
{
	uint8_t response_code;

	do
	{
		xtimer_usleep(100 * US_PER_MS);
		if (i2c_read_bytes(I2C, ADDR, result_data, 20, 0x00) < 0)
		{
			DEBUG("\n[ph_ezo debug] Read from ph ezo failed \n");
			return PH_EZO_READ_ERR;
		}
		response_code = result_data[0];
	} while (response_code != PH_EZO_SUCCESS);

	return PH_EZO_OK;
}

static int _write_i2c_ezo(ph_ezo_t *dev, uint8_t *command, size_t *size)
{
	if (i2c_write_bytes(I2C, ADDR, command, *size, 0x0) < 0)
	{
		DEBUG("\n[ph_ezo debug] Writing to ph ezo failed \n");
		return PH_EZO_WRITE_ERR;
	}

	return PH_EZO_OK;
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

	return PH_EZO_OK;
}
