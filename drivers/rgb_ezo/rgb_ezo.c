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
 * @brief       RGB EZO device driver
 *
 * @author      Ting XU <timtsui@outlook.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 * @}
 */

#include "xtimer.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "fmt.h"
#include "periph/gpio.h"

#include "assert.h"
#include "periph/i2c.h"

#include "rgb_ezo.h"
#include "include/rgb_ezo_internal.h"
#include "include/rgb_ezo_params.h"

#define ENABLE_DEBUG    (1)
#include "debug.h"

#define I2C (dev->params.i2c)
#define ADDR (dev->params.addr)

/* private function declaration */
static int _cmd_process_wait(rgb_ezo_t *dev);
static int _read_i2c_ezo(rgb_ezo_t *dev, char *result_data);
static int _write_i2c_ezo(rgb_ezo_t *dev, uint8_t *command, size_t *size);
int _parse_string_to_array(char *input_string, char *delimiter,
		char **output_string_array, uint8_t amount_of_values);
int _insert_to_array_in_mid(char *input_string, char *cmd_string,
		char *output_string);

/* Public functions */
int rgb_ezo_init(rgb_ezo_t *dev, const rgb_ezo_params_t *params)
{
	/* check if dev and params == NULL */
	assert(dev && params);

	int result = 0;
	char result_data_substring[3];

	/* assign params argument to params of the device descriptor */
	dev->params = *params;

	/* allocate a char array with size 20 to hold the rgb ezo data which is read from
	 * the device */
	char *result_data = malloc(20 * sizeof(char));

	/* Lock the i2c bus so other code parts cant use it (mutex) */
	i2c_acquire(I2C);

	/* define an int array for the rgb ezo command */
	uint8_t cmd[] = RGB_EZO_DEV_INFO;
	size_t size = sizeof(cmd) / sizeof(uint8_t);

	/* write the command to the rgb ezo */
	if (_write_i2c_ezo(dev, (uint8_t *) cmd, &size) < 0)
	{
		result = RGB_EZO_WRITE_ERR;
		goto RETURN;
	}

	/* read the data from the rgb ezo */
	if (_read_i2c_ezo(dev, result_data) < 0)
	{
		result = RGB_EZO_READ_ERR;
		goto RETURN;
	}

	strncpy(result_data_substring, result_data + 4, 3);
	char device_type[] = "RGB";
	if (strcmp(result_data_substring, device_type) == 0)
	{
		result = RGB_EZO_OK;
	}
	else
	{
		DEBUG("data: %.*s\n", 3, result_data + 4);
		result = RGB_EZO_NOT_RGB;
	}

	RETURN: i2c_release(I2C);
	free(result_data);

	return result;
}

int rgb_ezo_set_indicator_led_state(rgb_ezo_t *dev, rgb_ezo_led_state_t state)
{
	assert(dev);
	i2c_acquire(I2C);

	char cmd[3];
	sprintf(cmd, "%s%d", RGB_EZO_INDICATOR_LED, state);
	size_t size = strlen(cmd);

	if (_write_i2c_ezo(dev, (uint8_t *) cmd, &size) < 0)
	{
		i2c_release(I2C);
		return RGB_EZO_WRITE_ERR;
	}

	/** blocks till command finishes */
	if (_cmd_process_wait(dev) < 0)
	{
		i2c_release(I2C);
		return RGB_EZO_READ_ERR;
	}

	i2c_release(I2C);

	return RGB_EZO_OK;
}

int rgb_ezo_set_i2c_address(rgb_ezo_t *dev, uint8_t addr)
{
	assert(dev);
	i2c_acquire(I2C);

	char command[7];
	sprintf(command, "%s%d", RGB_EZO_I2C_ADDR_SET, addr);
	size_t size = strlen(command);

	if (_write_i2c_ezo(dev, (uint8_t *) command, &size) < 0)
	{
		DEBUG("\n[rgb_ezo debug] Setting I2C address to %x failed\n", addr);
		i2c_release(I2C);
		return RGB_EZO_WRITE_ERR;
	}
	dev->params.addr = addr;
	i2c_release(I2C);

	xtimer_usleep(2000 * US_PER_MS);

	return RGB_EZO_OK;
}

int rgb_ezo_calibration(rgb_ezo_t *dev)
{
	assert(dev);

	uint8_t calibration_cmd[] = RGB_EZO_CALIBRATE;
	size_t size = sizeof(calibration_cmd) / sizeof(uint8_t);

	i2c_acquire(I2C);

	if (_write_i2c_ezo(dev, (uint8_t *) calibration_cmd, &size) < 0)
	{
		i2c_release(I2C);

		return RGB_EZO_WRITE_ERR;
	}

	/** blocks till command finishes */
	if (_cmd_process_wait(dev) < 0)
	{
		i2c_release(I2C);
		return RGB_EZO_READ_ERR;
	}

	i2c_release(I2C);

	return RGB_EZO_OK;
}

int rgb_ezo_gamma_correction(rgb_ezo_t *dev, uint16_t value)
{
	assert(dev);
	i2c_acquire(I2C);

	char command[7];
	char gamma_value_float_string[7];

	fmt_float(gamma_value_float_string, ((float)value) / 100.0, 2);

	sprintf(command, "%s%s", RGB_EZO_GAMMA_CORRECTION,
			gamma_value_float_string);
	size_t size = strlen(command);

	if (_write_i2c_ezo(dev, (uint8_t *) command, &size) < 0)
	{
		DEBUG("\n[rgb_ezo debug] Setting gamma correction to %d failed\n",
				value);
		i2c_release(I2C);
		return RGB_EZO_WRITE_ERR;
	}
	i2c_release(I2C);

	xtimer_usleep(2000 * US_PER_MS);

	return RGB_EZO_OK;
}

int rgb_ezo_get_gamma_correction(rgb_ezo_t *dev, uint16_t *correction_value)
{
	assert(dev);

	int result = 0;

	char *result_data = malloc(10 * sizeof(char));
	i2c_acquire(I2C);

	uint8_t cmd[] = RGB_EZO_GAMMA_VALUE;
	size_t size = sizeof(cmd) / sizeof(uint8_t);

	if (_write_i2c_ezo(dev, (uint8_t *) cmd, &size) < 0)
	{
		result = RGB_EZO_WRITE_ERR;
		goto RETURN;
	}

	if (_read_i2c_ezo(dev, result_data) < 0)
	{
		result = RGB_EZO_READ_ERR;
		goto RETURN;
	}

	char *parsed_values[2];
	char result_data_substring[5];

	strncpy(result_data_substring, result_data + 4, 4);
	_parse_string_to_array(result_data_substring, ".", parsed_values, 2);
	*correction_value = atoi(parsed_values[0]) * 100 + atoi(parsed_values[1]);

	RETURN: i2c_release(I2C);
	free(result_data);

	return result;
}


int rgb_ezo_set_parameters_state(rgb_ezo_t *dev, rgb_ezo_parameter_t parameter,
bool enabled)
{
	assert(dev);

	char command[8];
	i2c_acquire(I2C);

	switch (parameter)
	{
	case RGB_EZO_RGB:
		sprintf(command, "%s%s%d", RGB_EZO_SET_PARAMETER, "RGB,", enabled);
		break;
	case RGB_EZO_LUX:
		sprintf(command, "%s%s%d", RGB_EZO_SET_PARAMETER, "LUX,", enabled);
		break;
	case RGB_EZO_CIE:
		sprintf(command, "%s%s%d", RGB_EZO_SET_PARAMETER, "CIE,", enabled);
		break;
	default:
		break;
	}

	size_t size = sizeof(command) / sizeof(uint8_t);

	if (_write_i2c_ezo(dev, (uint8_t *) command, &size) < 0)
	{
		i2c_release(I2C);
		return RGB_EZO_WRITE_ERR;
	}

	i2c_release(I2C);

	return RGB_EZO_OK;
}

int rgb_ezo_get_parameter_state(rgb_ezo_t *dev, char *string)
{
	assert(dev);

	char *result_data = malloc(30 * sizeof(char));
	char enabled_parameter[16];

	uint8_t parameter_state_cmd[] = RGB_EZO__PARAMETER_STATE;
	size_t size = sizeof(parameter_state_cmd) / sizeof(uint8_t);

	i2c_acquire(I2C);

	if (_write_i2c_ezo(dev, (uint8_t *) parameter_state_cmd, &size) < 0)
	{
		i2c_release(I2C);
		free(result_data);
		return RGB_EZO_WRITE_ERR;
	}

	/* reads the data from the rgb ezo */
	if (_read_i2c_ezo(dev, result_data) < 0)
	{
		i2c_release(I2C);
		free(result_data);
		return RGB_EZO_READ_ERR;
	}

	char device_type[] = "1?0";
	if (strcmp(result_data, device_type) == 0)
	{
		strcpy(string, "No parameter enabled");
	}
	else
	{
		strncpy(enabled_parameter, result_data + 5, 12);
		strcpy(string, enabled_parameter);
	}

	i2c_release(I2C);
	return RGB_EZO_OK;
}

int rgb_ezo_sleep_mode(rgb_ezo_t *dev)
{
	assert(dev);
	i2c_acquire(I2C);

	uint8_t cmd[] = RGB_EZO_SLEEP_MODE;
	size_t size = sizeof(cmd) / sizeof(uint8_t);

	if (_write_i2c_ezo(dev, (uint8_t *) cmd, &size) < 0)
	{
		DEBUG("\n[rgb_ezo debug] Enable sleep mode failed\n");
		i2c_release(I2C);
		return RGB_EZO_WRITE_ERR;
	}

	/** blocks till command finishes */
	if (_cmd_process_wait(dev) < 0)
	{
		i2c_release(I2C);
		return RGB_EZO_READ_ERR;
	}

	i2c_release(I2C);

	return RGB_EZO_OK;
}

int rgb_ezo_read_rgb(rgb_ezo_t *dev, uint16_t *rgb_value)
{
	assert(dev);
	int8_t result = RGB_EZO_OK;

	char *result_data = malloc(20 * sizeof(char));
	uint8_t read_cmd[] = RGB_EZO_TAKE_READING;
	size_t size = sizeof(read_cmd) / sizeof(uint8_t);

	i2c_acquire(I2C);

	if (_write_i2c_ezo(dev, (uint8_t *) read_cmd, &size) < 0)
	{
		result = RGB_EZO_WRITE_ERR;
		goto RETURN;
	}

	if (_read_i2c_ezo(dev, result_data) < 0)
	{
		result = RGB_EZO_READ_ERR;
		goto RETURN;
	}

	printf("raw data: %s\n", result_data);


	*rgb_value = 0;

	/* the sensor must warm-up before it can output readings, return the data
	 * while RGB sensor is warmed-up */
//	if (strcmp(result_data + 1, "*WARM") == 0)
//	{
//		result = RGB_EZO_WARMING;
//		goto RETURN;
//	}
//
//	else
//	{
//		char *parsed_values[2] =
//		{ "" };
//		_parse_string_to_array(result_data + 1, ",", parsed_values, 2);
//		*rgb_value = atoi(parsed_values[0]);
//
//		char *parsed_temp_values[2] =
//		{ "" };
//		_parse_string_to_array(parsed_values[1], ".", parsed_temp_values, 2);
//		*internal_temperature = atoi(parsed_temp_values[0]) * 1000
//				+ atoi(parsed_temp_values[1]);
//	}

	RETURN: i2c_release(I2C);
	free(result_data);

	return result;
}
//
//int rgb_ezo_read_rgb(rgb_ezo_t *dev, uint16_t *rgb_value)
//{
//	assert(dev);
//
//	int8_t result = RGB_EZO_OK;
//
//	char *result_data = malloc(5 * sizeof(char));
//
//	uint8_t read_cmd[] = RGB_EZO_TAKE_READING;
//	size_t size = sizeof(read_cmd) / sizeof(uint8_t);
//
//	i2c_acquire(I2C);
//
//	if (_write_i2c_ezo(dev, (uint8_t *) read_cmd, &size) < 0)
//	{
//		result = RGB_EZO_WRITE_ERR;
//		goto RETURN;
//	}
//
//	if (_read_i2c_ezo(dev, result_data) < 0)
//	{
//		result = RGB_EZO_READ_ERR;
//		goto RETURN;
//	}
//
//	/* the sensor must warm-up before it can output readings, return the data
//	 * while RGB sensor is warmed-up */
//	if (strcmp(result_data + 1, "*WARM") == 0)
//	{
//		result = RGB_EZO_WARMING;
//		goto RETURN;
//	}
//
//	else
//	{
//		char *parsed_values[2] =
//		{ "" };
//		_parse_string_to_array(result_data + 1, ",", parsed_values, 2);
//		*rgb_value = atoi(parsed_values[0]);
//
//		char *parsed_temp_values[2] =
//		{ "" };
//		_parse_string_to_array(parsed_values[1], ".", parsed_temp_values, 2);
//		*internal_temperature = atoi(parsed_temp_values[0]) * 1000
//				+ atoi(parsed_temp_values[1]);
//	}
//
//	RETURN: i2c_release(I2C);
//	free(result_data);
//
//	return result;
//}

/* Private functions */
static int _cmd_process_wait(rgb_ezo_t *dev)
{
	char *response_code = malloc(2 * sizeof(char));

	if (_read_i2c_ezo(dev, response_code))
	{
		return RGB_EZO_READ_ERR;
	}
	free(response_code);

	return RGB_EZO_OK;
}

static int _read_i2c_ezo(rgb_ezo_t *dev, char *result_data)
{
	uint8_t response_code;

	do
	{
		xtimer_usleep(100 * US_PER_MS);
		if (i2c_read_bytes(I2C, ADDR, result_data, 20, 0x00) < 0)
		{
			DEBUG("\n[rgb_ezo debug] Read from rgb ezo failed \n");
			return RGB_EZO_READ_ERR;
		}
		response_code = result_data[0];
	} while (response_code != RGB_EZO_SUCCESS);

	return RGB_EZO_OK;
}

static int _write_i2c_ezo(rgb_ezo_t *dev, uint8_t *command, size_t *size)
{
	if (i2c_write_bytes(I2C, ADDR, command, *size, 0x0) < 0)
	{
		DEBUG("\n[rgb_ezo debug] Writing to rgb ezo failed \n");
		return RGB_EZO_WRITE_ERR;
	}

	return RGB_EZO_OK;
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

	return RGB_EZO_OK;
}

