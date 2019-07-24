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

#include "assert.h"
#include "periph/i2c.h"

#include "co2_ezo.h"
#include "co2_ezo_internal.h"
#include "co2_ezo_params.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

#define I2C (dev->params.i2c)
#define ADDR (dev->params.addr)

static int _read_i2c_ezo(co2_ezo_t *dev, char *result_data)
{
    uint8_t data;
    uint8_t response_code = 0;
    bool data_read = false;
    int i = 0;

    printf("read start\n");

    i2c_read_bytes(I2C, ADDR, &data, 1, I2C_NOSTOP);
    response_code = data;
    printf("response code: %d\n", response_code);

    while (!data_read) {

    	if(response_code == 1){
        i2c_read_bytes(I2C, ADDR, &data, 1, I2C_NOSTART);
              xtimer_usleep(10 * US_PER_MS);
    	}
        printf("data: %d\n", data);

        if (data == 254) {
            xtimer_usleep(20 * US_PER_MS);
        }
        else if (i == 0) {
            response_code = data;
            i++;
        }
        else if (data == 0) {
            data_read = true;
            i++;
            result_data[i] = '\0';
        }
        else {
            result_data[i] = (char)data;
            i++;
        }
    }
    i2c_read_bytes(I2C, ADDR, &data, 1, 0x0);

    printf("i: %d - response code: %d - result data: %s\n", i, response_code,
           result_data);


    return CO2_EZO_OK;
}

static int _write_i2c_ezo(co2_ezo_t *dev, char *command)
{
	(void) command;
//    printf("start write: %x\n", (uint8_t *)command);
    if (i2c_write_bytes(I2C, ADDR, "i", 1, 0x0) < 0) {
        i2c_release(I2C);
        return CO2_EZO_WRITE_ERR;
    }
    return CO2_EZO_OK;
}

int co2_ezo_init(co2_ezo_t *dev, const co2_ezo_params_t *params)
{
    assert(dev && params);
    dev->params = *params;

    char *result_data = malloc(13 * sizeof(char));

    i2c_acquire(I2C);

    if (_write_i2c_ezo(dev, CO2_EZO_DEV_INFO) < 0) {
        DEBUG("\n[co2_ezo debug] writing to co2 ezo failed \n");
        i2c_release(I2C);
        free(result_data);
        return CO2_EZO_WRITE_ERR;
    }

    xtimer_usleep(500 * US_PER_MS);
    if (_read_i2c_ezo(dev, result_data) < 0) {
        DEBUG("\n[co2_ezo debug] read from co2 ezo failed \n");
        i2c_release(I2C);
        free(result_data);
        return CO2_EZO_READ_ERR;
    }

    i2c_release(I2C);
    free(result_data);

    return CO2_EZO_OK;
}
