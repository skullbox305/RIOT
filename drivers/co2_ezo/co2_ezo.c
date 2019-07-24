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

#define ENABLE_DEBUG    (1)
#include "debug.h"

#define I2C (dev->params.i2c)
#define ADDR (dev->params.addr)

static int _read_i2c_ezo(co2_ezo_t *dev, char *result_data)
{
    (void)result_data;
//    char data[20];
    uint8_t response_code;

    xtimer_usleep(1000);

    if (i2c_read_bytes(I2C, ADDR, result_data, 20, 0x00)) {
        return CO2_EZO_READ_ERR;
    }

    response_code = result_data[0];

    if (response_code == CO2_EZO_CMD_PENDING) {
        xtimer_usleep(900 * US_PER_MS);
        if (i2c_read_bytes(I2C, ADDR, result_data, 20, 0x00)) {
            return CO2_EZO_READ_ERR;
        }
    }

    DEBUG("\nresponse code: %d, data: %s\n", response_code, result_data + 1);

    return CO2_EZO_OK;
}

static int _write_i2c_ezo(co2_ezo_t *dev, uint8_t *command, size_t *size)
{
    if (i2c_write_bytes(I2C, ADDR, command, *size, 0x0) < 0) {
        return CO2_EZO_WRITE_ERR;
    }
    return CO2_EZO_OK;
}

int co2_ezo_init(co2_ezo_t *dev, const co2_ezo_params_t *params)
{
    assert(dev && params);
    dev->params = *params;

    char *result_data = malloc(20 * sizeof(char));

    i2c_acquire(I2C);

    uint8_t cmd[] = CO2_EZO_TAKE_READING;
    size_t size = sizeof(cmd) / sizeof(uint8_t);

    if (_write_i2c_ezo(dev, (uint8_t *)cmd, &size) < 0) {
        DEBUG("\n[co2_ezo debug] writing to co2 ezo failed \n");
        i2c_release(I2C);
        free(result_data);
        return CO2_EZO_WRITE_ERR;
    }

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
