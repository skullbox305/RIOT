/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_rtd_oem
 * @{
 *
 * @file
 * @brief       RTD OEM device driver
 *
 * @author      Ting XU <your-email@placeholder.com>
 * @}
 */

#include "xtimer.h"
#include "assert.h"
#include "periph/i2c.h"
#include "periph/gpio.h"

#include "rtd_oem.h"

#include "rtd_oem_params.h"
#include "rtd_oem_regs.h"

#define ENABLE_DEBUG    (0)
#include "debug.h"

#define I2C (dev->params.i2c)
#define ADDR (dev->params.addr)
#define IRQ_OPTION (dev->params.irq_option)

int rtd_oem_init(rtd_oem_t *dev, const rtd_oem_params_t *params)
{
    assert(dev && params);

    dev->params = *params;

    uint8_t reg_data;

    i2c_acquire(I2C);

    /* Register read test */
    if (i2c_read_regs(I2C, ADDR, RTD_OEM_REG_DEVICE_TYPE,
                      &reg_data, 1, 0x0) < 0) {
        DEBUG("\n[rtd_oem debug] init - error: unable to read reg %x\n",
              RTD_OEM_REG_DEVICE_TYPE);

        i2c_release(I2C);
        return RTD_OEM_NODEV;
    }

    /* Test if the device ID of the attached RTD OEM sensor equals the
     * value of the RTD_OEM_REG_DEVICE_TYPE register
     * */
    if (reg_data != RTD_OEM_DEVICE_TYPE_ID) {
        DEBUG("\n[rtd_oem debug] init - error: the attached device is not a RTD OEM "
              "Sensor. Read Device Type ID is: %i\n",
              reg_data);
        i2c_release(I2C);
        return RTD_OEM_NOT_RTD;
    }
    i2c_release(I2C);

    return RTD_OEM_OK;
}
