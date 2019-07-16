/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 *
 */

/**
 * @ingroup     sys_auto_init_saul
 * @{
 *
 * @file
 * @brief       Auto initialization of Atlas Scientific RTD OEM sensor
 *
 * @author      Ting XU <timtsui@outlook.com.com>
 * @author		Igor Knippenberg <igor.knippenberg@gmail.com>
 *
 * @}
 */

#ifdef MODULE_RTD_OEM

#include "assert.h"
#include "log.h"

#include "saul_reg.h"
#include "rtd_oem.h"
#include "rtd_oem_params.h"

/**
 * @brief   Define the number of configured sensors
 */
#define RTD_OEM_NUM   (sizeof(rtd_oem_params) / sizeof(rtd_oem_params[0]))

/**
 * @brief   Allocate memory for the device descriptors
 */
static rtd_oem_t rtd_oem_devs[RTD_OEM_NUM];

/**
 * @brief   Memory for the SAUL registry entries
 */
static saul_reg_t saul_entries[RTD_OEM_NUM];

/**
 * @brief   Define the number of saul info
 */
#define RTD_OEM_INFO_NUM (sizeof(rtd_oem_saul_info) / sizeof(rtd_oem_saul_info[0]))

/**
 * @brief   Reference the driver struct
 */
extern saul_driver_t rtd_oem_saul_driver;

void auto_init_rtd_oem(void)
{
    assert(RTD_OEM_INFO_NUM == RTD_OEM_NUM);

    for (unsigned i = 0; i < RTD_OEM_NUM; i++) {
        LOG_DEBUG("[auto_init_saul] initializing rtd_oem #%d\n", i);
        if (rtd_oem_init(&rtd_oem_devs[i], &rtd_oem_params[i]) < 0) {
            LOG_ERROR("[auto_init_saul] error initializing rtd_oem #%d\n", i);
            continue;
        }

        saul_entries[i].dev = &(rtd_oem_devs[i]);
        saul_entries[i].name = rtd_oem_saul_info[i].name;
        saul_entries[i].driver = &rtd_oem_saul_driver;
        saul_reg_add(&(saul_entries[i]));
    }
}

#else
typedef int dont_be_pedantic;
#endif /* MODULE_RTD_OEM */
