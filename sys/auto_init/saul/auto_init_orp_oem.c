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
 * @brief       Auto initialization of Atlas Scientific pH OEM sensor
 *
 * @author      Ting XU <timtsui@outlook.com.com>
 * @author		Igor Knippenberg <igor.knippenberg@gmail.com>
 *
 * @}
 */

#ifdef MODULE_ORP_OEM

#include "assert.h"
#include "log.h"

#include "saul_reg.h"
#include "orp_oem.h"
#include "orp_oem_params.h"

/**
 * @brief   Define the number of configured sensors
 */
#define ORP_OEM_NUM   (sizeof(orp_oem_params) / sizeof(orp_oem_params[0]))

/**
 * @brief   Allocate memory for the device descriptors
 */
static orp_oem_t orp_oem_devs[ORP_OEM_NUM];

/**
 * @brief   Memory for the SAUL registry entries
 */
static saul_reg_t saul_entries[ORP_OEM_NUM];

/**
 * @brief   Define the number of saul info
 */
#define ORP_OEM_INFO_NUM (sizeof(orp_oem_saul_info) / sizeof(orp_oem_saul_info[0]))

/**
 * @brief   Reference the driver struct
 */
extern saul_driver_t orp_oem_saul_driver;

void auto_init_orp_oem(void)
{
    assert(ORP_OEM_INFO_NUM == ORP_OEM_NUM);

    for (unsigned i = 0; i < ORP_OEM_NUM; i++) {
        LOG_DEBUG("[auto_init_saul] initializing orp_oem #%d\n", i);
        if (orp_oem_init(&orp_oem_devs[i], &orp_oem_params[i]) < 0) {
            LOG_ERROR("[auto_init_saul] error initializing orp_oem #%d\n", i);
            continue;
        }

        saul_entries[i].dev = &(orp_oem_devs[i]);
        saul_entries[i].name = orp_oem_saul_info[i].name;
        saul_entries[i].driver = &orp_oem_saul_driver;
        saul_reg_add(&(saul_entries[i]));
    }
}

#else
typedef int dont_be_pedantic;
#endif /* MODULE_ORP_OEM */
