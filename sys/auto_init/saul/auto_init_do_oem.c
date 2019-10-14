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
 * @brief       Auto initialization of Atlas Scientific DO OEM sensor
 *
 * @author      Ting XU <timtsui@outlook.com.com>
 * @author		Igor Knippenberg <igor.knippenberg@gmail.com>
 *
 * @}
 */

#ifdef MODULE_DO_OEM

#include "assert.h"
#include "log.h"

#include "saul_reg.h"
#include "do_oem.h"
#include "do_oem_params.h"

/**
 * @brief   Define the number of configured sensors
 */
#define DO_OEM_NUM   (sizeof(do_oem_params) / sizeof(do_oem_params[0]))

/**
 * @brief   Allocate memory for the device descriptors
 */
static do_oem_t do_oem_devs[DO_OEM_NUM];

/**
 * @brief DO OEM provides two sensor measurements (DO_MG,DO_P) and three saul
 *        devices for salinity, pressure and temperature compensation
 */
#define SENSORS_NUMOF 5

/**
 * @brief   Memory for the SAUL registry entries
 */
static saul_reg_t saul_entries[DO_OEM_NUM * SENSORS_NUMOF];

/**
 * @brief   Define the number of saul info
 */
#define DO_OEM_INFO_NUM (sizeof(do_oem_saul_info) / sizeof(do_oem_saul_info[0]))

/**
 * @brief   Reference the driver struct
 */
extern saul_driver_t do_oem_do_mg_saul_driver;
extern saul_driver_t do_oem_do_percent_saul_driver;
extern saul_driver_t do_oem_sal_comp_saul_driver;
extern saul_driver_t do_oem_pres_comp_saul_driver;
extern saul_driver_t do_oem_temp_comp_saul_driver;

void auto_init_do_oem(void)
{
    assert(DO_OEM_INFO_NUM == DO_OEM_NUM);
    unsigned ix = 0;
    for (unsigned i = 0; i < DO_OEM_NUM; i++) {
        LOG_DEBUG("[auto_init_saul] initializing do_oem #%d\n", i);
        if (do_oem_init(&do_oem_devs[i], &do_oem_params[i]) < 0) {
            LOG_ERROR("[auto_init_saul] error initializing do_oem #%d\n", i);
            continue;
        }

        /* D.O. value in mg/L */
        saul_entries[ix].dev = &(do_oem_devs[i]);
        saul_entries[ix].name = do_oem_saul_info[i].name;
        saul_entries[ix].driver = &do_oem_do_mg_saul_driver;
        saul_reg_add(&(saul_entries[ix]));
        ix++;

        /* D.O. value in % saturation */
        saul_entries[ix].dev = &(do_oem_devs[i]);
        saul_entries[ix].name = do_oem_saul_info[i].name;
        saul_entries[ix].driver = &do_oem_do_percent_saul_driver;
        saul_reg_add(&(saul_entries[ix]));
        ix++;

        /* Salinity compensation */
        saul_entries[ix].dev = &(do_oem_devs[i]);
        saul_entries[ix].name = do_oem_saul_info[i].name;
        saul_entries[ix].driver = &do_oem_sal_comp_saul_driver;
        saul_reg_add(&(saul_entries[ix]));
        ix++;

        /* Pressure compensation */
        saul_entries[ix].dev = &(do_oem_devs[i]);
        saul_entries[ix].name = do_oem_saul_info[i].name;
        saul_entries[ix].driver = &do_oem_pres_comp_saul_driver;
        saul_reg_add(&(saul_entries[ix]));
        ix++;

        /* Temperature compensation */
        saul_entries[ix].dev = &(do_oem_devs[i]);
        saul_entries[ix].name = do_oem_saul_info[i].name;
        saul_entries[ix].driver = &do_oem_temp_comp_saul_driver;
        saul_reg_add(&(saul_entries[ix]));
    }
}

#else
typedef int dont_be_pedantic;
#endif /* MODULE_DO_OEM */
