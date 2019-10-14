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
 * @brief       Auto initialization of Atlas Scientific EC OEM sensor
 *
 * @author      Ting XU <timtsui@outlook.com.com>
 * @author		Igor Knippenberg <igor.knippenberg@gmail.com>
 *
 * @}
 */

#ifdef MODULE_EC_OEM

#include "assert.h"
#include "log.h"

#include "saul_reg.h"
#include "ec_oem.h"
#include "ec_oem_params.h"

/**
 * @brief   Define the number of configured sensors
 */
#define EC_OEM_NUM   (sizeof(ec_oem_params) / sizeof(ec_oem_params[0]))

/**
 * @brief   Allocate memory for the device descriptors
 */
static ec_oem_t ec_oem_devs[EC_OEM_NUM];

/**
 * @brief EC OEM provides three sensor measurements (EC, TDS, PSS) and one
 *        saul device for temperature compensation
 */
#define SENSORS_NUMOF 4

/**
 * @brief   Memory for the SAUL registry entries
 */
static saul_reg_t saul_entries[EC_OEM_NUM * SENSORS_NUMOF];

/**
 * @brief   Define the number of saul info
 */
#define EC_OEM_INFO_NUM (sizeof(ec_oem_saul_info) / sizeof(ec_oem_saul_info[0]))

/**
 * @brief   Reference the driver struct
 */
extern saul_driver_t ec_oem_ec_saul_driver;
extern saul_driver_t ec_oem_tds_saul_driver;
extern saul_driver_t ec_oem_pss_saul_driver;
extern saul_driver_t ec_oem_temp_comp_saul_driver;

void auto_init_ec_oem(void)
{
	assert(EC_OEM_INFO_NUM == EC_OEM_NUM);
	unsigned ix = 0;
	for (unsigned i = 0; i < EC_OEM_NUM; i++)
	{
		LOG_DEBUG("[auto_init_saul] initializing ec_oem #%d\n", i);
		if (ec_oem_init(&ec_oem_devs[i], &ec_oem_params[i]) < 0)
		{
			LOG_ERROR("[auto_init_saul] error initializing ec_oem #%d\n", i);
			continue;
		}

		/* EC value */
		saul_entries[ix].dev = &(ec_oem_devs[i]);
		saul_entries[ix].name = ec_oem_saul_info[i].name;
		saul_entries[ix].driver = &ec_oem_ec_saul_driver;
		saul_reg_add(&(saul_entries[ix]));
		ix++;

		/* TDS value */
		saul_entries[ix].dev = &(ec_oem_devs[i]);
		saul_entries[ix].name = ec_oem_saul_info[i].name;
		saul_entries[ix].driver = &ec_oem_tds_saul_driver;
		saul_reg_add(&(saul_entries[ix]));
		ix++;

		/* PSS value */
		saul_entries[ix].dev = &(ec_oem_devs[i]);
		saul_entries[ix].name = ec_oem_saul_info[i].name;
		saul_entries[ix].driver = &ec_oem_pss_saul_driver;
		saul_reg_add(&(saul_entries[ix]));
		ix++;

		/* temp compensation */
		saul_entries[ix].dev = &(ec_oem_devs[i]);
		saul_entries[ix].name = ec_oem_saul_info[i].name;
		saul_entries[ix].driver = &ec_oem_temp_comp_saul_driver;
		saul_reg_add(&(saul_entries[ix]));
	}
}

#else
typedef int dont_be_pedantic;
#endif /* MODULE_EC_OEM */
