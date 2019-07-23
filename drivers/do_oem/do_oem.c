/*
 * Copyright (C) 2019 University of Applied Sciences Emden / Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_do_oem
 * @{
 *
 * @file
 * @brief       EC OEM device driver
 *
 * @author      Ting Xu <timtsui@outlook.com>
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 * @}
 */

#include "xtimer.h"
#include "assert.h"

#include "periph/i2c.h"
#include "periph/gpio.h"

#include "do_oem.h"
#include "include/do_oem_params.h"
#include "include/do_oem_regs.h"

#define ENABLE_DEBUG    (1)
#include "debug.h"

#define I2C (dev->params.i2c)
#define ADDR (dev->params.addr)
#define IRQ_OPTION (dev->params.irq_option)

/**
 * @brief   Unlocks the DO_OEM_REG_UNLOCK register to be able to change the
 *          I2C device address, by writing 0x55 and 0xAA to the register
 *
 * @param[in] dev device descriptor
 *
 * @return DO_OEM_OK on success
 * @return DO_OEM_WRITE_ERR if writing to the device failed
 */
static int _unlock_address_reg(do_oem_t *dev);

/**
 * @brief   Setting the OEM OEM interrupt mode to the defined mode provided
 *          in the device descriptor
 *
 * @param[in] dev device descriptor
 *
 * @return DO_OEM_OK on success
 * @return DO_OEM_WRITE_ERR if writing to the device failed
 */
static int _set_interrupt_pin(const do_oem_t *dev);

/**
 * @brief   Polls the DO_OEM_REG_NEW_READING register as long as it does not
 *          equal 0x01, which indicates that a new EC reading is available.
 *          Polling is done in an interval of 20ms. Estimated completion ~640ms
 *
 * @param[in] dev device descriptor
 *
 * @return DO_OEM_OK on success
 * @return DO_OEM_READ_ERR if reading from the register failed
 * @return DO_OEM_WRITE_ERR if reseting the register failed
 */
static int _new_reading_available(const do_oem_t *dev);

///**
// * @brief   Sets the DO_OEM_REG_CALIBRATION_BASE register to the EC
// *          @p calibration_value which the device will be calibrated to.
// *
// * @param[in] dev device descriptor
// * @param[in] calibration_value DO value the device will be calibrated to
// *
// * @return DO_OEM_OK on success
// * @return DO_OEM_READ_ERR if reading from the register failed
// * @return DO_OEM_WRITE_ERR if writing the calibration_value to the device failed
// */
//static int _set_calibration_value(const do_oem_t *dev,
//		uint16_t calibration_value);

int do_oem_init(do_oem_t *dev, const do_oem_params_t *params)
{
	assert(dev && params);

	dev->params = *params;

	uint8_t reg_data;

	i2c_acquire(I2C);

	/* Register read test */
	if (i2c_read_regs(I2C, ADDR, DO_OEM_REG_DEVICE_TYPE, &reg_data, 3, 0x0) < 0)
	{
		DEBUG("\n[do_oem debug] init - error: unable to read reg %x\n",
				DO_OEM_REG_DEVICE_TYPE);

		i2c_release(I2C);
		return DO_OEM_NODEV;
	}

	/* Test if the device ID of the attached DO OEM sensor equals the
	 * value of the DO_OEM_REG_DEVICE_TYPE register
	 * */
	if (reg_data != DO_OEM_DEVICE_TYPE_ID)
	{
		DEBUG(
				"\n[do_oem debug] init - error: the attached device is not a DO OEM "
						"Sensor. Read Device Type ID is: %i\n", reg_data);
		i2c_release(I2C);
		return DO_OEM_NOT_DO;
	}
	i2c_release(I2C);

	return DO_OEM_OK;
}

static int _unlock_address_reg(do_oem_t *dev)
{
	uint8_t reg_value = 1;

	i2c_acquire(I2C);

	i2c_write_reg(I2C, ADDR, DO_OEM_REG_UNLOCK, 0x55, 0x0);
	i2c_write_reg(I2C, ADDR, DO_OEM_REG_UNLOCK, 0xAA, 0x0);
	/* if successfully unlocked the register will equal 0x00 */
	i2c_read_reg(I2C, ADDR, DO_OEM_REG_UNLOCK, &reg_value, 0x0);

	if (reg_value != 0x00)
	{
		DEBUG("\n[do_oem debug] Failed at unlocking I2C address register. \n");
		i2c_release(I2C);
		return DO_OEM_WRITE_ERR;
	}
	i2c_release(I2C);

	return DO_OEM_OK;
}

int do_oem_set_i2c_address(do_oem_t *dev, uint8_t addr)
{
	assert(dev);

	if (_unlock_address_reg(dev) != DO_OEM_OK)
	{
		return DO_OEM_WRITE_ERR;
	}

	i2c_acquire(I2C);

	if (i2c_write_reg(I2C, ADDR, DO_OEM_REG_ADDRESS, addr, 0x0) < 0)
	{
		DEBUG("\n[do_oem debug] Setting I2C address to %x failed\n", addr);
		i2c_release(I2C);
		return DO_OEM_WRITE_ERR;
	}

	dev->params.addr = addr;
	i2c_release(I2C);

	return DO_OEM_OK;
}

static int _set_interrupt_pin(const do_oem_t *dev)
{
	assert(dev);
	i2c_acquire(I2C);

	if (i2c_write_reg(I2C, ADDR, DO_OEM_REG_INTERRUPT, IRQ_OPTION, 0x0) < 0)
	{
		DEBUG("\n[do_oem debug] Setting interrupt pin to option %d failed.\n",
				IRQ_OPTION);
		i2c_release(I2C);
		return DO_OEM_WRITE_ERR;
	}

	i2c_release(I2C);

	return DO_OEM_OK;
}

int do_oem_enable_interrupt(do_oem_t *dev, do_oem_interrupt_pin_cb_t cb,
		void *arg)
{
	if (dev->params.interrupt_pin == GPIO_UNDEF)
	{
		return DO_OEM_INTERRUPT_GPIO_UNDEF;
	}

	if (_set_interrupt_pin(dev) < 0)
	{
		return DO_OEM_WRITE_ERR;
	}

	int gpio_flank = 0;

	switch (IRQ_OPTION)
	{
	case DO_OEM_IRQ_FALLING:
		gpio_flank = GPIO_FALLING;
		break;
	case DO_OEM_IRQ_RISING:
		gpio_flank = GPIO_RISING;
		break;
	case DO_OEM_IRQ_BOTH:
		gpio_flank = GPIO_BOTH;
		break;
	}

	dev->arg = arg;
	dev->cb = cb;
	if (gpio_init_int(dev->params.interrupt_pin, dev->params.gpio_mode,
			gpio_flank, cb, arg) < 0)
	{
		DEBUG("\n[do_oem debug] Initializing interrupt gpio pin failed.\n");
		return DO_OEM_GPIO_INIT_ERR;
	}

	return DO_OEM_OK;
}

int do_oem_reset_interrupt_pin(const do_oem_t *dev)
{
	/* no reset needed for mode DO_OEM_IRQ_BOTH */
	if (dev->params.irq_option == DO_OEM_IRQ_BOTH)
	{
		return DO_OEM_OK;
	}

	if (_set_interrupt_pin(dev) < 0)
	{
		return DO_OEM_WRITE_ERR;
	}
	return DO_OEM_OK;
}

int do_oem_set_led_state(const do_oem_t *dev, do_oem_led_state_t state)
{
	assert(dev);
	i2c_acquire(I2C);

	if (i2c_write_reg(I2C, ADDR, DO_OEM_REG_LED, state, 0x0) < 0)
	{
		DEBUG("\n[do_oem debug] Setting LED state to %d failed.\n", state);
		i2c_release(I2C);
		return DO_OEM_WRITE_ERR;
	}
	i2c_release(I2C);

	return DO_OEM_OK;
}

int do_oem_set_device_state(const do_oem_t *dev, do_oem_device_state_t state)
{
	assert(dev);
	i2c_acquire(I2C);

	if (i2c_write_reg(I2C, ADDR, DO_OEM_REG_HIBERNATE, state, 0x0) < 0)
	{
		DEBUG("\n[do_oem debug] Setting device state to %d failed\n", state);
		i2c_release(I2C);
		return DO_OEM_WRITE_ERR;
	}
	i2c_release(I2C);

	return DO_OEM_OK;
}

static int _new_reading_available(const do_oem_t *dev)
{
	int8_t new_reading_available;

	assert(dev);
	i2c_acquire(I2C);
	do
	{
		if (i2c_read_reg(I2C, ADDR, DO_OEM_REG_NEW_READING,
				&new_reading_available, 0x0) < 0)
		{
			DEBUG(
					"\n[do_oem debug] Failed at reading DO_OEM_REG_NEW_READING\n");
			i2c_release(I2C);
			return DO_OEM_READ_ERR;
		}
		xtimer_usleep(20 * US_PER_MS);
	} while (new_reading_available == 0);

	/* need to manually reset register back to 0x00 */
	if (i2c_write_reg(I2C, ADDR, DO_OEM_REG_NEW_READING, 0x00, 0x0) < 0)
	{
		DEBUG("\n[do_oem debug] Resetting DO_OEM_REG_NEW_READING failed\n");
		i2c_release(I2C);
		return DO_OEM_WRITE_ERR;
	}
	i2c_release(I2C);

	return DO_OEM_OK;
}

int do_oem_start_new_reading(const do_oem_t *dev)
{
	if (do_oem_set_device_state(dev, DO_OEM_TAKE_READINGS) < 0)
	{
		return DO_OEM_WRITE_ERR;
	}

	/* if interrupt pin is undefined, poll till new reading was taken and stop
	 * device form taking further readings */
	if (dev->params.interrupt_pin == GPIO_UNDEF)
	{
		int result = _new_reading_available(dev);
		if (result < 0)
		{
			return result;
		}

		if (do_oem_set_device_state(dev, DO_OEM_STOP_READINGS) < 0)
		{
			return DO_OEM_WRITE_ERR;
		}
	}
	return DO_OEM_OK;
}

int do_oem_clear_calibration(const do_oem_t *dev)
{
	uint8_t reg_value;

	assert(dev);
	i2c_acquire(I2C);
	if (i2c_write_reg(I2C, ADDR, DO_OEM_REG_CALIBRATION_REQUEST, 0x01, 0) < 0)
	{
		DEBUG("\n[do_oem debug] Clearing calibration failed \n");
		i2c_release(I2C);
		return DO_OEM_WRITE_ERR;
	}

	do
	{
		if (i2c_read_reg(I2C, ADDR, DO_OEM_REG_CALIBRATION_REQUEST, &reg_value,
				0) < 0)
		{
			i2c_release(I2C);
			return DO_OEM_READ_ERR;
		}
	} while (reg_value != 0x00);

	i2c_release(I2C);

	return DO_OEM_OK;
}

//static int _set_calibration_value(const do_oem_t *dev,
//		uint32_t calibration_value)
//{
//	uint8_t reg_value[4];
//
//	reg_value[0] = 0x00;
//	reg_value[1] = (uint8_t) (calibration_value >> 16);
//	reg_value[2] = (uint8_t) (calibration_value >> 8);
//	reg_value[3] = (uint8_t) (calibration_value & 0x000000FF);
//
//	i2c_acquire(I2C);
//
//	if (i2c_write_regs(I2C, ADDR, DO_OEM_REG_CALIBRATION_BASE, &reg_value, 4, 0)
//			< 0)
//	{
//		DEBUG("\n[do_oem debug] Writing calibration value failed \n");
//		i2c_release(I2C);
//		return DO_OEM_WRITE_ERR;
//	}
//
//	/* Calibration is critical, so check if written value is in fact correct */
//	if (i2c_read_regs(I2C, ADDR, DO_OEM_REG_CALIBRATION_BASE, &reg_value, 4, 0)
//			< 0)
//	{
//		DEBUG("\n[do_oem debug] Reading the calibration value failed \n");
//		i2c_release(I2C);
//		return DO_OEM_READ_ERR;
//	}
//
//	uint32_t confirm_value = (int32_t) (reg_value[1] << 16)
//			| (int32_t) (reg_value[2] << 8) | (int32_t) (reg_value[3]);
//
//	if (confirm_value != calibration_value)
//	{
//		DEBUG("\n[do_oem debug] Setting calibration register to EC raw %ld "
//				"failed \n", calibration_value);
//		i2c_release(I2C);
//		return DO_OEM_WRITE_ERR;
//	}
//
//	i2c_release(I2C);
//
//	return DO_OEM_OK;
//}

int do_oem_set_calibration(const do_oem_t *dev,
		do_oem_calibration_option_t option)
{
	assert(dev);

//	uint8_t reg_value;

	i2c_acquire(I2C);

	if (i2c_write_reg(I2C, ADDR, DO_OEM_REG_CALIBRATION_REQUEST, option, 0) < 0)
	{
		DEBUG("\n[do_oem debug] Sending calibration request failed\n");
		return DO_OEM_WRITE_ERR;
	}

//	do
//	{
//		if (i2c_read_reg(I2C, ADDR, DO_OEM_REG_CALIBRATION_REQUEST, &reg_value,
//				0) < 0)
//		{
//			DEBUG(
//					"\n[do_oem debug] Reading calibration request status failed\n");
//			i2c_release(I2C);
//			return DO_OEM_READ_ERR;
//		}
//	} while (reg_value != 0 x00);

	i2c_release(I2C);

	return DO_OEM_OK;
}

int do_oem_read_calibration_state(const do_oem_t *dev,
		uint16_t *calibration_state)
{
	assert(dev);
	i2c_acquire(I2C);

	if (i2c_read_reg(I2C, ADDR, DO_OEM_REG_CALIBRATION_CONFIRM,
			calibration_state, 0) < 0)
	{
		DEBUG(
				"\n[do_oem debug] Failed at reading calibration confirm register\n");
		i2c_release(I2C);
		return DO_OEM_READ_ERR;
	}
	i2c_release(I2C);
	return DO_OEM_OK;
}

int do_oem_set_sal_compensation(const do_oem_t *dev,
		uint32_t salinity_compensation)
{

	assert(dev);
	uint8_t reg_value[4];

	reg_value[0] = 0x00;
	reg_value[1] = (uint8_t) (salinity_compensation >> 16);
	reg_value[2] = (uint8_t) (salinity_compensation >> 8);
	reg_value[3] = (uint8_t) (salinity_compensation & 0x000000FF);

	i2c_acquire(I2C);

	if (i2c_write_regs(I2C, ADDR, DO_OEM_REG_SALI_COMPENSATION_BASE, &reg_value,
			4, 0) < 0)
	{
		DEBUG("\n[do_oem debug] Setting salinity compensation of device to "
				"%ld failed\n", salinity_compensation);
		i2c_release(I2C);
		return DO_OEM_WRITE_ERR;
	}
	i2c_release(I2C);

	return DO_OEM_OK;
}

int do_oem_set_pres_compensation(const do_oem_t *dev,
		uint32_t pressure_compensation)
{


	assert(dev);
	uint8_t reg_value[4];

	reg_value[0] = 0x00;
	reg_value[1] = (uint8_t) (pressure_compensation >> 16);
	reg_value[2] = (uint8_t) (pressure_compensation >> 8);
	reg_value[3] = (uint8_t) (pressure_compensation & 0x000000FF);

	i2c_acquire(I2C);

	if (i2c_write_regs(I2C, ADDR, DO_OEM_REG_PRES_COMPENSATION_BASE, &reg_value,
			4, 0) < 0)
	{
		DEBUG("\n[do_oem debug] Setting pressure compensation of device to "
				"%ld failed\n", pressure_compensation);
		i2c_release(I2C);
		return DO_OEM_WRITE_ERR;
	}
	i2c_release(I2C);

	return DO_OEM_OK;
}

int do_oem_set_temp_compensation(const do_oem_t *dev,
		uint32_t temperature_compensation)
{

	assert(dev);
	uint8_t reg_value[4];

	reg_value[0] = 0x00;
	reg_value[1] = (uint8_t) (temperature_compensation >> 16);
	reg_value[2] = (uint8_t) (temperature_compensation >> 8);
	reg_value[3] = (uint8_t) (temperature_compensation & 0x000000FF);

	i2c_acquire(I2C);

	if (i2c_write_regs(I2C, ADDR, DO_OEM_REG_TEMP_COMPENSATION_BASE, &reg_value,
			4, 0) < 0)
	{
		DEBUG("\n[do_oem debug] Setting temperature compensation of device to "
				"%ld failed\n", temperature_compensation);
		i2c_release(I2C);
		return DO_OEM_WRITE_ERR;
	}
	i2c_release(I2C);

	return DO_OEM_OK;
}

int do_oem_read_sali_compensation(const do_oem_t *dev,
		uint32_t *salinity_compensation)
{
	uint8_t reg_value[4];

	assert(dev);
	i2c_acquire(I2C);

	if (i2c_read_regs(I2C, ADDR, DO_OEM_REG_SALI_COMFIRMATION_BASE, &reg_value,
			4, 0) < 0)
	{
		DEBUG("[do_oem debug] Getting salinity compensation value failed\n");
		i2c_release(I2C);
		return DO_OEM_READ_ERR;
	}
	*salinity_compensation = (int32_t) (reg_value[1] << 16)
			| (int32_t) (reg_value[2] << 8) | (int32_t) (reg_value[3]);

	i2c_release(I2C);

	return DO_OEM_OK;
}

int do_oem_read_pres_compensation(const do_oem_t *dev,
		uint32_t *pressure_compensation)
{
	uint8_t reg_value[4];

	assert(dev);
	i2c_acquire(I2C);

	if (i2c_read_regs(I2C, ADDR, DO_OEM_REG_PRES_COMFIRMATION_BASE, &reg_value,
			4, 0) < 0)
	{
		DEBUG("[do_oem debug] Getting pressure compensation value failed\n");
		i2c_release(I2C);
		return DO_OEM_READ_ERR;
	}
	*pressure_compensation = (int32_t) (reg_value[1] << 16)
			| (int32_t) (reg_value[2] << 8) | (int32_t) (reg_value[3]);

	i2c_release(I2C);

	return DO_OEM_OK;
}

int do_oem_read_temp_compensation(const do_oem_t *dev,
		uint32_t *temperature_compensation)
{
	uint8_t reg_value[4];

	assert(dev);
	i2c_acquire(I2C);

	if (i2c_read_regs(I2C, ADDR, DO_OEM_REG_TEMP_COMFIRMATION_BASE, &reg_value,
			4, 0) < 0)
	{
		DEBUG("[do_oem debug] Getting pressure compensation value failed\n");
		i2c_release(I2C);
		return DO_OEM_READ_ERR;
	}
	*temperature_compensation = (int32_t) (reg_value[1] << 16)
			| (int32_t) (reg_value[2] << 8) | (int32_t) (reg_value[3]);

	i2c_release(I2C);

	return DO_OEM_OK;
}

int do_oem_read_do_mg(const do_oem_t *dev, uint16_t *do_mg_value)
{
	uint8_t reg_value[4];

	assert(dev);
	i2c_acquire(I2C);

	if (i2c_read_regs(I2C, ADDR, DO_OEM_REG_DO_MGL_READING_BASE, &reg_value, 4,
			0) < 0)
	{
		DEBUG("[do_oem debug] Getting DO(mg/L) value failed\n");
		i2c_release(I2C);
		return DO_OEM_READ_ERR;
	}
	*do_mg_value = (int16_t) (reg_value[2] << 8) | (int16_t) (reg_value[3]);

	i2c_release(I2C);

	return DO_OEM_OK;
}

int do_oem_read_do_percent(const do_oem_t *dev, uint16_t *do_percent_value)
{
	uint8_t reg_value[4];

	assert(dev);
	i2c_acquire(I2C);

	if (i2c_read_regs(I2C, ADDR, DO_OEM_REG_DO_PERCENT_READING_BASE, &reg_value,
			4, 0) < 0)
	{
		DEBUG("[do_oem debug] Getting TDS value failed\n");
		i2c_release(I2C);
		return DO_OEM_READ_ERR;
	}
	*do_percent_value = (int16_t) (reg_value[2] << 8)
			| (int16_t) (reg_value[3]);

	i2c_release(I2C);

	return DO_OEM_OK;
}
