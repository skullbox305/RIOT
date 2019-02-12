/*
 * Copyright (C) 2019 University of Applied Sciences Emden/Leer
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     boards_nrf52840-mdk
 * @{
 *
 * @file
 * @brief       Peripheral configuration for the nRF52840-MDK USB Dongle
 *
 * @author      Igor Knippenberg <igor.knippenberg@gmail.com>
 *
 */

#ifndef PERIPH_CONF_H
#define PERIPH_CONF_H

#include "periph_cpu.h"
#include "cfg_clock_32_1.h"
#include "cfg_rtt_default.h"
#include "cfg_timer_default.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    UART configuration
 * @{
 */
static const uart_conf_t uart_config[] = {
    {
        .dev = NRF_UARTE0,
        .rx_pin = GPIO_PIN(0, 19),
        .tx_pin = GPIO_PIN(0, 20),
        .rts_pin = (uint8_t)GPIO_UNDEF,
        .cts_pin = (uint8_t)GPIO_UNDEF,
        .irqn = UARTE0_UART0_IRQn,
    },
};

#define UART_0_ISR          (isr_uart0)

#define UART_NUMOF          (sizeof(uart_config) / sizeof(uart_config[0]))
/** @} */

/**
 * @name    SPI configuration
 * @{
 */
static const spi_conf_t spi_config[] = {
    {
        .dev = NRF_SPI0,
        .sclk = 15,
        .mosi = 13,
        .miso = 14
    }
};
#define SPI_NUMOF           (sizeof(spi_config) / sizeof(spi_config[0]))
/** @} */

/**
 * @name    I2C configuration
 * @{
 */
static const i2c_conf_t i2c_config[] = {
    {
        .dev = NRF_TWIM1,
        .scl = 20,
        .sda = 21,
        .speed = I2C_SPEED_NORMAL
    }
};
#define I2C_NUMOF           (sizeof(i2c_config) / sizeof(i2c_config[0]))
/** @} */

/**
 * @name   PWM configuration
 * @{
 */
static const pwm_conf_t pwm_config[] = {
    { NRF_PWM0, { 4, 5 } }
};
#define PWM_NUMOF           (sizeof(pwm_config) / sizeof(pwm_config[0]))
/** @} */


#ifdef __cplusplus
}
#endif

#endif /* PERIPH_CONF_H */
