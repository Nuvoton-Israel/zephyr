/*
 * Copyright (c) 2020 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * This file mirrors the interface that the shared Nuvoton npcx peripheral
 * drivers expect from their SoC. It deliberately keeps the npcx_ prefixed
 * names, because those names are part of the interface the drivers call
 * into; only the hardware description behind them is npcm specific and it
 * comes from the npcm4 devicetree binding headers.
 */

#ifndef _NUVOTON_NPCM_SOC_GPIO_H_
#define _NUVOTON_NPCM_SOC_GPIO_H_

#include <zephyr/device.h>
#include <zephyr/dt-bindings/gpio/nuvoton-npcm4-gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Pin number for each GPIO device */
#define NPCX_GPIO_PORT_PIN_NUM NPCM4_GPIO_PORT_PIN_NUM

/**
 * @brief Get GPIO device instance by port index
 *
 * @param port GPIO device index
 *
 * @retval Pointer to structure device
 * @retval NULL Invalid parameter of GPIO port index
 */
const struct device *npcx_get_gpio_dev(int port);

/**
 * @brief Enable the connection between io pads and GPIO instance
 *
 * @param dev Pointer to device structure for the gpio driver instance.
 * @param pin Pin number.
 */
void npcx_gpio_enable_io_pads(const struct device *dev, int pin);

/**
 * @brief Disable the connection between io pads and GPIO instance
 *
 * @param dev Pointer to device structure for the gpio driver instance.
 * @param pin Pin number.
 */
void npcx_gpio_disable_io_pads(const struct device *dev, int pin);

#ifdef __cplusplus
}
#endif

#endif /* _NUVOTON_NPCM_SOC_GPIO_H_ */
