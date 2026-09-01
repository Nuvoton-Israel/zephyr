/*
 * Copyright (c) 2021 Nuvoton Technology Corporation.
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

#ifndef _NUVOTON_NPCM_SOC_POWER_H_
#define _NUVOTON_NPCM_SOC_POWER_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Disable UART RX wake-up interrupt.
 */
void npcx_uart_disable_access_interrupt(void);

/**
 * @brief Enable UART RX wake-up interrupt.
 */
void npcx_uart_enable_access_interrupt(void);

#ifdef __cplusplus
}
#endif

#endif /* _NUVOTON_NPCM_SOC_POWER_H_ */
