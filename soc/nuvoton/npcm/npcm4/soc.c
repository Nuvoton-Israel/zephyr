/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <soc.h>

extern void scfg_init(void);

void soc_early_init_hook(void)
{
	/*
	 * Switch every pad whose default function is not an IO back to GPIO
	 * before any peripheral driver applies its own pin control state.
	 */
	scfg_init();
}
