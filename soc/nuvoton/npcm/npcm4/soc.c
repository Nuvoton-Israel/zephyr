/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <zephyr/linker/sections.h>

#if CONFIG_SOC_LATE_INIT_HOOK
__pinned_bss bool npcm_sys_application;

/* Hook at the end of the kernel initialization */
void soc_late_init_hook(void)
{
	/* Set the flag to indicate the system is in application stage */
	npcm_sys_application = true;
}
#endif
