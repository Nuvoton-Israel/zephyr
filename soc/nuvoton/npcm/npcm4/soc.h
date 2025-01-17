/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _NUVOTON_NPCM_SOC_H_
#define _NUVOTON_NPCM_SOC_H_

/* CMSIS required definitions */
#define __FPU_PRESENT CONFIG_CPU_HAS_FPU
#define __MPU_PRESENT CONFIG_CPU_HAS_ARM_MPU

/* Add include for DTS generated information */
#include <zephyr/devicetree.h>
#include <cmsis_core_m_defaults.h>

/* NPCM4 definitions */
#include "gdma.h"

/**
 * @brief Check if the post kernel stage is done
 *
 * @return true if invoked before application initialization
 * @return false if invoked during/after application initialization
 */
static inline bool npcm_is_post_kernel(void)
{
#if CONFIG_SOC_LATE_INIT_HOOK
	extern bool npcm_sys_application;

	return !npcm_sys_application;
#else
	return false;
#endif
}

#endif /* _NUVOTON_NPCM_SOC_H_ */
