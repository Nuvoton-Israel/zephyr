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

#include <cmsis_core_m_defaults.h>
#include <zephyr/devicetree.h>

/*
 * The NPCM400 hardware description lives entirely in the devicetree binding
 * headers under include/zephyr/dt-bindings. reg_def.h pulls them in, declares
 * the matching register structures and binds them to the naming expected by
 * the reused npcx peripheral drivers.
 */
#include "reg_def.h"
#include "clock_def.h"

#include <soc_dt.h>
#include <soc_pins.h>

/* Clock prescaler register values derived from the pcc devicetree node */
#define VAL_HFCGP   ((FPRED_VAL << 4) | AHB6DIV_VAL)
#define VAL_HFCBCD  (APB1DIV_VAL | (APB2DIV_VAL << 4))
#define VAL_HFCBCD1 ((MCLKD_SL << 2) | FIUDIV_VAL)
#define VAL_HFCBCD2 APB3DIV_VAL

#endif /* _NUVOTON_NPCM_SOC_H_ */
