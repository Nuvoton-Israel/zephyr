/*
 * Copyright (c) 2026 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PWM_NPCM4_PWM_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PWM_NPCM4_PWM_H_

/**
 * @name NPCM400 PWM register offsets
 * @{
 */
#define NPCM4_PWM_PRSC_OFFSET     0x000
#define NPCM4_PWM_CTR_OFFSET      0x002
#define NPCM4_PWM_PWMCTL_OFFSET   0x004
#define NPCM4_PWM_DCR_OFFSET      0x006
#define NPCM4_PWM_PWMCTLEX_OFFSET 0x00c
/** @} */

/**
 * @name NPCM400 PWM register fields
 * @{
 */
#define NPCM4_PWMCTL_INVP            0
#define NPCM4_PWMCTL_CKSEL           1
#define NPCM4_PWMCTL_HB_DC_CTL_FIELD FIELD(2, 2)
#define NPCM4_PWMCTL_PWR             7
#define NPCM4_PWMCTLEX_FCK_SEL_FIELD FIELD(4, 2)
#define NPCM4_PWMCTLEX_OD_OUT        7
/** @} */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PWM_NPCM4_PWM_H_ */
