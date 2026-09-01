/*
 * Copyright (c) 2026 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_NUVOTON_NPCM4_GPIO_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_NUVOTON_NPCM4_GPIO_H_

/**
 * @name NPCM400 GPIO port register offsets
 *
 * Every GPIO port occupies eight consecutive byte-wide registers.
 * @{
 */
#define NPCM4_GPIO_PDOUT_OFFSET     0x000
#define NPCM4_GPIO_PDIN_OFFSET      0x001
#define NPCM4_GPIO_PDIR_OFFSET      0x002
#define NPCM4_GPIO_PPULL_OFFSET     0x003
#define NPCM4_GPIO_PPUD_OFFSET      0x004
#define NPCM4_GPIO_PENVDD_OFFSET    0x005
#define NPCM4_GPIO_PTYPE_OFFSET     0x006
#define NPCM4_GPIO_PLOCK_CTL_OFFSET 0x007

/** Number of pins per GPIO port. */
#define NPCM4_GPIO_PORT_PIN_NUM 8

/** @} */

/**
 * @name GPIO pin voltage flags
 *
 * The voltage flags are a Zephyr specific extension of the standard GPIO
 * flags specified by the Linux GPIO binding for use with the Nuvoton NPCM
 * SoCs.
 *
 * @{
 */

/** @cond INTERNAL_HIDDEN */
#define NPCM4_GPIO_VOLTAGE_POS  11
#define NPCM4_GPIO_VOLTAGE_MASK (1U << NPCM4_GPIO_VOLTAGE_POS)
/** @endcond */

/** Set pin at the default voltage level (3.3V) */
#define NPCM4_GPIO_VOLTAGE_DEFAULT (0U << NPCM4_GPIO_VOLTAGE_POS)
/** Set pin voltage level at 1.8 V */
#define NPCM4_GPIO_VOLTAGE_1P8     (1U << NPCM4_GPIO_VOLTAGE_POS)

/** @} */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_GPIO_NUVOTON_NPCM4_GPIO_H_ */
