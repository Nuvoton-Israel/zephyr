/*
 * Copyright (c) 2026 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_INTERRUPT_CONTROLLER_NPCM4_MIWU_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_INTERRUPT_CONTROLLER_NPCM4_MIWU_H_

/**
 * @name NPCM400 Multi-Input Wake-Up Unit (MIWU) tables and groups
 * @{
 */

/** Number of MIWU instances. */
#define NPCM4_MIWU_TABLE_COUNT 3
/** Number of wake-up groups within one MIWU instance. */
#define NPCM4_MIWU_GROUP_COUNT 8
/** Number of wake-up inputs within one group. */
#define NPCM4_MIWU_INPUT_COUNT 8

#define NPCM4_MIWU_TABLE_0 0
#define NPCM4_MIWU_TABLE_1 1
#define NPCM4_MIWU_TABLE_2 2

#define NPCM4_MIWU_GROUP_1 0
#define NPCM4_MIWU_GROUP_2 1
#define NPCM4_MIWU_GROUP_3 2
#define NPCM4_MIWU_GROUP_4 3
#define NPCM4_MIWU_GROUP_5 4
#define NPCM4_MIWU_GROUP_6 5
#define NPCM4_MIWU_GROUP_7 6
#define NPCM4_MIWU_GROUP_8 7

/** @} */

/**
 * @name NPCM400 MIWU register offsets
 *
 * The wake-up registers of a MIWU instance are not laid out as one
 * contiguous block per group:
 *   - WKEDG and WKAEDG are interleaved in pairs,
 *   - WKPND and WKPCL of the same group are four bytes apart,
 *   - WKEN and WKINEN are interleaved in pairs,
 *   - WKMOD is a separate contiguous block starting at 0x70,
 *   - groups 6 - 8 (index 5 - 7) are relocated to a second address window.
 *
 * Every offset is therefore expressed as a formula over the group index
 * rather than assumed from a fixed stride.
 * @{
 */
#define NPCM4_WKEDG_OFFSET(n)  (0x000 + ((n) * 2) + (((n) < 5) ? 0 : 0x01e))
#define NPCM4_WKAEDG_OFFSET(n) (0x001 + ((n) * 2) + (((n) < 5) ? 0 : 0x01e))
#define NPCM4_WKPND_OFFSET(n)  (0x00a + ((n) * 4) + (((n) < 5) ? 0 : 0x010))
#define NPCM4_WKPCL_OFFSET(n)  (0x00c + ((n) * 4) + (((n) < 5) ? 0 : 0x010))
#define NPCM4_WKEN_OFFSET(n)   (0x01e + ((n) * 2) + (((n) < 5) ? 0 : 0x012))
#define NPCM4_WKINEN_OFFSET(n) (0x01f + ((n) * 2) + (((n) < 5) ? 0 : 0x012))
#define NPCM4_WKMOD_OFFSET(n)  (0x070 + (n))

/** @} */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_INTERRUPT_CONTROLLER_NPCM4_MIWU_H_ */
