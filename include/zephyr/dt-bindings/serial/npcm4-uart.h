/*
 * Copyright (c) 2026 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_SERIAL_NPCM4_UART_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_SERIAL_NPCM4_UART_H_

/**
 * @name NPCM400 UART register offsets
 * @{
 */
#define NPCM4_UART_UTBUF_OFFSET  0x000
#define NPCM4_UART_URBUF_OFFSET  0x002
#define NPCM4_UART_UICTRL_OFFSET 0x004
#define NPCM4_UART_USTAT_OFFSET  0x006
#define NPCM4_UART_UFRS_OFFSET   0x008
#define NPCM4_UART_UMDSL_OFFSET  0x00a
#define NPCM4_UART_UBAUD_OFFSET  0x00c
#define NPCM4_UART_UPSR_OFFSET   0x00e
#define NPCM4_UART_UFCTRL_OFFSET 0x016
#define NPCM4_UART_UTXFLV_OFFSET 0x018
#define NPCM4_UART_URXFLV_OFFSET 0x01a
/** @} */

/**
 * @name NPCM400 UART register fields
 * @{
 */
#define NPCM4_UICTRL_TBE 0
#define NPCM4_UICTRL_RBF 1
#define NPCM4_UICTRL_ETI 5
#define NPCM4_UICTRL_ERI 6
#define NPCM4_UICTRL_EEI 7

#define NPCM4_USTAT_PE   0
#define NPCM4_USTAT_FE   1
#define NPCM4_USTAT_DOE  2
#define NPCM4_USTAT_ERR  3
#define NPCM4_USTAT_BKD  4
#define NPCM4_USTAT_RB9  5
#define NPCM4_USTAT_XMIP 6

#define NPCM4_UFRS_CHAR_FIELD FIELD(0, 2)
#define NPCM4_UFRS_STP        2
#define NPCM4_UFRS_XB9        3
#define NPCM4_UFRS_PSEL_FIELD FIELD(4, 2)
#define NPCM4_UFRS_PEN        6

#define NPCM4_UFRS_CHAR_DATA_BIT_8 0
#define NPCM4_UFRS_CHAR_DATA_BIT_7 1

#define NPCM4_UMDSL_FIFO_MD 0
#define NPCM4_UMDSL_ETD     4
#define NPCM4_UMDSL_ERD     5

#define NPCM4_UFCTRL_FIFOEN 0
#define NPCM4_UTXFLV_TFL    FIELD(0, 5)
#define NPCM4_URXFLV_RFL    FIELD(0, 5)

/** Depth of the transmit and receive FIFOs, in bytes. */
#define NPCM4_SZ_UART_FIFO 16
/** @} */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_SERIAL_NPCM4_UART_H_ */
