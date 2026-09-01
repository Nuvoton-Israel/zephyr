/*
 * Copyright (c) 2026 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _NUVOTON_NPCM_REG_DEF_H
#define _NUVOTON_NPCM_REG_DEF_H

#include <zephyr/dt-bindings/clock/npcm4_clock.h>
#include <zephyr/dt-bindings/gpio/nuvoton-npcm4-gpio.h>
#include <zephyr/dt-bindings/interrupt-controller/npcm4_miwu.h>
#include <zephyr/dt-bindings/pinctrl/npcm4-pinctrl.h>
#include <zephyr/dt-bindings/pwm/npcm4-pwm.h>
#include <zephyr/dt-bindings/serial/npcm4-uart.h>

#include <reg_access.h>

/*
 * This header only declares the shape of the NPCM400 register blocks. Every
 * register offset, bit position and field encoding lives in the devicetree
 * binding headers included above, so that the devicetree and the C code are
 * guaranteed to agree on a single description of the hardware.
 *
 * The layout of each structure below is verified against those binding
 * headers by the build-time assertions in registers.c.
 */

/*
 * Core Domain Clock Generator (CDCG) device registers
 */
struct cdcg_reg {
	/* High Frequency Clock Generator (HFCG) registers */
	/* 0x000: HFCG Control */
	volatile uint8_t HFCGCTRL;
	volatile uint8_t reserved1;
	/* 0x002: HFCG M Low Byte Value */
	volatile uint8_t HFCGML;
	volatile uint8_t reserved2;
	/* 0x004: HFCG M High Byte Value */
	volatile uint8_t HFCGMH;
	volatile uint8_t reserved3;
	/* 0x006: HFCG N Value */
	volatile uint8_t HFCGN;
	volatile uint8_t reserved4;
	/* 0x008: HFCG Prescaler */
	volatile uint8_t HFCGP;
	volatile uint8_t reserved5[7];
	/* 0x010: HFCG Bus Clock Dividers */
	volatile uint8_t HFCBCD;
	volatile uint8_t reserved6;
	/* 0x012: HFCG Bus Clock Dividers 1 */
	volatile uint8_t HFCBCD1;
	volatile uint8_t reserved7;
	/* 0x014: HFCG Bus Clock Dividers 2 */
	volatile uint8_t HFCBCD2;
	volatile uint8_t reserved8[8];
	/* 0x01d: HFCG Bus Clock Dividers 3 */
	volatile uint8_t HFCBCD3;
	volatile uint8_t reserved9[226];

	/* Low Frequency Clock Generator (LFCG) registers */
	/* 0x100: LFCG Control */
	volatile uint8_t LFCGCTL;
	volatile uint8_t reserved10;
	/* 0x102: High-Frequency Reference Divisor I */
	volatile uint16_t HFRDI;
	/* 0x104: High-Frequency Reference Divisor F */
	volatile uint16_t HFRDF;
	/* 0x106: FRCLK Clock Divisor */
	volatile uint16_t FRCDIV;
	/* 0x108: Divisor Correction Value 1 */
	volatile uint16_t DIVCOR1;
	/* 0x10a: Divisor Correction Value 2 */
	volatile uint16_t DIVCOR2;
	volatile uint8_t reserved11[8];
	/* 0x114: LFCG Control 2 */
	volatile uint8_t LFCGCTL2;
	volatile uint8_t reserved12;
};

/*
 * Power Management Controller (PMC) device registers
 *
 * The nine power-down control registers are not contiguous, so they are
 * reached through NPCM4_PWDWN_CTL_OFFSET() instead of a structure member.
 */
struct pmc_reg {
	/* 0x000: Power Management Controller */
	volatile uint8_t PMCSR;
	volatile uint8_t reserved1[2];
	/* 0x003: Enable in Sleep Control */
	volatile uint8_t ENIDL_CTL;
	/* 0x004: Disable in Idle Control */
	volatile uint8_t DISIDL_CTL;
	/* 0x005: Disable in Idle Control 1 */
	volatile uint8_t DISIDL_CTL1;
	volatile uint8_t reserved2[26];
	/* 0x020 - 0x021: RAM Power-Down Control 1 - 2 */
	volatile uint8_t RAM_PD[2];
};

/* Macro function for the PMC power-down control multi-registers */
#define NPCM4_PWDWN_CTL(base, n) (*(volatile uint8_t *)((base) + NPCM4_PWDWN_CTL_OFFSET(n)))

/* Macro functions for the Development and Debugger Interface (DDI) registers */
#define NPCM4_DBGCTRL(base)   (*(volatile uint8_t *)((base) + 0x004))
#define NPCM4_DBGFRZEN1(base) (*(volatile uint8_t *)((base) + 0x006))
#define NPCM4_DBGFRZEN2(base) (*(volatile uint8_t *)((base) + 0x007))
#define NPCM4_DBGFRZEN3(base) (*(volatile uint8_t *)((base) + 0x008))
#define NPCM4_DBGFRZEN4(base) (*(volatile uint8_t *)((base) + 0x009))

/* DDI register fields */
#define NPCM4_DBGCTRL_CCDEV_SEL      FIELD(6, 2)
#define NPCM4_DBGFRZEN3_GLBL_FRZ_DIS 7

/*
 * System Configuration (SCFG) device registers
 */
struct scfg_reg {
	/* 0x000: Device Control */
	volatile uint8_t DEVCNT;
	/* 0x001: Straps Status */
	volatile uint8_t STRPST;
	/* 0x002: Reset Control and Status */
	volatile uint8_t RSTCTL;
	volatile uint8_t reserved1;
	/* 0x004: Device Control 3 */
	volatile uint8_t DEV_CTL3;
	volatile uint8_t reserved2;
	/* 0x006: Device Control 4 */
	volatile uint8_t DEV_CTL4;
	volatile uint8_t reserved3[4];
	/* 0x00b: Device Alternate Function 10 */
	volatile uint8_t DEVALT10;
	/* 0x00c: Device Alternate Function 11 */
	volatile uint8_t DEVALT11;
	/* 0x00d: Device Alternate Function 12 */
	volatile uint8_t DEVALT12;
	volatile uint8_t reserved4[2];
	/* 0x010 - 0x01f: Device Alternate Function 0 - F */
	volatile uint8_t DEVALT0[16];
	volatile uint8_t reserved5[4];
	/* 0x024: Device Alternate Function CX */
	volatile uint8_t DEVALTCX;
	volatile uint8_t reserved6[3];
	/* 0x028: Device Pull-Up Enable 0 */
	volatile uint8_t DEVPU0;
	/* 0x029: Device Pull-Down Enable 1 */
	volatile uint8_t DEVPD1;
	/* 0x02a: Low-Voltage Pins Control 0 */
	volatile uint8_t LV_CTL0;
	/* 0x02b: Low-Voltage Pins Control 1 */
	volatile uint8_t LV_CTL1;
};

/* Macro functions for the SCFG multi-registers */
#define NPCM4_DEV_CTL(base, n)     (*(volatile uint8_t *)((base) + (n)))
#define NPCM4_DEVALT(base, n)      (*(volatile uint8_t *)((base) + NPCM4_DEVALT_OFFSET(n)))
#define NPCM4_DEVALT_LK(base, n)   (*(volatile uint8_t *)((base) + NPCM4_DEVALT_LK_OFFSET(n)))
#define NPCM4_PUPD_EN(base, n)     (*(volatile uint8_t *)((base) + NPCM4_PUPD_EN_OFFSET(n)))
#define NPCM4_LV_GPIO_CTL(base, n) (*(volatile uint8_t *)((base) + NPCM4_LV_GPIO_CTL_OFFSET(n)))

/*
 * Supported host interface type for the HIF_TYP_SEL field of DEVCNT.
 *
 * The enumeration keeps the npcx name because it is part of the prototype of
 * npcx_host_interface_sel() declared by the reused soc_pins.h.
 */
enum npcx_hif_type {
	NPCX_HIF_TYPE_NONE,
	NPCX_HIF_TYPE_LPC,
	NPCX_HIF_TYPE_ESPI_SHI,
};

/* Supported VOSCCLK frequency for the SIO_CLK_SEL field of DEV_CTL3 */
enum npcm_voscclk_type {
	NPCM_VOSCCLK_96MHZ,
	NPCM_VOSCCLK_100MHZ,
	NPCM_VOSCCLK_120MHZ,
	NPCM_VOSCCLK_90MHZ,
};

/*
 * System Glue (GLUE) device registers
 */
struct glue_reg {
	volatile uint8_t reserved1[2];
	/* 0x002: SMBus Start Bit Detection */
	volatile uint8_t SMB_SBD;
	/* 0x003: SMBus Event Enable */
	volatile uint8_t SMB_EEN;
	volatile uint8_t reserved2[12];
	/* 0x010: Simple Debug Port Data 0 */
	volatile uint8_t SDPD0;
	volatile uint8_t reserved3;
	/* 0x012: Simple Debug Port Data 1 */
	volatile uint8_t SDPD1;
	volatile uint8_t reserved4;
	/* 0x014: Simple Debug Port Control and Status */
	volatile uint8_t SDP_CTS;
	volatile uint8_t reserved5[12];
	/* 0x021: SMBus Bus Select */
	volatile uint8_t SMB_SEL;
	volatile uint8_t reserved6[5];
	/* 0x027: PSL Control and Status */
	volatile uint8_t PSL_CTS;
};

/* GLUE register fields */
/* PSL input detection mode is configured by bits 7:4 of PSL_CTS */
#define NPCM4_PSL_CTS_MODE_BIT(bit)  BIT((bit) + 4)
/* PSL input assertion events are reported by bits 3:0 of PSL_CTS */
#define NPCM4_PSL_CTS_EVENT_BIT(bit) BIT(bit)

/*
 * Universal Asynchronous Receiver-Transmitter (UART) device registers
 */
struct uart_reg {
	/* 0x000: Transmit Data Buffer */
	volatile uint8_t UTBUF;
	volatile uint8_t reserved1;
	/* 0x002: Receive Data Buffer */
	volatile uint8_t URBUF;
	volatile uint8_t reserved2;
	/* 0x004: Interrupt Control */
	volatile uint8_t UICTRL;
	volatile uint8_t reserved3;
	/* 0x006: Status */
	volatile uint8_t USTAT;
	volatile uint8_t reserved4;
	/* 0x008: Frame Select */
	volatile uint8_t UFRS;
	volatile uint8_t reserved5;
	/* 0x00a: Mode Select */
	volatile uint8_t UMDSL;
	volatile uint8_t reserved6;
	/* 0x00c: Baud Rate Divisor */
	volatile uint8_t UBAUD;
	volatile uint8_t reserved7;
	/* 0x00e: Baud Rate Prescaler */
	volatile uint8_t UPSR;
	volatile uint8_t reserved8[7];
	/* 0x016: FIFO Control */
	volatile uint8_t UFCTRL;
	volatile uint8_t reserved9;
	/* 0x018: TX FIFO Current Level */
	volatile uint8_t UTXFLV;
	volatile uint8_t reserved10;
	/* 0x01a: RX FIFO Current Level */
	volatile uint8_t URXFLV;
	volatile uint8_t reserved11[12];
};

/* Macro functions for the MIWU multi-registers */
#define NPCM4_WKEDG(base, group)  (*(volatile uint8_t *)((base) + NPCM4_WKEDG_OFFSET(group)))
#define NPCM4_WKAEDG(base, group) (*(volatile uint8_t *)((base) + NPCM4_WKAEDG_OFFSET(group)))
#define NPCM4_WKPND(base, group)  (*(volatile uint8_t *)((base) + NPCM4_WKPND_OFFSET(group)))
#define NPCM4_WKPCL(base, group)  (*(volatile uint8_t *)((base) + NPCM4_WKPCL_OFFSET(group)))
#define NPCM4_WKEN(base, group)   (*(volatile uint8_t *)((base) + NPCM4_WKEN_OFFSET(group)))
#define NPCM4_WKINEN(base, group) (*(volatile uint8_t *)((base) + NPCM4_WKINEN_OFFSET(group)))
#define NPCM4_WKMOD(base, group)  (*(volatile uint8_t *)((base) + NPCM4_WKMOD_OFFSET(group)))

/*
 * General-Purpose I/O (GPIO) device registers
 */
struct gpio_reg {
	/* 0x000: Port GPIOx Data Out */
	volatile uint8_t PDOUT;
	/* 0x001: Port GPIOx Data In */
	volatile uint8_t PDIN;
	/* 0x002: Port GPIOx Direction */
	volatile uint8_t PDIR;
	/* 0x003: Port GPIOx Pull-Up or Pull-Down Enable */
	volatile uint8_t PPULL;
	/* 0x004: Port GPIOx Pull-Up/Down Selection */
	volatile uint8_t PPUD;
	/* 0x005: Port GPIOx Drive Enable by VDD Present */
	volatile uint8_t PENVDD;
	/* 0x006: Port GPIOx Output Type */
	volatile uint8_t PTYPE;
	/* 0x007: Port GPIOx Lock Control */
	volatile uint8_t PLOCK_CTL;
};

/*
 * Pulse Width Modulator (PWM) device registers
 */
struct pwm_reg {
	/* 0x000: Clock Prescaler */
	volatile uint16_t PRSC;
	/* 0x002: Cycle Time */
	volatile uint16_t CTR;
	/* 0x004: PWM Control */
	volatile uint8_t PWMCTL;
	volatile uint8_t reserved1;
	/* 0x006: Duty Cycle */
	volatile uint16_t DCR;
	volatile uint8_t reserved2[4];
	/* 0x00c: PWM Control Extended */
	volatile uint8_t PWMCTLEX;
	volatile uint8_t reserved3;
};

#include "npcx_driver_abi.h"

#endif /* _NUVOTON_NPCM_REG_DEF_H */
