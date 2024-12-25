/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MISC_C2H_NPCM_H_
#define ZEPHYR_INCLUDE_DRIVERS_MISC_C2H_NPCM_H_

#include <zephyr/device.h>

/*
 * Core Access to Host (C2H) device registers
 */
struct c2h_reg {
	/* 0x000: Indirect Host I/O Address */
	volatile uint16_t IHIOA;
	/* 0x002: Indirect Host Data */
	volatile uint8_t IHD;
	volatile uint8_t reserved1;
	/* 0x004: Lock Host Access */
	volatile uint16_t LKSIOHA;
	/* 0x006: Access Lock Violation */
	volatile uint16_t SIOLV;
	/* 0x008: Core-to-Host Modules Access Enable */
	volatile uint16_t CRSMAE;
	/* 0x00A: Module Control */
	volatile uint8_t SIBCTRL;
	volatile uint8_t reserved2;
	/* 0x00C: Lock Host Access 2 */
	volatile uint16_t LKSIOHA2;
	/* 0x00E: Access Lock Violation 2 */
	volatile uint16_t SIOLV2;
	volatile uint8_t reserved3[14];
	/* 0x01E: Core to Host Access Version */
	volatile uint8_t C2H_VER;
};

/* C2H register fields */
#define NPCM_LKSIOHA_LKCFG   0
#define NPCM_LKSIOHA_LKSPHA  2
#define NPCM_LKSIOHA_LKHIKBD 11
#define NPCM_CRSMAE_CFGAE    0
#define NPCM_CRSMAE_HIKBDAE  11
#define NPCM_SIOLV_SPLV      2
#define NPCM_SIBCTRL_CSAE    0
#define NPCM_SIBCTRL_CSRD    1
#define NPCM_SIBCTRL_CSWR    2

typedef enum {
	/* 1: Configuration */
	SIB_DEV_CFG = 1,
	/* 2: Print port */
	SIB_DEV_PRT,
	/* 3: UARTA */
	SIB_DEV_UARTA,
	/* 4: UARTB */
	SIB_DEV_UARTB,
	/* 5: Mailbox */
	SIB_DEV_MAILBOX,
	/* 6: CIR */
	SIB_DEV_CIR,
	/* 7: RTC */
	SIB_DEV_RTC,
	/* 8: Extend RAM */
	SIB_DEV_HRAM,
	/* 9: Mobile System Wake-Up Control */
	SIB_DEV_MSWC,
	/* 10: Shared Memory Core Access 2 */
	SIB_DEV_SHM2,
	/* 11: Shared Memory Core Access */
	SIB_DEV_SHM,
	/* 12: KBC */
	SIB_DEV_KBC,
	/* 13: Power Management Channel 1 */
	SIB_DEV_PMCHAN1,
	/* 14: Power Management Channel 2 */
	SIB_DEV_PMCHAN2,
	/* 15: Power Management Channel 3 */
	SIB_DEV_PMCHAN3,
	/* 16: Power Management Channel 4 */
	SIB_DEV_PMCHAN4,
	/* 17: UARTC */
	SIB_DEV_UARTC,
	/* 18: UARTD */
	SIB_DEV_UARTD,
	/* 19: UARTE */
	SIB_DEV_UARTE,
	/* 20: UARTF */
	SIB_DEV_UARTF,
} SIB_DEVICE_T;

typedef enum {
	RTC_SEC = 0x00,
	RTC_SEC_ALARM,
	RTC_MIN,
	RTC_MIN_ALARM,
	RTC_HOUR,
	RTC_HOUR_ALARM,
	RTC_WEEKDAY,
	RTC_DAY,
	RTC_MONTH,
	RTC_YEAR,
	/* Timer Configuration: 0xA */
	RTC_CFG,
	/* Control */
	RTC_CTL,
	/* Alarm Interrupt Flag: 0xC */
	RTC_ALMFLG,
	/* Control and Status: 0xD */
	RTC_CTS,
	RTC_WEEKDAY_ALARM,
	RTC_DAY_ALARM,
	RTC_MONTH_ALARM,
	RTC_YEAR_ALARM,
} SIB_RTC_OFFSET_Enum;

typedef enum {
	CFG_INDEX = 0x00, /* port 2E or 4E */
	CFG_DATA,         /* port 2F or 4F */
} SIB_DEV_CFG_Enum;

typedef enum {
	RTC_INDEX = 0x00,
	RTC_DATA,
} SIB_DEV_RTC_Enum;

void c2h_write_io_cfg_reg(const struct device *dev, uint8_t reg_index, uint8_t reg_data);

uint8_t c2h_read_io_cfg_reg(const struct device *dev, uint8_t reg_index);

void rtc_write_offset(const struct device *dev, SIB_RTC_OFFSET_Enum offset, uint8_t value);

uint8_t rtc_read_offset(const struct device *dev, SIB_RTC_OFFSET_Enum offset);

#endif
