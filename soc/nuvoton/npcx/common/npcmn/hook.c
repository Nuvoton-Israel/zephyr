/*
 * Copyright (c) 2025 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * This file contains the hook functions that are called by the ROM code
 * during the boot process. The ROM code will call these functions at
 * specific points in the boot process to allow for customization of the
 * boot process.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

/* Section attributes for ROM hook placement */
#define HOOK_DATA_ATTRIBUTE __attribute__((section(".rom_hooks_data")))
#define HOOK_FUNC_ATTRIBUTE __attribute__((section(".rom_hooks_fn")))

/* Memory access macros */
#define REG8(addr)  (*((volatile uint8_t  *)(addr)))
#define REG32(addr) (*((volatile uint32_t *)(addr)))

/* Register addresses */
#define FIU_FWINFO_STATUS_REG  0x100c59d4U
#define DEVALT5C_REG           0x400c305cU
#define GPIO5_DIN_REG          0x4008b011U
#define GPIO5_DIR_REG          0x4008b012U
#define GPIO5_PULL_REG         0x4008b013U
#define GPIO5_PUD_REG          0x4008b014U

/* Register bit definitions */
#define FIU_AUTH_PASS_BIT      BIT(1)
#define DEVALT5C_GPIO_SEL_BIT  BIT(1)
#define GPIO50_BIT             BIT(0)

/* Hook functions forward declarations */
extern void Rom_hook2(void);
extern void Rom_hook3(void);
extern void Rom_hook4(void);

/* Hook function address table - called by ROM bootloader */
HOOK_DATA_ATTRIBUTE const void *hook_func_table[] = {
	(void *)0x00000000, /* Hook 1 must be flash code */
	(void *)Rom_hook2,
	(void *)Rom_hook3,
	(void *)Rom_hook4,
};

/*
 * Rom_hook2 - Recovery mode detection via GPIO50
 *
 * This hook checks GPIO50 state during boot. If GPIO50 is pulled low
 * when authentication passes, it clears the authentication bit to
 * force the device into recovery mode.
 *
 * GPIO50 configuration:
 * - Input mode
 * - Internal pull-up resistor enabled
 * - External pull-down triggers recovery mode entry
 */
HOOK_FUNC_ATTRIBUTE __attribute__((weak)) void Rom_hook2(void)
{
	uint8_t gpio_val;
	uint32_t fw_status;

	/* Enable GPIO50 alternate function (DEVALT5C[1] = 1) */
	REG8(DEVALT5C_REG) |= DEVALT5C_GPIO_SEL_BIT;

	/* Configure GPIO50 as input with pull-up */
	REG8(GPIO5_DIR_REG) &= ~GPIO50_BIT;  /* Input mode */
	REG8(GPIO5_PUD_REG) &= ~GPIO50_BIT;  /* Pull-up mode */
	REG8(GPIO5_PULL_REG) |= GPIO50_BIT;  /* Enable pull resistor */

	/* Read firmware authentication status */
	fw_status = REG32(FIU_FWINFO_STATUS_REG);

	/* Check if firmware authentication passed */
	if (fw_status & FIU_AUTH_PASS_BIT) {
		/* Read GPIO50 state */
		gpio_val = REG8(GPIO5_DIN_REG);

		/* If GPIO50 is low (pulled down), clear auth to enter recovery */
		if (!(gpio_val & GPIO50_BIT)) {
			REG32(FIU_FWINFO_STATUS_REG) = fw_status & ~FIU_AUTH_PASS_BIT;
		}
	}

	/* Disable pull resistor to save power */
	REG8(GPIO5_PULL_REG) &= ~GPIO50_BIT;
}

HOOK_FUNC_ATTRIBUTE __attribute((weak)) void Rom_hook3(void)
{
	/* hook 3 */
}

HOOK_FUNC_ATTRIBUTE __attribute((weak)) void Rom_hook4(void)
{
	/* hook 4 */
}
