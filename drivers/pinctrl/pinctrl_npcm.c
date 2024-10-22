/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcm_pinctrl

#include "zephyr/devicetree.h"
#include <assert.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/pinctrl/npcm-pinctrl.h>
#include <zephyr/drivers/gpio.h>
#include <soc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pinctrl_npcm, LOG_LEVEL_ERR);

/* Multi-function pinmux data */
#define NPCM_PINMUX_REG_SIZE 5
#define NPCM_PINMUX_ALT_SIZE 6

struct npcm_pinmux_config {
	uint16_t reg[NPCM_PINMUX_REG_SIZE];
	uint8_t alt[NPCM_PINMUX_ALT_SIZE]; /* Bit n in alt represents reg[n] bit value */
};

#define NPCM_PINMUX_GROUP_RDATA(id, reg_group, alt_group)                                          \
	[id] = {.reg = reg_group, .alt = alt_group}
#define NPCM_PINMUX_ID(node_id, prop) DT_PROP_BY_IDX(node_id, id, 0)
#define NPCM_PINMUX_REG(node_id, prop)                                                             \
	{DT_PROP_BY_IDX(node_id, prop, 4), DT_PROP_BY_IDX(node_id, prop, 3),                       \
	 DT_PROP_BY_IDX(node_id, prop, 2), DT_PROP_BY_IDX(node_id, prop, 1),                       \
	 DT_PROP_BY_IDX(node_id, prop, 0)}
#define NPCM_PINMUX_ALT(node_id, prop)                                                             \
	{DT_PROP_BY_IDX(node_id, prop, 0), DT_PROP_BY_IDX(node_id, prop, 1),                       \
	 DT_PROP_BY_IDX(node_id, prop, 2), DT_PROP_BY_IDX(node_id, prop, 3),                       \
	 DT_PROP_BY_IDX(node_id, prop, 4), DT_PROP_BY_IDX(node_id, prop, 5)}

#define NPCM_PINMUX_INIT(node_id, prop)                                                            \
	COND_CODE_1( \
		DT_PROP(node_id, pinmux_group_def), \
		(NPCM_PINMUX_GROUP_RDATA( \
				NPCM_PINMUX_ID(node_id, prop), \
				NPCM_PINMUX_REG(node_id, pin_reg),  \
				NPCM_PINMUX_ALT(node_id, alt)),), \
		())

/*
 * Multi-function pinmux definitions.
 *
 * The GPIO must be the first alt function, which is the default selection for the GPIO driver.
 * The rest of the alt functions should be defined in the order of the alt functions.
 */
static const struct npcm_pinmux_config npcm_pinmux_table[] = {
	DT_FOREACH_CHILD_VARGS(DT_DRV_INST(0), NPCM_PINMUX_INIT)};

/* Driver config */
struct npcm_pinctrl_config {
	/* Device base address used for pinctrl driver */
	uintptr_t base_scfg;
	uintptr_t base_glue;
};

static const struct npcm_pinctrl_config npcm_pinctrl_cfg = {
	.base_scfg = DT_INST_REG_ADDR_BY_NAME(0, scfg),
	.base_glue = DT_INST_REG_ADDR_BY_NAME(0, glue),
};

/* SCFG multi-registers */
#define NPCM_SCFG_OFFSET(n)     (((n) >> NPCM_PINCTRL_SHIFT) & NPCM_PINCTRL_MASK)
#define NPCM_SCFG(base, n)      (*(volatile uint8_t *)(base + NPCM_SCFG_OFFSET(n)))
#define NPCM_SCFG_BIT_OFFSET(n) ((n) & NPCM_PINCTRL_BIT_MASK)

static void npcm_periph_pinmux_configure(const struct npcm_periph_pinmux *alt)
{
	const uintptr_t scfg_base = npcm_pinctrl_cfg.base_scfg;
	uint8_t cfg_value;
	uint16_t reg_value;
	uint32_t bit_mask;
	int i;

	/* Check index is exist */
	if ((alt->id >= ARRAY_SIZE(npcm_pinmux_table)) ||
	    (alt->alt_group >= NPCM_PINMUX_ALT_SIZE)) {
		return;
	}

	/* Check for empty config */
	cfg_value = npcm_pinmux_table[alt->id].alt[alt->alt_group];
	if (cfg_value == NPCM_PIN_CFG_NULL) {
		return;
	}

	/* Set bit value in alt */
	for (i = 0; i < NPCM_PINMUX_REG_SIZE; i++) {
		reg_value = npcm_pinmux_table[alt->id].reg[i];
		if (reg_value == NPCM_PIN_REG_NULL) {
			continue;
		}

		bit_mask = BIT(NPCM_SCFG_BIT_OFFSET(reg_value));
		if (cfg_value & BIT(i)) {
			NPCM_SCFG(scfg_base, reg_value) |= bit_mask;
		} else {
			NPCM_SCFG(scfg_base, reg_value) &= ~bit_mask;
		}
	}
}

static void npcm_periph_pupd_configure(const struct npcm_periph_pupd *pupd,
				       enum npcm_io_bias_type type)
{
	const uintptr_t scfg_base = npcm_pinctrl_cfg.base_scfg;
	uint8_t pupd_mask = BIT(NPCM_SCFG_BIT_OFFSET(pupd->id));

	if (type == NPCM_BIAS_TYPE_NONE) {
		NPCM_SCFG(scfg_base, pupd->id) &= ~pupd_mask;
	} else {
		NPCM_SCFG(scfg_base, pupd->id) |= pupd_mask;
	}
}

static void npcm_device_control_configure(const struct npcm_dev_ctl *ctrl)
{
	const uintptr_t scfg_base = npcm_pinctrl_cfg.base_scfg;
	uint8_t devctl_mask = BIT(NPCM_SCFG_BIT_OFFSET(ctrl->id));

	if (ctrl->is_set == 0) {
		NPCM_SCFG(scfg_base, ctrl->id) &= ~devctl_mask;
	} else {
		NPCM_SCFG(scfg_base, ctrl->id) |= devctl_mask;
	}
}

/* Pinctrl API implementation */
int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
	ARG_UNUSED(reg);
	uint8_t i;

	/* Configure all peripheral devices' properties here. */
	for (i = 0; i < pin_cnt; i++) {
		switch (pins[i].flags.type) {
		case NPCM_PINCTRL_TYPE_PERIPH_PINMUX:
			/* Configure peripheral device's pinmux functionality */
			npcm_periph_pinmux_configure(&pins[i].cfg.pinmux);
			break;

		case NPCM_PINCTRL_TYPE_PERIPH_PUPD:
			/* Configure peripheral device's internal PU/PD */
			npcm_periph_pupd_configure(&pins[i].cfg.pupd, pins[i].flags.io_bias_type);
			break;

		case NPCM_PINCTRL_TYPE_DEVICE_CTRL:
			/* Configure device's io characteristics */
			npcm_device_control_configure(&pins[i].cfg.dev_ctl);
			break;

		default:
			return -ENOTSUP;
		}
	}

	return 0;
}
