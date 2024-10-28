/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcm_miwu

/**
 * Nuvoton NPCM MIWU driver.
 *
 * The Multi-Input Wake-Up Unit (MIWU) in Nuvoton NPCM devices supports waking the system from
 * 'Sleep' or 'Deep Sleep' power states. It provides signal conditioning with 'Level' and 'Edge'
 * trigger types and groups external interrupt sources for the NVIC.
 *
 * The NPCM series includes three identical MIWU modules: MIWU0, MIWU1, and MIWU2, which together
 * support up to 143 internal and/or external wake-up input (WUI) sources.
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_npcm.h>
#include <zephyr/drivers/interrupt_controller/intc_npcm_miwu.h>
#include <zephyr/irq.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(intc_npcm_miwu, CONFIG_INTC_LOG_LEVEL);

/* MIWU module instances */
#define NPCM_MIWU_DEV(inst) DEVICE_DT_INST_GET(inst),

static const struct device *miwu_devs[] = {DT_INST_FOREACH_STATUS_OKAY(NPCM_MIWU_DEV)};

BUILD_ASSERT(ARRAY_SIZE(miwu_devs) == NPCM_MIWU_TABLE_MAX,
	     "Size of miwu_devs array must equal to NPCM_MIWU_TABLE_MAX");

/* MIWU multi-registers */
#define NPCM_WK(n, start, step, offset) ((start) + ((n) * (step)) + ((n) < 5 ? 0 : (offset)))

#define NPCM_WKEDG(base, n)  (*(volatile uint8_t *)(base + NPCM_WK(n, 0x000, 2, 0x1E)))
#define NPCM_WKAEDG(base, n) (*(volatile uint8_t *)(base + NPCM_WK(n, 0x001, 2, 0x1E)))
#define NPCM_WKPND(base, n)  (*(volatile uint8_t *)(base + NPCM_WK(n, 0x00A, 4, 0x10)))
#define NPCM_WKPCL(base, n)  (*(volatile uint8_t *)(base + NPCM_WK(n, 0x00C, 4, 0x10)))
#define NPCM_WKEN(base, n)   (*(volatile uint8_t *)(base + NPCM_WK(n, 0x01E, 2, 0x12)))
#define NPCM_WKINEN(base, n) (*(volatile uint8_t *)(base + NPCM_WK(n, 0x01F, 2, 0x12)))
#define NPCM_WKMOD(base, n)  (*(volatile uint8_t *)(base + NPCM_WK(n, 0x070, 1, 0)))

#define NPCM_WK_OVWR(reg_name, base, n)                                                            \
	(reg_name(base, NPCM_WUI_GROUP_OFFSET(n)) = BIT(NPCM_WUI_BIT_OFFSET(n)))
#define NPCM_WK_SET(reg_name, base, n)                                                             \
	(reg_name(base, NPCM_WUI_GROUP_OFFSET(n)) |= BIT(NPCM_WUI_BIT_OFFSET(n)))
#define NPCM_WK_CLR(reg_name, base, n)                                                             \
	(reg_name(base, NPCM_WUI_GROUP_OFFSET(n)) &= ~BIT(NPCM_WUI_BIT_OFFSET(n)))

/* Access helper */
#define NPCM_MIWU_IS_BIT_SET(reg, bit) (((reg >> bit) & 0x1) != 0)
#define NPCM_MIWU_IS_REG_BIT_SET(reg_name, base, n)                                                \
	(NPCM_MIWU_IS_BIT_SET(reg_name(base, NPCM_WUI_GROUP_OFFSET(n)), NPCM_WUI_BIT_OFFSET(n)))

/* Driver convenience defines */
#define NPCM_MIWU_DEV_CFG_BASE_INST(dev) ((const struct intc_miwu_config *)(dev)->config)->base
#define NPCM_MIWU_DEV_CFG_IDX_INST(dev)  ((const struct intc_miwu_config *)(dev)->config)->index
#define NPCM_MIWU_DEV_DATA_CB_INST(dev, group)                                                     \
	((struct intc_miwu_data *)(dev)->data)->cb_list_grp[group]

/* Driver config */
struct intc_miwu_config {
	uintptr_t base; /* The miwu controller base address */
	uint8_t index;  /* The index of miwu controller */
};

/* Driver data */
struct intc_miwu_data {
	/* Callback functions list for each MIWU group */
	sys_slist_t cb_list_grp[NPCM_MIWU_GROUP_MAX];
};

/* NPCM wake-up isr data */
typedef uint32_t npcm_miwu_isr_data_t;

#define NPCM_MIWU_ISR_TABLE_OFFSET(n)      (((n) >> 8) & 0xFF)
#define NPCM_MIWU_ISR_GROUP_MASK_OFFSET(n) ((n) & 0xFF)
#define NPCM_MIWU_ISR_PARAM(table, group_mask)                                                     \
	((npcm_miwu_isr_data_t)((((table) & 0xFF) << 8) | ((group_mask) & 0xFF)))

/* MIWU interrupt functions */
const struct device *npcm_miwu_dev_get(const struct npcm_wui *wui)
{
	uint8_t idx = NPCM_WUI_TABLE_OFFSET(wui->wk_src_idx);
	uint8_t i;

	for (i = 0; i < NPCM_MIWU_TABLE_MAX; i++) {
		if (idx == NPCM_MIWU_DEV_CFG_IDX_INST(miwu_devs[i])) {
			return miwu_devs[i];
		}
	}

	return NULL;
}

void npcm_miwu_io_enable(const struct npcm_wui *wui)
{
	const struct device *dev = npcm_miwu_dev_get(wui);

	if (!dev) {
		return;
	}

	NPCM_WK_SET(NPCM_WKINEN, NPCM_MIWU_DEV_CFG_BASE_INST(dev), wui->wk_src_idx);
}

void npcm_miwu_io_disable(const struct npcm_wui *wui)
{
	const struct device *dev = npcm_miwu_dev_get(wui);

	if (!dev) {
		return;
	}

	NPCM_WK_CLR(NPCM_WKINEN, NPCM_MIWU_DEV_CFG_BASE_INST(dev), wui->wk_src_idx);
}

void npcm_miwu_irq_enable(const struct npcm_wui *wui)
{
	const struct device *dev = npcm_miwu_dev_get(wui);

	if (!dev) {
		return;
	}

	NPCM_WK_SET(NPCM_WKEN, NPCM_MIWU_DEV_CFG_BASE_INST(dev), wui->wk_src_idx);
}

void npcm_miwu_irq_disable(const struct npcm_wui *wui)
{
	const struct device *dev = npcm_miwu_dev_get(wui);

	if (!dev) {
		return;
	}

	NPCM_WK_CLR(NPCM_WKEN, NPCM_MIWU_DEV_CFG_BASE_INST(dev), wui->wk_src_idx);
}

bool npcm_miwu_irq_get_state(const struct npcm_wui *wui)
{
	const struct device *dev = npcm_miwu_dev_get(wui);

	if (!dev) {
		return false;
	}

	return NPCM_MIWU_IS_REG_BIT_SET(NPCM_WKEN, NPCM_MIWU_DEV_CFG_BASE_INST(dev),
					wui->wk_src_idx);
}

void npcm_miwu_irq_clear_pending(const struct npcm_wui *wui)
{
	const struct device *dev = npcm_miwu_dev_get(wui);

	if (!dev) {
		return;
	}

	NPCM_WK_OVWR(NPCM_WKPCL, NPCM_MIWU_DEV_CFG_BASE_INST(dev), wui->wk_src_idx);
}

bool npcm_miwu_irq_get_and_clear_pending(const struct npcm_wui *wui)
{
	bool pending = npcm_miwu_irq_get_state(wui);

	if (pending) {
		npcm_miwu_irq_clear_pending(wui);
	}

	return pending;
}

void npcm_miwu_interrupt_mode_set(const struct npcm_wui *wui, enum miwu_int_mode mode,
				  enum miwu_int_trig trig)
{
	const struct device *dev = npcm_miwu_dev_get(wui);

	if (!dev) {
		return;
	}

	const uint32_t base = NPCM_MIWU_DEV_CFG_BASE_INST(dev);
	uint8_t id = wui->wk_src_idx;

	/* Set detection mode and enable interrupt */
	if (mode == NPCM_MIWU_MODE_LEVEL) {
		NPCM_WK_SET(NPCM_WKMOD, base, id);

		if (trig == NPCM_MIWU_TRIG_LOW) {
			NPCM_WK_SET(NPCM_WKEDG, base, id);
		} else if (trig == NPCM_MIWU_TRIG_HIGH) {
			NPCM_WK_CLR(NPCM_WKEDG, base, id);
		}
	} else {
		NPCM_WK_CLR(NPCM_WKMOD, base, id);

		if (trig == NPCM_MIWU_TRIG_LOW) {
			NPCM_WK_CLR(NPCM_WKAEDG, base, id);
			NPCM_WK_SET(NPCM_WKEDG, base, id);
		} else if (trig == NPCM_MIWU_TRIG_HIGH) {
			NPCM_WK_CLR(NPCM_WKAEDG, base, id);
			NPCM_WK_CLR(NPCM_WKEDG, base, id);
		} else {
			NPCM_WK_SET(NPCM_WKAEDG, base, id);
		}
	}
}

int npcm_miwu_interrupt_configure(const struct npcm_wui *wui, enum miwu_int_mode mode,
				  enum miwu_int_trig trig)
{
	/* Check trigger mode is valid */
	if ((mode == NPCM_MIWU_MODE_LEVEL) && (trig == NPCM_MIWU_TRIG_BOTH)) {
		return -EINVAL;
	}

	/* Disable interrupt of wake-up input source before configuration */
	npcm_miwu_irq_disable(wui);

	/* Set detection mode */
	npcm_miwu_interrupt_mode_set(wui, mode, trig);

	/* Enable wake-up input sources */
	npcm_miwu_io_enable(wui);

	/* Clear pending bit since it might be set if WKINEN bit changed */
	npcm_miwu_irq_clear_pending(wui);

	return 0;
}

/* MIWU callback functions */
void npcm_miwu_callback_init_gpio(struct miwu_callback *callback, const struct npcm_wui *io_wui,
				  int port)
{
	/* Initialize WUI and GPIO settings in unused bits field */
	callback->wui.wk_src_idx = io_wui->wk_src_idx;
	callback->cb_type = NPCM_MIWU_CALLBACK_GPIO;
}

void npcm_miwu_callback_init_dev(struct miwu_callback *callback, const struct npcm_wui *dev_wui,
				 miwu_dev_callback_handler_t handler, const struct device *source)
{
	/* Initialize WUI and input device settings */
	callback->wui.wk_src_idx = dev_wui->wk_src_idx;
	callback->dev_cb.source = source;
	callback->cb_type = NPCM_MIWU_CALLBACK_DEV;
	callback->dev_cb.handler = handler;
}

int npcm_miwu_callback_manage(struct miwu_callback *cb, bool set)
{
	__ASSERT(cb, "No callback!");

	const struct device *dev = npcm_miwu_dev_get(&cb->wui);

	__ASSERT(dev, "No MIWU device!");

	sys_slist_t *cb_list =
		&NPCM_MIWU_DEV_DATA_CB_INST(dev, NPCM_WUI_GROUP_OFFSET(cb->wui.wk_src_idx));

	if (!sys_slist_is_empty(cb_list)) {
		if (!sys_slist_find_and_remove(cb_list, &cb->node)) {
			if (!set) {
				return -EINVAL;
			}
		}
	}

	if (set) {
		sys_slist_prepend(cb_list, &cb->node);
	}

	return 0;
}

/* MIWU isr functions */
static void npcm_miwu_isr_dispatch(sys_slist_t *cb_list, uint8_t mask)
{
	struct miwu_callback *cb, *tmp;

	SYS_SLIST_FOR_EACH_CONTAINER_SAFE(cb_list, cb, tmp, node) {
		if (cb->cb_type == NPCM_MIWU_CALLBACK_GPIO) {
			if (BIT(NPCM_WUI_BIT_OFFSET(cb->wui.wk_src_idx)) & mask) {
				__ASSERT(cb->io_cb.handler, "No GPIO callback handler!");
				cb->io_cb.handler(gpio_npcm_dev_get(cb->io_cb.gpio_port),
						  (struct gpio_callback *)cb, cb->io_cb.pin_mask);
			}
		} else {
			if (BIT(NPCM_WUI_BIT_OFFSET(cb->wui.wk_src_idx)) & mask) {
				__ASSERT(cb->dev_cb.handler, "No device callback handler!");
				cb->dev_cb.handler(cb->dev_cb.source, &cb->wui);
			}
		}
	}
}

static void npcm_miwu_isr_priority(const struct npcm_wui *wui)
{
	const struct device *dev = npcm_miwu_dev_get(wui);

	__ASSERT(dev, "Invalid MIWU device!");

	const uint32_t base = NPCM_MIWU_DEV_CFG_BASE_INST(dev);
	uint8_t group = NPCM_WUI_GROUP_OFFSET(wui->wk_src_idx);
	sys_slist_t cbs = NPCM_MIWU_DEV_DATA_CB_INST(dev, group);
	uint8_t mask = NPCM_WKPND(base, group) & NPCM_WKEN(base, group);

	/* Clear pending bits before dispatch ISR */
	if (mask) {
		NPCM_WKPCL(base, group) = mask;
	}

	/* Dispatch registered gpio isrs */
	npcm_miwu_isr_dispatch(&cbs, mask);
}

/* MIWU interrupt function implementations */
#define NPCM_MIWU_ISR_FUNC_IMPL(inst)                                                              \
	static void intc_miwu_isr##inst(void *arg)                                                 \
	{                                                                                          \
		uint32_t data = (npcm_miwu_isr_data_t)arg;                                         \
		uint8_t table = NPCM_MIWU_ISR_TABLE_OFFSET(data);                                  \
		uint8_t group_mask = NPCM_MIWU_ISR_GROUP_MASK_OFFSET(data);                        \
		uint8_t group = 0;                                                                 \
		struct npcm_wui wui;                                                               \
                                                                                                   \
		/* Check all MIWU groups belong to the same irq */                                 \
		do {                                                                               \
			if (group_mask & 0x01) {                                                   \
				wui.wk_src_idx = NPCM_MIWU_TABLE_OFFSET(table) +                   \
						 NPCM_MIWU_GROUP_OFFSET(group);                    \
				npcm_miwu_isr_priority(&wui);                                      \
			}                                                                          \
			group++;                                                                   \
			group_mask >>= 1;                                                          \
		} while (group_mask != 0);                                                         \
	}
#define NPCM_DT_MIWU_IRQ_CONNECT_FUNC(node_id, inst)                                               \
	do {                                                                                       \
		IRQ_CONNECT(DT_PROP(node_id, nuvoton_irq), DT_PROP(node_id, nuvoton_irq_priority), \
			    intc_miwu_isr##inst,                                                   \
			    (void *)NPCM_MIWU_ISR_PARAM(DT_INST_PROP(inst, nuvoton_index),         \
							DT_PROP(node_id, nuvoton_group_mask)),     \
			    0);                                                                    \
		irq_enable(DT_PROP(node_id, nuvoton_irq));                                         \
	} while (0)
#define NPCM_DT_MIWU_IRQ_CONNECT_IMPL(node_id, inst) NPCM_DT_MIWU_IRQ_CONNECT_FUNC(node_id, inst);

/* MIWU initialization function implementations */
#define NPCM_MIWU_INIT_FUNC_IMPL(inst)                                                             \
	static int intc_miwu_init##inst(const struct device *dev)                                  \
	{                                                                                          \
		const uint32_t base = NPCM_MIWU_DEV_CFG_BASE_INST(dev);                            \
		uint8_t i;                                                                         \
                                                                                                   \
		/* Clear all MIWUs' pending and enable bits of MIWU device */                      \
		for (i = 0; i < NPCM_MIWU_GROUP_MAX; i++) {                                        \
			NPCM_WKEN(base, i) = 0;                                                    \
			NPCM_WKPCL(base, i) = 0xFF;                                                \
		}                                                                                  \
                                                                                                   \
		/* Config IRQ and MWIU group directly */                                           \
		DT_INST_FOREACH_CHILD_VARGS(inst, NPCM_DT_MIWU_IRQ_CONNECT_IMPL, inst)             \
                                                                                                   \
		return 0;                                                                          \
	}

/* MIWU driver registration */
#define NPCM_MIWU_INIT(inst)                                                                       \
	static int intc_miwu_init##inst(const struct device *dev);                                 \
	struct intc_miwu_data miwu_data_##inst;                                                    \
	static const struct intc_miwu_config miwu_config_##inst = {                                \
		.base = DT_INST_REG_ADDR(inst),                                                    \
		.index = DT_INST_PROP(inst, nuvoton_index),                                        \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, intc_miwu_init##inst, NULL, &miwu_data_##inst,                 \
			      &miwu_config_##inst, PRE_KERNEL_1, CONFIG_INTC_INIT_PRIORITY, NULL); \
                                                                                                   \
	NPCM_MIWU_ISR_FUNC_IMPL(inst)                                                              \
	NPCM_MIWU_INIT_FUNC_IMPL(inst)

DT_INST_FOREACH_STATUS_OKAY(NPCM_MIWU_INIT)
