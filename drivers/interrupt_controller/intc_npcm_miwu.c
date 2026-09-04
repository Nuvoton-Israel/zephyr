/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcm_miwu

#include <zephyr/drivers/interrupt_controller/intc_npcm_miwu.h>
#include <zephyr/irq.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(intc_npcm_miwu, CONFIG_INTC_LOG_LEVEL);

#define NPCM_MIWU_GROUPS 8

/*
 * Byte offsets of one input group's registers. Bit n represents input n in
 * every register. g is the zero-based group; groups 6 through 8 use a second
 * register block.
 */
#define NPCM_WKEDG(g) (0x00 + 2 * (g) + ((g) < 5 ? 0 : 0x1e))
#define NPCM_WKPND(g) (0x0a + 4 * (g) + ((g) < 5 ? 0 : 0x10))
#define NPCM_WKEN(g)  (0x1e + 2 * (g) + ((g) < 5 ? 0 : 0x12))

struct npcm_miwu_group_regs {
	uint8_t wkedg;  /* 1: low level / falling edge */
	uint8_t wkaedg; /* 1: any edge */
	uint8_t wkpnd;
	uint8_t wkpcl;
	uint8_t wken;
	uint8_t wkinen;
	uint8_t wkmod; /* 1: level detection */
};

#define NPCM_MIWU_GROUP_REGS(g, _)                                                                 \
	{                                                                                          \
		.wkedg = NPCM_WKEDG(g), .wkaedg = NPCM_WKEDG(g) + 1, .wkpnd = NPCM_WKPND(g),       \
		.wkpcl = NPCM_WKPND(g) + 2, .wken = NPCM_WKEN(g), .wkinen = NPCM_WKEN(g) + 1,      \
		.wkmod = 0x70 + (g),                                                               \
	}

static const struct npcm_miwu_group_regs npcm_miwu_regs[NPCM_MIWU_GROUPS] = {
	LISTIFY(NPCM_MIWU_GROUPS, NPCM_MIWU_GROUP_REGS, (,))};

/* Register bits selecting each trigger condition. */
static const struct {
	uint8_t mod;
	uint8_t edg;
	uint8_t aedg;
} npcm_wui_trig_bits[] = {
	[NPCM_WUI_TRIG_EDGE_RISING] = {.mod = 0, .edg = 0, .aedg = 0},
	[NPCM_WUI_TRIG_EDGE_FALLING] = {.mod = 0, .edg = 1, .aedg = 0},
	[NPCM_WUI_TRIG_EDGE_BOTH] = {.mod = 0, .edg = 0, .aedg = 1},
	[NPCM_WUI_TRIG_LEVEL_HIGH] = {.mod = 1, .edg = 0, .aedg = 0},
	[NPCM_WUI_TRIG_LEVEL_LOW] = {.mod = 1, .edg = 1, .aedg = 0},
};

struct npcm_miwu_config {
	mm_reg_t base;
	void (*irq_config)(void);
	uint8_t serviced; /* bit g set when this driver owns group g + 1's NVIC line */
};

struct npcm_miwu_data {
	sys_slist_t callbacks[NPCM_MIWU_GROUPS];
	struct k_spinlock lock;
};

/* IRQ_CONNECT() argument naming the group behind one NVIC line. */
struct npcm_miwu_irq {
	const struct device *dev;
	uint8_t group; /* zero-based */
};

static void npcm_reg_update(mm_reg_t addr, uint8_t mask, bool set)
{
	uint8_t val = sys_read8(addr);

	sys_write8(set ? (val | mask) : (val & ~mask), addr);
}

static const struct npcm_miwu_group_regs *npcm_wui_regs(const struct npcm_wui_dt_spec *wui)
{
	__ASSERT(wui->group >= 1 && wui->group <= NPCM_MIWU_GROUPS, "MIWU group must be 1 to 8");
	__ASSERT(wui->bit < 8, "MIWU input must be 0 to 7");

	return &npcm_miwu_regs[wui->group - 1];
}

int npcm_wui_configure(const struct npcm_wui_dt_spec *wui, enum npcm_wui_trigger trig)
{
	const struct npcm_miwu_config *cfg = wui->dev->config;
	const struct npcm_miwu_group_regs *r = npcm_wui_regs(wui);
	struct npcm_miwu_data *data = wui->dev->data;
	const uint8_t mask = BIT(wui->bit);
	k_spinlock_key_t key;

	if (trig >= ARRAY_SIZE(npcm_wui_trig_bits)) {
		return -EINVAL;
	}

	key = k_spin_lock(&data->lock);

	/* Disable while the detection changes so no false event is generated. */
	npcm_reg_update(cfg->base + r->wken, mask, false);
	npcm_reg_update(cfg->base + r->wkmod, mask, npcm_wui_trig_bits[trig].mod);
	npcm_reg_update(cfg->base + r->wkedg, mask, npcm_wui_trig_bits[trig].edg);
	npcm_reg_update(cfg->base + r->wkaedg, mask, npcm_wui_trig_bits[trig].aedg);
	sys_write8(mask, cfg->base + r->wkpcl); /* Clear events from the old configuration. */
	npcm_reg_update(cfg->base + r->wkinen, mask, true);

	k_spin_unlock(&data->lock, key);

	return 0;
}

void npcm_wui_enable(const struct npcm_wui_dt_spec *wui)
{
	const struct npcm_miwu_config *cfg = wui->dev->config;
	struct npcm_miwu_data *data = wui->dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	npcm_reg_update(cfg->base + npcm_wui_regs(wui)->wken, BIT(wui->bit), true);
	k_spin_unlock(&data->lock, key);
}

void npcm_wui_disable(const struct npcm_wui_dt_spec *wui)
{
	const struct npcm_miwu_config *cfg = wui->dev->config;
	struct npcm_miwu_data *data = wui->dev->data;
	k_spinlock_key_t key = k_spin_lock(&data->lock);

	npcm_reg_update(cfg->base + npcm_wui_regs(wui)->wken, BIT(wui->bit), false);
	k_spin_unlock(&data->lock, key);
}

void npcm_wui_clear_pending(const struct npcm_wui_dt_spec *wui)
{
	const struct npcm_miwu_config *cfg = wui->dev->config;

	/* Write-1-to-clear, so no lock is needed. */
	sys_write8(BIT(wui->bit), cfg->base + npcm_wui_regs(wui)->wkpcl);
}

bool npcm_wui_is_pending(const struct npcm_wui_dt_spec *wui)
{
	const struct npcm_miwu_config *cfg = wui->dev->config;

	return (sys_read8(cfg->base + npcm_wui_regs(wui)->wkpnd) & BIT(wui->bit)) != 0;
}

int npcm_wui_add_callback(const struct npcm_wui_dt_spec *wui, struct npcm_wui_callback *cb)
{
	const struct npcm_miwu_config *cfg = wui->dev->config;
	struct npcm_miwu_data *data = wui->dev->data;
	sys_slist_t *list = &data->callbacks[wui->group - 1];
	k_spinlock_key_t key;

	if ((cfg->serviced & BIT(wui->group - 1)) == 0) {
		return -ENOTSUP;
	}
	if (cb->handler == NULL) {
		return -EINVAL;
	}

	key = k_spin_lock(&data->lock);

	sys_slist_find_and_remove(list, &cb->node);
	cb->bit = wui->bit;
	sys_slist_prepend(list, &cb->node);

	k_spin_unlock(&data->lock, key);

	return 0;
}

int npcm_wui_remove_callback(const struct npcm_wui_dt_spec *wui, struct npcm_wui_callback *cb)
{
	struct npcm_miwu_data *data = wui->dev->data;
	sys_slist_t *list = &data->callbacks[wui->group - 1];
	k_spinlock_key_t key = k_spin_lock(&data->lock);
	bool found = sys_slist_find_and_remove(list, &cb->node);

	k_spin_unlock(&data->lock, key);

	return found ? 0 : -EINVAL;
}

static void npcm_miwu_isr(const void *arg)
{
	const struct npcm_miwu_irq *irq = arg;
	const struct npcm_miwu_config *cfg = irq->dev->config;
	const struct npcm_miwu_group_regs *r = &npcm_miwu_regs[irq->group];
	struct npcm_miwu_data *data = irq->dev->data;
	struct npcm_wui_callback *cb, *tmp;
	uint8_t pending = sys_read8(cfg->base + r->wkpnd) & sys_read8(cfg->base + r->wken);

	sys_write8(pending, cfg->base + r->wkpcl);

	SYS_SLIST_FOR_EACH_CONTAINER_SAFE(&data->callbacks[irq->group], cb, tmp, node) {
		if ((pending & BIT(cb->bit)) != 0) {
			cb->handler(cb);
		}
	}
}

static int npcm_miwu_init(const struct device *dev)
{
	const struct npcm_miwu_config *cfg = dev->config;

	/* Disable the serviced groups and clear any pending events. */
	for (int g = 0; g < NPCM_MIWU_GROUPS; g++) {
		if ((cfg->serviced & BIT(g)) == 0) {
			continue;
		}
		sys_write8(0, cfg->base + npcm_miwu_regs[g].wken);
		sys_write8(UINT8_MAX, cfg->base + npcm_miwu_regs[g].wkpcl);
	}

	cfg->irq_config();

	return 0;
}

/*
 * Each group drives one NVIC line. The driver services the groups whose
 * lines the node lists in interrupt-names.
 */
#define NPCM_MIWU_GROUP_NAMES wkinta, wkintb, wkintc, wkintd, wkinte, wkintf, wkintg, wkinth

#define NPCM_MIWU_SERVICED_BIT(g, name, n) (DT_INST_IRQ_HAS_NAME(n, name) ? BIT(g) : 0)
#define NPCM_MIWU_SERVICED(n)                                                                      \
	(FOR_EACH_IDX_FIXED_ARG(NPCM_MIWU_SERVICED_BIT, (|), n, NPCM_MIWU_GROUP_NAMES))

#define NPCM_MIWU_IRQ_DEFINE(g, name, n)                                                           \
	IF_ENABLED(DT_INST_IRQ_HAS_NAME(n, name),                                                  \
		   (static const struct npcm_miwu_irq npcm_miwu_irq_##n##_##name = {               \
			    .dev = DEVICE_DT_INST_GET(n), .group = g};))

#define NPCM_MIWU_IRQ_CONNECT(g, name, n)                                                          \
	IF_ENABLED(DT_INST_IRQ_HAS_NAME(n, name),                                                  \
		   (IRQ_CONNECT(DT_INST_IRQ_BY_NAME(n, name, irq),                                 \
				DT_INST_IRQ_BY_NAME(n, name, priority), npcm_miwu_isr,             \
				&npcm_miwu_irq_##n##_##name, 0);                                   \
		    irq_enable(DT_INST_IRQ_BY_NAME(n, name, irq));))

/* A group's NVIC line has one owner: a GPIO port or this driver, never both. */
#define NPCM_MIWU_GPIO_CLAIMS(gpio_node, n)                                                        \
	(DT_SAME_NODE(DT_PHANDLE(gpio_node, nuvoton_miwu), DT_DRV_INST(n)) &&                      \
	 (NPCM_MIWU_SERVICED(n) & BIT(DT_PROP(gpio_node, nuvoton_wui_group) - 1)) != 0) ||

#if DT_HAS_COMPAT_STATUS_OKAY(nuvoton_npcm_gpio)
#define NPCM_MIWU_OWNERSHIP_CHECK(n)                                                               \
	BUILD_ASSERT(                                                                              \
		!(DT_FOREACH_STATUS_OKAY_VARGS(nuvoton_npcm_gpio, NPCM_MIWU_GPIO_CLAIMS, n) 0),    \
		DT_NODE_PATH(DT_DRV_INST(n)) ": interrupts lists a GPIO port's group");
#else
#define NPCM_MIWU_OWNERSHIP_CHECK(n)
#endif

/* clang-format off */
#define NPCM_MIWU_INIT(n)                                                                          \
	NPCM_MIWU_OWNERSHIP_CHECK(n)                                                               \
	FOR_EACH_IDX_FIXED_ARG(NPCM_MIWU_IRQ_DEFINE, (), n, NPCM_MIWU_GROUP_NAMES)                 \
                                                                                                   \
	static void npcm_miwu_irq_config_##n(void)                                                 \
	{                                                                                          \
		FOR_EACH_IDX_FIXED_ARG(NPCM_MIWU_IRQ_CONNECT, (), n, NPCM_MIWU_GROUP_NAMES)        \
	}                                                                                          \
                                                                                                   \
	static const struct npcm_miwu_config npcm_miwu_cfg_##n = {                                 \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.irq_config = npcm_miwu_irq_config_##n,                                            \
		.serviced = NPCM_MIWU_SERVICED(n),                                                 \
	};                                                                                         \
                                                                                                   \
	static struct npcm_miwu_data npcm_miwu_data_##n;                                           \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, npcm_miwu_init, NULL, &npcm_miwu_data_##n, &npcm_miwu_cfg_##n,    \
			      PRE_KERNEL_1, CONFIG_INTC_INIT_PRIORITY, NULL);

/* clang-format on */

DT_INST_FOREACH_STATUS_OKAY(NPCM_MIWU_INIT)
