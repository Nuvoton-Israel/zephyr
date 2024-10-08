/*
 * Copyright (c) 2020 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcm4xx_espi

#include <assert.h>
#include <drivers/espi.h>
#include <drivers/gpio.h>
#include <drivers/clock_control.h>
#include <dt-bindings/espi/npcm4xx_espi.h>
#include <kernel.h>
#include <soc.h>
#include "espi_utils.h"
#include "soc_host.h"
#include "soc_miwu.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(espi, CONFIG_ESPI_LOG_LEVEL);


/* Driver convenience defines */
#define DRV_CONFIG(dev) ((const struct espi_npcm4xx_config *)(dev)->config)

#define DRV_DATA(dev) ((struct espi_npcm4xx_data *)(dev)->data)

#define HAL_INSTANCE(dev) (struct espi_reg *)(DRV_CONFIG(dev)->base)

/* eSPI channels */
#define NPCM4XX_ESPI_CH_PC              0
#define NPCM4XX_ESPI_CH_VW              1
#define NPCM4XX_ESPI_CH_OOB             2
#define NPCM4XX_ESPI_CH_FLASH           3
#define NPCM4XX_ESPI_CH_COUNT           4
#define NPCM4XX_ESPI_HOST_CH_EN(ch)     (ch + 4)

/* eSPI max supported frequency */
#define NPCM4XX_ESPI_MAXFREQ_20         0
#define NPCM4XX_ESPI_MAXFREQ_25         1
#define NPCM4XX_ESPI_MAXFREQ_33         2
#define NPCM4XX_ESPI_MAXFREQ_50         3
#define NPCM4XX_ESPI_MAXFREQ_66         4

/* Minimum delay before acknowledging a virtual wire */
#define NPCM4XX_ESPI_VWIRE_ACK_DELAY    10ul /* 10 us */

/* OOB channel maximum payload size */
#define NPCM4XX_ESPI_OOB_MAX_PAYLOAD    64
#define NPCM4XX_OOB_RX_PACKAGE_LEN(hdr) (((hdr & 0xff000000) >> 24) | \
							((hdr & 0xf0000) >> 8))

/* eSPI cycle type field for OOB and FLASH channels */
#define ESPI_FLASH_READ_CYCLE_TYPE   0x00
#define ESPI_FLASH_WRITE_CYCLE_TYPE  0x01
#define ESPI_FLASH_ERASE_CYCLE_TYPE  0x02
#define ESPI_OOB_GET_CYCLE_TYPE      0x21
#define ESPI_OOB_TAG                 0x00
#define ESPI_OOB_MAX_TIMEOUT         500ul /* 500 ms */

/* SM GPIO: 0~63, MS GPIO: 64~127 */
#define VM_SMGPIO_START		0
#define VW_SMGPIO_NUM		64
#define VM_MSGPIO_START		64
#define VW_MSGPIO_NUM		64
#define VWGPMS_MODIFIED	BIT(16)

struct espi_npcm4xx_config {
	uintptr_t base;
	/* clock configuration */
	struct npcm4xx_clk_cfg clk_cfg;
	/* mapping table between eSPI reset signal and wake-up input */
	struct npcm4xx_wui espi_rst_wui;
};

struct vwgpio_event {
	uint8_t enable : 1;
	uint8_t state: 1;
	uint8_t type: 3;
	uint8_t flags: 3;
};

struct espi_npcm4xx_data {
	sys_slist_t callbacks;
	uint8_t plt_rst_asserted;
	uint8_t espi_rst_asserted;
	uint8_t sx_state;
#if defined(CONFIG_ESPI_OOB_CHANNEL)
	struct k_sem oob_rx_lock;
#endif
	struct vwgpio_event events[VW_MSGPIO_NUM];
};

/* eSPI bus interrupt configuration structure and macro function */
struct espi_bus_isr {
	uint8_t status_bit; /* bit order in ESPISTS register */
	uint8_t int_en_bit; /* bit order in ESPIIE register */
	uint8_t wake_en_bit; /* bit order in ESPIWE register */
	void (*bus_isr)(const struct device *dev); /* eSPI bus ISR */
};

#define NPCM4XX_ESPI_BUS_INT_ITEM(event, isr) {     \
	.status_bit = NPCM4XX_ESPISTS_##event,      \
	.int_en_bit = NPCM4XX_ESPIIE_##event##IE,   \
	.wake_en_bit = NPCM4XX_ESPIWE_##event##WE,  \
	.bus_isr = isr }

/* eSPI Virtual Wire Input (Master-to-Slave) signals configuration structure */
struct npcm4xx_vw_in_config {
	enum espi_vwire_signal sig; /* Virtual Wire signal */
	uint8_t  reg_idx; /* register index for VW signal */
	uint8_t  bitmask; /* VW signal bits-mask */
	struct npcm4xx_wui vw_wui; /* WUI mapping in MIWU modules for VW signal */
};

/* eSPI Virtual Wire Output (Slave-to-Master) signals configuration structure */
struct npcm4xx_vw_out_config {
	enum espi_vwire_signal sig; /* Virtual Wire signal */
	uint8_t  reg_idx; /* register index for VW signal */
	uint8_t  bitmask; /* VW signal bits-mask */
};

/* eSPI local bus interrupt service functions */
static void espi_bus_err_isr(const struct device *dev)
{
	struct espi_reg *const inst = HAL_INSTANCE(dev);
	uint32_t err = inst->ESPIERR;

	LOG_ERR("eSPI Bus Error %08X", err);
	/* Clear error status bits */
	inst->ESPIERR = err;
}

static void espi_bus_inband_rst_isr(const struct device *dev)
{
	ARG_UNUSED(dev);
	LOG_DBG("%s issued", __func__);
}

static void espi_bus_reset_isr(const struct device *dev)
{
	ARG_UNUSED(dev);
	LOG_DBG("%s issued", __func__);
	/* Do nothing! This signal is handled in ESPI_RST VW signal ISR */
}

static void espi_bus_cfg_update_isr(const struct device *dev)
{
	int chan;
	struct espi_reg *const inst = HAL_INSTANCE(dev);
	struct espi_npcm4xx_data *const data = DRV_DATA(dev);
	struct espi_event evt = { .evt_type = ESPI_BUS_EVENT_CHANNEL_READY,
				  .evt_details = 0,
				  .evt_data = 0 };
	/* If host enable bits are not sync with ready bits on slave side. */
	uint8_t chg_mask = GET_FIELD(inst->ESPICFG, NPCM4XX_ESPICFG_HCHANS_FIELD)
			 ^ GET_FIELD(inst->ESPICFG, NPCM4XX_ESPICFG_CHANS_FIELD);
	chg_mask &= (ESPI_CHANNEL_VWIRE | ESPI_CHANNEL_OOB |
							ESPI_CHANNEL_FLASH);

	LOG_DBG("ESPI CFG Change Updated! 0x%02X", chg_mask);
	/*
	 * If host enable/disable channel for VW/OOB/FLASH, npcm4xx should follow
	 * except Peripheral channel. It is handled after receiving PLTRST
	 * event separately.
	 */
	for (chan = NPCM4XX_ESPI_CH_VW; chan < NPCM4XX_ESPI_CH_COUNT; chan++) {
		/* Channel ready bit isn't sync with enabled bit on host side */
		if (chg_mask & BIT(chan)) {
			evt.evt_data = IS_BIT_SET(inst->ESPICFG,
						NPCM4XX_ESPI_HOST_CH_EN(chan));
			evt.evt_details = BIT(chan);

			if (evt.evt_data)
				inst->ESPICFG |= BIT(chan);
			else
				inst->ESPICFG &= ~BIT(chan);

			espi_send_callbacks(&data->callbacks, dev, evt);
		}
	}
	LOG_DBG("ESPI CFG Updated! 0x%02X", GET_FIELD(inst->ESPICFG,
						NPCM4XX_ESPICFG_CHANS_FIELD));
}

static int espi_vwgpio_get_value(const struct device *dev, unsigned int offset)
{
	struct espi_reg *const inst = HAL_INSTANCE(dev);
	uint32_t wire = offset % 4;
	uint32_t index;
	uint32_t val;

	/* Accept SM/MS GPIO */
	if (offset >= (VM_MSGPIO_START + VW_MSGPIO_NUM))
		return -EINVAL;

	if (offset >= VM_MSGPIO_START) {
		index = (offset - VM_MSGPIO_START) / 4;
		val = inst->VWGPMS[index];

		/* Check wire valid bit, invalid return default value */
		if (!(val & BIT(wire + 4)))
			return !!(BIT(offset - VM_MSGPIO_START));
	} else {
		index = offset / 4;
		val = inst->VWGPMS[index];
		/* Check wire valid bit*/
		if (!(val & BIT(wire + 4)))
			return -EIO;
	}

	return !!(val & BIT(wire));
}


static void espi_npcm_vwgpio_check_event(const struct device *dev,
				    unsigned int event_idx)
{
	struct espi_reg *const inst = HAL_INSTANCE(dev);
	struct espi_npcm4xx_data *const vwgpio = DRV_DATA(dev);
	struct vwgpio_event *event;
	bool raise_irq = false;
	uint32_t index = event_idx / 4;
	uint32_t wire = event_idx % 4;
	uint8_t new_state;
	uint32_t val;

	if (event_idx >= VW_MSGPIO_NUM)
		return;

	val = inst->VWGPMS[index];
	inst->VWGPMS[index] |= VWGPMS_MODIFIED;

	event = &vwgpio->events[event_idx];

	/* Check event enable 
	if (!event->enable)
		goto out;*/

	/* Check wire valid bit */
	if (!(val & BIT(wire + 4)))
		goto out;

	new_state = !!(val & BIT(wire));
	if (event->state  != new_state) {
		event->state = new_state;
		raise_irq = true;
	}

out:
	if (raise_irq) {
		struct espi_event evt = { ESPI_BUS_EVENT_VWIRE_RECEIVED, 0, 0 };
		evt.evt_details = VM_MSGPIO_START + event_idx;
		evt.evt_data = event->state;

		LOG_INF("GPIO NUM %d value %d", evt.evt_details, evt.evt_data);

		espi_send_callbacks(&vwgpio->callbacks, dev, evt);
	}
}

static void espi_npcm_vwgpio_init(const struct device *dev)
{
	struct espi_npcm4xx_data *const vwgpio = DRV_DATA(dev);

	/* Get gpio initial state */
	memset(&vwgpio->events, 0, sizeof(vwgpio->events));
	for (int i = 0; i < VW_MSGPIO_NUM; i++)
		vwgpio->events[i].state =
			espi_vwgpio_get_value(dev, VM_MSGPIO_START + i);
}

static void espi_bus_vw_update_isr(const struct device *dev)
{
	LOG_INF("ESPI VW Updated!");

	for (int i = 0; i < VW_MSGPIO_NUM; i++) {
		espi_npcm_vwgpio_check_event(dev, i);
	}
}


const struct espi_bus_isr espi_bus_isr_tbl[] = {
	NPCM4XX_ESPI_BUS_INT_ITEM(BERR, espi_bus_err_isr),
	NPCM4XX_ESPI_BUS_INT_ITEM(IBRST, espi_bus_inband_rst_isr),
	NPCM4XX_ESPI_BUS_INT_ITEM(ESPIRST, espi_bus_reset_isr),
	NPCM4XX_ESPI_BUS_INT_ITEM(CFGUPD, espi_bus_cfg_update_isr),
	NPCM4XX_ESPI_BUS_INT_ITEM(VWUPD, espi_bus_vw_update_isr),
};

static void espi_bus_generic_isr(void *arg)
{
	const struct device *dev = (const struct device *)arg;
	struct espi_reg *const inst = HAL_INSTANCE(dev);
	int i;
	uint32_t mask, status;

	/*
	 * We need to set the same bit in mask in case bit 27
	 * in ESPISTS of npcm4xx is not cleared in ISR.
	 */
	mask = inst->ESPIIE ;
	status = inst->ESPISTS & mask;

	/* Clear pending bits of status register first */
	inst->ESPISTS = status;

	LOG_DBG("%s: 0x%08X", __func__, status);
	for (i = 0; i < ARRAY_SIZE(espi_bus_isr_tbl); i++) {
		struct espi_bus_isr entry = espi_bus_isr_tbl[i];

		if (status & BIT(entry.status_bit)) {
			if (entry.bus_isr != NULL) {
				entry.bus_isr(dev);
			}
		}
	}
}

/* eSPI api functions */
static int espi_npcm4xx_configure(const struct device *dev, struct espi_cfg *cfg)
{
	struct espi_reg *const inst = HAL_INSTANCE(dev);

	uint8_t max_freq = 0;
	uint8_t cur_io_mode, io_mode = 0;

	/* Configure eSPI frequency */
	switch (cfg->max_freq) {
	case 20:
		max_freq = NPCM4XX_ESPI_MAXFREQ_20;
		break;
	case 25:
		max_freq = NPCM4XX_ESPI_MAXFREQ_25;
		break;
	case 33:
		max_freq = NPCM4XX_ESPI_MAXFREQ_33;
		break;
	case 50:
		max_freq = NPCM4XX_ESPI_MAXFREQ_50;
		break;
	case 66:
		max_freq = NPCM4XX_ESPI_MAXFREQ_66;
		break;
	default:
		return -EINVAL;
	}
	SET_FIELD(inst->ESPICFG, NPCM4XX_ESPICFG_MAXFREQ_FIELD, max_freq);

	/* Configure eSPI IO mode */
	io_mode = (cfg->io_caps >> 1);
	if (io_mode > 3) {
		return -EINVAL;
	}

	cur_io_mode = GET_FIELD(inst->ESPICFG, NPCM4XX_ESPICFG_IOMODE_FIELD);
	if (io_mode != cur_io_mode) {
		SET_FIELD(inst->ESPICFG, NPCM4XX_ESPICFG_IOMODE_FIELD, io_mode);
	}


	inst->ESPICFG |= BIT(NPCM4XX_ESPICFG_VWMS_VALID_EN);
	inst->ESPICFG |= BIT(NPCM4XX_ESPICFG_VWSM_VALID_EN);

	/* Configure eSPI supported channels */
	if (cfg->channel_caps & ESPI_CHANNEL_PERIPHERAL)
		inst->ESPICFG |= BIT(NPCM4XX_ESPICFG_PCCHN_SUPP);

	if (cfg->channel_caps & ESPI_CHANNEL_VWIRE)
		inst->ESPICFG |= BIT(NPCM4XX_ESPICFG_VWCHN_SUPP);

	if (cfg->channel_caps & ESPI_CHANNEL_OOB)
		inst->ESPICFG |= BIT(NPCM4XX_ESPICFG_OOBCHN_SUPP);

	if (cfg->channel_caps & ESPI_CHANNEL_FLASH)
		inst->ESPICFG |= BIT(NPCM4XX_ESPICFG_FLASHCHN_SUPP);

	LOG_DBG("%s: %d %d ESPICFG: 0x%08X", __func__,
				max_freq, io_mode, inst->ESPICFG);

	return 0;
}

static bool espi_npcm4xx_channel_ready(const struct device *dev,
					enum espi_channel ch)
{
	struct espi_reg *const inst = HAL_INSTANCE(dev);
	bool sts;

	switch (ch) {
	case ESPI_CHANNEL_PERIPHERAL:
		sts = IS_BIT_SET(inst->ESPICFG, NPCM4XX_ESPICFG_PCHANEN);
		break;
	case ESPI_CHANNEL_VWIRE:
		sts = IS_BIT_SET(inst->ESPICFG, NPCM4XX_ESPICFG_VWCHANEN);
		break;
	case ESPI_CHANNEL_OOB:
		sts = IS_BIT_SET(inst->ESPICFG, NPCM4XX_ESPICFG_OOBCHANEN);
		break;
	case ESPI_CHANNEL_FLASH:
		sts = IS_BIT_SET(inst->ESPICFG, NPCM4XX_ESPICFG_FLASHCHANEN);
		break;
	default:
		sts = false;
		break;
	}

	return sts;
}

static int espi_npcm4xx_manage_callback(const struct device *dev,
				    struct espi_callback *callback, bool set)
{
	struct espi_npcm4xx_data *const data = DRV_DATA(dev);

	return espi_manage_callback(&data->callbacks, callback, set);
}


/* eSPI driver registration */
static int espi_npcm4xx_init(const struct device *dev);

static const struct espi_driver_api espi_npcm4xx_driver_api = {
	.config = espi_npcm4xx_configure,
	.get_channel_status = espi_npcm4xx_channel_ready,
	.manage_callback = espi_npcm4xx_manage_callback,
};

static struct espi_npcm4xx_data espi_npcm4xx_data;

static const struct espi_npcm4xx_config espi_npcm4xx_config = {
	.base = DT_INST_REG_ADDR(0),
	.espi_rst_wui = NPCM4XX_DT_WUI_ITEM_BY_NAME(0, espi_rst_wui),
	.clk_cfg = NPCM4XX_DT_CLK_CFG_ITEM(0),
};

DEVICE_DT_INST_DEFINE(0, &espi_npcm4xx_init, NULL,
		    &espi_npcm4xx_data, &espi_npcm4xx_config,
		    PRE_KERNEL_2, CONFIG_ESPI_INIT_PRIORITY,
		    &espi_npcm4xx_driver_api);

static int espi_npcm4xx_init(const struct device *dev)
{
	const struct espi_npcm4xx_config *const config = DRV_CONFIG(dev);
	struct espi_npcm4xx_data *const data = DRV_DATA(dev);
	struct espi_reg *const inst = HAL_INSTANCE(dev);
	const struct device *clk_dev = device_get_binding(NPCM4XX_CLK_CTRL_NAME);
	int i, ret;

#define ESPI_FREQ_20MHZ       20u
#define ESPI_FREQ_25MHZ       25u
#define ESPI_FREQ_33MHZ       33u
#define ESPI_FREQ_50MHZ       50u
#define ESPI_FREQ_66MHZ       66u

	/* Indicate to eSPI master simplest configuration: Single line,
	 * 20MHz frequency and only logical channel 0 and 1 are supported
	 */
	struct espi_cfg cfg = {
		.io_caps = ESPI_IO_MODE_SINGLE_LINE | ESPI_IO_MODE_DUAL_LINES
			   | ESPI_IO_MODE_QUAD_LINES,
		.channel_caps = ESPI_CHANNEL_VWIRE | ESPI_CHANNEL_PERIPHERAL,
		.max_freq = ESPI_FREQ_20MHZ,
	};

	/* If eSPI driver supports additional capabilities use them */
#ifdef CONFIG_ESPI_OOB_CHANNEL
	cfg.channel_caps |= ESPI_CHANNEL_OOB;
#endif
	inst->ESPICFG &= ~BIT(NPCM4XX_ESPICFG_VWCHANEN);
	/* Turn on eSPI device clock first */
	ret = clock_control_on(clk_dev, (clock_control_subsys_t *)
							&config->clk_cfg);
	if (ret < 0) {
		LOG_ERR("Turn on eSPI clock fail %d", ret);
		return ret;
	}

	espi_npcm_vwgpio_init(dev);

	for (i = 0; i < ARRAY_SIZE(inst->VWGPMS); i++) {
		inst->VWGPMS[i] |= (BIT(NPCM4XX_VWGPMS_INDEX_EN) | BIT(NPCM4XX_VWGPMS_IE));
		inst->VWGPMS[i] &= ~BIT(NPCM4XX_VWGPMS_ENESPIRST);
	}

	for (i = 0; i < ARRAY_SIZE(inst->VWGPSM); i++) {
		inst->VWGPSM[i] |= (BIT(NPCM4XX_VWGPSM_INDEX_EN) | BIT(NPCM4XX_VWGPSM_IE));
	}

	/* Enable events which share the same espi bus interrupt */
	for (i = 0; i < ARRAY_SIZE(espi_bus_isr_tbl); i++) {
		inst->ESPIIE |= BIT(espi_bus_isr_tbl[i].int_en_bit);
	}

	inst->ESPIHINDP |= (0xf);

	espi_npcm4xx_configure(dev, &cfg);
	/* Configure host sub-modules which HW blocks belong to core domain */
	npcm4xx_host_init_subs_core_domain(dev, &data->callbacks);

	/* eSPI Bus interrupt installation */
	IRQ_CONNECT(DT_INST_IRQN(0),
		    DT_INST_IRQ(0, priority),
		    espi_bus_generic_isr,
		    DEVICE_DT_INST_GET(0), 0);

	/* Enable eSPI bus interrupt */
	irq_enable(DT_INST_IRQN(0));
	return 0;
}
