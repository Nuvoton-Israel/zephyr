/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcm_watchdog

/**
 * Nuvoton NPCM watchdog modules driver
 *
 * This file contains the drivers of NPCM Watchdog module that generates the clocks and interrupts
 * (T0 Timer) used for its callback functions in the system. It also provides watchdog reset signal
 * generation in response to a failure detection.
 * Please refer the block diagram for more detail.
 *
 *            +---------------------+    +-----------------+
 *  LFCLK --->| T0 Prescale Counter |-+->| 16-Bit T0 Timer |--------> T0 Timer
 * (32kHz)    |     (TWCP 1:32)     | |  |     (TWDT0)     |           Event
 *            +---------------------+ |  +-----------------+
 *  +---------------------------------+
 *  |
 *  |    +-------------------+    +-----------------+
 *  +--->| Watchdog Prescale |--->| 8-Bit Watchdog  |-----> Watchdog Event/Reset
 *       |    (WDCP 1:32)    |    | Counter (WDCNT) |       after n clocks
 *       +-------------------+    +-----------------+
 *
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/drivers/interrupt_controller/intc_npcm_miwu.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wdt_npcm, CONFIG_WDT_LOG_LEVEL);

/* Low Frequency clock 32.768 kHz */
#define LFCLK        32768UL
#define NPCM_WDT_CLK LFCLK

/* Pre-scaler */
#define NPCM_T0_PRESCALER  32
#define NPCM_WDT_PRESCALER 32

/*
 * Maximum watchdog window time.
 * The maximum time supported by 8-bits watchdog counter is
 *  256 * (32 * 32) / 32768 = 8 sec.
 */
#define NPCM_WDT_MAX_WND_TIME                                                                      \
	((1 << 8) * (NPCM_T0_PRESCALER * NPCM_WDT_PRESCALER) * 1000 / NPCM_WDT_CLK)

/*
 * Minimum watchdog window time.
 * Ensure we have waited at least 3 watchdog clocks since touching WD timer,
 *  3 / (32768 / 1024) HZ = 93.75 ms and round up to 100 ms.
 */
#define NPCM_WDT_MIN_WND_TIME ROUND_UP(3 * 1000 / (NPCM_WDT_CLK / 1024), 10)

/* Timeout for reloading and restarting Timer 0. (Unit: ms) */
#define NPCM_T0CSR_RST_TIMEOUT 2

/* Timeout for stopping watchdog. (Unit: ms) */
#define NPCM_WATCHDOG_STOP_TIMEOUT 1

/* Watchdog instances */
#define NPCM_WDT_UNLOCK_SEQ0 0x87
#define NPCM_WDT_UNLOCK_SEQ1 0x61
#define NPCM_WDT_UNLOCK_SEQ2 0x63
#define NPCM_WDT_FEED_VALUE  0x5C

/* Debug Interface registers */
#define NPCM_DBGCTRL_OFFSET 0x22
struct dbg_reg {
	/* 0x000: Debug Control */
	volatile uint8_t DBGCTRL;
	volatile uint8_t reserved1[0x53];
	/* 0x054: Debug Freeze Enable 1 */
	volatile uint8_t DBGFRZEN1;
	/* 0x055: Debug Freeze Enable 2 */
	volatile uint8_t DBGFRZEN2;
	/* 0x056: Debug Freeze Enable 3 */
	volatile uint8_t DBGFRZEN3;
	/* 0x057: Debug Freeze Enable 4 */
	volatile uint8_t DBGFRZEN4;
	/* 0x058: Debug Freeze Enable 5 */
	volatile uint8_t DBGFRZEN5;
};

/* Debug Interface registers fields */
#define NPCM_DBGFRZEN3_GLBL_FRZ_DIS 7

/*
 * Timer Watchdog (TWD) device registers
 */
struct twd_reg {
	/* 0x000: Timer and Watchdog Configuration */
	volatile uint8_t TWCFG;
	volatile uint8_t reserved1[1];
	/* 0x002: Timer and Watchdog Clock Prescaler */
	volatile uint8_t TWCP;
	volatile uint8_t reserved2[1];
	/* 0x004: TWD Timer 0 Counter Preset */
	volatile uint16_t TWDT0;
	/* 0x006: TWDT0 Control and Status */
	volatile uint8_t T0CSR;
	volatile uint8_t reserved3[1];
	/* 0x008: Watchdog Count */
	volatile uint8_t WDCNT;
	volatile uint8_t reserved4[1];
	/* 0x00A: Watchdog Service Data Match */
	volatile uint8_t WDSDM;
	volatile uint8_t reserved5[1];
	/* 0x00C: TWD Timer 0 Counter */
	volatile uint16_t TWMT0;
	/* 0x00E: Watchdog Counter */
	volatile uint8_t TWMWD;
	volatile uint8_t reserved6[1];
	/* 0x010: Watchdog Clock Prescaler */
	volatile uint8_t WDCP;
};

/* TWD register fields */
#define NPCM_TWCFG_LTWD_CFG  0 /* [0]-LTWCFG   : Set to lock TWCFG register */
#define NPCM_TWCFG_LTWCP     1 /* [1]-LTWCP    : Set to lock TWCP register */
#define NPCM_TWCFG_LTWDT0    2 /* [2]-LTWDT0   : Set to lock TWDT0 register */
#define NPCM_TWCFG_LWDCNT    3 /* [3]-LWDCNT   : Set to lock WDCNT register */
#define NPCM_TWCFG_WDCT0I    4 /* [4]-WDCT0I   : Set to select T0IN as watchdog prescaler clock */
#define NPCM_TWCFG_WDSDME    5 /* [5]-WDSDME   : Set to feed watchdog by writing 5Ch to WDSDM */
#define NPCM_T0CSR_RST       0 /* [0]-RST      : Set to force timer0 to reload and restart */
#define NPCM_T0CSR_TC        1 /* [1]-TC       : Timer0 counter reaches 0 */
#define NPCM_T0CSR_WDLTD     3 /* [3]-WDLTD    : Watchdog touch performed, 1=touched, 0=counting */
#define NPCM_T0CSR_WDRST_STS 4 /* [4]-WDRST_STS: Set to generate watchdog reset */
#define NPCM_T0CSR_WD_RUN    5 /* [5]-WD_RUN   : Watchdog counter status, 1=running, 0=stopped */
#define NPCM_T0CSR_T0EN      6 /* [6]-T0EN     : Set to enable t0out */
#define NPCM_T0CSR_TESDIS    7 /* [7]-TESDIS   : Set to disable watchdog event triggered */

/* Device config */
struct npcm_wdt_config {
	uintptr_t base_scfg;              /* The scfg device base address */
	uintptr_t base_twd;               /* The wdt controller base address */
	const struct npcm_wui timer0_wui; /* The timer wake-up input source configuration */
};

static const struct npcm_wdt_config npcm_wdt_cfg = {
	.base_scfg = DT_REG_ADDR_BY_NAME(DT_INST(0, nuvoton_npcm_pinctrl), scfg),
	.base_twd = DT_INST_REG_ADDR(0),
	.timer0_wui = DT_INST_PROP(0, nuvoton_timer0_wui),
};

/* Driver data */
struct npcm_wdt_data {
	int64_t last_watchdog_touch; /* Timestamp of touching watchdog last time. (Unit: ms) */
	wdt_callback_t cb;           /* Timeout callback used to handle watchdog event */
	uint32_t timeout;            /* Watchdog feed timeout. (Unit: ms) */
	bool timeout_installed;      /* Indicate whether a watchdog timeout is installed */
};
static struct npcm_wdt_data npcm_wdt_dev_data;
struct miwu_callback npcm_miwu_cb;

/* Driver convenience defines */
#define HAL_BASE_TWD_INST(dev)                                                                     \
	((struct twd_reg *)((const struct npcm_wdt_config *)(dev)->config)->base_twd)
#define HAL_BASE_DBGCTRL_INST(dev)                                                                 \
	((struct dbg_reg *)((const struct npcm_wdt_config *)(dev)->config)->base_scfg +            \
	 NPCM_DBGCTRL_OFFSET)
#define HAL_TIMER0_WUI_INST(dev)                                                                   \
	((const struct npcm_wui *)(&((const struct npcm_wdt_config *)(dev)->config)->timer0_wui))

/* Bit access helper */
#define NPCM_TWD_IS_BIT_SET(reg, bit)         (((reg >> bit) & (0x1)) != 0)
#define NPCM_TWD_GET_FIELD(reg, offset, size) (((reg) >> (offset)) & ((1 << (size)) - 1))

/* WDT local functions */
static inline int npcm_wdt_t0out_reload(const struct device *dev)
{
	struct twd_reg *const inst = HAL_BASE_TWD_INST(dev);
	uint64_t start_time;

	/* Reload and restart T0 timer */
	inst->T0CSR = (inst->T0CSR & ~BIT(NPCM_T0CSR_WDRST_STS)) | BIT(NPCM_T0CSR_RST);

	/* Wait for timer is loaded and restart */
	start_time = k_uptime_get();
	while (NPCM_TWD_IS_BIT_SET(inst->T0CSR, NPCM_T0CSR_RST)) {
		if (k_uptime_get() - start_time > NPCM_T0CSR_RST_TIMEOUT) {
			/* RST bit is still set? */
			if (NPCM_TWD_IS_BIT_SET(inst->T0CSR, NPCM_T0CSR_RST)) {
				LOG_ERR("Timeout: reload T0 timer!");
				return -ETIMEDOUT;
			}
		}
	}

	return 0;
}

static inline int npcm_wdt_wait_stopped(const struct device *dev)
{
	struct twd_reg *const inst = HAL_BASE_TWD_INST(dev);
	uint64_t start_time = k_uptime_get();

	/* If watchdog is still running? */
	while (NPCM_TWD_IS_BIT_SET(inst->T0CSR, NPCM_T0CSR_WD_RUN)) {
		if (k_uptime_get() - start_time > NPCM_WATCHDOG_STOP_TIMEOUT) {
			/* WD_RUN bit is still set? */
			if (NPCM_TWD_IS_BIT_SET(inst->T0CSR, NPCM_T0CSR_WD_RUN)) {
				LOG_ERR("Timeout: stop watchdog timer!");
				return -ETIMEDOUT;
			}
		}
	}

	return 0;
}

static void npcm_dbg_freeze_enable(const struct device *dev, bool enable)
{
	struct dbg_reg *const inst = HAL_BASE_DBGCTRL_INST(dev);

	inst->DBGFRZEN3 = enable ? (inst->DBGFRZEN3 & ~BIT(NPCM_DBGFRZEN3_GLBL_FRZ_DIS))
				 : (inst->DBGFRZEN3 | BIT(NPCM_DBGFRZEN3_GLBL_FRZ_DIS));
}

static void npcm_wdt_t0out_isr(const struct device *dev, struct npcm_wui *wui)
{
	ARG_UNUSED(wui);
	struct npcm_wdt_data *const data = dev->data;

	LOG_DBG("WDT reset will issue after %d delay cycle! WUI(%d %d %d)",
		CONFIG_WDT_NPCM_DELAY_CYCLES, NPCM_WUI_TABLE_OFFSET(wui->wk_src_idx),
		NPCM_WUI_GROUP_OFFSET(wui->wk_src_idx), NPCM_WUI_BIT_OFFSET(wui->wk_src_idx));

	/* Handle the watchdog event */
	if (data->cb) {
		data->cb(dev, 0);
	}
}

static void npcm_wdt_config_t0out_interrupt(const struct device *dev)
{
	struct twd_reg *const inst = HAL_BASE_TWD_INST(dev);
	const struct npcm_wui *timer0_wui = HAL_TIMER0_WUI_INST(dev);

	/* Enable t0out */
	inst->T0CSR |= BIT(NPCM_T0CSR_T0EN);

	/* Initialize a MIWU device input and its callback function */
	npcm_miwu_callback_init_dev(&npcm_miwu_cb, timer0_wui, npcm_wdt_t0out_isr, dev);
	npcm_miwu_callback_manage(&npcm_miwu_cb, true);

	/* Configure the T0 wake-up event triggered from a rising edge on T0OUT signal */
	npcm_miwu_interrupt_configure(timer0_wui, NPCM_MIWU_MODE_EDGE, NPCM_MIWU_TRIG_HIGH);
}

/* WDT API functions */
static int npcm_wdt_setup(const struct device *dev, uint8_t options)
{
	struct twd_reg *const inst = HAL_BASE_TWD_INST(dev);
	const struct npcm_wui *timer0_wui = HAL_TIMER0_WUI_INST(dev);
	struct npcm_wdt_data *const data = dev->data;
	int rv;

	/* First, disable irq of t0-out expired event */
	npcm_miwu_irq_disable(timer0_wui);

	/* Check timeout is installed */
	if (!data->timeout_installed) {
		LOG_ERR("No valid WDT timeout installed");
		return -EINVAL;
	}

	/* Check WDT timer is not busy */
	if (NPCM_TWD_IS_BIT_SET(inst->T0CSR, NPCM_T0CSR_WD_RUN)) {
		LOG_ERR("WDT timer is busy");
		return -EBUSY;
	}

	/* Check WDT options is supported */
	if ((options & WDT_OPT_PAUSE_IN_SLEEP) != 0) {
		LOG_ERR("WDT_OPT_PAUSE_IN_SLEEP is not supported");
		return -ENOTSUP;
	}

	/* Stall the WDT counter when halted by debugger */
	npcm_dbg_freeze_enable(dev, (options & WDT_OPT_PAUSE_HALTED_BY_DBG) != 0);

	/*
	 * One clock period of T0 timer is 32/32.768 KHz = 0.976 ms.
	 * Then the counter value is timeout/0.976 - 1.
	 */
	inst->TWDT0 =
		MAX(DIV_ROUND_UP(data->timeout * NPCM_WDT_CLK, NPCM_T0_PRESCALER * 1000) - 1, 1);

	/* Configure 8-bit watchdog counter */
	inst->WDCNT =
		MIN(DIV_ROUND_UP(data->timeout, NPCM_WDT_PRESCALER) + CONFIG_WDT_NPCM_DELAY_CYCLES,
		    0xff);

	LOG_DBG("WDT setup: TWDT0, WDCNT are %d, %d", inst->TWDT0, inst->WDCNT);

	/* Reload and restart T0 timer */
	rv = npcm_wdt_t0out_reload(dev);

	/* Configure t0 timer interrupt and its isr */
	npcm_wdt_config_t0out_interrupt(dev);

	/* Enable irq of t0-out expired event */
	npcm_miwu_irq_enable(timer0_wui);

	return rv;
}

static int npcm_wdt_disable(const struct device *dev)
{
	struct twd_reg *const inst = HAL_BASE_TWD_INST(dev);
	const struct npcm_wui *timer0_wui = HAL_TIMER0_WUI_INST(dev);
	struct npcm_wdt_data *const data = dev->data;

	/* Ensure we have waited at least 3 watchdog ticks before stopping watchdog */
	while (k_uptime_get() - data->last_watchdog_touch < NPCM_WDT_MIN_WND_TIME) {
		continue;
	}

	/* Stop and unlock watchdog by writing 87h, 61h and 63h sequence bytes to WDSDM register */
	inst->WDSDM = NPCM_WDT_UNLOCK_SEQ0;
	inst->WDSDM = NPCM_WDT_UNLOCK_SEQ1;
	inst->WDSDM = NPCM_WDT_UNLOCK_SEQ2;

	/* Disable irq of t0-out expired event and mark it uninstalled */
	npcm_miwu_irq_disable(timer0_wui);
	data->timeout_installed = false;

	/* Wait until watchdog is stopped */
	return npcm_wdt_wait_stopped(dev);
}

static int npcm_wdt_install_timeout(const struct device *dev, const struct wdt_timeout_cfg *cfg)
{
	struct twd_reg *const inst = HAL_BASE_TWD_INST(dev);
	struct npcm_wdt_data *const data = dev->data;

	/* If watchdog is already running */
	if (NPCM_TWD_IS_BIT_SET(inst->T0CSR, NPCM_T0CSR_WD_RUN)) {
		return -EBUSY;
	}

	/* No window watchdog support */
	if (cfg->window.min != 0) {
		data->timeout_installed = false;
		return -EINVAL;
	}

	/*
	 * The allowed range of 1-8000 in milliseconds.
	 * Check if the provided value is within this range.
	 */
	if (cfg->window.max > NPCM_WDT_MAX_WND_TIME || cfg->window.max == 0) {
		data->timeout_installed = false;
		return -EINVAL;
	}

	/* Save watchdog timeout */
	data->timeout = cfg->window.max;

	/* Install user timeout isr */
	data->cb = cfg->callback;
	data->timeout_installed = true;

	/* Channel ID is always 0 */
	return 0;
}

static int npcm_wdt_feed(const struct device *dev, int channel_id)
{
	ARG_UNUSED(channel_id);
	struct twd_reg *const inst = HAL_BASE_TWD_INST(dev);
	struct npcm_wdt_data *const data = dev->data;

	/* Feed watchdog by writing 5Ch to WDSDM */
	inst->WDSDM = NPCM_WDT_FEED_VALUE;
	data->last_watchdog_touch = k_uptime_get();

	/* Reload and restart T0 timer */
	return npcm_wdt_t0out_reload(dev);
}

/* WDT driver APIs */
static DEVICE_API(wdt, npcm_wdt_driver_api) = {
	.setup = npcm_wdt_setup,
	.disable = npcm_wdt_disable,
	.install_timeout = npcm_wdt_install_timeout,
	.feed = npcm_wdt_feed,
};

/* WDT initialization function */
static int npcm_wdt_init(const struct device *dev)
{
	struct twd_reg *const inst = HAL_BASE_TWD_INST(dev);

#ifdef CONFIG_WDT_DISABLE_AT_BOOT
	npcm_wdt_disable(dev);
#endif

	/* Setup watchdog configs */
	inst->TWCFG = BIT(NPCM_TWCFG_WDSDME) | BIT(NPCM_TWCFG_WDCT0I);

	/* Disable early touch functionality */
	inst->T0CSR = (inst->T0CSR & ~BIT(NPCM_T0CSR_WDRST_STS)) | BIT(NPCM_T0CSR_TESDIS);

	/* Find the power of 2 to the pre-scale ratio */
	inst->TWCP = LOG2(NPCM_T0_PRESCALER);  /* T0 Timer freq is LFCLK/32 Hz */
	inst->WDCP = LOG2(NPCM_WDT_PRESCALER); /* Watchdog freq is T0CLK/32 Hz */

	return 0;
}

/* WDT driver registration */
DEVICE_DT_INST_DEFINE(0, npcm_wdt_init, NULL, &npcm_wdt_dev_data, &npcm_wdt_cfg, PRE_KERNEL_1,
		      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &npcm_wdt_driver_api);
