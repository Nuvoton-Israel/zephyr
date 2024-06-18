/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcm_adc

#include <zephyr/drivers/adc.h>
#include <zephyr/dt-bindings/adc/npcm_adc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_npcm, CONFIG_ADC_LOG_LEVEL);

/* ADC voltage channel configs */
struct adc_npcm_channel_cfg {
	uint8_t ctrl0_ch_sel_value; /* Channel selection value in CTRL0 */
	uint8_t ctrl6_bit_offset;   /* Bit n of CTRL6 */
	uint8_t ctrl6_bit_set: 1;   /* Bit value in CTRL6 */
	uint8_t is_diode: 1;        /* Diode or Voltage */
	uint8_t is_vin_thr: 1;      /* VIN/THR multi config */
} __packed;

/* Device config */
struct adc_npcm_config {
	uintptr_t base;                                 /* ADC controller base address */
	const uint8_t channel_count;                    /* Number of ADC channels */
	void (*irq_cfg_func)(void);                     /* Routine for configuring ADC's ISR */
	const struct pinctrl_dev_config *pin_dev_cfg;   /* Pinctrl device config */
	const struct adc_npcm_channel_cfg *channel_cfg; /* ADC channel configurations */
};

/* Driver data */
typedef uint16_t adc_npcm_buf_data_t;
struct adc_npcm_data {
	struct adc_context ctx;       /* Mutex of ADC channels */
	uint8_t CurChannel_Idx;       /* Current channel index */
	uint32_t channels;            /* Bit-mask for channels included in each sampling sequence */
	const struct device *adc_dev; /* ADC Device pointer used in api functions */
	adc_npcm_buf_data_t *buffer;  /* Buffer to store ADC data */
	adc_npcm_buf_data_t *repeat_buffer; /* Buffer to store ADC data for repeat mode */
	adc_npcm_buf_data_t *buf_end;       /* End pointer of buffer */
	uint16_t ref_internal;              /* ADC internal reference */
};

/* Driver convenience defines */
#define HAL_BASE_CFG_INST(dev)                                                                     \
	((struct adc_reg *)((const struct adc_npcm_config *)(dev)->config)->base)
#define HAL_CFG_CHANNEL_CFG_INST(dev) ((const struct adc_npcm_config *)(dev->config))->channel_cfg

/*
 * Analog-To-Digital Converter (ADC) device registers
 */
struct adc_reg {
	volatile uint16_t RESERVED0;
	/* 0x02: DSADC control register0 */
	volatile uint8_t DSADCCTRL0;
	volatile uint8_t RESERVED1[11];
	/* 0x0E: Operation Mode select */
	volatile uint16_t ADCTM;
	volatile uint16_t RESERVED2;
	/* 0x12: Offset setting for tdp */
	volatile uint16_t ADCTDPO[3];
	volatile uint16_t RESERVED3[4];
	/* 0x20: DSADC Analog Control register 1 */
	volatile uint8_t ADCACTRL1;
	volatile uint8_t RESERVED4;
	/* 0x22: DSADC Analog Power Down Control */
	volatile uint16_t ADCACTRL2;
	volatile uint8_t RESERVED5[2];
	/* 0x26: Voltage / Thermister mode select */
	volatile uint8_t DSADCCTRL6;
	volatile uint8_t RESERVED6;
	/* 0x28: Voltage / Thermister mode select */
	volatile uint8_t DSADCCTRL7;
	volatile uint8_t RESERVED7[3];
	/* 0x2C: Voltage / Thermister mode select */
	volatile uint16_t DSADCCTRL8;
	volatile uint8_t RESERVED8[74];
	/* 0x78: DSADC Configuration */
	volatile uint16_t DSADCCFG;
	/* 0x7A: DSADC Channel select */
	volatile uint8_t DSADCCS;
	volatile uint8_t RESERVED9;
	/* 0x7C: DSADC global status */
	volatile uint16_t DSADCSTS;
	volatile uint16_t RESERVED10;
	/* 0x80: Temperature Channel Data */
	volatile uint16_t TCHDAT;
};

/* ADC register fields */
#define NPCM_CTRL0_VNT    5
#define NPCM_CTRL0_CH_SEL 0

#define NPCM_TM_T_MODE5 8
#define NPCM_TM_T_MODE4 6
#define NPCM_TM_T_MODE3 4
#define NPCM_TM_T_MODE2 2
#define NPCM_TM_T_MODE1 0

#define NPCM_TD_POST_OFFSET 8

#define NPCM_ACTRL1_PWCTRL      1
#define NPCM_ACTRL2_PD_VPP_PG   10
#define NPCM_ACTRL2_PD_PM       9
#define NPCM_ACTRL2_PD_ATX_5VSB 8
#define NPCM_ACTRL2_PD_ANA      7
#define NPCM_ACTRL2_PD_BG       6
#define NPCM_ACTRL2_PD_DSM      5
#define NPCM_ACTRL2_PD_DVBE     4
#define NPCM_ACTRL2_PD_IREF     3
#define NPCM_ACTRL2_PD_ISEN     2
#define NPCM_ACTRL2_PD_RG       1

#define NPCM_CFG_IOVFEN 6
#define NPCM_CFG_ICEN   5
#define NPCM_CFG_START  4

#define NPCM_CS_CC5 5
#define NPCM_CS_CC4 4
#define NPCM_CS_CC3 3
#define NPCM_CS_CC2 2
#define NPCM_CS_CC1 1
#define NPCM_CS_CC0 0

#define NPCM_STS_OVFEV 1
#define NPCM_STS_EOCEV 0

#define NPCM_TCHDAT_NEW  15
#define NPCM_TCHDAT_DAT  3
#define NPCM_TCHDAT_FRAC 0

#define NPCM_THRCTL_EN  15
#define NPCM_THRCTL_VAL 0
#define NPCM_THRDCTL_EN 15

#define NPCM_CTRL0_CH_SEL_MASK 0x1F
#define NPCM_TCHDAT_DAT_MASK   0x7FF

#define NPCM_ADC_REF_INTERNAL 2048

/* ADC local functions */
static inline void adc_npcm_clear_status(struct adc_reg *adc_regs)
{
	/* Clear OVFEV and EOCEV if bits are set */
	adc_regs->DSADCSTS = adc_regs->DSADCSTS;
}

static inline void adc_npcm_set_volt_large_2048mV(struct adc_reg *adc_regs)
{
	adc_regs->ADCACTRL1 &= ~BIT(NPCM_ACTRL1_PWCTRL);
}

static inline void adc_npcm_set_voltage_smaller_2048mV(struct adc_reg *adc_regs)
{
	adc_regs->ADCACTRL1 |= BIT(NPCM_ACTRL1_PWCTRL);
}

static inline void adc_npcm_channel_select_temp(struct adc_reg *adc_regs)
{
	adc_regs->DSADCCTRL0 &= ~BIT(NPCM_CTRL0_VNT);
}

static inline void adc_npcm_channel_select_volt(struct adc_reg *adc_regs)
{
	adc_regs->DSADCCTRL0 |= BIT(NPCM_CTRL0_VNT);
}

static inline void adc_npcm_channel_select(struct adc_reg *adc_regs,
					   const struct adc_npcm_channel_cfg *config)
{
	adc_regs->DSADCCTRL0 |= config->ctrl0_ch_sel_value & NPCM_CTRL0_CH_SEL_MASK;
}

static inline void adc_npcm_mode_select_volt(struct adc_reg *adc_regs,
					     const struct adc_npcm_channel_cfg *config)
{
	adc_regs->DSADCCTRL6 |= BIT(config->ctrl6_bit_offset);
}

static inline void adc_npcm_mode_select_temp(struct adc_reg *adc_regs,
					     const struct adc_npcm_channel_cfg *config)
{
	adc_regs->DSADCCTRL6 &= ~BIT(config->ctrl6_bit_offset);
}

static void adc_npcm_set_diode_channel(struct adc_reg *adc_regs,
				       const struct adc_npcm_channel_cfg *config)
{
	adc_npcm_set_voltage_smaller_2048mV(adc_regs);
	adc_npcm_channel_select_temp(adc_regs);
	adc_npcm_channel_select(adc_regs, config);
	adc_regs->ADCTM &= ~(0x03 << (NPCM_TM_T_MODE1 +
				      (config->ctrl0_ch_sel_value & NPCM_CTRL0_CH_SEL_MASK) * 2));
}

static void adc_npcm_set_voltage_channel(struct adc_reg *adc_regs,
					 const struct adc_npcm_channel_cfg *config)
{
	adc_npcm_channel_select_volt(adc_regs);
	adc_npcm_channel_select(adc_regs, config);

	if (config->is_vin_thr) {
		if (config->ctrl6_bit_set) {
			adc_npcm_mode_select_volt(adc_regs, config);
		} else {
			adc_npcm_mode_select_temp(adc_regs, config);
			adc_npcm_set_voltage_smaller_2048mV(adc_regs);
		}
	}
}

static void adc_npcm_configure_channel(struct adc_reg *adc_regs,
				       const struct adc_npcm_channel_cfg *config)
{
	if (config->is_diode) {
		adc_npcm_set_diode_channel(adc_regs, config);
	} else {
		adc_npcm_set_voltage_channel(adc_regs, config);
	}
}

static int adc_npcm_validate_buffer_size(const struct adc_npcm_config *config,
					 const struct adc_sequence *sequence)
{
	uint8_t channels = POPCOUNT(sequence->channels & BIT_MASK(config->channel_count));
	size_t needed = channels * sizeof(adc_npcm_buf_data_t);

	if (sequence->options) {
		needed *= (1 + sequence->options->extra_samplings);
	}

	return (sequence->buffer_size < needed) ? -ENOSPC : 0;
}

static adc_npcm_buf_data_t
adc_npcm_get_data(struct adc_reg *adc_regs, const struct adc_npcm_channel_cfg *config, uint16_t ref)
{
	adc_npcm_buf_data_t data = adc_regs->TCHDAT & NPCM_TCHDAT_DAT_MASK;

	if (config->is_diode || (config->is_vin_thr && !config->ctrl6_bit_set)) {
		/* 11-bit to 16-bit data */
		data <<= 5;
	} else {
		/* 1LSB = Vref/1024 */
		data *= ref / 1024;
	}

	return data;
}

static int adc_npcm_start_read(const struct adc_npcm_config *config, struct adc_npcm_data *data,
			       const struct adc_sequence *sequence)
{
	int error = 0;

	if (!sequence->channels || (sequence->channels & ~BIT_MASK(config->channel_count))) {
		LOG_ERR("Invalid ADC channels");
		return -EINVAL;
	}

	/* The resolution of the NPCM ADC is fixed at 10 bits */
	if (sequence->resolution != 10) {
		LOG_ERR("Unfixed 10 bit ADC resolution");
		return -ENOTSUP;
	}

	error = adc_npcm_validate_buffer_size(config, sequence);
	if (error) {
		LOG_ERR("ADC buffer size too small");
		return error;
	}

	/* Save ADC sequence sampling buffer and its end pointer address */
	data->buffer = sequence->buffer;
	data->buf_end = data->buffer + sequence->buffer_size / sizeof(adc_npcm_buf_data_t);

	/* Start ADC conversion */
	adc_context_start_read(&data->ctx, sequence);
	error = adc_context_wait_for_completion(&data->ctx);

	return error;
}

static void adc_npcm_start_convert(const struct device *dev)
{
	struct adc_reg *const adc_regs = HAL_BASE_CFG_INST(dev);
	struct adc_npcm_data *const data = dev->data;
	const struct adc_npcm_channel_cfg *npcm_channel_configs = HAL_CFG_CHANNEL_CFG_INST(dev);

	/* Set voltage large than 2.048V */
	adc_npcm_set_volt_large_2048mV(adc_regs);

	/* Start ADC scan conversion */
	adc_regs->DSADCCTRL0 &= ~NPCM_CTRL0_CH_SEL_MASK; /* Clear CH_SEL */

	/* Set channel */
	adc_npcm_configure_channel(adc_regs, &npcm_channel_configs[data->CurChannel_Idx]);

	/* Clear sts */
	adc_npcm_clear_status(adc_regs);

	/* Enable interrupt(ADC_ICEN) and start */
	adc_regs->DSADCCFG |= BIT(NPCM_CFG_ICEN) | BIT(NPCM_CFG_START);
}

/* ADC generic functions */
static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_npcm_data *const data = CONTAINER_OF(ctx, struct adc_npcm_data, ctx);

	data->repeat_buffer = data->buffer;
	data->channels = ctx->sequence.channels;
	data->CurChannel_Idx = find_lsb_set(data->channels) - 1; /* Next channel to sample */

	LOG_DBG("[%s] data->CurChannel_Idx=%d\n", __func__, data->CurChannel_Idx);

	if (!data->channels) {
		LOG_ERR("No ADC channel can start sampling!!");
	} else {
		adc_npcm_start_convert(data->adc_dev);
	}
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx, bool repeat_sampling)
{
	struct adc_npcm_data *const data = CONTAINER_OF(ctx, struct adc_npcm_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

/* ADC API functions */
static int adc_npcm_channel_setup(const struct device *dev,
				  const struct adc_channel_cfg *channel_cfg)
{
	const struct adc_npcm_config *config = dev->config;
	uint8_t channel_id = channel_cfg->channel_id;

	if (channel_id >= config->channel_count) {
		LOG_ERR("Invalid channel %d", channel_id);
		return -EINVAL;
	}

	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		LOG_ERR("Unsupported channel acquisition time");
		return -ENOTSUP;
	}

	if (channel_cfg->differential) {
		LOG_ERR("Differential channels are not supported");
		return -ENOTSUP;
	}

	if (channel_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("Unsupported channel gain %d", channel_cfg->gain);
		return -ENOTSUP;
	}

	if (channel_cfg->reference != ADC_REF_INTERNAL) {
		LOG_ERR("Unsupported channel reference");
		return -ENOTSUP;
	}

	return 0;
}

static int adc_npcm_read(const struct device *dev, const struct adc_sequence *sequence)
{
	const struct adc_npcm_config *config = dev->config;
	struct adc_npcm_data *const data = dev->data;
	int error;

	adc_context_lock(&data->ctx, false, NULL);
	error = adc_npcm_start_read(config, data, sequence);
	adc_context_release(&data->ctx, error);

	return error;
}

#if defined(CONFIG_ADC_ASYNC)
static int adc_npcm_read_async(const struct device *dev, const struct adc_sequence *sequence,
			       struct k_poll_signal *async)
{
	struct adc_npcm_data *const data = dev->data;
	int error;

	adc_context_lock(&data->ctx, true, async);
	error = adc_npcm_start_read(dev, sequence);
	adc_context_release(&data->ctx, error);

	return error;
}
#endif /* CONFIG_ADC_ASYNC */

/* ADC initialization function */
static int adc_npcm_init(const struct device *dev)
{
	const struct adc_npcm_config *const config = dev->config;
	struct adc_npcm_data *const data = ((struct adc_npcm_data *)(dev)->data);
	int ret;

	/* Save ADC device in data */
	data->adc_dev = dev;

	/* Configure ADC interrupt and enable it */
	config->irq_cfg_func();

	/* Initialize mutex of ADC channels */
	adc_context_unlock_unconditionally(&data->ctx);

	/* Configure pin-mux for ADC device */
	ret = pinctrl_apply_state(config->pin_dev_cfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("ADC pinctrl setup failed (%d)", ret);
		return ret;
	}

	/* Apply internal reference */
	data->ref_internal = NPCM_ADC_REF_INTERNAL;

	return 0;
}

/* ADC driver APIs */
static DEVICE_API(adc, adc_npcm_driver_api) = {
	.channel_setup = adc_npcm_channel_setup,
	.read = adc_npcm_read,
#if defined(CONFIG_ADC_ASYNC)
	.read_async = adc_npcm_read_async,
#endif /* CONFIG_ADC_ASYNC */
	.ref_internal = NPCM_ADC_REF_INTERNAL,
};

/* ADC interrupt configuration implementation */
static void adc_npcm_isr(const struct device *dev)
{
	struct adc_reg *const adc_regs = HAL_BASE_CFG_INST(dev);
	struct adc_npcm_data *const data = dev->data;
	const struct adc_npcm_channel_cfg *npcm_channel_configs = HAL_CFG_CHANNEL_CFG_INST(dev);
	adc_npcm_buf_data_t result;

	/* Clear sts */
	adc_npcm_clear_status(adc_regs);

	/* Get reading */
	result = adc_npcm_get_data(adc_regs, &npcm_channel_configs[data->CurChannel_Idx],
				   data->ref_internal);

	/* Store reading */
	if (data->buffer < data->buf_end) {
		*data->buffer++ = result;
	}
	data->channels &= ~BIT(data->CurChannel_Idx);

	/* Check if there are more channels to sample */
	if (data->channels) {
		/* Next channel to sample */
		data->CurChannel_Idx = find_lsb_set(data->channels) - 1;
		adc_npcm_start_convert(dev);
	} else {
		/* Inform sampling is done */
		adc_context_on_sampling_done(&data->ctx, dev);
	}
}

#define NPCM_ADC_IRQ_CFG_FUNC_IMPL(inst)                                                           \
	static void adc_npcm_irq_cfg_##inst(void)                                                  \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), adc_npcm_isr,         \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}

/* ADC channel configuration implementation */
#define GET_CHANNEL(cfg)   ((cfg >> NPCM_ADC_CHANNEL_SEL_OFFSET) & NPCM_ADC_CHANNEL_SEL_MASK)
#define GET_CTRL6_BIT(cfg) ((cfg >> NPCM_ADC_CTRL6_BIT_OFFSET) & NPCM_ADC_CTRL6_BIT_MASK)
#define GET_CTRL6_BIT_SET(cfg)                                                                     \
	((cfg >> NPCM_ADC_CTRL6_BIT_SET_OFFSET) & NPCM_ADC_CTRL6_BIT_SET_MASK)
#define GET_IS_DIODE(cfg)   ((cfg >> NPCM_ADC_IS_DIODE_OFFSET) & NPCM_ADC_IS_DIODE_MASK)
#define GET_IS_VIN_THR(cfg) ((cfg >> NPCM_ADC_IS_VIN_THR_OFFSET) & NPCM_ADC_IS_VIN_THR_MASK)
#define GET_ID(cfg)         ((cfg >> NPCM_ADC_ID_OFFSET) & NPCM_ADC_ID_MASK)
#define NPCM_ADC_CFG_IMPL(node_id, prop, idx)                                                      \
	[GET_ID(DT_PROP_BY_IDX(node_id, prop, idx))] = {                                           \
		.ctrl0_ch_sel_value = GET_CHANNEL(DT_PROP_BY_IDX(node_id, prop, idx)),             \
		.ctrl6_bit_offset = GET_CTRL6_BIT(DT_PROP_BY_IDX(node_id, prop, idx)),             \
		.ctrl6_bit_set = GET_CTRL6_BIT_SET(DT_PROP_BY_IDX(node_id, prop, idx)),            \
		.is_diode = GET_IS_DIODE(DT_PROP_BY_IDX(node_id, prop, idx)),                      \
		.is_vin_thr = GET_IS_VIN_THR(DT_PROP_BY_IDX(node_id, prop, idx)),                  \
	}

/* ADC driver registration */
#define NPCM_ADC_INIT(inst)                                                                        \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
                                                                                                   \
	NPCM_ADC_IRQ_CFG_FUNC_IMPL(inst)                                                           \
                                                                                                   \
	static const struct adc_npcm_channel_cfg npcm_channel_configs_##inst[] = {                 \
		DT_INST_FOREACH_PROP_ELEM_SEP(inst, nuvoton_channel_cfg, NPCM_ADC_CFG_IMPL, (,)), \
	};                                                                                         \
                                                                                                   \
	static const struct adc_npcm_config adc_npcm_cfg_##inst = {                                \
		.base = DT_INST_REG_ADDR(inst),                                                    \
		.channel_count = DT_INST_PROP(inst, nuvoton_channel_count),                        \
		.pin_dev_cfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                               \
		.irq_cfg_func = adc_npcm_irq_cfg_##inst,                                           \
		.channel_cfg = npcm_channel_configs_##inst,                                        \
	};                                                                                         \
	BUILD_ASSERT(DT_INST_PROP(inst, nuvoton_channel_count) ==                                  \
			     ARRAY_SIZE(npcm_channel_configs_##inst),                              \
		     "prop. nuvoton,channel-count must be equal to channel config number");        \
                                                                                                   \
	static struct adc_npcm_data adc_npcm_data_##inst = {                                       \
		ADC_CONTEXT_INIT_TIMER(adc_npcm_data_##inst, ctx),                                 \
		ADC_CONTEXT_INIT_LOCK(adc_npcm_data_##inst, ctx),                                  \
		ADC_CONTEXT_INIT_SYNC(adc_npcm_data_##inst, ctx),                                  \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, adc_npcm_init, NULL, &adc_npcm_data_##inst,                    \
			      &adc_npcm_cfg_##inst, PRE_KERNEL_1, CONFIG_ADC_INIT_PRIORITY,        \
			      &adc_npcm_driver_api);

DT_INST_FOREACH_STATUS_OKAY(NPCM_ADC_INIT)
