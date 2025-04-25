/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcm_i2c

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/i2c/i2c.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2c_npcm, CONFIG_I2C_LOG_LEVEL);

/*
 * i2c device registers
 */
struct i2c_reg {
	/* 0x00: I2C Serial Data */
	volatile uint8_t SMBnSDA;
	volatile uint8_t RESERVE0[1];
	/* 0x02: I2C Status */
	volatile uint8_t SMBnST;
	volatile uint8_t RESERVE1[1];
	/* 0x04: I2C Control Status */
	volatile uint8_t SMBnCST;
	volatile uint8_t RESERVE2[1];
	/* 0x06: I2C Control 1 */
	volatile uint8_t SMBnCTL1;
	volatile uint8_t RESERVE3[1];
	/* 0x08: I2C Own Address 1 */
	volatile uint8_t SMBnADDR1;
	/* 0x09: Timeout Status */
	volatile uint8_t TIMEOUT_ST;
	/* 0x0A: I2C Control 2 */
	volatile uint8_t SMBnCTL2;
	/* 0x0B: Timeout Enable */
	volatile uint8_t TIMEOUT_EN;
	/* 0x0C: I2C Own Address 2 */
	volatile uint8_t SMBnADDR2;
	volatile uint8_t RESERVE4[1];
	/* 0x0E: I2C Control 3 */
	volatile uint8_t SMBnCTL3;
	/* 0x0F: DMA Control */
	volatile uint8_t DMA_CTRL;
	/* 0x10: I2C Own Address 3 */
	volatile uint8_t SMBnADDR3;
	/* 0x11: I2C Own Address 7 */
	volatile uint8_t SMBnADDR7;
	/* 0x12: I2C Own Address 4 */
	volatile uint8_t SMBnADDR4;
	/* 0x13: I2C Own Address 8 */
	volatile uint8_t SMBnADDR8;
	/* 0x14: I2C Own Address 5 */
	volatile uint8_t SMBnADDR5;
	/* 0x15: I2C Own Address 9 */
	volatile uint8_t SMBnADDR9;
	/* 0x16: I2C Own Address 6 */
	volatile uint8_t SMBnADDR6;
	/* 0x17: I2C Own Address 10 */
	volatile uint8_t SMBnADDR10;
	/* 0x18: I2C Control Status 2 */
	volatile uint8_t SMBnCST2;
	/* 0x19: I2C Control Status 3 */
	volatile uint8_t SMBnCST3;
	/* 0x1A: I2C Control 4 */
	volatile uint8_t SMBnCTL4;
	/* 0x1B: I2C Control 5 */
	volatile uint8_t SMBnCTL5;
	/* 0x1C: I2C SCL Low Time (Fast Mode) */
	volatile uint8_t SMBnSCLLT;
	/* 0x1D: I2C Address Match Status */
	volatile uint8_t ADDMTCH_ST;
	/* 0x1E: I2C SCL High Time (Fast Mode) */
	volatile uint8_t SMBnSCLHT;
	/* 0x1F: I2C Version */
	volatile uint8_t SMBn_VER;
	/* 0x20: DMA Address Byte 1 */
	volatile uint8_t DMA_ADDR1;
	/* 0x21: DMA Address Byte 2 */
	volatile uint8_t DMA_ADDR2;
	/* 0x22: DMA Address Byte 3 */
	volatile uint8_t DMA_ADDR3;
	/* 0x23: DMA Address Byte 4 */
	volatile uint8_t DMA_ADDR4;
	/* 0x24: Data Length Byte 1 */
	volatile uint8_t DATA_LEN1;
	/* 0x25: Data Length Byte 2 */
	volatile uint8_t DATA_LEN2;
	/* 0x26: Data Counter Byte 1 */
	volatile uint8_t DATA_CNT1;
	/* 0x27: Data Counter Byte 2 */
	volatile uint8_t DATA_CNT2;
	volatile uint8_t RESERVE7[1];
	/* 0x29: Timeout Control 1 */
	volatile uint8_t TIMEOUT_CTL1;
	/* 0x2A: Timeout Control 2 */
	volatile uint8_t TIMEOUT_CTL2;
	/* 0x2B: I2C PEC Data */
	volatile uint8_t SMBnPEC;
};

/**
 * I2C register fields
 */
/* SMBnST fields */
#define NPCM_SMBnST_XMIT   0
#define NPCM_SMBnST_MASTER 1
#define NPCM_SMBnST_NMATCH 2
#define NPCM_SMBnST_STASTR 3
#define NPCM_SMBnST_NEGACK 4
#define NPCM_SMBnST_BER    5
#define NPCM_SMBnST_SDAST  6
#define NPCM_SMBnST_SLVSTP 7

/* SMBnCST fields */
#define NPCM_SMBnCST_BUSY     0
#define NPCM_SMBnCST_BB       1
#define NPCM_SMBnCST_MATCH    2
#define NPCM_SMBnCST_GCMATCH  3
#define NPCM_SMBnCST_TSDA     4
#define NPCM_SMBnCST_TGSCL    5
#define NPCM_SMBnCST_MATCHAF  6
#define NPCM_SMBnCST_ARPMATCH 7

/* SMBnCTL1 fields */
#define NPCM_SMBnCTL1_START   0
#define NPCM_SMBnCTL1_STOP    1
#define NPCM_SMBnCTL1_INTEN   2
#define NPCM_SMBnCTL1_EOBINTE 3
#define NPCM_SMBnCTL1_ACK     4
#define NPCM_SMBnCTL1_GCMEN   5
#define NPCM_SMBnCTL1_NMINTE  6
#define NPCM_SMBnCTL1_STASTRE 7

/* SMBnADDR1-10 fields */
#define NPCM_SMBnADDR_ADDR 0
#define NPCM_SMBnADDR_SAEN 7

/* TIMEOUT_ST fields */
#define NPCM_TIMEOUT_ST_T_OUTST1    0
#define NPCM_TIMEOUT_ST_T_OUTST2    1
#define NPCM_TIMEOUT_ST_T_OUTST1_EN 6
#define NPCM_TIMEOUT_ST_T_OUTST2_EN 7

/* SMBnCTL2 fields */
#define NPCM_SMBnCTL2_ENABLE       0
#define NPCM_SMBnCTL2_SCLFRQ60     1
#define NPCM_SMBnCTL2_SCLFRQ60_END 7

/* TIMEOUT_EN fields */
#define NPCM_TIMEOUT_EN_TIMEOUT_EN   0
#define NPCM_TIMEOUT_EN_TO_CKDIV     2
#define NPCM_TIMEOUT_EN_TO_CKDIV_END 7
#define NPCM_TIMEOUT_EN_TO_CKDIV_MIN 0x3
#define NPCM_TIMEOUT_EN_TO_CKDIV_MAX 0x3F

/* SMBnCTL3 fields */
#define NPCM_SMBnCTL3_SCLFRQ87     0
#define NPCM_SMBnCTL3_SCLFRQ87_END 1
#define NPCM_SMBnCTL3_ARPMEN       2
#define NPCM_SMBnCTL3_SLP_START    3
#define NPCM_SMBnCTL3_400K_MODE    4
#define NPCM_SMBnCTL3_SDA_LVL      6
#define NPCM_SMBnCTL3_SCL_LVL      7

/* DMA_CTRL fields */
#define NPCM_DMA_CTRL_DMA_INT_CLR 0
#define NPCM_DMA_CTRL_DMA_EN      1
#define NPCM_DMA_CTRL_LAST_PEC    2
#define NPCM_DMA_CTRL_DMA_STALL   3
#define NPCM_DMA_CTRL_DMA_IRQ     7

/* SMBnCST2 fields */
#define NPCM_SMBnCST2_MATCHA1F 0
#define NPCM_SMBnCST2_MATCHA2F 1
#define NPCM_SMBnCST2_MATCHA3F 2
#define NPCM_SMBnCST2_MATCHA4F 3
#define NPCM_SMBnCST2_MATCHA5F 4
#define NPCM_SMBnCST2_MATCHA6F 5
#define NPCM_SMBnCST2_MATCHA7F 6
#define NPCM_SMBnCST2_INTSTS   7

/* SMBnCST3 fields */
#define NPCM_SMBnCST3_MATCHA8F  0
#define NPCM_SMBnCST3_MATCHA9F  1
#define NPCM_SMBnCST3_MATCHA10F 2
#define NPCM_SMBnCST3_EO_BUSY   7

/**
 * Configure TX and RX buffer size for I2C DMA
 * This setting applies to all (12c1a, 12c1b, 12c2a ...)
 */
#define CONFIG_I2C_MAX_TX_SIZE 256
#define CONFIG_I2C_MAX_RX_SIZE 256

/* Default maximum time we allow for an I2C transfer (unit:ms) */
#define I2C_TRANS_TIMEOUT K_MSEC(500)

/* Default max waiting time for i2c ready (unit:ms) */
#define I2C_WAITING_TIME K_MSEC(1000)

/* Clock control device */
#define NPCM_CLK_CTRL_DEV DT_NODELABEL(pcc)

/* I2C controller configuration */
#define MIN_STANDARD_MODE 8
#define MAX_STANDARD_MODE 511
#define MIN_FAST_MODE     5
#define MAX_FAST_MODE     255
#define HLDT_48MHZ        17
#define HLDT_20MHZ        9
#define HLDT_DEFAULT      7
#define ADDR_7BIT_MASK    GENMASK(7, 1)
#define ADDR_WRITE_MASK   BIT(0)
#define SCL_H_TIME_SHIFT  3
#define SCL_L_TIME_SHIFT  1

/* Data abort timeout */
#define ABORT_TIMEOUT 10000

/* I2C operation state */
enum i2c_npcm_oper_state {
	I2C_NPCM_OPER_STA_IDLE,
	I2C_NPCM_OPER_STA_START,
	I2C_NPCM_OPER_STA_WRITE,
	I2C_NPCM_OPER_STA_READ,
	I2C_NPCM_OPER_STA_QUICK,
};

/* Device configuration */
struct i2c_npcm_config {
	uintptr_t base;                        /* The i2c controller base address */
	uint32_t clk_cfg;                      /* The clock configuration */
	uint32_t default_bitrate;              /* The default bitrate */
	uint8_t irq;                           /* The i2c controller irq */
	const struct pinctrl_dev_config *pcfg; /* The pinmux configuration */
};

struct i2c_npcm_data {
	struct k_sem lock_sem; /* The mutex of i2c controller */
	struct k_sem sync_sem; /* The semaphore used for synchronization */
	enum i2c_npcm_oper_state ctrl_oper_state;
	enum i2c_npcm_oper_state target_oper_state;
	uint32_t bitrate;
	uint32_t source_clk;
	uint16_t rx_cnt;
	uint16_t tx_cnt;
	uint8_t dev_addr;                                    /* The device address (8 bits) */
	uint8_t rx_buf[CONFIG_I2C_MAX_TX_SIZE] __aligned(4); /* Must be 4-byte aligned for DMA */
	uint8_t tx_buf[CONFIG_I2C_MAX_RX_SIZE] __aligned(4); /* Must be 4-byte aligned for DMA */
	uint8_t *rx_msg_buf;
	int err_code;
	struct i2c_target_config *target_cfg;
};

/* Driver convenience defines */
#define NPCM_SET_REG(reg, offset, value)                                                           \
	(reg) = ((reg) & ~(GENMASK(offset##_END, offset))) |                                       \
		(((value) << (offset)) & (GENMASK(offset##_END, offset)))
#define I2C_BASE_INST(dev) ((struct i2c_reg *)((const struct i2c_npcm_config *)(dev)->config)->base)

/* I2C local inline functions */
/**
 * This macro should be set only when in Controller mode or when requesting Controller mode.
 * Set START bit to CTL1 register of I2C module, but exclude STOP bit, ACK bit.
 */
#define I2C_SMBnCTL1_MASK                                                                          \
	(BIT(NPCM_SMBnCTL1_START) | BIT(NPCM_SMBnCTL1_STOP) | BIT(NPCM_SMBnCTL1_ACK))
static inline void i2c_start(struct i2c_reg *inst)
{
	inst->SMBnCTL1 = (inst->SMBnCTL1 & ~I2C_SMBnCTL1_MASK) | BIT(NPCM_SMBnCTL1_START);
}

static inline void i2c_stop(struct i2c_reg *inst)
{
	inst->SMBnCTL1 = ((inst)->SMBnCTL1 & ~I2C_SMBnCTL1_MASK) | BIT(NPCM_SMBnCTL1_STOP);
}

static inline void i2c_enable_stall(struct i2c_reg *inst)
{
	inst->SMBnCTL1 = (inst->SMBnCTL1 & ~I2C_SMBnCTL1_MASK) | BIT(NPCM_SMBnCTL1_STASTRE);
}

static inline void i2c_disable_stall(struct i2c_reg *inst)
{
	inst->SMBnCTL1 = inst->SMBnCTL1 & ~(I2C_SMBnCTL1_MASK | BIT(NPCM_SMBnCTL1_STASTRE));
}

static inline void i2c_enable(struct i2c_reg *inst)
{
	inst->SMBnCTL2 |= BIT(NPCM_SMBnCTL2_ENABLE);
}

static inline void i2c_disable(struct i2c_reg *inst)
{
	inst->SMBnCTL2 &= ~BIT(NPCM_SMBnCTL2_ENABLE);
}

static inline void i2c_interrupt_enable(struct i2c_reg *inst)
{
	inst->SMBnCTL1 |= BIT(NPCM_SMBnCTL1_INTEN);
}

/* I2C mode functions */
static inline void i2c_set_standard_mode_frequency(struct i2c_reg *inst, uint32_t reg_tmp)
{
	/* Set SCL Frequency[8:0] to SMBnCTL3[8:7] and SMBnCTL2[6:0] */
	NPCM_SET_REG(inst->SMBnCTL2, NPCM_SMBnCTL2_SCLFRQ60, reg_tmp & 0x7f);
	NPCM_SET_REG(inst->SMBnCTL3, NPCM_SMBnCTL3_SCLFRQ87, reg_tmp >> 7);
}

static inline void i2c_set_fast_mode_frequency(struct i2c_reg *inst, uint32_t reg_tmp)
{
	i2c_set_standard_mode_frequency(inst, 0);

	/* Set SCL High Time and Low Time */
	inst->SMBnSCLHT = reg_tmp - SCL_H_TIME_SHIFT;
	inst->SMBnSCLLT = reg_tmp - SCL_L_TIME_SHIFT;
}

/* I2C general functions */
static void i2c_reset_module(struct i2c_reg *inst, struct i2c_npcm_data *data)
{
	uint32_t ctl1_tmp;
	uint32_t timeout_en_tmp;

	ctl1_tmp = inst->SMBnCTL1;
	timeout_en_tmp = inst->TIMEOUT_EN;

	/* Disable and then Enable I2C module */
	i2c_disable(inst);
	i2c_enable(inst);

	inst->SMBnCTL1 = (ctl1_tmp & (BIT(NPCM_SMBnCTL1_INTEN) | BIT(NPCM_SMBnCTL1_EOBINTE) |
				      BIT(NPCM_SMBnCTL1_GCMEN) | BIT(NPCM_SMBnCTL1_NMINTE)));
	inst->TIMEOUT_EN = timeout_en_tmp;

	data->ctrl_oper_state = I2C_NPCM_OPER_STA_IDLE;
}

static void i2c_abort_data(struct i2c_reg *inst)
{
	uint16_t timeout = ABORT_TIMEOUT;

	/* Generate a STOP condition */
	i2c_stop(inst);

	/* Clear NEGACK, STASTR and BER bits */
	inst->SMBnST = (BIT(NPCM_SMBnST_STASTR) | BIT(NPCM_SMBnST_NEGACK) | BIT(NPCM_SMBnST_BER));

	/* Wait till STOP condition is generated */
	while (--timeout) {
		if ((inst->SMBnCTL1 & BIT(NPCM_SMBnCTL1_STOP)) == 0x00) {
			break;
		}
	}

	/* Clear BB (BUS BUSY) bit */
	inst->SMBnCST = BIT(NPCM_SMBnCST_BB);
}

static void i2c_notify(struct i2c_reg *inst, struct i2c_npcm_data *data, int error)
{
#if CONFIG_I2C_NPCM_CONTROLLER_HW_TIMEOUT_EN
	/* Disable HW Timeout */
	inst->TIMEOUT_EN &= ~BIT(NPCM_TIMEOUT_EN_TIMEOUT_EN);
#else
	ARG_UNUSED(inst);
#endif

	data->ctrl_oper_state = I2C_NPCM_OPER_STA_IDLE;
	data->err_code = error;

	k_sem_give(&data->sync_sem);
}

static int i2c_wait_completion(struct i2c_reg *inst, struct i2c_npcm_data *data)
{
	if (k_sem_take(&data->sync_sem, I2C_WAITING_TIME) != 0) {
		i2c_reset_module(inst, data);
		data->err_code = -ETIMEDOUT;
	}

	return data->err_code;
}

static void i2c_set_hold_time(struct i2c_reg *inst, uint32_t clk)
{
	/* Set HLDT (48MHz, HLDT = 17, Hold Time = 360ns) */
	if (clk >= MHZ(40)) {
		inst->SMBnCTL4 = HLDT_48MHZ;
	} else if (clk >= MHZ(20)) {
		inst->SMBnCTL4 = HLDT_20MHZ;
	} else {
		inst->SMBnCTL4 = HLDT_DEFAULT;
	}
}

static void i2c_set_baudrate(const struct device *dev, uint32_t bus_freq)
{
	uint32_t reg_tmp;
	const struct i2c_npcm_config *const config = dev->config;
	struct i2c_reg *const inst = I2C_BASE_INST(dev);
	const struct device *const clk_dev = DEVICE_DT_GET(NPCM_CLK_CTRL_DEV);
	struct i2c_npcm_data *const data = dev->data;

	if (clock_control_get_rate(clk_dev, (clock_control_subsys_t *)config->clk_cfg,
				   &data->source_clk) != 0) {
		LOG_ERR("Get %s clock source.", dev->name);
	}

	LOG_DBG("i2c clock source: %d", data->source_clk);
	LOG_DBG("bitrate: %d", bus_freq);

	reg_tmp = data->source_clk / (bus_freq * 4); /* SCLFRQ = tSCL / (4 * tBCLK) */

	if (bus_freq < KHZ(400)) {
		reg_tmp = CLAMP(reg_tmp, MIN_STANDARD_MODE, MAX_STANDARD_MODE);
		/* Disable fast mode and fast mode plus */
		inst->SMBnCTL3 &= ~BIT(NPCM_SMBnCTL3_400K_MODE);
		i2c_set_standard_mode_frequency(inst, reg_tmp);
	} else {
		reg_tmp = CLAMP(reg_tmp, MIN_FAST_MODE, MAX_FAST_MODE);
		/* Enable fast mode and fast mode plus */
		inst->SMBnCTL3 |= BIT(NPCM_SMBnCTL3_400K_MODE);
		i2c_set_fast_mode_frequency(inst, reg_tmp);
	}

	i2c_set_hold_time(inst, data->source_clk);
}

/* I2C DMA functions */
static uint16_t i2c_DMA_get_cnt(struct i2c_reg *inst)
{
	return (uint16_t)(((uint16_t)(inst->DATA_CNT1) << 8) + (uint16_t)(inst->DATA_CNT2));
}

static void i2c_DMA_start(struct i2c_reg *inst, uint32_t addr, uint16_t len)
{
	/* Set DMA address and length */
	inst->DMA_ADDR1 = (uint8_t)((addr) >> 0);
	inst->DMA_ADDR2 = (uint8_t)((addr) >> 8);
	inst->DMA_ADDR3 = (uint8_t)((addr) >> 16);
	inst->DMA_ADDR4 = (uint8_t)((addr) >> 24);
	inst->DATA_LEN1 = (uint8_t)((len) >> 0);
	inst->DATA_LEN2 = (uint8_t)((len) >> 8);

	/* Clear DMA interrupt */
	inst->DMA_CTRL = BIT(NPCM_DMA_CTRL_DMA_INT_CLR);

	/* Enable DMA */
	inst->DMA_CTRL = BIT(NPCM_DMA_CTRL_DMA_EN);
}

#if CONFIG_I2C_NPCM_CONTROLLER_HW_TIMEOUT_EN
/* I2C timeout functions */
void i2c_timeout_cumulative_clockcycle_set(struct i2c_npcm_data *data, struct i2c_reg *inst,
					   uint8_t interval_ms)
{
	uint8_t div = (data->source_clk / 1000 / 1000) - 1; /* Divider = set value + 1, unit: MHz */

	/* Check div is valid */
	if ((div < NPCM_TIMEOUT_EN_TO_CKDIV_MIN) || (div > NPCM_TIMEOUT_EN_TO_CKDIV_MAX)) {
		return;
	}
	i2c_enable(inst);
	NPCM_SET_REG(inst->TIMEOUT_EN, NPCM_TIMEOUT_EN_TO_CKDIV, div);
	inst->TIMEOUT_ST |= BIT(NPCM_TIMEOUT_ST_T_OUTST2_EN);
	inst->TIMEOUT_ST = BIT(NPCM_TIMEOUT_ST_T_OUTST2);
	inst->TIMEOUT_CTL1 = interval_ms;
}

void i2c_timeout_cumulative_clocklow_set(struct i2c_npcm_data *data, struct i2c_reg *inst,
					 uint8_t interval_ms)
{
	uint8_t div = (data->source_clk / 1000 / 1000) - 1; /* Divider = set value + 1, unit: MHz */

	/* Check div is valid */
	if ((div < NPCM_TIMEOUT_EN_TO_CKDIV_MIN) || (div > NPCM_TIMEOUT_EN_TO_CKDIV_MAX)) {
		return;
	}
	i2c_enable(inst);
	NPCM_SET_REG(inst->TIMEOUT_EN, NPCM_TIMEOUT_EN_TO_CKDIV, div);
	inst->TIMEOUT_ST |= BIT(NPCM_TIMEOUT_ST_T_OUTST1_EN);
	inst->TIMEOUT_ST = BIT(NPCM_TIMEOUT_ST_T_OUTST1);
	inst->TIMEOUT_CTL2 = interval_ms;
}
#endif

/* I2C controller functions */
static void i2c_ctrl_handle_SDA_STATUS(struct i2c_reg *inst, struct i2c_npcm_data *data)
{
	switch (data->ctrl_oper_state) {
	case I2C_NPCM_OPER_STA_START:
		if (data->tx_cnt == 0 && data->rx_cnt == 0) {
			/* Quick command (SMBUS protocol) */
			data->ctrl_oper_state = I2C_NPCM_OPER_STA_QUICK;
			i2c_enable_stall(inst);

			/* Quick read or quick write is determined by address */
			inst->SMBnSDA = data->dev_addr;
		} else if (data->tx_cnt == 0 && data->rx_cnt > 0) {
			/* Receive mode */
			data->ctrl_oper_state = I2C_NPCM_OPER_STA_READ;
			i2c_enable_stall(inst);

			/* Send read address */
			inst->SMBnSDA = data->dev_addr | ADDR_WRITE_MASK;
		} else if (data->tx_cnt > 0) {
			/* Transmit mode */
			data->ctrl_oper_state = I2C_NPCM_OPER_STA_WRITE;

			/* Send write address */
			inst->SMBnSDA = data->dev_addr & ADDR_7BIT_MASK;
		}
		break;

	case I2C_NPCM_OPER_STA_WRITE:
		/* Set DMA register to send data */
		i2c_DMA_start(inst, (uint32_t)data->tx_buf, data->tx_cnt);
		break;

	default:
		/* Error */
		break;
	}
}

static void i2c_ctrl_handle_DMA_IRQ(struct i2c_reg *inst, struct i2c_npcm_data *data)
{
	if (data->ctrl_oper_state == I2C_NPCM_OPER_STA_WRITE) {
		if (data->rx_cnt == 0) {
			/* No need to receive data */
			i2c_stop(inst);
			i2c_notify(inst, data, 0);
		} else {
			data->ctrl_oper_state = I2C_NPCM_OPER_STA_READ;
			i2c_enable_stall(inst);
			i2c_start(inst);
			inst->SMBnSDA = (data->dev_addr | ADDR_WRITE_MASK);
		}
	} else {
		i2c_stop(inst);
		data->rx_cnt = i2c_DMA_get_cnt(inst);
		i2c_notify(inst, data, 0);
	}

	/* Clear DMA interrupt */
	inst->DMA_CTRL = BIT(NPCM_DMA_CTRL_DMA_INT_CLR);
}

static void i2c_ctrl_handle_STALL(struct i2c_reg *inst, struct i2c_npcm_data *data)
{
	if (data->ctrl_oper_state == I2C_NPCM_OPER_STA_READ) {
		/* Set DMA register to read data */
		/* Return Negative Acknowledge when DMA received last byte. */
		inst->DMA_CTRL = (inst->DMA_CTRL & ~BIT(NPCM_DMA_CTRL_DMA_INT_CLR)) |
				 BIT(NPCM_DMA_CTRL_LAST_PEC);
		i2c_DMA_start(inst, (uint32_t)data->rx_buf, data->rx_cnt);
	} else if (data->ctrl_oper_state == I2C_NPCM_OPER_STA_QUICK) {
		/* Quick command */
		i2c_stop(inst);
		i2c_notify(inst, data, 0);
	} else {
		/* Error */
		LOG_ERR("controller: stall error");
	}

	i2c_disable_stall(inst);

	/* Clear STASTR flag */
	inst->SMBnST = BIT(NPCM_SMBnST_STASTR);
}

/* I2C target functions */
static void i2c_target_read(struct i2c_reg *inst, struct i2c_npcm_data *data,
			    const struct i2c_target_callbacks *target_cb)
{
	uint16_t len = 0;
	uint16_t dma_cnt = i2c_DMA_get_cnt(inst);

	while (len < dma_cnt) {
		target_cb->write_received(data->target_cfg, data->rx_buf[len]);
		len++;
	}
}

static void i2c_target_handle_NMATCH(struct i2c_reg *inst, struct i2c_npcm_data *data)
{
	const struct i2c_target_callbacks *target_cb = data->target_cfg->callbacks;
	uint16_t len;

	if (inst->SMBnST & BIT(NPCM_SMBnST_XMIT)) {
		/* Target received Read-Address */
		if (data->target_oper_state != I2C_NPCM_OPER_STA_START) {
			/* Target received data before */
			i2c_target_read(inst, data, target_cb);
		}

		/* Prepare tx data */
		if (target_cb->read_requested(data->target_cfg, data->tx_buf) == 0) {
			len = 0;
			while (++len < sizeof(data->tx_buf)) {
				if (target_cb->read_processed(data->target_cfg,
							      data->tx_buf + len) != 0) {
					break;
				}
			}
			data->tx_cnt = len;
		} else {
			/* Target has no data to send */
			data->tx_cnt = 0;
		}

		data->target_oper_state = I2C_NPCM_OPER_STA_WRITE;

		if (data->tx_cnt != 0) {
			/* Set DMA register to send data */
			i2c_DMA_start(inst, (uint32_t)data->tx_buf, data->tx_cnt);
		} else {
			data->target_oper_state = I2C_NPCM_OPER_STA_QUICK;
		}
	} else {
		/* Target received Write-Address */
		data->target_oper_state = I2C_NPCM_OPER_STA_READ;

		/* Set DMA register to get data */
		i2c_DMA_start(inst, (uint32_t)data->rx_buf, sizeof(data->rx_buf));
		target_cb->write_requested(data->target_cfg);
	}

	/* Clear address match bit & SDA pull high */
	inst->SMBnST = BIT(NPCM_SMBnST_NMATCH);
}

static void i2c_target_handle_SDA_STATUS(struct i2c_reg *inst, struct i2c_npcm_data *data)
{
	const struct i2c_target_callbacks *target_cb = data->target_cfg->callbacks;
	uint16_t len = 0;
	uint16_t dma_cnt;
	uint8_t overflow_byte;

	if (data->target_oper_state == I2C_NPCM_OPER_STA_READ) {
		/* Overflow */
		overflow_byte = inst->SMBnSDA;

		dma_cnt = i2c_DMA_get_cnt(inst);
		while (len < dma_cnt) {
			if (target_cb->write_received(data->target_cfg, data->rx_buf[len])) {
				break;
			}

			len++;
		}

		target_cb->write_received(data->target_cfg, overflow_byte);
		data->target_oper_state = I2C_NPCM_OPER_STA_START;
	} else {
		/* No enough DMA data to send, set SDA to default value */
		inst->SMBnSDA = ADDR_7BIT_MASK | ADDR_WRITE_MASK;
	}
}

static void i2c_target_handle_STOP(struct i2c_reg *inst, struct i2c_npcm_data *data)
{
	const struct i2c_target_callbacks *target_cb = data->target_cfg->callbacks;

	if (data->target_oper_state == I2C_NPCM_OPER_STA_READ) {
		i2c_target_read(inst, data, target_cb);
	}
	if (data->target_oper_state != I2C_NPCM_OPER_STA_IDLE) {
		target_cb->stop(data->target_cfg);
	}
	data->target_oper_state = I2C_NPCM_OPER_STA_START;

	/* Clear STOP flag */
	inst->SMBnST = BIT(NPCM_SMBnST_SLVSTP);
}

static void i2c_target_addr_set(struct i2c_reg *inst, uint8_t target_addr)
{
	/* Set target addr 1 */
	inst->SMBnADDR1 = (target_addr | BIT(NPCM_SMBnADDR_SAEN));

	/* Enable I2C address match interrupt */
	inst->SMBnCTL1 |= BIT(NPCM_SMBnCTL1_NMINTE);
}

/* I2C data processing functions */
static int i2c_msg_combine(struct i2c_npcm_data *data, struct i2c_msg *msgs, uint8_t num_msgs)
{
	uint8_t step = 0;
	uint8_t i;

	for (i = 0; i < num_msgs; i++) {
		if ((msgs[i].flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE) {
			/* Support more than one write msg in a transfer */
			if (step != 0) {
				return -1;
			}

			/* Write data to tx buffer */
			gdma_memcpy_u8(data->tx_buf + data->tx_cnt, msgs[i].buf, msgs[i].len);
			data->tx_cnt += msgs[i].len;
		} else {
			/* Just support one read msg in a transfer */
			if (step == 1) {
				return -1;
			}
			step = 1;

			/* Read data from rx buffer */
			data->rx_cnt = msgs[i].len;
			data->rx_msg_buf = msgs[i].buf;
		}
	}

	return 0;
}

/* I2C controller isr functions */
static void i2c_ctrl_isr(struct i2c_reg *inst, struct i2c_npcm_data *data)
{
	/* Handle timeout */
#if CONFIG_I2C_NPCM_CONTROLLER_HW_TIMEOUT_EN
	if (inst->TIMEOUT_ST & BIT(NPCM_TIMEOUT_ST_T_OUTST1)) {
		inst->TIMEOUT_ST = BIT(NPCM_TIMEOUT_ST_T_OUTST1);
		i2c_reset_module(inst, data);
		i2c_notify(inst, data, -ETIMEDOUT);
	}
	if (inst->TIMEOUT_ST & BIT(NPCM_TIMEOUT_ST_T_OUTST2)) {
		inst->TIMEOUT_ST = BIT(NPCM_TIMEOUT_ST_T_OUTST2);
		i2c_reset_module(inst, data);
		i2c_notify(inst, data, -ETIMEDOUT);
	}
#endif

	/* NACK occurred */
	if (inst->SMBnST & BIT(NPCM_SMBnST_NEGACK)) {
		i2c_abort_data(inst);
		inst->DMA_CTRL = BIT(NPCM_DMA_CTRL_DMA_INT_CLR);
		i2c_notify(inst, data, -ENXIO);
	}

	/* BUS ERROR occurred */
	if (inst->SMBnST & BIT(NPCM_SMBnST_BER)) {
		i2c_abort_data(inst);
		i2c_reset_module(inst, data);
		i2c_notify(inst, data, -EAGAIN);
	}

	/* SDA status is set - transmit or receive */
	if (inst->SMBnST & BIT(NPCM_SMBnST_SDAST)) {
		i2c_ctrl_handle_SDA_STATUS(inst, data);
	}

	/* Stall occurred */
	if (inst->SMBnST & BIT(NPCM_SMBnST_STASTR)) {
		i2c_ctrl_handle_STALL(inst, data);
	}

	/* DMA IRQ occurred */
	if (inst->DMA_CTRL & BIT(NPCM_DMA_CTRL_DMA_IRQ)) {
		i2c_ctrl_handle_DMA_IRQ(inst, data);
	}
}

static void i2c_target_isr(struct i2c_reg *inst, struct i2c_npcm_data *data)
{
#if CONFIG_I2C_NPCM_TARGET_HW_TIMEOUT_EN
	/* Timeout occurred */
	if (inst->TIMEOUT_ST & BIT(NPCM_TIMEOUT_ST_T_OUTST1)) {
		LOG_ERR("target: HW timeout");
		data->target_oper_state = I2C_NPCM_OPER_STA_START;
		inst->TIMEOUT_ST = BIT(NPCM_TIMEOUT_ST_T_OUTST1);
		i2c_reset_module(inst, data);
	}
	if (inst->TIMEOUT_ST & BIT(NPCM_TIMEOUT_ST_T_OUTST2)) {
		LOG_ERR("target: HW timeout");
		data->target_oper_state = I2C_NPCM_OPER_STA_START;
		inst->TIMEOUT_ST = BIT(NPCM_TIMEOUT_ST_T_OUTST2);
		i2c_reset_module(inst, data);
	}
#endif

	/* NACK occurred */
	if (inst->SMBnST & BIT(NPCM_SMBnST_NEGACK)) {
		/* Set 1 to clear NEGACK */
		inst->SMBnST = BIT(NPCM_SMBnST_NEGACK);
	}

	/* BUS ERROR occurred */
	if (inst->SMBnST & BIT(NPCM_SMBnST_BER)) {
		if (data->target_oper_state != I2C_NPCM_OPER_STA_QUICK) {
			LOG_ERR("target: bus error");
		}
		data->target_oper_state = I2C_NPCM_OPER_STA_START;

		/* Clear BER */
		inst->SMBnST = BIT(NPCM_SMBnST_BER);
		i2c_reset_module(inst, data);
	}

	/* DMA IRQ occurred */
	if (inst->DMA_CTRL & BIT(NPCM_DMA_CTRL_DMA_IRQ)) {
		if (data->target_oper_state == I2C_NPCM_OPER_STA_READ) {
			/* If DMA overflow, send NACK to Controller, and next IRQ is SDAST alert */
			inst->SMBnCTL1 =
				(inst->SMBnCTL1 & ~I2C_SMBnCTL1_MASK) | BIT(NPCM_SMBnCTL1_ACK);
		}

		/* Clear DMA interrupt */
		inst->DMA_CTRL = BIT(NPCM_DMA_CTRL_DMA_INT_CLR);
	}

	/* Address match occurred */
	if (inst->SMBnST & BIT(NPCM_SMBnST_NMATCH)) {
		i2c_target_handle_NMATCH(inst, data);
	}

	/* SDA status is set - transmit or receive */
	if (inst->SMBnST & BIT(NPCM_SMBnST_SDAST)) {
		i2c_target_handle_SDA_STATUS(inst, data);
	}

	/* Target STOP occurred */
	if (inst->SMBnST & BIT(NPCM_SMBnST_SLVSTP)) {
		i2c_target_handle_STOP(inst, data);
	}
}

static void i2c_npcm_isr(const struct device *dev)
{
	struct i2c_reg *const inst = I2C_BASE_INST(dev);
	struct i2c_npcm_data *data = dev->data;

	if (data->ctrl_oper_state != I2C_NPCM_OPER_STA_IDLE) {
		i2c_ctrl_isr(inst, data);
	} else {
		if (data->target_oper_state == I2C_NPCM_OPER_STA_IDLE) {
			/* Clear all interrupt status */
			inst->SMBnST = 0xFF;
		} else {
			i2c_target_isr(inst, data);
		}
	}
}

/* I2C API functions */
static int i2c_npcm_configure(const struct device *dev, uint32_t dev_config)
{
	struct i2c_npcm_data *const data = dev->data;

	if (!(dev_config & I2C_MODE_CONTROLLER)) {
		return -ENOTSUP;
	}

	if (dev_config & I2C_ADDR_10_BITS) {
		return -ENOTSUP;
	}

	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		/* 100 Kbit/s */
		data->bitrate = I2C_BITRATE_STANDARD;
		break;

	case I2C_SPEED_FAST:
		/* 400 Kbit/s */
		data->bitrate = I2C_BITRATE_FAST;
		break;

	case I2C_SPEED_FAST_PLUS:
		/* 1 Mbit/s */
		data->bitrate = I2C_BITRATE_FAST_PLUS;
		break;

	default:
		/* Not supported */
		return -ERANGE;
	}

	i2c_set_baudrate(dev, data->bitrate);

	return 0;
}

static int i2c_npcm_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			     uint16_t addr)
{
	struct i2c_reg *const inst = I2C_BASE_INST(dev);
	struct i2c_npcm_data *const data = dev->data;
	int ret;

	if (k_sem_take(&data->lock_sem, I2C_WAITING_TIME) != 0) {
		return -EBUSY;
	}

	/* Disable target addr 1 */
	inst->SMBnADDR1 &= ~BIT(NPCM_SMBnADDR_SAEN);

	/* Prepare data to transfer */
	data->rx_cnt = 0;
	data->tx_cnt = 0;
	data->dev_addr = addr << 1;
	data->ctrl_oper_state = I2C_NPCM_OPER_STA_START;
	data->err_code = 0;
	if (i2c_msg_combine(data, msgs, num_msgs) < 0) {
		ret = -EPROTONOSUPPORT;
		goto UNLOCK;
	}

	if (data->rx_cnt == 0 && data->tx_cnt == 0) {
		/* Quick command */
		if (num_msgs != 1) {
			/* Quick command must have one msg */
			ret = -EPROTONOSUPPORT;
			goto UNLOCK;
		}

		/* Set address to Write or Read address */
		data->dev_addr = (msgs->flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE
					 ? data->dev_addr & ADDR_7BIT_MASK
					 : data->dev_addr | ADDR_WRITE_MASK;
	}

#if CONFIG_I2C_NPCM_CONTROLLER_HW_TIMEOUT_EN
	/* Set I2C HW timeout value */
	i2c_timeout_cumulative_clockcycle_set(data, inst,
					      CONFIG_I2C_NPCM_CONTROLLER_HW_TIMEOUT_CLK_CYCLE_TIME);
	i2c_timeout_cumulative_clocklow_set(data, inst,
					    CONFIG_I2C_NPCM_CONTROLLER_HW_TIMEOUT_CLK_LOW_TIME);

	/* Enable HW Timeout */
	inst->TIMEOUT_EN |= BIT(NPCM_TIMEOUT_EN_TIMEOUT_EN);
#endif

	k_sem_reset(&data->sync_sem);

	i2c_start(inst);
	ret = i2c_wait_completion(inst, data);

	if (data->rx_cnt != 0) {
		gdma_memcpy_u8(data->rx_msg_buf, data->rx_buf, data->rx_cnt);
	}

UNLOCK:
	/* Enable target addr 1 */
	inst->SMBnADDR1 |= BIT(NPCM_SMBnADDR_SAEN);

	k_sem_give(&data->lock_sem);

	return ret;
}

static int i2c_npcm_target_register(const struct device *dev, struct i2c_target_config *cfg)
{
	struct i2c_npcm_data *data = dev->data;
	struct i2c_reg *const inst = I2C_BASE_INST(dev);
	int ret = 0;

	/* Check if target is valid */
	if (!cfg) {
		return -EINVAL;
	}
	/* Check if 10-bit address is supported */
	if (cfg->flags & I2C_TARGET_FLAGS_ADDR_10_BITS) {
		return -ENOTSUP;
	}
	/* Check if target is valid */
	if (k_sem_take(&data->lock_sem, I2C_WAITING_TIME) != 0) {
		return -EBUSY;
	}
	/* Check target is already registered */
	if (data->target_cfg) {
		ret = -EBUSY;
		goto UNLOCK;
	}

	data->target_cfg = cfg;
	data->target_oper_state = I2C_NPCM_OPER_STA_START;

	/* Set target with 7 bit address */
	i2c_target_addr_set(inst, cfg->address);

UNLOCK:
	k_sem_give(&data->lock_sem);

	return ret;
}

static int i2c_npcm_target_unregister(const struct device *dev, struct i2c_target_config *config)
{
	struct i2c_reg *const inst = I2C_BASE_INST(dev);
	struct i2c_npcm_data *data = dev->data;

	/* Check if target is valid */
	if (!data->target_cfg) {
		return -EINVAL;
	}
	/* Check if target state is idle */
	if (data->target_oper_state != I2C_NPCM_OPER_STA_START &&
	    data->ctrl_oper_state != I2C_NPCM_OPER_STA_IDLE) {
		return -EBUSY;
	}
	/* Check if target is already unregistered */
	if (k_sem_take(&data->lock_sem, I2C_WAITING_TIME) != 0) {
		return -EBUSY;
	}

	/* Clear target address 1 */
	inst->SMBnADDR1 = 0;

	/* Disable I2C address match interrupt */
	inst->SMBnCTL1 &= ~BIT(NPCM_SMBnCTL1_NMINTE);

	/* Clear all interrupt status */
	inst->SMBnST = 0xFF;

	/* Reset target state */
	data->target_oper_state = I2C_NPCM_OPER_STA_IDLE;
	data->target_cfg = NULL;

	k_sem_give(&data->lock_sem);

	return 0;
}

/* I2C controller init function */
static int i2c_npcm_device_init(const struct device *dev)
{
	const struct i2c_npcm_config *const config = dev->config;
	struct i2c_npcm_data *const data = dev->data;
	const struct device *const clk_dev = DEVICE_DT_GET(NPCM_CLK_CTRL_DEV);
	struct i2c_reg *const inst = I2C_BASE_INST(dev);
	int ret;

	LOG_DBG("Initializing I2C device: %s", dev->name);

	/* Configure pin-mux for I2C device */
	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Failed to setup I2C pinctrl (%d)", ret);
		return ret;
	}

	/* Turn on device clock and get source clock frequency */
	ret = clock_control_on(clk_dev, (clock_control_subsys_t *)config->clk_cfg);
	if (ret != 0) {
		LOG_ERR("Failed to turn on clock for %s", dev->name);
		return -EIO;
	}

	/* Reset data */
	gdma_memset_u8((uint8_t *)data, 0, sizeof(struct i2c_npcm_data));

	/* Set default baud rate for I2C */
	data->bitrate = config->default_bitrate;
	i2c_set_baudrate(dev, data->bitrate);

	LOG_DBG("bitrate: %d", data->bitrate);

	/* Enable I2C module and interrupt */
	i2c_enable(inst);
	i2c_interrupt_enable(inst);

	/* Initialize mutex and semaphore for I2C controller */
	ret = k_sem_init(&data->lock_sem, 1, 1);
	if (ret != 0) {
		LOG_ERR("Failed to initialize lock semaphore (%d)", ret);
		return ret;
	}

	ret = k_sem_init(&data->sync_sem, 0, 1);
	if (ret != 0) {
		LOG_ERR("Failed to initialize sync semaphore (%d)", ret);
		return ret;
	}

	/* Initialize driver status machine */
	data->ctrl_oper_state = I2C_NPCM_OPER_STA_IDLE;
	data->target_cfg = NULL;

	return ret;
}

/* I2C driver APIs */
static DEVICE_API(i2c, i2c_npcm_driver_api) = {
	.configure = i2c_npcm_configure,
	.transfer = i2c_npcm_transfer,
	.target_register = i2c_npcm_target_register,
	.target_unregister = i2c_npcm_target_unregister,
};

/* I2C controller init macro functions */
#define I2C_NPCM_CTRL_INIT_FUNC_IMPL(inst)                                                         \
	static int i2c_npcm_init_##inst(const struct device *dev)                                  \
	{                                                                                          \
		int ret;                                                                           \
                                                                                                   \
		ret = i2c_npcm_device_init(dev);                                                   \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), i2c_npcm_isr,         \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN(inst));                                                    \
                                                                                                   \
		return ret;                                                                        \
	}

/* I2C controller registration */
#define I2C_NPCM_CTRL_INIT(inst)                                                                   \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
                                                                                                   \
	static int i2c_npcm_init_##inst(const struct device *dev);                                 \
	static const struct i2c_npcm_config i2c_npcm_cfg_##inst = {                                \
		.base = DT_INST_REG_ADDR(inst),                                                    \
		.clk_cfg = DT_INST_PHA(inst, clocks, clk_id),                                     \
		.default_bitrate = DT_INST_PROP(inst, clock_frequency),                            \
		.irq = DT_INST_IRQN(inst),                                                         \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                      \
	};                                                                                         \
	static struct i2c_npcm_data i2c_npcm_data_##inst;                                          \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, i2c_npcm_init_##inst, NULL, &i2c_npcm_data_##inst,             \
			      &i2c_npcm_cfg_##inst, PRE_KERNEL_1, CONFIG_I2C_INIT_PRIORITY,        \
			      &i2c_npcm_driver_api);                                               \
                                                                                                   \
	I2C_NPCM_CTRL_INIT_FUNC_IMPL(inst)

DT_INST_FOREACH_STATUS_OKAY(I2C_NPCM_CTRL_INIT)
