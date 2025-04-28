/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcm_fiu_qspi

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/flash/npcm_flash_api_ex.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/dt-bindings/flash_controller/npcm_qspi.h>
#include <soc.h>

#include "flash_npcm_qspi.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(npcm_fiu_qspi, CONFIG_FLASH_LOG_LEVEL);

#define NPCM_FIU_PVT_CS		NPCM_QSPI_SW_CS0
#define NPCM_FIU_SHD_CS		NPCM_QSPI_SW_CS1
#define NPCM_FIU_BACK_CS	NPCM_QSPI_SW_CS2

/* Driver convenience defines */
#define HAL_INSTANCE(dev)                                                                          \
	((struct fiu_reg *)((const struct npcm_qspi_fiu_config *)(dev)->config)->base)
#define HAL_SFCG_INST() (struct scfg_reg *)(npcm_scfg_cfg.base_scfg)
#define NPCM_DEV_CTL3   0x04
#define DEV_CTL3_INST   (npcm_scfg_cfg.base_scfg + NPCM_DEV_CTL3)

/* Driver access helper */
#define GET_POS_FIELD(pos, size)     pos
#define GET_SIZE_FIELD(pos, size)    size
#define FIELD_POS(field)             GET_POS_##field
#define FIELD_SIZE(field)            GET_SIZE_##field
#define SET_FIELD(reg, field, value) _SET_FIELD_(reg, FIELD_POS(field), FIELD_SIZE(field), value)
#define _SET_FIELD_(reg, f_pos, f_size, value)                                                     \
	((reg) = ((reg) & (~(((1 << (f_size)) - 1) << (f_pos)))) | ((value) << (f_pos)))

/* SCFG base */
struct npcm_scfg_config {
	/* scfg device base address */
	uintptr_t base_scfg;
};

static const struct npcm_scfg_config npcm_scfg_cfg = {
	.base_scfg = DT_REG_ADDR_BY_NAME(DT_INST(0, nuvoton_npcm_pinctrl), scfg),
};

/* Flash Interface Unit (FIU) device registers */
struct fiu_reg {
	volatile uint8_t reserved1;
	/* 0x001: Burst Configuration */
	volatile uint8_t BURST_CFG;
	/* 0x002: FIU Response Configuration */
	volatile uint8_t RESP_CFG;
	volatile uint8_t reserved2[17];
	/* 0x014: SPI Flash Configuration */
	volatile uint8_t SPI_FL_CFG;
	volatile uint8_t reserved3;
	/* 0x016: UMA Code Byte */
	volatile uint8_t UMA_CODE;
	/* 0x017: UMA Address Byte 0 */
	volatile uint8_t UMA_AB0;
	/* 0x018: UMA Address Byte 1 */
	volatile uint8_t UMA_AB1;
	/* 0x019: UMA Address Byte 2 */
	volatile uint8_t UMA_AB2;
	/* 0x01A: UMA Data Byte 0 */
	volatile uint8_t UMA_DB0;
	/* 0x01B: UMA Data Byte 1 */
	volatile uint8_t UMA_DB1;
	/* 0x01C: UMA Data Byte 2 */
	volatile uint8_t UMA_DB2;
	/* 0x01D: UMA Data Byte 3 */
	volatile uint8_t UMA_DB3;
	/* 0x01E: UMA Control and Status */
	volatile uint8_t UMA_CTS;
	/* 0x01F: UMA Extended Control and Status */
	volatile uint8_t UMA_ECTS;
	/* 0x020: UMA Data Bytes 0-3 */
	volatile uint32_t UMA_DB0_3;
	volatile uint8_t reserved4[2];
	/* 0x026: CRC Control Register */
	volatile uint8_t CRCCON;
	/* 0x027: CRC Entry Register */
	volatile uint8_t CRCENT;
	/* 0x028: CRC Initialization and Result Register */
	volatile uint32_t CRCRSLT;
	volatile uint8_t reserved5[2];
	/* 0x02E: FIU Read Command for Back-up flash */
	volatile uint8_t RD_CMD_BACK;
	volatile uint8_t reserved6;
	/* 0x030: FIU Read Command for private flash */
	volatile uint8_t RD_CMD_PVT;
	/* 0x031: FIU Read Command for shared flash */
	volatile uint8_t RD_CMD_SHD;
	volatile uint8_t reserved7;
	/* 0x033: FIU Extended Configuration */
	volatile uint8_t FIU_EXT_CFG;
	/* 0x034: UMA AB0~3 */
	volatile uint32_t UMA_AB0_3;
	volatile uint8_t reserved8[4];
	/* 0x03C: Set command enable in 4 Byte address mode */
	volatile uint8_t SET_CMD_EN;
	/* 0x03D: 4 Byte address mode Enable */
	volatile uint8_t ADDR_4B_EN;
	volatile uint8_t reserved9[3];
	/* 0x041: Master Inactive Counter Threshold */
	volatile uint8_t MI_CNT_THRSH;
	/* 0x042: FIU Matser Status */
	volatile uint8_t FIU_MSR_STS;
	/* 0x043: FIU Master Interrupt Enable and Configuration */
	volatile uint8_t FIU_MSR_IE_CFG;
	/* 0x044: Quad Program Enable */
	volatile uint8_t Q_P_EN;
	volatile uint8_t reserved10[3];
	/* 0x048: Extended Data Byte Configuration */
	volatile uint8_t EXT_DB_CFG;
	/* 0x049: Direct Write Configuration */
	volatile uint8_t DIRECT_WR_CFG;
	volatile uint8_t reserved11[6];
	/* 0x050 ~ 0x060: Extended Data Byte F to 0 */
	volatile uint8_t EXT_DB_F_0[16];
};

/* FIU register fields */
/* 0x001: BURST CFG */
#define NPCM_BURST_CFG_R_BURST			FIELD(0, 2)
#define NPCM_BURST_CFG_SLAVE			2
#define NPCM_BURST_CFG_UNLIM_BURST		3
#define NPCM_BURST_CFG_SPI_WR_EN		7

/* 0x002: RESP CFG */
#define NPCM_RESP_CFG_QUAD_EN			3

/* 0x014: SPI FL CFG */
#define NPCM_SPI_FL_CFG_RD_MODE			FIELD(6, 2)
#define NPCM_SPI_FL_CFG_RD_MODE_NORMAL		0
#define NPCM_SPI_FL_CFG_RD_MODE_FAST		1
#define NPCM_SPI_FL_CFG_RD_MODE_FAST_DUAL	3

/* 0x01E: UMA CTS */
#define NPCM_UMA_CTS_D_SIZE			FIELD(0, 3)
#define NPCM_UMA_CTS_A_SIZE			3
#define NPCM_UMA_CTS_C_SIZE			4
#define NPCM_UMA_CTS_RD_WR			5
#define NPCM_UMA_CTS_DEV_NUM			6
#define NPCM_UMA_CTS_EXEC_DONE			7

/* 0x01F: UMA ECTS */
#define NPCM_UMA_ECTS_SW_CS0			0
#define NPCM_UMA_ECTS_SW_CS1			1
#define NPCM_UMA_ECTS_SW_CS2			2
#define NPCM_UMA_ECTS_DEV_NUM_BACK		3
#define NPCM_UMA_ECTS_UMA_ADDR_SIZE		FIELD(4, 3)

/* 0x026: CRC Control Register */
#define NPCM_CRCCON_CALCEN			0
#define NPCM_CRCCON_CKSMCRC			1
#define NCPM_CRCCON_UMAENT			2


/* 0x033: FIU Extended Configuration Register */
#define NPCM_FIU_EXT_CFG_FOUR_BADDR		0

/* 0x03C: Set command enable 4 bytes address mode */
#define NPCM_SET_CMD_EN_PVT_CMD_EN		4
#define NCPM_SET_CMD_EN_SHD_CMD_EN		5
#define NCPM_SET_CMD_EN_BACK_CMD_EN		6

/* 0x03D: 4 bytes address mode enable */
#define NPCM_ADDR_4B_EN_PVT_4B			4
#define NCPM_ADDR_4B_EN_SHD_4B			5
#define NCPM_ADDR_4B_EN_BACK_4B			6

/* 0x043: FIU master interrupt enable and configuration register */
#define NPCM_FIU_MSR_IE_CFG_RD_PEND_UMA_IE	0
#define NPCM_FIU_MSR_IE_CFG_RD_PEND_FIU_IE	1
#define NPCM_FIU_MSR_IE_CFG_MSTR_INACT_IE	2
#define NPCM_FIU_MSR_IE_CFG_UMA_BLOCK		3

/* 0x044: Quad program enable register */
#define NPCM_Q_P_EN_QUAD_P_EN			0

/* 0x048: Extended data byte configurartion */
#define NPCM_EXT_DB_CFG_D_SIZE_DB		FIELD(0, 5)
#define NPCM_EXT_DB_CFG_EXT_DB_EN		5

/* 0x049: Direct write configuration */
#define NPCM_DIRECT_WR_CFG_DIRECT_WR_BLOCK	1
#define NPCM_DIRECT_WR_CFG_DW_CS2		5
#define NPCM_DIRECT_WR_CFG_DW_CS1		6
#define NPCM_DIRECT_WR_CFG_DW_CS0		7

/* BURST CFG R_BURST selections */
#define NPCM_BURST_CFG_R_BURST_1B		0
#define NPCM_BURST_CFG_R_BURST_16B		3

#define NPCM_FIU_ADDR_3B_LENGTH			0x3
#define NPCM_FIU_ADDR_4B_LENGTH			0x4
#define NPCM_FIU_EXT_DB_SIZE			0x10

/* UMA fields selections */
#define UMA_FLD_ADDR     BIT(NPCM_UMA_CTS_A_SIZE)  /* 3-bytes ADR field */
#define UMA_FLD_NO_CMD   BIT(NPCM_UMA_CTS_C_SIZE)  /* No 1-Byte CMD field */
#define UMA_FLD_WRITE    BIT(NPCM_UMA_CTS_RD_WR)   /* Write transaction */
#define UMA_FLD_SHD_SL   BIT(NPCM_UMA_CTS_DEV_NUM) /* Shared flash selected */
#define UMA_FLD_EXEC     BIT(NPCM_UMA_CTS_EXEC_DONE)

#define UMA_FIELD_ADDR_0 0x00
#define UMA_FIELD_ADDR_1 0x01
#define UMA_FIELD_ADDR_2 0x02
#define UMA_FIELD_ADDR_3 0x03
#define UMA_FIELD_ADDR_4 0x04

#define UMA_FIELD_DATA_0 0x00
#define UMA_FIELD_DATA_1 0x01
#define UMA_FIELD_DATA_2 0x02
#define UMA_FIELD_DATA_3 0x03
#define UMA_FIELD_DATA_4 0x04

/* UMA code for transaction */
#define UMA_CODE_ONLY_WRITE           (UMA_FLD_EXEC | UMA_FLD_WRITE)
#define UMA_CODE_ONLY_READ_BYTE(n)    (UMA_FLD_EXEC | UMA_FLD_NO_CMD | UMA_FIELD_DATA_##n)

/* Device config */
struct npcm_qspi_fiu_config {
	/* Flash controller base address */
	uintptr_t base;
	/* Clock configuration */
	uint32_t clk_cfg;
};

/* Clock control device */
#define NPCM_CLK_CTRL_DEV DT_NODELABEL(pcc)

#define NPCM_FIU_FLASH_WP  BIT(0)
#define NPCM_SPIM_FLASH_WP BIT(1)
#define NPCM_DEV_CTL3_WP_INT_FL               0
#define NPCM_DEV_CTL3_WP_GPIO55               1
#define NPCM_DEV_CTL3_WP_GPIO76               2
static int npcm_pinctrl_flash_write_protect_set(int interface)
{
	volatile uintptr_t dev_ctl3 = DEV_CTL3_INST;
	uint32_t reg_val;

	switch (interface) {
	case NPCM_FIU_FLASH_WP:
		reg_val = sys_read32(dev_ctl3) | BIT(NPCM_DEV_CTL3_WP_GPIO55);
		sys_write32(reg_val, dev_ctl3);
		if (!(sys_read32(dev_ctl3) & BIT(NPCM_DEV_CTL3_WP_GPIO55))) {
			return -EIO;
		}
		break;

	case NPCM_SPIM_FLASH_WP:
		reg_val = sys_read32(dev_ctl3) | BIT(NPCM_DEV_CTL3_WP_INT_FL);
		sys_write32(reg_val, dev_ctl3);
		if (!(sys_read32(dev_ctl3) & BIT(NPCM_DEV_CTL3_WP_INT_FL))) {
			return -EIO;
		}
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

/* NPCM SPI User Mode Access (UMA) functions */
static inline void qspi_npcm_uma_cs_level(const struct device *dev, uint8_t sw_cs, bool level)
{
	struct fiu_reg *const inst = HAL_INSTANCE(dev);

	/* Set chip select to high/low level */
	if (level) {
		inst->UMA_ECTS |= BIT(sw_cs);
	} else {
		inst->UMA_ECTS &= ~BIT(sw_cs);
	}
}

static inline void qspi_npcm_uma_write_byte(const struct device *dev, uint8_t data)
{
	struct fiu_reg *const inst = HAL_INSTANCE(dev);
	struct npcm_qspi_data *const qspi_data = dev->data;
	const struct npcm_qspi_cfg *qspi_cfg = qspi_data->cur_cfg;
	int cts = 0;

	/* Set data to UMA_CODE and trigger UMA */
	inst->UMA_CODE = data;

	cts = UMA_CODE_ONLY_WRITE;

	/* share flash select, otherwise pvt or back */
	if (qspi_cfg->flags & NPCM_FIU_SHD_CS) {
		cts |= UMA_FLD_SHD_SL;
	} else {
		cts &= ~UMA_FLD_SHD_SL;
	}

	inst->UMA_CTS = cts;

	/* EXEC_DONE will be zero automatically if a UMA transaction is completed. */
	while (IS_BIT_SET(inst->UMA_CTS, NPCM_UMA_CTS_EXEC_DONE)) {
		continue;
	}
}

static inline void qspi_npcm_uma_read_byte(const struct device *dev, uint8_t *data)
{
	struct fiu_reg *const inst = HAL_INSTANCE(dev);
	struct npcm_qspi_data *const qspi_data = dev->data;
	const struct npcm_qspi_cfg *qspi_cfg = qspi_data->cur_cfg;
	int cts = 0;

	cts = UMA_CODE_ONLY_READ_BYTE(1);

	/* share flash select, otherwise pvt or back */
	if (qspi_cfg->flags & NPCM_FIU_SHD_CS) {
		cts |= UMA_FLD_SHD_SL;
	} else {
		cts &= ~UMA_FLD_SHD_SL;
	}

	/* Trigger UMA and Get data from DB0 later */
	inst->UMA_CTS = cts;

	while (IS_BIT_SET(inst->UMA_CTS, NPCM_UMA_CTS_EXEC_DONE)) {
		continue;
	}

	*data = inst->UMA_DB0;
}

/* NPCM SPI Direct Read Access (DRA)/User Mode Access (UMA) configuration functions */
static inline void qspi_npcm_config_uma_mode(const struct device *dev,
					     const struct npcm_qspi_cfg *qspi_cfg)
{
	struct fiu_reg *const inst = HAL_INSTANCE(dev);

	/* back flash select, otherwise share or pvt */
	if (qspi_cfg->flags & NPCM_FIU_BACK_CS) {
		inst->UMA_ECTS |= BIT(NPCM_UMA_ECTS_DEV_NUM_BACK);
	} else {
		inst->UMA_ECTS &= ~BIT(NPCM_UMA_ECTS_DEV_NUM_BACK);
	}
}

static inline void qspi_npcm_config_dra_4byte_mode(const struct device *dev,
						   const struct npcm_qspi_cfg *qspi_cfg)
{
#if defined(CONFIG_FLASH_NPCM_FIU_SUPP_DRA_4B_ADDR)
	struct fiu_reg *const inst = HAL_INSTANCE(dev);

	if (qspi_cfg->enter_4ba != 0) {
		inst->ADDR_4B_EN |= (qspi_cfg->flags & NPCM_QSPI_SW_CS_MASK) << 4;

	} else {
		inst->ADDR_4B_EN = 0;
	}
#endif /* CONFIG_FLASH_NPCM_FIU_SUPP_DRA_4B_ADDR */
}

static inline void qspi_npcm_config_dra_mode(const struct device *dev,
					     const struct npcm_qspi_cfg *qspi_cfg)
{
	struct fiu_reg *const inst = HAL_INSTANCE(dev);

	/* Enable quad mode of Direct Read Mode if needed */
	if (qspi_cfg->qer_type != JESD216_DW15_QER_NONE) {
		inst->RESP_CFG |= BIT(NPCM_RESP_CFG_QUAD_EN);
	} else {
		inst->RESP_CFG &= ~BIT(NPCM_RESP_CFG_QUAD_EN);
	}

	/* Selects the SPI read access type of Direct Read Access mode */
	switch (qspi_cfg->rd_mode) {
		case NPCM_RD_MODE_NORMAL:
			SET_FIELD(inst->SPI_FL_CFG, NPCM_SPI_FL_CFG_RD_MODE,
					NPCM_SPI_FL_CFG_RD_MODE_NORMAL);
                        break;
		case NPCM_RD_MODE_FAST:
			SET_FIELD(inst->SPI_FL_CFG, NPCM_SPI_FL_CFG_RD_MODE,
					NPCM_SPI_FL_CFG_RD_MODE_FAST);
			break;
		case NPCM_RD_MODE_FAST_DUAL:
			SET_FIELD(inst->SPI_FL_CFG, NPCM_SPI_FL_CFG_RD_MODE,
					NPCM_SPI_FL_CFG_RD_MODE_FAST_DUAL);
			break;
		case NPCM_RD_MODE_QUAD:
			SET_FIELD(inst->SPI_FL_CFG, NPCM_SPI_FL_CFG_RD_MODE,
					NPCM_SPI_FL_CFG_RD_MODE_FAST_DUAL);
			inst->RESP_CFG |= BIT(NPCM_RESP_CFG_QUAD_EN);
			break;
                default:
			LOG_ERR("un-support rd mode:%d", qspi_cfg->rd_mode);
			break;
	}

	/* Enable/Disable 4 byte address mode for Direct Read Access (DRA) */
	qspi_npcm_config_dra_4byte_mode(dev, qspi_cfg);

	/* set read max burst 16 bytes */
	SET_FIELD(inst->BURST_CFG, NPCM_BURST_CFG_R_BURST,
			NPCM_BURST_CFG_R_BURST_16B);
}

static inline void qspi_npcm_fiu_set_operation(const struct device *dev, uint32_t operation)
{
	if ((operation & NPCM_EX_OP_EXT_FLASH_WP) != 0) {
		npcm_pinctrl_flash_write_protect_set(NPCM_FIU_FLASH_WP);
	}
}

static inline void qspi_npcm_fiu_uma_lock(const struct device *dev)
{
	struct fiu_reg *const inst = HAL_INSTANCE(dev);

	inst->FIU_MSR_IE_CFG |= BIT(NPCM_FIU_MSR_IE_CFG_UMA_BLOCK);
}

static inline void qspi_npcm_fiu_uma_release(const struct device *dev)
{
	struct fiu_reg *const inst = HAL_INSTANCE(dev);

	inst->FIU_MSR_IE_CFG &= ~BIT(NPCM_FIU_MSR_IE_CFG_UMA_BLOCK);
}

/* NPCM specific QSPI-FIU controller functions */
static int qspi_npcm_fiu_uma_transceive(const struct device *dev, struct npcm_transceive_cfg *cfg,
				     uint32_t flags)
{
	struct npcm_qspi_data *const data = dev->data;

	/* Transaction is permitted? */
	if ((data->operation & NPCM_EX_OP_LOCK_TRANSCEIVE) != 0) {
		return -EPERM;
	}

	/* UMA block */
	qspi_npcm_fiu_uma_lock(dev);

	/* Assert chip select */
	qspi_npcm_uma_cs_level(dev, data->sw_cs, false);

	/* Transmit op-code first */
	qspi_npcm_uma_write_byte(dev, cfg->opcode);

	if ((flags & NPCM_TRANSCEIVE_ACCESS_ADDR) != 0) {
		/* 3-byte or 4-byte address? */
		const int addr_start = (data->cur_cfg->enter_4ba != 0) ? 0 : 1;

		for (size_t i = addr_start; i < 4; i++) {
			LOG_DBG("addr %d, %02x", i, cfg->addr.u8[i]);
			qspi_npcm_uma_write_byte(dev, cfg->addr.u8[i]);
		}
	}

	if ((flags & NPCM_TRANSCEIVE_ACCESS_WRITE) != 0) {
		if (cfg->tx_buf == NULL) {
			return -EINVAL;
		}
		for (size_t i = 0; i < cfg->tx_count; i++) {
			qspi_npcm_uma_write_byte(dev, cfg->tx_buf[i]);
		}
	}

	if ((flags & NPCM_TRANSCEIVE_ACCESS_READ) != 0) {
		if (cfg->rx_buf == NULL) {
			return -EINVAL;
		}
		for (size_t i = 0; i < cfg->rx_count; i++) {
			qspi_npcm_uma_read_byte(dev, cfg->rx_buf + i);
		}
	}

	/* De-assert chip select */
	qspi_npcm_uma_cs_level(dev, data->sw_cs, true);

	/* UMA unblock */
	qspi_npcm_fiu_uma_release(dev);

	return 0;
}

static void qspi_npcm_fiu_mutex_lock_configure(const struct device *dev,
					const struct npcm_qspi_cfg *cfg,
					const uint32_t operation)
{
	struct npcm_qspi_data *const data = dev->data;

	k_sem_take(&data->lock_sem, K_FOREVER);

	/* If the current device is different from previous one, configure it */
	if (data->cur_cfg != cfg) {
		data->cur_cfg = cfg;

		/* Apply pin-muxing and tri-state */
		pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);

		/* Save SW CS bit used in UMA mode */
		data->sw_cs = find_lsb_set(cfg->flags & NPCM_QSPI_SW_CS_MASK) - 1;

		/* Configure User Mode Access (UMA) settings */
		qspi_npcm_config_uma_mode(dev, cfg);

		/* Configure for Direct Read Access (DRA) settings */
		qspi_npcm_config_dra_mode(dev, cfg);
	}

	/* Set QSPI bus operation */
	if (data->operation != operation) {
		qspi_npcm_fiu_set_operation(dev, operation);
		data->operation = operation;
	}
}

static void qspi_npcm_fiu_mutex_unlock(const struct device *dev)
{
	struct npcm_qspi_data *const data = dev->data;

	k_sem_give(&data->lock_sem);
}

struct npcm_qspi_ops npcm_qspi_fiu_ops = {
	.lock_configure = qspi_npcm_fiu_mutex_lock_configure,
        .unlock = qspi_npcm_fiu_mutex_unlock,
        .transceive = qspi_npcm_fiu_uma_transceive,
};

static int qspi_npcm_fiu_init(const struct device *dev)
{
	const struct npcm_qspi_fiu_config *const config = dev->config;
	struct npcm_qspi_data *const data = dev->data;
	const struct device *const clk_dev = DEVICE_DT_GET(NPCM_CLK_CTRL_DEV);
	int ret;

	if (!device_is_ready(clk_dev)) {
		LOG_ERR("%s device not ready", clk_dev->name);
		return -ENODEV;
	}

	/* Turn on device clock first and get source clock freq. */
	ret = clock_control_on(clk_dev, (clock_control_subsys_t *)config->clk_cfg);
	if (ret < 0) {
		LOG_ERR("Turn on FIU clock fail %d", ret);
		return ret;
	}

	/* initialize mutex for qspi controller */
	k_sem_init(&data->lock_sem, 1, 1);

	return 0;
}

#define NPCM_SPI_FIU_INIT(n)                                                                       \
	static const struct npcm_qspi_fiu_config npcm_qspi_fiu_config_##n = {                      \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.clk_cfg = DT_INST_PHA(n, clocks, clk_id),                                         \
	};                                                                                         \
	static struct npcm_qspi_data npcm_qspi_data_##n = {.qspi_ops = &npcm_qspi_fiu_ops};        \
	DEVICE_DT_INST_DEFINE(n, qspi_npcm_fiu_init, NULL, &npcm_qspi_data_##n,                    \
			      &npcm_qspi_fiu_config_##n, PRE_KERNEL_1, CONFIG_FLASH_INIT_PRIORITY, \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(NPCM_SPI_FIU_INIT)
