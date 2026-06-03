/*
 * Copyright (c) 2026 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT nuvoton_npcm4_i3c

#include <drivers/clock_control.h>
#include <drivers/reset_control.h>
#include <drivers/i3c/i3c.h>
#include <soc.h>
#include <sys/util.h>
#include <device.h>
#include <kernel.h>
#include <init.h>
#include <sys/sys_io.h>
#include <logging/log.h>
#define LOG_LEVEL CONFIG_I3C_LOG_LEVEL
LOG_MODULE_REGISTER(i3c_npcm4);

#include <portability/cmsis_os2.h>

#define DEV_CFG(dev)			((const struct i3c_npcm4_config *)(dev)->config)
#define DEV_DATA(dev)			((struct i3c_npcm4_obj *)(dev)->data)

/* NPCM4 PDMA Definitions */
#define PDMA_CHAN(b, c)		(b + c * 0x10)

/* NPCM4 I3C Register Definitions */
#define DMACTRL_DMAWIDTH(x)	FIELD_PREP(GENMASK(5, 4), (x))
#define DMACTRL_DMATB_EN	BIT(3)
#define DMACTRL_DMAFB_EN	BIT(1)

/* STATUS bit fileds */
#define STATUS_HJDIS		BIT(27)
#define STATUS_MRDIS		BIT(25)
#define STATUS_IBIDIS		BIT(24)
#define STATUS_EVENT		BIT(18)
#define STATUS_RXPEND		BIT(11)
#define STATUS_STOP		BIT(10)
#define STATUS_MATCHED		BIT(9)

/* IRQ number */
#define NUM_CALLBACKS		11
#define IRQ_START		8
#define IRQ_STOP		10
#define IRQ_DACHG		13
#define IRQ_EVENT		18

/* Slave State */
#define SLAVE_DA_ASSIGNED	BIT(0)
#define SLAVE_WAIT_FOR_RX	BIT(1)
#define SLAVE_WAIT_FOR_TX	BIT(2)
#define SLAVE_TX_READY		BIT(3)
#define SLAVE_REQUEST_IBI	BIT(4)

/* Device configuration structure */
struct i3c_npcm4_config {
	int inst_id;

	/* Slave config */
	int assigned_addr;
	uint8_t bcr;
	uint8_t dcr;
	uint16_t part_id; /* PID[3:2] */
	uint16_t vendor_def_id; /* PID[1:0] */

	int dma_tx_channel;
	int dma_rx_channel;
	struct i3c_reg *base;
	struct pdma_reg *pdma_base;
	uintptr_t pmc_base;
	uint32_t irq;
	struct npcm4xx_clk_cfg clk_cfg;
	bool slave;
	bool secondary;
	uint32_t i3c_scl_hz;
	uint32_t i2c_scl_hz;
	int32_t hj_timeout_ms; /* -1 = disabled */
};

typedef int (*isr_cb_t)(const struct device *dev);

/* Device runtime data structure */
struct i3c_npcm4_obj {
	const struct device *dev;
	uint32_t apb3_rate;

	/* Slave mode data */
	struct i3c_slave_setup slave_data;
	int state;

	/* Semaphore for complete event */
	struct k_sem complete;

	/* Delayed work to cancel hot-join request on timeout */
	struct k_work_delayable hj_timeout_work;

	/* DMA structure */
	uint8_t *dma_buf;
	struct dsct_reg *rx_desc;
	struct dsct_reg *tx_desc;
	uint32_t txlen;
	bool use_dma_tx;

	/* ISR callbacks */
	isr_cb_t isr_cb[NUM_CALLBACKS];
};

#define I3C_NPCM4_FIFO_SIZE	16
#define NUM_MODULES		6

/* DMA definitions */
#define DMA_BUF_SIZE		256
#define TXWIDTH_8		(0 << 12)
#define DAINC_1			(1 << 10)
#define DAINC_FIXED		(3 << 10)
#define SAINC_1			(1 << 8)
#define SAINC_FIXED		(3 << 8)
#define REQ_TYPE_SINGLE		(1 << 2)
#define BASIC_MODE		1
#define SG_MODE			2

static char dma_buf_pool[DMA_BUF_SIZE * NUM_MODULES];
/* DSCT table for scatter-gather mode */
struct dsct_reg sg_dsct[NUM_MODULES * 2] __aligned(256);

int i3c_npcm4_slave_get_dynamic_addr(const struct device *dev, uint8_t *dynamic_addr);

static inline int readl_poll_timeout(uintptr_t addr, uint32_t mask,
				     uint32_t expect_bits,
				     int timeout_ms, int interval_us,
				     bool atomic)
{
	int64_t deadline = k_uptime_get() + timeout_ms;

	do {
		uint32_t val = sys_read32((mem_addr_t)addr);

		if ((val & mask) == expect_bits)
			return 0;

		if (!atomic) {
			if (interval_us > 0) {
				k_usleep(interval_us);
			} else {
				k_yield();
			}
		}
	} while (k_uptime_get() <= deadline);

	return -ETIMEDOUT;
}

static inline void i3c_npcm4_stop_dma_rx(const struct device *dev)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	uint32_t val = config->base->DMACTRL & ~GENMASK(1, 0);

	config->pdma_base->STOP = (1 << config->dma_rx_channel);
	config->base->DMACTRL = val;
	obj->state &= ~SLAVE_WAIT_FOR_RX;
}

static inline void i3c_npcm4_stop_dma_tx(const struct i3c_npcm4_config *config)
{
	uint32_t val = config->base->DMACTRL & ~GENMASK(3, 2);

	config->pdma_base->STOP = (1 << config->dma_tx_channel);
	config->base->DMACTRL = val;
}

static inline void i3c_npcm4_enable_interrupt(struct i3c_reg *reg, int irq)
{
	reg->INTSET = (1 << irq);
}

static inline void i3c_npcm4_disable_interrupt(struct i3c_reg *reg, int irq)
{
	reg->INTCLR = (1 << irq);
}

static inline void i3c_npcm4_clear_irq_status(struct i3c_reg *reg, int irq)
{
	reg->STATUS = (1 << irq);
}

static void i3c_npcm4_register_isr_cb(const struct device *dev, int irq, isr_cb_t cb)
{
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct i3c_reg *reg = config->base;

	if (irq < IRQ_START || irq > IRQ_EVENT)
		return;

	obj->isr_cb[irq - IRQ_START] = cb;

	if (cb) {
		/* Clear IRQ status and enable interrupt */
		i3c_npcm4_clear_irq_status(reg, irq);
		i3c_npcm4_enable_interrupt(reg, irq);
	} else {
		i3c_npcm4_disable_interrupt(reg, irq);
	}
}

static int i3c_npcm4_dma_read(const struct device *dev)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct dsct_reg *desc = (struct dsct_reg *)&config->pdma_base->DSCT[config->dma_rx_channel];
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	uint32_t val = config->base->DMACTRL & ~GENMASK(1, 0);

	obj->rx_desc = desc;
	obj->state |= SLAVE_WAIT_FOR_RX;
	/*
	 * Fill the pdma descriptor:
	 * Transfer width: 8 bit
	 * Source Address: fixed
	 * Destination Address: increaded by 1 transfer width
	 * Single request type
	 * Basic mode
	 */
	desc->CTL = ((DMA_BUF_SIZE - 1) << 16) | TXWIDTH_8 | SAINC_FIXED | DAINC_1 | REQ_TYPE_SINGLE | BASIC_MODE;
	desc->ENDSA = (uint32_t)&config->base->RDATAB;
	desc->ENDDA = (uint32_t)obj->dma_buf;
	desc->NEXT = 0;

	/* Enable PDMA RX channel */
	config->pdma_base->CHCTL |= 1 << config->dma_rx_channel;

	/*
	 * Enable I3C DMA (setup DMACTRL)
	 * transfer width: 1 byte
	 * direction: DMA from bus
	 */
	config->base->DMACTRL = (val | DMACTRL_DMAFB_EN);

	return 0;
}

static int i3c_npcm4_dma_write(const struct device *dev, uint8_t *buf, int len)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct dsct_reg *head = (struct dsct_reg *)&config->pdma_base->DSCT[config->dma_tx_channel];
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	struct dsct_reg *desc = &sg_dsct[config->inst_id * 2];
	uint32_t val = config->base->DMACTRL & ~GENMASK(3, 2);

	obj->tx_desc = head;

	if (len > DMA_BUF_SIZE) {
		LOG_ERR("Invalid write len %d", len);
		return -EINVAL;
	}

	/*
	 * Use scatter-gather mode for DMA write.
	 * Setup PDMA_DSCT struct
	 * Scatter-Gather mode, Next to sg descriptor
	 */
	head->CTL = SG_MODE;
	head->NEXT = (uint32_t)desc;

	/*
	 * Setup first descriptor
	 * Transfer width: 8 bit
	 * Source Address: fixed
	 * Destination Address: increment
	 * Single request type, Scatter-gather mode
	 */
	if (len > 1) {
		desc->CTL = ((len - 2) << 16) | TXWIDTH_8 | SAINC_1 | DAINC_FIXED |
			     REQ_TYPE_SINGLE | SG_MODE;
		desc->ENDSA = (uint32_t)buf;
		desc->ENDDA = (uint32_t)&config->base->WDATAB1;
		desc->NEXT = (uint32_t)desc + 16;
		desc++;
	}

	/*
	 * Setup last descriptor for last byte destinated to WDATABE
	 * Transfer width: 8 bit
	 * Source Address: fixed
	 * Destination Address: increment
	 * Single request type, basic mode
	 */
	desc->CTL = (0 << 16) | TXWIDTH_8 | SAINC_1 | DAINC_FIXED |
		     REQ_TYPE_SINGLE | BASIC_MODE;
	desc->ENDSA = (uint32_t)&buf[len - 1];
	desc->ENDDA = (uint32_t)&config->base->WDATABE;
	desc->NEXT = 0;

	obj->txlen = len;

	/* Enable PDMA TX channel */
	config->pdma_base->CHCTL |= 1 << config->dma_tx_channel;

	/*
	 * Setup I3C DMACTRL
	 * transfer width: 1 byte
	 * direction: DMA to bus
	 */
	config->base->DMACTRL = (val | DMACTRL_DMATB_EN);

	return 0;
}

static int i3c_npcm4_slave_write_fifo(struct i3c_reg *base, uint8_t *buf, int len)
{
	int i;

	for (i = 0; i < len - 1; i++)
		base->WDATAB = buf[i];

	/* Write last byte */
	base->WDATABE = buf[len - 1];

	LOG_DBG("\nwrite %d bytes to fifo done\n", len);
	return 0;
}

static int i3c_npcm4_slave_write(const struct device *dev, uint8_t *buf, int len)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);

	if (len <= 0 || len > DMA_BUF_SIZE)
		return -EINVAL;

	LOG_DBG("len %d", len);
	/* Flush TX FIFO */
	config->base->DATACTRL = BIT(0);

	/* Write to FIFO directly if data length is less than FIFO space */
	if (len <= I3C_NPCM4_FIFO_SIZE) {
		obj->use_dma_tx = false;
		return i3c_npcm4_slave_write_fifo(config->base, buf, len);
	}

	/* Start the DMA write operation */
	obj->use_dma_tx = true;
	i3c_npcm4_dma_write(dev, buf, len);

	return 0;
}

static int i3c_npcm4_slave_generate_ibi(const struct device *dev, uint8_t *payload, int len)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	uint32_t ctrl_val, ibiext_val;
	int i;

	if (len > 8) {
		LOG_ERR("IBI data too large");
		return -EINVAL;
	}
	LOG_DBG("generate IBI payload, len = %d", len);
	ctrl_val = (payload[0] << 8) | BIT(0); /* IBI event */

	if (len > 4) {
		ibiext_val = 0;
		for (i = 4; i < len; i++)
			ibiext_val |= (payload[i] << ((i - 4) * 8));
		config->base->IBIEXT2 = ibiext_val;
	}
	if (len > 1) {
		ctrl_val |= BIT(3); /* Has extended IBI data */
		ibiext_val = len - 1;
		for (i = 1; i < 4; i++)
			ibiext_val |= (payload[i] << (i * 8));
		config->base->IBIEXT1 = ibiext_val;
	}
	config->base->CTRL = ctrl_val;

	return 0;
}

static int i3c_npcm4_isr_rx_done(const struct device *dev)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	struct dsct_reg *desc = (struct dsct_reg *)&config->pdma_base->DSCT[config->dma_rx_channel];
	struct i3c_reg *reg = config->base;
	const struct i3c_slave_callbacks *cb;
	struct i3c_slave_payload *payload;
	uint32_t val;
	int rxcnt;

	if (!obj->rx_desc) {
		LOG_WRN("no rx desc");
		return 0;
	}

	/* Check MATCHED */
	val = reg->STATUS;
	if ((val & STATUS_MATCHED) == 0) {
		LOG_DBG("Not matched: Status=0x%08x", val);
		return 0;
	}
	reg->STATUS = STATUS_MATCHED;

	/* Wait for RX FIFO empty */
	if (readl_poll_timeout((uint32_t)&reg->STATUS, STATUS_RXPEND, 0, 1, 0, true) != 0) {
		LOG_WRN("rx FIFO is not empty");
		return -EFAULT;
	}
	val = config->pdma_base->TDSTS;
	if (val & (1 << config->dma_rx_channel))
		config->pdma_base->TDSTS = (1 << config->dma_rx_channel);
	val = desc->CTL;
	val = FIELD_GET(GENMASK(29, 16), desc->CTL) + 1;
	rxcnt = DMA_BUF_SIZE - val;
	if (rxcnt > 0) {
		LOG_DBG("\nRX DONE(%d bytes)\n", rxcnt);
		obj->rx_desc = NULL;
		i3c_npcm4_stop_dma_rx(dev);
	} else {
		LOG_DBG("\nno rx data\n");
		return 0;
	}

	cb = obj->slave_data.callbacks;
	if (cb->write_requested) {
		payload = cb->write_requested(obj->slave_data.dev);
		payload->size = rxcnt;
		memcpy(payload->buf, obj->dma_buf, rxcnt);
	}

	if (cb->write_done) {
		cb->write_done(obj->slave_data.dev);
	}
	if (!obj->rx_desc) {
		/* Start a new dma operation for next transfer */
		i3c_npcm4_dma_read(dev);
	}

	return 0;
}

static int i3c_npcm4_isr_tx_done(const struct device *dev)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	struct dsct_reg *desc = (struct dsct_reg *)&config->pdma_base->DSCT[config->dma_tx_channel];
	uint32_t val, tx_done_flag;
	int txcnt;
	int ret = 0;

	config->base->STATUS = STATUS_MATCHED;
	if (!obj->use_dma_tx) {
		val = config->base->DATACTRL;
		if (val & GENMASK(20, 16))
			LOG_WRN("TX FIFO is not empty");
		LOG_DBG("\nFIFO TX DONE\n");
		obj->state &= ~SLAVE_WAIT_FOR_TX;
		k_sem_give(&obj->complete);
		return 0;
	}
	/* Wait for Transfer Done flag */
	tx_done_flag = BIT(config->dma_tx_channel);
	if (readl_poll_timeout((uint32_t)&config->pdma_base->TDSTS, tx_done_flag, tx_done_flag,
				1, 0, true) != 0) {
		LOG_WRN("DMA transfer is not finished");
		ret = -ETIMEDOUT;
		goto err_quit;
	}
	/* Clear flag */
	config->pdma_base->TDSTS = (1 << config->dma_tx_channel);

	val = desc->CTL;
	val = FIELD_GET(GENMASK(29, 16), desc->CTL);
	txcnt = obj->txlen - val;
	if (txcnt > 0) {
		LOG_DBG("\nDMA TX DONE(%d bytes)\n", txcnt);
		obj->state &= ~SLAVE_WAIT_FOR_TX;
		obj->tx_desc = NULL;
		i3c_npcm4_stop_dma_tx(config);
		k_sem_give(&obj->complete);

		return 0;
	}
	LOG_WRN("tx count is 0");

err_quit:
	obj->state &= ~SLAVE_WAIT_FOR_TX;
	obj->tx_desc = NULL;
	i3c_npcm4_stop_dma_tx(config);
	k_sem_give(&obj->complete);

	return ret;
}

static int i3c_npcm4_isr_check_ibi_done(const struct device *dev)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	struct i3c_reg *reg = config->base;

	/* Wait for EVENT status */
	if (readl_poll_timeout((uint32_t)&reg->STATUS, BIT(18), BIT(18), 1, 0, true) != 0) {
		LOG_WRN("No Event requested");
		return 0;
	}
	LOG_DBG("\nIBI done, event=%d\n", (int)FIELD_GET(GENMASK(21, 20), reg->STATUS));
	/* Clear EVENT status */
	reg->STATUS = STATUS_EVENT;
	obj->state &= ~SLAVE_REQUEST_IBI;
	if (obj->state & SLAVE_TX_READY) {
		obj->state &= ~SLAVE_TX_READY;
		obj->state |= SLAVE_WAIT_FOR_TX;
	}

	return 0;
}

static int i3c_npcm4_isr_receive_stop(const struct device *dev)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	struct i3c_reg *reg = config->base;
	uint32_t val;

	LOG_DBG("Received STOP: state=0x%x\n", obj->state);
	val = reg->STATUS;
	if ((val & STATUS_MATCHED) == 0 && (obj->state & SLAVE_REQUEST_IBI) == 0) {
		LOG_DBG("Address not matched: Status=0x%08x", val);
		return 0;
	}
	if ((obj->state & SLAVE_WAIT_FOR_RX) && obj->rx_desc) {
		i3c_npcm4_isr_rx_done(dev);
	}
	if (obj->state & SLAVE_WAIT_FOR_TX) {
		i3c_npcm4_isr_tx_done(dev);
	}
	if (obj->state & SLAVE_REQUEST_IBI) {
		i3c_npcm4_isr_check_ibi_done(dev);
	}

	return 0;
}

static int i3c_npcm4_isr_da_changed(const struct device *dev)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	uint8_t addr = 0;

	i3c_npcm4_slave_get_dynamic_addr(dev, &addr);
	LOG_WRN("DA changed: 0x%x", addr);
	config->base->STATUS = STATUS_STOP;

	if (addr) {
		obj->state |= SLAVE_DA_ASSIGNED;
		if (!obj->rx_desc)
			i3c_npcm4_dma_read(dev);
		i3c_npcm4_register_isr_cb(dev, IRQ_STOP, i3c_npcm4_isr_receive_stop);
	} else {
		obj->state &= ~SLAVE_DA_ASSIGNED;
		i3c_npcm4_register_isr_cb(dev, IRQ_STOP, NULL);
	}

	return 0;
}

/* Master mode API implementations */

/**
 * @brief Attach an I3C device to the controller
 */
int i3c_npcm4_master_attach_device(const struct device *dev, struct i3c_dev_desc *slave)
{

	return 0;
}

/**
 * @brief Detach an I3C device from the controller
 */
int i3c_npcm4_master_detach_device(const struct device *dev, struct i3c_dev_desc *slave)
{

	return 0;
}

/**
 * @brief Send CCC (Common Command Code) to I3C device(s)
 */
int i3c_npcm4_master_send_ccc(const struct device *dev, struct i3c_ccc_cmd *ccc)
{
	return 0;
}

/**
 * @brief Perform private I3C transfer
 */
int i3c_npcm4_master_priv_xfer(struct i3c_dev_desc *i3cdev, struct i3c_priv_xfer *xfers,
			       int nxfers)
{
	return 0;
}

/**
 * @brief Request IBI (In-Band Interrupt) from slave device
 */
int i3c_npcm4_master_request_ibi(struct i3c_dev_desc *i3cdev, struct i3c_ibi_callbacks *cb)
{

	return 0;
}

/**
 * @brief Enable IBI for specific device
 */
int i3c_npcm4_master_enable_ibi(struct i3c_dev_desc *i3cdev)
{

	return 0;
}

/**
 * @brief Send ENTDAA (Enter Dynamic Address Assignment)
 */
int i3c_npcm4_master_send_entdaa(struct i3c_dev_desc *i3cdev)
{

	return 0;
}

/* Slave mode API implementations */

/**
 * @brief Register as I3C slave device
 */
int i3c_npcm4_slave_register(const struct device *dev, struct i3c_slave_setup *slave_data)
{
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct i3c_reg *reg = config->base;
	uint32_t status;
	uint8_t addr;

	obj->slave_data.max_payload_len = slave_data->max_payload_len;
	obj->slave_data.callbacks = slave_data->callbacks;
	obj->slave_data.dev = slave_data->dev;

	/* Clear status */
	status = reg->STATUS;
	reg->STATUS = status;

	if (i3c_npcm4_slave_get_dynamic_addr(dev, &addr) == 0)
		obj->state |= SLAVE_DA_ASSIGNED;
	else
		obj->state &= ~SLAVE_DA_ASSIGNED;

	i3c_npcm4_register_isr_cb(dev, IRQ_DACHG, i3c_npcm4_isr_da_changed);

	return 0;
}

/**
 * @brief Prepare read data for master read transfer
 */
int i3c_npcm4_slave_put_read_data(const struct device *dev, struct i3c_slave_payload *data,
				  struct i3c_ibi_payload *ibi_notify)
{
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct i3c_reg *reg = config->base;

	__ASSERT_NO_MSG(data);
	__ASSERT_NO_MSG(data->buf);
	__ASSERT_NO_MSG(data->size);

	LOG_DBG("put data");
	/* Wait for bus STOP */
	if (readl_poll_timeout((uint32_t)&reg->STATUS, BIT(0), 0, 10, 0, false) != 0) {
		LOG_ERR("bus is busy");
		return -EBUSY;
	}

	if (i3c_npcm4_slave_write(dev, data->buf, data->size) == 0)
		obj->state |= SLAVE_TX_READY;
	k_sem_init(&obj->complete, 0, 1);

	if (ibi_notify) {
		reg->STATUS = STATUS_EVENT;
		obj->state |= SLAVE_REQUEST_IBI;
		i3c_npcm4_slave_generate_ibi(dev, ibi_notify->buf, ibi_notify->size);
	}
	k_sem_take(&obj->complete, K_FOREVER);
	LOG_DBG("complete");

	return 0;
}

/**
 * @brief Get dynamically assigned address in slave mode
 */
int i3c_npcm4_slave_get_dynamic_addr(const struct device *dev, uint8_t *dynamic_addr)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	uint32_t val;

	val = config->base->DYNADDR;
	if ((val & BIT(0)) == 0) {
		LOG_DBG("no invalid dynamic addr");
		return -EINVAL;
	}

	*dynamic_addr = (uint8_t)(FIELD_GET(GENMASK(7, 1), val));
	LOG_DBG("dynaddr=0x%x", *dynamic_addr);

	return 0;
}

/**
 * @brief Get event enabling status in slave mode
 */
int i3c_npcm4_slave_get_event_enabling(const struct device *dev, uint32_t *event_en)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	uint32_t status;

	status = config->base->STATUS;

	*event_en = 0;
	if ((status & STATUS_IBIDIS) == 0)
		*event_en |= I3C_SLAVE_EVENT_SIR;
	if ((status & STATUS_MRDIS) == 0)
		*event_en |= I3C_SLAVE_EVENT_MR;
	if ((status & STATUS_HJDIS) == 0)
		*event_en |= I3C_SLAVE_EVENT_HJ;

	LOG_DBG("event_en = 0x%x", *event_en);

	return 0;
}

/**
 * @brief Send SIR (Slave Interrupt Request) with payload
 */
int i3c_npcm4_slave_send_sir(const struct device *dev, struct i3c_ibi_payload *payload)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct i3c_reg *reg = config->base;
	uint8_t addr = 0;
	uint32_t val;

	LOG_DBG("Send IBI request");
	if (i3c_npcm4_slave_get_dynamic_addr(dev, &addr) < 0) {
		LOG_ERR("No valid Dynamic address\n");
		return -EINVAL;
	}

	val = reg->STATUS;
	if (val & BIT(24)) {
		LOG_ERR("IBI request is disabled\n");
		return -EINVAL;
	}

	/* Wait for bus STOP */
	if (readl_poll_timeout((uint32_t)&reg->STATUS, BIT(0), 0, 10, 0, false) != 0) {
		LOG_ERR("bus is busy");
		return -EBUSY;
	}

	i3c_npcm4_slave_generate_ibi(dev, payload->buf, payload->size);

	return 0;
}

/**
 * @brief Send Hot-Join request in slave mode
 */
static void i3c_npcm4_hj_timeout_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct i3c_npcm4_obj *obj =
		CONTAINER_OF(dwork, struct i3c_npcm4_obj, hj_timeout_work);
	const struct i3c_npcm4_config *config = DEV_CFG(obj->dev);
	struct i3c_reg *reg = config->base;

	LOG_ERR("HJ request not sent within timeout, cancelling");
	i3c_npcm4_register_isr_cb(obj->dev, IRQ_EVENT, NULL);
	/* Cancel HJ request: clear CTRL bits[1:0] */
	reg->CTRL &= ~GENMASK(1, 0);
}

static int i3c_npcm4_isr_hj_event(const struct device *dev)
{
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct i3c_reg *reg = config->base;
	uint32_t event = (uint32_t)FIELD_GET(GENMASK(21, 20), reg->STATUS);

	/* Clear EVENT status */
	reg->STATUS = STATUS_EVENT;

	/* HJ request sent: cancel the timeout work */
	k_work_cancel_delayable(&obj->hj_timeout_work);
	i3c_npcm4_register_isr_cb(dev, IRQ_EVENT, NULL);

	if ((event & 0x2) == 0)
		LOG_ERR("HJ request not transmitted, event=0x%x", event);
	else
		LOG_DBG("HJ request sent, event=0x%x", event);

	return 0;
}

int i3c_npcm4_slave_hj_req(const struct device *dev)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	struct i3c_reg *reg = config->base;
	uint8_t addr = 0;

	LOG_INF("Send HJ req");
	if (i3c_npcm4_slave_get_dynamic_addr(dev, &addr) == 0) {
		LOG_ERR("Dynamic address present = 0x%x\n", addr);
		return -EINVAL;
	}

	/* Wait for bus STOP */
	if (readl_poll_timeout((uint32_t)&reg->STATUS, BIT(0), 0, 10, 0, false) != 0) {
		LOG_ERR("bus is busy");
		return -EBUSY;
	}
	if (config->hj_timeout_ms > 0) {
		/* Register EVENT ISR callback and schedule timeout */
		i3c_npcm4_register_isr_cb(dev, IRQ_EVENT, i3c_npcm4_isr_hj_event);
		k_work_schedule(&obj->hj_timeout_work, K_MSEC(config->hj_timeout_ms));
	}

	reg->CTRL |= 0x3;

	return 0;
}

/**
 * @brief Set static address in slave mode
 */
int i3c_npcm4_slave_set_static_addr(const struct device *dev, uint8_t static_addr)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct i3c_reg *reg = config->base;
	uint32_t val;

	/* Update bit field [31:25] in the CONFIG register */
	LOG_INF("change static addr to 0x%x", static_addr);
	val = reg->CONFIG & ~GENMASK(31, 25);
	val |= FIELD_PREP(GENMASK(31, 25), static_addr);
	reg->CONFIG = val;

	return 0;
}

/**
 * @brief Set PID extra info
 */
int i3c_npcm4_set_pid_extra_info(const struct device *dev, uint16_t extra_info)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct i3c_reg *reg = config->base;
	uint32_t val;

	/* Update bit field [11:0] in the PARTNO register */
	LOG_INF("change PID extra_info to 0x%04x", extra_info);
	val = reg->PARTNO & ~GENMASK(11, 0);
	val |= FIELD_PREP(GENMASK(11, 0), extra_info);
	reg->PARTNO = val;

	return 0;
}

static void i3c_npcm4_slave_isr(const struct device *dev)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	struct i3c_reg *reg = config->base;
	uint32_t status = reg->INTMASKED;
	int i;

	LOG_DBG("status=0x%x", reg->STATUS);
	for (i = IRQ_START; i <= IRQ_EVENT; i++) {
		if (status & (1 << i)) {
			/* Clear status bit */
			reg->STATUS = 1 << i;
			if (obj->isr_cb[(i - IRQ_START)])
				obj->isr_cb[(i - IRQ_START)](dev);
		}
	}
}

static void i3c_npcm4_isr(const struct device *dev)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);

	if (config->slave)
		return i3c_npcm4_slave_isr(dev);
}


static int i3c_npcm4_slave_init(const struct device *dev,
				const struct i3c_npcm4_config *config)
{
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	struct i3c_reg *reg = config->base;
	uint32_t val;

	LOG_DBG("bcr=0x%x, dcr=0x%x", config->bcr, config->dcr);

	obj->rx_desc = NULL;
	obj->tx_desc = NULL;

	obj->state = 0;
	/* Setup slave BCR/DCR/PID */
	reg->IDEXT = (config->bcr << 16) | (config->dcr << 8);
	reg->PARTNO = (config->part_id << 16) | (config->vendor_def_id);

	/* Setup static address and enable slave mode */
	val = FIELD_PREP(GENMASK(31, 25), config->assigned_addr);
	val |= FIELD_PREP(GENMASK(22, 16), (obj->apb3_rate / 1000000UL));
	val |= BIT(0); /* Enable */
	reg->CONFIG = val;

	return 0;
}

static int i3c_npcm4_setup_dma(const struct device *dev)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	uint32_t *PDMA_REQSEL;
	uint32_t val, shift;

	if (config->dma_tx_channel > 15 || config->dma_rx_channel > 15) {
		LOG_ERR("Invalid DMA channel");
		return -EINVAL;
	}

	LOG_DBG("setup dma: tx chanel %d, rx channel %d", config->dma_tx_channel, config->dma_rx_channel);

	/* Select I3C module for DMA TX channel */
	PDMA_REQSEL = (uint32_t *)&config->pdma_base->REQSEL0_3 + (config->dma_tx_channel / 4);
	shift = (config->dma_tx_channel % 4) * 8;
	val = *PDMA_REQSEL & ~(0xFF << shift);
	val |= (6 + config->inst_id * 2) << shift;
	*PDMA_REQSEL = val;

	/* Select I3C module for DMA RX channel */
	PDMA_REQSEL = (uint32_t *)&config->pdma_base->REQSEL0_3 + (config->dma_rx_channel / 4);
	shift = (config->dma_rx_channel % 4) * 8;
	val = *PDMA_REQSEL & ~(0xFF << shift);
	val |= (5 + config->inst_id * 2) << shift;
	*PDMA_REQSEL = val;

	/* Setup dsct base address */
	config->pdma_base->SCATBA = (uint32_t)sg_dsct;
	obj->dma_buf = &dma_buf_pool[DMA_BUF_SIZE * config->inst_id];
	return 0;
}

/* Device initialization */
static int i3c_npcm4_init(const struct device *dev)
{
	const struct i3c_npcm4_config *config = DEV_CFG(dev);
	struct i3c_npcm4_obj *obj = DEV_DATA(dev);
	const struct device *const clk_dev = device_get_binding(NPCM4XX_CLK_CTRL_NAME);
	uint32_t apb3_rate;
	int ret;

	LOG_WRN("Built at:   %s %s", __DATE__, __TIME__);

	/* Turn on device clock first and get source clock freq. */
	ret = clock_control_on(clk_dev, (clock_control_subsys_t *)
			&config->clk_cfg);
	if (ret < 0) {
		LOG_ERR("Turn on I3C clock fail %d", ret);
		return ret;
	}
	ret = clock_control_get_rate(clk_dev, (clock_control_subsys_t *)
			&config->clk_cfg, &apb3_rate);
	if (ret < 0) {
		LOG_ERR("Get I3C clock rate error %d", ret);
		return ret;
	}
	obj->apb3_rate = apb3_rate;
	obj->dev = dev;
	k_work_init_delayable(&obj->hj_timeout_work, i3c_npcm4_hj_timeout_handler);

	ret = i3c_npcm4_setup_dma(dev);
	if (ret < 0) {
		LOG_ERR("i3c_npcm4_setup_dma fail %d", ret);
		return ret;
	}

	if (config->slave) {
		ret = i3c_npcm4_slave_init(dev, config);
		if (ret < 0) {
			LOG_ERR("i3c_npcm4_slave_init fail %d", ret);
			return ret;
		}
	}

	LOG_INF("NPCM4 I3C initialized successfully");
	return 0;
}

/* Device instantiation macro */
#define I3C_NPCM4_INIT(n) static int i3c_npcm4_config_func_##n(const struct device *dev);\
	static const struct i3c_npcm4_config i3c_npcm4_config_##n = {\
		.inst_id = DT_INST_PROP_OR(n, instance_id, 0),\
		.assigned_addr = DT_INST_PROP_OR(n, assigned_address, 0),\
		.slave = DT_INST_PROP_OR(n, slave, 0),\
		.secondary = DT_INST_PROP_OR(n, secondary, 0),\
		.bcr = DT_INST_PROP_OR(n, bcr, 0),\
		.dcr = DT_INST_PROP_OR(n, dcr, 0),\
		.part_id = DT_INST_PROP_OR(n, part_id, 0),\
		.vendor_def_id = DT_INST_PROP_OR(n, vendor_def_id, 0),\
		.dma_tx_channel = DT_INST_PROP_OR(n, dma_tx_channel, 0xff),\
		.dma_rx_channel = DT_INST_PROP_OR(n, dma_rx_channel, 0xff),\
		.i2c_scl_hz = DT_INST_PROP_OR(n, i2c_scl_hz, 0),\
		.i3c_scl_hz = DT_INST_PROP_OR(n, i3c_scl_hz, 0),\
		.hj_timeout_ms = DT_INST_PROP_OR(n, hj_request_timeout_ms, -1),\
		.base = (struct i3c_reg *)DT_INST_REG_ADDR_BY_NAME(n, i3c),\
		.pdma_base = (struct pdma_reg *)DT_INST_REG_ADDR_BY_NAME(n, pdma),\
		.pmc_base = DT_INST_REG_ADDR_BY_NAME(n, pmc),\
		.clk_cfg = NPCM4XX_DT_CLK_CFG_ITEM(n),\
		.irq = DT_INST_IRQN(n),\
	};\
	static struct i3c_npcm4_obj i3c_npcm4_obj##n;\
	DEVICE_DT_INST_DEFINE(n, &i3c_npcm4_config_func_##n, NULL, &i3c_npcm4_obj##n,\
			      &i3c_npcm4_config_##n, POST_KERNEL,\
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL);\
	static int i3c_npcm4_config_func_##n(const struct device *dev)\
	{\
		int ret;\
		ret = i3c_npcm4_init(dev);\
		if (ret < 0) \
			return ret;\
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), i3c_npcm4_isr,\
			DEVICE_DT_INST_GET(n), 0);\
		irq_enable(DT_INST_IRQN(n));\
		return 0;\
	}

DT_INST_FOREACH_STATUS_OKAY(I3C_NPCM4_INIT)
