/*
 * Copyright (c) 2022 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aspeed_swmbx

#include <sys/util.h>
#include <sys/slist.h>
#include <kernel.h>
#include <errno.h>
#include <drivers/i2c.h>
#include <string.h>
#include <drivers/i2c/slave/swmbx.h>

#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(i2c_slave_swmbx);

struct i2c_swmbx_fifo {
	sys_snode_t list;
	uint8_t value;
};

struct i2c_swmbx_notify {
	struct k_sem *sem_notify;
	uint8_t address;
	uint8_t enable;
};

struct i2c_swmbx_fifo_data {
	struct k_sem *sem_fifo;
	struct i2c_swmbx_fifo *buffer;
	struct i2c_swmbx_fifo *current;
	sys_slist_t list_head;
	uint8_t fifo_offset;
	uint8_t enable;
	uint32_t msg_index;
	uint32_t max_msg_count;

	uint32_t volatile cur_msg_count;
};

struct i2c_swmbx_fifo_config {
	const struct device *parent;
	char *fifo_label;
	uint8_t fifo_max_size;
	uint8_t fifo_offset;
};

struct i2c_swmbx_slave_data {
	const struct device *i2c_controller;
	struct i2c_slave_config config;
	uint32_t bus_base;
	uint8_t bus_count;
	uint32_t *mbx_info;
	uint32_t buffer_size;
	uint8_t *buffer;
	uint32_t buffer_idx;
	bool first_write;
	uint8_t mbx_en;
	uint8_t mbx_notify_idx;
	uint32_t mbx_protect[SWMBX_PROTECT_COUNT];
	struct i2c_swmbx_notify notify[SWMBX_NOTIFY_COUNT];
	struct i2c_swmbx_fifo_data fifo[SWMBX_FIFO_COUNT];
	const struct i2c_swmbx_fifo_config *fifo_cfg;
	struct i2c_swmbx_fifo_data *fifo_data;
	uint8_t fifo_count;
};

struct i2c_swmbx_slave_config {
	char *controller_dev_name;
	uint32_t bus_base;
	uint8_t address;
	uint32_t buffer_size;
	uintptr_t buffer;
};

/* convenience defines */
#define DEV_CFG(dev)		\
	((const struct i2c_swmbx_slave_config *const)	\
	(dev)->config)
#define DEV_DATA(dev)	\
	((struct i2c_swmbx_slave_data *const)(dev)->data)
#define DEV_F_CFG(dev)	\
	((const struct i2c_swmbx_fifo_config *const)(dev)->config)
#define DEV_F_DATA(dev)	\
	((struct i2c_swmbx_fifo_data *const)(dev)->data)

int swmbx_write(const struct device *dev, uint8_t index, uint8_t *val)
{
	if (dev == NULL)
		return -EINVAL;

	struct i2c_swmbx_slave_data *data = dev->data;

	data->buffer[index] = *val;

	return 0;
}

int swmbx_read(const struct device *dev, uint8_t index, uint8_t *val)
{
	if (dev == NULL)
		return -EINVAL;

	struct i2c_swmbx_slave_data *data = dev->data;

	*val = data->buffer[index];

	return 0;
}

/* general control */
int swmbx_enable_behavior(const struct device *dev, uint32_t item_flag, uint8_t enable)
{
	if ((dev == NULL) || ((item_flag & FLAG_MASK) == 0))
		return -EINVAL;

	struct i2c_swmbx_slave_data *data = dev->data;

	data->mbx_en = item_flag;

	return 0;
}

/* swmbx write protect */
int swmbx_apply_protect(const struct device *dev, uint32_t *bitmap, uint8_t start_idx, uint8_t num)
{
	if ((dev == NULL) || (bitmap == NULL) ||
	(start_idx + num) > (SWMBX_PROTECT_COUNT))
		return -EINVAL;

	struct i2c_swmbx_slave_data *data = dev->data;

	for (uint8_t i = start_idx; i < (start_idx + num); i++)
		data->mbx_protect[i] = bitmap[i];

	return 0;
}

int swmbx_update_protect(const struct device *dev, uint8_t addr, uint8_t enable)
{
	if (dev == NULL)
		return -EINVAL;

	struct i2c_swmbx_slave_data *data = dev->data;
	uint8_t index, bit;
	uint32_t value;

	/* calculte bitmap position */
	index = addr / 0x20;
	bit = addr % 0x20;

	value = data->mbx_protect[index];

	if (enable)
		value |= (0x1 << bit);
	else
		value = value & ~(0x1 << bit);

	data->mbx_protect[index] = value;

	return 0;
}

int swmbx_update_notify(const struct device *dev, struct k_sem *sem,
uint8_t idx, uint8_t addr, uint8_t enable)
{
	if ((dev == NULL) || idx > (SWMBX_NOTIFY_COUNT-1))
		return -EINVAL;

	struct i2c_swmbx_slave_data *data = dev->data;

	/* check enable or not */
	if (enable) {
		if ((data->notify[idx].enable) || sem == NULL)
			return -EINVAL;

		data->notify[idx].sem_notify = sem;
		data->notify[idx].address = addr;
	} else {
		data->notify[idx].sem_notify = NULL;
	}

	data->notify[idx].enable = enable;

	return 0;
}

int swmbx_update_fifo(const struct device *dev, struct k_sem *sem,
uint8_t idx, uint8_t addr, uint8_t depth, uint8_t enable)
{
	if ((dev == NULL) || idx > (SWMBX_FIFO_COUNT-1))
		return -EINVAL;

	struct i2c_swmbx_slave_data *data = dev->data;
	sys_snode_t *list_node = NULL;

	/* check enable or not */
	if (enable) {
		if ((data->fifo[idx].enable) || depth == 0 || sem == NULL)
			return -EINVAL;

		/* malloc the message buffer */
		data->fifo[idx].buffer = k_malloc(sizeof(struct i2c_swmbx_fifo) * depth);
		if (!data->fifo[idx].buffer) {
			data->fifo[idx].enable = 0x0;
			LOG_ERR("swmbx could not alloc enougth fifo buffer");
			return -EINVAL;
		}

		/* initial single list structure*/
		sys_slist_init(&(data->fifo[idx].list_head));

		data->fifo[idx].sem_fifo = sem;
		data->fifo[idx].max_msg_count = depth;
		data->fifo[idx].fifo_offset = addr;
		data->fifo[idx].msg_index = 0;
		data->fifo[idx].cur_msg_count = 0;
	} else {
		if (data->fifo[idx].buffer) {
			/* free link list */
			do {
				list_node = sys_slist_peek_head(&(data->fifo[idx].list_head));
				if (list_node != NULL) {
					LOG_DBG("swmbx: slave drop fifo %x", (uint32_t)list_node);
					/* remove this item from list */
					sys_slist_find_and_remove(&(data->fifo[idx].list_head), list_node);
				}
			} while (list_node != NULL);

			/* free memory */
			k_free(data->fifo[idx].buffer);
			data->fifo[idx].buffer = NULL;
		}

		data->fifo[idx].sem_fifo = NULL;
	}

	data->fifo[idx].enable = enable;

	return 0;
}

/* internal api for pfr sw mbx control */
int check_swmbx_notify(struct i2c_swmbx_slave_data *data, uint8_t addr)
{
	for (uint8_t i = 0; i < SWMBX_NOTIFY_COUNT; i++) {
		if ((data->notify[i].address == addr) && (data->notify[i].enable)
		&& (data->notify[i].sem_notify)) {
			data->mbx_notify_idx = i;
			return 1;
		}
	}

	return 0;
}

int check_swmbx_protect(struct i2c_swmbx_slave_data *data, uint8_t addr)
{
	uint8_t index, bit;
	uint32_t value;

	/* calculte bitmap position */
	index = addr / 0x20;
	bit = addr % 0x20;

	value = data->mbx_protect[index];

	if (value & (0x1 << bit))
		return 1;

	return 0;
}

/* turn on / off other swmbx support */
void turn_swmbx_slave(struct i2c_swmbx_slave_data *data, uint8_t on)
{
	uint8_t i = 0, address = 0;
	uint32_t i2c_base = SWMBX_INFO_BASE & 0xFFFF0000;
	uint32_t *swmbx_g_info = (uint32_t *)(SWMBX_INFO_BASE);

	for (i = 0; i < SWMBX_DEVICE_COUNT; i++) {
		/* affect the other device */
		if (*(swmbx_g_info + i) & SWMBX_REGISTER) {
			i2c_base += (i + 1) * 0x80;

			if (on)
				address = *(swmbx_g_info + i) & 0xFF;
			else
				address = 0;

			if (i2c_base != data->bus_base) {
				/*Set slave addr.*/
				sys_write32(address |
				(sys_read32(i2c_base + AST_I2CS_ADDR_CTRL)
				& ~AST_I2CS_ADDR1_MASK), i2c_base + AST_I2CS_ADDR_CTRL);
			}
		}
	}
}

int swmbx_slave_read(const struct device *dev, uint8_t *swmbx_data,
		      unsigned int offset)
{
	struct i2c_swmbx_slave_data *data = dev->data;

	if (!data || offset >= data->buffer_size) {
		return -EINVAL;
	}

	*swmbx_data = data->buffer[offset];

	return 0;
}

static int swmbx_slave_write_requested(struct i2c_slave_config *config)
{
	struct i2c_swmbx_slave_data *data = CONTAINER_OF(config,
						struct i2c_swmbx_slave_data,
						config);

	LOG_DBG("swmbx: write req");

	/* turn off other swmbx */
	turn_swmbx_slave(data, 0x0);

	data->first_write = true;

	return 0;
}

static int swmbx_slave_read_requested(struct i2c_slave_config *config,
				       uint8_t *val)
{
	struct i2c_swmbx_slave_data *data = CONTAINER_OF(config,
						struct i2c_swmbx_slave_data,
						config);

	/* turn off other swmbx */
	turn_swmbx_slave(data, 0x0);

	*val = data->buffer[data->buffer_idx];

	LOG_DBG("swmbx: read req, val=0x%x", *val);

	/* Increment will be done in the read_processed callback */

	return 0;
}

static int swmbx_slave_write_received(struct i2c_slave_config *config,
				       uint8_t val)
{
	struct i2c_swmbx_slave_data *data = CONTAINER_OF(config,
						struct i2c_swmbx_slave_data,
						config);

	LOG_DBG("swmbx: write done, val=0x%x", val);

	if (data->first_write) {
		data->buffer_idx = val;
		data->first_write = false;
	} else {
		/* check write protect behavior */
		if (!check_swmbx_protect(data, data->buffer_idx) ||
		!(data->mbx_en & SWMBX_PROTECT))
			data->buffer[data->buffer_idx] = val;

		/* check notify behavior */
		if (data->mbx_en & SWMBX_NOTIFY) {
			if (check_swmbx_notify(data, data->buffer_idx)) {
				k_sem_give(data->notify[data->mbx_notify_idx].sem_notify);
			}
		}

		data->buffer_idx++;
	}

	data->buffer_idx = data->buffer_idx % data->buffer_size;

	return 0;
}

static int swmbx_slave_read_processed(struct i2c_slave_config *config,
				       uint8_t *val)
{
	struct i2c_swmbx_slave_data *data = CONTAINER_OF(config,
						struct i2c_swmbx_slave_data,
						config);

	/* Increment here */
	data->buffer_idx = (data->buffer_idx + 1) % data->buffer_size;

	*val = data->buffer[data->buffer_idx];

	LOG_DBG("swmbx: read done, val=0x%x", *val);

	/* Increment will be done in the next read_processed callback
	 * In case of STOP, the byte won't be taken in account
	 */

	return 0;
}

static int swmbx_slave_stop(struct i2c_slave_config *config)
{
	struct i2c_swmbx_slave_data *data = CONTAINER_OF(config,
						struct i2c_swmbx_slave_data,
						config);

	LOG_DBG("swmbx: stop");

	/* turn on other swmbx */
	turn_swmbx_slave(data, 0x1);

	data->first_write = true;

	return 0;
}

static int swmbx_slave_register(const struct device *dev)
{
	struct i2c_swmbx_slave_data *data = dev->data;
	uint8_t i;

	*(data->mbx_info) |= SWMBX_REGISTER;

	/* initial data structure value */
	for (i = 0; i < SWMBX_PROTECT_COUNT; i++) {
		data->mbx_protect[i] = 0;
		data->notify[i].enable = 0;
		data->notify[i].sem_notify = NULL;
		data->fifo[i].enable = 0;
		data->fifo[i].buffer = NULL;
		data->fifo[i].sem_fifo = NULL;
	}

	return i2c_slave_register(data->i2c_controller, &data->config);
}

static int swmbx_slave_unregister(const struct device *dev)
{
	struct i2c_swmbx_slave_data *data = dev->data;
	uint8_t i;

	*(data->mbx_info) &= ~(SWMBX_REGISTER);

	/* release fifo memory */
	for (i = 0; i < SWMBX_FIFO_COUNT; i++) {
		swmbx_update_fifo(dev, NULL, i, 0x0, 0x0, 0x0);
	}

	return i2c_slave_unregister(data->i2c_controller, &data->config);
}

static const struct i2c_slave_driver_api swmbx_api_funcs = {
	.driver_register = swmbx_slave_register,
	.driver_unregister = swmbx_slave_unregister,
};

static const struct i2c_slave_callbacks swmbx_callbacks = {
	.write_requested = swmbx_slave_write_requested,
	.read_requested = swmbx_slave_read_requested,
	.write_received = swmbx_slave_write_received,
	.read_processed = swmbx_slave_read_processed,
	.stop = swmbx_slave_stop,
};

static int i2c_swmbx_slave_init(const struct device *dev)
{
	struct i2c_swmbx_slave_data *data = DEV_DATA(dev);
	const struct i2c_swmbx_slave_config *cfg = DEV_CFG(dev);
	uint32_t *swmbx_g_info = (uint32_t *)(SWMBX_INFO_BASE);

	data->i2c_controller =
		device_get_binding(cfg->controller_dev_name);
	if (!data->i2c_controller) {
		LOG_ERR("i2c controller not found: %s",
			    cfg->controller_dev_name);
		return -EINVAL;
	}

	data->buffer_size = cfg->buffer_size;
	data->buffer = (uint8_t *)(cfg->buffer);
	data->bus_base = cfg->bus_base;
	data->config.address = cfg->address;
	data->config.callbacks = &swmbx_callbacks;

	data->bus_count = ((cfg->bus_base & 0xFFFF) / 0x80) - 1;
	data->mbx_info = swmbx_g_info + data->bus_count;
	*(data->mbx_info) = cfg->address;

	return 0;
}

struct fifo_info {
	const struct i2c_swmbx_fifo_config *cfg;
	struct i2c_swmbx_fifo_data *data;
};

#define I2C_SWMBX_INIT(inst)						\
	static struct i2c_swmbx_slave_data				\
		i2c_swmbx_slave_##inst##_dev_data;			\
									\
	static const struct i2c_swmbx_slave_config			\
		i2c_swmbx_slave_##inst##_cfg = {			\
		.controller_dev_name = DT_INST_BUS_LABEL(inst),		\
		.bus_base = DT_REG_ADDR(DT_INST_BUS(inst)),	\
		.address = DT_INST_REG_ADDR(inst),			\
		.buffer_size = DT_INST_PROP(inst, size),		\
		.buffer = DT_INST_PROP(inst, base),			\
	};								\
									\
	DEVICE_DT_INST_DEFINE(inst,					\
			    &i2c_swmbx_slave_init,			\
			    NULL,			\
			    &i2c_swmbx_slave_##inst##_dev_data,	\
			    &i2c_swmbx_slave_##inst##_cfg,		\
			    POST_KERNEL,				\
			    CONFIG_I2C_SLAVE_INIT_PRIORITY,		\
			    &swmbx_api_funcs);

DT_INST_FOREACH_STATUS_OKAY(I2C_SWMBX_INIT)

