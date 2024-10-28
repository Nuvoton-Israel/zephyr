/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_npcm_gpio

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/drivers/gpio/gpio_npcm.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/pinctrl/npcm-pinctrl.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gpio_npcm, CONFIG_GPIO_LOG_LEVEL);

/* GPIO module instances */
#define NPCM_GPIO_DEV(inst) DEVICE_DT_INST_GET(inst),
static const struct device *gpio_devs[] = {DT_INST_FOREACH_STATUS_OKAY(NPCM_GPIO_DEV)};

/* SCFG base */
struct npcm_scfg_config {
	/* scfg device base address */
	uintptr_t base_scfg;
};

static const struct npcm_scfg_config npcm_scfg_cfg = {
	.base_scfg = DT_REG_ADDR_BY_NAME(DT_INST_PARENT(0), scfg),
};

/* Driver convenience defines */
#define HAL_BASE_INST(dev)                                                                         \
	((struct gpio_reg *)((const struct gpio_npcm_config *)(dev)->config)->base)
#define HAL_PORT_INST(dev)   (((const struct gpio_npcm_config *)(dev)->config)->port)
#define HAL_NGPIOS_INST(dev) (((const struct gpio_npcm_config *)(dev)->config)->ngpios)
#define SCFG_INST()          (struct scfg_reg *)(npcm_scfg_cfg.base_scfg)

/* Platform specific GPIO functions */
const struct device *gpio_npcm_dev_get(int port)
{
	uint8_t i;

	for (i = 0; i < ARRAY_SIZE(gpio_devs); i++) {
		if (HAL_PORT_INST(gpio_devs[i]) == port) {
			return gpio_devs[i];
		}
	}

	return NULL;
}

static int gpio_npcm_set_pincfg(const struct device *dev, gpio_pin_t pin)
{
	int port = HAL_PORT_INST(dev);
	uint8_t ngpios = HAL_NGPIOS_INST(dev);
	pinctrl_soc_pin_t pin_cfg;

	/* Check if port and pin are valid */
	if ((port >= NPCM_GPIO_PORT_NUM) || (pin >= ngpios)) {
		return -EINVAL;
	}

	pin_cfg.props.type = NPCM_PINCTRL_TYPE_PERIPH_PINMUX;
	pin_cfg.props.id = NPCM_PINCTRL_NUM_IDX(port, pin);
	pin_cfg.props.group = NPCM_GPIO_PIN_GROUP_DEF_IDX;

	/* Set GPIO with pinctrl API */
	if (pinctrl_configure_pins(&pin_cfg, 1, PINCTRL_REG_NONE) < 0) {
		return -ENOTSUP;
	}

	return 0;
}

void gpio_npcm_enable_io_pads(const struct device *dev, int pin)
{
	const struct gpio_npcm_config *const config = dev->config;
	const struct npcm_wui *io_wui;
	uint8_t ngpios = HAL_NGPIOS_INST(dev);

	/* Check pin mapping is valid */
	if (pin >= config->gpio_wui_map_size) {
		LOG_ERR("Invalid GPIO(%x, %d) pin", config->port, pin);
		return;
	}

	io_wui = &config->gpio_wui_maps[pin];

	/* Check miwu table is valid */
	if (NPCM_WUI_TABLE_OFFSET(io_wui->wk_src_idx) >= NPCM_MIWU_GROUP_MAX) {
		LOG_ERR("Cannot enable GPIO(%x, %d) pad", config->port, pin);
		return;
	}

	/*
	 * If this pin is configured as a GPIO interrupt source, do not
	 * implement bypass or the system cannot wake up via this event.
	 */
	if (pin < ngpios && !npcm_miwu_irq_get_state(io_wui)) {
		npcm_miwu_io_enable(io_wui);
	}
}

void gpio_npcm_disable_io_pads(const struct device *dev, int pin)
{
	const struct gpio_npcm_config *const config = dev->config;
	const struct npcm_wui *io_wui;
	uint8_t ngpios = HAL_NGPIOS_INST(dev);

	/* Check pin mapping is valid */
	if (pin >= config->gpio_wui_map_size) {
		LOG_ERR("Invalid GPIO(%x, %d) pin", config->port, pin);
		return;
	}

	io_wui = &config->gpio_wui_maps[pin];

	/* Check miwu table is valid */
	if (NPCM_WUI_TABLE_OFFSET(io_wui->wk_src_idx) >= NPCM_MIWU_GROUP_MAX) {
		LOG_ERR("Cannot disable GPIO(%x, %d) pad", config->port, pin);
		return;
	}

	/*
	 * If this pin is configured as a GPIO interrupt source, do not
	 * implement bypass or the system cannot wake up via this event.
	 */
	if (pin < ngpios && !npcm_miwu_irq_get_state(io_wui)) {
		npcm_miwu_io_disable(io_wui);
	}
}

/* GPIO API functions */
static int gpio_npcm_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	struct gpio_reg *const inst = HAL_BASE_INST(dev);
	struct scfg_reg *inst_scfg = SCFG_INST();
	uint32_t mask = BIT(pin);

	/* Check unsupported "Open Source" mode is set */
	if (((flags & GPIO_SINGLE_ENDED) != 0) && ((flags & GPIO_LINE_OPEN_DRAIN) == 0)) {
		return -ENOTSUP;
	}

	/* Set pinmux */
	if (gpio_npcm_set_pincfg(dev, pin) < 0) {
		return -ENOTSUP;
	}

	/*
	 * Configure pin as input.
	 * Output is configured only after setting all other attributes.
	 */
	if ((flags & GPIO_OUTPUT) == 0) {
		inst->PDIR &= ~mask;
	}

	/* Select open drain: 0=push-pull, 1=open-drain */
	if ((flags & GPIO_OPEN_DRAIN) != 0) {
		inst->PTYPE |= mask;
	} else {
		inst->PTYPE &= ~mask;
	}

	/* Open drain output mode want to enable internal pull up */
	if ((flags & GPIO_OPEN_DRAIN) && (flags & GPIO_OUTPUT)) {
		if ((flags & GPIO_PULL_UP)) {
			inst_scfg->DEVALTCX |= BIT(NPCM_DEVALTCX_GPIO_PULL_EN);
		} else {
			inst_scfg->DEVALTCX &= ~BIT(NPCM_DEVALTCX_GPIO_PULL_EN);
		}
	}

	/* Enable and select pull-up/down of GPIO: 0=pull-up, 1=pull-down */
	if ((flags & GPIO_PULL_UP) != 0) {
		inst->PPUD &= ~mask;
		inst->PPULL |= mask;
	} else if ((flags & GPIO_PULL_DOWN) != 0) {
		inst->PPUD |= mask;
		inst->PPULL |= mask;
	} else {
		/* disable pull down/up */
		inst->PPULL &= ~mask;
	}

	/* Set level: 0=low, 1=high */
	if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
		inst->PDOUT |= mask;
	} else if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
		inst->PDOUT &= ~mask;
	}

	/* Configure pin as output, if requested: 0=input, 1=output */
	if ((flags & GPIO_OUTPUT) != 0) {
		inst->PDIR |= mask;
	}

	return 0;
}

#ifdef CONFIG_GPIO_GET_CONFIG
static int gpio_npcm_pin_get_config(const struct device *dev, gpio_pin_t pin,
				    gpio_flags_t *out_flags)
{
	struct gpio_reg *const inst = HAL_BASE_INST(dev);
	uint32_t mask = BIT(pin);
	gpio_flags_t flags = 0;

	/* Check the pin direction: 0=input, 1=output */
	if (inst->PDIR & mask) {
		flags |= GPIO_OUTPUT;

		/* Check the pin type: 0=push-pull, 1=open-drain */
		if (inst->PTYPE & mask) {
			flags |= GPIO_OPEN_DRAIN;
		}

		/* Check the pin output level: 0=low, 1=high */
		if (inst->PDOUT & mask) {
			flags |= GPIO_OUTPUT_HIGH;
		} else {
			flags |= GPIO_OUTPUT_LOW;
		}
	} else {
		flags |= GPIO_INPUT;

		/* Check the pin pull-up/down: 0=disabled, 1=enabled */
		if (inst->PPULL & mask) {
			/* Check the pin is pull-up or pull-down: 0=pull-up, 1=pull-down */
			if (inst->PPUD & mask) {
				flags |= GPIO_PULL_DOWN;
			} else {
				flags |= GPIO_PULL_UP;
			}
		}
	}

	*out_flags = flags;

	return 0;
}
#endif

static int gpio_npcm_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
	struct gpio_reg *const inst = HAL_BASE_INST(dev);

	/* Get raw bits of GPIO input registers */
	*value = inst->PDIN;

	return 0;
}

static int gpio_npcm_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask,
					 gpio_port_value_t value)
{
	struct gpio_reg *const inst = HAL_BASE_INST(dev);
	uint8_t out = inst->PDOUT;

	inst->PDOUT = ((out & ~mask) | (value & mask));

	return 0;
}

static int gpio_npcm_port_set_bits_raw(const struct device *dev, gpio_port_value_t mask)
{
	struct gpio_reg *const inst = HAL_BASE_INST(dev);

	/* Set raw bits of GPIO output registers */
	inst->PDOUT |= mask;

	return 0;
}

static int gpio_npcm_port_clear_bits_raw(const struct device *dev, gpio_port_value_t mask)
{
	struct gpio_reg *const inst = HAL_BASE_INST(dev);

	/* Clear raw bits of GPIO output registers */
	inst->PDOUT &= ~mask;

	return 0;
}

static int gpio_npcm_port_toggle_bits(const struct device *dev, gpio_port_value_t mask)
{
	struct gpio_reg *const inst = HAL_BASE_INST(dev);

	/* Toggle raw bits of GPIO output registers */
	inst->PDOUT ^= mask;

	return 0;
}

static int gpio_npcm_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin,
					     enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	const struct gpio_npcm_config *const config = dev->config;
	const struct npcm_wui *wui;
	enum miwu_int_mode miwu_mode;
	enum miwu_int_trig miwu_trig;
	int ret = 0;

	/* Check pin mapping is valid */
	if (pin >= config->gpio_wui_map_size) {
		LOG_ERR("Invalid GPIO(%x, %d) pin", config->port, pin);
		return -EINVAL;
	}
	wui = &config->gpio_wui_maps[pin];

	/* Check miwu table is valid */
	if (NPCM_WUI_TABLE_OFFSET(wui->wk_src_idx) >= NPCM_MIWU_GROUP_MAX) {
		LOG_ERR("Cannot configure GPIO(%x, %d)", config->port, pin);
		return -EINVAL;
	}

	/* Disable irq of wake-up input io-pads before configuring them */
	npcm_miwu_irq_disable(wui);

	/* Configure and enable interrupt? */
	if (mode != GPIO_INT_MODE_DISABLED) {
		/* Determine interrupt is level or edge mode? */
		if (mode == GPIO_INT_MODE_EDGE) {
			miwu_mode = NPCM_MIWU_MODE_EDGE;
		} else {
			miwu_mode = NPCM_MIWU_MODE_LEVEL;
		}

		/* Determine trigger mode is low, high or both? */
		if (trig == GPIO_INT_TRIG_LOW) {
			miwu_trig = NPCM_MIWU_TRIG_LOW;
		} else if (trig == GPIO_INT_TRIG_HIGH) {
			miwu_trig = NPCM_MIWU_TRIG_HIGH;
		} else if (trig == GPIO_INT_TRIG_BOTH) {
			miwu_trig = NPCM_MIWU_TRIG_BOTH;
		} else {
			LOG_ERR("Invalid interrupt trigger type %d", trig);
			return -EINVAL;
		}

		/* Call MIWU routine to setup interrupt configuration */
		ret = npcm_miwu_interrupt_configure(wui, miwu_mode, miwu_trig);
		if (ret < 0) {
			LOG_ERR("Configure MIWU interrupt failed");
			return ret;
		}

		/* Enable it after configuration is completed */
		npcm_miwu_irq_enable(wui);
	}

	return 0;
}

static int gpio_npcm_manage_callback(const struct device *dev, struct gpio_callback *callback,
				     bool set)
{
	const struct gpio_npcm_config *const config = dev->config;
	struct miwu_callback *miwu_cb = (struct miwu_callback *)callback;
	unsigned int pin = find_lsb_set(callback->pin_mask) - 1;

	/* pin_mask should not be zero */
	if (pin < 0) {
		return -EINVAL;
	}

	/* Check pin mapping is valid */
	if (pin >= config->gpio_wui_map_size) {
		LOG_ERR("Invalid GPIO(%x, %d) pin", config->port, pin);
		return -EINVAL;
	}

	/* Has the IO pin valid MIWU input source? */
	if (NPCM_WUI_TABLE_OFFSET(config->gpio_wui_maps[pin].wk_src_idx) >= NPCM_MIWU_GROUP_MAX) {
		LOG_ERR("Cannot manage GPIO(%x, %d) callback!", config->port, pin);
		return -EINVAL;
	}

	/* Initialize WUI information in unused bits field */
	npcm_miwu_callback_init_gpio(miwu_cb, &config->gpio_wui_maps[pin], config->port);

	/* Insert or remove a IO callback which being called in MIWU ISRs */
	return npcm_miwu_callback_manage(miwu_cb, set);
}

/* GPIO initialization function */
int gpio_npcm_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

/* GPIO driver APIs */
static DEVICE_API(gpio, gpio_npcm_driver) = {
	.pin_configure = gpio_npcm_pin_configure,
#ifdef CONFIG_GPIO_GET_CONFIG
	.pin_get_config = gpio_npcm_pin_get_config,
#endif
	.port_get_raw = gpio_npcm_port_get_raw,
	.port_set_masked_raw = gpio_npcm_port_set_masked_raw,
	.port_set_bits_raw = gpio_npcm_port_set_bits_raw,
	.port_clear_bits_raw = gpio_npcm_port_clear_bits_raw,
	.port_toggle_bits = gpio_npcm_port_toggle_bits,
	.pin_interrupt_configure = gpio_npcm_pin_interrupt_configure,
	.manage_callback = gpio_npcm_manage_callback,
};

/* GPIO driver registration */
#define NPCM_DT_WUI(inst, prop, idx) {.wk_src_idx = DT_PROP_BY_IDX(inst, prop, idx)},
#define NPCM_GPIO_DEVICE_INIT(inst)                                                                \
	static const struct gpio_npcm_config gpio_npcm_cfg_##inst = {                              \
		.common =                                                                          \
			{                                                                          \
				.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(inst),            \
			},                                                                         \
		.base = DT_INST_REG_ADDR(inst),                                                    \
		.port = DT_INST_PROP(inst, nuvoton_index),                                         \
		.ngpios = DT_INST_PROP(inst, ngpios),                                              \
		.gpio_wui_map_size = DT_INST_PROP_LEN(inst, nuvoton_gpio_wui_maps),                \
		.gpio_wui_maps = {DT_INST_FOREACH_PROP_ELEM(inst, nuvoton_gpio_wui_maps,           \
							    NPCM_DT_WUI)},                         \
	};                                                                                         \
	BUILD_ASSERT(DT_INST_PROP_LEN(inst, nuvoton_gpio_wui_maps) <= DT_INST_PROP(inst, ngpios),  \
		     "size of prop. nuvoton,gpio-wui-maps must not greater than the pin number!"); \
                                                                                                   \
	static struct gpio_npcm_data gpio_npcm_data_##inst;                                        \
	BUILD_ASSERT(DT_INST_PROP(inst, nuvoton_index) < NPCM_GPIO_PORT_NUM,                       \
		     "prop. port must be less than the max port number");                          \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, gpio_npcm_init, NULL, &gpio_npcm_data_##inst,                  \
			      &gpio_npcm_cfg_##inst, PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY,      \
			      &gpio_npcm_driver);

DT_INST_FOREACH_STATUS_OKAY(NPCM_GPIO_DEVICE_INIT)
