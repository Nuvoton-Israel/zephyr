/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_INTERRUPT_CONTROLLER_NPCM_MIWU_H_
#define ZEPHYR_INCLUDE_DRIVERS_INTERRUPT_CONTROLLER_NPCM_MIWU_H_

#include <zephyr/drivers/gpio.h>
#include <zephyr/dt-bindings/gpio/nuvoton-npcm-gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* NPCM wake-up input source offset */
#define NPCM_WUI_TABLE_OFFSET(n) (((n) >> NPCM_MIWU_TABLE_SHIFT) & NPCM_MIWU_TABLE_MASK)
#define NPCM_WUI_GROUP_OFFSET(n) (((n) >> NPCM_MIWU_GROUP_SHIFT) & NPCM_MIWU_GROUP_MASK)
#define NPCM_WUI_BIT_OFFSET(n)   ((n) & NPCM_MIWU_BIT_MASK)

/* The NPCM MIWU table index */
enum miwu_table {
	NPCM_MIWU_TABLE_0,
	NPCM_MIWU_TABLE_1,
	NPCM_MIWU_TABLE_2,
	NPCM_MIWU_TABLE_MAX
};

/* The NPCM MIWU group index */
enum miwu_group {
	NPCM_MIWU_GROUP_1,
	NPCM_MIWU_GROUP_2,
	NPCM_MIWU_GROUP_3,
	NPCM_MIWU_GROUP_4,
	NPCM_MIWU_GROUP_5,
	NPCM_MIWU_GROUP_6,
	NPCM_MIWU_GROUP_7,
	NPCM_MIWU_GROUP_8,
	NPCM_MIWU_GROUP_MAX
};

/* The NPCM interrupt modes */
enum miwu_int_mode {
	NPCM_MIWU_MODE_LEVEL,
	NPCM_MIWU_MODE_EDGE,
};

/* The NPCM interrupt trigger modes */
enum miwu_int_trig {
	NPCM_MIWU_TRIG_LOW,  /** Edge failing or active low detection */
	NPCM_MIWU_TRIG_HIGH, /** Edge rising or active high detection */
	NPCM_MIWU_TRIG_BOTH, /** Both edge rising and failing detection */
};

/* The NPCM miwu driver callback type */
enum {
	NPCM_MIWU_CALLBACK_GPIO,
	NPCM_MIWU_CALLBACK_DEV,
};

/**
 * @brief NPCM wake-up input source structure
 *
 * Used to indicate a Wake-Up Input source (WUI) belongs to which group and bit of Multi-Input
 * Wake-Up Unit (MIWU) modules.
 */
struct npcm_wui {
	uint8_t wk_src_idx;  /** Wake-up input source index: [7:6]=table, [5:3]=group, [2:0]=bit */
	uint8_t reserved[3]; /** Reserved */
};

/**
 * Define npcm miwu driver callback handler signature for wake-up input source of generic hardware.
 * Its parameters contain the device issued interrupt and corresponding WUI source.
 */
typedef void (*miwu_dev_callback_handler_t)(const struct device *source, struct npcm_wui *wui);

/**
 * @brief MIWU callback structure for a gpio or device input
 *
 * Used to register a generic gpio/device callback in the driver instance callback list. Beware
 * such structure should not be allocated on stack.
 */
struct miwu_callback {
	sys_snode_t node;    /** Node of single-linked list */
	uint8_t cb_type;     /** Callback type */
	struct npcm_wui wui; /** Device instance register callback function */

	union {
		struct {
			gpio_callback_handler_t handler; /** GPIO event callback function */
			uint8_t pin_mask;    /** A mask of pins the callback is interested in */
			uint8_t gpio_port;   /** GPIO device index */
			uint8_t reserved[2]; /** Reserved */
		} io_cb;

		struct {
			miwu_dev_callback_handler_t handler; /** Device event callback function */
			const struct device *source;         /** Wake-up input source */
		} dev_cb;
	};
};

/**
 * @brief Enable interrupt of the wake-up input source
 *
 * @param A pointer on wake-up input source
 */
void npcm_miwu_irq_enable(const struct npcm_wui *wui);

/**
 * @brief Disable interrupt of the wake-up input source
 *
 * @param wui A pointer on wake-up input source
 */
void npcm_miwu_irq_disable(const struct npcm_wui *wui);

/**
 * @brief Connect io to the wake-up input source
 *
 * @param wui A pointer on wake-up input source
 */
void npcm_miwu_io_enable(const struct npcm_wui *wui);

/**
 * @brief Disconnect io to the wake-up input source
 *
 * @param wui A pointer on wake-up input source
 */
void npcm_miwu_io_disable(const struct npcm_wui *wui);

/**
 * @brief Get interrupt state of the wake-up input source
 *
 * @param wui A pointer on wake-up input source
 *
 * @retval 0 if interrupt is disabled, otherwise interrupt is enabled
 */
bool npcm_miwu_irq_get_state(const struct npcm_wui *wui);

/**
 * @brief Get & clear interrupt pending bit of the wake-up input source
 *
 * @param wui A pointer on wake-up input source
 *
 * @retval 1 if interrupt is pending
 */
bool npcm_miwu_irq_get_and_clear_pending(const struct npcm_wui *wui);

/**
 * @brief Configure interrupt type of the wake-up input source
 *
 * @param wui Pointer to wake-up input source for configuring
 * @param mode Interrupt mode supported by NPCM MIWU
 * @param trig Interrupt trigger mode supported by NPCM MIWU
 *
 * @retval 0 If successful
 * @retval -EINVAL Invalid parameters
 */
int npcm_miwu_interrupt_configure(const struct npcm_wui *wui, enum miwu_int_mode mode,
				  enum miwu_int_trig trig);

/**
 * @brief Function to initialize a struct miwu_callback with gpio properly
 *
 * @param callback Pointer to io callback structure for initialization
 * @param io_wui Pointer to wake-up input IO source
 * @param port GPIO port issued a callback function
 */
void npcm_miwu_callback_init_gpio(struct miwu_callback *callback, const struct npcm_wui *io_wui,
				  int port);

/**
 * @brief Function to initialize a struct miwu_callback with device properly
 *
 * @param callback Pointer to device callback structure for initialization
 * @param dev_wui Pointer to wake-up input device source
 * @param handler A function called when its device input event issued
 * @param source Pointer to device instance issued a callback function
 */
void npcm_miwu_callback_init_dev(struct miwu_callback *callback, const struct npcm_wui *dev_wui,
				 miwu_dev_callback_handler_t handler, const struct device *source);

/**
 * @brief Function to insert or remove a miwu callback from a callback list
 *
 * @param callback Pointer to miwu callback structure
 * @param set A boolean indicating insertion or removal of the callback
 *
 * @retval 0 If successful.
 * @retval -EINVAL Invalid parameters
 */
int npcm_miwu_callback_manage(struct miwu_callback *cb, bool set);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_INTERRUPT_CONTROLLER_NPCM_MIWU_H_ */
