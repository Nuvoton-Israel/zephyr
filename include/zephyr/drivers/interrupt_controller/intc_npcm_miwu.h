/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Nuvoton NPCM Multi-Input Wake-Up (MIWU) input API
 *
 * A MIWU module has eight groups of eight wake-up inputs (WUI). Peripheral
 * drivers whose wake-up signal is routed to a MIWU input use this API to
 * select the trigger condition, enable the input, and receive a callback when
 * the input fires. The input is named by a nuvoton,wui phandle in the
 * peripheral's devicetree node:
 *
 *     rtc { nuvoton,wui = <&miwu1 8 7>; };
 *
 * Callbacks are available only on groups whose NVIC line is listed in the
 * MIWU node. An input with its own NVIC line is configured through this API
 * too, and its owning driver connects the interrupt itself.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_INTERRUPT_CONTROLLER_INTC_NPCM_MIWU_H_
#define ZEPHYR_INCLUDE_DRIVERS_INTERRUPT_CONTROLLER_INTC_NPCM_MIWU_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/slist.h>

#ifdef __cplusplus
extern "C" {
#endif

/** One MIWU input, as named by a nuvoton,wui devicetree property. */
struct npcm_wui_dt_spec {
	const struct device *dev;
	uint8_t group; /* 1 to 8, datasheet numbering */
	uint8_t bit;   /* 0 to 7 */
};

/**
 * @brief Initialize a npcm_wui_dt_spec from a devicetree property element.
 *
 * @param node_id Devicetree node identifier.
 * @param prop Lowercase-and-underscores property name.
 * @param idx Element index in the property.
 */
#define NPCM_WUI_DT_SPEC_GET_BY_IDX(node_id, prop, idx)                                            \
	{                                                                                          \
		.dev = DEVICE_DT_GET(DT_PHANDLE_BY_IDX(node_id, prop, idx)),                       \
		.group = DT_PHA_BY_IDX(node_id, prop, idx, group),                                 \
		.bit = DT_PHA_BY_IDX(node_id, prop, idx, bit),                                     \
	}

/** Equivalent to NPCM_WUI_DT_SPEC_GET_BY_IDX(node_id, prop, 0). */
#define NPCM_WUI_DT_SPEC_GET(node_id, prop) NPCM_WUI_DT_SPEC_GET_BY_IDX(node_id, prop, 0)

/** Equivalent to NPCM_WUI_DT_SPEC_GET(DT_DRV_INST(inst), prop). */
#define NPCM_WUI_DT_SPEC_INST_GET(inst, prop) NPCM_WUI_DT_SPEC_GET(DT_DRV_INST(inst), prop)

/** Trigger condition of a MIWU input. */
enum npcm_wui_trigger {
	NPCM_WUI_TRIG_EDGE_RISING,
	NPCM_WUI_TRIG_EDGE_FALLING,
	NPCM_WUI_TRIG_EDGE_BOTH,
	NPCM_WUI_TRIG_LEVEL_HIGH,
	NPCM_WUI_TRIG_LEVEL_LOW,
};

struct npcm_wui_callback;

/** Called from the MIWU ISR when the input the callback was added to fires. */
typedef void (*npcm_wui_callback_handler_t)(struct npcm_wui_callback *cb);

/**
 * @brief MIWU input callback.
 *
 * Embed it in the owning driver's data and recover that data with
 * CONTAINER_OF() in the handler. The memory must stay valid while the
 * callback is added.
 *
 * The handler runs in ISR context after the input's pending flag has been
 * cleared. A level-triggered input whose source is still asserted pends
 * again immediately, so the handler must deassert the source or disable
 * the input. Consumers must initialize after CONFIG_INTC_INIT_PRIORITY;
 * CONFIG_CHECK_INIT_PRIORITIES verifies this through the phandle.
 */
struct npcm_wui_callback {
	sys_snode_t node;
	npcm_wui_callback_handler_t handler;
	uint8_t bit; /* set by npcm_wui_add_callback() */
};

/**
 * @brief Select the trigger condition of an input.
 *
 * The input is disabled while its detection changes, pending events from
 * the previous configuration are cleared, and the input is enabled for
 * detection. It stays disabled until npcm_wui_enable() is called, so a
 * reconfigured input cannot raise a false event.
 *
 * @retval 0 on success.
 * @retval -EINVAL if @p trig is not a valid trigger.
 */
int npcm_wui_configure(const struct npcm_wui_dt_spec *wui, enum npcm_wui_trigger trig);

/** @brief Let a trigger event on the input generate a wake-up or interrupt. */
void npcm_wui_enable(const struct npcm_wui_dt_spec *wui);

/** @brief Stop the input from generating wake-ups or interrupts. */
void npcm_wui_disable(const struct npcm_wui_dt_spec *wui);

/** @brief Clear the input's pending event. */
void npcm_wui_clear_pending(const struct npcm_wui_dt_spec *wui);

/** @brief Return true if the input has a pending event. */
bool npcm_wui_is_pending(const struct npcm_wui_dt_spec *wui);

/**
 * @brief Add a callback for an input.
 *
 * Adding a callback that is already added moves it to the input.
 *
 * @retval 0 on success.
 * @retval -EINVAL if the callback has no handler.
 * @retval -ENOTSUP if the MIWU driver does not service the input's group.
 */
int npcm_wui_add_callback(const struct npcm_wui_dt_spec *wui, struct npcm_wui_callback *cb);

/**
 * @brief Remove a callback added with npcm_wui_add_callback().
 *
 * @retval 0 on success.
 * @retval -EINVAL if the callback was not added to the input's group.
 */
int npcm_wui_remove_callback(const struct npcm_wui_dt_spec *wui, struct npcm_wui_callback *cb);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_INTERRUPT_CONTROLLER_INTC_NPCM_MIWU_H_ */
