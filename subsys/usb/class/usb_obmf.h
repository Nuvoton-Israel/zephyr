/*
 * Copyright (c) 2025 Google LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SUBSYS_USB_CLASS_USB_OBMF_H_
#define ZEPHYR_SUBSYS_USB_CLASS_USB_OBMF_H_

#include <stdint.h>
#include <device.h>

#ifdef __cplusplus
extern "C" {
#endif

struct obmf_ops {
	void (*read)(const struct device *dev, uint32_t len, uint8_t *data);
};

void usb_obmf_register_device(const struct device *dev,
			      const struct obmf_ops *ops);

int obmf_usb_ep_write(const struct device *dev, const uint8_t *data,
		      uint32_t data_len, uint32_t *bytes_ret);

#if CONFIG_OBMF_INTERRUPT_EP_SUPPORT
int obmf_usb_int_ep_write(const struct device *dev, const uint8_t *data,
			  uint32_t data_len, uint32_t *bytes_ret);
#endif

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_USB_CLASS_USB_OBMF_H_ */
