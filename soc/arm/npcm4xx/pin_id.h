/*
 * Copyright (c) 2023 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __PIN_ID_H__
#define __PIN_ID_H__

/* Pin ID */
enum {
	PIN_NONE = -1,
#define PIN_DEFINE(pin) pin,
    #include "pin_def_list.h"
#undef PIN_DEFINE
	MAX_PIN_ID,
};

#endif /* #ifndef __PIN_ID_H__ */
