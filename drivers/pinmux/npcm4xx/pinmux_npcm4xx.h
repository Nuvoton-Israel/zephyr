/*
 * Copyright (c) 2023 Nuvoton Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __PINMUX_NCPM4XX_H__
#define __PINMUX_NPCM4XX_H__
#include "pin_id.h"
#include "sig_id.h"
#include "fun_id.h"

/**
 *  Reference: https://sf-zhou.github.io/programming/cpp_macro_number_of_arguments.html
 */
#define COUNT_ARG_N(_1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, \
		    _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26,	 \
		    _27, _28, _29, _30, _31, _32, _33, _34, _35, _36, _37, _38,	 \
		    _39, _40, _41, _42, _43, _44, _45, _46, _47, _48, _49, _50,	 \
		    _51, _52, _53, _54, _55, _56, _57, _58, _59, _60, _61, _62,	 \
		    _63, N, ...)						 \
	N
#define COUNT(...)								      \
	COUNT_ARG_N(dummy, ##__VA_ARGS__, 62, 61, 60, 59, 58, 57, 56, 55, 54, 53, 52, \
		    51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36,   \
		    35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20,   \
		    19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2,   \
		    1, 0)
/**
 * describe how to enable a singal: set/clr scfg registers w/ specific bitfields
 */
struct npcm4xx_sig_en {
	uint16_t offset;                /* scfg offset */
	uint8_t bits;                   /* bits to be set or clr */
	uint8_t op;                     /* operation: 0=set bits, 1=clr bits */
};

/**
 * describe how to enable a signal, include:
 * 1. number of npcm4xx_sig_en
 * 2. pin id of this signal used.
 * 3. a list of npcm4xx_sig_en(s)
 */
struct npcm4xx_sig_desc {
	uint8_t nsig_en;
	uint16_t pin;
	const struct npcm4xx_sig_en *sig_en;
};

/**
 * describe how to enable a function group, include:
 * 1. number of signals
 * 2. a list of signal id(s)
 */
struct npcm4xx_fun_desc {
	uint8_t nsignal;
	uint16_t sig_id_list[];
};

#define SIG_SYM(sig) sig_desc_ ## sig
#define SIG_SYM_PTR(sig) (&SIG_SYM(sig))
#define SIG_EN_SYM(sig) sig_en_ ## sig

#define FUN_SYM(fun) fun_desc_ ## fun
#define FUN_SYM_PTR(fun) (&FUN_SYM(fun))

#define SIG_DESC_SET(offset, idx) { offset, idx, 0 }
#define SIG_DESC_CLEAR(offset, idx) { offset, idx, 1 }
/* Usage:
 *		SIG_DECL(PWM0, AD26, SIG_DESC_SET(0x41C, 16));
 */
#define SIG_DECL(_sig_, _pin_, ...)			  \
	SIG_EN_DECL(_sig_, __VA_ARGS__);		  \
	const struct npcm4xx_sig_desc SIG_SYM(_sig_) = {	  \
		.pin = _pin_,				  \
		.nsig_en = ARRAY_SIZE(SIG_EN_SYM(_sig_)), \
		.sig_en = SIG_EN_SYM(_sig_),		  \
	};

#define SIG_EN_DECL(_sig_, ...)	\
	static const struct npcm4xx_sig_en SIG_EN_SYM(_sig_)[] = { __VA_ARGS__ }

/* Usage:
 *		FUN_DECL(PINCTRL_UART0_DEFAULT, SIG_SYM_PTR(CR_SOUT), SIG_SYM_PTR(CR_SIN));
 */
#define FUN_DECL(_fun_, ...)				\
	const struct npcm4xx_fun_desc FUN_SYM(_fun_) = {	\
		.nsignal = COUNT(__VA_ARGS__),		\
		.sig_id_list = { __VA_ARGS__ },		\
	}

#endif /* #ifndef __PINMUX_NPCM4XX_H__ */
