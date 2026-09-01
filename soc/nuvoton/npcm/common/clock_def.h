/*
 * Copyright (c) 2026 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _NUVOTON_NPCM_CLOCK_DEF_H_
#define _NUVOTON_NPCM_CLOCK_DEF_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/devicetree.h>
#include <soc_clock.h>

/* Clock settings from the pcc node */
/* Target OFMCLK freq */
#define OFMCLK      DT_PROP(DT_NODELABEL(pcc), clock_frequency)
/* Core clock prescaler */
#define FPRED_VAL   (DT_PROP(DT_NODELABEL(pcc), core_prescaler) - 1)
/* APB1 clock divider */
#define APB1DIV_VAL (DT_PROP(DT_NODELABEL(pcc), apb1_prescaler) - 1)
/* APB2 clock divider */
#define APB2DIV_VAL (DT_PROP(DT_NODELABEL(pcc), apb2_prescaler) - 1)
/* APB3 clock divider */
#define APB3DIV_VAL (DT_PROP(DT_NODELABEL(pcc), apb3_prescaler) - 1)

/* Core domain clock */
#define CORE_CLK (OFMCLK / DT_PROP(DT_NODELABEL(pcc), core_prescaler))

/* AHB6 clock */
#if (CORE_CLK > (MAX_OFMCLK / 2))
#define AHB6DIV_VAL 1 /* AHB6_CLK = CORE_CLK/2 */
#else
#define AHB6DIV_VAL 0 /* AHB6_CLK = CORE_CLK */
#endif

/* FIU clock divider */
#if (CORE_CLK > (MAX_OFMCLK / 2))
#define FIUDIV_VAL 1 /* FIU_CLK = CORE_CLK/2 */
#else
#define FIUDIV_VAL 0 /* FIU_CLK = CORE_CLK */
#endif

/* I3C clock divider, MCLKD must stay between 40MHz and 50MHz */
#if (OFMCLK == MHZ(120))
#define MCLKD_SL 2 /* I3C_CLK = (MCLK / 3) */
#elif (OFMCLK <= MHZ(100) && OFMCLK >= MHZ(80))
#define MCLKD_SL 1 /* I3C_CLK = (MCLK / 2) */
#else
#define MCLKD_SL 0 /* I3C_CLK = MCLK */
#endif

/* Low Frequency clock */
#define LFCLK              32768
/* FMUL clock */
#define FMCLK              OFMCLK /* FMUL clock = OFMCLK */
/* APBs source clock */
#define APBSRC_CLK         OFMCLK
/* USB2.0 clock */
#define USB20_CLK          12000000
/* SIO clock */
#define SIO_CLK            48000000
/* Get APB clock freq */
#define NPCX_APB_CLOCK(no) (APBSRC_CLK / (APB##no##DIV_VAL + 1))

/*
 * Frequency multiplier M/N value definitions according to the requested
 * OFMCLK (Unit:Hz).
 */
#if (OFMCLK > (MAX_OFMCLK / 2))
#define HFCGN_VAL 0x82 /* Set XF_RANGE as 1 */
#else
#define HFCGN_VAL 0x02
#endif

#if (OFMCLK == 96000000)
#define HFCGMH_VAL 0x0B
#define HFCGML_VAL 0x72
#elif (OFMCLK == 90000000)
#define HFCGMH_VAL 0x0A
#define HFCGML_VAL 0xBA
#elif (OFMCLK == 80000000)
#define HFCGMH_VAL 0x09
#define HFCGML_VAL 0x89
#elif (OFMCLK == 66000000)
#define HFCGMH_VAL 0x07
#define HFCGML_VAL 0xDE
#elif (OFMCLK == 48000000)
#define HFCGMH_VAL 0x0B
#define HFCGML_VAL 0x72
#else
#error "Unsupported OFMCLK Frequency"
#endif

#endif /* _NUVOTON_NPCM_CLOCK_DEF_H_ */
