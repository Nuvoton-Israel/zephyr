/*
 * Copyright (c) 2026 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_NPCM4_CLOCK_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_NPCM4_CLOCK_H_

#include <zephyr/dt-bindings/clock/npcm_clock.h>

/*
 * Clock bus references used as the first cell of a 'clocks' phandle-array
 * entry. The numbering is shared with the reused Nuvoton clock controller
 * driver.
 */
#define NPCM4_CLOCK_BUS_FREERUN 0
#define NPCM4_CLOCK_BUS_LFCLK   1
#define NPCM4_CLOCK_BUS_OSC     2
#define NPCM4_CLOCK_BUS_FIU     3
#define NPCM4_CLOCK_BUS_CORE    4
#define NPCM4_CLOCK_BUS_APB1    5
#define NPCM4_CLOCK_BUS_APB2    6
#define NPCM4_CLOCK_BUS_APB3    7
#define NPCM4_CLOCK_BUS_APB4    8
#define NPCM4_CLOCK_BUS_AHB6    9
#define NPCM4_CLOCK_BUS_FMCLK   10
#define NPCM4_CLOCK_BUS_FIU0    NPCM4_CLOCK_BUS_FIU
#define NPCM4_CLOCK_BUS_FIU1    11
#define NPCM4_CLOCK_BUS_MCLKD   12

/*
 * Power-Down Control (PWDWN_CTL) registers of the Power Management Controller.
 *
 * NPCM400 implements nine registers and their addresses are NOT contiguous:
 * PWDWN_CTL0 - PWDWN_CTL6 live at offsets 0x07 - 0x0d while PWDWN_CTL7 and
 * PWDWN_CTL8 live at 0x15 - 0x16.
 */
#define NPCM4_PWDWN_CTL0 0
#define NPCM4_PWDWN_CTL1 1
#define NPCM4_PWDWN_CTL2 2
#define NPCM4_PWDWN_CTL3 3
#define NPCM4_PWDWN_CTL4 4
#define NPCM4_PWDWN_CTL5 5
#define NPCM4_PWDWN_CTL6 6
#define NPCM4_PWDWN_CTL7 7
#define NPCM4_PWDWN_CTL8 8

#define NPCM4_PWDWN_CTL_COUNT 9

#define NPCM4_PWDWN_CTL_OFFSET(n) (((n) < 7) ? (0x007 + (n)) : (0x015 + ((n) - 7)))

/*
 * Split a peripheral clock identifier from npcm_clock.h into the PWDWN_CTL
 * register index and the bit position within that register. Using these in
 * the devicetree keeps a peripheral's clock gate tied to a single named
 * constant instead of two hand-copied numbers.
 *
 * Example:
 *   clocks = <&pcc NPCM4_CLOCK_BUS_APB2
 *             NPCM4_PWDWN_CTL_OF(NPCM_CLOCK_UART)
 *             NPCM4_PWDWN_BIT_OF(NPCM_CLOCK_UART)>;
 */
#define NPCM4_PWDWN_CTL_OF(clk) ((clk) >> 3)
#define NPCM4_PWDWN_BIT_OF(clk) ((clk) & 0x7)

/* Core Domain Clock Generator (CDCG) register offsets */
#define NPCM4_HFCGCTRL_OFFSET 0x000
#define NPCM4_HFCGML_OFFSET   0x002
#define NPCM4_HFCGMH_OFFSET   0x004
#define NPCM4_HFCGN_OFFSET    0x006
#define NPCM4_HFCGP_OFFSET    0x008
#define NPCM4_HFCBCD_OFFSET   0x010
#define NPCM4_HFCBCD1_OFFSET  0x012
#define NPCM4_HFCBCD2_OFFSET  0x014
#define NPCM4_HFCBCD3_OFFSET  0x01d
#define NPCM4_LFCGCTL_OFFSET  0x100
#define NPCM4_LFCGCTL2_OFFSET 0x114

/* CDCG register fields */
#define NPCM4_HFCGCTRL_LOAD     0
#define NPCM4_HFCGCTRL_LOCK     2
#define NPCM4_HFCGCTRL_CLK_CHNG 7

#define NPCM4_LFCGCTL2_XT_OSC_SL_EN 6

/* Power Management Controller (PMC) register offsets */
#define NPCM4_PMCSR_OFFSET       0x000
#define NPCM4_ENIDL_CTL_OFFSET   0x003
#define NPCM4_DISIDL_CTL_OFFSET  0x004
#define NPCM4_DISIDL_CTL1_OFFSET 0x005
#define NPCM4_RAM_PD_OFFSET      0x020

/* PMC register fields */
#define NPCM4_PMCSR_DI_INSTW        0
#define NPCM4_PMCSR_DHF             1
#define NPCM4_PMCSR_IDLE            2
#define NPCM4_PMCSR_NWBI            3
#define NPCM4_PMCSR_OHFC            6
#define NPCM4_PMCSR_OLFC            7
#define NPCM4_DISIDL_CTL_RAM_DID    5
#define NPCM4_ENIDL_CTL_ADC_LFSL    7
#define NPCM4_ENIDL_CTL_LP_WK_CTL   6
#define NPCM4_ENIDL_CTL_PECI_ENI    2
#define NPCM4_ENIDL_CTL_ADC_ACC_DIS 1

/* Maximum output frequency of the frequency multiplier */
#define NPCM4_MAX_OFMCLK 96000000

/*
 * Maximum frequency an APB bus may run at. Unlike the npcx series, the
 * NPCM400 APB buses may be clocked at the full OFMCLK rate.
 */
#define NPCM4_MAX_APB_CLOCK NPCM4_MAX_OFMCLK

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_NPCM4_CLOCK_H_ */
