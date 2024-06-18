/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_ADC_NPCM_ADC_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_ADC_NPCM_ADC_H_

/**
 * ADC channel number
 */
/* Voltage */
#define NPCM_ADC_CH0_AVSB   0
#define NPCM_ADC_CH1_VSB    1
#define NPCM_ADC_CH2_VCC    2
#define NPCM_ADC_CH3_VHIF   3
#define NPCM_ADC_CH4_VIN7   4
#define NPCM_ADC_CH5_VIN5   5
#define NPCM_ADC_CH6_VIN16  6
#define NPCM_ADC_CH7_THR16  7
#define NPCM_ADC_CH8_VIN15  8
#define NPCM_ADC_CH9_THR15  9
#define NPCM_ADC_CH10_VIN14 10
#define NPCM_ADC_CH11_THR14 11
#define NPCM_ADC_CH12_VIN1  12
#define NPCM_ADC_CH13_THR1  13
#define NPCM_ADC_CH14_VIN2  14
#define NPCM_ADC_CH15_THR2  15
#define NPCM_ADC_CH16_VIN3  16
#define NPCM_ADC_CH17_VTT   17
#define NPCM_ADC_CH18_VBAT  18

/* Diode */
#define NPCM_ADC_CH19_TD2P 19
#define NPCM_ADC_CH20_TD1P 20
#define NPCM_ADC_CH21_TD0P 21
#define NPCM_ADC_CH22_TD3P 22
#define NPCM_ADC_CH23_TD4P 23
#define NPCM_ADC_CH_MAX    24

#define NPCM_ADC_CHANNEL_SEL_OFFSET   0
#define NPCM_ADC_CHANNEL_SEL_MASK     0xFF
#define NPCM_ADC_CTRL6_BIT_OFFSET     8
#define NPCM_ADC_CTRL6_BIT_MASK       0xFF
#define NPCM_ADC_CTRL6_BIT_SET_OFFSET 16
#define NPCM_ADC_CTRL6_BIT_SET_MASK   0x1
#define NPCM_ADC_IS_DIODE_OFFSET      17
#define NPCM_ADC_IS_DIODE_MASK        0x1
#define NPCM_ADC_IS_VIN_THR_OFFSET    18
#define NPCM_ADC_IS_VIN_THR_MASK      0x1
#define NPCM_ADC_ID_OFFSET            19
#define NPCM_ADC_ID_MASK              0xFF

/* ADC channel configuration */
#define NPCM_ADC_CFG(id, diode, vin_thr, channel, bit, bit_set)                                    \
	((((id) & NPCM_ADC_ID_MASK) << NPCM_ADC_ID_OFFSET) +                                       \
	 (((diode) & NPCM_ADC_IS_DIODE_MASK) << NPCM_ADC_IS_DIODE_OFFSET) +                        \
	 (((vin_thr) & NPCM_ADC_IS_VIN_THR_MASK) << NPCM_ADC_IS_VIN_THR_OFFSET) +                  \
	 (((channel) & NPCM_ADC_CHANNEL_SEL_MASK) << NPCM_ADC_CHANNEL_SEL_OFFSET) +                \
	 (((bit) & NPCM_ADC_CTRL6_BIT_MASK) << NPCM_ADC_CTRL6_BIT_OFFSET) +                        \
	 (((bit_set) & NPCM_ADC_CTRL6_BIT_SET_MASK) << NPCM_ADC_CTRL6_BIT_SET_OFFSET))

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_ADC_NPCM_ADC_H_ */
