/*
 * Copyright (c) 2023 Nuvoton Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef CONFIG_GPIO_NPCM4XX
/* GPIO_0 */
GPIO_SIG_DEFINE(GPIO00, A1, SIG_DESC_CLEAR(0x4D, 0), SIG_DESC_CLEAR(0x19, 4), SIG_DESC_CLEAR(0x15, 1))
GPIO_SIG_DEFINE(GPIO01, B1, SIG_DESC_CLEAR(0x4D, 0), SIG_DESC_CLEAR(0x19, 3), SIG_DESC_CLEAR(0x13, 7))
GPIO_SIG_DEFINE(GPIO02, C1, SIG_DESC_CLEAR(0x4D, 0), SIG_DESC_CLEAR(0x19, 0), SIG_DESC_CLEAR(0x15, 2))
GPIO_SIG_DEFINE(GPIO03, D1, SIG_DESC_CLEAR(0x4D, 0), SIG_DESC_CLEAR(0x19, 3), SIG_DESC_CLEAR(0x14, 0))
GPIO_SIG_DEFINE(GPIO04, E1, SIG_DESC_CLEAR(0x4D, 0), SIG_DESC_CLEAR(0x19, 1), SIG_DESC_CLEAR(0x1B, 1), SIG_DESC_CLEAR(0x14, 3))
GPIO_SIG_DEFINE(GPIO05, D2, SIG_DESC_CLEAR(0x4D, 0), SIG_DESC_CLEAR(0x19, 2), SIG_DESC_CLEAR(0x1A, 6), SIG_DESC_CLEAR(0x13, 1))
GPIO_SIG_DEFINE(GPIO06, C3, SIG_DESC_CLEAR(0x4D, 1), SIG_DESC_CLEAR(0x19, 3), SIG_DESC_CLEAR(0x15, 3))
GPIO_SIG_DEFINE(GPIO07, D3, SIG_DESC_CLEAR(0x4D, 1), SIG_DESC_CLEAR(0x19, 5), SIG_DESC_CLEAR(0x14, 1))
/* GPIO_1 */
GPIO_SIG_DEFINE(GPIO10, F4, SIG_DESC_CLEAR(0x1B, 5))
GPIO_SIG_DEFINE(GPIO11, F3, SIG_DESC_CLEAR(0x1E, 5), SIG_DESC_CLEAR(0x1B, 4))
/* GPIO12 only avaliable when VHIF = 3.3V and PLTRST (GPIOB7) = 3VSB */
GPIO_SIG_DEFINE(GPIO12, G4, SIG_DESC_CLEAR(0x1B, 0))
GPIO_SIG_DEFINE(GPIO13, G3, SIG_DESC_CLEAR(0x1E, 5), SIG_DESC_CLEAR(0x1B, 4))
GPIO_SIG_DEFINE(GPIO14, H4, SIG_DESC_CLEAR(0x1B, 2), SIG_DESC_CLEAR(0x1C, 6))
GPIO_SIG_DEFINE(GPIO15, H3, SIG_DESC_CLEAR(0x1A, 7), SIG_DESC_CLEAR(0x1B, 3), SIG_DESC_CLEAR(0x1C, 7))
GPIO_SIG_DEFINE(GPIO16, H2, SIG_DESC_CLEAR(0x1B, 4), SIG_DESC_CLEAR(0x6D, 7))
GPIO_SIG_DEFINE(GPIO17, J4, SIG_DESC_CLEAR(0x17, 6), SIG_DESC_CLEAR(0x1B, 6), SIG_DESC_CLEAR(0x6D, 7))
/* GPIO_2 */
GPIO_SIG_DEFINE(GPIO20, M4, SIG_DESC_CLEAR(0x16, 7), SIG_DESC_CLEAR(0x16, 2), SIG_DESC_CLEAR(0x1E, 7))
GPIO_SIG_DEFINE(GPIO21, L4, SIG_DESC_CLEAR(0x10, 2), SIG_DESC_CLEAR(0x0B, 6), SIG_DESC_CLEAR(0x6B, 4), SIG_DESC_CLEAR(0x16, 2), SIG_DESC_CLEAR(0x17, 1))
GPIO_SIG_DEFINE(GPIO22, J5, SIG_DESC_CLEAR(0x0B, 6), SIG_DESC_CLEAR(0x6B, 5), SIG_DESC_CLEAR(0x16, 2), SIG_DESC_CLEAR(0x16, 3))
GPIO_SIG_DEFINE(GPIO23, K4, SIG_DESC_CLEAR(0x10, 1), SIG_DESC_CLEAR(0x0B, 1), SIG_DESC_CLEAR(0x6B, 6), SIG_DESC_CLEAR(0x16, 2))
GPIO_SIG_DEFINE(GPIO24, K5, SIG_DESC_CLEAR(0x0B, 1), SIG_DESC_CLEAR(0x6B, 7), SIG_DESC_CLEAR(0x16, 2), SIG_DESC_CLEAR(0x16, 3))
GPIO_SIG_DEFINE(GPIO25, L5, SIG_DESC_CLEAR(0x0B, 2), SIG_DESC_CLEAR(0x16, 3), SIG_DESC_CLEAR(0x16, 2), SIG_DESC_CLEAR(0x1A, 5))
GPIO_SIG_DEFINE(GPIO26, M5, SIG_DESC_CLEAR(0x0B, 2), SIG_DESC_CLEAR(0x16, 2), SIG_DESC_CLEAR(0x1A, 5))
GPIO_SIG_DEFINE(GPIO27, G6, SIG_DESC_CLEAR(0x0B, 4), SIG_DESC_CLEAR(0x16, 2), SIG_DESC_CLEAR(0x16, 3), SIG_DESC_CLEAR(0x16, 1))
/* GPIO_3 */
GPIO_SIG_DEFINE(GPIO30, J6, SIG_DESC_CLEAR(0x6B, 0), SIG_DESC_CLEAR(0x31, 4), SIG_DESC_CLEAR(0x16, 2), SIG_DESC_CLEAR(0x16, 3))
GPIO_SIG_DEFINE(GPIO31, K6, SIG_DESC_CLEAR(0x6B, 1), SIG_DESC_CLEAR(0x31, 5), SIG_DESC_CLEAR(0x16, 2), SIG_DESC_CLEAR(0x16, 3))
GPIO_SIG_DEFINE(GPIO32, J7, SIG_DESC_CLEAR(0x6B, 2), SIG_DESC_CLEAR(0x16, 2), SIG_DESC_CLEAR(0x16, 3), SIG_DESC_CLEAR(0x6D, 2))
GPIO_SIG_DEFINE(GPIO33, K7, SIG_DESC_CLEAR(0x6B, 3), SIG_DESC_CLEAR(0x16, 2), SIG_DESC_CLEAR(0x16, 3), SIG_DESC_CLEAR(0x6D, 2))
GPIO_SIG_DEFINE(GPIO34, L6, SIG_DESC_CLEAR(0x31, 0), SIG_DESC_CLEAR(0x16, 2), SIG_DESC_CLEAR(0x1A, 3))
GPIO_SIG_DEFINE(GPIO35, M6, SIG_DESC_CLEAR(0x31, 1), SIG_DESC_CLEAR(0x16, 2), SIG_DESC_CLEAR(0x16, 3), SIG_DESC_CLEAR(0x1A, 3))
GPIO_SIG_DEFINE(GPIO36, L7, SIG_DESC_CLEAR(0x16, 2), SIG_DESC_CLEAR(0x17, 5), SIG_DESC_CLEAR(0x14, 4), SIG_DESC_CLEAR(0x1A, 2))
GPIO_SIG_DEFINE(GPIO37, M7, SIG_DESC_CLEAR(0x16, 2), SIG_DESC_CLEAR(0x16, 3), SIG_DESC_CLEAR(0x13, 2), SIG_DESC_CLEAR(0x1A, 2))
/* GPIO_4 */
GPIO_SIG_DEFINE(GPIO40, M8, SIG_DESC_CLEAR(0x1E, 3), SIG_DESC_CLEAR(0x2D, 6))
GPIO_SIG_DEFINE(GPIO41, M9, SIG_DESC_CLEAR(0x1E, 3), SIG_DESC_CLEAR(0x2D, 6))
GPIO_SIG_DEFINE(GPIO42, M10, SIG_DESC_CLEAR(0x1E, 3), SIG_DESC_CLEAR(0x2D, 6))
GPIO_SIG_DEFINE(GPIO43, M11, SIG_DESC_CLEAR(0x1E, 3), SIG_DESC_CLEAR(0x2D, 6))
GPIO_SIG_DEFINE(GPIO44, M12, SIG_DESC_CLEAR(0x1C, 3))
GPIO_SIG_DEFINE(GPIO45, L12, SIG_DESC_CLEAR(0x1C, 3))
GPIO_SIG_DEFINE(GPIO46, K12, SIG_DESC_CLEAR(0x1C, 3))
GPIO_SIG_DEFINE(GPIO47, J12, SIG_DESC_CLEAR(0x1C, 3))
/* GPIO_5 */
GPIO_SIG_DEFINE(GPIO50, K8, SIG_DESC_SET(0x5C, 1))
GPIO_SIG_DEFINE(GPIO51, J8, SIG_DESC_SET(0x5C, 0))
GPIO_SIG_DEFINE(GPIO52, L9, SIG_DESC_SET(0x5C, 2))
GPIO_SIG_DEFINE(GPIO53, L10, SIG_DESC_SET(0x5F, 0))
GPIO_SIG_DEFINE(GPIO54, L11, SIG_DESC_SET(0x5C, 3),  SIG_DESC_CLEAR(0x15, 5))
/* WP_GPIO55 should set DEV_CTRL3[1] to zero (W1S) */
GPIO_SIG_DEFINE(GPIO55, H12, SIG_DESC_CLEAR(0x13, 1), SIG_DESC_CLEAR(0x1C, 2))
GPIO_SIG_DEFINE(GPIO56, G12, SIG_DESC_CLEAR(0x1C, 2))
GPIO_SIG_DEFINE(GPIO57, H10, SIG_DESC_CLEAR(0x1E, 4))
/* GPIO_6 */
GPIO_SIG_DEFINE(GPIO60, K10, SIG_DESC_SET(0x5D, 2), SIG_DESC_CLEAR(0x1A, 0))
GPIO_SIG_DEFINE(GPIO61, K11, SIG_DESC_SET(0x5D, 1), SIG_DESC_CLEAR(0x1A, 0))
GPIO_SIG_DEFINE(GPIO62, J10, SIG_DESC_SET(0x5D, 0))
GPIO_SIG_DEFINE(GPIO63, J11, SIG_DESC_SET(0x5F, 2))
GPIO_SIG_DEFINE(GPIO64, H11, SIG_DESC_CLEAR(0x5C, 4), SIG_DESC_CLEAR(0x5F, 3), SIG_DESC_CLEAR(0x19, 6), SIG_DESC_CLEAR(0x14, 5))
GPIO_SIG_DEFINE(GPIO65, G9, SIG_DESC_SET(0x5F, 1))
GPIO_SIG_DEFINE(GPIO66, D12, SIG_DESC_CLEAR(0x1C, 1))
GPIO_SIG_DEFINE(GPIO67, C12, SIG_DESC_CLEAR(0x1C, 1))
/* GPIO_7 */
GPIO_SIG_DEFINE(GPIO70, B12, SIG_DESC_CLEAR(0x1C, 1))
/* GPIO71 depend on DSW_EN(GPIO74) strap pin */
GPIO_SIG_DEFINE(GPIO71, F9, SIG_DESC_SET(0x5C, 5))
GPIO_SIG_DEFINE(GPIO72, E9, SIG_DESC_SET(0x17, 0))
GPIO_SIG_DEFINE(GPIO73, A12, SIG_DESC_CLEAR(0x1C, 1))
/* GPIO74/GPIO75 depend on strap pin */
/* WP_GPIO76 should set DEV_CTRL3[2] to zero (W1S) */
GPIO_SIG_DEFINE(GPIO76, A11, SIG_DESC_SET(0x1C, 4))
GPIO_SIG_DEFINE(GPIO77, A10, SIG_DESC_SET(0x1C, 4))
/* GPIO_8 */
/* GPIO80 depend on DSW_EN(GPIO74) strap pin */
GPIO_SIG_DEFINE(GPIO80, B10, SIG_DESC_CLEAR(0x1E, 0))
GPIO_SIG_DEFINE(GPIO81, C10, SIG_DESC_CLEAR(0x13, 3))
GPIO_SIG_DEFINE(GPIO82, B9, SIG_DESC_CLEAR(0x1A, 4), SIG_DESC_CLEAR(0x14, 2))
GPIO_SIG_DEFINE(GPIO83, A9, SIG_DESC_CLEAR(0x1A, 4), SIG_DESC_CLEAR(0x13, 0))
/* GPIO84/GPIO85 not exist */
GPIO_SIG_DEFINE(GPIO86, C8, SIG_DESC_CLEAR(0x11, 7))
GPIO_SIG_DEFINE(GPIO87, F7, SIG_DESC_CLEAR(0x11, 5))
/* GPIO_9 */
GPIO_SIG_DEFINE(GPIO90, D7, SIG_DESC_CLEAR(0x12, 2), SIG_DESC_CLEAR(0x12, 3))
GPIO_SIG_DEFINE(GPIO91, C7, SIG_DESC_CLEAR(0x12, 1), SIG_DESC_CLEAR(0x12, 3))
GPIO_SIG_DEFINE(GPIO92, A6, SIG_DESC_CLEAR(0x12, 0), SIG_DESC_CLEAR(0x12, 3))
GPIO_SIG_DEFINE(GPIO93, B6, SIG_DESC_CLEAR(0x11, 1), SIG_DESC_CLEAR(0x12, 3))
GPIO_SIG_DEFINE(GPIO94, C6, SIG_DESC_CLEAR(0x11, 2))
GPIO_SIG_DEFINE(GPIO95, D6, SIG_DESC_CLEAR(0x11, 3), SIG_DESC_CLEAR(0x11, 4))
GPIO_SIG_DEFINE(GPIO96, A5, SIG_DESC_SET(0x17, 3))
GPIO_SIG_DEFINE(GPIO97, B5, SIG_DESC_SET(0x17, 4))
/* GPIO_A */
GPIO_SIG_DEFINE(GPIOA0, C5, SIG_DESC_CLEAR(0x14, 6))
GPIO_SIG_DEFINE(GPIOA1, D5, SIG_DESC_CLEAR(0x13, 4))
GPIO_SIG_DEFINE(GPIOA2, B3, SIG_DESC_CLEAR(0x31, 2), SIG_DESC_CLEAR(0x1A, 1))
GPIO_SIG_DEFINE(GPIOA3, D4, SIG_DESC_CLEAR(0x14, 7))
GPIO_SIG_DEFINE(GPIOA4, C4, SIG_DESC_CLEAR(0x13, 5))
GPIO_SIG_DEFINE(GPIOA5, B4, SIG_DESC_CLEAR(0x15, 0))
GPIO_SIG_DEFINE(GPIOA6, A4, SIG_DESC_CLEAR(0x13, 6))
GPIO_SIG_DEFINE(GPIOA7, B2, SIG_DESC_CLEAR(0x31, 3), SIG_DESC_CLEAR(0x1A, 1))
/* GPIO_B */
GPIO_SIG_DEFINE(GPIOB0, C2, SIG_DESC_CLEAR(0x15, 6))
GPIO_SIG_DEFINE(GPIOB1, F1, SIG_DESC_CLEAR(0x10, 5))
GPIO_SIG_DEFINE(GPIOB2, E2, SIG_DESC_CLEAR(0x10, 6))
/* GPIOB3 depend on strap pin ESPI_EN(GPIOB4) == zero */
GPIO_SIG_DEFINE(GPIOB3, F2, SIG_DESC_CLEAR(0x10, 7))
/* GPIOB4 = ESPI_EN strap pin */
GPIO_SIG_DEFINE(GPIOB5, E4, SIG_DESC_CLEAR(0x1E, 4))
GPIO_SIG_DEFINE(GPIOB6, L8, SIG_DESC_CLEAR(0x1E, 6), SIG_DESC_CLEAR(0x16, 2))
/* GPIOB7 = PLTRST# depend on ESPI_EN (GPIOB4) == 1 */
/* GPIO_C */
GPIO_SIG_DEFINE(GPIOC0, M2, SIG_DESC_CLEAR(0x31, 6), SIG_DESC_CLEAR(0x62, 0))
GPIO_SIG_DEFINE(GPIOC1, L2, SIG_DESC_CLEAR(0x31, 7), SIG_DESC_CLEAR(0x62, 1))
GPIO_SIG_DEFINE(GPIOC2, L2, SIG_DESC_CLEAR(0x62, 2), SIG_DESC_CLEAR(0x6D, 0))
GPIO_SIG_DEFINE(GPIOC3, L2, SIG_DESC_CLEAR(0x62, 3), SIG_DESC_CLEAR(0x6D, 0))
GPIO_SIG_DEFINE(GPIOC4, M3, SIG_DESC_CLEAR(0x6C, 0), SIG_DESC_CLEAR(0x62, 4))
GPIO_SIG_DEFINE(GPIOC5, L3, SIG_DESC_CLEAR(0x6C, 1), SIG_DESC_CLEAR(0x62, 5))
GPIO_SIG_DEFINE(GPIOC6, L2, SIG_DESC_CLEAR(0x62, 6), SIG_DESC_CLEAR(0x6D, 1))
GPIO_SIG_DEFINE(GPIOC7, L2, SIG_DESC_CLEAR(0x62, 7), SIG_DESC_CLEAR(0x6D, 1))
/* GPIO_D */
GPIO_SIG_DEFINE(GPIOD0, G11, SIG_DESC_CLEAR(0x0B, 3), SIG_DESC_CLEAR(0x66, 0))
GPIO_SIG_DEFINE(GPIOD1, F11, SIG_DESC_CLEAR(0x0B, 3), SIG_DESC_CLEAR(0x66, 1))
GPIO_SIG_DEFINE(GPIOD2, G10, SIG_DESC_CLEAR(0x6D, 3), SIG_DESC_CLEAR(0x66, 2))
GPIO_SIG_DEFINE(GPIOD3, G10, SIG_DESC_CLEAR(0x6D, 3), SIG_DESC_CLEAR(0x66, 3))
GPIO_SIG_DEFINE(GPIOD4, E11, SIG_DESC_CLEAR(0x6D, 4), SIG_DESC_CLEAR(0x66, 4))
GPIO_SIG_DEFINE(GPIOD5, D11, SIG_DESC_CLEAR(0x6D, 4), SIG_DESC_CLEAR(0x66, 5))
GPIO_SIG_DEFINE(GPIOD6, E10, SIG_DESC_CLEAR(0x6D, 5), SIG_DESC_CLEAR(0x66, 6))
GPIO_SIG_DEFINE(GPIOD7, D10, SIG_DESC_CLEAR(0x6D, 5), SIG_DESC_CLEAR(0x66, 7))
/* GPIO_E */
GPIO_SIG_DEFINE(GPIOE0, J9, SIG_DESC_SET(0x5E, 2), SIG_DESC_SET(0x5E, 5))
GPIO_SIG_DEFINE(GPIOE1, K9, SIG_DESC_SET(0x5E, 0))
GPIO_SIG_DEFINE(GPIOE2, H9, SIG_DESC_SET(0x5E, 3))
/* GPIO_E3 = mode_strap_1 */
GPIO_SIG_DEFINE(GPIOE4, D9, SIG_DESC_SET(0x5D, 5))
GPIO_SIG_DEFINE(GPIOE5, D8, SIG_DESC_SET(0x5E, 1))
GPIO_SIG_DEFINE(GPIOE6, C9, SIG_DESC_SET(0x5D, 4))
/* GPIOE7 not exist */
/* GPIO_F */
GPIO_SIG_DEFINE(GPIOF0, B8, SIG_DESC_CLEAR(0x34, 6))
GPIO_SIG_DEFINE(GPIOF1, A8, SIG_DESC_CLEAR(0x34, 6))
GPIO_SIG_DEFINE(GPIOF2, B7, SIG_DESC_CLEAR(0x34, 7))
GPIO_SIG_DEFINE(GPIOF3, A7, SIG_DESC_CLEAR(0x34, 7))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart0), okay) && CONFIG_UART_NPCM4XX
/* DEVALTB, DEVALTC */
SIG_DEFINE(UARTA_CR_SIN, H4, SIG_DESC_CLEAR(0x1B, 2), SIG_DESC_SET(0x1C, 6))
/* DEVALTA, DEVLATB */
SIG_DEFINE(UARTA_CR_SOUT, H3, SIG_DESC_SET(0x1A, 7), SIG_DESC_CLEAR(0x1B, 3))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart1), okay) && CONFIG_UART_NPCM4XX
/* DEVALT6B, DEVALT31 */
SIG_DEFINE(UARTB_CR_SIN, J6, SIG_DESC_SET(0x6B, 0), SIG_DESC_CLEAR(0x31, 4))
/* DEVALT6B, DEVALT31 */
SIG_DEFINE(UARTB_CR_SOUT, K6, SIG_DESC_SET(0x6B, 1), SIG_DESC_CLEAR(0x31, 5))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c1a), okay) && CONFIG_I2C_NPCM4XX
/* DEVALT5D, DEVALTA */
SIG_DEFINE(SCL1A, K10, SIG_DESC_SET(0x5D, 2), SIG_DESC_SET(0x1A, 0))
/* DEVALT5D, DEVALTA */
SIG_DEFINE(SDA1A, K11, SIG_DESC_SET(0x5D, 1), SIG_DESC_SET(0x1A, 0))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c1b), okay) && CONFIG_I2C_NPCM4XX
/* DEVALT6, DEVALT6, DEVALTA */
SIG_DEFINE(SCL1B, M5, SIG_DESC_CLEAR(0x16, 2), SIG_DESC_CLEAR(0x16, 3), SIG_DESC_SET(0x1A, 5))
/* DEVALT6, DEVALT6, DEVALTA */
SIG_DEFINE(SDA1B, L5, SIG_DESC_CLEAR(0x16, 2), SIG_DESC_CLEAR(0x16, 3), SIG_DESC_SET(0x1A, 5))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c2a), okay) && CONFIG_I2C_NPCM4XX
/* DEVALTE */
SIG_DEFINE(SCL2A, F3, SIG_DESC_SET(0x1E, 5))
/* DEVALTE */
SIG_DEFINE(SDA2A, G3, SIG_DESC_SET(0x1E, 5))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c3a), okay) && CONFIG_I2C_NPCM4XX
/* DEVALT31, DEVALTA */
SIG_DEFINE(SCL3A, B2, SIG_DESC_CLEAR(0x31, 3), SIG_DESC_SET(0x1A, 1))
/* DEVALT31, DEVALTA */
SIG_DEFINE(SDA3A, B3, SIG_DESC_CLEAR(0x31, 2), SIG_DESC_SET(0x1A, 1))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c4a), okay) && CONFIG_I2C_NPCM4XX
/* DEVALT6, DEVALT7, DEVALT4, DEVALTA */
SIG_DEFINE(SCL4A, L7, SIG_DESC_CLEAR(0x16, 2), SIG_DESC_CLEAR(0x17, 5), SIG_DESC_CLEAR(0x14, 4), SIG_DESC_SET(0x1A, 2))
/* DEVALT6, DEVALT6, DEVALT3, DEVALTA */
SIG_DEFINE(SDA4A, M7, SIG_DESC_CLEAR(0x16, 2), SIG_DESC_CLEAR(0x16, 3), SIG_DESC_CLEAR(0x13, 2), SIG_DESC_SET(0x1A, 2))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c4b), okay) && CONFIG_I2C_NPCM4XX
/* DEVALT34, DEVALT6D */
SIG_DEFINE(SCL4B, B8, SIG_DESC_CLEAR(0x34, 6), SIG_DESC_SET(0x6D, 6))
/* DEVALT34, DEVALT6D */
SIG_DEFINE(SDA4B, A8, SIG_DESC_CLEAR(0x34, 6), SIG_DESC_SET(0x6D, 6))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c5a), okay) && CONFIG_I2C_NPCM4XX
/* DEVALT31, DEVALT6 */
SIG_DEFINE(SCL5A, L6, SIG_DESC_CLEAR(0x31, 0), SIG_DESC_CLEAR(0x16, 2), SIG_DESC_SET(0x1A, 3))
/* DEVALT31, DEVALT6, DEVALT6, DEVALTA */
SIG_DEFINE(SDA5A, M6, SIG_DESC_CLEAR(0x31, 1), SIG_DESC_CLEAR(0x16, 2), SIG_DESC_CLEAR(0x16, 3), SIG_DESC_SET(0x1A, 3))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c6a), okay) && CONFIG_I2C_NPCM4XX
/* DEVALT7, DEVALT7 */
SIG_DEFINE(SCL6A, A5, SIG_DESC_CLEAR(0x17, 2), SIG_DESC_CLEAR(0x17, 3))
/* DEVALT7, DEVALT7, DEVALT5 */
SIG_DEFINE(SDA6A, B5, SIG_DESC_CLEAR(0x17, 2), SIG_DESC_CLEAR(0x17, 4), SIG_DESC_CLEAR(0x15, 4))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c6b), okay) && CONFIG_I2C_NPCM4XX
/* DEVALTB, DEVALT6D */
SIG_DEFINE(SCL6B, H2, SIG_DESC_CLEAR(0x1B, 4), SIG_DESC_SET(0x6D, 7))
/* DEVALT7, DEVALTB, DEVALT6D */
SIG_DEFINE(SDA6B, J4, SIG_DESC_CLEAR(0x17, 6), SIG_DESC_CLEAR(0x1B, 6), SIG_DESC_SET(0x6D, 7))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c7a), okay) && CONFIG_I2C_NPCM4XX
/* DEVALT62, DEVALT6D */
SIG_DEFINE(SCL7A, K2, SIG_DESC_CLEAR(0x62, 2), SIG_DESC_SET(0x6D, 0))
/* DEVALT62, DEVALT6D */
SIG_DEFINE(SDA7A, J2, SIG_DESC_CLEAR(0x62, 3), SIG_DESC_SET(0x6D, 0))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c8a), okay) && CONFIG_I2C_NPCM4XX
/* DEVALT62, DEVALT6D */
SIG_DEFINE(SCL8A, K3, SIG_DESC_CLEAR(0x62, 6), SIG_DESC_SET(0x6D, 1))
/* DEVALT62, DEVALT6D */
SIG_DEFINE(SDA8A, J3, SIG_DESC_CLEAR(0x62, 7), SIG_DESC_SET(0x6D, 1))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c9a), okay) && CONFIG_I2C_NPCM4XX
/* DEVALT6B, DEVALT6, DEVALT6, DEVALT6D */
SIG_DEFINE(SCL9A, J7, SIG_DESC_CLEAR(0x6B, 2), SIG_DESC_CLEAR(0x16, 2), SIG_DESC_CLEAR(0x16, 3), SIG_DESC_SET(0x6D, 2))
/* DEVALT6B, DEVALT6, DEVALT6, DEVALT6D */
SIG_DEFINE(SDA9A, K7, SIG_DESC_CLEAR(0x6B, 3), SIG_DESC_CLEAR(0x16, 2), SIG_DESC_CLEAR(0x16, 3), SIG_DESC_SET(0x6D, 2))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c10a), okay) && CONFIG_I2C_NPCM4XX
/* DEVALT66, DEVALT6D */
SIG_DEFINE(SCL10A, G10, SIG_DESC_CLEAR(0x66, 2), SIG_DESC_SET(0x6D, 3))
/* DEVALT66, DEVALT6D */
SIG_DEFINE(SDA10A, F10, SIG_DESC_CLEAR(0x66, 3), SIG_DESC_SET(0x6D, 3))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c11a), okay) && CONFIG_I2C_NPCM4XX
/* DEVALT66, DEVALT6D */
SIG_DEFINE(SCL11A, E11, SIG_DESC_CLEAR(0x66, 4), SIG_DESC_SET(0x6D, 4))
/* DEVALT66, DEVALT6D */
SIG_DEFINE(SDA11A, D11, SIG_DESC_CLEAR(0x66, 5), SIG_DESC_SET(0x6D, 4))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c12a), okay) && CONFIG_I2C_NPCM4XX
/* DEVALT66, DEVALT6D */
SIG_DEFINE(SCL12A, E10, SIG_DESC_CLEAR(0x66, 6), SIG_DESC_SET(0x6D, 5))
/* DEVALT66, DEVALT6D */
SIG_DEFINE(SDA12A, D10, SIG_DESC_CLEAR(0x66, 7), SIG_DESC_SET(0x6D, 5))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(peci0), okay) && CONFIG_PECI_NPCM4XX
/* DEVALT7, DEVALT5 */
SIG_DEFINE(PECI, B5, SIG_DESC_CLEAR(0x17, 4), SIG_DESC_SET(0x15, 4))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(spi_fiu0), okay) && CONFIG_SPI_NPCM4XX_FIU
/* DEVALTC, DEVCNT */
SIG_DEFINE(SHD_CS, M12, SIG_DESC_SET(0x1c, 3), SIG_DESC_CLEAR(0x0, 6))
SIG_DEFINE(SHD_SCLK, J12, SIG_DESC_SET(0x1c, 3), SIG_DESC_CLEAR(0x0, 6))
SIG_DEFINE(SHD_DIO0, L12, SIG_DESC_SET(0x1c, 3), SIG_DESC_CLEAR(0x0, 6))
SIG_DEFINE(SHD_DIO1, K12, SIG_DESC_SET(0x1c, 3), SIG_DESC_CLEAR(0x0, 6))
SIG_DEFINE(SHD_DIO2, H12, SIG_DESC_SET(0x1c, 2))
SIG_DEFINE(SHD_DIO3, G12, SIG_DESC_SET(0x1c, 2))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(adc0), okay) && CONFIG_ADC_NPCM4XX
/* DEVALT1 */
SIG_DEFINE(VIN7, C8, SIG_DESC_SET(0x11, 7))
/* DEVALT1 */
SIG_DEFINE(VIN5, F7, SIG_DESC_SET(0x11, 5))
/* DEVALT2 */
SIG_DEFINE(VIN16, D7, SIG_DESC_SET(0x12, 2))
SIG_DEFINE(THR16, D7, SIG_DESC_SET(0x12, 2))
SIG_DEFINE(TD2P, D7, SIG_DESC_SET(0x12, 2))
/* DEVALT2 */
SIG_DEFINE(VIN15, C7, SIG_DESC_SET(0x12, 1))
SIG_DEFINE(THR15, C7, SIG_DESC_SET(0x12, 1))
SIG_DEFINE(TD1P, C7, SIG_DESC_SET(0x12, 1))
/* DEVALT2 */
SIG_DEFINE(VIN14, A6, SIG_DESC_SET(0x12, 0))
SIG_DEFINE(THR14, A6, SIG_DESC_SET(0x12, 0))
SIG_DEFINE(TD0P, A6, SIG_DESC_SET(0x12, 0))
/* DEVALT1 */
SIG_DEFINE(VIN1, B6, SIG_DESC_SET(0x11, 1))
SIG_DEFINE(THR1, B6, SIG_DESC_SET(0x11, 1))
SIG_DEFINE(TD3P, B6, SIG_DESC_SET(0x11, 1))
/* DEVALT1 */
SIG_DEFINE(VIN2, C6, SIG_DESC_SET(0x11, 2))
SIG_DEFINE(THR2, C6, SIG_DESC_SET(0x11, 2))
SIG_DEFINE(TD4P, C6, SIG_DESC_SET(0x11, 2))
/* DEVALT1 */
SIG_DEFINE(VIN3, D6, SIG_DESC_CLEAR(0x11, 4), SIG_DESC_SET(0x11, 3))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(spip1), okay) && CONFIG_SPIP_NPCM4XX
/* DEVALTC, DEV_CTL3 */
SIG_DEFINE(SPIP1_CS, A12, SIG_DESC_SET(0x1c, 1))
SIG_DEFINE(SPIP1_SCLK, D12, SIG_DESC_SET(0x1c, 1))
SIG_DEFINE(SPIP1_DIO0, C12, SIG_DESC_SET(0x1c, 1))
SIG_DEFINE(SPIP1_DIO1, B12, SIG_DESC_SET(0x1c, 1))
SIG_DEFINE(SPIP1_DIO2, A11, SIG_DESC_SET(0x1c, 1), SIG_DESC_CLEAR(0x04, 2))
SIG_DEFINE(SPIP1_DIO3, A10, SIG_DESC_SET(0x1c, 1))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(usbd0), okay) && CONFIG_USB_DC_NPCM4XX
/* DEVALT2E */
SIG_DEFINE(USBD_PHY_ICLK, A4, SIG_DESC_SET(0x2e, 4), SIG_DESC_CLEAR(0x2e, 6))
SIG_DEFINE(USBD_PHY_XCLK, A4, SIG_DESC_SET(0x2e, 4), SIG_DESC_SET(0x2e, 6))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i3c4), okay) && CONFIG_I3C_NPCM4XX
/* DEVALTA.4=1 */
SIG_DEFINE(I3C5_SCL, B9, SIG_DESC_SET(0x1A, 4))
/* DEVALTA.4=1, DEVPD3.3=1 */
SIG_DEFINE(I3C5_SDA, A9, SIG_DESC_SET(0x1A, 4), SIG_DESC_SET(0x7B, 3))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i3c5), okay) && CONFIG_I3C_NPCM4XX
/* DEVALT7.3=0, DEVALT7.2=1 */
SIG_DEFINE(I3C6_SCL, A5, SIG_DESC_CLEAR(0x17, 3), SIG_DESC_SET(0x17, 2))
/* DEVALT7.4=0, DEVALT5.4=0, DEVPD3.4=1 */
SIG_DEFINE(I3C6_SDA, B5, SIG_DESC_CLEAR(0x17, 4), SIG_DESC_CLEAR(0x15, 4), SIG_DESC_SET(0x7B, 4))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(sgpio0), okay) && CONFIG_SGPIO_NPCM4XX
/* DEVALT6A.0=1 */
SIG_DEFINE(IOX1_LDSH, C11, SIG_DESC_SET(0x6A, 0))
/* DEVALT6A.0=1 */
SIG_DEFINE(IOX1_DOUT, B11, SIG_DESC_SET(0x6A, 0))
/* DEVALT34.7=0, DEVALT6A.0=1 */
SIG_DEFINE(IOX1_DIN, B7, SIG_DESC_CLEAR(0x34, 7), SIG_DESC_SET(0x6A, 0))
/* DEVALT34.7=0, DEVALT6A.0=1 */
SIG_DEFINE(IOX1_SCLK, A7, SIG_DESC_CLEAR(0x34, 7), SIG_DESC_SET(0x6A, 0))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(sgpio1), okay) && CONFIG_SGPIO_NPCM4XX
/* DEVALT4.6=0, DEVALT6A.1=1 */
SIG_DEFINE(IOX2_LDSH, C5, SIG_DESC_CLEAR(0x14, 6), SIG_DESC_SET(0x6A, 1))
/* DEVALT3.4=0, DEVALT6A.1=1*/
SIG_DEFINE(IOX2_DOUT, D5, SIG_DESC_CLEAR(0x13, 4), SIG_DESC_SET(0x6A, 1))
/* DEVALT4.7=0, DEVALT6A.1=1 */
SIG_DEFINE(IOX2_DIN, D4, SIG_DESC_CLEAR(0x14, 7), SIG_DESC_SET(0x6A, 1))
/* DEVALT3.5=0, DEVALT6A.1=1 */
SIG_DEFINE(IOX2_SCLK, C4, SIG_DESC_CLEAR(0x13, 5), SIG_DESC_SET(0x6A, 1))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(emac), okay) && CONFIG_ETH_NPCM4XX
SIG_DEFINE(EMAC, D3, SIG_DESC_SET(0x4D, 0), SIG_DESC_SET(0x4D, 1), SIG_DESC_SET(0x4C, 3))
#endif
