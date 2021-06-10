/*
 * Copyright (c) 2020-2021 Aspeed Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart10), okay)
FUN_DEFINE(UART10, TXD11, RXD11)
#endif

#ifdef CONFIG_ARM_ICE
FUN_DEFINE(ARM_ICE, ARM_TRST, ARM_TCK, ARM_TDI, ARM_TDO, ARM_TMS)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(adc0), okay) &&  CONFIG_ADC_ASPEED
FUN_DEFINE(ADC0, ADC0)
FUN_DEFINE(ADC1, ADC1)
FUN_DEFINE(ADC2, ADC2)
FUN_DEFINE(ADC3, ADC3)
FUN_DEFINE(ADC4, ADC4)
FUN_DEFINE(ADC5, ADC5)
FUN_DEFINE(ADC6, ADC6)
FUN_DEFINE(ADC7, ADC7)
#endif /* end of "#if CONFIG_DEVICE_ADC" */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(pwm), okay) && CONFIG_PWM_ASPEED
FUN_DEFINE(PWM0, PWM0)
FUN_DEFINE(PWM1, PWM1)
FUN_DEFINE(PWM2, PWM2)
FUN_DEFINE(PWM3, PWM3)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(tach), okay) && CONFIG_TACH_ASPEED
FUN_DEFINE(TACH0, TACH0)
FUN_DEFINE(TACH1, TACH1)
FUN_DEFINE(TACH2, TACH2)
FUN_DEFINE(TACH3, TACH3)
FUN_DEFINE(TACH4, TACH4)
FUN_DEFINE(TACH5, TACH5)
FUN_DEFINE(TACH6, TACH6)
FUN_DEFINE(TACH7, TACH7)
FUN_DEFINE(TACH8, TACH8)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(jtag0), okay) &&  CONFIG_JTAG_ASPEED
FUN_DEFINE(JTAGM0, MTRSTN1, MTDI1, MTCK1, MTMS1, MTDO1)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(jtag1), okay) &&  CONFIG_JTAG_ASPEED
FUN_DEFINE(JTAGM1, MTRSTN2, MTDI2, MTCK2, MTMS2, MTDO2)
#endif

#ifdef CONFIG_DEVICE_GPIO
#endif

#ifdef CONFIG_DEVICE_SGPIOM
FUN_DEFINE(SGPIOM, SGPMCLK, SGPMLD, SGPMO, SGPMI)
#endif

#ifdef CONFIG_DEVICE_I3C
FUN_DEFINE(HVI3C0, HVI3C1SCL, HVI3C1SDA)
FUN_DEFINE(HVI3C1, HVI3C2SCL, HVI3C2SDA)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c0), okay) && CONFIG_I2C_ASPEED
FUN_DEFINE(I2C0, SCL1, SDA1)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c1), okay) && CONFIG_I2C_ASPEED
FUN_DEFINE(I2C1, SCL2, SDA2)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c2), okay) && CONFIG_I2C_ASPEED
FUN_DEFINE(I2C2, SCL3, SDA3)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c3), okay) && CONFIG_I2C_ASPEED
FUN_DEFINE(I2C3, SCL4, SDA4)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c4), okay) && CONFIG_I2C_ASPEED
FUN_DEFINE(I2C4, SCL5, SDA5)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c5), okay) && CONFIG_I2C_ASPEED
FUN_DEFINE(I2C5, SCL6, SDA6)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c6), okay) && CONFIG_I2C_ASPEED
FUN_DEFINE(I2C6, SCL7, SDA7)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c7), okay) && CONFIG_I2C_ASPEED
FUN_DEFINE(I2C7, SCL8, SDA8)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c8), okay) && CONFIG_I2C_ASPEED
FUN_DEFINE(I2C8, SCL9, SDA9)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c9), okay) && CONFIG_I2C_ASPEED
FUN_DEFINE(I2C9, SCL10, SDA10)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c10), okay) && CONFIG_I2C_ASPEED
FUN_DEFINE(I2C10, SCL11, SDA11)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c11), okay) && CONFIG_I2C_ASPEED
FUN_DEFINE(I2C11, SCL12, SDA12)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c12), okay) && CONFIG_I2C_ASPEED
FUN_DEFINE(I2C12, SCL13, SDA13)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c13), okay) && CONFIG_I2C_ASPEED
FUN_DEFINE(I2C13, SCL14, SDA14)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c14), okay) && CONFIG_I2C_ASPEED
FUN_DEFINE(I2C14, SCL15, SDA15)
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c15), okay) && CONFIG_I2C_ASPEED
FUN_DEFINE(I2C15, SCL16, SDA16)
#endif

#ifdef CONFIG_DEVICE_I2C
FUN_DEFINE(I2C0, SCL1, SDA1)
FUN_DEFINE(I2C1, SCL2, SDA2)
FUN_DEFINE(I2C2, SCL3, SDA3)
FUN_DEFINE(I2C3, SCL4, SDA4)
FUN_DEFINE(I2C4, SCL5, SDA5)
FUN_DEFINE(I2C5, SCL6, SDA6)
FUN_DEFINE(I2C8, SCL9, SDA9)
FUN_DEFINE(I2C9, SCL10, SDA10)
FUN_DEFINE(I2C10, SCL11, SDA11)
FUN_DEFINE(I2C11, SCL12, SDA12)
FUN_DEFINE(I2C12, SCL13, SDA13)
FUN_DEFINE(I2C13, SCL14, SDA14)
FUN_DEFINE(I2C14, SCL15, SDA15)
FUN_DEFINE(I2C15, SCL16, SDA16)
#endif

#if CONFIG_DEVICE_FMC_SPI
FUN_DEFINE(FWSPIQ, FWSPIQ2, FWSPIQ3)
#endif

#if CONFIG_DEVICE_FMC_SPI
FUN_DEFINE(SPI1Q, SPI1DQ2, SPI1DQ3)
#endif

#if CONFIG_DEVICE_FMC_SPI
FUN_DEFINE(SPI2, SPI2CS0, SPI2CS1, SPI2CK, SPI2DQ0, SPI2DQ1)
FUN_DEFINE(SPI2Q, SPI2DQ2, SPI2DQ3)
#endif
