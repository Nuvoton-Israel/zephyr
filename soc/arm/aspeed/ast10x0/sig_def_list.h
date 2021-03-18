#ifdef CONFIG_STDIO_UART
#if (CONFIG_STDIO_UART == 10)
SIG_DEFINE(TXD11,  P23,    SIG_DESC_SET(0x430, 22))
SIG_DEFINE(RXD11,  T24,    SIG_DESC_SET(0x430, 23))
#endif
#endif

#ifdef CONFIG_ARM_ICE
SIG_DEFINE(ARM_TRST, A11,    SIG_DESC_SET(0x41C, 25))
SIG_DEFINE(ARM_TCK,   D7,     SIG_DESC_SET(0x41C, 26))
SIG_DEFINE(ARM_TDI,   B10,    SIG_DESC_SET(0x41C, 27))
SIG_DEFINE(ARM_TDO,   B9,     SIG_DESC_SET(0x41C, 28))
SIG_DEFINE(ARM_TMS,   D6,     SIG_DESC_SET(0x41C, 29))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(adc0), okay) &&  CONFIG_ADC_ASPEED
SIG_DEFINE(ADC0, E3, SIG_DESC_CLEAR(0x430, 24))
SIG_DEFINE(ADC1, D1, SIG_DESC_CLEAR(0x430, 25))
SIG_DEFINE(ADC2, E2, SIG_DESC_CLEAR(0x430, 26))
SIG_DEFINE(ADC3, F3, SIG_DESC_CLEAR(0x430, 27))
SIG_DEFINE(ADC4, G4, SIG_DESC_CLEAR(0x430, 28))
SIG_DEFINE(ADC5, E1, SIG_DESC_CLEAR(0x430, 29))
SIG_DEFINE(ADC6, F2, SIG_DESC_CLEAR(0x430, 30))
SIG_DEFINE(ADC7, J4, SIG_DESC_CLEAR(0x430, 31))
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(adc1), okay) &&  CONFIG_ADC_ASPEED
SIG_DEFINE(ADC8, L5, SIG_DESC_CLEAR(0x434, 0))
SIG_DEFINE(ADC9, F1, SIG_DESC_CLEAR(0x434, 1))
SIG_DEFINE(ADC10,G2, SIG_DESC_CLEAR(0x434, 2))
SIG_DEFINE(ADC11,H3, SIG_DESC_CLEAR(0x434, 3))
SIG_DEFINE(ADC12,J3, SIG_DESC_CLEAR(0x434, 4))
SIG_DEFINE(ADC13,K4, SIG_DESC_CLEAR(0x434, 5))
SIG_DEFINE(ADC14,L4, SIG_DESC_CLEAR(0x434, 6))
SIG_DEFINE(ADC15,G1, SIG_DESC_CLEAR(0x434, 7))
#endif /* end of "#if CONFIG_DEVICE_ADC" */

#if DT_NODE_HAS_STATUS(DT_NODELABEL(pwm), okay) &&  CONFIG_PWM_ASPEED
SIG_DEFINE(PWM0,  R9, SIG_DESC_SET(0x414, 8))
SIG_DEFINE(PWM1,  P9, SIG_DESC_SET(0x414, 9))
SIG_DEFINE(PWM2,  N10, SIG_DESC_SET(0x414, 10))
SIG_DEFINE(PWM3,  M11, SIG_DESC_SET(0x414, 11))
SIG_DEFINE(PWM4,  T9, SIG_DESC_SET(0x414, 12))
SIG_DEFINE(PWM5,  R10, SIG_DESC_SET(0x414, 13))
SIG_DEFINE(PWM6,  P10, SIG_DESC_SET(0x414, 14))
SIG_DEFINE(PWM7,  T10, SIG_DESC_SET(0x414, 15))
SIG_DEFINE(PWM8,  K1, SIG_DESC_SET(0x410, 0))
SIG_DEFINE(PWM9,  L1, SIG_DESC_SET(0x410, 1))
SIG_DEFINE(PWM10, M1, SIG_DESC_SET(0x410, 2))
SIG_DEFINE(PWM11, N2, SIG_DESC_SET(0x410, 3))
SIG_DEFINE(PWM12, P2, SIG_DESC_SET(0x410, 4))
SIG_DEFINE(PWM13, N1, SIG_DESC_SET(0x410, 5))
SIG_DEFINE(PWM14, P1, SIG_DESC_SET(0x410, 6))
SIG_DEFINE(PWM15, R1, SIG_DESC_SET(0x410, 7))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(tach), okay) &&  CONFIG_TACH_ASPEED
SIG_DEFINE(TACH0, R11, SIG_DESC_SET(0x414, 16))
SIG_DEFINE(TACH1, P11, SIG_DESC_SET(0x414, 17))
SIG_DEFINE(TACH2, T13, SIG_DESC_SET(0x414, 18))
SIG_DEFINE(TACH3, R14, SIG_DESC_SET(0x414, 19))
SIG_DEFINE(TACH4, T14, SIG_DESC_SET(0x414, 20))
SIG_DEFINE(TACH5, T15, SIG_DESC_SET(0x414, 21))
SIG_DEFINE(TACH6, R15, SIG_DESC_SET(0x414, 22))
SIG_DEFINE(TACH7, P14, SIG_DESC_SET(0x414, 23))
SIG_DEFINE(TACH8, N13, SIG_DESC_SET(0x414, 24))
SIG_DEFINE(TACH9, M12, SIG_DESC_SET(0x414, 25))
SIG_DEFINE(TACH10,L11, SIG_DESC_SET(0x414, 26))
SIG_DEFINE(TACH11,R16, SIG_DESC_SET(0x414, 27))
SIG_DEFINE(TACH12,R2, SIG_DESC_SET(0x410, 8))
SIG_DEFINE(TACH13,P3, SIG_DESC_SET(0x410, 9))
SIG_DEFINE(TACH14,N4, SIG_DESC_SET(0x410, 10))
SIG_DEFINE(TACH15,M5, SIG_DESC_SET(0x410, 11))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(jtag0), okay) &&  CONFIG_JTAG_ASPEED
SIG_DEFINE(MTRSTN1, A11,    SIG_DESC_CLEAR(0x41C, 25),    SIG_DESC_SET(0x4BC, 25))
SIG_DEFINE(MTDI1,   D7,     SIG_DESC_CLEAR(0x41C, 26),    SIG_DESC_SET(0x4BC, 26))
SIG_DEFINE(MTCK1,   B10,    SIG_DESC_CLEAR(0x41C, 27),    SIG_DESC_SET(0x4BC, 27))
SIG_DEFINE(MTMS1,   B9,     SIG_DESC_CLEAR(0x41C, 28),    SIG_DESC_SET(0x4BC, 28))
SIG_DEFINE(MTDO1,   D6,     SIG_DESC_CLEAR(0x41C, 29),    SIG_DESC_SET(0x4BC, 29))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(jtag1), okay) &&  CONFIG_JTAG_ASPEED
SIG_DEFINE(MTRSTN2, D10,    SIG_DESC_SET(0x41C, 20))
SIG_DEFINE(MTDI2,   B11,    SIG_DESC_SET(0x41C, 21))
SIG_DEFINE(MTCK2,   A12,    SIG_DESC_SET(0x41C, 22))
SIG_DEFINE(MTMS2,   D8,     SIG_DESC_SET(0x41C, 23))
SIG_DEFINE(MTDO2,   C10,    SIG_DESC_SET(0x41C, 24))
#endif

#ifdef CONFIG_DEVICE_I3C
SIG_DEFINE(I3C1SCL, J14, SIG_DESC_SET(0x418, 16))
SIG_DEFINE(I3C1SDA, H13, SIG_DESC_SET(0x418, 17))
SIG_DEFINE(I3C2SCL, G12, SIG_DESC_SET(0x418, 18))
SIG_DEFINE(I3C2SDA, F12, SIG_DESC_SET(0x418, 19))
SIG_DEFINE(I3C3SCL, E12, SIG_DESC_SET(0x418, 20))
SIG_DEFINE(I3C3SDA, J16, SIG_DESC_SET(0x418, 21))
SIG_DEFINE(I3C4SCL, J15, SIG_DESC_SET(0x418, 22))
SIG_DEFINE(I3C4SDA, H14, SIG_DESC_SET(0x418, 23))
SIG_DEFINE(HVI3C1SCL, K13, SIG_DESC_SET(0x4B8, 8))
SIG_DEFINE(HVI3C1SDA, J12, SIG_DESC_SET(0x4B8, 9))
SIG_DEFINE(HVI3C2SCL, L15, SIG_DESC_SET(0x4B8, 10))
SIG_DEFINE(HVI3C2SDA, K14, SIG_DESC_SET(0x4B8, 11))
SIG_DEFINE(HVI3C3SCL, G11, SIG_DESC_SET(0x4B8, 12))
SIG_DEFINE(HVI3C3SDA, F11, SIG_DESC_SET(0x4B8, 13))
SIG_DEFINE(HVI3C4SCL, K16, SIG_DESC_SET(0x4B8, 14))
SIG_DEFINE(HVI3C4SDA, K15, SIG_DESC_SET(0x4B8, 15))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c0), okay) &&  CONFIG_I2C_ASPEED
SIG_DEFINE(SCL1, P15, SIG_DESC_SET(0x414, 28))
SIG_DEFINE(SDA1, N14, SIG_DESC_SET(0x414, 29))
SIG_DEFINE(SALT1, K1, SIG_DESC_SET(0x4B0, 0))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c1), okay) &&  CONFIG_I2C_ASPEED
SIG_DEFINE(SCL2, M13, SIG_DESC_SET(0x414, 30))
SIG_DEFINE(SDA2, K11, SIG_DESC_SET(0x414, 31))
SIG_DEFINE(SALT2, L1, SIG_DESC_SET(0x4B0, 1))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c2), okay) &&  CONFIG_I2C_ASPEED
SIG_DEFINE(SCL3, P16, SIG_DESC_SET(0x418, 0))
SIG_DEFINE(SDA3, N15, SIG_DESC_SET(0x418, 1))
SIG_DEFINE(SALT3, M1, SIG_DESC_SET(0x4B0, 2))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c3), okay) &&  CONFIG_I2C_ASPEED
SIG_DEFINE(SCL4, M14, SIG_DESC_SET(0x418, 2))
SIG_DEFINE(SDA4, K12, SIG_DESC_SET(0x418, 3))
SIG_DEFINE(SALT4, N2, SIG_DESC_SET(0x4B0, 3))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c4), okay) &&  CONFIG_I2C_ASPEED
SIG_DEFINE(SCL5, J11, SIG_DESC_SET(0x418, 4))
SIG_DEFINE(SDA5, N16, SIG_DESC_SET(0x418, 5))
SIG_DEFINE(SALT5, P2, SIG_DESC_SET(0x4B0, 4))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c5), okay) &&  CONFIG_I2C_ASPEED
SIG_DEFINE(SCL6, M15, SIG_DESC_SET(0x418, 6))
SIG_DEFINE(SDA6, L14, SIG_DESC_SET(0x418, 7))
SIG_DEFINE(SALT6, N1, SIG_DESC_SET(0x4B0, 5))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c6), okay) &&  CONFIG_I2C_ASPEED
SIG_DEFINE(SCL7, K13, SIG_DESC_SET(0x418, 8))
SIG_DEFINE(SDA7, J12, SIG_DESC_SET(0x418, 9))
SIG_DEFINE(SALT7, P1, SIG_DESC_SET(0x4B0, 6))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c7), okay) &&  CONFIG_I2C_ASPEED
SIG_DEFINE(SCL8, L15, SIG_DESC_SET(0x418, 10))
SIG_DEFINE(SDA8, K14, SIG_DESC_SET(0x418, 11))
SIG_DEFINE(SALT8, R1, SIG_DESC_SET(0x4B0, 7))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c8), okay) &&  CONFIG_I2C_ASPEED
SIG_DEFINE(SCL9, G11, SIG_DESC_SET(0x418, 12))
SIG_DEFINE(SDA9, F11, SIG_DESC_SET(0x418, 13))
SIG_DEFINE(SALT9, R2, SIG_DESC_SET(0x4B0, 8), SIG_DESC_CLEAR(0x410, 8))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c9), okay) &&  CONFIG_I2C_ASPEED
SIG_DEFINE(SCL10,K16, SIG_DESC_SET(0x418, 14))
SIG_DEFINE(SDA10,K15, SIG_DESC_SET(0x418, 15))
SIG_DEFINE(SALT10,P3, SIG_DESC_SET(0x4B0, 9), SIG_DESC_CLEAR(0x410, 9))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c10), okay) &&  CONFIG_I2C_ASPEED
SIG_DEFINE(SCL11,J14, SIG_DESC_CLEAR(0x418, 16), SIG_DESC_SET(0x4B8, 16))
SIG_DEFINE(SDA11,H13, SIG_DESC_CLEAR(0x418, 17), SIG_DESC_SET(0x4B8, 17))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c11), okay) &&  CONFIG_I2C_ASPEED
SIG_DEFINE(SCL12,G12, SIG_DESC_CLEAR(0x418, 18), SIG_DESC_SET(0x4B8, 18))
SIG_DEFINE(SDA12,F12, SIG_DESC_CLEAR(0x418, 19), SIG_DESC_SET(0x4B8, 19))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c12), okay) &&  CONFIG_I2C_ASPEED
SIG_DEFINE(SCL13,E12, SIG_DESC_CLEAR(0x418, 20), SIG_DESC_SET(0x4B8, 20))
SIG_DEFINE(SDA13,J16, SIG_DESC_CLEAR(0x418, 21), SIG_DESC_SET(0x4B8, 21))
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c13), okay) &&  CONFIG_I2C_ASPEED
SIG_DEFINE(SCL14,J15, SIG_DESC_CLEAR(0x418, 22), SIG_DESC_SET(0x4B8, 22))
SIG_DEFINE(SDA14,H14, SIG_DESC_CLEAR(0x418, 23), SIG_DESC_SET(0x4B8, 23))
#endif

#ifdef CONFIG_DEVICE_GPIO
/* UART1 */
SIG_DEFINE(GPIOL0, G13, SIG_DESC_SET(0x430, 24))
SIG_DEFINE(GPIOL1, F13, SIG_DESC_SET(0x430, 24))
/* UART6 */
SIG_DEFINE(GPIOM0, E14, SIG_DESC_SET(0x434, 0), SIG_DESC_SET(0x694, 16))
SIG_DEFINE(GPIOM1, D14, SIG_DESC_SET(0x434, 1), SIG_DESC_SET(0x694, 17))
/* UART7 */
SIG_DEFINE(GPIOM2, F16, SIG_DESC_SET(0x434, 2), SIG_DESC_SET(0x694, 18))
SIG_DEFINE(GPIOM3, F15, SIG_DESC_SET(0x434, 3), SIG_DESC_SET(0x694, 19))
/* UART8 */
SIG_DEFINE(GPIOM4, E15, SIG_DESC_SET(0x434, 4), SIG_DESC_SET(0x694, 20))
SIG_DEFINE(GPIOM5, E16, SIG_DESC_SET(0x434, 5), SIG_DESC_SET(0x694, 21))
/* UART5 */
SIG_DEFINE(GPIOM6, D15, SIG_DESC_SET(0x434, 6), SIG_DESC_SET(0x694, 22))
SIG_DEFINE(GPIOM7, C14, SIG_DESC_SET(0x434, 7), SIG_DESC_SET(0x694, 23))
/* ESPI */
SIG_DEFINE(GPION4, B16, SIG_DESC_SET(0x434, 4), SIG_DESC_SET(0x694, 20))
SIG_DEFINE(GPION5, A15, SIG_DESC_SET(0x434, 5), SIG_DESC_SET(0x694, 21))
SIG_DEFINE(GPION6, D12, SIG_DESC_SET(0x434, 6), SIG_DESC_SET(0x694, 22))
SIG_DEFINE(GPION7, E11, SIG_DESC_SET(0x434, 7), SIG_DESC_SET(0x694, 23))
SIG_DEFINE(GPIOO0, A14, SIG_DESC_SET(0x434, 4), SIG_DESC_SET(0x694, 20))
SIG_DEFINE(GPIOO1, B13, SIG_DESC_SET(0x434, 5), SIG_DESC_SET(0x694, 21))
SIG_DEFINE(GPIOO2, C12, SIG_DESC_SET(0x434, 6), SIG_DESC_SET(0x694, 22))
SIG_DEFINE(GPIOO3, A13, SIG_DESC_SET(0x434, 7), SIG_DESC_SET(0x694, 23))
/* FWSPI */
SIG_DEFINE(GPIOQ5, B7, SIG_DESC_SET(0x434, 5), SIG_DESC_SET(0x694, 21))
SIG_DEFINE(GPIOQ6, C5, SIG_DESC_SET(0x434, 6), SIG_DESC_SET(0x694, 22))
SIG_DEFINE(GPIOQ7, C4, SIG_DESC_SET(0x434, 7), SIG_DESC_SET(0x694, 23))
SIG_DEFINE(GPIOR0, A8, SIG_DESC_SET(0x434, 4), SIG_DESC_SET(0x694, 20))
SIG_DEFINE(GPIOR1, B6, SIG_DESC_SET(0x434, 5), SIG_DESC_SET(0x694, 21))
/* SPI1 */
SIG_DEFINE(GPIOR4, A7, SIG_DESC_SET(0x434, 4), SIG_DESC_SET(0x694, 20))
SIG_DEFINE(GPIOR5, A6, SIG_DESC_SET(0x434, 5), SIG_DESC_SET(0x694, 21))
SIG_DEFINE(GPIOR6, A4, SIG_DESC_SET(0x434, 6), SIG_DESC_SET(0x694, 22))
SIG_DEFINE(GPIOR7, B3, SIG_DESC_SET(0x434, 7), SIG_DESC_SET(0x694, 23))
SIG_DEFINE(GPIOS0, A3, SIG_DESC_SET(0x434, 7), SIG_DESC_SET(0x694, 23))
/* ADC */
SIG_DEFINE(GPIT0, E3, SIG_DESC_SET(0x430, 24))
SIG_DEFINE(GPIT1, D1, SIG_DESC_SET(0x430, 25))
SIG_DEFINE(GPIT2, E2, SIG_DESC_SET(0x430, 26))
SIG_DEFINE(GPIT3, F3, SIG_DESC_SET(0x430, 27))
SIG_DEFINE(GPIT4, G4, SIG_DESC_SET(0x430, 28))
SIG_DEFINE(GPIT5, E1, SIG_DESC_SET(0x430, 29))
SIG_DEFINE(GPIT6, F2, SIG_DESC_SET(0x430, 30))
SIG_DEFINE(GPIT7, J4, SIG_DESC_SET(0x430, 31))
SIG_DEFINE(GPIU0, L5, SIG_DESC_SET(0x434, 0))
SIG_DEFINE(GPIU1, F1, SIG_DESC_SET(0x434, 1))
SIG_DEFINE(GPIU2, G2, SIG_DESC_SET(0x434, 2))
SIG_DEFINE(GPIU3, H3, SIG_DESC_SET(0x434, 3))
SIG_DEFINE(GPIU4, J3, SIG_DESC_SET(0x434, 4))
SIG_DEFINE(GPIU5, K4, SIG_DESC_SET(0x434, 5))
SIG_DEFINE(GPIU6, L4, SIG_DESC_SET(0x434, 6))
SIG_DEFINE(GPIU7, G1, SIG_DESC_SET(0x434, 7))
#endif

#ifdef CONFIG_DEVICE_SGPIOM
SIG_DEFINE(SGPMCLK,D16, SIG_DESC_SET(0x41C, 8))
SIG_DEFINE(SGPMLD, C15, SIG_DESC_SET(0x41C, 9))
SIG_DEFINE(SGPMO,  C16, SIG_DESC_SET(0x41C, 10))
SIG_DEFINE(SGPMI,  B15, SIG_DESC_SET(0x41C, 11))
#endif

#if CONFIG_DEVICE_FMC_SPI
SIG_DEFINE(FWSPIQ2 ,B5, SIG_DESC_SET(0x430, 10))
SIG_DEFINE(FWSPIQ3, C3, SIG_DESC_SET(0x430, 11))
#endif

#if CONFIG_DEVICE_FMC_SPI
SIG_DEFINE(SPI1DQ2, B2, SIG_DESC_SET(0x430, 17))
SIG_DEFINE(SPI1DQ3, A2, SIG_DESC_SET(0x430, 18))
#endif

#if CONFIG_DEVICE_FMC_SPI
SIG_DEFINE(SPI2CS0, C8, SIG_DESC_SET(0x41C, 30))
SIG_DEFINE(SPI2CS1, C7, SIG_DESC_SET(0x41C, 31))
SIG_DEFINE(SPI2CK, C6, SIG_DESC_SET(0x430, 0))
SIG_DEFINE(SPI2DQ0, D5, SIG_DESC_SET(0x430, 1))
SIG_DEFINE(SPI2DQ1, D4, SIG_DESC_SET(0x430, 2))
SIG_DEFINE(SPI2DQ2, A9, SIG_DESC_SET(0x430, 3))
SIG_DEFINE(SPI2DQ3, B8, SIG_DESC_SET(0x430, 4))
#endif

