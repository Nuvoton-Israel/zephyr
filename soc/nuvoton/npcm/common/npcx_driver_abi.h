/*
 * Copyright (c) 2026 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _NUVOTON_NPCM_NPCX_DRIVER_ABI_H
#define _NUVOTON_NPCM_NPCX_DRIVER_ABI_H

/*
 * The NPCM400 reuses the Nuvoton npcx peripheral drivers instead of carrying
 * duplicates of them. Those drivers refer to their registers through a set of
 * NPCX_ prefixed macros.
 *
 * This header is the single place that binds that driver-facing naming to the
 * NPCM400 description held in the include/zephyr/dt-bindings npcm4 headers.
 * Nothing below adds hardware knowledge: every right-hand side resolves to a
 * devicetree binding constant, so the devicetree and the drivers always agree
 * on the same numbers.
 *
 * Keep this file free of #ifdef and of literal register values.
 */

/* System Configuration (SCFG) accessors */
#define NPCX_DEV_CTL     NPCM4_DEV_CTL
#define NPCX_DEVALT      NPCM4_DEVALT
#define NPCX_DEVALT_LK   NPCM4_DEVALT_LK
#define NPCX_PUPD_EN     NPCM4_PUPD_EN
#define NPCX_LV_GPIO_CTL NPCM4_LV_GPIO_CTL

#define NPCX_DEVALT_LK_GROUP_MASK NPCM4_DEVALT_LK_GROUP_MASK

#define NPCX_DEVCNT_HIF_TYP_SEL_FIELD NPCM4_DEVCNT_HIF_TYP_SEL_FIELD
#define NPCX_DEV_CTL4_WP_IF           NPCM4_DEV_CTL4_WP_IF

/* Development and Debugger Interface (DDI) accessors */
#define NPCX_DBGCTRL   NPCM4_DBGCTRL
#define NPCX_DBGFRZEN1 NPCM4_DBGFRZEN1
#define NPCX_DBGFRZEN2 NPCM4_DBGFRZEN2
#define NPCX_DBGFRZEN3 NPCM4_DBGFRZEN3
#define NPCX_DBGFRZEN4 NPCM4_DBGFRZEN4

#define NPCX_DBGFRZEN3_GLBL_FRZ_DIS NPCM4_DBGFRZEN3_GLBL_FRZ_DIS

/* System Glue (GLUE) fields */
#define NPCX_PSL_CTS_MODE_BIT  NPCM4_PSL_CTS_MODE_BIT
#define NPCX_PSL_CTS_EVENT_BIT NPCM4_PSL_CTS_EVENT_BIT

/* Power Management Controller (PMC) accessors and fields */
#define NPCX_PWDWN_CTL NPCM4_PWDWN_CTL

#define NPCX_PMCSR_DI_INSTW        NPCM4_PMCSR_DI_INSTW
#define NPCX_PMCSR_DHF             NPCM4_PMCSR_DHF
#define NPCX_PMCSR_IDLE            NPCM4_PMCSR_IDLE
#define NPCX_PMCSR_NWBI            NPCM4_PMCSR_NWBI
#define NPCX_PMCSR_OHFC            NPCM4_PMCSR_OHFC
#define NPCX_PMCSR_OLFC            NPCM4_PMCSR_OLFC
#define NPCX_DISIDL_CTL_RAM_DID    NPCM4_DISIDL_CTL_RAM_DID
#define NPCX_ENIDL_CTL_ADC_LFSL    NPCM4_ENIDL_CTL_ADC_LFSL
#define NPCX_ENIDL_CTL_LP_WK_CTL   NPCM4_ENIDL_CTL_LP_WK_CTL
#define NPCX_ENIDL_CTL_PECI_ENI    NPCM4_ENIDL_CTL_PECI_ENI
#define NPCX_ENIDL_CTL_ADC_ACC_DIS NPCM4_ENIDL_CTL_ADC_ACC_DIS

/* Core Domain Clock Generator (CDCG) fields */
#define NPCX_HFCGCTRL_LOAD         NPCM4_HFCGCTRL_LOAD
#define NPCX_HFCGCTRL_LOCK         NPCM4_HFCGCTRL_LOCK
#define NPCX_HFCGCTRL_CLK_CHNG     NPCM4_HFCGCTRL_CLK_CHNG
#define NPCX_LFCGCTL2_XT_OSC_SL_EN NPCM4_LFCGCTL2_XT_OSC_SL_EN

/* Multi-Input Wake-Up Unit (MIWU) accessors */
#define NPCX_WKEDG  NPCM4_WKEDG
#define NPCX_WKAEDG NPCM4_WKAEDG
#define NPCX_WKPND  NPCM4_WKPND
#define NPCX_WKPCL  NPCM4_WKPCL
#define NPCX_WKEN   NPCM4_WKEN
#define NPCX_WKINEN NPCM4_WKINEN
#define NPCX_WKMOD  NPCM4_WKMOD

/* Universal Asynchronous Receiver-Transmitter (UART) fields */
#define NPCX_UICTRL_TBE NPCM4_UICTRL_TBE
#define NPCX_UICTRL_RBF NPCM4_UICTRL_RBF
#define NPCX_UICTRL_ETI NPCM4_UICTRL_ETI
#define NPCX_UICTRL_ERI NPCM4_UICTRL_ERI
#define NPCX_UICTRL_EEI NPCM4_UICTRL_EEI

#define NPCX_USTAT_PE   NPCM4_USTAT_PE
#define NPCX_USTAT_FE   NPCM4_USTAT_FE
#define NPCX_USTAT_DOE  NPCM4_USTAT_DOE
#define NPCX_USTAT_ERR  NPCM4_USTAT_ERR
#define NPCX_USTAT_BKD  NPCM4_USTAT_BKD
#define NPCX_USTAT_RB9  NPCM4_USTAT_RB9
#define NPCX_USTAT_XMIP NPCM4_USTAT_XMIP

#define NPCX_UFRS_CHAR_FIELD NPCM4_UFRS_CHAR_FIELD
#define NPCX_UFRS_STP        NPCM4_UFRS_STP
#define NPCX_UFRS_XB9        NPCM4_UFRS_XB9
#define NPCX_UFRS_PSEL_FIELD NPCM4_UFRS_PSEL_FIELD
#define NPCX_UFRS_PEN        NPCM4_UFRS_PEN

#define NPCX_UFRS_CHAR_DATA_BIT_8 NPCM4_UFRS_CHAR_DATA_BIT_8
#define NPCX_UFRS_CHAR_DATA_BIT_7 NPCM4_UFRS_CHAR_DATA_BIT_7

#define NPCX_UMDSL_FIFO_MD NPCM4_UMDSL_FIFO_MD
#define NPCX_UMDSL_ETD     NPCM4_UMDSL_ETD
#define NPCX_UMDSL_ERD     NPCM4_UMDSL_ERD

#define NPCX_UTXFLV_TFL NPCM4_UTXFLV_TFL
#define NPCX_URXFLV_RFL NPCM4_URXFLV_RFL

/*
 * The extended FIFO code path of the npcx serial driver names its constants
 * after the npck series that introduced it.
 */
#define NPCK_FIFO_EN      NPCM4_UFCTRL_FIFOEN
#define NPCK_SZ_UART_FIFO NPCM4_SZ_UART_FIFO

/* Pulse Width Modulator (PWM) fields */
#define NPCX_PWMCTL_INVP            NPCM4_PWMCTL_INVP
#define NPCX_PWMCTL_CKSEL           NPCM4_PWMCTL_CKSEL
#define NPCX_PWMCTL_HB_DC_CTL_FIELD NPCM4_PWMCTL_HB_DC_CTL_FIELD
#define NPCX_PWMCTL_PWR             NPCM4_PWMCTL_PWR
#define NPCX_PWMCTLEX_FCK_SEL_FIELD NPCM4_PWMCTLEX_FCK_SEL_FIELD
#define NPCX_PWMCTLEX_OD_OUT        NPCM4_PWMCTLEX_OD_OUT

/* Clock tree limits consumed by the npcx clock controller build assertions */
#define MAX_OFMCLK    NPCM4_MAX_OFMCLK
#define MAX_APB_CLOCK NPCM4_MAX_APB_CLOCK

#endif /* _NUVOTON_NPCM_NPCX_DRIVER_ABI_H */
