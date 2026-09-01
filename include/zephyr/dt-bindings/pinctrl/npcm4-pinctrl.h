/*
 * Copyright (c) 2026 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_NPCM4_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_NPCM4_PINCTRL_H_

/*
 * System Configuration (SCFG) register offsets of the NPCM400.
 *
 * These describe where the pin-mux, pull-up/down and low-voltage control
 * registers live so that the reused Nuvoton pinctrl driver does not have to
 * carry any NPCM400 specific knowledge.
 */
#define NPCM4_SCFG_DEVCNT_OFFSET   0x000
#define NPCM4_SCFG_STRPST_OFFSET   0x001
#define NPCM4_SCFG_RSTCTL_OFFSET   0x002
#define NPCM4_SCFG_DEV_CTL3_OFFSET 0x004
#define NPCM4_SCFG_DEV_CTL4_OFFSET 0x006
#define NPCM4_SCFG_DEVALT10_OFFSET 0x00b
#define NPCM4_SCFG_DEVALT11_OFFSET 0x00c
#define NPCM4_SCFG_DEVALT12_OFFSET 0x00d
#define NPCM4_SCFG_DEVALTCX_OFFSET 0x024
#define NPCM4_SCFG_DEVPU0_OFFSET   0x028
#define NPCM4_SCFG_DEVPD1_OFFSET   0x029
#define NPCM4_SCFG_LV_CTL0_OFFSET  0x02a
#define NPCM4_SCFG_LV_CTL1_OFFSET  0x02b

/*
 * Device Alternate Function (DEVALT) registers.
 *
 * DEVALT0 - DEVALTF occupy offsets 0x00 - 0x0f, i.e. the group index is the
 * register offset.
 */
#define NPCM4_DEVALT_OFFSET(n) (n)

/*
 * Device Pull-Up / Pull-Down Enable registers.
 *
 * Groups 0 and 1 (DEVPU0, DEVPD1) are adjacent at 0x28 - 0x29 while groups 2
 * and 3 are relocated to 0x73 and 0x7b.
 */
#define NPCM4_PUPD_EN_OFFSET(n) (((n) == 2) ? 0x073 : ((n) == 3) ? 0x07b : (0x028 + (n)))

/*
 * Low-Voltage GPIO Control registers.
 *
 * Groups 0 - 3 map to LV_CTL0 - LV_CTL3 at 0x2a - 0x2d while group 4 is
 * relocated to 0x6e.
 */
#define NPCM4_LV_GPIO_CTL_OFFSET(n) (((n) == 4) ? 0x06e : (0x02a + (n)))

/*
 * NPCM400 does not implement the DEVALT_LK pin-mux lock registers. The empty
 * group mask makes the pinctrl driver skip the lock path entirely, so the
 * offset macro below is never evaluated at run time.
 */
#define NPCM4_DEVALT_LK_GROUP_MASK 0
#define NPCM4_DEVALT_LK_OFFSET(n)  NPCM4_DEVALT_OFFSET(n)

/* SCFG register fields */
#define NPCM4_DEVCNT_F_SPI_TRIS         6
#define NPCM4_DEVCNT_HIF_TYP_SEL_FIELD  FIELD(2, 2)
#define NPCM4_DEVCNT_JEN1_HEN           5
#define NPCM4_DEVCNT_JEN0_HEN           4
#define NPCM4_STRPST_TRIST              1
#define NPCM4_STRPST_TEST               2
#define NPCM4_STRPST_JEN1               4
#define NPCM4_STRPST_JEN0               5
#define NPCM4_STRPST_SPI_COMP           7
#define NPCM4_RSTCTL_VCC1_RST_STS       0
#define NPCM4_RSTCTL_DBGRST_STS         1
#define NPCM4_RSTCTL_VCC1_RST_SCRATCH   3
#define NPCM4_RSTCTL_LRESET_PLTRST_MODE 5
#define NPCM4_RSTCTL_HIPRST_MODE        6
#define NPCM4_DEV_CTL3_RNGINT_MD        1
#define NPCM4_DEV_CTL3_FVCC1_PURST_EN   2
#define NPCM4_DEV_CTL3_I3C1_MS          3
#define NPCM4_DEV_CTL3_I3C2_MS          4
#define NPCM4_DEV_CTL3_I3C3_MS          5
#define NPCM4_DEV_CTL3_SIO_CLK_SEL      FIELD(6, 2)
#define NPCM4_DEV_CTL4_F_SPI_SLLK       2
#define NPCM4_DEV_CTL4_SPI_SP_SEL       4
#define NPCM4_DEV_CTL4_WP_IF            5
#define NPCM4_DEV_CTL4_VCC1_RST_LK      6

/* System Glue (GLUE) register offsets */
#define NPCM4_GLUE_SMB_SBD_OFFSET 0x002
#define NPCM4_GLUE_SMB_EEN_OFFSET 0x003
#define NPCM4_GLUE_SDPD0_OFFSET   0x010
#define NPCM4_GLUE_SDPD1_OFFSET   0x012
#define NPCM4_GLUE_SDP_CTS_OFFSET 0x014
#define NPCM4_GLUE_SMB_SEL_OFFSET 0x021
#define NPCM4_GLUE_PSL_CTS_OFFSET 0x027

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_NPCM4_PINCTRL_H_ */
