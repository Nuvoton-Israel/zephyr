/**************************************************************************//**
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __CP_REG_H__
#define __CP_REG_H__

struct CP_T
{
    __IO uint16_t B2CPST0;
    __IO uint16_t B2CPST1;
    __IO uint16_t CP2BNT0;
    __IO uint16_t CP2BNT1;
    __IO uint16_t CPSTATR;
    __IO uint16_t CPCFGR;
    __IO uint16_t LKREG1;
    __IO uint16_t LKREG2;
    __IO uint16_t CP2TIP_INT;
    __IO uint16_t _reserved;
    __IO uint16_t SRAMWINC;
    __IO uint16_t SPI01WINC;
    __IO uint16_t SPI3WINC;
    __IO uint16_t SPIXWINC;
    __IO uint16_t MISCWINC;
};

#endif  /* __CP_REG_H__ */
