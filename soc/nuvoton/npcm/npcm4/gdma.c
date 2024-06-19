/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "gdma.h"
#include "zephyr/sys/util.h"

/**
 * @brief Set a block of memory to a specified value using GDMA.
 *
 * @param dat Pointer to the destination memory block.
 * @param set_val Value to set.
 * @param setlen Length of the memory block to set.
 */
void gdma_memset_u8(uint8_t *dat, uint8_t set_val, uint32_t setlen)
{
	uint8_t val = set_val;

	if (setlen == 0) {
		return;
	}

	GDMA_SRCB0 = (uint32_t)&val;
	GDMA_DSTB0 = (uint32_t)dat;
	GDMA_TCNT0 = setlen;
	GDMA_CTL0 = 0x10081;

	while (GDMA_CTL0 & 0x1) {
	}
	GDMA_CTL0 = 0;
}

/**
 * @brief Copy a block of memory using GDMA (byte-wise).
 *
 * @param dst Pointer to the destination memory block.
 * @param src Pointer to the source memory block.
 * @param cpylen Length of the memory block to copy.
 */
void gdma_memcpy_u8(uint8_t *dst, uint8_t *src, uint32_t cpylen)
{
	if (cpylen == 0) {
		return;
	}

	GDMA_SRCB0 = (uint32_t)src;
	GDMA_DSTB0 = (uint32_t)dst;
	GDMA_TCNT0 = cpylen;
	GDMA_CTL0 = 0x10001;

	while (GDMA_CTL0 & 0x1) {
	}
	GDMA_CTL0 = 0;
}

/**
 * @brief Copy a block of memory using GDMA (word-wise).
 *
 * @param dst Pointer to the destination memory block.
 * @param src Pointer to the source memory block.
 * @param cpylen Length of the memory block to copy (in bytes).
 */
void gdma_memcpy_u32(uint8_t *dst, uint8_t *src, uint32_t cpylen)
{
	if (cpylen == 0) {
		return;
	}

	GDMA_SRCB0 = (uint32_t)src;
	GDMA_DSTB0 = (uint32_t)dst;
	GDMA_TCNT0 = cpylen >> 2;
	GDMA_CTL0 = 0x12001;

	while (GDMA_CTL0 & 0x1) {
	}
	GDMA_CTL0 = 0;
}

/**
 * @brief Copy a block of memory using GDMA with burst mode (word-wise).
 *
 * @param dst Pointer to the destination memory block.
 * @param src Pointer to the source memory block.
 * @param cpylen Length of the memory block to copy (in bytes).
 */
void gdma_memcpy_burst_u32(uint8_t *dst, uint8_t *src, uint32_t cpylen)
{
	uint32_t rlen;

	if (cpylen == 0) {
		return;
	}

	/* Source and destination address must be 16-byte aligned */
	if (((uint32_t)src & 0x0F) || ((uint32_t)dst & 0x0F)) {
		gdma_memcpy_u8(dst, src, cpylen);
		return;
	}

	/* Aligned 64-byte length */
	rlen = cpylen & GENMASK(31, 6);
	if (rlen) {
		FIU0_BURST_CFG = 0x0B;

		GDMA_SRCB0 = (uint32_t)src;
		GDMA_DSTB0 = (uint32_t)dst;
		GDMA_TCNT0 = rlen >> 4;
		GDMA_CTL0 = 0x12201;

		while (GDMA_CTL0 & 0x1) {
		}
		GDMA_CTL0 = 0;

		FIU0_BURST_CFG = 0x03;

		src += rlen;
		dst += rlen;
		cpylen -= rlen;
	}

	/* Remaining length */
	if (cpylen) {
		gdma_memcpy_u8(dst, src, cpylen);
	}
}

/**
 * @brief Copy a block of memory using GDMA with fixed destination address (word-wise).
 *
 * @param dst Pointer to the destination memory block.
 * @param src Pointer to the source memory block.
 * @param cpylen Length of the memory block to copy (in bytes).
 */
void gdma_memcpy_u32_dstfix(uint8_t *dst, uint8_t *src, uint32_t cpylen)
{
	if (cpylen == 0) {
		return;
	}

	GDMA_SRCB0 = (uint32_t)src;
	GDMA_DSTB0 = (uint32_t)dst;
	GDMA_TCNT0 = cpylen >> 2;
	GDMA_CTL0 = 0x12041;

	while (GDMA_CTL0 & 0x1) {
	}
	GDMA_CTL0 = 0;
}

/**
 * @brief Copy a block of memory using GDMA with fixed source address (word-wise).
 *
 * @param dst Pointer to the destination memory block.
 * @param src Pointer to the source memory block.
 * @param cpylen Length of the memory block to copy (in bytes).
 */
void gdma_memcpy_u32_srcfix(uint8_t *dst, uint8_t *src, uint32_t cpylen)
{
	if (cpylen == 0) {
		return;
	}

	GDMA_SRCB0 = (uint32_t)src;
	GDMA_DSTB0 = (uint32_t)dst;
	GDMA_TCNT0 = cpylen >> 2;
	GDMA_CTL0 = 0x12081;

	while (GDMA_CTL0 & 0x1) {
	}
	GDMA_CTL0 = 0;
}
