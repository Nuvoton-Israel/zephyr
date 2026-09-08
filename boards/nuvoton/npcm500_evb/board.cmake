# Copyright (c) 2026 Nuvoton Technology Corporation.
#
# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=npcm500" "--speed=4000")
board_runner_args(jlink "--file=./build/zephyr/${CONFIG_KERNEL_BIN_NAME}.npcm.bin")
board_runner_args(jlink "--reset-after-load")
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
