#
# Copyright (c) 2024 Nuvoton Technology Corporation.
#
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(
  .
  ${ZEPHYR_BASE}/drivers
)

set(SOC_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker.ld CACHE INTERNAL "")
