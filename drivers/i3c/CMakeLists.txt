# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(i3c_common.c)
add_subdirectory_ifdef(CONFIG_I3C_SLAVE		slave)

zephyr_library_sources_ifdef(CONFIG_I3C_ASPEED	i3c_global_aspeed.c)
zephyr_library_sources_ifdef(CONFIG_I3C_ASPEED	i3c_aspeed.c)

zephyr_library_sources_ifdef(CONFIG_I3C_NPCM4XX	i3c_npcm4xx.c)
add_subdirectory_ifdef(CONFIG_I3C_NPCM4XX	npcm4xx)
zephyr_include_directories_ifdef(
	CONFIG_I3C_NPCM4XX
	${ZEPHYR_BASE}/drivers/i3c/npcm4xx
)

zephyr_library_sources_ifdef(CONFIG_I3C_SHELL	i3c_shell.c)
