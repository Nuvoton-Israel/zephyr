# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2020 Linumiz
# Author: Saravanan Sekar <saravanan@linumiz.com>

add_subdirectory(common)
add_subdirectory(${SOC_SERIES})

zephyr_include_directories(.)

# ImageGenerator
if (DEFINED CONFIG_SECURE_BOOT_ENABLE_NPCM4XX)
set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
    COMMAND cp ${PROJECT_BINARY_DIR}/${CONFIG_KERNEL_BIN_NAME}.bin ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/Input
    COMMAND cp ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/Xml/_NTC_config.xml ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/Xml/_NTC_config_t.xml
    COMMAND sed -i 's/@inputfile/${CONFIG_KERNEL_BIN_NAME}/g' ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/Xml/_NTC_config_t.xml
    COMMAND sed -i 's/@gen_otp/1/g' ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/Xml/_NTC_config_t.xml
    COMMAND sed -i 's/@recover_image_offset/${CONFIG_BACKUP_IMAGE_OFFSET_NPCM4XX}/g' ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/Xml/_NTC_config_t.xml
    COMMAND sed -i 's/@secure_boot/1/g' ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/Xml/_NTC_config_t.xml
    COMMAND sed -i 's/@secure_level/1/g' ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/Xml/_NTC_config_t.xml
    COMMAND sed -i 's/@longkey_used/1/g' ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/Xml/_NTC_config_t.xml
    COMMAND sed -i 's/@longkey_sel/1/g' ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/Xml/_NTC_config_t.xml
    COMMAND sed -i 's/@not_do_backup/${CONFIG_DONT_UPDATE_BACKUP_IMAGE_NPCM4XX}/g' ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/Xml/_NTC_config_t.xml
    COMMAND ${PYTHON_EXECUTABLE} ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/ImageGenerator.py /g ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/Xml/_NTC_config_t.xml
    COMMAND cp ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/Output/*.bin ${PROJECT_BINARY_DIR}
    COMMAND rm -rf ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/Xml/_NTC_config_t.xml
    COMMAND rm -rf ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/Output/*.bin
    COMMAND rm -rf ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/Input/*.bin
)
else ()
set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
    COMMAND cp ${PROJECT_BINARY_DIR}/${CONFIG_KERNEL_BIN_NAME}.bin ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/Input
    COMMAND cp ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/Xml/_NTC_config.xml ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/Xml/_NTC_config_t.xml
    COMMAND sed -i 's/@inputfile/${CONFIG_KERNEL_BIN_NAME}/g' ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/Xml/_NTC_config_t.xml
    COMMAND sed -i 's/@gen_otp/0/g' ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/Xml/_NTC_config_t.xml
    COMMAND sed -i 's/@recover_image_offset/0x40/g' ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/Xml/_NTC_config_t.xml
    COMMAND sed -i 's/@secure_boot/0/g' ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/Xml/_NTC_config_t.xml
    COMMAND sed -i 's/@secure_level/0/g' ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/Xml/_NTC_config_t.xml
    COMMAND sed -i 's/@longkey_used/0/g' ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/Xml/_NTC_config_t.xml
    COMMAND sed -i 's/@longkey_sel/0/g' ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/Xml/_NTC_config_t.xml
    COMMAND sed -i 's/@not_do_backup/1/g' ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/Xml/_NTC_config_t.xml
    COMMAND ${PYTHON_EXECUTABLE} ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/ImageGenerator.py /g ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/Xml/_NTC_config_t.xml
    COMMAND cp ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/Output/*.bin ${PROJECT_BINARY_DIR}
    COMMAND rm -rf ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/Xml/_NTC_config_t.xml
    COMMAND rm -rf ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/Output/*.bin
    COMMAND rm -rf ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/ImageGenerator/Input/*.bin
 )
 endif()
