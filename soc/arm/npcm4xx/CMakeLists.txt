# SPDX-License-Identifier: Apache-2.0
#
# Copyright (c) 2020 Linumiz
# Author: Saravanan Sekar <saravanan@linumiz.com>

add_subdirectory(common)
add_subdirectory(${SOC_SERIES})
# NuveSIOBinGenTool
set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
    COMMAND cp ${PROJECT_BINARY_DIR}/${CONFIG_KERNEL_BIN_NAME}.bin ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/NuveSIOBinGenTool/Input
    COMMAND cp ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/NuveSIOBinGenTool/Xml/_NTC_Internal.xml ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/NuveSIOBinGenTool/Xml/_NTC_Internal_t.xml
    COMMAND sed -i 's/@inputfile/${CONFIG_KERNEL_BIN_NAME}/g' ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/NuveSIOBinGenTool/Xml/_NTC_Internal_t.xml
    COMMAND ${PYTHON_EXECUTABLE} ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/NuveSIOBinGenTool/NuveSIOBinGenTool.py /g ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/NuveSIOBinGenTool/Xml/_NTC_Internal_t.xml
    COMMAND cp ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/NuveSIOBinGenTool/Output/*.bin ${PROJECT_BINARY_DIR}
    COMMAND rm -rf ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/NuveSIOBinGenTool/Xml/_NTC_Internal_t.xml
    COMMAND rm -rf ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/NuveSIOBinGenTool/Output/*.bin
    COMMAND rm -rf ${SOC_DIR}/${ARCH}/${SOC_FAMILY}/common/NuveSIOBinGenTool/Input/*.bin
 )