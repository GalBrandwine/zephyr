# ST Microelectronics ISM330BX 6-axis IMU sensor driver
#
# Copyright (c) 2023 STMicroelectronics
#
# SPDX-License-Identifier: Apache-2.0
#
zephyr_library()
zephyr_library_sources_ifdef(CONFIG_ISM330BX ism330bx.c)
zephyr_library_sources_ifdef(CONFIG_ISM330BX_LOG_LEVEL_DBG ism330bx_devtools.c)
zephyr_library_sources_ifdef(CONFIG_ISM330BX_SENSORHUB  ism330bx_shub.c)
zephyr_library_sources_ifdef(CONFIG_ISM330BX_TRIGGER    ism330bx_trigger.c)
zephyr_library_include_directories(../stmemsc)
