# ST Microelectronics ISM330BX 6-axis IMU sensor driver
#
# Copyright (c) 2023 STMicroelectronics
#
# SPDX-License-Identifier: Apache-2.0
#
zephyr_library()

zephyr_library_sources(ism330bx.c)
zephyr_library_sources_ifdef(CONFIG_ISM330BX_SENSORHUB  ism330bx_shub.c)
zephyr_library_sources_ifdef(CONFIG_ISM330BX_TRIGGER    ism330bx_trigger.c)
# zephyr_library_sources_ifdef(CONFIG_SENSOR_ASYNC_API ism330bx_rtio.c ism330bx_decoder.c) // WIP
# zephyr_library_sources_ifdef(CONFIG_LSM6DSV16X_STREAM ism330bx_rtio_stream.c) // WIP

zephyr_library_include_directories(../stmemsc)
