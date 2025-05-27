/**
 * @file drivers/sensor.h
 *
 * @brief Public APIs for the sensor driver.
 */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef INCLUDE_ISM330BX_SENSOR_H_
#define INCLUDE_ISM330BX_SENSOR_H_
#include <zephyr/drivers/sensor.h>
enum ism330bx_channel {
	ISM330BX_CHAN_INTERRUPT_GENERATION = SENSOR_CHAN_COMMON_COUNT,
};
enum ism330bx_attribute {
	ISM330BX_ATTR_SAMPLING_MODE = SENSOR_ATTR_COMMON_COUNT,
	ORIENTATION_DETECTION_EVENT,
	ORIENTATION_DETECTION_LOWPASS_FILTER_50_DEG_THRESH,
};
enum ism330bx_trigger_type {
	SENSOR_TRIG_SIGNIFICANT_MOTION = SENSOR_TRIG_COMMON_COUNT,
	SENSOR_TRIG_WAKE_UP,
};
#endif /* INCLUDE_ISM330BX_SENSOR_H_ */
