/* ST Microelectronics ISM330BX 6-axis IMU sensor driver
 *
 * Copyright (c) 2023 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/ism330bx.pdf
 */

#define DT_DRV_COMPAT st_ism330bx

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <string.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/pm/device.h>

#include "ism330bx.h"
#include "ism330bx_sensor.h"
#if defined(CONFIG_ISM330BX_LOG_LEVEL_DBG)
#include "ism330bx_devtools.h"
#endif /* CONFIG_ISM330BX_LOG_LEVEL_DBG */
LOG_MODULE_REGISTER(ism330bx, CONFIG_ISM330BX_LOG_LEVEL);

// Forward declarations
static int ism330bx_init_chip(const struct device *dev);
static int ism330bx_accel_channel_get(enum sensor_channel chan, struct sensor_value *val,
				      struct ism330bx_data *data);

static int ism330bx_turn_on(const struct device *dev)
{
	LOG_WRN("Work in progress. Need to configure ISM to be turned on");
}

static int ism330bx_turn_off(const struct device *dev)
{
	LOG_WRN("Work in progress. Need to configure ISM to be turned off");
}

/**
 * @brief  Power management API implementation
 *
 * https://docs.zephyrproject.org/latest/services/pm/device.html
 *
 * @param  dev      const struct device *dev
 * @param  action      pm_device_action
 * @retval          interface status (MANDATORY: return 0 -> no Error)
 * @return rc
 */
static int ism330bx_pm_action(const struct device *dev, enum pm_device_action action)
{
	int rc = 0;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
#ifndef CONFIG_ISM330BX_TRIGGER
		LOG_DBG("ISM330BX: PM_DEVICE_ACTION_SUSPEND");
		rc = ism330bx_turn_off(dev);
#endif
		break;

	case PM_DEVICE_ACTION_RESUME:
		LOG_DBG("ISM330BX: PM_DEVICE_ACTION_RESUME");
		rc = ism330bx_turn_on(dev);
		break;
	case PM_DEVICE_ACTION_TURN_ON:
		LOG_DBG("PM_DEVICE_ACTION_TURN_ON. This will turn on the power sources");
		rc = ism330bx_turn_on(dev);
		break;

	case PM_DEVICE_ACTION_TURN_OFF:
#ifndef CONFIG_ISM330BX_TRIGGER
		LOG_DBG("ISM330BX: PM_DEVICE_ACTION_TURN_OFF");
		rc = ism330bx_turn_off(dev);
#endif
		break;

	default:
		LOG_ERR("Got unknown power management action");
		rc = -EINVAL;
		break;
	}
	return rc;
}

/*
 * values taken from ism330bx_data_rate_t in hal/st module. The mode/accuracy
 * should be selected through accel-odr property in DT
 */
static const float ism330bx_odr_map[3][13] = {
	/* High Accuracy off */
	{0.0f, 1.875f, 7.5f, 15.0f, 30.0f, 60.0f, 120.0f, 240.0f, 480.0f, 960.0f, 1920.0f, 3840.0f,
	 7680.0f},

	/* High Accuracy 1 */
	{0.0f, 1.875f, 7.5f, 15.625f, 31.25f, 62.5f, 125.0f, 250.0f, 500.0f, 1000.0f, 2000.0f,
	 4000.0f, 8000.0f},

	/* High Accuracy 2 */
	{0.0f, 1.875f, 7.5f, 12.5f, 25.0f, 50.0f, 100.0f, 200.0f, 400.0f, 800.0f, 1600.0f, 3200.0f,
	 6400.0f},
};

static int ism330bx_freq_to_odr_val(const struct device *dev, enum sensor_channel channel,
				    uint16_t freq)
{
	const struct ism330bx_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;

	int8_t mode;
	size_t i;

	switch (channel) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		ism330bx_xl_data_rate_t xl_odr;
		if (ism330bx_xl_data_rate_get(ctx, &xl_odr) < 0) {
			return -EINVAL;
		}
		mode = (xl_odr >> 4) & 0xf;

		for (i = 0; i < ARRAY_SIZE(ism330bx_odr_map[mode]); i++) {
			if (freq <= ism330bx_odr_map[mode][i]) {
				LOG_DBG("mode idx: %d - odr idx: %d. freq: %f", mode, i,
					ism330bx_odr_map[mode][i]);
				return i;
			}
		}
		break;
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_GYRO_XYZ:
		ism330bx_gy_data_rate_t gy_odr;
		if (ism330bx_gy_data_rate_get(ctx, &gy_odr) < 0) {
			return -EINVAL;
		}
		mode = (gy_odr >> 4) & 0xf;

		for (i = 0; i < ARRAY_SIZE(ism330bx_odr_map[mode]); i++) {
			if (freq <= ism330bx_odr_map[mode][i]) {
				LOG_DBG("mode idx: %d - odr idx: %d. freq: %f", mode, i,
					ism330bx_odr_map[mode][i]);
				return i;
			}
		}
		break;
	default:
		LOG_ERR("Unsupported channel for ODR getting");
		return -ENOTSUP;
	}

	return -EINVAL;
}

static const uint16_t ism330bx_accel_fs_map[] = {2, 4, 8, 16};

static int ism330bx_accel_range_to_fs_val(int32_t range)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(ism330bx_accel_fs_map); i++) {
		if (range == ism330bx_accel_fs_map[i]) {
			return i;
		}
	}

	return -EINVAL;
}

static const uint16_t ism330bx_gyro_fs_map[] = {125, 250, 500, 1000, 2000, 0,   0,
						0,   0,   0,   0,    0,    4000};
static const uint16_t ism330bx_gyro_fs_sens[] = {1, 2, 4, 8, 16, 0, 0, 0, 0, 0, 0, 0, 32};

static int ism330bx_gyro_range_to_fs_val(int32_t range)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(ism330bx_gyro_fs_map); i++) {
		if (range == ism330bx_gyro_fs_map[i]) {
			return i;
		}
	}

	return -EINVAL;
}

static int ism330bx_accel_set_fs_raw(const struct device *dev, uint8_t fs)
{
	const struct ism330bx_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	struct ism330bx_data *data = dev->data;
	ism330bx_xl_full_scale_t val;

	switch (fs) {
	case 0:
		val = ISM330BX_2g;
		break;
	case 1:
		val = ISM330BX_4g;
		break;
	case 2:
		val = ISM330BX_8g;
		break;
	default:
		return -EIO;
	}

	if (ism330bx_xl_full_scale_set(ctx, val) < 0) {
		return -EIO;
	}

	data->accel_fs = fs;

	return 0;
}

/**
 * ism330bx_accel_set_odr_raw - set new accelerometer sampling frequency
 * @dev: Pointer to instance of struct device (I2C or SPI)
 * @odr: Output data rate
 */
static int ism330bx_accel_set_odr_raw(const struct device *dev, uint8_t odr)
{
	const struct ism330bx_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	struct ism330bx_data *data = dev->data;
	if (ism330bx_xl_data_rate_set(ctx, odr) < 0) {
		return -EIO;
	}

	data->accel_freq = odr;

	return 0;
}

static int ism330bx_gyro_set_fs_raw(const struct device *dev, uint8_t fs)
{
	const struct ism330bx_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	LOG_DBG("gyro range is %d", fs);
	if (ism330bx_gy_full_scale_set(ctx, fs) < 0) {
		return -EIO;
	}

	return 0;
}

/**
 * ism330bx_gyro_set_odr_raw - set new gyroscope sampling frequency
 * @dev: Pointer to instance of struct device (I2C or SPI)
 * @odr: Output data rate
 */
static int ism330bx_gyro_set_odr_raw(const struct device *dev, uint8_t odr)
{
	const struct ism330bx_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	if (ism330bx_gy_data_rate_set(ctx, odr) < 0) {
		return -EIO;
	}

	return 0;
}

static int ism330bx_accel_set_mode(const struct device *dev, uint8_t mode)
{

	const struct ism330bx_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	ism330bx_xl_mode_t mode_raw;
	switch (mode) {
	case 0: /* High Performance */
		mode_raw = ISM330BX_XL_HIGH_PERFORMANCE_MD;
		break;
	case 2: /* High Accuracy */
		mode_raw = ISM330BX_XL_HIGH_PERFORMANCE_TDM_MD;
		break;
	case 4: /* Low Power 2 */
		mode_raw = ISM330BX_XL_LOW_POWER_2_AVG_MD;
		break;
	case 5: /* Low Power 4 */
		mode_raw = ISM330BX_XL_LOW_POWER_4_AVG_MD;
		break;
	case 6: /* Low Power 8 */
		mode_raw = ISM330BX_XL_LOW_POWER_8_AVG_MD;
		break;
	default:
		return -EIO;
	}
#if defined(CONFIG_ISM330BX_LOG_LEVEL_DBG)
	ism330bx_helper_print_accel_mode(&mode_raw);
#endif /* CONFIG_ISM330BX_LOG_LEVEL_DBG */
	return ism330bx_xl_mode_set(ctx, mode_raw);
}
static int ism330bx_accel_set_odr(const struct device *dev, uint16_t freq)
{
	int odr;

	odr = ism330bx_freq_to_odr_val(dev, SENSOR_CHAN_ACCEL_XYZ, freq);
	if (odr < 0) {
		return odr;
	}

	if (ism330bx_accel_set_odr_raw(dev, odr) < 0) {
		LOG_ERR("failed to set accelerometer sampling rate");
		return -EIO;
	}

	return 0;
}

static int ism330bx_accel_range_set(const struct device *dev, int32_t range)
{
	int fs;
	struct ism330bx_data *data = dev->data;

	fs = ism330bx_accel_range_to_fs_val(range);
	if (fs < 0) {
		return fs;
	}

	if (ism330bx_accel_set_fs_raw(dev, fs) < 0) {
		LOG_ERR("failed to set accelerometer full-scale");
		return -EIO;
	}

	data->acc_gain = ism330bx_accel_fs_map[fs] * GAIN_UNIT_XL / 2;
	return 0;
}

static int ism330bx_accel_config(const struct device *dev, enum sensor_channel chan,
				 enum sensor_attribute attr, const struct sensor_value *val)
{

	switch (attr) {
	case SENSOR_ATTR_FULL_SCALE:
		return ism330bx_accel_range_set(
			dev, sensor_ms2_to_g(val)); // TODO: Check if this is correct. Unused
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		return ism330bx_accel_set_odr(dev, val->val1);
	case ISM330BX_ATTR_SAMPLING_MODE:
		return ism330bx_accel_set_mode(dev, val->val1);
	default:
		LOG_ERR("Accel attribute %d not supported.", attr);
		return -ENOTSUP;
	}

	return 0;
}

static int ism330bx_gyro_odr_set(const struct device *dev, uint16_t freq)
{
	int odr;

	if (freq != 0 && freq < 7) {
		LOG_WRN("ism330bx gyroscope does not support less than 7.5Hz");
		return -EIO;
	}

	odr = ism330bx_freq_to_odr_val(dev, SENSOR_CHAN_GYRO_XYZ, freq);
	if (odr < 0) {
		return odr;
	}

	if (ism330bx_gyro_set_odr_raw(dev, odr) < 0) {
		LOG_ERR("failed to set gyroscope sampling rate");
		return -EIO;
	}

	return 0;
}

static int ism330bx_gyro_range_set(const struct device *dev, int32_t range)
{
	int fs;
	struct ism330bx_data *data = dev->data;

	fs = ism330bx_gyro_range_to_fs_val(range);
	if (fs < 0) {
		return fs;
	}

	if (ism330bx_gyro_set_fs_raw(dev, fs) < 0) {
		LOG_ERR("failed to set gyroscope full-scale");
		return -EIO;
	}

	data->gyro_gain = (ism330bx_gyro_fs_sens[fs] * GAIN_UNIT_G);
	return 0;
}

static int ism330bx_gyro_set_mode(const struct device *dev, uint8_t mode)
{

	const struct ism330bx_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	ism330bx_gy_mode_t mode_raw;
	switch (mode) {
	case 0: /* High Performance */
		mode_raw = ISM330BX_GY_HIGH_PERFORMANCE_MD;
		break;
		;
	case 4: /* Sleep */
		mode_raw = ISM330BX_GY_SLEEP_MD;
		break;
	case 5: /* Low Power */
		mode_raw = ISM330BX_GY_LOW_POWER_MD;
		break;
	default:
		return -EIO;
	}
#if defined(CONFIG_ISM330BX_LOG_LEVEL_DBG)
	ism330bx_helper_print_gyro_mode(&mode_raw);
#endif
	return ism330bx_gy_mode_set(ctx, mode_raw);
}

static int ism330bx_gyro_config(const struct device *dev, enum sensor_channel chan,
				enum sensor_attribute attr, const struct sensor_value *val)
{
	switch (attr) {
	case SENSOR_ATTR_FULL_SCALE:
		return ism330bx_gyro_range_set(dev, sensor_rad_to_degrees(val));
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		return ism330bx_gyro_odr_set(dev, (uint16_t)val->val1);
	case ISM330BX_ATTR_SAMPLING_MODE:
		return ism330bx_gyro_set_mode(dev, (uint8_t)val->val1);
	default:
		LOG_ERR("Gyro attribute not supported.");
		return -ENOTSUP;
	}

	return 0;
}

#ifdef CONFIG_ISM330BX_FREEFALL
static int ism330bx_attr_set_ff_dur(const struct device *dev, enum sensor_channel chan,
				    enum sensor_attribute attr, const struct sensor_value *val)
{
	int rc;
	uint16_t duration;
	const struct ism330bx_config *cfg = dev->config;
	struct ism330bx_data *ism330bx = dev->data;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;

	LOG_DBG("%s on channel %d", __func__, chan);

	/* can only be set for all directions at once */
	if (chan != SENSOR_CHAN_ACCEL_XYZ) {
		return -EINVAL;
	}

	/**
	 * The given duration in milliseconds with the val
	 * parameter is converted into register specific value.
	 */
	duration = (ism330bx->odr * (uint16_t)sensor_value_to_double(val)) / 1000;

	LOG_DBG("Freefall: duration is %d ms", (uint16_t)sensor_value_to_double(val));
	rc = ism330bx_ff_dur_set(ctx, duration);
	if (rc != 0) {
		LOG_ERR("Failed to set freefall duration");
		return -EIO;
	}
	return rc;
}
#endif /* CONFIG_ISM330BX_FREEFALL */

#ifdef CONFIG_ISM330BX_6D_ORIENTATION_DETECTION
/**
 * @brief Set 6D orientation detection threshold
 * @param dev Pointer to the device structure
 *
 * See section 5.4 6D orientation detection in
 * https://www.st.com/resource/en/application_note/an6109-ism330bx-6axis-imu-with-wide-bandwidth-lownoise-accelerometer-embedded-sensor-fusion-and-ai-for-industrial-applications-stmicroelectronics.pdf
 *
 */
static int ism330bx_attr_set_6d_orientation_detection_with_lowpass_filter_50_deg_threshold(
	const struct device *dev)
{
	int rc;
	const struct ism330bx_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;

	LOG_DBG("setting 6d_orientation_detection [using LOWPASS filter, 50 degrees threshold]");

	// 1. Write 41h to TAP_CFG0 // Enable LPF2 filter for 6D functionality and latched mode
	rc = ism330bx_filt_sixd_feed_set(ctx, ISM330BX_SIXD_FEED_LOW_PASS);
	__ASSERT(rc == 0, "[step 1] Failed to set SIXD_LOW_PASS ON");

	// Check if the filter is set correctly
	ism330bx_filt_sixd_feed_t sixd_feed;
	rc = ism330bx_filt_sixd_feed_get(ctx, &sixd_feed);
	__ASSERT(rc == 0 && sixd_feed == ISM330BX_SIXD_FEED_LOW_PASS,
		 "[step 1] Failed checking SIXD_LOW_PASS ON");

	rc = ism330bx_6d_threshold_set(ctx, ISM330BX_DEG_50);
	__ASSERT(rc == 0, "[step 2] Failed to set TAP_THS_6D ON");

	// Check if the threshold is set correctly
	ism330bx_6d_threshold_t sixd_threshold;
	rc = ism330bx_6d_threshold_get(ctx, &sixd_threshold);
	__ASSERT(rc == 0 && sixd_threshold == ISM330BX_DEG_50,
		 "[step 2] Failed to set TAP_THS_6D ON ");

	return rc;
}

#endif /* CONFIG_ISM330BX_6D_ORIENTATION_DETECTION */

static int ism330bx_attr_set(const struct device *dev, enum sensor_channel chan,
			     enum sensor_attribute attr, const struct sensor_value *val)
{
	enum pm_device_state power_state;
	pm_device_state_get(dev, &power_state);
	if (power_state != PM_DEVICE_STATE_ACTIVE) {
		LOG_DBG("%s: NOTE: power state is not active [%d]. setting attribute is useless.",
			__func__, power_state);
		if (ism330bx_pm_action(dev, PM_DEVICE_ACTION_TURN_ON) < 0) {
			LOG_ERR("failed to ism330bx_turn_on. %s:%d", __FILE__, __LINE__);
			return -ENOEXEC;
		}
	}
#if defined(CONFIG_ISM330BX_SENSORHUB)
	struct ism330bx_data *data = dev->data;
#endif /* CONFIG_ISM330BX_SENSORHUB */

	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		return ism330bx_accel_config(dev, chan, attr, val);
	case SENSOR_CHAN_GYRO_XYZ:
		return ism330bx_gyro_config(dev, chan, attr, val);

	case ISM330BX_CHAN_INTERRUPT_GENERATION:
		switch (attr) {
		case ORIENTATION_DETECTION_LOWPASS_FILTER_50_DEG_THRESH:
#if defined(CONFIG_ISM330BX_6D_ORIENTATION_DETECTION)
			return ism330bx_attr_set_6d_orientation_detection_with_lowpass_filter_50_deg_threshold(
				dev);
			break;
#else
			LOG_ERR("CONFIG_ISM330BX_6D_ORIENTATION_DETECTION not configured.");
			return -ENOTSUP;
#endif /* CONFIG_ISM330BX_6D_ORIENTATION_DETECTION */
		default:
			LOG_ERR("Section Position delta xyz doesn't support attribute %d.", attr);
			return -ENOTSUP;
		}
		break;
#if defined(CONFIG_ISM330BX_SENSORHUB)
	case SENSOR_CHAN_MAGN_XYZ:
	case SENSOR_CHAN_PRESS:
	case SENSOR_CHAN_HUMIDITY:
		if (!data->shub_inited) {
			LOG_ERR("shub not inited.");
			return -ENOTSUP;
		}

		return ism330bx_shub_config(dev, chan, attr, val);
#endif /* CONFIG_ISM330BX_SENSORHUB */
	default:
		LOG_WRN("attr_set() not supported on channel %d.", chan);
		return -ENOTSUP;
	}

	return 0;
}

static int ism330bx_attr_get(const struct device *dev, enum sensor_channel chan,
			     enum sensor_attribute attr, struct sensor_value *val)
{
	const struct ism330bx_data *data = dev->data;
	const struct ism330bx_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	int ret = -ENOTSUP;

	enum pm_device_state power_state;
	pm_device_state_get(dev, &power_state);
	if (power_state != PM_DEVICE_STATE_ACTIVE) {
		LOG_WRN("NOTE: power state is not active [%d]. getting attribute is useless.",
			power_state);
		return -ENOEXEC;
	}

	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		ism330bx_xl_mode_t mode;
		switch (attr) {
		case SENSOR_ATTR_SAMPLING_FREQUENCY:
			ism330bx_xl_data_rate_t data_rate;

			ret = ism330bx_xl_data_rate_get(ctx, &data_rate);
			if (ret != 0) {
				LOG_ERR("Failed to get accelerometer data rate");
				return -EIO;
			}

			ret = ism330bx_xl_mode_get(ctx, &mode);
			if (ret != 0) {
				LOG_ERR("Failed to get accelerometer data rate");
				return -EIO;
			}

			int mode_idx = (mode >> 4) & 0xf;
			sensor_value_from_float(val, ism330bx_odr_map[mode_idx][data_rate]);
			return ret;
		case ISM330BX_ATTR_SAMPLING_MODE:
			ret = ism330bx_xl_mode_get(ctx, &mode);
			if (ret != 0) {
				LOG_ERR("Failed to get accelerometer data rate");
				return -EIO;
			}
#if defined(CONFIG_ISM330BX_LOG_LEVEL_DBG)
			ism330bx_helper_print_accel_mode(&mode);
#endif /* CONFIG_ISM330BX_LOG_LEVEL_DBG */
			val->val1 = mode;
			return ret;
		default:
			LOG_ERR("Accel attribute %d not supported.", attr);
		}

		break;
	case SENSOR_CHAN_GYRO_XYZ:
		switch (attr) {
		case SENSOR_ATTR_SAMPLING_FREQUENCY:
			ism330bx_gy_data_rate_t data_rate;
			ret = ism330bx_gy_data_rate_get(ctx, &data_rate);
			if (ret != 0) {
				LOG_ERR("Failed to get accelerometer data rate");
				return -EIO;
			}
#if defined(CONFIG_ISM330BX_LOG_LEVEL_DBG)
			ism330bx_helper_print_gyro_odr(&data_rate);
#endif /* CONFIG_ISM330BX_LOG_LEVEL_DBG */
			ism330bx_gy_mode_t mode;
			ret = ism330bx_gy_mode_get(ctx, &mode);
			if (ret != 0) {
				LOG_ERR("Failed to get accelerometer data rate");
				return -EIO;
			}

#if defined(CONFIG_ISM330BX_LOG_LEVEL_DBG)
			ism330bx_helper_print_gyro_mode(&mode);
#endif /* CONFIG_ISM330BX_LOG_LEVEL_DBG */
			int mode_idx = (mode >> 4) & 0xf;
			sensor_value_from_float(val, ism330bx_odr_map[mode_idx][data_rate]);
			return ret;
		case ISM330BX_ATTR_SAMPLING_MODE:
			ret = ism330bx_gy_mode_get(ctx, &mode);
			if (ret != 0) {
				LOG_ERR("Failed to get accelerometer data rate");
				return -EIO;
			}
#if defined(CONFIG_ISM330BX_LOG_LEVEL_DBG)
			ism330bx_helper_print_gyro_mode(&mode);
#endif /* CONFIG_ISM330BX_LOG_LEVEL_DBG */
			val->val1 = mode;
			return ret;
		default:
			LOG_ERR("Gyro attribute %d not supported.", attr);
		}
		break;
	case ISM330BX_CHAN_INTERRUPT_GENERATION:
		switch (attr) {
		case ORIENTATION_DETECTION_EVENT:
#if defined(CONFIG_ISM330BX_LOG_LEVEL_DBG)
			ism330bx_helper_print_orientation_interrupt_flags(&data->all_sources);
#endif /* CONFIG_ISM330BX_LOG_LEVEL_DBG */
			ism330bx_d6d_src_t *d6d_src = (ism330bx_d6d_src_t *)val;
			d6d_src->zl = data->all_sources.six_d_zl;
			d6d_src->zh = data->all_sources.six_d_zh;
			d6d_src->yl = data->all_sources.six_d_yl;
			d6d_src->yh = data->all_sources.six_d_yh;
			d6d_src->xl = data->all_sources.six_d_xl;
			d6d_src->xh = data->all_sources.six_d_xh;
			break;
		default:
			break;
		}

	default:
		break;
	}
	return ret;
}
static int ism330bx_sample_fetch_accel(const struct device *dev)
{
	const struct ism330bx_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	struct ism330bx_data *data = dev->data;

	if (ism330bx_acceleration_raw_get(ctx, data->acc) < 0) {
		LOG_ERR("Failed to read sample");
		return -EIO;
	}

	return 0;
}

static int ism330bx_sample_fetch_gyro(const struct device *dev)
{
	const struct ism330bx_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	struct ism330bx_data *data = dev->data;

	if (ism330bx_angular_rate_raw_get(ctx, data->gyro) < 0) {
		LOG_ERR("Failed to read sample");
		return -EIO;
	}

	return 0;
}

#if defined(CONFIG_ISM330BX_ENABLE_TEMP)
static int ism330bx_sample_fetch_temp(const struct device *dev)
{
	const struct ism330bx_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	struct ism330bx_data *data = dev->data;

	if (ism330bx_temperature_raw_get(ctx, &data->temp_sample) < 0) {
		printf("Failed to read sample");
		return -EIO;
	}

	return 0;
}
#endif

#if defined(CONFIG_ISM330BX_SENSORHUB)
static int ism330bx_sample_fetch_shub(const struct device *dev)
{
	if (ism330bx_shub_fetch_external_devs(dev) < 0) {
		printf("failed to read ext shub devices");
		return -EIO;
	}

	return 0;
}
#endif /* CONFIG_ISM330BX_SENSORHUB */

static int ism330bx_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
#if defined(CONFIG_ISM330BX_SENSORHUB)
	struct ism330bx_data *data = dev->data;
#endif /* CONFIG_ISM330BX_SENSORHUB */

	enum pm_device_state power_state;
	pm_device_state_get(dev, &power_state);
	LOG_DBG("Before: power_state: %d", power_state);
	if (power_state != PM_DEVICE_STATE_ACTIVE &&
	    ism330bx_pm_action(dev, PM_DEVICE_ACTION_TURN_ON) < 0) {
		LOG_ERR("failed to ism330bx_turn_on. %s:%d", __FILE__, __LINE__);
		return -ENOEXEC;
	}
	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		ism330bx_sample_fetch_accel(dev);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		ism330bx_sample_fetch_gyro(dev);
		break;
#if defined(CONFIG_ISM330BX_ENABLE_TEMP)
	case SENSOR_CHAN_DIE_TEMP:
		ism330bx_sample_fetch_temp(dev);
		break;
#endif
	case SENSOR_CHAN_ALL:
		ism330bx_sample_fetch_accel(dev);
		ism330bx_sample_fetch_gyro(dev);
#if defined(CONFIG_ISM330BX_ENABLE_TEMP)
		ism330bx_sample_fetch_temp(dev);
#endif
#if defined(CONFIG_ISM330BX_SENSORHUB)
		if (data->shub_inited) {
			ism330bx_sample_fetch_shub(dev);
		}
#endif
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static inline void ism330bx_accel_convert(struct sensor_value *val, int raw_val,
					  uint32_t sensitivity)
{
	int64_t dval;

	/* Sensitivity is exposed in ug/LSB */
	/* Convert to m/s^2 */
	dval = (int64_t)(raw_val)*sensitivity;
	sensor_ug_to_ms2(dval, val);
}

static inline int ism330bx_accel_get_channel(enum sensor_channel chan, struct sensor_value *val,
					     struct ism330bx_data *data, uint32_t sensitivity)
{
	uint8_t i;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		ism330bx_accel_convert(val, data->acc[0], sensitivity);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		ism330bx_accel_convert(val, data->acc[1], sensitivity);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		ism330bx_accel_convert(val, data->acc[2], sensitivity);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		for (i = 0; i < 3; i++) {
			ism330bx_accel_convert(val++, data->acc[i], sensitivity);
		}
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int ism330bx_accel_channel_get(enum sensor_channel chan, struct sensor_value *val,
				      struct ism330bx_data *data)
{
	return ism330bx_accel_get_channel(chan, val, data, data->acc_gain);
}

static inline void ism330bx_gyro_convert(struct sensor_value *val, int raw_val,
					 uint32_t sensitivity)
{
	int64_t dval;

	/* Sensitivity is exposed in udps/LSB */
	/* So, calculate value in 10 udps unit and then to rad/s */
	dval = (int64_t)(raw_val)*sensitivity / 10;
	sensor_10udegrees_to_rad(dval, val);
}

static inline int ism330bx_gyro_get_channel(enum sensor_channel chan, struct sensor_value *val,
					    struct ism330bx_data *data, uint32_t sensitivity)
{
	uint8_t i;

	switch (chan) {
	case SENSOR_CHAN_GYRO_X:
		ism330bx_gyro_convert(val, data->gyro[0], sensitivity);
		break;
	case SENSOR_CHAN_GYRO_Y:
		ism330bx_gyro_convert(val, data->gyro[1], sensitivity);
		break;
	case SENSOR_CHAN_GYRO_Z:
		ism330bx_gyro_convert(val, data->gyro[2], sensitivity);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		for (i = 0; i < 3; i++) {
			ism330bx_gyro_convert(val++, data->gyro[i], sensitivity);
		}
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int ism330bx_gyro_channel_get(enum sensor_channel chan, struct sensor_value *val,
				     struct ism330bx_data *data)
{
	return ism330bx_gyro_get_channel(chan, val, data, data->gyro_gain);
}

#if defined(CONFIG_ISM330BX_ENABLE_TEMP)
static void ism330bx_gyro_channel_get_temp(struct sensor_value *val, struct ism330bx_data *data)
{
	int32_t micro_c;

	/* convert units to micro Celsius. Raw temperature samples are
	 * expressed in 256 LSB/deg_C units. And LSB output is 0 at 25 C.
	 */
	micro_c = (data->temp_sample * 1000000) / 256;

	val->val1 = micro_c / 1000000 + 25;
	val->val2 = micro_c % 1000000;
}
#endif

#if defined(CONFIG_ISM330BX_SENSORHUB)
static inline void ism330bx_magn_convert(struct sensor_value *val, int raw_val,
					 uint16_t sensitivity)
{
	double dval;

	/* Sensitivity is exposed in ugauss/LSB */
	dval = (double)(raw_val * sensitivity);
	val->val1 = (int32_t)dval / 1000000;
	val->val2 = (int32_t)dval % 1000000;
}

static inline int ism330bx_magn_get_channel(enum sensor_channel chan, struct sensor_value *val,
					    struct ism330bx_data *data)
{
	int16_t sample[3];
	int idx;

	idx = ism330bx_shub_get_idx(data->dev, SENSOR_CHAN_MAGN_XYZ);
	if (idx < 0) {
		printf("external magn not supported");
		return -ENOTSUP;
	}

	sample[0] = (int16_t)(data->ext_data[idx][0] | (data->ext_data[idx][1] << 8));
	sample[1] = (int16_t)(data->ext_data[idx][2] | (data->ext_data[idx][3] << 8));
	sample[2] = (int16_t)(data->ext_data[idx][4] | (data->ext_data[idx][5] << 8));

	switch (chan) {
	case SENSOR_CHAN_MAGN_X:
		ism330bx_magn_convert(val, sample[0], data->magn_gain);
		break;
	case SENSOR_CHAN_MAGN_Y:
		ism330bx_magn_convert(val, sample[1], data->magn_gain);
		break;
	case SENSOR_CHAN_MAGN_Z:
		ism330bx_magn_convert(val, sample[2], data->magn_gain);
		break;
	case SENSOR_CHAN_MAGN_XYZ:
		ism330bx_magn_convert(val, sample[0], data->magn_gain);
		ism330bx_magn_convert(val + 1, sample[1], data->magn_gain);
		ism330bx_magn_convert(val + 2, sample[2], data->magn_gain);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static inline void ism330bx_hum_convert(struct sensor_value *val, struct ism330bx_data *data)
{
	float rh;
	int16_t raw_val;
	struct hts221_data *ht = &data->hts221;
	int idx;

	idx = ism330bx_shub_get_idx(data->dev, SENSOR_CHAN_HUMIDITY);
	if (idx < 0) {
		printf("external press/temp not supported");
		return;
	}

	raw_val = (int16_t)(data->ext_data[idx][0] | (data->ext_data[idx][1] << 8));

	/* find relative humidty by linear interpolation */
	rh = (ht->y1 - ht->y0) * raw_val + ht->x1 * ht->y0 - ht->x0 * ht->y1;
	rh /= (ht->x1 - ht->x0);

	/* convert humidity to integer and fractional part */
	val->val1 = rh;
	val->val2 = rh * 1000000;
}

static inline void ism330bx_press_convert(struct sensor_value *val, struct ism330bx_data *data)
{
	int32_t raw_val;
	int idx;

	idx = ism330bx_shub_get_idx(data->dev, SENSOR_CHAN_PRESS);
	if (idx < 0) {
		printf("external press/temp not supported");
		return;
	}

	raw_val = (int32_t)(data->ext_data[idx][0] | (data->ext_data[idx][1] << 8) |
			    (data->ext_data[idx][2] << 16));

	/* Pressure sensitivity is 4096 LSB/hPa */
	/* Convert raw_val to val in kPa */
	val->val1 = (raw_val >> 12) / 10;
	val->val2 =
		(raw_val >> 12) % 10 * 100000 + (((int32_t)((raw_val) & 0x0FFF) * 100000L) >> 12);
}

static inline void ism330bx_temp_convert(struct sensor_value *val, struct ism330bx_data *data)
{
	int16_t raw_val;
	int idx;

	idx = ism330bx_shub_get_idx(data->dev, SENSOR_CHAN_PRESS);
	if (idx < 0) {
		printf("external press/temp not supported");
		return;
	}

	raw_val = (int16_t)(data->ext_data[idx][3] | (data->ext_data[idx][4] << 8));

	/* Temperature sensitivity is 100 LSB/deg C */
	val->val1 = raw_val / 100;
	val->val2 = (int32_t)raw_val % 100 * (10000);
}
#endif

static int ism330bx_channel_get(const struct device *dev, enum sensor_channel chan,
				struct sensor_value *val)
{

	struct ism330bx_data *data = dev->data;
	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		ism330bx_accel_channel_get(chan, val, data);
		break;
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_GYRO_XYZ:
		ism330bx_gyro_channel_get(chan, val, data);
		break;
#if defined(CONFIG_ISM330BX_ENABLE_TEMP)
	case SENSOR_CHAN_DIE_TEMP:
		ism330bx_gyro_channel_get_temp(val, data);
		break;
#endif
#if defined(CONFIG_ISM330BX_SENSORHUB)
	case SENSOR_CHAN_MAGN_X:
	case SENSOR_CHAN_MAGN_Y:
	case SENSOR_CHAN_MAGN_Z:
	case SENSOR_CHAN_MAGN_XYZ:
		if (!data->shub_inited) {
			LOG_ERR("attr_set() shub not inited.");
			return -ENOTSUP;
		}

		ism330bx_magn_get_channel(chan, val, data);
		break;

	case SENSOR_CHAN_HUMIDITY:
		if (!data->shub_inited) {
			LOG_ERR("attr_set() shub not inited.");
			return -ENOTSUP;
		}

		ism330bx_hum_convert(val, data);
		break;

	case SENSOR_CHAN_PRESS:
		if (!data->shub_inited) {
			LOG_ERR("attr_set() shub not inited.");
			return -ENOTSUP;
		}

		ism330bx_press_convert(val, data);
		break;

	case SENSOR_CHAN_AMBIENT_TEMP:
		if (!data->shub_inited) {
			LOG_ERR("attr_set() shub not inited.");
			return -ENOTSUP;
		}

		ism330bx_temp_convert(val, data);
		break;
#endif
	default:
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api ism330bx_driver_api = {
	.attr_set = ism330bx_attr_set,
	.attr_get = ism330bx_attr_get,
#if CONFIG_ISM330BX_TRIGGER
	.trigger_set = ism330bx_trigger_set,
#endif
	.sample_fetch = ism330bx_sample_fetch,
	.channel_get = ism330bx_channel_get,
};

static int ism330bx_init_chip(const struct device *dev)
{
	const struct ism330bx_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	struct ism330bx_data *ism330bx = dev->data;
	uint8_t chip_id;
	uint8_t odr, fs;

	/* All registers except 0x01 are different between banks, including the WHO_AM_I
	 * register and the register used for a SW reset.  If the ism330bx wasn't on the user
	 * bank when it reset, then both the chip id check and the sw reset will fail unless we
	 * set the bank now.
	 */
	if (ism330bx_mem_bank_set(ctx, ISM330BX_MAIN_MEM_BANK) < 0) {
		LOG_ERR("Failed to set user bank");
		return -EIO;
	}

	if (ism330bx_device_id_get(ctx, &chip_id) < 0) {
		LOG_ERR("Failed reading chip id");
		return -EIO;
	}
	LOG_INF("chip id 0x%x", chip_id);

	if (chip_id != ISM330BX_ID) {
		LOG_ERR("Invalid chip id 0x%x", chip_id);
		return -EIO;
	}

	/* reset device (sw_por) */
	if (ism330bx_reset_set(ctx, ISM330BX_GLOBAL_RST) < 0) {
		LOG_ERR("Failed to reset device");
		return -EIO;
	}

	/* Since setting sw_por to 1 -wait 30ms as reported in AN6109 */
	k_sleep(K_MSEC(30));

	fs = cfg->accel_range;
#if defined(CONFIG_ISM330BX_LOG_LEVEL_DBG)
	ism330bx_helper_print_accel_full_scale((ism330bx_xl_full_scale_t *)&fs);
#endif /* CONFIG_ISM330BX_LOG_LEVEL_DBG */
	if (ism330bx_accel_set_fs_raw(dev, fs) < 0) {
		LOG_ERR("failed to set accelerometer range %d", fs);
		return -EIO;
	}
	ism330bx->acc_gain = ism330bx_accel_fs_map[fs] * GAIN_UNIT_XL / 2;

	odr = cfg->accel_odr;
#if defined(CONFIG_ISM330BX_LOG_LEVEL_DBG)
	LOG_DBG("accel odr from config:");
	ism330bx_helper_print_accel_odr((ism330bx_xl_data_rate_t *)&odr);
#endif /* CONFIG_ISM330BX_LOG_LEVEL_DBG */
	if (ism330bx_accel_set_odr_raw(dev, odr) < 0) {
		LOG_ERR("failed to set accelerometer odr %d", odr);
		return -EIO;
	}
	ism330bx->acc_odr = cfg->accel_odr;

	fs = cfg->gyro_range;
#if defined(CONFIG_ISM330BX_LOG_LEVEL_DBG)
	ism330bx_helper_print_gyro_full_scale((ism330bx_gy_full_scale_t *)&fs);
#endif /* CONFIG_ISM330BX_LOG_LEVEL_DBG */
	if (ism330bx_gyro_set_fs_raw(dev, fs) < 0) {
		LOG_ERR("failed to set gyroscope range %d", fs);
		return -EIO;
	}
	ism330bx->gyro_gain = (ism330bx_gyro_fs_sens[fs] * GAIN_UNIT_G);

	odr = cfg->gyro_odr;
#if defined(CONFIG_ISM330BX_LOG_LEVEL_DBG)
	LOG_DBG("gyro odr from config:");
	ism330bx_helper_print_gyro_odr((ism330bx_gy_data_rate_t *)&odr);
#endif /* CONFIG_ISM330BX_LOG_LEVEL_DBG */
	ism330bx->gyro_freq = odr;
	if (ism330bx_gyro_set_odr_raw(dev, odr) < 0) {
		LOG_ERR("failed to set gyroscope odr %d", odr);
		return -EIO;
	}
	ism330bx->gyro_odr = cfg->gyro_odr;

	if (ism330bx_block_data_update_set(ctx, 1) < 0) {
		LOG_ERR("failed to set BDU mode");
		return -EIO;
	}
	return 0;
}

static int ism330bx_init(const struct device *dev)
{
	int ret = -ENOEXEC;
	const struct ism330bx_config *cfg = dev->config;
	struct ism330bx_data *data = dev->data;
	LOG_INF("Initialize device %s on SPI bus %s", dev->name, cfg->stmemsc_cfg.spi.bus->name);

	data->dev = dev;
	ret = ism330bx_pm_action(dev, PM_DEVICE_ACTION_TURN_ON);
	if (ret < 0) {
		LOG_ERR("failed to ism330bx_turn_on");
		return -ENOEXEC;
	}

	/*
	After the device is powered up, it performs a 10 ms (maximum) boot procedure to load the
	trimming parameters. After the boot is completed, both the accelerometer and the gyroscope
	are automatically configured in powerdown mode. During the boot time, the registers are not
	accessible.
	*/
	k_sleep(K_MSEC(10));

	if (ism330bx_init_chip(dev) < 0) {
		LOG_ERR("failed to initialize chip");
		return -EIO;
	}

#ifdef CONFIG_ISM330BX_TRIGGER
	if (cfg->trig_enabled) {
		LOG_INF("initiating interrupt pins");
		if (ism330bx_init_interrupt(dev) < 0) {
			LOG_ERR("Failed to initialize interrupt.");
			return -EIO;
		}
	}
#endif

#ifdef CONFIG_ISM330BX_SENSORHUB
	data->shub_inited = true;
	if (ism330bx_shub_init(dev) < 0) {
		LOG_INF("shub: no external chips found");
		data->shub_inited = false;
	}
#endif

#ifndef CONFIG_ISM330BX_TRIGGER
	// /* If interrupts are not enabled, turn off the device to save power */
	// if (ism330bx_pm_action(dev, PM_DEVICE_ACTION_TURN_OFF) < 0)
	// {
	//     LOG_ERR("failed to ism330bx_turn_off");
	//     return -ENOEXEC;
	// }
	// ret = pm_device_runtime_enable(dev);
	// if (ret < 0)
	// {
	//     LOG_ERR("failed to pm_device_runtime_enable");
	//     return -ENOEXEC;
	// }
#endif // CONFIG_ISM330BX_TRIGGER

	return 0;
}

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "ISM330BX driver enabled without any devices"
#endif

/*
 * Device creation macro, shared by ISM330BX_DEFINE_SPI() and
 * ISM330BX_DEFINE_I2C().
 */

#define ISM330BX_DEVICE_INIT(inst)                                                                 \
	PM_DEVICE_DT_INST_DEFINE(inst, ism330bx_pm_action);                                        \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, ism330bx_init, PM_DEVICE_DT_INST_GET(inst),             \
				     &ism330bx_data_##inst, &ism330bx_config_##inst, POST_KERNEL,  \
				     CONFIG_SENSOR_INIT_PRIORITY, &ism330bx_driver_api);

/*
 * Instantiation macros used when a device is on a SPI bus.
 */

#ifdef CONFIG_ISM330BX_TRIGGER
#define ISM330BX_CFG_IRQ(inst)                                                                     \
	.trig_enabled = true, .int1_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int1_gpios, {0}),        \
	.int2_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int2_gpios, {0}),                              \
	.drdy_pulsed = DT_INST_PROP(inst, drdy_pulsed), .drdy_pin = DT_INST_PROP(inst, drdy_pin)
#else
#define ISM330BX_CFG_IRQ(inst)
#endif /* CONFIG_ISM330BX_TRIGGER */

#define ISM330BX_SPI_OP                                                                            \
	(SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA)

#define ISM330BX_CONFIG_COMMON(inst)                                                               \
	.accel_odr = DT_INST_PROP(inst, accel_odr),                                                \
	.accel_range = DT_INST_PROP(inst, accel_range), .gyro_odr = DT_INST_PROP(inst, gyro_odr),  \
	.gyro_range = DT_INST_PROP(inst, gyro_range),                                              \
	IF_ENABLED(UTIL_OR(DT_INST_NODE_HAS_PROP(inst, int1_gpios),  \
                       DT_INST_NODE_HAS_PROP(inst, int2_gpios)), \
               (ISM330BX_CFG_IRQ(inst)))

#define ISM330BX_CONFIG_SPI(inst)                                                                  \
	{STMEMSC_CTX_SPI(&ism330bx_config_##inst.stmemsc_cfg),                                     \
	 .stmemsc_cfg =                                                                            \
		 {                                                                                 \
			 .spi = SPI_DT_SPEC_INST_GET(inst, ISM330BX_SPI_OP, 0),                    \
		 },                                                                                \
	 ISM330BX_CONFIG_COMMON(inst)}

/*
 * Instantiation macros used when a device is on an I2C bus.
 */

#define ISM330BX_CONFIG_I2C(inst)                                                                  \
	{STMEMSC_CTX_I2C(&ism330bx_config_##inst.stmemsc_cfg),                                     \
	 .stmemsc_cfg =                                                                            \
		 {                                                                                 \
			 .i2c = I2C_DT_SPEC_INST_GET(inst),                                        \
		 },                                                                                \
	 ISM330BX_CONFIG_COMMON(inst)}

/*
 * Main instantiation macro. Use of COND_CODE_1() selects the right
 * bus-specific macro at preprocessor time.
 */

#define ISM330BX_DEFINE(inst)                                                                      \
	static struct ism330bx_data ism330bx_data_##inst;                                          \
	static const struct ism330bx_config ism330bx_config_##inst = COND_CODE_1(DT_INST_ON_BUS(inst, spi),                                                                         \
                    (ISM330BX_CONFIG_SPI(inst)),                                                                       \
                    (ISM330BX_CONFIG_I2C(inst)));                \
	ISM330BX_DEVICE_INIT(inst)

DT_INST_FOREACH_STATUS_OKAY(ISM330BX_DEFINE)
