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

#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <string.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>

#include "ism330bx.h"

LOG_MODULE_REGISTER(ism330bx, CONFIG_ISM330BX_LOG_LEVEL);

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
// static int ism330bx_pm_action(const struct device *dev, enum pm_device_action action)
// {
// 	int rc = 0;

// 	switch (action) {
// 	case PM_DEVICE_ACTION_TURN_ON:
// 		printk("ISM330BX: PM_DEVICE_ACTION_TURN_ON");
// 		// TODO: do something
// 		break;

// 	case PM_DEVICE_ACTION_TURN_OFF:
// 		printk("ISM330BX: PM_DEVICE_ACTION_TURN_OFF");
// 		// TODO: do something
// 		break;

// 	case PM_DEVICE_ACTION_SUSPEND:
// 		printk("ISM330BX: PM_DEVICE_ACTION_SUSPEND");
// 		break;
// 	case PM_DEVICE_ACTION_RESUME:
// 		printk("ISM330BX: PM_DEVICE_ACTION_RESUME");
// 		break;

// 	default:
// 		LOG_ERR("Got unknown power management action");
// 		rc = -EINVAL;
// 		break;
// 	}
// 	return rc;
// }

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

static int ism330bx_freq_to_odr_val(const struct device *dev, uint16_t freq)
{
	const struct ism330bx_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	ism330bx_xl_data_rate_t odr;
	int8_t mode;
	size_t i;

	if (ism330bx_xl_data_rate_get(ctx, &odr) < 0) {
		return -EINVAL;
	}
	printk("Gal in ism330bx_freq_to_odr_val odr: %d\n", odr);
	mode = (odr >> 4) & 0xf;
	printk("Gal in ism330bx_freq_to_odr_val mode: %d\n", mode);

	for (i = 0; i < ARRAY_SIZE(ism330bx_odr_map[mode]); i++) {
		if (freq <= ism330bx_odr_map[mode][i]) {
			printk("mode: %d - odr: %d\n", mode, i);
			return i;
		}
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
	// printk("Gal in ism330bx_accel_set_fs_raw fs: %d\n", fs);
	if (ism330bx_xl_full_scale_set(ctx, val) < 0) {
		return -EIO;
	}

	data->accel_fs = fs;

	return 0;
}

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

	if (ism330bx_gy_full_scale_set(ctx, fs) < 0) {
		return -EIO;
	}

	return 0;
}

static int ism330bx_gyro_set_odr_raw(const struct device *dev, uint8_t odr)
{
	const struct ism330bx_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;

	if (ism330bx_gy_data_rate_set(ctx, odr) < 0) {
		return -EIO;
	}

	return 0;
}

static int ism330bx_accel_odr_set(const struct device *dev, uint16_t freq)
{
	int odr;

	odr = ism330bx_freq_to_odr_val(dev, freq);
	// printk("Gal in ism330bx_accel_odr_set odr: %d\n", odr);
	if (odr < 0) {
		return odr;
	}

	if (ism330bx_accel_set_odr_raw(dev, odr) < 0) {
		printk("failed to set accelerometer sampling rate\n");
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
		printk("failed to set accelerometer full-scale");
		return -EIO;
	}

	data->acc_gain = ism330bx_accel_fs_map[fs] * GAIN_UNIT_XL / 2;
	return 0;
}

static int ism330bx_accel_config(const struct device *dev, enum sensor_channel chan,
				 enum sensor_attribute attr, const struct sensor_value *val)
{
	const struct ism330bx_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	ism330bx_xl_mode_t mode;

	switch (attr) {
	case SENSOR_ATTR_FULL_SCALE:
		return ism330bx_accel_range_set(dev, sensor_ms2_to_g(val));
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		return ism330bx_accel_odr_set(dev, val->val1);
	case SENSOR_ATTR_CONFIGURATION:
		switch (val->val1) {
		/*
		    ISM330BX_XL_HIGH_PERFORMANCE_MD              = 0x0,
		    ISM330BX_XL_HIGH_PERFORMANCE_TDM_MD          = 0x2,
		    ISM330BX_XL_LOW_POWER_2_AVG_MD               = 0x4,
		    ISM330BX_XL_LOW_POWER_4_AVG_MD               = 0x5,
		    ISM330BX_XL_LOW_POWER_8_AVG_MD               = 0x6,
		*/
		case 0: /* High Performance */
			mode = ISM330BX_XL_HIGH_PERFORMANCE_MD;
			break;
		// case 1: /* High Accuracy */
		//     mode = ISM330BX_XL_HIGH_ACCURACY_ODR_MD;
		//     break;
		case 2: /* High Accuracy */
			mode = ISM330BX_XL_HIGH_PERFORMANCE_TDM_MD;
			break;
		// case 3: /* ODR triggered */
		//     mode = ISM330BX_XL_ODR_TRIGGERED_MD;
		//     break;
		case 4: /* Low Power 2 */
			mode = ISM330BX_XL_LOW_POWER_2_AVG_MD;
			break;
		case 5: /* Low Power 4 */
			mode = ISM330BX_XL_LOW_POWER_4_AVG_MD;
			break;
		case 6: /* Low Power 8 */
			mode = ISM330BX_XL_LOW_POWER_8_AVG_MD;
			break;
		// case 7: /* Normal */
		//     mode = ISM330BX_XL_NORMAL_MD;
		//     break;
		default:
			return -EIO;
		}

		return ism330bx_xl_mode_set(ctx, mode);
	default:
		printk("Accel attribute not supported.");
		return -ENOTSUP;
	}

	return 0;
}

static int ism330bx_gyro_odr_set(const struct device *dev, uint16_t freq)
{
	int odr;

	if (freq < 7) {
		LOG_WRN("ism330bx gyroscope does not support less than 7.5Hz");
		return -EIO;
	}

	odr = ism330bx_freq_to_odr_val(dev, freq);
	if (odr < 0) {
		return odr;
	}

	if (ism330bx_gyro_set_odr_raw(dev, odr) < 0) {
		printk("failed to set gyroscope sampling rate");
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
		printk("failed to set gyroscope full-scale");
		return -EIO;
	}

	data->gyro_gain = (ism330bx_gyro_fs_sens[fs] * GAIN_UNIT_G);
	return 0;
}

static int ism330bx_gyro_config(const struct device *dev, enum sensor_channel chan,
				enum sensor_attribute attr, const struct sensor_value *val)
{
	const struct ism330bx_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	ism330bx_gy_mode_t mode;

	switch (attr) {
	case SENSOR_ATTR_FULL_SCALE:
		return ism330bx_gyro_range_set(dev, sensor_rad_to_degrees(val));
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		return ism330bx_gyro_odr_set(dev, val->val1);
	case SENSOR_ATTR_CONFIGURATION:
		switch (val->val1) {
		/*
		    ISM330BX_GY_HIGH_PERFORMANCE_MD              = 0x0,
		    ISM330BX_GY_SLEEP_MD                         = 0x4,
		    ISM330BX_GY_LOW_POWER_MD                     = 0x5,
		*/
		case 0: /* High Performance */
			mode = ISM330BX_GY_HIGH_PERFORMANCE_MD;
			break;
		// case 1: /* High Accuracy */
		//     mode = ISM330BX_GY_HIGH_ACCURACY_ODR_MD;
		//     break;
		case 4: /* Sleep */
			mode = ISM330BX_GY_SLEEP_MD;
			break;
		case 5: /* Low Power */
			mode = ISM330BX_GY_LOW_POWER_MD;
			break;
		default:
			return -EIO;
		}

		return ism330bx_gy_mode_set(ctx, mode);
	default:
		printk("Gyro attribute not supported.");
		return -ENOTSUP;
	}

	return 0;
}

static int ism330bx_attr_set(const struct device *dev, enum sensor_channel chan,
			     enum sensor_attribute attr, const struct sensor_value *val)
{
#if defined(CONFIG_ISM330BX_SENSORHUB)
	struct ism330bx_data *data = dev->data;
#endif /* CONFIG_ISM330BX_SENSORHUB */

	printk("Gal is in attr_set: chan: %d, attr: %d", chan, attr);
	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		return ism330bx_accel_config(dev, chan, attr, val);
	case SENSOR_CHAN_GYRO_XYZ:
		return ism330bx_gyro_config(dev, chan, attr, val);
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
		LOG_WRN("attr_set() not supported on this channel.");
		return -ENOTSUP;
	}

	return 0;
}

static int ism330bx_attr_get(const struct device *dev, enum sensor_channel chan,
			     enum sensor_attribute attr, struct sensor_value *val)
{
	const struct ism330bx_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	int ret = -ENOTSUP;

	// enum pm_device_state power_state;
	// pm_device_state_get(dev, &power_state);
	// if (power_state != PM_DEVICE_STATE_ACTIVE) {
	// 	LOG_WRN("ism330bx not powered");
	// 	if (ism330bx_pm_action(dev, PM_DEVICE_ACTION_TURN_ON) < 0) {
	// 		AUGU_LOG_ERR("failed to augu_ism330bx_turn_on. %s:%d", __FILE__, __LINE__);
	// 		return -ENOEXEC;
	// 	}
	// }

	switch (attr) {
	case SENSOR_ATTR_FULL_SCALE:
		ism330bx_d6d_src_t d6d_src;
		ism330bx_all_sources_t all_sources;
		ret = ism330bx_all_sources_get(ctx, &all_sources);
		if (ret != 0) {
			return -EIO;
		}

		d6d_src.zl = all_sources.six_d_zl;
		d6d_src.zh = all_sources.six_d_zh;
		d6d_src.yl = all_sources.six_d_yl;
		d6d_src.yh = all_sources.six_d_yh;
		d6d_src.xl = all_sources.six_d_xl;
		d6d_src.xh = all_sources.six_d_xh;
		printk("d6d_src d6d_src.zl: %02x\n", d6d_src.zl);
		printk("d6d_src d6d_src.zh: %02x\n", d6d_src.zh);
		printk("d6d_src d6d_src.yl: %02x\n", d6d_src.yl);
		printk("d6d_src d6d_src.yh: %02x\n", d6d_src.yh);
		printk("d6d_src d6d_src.xl: %02x\n", d6d_src.xl);
		printk("d6d_src d6d_src.xh: %02x\n", d6d_src.xh);
		break;

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

	// printk("Gal in ism330bx_sample_fetch_accel\n");
	if (ism330bx_acceleration_raw_get(ctx, data->acc) < 0) {
		printk("Failed to read sample");
		return -EIO;
	}

	return 0;
}

static int ism330bx_sample_fetch_gyro(const struct device *dev)
{
	const struct ism330bx_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	struct ism330bx_data *data = dev->data;

	// printk("Gal in ism330bx_sample_fetch_gyro\n");
	if (ism330bx_angular_rate_raw_get(ctx, data->gyro) < 0) {
		printk("Failed to read sample");
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
		printk("Failed to read sample");
		return -EIO;
	}

	return 0;
}
#endif

#if defined(CONFIG_ISM330BX_SENSORHUB)
static int ism330bx_sample_fetch_shub(const struct device *dev)
{
	if (ism330bx_shub_fetch_external_devs(dev) < 0) {
		printk("failed to read ext shub devices");
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

	// enum pm_device_state power_state;
	// pm_device_state_get(dev, &power_state);
	// printk("Before: power_state: %d", power_state);
	// if (power_state != PM_DEVICE_STATE_ACTIVE) {
	// 	if (ism330bx_pm_action(dev, PM_DEVICE_ACTION_TURN_ON) < 0) {
	// 		AUGU_LOG_ERR("failed to augu_ism330bx_turn_on. %s:%d", __FILE__, __LINE__);
	// 		return -ENOEXEC;
	// 	}
	// }
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

	// pm_device_state_get(dev, &power_state);
	// printk("Before: power_state: %d", power_state);
	// if (ism330bx_pm_action(dev, PM_DEVICE_ACTION_TURN_OFF) < 0)
	// {
	//     AUGU_LOG_ERR("failed to ism330bx_pm_action ACTION_TURN_OFF. %s:%d", __FILE__,
	//     __LINE__); return -ENOEXEC;
	// }
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
		printk("external magn not supported");
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
		printk("external press/temp not supported");
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
		printk("external press/temp not supported");
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
		printk("external press/temp not supported");
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

	// printk("Gal is calling ism330bx_channel_get: %d", chan);
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
		printk("Failed to set user bank");
		return -EIO;
	}

	if (ism330bx_device_id_get(ctx, &chip_id) < 0) {
		printk("Failed reading chip id");
		return -EIO;
	}
	printk("chip id 0x%x\n", chip_id);

	if (chip_id != ISM330BX_ID) {
		LOG_ERR("Invalid chip id 0x%x\n", chip_id);
		return -EIO;
	}

	/* reset device (sw_por) */
	if (ism330bx_reset_set(ctx, ISM330BX_GLOBAL_RST) < 0) {
		printk("Failed to reset device");
		return -EIO;
	}

	/* wait 30ms as reported in AN5763 */
	k_sleep(K_MSEC(30));

	fs = cfg->accel_range;
	printk("accel range is %d\n", fs);
	if (ism330bx_accel_set_fs_raw(dev, fs) < 0) {
		printk("failed to set accelerometer range %d\n", fs);
		return -EIO;
	}
	ism330bx->acc_gain = ism330bx_accel_fs_map[fs] * GAIN_UNIT_XL / 2;

	odr = cfg->accel_odr;
	printk("accel odr is %d [Note: 0 is power off]\n", odr);
	if (ism330bx_accel_set_odr_raw(dev, odr) < 0) {
		printk("failed to set accelerometer odr %d\n", odr);
		return -EIO;
	}

	fs = cfg->gyro_range;
	printk("gyro range is %d\n", fs);
	if (ism330bx_gyro_set_fs_raw(dev, fs) < 0) {
		printk("failed to set gyroscope range %d\n", fs);
		return -EIO;
	}
	ism330bx->gyro_gain = (ism330bx_gyro_fs_sens[fs] * GAIN_UNIT_G);

	odr = cfg->gyro_odr;
	printk("gyro odr is %d [Note: 0 is power off]\n", odr);
	ism330bx->gyro_freq = odr;
	if (ism330bx_gyro_set_odr_raw(dev, odr) < 0) {
		printk("failed to set gyroscope odr %d\n", odr);
		return -EIO;
	}

	if (ism330bx_block_data_update_set(ctx, 1) < 0) {
		printk("failed to set BDU mode\n");
		return -EIO;
	}

	return 0;
}

static int ism330bx_init(const struct device *dev)
{
#ifdef CONFIG_ISM330BX_TRIGGER
	const struct ism330bx_config *cfg = dev->config;
#endif
	struct ism330bx_data *data = dev->data;
	printk("Initialize device %s\n", dev->name);
	data->dev = dev;

	// if (ism330bx_pm_action(dev, PM_DEVICE_ACTION_TURN_ON) < 0) {
	// 	printk("failed to ism330bx_turn_on");
	// 	return -ENOEXEC;
	// }

	if (ism330bx_init_chip(dev) < 0) {
		printk("failed to initialize chip\n");
		return -EIO;
	}

#ifdef CONFIG_ISM330BX_TRIGGER
	if (cfg->trig_enabled) {
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

	// if (ism330bx_pm_action(dev, PM_DEVICE_ACTION_TURN_OFF) < 0) {
	// 	printk("failed to ism330bx_turn_off");
	// 	return -ENOEXEC;
	// }
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
	SENSOR_DEVICE_DT_INST_DEFINE(inst, ism330bx_init, NULL, &ism330bx_data_##inst,             \
				     &ism330bx_config_##inst, POST_KERNEL,                         \
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
