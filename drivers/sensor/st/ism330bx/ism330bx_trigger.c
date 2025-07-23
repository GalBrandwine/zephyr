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

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include "ism330bx.h"
#include "ism330bx_sensor.h"
#if defined(CONFIG_ISM330BX_LOG_LEVEL_DBG)
#include "ism330bx_devtools.h"
#endif /* CONFIG_ISM330BX_LOG_LEVEL_DBG */
LOG_MODULE_DECLARE(ism330bx, CONFIG_ISM330BX_LOG_LEVEL);

enum ism330bx_interrupt_pins {
	wake_up,
	significant_motion,
	orientation_changed,
	drdy_accel,
	drdy_gyro
};

void set_interrupt_pin(const struct device *dev, enum ism330bx_interrupt_pins interrupt_pin);

#if defined(CONFIG_ISM330BX_WAKEUP_DETECTION)
/**
 * @brief Set wake up interrupt
 *
 * @param dev Pointer to the device structure
 *
 * See section 5.3 6D Wake-up interrupt in
 * https://www.st.com/resource/en/application_note/an6109-ism330bx-6axis-imu-with-wide-bandwidth-lownoise-accelerometer-embedded-sensor-fusion-and-ai-for-industrial-applications-stmicroelectronics.pdf
 *
 */
static int ism330bx_enable_wake_up_int(const struct device *dev, int enable)
{
	const struct ism330bx_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)ism330bx_data->ism330bx_spi_data.ctx;
	int ret;

	if (enable) {

		ism330bx_ctrl9_t ctrl9;
		ret += ism330bx_filt_xl_fast_settling_get(ctx, &ctrl9);
		ctrl9.xl_fastsettl_mode = 1;
		ret += ism330bx_filt_xl_fast_settling_set(ctx, ctrl9.xl_fastsettl_mode);
		LOG_DBG("done step 2.1 - fast settling mode");

		// 1.Write 34h to INACTIVITY_DUR// Set wake-up threshold resolution to 62.5 mg
		ism330bx_inactivity_dur_t inactivity_dur;
		ret = ism330bx_read_reg(ctx, ISM330BX_INACTIVITY_DUR, &inactivity_dur, 1);
		inactivity_dur.wu_inact_ths_w = 0b011; // Inactivity threshold = 62.5 mg
		inactivity_dur.xl_inact_odr = 1;
		ret += ism330bx_write_reg(ctx, ISM330BX_INACTIVITY_DUR, &inactivity_dur, 1);
		LOG_DBG("done step 1");
		// 2.Write 11h to TAP_CFG0// Select HPF path and enable latched mode
		ism330bx_tap_cfg0_t tap_cfg0;
		ret += ism330bx_read_reg(ctx, ISM330BX_TAP_CFG0, &tap_cfg0, 1);
		tap_cfg0.lir = 0; // Latched
		tap_cfg0.slope_fds = 1;
		tap_cfg0.hw_func_mask_xl_settl =
			1; // Raise this Bit to avoid spurious interrupts generation
		ret += ism330bx_write_reg(ctx, ISM330BX_TAP_CFG0, &tap_cfg0, 1);
		LOG_DBG("done step 2");

		// 3.Write 01h to WAKE_UP_THS// Set wake-up threshold
		ism330bx_wake_up_ths_t wake_up_ths;
		ret += ism330bx_read_reg(ctx, ISM330BX_WAKE_UP_THS, &wake_up_ths, 1);
		wake_up_ths.wk_ths = 0b01;
		ret += ism330bx_write_reg(ctx, ISM330BX_WAKE_UP_THS, &wake_up_ths, 1);
		LOG_DBG("done step 3");
		// 4.Write 00h to WAKE_UP_DUR// Set duration to 0
		// ret += ism330bx_write_reg(ctx, ISM330BX_WAKE_UP_DUR, 0x00, 1);
		ism330bx_act_wkup_time_windows_t window;
		ret += ism330bx_act_wkup_time_windows_get(ctx, &window);
		window.shock = 0;
		window.quiet = 0;
		ret += ism330bx_act_wkup_time_windows_set(ctx, window);
		LOG_DBG("done step 4");
	} else {
		return EXIT_SUCCESS;
	}
	__ASSERT(ret == 0, "Failed to set wake_up mode");

	/*set interrupt */
	set_interrupt_pin(dev, wake_up);
	return ret;
}
#endif /* CONFIG_ISM330BX_WAKEUP_DETECTION */

#if defined(CONFIG_ISM330BX_SIGNIFICANT_MOTION_DETECTION)
/**
 * @brief Set significant motion detection
 * @note The significant motion feature automatically enables the internal step counter algorithm.
 * Event is trigger after 10 steps.
 *
 * @param dev Pointer to the device structure
 *
 * See section 6.2 6D Significant motion in
 * https://www.st.com/resource/en/application_note/an6109-ism330bx-6axis-imu-with-wide-bandwidth-lownoise-accelerometer-embedded-sensor-fusion-and-ai-for-industrial-applications-stmicroelectronics.pdf
 *
 */
static int ism330bx_enable_significant_motion_int(const struct device *dev, int enable)
{
	const struct ism330bx_config *cfg = dev->config;
	struct ism330bx_data *ism330bx_data = dev->data;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)ism330bx_data->ism330bx_spi_data.ctx;
	int ret;
	ism330bx_pin_int_route_t val;

	if (enable) {
		ism330bx_xl_data_rate_t odr;
		if (ism330bx_xl_data_rate_get(ctx, &odr) < 0) {
			return -EIO;
		}

		/*
		NOTE:
		The significant motion function works at 30 Hz, so the accelerometer ODR must be set
		at a value of 30 Hz or higher. It generates an interrupt when the difference between
		the number of steps counted from its initialization/reset is higher than 10 steps.
		After an interrupt generation, the algorithm internal state is reset.
		*/
		__ASSERT(odr >= ISM330BX_XL_ODR_AT_30Hz,
			 "Significant motion detection requires ACCEL ODR >= 30Hz");

		int16_t buf[3];
		/* dummy read: re-trigger interrupt */
		ism330bx_angular_rate_raw_get(ctx, buf);

		LOG_DBG("setting significant_motion_detection");
		/*
		1. Write 80h to FUNC_CFG_ACCESS// Enable access to embedded functions registers
		2. Write 20h to EMB_FUNC_EN_A// Enable significant motion detection
		Write 00h to FUNC_CFG_ACCESS// Disable access to embedded functions registers
		*/
		ret = ism330bx_sigmot_mode_set(ctx, ISM330BX_EN_BIT);
	} else {
		return ism330bx_sigmot_mode_set(ctx, ISM330BX_DIS_BIT);
	}
	__ASSERT(ret == 0, "Failed to set sigmot mode");

	/*set interrupt */
	set_interrupt_pin(dev, significant_motion);

	return ret;
}
#endif /* CONFIG_ISM330BX_SIGNIFICANT_MOTION_DETECTION */

/**
 * ism330bx_enable_g_int - Orientation change -  enable selected int pin to generate interrupt
 */
static int ism330bx_enable_six_d_int(const struct device *dev, int enable)
{
	const struct ism330bx_config *cfg = dev->config;
	struct ism330bx_data *ism330bx_data = dev->data;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)ism330bx_data->ism330bx_spi_data.ctx;
	int ret;
	ism330bx_pin_int_route_t val;

	if (enable) {
		int16_t buf[3];

		/* dummy read: re-trigger interrupt */
		ism330bx_angular_rate_raw_get(ctx, buf);
	}

	/* set interrupt */
	set_interrupt_pin(dev, orientation_changed);
	return ret;
}

/**
 * ism330bx_enable_xl_int - XL enable selected int pin to generate interrupt
 */
static int ism330bx_enable_xl_int(const struct device *dev, int enable)
{
	const struct ism330bx_config *cfg = dev->config;
	struct ism330bx_data *ism330bx_data = dev->data;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)ism330bx_data->ism330bx_spi_data.ctx;
	int ret = 0;

	if (enable) {
		int16_t buf[3];

		/* dummy read: re-trigger interrupt */
		ret = ism330bx_acceleration_raw_get(ctx, buf);
	}

	/* set interrupt */
	set_interrupt_pin(dev, drdy_accel);

	return ret;
}

/**
 * ism330bx_enable_g_int - Gyro enable selected int pin to generate interrupt
 */
static int ism330bx_enable_g_int(const struct device *dev, int enable)
{
	const struct ism330bx_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	int ret;

	if (enable) {
		ism330bx_gy_mode_t gy_mode;
		ret += ism330bx_gy_mode_get(ctx, &gy_mode);
		if (gy_mode == ISM330BX_GY_SLEEP_MD) {
			LOG_WRN("gyro is in sleep mode. It will not trigger interrupts")
		}

		int16_t buf[3];

		/* dummy read: re-trigger interrupt */
		ism330bx_angular_rate_raw_get(ctx, buf);
	}

	/* set interrupt */
	set_interrupt_pin(dev, drdy_gyro);

	return ret;
}

void set_interrupt_pin(const struct device *dev, enum ism330bx_interrupt_pins interrupt_pin)
{
	int ret = 0;
	const struct ism330bx_config *cfg = dev->config;
	struct ism330bx_data *ism330bx_data = dev->data;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)ism330bx_data->ism330bx_spi_data.ctx;
	ism330bx_pin_int_route_t interrupt_flags;
	if (cfg->drdy_pin == 1) {
		ret = ism330bx_pin_int1_route_get(ctx, &interrupt_flags);
		__ASSERT(ret == 0, "pint_int1_route_get error");

		switch (interrupt_pin) {
		case wake_up:
			interrupt_flags.wake_up = ISM330BX_EN_BIT;
			break;
		case significant_motion:
			interrupt_flags.sig_mot = ISM330BX_EN_BIT;
			break;
		case orientation_changed:
			interrupt_flags.six_d = ISM330BX_EN_BIT;
			break;
		case drdy_accel:
			interrupt_flags.drdy_xl = ISM330BX_EN_BIT;
			break;
		case drdy_gyro:
			interrupt_flags.drdy_gy = ISM330BX_EN_BIT;
			break;

		default:
			break;
		}

		ret = ism330bx_pin_int1_route_set(ctx, interrupt_flags);
		__ASSERT(ret == 0, "Failed to set wake_up route");

		// Sanity check
		memset(&interrupt_flags, sizeof(ism330bx_pin_int_route_t), 0);
		ret = ism330bx_pin_int1_route_get(ctx, &interrupt_flags);

		switch (interrupt_pin) {
		case wake_up:
			__ASSERT(ret == 0 && interrupt_flags.wake_up == ISM330BX_EN_BIT,
				 "failed setting wake_up");
			break;
		case significant_motion:
			__ASSERT(ret == 0 && interrupt_flags.sig_mot == ISM330BX_EN_BIT,
				 "failed setting sig_mot");
			break;
		case orientation_changed:
			__ASSERT(ret == 0 && interrupt_flags.six_d == ISM330BX_EN_BIT,
				 "failed setting six_d");
			break;
		case drdy_accel:
			__ASSERT(ret == 0 && interrupt_flags.drdy_xl == ISM330BX_EN_BIT,
				 "failed setting drdy_xl");
			break;
		case drdy_gyro:
			__ASSERT(ret == 0 && interrupt_flags.drdy_gy == ISM330BX_EN_BIT,
				 "failed setting drdy_gy");
			break;
		default:
			break;
		}
	} else {

		ret = ism330bx_pin_int2_route_get(ctx, &interrupt_flags);
		__ASSERT(ret == 0, "pint_int2_route_get error");

		switch (interrupt_pin) {
		case wake_up:
			interrupt_flags.wake_up = ISM330BX_DIS_BIT;
			break;
		case significant_motion:
			interrupt_flags.sig_mot = ISM330BX_DIS_BIT;
			break;
		case orientation_changed:
			interrupt_flags.six_d = ISM330BX_DIS_BIT;
			break;
		case drdy_accel:
			interrupt_flags.drdy_xl = ISM330BX_DIS_BIT;
			break;
		case drdy_gyro:
			interrupt_flags.drdy_gy = ISM330BX_DIS_BIT;
			break;
		default:
			break;
		}
		ret = ism330bx_pin_int2_route_set(ctx, interrupt_flags);
		__ASSERT(ret == 0, "Failed to disable interrupt pin route");
	}
}

/**
 * ism330bx_trigger_set - link external trigger to event data ready
 */
int ism330bx_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
			 sensor_trigger_handler_t handler)
{
	const struct ism330bx_config *cfg = dev->config;
	struct ism330bx_data *ism330bx = dev->data;

	if (!cfg->trig_enabled) {
		LOG_ERR("trigger_set op not supported");
		return -ENOTSUP;
	}

	switch (trig->chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		LOG_DBG("[SENSOR_CHAN_ACCEL_XYZ] assigning handler and trigger-type to drdy_acc");
		ism330bx->handler_drdy_acc = handler;
		ism330bx->trig_drdy_acc = trig;
		if (handler) {
			return ism330bx_enable_xl_int(dev, ISM330BX_EN_BIT);
		} else {
			return ism330bx_enable_xl_int(dev, ISM330BX_DIS_BIT);
		}
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		LOG_DBG("[SENSOR_CHAN_GYRO_XYZ] assigning handler and trigger-type to drdy_gyr");
		ism330bx->handler_drdy_gyr = handler;
		ism330bx->trig_drdy_gyr = trig;
		if (handler) {
			return ism330bx_enable_g_int(dev, ISM330BX_EN_BIT);
		} else {
			return ism330bx_enable_g_int(dev, ISM330BX_DIS_BIT);
		}
		break;
	case SENSOR_CHAN_POS_DXYZ:
		switch (trig->type) {
		case SENSOR_TRIG_MOTION:
#if defined(CONFIG_ISM330BX_6D_ORIENTATION_DETECTION)
#if defined(CONFIG_ISM330BX_LOG_LEVEL_DBG)
			ism330bx_helper_print_orientation_interrupt_settings(
				(stmdev_ctx_t *)&cfg->ctx);
#endif /* CONFIG_ISM330BX_LOG_LEVEL_DBG */
			LOG_DBG("assigning handler and trigger-type to drdy_d6d");
			ism330bx->handler_drdy_d6d = handler;
			ism330bx->trig_drdy_d6d = trig;
			if (handler) {
				return ism330bx_enable_six_d_int(dev, ISM330BX_EN_BIT);
			} else {
				LOG_WRN("No handler given, disabling six_d interrupt");
				return ism330bx_enable_six_d_int(dev, ISM330BX_DIS_BIT);
			}
#else
			LOG_ERR("CONFIG_ISM330BX_6D_ORIENTATION_DETECTION not enabled");
			return -ENOTSUP;
#endif /* CONFIG_ISM330BX_6D_ORIENTATION_DETECTION */
		case SENSOR_TRIG_SIGNIFICANT_MOTION:
#if defined(CONFIG_ISM330BX_SIGNIFICANT_MOTION_DETECTION)
			LOG_DBG("assigning handler and trigger-type to drdy_sig_mot");
			ism330bx->handler_drdy_sig_mot = handler;
			ism330bx->trig_drdy_sig_mot = trig;
			if (handler) {
				return ism330bx_enable_significant_motion_int(dev, ISM330BX_EN_BIT);
			} else {
				LOG_WRN("No handler given, disabling six_d interrupt");
				return ism330bx_enable_significant_motion_int(dev,
									      ISM330BX_DIS_BIT);
			}

#else
			LOG_ERR("SENSOR_TRIG_SIGNIFICANT_MOTION not configured");
			return -ENOTSUP;
#endif
#if defined(CONFIG_ISM330BX_WAKEUP_DETECTION)
		case SENSOR_TRIG_WAKE_UP:
			LOG_DBG("assigning handler and trigger-type to drdy_wake_up");
			ism330bx->handler_drdy_wake_up = handler;
			ism330bx->trig_drdy_wake_up = trig;
			if (handler) {
				return ism330bx_enable_wake_up_int(dev, ISM330BX_EN_BIT);
			} else {
				LOG_WRN("No handler given, disabling six_d interrupt");
				return ism330bx_enable_wake_up_int(dev, ISM330BX_DIS_BIT);
			}
#else
			LOG_ERR("SENSOR_TRIG_WAKE_UP not configured");
			return -ENOTSUP;
#endif /* CONFIG_ISM330BX_WAKEUP_DETECTION */
		default:
			LOG_ERR("unexpected trigger type %d", trig->type);
		}
	default:
		LOG_ERR("unexpected channel %d", trig->chan);
		return -ENOTSUP;
	}

	return -ENOTSUP;
}

static int ism330bx_handle_drdy_acc_int(const struct device *dev)
{
	struct ism330bx_data *ism330bx = dev->data;
	sensor_trigger_handler_t handler = ism330bx->handler_drdy_acc;

	if (handler) {
		handler(dev, ism330bx->trig_drdy_acc);
	} else {
		LOG_ERR("handler is NULL");
	}

	return 0;
}
static int ism330bx_handle_drdy_gyr_int(const struct device *dev)
{
	struct ism330bx_data *ism330bx = dev->data;
	sensor_trigger_handler_t handler = ism330bx->handler_drdy_gyr;

	if (handler) {
		handler(dev, ism330bx->trig_drdy_gyr);
	} else {
		LOG_ERR("handler is NULL");
	}

	return 0;
}

#ifdef CONFIG_ISM330BX_WAKEUP_DETECTION
static int ism330bx_handle_wake_up_int(const struct device *dev)
{
	struct ism330bx_data *ism330bx = dev->data;
	sensor_trigger_handler_t handler = ism330bx->handler_drdy_wake_up;

	if (handler) {

		handler(dev, ism330bx->trig_drdy_wake_up);
	} else {
		LOG_ERR("handler is NULL");
	}

	return 0;
}
#endif /* CONFIG_ISM330BX_WAKEUP_DETECTION */

#ifdef CONFIG_ISM330BX_SIGNIFICANT_MOTION_DETECTION
static int ism330bx_handle_sig_mot_int(const struct device *dev)
{
	struct ism330bx_data *ism330bx = dev->data;
	sensor_trigger_handler_t handler = ism330bx->handler_drdy_sig_mot;

	if (handler) {

		handler(dev, ism330bx->trig_drdy_sig_mot);
	} else {
		LOG_ERR("handler is NULL");
	}

	return 0;
}
#endif /* CONFIG_ISM330BX_SIGNIFICANT_MOTION_DETECTION */
#ifdef CONFIG_ISM330BX_6D_ORIENTATION_DETECTION
static int ism330bx_handle_d6d_int(const struct device *dev)
{
	struct ism330bx_data *ism330bx = dev->data;
	sensor_trigger_handler_t handler = ism330bx->handler_drdy_d6d;

	if (handler) {

		handler(dev, ism330bx->trig_drdy_d6d);
	} else {
		LOG_ERR("handler is NULL");
	}

	return 0;
}
#endif /* CONFIG_ISM330BX_6D_ORIENTATION_DETECTION */

/**
 * ism330bx_handle_interrupt - handle the drdy event
 * read data and call handler if registered any
 */
static void ism330bx_handle_interrupt(const struct device *dev)
{
	struct ism330bx_data *data = dev->data;
	const struct ism330bx_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;

	ism330bx_all_sources_get(ctx, &data->all_sources);
	ism330bx_all_sources_t *sources = &data->all_sources;

	LOG_INF("handling interrupt [sources.drdy_xl: %d, sources.drdy_gy: %d, sources.six_d: %d]",
		sources->drdy_xl, sources->drdy_gy, sources->six_d);

#ifdef CONFIG_ISM330BX_SIGNIFICANT_MOTION_DETECTION
	if ((sources->sig_mot) && (data->handler_drdy_sig_mot != NULL)) {
		ism330bx_handle_sig_mot_int(dev);
	}
#endif /* CONFIG_ISM330BX_SIGNIFICANT_MOTION_DETECTION */
#ifdef CONFIG_ISM330BX_6D_ORIENTATION_DETECTION
	if ((sources->six_d) && (data->handler_drdy_d6d != NULL)) {
		ism330bx_handle_d6d_int(dev);
	}
#endif /* CONFIG_ISM330BX_6D_ORIENTATION_DETECTION */
#ifdef CONFIG_ISM330BX_WAKEUP_DETECTION
	if ((sources->wake_up) && (data->handler_drdy_wake_up != NULL)) {
		ism330bx_handle_wake_up_int(dev);
	}
#endif /* CONFIG_ISM330BX_WAKEUP_DETECTION */
	if ((sources->drdy_xl) && (data->handler_drdy_acc != NULL)) {
		ism330bx_handle_drdy_acc_int(dev);
	}

	if ((sources->drdy_gy) && (data->handler_drdy_gyr != NULL)) {
		ism330bx_handle_drdy_gyr_int(dev);
	}

	gpio_pin_interrupt_configure_dt(data->drdy_gpio, GPIO_INT_EDGE_TO_ACTIVE);
}

static void ism330bx_gpio_callback(const struct device *dev, struct gpio_callback *cb,
				   uint32_t pins)
{
	struct ism330bx_data *ism330bx = CONTAINER_OF(cb, struct ism330bx_data, gpio_cb);

	ARG_UNUSED(pins);

	gpio_pin_interrupt_configure_dt(ism330bx->drdy_gpio, GPIO_INT_DISABLE);

#if defined(CONFIG_ISM330BX_TRIGGER_OWN_THREAD)
	k_sem_give(&ism330bx->gpio_sem);
#elif defined(CONFIG_ISM330BX_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&ism330bx->work);
#endif /* CONFIG_ISM330BX_TRIGGER_OWN_THREAD */
}

#ifdef CONFIG_ISM330BX_TRIGGER_OWN_THREAD
static void ism330bx_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct ism330bx_data *ism330bx = p1;

	while (1) {
		k_sem_take(&ism330bx->gpio_sem, K_FOREVER);
		ism330bx_handle_interrupt(ism330bx->dev);
	}
}
#endif /* CONFIG_ISM330BX_TRIGGER_OWN_THREAD */

#ifdef CONFIG_ISM330BX_TRIGGER_GLOBAL_THREAD
static void ism330bx_work_cb(struct k_work *work)
{
	struct ism330bx_data *ism330bx = CONTAINER_OF(work, struct ism330bx_data, work);

	ism330bx_handle_interrupt(ism330bx->dev);
}
#endif /* CONFIG_ISM330BX_TRIGGER_GLOBAL_THREAD */

int ism330bx_init_interrupt(const struct device *dev)
{
	struct ism330bx_data *ism330bx = dev->data;
	const struct ism330bx_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	int ret;

	ism330bx->drdy_gpio = (cfg->drdy_pin == 1) ? (struct gpio_dt_spec *)&cfg->int1_gpio
						   : (struct gpio_dt_spec *)&cfg->int2_gpio;

	/* setup data ready gpio interrupt (INT1 or INT2) */
	if (!gpio_is_ready_dt(ism330bx->drdy_gpio)) {
		LOG_ERR("Cannot get pointer to drdy_gpio device");
		return -EINVAL;
	}

#if defined(CONFIG_ISM330BX_TRIGGER_OWN_THREAD)
	LOG_DBG("Configuring trigger using own_thread [Stack size: %d]",
		CONFIG_ISM330BX_THREAD_STACK_SIZE);
	k_sem_init(&ism330bx->gpio_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&ism330bx->thread, ism330bx->thread_stack,
			CONFIG_ISM330BX_THREAD_STACK_SIZE, ism330bx_thread, ism330bx, NULL, NULL,
			K_PRIO_COOP(CONFIG_ISM330BX_THREAD_PRIORITY), 0, K_NO_WAIT);
	k_thread_name_set(&ism330bx->thread, "ism330bx");
#elif defined(CONFIG_ISM330BX_TRIGGER_GLOBAL_THREAD)
	LOG_DBG("Configuring trigger using global thread");
	ism330bx->work.handler = ism330bx_work_cb;
#endif /* CONFIG_ISM330BX_TRIGGER_OWN_THREAD */

	ret = gpio_pin_configure_dt(ism330bx->drdy_gpio, GPIO_INPUT);
	if (ret < 0) {
		LOG_WRN("Could not configure gpio");
		return ret;
	}

	gpio_init_callback(&ism330bx->gpio_cb, ism330bx_gpio_callback,
			   BIT(ism330bx->drdy_gpio->pin));

	if (gpio_add_callback(ism330bx->drdy_gpio->port, &ism330bx->gpio_cb) < 0) {
		LOG_WRN("Could not set gpio callback");
		return -EIO;
	}

	/* set data ready mode on int1/int2 */
	ism330bx_data_ready_mode_t mode =
		cfg->drdy_pulsed ? ISM330BX_DRDY_PULSED : ISM330BX_DRDY_LATCHED;

	ret = ism330bx_data_ready_mode_set(ctx, mode);
	if (ret < 0) {
		LOG_WRN("drdy_pulsed config error %d", (int)cfg->drdy_pulsed);
		return ret;
	}

	return gpio_pin_interrupt_configure_dt(ism330bx->drdy_gpio, GPIO_INT_EDGE_TO_ACTIVE);
}
