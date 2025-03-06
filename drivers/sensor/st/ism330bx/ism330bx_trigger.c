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

#include "ism330bx.h"

LOG_MODULE_DECLARE(ISM330BX, CONFIG_SENSOR_LOG_LEVEL);

/**
 * ism330bx_enable_xl_int - XL enable selected int pin to generate interrupt
 */
static int ism330bx_enable_xl_int(const struct device *dev, int enable)
{
	const struct ism330bx_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	int ret;

	if (enable) {
		int16_t buf[3];

		/* dummy read: re-trigger interrupt */
		ism330bx_acceleration_raw_get(ctx, buf);
	}

	/* set interrupt */
	if (cfg->drdy_pin == 1) {
		ism330bx_pin_int_route_t val;

		ret = ism330bx_pin_int1_route_get(ctx, &val);
		if (ret < 0) {
			LOG_ERR("pint_int1_route_get error");
			return ret;
		}

		val.drdy_xl = 1;

		ret = ism330bx_pin_int1_route_set(ctx, val);
	} else {
		ism330bx_pin_int_route_t val;

		ret = ism330bx_pin_int2_route_get(ctx, &val);
		if (ret < 0) {
			LOG_ERR("pint_int2_route_get error");
			return ret;
		}

		val.drdy_xl = 1;

		ret = ism330bx_pin_int2_route_set(ctx, val);
	}

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
		int16_t buf[3];

		/* dummy read: re-trigger interrupt */
		ism330bx_angular_rate_raw_get(ctx, buf);
	}

	/* set interrupt */
	if (cfg->drdy_pin == 1) {
		ism330bx_pin_int_route_t val;

		ret = ism330bx_pin_int1_route_get(ctx, &val);
		if (ret < 0) {
			LOG_ERR("pint_int1_route_get error");
			return ret;
		}

		val.drdy_gy = 1;

		ret = ism330bx_pin_int1_route_set(ctx, val);
	} else {
		ism330bx_pin_int_route_t val;

		ret = ism330bx_pin_int2_route_get(ctx, &val);
		if (ret < 0) {
			LOG_ERR("pint_int2_route_get error");
			return ret;
		}

		val.drdy_gy = 1;

		ret = ism330bx_pin_int2_route_set(ctx, val);
	}

	return ret;
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

	if (trig->chan == SENSOR_CHAN_ACCEL_XYZ) {
		ism330bx->handler_drdy_acc = handler;
		ism330bx->trig_drdy_acc = trig;
		if (handler) {
			return ism330bx_enable_xl_int(dev, ISM330BX_EN_BIT);
		} else {
			return ism330bx_enable_xl_int(dev, ISM330BX_DIS_BIT);
		}
	} else if (trig->chan == SENSOR_CHAN_GYRO_XYZ) {
		ism330bx->handler_drdy_gyr = handler;
		ism330bx->trig_drdy_gyr = trig;
		if (handler) {
			return ism330bx_enable_g_int(dev, ISM330BX_EN_BIT);
		} else {
			return ism330bx_enable_g_int(dev, ISM330BX_DIS_BIT);
		}
	}

	return -ENOTSUP;
}

/**
 * ism330bx_handle_interrupt - handle the drdy event
 * read data and call handler if registered any
 */
static void ism330bx_handle_interrupt(const struct device *dev)
{
	struct ism330bx_data *ism330bx = dev->data;
	const struct ism330bx_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	ism330bx_data_ready_t status;

	while (1) {
		if (ism330bx_flag_data_ready_get(ctx, &status) < 0) {
			LOG_DBG("failed reading status reg");
			return;
		}

		if ((status.drdy_xl == 0) && (status.drdy_gy == 0)) {
			break;
		}

		if ((status.drdy_xl) && (ism330bx->handler_drdy_acc != NULL)) {
			ism330bx->handler_drdy_acc(dev, ism330bx->trig_drdy_acc);
		}

		if ((status.drdy_gy) && (ism330bx->handler_drdy_gyr != NULL)) {
			ism330bx->handler_drdy_gyr(dev, ism330bx->trig_drdy_gyr);
		}
	}

	gpio_pin_interrupt_configure_dt(ism330bx->drdy_gpio, GPIO_INT_EDGE_TO_ACTIVE);
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
	k_sem_init(&ism330bx->gpio_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&ism330bx->thread, ism330bx->thread_stack,
			CONFIG_ISM330BX_THREAD_STACK_SIZE, ism330bx_thread, ism330bx, NULL, NULL,
			K_PRIO_COOP(CONFIG_ISM330BX_THREAD_PRIORITY), 0, K_NO_WAIT);
	k_thread_name_set(&ism330bx->thread, "ism330bx");
#elif defined(CONFIG_ISM330BX_TRIGGER_GLOBAL_THREAD)
	ism330bx->work.handler = ism330bx_work_cb;
#endif /* CONFIG_ISM330BX_TRIGGER_OWN_THREAD */

	ret = gpio_pin_configure_dt(ism330bx->drdy_gpio, GPIO_INPUT);
	if (ret < 0) {
		LOG_DBG("Could not configure gpio");
		return ret;
	}

	gpio_init_callback(&ism330bx->gpio_cb, ism330bx_gpio_callback,
			   BIT(ism330bx->drdy_gpio->pin));

	if (gpio_add_callback(ism330bx->drdy_gpio->port, &ism330bx->gpio_cb) < 0) {
		LOG_DBG("Could not set gpio callback");
		return -EIO;
	}

	/* set data ready mode on int1/int2 */
	LOG_DBG("drdy_pulsed is %d", (int)cfg->drdy_pulsed);
	ism330bx_data_ready_mode_t mode =
		cfg->drdy_pulsed ? ISM330BX_DRDY_PULSED : ISM330BX_DRDY_LATCHED;

	ret = ism330bx_data_ready_mode_set(ctx, mode);
	if (ret < 0) {
		LOG_ERR("drdy_pulsed config error %d", (int)cfg->drdy_pulsed);
		return ret;
	}

	return gpio_pin_interrupt_configure_dt(ism330bx->drdy_gpio, GPIO_INT_EDGE_TO_ACTIVE);
}
