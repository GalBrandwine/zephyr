/*
 * Copyright (c) 2018 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/sys/util.h>

static int print_samples;
static int ism330bx_trig_cnt;

static struct sensor_value accel_x_out, accel_y_out, accel_z_out;
static struct sensor_value gyro_x_out, gyro_y_out, gyro_z_out;
// #if defined(CONFIG_LSM6DSL_EXT0_LIS2MDL)
// static struct sensor_value magn_x_out, magn_y_out, magn_z_out;
// #endif
// #if defined(CONFIG_LSM6DSL_EXT0_LPS22HB)
// static struct sensor_value press_out, temp_out;
// #endif

#ifdef CONFIG_ISM330BX_TRIGGER
static void ism330bx_trigger_handler(const struct device *dev, const struct sensor_trigger *trig)
{
	static struct sensor_value accel_x, accel_y, accel_z;
	static struct sensor_value gyro_x, gyro_y, gyro_z;
#if defined(CONFIG_LSM6DSL_EXT0_LIS2MDL)
	static struct sensor_value magn_x, magn_y, magn_z;
#endif
#if defined(CONFIG_LSM6DSL_EXT0_LPS22HB)
	static struct sensor_value press, temp;
#endif
	ism330bx_trig_cnt++;

	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &accel_x);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &accel_z);

	/* ism330bx gyro */
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_GYRO_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_X, &gyro_x);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Y, &gyro_y);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Z, &gyro_z);

#if defined(CONFIG_LSM6DSL_EXT0_LIS2MDL)
	/* ism330bx external mag */
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_MAGN_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_MAGN_X, &magn_x);
	sensor_channel_get(dev, SENSOR_CHAN_MAGN_Y, &magn_y);
	sensor_channel_get(dev, SENSOR_CHAN_MAGN_Z, &magn_z);
#endif

#if defined(CONFIG_LSM6DSL_EXT0_LPS22HB)
	/* ism330bx external press/temp */
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_PRESS);
	sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press);

	sensor_sample_fetch_chan(dev, SENSOR_CHAN_AMBIENT_TEMP);
	sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
#endif

	if (print_samples) {
		print_samples = 0;

		accel_x_out = accel_x;
		accel_y_out = accel_y;
		accel_z_out = accel_z;

		gyro_x_out = gyro_x;
		gyro_y_out = gyro_y;
		gyro_z_out = gyro_z;

#if defined(CONFIG_LSM6DSL_EXT0_LIS2MDL)
		magn_x_out = magn_x;
		magn_y_out = magn_y;
		magn_z_out = magn_z;
#endif

#if defined(CONFIG_LSM6DSL_EXT0_LPS22HB)
		press_out = press;
		temp_out = temp;
#endif
	}
}
#endif

int main(void)
{
	int cnt = 0;
	char out_str[64];
	struct sensor_value odr_attr;
	const struct device *const ism330bx_dev = DEVICE_DT_GET_ONE(st_ism330bx);

	if (!device_is_ready(ism330bx_dev)) {
		printk("sensor: device not ready.\n");
		return 0;
	}

	static const struct gpio_dt_spec ism330bx_power =
		GPIO_DT_SPEC_GET_OR(DT_NODELABEL(ism330bx_on), gpios, {0});
	if (!device_is_ready(ism330bx_power.port)) {
		printk("power: device not ready.\n");
		return 0;
	}
	gpio_pin_configure_dt(&ism330bx_power, GPIO_OUTPUT_INACTIVE);
	int ret = gpio_pin_set_dt(&ism330bx_power, 0);
	if (ret < 0) {
		printk("Error %d: Failed to set pin %d to high\n", ret, ism330bx_power.pin);
		// Handle error
	} else {
		printk("Successfully set pin %d to high (1)\n", ism330bx_power.pin);
	}
	/* set accel/gyro sampling frequency to 60 Hz */
	odr_attr.val1 = 60;
	odr_attr.val2 = 0;

	if (sensor_attr_set(ism330bx_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY,
			    &odr_attr) < 0) {
		printk("Cannot set sampling frequency for accelerometer.\n");
		return 0;
	}

	if (sensor_attr_set(ism330bx_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY,
			    &odr_attr) < 0) {
		printk("Cannot set sampling frequency for gyro.\n");
		return 0;
	}

#ifdef CONFIG_ISM330BX_TRIGGER
	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;

	if (sensor_trigger_set(ism330bx_dev, &trig, ism330bx_trigger_handler) != 0) {
		printk("Could not set sensor type and channel\n");
		return 0;
	}
#endif

	if (sensor_sample_fetch(ism330bx_dev) < 0) {
		printk("Sensor sample update error\n");
		return 0;
	}

	while (1) {
		/* Erase previous */
		printk("\0033\014");
		printf("ISM330BX sensor samples:\n\n");

		/* ism330bx accel */
		sprintf(out_str, "accel x:%f ms/2 y:%f ms/2 z:%f ms/2",
			sensor_value_to_double(&accel_x_out), sensor_value_to_double(&accel_y_out),
			sensor_value_to_double(&accel_z_out));
		printk("%s\n", out_str);

		/* ism330bx gyro */
		sprintf(out_str, "gyro x:%f dps y:%f dps z:%f dps",
			sensor_value_to_double(&gyro_x_out), sensor_value_to_double(&gyro_y_out),
			sensor_value_to_double(&gyro_z_out));
		printk("%s\n", out_str);

#if defined(CONFIG_LSM6DSL_EXT0_LIS2MDL)
		/* ism330bx external magn */
		sprintf(out_str, "magn x:%f gauss y:%f gauss z:%f gauss",
			sensor_value_to_double(&magn_x_out), sensor_value_to_double(&magn_y_out),
			sensor_value_to_double(&magn_z_out));
		printk("%s\n", out_str);
#endif

#if defined(CONFIG_LSM6DSL_EXT0_LPS22HB)
		/* ism330bx external press/temp */
		sprintf(out_str, "press: %f kPa - temp: %f deg", sensor_value_to_double(&press_out),
			sensor_value_to_double(&temp_out));
		printk("%s\n", out_str);
#endif

		printk("loop:%d trig_cnt:%d\n\n", ++cnt, ism330bx_trig_cnt);

		print_samples = 1;
		k_sleep(K_MSEC(2000));
	}
}
