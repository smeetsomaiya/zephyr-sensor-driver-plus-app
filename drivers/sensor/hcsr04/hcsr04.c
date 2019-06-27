/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <i2c.h>
#include <gpio.h>
#include <kernel.h>
#include <sensor.h>
#include <gpio.h>
#include <pwm.h>
#include <pinmux.h>
#include <stdio.h>
#include <string.h>
#include <misc/util.h>
#include <misc/__assert.h>
#include <logging/log.h>
#include "../boards/x86/galileo/board.h"
#include "../boards/x86/galileo/pinmux_galileo.h"
#include "../drivers/gpio/gpio_dw_registers.h"
#include "../drivers/gpio/gpio_dw.h"

#include "hcsr04.h"


#define EDGE_FALLING    (GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW)
#define EDGE_RISING	(GPIO_INT_EDGE | GPIO_INT_ACTIVE_HIGH)

static struct device *pinmux;

const int io_to_zephyr[5] = {3, 4, 5, 6, 7};

#define LOG_LEVEL CONFIG_SENSOR_LOG_LEVEL
LOG_MODULE_REGISTER(HCSR04);

static void hcsr04_gpio_callback(struct device *dev,
				  struct gpio_callback *cb, u32_t pins)
{
	struct hcsr04_data *drv_data =
		CONTAINER_OF(cb, struct hcsr04_data, gpio_cb);

	drv_data->status = 1;

	struct isr_data *callback_data = drv_data->cb_data;

	int ret, val;
	if(callback_data->start_time == 0) {
		callback_data->start_time = k_cycle_get_32();
	} else {
		callback_data->stop_time = k_cycle_get_32();
	}

	ret = gpio_pin_read(drv_data->gpio, io_to_zephyr[drv_data->echo_pin], &val);

	if(val == 1) {
		ret = gpio_pin_configure(drv_data->gpio, io_to_zephyr[drv_data->echo_pin], GPIO_DIR_IN | GPIO_INT | EDGE_FALLING);
		if(ret < 0) {
			printk("error setting falling edge for %d\n", drv_data->echo_pin);
			return;
		}
	} else if(val == 0) {
		//Calculate pulse width

		callback_data->distance_cm = (SYS_CLOCK_HW_CYCLES_TO_NS(
					   callback_data->stop_time - callback_data->start_time))/58000;
		drv_data->status = 0;
		k_sem_give(&drv_data->data_sem); //Indicate the value is ready to channel get		

		if(callback_data->distance_cm < 0) printk("Negative distance");
		callback_data->start_time = callback_data->stop_time = 0;
		printk("Distance is %ld from device\n",callback_data->distance_cm);

		ret = gpio_pin_configure(drv_data->gpio, io_to_zephyr[drv_data->echo_pin], GPIO_DIR_IN | GPIO_INT | EDGE_RISING);
		if(ret < 0) {
			printk("error setting rising edge for %d\n", drv_data->echo_pin);
			return;
		}
	}
}


static int initGPIO(int cmp, struct hcsr04_data *hcsr_data) {
	pinmux = device_get_binding(CONFIG_PINMUX_NAME);
	struct galileo_data *dev = pinmux->driver_data;

	hcsr_data->gpio = dev->gpio_dw; 	//retrieving gpio_dw driver struct from pinmux driver
				// Alternatively, gpiob = device_get_binding(PORT);
	if (!hcsr_data->gpio) {
		printk("error\n");
		return -EIO;
	}

	int ret;
	
	if(!cmp) {
		hcsr_data->trigger_pin = CONFIG_HCSR04_0_TRIG_PIN;		
		hcsr_data->echo_pin = CONFIG_HCSR04_0_ECHO_PIN;
	} else {
		hcsr_data->trigger_pin = CONFIG_HCSR04_1_TRIG_PIN;
		hcsr_data->echo_pin = CONFIG_HCSR04_1_ECHO_PIN;
	}

	ret = pinmux_pin_set(pinmux,hcsr_data->trigger_pin,PINMUX_FUNC_A); 	//IO1 or zephyr GPIO4 as output
	if(ret < 0) {
		printk("error setting pin for IO1\n");
		return ret;
	}
	ret = pinmux_pin_set(pinmux,hcsr_data->echo_pin,PINMUX_FUNC_B); 	//IO2 or zephyr GPIO5 as input
	if(ret < 0) {
		printk("error setting pin for IO2\n");
		return ret;
	}
	
	//Configure interrupt
	ret = gpio_pin_configure(hcsr_data->gpio, io_to_zephyr[hcsr_data->echo_pin], GPIO_DIR_IN | GPIO_INT | EDGE_RISING);
	if(ret < 0) {
		printk("error setting pin for IO2\n");
		return ret;
	}	

	struct gpio_callback gpio_cb;
	hcsr_data->gpio_cb = gpio_cb;

	gpio_init_callback(&hcsr_data->gpio_cb, hcsr04_gpio_callback, BIT(io_to_zephyr[hcsr_data->echo_pin]));

	ret = gpio_add_callback(hcsr_data->gpio, &hcsr_data->gpio_cb);
	if(ret < 0) {
		printk("error in gpio_add_callback %d\n", ret);
		return ret;
	}
	ret = gpio_pin_enable_callback(hcsr_data->gpio, io_to_zephyr[hcsr_data->echo_pin]);
	if(ret < 0) {
		printk("error in gpio_pin_enable_callback %d\n", ret);
		return ret;
	}
	printk("GPIO init done\n");
	return 0;
}


static int hcsr04_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	struct hcsr04_data *drv_data = dev->driver_data;
	if(drv_data->status) {
		printk("hcsr04_sample_fetch BUSY\n");
		return -1;
	}
	printk("hcsr04_sample_fetch\n");

	//Discard old data when starting a new measurement - It will already be 0 if no race condition
//	drv_data->cb_data->distance_cm = 0;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

//	printk("Sending trigger on pin %d\n",drv_data->trigger_pin);
	
	int ret = gpio_pin_write(drv_data->gpio, io_to_zephyr[drv_data->trigger_pin], 0);
	if(ret < 0) {
		printk("Error sending trigger\n");
	}
	ret = gpio_pin_write(drv_data->gpio, io_to_zephyr[drv_data->trigger_pin], 1);
	if(ret < 0) {
		printk("Error sending trigger\n");
	}
	k_sleep(10);
	ret = gpio_pin_write(drv_data->gpio, io_to_zephyr[drv_data->trigger_pin], 0);
	if(ret < 0) {
		printk("Error sending trigger\n");
	}	
	return 0;
}

static int hcsr04_channel_get(struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	printk("hcsr04_channel_get\n");
	struct hcsr04_data *drv_data = dev->driver_data;
	if(drv_data->status && !drv_data->cb_data->distance_cm) {
		if(k_sem_take(&drv_data->data_sem, K_MSEC(drv_data->timeout)) != 0) {
			printk("Timed out for %d\n", drv_data->timeout);
			return -1;
		} else {
			val->val1 = drv_data->cb_data->distance_cm;
			if(val->val1 == 0) {
				printk("val %d %d %d\n",val->val1, drv_data->status, drv_data->cb_data->distance_cm);
			}
			drv_data->cb_data->distance_cm = 0;
		}
	} else if(!drv_data->status && !drv_data->cb_data->distance_cm) {
		printk("Re-triggering\n");
		hcsr04_sample_fetch(dev, SENSOR_CHAN_ALL);

		if(k_sem_take(&drv_data->data_sem, K_MSEC(drv_data->timeout)) != 0) {
			printk("Timed out for %d\n", drv_data->timeout);
			return -1;
		} else {
			val->val1 = drv_data->cb_data->distance_cm;
			if(val->val1 == 0) {
				printk("val %d %d %d\n",val->val1, drv_data->status, drv_data->cb_data->distance_cm);
			}
			drv_data->cb_data->distance_cm = 0;
		}
	} else {
		val->val1 = drv_data->cb_data->distance_cm;
		if(val->val1 == 0) {
			printk("val %d %d %d\n",val->val1, drv_data->status, drv_data->cb_data->distance_cm);
		}
		drv_data->cb_data->distance_cm = 0; //Clear after each channel get
	}
	return 0;
}

static const struct sensor_driver_api hcsr04_driver_api = {
	.sample_fetch = hcsr04_sample_fetch,	
	.channel_get = hcsr04_channel_get,
};

static int hcsr04_init(struct device *dev)
{
	printk("hcsr04_init %s\n", dev->config->name);

	struct hcsr04_data *drv_data = dev->driver_data;

	int cmp = strcmp(dev->config->name, "HCSR04_0");

	int ret = initGPIO(cmp, drv_data); //Set the GPIO config
	if(ret) {
		printk("Error initializing GPIO\n");
		return -EIO;
	}
	drv_data->status = 0;
	drv_data->timeout = CONFIG_HCSR04_TIMEOUT;
	k_sem_init(&drv_data->data_sem, 0, 1); //Starts as unavailable
	return 0;
}

static struct hcsr04_data hcsr04_0_data, hcsr04_1_data;

DEVICE_AND_API_INIT(HCSR04_00, "HCSR04_0", hcsr04_init, &hcsr04_0_data,
		    NULL, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
		    &hcsr04_driver_api);
DEVICE_AND_API_INIT(HCSR04_01, "HCSR04_1", hcsr04_init, &hcsr04_1_data,
		    NULL, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
		    &hcsr04_driver_api);
