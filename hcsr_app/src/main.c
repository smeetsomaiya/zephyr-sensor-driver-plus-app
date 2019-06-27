/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <init.h>
#include <stdio.h>
#include <stdlib.h>
#include <sensor.h>
#include <shell/shell.h>
#include <misc/printk.h>
#include "../boards/x86/galileo/board.h"
#include "../boards/x86/galileo/pinmux_galileo.h"
#include "../drivers/gpio/gpio_dw_registers.h"
#include "../drivers/gpio/gpio_dw.h"

#include <gpio.h>
#include <pwm.h>
#include <pinmux.h>

#include <stdio.h>
#include <string.h>

#define SLEEP_TIME	1000

long distance_cm = 0;
int start, stop, elapsed_start, elapsed_stop[256]={0};
int t_count = 0;
int array11[100] = {0};

#define EDGE_FALLING    (GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW)
#define EDGE_RISING	(GPIO_INT_EDGE | GPIO_INT_ACTIVE_HIGH)

static struct device *pinmux, *gpiob;
static struct gpio_callback gpio_cb;

struct sensor_value sensor_dump[256];

struct device *selected_device;

void measureOneCallback(struct device *gpiob, struct gpio_callback *cb,
		    u32_t pins)
{
	int ret, val;
	if(start == 0) {
		start = k_cycle_get_32();
	} else {
		stop = k_cycle_get_32();
	}
	ret = gpio_pin_read(gpiob, 5, &val);
	if(val == 1) {
		ret = gpio_pin_configure(gpiob, 5, GPIO_DIR_IN | GPIO_INT | EDGE_FALLING);
		if(ret < 0) {
			printk("error setting falling edge for IO2\n");
			return;
		}
	} else if(val == 0) {
		//Calculate pulse width
		distance_cm = (SYS_CLOCK_HW_CYCLES_TO_NS(stop - start))/58000;  // Distance calculations in cm
		start = stop = 0;
		printk("Distance is %ld",distance_cm);

		ret = gpio_pin_configure(gpiob, 5, GPIO_DIR_IN | GPIO_INT | EDGE_RISING);
		if(ret < 0) {
			printk("error setting rising edge for IO2\n");
			return;
		}

	}
	++t_count;
}


static int initGPIO() {
	pinmux = device_get_binding(CONFIG_PINMUX_NAME);
	struct galileo_data *dev = pinmux->driver_data;

	gpiob = dev->gpio_dw; 	//retrieving gpio_dw driver struct from pinmux driver
				// Alternatively, gpiob = device_get_binding(PORT);
	if (!gpiob) {
		printk("error\n");
		return -1;
	}

	int ret;

	ret = pinmux_pin_set(pinmux,1,PINMUX_FUNC_A); 	//IO1 or zephyr GPIO4 as output
	if(ret < 0)
		printk("error setting pin for IO1\n");

	ret = pinmux_pin_set(pinmux,2,PINMUX_FUNC_B); 	//IO2 or zephyr GPIO5 as input
	if(ret < 0)
		printk("error setting pin for IO2\n");
	
	//Configure interrupt
	ret = gpio_pin_configure(gpiob, 5, GPIO_DIR_IN | GPIO_INT | EDGE_RISING);
	if(ret < 0) {
		printk("error setting pin for IO2\n");
		return ret;
	}	
	gpio_init_callback(&gpio_cb, measureOneCallback, BIT(5));

	ret = gpio_add_callback(gpiob, &gpio_cb);
	if(ret < 0) {
		printk("error in gpio_add_callback %d\n", ret);
		return ret;
	}
	ret = gpio_pin_enable_callback(gpiob, 5);
	if(ret < 0) {
		printk("error in gpio_pin_enable_callback %d\n", ret);
		return ret;
	}
	printk("Init GPIO done\n");
	return 0;
}


static int select_u(const struct shell *shell, size_t argc,
			     char **argv) {
	int ret = initGPIO(true); //Set the GPIO config
	if(ret) {
		printk("Error initializing GPIO or PWM\n");
		return -1;
	}

	while(t_count < 100) {
 		ret = gpio_pin_write(gpiob, 4, 0);
  		ret = gpio_pin_write(gpiob, 4, 1);
 		k_sleep(10);
		ret = gpio_pin_write(gpiob, 4, 0);
	}
	ret = gpio_pin_write(gpiob, 4, 0);
	printk("Trigger sent\n");
	return 0;
}


static int select(const struct shell *shell, size_t argc,
			     char **argv) {
	int select_dev_num = atoi(argv[1]);
	if(!select_dev_num) {
	 	selected_device = device_get_binding("HCSR04_0");
	} else {
	 	selected_device = device_get_binding("HCSR04_1");			
	}
	printk("Selected device is %s\n", selected_device->config->name);
	return 0;
}

static int start_measure(const struct shell *shell, size_t argc,
			     char **argv) {
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int times = atoi(argv[1]);

	for(int i = 0; i < 256; ++i) {
		sensor_dump->val1 = 0;
	}
	printk("Sample fetch %d\n", times);
	elapsed_start = k_uptime_get();
	
	for(int i = 0; i < times; ++i) {
		if (sensor_sample_fetch(selected_device) < 0) {
			printf("Sensor sample update error\n");
			return -1;
		}
		//To prevent channel_get from writing 0 value to sensor_dump - probably a limitation of the sensor when the distance
		// is fluctuating rapidly.
		k_sleep(500);
		if (sensor_channel_get(selected_device, SENSOR_CHAN_ALL, &sensor_dump[i]) < 0) {
			printf("HCSR04 timeout\n");
			return -1;
		}
		printk("Channel get returned %d\n", sensor_dump[i].val1);
		elapsed_stop[i] = k_uptime_get();
	}
	return 0;
}

static int dump(const struct shell *shell, size_t argc,
			     char **argv) {
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	int p1 = atoi(argv[1]);
	int p2 = atoi(argv[2]);

	if(p2 >= p1) {
		for(int i = p1-1; i <= p2-1; ++i) {
			printk("Elapsed time %u:\tValue %d cms\n", elapsed_stop[i] - elapsed_start, sensor_dump[i].val1);
		}
	} else {
		printk("Arg 1 should be less than or equal to Arg2\n");
	}
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_cmd_measure,
	SHELL_CMD_ARG(select, NULL, "Select HCSR", select, 0, 0),
	SHELL_CMD_ARG(start, NULL, "Start distance measurement", start_measure, 0, 0),
	SHELL_CMD_ARG(dump, NULL, "Dump the collected samples", dump, 0, 0),
	SHELL_CMD_ARG(test, NULL, "Test HCSR", select_u, 0, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(hcsr, &sub_cmd_measure, "Start measurement", NULL);

void main(void)
{
	printk("Init done - %s\n", CONFIG_BOARD);
}
