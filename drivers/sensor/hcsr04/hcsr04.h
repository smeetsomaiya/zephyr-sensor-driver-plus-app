/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_HDC1008_HCSR04_H_
#define ZEPHYR_DRIVERS_SENSOR_HDC1008_HCSR04_H_

#include <kernel.h>

#define HCSR04_I2C_ADDRESS	0x40

#define HCSR04_REG_TEMP	0x0
#define HCSR04_REG_HUMIDITY	0x1
#define HCSR04_REG_MANUFID	0xFE
#define HCSR04_REG_DEVICEID	0xFF

#define HCSR04_MANUFID		0x5449
#define HCSR04_DEVICEID	0x1000

struct isr_data {
	int start_time;
	int stop_time;
	int readVal;
	float distance_cm;
};

struct hcsr04_data {
	struct device *gpio;
	struct gpio_callback gpio_cb;
	u16_t trigger_pin;
	u16_t echo_pin;
	u16_t t_sample;
	u16_t rh_sample;
	struct k_sem data_sem;
	struct isr_data *cb_data;
	u8_t status;
	u16_t timeout;
};


#endif
