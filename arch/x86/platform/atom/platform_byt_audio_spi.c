/*
 * platform_byt_audio.c: Baytrail audio platform data initilization file
 *
 * (C) Copyright 2013 Intel Corporation
 * Author: Omair Md Abudllah <omair.m.abdullah@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <asm/platform_sst_audio.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/platform_device.h>
#include <linux/sfi.h>
#include <linux/spi/spi.h>
#include <asm/intel-mid.h>

static struct regulator_consumer_supply dc1v8_consumers[] = {
	REGULATOR_SUPPLY("AVDD", "spi-WM510204:00"), /* wm5102 */
	REGULATOR_SUPPLY("DBVDD1", "spi-WM510204:00"), /* wm5102 */
	REGULATOR_SUPPLY("LDOVDD", "spi-WM510204:00"),

	REGULATOR_SUPPLY("DBVDD2", "wm5102-codec"),
	REGULATOR_SUPPLY("DBVDD3", "wm5102-codec"),
	REGULATOR_SUPPLY("CPVDD", "wm5102-codec"),
	REGULATOR_SUPPLY("SPKVDDL", "wm5102-codec"),
	REGULATOR_SUPPLY("SPKVDDR", "wm5102-codec"),
	REGULATOR_SUPPLY("CPVDD", "spi-WM510204:00"),
};

static struct regulator_init_data dc1v8_data = {
	.constraints = {
		.always_on = 1,
	},
	.num_consumer_supplies = ARRAY_SIZE(dc1v8_consumers),
	.consumer_supplies = dc1v8_consumers,
};

static struct fixed_voltage_config dc1v8vdd_pdata = {
	.supply_name = "DC_1V8",
	.microvolts = 1800000,
	.init_data = &dc1v8_data,
	.gpio = -1,
};

static struct platform_device dc1v8_device = {
	.name		= "reg-fixed-voltage",
	.id		= 0,
	.dev = {
		.platform_data = &dc1v8vdd_pdata,
	},
};

static int __init byt_audio_platform_init(void)
{
	struct platform_device *pdev;
	int ret;

	pr_info("in %s\n", __func__);

	platform_device_register(&dc1v8_device);

	return 0;
}
device_initcall(byt_audio_platform_init);
