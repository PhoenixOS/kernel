/*
 * Tablet support (power button, docking events) for Dell Venue 11 Pro
 *
 * (C) Copyright 2015,2016 Jan-Michael Brummer
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/acpi.h>
#include <acpi/button.h>
#include "../../acpi/acpica/aclocal.h"
#include "../../acpi/acpica/acevents.h"

#define DV11P_TABLET_HID			"INT33D6"
#define DV11P_TABLET_DEVICE_NAME		"Dell Venue 11 Pro Tablet"

#define DV11P_TABLET_NOTIFY_PRESS_POWER		0xc1
#define DV11P_TABLET_NOTIFY_RELEASE_POWER	0xc0

#define DV11P_TABLET_NOTIFY_DOCK_MODE_ON	0xca
#define DV11P_TABLET_NOTIFY_DOCK_MODE_OFF	0xcb

#define DV11P_TABLET_NOTIFY_TABLET_MODE_ON	0xcc
#define DV11P_TABLET_NOTIFY_TABLET_MODE_OFF	0xcd

#define DV11P_TABLET_PBTN_GPE_NUMBER 		0x08


ACPI_MODULE_NAME("dell venue 11 pro tablet support");

MODULE_AUTHOR("Jan-Michael Brummer");
MODULE_DESCRIPTION("Dell Venue 11 Pro Tablet Driver");
MODULE_LICENSE("GPL v2");

/*
 * Power button and events are handled by EC
 */
static const struct acpi_device_id dv11p_tablet_device_ids[] = {
	{DV11P_TABLET_HID,    0},
	{"", 0},
};

MODULE_DEVICE_TABLE(acpi, dv11p_tablet_device_ids);

struct dv11p_tablet {
	unsigned int type;
	struct input_dev *input;
	char phys[32];			/* for input device */
	unsigned long pushed;
	bool suspended;
	bool ignore_next_pbtn_event;
};

static void dv11p_tablet_notify(struct acpi_device *device, u32 event)
{
	struct dv11p_tablet *tablet = acpi_driver_data(device);
	struct input_dev *input;
	int key_code = KEY_RESERVED;
	bool pressed = false;
	bool dock_mode = false;
	bool dock_mode_set = false;
	bool tablet_mode = false;
	bool tablet_mode_set = false;

	switch (event) {
	/* Power button press, release handle */
	case DV11P_TABLET_NOTIFY_PRESS_POWER:
		pressed = true;
		/* fall through */
	case DV11P_TABLET_NOTIFY_RELEASE_POWER:
		key_code = KEY_POWER;
		break;
	/* Tablet dock mode on / off */
	case DV11P_TABLET_NOTIFY_DOCK_MODE_ON:
		dock_mode = true;
		/* fall through */
	case DV11P_TABLET_NOTIFY_DOCK_MODE_OFF:
		dock_mode_set = true;
		break;
	/* Tablet mode on / off */
	case DV11P_TABLET_NOTIFY_TABLET_MODE_ON:
		tablet_mode = true;
		/* fall through */
	case DV11P_TABLET_NOTIFY_TABLET_MODE_OFF:
		tablet_mode_set = true;
		break;
	default:
		dev_info_ratelimited(&device->dev,
				  "Unsupported event [0x%x]\n", event);
		break;
	}

	if ((KEY_RESERVED == key_code) && !dock_mode_set && !tablet_mode_set)
		return;
	if (pressed)
		pm_wakeup_event(&device->dev, 0);
	if (tablet->suspended) {
		if(key_code == KEY_POWER) {
			tablet->ignore_next_pbtn_event = true;
		}
		return;
	}
	if (tablet->ignore_next_pbtn_event) {
		tablet->ignore_next_pbtn_event = false;
		return;
	}

	input = tablet->input;

	if (key_code != KEY_RESERVED) {
		/* Report button press event */
		input_report_key(input, key_code, pressed);
	} else if (dock_mode_set) {
		/* Report un/dock mode */
		input_report_switch(input, SW_DOCK, dock_mode);
	} else if (tablet_mode_set) {
		/* Report tablet mode */
		input_report_switch(input, SW_TABLET_MODE, tablet_mode);
	}

	input_sync(input);
}

#ifdef CONFIG_PM_SLEEP
static acpi_status
acpi_hw_correct_pbtn_gpe_wake_mask(struct acpi_gpe_xrupt_info *gpe_xrupt_info,
				struct acpi_gpe_block_info *gpe_block,
				void *context)
{
	u32 i;
	struct acpi_gpe_register_info *gpe_register_info;

	/* Examine each GPE Register within the block */

	for (i = 0; i < gpe_block->register_count; i++) {
		gpe_register_info = &gpe_block->register_info[i];

		if (gpe_register_info->base_gpe_number == DV11P_TABLET_PBTN_GPE_NUMBER) {
			gpe_register_info->enable_for_wake = gpe_register_info->enable_for_run;
			return AE_OK;
		}
	}
	return AE_NOT_FOUND;
}

static int dv11p_tablet_suspend(struct device *dev)
{
	struct acpi_device *device = to_acpi_device(dev);
	struct dv11p_tablet *tablet = acpi_driver_data(device);

	// Set the mask for the GPE associated with the power button to the
	// value of the run mask, otherwise resuming is not possible with the
	// power button.
	acpi_ev_walk_gpe_list(acpi_hw_correct_pbtn_gpe_wake_mask, NULL);

	tablet->suspended = true;
	return 0;
}

static int dv11p_tablet_resume(struct device *dev)
{
	struct acpi_device *device = to_acpi_device(dev);
	struct dv11p_tablet *tablet = acpi_driver_data(device);

	tablet->suspended = false;
	return 0;
}
#endif

static int dv11p_tablet_add(struct acpi_device *device)
{
	struct dv11p_tablet *tablet;
	struct input_dev *input;
	const char *hid = acpi_device_hid(device);
	char *name;
	int error;

	tablet = kzalloc(sizeof(struct dv11p_tablet), GFP_KERNEL);
	if (!tablet)
		return -ENOMEM;

	device->driver_data = tablet;

	tablet->input = input = input_allocate_device();
	if (!input) {
		error = -ENOMEM;
		goto err_free_tablet;
	}

	name = acpi_device_name(device);
	strcpy(name, DV11P_TABLET_DEVICE_NAME);
	snprintf(tablet->phys, sizeof(tablet->phys), "%s/buttons", hid);

	input->name = name;
	input->phys = tablet->phys;
	input->id.bustype = BUS_HOST;
	input->dev.parent = &device->dev;
	input_set_capability(input, EV_KEY, KEY_POWER);
	input_set_capability(input, EV_SW, SW_DOCK);
	input_set_capability(input, EV_SW, SW_TABLET_MODE);

	error = input_register_device(input);
	if (error)
		goto err_free_input;

	dev_info(&device->dev,
			"%s [%s]\n", name, acpi_device_bid(device));
	return 0;

 err_free_input:
	input_free_device(input);
 err_free_tablet:
	kfree(tablet);
	return error;
}

static int dv11p_tablet_remove(struct acpi_device *device)
{
	struct dv11p_tablet *tablet = acpi_driver_data(device);

	input_unregister_device(tablet->input);
	kfree(tablet);
	return 0;
}

static SIMPLE_DEV_PM_OPS(dv11p_tablet_pm,
		dv11p_tablet_suspend, dv11p_tablet_resume);

static struct acpi_driver dv11p_tablet_driver = {
	.name = "dell venue 11 pro tablet",
	.class = "DellVenue11Pro",
	.ids = dv11p_tablet_device_ids,
	.ops = {
		.add = dv11p_tablet_add,
		.remove = dv11p_tablet_remove,
		.notify = dv11p_tablet_notify,
	},
	.drv.pm = &dv11p_tablet_pm,
};

module_acpi_driver(dv11p_tablet_driver);
