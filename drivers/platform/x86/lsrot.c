/*
 *  Lenovo front-screen buttons testing driver
 *
 *  Copyright (C) 2010 Javier S. Pedro
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>

#include <linux/input.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/acpi.h>

#define DRIVER_NAME "lsrot"
#define DRIVER_AUTHOR "javispedro"
#define DRIVER_DESC "Lenovo front-screen buttons driver"

/* WMI event class */
#define WMI_ACPI_EVENT_GUID	"ABBC0F20-8EA1-11D1-00A0-C90629100000"
/* WMI block class (with set/get methods) */
#define WMI_ACPI_BLOCK_GUID	"ABBC0F40-8EA1-11D1-00A0-C90629100000"

/* Convert AP[0-7] into proper WSIO argument */
/* Example: to write to AP05, arg0 must be 8+5 = 13 */
#define AP_BIT(x) (8 + (x))

/* Offsets to interesting parts of the event "buffer" */
#define EVID(x)			((x)[1])
#define SKEY(x)			((x)[4])
#define TBMD(x)			((x)[0x17])
#define RTAG(x)			((x)[0x18])

/* Experimentation */
#define EVID_FRONT_KEY		0x19
#define EVID_HINGE			0x2C
#define EVID_ORIENTATION	0x2D

#define SKEY_TOUCH			0x86
#define SKEY_TOUCH_MAPPING	KEY_LEFTMETA
#define SKEY_ROTATE			0x90
#define SKEY_ROTATE_MAPPING 	KEY_F12

/* the input device */
static struct input_dev *input_dev;

static void report_key(int key)
{
	input_report_key(input_dev, key, 1);
	input_sync(input_dev);
	input_report_key(input_dev, key, 0);
	input_sync(input_dev);
}

static void handle_key_event(u8 *data)
{
	switch (SKEY(data)) {
		case SKEY_TOUCH:
			report_key(SKEY_TOUCH_MAPPING);
		break;
		case SKEY_ROTATE:
			report_key(SKEY_ROTATE_MAPPING);
		break;
		default:
			printk(KERN_WARNING "%s: unknown key code 0x%hx\n", DRIVER_NAME, SKEY(data));
		break;
	}
}

static void handle_hinge_event(u8 *data)
{
	bool tablet_mode = TBMD(data);
	input_report_switch(input_dev, SW_TABLET_MODE, tablet_mode);
	input_sync(input_dev);
}

static void handle_orientation_event(u8 *data)
{
	u8 orientation = RTAG(data);
	/* TODO : Understand RTAG values */
	printk(KERN_DEBUG "%s: new orientation: 0x%hx\n", DRIVER_NAME, orientation);
}

static int set_ap_bit(int ap, bool enabled)
{
	acpi_status status;
	static char buffer[0x80];
	struct acpi_buffer input = { sizeof(buffer), buffer };

	/* Actually, only bytes 0, 1, 8, 9, 10, 11 and 16 are read by DSDT,
	 * but let's play safe and clear them all. */
	memset(buffer, 0, sizeof(buffer));
	/* The DSDT expects all those magic numbers. It does nothing otherwise. */
	buffer[0] = 1;
	buffer[1] = 16;
	buffer[8] = AP_BIT(ap);
	buffer[10] = 0; /* Redundant: this really has to be zero, otherwise
	                   you're tricking the DSDT into firing a false event. */
	buffer[16] = enabled ? 1 : 0;

	printk(KERN_DEBUG "%s: setting ap bit %d to %d\n", DRIVER_NAME,
		ap, enabled ? 1 : 0);

	status = wmi_set_block(WMI_ACPI_BLOCK_GUID, 0, &input);
	if (ACPI_FAILURE(status)) {
		printk(KERN_WARNING "%s: failed to set ap bit\n", DRIVER_NAME);
		return -1;
	}

	return 0;
}

static void event_handler(u32 value, void *context)
{
	struct acpi_buffer response = { ACPI_ALLOCATE_BUFFER, NULL };
	union acpi_object *obj;
	acpi_status status;

	status = wmi_get_event_data(value, &response);
	if (ACPI_FAILURE(status)) {
		printk(KERN_WARNING "%s: bad event\n", DRIVER_NAME);
		return;
	}

	obj = (union acpi_object *)response.pointer;

	if (obj && obj->type == ACPI_TYPE_BUFFER && obj->buffer.length == 0x40) {
		u8 * data = (u8*) obj->buffer.pointer;
		switch (EVID(data)) {
			case EVID_FRONT_KEY:
				handle_key_event(data);
			break;
			case EVID_HINGE:
				handle_hinge_event(data);
			break;
			case EVID_ORIENTATION:
				handle_orientation_event(data);
			break;
			default:
				printk(KERN_WARNING "%s: unknown event code: %hx\n", DRIVER_NAME,
					data[1]);
			break;
		}
	} else {
		printk(KERN_WARNING "%s: bad event\n", DRIVER_NAME);
	}

	kfree(obj);
}

static int __init lsrot_init_module (void)
{
	acpi_status status;
	int err;

	if (!wmi_has_guid(WMI_ACPI_EVENT_GUID) || !wmi_has_guid(WMI_ACPI_BLOCK_GUID)) {
		printk(KERN_ERR "%s: missing required wmi guids\n",
		       DRIVER_NAME);
		err = -ENODEV;
		goto err_clean;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		goto err_clean;
	}

	input_dev->name = DRIVER_DESC;
	input_dev->id.bustype = BUS_HOST;

	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_SW, input_dev->evbit);
	set_bit(SKEY_TOUCH_MAPPING, input_dev->keybit);
	set_bit(SKEY_ROTATE_MAPPING, input_dev->keybit);
	set_bit(SW_TABLET_MODE, input_dev->swbit);

	err = input_register_device(input_dev);
	if (err) {
		goto err_clean;
	}

	/* AP05 is the only one that sets EC0.APLN */
	if (set_ap_bit(5, true)) {
		printk(KERN_ERR "%s: failed to set ap bit\n", DRIVER_NAME);
		err = -ENXIO;
		goto err_clean_input_dev;
	}

	status = wmi_install_notify_handler(WMI_ACPI_EVENT_GUID,
	                                    event_handler, NULL);

	if (ACPI_FAILURE(status)) {
		printk(KERN_ERR "%s: unable to register WMI event handler\n",
		       DRIVER_NAME);
		err = -ENODEV;
		goto err_clean_input_dev;
	}

	return 0;

err_clean_input_dev:
	input_free_device(input_dev);
err_clean:
	return err;
}

static void __exit lsrot_cleanup_module (void)
{
	wmi_remove_notify_handler(WMI_ACPI_EVENT_GUID);
	set_ap_bit(5, false);
	input_unregister_device(input_dev);
	input_free_device(input_dev);
}

module_init(lsrot_init_module);
module_exit(lsrot_cleanup_module);

MODULE_LICENSE("GPL");

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);

MODULE_ALIAS("wmi:" WMI_ACPI_EVENT_GUID);
MODULE_ALIAS("wmi:" WMI_ACPI_BLOCK_GUID);
