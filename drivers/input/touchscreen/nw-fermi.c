/*
 * Nextwindow Fermi touchscreen driver - Version: 0.6.5.0
 *
 * Copyright (C) 2001-2004 Greg Kroah-Hartman (greg@kroah.com)
 * Copyright (C) 2009-2011 Daniel Newton (djpnewton@gmail.com)
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * This driver is based on the Linux USB Skeleton driver.
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kref.h>
#include <asm/uaccess.h>
#include <linux/usb.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/kfifo.h>
#include <linux/vmalloc.h>
#include <linux/version.h>

/* HID mouse defines */
#define NW1950_MIN_X 0
#define NW1950_MAX_X 32767
#define NW1950_MIN_Y 0
#define NW1950_MAX_Y 32767
#define NW1950_MIN_W 0
#define NW1950_MAX_W 32767
#define NW1950_DEFAULT_W 1000
#define NW1950_MIN_H 0
#define NW1950_MAX_H 32767
#define NW1950_DEFAULT_H 1000

/* table of devices that work with this driver */
static struct usb_device_id fermi_table [] = {
	/* 1950 devices */
	{ USB_DEVICE(0x1926, 0x0006) },
	{ USB_DEVICE(0x1926, 0x007B) },
	{ USB_DEVICE(0x1926, 0x006A) },
	{ USB_DEVICE(0x1926, 0x006B) },
	{ USB_DEVICE(0x1926, 0x006C) },
	{ USB_DEVICE(0x1926, 0x006D) },
	{ USB_DEVICE(0x1926, 0x006E) },
	{ USB_DEVICE(0x1926, 0x006F) },
	{ USB_DEVICE(0x1926, 0x0070) },
	{ USB_DEVICE(0x1926, 0x007C) },
	{ USB_DEVICE(0x1926, 0x0071) },
	{ USB_DEVICE(0x1926, 0x0072) },
	{ USB_DEVICE(0x1926, 0x0073) },
	{ USB_DEVICE(0x1926, 0x0081) },
	{ USB_DEVICE(0x1926, 0x0082) },
	{ USB_DEVICE(0x1926, 0x0083) },
	{ USB_DEVICE(0x1926, 0x0084) },
	{ USB_DEVICE(0x1926, 0x0089) },
	{ USB_DEVICE(0x1926, 0x008A) },
	{ USB_DEVICE(0x1926, 0x008B) },
	{ USB_DEVICE(0x1926, 0x008C) },
	{ USB_DEVICE(0x1926, 0x008D) },
	{ USB_DEVICE(0x1926, 0x008E) },
	{ USB_DEVICE(0x1926, 0x0097) },
	{ USB_DEVICE(0x1926, 0x0098) },
	{ USB_DEVICE(0x1926, 0x0099) },
	{ USB_DEVICE(0x1926, 0x007D) },
	{ USB_DEVICE(0x1926, 0x0064) },
	{ USB_DEVICE(0x1926, 0x0065) },
	{ USB_DEVICE(0x1926, 0x0074) },
	{ USB_DEVICE(0x1926, 0x0066) },
	{ USB_DEVICE(0x1926, 0x0067) },
	{ USB_DEVICE(0x1926, 0x0068) },
	{ USB_DEVICE(0x1926, 0x0075) },
	{ USB_DEVICE(0x1926, 0x0069) },
	{ USB_DEVICE(0x1926, 0x0076) },
	{ USB_DEVICE(0x1926, 0x0077) },
	{ USB_DEVICE(0x1926, 0x0085) },
	{ USB_DEVICE(0x1926, 0x0078) },
	{ USB_DEVICE(0x1926, 0x0079) },
	{ USB_DEVICE(0x1926, 0x007A) },
	{ USB_DEVICE(0x1926, 0x007E) },
	{ USB_DEVICE(0x1926, 0x0086) },
	{ USB_DEVICE(0x1926, 0x0087) },
	{ USB_DEVICE(0x1926, 0x007F) },
	{ USB_DEVICE(0x1926, 0x0080) },
	{ USB_DEVICE(0x1926, 0x0088) },
	{ USB_DEVICE(0x1926, 0x008F) },
	{ USB_DEVICE(0x1926, 0x0090) },
	{ USB_DEVICE(0x1926, 0x0091) },
	{ USB_DEVICE(0x1926, 0x0092) },
	{ USB_DEVICE(0x1926, 0x0093) },
	{ USB_DEVICE(0x1926, 0x0094) },
	{ USB_DEVICE(0x1926, 0x0095) },
	{ USB_DEVICE(0x1926, 0x0096) },
	{ USB_DEVICE(0x1926, 0x009A) },
	/* Lister devices */
	{ USB_DEVICE(0x1926, 0x09C4) },
	{ USB_DEVICE(0x1926, 0x0BB8) },
	{ USB_DEVICE(0x1926, 0x0BB9) },
	{ USB_DEVICE(0x1926, 0x0BBA) },
	{ USB_DEVICE(0x1926, 0x0BBB) },
	{ USB_DEVICE(0x1926, 0x0BBC) },
	{ USB_DEVICE(0x1926, 0x0BBD) },
	{ USB_DEVICE(0x1926, 0x0BBE) },
	{ USB_DEVICE(0x1926, 0x0BBF) },
	{ USB_DEVICE(0x1926, 0x0BC0) },
	{ USB_DEVICE(0x1926, 0x0BC1) },
	{ USB_DEVICE(0x1926, 0x0BC2) },
	{ USB_DEVICE(0x1926, 0x0BC3) },
	{ USB_DEVICE(0x1926, 0x0BC4) },
	{ USB_DEVICE(0x1926, 0x0BC5) },
	{ USB_DEVICE(0x1926, 0x0BC6) },
	{ USB_DEVICE(0x1926, 0x0BC7) },
	{ USB_DEVICE(0x1926, 0x0BC8) },
	{ USB_DEVICE(0x1926, 0x0BC9) },
	{ USB_DEVICE(0x1926, 0x0BCA) },
	{ USB_DEVICE(0x1926, 0x0BCB) },
	{ USB_DEVICE(0x1926, 0x0BCC) },
	{ USB_DEVICE(0x1926, 0x0BCD) },
	{ USB_DEVICE(0x1926, 0x0BCE) },
	{ USB_DEVICE(0x1926, 0x0C1C) },
	{ USB_DEVICE(0x1926, 0x0C80) },
	{ USB_DEVICE(0x1926, 0x0C81) },
	{ USB_DEVICE(0x1926, 0x0C82) },
	{ USB_DEVICE(0x1926, 0x0C83) },
	{ USB_DEVICE(0x1926, 0x0C84) },
	{ USB_DEVICE(0x1926, 0x0C85) },
	{ USB_DEVICE(0x1926, 0x0BD2) },
	{ USB_DEVICE(0x1926, 0x0C87) },
	{ USB_DEVICE(0x1926, 0x0C88) },
	{ USB_DEVICE(0x1926, 0x0C89) },
	{ USB_DEVICE(0x1926, 0x0C8A) },
	{ USB_DEVICE(0x1926, 0x0C8B) },
	{ USB_DEVICE(0x1926, 0x0C8C) },
	{ USB_DEVICE(0x1926, 0x0D49) },
	{ USB_DEVICE(0x1926, 0x0D4A) },
	{ USB_DEVICE(0x1926, 0x0DAC) },
	{ USB_DEVICE(0x1926, 0x0DAD) },
	{ USB_DEVICE(0x1926, 0x0DAE) },
	{ USB_DEVICE(0x1926, 0x0DAF) },
	{ USB_DEVICE(0x1926, 0x0DB0) },
	{ USB_DEVICE(0x1926, 0x0DB1) },
	{ USB_DEVICE(0x1926, 0x0DB2) },
	{ USB_DEVICE(0x1926, 0x0DB3) },
	{ USB_DEVICE(0x1926, 0x0DB4) },
	{ USB_DEVICE(0x1926, 0x0DB5) },
	{ USB_DEVICE(0x1926, 0x0DB6) },
	{ USB_DEVICE(0x1926, 0x0DB7) },
	{ USB_DEVICE(0x1926, 0x0DBC) },
	{ USB_DEVICE(0x1926, 0x0DBD) },
	{ USB_DEVICE(0x1926, 0x0DBE) },
	{ USB_DEVICE(0x1926, 0x0DBF) },
	{ USB_DEVICE(0x1926, 0x0DC2) },
	{ USB_DEVICE(0x1926, 0x0E11) },
	{ USB_DEVICE(0x1926, 0x0E12) },
	{ USB_DEVICE(0x1926, 0x0E13) },
	{ USB_DEVICE(0x1926, 0x0E14) },
	{ USB_DEVICE(0x1926, 0x0E15) },
	{ USB_DEVICE(0x1926, 0x0E16) },
	{ USB_DEVICE(0x1926, 0x0E17) },
	{ USB_DEVICE(0x1926, 0x0ED9) },
	{ USB_DEVICE(0x1926, 0x0F3D) },
	{ USB_DEVICE(0x1926, 0x0F3E) },
	{ USB_DEVICE(0x1926, 0x183E) },
	{ USB_DEVICE(0x1926, 0x183F) },
	{ USB_DEVICE(0x1926, 0x1840) },
	{ USB_DEVICE(0x1926, 0x1843) },
	{ USB_DEVICE(0x1926, 0x1847) },
	/* Podium devices */
	{ USB_DEVICE(0x0b8c, 0x0069) },
	/* Holly devices */
	{ USB_DEVICE(0x1926, 0x0009) },
	{ }					/* Terminating entry */
};
MODULE_DEVICE_TABLE(usb, fermi_table);


/* Get a minor range for your devices from the usb maintainer */
#define USB_SKEL_MINOR_BASE	192

/* our private defines. if this grows any larger, use your own .h file */
#define MAX_TRANSFER		(PAGE_SIZE - 512)
/* MAX_TRANSFER is chosen so that the VM is not stressed by
   allocations > PAGE_SIZE and the number of packets in a page
   is an integer 512 is the largest possible packet on EHCI */
#define WRITES_IN_FLIGHT	8
/* arbitrarily chosen */

#define BULK_TRANSFER_SIZE 0x1000

#define BULK_FIFO_SIZE 0x10000

/* Structure to hold all of our device specific stuff */
struct usb_fermi {
	struct usb_device	*udev;			/* the usb device for this device */
	struct usb_interface	*interface;		/* the interface for this device */
	struct semaphore	limit_sem;		/* limiting the number of writes in progress */
	struct usb_anchor	submitted;		/* in case we need to retract our submissions */
	struct kfifo		bulk_fifo;		/* the buffer to receive data */
	void*			bulk_interim_buf;	/* interim buf between bulk_fifo and copy_to_user */
	__u8			bulk_in_endpointAddr;	/* the address of the bulk in endpoint */
	int			errors;			/* the last request tanked */
	int			open_count;		/* count the number of openers */
	spinlock_t		err_lock;		/* lock for errors */
	spinlock_t		bulk_lock;		/* lock for bulk fifo */
	struct kref		kref;
	struct mutex		io_mutex;		/* synchronize I/O with disconnect */
	struct input_dev*	input_dev;		/* the input device (to control cursor) */
	bool			input_dev_registered;	/* did we manage to register the input device */
	struct completion       bulk_in_completion;     /* to wait for bulk usb data */
};
#define to_fermi_dev(d) container_of(d, struct usb_fermi, kref)

static struct usb_driver fermi_driver;
static void fermi_draw_down(struct usb_fermi *dev);

static void fermi_delete(struct kref *kref)
{
	struct usb_fermi *dev = to_fermi_dev(kref);

	usb_put_dev(dev->udev);
	if (dev->input_dev) {
		if (dev->input_dev_registered)
			input_unregister_device(dev->input_dev);
		else
			input_free_device(dev->input_dev);
	}
	kfifo_free(&dev->bulk_fifo);
	if (dev->bulk_interim_buf)
		vfree(dev->bulk_interim_buf);
	kfree(dev);
}

static int fermi_open(struct inode *inode, struct file *file)
{
	struct usb_fermi *dev;
	struct usb_interface *interface;
	int subminor;
	int retval = 0;

	subminor = iminor(inode);

	interface = usb_find_interface(&fermi_driver, subminor);
	if (!interface) {
		//err ("%s - error, can't find device for minor %d",
		//     __func__, subminor);
		retval = -ENODEV;
		goto exit;
	}

	dev = usb_get_intfdata(interface);
	if (!dev) {
		retval = -ENODEV;
		goto exit;
	}

	/* increment our usage count for the device */
	kref_get(&dev->kref);

	/* lock the device to allow correctly handling errors
	 * in resumption */
	mutex_lock(&dev->io_mutex);

	if (!dev->open_count++) {
		retval = usb_autopm_get_interface(interface);
			if (retval) {
				dev->open_count--;
				mutex_unlock(&dev->io_mutex);
				kref_put(&dev->kref, fermi_delete);
				goto exit;
			}
	} /* else { //uncomment this block if you want exclusive open
		retval = -EBUSY;
		dev->open_count--;
		mutex_unlock(&dev->io_mutex);
		kref_put(&dev->kref, fermi_delete);
		goto exit;
	} */
	/* prevent the device from being autosuspended */

	/* save our object in the file's private structure */
	file->private_data = dev;
	mutex_unlock(&dev->io_mutex);

exit:
	return retval;
}

static int fermi_release(struct inode *inode, struct file *file)
{
	struct usb_fermi *dev;

	dev = (struct usb_fermi *)file->private_data;
	if (dev == NULL)
		return -ENODEV;

	/* allow the device to be autosuspended */
	mutex_lock(&dev->io_mutex);
	if (!--dev->open_count && dev->interface)
		usb_autopm_put_interface(dev->interface);
	mutex_unlock(&dev->io_mutex);

	/* decrement the count on our device */
	kref_put(&dev->kref, fermi_delete);
	return 0;
}

static int fermi_flush(struct file *file, fl_owner_t id)
{
	struct usb_fermi *dev;
	int res;

	dev = (struct usb_fermi *)file->private_data;
	if (dev == NULL)
		return -ENODEV;

	/* wait for io to stop */
	mutex_lock(&dev->io_mutex);
	fermi_draw_down(dev);

	/* read out errors, leave subsequent opens a clean slate */
	spin_lock_irq(&dev->err_lock);
	res = dev->errors ? (dev->errors == -EPIPE ? -EPIPE : -EIO) : 0;
	dev->errors = 0;
	spin_unlock_irq(&dev->err_lock);

	mutex_unlock(&dev->io_mutex);

	return res;
}

static ssize_t fermi_read(struct file *file, char *buffer, size_t count, loff_t *ppos)
{
	struct usb_fermi *dev;
	int retval = 0;

	dev = (struct usb_fermi *)file->private_data;

	while (retval == 0) {
		mutex_lock(&dev->io_mutex);
		if (!dev->interface)		/* disconnect() was called */
			retval = -ENODEV;
		else {
			/* read from the fifo to an interim buffer */
			retval = kfifo_out_locked(&dev->bulk_fifo,
					(unsigned char *)dev->bulk_interim_buf,
					min(count, (size_t)BULK_FIFO_SIZE),
					&dev->bulk_lock);

			if (retval > 0) {
				/* read from the interim buffer to user buffer */
				if (copy_to_user(buffer, dev->bulk_interim_buf, retval)) {
					retval = -EFAULT;
					printk("copy_to_user err!\n");
				}
			} else if (retval == 0) {
				mutex_unlock(&dev->io_mutex);

				/*
				 * IO may take forever
				 * hence wait in an interruptible state
				 */
				init_completion(&dev->bulk_in_completion);
				retval = wait_for_completion_interruptible(&dev->bulk_in_completion);

				continue;
			}
		}

		mutex_unlock(&dev->io_mutex);
	}

	//printk("fermi_read return code: %d\n", retval);
	return retval;
}

#include "nwfermi_public.h"

static void fermi_input_event(struct usb_fermi *dev, struct fermi_touch_report_t *touch_report)
{
	int i;
	int count = min(touch_report->count, (unsigned char)2);
	// multitouch
	for (i = 0; i < count; i++) {
		input_report_abs(dev->input_dev, ABS_MT_TRACKING_ID, touch_report->touch[i].id);
		input_report_abs(dev->input_dev, ABS_MT_POSITION_X, touch_report->touch[i].x);
		input_report_abs(dev->input_dev, ABS_MT_POSITION_Y, touch_report->touch[i].y);
		// we just set width and height to 1 now for android 2.x as it seems to need it
		input_report_abs(dev->input_dev, ABS_MT_TOUCH_MAJOR, NW1950_DEFAULT_W);
		input_report_abs(dev->input_dev, ABS_MT_TOUCH_MINOR, NW1950_DEFAULT_H);
		input_mt_sync(dev->input_dev);
	}
	// mouse
	if (touch_report->touch[0].state == FERMI_TOUCH_DOWN ||
			touch_report->touch[0].state == FERMI_TOUCH ||
			touch_report->touch[0].state == FERMI_TOUCH_UP ||
			touch_report->touch[0].state == FERMI_TOUCH_HOVER) {
		input_report_key(dev->input_dev, BTN_LEFT, touch_report->touch[0].state == FERMI_TOUCH_UP ? 0 : 1);
		input_report_abs(dev->input_dev, ABS_X, touch_report->touch[0].x);
		input_report_abs(dev->input_dev, ABS_Y, touch_report->touch[0].y);
	}
	// sync
	input_sync(dev->input_dev);
	//printk("fermi_write BTN_LEFT: %d, BTN_RIGHT: %d, ABS_X: %d, ABS_Y: %d\n", button & 1, button & 2, x , y);
}

static ssize_t fermi_write(struct file *file, const char *user_buffer,
			  size_t count, loff_t *ppos)
{
	struct usb_fermi *dev;
	char *buf;
	ssize_t retval = 0;

	dev = (struct usb_fermi *)file->private_data;

	/**@todo do we need to sync this using a mutex lock ? */

	/* get the user data */
	buf = kzalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	if (copy_from_user(buf, user_buffer, count)) {
		retval = -EFAULT;
		goto end;
	}

	/* fire data to input subsystem */
	if (count >= sizeof(struct fermi_touch_report_t))
		fermi_input_event(dev, (struct fermi_touch_report_t*)buf);

	retval = count;

end:
	if (buf)
		kfree(buf);
	return retval;
}

static const struct file_operations fermi_fops = {
	.owner =	THIS_MODULE,
	.read =		fermi_read,
	.write =	fermi_write,
	.open =		fermi_open,
	.release =	fermi_release,
	.flush =	fermi_flush,
};

/*
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with the driver core
 */
static struct usb_class_driver fermi_class = {
	.name =		"nwfermi%d",
	.fops =		&fermi_fops,
	.minor_base =	USB_SKEL_MINOR_BASE,
};

static void fermi_bulk_read_complete(struct urb* urb)
{
	struct usb_fermi* dev = (struct usb_fermi*)urb->context;
	int retval = 0;

	if (urb->status) {
		// causes "inplicit declaration" -> dbg("URB Status: %d\n", urb->status);
		switch(urb->status) {
		/* device gone, unplugged or unlinked */
		case -ECONNRESET:
		case -ENODEV:
		case -ENOENT:
		case -ESHUTDOWN:
			retval = -ENODEV;
			break;
		/* errors that might occur during unplugging */
		case -EILSEQ:
		case -EPROTO:
		case -ETIME:
			retval = -EIO;
			break;
		default:
			//err("Error on read completion routine. Code: %d", urb->status);
			retval = -EFAULT;
		}
	} else {
		if (!dev->interface)		/* disconnect() was called */
			retval = -ENODEV;
		else {
			/* copy data to bulk fifo */
			kfifo_in_locked(&dev->bulk_fifo, urb->transfer_buffer, urb->actual_length,
					&dev->bulk_lock);

			/* wake up the fops read thread */
			complete(&dev->bulk_in_completion);

			/* re-initialize the urb */
			usb_fill_bulk_urb(urb, dev->udev,
					usb_rcvbulkpipe(dev->udev, dev->bulk_in_endpointAddr),
					urb->transfer_buffer, urb->transfer_buffer_length,
					fermi_bulk_read_complete, dev);
			urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

			/* resumbit the urb */
			retval = usb_submit_urb(urb, GFP_ATOMIC);
		}
	}

	if (retval) {
		/* free resources */
		usb_free_coherent(dev->udev, urb->transfer_buffer_length, urb->transfer_buffer, urb->transfer_dma);
		usb_free_urb(urb);
	}
}

static int fermi_start_bulk_reads(struct usb_fermi* dev)
{
	int retval = -ENOMEM;
	struct urb* urb=NULL;
	char* buf = NULL;

	/* create an URB */
	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb)
		goto error;

	/* allocate transfer buffer */
	buf = usb_alloc_coherent(dev->udev, BULK_TRANSFER_SIZE, GFP_KERNEL, &urb->transfer_dma);
	if (!buf)
		goto error;

	/* initialize the urb */
	usb_fill_bulk_urb(urb, dev->udev,
			  usb_rcvbulkpipe(dev->udev, dev->bulk_in_endpointAddr),
			  buf, BULK_TRANSFER_SIZE, fermi_bulk_read_complete, dev);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	//usb_anchor_urb(urb, &dev->submitted);

	/* send the data out the bulk port */
	retval = usb_submit_urb(urb, GFP_KERNEL);

error:
	if (retval) {
		if (buf)
			usb_free_coherent(dev->udev, BULK_TRANSFER_SIZE, buf, urb->transfer_dma);
		if (urb)
			usb_free_urb(urb);
	}
	return retval;
}

static int fermi_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
	struct usb_fermi *dev;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	int i;
	int retval = 0;

	/* allocate memory for our device state and initialize it */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		//err("Out of memory");
		retval = -ENOMEM;
		goto error;
	}
	kref_init(&dev->kref);
	sema_init(&dev->limit_sem, WRITES_IN_FLIGHT);
	mutex_init(&dev->io_mutex);
	spin_lock_init(&dev->err_lock);
	spin_lock_init(&dev->bulk_lock);
	init_usb_anchor(&dev->submitted);
	init_completion(&dev->bulk_in_completion);

	dev->udev = usb_get_dev(interface_to_usbdev(interface));
	dev->interface = interface;

	/* set up the endpoint information */
	/* use only the first bulk-in and bulk-out endpoints */
	iface_desc = interface->cur_altsetting;
	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		endpoint = &iface_desc->endpoint[i].desc;

		if (!dev->bulk_in_endpointAddr && usb_endpoint_is_bulk_in(endpoint)) {
			/* we found a bulk in endpoint */
			dev->bulk_in_endpointAddr = endpoint->bEndpointAddress;
		}

	}
	if (!(dev->bulk_in_endpointAddr)) {
		//err("Could not find bulk-in endpoint");
		goto error;
	}

	/* initialize our fifo */
	retval = kfifo_alloc(&dev->bulk_fifo, BULK_FIFO_SIZE, GFP_KERNEL);
	if (retval) {
		//err("Could not allocate bulk_fifo");
		goto error;
	}
	dev->bulk_interim_buf = vmalloc(BULK_FIFO_SIZE);
	if (!dev->bulk_interim_buf) {
		//err("Could not allocate bulk_interim_buf");
		goto error;
	}

	/* initialize our input device */
	dev->input_dev = input_allocate_device();
	if (!dev->input_dev) {
		//err("Could not allocate input device");
		retval = -ENOMEM;
		goto error;
	}
	dev->input_dev->name = "Nextwindow Fermi Touchscreen";
	dev->input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	dev->input_dev->keybit[BIT_WORD(BTN_MOUSE)] = BIT_MASK(BTN_LEFT) | BIT_MASK(BTN_RIGHT) | BIT_MASK(BTN_TOOL_FINGER);
	input_set_abs_params(dev->input_dev, ABS_X, NW1950_MIN_X, NW1950_MAX_X, 0, 0);
	input_set_abs_params(dev->input_dev, ABS_Y, NW1950_MIN_Y, NW1950_MAX_Y, 0, 0);
	input_set_abs_params(dev->input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);
	input_set_abs_params(dev->input_dev, ABS_MT_POSITION_X, NW1950_MIN_X, NW1950_MAX_X, 0, 0);
	input_set_abs_params(dev->input_dev, ABS_MT_POSITION_Y, NW1950_MIN_Y, NW1950_MAX_Y, 0, 0);
	input_set_abs_params(dev->input_dev, ABS_MT_TOUCH_MAJOR, NW1950_MIN_W, NW1950_MAX_W, 0, 0);
	input_set_abs_params(dev->input_dev, ABS_MT_TOUCH_MINOR, NW1950_MIN_H, NW1950_MAX_H, 0, 0);
	retval = input_register_device(dev->input_dev);
	if (retval) {
		//err("Could not register input device");
		dev->input_dev_registered = false;
		goto error;
	}
	dev->input_dev_registered = true;

	/* save our data pointer in this interface device */
	usb_set_intfdata(interface, dev);

	/* we can register the device now, as it is ready */
	retval = usb_register_dev(interface, &fermi_class);
	if (retval) {
		/* something prevented us from registering this driver */
		//err("Not able to get a minor for this device.");
		usb_set_intfdata(interface, NULL);
		goto error;
	}

	/* let the user know what node this device is now attached to */
	dev_info(&interface->dev,
		 "NextWindow Fermi device now attached to nwfermi-%d\n",
		 interface->minor);

	/* start our bulk reader */
	fermi_start_bulk_reads(dev);

	return 0;

error:
	if (dev) {
		/* this frees allocated memory */
		kref_put(&dev->kref, fermi_delete);
	}
	return retval;
}

static void fermi_disconnect(struct usb_interface *interface)
{
	struct usb_fermi *dev;
	int minor = interface->minor;

	/* the minor number should be equal to or greater than 0 */
	if (minor < 0)
		return;

	dev = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);

	/* prevent more I/O from starting */
	mutex_lock(&dev->io_mutex);
	dev->interface = NULL;
	mutex_unlock(&dev->io_mutex);

	/* give back our minor */
	usb_deregister_dev(interface, &fermi_class);

	usb_kill_anchored_urbs(&dev->submitted);

	/* kill any current waiting IO */
	complete(&dev->bulk_in_completion);

	/* decrement our usage count */
	kref_put(&dev->kref, fermi_delete);

	dev_info(&interface->dev, "nwfermi #%d now disconnected", minor);
}

static void fermi_draw_down(struct usb_fermi *dev)
{
	int time;

	time = usb_wait_anchor_empty_timeout(&dev->submitted, 1000);
	if (!time)
		usb_kill_anchored_urbs(&dev->submitted);
}

static int fermi_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct usb_fermi *dev = usb_get_intfdata(intf);

	if (dev)
		fermi_draw_down(dev);

	return 0;
}

static int fermi_resume(struct usb_interface *intf)
{
	return 0;
}

static int fermi_pre_reset(struct usb_interface *intf)
{
	struct usb_fermi *dev = usb_get_intfdata(intf);

	mutex_lock(&dev->io_mutex);
	fermi_draw_down(dev);

	return 0;
}

static int fermi_post_reset(struct usb_interface *intf)
{
	struct usb_fermi *dev = usb_get_intfdata(intf);

	/* we are sure no URBs are active - no locking needed */
	dev->errors = -EPIPE;
	mutex_unlock(&dev->io_mutex);

	return 0;
}

static struct usb_driver fermi_driver = {
	.name =		"nwfermi",
	.probe =	fermi_probe,
	.disconnect =	fermi_disconnect,
	.suspend =	fermi_suspend,
	.resume =	fermi_resume,
	.pre_reset =	fermi_pre_reset,
	.post_reset =	fermi_post_reset,
	.id_table =	fermi_table,
	.supports_autosuspend = 1,
};

static int __init usb_fermi_init(void)
{
	int result;

	/* register this driver with the USB subsystem */
	result = usb_register(&fermi_driver);
	//if (result)
		//err("usb_register failed. Error number %d", result);

	return result;
}

static void __exit usb_fermi_exit(void)
{
	/* deregister this driver with the USB subsystem */
	usb_deregister(&fermi_driver);
}

module_init(usb_fermi_init);
module_exit(usb_fermi_exit);

MODULE_LICENSE("GPL");
