/*
 *  intel_hdmi_lpe_audio.c - Intel HDMI LPE audio driver for Atom platforms
 *
 *  Copyright (C) 2016 Intel Corp
 *  Authors:
 *		Jerome Anand <jerome.anand@intel.com>
 *		Aravind Siddappaji <aravindx.siddappaji@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#define pr_fmt(fmt)	"hdmi_lpe_audio: " fmt

#include <linux/platform_device.h>
#include <linux/irqreturn.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <sound/pcm.h>
#include <sound/core.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/control.h>
#include <sound/initval.h>
#include <drm/intel_lpe_audio.h>
#include "intel_hdmi_lpe_audio.h"
#include "intel_hdmi_audio.h"

/* globals*/
struct platform_device *gpdev;
int _hdmi_state;
union otm_hdmi_eld_t hdmi_eld;

struct hdmi_lpe_audio_ctx {
	int irq;
	void __iomem *mmio_start;
	had_event_call_back had_event_callbacks;
	struct snd_intel_had_interface *had_interface;
	void *had_pvt_data;
	int tmds_clock_speed;
	unsigned int had_config_offset;
	int hdmi_audio_interrupt_mask;
	struct work_struct hdmi_audio_wq;
};

static inline void hdmi_set_eld(void *eld)
{
	int size = (sizeof(hdmi_eld)) > HDMI_MAX_ELD_BYTES ?
				HDMI_MAX_ELD_BYTES :
				(sizeof(hdmi_eld));

	memcpy((void *)&hdmi_eld, eld, size);
}

static inline int hdmi_get_eld(void *eld)
{
	memcpy(eld, (void *)&hdmi_eld, sizeof(hdmi_eld));

	{
		int i;
		uint8_t *eld_data = (uint8_t *)&hdmi_eld;

		pr_debug("hdmi_get_eld:\n{{");

		for (i = 0; i < sizeof(hdmi_eld); i++)
			pr_debug("0x%x, ", eld_data[i]);

		pr_debug("}}\n");
	}
	return HAD_SUCCESS;
}


static inline struct hdmi_lpe_audio_ctx *get_hdmi_context(void)
{
	struct hdmi_lpe_audio_ctx *ctx;

	ctx = platform_get_drvdata(gpdev);
	return ctx;
}

/*
 * return whether HDMI audio device is busy.
 */
bool mid_hdmi_audio_is_busy(void *ddev)
{
	struct hdmi_lpe_audio_ctx *ctx;
	int hdmi_audio_busy = 0;
	struct hdmi_audio_event hdmi_audio_event;

	pr_debug("%s: Enter",  __func__);

	ctx = platform_get_drvdata(gpdev);

	if (_hdmi_state == hdmi_connector_status_disconnected) {
		/* HDMI is not connected, assuming audio device is idle. */
		return false;
	}

	if (ctx->had_interface) {
		hdmi_audio_event.type = HAD_EVENT_QUERY_IS_AUDIO_BUSY;
		hdmi_audio_busy = ctx->had_interface->query(
				ctx->had_pvt_data,
				hdmi_audio_event);
		return hdmi_audio_busy != 0;
	}
	return false;
}

/*
 * return whether HDMI audio device is suspended.
 */
bool mid_hdmi_audio_suspend(void *ddev)
{
	struct hdmi_lpe_audio_ctx *ctx;
	struct hdmi_audio_event hdmi_audio_event;
	int ret = 0;

	ctx = platform_get_drvdata(gpdev);

	if (_hdmi_state == hdmi_connector_status_disconnected) {
		/* HDMI is not connected, assuming audio device
		 * is suspended already.
		 */
		return true;
	}

	pr_debug("%s: _hdmi_state %d",  __func__,
			_hdmi_state);

	if (ctx->had_interface) {
		hdmi_audio_event.type = 0;
		ret = ctx->had_interface->suspend(ctx->had_pvt_data,
				hdmi_audio_event);
		return (ret == 0) ? true : false;
	}
	return true;
}

void mid_hdmi_audio_resume(void *ddev)
{
	struct hdmi_lpe_audio_ctx *ctx;

	ctx = platform_get_drvdata(gpdev);

	if (_hdmi_state == hdmi_connector_status_disconnected) {
		/* HDMI is not connected, there is no need
		 * to resume audio device.
		 */
		return;
	}

	pr_debug("%s: _hdmi_state %d",  __func__, _hdmi_state);

	if (ctx->had_interface)
		ctx->had_interface->resume(ctx->had_pvt_data);
}

void mid_hdmi_audio_signal_event(enum had_event_type event)
{
	struct hdmi_lpe_audio_ctx *ctx;

	pr_debug("%s: Enter\n", __func__);

	ctx = platform_get_drvdata(gpdev);

	if (ctx->had_event_callbacks)
		(*ctx->had_event_callbacks)(event,
			ctx->had_pvt_data);
}

/**
 * hdmi_audio_write:
 * used to write into display controller HDMI audio registers.
 *
 */
static int hdmi_audio_write(uint32_t reg, uint32_t val)
{
	struct hdmi_lpe_audio_ctx *ctx;

	ctx = platform_get_drvdata(gpdev);

	pr_debug("%s: reg[0x%x] = 0x%x\n", __func__, reg, val);

	iowrite32(val, (ctx->mmio_start+reg));

	return HAD_SUCCESS;
}

/**
 * hdmi_audio_read:
 * used to get the register value read from
 * display controller HDMI audio registers.
 */
static int hdmi_audio_read(uint32_t reg, uint32_t *val)
{
	struct hdmi_lpe_audio_ctx *ctx;

	ctx = platform_get_drvdata(gpdev);
	*val = ioread32(ctx->mmio_start+reg);
	pr_debug("%s: reg[0x%x] = 0x%x\n", __func__, reg, *val);
	return HAD_SUCCESS;
}

/**
 * hdmi_audio_rmw:
 * used to update the masked bits in display controller HDMI
 * audio registers.
 */
static int hdmi_audio_rmw(uint32_t reg, uint32_t val, uint32_t mask)
{
	struct hdmi_lpe_audio_ctx *ctx;
	uint32_t val_tmp = 0;

	ctx = platform_get_drvdata(gpdev);

	val_tmp = (val & mask) |
			((ioread32(ctx->mmio_start + reg)) & ~mask);

	iowrite32(val_tmp, (ctx->mmio_start+reg));
	pr_debug("%s: reg[0x%x] = 0x%x\n", __func__, reg, val_tmp);

	return HAD_SUCCESS;
}

/**
 * hdmi_audio_get_caps:
 * used to return the HDMI audio capabilities.
 * e.g. resolution, frame rate.
 */
static int hdmi_audio_get_caps(enum had_caps_list get_element,
			void *capabilities)
{
	struct hdmi_lpe_audio_ctx *ctx;
	int ret = HAD_SUCCESS;

	ctx = get_hdmi_context();

	pr_debug("%s: Enter\n", __func__);

	switch (get_element) {
	case HAD_GET_ELD:
		ret = hdmi_get_eld(capabilities);
		break;
	case HAD_GET_DISPLAY_RATE:
		/* ToDo: Verify if sampling freq logic is correct */
		memcpy(capabilities, &(ctx->tmds_clock_speed),
			sizeof(uint32_t));
		pr_debug("%s: tmds_clock_speed = 0x%x\n", __func__,
				ctx->tmds_clock_speed);
		break;
	default:
		break;
	}

	return ret;
}

/**
 * hdmi_audio_get_register_base
 * used to get the current hdmi base address
 */
int hdmi_audio_get_register_base(uint32_t **reg_base,
		uint32_t *config_offset)
{
	struct hdmi_lpe_audio_ctx *ctx;

	ctx = platform_get_drvdata(gpdev);
	*reg_base = (uint32_t *)(ctx->mmio_start);
	*config_offset = ctx->had_config_offset;
	pr_debug("%s: reg_base = 0x%p, cfg_off = 0x%x\n", __func__,
			*reg_base, *config_offset);
	return HAD_SUCCESS;
}

/**
 * hdmi_audio_set_caps:
 * used to set the HDMI audio capabilities.
 * e.g. Audio INT.
 */
int hdmi_audio_set_caps(enum had_caps_list set_element,
			void *capabilties)
{
	struct hdmi_lpe_audio_ctx *ctx;

	ctx = platform_get_drvdata(gpdev);

	pr_debug("%s: cap_id = 0x%x\n", __func__, set_element);

	switch (set_element) {
	case HAD_SET_ENABLE_AUDIO_INT:
		{
			uint32_t status_reg;

			hdmi_audio_read(AUD_HDMI_STATUS_v2 +
				ctx->had_config_offset, &status_reg);
			status_reg |=
				HDMI_AUDIO_BUFFER_DONE | HDMI_AUDIO_UNDERRUN;
			hdmi_audio_write(AUD_HDMI_STATUS_v2 +
				ctx->had_config_offset, status_reg);
			hdmi_audio_read(AUD_HDMI_STATUS_v2 +
				ctx->had_config_offset, &status_reg);

		}
		break;
	default:
		break;
	}

	return HAD_SUCCESS;
}

static struct  hdmi_audio_registers_ops hdmi_audio_reg_ops = {
	.hdmi_audio_get_register_base = hdmi_audio_get_register_base,
	.hdmi_audio_read_register = hdmi_audio_read,
	.hdmi_audio_write_register = hdmi_audio_write,
	.hdmi_audio_read_modify = hdmi_audio_rmw,
};

static struct hdmi_audio_query_set_ops hdmi_audio_get_set_ops = {
	.hdmi_audio_get_caps = hdmi_audio_get_caps,
	.hdmi_audio_set_caps = hdmi_audio_set_caps,
};

int mid_hdmi_audio_setup(
		had_event_call_back audio_callbacks,
		struct hdmi_audio_registers_ops *reg_ops,
		struct hdmi_audio_query_set_ops *query_ops)
{
	struct hdmi_lpe_audio_ctx *ctx;

	ctx = platform_get_drvdata(gpdev);

	pr_debug("%s: called\n",  __func__);

	reg_ops->hdmi_audio_get_register_base =
		(hdmi_audio_reg_ops.hdmi_audio_get_register_base);
	reg_ops->hdmi_audio_read_register =
		(hdmi_audio_reg_ops.hdmi_audio_read_register);
	reg_ops->hdmi_audio_write_register =
		(hdmi_audio_reg_ops.hdmi_audio_write_register);
	reg_ops->hdmi_audio_read_modify =
		(hdmi_audio_reg_ops.hdmi_audio_read_modify);
	query_ops->hdmi_audio_get_caps =
		hdmi_audio_get_set_ops.hdmi_audio_get_caps;
	query_ops->hdmi_audio_set_caps =
		hdmi_audio_get_set_ops.hdmi_audio_set_caps;

	ctx->had_event_callbacks = audio_callbacks;

	return HAD_SUCCESS;
}

void _had_wq(struct work_struct *work)
{
	mid_hdmi_audio_signal_event(HAD_EVENT_HOT_PLUG);
}

int mid_hdmi_audio_register(struct snd_intel_had_interface *driver,
				void *had_data)
{
	struct hdmi_lpe_audio_ctx *ctx;

	ctx = platform_get_drvdata(gpdev);

	pr_debug("%s: called\n", __func__);

	ctx->had_pvt_data = had_data;
	ctx->had_interface = driver;

	/* The Audio driver is loading now and we need to notify
	 * it if there is an HDMI device attached
	 */
	INIT_WORK(&ctx->hdmi_audio_wq, _had_wq);
	pr_debug("%s: Scheduling HDMI audio work queue\n", __func__);
	schedule_work(&ctx->hdmi_audio_wq);

	return HAD_SUCCESS;
}

static irqreturn_t display_pipe_interrupt_handler(int irq, void *dev_id)
{
	u32 audio_stat, audio_reg;

	struct hdmi_lpe_audio_ctx *ctx;

	pr_debug("%s: Enter\n", __func__);

	ctx = platform_get_drvdata(gpdev);

	audio_reg = ctx->had_config_offset + AUD_HDMI_STATUS_v2;
	hdmi_audio_read(audio_reg, &audio_stat);

	if (audio_stat & HDMI_AUDIO_UNDERRUN) {
		hdmi_audio_write(audio_reg, HDMI_AUDIO_UNDERRUN);
		mid_hdmi_audio_signal_event(
				HAD_EVENT_AUDIO_BUFFER_UNDERRUN);
	}

	if (audio_stat & HDMI_AUDIO_BUFFER_DONE) {
		hdmi_audio_write(audio_reg, HDMI_AUDIO_BUFFER_DONE);
		mid_hdmi_audio_signal_event(
				HAD_EVENT_AUDIO_BUFFER_DONE);
	}

	return IRQ_HANDLED;
}

static void notify_audio_lpe(void *audio_ptr)
{
	struct hdmi_lpe_audio_ctx *ctx = get_hdmi_context();
	struct intel_hdmi_lpe_audio_pdata *pdata = gpdev->dev.platform_data;
	struct intel_hdmi_lpe_audio_eld *eld = audio_ptr;

	if (pdata->hdmi_connected != true) {

		pr_debug("%s: Event: HAD_NOTIFY_HOT_UNPLUG\n",
			__func__);

		if (_hdmi_state == hdmi_connector_status_connected) {

			_hdmi_state =
				hdmi_connector_status_disconnected;

			mid_hdmi_audio_signal_event(
				HAD_EVENT_HOT_UNPLUG);
		} else
			pr_debug("%s: Already Unplugged!\n", __func__);

	} else if (eld != NULL) {

		hdmi_set_eld(eld->eld_data);

		mid_hdmi_audio_signal_event(HAD_EVENT_HOT_PLUG);

		_hdmi_state = hdmi_connector_status_connected;

		pr_debug("%s: HAD_NOTIFY_ELD : port = %d, tmds = %d\n",
			__func__, eld->port_id,
			pdata->tmds_clock_speed);

		if (pdata->tmds_clock_speed) {
			ctx->tmds_clock_speed = pdata->tmds_clock_speed;
			mid_hdmi_audio_signal_event(HAD_EVENT_MODE_CHANGING);
		}
	} else
		pr_debug("%s: Event: NULL EDID!!\n", __func__);
}

/**
 * hdmi_lpe_audio_probe - start bridge with i915
 *
 * This function is called when the i915 driver creates the
 * hdmi-lpe-audio platform device. Card creation is deferred until a
 * hot plug event is received
 */
static int hdmi_lpe_audio_probe(struct platform_device *pdev)
{
	struct hdmi_lpe_audio_ctx *ctx;
	struct intel_hdmi_lpe_audio_pdata *pdata;
	int irq;
	struct resource *res_mmio;
	void __iomem *mmio_start;
	int ret = 0;
	unsigned long flag_irq;
	const struct pci_device_id cherryview_ids[] = {
		{PCI_DEVICE(0x8086, 0x22b0)},
		{PCI_DEVICE(0x8086, 0x22b1)},
		{PCI_DEVICE(0x8086, 0x22b2)},
		{PCI_DEVICE(0x8086, 0x22b3)},
		{}
	};

	pr_debug("Enter %s\n", __func__);

	/*TBD:remove globals*/
	gpdev = pdev;
	_hdmi_state = hdmi_connector_status_disconnected;

	/* get resources */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		pr_err("Could not get irq resource\n");
		return -ENODEV;
	}

	res_mmio = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res_mmio) {
		pr_err("Could not get IO_MEM resources\n");
		return -ENXIO;
	}

	pr_debug("%s: mmio_start = 0x%x, mmio_end = 0x%x\n", __func__,
		(unsigned int)res_mmio->start, (unsigned int)res_mmio->end);

	mmio_start = ioremap_nocache(res_mmio->start,
				(size_t)((res_mmio->end -
					res_mmio->start) + 1));
	if (!mmio_start) {
		pr_err("Could not get ioremap\n");
		return -EACCES;
	}

	/* setup interrupt handler */
	ret = request_irq(irq, display_pipe_interrupt_handler,
			0, /* FIXME: is IRQF_SHARED needed ? */
			pdev->name,
			NULL);
	if (ret < 0) {
		pr_err("request_irq failed\n");
		iounmap(mmio_start);
		return -ENODEV;
	}

	/* alloc and save context */
	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (ctx == NULL) {
		free_irq(irq, NULL);
		iounmap(mmio_start);
		return -ENOMEM;
	}

	ctx->irq = irq;
	pr_debug("hdmi lpe audio: irq num = %d\n", irq);
	ctx->mmio_start = mmio_start;
	ctx->tmds_clock_speed = DIS_SAMPLE_RATE_148_5;

	if (pci_dev_present(cherryview_ids)) {
		pr_debug("%s: Cherrytrail LPE - Detected\n", __func__);
		ctx->had_config_offset = AUDIO_HDMI_CONFIG_C;
	} else {
		pr_debug("%s: Baytrail LPE - Assume\n", __func__);
		ctx->had_config_offset = AUDIO_HDMI_CONFIG_A;
	}

	pdata = pdev->dev.platform_data;

	if (pdata == NULL) {
		pr_err("%s: quit: pdata not allocated by i915!!\n", __func__);
		kfree(ctx);
		free_irq(irq, NULL);
		iounmap(mmio_start);
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, ctx);

	ret = hdmi_audio_probe((void *)pdev);

	pr_debug("hdmi lpe audio: setting pin eld notify callback\n");

	spin_lock_irqsave(&pdata->lpe_audio_slock, flag_irq);
	pdata->notify_audio_lpe = notify_audio_lpe;
	if (pdata->notify_pending) {

		pr_debug("%s: handle pending notification\n", __func__);
		notify_audio_lpe(&pdata->eld);
		pdata->notify_pending = false;
	}
	spin_unlock_irqrestore(&pdata->lpe_audio_slock, flag_irq);

	return ret;
}

/**
 * hdmi_lpe_audio_remove - stop bridge with i915
 *
 * This function is called when the platform device is destroyed. The sound
 * card should have been removed on hot plug event.
 */
static int hdmi_lpe_audio_remove(struct platform_device *pdev)
{
	struct hdmi_lpe_audio_ctx *ctx;

	pr_debug("Enter %s\n", __func__);

	hdmi_audio_remove(pdev);

	/* get context, release resources */
	ctx = platform_get_drvdata(pdev);
	iounmap(ctx->mmio_start);
	free_irq(ctx->irq, NULL);
	kfree(ctx);
	return HAD_SUCCESS;
}

static int hdmi_lpe_audio_suspend(struct platform_device *pt_dev,
				pm_message_t state)
{
	pr_debug("Enter %s\n", __func__);
	mid_hdmi_audio_suspend(NULL);
	return HAD_SUCCESS;
}

static int hdmi_lpe_audio_resume(struct platform_device *pt_dev)
{
	pr_debug("Enter %s\n", __func__);
	mid_hdmi_audio_resume(NULL);
	return HAD_SUCCESS;
}

static struct platform_driver hdmi_lpe_audio_driver = {
	.driver		= {
		.name  = "hdmi-lpe-audio",
	},
	.probe          = hdmi_lpe_audio_probe,
	.remove		= hdmi_lpe_audio_remove,
	.suspend	= hdmi_lpe_audio_suspend,
	.resume		= hdmi_lpe_audio_resume
};

module_platform_driver(hdmi_lpe_audio_driver);
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:hdmi_lpe_audio");
