/******************************************************************************
 *
 * Copyright(c) 2009-2013  Realtek Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110, USA
 *
 * The full GNU General Public License is included in this distribution in the
 * file called LICENSE.
 *
 * Contact Information:
 * wlanfae <wlanfae@realtek.com>
 * Realtek Corporation, No. 2, Innovation Road II, Hsinchu Science Park,
 * Hsinchu 300, Taiwan.
 *
 * Christian Lamparter <chunkeey@googlemail.com>
 * Joshua Roys <Joshua.Roys@gtri.gatech.edu>
 * Larry Finger <Larry.Finger@lwfinger.net>
 *
 *****************************************************************************/
#include <linux/kernel.h>

#include <linux/firmware.h>
#include "r92su.h"
#include "usb.h"
#include "fw.h"
#include "def.h"
#include "reg.h"
#include "debug.h"

static int r92su_parse_firmware(struct r92su *r92su)
{
	const struct fw_hdr *hdr;
	unsigned int sram_size;
	unsigned int imem_size;
	unsigned int dmem_size;

	if (r92su->fw->size > RTL8192_MAX_RAW_FIRMWARE_CODE_SIZE) {
		R92SU_ERR(r92su, "firmware is too big.\n");
		return -EINVAL;
	}

	r92su->fw_header = hdr = (const void *) r92su->fw->data;

	if ((hdr->signature != cpu_to_le16(R8192SU_FW_SIGNATURE)) &&
	    (hdr->signature != cpu_to_le16(R8712SU_FW_SIGNATURE))) {
		R92SU_ERR(r92su, "firmware signature check has failed.\n");
		return -EINVAL;
	}

	r92su->fw_version = le16_to_cpu(hdr->version);
	R92SU_INFO(r92su, "firmware version: 0x%x\n", r92su->fw_version);

	r92su->fw_imem_len = imem_size = le32_to_cpu(hdr->img_imem_size);
	r92su->fw_sram_len = sram_size = le32_to_cpu(hdr->img_sram_size);
	dmem_size = le32_to_cpu(hdr->dmem_size);

	r92su->fw_imem = r92su->fw->data + RT_8192S_FIRMWARE_HDR_SIZE;
	r92su->fw_sram = r92su->fw_imem + imem_size;

	if (imem_size == 0 || imem_size >= RTL8192_MAX_FIRMWARE_CODE_SIZE) {
		R92SU_ERR(r92su, "firmware's imem size is out of range\n");
		return -EINVAL;
	}

	if (sram_size == 0 || sram_size >= RTL8192_MAX_FIRMWARE_CODE_SIZE) {
		R92SU_ERR(r92su, "firmware's sram size is out of range\n");
		return -EINVAL;
	}

	if (dmem_size != sizeof(struct fw_priv)) {
		R92SU_ERR(r92su, "firmware's dmem size is out of range\n");
		return -EINVAL;
	}

	return 0;
}

static int r92su_prepare_firmware(struct r92su *r92su)
{
	struct fw_priv *dmem = &r92su->fw_dmem;

	memset(dmem, 0, sizeof(*dmem));

	/* For SDIO:
	 * dmem->hci_sel = RTL8712_HCI_TYPE_72SDIO;
	 */
	r92su_usb_prepare_firmware(r92su);

	dmem->mp_mode = 0; /* what's mp mode? multi-peer? mesh portal? */
	dmem->qos_en = 1;
	dmem->bw_40mhz_en = !r92su->disable_ht;
	dmem->ampdu_en = !r92su->disable_ht;
	dmem->aggregation_offload = !r92su->disable_ht;
	dmem->rate_control_offload = 1;
	dmem->mlme_offload = 1;
	dmem->vcs_type = 2; /* 0: off, 1: on, 2: auto */
	dmem->vcs_mode = 1; /* 0: off(presumably), 1:RTS/CTS, 2:CTS-Self */

	dmem->turbo_mode = 0;
	dmem->low_power_mode = 0;
	dmem->chip_version = r92su->chip_rev; /* not necessarily correct ?! */
	dmem->rf_config = r92su->rf_type;

	/* When scanning, send out two probe requests.
	 *  1. wildcard ssid
	 *  2. specified ssid - if set/available
	 */
	dmem->rsvd024 = 1;

	return 0;
}

static int r92su_upload_firmware_part(struct r92su *r92su,
				      const void *data, unsigned int len)
{
	const unsigned int block_size = 2048;

	unsigned int done = 0;
	const void *iter = data;
	int err;

	do {
		tx_hdr *hdr;
		struct sk_buff *skb;
		unsigned int current_block;

		skb = __dev_alloc_skb(TX_DESC_SIZE + block_size, GFP_KERNEL);
		if (!skb)
			return -ENOMEM;

		hdr = (tx_hdr *) skb_put(skb, sizeof(*hdr));
		memset(hdr, 0, sizeof(*hdr));

		current_block = min(block_size, len - done);
		done += current_block;

		SET_TX_DESC_PKT_SIZE(hdr, current_block);
		if (len == done)
			SET_TX_DESC_LINIP(hdr, 1);

		memcpy(skb_put(skb, current_block), iter, current_block);
		err = r92su_usb_tx(r92su, skb, RTL8712_VOQ);
		if (err)
			return err;

		iter += current_block;
	} while (done < len);

	return 0;
}

static int r92su_upload_mem_wait(struct r92su *r92su, const u8 done_flag,
				 const u8 done2_flag, const char *mem,
				 int tries, const int loop_wait)
{
	u8 cpu_status;

	do {
		cpu_status = r92su_read8(r92su, REG_TCR);
		if (cpu_status & done_flag)
			break;
		msleep(loop_wait);
	} while (--tries);

	if (!(cpu_status & done2_flag) || (tries == 0)) {
		R92SU_ERR(r92su, "firmware's %s upload %s cpu_status=0x%x\n",
			mem, (tries == 0) ? "timedout" : "failed", cpu_status);
		return -EAGAIN;
	}
	return 0;
}

static int r92su_firmware_enable_cpu(struct r92su *r92su)
{
	u32 tries = 1000;
	u16 func;
	u8 clk, cpu_status;

	/* select clock */
	clk = r92su_read8(r92su, REG_SYS_CLKR);
	clk |= SYS_CPU_CLKSEL;
	r92su_write8(r92su, REG_SYS_CLKR, clk);

	/* Enable CPU. */
	func = r92su_read16(r92su, REG_SYS_FUNC_EN);
	func |= FEN_CPUEN;
	r92su_write16(r92su, REG_SYS_FUNC_EN, func);

	/* Polling IMEM Ready after CPU has refilled. */
	do {
		cpu_status = r92su_read8(r92su, REG_TCR);
		if (cpu_status & IMEM_RDY)
			break;

		udelay(20);
	} while (--tries);

	return !!(cpu_status & IMEM_RDY) ? 0 : -ETIMEDOUT;
}

static int r92su_upload_imem(struct r92su *r92su)
{
	int err = -EINVAL;

	err = r92su_upload_firmware_part(r92su, r92su->fw_imem,
					 r92su->fw_imem_len);
	if (err)
		return err;

	err = r92su_upload_mem_wait(r92su, IMEM_CODE_DONE,
				     IMEM_CHK_RPT, "imem", 3, 20);
	if (err)
		return err;
	return 0;
}

static int r92su_upload_sram(struct r92su *r92su)
{
	int err = -EINVAL;

	err = r92su_upload_firmware_part(r92su, r92su->fw_sram,
					 r92su->fw_sram_len);
	if (err)
		return err;

	err = r92su_upload_mem_wait(r92su, EMEM_CODE_DONE,
				    EMEM_CHK_RPT, "sram", 3, 20);
	if (err)
		return err;

	err = r92su_firmware_enable_cpu(r92su);
	if (err)
		return err;

	return 0;
}

static int r92su_upload_dmem(struct r92su *r92su)
{
	int err = -EINVAL, boot_loops;

	err = r92su_upload_firmware_part(r92su, &r92su->fw_dmem,
					 sizeof(r92su->fw_dmem));
	if (err)
		return err;

	err = r92su_upload_mem_wait(r92su, DMEM_CODE_DONE,
				    DMEM_CODE_DONE, "dmem", 100, 20);
	if (err)
		return err;

	if (r92su_read8(r92su, REG_EEPROM_CMD) & EEPROM_CMD_93C46) {
		/* When booting from the eeprom, the firmware needs more
		 * time to complete the boot procedure. */
		boot_loops = 60;
	} else {
		/* Boot from eefuse is faster */
		boot_loops = 30;
	}

	err = r92su_upload_mem_wait(r92su, FWRDY, LOAD_FW_READY, "boot",
				    boot_loops, 100);
	if (err)
		return err;

	return 0;
}

int r92su_upload_firmware(struct r92su *r92su)
{
	int err = -EINVAL;

	err = r92su_upload_imem(r92su);
	if (err)
		return err;

	err = r92su_upload_sram(r92su);
	if (err)
		return err;

	err = r92su_upload_dmem(r92su);
	if (err)
		return err;

	r92su->fw_loaded = true;
	return err;
}

int r92su_load_firmware(struct r92su *r92su)
{
	int err = -EINVAL;

	/* only load and parse the firmware only once. */
	if (r92su->fw)
		return 0;

	err = request_firmware(&r92su->fw, RTL8192SU_FIRMWARE,
			       &r92su->udev->dev);
	if (err) {
		R92SU_ERR(r92su, "firmware '%s' not found.\n",
			  RTL8192SU_FIRMWARE);
		return err;
	}

	err = r92su_parse_firmware(r92su);
	if (err)
		return err;

	err = r92su_prepare_firmware(r92su);
	if (err)
		return err;

	return err;
}

void r92su_release_firmware(struct r92su *r92su)
{
	release_firmware(r92su->fw);
	r92su->fw = NULL;
}
