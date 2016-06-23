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

#include "r92su.h"
#include "hw.h"
#include "reg.h"

#include "usb.h"
#include "debug.h"

static u8 r92su_halset_sysclk(struct r92su *r92su, u16 clk_set)
{
	u16 clk;
	u8 tries = 100;
	bool result;

	r92su_write16(r92su, REG_SYS_CLKR, clk_set);

	/* Wait until the MAC is synchronized. */
	udelay(400);

	/* Check if it is set ready. */
	clk = r92su_read16(r92su, REG_SYS_CLKR);
	result = ((clk & SYS_FWHW_SEL) == (clk_set & SYS_FWHW_SEL));

	if (!(clk_set & (SYS_SWHW_SEL | SYS_FWHW_SEL))) {
		do {
			udelay(10);

			clk = r92su_read16(r92su, REG_SYS_CLKR);
			if ((clk & SYS_SWHW_SEL))
				return true;

			R92SU_ERR(r92su, "wait for SYS_SWHW_SEL in %x\n", clk);
		} while (--tries);
		return false;
	}
	return result;
}

static int r92su_usb_init_b_and_c_cut(struct r92su *r92su)
{
	unsigned int tries = 20;
	u8 tmpu1b;
	u16 tmpu2b;

	/* Prevent EFUSE leakage */
	r92su_write8(r92su, REG_EFUSE_TEST + 3, 0xb0);
	msleep(20);
	r92su_write8(r92su, REG_EFUSE_TEST + 3, 0x30);

	/* Set control path switch to HW control and reset digital core,
	 * CPU core and MAC I/O core. */
	tmpu2b = r92su_read16(r92su, REG_SYS_CLKR);
	if (tmpu2b & SYS_FWHW_SEL) {
		tmpu2b &= ~(SYS_SWHW_SEL | SYS_FWHW_SEL);

		/* Set failed, return to prevent hang. */
		if (!r92su_halset_sysclk(r92su, tmpu2b))
			return -EIO;
	}

	/* Reset MAC-IO and CPU and Core Digital BIT(10)/11/15 */
	tmpu1b = r92su_read8(r92su, REG_SYS_FUNC_EN + 1);
	tmpu1b &= 0x73;
	r92su_write8(r92su, REG_SYS_FUNC_EN + 1, tmpu1b);
	/* wait for BIT 10/11/15 to pull high automatically!! */
	mdelay(1);

	r92su_write8(r92su, REG_SPS0_CTRL + 1, 0x53);
	r92su_write8(r92su, REG_SPS0_CTRL, 0x57);

	/* Enable AFE Macro Block's Bandgap */
	tmpu1b = r92su_read8(r92su, REG_AFE_MISC);
	r92su_write8(r92su, REG_AFE_MISC, (tmpu1b | AFE_BGEN));
	mdelay(1);

	/* Enable AFE Mbias */
	tmpu1b = r92su_read8(r92su, REG_AFE_MISC);
	r92su_write8(r92su, REG_AFE_MISC, (tmpu1b | AFE_BGEN |
		       AFE_MBEN | AFE_MISC_I32_EN));
	mdelay(1);

	/* Enable LDOA15 block	*/
	tmpu1b = r92su_read8(r92su, REG_LDOA15_CTRL);
	r92su_write8(r92su, REG_LDOA15_CTRL, (tmpu1b | LDA15_EN));

	/* Enable LDOV12D block */
	tmpu1b = r92su_read8(r92su, REG_LDOV12D_CTRL);
	r92su_write8(r92su, REG_LDOV12D_CTRL, (tmpu1b | LDV12_EN));

	/* Set Digital Vdd to Retention isolation Path. */
	tmpu2b = r92su_read16(r92su, REG_SYS_ISO_CTRL);
	r92su_write16(r92su, REG_SYS_ISO_CTRL, (tmpu2b | ISO_PWC_DV2RP));

	/* For warm reboot NIC disappear bug.
	 * Also known as: Engineer Packet CP test Enable */
	tmpu2b = r92su_read16(r92su, REG_SYS_FUNC_EN);
	r92su_write16(r92su, REG_SYS_FUNC_EN, (tmpu2b | BIT(13)));

	/* Support 64k IMEM */
	tmpu1b = r92su_read8(r92su, REG_SYS_ISO_CTRL + 1);
	r92su_write8(r92su, REG_SYS_ISO_CTRL + 1, (tmpu1b & 0x68));

	/* Enable AFE clock source */
	tmpu1b = r92su_read8(r92su, REG_AFE_XTAL_CTRL);
	r92su_write8(r92su, REG_AFE_XTAL_CTRL, (tmpu1b | 0x01));
	/* Delay 1.5ms */
	mdelay(2);
	tmpu1b = r92su_read8(r92su, REG_AFE_XTAL_CTRL + 1);
	r92su_write8(r92su, REG_AFE_XTAL_CTRL + 1, (tmpu1b & 0xfb));

	/* Enable AFE PLL Macro Block *
	 * We need to delay 100u before enabling PLL. */
	udelay(200);
	tmpu1b = r92su_read8(r92su, REG_AFE_PLL_CTRL);
	r92su_write8(r92su, REG_AFE_PLL_CTRL, (tmpu1b | BIT(0) | BIT(4)));

	/* for divider reset
	 * The clock will be stable with 500us delay after the PLL reset */
	udelay(500);
	r92su_write8(r92su, REG_AFE_PLL_CTRL, (tmpu1b | BIT(0) |
		       BIT(4) | BIT(6)));
	udelay(500);
	r92su_write8(r92su, REG_AFE_PLL_CTRL, (tmpu1b | BIT(0) | BIT(4)));
	udelay(500);

	/* Release isolation AFE PLL & MD */
	tmpu1b = r92su_read8(r92su, REG_SYS_ISO_CTRL);
	r92su_write8(r92su, REG_SYS_ISO_CTRL, (tmpu1b & 0xee));

	/* Switch to 40MHz clock */
	r92su_write8(r92su, REG_SYS_CLKR, 0x00);

	/* Disable CPU clock and 80MHz SSC to fix FW download timing issue */
	tmpu1b = r92su_read8(r92su, REG_SYS_CLKR);
	r92su_write8(r92su, REG_SYS_CLKR, (tmpu1b | 0xa0));

	/* Enable MAC clock */
	tmpu2b = r92su_read16(r92su, REG_SYS_CLKR);
	tmpu2b |= BIT(12) | BIT(11);
	if (!r92su_halset_sysclk(r92su, tmpu2b))
		return -EIO;

	r92su_write8(r92su, REG_PMC_FSM, 0x02);

	/* Enable Core digital and enable IOREG R/W */
	tmpu2b = r92su_read16(r92su, REG_SYS_FUNC_EN);
	r92su_write16(r92su, REG_SYS_FUNC_EN, (tmpu2b | BIT(11)));

	/* enable REG_EN */
	tmpu2b = r92su_read16(r92su, REG_SYS_FUNC_EN);
	r92su_write16(r92su, REG_SYS_FUNC_EN, (tmpu2b | BIT(15)));

	/* Switch the control path to FW */
	tmpu2b = r92su_read16(r92su, REG_SYS_CLKR);
	tmpu2b |= SYS_FWHW_SEL;
	tmpu2b &= ~SYS_SWHW_SEL;
	if (!r92su_halset_sysclk(r92su, tmpu2b))
		return -EIO;

	r92su_write16(r92su, REG_CR, HCI_TXDMA_EN |
		HCI_RXDMA_EN | TXDMA_EN | RXDMA_EN | FW2HW_EN |
		DDMA_EN | MACTXEN | MACRXEN | SCHEDULE_EN |
		BB_GLB_RSTN | BBRSTN);

	/* Fix USB RX FIFO error */
	tmpu1b = r92su_read8(r92su, REG_USB_AGG_TO);
	r92su_write8(r92su, REG_USB_AGG_TO, tmpu1b | BIT(7));

	/* Enable MAC clock */
	tmpu2b = r92su_read16(r92su, REG_SYS_CLKR);
	tmpu2b &= ~SYS_CPU_CLKSEL;
	if (!r92su_halset_sysclk(r92su, tmpu2b))
		return -EIO;

	/* Fix 8051 ROM incorrect code operation */
	r92su_write8(r92su, REG_USB_MAGIC, USB_MAGIC_BIT7);

	/* To make sure that TxDMA can ready to download FW. */
	/* We should reset TxDMA if IMEM RPT was not ready. */
	do {
		tmpu1b = r92su_read8(r92su, REG_TCR);
		if ((tmpu1b & TXDMA_INIT_VALUE) == TXDMA_INIT_VALUE)
			break;

		udelay(5);
	} while (--tries);

	if (tries == 0) {
		R92SU_ERR(r92su,
			 "Polling TXDMA_INIT_VALUE timed out! Current TCR(%#x)\n",
			 tmpu1b);
		tmpu1b = r92su_read8(r92su, REG_CR);
		r92su_write8(r92su, REG_CR, tmpu1b & (~TXDMA_EN));
		udelay(2);
		/* Reset TxDMA */
		r92su_write8(r92su, REG_CR, tmpu1b | TXDMA_EN);
	}

	return 0;
}

int r92su_hw_early_mac_setup(struct r92su *r92su)
{
	int err = -EINVAL;

	/* Clear RPWM to ensure driver and fw are back in the initial state */
	r92su_write8(r92su, REG_USB_HRPWM, PS_ACTIVE);

	switch (r92su->chip_rev) {
	case R92SU_C_CUT:
	case R92SU_B_CUT:
		err = r92su_usb_init_b_and_c_cut(r92su);
		if (err)
			return err;
		break;

	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int r92su_upload_finish(struct r92su *r92su)
{
	uint32_t cr;

	/* Right here, we can set TCR/RCR to desired value */
	/* and config MAC lookback mode to normal mode */

	cr = r92su_read32(r92su, REG_TCR);
	cr &= ~TCR_ICV;
	r92su_write32(r92su, REG_TCR, cr);

	cr = r92su_read32(r92su, REG_RCR);
	cr |= RCR_APP_PHYST_RXFF | RCR_APP_ICV | RCR_APP_MIC;
	r92su_write32(r92su, REG_RCR, cr);

	/* Set to normal mode. */
	r92su_write8(r92su, REG_LBKMD_SEL, LBK_NORMAL);
	return 0;
}

static int r92su_usb_final_macconfig(struct r92su *r92su)
{
	u8 tmp;
	/* Setting TX/RX page size to 128 byte */
	tmp = r92su_read8(r92su, REG_PBP);
	tmp |= PBP_PAGE_128B;
	r92su_write8(r92su, REG_PBP, tmp);
	r92su->rx_alignment = 128;

	/* enable aggregation */
	tmp = r92su_read8(r92su, REG_RXDMA_RXCTRL);
	tmp |= RXDMA_AGG_EN;
	r92su_write8(r92su, REG_RXDMA_RXCTRL, tmp);

	/* 48 pages * 128 Byte / Page = 6kb */
	r92su_write8(r92su, REG_RXDMA_AGG_PG_TH, 48);

	/* 1.7 ms / "0x04" */
	r92su_write8(r92su, REG_USB_DMA_AGG_TO, 0x04);

	/* Fix the RX FIFO issue (USB Error) */
	tmp = r92su_read8(r92su, REG_USB_AGG_TO);
	tmp |= BIT(7);
	r92su_write8(r92su, REG_USB_AGG_TO, tmp);
	return 0;
}

static int r92su_macconfig_after_fwdownload(struct r92su *r92su)
{
	u32 tmp16;

	/* (re-)start all queues */
	tmp16 = r92su_read16(r92su, REG_TXPAUSE);
	tmp16 &= ~(STOPBK | STOPBE | STOPVI | STOPVO | STOPMGT |
		STOPHIGH | STOPHCCA | BIT(7));
	r92su_write16(r92su, REG_TXPAUSE, tmp16);

	return 0;
}

static int r92su_wps_cfg_inputmethod(struct r92su *r92su)
{
	u8 u1tmp;

	/* The following config GPIO function */
	r92su_write8(r92su, REG_MAC_PINMUX_CTRL, (GPIOMUX_EN | GPIOSEL_GPIO));
	u1tmp = r92su_read8(r92su, REG_GPIO_IO_SEL);

	/* config GPIO4 to input */
	u1tmp &= ~HAL_8192S_HW_GPIO_WPS_BIT;
	r92su_write8(r92su, REG_GPIO_IO_SEL, u1tmp);
	return 0;
}

static bool r92su_wps_detect(struct r92su *r92su)
{
	u8 u1tmp;

	/* The following config GPIO function */
	r92su_write8(r92su, REG_MAC_PINMUX_CTRL, (GPIOMUX_EN | GPIOSEL_GPIO));
	u1tmp = r92su_read8(r92su, REG_GPIO_IO_SEL);

	/* config GPIO4 to input */
	u1tmp &= ~HAL_8192S_HW_GPIO_WPS_BIT;
	r92su_write8(r92su, REG_GPIO_IO_SEL, u1tmp);

	/* On some of the platform, driver cannot read correct
	 * value without delay between Write_GPIO_SEL and Read_GPIO_IN */
	mdelay(10);

	/* check GPIO4 */
	u1tmp = r92su_read8(r92su, REG_GPIO_CTRL);
	return !!(u1tmp & HAL_8192S_HW_GPIO_WPS_BIT);
}

int r92su_hw_late_mac_setup(struct r92su *r92su)
{
	int err = -EINVAL;

	err = r92su_upload_finish(r92su);
	if (err)
		return err;

	err = r92su_macconfig_after_fwdownload(r92su);
	if (err)
		return err;

	err = r92su_usb_final_macconfig(r92su);
	if (err)
		return err;

	err = r92su_wps_cfg_inputmethod(r92su);
	if (err)
		return err;

	return 0;
}

int r92su_hw_mac_deinit(struct r92su *r92su)
{
	r92su_write8(r92su, REG_RF_CTRL, 0x00);
	/* Turn off BB */
	msleep(20);

	/* Turn off MAC */
	r92su_write8(r92su, REG_SYS_CLKR+1, 0x38); /* Switch Control Path */
	r92su_write8(r92su, REG_SYS_FUNC_EN+1, 0x70);
	r92su_write8(r92su, REG_PMC_FSM, 0x06);  /* Enable Loader Data Keep */
	r92su_write8(r92su, REG_SYS_ISO_CTRL, 0xF9); /* Isolation signals from
						      * CORE, PLL */
	r92su_write8(r92su, REG_SYS_ISO_CTRL+1, 0xe8); /* Enable EFUSE 1.2V */
	r92su_write8(r92su, REG_AFE_PLL_CTRL, 0x00); /* Disable AFE PLL. */
	r92su_write8(r92su, REG_LDOA15_CTRL, 0x54);  /* Disable A15V */
	r92su_write8(r92su, REG_SYS_FUNC_EN+1, 0x50); /* Disable E-Fuse 1.2V */
	r92su_write8(r92su, REG_LDOV12D_CTRL, 0x24); /* Disable LDO12(for CE) */
	r92su_write8(r92su, REG_AFE_MISC, 0x30); /* Disable AFE BG&MB */
	/* Option for Disable 1.6V LDO. */
	r92su_write8(r92su, REG_SPS0_CTRL, 0x56); /* Disable 1.6V LDO */
	r92su_write8(r92su, REG_SPS0_CTRL+1, 0x43);  /* Set SW PFM */
	return 0;
}

int r92su_hw_mac_set_rx_filter(struct r92su *r92su,
	bool data, bool all_mgt, bool ctrl, bool monitor)
{
	u32 rcr;

	rcr = r92su_read32(r92su, REG_RCR);
	if (data)
		rcr |= RCR_ADF;
	else
		rcr &= ~RCR_ADF;
	if (ctrl)
		rcr |= RCR_ACF;
	else
		rcr &= ~RCR_ACF;

	rcr |= RCR_AMF;

	if (monitor) {
		/* Accept all destinations and don't upload garbage ?! */
		rcr |= RCR_AAP | RCR_CBSSID;
	} else {
		rcr &= ~(RCR_AAP | RCR_CBSSID);
	}

	/* Note: Playing with APPFCS can be dangerous. For example
	 * the FCS is contained in the firmware survey_done events
	 * BSS information (handled by "c2h_survey_event")
	 */
	rcr |= RCR_APPFCS | RCR_APWRMGT | /* RCR_ADD3 <== what's that? WDS? */
	       RCR_APP_MIC | RCR_APP_ICV | /* Keep MIC & ICV */
	       RCR_AICV | /* Accept ICV error */
	       RCR_AB | RCR_AM  | /* Accept Broadcast, Multicast */
	       RCR_APM | /* Accept Physical match */
	       RCR_APP_PHYST_STAFF /* Accept PHY status */;

	r92su_write32(r92su, REG_RCR, rcr);

	/* A cleared bit in the rxfltmapX bitmap means that it won't
	 * filter a individual frame type */
	r92su_write16(r92su, REG_RXFLTMAP2, data ? 0 : 0xffff);
	r92su_write16(r92su, REG_RXFLTMAP1, ctrl ? 0 : 0xffff);

	/* changing RXFLTMAP0 affects the firmware's scan/survey ability*/
	r92su_write16(r92su, REG_RXFLTMAP0, all_mgt ? 0 : 0x3f3f);
	return 0;
}

void r92su_hw_queue_service_work(struct r92su *r92su)
{
	queue_delayed_work(system_unbound_wq, &r92su->service_work, 2 * HZ);
}

int r92su_signal_scale_mapping(u32 raw_signal)
{
#define QUAL(val, low_eq, high_eq, res)					\
	do {								\
		if (((val) >= (low_eq)) && ((val) <= (high_eq)))	\
			return res;					\
	} while (0)


	/* is this some sort of scaled SNR ? */
	QUAL(raw_signal, 51, 100, 100);
	QUAL(raw_signal, 41, 50, 80 + (raw_signal - 40) * 2);
	QUAL(raw_signal, 31, 40, 36 + raw_signal);
	QUAL(raw_signal, 21, 30, 34 + raw_signal);
	QUAL(raw_signal, 10, 20, 42 + ((raw_signal - 10) * 3) / 2);
	QUAL(raw_signal, 5, 9, 22 + ((raw_signal - 5) * 3) / 2);
	QUAL(raw_signal, 1, 4, 6 + ((raw_signal - 1) * 3) / 2);
	return raw_signal;

#undef QUAL
}

static void r92su_query_fw_rx_phy_status(struct r92su *r92su)
{
	int tries = 50;

	r92su_write32(r92su, REG_IOCMD_CTRL, 0xf4000001);
	msleep(100);
	while ((r92su_read32(r92su, REG_IOCMD_CTRL)) && --tries)
		msleep(20);

	if (tries != 0) {
		struct cfg80211_bss *bss;
		int qual;
		qual = r92su_signal_scale_mapping(
			r92su_read32(r92su, REG_IOCMD_DATA) >> 4);

		rcu_read_lock();
		bss = rcu_dereference(r92su->connect_bss);
		if (bss) {
			struct r92su_bss_priv *bss_priv;
			struct r92su_sta *sta;

			bss->signal = qual;

			bss_priv = r92su_get_bss_priv(bss);
			sta = bss_priv->sta;
			if (sta)
				sta->signal = qual;
		}
		rcu_read_unlock();
	}
}

static void r92su_hw_service_work(struct work_struct *work)
{
	struct r92su *r92su = container_of(work, struct r92su,
		service_work.work);

	mutex_lock(&r92su->lock);
	if (!r92su_is_initializing(r92su))
		goto out;

	if (r92su->wps_pbc) {
		bool state = r92su_wps_detect(r92su);
		if (r92su->wps_pbc_state != state) {
			r92su->wps_pbc_state = state;
			input_report_key(r92su->wps_pbc, KEY_WPS_BUTTON,
					 state);
			input_sync(r92su->wps_pbc);
		}
	}

	r92su->scanned = false;

	r92su_query_fw_rx_phy_status(r92su);

	r92su_hw_queue_service_work(r92su);
out:
	mutex_unlock(&r92su->lock);
}

void r92su_hw_init(struct r92su *r92su)
{
	INIT_DELAYED_WORK(&r92su->service_work, r92su_hw_service_work);
}

int r92su_hw_read_chip_version(struct r92su *r92su)
{
	u8 rev;

	rev = GET_VAL(PCM_FSM_VER, r92su_read32(r92su, REG_PMC_FSM));
	if (rev != R92SU_C_CUT) {
		rev = (rev >> 1) + 1;
		if (rev > R92SU_C_CUT) {
			/* the vendor code will default to B_CUT
			 * in this case. But I'm not sure what the
			 * math above is all about. */
			return -EINVAL;
		}
	}
	r92su->chip_rev = rev;
	return 0;
}
