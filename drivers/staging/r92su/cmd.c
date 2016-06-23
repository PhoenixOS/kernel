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
#include "cmd.h"
#include "usb.h"
#include "reg.h"
#include "eeprom.h"
#include "def.h"
#include "trace.h"

void r92su_cmd_init(struct r92su *r92su)
{
	r92su->h2c_seq = 1;
	r92su->c2h_seq = 0;
	spin_lock_init(&r92su->tx_cmd_lock);
}

struct sk_buff *r92su_h2c_alloc(struct r92su *r92su, int len, gfp_t flag)
{
	struct sk_buff *skb;
	unsigned int new_len =
		ALIGN(len + TX_DESC_SIZE + H2CC2H_HDR_LEN, H2CC2H_HDR_LEN);
	skb = __dev_alloc_skb(new_len, flag);
	if (skb)
		skb_reserve(skb, TX_DESC_SIZE + H2CC2H_HDR_LEN);

	return skb;
}

static void r92su_h2c_fill_header(struct r92su *r92su,
				  struct sk_buff *skb,
				  unsigned int len,
				  enum fw_h2c_cmd cmd,
				  bool last)
{
	struct h2cc2h *h2c;
	h2c = (struct h2cc2h *) skb_push(skb, H2CC2H_HDR_LEN);

	memset(h2c, 0, sizeof(*h2c));
	h2c->cmd_seq = r92su->h2c_seq++;
	if (last)
		h2c->cmd_seq |= 0x80;
	h2c->len = cpu_to_le16(ALIGN(len, H2CC2H_HDR_LEN));
	h2c->event = cmd;
}

static void __r92su_tx_fill_header(tx_hdr *tx_hdr, unsigned int len,
				   bool first, bool last, unsigned int qsel)
{
	SET_TX_DESC_PKT_SIZE(tx_hdr, len);
	SET_TX_DESC_OFFSET(tx_hdr, TX_DESC_SIZE);
	SET_TX_DESC_LAST_SEG(tx_hdr, last);
	SET_TX_DESC_FIRST_SEG(tx_hdr, first);
	SET_TX_DESC_OWN(tx_hdr, 1);
	SET_TX_DESC_QUEUE_SEL(tx_hdr, qsel);
}

static void r92su_tx_fill_header(struct sk_buff *skb,
				 unsigned int len, bool first, bool last)
{
	tx_hdr *txhdr;
	txhdr = (tx_hdr *) skb_push(skb, sizeof(*txhdr));
	memset(txhdr, 0, sizeof(*txhdr));
	__r92su_tx_fill_header(txhdr, len, first, last, QSLT_CMD);
}

int r92su_h2c_submit(struct r92su *r92su, struct sk_buff *skb,
		     const enum fw_h2c_cmd cmd)
{
	unsigned long flags;
	int err;

	spin_lock_irqsave(&r92su->tx_cmd_lock, flags);
	r92su_h2c_fill_header(r92su, skb, skb->len, cmd, true);
	trace_r92su_h2c(wiphy_dev(r92su->wdev.wiphy),
			(struct h2cc2h *) skb->data);
	r92su_tx_fill_header(skb, skb->len, true, true);
	err = r92su_usb_tx(r92su, skb, RTL8712_H2CCMD);
	spin_unlock_irqrestore(&r92su->tx_cmd_lock, flags);
	return err;
}

int r92su_h2c_copy(struct r92su *r92su, const enum fw_h2c_cmd cmd,
		   const int len, const void *data, gfp_t flag)
{
	struct sk_buff *skb;
	int rest;

	skb = r92su_h2c_alloc(r92su, len, flag);
	if (!skb)
		return -ENOMEM;

	rest = H2CC2H_HDR_LEN - (len % H2CC2H_HDR_LEN);
	memcpy(skb_put(skb, len), data, len);
	memset(skb_put(skb, rest), 0, rest);

	return r92su_h2c_submit(r92su, skb, cmd);
}

int r92su_h2c_set_mac_addr(struct r92su *r92su, const u8 *addr)
{
	struct h2c_set_mac mac_args = { };

	memcpy(&mac_args.mac_addr, addr, ETH_ALEN);

	return r92su_h2c_copy(r92su, H2C_SET_MAC_ADDRESS_CMD, sizeof(mac_args),
			      &mac_args, GFP_KERNEL);

}

int r92su_h2c_set_opmode(struct r92su *r92su, const enum h2c_op_modes mode)
{
	struct h2c_op_mode mode_args = { };

	mode_args.mode = mode;

	return r92su_h2c_copy(r92su, H2C_SETOPMODE_CMD, sizeof(mode_args),
			      &mode_args, GFP_KERNEL);
}

int r92su_h2c_set_channel(struct r92su *r92su, const int channel)
{
	struct h2c_set_channel chan_args;

	chan_args.channel = cpu_to_le32(channel);

	return r92su_h2c_copy(r92su, H2C_SETCHANNEL_CMD, sizeof(chan_args),
			      &chan_args, GFP_KERNEL);
}

int r92su_fw_iocmd(struct r92su *r92su, const u32 cmd)
{
	int tries = 25;

	r92su_write32(r92su, REG_IOCMD_CTRL, cmd);

	do {
		if (--tries == 0)
			return -ETIMEDOUT;
		msleep(20);
	} while (r92su_read32(r92su, REG_IOCMD_CTRL) != 0);

	return 0;
}

int r92su_h2c_survey(struct r92su *r92su, struct cfg80211_ssid *ssid)
{
	struct h2c_site_survey survey = { };

	survey.active = cpu_to_le32(ssid != NULL);
	survey.bsslimit = cpu_to_le32(48);
	if (ssid) {
		survey.ssidlen = cpu_to_le32(ssid->ssid_len);
		memcpy(&survey.ssid, ssid->ssid, ssid->ssid_len);
	}

	return r92su_h2c_copy(r92su, H2C_SITESURVEY_CMD, sizeof(survey),
			      &survey, GFP_KERNEL);
}

int r92su_h2c_disconnect(struct r92su *r92su)
{
	struct h2c_disconnect disconnect = { };

	return r92su_h2c_copy(r92su, H2C_DISCONNECT_CMD, sizeof(disconnect),
			      &disconnect, GFP_KERNEL);
}

int r92su_h2c_connect(struct r92su *r92su, const struct h2cc2h_bss *orig_bss,
		      const bool join, const u8 *ie, const u32 ie_len)
{
	struct h2cc2h_bss *bss;
	struct sk_buff *skb;
	int rest;

	skb = r92su_h2c_alloc(r92su, sizeof(*bss) + ie_len, GFP_KERNEL);
	if (!skb)
		return -ENOMEM;

	bss = (struct h2cc2h_bss *)skb_put(skb, sizeof(*bss));
	*bss = *orig_bss;

	/* the ie_length also contains the fixed 12-byte ies (tsf/...) */
	bss->ie_length = cpu_to_le32(12 + ie_len);
	if (ie && ie_len > 0)
		memcpy(skb_put(skb, ie_len), ie, ie_len);

	bss->length = cpu_to_le32(skb->len);
	rest = H2CC2H_HDR_LEN - (skb->len % H2CC2H_HDR_LEN);
	memset(skb_put(skb, rest), 0, rest);
	return r92su_h2c_submit(r92su, skb, join ? H2C_JOINBSS_CMD :
		H2C_CREATEBSS_CMD);
}

static const u8 r92su_enc_alg_len[] = {
	[NO_ENCRYPTION] = 0,
	[WEP40_ENCRYPTION] = WLAN_KEY_LEN_WEP40,
	[WEP104_ENCRYPTION] = WLAN_KEY_LEN_WEP104,
	/* the rx and tx mic from the tkip key data
	 * are not uploaded to the firmware */
	[TKIP_ENCRYPTION] = 16,
	[TKIP_WTMIC] = 16,
	[AESCCMP_ENCRYPTION] = WLAN_KEY_LEN_CCMP,
};

int r92su_h2c_set_key(struct r92su *r92su, const enum r92su_enc_alg algo,
		      const u8 key_id, const bool group_key, const u8 *keydata)
{
	struct h2c_key key = { };

	BUILD_BUG_ON(ARRAY_SIZE(r92su_enc_alg_len) != __MAX_ENCRYPTION);

	key.algorithm = algo;
	key.key_id = key_id;
	key.group_key = !!group_key;
	memcpy(key.key, keydata, r92su_enc_alg_len[algo]);
	return r92su_h2c_copy(r92su, H2C_SETKEY_CMD, sizeof(key),
			      &key, GFP_KERNEL);
}

int r92su_h2c_set_sta_key(struct r92su *r92su, const enum r92su_enc_alg algo,
			  const u8 *mac_addr, const u8 *keydata)
{
	struct h2c_sta_key key = { };

	key.algorithm = algo;
	memcpy(key.mac_addr, mac_addr, ETH_ALEN);
	memcpy(key.key, keydata, r92su_enc_alg_len[algo]);
	return r92su_h2c_copy(r92su, H2C_SETSTAKEY_CMD, sizeof(key),
			      &key, GFP_KERNEL);
}

int r92su_h2c_set_auth(struct r92su *r92su, const enum r92su_auth_mode mode,
		       const enum r92su_auth_1x _1x)
{
	struct h2c_auth auth = { };

	auth.mode = mode;
	auth._1x = _1x;
	return r92su_h2c_copy(r92su, H2C_SETAUTH_CMD, sizeof(auth),
			      &auth, GFP_KERNEL);
}

int r92su_h2c_start_ba(struct r92su *r92su, const unsigned int tid)
{
	struct h2c_add_ba_req req = { };
	req.tid = cpu_to_le32(tid);

	return r92su_h2c_copy(r92su, H2C_ADDBA_REQ_CMD, sizeof(req),
			      &req, GFP_ATOMIC);
}

int r92su_h2c_set_power_mode(struct r92su *r92su, const u8 ps_mode,
			     const u8 smart_ps)
{
	struct h2c_set_power_mode pwr = { };
	pwr.mode = ps_mode;
	pwr.bcn_pass_cnt = 0;
	pwr.smart_ps = smart_ps;

	return r92su_h2c_copy(r92su, H2C_SETPWRMODE_CMD, sizeof(pwr),
			      &pwr, GFP_ATOMIC);
}
