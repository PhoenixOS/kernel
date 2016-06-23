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
#include <linux/etherdevice.h>

#include <net/ieee80211_radiotap.h>
#include <asm/unaligned.h>

#include "r92su.h"
#include "rx.h"
#include "usb.h"
#include "def.h"
#include "event.h"
#include "wep.h"
#include "tkip.h"
#include "michael.h"
#include "aes_ccm.h"
#include "debug.h"
#include "trace.h"

static void r92su_rx_add_radiotap(struct r92su *r92su,
				  rx_hdr *rx_hdr,
				  struct sk_buff *skb,
				  unsigned int rtap_len)
{
	unsigned char *pos;
	struct ieee80211_radiotap_header *rthdr;
	unsigned int rx_mcs;
	bool ht;

	rthdr = (struct ieee80211_radiotap_header *)skb_push(skb, rtap_len);
	memset(rthdr, 0, sizeof(*rthdr));

	rthdr->it_present =
		cpu_to_le32((1 << IEEE80211_RADIOTAP_FLAGS) |
			    (1 << IEEE80211_RADIOTAP_CHANNEL) |
			    (1 << IEEE80211_RADIOTAP_ANTENNA) |
			    (1 << IEEE80211_RADIOTAP_RX_FLAGS) |
			    (1 << IEEE80211_RADIOTAP_TSFT));
	rthdr->it_len = cpu_to_le16(rtap_len);

	pos = (unsigned char *)(rthdr + 1);

	/* Note: the order of the entries is important! */

	/* IEEE80211_RADIOTAP_TSFT + TSFT Alignment */
	while ((pos - (u8 *)rthdr) & 7)
		*pos++ = 0;

	put_unaligned_le64(GET_RX_DESC_TSFL(rx_hdr), pos);
	pos += 8;

	/* IEEE80211_RADIOTAP_FLAGS */
	*pos = IEEE80211_RADIOTAP_F_FCS;
	if (GET_RX_DESC_CRC32(rx_hdr))
		*pos |= IEEE80211_RADIOTAP_F_BADFCS;
	if (GET_RX_DESC_SPLCP(rx_hdr))
		*pos |= IEEE80211_RADIOTAP_F_SHORTPRE;
	pos++;

	rx_mcs = GET_RX_DESC_RX_MCS(rx_hdr);

	/* IEEE80211_RADIOTAP_RATE */
	ht = GET_RX_DESC_RX_HT(rx_hdr);
	if (ht) {
		/* Without rate information don't add it. If we have,
		 * MCS information is a separate field in radiotap,
		 * added below. The byte here is needed as padding
		 * for the channel though, so initialise it to 0.
		 */
		*pos = 0;
	} else {
		rthdr->it_present |= cpu_to_le32(1 << IEEE80211_RADIOTAP_RATE);
		*pos = r92su->band_2GHZ.bitrates[rx_mcs].bitrate / 5;
	}
	pos++;

	/* IEEE80211_RADIOTAP_CHANNEL */
	put_unaligned_le16(r92su->current_channel->center_freq, pos);
	pos += 2;
	if (ht)
		put_unaligned_le16(IEEE80211_CHAN_DYN | IEEE80211_CHAN_2GHZ,
				   pos);
	else if (rx_mcs > 3)
		put_unaligned_le16(IEEE80211_CHAN_OFDM | IEEE80211_CHAN_2GHZ,
				   pos);
	else
		put_unaligned_le16(IEEE80211_CHAN_CCK | IEEE80211_CHAN_2GHZ,
				   pos);
	pos += 2;

	/* IEEE80211_RADIOTAP_ANTENNA */
	*pos++ = 0;

	/* IEEE80211_RADIOTAP_RX_FLAGS */
	/* ensure 2 byte alignment for the 2 byte field as required */
	if ((pos - (u8 *)rthdr) & 1)
		*pos++ = 0;
	put_unaligned_le16(0, pos);
	pos += 2;

	if (ht) {
		rthdr->it_present |= cpu_to_le32(1 << IEEE80211_RADIOTAP_MCS);
		*pos++ = IEEE80211_RADIOTAP_MCS_HAVE_MCS |
			 IEEE80211_RADIOTAP_MCS_HAVE_BW;
		*pos = 0;
		if (GET_RX_DESC_BW(rx_hdr))
			*pos |= IEEE80211_RADIOTAP_MCS_BW_40;
		pos++;
		*pos++ = rx_mcs;
	}

	if (GET_RX_DESC_PAGGR(rx_hdr)) {
		u16 flags = 0;

		/* ensure 4 byte alignment */
		while ((pos - (u8 *)rthdr) & 3)
			pos++;

		rthdr->it_present |=
			cpu_to_le32(1 << IEEE80211_RADIOTAP_AMPDU_STATUS);

		if (GET_RX_DESC_FAGGR(rx_hdr))
			r92su->ampdu_reference++;

		put_unaligned_le32(r92su->ampdu_reference, pos);
		pos += 4;

		/* ampdu flags, no flags ? */
		put_unaligned_le16(flags, pos);
		pos += 2;

		*pos++ = 0;	/* no crc delim */
		*pos++ = 0;
	}
}

static unsigned int r92su_rx_calc_radiotap_len(struct r92su *r92su,
					       rx_hdr *rx_hdr)
{
	unsigned int rtap_len;

	/* Note: the order of the entries is important! */

	rtap_len = sizeof(struct ieee80211_radiotap_header);
	rtap_len += 9;

	/* TSF + Align */
	rtap_len = ALIGN(rtap_len, 8);
	rtap_len += 8;

	/* padding for RX_FLAGS if necessary */
	rtap_len = ALIGN(rtap_len, 2);

	if (GET_RX_DESC_RX_HT(rx_hdr))
		rtap_len += 3;

	if (GET_RX_DESC_PAGGR(rx_hdr)) {
		rtap_len = ALIGN(rtap_len, 4);
		rtap_len += 8;
	}

	return rtap_len;
}

static void r92su_rx_dropped(struct r92su *r92su, unsigned int count)
{
	r92su->wdev.netdev->stats.rx_dropped += count;
}

static void __r92su_rx_deliver(struct r92su *r92su, struct sk_buff *skb)
{
	int rc;

	skb_orphan(skb);

	if (in_interrupt())
		rc = netif_rx_ni(skb);
	else
		rc = netif_rx(skb);

	if (rc == NET_RX_SUCCESS) {
		skb->dev->stats.rx_packets++;
		skb->dev->stats.rx_bytes += skb->len;
	} else
		r92su_rx_dropped(r92su, 1);
}

static void r92su_rx_monitor(struct r92su *r92su, const struct rx_packet *rx,
			     const struct ieee80211_hdr *hdr,
			     struct sk_buff *skb)
{
	rx_hdr rx_hdr;
	unsigned int rtap_len;

	memcpy(&rx_hdr, &rx->hdr, sizeof(rx_hdr));

	rtap_len = r92su_rx_calc_radiotap_len(r92su, &rx_hdr);
	r92su_rx_add_radiotap(r92su, &rx_hdr, skb, rtap_len);

	skb_reset_mac_header(skb);
	skb->dev = r92su->wdev.netdev;
	skb->ip_summed = CHECKSUM_UNNECESSARY;
	skb->pkt_type = PACKET_OTHERHOST;
	skb->protocol = htons(ETH_P_802_2);
	__r92su_rx_deliver(r92su, skb);
}

static void r92su_rx_deliver(struct r92su *r92su, struct sk_buff *skb)
{
	skb_reset_mac_header(skb);
	skb->dev = r92su->wdev.netdev;
	skb->ip_summed = CHECKSUM_NONE;
	skb->protocol = eth_type_trans(skb, skb->dev);
	__r92su_rx_deliver(r92su, skb);
}

struct r92su_rx_info {
	struct r92su_sta *sta;
	struct r92su_key *key;
	u64 iv;
	bool has_protect;
	bool needs_decrypt;
};

enum r92su_rx_control_t {
	RX_CONTINUE,
	RX_QUEUE,
	RX_DROP,
};

static struct r92su_rx_info *r92su_get_rx_info(struct sk_buff *skb)
{
	BUILD_BUG_ON(sizeof(struct r92su_rx_info) > sizeof(skb->cb));
	return (struct r92su_rx_info *) skb->cb;
}

static inline u16 get_tid_h(struct ieee80211_hdr *hdr)
{
	return (ieee80211_get_qos_ctl(hdr))[0] & IEEE80211_QOS_CTL_TID_MASK;
}

static enum r92su_rx_control_t
r92su_rx_deduplicate(struct r92su *r92su, struct sk_buff *skb,
		     struct r92su_bss_priv *bss_priv)
{
	struct ieee80211_hdr *i3e = (void *)skb->data;
	struct r92su_rx_info *rx_info = r92su_get_rx_info(skb);
	struct r92su_sta *sta = rx_info->sta;
	__le16 *rx_seq;

	if (sta) {
		if (ieee80211_is_data_qos(i3e->frame_control))
			rx_seq = &sta->rx_seq_tid[get_tid_h(i3e)];
		else
			rx_seq = &sta->rx_seq;

		if (*rx_seq == i3e->seq_ctrl) {
			sta->drop_dup++;
			return RX_DROP;
		}

		*rx_seq = i3e->seq_ctrl;
	}
	return RX_CONTINUE;
}

static enum r92su_rx_control_t
r92su_rx_handle_mgmt(struct r92su *r92su, struct sk_buff *skb,
		     struct r92su_bss_priv *bss_priv)
{
	struct ieee80211_hdr *i3e = (void *)skb->data;

	if (ieee80211_is_mgmt(i3e->frame_control)) {
#if 0
		cfg80211_rx_mgmt(&r92su->wdev,
				 r92su->current_channel->center_freq, 0,
				 skb->data, skb->len, 0, GFP_ATOMIC);
#endif
		dev_kfree_skb_any(skb);
		return RX_QUEUE;
	}
	return RX_CONTINUE;
}

static enum r92su_rx_control_t
r92su_rx_find_sta(struct r92su *r92su, struct sk_buff *skb,
		  struct r92su_bss_priv *bss_priv)
{
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *) skb->data;
	struct r92su_sta *sta;
	struct r92su_rx_info *rx_info = r92su_get_rx_info(skb);

	sta = r92su_sta_get(r92su, ieee80211_get_SA(hdr));
	if (!sta)
		sta = bss_priv->sta;

	rx_info->sta = sta;
	return RX_CONTINUE;
}

static enum r92su_rx_control_t
r92su_rx_find_key(struct r92su *r92su, struct sk_buff *skb,
		  struct r92su_bss_priv *bss_priv)
{
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *) skb->data;
	struct r92su_rx_info *rx_info = r92su_get_rx_info(skb);
	struct r92su_key *key;

	if (rx_info->has_protect) {
		u8 *iv = r92su_get_i3e_payload(hdr);
		u8 idx = iv[3] >> 6 & 0x3;

		key = rcu_dereference(bss_priv->group_key[idx]);

		if (!key)
			key = rcu_dereference(rx_info->sta->sta_key);

		if (!key)
			key = rcu_dereference(bss_priv->sta->sta_key);

		if (!key)
			return RX_DROP;

		rx_info->key = key;
	} else {
		/* see r92su_rx_port_check */
	}

	return RX_CONTINUE;
}

static enum r92su_rx_control_t
r92su_rx_port_check(struct r92su *r92su, struct sk_buff *skb,
		 struct r92su_bss_priv *bss_priv)
{
	struct r92su_rx_info *rx_info = r92su_get_rx_info(skb);

	if (bss_priv->sta->enc_sta && !rx_info->has_protect) {
		if (bss_priv->control_port_no_encrypt &&
		    skb->protocol != bss_priv->control_port_ethertype) {
			return RX_DROP;
		}
	}
	return RX_CONTINUE;
}

static enum r92su_rx_control_t
r92su_rx_iv_handle(struct r92su *r92su, struct sk_buff *skb,
		   struct r92su_bss_priv *bss_priv)
{
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *) skb->data;
	struct r92su_rx_info *rx_info = r92su_get_rx_info(skb);
	struct r92su_key *key;
	unsigned int hdr_len = ieee80211_hdrlen(hdr->frame_control);
	unsigned int iv_len;
	u8 *iv;

	key = rx_info->key;

	if (!key)
		return RX_CONTINUE;

	iv_len = r92su_get_iv_len(key);
	iv = ((u8 *) hdr) + hdr_len;

	switch (key->type) {
	case WEP40_ENCRYPTION:
	case WEP104_ENCRYPTION: {
		u32 seq;

		seq = iv[0];
		seq = (seq << 8) + iv[1];
		seq = (seq << 8) + iv[2];
		rx_info->iv = seq;
		break;
	}

	case TKIP_ENCRYPTION: {
		u64 seq;

		/* check ext iv */
		if (!(iv[3] & BIT(5)))
			return RX_DROP;

		seq = iv[7];
		seq = (seq << 8) + iv[6];
		seq = (seq << 8) + iv[5];
		seq = (seq << 8) + iv[4];
		seq = (seq << 8) + iv[0];
		seq = (seq << 8) + iv[2];

		if (key->tkip.rx_seq >= seq)
			return RX_DROP;

		rx_info->iv = seq;
		break;
	}

	case AESCCMP_ENCRYPTION: {
		u64 seq;
		/* check ext iv */
		if (!(iv[3] & BIT(5)))
			return RX_DROP;

		seq = iv[7];
		seq = (seq << 8) + iv[6];
		seq = (seq << 8) + iv[5];
		seq = (seq << 8) + iv[4];
		seq = (seq << 8) + iv[1];
		seq = (seq << 8) + iv[0];
		if (seq <= key->ccmp.rx_seq)
			return RX_DROP;

		rx_info->iv = seq;
		break;
	}

	default:
		WARN(1, "invalid key type %d\n", key->type);
		return RX_DROP;
	}

	if (skb->len <= iv_len)
		return RX_DROP;

	memmove(((u8 *) hdr) + iv_len, hdr, hdr_len);
	skb_pull(skb, iv_len);
	return RX_CONTINUE;
}

static enum r92su_rx_control_t
r92su_rx_tkip_handle(struct r92su *r92su, struct sk_buff *skb,
		     struct r92su_key *key)
{
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *) skb->data;
	int data_len;
	int hdr_len = ieee80211_hdrlen(hdr->frame_control);
	void *tail, *data;
	u8 mic[MICHAEL_MIC_LEN];

	data = ((void *) hdr) + hdr_len;
	data_len = skb->len - hdr_len - IEEE80211_TKIP_ICV_LEN -
		   MICHAEL_MIC_LEN;
	if (data_len < 0)
		return RX_DROP;

	tail = data + data_len;
	michael_mic(&key->tkip.key.
		    key[NL80211_TKIP_DATA_OFFSET_RX_MIC_KEY],
		    hdr, data, data_len, mic);

	if (memcmp(mic, tail, MICHAEL_MIC_LEN) != 0) {
		cfg80211_michael_mic_failure(r92su->wdev.netdev,
			hdr->addr2,
			is_multicast_ether_addr(hdr->addr1) ?
			NL80211_KEYTYPE_GROUP :
			NL80211_KEYTYPE_PAIRWISE,
			key->index, NULL, GFP_ATOMIC);
		return RX_DROP;
	}

	return RX_CONTINUE;
}

static enum r92su_rx_control_t
r92su_rx_crypto_handle(struct r92su *r92su, struct sk_buff *skb,
		       struct r92su_bss_priv *bss_priv)
{
	struct r92su_rx_info *rx_info = r92su_get_rx_info(skb);
	struct r92su_key *key = rx_info->key;
	int remove_len;
	enum r92su_rx_control_t res;

	if (!key)
		return RX_CONTINUE;

	switch (key->type) {
	case WEP40_ENCRYPTION:
	case WEP104_ENCRYPTION:
		if (rx_info->needs_decrypt) {
			if (ieee80211_wep_decrypt(key->wep.tfm, skb,
						  key->wep.key, rx_info->iv,
						  key->key_len, key->index)) {
				return RX_DROP;
			}
		}

		key->wep.rx_seq = rx_info->iv;
		remove_len = IEEE80211_WEP_ICV_LEN;
		break;

	case TKIP_ENCRYPTION:
		if (rx_info->needs_decrypt) {
			if (ieee80211_tkip_decrypt_data(key->tkip.tfm,
							key->tkip.key._key.key,
							skb, rx_info->iv))
				return RX_DROP;
		}

		res = r92su_rx_tkip_handle(r92su, skb, key);
		if (res != RX_CONTINUE)
			return res;

		key->tkip.rx_seq = rx_info->iv;
		remove_len = IEEE80211_TKIP_ICV_LEN + MICHAEL_MIC_LEN;
		break;

	case AESCCMP_ENCRYPTION:
		if (rx_info->needs_decrypt) {
			if (ieee80211_aes_ccm_decrypt(key->ccmp.tfm, skb,
						      rx_info->iv,
						      IEEE80211_CCMP_MIC_LEN))
				return RX_DROP;
		}

		key->ccmp.rx_seq = rx_info->iv;
		remove_len = IEEE80211_CCMP_MIC_LEN;
		break;

	default:
		WARN(1, "invalid key type %d\n", key->type);
		return RX_DROP;
	}

	if (skb->len <= remove_len)
		return RX_DROP;

	skb_trim(skb, skb->len - remove_len);
	return RX_CONTINUE;
}

static enum r92su_rx_control_t
r92su_rx_data_to_8023(struct r92su *r92su, struct sk_buff *skb,
		      struct r92su_bss_priv *bss_priv, struct sk_buff **_skb,
		      struct sk_buff_head *queue)
{
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *) skb->data;
	bool is_amsdu = false;

	if (ieee80211_is_data_qos(hdr->frame_control)) {
		u8 *qos = ieee80211_get_qos_ctl(hdr);
		is_amsdu = !!(qos[0] & BIT(7));
	}

	if (is_amsdu) {
		struct r92su_rx_info tmp_rx_info = *r92su_get_rx_info(skb);
		struct ethhdr ethhdr;

		if (ieee80211_data_to_8023_exthdr(skb, &ethhdr,
		    wdev_address(&r92su->wdev), r92su->wdev.iftype))
			return RX_DROP;

		ieee80211_amsdu_to_8023s(skb, queue,
				       wdev_address(&r92su->wdev),
				       r92su->wdev.iftype, 0, NULL, NULL);
		*_skb = NULL;

		if (skb_queue_empty(queue)) {
			/* amsdu_to_8023s encountered an error. */
			return RX_DROP;
		}

		skb_queue_walk(queue, skb)
			*r92su_get_rx_info(skb) = tmp_rx_info;
	} else {
		int err;

		err = ieee80211_data_to_8023(skb, wdev_address(&r92su->wdev),
					     r92su->wdev.iftype);

		if (err)
			return RX_DROP;

		__skb_queue_tail(queue, skb);
		*_skb = NULL;
	}

	return RX_CONTINUE;
}

static u8 r92su_get_priority(struct r92su *r92su, struct sk_buff *skb)
{
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *) skb->data;
	if (ieee80211_is_data_qos(hdr->frame_control)) {
		u8 *qos = ieee80211_get_qos_ctl(hdr);
		skb->priority = qos[0] & IEEE80211_QOS_CTL_TAG1D_MASK;
	} else
		skb->priority = 0;

	return ieee802_1d_to_ac[skb->priority];
}

static enum r92su_rx_control_t
r92su_rx_hw_header_check(struct r92su *r92su, struct sk_buff *skb,
			 struct r92su_bss_priv *bss_priv,
			 const struct rx_packet *rx)
{
	struct r92su_rx_info *rx_info;
	struct ieee80211_hdr *hdr;
	unsigned int min_len;
	bool has_protect, needs_decrypt = false;

	if (skb->len < (sizeof(*hdr) + FCS_LEN))
		return RX_DROP;

	/* remove FCS - see comment in rx_filter about APPFCS */
	skb_trim(skb, skb->len - FCS_LEN);

	hdr = (struct ieee80211_hdr *) skb->data;

	/* filter out frames with bad fcs... if they did end up here */
	if (GET_RX_DESC_CRC32(&rx->hdr))
		return RX_DROP;

	has_protect = ieee80211_has_protected(hdr->frame_control);

	if (has_protect && GET_RX_DESC_SWDEC(&rx->hdr))
		needs_decrypt = true;

	/* TCP/IP checksum offloading needs to be tested and verified first.
	 * If you enable this code, don't forget to edit r92su_rx_deliver!
	 *
	 * if (GET_RX_DESC_TCP_CHK_VALID(&rx->hdr) &&
	 *     GET_RX_DESC_TCP_CHK_RPT(&rx->hdr) &&
	 *     GET_RX_DESC_IP_CHK_RPT(&rx->hdr))
	 *	skb->ip_summed = CHECKSUM_UNNECESSARY;
	 */

	/* report icv error
	 * The vendor driver ignores the flag, probably because someone
	 * "knew" that the hardware/firmware doesn't calculate the right
	 * icv in all cases (especially with fragments).
	 * if (rx->hdr.icv) {
	 *	return RX_DROP;
	 * }
	 */

	if (!(ieee80211_is_data_present(hdr->frame_control) ||
	     ieee80211_is_mgmt(hdr->frame_control)))
		return RX_DROP;

	/* just in case: clear out the whole skb->cb */
	memset(skb->cb, 0, sizeof(skb->cb));

	min_len = ieee80211_hdrlen(hdr->frame_control);
	if (skb->len < min_len)
		return RX_DROP;

	skb_set_queue_mapping(skb, r92su_get_priority(r92su, skb));

	rx_info = r92su_get_rx_info(skb);
	rx_info->has_protect = has_protect;
	rx_info->needs_decrypt = needs_decrypt;
	return RX_CONTINUE;
}

static enum r92su_rx_control_t
r92su_rx_sta_stats(struct r92su *r92su, struct sk_buff *skb,
		   struct r92su_bss_priv *bss_priv,
		   const struct rx_packet *rx)
{
	struct r92su_rx_info *rx_info = r92su_get_rx_info(skb);
	unsigned int rate = GET_RX_DESC_RX_MCS(&rx->hdr);
	enum rate_info_flags flag = 0;
	enum rate_info_bw bw;

	if (GET_RX_DESC_RX_HT(&rx->hdr)) {
		flag |= RATE_INFO_FLAGS_MCS;

		if (GET_RX_DESC_BW(&rx->hdr))
			bw = RATE_INFO_BW_40;
		else
			bw = RATE_INFO_BW_20;
	} else {
		rate = r92su->band_2GHZ.bitrates[rate].bitrate;
		bw = RATE_INFO_BW_20;
	}

	rx_info->sta->last_rx_rate_bw = bw;
	rx_info->sta->last_rx_rate_flag = flag;
	rx_info->sta->last_rx_rate = rate;
	return RX_CONTINUE;
}

static bool r92su_check_if_match(struct r92su *r92su,
				 struct sk_buff *new_skb,
				 struct sk_buff_head *defrag,
				 struct r92su_key *key)
{
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *) new_skb->data;
	struct ieee80211_hdr *tmp_hdr;
	struct sk_buff *tmp;
	u16 cur_seq, cur_frag, tmp_frag, tmp_seq;

	tmp = skb_peek_tail(defrag);
	if (!tmp)
		return false;

	cur_seq = (le16_to_cpu(hdr->seq_ctrl) & IEEE80211_SCTL_SEQ) >> 4;
	cur_frag = le16_to_cpu(hdr->seq_ctrl) & IEEE80211_SCTL_FRAG;

	tmp_hdr = (struct ieee80211_hdr *) tmp->data;
	tmp_frag = le16_to_cpu(tmp_hdr->seq_ctrl) & IEEE80211_SCTL_FRAG;
	tmp_seq = (le16_to_cpu(tmp_hdr->seq_ctrl) & IEEE80211_SCTL_SEQ) >> 4;
	if (tmp_seq != cur_seq && cur_frag != (1 + tmp_frag))
		return false;

	if (key && key->type == AESCCMP_ENCRYPTION) {
		struct r92su_rx_info *old_rx_info = r92su_get_rx_info(tmp);
		struct r92su_rx_info *new_rx_info = r92su_get_rx_info(new_skb);

		if (old_rx_info->iv + 1 != new_rx_info->iv)
			return false;
	}

	return true;
}

static void r92su_defrag_drop(struct r92su *r92su,
			      struct r92su_defrag_entry *defrag)
{
	r92su_rx_dropped(r92su, skb_queue_len(&defrag->queue));
	__skb_queue_purge(&defrag->queue);
	defrag->size = 0;
}

static void r92su_defrag_add(struct r92su *r92su,
			     struct r92su_defrag_entry *defrag,
			     struct sk_buff *skb)
{
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *) skb->data;
	int hdrlen = ieee80211_hdrlen(hdr->frame_control);

	memset(skb->cb, 0, sizeof(skb->cb));
	__skb_queue_tail(&defrag->queue, skb);
	defrag->size += skb->len - hdrlen;
}

static enum r92su_rx_control_t
r92su_rx_defrag(struct r92su *r92su, struct sk_buff *skb,
		struct r92su_bss_priv *bss_priv)
{
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *) skb->data;
	struct r92su_rx_info *rx_info = r92su_get_rx_info(skb);
	struct r92su_sta *sta = rx_info->sta;
	int hdrlen = ieee80211_hdrlen(hdr->frame_control);
	u16 qidx = skb_get_queue_mapping(skb);
	u16 cur_frag = le16_to_cpu(hdr->seq_ctrl) & IEEE80211_SCTL_FRAG;
	struct r92su_defrag_entry *defrag = &sta->defrag[qidx];
	struct sk_buff_head *dq = &defrag->queue;

	if (unlikely(ieee80211_has_morefrags(hdr->frame_control))) {
		if (cur_frag == 0)
			goto new_queued;

		if (!r92su_check_if_match(r92su, skb, dq, rx_info->key))
			goto dropped;
		else
			goto queued;
	} else {
		struct sk_buff *tmp;

		if (likely(cur_frag == 0)) {
			r92su_defrag_drop(r92su, defrag);
			return RX_CONTINUE;
		}

		if (!r92su_check_if_match(r92su, skb, dq, rx_info->key))
			goto dropped;

		defrag->size += skb->len - hdrlen;

		if (unlikely(pskb_expand_head(skb, defrag->size -
			     skb->len, 0, GFP_ATOMIC)))
			goto dropped;

		while ((tmp = __skb_dequeue_tail(dq))) {
			/* this also replaces the current ieee80211 header
			 * with the one queued in the defrag queue.
			 */
			hdr = (struct ieee80211_hdr *) skb->data;
			skb_pull(skb, ieee80211_hdrlen(hdr->frame_control));
			memcpy(skb_push(skb, tmp->len), tmp->data, tmp->len);
			dev_kfree_skb_any(tmp);
		}
		return RX_CONTINUE;
	}

new_queued:
	r92su_defrag_drop(r92su, defrag);
queued:
	r92su_defrag_add(r92su, defrag, skb);
	return RX_QUEUE;

dropped:
	r92su_defrag_drop(r92su, defrag);
	return RX_DROP;
}

static void
r92su_reorder_release_frame(struct r92su *r92su, struct r92su_rx_tid *tid,
			    int index, struct sk_buff_head *queue)
{
	struct sk_buff *skb = tid->reorder_buf[index];

	if (!skb)
		goto no_frame;

	tid->len--;
	tid->reorder_buf[index] = NULL;
	__skb_queue_tail(queue, skb);

no_frame:
	tid->head_seq = ieee80211_sn_inc(tid->head_seq);
}

static void
r92su_reorder_release_frames(struct r92su *r92su, struct r92su_rx_tid *tid,
			     u16 new_head_seq, struct sk_buff_head *queue)
{
	int index;

	while (ieee80211_sn_less(tid->head_seq, new_head_seq)) {
		index = ieee80211_sn_sub(tid->head_seq, tid->ssn) % tid->size;
		r92su_reorder_release_frame(r92su, tid, index, queue);
	}
}

#define HT_RX_REORDER_BUF_TIMEOUT (HZ / 10)

static void
r92su_reorder_sta_release(struct r92su *r92su, struct r92su_rx_tid *tid,
			  struct sk_buff_head *queue)
{
	int index, j;

	index = ieee80211_sn_sub(tid->head_seq, tid->ssn) % tid->size;
	if (tid->len && !tid->reorder_buf[index]) {
		int skipped = 1;
		for (j = (index + 1) % tid->size; j != index;
		     j = (j + 1) % tid->size) {
			if (!tid->reorder_buf[j]) {
				skipped++;
				continue;
			}

			if (skipped &&
			    time_is_after_jiffies(tid->reorder_time[j]))
				goto set_release_timer;

			tid->head_seq = ieee80211_sn_add(tid->head_seq,
							 skipped);
			r92su_reorder_release_frame(r92su, tid, j, queue);
			skipped = 0;
		}
	} else {
		while (tid->reorder_buf[index]) {
			r92su_reorder_release_frame(r92su, tid, index, queue);
			index = ieee80211_sn_sub(tid->head_seq, tid->ssn) %
				tid->size;
		}
	}

	if (tid->len) {
		j = index = ieee80211_sn_sub(tid->head_seq, tid->ssn) %
			tid->size;

		for (; j != (index - 1) % tid->size; j = (j + 1) % tid->size) {
			if (tid->reorder_buf[j])
				break;
		}
set_release_timer:
		mod_timer(&tid->reorder_timer, tid->reorder_time[j] + 1);
	} else {
		del_timer(&tid->reorder_timer);
	}
}

static enum r92su_rx_control_t
r92su_rx_reorder_ampdu(struct r92su *r92su, struct sk_buff *skb,
		       struct r92su_bss_priv *bss_priv,
		       struct sk_buff **_skb,
		       struct sk_buff_head *queue)
{
	struct ieee80211_hdr *hdr = (struct ieee80211_hdr *) skb->data;
	struct r92su_rx_info *rx_info = r92su_get_rx_info(skb);
	struct r92su_rx_tid *tid;
	u16 sc = le16_to_cpu(hdr->seq_ctrl);
	u16 mpdu_seq = (sc & IEEE80211_SCTL_SEQ) >> 4;
	u16 head_seq;
	int index;
	bool queue_frame = true;

	/* Currently, qos-nullfunc frames are not passed to the driver
	 * by the firmware. The qos-nullfunc check is actually not
	 * necessary.
	 */
	if (!ieee80211_is_data_qos(hdr->frame_control) ||
	    ieee80211_is_qos_nullfunc(hdr->frame_control))
		goto out;

	if (!rx_info->sta)
		goto out;

	tid = rcu_dereference(rx_info->sta->rx_tid[get_tid_h(hdr)]);
	if (!tid)
		goto out;

	spin_lock(&tid->lock);

	head_seq = tid->head_seq;

	/* frame with out of date sequence number */
	if (ieee80211_sn_less(mpdu_seq, head_seq))
		goto drop_unlock;

	if (!ieee80211_sn_less(mpdu_seq, head_seq + tid->size)) {
		head_seq = ieee80211_sn_inc(
			ieee80211_sn_sub(mpdu_seq, tid->size));


		r92su_reorder_release_frames(r92su, tid, head_seq, queue);
	}

	index = ieee80211_sn_sub(mpdu_seq, tid->ssn) % tid->size;

	if (tid->reorder_buf[index])
		goto drop_unlock;

	if (mpdu_seq == tid->head_seq && tid->len == 0) {
		tid->head_seq = ieee80211_sn_inc(mpdu_seq);
	} else {
		queue_frame = false;
		tid->reorder_buf[index] = skb;
		tid->reorder_time[index] = jiffies + HT_RX_REORDER_BUF_TIMEOUT;
		tid->len++;
		r92su_reorder_sta_release(r92su, tid, queue);
	}
	spin_unlock(&tid->lock);

out:
	if (queue_frame)
		__skb_queue_tail(queue, skb);
	*_skb = NULL;
	return RX_CONTINUE;

drop_unlock:
	spin_unlock(&tid->lock);
	return RX_DROP;
}

#define __RX_HANDLER(func, dropgoto, queuegoto, args...) do {	\
	enum r92su_rx_control_t __ret;				\
	__ret = func(r92su, skb, bss_priv, ## args);		\
	switch (__ret) {					\
	case RX_CONTINUE:					\
		break;						\
	case RX_QUEUE:						\
		goto queuegoto;					\
	case RX_DROP:						\
		goto dropgoto;					\
	}							\
} while (0)

static void r92su_rx_handler(struct r92su *r92su,
			     struct r92su_bss_priv *bss_priv,
			     struct sk_buff_head *queue)
{
	struct sk_buff_head frames;
	struct sk_buff *skb;

	__skb_queue_head_init(&frames);

#define RX_HANDLER_PREP(func, args...)				\
	__RX_HANDLER(func, rx_drop, out, ## args)

#define RX_HANDLER_MAIN(func, args...)				\
	__RX_HANDLER(func, rx_drop_main, queued_main, ## args)

	spin_lock(&r92su->rx_path);

	while ((skb = __skb_dequeue(queue))) {
		RX_HANDLER_PREP(r92su_rx_find_key);
		RX_HANDLER_PREP(r92su_rx_iv_handle);
		RX_HANDLER_PREP(r92su_rx_crypto_handle);
		RX_HANDLER_PREP(r92su_rx_defrag);
		RX_HANDLER_PREP(r92su_rx_data_to_8023, &skb, &frames);
out:
		continue;
rx_drop:
		r92su_rx_dropped(r92su, 1);
		dev_kfree_skb_any(skb);
	}

	while ((skb = __skb_dequeue(&frames))) {
		RX_HANDLER_MAIN(r92su_rx_port_check);
		r92su_rx_deliver(r92su, skb);
queued_main:
		continue;

rx_drop_main:
		r92su_rx_dropped(r92su, 1);
		dev_kfree_skb_any(skb);
	}

	spin_unlock(&r92su->rx_path);
#undef RX_HANDLER_PREP
#undef RX_HANDLER_MAIN
}

static void r92su_rx_data(struct r92su *r92su,
	const struct rx_packet *rx, struct ieee80211_hdr *hdr,
	struct sk_buff *skb)
{
	struct sk_buff_head frames;
	struct cfg80211_bss *bss;
	struct r92su_bss_priv *bss_priv;

#define RX_HANDLER(func, args...)			\
	__RX_HANDLER(func, rx_drop, out, ## args)

	__skb_queue_head_init(&frames);

	rcu_read_lock();
	if (!r92su_is_connected(r92su))
		goto rx_drop;

	bss = rcu_dereference(r92su->connect_bss);
	if (!bss)
		goto rx_drop;

	bss_priv = r92su_get_bss_priv(bss);
	RX_HANDLER(r92su_rx_hw_header_check, rx);
	RX_HANDLER(r92su_rx_find_sta);
	RX_HANDLER(r92su_rx_deduplicate);
	RX_HANDLER(r92su_rx_sta_stats, rx);
	RX_HANDLER(r92su_rx_handle_mgmt);

	/* this moves the frame onto the in_frames queue */
	RX_HANDLER(r92su_rx_reorder_ampdu, &skb, &frames);

	r92su_rx_handler(r92su, bss_priv, &frames);
out:
	rcu_read_unlock();
	return;

rx_drop:
	rcu_read_unlock();

	r92su_rx_dropped(r92su, 1);
	__skb_queue_purge(&frames);
	dev_kfree_skb_any(skb);

#undef __RX_HANDLER
#undef RX_HANDLER
}

void r92su_reorder_tid_timer(unsigned long arg)
{
	struct sk_buff_head frames;
	struct r92su_rx_tid *tid;
	struct r92su *r92su;
	struct r92su_sta *sta;
	struct cfg80211_bss *bss;
	struct r92su_bss_priv *bss_priv;

	__skb_queue_head_init(&frames);

	rcu_read_lock();
	tid = (struct r92su_rx_tid *) arg;
	r92su = tid->r92su;
	sta = tid->sta;

	if (!r92su_is_connected(r92su))
		goto out;

	bss = rcu_dereference(r92su->connect_bss);
	if (!bss || !sta)
		goto out;

	bss_priv = r92su_get_bss_priv(bss);

	spin_lock(&tid->lock);
	r92su_reorder_sta_release(r92su, tid, &frames);
	spin_unlock(&tid->lock);

	r92su_rx_handler(r92su, bss_priv, &frames);

out:
	rcu_read_unlock();
}

static struct sk_buff *rx92su_rx_copy_data(
	const struct rx_packet *rx, unsigned int rx_len,
	struct ieee80211_hdr *frame, unsigned int frame_len)
{
	struct sk_buff *skb;
	int reserved = 0;

	if (ieee80211_is_data_qos(frame->frame_control)) {
		u8 *qc = ieee80211_get_qos_ctl(frame);
		reserved += NET_IP_ALIGN;

		if (*qc & IEEE80211_QOS_CTL_A_MSDU_PRESENT)
			reserved += NET_IP_ALIGN;
	}

	if (ieee80211_has_a4(frame->frame_control))
		reserved += NET_IP_ALIGN;

	reserved = 32 + (reserved & NET_IP_ALIGN);

	skb = dev_alloc_skb(rx_len + frame_len + reserved);
	if (likely(skb)) {
		skb_reserve(skb, reserved);
		memcpy(skb_put(skb, rx_len), rx, rx_len);
		memcpy(skb_put(skb, frame_len), frame, frame_len);
	}
	return skb;
}

static void r92su_rx_tasklet(unsigned long arg0)
{
	struct r92su *r92su = (struct r92su *) arg0;
	struct sk_buff *skb;

	while ((skb = skb_dequeue(&r92su->rx_queue))) {
		struct rx_packet *rx;
		struct ieee80211_hdr *hdr;
		unsigned int drvinfo_size;

		trace_r92su_rx_data(wiphy_dev(r92su->wdev.wiphy), skb);

		rx = (struct rx_packet *) skb->data;
		drvinfo_size = GET_RX_DESC_DRVINFO_SIZE(&rx->hdr);
		hdr = (struct ieee80211_hdr *) skb_pull(skb,
			RX_DESC_SIZE + drvinfo_size * 8);
		switch (r92su->wdev.iftype) {
		case NL80211_IFTYPE_MONITOR:
			r92su_rx_monitor(r92su, rx, hdr, skb);
			break;
		default:
			r92su_rx_data(r92su, rx, hdr, skb);
			break;
		}
	}
}

void r92su_rx_init(struct r92su *r92su)
{
	skb_queue_head_init(&r92su->rx_queue);
	tasklet_init(&r92su->rx_tasklet, r92su_rx_tasklet,
		     (unsigned long) r92su);
}

void r92su_rx_deinit(struct r92su *r92su)
{
	tasklet_kill(&r92su->rx_tasklet);
	skb_queue_purge(&r92su->rx_queue);
}

void r92su_rx(struct r92su *r92su, void *buf, const unsigned int len)
{
	struct rx_packet *rx = (struct rx_packet *) buf;
	void *end;
	unsigned int pkt_cnt, pkt_len, hdr_len;

	end = ((void *) buf) + min_t(unsigned int, len,
		RTL92SU_SIZE_MAX_RX_BUFFER - RX_DESC_SIZE);

	/* pkt_cnt seems to be valid only for the first aggregated packet?! */
	pkt_cnt = GET_RX_DESC_PKTCNT(rx->hdr);
	pkt_cnt = max_t(unsigned int, pkt_cnt, 1);

	while (buf < end && pkt_cnt--) {
		unsigned int drvinfo, shift;
		rx = (struct rx_packet *) buf;

		drvinfo = GET_RX_DESC_DRVINFO_SIZE(&rx->hdr) *
			RX_DRV_INFO_SIZE_UNIT;
		shift = GET_RX_DESC_SHIFT(&rx->hdr);
		pkt_len = GET_RX_DESC_PKT_LEN(&rx->hdr);
		hdr_len = RX_DESC_SIZE + drvinfo;

		if (buf + pkt_len + hdr_len + shift > end)
			goto err_garbage;

		if (GET_RX_DESC_IS_CMD(&rx->hdr)) {
			if (len - sizeof(rx->hdr) <
			    le16_to_cpu(rx->c2h.len) + sizeof(rx->c2h)) {
				R92SU_ERR(r92su,
					"received clipped c2h command.");
				r92su_mark_dead(r92su);
			} else
				r92su_c2h_event(r92su, &rx->c2h);
		} else {
			struct ieee80211_hdr *i3e;
			struct sk_buff *skb;

			if (!r92su_is_connected(r92su))
				continue;

			i3e = ((void *) buf) + hdr_len + shift;
			skb = rx92su_rx_copy_data(rx, hdr_len, i3e, pkt_len);
			if (skb)
				skb_queue_tail(&r92su->rx_queue, skb);
			else
				r92su_rx_dropped(r92su, 1);
		}
		buf += ALIGN(hdr_len + pkt_len, r92su->rx_alignment);
	}
	if (r92su_is_connected(r92su))
		tasklet_schedule(&r92su->rx_tasklet);
	return;

err_garbage:
	R92SU_ERR(r92su, "received clipped frame.");
	return;
}

u8 *r92su_find_ie(u8 *ies, const u32 len, const u8 ie)
{
	u8 *pos, *end;

	pos = ies;
	end = ies + len;
	while (pos < end) {
		if (pos + 2 + pos[1] > end)
			return NULL;

		if (pos[0] == ie)
			return pos;

		pos += 2 + pos[1];
	}
	return NULL;
}
