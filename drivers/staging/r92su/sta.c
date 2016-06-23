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
#include <linux/workqueue.h>
#include <linux/random.h>
#include "r92su.h"
#include "aes_ccm.h"
#include "tkip.h"
#include "sta.h"
#include "wep.h"
#include "rx.h"

static void r92su_free_tid_rcu(struct rcu_head *head)
{
	struct r92su_rx_tid *tid = container_of(head, struct r92su_rx_tid,
						rcu_head);
	int i;

	del_timer_sync(&tid->reorder_timer);

	spin_lock_bh(&tid->lock);
	tid->len = 0;
	for (i = 0; i < ARRAY_SIZE(tid->reorder_buf); i++) {
		if (tid->reorder_buf[i]) {
			dev_kfree_skb_any(tid->reorder_buf[i]);
			tid->reorder_buf[i] = NULL;
		}
	}
	spin_unlock_bh(&tid->lock);
	kfree_rcu(tid, rcu_head);
}

static void r92su_free_tid(struct r92su_rx_tid *tid)
{
	if (tid) {
		tid->sta = NULL;
		call_rcu(&tid->rcu_head, r92su_free_tid_rcu);
	}
}

static void r92su_free_sta_rcu(struct rcu_head *rch)
{
	struct r92su_sta *sta = container_of(rch, struct r92su_sta, rcu_head);
	struct r92su_key *key;
	int i;

	for (i = 0; i < ARRAY_SIZE(sta->defrag); i++)
		skb_queue_purge(&sta->defrag[i].queue);

	rcu_read_lock();
	for (i = 0; i < ARRAY_SIZE(sta->rx_tid); i++) {
		struct r92su_rx_tid *rx_tid;
		rx_tid = rcu_dereference(sta->rx_tid[i]);
		rcu_assign_pointer(sta->rx_tid[i], NULL);
		r92su_free_tid(rx_tid);
	}
	key = rcu_dereference(sta->sta_key);
	rcu_assign_pointer(sta->sta_key, NULL);
	r92su_key_free(key);

	rcu_read_unlock();
	kfree(sta);
}

void r92su_sta_alloc_tid(struct r92su *r92su,
			 struct r92su_sta *sta,
			 const u8 tid, u16 ssn)
{
	struct r92su_rx_tid *new_tid;
	struct r92su_rx_tid *old_tid;

	new_tid = kzalloc(sizeof(*new_tid), GFP_ATOMIC);

	rcu_read_lock();
	old_tid = rcu_dereference(sta->rx_tid[tid]);
	if (new_tid) {
		spin_lock_init(&new_tid->lock);
		new_tid->tid = tid;

		setup_timer(&new_tid->reorder_timer,
			    r92su_reorder_tid_timer,
		    (unsigned long) new_tid);
		new_tid->r92su = r92su;
		new_tid->sta = sta;
		new_tid->head_seq = new_tid->ssn = ssn >> 4;
		new_tid->size = 32;	 /* taken from a monitor */

		rcu_assign_pointer(sta->rx_tid[tid], new_tid);
	} else {
		rcu_assign_pointer(sta->rx_tid[tid], NULL);
	}
	r92su_free_tid(old_tid);
	rcu_read_unlock();
}

struct r92su_sta *r92su_sta_alloc(struct r92su *r92su, const u8 *mac_addr,
				  const unsigned int mac_id,
				  const unsigned int aid, const gfp_t flag)
{
	struct r92su_sta *sta;

	sta = kzalloc(sizeof(*sta), flag);
	if (sta) {
		unsigned long flags;
		struct timespec uptime;
		int i;

		for (i = 0; i < ARRAY_SIZE(sta->defrag); i++)
			skb_queue_head_init(&sta->defrag[i].queue);

		if (mac_addr)
			memcpy(sta->mac_addr, mac_addr, ETH_ALEN);
		sta->mac_id = mac_id;
		sta->aid = aid;

		ktime_get_ts(&uptime);
		sta->last_connected = uptime.tv_sec;

		/* Remove the old station */
		r92su_sta_del(r92su, mac_id);

		spin_lock_irqsave(&r92su->sta_lock, flags);
		list_add_rcu(&sta->list, &r92su->sta_list);
		r92su->sta_generation++;
		r92su->sta_num++;
		spin_unlock_irqrestore(&r92su->sta_lock, flags);
	}
	return sta;
}

struct r92su_sta *r92su_sta_get(struct r92su *r92su, const u8 *mac_addr)
{
	struct r92su_sta *sta;

	list_for_each_entry_rcu(sta, &r92su->sta_list, list) {
		if (memcmp(sta->mac_addr, mac_addr, ETH_ALEN) == 0)
			return sta;
	}

	return NULL;
}

void r92su_sta_del(struct r92su *r92su, int mac_id)
{
	struct r92su_sta *sta;
	unsigned long flags;

	rcu_read_lock();
	sta = r92su_sta_get_by_macid(r92su, mac_id);
	if (sta) {
		spin_lock_irqsave(&r92su->sta_lock, flags);
		list_del_rcu(&sta->list);
		r92su->sta_generation++;
		r92su->sta_num--;
		spin_unlock_irqrestore(&r92su->sta_lock, flags);

		call_rcu(&sta->rcu_head, r92su_free_sta_rcu);
	}
	rcu_read_unlock();
}

static u32 get_random_wep_seq(void)
{
	u32 buf;
	get_random_bytes(&buf, sizeof(buf));
	return buf;
}

struct r92su_key *r92su_key_alloc(const u32 cipher, const u8 idx,
				  const u8 *mac_addr, const bool pairwise,
				  const u8 *key)
{
	struct r92su_key *new_key;

	new_key = kzalloc(sizeof(*new_key), GFP_KERNEL);
	if (!new_key)
		return ERR_PTR(-ENOMEM);

	new_key->pairwise = pairwise;
	new_key->index = idx;
	if (mac_addr)
		memcpy(new_key->mac_addr, mac_addr, ETH_ALEN);

	switch (cipher) {
	case WLAN_CIPHER_SUITE_WEP40:
	case WLAN_CIPHER_SUITE_WEP104:
		if (cipher == WLAN_CIPHER_SUITE_WEP40) {
			new_key->type = WEP40_ENCRYPTION;
			new_key->key_len = WLAN_KEY_LEN_WEP40;
		} else {
			new_key->type = WEP104_ENCRYPTION;
			new_key->key_len = WLAN_KEY_LEN_WEP104;
		}

		new_key->wep.tx_seq = get_random_wep_seq();
		new_key->wep.rx_seq = 0;
		memcpy(new_key->wep.wep40_key, key, new_key->key_len);
		new_key->wep.tfm = ieee80211_wep_init();
		if (IS_ERR(new_key->wep.tfm)) {
			kfree(new_key);
			return ERR_PTR(PTR_ERR(new_key->wep.tfm));
		}
		break;

	case WLAN_CIPHER_SUITE_TKIP:
		new_key->type = TKIP_ENCRYPTION;
		new_key->key_len = WLAN_KEY_LEN_TKIP;
		new_key->tkip.tx_seq = 1;
		new_key->tkip.rx_seq = 1;
		memcpy(new_key->tkip.key.key, key,
		       sizeof(new_key->tkip.key.key));
		new_key->tkip.tfm = ieee80211_wep_init();
		if (IS_ERR(new_key->tkip.tfm)) {
			kfree(new_key);
			return ERR_PTR(PTR_ERR(new_key->tkip.tfm));
		}
		break;

	case WLAN_CIPHER_SUITE_CCMP:
		new_key->type = AESCCMP_ENCRYPTION;
		new_key->key_len = WLAN_KEY_LEN_CCMP;
		new_key->ccmp.tx_seq = 0;
		new_key->ccmp.rx_seq = 0;
		memcpy(new_key->ccmp.key, key, sizeof(new_key->ccmp.key));
		new_key->ccmp.tfm = ieee80211_aes_key_setup_encrypt(key,
			WLAN_KEY_LEN_CCMP, IEEE80211_CCMP_MIC_LEN);
		if (IS_ERR(new_key->ccmp.tfm)) {
			kfree(new_key);
			return ERR_PTR(PTR_ERR(new_key->ccmp.tfm));
		}
		break;

	default:
		WARN(1, "invalid cipher suite 0x%x\n", cipher);
		kfree(new_key);
		return ERR_PTR(-EINVAL);
	}
	return new_key;
}

void r92su_key_free(struct r92su_key *key)
{
	if (key) {
		/* WARN(key->uploaded, "Delete %s key of type %d in slot %d which is still uploaded in hw",
		 *	key->pairwise ? "pairwise" : "", key->type, key->index);
		 */
		switch (key->type) {
		case WEP40_ENCRYPTION:
		case WEP104_ENCRYPTION:
			ieee80211_wep_free(key->wep.tfm);
			break;
		case TKIP_ENCRYPTION:
			ieee80211_wep_free(key->tkip.tfm);
			break;
		case AESCCMP_ENCRYPTION:
			ieee80211_aes_key_free(key->ccmp.tfm);
			break;
		default:
			break;
		}
		kfree_rcu(key, rcu_head);
	}
}

void r92su_sta_set_sinfo(struct r92su *r92su, struct r92su_sta *sta,
			 struct station_info *sinfo)
{
	struct timespec uptime;
	struct cfg80211_bss *bss;
	struct r92su_bss_priv *bss_priv;
	sinfo->generation = r92su->sta_generation;
	sinfo->filled = BIT(NL80211_STA_INFO_CONNECTED_TIME) |
			BIT(NL80211_STA_INFO_RX_BITRATE) |
			BIT(NL80211_STA_INFO_BSS_PARAM) |
			BIT(NL80211_STA_INFO_STA_FLAGS) |
			BIT(NL80211_STA_INFO_SIGNAL);

	ktime_get_ts(&uptime);
	sinfo->connected_time = uptime.tv_sec - sta->last_connected;
	sinfo->signal = sta->signal;
	sinfo->rxrate.flags = sta->last_rx_rate_flag;
	sinfo->rxrate.bw = sta->last_rx_rate_bw;
	if (sta->last_rx_rate_flag & RATE_INFO_FLAGS_MCS)
		sinfo->rxrate.mcs = sta->last_rx_rate;
	else
		sinfo->rxrate.legacy = sta->last_rx_rate;

	sinfo->sta_flags.mask = BIT(NL80211_STA_FLAG_ASSOCIATED) |
				BIT(NL80211_STA_FLAG_AUTHENTICATED) |
				BIT(NL80211_STA_FLAG_WME);

	sinfo->sta_flags.set = BIT(NL80211_STA_FLAG_ASSOCIATED) |
			       BIT(NL80211_STA_FLAG_AUTHENTICATED);

	if (sta->qos_sta)
		sinfo->sta_flags.set |= BIT(NL80211_STA_FLAG_WME);

	sinfo->bss_param.flags = 0;
	rcu_read_lock();
	bss = rcu_dereference(r92su->connect_bss);
	if (bss) {
		bss_priv = r92su_get_bss_priv(bss);
		if (bss->capability & WLAN_CAPABILITY_SHORT_PREAMBLE)
			sinfo->bss_param.flags |= BSS_PARAM_FLAGS_SHORT_PREAMBLE;
		if (bss->capability & WLAN_CAPABILITY_SHORT_SLOT_TIME)
			sinfo->bss_param.flags |= BSS_PARAM_FLAGS_SHORT_SLOT_TIME;
		sinfo->bss_param.dtim_period = bss_priv->dtim_period;
		sinfo->bss_param.beacon_interval = bss->beacon_interval;
	}
	rcu_read_unlock();
}

struct r92su_sta *r92su_sta_get_by_idx(struct r92su *r92su, int idx)
{
	struct r92su_sta *sta;
	int i = 0;

	list_for_each_entry_rcu(sta, &r92su->sta_list, list) {
		if (i < idx) {
			i++;
			continue;
		}
		return sta;
	}

	return NULL;
}

struct r92su_sta *r92su_sta_get_by_macid(struct r92su *r92su, int mac_id)
{
	struct r92su_sta *sta;

	list_for_each_entry_rcu(sta, &r92su->sta_list, list) {
		if (sta->mac_id == mac_id)
			return sta;
	}

	return NULL;
}
