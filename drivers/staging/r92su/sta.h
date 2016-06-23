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
#ifndef __R92SU_STA_H__
#define __R92SU_STA_H__

#include <linux/kernel.h>
#include <linux/rcupdate.h>
#include <linux/skbuff.h>
#include <linux/timer.h>

#include "def.h"

struct crypto_aead;
struct r92su;
struct r92su_key;
struct r92su_sta;

/* The MACID field in the tx and rx headers are 5 bits long.
 * Therefore, the driver can manage 2^5 = 32 stations.
 */
#define MAX_STA	32

struct r92su_rx_tid {
	struct rcu_head rcu_head;
	spinlock_t lock;

	u16 dropped;

	u16 tid;
	u16 ssn;
	u16 head_seq;
	u16 len;
	u16 size;
	struct sk_buff *reorder_buf[64];
	unsigned long reorder_time[64];

	struct r92su *r92su;
	struct r92su_sta *sta;
	struct timer_list reorder_timer;
};

struct r92su_key {
	struct rcu_head rcu_head;

	u8 mac_addr[ETH_ALEN];
	enum r92su_enc_alg type;
	unsigned int key_len;
	bool uploaded;
	bool pairwise;
	unsigned int index;
	union {
		struct {
			u64 tx_seq:48;
			u64 rx_seq:48;
			u8 key[WLAN_KEY_LEN_CCMP];
			struct crypto_aead *tfm;
		} ccmp;

		struct {
			u64 tx_seq:48;
			u64 rx_seq:48;
			union {
				struct {
					u8 key[16];
					u8 mic1[8];
					u8 mic2[8];
				} __packed _key;
				u8 key[WLAN_KEY_LEN_TKIP];
			} key;
			struct crypto_cipher *tfm;
		} tkip;

		struct {
			u32 tx_seq:24;
			u32 rx_seq:24;
			union {
				u8 wep104_key[WLAN_KEY_LEN_WEP104];
				u8 wep40_key[WLAN_KEY_LEN_WEP40];
				u8 key[WLAN_KEY_LEN_WEP104];
			};
			struct crypto_cipher *tfm;
		} wep;
	};
};

struct r92su_defrag_entry {
	struct sk_buff_head queue;
	unsigned int size;
};

struct r92su_sta {
	struct rcu_head rcu_head;
	struct list_head list;

	u8 mac_addr[ETH_ALEN];
	unsigned int mac_id;
	unsigned int tx_seq;
	unsigned int tx_seq_tid[IEEE80211_NUM_TIDS];
	unsigned int aid;

	bool qos_sta;
	bool ht_sta;
	bool enc_sta;

	long last_connected;

	int signal;

	u32 last_rx_rate;
	enum rate_info_flags last_rx_rate_flag;
	enum rate_info_bw last_rx_rate_bw;

	/* deduplication */
	__le16 rx_seq;
	__le16 rx_seq_tid[IEEE80211_NUM_TIDS];

	struct r92su_rx_tid __rcu *rx_tid[IEEE80211_NUM_TIDS];

	u64 drop_dup;

	struct r92su_defrag_entry defrag[NUM_ACS];

	struct r92su_key __rcu *sta_key;
};

struct r92su_sta *r92su_sta_alloc(struct r92su *r92su, const u8 *mac_addr,
				  const unsigned int mac_id,
				  const unsigned int aid,
				  const gfp_t flag);

void r92su_sta_alloc_tid(struct r92su *r92su, struct r92su_sta *sta,
			 const u8 tid, u16 size);

struct r92su_key *r92su_key_alloc(const u32 cipher, const u8 idx,
				  const u8 *mac_addr, const bool pairwise,
				  const u8 *key);

void r92su_key_free(struct r92su_key *key);

void r92su_sta_set_sinfo(struct r92su *r92su, struct r92su_sta *sta,
			 struct station_info *sinfo);

/* the following functions need rcu_read_lock! */
struct r92su_sta *r92su_sta_get(struct r92su *r92su, const u8 *mac_addr);
struct r92su_sta *r92su_sta_get_by_macid(struct r92su *r92su, int macid);
struct r92su_sta *r92su_sta_get_by_idx(struct r92su *r92su, int idx);

void r92su_sta_del(struct r92su *r92su, int mac_id);

#endif /* __R92SU_STA_H__ */
