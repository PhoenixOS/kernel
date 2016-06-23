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
#ifndef __R92SU_R92SU_H__
#define __R92SU_R92SU_H__

#include <linux/mii.h>
#include <linux/skbuff.h>
#include <linux/usb.h>
#include <linux/workqueue.h>

#include <linux/input.h>
#include <linux/ieee80211.h>
#include <linux/rcupdate.h>
#include <linux/mutex.h>
#include <net/cfg80211.h>
#include <linux/llist.h>

#include "h2cc2h.h"
#include "eeprom.h"
#include "fw.h"
#include "debugfs.h"
#include "def.h"
#include "sta.h"

#define R92SU_DRVNAME		"r92su"
#define RTL8192SU_FIRMWARE	"rtlwifi/rtl8712u.bin"

#define R92SU_TX_HEAD_ROOM					\
	(TX_DESC_SIZE + 4 /* HW/FW tx descriptor + align */	\
	 + 4 * ETH_ALEN /* DA/SA/BSSID/WDS mac addrs */		\
	 + 2 + 2 + 2 + 2 /* fc, seq, dur, qos */		\
	 + 8 + 8 /* rfc1042/bridge + (Ext) iv */		\
	 - ETH_HLEN /* Ethernet header gets replaced */)

#define R92SU_TX_TAIL_ROOM					\
	(8 /* TKIP & CCMP MIC */ + 4 /* ICV */)

enum r92su_state_t {
	R92SU_DEAD,	/* unresponsive */
	R92SU_UNLOAD,	/* unalloc */
	R92SU_PROBE,	/* when starting */
	R92SU_STOP,	/* when stopped */
	R92SU_INIT,	/* before ndo_open */
	R92SU_OPEN,	/* after ndo_open */
	R92SU_CONNECTED,/* Either connected to a AP or running an IBSS */
};

enum r92su_chip_revision_t {
	R92SU_FPGA  = 0,
	R92SU_A_CUT = 1,
	R92SU_B_CUT = 2,
	R92SU_C_CUT = 3,

	__R92SU_MAX_REV
};

enum r92su_rf_type_t {
	R92SU_1T1R = 0x11,
	R92SU_1T2R = 0x12,
	R92SU_2T2R = 0x22
};

enum rtl8712_queues_t {
	RTL8712_BKQ = 1,
	RTL8712_BEQ = 3,
	RTL8712_VIQ = 5,
	RTL8712_VOQ = 7,

	RTL8712_H2CCMD,
	RTL8712_BCNQ,
	RTL8712_BMCQ,
	RTL8712_MGTQ,

	RTL8712_RX0FF,
	RTL8712_C2HCMD,
	__RTL8712_LAST
};

enum ieee80211_ac_numbers {
	IEEE80211_AC_VO         = 0,
	IEEE80211_AC_VI         = 1,
	IEEE80211_AC_BE         = 2,
	IEEE80211_AC_BK         = 3,
};

static const int ieee802_1d_to_ac[8] = {
	IEEE80211_AC_BE,
	IEEE80211_AC_BK,
	IEEE80211_AC_BK,
	IEEE80211_AC_BE,
	IEEE80211_AC_VI,
	IEEE80211_AC_VI,
	IEEE80211_AC_VO,
	IEEE80211_AC_VO
};

typedef u32 ep_map[__RTL8712_LAST];

struct r92su {
	/* usb */
	struct usb_interface *intf;
	struct usb_device *udev;
	struct usb_anchor rx_submitted;
	struct usb_anchor tx_wait;
	struct usb_anchor tx_submitted;
	atomic_t tx_pending_urbs;
	unsigned int ep_num;
	const ep_map *ep_map;

	/* general */
	struct mutex lock;
	struct workqueue_struct *wq;
	enum r92su_state_t state;
	struct wireless_dev wdev;
	struct ieee80211_channel *current_channel;
	struct delayed_work service_work;

	/* cmd */
	unsigned int h2c_seq:7;
	unsigned int c2h_seq:7;
	spinlock_t tx_cmd_lock;

	/* rx */
	spinlock_t rx_path;
	unsigned int ampdu_reference;
	struct sk_buff_head rx_queue;
	struct tasklet_struct rx_tasklet;
	unsigned int rx_alignment;

	/* sta + keys */
	unsigned int sta_generation;
	unsigned int sta_num;
	struct list_head sta_list;
	spinlock_t sta_lock;

	/* cfg80211 info */
	struct ieee80211_supported_band band_2GHZ;

	/* scan / site survey */
	struct cfg80211_scan_request *scan_request;
	struct c2h_join_bss_event *connect_result;
	struct cfg80211_bss *wep_shared_bss;
	struct cfg80211_bss *want_connect_bss;
	struct cfg80211_bss __rcu *connect_bss;
	struct work_struct connect_bss_work;
	struct delayed_work survey_done_work;
	struct completion scan_done;
	struct llist_head add_bss_list;
	struct work_struct add_bss_work;
	struct work_struct disconnect_work;
	bool scanned;

	/* eeprom / hw_info */
	struct r92su_eeprom eeprom;
	enum r92su_eeprom_type eeprom_type;
	enum r92su_chip_revision_t chip_rev;
	enum r92su_rf_type_t rf_type;

	/* fw */
	const struct firmware *fw;
	const struct fw_hdr *fw_header;
	const void *fw_imem;
	const void *fw_sram;
	u32 fw_imem_len;
	u32 fw_sram_len;
	u16 fw_version;
	struct fw_priv fw_dmem;
	bool fw_loaded;
	bool disable_ht;

	/* wps pbc button */
	struct input_dev *wps_pbc;
	bool wps_pbc_state;

	/* power save */
	u8 rpwm;		/* requested power state for fw */
	u8 rpwm_tog;		/* toggling - keeps track if "rpwm"
				 * was updated */

	u8 cpwm;		/* fw current power state. updated when
				 * 1. read from HCPWM
				 * 2. driver lowers power level
				 */

	u8 cpwm_tog;		/* toggling - keeps track if "cpwm"
				 * was updated */

	/* debug */
	struct dentry *dfs;
	struct r92su_debug debug;
};

struct r92su_add_bss {
	struct llist_node head;
	struct h2cc2h_bss fw_bss;
};

struct r92su_tx_tid {
	bool addba_issued;
	struct sk_buff_head agg_queue;
};

struct r92su_bss_priv {
	struct h2cc2h_bss fw_bss;
	struct r92su_sta *sta;

	/* only valid during connect */
	u8 *assoc_ie;
	unsigned int assoc_ie_len;

	int def_uni_key_idx;
	int def_multi_key_idx;
	struct r92su_key __rcu *group_key[4];

	struct r92su_tx_tid tx_tid[IEEE80211_NUM_TIDS];

	__be16 control_port_ethertype;
	bool control_port_no_encrypt;
	bool control_port;

	unsigned int dtim_period;
};

static inline bool r92su_is_dead(struct r92su *r92su)
{
	return r92su->state == R92SU_DEAD;
}

static inline bool r92su_is_probing(struct r92su *r92su)
{
	return r92su->state >= R92SU_PROBE;
}

static inline bool r92su_is_initializing(struct r92su *r92su)
{
	return r92su->state >= R92SU_INIT;
}

static inline bool r92su_is_stopped(struct r92su *r92su)
{
	return r92su->state == R92SU_STOP;
}

static inline bool r92su_is_open(struct r92su *r92su)
{
	return r92su->state >= R92SU_OPEN;
}

static inline bool r92su_is_connected(struct r92su *r92su)
{
	return r92su->state >= R92SU_CONNECTED;
}

static inline void r92su_set_state(struct r92su *r92su,
				   enum r92su_state_t new_state)
{
	r92su->state = new_state;
}

static inline void r92su_mark_dead(struct r92su *r92su)
{
	struct net_device *ndev = r92su->wdev.netdev;
	r92su_set_state(r92su, R92SU_DEAD);

	if (ndev)
		netif_carrier_off(ndev);
}

static inline
struct r92su_bss_priv *r92su_get_bss_priv(struct cfg80211_bss *bss)
{
	return (struct r92su_bss_priv *) bss->priv;
}

static inline u8 *r92su_get_i3e_payload(struct ieee80211_hdr *hdr)
{
	return ((u8 *) hdr) + ieee80211_hdrlen(hdr->frame_control);
}

static inline unsigned int r92su_get_iv_len(struct r92su_key *key)
{
	switch (key->type) {
	case WEP40_ENCRYPTION:
	case WEP104_ENCRYPTION:
		return 4;

	case TKIP_ENCRYPTION:
	case AESCCMP_ENCRYPTION:
		return 8;

	default:
		return 0;
	}
}

struct r92su *r92su_alloc(struct device *main_dev);
int r92su_setup(struct r92su *r92su);
int r92su_register(struct r92su *r92su);
void r92su_unregister(struct r92su *r92su);
void r92su_free(struct r92su *r92su);

void r92su_disconnect_bss_event(struct r92su *r92su);

#endif /* __R92SU_R92SU_H__ */
