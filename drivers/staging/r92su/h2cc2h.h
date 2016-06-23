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
#ifndef __R92SU_H2CC2H_H__
#define __R92SU_H2CC2H_H__

#include <linux/ieee80211.h>

struct h2c_rates {
	u8 rates[8];
} __packed;

struct h2c_ext_rates {
	u8 rates[16];
} __packed;

struct h2c_iocmd {
	union {
		struct {
			u8 cmdclass;
			__le16 value;
			u8 index;
		} __packed;

		u32 cmd;
	} __packed;
} __packed;

struct h2c_set_power_mode {
	u8 mode;
	u8 flag_low_traffic_en;
	u8 flag_lpnav_en;
	u8 flag_rf_low_snr_en;

	u8 flag_dps_en;		/* 1: dps, 0: 32k */
	u8 bcn_rx_en;
	u8 bcn_pass_cnt;	/* fw report one beacon information to driver
				   when it receives bcn_pass_cnt  beacons */
	u8 bcn_to;		/* beacon TO (ms).  0 = no limit. */

	__le16 bcn_itv;
	u8 app_itv;		/* only for VOIP mode. */
	u8 awake_bcn_itv;

	u8 smart_ps;
	u8 bcn_pass_time;	/* unit: 100ms */
} __packed;

struct h2c_sta_psm {
	u8 aid;
	u8 status;
	u8 addr[ETH_ALEN];
} __packed;

struct h2c_basic_rates {
	struct h2c_rates basic_rates;
} __packed;

enum h2c_channel_plan_types {
	CHANNEL_PLAN_FCC = 0,
	CHANNEL_PLAN_IC = 1,
	CHANNEL_PLAN_ETSI = 2,
	CHANNEL_PLAN_SPAIN = 3,
	CHANNEL_PLAN_FRANCE = 4,
	CHANNEL_PLAN_MKK = 5,
	CHANNEL_PLAN_MKK1 = 6,
	CHANNEL_PLAN_ISRAEL = 7,
	CHANNEL_PLAN_TELEC = 8,

	/* Deprecated */
	CHANNEL_PLAN_MIC = 9,
	CHANNEL_PLAN_GLOBAL_DOMAIN = 10,
	CHANNEL_PLAN_WORLD_WIDE_13 = 11,
	CHANNEL_PLAN_TELEC_NETGEAR = 12,

	CHANNEL_PLAN_NCC = 13,
	CHANNEL_PLAN_5G = 14,
	CHANNEL_PLAN_5G_40M = 15,

	/* keep it last */
	__MAX_CHANNEL_PLAN
};

struct h2c_channel_plan {
	__le32 channel_plan;	/* see h2c_channel_plan_types */
} __packed;

struct h2c_antenna {
	u8 tx_antenna_set;
	u8 rx_antenna_set;
	u8 tx_antenna;
	u8 rx_antenna;
} __packed;

struct h2c_set_mac {
	u8 mac_addr[ETH_ALEN];
} __packed;

struct h2c_join_bss_rpt {
	u8 opmode;
	u8 ps_qos_info;
	u8 bssid[6];
	__le16 bcnitv;
	__le16 aid;
} __packed;

struct h2c_site_survey {
	__le32 active;		/* 0: passive, 1:active */
	__le32 bsslimit;	/* 1 - 48 */
	__le32 ssidlen;
	u8 ssid[IEEE80211_MAX_SSID_LEN + 1];
} __packed;

struct h2c_disconnect {
	__le32 rsvd0;
} __packed;

struct h2c_set_channel {
	__le32 channel;
} __packed;

enum r92su_auth_mode {
	R92SU_AUTH_OPEN = 0,
	R92SU_AUTH_SHARED = 1,
	R92SU_AUTH_8021X = 2,
};

enum r92su_auth_1x {
	R92SU_WPA_PSK = 0,
	R92SU_WPA_EAP = 1,
};

struct h2c_auth {
	u8 mode;
	u8 _1x;		/* 0 = PSK, 1 = EAP */
	u8 rsvd2[2];
} __packed;

struct h2c_key {
	u8 algorithm;	/* r92su_enc_alg */
	u8 key_id;
	u8 group_key;	/* 0 = unicast key, 1 = group key */
	u8 key[16];
} __packed;

struct h2c_sta_key {
	u8 mac_addr[ETH_ALEN];
	u8 algorithm;	/* r92su_enc_alg */
	u8 key[16];
};

struct h2c_add_ba_req {
	__le32 tid;
};

struct c2h_sta_key_event {
	u8 mac_addr[ETH_ALEN];
	u8 keyid;
	u8 rsvd;
};

struct h2c_11fh_network_configuration {	/* ndis_802_11_configuration_fh */
	__le32 length;
	__le32 hop_pattern;		/* as defined by 802.11, MSB set */
	__le32 hop_set;			/* = 1, if non-802.11 */
	__le32 dwell_time;		/* units are in Kibi usec */
} __packed;

struct h2c_network_configuration {	/* ndis_802_11_configuration */
	__le32 length;
	__le32 beacon_period;		/* units are in Kibi usec */
	__le32 atim_window;		/* units are in Kibi usec */
	__le32 frequency;		/* Spec says units are kHz -
					 * but the fw gives us
					 * channel index 1-14?! */
	struct h2c_11fh_network_configuration fh_config;
} __packed;

enum h2c_network_infrastruct_mode {
	MODE_IBSS,
	MODE_BSS,
	MODE_AUTO,
	MODE_INFRA_MAX,		/* Apparently that's not a real value,
				 * just the upper bound
				 */
	/* keep this last */
	__MAX_NETWORK_MODE
};

enum h2c_op_modes {
	OP_AUTO = 0,		/* Let the driver decides which AP to join */
	OP_ADHOC,		/* Single cell network (Ad-Hoc Clients) */
	OP_INFRA,		/* Multi cell network, roaming, ... */

	/* keep this last */
	__MAC_OP_MODES
};

enum h2c_network_type {
	TYPE_11FH = 0,
	TYPE_11DS,
	TYPE_11OFDM5GHZ,
	TYPE_11OFDM2GHZ,

	/* keep this last */
	__MAX_NETWORK_TYPE
};

struct h2c_op_mode {
	u8 mode;		/* see h2c_op_modes */
	u8 padding[3];
} __packed;

struct h2c_ssid {		/* ndis_802_11_ssid */
	__le32 length;
	u8 ssid[IEEE80211_MAX_SSID_LEN];
} __packed;

struct h2c_fixed_ies {
	__le64 timestamp;
	__le16 beaconint;
	__le16 caps;
	u8 ie[0];		/*
				 * (fixed?) and variable IE content -
				 * can be up to 768 bytes
				 */
} __packed;

struct h2cc2h_bss {		/* ndis_wlan_bssid_ex */
	__le32 length;		/* length of the full struct */
	u8 bssid[6];
	u8 padding[2];
	struct h2c_ssid	ssid;
	__le32 privacy;
	__le32 rssi;		/* ??? - maybe for request/survey */

	__le32 type;		/* see h2c_network_type */
	struct h2c_network_configuration config;
	__le32 mode;		/* see h2c_network_infrastruct_mode */
	struct h2c_ext_rates rates;
	__le32 ie_length;	/* not sure if this include fixed too */
	struct h2c_fixed_ies ies;
} __packed;

struct c2h_survery_done_event {
	__le32 bss_cnt;
} __packed;

enum c2h_join_network_types_t {
	WIRELESS_INVALID	= 0,
	WIRELESS_11B		= 1,
	WIRELESS_11G		= 2,
	WIRELESS_11BG		= (WIRELESS_11B | WIRELESS_11G),
	WIRELESS_11A		= 4,
	WIRELESS_11N		= 8,
	WIRELESS_11GN		= (WIRELESS_11G | WIRELESS_11N),
	WIRELESS_11BGN		= (WIRELESS_11B | WIRELESS_11G | WIRELESS_11N),
};

struct c2h_join_bss_event {
	__le32 head;
	__le32 tail;
	__le32 network_type;	/* should be h2c_network_type ...*/
	__le32 fixed;
	__le32 last_scanned;
	__le32 aid;
	__le32 join_result;
	struct h2cc2h_bss bss;
} __packed;

typedef u8 mac_rates_t[13];

struct h2c_data_rates {
	u8 mac_id;
	mac_rates_t rates;
} __packed;

struct c2h_add_sta_event {
	u8 mac_addr[ETH_ALEN];
	u8 padding[2];
	__le32 aid;
} __packed;

struct c2h_del_sta_event {
	u8 mac_addr[ETH_ALEN];
	u8 padding[2];
} __packed;

struct c2h_add_ba_event {
	u8 mac_addr[ETH_ALEN];
	__le16 ssn;
	u8 tid;
} __packed;

enum r92su_power_mgnt {
	PS_MODE_ACTIVE = 0,
	PS_MODE_MIN,
	PS_MODE_MAX,
	PS_MODE_DTIM,
	PS_MODE_VOIP,
	PS_MODE_UAPSD_WMM,
	PS_MODE_UAPSD,
	PS_MODE_IBSS,
	PS_MODE_WOWWLAN,
	PS_MODE_RADIO_OFF,
	PS_MODE_CARD_DISABLE,

	/* keep last */
	__PS_MODE_NUM,
};

/*
 * BIT[2:0] = HW state
 * BIT[3] = Protocol PS state, 0: register active state,
 *			       1: register sleep state
 * BIT[4] = sub-state
*/
#define PS_ACTIVE		0
#define PS_DYNAMIC_POWER_SAVE	BIT(0)
#define PS_LOW_CLOCK		(PS_DYNAMIC_POWER_SAVE)
#define PS_RADIO_OFF		BIT(1)
#define PS_ALL_ON		BIT(2)
#define PS_ST_ACTIVE		BIT(3)
#define PS_LOW_PERFORMANCE	BIT(4)
#define PS_STATE_MASK		(PS_DYNAMIC_POWER_SAVE | PS_RADIO_OFF | \
				 PS_ALL_ON | PS_LOW_PERFORMANCE)
#define PS_STATE_HW_MASK	(PS_DYNAMIC_POWER_SAVE | PS_RADIO_OFF | \
				 PS_ALL_ON)
#define PS_TOG			BIT(7)

#define PS_STATE_S0		(PS_DYNAMIC_POWER_SAVE)

#define PS_STATE_S1		(PS_LOW_CLOCK)
#define PS_STATE_S2		(PS_RADIO_OFF)
#define PS_STATE_S3		(PS_ALL_ON)
#define PS_STATE_S4		(PS_ST_ACTIVE | PS_ALL_ON)

struct c2h_pwr_state_event {
	u8 mode;
	u8 state; /* PS_... / CPWM value */
	__le16 rsvd;
} __packed;

enum fw_c2h_event {
	C2H_READ_MACREG_EVENT,				/* 0 */
	C2H_READBB_EVENT,
	C2H_READRF_EVENT,
	C2H_READ_EEPROM_EVENT,
	C2H_READ_EFUSE_EVENT,
	C2H_READ_CAM_EVENT,				/* 5 */
	C2H_GET_BASIC_RATE_EVENT,
	C2H_GET_DATA_RATE_EVENT,
	C2H_SURVEY_EVENT,
	C2H_SURVEY_DONE_EVENT,
	C2H_JOIN_BSS_EVENT,				/* 10 */
	C2H_ADD_STA_EVENT,
	C2H_DEL_STA_EVENT,
	C2H_ATIM_DONE_EVENT,
	C2H_TX_REPORT_EVENT,
	C2H_CCX_REPORT_EVENT,				/* 15 */
	C2H_DTM_REPORT_EVENT,
	C2H_TX_RATE_STATS_EVENT,
	C2H_C2H_LBK_EVENT,
	C2H_FWDBG_EVENT,
	C2H_C2HFEEDBACK_EVENT,				/* 20 */
	C2H_ADDBA_EVENT,
	C2H_HBCN_EVENT,
	C2H_REPORT_PWR_STATE_EVENT,
	C2H_WPS_PBC_EVENT,
	C2H_ADDBA_REPORT_EVENT,				/* 25 */

	/* keep it last */
	__MAX_C2H					/* 26 */
};

enum fw_h2c_cmd {
	H2C_READ_MACREG_CMD,				/* 0 */
	H2C_WRITE_MACREG_CMD,
	H2C_READBB_CMD,
	H2C_WRITEBB_CMD,
	H2C_READRF_CMD,
	H2C_WRITERF_CMD,				/* 5 */
	H2C_READ_EEPROM_CMD,
	H2C_WRITE_EEPROM_CMD,
	H2C_READ_EFUSE_CMD,
	H2C_WRITE_EFUSE_CMD,
	H2C_READ_CAM_CMD,				/* 10 */
	H2C_WRITE_CAM_CMD,
	H2C_SETBCNITV_CMD,
	H2C_SETMBIDCFG_CMD,
	H2C_JOINBSS_CMD,
	H2C_DISCONNECT_CMD,				/* 15 */
	H2C_CREATEBSS_CMD,
	H2C_SETOPMODE_CMD,
	H2C_SITESURVEY_CMD,
	H2C_SETAUTH_CMD,
	H2C_SETKEY_CMD,					/* 20 */
	H2C_SETSTAKEY_CMD,
	H2C_SETASSOCSTA_CMD,
	H2C_DELASSOCSTA_CMD,
	H2C_SETSTAPWRSTATE_CMD,
	H2C_SETBASICRATE_CMD,				/* 25 */
	H2C_GETBASICRATE_CMD,
	H2C_SETDATARATE_CMD,
	H2C_GETDATARATE_CMD,
	H2C_SETPHYINFO_CMD,
	H2C_GETPHYINFO_CMD,				/* 30 */
	H2C_SETPHY_CMD,
	H2C_GETPHY_CMD,
	H2C_READRSSI_CMD,
	H2C_READGAIN_CMD,
	H2C_SETATIM_CMD,				/* 35 */
	H2C_SETPWRMODE_CMD,
	H2C_JOINBSSRPT_CMD,
	H2C_SETRATABLE_CMD,
	H2C_GETRATABLE_CMD,
	H2C_GETCCXREPORT_CMD,				/* 40 */
	H2C_GETDTMREPORT_CMD,
	H2C_GETTXRATESTATICS_CMD,
	H2C_SETUSBSUSPEND_CMD,
	H2C_SETH2CLBK_CMD,
	H2C_ADDBA_REQ_CMD,				/* 45 */
	H2C_SETCHANNEL_CMD,
	H2C_SET_TXPOWER_CMD,
	H2C_SWITCH_ANTENNA_CMD,
	H2C_SET_XTAL_CAP_CMD,
	H2C_SET_SINGLE_CARRIER_TX_CMD,			/* 50 */
	H2C_SET_SINGLE_TONE_CMD,
	H2C_SET_CARRIER_SUPPRESION_TX_CMD,
	H2C_SET_CONTINOUS_TX_CMD,
	H2C_SWITCH_BW_CMD,
	H2C_TX_BEACON_CMD,				/* 55 */
	H2C_SET_POWER_TRACKING_CMD,
	H2C_AMSDU_TO_AMPDU_CMD,
	H2C_SET_MAC_ADDRESS_CMD,
	H2C_DISCONNECT_CTRL_CMD,
	H2C_SET_CHANNELPLAN_CMD,			/* 60 */
	H2C_DISCONNECT_CTRL_EX_CMD,
	H2C_GET_H2C_LBK_CMD,
	H2C_SET_PWR_PARAM_CMD,
	H2C_WOWWLAN_CONTROL_CMD,
	H2C_SET_PROBE_REQ_EXTRA_IE_CMD,			/* 65 */
	H2C_SET_ASSOC_REQ_EXTRA_IE_CMD,
	H2C_SET_PROBE_RSP_EXTRA_IE_CMD,
	H2C_SET_ASSOC_RSP_EXTRA_IE_CMD,
	H2C_GET_CURRENT_DATA_RATE_CMD,
	H2C_GET_TX_RETRY_CNT_CMD,			/* 70 */
	H2C_GET_RX_RETRY_CNT_CMD,
	H2C_GET_BCN_OK_CNT_CMD,
	H2C_GET_BCN_ERR_CNT_CMD,
	H2C_GET_CURRENT_TXPOWER_CMD,
	H2C_SET_DIG_CMD,				/* 75 */
	H2C_SET_RA_CMD,
	H2C_SET_PT_CMD,
	H2C_READ_RSSI_CMD,

	/* keep it last */
	__MAX_H2C					/* 79 */
};

static inline void __check_h2cc2h__(void)
{
	BUILD_BUG_ON(sizeof(struct c2h_survery_done_event) != 4);
	BUILD_BUG_ON(sizeof(struct c2h_del_sta_event) != 8);
	BUILD_BUG_ON(sizeof(struct c2h_add_ba_event) != 9);
	BUILD_BUG_ON(sizeof(struct c2h_add_sta_event) != 12);
	BUILD_BUG_ON(sizeof(struct c2h_sta_key_event) != 8);

	BUILD_BUG_ON(sizeof(struct h2c_op_mode) != 4);
	BUILD_BUG_ON(sizeof(struct h2c_add_ba_req) != 4);
	BUILD_BUG_ON(sizeof(struct h2c_set_channel) != 4);
	BUILD_BUG_ON(sizeof(struct h2c_disconnect) != 4);
	BUILD_BUG_ON(sizeof(struct h2c_antenna) != 4);
	BUILD_BUG_ON(sizeof(struct h2c_channel_plan) != 4);
	BUILD_BUG_ON(sizeof(struct h2c_set_mac) != 6);
	BUILD_BUG_ON(sizeof(struct h2c_rates) != 8);
	BUILD_BUG_ON(sizeof(struct h2c_sta_psm) != 8);
	BUILD_BUG_ON(sizeof(struct h2c_basic_rates) != 8);
	BUILD_BUG_ON(sizeof(struct h2c_join_bss_rpt) != 12);
	BUILD_BUG_ON(sizeof(struct h2c_set_power_mode) != 14);
	BUILD_BUG_ON(sizeof(struct h2c_ext_rates) != 16);
	BUILD_BUG_ON(sizeof(struct h2c_key) != 19);
	BUILD_BUG_ON(sizeof(struct h2c_sta_key) != 23);
	BUILD_BUG_ON(sizeof(struct h2c_ssid) != 36);
	BUILD_BUG_ON(sizeof(struct h2c_site_survey) != 45);
}

#endif /* __R92SU_H2CC2H_H__ */
