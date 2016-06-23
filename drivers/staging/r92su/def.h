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
#ifndef __R92SU_DEF_H__
#define __R92SU_DEF_H__

#include "h2cc2h.h"

#define RX_MPDU_QUEUE				0
#define RX_CMD_QUEUE				1
#define RX_MAX_QUEUE				2
#define NUM_ACS					4

#define SHORT_SLOT_TIME				9
#define NON_SHORT_SLOT_TIME			20

/* Rx smooth factor */
#define	RX_SMOOTH_FACTOR			20

/* Queue Select Value in TxDesc */
#define QSLT_BK					0x1
#define QSLT_BE					0x3
#define QSLT_VI					0x5
#define QSLT_VO					0x7
#define QSLT_BEACON				0x10
#define QSLT_HIGH				0x11
#define QSLT_MGNT				0x12
#define QSLT_CMD				0x13

#define	PHY_RSSI_SLID_WIN_MAX			100
#define	PHY_LINKQUALITY_SLID_WIN_MAX		20
#define	PHY_BEACON_RSSI_SLID_WIN_MAX		10

/* Tx Desc */
#define TX_DESC_SIZE				32

#define BIT_LEN_MASK_32(__bitlen)	 \
	(0xFFFFFFFF >> (32 - (__bitlen)))

#define SHIFT_AND_MASK_LE(__pdesc, __off, __shift, __mask)		\
	((le32_to_cpup(((__le32 *) __pdesc) + (__off)) >> (__shift)) &	\
	BIT_LEN_MASK_32(__mask))

#define SET_BITS_OFFSET_LE(__pdesc, __off, __shift, __len, __val)	\
	do {								\
		__le32 *__ptr = ((__le32 *)(__pdesc)) + (__off);	\
		u32 __mask = BIT_LEN_MASK_32(__len) << (__shift);	\
		*__ptr &= cpu_to_le32(~__mask);				\
		*__ptr |= cpu_to_le32(((__val) << (__shift)) & __mask);	\
	} while (0)

/* macros to read/write various fields in RX or TX descriptors */

/* Dword 0 */
#define SET_TX_DESC_PKT_SIZE(__pdesc, __val)			\
	SET_BITS_OFFSET_LE(__pdesc, 0, 0, 16, __val)
#define SET_TX_DESC_OFFSET(__pdesc, __val)			\
	SET_BITS_OFFSET_LE(__pdesc, 0, 16, 8, __val)
#define SET_TX_DESC_TYPE(__pdesc, __val)			\
	SET_BITS_OFFSET_LE(__pdesc, 0, 24, 2, __val)
#define SET_TX_DESC_LAST_SEG(__pdesc, __val)			\
	SET_BITS_OFFSET_LE(__pdesc, 0, 26, 1, __val)
#define SET_TX_DESC_FIRST_SEG(__pdesc, __val)			\
	SET_BITS_OFFSET_LE(__pdesc, 0, 27, 1, __val)
#define SET_TX_DESC_LINIP(__pdesc, __val)			\
	SET_BITS_OFFSET_LE(__pdesc, 0, 28, 1, __val)
#define SET_TX_DESC_OWN(__pdesc, __val)				\
	SET_BITS_OFFSET_LE(__pdesc, 0, 31, 1, __val)

/* Dword 1 */
#define GET_TX_DESC_QUEUE_SEL(__pdesc)				\
	SHIFT_AND_MASK_LE(__pdesc, 1, 8, 5)

#define SET_TX_DESC_MACID(__pdesc, __val)			\
	SET_BITS_OFFSET_LE(__pdesc, 1, 0, 5, __val)
#define SET_TX_DESC_QUEUE_SEL(__pdesc, __val)			\
	SET_BITS_OFFSET_LE(__pdesc, 1, 8, 5, __val)
#define SET_TX_DESC_NON_QOS(__pdesc, __val)			\
	SET_BITS_OFFSET_LE(__pdesc, 1, 16, 1, __val)
#define SET_TX_DESC_KEY_ID(__pdesc, __val)			\
	SET_BITS_OFFSET_LE(__pdesc, 1, 17, 2, __val)
#define SET_TX_DESC_SEC_TYPE(__pdesc, __val)			\
	SET_BITS_OFFSET_LE(__pdesc, 1, 22, 2, __val)

/* Dword 2 */
#define SET_TX_DESC_BMC(__pdesc, __val)				\
	SET_BITS_OFFSET_LE(__pdesc, 2, 7, 1, __val)
#define SET_TX_DESC_AGG_BREAK(__pdesc, __val)			\
	SET_BITS_OFFSET_LE(__pdesc, 2, 30, 1, __val)

/* Dword 3 */
#define SET_TX_DESC_PRIORITY(__pdesc, __val)			\
	SET_BITS_OFFSET_LE(__pdesc, 3, 16, 12, __val)

/* Dword 4 */
#define SET_TX_DESC_USER_RATE(__pdesc, __val)			\
	SET_BITS_OFFSET_LE(__pdesc, 4, 31, 1, __val)

/* Dword 5 */
#define SET_TX_DESC_USER_TX_RATE(__pdesc, __val)		\
	SET_BITS_OFFSET_LE(__pdesc, 5, 0, 31, __val)

/* Dword 6 */
#define SET_TX_DESC_IP_CHECK_SUM(__pdesc, __val)		\
	SET_BITS_OFFSET_LE(__pdesc, 6, 0, 16, __val)
#define SET_TX_DESC_TCP_CHECK_SUM(__pdesc, __val)		\
	SET_BITS_OFFSET_LE(__pdesc, 6, 16, 16, __val)

/* Dword 7 */
#define SET_TX_DESC_TX_BUFFER_SIZE(__pdesc, __val)		\
	SET_BITS_OFFSET_LE(__pdesc, 7, 0, 16, __val)
#define SET_TX_DESC_CMD_SEQ(__pdesc, __val)			\
	SET_BITS_OFFSET_LE(__pdesc, 7, 24, 7, __val)

typedef __le32 tx_hdr[8];

/* Rx Desc */
#define RX_DESC_SIZE				24
#define RX_DRV_INFO_SIZE_UNIT			8

/* Dword 0 */
#define GET_RX_DESC_PKT_LEN(__pdesc)				\
	SHIFT_AND_MASK_LE(__pdesc, 0, 0, 14)
#define GET_RX_DESC_CRC32(__pdesc)				\
	SHIFT_AND_MASK_LE(__pdesc, 0, 14, 1)
#define GET_RX_DESC_ICV(__pdesc)				\
	SHIFT_AND_MASK_LE(__pdesc, 0, 15, 1)
#define GET_RX_DESC_DRVINFO_SIZE(__pdesc)			\
	SHIFT_AND_MASK_LE(__pdesc, 0, 16, 4)
#define GET_RX_DESC_SECURITY(__pdesc)				\
	SHIFT_AND_MASK_LE(__pdesc, 0, 20, 3)
#define GET_RX_DESC_QOS(__pdesc)				\
	SHIFT_AND_MASK_LE(__pdesc, 0, 23, 1)
#define GET_RX_DESC_SHIFT(__pdesc)				\
	SHIFT_AND_MASK_LE(__pdesc, 0, 24, 2)
#define GET_RX_DESC_PHY_STATUS(__pdesc)				\
	SHIFT_AND_MASK_LE(__pdesc, 0, 26, 1)
#define GET_RX_DESC_SWDEC(__pdesc)				\
	SHIFT_AND_MASK_LE(__pdesc, 0, 27, 1)
#define GET_RX_DESC_LAST_SEG(__pdesc)				\
	SHIFT_AND_MASK_LE(__pdesc, 0, 28, 1)
#define GET_RX_DESC_FIRST_SEG(__pdesc)				\
	SHIFT_AND_MASK_LE(__pdesc, 0, 29, 1)
#define GET_RX_DESC_EOR(__pdesc)				\
	SHIFT_AND_MASK_LE(__pdesc, 0, 30, 1)
#define GET_RX_DESC_OWN(__pdesc)				\
	SHIFT_AND_MASK_LE(__pdesc, 0, 31, 1)

/* Dword 1 */
#define GET_RX_DESC_IS_CMD(__pdesc)				\
	(SHIFT_AND_MASK_LE(__pdesc, 1, 0, 9) == 0x1ff)
#define GET_RX_DESC_MACID(__pdesc)				\
	SHIFT_AND_MASK_LE(__pdesc, 1, 0, 5)
#define GET_RX_DESC_TID(__pdesc)				\
	SHIFT_AND_MASK_LE(__pdesc, 1, 5, 4)
#define GET_RX_DESC_PAGGR(__pdesc)				\
	SHIFT_AND_MASK_LE(__pdesc, 1, 14, 1)
#define GET_RX_DESC_FAGGR(__pdesc)				\
	SHIFT_AND_MASK_LE(__pdesc, 1, 15, 1)
#define GET_RX_DESC_A1_FIT(__pdesc)				\
	SHIFT_AND_MASK_LE(__pdesc, 1, 16, 4)
#define GET_RX_DESC_A2_FIT(__pdesc)				\
	SHIFT_AND_MASK_LE(__pdesc, 1, 20, 4)
#define GET_RX_DESC_PAM(__pdesc)				\
	SHIFT_AND_MASK_LE(__pdesc, 1, 24, 1)
#define GET_RX_DESC_PWR(__pdesc)				\
	SHIFT_AND_MASK_LE(__pdesc, 1, 25, 1)
#define GET_RX_DESC_MORE_DATA(__pdesc)				\
	SHIFT_AND_MASK_LE(__pdesc, 1, 26, 1)
#define GET_RX_DESC_MORE_FRAG(__pdesc)				\
	SHIFT_AND_MASK_LE(__pdesc, 1, 27, 1)
#define GET_RX_DESC_TYPE(__pdesc)				\
	SHIFT_AND_MASK_LE(__pdesc, 1, 28, 2)
#define GET_RX_DESC_MC(__pdesc)					\
	SHIFT_AND_MASK_LE(__pdesc, 1, 30, 1)
#define GET_RX_DESC_BC(__pdesc)					\
	SHIFT_AND_MASK_LE(__pdesc, 1, 31, 1)

/* Dword 2 */
#define GET_RX_DESC_SEQ(__pdesc)				\
	SHIFT_AND_MASK_LE(__pdesc, 2, 0, 12)
#define GET_RX_DESC_FRAG(__pdesc)				\
	SHIFT_AND_MASK_LE(__pdesc, 2, 12, 4)
#define GET_RX_DESC_PKTCNT(__pdesc)				\
	SHIFT_AND_MASK_LE(__pdesc, 2, 16, 8)

/* Dword 3 */
#define GET_RX_DESC_RX_MCS(__pdesc)				\
	SHIFT_AND_MASK_LE(__pdesc, 3, 0, 6)
#define GET_RX_DESC_RX_HT(__pdesc)				\
	SHIFT_AND_MASK_LE(__pdesc, 3, 6, 1)
#define GET_RX_DESC_SPLCP(__pdesc)				\
	SHIFT_AND_MASK_LE(__pdesc, 3, 8, 1)
#define GET_RX_DESC_BW(__pdesc)					\
	SHIFT_AND_MASK_LE(__pdesc, 3, 9, 1)
#define GET_RX_DESC_TCP_CHK_RPT(__pdesc)			\
	SHIFT_AND_MASK_LE(__pdesc, 3, 11, 1)
#define GET_RX_DESC_IP_CHK_RPT(__pdesc)				\
	SHIFT_AND_MASK_LE(__pdesc, 3, 12, 1)
#define GET_RX_DESC_TCP_CHK_VALID(__pdesc)			\
	SHIFT_AND_MASK_LE(__pdesc, 3, 13, 1)

/* Dword 5 */
#define GET_RX_DESC_TSFL(__pdesc)				\
	SHIFT_AND_MASK_LE(__pdesc, 5, 0, 32)

typedef __le32 rx_hdr[6];

struct rx_hdr_phy_cck {
	/* For CCK rate descriptor. This is an unsigned 8:1 variable.
	 * LSB bit present 0.5. And MSB 7 bts present a signed value.
	 * Range from -64 to + 63.5 */
	u8 adc_pwdb_X[4];
	u8 sq_rpt;
	u8 cck_agc_rpt;
} __packed;

struct rx_hdr_phy_ofdm {
	u8 gain_trws[4];
	u8 pwdb_all;
	u8 cfosho[4];
	u8 cfotail[4];
	u8 rxevm[2];
	u8 rxsnr[4];
	u8 pdsnr[2];
	u8 csi_current[2];
	u8 csi_target[2];
	u8 sigevm[2];
	u8 max_ex_power;
} __packed;

union rx_hdr_phy {
	struct rx_hdr_phy_cck cck;
	struct rx_hdr_phy_ofdm ofdm;
};

#define H2CC2H_HDR_LEN		(8)

struct h2cc2h {
	__le16 len;
	u8 event;
	u8 cmd_seq;

	u8 agg_num;
	u8 unkn;
	__le16 agg_total_len;

	u8 data[0];
} __packed;

struct tx_packet {
	tx_hdr hdr;

	union {
		struct ieee80211_hdr i3e;
		struct h2cc2h h2c;
		u8 raw_data[0];
	} __packed;
} __packed;

struct rx_packet {
	rx_hdr hdr;
	union {
		/* No direct access to the rx data possible. The rx_hdr
		 * contains shift value (used to tell the offset of the
		 * ieee80211_hdr in 64-bit words).
		 */

		struct h2cc2h c2h;
		union rx_hdr_phy phy;
		u8 raw_data[0];
	} __packed;
} __packed;

enum rtl_desc92_rate {
	DESC92_RATE1M = 0x00,
	DESC92_RATE2M = 0x01,
	DESC92_RATE5_5M = 0x02,
	DESC92_RATE11M = 0x03,

	DESC92_RATE6M = 0x04,
	DESC92_RATE9M = 0x05,
	DESC92_RATE12M = 0x06,
	DESC92_RATE18M = 0x07,
	DESC92_RATE24M = 0x08,
	DESC92_RATE36M = 0x09,
	DESC92_RATE48M = 0x0a,
	DESC92_RATE54M = 0x0b,

	DESC92_RATEMCS0 = 0x0c,
	DESC92_RATEMCS1 = 0x0d,
	DESC92_RATEMCS2 = 0x0e,
	DESC92_RATEMCS3 = 0x0f,
	DESC92_RATEMCS4 = 0x10,
	DESC92_RATEMCS5 = 0x11,
	DESC92_RATEMCS6 = 0x12,
	DESC92_RATEMCS7 = 0x13,
	DESC92_RATEMCS8 = 0x14,
	DESC92_RATEMCS9 = 0x15,
	DESC92_RATEMCS10 = 0x16,
	DESC92_RATEMCS11 = 0x17,
	DESC92_RATEMCS12 = 0x18,
	DESC92_RATEMCS13 = 0x19,
	DESC92_RATEMCS14 = 0x1a,
	DESC92_RATEMCS15 = 0x1b,
	DESC92_RATEMCS15_SG = 0x1c,
	DESC92_RATEMCS32 = 0x20,
};

enum rf_optype {
	RF_OP_BY_SW_3WIRE = 0,
	RF_OP_BY_FW,
	RF_OP_MAX
};

enum ic_inferiority {
	IC_INFERIORITY_A = 0,
	IC_INFERIORITY_B = 1,
};

enum rf_type {
	RF_1T1R = 0,
	RF_1T2R = 1,
	RF_2T2R = 2,
	RF_2T2R_GREEN = 3,
};

enum ht_channel_width {
	HT_CHANNEL_WIDTH_20 = 0,
	HT_CHANNEL_WIDTH_20_40 = 1,
};

enum r92su_enc_alg {
	NO_ENCRYPTION = 0,
	WEP40_ENCRYPTION = 1,
	TKIP_ENCRYPTION = 2,
	TKIP_WTMIC = 3,
	AESCCMP_ENCRYPTION = 4,
	WEP104_ENCRYPTION = 5,

	__MAX_ENCRYPTION
};

static inline void __check_def__(void)
{
	BUILD_BUG_ON(sizeof(tx_hdr) != TX_DESC_SIZE);
	BUILD_BUG_ON(sizeof(rx_hdr) != RX_DESC_SIZE);
	BUILD_BUG_ON(sizeof(struct h2cc2h) != H2CC2H_HDR_LEN);
}

#endif /* __R92SU_DEF_H__ */
