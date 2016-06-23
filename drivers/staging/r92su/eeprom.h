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
#ifndef __R92SU_EEPROM_H__
#define __R92SU_EEPROM_H__

enum r92su_eeprom_type {
	EEPROM_93C46 = 0,
	EEPROM_93C56 = 1,
	EEPROM_BOOT_EFUSE = 2,
};

#define R92SU_EFUSE_REAL_SIZE	(512)

#define RTL8190_EEPROM_ID	(0x8129)

#define CHAN_NUM_2G     14
#define RF_PATH         2
#define CHAN_SET	3
/* 0 = Chan 1-3, 1 = Chan 4-8, 2 = Chan 9-14 */

enum r92su_custom_id_t {
	EEPROM_CID_DEFAULT		= 0x00,
	EEPROM_CID_ALPHA		= 0x01,
	EEPROM_CID_SENAO		= 0x03,
	EEPROM_CID_NETCORE		= 0x05,
	EEPROM_CID_CAMEO		= 0X08,
	EEPROM_CID_SITECOM		= 0x09,
	EEPROM_CID_COREGA		= 0x0b,
	EEPROM_CID_EDIMAX_BELKIN	= 0x0c,
	EEPROM_CID_SERCOMM_BELKIN	= 0x0e,
	EEPROM_CID_CAMEO1		= 0x0f,
	EEPROM_CID_WNC_COREGA		= 0x12,
	EEPROM_CID_CLEVO		= 0x13,
	EEPROM_CID_WHQL			= 0xfe,
};

struct r92su_eeprom {
	__le16 id;				/*  0 -  1 */
	__le16 hpon;				/*  2 -  3 */
	__le16 clk;				/*  4 -  5 */
	__le16 testr;				/*  6 -  7 */
	__le16 vid;				/*  8 -  9 */
	__le16 did;				/* 10 - 11 */
	u8 usb_optional;			/* 12 */
	u8 usb_phy_para1[5];			/* 13 - 17 */
	u8 mac_addr[6];				/* 18 - 23 */

	/* seems to contain vendor and device identification strings */
	u8 unkn2[56];				/* 24 - 79 */

	/* WARNING
	 * These definitions are mostly guesswork
	 */
	u8 version;				/* 80 */
	u8 channel_plan;			/* 81 */
	u8 custom_id;				/* 82 */
	u8 sub_custom_id;			/* 83 */
	u8 board_type;				/* 84 */

	/* tx power base */
	u8 tx_pwr_cck[RF_PATH][CHAN_SET];	/* 85 - 90 */
	u8 tx_pwr_ht40_1t[RF_PATH][CHAN_SET];	/* 91 - 96 */
	u8 tx_pwr_ht40_2t[RF_PATH][CHAN_SET];	/* 97 - 102 */

	u8 pw_diff;				/* 103 */
	u8 thermal_meter;			/* 104 */
	u8 crystal_cap;				/* 105 */
	u8 unkn3;				/* 106 */
	u8 tssi[RF_PATH];			/* 107 - 108 */
	u8 unkn4;				/* 109 */
	u8 tx_pwr_ht20_diff[3];			/* 110 - 112 */
	u8 tx_pwr_ofdm_diff[2];			/* 113 - 114, 124 */
	u8 unkn5[6];				/* 115 - 120 */
	u8 tx_pwr_group[1];			/* 121 ??? */
	u8 regulatory;				/* 122 ??? */
	u8 rf_ind_power_diff;			/* 123 */
	u8 tx_pwr_ofdm_diff_cont;		/* 124 */
	u8 unkn6[3];				/* 125 - 127 */
} __packed;

static inline void __check_eeprom__(void)
{
	BUILD_BUG_ON(sizeof(struct r92su_eeprom) > 128);
}

struct r92su;

u8 r92su_efuse_read(struct r92su *r92su, u16 address);
int r92su_eeprom_read(struct r92su *r92su);
#endif /* __R92SU_EEPROM_H__ */
