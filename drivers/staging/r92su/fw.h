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
#ifndef __R92SU_FW_H__
#define __R92SU_FW_H__

#define RTL8712_HCI_TYPE_72USB			0x12
#define RTL8712_HCI_TYPE_72SDIO			0x03

#define RTL8192_MAX_FIRMWARE_CODE_SIZE		(64 * 1024)
#define RTL8192_MAX_RAW_FIRMWARE_CODE_SIZE	200000
#define RTL8192_CPU_START_OFFSET		0x80
/* Firmware Local buffer size. 64k */
#define	MAX_FIRMWARE_CODE_SIZE			0xFF00

#define	RT_8192S_FIRMWARE_HDR_SIZE		80
#define RT_8192S_FIRMWARE_HDR_EXCLUDE_PRI_SIZE	32

/* support till 64 bit bus width OS */
#define MAX_DEV_ADDR_SIZE			8
#define MAX_FIRMWARE_INFORMATION_SIZE		32
#define MAX_802_11_HEADER_LENGTH		(40 + \
						MAX_FIRMWARE_INFORMATION_SIZE)
#define ENCRYPTION_MAX_OVERHEAD			128
#define MAX_FRAGMENT_COUNT			8
#define MAX_TRANSMIT_BUFFER_SIZE		(1600 + \
						(MAX_802_11_HEADER_LENGTH + \
						ENCRYPTION_MAX_OVERHEAD) *\
						MAX_FRAGMENT_COUNT)

/* The following DM control code are for Reg0x364, */
#define	FW_DIG_ENABLE_CTL			BIT(0)
#define	FW_HIGH_PWR_ENABLE_CTL			BIT(1)
#define	FW_SS_CTL				BIT(2)
#define	FW_RA_INIT_CTL				BIT(3)
#define	FW_RA_BG_CTL				BIT(4)
#define	FW_RA_N_CTL				BIT(5)
#define	FW_PWR_TRK_CTL				BIT(6)
#define	FW_IQK_CTL				BIT(7)
#define	FW_FA_CTL				BIT(8)
#define	FW_DRIVER_CTRL_DM_CTL			BIT(9)
#define	FW_PAPE_CTL_BY_SW_HW			BIT(10)
#define	FW_DISABLE_ALL_DM			0
#define	FW_PWR_TRK_PARAM_CLR			0x0000ffff
#define	FW_RA_PARAM_CLR				0xffff0000

/* 8-bytes alignment required */
struct fw_priv {
	/* --- long word 0 ---- */
	/* 0x12: CE product, 0x92: IT product */
	u8 signature_0;
	/* 0x87: CE product, 0x81: IT product */
	u8 signature_1;
	/* 0x81: PCI-AP, 01:PCIe, 02: 92S-U,
	 * 0x82: USB-AP, 0x12: 72S-U, 03:SDIO */
	u8 hci_sel;
	/* the same value as register value  */
	u8 chip_version;
	/* customer  ID low byte */
	u8 customer_id_0;
	/* customer  ID high byte */
	u8 customer_id_1;
	/* 0x11:  1T1R, 0x12: 1T2R,
	 * 0x92: 1T2R turbo, 0x22: 2T2R */
	u8 rf_config;
	/* 4: 4EP, 6: 6EP, 11: 11EP */
	u8 usb_ep_num;

	/* --- long word 1 ---- */
	/* regulatory class bit map 0 */
	u8 regulatory_class_0;
	/* regulatory class bit map 1 */
	u8 regulatory_class_1;
	/* regulatory class bit map 2 */
	u8 regulatory_class_2;
	/* regulatory class bit map 3 */
	u8 regulatory_class_3;
	/* 0:SWSI, 1:HWSI, 2:HWPI */
	u8 rfintfs;
	u8 def_nettype;
	u8 turbo_mode;
	u8 low_power_mode;

	/* --- long word 2 ---- */
	/* 0x00: normal, 0x03: MACLBK, 0x01: PHYLBK */
	u8 lbk_mode;
	/* 1: for MP use, 0: for normal
	 * driver (to be discussed) */
	u8 mp_mode;
	/* 0: off, 1: on, 2: auto */
	u8 vcs_type;
	/* 0: none, 1: RTS/CTS, 2: CTS to self */
	u8 vcs_mode;
	u8 rsvd022;
	u8 rsvd023;
	u8 rsvd024;
	u8 rsvd025;

	/* --- long word 3 ---- */
	/* QoS enable */
	u8 qos_en;
	/* 40MHz BW enable */
	/* 4181 convert AMSDU to AMPDU, 0: disable */
	u8 bw_40mhz_en;
	u8 amsdu2ampdu_en;
	/* 11n AMPDU enable */
	u8 ampdu_en;
	/* FW offloads, 0: driver handles */
	u8 rate_control_offload;
	/* FW offloads, 0: driver handles */
	u8 aggregation_offload;
	u8 rsvd030;
	u8 rsvd031;

	/* --- long word 4 ---- */
	/* 1. FW offloads, 0: driver handles */
	u8 beacon_offload;
	/* 2. FW offloads, 0: driver handles */
	u8 mlme_offload;
	/* 3. FW offloads, 0: driver handles */
	u8 hwpc_offload;
	/* 4. FW offloads, 0: driver handles */
	u8 tcp_checksum_offload;
	/* 5. FW offloads, 0: driver handles */
	u8 tcp_offload;
	/* 6. FW offloads, 0: driver handles */
	u8 ps_control_offload;
	/* 7. FW offloads, 0: driver handles */
	u8 wwlan_offload;
	u8 rsvd040;

	/* --- long word 5 ---- */
	/* tcp tx packet length low byte */
	u8 tcp_tx_frame_len_l;
	/* tcp tx packet length high byte */
	u8 tcp_tx_frame_len_h;
	/* tcp rx packet length low byte */
	u8 tcp_rx_frame_len_l;
	/* tcp rx packet length high byte */
	u8 tcp_rx_frame_len_h;
	u8 rsvd050;
	u8 rsvd051;
	u8 rsvd052;
	u8 rsvd053;
} __packed;

#define	R8712SU_FW_SIGNATURE	(0x8712)
#define	R8192SU_FW_SIGNATURE	(0x8192)

/* 8-byte alinment required */
struct fw_hdr {
	__le16 signature;

	/* 0x8000 ~ 0x8FFF for FPGA version,
	 * 0x0000 ~ 0x7FFF for ASIC version */
	__le16 version;

	/* define the size of boot loader */
	__le32 dmem_size;

	/* define the size of FW in IMEM */
	__le32 img_imem_size;
	/* define the size of FW in SRAM */
	__le32 img_sram_size;

	/* define the size of DMEM variable */
	__le32 fw_priv_size;
	__le16 efuse_addr;
	__le16 h2ccmd_resp_addr;

	__le32 svn_evision;
	__le32 release_time;

	struct fw_priv fwpriv;

} __packed;

static inline void __check_fw__(void)
{
	BUILD_BUG_ON(sizeof(struct fw_priv) >= MAX_FIRMWARE_CODE_SIZE);
}

struct r92su;

int r92su_load_firmware(struct r92su *r92su);
int r92su_upload_firmware(struct r92su *r92su);
void r92su_release_firmware(struct r92su *r92su);

#endif /* __R92SU_FW_H__ */
