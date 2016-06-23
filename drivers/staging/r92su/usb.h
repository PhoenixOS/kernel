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
#ifndef __R92SU_USB_H__
#define __R92SU_USB_H__

#define RTL_RX_DESC_SIZE		24

#define RTL_USB_MAX_TXQ_NUM		4		/* max tx queue */
#define RTL_USB_MAX_EP_NUM		6		/* max ep number */
#define RTL_USB_MAX_TX_URBS_NUM		8
#define RTL_USB_MAX_RX_URBS_NUM		8

#define	REALTEK_USB_VENQT_READ			0xC0
#define	REALTEK_USB_VENQT_WRITE			0x40
#define REALTEK_USB_VENQT_CMD_REQ		0x05
#define	REALTEK_USB_VENQT_CMD_IDX		0x00

#define RTL92SU_SIZE_MAX_RX_BUFFER		32768

#define RTL8712_EP_RX				3
#define RTL8712_EP_RX9				9
#define RTL8712_EP_TX4				4
#define RTL8712_EP_TX5				5
#define RTL8712_EP_TX6				6
#define RTL8712_EP_TX7				7
#define RTL8712_EP_TX10				10
#define RTL8712_EP_TX11				11
#define RTL8712_EP_TX12				12
#define RTL8712_EP_TX13				13
#define RTL8712_EP_CTRL				0

u8 r92su_read8(struct r92su *r92su, const u32 address);
u16 r92su_read16(struct r92su *r92su, const u32 address);
u32 r92su_read32(struct r92su *r92su, const u32 address);
void r92su_write8(struct r92su *r92su, const u32 address, const u8 data);
void r92su_write16(struct r92su *r92su, const u32 address, const u16 data);
void r92su_write32(struct r92su *r92su, const u32 address, const u32 data);
void r92su_usb_prepare_firmware(struct r92su *r92su);
int r92su_usb_tx(struct r92su *r92su, struct sk_buff *skb,
		 const enum rtl8712_queues_t queue);

#endif /* __R92SU_USB_H__ */
