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
#ifndef __R92SU_HW_H__
#define __R92SU_HW_H__

struct r92su;

void r92su_hw_init(struct r92su *r92su);

int r92su_hw_read_chip_version(struct r92su *r92su);

int r92su_hw_early_mac_setup(struct r92su *r92su);
int r92su_hw_late_mac_setup(struct r92su *r92su);
int r92su_hw_mac_deinit(struct r92su *r92su);
bool r92su_hw_wps_detect(struct r92su *r92su);
int r92su_hw_mac_set_rx_filter(struct r92su *r92su,
	bool data, bool mgt, bool ctrl, bool monitor);
void r92su_hw_queue_service_work(struct r92su *r92su);
int r92su_signal_scale_mapping(u32 raw_signal);
#endif /* __R92SU_HW_H__ */
