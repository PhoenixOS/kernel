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
#ifndef __R92SU_CMD_H__
#define __R92SU_CMD_H__

#include <linux/mm.h>

#include "def.h"

struct sk_buff;
struct r92su;
struct ieee80211_channel;

void r92su_cmd_init(struct r92su *r92su);

struct sk_buff *r92su_h2c_alloc(struct r92su *r92su, int len, gfp_t flag);

int r92su_h2c_copy(struct r92su *r92su, const enum fw_h2c_cmd cmd,
		   const int len, const void *data, gfp_t flag);

int r92su_h2c_submit(struct r92su *r92su, struct sk_buff *skb,
		     const enum fw_h2c_cmd cmd);

int r92su_h2c_set_mac_addr(struct r92su *r92su, const u8 *addr);
int r92su_h2c_set_opmode(struct r92su *r92su, const enum h2c_op_modes mode);
int r92su_h2c_set_channel(struct r92su *r92su, const int channel);
int r92su_fw_iocmd(struct r92su *r92su, const u32 cmd);
int r92su_h2c_survey(struct r92su *r92su, struct cfg80211_ssid *ssid);

int r92su_h2c_disconnect(struct r92su *r92su);

int r92su_h2c_connect(struct r92su *r92su, const struct h2cc2h_bss *bss,
		      const bool join, const u8 *ie, const u32 ie_len);

int r92su_h2c_set_key(struct r92su *r92su, const enum r92su_enc_alg algo,
		      const u8 key_id, const bool group_key,
		      const u8 *keydata);

int r92su_h2c_set_sta_key(struct r92su *r92su, const enum r92su_enc_alg algo,
			  const u8 *mac_addr, const u8 *keydata);

int r92su_h2c_set_auth(struct r92su *r92su, const enum r92su_auth_mode auth,
		       const enum r92su_auth_1x _1x);

int r92su_h2c_start_ba(struct r92su *r92su, const unsigned int tid);

int r92su_h2c_set_power_mode(struct r92su *r92su, const u8 ps_mode,
                             const u8 smart_ps);
#endif /* __R92SU_CMD_H__ */
