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
#ifndef __R92SU_DEBUG_H__
#define __R92SU_DEBUG_H__

#include "r92su.h"

void __r92su_err(struct r92su *r92su, const char *fmt, ...) __printf(2, 3);
void __r92su_info(struct r92su *r92su, const char *fmt, ...) __printf(2, 3);
void __r92su_dbg(struct r92su *r92su, const char *fmt, ...) __printf(2, 3);

#define R92SU_ERR(r, f, a...) __r92su_err((r), (f), ## a)
#define R92SU_INFO(r, f, a...) __r92su_err((r), (f), ## a)
#define R92SU_DBG(r, f, a...) __r92su_dbg((r), (f), ## a)

#endif /* __R92SU_DEBUG_H__ */

