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
#include <linux/kernel.h>
#include <net/cfg80211.h>
#include <linux/export.h>
#include "r92su.h"
#include "debug.h"
#include "trace.h"

#define __r92su_fn(fn, print)					\
void __r92su_ ##fn(struct r92su *r92su,	const char *fmt, ...)	\
{								\
	struct va_format vaf = {				\
		.fmt = fmt,					\
	};							\
	va_list args;						\
								\
	va_start(args, fmt);					\
	vaf.va = &args;						\
	if (print)						\
		wiphy_ ##fn(r92su->wdev.wiphy, "%pV", &vaf);	\
	trace_r92su_ ##fn(wiphy_dev(r92su->wdev.wiphy), &vaf);	\
	va_end(args);						\
}

__r92su_fn(err, true)
__r92su_fn(info, true)
__r92su_fn(dbg, false)
