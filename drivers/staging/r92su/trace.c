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
#include <linux/module.h>

/* sparse doesn't like tracepoint macros */
#ifndef __CHECKER__

#define CREATE_TRACE_POINTS
#include "trace.h"

EXPORT_TRACEPOINT_SYMBOL(r92su_h2c);
EXPORT_TRACEPOINT_SYMBOL(r92su_c2h);
EXPORT_TRACEPOINT_SYMBOL(r92su_ioread32);
EXPORT_TRACEPOINT_SYMBOL(r92su_ioread16);
EXPORT_TRACEPOINT_SYMBOL(r92su_ioread8);
EXPORT_TRACEPOINT_SYMBOL(r92su_iowrite32);
EXPORT_TRACEPOINT_SYMBOL(r92su_iowrite16);
EXPORT_TRACEPOINT_SYMBOL(r92su_iowrite8);
EXPORT_TRACEPOINT_SYMBOL(r92su_tx_data);
EXPORT_TRACEPOINT_SYMBOL(r92su_rx_data);
EXPORT_TRACEPOINT_SYMBOL(r92su_err);
EXPORT_TRACEPOINT_SYMBOL(r92su_info);
EXPORT_TRACEPOINT_SYMBOL(r92su_dbg);
#endif

