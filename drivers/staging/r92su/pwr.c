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

#include "r92su.h"
#include "usb.h"
#include "cmd.h"
#include "reg.h"
#include "pwr.h"

#include "debug.h"

static void r92su_set_rpwn(struct r92su *r92su, u8 mode)
{
	u8 rpwm;

	r92su->rpwm_tog ^= 0x80;

	rpwm = mode | r92su->rpwm_tog;
	r92su->rpwm = rpwm;
	r92su->cpwm = mode;
	r92su_write8(r92su, REG_USB_HRPWM, rpwm);
}

void r92su_set_power(struct r92su *r92su, bool on)
{
	if (on) {
		r92su_set_rpwn(r92su, PS_STATE_S4);
		r92su_h2c_set_power_mode(r92su, PS_MODE_ACTIVE, 0);
	} else {
		r92su_set_rpwn(r92su, PS_STATE_S2);
		r92su_h2c_set_power_mode(r92su, PS_MODE_RADIO_OFF, 1);
	}
}
