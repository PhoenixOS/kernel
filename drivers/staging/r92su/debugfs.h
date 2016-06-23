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
#ifndef __R92SU_DEBUGFS_H__
#define __R92SU_DEBUGFS_H__

struct r92su;

enum r92su_mem_type_t {
	R92SU_8,
	R92SU_16,
	R92SU_32,

	/* last entry */
	__MAX_R92SU_MEM_TYPE
};

struct r92su_debug_mem_rbe {
	u32 reg;
	u32 value;
	enum r92su_mem_type_t type;
};

#define R92SU_DEBUG_RING_SIZE			64

struct r92su_debug {
	struct r92su_debug_mem_rbe ring[R92SU_DEBUG_RING_SIZE];
	unsigned int ring_head, ring_tail, ring_len;
};

#ifdef CONFIG_R92SU_DEBUGFS

int r92su_register_debugfs(struct r92su *r92su);
void r92su_unregister_debugfs(struct r92su *r92su);

#else /* CONFIG_R92SU_DEBUGFS */
static inline int r92su_register_debugfs(struct r92su *r92su)
{
	return 0;
}

static inline void r92su_unregister_debugfs(struct r92su *r92su)
{
}
#endif /* CONFIG_R92SU_DEBUGFS */

#endif /* __R92SU_DEBUGFS_H__ */
