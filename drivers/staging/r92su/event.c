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

#include <asm/unaligned.h>

#include "r92su.h"
#include "event.h"
#include "h2cc2h.h"
#include "trace.h"
#include "debug.h"

typedef void (*c2h_handler)(struct r92su *, const struct h2cc2h *);

static void c2h_fwdbg_event(struct r92su *r92su, const struct h2cc2h *c2h)
{
	u16 c2h_len = le16_to_cpu(c2h->len);

	R92SU_DBG(r92su, "fwdbg: %.*s%s", c2h_len, c2h->data,
		  c2h->data[c2h_len - 2] == '\n' ? "" : "\n");
}

static void c2h_survey_event(struct r92su *r92su, const struct h2cc2h *c2h)
{
	const struct h2cc2h_bss *c2h_bss = (const void *)&c2h->data;
	struct r92su_add_bss *bss_priv;
	u32 bss_len;
	u16 len;

	/* Looks like the FW just attaches the raw probe_response IEs
	 * ... along with the FCS (since we enabled the RX flag for it
	 */
	len = le16_to_cpu(c2h->len) - FCS_LEN;
	bss_len = le32_to_cpu(c2h_bss->length) - FCS_LEN;

	if (len < sizeof(*c2h_bss) || len != bss_len ||
	    le32_to_cpu(c2h_bss->ie_length) <= 12) {
		R92SU_ERR(r92su, "received survey event with bad length.");
		r92su_mark_dead(r92su);
		return;
	}

	bss_priv = kmalloc(len - sizeof(*c2h_bss) + sizeof(*bss_priv),
			   GFP_ATOMIC);
	if (!bss_priv)
		return;

	memcpy(&bss_priv->fw_bss, c2h_bss, len);
	bss_priv->fw_bss.length = cpu_to_le32(len);
	bss_priv->fw_bss.ie_length = cpu_to_le32(
		le32_to_cpu(bss_priv->fw_bss.ie_length) - FCS_LEN);
	llist_add(&bss_priv->head, &r92su->add_bss_list);
	queue_work(r92su->wq, &r92su->add_bss_work);
}

static void c2h_survey_done_event(struct r92su *r92su,
				  const struct h2cc2h *c2h)
{
	/* prevent race with r92su_stop */
	if (!r92su_is_open(r92su))
		return;

	if (cancel_delayed_work(&r92su->survey_done_work))
		queue_delayed_work(r92su->wq, &r92su->survey_done_work, 0);
}

static void c2h_join_bss_event(struct r92su *r92su, const struct h2cc2h *c2h)
{
	const struct c2h_join_bss_event *join_bss = (const void *)c2h->data;

	if (r92su->connect_result)
		return;

	r92su->connect_result = kmemdup(join_bss, le16_to_cpu(c2h->len),
					GFP_ATOMIC);
	queue_work(r92su->wq, &r92su->connect_bss_work);
}

static void c2h_add_sta_event(struct r92su *r92su, const struct h2cc2h *c2h)
{
	const struct c2h_add_sta_event *addsta = (const void *) c2h->data;
	struct r92su_sta *new_sta;
	unsigned int id = le32_to_cpu(addsta->aid);

	new_sta = r92su_sta_alloc(r92su, addsta->mac_addr, id,
				  le32_to_cpu(addsta->aid),
				  GFP_ATOMIC);
	if (!new_sta)
		R92SU_ERR(r92su, "failed to alloc new station %pM",
			  addsta->mac_addr);
}

static void c2h_del_sta_event(struct r92su *r92su, const struct h2cc2h *c2h)
{
	const struct c2h_del_sta_event *delsta = (const void *) c2h->data;

	switch (r92su->wdev.iftype) {
	case NL80211_IFTYPE_STATION:
		r92su_disconnect_bss_event(r92su);
		break;
	default: {
		struct r92su_sta *sta;

		rcu_read_lock();
		sta = r92su_sta_get(r92su, delsta->mac_addr);
		if (sta)
			r92su_sta_del(r92su, sta->mac_id);
		rcu_read_unlock();
		break;
	}
	}
}

static void c2h_atim_done_event(struct r92su *r92su, const struct h2cc2h *c2h)
{

}

static void c2h_report_pwr_state_event(struct r92su *r92su,
				       const struct h2cc2h *c2h)
{
	const struct c2h_pwr_state_event *pwr = (const void *) c2h->data;
	u8 cpwm_tog = pwr->state & PS_TOG;

	if (r92su->cpwm_tog == cpwm_tog) {
		R92SU_ERR(r92su, "firmware is stuck, it didn't update CPWM "
			  "(it's stuck at 0x%x)", cpwm_tog);
	}

	r92su->cpwm = pwr->state & PS_STATE_MASK;
	r92su->cpwm_tog = cpwm_tog;
}

static void c2h_wps_pbc_event(struct r92su *r92su, const struct h2cc2h *c2h)
{

}

static void c2h_addba_report_event(struct r92su *r92su,
				   const struct h2cc2h *c2h)
{
	const struct c2h_add_ba_event *add_ba = (const void *) c2h->data;
	struct r92su_sta *sta;

	rcu_read_lock();
	sta = r92su_sta_get(r92su, add_ba->mac_addr);
	if (sta) {
		r92su_sta_alloc_tid(r92su, sta, add_ba->tid,
				    le16_to_cpu(add_ba->ssn));
	}
	rcu_read_unlock();
}

void r92su_c2h_event(struct r92su *r92su, const struct h2cc2h *c2h)
{
	unsigned int sequence = r92su->c2h_seq++;

	trace_r92su_c2h(wiphy_dev(r92su->wdev.wiphy), c2h);

	if (sequence != c2h->cmd_seq) {
		R92SU_DBG(r92su, "received an c2h event out of sequence.\n");
		R92SU_DBG(r92su, "expected: %d, got %d\n", sequence,
			  c2h->cmd_seq);

		r92su->c2h_seq = c2h->cmd_seq + 1;
	}

	R92SU_DBG(r92su, "c2h event:%x len:%d\n",
		  c2h->event, le16_to_cpu(c2h->len));

	switch (c2h->event) {
	case C2H_FWDBG_EVENT:
		c2h_fwdbg_event(r92su, c2h);
		break;
	case C2H_SURVEY_EVENT:
		c2h_survey_event(r92su, c2h);
		break;
	case C2H_SURVEY_DONE_EVENT:
		c2h_survey_done_event(r92su, c2h);
		break;
	case C2H_JOIN_BSS_EVENT:
		c2h_join_bss_event(r92su, c2h);
		break;
	case C2H_ADD_STA_EVENT:
		c2h_add_sta_event(r92su, c2h);
		break;
	case C2H_DEL_STA_EVENT:
		c2h_del_sta_event(r92su, c2h);
		break;
	case C2H_ATIM_DONE_EVENT:
		c2h_atim_done_event(r92su, c2h);
		break;
	case C2H_REPORT_PWR_STATE_EVENT:
		c2h_report_pwr_state_event(r92su, c2h);
		break;
	case C2H_WPS_PBC_EVENT:
		c2h_wps_pbc_event(r92su, c2h);
		break;
	case C2H_ADDBA_REPORT_EVENT:
		c2h_addba_report_event(r92su, c2h);
		break;

	default:
		R92SU_ERR(r92su, "received invalid c2h event:%x\n", c2h->event);
		print_hex_dump_bytes("C2H:", DUMP_PREFIX_OFFSET, c2h,
				     le16_to_cpu(c2h->len) + sizeof(*c2h));
		r92su_mark_dead(r92su);
		break;
	}
}
