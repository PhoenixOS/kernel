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
#include <linux/debugfs.h>
#include <linux/vmalloc.h>
#include "r92su.h"

#include "debugfs.h"
#include "usb.h"
#include "hw.h"

#define ADD(buf, off, max, fmt, args...)				\
	(off += snprintf(&buf[(off)], (max - off), (fmt), ##args))

struct r92su_debugfs_fops {
	unsigned int read_bufsize;
	umode_t attr;
	char *(*read)(struct r92su *r92su, char *buf, size_t bufsize,
		      ssize_t *len);
	ssize_t (*write)(struct r92su *r92su, const char *buf, size_t size);
	const struct file_operations fops;

	enum r92su_state_t req_dev_state;
};

static ssize_t r92su_debugfs_read(struct file *file, char __user *userbuf,
				  size_t count, loff_t *ppos)
{
	struct r92su_debugfs_fops *dfops;
	struct r92su *r92su;
	char *buf = NULL, *res_buf = NULL;
	ssize_t ret = 0;
	int err = 0;

	if (!count)
		return 0;

	r92su = file->private_data;

	if (!r92su)
		return -ENODEV;
	dfops = container_of(file->f_op, struct r92su_debugfs_fops, fops);

	if (!dfops->read)
		return -ENOSYS;

	if (dfops->read_bufsize) {
		buf = vmalloc(dfops->read_bufsize);
		if (!buf)
			return -ENOMEM;
	}

	mutex_lock(&r92su->lock);
	if (r92su->state < dfops->req_dev_state) {
		err = -ENODEV;
		res_buf = buf;
		goto out_free;
	}

	res_buf = dfops->read(r92su, buf, dfops->read_bufsize, &ret);

	if (ret > 0)
		err = simple_read_from_buffer(userbuf, count, ppos,
					      res_buf, ret);
	else
		err = ret;

	WARN_ONCE(dfops->read_bufsize && (res_buf != buf),
		  "failed to write output buffer back to debugfs");

out_free:
	vfree(res_buf);
	mutex_unlock(&r92su->lock);
	return err;
}

static ssize_t r92su_debugfs_write(struct file *file,
	const char __user *userbuf, size_t count, loff_t *ppos)
{
	struct r92su_debugfs_fops *dfops;
	struct r92su *r92su;
	char *buf = NULL;
	int err = 0;

	if (!count)
		return 0;

	if (count > PAGE_SIZE)
		return -E2BIG;

	r92su = file->private_data;

	if (!r92su)
		return -ENODEV;
	dfops = container_of(file->f_op, struct r92su_debugfs_fops, fops);

	if (!dfops->write)
		return -ENOSYS;

	buf = vmalloc(count);
	if (!buf)
		return -ENOMEM;

	if (copy_from_user(buf, userbuf, count)) {
		err = -EFAULT;
		goto out_free;
	}

	if (mutex_trylock(&r92su->lock) == 0) {
		err = -EAGAIN;
		goto out_free;
	}

	if (r92su->state < dfops->req_dev_state) {
		err = -ENODEV;
		goto out_unlock;
	}

	err = dfops->write(r92su, buf, count);
	if (err)
		goto out_unlock;

out_unlock:
	mutex_unlock(&r92su->lock);

out_free:
	vfree(buf);
	return err;
}

#define __DEBUGFS_DECLARE_FILE(name, _read, _write, _read_bufsize,	\
			       _attr, _dstate)				\
static const struct r92su_debugfs_fops r92su_debugfs_##name ##_ops = {	\
	.read_bufsize = _read_bufsize,					\
	.read = _read,							\
	.write = _write,						\
	.attr = _attr,							\
	.req_dev_state = _dstate,					\
	.fops = {							\
		.open   = simple_open,					\
		.read   = r92su_debugfs_read,				\
		.write  = r92su_debugfs_write,				\
		.owner  = THIS_MODULE,					\
	},								\
}

#define DEBUGFS_DECLARE_FILE(name, _read, _write, _read_bufsize, _attr)	\
	__DEBUGFS_DECLARE_FILE(name, _read, _write, _read_bufsize,	\
			       _attr, R92SU_OPEN)

#define DEBUGFS_DECLARE_RO_FILE(name, _read_bufsize)			\
	DEBUGFS_DECLARE_FILE(name, r92su_debugfs_##name ##_read,	\
			     NULL, _read_bufsize, S_IRUSR)

#define DEBUGFS_DECLARE_WO_FILE(name)					\
	DEBUGFS_DECLARE_FILE(name, NULL, r92su_debugfs_##name ##_write,	\
			     0, S_IWUSR)

#define DEBUGFS_DECLARE_RW_FILE(name, _read_bufsize)			\
	DEBUGFS_DECLARE_FILE(name, r92su_debugfs_##name ##_read,	\
			     r92su_debugfs_##name ##_write,		\
			     _read_bufsize, S_IRUSR | S_IWUSR)

#define __DEBUGFS_DECLARE_RW_FILE(name, _read_bufsize, _dstate)		\
	__DEBUGFS_DECLARE_FILE(name, r92su_debugfs_##name ##_read,	\
			       r92su_debugfs_##name ##_write,		\
			       _read_bufsize, S_IRUSR | S_IWUSR, _dstate)

#define DEBUGFS_READONLY_FILE(name, _read_bufsize, fmt, value...)	\
static char *r92su_debugfs_ ##name ## _read(struct r92su *r92su,	\
					    char *buf, size_t buf_size,	\
					    ssize_t *len)		\
{									\
	ADD(buf, *len, buf_size, fmt "\n", ##value);			\
	return buf;							\
}									\
DEBUGFS_DECLARE_RO_FILE(name, _read_bufsize)

static const char *meminfo[__MAX_R92SU_MEM_TYPE] = {
	[R92SU_8] =  "byte",
	[R92SU_16] = "word",
	[R92SU_32] = " int"
};

static char *r92su_debugfs_eeprom_read(struct r92su *r92su,
					 char *buf, size_t buf_size,
					 ssize_t *len)
{
	struct r92su_eeprom *e = &r92su->eeprom;

	/* The complete eeprom layout is not known yet.
	 * Hence, this formatted output is left incomplete.
	 */

	ADD(buf, *len, buf_size, "id:           %.4x\n", e->id);
	ADD(buf, *len, buf_size, "hpon:         %.4x\n", e->hpon);
	ADD(buf, *len, buf_size, "clk:          %.4x\n", e->clk);
	ADD(buf, *len, buf_size, "testr:        %.4x\n", e->testr);
	ADD(buf, *len, buf_size, "vid:          %.4x\n", e->vid);
	ADD(buf, *len, buf_size, "did:          %.4x\n", e->did);
	ADD(buf, *len, buf_size, "usb_optional: %.2x\n", e->usb_optional);
	ADD(buf, *len, buf_size, "usb_phy_parm: %.2x %.2x %.2x %.2x %.2x\n",
	    e->usb_phy_para1[0], e->usb_phy_para1[1], e->usb_phy_para1[2],
	    e->usb_phy_para1[3], e->usb_phy_para1[4]);

	ADD(buf, *len, buf_size, "mac_addr:     %pM\n", e->mac_addr);
	ADD(buf, *len, buf_size, "version:      %.2x\n", e->version);
	ADD(buf, *len, buf_size, "channel plan: %.2x\n", e->channel_plan);
	ADD(buf, *len, buf_size, "custom_id:    %.2x\n", e->custom_id);
	ADD(buf, *len, buf_size, "sub_custom_id:%.2x\n", e->sub_custom_id);
	ADD(buf, *len, buf_size, "board_type:   %.2x\n", e->board_type);
	return buf;
}
DEBUGFS_DECLARE_RO_FILE(eeprom, 1024);

static char *r92su_debugfs_eeprom_raw_read(struct r92su *r92su,
					   char *buf, size_t buf_size,
					   ssize_t *len)
{
	*len = min(sizeof(r92su->eeprom), buf_size);
	memcpy(buf, &r92su->eeprom, *len);
	return buf;
}
DEBUGFS_DECLARE_RO_FILE(eeprom_raw, 1024);

static char *r92su_debugfs_sta_table_read(struct r92su *r92su,
					  char *buf, size_t buf_size,
					  ssize_t *len)
{
	int i;
	rcu_read_lock();
	for (i = 0; i < MAX_STA; i++) {
		struct r92su_sta *sta = r92su_sta_get_by_macid(r92su, i);
		struct r92su_key *key;

		ADD(buf, *len, buf_size, "mac_id: %2d ", i);
		if (!sta) {
			ADD(buf, *len, buf_size, " - empty -\n");
			continue;
		}

		key = rcu_dereference(sta->sta_key);
		ADD(buf, *len, buf_size,
		    "mac_addr:%pM aid:%d id2:%d enc:%d qos:%d ht:%d\n",
		    sta->mac_addr, sta->aid, sta->mac_id, sta->enc_sta,
		    sta->qos_sta, sta->ht_sta);

		if (key) {
			ADD(buf, *len, buf_size,
			    "key: type:%d, key_len:%d, idx:%d\n",
			    key->type, key->key_len, key->index);
		}
	}
	rcu_read_unlock();
	return buf;
}
DEBUGFS_DECLARE_RO_FILE(sta_table, 1024);

static char *r92su_debugfs_connected_bss_read(struct r92su *r92su,
					      char *buf, size_t buf_size,
					      ssize_t *len)
{
	int i;
	struct cfg80211_bss *bss;
	struct r92su_bss_priv *bss_priv;

	rcu_read_lock();
	bss = rcu_dereference(r92su->connect_bss);
	if (!bss) {
		rcu_read_unlock();
		return buf;
	}

	bss_priv = r92su_get_bss_priv(bss);

	ADD(buf, *len, buf_size, "BSSID:%pM bcn:%d capa:%d freq:%d\n",
	   bss->bssid, bss->beacon_interval, bss->capability,
	   bss->channel->center_freq);

	ADD(buf, *len, buf_size, "mac-id:%d, default multi key index:%d, %d\n",
	    bss_priv->sta->mac_id, bss_priv->def_multi_key_idx,
	    bss_priv->def_uni_key_idx);

	for (i = 0; i < ARRAY_SIZE(bss_priv->group_key); i++) {
		struct r92su_key *key = rcu_dereference(bss_priv->group_key[i]);
		if (!key)
			continue;

		ADD(buf, *len, buf_size,
		    "key: type:%d, key_len:%d, idx:%d\n",
		    key->type, key->key_len, key->index);
	}
	rcu_read_unlock();
	return buf;
}
DEBUGFS_DECLARE_RO_FILE(connected_bss, 1024);

static ssize_t r92su_debugfs_hw_ioread_write(struct r92su *r92su,
					     const char *buf, size_t count)
{
	int err = 0, n = 0, max_len = 32, res;
	unsigned int reg, tmp;
	enum r92su_mem_type_t type;

	if (!count)
		return 0;

	if (count > max_len)
		return -E2BIG;

	res = sscanf(buf, "0x%X %d", &reg, &n);
	if (res < 1) {
		err = -EINVAL;
		goto out;
	}

	if (res == 1)
		n = 1;

	switch (n) {
	case 4:
		tmp = r92su_read32(r92su, reg);
		type = R92SU_32;
		break;
	case 2:
		tmp = r92su_read16(r92su, reg);
		type = R92SU_16;
		break;
	case 1:
		tmp = r92su_read8(r92su, reg);
		type = R92SU_8;
		break;
	default:
		err = -EMSGSIZE;
		goto out;
	}

	r92su->debug.ring[r92su->debug.ring_tail].reg = reg;
	r92su->debug.ring[r92su->debug.ring_tail].value = tmp;
	r92su->debug.ring[r92su->debug.ring_tail].type = type;
	r92su->debug.ring_tail++;
	r92su->debug.ring_tail %= R92SU_DEBUG_RING_SIZE;

	if (r92su->debug.ring_len < R92SU_DEBUG_RING_SIZE)
		r92su->debug.ring_len++;

out:
	return err ? err : count;
}

static char *r92su_debugfs_hw_ioread_read(struct r92su *r92su, char *buf,
					  size_t bufsize, ssize_t *ret)
{
	int i;

	ADD(buf, *ret, bufsize,
		"                      33222222 22221111 11111100 00000000\n");
	ADD(buf, *ret, bufsize,
		"                      10987654 32109876 54321098 76543210\n");

	while (r92su->debug.ring_len) {
		struct r92su_debug_mem_rbe *rbe;

		rbe = &r92su->debug.ring[r92su->debug.ring_head];
		ADD(buf, *ret, bufsize, "%.4x = %.8x [%s]",
		    rbe->reg, rbe->value, meminfo[rbe->type]);

		for (i = 31; i >= 0; i--) {
			ADD(buf, *ret, bufsize, "%c%s",
			    rbe->value & BIT(i) ? 'X' : ' ',
			    (i % 8) == 0 ? " " : "");
		}
		ADD(buf, *ret, bufsize, "\n");

		r92su->debug.ring_head++;
		r92su->debug.ring_head %= R92SU_DEBUG_RING_SIZE;

		r92su->debug.ring_len--;
	}
	r92su->debug.ring_head = r92su->debug.ring_tail;
	return buf;
}
DEBUGFS_DECLARE_RW_FILE(hw_ioread, 160 + R92SU_DEBUG_RING_SIZE * 40);

static ssize_t r92su_debugfs_hw_iowrite_write(struct r92su *r92su,
					      const char *buf, size_t count)
{
	int err = 0, max_len = 22, res;
	u32 reg, val, n;

	if (!count)
		return 0;

	if (count > max_len)
		return -E2BIG;

	res = sscanf(buf, "0x%X 0x%X %d", &reg, &val, &n);
	if (res != 3) {
		if (res != 2) {
			err = -EINVAL;
			goto out;
		}
		n = 1;
	}

	switch (n) {
	case 4:
		r92su_write32(r92su, reg, val);
		err = 0;
		break;
	case 2:
		r92su_write16(r92su, reg, val);
		err = 0;
		break;
	case 1:
		r92su_write8(r92su, reg, val);
		err = 0;
		break;
	default:
		err = -EINVAL;
		if (err)
			goto out;
		break;
	}

out:
	return err ? err : count;
}
DEBUGFS_DECLARE_WO_FILE(hw_iowrite);


DEBUGFS_READONLY_FILE(tx_pending_urbs, 12, "%d",
		      atomic_read(&r92su->tx_pending_urbs));
DEBUGFS_READONLY_FILE(chip_rev, 12, "0x%x", r92su->chip_rev);

DEBUGFS_READONLY_FILE(eeprom_type, 12, "0x%x", r92su->eeprom_type);

DEBUGFS_READONLY_FILE(rx_queue_len, 20, "%d", skb_queue_len(&r92su->rx_queue));

DEBUGFS_READONLY_FILE(rf_type, 12, "0x%x", r92su->rf_type);

DEBUGFS_READONLY_FILE(cpwm, 12, "0x%x", r92su->cpwm);
DEBUGFS_READONLY_FILE(rpwm, 12, "0x%x", r92su->rpwm);

DEBUGFS_READONLY_FILE(h2c_seq, 12, "%d", r92su->h2c_seq);
DEBUGFS_READONLY_FILE(c2h_seq, 12, "%d", r92su->c2h_seq);

int r92su_register_debugfs(struct r92su *r92su)
{
	r92su->dfs = debugfs_create_dir(KBUILD_MODNAME,
		r92su->wdev.wiphy->debugfsdir);

#define DEBUGFS_ADD(name)						\
	debugfs_create_file(#name, r92su_debugfs_##name ##_ops.attr,	\
			    r92su->dfs, r92su,				\
			    &r92su_debugfs_##name ## _ops.fops);

	if (!r92su->dfs)
		return -EINVAL;

	DEBUGFS_ADD(tx_pending_urbs);
	DEBUGFS_ADD(hw_ioread);
	DEBUGFS_ADD(hw_iowrite);
	DEBUGFS_ADD(chip_rev);
	DEBUGFS_ADD(eeprom_type);
	DEBUGFS_ADD(rf_type);
	DEBUGFS_ADD(sta_table);
	DEBUGFS_ADD(connected_bss);
	DEBUGFS_ADD(eeprom);
	DEBUGFS_ADD(eeprom_raw);
	DEBUGFS_ADD(h2c_seq);
	DEBUGFS_ADD(c2h_seq);
	DEBUGFS_ADD(cpwm);
	DEBUGFS_ADD(rpwm);
	DEBUGFS_ADD(rx_queue_len);
	return 0;

#undef DEBUGFS_ADD
}

void r92su_unregister_debugfs(struct r92su *r92su)
{
	debugfs_remove_recursive(r92su->dfs);
}
