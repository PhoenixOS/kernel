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
#include "eeprom.h"
#include "cmd.h"
#include "reg.h"
#include "usb.h"
#include "debug.h"

static int r92su_parse_eeprom(struct r92su *r92su)
{
	if (r92su->eeprom.id != cpu_to_le16(RTL8190_EEPROM_ID)) {
		R92SU_ERR(r92su, "eeprom signature check has failed.\n");
		return -EINVAL;
	}

	switch (r92su->eeprom.board_type) {
	case 0:
		r92su->rf_type = R92SU_1T1R;
		break;
	case 1:
		r92su->rf_type = R92SU_1T2R;
		break;
	case 2:
		r92su->rf_type = R92SU_2T2R;
		break;
	default:
		R92SU_ERR(r92su, "unknown board type:%d.\n",
			  r92su->eeprom.board_type);
		return -EINVAL;
	}
	return 0;
}

static void r92su_efuse_initialize(struct r92su *r92su)
{
	u8 tmp;

	/* Enable LDOE25 Macro Block */
	tmp = r92su_read8(r92su, REG_EFUSE_TEST + 3);
	tmp |= BIT(7);
	r92su_write8(r92su, REG_EFUSE_TEST + 3, tmp);

	/* Enable EFuse Clock and set it for write action to 40 MHz */
	r92su_write8(r92su, REG_EFUSE_CLK_CTRL, 0x3);

	r92su_write8(r92su, REG_EFUSE_CTRL + 3, 0x72);
}

static void r92su_efuse_shutdown(struct r92su *r92su)
{
	u8 tmp;

	/* Disable LDOE25 Macro Block */
	tmp = r92su_read8(r92su, REG_EFUSE_TEST + 3);
	tmp &= ~BIT(7);
	r92su_write8(r92su, REG_EFUSE_TEST + 3, tmp);

	/* Change EFuse Clock for write action to 500 KHz */
	r92su_write8(r92su, REG_EFUSE_CLK_CTRL, 0x2);
}

static u8 __r92su_efuse_read(struct r92su *r92su, u16 address)
{
	u8 tmp;
	int i = 0;

#define R92SU_EFUSE_READ_RETRIES	10

	r92su_write8(r92su, REG_EFUSE_CTRL + 1, (address & 0xff));

	tmp = r92su_read8(r92su, REG_EFUSE_CTRL + 2);
	tmp &= ~(BIT(0) | BIT(1));
	tmp |= (address >> 8) & 0x3;
	r92su_write8(r92su, REG_EFUSE_CTRL + 2, tmp);
	r92su_write8(r92su, REG_EFUSE_CTRL + 3, 0x72); /* read */

	do {
		i++;
		tmp = r92su_read8(r92su, REG_EFUSE_CTRL + 3);
	} while (!(tmp & 0x80) && i < R92SU_EFUSE_READ_RETRIES);

	if (i >= R92SU_EFUSE_READ_RETRIES)
		return 0xff;

	return r92su_read8(r92su, REG_EFUSE_CTRL);

#undef R92SU_EFUSE_READ_RETRIES
}

u8 r92su_efuse_read(struct r92su *r92su, u16 address)
{
	u8 result;

	r92su_efuse_initialize(r92su);
	result = __r92su_efuse_read(r92su, address);
	r92su_efuse_shutdown(r92su);
	return result;
}

static int r92su_fetch_eeprom_data(struct r92su *r92su)
{
	u8 *eimage = (void *) &r92su->eeprom;
	int off, i, len;
	u8 eprom;

	eprom = r92su_read8(r92su, REG_EEPROM_CMD);

	if (eprom & EEPROM_CMD_93C46)
		r92su->eeprom_type = EEPROM_93C46;
	else
		r92su->eeprom_type = EEPROM_BOOT_EFUSE;

	if (!(eprom & EEPROM_CMD_AUTOLOAD_OK))
		return -EAGAIN;

	r92su_efuse_initialize(r92su);

	memset(&r92su->eeprom, 0xff, sizeof(r92su->eeprom));

	len = 0;
	off = 0;
#define EFUSE_BLOCK_SIZE (8)
#define EFUSE_FETCH_SIZE (2)

	/* guard against out-of-bound writes in the following code */
	BUILD_BUG_ON(sizeof(r92su->eeprom) < 0xf * EFUSE_BLOCK_SIZE);

	/* The eeprom data is stored in sparse blocks. Each block can be
	 * between 1 byte (just descriptor, but no payload/bad block) and
	 * 9 bytes (descriptor + 4 * 2 byte payload).
	 *
	 * The blocks are organized as follows:
	 *  1. Block Descriptor Byte [desc]
	 *      high nibble (= pos)
	 *		position of the data block in the complete
	 *		eeprom image (=eimage) [pos]
	 *
	 *      low nibble (= map)
	 *		bitmap of useable data (0 = good, 1 = bad/skip)
	 *
	 *  2-9. data block (each 16-bits)
	 */
	while (off < R92SU_EFUSE_REAL_SIZE) {
		u8 desc, map, pos;

		desc = __r92su_efuse_read(r92su, off++);
		if (desc == 0xff)
			goto out;

		pos = desc >> 4;
		map = desc & 0xf;
		for (i = 0; i < EFUSE_BLOCK_SIZE; i += EFUSE_FETCH_SIZE) {
			if (!(map & 0x01)) {
				len += EFUSE_FETCH_SIZE;
				eimage[pos * EFUSE_BLOCK_SIZE + i] =
					__r92su_efuse_read(r92su, off++);
				if (off >= R92SU_EFUSE_REAL_SIZE)
					goto out;

				eimage[pos * EFUSE_BLOCK_SIZE + i + 1] =
					__r92su_efuse_read(r92su, off++);
				if (off >= R92SU_EFUSE_REAL_SIZE)
					goto out;

			}
			map >>= 1;
		}
	}
out:

	r92su_efuse_shutdown(r92su);
	return 0;

#undef EFUSE_BLOCK_SIZE
#undef EFUSE_FETCH_SIZE
}

int r92su_eeprom_read(struct r92su *r92su)
{
	int err = -EINVAL;

	err = r92su_fetch_eeprom_data(r92su);
	if (err)
		return err;

	err = r92su_parse_eeprom(r92su);
	if (err)
		return err;

	return err;
}
