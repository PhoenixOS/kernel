/*
 * Copyright 2002-2004, Instant802 Networks, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef TKIP_H
#define TKIP_H

#include <linux/types.h>
#include <linux/crypto.h>

enum ieee80211_internal_tkip_state {
	TKIP_STATE_NOT_INIT,
	TKIP_STATE_PHASE1_DONE,
	TKIP_STATE_PHASE1_HW_UPLOADED,
};

struct tkip_ctx {
	u32 iv32;	/* current iv32 */
	u16 iv16;	/* current iv16 */
	u16 p1k[5];	/* p1k cache */
	u32 p1k_iv32;	/* iv32 for which p1k computed */
};

void ieee80211_tkip_encrypt_data(struct crypto_cipher *tfm,
				 const u8 *key, struct sk_buff *skb, u64 pn);
int ieee80211_tkip_decrypt_data(struct crypto_cipher *tfm,
				const u8 *key, struct sk_buff *skb, u64 pn);
#endif /* TKIP_H */
