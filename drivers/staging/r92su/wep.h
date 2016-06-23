/*
 * Software WEP encryption implementation
 * Copyright 2002, Jouni Malinen <jkmaline@cc.hut.fi>
 * Copyright 2003, Instant802 Networks, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef WEP_H
#define WEP_H

#include <linux/types.h>
#include <linux/crypto.h>
#include <linux/skbuff.h>

struct crypto_cipher *ieee80211_wep_init(void);
void ieee80211_wep_free(struct crypto_cipher *tfm);

void ieee80211_wep_encrypt_data(struct crypto_cipher *tfm, u8 *rc4key,
				size_t klen, u8 *data, size_t data_len);

void ieee80211_wep_encrypt(struct crypto_cipher *tfm, struct sk_buff *skb,
			   const u8 *key, const u32 iv, int keylen,
			   int keyidx);

int ieee80211_wep_decrypt_data(struct crypto_cipher *tfm, u8 *rc4key,
			       size_t klen, u8 *data, size_t data_len);

int ieee80211_wep_decrypt(struct crypto_cipher *tfm, struct sk_buff *skb,
			  const u8 *key, const u32 iv, int keylen,
			  int keyidx);
#endif /* WEP_H */
