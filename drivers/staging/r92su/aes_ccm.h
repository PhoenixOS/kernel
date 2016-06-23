#ifndef AES_CCM_H
#define AES_CCM_H

#include <linux/types.h>

struct crypto_aead;

struct crypto_aead *ieee80211_aes_key_setup_encrypt(const u8 key[],
						    size_t key_len,
						    size_t mic_len);

void ieee80211_aes_key_free(struct crypto_aead *tfm);


int ieee80211_aes_ccm_decrypt(struct crypto_aead *tfm, struct sk_buff *skb,
			      const u64 pn, size_t mic_len);

void ieee80211_aes_ccm_encrypt(struct crypto_aead *tfm, struct sk_buff *skb,
			       const u64 pn, size_t mic_len);

#endif /* AES_CCM_H */
