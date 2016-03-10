#include <linux/kernel.h>

#include "SSEPlus_REF.h"


void ssp_abs_epi8(ssp_m128 *A)
{
	A->s8[0]  = (A->s8[0] < 0) ? -A->s8[0]  : A->s8[0];
	A->s8[1]  = (A->s8[1] < 0) ? -A->s8[1]  : A->s8[1];
	A->s8[2]  = (A->s8[2] < 0) ? -A->s8[2]  : A->s8[2];
	A->s8[3]  = (A->s8[3] < 0) ? -A->s8[3]  : A->s8[3];
	A->s8[4]  = (A->s8[4] < 0) ? -A->s8[4]  : A->s8[4];
	A->s8[5]  = (A->s8[5] < 0) ? -A->s8[5]  : A->s8[5];
	A->s8[6]  = (A->s8[6] < 0) ? -A->s8[6]  : A->s8[6];
	A->s8[7]  = (A->s8[7] < 0) ? -A->s8[7]  : A->s8[7];
	A->s8[8]  = (A->s8[8] < 0) ? -A->s8[8]  : A->s8[8];
	A->s8[9]  = (A->s8[9] < 0) ? -A->s8[9]  : A->s8[9];
	A->s8[10] = (A->s8[10] < 0) ? -A->s8[10] : A->s8[10];
	A->s8[11] = (A->s8[11] < 0) ? -A->s8[11] : A->s8[11];
	A->s8[12] = (A->s8[12] < 0) ? -A->s8[12] : A->s8[12];
	A->s8[13] = (A->s8[13] < 0) ? -A->s8[13] : A->s8[13];
	A->s8[14] = (A->s8[14] < 0) ? -A->s8[14] : A->s8[14];
	A->s8[15] = (A->s8[15] < 0) ? -A->s8[15] : A->s8[15];
}

void ssp_abs_epi16(ssp_m128 *A)
{
	A->s16[0] = (A->s16[0] < 0) ? -A->s16[0]  : A->s16[0];
	A->s16[1] = (A->s16[1] < 0) ? -A->s16[1]  : A->s16[1];
	A->s16[2] = (A->s16[2] < 0) ? -A->s16[2]  : A->s16[2];
	A->s16[3] = (A->s16[3] < 0) ? -A->s16[3]  : A->s16[3];
	A->s16[4] = (A->s16[4] < 0) ? -A->s16[4]  : A->s16[4];
	A->s16[5] = (A->s16[5] < 0) ? -A->s16[5]  : A->s16[5];
	A->s16[6] = (A->s16[6] < 0) ? -A->s16[6]  : A->s16[6];
	A->s16[7] = (A->s16[7] < 0) ? -A->s16[7]  : A->s16[7];
}

void ssp_abs_epi32(ssp_m128 *A)
{
	A->s32[0] = (A->s32[0] < 0) ? -A->s32[0]  : A->s32[0];
	A->s32[1] = (A->s32[1] < 0) ? -A->s32[1]  : A->s32[1];
	A->s32[2] = (A->s32[2] < 0) ? -A->s32[2]  : A->s32[2];
	A->s32[3] = (A->s32[3] < 0) ? -A->s32[3]  : A->s32[3];
}

ssp_m128 ssp_shuffle_epi8(ssp_m128 *A, ssp_m128 *MSK)
{
	ssp_m128 B;

	B.s8[0]  = (MSK->s8[0]  & 0x80) ? 0 : A->s8[(MSK->s8[0]  & 0xf)];
	B.s8[1]  = (MSK->s8[1]  & 0x80) ? 0 : A->s8[(MSK->s8[1]  & 0xf)];
	B.s8[2]  = (MSK->s8[2]  & 0x80) ? 0 : A->s8[(MSK->s8[2]  & 0xf)];
	B.s8[3]  = (MSK->s8[3]  & 0x80) ? 0 : A->s8[(MSK->s8[3]  & 0xf)];
	B.s8[4]  = (MSK->s8[4]  & 0x80) ? 0 : A->s8[(MSK->s8[4]  & 0xf)];
	B.s8[5]  = (MSK->s8[5]  & 0x80) ? 0 : A->s8[(MSK->s8[5]  & 0xf)];
	B.s8[6]  = (MSK->s8[6]  & 0x80) ? 0 : A->s8[(MSK->s8[6]  & 0xf)];
	B.s8[7]  = (MSK->s8[7]  & 0x80) ? 0 : A->s8[(MSK->s8[7]  & 0xf)];
	B.s8[8]  = (MSK->s8[8]  & 0x80) ? 0 : A->s8[(MSK->s8[8]  & 0xf)];
	B.s8[9]  = (MSK->s8[9]  & 0x80) ? 0 : A->s8[(MSK->s8[9]  & 0xf)];
	B.s8[10] = (MSK->s8[10] & 0x80) ? 0 : A->s8[(MSK->s8[10] & 0xf)];
	B.s8[11] = (MSK->s8[11] & 0x80) ? 0 : A->s8[(MSK->s8[11] & 0xf)];
	B.s8[12] = (MSK->s8[12] & 0x80) ? 0 : A->s8[(MSK->s8[12] & 0xf)];
	B.s8[13] = (MSK->s8[13] & 0x80) ? 0 : A->s8[(MSK->s8[13] & 0xf)];
	B.s8[14] = (MSK->s8[14] & 0x80) ? 0 : A->s8[(MSK->s8[14] & 0xf)];
	B.s8[15] = (MSK->s8[15] & 0x80) ? 0 : A->s8[(MSK->s8[15] & 0xf)];

	return B;
}

void ssp_alignr_epi8(ssp_m128 *ret, ssp_m128 *a, ssp_m128 *b, const unsigned int ralign)
{
	u8 tmp[32];
	int i, j;

	if (ralign == 0) {
		*ret = *b;
		return;
	}

	ret->u64[1] = ret->u64[0] = 0;

	if (ralign >= 32)
		return;

	*((ssp_m128 *)(&tmp[0])) = *b;
	*((ssp_m128 *)(&tmp[16])) = *a;

	for (i = 15 + ralign, j = 15; i >= ralign; i--, j--)
		ret->u8[j] = (i < 32) ? tmp[i] : 0;
}

ssp_m128 ssp_sign_epi8(ssp_m128* a, ssp_m128* b)
{
	ssp_m128 A, B;
	A = *a;
	B = *b;

	A.s8[0]  = (B.s8[0]<0)  ? (-A.s8[0])  :((B.s8[0]==0) ? 0: A.s8[0]);
	A.s8[1]  = (B.s8[1]<0)  ? (-A.s8[1])  :((B.s8[1]==0) ? 0: A.s8[1]);
	A.s8[2]  = (B.s8[2]<0)  ? (-A.s8[2])  :((B.s8[2]==0) ? 0: A.s8[2]);
	A.s8[3]  = (B.s8[3]<0)  ? (-A.s8[3])  :((B.s8[3]==0) ? 0: A.s8[3]);
	A.s8[4]  = (B.s8[4]<0)  ? (-A.s8[4])  :((B.s8[4]==0) ? 0: A.s8[4]);
	A.s8[5]  = (B.s8[5]<0)  ? (-A.s8[5])  :((B.s8[5]==0) ? 0: A.s8[5]);
	A.s8[6]  = (B.s8[6]<0)  ? (-A.s8[6])  :((B.s8[6]==0) ? 0: A.s8[6]);
	A.s8[7]  = (B.s8[7]<0)  ? (-A.s8[7])  :((B.s8[7]==0) ? 0: A.s8[7]);
	A.s8[8]  = (B.s8[8]<0)  ? (-A.s8[8])  :((B.s8[8]==0) ? 0: A.s8[8]);
	A.s8[9]  = (B.s8[9]<0)  ? (-A.s8[9])  :((B.s8[9]==0) ? 0: A.s8[9]);
	A.s8[10] = (B.s8[10]<0) ? (-A.s8[10]) :((B.s8[10]==0)? 0: A.s8[10]);
	A.s8[11] = (B.s8[11]<0) ? (-A.s8[11]) :((B.s8[11]==0)? 0: A.s8[11]);
	A.s8[12] = (B.s8[12]<0) ? (-A.s8[12]) :((B.s8[12]==0)? 0: A.s8[12]);
	A.s8[13] = (B.s8[13]<0) ? (-A.s8[13]) :((B.s8[13]==0)? 0: A.s8[13]);
	A.s8[14] = (B.s8[14]<0) ? (-A.s8[14]) :((B.s8[14]==0)? 0: A.s8[14]);
	A.s8[15] = (B.s8[15]<0) ? (-A.s8[15]) :((B.s8[15]==0)? 0: A.s8[15]);

	return A;
}

#define SSP_SATURATION(a, pos_limit, neg_limit) (a>pos_limit) ? pos_limit : ((a<neg_limit)?neg_limit:a)

ssp_m128 ssp_sign_epi16(ssp_m128* a, ssp_m128* b)
{
	ssp_m128 A, B;
	A = *a;
	B = *b;

	A.s16[0]  = (B.s16[0]<0)  ? (-A.s16[0])  :((B.s16[0]==0) ? 0: A.s16[0]);
	A.s16[1]  = (B.s16[1]<0)  ? (-A.s16[1])  :((B.s16[1]==0) ? 0: A.s16[1]);
	A.s16[2]  = (B.s16[2]<0)  ? (-A.s16[2])  :((B.s16[2]==0) ? 0: A.s16[2]);
	A.s16[3]  = (B.s16[3]<0)  ? (-A.s16[3])  :((B.s16[3]==0) ? 0: A.s16[3]);
	A.s16[4]  = (B.s16[4]<0)  ? (-A.s16[4])  :((B.s16[4]==0) ? 0: A.s16[4]);
	A.s16[5]  = (B.s16[5]<0)  ? (-A.s16[5])  :((B.s16[5]==0) ? 0: A.s16[5]);
	A.s16[6]  = (B.s16[6]<0)  ? (-A.s16[6])  :((B.s16[6]==0) ? 0: A.s16[6]);
	A.s16[7]  = (B.s16[7]<0)  ? (-A.s16[7])  :((B.s16[7]==0) ? 0: A.s16[7]);

	return A;
}

ssp_m128 ssp_sign_epi32(ssp_m128* a, ssp_m128* b)
{
	ssp_m128 A, B;
	A = *a;
	B = *b;

	A.s32[0]  = (B.s32[0]<0)  ? (-A.s32[0])  :((B.s32[0]==0) ? 0: A.s32[0]);
	A.s32[1]  = (B.s32[1]<0)  ? (-A.s32[1])  :((B.s32[1]==0) ? 0: A.s32[1]);
	A.s32[2]  = (B.s32[2]<0)  ? (-A.s32[2])  :((B.s32[2]==0) ? 0: A.s32[2]);
	A.s32[3]  = (B.s32[3]<0)  ? (-A.s32[3])  :((B.s32[3]==0) ? 0: A.s32[3]);

	return A;
}

ssp_m128 ssp_mulhrs_epi16( ssp_m128* a, ssp_m128* b )
{
	ssp_m128 A,B;
	A = *a;
	B = *b;

	A.s16[0] = (s16) ((A.s16[0] * B.s16[0] + 0x4000) >> 15);
	A.s16[1] = (s16) ((A.s16[1] * B.s16[1] + 0x4000) >> 15);
	A.s16[2] = (s16) ((A.s16[2] * B.s16[2] + 0x4000) >> 15);
	A.s16[3] = (s16) ((A.s16[3] * B.s16[3] + 0x4000) >> 15);
	A.s16[4] = (s16) ((A.s16[4] * B.s16[4] + 0x4000) >> 15);
	A.s16[5] = (s16) ((A.s16[5] * B.s16[5] + 0x4000) >> 15);
	A.s16[6] = (s16) ((A.s16[6] * B.s16[6] + 0x4000) >> 15);
	A.s16[7] = (s16) ((A.s16[7] * B.s16[7] + 0x4000) >> 15);

	return A;
}

ssp_m128 ssp_maddubs_epi16( ssp_m128* a,  ssp_m128* b)
{
	ssp_m128 A, B, C;
	int tmp[8];
	A = *a;
	B = *b;

	// a is 8 bit unsigned integer, b is signed integer
	tmp[0] = A.u8[0] * B.s8[0] +  A.u8[1] * B.s8[1];
	C.s16[0] = (s16)(SSP_SATURATION(tmp[0], 32767, -32768));

	tmp[1] = A.u8[2] * B.s8[2] +  A.u8[3] * B.s8[3];
	C.s16[1] = (s16)(SSP_SATURATION(tmp[1], 32767, -32768));

	tmp[2] = A.u8[4] * B.s8[4] +  A.u8[5] * B.s8[5];
	C.s16[2] = (s16)(SSP_SATURATION(tmp[2], 32767, -32768));

	tmp[3] = A.u8[6] * B.s8[6] +  A.u8[7] * B.s8[7];
	C.s16[3] = (s16)(SSP_SATURATION(tmp[3], 32767, -32768));

	tmp[4] = A.u8[8] * B.s8[8] +  A.u8[9] * B.s8[9];
	C.s16[4] = (s16)(SSP_SATURATION(tmp[4], 32767, -32768));

	tmp[5] = A.u8[10] * B.s8[10] +  A.u8[11] * B.s8[11];
	C.s16[5] = (s16)(SSP_SATURATION(tmp[5], 32767, -32768));

	tmp[6] = A.u8[12] * B.s8[12] +  A.u8[13] * B.s8[13];
	C.s16[6] = (s16)(SSP_SATURATION(tmp[6], 32767, -32768));

	tmp[7] = A.u8[14] * B.s8[14] +  A.u8[15] * B.s8[15];
	C.s16[7] = (s16)(SSP_SATURATION(tmp[7], 32767, -32768));

	return C;
}

ssp_m128 ssp_hsub_epi16( ssp_m128* a, ssp_m128* b )
{
	ssp_m128 A, B;
	A = *a;
	B = *b;

	A.s16[0] = A.s16[0] - A.s16[1];
	A.s16[1] = A.s16[2] - A.s16[3];
	A.s16[2] = A.s16[4] - A.s16[5];
	A.s16[3] = A.s16[6] - A.s16[7];
	A.s16[4] = B.s16[0] - B.s16[1];
	A.s16[5] = B.s16[2] - B.s16[3];
	A.s16[6] = B.s16[4] - B.s16[5];
	A.s16[7] = B.s16[6] - B.s16[7];

	return A;
}

ssp_m128 ssp_hsub_epi32( ssp_m128* a, ssp_m128* b )
{
	ssp_m128 A, B;
	A = *a;
	B = *b;

	A.s32[0] = A.s32[0] - A.s32[1];
	A.s32[1] = A.s32[2] - A.s32[3];
	A.s32[2] = B.s32[0] - B.s32[1];
	A.s32[3] = B.s32[2] - B.s32[3];

	return A;
}
ssp_m128 ssp_hsubs_epi16( ssp_m128* a, ssp_m128* b )
{
	ssp_m128 A, B;
	int answer[8];
	A = *a;
	B = *b;

	answer[0] = A.s16[0] - A.s16[1];
	A.s16[0]  = (s16) (SSP_SATURATION(answer[0], 32767, -32768));
	answer[1] = A.s16[2] - A.s16[3];
	A.s16[1]  = (s16) (SSP_SATURATION(answer[1], 32767, -32768));
	answer[2] = A.s16[4] - A.s16[5];
	A.s16[2]  = (s16) (SSP_SATURATION(answer[2], 32767, -32768));
	answer[3] = A.s16[6] - A.s16[7];
	A.s16[3]  = (s16) (SSP_SATURATION(answer[3], 32767, -32768));
	answer[4] = B.s16[0] - B.s16[1];
	A.s16[4]  = (s16) (SSP_SATURATION(answer[4], 32767, -32768));
	answer[5] = B.s16[2] - B.s16[3];
	A.s16[5]  = (s16) (SSP_SATURATION(answer[5], 32767, -32768));
	answer[6] = B.s16[4] - B.s16[5];
	A.s16[6]  = (s16) (SSP_SATURATION(answer[6], 32767, -32768));
	answer[7] = B.s16[6] - B.s16[7];
	A.s16[7]  = (s16) (SSP_SATURATION(answer[7], 32767, -32768));

	return A;
}

ssp_m128 ssp_hadd_epi16( ssp_m128* a, ssp_m128* b )
{
	ssp_m128 A, B;
	A = *a;
	B = *b;

	A.s16[0] = A.s16[0] + A.s16[1];
	A.s16[1] = A.s16[2] + A.s16[3];
	A.s16[2] = A.s16[4] + A.s16[5];
	A.s16[3] = A.s16[6] + A.s16[7];
	A.s16[4] = B.s16[0] + B.s16[1];
	A.s16[5] = B.s16[2] + B.s16[3];
	A.s16[6] = B.s16[4] + B.s16[5];
	A.s16[7] = B.s16[6] + B.s16[7];
	return A;
}

ssp_m128 ssp_hadd_epi32( ssp_m128* a, ssp_m128* b )
{
	ssp_m128 A, B;
	A = *a;
	B = *b;

	A.s32[0] = A.s32[0] + A.s32[1];
	A.s32[1] = A.s32[2] + A.s32[3];
	A.s32[2] = B.s32[0] + B.s32[1];
	A.s32[3] = B.s32[2] + B.s32[3];

	return A;
}

ssp_m128 ssp_hadds_epi16( ssp_m128* a, ssp_m128* b )
{
	ssp_m128 A, B;
	int answer[8];
	A = *a;
	B = *b;

	answer[0] = A.s16[0] + A.s16[1];
	A.s16[0]  = (s16) (SSP_SATURATION(answer[0], 32767, -32768));
	answer[1] = A.s16[2] + A.s16[3];
	A.s16[1]  = (s16) (SSP_SATURATION(answer[1], 32767, -32768));
	answer[2] = A.s16[4] + A.s16[5];
	A.s16[2]  = (s16) (SSP_SATURATION(answer[2], 32767, -32768));
	answer[3] = A.s16[6] + A.s16[7];
	A.s16[3]  = (s16) (SSP_SATURATION(answer[3], 32767, -32768));
	answer[4] = B.s16[0] + B.s16[1];
	A.s16[4]  = (s16) (SSP_SATURATION(answer[4], 32767, -32768));
	answer[5] = B.s16[2] + B.s16[3];
	A.s16[5]  = (s16) (SSP_SATURATION(answer[5], 32767, -32768));
	answer[6] = B.s16[4] + B.s16[5];
	A.s16[6]  = (s16) (SSP_SATURATION(answer[6], 32767, -32768));
	answer[7] = B.s16[6] + B.s16[7];
	A.s16[7]  = (s16) (SSP_SATURATION(answer[7], 32767, -32768));

	return A;
}
