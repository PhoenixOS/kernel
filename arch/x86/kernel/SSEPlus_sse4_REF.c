#include <linux/kernel.h>

#include "SSEPlus_REF.h"

ssp_m128 ssp_cvtepi8_epi16 ( ssp_m128* a)
{
	ssp_m128 A = *a;

	A.s16[7] = A.s8[7];
	A.s16[6] = A.s8[6];
	A.s16[5] = A.s8[5];
	A.s16[4] = A.s8[4];
	A.s16[3] = A.s8[3];
	A.s16[2] = A.s8[2];
	A.s16[1] = A.s8[1];
	A.s16[0] = A.s8[0];
	return A;
}

ssp_m128 ssp_cvtepi8_epi32 ( ssp_m128* a)
{
	ssp_m128 A = *a;

	A.s32[3] = A.s8[3];
	A.s32[2] = A.s8[2];
	A.s32[1] = A.s8[1];
	A.s32[0] = A.s8[0];
	return A;
}

ssp_m128 ssp_cvtepi8_epi64 ( ssp_m128* a)
{
	ssp_m128 A = *a;

	A.s64[1] = A.s8[1];
	A.s64[0] = A.s8[0];
	return A;
}

ssp_m128 ssp_cvtepi16_epi32 ( ssp_m128* a)
{
	ssp_m128 A = *a;

	A.s32[3] = A.s16[3];
	A.s32[2] = A.s16[2];
	A.s32[1] = A.s16[1];
	A.s32[0] = A.s16[0];
	return A;
}

ssp_m128 ssp_cvtepi16_epi64( ssp_m128* a)
{
	ssp_m128 A = *a;

	A.s64[1] = A.s16[1];
	A.s64[0] = A.s16[0];
	return A;
}

ssp_m128 ssp_cvtepi32_epi64 ( ssp_m128* a)
{
	ssp_m128 A = *a;

	A.s64[1] = A.s32[1];
	A.s64[0] = A.s32[0];
	return A;
}

ssp_m128 ssp_cvtepu8_epi16 ( ssp_m128* a)
{
	ssp_m128 A = *a;

	A.s16[7] = A.u8[7];
	A.s16[6] = A.u8[6];
	A.s16[5] = A.u8[5];
	A.s16[4] = A.u8[4];
	A.s16[3] = A.u8[3];
	A.s16[2] = A.u8[2];
	A.s16[1] = A.u8[1];
	A.s16[0] = A.u8[0];
	return A;
}

ssp_m128 ssp_cvtepu8_epi32 ( ssp_m128* a)
{
	ssp_m128 A = *a;

	A.s32[3] = A.u8[3];
	A.s32[2] = A.u8[2];
	A.s32[1] = A.u8[1];
	A.s32[0] = A.u8[0];
	return A;
}

ssp_m128 ssp_cvtepu8_epi64 ( ssp_m128* a)
{
	ssp_m128 A = *a;

	A.s64[1] = A.u8[1];
	A.s64[0] = A.u8[0];
	return A;
}

ssp_m128 ssp_cvtepu16_epi32 ( ssp_m128* a)
{
	ssp_m128 A = *a;

	A.s32[3] = A.u16[3];
	A.s32[2] = A.u16[2];
	A.s32[1] = A.u16[1];
	A.s32[0] = A.u16[0];
	return A;
}

ssp_m128 ssp_cvtepu16_epi64 ( ssp_m128* a)
{
	ssp_m128 A = *a;

	A.s64[1] = A.u16[1];
	A.s64[0] = A.u16[0];
	return A;
}

ssp_m128 ssp_cvtepu32_epi64 ( ssp_m128* a)
{
	ssp_m128 A = *a;

	A.s64[1] = A.u32[1];
	A.s64[0] = A.u32[0];
	return A;
}

#define SSP_SET_MIN( sd, s) sd=(sd<s)?sd:s;
#define SSP_SET_MAX( sd, s) sd=(sd>s)?sd:s;

ssp_m128 ssp_blend_epi16( ssp_m128* a, ssp_m128* b, const int mask )
{
	ssp_m128 A, B;
	A = *a;
	B = *b;

	A.s16[0] = (mask & 0x01) ? B.s16[0] : A.s16[0];
	A.s16[1] = (mask & 0x02) ? B.s16[1] : A.s16[1];
	A.s16[2] = (mask & 0x04) ? B.s16[2] : A.s16[2];
	A.s16[3] = (mask & 0x08) ? B.s16[3] : A.s16[3];
	A.s16[4] = (mask & 0x10) ? B.s16[4] : A.s16[4];
	A.s16[5] = (mask & 0x20) ? B.s16[5] : A.s16[5];
	A.s16[6] = (mask & 0x40) ? B.s16[6] : A.s16[6];
	A.s16[7] = (mask & 0x80) ? B.s16[7] : A.s16[7];
	return A;
}

ssp_m128 ssp_blend_pd( ssp_m128* a, ssp_m128* b, const int mask )
{
	ssp_m128 A, B;
	A = *a;
	B = *b;

	A.f64[0] = (mask & 0x1) ? B.f64[0] : A.f64[0];
	A.f64[1] = (mask & 0x2) ? B.f64[1] : A.f64[1];

	return A;
}

ssp_m128 ssp_blend_ps( ssp_m128* a, ssp_m128* b, const int mask )
{
	ssp_m128 A, B;
	A = *a;
	B = *b;

	A.f32[0] = (mask & 0x1) ? B.f32[0] : A.f32[0];
	A.f32[1] = (mask & 0x2) ? B.f32[1] : A.f32[1];
	A.f32[2] = (mask & 0x4) ? B.f32[2] : A.f32[2];
	A.f32[3] = (mask & 0x8) ? B.f32[3] : A.f32[3];

	return A;
}

ssp_m128 ssp_blendv_epi8( ssp_m128* a, ssp_m128* b, ssp_m128* mask )
{
	ssp_m128 A, B, Mask;
	A = *a;
	B = *b;
	Mask = *mask;

	A.s8[0]  = (Mask.s8[0]  & 0x80) ? B.s8[0]  : A.s8[0];
	A.s8[1]  = (Mask.s8[1]  & 0x80) ? B.s8[1]  : A.s8[1];
	A.s8[2]  = (Mask.s8[2]  & 0x80) ? B.s8[2]  : A.s8[2];
	A.s8[3]  = (Mask.s8[3]  & 0x80) ? B.s8[3]  : A.s8[3];
	A.s8[4]  = (Mask.s8[4]  & 0x80) ? B.s8[4]  : A.s8[4];
	A.s8[5]  = (Mask.s8[5]  & 0x80) ? B.s8[5]  : A.s8[5];
	A.s8[6]  = (Mask.s8[6]  & 0x80) ? B.s8[6]  : A.s8[6];
	A.s8[7]  = (Mask.s8[7]  & 0x80) ? B.s8[7]  : A.s8[7];
	A.s8[8]  = (Mask.s8[8]  & 0x80) ? B.s8[8]  : A.s8[8];
	A.s8[9]  = (Mask.s8[9]  & 0x80) ? B.s8[9]  : A.s8[9];
	A.s8[10] = (Mask.s8[10] & 0x80) ? B.s8[10] : A.s8[10];
	A.s8[11] = (Mask.s8[11] & 0x80) ? B.s8[11] : A.s8[11];
	A.s8[12] = (Mask.s8[12] & 0x80) ? B.s8[12] : A.s8[12];
	A.s8[13] = (Mask.s8[13] & 0x80) ? B.s8[13] : A.s8[13];
	A.s8[14] = (Mask.s8[14] & 0x80) ? B.s8[14] : A.s8[14];
	A.s8[15] = (Mask.s8[15] & 0x80) ? B.s8[15] : A.s8[15];
	return A;
}

ssp_m128 ssp_blendv_pd( ssp_m128* a, ssp_m128* b, ssp_m128* mask )
{
	ssp_m128 A, B, Mask;
	A = *a;
	B = *b;
	Mask = *mask;

	A.f64[0] = (Mask.u64[0] & 0x8000000000000000ull) ? B.f64[0] : A.f64[0];
	A.f64[1] = (Mask.u64[1] & 0x8000000000000000ull) ? B.f64[1] : A.f64[1];

	return A;
}

ssp_m128 ssp_blendv_ps( ssp_m128* a, ssp_m128* b, ssp_m128* mask )
{
	ssp_m128 A, B, Mask;
	A = *a;
	B = *b;
	Mask = *mask;

	A.f32[0] = (Mask.u32[0] & 0x80000000) ? B.f32[0] : A.f32[0];
	A.f32[1] = (Mask.u32[1] & 0x80000000) ? B.f32[1] : A.f32[1];
	A.f32[2] = (Mask.u32[2] & 0x80000000) ? B.f32[2] : A.f32[2];
	A.f32[3] = (Mask.u32[3] & 0x80000000) ? B.f32[3] : A.f32[3];

	return A;
}

ssp_m128 ssp_min_epi8( ssp_m128* a, ssp_m128* b )
{
	ssp_m128 A,B;
	A = *a;
	B = *b;

	SSP_SET_MIN( A.s8[ 0], B.s8[ 0] );
	SSP_SET_MIN( A.s8[ 1], B.s8[ 1] );
	SSP_SET_MIN( A.s8[ 2], B.s8[ 2] );
	SSP_SET_MIN( A.s8[ 3], B.s8[ 3] );
	SSP_SET_MIN( A.s8[ 4], B.s8[ 4] );
	SSP_SET_MIN( A.s8[ 5], B.s8[ 5] );
	SSP_SET_MIN( A.s8[ 6], B.s8[ 6] );
	SSP_SET_MIN( A.s8[ 7], B.s8[ 7] );
	SSP_SET_MIN( A.s8[ 8], B.s8[ 8] );
	SSP_SET_MIN( A.s8[ 9], B.s8[ 9] );
	SSP_SET_MIN( A.s8[10], B.s8[10] );
	SSP_SET_MIN( A.s8[11], B.s8[11] );
	SSP_SET_MIN( A.s8[12], B.s8[12] );
	SSP_SET_MIN( A.s8[13], B.s8[13] );
	SSP_SET_MIN( A.s8[14], B.s8[14] );
	SSP_SET_MIN( A.s8[15], B.s8[15] );
	return A;
}

ssp_m128 ssp_max_epi8( ssp_m128* a, ssp_m128* b )
{
	ssp_m128 A,B;
	A = *a;
	B = *b;

	SSP_SET_MAX( A.s8[ 0], B.s8[ 0] );
	SSP_SET_MAX( A.s8[ 1], B.s8[ 1] );
	SSP_SET_MAX( A.s8[ 2], B.s8[ 2] );
	SSP_SET_MAX( A.s8[ 3], B.s8[ 3] );
	SSP_SET_MAX( A.s8[ 4], B.s8[ 4] );
	SSP_SET_MAX( A.s8[ 5], B.s8[ 5] );
	SSP_SET_MAX( A.s8[ 6], B.s8[ 6] );
	SSP_SET_MAX( A.s8[ 7], B.s8[ 7] );
	SSP_SET_MAX( A.s8[ 8], B.s8[ 8] );
	SSP_SET_MAX( A.s8[ 9], B.s8[ 9] );
	SSP_SET_MAX( A.s8[10], B.s8[10] );
	SSP_SET_MAX( A.s8[11], B.s8[11] );
	SSP_SET_MAX( A.s8[12], B.s8[12] );
	SSP_SET_MAX( A.s8[13], B.s8[13] );
	SSP_SET_MAX( A.s8[14], B.s8[14] );
	SSP_SET_MAX( A.s8[15], B.s8[15] );
	return A;
}

ssp_m128 ssp_min_epu16 ( ssp_m128* a, ssp_m128* b )
{
	ssp_m128 A,B;
	A = *a;
	B = *b;

	SSP_SET_MIN( A.u16[ 0], B.u16[ 0] );
	SSP_SET_MIN( A.u16[ 1], B.u16[ 1] );
	SSP_SET_MIN( A.u16[ 2], B.u16[ 2] );
	SSP_SET_MIN( A.u16[ 3], B.u16[ 3] );
	SSP_SET_MIN( A.u16[ 4], B.u16[ 4] );
	SSP_SET_MIN( A.u16[ 5], B.u16[ 5] );
	SSP_SET_MIN( A.u16[ 6], B.u16[ 6] );
	SSP_SET_MIN( A.u16[ 7], B.u16[ 7] );
	return A;
}

ssp_m128 ssp_max_epu16 ( ssp_m128* a, ssp_m128* b )
{
	ssp_m128 A,B;
	A = *a;
	B = *b;

	SSP_SET_MAX( A.u16[ 0], B.u16[ 0] );
	SSP_SET_MAX( A.u16[ 1], B.u16[ 1] );
	SSP_SET_MAX( A.u16[ 2], B.u16[ 2] );
	SSP_SET_MAX( A.u16[ 3], B.u16[ 3] );
	SSP_SET_MAX( A.u16[ 4], B.u16[ 4] );
	SSP_SET_MAX( A.u16[ 5], B.u16[ 5] );
	SSP_SET_MAX( A.u16[ 6], B.u16[ 6] );
	SSP_SET_MAX( A.u16[ 7], B.u16[ 7] );
	return A;
}

ssp_m128 ssp_min_epi32( ssp_m128* a, ssp_m128* b )
{
	ssp_m128 A,B;
	A = *a;
	B = *b;

	SSP_SET_MIN( A.s32[ 0], B.s32[ 0] );
	SSP_SET_MIN( A.s32[ 1], B.s32[ 1] );
	SSP_SET_MIN( A.s32[ 2], B.s32[ 2] );
	SSP_SET_MIN( A.s32[ 3], B.s32[ 3] );
	return A;
}

ssp_m128 ssp_max_epi32( ssp_m128* a, ssp_m128* b )
{
	ssp_m128 A,B;
	A = *a;
	B = *b;

	SSP_SET_MAX( A.s32[ 0], B.s32[ 0] );
	SSP_SET_MAX( A.s32[ 1], B.s32[ 1] );
	SSP_SET_MAX( A.s32[ 2], B.s32[ 2] );
	SSP_SET_MAX( A.s32[ 3], B.s32[ 3] );
	return A;
}

ssp_m128 ssp_min_epu32 ( ssp_m128* a, ssp_m128* b )
{
	ssp_m128 A,B;
	A = *a;
	B = *b;

	SSP_SET_MIN( A.u32[ 0], B.u32[ 0] );
	SSP_SET_MIN( A.u32[ 1], B.u32[ 1] );
	SSP_SET_MIN( A.u32[ 2], B.u32[ 2] );
	SSP_SET_MIN( A.u32[ 3], B.u32[ 3] );
	return A;
}

ssp_m128 ssp_max_epu32 ( ssp_m128* a, ssp_m128* b )
{
	ssp_m128 A,B;
	A = *a;
	B = *b;

	SSP_SET_MAX( A.u32[ 0], B.u32[ 0] );
	SSP_SET_MAX( A.u32[ 1], B.u32[ 1] );
	SSP_SET_MAX( A.u32[ 2], B.u32[ 2] );
	SSP_SET_MAX( A.u32[ 3], B.u32[ 3] );
	return A;
}

#undef SSP_SET_MIN
#undef SSP_SET_MAX

int ssp_testc_si128( ssp_m128* a, ssp_m128* b)
{
	ssp_m128 A,B;
	A = *a;
	B = *b;

	return ( (A.s64[0] & B.s64[0]) == A.s64[0] ) &&
		   ( (A.s64[1] & B.s64[1]) == A.s64[1] ) ;
}

int ssp_testz_si128( ssp_m128* a, ssp_m128* b)
{
	ssp_m128 A,B;
	A = *a;
	B = *b;

	return ( (A.s64[0] & B.s64[0]) == 0 ) &&
		   ( (A.s64[1] & B.s64[1]) == 0 ) ;
}

ssp_m128 ssp_cmpeq_epi64( ssp_m128* a, ssp_m128* b )
{
	ssp_m128 A, B;
	A = *a;
	B = *b;

	if( A.s64[0] == B.s64[0] )
		A.s64[0] = 0xFFFFFFFFFFFFFFFFll;
	else
		A.s64[0] = 0x0ll;

	if( A.s64[1] == B.s64[1] )
		A.s64[1] = 0xFFFFFFFFFFFFFFFFll;
	else
		A.s64[1] = 0x0ll;
	return A;
}

ssp_m128 ssp_packus_epi32( ssp_m128* a, ssp_m128* b )
{
	ssp_m128 A,B;
	A = *a;
	B = *b;

	if( A.s32[0] < 0 )
		A.u16[0] = 0;
	else
		if( A.s32[0] > 0xFFFF )
			A.u16[0] = 0xFFFF;
		else
			A.s16[0] = (u16)A.s32[0];

	if( A.s32[1] < 0 )
		A.u16[1] = 0;
	else
		if( A.s32[1] > 0xFFFF )
			A.u16[1] = 0xFFFF;
		else
			A.s16[1] = (u16)A.s32[1];

	if( A.s32[2] < 0 )
		A.u16[2] = 0;
	else
		if( A.s32[2] > 0xFFFF )
			A.u16[2] = 0xFFFF;
		else
			A.s16[2] = (u16)A.s32[2];


	if( A.s32[3] < 0 )
		A.u16[3] = 0;
	else
		if( A.s32[3] > 0xFFFF )
			A.u16[3] = 0xFFFF;
		else
			A.s16[3] = (u16)A.s32[3];

	if( B.s32[0] < 0 )
		A.u16[4] = 0;
	else
		if( B.s32[0] > 0xFFFF )
			A.u16[4] = 0xFFFF;
		else
			A.s16[4] = (u16)B.s32[0];

	if( B.s32[1] < 0 )
		A.u16[5] = 0;
	else
		if( B.s32[1] > 0xFFFF )
			A.u16[5] = 0xFFFF;
		else
			A.s16[5] = (u16)B.s32[1];

	if( B.s32[2] < 0 )
		A.u16[6] = 0;
	else
		if( B.s32[2] > 0xFFFF )
			A.u16[6] = 0xFFFF;
		else
			A.s16[6] = (u16)B.s32[2];


	if( B.s32[3] < 0 )
		A.u16[7] = 0;
	else
		if( B.s32[3] > 0xFFFF )
			A.u16[7] = 0xFFFF;
		else
			A.s16[7] = (u16)B.s32[3];

	return A;
}

ssp_m128 ssp_mpsadbw_epu8( ssp_m128* a,  ssp_m128* b,   const int msk  )
{
	u8 Abyte[11], Bbyte[4], tmp[4];
	u8 Boffset, Aoffset;
	int i;

	ssp_m128 A,B;
	A = *a;
	B = *b;

	Boffset = (msk & 0x3) << 2; // *32/8,   for byte size count
	Aoffset = (msk & 0x4);      // *32/8/4, for byte size count and shift msk to bit 2

	for (i=0; i<11; i++)
		Abyte[i] = A.u8[i+Aoffset];

	Bbyte[0] = B.u8[Boffset  ];
	Bbyte[1] = B.u8[Boffset+1];
	Bbyte[2] = B.u8[Boffset+2];
	Bbyte[3] = B.u8[Boffset+3];

	for (i=0; i<8; i++) {
		tmp[0] = (Abyte[i  ] > Bbyte[0]) ? (Abyte[i  ] - Bbyte[0]) :  (Bbyte[0] - Abyte[i  ]);        //abs diff
		tmp[1] = (Abyte[i+1] > Bbyte[1]) ? (Abyte[i+1] - Bbyte[1]) :  (Bbyte[1] - Abyte[i+1]);
		tmp[2] = (Abyte[i+2] > Bbyte[2]) ? (Abyte[i+2] - Bbyte[2]) :  (Bbyte[2] - Abyte[i+2]);
		tmp[3] = (Abyte[i+3] > Bbyte[3]) ? (Abyte[i+3] - Bbyte[3]) :  (Bbyte[3] - Abyte[i+3]);

		A.u16[i] = tmp[0] + tmp[1] + tmp[2] + tmp[3];
	}

	return A;
}

ssp_m128 ssp_mul_epi32( ssp_m128* a, ssp_m128* b )
{
	ssp_m128 A,B;
	A = *a;
	B = *b;

	A.s64[0] = A.s32[0] * B.s32[0];
	A.s64[1] = A.s32[2] * B.s32[2];
	return A;
}

ssp_m128 ssp_mullo_epi32( ssp_m128* a, ssp_m128* b )
{
	ssp_m128 t[2];
	ssp_m128 A,B;
	A = *a;
	B = *b;

	t[0].s64[0] = A.s32[0] * B.s32[0];
	t[0].s64[1] = A.s32[1] * B.s32[1];
	t[1].s64[0] = A.s32[2] * B.s32[2];
	t[1].s64[1] = A.s32[3] * B.s32[3];

	A.s32[0] = t[0].s32[0];
	A.s32[1] = t[0].s32[2];
	A.s32[2] = t[1].s32[0];
	A.s32[3] = t[1].s32[2];
	return A;
}

ssp_m128 ssp_minpos_epu16( ssp_m128* shortValues )
{
	ssp_m128 ShortValues;
	ShortValues = *shortValues;

	if( ShortValues.u16[1] < ShortValues.u16[0] ) {
		ShortValues.u16[0] = ShortValues.u16[1];
		ShortValues.u16[1] = 1;
	} else
		ShortValues.u16[1] = 0;


#define FN( I )                                     \
	if( ShortValues.u16[I] < ShortValues.u16[0] ) { \
		ShortValues.u16[0] = ShortValues.u16[I];    \
		ShortValues.u16[1] = I;                     \
	}

	FN( 2 );
	FN( 3 );
	FN( 4 );
	FN( 5 );
	FN( 6 );
	FN( 7 );

	ShortValues.u32[1] = 0;
	ShortValues.u64[1] = 0;

#undef FN

	return ShortValues;
}

ssp_m128 ssp_insert_epi8( ssp_m128* a, int b, const int ndx )       // Verify behavior on Intel Hardware
{
	ssp_m128 A;
	A = *a;

	A.s8[ndx & 0xF] = (s8)b;
	return A;
}

ssp_m128 ssp_insert_epi32( ssp_m128* a, int b, const int ndx )      // Verify behavior on Intel Hardware
{
	ssp_m128 A;
	A = *a;

	A.s32[ndx & 0x3] = b;
	return A;
}

ssp_m128 ssp_insert_epi64( ssp_m128* a, s64 b, const int ndx )  // Verify behavior on Intel Hardware
{
	ssp_m128 A;
	A = *a;

	A.s64[ndx & 0x1] = b;
	return A;
}

int ssp_extract_epi8( ssp_m128* a, const int ndx )
{
	ssp_m128 A;
	A = *a;
	return (int)A.u8[ndx&0xF];
}

int ssp_extract_epi32( ssp_m128* a, const int imm )
{
	ssp_m128 A;
	A = *a;
	return (int)A.u32[imm&0x3];
}

s64 ssp_extract_epi64( ssp_m128* a, const int ndx )
{
	ssp_m128 A;
	A = *a;
	return A.s64[ndx & 0x1];
}

ssp_m128 ssp_stream_load_si128( ssp_m128 *p )
{
	return *p;
}

unsigned short ssp_popcnt_16( unsigned short val )
{
	int i;
	u16 cnt=0;
	for( i=0; i<15, val; ++i, val = val>>1 )
		cnt += val & 0x1;
	return cnt;
}

unsigned int ssp_popcnt_32( unsigned int val )
{
	int i;
	u32 cnt = 0;
	for( i=0; i<31, val; ++i, val = val>>1 )
		cnt += val & 0x1;
	return cnt;
}

u64 ssp_popcnt_64( u64 val )
{
	int i;
	u64 cnt = 0;
	for( i=0; i<63, val; ++i, val = val>>1 )
		cnt += val & 0x1;
	return cnt;
}
