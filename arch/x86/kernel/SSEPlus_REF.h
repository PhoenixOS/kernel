#ifndef __SSEPLUS_NUMBER_REF_H__
#define __SSEPLUS_NUMBER_REF_H__



typedef float              ssp_f32;
typedef double             ssp_f64;

typedef union {
	u64 u64[2];
	s64 s64[2];
	ssp_f64 f64[2];
	u32 u32[4];
	s32 s32[4];
	ssp_f32 f32[4];
	u16 u16[8];
	s16 s16[8];
	u8 u8[16];
	s8 s8[16];
} ssp_m128 __aligned(16);

struct pt_regs;

// SSSE3

void ssp_abs_epi8(ssp_m128 *A);
void ssp_abs_epi16(ssp_m128 *A);
void ssp_abs_epi32(ssp_m128 *A);
ssp_m128 ssp_shuffle_epi8(ssp_m128 *A, ssp_m128 *MSK);
void ssp_alignr_epi8(ssp_m128 *ret, ssp_m128 *a, ssp_m128 *b, const unsigned int ralign);
ssp_m128 ssp_sign_epi8(ssp_m128* a, ssp_m128* b);
ssp_m128 ssp_sign_epi16(ssp_m128* a, ssp_m128* b);
ssp_m128 ssp_sign_epi32(ssp_m128* a, ssp_m128* b);
ssp_m128 ssp_mulhrs_epi16( ssp_m128* a, ssp_m128* b );
ssp_m128 ssp_maddubs_epi16( ssp_m128* a,  ssp_m128* b);
ssp_m128 ssp_hsub_epi16( ssp_m128* a, ssp_m128* b );
ssp_m128 ssp_hsub_epi32( ssp_m128* a, ssp_m128* b );
ssp_m128 ssp_hsubs_epi16( ssp_m128* a, ssp_m128* b );
ssp_m128 ssp_hadd_epi16( ssp_m128* a, ssp_m128* b );
ssp_m128 ssp_hadd_epi32( ssp_m128* a, ssp_m128* b );
ssp_m128 ssp_hadds_epi16( ssp_m128* a, ssp_m128* b );



// SSE4.1

ssp_m128 ssp_cvtepi8_epi16 ( ssp_m128* a);
ssp_m128 ssp_cvtepi8_epi32 ( ssp_m128* a);
ssp_m128 ssp_cvtepi8_epi64 ( ssp_m128* a);
ssp_m128 ssp_cvtepi16_epi32 ( ssp_m128* a);
ssp_m128 ssp_cvtepi16_epi64( ssp_m128* a);
ssp_m128 ssp_cvtepi32_epi64 ( ssp_m128* a);
ssp_m128 ssp_cvtepu8_epi16 ( ssp_m128* a);
ssp_m128 ssp_cvtepu8_epi32 ( ssp_m128* a);
ssp_m128 ssp_cvtepu8_epi64 ( ssp_m128* a);
ssp_m128 ssp_cvtepu16_epi32 ( ssp_m128* a);
ssp_m128 ssp_cvtepu16_epi64 ( ssp_m128* a);
ssp_m128 ssp_cvtepu32_epi64 ( ssp_m128* a);
ssp_m128 ssp_blend_epi16( ssp_m128* a, ssp_m128* b, const int mask );
ssp_m128 ssp_blend_pd( ssp_m128* a, ssp_m128* b, const int mask );
ssp_m128 ssp_blend_ps( ssp_m128* a, ssp_m128* b, const int mask );
ssp_m128 ssp_blendv_epi8( ssp_m128* a, ssp_m128* b, ssp_m128* mask );
ssp_m128 ssp_blendv_pd( ssp_m128* a, ssp_m128* b, ssp_m128* mask );
ssp_m128 ssp_blendv_ps( ssp_m128* a, ssp_m128* b, ssp_m128* mask );
ssp_m128 ssp_min_epi8( ssp_m128* a, ssp_m128* b );
ssp_m128 ssp_max_epi8( ssp_m128* a, ssp_m128* b );
ssp_m128 ssp_min_epu16 ( ssp_m128* a, ssp_m128* b );
ssp_m128 ssp_max_epu16 ( ssp_m128* a, ssp_m128* b );
ssp_m128 ssp_min_epi32( ssp_m128* a, ssp_m128* b );
ssp_m128 ssp_max_epi32( ssp_m128* a, ssp_m128* b );
ssp_m128 ssp_min_epu32 ( ssp_m128* a, ssp_m128* b );
ssp_m128 ssp_max_epu32 ( ssp_m128* a, ssp_m128* b );
int ssp_testc_si128( ssp_m128* a, ssp_m128* b);
int ssp_testz_si128( ssp_m128* a, ssp_m128* b);
ssp_m128 ssp_cmpeq_epi64( ssp_m128* a, ssp_m128* b );
ssp_m128 ssp_packus_epi32( ssp_m128* a, ssp_m128* b );
ssp_m128 ssp_mpsadbw_epu8( ssp_m128* a,  ssp_m128* b,   const int msk  );
ssp_m128 ssp_mul_epi32( ssp_m128* a, ssp_m128* b );
ssp_m128 ssp_mullo_epi32( ssp_m128* a, ssp_m128* b );
ssp_m128 ssp_minpos_epu16( ssp_m128* shortValues );
ssp_m128 ssp_insert_epi8( ssp_m128* a, int b, const int ndx );
ssp_m128 ssp_insert_epi32( ssp_m128* a, int b, const int ndx );
ssp_m128 ssp_insert_epi64( ssp_m128* a, s64 b, const int ndx );
int ssp_extract_epi8( ssp_m128* a, const int ndx );
int ssp_extract_epi32( ssp_m128* a, const int imm );
s64 ssp_extract_epi64( ssp_m128* a, const int ndx );
ssp_m128 ssp_stream_load_si128( ssp_m128 *p );

// SSE4.1 floating point

ssp_m128 ssp_round_pd( ssp_m128* val, int iRoundMode );
ssp_m128 ssp_round_ps( ssp_m128* val, int iRoundMode );
ssp_m128 ssp_round_sd( ssp_m128* dst, ssp_m128* val, int iRoundMode );
ssp_m128 ssp_round_ss( ssp_m128* dst, ssp_m128* val, int iRoundMode );
ssp_m128 ssp_dp_pd( ssp_m128* a, ssp_m128* b, const int mask );
ssp_m128 ssp_dp_ps( ssp_m128* a, ssp_m128* b, const int mask );
ssp_m128 ssp_insert_ps( ssp_m128* a, ssp_m128* b, const int sel );
int ssp_extract_ps( ssp_m128* a, const int ndx );


// popcnt

unsigned short ssp_popcnt_16( unsigned short val );
unsigned int ssp_popcnt_32( unsigned int val );
u64 ssp_popcnt_64( u64 val );

// utils

ssp_m128 getXMMRegister(int index, int extended);
void setXMMRegister(int index, int extended, ssp_m128* value);

unsigned long* getRegisterPtr(int index, struct pt_regs* regs, int extended);
void setRegister(int index, struct pt_regs* regs, int extended, unsigned long value);

int decodeMemAddress(int opcode, struct pt_regs* regs, int rex, u8* extraBytes, unsigned long *memAddr);

int getOp2MemValue(int opcode, struct pt_regs* regs, int rex, u8* extraBytes, unsigned long* value);
int getOp2XMMValue(int opcode, struct pt_regs* regs, int rex, u8* extraBytes, ssp_m128* value);

#define REX_B 0x41
#define REX_X 0x42
#define REX_R 0x44
#define REX_W 0x48

static inline
int testREX(int rex, int mask)
{
	return (rex & mask) == mask;
}


/** @}
 *  @}
 */

#endif // __SSEPLUS_NUMBER_REF_H__
