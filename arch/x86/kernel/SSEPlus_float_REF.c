#include <linux/kernel.h>

#include <asm/fpu/internal.h>

#include "SSEPlus_REF.h"

#define SSP_FROUND_TO_NEAREST_INT    0x00
#define SSP_FROUND_TO_NEG_INF        0x01
#define SSP_FROUND_TO_POS_INF        0x02
#define SSP_FROUND_TO_ZERO           0x03
#define SSP_FROUND_CUR_DIRECTION     0x04

#define SSP_FROUND_RAISE_EXC         0x00
#define SSP_FROUND_NO_EXC            0x08

/** @addtogroup supplimental_REF
 *  @{
 *  @name Number Operations{
 */

static
int ssp_number_isValidNumber_F32_REF( s32* val )//TODO: move into utility collection
{
	// Check for NAN, +infin, or -infin (exponent: 111 1111 1)
	// Are the exponent bits all 1's?
	if( (*val & 0x7F800000) == 0x7F800000 ) {
		return 0;
	}
	return 1;
}

static
int ssp_number_isValidNumber_F64_REF( s64* val )   //TODO: move into utility collection
{
	// Check for NAN, +infin, or -infin (exponent: 1111 1111)
	// Are the exponent bits all 1's?
	if( (*val & 0x7FF0000000000000ll) == 0x7FF0000000000000ll ) {
		return 0;
	}
	return 1;
}

static
ssp_f32 ssp_number_changeSNanToQNaN_F32_REF( s32* val )//TODO: move into utility collection
{
	ssp_f32* retVal = (ssp_f32*)val;
	// Check if the value is already a QNaN
	if( (*val & 0x00400000) != 0x00400000 ) {
		// Check if the value is + or - infinitie
		if( (*val | 0x7F800000) != 0x7F800000 ) {
			// Convert SNan To QNaN
			*retVal = (ssp_f32)( *val | 0x00400000 );
		}
	}
	return *retVal;
}

static
ssp_f64 ssp_number_changeSNanToQNaN_F64_REF( s64* val )//TODO: move into utility collection
{
	ssp_f64* retVal = (ssp_f64*)val;
	// Check if the value is already a QNaN
	if( (*val & 0x0008000000000000ll) != 0x0008000000000000ll ) {
		// Check if the value is + or - infinitie
		if( (*val | 0x7FF0000000000000ll) != 0x7FF0000000000000ll ) {
			// Convert SNan To QNaN
			*retVal = (ssp_f64)( *val | 0x0008000000000000ll );
		}
	}
	return *retVal;
}


#define __HI(x) *(1+(int*)&x)
#define __LO(x) *(int*)&x

static const ssp_f64 huge = 1.0e300;

static
ssp_f64 ssp_ceil(ssp_f64 x)
{
	int i0,i1,j0;
	unsigned i,j;
	i0 =  __HI(x);
	i1 =  __LO(x);
	j0 = ((i0>>20)&0x7ff)-0x3ff;
	if(j0<20) {
		if(j0<0) {	/* raise inexact if x != 0 */
			if(huge+x>0.0) {/* return 0*sign(x) if |x|<1 */
				if(i0<0) {
					i0=0x80000000;
					i1=0;
				} else if((i0|i1)!=0) {
					i0=0x3ff00000;
					i1=0;
				}
			}
		} else {
			i = (0x000fffff)>>j0;
			if(((i0&i)|i1)==0)
				return x; /* x is integral */
			if(huge+x>0.0) {	/* raise inexact flag */
				if(i0>0)
					i0 += (0x00100000)>>j0;
				i0 &= (~i); i1=0;
			}
		}
	} else if (j0>51) {
		if(j0==0x400)
			return x+x;	/* inf or NaN */
		else
			return x;		/* x is integral */
	} else {
		i = ((unsigned)(0xffffffff))>>(j0-20);
		if((i1&i)==0)
			return x;	/* x is integral */
		if(huge+x>0.0) {		/* raise inexact flag */
			if(i0>0) {
				if(j0==20)
					i0+=1;
				else {
					j = i1 + (1<<(52-j0));
					if(j<i1)
						i0+=1;	/* got a carry */
					i1 = j;
				}
			}
			i1 &= (~i);
		}
	}
	__HI(x) = i0;
	__LO(x) = i1;
	return x;
}

static
ssp_f64 ssp_floor(ssp_f64 x)
{
	int i0,i1,j0;
	unsigned i,j;
	i0 =  __HI(x);
	i1 =  __LO(x);
	j0 = ((i0>>20)&0x7ff)-0x3ff;
	if(j0<20) {
		if(j0<0) {	/* raise inexact if x != 0 */
			if(huge+x>0.0) {/* return 0*sign(x) if |x|<1 */
				if(i0>=0)
					i0=i1=0;
				else if(((i0&0x7fffffff)|i1)!=0) {
					i0=0xbff00000;
					i1=0;
				}
			}
		} else {
			i = (0x000fffff)>>j0;
			if(((i0&i)|i1)==0)
				return x; /* x is integral */
			if(huge+x>0.0) {	/* raise inexact flag */
				if(i0<0)
					i0 += (0x00100000)>>j0;
				i0 &= (~i); i1=0;
			}
		}
	} else if (j0>51) {
		if(j0==0x400)
			return x+x;	/* inf or NaN */
		else
			return x;		/* x is integral */
	} else {
		i = ((unsigned)(0xffffffff))>>(j0-20);
		if((i1&i)==0)
			return x;	/* x is integral */
		if(huge+x>0.0) {		/* raise inexact flag */
			if(i0<0) {
				if(j0==20)
					i0+=1;
				else {
					j = i1+(1<<(52-j0));
					if(j<i1)
						i0 +=1 ;	/* got a carry */
					i1=j;
				}
			}
			i1 &= (~i);
		}
	}
	__HI(x) = i0;
	__LO(x) = i1;
	return x;
}

ssp_m128 ssp_round_pd( ssp_m128* val, int iRoundMode )
{
	s64 *valPtr;
	ssp_m128 Val;
	Val = *val;

	kernel_fpu_begin();

	switch( iRoundMode & 0x3 )
	{
	case SSP_FROUND_CUR_DIRECTION:
		break;
	case SSP_FROUND_TO_ZERO:
		valPtr = (s64*)(&Val.f64[0]);
		if( ssp_number_isValidNumber_F64_REF( valPtr ) )
			Val.f64[0] = (ssp_f64)( (s64)Val.f64[0] );

		valPtr = (s64*)(&Val.f64[1]);
		if( ssp_number_isValidNumber_F64_REF( valPtr ) )
			Val.f64[1] = (ssp_f64)( (s64)Val.f64[1] );
		break;
	case SSP_FROUND_TO_POS_INF:
		valPtr = (s64*)(&Val.f64[0]);
		if( ssp_number_isValidNumber_F64_REF( valPtr ) )
			Val.f64[0] = ssp_ceil( Val.f64[0] );

		valPtr = (s64*)(&Val.f64[1]);
		if( ssp_number_isValidNumber_F64_REF( valPtr ) )
			Val.f64[1] = ssp_ceil( Val.f64[1] );
		break;
	case SSP_FROUND_TO_NEG_INF:
		valPtr = (s64*)(&Val.f64[0]);
		if( ssp_number_isValidNumber_F64_REF( valPtr ) )
			Val.f64[0] = ssp_floor( Val.f64[0] );

		valPtr = (s64*)(&Val.f64[1]);
		if( ssp_number_isValidNumber_F64_REF( valPtr ) )
			Val.f64[1] = ssp_floor( Val.f64[1] );
		break;
	default: // SSP_FROUND_TO_NEAREST_INT
		valPtr = (s64*)(&Val.f64[0]);
		if( ssp_number_isValidNumber_F64_REF( valPtr ) )
			Val.f64[0] = (ssp_f64)( (Val.f64[0]>0) ? (s64)(Val.f64[0]+0.5) : (s64)(Val.f64[0]-0.5) );
		else
			Val.f64[0] = ssp_number_changeSNanToQNaN_F64_REF( valPtr );

		valPtr = (s64*)(&Val.f64[1]);
		if( ssp_number_isValidNumber_F64_REF( valPtr ) )
			Val.f64[1] = (ssp_f64)( (Val.f64[1]>0) ? (s64)(Val.f64[1]+0.5) : (s64)(Val.f64[1]-0.5) );
		else
			Val.f64[1] = ssp_number_changeSNanToQNaN_F64_REF( valPtr );
	}

	kernel_fpu_end();

	return Val;
}

ssp_m128 ssp_round_ps( ssp_m128* val, int iRoundMode )
{
	s32 *valPtr;
	ssp_m128 Val;
	Val = *val;

	kernel_fpu_begin();

	switch( iRoundMode & 0x3 )
	{
	case SSP_FROUND_CUR_DIRECTION:
		break;
	case SSP_FROUND_TO_ZERO:
		valPtr = (s32*)(&Val.f32[0]);
		if( ssp_number_isValidNumber_F32_REF( valPtr ) ) {
			if( Val.f32[0] >= 0 )
				Val.f32[0] = (ssp_f32)( (s32)Val.f32[0] );
			else {
				Val.f32[0] = (ssp_f32)( (s32)Val.f32[0] );
				//Val.s32[0] = Val.s32[0] | 0x80000000;
			}
		}

		valPtr = (s32*)(&Val.f32[1]);
		if( ssp_number_isValidNumber_F32_REF( valPtr ) ) {
			if( Val.f32[1] >= 0 )
				Val.f32[1] = (ssp_f32)( (s32)Val.f32[1] );
			else {
				Val.f32[1] = (ssp_f32)( (s32)Val.f32[1] );
				//Val.s32[1] = Val.s32[1] | 0x80000000;
			}
		}

		valPtr = (s32*)(&Val.f32[2]);
		if( ssp_number_isValidNumber_F32_REF( valPtr ) ) {
			if( Val.f32[2] >= 0 )
				Val.f32[2] = (ssp_f32)( (s32)Val.f32[2] );
			else {
				Val.f32[2] = (ssp_f32)( (s32)Val.f32[2] );
				//Val.s32[2] = Val.s32[2] | 0x80000000;
			}
		}

		valPtr = (s32*)(&Val.f32[3]);
		if( ssp_number_isValidNumber_F32_REF( valPtr ) ) {
			if( Val.f32[3] >= 0 )
				Val.f32[3] = (ssp_f32)( (s32)Val.f32[3] );
			else {
				Val.f32[3] = (ssp_f32)( (s32)Val.f32[3] );
				//Val.s32[3] = Val.s32[3] | 0x80000000;
			}
		}
		break;
	case SSP_FROUND_TO_POS_INF:
		valPtr = (s32*)(&Val.f32[0]);
		if( ssp_number_isValidNumber_F32_REF( valPtr ) )
			Val.f32[0] = (ssp_f32)ssp_ceil( Val.f32[0] );

		valPtr = (s32*)(&Val.f32[1]);
		if( ssp_number_isValidNumber_F32_REF( valPtr ) )
			Val.f32[1] = (ssp_f32)ssp_ceil( Val.f32[1] );

		valPtr = (s32*)(&Val.f32[2]);
		if( ssp_number_isValidNumber_F32_REF( valPtr ) )
			Val.f32[2] = (ssp_f32)ssp_ceil( Val.f32[2] );

		valPtr = (s32*)(&Val.f32[3]);
		if( ssp_number_isValidNumber_F32_REF( valPtr ) )
			Val.f32[3] = (ssp_f32)ssp_ceil( Val.f32[3] );
		break;
	case SSP_FROUND_TO_NEG_INF:
		valPtr = (s32*)(&Val.f32[0]);
		if( ssp_number_isValidNumber_F32_REF( valPtr ) )
			Val.f32[0] = (ssp_f32)ssp_floor( Val.f32[0] );

		valPtr = (s32*)(&Val.f32[1]);
		if( ssp_number_isValidNumber_F32_REF( valPtr ) )
			Val.f32[1] = (ssp_f32)ssp_floor( Val.f32[1] );

		valPtr = (s32*)(&Val.f32[2]);
		if( ssp_number_isValidNumber_F32_REF( valPtr ) )
			Val.f32[2] = (ssp_f32)ssp_floor( Val.f32[2] );

		valPtr = (s32*)(&Val.f32[3]);
		if( ssp_number_isValidNumber_F32_REF( valPtr ) )
			Val.f32[3] = (ssp_f32)ssp_floor( Val.f32[3] );
		break;
	default: // SSP_FROUND_TO_NEAREST_INT
		valPtr = (s32*)(&Val.f32[0]);
		if( ssp_number_isValidNumber_F32_REF( valPtr ) )
			Val.f32[0] = (ssp_f32)( (Val.f32[0]>0) ? (s32)(Val.f32[0]+0.5) : (s32)(Val.f32[0]-0.5) );
		else
			Val.f32[0] = ssp_number_changeSNanToQNaN_F32_REF( valPtr );

		valPtr = (s32*)(&Val.f32[1]);
		if( ssp_number_isValidNumber_F32_REF( valPtr ) )
			Val.f32[1] = (ssp_f32)( (Val.f32[1]>0) ? (s32)(Val.f32[1]+0.5) : (s32)(Val.f32[1]-0.5) );
		else
			Val.f32[1] = ssp_number_changeSNanToQNaN_F32_REF( valPtr );

		valPtr = (s32*)(&Val.f32[2]);
		if( ssp_number_isValidNumber_F32_REF( valPtr ) )
			Val.f32[2] = (ssp_f32)( (Val.f32[2]>0) ? (s32)(Val.f32[2]+0.5) : (s32)(Val.f32[2]-0.5) );
		else
			Val.f32[2] = ssp_number_changeSNanToQNaN_F32_REF( valPtr );

		valPtr = (s32*)(&Val.f32[3]);
		if( ssp_number_isValidNumber_F32_REF( valPtr ) )
			Val.f32[3] = (ssp_f32)( (Val.f32[3]>0) ? (s32)(Val.f32[3]+0.5) : (s32)(Val.f32[3]-0.5) );
		else
			Val.f32[3] = ssp_number_changeSNanToQNaN_F32_REF( valPtr );
	}

	if( -0.0f == Val.f32[0] ) Val.f32[0]=+0.0f;
	if( -0.0f == Val.f32[1] ) Val.f32[1]=+0.0f;
	if( -0.0f == Val.f32[2] ) Val.f32[2]=+0.0f;
	if( -0.0f == Val.f32[3] ) Val.f32[3]=+0.0f;

	kernel_fpu_end();

	return Val;
}

ssp_m128 ssp_round_sd( ssp_m128* dst, ssp_m128* val, int iRoundMode )
{
	s64 *valPtr;
	ssp_m128 Dst, Val;
	Dst = *dst;
	Val = *val;

	kernel_fpu_begin();

	switch( iRoundMode & 0x3 )
	{
	case SSP_FROUND_CUR_DIRECTION:
		break;
	case SSP_FROUND_TO_ZERO:
		valPtr = (s64*)(&Val.f64[0]);
		if( ssp_number_isValidNumber_F64_REF( valPtr ) )
			Dst.f64[0] = (ssp_f64)( (s64)Val.f64[0] );
		break;
	case SSP_FROUND_TO_POS_INF:
		valPtr = (s64*)(&Val.f64[0]);
		if( ssp_number_isValidNumber_F64_REF( valPtr ) )
			Dst.f64[0] = ssp_ceil( Val.f64[0] );
		break;
	case SSP_FROUND_TO_NEG_INF:
		valPtr = (s64*)(&Val.f64[0]);
		if( ssp_number_isValidNumber_F64_REF( valPtr ) )
			Dst.f64[0] = ssp_floor( Val.f64[0] );
		break;
	default: // SSP_FROUND_TO_NEAREST_INT
		valPtr = (s64*)(&Val.f64[0]);
		if( ssp_number_isValidNumber_F64_REF( valPtr ) )
			Dst.f64[0] = (ssp_f64)( (Val.f64[0]>0) ? (s64)(Val.f64[0]+0.5) : (s64)(Val.f64[0]-0.5) );
		else
			Dst.f64[0] = ssp_number_changeSNanToQNaN_F64_REF( valPtr );
	}

	kernel_fpu_end();

	return Dst;
}

ssp_m128 ssp_round_ss( ssp_m128* dst, ssp_m128* val, int iRoundMode )        //_mm_round_ss
{
	s32 *valPtr;
	ssp_m128 Dst, Val;
	Dst = *dst;
	Val = *val;

	kernel_fpu_begin();

	switch( iRoundMode & 0x3 )
	{
	case SSP_FROUND_CUR_DIRECTION:
		break;
	case SSP_FROUND_TO_ZERO:
		valPtr = (s32*)(&Val.f32[0]);
		if( ssp_number_isValidNumber_F32_REF( valPtr ) ) {
			Dst.f32[0] = (ssp_f32)( (s32)Val.f32[0] );
			if( Val.f32[0] <= -0 )
				Dst.s32[0] = Dst.s32[0] | 0x80000000;
		}
		break;
	case SSP_FROUND_TO_POS_INF:
		valPtr = (s32*)(&Val.f32[0]);
		if( ssp_number_isValidNumber_F32_REF( valPtr ) )
			Dst.f32[0] = (ssp_f32)ssp_ceil( Val.f32[0] );
		break;
	case SSP_FROUND_TO_NEG_INF:
		valPtr = (s32*)(&Val.f32[0]);
		if( ssp_number_isValidNumber_F32_REF( valPtr ) )
			Dst.f32[0] = (ssp_f32)ssp_floor( Val.f32[0] );
		break;
	default: // SSP_FROUND_TO_NEAREST_INT
		valPtr = (s32*)(&Val.f32[0]);
		if( ssp_number_isValidNumber_F32_REF( valPtr ) )
			Dst.f32[0] = (ssp_f32)( (Val.f32[0]>0) ? (s32)(Val.f32[0]+0.5) : (s32)(Val.f32[0]-0.5) );
		else
			Dst.f32[0] = ssp_number_changeSNanToQNaN_F32_REF( valPtr );
	}

	kernel_fpu_end();

	return Dst;
}

ssp_m128 ssp_dp_pd( ssp_m128* a, ssp_m128* b, const int mask )
{
	ssp_f64 tmp[3];
	ssp_m128 A, B;
	A = *a;
	B = *b;

	kernel_fpu_begin();

	tmp[0] = (mask & 0x10) ? (A.f64[0] * B.f64[0]) : 0.0;
	tmp[1] = (mask & 0x20) ? (A.f64[1] * B.f64[1]) : 0.0;

	tmp[2] = tmp[0] + tmp[1];

	A.f64[0] = (mask & 0x1) ? tmp[2] : 0.0;
	A.f64[1] = (mask & 0x2) ? tmp[2] : 0.0;

	kernel_fpu_end();

	return A;
}

ssp_m128 ssp_dp_ps( ssp_m128* a, ssp_m128* b, const int mask )
{
	ssp_f32 tmp[5];
	ssp_m128 A, B;
	A = *a;
	B = *b;

	kernel_fpu_begin();

	tmp[0] = (mask & 0x10) ? (A.f32[0] * B.f32[0]) : 0.0f;
	tmp[1] = (mask & 0x20) ? (A.f32[1] * B.f32[1]) : 0.0f;
	tmp[2] = (mask & 0x40) ? (A.f32[2] * B.f32[2]) : 0.0f;
	tmp[3] = (mask & 0x80) ? (A.f32[3] * B.f32[3]) : 0.0f;

	tmp[4] = tmp[0] + tmp[1] + tmp[2] + tmp[3];

	A.f32[0] = (mask & 0x1) ? tmp[4] : 0.0f;
	A.f32[1] = (mask & 0x2) ? tmp[4] : 0.0f;
	A.f32[2] = (mask & 0x4) ? tmp[4] : 0.0f;
	A.f32[3] = (mask & 0x8) ? tmp[4] : 0.0f;

	kernel_fpu_end();

	return A;
}

ssp_m128 ssp_insert_ps( ssp_m128* a, ssp_m128* b, const int sel )          // Verify behavior on Intel Hardware
{
	ssp_f32 tmp;
	int count_d,zmask;

	kernel_fpu_begin();

	ssp_m128 A,B;
	A = *a;
	B = *b;

	tmp     = B.f32[(sel & 0xC0)>>6];   // 0xC0 = sel[7:6]
	count_d = (sel & 0x30)>>4;          // 0x30 = sel[5:4]
	zmask   = sel & 0x0F;               // 0x0F = sel[3:0]

	A.f32[count_d] = tmp;

	A.f32[0] = (zmask & 0x1) ? 0 : A.f32[0];
	A.f32[1] = (zmask & 0x2) ? 0 : A.f32[1];
	A.f32[2] = (zmask & 0x4) ? 0 : A.f32[2];
	A.f32[3] = (zmask & 0x8) ? 0 : A.f32[3];

	kernel_fpu_end();

	return A;
}

int ssp_extract_ps( ssp_m128* a, const int ndx )
{
	ssp_m128 A;

	kernel_fpu_begin();

	A.f32[ndx&0x3] = a->f32[ndx&0x3];

	kernel_fpu_end();

	return A.s32[ndx&0x3];
}
