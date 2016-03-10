#include <linux/kernel.h>

#include <asm/stacktrace.h>
#include <asm/processor.h>

#include "SSEPlus_REF.h"

ssp_m128 getXMMRegister(int index, int extended)
{
	u8 buf[32];
	ssp_m128 *alignedValue = (ssp_m128 *)ALIGN((unsigned long)buf, 16);

#ifdef CONFIG_X86_64
	if (extended)
		switch (index) {
		case 0:
			asm volatile("movdqa %%xmm8, %0" : "=m"(*alignedValue));
			break;
		case 1:
			asm volatile("movdqa %%xmm9, %0" : "=m"(*alignedValue));
			break;
		case 2:
			asm volatile("movdqa %%xmm10, %0" : "=m"(*alignedValue));
			break;
		case 3:
			asm volatile("movdqa %%xmm11, %0" : "=m"(*alignedValue));
			break;
		case 4:
			asm volatile("movdqa %%xmm12, %0" : "=m"(*alignedValue));
			break;
		case 5:
			asm volatile("movdqa %%xmm13, %0" : "=m"(*alignedValue));
			break;
		case 6:
			asm volatile("movdqa %%xmm14, %0" : "=m"(*alignedValue));
			break;
		case 7:
			asm volatile("movdqa %%xmm15, %0" : "=m"(*alignedValue));
			break;
		}
	else
#endif
	switch (index) {
	case 0:
		asm volatile("movdqa %%xmm0, %0" : "=m"(*alignedValue));
		break;
	case 1:
		asm volatile("movdqa %%xmm1, %0" : "=m"(*alignedValue));
		break;
	case 2:
		asm volatile("movdqa %%xmm2, %0" : "=m"(*alignedValue));
		break;
	case 3:
		asm volatile("movdqa %%xmm3, %0" : "=m"(*alignedValue));
		break;
	case 4:
		asm volatile("movdqa %%xmm4, %0" : "=m"(*alignedValue));
		break;
	case 5:
		asm volatile("movdqa %%xmm5, %0" : "=m"(*alignedValue));
		break;
	case 6:
		asm volatile("movdqa %%xmm6, %0" : "=m"(*alignedValue));
		break;
	case 7:
		asm volatile("movdqa %%xmm7, %0" : "=m"(*alignedValue));
		break;
	}

	return *alignedValue;
}

void setXMMRegister(int index, int extended, ssp_m128* value)
{
	u8 buf[32];
	ssp_m128 *alignedValue = (ssp_m128 *)ALIGN((unsigned long)buf, 16);
	*alignedValue = *value;

#ifdef CONFIG_X86_64
	if (extended)
		switch (index) {
		case 0:
			asm volatile("movdqa %0, %%xmm8" : : "m" (*alignedValue));
			break;
		case 1:
			asm volatile("movdqa %0, %%xmm9" : : "m" (*alignedValue));
			break;
		case 2:
			asm volatile("movdqa %0, %%xmm10" : : "m" (*alignedValue));
			break;
		case 3:
			asm volatile("movdqa %0, %%xmm11" : : "m" (*alignedValue));
			break;
		case 4:
			asm volatile("movdqa %0, %%xmm12" : : "m" (*alignedValue));
			break;
		case 5:
			asm volatile("movdqa %0, %%xmm13" : : "m" (*alignedValue));
			break;
		case 6:
			asm volatile("movdqa %0, %%xmm14" : : "m" (*alignedValue));
			break;
		case 7:
			asm volatile("movdqa %0, %%xmm15" : : "m" (*alignedValue));
			break;
		}
	else
#endif
	switch (index) {
	case 0:
		asm volatile("movdqa %0, %%xmm0" : : "m" (*alignedValue));
		break;
	case 1:
		asm volatile("movdqa %0, %%xmm1" : : "m" (*alignedValue));
		break;
	case 2:
		asm volatile("movdqa %0, %%xmm2" : : "m" (*alignedValue));
		break;
	case 3:
		asm volatile("movdqa %0, %%xmm3" : : "m" (*alignedValue));
		break;
	case 4:
		asm volatile("movdqa %0, %%xmm4" : : "m" (*alignedValue));
		break;
	case 5:
		asm volatile("movdqa %0, %%xmm5" : : "m" (*alignedValue));
		break;
	case 6:
		asm volatile("movdqa %0, %%xmm6" : : "m" (*alignedValue));
		break;
	case 7:
		asm volatile("movdqa %0, %%xmm7" : : "m" (*alignedValue));
		break;
	}
}

unsigned long* getRegisterPtr(int index, struct pt_regs* regs, int extended) {
	unsigned long* regTable1[] = {
		&regs->ax, &regs->cx,
		&regs->dx, &regs->bx,
		&regs->sp, &regs->bp,
		&regs->si, &regs->di,
	};
	unsigned long** regTable = regTable1;
#ifdef CONFIG_X86_64
	unsigned long* regTable2[] = {
		&regs->r8, &regs->r9,
		&regs->r10, &regs->r11,
		&regs->r12, &regs->r13,
		&regs->r14, &regs->r15,
	};
	if (extended) {
		regTable = regTable2;
	}
#endif
	return regTable[index];
}

void setRegister(int index, struct pt_regs* regs, int extended, unsigned long value) {
	*getRegisterPtr(index, regs, extended) = value;
}

int decodeMemAddress(int opcode, struct pt_regs* regs, int rex, u8* extraBytes, unsigned long *memAddr) {
	int extraLength;
	int srcIndex = opcode & 0x7;
	if (srcIndex == 0x4) {
		int opHigh = (extraBytes[0]>>3) & 0x7;
		int opLow = extraBytes[0] & 0x7;
		if (opHigh == 0x4 && opLow == 0x04) {
			*memAddr = *getRegisterPtr(opLow, regs, testREX(rex, REX_B));
		} else {
			int multiplier = 1 << (extraBytes[0]>>6);
			*memAddr = *getRegisterPtr(opLow, regs, testREX(rex, REX_B)) +
						*getRegisterPtr(opHigh, regs, testREX(rex, REX_X)) * multiplier;
		}
		extraLength = 1;
	} else if (srcIndex == 0x5 && opcode < 0x40) {
		*memAddr = *(u32*)&extraBytes[0];
		extraLength = 4;
	} else {
		*memAddr = *getRegisterPtr(srcIndex, regs, testREX(rex, REX_B));
		extraLength = 0;
	}

	if (opcode & 0x40) {
		*memAddr += (s8)extraBytes[extraLength];
		extraLength += 1;
	} else if (opcode & 0x80) {
		*memAddr += *(s64*)&extraBytes[extraLength];
		extraLength += 4;
	}

	return extraLength;
}

int getOp2MemValue(int opcode, struct pt_regs* regs, int rex, u8* extraBytes, unsigned long* value) {
	if (opcode >= 0xc0) {
		*value = *getRegisterPtr(opcode & 0x7, regs, testREX(rex, REX_B));
		return 0;
	} else {
		unsigned long memAddr = 0;
		u8 data[sizeof(unsigned long)] = {0};
		int extraLen = decodeMemAddress(opcode, regs, rex, extraBytes, &memAddr);

		if (memAddr) {
			copy_from_user((void *)data, (const void __user *)memAddr, sizeof(unsigned long));
			*value = *(unsigned long*)data;
			return extraLen;
		}
	}
	return -1;
}

int getOp2XMMValue(int opcode, struct pt_regs* regs, int rex, u8* extraBytes, ssp_m128* value) {
	if (opcode >= 0xc0) {
		*value = getXMMRegister(opcode & 0x7, testREX(rex, REX_B));
		return 0;
	} else {
		unsigned long memAddr = 0;
		u8 data[sizeof(ssp_m128)] = {0};
		int extraLen = decodeMemAddress(opcode, regs, rex, extraBytes, &memAddr);

		if (memAddr) {
			copy_from_user((void *)data, (const void __user *)memAddr, sizeof(ssp_m128));
			*value = *(ssp_m128*)data;
			return extraLen;
		}
	}
	return -1;
}
