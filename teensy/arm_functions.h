#ifndef ARMFUNCTIONS_H
#define ARMFUNCTIONS_H

static inline uint32_t arm_bit_clear(uint32_t val,uint32_t idx) __attribute__((always_inline, unused));
static inline uint32_t arm_bit_clear(uint32_t val,uint32_t idx)
{
	uint32_t out;
	asm volatile("lsls %0, %1, %2" : "=r" (out) : "r" (0x01), "r" (idx));
	asm volatile("bics %0, %1, %2" : "=r" (out) : "r" (val), "r" (out));
	return out;
}

static inline uint32_t arm_bit_set(uint32_t val,uint32_t idx) __attribute__((always_inline, unused));
static inline uint32_t arm_bit_set(uint32_t val,uint32_t idx)
{
	uint32_t out;
	asm volatile("lsls %0, %1, %2" : "=r" (out) : "r" (0x01), "r" (idx));
	asm volatile("orrs  %0, %1, %2" : "=r" (out) : "r" (val), "r" (out));
	return out;
}


//Floating-point Fused Multiply Accumulate and Subtract.
//VFMA{cond}.F32 {Sd,} Sn, Sm
//VFMS
static inline float arm_fp_multiply_subtract(float n1,float n2,float n3,float n4) __attribute__((always_inline, unused));
static inline float arm_fp_multiply_subtract(float n1,float n2,float n3,float n4)
{
	float res,res1,res2;
	asm volatile("vmul %0, %1, %2" : "=r" (res1) : "r" (n1), "r" (n2));
	asm volatile("vmul  %0, %1, %2" : "=r" (res2) : "r" (n3), "r" (n4));
	asm volatile("vsub  %0, %1, %2" : "=r" (res) : "r" (res1), "r" (res2));

	return res;
}

#endif //ARMFUNCTIONS_H
