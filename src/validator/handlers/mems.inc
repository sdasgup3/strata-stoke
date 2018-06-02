/*
   The follwoings 61 are supported by both Strata and stoke handler.
   So tested their equivalence using z3 and keep the simpler
   stoke ones.

   The following is the att opcode names of the same
adcb_m8_imm8
adcl_m32_imm8
adcb_m8_r8
adcb_r8_m8
adcl_m32_imm32
adcl_m32_r32
adcq_m64_r64
adcq_m64_imm8
adcl_r32_m32
adcq_m64_imm32
adcq_r64_m64
adcw_m16_imm8
adcw_m16_imm16
adcw_m16_r16
adcw_r16_m16
cmoveq_r64_m64
movb_m8_rh
movb_rh_m8
movq_m64_imm32
movq_m64_r64
movq_r64_m64
movsbq_r64_m8
movslq_r64_m32
movswq_r64_m16
orq_m64_imm32
orq_m64_imm8
orq_m64_r64
orq_r64_m64
popcntq_r64_m64
salq_m64_cl
salq_m64_imm8
sarq_m64_cl
sarq_m64_imm8
shrq_m64_cl
shrq_m64_imm8
vaddpd_ymm_ymm_m256
vaddps_ymm_ymm_m256
vcvtdq2ps_ymm_m256
vcvtps2dq_ymm_m256
vcvtps2pd_ymm_m128
vcvttps2dq_ymm_m256
vdivpd_ymm_ymm_m256
vdivps_ymm_ymm_m256
vfmadd132ps_ymm_ymm_m256
vfnmadd132pd_ymm_ymm_m256
vmaxpd_ymm_ymm_m256
vmaxps_ymm_ymm_m256
vminpd_ymm_ymm_m256
vminps_ymm_ymm_m256
vmulpd_ymm_ymm_m256
vmulps_ymm_ymm_m256
vrcpps_ymm_m256
vrsqrtps_ymm_m256
vsqrtpd_ymm_m256
vsqrtps_ymm_m256
vsubpd_ymm_ymm_m256
vsubps_ymm_ymm_m256
xorq_m64_imm32
xorq_m64_imm8
xorq_m64_r64
xorq_r64_m64
*/

|| opcode == ADC_M8_IMM8
|| opcode == ADC_M32_IMM8
|| opcode == ADC_M8_R8
|| opcode == ADC_R8_M8
|| opcode == ADC_M32_IMM32
|| opcode == ADC_M32_R32
|| opcode == ADC_M64_R64
|| opcode == ADC_M64_IMM8
|| opcode == ADC_R32_M32
|| opcode == ADC_M64_IMM32
|| opcode == ADC_R64_M64
|| opcode == ADC_M16_IMM8
|| opcode == ADC_M16_IMM16
|| opcode == ADC_M16_R16
|| opcode == ADC_R16_M16
|| opcode == CMOVE_R64_M64
|| opcode == MOV_M8_RH
|| opcode == MOV_RH_M8
|| opcode == MOV_M64_IMM32
|| opcode == MOV_M64_R64
|| opcode == MOV_R64_M64
|| opcode == MOVSX_R64_M8
|| opcode == MOVSXD_R64_M32
|| opcode == MOVSX_R64_M16
|| opcode == OR_M64_IMM32
|| opcode == OR_M64_IMM8
|| opcode == OR_M64_R64
|| opcode == OR_R64_M64
|| opcode == POPCNT_R64_M64
|| opcode == SAL_M64_CL
|| opcode == SAL_M64_IMM8
|| opcode == SAR_M64_CL
|| opcode == SAR_M64_IMM8
|| opcode == SHR_M64_CL
|| opcode == SHR_M64_IMM8
|| opcode == VADDPD_YMM_YMM_M256
|| opcode == VADDPS_YMM_YMM_M256
|| opcode == VCVTDQ2PS_YMM_M256
|| opcode == VCVTPS2DQ_YMM_M256
|| opcode == VCVTPS2PD_YMM_M128
|| opcode == VCVTTPS2DQ_YMM_M256
|| opcode == VDIVPD_YMM_YMM_M256
|| opcode == VDIVPS_YMM_YMM_M256
|| opcode == VFMADD132PS_YMM_YMM_M256
|| opcode == VFNMADD132PD_YMM_YMM_M256
|| opcode == VMAXPD_YMM_YMM_M256
|| opcode == VMAXPS_YMM_YMM_M256
|| opcode == VMINPD_YMM_YMM_M256
|| opcode == VMINPS_YMM_YMM_M256
|| opcode == VMULPD_YMM_YMM_M256
|| opcode == VMULPS_YMM_YMM_M256
|| opcode == VRCPPS_YMM_M256
|| opcode == VRSQRTPS_YMM_M256
|| opcode == VSQRTPD_YMM_M256
|| opcode == VSQRTPS_YMM_M256
|| opcode == VSUBPD_YMM_YMM_M256
|| opcode == VSUBPS_YMM_YMM_M256
|| opcode == XOR_M64_IMM32
|| opcode == XOR_M64_IMM8
|| opcode == XOR_M64_R64
|| opcode == XOR_R64_M64
