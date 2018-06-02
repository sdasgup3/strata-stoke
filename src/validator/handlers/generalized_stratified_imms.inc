/*
   They are all generalized(same or extend to higher value) from base instructions which are in turn
are modelled using non-strata handlers.

Still, Strata handler owns them because of the logic:
if I is generalized using G {
 if(G a base instruction) return strata handler
 else return handler_of(G)
}

After the non strata handler model the semantics, the strata handler can't keep
the symbolic Imp that we need to port the rules to K
Hence moving them to non-strata handler; which faithful handle the
extension of imm using sign-ext
*/
|| opcode == ADC_AL_IMM8
|| opcode == ADC_AX_IMM16
|| opcode == ADC_EAX_IMM32
|| opcode == ADC_R16_IMM16
|| opcode == ADC_R16_IMM8
|| opcode == ADC_R32_IMM32
|| opcode == ADC_R32_IMM8
|| opcode == ADC_R64_IMM32
|| opcode == ADC_R64_IMM8
|| opcode == ADC_RAX_IMM32
|| opcode == ADC_R8_IMM8
|| opcode == MOV_R64_IMM32
|| opcode == MOV_RH_IMM8
|| opcode == OR_R64_IMM32
|| opcode == OR_RAX_IMM32
|| opcode == OR_R64_IMM8
|| opcode == SAL_R64_IMM8
|| opcode == SAR_R64_IMM8
|| opcode == SHR_R64_IMM8
|| opcode == XOR_R64_IMM32
|| opcode == XOR_R64_IMM8
|| opcode == XOR_RAX_IMM32
