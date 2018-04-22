// Copyright 2013-2015 Stanford University
//
// Licensed under the Apache License, Version 2.0 (the License);
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an AS IS BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "src/validator/handlers/strata_handler.h"
#include "src/tunit/tunit.h"
#include "src/specgen/specgen.h"
#include "src/specgen/support.h"
#include "src/validator/handlers.h"
#include "src/symstate/transform_visitor.h"
#include "src/ext/cpputil/include/io/console.h"
#include "src/validator/error.h"

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>

using namespace std;
using namespace stoke;
using namespace x64asm;
using namespace cpputil;
using namespace boost;

namespace {

/** A class to rename variables in symbolic formulas. */
class SymVarRenamer : public SymTransformVisitor {
public:
  SymVarRenamer(const function<SymBitVectorAbstract*(SymBitVectorVar*)>& bv_rename, const function<SymBoolAbstract*(SymBoolVar*)>& b_rename) : bv_rename_(bv_rename), b_rename_(b_rename) {}

  SymBitVectorAbstract* visit(const SymBitVectorVar * const bv) {
    return bv_rename_((SymBitVectorVar*)bv);
  }

  SymBoolAbstract* visit(const SymBoolVar * const bv) {
    return b_rename_((SymBoolVar*)bv);
  }

  const function<SymBitVectorAbstract*(SymBitVectorVar*)> bv_rename_;
  const function<SymBoolAbstract*(SymBoolVar*)> b_rename_;
};

/** Take a gp register and convert it into a 64 bit register. */
R64 r_to_r64(const R& reg) {
  size_t idx = reg;
  if (reg.type() == Type::RH) {
    return Constants::r64s()[idx - 4];
  }
  return Constants::r64s()[idx];
}
/** Take an sse register and convert it into a 256 bit register. */
Ymm sse_to_ymm(const Sse& reg) {
  size_t idx = reg;
  return Constants::ymms()[idx];
}

bool is_register_only(Opcode opcode) {
  Instruction instr(opcode);
  auto imm8 = specgen_is_imm8(opcode);
  for (size_t j = 0; j < instr.arity(); j++) {
    switch (instr.type(j)) {
    case x64asm::Type::RH:
    case x64asm::Type::AL:
    case x64asm::Type::CL:
    case x64asm::Type::R_8:
    case x64asm::Type::AX:
    case x64asm::Type::DX:
    case x64asm::Type::R_16:
    case x64asm::Type::EAX:
    case x64asm::Type::R_32:
    case x64asm::Type::RAX:
    case x64asm::Type::R_64:
    case x64asm::Type::XMM_0:
    case x64asm::Type::XMM:
    case x64asm::Type::YMM:

    // also allow some non-register but fixed operands
    case x64asm::Type::ZERO:
    case x64asm::Type::ONE:
    case x64asm::Type::THREE:
      break;
    case x64asm::Type::IMM_8:
      if (imm8) break;
      return false;
    default:
      return false;
    }
  }
  return true;
}

// bool is_register_type(const Type& t) {
//   switch (t) {
//   case x64asm::Type::RH:
//   case x64asm::Type::AL:
//   case x64asm::Type::CL:
//   case x64asm::Type::R_8:
//   case x64asm::Type::AX:
//   case x64asm::Type::DX:
//   case x64asm::Type::R_16:
//   case x64asm::Type::EAX:
//   case x64asm::Type::R_32:
//   case x64asm::Type::RAX:
//   case x64asm::Type::R_64:
//   case x64asm::Type::XMM_0:
//   case x64asm::Type::XMM:
//   case x64asm::Type::YMM:
//     break;
//   default:
//     return false;
//   }
//   return true;
// }

bool is_gp_type(const Type& t) {
  switch (t) {
  case x64asm::Type::RH:
  case x64asm::Type::AL:
  case x64asm::Type::CL:
  case x64asm::Type::R_8:
  case x64asm::Type::AX:
  case x64asm::Type::DX:
  case x64asm::Type::R_16:
  case x64asm::Type::EAX:
  case x64asm::Type::R_32:
  case x64asm::Type::RAX:
  case x64asm::Type::R_64:
    break;
  default:
    return false;
  }
  return true;
}

bool is_imm_type(const Type& t) {
  switch (t) {
  case x64asm::Type::IMM_8:
  case x64asm::Type::IMM_16:
  case x64asm::Type::IMM_32:
  case x64asm::Type::IMM_64:
    return true;
  default:
    break;
  }
  return false;
}

bool is_sse_type(const Type& t) {
  switch (t) {
  case x64asm::Type::XMM_0:
  case x64asm::Type::XMM:
  case x64asm::Type::YMM:
    return true;
  default:
    break;
  }
  return false;
}

bool is_sse_mem_type(const Type& t) {
  switch (t) {
  case x64asm::Type::M_32:
  case x64asm::Type::M_64:
    return true;
  default:
    break;
  }
  return false;
}

bool both_or_none_rh(const Type& t0, const Type& t1) {
  if (t0 == Type::RH || t1 == Type::RH) {
    return t0 == t1;
  }
  return true;
}

/**
 * Given two instructions with the same opcode, and a register from the context
 * of one of these instructions, translate it into a register in the context
 * of instr_to (translates operands, but leaves other registers).
 */
const R translate_gp_register(const R& operand_from, const Instruction& instr_from, const Instruction& instr_to) {
  for (size_t i = 0; i < instr_from.arity(); i++) {
    // direct match?
    if (operand_from == instr_from.get_operand<Operand>(i)) {
      return instr_to.get_operand<R>(i);
    }
    // same full register?
    if (instr_from.get_operand<Operand>(i).is_gp_register()) {
      if (r_to_r64(operand_from) == r_to_r64(instr_from.get_operand<R>(i))) {
        if (instr_to.get_operand<Operand>(i).is_gp_register()) {
          return r_to_r64(instr_to.get_operand<R>(i));
        } else {
          return instr_to.get_operand<R>(i);
        }
      }
    }
  }
  // no translation necessary
  return operand_from;
};
const Sse translate_sse_register(const Sse& operand_from, const Instruction& instr_from, const Instruction& instr_to) {
  for (size_t i = 0; i < instr_from.arity(); i++) {
    // direct match?
    if (operand_from == instr_from.get_operand<Operand>(i)) {
      return instr_to.get_operand<Sse>(i);
    }
    // same full register?
    if (instr_from.get_operand<Operand>(i).is_sse_register()) {
      if (sse_to_ymm(operand_from) == sse_to_ymm(instr_from.get_operand<Sse>(i))) {
        if (instr_to.get_operand<Operand>(i).is_sse_register()) {
          return sse_to_ymm(instr_to.get_operand<Sse>(i));
        } else {
          return instr_to.get_operand<Sse>(i);
        }
      }
    }
  }
  // no translation necessary
  return operand_from;
};

// this is a HACK: we use the transform visitor to create sym nodes.
auto transformer = SymTransformVisitor();

/**
 * Like translate_register, but the operand_from is a ymm/r64 register that may
 * correspond to one of the operands.
 */
SymBitVectorAbstract* translate_max_register(const SymState& state, const Operand& operand_from, const Instruction& instr_from, const Instruction& instr_to) {
  for (size_t i = 0; i < instr_from.arity(); i++) {
    // same 64 bit register?
    if (operand_from.type() == Type::R_64 || operand_from.type() == Type::RAX) {
      if (is_gp_type(instr_from.type(i)) && operand_from == r_to_r64(instr_from.get_operand<R>(i))) {
        if (is_imm_type(instr_to.type(i))) {
          auto val = (uint64_t)instr_to.get_operand<Imm>(i);
          auto c = transformer.make_bitvector_constant(bit_width_of_type(instr_to.type(i)), val);
          return transformer.make_bitvector_sign_extend(c, 64);
        } else if (is_gp_type(instr_to.type(i))) {
          auto translated_reg = r_to_r64(instr_to.get_operand<R>(i));
          return (SymBitVectorAbstract*)state.lookup(translated_reg).ptr;
        } else {
          auto operand_to = instr_to.get_operand<Operand>(i);
          auto res = (SymBitVectorAbstract*)state.lookup(operand_to).ptr;
          // std::cout << "translate_max_register: " << operand_to << "\n\t" << res << "\n";
          assert(operand_to.size() <= operand_from.size());
          if (operand_to.size() < operand_from.size()) {
            return transformer.make_bitvector_sign_extend(res, operand_from.size());
          }
          return res;
        }
      }

      // same 256 bit register?
    } else if (operand_from.type() == Type::YMM) {
      if (is_sse_type(instr_from.type(i)) && operand_from == sse_to_ymm(instr_from.get_operand<Sse>(i))) {
        if (is_sse_type(instr_to.type(i))) {
          auto translated_reg = sse_to_ymm(instr_to.get_operand<Sse>(i));
          return (SymBitVectorAbstract*)state.lookup(translated_reg).ptr;
        } else {
          auto operand_to = instr_to.get_operand<Operand>(i);
          auto res = (SymBitVectorAbstract*)state.lookup(operand_to).ptr;
          assert(operand_to.size() <= operand_from.size());
          if (operand_to.size() < operand_from.size()) {
            return transformer.make_bitvector_sign_extend(res, operand_from.size());
          }
          return res;
        }
      }
    }
  }
  // no translation necessary
  return (SymBitVectorAbstract*)state.lookup(operand_from).ptr;
};


// #define DEBUG_STRATA_HANDLER
#ifdef DEBUG_STRATA_HANDLER

void print_state(SymState& state, RegSet rs) {
  SymPrettyVisitor pretty(cout);

  auto print = [&pretty](const auto c) {
    pretty(c);
  };

  // print symbolic state
  bool printed = false;
  rs = rs & ((RegSet::all_gps() | RegSet::all_ymms()) +
             Constants::eflags_cf() +
             Constants::eflags_sf() +
             Constants::eflags_zf() +
             Constants::eflags_of() +
             Constants::eflags_pf() +
             Constants::eflags_af());
  for (auto gp_it = rs.gp_begin(); gp_it != rs.gp_end(); ++gp_it) {
    auto widened = r_to_r64(*gp_it);
    auto val = state.lookup(widened);
    Console::msg() << widened << "/" << (*gp_it) << ": ";
    print(val);
    Console::msg() << endl;
    printed = true;
  }
  if (printed) cout << endl;
  printed = false;
  for (auto sse_it = rs.sse_begin(); sse_it != rs.sse_end(); ++sse_it) {
    auto val = state.lookup(*sse_it);
    Console::msg() << (*sse_it) << ": ";
    print(val);
    Console::msg() << endl;
    printed = true;
  }
  if (printed) cout << endl;
  printed = false;
  for (auto flag_it = rs.flags_begin(); flag_it != rs.flags_end(); ++flag_it) {
    SymBool val = state[*flag_it];
    Console::msg() << (*flag_it) << ": ";
    print(val);
    Console::msg() << endl;
    printed = true;
  }
  if (printed) cout << endl;
  printed = false;
  // Console::msg() << "sigfpe  : ";
  // print(state.sigfpe);
  // Console::msg() << endl;
  // Console::msg() << "sigbus  : ";
  // print(state.sigbus);
  // Console::msg() << endl;
  // Console::msg() << "sigsegv : ";
  // print(state.sigsegv);
  // Console::msg() << endl;
}
#endif

} // end namespace

void StrataHandler::init() {

  reg_only_alternative_.clear();
  reg_only_alternative_mem_reduce_.clear();
  reg_only_alternative_extend_.clear();
  reg_only_alternative_duplicate_.clear();

  // map from mnenomic to all register-only instructions
  map<string, vector<Opcode>> str_to_opcode;
  for (auto i = 0; i < X64ASM_NUM_OPCODES; ++i) {
    auto opcode = (Opcode)i;
    string text = opcode_write_att(opcode);

    if (is_register_only(opcode)) {
      auto& vector = str_to_opcode[text];
      vector.push_back(opcode);
    }
  }

  // first map duplicates to their _1 version
  for (auto i = 0; i < X64ASM_NUM_OPCODES; ++i) {
    auto opcode = (Opcode)i;
    if (specgen_is_duplicate(opcode)) {
      string text = opcode_write_att(opcode);
      auto& options = str_to_opcode[text];
      Instruction instr(opcode);
      for (auto& option : options) {
        Instruction alt(option);
        if (alt.arity() != instr.arity()) continue;
        bool all_same = true;
        for (size_t j = 0; j < instr.arity(); j++) {
          if (instr.type(j) != alt.type(j)) {
            all_same = false;
            break;
          }
        }

        if (all_same) {
          reg_only_alternative_duplicate_[opcode] = option;
          // cout << opcode << " -> " << option << endl;
          break;
        }
      }
    }
  }

  // now determine for every instruction the corresponding reg-only opcode
  for (auto i = 0; i < X64ASM_NUM_OPCODES; ++i) {
    auto opcode = (Opcode)i;

    if (is_register_only(opcode)) continue;
    if (specgen_is_mm(opcode)) continue;
    if (specgen_is_base(opcode)) continue;

    string text = opcode_write_att(opcode);
    auto& options = str_to_opcode[text];
    Instruction instr(opcode);

    // check if there is an opcode with the same width operands
    bool found = false;
    for (auto& option : options) {
      Instruction alt(option);
      if (alt.arity() != instr.arity()) continue;
      bool same_widths = true;
      for (size_t j = 0; j < instr.arity(); j++) {
        auto notsame = bit_width_of_type(instr.type(j)) != bit_width_of_type(alt.type(j));
        auto rhok = both_or_none_rh(instr.type(j), alt.type(j));
        if (notsame || !rhok) {
          same_widths = false;
          break;
        }
      }

      if (same_widths) {
        found = true;
        // cout << opcode << " -> " << option << endl;
        reg_only_alternative_[opcode] = option;
        break;
      }
    }

    if (!found) {
      // check for an imm instruction that has one with larger width
      for (auto& option : options) {
        Instruction alt(option);
        if (alt.arity() != instr.arity()) continue;
        bool larger_widths = true;
        for (size_t j = 0; j < instr.arity(); j++) {
          bool larger = bit_width_of_type(instr.type(j)) <= bit_width_of_type(alt.type(j));
          bool same = bit_width_of_type(instr.type(j)) == bit_width_of_type(alt.type(j));
          bool imm_type = is_imm_type(instr.type(j));
          auto rhok = both_or_none_rh(instr.type(j), alt.type(j));
          if (!(same || (larger && imm_type)) || !rhok) {
            larger_widths = false;
            break;
          }
        }

        if (larger_widths) {
          found = true;
          // cout << opcode << " -> " << option << endl;
          reg_only_alternative_extend_[opcode] = option;
          break;
        }
      }
    }

    if (!found) {
      // check for an float memory instruction
      for (auto& option : options) {
        Instruction alt(option);
        if (alt.arity() != instr.arity()) continue;
        bool larger_widths = true;
        for (size_t j = 0; j < instr.arity(); j++) {
          bool same = bit_width_of_type(instr.type(j)) == bit_width_of_type(alt.type(j));
          bool ymm_type = is_sse_type(alt.type(j));
          bool float_mem_type = is_sse_mem_type(instr.type(j));
          auto rhok = both_or_none_rh(instr.type(j), alt.type(j));
          if (!(same || (ymm_type && float_mem_type)) || !rhok) {
            larger_widths = false;
            break;
          }
        }

        if (larger_widths) {
          found = true;
          // cout << opcode << " -> " << option << endl;
          reg_only_alternative_mem_reduce_[opcode] = option;
          break;
        }
      }
    }

    if (!found) {
      if (!specgen_is_system(opcode) &&
          !specgen_is_float(opcode) &&
          !specgen_is_jump(opcode) &&
          !specgen_is_crypto(opcode) &&
          !specgen_is_sandbox_unsupported(opcode)) {
        // cout << opcode << endl;
      }
    }
  }

  // for (auto i = 0; i < X64ASM_NUM_OPCODES; ++i) {
  //   auto opcode = (Opcode)i;
  //   if (reg_only_alternative_.find(opcode) != reg_only_alternative_.end()) {
  //     //cout << opcode << " -> " << reg_only_alternative_[opcode] << endl;
  //     cout << "-> " << opcode << endl;
  //     cout << "   " << reg_only_alternative_[opcode] << endl;
  //   }
  // }
}

bool StrataHandler::is_supported(const x64asm::Opcode& opcode) {
  // Generalization from statified instr is wrong for these instructions
  if (opcode == MOVSD_XMM_M64
      || opcode == MOVSS_XMM_M32
      || opcode == XCHG_M8_R8
      || opcode == XCHG_M8_RH
      || opcode == XCHG_M16_R16
      || opcode == XCHG_M32_R32
      || opcode == XCHG_M64_R64
      || opcode == XCHG_R8_M8
      || opcode == XCHG_RH_M8
      || opcode == XCHG_R16_M16
      || opcode == XCHG_R32_M32
      || opcode == XCHG_R64_M64
      || opcode == XADD_M32_R32 // The generalization from reg -> mem is buggy
      || opcode == VCVTPD2DQ_XMM_M256 // The generalization of vcvtpd2dq_xmm_m256 from reg -> mem is buggy is master/strata branch
      || opcode == BT_M16_IMM8 // https://github.com/sdasgup3/binary-decompilation/wiki/Bugs-Found#self-goal
      || opcode == BT_M16_R16
      || opcode == BT_M32_IMM8
      || opcode == BT_M32_R32
      || opcode == BT_M64_IMM8
      || opcode == BT_M64_R64
      || opcode == CMP_M8_IMM8 || opcode == CMP_M8_RH || opcode == CMP_M8_R8 || opcode == CMP_RH_M8 || opcode == CMP_R8_M8 || opcode == CMP_M32_IMM32 || opcode == CMP_M32_IMM8 || opcode == CMP_M32_R32 || opcode == CMP_M64_IMM32 || opcode == CMP_R32_M32 || opcode == CMP_M64_IMM8 || opcode == CMP_M64_R64 || opcode == CMP_R64_M64 || opcode == CMP_M16_IMM16 || opcode == CMP_M16_IMM8 || opcode == CMP_M16_R16 || opcode == CMP_R16_M16 || opcode == DEC_M8 || opcode == DEC_M32 || opcode == DEC_M64 || opcode == DEC_M16 || opcode == INC_M16 || opcode == NEG_M8 || opcode == NEG_M32 || opcode == NEG_M16 || opcode == NEG_M64 || opcode == SBB_M8_IMM8 || opcode == SBB_M8_RH || opcode == SBB_M8_R8 || opcode == SBB_M32_IMM32 || opcode == SBB_M32_IMM8 || opcode == SBB_RH_M8 || opcode == SBB_R8_M8 || opcode == SBB_M64_IMM8 || opcode == SBB_M64_IMM32 || opcode == SBB_M32_R32 || opcode == SBB_M16_IMM16 || opcode == SBB_M16_IMM8 || opcode == SBB_R32_M32 || opcode == SBB_M16_R16 || opcode == SBB_R16_M16 || opcode == SBB_M64_R64 || opcode == SBB_R64_M64 || opcode == SUB_M8_IMM8 || opcode == SUB_M8_R8 || opcode == SUB_M8_RH || opcode == SUB_R8_M8 || opcode == SUB_RH_M8 || opcode == SUB_M32_IMM32 || opcode == SUB_M32_IMM8 || opcode == SUB_M32_R32 || opcode == SUB_R32_M32 || opcode == SUB_M64_IMM8 || opcode == SUB_M64_IMM32 || opcode == SUB_M64_R64 || opcode == SUB_R64_M64 || opcode == SUB_M16_IMM16 || opcode == SUB_M16_IMM8 || opcode == SUB_M16_R16 || opcode == SUB_R16_M16 // The strata formula for af is not equivalent to non-strata handler's. Also the other formulas for register/flags could be much simple is we use the non-strata handler instead.
      || opcode == PMOVSXBD_XMM_M32 //  <-- START: The strata formula contains uifs where as the non strata ones do not have those.
      || opcode == PMOVSXWD_XMM_M64 // Equiv checks fails. We prefer the not strata ones are they lacks uifs and simple.
      || opcode == PMOVSXWQ_XMM_M32
      || opcode == VPMOVSXBD_XMM_M32
      || opcode == VPMOVSXBD_YMM_M64
      || opcode == VPMOVSXBQ_YMM_M32
      || opcode == VPMOVSXWD_XMM_M64
      || opcode == VPMOVSXWD_YMM_M128
      || opcode == VPMOVSXWQ_XMM_M32
      || opcode == VPMOVSXWQ_YMM_M64
      || opcode == PMOVZXBD_XMM_M32
      || opcode == PMOVZXWD_XMM_M64
      || opcode == PUNPCKHWD_XMM_M128
      || opcode == PUNPCKLWD_XMM_M128
      || opcode == VFMSUBADD132PS_XMM_XMM_M128
      || opcode == VFMSUBADD213PS_XMM_XMM_M128
      || opcode == VFMSUBADD231PS_XMM_XMM_M128
      || opcode == VPMOVZXBD_XMM_M32
      || opcode == VPMOVZXBD_YMM_M64
      || opcode == VPMOVZXWD_XMM_M64
      || opcode == VPMOVZXWD_YMM_M128
      || opcode == VPMOVZXWQ_YMM_M64
      || opcode == VPUNPCKHWD_XMM_XMM_M128
      || opcode == VPUNPCKHWD_YMM_YMM_M256
      || opcode == VPUNPCKLWD_XMM_XMM_M128
      || opcode == VPUNPCKLWD_YMM_YMM_M256
      || opcode == VFMADD132SD_XMM_XMM_M64
      || opcode == VFNMADD132SS_XMM_XMM_M32
      || opcode == VFMSUBADD132PS_XMM_XMM_M128
      || opcode == VFMADDSUB213PS_XMM_XMM_M128 // <-- END
      || opcode == VFMADDSUB132PD_XMM_XMM_M128 // <- Both stata and master branch yeilds formula for these using strata handler, but I write simple formula
      || opcode == VFMADDSUB213PD_XMM_XMM_M128 // for those for regosters, which can now be generalized to registers; Get rid of complex formula
      || opcode == VFMADDSUB132PS_XMM_XMM_M128
      || opcode == VFMSUBADD213PD_XMM_XMM_M128
      || opcode == VFMADDSUB231PD_XMM_XMM_M128
      || opcode == VFMSUBADD132PD_XMM_XMM_M128
      || opcode == VFMSUBADD231PD_XMM_XMM_M128
      || opcode == VFMADDSUB231PS_XMM_XMM_M128 // <-- END: This particula one has a huge formula otherwise; check master strata to reproduce
      || opcode == CMOVO_R64_M64
      || opcode == HSUBPS_XMM_M128
      || opcode == CMOVL_R64_M64
      || opcode == VPADDD_XMM_XMM_M128
      || opcode == CMOVNP_R64_M64
      || opcode == VPHADDD_YMM_YMM_M256
      || opcode == AND_R16_M16
      || opcode == CMOVNS_R64_M64
      || opcode == AND_M16_IMM16
      || opcode == BLSI_R32_M32
      || opcode == VPUNPCKLDQ_YMM_YMM_M256
      || opcode == VPMADDWD_YMM_YMM_M256
      || opcode == VHSUBPS_XMM_XMM_M128
      || opcode == PSUBQ_XMM_M128
      || opcode == VPSUBD_YMM_YMM_M256
      || opcode == TZCNT_R16_M16
      || opcode == ROR_M16_ONE
      || opcode == CMOVAE_R64_M64
      || opcode == VPHSUBD_YMM_YMM_M256
      || opcode == VPMADDWD_XMM_XMM_M128
      || opcode == VHADDPS_YMM_YMM_M256
      || opcode == VUNPCKLPS_YMM_YMM_M256
      || opcode == PADDD_XMM_M128
      || opcode == VPSUBQ_XMM_XMM_M128
      || opcode == VHADDPS_XMM_XMM_M128
      || opcode == VXORPS_YMM_YMM_M256
      || opcode == AND_M16_R16
      || opcode == PSUBD_XMM_M128
      || opcode == AND_M16_IMM8
      || opcode == CMOVNC_R64_M64
      || opcode == VUNPCKLPS_XMM_XMM_M128
      || opcode == VHSUBPS_YMM_YMM_M256
      || opcode == CMOVNGE_R64_M64
      || opcode == VPUNPCKLDQ_XMM_XMM_M128
      || opcode == VPADDD_YMM_YMM_M256
      || opcode == VPSUBD_XMM_XMM_M128
      || opcode == CMOVNO_R64_M64
      || opcode == CMOVNB_R64_M64
      || opcode == VPXOR_YMM_YMM_M256
      || opcode == HADDPS_XMM_M128
      || opcode == CMOVPO_R64_M64
      || opcode == VXORPD_YMM_YMM_M256
      || opcode == PHSUBD_XMM_M128
      || opcode == CMOVNL_R64_M64
      || opcode == CMOVS_R64_M64
      || opcode == CMOVGE_R64_M64
      || opcode == VPMOVSXDQ_YMM_M128
      || opcode == VPHSUBD_XMM_XMM_M128    
     ) {
    return false;
  }

  auto rs = support_reason(opcode);
  // cout << "Opcode: " << opcode << " Reason: " << rs << "\n";

  return rs != SupportReason::NONE;
}

SupportReason StrataHandler::support_reason(const x64asm::Opcode& opcode) {
  stringstream ss;
  ss << opcode;
  auto opcode_str = ss.str();
  auto candidate_file = strata_path_ + "/" + opcode_str + ".s";

  // can we convert this into a register only instruction?
  bool found = false;
  auto reason = SupportReason::NONE;
  Opcode alt = XOR_R8_R8;
  if (reg_only_alternative_duplicate_.find(opcode) != reg_only_alternative_duplicate_.end()) {
    alt = reg_only_alternative_duplicate_[opcode];
    found = true;
    reason = SupportReason::GENERALIZE_SAME;
  } else if (reg_only_alternative_.find(opcode) != reg_only_alternative_.end()) {
    alt = reg_only_alternative_[opcode];
    found = true;
    reason = SupportReason::GENERALIZE_SAME;
  } else if (reg_only_alternative_mem_reduce_.find(opcode) != reg_only_alternative_mem_reduce_.end()) {
    alt = reg_only_alternative_mem_reduce_[opcode];
    found = true;
    reason = SupportReason::GENERALIZE_SHRINK;
  } else if (reg_only_alternative_extend_.find(opcode) != reg_only_alternative_extend_.end()) {
    alt = reg_only_alternative_extend_[opcode];
    found = true;
    reason = SupportReason::GENERALIZE_EXTEND;
  }

  if (found) {
    if (specgen_is_base(alt)) {
      // std::cout << "Base Instruction!!\n";
      return reason;
    }
    if (is_supported(alt)) return reason;
  } else {
    // we have a learned circuit
    if (filesystem::exists(candidate_file)) {
      return SupportReason::LEARNED;
    }
  }

  return SupportReason::NONE;
}

int StrataHandler::used_for(const x64asm::Opcode& op) {
  int res = 0;

  for (auto i = 0; i < X64ASM_NUM_OPCODES; ++i) {
    auto opcode = (Opcode)i;
    bool found = false;
    Opcode alt = XOR_R8_R8;
    if (reg_only_alternative_duplicate_.find(opcode) != reg_only_alternative_duplicate_.end()) {
      alt = reg_only_alternative_duplicate_[opcode];
      found = true;
    } else if (reg_only_alternative_.find(opcode) != reg_only_alternative_.end()) {
      alt = reg_only_alternative_[opcode];
      found = true;
    } else if (reg_only_alternative_mem_reduce_.find(opcode) != reg_only_alternative_mem_reduce_.end()) {
      alt = reg_only_alternative_mem_reduce_[opcode];
      found = true;
    } else if (reg_only_alternative_extend_.find(opcode) != reg_only_alternative_extend_.end()) {
      alt = reg_only_alternative_extend_[opcode];
      found = true;
    }

    if (found && alt == op) {
      res += 1;
    }
  }

  return res;
}

int specgen_get_imm8(const Instruction& instr) {
  return (int)instr.get_operand<Imm8>(instr.arity() - 1);
}

Handler::SupportLevel StrataHandler::get_support(const x64asm::Instruction& instr) {
  auto yes = (Handler::SupportLevel)(Handler::BASIC | Handler::CEG | Handler::ANALYSIS);
  if (!operands_supported(instr)) {
    return Handler::NONE;
  }
  auto opcode = instr.get_opcode();
  if (is_supported(opcode)) {
    return yes;
  }

  // check for imm8 support
  if (specgen_is_imm8(opcode)) {
    stringstream ss;
    ss << opcode << "_" << specgen_get_imm8(instr);
    auto candidate_file = strata_path_ + "/" + ss.str() + ".s";
    // we have a learned circuit
    if (filesystem::exists(candidate_file)) {
      return yes;
    }
  }

  return Handler::NONE;
}

void StrataHandler::build_circuit(const x64asm::Instruction& instr, SymState& final) {
  auto& should_simplify = simplify_;
  auto& simplifier = simplifier_;
  auto& tc = tc_;
  auto& ch = ch_;
  auto simplify = [&simplifier, &should_simplify](SymBitVectorAbstract* circuit) {
    if (should_simplify) {
      return simplifier.simplify(circuit);
    }
    return SymBitVector(circuit);
  };
  auto simplifybool = [&simplifier, &should_simplify](SymBoolAbstract* circuit) {
    if (should_simplify) {
      return simplifier.simplify(circuit);
    }
    return SymBool(circuit);
  };

  auto opcode = instr.get_opcode();
  stringstream ss;
  ss << opcode;
  auto opcode_str = ss.str();
  auto candidate_file = strata_path_ + "/" + opcode_str + ".s";
  if (specgen_is_imm8(opcode)) {
    stringstream ss;
    ss << opcode << "_" << dec << specgen_get_imm8(instr);
    candidate_file = strata_path_ + "/" + ss.str() + ".s";
  }

  error_ = "";

  if (specgen_is_base(opcode) || opcode == Opcode::CALL_LABEL) {
    ch.build_circuit(instr, final);
    if (ch.has_error()) {
      error_ = "ComboHandler encountered an error: " + ch.error();
      return;
    }
#ifdef DEBUG_STRATA_HANDLER
    cout << "-------------------------------------" << endl;
    cout << "Getting base circuit for " << instr << endl << endl;
    cout << "Final state:" << endl;
    print_state(final, instr.maybe_write_set());
    cout << "-------------------------------------" << endl;
#endif
    return;
  }

  // handle duplicate instructions
  if (reg_only_alternative_duplicate_.find(opcode) != reg_only_alternative_duplicate_.end()) {
    // get circuit for register only opcode
    Instruction alt = instr;
    alt.set_opcode(reg_only_alternative_duplicate_[opcode]);
    build_circuit(alt, final);
    return;
  }

  auto typecheck = [&tc, this](auto circuit, size_t exptected_size) {
#ifdef DEBUG_STRATA_HANDLER
    auto actual = tc(circuit);
    if (tc.has_error()) {
      error_ = "Encountered error during type-checking of: " + tc.error();
      return false;
    }
    if (actual != exptected_size) {
      assert(false);
      stringstream ss;
      ss << "Expected " << exptected_size << " bits, but got " << actual << " instead for ";
      SymPrettyVisitor pretty(ss);
      pretty(circuit);
      ss << ".";
      error_ = ss.str();
      return false;
    }
#endif
    return true;
  };

  // keep a copy of the start state
  SymState start = final;
  start.set_delete_memory(false);

  // the state which will be the circuit for our alternative instruction
  SymState tmp(opcode_str);

  Instruction specgen_instr(XOR_R8_R8);
  if (reg_only_alternative_.find(opcode) != reg_only_alternative_.end()) {
    // handle instructions with a direct register only alternative
    // get circuit for register only opcode
    specgen_instr = get_instruction(reg_only_alternative_[opcode]);
    build_circuit(specgen_instr, tmp);
    if (ch.has_error()) {
      std::cout << specgen_instr << "\n";
      error_ = "StrataHandler (Direct) encountered an error: " + ch.error();
      return;
    }
  } else if (reg_only_alternative_extend_.find(opcode) != reg_only_alternative_extend_.end()) {
    // handle instructions that need extending
    // this is actually the same as above
    specgen_instr = get_instruction(reg_only_alternative_extend_[opcode]);
    build_circuit(specgen_instr, tmp);
    if (ch.has_error()) {
      std::cout << specgen_instr << "\n";
      error_ = "StrataHandler (Extend) encountered an error: " + ch.error();
      return;
    }
  } else if (reg_only_alternative_mem_reduce_.find(opcode) != reg_only_alternative_mem_reduce_.end()) {
    // handle instructions that need extending
    // this is actually the same as above
    specgen_instr = get_instruction(reg_only_alternative_mem_reduce_[opcode]);
    build_circuit(specgen_instr, tmp);
    if (ch.has_error()) {
      std::cout << specgen_instr << "\n";
      error_ = "StrataHandler (Shrink) encountered an error: " + ch.error();
      return;
    }
  } else {
    // we are dealing with a circuit that we have learned
    specgen_instr = get_instruction(opcode);

    // read cache
    auto it = formula_cache_.find(opcode);
    if (it != formula_cache_.end()) {
      tmp = SymState(it->second);
    } else {
      // read program
      ifstream file(candidate_file);
      TUnit t;
      file >> t;

      if (failed(file)) {
        cerr << "INTERNAL STOKE ERROR, please report" << endl;
        cerr << "Failed to parse " << candidate_file << endl;
        cerr << "Message: " << fail_msg(file) << endl;
        exit(1);
      }

      // build formula for program
      auto code = t.get_code();
      assert(code[0].get_opcode() == Opcode::LABEL_DEFN);
      assert(code[code.size() - 1].get_opcode() == Opcode::RET);
      for (size_t i = 1; i < code.size()-1; i++) {
        build_circuit(code[i], tmp);
      }

      // cache for future
      //formula_cache_[opcode] = SymState(tmp);
    }
  }

// #ifdef DEBUG_STRATA_HANDLER
//   cout << "=====================================" << endl;
//   cout << "Computing circuit for " << instr << endl << endl;
//   cout << t.get_code() << endl << endl;
//   cout << "Initial state:" << endl;
//   print_state(start, instr.maybe_write_set());
//   cout << "State for specgen instruction: " << specgen_instr << ":" << endl;
//   print_state(tmp, specgen_instr.maybe_write_set());
// #endif

  // take a formula for specgen_instr in state tmp, and convert it to one that
  // makes sense for instr in state
  SymVarRenamer translate_circuit(
  [&instr, &specgen_instr, &start, &opcode_str](SymBitVectorVar* var) -> SymBitVectorAbstract* {
    auto name = var->name_;
    if (name.size() <= opcode_str.size() || name.substr(name.size() - opcode_str.size()) != opcode_str) {
      // no renaming for variable of unfamiliar names
      return var;
    }
    auto real_name = name.substr(0, name.size() - opcode_str.size() - 1);
    R64 gp = Constants::rax();
    Ymm ymm = Constants::ymm0();
    if (stringstream(real_name) >> gp) {
      return translate_max_register(start, gp, specgen_instr, instr);
    } else if (stringstream(real_name) >> ymm) {
      return translate_max_register(start, ymm, specgen_instr, instr);
    }
    assert(false);
    return NULL;
  },
  [&start, &opcode_str](SymBoolVar* var) -> SymBoolAbstract* {
    auto name = var->name_;
    if (name.size() <= opcode_str.size() || name.substr(name.size() - opcode_str.size()) != opcode_str) {
      // no renaming for variable of unfamiliar names
      return var;
    }
    auto real_name = name.substr(0, name.size() - opcode_str.size() - 1);
    Eflags reg = Constants::eflags_cf();
    if (stringstream(real_name) >> reg) {
      return (SymBoolAbstract*)start[reg].ptr;
    }
    assert(false);
    return NULL;
  }
  );

  auto extend_or_shrink = [](auto& in, uint64_t size) {
    if (in.width() > size) {
      in = in[size-1][0];
    }
    if (in.width() < size) {
      in = in.sign_extend(size);
    }
    return in;
  };

  // loop over all live outs and update the final state
  auto liveouts = specgen_instr.maybe_write_set();
  if (opcode_str.size() > 4 && opcode_str.substr(0, 4) == "xadd") {
    // for xadd, we need to hard-code the order of registers
    auto op0 = specgen_instr.get_operand<R>(0);
    auto op1 = specgen_instr.get_operand<R>(1);
    if (opcode == Opcode::XADD_R32_R32) {
      // 64 bit extension
      op0 = Constants::r64s()[(size_t)op0];
      op1 = Constants::r64s()[(size_t)op1];
    }
    for (auto iter : {
           op1, op0
         }) {
      auto iter_translated = translate_gp_register(iter, specgen_instr, instr);
      // look up live out in tmp state
      auto val = tmp[iter];
      if (!typecheck(val, (iter).size())) return;
      // rename variables in the tmp state to the values in start
      auto val_renamed = simplify(translate_circuit(val));
      val_renamed = extend_or_shrink(val_renamed, iter_translated.size());
      if (!typecheck(val_renamed, (iter).size())) return;
      // update the start state with the circuits from tmp
      final.set(iter_translated, val_renamed, false, true);
    }
  } else {
    for (auto iter = liveouts.gp_begin(); iter != liveouts.gp_end(); ++iter) {
      auto iter_translated = translate_gp_register(*iter, specgen_instr, instr);
      // look up live out in tmp state
      auto val = tmp[*iter];
#ifdef DEBUG_STRATA_HANDLER
      cout << "Register        -> " << (*iter) << endl;
      cout << "  translates to => " << iter_translated << endl;
#endif
      if (!typecheck(val, (*iter).size())) return;
      // rename variables in the tmp state to the values in start
      auto val_renamed = simplify(translate_circuit(val));
      val_renamed = extend_or_shrink(val_renamed, iter_translated.size());
#ifdef DEBUG_STRATA_HANDLER
      cout << "Value is               -> " << simplify(val) << endl;
      cout << "  after renaming it is => " << simplify(val_renamed) << endl;
      cout << endl;
#endif
      if (!typecheck(val_renamed, iter_translated.size())) return;
      // update the start state with the circuits from tmp
      final.set(iter_translated, val_renamed, false, true);
    }
  }

  for (auto iter = liveouts.sse_begin(); iter != liveouts.sse_end(); ++iter) {
    auto iter_translated = translate_sse_register(*iter, specgen_instr, instr);
    // look up live out in tmp state (after translating operators as necessary)
    auto val = tmp[*iter];
    if (!typecheck(val, (*iter).size())) return;
    // rename variables in the tmp state to the values in start
    auto val_renamed = simplify(translate_circuit(val));
    val_renamed = extend_or_shrink(val_renamed, iter_translated.size());
    if (!typecheck(val_renamed, iter_translated.size())) return;
    // update the start state with the circuits from tmp
    final.set(iter_translated, val_renamed, false, true);
  }
  for (auto iter = liveouts.flags_begin(); iter != liveouts.flags_end(); ++iter) {
    auto iter_translated = *iter;

    // cout << "Flag        -> " << (*iter) << endl;
    // cout << "  translates to => " << iter_translated << endl;

    // look up live out in tmp state (no translation necessary for flags)
    auto val = tmp[*iter];
    if (!typecheck(val, 1)) return;
    // rename variables in the tmp state to the values in start
    auto val_renamed = simplifybool(translate_circuit(val));

    // cout << "Value is               -> " << val << endl;
    // cout << "  after renaming it is => " << val_renamed << endl;
    // cout << endl;

    if (!typecheck(val_renamed, 1)) return;
    // update the start state with the circuits from tmp
    final.set(iter_translated, val_renamed);
  }

  // set all undefined outputs to a new temporary variable
  auto undefs = instr.must_undef_set();
  for (auto iter = undefs.gp_begin(); iter != undefs.gp_end(); ++iter) {
    auto width = bit_width_of_type((*iter).type());
    final.set(*iter, SymBitVector::tmp_var(width), false, true);
  }
  for (auto iter = undefs.sse_begin(); iter != undefs.sse_end(); ++iter) {
    auto width = bit_width_of_type((*iter).type());
    final.set(*iter, SymBitVector::tmp_var(width), false, true);
  }
  for (auto iter = undefs.flags_begin(); iter != undefs.flags_end(); ++iter) {
    final.set(*iter, SymBool::tmp_var());
  }

#ifdef DEBUG_STRATA_HANDLER
  cout << "Final state" << endl;
  print_state(final, instr.maybe_write_set());
  cout << "=====================================" << endl;
#endif
}

vector<x64asm::Opcode> StrataHandler::full_support_opcodes() {
  vector<x64asm::Opcode> res;
  for (size_t i = 0; i < X64ASM_NUM_OPCODES; ++i) {
    auto opcode = (x64asm::Opcode)i;
    if (is_supported(opcode)) {
      res.push_back(opcode);
    }
  }
  return res;
}
