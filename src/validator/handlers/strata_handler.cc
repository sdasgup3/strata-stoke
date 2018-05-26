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
      || opcode == CMOVO_R64_M64 // <- Adding simple but equiv formulas from other handlers
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
      || opcode == VPHSUBD_XMM_XMM_M128 // <-- END
      || opcode == SHRX_R32_M32_R32 // <-- FOr the same purpose, but manually added these formula in other handler to keep the formula simpler, otherwise the strata formula is too complex
      || opcode == CMPXCHG_M16_R16
      || opcode == CMPXCHG_M64_R64
      || opcode == CMPXCHG_M8_RH
      || opcode == CMPXCHG_M8_R8
      || opcode == VFNMADD231SD_XMM_XMM_M64
      || opcode == VFMADD132PS_XMM_XMM_M128
      || opcode == VFNMADD213SS_XMM_XMM_M32
      || opcode == VFNMADD132PD_XMM_XMM_M128
      || opcode == VFNMSUB213SD_XMM_XMM_M64
      || opcode == VFMADD213SD_XMM_XMM_M64
      || opcode == VFNMADD213SD_XMM_XMM_M64
      || opcode == VFMSUB213SS_XMM_XMM_M32    // <-- END
      /* Next 692 Stratified formulas
        || opcode ==  ADD_R64_R64 || opcode ==  ANDN_R64_R64_R64 || opcode ==  CBW || opcode ==  CLC || opcode ==  CDQE || opcode ==  CMOVZ_R64_R64 || opcode ==  CMOVZ_R16_R16 || opcode ==  CVTSS2SD_XMM_XMM || opcode ==  DEC_R8 || opcode ==  INC_R32 || opcode ==  INC_R16 || opcode ==  MOVAPD_XMM_XMM || opcode ==  MOVAPS_XMM_XMM || opcode ==  MOVD_R32_XMM || opcode ==  MOVDDUP_XMM_XMM || opcode ==  MOVDQA_XMM_XMM || opcode ==  MOVDQU_XMM_XMM || opcode ==  MOVSX_R32_R8 || opcode ==  MOVUPS_XMM_XMM || opcode ==  MOV_R16_R16 || opcode ==  MOVZX_R32_RH || opcode ==  MOVZX_R16_R8 || opcode ==  MOVZX_R16_RH || opcode ==  MOVZX_R32_R16 || opcode ==  NOP || opcode ==  NOP_R32 || opcode ==  NOP_R16 || opcode ==  RCL_R32_ONE || opcode ==  RCL_R64_ONE || opcode ==  RCL_R16_ONE || opcode ==  SARX_R64_R64_R64 || opcode ==  SETAE_R8 || opcode ==  SETB_R8 || opcode ==  SETBE_R8 || opcode ==  SETC_RH || opcode ==  SETE_R8 || opcode ==  SETNAE_RH || opcode ==  SETNO_RH || opcode ==  SETNP_R8 || opcode ==  SETNP_RH || opcode ==  SETO_R8 || opcode ==  SETP_R8 || opcode ==  SETS_R8 || opcode ==  SETZ_R8 || opcode ==  SHL_R64_CL || opcode ==  SHLX_R64_R64_R64 || opcode ==  STC || opcode ==  VADDPD_XMM_XMM_XMM || opcode ==  VCVTDQ2PS_XMM_XMM || opcode ==  VCVTPD2PS_XMM_XMM || opcode ==  VCVTPS2DQ_XMM_XMM || opcode ==  VCVTPS2PD_XMM_XMM || opcode ==  VFMADD213PD_YMM_YMM_YMM || opcode ==  VFMADD213PS_YMM_YMM_YMM || opcode ==  VFMADD231PS_YMM_YMM_YMM || opcode ==  VFMSUB213PD_YMM_YMM_YMM || opcode ==  VFMSUB213PS_YMM_YMM_YMM || opcode ==  VFMSUB231PD_YMM_YMM_YMM || opcode ==  VFMSUB231PS_YMM_YMM_YMM || opcode ==  VFNMADD213PD_YMM_YMM_YMM || opcode ==  VFNMADD213PS_YMM_YMM_YMM || opcode ==  VFNMADD231PD_YMM_YMM_YMM || opcode ==  VFNMADD231PS_YMM_YMM_YMM || opcode ==  VFNMSUB213PD_YMM_YMM_YMM || opcode ==  VFNMSUB213PS_YMM_YMM_YMM || opcode ==  VFNMSUB231PD_YMM_YMM_YMM || opcode ==  VFNMSUB231PS_YMM_YMM_YMM || opcode ==  VMOVAPD_XMM_XMM || opcode ==  VMOVAPS_XMM_XMM || opcode ==  VMOVDQA_XMM_XMM || opcode ==  VMOVDQA_YMM_YMM || opcode ==  VMOVDQU_XMM_XMM || opcode ==  VMOVDQU_YMM_YMM || opcode ==  VMOVHLPS_XMM_XMM_XMM || opcode ==  VMOVQ_R64_XMM || opcode ==  VMOVQ_XMM_R64 || opcode ==  VMOVUPD_XMM_XMM || opcode ==  VMOVUPD_YMM_YMM || opcode ==  VMOVUPS_XMM_XMM || opcode ==  VMULPS_XMM_XMM_XMM || opcode ==  VORPD_XMM_XMM_XMM || opcode ==  VPBROADCASTQ_XMM_XMM || opcode ==  VPMOVSXDQ_XMM_XMM || opcode ==  VPMOVZXDQ_XMM_XMM || opcode ==  VPMOVZXWQ_XMM_XMM || opcode ==  VPUNPCKHQDQ_XMM_XMM_XMM || opcode ==  VPUNPCKLQDQ_XMM_XMM_XMM || opcode ==  VRCPPS_XMM_XMM || opcode ==  VRSQRTPS_XMM_XMM || opcode ==  VSQRTPS_XMM_XMM || opcode ==  VXORPS_XMM_XMM_XMM || opcode ==  XCHG_R32_R32 || opcode ==  XCHG_R64_R64 || opcode ==  XCHG_R16_R16 || opcode ==  XOR_R16_R16 || opcode ==  ADC_RH_R8 || opcode ==  ADDPD_XMM_XMM || opcode ==  ADD_R16_R16 || opcode ==  CMOVNZ_R16_R16 || opcode ==  CQO || opcode ==  CVTPS2DQ_XMM_XMM || opcode ==  CVTPS2PD_XMM_XMM || opcode ==  CVTSS2SI_R32_XMM || opcode ==  CVTTSS2SI_R32_XMM || opcode ==  DEC_R64 || opcode ==  DIVPD_XMM_XMM || opcode ==  DIVPS_XMM_XMM || opcode ==  INC_R8 || opcode ==  MOV_RH_RH || opcode ==  MOVLHPS_XMM_XMM || opcode ==  MOVQ_R64_XMM || opcode ==  MOVQ_XMM_XMM || opcode ==  MOVSX_R32_RH || opcode ==  MOVSX_R16_RH || opcode ==  MOVSD_XMM_XMM || opcode ==  MOVSS_XMM_XMM || opcode ==  MOVSX_R32_R16 || opcode ==  MOVUPD_XMM_XMM || opcode ==  MOVZX_R64_R8 || opcode ==  MULPD_XMM_XMM || opcode ==  MULPS_XMM_XMM || opcode ==  MULSD_XMM_XMM || opcode ==  MULSS_XMM_XMM || opcode ==  NOT_R16 || opcode ==  ORPS_XMM_XMM || opcode ==  OR_R16_R16 || opcode ==  PADDQ_XMM_XMM || opcode ==  PMOVSXDQ_XMM_XMM || opcode ==  PMOVZXWQ_XMM_XMM || opcode ==  POPCNT_R16_R16 || opcode ==  POR_XMM_XMM || opcode ==  RCPPS_XMM_XMM || opcode ==  RCPSS_XMM_XMM || opcode ==  ROL_R64_ONE || opcode ==  ROL_R16_ONE || opcode ==  RSQRTPS_XMM_XMM || opcode ==  SETB_RH || opcode ==  SETC_R8 || opcode ==  SETNGE_RH || opcode ==  SETNO_R8 || opcode ==  SETNZ_RH || opcode ==  SETPO_R8 || opcode ==  SHL_R8_ONE || opcode ==  SHL_R32_ONE || opcode ==  SHL_R64_ONE || opcode ==  SQRTPS_XMM_XMM || opcode ==  UNPCKLPD_XMM_XMM || opcode ==  VADDPS_XMM_XMM_XMM || opcode ==  VBROADCASTSD_YMM_XMM || opcode ==  VBROADCASTSS_XMM_XMM || opcode ==  VCVTSD2SI_R32_XMM || opcode ==  VCVTSS2SD_XMM_XMM_XMM || opcode ==  VCVTSS2SI_R32_XMM || opcode ==  VCVTTPD2DQ_XMM_XMM || opcode ==  VCVTTSD2SI_R32_XMM || opcode ==  VFMADD231PD_YMM_YMM_YMM || opcode ==  VFMADD231PS_XMM_XMM_XMM || opcode ==  VFMSUB132PS_XMM_XMM_XMM || opcode ==  VFNMSUB231PS_XMM_XMM_XMM || opcode ==  VMAXPS_XMM_XMM_XMM || opcode ==  VMINPS_XMM_XMM_XMM || opcode ==  VMOVAPD_YMM_YMM || opcode ==  VMOVAPS_YMM_YMM || opcode ==  VMOVDDUP_XMM_XMM || opcode ==  VMOVLHPS_XMM_XMM_XMM || opcode ==  VMOVSD_XMM_XMM_XMM || opcode ==  VMOVSS_XMM_XMM_XMM || opcode ==  VMOVUPS_YMM_YMM || opcode ==  VORPS_XMM_XMM_XMM || opcode ==  VPBROADCASTQ_YMM_XMM || opcode ==  VPXOR_XMM_XMM_XMM || opcode ==  VSQRTPD_XMM_XMM || opcode ==  VSUBPD_XMM_XMM_XMM || opcode ==  XADD_R8_R8 || opcode ==  XADD_R32_R32 || opcode ==  XADD_R64_R64 || opcode ==  XCHG_EAX_R32 || opcode ==  XCHG_R32_EAX || opcode ==  XCHG_R64_RAX || opcode ==  XCHG_RAX_R64 || opcode ==  XCHG_R16_AX || opcode ==  XOR_R8_RH || opcode ==  XOR_R32_R32 || opcode ==  XORPS_XMM_XMM || opcode ==  ADC_R8_RH || opcode ==  ADC_RH_RH || opcode ==  ADD_R8_R8 || opcode ==  ADD_R32_R32 || opcode ==  ADDPS_XMM_XMM || opcode ==  ADDSD_XMM_XMM || opcode ==  ADDSS_XMM_XMM || opcode ==  CVTDQ2PD_XMM_XMM || opcode ==  CVTDQ2PS_XMM_XMM || opcode ==  CVTPD2PS_XMM_XMM || opcode ==  CVTSD2SI_R32_XMM || opcode ==  CVTSD2SS_XMM_XMM || opcode ==  CVTTPD2DQ_XMM_XMM || opcode ==  CWDE || opcode ==  DEC_R32 || opcode ==  DEC_R16 || opcode ==  DIVSD_XMM_XMM || opcode ==  MAXPS_XMM_XMM || opcode ==  MINPS_XMM_XMM || opcode ==  MOV_R8_R8 || opcode ==  MOV_R32_R32 || opcode ==  MOVZX_R32_R8 || opcode ==  NEG_R16 || opcode ==  NOT_R32 || opcode ==  NOT_R64 || opcode ==  ORPD_XMM_XMM || opcode ==  PMOVZXDQ_XMM_XMM || opcode ==  PUNPCKLQDQ_XMM_XMM || opcode ==  PXOR_XMM_XMM || opcode ==  ROL_R8_ONE || opcode ==  SAL_R8_ONE || opcode ==  SAL_R32_ONE || opcode ==  SAL_R64_ONE || opcode ==  SETGE_R8 || opcode ==  SETL_RH || opcode ==  SETNAE_R8 || opcode ==  SETNB_RH || opcode ==  SETNBE_R8 || opcode ==  SETNC_RH || opcode ==  SETNE_RH || opcode ==  SETNS_R8 || opcode ==  SETPE_R8 || opcode ==  SHL_RH_ONE || opcode ==  SHRX_R64_R64_R64 || opcode ==  SQRTPD_XMM_XMM || opcode ==  SQRTSD_XMM_XMM || opcode ==  SQRTSS_XMM_XMM || opcode ==  SUBPS_XMM_XMM || opcode ==  SUBSD_XMM_XMM || opcode ==  UNPCKHPD_XMM_XMM || opcode ==  VADDSS_XMM_XMM_XMM || opcode ==  VANDNPS_XMM_XMM_XMM || opcode ==  VBROADCASTSS_YMM_XMM || opcode ==  VCVTPD2DQ_XMM_XMM || opcode ==  VDIVPD_XMM_XMM_XMM || opcode ==  VDIVPS_XMM_XMM_XMM || opcode ==  VFMADD213SS_XMM_XMM_XMM || opcode ==  VFMSUB231PS_XMM_XMM_XMM || opcode ==  VFMSUB231SS_XMM_XMM_XMM || opcode ==  VFNMADD213PS_XMM_XMM_XMM || opcode ==  VFNMSUB132PS_XMM_XMM_XMM || opcode ==  VFNMSUB132SS_XMM_XMM_XMM || opcode ==  VFNMSUB213PS_XMM_XMM_XMM || opcode ==  VHADDPD_XMM_XMM_XMM || opcode ==  VMAXPD_XMM_XMM_XMM || opcode ==  VMAXSD_XMM_XMM_XMM || opcode ==  VMAXSS_XMM_XMM_XMM || opcode ==  VMINPD_XMM_XMM_XMM || opcode ==  VMOVD_R32_XMM || opcode ==  VMOVDDUP_YMM_YMM || opcode ==  VMOVQ_XMM_XMM || opcode ==  VMOVSLDUP_XMM_XMM || opcode ==  VMULPD_XMM_XMM_XMM || opcode ==  VMULSD_XMM_XMM_XMM || opcode ==  VMULSS_XMM_XMM_XMM || opcode ==  VPADDQ_XMM_XMM_XMM || opcode ==  VPBROADCASTD_XMM_XMM || opcode ==  VPBROADCASTD_YMM_XMM || opcode ==  VPOR_XMM_XMM_XMM || opcode ==  VRCPSS_XMM_XMM_XMM || opcode ==  VSUBSS_XMM_XMM_XMM || opcode ==  VUNPCKLPD_XMM_XMM_XMM || opcode ==  VXORPD_XMM_XMM_XMM || opcode ==  XADD_R8_RH || opcode ==  XADD_RH_R8 || opcode ==  XADD_R16_R16 || opcode ==  XCHG_R8_RH || opcode ==  XCHG_RH_RH || opcode ==  XCHG_AX_R16 || opcode ==  XOR_R8_R8 || opcode ==  ADD_R8_RH || opcode ==  ADD_RH_R8 || opcode ==  BSWAP_R32 || opcode ==  CMOVBE_R64_R64 || opcode ==  CMOVNZ_R32_R32 || opcode ==  CVTPD2DQ_XMM_XMM || opcode ==  CVTTSD2SI_R32_XMM || opcode ==  CWD || opcode ==  DIVSS_XMM_XMM || opcode ==  HADDPD_XMM_XMM || opcode ==  INC_RH || opcode ==  INC_R64 || opcode ==  MAXPD_XMM_XMM || opcode ==  MAXSD_XMM_XMM || opcode ==  MAXSS_XMM_XMM || opcode ==  MINPD_XMM_XMM || opcode ==  MINSS_XMM_XMM || opcode ==  MOVQ_XMM_R64 || opcode ==  MOVSLDUP_XMM_XMM || opcode ==  MOVZX_R64_R16 || opcode ==  NEG_R8 || opcode ==  NOT_R8 || opcode ==  OR_RH_R8 || opcode ==  OR_R32_R32 || opcode ==  PANDN_XMM_XMM || opcode ==  POPCNT_R32_R32 || opcode ==  PUNPCKHQDQ_XMM_XMM || opcode ==  RCL_R8_ONE || opcode ==  ROL_RH_ONE || opcode ==  ROL_R32_ONE || opcode ==  RSQRTSS_XMM_XMM || opcode ==  SAL_RH_ONE || opcode ==  SETA_RH || opcode ==  SETAE_RH || opcode ==  SETBE_RH || opcode ==  SETE_RH || opcode ==  SETGE_RH || opcode ==  SETL_R8 || opcode ==  SETLE_RH || opcode ==  SETNE_R8 || opcode ==  SETNGE_R8 || opcode ==  SETNL_R8 || opcode ==  SETNZ_R8 || opcode ==  SETO_RH || opcode ==  SETPO_RH || opcode ==  SETZ_RH || opcode ==  SHL_R16_ONE || opcode ==  SUBPD_XMM_XMM || opcode ==  SUBSS_XMM_XMM || opcode ==  VADDSD_XMM_XMM_XMM || opcode ==  VANDNPD_XMM_XMM_XMM || opcode ==  VCVTDQ2PD_XMM_XMM || opcode ==  VCVTSD2SS_XMM_XMM_XMM || opcode ==  VCVTTSS2SI_R32_XMM || opcode ==  VDIVSD_XMM_XMM_XMM || opcode ==  VFMADD231SS_XMM_XMM_XMM || opcode ==  VFMSUB132PD_XMM_XMM_XMM || opcode ==  VFMSUB132SS_XMM_XMM_XMM || opcode ==  VFMSUB213PS_XMM_XMM_XMM || opcode ==  VFNMADD132PS_XMM_XMM_XMM || opcode ==  VFNMADD231PD_XMM_XMM_XMM || opcode ==  VFNMSUB213SS_XMM_XMM_XMM || opcode ==  VMINSD_XMM_XMM_XMM || opcode ==  VMOVSHDUP_XMM_XMM || opcode ==  VMOVSLDUP_YMM_YMM || opcode ==  VORPD_YMM_YMM_YMM || opcode ==  VPANDN_XMM_XMM_XMM || opcode ==  VPBROADCASTW_YMM_XMM || opcode ==  VSQRTSD_XMM_XMM_XMM || opcode ==  VSQRTSS_XMM_XMM_XMM || opcode ==  VSUBPS_XMM_XMM_XMM || opcode ==  VSUBSD_XMM_XMM_XMM || opcode ==  VUNPCKHPD_YMM_YMM_YMM || opcode ==  VUNPCKHPS_XMM_XMM_XMM || opcode ==  XCHG_R8_R8 || opcode ==  XCHG_RH_R8 || opcode ==  XOR_RH_R8 || opcode ==  XORPD_XMM_XMM || opcode ==  ADD_RH_RH || opcode ==  ADDSUBPD_XMM_XMM || opcode ==  ADDSUBPS_XMM_XMM || opcode ==  AND_R8_RH || opcode ==  ANDN_R32_R32_R32 || opcode ==  ANDNPS_XMM_XMM || opcode ==  BLSMSK_R32_R32 || opcode ==  CMOVE_R16_R16 || opcode ==  CMOVNAE_R32_R32 || opcode ==  CMOVNA_R64_R64 || opcode ==  CMOVNE_R32_R32 || opcode ==  CMOVNE_R64_R64 || opcode ==  CMOVNE_R16_R16 || opcode ==  CMOVNS_R32_R32 || opcode ==  CMOVPO_R32_R32 || opcode ==  MINSD_XMM_XMM || opcode ==  MOVHLPS_XMM_XMM || opcode ==  MOVSX_R16_R8 || opcode ==  MOVSHDUP_XMM_XMM || opcode ==  NEG_RH || opcode ==  NOT_RH || opcode ==  OR_R8_RH || opcode ==  OR_RH_RH || opcode ==  PAND_XMM_XMM || opcode ==  PUNPCKHDQ_XMM_XMM || opcode ==  SAL_R16_ONE || opcode ==  SETA_R8 || opcode ==  SETNA_RH || opcode ==  SETNC_R8 || opcode ==  SETNG_R8 || opcode ==  SETNG_RH || opcode ==  SETNL_RH || opcode ==  SETP_RH || opcode ==  SETPE_RH || opcode ==  SETS_RH || opcode ==  UNPCKHPS_XMM_XMM || opcode ==  VANDNPD_YMM_YMM_YMM || opcode ==  VDIVSS_XMM_XMM_XMM || opcode ==  VFMADD132SS_XMM_XMM_XMM || opcode ==  VFMSUB213PD_XMM_XMM_XMM || opcode ==  VFMSUB231PD_XMM_XMM_XMM || opcode ==  VFMSUB231SD_XMM_XMM_XMM || opcode ==  VFNMADD132SS_XMM_XMM_XMM || opcode ==  VFNMADD213PD_XMM_XMM_XMM || opcode ==  VFNMADD231PS_XMM_XMM_XMM || opcode ==  VHADDPD_YMM_YMM_YMM || opcode ==  VHSUBPD_XMM_XMM_XMM || opcode ==  VMINSS_XMM_XMM_XMM || opcode ==  VMOVD_XMM_R32 || opcode ==  VMOVMSKPD_R32_XMM || opcode ==  VMOVSHDUP_YMM_YMM || opcode ==  VORPS_YMM_YMM_YMM || opcode ==  VPMOVZXBQ_XMM_XMM || opcode ==  VPMOVZXDQ_YMM_XMM || opcode ==  VPOR_YMM_YMM_YMM || opcode ==  VPUNPCKHQDQ_YMM_YMM_YMM || opcode ==  VPUNPCKLQDQ_YMM_YMM_YMM || opcode ==  VRSQRTSS_XMM_XMM_XMM || opcode ==  XOR_RH_RH || opcode ==  AND_RH_R8 || opcode ==  AND_R32_R32 || opcode ==  ANDNPD_XMM_XMM || opcode ==  ANDPD_XMM_XMM || opcode ==  ANDPS_XMM_XMM || opcode ==  BSWAP_R64 || opcode ==  CDQ || opcode ==  CMC || opcode ==  CMOVA_R32_R32 || opcode ==  CMOVB_R32_R32 || opcode ==  CMOVB_R64_R64 || opcode ==  CMOVC_R16_R16 || opcode ==  CMOVE_R32_R32 || opcode ==  CMOVGE_R16_R16 || opcode ==  CMOVNBE_R32_R32 || opcode ==  CMOVNBE_R64_R64 || opcode ==  CMOVNC_R32_R32 || opcode ==  CMOVNP_R32_R32 || opcode ==  CMOVNP_R16_R16 || opcode ==  CMOVNZ_R64_R64 || opcode ==  CMOVPE_R32_R32 || opcode ==  CMOVPE_R64_R64 || opcode ==  CVTSI2SS_XMM_R32 || opcode ==  DEC_RH || opcode ==  HSUBPD_XMM_XMM || opcode ==  MOVD_XMM_R32 || opcode ==  MOVMSKPD_R64_XMM || opcode ==  OR_R8_R8 || opcode ==  PHADDD_XMM_XMM || opcode ==  PMOVZXBQ_XMM_XMM || opcode ==  RCL_RH_ONE || opcode ==  SAR_R64_ONE || opcode ==  SETLE_R8 || opcode ==  SETNA_R8 || opcode ==  SETNB_R8 || opcode ==  SETNBE_RH || opcode ==  SETNLE_R8 || opcode ==  SETNS_RH || opcode ==  TEST_R32_R32 || opcode ==  UNPCKLPS_XMM_XMM || opcode ==  VADDSUBPD_XMM_XMM_XMM || opcode ==  VADDSUBPS_XMM_XMM_XMM || opcode ==  VANDNPS_YMM_YMM_YMM || opcode ==  VANDPD_XMM_XMM_XMM || opcode ==  VANDPS_XMM_XMM_XMM || opcode ==  VCVTSI2SD_XMM_XMM_R32 || opcode ==  VFMADD231PD_XMM_XMM_XMM || opcode ==  VFMSUB132SD_XMM_XMM_XMM || opcode ==  VFMSUB213SD_XMM_XMM_XMM || opcode ==  VFNMADD231SS_XMM_XMM_XMM || opcode ==  VFNMSUB132PD_XMM_XMM_XMM || opcode ==  VHSUBPD_YMM_YMM_YMM || opcode ==  VMOVMSKPD_R64_XMM || opcode ==  VMOVMSKPD_R64_YMM || opcode ==  VPAND_XMM_XMM_XMM || opcode ==  VPANDN_YMM_YMM_YMM || opcode ==  VPBROADCASTB_XMM_XMM || opcode ==  VPBROADCASTW_XMM_XMM || opcode ==  VPUNPCKHDQ_XMM_XMM_XMM || opcode ==  VUNPCKHPD_XMM_XMM_XMM || opcode ==  VUNPCKHPS_YMM_YMM_YMM || opcode ==  VUNPCKLPD_YMM_YMM_YMM || opcode ==  XADD_RH_RH || opcode ==  AND_R8_R8 || opcode ==  BLSR_R32_R32 || opcode ==  BT_R64_R64 || opcode ==  CMOVAE_R32_R32 || opcode ==  CMOVA_R64_R64 || opcode ==  CMOVB_R16_R16 || opcode ==  CMOVC_R32_R32 || opcode ==  CMOVL_R32_R32 || opcode ==  CMOVNAE_R64_R64 || opcode ==  CMOVNAE_R16_R16 || opcode ==  CMOVNA_R32_R32 || opcode ==  CMOVNGE_R32_R32 || opcode ==  CMOVNS_R16_R16 || opcode ==  CMOVPE_R16_R16 || opcode ==  CMOVP_R32_R32 || opcode ==  CMOVPO_R16_R16 || opcode ==  CMOVP_R64_R64 || opcode ==  CMOVZ_R32_R32 || opcode ==  CVTSI2SD_XMM_R32 || opcode ==  MOVMSKPD_R32_XMM || opcode ==  NEG_R32 || opcode ==  PADDD_XMM_XMM || opcode ==  PUNPCKLDQ_XMM_XMM || opcode ==  SAR_R8_ONE || opcode ==  SAR_R32_ONE || opcode ==  SAR_R16_ONE || opcode ==  SBB_RH_R8 || opcode ==  SBB_RH_RH || opcode ==  SBB_R32_R32 || opcode ==  SBB_R64_R64 || opcode ==  SBB_R16_R16 || opcode ==  SETG_R8 || opcode ==  SETNLE_RH || opcode ==  SUB_R64_R64 || opcode ==  TEST_R8_R8 || opcode ==  TEST_R16_R16 || opcode ==  VANDPS_YMM_YMM_YMM || opcode ==  VCVTSI2SS_XMM_XMM_R32 || opcode ==  VCVTTPS2DQ_XMM_XMM || opcode ==  VFMADD132PD_XMM_XMM_XMM || opcode ==  VFMADD132SD_XMM_XMM_XMM || opcode ==  VFMADD213PS_XMM_XMM_XMM || opcode ==  VFMADD213SD_XMM_XMM_XMM || opcode ==  VFMADDSUB132PD_XMM_XMM_XMM || opcode ==  VFNMADD132SD_XMM_XMM_XMM || opcode ==  VFNMADD213SS_XMM_XMM_XMM || opcode ==  VFNMSUB213PD_XMM_XMM_XMM || opcode ==  VMOVMSKPD_R32_YMM || opcode ==  VPBROADCASTB_YMM_XMM || opcode ==  VPHADDD_XMM_XMM_XMM || opcode ==  VPHADDD_YMM_YMM_YMM || opcode ==  VPMOVZXBQ_YMM_XMM || opcode ==  VPUNPCKHDQ_YMM_YMM_YMM || opcode ==  VPUNPCKLDQ_XMM_XMM_XMM || opcode ==  VUNPCKLPS_YMM_YMM_YMM || opcode ==  CMOVAE_R64_R64 || opcode ==  CMOVBE_R32_R32 || opcode ==  CMOVC_R64_R64 || opcode ==  CMOVLE_R64_R64 || opcode ==  CMOVNB_R32_R32 || opcode ==  CMOVNGE_R16_R16 || opcode ==  CMOVNLE_R32_R32 || opcode ==  CMOVNLE_R64_R64 || opcode ==  CMOVNL_R32_R32 || opcode ==  CMOVNO_R32_R32 || opcode ==  CMOVO_R32_R32 || opcode ==  CMOVP_R16_R16 || opcode ==  CMOVS_R16_R16 || opcode ==  CMP_R8_RH || opcode ==  CMP_R64_R64 || opcode ==  CVTTPS2DQ_XMM_XMM || opcode ==  HSUBPS_XMM_XMM || opcode ==  NEG_R64 || opcode ==  ROR_R8_ONE || opcode ==  SAR_RH_ONE || opcode ==  SBB_R8_R8 || opcode ==  SBB_R8_RH || opcode ==  SETG_RH || opcode ==  SUB_RH_R8 || opcode ==  SUB_RH_RH || opcode ==  SUB_R32_R32 || opcode ==  SUB_R16_R16 || opcode ==  TEST_R8_RH || opcode ==  TEST_R64_R64 || opcode ==  VANDPD_YMM_YMM_YMM || opcode ==  VFMADD132PS_XMM_XMM_XMM || opcode ==  VFMADD213PD_XMM_XMM_XMM || opcode ==  VFMADD231SD_XMM_XMM_XMM || opcode ==  VFMADDSUB213PS_XMM_XMM_XMM || opcode ==  VFNMSUB213SD_XMM_XMM_XMM || opcode ==  VFNMSUB231PD_XMM_XMM_XMM || opcode ==  VHADDPS_XMM_XMM_XMM || opcode ==  VHSUBPS_YMM_YMM_YMM || opcode ==  VPADDD_XMM_XMM_XMM || opcode ==  VPADDD_YMM_YMM_YMM || opcode ==  VPMOVSXDQ_YMM_XMM || opcode ==  VPSUBQ_XMM_XMM_XMM || opcode ==  VPUNPCKLDQ_YMM_YMM_YMM || opcode ==  VUNPCKLPS_XMM_XMM_XMM || opcode ==  VXORPD_YMM_YMM_YMM || opcode ==  VXORPS_YMM_YMM_YMM || opcode ==  AND_RH_RH || opcode ==  AND_R64_R64 || opcode ==  BLSI_R32_R32 || opcode ==  CMOVA_R16_R16 || opcode ==  CMOVBE_R16_R16 || opcode ==  CMOVGE_R32_R32 || opcode ==  CMOVG_R32_R32 || opcode ==  CMOVG_R64_R64 || opcode ==  CMOVLE_R16_R16 || opcode ==  CMOVL_R16_R16 || opcode ==  CMOVNB_R16_R16 || opcode ==  CMOVNC_R64_R64 || opcode ==  CMOVNG_R32_R32 || opcode ==  CMOVNG_R64_R64 || opcode ==  CMOVNO_R16_R16 || opcode ==  CMOVO_R16_R16 || opcode ==  CMOVS_R32_R32 || opcode ==  CMP_R8_R8 || opcode ==  CMP_RH_R8 || opcode ==  CMP_RH_RH || opcode ==  CMP_R32_R32 || opcode ==  CMP_R16_R16 || opcode ==  CMPXCHG_RH_RH || opcode ==  CMPXCHG_R64_R64 || opcode ==  HADDPS_XMM_XMM || opcode ==  PHSUBD_XMM_XMM || opcode ==  PSUBQ_XMM_XMM || opcode ==  ROR_RH_ONE || opcode ==  SUB_R8_R8 || opcode ==  SUB_R8_RH || opcode ==  TEST_RH_R8 || opcode ==  TEST_RH_RH || opcode ==  TZCNT_R32_R32 || opcode ==  VFMADDSUB132PS_XMM_XMM_XMM || opcode ==  VFMADDSUB231PS_XMM_XMM_XMM || opcode ==  VFMSUBADD213PD_XMM_XMM_XMM || opcode ==  VFNMADD132PD_XMM_XMM_XMM || opcode ==  VFNMADD213SD_XMM_XMM_XMM || opcode ==  VFNMADD231SD_XMM_XMM_XMM || opcode ==  VFNMSUB132SD_XMM_XMM_XMM || opcode ==  VFNMSUB231SD_XMM_XMM_XMM || opcode ==  VFNMSUB231SS_XMM_XMM_XMM || opcode ==  VHSUBPS_XMM_XMM_XMM || opcode ==  VMOVMSKPS_R32_XMM || opcode ==  VMOVMSKPS_R64_XMM || opcode ==  VPAND_YMM_YMM_YMM || opcode ==  VPXOR_YMM_YMM_YMM || opcode ==  AND_R16_R16 || opcode ==  CMOVAE_R16_R16 || opcode ==  CMOVGE_R64_R64 || opcode ==  CMOVG_R16_R16 || opcode ==  CMOVLE_R32_R32 || opcode ==  CMOVNA_R16_R16 || opcode ==  CMOVNBE_R16_R16 || opcode ==  CMOVNB_R64_R64 || opcode ==  CMOVNC_R16_R16 || opcode ==  CMOVNG_R16_R16 || opcode ==  CMOVNL_R16_R16 || opcode ==  CMPXCHG_RH_R8 || opcode ==  CMPXCHG_R16_R16 || opcode ==  MOVMSKPS_R32_XMM || opcode ==  ROR_R16_ONE || opcode ==  SHRX_R32_R32_R32 || opcode ==  TZCNT_R16_R16 || opcode ==  VFMSUBADD231PD_XMM_XMM_XMM || opcode ==  VHADDPS_YMM_YMM_YMM || opcode ==  VPHSUBD_XMM_XMM_XMM || opcode ==  VPMOVZXWD_XMM_XMM || opcode ==  BT_R32_R32 || opcode ==  CMOVL_R64_R64 || opcode ==  CMOVNLE_R16_R16 || opcode ==  CMOVNO_R64_R64 || opcode ==  CMOVO_R64_R64 || opcode ==  CMOVS_R64_R64 || opcode ==  CMPXCHG_R8_R8 || opcode ==  MOVMSKPS_R64_XMM || opcode ==  PMOVZXWD_XMM_XMM || opcode ==  VFMADDSUB231PD_XMM_XMM_XMM || opcode ==  VFMSUBADD132PD_XMM_XMM_XMM || opcode ==  VPHSUBD_YMM_YMM_YMM || opcode ==  VPMOVZXBD_XMM_XMM || opcode ==  VPMOVZXWD_YMM_XMM || opcode ==  VPMOVZXWQ_YMM_XMM || opcode ==  VPSUBD_XMM_XMM_XMM || opcode ==  CMOVNGE_R64_R64 || opcode ==  CMOVNL_R64_R64 || opcode ==  CMOVNS_R64_R64 || opcode ==  CMOVPO_R64_R64 || opcode ==  CMPXCHG_R8_RH || opcode ==  PMOVSXWD_XMM_XMM || opcode ==  PMOVZXBD_XMM_XMM || opcode ==  PSUBD_XMM_XMM || opcode ==  PUNPCKLWD_XMM_XMM || opcode ==  VFMADDSUB213PD_XMM_XMM_XMM || opcode ==  VFMSUBADD231PS_XMM_XMM_XMM || opcode ==  VPMOVZXBD_YMM_XMM || opcode ==  VPSUBD_YMM_YMM_YMM || opcode ==  CMOVNP_R64_R64 || opcode ==  PMOVSXBD_XMM_XMM || opcode ==  PUNPCKHWD_XMM_XMM || opcode ==  VFMSUB213SS_XMM_XMM_XMM || opcode ==  VFMSUBADD213PS_XMM_XMM_XMM || opcode ==  VPMOVSXWD_YMM_XMM || opcode ==  VPMOVSXWQ_XMM_XMM || opcode ==  VPUNPCKLWD_XMM_XMM_XMM || opcode ==  PMOVSXWQ_XMM_XMM || opcode ==  VPMOVSXBD_XMM_XMM || opcode ==  VPMOVSXWD_XMM_XMM || opcode ==  VPMOVSXWQ_YMM_XMM || opcode ==  VPUNPCKHWD_XMM_XMM_XMM || opcode ==  VPUNPCKLWD_YMM_YMM_YMM || opcode ==  VPMOVSXBD_YMM_XMM || opcode ==  VPMOVSXBQ_YMM_XMM || opcode ==  VPUNPCKHWD_YMM_YMM_YMM || opcode ==  PMOVSXBQ_XMM_XMM || opcode ==  VPMOVSXBQ_XMM_XMM || opcode ==  VFMSUBADD132PS_XMM_XMM_XMM 
        */
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
