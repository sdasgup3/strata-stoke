// Copyright 2013-2016 Stanford University
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


#include "src/validator/handler.h"

using namespace stoke;
using namespace x64asm;
using namespace std;


bool Handler::regset_is_supported(RegSet rs) const {
  /* Check to make sure all liveout are supported. */
  /* Right now we support gps, xmms, ACOPSZ eflags */
  RegSet supported = (RegSet::all_gps() | RegSet::all_ymms()) +
                     eflags_cf + eflags_of +
                     eflags_pf + eflags_sf + eflags_zf;

  // TODO mxcsr's presense here is a bug.  See #339.
  for (size_t i = 0; i < mxcsr.size(); ++i) {
    supported = supported + mxcsr[i];
  }

  // Do the check.
  return (supported & rs) == rs;
}

/* Returns true if the validator supports all the operands of the instruction. */
bool Handler::operands_supported(const Instruction& instr) {

  for (size_t i = 0; i < instr.arity(); ++i) {
    auto& o = instr.get_operand<Operand>(i);
    if (!o.is_gp_register() && !o.is_sse_register() && !o.is_immediate() &&
        !o.is_typical_memory() && o.type() != Type::LABEL) {
      error_ = "Operand " + to_string(i) + " not supported.";
      return false;
    }
  }

  return true;
}

SymBool Handler::plus_of(SymBool arg1_msb, SymBool arg2_msb, SymBool total_msb) const {
  return (arg1_msb == arg2_msb) & (arg1_msb != total_msb);
}

SymBool Handler::minus_of(SymBool arg1_msb, SymBool arg2_msb, SymBool total_msb) const {
  return (arg2_msb == total_msb) & (arg1_msb != arg2_msb);
}

SymBool Handler::makeAF(SymBitVector arg1, SymBitVector arg2, SymBitVector total) const {
  auto res =  arg1 ^ arg2 ^ total;
  return res[4];
}

const array<const char*, X64ASM_NUM_OPCODES> Handler::att_ = {{
    "<label definition>"
#include "src/ext/x64asm/src/opcode.att"
  }
};


