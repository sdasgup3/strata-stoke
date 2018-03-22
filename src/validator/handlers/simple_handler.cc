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


#include "src/validator/error.h"
#include "src/validator/handlers/simple_handler.h"

using namespace std;
using namespace stoke;
using namespace x64asm;

namespace {

const SymBitVector vectorize(const SymFunction& f, const SymBitVector& a, const SymBitVector& b, const SymBitVector& c) {
  auto w = f.args[0];
  auto maxw = max(f.return_type, f.args[0]);
  if (f.args.size() == 3) {
    auto res = f(a[w-1][0], b[w-1][0], c[w-1][0]);
    for (auto i = 1; i < 256 / maxw; i++) {
      res = f(a[(i+1)*w-1][i*w], b[(i+1)*w-1][i*w], c[(i+1)*w-1][i*w]) || res;
    }
    return res;
  } else if (f.args.size() == 1) {
    auto res = f(b[w-1][0]);
    for (auto i = 1; i < 256 / maxw; i++) {
      res = f(b[(i+1)*w-1][i*w]) || res;
    }
    return res;
  } else {
    assert(false);
  }
  return a;
}

}

void SimpleHandler::add_all() {

  add_opcode_str({"andb", "andw", "andl", "andq"},
  [this] (Operand dst, Operand src, SymBitVector a, SymBitVector b, SymState& ss) {
    if (src.size() < dst.size())
      b = b.extend(dst.size());

    ss.set(dst, a & b);
    ss.set(eflags_cf, SymBool::_false());
    ss.set(eflags_of, SymBool::_false());
    ss.set(eflags_af, SymBool::tmp_var());
    ss.set_szp_flags(a & b);
  });

  add_opcode_str({"andnl", "andnq"},
  [this] (Operand dst, Operand src1, Operand src2, SymBitVector a, SymBitVector b, SymBitVector c, SymState& ss) {
    auto tmp = (!b) & c;
    ss.set(dst, tmp);
    ss.set(eflags_sf, tmp[dst.size()-1]);
    ss.set(eflags_zf, tmp == SymBitVector::constant(dst.size(), 0));
    ss.set(eflags_of, SymBool::_false());
    ss.set(eflags_cf, SymBool::_false());
    ss.set(eflags_af, SymBool::tmp_var());
    ss.set(eflags_pf, SymBool::tmp_var());
  });

  add_opcode_str({"bextrl", "bextrq"},
  [this] (Operand dst, Operand src1, Operand src2, SymBitVector a, SymBitVector b, SymBitVector c, SymState& ss) {
    size_t size = dst.size();
    auto start = c[7][0];
    auto len = c[15][8];
    auto start_zx = SymBitVector::constant(504, 0) || start;
    auto len_zx = SymBitVector::constant(504, 0) || len;
    auto temp = SymBitVector::constant(512 - size, 0) || b;

    // compute temp[511:start]
    auto shift = temp >> start_zx;
    // create bitmask to get temp[len-1:0]
    auto bitmask = !((SymBitVector::constant(512, -1) >> len_zx) << len_zx);
    // finish getting temp[start+len-1:start]; it's already zero extended to length 512!
    auto extract = shift & bitmask;
    auto result = extract[size-1][0];
    ss.set(dst, result);

    ss.set(eflags_zf, result == SymBitVector::constant(size, 0));

    ss.set(eflags_of, SymBool::_false());
    ss.set(eflags_cf, SymBool::_false());

    ss.set(eflags_af, SymBool::tmp_var());
    ss.set(eflags_sf, SymBool::tmp_var());
    ss.set(eflags_pf, SymBool::tmp_var());
  });

  add_opcode_str({"blsrl", "blsrq"},
  [this] (Operand dst, Operand src, SymBitVector a, SymBitVector b, SymState& ss) {
    auto zero = SymBitVector::constant(dst.size(), 0);
    auto one  = SymBitVector::constant(dst.size(), 1);
    auto temp = (b - one) & b;
    ss.set(eflags_zf, temp == zero);
    ss.set(eflags_sf, temp[dst.size() - 1]);
    ss.set(eflags_cf, b == zero);
    ss.set(eflags_of, SymBool::_false());
    ss.set(dst, temp);
  });

  // to convert between Intel/AT&T mnemonics, see:
  // https://sourceware.org/binutils/docs/as/i386_002dMnemonics.html
  add_opcode_str({"cbtw", "cbw"},
  [this] (SymState& ss) {
    ss.set(ax, ss[al].extend(16));
  });

  add_opcode_str({"cltd", "cdq"},
  [this] (SymState& ss) {
    auto se = ss[eax].extend(64);
    ss.set(edx, se[63][32]);
  });

  add_opcode_str({"cltq", "cdqe"},
  [this] (SymState& ss) {
    ss.set(rax, ss[eax].extend(64));
  });

  add_opcode_str({"cqto", "cqo"},
  [this] (SymState& ss) {
    auto se = ss[rax].extend(128);
    ss.set(rdx, se[127][64]);
  });

  add_opcode_str({"cwtd", "cwd"},
  [this] (SymState& ss) {
    auto se = ss[ax].extend(32);
    ss.set(dx, se[31][16]);
  });

  add_opcode_str({"cwtl", "cwde"},
  [this] (SymState& ss) {
    ss.set(eax, ss[ax].extend(32));
  });

  add_opcode_str({"decb", "decw", "decl", "decq"},
  [this] (Operand dst, SymBitVector a, SymState& ss) {
    SymBitVector one = SymBitVector::constant(dst.size(), 1);

    ss.set(dst, a - one);
    ss.set(eflags_of, a[dst.size() - 1] &
           (a[dst.size() - 2][0] == SymBitVector::constant(dst.size() - 1, 0)));
    ss.set(eflags_af, a[3][0] == SymBitVector::constant(4, 0x0));
    ss.set_szp_flags(a - one, dst.size());

  });

  add_opcode_str({"imulq", "imull", "imulw"},
  [this] (Operand dst, Operand src, SymBitVector a, SymBitVector b, SymState& ss) {
    auto n = a.width();

    auto a_ext = a.extend(2*n);
    auto b_ext = b.extend(2*n);

    auto full_res = a_ext * b_ext;
    auto res = full_res[n-1][0];
    auto truncated_res = res.extend(2*n);
    auto of = full_res != truncated_res;

    ss.set(dst, res);

    ss.set(eflags_zf, SymBool::tmp_var());
    ss.set(eflags_af, SymBool::tmp_var());
    ss.set(eflags_pf, SymBool::tmp_var());

    ss.set(eflags_of, of);
    ss.set(eflags_cf, of);
    ss.set(eflags_sf, res[n-1]);
  });

  add_opcode_str({"imulq", "imull", "imulw"},
  [this] (Operand dst, Operand src1, Operand src2, SymBitVector a, SymBitVector b, SymBitVector c, SymState& ss) {
    auto n = a.width();

    auto a_ext = b.extend(2*n);
    auto b_ext = c.extend(2*n);

    auto full_res = a_ext * b_ext;
    auto res = full_res[n-1][0];
    auto truncated_res = res.extend(2*n);
    auto of = full_res != truncated_res;

    ss.set(dst, res);

    ss.set(eflags_zf, SymBool::tmp_var());
    ss.set(eflags_af, SymBool::tmp_var());
    ss.set(eflags_pf, SymBool::tmp_var());

    ss.set(eflags_of, of);
    ss.set(eflags_cf, of);
    ss.set(eflags_sf, res[n-1]);
  });

  add_opcode_str({"mulq", "mull", "mulw", "mulb"},
  [this] (Operand src, SymBitVector a, SymState& ss) {
    auto n = a.width();

    SymBitVector b;
    if (n == 8) {
      b = ss[Constants::al()];
    } else if (n == 16) {
      b = ss[Constants::ax()];
    } else if (n == 32) {
      b = ss[Constants::eax()];
    } else if (n == 64) {
      b = ss[Constants::rax()];
    } else {
      assert(false);
    }

    auto a_ext = SymBitVector::constant(n, 0) || a;
    auto b_ext = SymBitVector::constant(n, 0) || b;

    auto full_res = a_ext * b_ext;
    auto of = full_res[2*n-1][n] != SymBitVector::constant(n, 0);

    if (n == 8) {
      ss.set(Constants::ax(), full_res);
    } else if (n == 16) {
      ss.set(Constants::dx(), full_res[2*n-1][n]);
      ss.set(Constants::ax(), full_res[n-1][0]);
    } else if (n == 32) {
      ss.set(Constants::edx(), full_res[2*n-1][n]);
      ss.set(Constants::eax(), full_res[n-1][0]);
    } else if (n == 64) {
      ss.set(Constants::rdx(), full_res[2*n-1][n]);
      ss.set(Constants::rax(), full_res[n-1][0]);
    }

    ss.set(eflags_zf, SymBool::tmp_var());
    ss.set(eflags_af, SymBool::tmp_var());
    ss.set(eflags_pf, SymBool::tmp_var());
    ss.set(eflags_sf, SymBool::tmp_var());

    ss.set(eflags_of, of);
    ss.set(eflags_cf, of);
  });

  add_opcode_str({"imulq", "imull", "imulw", "imulb"},
  [this] (Operand src, SymBitVector a, SymState& ss) {
    auto n = a.width();

    SymBitVector b;
    if (n == 8) {
      b = ss[Constants::al()];
    } else if (n == 16) {
      b = ss[Constants::ax()];
    } else if (n == 32) {
      b = ss[Constants::eax()];
    } else if (n == 64) {
      b = ss[Constants::rax()];
    } else {
      assert(false);
    }

    auto a_ext = a.extend(2*n);
    auto b_ext = b.extend(2*n);

    auto full_res = a_ext * b_ext;
    auto of = full_res != full_res[n-1][0].extend(2*n);

    if (n == 8) {
      ss.set(Constants::ax(), full_res);
    } else if (n == 16) {
      ss.set(Constants::dx(), full_res[2*n-1][n]);
      ss.set(Constants::ax(), full_res[n-1][0]);
    } else if (n == 32) {
      ss.set(Constants::edx(), full_res[2*n-1][n]);
      ss.set(Constants::eax(), full_res[n-1][0]);
    } else if (n == 64) {
      ss.set(Constants::rdx(), full_res[2*n-1][n]);
      ss.set(Constants::rax(), full_res[n-1][0]);
    }

    ss.set(eflags_zf, SymBool::tmp_var());
    ss.set(eflags_af, SymBool::tmp_var());
    ss.set(eflags_pf, SymBool::tmp_var());

    ss.set(eflags_sf, full_res[n-1]);
    ss.set(eflags_of, of);
    ss.set(eflags_cf, of);
  });

  add_opcode_str({"idivb", "idivw", "idivl", "idivq"},
  [this] (Operand src, SymBitVector a, SymState& ss) {
    auto n = a.width();

    SymBitVector numerator;
    if (n == 8) {
      SymFunction f_quot("idiv_quotient_int8", 8, {16, 8});
      SymFunction f_rem("idiv_remainder_int8", 8, {16, 8});
      numerator = ss[Constants::ax()];
      ss.set(Constants::al(), f_quot(numerator, a));
      ss.set(Constants::ah(), f_rem(numerator, a));
    } else if (n == 16) {
      SymFunction f_quot("idiv_quotient_int16", 16, {32, 16});
      SymFunction f_rem("idiv_remainder_int16", 16, {32, 16});
      numerator = ss[Constants::dx()] || ss[Constants::ax()];
      ss.set(Constants::dx(), f_rem(numerator, a));
      ss.set(Constants::ax(), f_quot(numerator, a));
    } else if (n == 32) {
      SymFunction f_quot("idiv_quotient_int32", 32, {64, 32});
      SymFunction f_rem("idiv_remainder_int32", 32, {64, 32});
      numerator = ss[Constants::edx()] || ss[Constants::eax()];
      ss.set(Constants::edx(), f_rem(numerator, a));
      ss.set(Constants::eax(), f_quot(numerator, a));
    } else if (n == 64) {
      SymFunction f_quot("idiv_quotient_int64", 64, {128, 64});
      SymFunction f_rem("idiv_remainder_int64", 64, {128, 64});
      numerator = ss[Constants::rdx()] || ss[Constants::rax()];
      ss.set(Constants::rdx(), f_rem(numerator, a));
      ss.set(Constants::rax(), f_quot(numerator, a));
    } else {
      assert(false);
    }

    ss.set(eflags_zf, SymBool::tmp_var());
    ss.set(eflags_af, SymBool::tmp_var());
    ss.set(eflags_pf, SymBool::tmp_var());
    ss.set(eflags_sf, SymBool::tmp_var());
    ss.set(eflags_of, SymBool::tmp_var());
    ss.set(eflags_cf, SymBool::tmp_var());
  });

  add_opcode_str({"divb", "divw", "divl", "divq"},
  [this] (Operand src, SymBitVector a, SymState& ss) {
    auto n = a.width();

    SymBitVector numerator;
    if (n == 8) {
      SymFunction f_quot("div_quotient_int8", 8, {16, 8});
      SymFunction f_rem("div_remainder_int8", 8, {16, 8});
      numerator = ss[Constants::ax()];
      ss.set(Constants::al(), f_quot(numerator, a));
      ss.set(Constants::ah(), f_rem(numerator, a));
    } else if (n == 16) {
      SymFunction f_quot("div_quotient_int16", 16, {32, 16});
      SymFunction f_rem("div_remainder_int16", 16, {32, 16});
      numerator = ss[Constants::dx()] || ss[Constants::ax()];
      ss.set(Constants::dx(), f_rem(numerator, a));
      ss.set(Constants::ax(), f_quot(numerator, a));
    } else if (n == 32) {
      SymFunction f_quot("div_quotient_int32", 32, {64, 32});
      SymFunction f_rem("div_remainder_int32", 32, {64, 32});
      numerator = ss[Constants::edx()] || ss[Constants::eax()];
      ss.set(Constants::edx(), f_rem(numerator, a));
      ss.set(Constants::eax(), f_quot(numerator, a));
    } else if (n == 64) {
      SymFunction f_quot("div_quotient_int64", 64, {128, 64});
      SymFunction f_rem("div_remainder_int64", 64, {128, 64});
      numerator = ss[Constants::rdx()] || ss[Constants::rax()];
      ss.set(Constants::rdx(), f_rem(numerator, a));
      ss.set(Constants::rax(), f_quot(numerator, a));
    } else {
      assert(false);
    }

    ss.set(eflags_zf, SymBool::tmp_var());
    ss.set(eflags_af, SymBool::tmp_var());
    ss.set(eflags_pf, SymBool::tmp_var());
    ss.set(eflags_sf, SymBool::tmp_var());
    ss.set(eflags_of, SymBool::tmp_var());
    ss.set(eflags_cf, SymBool::tmp_var());
  });

  add_opcode_str({"incb", "incw", "incl", "incq"},
  [this] (Operand dst, SymBitVector a, SymState& ss) {
    SymBitVector one = SymBitVector::constant(dst.size(), 1);

    ss.set(dst, a + one);
    ss.set(eflags_of, !a[dst.size() - 1] &
           (a[dst.size()-2][0] == SymBitVector::constant(dst.size() - 1, -1)));
    ss.set(eflags_af, a[3][0] == SymBitVector::constant(4, 0xf));
    ss.set_szp_flags(a + one, dst.size());

  });

  // Extendng Base

  // Extend Strata Base: psrad
  add_opcode_str({"psrad"},
  [] (Operand dst, Operand src, SymBitVector a, SymBitVector b, SymState& ss) {
    auto shift_count1 = SymBitVector::constant(32, 32);
    auto shift_count2 = b[31][0];
    auto bb = b[63][0];
    auto const_ = SymBitVector::constant(64, 31);
    auto shift_count = (bb > const_).ite(shift_count1, shift_count2);

    auto result = a[31][0].s_shr(shift_count);

    for (uint16_t i = 1 ; i < a.width()/32; i++) {
      result = a[31 + i*32][i*32].s_shr(shift_count) || result;
    }
    ss.set(dst, result);
  });

  // Extend Strata Base: vpsrad
  add_opcode_str({"vpsrad"},
  [] (Operand dst, Operand src1, Operand src2, SymBitVector d, SymBitVector a, SymBitVector b, SymState& ss) {
    auto shift_count1 = SymBitVector::constant(32, 32);
    auto shift_count2 = b[31][0];
    auto bb = b[63][0];
    auto const_ = SymBitVector::constant(64, 31);
    auto shift_count = (bb > const_).ite(shift_count1, shift_count2);

    auto result = a[31][0].s_shr(shift_count);

    for (uint16_t i = 1 ; i < a.width()/32; i++) {
      result = a[31 + i*32][i*32].s_shr(shift_count) || result;
    }
    ss.set(dst, result, true);
  });

  // Extend Strata Base: psraw
  add_opcode_str({"psraw"},
  [] (Operand dst, Operand src, SymBitVector a, SymBitVector b, SymState& ss) {
    auto shift_count1 = SymBitVector::constant(16, 16);
    auto shift_count2 = b[15][0];
    auto bb = b[63][0];
    auto const_ = SymBitVector::constant(64, 15);
    auto shift_count = (bb > const_).ite(shift_count1, shift_count2);

    auto result = a[15][0].s_shr(shift_count);

    for (uint16_t i = 1 ; i < a.width()/16; i++) {
      result = a[15 + i*16][i*16].s_shr(shift_count) || result;
    }
    ss.set(dst, result);
  });


  // Extend Strata Base: vpsraw
  add_opcode_str({"vpsraw"},
  [] (Operand dst, Operand src1, Operand src2, SymBitVector d, SymBitVector a, SymBitVector b, SymState& ss) {
    auto shift_count1 = SymBitVector::constant(16, 16);
    auto shift_count2 = b[15][0];
    auto bb = b[63][0];
    auto const_ = SymBitVector::constant(64, 15);
    auto shift_count = (bb > const_).ite(shift_count1, shift_count2);

    auto result = a[15][0].s_shr(shift_count);

    for (uint16_t i = 1 ; i < a.width()/16; i++) {
      result = a[15 + i*16][i*16].s_shr(shift_count) || result;
    }
    ss.set(dst, result, true);
  });

  // Extend Strata Base: (v)ptest
  add_opcode_str({"ptest", "vptest"},
  [] (Operand dst, Operand src, SymBitVector a, SymBitVector b, SymState& ss) {
    auto aa = a[dst.size()-1][0];
    auto bb = b[dst.size()-1][0];
    auto temp1 =  aa & bb;
    auto temp2 = (!aa) & bb;
    auto zero = SymBitVector::constant(dst.size(), 0);

    ss.set(eflags_zf, temp1 == zero);
    ss.set(eflags_cf, temp2 == zero);
    ss.set(eflags_af, SymBool::_false());
    ss.set(eflags_of, SymBool::_false());
    ss.set(eflags_pf, SymBool::_false());
    ss.set(eflags_sf, SymBool::_false());
  });

  // Extend Strata Base: vtestpd
  add_opcode_str({"vtestpd"},
  [] (Operand dst, Operand src, SymBitVector a, SymBitVector b, SymState& ss) {
    auto aa = a[dst.size()-1][0];
    auto bb = b[dst.size()-1][0];
    auto temp1 =  aa & bb;
    auto temp2 = (!aa) & bb;
    auto zero = SymBitVector::constant(1, 0);

    auto cond1_ = (temp1[63][63] == zero);
    auto cond2_ = (temp2[63][63] == zero);
    for (size_t i = 2; i <= dst.size()/64; i++) {
      cond1_ = cond1_ & (temp1[i*64-1][i*64-1] == zero);
      cond2_ = cond2_ & (temp2[i*64-1][i*64-1] == zero);
    }

    ss.set(eflags_zf, cond1_);
    ss.set(eflags_cf, cond2_);
    ss.set(eflags_af, SymBool::_false());
    ss.set(eflags_of, SymBool::_false());
    ss.set(eflags_pf, SymBool::_false());
    ss.set(eflags_sf, SymBool::_false());
  });

  // Extend Strata Base: vtestps
  add_opcode_str({"vtestps"},
  [] (Operand dst, Operand src, SymBitVector a, SymBitVector b, SymState& ss) {
    auto aa = a[dst.size()-1][0];
    auto bb = b[dst.size()-1][0];
    auto temp1 =  aa & bb;
    auto temp2 = (!aa) & bb;
    auto zero = SymBitVector::constant(1, 0);

    auto cond1_ = (temp1[31][31] == zero);
    auto cond2_ = (temp2[31][31] == zero);
    for (size_t i = 2; i <= dst.size()/32; i++) {
      cond1_ = cond1_ & (temp1[i*32-1][i*32-1] == zero);
      cond2_ = cond2_ & (temp2[i*32-1][i*32-1] == zero);
    }

    ss.set(eflags_zf, cond1_);
    ss.set(eflags_cf, cond2_);
    ss.set(eflags_af, SymBool::_false());
    ss.set(eflags_of, SymBool::_false());
    ss.set(eflags_pf, SymBool::_false());
    ss.set(eflags_sf, SymBool::_false());
  });

  // (v)comisd
  add_opcode_str({"comisd", "vcomisd", "ucomisd", "vucomisd"},
  [] (Operand dst, Operand src, SymBitVector a, SymBitVector b, SymState& ss) {
    SymFunction f("comisd", 2, {64, 64});
    auto aa = a[63][0];
    auto bb = b[63][0];
    auto result = f (aa, bb);

    auto cond1Forzf = ((result == SymBitVector::constant(2, 0)) | (result == SymBitVector::constant(2, 3)));
    auto cond1Forpf = (result == SymBitVector::constant(2, 0));
    auto cond1Forcf = ((result == SymBitVector::constant(2, 0)) | (result == SymBitVector::constant(2, 2)));

    ss.set(eflags_zf, cond1Forzf);
    ss.set(eflags_pf, cond1Forpf);
    ss.set(eflags_cf, cond1Forcf);

    ss.set(eflags_af, SymBool::_false());
    ss.set(eflags_of, SymBool::_false());
    ss.set(eflags_sf, SymBool::_false());
  });

  // (v)comiss
  add_opcode_str({"comiss", "vcomiss", "ucomiss", "vucomiss"},
  [] (Operand dst, Operand src, SymBitVector a, SymBitVector b, SymState& ss) {
    SymFunction f("comiss", 2, {32, 32});
    auto aa = a[31][0];
    auto bb = b[31][0];
    auto result = f (aa, bb);

    auto cond1Forzf = ((result == SymBitVector::constant(2, 0)) | (result == SymBitVector::constant(2, 3)));
    auto cond1Forpf = (result == SymBitVector::constant(2, 0));
    auto cond1Forcf = ((result == SymBitVector::constant(2, 0)) | (result == SymBitVector::constant(2, 2)));

    ss.set(eflags_zf, cond1Forzf);
    ss.set(eflags_pf, cond1Forpf);
    ss.set(eflags_cf, cond1Forcf);

    ss.set(eflags_af, SymBool::_false());
    ss.set(eflags_of, SymBool::_false());
    ss.set(eflags_sf, SymBool::_false());
  });

  add_opcode_str({"pmovmskb", "vpmovmskb"},
  [this] (Operand dst, Operand src, SymBitVector a, SymBitVector b, SymState& ss) {
    size_t dst_size = dst.size();
    size_t src_size = src.size();

    auto mask = SymBitVector::from_bool(b[7]);
    for (size_t i = 1; i < src_size/8; ++i) {
      mask = SymBitVector::from_bool(b[8*i+7]) || mask;
    }

    size_t pad = dst_size - src_size/8;
    if (pad > 0)
      ss.set(dst, SymBitVector::constant(pad, 0) || mask);
    else
      ss.set(dst, mask);

  });

  // pblendvb
  add_opcode_str({"pblendvb"},
  [this] (Operand dst, Operand src1, Operand src2, SymBitVector a, SymBitVector b, SymBitVector c, SymState& ss) {

    size_t src_size = src1.size();
    auto result = (c[7][7] == SymBitVector::constant(1, 1)).ite(b[7][0], a[7][0]);
    for (size_t i = 1; i < src_size/8; ++i) {
      result =  (c[8*i+7][8*i+7] == SymBitVector::constant(1, 1)).ite(b[8*i+7][8*i], a[8*i+7][8*i]) || result;
    }

    ss.set(dst, result);
  });

  // vpblendvb
  add_opcode_str({"vpblendvb"},
                 [this] (Operand dst, Operand src1, Operand src2, Operand src3, SymBitVector a, SymBitVector b, SymBitVector c,
  SymBitVector d, SymState& ss) {

    size_t src_size = src1.size();
    auto result = (d[7][7] == SymBitVector::constant(1, 1)).ite(c[7][0], b[7][0]);
    for (size_t i = 1; i < src_size/8; ++i) {
      result =  (d[8*i+7][8*i+7] == SymBitVector::constant(1, 1)).ite(c[8*i+7][8*i], b[8*i+7][8*i]) || result;
    }

    ss.set(dst, result, true);

  });

  /* vpshufb
  add_opcode_str({"vpshufb"},
  [this] (Operand dst, Operand src1, Operand src2, SymBitVector a, SymBitVector b, SymBitVector c, SymState& ss) {
    size_t dest_size = dst.size();

    auto index = c[3][0];
    auto shufResult = b[index*8+7][index*8];
    auto result = (c[7][7] ==  SymBitVector::constant(1, 1)).ite(SymBitVector::constant(8, 0), shufResult);
    for (size_t i = 1; i <= 15; ++i) {
      auto index = c[(i*8)+3][i*8];
      result =  (c[(i * 8)+7][(i * 8)+7] ==  SymBitVector::constant(1, 1)).ite(SymBitVector::constant(8, 0), shufResult) || result;
    }

    ss.set(dst, result, true);

  });
  */

  // for min/max|ss/sd: can't be done with packed handler because the upper 96/64 bits are from src1, not dest in the v variant

  add_opcode_str({"minsd"},
  [] (Operand dst, Operand src, SymBitVector a, SymBitVector b, SymState& ss) {
    SymFunction f("mincmp_double", 1, {64, 64});
    auto aa = a[63][0];
    auto bb = b[63][0];
    ss.set(dst, a[127][64] || (f(aa, bb)[0]).ite(aa, bb));
  });

  add_opcode_str({"vminsd"},
  [] (Operand dst, Operand src1, Operand src2, SymBitVector a, SymBitVector b, SymBitVector c, SymState& ss) {
    SymFunction f("mincmp_double", 1, {64, 64});
    auto bb = b[63][0];
    auto cc = c[63][0];
    ss.set(dst, b[127][64] || (f(bb, cc)[0]).ite(bb, cc), true);
  });

  add_opcode_str({"minss"},
  [] (Operand dst, Operand src, SymBitVector a, SymBitVector b, SymState& ss) {
    SymFunction f("mincmp_single", 1, {32, 32});
    auto aa = a[31][0];
    auto bb = b[31][0];
    ss.set(dst, a[127][32] || (f(aa, bb)[0]).ite(aa, bb));
  });

  add_opcode_str({"vminss"},
  [] (Operand dst, Operand src1, Operand src2, SymBitVector a, SymBitVector b, SymBitVector c, SymState& ss) {
    SymFunction f("mincmp_single", 1, {32, 32});
    auto bb = b[31][0];
    auto cc = c[31][0];
    ss.set(dst, b[127][32] || (f(bb, cc)[0]).ite(bb, cc), true);
  });

  add_opcode_str({"maxsd"},
  [] (Operand dst, Operand src, SymBitVector a, SymBitVector b, SymState& ss) {
    SymFunction f("maxcmp_double", 1, {64, 64});
    auto aa = a[63][0];
    auto bb = b[63][0];
    ss.set(dst, a[127][64] || (f(aa, bb)[0]).ite(aa, bb));
  });

  add_opcode_str({"vmaxsd"},
  [] (Operand dst, Operand src1, Operand src2, SymBitVector a, SymBitVector b, SymBitVector c, SymState& ss) {
    SymFunction f("maxcmp_double", 1, {64, 64});
    auto bb = b[63][0];
    auto cc = c[63][0];
    ss.set(dst, b[127][64] || (f(bb, cc)[0]).ite(bb, cc), true);
  });

  add_opcode_str({"maxss"},
  [] (Operand dst, Operand src, SymBitVector a, SymBitVector b, SymState& ss) {
    SymFunction f("maxcmp_single", 1, {32, 32});
    auto aa = a[31][0];
    auto bb = b[31][0];
    ss.set(dst, a[127][32] || (f(aa, bb)[0]).ite(aa, bb));
  });

  add_opcode_str({"vmaxss"},
  [] (Operand dst, Operand src1, Operand src2, SymBitVector a, SymBitVector b, SymBitVector c, SymState& ss) {
    SymFunction f("maxcmp_single", 1, {32, 32});
    auto bb = b[31][0];
    auto cc = c[31][0];
    ss.set(dst, b[127][32] || (f(bb, cc)[0]).ite(bb, cc), true);
  });

  // can't be done with packed handler because of special case for memory
  add_opcode_str({"movsd"},
  [] (Operand dst, Operand src, SymBitVector a, SymBitVector b, SymState& ss) {
    if (src.is_sse_register()) {
      if (dst.is_typical_memory()) {
        ss.set(dst, b[63][0]);
      } else {
        ss.set(dst, a[127][64] || b[63][0]);
      }
    } else {
      auto zeros = SymBitVector::constant(128 - 64, 0);
      ss.set(dst, zeros || b[63][0]);
    }
  });

  // can't be done with packed handler because of special case for memory
  add_opcode_str({"vmovsd"},
  [] (Operand dst, Operand src, SymBitVector a, SymBitVector b, SymState& ss) {
    assert(src.is_typical_memory() || dst.is_typical_memory());
    if (src.is_typical_memory()) {
      // memory to register
      auto zeros = SymBitVector::constant(128 - 64, 0);
      ss.set(dst, zeros || b[63][0], true);
    } else {
      // register to memory
      ss.set(dst, b[63][0]);
    }
  });

  // can't be done with packed handler because of special case for memory
  add_opcode_str({"vmovsd"},
  [] (Operand dst, Operand src1, Operand src2, SymBitVector a, SymBitVector b, SymBitVector c, SymState& ss) {
    assert(src1.is_sse_register() && src2.is_sse_register() && dst.is_sse_register());
    ss.set(dst, b[127][64] || c[63][0], true);
  });

  // can't be done with packed handler because of special case for memory
  add_opcode_str({"movss"},
  [] (Operand dst, Operand src, SymBitVector a, SymBitVector b, SymState& ss) {
    if (src.is_sse_register()) {
      if (dst.is_typical_memory()) {
        ss.set(dst, b[31][0]);
      } else {
        ss.set(dst, a[127][32] || b[31][0]);
      }
    } else {
      auto zeros = SymBitVector::constant(128 - 32, 0);
      ss.set(dst, zeros || b[31][0]);
    }
  });

  // can't be done with packed handler because of special case for memory
  add_opcode_str({"vmovss"},
  [] (Operand dst, Operand src, SymBitVector a, SymBitVector b, SymState& ss) {
    assert(src.is_typical_memory() || dst.is_typical_memory());
    if (src.is_typical_memory()) {
      // memory to register
      auto zeros = SymBitVector::constant(128 - 32, 0);
      ss.set(dst, zeros || b[31][0], true);
    } else {
      // register to memory
      ss.set(dst, b[31][0]);
    }
  });

  // can't be done with packed handler because of special case for memory
  add_opcode_str({"vmovss"},
  [] (Operand dst, Operand src1, Operand src2, SymBitVector a, SymBitVector b, SymBitVector c, SymState& ss) {
    assert(src1.is_sse_register() && src2.is_sse_register() && dst.is_sse_register());
    ss.set(dst, b[127][32] || c[31][0], true);
  });


  add_opcode_str({"negb", "negw", "negl", "negq"},
  [] (Operand dst, SymBitVector a, SymState& ss) {
    ss.set(dst, -a);
    ss.set(eflags_cf, a != SymBitVector::constant(dst.size(), 0));
    ss.set(eflags_of, a[dst.size()-1] & (-a)[dst.size()-1]);
    ss.set(eflags_af, a[3] & (-a)[3]);
    ss.set_szp_flags(-a);
  });

  add_opcode_str({"nop", "nopb", "nopw", "nopl", "nopq"},
  [] (SymState& ss) {});

  add_opcode_str({"notb", "notw", "notl", "notq"},
  [] (Operand dst, SymBitVector a, SymState& ss) {
    ss.set(dst, !a);
  });

  add_opcode_str({"orb", "orw", "orl", "orq"},
  [this] (Operand dst, Operand src, SymBitVector a, SymBitVector b, SymState& ss) {
    if (src.size() < dst.size())
      b = b.extend(dst.size());

    ss.set(dst, a | b);
    ss.set(eflags_cf, SymBool::_false());
    ss.set(eflags_of, SymBool::_false());
    ss.set(eflags_af, SymBool::tmp_var());
    ss.set_szp_flags(a | b);
  });

  add_opcode_str({"popq"},
  [this] (Operand dst, SymBitVector a, SymState& ss) {
    M64 target = M64(rsp);
    ss.set(dst, ss[target]);
    if (dst != rsp) {
      ss.set(rsp, ss[rsp] + SymBitVector::constant(64, 8));
    }
  });

  add_opcode_str({"popw"},
  [this] (Operand dst, SymBitVector a, SymState& ss) {
    M16 target = M16(rsp);
    ss.set(dst, ss[target]);
    if (dst != sp) {
      ss.set(rsp, ss[rsp] + SymBitVector::constant(64, 2));
    }
  });

  add_opcode_str({"popcntw", "popcntl", "popcntq"},
  [this] (Operand dst, Operand src, SymBitVector a, SymBitVector b, SymState& ss) {

    std::function<SymBitVector (SymBitVector, uint16_t)> helper =
    [&] (SymBitVector src, uint16_t size) {
      if (size == 1) {
        return src;
      } else {
        uint16_t half = size/2;
        SymBitVector zeros = SymBitVector::constant(half, 0);
        SymBitVector left = src[size-1][half];
        SymBitVector right = src[half-1][0];
        return (zeros || helper(left, half)) + (zeros || helper(right, half));
      }
    };

    uint16_t size = dst.size();

    ss.set(dst, helper(b, size));
    ss.set(eflags_zf, b == SymBitVector::constant(size, 0));
    ss.set(eflags_cf, SymBool::_false());
    ss.set(eflags_pf, SymBool::_false());
    ss.set(eflags_sf, SymBool::_false());
    ss.set(eflags_of, SymBool::_false());
    ss.set(eflags_af, SymBool::_false());
  });

  add_opcode_str({"pshuflw"},
  [this] (Operand dst, Operand src, Operand i, SymBitVector a, SymBitVector b, SymBitVector imm, SymState &ss) {
    uint64_t constant = (static_cast<const SymBitVectorConstant*>(imm.ptr))->constant_;
    SymBitVector result;
    for (size_t i = 0; i < 4; ++i) {
      auto amt = SymBitVector::constant(128, ((constant & (0x3 << 2*i)) >> 2*i) << 4);
      result = (b >> amt)[15][0] || result;
    }
    result = b[127][64] || result;
    ss.set(dst, result);
  });

  add_opcode_str({"vpshuflw"},
  [this] (Operand dst, Operand src, Operand i, SymBitVector a, SymBitVector b, SymBitVector imm, SymState &ss) {
    uint64_t constant = (static_cast<const SymBitVectorConstant*>(imm.ptr))->constant_;
    SymBitVector result;
    for (size_t i = 0; i < 4; ++i) {
      auto amt = SymBitVector::constant(dst.size(), ((constant & (0x3 << 2*i)) >> 2*i) << 4);
      result = (b >> amt)[15][0] || result;
    }
    result = b[127][64] || result;
    if (dst.size() == 256) {
      for (size_t i = 0; i < 4; ++i) {
        auto amt = SymBitVector::constant(dst.size(), ((constant & (0x3 << 2*i)) >> 2*i) << 4);
        result = (b >> amt)[143][128] || result;
      }
      result = b[255][192] || result;
    }
    ss.set(dst, result, true);
  });

  add_opcode_str({"pushq"},
  [this] (Operand dst, SymBitVector a, SymState& ss) {
    ss.set(rsp, ss[rsp] - SymBitVector::constant(64, 8));
    M64 target = M64(rsp);
    ss.set(target, a.extend(64));
  });

  add_opcode_str({"pushl"},
  [this] (Operand dst, SymBitVector a, SymState& ss) {
    ss.set(rsp, ss[rsp] - SymBitVector::constant(64, 4));
    M32 target = M32(rsp);
    ss.set(target, a.extend(32));
  });

  add_opcode_str({"pushw"},
  [this] (Operand dst, SymBitVector a, SymState& ss) {
    ss.set(rsp, ss[rsp] - SymBitVector::constant(64, 2));
    M16 target = M16(rsp);
    ss.set(target, a.extend(16));
  });

  add_opcode_str({"shufpd"},
                 [this] (Operand dst, Operand src, Operand ctl,
  SymBitVector arg1, SymBitVector arg2, SymBitVector imm, SymState& ss) {

    SymBitVector output;
    output = (imm[0]).ite(arg1[127][64], arg1[63][0]);
    output = (imm[1]).ite(arg2[127][64], arg2[63][0]) || output;
    ss.set(dst, output);
  });

  add_opcode_str({"vshufpd"},
                 [this] (Operand dst, Operand src1, Operand src2, Operand ctl,
                         SymBitVector ignore, SymBitVector arg1, SymBitVector arg2, SymBitVector imm,
  SymState& ss) {

    SymBitVector output;
    output = (imm[0]).ite(arg1[127][64], arg1[63][0]);
    output = (imm[1]).ite(arg2[127][64], arg2[63][0]) || output;

    if (dst.size() == 256) {
      output = (imm[2]).ite(arg1[255][192], arg1[191][128]) || output;
      output = (imm[3]).ite(arg2[255][192], arg2[191][128]) || output;
    }

    ss.set(dst, output, true);
  });

  add_opcode_str({"shufps"},
                 [this] (Operand dst, Operand src, Operand ctl,
  SymBitVector arg1, SymBitVector arg2, SymBitVector imm, SymState& ss) {

    SymBitVector output;
    for (size_t i = 0; i < 4; ++i) {
      SymBitVector target = (i < 2 ? arg1 : arg2);
      output = imm[2*i].ite(
                 imm[2*i + 1].ite(target[127][96], target[63][32]),
                 imm[2*i + 1].ite(target[95][64],  target[31][0])) || output;
    }
    ss.set(dst, output);

  });

  add_opcode_str({"vshufps"},
                 [this] (Operand dst, Operand src, Operand src2, Operand ctl,
                         SymBitVector ignore, SymBitVector arg1, SymBitVector arg2, SymBitVector imm,
  SymState& ss) {

    SymBitVector output;
    for (size_t i = 0; i < 4; ++i) {
      SymBitVector target = (i < 2 ? arg1 : arg2);
      output = imm[2*i].ite(
                 imm[2*i + 1].ite(target[127][96], target[63][32]),
                 imm[2*i + 1].ite(target[95][64],  target[31][0])) || output;
    }

    if (dst.size() == 256) {

      for (size_t i = 0; i < 4; ++i) {
        SymBitVector target = (i < 2 ? arg1 : arg2);
        output = imm[2*i].ite(
                   imm[2*i + 1].ite(target[255][224], target[191][160]),
                   imm[2*i + 1].ite(target[223][192],  target[159][128])) || output;
      }

    }

    ss.set(dst, output, true);
  });

  add_opcode_str({"testb", "testw", "testl", "testq"},
  [this] (Operand dst, Operand src, SymBitVector a, SymBitVector b, SymState& ss) {
    if (src.size() < dst.size())
      b = b.extend(dst.size());

    ss.set(eflags_cf, SymBool::_false());
    ss.set(eflags_of, SymBool::_false());
    ss.set(eflags_af, SymBool::tmp_var());
    ss.set_szp_flags(a & b);
  });

  add_opcode_str({"vbroadcastf128"},
  [this] (Operand dst, Operand src, SymBitVector a, SymBitVector b, SymState& ss) {
    uint16_t size = 128;
    SymBitVector output = b[size-1][0];
    for (uint16_t i = size; i < dst.size(); i += size) {
      output = output || b[size-1][0];
    }
    ss.set(dst, output, true);
  });

  add_opcode_str({"vbroadcastsd"},
  [this] (Operand dst, Operand src, SymBitVector a, SymBitVector b, SymState& ss) {
    uint16_t size = 64;
    SymBitVector output = b[size-1][0];
    for (uint16_t i = size; i < dst.size(); i += size) {
      output = output || b[size-1][0];
    }
    ss.set(dst, output, true);
  });

  add_opcode_str({"vbroadcastss"},
  [this] (Operand dst, Operand src, SymBitVector a, SymBitVector b, SymState& ss) {
    uint16_t size = 32;
    SymBitVector output = b[size-1][0];
    for (uint16_t i = size; i < dst.size(); i += size) {
      output = output || b[size-1][0];
    }
    ss.set(dst, output, true);
  });


  add_opcode_str({"xchgb", "xchgw", "xchgl", "xchgq"},
  [this] (Operand dst, Operand src, SymBitVector a, SymBitVector b, SymState& ss) {
    ss.set(dst, b);
    ss.set(src, a);
  });

  add_opcode_str({"xorb", "xorw", "xorl", "xorq"},
  [this] (Operand dst, Operand src, SymBitVector a, SymBitVector b, SymState& ss) {
    if (src.size() < dst.size())
      b = b.extend(dst.size());

    ss.set(dst, a ^ b);
    ss.set(eflags_cf, SymBool::_false());
    ss.set(eflags_of, SymBool::_false());
    ss.set(eflags_af, SymBool::tmp_var());
    ss.set_szp_flags(a ^ b);
  });

  add_opcode_str({"vzeroall"},
  [this] (SymState& ss) {
    for (auto ymm : Constants::ymms()) {
      ss.set(ymm, SymBitVector::constant(256, 0));
    }
  });

  add_opcode_str({"vzeroupper"},
  [this] (SymState& ss) {
    size_t i = 0;
    for (auto ymm : Constants::ymms()) {
      ss.set(ymm, SymBitVector::constant(128, 0) || ss[Constants::xmms()[i]]);
      i += 1;
    }
  });


  add_opcode({VCVTPD2DQ_XMM_YMM},
  [this] (Operand dst, Operand src1, SymBitVector a, SymBitVector b, SymState& ss) {
    SymFunction f("cvt_double_to_int32", 32, {64});
    ss.set(dst, vectorize(f, a, b, a), true);
  });

  add_opcode({VCVTPD2PS_XMM_YMM},
  [this] (Operand dst, Operand src1, SymBitVector a, SymBitVector b, SymState& ss) {
    SymFunction f("cvt_double_to_single", 32, {64});
    ss.set(dst, vectorize(f, a, b, a), true);
  });

  add_opcode({VCVTTPD2DQ_XMM_YMM},
  [this] (Operand dst, Operand src1, SymBitVector a, SymBitVector b, SymState& ss) {
    SymFunction f("cvt_double_to_int32_truncate", 32, {64});
    ss.set(dst, vectorize(f, a, b, a), true);
  });


  add_opcode({VFMADD132PS_YMM_YMM_YMM},
  [this] (Operand dst, Operand src1, Operand src2, SymBitVector a, SymBitVector b, SymBitVector c, SymState& ss) {
    SymFunction f("vfmadd132_single", 32, {32, 32, 32});
    ss.set(dst, vectorize(f, a, b, c), true);
  });

  add_opcode({VFMADD132PD_YMM_YMM_YMM},
  [this] (Operand dst, Operand src1, Operand src2, SymBitVector a, SymBitVector b, SymBitVector c, SymState& ss) {
    SymFunction f("vfmadd132_double", 64, {64, 64, 64});
    ss.set(dst, vectorize(f, a, b, c), true);
  });

  add_opcode({VFMSUB132PS_YMM_YMM_YMM},
  [this] (Operand dst, Operand src1, Operand src2, SymBitVector a, SymBitVector b, SymBitVector c, SymState& ss) {
    SymFunction f("vfmsub132_single", 32, {32, 32, 32});
    ss.set(dst, vectorize(f, a, b, c), true);
  });

  add_opcode({VFMSUB132PD_YMM_YMM_YMM},
  [this] (Operand dst, Operand src1, Operand src2, SymBitVector a, SymBitVector b, SymBitVector c, SymState& ss) {
    SymFunction f("vfmsub132_double", 64, {64, 64, 64});
    ss.set(dst, vectorize(f, a, b, c), true);
  });

  add_opcode({VFNMADD132PS_YMM_YMM_YMM},
  [this] (Operand dst, Operand src1, Operand src2, SymBitVector a, SymBitVector b, SymBitVector c, SymState& ss) {
    SymFunction f("vfnmadd132_single", 32, {32, 32, 32});
    ss.set(dst, vectorize(f, a, b, c), true);
  });

  add_opcode({VFNMADD132PD_YMM_YMM_YMM},
  [this] (Operand dst, Operand src1, Operand src2, SymBitVector a, SymBitVector b, SymBitVector c, SymState& ss) {
    SymFunction f("vfnmadd132_double", 64, {64, 64, 64});
    ss.set(dst, vectorize(f, a, b, c), true);
  });

  add_opcode({VFNMSUB132PS_YMM_YMM_YMM},
  [this] (Operand dst, Operand src1, Operand src2, SymBitVector a, SymBitVector b, SymBitVector c, SymState& ss) {
    SymFunction f("vfnmsub132_single", 32, {32, 32, 32});
    ss.set(dst, vectorize(f, a, b, c), true);
  });

  add_opcode({VFNMSUB132PD_YMM_YMM_YMM},
  [this] (Operand dst, Operand src1, Operand src2, SymBitVector a, SymBitVector b, SymBitVector c, SymState& ss) {
    SymFunction f("vnfmsub132_double", 64, {64, 64, 64});
    ss.set(dst, vectorize(f, a, b, c), true);
  });


  add_opcode_str({"vfmadd132ss"},
  [this] (Operand dst, Operand src1, Operand src2, SymBitVector a, SymBitVector b, SymBitVector c, SymState& ss) {
    SymFunction f("vfmadd132_single", 32, {32, 32, 32});
    auto res = f(a[31][0], b[31][0], c[31][0]);
    ss.set(dst, SymBitVector::constant(128, 0) || ss[dst][127][32] || res, true);
  });

  add_opcode_str({"vfmadd132sd"},
  [this] (Operand dst, Operand src1, Operand src2, SymBitVector a, SymBitVector b, SymBitVector c, SymState& ss) {
    SymFunction f("vfmadd132_double", 64, {64, 64, 64});
    auto res = f(a[63][0], b[63][0], c[63][0]);
    ss.set(dst, SymBitVector::constant(128, 0) || ss[dst][127][64] || res, true);
  });

  add_opcode_str({"vfmsub132ss"},
  [this] (Operand dst, Operand src1, Operand src2, SymBitVector a, SymBitVector b, SymBitVector c, SymState& ss) {
    SymFunction f("vfmsub132_single", 32, {32, 32, 32});
    auto res = f(a[31][0], b[31][0], c[31][0]);
    ss.set(dst, SymBitVector::constant(128, 0) || ss[dst][127][32] || res, true);
  });

  add_opcode_str({"vfmsub132sd"},
  [this] (Operand dst, Operand src1, Operand src2, SymBitVector a, SymBitVector b, SymBitVector c, SymState& ss) {
    SymFunction f("vfmsub132_double", 64, {64, 64, 64});
    auto res = f(a[63][0], b[63][0], c[63][0]);
    ss.set(dst, SymBitVector::constant(128, 0) || ss[dst][127][64] || res, true);
  });

  add_opcode_str({"vfnmadd132ss"},
  [this] (Operand dst, Operand src1, Operand src2, SymBitVector a, SymBitVector b, SymBitVector c, SymState& ss) {
    SymFunction f("vfnmadd132_single", 32, {32, 32, 32});
    auto res = f(a[31][0], b[31][0], c[31][0]);
    ss.set(dst, SymBitVector::constant(128, 0) || ss[dst][127][32] || res, true);
  });

  add_opcode_str({"vfnmadd132sd"},
  [this] (Operand dst, Operand src1, Operand src2, SymBitVector a, SymBitVector b, SymBitVector c, SymState& ss) {
    SymFunction f("vfnmadd132_double", 64, {64, 64, 64});
    auto res = f(a[63][0], b[63][0], c[63][0]);
    ss.set(dst, SymBitVector::constant(128, 0) || ss[dst][127][64] || res, true);
  });

  add_opcode_str({"vfnmsub132ss"},
  [this] (Operand dst, Operand src1, Operand src2, SymBitVector a, SymBitVector b, SymBitVector c, SymState& ss) {
    SymFunction f("vfnmsub132_single", 32, {32, 32, 32});
    auto res = f(a[31][0], b[31][0], c[31][0]);
    ss.set(dst, SymBitVector::constant(128, 0) || ss[dst][127][32] || res, true);
  });

  add_opcode_str({"vfnmsub132sd"},
  [this] (Operand dst, Operand src1, Operand src2, SymBitVector a, SymBitVector b, SymBitVector c, SymState& ss) {
    SymFunction f("vnfmsub132_double", 64, {64, 64, 64});
    auto res = f(a[63][0], b[63][0], c[63][0]);
    ss.set(dst, SymBitVector::constant(128, 0) || ss[dst][127][64] || res, true);
  });

}

Handler::SupportLevel SimpleHandler::get_support(const x64asm::Instruction& instr) {

  if (!operands_supported(instr)) {
    return Handler::NONE;
  }

  auto opcode = instr.get_opcode();

  switch (instr.arity()) {
  case 0:
    if (!operator_0_.count(opcode))
      return Handler::NONE;
    break;
  case 1:
    if (!operator_1_.count(opcode))
      return Handler::NONE;
    break;
  case 2:
    if (!operator_2_.count(opcode))
      return Handler::NONE;
    break;
  case 3:
    if (!operator_3_.count(opcode))
      return Handler::NONE;
    break;
  case 4:
    if (!operator_4_.count(opcode))
      return Handler::NONE;
    break;
  default:
    return Handler::NONE;
  }

  for (size_t i = 0; i < instr.arity(); ++i) {
    Operand o = instr.get_operand<Operand>(i);
    if (!o.is_gp_register() && !o.is_typical_memory() &&
        !o.is_sse_register() && !o.is_immediate())
      return Handler::NONE;
  }

  return (Handler::SupportLevel)(Handler::BASIC | Handler::CEG | Handler::ANALYSIS);

}

void SimpleHandler::build_circuit(const x64asm::Instruction& instr, SymState& state) {

  auto opcode = instr.get_opcode();

  error_ = "";
  if (!get_support(instr)) {
    error_ = "No support for this instruction.";
    return;
  }

  // BRC -- This is a hack to deal with an intel manual bug.  xchg %eax, %eax
  // is a nop for some encodings, but with other encodings, it will zero bits
  // 32..63 of rax.  The right way to solve this would be to setup handlers by
  // opcode *value* rather than memonic, but that seems like a lot to change
  // for this one bug.
  if ((instr.get_opcode() == XCHG_EAX_R32 || instr.get_opcode() == XCHG_R32_EAX) &&
      instr.get_operand<R32>(0) == eax && instr.get_operand<R32>(1) == eax) {
    return;
  }

  // This is a hack to deal with all no-op cases.  We don't want to enumerate
  // all the opcodes and so forth just to find we've missed one.
  if (instr.is_nop()) {
    return;
  }

  // Figure out the right arguments
  size_t arity = instr.arity();
  switch (arity) {
  case 0: {
    auto f = operator_0_.at(opcode);
    f(state);
    break;
  }

  case 1: {
    auto f = operator_1_.at(opcode);
    Operand src = instr.get_operand<Operand>(0);
    SymBitVector value = state[src];
    f(src, value, state);
    break;
  }

  case 2: {
    auto f = operator_2_.at(opcode);
    Operand o1 = instr.get_operand<Operand>(0);
    Operand o2 = instr.get_operand<Operand>(1);
    SymBitVector v1 = state[o1];
    SymBitVector v2 = state[o2];
    f(o1, o2, v1, v2, state);
    break;
  }

  case 3: {
    auto f = operator_3_.at(opcode);
    Operand o1 = instr.get_operand<Operand>(0);
    Operand o2 = instr.get_operand<Operand>(1);
    Operand o3 = instr.get_operand<Operand>(2);
    SymBitVector v1 = state[o1];
    SymBitVector v2 = state[o2];
    SymBitVector v3 = state[o3];
    f(o1, o2, o3, v1, v2, v3, state);
    break;
  }

  case 4: {
    auto f = operator_4_.at(opcode);
    Operand o1 = instr.get_operand<Operand>(0);
    Operand o2 = instr.get_operand<Operand>(1);
    Operand o3 = instr.get_operand<Operand>(2);
    Operand o4 = instr.get_operand<Operand>(3);
    SymBitVector v1 = state[o1];
    SymBitVector v2 = state[o2];
    SymBitVector v3 = state[o3];
    SymBitVector v4 = state[o4];
    f(o1, o2, o3, o4, v1, v2, v3, v4, state);
    break;
  }

  default: {
    error_ = "Simple handler only support 0, 1, 2, 3 or 4 operands.";
    break;
  }

  }

}

void SimpleHandler::add_opcode(vector<Opcode> opcodes, ConstantOperator op) {
  for (auto it : opcodes) {
    operator_0_[it] = op;
  }
}
void SimpleHandler::add_opcode(vector<Opcode> opcodes, UnaryOperator op) {
  for (auto it : opcodes) {
    operator_1_[it] = op;
  }
}
void SimpleHandler::add_opcode(vector<Opcode> opcodes, BinaryOperator op) {
  for (auto it : opcodes) {
    operator_2_[it] = op;
  }
}
void SimpleHandler::add_opcode(vector<Opcode> opcodes, TrinaryOperator op) {
  for (auto it : opcodes) {
    operator_3_[it] = op;
  }
}
void SimpleHandler::add_opcode(vector<Opcode> opcodes, QuadOperator op) {
  for (auto it : opcodes) {
    operator_4_[it] = op;
  }
}

void SimpleHandler::add_opcode_str(vector<string> opcodes, ConstantOperator op) {
  for (auto it2 : Handler::opcodes_convert(opcodes)) {
    operator_0_[it2] = op;
  }
}
void SimpleHandler::add_opcode_str(vector<string> opcodes, UnaryOperator op) {
  for (auto it2 : Handler::opcodes_convert(opcodes)) {
    operator_1_[it2] = op;
  }
}
void SimpleHandler::add_opcode_str(vector<string> opcodes, BinaryOperator op) {
  for (auto it2 : Handler::opcodes_convert(opcodes)) {
    operator_2_[it2] = op;
  }
}
void SimpleHandler::add_opcode_str(vector<string> opcodes, TrinaryOperator op) {
  for (auto it2 : Handler::opcodes_convert(opcodes)) {
    operator_3_[it2] = op;
  }
}
void SimpleHandler::add_opcode_str(vector<string> opcodes, QuadOperator op) {
  for (auto it2 : Handler::opcodes_convert(opcodes)) {
    operator_4_[it2] = op;
  }
}

