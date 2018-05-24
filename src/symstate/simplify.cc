// Copyright 2013-2016 Stanford University
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "src/symstate/transform_visitor.h"
#include "src/symstate/simplify.h"

using namespace std;

namespace stoke {

namespace {

/**
 * Merges two consecutive extracts.
 * Handles bit extracts of constants.
 * Handles "full" bit extracts, where all bits are used.
 * Handles selection immediately followed by concatenation.
 *
 * E.g., b[12:10][1:0] becomes b[11:10].
 * E.g., 0x0[1:0] becomes 0x0
 * E.g., f_returns_32_bits(0x0)[31:0] becomes f_returns_32_bits(0x0)
 * E.g., a[63:32] || a[31:0] becomes a[63:0]
 */
class SymMergeExtracts : public SymTransformVisitor {

public:

  SymMergeExtracts(map<SymBoolAbstract*, SymBoolAbstract*>& cache_bool, map<SymBitVectorAbstract*, SymBitVectorAbstract*>& cache_bits, map<SymArrayAbstract*, SymArrayAbstract*>& cache_array) : SymTransformVisitor(cache_bool, cache_bits, cache_array) {}

  SymBitVectorAbstract* visit(const SymBitVectorExtract * const bv) {
    if (is_cached(bv)) return get_cached(bv);
    auto lhs = (*this)(bv->bv_);
    if (lhs->width_ == bv->width_) return cache(bv, lhs);
    if (lhs->type() == SymBitVector::EXTRACT) {
      auto inner = static_cast<const SymBitVectorExtract * const>(lhs);
      auto low = bv->low_bit_ + inner->low_bit_;
      auto high = low + (bv->high_bit_ - bv->low_bit_);
      return cache(bv, make_bitvector_extract(inner->bv_, high, low));
    }
    if (lhs == bv->bv_) {
      return cache(bv, (SymBitVectorExtract*)bv);
    }
    return cache(bv, make_bitvector_extract(lhs, bv->high_bit_, bv->low_bit_));
  }

  SymBitVectorAbstract* visit(const SymBitVectorConcat * const bv) {
    if (is_cached(bv)) return get_cached(bv);
    auto lhs = (*this)(bv->a_);
    auto rhs = (*this)(bv->b_);

    // std::cout << "\n\nSymBitVectorConcat simplify:\n" << SymBitVector(bv) << "\n";

    if (lhs->type() == SymBitVector::EXTRACT && rhs->type() == SymBitVector::EXTRACT) {
      auto l = static_cast<const SymBitVectorExtract * const>(lhs);
      auto r = static_cast<const SymBitVectorExtract * const>(rhs);
      if (l->bv_->equals(r->bv_) && l->low_bit_ == r->high_bit_+1) {
        return cache(bv, make_bitvector_extract(l->bv_, l->high_bit_, r->low_bit_));
      }
    }

    /*
    ** Concatenation of non-consecutive extracts.
    */
    // E.g., a[63:32] || (a[31:0] || X) becomes (a[63:0] || X)
    if (lhs->type() == SymBitVector::EXTRACT && rhs->type() ==
        SymBitVector::CONCAT) {
      auto l = static_cast<const SymBitVectorExtract * const>(lhs);
      auto r = static_cast<const SymBitVectorConcat * const>(rhs);

      auto r_lhs = (*this)(r->a_);
      auto r_rhs = (*this)(r->b_);

      if (r_lhs->type() == SymBitVector::EXTRACT) {
        auto r_lhs_extract = static_cast<const SymBitVectorExtract * const>(r_lhs);
        if (l->bv_->equals(r_lhs_extract->bv_) && l->low_bit_ == r_lhs_extract->high_bit_+1) {
          return cache(bv, make_binop(SymBitVector::CONCAT,
                                      make_bitvector_extract(l->bv_, l->high_bit_,
                                          r_lhs_extract->low_bit_), r_rhs));
        }
      }
    }
    if (lhs == bv->a_ && rhs == bv->b_) {
      return cache(bv, (SymBitVectorConcat*)bv);
    }
    return cache(bv, make_binop(bv->type(), lhs, rhs));
  }

};

/**
 * Moves bit extracts over other operators (where save).
 * Also moves bit extracts over concatenation, possibly throwing away one side.
 * Also moves bit extracts over sign-extends, possibly removing them all together
 *
 * E.g. (a | b)[5:2] becomes (a[5:2] | b[5:2])
 */
class SymMoveExtractsInside : public SymTransformVisitor {

public:

  SymMoveExtractsInside(map<SymBoolAbstract*, SymBoolAbstract*>& cache_bool, map<SymBitVectorAbstract*, SymBitVectorAbstract*>& cache_bits, map<SymArrayAbstract*, SymArrayAbstract*>& cache_array) : SymTransformVisitor(cache_bool, cache_bits, cache_array) {}

  SymBitVectorAbstract* visit(const SymBitVectorExtract * const bv) {
    if (is_cached(bv)) return get_cached(bv);
    auto inner = (*this)(bv->bv_);
    switch (inner->type()) {
    case SymBitVector::AND:
    case SymBitVector::OR:
    case SymBitVector::XOR: {
      SymBitVectorBinop* binop = (SymBitVectorBinop*)inner;
      auto a = make_bitvector_extract(binop->a_, bv->high_bit_, bv->low_bit_);
      auto b = make_bitvector_extract(binop->b_, bv->high_bit_, bv->low_bit_);
      return cache(bv, make_binop(binop->type(), a, b));
    }
    case SymBitVector::CONCAT: {
      SymBitVectorBinop* binop = (SymBitVectorBinop*)inner;
      auto lhs = binop->a_;
      auto rhs = binop->b_;
      // bit extract is fully in the lhs of the concat
      if (bv->low_bit_ >= rhs->width_) {
        return cache(bv, make_bitvector_extract(lhs, bv->high_bit_ - rhs->width_, bv->low_bit_ - rhs->width_));
      }
      // bit extract is fully in the rhs of the concat
      if (bv->high_bit_ < rhs->width_) {
        return cache(bv, make_bitvector_extract(rhs, bv->high_bit_, bv->low_bit_));
      }
      // there is overlap
      auto a = make_bitvector_extract(lhs, bv->high_bit_ - rhs->width_, 0);
      auto b = make_bitvector_extract(rhs, rhs->width_ - 1, bv->low_bit_);
      return cache(bv, make_binop(SymBitVector::CONCAT, a, b));
    }
    case SymBitVector::SIGN_EXTEND: {
      SymBitVectorSignExtend* sext = (SymBitVectorSignExtend*)inner;
      auto lhs = sext->bv_;
      if (bv->high_bit_ < lhs->width_) {
        return cache(bv, make_bitvector_extract(lhs, bv->high_bit_, bv->low_bit_));
      }
      // TODO: there are some more cases that could be handled
      break;
    }

    /*
    ** Distribute Extract over ITE
    */
    case SymBitVector::ITE: {
      SymBitVectorIte* ite = (SymBitVectorIte*)inner;
      auto a = make_bitvector_extract(ite->a_, bv->high_bit_, bv->low_bit_);
      auto b = make_bitvector_extract(ite->b_, bv->high_bit_, bv->low_bit_);
      return cache(bv, make_bitvector_ite(ite->cond_, a, b));
    }
    case SymBitVector::NOT: {
      SymBitVectorNot* n = (SymBitVectorNot*)inner;
      auto a = make_bitvector_extract(n->bv_, bv->high_bit_, bv->low_bit_);
      return cache(bv, make_unop(n->type(), a));
    }
    default:
      break;
    }
    if (inner == bv->bv_) {
      return cache(bv, (SymBitVectorExtract*)bv);
    }
    return cache(bv, make_bitvector_extract(inner, bv->high_bit_, bv->low_bit_));
  }

};

/**
 * Constant propagation.
 */
class SymConstProp : public SymTransformVisitor {

public:

  SymConstProp(map<SymBoolAbstract*, SymBoolAbstract*>& cache_bool, map<SymBitVectorAbstract*, SymBitVectorAbstract*>& cache_bits, map<SymArrayAbstract*, SymArrayAbstract*>& cache_array) : SymTransformVisitor(cache_bool, cache_bits, cache_array) {}

  SymBitVectorAbstract* visit(const SymBitVectorFunction * const bv) {
    if (is_cached(bv)) return get_cached(bv);

    // add/subtract of 0
    // multiply of 0
    auto& f = bv->f_;
    if (f.args.size() == 2) {
      auto a = (*this)(bv->args_[0]);
      auto b = (*this)(bv->args_[1]);

      //This seems wrong as
      // -0.0 + 0.0 = 0.0
      // -0.0 - 0.0 = 0.0
      /*
      if ((f.name == "sub_single" || f.name == "sub_double") && is_zero(b)) {
        return cache(bv, (SymBitVectorAbstract*) a);
      }
      if ((f.name == "add_single" || f.name == "add_double") && is_zero(b)) {
        return cache(bv, (SymBitVectorAbstract*) a);
      }
      if ((f.name == "add_single" || f.name == "add_double") && is_zero(a)) {
        return cache(bv, (SymBitVectorAbstract*) b);
      }
      */
      if ((f.name == "sub_single" || f.name == "sub_double" || f.name == "add_single" || f.name == "add_double") && is_zero(a) && is_zero(b)) {
        return cache(bv, make_constant(bv->width_, 0));
      }

      // Strata Bug
      //if ((f.name == "sub_single" || f.name == "sub_double") && a->equals(b)) {
      //  return cache(bv, make_constant(bv->width_, 0));
      //}
      if ((f.name == "mul_single" || f.name == "mul_double") && (is_zero(a) && is_zero(b))) {
        return cache(bv, make_constant(bv->width_, 0));
      }


      //DSAND: +-( a_lhs || a_rhs, b  ) and b == 0 and a_lhs == 0 (means a is
      // NOT NaN)
      if ((f.name == "sub_single" || f.name == "sub_double" || f.name == "add_single" || f.name == "add_double") && is_zero(b)) {
        if(a->type() == SymBitVector::CONCAT) {
          auto concat = (SymBitVectorConcat*)(a);
          auto a_lhs = (SymBitVectorAbstract*)concat->a_;
          auto a_rhs = (SymBitVectorAbstract*)concat->b_;

          if(is_zero(a_lhs)) {
            return cache(bv, (SymBitVectorAbstract*) a);  
          }
        }
      }

      //DSAND: +(a,  b_lhs || b_rhs) and a == 0 and b_lhs == 0 (means b is
      // NOT NaN)
      if ((f.name == "add_single" || f.name == "add_double") && is_zero(a)) {
        if(b->type() == SymBitVector::CONCAT) {
          auto concat = (SymBitVectorConcat*)(b);
          auto b_lhs = (SymBitVectorAbstract*)concat->a_;
          auto b_rhs = (SymBitVectorAbstract*)concat->b_;

          if(is_zero(b_lhs)) {
            return cache(bv, (SymBitVectorAbstract*) b);  
          }
        }
      }
    }

    if (f.args.size() == 3) {
      auto a = (*this)(bv->args_[0]);
      auto b = (*this)(bv->args_[1]);
      auto c = (*this)(bv->args_[2]);
      if ((f.name == "vfmsub132_double" || f.name == "vfmsub132_single") && (is_zero(a) && is_zero(b) && is_zero(c))) {
        return cache(bv, make_constant(bv->width_, 0));
      }
    }
    // conversion of 0
    // sqrt of 0
    if (f.args.size() == 1) {
      auto a = (*this)(bv->args_[0]);
      if (is_zero(a)) {
        if (f.name == "sqrt_double" ||
            f.name == "sqrt_single" ||
            f.name == "cvt_int32_to_double" ||
            f.name == "cvt_int32_to_single" ||
            f.name == "cvt_double_to_int32" ||
            f.name == "cvt_double_to_single" ||
            f.name == "cvt_single_to_int32" ||
            f.name == "cvt_single_to_double" ||
            f.name == "cvt_double_to_int64" ||
            f.name == "cvt_int64_to_double" ||
            f.name == "cvt_int64_to_single" ||
            f.name == "cvt_single_to_int64" ||
            f.name == "cvt_double_to_int32_truncate" ||
            f.name == "cvt_single_to_int32_truncate" ||
            f.name == "cvt_double_to_int64_truncate" ||
            f.name == "cvt_single_to_int64_truncate") {
          return cache(bv, make_constant(bv->width_, 0));
        }
      }
    }

    return SymTransformVisitor::visit(bv);
  }

  SymBitVectorAbstract* visit(const SymBitVectorSignExtend * const bv) {
    if (is_cached(bv)) return get_cached(bv);
    auto inner = (*this)(bv->bv_);
    if (is_const(inner) && bv->size_ <= 64) {
      int64_t value = read_const(inner);
      // mask away upper bits
      int64_t newval = value & mask(inner->width_);
      // move up
      newval <<= 64 - inner->width_;
      // move back down, shifting in ones
      newval >>= 64 - inner->width_;
      return cache(bv, make_constant(bv->size_, newval));
    }
    if (inner == bv->bv_) {
      return cache(bv, (SymBitVectorExtract*)bv);
    }
    return cache(bv, make_bitvector_sign_extend(inner, bv->size_));
  }

  SymBitVectorAbstract* visit(const SymBitVectorExtract * const bv) {
    if (is_cached(bv)) return get_cached(bv);
    auto lhs = (*this)(bv->bv_);
    if (is_const(lhs)) {
      uint64_t val = read_const(lhs);
      auto newsize = bv->high_bit_ - bv->low_bit_ + 1;
      auto newconstant = val >> bv->low_bit_;
      return cache(bv, make_constant(newsize, newconstant));
    }
    if (lhs == bv->bv_) {
      return cache(bv, (SymBitVectorExtract*)bv);
    }
    return cache(bv, make_bitvector_extract(lhs, bv->high_bit_, bv->low_bit_));
  }

  SymBitVectorAbstract* visit_binop(const SymBitVectorBinop * const bv) {
    if (is_cached(bv)) return get_cached(bv);
    auto lhs = (*this)(bv->a_);
    auto rhs = (*this)(bv->b_);
    auto width = bv->width_;

    if (is_const(lhs) && is_const(rhs) && width <= 64) {
      uint64_t l = read_const(lhs);
      uint64_t r = read_const(rhs);
      int64_t ls = read_sconst(lhs);
      int64_t rs = read_sconst(rhs);
      switch (bv->type()) {
      case SymBitVector::AND:
        return cache(bv, make_constant(width, l & r));
      case SymBitVector::CONCAT:
        return cache(bv, make_constant(width, (l << rhs->width_) | r));
      case SymBitVector::DIV:
        break;
      case SymBitVector::MINUS:
        return cache(bv, make_constant(width, ls - rs));
      case SymBitVector::MOD:
        return cache(bv, make_constant(width, ls % rs));
      case SymBitVector::MULT:
        return cache(bv, make_constant(width, ls * rs));
      case SymBitVector::OR:
        return cache(bv, make_constant(width, l | r));
      case SymBitVector::PLUS:
        return cache(bv, make_constant(width, ls + rs));
      case SymBitVector::ROTATE_LEFT:
        break;
      case SymBitVector::ROTATE_RIGHT:
        break;
      case SymBitVector::SHIFT_RIGHT:
        break;
      case SymBitVector::SHIFT_LEFT:
        break;
      case SymBitVector::SIGN_DIV:
        break;
      case SymBitVector::SIGN_MOD:
        break;
      case SymBitVector::SIGN_SHIFT_RIGHT:
        break;
      case SymBitVector::XOR:
        return cache(bv, make_constant(width, l ^ r));
        break;
      default:
        break;
      }
    }

    // addition/subtraction of zero
    if (bv->type() == SymBitVector::PLUS && is_zero(lhs)) {
      return cache(bv, rhs);
    }
    if (bv->type() == SymBitVector::PLUS && is_zero(rhs)) {
      return cache(bv, lhs);
    }
    if (bv->type() == SymBitVector::MINUS && is_zero(rhs)) {
      return cache(bv, lhs);
    }
    /*
    if (bv->type() == SymBitVector::CONCAT && (0 == lhs->width_)) {
      return cache(bv, rhs);
    }
    if (bv->type() == SymBitVector::CONCAT && (0 == rhs->width_)) {
      return cache(bv, lhs);
    }
    */

    // DSAND: W1'0 || (W2'0 || X) ==> (W1+W2)'0 || X
    if (bv->type() == SymBitVector::CONCAT && is_zero(lhs) && rhs->type() ==
        SymBitVector::CONCAT) {
      auto r = (SymBitVectorConcat*)(rhs);
      auto rhs_a = (SymBitVectorAbstract*)r->a_;
      auto rhs_b = (SymBitVectorAbstract*)r->b_;
      if (is_zero(rhs_a)) {
        return cache(bv, make_binop(bv->type(), make_constant(lhs->width_ +
                                    rhs_a->width_, 0), rhs_b));
      }
    }

    // move binop over ite
    if (lhs->type() == SymBitVector::ITE) {
      SymBitVectorIte* ite = (SymBitVectorIte*)lhs;
      if (is_const(ite->a_) && is_const(ite->a_) && is_const(rhs)) {
        auto a = make_binop(bv->type(), (SymBitVectorAbstract*)ite->a_, rhs);
        auto b = make_binop(bv->type(), (SymBitVectorAbstract*)ite->b_, rhs);
        return cache(bv, make_bitvector_ite(ite->cond_, a, b));
      }
    }
    if (rhs->type() == SymBitVector::ITE) {
      SymBitVectorIte* ite = (SymBitVectorIte*)rhs;
      if (is_const(ite->a_) && is_const(ite->a_) && is_const(lhs)) {
        auto a = make_binop(bv->type(), lhs, (SymBitVectorAbstract*)ite->a_);
        auto b = make_binop(bv->type(), lhs, (SymBitVectorAbstract*)ite->b_);
        return cache(bv, make_bitvector_ite(ite->cond_, a, b));
      }
    }

    /* move concat over ite
    if(bv->type() == SymBitVector::CONCAT) {
        if (lhs->type() == SymBitVector::ITE) {
          SymBitVectorIte* ite = (SymBitVectorIte*)lhs;
          auto a = make_binop(bv->type(), (SymBitVectorAbstract*)ite->a_, rhs);
          auto b = make_binop(bv->type(), (SymBitVectorAbstract*)ite->b_, rhs);
          return cache(bv, make_bitvector_ite(ite->cond_, a, b));
        }

        if (rhs->type() == SymBitVector::ITE) {
          SymBitVectorIte* ite = (SymBitVectorIte*)rhs;
          auto a = make_binop(bv->type(), lhs, (SymBitVectorAbstract*)ite->a_);
          auto b = make_binop(bv->type(), lhs, (SymBitVectorAbstract*)ite->b_);
          return cache(bv, make_bitvector_ite(ite->cond_, a, b));
        }
    }
    */

    /*
    ** DSAND: Distribute add/xor/or/and over ITE
    ite(C, A1, B1) + ite(C, A2, B2)
    */
    if (bv->type() == SymBitVector::XOR || bv->type() == SymBitVector::OR ||
        bv->type() == SymBitVector::AND || bv->type() == SymBitVector::PLUS) {
      if (lhs->type() == SymBitVector::ITE && rhs->type() == SymBitVector::ITE) {
        SymBitVectorIte* l = (SymBitVectorIte*)lhs;
        SymBitVectorIte* r = (SymBitVectorIte*)rhs;

        if (l->cond_->equals(r->cond_)) {
          auto a = make_binop(bv->type(), (SymBitVectorAbstract*)l->a_, (SymBitVectorAbstract*)r->a_);
          auto b = make_binop(bv->type(), (SymBitVectorAbstract*)l->b_, (SymBitVectorAbstract*)r->b_);
          return cache(bv, make_bitvector_ite(l->cond_, a, b));
        }
      }
    }

    // a ^ a
    if (bv->type() == SymBitVector::XOR && lhs->equals(rhs)) {
      return cache(bv, make_constant(width, 0));
    }

    //DSAND:: a ^ 0 == 0 ^ a == a
    if (bv->type() == SymBitVector::XOR && is_zero(lhs)) {
      return cache(bv, rhs);
    }
    if (bv->type() == SymBitVector::XOR && is_zero(rhs)) {
      return cache(bv, lhs);
    }
    //DSAND:: a | 0 == 0 | a == a
    if (bv->type() == SymBitVector::OR && is_zero(lhs)) {
      return cache(bv, rhs);
    }
    if (bv->type() == SymBitVector::OR && is_zero(rhs)) {
      return cache(bv, lhs);
    }
    //DSAND:: a & 0 == 0 & a == a
    if (bv->type() == SymBitVector::AND && is_zero(lhs)) {
      return cache(bv, make_constant(lhs->width_, 0));
    }
    if (bv->type() == SymBitVector::AND && is_zero(rhs)) {
      return cache(bv, make_constant(rhs->width_, 0));
    }

    // a | a
    if (bv->type() == SymBitVector::OR && lhs->equals(rhs)) {
      return cache(bv, lhs);
    }

    // a & a
    if (bv->type() == SymBitVector::AND && lhs->equals(rhs)) {
      return cache(bv, lhs);
    }

    if (lhs == bv->a_ && rhs == bv->b_) {
      return cache(bv, (SymBitVectorBinop*)bv);
    }
    return cache(bv, make_binop(bv->type(), lhs, rhs));
  }

  SymBoolAbstract* visit_compare(const SymBoolCompare * const bv) {
    if (is_cached(bv)) return get_cached(bv);
    auto lhs = (*this)(bv->a_);
    auto rhs = (*this)(bv->b_);

    if (is_const(lhs) && is_const(rhs)) {
      uint64_t l = read_const(lhs);
      uint64_t r = read_const(rhs);
      int64_t ls = read_sconst(lhs);
      int64_t rs = read_sconst(rhs);
      switch (bv->type()) {
      case SymBool::EQ:
        return cache(bv, make_constant(l == r));
      case SymBool::GE:
        return cache(bv, make_constant(l >= r));
      case SymBool::GT:
        return cache(bv, make_constant(l > r));
      case SymBool::LE:
        return cache(bv, make_constant(l <= r));
      case SymBool::LT:
        return cache(bv, make_constant(l < r));
      case SymBool::SIGN_GE:
        return cache(bv, make_constant(ls >= rs));
      case SymBool::SIGN_GT:
        return cache(bv, make_constant(ls > rs));
      case SymBool::SIGN_LE:
        return cache(bv, make_constant(ls <= rs));
      case SymBool::SIGN_LT:
        return cache(bv, make_constant(ls < rs));
      default:
        break;
      }
    }

    /*
    ** Distribute eqMInt over ITE
    rule
    eqMInt
    (
    (#ifMInt B:Bool #then MIA:MInt #else MIB:MInt #fi),
    MIC:MInt
    )
    =>
    (#ifBool B:Bool
      #then eqMInt(MIA:MInt, MIC:MInt)
      #else eqMInt(MIB:MInt, MIC:MInt)
    #fi)

    if(bv->type() == SymBool::EQ) {
      auto lhs = (*this)(bv->a_);
      auto rhs = (*this)(bv->b_);

      if(lhs->type() == SymBitVector::ITE) {
        SymBitVectorIte* l = (SymBitVectorIte*)lhs;
        return cache(bv, make_bitvector_ite(l->cond_, make_compare(bv->type(),
                (SymBitVectorAbstract*)l->a_, rhs), make_compare(bv->type(), (SymBitVectorAbstract*)l->b_, rhs)));
      }
    }
    */



    if (lhs == bv->a_ && rhs == bv->b_) {
      return cache(bv, (SymBoolCompare*)bv);
    }
    return cache(bv, make_compare(bv->type(), lhs, rhs));
  }

  SymBoolAbstract* visit_binop(const SymBoolBinop * const bv) {
    if (is_cached(bv)) return get_cached(bv);
    auto lhs = (*this)(bv->a_);
    auto rhs = (*this)(bv->b_);

    if (is_const(lhs) && is_const(rhs)) {
      bool l = read_const(lhs);
      bool r = read_const(rhs);
      switch (bv->type()) {
      case SymBool::AND:
        return cache(bv, make_constant(l && r));
      case SymBool::IFF:
        return cache(bv, make_constant(l == r));
      case SymBool::IMPLIES:
        return cache(bv, make_constant(!l || r));
      case SymBool::OR:
        return cache(bv, make_constant(l || r));
      case SymBool::XOR:
        return cache(bv, make_constant(!(l == r)));
      default:
        break;
      }
    }

    if (lhs == bv->a_ && rhs == bv->b_) {
      return cache(bv, (SymBoolBinop*)bv);
    }
    return cache(bv, make_binop(bv->type(), lhs, rhs));
  }

  SymBoolAbstract* visit(const SymBoolNot * const b) {
    if (is_cached(b)) return get_cached(b);
    auto lhs = (*this)(b->b_);

    if (is_const(lhs)) {
      return cache(b, make_constant(!read_const(lhs)));
    }

    if (lhs == b->b_) {
      return cache(b, (SymBoolNot*)b);
    }
    return cache(b, make_bool_not(lhs));
  }

  SymBitVectorAbstract* visit_unop(const SymBitVectorUnop * const bv) {
    if (is_cached(bv)) return get_cached(bv);
    auto lhs = (*this)(bv->bv_);
    auto width = bv->width_;
    if (is_const(lhs) && width <= 64) {
      uint64_t l = read_const(lhs);
      int64_t ls = read_sconst(lhs);
      switch (bv->type()) {
      case SymBitVector::NOT:
        return cache(bv, make_constant(width, ~l));
      case SymBitVector::U_MINUS: {
        int64_t val = -ls;
        return cache(bv, make_constant(width, val));
      }
      default:
        break;
      }
    }

    if (lhs == bv->bv_) {
      return cache(bv, (SymBitVectorAbstract*)bv);
    }
    return cache(bv, make_unop(bv->type(), lhs));
  }

  SymBitVectorAbstract* visit(const SymBitVectorIte * const bv) {
    if (is_cached(bv)) return get_cached(bv);
    auto c = (*this)(bv->cond_);
    auto lhs = (*this)(bv->a_);
    auto rhs = (*this)(bv->b_);

    if (is_const(c)) {
      return cache(bv, read_const(c) ? lhs : rhs);
    }

    // Both then/else same
    if (lhs->equals(rhs)) {
      return cache(bv, lhs);
    }

    // DSAND: If(cf == mi(1, 1)) then mi(1, 1) else mi(1, 0) => cf
    // If(C1 == C2) then lhs else rhs
    switch (c->type()) {
    case SymBool::EQ: {
      SymBoolCompare* binop = (SymBoolCompare*)c;
      auto c1 = (*this)(binop->a_);
      auto c2 = (*this)(binop->b_);
      if ((1 == c1->width_) && is_one(c2) && ( 1 == lhs->width_ && is_one(lhs))
          && (1 == rhs->width_ && is_zero(rhs))) {
        return cache(bv, c1);
      }
    }
    default:
      break;
    }


    if (lhs == bv->a_ && rhs == bv->b_ && c == bv->cond_) {
      return cache(bv, (SymBitVectorIte*)bv);
    }
    SymBitVectorIte* res = NULL;
    return cache(bv, make_bitvector_ite(c, lhs, rhs));
  }

private:

  bool is_const(const SymBitVectorAbstract* const s) {
    return s->type() == SymBitVector::CONSTANT;
  }

  bool is_const(const SymBoolAbstract* const s) {
    return (s->type() == SymBool::FALSE) || (s->type() == SymBool::TRUE);
  }

  bool is_zero(const SymBitVectorAbstract* const b) {
    return is_const(b) && read_const(b) == 0;
  }

  bool is_one(const SymBitVectorAbstract* const b) {
    return is_const(b) && read_const(b) == 1;
  }

  /** Returns bit pattern consisting of 0s and ending with 'ones' many 1s. */
  uint64_t mask(uint16_t ones) {
    if (ones == 0) return 0;
    if (ones == 64) return -1;
    return (1ULL << ones) - 1;
  }

  /** Read an unsigned constant. */
  uint64_t read_const(const SymBitVectorAbstract* const s) {
    auto c = (SymBitVectorConstant*)s;
    return c->constant_ & mask(c->size_);
  }

  /** Read a signed constant. */
  int64_t read_sconst(const SymBitVectorAbstract* const s) {
    int64_t val = read_const(s);
    return (val << (64 - s->width_)) >> (64 - s->width_);
  }

  /** Read a bool constant. */
  uint64_t read_const(const SymBoolAbstract* const s) {
    return (s->type() == SymBool::TRUE);
  }

  /** Creates a new constant, masking out bits that are too high. */
  SymBitVectorConstant* make_constant(uint16_t size, uint64_t constant) {
    return make_bitvector_constant(size, constant & mask(size));
  }

  /** Creates a new constant. */
  SymBoolAbstract* make_constant(bool constant) {
    if (constant) {
      return make_bool_true();
    }
    return make_bool_false();
  }

};

} // namespace


SymBitVector SymSimplify::simplify(const SymBitVector& b) {
  auto ptr = b.ptr;
  //std::cout << "\n\nEnter simplify:\n" << SymBitVector(ptr) << "\n";

  SymMergeExtracts merger(cache_bool1_, cache_bits1_, cache_array1_);
  SymMoveExtractsInside mover(cache_bool2_, cache_bits2_, cache_array2_);
  SymConstProp constprop(cache_bool3_, cache_bits3_, cache_array3_);

  // apply transformations until no further simplifications are possible
  while (true) {
    auto old = ptr;
    ptr = mover(ptr);
    //std::cout << "Mover:\n" << SymBitVector(ptr) << "\n";
    ptr = merger(ptr);
    //std::cout << "Merger:\n" << SymBitVector(ptr) << "\n";
    ptr = constprop(ptr);
    //std::cout << "ConstProp:\n" << SymBitVector(ptr) << "\n";
    if (old == ptr) break;
  }

  //std::cout << "Retun simplify:\n" << SymBitVector(ptr) << "\n";
  return SymBitVector(ptr);
}

SymBool SymSimplify::simplify(const SymBool& b) {
  auto ptr = b.ptr;

  SymMergeExtracts merger(cache_bool1_, cache_bits1_, cache_array1_);
  SymMoveExtractsInside mover(cache_bool2_, cache_bits2_, cache_array2_);
  SymConstProp constprop(cache_bool3_, cache_bits3_, cache_array3_);

  // apply transformations until no further simplifications are possible
  while (true) {
    auto old = ptr;
    ptr = mover(ptr);
    ptr = merger(ptr);
    ptr = constprop(ptr);
    if (old == ptr) break;
  }

  return SymBool(ptr);
}

SymArray SymSimplify::simplify(const SymArray& b) {
  auto ptr = b.ptr;

  SymMergeExtracts merger(cache_bool1_, cache_bits1_, cache_array1_);
  SymMoveExtractsInside mover(cache_bool2_, cache_bits2_, cache_array2_);
  SymConstProp constprop(cache_bool3_, cache_bits3_, cache_array3_);

  // apply transformations until no further simplifications are possible
  while (true) {
    auto old = ptr;
    ptr = mover(ptr);
    ptr = merger(ptr);
    ptr = constprop(ptr);
    if (old == ptr) break;
  }

  return SymArray(ptr);
}

} // namespace stoke
