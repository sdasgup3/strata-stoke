#ifndef _STOKE_SRC_SYMSTATE_BW_VISITOR
#define _STOKE_SRC_SYMSTATE_BW_VISITOR

#include "src/symstate/visitor.h"

namespace stoke {

class BitWidthVisitor : public SymVisitor<uint16_t, uint16_t, uint16_t> {

public:
  BitWidthVisitor(std::ostream& os) : os_(os) {}

  uint16_t visit_binop(const SymBitVectorBinop * const bv) {

    switch (bv->type()) {
    case SymBitVector::AND:
    case SymBitVector::DIV:
    case SymBitVector::MINUS:
    case SymBitVector::MOD:
    case SymBitVector::MULT:
    case SymBitVector::OR:
    case SymBitVector::PLUS:
    case SymBitVector::ROTATE_LEFT:
    case SymBitVector::ROTATE_RIGHT:
    case SymBitVector::SHIFT_LEFT:
    case SymBitVector::SHIFT_RIGHT:
    case SymBitVector::SIGN_DIV:
    case SymBitVector::SIGN_MOD:
    case SymBitVector::SIGN_SHIFT_RIGHT:
    case SymBitVector::XOR:
      return (*this)(bv->a_);
      break;
    case SymBitVector::CONCAT:
      return (*this)(bv->a_) + (*this)(bv->b_);
      break;
    default:
      os_ << "(UNHANDLED_BINOP" << bv->type() << " ";
      assert(false);
      break;
    }

  }

  /* Visit a binop on a bool */
  uint16_t visit_binop(const SymBoolBinop * const b) {
    switch (b->type()) {
    case SymBool::AND:
    case SymBool::IFF:
    case SymBool::IMPLIES:
    case SymBool::OR:
    case SymBool::XOR:
      return (*this)(b->a_);
      break;
    default:
      os_ << "(UNHANDLED_BINOP" << b->type() << " ";
      assert(false);
    }
  }


  uint16_t visit_unop(const SymBitVectorUnop * const bv) {
    switch (bv->type()) {
    case SymBitVector::NOT:
      return (*this)(bv->bv_);
      break;
    case SymBitVector::U_MINUS:
      return (*this)(bv->bv_);
      break;
    default:
      os_ << "UNHANDLED_UNOP" << bv->type() << " ";
      assert(false);
    }

  }


  uint16_t visit_compare(const SymBoolCompare * const b) {

    switch (b->type()) {
    case SymBool::EQ:
    case SymBool::GE:
    case SymBool::SIGN_GE:
    case SymBool::GT:
    case SymBool::SIGN_GT:
    case SymBool::LE:
    case SymBool::SIGN_LE:
    case SymBool::LT:
    case SymBool::SIGN_LT:
      return (*this)(b->a_);
      break;
    default:
      os_ << "(UNHANDLED_COMPARE" << b->type() << " ";
      assert(false);
    }

  }

  /** Visit a bit-vector constant */
  uint16_t visit(const SymBitVectorConstant * const bv) {
    return bv->size_;
  }

  /** Visit a bit-vector extract */
  uint16_t visit(const SymBitVectorExtract * const bv) {
    return (bv->high_bit_ - bv->low_bit_) + 1 ;
  }

  /** Visit a bit-vector function */
  uint16_t visit(const SymBitVectorFunction * const bv) {
    return bv->f_.return_type;
  }

  /** Visit a bit-vector if-then-else */
  uint16_t visit(const SymBitVectorIte * const bv) {
    return (*this)(bv->a_);
  }

  /** Visit a bit-vector sign-extend */
  uint16_t visit(const SymBitVectorSignExtend * const bv) {
    return bv->size_ ;
  }

  /** Visit a bit-vector variable */
  uint16_t visit(const SymBitVectorVar * const bv) {
    return bv->size_ ;
  }

  /** Visit an array lookup */
  uint16_t visit(const SymBitVectorArrayLookup * const bv) {
    assert(false);
    return 0;
  }

  /** Visit a boolean ARRAY EQ */
  uint16_t visit(const SymBoolArrayEq * const b) {
    assert(false);
    return 0;
  }

  /** Visit a boolean FALSE */
  uint16_t visit(const SymBoolFalse * const b) {
    return 1;
  }

  /** Visit a boolean NOT */
  uint16_t visit(const SymBoolNot * const b) {
    return (*this)(b->b_);
  }

  /** Visit a boolean TRUE */
  uint16_t visit(const SymBoolTrue * const b) {
    return 1;
  }

  /** Visit a boolean VAR */
  uint16_t visit(const SymBoolVar * const b) {
    return 1;
  }

  /** Visit an array STORE */
  uint16_t visit(const SymArrayStore * const a) {
    assert(false);
    return 0;
  }

  /** Visit an array VAR */
  uint16_t visit(const SymArrayVar * const a) {
    assert(false);
    return 0;
  }

private:
  std::ostream& os_;

};

} //namespace

#endif
