#ifndef _STOKE_SRC_SYMSTATE_K_VISITOR
#define _STOKE_SRC_SYMSTATE_K_VISITOR

#include "src/symstate/visitor.h"
#include "src/symstate/bitwidth_visitor.h"

namespace stoke {
class SymKVisitor : public SymVisitor<void, void, void> {

public:
  SymKVisitor(std::ostream& os) : os_(os), bitwidthMInt(os) {}

  void visit_binop(const SymBitVectorBinop * const bv) {

    switch (bv->type()) {
    case SymBitVector::AND:
      os_ << "andMInt( ";
      (*this)(bv->a_);
      os_ << ", ";
      (*this)(bv->b_);
      os_ << ")";
      break;
    case SymBitVector::CONCAT:
      os_ << "concatenateMInt( ";
      (*this)(bv->a_);
      os_ << ", ";
      (*this)(bv->b_);
      os_ << ")";
      break;
    case SymBitVector::DIV:
      os_ << "udivMInt( ";
      (*this)(bv->a_);
      os_ << ", ";
      (*this)(bv->b_);
      os_ << ")";
      break;
    case SymBitVector::MINUS:
      os_ << "subMInt( ";
      (*this)(bv->a_);
      os_ << ", ";
      (*this)(bv->b_);
      os_ << ")";
      break;
    case SymBitVector::MOD:
      os_ << "uremMInt( ";
      (*this)(bv->a_);
      os_ << ", ";
      (*this)(bv->b_);
      os_ << ")";
      break;
    case SymBitVector::MULT:
      os_ << "mulMInt( ";
      (*this)(bv->a_);
      os_ << ", ";
      (*this)(bv->b_);
      os_ << ")";
      break;
    case SymBitVector::OR:
      os_ << "orMInt( ";
      (*this)(bv->a_);
      os_ << ", ";
      (*this)(bv->b_);
      os_ << ")";
      break;
    case SymBitVector::PLUS:
      os_ << "addMInt( ";
      (*this)(bv->a_);
      os_ << ", ";
      (*this)(bv->b_);
      os_ << ")";
      break;
    case SymBitVector::ROTATE_LEFT:
      os_ << "rol( ";
      (*this)(bv->a_);
      os_ << ", ";
      (*this)(bv->b_);
      os_ << ")";
      break;
    case SymBitVector::ROTATE_RIGHT:
      os_ << "ror( ";
      (*this)(bv->a_);
      os_ << ", ";
      (*this)(bv->b_);
      os_ << ")";
      break;
    case SymBitVector::SHIFT_LEFT:
      os_ << "shiftLeftMInt( ";
      (*this)(bv->a_);
      os_ << ", uvalueMInt(";
      (*this)(bv->b_);
      os_ << "))";
      break;
    case SymBitVector::SHIFT_RIGHT:
      os_ << "lshrMInt( ";
      (*this)(bv->a_);
      os_ << ", uvalueMInt(";
      (*this)(bv->b_);
      os_ << "))";
      break;
    case SymBitVector::SIGN_DIV:
      os_ << "sdivMInt( ";
      (*this)(bv->a_);
      os_ << ", ";
      (*this)(bv->b_);
      os_ << ")";
      break;
    case SymBitVector::SIGN_MOD:
      os_ << "sremMInt( ";
      (*this)(bv->a_);
      os_ << ", ";
      (*this)(bv->b_);
      os_ << ")";
      break;
    case SymBitVector::SIGN_SHIFT_RIGHT:
      os_ << "aShiftRightMInt( ";
      (*this)(bv->a_);
      os_ << ", uvalueMInt(";
      (*this)(bv->b_);
      os_ << "))";
      break;
    case SymBitVector::XOR:
      os_ << "xorMInt( ";
      (*this)(bv->a_);
      os_ << ", ";
      (*this)(bv->b_);
      os_ << ")";
      break;
    default:
      os_ << "(UNHANDLED_BINOP" << bv->type() << " ";
      assert(false);
    }

  }

  /* Visit a binop on a bool */
  void visit_binop(const SymBoolBinop * const b) {

    os_ << "(";
    (*this)(b->a_);

    switch (b->type()) {
    case SymBool::AND:
      os_ << " andBool ";
      break;
    case SymBool::IFF:
      os_ << " ==Bool ";
      break;
    case SymBool::IMPLIES:
      os_ << " implies ";
      break;
    case SymBool::OR:
      os_ << " orBool ";
      break;
    case SymBool::XOR:
      os_ << " xorBool ";
      break;
    default:
      os_ << "(UNHANDLED_BINOP" << b->type() << " ";
      assert(false);
    }

    (*this)(b->b_);
    os_ << ")";
  }


  void visit_unop(const SymBitVectorUnop * const bv) {

    switch (bv->type()) {
    case SymBitVector::NOT:
      os_ << "negMInt( ";
      (*this)(bv->bv_);
      os_ << ")";
      break;
    case SymBitVector::U_MINUS:
    {
      uint16_t sz = bitwidthMInt((bv->bv_));
      os_ << "addMInt(mi("<< sz  << ", 1), negMInt( ";
      (*this)(bv->bv_);
      os_ << "))";
      break;
    }
    default:
      os_ << "UNHANDLED_UNOP" << bv->type() << " ";
      assert(false);
    }

  }


  void visit_compare(const SymBoolCompare * const b) {

    switch (b->type()) {
    case SymBool::EQ:
      os_ << "eqMInt( ";
      break;
    case SymBool::GE:
      os_ << "ugeMInt( ";
      break;
    case SymBool::SIGN_GE:
      os_ << "sgeMInt( ";
      break;
    case SymBool::GT:
      os_ << "ugtMInt( ";
      break;
    case SymBool::SIGN_GT:
      os_ << "sgtMInt( ";
      break;
    case SymBool::LE:
      os_ << "uleMInt( ";
      break;
    case SymBool::SIGN_LE:
      os_ << "sleMInt( ";
      break;
    case SymBool::LT:
      os_ << "ultMInt( ";
      break;
    case SymBool::SIGN_LT:
      os_ << "sltMInt( ";
      break;
    default:
      os_ << "(UNHANDLED_COMPARE" << b->type() << " ";
      assert(false);
    }

    (*this)(b->a_);
    os_ << ", ";
    (*this)(b->b_);
    os_ << ")";

  }

  /** Visit a bit-vector constant */
  void visit(const SymBitVectorConstant * const bv) {
    os_ << "mi(" << std::dec << bv->size_ << ", " << bv->constant_  << ")";
  }

  /** Visit a bit-vector extract */
  void visit(const SymBitVectorExtract * const bv) {
    uint16_t sz = bitwidthMInt((bv->bv_));
    os_ << "extractMInt( ";
    (*this)(bv->bv_);
    os_ << ", " << std::dec <<  sz - bv->high_bit_ - 1 << ", " <<
        sz - bv->low_bit_  << ")";
  }

  /** Visit a bit-vector function */
  void visit(const SymBitVectorFunction * const bv) {
    os_ << bv->f_.name << "(";

    for (size_t i = 0; i < bv->args_.size(); ++i) {
      (*this)(bv->args_[i]);
      if (i != bv->args_.size() - 1)
        os_ << ", ";
    }

    os_ << ")";
  }

  /** Visit a bit-vector if-then-else */
  void visit(const SymBitVectorIte * const bv) {
    os_ << "(#ifMInt ";
    (*this)(bv->cond_);
    os_ << " #then ";
    (*this)(bv->a_);
    os_ << " #else ";
    (*this)(bv->b_);
    os_ << " #fi)";
  }

  /** Visit a bit-vector sign-extend */
  void visit(const SymBitVectorSignExtend * const bv) {
    //os_ << "signExtend( ";
    //(*this)(bv->bv_);
    //os_ << ", " << bv->size_ << ")";
    os_ << "mi(" << bv->size_ << ", svalueMInt(";
    (*this)(bv->bv_);
    os_ << "))";
  }

  /** Visit a bit-vector variable */
  void visit(const SymBitVectorVar * const bv) {
    os_ <<  bv->name_ ;
  }

  /** Visit an array lookup */
  void visit(const SymBitVectorArrayLookup * const bv) {
    os_ << "(";
    (*this)(bv->a_);
    os_ << ")[";
    (*this)(bv->key_);
    os_ << "]";
  }

  /** Visit a boolean ARRAY EQ */
  void visit(const SymBoolArrayEq * const b) {
    os_ << "(== ";
    (*this)(b->a_);
    os_ << " ";
    (*this)(b->b_);
    os_ << ")";
  }

  /** Visit a boolean FALSE */
  void visit(const SymBoolFalse * const b) {
    os_ << "FALSE";
  }

  /** Visit a boolean NOT */
  void visit(const SymBoolNot * const b) {
    os_ << "(notBool ";
    (*this)(b->b_);
    os_ << ")";
  }

  /** Visit a boolean TRUE */
  void visit(const SymBoolTrue * const b) {
    os_ << "TRUE";
  }

  /** Visit a boolean VAR */
  void visit(const SymBoolVar * const b) {
    os_ <<  b->name_ ;
  }

  /** Visit an array STORE */
  void visit(const SymArrayStore * const a) {
    os_ << "(";
    (*this)(a->a_);
    os_ << " update ";
    (*this)(a->key_);
    os_ << " -> ";
    (*this)(a->value_);
    os_ << ")";
  }

  /** Visit an array VAR */
  void visit(const SymArrayVar * const a) {
    os_ << "<" << a->name_ << "|" << a->key_size_ << "|" << a->value_size_ << ">";
  }

private:
  std::ostream& os_;
  BitWidthVisitor bitwidthMInt;

};

} //namespace

#endif
