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

#ifndef STOKE_TOOLS_GADGETS_SOLVER_H
#define STOKE_TOOLS_GADGETS_SOLVER_H

#include "src/solver/smtsolver.h"
#include "src/solver/cvc4solver.h"
#include "src/solver/z3solver.h"
#include "tools/args/solver.inc"
#include "src/ext/z3/src/api/c++/z3++.h"

namespace stoke {

class SolverGadget : public SMTSolver {
public:
  SolverGadget() : SMTSolver() {

    switch (solver_arg) {
    case Solver::Z3:
      solver_ = new Z3Solver();
      break;
    case Solver::CVC4:
      solver_ = new Cvc4Solver();
      break;
    default:
      assert(false);
    }

    set_timeout(timeout_arg);
  }

  SMTSolver& set_timeout(uint64_t ms) {
    solver_->set_timeout(ms);
    return *this;
  }
  bool is_sat(const std::vector<SymBool>& constraints) {
    return solver_->is_sat(constraints);
  }
  bool has_model() const {
    return solver_->has_model();
  }
  cpputil::BitVector get_model_bv(const std::string& var, uint16_t octs) {
    return solver_->get_model_bv(var, octs);
  }
  bool get_model_bool(const std::string& var) {
    return solver_->get_model_bool(var);
  }
  std::map<uint64_t, cpputil::BitVector>
  get_model_array(const std::string& var, uint16_t key_size, uint16_t value_size) {
    return solver_->get_model_array(var, key_size, value_size);
  }
  virtual bool has_error() {
    return solver_->has_error();
  }
  virtual std::string get_error() {
    return solver_->get_error();
  }

  z3::expr getZ3Formula(const SymBool& bv)  {
    auto z3Solver_ = dynamic_cast<Z3Solver *>(solver_);
    assert(NULL != z3Solver_);
    return z3Solver_->getZ3Formula(bv);
  }
  z3::expr getZ3Formula(const SymBitVector& bv)  {
    auto z3Solver_ = dynamic_cast<Z3Solver *>(solver_);
    assert(NULL != z3Solver_);
    return z3Solver_->getZ3Formula(bv);
  }

private:

  SMTSolver* solver_;
};

} // namespace stoke

#endif
