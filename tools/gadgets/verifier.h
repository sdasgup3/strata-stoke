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

#ifndef STOKE_TOOLS_GADGETS_VERIFIER_H
#define STOKE_TOOLS_GADGETS_VERIFIER_H

#include <regex>

#include "src/ext/cpputil/include/io/console.h"

#include "src/cost/cost_function.h"
#include "src/verifier/hold_out.h"
#include "src/verifier/none.h"
#include "src/verifier/sequence.h"
#include "src/verifier/verifier.h"
#include "src/validator/bounded.h"
//#include "src/validator/ddec.h"

#include "tools/args/bounded_validator.inc"
//#include "tools/args/ddec_validator.inc"
#include "tools/args/in_out.inc"
#include "tools/args/testcases.inc"
#include "tools/args/verifier.inc"
#include "tools/gadgets/solver.h"

namespace stoke {

class VerifierGadget : public Verifier {
public:

  VerifierGadget(Sandbox& sandbox, CorrectnessCost& fxn) : Verifier(), verifier_(NULL), solver_(NULL) {

    solver_ = new SolverGadget();

    std::vector<Verifier*> verifiers;
    std::vector<std::string> splits = split(strategy_arg.value(), std::regex("[ ,+]"));
    for (auto it : splits) {
      if (it == "hold_out" && (test_set_arg.value().size() == 0 || testcases_arg.value().size() == 0)) {
        cpputil::Console::error() << "No test-cases given for hold_out verification." << std::endl;
      }
      verifiers.push_back(make_by_name(it, sandbox, fxn));
    }

    verifier_ = new SequenceVerifier(verifiers);

    verifier_->set_sandbox(&sandbox);
    verifier_->set_heap_out(heap_out_arg);
    verifier_->set_stack_out(stack_out_arg);
  }

  Verifier& set_sandbox(Sandbox* sb) {
    verifier_->set_sandbox(sb);
    return *this;
  }

  inline bool verify(const Cfg& target, const Cfg& rewrite) {
    return verifier_->verify(target, rewrite);
  }

  inline size_t counter_examples_available() {
    return verifier_->counter_examples_available();
  }

  std::vector<CpuState> get_counter_examples() {
    return verifier_->get_counter_examples();
  }

  bool has_error() {
    return verifier_->has_error();
  }

  std::string error() {
    return verifier_->error();
  }

private:

  BoundedValidator::AliasStrategy parse_alias() {
    std::string alias = alias_strategy_arg.value();

    if (alias == "basic" || alias == "tree" || alias == "prune" || alias == "treeprune") {
      return BoundedValidator::AliasStrategy::BASIC;
    } else if (alias == "string") {
      return BoundedValidator::AliasStrategy::STRING;
    } else if (alias == "string_antialias") {
      return BoundedValidator::AliasStrategy::STRING_NO_ALIAS;
    } else if (alias == "flat" || alias == "array") {
      return BoundedValidator::AliasStrategy::FLAT;
    } else {
      std::cerr << "Unrecognized alias strategy \"" << alias << "\"" << std::endl;
      exit(1);
    }
  }

  Verifier* make_by_name(std::string s, Sandbox& sandbox, CorrectnessCost& fxn) {
    if (s == "bounded") {
      auto bv = new BoundedValidator(*solver_, strata_path_arg.value());
      bv->set_bound(bound_arg.value());
      bv->set_alias_strategy(parse_alias());
      bv->set_no_bailout(no_bailout_arg.value());
      bv->set_nacl(verify_nacl_arg);
      return bv;
      /*
    } else if (s == "ddec") {
      auto ddec = new DdecValidator(*solver_);
      std::cout << "no try sign extend arg: " << no_try_sign_extend_arg.value() << std::endl;
      ddec->set_try_sign_extend(!no_try_sign_extend_arg.value());
      ddec->set_no_bv(no_ddec_bv_arg.value());
      ddec->set_sound_nullspace(sound_nullspace_arg.value());
      ddec->set_alias_strategy(parse_alias());
      ddec->set_bound(bound_arg.value());
      ddec->set_nacl(verify_nacl_arg);
      return ddec;
      */
    } else if (s == "hold_out") {
      return new HoldOutVerifier(fxn);
    } else if (s == "none") {
      return new NoneVerifier();
    } else {
      std::cerr << "Unrecognized verifier name \"" << s << "\"" << std::endl;
      exit(1);
    }
  }

  //from https://github.com/trikitrok/StringCalculatorAdditionUsingGoogleMock
  std::vector<std::string> split(const std::string& str, std::regex rgx) {
    std::sregex_token_iterator
    first{begin(str), end(str), rgx, -1},
          last;
    return{first, last};
  }

  Verifier* verifier_;
  SMTSolver* solver_;
};

} // namespace stoke

#endif
