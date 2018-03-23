// Copyright 2013-2015 Stanford University
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

#include <chrono>
#include <fstream>
#include <iostream>
#include <random>
#include <map>
#include <string>
#include <vector>
#include <cassert>

#include "src/ext/cpputil/include/command_line/command_line.h"
#include "src/ext/cpputil/include/signal/debug_handler.h"
#include "src/ext/cpputil/include/io/filterstream.h"
#include "src/ext/cpputil/include/io/column.h"
#include "src/ext/cpputil/include/io/console.h"

#include "src/ext/x64asm/src/reg_set.h"

#include "src/symstate/simplify.h"
#include "src/symstate/pretty_visitor.h"

#include "src/validator/bounded.h"
#include "src/validator/handler.h"
#include "src/validator/handlers/combo_handler.h"

#include "tools/gadgets/cost_function.h"
#include "tools/gadgets/functions.h"
#include "tools/gadgets/rewrite.h"
#include "tools/gadgets/sandbox.h"
#include "tools/gadgets/seed.h"
#include "tools/gadgets/solver.h"
#include "tools/gadgets/target.h"
#include "tools/gadgets/testcases.h"
#include "tools/gadgets/validator.h"
#include "tools/gadgets/verifier.h"



#include "src/specgen/specgen.h"
#include "src/specgen/support.h"

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>

using namespace cpputil;
using namespace std;
using namespace stoke;
using namespace x64asm;
using namespace boost;

int main(int argc, char** argv) {

  target_arg.required(true);
  rewrite_arg.required(false);
  CommandLineConfig::strict_with_convenience(argc, argv);
  DebugHandler::install_sigsegv();
  DebugHandler::install_sigill();

  SolverGadget solver;

  // Read the testcuite
  cout << "Reading the Testsuit\n";
  FunctionsGadget aux_fxns;
  TargetGadget target(aux_fxns, false);
  SeedGadget seed;
  TestSetGadget test_set(seed);

  // Prepare the sandbox
  cout << "Preparing Sandbox\n";
  SandboxGadget sb(test_set, aux_fxns);

  // Run all the testcases using the cfg `target`
  cout << "Run " << test_set.size() <<   " tests\n";
  sb.insert_function(target);
  sb.set_entrypoint(target.get_code()[0].get_operand<x64asm::Label>(0));
  sb.run();

  // Collect the run results
  cout << "Collect Results\n";
  std::vector<CpuState> reference_out_;
  for (auto i = sb.result_begin(), ie = sb.result_end(); i != ie; ++i) {
    CpuState ss = *i;
    if (ss.code != stoke::ErrorCode::NORMAL) {
      cout<< "Sandbox did not finish normally: " << (int)ss.code << std::endl;
      exit(4);
    }
    reference_out_.push_back(*i);
  }

  // Obtain the instruction
  auto instr = target.get_code()[1];
  auto opcode = instr.get_opcode();

  // Build the sym formula of the circuit
  cout << "Build Handler\n";
  ComboHandler ch;
  if (ch.get_support(instr) == Handler::SupportLevel::NONE) {
    cout << "\033[1;31mNot supported\033[0m\n";
    exit(4);
  }

  // check equivalence of two symbolic states for a given register
  auto is_eq = [&solver](auto& reg, auto a, auto b, stringstream& explanation, auto& cs) {
    SymBool eq = a == b;
    //cout << SymSimplify().simplify(a) << "\n\n";
    //cout << b << "\n\n";
    bool res = solver.is_sat({ eq });
    if (solver.has_error()) {
      explanation << "  solver encountered error: " << solver.get_error() << endl;
      return false;
    }
    if (!res) {
      cout << cs << "\n";
      cout << "\n\n";

      explanation << "  states do not agree for '" << (*reg) << "':" << endl;
      auto simplify = true;
      if (!simplify) {
        explanation << "    validator: " << (a) << endl;
      } else {
        explanation << "    validator: " << SymSimplify().simplify(a) << endl;
      }
      explanation << "    sandbox:   " << b << endl;
      return false;
    } else {
      return true;
    }
  };

  // The registers on which to proof equivalence
  /* All registers
  auto rs = RegSet::universe();
  // the af flag is not currently supported by the validator
  rs = rs - (RegSet::empty() + Constants::eflags_af());
  // don't check undefined outputs
  if (instr.get_opcode() == CALL_LABEL) {
    // more precise dataflow information from annotations
    for (const auto& fxn : aux_fxns) {
      if (instr.get_operand<Label>(0) == fxn.get_code()[0].get_operand<Label>(0)) {
        rs = rs - fxn.get_may_must_sets().maybe_undef_set;
      }
    }
  } else {
    rs = rs - instr.maybe_undef_set();
  }
  */
  // Just the live_outs
  auto rs = live_out_arg.value();


  //Iterate on each testcase
  cout << "Check Equivalence\n";
  int i = 0;
  for (const auto& cs : test_set) {
    // Create a formula with initial state as the test input
    SymState sym_validator(cs);
    ch.build_circuit(instr, sym_validator);
    if (ch.has_error()) {
      cout << "Error building a circuit: " << ch.error() << endl;
      return 0;
    }

    // Create a formula with with the final output
    SymState sb_validator(reference_out_[i]);

    // Do equivalence check
    auto eq = true;
    stringstream ss;
    ss << "Sandbox and validator do not agree for '" << instr << "' (opcode " << opcode << ")" << endl;
    for (auto gp_it = rs.gp_begin(); gp_it != rs.gp_end(); ++gp_it) {
      eq = eq && is_eq(gp_it, sym_validator.lookup(*gp_it), sb_validator.lookup(*gp_it), ss, cs);
    }
    for (auto sse_it = rs.sse_begin(); sse_it != rs.sse_end(); ++sse_it) {
      eq = eq && is_eq(sse_it, sym_validator.lookup(*sse_it), sb_validator.lookup(*sse_it), ss, cs);
    }
    for (auto flag_it = rs.flags_begin(); flag_it != rs.flags_end(); ++flag_it) {
      eq = eq && is_eq(flag_it, sym_validator[*flag_it], sb_validator[*flag_it], ss, cs);
    }
    if (!eq) {
      cout << ss.str() << endl;
      return 1;
    }


    i++;
    if (i%1000 == 0 ) {
      cout << "Completed " <<i << "cases" <<  endl;
    }
  }

  return 0;
}



