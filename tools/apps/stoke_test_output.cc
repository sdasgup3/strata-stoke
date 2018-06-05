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

  // Read the testcases and testouts
  cout << "Reading the Testsuit\n";
  FunctionsGadget aux_fxns;
  TargetGadget target(aux_fxns, false);
  SeedGadget seed;
  TestSetGadget test_set(seed);
  TestOutputGadget test_output(seed);

  if(test_set.size() != test_output.size()) {
    assert(false && "Size Mismatch1");
  }

  if(test_output.size() == 0) {
    assert(false && "No Output!");
  }


  // Prepare the sandbox
  cout << "Preparing Sandbox\n";
  SandboxGadget sb(test_set, aux_fxns);

  // Run all the testcases using the cfg `target`
  cout << "Run " << test_set.size() <<   " tests\n";
  sb.insert_function(target);
  sb.set_entrypoint(target.get_code()[0].get_operand<x64asm::Label>(0));
  sb.run();

  // Obtain the instruction
  auto instr = target.get_code()[1];
  auto opcode = instr.get_opcode();

  // Collect the run results
  cout << "Collect Results\n";
  std::vector<CpuState> reference_out_;
  for (auto i = sb.result_begin(), ie = sb.result_end(); i != ie; ++i) {
    CpuState ss = *i;
    if (ss.code != stoke::ErrorCode::NORMAL) {
      cout<< "Sandbox did not finish normally: " << opcode << " " <<  (int)ss.code << std::endl;
      exit(4);
    }
    reference_out_.push_back(*i);
  }

  if(reference_out_.size() != test_output.size()) {
    assert(false && "Size Mismatch2");
  }

  //Iterate on each testcase
  cout << "Begin Tests\n";
  int i = 0;
  RegSet *dummy = new RegSet();
  int count = 0;

  for (unsigned i = 0 ; i < test_output.size(); i++) {
    auto & cs_sandbox_out = reference_out_[i];
    auto & cs_output = test_output[i];


    if(cs_sandbox_out != cs_output) {
      cout << "Sandbox Run:\n" << cs_sandbox_out << endl;
      cout << "K Run:\n" << cs_output << endl;
    }
    i++;
    if (i%1000 == 0 ) {
      cout << "Completed " <<i << "cases" <<  endl;
    }

  }

  return 0;
}
