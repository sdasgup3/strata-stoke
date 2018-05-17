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

#include <iostream>

#include "src/ext/cpputil/include/command_line/command_line.h"
#include "src/ext/cpputil/include/signal/debug_handler.h"
#include "src/ext/cpputil/include/io/filterstream.h"
#include "src/ext/cpputil/include/io/column.h"
#include "tools/gadgets/solver.h"
#include "tools/gadgets/validator.h"

using namespace cpputil;
using namespace std;
using namespace stoke;
using namespace x64asm;

cpputil::ValueArg<std::string>& strata_path_arg =
  cpputil::ValueArg<std::string>::create("strata_path")
  .usage("<path/to/dir>")
  .description("The path to the directory with the strata circuits (a collection of .s files)")
  .default_val("");

int main(int argc, char** argv) {

  CommandLineConfig::strict_with_convenience(argc, argv);
    DebugHandler::install_sigsegv();
  DebugHandler::install_sigill();

  auto solver = new Z3Solver();
  //string strata_path("/home/sdasgup3/Github/strata-data/circuits/");
  auto validator_ = new Validator(*solver, strata_path_arg.value());
  for (size_t i = 0; i < X64ASM_NUM_OPCODES; ++i) {
    auto op = (Opcode)i;
    string att_ = opcode_write_att(op);
    string intel_ = opcode_write_intel(op);
    std::string str;
    std::stringstream ss(str);
    opcode_write_text(ss, op);

    std::cout << i << ":" << att_ << ":" << intel_ << ":" << ss.str() << ":" << validator_->is_supported(op) << "\n";
  }


  std::cout << " Strata Path: " << strata_path_arg.value() << "\n";

  return 0;
}
