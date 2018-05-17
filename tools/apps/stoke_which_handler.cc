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
#include "src/ext/cpputil/include/io/console.h"

#include "src/symstate/memory/trivial.h"
#include "src/validator/handlers/combo_handler.h"

#include "tools/gadgets/target.h"
#include "src/specgen/specgen.h"
#include "src/validator/handler.h"
#include "src/validator/handlers.h"

using namespace cpputil;
using namespace std;
using namespace stoke;
using namespace x64asm;

struct CodeReader {
  void operator()(std::istream& is, Code& t) {
    is >> t;
  }
};

struct CodeWriter {
  void operator()(std::ostream& os, const Code& t) {
    os << t;
  }
};

auto& input_header = Heading::create("More Input Formats:");

auto& code_arg = ValueArg<Code, CodeReader, CodeWriter>::create("code")
                 .description("Input code directly");

auto& dbg = Heading::create("Formula Printing Options:");
auto& only_live_out_arg = FlagArg::create("only_live_outs")
                          .description("Only show live out registers");
auto& show_unchanged_arg = FlagArg::create("show_unchanged")
                           .description("Show the formula for unchanged registers");
auto& use_smtlib_format_arg = FlagArg::create("smtlib_format")
                              .description("Show formula in smtlib format");
auto& no_simplify_arg = FlagArg::create("no_simplify")
                        .description("Don't simplify formulas before printing them.");

cpputil::ValueArg<std::string>& strata_path_arg =
  cpputil::ValueArg<std::string>::create("strata_path")
  .usage("<path/to/dir>")
  .description("The path to the directory with the strata circuits (a collection of .s files)");

auto& opc_arg = ValueArg<string>::create("opc")
                .description("The opcode to consider;  use opcode_number to indicate an imm8 argument");



int main(int argc, char** argv) {

  // not actually required here
  //strata_path_arg.required(true);
  target_arg.required(false);

  CommandLineConfig::strict_with_convenience(argc, argv);
  DebugHandler::install_sigsegv();
  DebugHandler::install_sigill();

  bool opcode_provided;
  if (opc_arg.has_been_provided()) {
    opcode_provided = true;
  }

  if (!opcode_provided) {
    if (!code_arg.has_been_provided() && !target_arg.has_been_provided()) {
      Console::error() << "Either --code or --target required." << endl;
    }

    if (code_arg.has_been_provided() && target_arg.has_been_provided()) {
      Console::error() << "At most one of --code and --target can be provided." << endl;
    }
  }

  Code code;
  if (code_arg.has_been_provided()) {
    code = code_arg.value();
  }

  if (opcode_provided) {
    auto instr = get_instruction_from_string(opc_arg);
    code.push_back(instr);
  }


  // build initial state
  SymState state;
  TrivialMemory* mem = NULL;
  state = SymState("", true);
  mem = new TrivialMemory();
  state.memory = mem;

  std::string strata_path_included_h("");
  std::string strata_path_non_included_h("");

  //ComboHandler ch(strata_path_arg.value());
  ComboHandler ch1("/home/sdasgup3/Github/strata-data/circuits");
  ComboHandler ch2("");

  Handler::SupportLevel level1;
  Handler::SupportLevel level2;
  for (auto it : code) {
    if (it.get_opcode() == Opcode::LABEL_DEFN) continue;
    if (it.get_opcode() == Opcode::RET) break;


    auto h = ch1.get_handler(it, level1) ;

    if (dynamic_cast<StrataHandler *>(h)) {
      //std::cout << opc_arg.value() << ":StrataHandler" << "\n";
      strata_path_included_h = string("StrataHandler");
    } else if (dynamic_cast<PackedHandler *>(h)) {
      //std::cout << opc_arg.value()  << ":PackedHandler" << "\n";
      strata_path_included_h = string("PackedHandler");
    } else if (dynamic_cast<SimpleHandler *>(h)) {
      //std::cout << opc_arg.value() <<":SimpleHandler" << "\n";
      strata_path_included_h = string("SimpleHandler");
    } else if (dynamic_cast<AddHandler *>(h)) {
      //std::cout << opc_arg.value() <<":AddHandler" << "\n";
      strata_path_included_h = string("AddHandler");
    } else if (dynamic_cast<ConditionalHandler *>(h)) {
      //std::cout << opc_arg.value() << ":ConditionalHandler" << "\n";
      strata_path_included_h = string("ConditionalHandler");
    } else if (dynamic_cast<LeaHandler *>(h)) {
      //std::cout << opc_arg.value() << ":LeaHandler" << "\n";
      strata_path_included_h = string("LeaHandler");
    } else if (dynamic_cast<MoveHandler *>(h)) {
      //std::cout << opc_arg.value() << ":MoveHandler" << "\n";
      strata_path_included_h = string("MoveHandler");
    } else if (dynamic_cast<PunpckHandler *>(h)) {
      //std::cout << opc_arg.value() << ":PunpckHandler" << "\n";
      strata_path_included_h = string("PunpckHandler");
    } else if (dynamic_cast<ShiftHandler *>(h)) {
      //std::cout << opc_arg.value() <<":ShiftHandler" << "\n";
      strata_path_included_h = string("ShiftHandler");
    } else if (dynamic_cast<PseudoHandler *>(h)) {
      //std::cout << opc_arg.value() << ":PseudoHandler" << "\n";
      strata_path_included_h = string("PseudoHandler");
    } else {
      //std::cout << opc_arg.value() << ":None" << "\n";
      strata_path_included_h = string("None");
    }

  }

  for (auto it : code) {
    if (it.get_opcode() == Opcode::LABEL_DEFN) continue;
    if (it.get_opcode() == Opcode::RET) break;


    auto h = ch2.get_handler(it, level2) ;
    if (dynamic_cast<PackedHandler *>(h)) {
      //std::cout << opc_arg.value()  << ":PackedHandler" << "\n";
      strata_path_non_included_h = string("PackedHandler");
    } else if (dynamic_cast<SimpleHandler *>(h)) {
      //std::cout << opc_arg.value() <<":SimpleHandler" << "\n";
      strata_path_non_included_h = string("SimpleHandler");
    } else if (dynamic_cast<AddHandler *>(h)) {
      //std::cout << opc_arg.value() <<":AddHandler" << "\n";
      strata_path_non_included_h = string("AddHandler");
    } else if (dynamic_cast<ConditionalHandler *>(h)) {
      //std::cout << opc_arg.value() << ":ConditionalHandler" << "\n";
      strata_path_non_included_h = string("ConditionalHandler");
    } else if (dynamic_cast<LeaHandler *>(h)) {
      //std::cout << opc_arg.value() << ":LeaHandler" << "\n";
      strata_path_non_included_h = string("LeaHandler");
    } else if (dynamic_cast<MoveHandler *>(h)) {
      //std::cout << opc_arg.value() << ":MoveHandler" << "\n";
      strata_path_non_included_h = string("MoveHandler");
    } else if (dynamic_cast<PunpckHandler *>(h)) {
      //std::cout << opc_arg.value() << ":PunpckHandler" << "\n";
      strata_path_non_included_h = string("PunpckHandler");
    } else if (dynamic_cast<ShiftHandler *>(h)) {
      //std::cout << opc_arg.value() <<":ShiftHandler" << "\n";
      strata_path_non_included_h = string("ShiftHandler");
    } else if (dynamic_cast<PseudoHandler *>(h)) {
      //std::cout << opc_arg.value() << ":PseudoHandler" << "\n";
      strata_path_non_included_h = string("PseudoHandler");
    } else {
      //std::cout << opc_arg.value() << ":None" << "\n";
      strata_path_non_included_h = string("None");
    }

  }


  if (strata_path_non_included_h == strata_path_included_h) {
    std::cout << opc_arg.value() << ":Equal:"<< strata_path_included_h << "\n";
  } else {
    std::cout << opc_arg.value() << ":"<< strata_path_included_h << ":"<< strata_path_non_included_h << "\n";
  }

  return 0;
}
