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


#ifndef STOKE_SRC_SYMSTATE_MEMORY_FLAT_H
#define STOKE_SRC_SYMSTATE_MEMORY_FLAT_H

#include <map>

#include "src/symstate/bitvector.h"
#include "src/symstate/memory.h"

namespace stoke {

/** Models memory as a giant array */
class FlatMemory : public SymMemory {

public:

  FlatMemory(bool no_constraints = false) {
    variable_ = SymArray::tmp_var(64, 8);
    start_variable_ = variable_;
    heap_ = variable_;
    variable_up_to_date_ = true;
    no_constraints_ = no_constraints;
  }

  ~FlatMemory() {
    for (auto& constrain: constraints_) {
      delete constrain.ptr;
    }

    for (auto& access_var: access_list_) {
      delete access_var.first;
    }

    deleteR(heap_.ptr);

  }

  void deleteR(const SymArrayAbstract* delptr) {
    if (NULL == delptr) {
      return;
    }

    if (auto ptr = dynamic_cast<const SymArrayStore*>(delptr)) {
      delete ptr->key_;
      delete ptr->value_;
      deleteR(ptr->a_);
      delete ptr;
    } else if (auto ptr = dynamic_cast<const SymArrayVar*>(delptr)) {
      delete ptr;
    }
    return;

  }

  /** Updates the memory with a write.
   *  Returns condition for segmentation fault */
  SymBool write(SymBitVector address, SymBitVector value, uint16_t size, size_t line_no);

  /** Reads from the memory.  Returns value and segv condition. */
  std::pair<SymBitVector,SymBool> read(SymBitVector address, uint16_t size, size_t line_no);

  /** Create a formula expressing these memory cells with another set. */
  SymBool equality_constraint(FlatMemory& other);

  std::vector<SymBool> get_constraints() {
    return constraints_;
  }

  /** Get a variable representing the memory at this state. */
  SymArray get_variable() {
    if (!variable_up_to_date_) {
      variable_ = SymArray::tmp_var(64, 8);
      variable_up_to_date_ = true;
      constraints_.push_back(variable_ == heap_);
    }

    return variable_;
  }

  SymArray get_start_variable() {
    return start_variable_;
  }

  /** Get list of accesses accessed (via read or write).  This is needed for
   * marking relevant cells valid in the counterexample. */
  std::map<const SymBitVectorAbstract*, uint64_t> get_access_list() {
    return access_list_;
  }

  /** The heap state */
  SymArray heap_;
  /** Extra constraints needed to make everything work. */
  std::vector<SymBool> constraints_;

private:

  /** A variable that represents the heap state */
  bool variable_up_to_date_;
  SymArray variable_;
  SymArray start_variable_;

  /** map of (symbolic address, size) pairs accessed. */
  std::map<const SymBitVectorAbstract*, uint64_t> access_list_;

  bool no_constraints_;

};

};

#endif
