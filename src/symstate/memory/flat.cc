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

#include "src/symstate/memory/flat.h"

using namespace stoke;
using namespace std;

/** Updates the memory with a write. */
SymBool FlatMemory::write(SymBitVector address, SymBitVector value, uint16_t size, size_t line_no) {

  // Little Endian
  // The least significant bit of value (i.e. the lowest bits) go in the lowest addresses

  //std::cout << "\n\nAddress: " << address << "\n";
  //std::cout << "Value: " << value << "\n";

  for (size_t i = 0; i < size/8; ++i) {
    heap_ = heap_.update(address + SymBitVector::constant(64, i), value[8*i+7][8*i]);
  }

  // Update the access list
  auto access_var = SymBitVector::tmp_var(64);
  if (!no_constraints_)
    constraints_.push_back(access_var == address);
  access_list_[access_var.ptr] = size;

  // Get a new array variable and update the heap
  if (!no_constraints_) {
    auto new_arr = SymArray::tmp_var(64, 8);
    auto constr = heap_ == new_arr;
    constraints_.push_back(constr);
    heap_ = new_arr;
  }

  return SymBool::_false();
}

/** Reads from the memory.  Returns value and segv condition. */
std::pair<SymBitVector,SymBool> FlatMemory::read(SymBitVector address, uint16_t size, size_t line_no) {
  // Update the access list
  auto access_var = SymBitVector::tmp_var(64);
  constraints_.push_back(access_var == address);
  access_list_[access_var.ptr] = size;



  SymBitVector value = heap_[address];

  for (size_t i = 1; i < size/8; ++i) {
    value = heap_[address + SymBitVector::constant(64, i)] || value;
  }
  //std::cout << "\n\nAddress(R): " << address << "\n";
  //std::cout << "\n\nValue(R): " << value << "\n";

  return pair<SymBitVector,SymBool>(value, SymBool::_false());
}

/** Create a formula expressing these memory cells with another set. */
SymBool FlatMemory::equality_constraint(FlatMemory& other) {
  return (heap_ == other.heap_);
}


