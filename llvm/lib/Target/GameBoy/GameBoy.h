//===-- GameBoy.h - Top-level interface for GameBoy representation ------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the entry points for global functions defined in the LLVM
// GameBoy back-end.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_GameBoy_H
#define LLVM_GameBoy_H

#include "llvm/CodeGen/SelectionDAGNodes.h"
#include "llvm/Pass.h"
#include "llvm/PassRegistry.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {

class GameBoyTargetMachine;
class FunctionPass;

Pass *createGameBoyShiftExpandPass();
FunctionPass *createGameBoyISelDag(GameBoyTargetMachine &TM,
                               CodeGenOpt::Level OptLevel);
FunctionPass *createGameBoyExpandPseudoPass();
FunctionPass *createGameBoyFrameAnalyzerPass();
FunctionPass *createGameBoyBranchSelectionPass();

void initializeGameBoyShiftExpandPass(PassRegistry &);
void initializeGameBoyExpandPseudoPass(PassRegistry &);

/// Contains the GameBoy backend.
namespace GameBoy {

/// An integer that identifies all of the supported GameBoy address spaces.
enum AddressSpace {
  DataMemory,
  ProgramMemory,
  ProgramMemory1,
  ProgramMemory2,
  ProgramMemory3,
  ProgramMemory4,
  ProgramMemory5,
  NumAddrSpaces,
};

/// @brief This is defined at compile time and determines the memory layouts.
enum MemoryBankController {
  MBC1,
  MBC2,
  MBC3,
  MBC4,
  MBC5,
};

MemoryBankController MBC = MBC5
unsigned int ROMBank;
unsigned int RAMBank;

/// Checks if a given type is a pointer to program memory.
template <typename T> bool isProgramMemoryAddress(T *V) {
  auto *PT = cast<PointerType>(V->getType());
  assert(PT != nullptr && "unexpected MemSDNode");
  return PT->getAddressSpace() == ProgramMemory ||
         PT->getAddressSpace() == ProgramMemory1 ||
         PT->getAddressSpace() == ProgramMemory2 ||
         PT->getAddressSpace() == ProgramMemory3 ||
         PT->getAddressSpace() == ProgramMemory4 ||
         PT->getAddressSpace() == ProgramMemory5;
}

template <typename T> bool isROMMemoryAddress(T *V) {
  // Attempt a cast to PointerType
  auto *PT = cast<PointerType>(V->getType());
  assert(PT != nullptr && "unexpected MemSDNode");
  switch(MBC) {
    case MBC1:
    case MBC2:
    case MBC3:
    case MBC4:
    case MBC5:
    default:
      return PT->getAddressSpace() >= 0 && PT->getAddressSpace() <= 511;
  }
}

template <typename T> AddressSpace getAddressSpace(T *V) {
  auto *PT = cast<PointerType>(V->getType());
  assert(PT != nullptr && "unexpected MemSDNode");
  unsigned AS = PT->getAddressSpace();
  if (AS < NumAddrSpaces)
    return static_cast<AddressSpace>(AS);
  return NumAddrSpaces;
}

/// @brief Based
/// @tparam T 
/// @param V 
/// @return 
template <typename T> unsigned int getBankedROMSpace(T *V) {
  auto *PT = cast<PointerType>(V->getType());
  assert(PT != nullptr && "unexpected MemSDNode");
  unsigned AS = PT->getAddressSpace();
  
  // Check ranges for individual MBCs
  switch(MBC) {
    default:
      if (AS < 511)
        return static_cast<unsigned int>(AS);
      return 511;
  }
}

inline bool isProgramMemoryAccess(MemSDNode const *N) {
  auto *V = N->getMemOperand()->getValue();
  if (V != nullptr && isProgramMemoryAddress(V))
    return true;
  return false;
}

inline bool isROMMemoryAccess(MemSDNode const *N) {
  auto *V = N->getMemOperand()->getValue();
  if (V != nullptr && isROMMemoryAddress(V))
    return true;
  return false;
}

// Get the index of the program memory bank.
//  -1: not program memory
//   0: ordinary program memory
// 1~5: extended program memory
inline int getProgramMemoryBank(MemSDNode const *N) {
  auto *V = N->getMemOperand()->getValue();
  if (V == nullptr || !isProgramMemoryAddress(V))
    return -1;
  AddressSpace AS = getAddressSpace(V);
  assert(ProgramMemory <= AS && AS <= ProgramMemory5);
  return static_cast<int>(AS - ProgramMemory);
}

/// @brief Get the index of the MBC ROM bank. Assumes MBC5.
/// @param N The memory SD node provided.
/// @return The correct index of the ROM bank available.
inline int getMemoryROMBank(MemSDNode const *N) {
  // Get the value of the memory operand.
  auto *V = N->getMemOperand()->getValue();
  if (V == nullptr || !isROMMemoryAccess(V))
    return -1;
  unsigned int AS = 
  switch(MBC) {
    case MBC5:
    default:
      // Check that this is within the range 1~512
      assert(0 <= V && V <= 511);
      return static_cast<int>(V);
  }
}

} // end of namespace GameBoy

} // end namespace llvm

#endif // LLVM_GameBoy_H
