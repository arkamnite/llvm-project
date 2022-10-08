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

template <typename T> AddressSpace getAddressSpace(T *V) {
  auto *PT = cast<PointerType>(V->getType());
  assert(PT != nullptr && "unexpected MemSDNode");
  unsigned AS = PT->getAddressSpace();
  if (AS < NumAddrSpaces)
    return static_cast<AddressSpace>(AS);
  return NumAddrSpaces;
}

inline bool isProgramMemoryAccess(MemSDNode const *N) {
  auto *V = N->getMemOperand()->getValue();
  if (V != nullptr && isProgramMemoryAddress(V))
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

} // end of namespace GameBoy

} // end namespace llvm

#endif // LLVM_GameBoy_H
