// This file contains the entry points for the global functions defined
// in the LLVM GameBoy back-end.

#ifndef LLVM_GAMEBOY_H
#define LLVM_GAMEBOY_H

#include "llvm/CodeGen/SelectionDAGNodes.h"
#include "llvm/Pass.h"
#include "llvm/PassRegistry.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {

class GameBoyTargetMachine;
class FunctionPass;

Pass *createGameBoyShiftExpandPass();
FunctionPass *createGameBoyISelDag(GameBoyTargetMachine &TM, CodeGenOpt::Level OptLevel);
// FunctionPass *createGameBoyExpandPseudoPass();
// FunctionPass *createGameBoyFrameAnalyzerPass();
// FunctionPass *createGameBoyBranchSelectionPass();

// Most likely will need to add a pass to add the prologue code for the ROM.
namespace GameBoy {

/// TODO: Add various address spaces; include ROM bank here?
// enum AddressSpace;
// inline int getCartridgeMemoryBank(); Are we using MBC5 only?

} // end namespace GameBoy
} // end namespace llvm

#endif // LLVM_GAMEBOY_H