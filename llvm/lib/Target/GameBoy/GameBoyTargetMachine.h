#ifndef LLVM_LIB_TARGET_GAMEBOY_GAMEBOYTARGETMACHINE_H
#define LLVM_LIB_TARGET_GAMEBOY_GAMEBOYTARGETMACHINE_H

// #include "GameBoySubTarget.h"
// #include "MCTargetDesc/GameBoyMCTargetDesc.h"

#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/SelectionDAGISel.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/Subtarget.h"
#include "llvm/CodeGen/TargetInstrInfo.h"

namespace llvm{
class Module;

class GameBoyTargetMachine : public LLVMTargetMachine {
    const DataLayout DataLayout;    // Calculate type size, alignment
    // GameBoySubTarget Subtarget;
    GameBoyInstrInfo InstrInfo;
    TargetFrameInfo FrameInfo;


}
}

#endif // LLVM_LIB_TARGET_GAMEBOY_GAMEBOYTARGETMACHINE_H