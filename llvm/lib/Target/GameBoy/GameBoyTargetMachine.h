// GameBoyTargetMachine.h - Define TargetMachine for Game Boy
// This file declares the Game boy specific subclass of TargetMachine
// 12/06/22 File created

#ifndef LLVM_LIB_TARGET_GAMEBOY_GAMEBOYTARGETMACHINE_H
#define LLVM_LIB_TARGET_GAMEBOY_GAMEBOYTARGETMACHINE_H

#include "GameBoyInstrInfo.h"
#include "llvm/Target/TargetMachine.h"
// #include "GameBoySubtarget.h" Will be used for Original DMG Unit

namespace llvm {

class Module;

class GameBoyTargetMachine : public LLVMTargetMachine {
    const DataLayout DataLayout;
    // GameBoySubtarget Subtarget;
    GameBoyInstrInfo InstrInfo;
        
}

}

#endif