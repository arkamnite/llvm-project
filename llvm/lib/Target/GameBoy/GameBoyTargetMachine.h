#ifndef LLVM_LIB_TARGET_GAMEBOY_GAMEBOYTARGETMACHINE_H
#define LLVM_LIB_TARGET_GAMEBOY_GAMEBOYTARGETMACHINE_H

// #include "GameBoySubTarget.h"
// #include "MCTargetDesc/GameBoyMCTargetDesc.h"

#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/SelectionDAGISel.h"

#include "llvm/Target/TargetMachine.h"
#include "GameBoyInstrInfo.h"
#include "GameBoyFrameLowering.h"

namespace llvm{
class Module;

class GameBoyTargetMachine : public LLVMTargetMachine {
    const DataLayout DataLayout;    // Calculate type size, alignment
    // GameBoySubTarget Subtarget;
    GameBoyInstrInfo InstrInfo;
    TargetFrameInfo FrameInfo;

protected:
    virtual const TargetAsmInfo *createTargetAsmInfo() const;

public:
    // TODO: Check the signature is correct.
    GameBoyTargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                   StringRef FS, const TargetOptions &Options,
                   Optional<Reloc::Model> RM, Optional<CodeModel::Model> CM,
                   CodeGenOpt::Level OL, bool JIT);

    ~GameBoyTargetMachine() override;

    // getInstrInfo()
    // getInstrInfo()
    // getFrameInfo()
    // getDataLayout()
    // getSubtargetImpl()
};
} // end namespace llvm

#endif // LLVM_LIB_TARGET_GAMEBOY_GAMEBOYTARGETMACHINE_H