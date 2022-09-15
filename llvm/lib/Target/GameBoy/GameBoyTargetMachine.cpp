#include "GameBoy.h"
#include "GameBoyTargetMachine.h"
#include "TargetInfo/GameBoyTargetInfo.h"
#include "llvm/MC/TargetRegistry.h"

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeGameBoyTarget() {
    // Register the target.
    RegisterTargetMachine<GameBoyTargetMachine> X(getTheGameBoyTarget());
    
}

namespace llvm {
// The LLVM-DMG ABI specifies a 16-bit pointer.
// All data is aligned on an 8-bit boundary, with
// exception to all 16-bit types.
// Unsupported types are aligned on an 8-bit boundary.
static const char *GameBoyDataLayout =
    "e-p:16:8-i8:8-i16:16-i32:16-i64:16-f32:8-f64:8-n8-a:8";

static Reloc::Model getEffectiveRelocModel(Optional<Reloc::Model> RM) {
  return RM.getValueOr(Reloc::Static);
}

GameBoyTargetMachine::GameBoyTargetMachine(const Target &T, const Triple &TT, 
                    StringRef CPU, StringRef FS, 
                    const TargetOptions &Options,
                    Optional<Reloc::Model> RM, 
                    Optional<CodeModel::Model> CM,
                    CodeGenOpt::Level OL, bool JIT) 
    : LLVMTargetMachine(T, GameBoyDataLayout, TT, "gameboy", FS, Options, getEffectiveRelocModel(RM),
    getEffectiveCodeModel(CM, CodeModel::Small), 0L)            
{
    this->TLOF = std::make_unique<GameBoyObjectFile>();
    initAsmInfo();
}
}