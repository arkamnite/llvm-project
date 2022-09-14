#include "GameBoyTargetMachine.h"
#include "GameBoy.h"
#include "TargetInfo/GameBoyTargetInfo.h"

/* Looks to be for TargetIndependent CodeGen.
extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeGameBoyTarget() {
    RegisterTargetMachine<GameBoyTargetMachine> X(getTheGameBoyTarget());
    auto *PR = PassRegistry::getPassRegistry();
    initializeGlobalISel(*PR);
}
*/

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

GameBoyTargetMachine(const Target &T, const Triple &TT, 
                    StringRef CPU, StringRef FS, 
                    const TargetOptions &Options,
                    Optional<Reloc::Model> RM, 
                    Optional<CodeModel::Model> CM,
                    CodeGenOpt::Level OL, bool JIT) 
    : LLVMTargetMachine(T, GameBoyDataLayout, TT, "gameboy", FS, Options, getEffectiveRelocModel(RM),
    getEffectiveCodeModel(CM, CodeModel::Small), 0L),
    SubTarget(TT, "gameboy", std::string(FS), *this)            
{
    this->TLOF = std::make_unique<GameBoyObjectFile>();
    initAsmInfo();
}