#include "llvm/MC/TargetRegistry.h"

using namespace llvm;

namespace llvm {
Target &getTheGameBoyTarget() {
    static Target TheGameBoyTarget;
    return TheGameBoyTarget;
}
} // namespace llvm

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeGameBoyTargetInfo() {
    RegisterTarget<Triple::gameboy, /* HasJIT */false> X(
        getTheGameBoyTarget(), "gameboy", "Nintendo GameBoy Family", "GameBoy");
}