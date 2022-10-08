#include "GameBoyMCTargetDesc.h"
#include "GameBoyInstPrinter.h"
#include "GameBoyMCAsmInfo.h"
#include "GameBoyTargetStreamer.h"
#include "TargetInfo/GameBoyTargetInfo.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/ErrorHandling.h"

#define GET_INSTRINFO_MC_DESC
#include "GameBoyInstrInfo.inc"

// #define GET_SUBTARGETINFO_MC_DESC
// #include "GameBoySubtargetInfo.inc"

#define GET_REGINFO_MC_DESC
#include "GameBoyRegisterInfo.inc"

using namespace llvm;

MCInstrInfo *llvm::createGameBoyMCInstrInfo() {
  MCInstrInfo *X = new MCInstrInfo();
  InitGameBoyMCInstrInfo(X);

  return X;
}

static MCRegisterInfo *createGameBoyMCRegisterINfo(const Triple &TT) {
  MCRegisterInfo *X = new MCRegisterInfo();
  InitGameBoyMCRegisterInfo(X, 0);

  return X;
}

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeGameBoyTargetMC() {
  // Register the MC asm info.
  RegisterMCAsmInfoFn X(getTheGameBoyTarget(), createGameBoyMCAsmInfo);

  for (Target *T :
       {&getTheGameBoyTarget()} {
    // Register the MC instruction info.
    TargetRegistry::RegisterMCInstrInfo(*T, createGameBoyMCInstrInfo);

    // Register the MC register info.
    TargetRegistry::RegisterMCRegInfo(*T, createGameBoyMCRegisterInfo);

    // Register the MC subtarget info.
    TargetRegistry::RegisterMCSubtargetInfo(*T, createGameBoyMCSubtargetInfo);

    // Register the MC Code Emitter.
    TargetRegistry::RegisterMCCodeEmitter(*T, createGameBoyMCCodeEmitter);

    // Register the asm backend.
    TargetRegistry::RegisterMCAsmBackend(*T, createGameBoyAsmBackend);

    // Register the object target streamer.
    TargetRegistry::RegisterObjectTargetStreamer(*T,
                                                 createObjectTargetStreamer);

    // Register the asm streamer.
    TargetRegistry::RegisterAsmTargetStreamer(*T, createTargetAsmStreamer);

    // Register the MCInstPrinter
    TargetRegistry::RegisterMCInstPrinter(*T, createGameBoyMCInstPrinter);
  }
}
