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

using namespace llvm;

#define GET_INSTRINFO_MC_DESC
#include "GameBoyGenInstrInfo.inc"

#define GET_SUBTARGETINFO_MC_DESC
#include "GameBoyGenSubtargetInfo.inc"

#define GET_REGINFO_MC_DESC
#include "GameBoyGenRegisterInfo.inc"

static MCAsmInfo *createGameBoyMCAsmInfo(const MCRegisterInfo &MRI,
                                       const Triple &TT,
                                       const MCTargetOptions &Options) {
  MCAsmInfo *MAI = new GameBoyELFMCAsmInfo(TT);
//   unsigned Reg = MRI.getDwarfRegNum(SP::O6, true);
//   MCCFIInstruction Inst = MCCFIInstruction::cfiDefCfa(nullptr, Reg, 0);
//   MAI->addInitialFrameState(Inst);
  return MAI;
}

static MCAsmInfo *createGameBoyV9MCAsmInfo(const MCRegisterInfo &MRI,
                                         const Triple &TT,
                                         const MCTargetOptions &Options) {
  MCAsmInfo *MAI = new GameBoyELFMCAsmInfo(TT);
//   unsigned Reg = MRI.getDwarfRegNum(SP::O6, true);
//   MCCFIInstruction Inst = MCCFIInstruction::cfiDefCfa(nullptr, Reg, 2047);
//   MAI->addInitialFrameState(Inst);
  return MAI;
}

static MCInstrInfo *createGameBoyMCInstrInfo() {
  MCInstrInfo *X = new MCInstrInfo();
  InitGameBoyMCInstrInfo(X);
  return X;
}

static MCRegisterInfo *createGameBoyMCRegisterInfo(const Triple &TT) {
  MCRegisterInfo *X = new MCRegisterInfo();
//   InitGameBoyMCRegisterInfo(X, SP::O7);
  return X;
}

static MCSubtargetInfo *
createGameBoyMCSubtargetInfo(const Triple &TT, StringRef CPU, StringRef FS) {
  if (CPU.empty())
    CPU = (TT.getArch() == Triple::sparcv9) ? "v9" : "v8";
  return createGameBoyMCSubtargetInfoImpl(TT, CPU, /*TuneCPU*/ CPU, FS);
}

static MCTargetStreamer *
createObjectTargetStreamer(MCStreamer &S, const MCSubtargetInfo &STI) {
  return new GameBoyTargetELFStreamer(S);
}

static MCTargetStreamer *createTargetAsmStreamer(MCStreamer &S,
                                                 formatted_raw_ostream &OS,
                                                 MCInstPrinter *InstPrint,
                                                 bool isVerboseAsm) {
  return new GameBoyTargetAsmStreamer(S, OS);
}

static MCInstPrinter *createGameBoyMCInstPrinter(const Triple &T,
                                               unsigned SyntaxVariant,
                                               const MCAsmInfo &MAI,
                                               const MCInstrInfo &MII,
                                               const MCRegisterInfo &MRI) {
  return new GameBoyInstPrinter(MAI, MII, MRI);
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
