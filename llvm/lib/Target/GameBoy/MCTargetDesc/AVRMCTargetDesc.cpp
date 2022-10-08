//===-- GameBoyMCTargetDesc.cpp - GameBoy Target Descriptions ---------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file provides GameBoy specific target descriptions.
//
//===----------------------------------------------------------------------===//

#include "GameBoyMCTargetDesc.h"
#include "GameBoyELFStreamer.h"
#include "GameBoyInstPrinter.h"
#include "GameBoyMCAsmInfo.h"
#include "GameBoyMCELFStreamer.h"
#include "GameBoyTargetStreamer.h"
#include "TargetInfo/GameBoyTargetInfo.h"

#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCELFStreamer.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/TargetRegistry.h"

#define GET_INSTRINFO_MC_DESC
#include "GameBoyGenInstrInfo.inc"

#define GET_SUBTARGETINFO_MC_DESC
#include "GameBoyGenSubtargetInfo.inc"

#define GET_REGINFO_MC_DESC
#include "GameBoyGenRegisterInfo.inc"

using namespace llvm;

MCInstrInfo *llvm::createGameBoyMCInstrInfo() {
  MCInstrInfo *X = new MCInstrInfo();
  InitGameBoyMCInstrInfo(X);

  return X;
}

static MCRegisterInfo *createGameBoyMCRegisterInfo(const Triple &TT) {
  MCRegisterInfo *X = new MCRegisterInfo();
  InitGameBoyMCRegisterInfo(X, 0);

  return X;
}

static MCSubtargetInfo *createGameBoyMCSubtargetInfo(const Triple &TT,
                                                 StringRef CPU, StringRef FS) {
  return createGameBoyMCSubtargetInfoImpl(TT, CPU, /*TuneCPU*/ CPU, FS);
}

static MCInstPrinter *createGameBoyMCInstPrinter(const Triple &T,
                                             unsigned SyntaxVariant,
                                             const MCAsmInfo &MAI,
                                             const MCInstrInfo &MII,
                                             const MCRegisterInfo &MRI) {
  if (SyntaxVariant == 0) {
    return new GameBoyInstPrinter(MAI, MII, MRI);
  }

  return nullptr;
}

static MCStreamer *createMCStreamer(const Triple &T, MCContext &Context,
                                    std::unique_ptr<MCAsmBackend> &&MAB,
                                    std::unique_ptr<MCObjectWriter> &&OW,
                                    std::unique_ptr<MCCodeEmitter> &&Emitter,
                                    bool RelaxAll) {
  return createELFStreamer(Context, std::move(MAB), std::move(OW),
                           std::move(Emitter), RelaxAll);
}

static MCTargetStreamer *
createGameBoyObjectTargetStreamer(MCStreamer &S, const MCSubtargetInfo &STI) {
  return new GameBoyELFStreamer(S, STI);
}

static MCTargetStreamer *createMCAsmTargetStreamer(MCStreamer &S,
                                                   formatted_raw_ostream &OS,
                                                   MCInstPrinter *InstPrint,
                                                   bool isVerboseAsm) {
  return new GameBoyTargetAsmStreamer(S);
}

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeGameBoyTargetMC() {
  // Register the MC asm info.
  RegisterMCAsmInfo<GameBoyMCAsmInfo> X(getTheGameBoyTarget());

  // Register the MC instruction info.
  TargetRegistry::RegisterMCInstrInfo(getTheGameBoyTarget(), createGameBoyMCInstrInfo);

  // Register the MC register info.
  TargetRegistry::RegisterMCRegInfo(getTheGameBoyTarget(), createGameBoyMCRegisterInfo);

  // Register the MC subtarget info.
  TargetRegistry::RegisterMCSubtargetInfo(getTheGameBoyTarget(),
                                          createGameBoyMCSubtargetInfo);

  // Register the MCInstPrinter.
  TargetRegistry::RegisterMCInstPrinter(getTheGameBoyTarget(),
                                        createGameBoyMCInstPrinter);

  // Register the MC Code Emitter
  TargetRegistry::RegisterMCCodeEmitter(getTheGameBoyTarget(),
                                        createGameBoyMCCodeEmitter);

  // Register the obj streamer
  TargetRegistry::RegisterELFStreamer(getTheGameBoyTarget(), createMCStreamer);

  // Register the obj target streamer.
  TargetRegistry::RegisterObjectTargetStreamer(getTheGameBoyTarget(),
                                               createGameBoyObjectTargetStreamer);

  // Register the asm target streamer.
  TargetRegistry::RegisterAsmTargetStreamer(getTheGameBoyTarget(),
                                            createMCAsmTargetStreamer);

  // Register the asm backend (as little endian).
  TargetRegistry::RegisterMCAsmBackend(getTheGameBoyTarget(), createGameBoyAsmBackend);
}
