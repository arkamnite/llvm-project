//===-- GameBoyTargetMachine.cpp - Define TargetMachine for GameBoy ---------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the GameBoy specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//

#include "GameBoyTargetMachine.h"

#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/IR/Module.h"
#include "llvm/MC/TargetRegistry.h"

#include "GameBoy.h"
#include "GameBoyTargetObjectFile.h"
#include "MCTargetDesc/GameBoyMCTargetDesc.h"
#include "TargetInfo/GameBoyTargetInfo.h"

namespace llvm {

static const char *GameBoyDataLayout =
    "e-p:16:8-i8:8-i16:16-i32:16-i64:16-f32:8-f64:8-n8-a:8";

/// Processes a CPU name.
static StringRef getCPU(StringRef CPU) {
  if (CPU.empty() || CPU == "generic") {
    return "GameBoy2";
  }

  return CPU;
}

static Reloc::Model getEffectiveRelocModel(Optional<Reloc::Model> RM) {
  return RM.getValueOr(Reloc::Static);
}

GameBoyTargetMachine::GameBoyTargetMachine(const Target &T, const Triple &TT,
                                   StringRef CPU, StringRef FS,
                                   const TargetOptions &Options,
                                   Optional<Reloc::Model> RM,
                                   Optional<CodeModel::Model> CM,
                                   CodeGenOpt::Level OL, bool JIT)
    : LLVMTargetMachine(T, GameBoyDataLayout, TT, getCPU(CPU), FS, Options,
                        getEffectiveRelocModel(RM),
                        getEffectiveCodeModel(CM, CodeModel::Small), OL),
      SubTarget(TT, std::string(getCPU(CPU)), std::string(FS), *this) {
  this->TLOF = std::make_unique<GameBoyTargetObjectFile>();
  initAsmInfo();
}

namespace {
/// GameBoy Code Generator Pass Configuration Options.
class GameBoyPassConfig : public TargetPassConfig {
public:
  GameBoyPassConfig(GameBoyTargetMachine &TM, PassManagerBase &PM)
      : TargetPassConfig(TM, PM) {}

  GameBoyTargetMachine &getGameBoyTargetMachine() const {
    return getTM<GameBoyTargetMachine>();
  }

  void addIRPasses() override;
  bool addInstSelector() override;
  void addPreSched2() override;
  void addPreEmitPass() override;
};
} // namespace

TargetPassConfig *GameBoyTargetMachine::createPassConfig(PassManagerBase &PM) {
  return new GameBoyPassConfig(*this, PM);
}

void GameBoyPassConfig::addIRPasses() {
  // Expand instructions like
  //   %result = shl i32 %n, %amount
  // to a loop so that library calls are avoided.
  addPass(createGameBoyShiftExpandPass());

  TargetPassConfig::addIRPasses();
}

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeGameBoyTarget() {
  // Register the target.
  RegisterTargetMachine<GameBoyTargetMachine> X(getTheGameBoyTarget());

  auto &PR = *PassRegistry::getPassRegistry();
  initializeGameBoyExpandPseudoPass(PR);
  initializeGameBoyShiftExpandPass(PR);
}

const GameBoySubtarget *GameBoyTargetMachine::getSubtargetImpl() const {
  return &SubTarget;
}

const GameBoySubtarget *GameBoyTargetMachine::getSubtargetImpl(const Function &) const {
  return &SubTarget;
}

//===----------------------------------------------------------------------===//
// Pass Pipeline Configuration
//===----------------------------------------------------------------------===//

bool GameBoyPassConfig::addInstSelector() {
  // Install an instruction selector.
  addPass(createGameBoyISelDag(getGameBoyTargetMachine(), getOptLevel()));
  // Create the frame analyzer pass used by the PEI pass.
  addPass(createGameBoyFrameAnalyzerPass());

  return false;
}

void GameBoyPassConfig::addPreSched2() {
  addPass(createGameBoyExpandPseudoPass());
}

void GameBoyPassConfig::addPreEmitPass() {
  // Must run branch selection immediately preceding the asm printer.
  addPass(&BranchRelaxationPassID);
}

} // end of namespace llvm
