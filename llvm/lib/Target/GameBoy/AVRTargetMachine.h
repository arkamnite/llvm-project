//===-- GameBoyTargetMachine.h - Define TargetMachine for GameBoy -------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the GameBoy specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_GameBoy_TARGET_MACHINE_H
#define LLVM_GameBoy_TARGET_MACHINE_H

#include "llvm/IR/DataLayout.h"
#include "llvm/Target/TargetMachine.h"

#include "GameBoyFrameLowering.h"
#include "GameBoyISelLowering.h"
#include "GameBoyInstrInfo.h"
#include "GameBoySelectionDAGInfo.h"
#include "GameBoySubtarget.h"

namespace llvm {

/// A generic GameBoy implementation.
class GameBoyTargetMachine : public LLVMTargetMachine {
public:
  GameBoyTargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                   StringRef FS, const TargetOptions &Options,
                   Optional<Reloc::Model> RM, Optional<CodeModel::Model> CM,
                   CodeGenOpt::Level OL, bool JIT);

  const GameBoySubtarget *getSubtargetImpl() const;
  const GameBoySubtarget *getSubtargetImpl(const Function &) const override;

  TargetLoweringObjectFile *getObjFileLowering() const override {
    return this->TLOF.get();
  }

  TargetPassConfig *createPassConfig(PassManagerBase &PM) override;

private:
  std::unique_ptr<TargetLoweringObjectFile> TLOF;
  GameBoySubtarget SubTarget;
};

} // end namespace llvm

#endif // LLVM_GameBoy_TARGET_MACHINE_H
