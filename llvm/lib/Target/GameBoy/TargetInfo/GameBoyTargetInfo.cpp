//===-- GameBoyTargetInfo.cpp - GameBoy Target Implementation ---------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "TargetInfo/GameBoyTargetInfo.h"
#include "llvm/MC/TargetRegistry.h"
namespace llvm {
Target &getTheGameBoyTarget() {
  static Target TheGameBoyTarget;
  return TheGameBoyTarget;
}
} // namespace llvm

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeGameBoyTargetInfo() {
  llvm::RegisterTarget<llvm::Triple::gameboy> X(llvm::getTheGameBoyTarget(), "Game Boy",
                                            "Nintendo Game Boy", "Game Boy");
}
