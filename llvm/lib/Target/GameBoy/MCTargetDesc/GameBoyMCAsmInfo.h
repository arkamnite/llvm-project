//===-- GameBoyMCAsmInfo.h - GameBoy asm properties ---------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the declaration of the GameBoyMCAsmInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_GameBoy_ASM_INFO_H
#define LLVM_GameBoy_ASM_INFO_H

#include "llvm/MC/MCAsmInfo.h"

namespace llvm {

class Triple;

/// Specifies the format of GameBoy assembly files.
class GameBoyMCAsmInfo : public MCAsmInfo {
public:
  explicit GameBoyMCAsmInfo(const Triple &TT, const MCTargetOptions &Options);
};

} // end namespace llvm

#endif // LLVM_GameBoy_ASM_INFO_H
