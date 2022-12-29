//===-- GameBoySubtarget.cpp - GameBoy Subtarget Information ----------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the GameBoy specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#include "GameBoySubtarget.h"

#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/TargetRegistry.h"

#include "GameBoy.h"
#include "GameBoyTargetMachine.h"
#include "MCTargetDesc/GameBoyMCTargetDesc.h"

#define DEBUG_TYPE "GameBoy-subtarget"

#define GET_SUBTARGETINFO_TARGET_DESC
#define GET_SUBTARGETINFO_CTOR
#include "GameBoyGenSubtargetInfo.inc"

namespace llvm {

GameBoySubtarget::GameBoySubtarget(const Triple &TT, const std::string &CPU,
                           const std::string &FS, const GameBoyTargetMachine &TM)
    : GameBoyGenSubtargetInfo(TT, CPU, /*TuneCPU*/ CPU, FS), ELFArch(0),

      // Subtarget features
      m_hasSRAM(false), m_hasJMPCALL(false), m_hasIJMPCALL(false),
      m_hasEIJMPCALL(false), m_hasADDSUBIW(false), m_hasSmallStack(false),
      m_hasMOVW(false), m_hasLPM(false), m_hasLPMX(false), m_hasELPM(false),
      m_hasELPMX(false), m_hasSPM(false), m_hasSPMX(false), m_hasDES(false),
      m_supportsRMW(false), m_supportsMultiplication(false), m_hasBREAK(false),
      m_hasTinyEncoding(false), m_hasMemMappedGPR(false),
      m_FeatureSetDummy(false),

      TLInfo(TM, initializeSubtargetDependencies(CPU, FS, TM)) {
  // Parse features string.
  ParseSubtargetFeatures(CPU, /*TuneCPU*/ CPU, FS);
}

GameBoySubtarget &
GameBoySubtarget::initializeSubtargetDependencies(StringRef CPU, StringRef FS,
                                              const TargetMachine &TM) {
  // Parse features string.
  ParseSubtargetFeatures(CPU, /*TuneCPU*/ CPU, FS);
  return *this;
}

} // end of namespace llvm
