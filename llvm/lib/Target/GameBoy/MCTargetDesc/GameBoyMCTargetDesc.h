//===-- GameBoyMCTargetDesc.h - GameBoy Target Descriptions -------------*- C++ -*-===//
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

#ifndef LLVM_GameBoy_MCTARGET_DESC_H
#define LLVM_GameBoy_MCTARGET_DESC_H

#include "llvm/Support/DataTypes.h"

#include <memory>

namespace llvm {

class MCAsmBackend;
class MCCodeEmitter;
class MCContext;
class MCInstrInfo;
class MCObjectTargetWriter;
class MCRegisterInfo;
class MCSubtargetInfo;
class MCTargetOptions;
class Target;

MCInstrInfo *createGameBoyMCInstrInfo();

/// Creates a machine code emitter for GameBoy.
MCCodeEmitter *createGameBoyMCCodeEmitter(const MCInstrInfo &MCII,
                                      MCContext &Ctx);

/// Creates an assembly backend for GameBoy.
MCAsmBackend *createGameBoyAsmBackend(const Target &T, const MCSubtargetInfo &STI,
                                  const MCRegisterInfo &MRI,
                                  const llvm::MCTargetOptions &TO);

/// Creates an ELF object writer for GameBoy.
std::unique_ptr<MCObjectTargetWriter> createGameBoyELFObjectWriter(uint8_t OSABI);

} // end namespace llvm

#define GET_REGINFO_ENUM
#include "GameBoyGenRegisterInfo.inc"

#define GET_INSTRINFO_ENUM
#include "GameBoyGenInstrInfo.inc"

#define GET_SUBTARGETINFO_ENUM
#include "GameBoyGenSubtargetInfo.inc"

#endif // LLVM_GameBoy_MCTARGET_DESC_H
