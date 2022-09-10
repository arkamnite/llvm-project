#ifndef LLVM_LIB_TARGET_GAMEBOY_MCTARGETDESC_GAMEBOYMCTARGETDESC_H
#define LLVM_LIB_TARGET_GAMEBOY_MCTARGETDESC_GAMEBOYMCTARGETDESC_H

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

MCCodeEmitter *createGameBoyMCCodeEmitter(const MCInstrInfo &MCII,
                                        MCContext &Ctx);
MCAsmBackend *createGameBoyAsmBackend(const Target &T, const MCSubtargetInfo &STI,
                                    const MCRegisterInfo &MRI,
                                    const MCTargetOptions &Options);
std::unique_ptr<MCObjectTargetWriter> createGameBoyELFObjectWriter(bool Is64Bit,
                                                                 uint8_t OSABI);
} // End llvm namespace

// Defines symbolic names for GameBoy registers.  This defines a mapping from
// register name to register number.
//
#define GET_REGINFO_ENUM
#include "GameBoyGenRegisterInfo.inc"

// Defines symbolic names for the GameBoy instructions.
//
#define GET_INSTRINFO_ENUM
#include "GameBoyGenInstrInfo.inc"

#define GET_SUBTARGETINFO_ENUM
#include "GameBoyGenSubtargetInfo.inc"

#endif
