#ifndef LLVM_LIB_TARGET_GAMEBOY_GAMEBOYCALLINGCONV_H
#define LLVM_LIB_TARGET_GAMEBOY_GAMEBOYCALLINGCONV_H

#include "MCTargetDesc/GameBoyMCTargetDesc.h"
#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/Support/MachineValueType.h"

namespace llvm {

bool RetCC_GameBoy_BUILTIN(unsigned ValNo, MVT ValVT, MVT LocVT,
               CCValAssign::LocInfo LocInfo, ISD::ArgFlagsTy ArgFlags,
               CCState &State);

/// Regular calling convention.
bool ArgCC_GameBoy_Default(unsigned ValNo, MVT ValVT, MVT LocVT,
                CCValAssign::LocInfo LocInfo, ISD::ArgFlagsTy ArgFlags,
                CCState &State);

} // namespace llvm

#endif // not LLVM_LIB_TARGET_GameBoy_GameBoyCALLINGCONV_H