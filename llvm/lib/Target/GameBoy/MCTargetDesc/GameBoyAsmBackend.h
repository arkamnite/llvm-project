#ifndef LLVM_GAMEBOY_ASM_BACKEND_H
#define LLVM_GAMEBOY_ASM_BACKEND_H

#include "llvm/ADT/Triple.h"
#include "llvm/MC/MCAsmBackend.h"

namespace llvm {

class MCAssembler;
class MCContext;
class MCFixupKindInfo;

/// Utilities for manipulating generated GameBoy machine code.
class GameBoyAsmBackend : public MCAsmBackend {
public:
    GameBoyAsmBackend(Triple::OSType OSType) : MCAsmBackend(support::little),
    OSType(OSType) {}

private:
    Triple::OSType OSType;
};

} // end namespace llvm

#endif // LLVM_GAMEBOY_ASM_BACKEND_H