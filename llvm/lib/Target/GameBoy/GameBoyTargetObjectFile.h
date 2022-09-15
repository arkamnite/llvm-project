#ifndef LLVM_GAMEBOY_TARGET_OBJECT_FILE_H
#define LLVM_GAMEBOY_TARGET_OBJECT_FILE_H

#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"

namespace llvm {

class GameBoyTargetObjectFile : public TargetLoweringObjectFileELF {
    typedef TargetLoweringObjectFileELF Base;

public:
    void Initialize(MCContext &ctx, const TargetMachine &TM) override;

    MCSection *SelectSectionForGlobal(const GlobalObject *GO, SectionKind Kind,
                                        const TargetMachine &TM) const override;

private:
    MCSection *ProgmemDataSection;
};

} // end namespace llvm

#endif // LLVM_GAMEBOY_TARGET_OBJECT_FILE_H