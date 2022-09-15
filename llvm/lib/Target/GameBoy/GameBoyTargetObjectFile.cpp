#include "GameBoyTargetObjectFile.h"
#include "GameBoyTargetMachine.h"

#include "llvm/BinaryFormat/ELF.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/GlobalValue.h"
#include "llvm/IR/Mangler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCSectionELF.h"

#include "GameBoy.h"

namespace llvm {
void GameBoyTargetObjectFile::Initialize(MCContext &Ctx, const TargetMachine &TM)
{
    Base::Initialize(Ctx, TM);
}

MCSection *GameBoyTargetObjectFile::SelectSectionForGlobal(
    const GlobalObject *GO, SectionKind Kind, const TargetMachine &TM
) const {
    return Base::SelectSectionForGlobal(GO, Kind, TM);
}


} // namespace llvm