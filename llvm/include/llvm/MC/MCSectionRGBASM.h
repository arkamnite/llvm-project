#ifndef LLVM_MC_MCSECTIONRGBASM_H
#define LLVM_MC_MCSECTIONRGBASM_H

#include "llvm/ADT/PointerIntPair.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/MC/MCSection.h"
#include "llvm/MC/MCSymbolELF.h"
#include "llvm/MC/SectionKind.h"

namespace llvm {

class MCSectionRGBASM final : public MCSection {

    /// @brief An RGBASM section type refers to whether it is
    /// designated as living in RAM, ROM, or WRAM.
    unsigned Type;

    /// @brief The starting address for this section.
    unsigned StartAddr;
private:
    friend class MCContext;

    // MCSectionRGBASM(StringReg Name, )
};
}


#endif // LLVM_MC_MCSECTIONRGBASM_H