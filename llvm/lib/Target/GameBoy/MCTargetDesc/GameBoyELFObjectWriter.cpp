#include "MCTargetDesc/GameBoyMCTargetDesc.h"

#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MCObjectWriter.h"
#include "llvm/MCSection.h"
#include "llvm/MCValue.h"

namespace llvm {

/// Writes GameBoy machine code into an ELF32 object file.
class GameBoyELFObjectWriter : public MCELFObjectTargetWriter {
public:
    GameBoyELFObjectWriter(uint8_t OSABI);

    virtual ~GameBoyELFObjectWriter() = default;

};

GameBoyELFObjectWriter::GameBoyELFObjectWriter(uint8_t OSABI)
    : MCELFObjectTargetWriter(false, OSABI, ELF::EM_GAMEBOY, true) {}

std::unique_ptr<GameBoyELFObjectWriter> createGameBoyELFObjectWriter(uint8_t OSABI) {
    return std::make_unique<GameBoyELFObjectWriter>(OSABI);
}
} // end namespace llvm