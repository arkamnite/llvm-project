#include "MCTargetDesc/GameBoyAsmBackend.h"
#include "MCTargetDesc/GameBoyMCTargetDesc.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDirectives.h"

namespace llvm {

std::unique_ptr<MCObjectTargetWriter> 
GameBoyAsmBackend::createObjectTargetWriter() const {
    return createGameBoyELFObjectWriter(MCELFObjectTargetWriter::getOSABI(OSType));
}

MCAsmBackend *createGameBoyAsmBackend(const Target &T, const MCSubtargetInfo &STI,
      const MCRegisterInfo &MRI,
      const llvm::MCTargetOptions &TO) {
    return new GameBoyAsmBackend(STI.getTargetTriple().getOS());
}

} // end namespace llvm