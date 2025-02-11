#include "GameBoyELFStreamer.h"

#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/SubtargetFeature.h"
#include "llvm/Support/FormattedStream.h"

#include "GameBoyMCTargetDesc.h"

namespace llvm {

static unsigned getEFlagsForFeatureSet(const FeatureBitset &Features) {
  unsigned EFlags = 0;

  // Set architecture
  if (Features[GameBoy::ELFArchGameBoy1])
    EFlags |= ELF::EF_AVR_ARCH_AVR1;
  else if (Features[GameBoy::ELFArchGameBoy2])
    EFlags |= ELF::EF_AVR_ARCH_AVR2;
  else if (Features[GameBoy::ELFArchGameBoy25])
    EFlags |= ELF::EF_AVR_ARCH_AVR25;
  else if (Features[GameBoy::ELFArchGameBoy3])
    EFlags |= ELF::EF_AVR_ARCH_AVR3;
  else if (Features[GameBoy::ELFArchGameBoy31])
    EFlags |= ELF::EF_AVR_ARCH_AVR31;
  else if (Features[GameBoy::ELFArchGameBoy35])
    EFlags |= ELF::EF_AVR_ARCH_AVR35;
  else if (Features[GameBoy::ELFArchGameBoy4])
    EFlags |= ELF::EF_AVR_ARCH_AVR4;
  else if (Features[GameBoy::ELFArchGameBoy5])
    EFlags |= ELF::EF_AVR_ARCH_AVR5;
  else if (Features[GameBoy::ELFArchGameBoy51])
    EFlags |= ELF::EF_AVR_ARCH_AVR51;
  else if (Features[GameBoy::ELFArchGameBoy6])
    EFlags |= ELF::EF_AVR_ARCH_AVR6;
  else if (Features[GameBoy::ELFArchTiny])
    EFlags |= ELF::EF_AVR_ARCH_AVRTINY;
  else if (Features[GameBoy::ELFArchXMEGA1])
    EFlags |= ELF::EF_AVR_ARCH_XMEGA1;
  else if (Features[GameBoy::ELFArchXMEGA2])
    EFlags |= ELF::EF_AVR_ARCH_XMEGA2;
  else if (Features[GameBoy::ELFArchXMEGA3])
    EFlags |= ELF::EF_AVR_ARCH_XMEGA3;
  else if (Features[GameBoy::ELFArchXMEGA4])
    EFlags |= ELF::EF_AVR_ARCH_XMEGA4;
  else if (Features[GameBoy::ELFArchXMEGA5])
    EFlags |= ELF::EF_AVR_ARCH_XMEGA5;
  else if (Features[GameBoy::ELFArchXMEGA6])
    EFlags |= ELF::EF_AVR_ARCH_XMEGA6;
  else if (Features[GameBoy::ELFArchXMEGA7])
    EFlags |= ELF::EF_AVR_ARCH_XMEGA7;

  return EFlags;
}

GameBoyELFStreamer::GameBoyELFStreamer(MCStreamer &S, const MCSubtargetInfo &STI)
    : GameBoyTargetStreamer(S) {

  MCAssembler &MCA = getStreamer().getAssembler();
  unsigned EFlags = MCA.getELFHeaderEFlags();

  EFlags |= getEFlagsForFeatureSet(STI.getFeatureBits());

  MCA.setELFHeaderEFlags(EFlags);
}

} // end namespace llvm
