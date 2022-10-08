//===-- GameBoyELFObjectWriter.cpp - GameBoy ELF Writer ---------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/GameBoyFixupKinds.h"
#include "MCTargetDesc/GameBoyMCExpr.h"
#include "MCTargetDesc/GameBoyMCTargetDesc.h"

#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSection.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Support/ErrorHandling.h"

namespace llvm {

/// Writes GameBoy machine code into an ELF32 object file.
class GameBoyELFObjectWriter : public MCELFObjectTargetWriter {
public:
  GameBoyELFObjectWriter(uint8_t OSABI);

  virtual ~GameBoyELFObjectWriter() = default;

  unsigned getRelocType(MCContext &Ctx, const MCValue &Target,
                        const MCFixup &Fixup, bool IsPCRel) const override;
};

GameBoyELFObjectWriter::GameBoyELFObjectWriter(uint8_t OSABI)
    : MCELFObjectTargetWriter(false, OSABI, ELF::EM_GameBoy, true) {}

unsigned GameBoyELFObjectWriter::getRelocType(MCContext &Ctx, const MCValue &Target,
                                          const MCFixup &Fixup,
                                          bool IsPCRel) const {
  MCSymbolRefExpr::VariantKind Modifier = Target.getAccessVariant();
  switch ((unsigned)Fixup.getKind()) {
  case FK_Data_1:
    switch (Modifier) {
    default:
      llvm_unreachable("Unsupported Modifier");
    case MCSymbolRefExpr::VK_None:
      return ELF::R_GameBoy_8;
    case MCSymbolRefExpr::VK_GameBoy_DIFF8:
      return ELF::R_GameBoy_DIFF8;
    case MCSymbolRefExpr::VK_GameBoy_LO8:
      return ELF::R_GameBoy_8_LO8;
    case MCSymbolRefExpr::VK_GameBoy_HI8:
      return ELF::R_GameBoy_8_HI8;
    case MCSymbolRefExpr::VK_GameBoy_HLO8:
      return ELF::R_GameBoy_8_HLO8;
    }
  case FK_Data_4:
    switch (Modifier) {
    default:
      llvm_unreachable("Unsupported Modifier");
    case MCSymbolRefExpr::VK_None:
      return ELF::R_GameBoy_32;
    case MCSymbolRefExpr::VK_GameBoy_DIFF32:
      return ELF::R_GameBoy_DIFF32;
    }
  case FK_Data_2:
    switch (Modifier) {
    default:
      llvm_unreachable("Unsupported Modifier");
    case MCSymbolRefExpr::VK_None:
      return ELF::R_GameBoy_16;
    case MCSymbolRefExpr::VK_GameBoy_NONE:
    case MCSymbolRefExpr::VK_GameBoy_PM:
      return ELF::R_GameBoy_16_PM;
    case MCSymbolRefExpr::VK_GameBoy_DIFF16:
      return ELF::R_GameBoy_DIFF16;
    }
  case GameBoy::fixup_32:
    return ELF::R_GameBoy_32;
  case GameBoy::fixup_7_pcrel:
    return ELF::R_GameBoy_7_PCREL;
  case GameBoy::fixup_13_pcrel:
    return ELF::R_GameBoy_13_PCREL;
  case GameBoy::fixup_16:
    return ELF::R_GameBoy_16;
  case GameBoy::fixup_16_pm:
    return ELF::R_GameBoy_16_PM;
  case GameBoy::fixup_lo8_ldi:
    return ELF::R_GameBoy_LO8_LDI;
  case GameBoy::fixup_hi8_ldi:
    return ELF::R_GameBoy_HI8_LDI;
  case GameBoy::fixup_hh8_ldi:
    return ELF::R_GameBoy_HH8_LDI;
  case GameBoy::fixup_lo8_ldi_neg:
    return ELF::R_GameBoy_LO8_LDI_NEG;
  case GameBoy::fixup_hi8_ldi_neg:
    return ELF::R_GameBoy_HI8_LDI_NEG;
  case GameBoy::fixup_hh8_ldi_neg:
    return ELF::R_GameBoy_HH8_LDI_NEG;
  case GameBoy::fixup_lo8_ldi_pm:
    return ELF::R_GameBoy_LO8_LDI_PM;
  case GameBoy::fixup_hi8_ldi_pm:
    return ELF::R_GameBoy_HI8_LDI_PM;
  case GameBoy::fixup_hh8_ldi_pm:
    return ELF::R_GameBoy_HH8_LDI_PM;
  case GameBoy::fixup_lo8_ldi_pm_neg:
    return ELF::R_GameBoy_LO8_LDI_PM_NEG;
  case GameBoy::fixup_hi8_ldi_pm_neg:
    return ELF::R_GameBoy_HI8_LDI_PM_NEG;
  case GameBoy::fixup_hh8_ldi_pm_neg:
    return ELF::R_GameBoy_HH8_LDI_PM_NEG;
  case GameBoy::fixup_call:
    return ELF::R_GameBoy_CALL;
  case GameBoy::fixup_ldi:
    return ELF::R_GameBoy_LDI;
  case GameBoy::fixup_6:
    return ELF::R_GameBoy_6;
  case GameBoy::fixup_6_adiw:
    return ELF::R_GameBoy_6_ADIW;
  case GameBoy::fixup_ms8_ldi:
    return ELF::R_GameBoy_MS8_LDI;
  case GameBoy::fixup_ms8_ldi_neg:
    return ELF::R_GameBoy_MS8_LDI_NEG;
  case GameBoy::fixup_lo8_ldi_gs:
    return ELF::R_GameBoy_LO8_LDI_GS;
  case GameBoy::fixup_hi8_ldi_gs:
    return ELF::R_GameBoy_HI8_LDI_GS;
  case GameBoy::fixup_8:
    return ELF::R_GameBoy_8;
  case GameBoy::fixup_8_lo8:
    return ELF::R_GameBoy_8_LO8;
  case GameBoy::fixup_8_hi8:
    return ELF::R_GameBoy_8_HI8;
  case GameBoy::fixup_8_hlo8:
    return ELF::R_GameBoy_8_HLO8;
  case GameBoy::fixup_diff8:
    return ELF::R_GameBoy_DIFF8;
  case GameBoy::fixup_diff16:
    return ELF::R_GameBoy_DIFF16;
  case GameBoy::fixup_diff32:
    return ELF::R_GameBoy_DIFF32;
  case GameBoy::fixup_lds_sts_16:
    return ELF::R_GameBoy_LDS_STS_16;
  case GameBoy::fixup_port6:
    return ELF::R_GameBoy_PORT6;
  case GameBoy::fixup_port5:
    return ELF::R_GameBoy_PORT5;
  default:
    llvm_unreachable("invalid fixup kind!");
  }
}

std::unique_ptr<MCObjectTargetWriter> createGameBoyELFObjectWriter(uint8_t OSABI) {
  return std::make_unique<GameBoyELFObjectWriter>(OSABI);
}

} // end of namespace llvm
