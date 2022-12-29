//===-- GameBoyMCExpr.cpp - GameBoy specific MC expression classes ----------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "GameBoyMCExpr.h"

#include "llvm/MC/MCAsmLayout.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCValue.h"

namespace llvm {

namespace {

const struct ModifierEntry {
  const char *const Spelling;
  GameBoyMCExpr::VariantKind VariantKind;
} ModifierNames[] = {
    {"lo8", GameBoyMCExpr::VK_GameBoy_LO8},       {"hi8", GameBoyMCExpr::VK_GameBoy_HI8},
    {"hh8", GameBoyMCExpr::VK_GameBoy_HH8}, // synonym with hlo8
    {"hlo8", GameBoyMCExpr::VK_GameBoy_HH8},      {"hhi8", GameBoyMCExpr::VK_GameBoy_HHI8},

    {"pm", GameBoyMCExpr::VK_GameBoy_PM},         {"pm_lo8", GameBoyMCExpr::VK_GameBoy_PM_LO8},
    {"pm_hi8", GameBoyMCExpr::VK_GameBoy_PM_HI8}, {"pm_hh8", GameBoyMCExpr::VK_GameBoy_PM_HH8},

    {"lo8_gs", GameBoyMCExpr::VK_GameBoy_LO8_GS}, {"hi8_gs", GameBoyMCExpr::VK_GameBoy_HI8_GS},
    {"gs", GameBoyMCExpr::VK_GameBoy_GS},
};

} // end of anonymous namespace

const GameBoyMCExpr *GameBoyMCExpr::create(VariantKind Kind, const MCExpr *Expr,
                                   bool Negated, MCContext &Ctx) {
  return new (Ctx) GameBoyMCExpr(Kind, Expr, Negated);
}

void GameBoyMCExpr::printImpl(raw_ostream &OS, const MCAsmInfo *MAI) const {
  assert(Kind != VK_GameBoy_None);

  if (isNegated())
    OS << '-';

  OS << getName() << '(';
  getSubExpr()->print(OS, MAI);
  OS << ')';
}

bool GameBoyMCExpr::evaluateAsConstant(int64_t &Result) const {
  MCValue Value;

  bool isRelocatable =
      getSubExpr()->evaluateAsRelocatable(Value, nullptr, nullptr);

  if (!isRelocatable)
    return false;

  if (Value.isAbsolute()) {
    Result = evaluateAsInt64(Value.getConstant());
    return true;
  }

  return false;
}

bool GameBoyMCExpr::evaluateAsRelocatableImpl(MCValue &Result,
                                          const MCAsmLayout *Layout,
                                          const MCFixup *Fixup) const {
  MCValue Value;
  bool isRelocatable = SubExpr->evaluateAsRelocatable(Value, Layout, Fixup);

  if (!isRelocatable)
    return false;

  if (Value.isAbsolute()) {
    Result = MCValue::get(evaluateAsInt64(Value.getConstant()));
  } else {
    if (!Layout)
      return false;

    MCContext &Context = Layout->getAssembler().getContext();
    const MCSymbolRefExpr *Sym = Value.getSymA();
    MCSymbolRefExpr::VariantKind Modifier = Sym->getKind();
    if (Modifier != MCSymbolRefExpr::VK_None)
      return false;
    if (Kind == VK_GameBoy_PM) {
      Modifier = MCSymbolRefExpr::VK_AVR_PM;
    }

    Sym = MCSymbolRefExpr::create(&Sym->getSymbol(), Modifier, Context);
    Result = MCValue::get(Sym, Value.getSymB(), Value.getConstant());
  }

  return true;
}

int64_t GameBoyMCExpr::evaluateAsInt64(int64_t Value) const {
  if (Negated)
    Value *= -1;

  switch (Kind) {
  case GameBoyMCExpr::VK_GameBoy_LO8:
    Value &= 0xff;
    break;
  case GameBoyMCExpr::VK_GameBoy_HI8:
    Value &= 0xff00;
    Value >>= 8;
    break;
  case GameBoyMCExpr::VK_GameBoy_HH8:
    Value &= 0xff0000;
    Value >>= 16;
    break;
  case GameBoyMCExpr::VK_GameBoy_HHI8:
    Value &= 0xff000000;
    Value >>= 24;
    break;
  case GameBoyMCExpr::VK_GameBoy_PM_LO8:
  case GameBoyMCExpr::VK_GameBoy_LO8_GS:
    Value >>= 1; // Program memory addresses must always be shifted by one.
    Value &= 0xff;
    break;
  case GameBoyMCExpr::VK_GameBoy_PM_HI8:
  case GameBoyMCExpr::VK_GameBoy_HI8_GS:
    Value >>= 1; // Program memory addresses must always be shifted by one.
    Value &= 0xff00;
    Value >>= 8;
    break;
  case GameBoyMCExpr::VK_GameBoy_PM_HH8:
    Value >>= 1; // Program memory addresses must always be shifted by one.
    Value &= 0xff0000;
    Value >>= 16;
    break;
  case GameBoyMCExpr::VK_GameBoy_PM:
  case GameBoyMCExpr::VK_GameBoy_GS:
    Value >>= 1; // Program memory addresses must always be shifted by one.
    break;

  case GameBoyMCExpr::VK_GameBoy_None:
    llvm_unreachable("Uninitialized expression.");
  }
  return static_cast<uint64_t>(Value) & 0xff;
}

GameBoy::Fixups GameBoyMCExpr::getFixupKind() const {
  GameBoy::Fixups Kind = GameBoy::Fixups::LastTargetFixupKind;

  switch (getKind()) {
  case VK_GameBoy_LO8:
    Kind = isNegated() ? GameBoy::fixup_lo8_ldi_neg : GameBoy::fixup_lo8_ldi;
    break;
  case VK_GameBoy_HI8:
    Kind = isNegated() ? GameBoy::fixup_hi8_ldi_neg : GameBoy::fixup_hi8_ldi;
    break;
  case VK_GameBoy_HH8:
    Kind = isNegated() ? GameBoy::fixup_hh8_ldi_neg : GameBoy::fixup_hh8_ldi;
    break;
  case VK_GameBoy_HHI8:
    Kind = isNegated() ? GameBoy::fixup_ms8_ldi_neg : GameBoy::fixup_ms8_ldi;
    break;

  case VK_GameBoy_PM_LO8:
    Kind = isNegated() ? GameBoy::fixup_lo8_ldi_pm_neg : GameBoy::fixup_lo8_ldi_pm;
    break;
  case VK_GameBoy_PM_HI8:
    Kind = isNegated() ? GameBoy::fixup_hi8_ldi_pm_neg : GameBoy::fixup_hi8_ldi_pm;
    break;
  case VK_GameBoy_PM_HH8:
    Kind = isNegated() ? GameBoy::fixup_hh8_ldi_pm_neg : GameBoy::fixup_hh8_ldi_pm;
    break;
  case VK_GameBoy_PM:
  case VK_GameBoy_GS:
    Kind = GameBoy::fixup_16_pm;
    break;
  case VK_GameBoy_LO8_GS:
    Kind = GameBoy::fixup_lo8_ldi_gs;
    break;
  case VK_GameBoy_HI8_GS:
    Kind = GameBoy::fixup_hi8_ldi_gs;
    break;

  case VK_GameBoy_None:
    llvm_unreachable("Uninitialized expression");
  }

  return Kind;
}

void GameBoyMCExpr::visitUsedExpr(MCStreamer &Streamer) const {
  Streamer.visitUsedExpr(*getSubExpr());
}

const char *GameBoyMCExpr::getName() const {
  const auto &Modifier =
      llvm::find_if(ModifierNames, [this](ModifierEntry const &Mod) {
        return Mod.VariantKind == Kind;
      });

  if (Modifier != std::end(ModifierNames)) {
    return Modifier->Spelling;
  }
  return nullptr;
}

GameBoyMCExpr::VariantKind GameBoyMCExpr::getKindByName(StringRef Name) {
  const auto &Modifier =
      llvm::find_if(ModifierNames, [&Name](ModifierEntry const &Mod) {
        return Mod.Spelling == Name;
      });

  if (Modifier != std::end(ModifierNames)) {
    return Modifier->VariantKind;
  }
  return VK_GameBoy_None;
}

} // end of namespace llvm
