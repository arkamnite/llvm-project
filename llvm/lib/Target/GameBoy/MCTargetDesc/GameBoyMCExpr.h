//===-- GameBoyMCExpr.h - GameBoy specific MC expression classes --------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_GameBoy_MCEXPR_H
#define LLVM_GameBoy_MCEXPR_H

#include "llvm/MC/MCExpr.h"

#include "MCTargetDesc/GameBoyFixupKinds.h"

namespace llvm {

/// A expression in GameBoy machine code.
class GameBoyMCExpr : public MCTargetExpr {
public:
  /// Specifies the type of an expression.
  enum VariantKind {
    VK_GameBoy_None = 0,

    VK_GameBoy_HI8,  ///< Corresponds to `hi8()`.
    VK_GameBoy_LO8,  ///< Corresponds to `lo8()`.
    VK_GameBoy_HH8,  ///< Corresponds to `hlo8() and hh8()`.
    VK_GameBoy_HHI8, ///< Corresponds to `hhi8()`.

    VK_GameBoy_PM,     ///< Corresponds to `pm()`, reference to program memory.
    VK_GameBoy_PM_LO8, ///< Corresponds to `pm_lo8()`.
    VK_GameBoy_PM_HI8, ///< Corresponds to `pm_hi8()`.
    VK_GameBoy_PM_HH8, ///< Corresponds to `pm_hh8()`.

    VK_GameBoy_LO8_GS, ///< Corresponds to `lo8(gs())`.
    VK_GameBoy_HI8_GS, ///< Corresponds to `hi8(gs())`.
    VK_GameBoy_GS,     ///< Corresponds to `gs()`.
  };

public:
  /// Creates an GameBoy machine code expression.
  static const GameBoyMCExpr *create(VariantKind Kind, const MCExpr *Expr,
                                 bool isNegated, MCContext &Ctx);

  /// Gets the type of the expression.
  VariantKind getKind() const { return Kind; }
  /// Gets the name of the expression.
  const char *getName() const;
  const MCExpr *getSubExpr() const { return SubExpr; }
  /// Gets the fixup which corresponds to the expression.
  GameBoy::Fixups getFixupKind() const;
  /// Evaluates the fixup as a constant value.
  bool evaluateAsConstant(int64_t &Result) const;

  bool isNegated() const { return Negated; }
  void setNegated(bool negated = true) { Negated = negated; }

  void printImpl(raw_ostream &OS, const MCAsmInfo *MAI) const override;
  bool evaluateAsRelocatableImpl(MCValue &Res, const MCAsmLayout *Layout,
                                 const MCFixup *Fixup) const override;

  void visitUsedExpr(MCStreamer &streamer) const override;

  MCFragment *findAssociatedFragment() const override {
    return getSubExpr()->findAssociatedFragment();
  }

  void fixELFSymbolsInTLSFixups(MCAssembler &Asm) const override {}

  static bool classof(const MCExpr *E) {
    return E->getKind() == MCExpr::Target;
  }

public:
  static VariantKind getKindByName(StringRef Name);

private:
  int64_t evaluateAsInt64(int64_t Value) const;

  const VariantKind Kind;
  const MCExpr *SubExpr;
  bool Negated;

private:
  explicit GameBoyMCExpr(VariantKind Kind, const MCExpr *Expr, bool Negated)
      : Kind(Kind), SubExpr(Expr), Negated(Negated) {}
  ~GameBoyMCExpr() = default;
};

} // end namespace llvm

#endif // LLVM_GameBoy_MCEXPR_H
