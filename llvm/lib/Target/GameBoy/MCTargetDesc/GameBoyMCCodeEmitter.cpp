//===-- GameBoyMCCodeEmitter.cpp - Convert GameBoy Code to Machine Code -----------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the GameBoyMCCodeEmitter class.
//
//===----------------------------------------------------------------------===//

#include "GameBoyMCCodeEmitter.h"

#include "MCTargetDesc/GameBoyMCExpr.h"
#include "MCTargetDesc/GameBoyMCTargetDesc.h"

#include "llvm/ADT/APFloat.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCFixup.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/EndianStream.h"
#include "llvm/Support/raw_ostream.h"

#define DEBUG_TYPE "mccodeemitter"

#define GET_INSTRMAP_INFO
#include "GameBoyGenInstrInfo.inc"
#undef GET_INSTRMAP_INFO

namespace llvm {

/// Performs a post-encoding step on a `LD` or `ST` instruction.
///
/// The encoding of the LD/ST family of instructions is inconsistent w.r.t
/// the pointer register and the addressing mode.
///
/// The permutations of the format are as followed:
/// ld Rd, X    `1001 000d dddd 1100`
/// ld Rd, X+   `1001 000d dddd 1101`
/// ld Rd, -X   `1001 000d dddd 1110`
///
/// ld Rd, Y    `1000 000d dddd 1000`
/// ld Rd, Y+   `1001 000d dddd 1001`
/// ld Rd, -Y   `1001 000d dddd 1010`
///
/// ld Rd, Z    `1000 000d dddd 0000`
/// ld Rd, Z+   `1001 000d dddd 0001`
/// ld Rd, -Z   `1001 000d dddd 0010`
///                 ^
///                 |
/// Note this one inconsistent bit - it is 1 sometimes and 0 at other times.
/// There is no logical pattern. Looking at a truth table, the following
/// formula can be derived to fit the pattern:
//
/// ```
/// inconsistent_bit = is_predec OR is_postinc OR is_reg_x
/// ```
//
/// We manually set this bit in this post encoder method.
unsigned
GameBoyMCCodeEmitter::loadStorePostEncoder(const MCInst &MI, unsigned EncodedValue,
                                       const MCSubtargetInfo &STI) const {

  assert(MI.getOperand(0).isReg() && MI.getOperand(1).isReg() &&
         "the load/store operands must be registers");

  unsigned Opcode = MI.getOpcode();

  // check whether either of the registers are the X pointer register.
  bool IsRegX = MI.getOperand(0).getReg() == GameBoy::R27R26 ||
                MI.getOperand(1).getReg() == GameBoy::R27R26;

  llvm_unreachable("Unimplemented loadStorePostEncoder");
  /* TODO: Check if this is needed in final code.
  bool IsPredec = Opcode == GameBoy::LDRdPtrPd || Opcode == GameBoy::STPtrPdRr;
  bool IsPostinc = Opcode == GameBoy::LDRdPtrPi || Opcode == GameBoy::STPtrPiRr;

  // Check if we need to set the inconsistent bit
  if (IsRegX || IsPredec || IsPostinc) {
    EncodedValue |= (1 << 12);
  }
  */

  return EncodedValue;
}

template <GameBoy::Fixups Fixup>
unsigned
GameBoyMCCodeEmitter::encodeRelCondBrTarget(const MCInst &MI, unsigned OpNo,
                                        SmallVectorImpl<MCFixup> &Fixups,
                                        const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  if (MO.isExpr()) {
    Fixups.push_back(
        MCFixup::create(0, MO.getExpr(), MCFixupKind(Fixup), MI.getLoc()));
    return 0;
  }

  assert(MO.isImm());

  // Take the size of the current instruction away.
  // With labels, this is implicitly done.
  auto target = MO.getImm();
  GameBoy::fixups::adjustBranchTarget(target);
  return target;
}

unsigned GameBoyMCCodeEmitter::encodeLDSTPtrReg(const MCInst &MI, unsigned OpNo,
                                            SmallVectorImpl<MCFixup> &Fixups,
                                            const MCSubtargetInfo &STI) const {
  auto MO = MI.getOperand(OpNo);

  // The operand should be a pointer register.
  assert(MO.isReg());

  switch (MO.getReg()) {
  case GameBoy::R27R26:
    return 0x03; // X: 0b11
  case GameBoy::R29R28:
    return 0x02; // Y: 0b10
  case GameBoy::R31R30:
    return 0x00; // Z: 0b00
  default:
    llvm_unreachable("invalid pointer register");
  }
}

/// Encodes a `memri` operand.
/// The operand is 7-bits.
/// * The lower 6 bits is the immediate
/// * The upper bit is the pointer register bit (Z=0,Y=1)
unsigned GameBoyMCCodeEmitter::encodeMemri(const MCInst &MI, unsigned OpNo,
                                       SmallVectorImpl<MCFixup> &Fixups,
                                       const MCSubtargetInfo &STI) const {
  auto RegOp = MI.getOperand(OpNo);
  auto OffsetOp = MI.getOperand(OpNo + 1);

  assert(RegOp.isReg() && "Expected register operand");

  uint8_t RegBit = 0;

  switch (RegOp.getReg()) {
  default:
    llvm_unreachable("Expected either Y or Z register");
  case GameBoy::R31R30:
    RegBit = 0;
    break; // Z register
  case GameBoy::R29R28:
    RegBit = 1;
    break; // Y register
  }

  int8_t OffsetBits;

  if (OffsetOp.isImm()) {
    OffsetBits = OffsetOp.getImm();
  } else if (OffsetOp.isExpr()) {
    OffsetBits = 0;
    Fixups.push_back(MCFixup::create(0, OffsetOp.getExpr(),
                                     MCFixupKind(GameBoy::fixup_6), MI.getLoc()));
  } else {
    llvm_unreachable("invalid value for offset");
  }

  return (RegBit << 6) | OffsetBits;
}

unsigned GameBoyMCCodeEmitter::encodeComplement(const MCInst &MI, unsigned OpNo,
                                            SmallVectorImpl<MCFixup> &Fixups,
                                            const MCSubtargetInfo &STI) const {
  // The operand should be an immediate.
  assert(MI.getOperand(OpNo).isImm());

  auto Imm = MI.getOperand(OpNo).getImm();
  return (~0) - Imm;
}

template <GameBoy::Fixups Fixup, unsigned Offset>
unsigned GameBoyMCCodeEmitter::encodeImm(const MCInst &MI, unsigned OpNo,
                                     SmallVectorImpl<MCFixup> &Fixups,
                                     const MCSubtargetInfo &STI) const {
  auto MO = MI.getOperand(OpNo);

  if (MO.isExpr()) {
    if (isa<GameBoyMCExpr>(MO.getExpr())) {
      // If the expression is already an GameBoyMCExpr (i.e. a lo8(symbol),
      // we shouldn't perform any more fixups. Without this check, we would
      // instead create a fixup to the symbol named 'lo8(symbol)' which
      // is not correct.
      return getExprOpValue(MO.getExpr(), Fixups, STI);
    }

    MCFixupKind FixupKind = static_cast<MCFixupKind>(Fixup);
    Fixups.push_back(
        MCFixup::create(Offset, MO.getExpr(), FixupKind, MI.getLoc()));

    return 0;
  }

  assert(MO.isImm());
  return MO.getImm();
}

unsigned GameBoyMCCodeEmitter::encodeCallTarget(const MCInst &MI, unsigned OpNo,
                                            SmallVectorImpl<MCFixup> &Fixups,
                                            const MCSubtargetInfo &STI) const {
  auto MO = MI.getOperand(OpNo);

  if (MO.isExpr()) {
    MCFixupKind FixupKind = static_cast<MCFixupKind>(GameBoy::fixup_call);
    Fixups.push_back(MCFixup::create(0, MO.getExpr(), FixupKind, MI.getLoc()));
    return 0;
  }

  assert(MO.isImm());

  auto Target = MO.getImm();
  GameBoy::fixups::adjustBranchTarget(Target);
  return Target;
}

unsigned GameBoyMCCodeEmitter::getExprOpValue(const MCExpr *Expr,
                                          SmallVectorImpl<MCFixup> &Fixups,
                                          const MCSubtargetInfo &STI) const {

  MCExpr::ExprKind Kind = Expr->getKind();

  if (Kind == MCExpr::Binary) {
    Expr = static_cast<const MCBinaryExpr *>(Expr)->getLHS();
    Kind = Expr->getKind();
  }

  if (Kind == MCExpr::Target) {
    GameBoyMCExpr const *GameBoyExpr = cast<GameBoyMCExpr>(Expr);
    int64_t Result;
    if (GameBoyExpr->evaluateAsConstant(Result)) {
      return Result;
    }

    MCFixupKind FixupKind = static_cast<MCFixupKind>(GameBoyExpr->getFixupKind());
    Fixups.push_back(MCFixup::create(0, GameBoyExpr, FixupKind));
    return 0;
  }

  assert(Kind == MCExpr::SymbolRef);
  return 0;
}

unsigned GameBoyMCCodeEmitter::getMachineOpValue(const MCInst &MI,
                                             const MCOperand &MO,
                                             SmallVectorImpl<MCFixup> &Fixups,
                                             const MCSubtargetInfo &STI) const {
  if (MO.isReg())
    return Ctx.getRegisterInfo()->getEncodingValue(MO.getReg());
  if (MO.isImm())
    return static_cast<unsigned>(MO.getImm());

  if (MO.isDFPImm())
    return static_cast<unsigned>(bit_cast<double>(MO.getDFPImm()));

  // MO must be an Expr.
  assert(MO.isExpr());

  return getExprOpValue(MO.getExpr(), Fixups, STI);
}

void GameBoyMCCodeEmitter::emitInstruction(uint64_t Val, unsigned Size,
                                       const MCSubtargetInfo &STI,
                                       raw_ostream &OS) const {
  size_t WordCount = Size / 2;

  for (int64_t i = WordCount - 1; i >= 0; --i) {
    uint16_t Word = (Val >> (i * 16)) & 0xFFFF;
    support::endian::write(OS, Word, support::endianness::little);
  }
}

void GameBoyMCCodeEmitter::encodeInstruction(const MCInst &MI, raw_ostream &OS,
                                         SmallVectorImpl<MCFixup> &Fixups,
                                         const MCSubtargetInfo &STI) const {
  const MCInstrDesc &Desc = MCII.get(MI.getOpcode());

  // Get byte count of instruction
  unsigned Size = Desc.getSize();

  assert(Size > 0 && "Instruction size cannot be zero");

  uint64_t BinaryOpCode = getBinaryCodeForInstr(MI, Fixups, STI);
  emitInstruction(BinaryOpCode, Size, STI, OS);
}

MCCodeEmitter *createGameBoyMCCodeEmitter(const MCInstrInfo &MCII,
                                      MCContext &Ctx) {
  return new GameBoyMCCodeEmitter(MCII, Ctx);
}

#include "GameBoyGenMCCodeEmitter.inc"

} // end of namespace llvm
