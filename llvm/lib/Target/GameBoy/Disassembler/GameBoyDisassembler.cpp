//===- GameBoyDisassembler.cpp - Disassembler for GameBoy ---------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file is part of the GameBoy Disassembler.
//
//===----------------------------------------------------------------------===//

#include "GameBoy.h"
#include "GameBoyRegisterInfo.h"
#include "GameBoySubtarget.h"
#include "MCTargetDesc/GameBoyMCTargetDesc.h"
#include "TargetInfo/GameBoyTargetInfo.h"

#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDecoderOps.h"
#include "llvm/MC/MCDisassembler/MCDisassembler.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/TargetRegistry.h"

using namespace llvm;

#define DEBUG_TYPE "GameBoy-disassembler"

typedef MCDisassembler::DecodeStatus DecodeStatus;

namespace {

/// A disassembler class for GameBoy.
class GameBoyDisassembler : public MCDisassembler {
public:
  GameBoyDisassembler(const MCSubtargetInfo &STI, MCContext &Ctx)
      : MCDisassembler(STI, Ctx) {}
  virtual ~GameBoyDisassembler() = default;

  DecodeStatus getInstruction(MCInst &Instr, uint64_t &Size,
                              ArrayRef<uint8_t> Bytes, uint64_t Address,
                              raw_ostream &CStream) const override;
};
} // namespace

static MCDisassembler *createGameBoyDisassembler(const Target &T,
                                             const MCSubtargetInfo &STI,
                                             MCContext &Ctx) {
  return new GameBoyDisassembler(STI, Ctx);
}

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeGameBoyDisassembler() {
  // Register the disassembler.
  TargetRegistry::RegisterMCDisassembler(getTheGameBoyTarget(),
                                         createGameBoyDisassembler);
}

static const uint16_t GPRDecoderTable[] = {
    GameBoy::R0,  GameBoy::R1,  GameBoy::R2,  GameBoy::R3,  GameBoy::R4,  GameBoy::R5,  GameBoy::R6,
    GameBoy::R7,  GameBoy::R8,  GameBoy::R9,  GameBoy::R10, GameBoy::R11, GameBoy::R12, GameBoy::R13,
    GameBoy::R14, GameBoy::R15, GameBoy::R16, GameBoy::R17, GameBoy::R18, GameBoy::R19, GameBoy::R20,
    GameBoy::R21, GameBoy::R22, GameBoy::R23, GameBoy::R24, GameBoy::R25, GameBoy::R26, GameBoy::R27,
    GameBoy::R28, GameBoy::R29, GameBoy::R30, GameBoy::R31,
};

static const uint16_t GameBoyGPRDecoderTable[] = {
    GameBoy::RA,  GameBoy::RF,  GameBoy::RB,  GameBoy::RC,  
    GameBoy::RD,  GameBoy::RE,  GameBoy::RH,  GameBoy::RL,
};

static const uint16_t GameBoyGPRPairDecoderTable[] = {
    GameBoy::RARF,  GameBoy::RBRC,  GameBoy::RDRE,  GameBoy::RHRL,
};

static const uint16_t GameBoyGPRPairPointerHLDecoderTable[] = {
    GameBoy::RHRL,
};

// Method added for GB 8-bit registers.
static DecodeStatus DecodeGPRRegisterClass(MCInst &Inst, unsigned RegNo,
                                            uint64_t Address,
                                            const MCDisassembler *Decoder) {
  if (RegNo > 7)
    return MCDisassembler::Fail;

  unsigned Register = GameBoyGPRDecoderTable[RegNo];
  Inst.addOperand(MCOperand::createReg(Register));
  return MCDisassembler::Success;
}

// Method added for GB 16-bit register pair class.
static DecodeStatus DecodeGPRPairRegisterClass(MCInst &Inst, unsigned RegNo,
                                            uint64_t Address,
                                            const MCDisassembler *Decoder) {
  // Do not include the AF register here.
  if (RegNo > 6 || RegNo < 2)
    return MCDisassembler::Fail;

  unsigned Register = GameBoyGPRPairDecoderTable[RegNo];
  Inst.addOperand(MCOperand::createReg(Register));
  return MCDisassembler::Success;
}

// Method added for GB LoadSourceA register class.
static DecodeStatus DecodeGPRLoadRegisterClass(MCInst &Inst, unsigned RegNo,
                                                      uint64_t Address,
                                                      const MCDisassembler *Decoder) {
  // Only return the A register here!
  if (RegNo != 0)
    return MCDisassembler::Fail;

  unsigned Register = GameBoyGPRDecoderTable[RegNo];
  Inst.addOperand(MCOperand::createReg(Register));
  return MCDisassembler::Success;
}

// Method added for GB LoadSourceA register class.
static DecodeStatus DecodeGPRPairPointerHLRegisterClass(MCInst &Inst, unsigned RegNo,
                                                      uint64_t Address,
                                                      const MCDisassembler *Decoder) {
  // Only return the A register here!
  if (RegNo != 0)
    return MCDisassembler::Fail;

  unsigned Register = GameBoyGPRPairPointerHLDecoderTable[RegNo];
  Inst.addOperand(MCOperand::createReg(Register));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeStackRegistersClass(MCInst &Inst, unsigned RegNo,
                                              uint64_t Address,
                                              const MCDisassembler *Decoder) {
 if (RegNo > 6)
    return MCDisassembler::Fail;

  unsigned Register = GameBoyGPRPairDecoderTable[RegNo];
  Inst.addOperand(MCOperand::createReg(Register));
  return MCDisassembler::Success; 
}

static DecodeStatus DecodeGPR8RegisterClass(MCInst &Inst, unsigned RegNo,
                                            uint64_t Address,
                                            const MCDisassembler *Decoder) {
  if (RegNo > 31)
    return MCDisassembler::Fail;

  unsigned Register = GPRDecoderTable[RegNo];
  Inst.addOperand(MCOperand::createReg(Register));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeLD8RegisterClass(MCInst &Inst, unsigned RegNo,
                                           uint64_t Address,
                                           const MCDisassembler *Decoder) {
  if (RegNo > 15)
    return MCDisassembler::Fail;

  unsigned Register = GPRDecoderTable[RegNo + 16];
  Inst.addOperand(MCOperand::createReg(Register));
  return MCDisassembler::Success;
}

static DecodeStatus decodeFIOARr(MCInst &Inst, unsigned Insn, uint64_t Address,
                                 const MCDisassembler *Decoder);

static DecodeStatus decodeFIORdA(MCInst &Inst, unsigned Insn, uint64_t Address,
                                 const MCDisassembler *Decoder);

static DecodeStatus decodeFIOBIT(MCInst &Inst, unsigned Insn, uint64_t Address,
                                 const MCDisassembler *Decoder);

static DecodeStatus decodeCallTarget(MCInst &Inst, unsigned Insn,
                                     uint64_t Address,
                                     const MCDisassembler *Decoder);

static DecodeStatus decodeFRd(MCInst &Inst, unsigned Insn, uint64_t Address,
                              const MCDisassembler *Decoder);

static DecodeStatus decodeFLPMX(MCInst &Inst, unsigned Insn, uint64_t Address,
                                const MCDisassembler *Decoder);

static DecodeStatus decodeFFMULRdRr(MCInst &Inst, unsigned Insn,
                                    uint64_t Address,
                                    const MCDisassembler *Decoder);

static DecodeStatus decodeFMOVWRdRr(MCInst &Inst, unsigned Insn,
                                    uint64_t Address,
                                    const MCDisassembler *Decoder);

static DecodeStatus decodeFWRdK(MCInst &Inst, unsigned Insn, uint64_t Address,
                                const MCDisassembler *Decoder);

static DecodeStatus decodeFMUL2RdRr(MCInst &Inst, unsigned Insn,
                                    uint64_t Address,
                                    const MCDisassembler *Decoder);

static DecodeStatus decodeMemri(MCInst &Inst, unsigned Insn, uint64_t Address,
                                const MCDisassembler *Decoder);

static DecodeStatus decodeLoadStore(MCInst &Inst, unsigned Insn,
                                    uint64_t Address,
                                    const MCDisassembler *Decoder);

#include "GameBoyGenDisassemblerTables.inc"

static DecodeStatus decodeFIOARr(MCInst &Inst, unsigned Insn, uint64_t Address,
                                 const MCDisassembler *Decoder) {
  unsigned addr = 0;
  addr |= fieldFromInstruction(Insn, 0, 4);
  addr |= fieldFromInstruction(Insn, 9, 2) << 4;
  unsigned reg = fieldFromInstruction(Insn, 4, 5);
  Inst.addOperand(MCOperand::createImm(addr));
  if (DecodeGPR8RegisterClass(Inst, reg, Address, Decoder) ==
      MCDisassembler::Fail)
    return MCDisassembler::Fail;
  return MCDisassembler::Success;
}

static DecodeStatus decodeFIORdA(MCInst &Inst, unsigned Insn, uint64_t Address,
                                 const MCDisassembler *Decoder) {
  unsigned addr = 0;
  addr |= fieldFromInstruction(Insn, 0, 4);
  addr |= fieldFromInstruction(Insn, 9, 2) << 4;
  unsigned reg = fieldFromInstruction(Insn, 4, 5);
  if (DecodeGPR8RegisterClass(Inst, reg, Address, Decoder) ==
      MCDisassembler::Fail)
    return MCDisassembler::Fail;
  Inst.addOperand(MCOperand::createImm(addr));
  return MCDisassembler::Success;
}

static DecodeStatus decodeFIOBIT(MCInst &Inst, unsigned Insn, uint64_t Address,
                                 const MCDisassembler *Decoder) {
  unsigned addr = fieldFromInstruction(Insn, 3, 5);
  unsigned b = fieldFromInstruction(Insn, 0, 3);
  Inst.addOperand(MCOperand::createImm(addr));
  Inst.addOperand(MCOperand::createImm(b));
  return MCDisassembler::Success;
}

static DecodeStatus decodeCallTarget(MCInst &Inst, unsigned Field,
                                     uint64_t Address,
                                     const MCDisassembler *Decoder) {
  // Call targets need to be shifted left by one so this needs a custom
  // decoder.
  Inst.addOperand(MCOperand::createImm(Field << 1));
  return MCDisassembler::Success;
}

static DecodeStatus decodeFRd(MCInst &Inst, unsigned Insn, uint64_t Address,
                              const MCDisassembler *Decoder) {
  unsigned d = fieldFromInstruction(Insn, 4, 5);
  if (DecodeGPR8RegisterClass(Inst, d, Address, Decoder) ==
      MCDisassembler::Fail)
    return MCDisassembler::Fail;
  return MCDisassembler::Success;
}

static DecodeStatus decodeFLPMX(MCInst &Inst, unsigned Insn, uint64_t Address,
                                const MCDisassembler *Decoder) {
  if (decodeFRd(Inst, Insn, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  Inst.addOperand(MCOperand::createReg(GameBoy::R31R30));
  return MCDisassembler::Success;
}

static DecodeStatus decodeFFMULRdRr(MCInst &Inst, unsigned Insn,
                                    uint64_t Address,
                                    const MCDisassembler *Decoder) {
  unsigned d = fieldFromInstruction(Insn, 4, 3) + 16;
  unsigned r = fieldFromInstruction(Insn, 0, 3) + 16;
  if (DecodeGPR8RegisterClass(Inst, d, Address, Decoder) ==
      MCDisassembler::Fail)
    return MCDisassembler::Fail;
  if (DecodeGPR8RegisterClass(Inst, r, Address, Decoder) ==
      MCDisassembler::Fail)
    return MCDisassembler::Fail;
  return MCDisassembler::Success;
}

static DecodeStatus decodeFMOVWRdRr(MCInst &Inst, unsigned Insn,
                                    uint64_t Address,
                                    const MCDisassembler *Decoder) {
  unsigned r = fieldFromInstruction(Insn, 4, 4) * 2;
  unsigned d = fieldFromInstruction(Insn, 0, 4) * 2;
  if (DecodeGPR8RegisterClass(Inst, r, Address, Decoder) ==
      MCDisassembler::Fail)
    return MCDisassembler::Fail;
  if (DecodeGPR8RegisterClass(Inst, d, Address, Decoder) ==
      MCDisassembler::Fail)
    return MCDisassembler::Fail;
  return MCDisassembler::Success;
}

static DecodeStatus decodeFWRdK(MCInst &Inst, unsigned Insn, uint64_t Address,
                                const MCDisassembler *Decoder) {
  unsigned d = fieldFromInstruction(Insn, 4, 2) * 2 + 24; // starts at r24:r25
  unsigned k = 0;
  k |= fieldFromInstruction(Insn, 0, 4);
  k |= fieldFromInstruction(Insn, 6, 2) << 4;
  if (DecodeGPR8RegisterClass(Inst, d, Address, Decoder) ==
      MCDisassembler::Fail)
    return MCDisassembler::Fail;
  if (DecodeGPR8RegisterClass(Inst, d, Address, Decoder) ==
      MCDisassembler::Fail)
    return MCDisassembler::Fail;
  Inst.addOperand(MCOperand::createImm(k));
  return MCDisassembler::Success;
}

static DecodeStatus decodeFMUL2RdRr(MCInst &Inst, unsigned Insn,
                                    uint64_t Address,
                                    const MCDisassembler *Decoder) {
  unsigned rd = fieldFromInstruction(Insn, 4, 4) + 16;
  unsigned rr = fieldFromInstruction(Insn, 0, 4) + 16;
  if (DecodeGPR8RegisterClass(Inst, rd, Address, Decoder) ==
      MCDisassembler::Fail)
    return MCDisassembler::Fail;
  if (DecodeGPR8RegisterClass(Inst, rr, Address, Decoder) ==
      MCDisassembler::Fail)
    return MCDisassembler::Fail;
  return MCDisassembler::Success;
}

static DecodeStatus decodeMemri(MCInst &Inst, unsigned Insn, uint64_t Address,
                                const MCDisassembler *Decoder) {
  // As in the EncoderMethod `GameBoyMCCodeEmitter::encodeMemri`, the memory
  // address is encoded into 7-bit, in which bits 0-5 are the immediate offset,
  // and the bit-6 is the pointer register bit (Z=0, Y=1).
  if (Insn > 127)
    return MCDisassembler::Fail;

  // Append the base register operand.
  Inst.addOperand(
      MCOperand::createReg((Insn & 0x40) ? GameBoy::R29R28 : GameBoy::R31R30));
  // Append the immediate offset operand.
  Inst.addOperand(MCOperand::createImm(Insn & 0x3f));

  return MCDisassembler::Success;
}

static DecodeStatus decodeLoadStore(MCInst &Inst, unsigned Insn,
                                    uint64_t Address,
                                    const MCDisassembler *Decoder) {
  // Get the register will be loaded or stored.
  unsigned RegVal = GPRDecoderTable[(Insn >> 4) & 0x1f];

  // Decode LDD/STD with offset less than 8.
  if ((Insn & 0xf000) == 0x8000) {
    unsigned RegBase = (Insn & 0x8) ? GameBoy::R29R28 : GameBoy::R31R30;
    unsigned Offset = Insn & 7; // We need not consider offset > 7.
    if ((Insn & 0x200) == 0) { // Decode LDD.
      llvm_unreachable("Unimplemented disassembler for LDDRdPtrQ");
      // Inst.setOpcode(GameBoy::LDDRdPtrQ);
      Inst.addOperand(MCOperand::createReg(RegVal));
      Inst.addOperand(MCOperand::createReg(RegBase));
      Inst.addOperand(MCOperand::createImm(Offset));
    } else { // Decode STD.
      llvm_unreachable("Unimplemented disassembler for STDPtrQRr");
      // Inst.setOpcode(GameBoy::STDPtrQRr);
      Inst.addOperand(MCOperand::createReg(RegBase));
      Inst.addOperand(MCOperand::createImm(Offset));
      Inst.addOperand(MCOperand::createReg(RegVal));
    }
    return MCDisassembler::Success;
  }

  // Decode the following 14 instructions. Bit 9 indicates load(0) or store(1),
  // bits 8~4 indicate the value register, bits 3-2 indicate the base address
  // register (11-X, 10-Y, 00-Z), bits 1~0 indicate the mode (00-basic,
  // 01-postinc, 10-predec).
  // ST X,  Rr : 1001 001r rrrr 1100
  // ST X+, Rr : 1001 001r rrrr 1101
  // ST -X, Rr : 1001 001r rrrr 1110
  // ST Y+, Rr : 1001 001r rrrr 1001
  // ST -Y, Rr : 1001 001r rrrr 1010
  // ST Z+, Rr : 1001 001r rrrr 0001
  // ST -Z, Rr : 1001 001r rrrr 0010
  // LD Rd, X  : 1001 000d dddd 1100
  // LD Rd, X+ : 1001 000d dddd 1101
  // LD Rd, -X : 1001 000d dddd 1110
  // LD Rd, Y+ : 1001 000d dddd 1001
  // LD Rd, -Y : 1001 000d dddd 1010
  // LD Rd, Z+ : 1001 000d dddd 0001
  // LD Rd, -Z : 1001 000d dddd 0010
  if ((Insn & 0xfc00) != 0x9000 || (Insn & 0xf) == 0)
    return MCDisassembler::Fail;

  // Get the base address register.
  unsigned RegBase;
  switch (Insn & 0xc) {
  case 0xc:
    RegBase = GameBoy::R27R26;
    break;
  case 0x8:
    RegBase = GameBoy::R29R28;
    break;
  case 0x0:
    RegBase = GameBoy::R31R30;
    break;
  default:
    return MCDisassembler::Fail;
  }

  // Set the opcode.
  switch (Insn & 0x203) {
  // case 0x200:
  //   Inst.setOpcode(GameBoy::STPtrRr);
  //   Inst.addOperand(MCOperand::createReg(RegBase));
  //   Inst.addOperand(MCOperand::createReg(RegVal));
  //   return MCDisassembler::Success;
  // case 0x201:
  //   Inst.setOpcode(GameBoy::STPtrPiRr);
  //   break;
  // case 0x202:
  //   Inst.setOpcode(GameBoy::STPtrPdRr);
  //   break;
  // case 0:
  //   Inst.setOpcode(GameBoy::LDRdPtr);
  //   Inst.addOperand(MCOperand::createReg(RegVal));
  //   Inst.addOperand(MCOperand::createReg(RegBase));
  //   return MCDisassembler::Success;
  // case 1:
  //   Inst.setOpcode(GameBoy::LDRdPtrPi);
  //   break;
  // case 2:
  //   Inst.setOpcode(GameBoy::LDRdPtrPd);
  //   break;
  default:
    return MCDisassembler::Fail;
  }

  // Build postinc/predec machine instructions.
  if ((Insn & 0x200) == 0) { // This is a load instruction.
    Inst.addOperand(MCOperand::createReg(RegVal));
    Inst.addOperand(MCOperand::createReg(RegBase));
    Inst.addOperand(MCOperand::createReg(RegBase));
  } else { // This is a store instruction.
    Inst.addOperand(MCOperand::createReg(RegBase));
    Inst.addOperand(MCOperand::createReg(RegBase));
    Inst.addOperand(MCOperand::createReg(RegVal));
    // STPtrPiRr and STPtrPdRr have an extra immediate operand.
    Inst.addOperand(MCOperand::createImm(1));
  }

  return MCDisassembler::Success;
}

static DecodeStatus readInstruction16(ArrayRef<uint8_t> Bytes, uint64_t Address,
                                      uint64_t &Size, uint32_t &Insn) {
  if (Bytes.size() < 2) {
    Size = 0;
    return MCDisassembler::Fail;
  }

  Size = 2;
  Insn = (Bytes[0] << 0) | (Bytes[1] << 8);

  return MCDisassembler::Success;
}

static DecodeStatus readInstruction32(ArrayRef<uint8_t> Bytes, uint64_t Address,
                                      uint64_t &Size, uint32_t &Insn) {

  if (Bytes.size() < 4) {
    Size = 0;
    return MCDisassembler::Fail;
  }

  Size = 4;
  Insn =
      (Bytes[0] << 16) | (Bytes[1] << 24) | (Bytes[2] << 0) | (Bytes[3] << 8);

  return MCDisassembler::Success;
}

static const uint8_t *getDecoderTable(uint64_t Size) {

  switch (Size) {
  case 1:
    return DecoderTable8;
  // case 2:
  //   return DecoderTable16;
  // case 4:
  //   return DecoderTable32;
  default:
    llvm_unreachable("instructions are only 8-bit");
  }
}

DecodeStatus GameBoyDisassembler::getInstruction(MCInst &Instr, uint64_t &Size,
                                             ArrayRef<uint8_t> Bytes,
                                             uint64_t Address,
                                             raw_ostream &CStream) const {
  uint32_t Insn;

  DecodeStatus Result;

  // Try decode a 16-bit instruction.
  {
    Result = readInstruction16(Bytes, Address, Size, Insn);

    if (Result == MCDisassembler::Fail)
      return MCDisassembler::Fail;

    // Try to auto-decode a 16-bit instruction.
    Result = decodeInstruction(getDecoderTable(Size), Instr, Insn, Address,
                               this, STI);
    if (Result != MCDisassembler::Fail)
      return Result;

    // Try to decode to a load/store instruction. ST/LD need a specified
    // DecoderMethod, as they already have a specified PostEncoderMethod.
    Result = decodeLoadStore(Instr, Insn, Address, this);
    if (Result != MCDisassembler::Fail)
      return Result;
  }

  // Try decode a 32-bit instruction.
  {
    Result = readInstruction32(Bytes, Address, Size, Insn);

    if (Result == MCDisassembler::Fail)
      return MCDisassembler::Fail;

    Result = decodeInstruction(getDecoderTable(Size), Instr, Insn, Address,
                               this, STI);

    if (Result != MCDisassembler::Fail) {
      return Result;
    }

    return MCDisassembler::Fail;
  }
}

typedef DecodeStatus (*DecodeFunc)(MCInst &MI, unsigned insn, uint64_t Address,
                                   const MCDisassembler *Decoder);
