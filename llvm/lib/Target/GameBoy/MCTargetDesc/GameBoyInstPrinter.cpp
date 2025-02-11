//===-- GameBoyInstPrinter.cpp - Convert GameBoy MCInst to assembly syntax --------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This class prints an GameBoy MCInst to a .s file.
//
//===----------------------------------------------------------------------===//

#include "GameBoyInstPrinter.h"

#include "MCTargetDesc/GameBoyMCTargetDesc.h"

#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/Debug.h"

#include <cstring>

#define DEBUG_TYPE "asm-printer"

namespace llvm {

// Include the auto-generated portion of the assembly writer.
#define PRINT_ALIAS_INSTR
#include "GameBoyGenAsmWriter.inc"

void GameBoyInstPrinter::printInst(const MCInst *MI, uint64_t Address,
                               StringRef Annot, const MCSubtargetInfo &STI,
                               raw_ostream &O) {
  unsigned Opcode = MI->getOpcode();

  // First handle load and store instructions with postinc or predec
  // of the form "ld reg, X+".
  // TODO: We should be able to rewrite this using TableGen data.
  switch (Opcode) {
  case GameBoy::LDRd8Ptr:
  case GameBoy::LDRdHLPtr:
  case GameBoy::LDAPtr:
  case GameBoy::LDAImm8Addr:
  case GameBoy::LDAImm16Addr:
  case GameBoy::LDRdPtr:
  case GameBoy::LDRdPairPtr:
    O << "\tld\t";
    printOperand(MI, 0, O);
    O << ", [";
    printOperand(MI, 1, O);
    O << "]";
    break;
  // Pointer is first argument
  case GameBoy::LDPtrA:
  case GameBoy::LDHLAddrRr:
  case GameBoy::LDHLAddrImm8:
  case GameBoy::LDPtrRd:
  case GameBoy::LDPtrRdPair:
    O << "\tld\t[";
    printOperand(MI, 0, O);
    O << "], ";
    printOperand(MI, 1, O);
    break;
  case GameBoy::LDImm8AddrA:
  case GameBoy::LDImm16AddrA:
    dbgs() << "LDImm16AddrA has issues";
    O << "\tld\t[";
    printOperand(MI, 0, O);
    O << "], ";
    printOperand(MI, 1, O);
    break;
  default:
    if (!printAliasInstr(MI, Address, O))
      printInstruction(MI, Address, O);

    printAnnotation(O, Annot);
    break;
  }
}

/// @brief Get the name of the register as it should appear in assembly.
/// This implementation differs from the original AVR definition as it prints
/// the full register pair name, rather than just the lower register as GCC does.
/// @param RegNum The number of the register we are looking to print.
/// @param MRI MCRegisterInfo reference.
/// @return A string representing the printable name.
const char *GameBoyInstPrinter::getPrettyRegisterName(unsigned RegNum,
                                                  MCRegisterInfo const &MRI) {
  // GCC prints register pairs by just printing the lower register
  // If the register contains a subregister, print it instead
  
  // if (MRI.getNumSubRegIndices() > 0) {
  //   unsigned RegLoNum = MRI.getSubReg(RegNum, GameBoy::sub_lo);
  //   RegNum = (RegLoNum != GameBoy::NoRegister) ? RegLoNum : RegNum;
  // }

  return getRegisterName(RegNum);
}

void GameBoyInstPrinter::printOperand(const MCInst *MI, unsigned OpNo,
                                  raw_ostream &O) {
  const MCOperandInfo &MOI = this->MII.get(MI->getOpcode()).OpInfo[OpNo];
  if (MOI.RegClass == GameBoy::ZREGRegClassID) {
    // Special case for the Z register, which sometimes doesn't have an operand
    // in the MCInst.
    O << "Z";
    return;
  }

  // See if we can skip over this non-printable operand.
  if (OpNo >= MI->size()) {
    // We need to be able to disassemble the operand `def $reg` 
    // See if it is possible to skip over this operand. This is useful for scenarios where 
    // we are printing a definition-operand in the same instruction.
    // If we exhaust the list then we move on to the next instruction.
    LLVM_DEBUG(dbgs() << "Skipping operand number " << OpNo << " for instruction number " << MI->getOpcode());
    return printOperand(MI, --OpNo, O);
    // O << OpNo << " <unknown> " << MI->size();
    // return;
  }

  const MCOperand &Op = MI->getOperand(OpNo);
  if (Op.isReg()) {
    bool isPtrReg = (MOI.RegClass == GameBoy::PTRREGSRegClassID) ||
                    (MOI.RegClass == GameBoy::PTRDISPREGSRegClassID) ||
                    (MOI.RegClass == GameBoy::ZREGRegClassID);

    if (isPtrReg) {
      O << getRegisterName(Op.getReg(), GameBoy::ptr);
    } else {
      O << getPrettyRegisterName(Op.getReg(), MRI);
    }
  } else if (Op.isImm()) {
    O << formatImm(Op.getImm());
  } else {
    assert(Op.isExpr() && "Unknown operand kind in printOperand");
    O << *Op.getExpr();
  }
}

/// This is used to print an immediate value that ends up
/// being encoded as a pc-relative value.
void GameBoyInstPrinter::printPCRelImm(const MCInst *MI, unsigned OpNo,
                                   raw_ostream &O) {
  if (OpNo > MI->size()) {
    // Not all operands are correctly disassembled at the moment. This means
    // that some machine instructions won't have all the necessary operands
    // set.
    // To avoid asserting, print <unknown> instead until the necessary support
    // has been implemented.
    O << "<unknown>";
    return;
  } else if (OpNo == MI->size()) {
    // See if the next operand is printable
    return printPCRelImm(MI, --OpNo, O);
  }

  const MCOperand &Op = MI->getOperand(OpNo);

  if (Op.isImm()) {
    int64_t Imm = Op.getImm();
    O << '.';

    // Print a position sign if needed.
    // Negative values have their sign printed automatically.
    if (Imm >= 0)
      O << '+';

    O << Imm;
  } else {
    assert(Op.isExpr() && "Unknown pcrel immediate operand");
    O << *Op.getExpr();
  }
}

void GameBoyInstPrinter::printMemri(const MCInst *MI, unsigned OpNo,
                                raw_ostream &O) {
  assert(MI->getOperand(OpNo).isReg() &&
         "Expected a register for the first operand");

  const MCOperand &OffsetOp = MI->getOperand(OpNo + 1);

  // Print the register.
  printOperand(MI, OpNo, O);

  // Print the {+,-}offset.
  if (OffsetOp.isImm()) {
    int64_t Offset = OffsetOp.getImm();

    if (Offset >= 0)
      O << '+';

    O << Offset;
  } else if (OffsetOp.isExpr()) {
    O << *OffsetOp.getExpr();
  } else {
    llvm_unreachable("unknown type for offset");
  }
}

} // end of namespace llvm
