//===-- GameBoyAsmPrinter.cpp - GameBoy LLVM assembly writer ----------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains a printer that converts from our internal representation
// of machine-dependent LLVM code to GAS-format GameBoy assembly language.
//
//===----------------------------------------------------------------------===//

#include "GameBoy.h"
#include "GameBoyMCInstLower.h"
#include "GameBoySubtarget.h"
#include "GameBoyTargetMachine.h"
#include "MCTargetDesc/GameBoyInstPrinter.h"
#include "MCTargetDesc/GameBoyMCExpr.h"
#include "TargetInfo/GameBoyTargetInfo.h"

#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/IR/Mangler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"

#define DEBUG_TYPE "GameBoy-asm-printer"

namespace llvm {

/// An GameBoy assembly code printer.
class GameBoyAsmPrinter : public AsmPrinter {
public:
  GameBoyAsmPrinter(TargetMachine &TM, std::unique_ptr<MCStreamer> Streamer)
      : AsmPrinter(TM, std::move(Streamer)), MRI(*TM.getMCRegisterInfo()) {}

  StringRef getPassName() const override { return "Game Boy Assembly Printer"; }

  void printOperand(const MachineInstr *MI, unsigned OpNo, raw_ostream &O);

  void printMemOperand(const MachineInstr *MI, unsigned OpNo, raw_ostream &O);

  bool PrintAsmOperand(const MachineInstr *MI, unsigned OpNum,
                       const char *ExtraCode, raw_ostream &O) override;

  bool PrintAsmMemoryOperand(const MachineInstr *MI, unsigned OpNum,
                             const char *ExtraCode, raw_ostream &O) override;

  void emitInstruction(const MachineInstr *MI) override;

  const MCExpr *lowerConstant(const Constant *CV) override;

  void emitXXStructor(const DataLayout &DL, const Constant *CV) override;

  bool doFinalization(Module &M) override;

  void emitStartOfAsmFile(Module &M) override;

  // void emitFunctionEntryLabel() override {}

private:
  const MCRegisterInfo &MRI;
  bool EmittedStructorSymbolAttrs = false;
};

void GameBoyAsmPrinter::printMemOperand(const MachineInstr *MI, unsigned OpNo, raw_ostream &O) {
  // These are used to print the square brackets required around the operands.
  switch (MI->getOpcode()) {
    default:
      break;
  }

  O << "[";
  printOperand(MI, OpNo, O);
  O << "]";
}

void GameBoyAsmPrinter::printOperand(const MachineInstr *MI, unsigned OpNo,
                                 raw_ostream &O) {
  const MachineOperand &MO = MI->getOperand(OpNo);

  switch (MO.getType()) {
  case MachineOperand::MO_Register:
    O << GameBoyInstPrinter::getPrettyRegisterName(MO.getReg(), MRI);
    break;
  case MachineOperand::MO_Immediate:
    // If it is marked as a load or a store, then an operand is
    // likely to be a pointer.
    // There is only one operand in these instructions.
    // if (MI->mayLoad()) {
    //   O << "[" << MO.getImm() << "]";
    // } else if (MI->mayStore()) {
    //   O << "[" << MO.getImm() << "]";
    // } else {
    //   O << MO.getImm();
    // }
      O << MO.getImm();
    break;
  case MachineOperand::MO_GlobalAddress:
    O << getSymbol(MO.getGlobal());
    break;
  case MachineOperand::MO_ExternalSymbol:
    O << *GetExternalSymbolSymbol(MO.getSymbolName());
    break;
  case MachineOperand::MO_MachineBasicBlock:
    O << *MO.getMBB()->getSymbol();
    break;
  default:
    llvm_unreachable("Not implemented yet!");
  }
}

bool GameBoyAsmPrinter::PrintAsmOperand(const MachineInstr *MI, unsigned OpNum,
                                    const char *ExtraCode, raw_ostream &O) {
  // Default asm printer can only deal with some extra codes,
  // so try it first.
  bool Error = AsmPrinter::PrintAsmOperand(MI, OpNum, ExtraCode, O);

  if (Error && ExtraCode && ExtraCode[0]) {
    if (ExtraCode[1] != 0)
      return true; // Unknown modifier.

    if (ExtraCode[0] >= 'A' && ExtraCode[0] <= 'Z') {
      const MachineOperand &RegOp = MI->getOperand(OpNum);

      assert(RegOp.isReg() && "Operand must be a register when you're"
                              "using 'A'..'Z' operand extracodes.");
      Register Reg = RegOp.getReg();

      unsigned ByteNumber = ExtraCode[0] - 'A';

      unsigned OpFlags = MI->getOperand(OpNum - 1).getImm();
      unsigned NumOpRegs = InlineAsm::getNumOperandRegisters(OpFlags);
      (void)NumOpRegs;

      const GameBoySubtarget &STI = MF->getSubtarget<GameBoySubtarget>();
      const TargetRegisterInfo &TRI = *STI.getRegisterInfo();

      const TargetRegisterClass *RC = TRI.getMinimalPhysRegClass(Reg);
      unsigned BytesPerReg = TRI.getRegSizeInBits(*RC) / 8;
      assert(BytesPerReg <= 2 && "Only 8 and 16 bit regs are supported.");

      unsigned RegIdx = ByteNumber / BytesPerReg;
      assert(RegIdx < NumOpRegs && "Multibyte index out of range.");

      Reg = MI->getOperand(OpNum + RegIdx).getReg();

      if (BytesPerReg == 2) {
        Reg = TRI.getSubReg(Reg, ByteNumber % BytesPerReg ? GameBoy::sub_hi
                                                          : GameBoy::sub_lo);
      }

      O << GameBoyInstPrinter::getPrettyRegisterName(Reg, MRI);
      return false;
    }
  }

  if (Error)
    printOperand(MI, OpNum, O);

  return false;
}

bool GameBoyAsmPrinter::PrintAsmMemoryOperand(const MachineInstr *MI,
                                          unsigned OpNum, const char *ExtraCode,
                                          raw_ostream &O) {
  if (ExtraCode && ExtraCode[0])
    return true; // Unknown modifier

  const MachineOperand &MO = MI->getOperand(OpNum);
  (void)MO;
  assert(MO.isReg() && "Unexpected inline asm memory operand");

  // TODO: We should be able to look up the alternative name for
  // the register if it's given.
  // TableGen doesn't expose a way of getting retrieving names
  // for registers.
  if (MI->getOperand(OpNum).getReg() == GameBoy::R31R30) {
    O << "Z";
  } else {
    assert(MI->getOperand(OpNum).getReg() == GameBoy::R29R28 &&
           "Wrong register class for memory operand.");
    O << "Y";
  }

  // If NumOpRegs == 2, then we assume it is product of a FrameIndex expansion
  // and the second operand is an Imm.
  unsigned OpFlags = MI->getOperand(OpNum - 1).getImm();
  unsigned NumOpRegs = InlineAsm::getNumOperandRegisters(OpFlags);

  if (NumOpRegs == 2) {
    O << '+' << MI->getOperand(OpNum + 1).getImm();
  }

  return false;
}

void GameBoyAsmPrinter::emitInstruction(const MachineInstr *MI) {
  GameBoyMCInstLower MCInstLowering(OutContext, *this);

  MCInst I;
  MCInstLowering.lowerInstruction(*MI, I);
  EmitToStreamer(*OutStreamer, I);
}

const MCExpr *GameBoyAsmPrinter::lowerConstant(const Constant *CV) {
  MCContext &Ctx = OutContext;

  if (const GlobalValue *GV = dyn_cast<GlobalValue>(CV)) {
    bool IsProgMem = GV->getAddressSpace() == GameBoy::ProgramMemory;
    if (IsProgMem) {
      const MCExpr *Expr = MCSymbolRefExpr::create(getSymbol(GV), Ctx);
      return GameBoyMCExpr::create(GameBoyMCExpr::VK_GameBoy_PM, Expr, false, Ctx);
    }
  }

  return AsmPrinter::lowerConstant(CV);
}

void GameBoyAsmPrinter::emitXXStructor(const DataLayout &DL, const Constant *CV) {
  if (!EmittedStructorSymbolAttrs) {
    OutStreamer->emitRawComment(
        " Emitting these undefined symbol references causes us to link the"
        " libgcc code that runs our constructors/destructors");
    OutStreamer->emitRawComment(" This matches GCC's behavior");

    MCSymbol *CtorsSym = OutContext.getOrCreateSymbol("__do_global_ctors");
    OutStreamer->emitSymbolAttribute(CtorsSym, MCSA_Global);

    MCSymbol *DtorsSym = OutContext.getOrCreateSymbol("__do_global_dtors");
    OutStreamer->emitSymbolAttribute(DtorsSym, MCSA_Global);

    EmittedStructorSymbolAttrs = true;
  }

  AsmPrinter::emitXXStructor(DL, CV);
}

bool GameBoyAsmPrinter::doFinalization(Module &M) {
  /*
  MCSymbol *DoCopyData = OutContext.getOrCreateSymbol("__do_copy_data");
  MCSymbol *DoClearBss = OutContext.getOrCreateSymbol("__do_clear_bss");

  // FIXME: We can disable __do_copy_data if there are no static RAM variables.

  OutStreamer->emitRawComment(
      " Declaring this symbol tells the CRT that it should");
  OutStreamer->emitRawComment(
      "copy all variables from program memory to RAM on startup");
  OutStreamer->emitSymbolAttribute(DoCopyData, MCSA_Global);

  OutStreamer->emitRawComment(
      " Declaring this symbol tells the CRT that it should");
  OutStreamer->emitRawComment("clear the zeroed data section on startup");
  OutStreamer->emitSymbolAttribute(DoClearBss, MCSA_Global);
  */

  return AsmPrinter::doFinalization(M);
}

void GameBoyAsmPrinter::emitStartOfAsmFile(Module &M) {
  const GameBoyTargetMachine &TM = (const GameBoyTargetMachine &)MMI->getTarget();
  const GameBoySubtarget *SubTM = (const GameBoySubtarget *)TM.getSubtargetImpl();
  if (!SubTM)
    return;

  // Always jump to main function
  OutStreamer->emitRawText(StringRef("\tjp\tmain\n"));
  // Emit enough space for the header.
  OutStreamer->emitRawText(StringRef("\tds $150 - @, 0\n"));

  // Get the first function
  
  // OutStreamer->emitAssignment(
  //     MMI->getContext().getOrCreateSymbol(StringRef("test_definition")),
  //     MCConstantExpr::create(SubTM->getIORegSPL(), MMI->getContext()));
  /*
  // Emit __tmp_reg__.
  OutStreamer->emitAssignment(
      MMI->getContext().getOrCreateSymbol(StringRef("__tmp_reg__")),
      MCConstantExpr::create(SubTM->getRegTmpIndex(), MMI->getContext()));
  // Emit __zero_reg__.
  OutStreamer->emitAssignment(
      MMI->getContext().getOrCreateSymbol(StringRef("__zero_reg__")),
      MCConstantExpr::create(SubTM->getRegZeroIndex(), MMI->getContext()));
  // Emit __SREG__.
  OutStreamer->emitAssignment(
      MMI->getContext().getOrCreateSymbol(StringRef("__SREG__")),
      MCConstantExpr::create(SubTM->getIORegSREG(), MMI->getContext()));
  // Emit __SP_H__ if available.
  if (!SubTM->hasSmallStack())
    OutStreamer->emitAssignment(
        MMI->getContext().getOrCreateSymbol(StringRef("__SP_H__")),
        MCConstantExpr::create(SubTM->getIORegSPH(), MMI->getContext()));
  // Emit __SP_L__.
  // Emit __EIND__ if available.
  if (SubTM->hasEIJMPCALL())
    OutStreamer->emitAssignment(
        MMI->getContext().getOrCreateSymbol(StringRef("__EIND__")),
        MCConstantExpr::create(SubTM->getIORegEIND(), MMI->getContext()));
  // Emit __RAMPZ__ if available.
  if (SubTM->hasELPM())
    OutStreamer->emitAssignment(
        MMI->getContext().getOrCreateSymbol(StringRef("__RAMPZ__")),
        MCConstantExpr::create(SubTM->getIORegRAMPZ(), MMI->getContext()));
  */
}

} // end of namespace llvm

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeGameBoyAsmPrinter() {
  llvm::RegisterAsmPrinter<llvm::GameBoyAsmPrinter> X(llvm::getTheGameBoyTarget());
}
