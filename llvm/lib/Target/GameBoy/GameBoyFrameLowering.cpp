//===-- GameBoyFrameLowering.cpp - GameBoy Frame Information ----------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the GameBoy implementation of TargetFrameLowering class.
//
//===----------------------------------------------------------------------===//

#include "GameBoyFrameLowering.h"

#include "GameBoy.h"
#include "GameBoyInstrInfo.h"
#include "GameBoyMachineFunctionInfo.h"
#include "GameBoyTargetMachine.h"
#include "MCTargetDesc/GameBoyMCTargetDesc.h"

#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/IR/Function.h"

#include <vector>

namespace llvm {

GameBoyFrameLowering::GameBoyFrameLowering()
    : TargetFrameLowering(TargetFrameLowering::StackGrowsDown, Align(1), -2) {}

bool GameBoyFrameLowering::canSimplifyCallFramePseudos(
    const MachineFunction &MF) const {
  // Always simplify call frame pseudo instructions, even when
  // hasReservedCallFrame is false.
  return true;
}

bool GameBoyFrameLowering::hasReservedCallFrame(const MachineFunction &MF) const {
  // Reserve call frame memory in function prologue under the following
  // conditions:
  // - Y pointer is reserved to be the frame pointer.
  // - The function does not contain variable sized objects.

  const MachineFrameInfo &MFI = MF.getFrameInfo();
  return hasFP(MF) && !MFI.hasVarSizedObjects();
}

uint64_t GameBoyFrameLowering::computeStackSize(MachineFunction &MF) const {
  // Frame info for this MF.
  MachineFrameInfo *MFI = MF.getFrameInfo();
  uint64_t stackSize = MFI->getStackSize();

  // We appear to be aligning the stack
  unsigned stackAlignment = getStackAlignment();
  if (stackAlignment > 0) {
    // Do stack alignment stuff here.
  }

  return stackSize;
}

// This implements the custom calling convention I've come up with. It probably isn't very good.
// TODO: Implement interrupt routines.
// MF contains a list of MBBs, whilst an MBB is a list of MIs. 
void GameBoyFrameLowering::emitPrologue(MachineFunction &MF, MachineBasicBlock &MBB) const {
  // Collect our info for this specific machine function.
  GameBoyMachineFunctionInfo *FunctionInfo = MF.getInfo<GameBoyMachineFunctionInfo>();
  // Create an iterator for our machine instructions.
  MachineBasicBlock::iterator MBBI = MBB.begin();
  // Get DebugLoc
  DebugLoc debugLoc = MBBI != MBB.end() ? MBBI->getDebugLoc() : DebugLoc();

  // Accommodate for the stack.
  auto stackSize = computeStackSize(MF);
  if (!stackSize) {
    // We do not need to push or pop from the stack, great work.
    return;
  }

}

// emit epilogue
// pop all values off the stack before return is issued, so that PC can be found and restored.

/*
void GameBoyFrameLowering::emitPrologue(MachineFunction &MF,
                                    MachineBasicBlock &MBB) const {
  MachineBasicBlock::iterator MBBI = MBB.begin();
  DebugLoc DL = (MBBI != MBB.end()) ? MBBI->getDebugLoc() : DebugLoc();
  const GameBoySubtarget &STI = MF.getSubtarget<GameBoySubtarget>();
  const GameBoyInstrInfo &TII = *STI.getInstrInfo();
  const GameBoyMachineFunctionInfo *AFI = MF.getInfo<GameBoyMachineFunctionInfo>();
  bool HasFP = hasFP(MF);

  // Interrupt handlers re-enable interrupts in function entry.
  if (AFI->isInterruptHandler()) {
    BuildMI(MBB, MBBI, DL, TII.get(GameBoy::BSETs))
        .addImm(0x07)
        .setMIFlag(MachineInstr::FrameSetup);
  }

  // Emit special prologue code to save R1, R0 and SREG in interrupt/signal
  // handlers before saving any other registers.
  if (AFI->isInterruptOrSignalHandler()) {
    BuildMI(MBB, MBBI, DL, TII.get(GameBoy::PUSHWRr))
        .addReg(GameBoy::R1R0, RegState::Kill)
        .setMIFlag(MachineInstr::FrameSetup);

    BuildMI(MBB, MBBI, DL, TII.get(GameBoy::INRdA), GameBoy::R0)
        .addImm(STI.getIORegSREG())
        .setMIFlag(MachineInstr::FrameSetup);
    BuildMI(MBB, MBBI, DL, TII.get(GameBoy::PUSHRr))
        .addReg(GameBoy::R0, RegState::Kill)
        .setMIFlag(MachineInstr::FrameSetup);
    BuildMI(MBB, MBBI, DL, TII.get(GameBoy::EORRdRr))
        .addReg(GameBoy::R1, RegState::Define)
        .addReg(GameBoy::R1, RegState::Kill)
        .addReg(GameBoy::R1, RegState::Kill)
        .setMIFlag(MachineInstr::FrameSetup);
  }

  // Early exit if the frame pointer is not needed in this function.
  if (!HasFP) {
    return;
  }

  const MachineFrameInfo &MFI = MF.getFrameInfo();
  unsigned FrameSize = MFI.getStackSize() - AFI->getCalleeSavedFrameSize();

  // Skip the callee-saved push instructions.
  while (
      (MBBI != MBB.end()) && MBBI->getFlag(MachineInstr::FrameSetup) &&
      (MBBI->getOpcode() == GameBoy::PUSHRr || MBBI->getOpcode() == GameBoy::PUSHWRr)) {
    ++MBBI;
  }

  // Update Y with the new base value.
  BuildMI(MBB, MBBI, DL, TII.get(GameBoy::SPREAD), GameBoy::R29R28)
      .addReg(GameBoy::SP)
      .setMIFlag(MachineInstr::FrameSetup);

  // Mark the FramePtr as live-in in every block except the entry.
  for (MachineBasicBlock &MBBJ : llvm::drop_begin(MF)) {
    MBBJ.addLiveIn(GameBoy::R29R28);
  }

  if (!FrameSize) {
    return;
  }

  // Reserve the necessary frame memory by doing FP -= <size>.
  unsigned Opcode = (isUInt<6>(FrameSize)) ? GameBoy::SBIWRdK : GameBoy::SUBIWRdK;

  MachineInstr *MI = BuildMI(MBB, MBBI, DL, TII.get(Opcode), GameBoy::R29R28)
                         .addReg(GameBoy::R29R28, RegState::Kill)
                         .addImm(FrameSize)
                         .setMIFlag(MachineInstr::FrameSetup);
  // The SREG implicit def is dead.
  MI->getOperand(3).setIsDead();

  // Write back R29R28 to SP and temporarily disable interrupts.
  BuildMI(MBB, MBBI, DL, TII.get(GameBoy::SPWRITE), GameBoy::SP)
      .addReg(GameBoy::R29R28)
      .setMIFlag(MachineInstr::FrameSetup);
}
*/

static void restoreStatusRegister(MachineFunction &MF, MachineBasicBlock &MBB) {
  const GameBoyMachineFunctionInfo *AFI = MF.getInfo<GameBoyMachineFunctionInfo>();

  MachineBasicBlock::iterator MBBI = MBB.getLastNonDebugInstr();

  DebugLoc DL = MBBI->getDebugLoc();
  const GameBoySubtarget &STI = MF.getSubtarget<GameBoySubtarget>();
  const GameBoyInstrInfo &TII = *STI.getInstrInfo();

  // Emit special epilogue code to restore R1, R0 and SREG in interrupt/signal
  // handlers at the very end of the function, just before reti.
  if (AFI->isInterruptOrSignalHandler()) {
    BuildMI(MBB, MBBI, DL, TII.get(GameBoy::POPRd), GameBoy::R0);
    BuildMI(MBB, MBBI, DL, TII.get(GameBoy::OUTARr))
        .addImm(STI.getIORegSREG())
        .addReg(GameBoy::R0, RegState::Kill);
    BuildMI(MBB, MBBI, DL, TII.get(GameBoy::POPWRd), GameBoy::R1R0);
  }
}

void GameBoyFrameLowering::emitEpilogue(MachineFunction &MF,
                                    MachineBasicBlock &MBB) const {
  const GameBoyMachineFunctionInfo *AFI = MF.getInfo<GameBoyMachineFunctionInfo>();

  // Early exit if the frame pointer is not needed in this function except for
  // signal/interrupt handlers where special code generation is required.
  if (!hasFP(MF) && !AFI->isInterruptOrSignalHandler()) {
    return;
  }

  MachineBasicBlock::iterator MBBI = MBB.getLastNonDebugInstr();
  assert(MBBI->getDesc().isReturn() &&
         "Can only insert epilog into returning blocks");

  DebugLoc DL = MBBI->getDebugLoc();
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  unsigned FrameSize = MFI.getStackSize() - AFI->getCalleeSavedFrameSize();
  const GameBoySubtarget &STI = MF.getSubtarget<GameBoySubtarget>();
  const GameBoyInstrInfo &TII = *STI.getInstrInfo();

  // Early exit if there is no need to restore the frame pointer.
  if (!FrameSize && !MF.getFrameInfo().hasVarSizedObjects()) {
    restoreStatusRegister(MF, MBB);
    return;
  }

  // Skip the callee-saved pop instructions.
  while (MBBI != MBB.begin()) {
    MachineBasicBlock::iterator PI = std::prev(MBBI);
    int Opc = PI->getOpcode();

    if (Opc != GameBoy::POPRd && Opc != GameBoy::POPWRd && !PI->isTerminator()) {
      break;
    }

    --MBBI;
  }

  if (FrameSize) {
    unsigned Opcode;

    // Select the optimal opcode depending on how big it is.
    if (isUInt<6>(FrameSize)) {
      Opcode = GameBoy::ADIWRdK;
    } else {
      Opcode = GameBoy::SUBIWRdK;
      FrameSize = -FrameSize;
    }

    // Restore the frame pointer by doing FP += <size>.
    MachineInstr *MI = BuildMI(MBB, MBBI, DL, TII.get(Opcode), GameBoy::R29R28)
                           .addReg(GameBoy::R29R28, RegState::Kill)
                           .addImm(FrameSize);
    // The SREG implicit def is dead.
    MI->getOperand(3).setIsDead();
  }

  // Write back R29R28 to SP and temporarily disable interrupts.
  BuildMI(MBB, MBBI, DL, TII.get(GameBoy::SPWRITE), GameBoy::SP)
      .addReg(GameBoy::R29R28, RegState::Kill);

  restoreStatusRegister(MF, MBB);
}

// Return true if the specified function should have a dedicated frame
// pointer register. This is true if the function meets any of the following
// conditions:
//  - a register has been spilled
//  - has allocas
//  - input arguments are passed using the stack
//
// Notice that strictly this is not a frame pointer because it contains SP after
// frame allocation instead of having the original SP in function entry.
bool GameBoyFrameLowering::hasFP(const MachineFunction &MF) const {
  const GameBoyMachineFunctionInfo *FuncInfo = MF.getInfo<GameBoyMachineFunctionInfo>();

  return (FuncInfo->getHasSpills() || FuncInfo->getHasAllocas() ||
          FuncInfo->getHasStackArgs() ||
          MF.getFrameInfo().hasVarSizedObjects());
}

bool GameBoyFrameLowering::spillCalleeSavedRegisters(
    MachineBasicBlock &MBB, MachineBasicBlock::iterator MI,
    ArrayRef<CalleeSavedInfo> CSI, const TargetRegisterInfo *TRI) const {
  if (CSI.empty()) {
    return false;
  }

  unsigned CalleeFrameSize = 0;
  DebugLoc DL = MBB.findDebugLoc(MI);
  MachineFunction &MF = *MBB.getParent();
  const GameBoySubtarget &STI = MF.getSubtarget<GameBoySubtarget>();
  const TargetInstrInfo &TII = *STI.getInstrInfo();
  GameBoyMachineFunctionInfo *GameBoyFI = MF.getInfo<GameBoyMachineFunctionInfo>();

  for (const CalleeSavedInfo &I : llvm::reverse(CSI)) {
    Register Reg = I.getReg();
    bool IsNotLiveIn = !MBB.isLiveIn(Reg);

    assert(TRI->getRegSizeInBits(*TRI->getMinimalPhysRegClass(Reg)) == 8 &&
           "Invalid register size");

    // Add the callee-saved register as live-in only if it is not already a
    // live-in register, this usually happens with arguments that are passed
    // through callee-saved registers.
    if (IsNotLiveIn) {
      MBB.addLiveIn(Reg);
    }

    // Do not kill the register when it is an input argument.
    BuildMI(MBB, MI, DL, TII.get(GameBoy::PUSHRr))
        .addReg(Reg, getKillRegState(IsNotLiveIn))
        .setMIFlag(MachineInstr::FrameSetup);
    ++CalleeFrameSize;
  }

  GameBoyFI->setCalleeSavedFrameSize(CalleeFrameSize);

  return true;
}

bool GameBoyFrameLowering::restoreCalleeSavedRegisters(
    MachineBasicBlock &MBB, MachineBasicBlock::iterator MI,
    MutableArrayRef<CalleeSavedInfo> CSI, const TargetRegisterInfo *TRI) const {
  if (CSI.empty()) {
    return false;
  }

  DebugLoc DL = MBB.findDebugLoc(MI);
  const MachineFunction &MF = *MBB.getParent();
  const GameBoySubtarget &STI = MF.getSubtarget<GameBoySubtarget>();
  const TargetInstrInfo &TII = *STI.getInstrInfo();

  for (const CalleeSavedInfo &CCSI : CSI) {
    Register Reg = CCSI.getReg();

    assert(TRI->getRegSizeInBits(*TRI->getMinimalPhysRegClass(Reg)) == 8 &&
           "Invalid register size");

    BuildMI(MBB, MI, DL, TII.get(GameBoy::POPRd), Reg);
  }

  return true;
}

/// Replace pseudo store instructions that pass arguments through the stack with
/// real instructions.
static void fixStackStores(MachineBasicBlock &MBB,
                           MachineBasicBlock::iterator StartMI,
                           const TargetInstrInfo &TII) {
  // Iterate through the BB until we hit a call instruction or we reach the end.
  for (MachineInstr &MI :
       llvm::make_early_inc_range(llvm::make_range(StartMI, MBB.end()))) {
    if (MI.isCall())
      break;

    unsigned Opcode = MI.getOpcode();

    // Only care of pseudo store instructions where SP is the base pointer.
    if (Opcode != GameBoy::STDSPQRr && Opcode != GameBoy::STDWSPQRr)
      continue;

    assert(MI.getOperand(0).getReg() == GameBoy::SP &&
           "SP is expected as base pointer");

    // Replace this instruction with a regular store. Use Y as the base
    // pointer since it is guaranteed to contain a copy of SP.
    unsigned STOpc =
        (Opcode == GameBoy::STDWSPQRr) ? GameBoy::STDWPtrQRr : GameBoy::STDPtrQRr;

    MI.setDesc(TII.get(STOpc));
    MI.getOperand(0).setReg(GameBoy::R31R30);
  }
}

MachineBasicBlock::iterator GameBoyFrameLowering::eliminateCallFramePseudoInstr(
    MachineFunction &MF, MachineBasicBlock &MBB,
    MachineBasicBlock::iterator MI) const {
  const GameBoySubtarget &STI = MF.getSubtarget<GameBoySubtarget>();
  const GameBoyInstrInfo &TII = *STI.getInstrInfo();

  if (hasReservedCallFrame(MF)) {
    return MBB.erase(MI);
  }

  DebugLoc DL = MI->getDebugLoc();
  unsigned int Opcode = MI->getOpcode();
  int Amount = TII.getFrameSize(*MI);

  if (Amount == 0) {
    return MBB.erase(MI);
  }

  assert(getStackAlign() == Align(1) && "Unsupported stack alignment");

  if (Opcode == TII.getCallFrameSetupOpcode()) {
    // Update the stack pointer.
    // In many cases this can be done far more efficiently by pushing the
    // relevant values directly to the stack. However, doing that correctly
    // (in the right order, possibly skipping some empty space for undef
    // values, etc) is tricky and thus left to be optimized in the future.
    BuildMI(MBB, MI, DL, TII.get(GameBoy::SPREAD), GameBoy::R31R30).addReg(GameBoy::SP);

    MachineInstr *New =
        BuildMI(MBB, MI, DL, TII.get(GameBoy::SUBIWRdK), GameBoy::R31R30)
            .addReg(GameBoy::R31R30, RegState::Kill)
            .addImm(Amount);
    New->getOperand(3).setIsDead();

    BuildMI(MBB, MI, DL, TII.get(GameBoy::SPWRITE), GameBoy::SP).addReg(GameBoy::R31R30);

    // Make sure the remaining stack stores are converted to real store
    // instructions.
    fixStackStores(MBB, MI, TII);
  } else {
    assert(Opcode == TII.getCallFrameDestroyOpcode());

    // Note that small stack changes could be implemented more efficiently
    // with a few pop instructions instead of the 8-9 instructions now
    // required.

    // Select the best opcode to adjust SP based on the offset size.
    unsigned AddOpcode;

    if (isUInt<6>(Amount)) {
      AddOpcode = GameBoy::ADIWRdK;
    } else {
      AddOpcode = GameBoy::SUBIWRdK;
      Amount = -Amount;
    }

    // Build the instruction sequence.
    BuildMI(MBB, MI, DL, TII.get(GameBoy::SPREAD), GameBoy::R31R30).addReg(GameBoy::SP);

    MachineInstr *New = BuildMI(MBB, MI, DL, TII.get(AddOpcode), GameBoy::R31R30)
                            .addReg(GameBoy::R31R30, RegState::Kill)
                            .addImm(Amount);
    New->getOperand(3).setIsDead();

    BuildMI(MBB, MI, DL, TII.get(GameBoy::SPWRITE), GameBoy::SP)
        .addReg(GameBoy::R31R30, RegState::Kill);
  }

  return MBB.erase(MI);
}

void GameBoyFrameLowering::determineCalleeSaves(MachineFunction &MF,
                                            BitVector &SavedRegs,
                                            RegScavenger *RS) const {
  TargetFrameLowering::determineCalleeSaves(MF, SavedRegs, RS);

  // If we have a frame pointer, the Y register needs to be saved as well.
  if (hasFP(MF)) {
    SavedRegs.set(GameBoy::R29);
    SavedRegs.set(GameBoy::R28);
  }
}
/// The frame analyzer pass.
///
/// Scans the function for allocas and used arguments
/// that are passed through the stack.
/// TODO: Update with new stack frame layout.
struct GameBoyFrameAnalyzer : public MachineFunctionPass {
  static char ID;
  GameBoyFrameAnalyzer() : MachineFunctionPass(ID) {}

  bool runOnMachineFunction(MachineFunction &MF) override {
    const MachineFrameInfo &MFI = MF.getFrameInfo();
    GameBoyMachineFunctionInfo *AFI = MF.getInfo<GameBoyMachineFunctionInfo>();

    // If there are no fixed frame indexes during this stage it means there
    // are allocas present in the function.
    if (MFI.getNumObjects() != MFI.getNumFixedObjects()) {
      // Check for the type of allocas present in the function. We only care
      // about fixed size allocas so do not give false positives if only
      // variable sized allocas are present.
      for (unsigned i = 0, e = MFI.getObjectIndexEnd(); i != e; ++i) {
        // Variable sized objects have size 0.
        if (MFI.getObjectSize(i)) {
          AFI->setHasAllocas(true);
          break;
        }
      }
    }

    // If there are fixed frame indexes present, scan the function to see if
    // they are really being used.
    if (MFI.getNumFixedObjects() == 0) {
      return false;
    }

    // Ok fixed frame indexes present, now scan the function to see if they
    // are really being used, otherwise we can ignore them.
    for (const MachineBasicBlock &BB : MF) {
      for (const MachineInstr &MI : BB) {
        int Opcode = MI.getOpcode();

        if ((Opcode != GameBoy::LDDRdPtrQ) && (Opcode != GameBoy::LDDWRdPtrQ) &&
            (Opcode != GameBoy::STDPtrQRr) && (Opcode != GameBoy::STDWPtrQRr)) {
          continue;
        }

        for (const MachineOperand &MO : MI.operands()) {
          if (!MO.isFI()) {
            continue;
          }

          if (MFI.isFixedObjectIndex(MO.getIndex())) {
            AFI->setHasStackArgs(true);
            return false;
          }
        }
      }
    }

    return false;
  }

  StringRef getPassName() const override { return "GameBoy Frame Analyzer"; }
};

char GameBoyFrameAnalyzer::ID = 0;

/// Creates instance of the frame analyzer pass.
FunctionPass *createGameBoyFrameAnalyzerPass() { return new GameBoyFrameAnalyzer(); }

} // end of namespace llvm
