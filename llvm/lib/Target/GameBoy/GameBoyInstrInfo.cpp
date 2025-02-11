//===-- GameBoyInstrInfo.cpp - GameBoy Instruction Information --------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the GameBoy implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "GameBoyInstrInfo.h"

#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/MachineConstantPool.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/Function.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"

#include "GameBoy.h"
#include "GameBoyMachineFunctionInfo.h"
#include "GameBoyRegisterInfo.h"
#include "GameBoyTargetMachine.h"
#include "MCTargetDesc/GameBoyMCTargetDesc.h"

#define GET_INSTRINFO_CTOR_DTOR
#include "GameBoyGenInstrInfo.inc"

namespace llvm {

GameBoyInstrInfo::GameBoyInstrInfo()
    : GameBoyGenInstrInfo(GameBoy::ADJCALLSTACKDOWN, GameBoy::ADJCALLSTACKUP), RI() {}

// A smaller and leaner version of the original AVR copyPhysReg.
// The Game Boy can only directly copy to and from an 8-bit register,
// but we have a pseudo instruction for copying from RP to RP.
void GameBoyInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                               MachineBasicBlock::iterator MI,
                               const DebugLoc &DL, MCRegister DestReg,
                               MCRegister SrcReg, bool KillSrc) const {
  const GameBoySubtarget &STI = MBB.getParent()->getSubtarget<GameBoySubtarget>();
  const GameBoyRegisterInfo &TRI = *STI.getRegisterInfo();
  unsigned Opc;

  // Check that we are copying from 8-bit to 8-bit registers.
  if (GameBoy::GPRRegClass.contains(DestReg, SrcReg)) {
    Opc = GameBoy::LDRdRr;

    BuildMI(MBB, MI, DL, get(Opc), DestReg).
      addReg(SrcReg, getKillRegState(KillSrc));

  // Otherwise handle a copy from an 16-bit to a 16-bit register
  } else if (GameBoy::GPRPairRegClass.contains(DestReg, SrcReg)) {
    Opc = GameBoy::LDRdPairRrPair;

    BuildMI(MBB, MI, DL, get(Opc), DestReg).
      addReg(SrcReg, getKillRegState(KillSrc));
  } else {
    // We have a case where the registers are in different classes.
    // Therefore we must handle copies from 8-bit registers to 16-bit registers.
    if (GameBoy::GPRRegClass.contains(SrcReg, SrcReg)) {
      // Split the register pair in half.
      Register DestLo, DestHi;

      // if (GameBoy::GPRRegClass.contains(DestReg, DestReg))
      //   dbgs() << "Attempting to split an 8-bit destination register\n";
      // if (GameBoy::GPRPairRegClass.contains(DestReg, DestReg))
      //   dbgs() << "Attempting to split a 16-bit destination register\n";

      // If an AVR destination register has been selected, then allocate a Game Boy register instead.
      if (GameBoy::GPR8RegClass.contains(DestReg, DestReg) || GameBoy::DREGSRegClass.contains(DestReg, DestReg)) {
        // We cannot perform narrowing copies, so do not handle
        // this circumstance.
        assert(!GameBoy::DREGSRegClass.contains(DestReg, DestReg) && "Cannot perform narrowing copy from 16-bit AVR reg!");
        // Give me more information about this current instruction.        
        auto mbbname = MBB.getFullName();
        dbgs() << "Attempting to change AVR register to Game Boy register...\n";
        dbgs() << "\t MBB Name: " << mbbname;
        dbgs() << "\n\t Instruction: " << *MI;

        // Is it possible to perhaps find a new register to access?
        auto availableRegisters = TRI.getAllocatableSet(*MBB.getParent(), &GameBoy::GPRPairRegClass);
        dbgs() << "\tNumber of available registers: " << availableRegisters.count() << "\n";
        auto replacementRegID = availableRegisters.find_first();
        MCRegister replacementReg;
        assert((replacementRegID != -1) && "No available Game Boy registers!");

        // Find out which register class we are moving to.
        // To avoid problems, we should ideally select
        // a 16-bit destination register as this is always
        // possible to move into.
        for (int i = 0; i < availableRegisters.count(); i++) {
          // See if the back register is available
          dbgs() << "\tExamining register " << MCRegister(availableRegisters.find_last()) << "\n";
          if (GameBoy::GPRPairRegClass.contains(MCRegister(availableRegisters.find_last()))) {
            replacementReg = MCRegister(availableRegisters.find_last());
            break; 
          } else {
            availableRegisters.flip(availableRegisters.find_last());
          }
        }

        // Ensure that we found a replacement register.
        assert(replacementReg && "Could not find a 16-bit destination register for Game Boy!");
        dbgs() << "\tAllocating to register " << replacementReg << "\n";

        // Actually swap the register.
        // Mark it as a define since it is not in use currently.
        // (*MI).getOperand(0).ChangeToRegister(replacementReg, true);

        // Before we change the value of DestReg, we need to make sure that we handle all other
        // instructions that are expecting it as a value.
        // DestReg = replacementReg;
        dbgs() << "\n\tNew instruction: " << *MI;        
      }

      dbgs() << "Splitting register " << DestReg.id() << "\n";
      TRI.splitReg(DestReg, DestLo, DestHi);
      // Load an immediate 0 into the upper half.
      BuildMI(MBB, MI, DL, get(GameBoy::LDRdImm8), DestHi).addImm(0);
      // Copy to the lower reg.
      BuildMI(MBB, MI, DL, get(GameBoy::LDRdRr), DestLo).addReg(SrcReg, getKillRegState(KillSrc));
    } else {
      // We cannot do narrowing copies
      dbgs() << "DstReg: " << DestReg << " SrcReg: " << SrcReg << "\n";
      llvm_unreachable("Impossible to copy from a 16-bit register to an 8-bit register");
    }
  }

  // Check if we are copying to or from the stack pointer
  if (DestReg == GameBoy::SP) {
    // LD SP, HL
    if (SrcReg == GameBoy::RHRL) {
      Opc = GameBoy::LDSPHL;
      BuildMI(MBB, MI, DL, get(GameBoy::LDSPHL), SrcReg).
        addReg(DestReg, getKillRegState(KillSrc));
    } else {
      // Not possible to copy another register to SP
      llvm_unreachable("Impossible copy from reg to SP");
    }
  } else if (SrcReg == GameBoy::SP) {
      llvm_unreachable("Impossible to copy from SP to reg");
  }
}

unsigned GameBoyInstrInfo::isLoadFromStackSlot(const MachineInstr &MI,
                                           int &FrameIndex) const {
  switch (MI.getOpcode()) {
  case GameBoy::LDRdPtrQ:
  case GameBoy::LDRdPairPtrQ: {
    if (MI.getOperand(1).isFI() && MI.getOperand(2).isImm() &&
        MI.getOperand(2).getImm() == 0) {
      FrameIndex = MI.getOperand(1).getIndex();
      return MI.getOperand(0).getReg();
    }
    break;
  }
  default:
    break;
  }

  return 0;
}

unsigned GameBoyInstrInfo::isStoreToStackSlot(const MachineInstr &MI,
                                          int &FrameIndex) const {
  switch (MI.getOpcode()) {
  case GameBoy::LDPtrQRd:
  case GameBoy::LDPtrQRdPair: {
    if (MI.getOperand(0).isFI() && MI.getOperand(1).isImm() &&
        MI.getOperand(1).getImm() == 0) {
      FrameIndex = MI.getOperand(0).getIndex();
      return MI.getOperand(2).getReg();
    }
    break;
  }
  default:
    break;
  }

  return 0;
}

void GameBoyInstrInfo::storeRegToStackSlot(MachineBasicBlock &MBB,
                                       MachineBasicBlock::iterator MI,
                                       Register SrcReg, bool isKill,
                                       int FrameIndex,
                                       const TargetRegisterClass *RC,
                                       const TargetRegisterInfo *TRI) const {
  MachineFunction &MF = *MBB.getParent();
  GameBoyMachineFunctionInfo *AFI = MF.getInfo<GameBoyMachineFunctionInfo>();

  // llvm_unreachable("Unimplemented storeRegToStackSlot");
  // /*
  AFI->setHasSpills(true);

  DebugLoc DL;
  if (MI != MBB.end()) {
    DL = MI->getDebugLoc();
  }

  const MachineFrameInfo &MFI = MF.getFrameInfo();

  MachineMemOperand *MMO = MF.getMachineMemOperand(
      MachinePointerInfo::getFixedStack(MF, FrameIndex),
      MachineMemOperand::MOStore, MFI.getObjectSize(FrameIndex),
      MFI.getObjectAlign(FrameIndex));

  unsigned Opcode = 0;
  if (TRI->isTypeLegalForClass(*RC, MVT::i8)) {
    Opcode = GameBoy::LDPtrQRd;
  } else if (TRI->isTypeLegalForClass(*RC, MVT::i16)) {
    Opcode = GameBoy::LDPtrQRdPair;
  } else {
    llvm_unreachable("Cannot store this register into a stack slot!");
  }

  BuildMI(MBB, MI, DL, get(Opcode))
      .addFrameIndex(FrameIndex)
      .addImm(0)
      .addReg(SrcReg, getKillRegState(isKill))
      .addMemOperand(MMO);
  // */
}

void GameBoyInstrInfo::loadRegFromStackSlot(MachineBasicBlock &MBB,
                                        MachineBasicBlock::iterator MI,
                                        Register DestReg, int FrameIndex,
                                        const TargetRegisterClass *RC,
                                        const TargetRegisterInfo *TRI) const {
  
  // llvm_unreachable("Unimplemented loadRegFromStackSlot");
  // /*
  DebugLoc DL;
  if (MI != MBB.end()) {
    DL = MI->getDebugLoc();
  }

  MachineFunction &MF = *MBB.getParent();
  const MachineFrameInfo &MFI = MF.getFrameInfo();

  MachineMemOperand *MMO = MF.getMachineMemOperand(
      MachinePointerInfo::getFixedStack(MF, FrameIndex),
      MachineMemOperand::MOLoad, MFI.getObjectSize(FrameIndex),
      MFI.getObjectAlign(FrameIndex));

  unsigned Opcode = 0;
  if (TRI->isTypeLegalForClass(*RC, MVT::i8)) {
    Opcode = GameBoy::LDRdPtrQ;
  } else if (TRI->isTypeLegalForClass(*RC, MVT::i16)) {
    Opcode = GameBoy::LDRdPairPtrQ;
    //: FIXME: remove this once PR13375 gets fixed
    // Opcode = GameBoy::LDDWRdYQ;
  } else {
    llvm_unreachable("Cannot load this register from a stack slot!");
  }

  BuildMI(MBB, MI, DL, get(Opcode), DestReg)
      .addFrameIndex(FrameIndex)
      .addImm(0)
      .addMemOperand(MMO);
  // */
}

const MCInstrDesc &GameBoyInstrInfo::getBrCond(GameBoyCC::CondCodes CC) const {
  switch (CC) {
  default:
    llvm_unreachable("Unknown condition code!");
  case GameBoyCC::COND_EQ:
    return get(GameBoy::JRZk);
    // return get(GameBoy::JREQk);
    // return get(GameBoy::BREQk);
  case GameBoyCC::COND_NE:
    return get(GameBoy::JRNZk);
  case GameBoyCC::COND_GE:
    return get(GameBoy::JRGTEk);
  case GameBoyCC::COND_LT:
    return get(GameBoy::JRLTk);
  case GameBoyCC::COND_SH:
    // llvm_unreachable("Unknown condition code 0!");
    return get(GameBoy::JRSHk);
  case GameBoyCC::COND_LO:
    // llvm_unreachable("Unknown condition code 1!");
    return get(GameBoy::JRLOk);
  case GameBoyCC::COND_MI:
    llvm_unreachable("Unknown condition code 2!");
    // return get(GameBoy::BRMIk);
  case GameBoyCC::COND_PL:
    llvm_unreachable("Unknown condition code 3!");
    // return get(GameBoy::BRPLk);
  }
}

GameBoyCC::CondCodes GameBoyInstrInfo::getCondFromBranchOpc(unsigned Opc) const {
  switch (Opc) {
  default:
    return GameBoyCC::COND_INVALID;
  case GameBoy::JREQk:
  case GameBoy::JRZk:
    return GameBoyCC::COND_EQ;
  case GameBoy::JRNEk:
  case GameBoy::JRNZk:
    return GameBoyCC::COND_NE;
  case GameBoy::JRSHk:
    return GameBoyCC::COND_SH;
  case GameBoy::JRLOk:
    return GameBoyCC::COND_LO;
  // case GameBoy::BRMIk:
  //   return GameBoyCC::COND_MI;
  // case GameBoy::BRPLk:
  //   return GameBoyCC::COND_PL;
  case GameBoy::JRGTEk:
    return GameBoyCC::COND_GE;
  case GameBoy::JRLTk:
    return GameBoyCC::COND_LT;
  }
}

GameBoyCC::CondCodes GameBoyInstrInfo::getOppositeCondition(GameBoyCC::CondCodes CC) const {
  switch (CC) {
  default:
    llvm_unreachable("Invalid condition!");
  case GameBoyCC::COND_EQ:
    return GameBoyCC::COND_NE;
  case GameBoyCC::COND_NE:
    return GameBoyCC::COND_EQ;
  case GameBoyCC::COND_SH:
    return GameBoyCC::COND_LO;
  case GameBoyCC::COND_LO:
    return GameBoyCC::COND_SH;
  case GameBoyCC::COND_GE:
    return GameBoyCC::COND_LT;
  case GameBoyCC::COND_LT:
    return GameBoyCC::COND_GE;
  case GameBoyCC::COND_MI:
    return GameBoyCC::COND_PL;
  case GameBoyCC::COND_PL:
    return GameBoyCC::COND_MI;
  }
}

bool GameBoyInstrInfo::analyzeBranch(MachineBasicBlock &MBB,
                                 MachineBasicBlock *&TBB,
                                 MachineBasicBlock *&FBB,
                                 SmallVectorImpl<MachineOperand> &Cond,
                                 bool AllowModify) const {
  // Start from the bottom of the block and work up, examining the
  // terminator instructions.
  MachineBasicBlock::iterator I = MBB.end();
  MachineBasicBlock::iterator UnCondBrIter = MBB.end();

  while (I != MBB.begin()) {
    --I;
    if (I->isDebugInstr()) {
      continue;
    }

    // Working from the bottom, when we see a non-terminator
    // instruction, we're done.
    if (!isUnpredicatedTerminator(*I)) {
      break;
    }

    // A terminator that isn't a branch can't easily be handled
    // by this analysis.
    if (!I->getDesc().isBranch()) {
      return true;
    }

    // Handle unconditional branches.
    //: TODO: add here jmp
    // if (I->getOpcode() == GameBoy::RJMPk) {
    if (I->getOpcode() == GameBoy::JRk) {
      UnCondBrIter = I;

      if (!AllowModify) {
        TBB = I->getOperand(0).getMBB();
        continue;
      }

      // If the block has any instructions after a JMP, delete them.
      MBB.erase(std::next(I), MBB.end());

      Cond.clear();
      FBB = nullptr;

      // Delete the JMP if it's equivalent to a fall-through.
      if (MBB.isLayoutSuccessor(I->getOperand(0).getMBB())) {
        TBB = nullptr;
        I->eraseFromParent();
        I = MBB.end();
        UnCondBrIter = MBB.end();
        continue;
      }

      // TBB is used to indicate the unconditinal destination.
      TBB = I->getOperand(0).getMBB();
      continue;
    }

    // Handle conditional branches.
    GameBoyCC::CondCodes BranchCode = getCondFromBranchOpc(I->getOpcode());
    if (BranchCode == GameBoyCC::COND_INVALID) {
      return true; // Can't handle indirect branch.
    }

    // Working from the bottom, handle the first conditional branch.
    if (Cond.empty()) {
      MachineBasicBlock *TargetBB = I->getOperand(0).getMBB();
      if (AllowModify && UnCondBrIter != MBB.end() &&
          MBB.isLayoutSuccessor(TargetBB)) {
        continue;
        // If we can modify the code and it ends in something like:
        //
        //     jCC L1
        //     jmp L2
        //   L1:
        //     ...
        //   L2:
        //
        // Then we can change this to:
        //
        //     jnCC L2
        //   L1:
        //     ...
        //   L2:
        //
        // Which is a bit more efficient.
        // We conditionally jump to the fall-through block.
        BranchCode = getOppositeCondition(BranchCode);
        unsigned JNCC = getBrCond(BranchCode).getOpcode();
        MachineBasicBlock::iterator OldInst = I;

        BuildMI(MBB, UnCondBrIter, MBB.findDebugLoc(I), get(JNCC))
            .addMBB(UnCondBrIter->getOperand(0).getMBB());
        // BuildMI(MBB, UnCondBrIter, MBB.findDebugLoc(I), get(GameBoy::RJMPk))
        BuildMI(MBB, UnCondBrIter, MBB.findDebugLoc(I), get(GameBoy::JRk))
            .addMBB(TargetBB);

        OldInst->eraseFromParent();
        UnCondBrIter->eraseFromParent();

        // Restart the analysis.
        UnCondBrIter = MBB.end();
        I = MBB.end();
        continue;
      }

      FBB = TBB;
      TBB = I->getOperand(0).getMBB();
      Cond.push_back(MachineOperand::CreateImm(BranchCode));
      continue;
    }

    // Handle subsequent conditional branches. Only handle the case where all
    // conditional branches branch to the same destination.
    assert(Cond.size() == 1);
    assert(TBB);

    // Only handle the case where all conditional branches branch to
    // the same destination.
    if (TBB != I->getOperand(0).getMBB()) {
      return true;
    }

    GameBoyCC::CondCodes OldBranchCode = (GameBoyCC::CondCodes)Cond[0].getImm();
    // If the conditions are the same, we can leave them alone.
    if (OldBranchCode == BranchCode) {
      continue;
    }

    return true;
  }

  return false;
}

unsigned GameBoyInstrInfo::insertBranch(MachineBasicBlock &MBB,
                                    MachineBasicBlock *TBB,
                                    MachineBasicBlock *FBB,
                                    ArrayRef<MachineOperand> Cond,
                                    const DebugLoc &DL, int *BytesAdded) const {
  if (BytesAdded)
    *BytesAdded = 0;

  // Shouldn't be a fall through.
  assert(TBB && "insertBranch must not be told to insert a fallthrough");
  assert((Cond.size() == 1 || Cond.size() == 0) &&
         "GameBoy branch conditions have one component!");

  if (Cond.empty()) {
    assert(!FBB && "Unconditional branch with multiple successors!");
    // auto &MI = *BuildMI(&MBB, DL, get(GameBoy::RJMPk)).addMBB(TBB);
    auto &MI = *BuildMI(&MBB, DL, get(GameBoy::JRk)).addMBB(TBB);
    if (BytesAdded)
      *BytesAdded += getInstSizeInBytes(MI);
    return 1;
  }

  // Conditional branch.
  unsigned Count = 0;
  GameBoyCC::CondCodes CC = (GameBoyCC::CondCodes)Cond[0].getImm();
  auto &CondMI = *BuildMI(&MBB, DL, getBrCond(CC)).addMBB(TBB);

  if (BytesAdded)
    *BytesAdded += getInstSizeInBytes(CondMI);
  ++Count;

  if (FBB) {
    // Two-way Conditional branch. Insert the second branch.
    // auto &MI = *BuildMI(&MBB, DL, get(GameBoy::RJMPk)).addMBB(FBB);
    auto &MI = *BuildMI(&MBB, DL, get(GameBoy::JRk)).addMBB(FBB);
    if (BytesAdded)
      *BytesAdded += getInstSizeInBytes(MI);
    ++Count;
  }

  return Count;
}

unsigned GameBoyInstrInfo::removeBranch(MachineBasicBlock &MBB,
                                    int *BytesRemoved) const {
  if (BytesRemoved)
    *BytesRemoved = 0;

  MachineBasicBlock::iterator I = MBB.end();
  unsigned Count = 0;

  while (I != MBB.begin()) {
    --I;
    if (I->isDebugInstr()) {
      continue;
    }
    //: TODO: add here the missing jmp instructions once they are implemented
    // like jmp, {e}ijmp, and other cond branches, ...
    // if (I->getOpcode() != GameBoy::RJMPk &&
    if (I->getOpcode() != GameBoy::JRk &&
        getCondFromBranchOpc(I->getOpcode()) == GameBoyCC::COND_INVALID) {
      break;
    }

    // Remove the branch.
    if (BytesRemoved)
      *BytesRemoved += getInstSizeInBytes(*I);
    I->eraseFromParent();
    I = MBB.end();
    ++Count;
  }

  return Count;
}

bool GameBoyInstrInfo::reverseBranchCondition(
    SmallVectorImpl<MachineOperand> &Cond) const {
  assert(Cond.size() == 1 && "Invalid GameBoy branch condition!");

  GameBoyCC::CondCodes CC = static_cast<GameBoyCC::CondCodes>(Cond[0].getImm());
  Cond[0].setImm(getOppositeCondition(CC));

  return false;
}

unsigned GameBoyInstrInfo::getInstSizeInBytes(const MachineInstr &MI) const {
  unsigned Opcode = MI.getOpcode();

  switch (Opcode) {
  // A regular instruction
  default: {
    const MCInstrDesc &Desc = get(Opcode);
    return Desc.getSize();
  }
  case TargetOpcode::EH_LABEL:
  case TargetOpcode::IMPLICIT_DEF:
  case TargetOpcode::KILL:
  case TargetOpcode::DBG_VALUE:
    return 0;
  case TargetOpcode::INLINEASM:
  case TargetOpcode::INLINEASM_BR: {
    const MachineFunction &MF = *MI.getParent()->getParent();
    const GameBoyTargetMachine &TM =
        static_cast<const GameBoyTargetMachine &>(MF.getTarget());
    const GameBoySubtarget &STI = MF.getSubtarget<GameBoySubtarget>();
    const TargetInstrInfo &TII = *STI.getInstrInfo();

    return TII.getInlineAsmLength(MI.getOperand(0).getSymbolName(),
                                  *TM.getMCAsmInfo());
  }
  }
}

MachineBasicBlock *
GameBoyInstrInfo::getBranchDestBlock(const MachineInstr &MI) const {
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("unexpected opcode!");
  case GameBoy::JP:
  // case GameBoy::JMPk:
  // case GameBoy::CALLk:
  // case GameBoy::RCALLk:
  // case GameBoy::RJMPk:
  case GameBoy::JRNEk:
  case GameBoy::JREQk:
  case GameBoy::JRZk:
  case GameBoy::JRNZk:
  case GameBoy::JRSHk:
  case GameBoy::JRLOk:
  // case GameBoy::JRMIk:
  // case GameBoy::JRPLk:
  case GameBoy::JRGTEk:
  case GameBoy::JRLTk:
  case GameBoy::JRk:
  case GameBoy::JRCk:
  case GameBoy::JRNCk:
    return MI.getOperand(0).getMBB();
  // case GameBoy::BRBSsk:
  // case GameBoy::BRBCsk:
  //   return MI.getOperand(1).getMBB();
  // case GameBoy::SBRCRrB:
  // case GameBoy::SBRSRrB:
  // case GameBoy::SBICAb:
  // case GameBoy::SBISAb:
    // llvm_unreachable("unimplemented branch instructions");
  }
}

bool GameBoyInstrInfo::isBranchOffsetInRange(unsigned BranchOp,
                                         int64_t BrOffset) const {

  switch (BranchOp) {
  default:
    llvm_unreachable("unexpected opcode!");
  case GameBoy::JP:
  // case GameBoy::JMPk:
  // case GameBoy::CALLk:
    return true;
  // case GameBoy::RCALLk:
  // case GameBoy::RJMPk:
  //   return isIntN(13, BrOffset);
  // case GameBoy::BRBSsk:
  // case GameBoy::BRBCsk:
  // case GameBoy::JRMIk:
  // case GameBoy::JRPLk:
  case GameBoy::JRSHk:
  case GameBoy::JRLOk:
  case GameBoy::JREQk:
  case GameBoy::JRNEk:
  case GameBoy::JRZk:
  case GameBoy::JRNZk:
  case GameBoy::JRk:
  case GameBoy::JRGTEk:
  case GameBoy::JRLTk:
  case GameBoy::JRCk:
  case GameBoy::JRNCk:
    return isIntN(7, BrOffset);
  }
}

void GameBoyInstrInfo::insertIndirectBranch(MachineBasicBlock &MBB,
                                        MachineBasicBlock &NewDestBB,
                                        MachineBasicBlock &RestoreBB,
                                        const DebugLoc &DL, int64_t BrOffset,
                                        RegScavenger *RS) const {
  // This method inserts a *direct* branch (JMP), despite its name.
  // LLVM calls this method to fixup unconditional branches; it never calls
  // insertBranch or some hypothetical "insertDirectBranch".
  // See lib/CodeGen/RegisterRelaxation.cpp for details.
  // We end up here when a jump is too long for a RJMP instruction.
  BuildMI(&MBB, DL, get(GameBoy::JP)).addMBB(&NewDestBB);
}

} // end of namespace llvm
