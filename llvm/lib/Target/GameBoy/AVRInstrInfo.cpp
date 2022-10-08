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

void GameBoyInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                               MachineBasicBlock::iterator MI,
                               const DebugLoc &DL, MCRegister DestReg,
                               MCRegister SrcReg, bool KillSrc) const {
  const GameBoySubtarget &STI = MBB.getParent()->getSubtarget<GameBoySubtarget>();
  const GameBoyRegisterInfo &TRI = *STI.getRegisterInfo();
  unsigned Opc;

  // Not all GameBoy devices support the 16-bit `MOVW` instruction.
  if (GameBoy::DREGSRegClass.contains(DestReg, SrcReg)) {
    if (STI.hasMOVW() && GameBoy::DREGSMOVWRegClass.contains(DestReg, SrcReg)) {
      BuildMI(MBB, MI, DL, get(GameBoy::MOVWRdRr), DestReg)
          .addReg(SrcReg, getKillRegState(KillSrc));
    } else {
      Register DestLo, DestHi, SrcLo, SrcHi;

      TRI.splitReg(DestReg, DestLo, DestHi);
      TRI.splitReg(SrcReg, SrcLo, SrcHi);

      // Copy each individual register with the `MOV` instruction.
      BuildMI(MBB, MI, DL, get(GameBoy::MOVRdRr), DestLo)
          .addReg(SrcLo, getKillRegState(KillSrc));
      BuildMI(MBB, MI, DL, get(GameBoy::MOVRdRr), DestHi)
          .addReg(SrcHi, getKillRegState(KillSrc));
    }
  } else {
    if (GameBoy::GPR8RegClass.contains(DestReg, SrcReg)) {
      Opc = GameBoy::MOVRdRr;
    } else if (SrcReg == GameBoy::SP && GameBoy::DREGSRegClass.contains(DestReg)) {
      Opc = GameBoy::SPREAD;
    } else if (DestReg == GameBoy::SP && GameBoy::DREGSRegClass.contains(SrcReg)) {
      Opc = GameBoy::SPWRITE;
    } else {
      llvm_unreachable("Impossible reg-to-reg copy");
    }

    BuildMI(MBB, MI, DL, get(Opc), DestReg)
        .addReg(SrcReg, getKillRegState(KillSrc));
  }
}

unsigned GameBoyInstrInfo::isLoadFromStackSlot(const MachineInstr &MI,
                                           int &FrameIndex) const {
  switch (MI.getOpcode()) {
  case GameBoy::LDDRdPtrQ:
  case GameBoy::LDDWRdYQ: { //: FIXME: remove this once PR13375 gets fixed
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
  case GameBoy::STDPtrQRr:
  case GameBoy::STDWPtrQRr: {
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
    Opcode = GameBoy::STDPtrQRr;
  } else if (TRI->isTypeLegalForClass(*RC, MVT::i16)) {
    Opcode = GameBoy::STDWPtrQRr;
  } else {
    llvm_unreachable("Cannot store this register into a stack slot!");
  }

  BuildMI(MBB, MI, DL, get(Opcode))
      .addFrameIndex(FrameIndex)
      .addImm(0)
      .addReg(SrcReg, getKillRegState(isKill))
      .addMemOperand(MMO);
}

void GameBoyInstrInfo::loadRegFromStackSlot(MachineBasicBlock &MBB,
                                        MachineBasicBlock::iterator MI,
                                        Register DestReg, int FrameIndex,
                                        const TargetRegisterClass *RC,
                                        const TargetRegisterInfo *TRI) const {
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
    Opcode = GameBoy::LDDRdPtrQ;
  } else if (TRI->isTypeLegalForClass(*RC, MVT::i16)) {
    // Opcode = GameBoy::LDDWRdPtrQ;
    //: FIXME: remove this once PR13375 gets fixed
    Opcode = GameBoy::LDDWRdYQ;
  } else {
    llvm_unreachable("Cannot load this register from a stack slot!");
  }

  BuildMI(MBB, MI, DL, get(Opcode), DestReg)
      .addFrameIndex(FrameIndex)
      .addImm(0)
      .addMemOperand(MMO);
}

const MCInstrDesc &GameBoyInstrInfo::getBrCond(GameBoyCC::CondCodes CC) const {
  switch (CC) {
  default:
    llvm_unreachable("Unknown condition code!");
  case GameBoyCC::COND_EQ:
    return get(GameBoy::BREQk);
  case GameBoyCC::COND_NE:
    return get(GameBoy::BRNEk);
  case GameBoyCC::COND_GE:
    return get(GameBoy::BRGEk);
  case GameBoyCC::COND_LT:
    return get(GameBoy::BRLTk);
  case GameBoyCC::COND_SH:
    return get(GameBoy::BRSHk);
  case GameBoyCC::COND_LO:
    return get(GameBoy::BRLOk);
  case GameBoyCC::COND_MI:
    return get(GameBoy::BRMIk);
  case GameBoyCC::COND_PL:
    return get(GameBoy::BRPLk);
  }
}

GameBoyCC::CondCodes GameBoyInstrInfo::getCondFromBranchOpc(unsigned Opc) const {
  switch (Opc) {
  default:
    return GameBoyCC::COND_INVALID;
  case GameBoy::BREQk:
    return GameBoyCC::COND_EQ;
  case GameBoy::BRNEk:
    return GameBoyCC::COND_NE;
  case GameBoy::BRSHk:
    return GameBoyCC::COND_SH;
  case GameBoy::BRLOk:
    return GameBoyCC::COND_LO;
  case GameBoy::BRMIk:
    return GameBoyCC::COND_MI;
  case GameBoy::BRPLk:
    return GameBoyCC::COND_PL;
  case GameBoy::BRGEk:
    return GameBoyCC::COND_GE;
  case GameBoy::BRLTk:
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
    if (I->getOpcode() == GameBoy::RJMPk) {
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
        BuildMI(MBB, UnCondBrIter, MBB.findDebugLoc(I), get(GameBoy::RJMPk))
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
    auto &MI = *BuildMI(&MBB, DL, get(GameBoy::RJMPk)).addMBB(TBB);
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
    auto &MI = *BuildMI(&MBB, DL, get(GameBoy::RJMPk)).addMBB(FBB);
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
    if (I->getOpcode() != GameBoy::RJMPk &&
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
  case GameBoy::JMPk:
  case GameBoy::CALLk:
  case GameBoy::RCALLk:
  case GameBoy::RJMPk:
  case GameBoy::BREQk:
  case GameBoy::BRNEk:
  case GameBoy::BRSHk:
  case GameBoy::BRLOk:
  case GameBoy::BRMIk:
  case GameBoy::BRPLk:
  case GameBoy::BRGEk:
  case GameBoy::BRLTk:
    return MI.getOperand(0).getMBB();
  case GameBoy::BRBSsk:
  case GameBoy::BRBCsk:
    return MI.getOperand(1).getMBB();
  case GameBoy::SBRCRrB:
  case GameBoy::SBRSRrB:
  case GameBoy::SBICAb:
  case GameBoy::SBISAb:
    llvm_unreachable("unimplemented branch instructions");
  }
}

bool GameBoyInstrInfo::isBranchOffsetInRange(unsigned BranchOp,
                                         int64_t BrOffset) const {

  switch (BranchOp) {
  default:
    llvm_unreachable("unexpected opcode!");
  case GameBoy::JMPk:
  case GameBoy::CALLk:
    return true;
  case GameBoy::RCALLk:
  case GameBoy::RJMPk:
    return isIntN(13, BrOffset);
  case GameBoy::BRBSsk:
  case GameBoy::BRBCsk:
  case GameBoy::BREQk:
  case GameBoy::BRNEk:
  case GameBoy::BRSHk:
  case GameBoy::BRLOk:
  case GameBoy::BRMIk:
  case GameBoy::BRPLk:
  case GameBoy::BRGEk:
  case GameBoy::BRLTk:
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
  BuildMI(&MBB, DL, get(GameBoy::JMPk)).addMBB(&NewDestBB);
}

} // end of namespace llvm
