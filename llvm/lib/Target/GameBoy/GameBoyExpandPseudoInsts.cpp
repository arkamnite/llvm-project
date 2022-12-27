//===-- GameBoyExpandPseudoInsts.cpp - Expand pseudo instructions -------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains a pass that expands pseudo instructions into target
// instructions. This pass should be run after register allocation but before
// the post-regalloc scheduling pass.
//
//===----------------------------------------------------------------------===//

#include "GameBoy.h"
#include "GameBoyInstrInfo.h"
#include "GameBoyTargetMachine.h"
#include "MCTargetDesc/GameBoyMCTargetDesc.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"

using namespace llvm;

#define GameBoy_EXPAND_PSEUDO_NAME "GameBoy pseudo instruction expansion pass"

namespace {

/// Expands "placeholder" instructions marked as pseudo into
/// actual GameBoy instructions.
class GameBoyExpandPseudo : public MachineFunctionPass {
public:
  static char ID;

  GameBoyExpandPseudo() : MachineFunctionPass(ID) {
    initializeGameBoyExpandPseudoPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;

  StringRef getPassName() const override { return GameBoy_EXPAND_PSEUDO_NAME; }

private:
  typedef MachineBasicBlock Block;
  typedef Block::iterator BlockIt;

  const GameBoyRegisterInfo *TRI;
  const TargetInstrInfo *TII;

  /// The register to be used for temporary storage.
  const Register SCRATCH_REGISTER = GameBoy::R0;
  /// The register that will always contain zero.
  const Register ZERO_REGISTER = GameBoy::R1;

  bool expandMBB(Block &MBB);
  bool expandMI(Block &MBB, BlockIt MBBI);
  template <unsigned OP> bool expand(Block &MBB, BlockIt MBBI);

  MachineInstrBuilder buildMI(Block &MBB, BlockIt MBBI, unsigned Opcode) {
    return BuildMI(MBB, MBBI, MBBI->getDebugLoc(), TII->get(Opcode));
  }

  MachineInstrBuilder buildMI(Block &MBB, BlockIt MBBI, unsigned Opcode,
                              Register DstReg) {
    return BuildMI(MBB, MBBI, MBBI->getDebugLoc(), TII->get(Opcode), DstReg);
  }

  MachineRegisterInfo &getRegInfo(Block &MBB) {
    return MBB.getParent()->getRegInfo();
  }

  bool expandArith(unsigned OpLo, unsigned OpHi, Block &MBB, BlockIt MBBI);
  bool expandLogic(unsigned Op, Block &MBB, BlockIt MBBI);
  bool expandLogicImm(unsigned Op, Block &MBB, BlockIt MBBI);
  bool isLogicImmOpRedundant(unsigned Op, unsigned ImmVal) const;

  template <typename Func> bool expandAtomic(Block &MBB, BlockIt MBBI, Func f);

  template <typename Func>
  bool expandAtomicBinaryOp(unsigned Opcode, Block &MBB, BlockIt MBBI, Func f);

  bool expandAtomicBinaryOp(unsigned Opcode, Block &MBB, BlockIt MBBI);

  /// Specific shift implementation for int8.
  bool expandLSLB7Rd(Block &MBB, BlockIt MBBI);
  bool expandLSRB7Rd(Block &MBB, BlockIt MBBI);
  bool expandASRB6Rd(Block &MBB, BlockIt MBBI);
  bool expandASRB7Rd(Block &MBB, BlockIt MBBI);

  /// Specific shift implementation for int16.
  bool expandLSLW4Rd(Block &MBB, BlockIt MBBI);
  bool expandLSRW4Rd(Block &MBB, BlockIt MBBI);
  bool expandASRW7Rd(Block &MBB, BlockIt MBBI);
  bool expandLSLW8Rd(Block &MBB, BlockIt MBBI);
  bool expandLSRW8Rd(Block &MBB, BlockIt MBBI);
  bool expandASRW8Rd(Block &MBB, BlockIt MBBI);
  bool expandLSLW12Rd(Block &MBB, BlockIt MBBI);
  bool expandLSRW12Rd(Block &MBB, BlockIt MBBI);
  bool expandASRW14Rd(Block &MBB, BlockIt MBBI);
  bool expandASRW15Rd(Block &MBB, BlockIt MBBI);

  // Common implementation of LPMWRdZ and ELPMWRdZ.
  bool expandLPMWELPMW(Block &MBB, BlockIt MBBI, bool IsExt);

  /// Scavenges a free GPR8 register for use.
  Register scavengeGPR8(MachineInstr &MI);
};

char GameBoyExpandPseudo::ID = 0;

bool GameBoyExpandPseudo::expandMBB(MachineBasicBlock &MBB) {
  bool Modified = false;

  BlockIt MBBI = MBB.begin(), E = MBB.end();
  while (MBBI != E) {
    BlockIt NMBBI = std::next(MBBI);
    Modified |= expandMI(MBB, MBBI);
    MBBI = NMBBI;
  }

  return Modified;
}

bool GameBoyExpandPseudo::runOnMachineFunction(MachineFunction &MF) {
  bool Modified = false;

  const GameBoySubtarget &STI = MF.getSubtarget<GameBoySubtarget>();
  TRI = STI.getRegisterInfo();
  TII = STI.getInstrInfo();

  // We need to track liveness in order to use register scavenging.
  MF.getProperties().set(MachineFunctionProperties::Property::TracksLiveness);

  for (Block &MBB : MF) {
    bool ContinueExpanding = true;
    unsigned ExpandCount = 0;

    // Continue expanding the block until all pseudos are expanded.
    do {
      assert(ExpandCount < 10 && "pseudo expand limit reached");
      (void)ExpandCount;

      bool BlockModified = expandMBB(MBB);
      Modified |= BlockModified;
      ExpandCount++;

      ContinueExpanding = BlockModified;
    } while (ContinueExpanding);
  }

  return Modified;
}

bool GameBoyExpandPseudo::expandArith(unsigned OpLo, unsigned OpHi, Block &MBB,
                                  BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg, DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(2).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool SrcIsKill = MI.getOperand(2).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  buildMI(MBB, MBBI, OpLo)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstLoReg, getKillRegState(DstIsKill))
      .addReg(SrcLoReg, getKillRegState(SrcIsKill));

  auto MIBHI =
      buildMI(MBB, MBBI, OpHi)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill))
          .addReg(SrcHiReg, getKillRegState(SrcIsKill));

  if (ImpIsDead)
    MIBHI->getOperand(3).setIsDead();

  // SREG is always implicitly killed
  MIBHI->getOperand(4).setIsKill();

  MI.eraseFromParent();
  return true;
}

bool GameBoyExpandPseudo::expandLogic(unsigned Op, Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg, DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(2).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool SrcIsKill = MI.getOperand(2).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  auto MIBLO =
      buildMI(MBB, MBBI, Op)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstLoReg, getKillRegState(DstIsKill))
          .addReg(SrcLoReg, getKillRegState(SrcIsKill));

  // SREG is always implicitly dead
  MIBLO->getOperand(3).setIsDead();

  auto MIBHI =
      buildMI(MBB, MBBI, Op)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill))
          .addReg(SrcHiReg, getKillRegState(SrcIsKill));

  if (ImpIsDead)
    MIBHI->getOperand(3).setIsDead();

  MI.eraseFromParent();
  return true;
}

bool GameBoyExpandPseudo::isLogicImmOpRedundant(unsigned Op,
                                            unsigned ImmVal) const {

  // ANDI Rd, 0xff is redundant.
  if (Op == GameBoy::ANDIRdK && ImmVal == 0xff)
    return true;

  // ORI Rd, 0x0 is redundant.
  if (Op == GameBoy::ORIRdK && ImmVal == 0x0)
    return true;

  return false;
}

bool GameBoyExpandPseudo::expandLogicImm(unsigned Op, Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  unsigned Imm = MI.getOperand(2).getImm();
  unsigned Lo8 = Imm & 0xff;
  unsigned Hi8 = (Imm >> 8) & 0xff;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  if (!isLogicImmOpRedundant(Op, Lo8)) {
    auto MIBLO =
        buildMI(MBB, MBBI, Op)
            .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
            .addReg(DstLoReg, getKillRegState(SrcIsKill))
            .addImm(Lo8);

    // SREG is always implicitly dead
    MIBLO->getOperand(3).setIsDead();
  }

  if (!isLogicImmOpRedundant(Op, Hi8)) {
    auto MIBHI =
        buildMI(MBB, MBBI, Op)
            .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
            .addReg(DstHiReg, getKillRegState(SrcIsKill))
            .addImm(Hi8);

    if (ImpIsDead)
      MIBHI->getOperand(3).setIsDead();
  }

  MI.eraseFromParent();
  return true;
}

//===----------------------------------------------------------------------===//
//===----------------------------------------------------------------------===//
// Game Boy Pseudo Instructions
//===----------------------------------------------------------------------===//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Loading
//===----------------------------------------------------------------------===//
template <>
bool GameBoyExpandPseudo::expand<GameBoy::LDRd8Ptr>(Block &MBB, BlockIt MBBI) {
  // LD Rd (RR) where RR is a register pair.
  // If LD A, (RR), then we must select LD A, (HL) or LD A (BC/DE)

  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  MachineInstrBuilder MINew;

  // If we are attempting to load a pointer into any other register than
  // RA, then we must first move the pointer to RA, then LD Rd, A

  // Select now from HL, or BC/DE
  auto rclass = getRegInfo(MBB).getRegClass(SrcReg);
  if (rclass == &GameBoy::GPRPairPointerHLRegClass) {
    MINew = buildMI(MBB, MBBI, GameBoy::LDRdHLPtr)
      .addReg(GameBoy::RA, RegState::Define)
      .addReg(GameBoy::RHRL, RegState::Define);
  } else {
    MINew = buildMI(MBB, MBBI, GameBoy::LDRdPtr)
      .addReg(GameBoy::RA, RegState::Define)
      .addReg(SrcReg);
  }    

  // Add LD Rd, A if needed
  if (DstReg != GameBoy::RA) {
    buildMI(MBB, MBBI, GameBoy::LDRdRr).addReg(DstReg).addReg(GameBoy::RA);
  }

  MINew.setMemRefs(MI.memoperands());
  MI.eraseFromParent();
  return true;
}

// Transfer the register pair via individual registers.
template <>
bool GameBoyExpandPseudo::expand<GameBoy::LDRdPairRrPair>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg, SrcLoReg, SrcHiReg;
  // Register SrcReg = MI.getPrevNode()->getOperand(0).getReg();
  // dbgs() << "Expanding LDRdPairRrPair\n";
  auto v = MI.getNumOperands();
  dbgs() << "Expanding LDRdPairRrPair, found " << v << " operands in original function\n";
  for (unsigned int i = 0; i < v; i++) {
    dbgs() << MI.getOperand(i) << "\n";
  }
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);
  // LD DstHi, SrcHi
  dbgs() << "Dst: " << MI.getOperand(0) << "\n";
  dbgs() << "Src: " << MI.getOperand(1) << "\n";
  auto ld = buildMI(MBB, MBBI, GameBoy::LDRdRr, DstHiReg).addReg(SrcHiReg, RegState::Define);
  // LD DstLo, SrcLo
  buildMI(MBB, MBBI, GameBoy::LDRdRr, DstLoReg).addReg(SrcLoReg, RegState::Define);
  // remove previous instruction.
  ld.setMemRefs(MI.memoperands());
  MI.removeFromParent();
  return true;
}


//===----------------------------------------------------------------------===//
// Arithmetic
//===----------------------------------------------------------------------===//

void printAllOperands(MachineInstr &MI) {
  for (unsigned int i = 0; i < MI.getNumOperands(); i++) {
    dbgs() << "\t" << MI.getOperand(i) << "\n";
  }
}

template<>
bool GameBoyExpandPseudo::expand<GameBoy::AddRdRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(1).getReg();
  Register SrcReg = MI.getOperand(2).getReg();
  bool SrcIsKill = MI.getOperand(2).isKill();
  bool DstIsDead = MI.getOperand(1).isDead();

  if (SrcReg.id() == DstReg.id()) {  
    dbgs() << "Expanding AddRdRr into Add A Rr\n";
    printAllOperands(MI);
    // ADD A, Rr
    buildMI(MBB, MBBI, GameBoy::AddARr)
      .addReg(GameBoy::RA, RegState::Define)
      .addReg(SrcReg, getKillRegState(SrcIsKill));
  } else {
    dbgs() << "Expanding AddRd(" << DstReg.id() << ") Rr(" << SrcReg.id() << ")\n";
    printAllOperands(MI);
    // LD A, Rr
    buildMI(MBB, MBBI, GameBoy::LDRdRr)
      .addReg(GameBoy::RA, RegState::Define)
      .addReg(SrcReg, getKillRegState(SrcIsKill));
    dbgs() << "A\n";
    // ADD A, Rd
    buildMI(MBB, MBBI, GameBoy::AddARr)
      .addReg(GameBoy::RA, RegState::Define)
      .addReg(DstReg, getDeadRegState(DstIsDead));
    dbgs() << "B\n";
    // LD Rd, A
    buildMI(MBB, MBBI, GameBoy::LDRdRr)
      .addReg(DstReg, RegState::Define)
      .addReg(GameBoy::RA);
    dbgs() << "C\n";
  }
  MI.eraseFromParent();
  return true;
}

// Will expand ADD R Imm8 using A as a temporary.
template<>
bool GameBoyExpandPseudo::expand<GameBoy::AddRdImm8>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
  dbgs() << "Expanding AddRdImm8\n";
  auto Val = MI.getOperand(1).getImm();
  // ADD A, Imm8
  buildMI(MBB, MBBI, GameBoy::AddAImm8, GameBoy::RA).addImm(Val);
  // LD R, A
  buildMI(MBB, MBBI, GameBoy::LDRdRr).addReg(DstReg).addReg(GameBoy::RA);
  MI.eraseFromParent();
  return true;
}

// Will expand ADD RdPair, RrPair into
// LD HL, Rd
// ADD HL, Rr
// LD Rd, HL
template<>
bool GameBoyExpandPseudo::expand<GameBoy::AddRdPairRrPair>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  // Register DstReg = MI.getOperand(0).getReg();
  auto v = MI.getNumOperands();
  dbgs() << "INCOMPLETE Expanding AddRdPairRrPair, found " << v << " operands in original function\n";
  for (unsigned int i = 0; i < v; i++) {
    dbgs() << MI.getOperand(i) << "\n";
  }

  return true;
}
// Will expand ADD Rpd, Imm16 
// TODO: Investigate optimisation using A and HIGH(n) LOW(n) from RGBASM.
template<>
bool GameBoyExpandPseudo::expand<GameBoy::AddRpdImm16>(Block &MBB, BlockIt MBBI) {
  dbgs() << "Expanding AddRpdImm16\n";
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(2).getReg();
  // LD HL, Rpd
  buildMI(MBB, MBBI, GameBoy::LDRdPairRrPair, GameBoy::RHRL).addReg(DstReg, RegState::Define);
  // LD Src, Imm16 happens implicitly.
  // ADD HL, Src
  buildMI(MBB, MBBI, GameBoy::ADDHLPair).addReg(GameBoy::RHRL).addReg(SrcReg, RegState::Define);
  // LD Rpd, HL
  buildMI(MBB, MBBI, GameBoy::LDRdPairRrPair).addReg(DstReg).addReg(GameBoy::RHRL);
  MI.eraseFromParent();
  return true;
}

//===----------------------------------------------------------------------===//
// Logic
//===----------------------------------------------------------------------===//
template<>
bool GameBoyExpandPseudo::expand<GameBoy::AndRdRr>(Block &MBB, BlockIt MBBI) {
  // See if Rd is A; if so, can avoid LD A, Rd
  // MachineInstr &MI = *MBBI;
  // Register DstReg = MI.getOperand(0).getReg();
  // Register SrcReg = MI.getOperand(1).getReg();
  llvm_unreachable("Incomplete AndRdRr");
  return true;
}

template<>
bool GameBoyExpandPseudo::expand<GameBoy::AndRdImm8>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("Incomplete AndRdImm8");
  return true;
}

template<>
bool GameBoyExpandPseudo::expand<GameBoy::OrRdRr>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("Incomplete OrRdRr");
  return true;
}

template<>
bool GameBoyExpandPseudo::expand<GameBoy::OrRdImm8>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("Incomplete OrRdImm8");
  return true;
}

template<>
bool GameBoyExpandPseudo::expand<GameBoy::XorRdRr>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("Incomplete XorRdRr");
  return true;
}

template<>
bool GameBoyExpandPseudo::expand<GameBoy::XorRdImm8>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("Incomplete XorRdImm8");
  return true;
}

template<>
bool GameBoyExpandPseudo::expand<GameBoy::CpRdRr>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("Incomplete CpRdRr");
  return true;
}

template<>
bool GameBoyExpandPseudo::expand<GameBoy::CpRdImm8>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("Incomplete CpRdImm8");
  return true;
}

//===----------------------------------------------------------------------===//
// General purpose arithmetic operands and CPU control instructions
//===----------------------------------------------------------------------===//
template<>
bool GameBoyExpandPseudo::expand<GameBoy::SEXT>(Block &MBB, BlockIt MBBI) {
  bool addDefines = false;
  MachineInstr &MI = *MBBI;
  Register SrcReg = MI.getOperand(1).getReg();
  // This will need to be split into HIGH and LOW.
  Register DstReg = MI.getOperand(0).getReg();
  Register DstHi, DstLow;
  // The Hi and Low are swapped here.
  TRI->splitReg(DstReg, DstHi, DstLow);
  dbgs() << "Expanding SEXT\n";
  for (unsigned int i = 0; i < MI.getNumOperands(); i++) {
    dbgs() << MI.getOperand(i) << "\n";
  } 
  // Low byte is copied as is, then the sign bit is pushed into the carry bit
  // The sbc instruction is exploited to get a 0 or -1 via (A - A - Carry),
  // which is therefore the same as (-Carry). We then store this in the upper
  // byte

  // Store low byte
  if (DstLow.id() != SrcReg.id()) {
    buildMI(MBB, MBBI, GameBoy::LDRdRr).addReg(DstLow).addReg(SrcReg);
    dbgs() << "\tStored low byte in " << TRI->getRegAsmName(DstLow).str() << "\n";
  } else {
    dbgs() << "\tLow byte does not need to be transferred\n";
  }

  // If the source register is not A, then this will need to be accounted for.
  // Push sign into carry with add a, a

  // This is a very rudimentary comparison, as for some reason the class is incorrectly
  // being read as GPR8lo for A, due to the register number possibly.
  auto regClass = TRI->getRegClass(SrcReg);
  auto name = TRI->getRegAsmName(SrcReg).str();
  bool lda = name.compare("RA");
  if (lda) {
    auto n = TRI->getRegClassName(TRI->getRegClass(SrcReg));
    dbgs() << "\t" << name <<" is in " << n << "\n";
    buildMI(MBB, MBBI, GameBoy::LDRdRr).addReg(GameBoy::RA, RegState::Define).addReg(SrcReg, RegState::Define);
    dbgs() << "\tMoved to RA\n";
    buildMI(MBB, MBBI, GameBoy::AddARr).addReg(GameBoy::RA, RegState::Define).addReg(GameBoy::RA);
    dbgs() << "\tPushed sign into carry\n";
    addDefines = true;
  } else {
    dbgs() << "\tNo need for LD A, Src\n";
    buildMI(MBB, MBBI, GameBoy::AddARr).addReg(SrcReg, RegState::Define).addReg(SrcReg);
    dbgs() << "\tPushed sign into carry\n";
  }

  // Convert to 0 or -1
  buildMI(MBB, MBBI, GameBoy::SbcARr).addReg(GameBoy::RA, RegState::Define).addReg(GameBoy::RA);
  dbgs() << "\tConverted to 0 or -1\n";

  // Store high byte
  buildMI(MBB, MBBI, GameBoy::LDRdRr).addReg(DstHi).addReg(GameBoy::RA);
  dbgs() << "\tStored high byte\n";

  // Remove old instruction
  MI.removeFromParent();
  return true;
}


//===----------------------------------------------------------------------===//
//===----------------------------------------------------------------------===//
// AVR Pseudo Instructions
//===----------------------------------------------------------------------===//
//===----------------------------------------------------------------------===//
template <>
bool GameBoyExpandPseudo::expand<GameBoy::ADDWRdRr>(Block &MBB, BlockIt MBBI) {
  return expandArith(GameBoy::ADDRdRr, GameBoy::ADCRdRr, MBB, MBBI);
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::ADCWRdRr>(Block &MBB, BlockIt MBBI) {
  return expandArith(GameBoy::ADCRdRr, GameBoy::ADCRdRr, MBB, MBBI);
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::SUBWRdRr>(Block &MBB, BlockIt MBBI) {
  return expandArith(GameBoy::SUBRdRr, GameBoy::SBCRdRr, MBB, MBBI);
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::SUBIWRdK>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  auto MIBLO =
      buildMI(MBB, MBBI, GameBoy::SUBIRdK)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstLoReg, getKillRegState(SrcIsKill));

  auto MIBHI =
      buildMI(MBB, MBBI, GameBoy::SBCIRdK)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(SrcIsKill));

  switch (MI.getOperand(2).getType()) {
  case MachineOperand::MO_GlobalAddress: {
    const GlobalValue *GV = MI.getOperand(2).getGlobal();
    int64_t Offs = MI.getOperand(2).getOffset();
    unsigned TF = MI.getOperand(2).getTargetFlags();
    MIBLO.addGlobalAddress(GV, Offs, TF | GameBoyII::MO_NEG | GameBoyII::MO_LO);
    MIBHI.addGlobalAddress(GV, Offs, TF | GameBoyII::MO_NEG | GameBoyII::MO_HI);
    break;
  }
  case MachineOperand::MO_Immediate: {
    unsigned Imm = MI.getOperand(2).getImm();
    MIBLO.addImm(Imm & 0xff);
    MIBHI.addImm((Imm >> 8) & 0xff);
    break;
  }
  default:
    llvm_unreachable("Unknown operand type!");
  }

  if (ImpIsDead)
    MIBHI->getOperand(3).setIsDead();

  // SREG is always implicitly killed
  MIBHI->getOperand(4).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::SBCWRdRr>(Block &MBB, BlockIt MBBI) {
  return expandArith(GameBoy::SBCRdRr, GameBoy::SBCRdRr, MBB, MBBI);
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::SBCIWRdK>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  unsigned Imm = MI.getOperand(2).getImm();
  unsigned Lo8 = Imm & 0xff;
  unsigned Hi8 = (Imm >> 8) & 0xff;
  unsigned OpLo = GameBoy::SBCIRdK;
  unsigned OpHi = GameBoy::SBCIRdK;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  auto MIBLO =
      buildMI(MBB, MBBI, OpLo)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstLoReg, getKillRegState(SrcIsKill))
          .addImm(Lo8);

  // SREG is always implicitly killed
  MIBLO->getOperand(4).setIsKill();

  auto MIBHI =
      buildMI(MBB, MBBI, OpHi)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(SrcIsKill))
          .addImm(Hi8);

  if (ImpIsDead)
    MIBHI->getOperand(3).setIsDead();

  // SREG is always implicitly killed
  MIBHI->getOperand(4).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::ANDWRdRr>(Block &MBB, BlockIt MBBI) {
  return expandLogic(GameBoy::ANDRdRr, MBB, MBBI);
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::ANDIWRdK>(Block &MBB, BlockIt MBBI) {
  return expandLogicImm(GameBoy::ANDIRdK, MBB, MBBI);
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::ORWRdRr>(Block &MBB, BlockIt MBBI) {
  return expandLogic(GameBoy::ORRdRr, MBB, MBBI);
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::ORIWRdK>(Block &MBB, BlockIt MBBI) {
  return expandLogicImm(GameBoy::ORIRdK, MBB, MBBI);
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::EORWRdRr>(Block &MBB, BlockIt MBBI) {
  return expandLogic(GameBoy::EORRdRr, MBB, MBBI);
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::COMWRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  unsigned OpLo = GameBoy::COMRd;
  unsigned OpHi = GameBoy::COMRd;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  auto MIBLO =
      buildMI(MBB, MBBI, OpLo)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstLoReg, getKillRegState(DstIsKill));

  // SREG is always implicitly dead
  MIBLO->getOperand(2).setIsDead();

  auto MIBHI =
      buildMI(MBB, MBBI, OpHi)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill));

  if (ImpIsDead)
    MIBHI->getOperand(2).setIsDead();

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::NEGWRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Do NEG on the upper byte.
  auto MIBHI =
      buildMI(MBB, MBBI, GameBoy::NEGRd)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, RegState::Kill);
  // SREG is always implicitly dead
  MIBHI->getOperand(2).setIsDead();

  // Do NEG on the lower byte.
  buildMI(MBB, MBBI, GameBoy::NEGRd)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstLoReg, getKillRegState(DstIsKill));

  // Do an extra SBC.
  auto MISBCI =
      buildMI(MBB, MBBI, GameBoy::SBCRdRr)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill))
          .addReg(ZERO_REGISTER);
  if (ImpIsDead)
    MISBCI->getOperand(3).setIsDead();
  // SREG is always implicitly killed
  MISBCI->getOperand(4).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::CPWRdRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg, DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool DstIsKill = MI.getOperand(0).isKill();
  bool SrcIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  unsigned OpLo = GameBoy::CPRdRr;
  unsigned OpHi = GameBoy::CPCRdRr;
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Low part
  buildMI(MBB, MBBI, OpLo)
      .addReg(DstLoReg, getKillRegState(DstIsKill))
      .addReg(SrcLoReg, getKillRegState(SrcIsKill));

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
                   .addReg(DstHiReg, getKillRegState(DstIsKill))
                   .addReg(SrcHiReg, getKillRegState(SrcIsKill));

  if (ImpIsDead)
    MIBHI->getOperand(2).setIsDead();

  // SREG is always implicitly killed
  MIBHI->getOperand(3).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::CPCWRdRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg, DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool DstIsKill = MI.getOperand(0).isKill();
  bool SrcIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  unsigned OpLo = GameBoy::CPCRdRr;
  unsigned OpHi = GameBoy::CPCRdRr;
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
                   .addReg(DstLoReg, getKillRegState(DstIsKill))
                   .addReg(SrcLoReg, getKillRegState(SrcIsKill));

  // SREG is always implicitly killed
  MIBLO->getOperand(3).setIsKill();

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
                   .addReg(DstHiReg, getKillRegState(DstIsKill))
                   .addReg(SrcHiReg, getKillRegState(SrcIsKill));

  if (ImpIsDead)
    MIBHI->getOperand(2).setIsDead();

  // SREG is always implicitly killed
  MIBHI->getOperand(3).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::LDIWRdK>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  unsigned OpLo = GameBoy::LDRdImm8;
  unsigned OpHi = GameBoy::LDRdImm8;
  // unsigned OpLo = GameBoy::LDIRdK;
  // unsigned OpHi = GameBoy::LDIRdK;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  auto MIBLO =
      buildMI(MBB, MBBI, OpLo)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead));

  auto MIBHI =
      buildMI(MBB, MBBI, OpHi)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead));

  switch (MI.getOperand(1).getType()) {
  case MachineOperand::MO_GlobalAddress: {
    const GlobalValue *GV = MI.getOperand(1).getGlobal();
    int64_t Offs = MI.getOperand(1).getOffset();
    unsigned TF = MI.getOperand(1).getTargetFlags();

    MIBLO.addGlobalAddress(GV, Offs, TF | GameBoyII::MO_LO);
    MIBHI.addGlobalAddress(GV, Offs, TF | GameBoyII::MO_HI);
    break;
  }
  case MachineOperand::MO_BlockAddress: {
    const BlockAddress *BA = MI.getOperand(1).getBlockAddress();
    unsigned TF = MI.getOperand(1).getTargetFlags();

    MIBLO.add(MachineOperand::CreateBA(BA, TF | GameBoyII::MO_LO));
    MIBHI.add(MachineOperand::CreateBA(BA, TF | GameBoyII::MO_HI));
    break;
  }
  case MachineOperand::MO_Immediate: {
    unsigned Imm = MI.getOperand(1).getImm();

    MIBLO.addImm(Imm & 0xff);
    MIBHI.addImm((Imm >> 8) & 0xff);
    break;
  }
  default:
    llvm_unreachable("Unknown operand type!");
  }

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::LDSWRdK>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  unsigned OpLo = GameBoy::LDSRdK;
  unsigned OpHi = GameBoy::LDSRdK;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  auto MIBLO =
      buildMI(MBB, MBBI, OpLo)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead));

  auto MIBHI =
      buildMI(MBB, MBBI, OpHi)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead));

  switch (MI.getOperand(1).getType()) {
  case MachineOperand::MO_GlobalAddress: {
    const GlobalValue *GV = MI.getOperand(1).getGlobal();
    int64_t Offs = MI.getOperand(1).getOffset();
    unsigned TF = MI.getOperand(1).getTargetFlags();

    MIBLO.addGlobalAddress(GV, Offs, TF);
    MIBHI.addGlobalAddress(GV, Offs + 1, TF);
    break;
  }
  case MachineOperand::MO_Immediate: {
    unsigned Imm = MI.getOperand(1).getImm();

    MIBLO.addImm(Imm);
    MIBHI.addImm(Imm + 1);
    break;
  }
  default:
    llvm_unreachable("Unknown operand type!");
  }

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::LDWRdPtr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register TmpReg = 0; // 0 for no temporary register
  Register SrcReg = MI.getOperand(1).getReg();
  bool SrcIsKill = MI.getOperand(1).isKill();
  unsigned OpLo = GameBoy::LDRdPtr;
  unsigned OpHi = GameBoy::LDDRdPtrQ;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Use a temporary register if src and dst registers are the same.
  if (DstReg == SrcReg)
    TmpReg = scavengeGPR8(MI);

  Register CurDstLoReg = (DstReg == SrcReg) ? TmpReg : DstLoReg;
  Register CurDstHiReg = (DstReg == SrcReg) ? TmpReg : DstHiReg;

  // Load low byte.
  auto MIBLO = buildMI(MBB, MBBI, OpLo)
                   .addReg(CurDstLoReg, RegState::Define)
                   .addReg(SrcReg);

  // Push low byte onto stack if necessary.
  if (TmpReg)
    buildMI(MBB, MBBI, GameBoy::PUSHRd).addReg(TmpReg);

  // Load high byte.
  auto MIBHI = buildMI(MBB, MBBI, OpHi)
                   .addReg(CurDstHiReg, RegState::Define)
                   .addReg(SrcReg, getKillRegState(SrcIsKill))
                   .addImm(1);

  if (TmpReg) {
    // Move the high byte into the final destination.
    buildMI(MBB, MBBI, GameBoy::MOVRdRr, DstHiReg).addReg(TmpReg);

    // Move the low byte from the scratch space into the final destination.
    buildMI(MBB, MBBI, GameBoy::POPRd, DstLoReg);
  }

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::LDWRdPtrPi>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsDead = MI.getOperand(1).isKill();
  unsigned OpLo = GameBoy::LDRdPtrPi;
  unsigned OpHi = GameBoy::LDRdPtrPi;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  assert(DstReg != SrcReg && "SrcReg and DstReg cannot be the same");

  auto MIBLO =
      buildMI(MBB, MBBI, OpLo)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(SrcReg, RegState::Define)
          .addReg(SrcReg, RegState::Kill);

  auto MIBHI =
      buildMI(MBB, MBBI, OpHi)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(SrcReg, RegState::Define | getDeadRegState(SrcIsDead))
          .addReg(SrcReg, RegState::Kill);

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::LDWRdPtrPd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsDead = MI.getOperand(1).isKill();
  unsigned OpLo = GameBoy::LDRdPtrPd;
  unsigned OpHi = GameBoy::LDRdPtrPd;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  assert(DstReg != SrcReg && "SrcReg and DstReg cannot be the same");

  auto MIBHI =
      buildMI(MBB, MBBI, OpHi)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(SrcReg, RegState::Define)
          .addReg(SrcReg, RegState::Kill);

  auto MIBLO =
      buildMI(MBB, MBBI, OpLo)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(SrcReg, RegState::Define | getDeadRegState(SrcIsDead))
          .addReg(SrcReg, RegState::Kill);

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::LDDWRdPtrQ>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register TmpReg = 0; // 0 for no temporary register
  Register SrcReg = MI.getOperand(1).getReg();
  unsigned Imm = MI.getOperand(2).getImm();
  bool SrcIsKill = MI.getOperand(1).isKill();
  unsigned OpLo = GameBoy::LDDRdPtrQ;
  unsigned OpHi = GameBoy::LDDRdPtrQ;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Since we add 1 to the Imm value for the high byte below, and 63 is the
  // highest Imm value allowed for the instruction, 62 is the limit here.
  assert(Imm <= 62 && "Offset is out of range");

  // Use a temporary register if src and dst registers are the same.
  if (DstReg == SrcReg)
    TmpReg = scavengeGPR8(MI);

  Register CurDstLoReg = (DstReg == SrcReg) ? TmpReg : DstLoReg;
  Register CurDstHiReg = (DstReg == SrcReg) ? TmpReg : DstHiReg;

  // Load low byte.
  auto MIBLO = buildMI(MBB, MBBI, OpLo)
                   .addReg(CurDstLoReg, RegState::Define)
                   .addReg(SrcReg)
                   .addImm(Imm);

  // Push low byte onto stack if necessary.
  if (TmpReg)
    buildMI(MBB, MBBI, GameBoy::PUSHRd).addReg(TmpReg);

  // Load high byte.
  auto MIBHI = buildMI(MBB, MBBI, OpHi)
                   .addReg(CurDstHiReg, RegState::Define)
                   .addReg(SrcReg, getKillRegState(SrcIsKill))
                   .addImm(Imm + 1);

  if (TmpReg) {
    // Move the high byte into the final destination.
    buildMI(MBB, MBBI, GameBoy::MOVRdRr, DstHiReg).addReg(TmpReg);

    // Move the low byte from the scratch space into the final destination.
    buildMI(MBB, MBBI, GameBoy::POPRd, DstLoReg);
  }

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

bool GameBoyExpandPseudo::expandLPMWELPMW(Block &MBB, BlockIt MBBI, bool IsExt) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register TmpReg = 0; // 0 for no temporary register
  Register SrcReg = MI.getOperand(1).getReg();
  bool SrcIsKill = MI.getOperand(1).isKill();
  unsigned OpLo = IsExt ? GameBoy::ELPMRdZPi : GameBoy::LPMRdZPi;
  unsigned OpHi = IsExt ? GameBoy::ELPMRdZ : GameBoy::LPMRdZ;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Set the I/O register RAMPZ for ELPM.
  if (IsExt) {
    const GameBoySubtarget &STI = MBB.getParent()->getSubtarget<GameBoySubtarget>();
    Register Bank = MI.getOperand(2).getReg();
    // out RAMPZ, rtmp
    buildMI(MBB, MBBI, GameBoy::OUTARr).addImm(STI.getIORegRAMPZ()).addReg(Bank);
  }

  // Use a temporary register if src and dst registers are the same.
  if (DstReg == SrcReg)
    TmpReg = scavengeGPR8(MI);

  Register CurDstLoReg = (DstReg == SrcReg) ? TmpReg : DstLoReg;
  Register CurDstHiReg = (DstReg == SrcReg) ? TmpReg : DstHiReg;

  // Load low byte.
  auto MIBLO = buildMI(MBB, MBBI, OpLo)
                   .addReg(CurDstLoReg, RegState::Define)
                   .addReg(SrcReg);

  // Push low byte onto stack if necessary.
  if (TmpReg)
    buildMI(MBB, MBBI, GameBoy::PUSHRd).addReg(TmpReg);

  // Load high byte.
  auto MIBHI = buildMI(MBB, MBBI, OpHi)
                   .addReg(CurDstHiReg, RegState::Define)
                   .addReg(SrcReg, getKillRegState(SrcIsKill));

  if (TmpReg) {
    // Move the high byte into the final destination.
    buildMI(MBB, MBBI, GameBoy::MOVRdRr, DstHiReg).addReg(TmpReg);

    // Move the low byte from the scratch space into the final destination.
    buildMI(MBB, MBBI, GameBoy::POPRd, DstLoReg);
  }

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::LPMWRdZ>(Block &MBB, BlockIt MBBI) {
  return expandLPMWELPMW(MBB, MBBI, false);
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::ELPMWRdZ>(Block &MBB, BlockIt MBBI) {
  return expandLPMWELPMW(MBB, MBBI, true);
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::ELPMBRdZ>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  Register BankReg = MI.getOperand(2).getReg();
  bool SrcIsKill = MI.getOperand(1).isKill();
  const GameBoySubtarget &STI = MBB.getParent()->getSubtarget<GameBoySubtarget>();

  // Set the I/O register RAMPZ for ELPM (out RAMPZ, rtmp).
  buildMI(MBB, MBBI, GameBoy::OUTARr).addImm(STI.getIORegRAMPZ()).addReg(BankReg);

  // Load byte.
  auto MILB = buildMI(MBB, MBBI, GameBoy::ELPMRdZ)
                  .addReg(DstReg, RegState::Define)
                  .addReg(SrcReg, getKillRegState(SrcIsKill));

  MILB.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::LPMWRdZPi>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("16-bit LPMPi is unimplemented");
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::ELPMBRdZPi>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("byte ELPMPi is unimplemented");
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::ELPMWRdZPi>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("16-bit ELPMPi is unimplemented");
}

template <typename Func>
bool GameBoyExpandPseudo::expandAtomic(Block &MBB, BlockIt MBBI, Func f) {
  MachineInstr &MI = *MBBI;
  const GameBoySubtarget &STI = MBB.getParent()->getSubtarget<GameBoySubtarget>();

  // Store the SREG.
  buildMI(MBB, MBBI, GameBoy::INRdA)
      .addReg(SCRATCH_REGISTER, RegState::Define)
      .addImm(STI.getIORegSREG());

  // Disable exceptions.
  buildMI(MBB, MBBI, GameBoy::BCLRs).addImm(7); // CLI

  f(MI);

  // Restore the status reg.
  buildMI(MBB, MBBI, GameBoy::OUTARr)
      .addImm(STI.getIORegSREG())
      .addReg(SCRATCH_REGISTER);

  MI.eraseFromParent();
  return true;
}

template <typename Func>
bool GameBoyExpandPseudo::expandAtomicBinaryOp(unsigned Opcode, Block &MBB,
                                           BlockIt MBBI, Func f) {
  return expandAtomic(MBB, MBBI, [&](MachineInstr &MI) {
    auto Op1 = MI.getOperand(0);
    auto Op2 = MI.getOperand(1);

    MachineInstr &NewInst =
        *buildMI(MBB, MBBI, Opcode).add(Op1).add(Op2).getInstr();
    f(NewInst);
  });
}

bool GameBoyExpandPseudo::expandAtomicBinaryOp(unsigned Opcode, Block &MBB,
                                           BlockIt MBBI) {
  return expandAtomicBinaryOp(Opcode, MBB, MBBI, [](MachineInstr &MI) {});
}

Register GameBoyExpandPseudo::scavengeGPR8(MachineInstr &MI) {
  MachineBasicBlock &MBB = *MI.getParent();
  RegScavenger RS;

  RS.enterBasicBlock(MBB);
  RS.forward(MI);

  BitVector Candidates =
      TRI->getAllocatableSet(*MBB.getParent(), &GameBoy::GPR8RegClass);

  // Exclude all the registers being used by the instruction.
  for (MachineOperand &MO : MI.operands()) {
    if (MO.isReg() && MO.getReg() != 0 && !MO.isDef() &&
        !Register::isVirtualRegister(MO.getReg()))
      Candidates.reset(MO.getReg());
  }

  BitVector Available = RS.getRegsAvailable(&GameBoy::GPR8RegClass);
  Available &= Candidates;

  signed Reg = Available.find_first();
  assert(Reg != -1 && "ran out of registers");
  return Reg;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::AtomicLoad8>(Block &MBB, BlockIt MBBI) {
  return expandAtomicBinaryOp(GameBoy::LDRdPtr, MBB, MBBI);
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::AtomicLoad16>(Block &MBB, BlockIt MBBI) {
  return expandAtomicBinaryOp(GameBoy::LDWRdPtr, MBB, MBBI);
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::AtomicStore8>(Block &MBB, BlockIt MBBI) {
  return expandAtomicBinaryOp(GameBoy::STPtrRr, MBB, MBBI);
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::AtomicStore16>(Block &MBB, BlockIt MBBI) {
  return expandAtomicBinaryOp(GameBoy::STWPtrRr, MBB, MBBI);
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::AtomicFence>(Block &MBB, BlockIt MBBI) {
  // On GameBoy, there is only one core and so atomic fences do nothing.
  MBBI->eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::STSWKRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg;
  Register SrcReg = MI.getOperand(1).getReg();
  bool SrcIsKill = MI.getOperand(1).isKill();
  unsigned OpLo = GameBoy::STSKRr;
  unsigned OpHi = GameBoy::STSKRr;
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);

  // Write the high byte first in case this address belongs to a special
  // I/O address with a special temporary register.
  auto MIBHI = buildMI(MBB, MBBI, OpHi);
  auto MIBLO = buildMI(MBB, MBBI, OpLo);

  switch (MI.getOperand(0).getType()) {
  case MachineOperand::MO_GlobalAddress: {
    const GlobalValue *GV = MI.getOperand(0).getGlobal();
    int64_t Offs = MI.getOperand(0).getOffset();
    unsigned TF = MI.getOperand(0).getTargetFlags();

    MIBLO.addGlobalAddress(GV, Offs, TF);
    MIBHI.addGlobalAddress(GV, Offs + 1, TF);
    break;
  }
  case MachineOperand::MO_Immediate: {
    unsigned Imm = MI.getOperand(0).getImm();

    MIBLO.addImm(Imm);
    MIBHI.addImm(Imm + 1);
    break;
  }
  default:
    llvm_unreachable("Unknown operand type!");
  }

  MIBLO.addReg(SrcLoReg, getKillRegState(SrcIsKill));
  MIBHI.addReg(SrcHiReg, getKillRegState(SrcIsKill));

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::STWPtrRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool DstIsUndef = MI.getOperand(0).isUndef();
  bool SrcIsKill = MI.getOperand(1).isKill();
  unsigned OpLo = GameBoy::STPtrRr;
  unsigned OpHi = GameBoy::STDPtrQRr;
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);

  //: TODO: need to reverse this order like inw and stsw?
  auto MIBLO = buildMI(MBB, MBBI, OpLo)
                   .addReg(DstReg, getUndefRegState(DstIsUndef))
                   .addReg(SrcLoReg, getKillRegState(SrcIsKill));

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
                   .addReg(DstReg, getUndefRegState(DstIsUndef))
                   .addImm(1)
                   .addReg(SrcHiReg, getKillRegState(SrcIsKill));

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::STWPtrPiRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(2).getReg();
  unsigned Imm = MI.getOperand(3).getImm();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(2).isKill();
  unsigned OpLo = GameBoy::STPtrPiRr;
  unsigned OpHi = GameBoy::STPtrPiRr;
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);

  assert(DstReg != SrcReg && "SrcReg and DstReg cannot be the same");

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
                   .addReg(DstReg, RegState::Define)
                   .addReg(DstReg, RegState::Kill)
                   .addReg(SrcLoReg, getKillRegState(SrcIsKill))
                   .addImm(Imm);

  auto MIBHI =
      buildMI(MBB, MBBI, OpHi)
          .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstReg, RegState::Kill)
          .addReg(SrcHiReg, getKillRegState(SrcIsKill))
          .addImm(Imm);

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::STWPtrPdRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(2).getReg();
  unsigned Imm = MI.getOperand(3).getImm();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(2).isKill();
  unsigned OpLo = GameBoy::STPtrPdRr;
  unsigned OpHi = GameBoy::STPtrPdRr;
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);

  assert(DstReg != SrcReg && "SrcReg and DstReg cannot be the same");

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
                   .addReg(DstReg, RegState::Define)
                   .addReg(DstReg, RegState::Kill)
                   .addReg(SrcHiReg, getKillRegState(SrcIsKill))
                   .addImm(Imm);

  auto MIBLO =
      buildMI(MBB, MBBI, OpLo)
          .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstReg, RegState::Kill)
          .addReg(SrcLoReg, getKillRegState(SrcIsKill))
          .addImm(Imm);

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::STDWPtrQRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsKill = MI.getOperand(0).isKill();
  unsigned Imm = MI.getOperand(1).getImm();
  Register SrcReg = MI.getOperand(2).getReg();
  bool SrcIsKill = MI.getOperand(2).isKill();

  // STD's maximum displacement is 63, so larger stores have to be split into a
  // set of operations
  if (Imm >= 63) {
    if (!DstIsKill) {
      buildMI(MBB, MBBI, GameBoy::PUSHWRr).addReg(DstReg);
    }

    buildMI(MBB, MBBI, GameBoy::SUBIWRdK)
        .addReg(DstReg, RegState::Define)
        .addReg(DstReg, RegState::Kill)
        .addImm(-Imm);

    buildMI(MBB, MBBI, GameBoy::STWPtrRr)
        .addReg(DstReg, RegState::Kill)
        .addReg(SrcReg, getKillRegState(SrcIsKill));

    if (!DstIsKill) {
      buildMI(MBB, MBBI, GameBoy::POPWRd).addDef(DstReg, RegState::Define);
    }
  } else {
    unsigned OpLo = GameBoy::STDPtrQRr;
    unsigned OpHi = GameBoy::STDPtrQRr;
    Register SrcLoReg, SrcHiReg;
    TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);

    auto MIBLO = buildMI(MBB, MBBI, OpLo)
                     .addReg(DstReg)
                     .addImm(Imm)
                     .addReg(SrcLoReg, getKillRegState(SrcIsKill));

    auto MIBHI = buildMI(MBB, MBBI, OpHi)
                     .addReg(DstReg, getKillRegState(DstIsKill))
                     .addImm(Imm + 1)
                     .addReg(SrcHiReg, getKillRegState(SrcIsKill));

    MIBLO.setMemRefs(MI.memoperands());
    MIBHI.setMemRefs(MI.memoperands());
  }

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::STDSPQRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  const MachineFunction &MF = *MBB.getParent();
  const GameBoySubtarget &STI = MF.getSubtarget<GameBoySubtarget>();

  assert(MI.getOperand(0).getReg() == GameBoy::SP &&
         "SP is expected as base pointer");

  assert(STI.getFrameLowering()->hasReservedCallFrame(MF) &&
         "unexpected STDSPQRr pseudo instruction");
  (void)STI;

  MI.setDesc(TII->get(GameBoy::STDPtrQRr));
  MI.getOperand(0).setReg(GameBoy::R29R28);

  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::STDWSPQRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  const MachineFunction &MF = *MBB.getParent();
  const GameBoySubtarget &STI = MF.getSubtarget<GameBoySubtarget>();

  assert(MI.getOperand(0).getReg() == GameBoy::SP &&
         "SP is expected as base pointer");

  assert(STI.getFrameLowering()->hasReservedCallFrame(MF) &&
         "unexpected STDWSPQRr pseudo instruction");
  (void)STI;

  MI.setDesc(TII->get(GameBoy::STDWPtrQRr));
  MI.getOperand(0).setReg(GameBoy::R29R28);

  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::INWRdA>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  unsigned Imm = MI.getOperand(1).getImm();
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  unsigned OpLo = GameBoy::INRdA;
  unsigned OpHi = GameBoy::INRdA;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Since we add 1 to the Imm value for the high byte below, and 63 is the
  // highest Imm value allowed for the instruction, 62 is the limit here.
  assert(Imm <= 62 && "Address is out of range");

  auto MIBLO =
      buildMI(MBB, MBBI, OpLo)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addImm(Imm);

  auto MIBHI =
      buildMI(MBB, MBBI, OpHi)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addImm(Imm + 1);

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::OUTWARr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg;
  unsigned Imm = MI.getOperand(0).getImm();
  Register SrcReg = MI.getOperand(1).getReg();
  bool SrcIsKill = MI.getOperand(1).isKill();
  unsigned OpLo = GameBoy::OUTARr;
  unsigned OpHi = GameBoy::OUTARr;
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);

  // Since we add 1 to the Imm value for the high byte below, and 63 is the
  // highest Imm value allowed for the instruction, 62 is the limit here.
  assert(Imm <= 62 && "Address is out of range");

  // 16 bit I/O writes need the high byte first
  auto MIBHI = buildMI(MBB, MBBI, OpHi)
                   .addImm(Imm + 1)
                   .addReg(SrcHiReg, getKillRegState(SrcIsKill));

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
                   .addImm(Imm)
                   .addReg(SrcLoReg, getKillRegState(SrcIsKill));

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::PUSHWRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg;
  Register SrcReg = MI.getOperand(0).getReg();
  bool SrcIsKill = MI.getOperand(0).isKill();
  unsigned Flags = MI.getFlags();
  unsigned OpLo = GameBoy::PUSHRd;
  unsigned OpHi = GameBoy::PUSHRd;
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);

  // Low part
  buildMI(MBB, MBBI, OpLo)
      .addReg(SrcLoReg, getKillRegState(SrcIsKill))
      .setMIFlags(Flags);

  // High part
  buildMI(MBB, MBBI, OpHi)
      .addReg(SrcHiReg, getKillRegState(SrcIsKill))
      .setMIFlags(Flags);

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::POPWRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  unsigned Flags = MI.getFlags();
  unsigned OpLo = GameBoy::POPRd;
  unsigned OpHi = GameBoy::POPRd;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  buildMI(MBB, MBBI, OpHi, DstHiReg).setMIFlags(Flags); // High
  buildMI(MBB, MBBI, OpLo, DstLoReg).setMIFlags(Flags); // Low

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::ROLBRd>(Block &MBB, BlockIt MBBI) {
  // In GameBoy, the rotate instructions behave quite unintuitively. They rotate
  // bits through the carry bit in SREG, effectively rotating over 9 bits,
  // instead of 8. This is useful when we are dealing with numbers over
  // multiple registers, but when we actually need to rotate stuff, we have
  // to explicitly add the carry bit.

  MachineInstr &MI = *MBBI;
  unsigned OpShift, OpCarry;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  OpShift = GameBoy::ADDRdRr;
  OpCarry = GameBoy::ADCRdRr;

  // add r16, r16
  // adc r16, r1

  // Shift part
  buildMI(MBB, MBBI, OpShift)
      .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstReg, RegState::Kill)
      .addReg(DstReg, RegState::Kill);

  // Add the carry bit
  auto MIB = buildMI(MBB, MBBI, OpCarry)
                 .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
                 .addReg(DstReg, getKillRegState(DstIsKill))
                 .addReg(ZERO_REGISTER);

  // SREG is always implicitly killed
  MIB->getOperand(2).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::RORBRd>(Block &MBB, BlockIt MBBI) {
  // In GameBoy, the rotate instructions behave quite unintuitively. They rotate
  // bits through the carry bit in SREG, effectively rotating over 9 bits,
  // instead of 8. This is useful when we are dealing with numbers over
  // multiple registers, but when we actually need to rotate stuff, we have
  // to explicitly add the carry bit.

  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();

  // bst r16, 0
  // ror r16
  // bld r16, 7

  // Move the lowest bit from DstReg into the T bit
  buildMI(MBB, MBBI, GameBoy::BST).addReg(DstReg).addImm(0);

  // Rotate to the right
  buildMI(MBB, MBBI, GameBoy::RORRd, DstReg).addReg(DstReg);

  // Move the T bit into the highest bit of DstReg.
  buildMI(MBB, MBBI, GameBoy::BLD, DstReg).addReg(DstReg).addImm(7);

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::LSLWRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  unsigned OpLo = GameBoy::ADDRdRr; // ADD Rd, Rd <==> LSL Rd
  unsigned OpHi = GameBoy::ADCRdRr; // ADC Rd, Rd <==> ROL Rd
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Low part
  buildMI(MBB, MBBI, OpLo)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstLoReg, getKillRegState(DstIsKill))
      .addReg(DstLoReg, getKillRegState(DstIsKill));

  auto MIBHI =
      buildMI(MBB, MBBI, OpHi)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill))
          .addReg(DstHiReg, getKillRegState(DstIsKill));

  if (ImpIsDead)
    MIBHI->getOperand(3).setIsDead();

  // SREG is always implicitly killed
  MIBHI->getOperand(4).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::LSLWHiRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // add hireg, hireg <==> lsl hireg
  auto MILSL =
      buildMI(MBB, MBBI, GameBoy::ADDRdRr)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill))
          .addReg(DstHiReg, getKillRegState(DstIsKill));

  if (ImpIsDead)
    MILSL->getOperand(3).setIsDead();

  MI.eraseFromParent();
  return true;
}

bool GameBoyExpandPseudo::expandLSLW4Rd(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // swap Rh
  // swap Rl
  buildMI(MBB, MBBI, GameBoy::SWAPRd)
      .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstHiReg, RegState::Kill);
  buildMI(MBB, MBBI, GameBoy::SWAPRd)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstLoReg, RegState::Kill);

  // andi Rh, 0xf0
  auto MI0 =
      buildMI(MBB, MBBI, GameBoy::ANDIRdK)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, RegState::Kill)
          .addImm(0xf0);
  // SREG is implicitly dead.
  MI0->getOperand(3).setIsDead();

  // eor Rh, Rl
  auto MI1 =
      buildMI(MBB, MBBI, GameBoy::EORRdRr)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, RegState::Kill)
          .addReg(DstLoReg);
  // SREG is implicitly dead.
  MI1->getOperand(3).setIsDead();

  // andi Rl, 0xf0
  auto MI2 =
      buildMI(MBB, MBBI, GameBoy::ANDIRdK)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstLoReg, getKillRegState(DstIsKill))
          .addImm(0xf0);
  // SREG is implicitly dead.
  MI2->getOperand(3).setIsDead();

  // eor Rh, Rl
  auto MI3 =
      buildMI(MBB, MBBI, GameBoy::EORRdRr)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill))
          .addReg(DstLoReg);
  if (ImpIsDead)
    MI3->getOperand(3).setIsDead();

  MI.eraseFromParent();
  return true;
}

bool GameBoyExpandPseudo::expandLSLW8Rd(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // mov Rh, Rl
  buildMI(MBB, MBBI, GameBoy::MOVRdRr)
      .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstLoReg);

  // clr Rl
  auto MIBLO =
      buildMI(MBB, MBBI, GameBoy::EORRdRr)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstLoReg, getKillRegState(DstIsKill))
          .addReg(DstLoReg, getKillRegState(DstIsKill));
  if (ImpIsDead)
    MIBLO->getOperand(3).setIsDead();

  MI.eraseFromParent();
  return true;
}

bool GameBoyExpandPseudo::expandLSLW12Rd(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // mov Rh, Rl
  buildMI(MBB, MBBI, GameBoy::MOVRdRr)
      .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstLoReg);

  // swap Rh
  buildMI(MBB, MBBI, GameBoy::SWAPRd)
      .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstHiReg, RegState::Kill);

  // andi Rh, 0xf0
  auto MI0 =
      buildMI(MBB, MBBI, GameBoy::ANDIRdK)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill))
          .addImm(0xf0);
  // SREG is implicitly dead.
  MI0->getOperand(3).setIsDead();

  // clr Rl
  auto MI1 =
      buildMI(MBB, MBBI, GameBoy::EORRdRr)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstLoReg, getKillRegState(DstIsKill))
          .addReg(DstLoReg, getKillRegState(DstIsKill));
  if (ImpIsDead)
    MI1->getOperand(3).setIsDead();

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::LSLWNRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  unsigned Imm = MI.getOperand(2).getImm();
  switch (Imm) {
  case 4:
    return expandLSLW4Rd(MBB, MBBI);
  case 8:
    return expandLSLW8Rd(MBB, MBBI);
  case 12:
    return expandLSLW12Rd(MBB, MBBI);
  default:
    llvm_unreachable("unimplemented lslwn");
    return false;
  }
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::LSRWRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  unsigned OpLo = GameBoy::RORRd;
  unsigned OpHi = GameBoy::LSRRd;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // High part
  buildMI(MBB, MBBI, OpHi)
      .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstHiReg, getKillRegState(DstIsKill));

  auto MIBLO =
      buildMI(MBB, MBBI, OpLo)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstLoReg, getKillRegState(DstIsKill));

  if (ImpIsDead)
    MIBLO->getOperand(2).setIsDead();

  // SREG is always implicitly killed
  MIBLO->getOperand(3).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::LSRWLoRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // lsr loreg
  auto MILSR =
      buildMI(MBB, MBBI, GameBoy::LSRRd)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstLoReg, getKillRegState(DstIsKill));

  if (ImpIsDead)
    MILSR->getOperand(2).setIsDead();

  MI.eraseFromParent();
  return true;
}

bool GameBoyExpandPseudo::expandLSRW4Rd(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // swap Rh
  // swap Rl
  buildMI(MBB, MBBI, GameBoy::SWAPRd)
      .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstHiReg, RegState::Kill);
  buildMI(MBB, MBBI, GameBoy::SWAPRd)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstLoReg, RegState::Kill);

  // andi Rl, 0xf
  auto MI0 =
      buildMI(MBB, MBBI, GameBoy::ANDIRdK)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstLoReg, RegState::Kill)
          .addImm(0xf);
  // SREG is implicitly dead.
  MI0->getOperand(3).setIsDead();

  // eor Rl, Rh
  auto MI1 =
      buildMI(MBB, MBBI, GameBoy::EORRdRr)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstLoReg, RegState::Kill)
          .addReg(DstHiReg);
  // SREG is implicitly dead.
  MI1->getOperand(3).setIsDead();

  // andi Rh, 0xf
  auto MI2 =
      buildMI(MBB, MBBI, GameBoy::ANDIRdK)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill))
          .addImm(0xf);
  // SREG is implicitly dead.
  MI2->getOperand(3).setIsDead();

  // eor Rl, Rh
  auto MI3 =
      buildMI(MBB, MBBI, GameBoy::EORRdRr)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstLoReg, getKillRegState(DstIsKill))
          .addReg(DstHiReg);
  if (ImpIsDead)
    MI3->getOperand(3).setIsDead();

  MI.eraseFromParent();
  return true;
}

bool GameBoyExpandPseudo::expandLSRW8Rd(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Move upper byte to lower byte.
  buildMI(MBB, MBBI, GameBoy::MOVRdRr)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstHiReg);

  // Clear upper byte.
  auto MIBHI =
      buildMI(MBB, MBBI, GameBoy::EORRdRr)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill))
          .addReg(DstHiReg, getKillRegState(DstIsKill));
  if (ImpIsDead)
    MIBHI->getOperand(3).setIsDead();

  MI.eraseFromParent();
  return true;
}

bool GameBoyExpandPseudo::expandLSRW12Rd(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Move upper byte to lower byte.
  buildMI(MBB, MBBI, GameBoy::MOVRdRr)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstHiReg);

  // swap Rl
  buildMI(MBB, MBBI, GameBoy::SWAPRd)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstLoReg, RegState::Kill);

  // andi Rl, 0xf
  auto MI0 =
      buildMI(MBB, MBBI, GameBoy::ANDIRdK)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstLoReg, getKillRegState(DstIsKill))
          .addImm(0xf);
  // SREG is implicitly dead.
  MI0->getOperand(3).setIsDead();

  // Clear upper byte.
  auto MIBHI =
      buildMI(MBB, MBBI, GameBoy::EORRdRr)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill))
          .addReg(DstHiReg, getKillRegState(DstIsKill));
  if (ImpIsDead)
    MIBHI->getOperand(3).setIsDead();

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::LSRWNRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  unsigned Imm = MI.getOperand(2).getImm();
  switch (Imm) {
  case 4:
    return expandLSRW4Rd(MBB, MBBI);
  case 8:
    return expandLSRW8Rd(MBB, MBBI);
  case 12:
    return expandLSRW12Rd(MBB, MBBI);
  default:
    llvm_unreachable("unimplemented lsrwn");
    return false;
  }
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::RORWRd>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("RORW unimplemented");
  return false;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::ROLWRd>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("ROLW unimplemented");
  return false;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::ASRWRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  unsigned OpLo = GameBoy::RORRd;
  unsigned OpHi = GameBoy::ASRRd;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // High part
  buildMI(MBB, MBBI, OpHi)
      .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstHiReg, getKillRegState(DstIsKill));

  auto MIBLO =
      buildMI(MBB, MBBI, OpLo)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstLoReg, getKillRegState(DstIsKill));

  if (ImpIsDead)
    MIBLO->getOperand(2).setIsDead();

  // SREG is always implicitly killed
  MIBLO->getOperand(3).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::ASRWLoRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // asr loreg
  auto MIASR =
      buildMI(MBB, MBBI, GameBoy::ASRRd)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstLoReg, getKillRegState(DstIsKill));

  if (ImpIsDead)
    MIASR->getOperand(2).setIsDead();

  MI.eraseFromParent();
  return true;
}

bool GameBoyExpandPseudo::expandASRW7Rd(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // lsl r24
  // mov r24,r25
  // rol r24
  // sbc r25,r25

  // lsl r24 <=> add r24, r24
  buildMI(MBB, MBBI, GameBoy::ADDRdRr)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstLoReg, RegState::Kill)
      .addReg(DstLoReg, RegState::Kill);

  // mov r24, r25
  buildMI(MBB, MBBI, GameBoy::MOVRdRr)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstHiReg);

  // rol r24 <=> adc r24, r24
  buildMI(MBB, MBBI, GameBoy::ADCRdRr)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstLoReg, getKillRegState(DstIsKill))
      .addReg(DstLoReg, getKillRegState(DstIsKill));

  // sbc r25, r25
  auto MISBC =
      buildMI(MBB, MBBI, GameBoy::SBCRdRr)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill))
          .addReg(DstHiReg, getKillRegState(DstIsKill));

  if (ImpIsDead)
    MISBC->getOperand(3).setIsDead();
  // SREG is always implicitly killed
  MISBC->getOperand(4).setIsKill();

  MI.eraseFromParent();
  return true;
}

bool GameBoyExpandPseudo::expandASRW8Rd(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Move upper byte to lower byte.
  buildMI(MBB, MBBI, GameBoy::MOVRdRr)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstHiReg);

  // Move the sign bit to the C flag.
  buildMI(MBB, MBBI, GameBoy::ADDRdRr)
      .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstHiReg, RegState::Kill)
      .addReg(DstHiReg, RegState::Kill);

  // Set upper byte to 0 or -1.
  auto MIBHI =
      buildMI(MBB, MBBI, GameBoy::SBCRdRr)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, getKillRegState(DstIsKill))
          .addReg(DstHiReg, getKillRegState(DstIsKill));

  if (ImpIsDead)
    MIBHI->getOperand(3).setIsDead();
  // SREG is always implicitly killed
  MIBHI->getOperand(4).setIsKill();

  MI.eraseFromParent();
  return true;
}
bool GameBoyExpandPseudo::expandASRW14Rd(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // lsl r25
  // sbc r24, r24
  // lsl r25
  // mov r25, r24
  // rol r24

  // lsl r25 <=> add r25, r25
  buildMI(MBB, MBBI, GameBoy::ADDRdRr)
      .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstHiReg, RegState::Kill)
      .addReg(DstHiReg, RegState::Kill);

  // sbc r24, r24
  buildMI(MBB, MBBI, GameBoy::SBCRdRr)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstLoReg, RegState::Kill)
      .addReg(DstLoReg, RegState::Kill);

  // lsl r25 <=> add r25, r25
  buildMI(MBB, MBBI, GameBoy::ADDRdRr)
      .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstHiReg, RegState::Kill)
      .addReg(DstHiReg, RegState::Kill);

  // mov r25, r24
  buildMI(MBB, MBBI, GameBoy::MOVRdRr)
      .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstLoReg);

  // rol r24 <=> adc r24, r24
  auto MIROL =
      buildMI(MBB, MBBI, GameBoy::ADCRdRr)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstLoReg, getKillRegState(DstIsKill))
          .addReg(DstLoReg, getKillRegState(DstIsKill));

  if (ImpIsDead)
    MIROL->getOperand(3).setIsDead();
  // SREG is always implicitly killed
  MIROL->getOperand(4).setIsKill();

  MI.eraseFromParent();
  return false;
}

bool GameBoyExpandPseudo::expandASRW15Rd(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool ImpIsDead = MI.getOperand(3).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // lsl r25
  // sbc r25, r25
  // mov r24, r25

  // lsl r25 <=> add r25, r25
  buildMI(MBB, MBBI, GameBoy::ADDRdRr)
      .addReg(DstHiReg, RegState::Define)
      .addReg(DstHiReg, RegState::Kill)
      .addReg(DstHiReg, RegState::Kill);

  // sbc r25, r25
  auto MISBC =
      buildMI(MBB, MBBI, GameBoy::SBCRdRr)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, RegState::Kill)
          .addReg(DstHiReg, RegState::Kill);
  if (ImpIsDead)
    MISBC->getOperand(3).setIsDead();
  // SREG is always implicitly killed
  MISBC->getOperand(4).setIsKill();

  // mov r24, r25
  buildMI(MBB, MBBI, GameBoy::MOVRdRr)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstHiReg);

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::ASRWNRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  unsigned Imm = MI.getOperand(2).getImm();
  switch (Imm) {
  case 7:
    return expandASRW7Rd(MBB, MBBI);
  case 8:
    return expandASRW8Rd(MBB, MBBI);
  case 14:
    return expandASRW14Rd(MBB, MBBI);
  case 15:
    return expandASRW15Rd(MBB, MBBI);
  default:
    llvm_unreachable("unimplemented asrwn");
    return false;
  }
}

bool GameBoyExpandPseudo::expandLSLB7Rd(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();

  // ror r24
  // clr r24
  // ror r24

  buildMI(MBB, MBBI, GameBoy::RORRd)
      .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstReg, RegState::Kill)
      ->getOperand(3)
      .setIsUndef(true);

  buildMI(MBB, MBBI, GameBoy::EORRdRr)
      .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstReg, RegState::Kill)
      .addReg(DstReg, RegState::Kill);

  auto MIRRC =
      buildMI(MBB, MBBI, GameBoy::RORRd)
          .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstReg, getKillRegState(DstIsKill));

  if (ImpIsDead)
    MIRRC->getOperand(2).setIsDead();

  // SREG is always implicitly killed
  MIRRC->getOperand(3).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::LSLBNRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  unsigned Imm = MI.getOperand(2).getImm();
  switch (Imm) {
  case 7:
    return expandLSLB7Rd(MBB, MBBI);
  default:
    llvm_unreachable("unimplemented lslbn");
    return false;
  }
}

bool GameBoyExpandPseudo::expandLSRB7Rd(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();

  // rol r24
  // clr r24
  // rol r24

  buildMI(MBB, MBBI, GameBoy::ADCRdRr)
      .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstReg, RegState::Kill)
      .addReg(DstReg, RegState::Kill)
      ->getOperand(4)
      .setIsUndef(true);

  buildMI(MBB, MBBI, GameBoy::EORRdRr)
      .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstReg, RegState::Kill)
      .addReg(DstReg, RegState::Kill);

  auto MIRRC =
      buildMI(MBB, MBBI, GameBoy::ADCRdRr)
          .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstReg, getKillRegState(DstIsKill))
          .addReg(DstReg, getKillRegState(DstIsKill));

  if (ImpIsDead)
    MIRRC->getOperand(3).setIsDead();

  // SREG is always implicitly killed
  MIRRC->getOperand(4).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::LSRBNRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  unsigned Imm = MI.getOperand(2).getImm();
  switch (Imm) {
  case 7:
    return expandLSRB7Rd(MBB, MBBI);
  default:
    llvm_unreachable("unimplemented lsrbn");
    return false;
  }
}

bool GameBoyExpandPseudo::expandASRB6Rd(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();

  // bst r24, 6
  // lsl r24
  // sbc r24, r24
  // bld r24, 0

  buildMI(MBB, MBBI, GameBoy::BST)
      .addReg(DstReg)
      .addImm(6)
      ->getOperand(2)
      .setIsUndef(true);

  buildMI(MBB, MBBI, GameBoy::ADDRdRr) // LSL Rd <==> ADD Rd, Rd
      .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstReg, RegState::Kill)
      .addReg(DstReg, RegState::Kill);

  buildMI(MBB, MBBI, GameBoy::SBCRdRr)
      .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstReg, RegState::Kill)
      .addReg(DstReg, RegState::Kill);

  buildMI(MBB, MBBI, GameBoy::BLD)
      .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstReg, getKillRegState(DstIsKill))
      .addImm(0)
      ->getOperand(3)
      .setIsKill();

  MI.eraseFromParent();
  return true;
}

bool GameBoyExpandPseudo::expandASRB7Rd(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();

  // lsl r24
  // sbc r24, r24

  buildMI(MBB, MBBI, GameBoy::ADDRdRr)
      .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstReg, RegState::Kill)
      .addReg(DstReg, RegState::Kill);

  auto MIRRC =
      buildMI(MBB, MBBI, GameBoy::SBCRdRr)
          .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstReg, getKillRegState(DstIsKill))
          .addReg(DstReg, getKillRegState(DstIsKill));

  if (ImpIsDead)
    MIRRC->getOperand(3).setIsDead();

  // SREG is always implicitly killed
  MIRRC->getOperand(4).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::ASRBNRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  unsigned Imm = MI.getOperand(2).getImm();
  switch (Imm) {
  case 6:
    return expandASRB6Rd(MBB, MBBI);
  case 7:
    return expandASRB7Rd(MBB, MBBI);
  default:
    llvm_unreachable("unimplemented asrbn");
    return false;
  }
}

/*
template <> bool GameBoyExpandPseudo::expand<GameBoy::SEXT>(Block &MBB, BlockIt MBBI) {
  dbgs() << "Expanding SEXT instruction\n";
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  // sext R17:R16, R17
  // mov     r16, r17
  // lsl     r17
  // sbc     r17, r17
  // sext R17:R16, R13
  // mov     r16, r13
  // mov     r17, r13
  // lsl     r17
  // sbc     r17, r17
  // sext R17:R16, R16
  // mov     r17, r16
  // lsl     r17
  // sbc     r17, r17
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  if (SrcReg != DstLoReg)
    buildMI(MBB, MBBI, GameBoy::MOVRdRr)
        .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
        .addReg(SrcReg);

  if (SrcReg != DstHiReg) {
    auto MOV = buildMI(MBB, MBBI, GameBoy::MOVRdRr)
                   .addReg(DstHiReg, RegState::Define)
                   .addReg(SrcReg);
    if (SrcReg != DstLoReg && SrcIsKill)
      MOV->getOperand(1).setIsKill();
  }

  buildMI(MBB, MBBI, GameBoy::ADDRdRr) // LSL Rd <==> ADD Rd, Rr
      .addReg(DstHiReg, RegState::Define)
      .addReg(DstHiReg, RegState::Kill)
      .addReg(DstHiReg, RegState::Kill);

  auto SBC =
      buildMI(MBB, MBBI, GameBoy::SBCRdRr)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, RegState::Kill)
          .addReg(DstHiReg, RegState::Kill);

  if (ImpIsDead)
    SBC->getOperand(3).setIsDead();

  // SREG is always implicitly killed
  SBC->getOperand(4).setIsKill();

  MI.eraseFromParent();
  return true;
}
*/

template <> bool GameBoyExpandPseudo::expand<GameBoy::ZEXT>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  // zext R25:R24, R20
  // mov      R24, R20
  // eor      R25, R25
  // zext R25:R24, R24
  // eor      R25, R25
  // zext R25:R24, R25
  // mov      R24, R25
  // eor      R25, R25
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  if (SrcReg != DstLoReg) {
    buildMI(MBB, MBBI, GameBoy::MOVRdRr)
        .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
        .addReg(SrcReg, getKillRegState(SrcIsKill));
  }

  auto EOR =
      buildMI(MBB, MBBI, GameBoy::EORRdRr)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(DstHiReg, RegState::Kill | RegState::Undef)
          .addReg(DstHiReg, RegState::Kill | RegState::Undef);

  if (ImpIsDead)
    EOR->getOperand(3).setIsDead();

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::SPREAD>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  unsigned Flags = MI.getFlags();
  unsigned OpLo = GameBoy::INRdA;
  unsigned OpHi = GameBoy::INRdA;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Low part
  buildMI(MBB, MBBI, OpLo)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addImm(0x3d)
      .setMIFlags(Flags);

  // High part
  buildMI(MBB, MBBI, OpHi)
      .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
      .addImm(0x3e)
      .setMIFlags(Flags);

  MI.eraseFromParent();
  return true;
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::SPWRITE>(Block &MBB, BlockIt MBBI) {
  const GameBoySubtarget &STI = MBB.getParent()->getSubtarget<GameBoySubtarget>();
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg;
  Register SrcReg = MI.getOperand(1).getReg();
  bool SrcIsKill = MI.getOperand(1).isKill();
  unsigned Flags = MI.getFlags();
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);

  buildMI(MBB, MBBI, GameBoy::INRdA)
      .addReg(GameBoy::R0, RegState::Define)
      .addImm(STI.getIORegSREG())
      .setMIFlags(Flags);

  buildMI(MBB, MBBI, GameBoy::BCLRs).addImm(0x07).setMIFlags(Flags);

  buildMI(MBB, MBBI, GameBoy::OUTARr)
      .addImm(0x3e)
      .addReg(SrcHiReg, getKillRegState(SrcIsKill))
      .setMIFlags(Flags);

  buildMI(MBB, MBBI, GameBoy::OUTARr)
      .addImm(STI.getIORegSREG())
      .addReg(GameBoy::R0, RegState::Kill)
      .setMIFlags(Flags);

  buildMI(MBB, MBBI, GameBoy::OUTARr)
      .addImm(0x3d)
      .addReg(SrcLoReg, getKillRegState(SrcIsKill))
      .setMIFlags(Flags);

  MI.eraseFromParent();
  return true;
}

bool GameBoyExpandPseudo::expandMI(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  int Opcode = MBBI->getOpcode();

#define EXPAND(Op)                                                             \
  case Op:                                                                     \
    return expand<Op>(MBB, MI)

  switch (Opcode) {
    // Game Boy
    EXPAND(GameBoy::LDRd8Ptr);
    EXPAND(GameBoy::LDRdPairRrPair);
    EXPAND(GameBoy::AddRdRr);
    EXPAND(GameBoy::AddRdImm8);
    EXPAND(GameBoy::AddRdPairRrPair);
    EXPAND(GameBoy::AddRpdImm16);
    EXPAND(GameBoy::AndRdRr);
    EXPAND(GameBoy::AndRdImm8);
    EXPAND(GameBoy::OrRdRr);
    EXPAND(GameBoy::OrRdImm8);
    EXPAND(GameBoy::XorRdRr);
    EXPAND(GameBoy::XorRdImm8);
    EXPAND(GameBoy::CpRdRr);
    EXPAND(GameBoy::CpRdImm8);
    // AVR
    EXPAND(GameBoy::ADDWRdRr);
    EXPAND(GameBoy::ADCWRdRr);
    EXPAND(GameBoy::SUBWRdRr);
    EXPAND(GameBoy::SUBIWRdK);
    EXPAND(GameBoy::SBCWRdRr);
    EXPAND(GameBoy::SBCIWRdK);
    EXPAND(GameBoy::ANDWRdRr);
    EXPAND(GameBoy::ANDIWRdK);
    EXPAND(GameBoy::ORWRdRr);
    EXPAND(GameBoy::ORIWRdK);
    EXPAND(GameBoy::EORWRdRr);
    EXPAND(GameBoy::COMWRd);
    EXPAND(GameBoy::NEGWRd);
    EXPAND(GameBoy::CPWRdRr);
    EXPAND(GameBoy::CPCWRdRr);
    // EXPAND(GameBoy::LDIWRdK);
    EXPAND(GameBoy::LDSWRdK);
    EXPAND(GameBoy::LDWRdPtr);
    EXPAND(GameBoy::LDWRdPtrPi);
    EXPAND(GameBoy::LDWRdPtrPd);
  case GameBoy::LDDWRdYQ: //: FIXME: remove this once PR13375 gets fixed
    EXPAND(GameBoy::LDDWRdPtrQ);
    EXPAND(GameBoy::LPMWRdZ);
    EXPAND(GameBoy::LPMWRdZPi);
    EXPAND(GameBoy::ELPMBRdZ);
    EXPAND(GameBoy::ELPMWRdZ);
    EXPAND(GameBoy::ELPMBRdZPi);
    EXPAND(GameBoy::ELPMWRdZPi);
    EXPAND(GameBoy::AtomicLoad8);
    EXPAND(GameBoy::AtomicLoad16);
    EXPAND(GameBoy::AtomicStore8);
    EXPAND(GameBoy::AtomicStore16);
    EXPAND(GameBoy::AtomicFence);
    EXPAND(GameBoy::STSWKRr);
    EXPAND(GameBoy::STWPtrRr);
    EXPAND(GameBoy::STWPtrPiRr);
    EXPAND(GameBoy::STWPtrPdRr);
    EXPAND(GameBoy::STDWPtrQRr);
    EXPAND(GameBoy::STDSPQRr);
    EXPAND(GameBoy::STDWSPQRr);
    EXPAND(GameBoy::INWRdA);
    EXPAND(GameBoy::OUTWARr);
    EXPAND(GameBoy::PUSHWRr);
    EXPAND(GameBoy::POPWRd);
    EXPAND(GameBoy::ROLBRd);
    EXPAND(GameBoy::RORBRd);
    EXPAND(GameBoy::LSLWRd);
    EXPAND(GameBoy::LSRWRd);
    EXPAND(GameBoy::RORWRd);
    EXPAND(GameBoy::ROLWRd);
    EXPAND(GameBoy::ASRWRd);
    EXPAND(GameBoy::LSLWHiRd);
    EXPAND(GameBoy::LSRWLoRd);
    EXPAND(GameBoy::ASRWLoRd);
    EXPAND(GameBoy::LSLWNRd);
    EXPAND(GameBoy::LSRWNRd);
    EXPAND(GameBoy::ASRWNRd);
    EXPAND(GameBoy::LSLBNRd);
    EXPAND(GameBoy::LSRBNRd);
    EXPAND(GameBoy::ASRBNRd);
    EXPAND(GameBoy::SEXT);
    EXPAND(GameBoy::ZEXT);
    EXPAND(GameBoy::SPREAD);
    EXPAND(GameBoy::SPWRITE);
  }
#undef EXPAND
  return false;
}

} // end of anonymous namespace

INITIALIZE_PASS(GameBoyExpandPseudo, "GameBoy-expand-pseudo", GameBoy_EXPAND_PSEUDO_NAME,
                false, false)
namespace llvm {

FunctionPass *createGameBoyExpandPseudoPass() { return new GameBoyExpandPseudo(); }

} // end of namespace llvm
