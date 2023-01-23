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
  // if (Op == GameBoy::ANDIRdK && ImmVal == 0xff)
  //   return true;

  // // ORI Rd, 0x0 is redundant.
  // if (Op == GameBoy::ORIRdK && ImmVal == 0x0)
  //   return true;

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
    MINew = buildMI(MBB, MBBI, GameBoy::LDRd8Ptr)
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
    // dbgs() << "Expanding AddRdRr into Add A Rr\n";
    // printAllOperands(MI);
    // ADD A, Rr
    buildMI(MBB, MBBI, GameBoy::AddARr)
      .addReg(GameBoy::RA, RegState::Define)
      .addReg(SrcReg, getKillRegState(SrcIsKill));
  } else {
    // dbgs() << "Expanding AddRd(" << DstReg.id() << ") Rr(" << SrcReg.id() << ")\n";
    // printAllOperands(MI);
    // LD A, Rr
    buildMI(MBB, MBBI, GameBoy::LDRdRr)
      .addReg(GameBoy::RA, RegState::Define)
      .addReg(SrcReg, getKillRegState(SrcIsKill));
    // dbgs() << "A\n";
    // ADD A, Rd
    buildMI(MBB, MBBI, GameBoy::AddARr)
      .addReg(GameBoy::RA, RegState::Define)
      .addReg(DstReg, getDeadRegState(DstIsDead));
    // dbgs() << "B\n";
    // LD Rd, A
    buildMI(MBB, MBBI, GameBoy::LDRdRr)
      .addReg(DstReg, RegState::Define)
      .addReg(GameBoy::RA);
    // dbgs() << "C\n";
  }
  MI.eraseFromParent();
  return true;
}

// Will expand ADD R Imm8 using A as a temporary.
template<>
bool GameBoyExpandPseudo::expand<GameBoy::AddRdImm8>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
  // dbgs() << "Expanding AddRdImm8\n";
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
  llvm_unreachable("Incomplete AddRdPairRrPair");
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
  // dbgs() << "Expanding AddRpdImm16\n";
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
  // CP Rd, Rr
  // The GB can only make comparisons to the A register.
  // This means that if Rd is not in A, then it must be moved there and
  // restored afterwards back to Rd if a move was necessary.
  // Most 8-bit comparisons will have either Rd or Rr in A.

  // Collect all necessary information
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  auto SrcIsKill = MI.getOperand(1).isKill();
  // Check whether the DstReg is A
  auto needsMove = !GameBoy::GPRLoadRegClass.contains(DstReg);
  
  if (needsMove)
    llvm_unreachable("Incomplete CpRdRr");

  // Perform a comparison between A and whatever register is needed here.
  buildMI(MBB, MBBI, GameBoy::CPARr)
    .addReg(DstReg)
    .addReg(SrcReg, getKillRegState(SrcIsKill));

  // Remove the old instruction
  MI.removeFromParent();
  return true;
}

template<>
bool GameBoyExpandPseudo::expand<GameBoy::CpWRdRr>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("Incomplete CpWRdRr");
  return true;
}

template<>
bool GameBoyExpandPseudo::expand<GameBoy::CpRdImm8>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("Incomplete CpRdImm8");
  return true;
}

template<>
bool GameBoyExpandPseudo::expand<GameBoy::CpRdImm16>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("Incomplete CpRdImm16");
  return true;
}

//===----------------------------------------------------------------------===//
// General purpose arithmetic operands and CPU control instructions
//===----------------------------------------------------------------------===//
template<>
bool GameBoyExpandPseudo::expand<GameBoy::ZEXT>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcReg = MI.getOperand(1).getReg();
  // This will need to be split into HIGH and LOW.
  Register DstReg = MI.getOperand(0).getReg();
  bool SrcIsKill = MI.getOperand(1).isKill();
  bool DstIsDead = MI.getOperand(0).isDead();
  Register DstHi, DstLow;
  // The Hi and Low are swapped here.
  TRI->splitReg(DstReg, DstHi, DstLow);

  // Store the low byte
  buildMI(MBB, MBBI, GameBoy::LDRdRr)
    .addReg(DstLow)
    .addReg(SrcReg, getKillRegState(SrcIsKill));

  // Store 0 in upper byte
  buildMI(MBB, MBBI, GameBoy::LDRdImm8)
    .addReg(DstHi)
    .addImm(0);

  // Remove old instruction
  MI.removeFromParent();
  return true;
}

template<>
bool GameBoyExpandPseudo::expand<GameBoy::SEXT>(Block &MBB, BlockIt MBBI) {
  bool addDefines = false;
  MachineInstr &MI = *MBBI;
  Register SrcReg = MI.getOperand(1).getReg();
  // This will need to be split into HIGH and LOW.
  Register DstReg = MI.getOperand(0).getReg();
  bool SrcIsKill = MI.getOperand(1).isKill();
  bool DstIsDead = MI.getOperand(0).isDead();
  Register DstHi, DstLow;
  // The Hi and Low are swapped here.
  TRI->splitReg(DstReg, DstHi, DstLow);
  // Low byte is copied as is, then the sign bit is pushed into the carry bit
  // The sbc instruction is exploited to get a 0 or -1 via (A - A - Carry),
  // which is therefore the same as (-Carry). We then store this in the upper
  // byte

  // Store low byte
  if (DstLow.id() != SrcReg.id()) {
    buildMI(MBB, MBBI, GameBoy::LDRdRr)
      .addReg(DstLow)
      .addReg(SrcReg, getKillRegState(SrcIsKill));
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
    buildMI(MBB, MBBI, GameBoy::LDRdRr)
      .addReg(GameBoy::RA, RegState::Define)
      .addReg(SrcReg, getKillRegState(SrcIsKill));
    buildMI(MBB, MBBI, GameBoy::AddARr)
      .addReg(GameBoy::RA, RegState::Define)
      .addReg(GameBoy::RA);
  } else {
    buildMI(MBB, MBBI, GameBoy::AddARr)
      .addReg(SrcReg, RegState::Define)
      .addReg(SrcReg);
  }

  // Convert to 0 or -1
  buildMI(MBB, MBBI, GameBoy::SbcARr)
    .addReg(GameBoy::RA, RegState::Define)
    .addReg(GameBoy::RA);

  // Store high byte
  buildMI(MBB, MBBI, GameBoy::LDRdRr)
    .addReg(DstHi, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(GameBoy::RA);

  // Remove old instruction
  MI.removeFromParent();
  return true;
}

//===----------------------------------------------------------------------===//
// Jump instructions
//===----------------------------------------------------------------------===//
template <>
bool GameBoyExpandPseudo::expand<GameBoy::JREQk>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("Incomplete JREQk");
  return true; 
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::JRNEk>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("Incomplete JRNEk");
  return true; 
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::JRGTEk>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("Incomplete JRGTEk");
  return true; 
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::JRLTk>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("Incomplete JRLTk");
  return true; 
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::JRSHk>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("Incomplete JRSHk");
  return true; 
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::JRLOk>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("Incomplete JRLOk");
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
    EXPAND(GameBoy::SEXT);
    EXPAND(GameBoy::ZEXT);
    EXPAND(GameBoy::JREQk);
    EXPAND(GameBoy::JRNEk);
    EXPAND(GameBoy::JRGTEk);
    EXPAND(GameBoy::JRLTk);
    EXPAND(GameBoy::JRSHk);
    EXPAND(GameBoy::JRLOk);
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
