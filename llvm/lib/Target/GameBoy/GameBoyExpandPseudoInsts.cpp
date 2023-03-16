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

void printAllOperands(MachineInstr &MI) {
  for (unsigned int i = 0; i < MI.getNumOperands(); i++) {
    dbgs() << "\t" << MI.getOperand(i) << "\n";
  }
}

//===----------------------------------------------------------------------===//
// Loading
//===----------------------------------------------------------------------===//
// LD Rd, (Rr)
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
  // Load the value found at address (Rr) into A
  MINew = buildMI(MBB, MBBI, GameBoy::LDAPtr, GameBoy::RA).addReg(SrcReg);

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
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);
  // LD DstHi, SrcHi
  auto ld = buildMI(MBB, MBBI, GameBoy::LDRdRr, DstHiReg).addReg(SrcHiReg, RegState::Define);
  // LD DstLo, SrcLo
  buildMI(MBB, MBBI, GameBoy::LDRdRr, DstLoReg).addReg(SrcLoReg, RegState::Define);
  // remove previous instruction.
  ld.setMemRefs(MI.memoperands());
  MI.removeFromParent();
  return true;
}

// LD Rd, (Imm8)
template<>
bool GameBoyExpandPseudo::expand<GameBoy::LDRdPtrImm8>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
  auto SrcImm = MI.getOperand(1).getGlobal();

  // Load the pointer into the A register first
  MachineInstrBuilder MINew;
  MINew = buildMI(MBB, MBBI, GameBoy::LDAImm16Addr, GameBoy::RA).addGlobalAddress(SrcImm);

  // See if we need to LD Rd, A
  if (DstReg != GameBoy::RA)
    buildMI(MBB, MBBI, GameBoy::LDRdRr).addReg(DstReg).addReg(GameBoy::RA);

  MINew.setMemRefs(MI.memoperands());
  MI.eraseFromParent();
  return true;
}

template<>
bool GameBoyExpandPseudo::expand<GameBoy::LDRdPtr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
  MachineInstrBuilder MINew;
  auto isglobal = MI.getOperand(1).isGlobal();
  auto isImm = MI.getOperand(1).isImm();
  auto isReg = MI.getOperand(1).isReg();

  if (isglobal) {
    auto SrcImm = MI.getOperand(1).getGlobal();
    // Load the pointer into the A register first
    MINew = buildMI(MBB, MBBI, GameBoy::LDAImm16Addr, GameBoy::RA).addGlobalAddress(SrcImm);
  } else if (isImm) {
    auto SrcImm = MI.getOperand(1).getImm();
    MINew = buildMI(MBB, MBBI, GameBoy::LDAImm16Addr, GameBoy::RA).addImm(SrcImm);
  } else if (isReg) {
    auto SrcReg = MI.getOperand(1).getReg();

    // Check if the pointer is held in HL
    if (SrcReg != GameBoy::RHRL) {
      // LD HL, PtrRegPair
      buildMI(MBB, MBBI, GameBoy::LDRdPairRrPair, GameBoy::RHRL).addReg(SrcReg);
    }
    // LD A, HL
    MINew = buildMI(MBB, MBBI, GameBoy::LDRdHLPtr, GameBoy::RA).addReg(GameBoy::RHRL);
  } else {
    llvm_unreachable("LDRdPtr: Unknown operand type!");
  }

  // See if we need to LD Rd, A
  if (DstReg != GameBoy::RA)
    buildMI(MBB, MBBI, GameBoy::LDRdRr).addReg(DstReg).addReg(GameBoy::RA);

  MINew.setMemRefs(MI.memoperands());
  MI.eraseFromParent();
  return true;
}

template<>
bool GameBoyExpandPseudo::expand<GameBoy::LDPtrRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  MachineInstrBuilder MINew;
  Register SrcReg = MI.getOperand(1).getReg();

  auto DstOp = MI.getOperand(0);
  bool isReg = MI.getOperand(0).isReg();

  // Temporary workaround- use HL to hold the address.
  if (DstOp.isGlobal()) {
    MINew = buildMI(MBB, MBBI, GameBoy::LDRdImm16, GameBoy::RHRL).addGlobalAddress(DstOp.getGlobal());
  }
  else if (DstOp.isImm()) {
    MINew = buildMI(MBB, MBBI, GameBoy::LDRdImm16, GameBoy::RHRL).addImm(DstOp.getImm());
  }
  else if (isReg) {
    dbgs() << "WARNING: LDPtrRd must check that Ptr is 16-bit\n";
    auto DstReg = MI.getOperand(0).getReg();
    // If the pointer is a register already, then see if the pointer is HL.
    if (DstReg != GameBoy::RHRL)
      buildMI(MBB, MBBI, GameBoy::LDRdPairRrPair, GameBoy::RHRL).addReg(DstReg);

  } else {
    llvm_unreachable("LDPtrRd: Invalid operand for destination!\n");
  }

  // We can use the LD (HL), Rr instruction to store the value instead.
  buildMI(MBB, MBBI, GameBoy::LDHLAddrRr).addReg(GameBoy::RHRL).addReg(SrcReg);
  // MINew = buildMI(MBB, MBBI, GameBoy::LDImm16AddrA).addGlobalAddress(DstImm);
  // MINew.setMemRefs(MI.memoperands());
  MI.eraseFromParent();
  return true;
}

template<>
bool GameBoyExpandPseudo::expand<GameBoy::LDRdPairPtr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  // printAllOperands(MI);

  Register PtrSrc;
  Register DstReg;
  Register DstRegHi;
  Register DstRegLow;

  // Get the source pointer.
  if (MI.getOperand(0).isReg())
    PtrSrc = MI.getOperand(0).getReg();
  else
    llvm_unreachable("Invalid source operand for pointer!");
 
  // Get the destination register pair.
  if (MI.getOperand(1).isReg())
    DstReg = MI.getOperand(1).getReg();
  else
    llvm_unreachable("Invalid destination operand!");

  // Split the source register.
  TRI->splitReg(DstReg, DstRegHi, DstRegLow);
  // dbgs() << "High register: " << DstRegHi.id() << " Low register: " << DstRegLow.id(); 

  // Load the lower byte of the pointer
  buildMI(MBB, MBBI, GameBoy::LDRdPtr)
    .addReg(DstRegLow, RegState::Define)
    .addReg(PtrSrc);

  // Increment the pointer to access high byte
  buildMI(MBB, MBBI, GameBoy::INCRd)
    .addReg(PtrSrc);

  // Load the upper byte
  buildMI(MBB, MBBI, GameBoy::LDRdPtr)
    .addReg(DstRegHi, RegState::Define)
    .addReg(PtrSrc);

  // Remove old instruction
  MI.removeFromParent();
  // llvm_unreachable("Unimplemented LDRdPairPtr!");
  return true;
}

template<>
bool GameBoyExpandPseudo::expand<GameBoy::LDPtrRdPair>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  // printAllOperands(MI);
  Register PtrDst;
  Register SrcReg;
  Register SrcRegHi;
  Register SrcRegLow;

  // Get the destination pointer.
  if (MI.getOperand(0).isReg())
    PtrDst = MI.getOperand(0).getReg();
  else
    llvm_unreachable("Invalid destination operand for pointer!");

  // Get the source register pair
  if (MI.getOperand(1).isReg())
    SrcReg = MI.getOperand(1).getReg();
  else
    llvm_unreachable("Invalid source operand!");

  // Split the source register.
  TRI->splitReg(SrcReg, SrcRegHi, SrcRegLow);
  // dbgs() << "High register: " << SrcRegHi.id() << " Low register: " << SrcRegLow.id(); 

  // First we must store the lower byte
  // LD Ptr, RdPairLow
  buildMI(MBB, MBBI, GameBoy::LDPtrRd)
    .addReg(PtrDst, RegState::Define)
    .addReg(SrcRegLow, RegState::Define);

  // Then we must store the upper byte in the next address.
  // INC Ptr
  buildMI(MBB, MBBI, GameBoy::INCRd)
    .addReg(PtrDst);
  // LD Ptr, RdPairHigh
  buildMI(MBB, MBBI, GameBoy::LDPtrRd)
    .addReg(PtrDst)
    .addReg(SrcRegHi, RegState::Define);

  // Remove from the bundle.
  MI.removeFromParent();
  return true;
  // llvm_unreachable("Unimplemented LDPtrRdPair!");
}

template<>
bool GameBoyExpandPseudo::expand<GameBoy::LDRdPtrQ>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  printAllOperands(MI);

  // Ptr = Ptr + q
  // LD Rd, Ptr
  // Ptr = Ptr - q

  llvm_unreachable("Unimplemented LDRdPtrQ!");
}

template<>
bool GameBoyExpandPseudo::expand<GameBoy::LDRdPairPtrQ>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  printAllOperands(MI);

  // Ptr = Ptr + q
  // LD RdLow, Ptr
  // INC Ptr
  // LD RdHigh, Ptr
  // DEC Ptr
  // Ptr = Ptr - q
  llvm_unreachable("Unimplemented LDRdPaurPtrQ!");
}

template<>
bool GameBoyExpandPseudo::expand<GameBoy::LDPtrQRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  printAllOperands(MI);

  // Expand into
  // Ptr = Ptr + q
  // LD Ptr + q, Rr
  // Ptr = Ptr - q

  llvm_unreachable("Unimplemented LDPtrQRd!");
}

template<>
bool GameBoyExpandPseudo::expand<GameBoy::LDPtrQRdPair>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  printAllOperands(MI);

  // Expand into
  // Ptr = Ptr + q
  // LD Ptr, RrLow
  // INC Ptr
  // LD Ptr, RrHigh
  // DEC Ptr
  // Ptr = Ptr - q

  llvm_unreachable("Unimplemented LDPtrQRdPair!");
}

//===----------------------------------------------------------------------===//
// Arithmetic
//===----------------------------------------------------------------------===//

template<>
bool GameBoyExpandPseudo::expand<GameBoy::AddRdRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(1).getReg();
  Register SrcReg = MI.getOperand(2).getReg();
  bool SrcIsKill = MI.getOperand(2).isKill();
  bool DstIsDead = MI.getOperand(1).isDead();

  if (SrcReg.id() == DstReg.id()) {  
    // ADD A, Rr
    buildMI(MBB, MBBI, GameBoy::AddARr)
      .addReg(GameBoy::RA, RegState::Define)
      .addReg(SrcReg, getKillRegState(SrcIsKill));
  } else {
    // LD A, Rr
    buildMI(MBB, MBBI, GameBoy::LDRdRr)
      .addReg(GameBoy::RA, RegState::Define)
      .addReg(SrcReg, getKillRegState(SrcIsKill));
    // ADD A, Rd
    buildMI(MBB, MBBI, GameBoy::AddARr)
      .addReg(GameBoy::RA, RegState::Define)
      .addReg(DstReg, getDeadRegState(DstIsDead));
    // LD Rd, A
    buildMI(MBB, MBBI, GameBoy::LDRdRr)
      .addReg(DstReg, RegState::Define)
      .addReg(GameBoy::RA);
  }
  MI.eraseFromParent();
  return true;
}

// Will expand ADD R Imm8 using A as a temporary.
template<>
bool GameBoyExpandPseudo::expand<GameBoy::AddRdImm8>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
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
  // printAllOperands(MI);
  unsigned lastOp = MI.getNumOperands() - 1;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  auto SrcIsKill = MI.getOperand(1).isKill();
  auto DstIsDead = MI.getOperand(0).isDead();
  printAllOperands(MI);
  // LD HL, Rpd
  buildMI(MBB, MBBI, GameBoy::LDRdPairRrPair, GameBoy::RHRL)
    .addReg(DstReg, getDeadRegState(DstIsDead));
  // LD Src, Imm16 happens implicitly.
  // ADD HL, Src
  buildMI(MBB, MBBI, GameBoy::ADDHLPair, GameBoy::RHRL)
    .addReg(SrcReg, getKillRegState(SrcIsKill));
  // LD Rpd, HL
  buildMI(MBB, MBBI, GameBoy::LDRdPairRrPair)
    .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(GameBoy::RHRL, RegState::Define);
  
  MI.eraseFromParent();
  return true;
}

template<>
bool GameBoyExpandPseudo::expand<GameBoy::SubRdPairRrPair>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  printAllOperands(MI);
  llvm_unreachable("Unimplemented expand SubRdPairRrPair!");
}

template<>
bool GameBoyExpandPseudo::expand<GameBoy::SbcRdPairRrPair>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  printAllOperands(MI);
  llvm_unreachable("Unimplemented expand SbcRdPairRrPair!");
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
bool GameBoyExpandPseudo::expand<GameBoy::AndRdRrPair>(Block &MBB, BlockIt MBBI) {
  // See if Rd is A; if so, can avoid LD A, Rd
  // MachineInstr &MI = *MBBI;
  // Register DstReg = MI.getOperand(0).getReg();
  // Register SrcReg = MI.getOperand(1).getReg();
  llvm_unreachable("Incomplete AndRdRrPair");
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
bool GameBoyExpandPseudo::expand<GameBoy::OrRdRrPair>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("Incomplete OrRdRrPair");
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
    buildMI(MBB, MBBI, GameBoy::LDRdRr).addReg(GameBoy::RA, RegState::Define).addReg(DstReg);

  // Perform a comparison between A and whatever register is needed here.
  buildMI(MBB, MBBI, GameBoy::CPARr)
    .addReg(GameBoy::RA)
    .addReg(SrcReg, getKillRegState(SrcIsKill));

  // Remove the old instruction
  MI.removeFromParent();
  return true;
}

template<>
bool GameBoyExpandPseudo::expand<GameBoy::CpWRdRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  // Split registers
  Register RdLoReg, RdHiReg, RrLoReg, RrHiReg;
  Register RdReg = MI.getOperand(0).getReg();
  Register RrReg = MI.getOperand(1).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(1).isKill();
  TRI->splitReg(RdReg, RdLoReg, RdHiReg);
  TRI->splitReg(RrReg, RrLoReg, RrHiReg);
  // OR A, A ; Clear the carry flag.
  buildMI(MBB, MBBI, GameBoy::OrARr, GameBoy::RA);
  // SUB LhsLow, RhsLow ; subtract on the lower byte
  /// LD A, LhsLow <! Clobbering, so we perform the load ourselves
  buildMI(MBB, MBBI, GameBoy::LDRdRr, GameBoy::RA).addReg(RdLoReg, RegState::Define);
  /// SUB A, RhsLow
  buildMI(MBB, MBBI, GameBoy::SubARr, RdLoReg).addReg(RrLoReg, RegState::Define);
  // SBC LhsHi, RhsHi ; Subtract with carry on the upper byte
  /// LD A, LhsHi
  buildMI(MBB, MBBI, GameBoy::LDRdRr, GameBoy::RA).addReg(RdHiReg, RegState::Define);
  /// SBC A, RhsHi
  buildMI(MBB, MBBI, GameBoy::SbcARr, GameBoy::RA).addReg(RrHiReg, RegState::Define);

  // ADD LHS, RHS ; Restore the value

  // Optimisation currently causes issues.
    // LD HL, LHS
    if (RdReg != GameBoy::RHRL)
      buildMI(MBB, MBBI, GameBoy::LDRdPairRrPair, GameBoy::RHRL).addReg(RdReg, RegState::Define | getDeadRegState(DstIsDead));
    // ADD HL, RHS
    buildMI(MBB, MBBI, GameBoy::ADDHLPair, GameBoy::RHRL).addReg(RrReg, RegState::Define);
    // LD LHS, HL
    if (RdReg != GameBoy::RHRL)
      buildMI(MBB, MBBI, GameBoy::LDRdPairRrPair, RdReg).addReg(GameBoy::RHRL);
    
  // buildMI(MBB, MBBI, GameBoy::AddRdPairRrPair)
  //   .addReg(RdReg, RegState::Define | getDeadRegState(DstIsDead))
  //   .addReg(RrReg, getKillRegState(SrcIsKill));
  // Remove the old instruction
  MI.removeFromParent();
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
  auto name = TRI->getRegAsmName(SrcReg).str();
  bool lda = name.compare("RA");
  if (lda) {
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
// Shift-rotate instructions
//===----------------------------------------------------------------------===//
template<>
bool GameBoyExpandPseudo::expand<GameBoy::RLNRd>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("Incomplete constant amount left rotate!");
  return true;
}

template<>
bool GameBoyExpandPseudo::expand<GameBoy::RRNRd>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("Incomplete constant amount right rotate!");
  return true;
}

template<>
bool GameBoyExpandPseudo::expand<GameBoy::RLNRdPair>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("Incomplete constant amount left rotate (16)!");
  return true;
}

template<>
bool GameBoyExpandPseudo::expand<GameBoy::RRNRdPair>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("Incomplete constant amount right rotate (16)!");
  return true;
}

template<>
bool GameBoyExpandPseudo::expand<GameBoy::SLARdPair>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  
  Register RdPair = MI.getOperand(0).getReg();
  Register RdLow, RdHigh;
  // Split register (use reverse order)
  TRI->splitReg(RdPair, RdHigh, RdLow);
  
  // SLA RdLow
  buildMI(MBB, MBBI, GameBoy::SLARd, RdLow);
  // RL RdHigh
  buildMI(MBB, MBBI, GameBoy::RLRd, RdHigh);

  // Remove old MI
  MI.removeFromParent();
  return true;
}

template<>
bool GameBoyExpandPseudo::expand<GameBoy::SRARdPair>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("Incomplete SRARdPair!");
  return true;
}

template<>
bool GameBoyExpandPseudo::expand<GameBoy::SRLRdPair>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("Incomplete SRLRdPair!");
  return true;
}

//===----------------------------------------------------------------------===//
// Pointer arithmetic
//===----------------------------------------------------------------------===//
template<>
bool GameBoyExpandPseudo::expand<GameBoy::AddHLAddr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  printAllOperands(MI);
  Register HL = MI.getOperand(0).getReg();
  auto Addr = MI.getOperand(2).getGlobal(); 

  // Push BC since we have to sacrifice it.
  buildMI(MBB, MBBI, GameBoy::PUSHRd, GameBoy::RBRC);
  // Load the pointer address into a register pair.
  buildMI(MBB, MBBI, GameBoy::LDRdImm16, GameBoy::RBRC).addGlobalAddress(Addr);
  // Add the two values.
  buildMI(MBB, MBBI, GameBoy::ADDHLPair, GameBoy::RHRL).addReg(GameBoy::RBRC);
  // Pop BC
  buildMI(MBB, MBBI, GameBoy::POPRd).addReg(GameBoy::RBRC);

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
  // Signed comparison for A <= X
  // This is first transformed to A < X + 1
  // Then, the backend produces a FALSE branch and
  // jumps to this if A >= X + 1

  MachineInstr &MI = *MBBI;

  // The false branch
  auto branch = MI.getOperand(0).getMBB();

  // Bit setting is required to convert a signed comparison
  // to an unsigned comparison. This is not yet implemented.
  dbgs() << "WARNING: Unimplemented bit-set for JRGTEk\n";

  // We will jump to the false branch if A >= X + 1
  // First jump if A = X + 1
  buildMI(MBB, MBBI, GameBoy::JRZk).addMBB(branch);
  // Jump is A > X + 1
  buildMI(MBB, MBBI, GameBoy::JRNCk).addMBB(branch);

  // Remove the old instruction
  MI.removeFromParent();
  // llvm_unreachable("Incomplete JRGTEk");
  return true; 
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::JRLTk>(Block &MBB, BlockIt MBBI) {
  // Signed comparison for A > X
  // The backend will change this to A >= X + 1
  MachineInstr &MI = *MBBI;

  // The false branch
  auto branch = MI.getOperand(0).getMBB();

  // First we must set the bits accordingly.
  // Bit testing is not implemented yet, so we must emit a warning.
  dbgs() << "WARNING: Unimplemented bit-set for JRLTk\n";

  // We will jump to the false branch if A < X + 1
  buildMI(MBB, MBBI, GameBoy::JRCk).addMBB(branch);

  // Remove the old instruction.
  MI.removeFromParent();
  // llvm_unreachable("Incomplete JRLTk");
  return true; 
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::JRSHk>(Block &MBB, BlockIt MBBI) {
  // Same as A <= X
  // This is first transformed to TRUE = A < X + 1
  // Then it is transformed to JUMP FALSE = A >= X + 1
  // Comparison has already been made, so we only need to include the jumps here.
  MachineInstr &MI = *MBBI;

  // The false branch.
  auto branch = MI.getOperand(0).getMBB();

  // Check if the operand is equal.
  // JR A = X, False
  buildMI(MBB, MBBI, GameBoy::JRZk).addMBB(branch);
  // Check if the operand is greater than X
  // JR A > X, False
  buildMI(MBB, MBBI, GameBoy::JRNCk).addMBB(branch);
  // Remove the old instruction.
  MI.removeFromParent();
  return true; 
}

template <>
bool GameBoyExpandPseudo::expand<GameBoy::JRLOk>(Block &MBB, BlockIt MBBI) {
  // llvm_unreachable("Incomplete JRLOk");
  // Same as A > X
  // This gets flipped to JUMP FALSE = A <= X?
  MachineInstr &MI = *MBBI;
  // The false branch
  auto branch = MI.getOperand(0).getMBB();
  // Use basic less-than comparison
  buildMI(MBB, MBBI, GameBoy::JRCk).addMBB(branch);
  // Remove old instruction
  MI.removeFromParent();
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
    EXPAND(GameBoy::LDRdPtrImm8);
    EXPAND(GameBoy::LDRdPtr);
    EXPAND(GameBoy::LDPtrRd);
    EXPAND(GameBoy::LDRdPairPtr);
    EXPAND(GameBoy::LDPtrRdPair);
    EXPAND(GameBoy::LDRdPtrQ);
    EXPAND(GameBoy::LDRdPairPtrQ);
    EXPAND(GameBoy::LDPtrQRd);
    EXPAND(GameBoy::LDPtrQRdPair);
    EXPAND(GameBoy::AddRdRr);
    EXPAND(GameBoy::AddRdImm8);
    EXPAND(GameBoy::AddRdPairRrPair);
    EXPAND(GameBoy::SubRdPairRrPair);
    EXPAND(GameBoy::SbcRdPairRrPair);
    EXPAND(GameBoy::AndRdRr);
    EXPAND(GameBoy::AndRdImm8);
    EXPAND(GameBoy::OrRdRr);
    EXPAND(GameBoy::OrRdImm8);
    EXPAND(GameBoy::XorRdRr);
    EXPAND(GameBoy::XorRdImm8);
    EXPAND(GameBoy::CpRdRr);
    EXPAND(GameBoy::CpRdImm8);
    EXPAND(GameBoy::CpWRdRr);
    EXPAND(GameBoy::SEXT);
    EXPAND(GameBoy::ZEXT);
    EXPAND(GameBoy::JREQk);
    EXPAND(GameBoy::JRNEk);
    EXPAND(GameBoy::JRGTEk);
    EXPAND(GameBoy::JRLTk);
    EXPAND(GameBoy::JRSHk);
    EXPAND(GameBoy::JRLOk);
    // Shift-rotate
    EXPAND(GameBoy::RLNRd);
    EXPAND(GameBoy::RRNRd);
    EXPAND(GameBoy::RLNRdPair);
    EXPAND(GameBoy::RRNRdPair);
    EXPAND(GameBoy::SRLRdPair);
    EXPAND(GameBoy::SRARdPair);
    EXPAND(GameBoy::SLARdPair);
    // Pointers
    EXPAND(GameBoy::AddHLAddr);
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
