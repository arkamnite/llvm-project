//===-- GameBoyInstrInfo.td - GameBoy Instruction defs -------------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file describes the GameBoy instructions in TableGen format.
//
//===----------------------------------------------------------------------===//

include "GameBoyInstrFormats.td"

//===----------------------------------------------------------------------===//
// AVR Type Profiles
//===----------------------------------------------------------------------===//

def SDT_GameBoyCallSeqStart : SDCallSeqStart<[SDTCisVT<0, i16>, SDTCisVT<1, i16>]>;
def SDT_GameBoyCallSeqEnd : SDCallSeqEnd<[SDTCisVT<0, i16>, SDTCisVT<1, i16>]>;
def SDT_GameBoyCall : SDTypeProfile<0, -1, [SDTCisVT<0, iPTR>]>;
def SDT_GameBoyWrapper : SDTypeProfile<1, 1, [SDTCisSameAs<0, 1>, SDTCisPtrTy<0>]>;
def SDT_GameBoyBrcond
    : SDTypeProfile<0, 2, [SDTCisVT<0, OtherVT>, SDTCisVT<1, i8>]>;
def SDT_GameBoyCmp : SDTypeProfile<0, 2, [SDTCisSameAs<0, 1>]>;
def SDT_GameBoyTst : SDTypeProfile<0, 1, [SDTCisInt<0>]>;
def SDT_GameBoySelectCC
    : SDTypeProfile<1, 3,
                    [SDTCisSameAs<0, 1>, SDTCisSameAs<1, 2>, SDTCisVT<3, i8>]>;

//===----------------------------------------------------------------------===//
// AVR Specific Node Definitions
//===----------------------------------------------------------------------===//

def GameBoyretflag : SDNode<"GameBoyISD::RET_FLAG", SDTNone,
                        [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]>;
def GameBoyretiflag : SDNode<"GameBoyISD::RETI_FLAG", SDTNone,
                         [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]>;

def GameBoycallseq_start : SDNode<"ISD::CALLSEQ_START", SDT_GameBoyCallSeqStart,
                              [SDNPHasChain, SDNPOutGlue]>;
def GameBoycallseq_end : SDNode<"ISD::CALLSEQ_END", SDT_GameBoyCallSeqEnd,
                            [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue]>;

def GameBoycall : SDNode<"GameBoyISD::CALL", SDT_GameBoyCall,
                     [SDNPHasChain, SDNPOutGlue, SDNPOptInGlue, SDNPVariadic]>;

def GameBoyWrapper : SDNode<"GameBoyISD::WRAPPER", SDT_GameBoyWrapper>;

def GameBoybrcond
    : SDNode<"GameBoyISD::BRCOND", SDT_GameBoyBrcond, [SDNPHasChain, SDNPInGlue]>;
def GameBoycmp : SDNode<"GameBoyISD::CMP", SDT_GameBoyCmp, [SDNPOutGlue]>;
def GameBoycmpc : SDNode<"GameBoyISD::CMPC", SDT_GameBoyCmp, [SDNPInGlue, SDNPOutGlue]>;
def GameBoytst : SDNode<"GameBoyISD::TST", SDT_GameBoyTst, [SDNPOutGlue]>;
def GameBoyselectcc : SDNode<"GameBoyISD::SELECT_CC", SDT_GameBoySelectCC, [SDNPInGlue]>;

// Shift nodes.
def GameBoylsl : SDNode<"GameBoyISD::LSL", SDTIntUnaryOp>;
def GameBoylsr : SDNode<"GameBoyISD::LSR", SDTIntUnaryOp>;
def GameBoyrol : SDNode<"GameBoyISD::ROL", SDTIntUnaryOp>;
def GameBoyror : SDNode<"GameBoyISD::ROR", SDTIntUnaryOp>;
def GameBoyasr : SDNode<"GameBoyISD::ASR", SDTIntUnaryOp>;
def GameBoylslhi : SDNode<"GameBoyISD::LSLHI", SDTIntUnaryOp>;
def GameBoylsrlo : SDNode<"GameBoyISD::LSRLO", SDTIntUnaryOp>;
def GameBoyasrlo : SDNode<"GameBoyISD::ASRLO", SDTIntUnaryOp>;
def GameBoylslbn : SDNode<"GameBoyISD::LSLBN", SDTIntBinOp>;
def GameBoylsrbn : SDNode<"GameBoyISD::LSRBN", SDTIntBinOp>;
def GameBoyasrbn : SDNode<"GameBoyISD::ASRBN", SDTIntBinOp>;
def GameBoylslwn : SDNode<"GameBoyISD::LSLWN", SDTIntBinOp>;
def GameBoylsrwn : SDNode<"GameBoyISD::LSRWN", SDTIntBinOp>;
def GameBoyasrwn : SDNode<"GameBoyISD::ASRWN", SDTIntBinOp>;

// Pseudo shift nodes for non-constant shift amounts.
def GameBoylslLoop : SDNode<"GameBoyISD::LSLLOOP", SDTIntShiftOp>;
def GameBoylsrLoop : SDNode<"GameBoyISD::LSRLOOP", SDTIntShiftOp>;
def GameBoyrolLoop : SDNode<"GameBoyISD::ROLLOOP", SDTIntShiftOp>;
def GameBoyrorLoop : SDNode<"GameBoyISD::RORLOOP", SDTIntShiftOp>;
def GameBoyasrLoop : SDNode<"GameBoyISD::ASRLOOP", SDTIntShiftOp>;

// SWAP node.
def GameBoySwap : SDNode<"GameBoyISD::SWAP", SDTIntUnaryOp>;

//===----------------------------------------------------------------------===//
// Game Boy Operands, Complex Patterns and Transformations Definitions
//===----------------------------------------------------------------------===//

// For 8-bit immediate pointers, the range must be 0xFF00 - 0xFFFF, with the
// 8-bit operand taking up the lower 16-bits of the range.
// E.g. LD A, (FF34) ; loads data at 0xFF34 into A.
// LD A, (34) ; incorrect usage, and will load data from 0x0034 into A.
def iaddr8 : Operand<i8> {

}

// def regpair_addr : Operand<>

//===----------------------------------------------------------------------===//
// AVR Operands, Complex Patterns and Transformations Definitions.
//===----------------------------------------------------------------------===//

def imm8_neg_XFORM : SDNodeXForm<imm, [{
                                   return CurDAG->getTargetConstant(
                                       -N->getAPIntValue(), SDLoc(N), MVT::i8);
                                 }]>;

def imm16_neg_XFORM
    : SDNodeXForm<imm, [{
                    return CurDAG->getTargetConstant(-N->getAPIntValue(),
                                                     SDLoc(N), MVT::i16);
                  }]>;

def imm0_63_neg : PatLeaf<(imm), [{
                            int64_t val = -N->getSExtValue();
                            return val >= 0 && val < 64;
                          }],
                          imm16_neg_XFORM>;

def uimm6 : PatLeaf<(imm), [{ return isUInt<6>(N->getZExtValue()); }]>;

// imm_com8_XFORM - Return the complement of a imm_com8 value
def imm_com8_XFORM
    : SDNodeXForm<imm, [{
                    return CurDAG->getTargetConstant(
                        ~((uint8_t) N->getZExtValue()), SDLoc(N), MVT::i8);
                  }]>;

// imm_com8 - Match an immediate that is a complement
// of a 8-bit immediate.
// Note: this pattern doesn't require an encoder method and such, as it's
// only used on aliases (Pat<> and InstAlias<>). The actual encoding
// is handled by the destination instructions, which use imm_com8.
def imm_com8_asmoperand : AsmOperandClass { let Name = "ImmCom8"; }
def imm_com8 : Operand<i8> { let ParserMatchClass = imm_com8_asmoperand; }

def ioaddr_XFORM
    : SDNodeXForm<imm, [{
                    uint8_t offset = Subtarget->getIORegisterOffset();
                    return CurDAG->getTargetConstant(
                        uint8_t(N->getZExtValue()) - offset, SDLoc(N), MVT::i8);
                  }]>;

def iobitpos8_XFORM
    : SDNodeXForm<imm, [{
                    return CurDAG->getTargetConstant(
                        Log2_32(uint8_t(N->getZExtValue())), SDLoc(N), MVT::i8);
                  }]>;

def iobitposn8_XFORM : SDNodeXForm<imm, [{
                                     return CurDAG->getTargetConstant(
                                         Log2_32(uint8_t(~N->getZExtValue())),
                                         SDLoc(N), MVT::i8);
                                   }]>;

def ioaddr8 : PatLeaf<(imm), [{
                        uint8_t offset = Subtarget->getIORegisterOffset();
                        uint64_t val = N->getZExtValue() - offset;
                        return val < 0x40;
                      }],
                      ioaddr_XFORM>;

def lowioaddr8 : PatLeaf<(imm), [{
                           uint8_t offset = Subtarget->getIORegisterOffset();
                           uint64_t val = N->getZExtValue() - offset;
                           return val < 0x20;
                         }],
                         ioaddr_XFORM>;

def ioaddr16 : PatLeaf<(imm), [{
                         uint8_t offset = Subtarget->getIORegisterOffset();
                         uint64_t val = N->getZExtValue() - offset;
                         return val < 0x3f;
                       }],
                       ioaddr_XFORM>;

def iobitpos8
    : PatLeaf<(imm), [{ return isPowerOf2_32(uint8_t(N->getZExtValue())); }],
              iobitpos8_XFORM>;

def iobitposn8
    : PatLeaf<(imm), [{ return isPowerOf2_32(uint8_t(~N->getZExtValue())); }],
              iobitposn8_XFORM>;

def MemriAsmOperand : AsmOperandClass {
  let Name = "Memri";
  let ParserMethod = "parseMemriOperand";
}

/// Address operand for `reg+imm` used by STD and LDD.
def memri : Operand<iPTR> {
  let MIOperandInfo = (ops PTRDISPREGS, i16imm);

  let PrintMethod = "printMemri";
  let EncoderMethod = "encodeMemri";
  let DecoderMethod = "decodeMemri";

  let ParserMatchClass = MemriAsmOperand;
}

// Address operand for `SP+imm` used by STD{W}SPQRr
def memspi : Operand<iPTR> {
  let MIOperandInfo = (ops GPRSP, i16imm);
  let PrintMethod = "printMemspi";
}

def relbrtarget_7 : Operand<OtherVT> {
  let PrintMethod = "printPCRelImm";
  let EncoderMethod = "encodeRelCondBrTarget<GameBoy::fixup_7_pcrel>";
}

def brtarget_13 : Operand<OtherVT> {
  let PrintMethod = "printPCRelImm";
  let EncoderMethod = "encodeRelCondBrTarget<GameBoy::fixup_13_pcrel>";
}

def rcalltarget_13 : Operand<i16> {
  let PrintMethod = "printPCRelImm";
  let EncoderMethod = "encodeRelCondBrTarget<GameBoy::fixup_13_pcrel>";
}

// The target of a 22 or 16-bit call/jmp instruction.
def call_target : Operand<iPTR> {
  let EncoderMethod = "encodeCallTarget";
  let DecoderMethod = "decodeCallTarget";
}

// A 16-bit address (which can lead to an R_GameBoy_16 relocation).
def imm16 : Operand<i16> { let EncoderMethod = "encodeImm<GameBoy::fixup_16, 2>"; }

/// A 6-bit immediate used in the ADIW/SBIW instructions.
def imm_arith6 : Operand<i16> {
  let EncoderMethod = "encodeImm<GameBoy::fixup_6_adiw, 0>";
}

/// An 8-bit immediate inside an instruction with the same format
/// as the `LDI` instruction (the `FRdK` format).
def imm_ldi8 : Operand<i8> {
  let EncoderMethod = "encodeImm<GameBoy::fixup_ldi, 0>";
}

/// A 5-bit port number used in SBIC and friends (the `FIOBIT` format).
def imm_port5 : Operand<i8> {
  let EncoderMethod = "encodeImm<GameBoy::fixup_port5, 0>";
}

/// A 6-bit port number used in the `IN` instruction and friends (the
/// `FIORdA` format.
def imm_port6 : Operand<i8> {
  let EncoderMethod = "encodeImm<GameBoy::fixup_port6, 0>";
}

// Addressing mode pattern reg+imm6
def addr : ComplexPattern<iPTR, 2, "SelectAddr", [], [SDNPWantRoot]>;

// AsmOperand class for a pointer register.
// Used with the LD/ST family of instructions.
// See FSTLD in GameBoyInstrFormats.td
def PtrRegAsmOperand : AsmOperandClass { let Name = "Reg"; }

// A special operand type for the LD/ST instructions.
// It converts the pointer register number into a two-bit field used in the
// instruction.
def LDSTPtrReg : Operand<i16> {
  let MIOperandInfo = (ops PTRREGS);
  let EncoderMethod = "encodeLDSTPtrReg";

  let ParserMatchClass = PtrRegAsmOperand;
}

// A special operand type for the LDD/STD instructions.
// It behaves identically to the LD/ST version, except restricts
// the pointer registers to Y and Z.
def LDDSTDPtrReg : Operand<i16> {
  let MIOperandInfo = (ops PTRDISPREGS);
  let EncoderMethod = "encodeLDSTPtrReg";

  let ParserMatchClass = PtrRegAsmOperand;
}

//===----------------------------------------------------------------------===//
// AVR predicates for subtarget features
//===----------------------------------------------------------------------===//

def HasSRAM : Predicate<"Subtarget->hasSRAM()">,
              AssemblerPredicate<(all_of FeatureSRAM)>;

def HasJMPCALL : Predicate<"Subtarget->hasJMPCALL()">,
                 AssemblerPredicate<(all_of FeatureJMPCALL)>;

def HasIJMPCALL : Predicate<"Subtarget->hasIJMPCALL()">,
                  AssemblerPredicate<(all_of FeatureIJMPCALL)>;

def HasEIJMPCALL : Predicate<"Subtarget->hasEIJMPCALL()">,
                   AssemblerPredicate<(all_of FeatureEIJMPCALL)>;

def HasADDSUBIW : Predicate<"Subtarget->hasADDSUBIW()">,
                  AssemblerPredicate<(all_of FeatureADDSUBIW)>;

def HasSmallStack : Predicate<"Subtarget->HasSmallStack()">,
                    AssemblerPredicate<(all_of FeatureSmallStack)>;

def HasMOVW : Predicate<"Subtarget->hasMOVW()">,
              AssemblerPredicate<(all_of FeatureMOVW)>;

def HasLPM : Predicate<"Subtarget->hasLPM()">,
             AssemblerPredicate<(all_of FeatureLPM)>;

def HasLPMX : Predicate<"Subtarget->hasLPMX()">,
              AssemblerPredicate<(all_of FeatureLPMX)>;

def HasELPM : Predicate<"Subtarget->hasELPM()">,
              AssemblerPredicate<(all_of FeatureELPM)>;

def HasELPMX : Predicate<"Subtarget->hasELPMX()">,
               AssemblerPredicate<(all_of FeatureELPMX)>;

def HasSPM : Predicate<"Subtarget->hasSPM()">,
             AssemblerPredicate<(all_of FeatureSPM)>;

def HasSPMX : Predicate<"Subtarget->hasSPMX()">,
              AssemblerPredicate<(all_of FeatureSPMX)>;

def HasDES : Predicate<"Subtarget->hasDES()">,
             AssemblerPredicate<(all_of FeatureDES)>;

def SupportsRMW : Predicate<"Subtarget->supportsRMW()">,
                  AssemblerPredicate<(all_of FeatureRMW)>;

def SupportsMultiplication : Predicate<"Subtarget->supportsMultiplication()">,
                             AssemblerPredicate<(all_of FeatureMultiplication)>;

def HasBREAK : Predicate<"Subtarget->hasBREAK()">,
               AssemblerPredicate<(all_of FeatureBREAK)>;

def HasTinyEncoding : Predicate<"Subtarget->hasTinyEncoding()">,
                      AssemblerPredicate<(all_of FeatureTinyEncoding)>;

// GameBoy specific condition code. These correspond to GameBoy_*_COND in
// GameBoyInstrInfo.td. They must be kept in synch.
def GameBoy_COND_EQ : PatLeaf<(i8 0)>;
def GameBoy_COND_NE : PatLeaf<(i8 1)>;
def GameBoy_COND_GE : PatLeaf<(i8 2)>;
def GameBoy_COND_LT : PatLeaf<(i8 3)>;
def GameBoy_COND_SH : PatLeaf<(i8 4)>;
def GameBoy_COND_LO : PatLeaf<(i8 5)>;
def GameBoy_COND_MI : PatLeaf<(i8 6)>;
def GameBoy_COND_PL : PatLeaf<(i8 7)>;

// Game Boy condition codes
// Zero / Not Zero
def GameBoy_COND_Z : PatLeaf<(i8 8)>;
def GameBoy_COND_NZ : PatLeaf<(i8 9)>;
// Carry / Not Carry
def GameBoy_COND_C : PatLeaf<(i8 10)>;
def GameBoy_COND_NC : PatLeaf<(i8 11)>;
// Subtraction / Not Subtraction
// def GameBoy_COND_N : PatLeaf<(i8 10)>;
// def GameBoy_COND_NN : PatLeaf<(i8 11)>;
// Half Carry / No Half Carry
// def GameBoy_COND_H : PatLeaf<(i8 12)>;
// def GameBoy_COND_NH : PatLeaf<(i8 13)>;

//===----------------------------------------------------------------------===//
//===----------------------------------------------------------------------===//
// GameBoy Instruction list
//
// Instructions are being re-written here so that the AVR instructions can
// eventually be phased out entirely by deleting them from the file.
// 
// TODO: Add implicit register uses to instructions, especially HL and F
//===----------------------------------------------------------------------===//
//===----------------------------------------------------------------------===//

// A special operand type for the LDRdRr instruction, which converts
// each register number into a 3-bit field used in the instruction.
def LDRdRrReg : Operand<i16> {
  let MIOperandInfo = (ops GPR);
  let EncoderMethod = "encodeLDRdRrReg";
  let ParserMatchClass = PtrRegAsmOperand;
}

// ADJCALLSTACKDOWN/UP implicitly use/def SP because they may be expanded into
// a stack adjustment and the codegen must know that they may modify the stack
// pointer before prolog-epilog rewriting occurs.
// Pessimistically assume ADJCALLSTACKDOWN / ADJCALLSTACKUP will become
// sub / add which can clobber SREG.
let Defs = [SP, SREG], Uses = [SP] in {
  def ADJCALLSTACKDOWN : Pseudo<(outs),
                                (ins i16imm
                                 : $amt, i16imm
                                 : $amt2),
                                "#ADJCALLSTACKDOWN", [(GameBoycallseq_start timm
                                                       : $amt, timm
                                                       : $amt2)]>;

  // R31R30 is used to update SP. It is normally free because it is a
  // call-clobbered register but it is necessary to set it as a def as the
  // register allocator might use it in rare cases (for rematerialization, it
  // seems). hasSideEffects needs to be set to true so this instruction isn't
  // considered dead.
  let Defs = [R31R30], hasSideEffects = 1 in def ADJCALLSTACKUP
      : Pseudo<(outs),
               (ins i16imm
                : $amt1, i16imm
                : $amt2),
               "#ADJCALLSTACKUP", [(GameBoycallseq_end timm
                                    : $amt1, timm
                                    : $amt2)]>;
}


//===----------------------------------------------------------------------===//
// 8-bit transfer and input/output pseudo instructions
//===----------------------------------------------------------------------===//

let mayLoad = 1 in {
// Pseudo instruction needed to lower LD Rd8, &Ptr
// LD Rd, (RR)
def LDRd8Ptr : Pseudo< (outs GPR : $rd),
                  (ins GPRPair : $ptrreg),
                  "ld\t$rd, [ $ptrreg ]",
                  [(set GPR : $rd, (load i16 : $ptrreg))]>;
}

//===----------------------------------------------------------------------===//
// 8-bit transfer and input/output instructions
//===----------------------------------------------------------------------===//
let hasSideEffects = 0 in {
def LDRdRr : GBFRdRr<0b01,
                      (outs GPR
                      : $rd),
                      (ins GPR
                      : $rr),
                      "ld\t$rd, $rr",
                      // [(set GPR : $rd, GPR : $rr)]>;
                      []>; // This is handled by copyPhysReg.
}

// [(Imm8) rule says that (Imm8) = in range FF00 - FFFF]
let isReMaterializable = 1 in {

// LD Rd, Imm8
def LDRdImm8 : GBFRd< 0b00, 0b110,
                      (outs GPR : $rd),
                      (ins i8imm : $imm8),
                      "ld\t$rd, $imm8",
                      [(set i8 : $rd, imm : $imm8)]>;
                      // []>;
}

let mayLoad = 1 in {


// To move entire register pairs, the following pseudo instruction
// must be constructed. LD RdH, RrH; LD RdL, RrL

// LD Rd, (HL)
def LDRdHLPtr : GBFRd< 0b01, 0b110,
                        (outs GPR : $rd),
                        (ins GPRPairPointerHL : $ptrreg),
                        "ld\t$rd, [hl]",
                        [(set GPR : $rd, (load i16 : $ptrreg))]>;

// LD A, (rr) where rr is a register pair
// LD A, (BC)
// LD A, (DE)
def LDAPtr : GBFLdAPtr< (outs GPRLoad : $rd),
                        (ins GPRPairPointer : $ptrreg),
                        "ld\ta, [ $ptrreg ]",
                        [(set GPRLoad :
                          $rd, (load i16 : $ptrreg))]>;

// LD A, (C)
// Applies specifically to the range FF00 - FFFF

// LD A, (Imm8) [ rules apply for (Imm8)]
def LDAImm8Addr : GBF0< 0b11110000,
                        (outs GPRLoad : $rd),
                        (ins i8imm : $addr),
                        "ld\ta, $addr",
                        [(set GPRLoad : $rd, (load i8 : $addr))]>;
// LD A, (Imm16)
def LDAImm16Addr : GBF0< 0b11111010,
                              (outs GPRLoad : $rd),
                              (ins i16imm : $addr),
                              "ld\ta, $addr",
                              [(set GPRLoad :
                                $rd, (load i16 : $addr))]>;
}

let mayStore = 1 in {
// Need a psuedo instruction for LD Ptr, Rr


// LD (HL), Rr
def LDHLAddrRr : GBFRd2<0b01110,
                        (outs),
                        (ins GPRPairPointerHL : $ptrreg, GPR : $rr),
                        "ld\t$ptrreg, $rr", // Brackets added in GameBoyInstPrinter.cpp
                        [(store GPR : $rr, GPRPairPointerHL : $ptrreg)]>;

// LD (HL), Imm8
def LDHLAddrImm8 : GBF0<0b00110110,
                            (outs),
                            (ins GPRPairPointerHL : $ptrreg, i8imm : $n),
                            "ld\t[hl], $n",
                            [(store i8 : $n, GPRPairPointerHL : $ptrreg)]>;

// LD (C), A
def LDCAddrA : GBF0<0b11100010,
                    (outs),
                    (ins GPRPointerC : $rcaddr, GPRLoad : $ra),
                    "ld\t[c], a",
                    [(store GPRLoad : $ra, GPRPointerC : $rcaddr)]>;

}

let mayStore = 1 in {
// LD (Imm8), A [ rules apply for (Imm8)]
def LDImm8AddrA : GBF0< 0b11100000,
                        (outs),
                        (ins GPRLoad : $ra, i8imm : $n),
                        "ld\t$n [addr], a",
                        [(store i8 : $ra, imm : $n)]>;
// LD (Imm16), A
def LDImm16AddrA : GBF0< 0b11101010,
                        (outs),
                        (ins GPRLoad : $ra, i16imm : $n),
                        "ld\t$n [addr], a",
                        [(store i8 : $ra, imm : $n)]>;
                        // []>;

// LD (rr), A : Load A into a location in memory.
// LD (BC), A
// LD (DE), A
def LDPtrA : GBFLdPtrA< (outs),
                        (ins GPRPairPointer : $ptrreg, GPRLoad : $ra),
                        "ld\t[ $ptrreg ], a",
                        [(store GPRLoad : $ra, GPRPairPointer : $ptrreg)]>;
}

let mayLoad = 1, hasSideEffects = 1 in {

// LD A, (HL+)
/*
def LDAHLInc : GBF0<0b00101010, 
                    (outs GPRLoadDestination : $rd),
                    (ins GPRPairHLLoadInc : $rr),
                    "ld\ta, [hl+]",
                    [(store GPRLoadDestination : $rd, )]>;
*/
// LD A, (HL-)
}


let mayStore = 1 in {

// Equivalent to:
// (HL) <- A
// HL <- HL ± 1

// LD (HL+), A 
// LD (HL-), A
}

//===----------------------------------------------------------------------===//
// 16-bit transfer pseudo instructions
//===----------------------------------------------------------------------===//

def LDRdPairRrPair : Pseudo<(outs GPRPair : $rd),
                            (ins GPRPair : $rr),
                            "ld\t$rd, $rr",
                            // [(set GPRPair : $rd, GPRPair : $rr)]>;
                            []>; // Handled by CopyPhysReg



//===----------------------------------------------------------------------===//
// 16-bit transfer instructions
//===----------------------------------------------------------------------===//

let isReMaterializable = 1 in  {

// LD dd, Imm16
def LDRdImm16 : GBFPair<0b00, 0b0001,
                        (outs GPRPair : $rd),
                        (ins i16imm : $src),
                        "ld\t$rd, $src",
                        [(set i16 : $rd, imm : $src)]>;
}

// LD SP, HL
let hasSideEffects = 0 in {
  def LDSPHL : GBF0<0b11111001,
                    (outs SPLoadRd : $sp),
                    (ins SPLoadRr : $hl),
                    "LD\tSP, HL",
                    []>;
}

let mayLoad = 1 in {
// POPRd / POP qq
  def POPRd : GBFPushPop<0b001,
                  (outs StackRegisters
                    : $reg),
                  (ins),
                  "pop\t$reg",
                  []>;

// PUSHRd / PUSH qq
  def PUSHRd : GBFPushPop<0b101,
                  (outs),
                  (ins StackRegisters
                    : $reg),
                  "push\t$reg",
                  []>;

// PUSH HL
// Used for pseudo-instructions where HL is used as a temporary.
}

// LDHL SP, s8

// LD (Imm16), SP

//===----------------------------------------------------------------------===//
// 8-bit arithmetic pseudo instructions
//===----------------------------------------------------------------------===//
let Constraints = "$src = $rd" in {
// AddRdRr = ADD Rd, Rr
// Expands to 
// LD A, Rr
// ADD A, Rd
// LD Rd, A
def AddRdRr : Pseudo< (outs GPR : $rd),
                      (ins GPR : $src, GPR : $rr),
                      "add\t$rd, $rr",
                      [(set i8 : $rd, 
                          (add i8 : $src, i8 : $rr))]>;

// ADD R, Imm8
// Expands to 
// ADD A, Imm8
// LD A, R
def AddRdImm8 : Pseudo< (outs GPR : $rd),
                        (ins GPR : $src, i8imm : $n),
                        "add\t$rd, $n",
                        [(set i8 : $rd, (add i8 : $src, i8 : $n))]>;
}

//===----------------------------------------------------------------------===//
// 8-bit arithmetic instructions
//===----------------------------------------------------------------------===//

// Force us to store the results in the same register we use as a source.
// In other words, A is used as a source as well as the destination, whilst $rr
// is any other 8-bit register.
let Constraints = "$src = $rd" in {
// ADD A, r
def AddARr : GBFRd2<0b10000,
                    (outs GPRLoad : $rd),
                    (ins GPR : $src, GPR : $rr),
                    "add\ta, $rr",
                    // [(set i8 : $rd,
                    //     (add i8 : $src, i8 : $rr))]>;
                      []>;

// ADD A, Imm8
def AddAImm8 : GBF0<0b11000110,
                      (outs GPRLoad : $rd),
                      (ins GPRLoad : $src, i8imm : $n),
                      "add\ta, $n",
                      // [(set i8 : $rd, (add i8 : $src, i8 : $n))]>;
                      []>;
}

// ADD A, (HL)
// Need to handle truncating the (HL) operand.
// def AddAHLPtr : GBFLdPtrA< (outs),
//                         (ins GPRPairPointer : $ptrreg, GPRLoad : $ra),
//                         "ld\t[ $ptrreg ], a",
//                         [(store GPRLoad : $ra, GPRPairPointer : $ptrreg)]>;

let Constraints = "$src = $rd" in {
// ADC A, r
def AdcARr : GBFRd2<0b10001,
                    (outs GPRLoad : $rd),
                    (ins GPRLoad : $src, GPR : $rr),
                    "adc\t$rd, $rr",
                    [(set i8 : $rd, (adde i8 : $src, i8 : $rr))]>;

// ADC A, Imm8
def AdcAImm8 : GBF0<0b11001110,
                      (outs GPRLoad : $rd),
                      (ins GPRLoad : $src, i8imm : $n),
                      "add\ta, $n",
                      [(set i8 : $rd, (adde i8 : $src, i8 : $n))]>;
}

// ADC A, (HL)

let Constraints = "$src = $rd" in {
// SUB A, r
def SubARr : GBFRd2<0b10010,
                    (outs GPRLoad : $rd),
                    (ins GPR : $src, GPR : $rr),
                    "sub\ta, $rr",
                    [(set i8 : $rd,
                        (sub i8 : $src, i8 : $rr))]>;

// SUB A, Imm8
def SubAImm8 : GBF0<0b11010110,
                      (outs GPRLoad : $rd),
                      (ins GPRLoad : $src, i8imm : $n),
                      "sub\ta, $n",
                      [(set i8 : $rd, (sub i8 : $src, i8 : $n))]>;
}
// SUB A, (HL)

let Constraints = "$src = $rd" in {
// SBC A, r
def SbcARr : GBFRd2<0b10011,
                    (outs GPRLoad : $rd),
                    (ins GPRLoad : $src, GPR : $rr),
                    "sbc\t$rd, $rr",
                    [(set i8 : $rd, (sube i8 : $src, i8 : $rr))]>;

// SBC A, Imm8
def SbcAImm8 : GBF0<0b11011110,
                      (outs GPRLoad : $rd),
                      (ins GPRLoad : $src, i8imm : $n),
                      "sbc\ta, $n",
                      [(set i8 : $rd, (sube i8 : $src, i8 : $n))]>;
}


// SBC A, (HL)

//===----------------------------------------------------------------------===//
// 8-bit logic instructions
//===----------------------------------------------------------------------===//

// Pseudo Instructions (excluding (HL))
// AND Rd, Rr
let Constraints = "$src = $rd" in {
let isCommutable = 1 in {
def AndRdRr : Pseudo< (outs GPR : $rd),
                      (ins GPR : $src, GPR : $rr),
                      "and\t$rd, $rr",
                      [(set i8 : $rd, (and i8 : $src, i8 : $rr))]>;
// OR Rd, Rr
def OrRdRr : Pseudo< (outs GPR : $rd),
                      (ins GPR : $src, GPR : $rr),
                      "or\t$rd, $rr",
                      [(set i8 : $rd, (or i8 : $src, i8 : $rr))]>;
// XOR Rd, Rr
def XorRdRr : Pseudo< (outs GPR : $rd),
                      (ins GPR : $src, GPR : $rr),
                      "xor\t$rd, $rr",
                      [(set i8 : $rd, (xor i8 : $src, i8 : $rr))]>;
}
// AND Rd, Imm8
def AndRdImm8 : Pseudo< (outs GPR : $rd),
                        (ins GPR : $src, i8imm : $n),
                        "and\t$rd, $n",
                        [(set i8 : $rd, (and i8 : $src, i8 : $n))]>;
// OR Rd, Imm8
def OrRdImm8 : Pseudo<  (outs GPR : $rd),
                        (ins GPR : $src, i8imm : $n),
                        "or\t$rd, $n",
                        [(set i8 : $rd, (or i8 : $src, i8 : $n))]>;
// XOR Rd, Imm8
def XorRdImm8 : Pseudo< (outs GPR : $rd),
                        (ins GPR : $src, i8imm : $n),
                        "xor\t$rd, $n",
                        [(set i8 : $rd, (xor i8 : $src, i8 : $n))]>;
}

let Defs = [RF] in {
// CP Rd, Rr
def CpRdRr : Pseudo<(outs),
                    (ins GPR : $rd, GPR : $rr),
                    "cp\t$rd, $rr",
                    [(GameBoycmp i8 : $rd, i8 : $rr), (implicit RF)]>;

// CPW Rd, Rr
def CpWRdRr : Pseudo< (outs),
                      (ins GPRPair : $rd, GPRPair : $rr),
                      "cp\t$rd, $rr",
                      [(GameBoycmp i16 : $rd, i16 : $rr), (implicit RF)]>;

// CP Rd, Imm8
def CpRdImm8 : Pseudo<(outs),
                    (ins GPR : $rd, i8imm : $n),
                    "cp\t$rd, $n",
                    [(GameBoycmp i8 : $rd, i8 : $n), (implicit RF)]>;

// CP Rd, Imm16
def CpRdImm16 : Pseudo<
                    (outs),
                    (ins GPRPair : $rd, i16imm : $n),
                    "cp\t$rd, $n",
                    [(GameBoycmp i16 : $rd, i16 : $n), (implicit RF)]>;
}

// Original instructions are as follows
let Constraints = "$src = $rd" in {
// AND A, r
def AndARr : GBFRd2<0b10100,
                    (outs GPR : $rd),
                    (ins GPR : $src, GPR : $rr),
                    "and\t$rd, $rr",
                    []>;
// AND A, Imm8
def AndAImm8 : GBF0<0b11100110,
                    (outs GPR : $rd),
                    (ins GPR : $src, i8imm : $n),
                    "and\t$rd, $n",
                    []>;
// OR A, r
def OrARr :  GBFRd2<0b10110,
                    (outs GPR : $rd),
                    (ins GPR : $src, GPR : $rr),
                    "or\t$rd, $rr",
                    []>;
// OR A, Imm8
def OrAImm8 : GBF0<0b11110110,
                    (outs GPR : $rd),
                    (ins GPR : $src, i8imm : $n),
                    "or\t$rd, $n",
                    []>;
// XOR A, r
def XorARr :  GBFRd2<0b10101,
                    (outs GPR : $rd),
                    (ins GPR : $src, GPR : $rr),
                    "xor\t$rd, $rr",
                    []>;
// XOR A, Imm8
def XorAImm8 : GBF0<0b11101110,
                    (outs GPR : $rd),
                    (ins GPR : $src, i8imm : $n),
                    "xor\t$rd, $n",
                    []>;
}

// CP A, r
def CPARr :  GBFRd2<0b10111,
                    (outs),
                    (ins GPR : $rd, GPR : $rr),
                    "cp\t$rd, $rr",
                    []>;
// CP A, Imm8
def CPAImm8 : GBF0<0b11111110,
                    (outs),
                    (ins GPR : $rd, i8imm : $n),
                    "cp\t$rd, $n",
                    []>;

// AND A, (HL)
// OR A, (HL)
// XOR A, (HL)
// CP A, (HL)

// INC r
let Constraints = "$src = $rd" in {
def INCRd : GBFRd<0b00, 0b100,
                  (outs GPR : $rd),
                  (ins GPR : $src),
                  "inc\t$rd",
                  [(set i8 : $rd, (add i8
                                  : $src, 1))]>;
// DEC r
def DECRd : GBFRd<0b00, 0b101,
                  (outs GPR : $rd),
                  (ins GPR : $src),
                  "inc\t$rd",
                  [(set i8 : $rd, (add i8
                                  : $src, -1))]>;
}
// INC (HL)

// DEC (HL)

//===----------------------------------------------------------------------===//
// 16-bit arithmetic pseudo instructions
//===----------------------------------------------------------------------===//

// ADD RPd, Imm16
// Uses HL as a temporary.
// the expand will PUSH and POP HL from the stack accordingly.
// let Constraints = "$src = $rd", Uses = [RHRL] in {
let Defs = [RHRL], Uses = [RHRL], Constraints = "$src = $rd" in {
// let Constraints = "$src = $rd" in {
def AddRpdImm16 : Pseudo< (outs GPRPair : $rd),
                          (ins GPRPair : $src, i16imm : $n),
                          "add\t$rd, $n",
                          [(set i16 : $rd, (add i16 : $src, i16 : $n))]>;

// ADD RdPair, RrPair
// Uses HL as a temporary; the expand will PUSH and POP HL from the
// stack accordingly.
def AddRdPairRrPair : Pseudo< (outs GPRPair : $rd),
                              (ins GPRPair : $src, GPRPair : $rr),
                              "add\t$rd, $rr",
                              [(set i16 : $rd, (add i16 : $src, i16 : $rr))]>;
}


//===----------------------------------------------------------------------===//
// 16-bit arithmetic instructions
//===----------------------------------------------------------------------===//

// ADD HL, RPd
let Constraints = "$src = $rd" in {
def ADDHLPair : GBFPair<0b00, 0b1001,
                        (outs GPRPairPointerHL : $rd),
                        (ins GPRPairPointerHL : $src, GPRPair : $rr),
                        "add\t$rd, $rr",
                        []>; // Manually selected by pseudo instructions.
                        // [(set i16 : $rd, (add i16 : $src, i16 : $rr))]>;
}
// ADD SP, s8

let Constraints = "$src = $rd" in {
// INC RPd
def INCPair : GBFPair<0b00, 0b0011,
                      (outs GPRPair : $rd),
                      (ins GPRPair : $src),
                      "inc\t$rd",
                      [(set i16 : $rd, (add i16 : $src, 1))]>;
// DEC RPd
def DECPair : GBFPair<0b00, 0b1011,
                      (outs GPRPair : $rd),
                      (ins GPRPair : $src),
                      "dec\t$rd",
                      [(set i16 : $rd, (add i16 : $src, -1))]>;
}

//===----------------------------------------------------------------------===//
// Rotate shift instructions
// Check which of these use RA
//===----------------------------------------------------------------------===//

// RLCA
// RRCA

// RLA
// RRA

// RLC r
// RLC (HL)
// RRC r
// RRC (HL)

// RL r
// RL (HL)
// RR r
// RR (HL)

// SLA r
// SLA (HL)

// SRA r
// SRA (HL)
// SRL r
// SRL (HL)

// SWAP r
// SWAP (HL)

//===----------------------------------------------------------------------===//
// Bit operations
//===----------------------------------------------------------------------===//

// BIT b, r
// BIT b, (HL)

// SET b, r
// SET b, (HL)

// RES b, r
// RES b, (HL)

//===----------------------------------------------------------------------===//
// Jump Instructions
//===----------------------------------------------------------------------===//

// JP Imm16
let isBarrier = 1, isBranch = 1, isTerminator = 1 in {
  def JP : GBF0<0b11000011,
                (outs),
                (ins call_target : $k),
                "jp\t$k",
                []>; // Handled by LLVM insertIndirectBranch
}
// JP cc, Imm16
// JP (HL)

let isBarrier = 1, isBranch = 1, isTerminator = 1 in {
// JR s8 (-2)
  def JRk : GBF0< 0b00011000,
                  (outs),
                  (ins relbrtarget_7 : $target),
                  "jr\t$target",
                  [(br bb : $target)]>;
}
// JR cc, s8 (-2)
let isBranch = 1, isTerminator = 1, Uses = [RF] in {
  def JRNZk : GBF0<0b11000010,
                  (outs),
                  (ins relbrtarget_7 : $target),
                  "jr\tnz, $target",
                  [(GameBoybrcond bb : $target, GameBoy_COND_NZ)]>;

  def JRZk : GBF0<0b11001010,
                  (outs),
                  (ins relbrtarget_7 : $target),
                  "jr\tz, $target",
                  [(GameBoybrcond bb : $target, GameBoy_COND_Z)]>;
  
  def JRNCk : GBF0< 0b11010010,
                    (outs),
                    (ins relbrtarget_7 : $target),
                    "jr\tnc, $target",
                    [(GameBoybrcond bb : $target, GameBoy_COND_NC)]>;
  
  def JRCk : GBF0<0b11011010,
                  (outs),
                  (ins relbrtarget_7 : $target),
                  "jr\tc, $target",
                  [(GameBoybrcond bb : $target, GameBoy_COND_C)]>;

  // Pseudo instructions that are composed of the existing
  // jump instructions
  // Equal
  def JREQk : Pseudo< (outs),
                      (ins relbrtarget_7 : $target),
                      "jr\teq, $target",
                      [(GameBoybrcond bb : $target, GameBoy_COND_EQ)]>;

  // Not equal
  def JRNEk : Pseudo< (outs),
                      (ins relbrtarget_7 : $target),
                      "jr\tne, $target", 
                      [(GameBoybrcond bb : $target, GameBoy_COND_NE)]>;

  // Greater than
  def JRGTEk : Pseudo<(outs),
                      (ins relbrtarget_7 : $target),
                      "jr\tgt, $target", 
                      [(GameBoybrcond bb : $target, GameBoy_COND_GE)]>;

  // Less than
  def JRLTk : Pseudo< (outs),
                      (ins relbrtarget_7 : $target),
                      "jr\tlt, $target", 
                      [(GameBoybrcond bb : $target, GameBoy_COND_LT)]>;
}

//===----------------------------------------------------------------------===//
// Call and return instructions
//===----------------------------------------------------------------------===//

// CALL Imm16
// CALL cc, Imm16

let isTerminator = 1, isReturn = 1, isBarrier = 1 in {
// RET
  def RET : GBF0< 0b11001001,
                  (outs), (ins), "ret",
                  [(GameBoyretflag)]>;
// RETI
// RET cc
// GameBoyISD::RETCC_FLAG has been added.
}

// RST t

//===----------------------------------------------------------------------===//
// General purpose arithmetic operations and CPU control instructions
//===----------------------------------------------------------------------===//

// Creating SEXT and ZEXT instructions as these are probably needed at the moment.
// The best way of conducting a SEXT is to load the lower byte of a register pair
// with the value,

// SEXT and ZEXT are needed to extend out values from 8-bit to 16-bit registers.

// ZEXT is as simple as storing the low byte, and then setting the high byte to 0.
def ZEXT : Pseudo<(outs GPRPair : $dst),
                  (ins GPR : $src),
                  "zext\t$dst, $src",
                  [(set i16 : $dst, (zext i8 : $src))]>;

// SEXT is more complex; the lower byte is stored as always, but we use add and sbc
// to push the sign into the carry. We then use the arithmetic R - R - C = -C to store
// the upper bit in the upper byte.
let Uses = [RA] in {
def SEXT : Pseudo<(outs GPRPair : $dst),
                  (ins GPR : $src),
                  "sext\t$dst, $src",
                  [(set i16 : $dst, (sext i8 : $src))]>;
}
// DAA A
// CPL A
// NOP
// CCF
// SCF
// DI
// EI
// HALT
// STOP