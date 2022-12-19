//===-- GameBoyCallingConv.td - Calling Conventions for GameBoy ----*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
// This describes the calling conventions for GameBoy architecture.
// Normal functions use a special calling convention, solved in code.
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// GameBoy Return Value Calling Convention
//===----------------------------------------------------------------------===//

// Special return value calling convention for runtime functions.
def RetCC_GameBoy_BUILTIN : CallingConv<[
  // CCIfType<[i8], CCAssignToReg<[R24, R25]>>,
  // CCIfType<[i16], CCAssignToReg<[R23R22, R25R24]>>
  CCIfType<[i8], CCAssignToReg<[RA, RE]>>,
  CCIfType<[i16], CCAssignToReg<[RBRC, RDRE]>>,
  // CCIfType<[i32], CCAssignToReg<[RDRERBRC]>>
]>;

//===----------------------------------------------------------------------===//
// GameBoy Argument Calling Conventions
//===----------------------------------------------------------------------===//

// The calling conventions are implemented in custom C++ code. The Game Boy calling
// convention for passing arguments is as follows:
// let Entry = 1 in {
def ArgCC_GameBoy_Default : CallingConv<[
  // CCIfType<[i8], CCAssignToReg<[RA, RB, RC]>>,
  // CCIfType<[i16], CCAssignToReg<[RBRC]>>,
  CCIfType<[i8], CCAssignToReg<[RA, RE]>>,
  // CCIfType<[i8], CCAssignToStack<1, 1>>,
  CCIfType<[i16], CCAssignToReg<[RBRC, RDRE]>>,
  // CCAssignToStack<2, 2>
]>;
// }

// Calling convention for variadic functions.
def ArgCC_GameBoy_Vararg : CallingConv<[
  // i8 are always passed through the stack with a byte slot and byte alignment.
  CCIfType<[i8], CCAssignToStack<1, 1>>,
  // i16 are always passed through the stack with an alignment of 1.
  CCAssignToStack<2, 1>
]>;

//===----------------------------------------------------------------------===//
// Callee-saved register lists.
//===----------------------------------------------------------------------===//

def CSR_Normal : CalleeSavedRegs<(add R29, R28, (sequence "R%u", 17, 2))>;
def CSR_NormalTiny : CalleeSavedRegs<(add R29, R28, R19, R18)>;
def CSR_Interrupts : CalleeSavedRegs<(add(sequence "R%u", 31, 2))>;
def CSR_InterruptsTiny : CalleeSavedRegs<(add(sequence "R%u", 31, 18))>;

// These are the call-saved registers.
// def CSR : CalleeSavedRegs<(add RBRC, RARF)>;
def CSR : CalleeSavedRegs<(add RHRL)>;

// In SDCC Version 0, RA, RB, RC are used for parameters and return values.
def CSR_SDCC_V0 : CalleeSavedRegs<(add RBRC, RHRL)>;
// In SDCC Version 1, return values are passed via A, BC, DE, and HL is unused. 
def CSR_SDCC_V1 : CalleeSavedRegs<(add RBRC, RHRL)>;
