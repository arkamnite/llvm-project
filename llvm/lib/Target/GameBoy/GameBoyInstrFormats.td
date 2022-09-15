// This format specifies the encoding used by instructions.
// Used by the code emitter to determine which form of "fixup"
// to use; these are kept in sync with GameBoyBaseInfo.h

// A generic GameBoy instruction is 8 bits long.
class GameBoyInstr<dag outs, dag ins, string asmstr, list<dag> pattern> : Instruction {
    
    // Instructions are 8-bits long.
    field bits<8> Inst;
    let Size = 1;
    let Namespace = "GameBoy";

    dag OutOperandList = outs;
    dag InOperandList = ins;
    let AsmString = asmstr;
    let Pattern = pattern;

    field bits<8> SoftFail = 0;
}

// There exist 16-bit GameBoy instructions.
// 
class GameBoyInstr16<dag outs, dag ins, string asmstr, list<dag> pattern> :  Instruction {
    let Namespace = "GameBoy";

    // Instructions are 16-bits long.
    field bits<16> Inst;
    let Size = 2;

    dag OutOperandList = outs;
    dag InOperandList = ins;
    let AsmString = asmstr;
    let Pattern = pattern;
}

//=== Register codes ===//
// A : 111
// B : 000
// C : 001
// D : 101
// E : 011
// H : 100
// L : 101


//===---------------------------------------------------------------===//
// GameBoy instruction formats are derived from the types of operands
// each instruction takes.
//===---------------------------------------------------------------===//

// Format 0: No operands used.
class FZ<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
: GameBoyInstr<outs, ins, asmstr, pattern>
{ 
    let Inst = opcode;
}

// Cycle operand instruction
// xx - xCC - xxx

//===--------------------------------------------------------------===//
// Register instruction: <| xx - rrr - xxx |>
//===--------------------------------------------------------------===//
class FR<bits<2> op1, bits<3> op2, dag outs, dag ins, string asmstr, list<dag> pattern> 
: GameBoyInstr<outs, ins, asmstr, pattern> 
{
    bits<3> register;
    // let op =        0; // Operation values not used by GB.
    let Inst{7 - 6} = op1;
    let Inst{5 - 3} = register;
    let Inst{2 - 0} = op2;
}

//===--------------------------------------------------------------===//
// Register operand instruction 2 (arithmetic and logic instructions)
// xx - xxx - rrr
//===--------------------------------------------------------------===//

//===--------------------------------------------------------------===//
// Register pair operand instruction
// xx - rrx - xxx
//===--------------------------------------------------------------===//

//===--------------------------------------------------------------===//
// Register-register operand instruction
// xx - rrr - rrr
//===--------------------------------------------------------------===//
class FRdRr<bits<2> op, dag outs, dag ins, string asmstr, list<dag> pattern>
: GameBoyInstr<outs, ins, asmstr, pattern>
{
    bits<3> rd;
    bits<3> rr;

    let Inst{7 - 6} = op;
    let Inst{5 - 3} = rd;
    let Inst{2 - 0} = rr;
}

//===--------------------------------------------------------------===//
// Immediate 8-bit data instruction
// xx - xxx - xxx
// n8
//===--------------------------------------------------------------===//

//===--------------------------------------------------------------===//
// Immediate 8-bit data, with register, instruction
// xx - rrr - xxx
// n8
//===--------------------------------------------------------------===//

//===--------------------------------------------------------------===//
// Immediate 16-bit data instruction
// xx - xxx - xxx
// n8
// n8
//===--------------------------------------------------------------===//

//===--------------------------------------------------------------===//
// Immediate 16-bit data instruction, with register pair
// xx - ddx - xxx
// n8
// n8
//===--------------------------------------------------------------===//

//===--------------------------------------------------------------===//
// Immediate 16-bit instruction, with cycle
// xx - xcc - xxx
// n8
// n8
//===--------------------------------------------------------------===//

//===---------------------------------------------------------------===//
// Rotate shift instructions follow a new type of format
//===---------------------------------------------------------------===//

//===--------------------------------------------------------------===//
// Rotate register
// xx - xxx - xxx
// xx - xxx - rrr
//===--------------------------------------------------------------===//

//===--------------------------------------------------------------===//
// Rotate register pair
// xx - xxx - xxx
// xx - xxx - xxx
//===--------------------------------------------------------------===//

//===---------------------------------------------------------------===//
// Bit operation instructions also follow a different format.
//===---------------------------------------------------------------===//

//===--------------------------------------------------------------===//
// Bit-register instruction
// xx - xxx - xxx
// xx - bbb - rrr
//===--------------------------------------------------------------===//

//===--------------------------------------------------------------===//
// Bit instruction
// xx - xxx - xxx
// xx - bbb - xxx
//===--------------------------------------------------------------===//