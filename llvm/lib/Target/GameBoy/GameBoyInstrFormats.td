// This format specifies the encoding used by instructions.
// Used by the code emitter to determine which form of "fixup"
// to use; these are kept in sync with GameBoyBaseInfo.h

class GBInstr<dag outs, dag ins, string asmstr, list<dag> pattern, InstrItinClass itin = NoItinerary> : Instruction {
    
    // Instructions are 8-bits long.
    field bits<8> Inst;

    let Namespace = "GB";
    // All instructions are 8 bits long.
    let Size = 8;

    dag OutOperandList = outs;
    dag InOperandList = ins;
    let AsmString = asmstr;
    let Pattern = pattern;

    let DecoderNamespace = "GameBoy";
    field bits<8> SoftFail = 0;

    let Itinerary = itin;
}

//===---------------------------------------------------------------===//
// GameBoy instruction formats are derived from the types of operands
// each instruction takes.
//===---------------------------------------------------------------===//

// No operand
// xx - xxx - xxx

// Cycle operand instruction
// xx - xCC - xxx

// Register operand instruction
// xx - rrr - xxx

// Register operand instruction 2 (arithmetic and logic instructions)
// xx - xxx - rrr

// Register pair operand instruction
// xx - rrx - xxx

// Register-register operand instruction
// xx - rrr - rrr

// Immediate 8-bit data instruction
// xx - xxx - xxx
// n8

// Immediate 8-bit data, with register, instruction
// xx - rrr - xxx
// n8

// Immediate 16-bit data instruction
// xx - xxx - xxx
// n8
// n8

// Immediate 16-bit data instruction, with register pair
// xx - ddx - xxx
// n8
// n8

// Immediate 16-bit instruction, with cycle
// xx - xcc - xxx
// n8
// n8

//===---------------------------------------------------------------===//
// Rotate shift instructions follow a new type of format
//===---------------------------------------------------------------===//

// Rotate register
// xx - xxx - xxx
// xx - xxx - rrr

// Rotate register pair
// xx - xxx - xxx
// xx - xxx - xxx

//===---------------------------------------------------------------===//
// Bit operation instructions also follow a different format.
//===---------------------------------------------------------------===//

// Bit-register instruction
// xx - xxx - xxx
// xx - bbb - rrr

// Bit instruction
// xx - xxx - xxx
// xx - bbb - xxx

