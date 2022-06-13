// GameBoyRegisterInfo.td - Game Boy Register definitions

// any type of register.
class GBReg<string n> : Register<n> {
    let Namespace = "GB";
}

class GBCtrlReg<string n> : Register<n> {
    let Namespace = "GB";
}

class Reg<string n> : Register<n>;

// A pair of registers
class RegPair<string n, list<Register> subregs> : GBReg<n> {
    let SubRegs = subregs;
}

// Control register.
def Flags : GBCtrlReg<"Flags">;

// Individual 8-bit registers.
def A : Reg<"A">, DwarfRegNum<[0]>;
def B : Reg<"B">, DwarfRegNum<[1]>;
def C : Reg<"C">, DwarfRegNum<[2]>;
def D : Reg<"D">, DwarfRegNum<[3]>;
def E : Reg<"E">, DwarfRegNum<[4]>;
def H : Reg<"H">, DwarfRegNum<[5]>;
def L : Reg<"L">, DwarfRegNum<[6]>;

// Register Pair aliases
def BC : RegPair<"BC", [A, B]>, DwarfRegNum<[10]>;
def DE : RegPair<"DE", [D, E]>, DwarfRegNum<[11]>;
def HL : RegPair<"HL", [H, L]>, DwarfRegNum<[12]>;

// Register classes
// There is only one type of general use register
def I8Regs : RegisterClass<"GB", [i8], 8, (add A, B, C, D, E, H, L)>;
def I8Pair : RegisterClass<"GB", [v2i8], 16, (add GC, DE, HL)>;