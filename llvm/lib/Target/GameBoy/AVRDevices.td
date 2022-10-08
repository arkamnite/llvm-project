//===---------------------------------------------------------------------===//
// GameBoy Device Definitions
//===---------------------------------------------------------------------===//

// :TODO: Implement the skip errata, see `gcc/config/GameBoy/GameBoy-arch.h` for details
// :TODO: We define all devices with SRAM to have all variants of LD/ST/LDD/STD.
//        In reality, GameBoy1 (no SRAM) has one variant each of `LD` and `ST`.
//        GameBoy2 (with SRAM) adds the rest of the variants.

// A feature set aggregates features, grouping them. We don't want to create a
// new member in GameBoySubtarget (to store a value) for each set because we do not
// care if the set is supported, only the subfeatures inside the set. We fix
// this by simply setting the same dummy member for all feature sets, which is
// then ignored.
class FeatureSet<string name, string desc, list<SubtargetFeature> i>
    : SubtargetFeature<name, "m_FeatureSetDummy", "true", desc, i>;

// A family of microcontrollers, defining a set of supported features.
class Family<string name, list<SubtargetFeature> i>
    : FeatureSet<
          name, !strconcat("The device is a part of the ", name, " family"), i>;

// The device has SRAM, and supports the bare minimum of
// SRAM-relevant instructions.
//
// These are:
// LD - all 9 variants
// ST - all 9 variants
// LDD - two variants for Y and Z
// STD - two variants for Y and Z
// `LDS Rd, K`
// `STS k, Rr`
// `PUSH`/`POP`
def FeatureSRAM : SubtargetFeature<"sram", "m_hasSRAM", "true",
                                   "The device has random access memory">;

// The device supports the `JMP k` and `CALL k` instructions.
def FeatureJMPCALL : SubtargetFeature<"jmpcall", "m_hasJMPCALL", "true",
                                      "The device supports the `JMP` and "
                                      "`CALL` instructions">;

// The device supports the indirect branches `IJMP` and `ICALL`.
def FeatureIJMPCALL : SubtargetFeature<"ijmpcall", "m_hasIJMPCALL", "true",
                                       "The device supports `IJMP`/`ICALL`"
                                       "instructions">;

// The device supports the extended indirect branches `EIJMP` and `EICALL`.
def FeatureEIJMPCALL : SubtargetFeature<"eijmpcall", "m_hasEIJMPCALL", "true",
                                        "The device supports the "
                                        "`EIJMP`/`EICALL` instructions">;

// The device supports `ADDI Rd, K`, `SUBI Rd, K`.
def FeatureADDSUBIW : SubtargetFeature<"addsubiw", "m_hasADDSUBIW", "true",
                                       "Enable 16-bit register-immediate "
                                       "addition and subtraction instructions">;

// The device has an 8-bit stack pointer (SP) register.
def FeatureSmallStack
    : SubtargetFeature<"smallstack", "m_hasSmallStack", "true",
                       "The device has an 8-bit "
                       "stack pointer">;

// The device supports the 16-bit GPR pair MOVW instruction.
def FeatureMOVW : SubtargetFeature<"movw", "m_hasMOVW", "true",
                                   "The device supports the 16-bit MOVW "
                                   "instruction">;

// The device supports the `LPM` instruction, with implied destination being r0.
def FeatureLPM : SubtargetFeature<"lpm", "m_hasLPM", "true",
                                  "The device supports the `LPM` instruction">;

// The device supports the `LPM Rd, Z[+] instruction.
def FeatureLPMX : SubtargetFeature<"lpmx", "m_hasLPMX", "true",
                                   "The device supports the `LPM Rd, Z[+]` "
                                   "instruction">;

// The device supports the `ELPM` instruction.
def FeatureELPM : SubtargetFeature<"elpm", "m_hasELPM", "true",
                                   "The device supports the ELPM instruction">;

// The device supports the `ELPM Rd, Z[+]` instructions.
def FeatureELPMX : SubtargetFeature<"elpmx", "m_hasELPMX", "true",
                                    "The device supports the `ELPM Rd, Z[+]` "
                                    "instructions">;

// The device supports the `SPM` instruction.
def FeatureSPM : SubtargetFeature<"spm", "m_hasSPM", "true",
                                  "The device supports the `SPM` instruction">;

// The device supports the `SPM Z+` instruction.
def FeatureSPMX : SubtargetFeature<"spmx", "m_hasSPMX", "true",
                                   "The device supports the `SPM Z+` "
                                   "instruction">;

// The device supports the `DES k` instruction.
def FeatureDES : SubtargetFeature<"des", "m_hasDES", "true",
                                  "The device supports the `DES k` encryption "
                                  "instruction">;

// The device supports the Read-Write-Modify instructions
// XCH, LAS, LAC, and LAT.
def FeatureRMW : SubtargetFeature<"rmw", "m_supportsRMW", "true",
                                  "The device supports the read-write-modify "
                                  "instructions: XCH, LAS, LAC, LAT">;

// The device supports the `[F]MUL[S][U]` family of instructions.
def FeatureMultiplication
    : SubtargetFeature<"mul", "m_supportsMultiplication", "true",
                       "The device supports the "
                       "multiplication instructions">;

// The device supports the `BREAK` instruction.
def FeatureBREAK : SubtargetFeature<"break", "m_hasBREAK", "true",
                                    "The device supports the `BREAK` debugging "
                                    "instruction">;

// The device has instruction encodings specific to the Tiny core.
def FeatureTinyEncoding
    : SubtargetFeature<"tinyencoding", "m_hasTinyEncoding", "true",
                       "The device has Tiny core specific "
                       "instruction encodings">;

// The device has CPU registers mapped in data address space
def FeatureMMR : SubtargetFeature<"memmappedregs", "m_hasMemMappedGPR", "true",
                                  "The device has CPU registers "
                                  "mapped in data address space">;

class ELFArch<string name>
    : SubtargetFeature<"", "ELFArch", !strconcat("ELF::", name), "">;

// ELF e_flags architecture values
def ELFArchGameBoy1 : ELFArch<"EF_GameBoy_ARCH_GameBoy1">;
def ELFArchGameBoy2 : ELFArch<"EF_GameBoy_ARCH_GameBoy2">;
def ELFArchGameBoy25 : ELFArch<"EF_GameBoy_ARCH_GameBoy25">;
def ELFArchGameBoy3 : ELFArch<"EF_GameBoy_ARCH_GameBoy3">;
def ELFArchGameBoy31 : ELFArch<"EF_GameBoy_ARCH_GameBoy31">;
def ELFArchGameBoy35 : ELFArch<"EF_GameBoy_ARCH_GameBoy35">;
def ELFArchGameBoy4 : ELFArch<"EF_GameBoy_ARCH_GameBoy4">;
def ELFArchGameBoy5 : ELFArch<"EF_GameBoy_ARCH_GameBoy5">;
def ELFArchGameBoy51 : ELFArch<"EF_GameBoy_ARCH_GameBoy51">;
def ELFArchGameBoy6 : ELFArch<"EF_GameBoy_ARCH_GameBoy6">;
def ELFArchTiny : ELFArch<"EF_GameBoy_ARCH_GameBoyTINY">;
def ELFArchXMEGA1 : ELFArch<"EF_GameBoy_ARCH_XMEGA1">;
def ELFArchXMEGA2 : ELFArch<"EF_GameBoy_ARCH_XMEGA2">;
def ELFArchXMEGA3 : ELFArch<"EF_GameBoy_ARCH_XMEGA3">;
def ELFArchXMEGA4 : ELFArch<"EF_GameBoy_ARCH_XMEGA4">;
def ELFArchXMEGA5 : ELFArch<"EF_GameBoy_ARCH_XMEGA5">;
def ELFArchXMEGA6 : ELFArch<"EF_GameBoy_ARCH_XMEGA6">;
def ELFArchXMEGA7 : ELFArch<"EF_GameBoy_ARCH_XMEGA7">;

//===---------------------------------------------------------------------===//
// GameBoy Families
//===---------------------------------------------------------------------===//

// The device has at least the bare minimum that **every** single GameBoy
// device should have.
def FamilyGameBoy0 : Family<"GameBoy0", []>;

def FamilyGameBoy1 : Family<"GameBoy1", [FamilyGameBoy0, FeatureLPM, FeatureMMR]>;

def FamilyGameBoy2
    : Family<"GameBoy2",
             [FamilyGameBoy1, FeatureIJMPCALL, FeatureADDSUBIW, FeatureSRAM]>;

def FamilyGameBoy25
    : Family<"GameBoy25",
             [FamilyGameBoy2, FeatureMOVW, FeatureLPMX, FeatureSPM, FeatureBREAK]>;

def FamilyGameBoy3 : Family<"GameBoy3", [FamilyGameBoy2, FeatureJMPCALL]>;

def FamilyGameBoy31 : Family<"GameBoy31", [FamilyGameBoy3, FeatureELPM]>;

def FamilyGameBoy35
    : Family<"GameBoy35",
             [FamilyGameBoy3, FeatureMOVW, FeatureLPMX, FeatureSPM, FeatureBREAK]>;

def FamilyGameBoy4 : Family<"GameBoy4",
                        [FamilyGameBoy2, FeatureMultiplication, FeatureMOVW,
                         FeatureLPMX, FeatureSPM, FeatureBREAK]>;

def FamilyGameBoy5 : Family<"GameBoy5",
                        [FamilyGameBoy3, FeatureMultiplication, FeatureMOVW,
                         FeatureLPMX, FeatureSPM, FeatureBREAK]>;

def FamilyGameBoy51 : Family<"GameBoy51", [FamilyGameBoy5, FeatureELPM, FeatureELPMX]>;

def FamilyGameBoy6 : Family<"GameBoy6", [FamilyGameBoy51]>;

def FamilyTiny
    : Family<"GameBoytiny",
             [FamilyGameBoy0, FeatureBREAK, FeatureSRAM, FeatureTinyEncoding,
              FeatureSmallStack]>;

def FamilyXMEGA3 : Family<"xmega3",
                          [FamilyGameBoy0, FeatureLPM, FeatureIJMPCALL,
                           FeatureADDSUBIW, FeatureSRAM, FeatureJMPCALL,
                           FeatureMultiplication, FeatureMOVW, FeatureLPMX,
                           FeatureBREAK]>;

def FamilyXMEGA : Family<"xmega",
                         [FamilyGameBoy0, FeatureLPM, FeatureIJMPCALL,
                          FeatureADDSUBIW, FeatureSRAM, FeatureJMPCALL,
                          FeatureMultiplication, FeatureMOVW, FeatureLPMX,
                          FeatureSPM, FeatureBREAK, FeatureEIJMPCALL,
                          FeatureSPMX, FeatureDES, FeatureELPM, FeatureELPMX]>;

def FamilyXMEGAU : Family<"xmegau", [FamilyXMEGA, FeatureRMW]>;

def FeatureSetSpecial
    : FeatureSet<"special",
                 "Enable use of the entire instruction "
                 "set - used for debugging",
                 [
                   FeatureSRAM, FeatureJMPCALL, FeatureIJMPCALL,
                   FeatureEIJMPCALL, FeatureADDSUBIW, FeatureMOVW, FeatureLPM,
                   FeatureLPMX, FeatureELPM, FeatureELPMX, FeatureSPM,
                   FeatureSPMX, FeatureDES, FeatureRMW, FeatureMultiplication,
                   FeatureBREAK, FeatureMMR
                 ]>;

//===---------------------------------------------------------------------===//
// GameBoy microcontrollers supported.
//===---------------------------------------------------------------------===//

class Device<string Name, Family Fam, ELFArch Arch,
             list<SubtargetFeature> ExtraFeatures = []>
    : Processor<Name, NoItineraries, !listconcat([Fam, Arch], ExtraFeatures)>;

// Generic MCUs
// Note that several versions of GCC has strange ELF architecture
// settings for backwards compatibility - see `gas/config/tc-GameBoy.c`
// in GameBoy binutils. We do not replicate this.
def : Device<"GameBoy1", FamilyGameBoy1, ELFArchGameBoy1>;
def : Device<"GameBoy2", FamilyGameBoy2, ELFArchGameBoy2>;
def : Device<"GameBoy25", FamilyGameBoy25, ELFArchGameBoy25>;
def : Device<"GameBoy3", FamilyGameBoy3, ELFArchGameBoy3>;
def : Device<"GameBoy31", FamilyGameBoy31, ELFArchGameBoy31>;
def : Device<"GameBoy35", FamilyGameBoy35, ELFArchGameBoy35>;
def : Device<"GameBoy4", FamilyGameBoy4, ELFArchGameBoy4>;
def : Device<"GameBoy5", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"GameBoy51", FamilyGameBoy51, ELFArchGameBoy51>;
def : Device<"GameBoy6", FamilyGameBoy6, ELFArchGameBoy6>;
def : Device<"GameBoyxmega1", FamilyXMEGA, ELFArchXMEGA1>;
def : Device<"GameBoyxmega2", FamilyXMEGA, ELFArchXMEGA2>;
def : Device<"GameBoyxmega3", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"GameBoyxmega4", FamilyXMEGA, ELFArchXMEGA4>;
def : Device<"GameBoyxmega5", FamilyXMEGA, ELFArchXMEGA5>;
def : Device<"GameBoyxmega6", FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"GameBoyxmega7", FamilyXMEGA, ELFArchXMEGA7>;
def : Device<"GameBoytiny", FamilyTiny, ELFArchTiny>;

// Specific MCUs
// NOTE: This list has been synchronized with gcc-GameBoy 5.4.0 and GameBoy-libc 2.0.0.
def : Device<"at90s1200", FamilyGameBoy0, ELFArchGameBoy1, [FeatureSmallStack]>;
def : Device<"attiny11", FamilyGameBoy1, ELFArchGameBoy1, [FeatureSmallStack]>;
def : Device<"attiny12", FamilyGameBoy1, ELFArchGameBoy1, [FeatureSmallStack]>;
def : Device<"attiny15", FamilyGameBoy1, ELFArchGameBoy1, [FeatureSmallStack]>;
def : Device<"attiny28", FamilyGameBoy1, ELFArchGameBoy1, [FeatureSmallStack]>;
def : Device<"at90s2313", FamilyGameBoy2, ELFArchGameBoy2, [FeatureSmallStack]>;
def : Device<"at90s2323", FamilyGameBoy2, ELFArchGameBoy2, [FeatureSmallStack]>;
def : Device<"at90s2333", FamilyGameBoy2, ELFArchGameBoy2, [FeatureSmallStack]>;
def : Device<"at90s2343", FamilyGameBoy2, ELFArchGameBoy2, [FeatureSmallStack]>;
def : Device<"attiny22", FamilyGameBoy2, ELFArchGameBoy2, [FeatureSmallStack]>;
def : Device<"attiny26", FamilyGameBoy2, ELFArchGameBoy2,
             [FeatureLPMX, FeatureSmallStack]>;
def : Device<"at86rf401", FamilyGameBoy2, ELFArchGameBoy25, [FeatureMOVW, FeatureLPMX]>;
def : Device<"at90s4414", FamilyGameBoy2, ELFArchGameBoy2, [FeatureSmallStack]>;
def : Device<"at90s4433", FamilyGameBoy2, ELFArchGameBoy2, [FeatureSmallStack]>;
def : Device<"at90s4434", FamilyGameBoy2, ELFArchGameBoy2, [FeatureSmallStack]>;
def : Device<"at90s8515", FamilyGameBoy2, ELFArchGameBoy2>;
def : Device<"at90c8534", FamilyGameBoy2, ELFArchGameBoy2>;
def : Device<"at90s8535", FamilyGameBoy2, ELFArchGameBoy2>;
def : Device<"ata5272", FamilyGameBoy25, ELFArchGameBoy25>;
def : Device<"ata6616c", FamilyGameBoy25, ELFArchGameBoy25>;
def : Device<"attiny13", FamilyGameBoy25, ELFArchGameBoy25, [FeatureSmallStack]>;
def : Device<"attiny13a", FamilyGameBoy25, ELFArchGameBoy25, [FeatureSmallStack]>;
def : Device<"attiny2313", FamilyGameBoy25, ELFArchGameBoy25, [FeatureSmallStack]>;
def : Device<"attiny2313a", FamilyGameBoy25, ELFArchGameBoy25, [FeatureSmallStack]>;
def : Device<"attiny24", FamilyGameBoy25, ELFArchGameBoy25, [FeatureSmallStack]>;
def : Device<"attiny24a", FamilyGameBoy25, ELFArchGameBoy25, [FeatureSmallStack]>;
def : Device<"attiny4313", FamilyGameBoy25, ELFArchGameBoy25>;
def : Device<"attiny44", FamilyGameBoy25, ELFArchGameBoy25>;
def : Device<"attiny44a", FamilyGameBoy25, ELFArchGameBoy25>;
def : Device<"attiny84", FamilyGameBoy25, ELFArchGameBoy25>;
def : Device<"attiny84a", FamilyGameBoy25, ELFArchGameBoy25>;
def : Device<"attiny25", FamilyGameBoy25, ELFArchGameBoy25, [FeatureSmallStack]>;
def : Device<"attiny45", FamilyGameBoy25, ELFArchGameBoy25>;
def : Device<"attiny85", FamilyGameBoy25, ELFArchGameBoy25>;
def : Device<"attiny261", FamilyGameBoy25, ELFArchGameBoy25, [FeatureSmallStack]>;
def : Device<"attiny261a", FamilyGameBoy25, ELFArchGameBoy25, [FeatureSmallStack]>;
def : Device<"attiny441", FamilyGameBoy25, ELFArchGameBoy25>;
def : Device<"attiny461", FamilyGameBoy25, ELFArchGameBoy25>;
def : Device<"attiny461a", FamilyGameBoy25, ELFArchGameBoy25>;
def : Device<"attiny841", FamilyGameBoy25, ELFArchGameBoy25>;
def : Device<"attiny861", FamilyGameBoy25, ELFArchGameBoy25>;
def : Device<"attiny861a", FamilyGameBoy25, ELFArchGameBoy25>;
def : Device<"attiny87", FamilyGameBoy25, ELFArchGameBoy25>;
def : Device<"attiny43u", FamilyGameBoy25, ELFArchGameBoy25>;
def : Device<"attiny48", FamilyGameBoy25, ELFArchGameBoy25>;
def : Device<"attiny88", FamilyGameBoy25, ELFArchGameBoy25>;
def : Device<"attiny828", FamilyGameBoy25, ELFArchGameBoy25>;
def : Device<"at43usb355", FamilyGameBoy3, ELFArchGameBoy3>;
def : Device<"at76c711", FamilyGameBoy3, ELFArchGameBoy3>;
def : Device<"atmega103", FamilyGameBoy31, ELFArchGameBoy31>;
def : Device<"at43usb320", FamilyGameBoy31, ELFArchGameBoy31>;
def : Device<"attiny167", FamilyGameBoy35, ELFArchGameBoy35>;
def : Device<"at90usb82", FamilyGameBoy35, ELFArchGameBoy35>;
def : Device<"at90usb162", FamilyGameBoy35, ELFArchGameBoy35>;
def : Device<"ata5505", FamilyGameBoy35, ELFArchGameBoy35>;
def : Device<"ata6617c", FamilyGameBoy35, ELFArchGameBoy35>;
def : Device<"ata664251", FamilyGameBoy35, ELFArchGameBoy35>;
def : Device<"atmega8u2", FamilyGameBoy35, ELFArchGameBoy35>;
def : Device<"atmega16u2", FamilyGameBoy35, ELFArchGameBoy35>;
def : Device<"atmega32u2", FamilyGameBoy35, ELFArchGameBoy35>;
def : Device<"attiny1634", FamilyGameBoy35, ELFArchGameBoy35>;
def : Device<"atmega8", FamilyGameBoy2, ELFArchGameBoy4,
             [FeatureMultiplication, FeatureMOVW, FeatureLPMX, FeatureSPM]>;
def : Device<"ata6289", FamilyGameBoy4, ELFArchGameBoy4>;
def : Device<"atmega8a", FamilyGameBoy2, ELFArchGameBoy4,
             [FeatureMultiplication, FeatureMOVW, FeatureLPMX, FeatureSPM]>;
def : Device<"ata6285", FamilyGameBoy4, ELFArchGameBoy4>;
def : Device<"ata6286", FamilyGameBoy4, ELFArchGameBoy4>;
def : Device<"ata6612c", FamilyGameBoy4, ELFArchGameBoy4>;
def : Device<"atmega48", FamilyGameBoy4, ELFArchGameBoy4>;
def : Device<"atmega48a", FamilyGameBoy4, ELFArchGameBoy4>;
def : Device<"atmega48pa", FamilyGameBoy4, ELFArchGameBoy4>;
def : Device<"atmega48pb", FamilyGameBoy4, ELFArchGameBoy4>;
def : Device<"atmega48p", FamilyGameBoy4, ELFArchGameBoy4>;
def : Device<"atmega88", FamilyGameBoy4, ELFArchGameBoy4>;
def : Device<"atmega88a", FamilyGameBoy4, ELFArchGameBoy4>;
def : Device<"atmega88p", FamilyGameBoy4, ELFArchGameBoy4>;
def : Device<"atmega88pa", FamilyGameBoy4, ELFArchGameBoy4>;
def : Device<"atmega88pb", FamilyGameBoy4, ELFArchGameBoy4>;
def : Device<"atmega8515", FamilyGameBoy2, ELFArchGameBoy4,
             [FeatureMultiplication, FeatureMOVW, FeatureLPMX, FeatureSPM]>;
def : Device<"atmega8535", FamilyGameBoy2, ELFArchGameBoy4,
             [FeatureMultiplication, FeatureMOVW, FeatureLPMX, FeatureSPM]>;
def : Device<"atmega8hva", FamilyGameBoy4, ELFArchGameBoy4>;
def : Device<"at90pwm1", FamilyGameBoy4, ELFArchGameBoy4>;
def : Device<"at90pwm2", FamilyGameBoy4, ELFArchGameBoy4>;
def : Device<"at90pwm2b", FamilyGameBoy4, ELFArchGameBoy4>;
def : Device<"at90pwm3", FamilyGameBoy4, ELFArchGameBoy4>;
def : Device<"at90pwm3b", FamilyGameBoy4, ELFArchGameBoy4>;
def : Device<"at90pwm81", FamilyGameBoy4, ELFArchGameBoy4>;
def : Device<"ata5702m322", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"ata5782", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"ata5790", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"ata5790n", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"ata5791", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"ata5795", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"ata5831", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"ata6613c", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"ata6614q", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"ata8210", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"ata8510", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega16", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega16a", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega161", FamilyGameBoy3, ELFArchGameBoy5,
             [FeatureMultiplication, FeatureMOVW, FeatureLPMX, FeatureSPM]>;
def : Device<"atmega162", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega163", FamilyGameBoy3, ELFArchGameBoy5,
             [FeatureMultiplication, FeatureMOVW, FeatureLPMX, FeatureSPM]>;
def : Device<"atmega164a", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega164p", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega164pa", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega165", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega165a", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega165p", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega165pa", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega168", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega168a", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega168p", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega168pa", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega168pb", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega169", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega169a", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega169p", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega169pa", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega32", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega32a", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega323", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega324a", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega324p", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega324pa", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega324pb", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega325", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega325a", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega325p", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega325pa", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega3250", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega3250a", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega3250p", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega3250pa", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega328", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega328p", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega328pb", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega329", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega329a", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega329p", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega329pa", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega3290", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega3290a", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega3290p", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega3290pa", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega406", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega64", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega64a", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega640", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega644", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega644a", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega644p", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega644pa", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega645", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega645a", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega645p", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega649", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega649a", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega649p", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega6450", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega6450a", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega6450p", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega6490", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega6490a", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega6490p", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega64rfr2", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega644rfr2", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega16hva", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega16hva2", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega16hvb", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega16hvbrevb", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega32hvb", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega32hvbrevb", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega64hve", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega64hve2", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"at90can32", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"at90can64", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"at90pwm161", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"at90pwm216", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"at90pwm316", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega32c1", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega64c1", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega16m1", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega32m1", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega64m1", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega16u4", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega32u4", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega32u6", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"at90usb646", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"at90usb647", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"at90scr100", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"at94k", FamilyGameBoy3, ELFArchGameBoy5,
             [FeatureMultiplication, FeatureMOVW, FeatureLPMX]>;
def : Device<"m3000", FamilyGameBoy5, ELFArchGameBoy5>;
def : Device<"atmega128", FamilyGameBoy51, ELFArchGameBoy51>;
def : Device<"atmega128a", FamilyGameBoy51, ELFArchGameBoy51>;
def : Device<"atmega1280", FamilyGameBoy51, ELFArchGameBoy51>;
def : Device<"atmega1281", FamilyGameBoy51, ELFArchGameBoy51>;
def : Device<"atmega1284", FamilyGameBoy51, ELFArchGameBoy51>;
def : Device<"atmega1284p", FamilyGameBoy51, ELFArchGameBoy51>;
def : Device<"atmega128rfa1", FamilyGameBoy51, ELFArchGameBoy51>;
def : Device<"atmega128rfr2", FamilyGameBoy51, ELFArchGameBoy51>;
def : Device<"atmega1284rfr2", FamilyGameBoy51, ELFArchGameBoy51>;
def : Device<"at90can128", FamilyGameBoy51, ELFArchGameBoy51>;
def : Device<"at90usb1286", FamilyGameBoy51, ELFArchGameBoy51>;
def : Device<"at90usb1287", FamilyGameBoy51, ELFArchGameBoy51>;
def : Device<"atmega2560", FamilyGameBoy6, ELFArchGameBoy6>;
def : Device<"atmega2561", FamilyGameBoy6, ELFArchGameBoy6>;
def : Device<"atmega256rfr2", FamilyGameBoy6, ELFArchGameBoy6>;
def : Device<"atmega2564rfr2", FamilyGameBoy6, ELFArchGameBoy6>;
def : Device<"atxmega16a4", FamilyXMEGA, ELFArchXMEGA2>;
def : Device<"atxmega16a4u", FamilyXMEGAU, ELFArchXMEGA2>;
def : Device<"atxmega16c4", FamilyXMEGAU, ELFArchXMEGA2>;
def : Device<"atxmega16d4", FamilyXMEGA, ELFArchXMEGA2>;
def : Device<"atxmega32a4", FamilyXMEGA, ELFArchXMEGA2>;
def : Device<"atxmega32a4u", FamilyXMEGAU, ELFArchXMEGA2>;
def : Device<"atxmega32c3", FamilyXMEGAU, ELFArchXMEGA2>;
def : Device<"atxmega32c4", FamilyXMEGAU, ELFArchXMEGA2>;
def : Device<"atxmega32d3", FamilyXMEGA, ELFArchXMEGA2>;
def : Device<"atxmega32d4", FamilyXMEGA, ELFArchXMEGA2>;
def : Device<"atxmega32e5", FamilyXMEGAU, ELFArchXMEGA2>;
def : Device<"atxmega16e5", FamilyXMEGAU, ELFArchXMEGA2>;
def : Device<"atxmega8e5", FamilyXMEGAU, ELFArchXMEGA2>;
def : Device<"atxmega64a3", FamilyXMEGA, ELFArchXMEGA4>;
def : Device<"atxmega64a3u", FamilyXMEGAU, ELFArchXMEGA4>;
def : Device<"atxmega64a4u", FamilyXMEGAU, ELFArchXMEGA4>;
def : Device<"atxmega64b1", FamilyXMEGAU, ELFArchXMEGA4>;
def : Device<"atxmega64b3", FamilyXMEGAU, ELFArchXMEGA4>;
def : Device<"atxmega64c3", FamilyXMEGAU, ELFArchXMEGA4>;
def : Device<"atxmega64d3", FamilyXMEGA, ELFArchXMEGA4>;
def : Device<"atxmega64d4", FamilyXMEGA, ELFArchXMEGA4>;
def : Device<"atxmega64a1", FamilyXMEGA, ELFArchXMEGA5>;
def : Device<"atxmega64a1u", FamilyXMEGAU, ELFArchXMEGA5>;
def : Device<"atxmega128a3", FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega128a3u", FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega128b1", FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega128b3", FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega128c3", FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega128d3", FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega128d4", FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega192a3", FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega192a3u", FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega192c3", FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega192d3", FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega256a3", FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega256a3u", FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega256a3b", FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega256a3bu", FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega256c3", FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega256d3", FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega384c3", FamilyXMEGAU, ELFArchXMEGA6>;
def : Device<"atxmega384d3", FamilyXMEGA, ELFArchXMEGA6>;
def : Device<"atxmega128a1", FamilyXMEGA, ELFArchXMEGA7>;
def : Device<"atxmega128a1u", FamilyXMEGAU, ELFArchXMEGA7>;
def : Device<"atxmega128a4u", FamilyXMEGAU, ELFArchXMEGA7>;
def : Device<"attiny4", FamilyTiny, ELFArchTiny>;
def : Device<"attiny5", FamilyTiny, ELFArchTiny>;
def : Device<"attiny9", FamilyTiny, ELFArchTiny>;
def : Device<"attiny10", FamilyTiny, ELFArchTiny>;
def : Device<"attiny20", FamilyTiny, ELFArchTiny>;
def : Device<"attiny40", FamilyTiny, ELFArchTiny>;
def : Device<"attiny102", FamilyTiny, ELFArchTiny>;
def : Device<"attiny104", FamilyTiny, ELFArchTiny>;
def : Device<"attiny202", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"attiny402", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"attiny204", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"attiny404", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"attiny804", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"attiny1604", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"attiny406", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"attiny806", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"attiny1606", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"attiny807", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"attiny1607", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"attiny212", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"attiny412", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"attiny214", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"attiny414", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"attiny814", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"attiny1614", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"attiny416", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"attiny816", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"attiny1616", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"attiny3216", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"attiny417", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"attiny817", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"attiny1617", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"attiny3217", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"attiny1624", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"attiny1626", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"attiny1627", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"atmega808", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"atmega809", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"atmega1608", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"atmega1609", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"atmega3208", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"atmega3209", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"atmega4808", FamilyXMEGA3, ELFArchXMEGA3>;
def : Device<"atmega4809", FamilyXMEGA3, ELFArchXMEGA3>;
