add_llvm_component_group(GameBoy)

set(LLVM_TARGET_DEFINITIONS GameBoy.td)

tablegen(LLVM GameBoyGenAsmMatcher.inc -gen-asm-matcher)
tablegen(LLVM GameBoyGenAsmWriter.inc -gen-asm-writer)
tablegen(LLVM GameBoyGenCallingConv.inc -gen-callingconv)
tablegen(LLVM GameBoyGenDAGISel.inc -gen-dag-isel)
tablegen(LLVM GameBoyGenDisassemblerTables.inc -gen-disassembler)
tablegen(LLVM GameBoyGenInstrInfo.inc -gen-instr-info)
tablegen(LLVM GameBoyGenMCCodeEmitter.inc -gen-emitter)
tablegen(LLVM GameBoyGenRegisterInfo.inc -gen-register-info)
tablegen(LLVM GameBoyGenSubtargetInfo.inc -gen-subtarget)

add_public_tablegen_target(GameBoyCommonTableGen)

add_llvm_target(GameBoyCodeGen
  GameBoyAsmPrinter.cpp
  GameBoyExpandPseudoInsts.cpp
  GameBoyFrameLowering.cpp
  GameBoyInstrInfo.cpp
  GameBoyISelDAGToDAG.cpp
  GameBoyISelLowering.cpp
  GameBoyMCInstLower.cpp
  GameBoyRegisterInfo.cpp
  GameBoyShiftExpand.cpp
  GameBoySubtarget.cpp
  GameBoyTargetMachine.cpp
  GameBoyTargetObjectFile.cpp

  DEPENDS
  intrinsics_gen

  LINK_COMPONENTS
  AsmPrinter
  CodeGen
  Core
  MC
  GameBoyDesc
  GameBoyInfo
  SelectionDAG
  Support
  Target

  ADD_TO_COMPONENT
  GameBoy
)

add_subdirectory(AsmParser)
add_subdirectory(Disassembler)
add_subdirectory(MCTargetDesc)
add_subdirectory(TargetInfo)
