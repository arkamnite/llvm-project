add_llvm_component_group(GameBoy)

set(LLVM_TARGET_DEFINITIONS GameBoy.td)

tablegen(LLVM GameBoyAsmMatcher.inc -gen-asm-matcher)
tablegen(LLVM GameBoyAsmWriter.inc -gen-asm-writer)
tablegen(LLVM GameBoyCallingConv.inc -gen-callingconv)
tablegen(LLVM GameBoyDAGISel.inc -gen-dag-isel)
tablegen(LLVM GameBoyDisassemblerTables.inc -gen-disassembler)
tablegen(LLVM GameBoyRegisterInfo.inc -gen-register-info)
tablegen(LLVM GameBoyInstrInfo.inc -gen-instr-info)
tablegen(LLVM GameBoyMCCodeEmitter.inc -gen-emitter)
tablegen(LLVM GameBoyGenCodeEmitter.inc -gen-emitter)
# tablegen(LLVM GameBoySubtargetInfo.inc -gen-subtarget)

add_public_tablegen_target(GameBoyCommonTableGen)

add_llvm_target(GameBoyCodeGen
  GameBoyAsmPrinter.cpp
  GameBoyFrameLowering.cpp
  GameBoyInstrInfo.cpp
  GameBoyISelLowering.cpp
  GameBoyISelDAGToDAG.cpp
  GameBoyMachineFunction.cpp
  GameBoyMCInstLower.cpp
  GameBoyRegisterInfo.cpp
  # GameBoySubtarget.cpp
  GameBoyTargetMachine.cpp
  GameBoyTargetObjectFile.cpp

  LINK_COMPONENTS
  AsmPrinter
  CodeGen
  Core
  GlobalISel
  MC
  SelectionDAG
  GameBoyDesc
  GameBoyInfo
  Support
  Target

  ADD_TO_COMPONENT
  GameBoy
)

# add_subdirectory(AsmParser)
# add_subdirectory(Disassembler)
add_subdirectory(MCTargetDesc)
add_subdirectory(TargetInfo)