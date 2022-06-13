set(LLVM_TARGET_DEFINITIONS GameBoy.td)

# tablegen(LLVM SparcGenAsmMatcher.inc -gen-asm-matcher)
# tablegen(LLVM SparcGenAsmWriter.inc -gen-asm-writer)
# tablegen(LLVM SparcGenCallingConv.inc -gen-callingconv)
# tablegen(LLVM SparcGenDAGISel.inc -gen-dag-isel)
# tablegen(LLVM SparcGenDisassemblerTables.inc -gen-disassembler)
# tablegen(LLVM SparcGenInstrInfo.inc -gen-instr-info)
# tablegen(LLVM SparcGenMCCodeEmitter.inc -gen-emitter)
# tablegen(LLVM SparcGenRegisterInfo.inc -gen-register-info)
# tablegen(LLVM SparcGenSubtargetInfo.inc -gen-subtarget)

add_public_tablegen_target(GameBoyCommonTableGen)

add_llvm_target(GameBoyCodeGen
LINK_COMPONENTS
  AsmPrinter
  CodeGen
  Core
  MC
  SelectionDAG
  GameBoyDesc
  GameBoyInfo
  Support
  Target

  ADD_TO_COMPONENT
  GameBoy
  )

add_subdirectory(AsmParser)
add_subdirectory(Disassembler)
add_subdirectory(MCTargetDesc)
add_subdirectory(TargetInfo)