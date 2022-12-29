add_llvm_component_library(LLVMGameBoyAsmParser
  GameBoyAsmParser.cpp

  LINK_COMPONENTS
  MC
  MCParser
  GameBoyDesc
  GameBoyInfo
  Support

  ADD_TO_COMPONENT
  GameBoy
)
