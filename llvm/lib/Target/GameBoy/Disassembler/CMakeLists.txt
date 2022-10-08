add_llvm_component_library(LLVMGameBoyDisassembler
  GameBoyDisassembler.cpp

  LINK_COMPONENTS
  MCDisassembler
  GameBoyInfo
  Support

  ADD_TO_COMPONENT
  GameBoy
)

