add_llvm_component_library(LLVMGameBoyDesc
  GameBoyAsmBackend.cpp
  GameBoyELFObjectWriter.cpp
  GameBoyInstPrinter.cpp
  GameBoyMCAsmInfo.cpp
  GameBoyMCCodeEmitter.cpp
  GameBoyMCTargetDesc.cpp

  LINK_COMPONENTS
  MC
#   MCDisassembler
#   Support
#   GameBoyInfo

  ADD_TO_COMPONENT
  GameBoy
)