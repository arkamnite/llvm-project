add_llvm_component_library(LLVMGameBoyInfo
  GameBoyTargetInfo.cpp
  
  LINK_COMPONENTS
  MC
  Support
  
  ADD_TO_COMPONENT
  GameBoy
)