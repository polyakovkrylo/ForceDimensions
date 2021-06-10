#include "initForceDimensions.h"

extern "C" {
  void initExternalModule()
  {

  }

  const char* getModuleName()
  {
    return "ForceDimensions";
  }

  const char* getModuleVersion()
  {
    return "0.1";
  }
  
  const char* getModuleLicense()
  {
    return "LGPL";
  }
  
  const char* getModuleDescription()
  {
    return "ForceDimensions haptic device drivers";
  }

  const char* getModuleComponentList()
  {
    return "OmegaDriver";
  }
}
