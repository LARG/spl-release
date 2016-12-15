#ifndef SONARMODULE_RGN30PU
#define SONARMODULE_RGN30PU

#include <Module.h>
class SonarModule: public Module {
 public:
  void specifyMemoryDependency();
  void specifyMemoryBlocks();
  void initSpecificModule();

  void processFrame();
};

#endif /* end of include guard: SONARMODULE_RGN30PU */
