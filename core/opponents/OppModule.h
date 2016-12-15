#ifndef OPP_MODULE_H
#define OPP_MODULE_H

#include <Module.h>

class OppModule: public Module {
 public:
  void specifyMemoryDependency();
  void specifyMemoryBlocks();
  void initSpecificModule();
  void reInit() { }

  void processFrame();
};

#endif
