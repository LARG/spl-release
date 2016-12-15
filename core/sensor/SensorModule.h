#ifndef SENSOR_MODULE_H
#define SENSOR_MODULE_H

#include <Module.h>

class SensorModule: public Module {
 public:
  void specifyMemoryDependency();
  void specifyMemoryBlocks();
  void initSpecificModule();

  void processFrames() { }

};

#endif /* end of include guard: SENSOR_MODULE */
