#ifndef PERFECTLOCALIZATION_99KDYIX5
#define PERFECTLOCALIZATION_99KDYIX5

#include <Module.h>

class BodyModelBlock;
class JointBlock;
class RobotStateBlock;
class SimTruthDataBlock;
class WorldObjectBlock;

class PerfectLocalizationModule: public Module {
public:
  void specifyMemoryDependency();
  void specifyMemoryBlocks();
  void processFrame();

  BodyModelBlock* body_model_;
  JointBlock* joint_angles_;
  SimTruthDataBlock* truth_data_;
  WorldObjectBlock* world_objects_;
  RobotStateBlock* robot_state_;
};

#endif
