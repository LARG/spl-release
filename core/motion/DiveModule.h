#ifndef DIVE_MODULE_H
#define DIVE_MODULE_H


#include <string>
#include <iostream>
#include <Module.h>
#include <common/RobotInfo.h>
#include <vector>
#include <common/Roles.h>
#include <memory/BehaviorBlock.h>
#include <motion/SpecialMotionModule.h>

class DiveModule: public SpecialMotionModule {
public:
  DiveModule();

  bool isDiving() { return isDoingSpecialMotion() && (currMotion == diveLeft || currMotion == diveRight || currMotion == goalieSquat || currMotion == defenderSquat || currMotion == penaltyDiveLeft || currMotion == penaltyDiveRight); }
  void initDive(Dive::diveTypes type, int role);

protected:
  bool processFrameChild();
};


#endif

