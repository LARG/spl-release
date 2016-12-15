#pragma once

#include <Module.h>
#include <common/RobotInfo.h>

class WalkModule : public Module {
  public:
    virtual void handleStepIntoKick() = 0;
    virtual void processFrame() = 0;

    virtual float getOffsetX() = 0;
    virtual float getOffsetY() = 0;

    float STAND_ANGLES[NUM_JOINTS];
};
