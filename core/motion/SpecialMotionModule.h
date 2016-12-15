#ifndef SPECIAL_MOTION_MODULE_H
#define SPECIAL_MOTION_MODULE_H

#include <Module.h>

#include <common/Enum.h>
class SpecialMotionModule: public Module {
public:
  ENUM (Motion,
    Getup,
    standUpBackNao,
    standUpFrontNao,
    backFreeArms,
    goalieSquat,
    defenderSquat,
    diveLeft,
    diveRight
    //kickSidewardsNao,
    //kickDiagonalNao
  );

  ENUM (State,
    INITIAL,
    EXECUTE,
    WAIT,
    FINISHED,
    // from getup
    STIFFNESS_OFF,
    PREPARE_ARMS,
    STIFFNESS_ON,
    CROSS,
    STAND,
    FREE_ARMS,
    PAUSE_BEFORE_RESTART,
    // don't use this state except for testing, never changes states
    NO_ESCAPE
  );

public:
  SpecialMotionModule();
  void initSpecificModule();
  void specifyMemoryDependency();
  void specifyMemoryBlocks();

  void processFrame();
};

#endif /* end of include guard: GETUP_MODULE */
