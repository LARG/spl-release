#include "DiveModule.h"

#include <memory/FrameInfoBlock.h>
#include <memory/JointCommandBlock.h>
#include <memory/WalkRequestBlock.h>
#include <memory/OdometryBlock.h>
#include <memory/BodyModelBlock.h>
#include <memory/SensorBlock.h>
#include <memory/JointBlock.h>
#define FIRST_FRAME_GRADUAL

DiveModule::DiveModule()
{
}

void DiveModule::initDive(Dive::diveTypes type, int role) {
 
  if (role == Role::KEEPER){
    // Dive left
    if (type == Dive::LEFT){
        currMotion = diveLeft;
    }
    // Dive right
    else if (type == Dive::RIGHT){
        currMotion = diveRight;
    }
    // Squat center
    else if (type == Dive::CENTER){
      currMotion = goalieSquat;
    }
    // Penalty Dive Left
    else if (type == Dive::PENALTY_LEFT){
        currMotion = penaltyDiveLeft;
    }
    // Penalty Dive Right
    else if (type == Dive::PENALTY_RIGHT){
        currMotion = penaltyDiveRight;
    }
    else 
      return;
  }
  else {
    if (type == Dive::CENTER){
      currMotion = defenderSquat;
    }
    else
      return;
  }

  startMotion(currMotion);
}

bool DiveModule::processFrameChild() {
  if (state == EXECUTE){
    if (isMotionDoneExecuting()){
      std::cout << "DIVE Motion " << getName(currMotion) << " complete" << std::endl;
      walk_request_->dive_type_ = Dive::DONE;
      transitionToState(WAIT);
    }
    else {
      //std::cout << "EXECUTE DIVE "<< getName(currMotion) << std::endl;
      executeMotionSequence();
    }
    return true;
  } 
  return false;
}


