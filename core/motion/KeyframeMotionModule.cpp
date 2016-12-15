#include <motion/KeyframeMotionModule.h>
#include <common/Keyframe.h>
#include <memory/FrameInfoBlock.h>
#include <memory/JointCommandBlock.h>
#include <memory/WalkRequestBlock.h>
#include <memory/OdometryBlock.h>
#include <memory/BodyModelBlock.h>
#include <memory/JointBlock.h>
#include <memory/SensorBlock.h>
#include <memory/KickRequestBlock.h>

#define JOINT_EPSILON (3.f * DEG_T_RAD)
#define DEBUG false

KeyframeMotionModule::KeyframeMotionModule() : state_(Finished), sequence_(NULL) { }

void KeyframeMotionModule::initSpecificModule() {
}

void KeyframeMotionModule::start() {
}

void KeyframeMotionModule::finish() {
}

bool KeyframeMotionModule::finished() {
  return state_ == Finished;
}

void KeyframeMotionModule::specifyMemoryDependency() {
  requiresMemoryBlock("frame_info");
  requiresMemoryBlock("walk_request");
  requiresMemoryBlock("processed_joint_angles");
  requiresMemoryBlock("processed_joint_commands");
  requiresMemoryBlock("odometry");
  requiresMemoryBlock("processed_sensors");
  requiresMemoryBlock("body_model");
  requiresMemoryBlock("kick_request");
}

void KeyframeMotionModule::specifyMemoryBlocks() {
  cache_.memory = memory_;
  getMemoryBlock(cache_.frame_info,"frame_info");
  getMemoryBlock(cache_.walk_request,"walk_request");
  getMemoryBlock(cache_.joint,"processed_joint_angles");
  getMemoryBlock(cache_.joint_command,"processed_joint_commands");
  getMemoryBlock(cache_.odometry,"odometry");
  getMemoryBlock(cache_.sensor,"processed_sensors");
  getMemoryBlock(cache_.body_model,"body_model");
  getMemoryBlock(cache_.kick_request,"kick_request");
}

void KeyframeMotionModule::processFrame() {
}

void KeyframeMotionModule::performKick() {
}

bool KeyframeMotionModule::reachedKeyframe(const Keyframe& keyframe) {
  return true;
}

void KeyframeMotionModule::moveToInitial(const Keyframe& keyframe, int cframe) {
}

void KeyframeMotionModule::moveBetweenKeyframes(const Keyframe& start, const Keyframe& finish, int cframe) {
}
