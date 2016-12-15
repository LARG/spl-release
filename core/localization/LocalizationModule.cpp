#include <localization/LocalizationModule.h>
#include <memory/BehaviorBlock.h>
#include <iostream>

void LocalizationModule::specifyMemoryDependency() {
  requiresMemoryBlock("world_objects");
  requiresMemoryBlock("localization");
  requiresMemoryBlock("team_packets");
  requiresMemoryBlock("vision_frame_info");
  requiresMemoryBlock("vision_odometry");
  requiresMemoryBlock("robot_state");
  requiresMemoryBlock("game_state");
  requiresMemoryBlock("vision_joint_angles");
  requiresMemoryBlock("behavior");
  requiresMemoryBlock("vision_processed_sonar");
  requiresMemoryBlock("delayed_localization");
  requiresMemoryBlock("vision_walk_request");
  requiresMemoryBlock("vision_sensors");
  requiresMemoryBlock("calibration");
}

void LocalizationModule::specifyMemoryBlocks() {
  getOrAddMemoryBlock(cache_.world_object,"world_objects");
  getOrAddMemoryBlock(cache_.localization_mem,"localization");
  getOrAddMemoryBlock(cache_.team_packets,"team_packets");
  getOrAddMemoryBlock(cache_.frame_info,"vision_frame_info");
  getOrAddMemoryBlock(cache_.odometry,"vision_odometry");
  getOrAddMemoryBlock(cache_.robot_state,"robot_state");
  getOrAddMemoryBlock(cache_.game_state,"game_state");
  getOrAddMemoryBlock(cache_.joint,"vision_joint_angles");
  getOrAddMemoryBlock(cache_.behavior,"behavior");
  getOrAddMemoryBlock(cache_.sonar,"vision_processed_sonar");
  getOrAddMemoryBlock(cache_.delayed_localization,"delayed_localization");
  getOrAddMemoryBlock(cache_.walk_request, "vision_walk_request");
  getOrAddMemoryBlock(cache_.sensor, "vision_sensors");
  getOrAddMemoryBlock(cache_.calibration, "calibration");
}

void LocalizationModule::initConfig() {
}

void LocalizationModule::filterCloseBallPosition(float offset_x, float offset_y, float scale_y) {
}
