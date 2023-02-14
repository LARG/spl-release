#include <localization/LocalizationModule.h>
#include <memory/BehaviorBlock.h>
#include <iostream>
#include <localization/Logging.h>

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
  config_.load(util::cfgpath(util::Modules) + "/localization.yaml");
}

void LocalizationModule::filterCloseBallPosition(float offset_x, float offset_y, float scale_y) {
  Point2D relBall = cache_.behavior->keeperRelBallPos;
  tlog(50, "filtering close ball pos, from %2.f,%2.f, offset %2.2f,%2.2f, scale %2.2f", 
    relBall.x, relBall.y, offset_x, offset_y, scale_y
  );
  filtered_close_ball_.x =  (1.08360979  * (relBall.x + offset_x)) + 20.81230986; // tuned 7/12/14 in Austin on 45 by katie
  filtered_close_ball_.y = scale_y * (relBall.y + offset_y);
  // filtered_close_ball_.x =  relBall.x + offset_x; 
  // filtered_close_ball_.y = relBall.y + offset_y;

  tlog(50, "produced %2.f,%2.f", filtered_close_ball_.x, filtered_close_ball_.y);
  // std::cout << "relBall.x: " << relBall.x << std::endl; 
  // std::cout << "relBall.y: " << relBall.y << std::endl; 
  // std::cout << "offset.x: " << offset_x << std::endl; 
  // std::cout << "offset.y: " << offset_y << std::endl; 
  // std::cout << "scale.y: " << scale_y << std::endl; 
  // std::cout << "filtered.x : " << filtered_close_ball_.x << std::endl; 
  // std::cout << "filtered.y : " << filtered_close_ball_.y << std::endl; 
}
