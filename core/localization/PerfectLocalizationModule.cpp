#include <localization/PerfectLocalizationModule.h>

#include <memory/BodyModelBlock.h>
#include <memory/JointBlock.h>
#include <memory/SimTruthDataBlock.h>
#include <memory/WorldObjectBlock.h>
#include <memory/RobotStateBlock.h>

#include <math/Geometry.h>
#include <common/ImageParams.h>

void PerfectLocalizationModule::specifyMemoryDependency() {
  requiresMemoryBlock("vision_body_model");
  requiresMemoryBlock("vision_joint_angles");
  requiresMemoryBlock("sim_truth_data");
  requiresMemoryBlock("world_objects");
  requiresMemoryBlock("robot_state");
}

void PerfectLocalizationModule::specifyMemoryBlocks() {
  getMemoryBlock(body_model_,"vision_body_model");
  getMemoryBlock(joint_angles_,"vision_joint_angles");
  getMemoryBlock(truth_data_,"sim_truth_data");
  getMemoryBlock(world_objects_,"world_objects");
  getMemoryBlock(robot_state_,"robot_state");
}

void PerfectLocalizationModule::processFrame() {
  if (!isMemorySatisfied()) return;
  
  int WO_SELF=robot_state_->WO_SELF;
  
  WorldObject* self = &(world_objects_->objects_[WO_SELF]);
  self->loc.x = truth_data_->robot_pos_.translation.x;
  self->loc.y = truth_data_->robot_pos_.translation.y;
  self->orientation = truth_data_->robot_pos_.rotation;

  ImageParams iparams(Camera::TOP);  
  WorldObject* ball = &(world_objects_->objects_[WO_BALL]);
  ball->loc.x = truth_data_->ball_pos_.translation.x;
  ball->loc.y = truth_data_->ball_pos_.translation.y;
  Pose2D relBall = truth_data_->ball_pos_.globalToRelative(truth_data_->robot_pos_);
  ball->relPos.x = relBall.translation.x;
  ball->relPos.y = relBall.translation.y;
  ball->distance = relBall.translation.abs();
  ball->bearing = self->loc.getBearingTo(ball->loc, self->orientation);
  float orientation_diff = (self->orientation + joint_angles_->values_[HeadYaw]) - ball->orientation;
  int centerX = int(iparams.width * (orientation_diff / FOVx + 0.5));
  ball->imageCenterX = crop(centerX,0,iparams.width);
  // TODO this is incorrect, but roughly correct
  float camera_height = body_model_->abs_parts_[BodyPart::head].translation.z;
  float ball_dist = self->loc.getDistanceTo(ball->loc);
  float tilt_diff = acos(camera_height / ball_dist) + joint_angles_->values_[HeadPitch];
  int centerY = int(iparams.height * (tilt_diff / FOVy));
  ball->imageCenterY = crop(centerY,0,iparams.height);

  for (int i = 0; i < NUM_WORLD_OBJS; i++){
    // calculate distance and bearing to each object
    world_objects_->objects_[i].distance = self->loc.getDistanceTo(world_objects_->objects_[i].loc);
    world_objects_->objects_[i].bearing = self->loc.getBearingTo(world_objects_->objects_[i].loc, self->orientation);
  }
  
}
