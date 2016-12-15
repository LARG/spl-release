#include "almotionwrapper.h"

#include <alcommon/alproxy.h>
#include <alproxies/almotionproxy.h>

#include <math/Geometry.h>

#include <memory/JointCommandBlock.h>
#include <memory/WalkInfoBlock.h>
#include <memory/ALWalkParamBlock.h>
#include <memory/WalkRequestBlock.h>

ALMotionWrapper::ALMotionWrapper() {
}

ALMotionWrapper::~ALMotionWrapper() {
}

void ALMotionWrapper::setProxy(boost::shared_ptr<AL::ALMotionProxy> pr) {
  motion_proxy_ = pr;
}

void ALMotionWrapper::init() {
  // turn off walk stiffness protection here
  AL::ALValue config;
  config.arraySetSize(1);
  config[0].arraySetSize(2);
  config[0][0] = "ENABLE_STIFFNESS_PROTECTION";
  config[0][1] = false;

  motion_proxy_->setMotionConfig(config);

  // TODO FALL MANAGEMENT
  //motion_proxy_->setFallManagerEnabled(false);
  //config[0][0] = "ENABLE_FULL_FALL_MANAGEMENT_PROTECTION";
  //motion_proxy_->setMotionConfig(config);

  config[0][0] = "ENABLE_FOOT_CONTACT_PROTECTION";
  motion_proxy_->setMotionConfig(config);

  body_position_commands_.arraySetSize(NUM_BODY_JOINTS);
  head_position_commands_.arraySetSize(1);
  stiffness_commands_.arraySetSize(NUM_JOINTS);
  arm_position_commands_.arraySetSize(8);

  body_joint_names_.arraySetSize(NUM_BODY_JOINTS);
  for (int i = 0; i < NUM_BODY_JOINTS; i++)
    body_joint_names_[i] = getJointName((Joint)(i + BODY_JOINT_OFFSET));

  leg_joint_names_.arraySetSize(12);
  int ind = 0;
  for (int i = LHipYawPitch; i <= LAnkleRoll; i++) {
    leg_joint_names_[ind] = getJointName((Joint)i);
    ind++;
  }
  for (int i = RHipYawPitch; i <= RAnkleRoll; i++) {
    leg_joint_names_[ind] = getJointName((Joint)i);
    ind++;
  }

  arm_joint_names_.arraySetSize(8);
  ind = 0;
  for (int i = LShoulderPitch; i <= LElbowRoll; i++) {
    arm_joint_names_[ind] = getJointName((Joint)i);
    ind++;
  }
  for (int i = RShoulderPitch; i <= RElbowRoll; i++) {
    arm_joint_names_[ind] = getJointName((Joint)i);
    ind++;
  }

  head_pitch_joint_names_.arraySetSize(1);
  head_yaw_joint_names_.arraySetSize(1);

  head_pitch_joint_names_[0] = getJointName((Joint)HeadPitch);
  head_yaw_joint_names_[0] = getJointName((Joint)HeadYaw);

  stiffness_joint_names_.arraySetSize(NUM_JOINTS);
  for (int i = 0; i < NUM_JOINTS; i++)
    stiffness_joint_names_[i] = getJointName((Joint)i);

  // init params
  walk_params_.arraySetSize(NUM_AL_WALK_PARAMS);

  for (int i = 0; i < NUM_AL_WALK_PARAMS; i++){
    walk_params_[i].arraySetSize(2);
    walk_params_[i][0] = walkParamNames[i];
    walk_params_[i][1] = 0.0f;
  }

  //motion_proxy_->setWalkArmsEnable(true,true); //TODO: do we need this? it breaks the build - JM 06/14/15

  //// turn this off for gouda with bad foot contact
  //if (core->currentMemory()->commonMem->id == 98 ||
  //core->currentMemory()->commonMem->id == 93){

  //cout << "ID " << core->currentMemory()->commonMem->id
  //<< " turning off foot contact protection" << endl;

  //// turn off walk stiffness protection here
  //ALValue config2;
  //config2.arraySetSize(1);
  //config2[0].arraySetSize(2);
  //config2[0][0] = "ENABLE_FOOT_CONTACT_PROTECTION";
  //config2[0][1] = false;

  //motion_proxy_->setMotionConfig(config2);
  //}

  std::cout << "Done !" << std::endl;
}

void ALMotionWrapper::sendWalk(WalkRequestBlock *walk_request, WalkInfoBlock *walk_info) {
  if (!walk_request->new_command_)
    return;

  if ((walk_request->motion_ == WalkRequestBlock::WALK || walk_request->motion_ == WalkRequestBlock::STAND || walk_request->motion_ == WalkRequestBlock::STEP_LEFT || walk_request->motion_ == WalkRequestBlock::STEP_RIGHT) && (!motion_proxy_->walkIsActive())) {
    //motion_proxy_->killTasksUsingResources(body_joint_names_);
    motion_proxy_->killTasksUsingResources(leg_joint_names_);
  }

  switch (walk_request->motion_) {
  case WalkRequestBlock::WALK:
    //motion_proxy_->killTasksUsingResources(body_joint_names_);
    if (walk_request->walk_to_target_) {
      //std::cout << "target: " << walk_request->target_point_ << std::endl;
      getWalkInfo(walk_info);
      Pose2D target = walk_request->target_point_.globalToRelative(walk_info->robot_relative_next_position_);

      motion_proxy_->post.walkTo(target.translation.x / 1000.0,target.translation.y / 1000.0,target.rotation);//walk_request->target_point_.rotation); // /1000.0 for convert from m to mm
      //motion_proxy_->post.walkTo(walk_request->target_point_.x / 1000.0,walk_request->target_point_.y / 1000.0,0);//walk_request->target_point_.rotation); // /1000.0 for convert from m to mm
    } else {
      motion_proxy_->setWalkTargetVelocity(walk_request->speed_.translation.x,walk_request->speed_.translation.y,walk_request->speed_.rotation,1.0,walk_params_);
    }
    break;
  case WalkRequestBlock::STEP_LEFT:
    std::cerr << "STEP_LEFT currently not supported" << std::endl;
    //motion_proxy_->post.stepTo("LLeg", walk_request->speed_.translation.x,walk_request->speed_.translation.y,walk_request->speed_.rotation);
    break;
  case WalkRequestBlock::STEP_RIGHT:
    std::cerr << "STEP_RIGHT currently not supported" << std::endl;
    //motion_proxy_->post.stepTo("RLeg", walk_request->speed_.translation.x,walk_request->speed_.translation.y,walk_request->speed_.rotation);
    break;
  case WalkRequestBlock::STAND:
    motion_proxy_->setWalkTargetVelocity(0,0,0,1.0,walk_params_);
    break;
  case WalkRequestBlock::NONE:
    motion_proxy_->killWalk();
    break;
  case WalkRequestBlock::WAIT:
    // do nothing
    break;
  default:
    // TODO
    motion_proxy_->killWalk();
  }
}

void ALMotionWrapper::sendWalkParameters(ALWalkParamBlock *params) {
  if (!params->send_params_)
    return;

  params->send_params_ = false;
  motion_proxy_->setCollisionProtectionEnabled("Arms",false);
  // fill in values
  walk_params_[0][1] = params->maxStepX;
  walk_params_[1][1] = params->maxStepY;
  walk_params_[2][1] = params->maxStepTheta;
  walk_params_[3][1] = params->maxStepFrequency;
  walk_params_[4][1] = params->stepHeight;
  walk_params_[5][1] = params->torsoWx;
  walk_params_[6][1] = params->torsoWy;

  //AL::ALValue gait;
  //gait = motion_proxy_->getFootGaitConfig("Max");
  //gait[1][1] = 0.10;
  //gait = motion_proxy_->getFootGaitConfig("Max");
  //int numParams = 7;
  //std::cout << "Max: " << gait.getSize() << std::endl;
  //for (int i = 0; i < numParams; i++) {
    //std::cout << "  " << gait[i][0] << " " << gait[i][1] << std::endl;
  //}
  //gait = motion_proxy_->getFootGaitConfig("Min");
  //std::cout << "Min: " << gait.getSize() << std::endl;
  //for (int i = 0; i < numParams; i++) {
    //std::cout << "  " << gait[i][0] << " " << gait[i][1] << std::endl;
  //}
  //gait = motion_proxy_->getFootGaitConfig("Default");
  //std::cout << "Default: " << gait.getSize() << std::endl;
  //for (int i = 0; i < numParams; i++) {
    //std::cout << "  " << gait[i][0] << " " << gait[i][1] << std::endl;
  //}

  // fill in values
  //walk_params_[0][1] = params->walk_max_trapezoid_;
  //walk_params_[1][1] = params->walk_min_trapezoid_;
  //walk_params_[2][1] = params->walk_step_max_period_;
  //walk_params_[3][1] = params->walk_step_min_period_;
  //walk_params_[4][1] = params->walk_max_step_x_;
  //walk_params_[5][1] = params->walk_max_step_y_;
  //walk_params_[6][1] = params->walk_max_step_theta_;
  //walk_params_[7][1] = params->walk_step_height_;
  //walk_params_[8][1] = params->walk_foot_separation_;
  //walk_params_[9][1] = params->walk_foot_orientation_;
  //walk_params_[10][1] = params->walk_torso_height_;
  //walk_params_[11][1] = params->walk_inverted_pendulum_height_;
  //walk_params_[12][1] = params->torso_orientation_x_;
  //walk_params_[13][1] = params->torso_orientation_y_;

  //motion_proxy_->setMotionConfig(walk_params_);
  //motion_proxy_->getMotionConfig(walk_params_);

  //std::cout << "********************" << std::endl;
  //for (int i = 0; i < 14; i++)
  //std::cout << walk_params_[i][0] << " = " << walk_params_[i][1] << std::endl;
  //std::cout << "********************" << std::endl;

  // do some calculations of the max velocities
  //float steps_per_second = 50.0 / params->walk_step_min_period_;
  //max_vels_.translation.x = params->walk_max_step_x_ * steps_per_second * 1000.0;
  //max_vels_.translation.y = params->walk_max_step_y_ * steps_per_second * 1000.0;
  //max_vels_.rotation = DEG_T_RAD * params->walk_max_step_theta_ * steps_per_second;
}

void ALMotionWrapper::sendToActuators(JointCommandBlock *raw_joint_commands) {
  if (raw_joint_commands->send_arm_angles_) {
    int ind = 0;
    for (int i = LShoulderPitch; i <= LElbowRoll; i++) {
      arm_position_commands_[ind] = crop(raw_joint_commands->angles_[i],minJointLimits[i],maxJointLimits[i]);
      //std::cout << arm_position_commands[ind] << std::endl;
      ind++;
    }
    for (int i = RShoulderPitch; i <= RElbowRoll; i++) {
      arm_position_commands_[ind] = crop(raw_joint_commands->angles_[i],minJointLimits[i],maxJointLimits[i]);
      //std::cout << arm_position_commands[ind] << std::endl;
      ind++;
    }

    motion_proxy_->killTasksUsingResources(arm_joint_names_);
    motion_proxy_->post.angleInterpolation(arm_joint_names_,arm_position_commands_,raw_joint_commands->arm_command_time_ / 1000.0,true);
  }

  if (raw_joint_commands->send_body_angles_) {
    int joint_ind;
    for (int i = 0; i < NUM_BODY_JOINTS; i++) {
      joint_ind = i + BODY_JOINT_OFFSET;
      body_position_commands_[i] = crop(raw_joint_commands->angles_[joint_ind],minJointLimits[joint_ind],maxJointLimits[joint_ind]);
    }
    body_position_time_ = raw_joint_commands->body_angle_time_ / 1000.0; // to sec

    motion_proxy_->killTasksUsingResources(body_joint_names_);
    motion_proxy_->post.angleInterpolation(body_joint_names_,body_position_commands_,body_position_time_,true);
  }

  if (raw_joint_commands->send_head_pitch_angle_) {
    //    std::cout << "SEND HEAD PITCH ANGLE" << std::endl;
    head_position_commands_[0] = crop(raw_joint_commands->angles_[HeadPitch],minJointLimits[HeadPitch],maxJointLimits[HeadPitch]);
    head_position_time_ = raw_joint_commands->head_pitch_angle_time_ / 1000.0; // to sec
    motion_proxy_->killTasksUsingResources(head_pitch_joint_names_);
    if (raw_joint_commands->head_pitch_angle_change_){
      // change
      // 0.75 is speed fraction
      motion_proxy_->changeAngles(head_pitch_joint_names_,head_position_commands_,0.75);
    } else {
      // interp
      motion_proxy_->post.angleInterpolation(head_pitch_joint_names_,head_position_commands_,head_position_time_,true);
    }
  }

  if (raw_joint_commands->send_head_yaw_angle_) {
    //    std::cout << "SEND HEAD YAW ANGLE" << std::endl;
    head_position_commands_[0] = crop(raw_joint_commands->angles_[HeadYaw],minJointLimits[HeadYaw],maxJointLimits[HeadYaw]);
    head_position_time_ = raw_joint_commands->head_yaw_angle_time_ / 1000.0; // to sec
    motion_proxy_->killTasksUsingResources(head_yaw_joint_names_);
    if (raw_joint_commands->head_yaw_angle_change_){
      // change
      // 0.75 is speed fraction
      motion_proxy_->changeAngles(head_yaw_joint_names_,head_position_commands_,0.75);
    } else {
      // interp
      motion_proxy_->post.angleInterpolation(head_yaw_joint_names_,head_position_commands_,head_position_time_,true);
    }
  }

  if (raw_joint_commands->send_stiffness_) {
    for (int i = 0; i < NUM_JOINTS; i++)
      stiffness_commands_[i] = raw_joint_commands->stiffness_[i];
    stiffness_time_= raw_joint_commands->stiffness_time_ / 1000.0; // to sec
    // Todd: stiffnessInterpolation doesn't work correctly (sometimes huge delay)
    //motion_proxy_->post.stiffnessInterpolation(stiffness_joint_names_,stiffness_commands_,stiffness_time_);
    motion_proxy_->setStiffnesses(stiffness_joint_names_,stiffness_commands_);
  }

  if (raw_joint_commands->send_back_standup_){
    motion_proxy_->killTasksUsingResources(body_joint_names_);
    CarlosBackStandup();
    raw_joint_commands->send_back_standup_ = false;
  }
}

void ALMotionWrapper::getWalkInfo(WalkInfoBlock *walk_info) {

  // figure out current torso location in global frame
  std::vector<float> val;

  // get current robot position
  val = motion_proxy_->getRobotPosition(false);
  walk_info->robot_position_.setValues(val);
  walk_info->robot_position_.translation *= 1000.0;

  // get current robot velocity
  val = motion_proxy_->getRobotVelocity();
  walk_info->robot_velocity_.setValues(val);
  walk_info->robot_velocity_.translation *= 1000.0;

  // convert to fractions of max velocities
  walk_info->robot_velocity_frac_.translation.x = walk_info->robot_velocity_.translation.x / max_vels_.translation.x;
  walk_info->robot_velocity_frac_.translation.y = walk_info->robot_velocity_.translation.y / max_vels_.translation.y;
  walk_info->robot_velocity_frac_.rotation = walk_info->robot_velocity_.rotation / max_vels_.rotation;

  // next position command from our private motion.so
  val = motion_proxy_->getNextRobotPosition();
  walk_info->robot_next_position_.setValues(val);
  walk_info->robot_next_position_.translation *= 1000.0;

  // calculate relative next position
  walk_info->robot_relative_next_position_ = walk_info->robot_next_position_.globalToRelative(walk_info->robot_position_);

  walk_info->walk_is_active_ = motion_proxy_->walkIsActive();

}



void ALMotionWrapper::CarlosBackStandup(){
  AL::ALValue names, times, keys;

  names.arraySetSize(21);
  times.arraySetSize(21);
  keys.arraySetSize(21);

  names[0] = "HeadYaw";
  times[0].arraySetSize(12);
  keys[0].arraySetSize(12);

  times[0][0] = 1.00000;
  keys[0][0] = AL::ALValue::array(-0.0245859, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.333333, 0.00000));
  times[0][1] = 2.00000;
  keys[0][1] = AL::ALValue::array(-4.76838e-07, AL::ALValue::array(3, -0.333333, -0.0122152), AL::ALValue::array(3, 0.266667, 0.00977214));
  times[0][2] = 2.80000;
  keys[0][2] = AL::ALValue::array(0.0413760, AL::ALValue::array(3, -0.266667, -0.00000), AL::ALValue::array(3, 0.266667, 0.00000));
  times[0][3] = 3.60000;
  keys[0][3] = AL::ALValue::array(6.74350e-07, AL::ALValue::array(3, -0.266667, 3.85341e-07), AL::ALValue::array(3, 0.233333, -3.37173e-07));
  times[0][4] = 4.30000;
  keys[0][4] = AL::ALValue::array(3.37177e-07, AL::ALValue::array(3, -0.233333, 2.44455e-12), AL::ALValue::array(3, 0.166667, -1.74610e-12));
  times[0][5] = 4.80000;
  keys[0][5] = AL::ALValue::array(3.37175e-07, AL::ALValue::array(3, -0.166667, 1.74610e-12), AL::ALValue::array(3, 0.266667, -2.79377e-12));
  times[0][6] = 5.60000;
  keys[0][6] = AL::ALValue::array(-0.513931, AL::ALValue::array(3, -0.266667, -0.00000), AL::ALValue::array(3, 0.300000, 0.00000));
  times[0][7] = 6.50000;
  keys[0][7] = AL::ALValue::array(-0.302240, AL::ALValue::array(3, -0.300000, -0.129579), AL::ALValue::array(3, 0.300000, 0.129579));
  times[0][8] = 7.40000;
  keys[0][8] = AL::ALValue::array(0.263545, AL::ALValue::array(3, -0.300000, -0.00000), AL::ALValue::array(3, 0.200000, 0.00000));
  times[0][9] = 8.00000;
  keys[0][9] = AL::ALValue::array(0.120428, AL::ALValue::array(3, -0.200000, 0.0454910), AL::ALValue::array(3, 0.200000, -0.0454910));
  times[0][10] = 8.60000;
  keys[0][10] = AL::ALValue::array(-0.00940132, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.633333, 0.00000));
  times[0][11] = 10.5000;
  keys[0][11] = AL::ALValue::array(0.0659200, AL::ALValue::array(3, -0.633333, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

  names[1] = "HeadPitch";
  times[1].arraySetSize(12);
  keys[1].arraySetSize(12);

  times[1][0] = 1.00000;
  keys[1][0] = AL::ALValue::array(0.0628521, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.333333, 0.00000));
  times[1][1] = 2.00000;
  keys[1][1] = AL::ALValue::array(0.00000, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.266667, 0.00000));
  times[1][2] = 2.80000;
  keys[1][2] = AL::ALValue::array(0.514900, AL::ALValue::array(3, -0.266667, -0.00000), AL::ALValue::array(3, 0.266667, 0.00000));
  times[1][3] = 3.60000;
  keys[1][3] = AL::ALValue::array(-0.672001, AL::ALValue::array(3, -0.266667, -0.00000), AL::ALValue::array(3, 0.233333, 0.00000));
  times[1][4] = 4.30000;
  keys[1][4] = AL::ALValue::array(0.00000, AL::ALValue::array(3, -0.233333, -0.198541), AL::ALValue::array(3, 0.166667, 0.141815));
  times[1][5] = 4.80000;
  keys[1][5] = AL::ALValue::array(0.349066, AL::ALValue::array(3, -0.166667, -0.00000), AL::ALValue::array(3, 0.266667, 0.00000));
  times[1][6] = 5.60000;
  keys[1][6] = AL::ALValue::array(0.171766, AL::ALValue::array(3, -0.266667, -0.00000), AL::ALValue::array(3, 0.300000, 0.00000));
  times[1][7] = 6.50000;
  keys[1][7] = AL::ALValue::array(0.345107, AL::ALValue::array(3, -0.300000, -0.00000), AL::ALValue::array(3, 0.300000, 0.00000));
  times[1][8] = 7.40000;
  keys[1][8] = AL::ALValue::array(0.0104720, AL::ALValue::array(3, -0.300000, -0.00000), AL::ALValue::array(3, 0.200000, 0.00000));
  times[1][9] = 8.00000;
  keys[1][9] = AL::ALValue::array(0.0296706, AL::ALValue::array(3, -0.200000, -0.0191986), AL::ALValue::array(3, 0.200000, 0.0191986));
  times[1][10] = 8.60000;
  keys[1][10] = AL::ALValue::array(0.378859, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.633333, 0.00000));
  times[1][11] = 10.5000;
  keys[1][11] = AL::ALValue::array(-0.0399260, AL::ALValue::array(3, -0.633333, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

  names[2] = "LShoulderPitch";
  times[2].arraySetSize(12);
  keys[2].arraySetSize(12);

  times[2][0] = 1.00000;
  keys[2][0] = AL::ALValue::array(2.08560, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.333333, 0.00000));
  times[2][1] = 2.00000;
  keys[2][1] = AL::ALValue::array(2.08567, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.200000, 0.00000));
  times[2][2] = 2.60000;
  keys[2][2] = AL::ALValue::array(2.03865, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.333333, 0.00000));
  times[2][3] = 3.60000;
  keys[2][3] = AL::ALValue::array(2.08560, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.233333, 0.00000));
  times[2][4] = 4.30000;
  keys[2][4] = AL::ALValue::array(2.08560, AL::ALValue::array(3, -0.233333, -0.00000), AL::ALValue::array(3, 0.166667, 0.00000));
  times[2][5] = 4.80000;
  keys[2][5] = AL::ALValue::array(2.08560, AL::ALValue::array(3, -0.166667, -0.00000), AL::ALValue::array(3, 0.266667, 0.00000));
  times[2][6] = 5.60000;
  keys[2][6] = AL::ALValue::array(2.03251, AL::ALValue::array(3, -0.266667, 0.0530929), AL::ALValue::array(3, 0.300000, -0.0597295));
  times[2][7] = 6.50000;
  keys[2][7] = AL::ALValue::array(1.28698, AL::ALValue::array(3, -0.300000, 0.179222), AL::ALValue::array(3, 0.300000, -0.179222));
  times[2][8] = 7.40000;
  keys[2][8] = AL::ALValue::array(0.957173, AL::ALValue::array(3, -0.300000, 0.110755), AL::ALValue::array(3, 0.200000, -0.0738367));
  times[2][9] = 8.00000;
  keys[2][9] = AL::ALValue::array(0.733209, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.200000, 0.00000));
  times[2][10] = 8.60000;
  keys[2][10] = AL::ALValue::array(0.733209, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.633333, 0.00000));
  times[2][11] = 10.5000;
  keys[2][11] = AL::ALValue::array(1.59225, AL::ALValue::array(3, -0.633333, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

  names[3] = "LShoulderRoll";
  times[3].arraySetSize(13);
  keys[3].arraySetSize(13);

  times[3][0] = 1.00000;
  keys[3][0] = AL::ALValue::array(0.980184, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.333333, 0.00000));
  times[3][1] = 2.00000;
  keys[3][1] = AL::ALValue::array(0.366519, AL::ALValue::array(3, -0.333333, 0.0290888), AL::ALValue::array(3, 0.200000, -0.0174533));
  times[3][2] = 2.60000;
  keys[3][2] = AL::ALValue::array(0.349066, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.333333, 0.00000));
  times[3][3] = 3.60000;
  keys[3][3] = AL::ALValue::array(0.349066, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.233333, 0.00000));
  times[3][4] = 4.30000;
  keys[3][4] = AL::ALValue::array(0.366519, AL::ALValue::array(3, -0.233333, -0.00000), AL::ALValue::array(3, 0.166667, 0.00000));
  times[3][5] = 4.80000;
  keys[3][5] = AL::ALValue::array(0.00869999, AL::ALValue::array(3, -0.166667, -0.00000), AL::ALValue::array(3, 0.266667, 0.00000));
  times[3][6] = 5.60000;
  keys[3][6] = AL::ALValue::array(0.299088, AL::ALValue::array(3, -0.266667, -0.0965808), AL::ALValue::array(3, 0.133333, 0.0482904));
  times[3][7] = 6.00000;
  keys[3][7] = AL::ALValue::array(0.443314, AL::ALValue::array(3, -0.133333, -0.00000), AL::ALValue::array(3, 0.166667, 0.00000));
  times[3][8] = 6.50000;
  keys[3][8] = AL::ALValue::array(0.0705221, AL::ALValue::array(3, -0.166667, -0.00000), AL::ALValue::array(3, 0.300000, 0.00000));
  times[3][9] = 7.40000;
  keys[3][9] = AL::ALValue::array(0.266875, AL::ALValue::array(3, -0.300000, -0.0855972), AL::ALValue::array(3, 0.200000, 0.0570648));
  times[3][10] = 8.00000;
  keys[3][10] = AL::ALValue::array(0.498508, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.200000, 0.00000));
  times[3][11] = 8.60000;
  keys[3][11] = AL::ALValue::array(0.498508, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.633333, 0.00000));
  times[3][12] = 10.5000;
  keys[3][12] = AL::ALValue::array(0.219320, AL::ALValue::array(3, -0.633333, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

  names[4] = "LElbowYaw";
  times[4].arraySetSize(12);
  keys[4].arraySetSize(12);

  times[4][0] = 1.00000;
  keys[4][0] = AL::ALValue::array(-0.760906, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.333333, 0.00000));
  times[4][1] = 2.00000;
  keys[4][1] = AL::ALValue::array(0.157080, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.200000, 0.00000));
  times[4][2] = 2.60000;
  keys[4][2] = AL::ALValue::array(0.122678, AL::ALValue::array(3, -0.200000, 0.00872665), AL::ALValue::array(3, 0.333333, -0.0145444));
  times[4][3] = 3.60000;
  keys[4][3] = AL::ALValue::array(0.0872665, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.233333, 0.00000));
  times[4][4] = 4.30000;
  keys[4][4] = AL::ALValue::array(0.0872665, AL::ALValue::array(3, -0.233333, -0.00000), AL::ALValue::array(3, 0.166667, 0.00000));
  times[4][5] = 4.80000;
  keys[4][5] = AL::ALValue::array(0.0872665, AL::ALValue::array(3, -0.166667, -0.00000), AL::ALValue::array(3, 0.266667, 0.00000));
  times[4][6] = 5.60000;
  keys[4][6] = AL::ALValue::array(-1.96049, AL::ALValue::array(3, -0.266667, -0.00000), AL::ALValue::array(3, 0.300000, 0.00000));
  times[4][7] = 6.50000;
  keys[4][7] = AL::ALValue::array(-0.854480, AL::ALValue::array(3, -0.300000, -0.285068), AL::ALValue::array(3, 0.300000, 0.285068));
  times[4][8] = 7.40000;
  keys[4][8] = AL::ALValue::array(-0.250085, AL::ALValue::array(3, -0.300000, -0.187511), AL::ALValue::array(3, 0.200000, 0.125008));
  times[4][9] = 8.00000;
  keys[4][9] = AL::ALValue::array(0.0830765, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.200000, 0.00000));
  times[4][10] = 8.60000;
  keys[4][10] = AL::ALValue::array(0.0830765, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.633333, 0.00000));
  times[4][11] = 10.5000;
  keys[4][11] = AL::ALValue::array(-1.03089, AL::ALValue::array(3, -0.633333, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

  names[5] = "LElbowRoll";
  times[5].arraySetSize(13);
  keys[5].arraySetSize(13);

  times[5][0] = 1.00000;
  keys[5][0] = AL::ALValue::array(-0.624296, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.333333, 0.00000));
  times[5][1] = 2.00000;
  keys[5][1] = AL::ALValue::array(-0.0349066, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.200000, 0.00000));
  times[5][2] = 2.60000;
  keys[5][2] = AL::ALValue::array(-1.54462, AL::ALValue::array(3, -0.200000, 0.0104898), AL::ALValue::array(3, 0.333333, -0.0174830));
  times[5][3] = 3.60000;
  keys[5][3] = AL::ALValue::array(-1.56210, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.233333, 0.00000));
  times[5][4] = 4.30000;
  keys[5][4] = AL::ALValue::array(-0.698132, AL::ALValue::array(3, -0.233333, -0.302050), AL::ALValue::array(3, 0.166667, 0.215750));
  times[5][5] = 4.80000;
  keys[5][5] = AL::ALValue::array(-0.00869999, AL::ALValue::array(3, -0.166667, -0.00000), AL::ALValue::array(3, 0.133333, 0.00000));
  times[5][6] = 5.20000;
  keys[5][6] = AL::ALValue::array(-0.191986, AL::ALValue::array(3, -0.133333, 0.172908), AL::ALValue::array(3, 0.133333, -0.172908));
  times[5][7] = 5.60000;
  keys[5][7] = AL::ALValue::array(-1.04615, AL::ALValue::array(3, -0.133333, -0.00000), AL::ALValue::array(3, 0.300000, 0.00000));
  times[5][8] = 6.50000;
  keys[5][8] = AL::ALValue::array(-0.872804, AL::ALValue::array(3, -0.300000, -0.0360492), AL::ALValue::array(3, 0.300000, 0.0360492));
  times[5][9] = 7.40000;
  keys[5][9] = AL::ALValue::array(-0.829852, AL::ALValue::array(3, -0.300000, -0.0134900), AL::ALValue::array(3, 0.200000, 0.00899333));
  times[5][10] = 8.00000;
  keys[5][10] = AL::ALValue::array(-0.805354, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.200000, 0.00000));
  times[5][11] = 8.60000;
  keys[5][11] = AL::ALValue::array(-0.805354, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.633333, 0.00000));
  times[5][12] = 10.5000;
  keys[5][12] = AL::ALValue::array(-0.681054, AL::ALValue::array(3, -0.633333, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

 
  names[6] = "RShoulderPitch";
  times[6].arraySetSize(12);
  keys[6].arraySetSize(12);

  times[6][0] = 1.00000;
  keys[6][0] = AL::ALValue::array(2.08560, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.333333, 0.00000));
  times[6][1] = 2.00000;
  keys[6][1] = AL::ALValue::array(2.08567, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.200000, 0.00000));
  times[6][2] = 2.60000;
  keys[6][2] = AL::ALValue::array(1.95283, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.333333, 0.00000));
  times[6][3] = 3.60000;
  keys[6][3] = AL::ALValue::array(2.08560, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.233333, 0.00000));
  times[6][4] = 4.30000;
  keys[6][4] = AL::ALValue::array(2.08560, AL::ALValue::array(3, -0.233333, -0.00000), AL::ALValue::array(3, 0.166667, 0.00000));
  times[6][5] = 4.80000;
  keys[6][5] = AL::ALValue::array(2.08560, AL::ALValue::array(3, -0.166667, -0.00000), AL::ALValue::array(3, 0.266667, 0.00000));
  times[6][6] = 5.60000;
  keys[6][6] = AL::ALValue::array(2.07708, AL::ALValue::array(3, -0.266667, 0.00759185), AL::ALValue::array(3, 0.300000, -0.00854083));
  times[6][7] = 6.50000;
  keys[6][7] = AL::ALValue::array(2.03720, AL::ALValue::array(3, -0.300000, 0.0398808), AL::ALValue::array(3, 0.300000, -0.0398808));
  times[6][8] = 7.40000;
  keys[6][8] = AL::ALValue::array(1.44047, AL::ALValue::array(3, -0.300000, 0.229179), AL::ALValue::array(3, 0.200000, -0.152786));
  times[6][9] = 8.00000;
  keys[6][9] = AL::ALValue::array(0.891306, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.200000, 0.00000));
  times[6][10] = 8.60000;
  keys[6][10] = AL::ALValue::array(0.891306, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.633333, 0.00000));
  times[6][11] = 10.5000;
  keys[6][11] = AL::ALValue::array(1.46961, AL::ALValue::array(3, -0.633333, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

  names[7] = "RShoulderRoll";
  times[7].arraySetSize(12);
  keys[7].arraySetSize(12);

  times[7][0] = 1.00000;
  keys[7][0] = AL::ALValue::array(-1.07384, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.333333, 0.00000));
  times[7][1] = 2.00000;
  keys[7][1] = AL::ALValue::array(-0.366519, AL::ALValue::array(3, -0.333333, -0.0290888), AL::ALValue::array(3, 0.200000, 0.0174533));
  times[7][2] = 2.60000;
  keys[7][2] = AL::ALValue::array(-0.349066, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.333333, 0.00000));
  times[7][3] = 3.60000;
  keys[7][3] = AL::ALValue::array(-0.349066, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.233333, 0.00000));
  times[7][4] = 4.30000;
  keys[7][4] = AL::ALValue::array(-0.366519, AL::ALValue::array(3, -0.233333, -0.00000), AL::ALValue::array(3, 0.166667, 0.00000));
  times[7][5] = 4.80000;
  keys[7][5] = AL::ALValue::array(-0.00869999, AL::ALValue::array(3, -0.166667, -0.00000), AL::ALValue::array(3, 0.266667, 0.00000));
  times[7][6] = 5.60000;
  keys[7][6] = AL::ALValue::array(-0.457173, AL::ALValue::array(3, -0.266667, 0.0871929), AL::ALValue::array(3, 0.300000, -0.0980920));
  times[7][7] = 6.50000;
  keys[7][7] = AL::ALValue::array(-0.564555, AL::ALValue::array(3, -0.300000, 0.0414181), AL::ALValue::array(3, 0.300000, -0.0414181));
  times[7][8] = 7.40000;
  keys[7][8] = AL::ALValue::array(-0.705682, AL::ALValue::array(3, -0.300000, 0.0616220), AL::ALValue::array(3, 0.200000, -0.0410813));
  times[7][9] = 8.00000;
  keys[7][9] = AL::ALValue::array(-0.872665, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.200000, 0.00000));
  times[7][10] = 8.60000;
  keys[7][10] = AL::ALValue::array(-0.680678, AL::ALValue::array(3, -0.200000, -0.0577833), AL::ALValue::array(3, 0.633333, 0.182980));
  times[7][11] = 10.5000;
  keys[7][11] = AL::ALValue::array(-0.150374, AL::ALValue::array(3, -0.633333, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

  names[8] = "RElbowYaw";
  times[8].arraySetSize(12);
  keys[8].arraySetSize(12);

  times[8][0] = 1.00000;
  keys[8][0] = AL::ALValue::array(0.765425, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.333333, 0.00000));
  times[8][1] = 2.00000;
  keys[8][1] = AL::ALValue::array(-0.157080, AL::ALValue::array(3, -0.333333, 0.167790), AL::ALValue::array(3, 0.200000, -0.100674));
  times[8][2] = 2.60000;
  keys[8][2] = AL::ALValue::array(-0.257754, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.333333, 0.00000));
  times[8][3] = 3.60000;
  keys[8][3] = AL::ALValue::array(-0.0872665, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.233333, 0.00000));
  times[8][4] = 4.30000;
  keys[8][4] = AL::ALValue::array(-0.0872665, AL::ALValue::array(3, -0.233333, -0.00000), AL::ALValue::array(3, 0.166667, 0.00000));
  times[8][5] = 4.80000;
  keys[8][5] = AL::ALValue::array(-0.0872665, AL::ALValue::array(3, -0.166667, -0.00000), AL::ALValue::array(3, 0.266667, 0.00000));
  times[8][6] = 5.60000;
  keys[8][6] = AL::ALValue::array(-0.0813440, AL::ALValue::array(3, -0.266667, -0.00285404), AL::ALValue::array(3, 0.300000, 0.00321080));
  times[8][7] = 6.50000;
  keys[8][7] = AL::ALValue::array(-0.0690719, AL::ALValue::array(3, -0.300000, -0.00920402), AL::ALValue::array(3, 0.300000, 0.00920402));
  times[8][8] = 7.40000;
  keys[8][8] = AL::ALValue::array(-0.0261199, AL::ALValue::array(3, -0.300000, -0.0139375), AL::ALValue::array(3, 0.200000, 0.00929167));
  times[8][9] = 8.00000;
  keys[8][9] = AL::ALValue::array(0.000615569, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.200000, 0.00000));
  times[8][10] = 8.60000;
  keys[8][10] = AL::ALValue::array(0.000615569, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.633333, 0.00000));
  times[8][11] = 10.5000;
  keys[8][11] = AL::ALValue::array(0.912689, AL::ALValue::array(3, -0.633333, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

  names[9] = "RElbowRoll";
  times[9].arraySetSize(12);
  keys[9].arraySetSize(12);

  times[9][0] = 1.00000;
  keys[9][0] = AL::ALValue::array(0.710284, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.333333, 0.00000));
  times[9][1] = 2.00000;
  keys[9][1] = AL::ALValue::array(0.0349066, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.200000, 0.00000));
  times[9][2] = 2.60000;
  keys[9][2] = AL::ALValue::array(1.54462, AL::ALValue::array(3, -0.200000, -0.0104898), AL::ALValue::array(3, 0.333333, 0.0174830));
  times[9][3] = 3.60000;
  keys[9][3] = AL::ALValue::array(1.56210, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.233333, 0.00000));
  times[9][4] = 4.30000;
  keys[9][4] = AL::ALValue::array(0.698132, AL::ALValue::array(3, -0.233333, 0.302050), AL::ALValue::array(3, 0.166667, -0.215750));
  times[9][5] = 4.80000;
  keys[9][5] = AL::ALValue::array(0.00869999, AL::ALValue::array(3, -0.166667, -0.00000), AL::ALValue::array(3, 0.266667, 0.00000));
  times[9][6] = 5.60000;
  keys[9][6] = AL::ALValue::array(0.0966839, AL::ALValue::array(3, -0.266667, -0.00000), AL::ALValue::array(3, 0.300000, 0.00000));
  times[9][7] = 6.50000;
  keys[9][7] = AL::ALValue::array(0.0215180, AL::ALValue::array(3, -0.300000, -0.00000), AL::ALValue::array(3, 0.300000, 0.00000));
  times[9][8] = 7.40000;
  keys[9][8] = AL::ALValue::array(0.190258, AL::ALValue::array(3, -0.300000, -0.0864535), AL::ALValue::array(3, 0.200000, 0.0576357));
  times[9][9] = 8.00000;
  keys[9][9] = AL::ALValue::array(0.453786, AL::ALValue::array(3, -0.200000, -0.0616156), AL::ALValue::array(3, 0.200000, 0.0616156));
  times[9][10] = 8.60000;
  keys[9][10] = AL::ALValue::array(0.559952, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.633333, 0.00000));
  times[9][11] = 10.5000;
  keys[9][11] = AL::ALValue::array(0.380475, AL::ALValue::array(3, -0.633333, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

 
  names[10] = "LHipYawPitch";
  times[10].arraySetSize(11);
  keys[10].arraySetSize(11);

  times[10][0] = 1.00000;
  keys[10][0] = AL::ALValue::array(-0.470897, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.333333, 0.00000));
  times[10][1] = 2.00000;
  keys[10][1] = AL::ALValue::array(0.00000, AL::ALValue::array(3, -0.333333, -0.00642664), AL::ALValue::array(3, 0.400000, 0.00771196));
  times[10][2] = 3.20000;
  keys[10][2] = AL::ALValue::array(0.00771196, AL::ALValue::array(3, -0.400000, -0.00000), AL::ALValue::array(3, 0.133333, 0.00000));
  times[10][3] = 3.60000;
  keys[10][3] = AL::ALValue::array(-4.76838e-07, AL::ALValue::array(3, -0.133333, 0.00771244), AL::ALValue::array(3, 0.233333, -0.0134968));
  times[10][4] = 4.30000;
  keys[10][4] = AL::ALValue::array(-0.654977, AL::ALValue::array(3, -0.233333, -0.00000), AL::ALValue::array(3, 0.433333, 0.00000));
  times[10][5] = 5.60000;
  keys[10][5] = AL::ALValue::array(-0.498508, AL::ALValue::array(3, -0.433333, -0.00000), AL::ALValue::array(3, 0.300000, 0.00000));
  times[10][6] = 6.50000;
  keys[10][6] = AL::ALValue::array(-0.858999, AL::ALValue::array(3, -0.300000, -0.00000), AL::ALValue::array(3, 0.300000, 0.00000));
  times[10][7] = 7.40000;
  keys[10][7] = AL::ALValue::array(-0.696386, AL::ALValue::array(3, -0.300000, -0.0913488), AL::ALValue::array(3, 0.200000, 0.0608992));
  times[10][8] = 8.00000;
  keys[10][8] = AL::ALValue::array(-0.402255, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.200000, 0.00000));
  times[10][9] = 8.60000;
  keys[10][9] = AL::ALValue::array(-0.402255, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.633333, 0.00000));
  times[10][10] = 10.5000;
  keys[10][10] = AL::ALValue::array(-0.230059, AL::ALValue::array(3, -0.633333, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));
 
  names[11] = "LHipRoll";
  times[11].arraySetSize(12);
  keys[11].arraySetSize(12);

  times[11][0] = 1.00000;
  keys[11][0] = AL::ALValue::array(0.0445279, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.333333, 0.00000));
  times[11][1] = 2.00000;
  keys[11][1] = AL::ALValue::array(1.56923e-07, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.400000, 0.00000));
  times[11][2] = 3.20000;
  keys[11][2] = AL::ALValue::array(0.0583340, AL::ALValue::array(3, -0.400000, -0.00000), AL::ALValue::array(3, 0.133333, 0.00000));
  times[11][3] = 3.60000;
  keys[11][3] = AL::ALValue::array(1.56923e-07, AL::ALValue::array(3, -0.133333, 0.0149232), AL::ALValue::array(3, 0.233333, -0.0261155));
  times[11][4] = 4.30000;
  keys[11][4] = AL::ALValue::array(-0.0647821, AL::ALValue::array(3, -0.233333, -0.00000), AL::ALValue::array(3, 0.166667, 0.00000));
  times[11][5] = 4.80000;
  keys[11][5] = AL::ALValue::array(0.541052, AL::ALValue::array(3, -0.166667, -0.00000), AL::ALValue::array(3, 0.266667, 0.00000));
  times[11][6] = 5.60000;
  keys[11][6] = AL::ALValue::array(0.154976, AL::ALValue::array(3, -0.266667, 0.112296), AL::ALValue::array(3, 0.300000, -0.126333));
  times[11][7] = 6.50000;
  keys[11][7] = AL::ALValue::array(-0.174835, AL::ALValue::array(3, -0.300000, -0.00000), AL::ALValue::array(3, 0.300000, 0.00000));
  times[11][8] = 7.40000;
  keys[11][8] = AL::ALValue::array(0.00924597, AL::ALValue::array(3, -0.300000, -0.0733642), AL::ALValue::array(3, 0.200000, 0.0489095));
  times[11][9] = 8.00000;
  keys[11][9] = AL::ALValue::array(0.191986, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.200000, 0.00000));
  times[11][10] = 8.60000;
  keys[11][10] = AL::ALValue::array(0.174533, AL::ALValue::array(3, -0.200000, 0.00320627), AL::ALValue::array(3, 0.633333, -0.0101532));
  times[11][11] = 10.5000;
  keys[11][11] = AL::ALValue::array(0.151908, AL::ALValue::array(3, -0.633333, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

  names[12] ="LHipPitch";
  times[12].arraySetSize(12);
  keys[12].arraySetSize(12);

  times[12][0] = 1.00000;
  keys[12][0] = AL::ALValue::array(0.250085, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.333333, 0.00000));
  times[12][1] = 2.00000;
  keys[12][1] = AL::ALValue::array(-0.174533, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.400000, 0.00000));
  times[12][2] = 3.20000;
  keys[12][2] = AL::ALValue::array(0.374338, AL::ALValue::array(3, -0.400000, -0.00000), AL::ALValue::array(3, 0.133333, 0.00000));
  times[12][3] = 3.60000;
  keys[12][3] = AL::ALValue::array(-0.174533, AL::ALValue::array(3, -0.133333, 0.188172), AL::ALValue::array(3, 0.233333, -0.329302));
  times[12][4] = 4.30000;
  keys[12][4] = AL::ALValue::array(-1.17808, AL::ALValue::array(3, -0.233333, 0.271496), AL::ALValue::array(3, 0.166667, -0.193925));
  times[12][5] = 4.80000;
  keys[12][5] = AL::ALValue::array(-1.57080, AL::ALValue::array(3, -0.166667, -0.00000), AL::ALValue::array(3, 0.266667, 0.00000));
  times[12][6] = 5.60000;
  keys[12][6] = AL::ALValue::array(-0.857465, AL::ALValue::array(3, -0.266667, -0.237503), AL::ALValue::array(3, 0.300000, 0.267191));
  times[12][7] = 6.50000;
  keys[12][7] = AL::ALValue::array(-0.0567160, AL::ALValue::array(3, -0.300000, -0.00000), AL::ALValue::array(3, 0.300000, 0.00000));
  times[12][8] = 7.40000;
  keys[12][8] = AL::ALValue::array(-0.455555, AL::ALValue::array(3, -0.300000, 0.159699), AL::ALValue::array(3, 0.200000, -0.106466));
  times[12][9] = 8.00000;
  keys[12][9] = AL::ALValue::array(-0.855211, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.200000, 0.00000));
  times[12][10] = 8.60000;
  keys[12][10] = AL::ALValue::array(-0.835988, AL::ALValue::array(3, -0.200000, -0.0192230), AL::ALValue::array(3, 0.633333, 0.0608729));
  times[12][11] = 10.5000;
  keys[12][11] = AL::ALValue::array(0.213269, AL::ALValue::array(3, -0.633333, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

  names[13] ="LKneePitch";
  times[13].arraySetSize(12);
  keys[13].arraySetSize(12);

  times[13][0] = 1.00000;
  keys[13][0] = AL::ALValue::array(0.0919980, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.333333, 0.00000));
  times[13][1] = 2.00000;
  keys[13][1] = AL::ALValue::array(1.67552, AL::ALValue::array(3, -0.333333, -0.287267), AL::ALValue::array(3, 0.400000, 0.344720));
  times[13][2] = 3.20000;
  keys[13][2] = AL::ALValue::array(2.02024, AL::ALValue::array(3, -0.400000, -0.00000), AL::ALValue::array(3, 0.133333, 0.00000));
  times[13][3] = 3.60000;
  keys[13][3] = AL::ALValue::array(1.67552, AL::ALValue::array(3, -0.133333, 0.114239), AL::ALValue::array(3, 0.233333, -0.199918));
  times[13][4] = 4.30000;
  keys[13][4] = AL::ALValue::array(1.07777, AL::ALValue::array(3, -0.233333, -0.00000), AL::ALValue::array(3, 0.166667, 0.00000));
  times[13][5] = 4.80000;
  keys[13][5] = AL::ALValue::array(1.67552, AL::ALValue::array(3, -0.166667, -0.132434), AL::ALValue::array(3, 0.266667, 0.211894));
  times[13][6] = 5.60000;
  keys[13][6] = AL::ALValue::array(2.11075, AL::ALValue::array(3, -0.266667, -0.00134981), AL::ALValue::array(3, 0.300000, 0.00151853));
  times[13][7] = 6.50000;
  keys[13][7] = AL::ALValue::array(2.11227, AL::ALValue::array(3, -0.300000, -0.000261789), AL::ALValue::array(3, 0.300000, 0.000261789));
  times[13][8] = 7.40000;
  keys[13][8] = AL::ALValue::array(2.11253, AL::ALValue::array(3, -0.300000, -0.00000), AL::ALValue::array(3, 0.200000, 0.00000));
  times[13][9] = 8.00000;
  keys[13][9] = AL::ALValue::array(2.11253, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.200000, 0.00000));
  times[13][10] = 8.60000;
  keys[13][10] = AL::ALValue::array(2.11253, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.633333, 0.00000));
  times[13][11] = 10.5000;
  keys[13][11] = AL::ALValue::array(-0.0890139, AL::ALValue::array(3, -0.633333, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

  names[14] = "LAnklePitch";
  times[14].arraySetSize(12);
  keys[14].arraySetSize(12);

  times[14][0] = 1.00000;
  keys[14][0] = AL::ALValue::array(0.825249, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.333333, 0.00000));
  times[14][1] = 2.00000;
  keys[14][1] = AL::ALValue::array(0.244346, AL::ALValue::array(3, -0.333333, 0.193842), AL::ALValue::array(3, 0.400000, -0.232610));
  times[14][2] = 3.20000;
  keys[14][2] = AL::ALValue::array(-0.454107, AL::ALValue::array(3, -0.400000, -0.00000), AL::ALValue::array(3, 0.133333, 0.00000));
  times[14][3] = 3.60000;
  keys[14][3] = AL::ALValue::array(0.244346, AL::ALValue::array(3, -0.133333, -0.137162), AL::ALValue::array(3, 0.233333, 0.240033));
  times[14][4] = 4.30000;
  keys[14][4] = AL::ALValue::array(0.677476, AL::ALValue::array(3, -0.233333, -0.00000), AL::ALValue::array(3, 0.166667, 0.00000));
  times[14][5] = 4.80000;
  keys[14][5] = AL::ALValue::array(0.663225, AL::ALValue::array(3, -0.166667, 0.0142506), AL::ALValue::array(3, 0.266667, -0.0228010));
  times[14][6] = 5.60000;
  keys[14][6] = AL::ALValue::array(-0.455639, AL::ALValue::array(3, -0.266667, 0.276813), AL::ALValue::array(3, 0.300000, -0.311414));
  times[14][7] = 6.50000;
  keys[14][7] = AL::ALValue::array(-1.10145, AL::ALValue::array(3, -0.300000, 0.0880605), AL::ALValue::array(3, 0.300000, -0.0880605));
  times[14][8] = 7.40000;
  keys[14][8] = AL::ALValue::array(-1.18952, AL::ALValue::array(3, -0.300000, -0.00000), AL::ALValue::array(3, 0.200000, 0.00000));
  times[14][9] = 8.00000;
  keys[14][9] = AL::ALValue::array(-1.18952, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.200000, 0.00000));
  times[14][10] = 8.60000;
  keys[14][10] = AL::ALValue::array(-1.18952, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.633333, 0.00000));
  times[14][11] = 10.5000;
  keys[14][11] = AL::ALValue::array(0.0812601, AL::ALValue::array(3, -0.633333, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

  names[15] = "LAnkleRoll";
  times[15].arraySetSize(12);
  keys[15].arraySetSize(12);

  times[15][0] = 1.00000;
  keys[15][0] = AL::ALValue::array(-0.0337060, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.333333, 0.00000));
  times[15][1] = 2.00000;
  keys[15][1] = AL::ALValue::array(0.00000, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.400000, 0.00000));
  times[15][2] = 3.20000;
  keys[15][2] = AL::ALValue::array(-0.0337060, AL::ALValue::array(3, -0.400000, -0.00000), AL::ALValue::array(3, 0.133333, 0.00000));
  times[15][3] = 3.60000;
  keys[15][3] = AL::ALValue::array(0.00000, AL::ALValue::array(3, -0.133333, -0.0248232), AL::ALValue::array(3, 0.233333, 0.0434406));
  times[15][4] = 4.30000;
  keys[15][4] = AL::ALValue::array(0.171085, AL::ALValue::array(3, -0.233333, -0.00000), AL::ALValue::array(3, 0.166667, 0.00000));
  times[15][5] = 4.80000;
  keys[15][5] = AL::ALValue::array(-0.104720, AL::ALValue::array(3, -0.166667, 0.0726687), AL::ALValue::array(3, 0.266667, -0.116270));
  times[15][6] = 5.60000;
  keys[15][6] = AL::ALValue::array(-0.395731, AL::ALValue::array(3, -0.266667, -0.00000), AL::ALValue::array(3, 0.300000, 0.00000));
  times[15][7] = 6.50000;
  keys[15][7] = AL::ALValue::array(-0.0996681, AL::ALValue::array(3, -0.300000, -0.0731208), AL::ALValue::array(3, 0.300000, 0.0731208));
  times[15][8] = 7.40000;
  keys[15][8] = AL::ALValue::array(0.0429939, AL::ALValue::array(3, -0.300000, -0.0435546), AL::ALValue::array(3, 0.200000, 0.0290364));
  times[15][9] = 8.00000;
  keys[15][9] = AL::ALValue::array(0.118105, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.200000, 0.00000));
  times[15][10] = 8.60000;
  keys[15][10] = AL::ALValue::array(0.0872665, AL::ALValue::array(3, -0.200000, 0.0203671), AL::ALValue::array(3, 0.633333, -0.0644959));
  times[15][11] = 10.5000;
  keys[15][11] = AL::ALValue::array(-0.136484, AL::ALValue::array(3, -0.633333, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

  names[16] = "RHipRoll";
  times[16].arraySetSize(12);
  keys[16].arraySetSize(12);

  times[16][0] = 1.00000;
  keys[16][0] = AL::ALValue::array(-0.148756, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.333333, 0.00000));
  times[16][1] = 2.00000;
  keys[16][1] = AL::ALValue::array(0.00000, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.400000, 0.00000));
  times[16][2] = 3.20000;
  keys[16][2] = AL::ALValue::array(-0.0229680, AL::ALValue::array(3, -0.400000, -0.00000), AL::ALValue::array(3, 0.133333, 0.00000));
  times[16][3] = 3.60000;
  keys[16][3] = AL::ALValue::array(0.00000, AL::ALValue::array(3, -0.133333, -0.0102865), AL::ALValue::array(3, 0.233333, 0.0180013));
  times[16][4] = 4.30000;
  keys[16][4] = AL::ALValue::array(0.0618955, AL::ALValue::array(3, -0.233333, -0.00000), AL::ALValue::array(3, 0.166667, 0.00000));
  times[16][5] = 4.80000;
  keys[16][5] = AL::ALValue::array(-0.541052, AL::ALValue::array(3, -0.166667, 0.0108014), AL::ALValue::array(3, 0.266667, -0.0172823));
  times[16][6] = 5.60000;
  keys[16][6] = AL::ALValue::array(-0.558334, AL::ALValue::array(3, -0.266667, 0.0108922), AL::ALValue::array(3, 0.300000, -0.0122538));
  times[16][7] = 6.50000;
  keys[16][7] = AL::ALValue::array(-0.610490, AL::ALValue::array(3, -0.300000, 0.0113732), AL::ALValue::array(3, 0.300000, -0.0113732));
  times[16][8] = 7.40000;
  keys[16][8] = AL::ALValue::array(-0.626573, AL::ALValue::array(3, -0.300000, -0.00000), AL::ALValue::array(3, 0.200000, 0.00000));
  times[16][9] = 8.00000;
  keys[16][9] = AL::ALValue::array(-0.296706, AL::ALValue::array(3, -0.200000, -0.101520), AL::ALValue::array(3, 0.200000, 0.101520));
  times[16][10] = 8.60000;
  keys[16][10] = AL::ALValue::array(-0.0174533, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.633333, 0.00000));
  times[16][11] = 10.5000;
  keys[16][11] = AL::ALValue::array(-0.0628521, AL::ALValue::array(3, -0.633333, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

  names[17] = "RHipPitch";
  times[17].arraySetSize(12);
  keys[17].arraySetSize(12);

  times[17][0] = 1.00000;
  keys[17][0] = AL::ALValue::array(0.185572, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.333333, 0.00000));
  times[17][1] = 2.00000;
  keys[17][1] = AL::ALValue::array(-0.174533, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.400000, 0.00000));
  times[17][2] = 3.20000;
  keys[17][2] = AL::ALValue::array(0.282215, AL::ALValue::array(3, -0.400000, -0.00000), AL::ALValue::array(3, 0.133333, 0.00000));
  times[17][3] = 3.60000;
  keys[17][3] = AL::ALValue::array(-0.174533, AL::ALValue::array(3, -0.133333, 0.175366), AL::ALValue::array(3, 0.233333, -0.306891));
  times[17][4] = 4.30000;
  keys[17][4] = AL::ALValue::array(-1.16456, AL::ALValue::array(3, -0.233333, 0.271496), AL::ALValue::array(3, 0.166667, -0.193925));
  times[17][5] = 4.80000;
  keys[17][5] = AL::ALValue::array(-1.57080, AL::ALValue::array(3, -0.166667, -0.00000), AL::ALValue::array(3, 0.266667, 0.00000));
  times[17][6] = 5.60000;
  keys[17][6] = AL::ALValue::array(-1.52484, AL::ALValue::array(3, -0.266667, -0.00000), AL::ALValue::array(3, 0.300000, 0.00000));
  times[17][7] = 6.50000;
  keys[17][7] = AL::ALValue::array(-1.56012, AL::ALValue::array(3, -0.300000, -0.00000), AL::ALValue::array(3, 0.300000, 0.00000));
  times[17][8] = 7.40000;
  keys[17][8] = AL::ALValue::array(-1.02974, AL::ALValue::array(3, -0.300000, -0.130859), AL::ALValue::array(3, 0.200000, 0.0872392));
  times[17][9] = 8.00000;
  keys[17][9] = AL::ALValue::array(-0.905826, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.200000, 0.00000));
  times[17][10] = 8.60000;
  keys[17][10] = AL::ALValue::array(-0.905826, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.633333, 0.00000));
  times[17][11] = 10.5000;
  keys[17][11] = AL::ALValue::array(0.208583, AL::ALValue::array(3, -0.633333, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

  names[18] = "RKneePitch";
  times[18].arraySetSize(12);
  keys[18].arraySetSize(12);

  times[18][0] = 1.00000;
  keys[18][0] = AL::ALValue::array(0.191792, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.333333, 0.00000));
  times[18][1] = 2.00000;
  keys[18][1] = AL::ALValue::array(1.67552, AL::ALValue::array(3, -0.333333, -0.277978), AL::ALValue::array(3, 0.400000, 0.333574));
  times[18][2] = 3.20000;
  keys[18][2] = AL::ALValue::array(2.02645, AL::ALValue::array(3, -0.400000, -0.00000), AL::ALValue::array(3, 0.133333, 0.00000));
  times[18][3] = 3.60000;
  keys[18][3] = AL::ALValue::array(1.67552, AL::ALValue::array(3, -0.133333, 0.113041), AL::ALValue::array(3, 0.233333, -0.197823));
  times[18][4] = 4.30000;
  keys[18][4] = AL::ALValue::array(1.09386, AL::ALValue::array(3, -0.233333, -0.00000), AL::ALValue::array(3, 0.166667, 0.00000));
  times[18][5] = 4.80000;
  keys[18][5] = AL::ALValue::array(1.67552, AL::ALValue::array(3, -0.166667, -0.00000), AL::ALValue::array(3, 0.266667, 0.00000));
  times[18][6] = 5.60000;
  keys[18][6] = AL::ALValue::array(1.22111, AL::ALValue::array(3, -0.266667, 0.0902895), AL::ALValue::array(3, 0.300000, -0.101576));
  times[18][7] = 6.50000;
  keys[18][7] = AL::ALValue::array(1.09992, AL::ALValue::array(3, -0.300000, 0.0160710), AL::ALValue::array(3, 0.300000, -0.0160710));
  times[18][8] = 7.40000;
  keys[18][8] = AL::ALValue::array(1.08385, AL::ALValue::array(3, -0.300000, 0.0160710), AL::ALValue::array(3, 0.200000, -0.0107140));
  times[18][9] = 8.00000;
  keys[18][9] = AL::ALValue::array(0.876155, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.200000, 0.00000));
  times[18][10] = 8.60000;
  keys[18][10] = AL::ALValue::array(1.76278, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.633333, 0.00000));
  times[18][11] = 10.5000;
  keys[18][11] = AL::ALValue::array(-0.0766580, AL::ALValue::array(3, -0.633333, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

  names[19] = "RAnklePitch";
  times[19].arraySetSize(12);
  keys[19].arraySetSize(12);

  times[19][0] = 1.00000;
  keys[19][0] = AL::ALValue::array(0.466378, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.333333, 0.00000));
  times[19][1] = 2.00000;
  keys[19][1] = AL::ALValue::array(0.244346, AL::ALValue::array(3, -0.333333, 0.127369), AL::ALValue::array(3, 0.400000, -0.152842));
  times[19][2] = 3.20000;
  keys[19][2] = AL::ALValue::array(-0.374254, AL::ALValue::array(3, -0.400000, -0.00000), AL::ALValue::array(3, 0.133333, 0.00000));
  times[19][3] = 3.60000;
  keys[19][3] = AL::ALValue::array(0.244346, AL::ALValue::array(3, -0.133333, -0.132318), AL::ALValue::array(3, 0.233333, 0.231556));
  times[19][4] = 4.30000;
  keys[19][4] = AL::ALValue::array(0.717365, AL::ALValue::array(3, -0.233333, -0.00000), AL::ALValue::array(3, 0.166667, 0.00000));
  times[19][5] = 4.80000;
  keys[19][5] = AL::ALValue::array(0.663225, AL::ALValue::array(3, -0.166667, -0.00000), AL::ALValue::array(3, 0.266667, 0.00000));
  times[19][6] = 5.60000;
  keys[19][6] = AL::ALValue::array(0.785451, AL::ALValue::array(3, -0.266667, -0.00000), AL::ALValue::array(3, 0.300000, 0.00000));
  times[19][7] = 6.50000;
  keys[19][7] = AL::ALValue::array(0.783916, AL::ALValue::array(3, -0.300000, 0.00153411), AL::ALValue::array(3, 0.300000, -0.00153411));
  times[19][8] = 7.40000;
  keys[19][8] = AL::ALValue::array(0.441568, AL::ALValue::array(3, -0.300000, 0.0575958), AL::ALValue::array(3, 0.200000, -0.0383972));
  times[19][9] = 8.00000;
  keys[19][9] = AL::ALValue::array(0.403171, AL::ALValue::array(3, -0.200000, 0.0383972), AL::ALValue::array(3, 0.200000, -0.0383972));
  times[19][10] = 8.60000;
  keys[19][10] = AL::ALValue::array(-0.579449, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.633333, 0.00000));
  times[19][11] = 10.5000;
  keys[19][11] = AL::ALValue::array(0.0813440, AL::ALValue::array(3, -0.633333, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

  names[20] = "RAnkleRoll";
  times[20].arraySetSize(12);
  keys[20].arraySetSize(12);

  times[20][0] = 1.00000;
  keys[20][0] = AL::ALValue::array(0.167248, AL::ALValue::array(3, -0.333333, -0.00000), AL::ALValue::array(3, 0.333333, 0.00000));
  times[20][1] = 2.00000;
  keys[20][1] = AL::ALValue::array(8.63852e-08, AL::ALValue::array(3, -0.333333, 0.0313773), AL::ALValue::array(3, 0.400000, -0.0376527));
  times[20][2] = 3.20000;
  keys[20][2] = AL::ALValue::array(-0.0398420, AL::ALValue::array(3, -0.400000, -0.00000), AL::ALValue::array(3, 0.133333, 0.00000));
  times[20][3] = 3.60000;
  keys[20][3] = AL::ALValue::array(8.63852e-08, AL::ALValue::array(3, -0.133333, -0.00000), AL::ALValue::array(3, 0.233333, 0.00000));
  times[20][4] = 4.30000;
  keys[20][4] = AL::ALValue::array(-0.160838, AL::ALValue::array(3, -0.233333, -0.00000), AL::ALValue::array(3, 0.166667, 0.00000));
  times[20][5] = 4.80000;
  keys[20][5] = AL::ALValue::array(0.104720, AL::ALValue::array(3, -0.166667, -0.00000), AL::ALValue::array(3, 0.266667, 0.00000));
  times[20][6] = 5.60000;
  keys[20][6] = AL::ALValue::array(0.00924597, AL::ALValue::array(3, -0.266667, 0.0318202), AL::ALValue::array(3, 0.300000, -0.0357977));
  times[20][7] = 6.50000;
  keys[20][7] = AL::ALValue::array(-0.0981341, AL::ALValue::array(3, -0.300000, -0.00000), AL::ALValue::array(3, 0.300000, 0.00000));
  times[20][8] = 7.40000;
  keys[20][8] = AL::ALValue::array(0.443314, AL::ALValue::array(3, -0.300000, -0.155547), AL::ALValue::array(3, 0.200000, 0.103698));
  times[20][9] = 8.00000;
  keys[20][9] = AL::ALValue::array(0.679603, AL::ALValue::array(3, -0.200000, -0.00000), AL::ALValue::array(3, 0.200000, 0.00000));
  times[20][10] = 8.60000;
  keys[20][10] = AL::ALValue::array(0.277507, AL::ALValue::array(3, -0.200000, 0.0493334), AL::ALValue::array(3, 0.633333, -0.156222));
  times[20][11] = 10.5000;
  keys[20][11] = AL::ALValue::array(0.0629359, AL::ALValue::array(3, -0.633333, -0.00000), AL::ALValue::array(3, 0.00000, 0.00000));

  std::cout << "send carlos back getup " << std::endl << std::flush;
  motion_proxy_->post.angleInterpolationBezier(names, times, keys);

}
