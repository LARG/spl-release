#include "VisionCore.h"


#include <common/States.h>

#include <memory/BodyModelBlock.h>
#include <memory/CameraBlock.h>
#include <memory/FrameInfoBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/JointBlock.h>
#include <memory/KickRequestBlock.h>
#include <memory/OdometryBlock.h>
#include <memory/SensorBlock.h>
#include <memory/WalkRequestBlock.h>
#include <memory/JointCommandBlock.h>
#include <memory/ProcessedSonarBlock.h>
#include <memory/KickParamBlock.h>
#include <memory/WalkParamBlock.h>
#include <memory/ALWalkParamBlock.h>
#include <memory/WalkInfoBlock.h>
#include <memory/RobotStateBlock.h>
#include <memory/RobotVisionBlock.h>
#include <memory/WorldObjectBlock.h>

#include <communications/CommunicationModule.h>
#include <kinematics/KinematicsModule.h>
#include <sensor/SensorModule.h>
#include <vision/VisionModule.h>
#include <lights/LEDModule.h>
#include <localization/ParallelLocalizationModule.h>
#include <localization/PerfectLocalizationModule.h>
#include <opponents/OppModule.h>
#include <behavior/BehaviorModule.h>
#include <sensor/ButtonModule.h>
#include <audio/AudioModule.h>
#include <camera/ImageCapture.h>
#include <python/PythonInterface.h>
#include <common/RobotConfig.h>
#include <common/Kicks.h>

#include <iostream>
#include <chrono>
#include <boost/thread/barrier.hpp>
#include <pthread.h>

boost::barrier do_logging_(2); // do_logging tells the logging thread to log the current frame

VisionCore *VisionCore::inst_ = NULL;
LocalizationMethod::Type LocalizationMethod::DEFAULT = LocalizationMethod::Parallel;

bool VisionCore::isStreaming() {
  if(inst_ == nullptr) return false;
  if(inst_->communications_ == nullptr) return false;
  return inst_->communications_->streaming();
}

bool VisionCore::EnableOptimizations() {
#ifdef OPTIMIZE
  return true;
#else
  return false;
#endif
}

VisionCore::VisionCore (CoreType type, bool use_shared_memory, int team_num, int player_num, LocalizationMethod::Type locMethod):
  memory_(new MemoryFrame(use_shared_memory,MemoryOwner::VISION,team_num,player_num,false)),
  delete_memory_on_destruct_(true),
  type_(type),
  last_frame_processed_(0),
  // modules
  communications_(NULL),
  interpreter_(NULL),
  vision_(NULL),
  localization_(NULL),
  opponents_(NULL),
  perfect_localization_(NULL),
  behavior_(NULL),
  buttons_(NULL),
  leds_(NULL),
  audio_(NULL),
  log_(new FileLogWriter()),
  textlog_(std::make_unique<TextLogger>()),
  image_capture_(NULL),
  // sync memory
  sync_joint_angles_(NULL),
  sync_kick_request_(NULL),
  sync_odometry_(NULL),
  sync_sensors_(NULL),
  sync_walk_request_(NULL),
  sync_walk_response_(NULL),
  sync_joint_commands_(NULL),
  sync_processed_sonar_(NULL),
  sync_kick_params_(NULL),
  sync_walk_param_(NULL),
  sync_al_walk_param_(NULL),
  sync_walk_info_(NULL),
  frames_to_log_(0),
  disable_log_(false),
  is_logging_(false),
  logging_thread_(std::thread(&VisionCore::log_listener_, this)),
  log_in_progress_(false)
{
  init(team_num, player_num);
  initModules(locMethod);
  vtimer_.setInterval(30 * 5);
  camtimer_.setInterval(30 * 5);
  atimer_.setInterval(30 * 5);
  auto_wb_flag_ = false;

  pthread_attr_t tattr;
  int ret = pthread_attr_init (&tattr);
  int thread_prio = 0;
  int policy = 0;
  int min_prio_for_policy = 0, max_prio_for_policy = 0;
  sched_param param;

  pthread_attr_getschedpolicy(&tattr, &policy);
  min_prio_for_policy = sched_get_priority_min(policy);
  max_prio_for_policy = sched_get_priority_max(policy);

  pthread_setschedprio(logging_thread_.native_handle(), min_prio_for_policy);
}

VisionCore::~VisionCore() {
  // stop walking
  if ((sync_walk_request_ != NULL)  && (sync_walk_request_->motion_ == WalkRequestBlock::WALK)) {
    sync_walk_request_->stand();
  }

  // clean up modules
  if (interpreter_ != NULL)
    delete interpreter_;
  if (vision_ != NULL)
    delete vision_;
  if (leds_ != NULL)
    delete leds_;
  if (audio_ != NULL)
    delete audio_;
  if (localization_ != NULL)
    delete localization_;
  if (opponents_ != NULL)
    delete opponents_;
  if (perfect_localization_ != NULL)
    delete perfect_localization_;
  if (communications_ != NULL)
    delete communications_;
  if (behavior_ != NULL)
    delete behavior_;
  if (buttons_ != NULL)
    delete buttons_;

  // clean up the entire memory block
  if (delete_memory_on_destruct_ && memory_ != NULL)
    delete memory_;

  if (image_capture_ != NULL)
    delete image_capture_;
}

void VisionCore::processVisionFrame() {


  vtimer_.start();
  camtimer_.start();
  preVision();
  optionallyWriteLog();
  interpreter_->processFrame(); // main control is done in the interpreter

  // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  
  // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  
  // std::cout << "OptionallyWriteLog elapsed time (ms) = " <<  (std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()) /1000.0  <<std::endl;

  if (communications_ != NULL && interpreter_->is_ok_) {
    communications_->processFrame();
  }
  // atimer_.start();
  // if(game_state_->state() == SET || game_state_->state() == INITIAL)

  // DO NOT TURN this off. We check whether Audio Processing is necessary in AudioModule.
  // This part is necessary to reset whistle detection
  audio_->processFrame();
  // atimer_.stop();

  if (buttons_ !=NULL) {
    buttons_->processButtons(); // check for any button related changes
  }

  // Enable Auto White Balance
  if (camera_info_->calibrate_white_balance_ && !auto_wb_flag_){
//    image_capture_->enableAutoWB();
//    auto_wb_flag_ = true;             // SN 5/18: Turning this off while we look into how to deal with wb properly
  }

  if (!camera_info_->calibrate_white_balance_ && auto_wb_flag_){
    image_capture_->lockWB();
    auto_wb_flag_ = false;
  }

  auto& frame_id = vision_frame_info_->frame_id;
  last_frame_processed_ = frame_id;

  // vtimer_.pause();
  // optionallyWriteLog();
  if (communications_ != NULL)
    communications_->optionallyStream();
  // vtimer_.unpause();

  if(is_logging_ && disable_log_) {
    disable_log_ = false;
    disableLogging();
    disableTextLogging();
    if(communications_) {
      communications_->sendToolResponse(ToolPacket::LogComplete);
    }
  }
  postVision();
  vtimer_.stop();
  camtimer_.stop();
  if(vtimer_.ready() && interpreter_->is_ok_) {
#ifndef TOOL
    printf("Vision: %2.2f Hz [%2.2f ms] (capped by %2.2f Hz [%2.2f ms] with camera)\n", 
      vtimer_.avgrate(), vtimer_.avgtime_ms(), camtimer_.avgrate(), camtimer_.avgtime_ms());
    // printf("Audio: %2.2f Hz [%2.2f ms]", atimer_.avgrate(), atimer_.avgtime_ms());
#endif
  }
}

void VisionCore::setLogSelections(const ToolPacket& selections) {
  logging_selections_ = std::make_unique<ToolPacket>(selections);
}

void VisionCore::log_listener_()
{
  while(true)
  {
    // cout << "waiting to write log" << endl;
    do_logging_.wait();
    // cout << "writing log" << endl;
    log_in_progress_ = true;
    logMemory();
    if(log_by_frame_) frames_to_log_--;
    if(frames_to_log_ == 0 && log_by_frame_) disable_log_ = true;
    log_in_progress_ = false;
  }
}

void VisionCore::optionallyWriteLog() {
  if(!is_logging_)
    return;
  if ((log_interval_ > 1e-5) && (logtimer_.elapsed_s() < log_interval_)) // not long enough since log
    return;
  // bool headReady = (!robot_vision_->reported_head_moving) && (vision_frame_info_->seconds_since_start - robot_vision_->reported_head_stop_time > HEAD_STOP_THRESHOLD);
  // if ((log_interval_ > 1e-5) && !headReady && (logtimer_.elapsed_s() - log_interval_ < 0.34)) // logging by frequency, but head is moving, so let's wait a bit (at most 0.34s)
  //   return;
  if(logging_selections_ != nullptr && logging_selections_->hasRequiredObjects) {
    bool allow = false;
    for(int i = 0; i < NUM_WorldObjectTypes; i++) {
      if(logging_selections_->requiredObjects[i] && world_objects_->objects_[i].seen) {
        allow = true;
        break;
      }
    }
    if(!allow) return;
  }

  // cout << "going to log" << endl;
  if(!log_in_progress_)
    // cout << "waiting for logging" << endl;
    do_logging_.wait();

  // logMemory();
  logtimer_.restart();
  // if(log_by_frame_) frames_to_log_--;
  // if(frames_to_log_ == 0 && log_by_frame_) disable_log_ = true;
}

void VisionCore::init(int team_num, int player_num) {
  if (type_ == CORE_INIT) {
    FrameInfoBlock *frame_info;
    memory_->getBlockByName(frame_info,"raw_vision_frame_info");
    if (frame_info == NULL) {
      std::cout << "VISION CORE: ROBOT" << std::endl;
      type_ = CORE_ROBOT;
    } else {
      if (frame_info->source == MEMORY_SIM) {
        std::cout << "VISION CORE: SIM" << std::endl;
        type_ = CORE_SIM;
      } else if (frame_info->source == MEMORY_ROBOT) {
        std::cout << "VISION CORE: ROBOT" << std::endl;
        type_ = CORE_ROBOT;
      } else {
        std::cerr << "Unknown memory type when init vision core" << std::endl;
        exit(1);
      }
    }
  }

  inst_ = this;
  interpreter_ = new PythonModule(this);
  rconfig_ = std::make_unique<RobotConfig>();
  rconfig_->load(util::cfgpath(util::RobotConfig));

  if (type_ == CORE_ROBOT) {
    image_capture_ = new ImageCapture(memory_);
    image_capture_->initVision();
    std::cout << "DOING INIT VISION" << std::endl;
  }

  setMemoryVariables();

  initMemory();

  robot_state_->team_ = team_num;
  robot_state_->robot_id_ = rconfig_->robot_id;
  robot_state_->WO_SELF = robot_state_->global_index_ = player_num;
  if(type_ == CORE_TOOLSIM && team_num == TEAM_RED) {
    robot_state_->global_index_ += WO_TEAM_LAST;
  }
  robot_state_->clock_offset_ = 0;
  std::ifstream in("/home/nao/time.txt");
  if (in.good()) {
    in >> robot_state_->clock_offset_;
    in.close();
  } else {
    robot_state_->clock_offset_ = 0;
  }
  std::cout << "CLOCK OFFSET: " << robot_state_->clock_offset_ << std::endl;
}

void VisionCore::initModules(LocalizationMethod::Type locMethod) {

  if (!isToolCore()) {
    communications_ = new CommunicationModule(this);
    communications_->init(memory_,textlog_.get());
#ifndef TOOL
    // There are some circumstances where the tool will use "non-tool" core types for
    // simulation purposes, usually to gain parts of comm functionality. Comm should
    // never automatically start a TCP server while running on the tool, though, so
    // we do a final sanity check with the TOOL preprocessor definition.
    communications_->startTCPServer();
#endif
  }

  if ((type_ != CORE_TOOLSIM) && (type_ != CORE_TOOL_NO_VISION)){
    vision_ = new VisionModule();
    vision_->init(memory_,textlog_.get());
  }

  leds_ = new LEDModule();
  leds_->init(memory_,textlog_.get());

  audio_ = new AudioModule();
  audio_->init(memory_,textlog_.get());

  localization_ = new ParallelLocalizationModule();
  localization_->method = locMethod;
  localization_->init(memory_,textlog_.get());

  opponents_ = new OppModule();
  opponents_->init(memory_,textlog_.get());

  if (type_ == CORE_SIM){
    perfect_localization_ = new PerfectLocalizationModule();
    perfect_localization_->init(memory_,textlog_.get());
  }
  
  behavior_ = new BehaviorModule();
  behavior_->init(memory_,textlog_.get());

  buttons_ = new ButtonModule();
  buttons_->init(memory_,textlog_.get());

  /* MUST DO INTERPRETER LAST - OTHERWISE ALL THE MEMORY BLOCK POINTERS ARE NOT INITIALISED
     MQ 3/16/2011 */
  interpreter_->init(memory_,textlog_.get());

  if (communications_ != NULL) {
    communications_->interpreter_restart_requested_ = &(interpreter_->restart_requested_);
  }
}

void VisionCore::initMemory() {

  // create all the modules that are used
  //MemorySource mem_source = MEMORY_SIM;
  //if (type_ == CORE_ROBOT)
    //mem_source = MEMORY_ROBOT;

  // get raw info for vision interface
  memory_->getOrAddBlockByName(raw_vision_frame_info_,"raw_vision_frame_info");
  memory_->getOrAddBlockByName(raw_camera_info_,"raw_camera_info");
  
  //Add required memory blocks
  memory_->getOrAddBlockByName(camera_info_,"camera_info");
  memory_->getOrAddBlockByName(vision_frame_info_,"vision_frame_info");
  memory_->getOrAddBlockByName(game_state_,"game_state");
  memory_->getOrAddBlockByName(vision_body_model_,"vision_body_model");
  memory_->getOrAddBlockByName(vision_joint_angles_,"vision_joint_angles");
  memory_->getOrAddBlockByName(vision_kick_request_,"vision_kick_request");
  memory_->getOrAddBlockByName(vision_odometry_,"vision_odometry");
  memory_->getOrAddBlockByName(vision_sensors_,"vision_sensors");
  memory_->getOrAddBlockByName(vision_walk_request_,"vision_walk_request");
  memory_->getOrAddBlockByName(vision_walk_response_,"vision_walk_response");
  memory_->getOrAddBlockByName(vision_joint_commands_,"vision_joint_commands");
  memory_->getOrAddBlockByName(vision_processed_sonar_,"vision_processed_sonar");
  memory_->getOrAddBlockByName(vision_kick_params_,"vision_kick_params");
  memory_->getOrAddBlockByName(vision_walk_param_,"vision_walk_param");
  memory_->getOrAddBlockByName(vision_al_walk_param_,"vision_al_walk_param");
  memory_->getOrAddBlockByName(vision_walk_info_,"vision_walk_info");
  memory_->getOrAddBlockByName(robot_info_,"robot_info",MemoryOwner::SHARED);
  memory_->getOrAddBlockByName(robot_state_,"robot_state",MemoryOwner::SHARED);
  memory_->getOrAddBlockByName(robot_vision_,"robot_vision");
  memory_->getOrAddBlockByName(world_objects_,"world_objects", MemoryOwner::SHARED);

  // synchronized blocks
  if (!isToolCore()) {
    memory_->getOrAddBlockByName(sync_body_model_,"sync_body_model",MemoryOwner::SYNC);
    memory_->getOrAddBlockByName(sync_joint_angles_,"sync_joint_angles",MemoryOwner::SYNC);
    memory_->getOrAddBlockByName(sync_kick_request_,"sync_kick_request",MemoryOwner::SYNC);
    memory_->getOrAddBlockByName(sync_odometry_,"sync_odometry",MemoryOwner::SYNC);
    memory_->getOrAddBlockByName(sync_sensors_,"sync_sensors",MemoryOwner::SYNC);
    memory_->getOrAddBlockByName(sync_walk_request_,"sync_walk_request",MemoryOwner::SYNC);
    memory_->getOrAddBlockByName(sync_walk_response_,"sync_walk_response",MemoryOwner::SYNC);
    memory_->getOrAddBlockByName(sync_joint_commands_,"sync_joint_commands",MemoryOwner::SYNC);
    memory_->getOrAddBlockByName(sync_processed_sonar_,"sync_processed_sonar",MemoryOwner::SYNC);
    memory_->getOrAddBlockByName(sync_kick_params_,"sync_kick_params",MemoryOwner::SYNC);
    memory_->getOrAddBlockByName(sync_walk_param_,"sync_walk_param",MemoryOwner::SYNC);
    memory_->getOrAddBlockByName(sync_al_walk_param_,"sync_al_walk_param",MemoryOwner::SYNC);
    memory_->getOrAddBlockByName(sync_walk_info_,"sync_walk_info",MemoryOwner::SYNC);
  }

  // print out all the memory blocks we're using
  std::vector<std::string> memory_block_names;
  memory_->getBlockNames(memory_block_names,false);
  std::cout << "INITIAL MEMORY BLOCKS:" << std::endl;
  for (unsigned int i = 0; i < memory_block_names.size(); i++)
    std::cout << memory_block_names[i] << std::endl;
  std::cout << "--------------" << std::endl;

  textlog_->setFrameInfo(vision_frame_info_);
  log_->setType(type_);
  textlog_->setType(type_);
  world_objects_->resetFrames();
}

void VisionCore::logMemory() {
  log_->writeMemory(*memory_);
}

void VisionCore::enableLogging() {
  if(is_logging_) {
    std::cout << "Logging is already in progress\n";
    return;
  }
  is_logging_ = true;
  enableMemoryLogging();
  enableTextLogging();
}

void VisionCore::enableLogging(int frames, double frequency) {
  if(is_logging_) {
    std::cout << "Logging is already in progress\n";
    return;
  }
  log_by_frame_ = (frames > 0);
  frames_to_log_ = frames;
  log_interval_ = (frequency > 1e-5) ? frequency : 1.00; // To prevent it from logging too frequently
  enableLogging();
  logtimer_.start();
}

void VisionCore::enableMemoryLogging() {
  is_logging_ = true;
  log_->open("vision", true);
}

void VisionCore::enableTextLogging(const char *filename) {
  if (filename) {
    textlog_->open(filename);
  } else {
    textlog_->open("vision", true);
  }
}

void VisionCore::startDisableLogging(){
  if (is_logging_) {
    disable_log_ = true;
  } else {
    // logging is already off, but tool probably doesn't know it, send it again
    if(communications_) {
      communications_->sendToolResponse(ToolPacket::LogComplete);
    }
  }
}

void VisionCore::disableLogging() {
  if(!is_logging_) return;
  disableMemoryLogging();
  disableTextLogging();
  disable_log_ = false;
  is_logging_ = false;
  frames_to_log_ = 0;
}

void VisionCore::disableMemoryLogging() {
  log_->close();
}

void VisionCore::disableTextLogging() {
  textlog_->close();
}

void VisionCore::updateMemory(MemoryFrame* memory, bool locOnly) {
  if (memory_ && delete_memory_on_destruct_) {
    delete memory_;
  }
  delete_memory_on_destruct_ = false;
  memory_ = memory;
  setMemoryVariables();
  if (vision_ && !locOnly)
    vision_->updateModuleMemory(memory_);
  if (localization_)
    localization_->updateModuleMemory(memory_);
  if (opponents_)
    opponents_->updateModuleMemory(memory_);
  if (behavior_)
    behavior_->updateModuleMemory(memory_);
  if (buttons_ && !locOnly)
    buttons_->updateModuleMemory(memory_);
  if (leds_)
    leds_->updateModuleMemory(memory_);
  if (audio_)
    audio_->updateModuleMemory(memory_);

  if (interpreter_)
    interpreter_->updateModuleMemory(memory_);

  FrameInfoBlock *frame_info;
  memory_->getBlockByName(frame_info,"vision_frame_info");
  textlog_->setFrameInfo(frame_info);
}

void VisionCore::setMemoryVariables() {
  // Set the data path for the data folder
  memory_->data_path_ = util::cfgpath(util::Data);

  // Pass the coretype to the memory so that it can be accessed by different modules
  memory_->core_type_ = type_;
}

void VisionCore::preVision() {
  // called from interpreter befor vision module
  //std::cout << "pre vision" << std::endl << std::flush;

  // don't need to lock or receive data in tool
  if (isToolCore())
    return;

  vtimer_.pause();
  // see if we've already processed this frame
  image_capture_->requeue();
  vtimer_.unpause();

  receiveData();
}

void VisionCore::postVision() {
  // don't need unlock for tool
  if (isToolCore())
    return;
}

void VisionCore::publishData() {
  // don't publish data for tool
  if (isToolCore())
    return;

  vision_walk_request_->is_penalised_ = (game_state_->state() == PENALISED);

  memory_->motion_vision_lock_->lock();
  // copy over data to the motion process
  bool temp_kick_running = sync_kick_request_->kick_running_;
  bool temp_finished_with_step = sync_kick_request_->finished_with_step_;
  *sync_kick_request_ = *vision_kick_request_;
  sync_kick_request_->kick_running_ = temp_kick_running;
  sync_kick_request_->finished_with_step_ = temp_finished_with_step;
  //std::cout << "vis: " << vision_walk_request_->step_into_kick_ << std::endl;
  bool temp_finished_standing = sync_walk_request_->finished_standing_;
  *sync_walk_request_ = *vision_walk_request_;

  if(temp_finished_standing && !vision_walk_request_->start_command_)
    sync_walk_request_->finished_standing_ = temp_finished_standing;
  sync_walk_request_->start_command_ = vision_walk_request_->start_command_ = false;
  // just so motion has this info from vision
  sync_odometry_->fall_direction_ = vision_odometry_->fall_direction_;
  //std::cout << vision_walk_request_->odometry_fwd_offset_ << std::endl;
  *sync_joint_commands_ = *vision_joint_commands_;
  if (vision_kick_params_->send_params_)
    *sync_kick_params_ = *vision_kick_params_;
  if (vision_walk_param_->send_params_)
    *sync_walk_param_ = *vision_walk_param_;
  if (vision_al_walk_param_->send_params_)
    *sync_al_walk_param_ = *vision_al_walk_param_;
  memory_->motion_vision_lock_->unlock();

  // copy over data to the interface's vision thread
  camera_info_->copyToImageCapture(raw_camera_info_);
}


void VisionCore::receiveData() {
  // don't receive data for tool
  if (isToolCore())
    return;
  // SHOULD BE CALLED WHILE VISION LOCK IS HELD
  // copy over data from the interface's vision thread
  *vision_frame_info_ = *raw_vision_frame_info_;
  camera_info_->copyFromImageCapture(raw_camera_info_);

  memory_->motion_vision_lock_->lock();
  static double cum_x=0, cum_y=0, cum_rot= 0;
  if (type_ != CORE_TOOLSIM){
    // copy over data from the motion process
    *vision_body_model_ = *sync_body_model_;
    *vision_joint_angles_ = *sync_joint_angles_;
    *vision_sensors_ = *sync_sensors_;
    // odom
    *vision_odometry_ = *sync_odometry_;
    cum_x += vision_odometry_->displacement.translation.x;
    cum_y += vision_odometry_->displacement.translation.y;
    cum_rot += vision_odometry_->displacement.rotation;
    //cout << "Vision Cumul. Odometry: " << cum_x << ", " << cum_y << ", " <<cum_rot<< endl;;
    sync_odometry_->reset();
  }

  // kick request
  vision_kick_request_->kick_running_ = sync_kick_request_->kick_running_;
  vision_kick_request_->finished_with_step_ = sync_kick_request_->finished_with_step_;
  vision_kick_request_->kick_type_ = Kick::NO_KICK;
  vision_kick_request_->vision_kick_running_ = false;
  float fwdOffset = vision_walk_request_->odometry_fwd_offset_;
  float sideOffset = vision_walk_request_->odometry_side_offset_;
  float turnOffset = vision_walk_request_->odometry_turn_offset_;
  auto walk_type = vision_walk_request_->walk_type_;
  WalkControl temp_status = vision_walk_request_->walk_control_status_;
  Dive::diveTypes temp_diveStatus = vision_walk_request_->dive_type_;
  *vision_walk_request_ = *sync_walk_request_;
  *vision_walk_response_ = *sync_walk_response_;
  vision_walk_request_->odometry_fwd_offset_ = fwdOffset;
  vision_walk_request_->odometry_side_offset_ = sideOffset;
  vision_walk_request_->odometry_turn_offset_ = turnOffset;
  vision_walk_request_->walk_type_ = walk_type;
  vision_walk_request_->new_command_ = false;
  vision_walk_request_->perform_kick_ = false;
  vision_walk_request_->set_kick_step_params_ = false;
  vision_walk_request_->walk_decides_finished_with_target_ = false;

  if (temp_status == WALK_CONTROL_OFF)
    vision_walk_request_->walk_control_status_ = temp_status;
  else if (temp_status == WALK_CONTROL_SET && vision_walk_request_->walk_control_status_ == WALK_CONTROL_OFF)
    vision_walk_request_->walk_control_status_ = temp_status;
  else if (temp_status == WALK_CONTROL_DONE)
    sync_walk_request_->walk_control_status_ = temp_status;
  
  if (temp_diveStatus == Dive::NONE && sync_walk_request_->dive_type_ == Dive::DONE){
    vision_walk_request_->dive_type_ = temp_diveStatus;
  }
  
  
  vision_walk_param_->send_params_ = false;
  vision_al_walk_param_->send_params_ = false;
  vision_kick_params_->send_params_ = false;

  // walk
  *vision_walk_info_ = *sync_walk_info_;

  //sonar
  *vision_processed_sonar_ = *sync_processed_sonar_;

  // joint commands
  // Todd: no need to receive these here
  // just reset them to sending no commands
  vision_joint_commands_->send_body_angles_ = false;
  vision_joint_commands_->send_head_pitch_angle_ = false;
  vision_joint_commands_->send_head_yaw_angle_ = false;
  vision_joint_commands_->send_stiffness_ = false;
  vision_joint_commands_->send_sonar_command_ = false;
  vision_joint_commands_->send_arm_angles_  = false;

  memory_->motion_vision_lock_->unlock();

}

TextLogger* VisionCore::textlog() {
  return textlog_.get();
}

void VisionCore::motionLock() {
  if (!isToolCore())
    memory_->motion_vision_lock_->lock();
}

void VisionCore::motionUnlock() {
  if (!isToolCore())
    memory_->motion_vision_lock_->unlock();
}

bool VisionCore::isToolCore() {
  return (type_ == CORE_TOOL) || (type_ == CORE_TOOL_NO_VISION);
}
