#include "CommunicationModule.h"

#include <communications/RoboCupGameControlData.h>
#include <VisionCore.h>
#include <memory/LogWriter.h>

#include <memory/FrameInfoBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/LocalizationBlock.h>
#include <memory/RobotStateBlock.h>
#include <memory/TeamPacketsBlock.h>
#include <memory/WorldObjectBlock.h>
#include <memory/OpponentBlock.h>
#include <memory/OdometryBlock.h>
#include <memory/CameraBlock.h>
#include <memory/BehaviorBlock.h>
#include <memory/AudioProcessingBlock.h>
#include <communications/PacketConverter.h>
#include <communications/CommInfo.h>
#include <communications/TCPConnection.h>
#include <math/Common.h>

#include <netdb.h>

#include <iostream>

#include <communications/Logging.h>


#define MAX_MEM_WRITE_SIZE MAX_STREAMING_MESSAGE_LEN

#define SECONDS_PER_PACKET 2
#define PACKET_INVALID_DELAY 10
#define LOC_INVALID_DELAY 10

using namespace static_math;

bool* CommunicationModule::interpreter_restart_requested_(NULL);

CommunicationModule::CommunicationModule(VisionCore *core):
  teamUDP(NULL), toolUDP(NULL), gcDataUDP(NULL), gcReturnUDP(NULL),
  log_buffer_(NULL),
  connected_(false)
{
  core_ = core;
  vtime_ = 0;
  streaming_logger_ = std::make_unique<StreamLogWriter>();

  // Hack to get core to compile these methods since they're not used by core otherwise - JM 06/06/16 
  CommStruct s; s.setPacketsMissed(0, s.getPacketsMissed(0));
}

CommunicationModule::~CommunicationModule() {
  cleanupUDP();
  cleanupTCPServer();
  if (log_buffer_ != NULL)
    delete log_buffer_;
}

void CommunicationModule::specifyMemoryDependency() {
  requiresMemoryBlock("vision_frame_info");
  requiresMemoryBlock("game_state");
  requiresMemoryBlock("localization");
  requiresMemoryBlock("robot_state");
  requiresMemoryBlock("team_packets");
  requiresMemoryBlock("world_objects");
  requiresMemoryBlock("opponents");
  requiresMemoryBlock("vision_odometry");
  requiresMemoryBlock("camera_info");
  requiresMemoryBlock("behavior");
  requiresMemoryBlock("audio_processing");
  requiresMemoryBlock("vision_joint_commands");
}

void CommunicationModule::specifyMemoryBlocks() {
  getMemoryBlock(frame_info_,"vision_frame_info");
  getOrAddMemoryBlock(game_state_,"game_state");
  getOrAddMemoryBlock(robot_state_,"robot_state");
  getOrAddMemoryBlock(world_objects_,"world_objects");

  getOrAddMemoryBlock(localization_,"localization");
  getOrAddMemoryBlock(team_packets_,"team_packets");
  getOrAddMemoryBlock(opponents_,"opponents");
  getOrAddMemoryBlock(odometry_,"vision_odometry");
  getOrAddMemoryBlock(camera_,"camera_info");
  getOrAddMemoryBlock(behavior_,"behavior");
  getOrAddMemoryBlock(audio_processing_, "audio_processing");
  getOrAddMemoryBlock(joint_commands_,"vision_joint_commands");
}

void CommunicationModule::initSpecificModule() {
}

void CommunicationModule::cleanupUDP() {
  vector<UDPWrapper**> connections = { &teamUDP, &toolUDP, &gcDataUDP, &gcReturnUDP };
  for(auto c : connections) { 
    if(*c) delete *c;
    *c = NULL;
  }
}


void CommunicationModule::initUDP() {
  cleanupUDP();
  RobotConfig defaultConfig;
  if(CommInfo::TEAM_BROADCAST_IP == "")
    CommInfo::TEAM_BROADCAST_IP = defaultConfig.team_broadcast_ip;
  if(CommInfo::TEAM_UDP_PORT == 0)
    CommInfo::TEAM_UDP_PORT = defaultConfig.team_udp;
  teamUDP = new UDPWrapper(CommInfo::TEAM_UDP_PORT,true,CommInfo::TEAM_BROADCAST_IP);
  bool broadcast = util::endswith(CommInfo::GAME_CONTROLLER_IP, ".255");
  toolUDP = new UDPWrapper(CommInfo::TOOL_UDP_PORT,broadcast,CommInfo::TOOL_LISTEN_IP);
  gcDataUDP = new UDPWrapper(GAMECONTROLLER_DATA_PORT,broadcast,CommInfo::GAME_CONTROLLER_IP);
  gcReturnUDP = new UDPWrapper(GAMECONTROLLER_RETURN_PORT,broadcast,CommInfo::GAME_CONTROLLER_IP);
  teamUDP->startListenThread(&CommunicationModule::listenTeamUDP,this);
  toolUDP->startListenThread(&CommunicationModule::listenToolUDP,this);
  gcDataUDP->startListenThread(&CommunicationModule::listenGameControllerUDP,this);
  printf("Initialized communication wrappers--\n\tBroadcast IP: %s\n\tTeam UDP: %i\n", 
    CommInfo::TEAM_BROADCAST_IP.c_str(),
    CommInfo::TEAM_UDP_PORT
  );
  connected_ = true;
  teamTimer_.start();
  gcTimer_.start();
}

void CommunicationModule::processFrame() {
  if(robot_state_->WO_SELF == WO_TEAM_LISTENER) return;
  else
    sendTeamUDP();
}

void CommunicationModule::sendGameControllerUDP() {
  RoboCupGameControlReturnData packet;
  packet.playerNum = robot_state_->WO_SELF;
  packet.teamNum = game_state_->gameContTeamNum;
  auto rd = team_packets_->relayData[robot_state_->WO_SELF];
  packet.fallen = rd.bvrData.fallen();
  packet.pose[0] = rd.locData.robotX;
  packet.pose[1] = rd.locData.robotY;
  packet.pose[2] = rd.locData.orientation();
  packet.ballAge = rd.bvrData.ballMissed/30.0;
  packet.ball[0] = rd.locData.balls[0];
  packet.ball[1] = rd.locData.balls[1];
  bool res = gcReturnUDP->send(packet);
  if(!res) std::cerr << "Failed status message to Game Controller." << std::endl;
}

void CommunicationModule::listenTeamUDP() {
  SPLStandardMessage message;
  tlog(40, "Listening for packets.");
  bool res = this->teamUDP->recv(message, sizeof(message));
  tlog(40, "Received a UDP packet (frame %i), result: %i", this->frame_info_->frame_id, res);
  if(!res) return;
  tlog(40, "Checking ignore comms: %i", this->robot_state_->ignore_comms_);
  if(this->robot_state_->ignore_comms_) return;
 
  tlog(40, "Packet Robot Number: %i, Self: %i, Min: %i, Max: %i\n",
          message.playerNum, this->robot_state_->WO_SELF, WO_TEAM_FIRST,
          WO_TEAM_LAST);
  tlog(40, "GC Team: %i, Self: %i\n", message.teamNum, this->game_state_->gameContTeamNum);
  // not us but our team, and valid player num
  if (message.playerNum != this->robot_state_->WO_SELF &&
      message.teamNum == this->game_state_->gameContTeamNum &&
      message.playerNum >= WO_TEAM_FIRST && message.playerNum <= WO_TEAM_LAST) {
    tlog(40, "Converting packet.");
    TeamPacket tp = PacketConverter::convert(message);
    if(!message.valid) {
      fprintf(stderr, "INVALID MESSAGE RECEIVED FROM ROBOT %i, TEAM %i\n",
              message.playerNum, message.teamNum);
      return;
    }
    tlog(40, "Packet converted."); 
    tlog(40, "Getting relay data.");
    uint32_t robotTime = tp.sentTime;
    int diff = this->getVirtualTime() - robotTime;
    tlog(40, "Relay vtime diff: %i", diff);
    this->updateVirtualTime(robotTime);

    // Process relayable data
    this->tryUpdateRelayData(tp, message.playerNum);
  }
}

void CommunicationModule::tryUpdateRelayData(const TeamPacket& packet, int playerNum) {
  if(playerNum == robot_state_->WO_SELF) return;
  this->team_packets_->relayData[playerNum] = packet;
  this->team_packets_->frameReceived[playerNum] = this->frame_info_->frame_id;
  tlog(40, "Updating relay data for robot %i at frame %i\n", playerNum,
          this->team_packets_->frameReceived[playerNum]);
  if(this->getVirtualTime() - packet.sentTime < LOC_INVALID_DELAY)
    this->team_packets_->ballUpdated[playerNum] = this->frame_info_->frame_id;

  this->world_objects_->objects_[playerNum].loc.x = packet.locData.robotX;
  this->world_objects_->objects_[playerNum].loc.y = packet.locData.robotY;
  this->world_objects_->objects_[playerNum].orientation = packet.locData.orientation();

  this->world_objects_->objects_[playerNum].sd.x = packet.locData.robotSDX;
  this->world_objects_->objects_[playerNum].sd.y = packet.locData.robotSDY;
  this->world_objects_->objects_[playerNum].sdOrientation = packet.locData.sdOrient;
}

// send messages to teammates
void CommunicationModule::sendTeamUDP() {
  int WO_SELF = robot_state_->WO_SELF;
  if (WO_SELF < WO_TEAM_FIRST || WO_SELF > WO_TEAM_LAST) return;

  // Contruct the team packet, done before checking connect
  // so we can use the filled in data in the simulator
  TeamPacket tp;

  tp.robotIP = robot_state_->robot_id_;
  tp.sentTime = getVirtualTime();
  auto& locData = tp.locData;
  auto& bvrData = tp.bvrData;
    
  WorldObject& self = world_objects_->objects_[WO_SELF];
  locData.robotX = self.loc.x;
  locData.robotY = self.loc.y;
  locData.robotSDX = self.sd.x;
  locData.robotSDY = self.sd.y;
  locData.iorientation = self.orientation * LocStruct::OrientConversion;

  // ball loc
  WorldObject* ball = &(world_objects_->objects_[WO_BALL]);
  WorldObject* goal = &(world_objects_->objects_[WO_OPP_GOAL]);
  // Originally the ball position was pulled from localization mem. This bypasses
  // any possible logic for altering the location based on whether the ball was
  // recently seen. If ball->loc is good enough for behaviors it should be good
  // enough for comm.
  locData.balls[0] = ball->loc.x;
  locData.balls[1] = ball->loc.y;
  // Compute alternate balls
  // If out on the side line: placed on the center line 1 meter from the sideline on the appropriate side
  if(ball->loc.y > HALF_FIELD_Y || ball->loc.y < -HALF_FIELD_Y) {
    locData.balls[2] = ball->loc.x + 1'000;
    locData.balls[3] = sign(ball->loc.y) * (HALF_FIELD_Y - 1'000);
    locData.balls[4] = ball->loc.x - 1'000;
    locData.balls[5] = sign(ball->loc.y) * (HALF_FIELD_Y - 1'000);
  }
  // If out on the goal line: placed on the center line 1 meter from the sideline on the appropriate side
  else { // Default to assuming it's out on the goal line
    locData.balls[2] = 0;
    locData.balls[3] = sign(ball->loc.y) * (HALF_FIELD_Y - 1'000);
    locData.balls[4] = 0;
    locData.balls[5] = -sign(ball->loc.y) * (HALF_FIELD_Y - 1'000);
  }
  locData.ballCov = localization_->getBallCov(localization_->bestModel);

  // send info about when we've heard from each teammate
  // based on older commucation system; deprecated and should be changed
  for (int j = WO_TEAM_FIRST; j <= WO_TEAM_LAST; j++){
    tp.setPacketsMissed(j, frame_info_->frame_id - team_packets_->frameReceived[j]);
  }

  // send info about our role and ball
  // to help figure out roles
  bvrData.role = robot_state_->role_;
  bvrData.setBallSeen(ball->seen);
  bvrData.ballMissed = (frame_info_->frame_id - ball->frameLastSeen);
  bvrData.whistleScore = audio_processing_->whistle_score_;
  bvrData.whistleSd = audio_processing_->whistle_sd_;
  bvrData.whistleHeardFrame = audio_processing_->whistle_heard_frame_; 

  // some constants that affect ball bid calcultion and comms behavior
  // approximately how fast the robot walks
  const float walkForwardSpeed = 280.0f; // mm/s
  // approximately how fast the robot turns in place
  const float turningSpeed = 50.0f; // degrees/s
  // approximately how fast the robot turns around the ball
  const float rotateAroundBallSpeed = turningSpeed/3; // degrees/second
  // expected number of messages that are sent in a burst
  const float burstFactor = 2.0;
  // how much closer a supporter needs to be to become chaser
  const float switchingCost = 200.0;

  // Ball Bid
  // how close are we to the ball?
  bvrData.ballBid = ball->distance;
  // not just how close, but are we facing it?
  float initialRotation = fabs(ball->bearing)*RAD_T_DEG;
  // and how much do we have to turn after reaching the ball?
  float finalRotation = fabs(behavior_->largeGapHeading - ball->bearing)*RAD_T_DEG;
  if (finalRotation > 180.0) finalRotation = 360.0 - finalRotation;
  bvrData.ballBid += initialRotation / turningSpeed * walkForwardSpeed;
  bvrData.ballBid += finalRotation / rotateAroundBallSpeed * walkForwardSpeed;
  // and are we currently obstructed? (sonar)
  float timeSinceSonar = (frame_info_->seconds_since_start - behavior_->sonarObstacleTime);
  // half meter penalty for sonar avoiding robots
  if (timeSinceSonar < 0.25) bvrData.ballBid += 500;
  // and are we the keeper?
  if (robot_state_->WO_SELF == KEEPER){
      // keeper must be 33% closer than other robots to win out and go for it
      bvrData.ballBid *= 1.5;
      // currently diving?
      if (behavior_->keeperDiving != Dive::NONE) bvrData.ballBid += 12000;
      // not if ball is coming at us
      if (ball->relVel.x < -50) bvrData.ballBid += 6500;
      else if (ball->relVel.x < -30)
          bvrData.ballBid += 25.0 * -ball->relVel.x;
      // and don't for the ball unless it's close to the goal area
      if (robot_state_->role_ == KEEPER){
        // only in last 2 meters of field and not too far to sideline
        if (ball->loc.x > -(HALF_FIELD_X - OUTER_PENALTY_X) || (fabs(ball->loc.y) > GOAL_Y))
          bvrData.ballBid += 7000;
      // but if we're already chasing, chase up to midfield
      } else if (ball->loc.x > 0)
          bvrData.ballBid += 12000;
  }
  // and make sure we're not lost
  if (localization_->oppositeModels || localization_->fallenModels)
    bvrData.ballBid += 6000;

  // state
  bvrData.state = game_state_->state();

  // active getup or there is a fall direction means we've fallen
  bvrData.setFallen(odometry_->getting_up_side_ != Getup::NONE ||
                    odometry_->fall_direction_ != Fall::NONE);

  // set play stuff (possibly unused ??)
  bvrData.setPlayType = behavior_->setPlayInfo.type;
  bvrData.setReversed(behavior_->setPlayInfo.reversed);
  bvrData.setPlayTargetPlayer = behavior_->setPlayInfo.targetPlayer;

  // bad model ... ignore ball
  if (localization_->bestAlpha < 0.8 || localization_->oppositeModels)
    bvrData.ballMissed = 255;

  // how many frames to wait between messages
  int secsRemaining = game_state_->secsRemaining;
  if (game_state_->isFirstHalf == 1)
      secsRemaining += 600;
  int budget = std::max(1, game_state_->messageBudget);
  tp.bvrData.waitFrames = (int)std::ceil(secsRemaining*burstFactor*30/budget);

  //update data for self
  team_packets_->relayData[WO_SELF] = tp;
  team_packets_->ballUpdated[WO_SELF] = frame_info_->frame_id;


  // Determine whether sending a packet is necessary
  // do we have messages left in the budget?
  if (game_state_->messageBudget < 5) return; // Leaving a little buffer
  // do we need to tell the team about a whistle?
  // Default to not sending the message
  bool whistleDetected = false;
  // Unless whistle score is greater than 0.5
  if((audio_processing_->whistle_score_ >= 0.5) &&
     (audio_processing_->whistle_heard_frame_ > frame_info_->frame_id - 30)){
     whistleDetected = true; 
     bvrData.msgType = 2;
  }
  // have we heard from a teammate recently?
  int mrf = WO_TEAM_FIRST; // robot that send most recently (could be us)
  for (int i = WO_TEAM_FIRST+1; i <= WO_TEAM_LAST; i++){
      if (team_packets_->frameReceived[i] > team_packets_->frameReceived[mrf])
          mrf = i;
  }
  auto lastMsg = team_packets_->relayData[mrf];
  auto lastMsgTime = team_packets_->frameReceived[mrf];
  // If whistle was detected we prioritize sending the message
  if (!whistleDetected) {
    if (((lastMsgTime + lastMsg.bvrData.waitFrames) > frame_info_->frame_id)
        && (mrf != WO_SELF)) {
      // We HAVE heard from a teammate recently, so we should NOT send a message
      bool shouldSend = false;
      // UNLESS one of the following:
      // 1) We think they're flipped
      Point2D ourBall = Point2D(ball->loc.x, ball->loc.y);
      Point2D theirBall = lastMsg.locData.ballPos();
      if ((ourBall.getDistanceTo(theirBall) > 2000) &&
          (ourBall.getDistanceTo(-theirBall) < 1000)) {
        shouldSend = true;
        bvrData.msgType = 3;
      }
      // 2) We have a better bid
      float theirBid = team_packets_->relayData[mrf].bvrData.ballBid;
      // (assume they are chasing the ball, so adjust for time)
      theirBid -= walkForwardSpeed * (frame_info_->frame_id - lastMsgTime)/30.0;
      if (bvrData.ballBid + switchingCost < theirBid) {
        shouldSend = true;
        bvrData.msgType = 1;
      }
      if (!shouldSend) return;
    } else {
      // We have NOT heard from another robot recently, so someone should send
      // Let's give the precedence to the robot that sent the last message
      int sendOffset = (WO_SELF-mrf) % NUM_PLAYERS;
      if (frame_info_->frame_id < (lastMsgTime + lastMsg.bvrData.waitFrames +
          sendOffset)) return;
      // No point in sending if we don't see the ball
      if (!ball->seen) return;
      // We've seen the ball, and no one else has, so let's chase
      bvrData.msgType = 1;
    }
    
    // We should be in Playing state and Normal phase
    if (game_state_->state() != PLAYING) return;
    if (game_state_->gamePhase != PHASE_NORMAL) return;
  } else {
    // On whistle, 5 is the chaser.
    if (WO_SELF == WO_TEAM5)
        bvrData.msgType = 1;
  }
  // Make sure we haven't sent a packet too recently!
  if (teamTimer_.elapsed_s() < tp.bvrData.waitFrames/30.0) return;

  // send packet
  incrementVirtualTime();
  SPLStandardMessage packet = PacketConverter::convert(tp);
  packet.teamNum = game_state_->gameContTeamNum;
  packet.playerNum = WO_SELF;
  if(connected_ && frame_info_->source == MEMORY_ROBOT) {
      bool res = teamUDP->send(packet, packet.send_size());
    if (res){
      team_packets_->frameReceived[WO_SELF] = frame_info_->frame_id;
    }
    else
      std::cerr << "Sending team UDP failed" << std::endl;
  }
  teamTimer_.restart();
}

void CommunicationModule::sendToolResponse(ToolPacket message) {
  toolUDP->sendToSender(message);
}

void CommunicationModule::sendToolRequest(ToolPacket message) {
  toolUDP->send(message);
}

void CommunicationModule::listenToolUDP() {
  // in use  P
  ToolPacket tp;
  bool res = this->toolUDP->recv(tp);
  if(!res) return;
  int prev_state = this->game_state_->state();

  switch(tp.message) {
    case ToolPacket::StateInitial: this->game_state_->setState(INITIAL); break;
    case ToolPacket::StateReady: this->game_state_->setState(READY); break;
    case ToolPacket::StateSet: this->game_state_->setState(SET); break;
    case ToolPacket::StatePlaying: this->game_state_->setState(PLAYING); break;
    case ToolPacket::StatePenalized: this->game_state_->setState(PENALISED); break;
    case ToolPacket::StateFinished: this->game_state_->setState(FINISHED); break;
    case ToolPacket::StateTesting: this->game_state_->setState(TESTING); break;
    case ToolPacket::StateTestOdometry: {
        this->game_state_->setState(TEST_ODOMETRY);
        this->behavior_->test_odom_fwd = tp.odom_command.x;
        this->behavior_->test_odom_side = tp.odom_command.y;
        this->behavior_->test_odom_turn = tp.odom_command.theta;
        this->behavior_->test_odom_walk_time = tp.odom_command.time;
        this->behavior_->test_odom_new = true;
      }
      break;
    case ToolPacket::StateCameraTop: this->game_state_->setState(TOP_CAM); break;
    case ToolPacket::StateCameraBottom: this->game_state_->setState(BOTTOM_CAM); break;
    case ToolPacket::LogSelect: {
        this->handleLoggingBlocksMessage(tp);
        this->core_->setLogSelections(tp);
      }
      break;
    case ToolPacket::LogBegin: {
        this->core_->enableLogging(tp.frames, tp.interval);
        printf("Logging %i frames, once per %2.2f second%s\n", tp.frames, tp.interval, tp.interval == 1 ? "" : "s");
      }
      break;
    case ToolPacket::LogEnd: this->core_->startDisableLogging(); break;
    case ToolPacket::RestartInterpreter:
      if(interpreter_restart_requested_) *interpreter_restart_requested_ = true;
      break;
    case ToolPacket::SetTopCameraParameters:
      this->camera_->set_top_params_ = true;
      this->camera_->comm_module_request_received_ = true;
      this->handleCameraParamsMessage(this->camera_->params_top_camera_, tp.data.data());
      cout << "CommunicationModule: Set top camera params" << endl;
      break;
    case ToolPacket::SetBottomCameraParameters:
      this->camera_->set_bottom_params_ = true;
      this->camera_->comm_module_request_received_ = true;
      this->handleCameraParamsMessage(this->camera_->params_bottom_camera_, tp.data.data());
      cout << "CommmunicationModule: Set bottom camera params" << endl;
      break;
    case ToolPacket::GetCameraParameters:
      this->camera_->get_top_params_ = true;
      this->camera_->get_bottom_params_ = true;
      this->camera_->comm_module_request_received_ = true;
      cout << "CommunicationModule: Read camera params" << endl;
      break;
    case ToolPacket::ResetCameraParameters: // reset camera
      std::cout << "CommunicationModule: Reset camera " << std::endl;
      this->camera_->reset_bottom_camera_ = true;
      this->camera_->reset_top_camera_ = true;
      this->camera_->comm_module_request_received_ = true;
      break;
    case ToolPacket::ManualControl: {
        this->game_state_->setState(MANUAL_CONTROL);
        this->behavior_->test_odom_fwd = tp.odom_command.x;
        this->behavior_->test_odom_side = tp.odom_command.y;
        this->behavior_->test_odom_turn = tp.odom_command.theta;
        this->behavior_->test_stance = tp.odom_command.stance;
        this->behavior_->test_odom_walk_time = 10.0f;
        this->behavior_->test_odom_new = true;
      }
      break;
    case ToolPacket::RunBehavior: {
        this->game_state_->setState(INITIAL);
        this->core_->interpreter_->runBehavior((char*)&tp.data);
      }
      break;
    case ToolPacket::SetStiffness: 
      for (int i = 0; i < NUM_JOINTS; i++){
        this->joint_commands_->setJointStiffness(i, tp.jointStiffness[i]);
      }
      this->joint_commands_->send_stiffness_=true;
      this->joint_commands_->stiffness_time_=300;
      break;
  }

  if(prev_state != this->game_state_->state()) {
    printf("State changed from %s to %s\n", stateNames[prev_state].c_str(), stateNames[this->game_state_->state()].c_str());
  }
}

void CommunicationModule::handleCameraParamsMessage(CameraParams &params, char *msg) {

  std::vector<std::string> paramNames;
  std::vector<int> paramValues;

  int i = 0;
  std::string currentToken;

  bool tokenIsParamName = true;
  while (msg[i] != '|') {
    if (msg[i] == ' ') { // current token has ended
      if (tokenIsParamName) {
        paramNames.push_back(currentToken);
        tokenIsParamName = false;
      } else {
        paramValues.push_back(std::stoi(currentToken));
        tokenIsParamName = true;
      }
      currentToken.clear();
    } else {
      currentToken += msg[i];
    }
    i++;
  }

  for (unsigned i = 0; i < paramNames.size(); i++) {
    if (paramNames[i] == "AutoWhiteBalance") {
      params.kCameraAutoWhiteBalance = paramValues[i];
    } else if (paramNames[i] == "ExposureAuto") {
      params.kCameraExposureAuto = paramValues[i];
    } else if (paramNames[i] == "BacklightCompensation") {
      params.kCameraBacklightCompensation = paramValues[i];
    } else if (paramNames[i] == "Brightness") {
      params.kCameraBrightness = paramValues[i];
    } else if (paramNames[i] == "Contrast") {
      params.kCameraContrast = paramValues[i];
    } else if (paramNames[i] == "Saturation") {
      params.kCameraSaturation = paramValues[i];
    } else if (paramNames[i] == "Hue") {
      params.kCameraHue = paramValues[i];
    } else if (paramNames[i] == "Exposure") {
      params.kCameraExposure = paramValues[i];
    } else if (paramNames[i] == "Gain") {
      params.kCameraGain = paramValues[i];
    } else if (paramNames[i] == "Sharpness") {
      params.kCameraSharpness = paramValues[i];
    }
  }
}

void CommunicationModule::handleLoggingBlocksMessage(const ToolPacket& packet) {
  std::string block_name;
  bool log_block;
  int i = 0;
  const char* msg = packet.data.data();
  while (msg[i] != '|') {
    if (msg[i] == ' ') {
      i++;
      if (msg[i] == '0')
        log_block = false;
      else if (msg[i] == '1')
        log_block = true;
      else {
        std::cout << "bad logging info " << msg[i] << std::endl;
        return;
      }
      if (log_block) {
        std::cout << std::boolalpha << "setting logging of " << block_name << " to " << log_block << std::endl;
      }
      // special case for behavior trace (not a block)
      if (block_name.compare("behavior_trace") == 0){
        if (behavior_ != NULL)
          behavior_->log_behavior_trace_ = log_block;
      } else {
        memory_->setBlockLogging(block_name,log_block);
      }
      block_name.clear();
      i++;
      assert(msg[i] == ',');
    } else
      block_name += msg[i];
    i++;
  }
}

void CommunicationModule::listenGameControllerUDP() {
  RoboCupGameControlData gcData;
  bool res = this->gcDataUDP->recv(gcData);
  if(!res) return;
  if(this->robot_state_->ignore_comms_) return;

  // ch/ck version of packet
  if (gcData.version != GAMECONTROLLER_STRUCT_VERSION) {
    std::cout << "ERROR: expect GameControllerPacket version: " <<
        GAMECONTROLLER_STRUCT_VERSION << ", received: " <<gcData.version <<
        std::endl;
    return;
  }

  // Determine which TeamInfo is ours
  int teamNumber = this->game_state_->gameContTeamNum;
  int teamIndex=0;
  int oppIndex=1;
  if (gcData.teams[1].teamNumber==teamNumber) {
    teamIndex=1;
    oppIndex=0;
  } else if (gcData.teams[0].teamNumber != teamNumber) {
    std::cout << "ERROR: Our team (Team " << teamNumber <<
        ") isn't playing in this game (Team " << gcData.teams[0].teamNumber <<
        " vs Team " << gcData.teams[1].teamNumber << ")" << std::endl;
    return;
  }
  TeamInfo tInfo=gcData.teams[teamIndex];
  TeamInfo oppInfo=gcData.teams[oppIndex];

  int WO_SELF = this->robot_state_->WO_SELF;

  // process team color info
  if (this->robot_state_->team_ != tInfo.teamColour) {
    std::cout << "Gamecontroller has a team change" << std::endl;
    this->robot_state_->team_changed_ = true;
  }

  this->game_state_->messageBudget = tInfo.messageBudget;

  this->robot_state_->team_ = tInfo.teamColour;
  this->game_state_->isFirstHalf=gcData.firstHalf; // 1 = game in first half, 0 otherwise

  int oldState = this->game_state_->state();

  // process state info (robot's current game state; e.g. Playing, Ready, etc.)
  
  RobotInfo rInfo=tInfo.players[WO_SELF-1]; // We index from 1-5, they go 0-4
  // if robot is not penalized, set its state based on GC message
  if (rInfo.penalty == PENALTY_NONE) { 
    switch(gcData.state) {
      case STATE_INITIAL: this->game_state_->setState(INITIAL); break;
      case STATE_READY: this->game_state_->setState(READY); break;
      case STATE_SET: this->game_state_->setState(SET); break;
      case STATE_PLAYING: this->game_state_->setState(PLAYING); break;
      case STATE_FINISHED: this->game_state_->setState(FINISHED); break;
    }
    if(oldState != this->game_state_->state() &&
        this->game_state_->state() == PLAYING &&
        oldState != PENALISED)
      this->behavior_->timePlayingStarted = this->frame_info_->seconds_since_start - 20;
  } else { // else this robot is penalized.
    this->game_state_->setState(PENALISED);
    this->game_state_->spawnFromPenalty = (rInfo.penalty != PENALTY_SPL_ILLEGAL_MOTION_IN_SET);
    this->game_state_->secsTillUnpenalised=rInfo.secsTillUnpenalised;
  }
  

  // check if state changed
  if (oldState != this->game_state_->state()) {
    this->game_state_->lastStateChangeFromButton = false;
    if (oldState == PENALISED)
      this->game_state_->lastTimeLeftPenalized = (this->frame_info_->seconds_since_start);
  }

  switch(gcData.gamePhase) {
    case GAME_PHASE_NORMAL: this->game_state_->gamePhase = PHASE_NORMAL; break;
    case GAME_PHASE_PENALTYSHOOT: this->game_state_->gamePhase = PHASE_PENALTYSHOOT; break;
    case GAME_PHASE_OVERTIME: this->game_state_->gamePhase = PHASE_OVERTIME; break;
    case GAME_PHASE_TIMEOUT: this->game_state_->gamePhase = PHASE_TIMEOUT; break;
  }
  this->game_state_->isPenaltyKick = (this->game_state_->gamePhase==PHASE_PENALTYSHOOT);

  // free kick stuff
  this->game_state_->isFreeKick = (gcData.setPlay == SET_PLAY_GOAL_KICK) ||
      (gcData.setPlay == SET_PLAY_PUSHING_FREE_KICK) ||
      (gcData.setPlay == SET_PLAY_CORNER_KICK) ||
      (gcData.setPlay == SET_PLAY_KICK_IN);
  this->game_state_->isFreeKickTypeGoal = (gcData.setPlay == SET_PLAY_GOAL_KICK) ||
      (gcData.setPlay == SET_PLAY_CORNER_KICK) ||
      (gcData.setPlay == SET_PLAY_KICK_IN);
  this->game_state_->isFreeKickTypePenalty = (gcData.setPlay == SET_PLAY_PENALTY_KICK);

  // random stuff
  this->game_state_->isFirstHalf=gcData.firstHalf; // 1 = game in first half, 0 otherwise
  if (gcData.kickingTeam==this->game_state_->gameContTeamNum)
    this->game_state_->ourKickOff=true;
  else this->game_state_->ourKickOff=false;

  // number of seconds remaining in the half:
  this->game_state_->secsRemaining=gcData.secsRemaining;

  // check if somebody scored
  if (this->game_state_->ourScore<tInfo.score)
    std::cout << "WE SCORED !!" << std::endl;
  this->game_state_->ourScore=tInfo.score;

  if (this->game_state_->opponentScore < oppInfo.score)
    std::cout << "CRAP THEY SCORED ???" << std::endl;
  this->game_state_->opponentScore = oppInfo.score;

  this->game_state_->frameReceived = this->frame_info_->frame_id;

  if(gcTimer_.elapsed_s() > SECONDS_PER_PACKET) {
    this->sendGameControllerUDP();
    gcTimer_.restart();
  }
}

bool CommunicationModule::streaming() { return tcpserver_ != nullptr && tcpserver_->established(); }

void CommunicationModule::startTCPServer() {
  cleanupTCPServer();
  tcpserver_ = std::make_unique<TCPServer>(CommInfo::TOOL_TCP_PORT);
  tcpserver_->loop_server([this](auto result) {
    if(result == 0)
      printf("Started TCP Server, listening on port %i\n", tcpserver_->server_port());
    else {
      fprintf(stderr, "Error code %i starting TCP server: '%s'\n", result.value(), result.message().c_str());
    }
    this->cleanupStreamThread();
    stream_thread_ = std::make_unique<std::thread>(&CommunicationModule::stream, this);
  });
}

void CommunicationModule::cleanupTCPServer() {
  tcpserver_.reset();
  cleanupStreamThread();
}

void CommunicationModule::cleanupStreamThread() {
  if(stream_thread_ != nullptr) {
    stopping_stream_ = true;
    streaming_cv_.notify_one();
    stream_thread_->join();
    stream_thread_.reset();
    stopping_stream_ = false;
  }
}

void CommunicationModule::stream() {
  while(streaming()) {
    std::unique_lock<std::mutex> lock(streaming_mutex_);
    const auto& buffer = streaming_logger_->getBuffer();
    while(buffer.size == 0 && !stopping_stream_) {
      streaming_cv_.wait(lock, [&,this] { return buffer.size > 0 || stopping_stream_; });
      if(stopping_stream_) return;
    }
    tcpserver_->send_message(buffer);
    streaming_logger_->clearBuffer();
  }
}

void CommunicationModule::optionallyStream() {
  if(streaming()) {
    const StreamBuffer& buffer = streaming_logger_->getBuffer();
    if(buffer.size > 0) return;
    {
      std::lock_guard<std::mutex> lock(streaming_mutex_);
      streaming_logger_->writeMemory(*memory_);
    }
    streaming_cv_.notify_one();
  }
}

uint32_t CommunicationModule::getVirtualTime() {
  return vtime_;
}

void CommunicationModule::updateVirtualTime(uint32_t received) {
  vtime_ = max(vtime_, received);
}

void CommunicationModule::incrementVirtualTime() {
  vtime_++;
}
