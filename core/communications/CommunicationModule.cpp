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

#define PACKETS_PER_SECOND 1
#define SECONDS_PER_PACKET (1.0f/PACKETS_PER_SECOND)
#define FRAMES_PER_PACKET (30/PACKETS_PER_SECOND)
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
  packet.team = game_state_->gameContTeamNum;
  packet.player = robot_state_->WO_SELF;
  packet.message = GAMECONTROLLER_RETURN_MSG_ALIVE;
  bool res = gcReturnUDP->send(packet);
  //std::cout << "Sending GC message" << std::endl;
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
 
  bool listener = this->robot_state_->WO_SELF == WO_TEAM_LISTENER;
  tlog(40, "Packet Robot Number: %i, Self: %i, Min: %i, Max: %i\n", message.playerNum, this->robot_state_->WO_SELF, WO_TEAM_FIRST, WO_TEAM_LAST);
  tlog(40, "GC Team: %i, Self: %i\n", message.teamNum, this->game_state_->gameContTeamNum);
  // not us but our team, and valid player num
  if (message.playerNum != this->robot_state_->WO_SELF &&
      message.teamNum == this->game_state_->gameContTeamNum &&
      //tp.rbTeam == this->robot_state_->team_ &&
      message.playerNum >= WO_TEAM_FIRST && message.playerNum <= WO_TEAM_LAST) {
    tlog(40, "Converting packet.");
    TeamPacket tp = PacketConverter::convert(message);
    if(!message.valid) {
      fprintf(stderr, "INVALID MESSAGE RECEIVED FROM ROBOT %i, TEAM %i\n", message.playerNum, message.teamNum);
      return;
    }

    tlog(40, "Packet converted."); 
    tlog(40, "Getting relay data.");
    uint32_t robotTime = tp.sentTime;
    int diff = this->getVirtualTime() - robotTime;
    tlog(40, "Relay vtime diff: %i", diff);
    this->updateVirtualTime(robotTime);
    if(listener)
    {}//this->robot_state_->team_ = tp.rbTeam;
    else if (diff > 3 * PACKETS_PER_SECOND)
      return;
    tlog(40, "Is listener? %i", listener);

    // Opponents data isn't relayed
    //auto& oppMem = this->team_packets_->oppData[message.playerNum];
    //oppMem = tp.oppData;
    //this->team_packets_->oppUpdated[message.playerNum] = true;

    // Process relayable data
    this->tryUpdateRelayData(tp, message.playerNum);
  }
}

void CommunicationModule::tryUpdateRelayData(const TeamPacket& packet, int playerNum) {
  tlog(40, "Update relay data");
  if(playerNum == robot_state_->WO_SELF) return;
  auto& previous = this->team_packets_->relayData[playerNum];
  const auto& incoming = packet;
  tlog(40, "Incoming time: %i, Prev time: %i\n", incoming.sentTime, previous.sentTime);
  //if(incoming.sentTime > previous.sentTime || playerNum == packet.playerNum) {
    previous = incoming;
    this->team_packets_->frameReceived[playerNum] = this->frame_info_->frame_id;
    tlog(40, "Updating relay data for robot %i at frame %i\n", playerNum, this->team_packets_->frameReceived[playerNum]);
    if(this->getVirtualTime() - incoming.sentTime < LOC_INVALID_DELAY)
      this->team_packets_->ballUpdated[playerNum] = this->frame_info_->frame_id;

    this->world_objects_->objects_[playerNum].loc.x = incoming.locData.robotX;
    this->world_objects_->objects_[playerNum].loc.y = incoming.locData.robotY;
    this->world_objects_->objects_[playerNum].orientation = incoming.locData.orientation();

    this->world_objects_->objects_[playerNum].sd.x = incoming.locData.robotSDX;
    this->world_objects_->objects_[playerNum].sd.y = incoming.locData.robotSDY;
    this->world_objects_->objects_[playerNum].sdOrientation = incoming.locData.sdOrient;
  //}
}

// send messages to teammates
void CommunicationModule::sendTeamUDP() {
  // Contruct the team packet, done before checking connect
  // so we can use the filled in data in the simulator

  int WO_SELF = robot_state_->WO_SELF;

  if (WO_SELF < WO_TEAM_FIRST || WO_SELF > WO_TEAM_LAST) return;

  TeamPacket tp;

  //tp.playerNum = WO_SELF;
  //tp.rbTeam  = robot_state_->team_;
  //tp.gcTeam  = game_state_->gameContTeamNum;
  tp.robotIP = robot_state_->robot_id_;

  int i = WO_SELF; {
  //for(int i = WO_TEAM_FIRST; i <= WO_TEAM_LAST; i++) {
    auto& relayData = tp;
    relayData.sentTime = getVirtualTime();

    // Relay packets from other team members
    //if(i != WO_SELF) {
      //relayData = team_packets_->relayData[i];
      //continue;
    //}
    auto& locData = relayData.locData;
    auto& bvrData = relayData.bvrData;
    auto& commData = relayData;
    
    WorldObject& self = world_objects_->objects_[i];
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
    //if(ball->loc.x > HALF_FIELD_X || ball->loc.x < -HALF_FIELD_X) {
      locData.balls[2] = 0;
      locData.balls[3] = sign(ball->loc.y) * (HALF_FIELD_Y - 1'000);
      locData.balls[4] = 0;
      locData.balls[5] = -sign(ball->loc.y) * (HALF_FIELD_Y - 1'000);
    }

    locData.ballCov = localization_->getBallCov(localization_->bestModel);

    // send info about when we've heard from each teammate
    for (int j = WO_TEAM_FIRST; j <= WO_TEAM_LAST; j++){
      commData.setPacketsMissed(j, frame_info_->frame_id - team_packets_->frameReceived[j]);
    }

    // fill in opponents from opponent tracking
    // first set them all to unfilled
    /*
    for (int i = 0; i < MAX_OPP_MODELS_IN_MEM; i++){
      tp.oppData.opponents[i].filled = false;
    }
    int fillIndex = 0;
    for (int j = 0; j < MAX_OPP_MODELS_IN_MEM; j++){
      OpponentModel& model = opponents_->locModels[j];
      if (model.alpha == -1000) continue;

      // dont send garbage info
      if (fabs(model.X00) > 3000) continue;
      if (fabs(model.X10) > 3000) continue;
      if (model.SRXX == 0 || model.SRYY == 0) continue;
      if (model.SRXX > 10000 || model.SRYY > 10000) continue;
      if (std::isnan(model.X00) || std::isnan(model.X10)) continue;
      if (std::isnan(model.SRXX) || std::isnan(model.SRYY)) continue;

      // these are all in cm as well (how its used by the filter)
      tp.oppData.opponents[fillIndex].x = model.X00;
      tp.oppData.opponents[fillIndex].y = model.X10;
      tp.oppData.opponents[fillIndex].sdx = model.SRXX;
      tp.oppData.opponents[fillIndex].sdy = model.SRYY;
      tp.oppData.opponents[fillIndex].sdxy = model.SRXY;
      tp.oppData.opponents[fillIndex].framesMissed = (frame_info_->frame_id - model.frameLastObserved);
      tp.oppData.opponents[fillIndex].filled = true;

      fillIndex++;
      if (fillIndex >= MAX_OPP_MODELS_IN_MEM) break;
    }
    */

    // send info about our role and ball
    // to help figure out roles
    bvrData.role = robot_state_->role_;

    // ball info
    //bvrData.ballDistance = ball->distance;
    bvrData.setBallSeen(ball->seen);
    bvrData.ballMissed = (frame_info_->frame_id - ball->frameLastSeen);
    bvrData.whistleScore = audio_processing_->whistle_score_;
    bvrData.whistleSd = audio_processing_->whistle_sd_;

    // ball bid
    // not just how close, but are we facing it... and are we facing roughly the right direction?
    // turn ~50 deg / sec, walk 192 mm / sec.
    // so every 50 degrees off is anohter 192 mm we could have walked
    // but we only need to turn maybe halfway?
    // so i'll say every 50 degrees is worth 100mm
    bvrData.ballBid = ball->distance;

    float bearingError = fabs(ball->bearing)*RAD_T_DEG - 30.0;
    if (bearingError < 0) bearingError = 0;
    bvrData.ballBid += bearingError * 3.0;

    // For each 45 difference between our orientation and the goal's position,
    // we increase ball bid by 1,000. So a robot walking toward the ball and away
    // from the opponents' goal has to be 2 meters closer than a robot approaching
    // the ball and facing the opponents' goal.
    tlog(40, "Adjusting ball bid based on required orientation; initial bid: %2.2f", bvrData.ballBid);
    float initialRotation = ball->bearing;
    tlog(40, "Initial rotation (bearing to the ball): %2.2f", initialRotation * RAD_T_DEG);
    float finalRotation = behavior_->largeGapHeading;
    tlog(40, "Final rotation (bearing to the shot target from the ball's position): %2.2f", finalRotation * RAD_T_DEG);
    // Heading error is proportional to the total rotation time required. The initial rotation is about
    // the robot's origin - rotating in place - so it's about 3x faster. The final rotation is about the
    // ball, meaning the robot needs to strafe and rotate around a small arc.
    float headingError = fabs(initialRotation) / 3 + fabs(finalRotation);
    headingError *= RAD_T_DEG;
    tlog(40, "Final heading error with initial and final rotation normalized: %2.2f degrees", headingError);
    
    float walkForwardSpeed = 280.0f; // 280 mm/s
    float rotationSpeed = 180.0f / 12; // 180 degrees per 12 sec
    float rotationIncrease = headingError * walkForwardSpeed / rotationSpeed;
    tlog(40, "Ball bid rotation error increase: %2.4f * %2.2f / %2.2f = %2.4f", headingError, walkForwardSpeed, rotationSpeed, rotationIncrease);
    
    bvrData.ballBid += rotationIncrease;
    tlog(40, "Increased ball bid to %2.2f", bvrData.ballBid);

    // and we don't want to be downfield of the ball
    // ramp this error up to a max worth of 800 mm (close to the time required to rotate 180)
    float xError = self.loc.x - ball->loc.x;
    if (xError < 0) xError = 0;
    if (xError > 800) xError = 800;
    bvrData.ballBid += xError;

    // sonar
    float timeSinceSonar = (frame_info_->seconds_since_start - behavior_->sonarObstacleTime);
    if (timeSinceSonar < 0.25){
      // half meter penalty for sonar avoiding robots
      bvrData.ballBid += 500;
    }

    // If self is chaser and keeps kicking the ball out, penalize bid so another robot will maybe become chaser.
    float timeSinceOutOnUs = frame_info_->seconds_since_start - behavior_->outOnUsTime;
    if (timeSinceOutOnUs < 5) {
      // set the bid high enough for a non-keeper to take over
      bvrData.ballBid = 5000;
    }

    // penalty for keeper
    if (robot_state_->WO_SELF == KEEPER){
      // penalty for keeper?  or maybe an advantage to him?
      // keeper must be 33% closer than other robots to win out and go for it
      bvrData.ballBid *= 1.5;

      // not if diving
      if (behavior_->keeperDiving != Dive::NONE){
        bvrData.ballBid = 7000;
      }

      // not if ball is coming at us
      if (ball->relVel.x < -30){
        if (ball->relVel.x < -50){
          bvrData.ballBid += 6500;
        } else {
          bvrData.ballBid += 25.0 * -ball->relVel.x;
        }
      }

      // not too far up field
      if (robot_state_->role_ == KEEPER){
        // only in last 2 meters of field and not too far to sideline
        if (ball->loc.x > -2250 || (fabs(ball->loc.y) > 1500)) {
          bvrData.ballBid = 7000;
        }
      }
      // if we're already chasing, chase up to midfield
      else {
        if (ball->loc.x > 0) {
          bvrData.ballBid = 7000;
        }
      }
    }

    // we dont know which way we're going
    if (localization_->oppositeModels || localization_->fallenModels){
      bvrData.ballBid += 6000;
    }

    // state
    bvrData.state = game_state_->state();

    // active getup or there is a fall direction means we've fallen
    bvrData.setFallen(odometry_->getting_up_side_ != Getup::NONE || odometry_->fall_direction_ != Fall::NONE);

    // strategy
    //bvrData.keeperClearing = behavior_->keeperClearing;
    //bvrData.targetX = behavior_->absTargetPt.x;
    //bvrData.targetY = behavior_->absTargetPt.y;
    //bvrData.useTarget = behavior_->useAbsTarget;

    //bvrData.passInfo = behavior_->passInfo;
    //bvrData.setPlayInfo = behavior_->setPlayInfo;
    bvrData.setPlayType = behavior_->setPlayInfo.type;
    bvrData.setReversed(behavior_->setPlayInfo.reversed);
    bvrData.setPlayTargetPlayer = behavior_->setPlayInfo.targetPlayer;

    // bad model ... ignore ball
    if (localization_->bestAlpha < 0.8 || localization_->oppositeModels)
      bvrData.ballMissed = 60;
  }

  if (teamTimer_.elapsed_s() > SECONDS_PER_PACKET) {
    incrementVirtualTime();
    SPLStandardMessage packet = convert(tp);
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
  team_packets_->relayData[WO_SELF] = tp;
  //team_packets_->oppData[WO_SELF] = tp.oppData;
  team_packets_->frameReceived[WO_SELF] = frame_info_->frame_id;
  team_packets_->ballUpdated[WO_SELF] = frame_info_->frame_id;
  //team_packets_->oppUpdated[WO_SELF] = true;
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
  RoboCupGameControlData gc;
  bool res = this->gcDataUDP->recv(gc);
  if(!res) return;
  if(this->robot_state_->ignore_comms_) return;

  //std::cout << "getting messages" << std::endl;
  //printf("Team numbers: %d %d %d",gc.teams[0].teamNumber,gc.teams[1].teamNumber, this->game_state_->gameContTeamNum);
  //std::cout << std::endl;

  // check version of packet
  if (gc.version != GAMECONTROLLER_STRUCT_VERSION){
    cout << endl << "ERROR: expect GameControllerPacket version: "
         << GAMECONTROLLER_STRUCT_VERSION
         << ", received: " << gc.version << endl << endl;
  }

  int teamNumber = this->game_state_->gameContTeamNum;

  // First check if this packet is even meant for a robot on our team !
  if (gc.teams[0].teamNumber==teamNumber || gc.teams[1].teamNumber==teamNumber) {
    int teamIndex=0; // stores the index into the team array
    int oppIndex=1; // stores the index into the team array
    if (teamNumber==gc.teams[1].teamNumber) {
      teamIndex=1;
      oppIndex=0;
    }

    int WO_SELF = this->robot_state_->WO_SELF;
    TeamInfo t=gc.teams[teamIndex];
    //for (int p=0; p<6; p++) {
    //  RobotInfo r_=t.players[p]; // We index from 1-5, they go 0-4
    //  printf("Penalty: %d",r_.penalty);
    //  std::cout << std::endl;     
    //}
    //std::cout << "Team other" << std::endl;
    //TeamInfo t_=gc.teams[oppIndex];
    //for (int p=0; p<6; p++) {
    //  RobotInfo r_=t_.players[p]; // We index from 1-5, they go 0-4
    //  printf("Penalty: %d",r_.penalty);
    //  std::cout << std::endl;     
    //}

    // check if our team has changed
    //  if (t.teamColour != commonMem->team){
    // reset world objects for opposite field
    //  commonMem->teamChange = true;
    //}

    // process team color info
    if (this->robot_state_->team_ != t.teamColour) {
      std::cout << "Gamecontroller has a team change" << std::endl;
      this->robot_state_->team_changed_ = true;
    }

    this->robot_state_->team_ = t.teamColour;
    this->game_state_->isFirstHalf=gc.firstHalf; // 1 = game in first half, 0 otherwise

    int oldState = this->game_state_->state();

    // process state info (robot's current game state; e.g. Playing, Ready, etc.)
    {
      RobotInfo r=t.players[WO_SELF-1]; // We index from 1-5, they go 0-4
      // if robot is not penalized, set its state based on GC message
      if (r.penalty == PENALTY_NONE) { 
        switch(gc.state) {
          case STATE_INITIAL: this->game_state_->setState(INITIAL); break;
          case STATE_READY: this->game_state_->setState(READY); break;
          case STATE_SET: this->game_state_->setState(SET); break;
          // they want us to think of the free kicks as substates inside of playing
          case STATE_PLAYING: this->game_state_->setState(PLAYING); break;
          case STATE_FINISHED: this->game_state_->setState(FINISHED); break;
        }
        if(oldState != this->game_state_->state() && this->game_state_->state() == PLAYING && oldState != PENALISED)
          this->behavior_->timePlayingStarted = this->frame_info_->seconds_since_start - 20;

        // only continue for players (coach will cause index-out-of-bound errors
        // and we don't have anything else to update for coach anyway)
        if(WO_SELF > WO_TEAM_LAST || WO_SELF < WO_TEAM_FIRST) return;

      } else { // else this robot is penalized.
        this->game_state_->setState(PENALISED);
        this->game_state_->secsTillUnpenalised=r.secsTillUnpenalised;
      }
    } 

    // check if state changed
    if (oldState != this->game_state_->state()) {
      this->game_state_->lastStateChangeFromButton = false;
      if (oldState == PENALISED)
        this->game_state_->lastTimeLeftPenalized = (this->frame_info_->seconds_since_start);
      //  this->game_state->stateChange = true;
    }

    // check if penalty kick
    if (gc.gamePhase == GAME_PHASE_PENALTYSHOOT)
      this->game_state_->isPenaltyKick = true;
    else
      this->game_state_->isPenaltyKick = false;

    // free kick stuff
    this->game_state_->isFreeKick = (gc.setPlay == SET_PLAY_GOAL_FREE_KICK) ||
        (gc.setPlay == SET_PLAY_PUSHING_FREE_KICK) ||
        (gc.setPlay == SET_PLAY_CORNER_KICK) ||
        (gc.setPlay == SET_PLAY_KICK_IN);
    this->game_state_->isFreeKickTypeGoal = (gc.setPlay == SET_PLAY_GOAL_FREE_KICK) ||
        (gc.setPlay == SET_PLAY_CORNER_KICK) ||
        (gc.setPlay == SET_PLAY_KICK_IN);

    // random stuff
    this->game_state_->isFirstHalf=gc.firstHalf; // 1 = game in first half, 0 otherwise
    if (gc.kickingTeam==this->game_state_->gameContTeamNum) this->game_state_->ourKickOff=true;
    else this->game_state_->ourKickOff=false;

    // estimate of number of seconds remaining in the half:
    this->game_state_->secsRemaining=gc.secsRemaining;

    // check if somebody scored
    if (this->game_state_->ourScore<t.score)
      cout << "WE SCORED !!" << endl << flush;
    this->game_state_->ourScore=t.score;

    if (this->game_state_->opponentScore<gc.teams[oppIndex].score)
      cout << "CRAP THEY SCORED ???" << endl << flush;
    this->game_state_->opponentScore=gc.teams[oppIndex].score;

    this->game_state_->frameReceived = this->frame_info_->frame_id;

    if(gcTimer_.elapsed_s() > SECONDS_PER_PACKET) {
      this->sendGameControllerUDP();
      gcTimer_.restart();
    }
 
  }
  return;

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
