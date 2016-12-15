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
#include <communications/SPLStandardMessage.h>


#define MAX_MEM_WRITE_SIZE MAX_STREAMING_MESSAGE_LEN

#define PACKETS_PER_SECOND 3
#define SECONDS_PER_PACKET (1.0f/PACKETS_PER_SECOND)
#define FRAMES_PER_PACKET (30/PACKETS_PER_SECOND)
#define PACKET_INVALID_DELAY 10
#define LOC_INVALID_DELAY 10

using namespace static_math;
using SPLPacketView = SPLStandardMessage;

bool* CommunicationModule::interpreter_restart_requested_(NULL);

CommunicationModule::CommunicationModule(VisionCore *core):
  teamUDP(NULL), coachUDP(NULL), toolUDP(NULL), gcDataUDP(NULL), gcReturnUDP(NULL),
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
  vector<UDPWrapper**> connections = { &teamUDP, &coachUDP, &toolUDP, &gcDataUDP, &gcReturnUDP };
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
  if(robot_state_->WO_SELF == WO_TEAM_COACH)
    coachUDP = new UDPWrapper(SPL_COACH_MESSAGE_PORT,false,CommInfo::GAME_CONTROLLER_IP);
  toolUDP = new UDPWrapper(CommInfo::TOOL_UDP_PORT,broadcast,CommInfo::TOOL_LISTEN_IP);
  gcDataUDP = new UDPWrapper(GAMECONTROLLER_DATA_PORT,broadcast,CommInfo::GAME_CONTROLLER_IP);
  gcReturnUDP = new UDPWrapper(GAMECONTROLLER_RETURN_PORT,broadcast,CommInfo::GAME_CONTROLLER_IP);
  teamUDP->startListenThread(&CommunicationModule::listenTeamUDP,this);
  //coachUDP->startListenThread(&CommunicationModule::listenCoachUDP,this);
  toolUDP->startListenThread(&CommunicationModule::listenToolUDP,this);
  gcDataUDP->startListenThread(&CommunicationModule::listenGameControllerUDP,this);
  printf("Initialized communication wrappers--\n\tBroadcast IP: %s\n\tTeam UDP: %i\n", 
    CommInfo::TEAM_BROADCAST_IP.c_str(),
    CommInfo::TEAM_UDP_PORT
  );
  connected_ = true;
  coachTimer_.start();
  teamTimer_.start();
  gcTimer_.start();
}

void CommunicationModule::processFrame() {
  if(robot_state_->WO_SELF == WO_TEAM_LISTENER) return;
  if(robot_state_->WO_SELF == WO_TEAM_COACH)
    sendCoachUDP();
  else
    sendTeamUDP();
}

void CommunicationModule::listenCoachUDP() {
}

void CommunicationModule::sendCoachUDP() {
}

void CommunicationModule::sendGameControllerUDP() {
  RoboCupGameControlReturnData packet;
  packet.team = game_state_->gameContTeamNum;
  packet.player = robot_state_->WO_SELF;
  packet.message = GAMECONTROLLER_RETURN_MSG_ALIVE;
  bool res = gcReturnUDP->send(packet);
  if(!res) std::cerr << "Failed status message to Game Controller." << std::endl;
}

void CommunicationModule::listenTeamUDP() {
}

// send messages to teammates
void CommunicationModule::sendTeamUDP() {
}

void CommunicationModule::sendToolResponse(ToolPacket message) {
  toolUDP->sendToSender(message);
}

void CommunicationModule::sendToolRequest(ToolPacket message) {
  toolUDP->send(message);
}

void CommunicationModule::listenToolUDP() {
}

void CommunicationModule::handleCameraParamsMessage(CameraParams &params, char *msg) {
}

void CommunicationModule::listenGameControllerUDP() {
  RoboCupGameControlData gc;
  bool res = this->gcDataUDP->recv(gc);
  if(!res) return;
  if(this->robot_state_->ignore_comms_) return;

  //std::cout << "getting messages" << std::endl;

  // check version of packet
  if (gc.version != GAMECONTROLLER_STRUCT_VERSION){
    cout << endl << "ERROR: expect GameControllerPacket version: "
         << GAMECONTROLLER_STRUCT_VERSION
         << ", received: " << gc.version << endl << endl;
  }

  int teamNum = this->game_state_->gameContTeamNum;

  // First check if this packet is even meant for a robot on our team !
  if (gc.teams[0].teamNumber==teamNum || gc.teams[1].teamNumber==teamNum) {
    int teamIndex=0; // stores the index into the team array
    int oppIndex=1; // stores the index into the team array
    if (teamNum==gc.teams[1].teamNumber) {
      teamIndex=1;
      oppIndex=0;
    }

    int WO_SELF = this->robot_state_->WO_SELF;
    TeamInfo t=gc.teams[teamIndex];

    // process message from the coach
    CoachPacket& cp = this->team_packets_->cp;
    //cp.decodeMessage((const char*)gc.teams[this->robot_state_->team_].coachMessage);
    cp.decodeMessage((const char*)t.coachMessage);
    static int lastid = -1;
    if(lastid != cp.id) {
      cp.frameUpdated = this->frame_info_->frame_id;
    }
    lastid = cp.id;

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
      // if robot is coach or not penalized, set its state based on GC message
      if (WO_SELF == WO_TEAM_COACH || r.penalty == PENALTY_NONE) {
        switch(gc.state) {
          case STATE_INITIAL: this->game_state_->setState(INITIAL); break;
          case STATE_READY: this->game_state_->setState(READY); break;
          case STATE_SET: this->game_state_->setState(SET); break;
          case STATE_PLAYING: this->game_state_->setState(PLAYING); break;
          case STATE_FINISHED: this->game_state_->setState(FINISHED); break;
        }
        if(oldState != this->game_state_->state() && this->game_state_->state() == PLAYING && oldState != PENALISED)
          this->behavior_->timePlayingStarted = this->frame_info_->seconds_since_start - 20;

        // only continue for players (coach will cause index-out-of-bound errors
        // and we don't have anything else to update for coach anyway)
        if(WO_SELF > WO_TEAM_LAST || WO_SELF < WO_TEAM_FIRST) return;

      } else { // else this robot is penalized
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
    if (gc.secondaryState == STATE2_PENALTYSHOOT)
      this->game_state_->isPenaltyKick = true;
    else
      this->game_state_->isPenaltyKick = false;

    // random stuff
    this->game_state_->isFirstHalf=gc.firstHalf; // 1 = game in first half, 0 otherwise
    if (gc.kickOffTeam==this->game_state_->gameContTeamNum) this->game_state_->ourKickOff=true;
    else this->game_state_->ourKickOff=false;
    this->game_state_->lastOutBy = gc.dropInTeam;

    this->game_state_->dropInTime=gc.dropInTime;
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
    cleanupStreamThread();
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
