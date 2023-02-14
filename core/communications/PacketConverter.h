#pragma once

//#include <iostream>
#include <communications/SPLStandardMessage.h>
#include <communications/Compressor.h>
#include <common/TeamPacket.h>

typedef TeamPacket TeamPacket;

/* This struct contains all of the info in the TeamPacket except
   for the info required already by the SPLStandardMessage. In
   other words, std_data contains all non-standard fields. The 
   two convert functions translate between TeamPacket objects and
   SPLStandard message objects.
   */
struct std_data {
    uint32_t sentTime;
    //LocStruct
    Eigen::Matrix<float, 2, 2, Eigen::DontAlign> ballCov;
    float robotSDX;
    float robotSDY;
    float sdOrient;
    //BehaviorStruct
    float whistleSd;
    float whistleScore;
    uint32_t whistleHeardFrame;
    //Balls from LocStruct (reordered for struct packing)
    std::array<int16_t,4> altBalls;
    //BehaviorStruct (cont.)
    uint16_t ballBid = 3000;
    uint16_t waitFrames = 0;
    int16_t targetX = 0;
    int16_t targetY = 0;
    uint8_t state = 1;
    uint8_t role = 2;
    uint8_t msgType = 0;
    uint8_t setPlayType = 0;

    uint8_t setPlayTargetPlayer;
    uint8_t bits = 0;
    //Other
    std::array<uint8_t,WO_TEAM_LAST> packetsMissed;
    int8_t robotIP = -1;
};

/// @ingroup communications
class PacketConverter {
  public:
    static TeamPacket convert(SPLStandardMessage& message) {

      TeamPacket output;
      thread_local Compressor<10'000> compressor;

      std_data outputdata = compressor.decompress<std_data>(message.data,
              message.numOfDataBytes, message.valid);

      
      //fill in nonstandard fields
      output.sentTime = outputdata.sentTime;
      output.locData.ballCov = outputdata.ballCov;
      output.locData.robotSDX = outputdata.robotSDX;
      output.locData.robotSDY = outputdata.robotSDY;
      output.locData.sdOrient = outputdata.sdOrient;
      output.bvrData.whistleSd = outputdata.whistleSd;
      output.bvrData.whistleScore = outputdata.whistleScore;
      output.bvrData.whistleHeardFrame = outputdata.whistleHeardFrame;
      output.locData.balls[2] = outputdata.altBalls[0];
      output.locData.balls[3] = outputdata.altBalls[1];
      output.locData.balls[4] = outputdata.altBalls[2];
      output.locData.balls[5] = outputdata.altBalls[3];
      output.bvrData.ballBid = outputdata.ballBid;
      output.bvrData.waitFrames = outputdata.waitFrames;
      output.bvrData.ballMissed = int(message.ballAge*30);
      output.bvrData.targetX = outputdata.targetX;
      output.bvrData.targetY = outputdata.targetY;
      output.bvrData.state = outputdata.state;
      output.bvrData.role = outputdata.role;
      output.bvrData.msgType = outputdata.msgType;
      output.bvrData.setPlayType = outputdata.setPlayType;
      output.bvrData.setPlayTargetPlayer = outputdata.setPlayTargetPlayer;
      output.bvrData.bits = outputdata.bits;
      output.packetsMissed = outputdata.packetsMissed;
      output.robotIP = outputdata.robotIP;
      //stop filling in nonstandard fields
      //start filling in SPLStandardMessage fields
      output.bvrData.setFallen(message.fallen);
      output.locData.robotX = message.pose[0];
      output.locData.robotY = message.pose[1];
      output.locData.setOrientation(message.pose[2]);
      Point2D relBalls = Point2D(message.ball[0],message.ball[1]);
      output.locData.setBallPos(relBalls.relativeToGlobal(output.locData.robotPos(),
                  output.locData.orientation()));
      //stop filling in SPLStandardMessage fields

      return output;
    }

    static SPLStandardMessage convert(TeamPacket message) {

      SPLStandardMessage output;
      thread_local Compressor<10'000> compressor;
      output.numOfDataBytes = SPL_STANDARD_MESSAGE_DATA_SIZE;

      //start filling in SPLStandardMessage fields
      output.fallen = message.bvrData.fallen();
      output.pose[0] = message.locData.robotX;
      output.pose[1] = message.locData.robotY;
      output.pose[2] = message.locData.orientation();
      Point2D balls = message.locData.ballPos();
      Point2D relBalls = balls.globalToRelative(message.locData.robotPos(),
              message.locData.orientation());
      if (message.bvrData.ballSeen()) {
        output.ball[0] = relBalls.x;
        output.ball[1] = relBalls.y;
        output.ballAge = message.bvrData.ballMissed/30.0;
      } else {
        output.ballAge = -1;
      }
      //stop filling in SPLStandardMessage fields
      //fill in std_data struct
      std_data outputdata;
      outputdata.sentTime = message.sentTime;
      outputdata.ballCov = message.locData.ballCov;
      outputdata.robotSDX = message.locData.robotSDX;
      outputdata.robotSDY = message.locData.robotSDY;
      outputdata.sdOrient = message.locData.sdOrient;
      outputdata.whistleSd = message.bvrData.whistleSd;
      outputdata.whistleScore = message.bvrData.whistleScore;
      outputdata.whistleHeardFrame = message.bvrData.whistleHeardFrame;
      outputdata.altBalls[0] = message.locData.balls[2];
      outputdata.altBalls[1] = message.locData.balls[3];
      outputdata.altBalls[2] = message.locData.balls[4];
      outputdata.altBalls[3] = message.locData.balls[5];
      outputdata.ballBid = message.bvrData.ballBid;
      outputdata.waitFrames = message.bvrData.waitFrames;
      outputdata.targetX = message.bvrData.targetX;
      outputdata.targetY = message.bvrData.targetY;
      outputdata.state = message.bvrData.state;
      outputdata.role = message.bvrData.role;
      outputdata.msgType = message.bvrData.msgType;
      outputdata.setPlayType = message.bvrData.setPlayType;
      outputdata.setPlayTargetPlayer = message.bvrData.setPlayTargetPlayer;
      outputdata.bits = message.bvrData.bits;
      outputdata.packetsMissed = message.packetsMissed;
      outputdata.robotIP = message.robotIP;
      //stop filling out std_data struct

      compressor.compress(outputdata, output.data, output.numOfDataBytes);

      return output;
    }
};
