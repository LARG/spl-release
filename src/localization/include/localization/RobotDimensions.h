/**
 * @file RobotDimensions.h
 *
 * Description of the Dimensions of the Kondo Robot
 *
 * @author Cord Niehaus
 * Edited by Sam Barrett
 */

#ifndef __RobotDimensions_H__
#define __RobotDimensions_H__

#define RAD_T_DEG  180.0/3.141592653589793f
#define DEG_T_RAD  3.141592653589793f/180.0f

#define SIGMA 1.0

// piyushk: added these #defines for vision port
#define HIPOFFSETZ 85.0
#define NECKOFFSETZ 230

#define LUT_BIT 7
#define LUT_SIZE 128*128*128

#define SIM_IMAGE_X 320
#define SIM_IMAGE_Y 240
#define SIM_IMAGE_SIZE (SIM_IMAGE_X * SIM_IMAGE_Y * 3)

#define NUM_SONAR_VALS 10

#include <Eigen/Dense>
#include <cstring>
#include <string>
#include <cmath>
#include <vector>
#include <sstream>
#include <iostream>


#include "nao_sensor_msgs/msg/joint_indexes.hpp"

using nao_sensor_msgs::msg::JointIndexes;

class RobotDimensions {
 public:
  enum {
    hipOffsetY,
    lengthBetweenLegs,
    upperLegLength,
    lowerLegLength,
    torsoHipRoll,
    torsoToHip,
    torsoToHipZ,
    torsoToHeadPan,
    zLegJoint1ToHeadPan,
    hipOffsetZ,
    neckOffsetZ,
    armOffset1,
    armOffset2,
    armOffset3,
    upperArmLength,
    lowerArmLength,
    elbowOffsetY,
    footHeight,
    headTiltOffset,
    headPanOffset,
    xHeadTiltToBottomCamera,
    zHeadTiltToBottomCamera,
    xHeadTiltToTopCamera,
    zHeadTiltToTopCamera,
    tiltOffsetToBottomCamera,
    rollOffsetToBottomCamera,
    yawOffsetToBottomCamera,
    tiltOffsetToTopCamera,
    rollOffsetToTopCamera,
    yawOffsetToTopCamera,
    headTiltFactorBottomCamera,
    headTiltFactorTopCamera,
    headRollFactorBottomCamera,
    headRollFactorTopCamera,
    motionCycleTime,
    footLength,
    footWidth,
    backOfFootToAnkle,
    ankleToFootCenter,
    FSR_LFL_Offset1,
    FSR_LFL_Offset2,
    FSR_LFL_Offset3,
    FSR_LFR_Offset1,
    FSR_LFR_Offset2,
    FSR_LFR_Offset3,
    FSR_LRL_Offset1,
    FSR_LRL_Offset2,
    FSR_LRL_Offset3,
    FSR_LRR_Offset1,
    FSR_LRR_Offset2,
    FSR_LRR_Offset3,
    NUM_DIMENSIONS
  };
  std::array<float, NUM_DIMENSIONS> values_;
  std::array<bool, NUM_DIMENSIONS> isAngle_;

  RobotDimensions() {
    setNaoH21();
  }

  std::vector<Eigen::Vector3d> getLeftFSROffsets() {
    std::vector<Eigen::Vector3d> offsets;
    Eigen::Vector3d
      lfl(values_[FSR_LFL_Offset1],
          values_[FSR_LFL_Offset2],
          values_[FSR_LFL_Offset3]),
      lfr(values_[FSR_LFR_Offset1],
          values_[FSR_LFR_Offset2],
          values_[FSR_LFR_Offset3]),
      lrl(values_[FSR_LRL_Offset1],
          values_[FSR_LRL_Offset2],
          values_[FSR_LRL_Offset3]),
      lrr(values_[FSR_LRR_Offset1],
          values_[FSR_LRR_Offset2],
          values_[FSR_LRR_Offset3]);
    offsets.push_back(lfl);
    offsets.push_back(lfr);
    offsets.push_back(lrl);
    offsets.push_back(lrr);
    return offsets;
  }

  void setNaoH21() {
    // Nao v4 with no motorized hands
    isAngle_.fill(false);
    isAngle_[torsoHipRoll] = true;
    values_[hipOffsetY] = 50;
    values_[torsoToHipZ] = 85;
    values_[torsoToHip] = sqrtf(pow(values_[hipOffsetY], 2) +
        pow(values_[torsoToHipZ], 2));
    values_[torsoHipRoll] = atan2f(values_[hipOffsetY], values_[torsoToHipZ]);
    values_[lengthBetweenLegs] = 2 * values_[hipOffsetY];
    values_[upperLegLength] = 100;
    values_[lowerLegLength] = 102.90;
    values_[neckOffsetZ] = 126.5;
    values_[hipOffsetZ] = values_[torsoToHipZ];
    values_[torsoToHeadPan] = values_[neckOffsetZ];
    values_[zLegJoint1ToHeadPan] = 85.0 + 126.5;  // not used
    values_[armOffset1] = 0;
    values_[armOffset2] = 98;
    values_[armOffset3] = 126.5;
    values_[upperArmLength] = 105;
    values_[lowerArmLength] = 57.75 + 55.95;
    values_[elbowOffsetY] = 15.0;
    values_[footHeight] = 45.19;
    values_[headTiltOffset] = 0;
    values_[headPanOffset] = 0;
    values_[xHeadTiltToBottomCamera] = 50.71;
    values_[zHeadTiltToBottomCamera] = 17.74;
    values_[xHeadTiltToTopCamera] = 58.71;
    values_[zHeadTiltToTopCamera] = 63.64;
    values_[tiltOffsetToBottomCamera] = 39.7 * DEG_T_RAD;
    isAngle_[tiltOffsetToBottomCamera] = true;
    values_[rollOffsetToBottomCamera] = 0;
    isAngle_[rollOffsetToBottomCamera] = true;
    values_[yawOffsetToBottomCamera] = 0;
    isAngle_[yawOffsetToBottomCamera] = true;
    values_[tiltOffsetToTopCamera] = 1.2 * DEG_T_RAD;
    isAngle_[tiltOffsetToTopCamera] = true;
    values_[rollOffsetToTopCamera] = 0;
    isAngle_[rollOffsetToTopCamera] = true;
    values_[yawOffsetToTopCamera] = 0;
    isAngle_[yawOffsetToTopCamera] = true;
    values_[headTiltFactorBottomCamera] = 0;  // 1.5 * DEG_T_RAD;
    isAngle_[headTiltFactorBottomCamera] = true;
    values_[headTiltFactorTopCamera] = 0;  // 1.5 * DEG_T_RAD;
    isAngle_[headTiltFactorTopCamera] = true;
    values_[headRollFactorBottomCamera] = 0;
    isAngle_[headRollFactorBottomCamera] = true;
    values_[headRollFactorTopCamera] = 0;
    isAngle_[headRollFactorTopCamera] = true;
    values_[motionCycleTime] = 0.010;
    values_[footLength] = 160.0;
    values_[footWidth] = 75.0;
    values_[backOfFootToAnkle] = 0.33 * values_[footLength];
    values_[ankleToFootCenter] = (values_[footLength]/2.0) -
      values_[backOfFootToAnkle];
    values_[FSR_LFL_Offset1] = 70.25f;
    values_[FSR_LFL_Offset2] = 29.9f;
    values_[FSR_LFL_Offset3] = 0;
    values_[FSR_LFR_Offset1] = 70.25f;
    values_[FSR_LFR_Offset2] = 29.9f;
    values_[FSR_LFR_Offset3] = 0;
    values_[FSR_LRL_Offset1] = -30.25f;
    values_[FSR_LRL_Offset2] = 29.9f;
    values_[FSR_LRL_Offset3] = 0;
    values_[FSR_LRR_Offset1] = -29.65f;
    values_[FSR_LRR_Offset2] = -19.1f;
    values_[FSR_LRR_Offset3] = 0;
  }


  void setCameraParameters(float tiltBotCam, float rollBotCam, float tiltTopCam,
      float rollTopCam) {
    values_[tiltOffsetToBottomCamera] += tiltBotCam;
    values_[rollOffsetToBottomCamera] += rollBotCam;
    values_[tiltOffsetToTopCamera] += tiltTopCam;
    values_[rollOffsetToTopCamera] += rollTopCam;
  }

  void setHeadOffsets(float tilt, float pan) {
    values_[headTiltOffset] += tilt;
    values_[headPanOffset] += pan;
  }

  /* From RobotInfo.h*/
  // converted from ENUM_CLASS macro
  enum class SupportBase {
    SensorFoot,
    LeftFoot,
    RightFoot,
    TorsoBase
  };

  enum {
    HeadPan = JointIndexes::HEADYAW,
    HeadTilt = JointIndexes::HEADPITCH,
    NUM_HEAD_JOINTS = 2,
    BODY_JOINT_OFFSET = NUM_HEAD_JOINTS,
    NUM_BODY_JOINTS = JointIndexes::NUMJOINTS - NUM_HEAD_JOINTS,
    ARM_JOINT_FIRST = JointIndexes::LSHOULDERPITCH,
    ARM_JOINT_LAST = JointIndexes::RELBOWROLL,
    NUM_ARM_JOINTS = ARM_JOINT_LAST - ARM_JOINT_FIRST + 1,
    LEG_JOINT_FIRST = JointIndexes::LHIPYAWPITCH,
    LEG_JOINT_LAST = JointIndexes::RANKLEROLL,
    NUM_LEG_JOINTS = LEG_JOINT_LAST - LEG_JOINT_FIRST + 1
  };

  // joint limits
  const float minJointLimits[JointIndexes::NUMJOINTS] = {
    DEG_T_RAD * (-119.5 + SIGMA),  // HeadYaw
    DEG_T_RAD * (-38.5 + SIGMA),  // HeadPitch

    DEG_T_RAD * (42.44 + SIGMA),  // LHipYawPitch
    DEG_T_RAD * (-21.74 + SIGMA),  // LHipRoll
    DEG_T_RAD * (-88.0 + SIGMA),  // LHipPitch
    DEG_T_RAD * (-5.29 + SIGMA),  // LKneePitch
    DEG_T_RAD * (-68.15 + SIGMA),  // LAnklePitch
    DEG_T_RAD * (-22.79 + SIGMA),  // LAnkleRoll

    DEG_T_RAD * (-65.62 + SIGMA),  // RHipYawPitch
    DEG_T_RAD * (-45.29 + SIGMA),  // RHipRoll
    DEG_T_RAD * (-88 + SIGMA),  // RHipPitch
    DEG_T_RAD * (-5.90 + SIGMA),  // RKneePitch
    DEG_T_RAD * (-67.97 + SIGMA),  // RAnklePitch
    DEG_T_RAD * (-44.06 + SIGMA),  // RAnkleRoll

    DEG_T_RAD * (-119.5 + SIGMA),  // LShoulderPitch
    DEG_T_RAD * (-18.0 + SIGMA),  // LShoulderRoll
    DEG_T_RAD * (-119.5 + SIGMA),  // LElbowYaw
    DEG_T_RAD * (-88.5 + SIGMA),  // LElbowRoll


    DEG_T_RAD * (-119.5 + SIGMA),  // RShoulderPitch
    DEG_T_RAD * (-76.0 + SIGMA),  // RShoulderRoll
    DEG_T_RAD * (-119.5 + SIGMA),  // RElbowYaw
    DEG_T_RAD * (2.0 + SIGMA)  // RElbowRoll
  };


  const float maxJointLimits[JointIndexes::NUMJOINTS] = {
    DEG_T_RAD * (119.5 - SIGMA),  // (120.0 - SIGMA) // HeadYaw,
    DEG_T_RAD * (29.5 - SIGMA),  // HeadPitch

    DEG_T_RAD * (-65.62 - SIGMA),  // LHipYawPitch
    DEG_T_RAD * (45.29 - SIGMA),  // LHipRoll
    DEG_T_RAD * (27.73 - SIGMA),  // LHipPitch
    DEG_T_RAD * (121.04 - SIGMA),  // LKneePitch
    DEG_T_RAD * (52.86 - SIGMA),  // LAnklePitch
    DEG_T_RAD * (44.06 - SIGMA),  // LAnkleRoll

    DEG_T_RAD * (42.44 - SIGMA),  // RHipYawPitch
    DEG_T_RAD * (21.74 - SIGMA),  // RHipRoll
    DEG_T_RAD * (27.73 - SIGMA),  // RHipPitch
    DEG_T_RAD * (121.47 - SIGMA),  // RKneePitch
    DEG_T_RAD * (53.40 - SIGMA),  // RAnklePitch
    DEG_T_RAD * (22.80 - SIGMA),  // RAnkleRoll

    DEG_T_RAD * (119.5 - SIGMA),  // LShoulderPitch
    DEG_T_RAD * (76.0 - SIGMA),  // LShoulderRoll
    DEG_T_RAD * (119.5 - SIGMA),  // LElbowYaw
    DEG_T_RAD * (-2.0 - SIGMA),  // LElbowRoll

    DEG_T_RAD * (119.5 - SIGMA),  // RShoulderPitch
    DEG_T_RAD * (18.0 - SIGMA),  // RShoulderRoll
    DEG_T_RAD * (119.5 - SIGMA),  // RElbowYaw
    DEG_T_RAD * (88.5 - SIGMA)  // RElbowRoll
  };

  const std::string DimensionNames(int index) {
    std::string names[] = {
      "hipOffsetY",
      "lengthBetweenLegs",
      "upperLegLength",
      "lowerLegLength",
      "torsoHipRoll",
      "torsoToHip",
      "torsoToHipZ",
      "torsoToHeadPan",
      "zLegJoint1ToHeadPan",
      "hipOffsetZ",
      "neckOffsetZ",
      "armOffset1",
      "armOffset2",
      "armOffset3",
      "upperArmLength",
      "lowerArmLength",
      "elbowOffsetY",
      "footHeight",
      "headTiltOffset",
      "headPanOffset",
      "xHeadTiltToBottomCamera",
      "zHeadTiltToBottomCamera",
      "xHeadTiltToTopCamera",
      "zHeadTiltToTopCamera",
      "tiltOffsetToBottomCamera",
      "rollOffsetToBottomCamera",
      "yawOffsetToBottomCamera",
      "tiltOffsetToTopCamera",
      "rollOffsetToTopCamera",
      "yawOffsetToTopCamera",
      "headTiltFactorBottomCamera",
      "headTiltFactorTopCamera",
      "headRollFactorBottomCamera",
      "headRollFactorTopCamera",
      "motionCycleTime",
      "footLength",
      "footWidth",
      "backOfFootToAnkle",
      "ankleToFootCenter",
      "FSR_LFL_Offset1",
      "FSR_LFL_Offset2",
      "FSR_LFL_Offset3",
      "FSR_LFR_Offset1",
      "FSR_LFR_Offset2",
      "FSR_LFR_Offset3",
      "FSR_RFL_Offset1",
      "FSR_RFL_Offset2",
      "FSR_RFL_Offset3",
      "FSR_RFR_Offset1",
      "FSR_RFR_Offset2",
      "FSR_RFR_Offset3"
    };
    return names[index];
  }
};


// Change this Sensor enum based on the sensor list we are using
enum Sensors {
  gyroX,
  gyroY,
  gyroZ,
  accelX,
  accelY,
  accelZ,
  angleX,
  angleY,
  battery,
  fsrLFL,
  fsrLFR,
  fsrLRL,
  fsrLRR,
  fsrRFL,
  fsrRFR,
  fsrRRL,
  fsrRRR,
  bumperLL,
  bumperLR,
  bumperRL,
  bumperRR,
  centerButton,
  headFront,
  headMiddle,
  headRear,
  NUM_SENSORS
};

/* From RobotInfo.h */
// All the body parts that have weight
class BodyPart {
 public:
    enum Part {
    neck,
    head,
    top_camera,
    bottom_camera,
    left_shoulder,
    left_bicep,
    left_elbow,
    left_forearm,
    left_hand,
    right_shoulder,
    right_bicep,
    right_elbow,
    right_forearm,
    right_hand,
    left_pelvis,
    left_hip,
    left_thigh,
    left_tibia,
    left_ankle,
    left_foot,  // rotated at ankle (used for weight)
    left_sole,  // translated (used for pose)
    right_pelvis,
    right_hip,
    right_thigh,
    right_tibia,
    right_ankle,
    right_foot,
    right_sole,
    torso,
    virtual_base,
    NUM_PARTS
    };
};

// Location of key body parts
class BodyFrame {
 public:
  enum Part {
    origin,
    head,
    left_shoulder,
    left_elbow,
    left_wrist,
    right_shoulder,
    right_elbow,
    right_wrist,
    torso,
    left_hip,
    left_knee,
    left_ankle,
    left_foot,
    right_hip,
    right_knee,
    right_ankle,
    right_foot,
    left_foot_front_left,
    left_foot_front_right,
    left_foot_rear_left,
    left_foot_rear_right,
    right_foot_front_left,
    right_foot_front_right,
    right_foot_rear_left,
    right_foot_rear_right,
    NUM_POINTS,
  };
};

#endif
