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

#include <math/Vector3.h>
#include <math/Vector2.h>
#include <common/RobotInfo.h>
#include <cstring>

class RobotDimensions
{
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
  float values_[NUM_DIMENSIONS];
  static bool isAngle_[NUM_DIMENSIONS];

  RobotDimensions() {
    //setNao33();
    setNaoH21();
  }

  std::vector<Vector3<float> > getLeftFSROffsets() {
    std::vector<Vector3<float> > offsets;
    Vector3<float> 
      lfl(values_[FSR_LFL_Offset1], values_[FSR_LFL_Offset2], values_[FSR_LFL_Offset3]),
      lfr(values_[FSR_LFR_Offset1], values_[FSR_LFR_Offset2], values_[FSR_LFR_Offset3]),
      lrl(values_[FSR_LRL_Offset1], values_[FSR_LRL_Offset2], values_[FSR_LRL_Offset3]),
      lrr(values_[FSR_LRR_Offset1], values_[FSR_LRR_Offset2], values_[FSR_LRR_Offset3]);
    offsets.push_back(lfl);
    offsets.push_back(lfr);
    offsets.push_back(lrl);
    offsets.push_back(lrr);
    return offsets;
  }

  void setNaoH21() {
    // Nao v4 with no motorized hands
    memset(isAngle_, false, NUM_DIMENSIONS);
    isAngle_[torsoHipRoll] = true;
    values_[hipOffsetY] = 50;
    values_[torsoToHipZ] = 85; 
    values_[torsoToHip] = sqrtf(pow(values_[hipOffsetY],2) + pow(values_[torsoToHipZ],2));
    values_[torsoHipRoll] = atan2f(values_[hipOffsetY], values_[torsoToHipZ]);
		
    values_[lengthBetweenLegs] = 2 * values_[hipOffsetY];
	values_[upperLegLength] = 100;
	values_[lowerLegLength] = 102.90;
    values_[neckOffsetZ] = 126.5;
    values_[hipOffsetZ] = values_[torsoToHipZ];
    values_[torsoToHeadPan] = values_[neckOffsetZ];
    values_[zLegJoint1ToHeadPan] = 85.0 + 126.5; // not used
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
    values_[headTiltFactorBottomCamera] = 0; //1.5 * DEG_T_RAD;
    isAngle_[headTiltFactorBottomCamera] = true;
    values_[headTiltFactorTopCamera] = 0; //1.5 * DEG_T_RAD;
    isAngle_[headTiltFactorTopCamera] = true;
    values_[headRollFactorBottomCamera] = 0;
    isAngle_[headRollFactorBottomCamera] = true;
    values_[headRollFactorTopCamera] = 0;
    isAngle_[headRollFactorTopCamera] = true;
		values_[motionCycleTime] = 0.010;
    values_[footLength] = 160.0;
    values_[footWidth] = 75.0;
    values_[backOfFootToAnkle] = 0.33 * values_[footLength];
    values_[ankleToFootCenter] = (values_[footLength]/2.0)-values_[backOfFootToAnkle];
    
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


  void setCameraParameters(float tiltBotCam, float rollBotCam, float tiltTopCam, float rollTopCam) {
        values_[tiltOffsetToBottomCamera] += tiltBotCam;
        values_[rollOffsetToBottomCamera] += rollBotCam;
        values_[tiltOffsetToTopCamera] += tiltTopCam;
        values_[rollOffsetToTopCamera] += rollTopCam;
 
        std::cout << "***************New tilt offset to top cam: " << values_[tiltOffsetToTopCamera] << "\n";
  
  }

  void setHeadOffsets(float tilt, float pan) {
        values_[headTiltOffset] += tilt;
        values_[headPanOffset] += pan;

        std::cout << "*********** TILT << " << values_[headTiltOffset] << " PAN " << values_[headPanOffset] << "\n";
  }




};

static std::string DimensionNames[] = {
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

#endif
