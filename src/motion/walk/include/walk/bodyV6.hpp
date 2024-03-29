#ifndef BODY_V6_HPP
#define BODY_V6_HPP

#include <iostream>
#include <cmath>
#include <string>
//#include <utils/matrix_helpers.hpp>
#include "angles.hpp"


namespace Joints {
   // Codes for each joint as used by NaoQi
   typedef enum JointCodesEnum {
      HeadYaw = 0,
      HeadPitch,
      LShoulderPitch,
      LShoulderRoll,
      LElbowYaw,
      LElbowRoll,
      LWristYaw,
      LHipYawPitch,
      LHipRoll,
      LHipPitch,
      LKneePitch,
      LAnklePitch,
      LAnkleRoll,
      RHipRoll,
      RHipPitch,
      RKneePitch,
      RAnklePitch,
      RAnkleRoll,
      RShoulderPitch,
      RShoulderRoll,
      RElbowYaw,
      RElbowRoll,
      RWristYaw,
      LHand,
      RHand,
      NUMBER_OF_JOINTS
   } JointCode;

   const JointCode jointCodes[] = {
      HeadYaw,
      HeadPitch,
      LShoulderPitch,
      LShoulderRoll,
      LElbowYaw,
      LElbowRoll,
      LWristYaw,
      LHipYawPitch,
      LHipRoll,
      LHipPitch,
      LKneePitch,
      LAnklePitch,
      LAnkleRoll,
      RHipRoll,
      RHipPitch,
      RKneePitch,
      RAnklePitch,
      RAnkleRoll,
      RShoulderPitch,
      RShoulderRoll,
      RElbowYaw,
      RElbowRoll,
      RWristYaw,
      LHand,
      RHand,
   };

   // Note: Functions are defined at the end of this file...
   const std::string jointNames[] = {
      "HeadYaw",
      "HeadPitch",
      "LShoulderPitch",
      "LShoulderRoll",
      "LElbowYaw",
      "LElbowRoll",
      "LWristYaw",
      "LHipYawPitch",
      "LHipRoll",
      "LHipPitch",
      "LKneePitch",
      "LAnklePitch",
      "LAnkleRoll",
      "RHipRoll",
      "RHipPitch",
      "RKneePitch",
      "RAnklePitch",
      "RAnkleRoll",
      "RShoulderPitch",
      "RShoulderRoll",
      "RElbowYaw",
      "RElbowRoll",
      "RWristYaw",
      "LHand",
      "RHand",
   };

   const std::string fliteJointNames[] = {
      "Head Yaw",
      "Head Pitch",
      "L Shoulder Pitch",
      "L Shoulder Roll",
      "L Elbow Yaw",
      "L Elbow Roll",
      "L Wrist Yaw",
      "L Hip Yaw Pitch",
      "L Hip Roll",
      "L Hip Pitch",
      "L Knee Pitch",
      "L Ankle Pitch",
      "L Ankle Roll",
      "R Hip Roll",
      "R Hip Pitch",
      "R Knee Pitch",
      "R Ankle Pitch",
      "R Ankle Roll",
      "R Shoulder Pitch",
      "R Shoulder Roll",
      "R Elbow Yaw",
      "R Elbow Roll",
      "R Wrist Yaw",
      "L Hand",
      "R Hand",
   };

   // Limits for Joints in RADIANS
   namespace Radians {
      const float HeadYaw_Min          = DEG2RAD(-119.5);
      const float HeadYaw_Max          = DEG2RAD(119.5);

      const float HeadPitch_Min        = DEG2RAD(-38.5);
      const float HeadPitch_Max        = DEG2RAD(29.5);

      const float LShoulderPitch_Min   = DEG2RAD(-119.5);
      const float LShoulderPitch_Max   = DEG2RAD(119.5);

      const float LShoulderRoll_Min    = DEG2RAD(-18.0);
      const float LShoulderRoll_Max    = DEG2RAD(76.0);

      const float LElbowYaw_Min        = DEG2RAD(-119.5);
      const float LElbowYaw_Max        = DEG2RAD(119.5);

      const float LElbowRoll_Min       = DEG2RAD(-88.5);
      const float LElbowRoll_Max       = DEG2RAD(-2.0);

      const float LWristYaw_Min        = DEG2RAD(-104.5);
      const float LWristYaw_Max        = DEG2RAD(104.5);

      const float LHipYawPitch_Min     = DEG2RAD(-65.62);
      const float LHipYawPitch_Max     = DEG2RAD(42.44);

      const float LHipPitch_Min        = DEG2RAD(-88.0);
      const float LHipPitch_Max        = DEG2RAD(27.73);

      const float LHipRoll_Min         = DEG2RAD(-21.74);
      const float LHipRoll_Max         = DEG2RAD(45.29);

      const float LKneePitch_Min       = DEG2RAD(-5.29);
      const float LKneePitch_Max       = DEG2RAD(121.04);

      const float LAnklePitch_Min      = DEG2RAD(-68.15);
      const float LAnklePitch_Max      = DEG2RAD(52.86);

      const float LAnkleRoll_Min       = DEG2RAD(-22.79);
      const float LAnkleRoll_Max       = DEG2RAD(44.06);


      const float RHipPitch_Min        = DEG2RAD(-88.0);
      const float RHipPitch_Max        = DEG2RAD(27.73);

      const float RHipRoll_Min         = DEG2RAD(-45.29);
      const float RHipRoll_Max         = DEG2RAD(21.74);

      const float RKneePitch_Min       = DEG2RAD(-5.90);
      const float RKneePitch_Max       = DEG2RAD(121.47);

      const float RAnklePitch_Min      = DEG2RAD(-67.97);
      const float RAnklePitch_Max      = DEG2RAD(53.40);

      const float RAnkleRoll_Min       = DEG2RAD(-44.06);
      const float RAnkleRoll_Max       = DEG2RAD(22.80);


      const float RShoulderPitch_Min   = DEG2RAD(-119.5);
      const float RShoulderPitch_Max   = DEG2RAD(119.5);

      const float RShoulderRoll_Min    = DEG2RAD(-76.0);
      const float RShoulderRoll_Max    = DEG2RAD(18.0);

      const float RElbowYaw_Min        = DEG2RAD(-119.5);
      const float RElbowYaw_Max        = DEG2RAD(119.5);

      const float RElbowRoll_Min       = DEG2RAD(2.0);
      const float RElbowRoll_Max       = DEG2RAD(88.5);

      const float RWristYaw_Min        = DEG2RAD(-104.5);
      const float RWristYaw_Max        = DEG2RAD(104.5);

      const float LHand_Min            = 0.0;
      const float LHand_Max            = 1.0;

      const float RHand_Min            = 0.0;
      const float RHand_Max            = 1.0;

      // Array to hold the maximum and minimum boundaries
      const float MaxAngle[] = {
         HeadYaw_Max,
         HeadPitch_Max,
         LShoulderPitch_Max,
         LShoulderRoll_Max,
         LElbowYaw_Max,
         LElbowRoll_Max,
         LWristYaw_Max,
         LHipYawPitch_Max,
         LHipRoll_Max,
         LHipPitch_Max,
         LKneePitch_Max,
         LAnklePitch_Max,
         LAnkleRoll_Max,
         RHipRoll_Max,
         RHipPitch_Max,
         RKneePitch_Max,
         RAnklePitch_Max,
         RAnkleRoll_Max,
         RShoulderPitch_Max,
         RShoulderRoll_Max,
         RElbowYaw_Max,
         RElbowRoll_Max,
         RWristYaw_Max,
         LHand_Max,
         RHand_Max
      };

      const float MinAngle[] = {
         HeadYaw_Min,
         HeadPitch_Min,
         LShoulderPitch_Min,
         LShoulderRoll_Min,
         LElbowYaw_Min,
         LElbowRoll_Min,
         LWristYaw_Min,
         LHipYawPitch_Min,
         LHipRoll_Min,
         LHipPitch_Min,
         LKneePitch_Min,
         LAnklePitch_Min,
         LAnkleRoll_Min,
         RHipRoll_Min,
         RHipPitch_Min,
         RKneePitch_Min,
         RAnklePitch_Min,
         RAnkleRoll_Min,
         RShoulderPitch_Min,
         RShoulderRoll_Min,
         RElbowYaw_Min,
         RElbowRoll_Min,
         RWristYaw_Min,
         LHand_Min,
         RHand_Min
      };
   }

   // Motor speed reductions. These determine the maximum rotational
   // speed of the motors controlling the joint. It is 'highly' advised
   // that when controlling the motors these limits are taken into
   // consideration.  They have been placed in the same namespace as
   // their joint angle counter-parts as it makes sense given the
   // classification.
   // This is calculated as follows: No load speed / reduction ratio
   // / 60, then * 360 to get it into degrees as opposed to rotations.
   // Finally divide by 100 to get it into per 10ms. For example,
   // taking motor 1 type A,
   // (https://community.aldebaran-robotics.com/doc/1-14/family/nao_h25/motors_h25.html)
   // we get 8300 / 201.3 / 60 * 360 / 100. Since it's not a good idea
   // to push the motors to their limits, (overheating issues etc), we
   // only take 90% of this number, ie 2.23 (to 2 decimal places).

   // Motor limits in Radians
   namespace Radians {
      extern const float MOTOR1_REDUCTIONA_RAD;
      extern const float MOTOR1_REDUCTIONB_RAD;
      extern const float MOTOR2_REDUCTIONA_RAD;
      extern const float MOTOR2_REDUCTIONB_RAD;
      extern const float MOTOR3_REDUCTIONA_RAD;
      extern const float MOTOR3_REDUCTIONB_RAD;

      extern const float HeadPitchSpeed;
      extern const float HeadYawSpeed;
      extern const float ShoulderPitchSpeed;
      extern const float ShoulderRollSpeed;
      extern const float ElbowYawSpeed;
      extern const float ElbowRollSpeed;
      extern const float WristYawSpeed;
      extern const float HandSpeed;
      extern const float HipYawPitchSpeed;
      extern const float HipRollSpeed;
      extern const float HipPitchSpeed;
      extern const float KneePitchSpeed;
      extern const float AnklePitchSpeed;
      extern const float AnkleRollSpeed;

      extern const float MaxSpeed[NUMBER_OF_JOINTS];
   }


   /**
    * Given the specified joint, caps it at its limits if the angle exceeds the boundary
    * @param joint Joint to check the angle against
    * @param angle Angle to check (in RADIANS)
    */
   static inline float limitJointRadians(JointCode joint, const float &angle) {
      if (std::isnan(angle)) return 0.0;
      if (angle > Radians::MaxAngle[joint]) return Radians::MaxAngle[joint];
      if (angle < Radians::MinAngle[joint]) return Radians::MinAngle[joint];
      return angle;
   }

   void outputValues(std::ostream &, const float[NUMBER_OF_JOINTS]);
}

/**
 * Limb Masses and Lengths
 * Taken from the Naoqi documentation. Limb names as Naoqi uses.
 */

namespace V6Limbs {
   const float NeckOffsetZ = 126.50;
   const float ShoulderOffsetY = 98.00;
   const float ElbowOffsetY = 15.00;
   const float UpperArmLength = 105.00;
   const float LowerArmLength = 55.95;
   const float ShoulderOffsetZ = 100.00;
   const float HandOffsetX = 57.75;
   const float HandOffsetZ = 12.31;
   const float HipOffsetZ = 85.00;
   const float HipOffsetY = 50.00;
   const float ThighLength = 100.00;
   const float TibiaLength = 102.90;
   const float FootHeight = 45.19;

   const float Length[] = {
      NeckOffsetZ,
      ShoulderOffsetY,
      ElbowOffsetY,
      UpperArmLength,
      LowerArmLength,
      ShoulderOffsetZ,
      HandOffsetX,
      HandOffsetZ,
      HipOffsetZ,
      HipOffsetY,
      ThighLength,
      TibiaLength,
      FootHeight
   };

   const float TorsoMass = 1.04956;
   const float NeckMass = 0.07842; // Head Yaw
   const float HeadMass = 0.60533; // Head Pitch
   const float RightShoulderMass = 0.09304; // R Shoulder Pitch
   const float RightBicepMass = 0.15777; // R Shoulder Roll
   const float RightElbowMass = 0.06483; // R Elbow Yaw
   const float RightForearmMass = 0.07761; // R Elbow Roll
   const float RightHandMass = 0.18533; // R Wrist Yaw
   const float RightPelvisMass = 0.06981; // R Hip Yaw Pitch
   const float RightHipMass = 0.14053; // R Hip Roll
   const float RightThighMass = 0.38978; // R Hip Pitch
   const float RightTibiaMass = 0.30142; // R Knee Pitch
   const float RightAnkleMass = 0.13416; // R Ankle Pitch
   const float RightFootMass = 0.17184; // R Ankle Roll
   const float LeftShoulderMass = 0.09304; // L Shoulder Pitch
   const float LeftBicepMass = 0.15777; // L Shoulder Roll
   const float LeftElbowMass = 0.06483; // L Elbow Yaw
   const float LeftForearmMass = 0.07761; // L Elbow Roll
   const float LeftHandMass = 0.18533; // L Wrist Yaw
   const float LeftPelvisMass = 0.06981; // L Hip Yaw Pitch
   const float LeftHipMass = 0.14053; // L Hip Roll
   const float LeftThighMass = 0.38978; // L Hip Pitch
   const float LeftTibiaMass = 0.30142; // L Knee Pitch
   const float LeftAnkleMass = 0.13416; // L Ankle Pitch
   const float LeftFootMass = 0.17184; // L Ankle Roll

   const float Mass[] = {
      TorsoMass,
      NeckMass,
      HeadMass,
      RightShoulderMass,
      RightBicepMass,
      RightElbowMass,
      RightForearmMass,
      RightHandMass,
      RightPelvisMass,
      RightHipMass,
      RightThighMass,
      RightTibiaMass,
      RightAnkleMass,
      RightFootMass,
      LeftShoulderMass,
      LeftBicepMass,
      LeftElbowMass,
      LeftForearmMass,
      LeftHandMass,
      LeftPelvisMass,
      LeftHipMass,
      LeftThighMass,
      LeftTibiaMass,
      LeftAnkleMass,
      LeftFootMass,
   };

   // Location of centre of masses in mm. Note: these are converted from m
   // so that they match the unit of measurement of the lengths of the links
   // (see above).
   const float TorsoCoM[] = {-4.13, 0.09, 43.42, 1};
   const float NeckCoM[] = {-0.01, 0.14, -27.42, 1};
   const float HeadCoM[] = {-1.12, 0.03, 52.58, 1};
   const float RightShoulderCoM[] = {-1.65, 26.63, 0.14, 1};
   const float RightBicepCoM[] = {24.29, -9.52, 3.20, 1};
   const float RightElbowCoM[] = {-27.44, 0.00, -0.14, 1};
   const float RightForearmCoM[] = {25.52, -2.81, 0.90, 1};
   const float RightHandCoM[] = {34.34, -0.88, 3.08, 1};
   const float RightPelvisCoM[] = {-7.66, 12.00, 27.16, 1};
   const float RightHipCoM[] = {-15.49, -0.29, -5.16, 1};
   const float RightThighCoM[] = {1.39, -2.25, -53.74, 1};
   const float RightTibiaCoM[] = {3.94, -2.21, -49.38, 1};
   const float RightAnkleCoM[] = {0.45, -0.30, 6.84, 1};
   const float RightFootCoM[] = {25.40, -3.32, -32.39, 1};
   const float LeftShoulderCoM[] = {-1.65, -26.63, 0.14, 1};
   const float LeftBicepCoM[] = {24.55, 5.63, 3.30, 1};
   const float LeftElbowCoM[] = {-27.44, 0.00, -0.14, 1};
   const float LeftForearmCoM[] = {25.56, 2.81, 0.76, 1};
   const float LeftHandCoM[] = {34.34, -0.88, 3.08, 1};
   const float LeftPelvisCoM[] = {-7.81, -11.14, 26.61, 1};
   const float LeftHipCoM[] = {-15.49, 0.29, -5.15, 1};
   const float LeftThighCoM[] = {1.38, 2.21, -53.73, 1};
   const float LeftTibiaCoM[] = {4.53, 2.25, -49.36, 1};
   const float LeftAnkleCoM[] = {0.45, 0.29, 6.85, 1};
   const float LeftFootCoM[] = {25.42, 3.30, -32.39, 1};
}

namespace V6LEDs {
   enum EyeLEDs {
      EyeRed1,
      EyeRed2,
      EyeRed3,
      EyeRed4,
      EyeRed5,
      EyeRed6,
      EyeRed7,
      EyeRed8,
      EyeGreen1,
      EyeGreen2,
      EyeGreen3,
      EyeGreen4,
      EyeGreen5,
      EyeGreen6,
      EyeGreen7,
      EyeGreen8,
      EyeBlue1,
      EyeBlue2,
      EyeBlue3,
      EyeBlue4,
      EyeBlue5,
      EyeBlue6,
      EyeBlue7,
      EyeBlue8,
      NUM_EYE_LEDS,
   };
}

namespace Sensors {

   /**
    * foot FSR positions relative to ankle frame
    * 1st index specifies left or right, 2nd index specifies which of 4,
    * 3rd index specifies x or y axis
    */
   const float FSR_POS[2][4][2] = {
                                   { { 70.25, 29.9}, { 70.25,-23.1},
                                     {-30.25, 29.9}, {-29.65,-19.1} },
                                   { { 70.25, 23.1}, { 70.25,-29.9},
                                     {-30.25, 19.1}, {-29.65,-29.9} }
                                   };

   typedef enum SensorCodesEnum {

      Battery_Charge = 0,
      Battery_Status,
      Battery_Current,
      Battery_Temperature,

      InertialSensor_AccelerometerX,
      InertialSensor_AccelerometerY,
      InertialSensor_AccelerometerZ,

      InertialSensor_GyroscopeX,
      InertialSensor_GyroscopeY,
      InertialSensor_GyroscopeZ,

      InertialSensor_AngleX,
      InertialSensor_AngleY,

      SonarLeft,
      SonarRight,

      LFoot_FSR_FrontLeft,
      LFoot_FSR_FrontRight,
      LFoot_FSR_RearLeft,
      LFoot_FSR_RearRight,

      RFoot_FSR_FrontLeft,
      RFoot_FSR_FrontRight,
      RFoot_FSR_RearLeft,
      RFoot_FSR_RearRight,

      ChestBoard_Button,

      Head_Touch_Front,
      Head_Touch_Middle,
      Head_Touch_Rear,

      LFoot_Bumper_Left,
      LFoot_Bumper_Right,

      LHand_Touch_Back,
      LHand_Touch_Left,
      LHand_Touch_Right,

      RFoot_Bumper_Left,
      RFoot_Bumper_Right,

      RHand_Touch_Back,
      RHand_Touch_Left,
      RHand_Touch_Right,

      NUMBER_OF_SENSORS
   } SensorCode;

   const std::string sensorNames[] = {

      "Battery_Charge",
      "Battery_Current",
      "Battery_Status",
      "Battery_Temperature",

      "InertialSensor_AccelerometerX",
      "InertialSensor_AccelerometerY",
      "InertialSensor_AccelerometerZ",

      "InertialSensor_GyroscopeX",
      "InertialSensor_GyroscopeY",
      "InertialSensor_GyroscopeZ",

      "InertialSensor_AngleX",
      "InertialSensor_AngleY",

      "SonarLeft",
      "SonarRight",

      "LFoot_FSR_FrontLeft",
      "LFoot_FSR_FrontRight",
      "LFoot_FSR_RearLeft",
      "LFoot_FSR_RearRight",

      "RFoot_FSR_FrontLeft",
      "RFoot_FSR_FrontRight",
      "RFoot_FSR_RearLeft",
      "RFoot_FSR_RearRight",

      "ChestBoard_Button",

      "Head_Touch_Front",
      "Head_Touch_Middle",
      "Head_Touch_Rear",

      "LFoot_Bumper_Left",
      "LFoot_Bumper_Right",

      "LHand_Touch_Back",
      "LHand_Touch_Left",
      "LHand_Touch_Right",

      "RFoot_Bumper_Left",
      "RFoot_Bumper_Right",

      "RHand_Touch_Back",
      "RHand_Touch_Left",
      "RHand_Touch_Right"
   };
}

#endif // BODY_V6_HPP
