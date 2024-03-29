// Copyright 2021 Kenji Brameld
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef WALK__WALK_HPP_
#define WALK__WALK_HPP_

#include <functional>
#include "walk/XYZ_Coord.hpp"
#include "walk/ActionCommand.hpp"
#include "walk/Sensors.hpp"
#include "walk/BodyModel.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"



class Walk
{
public:
  Walk(rclcpp::Node* walkNode);
  void start();
  JointValues notifyJoints(const ActionCommand &command, const SensorValues &sensors, BodyModel &bodeyModel);
  geometry_msgs::msg::Twist get_odom() { return odometry; }
  
  rclcpp::Node* walkNode;

  enum Walk2014Option {
    STAND        = 0, // with knees straight and stiffness set to zero to conserve energy and heat generation in motors
    STANDUP      = 1, // process of moving from WALK crouch to STAND
    CROUCH       = 2, // process of transitioning from STAND to WALK
    WALK         = 3,
    READY        = 4, // stand still ready to walk
    KICK         = 5,
    NONE         = 6,
    NUMBER_OF_WALK_OPTIONS
  };

  enum WalkState {
    WALKING        = 0,
    STARTING       = 1,
    STOPPING       = 2,
    NOT_WALKING    = 3,
    NUMBER_OF_WALK_STATES
  };

  Walk2014Option walk2014Option;
  WalkState walkState;


private:

  // walk_msg::msg::Walk walk_command;
  // walk_sensor_msg::msg::Sensor sensor_readings;

  bool exactStepsRequested;

  ActionCommand active;

  // legacy code
  bool stopping;
  bool stopped;
  // time step, timers,
  float dt;
  float t;
  float globalTime;

  float timer;
  float T;                                                // period of half a walk cycle

  // Nao H25 V4 dimensions - from utils/body.hpp and converted to meters
  float thigh;                                            // thigh length in meters
  float tibia;                                            // tibia length in meters
  float ankle;                                            // height of ankle above ground

  // Walk 2014 parameters in meters and seconds
  float hiph;                                             // variable vertical distance ground to hip in meters
  float hiph0;                                            // some initial hiph
  float foothL;                                           // meters left foot is lifted off the ground
  float foothR;                                           // meters right foot is lifted off the ground
  float nextFootSwitchT;                                  // next time-point at which to change support foot
  float forward;                                          // Omnidirectional walk forward/backward
  float lastForward;                                      // previous forward value accepted
  float forwardL0, forwardL;                              // variable left foot position wrt standing
  float forwardR0, forwardR;                              // variable right foot position wrt standing
  float leftR;                                            // sideways step in meters for right foot
  float leftL;                                            // sideways step in meters for left  foot
  float left, lastLeft;                                   // Omnidirectional walk left/right
  float turn, lastTurn;                                   // Omnidirectional walk CW / CCW
  float power;                                            // Omnidirectional walk - reserved for kicking
  float bend;
  float speed;

  bool useShuffle;
  float stiffness;                                        // global stiffness (power to motors)
  float kneeStiffness;                                    // Specific dynamic knee stiffnesss to reduce overheating
  float ankleStiffness;                                   // Specific dynamic ankle stiffnesss to reduce overheating
  float currentVolume;                                    // Volume of the inputs requested
  float turnRL;                                           // turn variable
  float turnRL0;                                          // turnRL at support foot switch
  float lastStepLeft;                                     // left of swing foot at the start of that phase, used for recovering sidestep.
  bool supportFoothasChanged;                             // Indicates that support foot is deemed to have changed
  bool weightHasShifted;
  float comOffset;                                        // move in meters of CoM in x-direction when walking to spread weight more evenly over foot
  float comOffset0;
  float targetComOffset;
  bool shouldEmergencyStep;                               // whether an emergency step should be taken
  float currentTurnChange;
  bool ballXYDebug;

  float timerSinceLastBlock;                              // Timer to ensure stability after a blocking step

  // Gyro filters
  float filteredGyroX;
  float filteredGyroY;

  // Angle filters
  float filteredAngleY;

  // Balance Controller Variables
  float sagittalBalanceAdjustment;
  float coronalBalanceAdjustment;
  float sagittalHipBalanceAdjustment;

  // PD gyro controller
  float KpGyro = 0.07f;           // Proportional gain
  float KdGyro = 0.0003934f;           // Derivative gain
  float preErrorGyro;      // Previous tick gyro error

  // PID Angle controller
  float KpAngle = 0.2f;          // Proportional gain
  float KiAngle = 0.05f;          // Integral gain
  float KdAngle = 0.008f;          // Derivative gain
  float angleError;       // Current angle error
  float angleErrorSum;    // Error sum
  float preErrorAngle;    // Previous tick angle error

  // Kicks
  float kickT;
  float rock;
  float kneePitchL, kneePitchR, lastKneePitch;
  float anklePitchL, anklePitchR;
  float lastKickForward;
  float lastSide;
  float lastAnklePitch;
  float lastShoulderRollAmp;
  float lastFooth;
  float lastRock;
  float lastKickTime;
  float shoulderPitchL;                                   // to swing left  arm while walking / kicking
  float shoulderPitchR;                                   // to swing right arm while walking / kicking
  float shoulderRollL;
  float shoulderRollR;
  float dynamicSide;
  float turnAngle;
  float lastKickTurn;
  float kneePitchEnd;
  float anklePitchStart;
  float anklePitchEnd;
  float swingDelayFactor;
  bool holdAnkle;
  bool hipBalance;

  int tempCounter;

  // Kick parameter constants
  float stableCounter; // number of frames robot was stable for
  int kickStableNumFrames; // number of frames robot must be stable for
  float kickStableAngleX; // AngleX to consider stable before kicking (deg)
  float kickExtraStableAngleX; // AngleX to consider extra stable before kicking (deg)
  float kickExtraStableGyroscopeX; // GyroscopeX to consider extra stable before kicking (deg/s)
  float kickGyroscopeXOntoSupportFootThresh; // Maximum GyroscopeX value when rocking towards the support foot, to consider stable before kicking (deg/s)
  float kickGyroscopeXAwayFromSupportFootThresh; // Maximum GyroscopeX value when rocking away from the support foot, to consider stable before kicking (deg/s)
  float shiftPeriod; // time to shift weight on to one leg
  float shiftEndPeriod; // time to shift weight back from one leg
  float backPhase; // time to move kick foot back
  float kickPhase; // time to swing kick foot
  float throughPhase; // time to hold kick foot
  float endPhase; // time to return kick foot to zero position
  float shoulderRollAmpMultiplier; // arm roll to leave room for kicking
  float kickLean; // the base amount of lean when kicking - defined in options.cpp
  float kickLeanOffsetL; // kick lean offset when kicking with left foot for different robots
  float kickLeanOffsetR; // kick lean offset when kicking with right foot for different robots

  bool runningKickCalibration;

  //for odometry updates
  float prevTurn;
  float prevForwardL;
  float prevForwardR;
  float prevLeftL;
  float prevLeftR;
  geometry_msgs::msg::Twist odometry;
  
  bool isKicking; //whether we are kicking or not
  bool leftFootKick;
  


// Use for iterative inverse kinematics for turning (see documentation BH 2010)
struct Hpr {
  float Hp;
  float Hr;
  // Hpr(): Hp(0.0f), Hr(0.0f) { }
};

/**
* returns smooth values in the range 0 to 1 given time progresses from 0 to period
*/
float parabolicReturn(float); // step function with deadTimeFraction/2 delay
float parabolicReturnMod(float); // step function with deadTimeFraction/2 delay
float parabolicStep(float time, float period, float deadTimeFraction);                  // same as above, except a step up
float linearStep(float time, float period);                                             // linear increase from 0 to 1
float interpolateSmooth(float start, float end, float tCurrent, float tEnd);            // sinusoidal interpolation
float squareSmooth(float start, float end, float tCurrent, float tEnd);            //
float calculateKneeStiffness(float volume);
float calculateAnkleStiffness(float volume);


// Foot to Body coord transform used to calculate IK foot position and ankle tilts to keep foot in ground plane when turning
 XYZ_Coord mf2b(float Hyp, float Hp, float Hr, float Kp, float Ap,
                  float Ar, float xf, float yf, float zf);
 Hpr hipAngles(float Hyp, float Hp, float Hr, float Kp, float Ap,
                 float Ar, float xf, float yf, float zf, XYZ_Coord e);

 float clamp(const int min, float value, const int max) const;
};
#endif  // KICK__KICK_HPP_
