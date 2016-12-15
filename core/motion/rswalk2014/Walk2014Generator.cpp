/**
 * Walk2014Generator.cpp
 * BH 18th Jan 2014
 * The period of each foot-step is set by T. T generates the forcing function by alternatively lifting each foot
 * Control:
 * The change of support foot is driven by the ZMP switching sign, but must be > 3T/4. If > 3*T we switch to try revive
 * The front-back sway is controlled by ankle tilts proportional to the GyroY
 * The user specifies forward(m), left(m), turn(radians) to activate the walk.
 * If these values are all zero the robot stands with the motors turned down.
 * The CoM is moved forward when walking to position it closer to the center of the foot
 */

#include "Walk2014Generator.hpp" // Removed motion/generator - Josiah
#include <cmath>
#include <iostream>
#include "utils/angles.hpp"
#include "utils/body.hpp"
#include "utils/basic_maths.hpp"
#include "utils/speech.hpp"

#define SHIFT_PERIOD 2.0 // time to shift weight on to one leg
#define SHIFT_END_PERIOD 2.5 // time to shift weight back from one leg
#define KICK_LEAN 22.2 // sideways lean while kicking
#define KICK_STEP_HEIGHT 0.03 // how far to lift kicking foot
#define BACK_PHASE 0.65 // time to move kick foot back
#define MIN_KICK_PHASE 0.5 // time to swing kick foot
#define MAX_KICK_PHASE 0.2 // time to swing kick foot
#define THROUGH_PHASE 0.3 // time to hold kick foot
#define END_PHASE 0.6 // time to return kick foot to zero position
#define EPSILON 0.01       //10 mm
#define TURN_EPSILON 0.05  //2.8 degrees

using boost::program_options::variables_map;
using namespace std;
using namespace RSJoints;
using namespace RSSensors;

const float MM_PER_M  = 1000.0;                            // number of millimeters in one meter
const float CROUCH_STAND_PERIOD = 0.5;//0.5;                     // time in seconds to crouch
const float COM_OFFSET = 0.01;                             // center of mass offset in x direction in meters
 const float FORWARD_CHANGE = 0.15;//0.2                          // was 0.08. max change of 100mm/sec at each leg change to ratchet up/down
 const float LEFT_CHANGE = 0.2; 			   // Testing
 const float TURN_CHANGE = 0.8;				   // Testing
const float STAND_HIP_HEIGHT = 0.248;                      // for tall power saving stand
const float KNEE_PITCH_RANGE = DEG2RAD(60);                // the knee pitch range from standing to crouching
const float BASE_WALK_PERIOD = .23; //.23;                 // seconds to walk one step, ie 1/2 walk cycle
const float WALK_HIP_HEIGHT = .23;                         // Walk hip height - seems to work from .2 to .235
const float MAX_FORWARD = .3;//.3                              // meters
const float MAX_LEFT = .2;//.2                                 // meters
const float MAX_TURN = 0.87;//.87                                // radians
const float BASE_LEG_LIFT = 0.010;//0.010                         // meters

void ellipsoidClampWalk(float &forward, float &left, float &turn);
float evaluateWalkVolume(float x, float y, float z);


void checkNaN(float n, string s){
   if(n != n){
//      cout << s << " is NAN " << endl;
   }
}

Walk2014Generator::Walk2014Generator()
   :t(0.0f), z(0.0f), PI(3.1415927) {
   initialise();
   //llog(INFO) << "Walk2014Generator constructed" << std::endl;
}

Walk2014Generator::~Walk2014Generator() {
   //llog(INFO) << "Walk2014Generator destroyed" << std::endl;
}

void Walk2014Generator::initialise() {
   //llog(INFO) << "Walk2014 initializing" << endl;
   dt = 0.01;                                             // 100 Hz motion thread
   t = 0.0;                                               // initialise timers (in seconds)
   timer = 0.0;                                           // timer to crouch to walking height
   footSwitchT = 0.0;
   globalTime = 0;                                        // use for diagnostic purposes only
   T = BASE_WALK_PERIOD;                                  // seconds - the period of one step in a two step walk cycle
   stopping = false;                                      // legacy code for stopping robot?
   stopped = true;                                        // legacy code for stopped robot?
   leftL = leftR = lastLeft = left = 0.0;                 // Side-step for left, right foot, and (last) left command in meters
   turnRL = turnRL0 = 0;                                  // Initial turn variables for feet
   forwardL  = forwardR  = 0.0;                           // forward step per for left and right foot in meters
   forwardR0 = forwardL0 = 0;                             // initial last positions for left and right feet keep constant during next walk step
   forward = lastForward = 0.0;                           // Current and previous forward value
   shoulderPitchL = shoulderPitchR = 0;                   // arm swing while walking
   shoulderRollL  = shoulderRollR  = 0;                   // Not used yet
   hiph = hiph0 = STAND_HIP_HEIGHT;                       // make robot stand initially based on Stand command
   foothL = foothR = 0;                                   // robots feet are both on the ground initially
   thigh = Limbs::ThighLength/MM_PER_M;                   // thigh length in meters
   tibia = Limbs::TibiaLength/MM_PER_M;                   // tibia length in meters
   ankle = Limbs::FootHeight/MM_PER_M;                    // height of ankle above ground
   nextFootSwitchT = 0.0;                                 // next time-point to switch support foot (in seconds)
   stiffness = 0.9;                                       // initial motor stiffness
   walk2014Option = NONE;                                 // initial walk 2014 option
   walkState = NOT_WALKING;                               // initial walkState
   supportFoothasChanged = false;                         // triggers support foot change actions
   comOffset = 0;                                         // Center of Mass offset in sagittal plane used to spread weight along feet in x-dir
   prevTurn = prevForwardL = prevForwardR = 0;            // odometry
   prevLeftL = prevLeftR = 0;                             // odometry
   exactStepsRequested = false;
   
   armStuck = false; //testing: get out of arm being caught

   // Kick specific
   kickT = 0;
   kickPhase = MAX_KICK_PHASE;
   rock = 0;
   kneePitchL = kneePitchR = lastKneePitch = 0;
   anklePitchL = anklePitchR = 0;
   lastKickForward = 0;
   lastSide = 0;
   lastKickTime = T;
   dynamicSide = 0.0f;

}

void ellipsoidClampWalk(float &forward, float &left, float &turn, float speed) {
   const float MIN_SPEED = 0.0f;
   const float MAX_SPEED = 1.0f;
   speed = crop(speed, MIN_SPEED, MAX_SPEED);

   // limit max to 66-100% depending on speed
   float M_FORWARD = MAX_FORWARD * 0.66 + MAX_FORWARD * 0.34 * speed;
   float M_LEFT = MAX_LEFT * 0.66 + MAX_LEFT * 0.34 * speed;
   float M_TURN = MAX_TURN * 0.66 + MAX_TURN * 0.34 * speed;

   float clampedForward = crop(forward, -M_FORWARD, M_FORWARD);
   float clampedLeft = crop(left, -M_LEFT, M_LEFT);
   float clampedTurn = crop(turn, -M_TURN, M_TURN);

   // Values in range [-1..1]
   float forwardAmount = clampedForward / M_FORWARD;
   float leftAmount = clampedLeft / M_LEFT;
   float turnAmount = clampedTurn / M_TURN;

   float x = fabs(forwardAmount);
   float y = fabs(leftAmount);
   float z = fabs(turnAmount);

   // see if the point we are given is already inside the allowed walk params volume
   if (evaluateWalkVolume(x, y, z) > 1.0) {
      float scale = 0.5;
      float high = 1.0;
      float low = 0.0;

      // This is basically a binary search to find the point on the surface.
      for (unsigned i = 0; i < 10; i++) {
         // give priority to turn. keep it the same
         x = fabs(forwardAmount) * scale;
         y = fabs(leftAmount) * scale;

         if (evaluateWalkVolume(x, y, z) > 1.0) {
            float newScale = (scale + low) / 2.0;
            high = scale;
            scale = newScale;
         } else {
            float newScale = (scale + high) / 2.0;
            low = scale;
            scale = newScale;
         }
      }

      forwardAmount *= scale;
      leftAmount *= scale;
   }

   forward = M_FORWARD * forwardAmount;
   left = M_LEFT * leftAmount;
   turn = clampedTurn;
}

// x = forward, y = left, z = turn
float evaluateWalkVolume(float x, float y, float z) {
   // his affects the relationship between forward and left.
   float e = 1.05; //1.25

   // lower value allows turn to be higher with a high forward/left, higher values dont allow a high turn
   float n = 1;

   float r = 2.0 / e;
   float t = 2.0 / n;

   return pow(pow(x, r) + pow(y, r), (t / r)) + pow(z, t);
}


JointValues Walk2014Generator::makeJoints(ActionCommand::All* request,
                                          Odometry* odometry,
                                          const SensorValues &sensors,
                                          BodyModel &bodyModel,
                                          float ballX,
                                          float ballY) {
// 0. The very first time walk is called, the previous stand height could have been anything, so make sure we interpolate from that
   if (walk2014Option == NONE) {
      // Calculate the current hip height by checking how bent the knee is 
      hiph = sensors.joints.angles[LKneePitch] / KNEE_PITCH_RANGE * (WALK_HIP_HEIGHT - STAND_HIP_HEIGHT) + STAND_HIP_HEIGHT;
   }
   // 1. Read in new walk values (forward, left, turn, power) only at the start of a walk step cycle, ie when t = 0

   if (t == 0) {
      //WALK_HIP_HEIGHT = WALK_HIP_HEIGHT;
      active = request->body;
      lastForward = forward;
      forward       = (float)active.forward/MM_PER_M;    // in meters
      lastLeft      = left;                              // used to detect when the left walk parameter changes sign
      left          = (float)active.left/MM_PER_M;       // in meters
      turn          = active.turn;                       // in radians
      power         = active.power;                      // controls stiffness when standing and kicking when walking*
      bend          = active.bend;                       // knee-bend parameter
      speed         = active.speed;                      // used to distinguish between jabKick and walkKick
      foot          = active.foot;                       // kicking foot
      isFast        = active.isFast;
      if (stopping) {                                    // not used at present
      }
      else {
         stopped = false;                                  // (re)activate
      }
      // 1.0 For backwards compatibility with old interface (can be deleted when behaviours are updated)
      if(forward==0 and left==0 and turn==0 and power==0) bend=0;
      // 1.1 Scale back values to try to ensure stability. Just clipped for now (see wiki)
//      if(forward>MAX_FORWARD) forward = MAX_FORWARD; if(forward<-MAX_FORWARD) forward = -MAX_FORWARD;
//      if(left>MAX_LEFT) left = MAX_LEFT; if(left<-MAX_LEFT) left = -MAX_LEFT;
//      if(turn>MAX_TURN) turn = MAX_TURN; if(turn<-MAX_TURN) turn = -MAX_TURN;
//      for (int i=0; i < Joints::NUMBER_OF_JOINTS; i++) {
//        float temp = sensors.joints.temperaturs[i];
//        if(temp > 70)
//          speed = MIN(0.5,speed);
//        if(temp > 75)
//          speed = MIN(0.5,speed);
//      }
      if (!exactStepsRequested) {
        ellipsoidClampWalk(forward, left, turn, speed);
      }

      // 1.2 Modify T when sidestepping and turning
      T = BASE_WALK_PERIOD + 0.05*abs(left)/MAX_LEFT; // 0.1?

      // 1.3 jabKick - removed

      // 1.4 walkKick - removed
      //if (bodyModel.walkKick && (bodyModel.walkKickHeading > 1.3 || bodyModel.walkKickHeading < -1.3)) {
      //  T = 1.5*BASE_WALK_PERIOD;
      //}
      if (bodyModel.walkKick) {
        bodyModel.walkKickStart = true;
      } else {
        bodyModel.walkKickStart = false;
      }
      if (request->body.actionType == ActionCommand::Body::KICK and bodyModel.walkKick && not in_walk_kick_) { // Starting walk kick
        printf("Start walk kick %f\n",bodyModel.walkKickHeading);
//	cout << "Start walk kick" << endl;
        in_walk_kick_ = true;
        num_phase_changes_ = 0;
	request->body.actionType = ActionCommand::Body::WALK;
        // To set phase change limit
        if (bodyModel.walkKickLeftLeg == bodyModel.isLeftPhase) {
	  phase_change_limit_ = 1;
	} else if (bodyModel.walkKickLeftLeg != bodyModel.isLeftPhase) {
          phase_change_limit_ = 2;
	}/* else if (bodyModel.walkKickLeftLeg == bodyModel.isLeftPhase) {
          phase_change_limit_ = 1;
	} else { 
          phase_change_limit_ = 2;
	}*/
      } else if (request->body.actionType != ActionCommand::Body::KICK) {
          in_walk_kick_ = false;
          phase_change_limit_ = 10000;
          num_phase_changes_ = 0;
      } else if (not in_walk_kick_ or (not bodyModel.walkKick)){
          in_walk_kick_ = false;
          phase_change_limit_ = 10000;
          num_phase_changes_ = 0;
          request->body.actionType = ActionCommand::Body::WALK;
      } else { request->body.actionType = ActionCommand::Body::WALK;} // No matter what don't be in Kick past here

      if (not bodyModel.walkKick) {in_walk_kick_ = false;}

      // 1.5 ratchet forward by FORWARD_CHANGE
      if (!exactStepsRequested) {
        if (abs(forward-lastForward)>FORWARD_CHANGE) {                // ie greater than a FORWARD_CHANGE / sec change
          forward = lastForward + (forward-lastForward)/abs(forward-lastForward)*FORWARD_CHANGE;
        }
        if (abs(left-lastLeft) > LEFT_CHANGE) {
          left = lastLeft + (left-lastLeft)/abs(left-lastLeft)*LEFT_CHANGE;
        }
      }

      // 1.5 ratchet forward by FORWARD_CHANGE (uncomment to limit absolute change in "forward" to FORWARD_CHANGE)
//      // Cap forward change in speed
//      if (!exactStepsRequested && abs(forward-lastForward)>FORWARD_CHANGE && forward > lastForward ) {                
//      // ie greater than a FORWARD_CHANGE / sec change
//      // also, we only want to cap when *speeding up*, slowing down is fine      
//		forward = lastForward + (forward-lastForward)/abs(forward-lastForward)*FORWARD_CHANGE;
//      		}
//     lastForward   = forward;                           // back up old value in m/s
//     // testing: Cap left change in speed
//     if (!exactStepsRequested && abs(left-lastLeft)>LEFT_CHANGE && left > lastLeft) {                
//      		left = lastLeft + (left-lastLeft)/abs(left-lastLeft)*LEFT_CHANGE;
//      		}
//     lastLeft   = left;                           // back up old value in m/s
//     // testing: Cap turn change in speed
//     if (!exactStepsRequested && abs(turn-lastTurn)>TURN_CHANGE) {                // ie greater than a FORWARD_CHANGE / sec change
//      		turn = lastTurn + (turn-lastTurn)/abs(turn-lastTurn)*TURN_CHANGE;
//      		}
//     lastTurn   = turn;                           // back up old value in m/s


      // 1.6 Walk Calibration
      // The definition of forward, left and turn is the actual distance/angle traveled in one second
      // One walk-cycle consists of two Phases, a left phase (left swing foot) and a right phase (right swing foot)

      if (!exactStepsRequested) {
         forward *= 2*T; // theoretical calibration. 2 - because there are two steps per walk cycle
         left    *= 2*T;
         turn    *= 2*T;
         // linear calibration to achieve actual performance ie turn in action command achieves turn/sec in radians on the real robot
         forward *= 1.0;
         left    *= 0.82;
         turn    *= 1.43;
      }

      turn *= -1;   // reverses sign

   }

   // 2. Update timer
   t += dt;
   globalTime += dt;
   lastKickTime += dt;

   // 3. Determine Walk2014 Option
   if (request->body.actionType != ActionCommand::Body::KICK && kickT > 0 && request->body.actionType != ActionCommand::Body::REF_PICKUP) { 
      // We want to stop kicking, but we also don't want to let ref pick up take over during kick **HACK ALERT** this is particularly for mario whose foot sensor dies while kicking
      // Finish transition out if in the middle of a kick by skipping to the end phase
      if (kickT < BACK_PHASE + kickPhase + THROUGH_PHASE) {
         if (active.foot == ActionCommand::Body::LEFT) {
            lastKickForward = forwardL;
            lastKneePitch = kneePitchL;
            lastSide = leftL;
         } else {
            lastKickForward = forwardR;
            lastKneePitch = kneePitchR;
            lastSide = -leftR;
         }
         kickT = BACK_PHASE + kickPhase + THROUGH_PHASE;
      }
//comment this out so we never do rUNSWift slow kick
/*   } else if (active.actionType == ActionCommand::Body::KICK) {
      // This makes sure that the action type gets set back to walk just after a kick is finished.
      // If we don't leave enough time for this to happen, motion moves back into a kick before behaviour
      // can change its mind.
      if (lastKickTime < 2 * T) {
         request->body.actionType = ActionCommand::Body::WALK;
      } else if (abs(hiph - WALK_HIP_HEIGHT) < .0001) {                  // make sure we retain the designated walking height
         if (walk2014Option == WALK) { 
            // make sure walk is in neutral stance before kicking, L is symmetrical to R
            //            cout << "prepKick" << fabs(forwardL) << " " << fabs(leftL) << " " << turnRL << " " << t << endl;
            if (fabs(forwardL) < EPSILON && fabs(leftL) < EPSILON && fabs(turnRL) < TURN_EPSILON && t == dt) {
               // Assuming already at t = 0 from active getting set to kick
               // Any new settings the first time walk2014Option==KICK go here
               // Prep kick when the other foot is lifted
               if (active.foot == ActionCommand::Body::LEFT && !bodyModel.isLeftPhase) {
                  prepKick(true, bodyModel);
               } else if (active.foot != ActionCommand::Body::LEFT && bodyModel.isLeftPhase) {
                  prepKick(false, bodyModel);
               }
            }
         } else if(walk2014Option != KICK) {
            prepKick(active.foot == ActionCommand::Body::LEFT, bodyModel);
         }
      } else {                                               // hiph not= walkHipHeight
         if (walk2014Option != CROUCH) { // robot starts crouching to walking height
            hiph0 = hiph;
            timer = 0;
         }
         walk2014Option = CROUCH;                        // continue crouching
      }
*/
   } else if (walk2014Option == WALK && walkState != NOT_WALKING) {   // we are in the process of walking
      if (forward == 0 && left == 0 && turn == 0) {             // request to stop walking
         walkState = STOPPING;
      }
   } else if (bend == 0) {                                         // if we are not walking and wish to stand and power off
      if (abs(hiph - STAND_HIP_HEIGHT) < .0001) {               // and robot has reached stand height
         walk2014Option = STAND;                       // (keep) standing
      } else {                                             // if hiph not= standHipHeight
         if (walk2014Option != STANDUP) {
            hiph0 = hiph;
            timer = 0;
         }
         walk2014Option = STANDUP;                     // stand up first
      }
   } else if (forward == 0 && left == 0 && turn == 0 && bend == 1) { // not walking, but ready to go again, ie don't stand up
      if (abs(hiph - WALK_HIP_HEIGHT) < .0001) {
         walk2014Option = READY;
      } else {
         if (walk2014Option != CROUCH) { // robot starts crouching to walking height
            hiph0 = hiph;
            timer = 0;
         }
         walk2014Option = CROUCH;                        // continue crouching
      }
   } else {                                                  // if some walk parameters are non-zero
      if (abs(hiph - WALK_HIP_HEIGHT) < .0001) {                  // and we are at the designated walking height
         if(walk2014Option != WALK) {
            // Any new settings the first time walk2014Option==WALK go here (just for testing the walk)
            walkState = STARTING;
            nextFootSwitchT = T;
         }
         walk2014Option = WALK;                          // (keep) walking
      } else {                                               // hiph not= walkHipHeight
         if (walk2014Option != CROUCH) {                    // robot starts crouching to walking height
            hiph0 = hiph;
            timer = 0;
         }
         walk2014Option = CROUCH;                        // continue crouching
      }
   }
//   std::cout<< "Walk2014 Option: " << walk2014Option << " active: "<<active.actionType;
   // 4. Execute Walk2014 Option
   if (walk2014Option == STAND) {                             // Place CoM over ankle and turn set power to motors
      hiph = STAND_HIP_HEIGHT;
      forward = left = turn = 0;
      t = nextFootSwitchT = 0;
      stiffness = power;
      if (stiffness < 0.1) stiffness = 0.1;
      comOffset = 0;
   } else if (walk2014Option == STANDUP) {
      hiph = hiph0 + (STAND_HIP_HEIGHT - hiph0) * parabolicStep(timer, CROUCH_STAND_PERIOD, 0);
      forward = left = turn = 0;
      comOffset -= 0.02 * comOffset;                        // reduce offset to zero to allow stiffness to be turned down
      stiffness = 1;
      t = nextFootSwitchT = 0;
      timer += dt;
   } else if (walk2014Option == CROUCH){
      forward = left = turn = 0;
      stiffness = 1;
      hiph = hiph0 + (WALK_HIP_HEIGHT - hiph0) * parabolicStep(timer, CROUCH_STAND_PERIOD, 0);
      comOffset = COM_OFFSET * parabolicStep(timer, CROUCH_STAND_PERIOD, 0);   // move comOffset to 0.01 meters when walking
      t = nextFootSwitchT = 0;
      timer += dt;                                        // inc. option timer
   } else if (walk2014Option == WALK) {
      stiffness = 1; 
   } else if (walk2014Option == KICK) {
      stiffness = 1;
   }
   if (walk2014Option == READY) {
      forward = left = turn = 0;
      stiffness = power;
      if (stiffness < 0.4) stiffness = 0.4;                 // need enough stiffness to keep crouching posture
      t = nextFootSwitchT = 0;
   }

   // 5. Determine walk variables throughout the walk step phase
   if (walk2014Option == WALK and nextFootSwitchT > 0) {
      // 5.1 Calculate the height to lift each swing foot
      float maxFootHeight = BASE_LEG_LIFT + abs(forward)*0.01 + abs(left)*0.02;
      float varfootHeight = maxFootHeight*parabolicReturn(t/nextFootSwitchT);  // 0.012 lift of swing foot from ground
      // 5.2 - spare

      // 5.3L Calculate intra-walkphase forward, left and turn at time-step dt, for left swing foot
      if (bodyModel.isLeftPhase) {                         // if the support foot is right
         // 5.3.1L forward (the / by 4 is because the CoM moves as well and forwardL is wrt the CoM
         forwardR = forwardR0+((forward)/4-forwardR0)*linearStep(t,nextFootSwitchT);
         forwardL = forwardL0+parabolicStep(t,nextFootSwitchT,0)*(-(forward)/4-forwardL0); // swing-foot follow-through
         // 5.3.2L Jab-kick removed
         // Katie's walk kicks, adjusting forward
         if (in_walk_kick_ && bodyModel.walkKickStart && bodyModel.walkKickLeftLeg == true) {
           if (bodyModel.walkKickHeading == 0 && t > 0.1*T && t < 0.4*T) { // walk kick front with left foot
             forwardL = -0.12; // more negative -> foot more forward
           } else { // walk kick side with left foot (kick to right), need to test/tune more on a robot besides Arnold
             float forwardSide = -0.1;
             //forwardR = 0;
             foothR = 0;
             if (t > 0.03 && t <= 0.09) {  // first, move foot forward
               float numSteps = 0.06;
               float firstStep = 0.04;
               forwardL = (forwardSide)*((t-firstStep+0.01)/numSteps) + forwardL;
             } else if (t > 0.09 && t <= 0.15) {
               forwardL = forwardSide;
             } else if (t > 0.15 && t <= 0.18) {  // move foot back at the end
               float numSteps = 0.03;
               float firstStep = 0.16;
               forwardL = forwardL + (forwardSide*(1-((t-firstStep+0.01)/numSteps)));
             }
           }
         }
         // 5.3.3L spare
         // 5.3.4L left
         if (left > 0) {
            leftR =  leftAngle()/2;                     // share left between both feet, hence /2
            // if changing direction of left command, cancel swingAngle out of next left step
            if (lastLeft*left < 0) leftR -= swingAngle*(1-parabolicStep(t,nextFootSwitchT,0.1));
            leftL = -leftR;
         } else {
            leftL = swingAngle*(1-parabolicStep(t,nextFootSwitchT,0.0));
            leftR = -leftL;
         }
         if (in_walk_kick_ && bodyModel.walkKickStart && bodyModel.walkKickLeftLeg == true) {
           if (bodyModel.walkKickHeading != 0) { // walk kick side with left foot (kick to right)
             float leftInitial = -0.17;
             float leftKick = 0.02;
             if (t > 0.03 && t <= 0.06) {  // first, move foot left
               float numSteps = 0.03;
               float firstStep = 0.04;
               leftL = leftL + (leftInitial*((t-firstStep+0.01)/numSteps));
               leftR = -leftL;
             } else if (t > 0.06 && t <= 0.11) {
               leftL = leftInitial;
               leftR = -leftL;
             } else if (t > 0.11 && t <= 0.15) {  // in one time step, move right to kick
               leftL = leftKick;
               leftR = -leftL;
             } else if (t > 0.15 && t <= 0.18) {  // center leg again
               float numSteps = 0.03;
               float firstStep = 0.16;
               leftL = leftL + (leftKick*(1-((t-firstStep+0.01)/numSteps)));
               leftR = -leftL;
             }
           }
         }
         if (bodyModel.walkKickStart) { // walk kick side with left foot (kick to right)
           //cout << "t: " << t << ", forwardL: " << forwardL << ", leftL: " << leftL << endl;
         }

         // 5.3.5L turn
         if (turn < 0) turnRL = turnRL0 + (-.67*turn-turnRL0)*parabolicStep(t,nextFootSwitchT,0.0);
         else       turnRL = turnRL0 + (-.33*turn-turnRL0)*parabolicStep(t,nextFootSwitchT,0.0); //turn back to restore previous turn angle
         // 5.3.6L determine how high to lift the swing foot off the ground
         foothL = varfootHeight;                         // lift left swing foot
         foothR = 0;                                     // do not lift support foot;

         // Katie's walk kicks, adjusting foot height
         if (in_walk_kick_ && bodyModel.walkKickStart && bodyModel.walkKickLeftLeg == true && t < 0.2) {
           if (bodyModel.walkKickHeading == 0) {
             foothL = 0.04;
           } else {
             foothL = 0.02;
           }
         }
         if (in_walk_kick_ && bodyModel.walkKickStart && bodyModel.walkKickLeftLeg == true) {
           turnRL = 0.0;
         }
      }
      // 5.3R Calculate intra-walkphase forward, left and turn at time-step dt, for right swing foot
      if (!bodyModel.isLeftPhase) {                      // if the support foot is left
         // 5.3.1R forward
         forwardL = forwardL0+((forward)/4-forwardL0)*linearStep(t,nextFootSwitchT);
         forwardR = forwardR0+parabolicStep(t,nextFootSwitchT,0)*(-(forward)/4-forwardR0); // swing-foot follow-through
         // 5.3.2R Jab-Kick removed
         // Katie's walk kicks, adjusting forward
         if (in_walk_kick_ && bodyModel.walkKickStart && bodyModel.walkKickLeftLeg == false) {
           if (bodyModel.walkKickHeading == 0 && t > 0.1*T && t < 0.4*T) { // walk kick front with right foot
             forwardR = -0.12; // more negative -> foot more forward
             forwardL = forwardL + 0.03;
           }
         }
         // 5.3.3R spare
         // 5.3.4R left
         if (left < 0) {
            leftL = leftAngle()/2;                      // divide by 2 to share left between both feet
            // if changing direction of left command, cancel swingAngle out of next left step
            if (lastLeft*left < 0) leftL -= swingAngle*(1-parabolicStep(t,nextFootSwitchT,0.1));
            leftR = -leftL;
         } else {
            leftR = swingAngle*(1-parabolicStep(t,nextFootSwitchT,0.0));
            leftL = -leftR;
         }

         // 5.3.5R turn
         if (turn < 0) turnRL = turnRL0 + (.33*turn-turnRL0)*parabolicStep(t,nextFootSwitchT,0.0);
         else       turnRL = turnRL0 + (.67*turn-turnRL0)*parabolicStep(t,nextFootSwitchT,0.0);
         // 5.3.6R Foot height
         foothR = varfootHeight;
         foothL = 0;

         // Katie's walk kicks, adjusting foot height
         if (in_walk_kick_ && bodyModel.walkKickStart && bodyModel.walkKickLeftLeg == false && t < 0.7*T) {
           if (bodyModel.walkKickHeading == 0) { // walk kick front
             foothR = 0.04;
           }
         }
         if (in_walk_kick_ && bodyModel.walkKickStart && bodyModel.walkKickLeftLeg == false) {
           turnRL = 0.0;
         }
      }
      // 5.4 Special conditions when priming the walk
      if (walkState == STARTING) {
         turnRL = 0;                                     // don't turn on start of rocking
         foothL /= 3.5;                                  // reduce max lift due to short duration - may need to adjust this later
         foothR /= 3.5;                                  // "
         leftR = leftL = 0;                              // don't try to step left on starting and stopping
         forwardL = forwardR = 0;                        // don't move forward or backward
      }
      // 5.5 "natural" arm swing while walking/kicking to counterbalance foot swing
      shoulderPitchR = -forwardR * 6;                     // forwardR is in meters, 10 is an arbitrary scale factor to match previous walk
      shoulderPitchL = -forwardL * 6;
   } else if (walk2014Option == KICK) { // Kicking
      if (active.foot == ActionCommand::Body::LEFT) {
         makeForwardKickJoints(KICK_LEAN, KICK_STEP_HEIGHT, SHIFT_PERIOD, foothL, forwardL, leftL, kneePitchL, shoulderRollL, anklePitchL, ballY, request);
         leftR = leftL * 0.1; // Balance slightly more over support foot if need be
      } else { // with added adjustments for right side
         makeForwardKickJoints(-KICK_LEAN + 1.5, KICK_STEP_HEIGHT + 0.005, SHIFT_PERIOD, foothR, forwardR, leftR, kneePitchR, shoulderRollR, anklePitchR, ballY, request);
         leftR = -leftR; // switch signs for right side
         leftL = leftR * 0.12;
      }
   } else { // When walk option is not WALK or KICK
      foothL = foothR = 0;
   }

   // 6. Changing Support Foot. Note bodyModel.isLeftPhase means left foot is swing foot.
   //    t>0.75*T tries to avoid bounce, especially when side-stepping
   //    lastZMPL*ZMPL<0.0 indicates that support foot has changed
   //    t>3*T tires to get out of "stuck" situations
//   std::cout << t << endl;
   if (t > 3*T) { 
	std::cout << "phase expired, support foot changed, one foot in the air" << endl;
	armStuck = true;
    }
   if ((t > 0.75*T && bodyModel.lastZMPL*bodyModel.ZMPL < 0.0) || t > 3*T) supportFoothasChanged = true;
   if (supportFoothasChanged) bodyModel.setIsLeftPhase(bodyModel.ZMPL < 0); // set isLeft phase in body model for kinematics etc
   if (supportFoothasChanged && walk2014Option == WALK) {
      supportFoothasChanged = false;                      //reset
      footSwitchT = timer;
      // 6.1 Recover previous "left" swing angle
      if (bodyModel.isLeftPhase) swingAngle = leftL;
      else                      swingAngle = leftR;
      // 6.2 Decide on timing of next walk step phase
      if(walkState == NOT_WALKING){                       // Start the walk
         nextFootSwitchT = T;
         walkState = STARTING;
      } else if (walkState == STARTING) {
         nextFootSwitchT = T;
         walkState = WALKING;
      } else if (walkState == WALKING) {
         nextFootSwitchT = T;
         walkState = WALKING;                            // continue walking until interrupted by a command to stop (or kick?)
      } else if (walkState == STOPPING) {
         nextFootSwitchT = T;
         walkState = NOT_WALKING;
      } else {
         cout << "Should never get here: walkState error" << endl;
      }
      // 6.3 reset step phase time
      t = 0;                                              // reset step phase timer

      if (in_walk_kick_) num_phase_changes_ ++;

      if (num_phase_changes_ >= phase_change_limit_) {
	cout << "Reached limit" << endl;
        in_walk_kick_ = false;
        num_phase_changes_ = 1000;
        bodyModel.walkKick = false; 
      }
      // 6.4 backup values
      turnRL0 = turnRL;                                   // store turn value for use in next step
      forwardR0 = forwardR;                               // sagittal right foot
      forwardL0 = forwardL;                               // sagittal left foot
      // 6.5 Other stuff on support foot change
      // none at the moment
   } // end of changing support foot

   // 7. Sagittal Balance
   filteredGyroY = 0.75*filteredGyroY + 0.25*sensors.sensors[InertialSensor_GyrY];
   balanceAdjustment = filteredGyroY/25.0f; //25.0                  // adjust ankle tilt in proportion to filtered gryoY
   if (walk2014Option==READY) balanceAdjustment = 0;        // to stop swaying while not walking

   // 7.5 Coronal Balance
   filteredGyroX = 0.75*filteredGyroX + 0.25*sensors.sensors[InertialSensor_GyrX];
   coronalBalanceAdjustment = filteredGyroX/25.0f; //25.0            // adjust ankle roll in proportion to filtered gryoX

   // 8. Odometry update for localisation
   updateOdometry(odometry,bodyModel.isLeftPhase);

   // 9. Work out joint angles from walk variables above
   // 9.1 Left foot closed form inverse kinematics
   float leghL = hiph - foothL - ankle;                    // vertical height between ankle and hip in meters
   float legX0L = leghL / cos(leftL);                      // leg extension (eliminating knee) when forwardL = 0
   float legXL = sqrt(legX0L*legX0L
		   +(forwardL+comOffset)*(forwardL+comOffset));    //leg extension at forwardL

   float beta1L = acos((thigh*thigh+legXL*legXL-tibia*tibia)/(2.0f*thigh*legXL)); // acute angle at hip in thigh-tibia triangle
   float beta2L = acos((tibia*tibia+legXL*legXL-thigh*thigh)/(2.0f*tibia*legXL)); // acute angle at ankle in thigh-tibia triangle
   float tempL = legX0L/legXL; if(tempL>1.0f && (not in_walk_kick_ or bodyModel.walkKickStart == false)) tempL=1.0f;  // sin ratio to calculate leg extension pitch. If > 1 due to numerical error round down.
   float deltaL = asin(tempL);                             // leg extension angle
   float dirL = 1.0f; if ((forwardL+comOffset) > 0.0f) dirL = -1.0f; // signum of position of foot
   float HpL = beta1L + dirL*(M_PI/2.0f-deltaL);           // Hip pitch is sum of leg-extension + hip acute angle above
   float ApL = beta2L + dirL*(deltaL - M_PI/2.0f);         // Ankle pitch is a similar calculation for the ankle joint
   float KpL = HpL + ApL;                                  // to keep torso upright with both feet on the ground, the knee pitch is always the sum of the hip pitch and the ankle pitch.

   // 9.2 right foot closed form inverse kinematics (comments as above but for right leg)
   float leghR = hiph - foothR - ankle;
   float legX0R = leghR / cos(leftR);
   float legXR = sqrt(legX0R*legX0R
		   +(forwardR+comOffset)*(forwardR+comOffset));
   float dirR = 1.0f; if ((forwardR+comOffset) > 0.0f) dirR = -1.0f;
   float beta1R = acos((thigh*thigh+legXR*legXR-tibia*tibia)/(2.0f*thigh*legXR));
   float beta2R = acos((tibia*tibia+legXR*legXR-thigh*thigh)/(2.0f*tibia*legXR));
   float tempR = legX0R/legXR; if(tempR>1.0f && (not in_walk_kick_ or bodyModel.walkKickStart == false)) tempR=1.0f;
   float deltaR = asin(tempR);
   float HpR = beta1R + dirR*(M_PI/2.0f-deltaR);
   float ApR = beta2R + dirR*(deltaR - M_PI/2.0f);
   float KpR = HpR + ApR;
 
   // 9.3 Sert hip and ankle values
   float HrL = -leftL;
   float HrR = -leftR;
   float ArL = -HrL;
   float ArR = -HrR;
   if (walk2014Option == KICK) {
      HrL += rock;
      HrR += rock;
      ArL -= rock;
      ArR -= rock;
   }

   // 9.4 Adjust HpL, HrL, ApL, ArL LEFT based on Hyp turn to keep ankle in situ
   // Turning
   XYZ_Coord tL = mf2b(z, -HpL, HrL, KpL, -ApL, ArL, z, z, z);
   XYZ_Coord sL;
   float Hyp = -turnRL;
   for (int i = 0; i < 3; i++) {
      sL = mf2b(Hyp, -HpL, HrL, KpL, -ApL, ArL, z, z, z);
      XYZ_Coord e((tL.x-sL.x), (tL.y-sL.y), (tL.z-sL.z));
      Hpr hpr = hipAngles(Hyp, -HpL, HrL, KpL, -ApL, ArL, z, z, z, e);
      HpL -= hpr.Hp;
      HrL += hpr.Hr;
   }
   // ApL and ArL to make sure LEFT foot is parallel to ground
   XYZ_Coord up = mf2b(Hyp, -HpL, HrL, KpL, -ApL, ArL, 1.0f, 0.0f, 0.0f);
   XYZ_Coord ur = mf2b(Hyp, -HpL, HrL, KpL, -ApL, ArL, 0.0f, 1.0f, 0.0f);
   ApL = ApL + asin(sL.z-up.z);
   ArL = ArL + asin(sL.z-ur.z);

   // 9.5 Adjust HpR, HrR, ApR, ArR (RIGHT) based on Hyp turn to keep ankle in situ
   // Map to LEFT - we reuse the left foot IK because of symmetry right foot
   float Hr = -HrR;
   float Ar = -ArR;
   // Target foot origin in body coords
   XYZ_Coord tt = mf2b(z, -HpR, Hr, KpR, -ApR, Ar, z, z, z);
   XYZ_Coord s;
   Hyp = -turnRL;
   for (int i = 0; i < 3; i++) {
      s = mf2b(Hyp, -HpR, Hr, KpR, -ApR, Ar, z, z, z);
      XYZ_Coord e((tt.x-s.x), (tt.y-s.y), (tt.z-s.z));
      Hpr hpr = hipAngles(Hyp, -HpR, Hr, KpR, -ApR, Ar, z, z, z, e);
      HpR -= hpr.Hp;
      Hr += hpr.Hr;
   }
   // 9.6 Ap and Ar to make sure foot is parallel to ground
   XYZ_Coord u1 = mf2b(Hyp, -HpR, Hr, KpR, -ApR, Ar, 1.0f, 0.0f, 0.0f);
   XYZ_Coord u2 = mf2b(Hyp, -HpR, Hr, KpR, -ApR, Ar, 0.0f, 1.0f, 0.0f);
   ApR = ApR + asin(s.z-u1.z);
   Ar = Ar + asin(s.z-u2.z);
   // map back from left foot to right foot
   HrR = -Hr;
   ArR = -Ar;

   // 10. Set joint values and stiffness
   JointValues j = sensors.joints;
   for (uint8_t i = 0; i < RSJoints::NUMBER_OF_JOINTS; ++i) j.stiffnesses[i] = stiffness;

   // 10.1 Arms
   j.angles[LShoulderPitch] = DEG2RAD(90)+shoulderPitchL;
   j.angles[LShoulderRoll] = DEG2RAD(7)+shoulderRollL;
   j.angles[LElbowYaw] = DEG2RAD(0);
   j.angles[LElbowRoll] = DEG2RAD(0);
   j.angles[LWristYaw] = DEG2RAD(0);
   j.angles[RShoulderPitch] = DEG2RAD(90)+shoulderPitchR;
   j.angles[RShoulderRoll] = DEG2RAD(-7)-shoulderRollR;
   j.angles[RElbowYaw] = DEG2RAD(0);
   j.angles[RElbowRoll] = DEG2RAD(0);
   j.angles[RWristYaw] = DEG2RAD(0);
   j.stiffnesses[RSJoints::LShoulderPitch] = 0.75;
   j.stiffnesses[RSJoints::LShoulderRoll] = 0.75;
   j.stiffnesses[RSJoints::LElbowYaw] = 0.75;
   j.stiffnesses[RSJoints::LElbowRoll] = 0.75;
   j.stiffnesses[RSJoints::LWristYaw] = 0.75;
   j.stiffnesses[RSJoints::RShoulderPitch] = 0.75;
   j.stiffnesses[RSJoints::RShoulderRoll] = 0.75;
   j.stiffnesses[RSJoints::RElbowYaw] = 0.75;
   j.stiffnesses[RSJoints::RElbowRoll] = 0.75;
   j.stiffnesses[RSJoints::RWristYaw] = 0.75;


   // 10.2 Turn
   j.angles[RSJoints::LHipYawPitch] = -turnRL;

   // 10.3 Sagittal Joints
   j.angles[RSJoints::LHipPitch]    = -HpL;
   j.angles[RSJoints::RHipPitch]    = -HpR;
   j.angles[RSJoints::LKneePitch]   =  KpL;
   j.angles[RSJoints::RKneePitch]   =  KpR;
   // Only activate balance control if foot is on the ground
   j.angles[RSJoints::LAnklePitch]  = -ApL;
   j.angles[RSJoints::RAnklePitch]  = -ApR;
/*   if (bodyModel.walkKickHeading != 0 and in_walk_kick_ and (bodyModel.walkKickLeftLeg == bodyModel.isLeftPhase)) {
     balanceAdjustment -= DEG2RAD(10.0); // we tend to fall backwards during side kicks
     printf("Adjust for side kick\n");
   }*/
   if (walk2014Option == WALK && nextFootSwitchT>0 && (not in_walk_kick_ or bodyModel.walkKickStart == false)) {
      if(bodyModel.isLeftPhase) j.angles[RSJoints::RAnklePitch]  += balanceAdjustment;
      else                      j.angles[RSJoints::LAnklePitch]  += balanceAdjustment;
   } else if (walk2014Option == KICK) {
      if(bodyModel.isLeftPhase) j.angles[RSJoints::LAnklePitch]  += balanceAdjustment;
      else                      j.angles[RSJoints::RAnklePitch]  += balanceAdjustment;
   }

   // 10.4 Coronal Joints
   j.angles[RSJoints::LHipRoll]     =  HrL;
   j.angles[RSJoints::RHipRoll]     =  HrR;
   j.angles[RSJoints::LAnkleRoll]   =  ArL;
   j.angles[RSJoints::RAnkleRoll]   =  ArR;

   // Ruohan: testing coronal balance
//   if (walk2014Option == WALK && nextFootSwitchT>0) {
//      if(bodyModel.isLeftPhase) j.angles[RSJoints::RAnkleRoll]  += coronalBalanceAdjustment;
//      else                      j.angles[RSJoints::LAnkleRoll]  += coronalBalanceAdjustment;
//   } else if (walk2014Option == KICK) {
//      addKickJoints(j);
//      if(bodyModel.isLeftPhase) j.angles[RSJoints::LAnkleRoll]  += coronalBalanceAdjustment;
//      else                      j.angles[RSJoints::RAnkleRoll]  += coronalBalanceAdjustment;
//   } else {
//      j.angles[RSJoints::RAnkleRoll]  += coronalBalanceAdjustment;
//      j.angles[RSJoints::LAnkleRoll]  += coronalBalanceAdjustment;
//   }
   // testing end

   // Ruohan: testing hip balance
/*   if (bodyModel.walkKickStart == false) {
     j.angles[RSJoints::RHipPitch]  += balanceAdjustment;
     j.angles[RSJoints::LHipPitch]  += balanceAdjustment;
     j.angles[RSJoints::RHipRoll]  += coronalBalanceAdjustment;
     j.angles[RSJoints::LHipRoll]  += coronalBalanceAdjustment;
   }
*/
 //TODO

   // Add in joint adjustments for kicks
   if (walk2014Option == KICK) {
      addKickJoints(j);
   //   // Add in some coronal balancing
       if(bodyModel.isLeftPhase) j.angles[RSJoints::RAnkleRoll]  += coronalBalanceAdjustment;
       else                      j.angles[RSJoints::LAnkleRoll]  += coronalBalanceAdjustment;
   }

   //if (bodyModel.walkKick && bodyModel.walkKickLeftLeg == true && (bodyModel.walkKickHeading > 1.3 || bodyModel.walkKickHeading < -1.3)) {
   //  if (t <= 7) {
   //    j.angles[RSJoints::LHipRoll] = j.angles[RSJoints::LHipRoll] + 0.2;       
   //  } else if (t > 7 && T <= 14) {
   //    j.angles[RSJoints::LHipRoll] = j.angles[RSJoints::LHipRoll] - 0.1;       
   //  }
   //}

   //quick check if position went out of joint space
   for(int i = 0; i < RSJoints::NUMBER_OF_JOINTS; i++){
      checkNaN(j.angles[i], RSJoints::fliteJointNames[i]);
   }

   //limp arms when getting caught
//   if (armStuck && limp.elapsed_ms() >= 1000) {// (request->body.caughtLeft) {
//      limp.restart();
//   }
//   if (request->body.caughtRight) {
//      limp.restart();
//   }
   if (limp.elapsed_ms() < 6000 && armStuck  && (not in_walk_kick_ or bodyModel.walkKickStart == false)) { //500000
      j.stiffnesses[RSJoints::LShoulderPitch] = 0;
      j.stiffnesses[RSJoints::LShoulderRoll] = 0;
      j.stiffnesses[RSJoints::LElbowYaw] = 0;
      j.stiffnesses[RSJoints::LElbowRoll] = 0;
      j.stiffnesses[RSJoints::LWristYaw] = 0;
      j.stiffnesses[RSJoints::RShoulderPitch] = 0;
      j.stiffnesses[RSJoints::RShoulderRoll] = 0;
      j.stiffnesses[RSJoints::RElbowYaw] = 0;
      j.stiffnesses[RSJoints::RElbowRoll] = 0;
      j.stiffnesses[RSJoints::RWristYaw] = 0;
   }

   if (limp.elapsed_ms() < 500 && armStuck &&  (not in_walk_kick_ or bodyModel.walkKickStart == false)) { //500000
      if (bodyModel.isLeftPhase) {
      	// j.stiffnesses[RSJoints::RKneePitch] = 0.2;
      	j.angles[RSJoints::LKneePitch] -= 0.4;
      	j.angles[RSJoints::LAnklePitch] += 0.6;
      	// j.angles[RSJoints::LHipPitch] -= 0.0;
      }
      else {
      	//  j.stiffnesses[RSJoints::LKneePitch] = 0.2;
      	j.angles[RSJoints::RKneePitch] -= 0.4;
      	j.angles[RSJoints::RAnklePitch] += 0.6;
      	// j.angles[RSJoints::LHipPitch] -= 0.0;
      }

//      // if losing arm stiffness still cannot get out, robot will just fall
//        for (uint8_t i = 0; i < RSJoints::NUMBER_OF_JOINTS; ++i) j.stiffnesses[i] = 0;
   } 

   
//   std::cout << limp.elapsed_ms() << endl;
   if (limp.elapsed_ms() >= 6000 && armStuck &&  (not in_walk_kick_ or bodyModel.walkKickStart == false)) {
      // std::cout << "resume stiffness" << endl;
      for (uint8_t i = 0; i < RSJoints::NUMBER_OF_JOINTS; ++i) j.stiffnesses[i] = stiffness;
      j.stiffnesses[RSJoints::LShoulderPitch] = 0.75;
      j.stiffnesses[RSJoints::LShoulderRoll] = 0.75;
      j.stiffnesses[RSJoints::LElbowYaw] = 0.75;
      j.stiffnesses[RSJoints::LElbowRoll] = 0.75;
//      j.stiffnesses[RSJoints::LWristYaw] = 0.75;
      j.stiffnesses[RSJoints::RShoulderPitch] = 0.75;
      j.stiffnesses[RSJoints::RShoulderRoll] = 0.75;
      j.stiffnesses[RSJoints::RElbowYaw] = 0.75;
      j.stiffnesses[RSJoints::RElbowRoll] = 0.75;
//      j.stiffnesses[RSJoints::RWristYaw] = 0.75;

      limp.restart();
      armStuck = false;
   }


   return j;
}

void Walk2014Generator::prepKick(bool isLeft, BodyModel &bodyModel) {
   t = 0;
   walk2014Option = KICK;
   kickT = 0;
   if (isLeft) {
      lastKickForward = forwardL;
      lastKneePitch = kneePitchL;
      lastSide = leftL;
      bodyModel.setIsLeftPhase(true);
      forwardR = 0;
      foothR = 0;
   } else {
      lastKickForward = forwardR;
      lastKneePitch = kneePitchR;
      lastSide = -leftR;
      bodyModel.setIsLeftPhase(false);
      forwardL = 0;
      foothL = 0;
   }
}

void Walk2014Generator::makeForwardKickJoints(float kickLean, float kickStepH, float shiftPeriod, float &footh, float &forwardDist,
      float &side, float &kneePitch, float &shoulderRoll, float &anklePitch, float &ballY, ActionCommand::All* request) {

   kickT += dt;
   kickPhase = MIN_KICK_PHASE + (MAX_KICK_PHASE - MIN_KICK_PHASE) * power;
   float totalPhase = BACK_PHASE + kickPhase + THROUGH_PHASE + END_PHASE;

   // Update side position of ball as late as possible.
   if (request->body.misalignedKick) {
      dynamicSide = 0; // This will try to kick the robot with the outside of the foot.
   } else if (kickT < BACK_PHASE * 0.8) {
      dynamicSide = fabs(ballY) - 50; // offset to kick point on foot
   }
   // Max safe/useful value for lifting leg (without going past joint limits/balancing)
   float sideAmp = -MAX(0, MIN(60, dynamicSide)) / 200.0;
   float kickAmp = -0.07; // how far forward the kick should reach
   float shoulderRollAmp = -sideAmp / 2.5; // how much arm should lift to make room for raised kicking leg 

   // Shift weight over and swing foot back
   if (kickT < BACK_PHASE) {
      float totalShift = shiftPeriod / 4;
      float halfShift = totalShift / 2;
      // We spend the first part shifting our weight over.
      if (t < totalShift) {
         rock = DEG2RAD(kickLean) * parabolicStep(t, totalShift, 0);
         footh = kickStepH * parabolicStep(t, totalShift, 0);
      } else {
         turnRL = 0;
      }

      // Once we're halfway through shifting our weight, start moving the foot back.
      if (kickT >= halfShift) {
         float shiftedKickT = kickT - halfShift;
         forwardDist = interpolateSmooth(lastKickForward, -kickAmp * 0.9, shiftedKickT, BACK_PHASE - halfShift);
         side = interpolateSmooth(0, sideAmp, shiftedKickT, BACK_PHASE - halfShift);
         shoulderRoll = interpolateSmooth(0, shoulderRollAmp, shiftedKickT, BACK_PHASE - halfShift);
         kneePitch = interpolateSmooth(0, DEG2RAD(35) * 0.9, shiftedKickT, BACK_PHASE - halfShift);
         anklePitch = interpolateSmooth(0, DEG2RAD(5), shiftedKickT, BACK_PHASE - halfShift);
      }
   // Swing foot forward.
   } else if (kickT < (BACK_PHASE + kickPhase)) {
      forwardDist = interpolateSmooth(-kickAmp * 0.9, kickAmp, kickT - BACK_PHASE, kickPhase);                
      side = sideAmp;
      shoulderRoll = shoulderRollAmp;
      kneePitch = interpolateSmooth(DEG2RAD(35) * 0.9, DEG2RAD(-35), kickT - BACK_PHASE, kickPhase);
      anklePitch = -kneePitch;
      // Only start to flatten foot out at the halfway point
      if (forwardDist > 0) {
         anklePitch = DEG2RAD(5);
      }
   // Hold...
   } else if (kickT < (BACK_PHASE + kickPhase + THROUGH_PHASE)) {
      forwardDist = kickAmp;
      side = sideAmp;
      shoulderRoll = shoulderRollAmp;
      kneePitch = DEG2RAD(-35);
      anklePitch = -kneePitch;
      lastKickForward = forwardDist;
      lastKneePitch = kneePitch;
      lastSide = side;
   // Return foot.
   } else if (kickT < (BACK_PHASE + kickPhase + THROUGH_PHASE + END_PHASE)) {
      forwardDist = interpolateSmooth(lastKickForward, 0, kickT - BACK_PHASE - kickPhase - THROUGH_PHASE, END_PHASE);
      side = interpolateSmooth(lastSide, 0, kickT - BACK_PHASE - kickPhase - THROUGH_PHASE, END_PHASE);
      shoulderRoll = interpolateSmooth(shoulderRollAmp, 0, kickT - BACK_PHASE - kickPhase - THROUGH_PHASE, END_PHASE);
      kneePitch = interpolateSmooth(lastKneePitch, 0, kickT - BACK_PHASE - kickPhase - THROUGH_PHASE, END_PHASE);
      anklePitch = -kneePitch;
   // Shift weight back to both feet.
   } else if (kickT < (totalPhase + SHIFT_END_PERIOD / 4)) {
      forwardDist = 0;
      kneePitch = 0;
      anklePitch = 0;
      double endT = totalPhase - kickT;
      rock = DEG2RAD(kickLean) * sin((endT + SHIFT_END_PERIOD / 4) / SHIFT_END_PERIOD * 2 * M_PI);
      footh = kickStepH * sin((endT + SHIFT_END_PERIOD / 4) / SHIFT_END_PERIOD * 2 * M_PI);
   } else {
      kickT = 0;
      rock = 0;
      footh = 0;
      walk2014Option = WALK;
      walkState = NOT_WALKING;
      request->body.actionType = ActionCommand::Body::WALK;
      lastKickTime = 0;
   }
}

void Walk2014Generator::addKickJoints(JointValues &j){
   j.angles[RSJoints::LKneePitch] += kneePitchL;
   j.angles[RSJoints::LAnklePitch] += anklePitchL + shoulderRollL;
   j.angles[RSJoints::LShoulderPitch] -= kneePitchL;

   j.angles[RSJoints::RKneePitch] += kneePitchR;
   j.angles[RSJoints::RAnklePitch] += anklePitchR + shoulderRollR;
   j.angles[RSJoints::RShoulderPitch] -= kneePitchR;
}

float Walk2014Generator::leftAngle()
{
   float left_at_t = left*parabolicStep(t,nextFootSwitchT,0.0);
   float height = hiph-ankle;
   return atan(left_at_t/height);
}

float Walk2014Generator::linearStep(float time, float period) {
   if(time<=0) return 0;
   if(time>=period) return 1;
   return time/period;
}

float Walk2014Generator::parabolicReturn(float f){         //normalised [0,1] up and down
   double x = 0;
   double y = 0;
   if(f<0.25f) {
      y = 8*f*f;
   }
   if(f>=0.25f && f<0.5f) {
      x = 0.5f-f;
      y = 8*x*x;
      y = 1.0f-y;
   }
   if(f>=0.5f && f<0.75f){
      x = f-0.5f;
      y = 8*x*x;
      y = 1.0f-y;
   }
   if(f>=0.75f && f<=1.0f){
      x = 1.0f-f;
      y = 8*x*x;
   }
   return y;
}

float Walk2014Generator::parabolicStep(float time, float period, float deadTimeFraction){ //normalised [0,1] step up
   float deadTime = period*deadTimeFraction/2;
   if(time<deadTime+dt/2) return 0;
   if(time>period-deadTime-dt/2) return 1;
   float timeFraction = (time-deadTime)/(period-2*deadTime);
   if(time<period/2) return 2.0*timeFraction*timeFraction;
   return 4*timeFraction-2*timeFraction*timeFraction-1;
}

void Walk2014Generator::updateOdometry(Odometry *odometry, bool isLeftSwingFoot){
   // Work out incremental forward, left, and turn values for next time step
   float height = hiph-ankle;
   float turnOdo = -(turnRL - prevTurn);
   float leftOdo = (height*tan(leftR) - prevLeftR);
   float forwardOdo = (forwardR - prevForwardR);
   if(!isLeftSwingFoot){
      turnOdo *= -1;
      leftOdo = (height*tan(leftL) - prevLeftL);
      forwardOdo = (forwardL - prevForwardL);
   }
   forwardOdo *= MM_PER_M;
   leftOdo    *= MM_PER_M;
   //Calibrate odometry to match the actual speed
   forwardOdo *= 1;
   leftOdo    *= 1.23;
   turnOdo    *= -.53; // - .53
   //cout << forwardOdo <<" "<< leftOdo <<" "<< turnOdo << endl;
   *odometry = *odometry + Odometry(forwardOdo, leftOdo, turnOdo);

   // backup odometry values
   prevTurn = turnRL;
   prevLeftL = height*tan(leftL);
   prevLeftR = height*tan(leftR);
   prevForwardL = forwardL;
   prevForwardR = forwardR;
}

bool Walk2014Generator::isActive() {
   return !stopped;
}

bool Walk2014Generator::isStanding() {
  //return active.actionType == ActionCommand::Body::STAND || active.actionType == ActionCommand::Body::STAND_STRAIGHT; // || walk2014Option == STAND || (walk2014Option == STANDUP && hiph - 0.22 >= 0.01);
  return walkState == NOT_WALKING || bend == 0;
  //return (prevForwardL == prevLeftL) && (prevForwardL == prevTurn) && (prevTurn == 0);
}


void Walk2014Generator::readOptions(std::string path) {} //boost::program_options::variables_map &config) {}
void Walk2014Generator:: reset() {
   initialise();
   //llog(INFO) << "Walk2014 reset" << endl;
}

void Walk2014Generator:: stop() {
   stopping = true;
   //llog(INFO) << "Walk2014 stop" << endl;
}

float Walk2014Generator::interpolateSmooth(float start, float end, float tCurrent, float tEnd) {
   return start + (end - start) * (1 + cos(M_PI * tCurrent / tEnd - M_PI)) / 2;
}

XYZ_Coord Walk2014Generator::mf2b(float Hyp, float Hp, float Hr,
                                  float Kp,  float Ap, float Ar,
                                  float xf, float yf, float zf) {
// MFOOT2BODY Transform coords from foot to body.
// This code originates from 2010 using symbolic equations in Matlab to perform the coordinate transforms - see team report (BH)
// In future this approach to IK for the Nao should be reworked in closed form, significantly reducing the size of the code the
// the computational complexity (BH)
   XYZ_Coord result;
   float pi = M_PI;
   float tibia        = this->tibia*1000;
   float thigh        = this->thigh*1000;
   float k  = sqrt(2.0);
   float c1 = cos(Ap);
   float c2 = cos(Hr + pi/4.0);
   float c3 = cos(Hyp - pi/2.0);
   float c4 = cos(Hp);
   float c5 = cos(Kp);
   float c6 = cos(Ar - pi/2.0);
   float s1 = sin(Kp);
   float s2 = sin(Hp);
   float s3 = sin(Hyp - 1.0/2.0*pi);
   float s4 = sin(Hr + 1.0/4.0*pi);
   float s5 = sin(Ap);
   float s6 = sin(Ar - 1.0/2.0*pi);
   result.x = thigh*(s2*s3 - c2*c3*c4) + tibia*(s1*(c4*s3 + c2*c3*s2) +
       c5*(s2*s3 - c2*c3*c4)) - yf*(c6*(c1*(s1*(c4*s3 + c2*c3*s2) +
       c5*(s2*s3 - c2*c3*c4)) - s5*(s1*(s2*s3 - c2*c3*c4) - c5*(c4*s3 +
       c2*c3*s2))) + c3*s4*s6) + zf*(s6*(c1*(s1*(c4*s3 + c2*c3*s2) +
       c5*(s2*s3 - c2*c3*c4)) - s5*(s1*(s2*s3 - c2*c3*c4) - c5*(c4*s3 +
       c2*c3*s2))) - c3*c6*s4) + xf*(c1*(s1*(s2*s3 - c2*c3*c4) -
       c5*(c4*s3 + c2*c3*s2)) + s5*(s1*(c4*s3 + c2*c3*s2) +
       c5*(s2*s3 - c2*c3*c4)));
   result.y = xf*(c1*(c5*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) -
       (c3*c4*k)/2.0f) + s1*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) +
       (c3*k*s2)/2.0f)) + s5*(c5*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) +
       (c3*k*s2)/2.0f) - s1*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) -
       (c3*c4*k)/2.0f))) + tibia*(c5*(c4*((k*s4)/2.0f +
       (c2*k*s3)/2.0f) + (c3*k*s2)/2.0f) - s1*(s2*((k*s4)/2.0f +
       (c2*k*s3)/2.0f) - (c3*c4*k)/2.0f)) + thigh*(c4*((k*s4)/2.0f +
       (c2*k*s3)/2.0f) + (c3*k*s2)/2.0f) - yf*(s6*((c2*k)/2.0f -
       (k*s3*s4)/2.0f) + c6*(c1*(c5*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) +
       (c3*k*s2)/2.0f) - s1*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) -
       (c3*c4*k)/2.0f)) - s5*(c5*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) -
       (c3*c4*k)/2.0f) + s1*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) +
       (c3*k*s2)/2.0f)))) - zf*(c6*((c2*k)/2.0f - (k*s3*s4)/2.0f) -
       s6*(c1*(c5*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) + (c3*k*s2)/2.0f) -
       s1*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) - (c3*c4*k)/2.0f)) -
       s5*(c5*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) - (c3*c4*k)/2.0f) +
       s1*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) + (c3*k*s2)/2.0f))));
   result.z = yf*(s6*((c2*k)/2.0f + (k*s3*s4)/2.0f) +
       c6*(c1*(c5*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f) -
       s1*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f)) -
       s5*(c5*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f) +
       s1*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f)))) -
       tibia*(c5*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f) -
       s1*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f)) -
       thigh*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f) -
       xf*(c1*(c5*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f) +
       s1*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f)) +
       s5*(c5*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f) -
       s1*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f))) +
       zf*(c6*((c2*k)/2.0f + (k*s3*s4)/2.0f) - s6*(c1*(c5*(c4*((k*s4)/2.0f -
       (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f) - s1*(s2*((k*s4)/2.0f -
       (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f)) - s5*(c5*(s2*((k*s4)/2.0f -
       (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f) + s1*(c4*((k*s4)/2.0f -
       (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f))));
   return result;
}

Walk2014Generator::Hpr Walk2014Generator::hipAngles(float Hyp, float Hp,
                                  float Hr, float Kp,  float Ap, float Ar,
                                  float xf, float yf, float zf, XYZ_Coord e) {
// Code from 2010 to perform interative Inverse Kinematics.
// Symbolic equations generated in Matlab - see 2010 team report for details and reference
   Hpr result;
   float pi = M_PI;
   float tibia        = this->tibia*1000;
   float thigh        = this->thigh*1000;
   float k  = sqrt(2.0);
   float c1 = cos(Ap);
   float c2 = cos(Hr + pi/4.0);
   float c3 = cos(Hyp - pi/2.0);
   float c4 = cos(Hp);
   float c5 = cos(Kp);
   float c6 = cos(Ar - pi/2.0);
   float s1 = sin(Kp);
   float s2 = sin(Hp);
   float s3 = sin(Hyp - 1.0/2.0*pi);
   float s4 = sin(Hr + 1.0/4.0*pi);
   float s5 = sin(Ap);
   float s6 = sin(Ar - 1.0/2.0*pi);
   float j11 = thigh*(c4*s3 + c2*c3*s2) - tibia*(s1*(s2*s3 -
       c2*c3*c4) - c5*(c4*s3 + c2*c3*s2)) + xf*(c1*(s1*(c4*s3 +
       c2*c3*s2) + c5*(s2*s3 - c2*c3*c4)) - s5*(s1*(s2*s3 -
       c2*c3*c4) - c5*(c4*s3 + c2*c3*s2))) + c6*yf*(c1*(s1*(s2*s3 -
       c2*c3*c4) - c5*(c4*s3 + c2*c3*s2)) + s5*(s1*(c4*s3 +
       c2*c3*s2) + c5*(s2*s3 - c2*c3*c4))) - s6*zf*(c1*(s1*(s2*s3 -
       c2*c3*c4) - c5*(c4*s3 + c2*c3*s2)) + s5*(s1*(c4*s3 +
       c2*c3*s2) + c5*(s2*s3 - c2*c3*c4)));
   float j12 = yf*(c6*(c1*(c3*s1*s2*s4 - c3*c4*c5*s4) +
       s5*(c3*c4*s1*s4 + c3*c5*s2*s4)) - c2*c3*s6) -
       tibia*(c3*s1*s2*s4 - c3*c4*c5*s4) - zf*(s6*(c1*(c3*s1*s2*s4 -
       c3*c4*c5*s4) + s5*(c3*c4*s1*s4 + c3*c5*s2*s4)) + c2*c3*c6) +
       xf*(c1*(c3*c4*s1*s4 + c3*c5*s2*s4) - s5*(c3*s1*s2*s4 -
       c3*c4*c5*s4)) + c3*c4*s4*thigh;
   float j21 = xf*(c1*(c5*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) +
       (c3*k*s2)/2.0f) - s1*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) -
       (c3*c4*k)/2.0f)) -
       s5*(c5*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) - (c3*c4*k)/2.0f) +
       s1*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) + (c3*k*s2)/2.0f))) -
       tibia*(c5*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) - (c3*c4*k)/2.0f) +
       s1*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) + (c3*k*s2)/2.0f)) -
       thigh*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) - (c3*c4*k)/2.0f) +
       c6*yf*(c1*(c5*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) - (c3*c4*k)/2.0f) +
       s1*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) + (c3*k*s2)/2.0f)) +
       s5*(c5*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) + (c3*k*s2)/2.0f) -
       s1*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) - (c3*c4*k)/2.0f))) -
       s6*zf*(c1*(c5*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) - (c3*c4*k)/2.0f) +
       s1*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) + (c3*k*s2)/2.0f)) +
       s5*(c5*(c4*((k*s4)/2.0f + (c2*k*s3)/2.0f) + (c3*k*s2)/2.0f) -
       s1*(s2*((k*s4)/2.0f + (c2*k*s3)/2.0f) - (c3*c4*k)/2.0f)));
   float j22 = tibia*(c4*c5*((c2*k)/2.0f - (k*s3*s4)/2.0f) -
       s1*s2*((c2*k)/2.0f - (k*s3*s4)/2.0f)) + xf*(c1*(c4*s1*((c2*k)/2.0f -
       (k*s3*s4)/2.0f) + c5*s2*((c2*k)/2.0f - (k*s3*s4)/2.0f)) +
       s5*(c4*c5*((c2*k)/2.0f - (k*s3*s4)/2.0f) - s1*s2*((c2*k)/2.0f -
       (k*s3*s4)/2.0f))) + yf*(s6*((k*s4)/2.0f + (c2*k*s3)/2.0f) -
       c6*(c1*(c4*c5*((c2*k)/2.0f - (k*s3*s4)/2.0f) - s1*s2*((c2*k)/2.0f -
       (k*s3*s4)/2.0f)) - s5*(c4*s1*((c2*k)/2.0f - (k*s3*s4)/2.0f) +
       c5*s2*((c2*k)/2.0f - (k*s3*s4)/2.0f)))) + zf*(c6*((k*s4)/2.0f +
       (c2*k*s3)/2.0f) + s6*(c1*(c4*c5*((c2*k)/2.0f - (k*s3*s4)/2.0f) -
       s1*s2*((c2*k)/2.0f - (k*s3*s4)/2.0f)) - s5*(c4*s1*((c2*k)/2.0f -
       (k*s3*s4)/2.0f) + c5*s2*((c2*k)/2.0f - (k*s3*s4)/2.0f)))) +
       c4*thigh*((c2*k)/2.0f - (k*s3*s4)/2.0f);
   float j31 = tibia*(c5*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) +
       (c3*c4*k)/2.0f) + s1*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) -
       (c3*k*s2)/2.0f)) -
       xf*(c1*(c5*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f) -
       s1*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f)) -
       s5*(c5*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f) +
       s1*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f))) +
       thigh*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f) -
       c6*yf*(c1*(c5*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f) +
       s1*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f)) +
       s5*(c5*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f) -
       s1*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f))) +
       s6*zf*(c1*(c5*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f) +
       s1*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f)) +
       s5*(c5*(c4*((k*s4)/2.0f - (c2*k*s3)/2.0f) - (c3*k*s2)/2.0f) -
       s1*(s2*((k*s4)/2.0f - (c2*k*s3)/2.0f) + (c3*c4*k)/2.0f)));
   float j32 = -tibia*(c4*c5*((c2*k)/2.0f + (k*s3*s4)/2.0f) -
       s1*s2*((c2*k)/2.0f + (k*s3*s4)/2.0f)) - xf*(c1*(c4*s1*((c2*k)/2.0f +
       (k*s3*s4)/2.0f) + c5*s2*((c2*k)/2.0f + (k*s3*s4)/2.0f)) +
       s5*(c4*c5*((c2*k)/2.0f + (k*s3*s4)/2.0f) - s1*s2*((c2*k)/2.0f +
       (k*s3*s4)/2.0f))) - yf*(s6*((k*s4)/2.0f - (c2*k*s3)/2.0f) -
       c6*(c1*(c4*c5*((c2*k)/2.0f + (k*s3*s4)/2.0f) - s1*s2*((c2*k)/2.0f +
       (k*s3*s4)/2.0f)) - s5*(c4*s1*((c2*k)/2.0f + (k*s3*s4)/2.0f) +
       c5*s2*((c2*k)/2.0f + (k*s3*s4)/2.0f)))) - zf*(c6*((k*s4)/2.0f -
       (c2*k*s3)/2.0f) + s6*(c1*(c4*c5*((c2*k)/2.0f + (k*s3*s4)/2.0f) -
       s1*s2*((c2*k)/2.0f + (k*s3*s4)/2.0f)) - s5*(c4*s1*((c2*k)/2.0f +
       (k*s3*s4)/2.0f) + c5*s2*((c2*k)/2.0f + (k*s3*s4)/2.0f)))) -
       c4*thigh*((c2*k)/2.0f + (k*s3*s4)/2.0f);
   float xbe = e.x;
   float ybe = e.y;
   float zbe = e.z;
   float lambda = 0.4f;
   float la2 = lambda*lambda;
   float la4 = la2*la2;
   float j322 = j32*j32;
   float j222 = j22*j22;
   float j122 = j12*j12;
   float j212 = j21*j21;
   float j112 = j11*j11;
   float j312 = j31*j31;
   float sigma = 1.0f/(la4 + j112*j222 + j122*j212 + j112*j322 + j122*j312 +
   j212*j322 + j222*j312 + j112*la2 + j122*la2 + j212*la2 + j222*la2 +
   j312*la2 + j322*la2 - 2.0f*j11*j12*j21*j22 - 2.0f*j11*j12*j31*j32 -
   2.0f*j21*j22*j31*j32);
   result.Hp = sigma*xbe*(j11*j222 + j11*j322 + j11*la2 - j12*j21*j22 -
   j12*j31*j32) + sigma*ybe*(j122*j21 + j21*j322 + j21*la2 - j11*j12*j22 -
   j22*j31*j32) + sigma*zbe*(j122*j31 + j222*j31 + j31*la2 - j11*j12*j32 -
   j21*j22*j32);
   result.Hr =  sigma*xbe*(j12*j212 + j12*j312 + j12*la2 - j11*j21*j22 -
   j11*j31*j32) + sigma*ybe*(j112*j22 + j22*j312 + j22*la2 - j11*j12*j21 -
   j21*j31*j32) + sigma*zbe*(j112*j32 + j212*j32 + j32*la2 - j11*j12*j31 -
   j21*j22*j31);
   return result;
}

