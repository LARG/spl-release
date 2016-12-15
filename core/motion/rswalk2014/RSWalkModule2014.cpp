#include "RSWalkModule2014.h"
#include "WalkEnginePreProcessor.hpp"
#include "Walk2014Generator.hpp"
#include "Walk2015Generator.hpp"
#include "DistributedGenerator.hpp"
#include "ClippedGenerator.hpp"

#include <math/Geometry.h>

// Runswift files
#include "types/JointValues.hpp"
#include "types/AbsCoord.hpp"
#include "perception/kinematics/Kinematics.hpp"
#include "BodyModel.hpp"

#include <memory/BodyModelBlock.h>
#include <memory/WorldObjectBlock.h>//for rswalk2014
#include <memory/FrameInfoBlock.h>
#include <memory/JointBlock.h>
#include <memory/JointCommandBlock.h>
#include <memory/KickRequestBlock.h>
#include <memory/OdometryBlock.h>
#include <memory/RobotInfoBlock.h>
#include <memory/SensorBlock.h>
#include <memory/WalkInfoBlock.h>
#include <memory/WalkParamBlock.h>
#include <memory/WalkRequestBlock.h>
#include <memory/SpeechBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/RobotStateBlock.h>

#include <common/Kicks.h>

#include "GyroConfig.h"

#define ODOMETRY_LAG 8

#define DEBUG_OUTPUT false
#define GSL_COLLECT_DATA false

enum FootSensorRegion
{
    left_front,
    left_back,
    left_right,
    left_left,
    right_front,
    right_back,
    right_right,
    right_left,
    none
};


/*-----------------------------------------------------------------------------
* Motion thread tick function (copy from runswift MotionAdapter.cpp)
This would be like processFrame() function as in our code
*---------------------------------------------------------------------------*/
void RSWalkModule2014::processFrame() {


	// If we are changing commands we need to reset generators
	if(walk_request_->motion_ != prev_command && !walk_request_->perform_kick_){
	    if(DEBUG_OUTPUT) cout << "RESETTING GENERATORS! " << WalkRequestBlock::getName(walk_request_->motion_) << " != " << WalkRequestBlock::getName(prev_command) << "\n";
		standing = true;
	}

        static WalkControl twalk = WALK_CONTROL_OFF;
	if (twalk == WALK_CONTROL_OFF && walk_request_->walk_control_status_ == WALK_CONTROL_SET) {
		cout << "walk received control, ";
		cout << "Foot: " << (walk_request_->kick_with_left_ ? "LEFT" : "RIGHT") << endl;
	} else if (twalk == WALK_CONTROL_SET and not (walk_request_->walk_control_status_ == WALK_CONTROL_SET)) {
		cout << "Completed walk control last frame" << endl;
	        clipper->resetLinedUp();
	}
	twalk = walk_request_->walk_control_status_;	

	// Setting walk kick parameters in bodyModel
        bodyModel.walkKick = walk_request_->perform_kick_;
        bodyModel.walkKickLeftLeg = walk_request_->kick_with_left_;
        bodyModel.walkKickHeading = walk_request_->kick_heading_;

	// 1. Need odometry to send to WalkGenerator. makeJoints updates odometry for localization
	Odometry odo = Odometry(odometry_->displacement.translation.x, odometry_->displacement.translation.y, odometry_->displacement.rotation);
	Odometry prev = Odometry(odo); // Another copy for after makeJoints call

        // 1.1 Get ball position relative to robot - this will be changed to a target position for line-up
        // This is for lining up to the ball
        WorldObject* ball = &(world_objects_->objects_[WO_BALL]); //ball->loc.x (after localization)
        double ballX = ball->relPos.x;
        double ballY = ball->relPos.y;
        if(DEBUG_OUTPUT) cout << "BallX: "<<ballX<<" BallY: "<<ballY<<endl;


	// 2. Convert our request to runswift request
	// 2.1 Head
	ActionCommand::Head head; // Use default head. Not using HeadGenerator

	// 2.2 Body
	ActionCommand::Body body;

	if (walk_request_->motion_ == WalkRequestBlock::STAND)
	{
		if(DEBUG_OUTPUT) cout << "Requested: STANDUP\n";
		body.actionType = ActionCommand::Body::STAND; 
		body.forward = 0;
		body.left = 0;
		body.turn = 0;
		body.isFast = false; 
		body.bend = 1;
		body.power = 1;
	}
	else if (walk_request_->motion_ == WalkRequestBlock::STAND_STRAIGHT)
	{
		if (DEBUG_OUTPUT) cout << "Requested: STANDSTRAIGHT\n";
		body.actionType = ActionCommand::Body::STAND_STRAIGHT;
		body.forward = 0;
		body.left = 0;
		body.turn = 0;
		body.isFast = false;
		body.bend = 0;
		body.power = 0;
	} else if (walk_request_->motion_ == WalkRequestBlock::WALK) {	

		if(DEBUG_OUTPUT) cout << "Requested: WALK\n";
		body.actionType = ActionCommand::Body::WALK;
		body.forward = walk_request_->speed_.translation.x * walk_params_->rs_params_.speedMax.translation.x;
		body.left = walk_request_->speed_.translation.y * walk_params_->rs_params_.speedMax.translation.y;
		body.turn = walk_request_->speed_.rotation * walk_params_->rs_params_.speedMax.rotation;
		body.bend = 1;
		body.power = 1;
		body.speed = walk_params_->rs_params_.speed;
		body.isFast = false;

		if (bodyModel.walkKick and walk_request_->walk_control_status_ == WALK_CONTROL_SET) {//target_walk_active_) {
			body.actionType = ActionCommand::Body::KICK;
                        body.speed = 1.0;//7.0 * DEG_T_RAD;
			ballX = walk_request_->target_point_.translation.x;
			ballY = walk_request_->target_point_.translation.y;
			if (bodyModel.walkKickLeftLeg) body.foot = ActionCommand::Body::LEFT;
			else body.foot = ActionCommand::Body::RIGHT;
		} else {
			bodyModel.walkKick = false;
		} 

	} else if (walk_request_->motion_ == WalkRequestBlock::LINE_UP) {
		body.actionType = ActionCommand::Body::LINE_UP;
                body.forward = walk_request_->speed_.translation.x * walk_params_->rs_params_.speedMax.translation.x;
                body.left = walk_request_->speed_.translation.y * walk_params_->rs_params_.speedMax.translation.y;
                body.turn = walk_request_->speed_.rotation * walk_params_->rs_params_.speedMax.rotation;
                body.bend = 1;
		body.power = 1;
                body.speed = 1.0;//5.0 * DEG_T_RAD;
                body.isFast = false;
		ballX = walk_request_->target_point_.translation.x;
		ballY = walk_request_->target_point_.translation.y;
		body.turn = walk_request_->target_point_.rotation;

		if (walk_request_->kick_with_left_) body.foot = ActionCommand::Body::LEFT;
		else body.foot = ActionCommand::Body::RIGHT;
		//cout << "Line up foot: " << (body.foot == ActionCommand::Body::LEFT ? "LEFT" : "RIGHT") << endl;
                if (walk_request_->walk_control_status_ != WALK_CONTROL_SET) {
                  if(DEBUG_OUTPUT) cout << "Still lining up but lineup done... " << walk_request_->walk_control_status_ << " " << ballX << " " << ballY << endl;
                  body.actionType = ActionCommand::Body::STAND;
		  body.bend = 1;
		  body.forward = 0;
		  body.left = 0;
                  body.turn =0;
		}
		if (clipper->isLinedUp() && walk_request_->walk_control_status_ == WALK_CONTROL_SET) { //target_walk_is_active_) {
			cout << "Line up complete" << endl;
			body.actionType = ActionCommand::Body::STAND;
			body.bend = 1;
		        body.forward = 1;
			body.left = 0;
        	        body.turn =0;
                	//walk_request_->target_walk_is_active_ = false;
			kick_request_->finished_with_step_ = true;
			walk_request_->walk_control_status_ = WALK_CONTROL_DONE;
		} 
	} else if (walk_request_->motion_ == WalkRequestBlock::NONE || walk_request_->motion_ == WalkRequestBlock::WAIT)
	{
		if(DEBUG_OUTPUT) cout << "Requested: " << WalkRequestBlock::getName(walk_request_->motion_) << "\n";
		body.actionType = ActionCommand::Body::ActionType::NONE;
		body.forward = 0;
		body.bend = 1;
		body.left = 0;
		body.turn = 0;
		body.isFast = false;
	}
	else{
		if(DEBUG_OUTPUT) cout << "Other Request: " << walk_request_->motion_ << endl; 
		body.actionType = ActionCommand::Body::ActionType::STAND;
		body.forward = 0;
		body.left = 0;
		body.turn = 0;
		body.isFast = false;			
	}


        if(!body_model_->feet_on_ground_ && body.actionType != ActionCommand::Body::ActionType::NONE)
        {
                // Need something here so robot stops when picked up
		body.actionType = ActionCommand::Body::ActionType::REF_PICKUP;
        }

	// 2.3 LEDs and Sonar
	ActionCommand::LED leds; // Not important, use defaults - Josiah
        float sonar = 0.0;

	// Now we have everything we need for a runswift request
	ActionCommand::All request(head,body,leds,sonar);

	// 3. Create runswift sensor object
	SensorValues sensors;

	// 3.1 Sense Joints 
	for (int ut_ind=0; ut_ind<NUM_JOINTS; ut_ind++){
		if (ut_ind != RHipYawPitch){ // RS does not have this joint because RHYP is same as LHYP on Nao
			int rs_ind = utJointToRSJoint[ut_ind];
			sensors.joints.angles[rs_ind] = joints_->values_[ut_ind] * robot_joint_signs[ut_ind]; // changed from raw_joints - Josiah
			sensors.joints.temperatures[rs_ind] = sensors_->joint_temperatures_[ut_ind];
		}
	}

	// Detect if legs are getting hot
	if(sensors_->joint_temperatures_[RKneePitch] > 100 || sensors_->joint_temperatures_[RHipPitch] > 100)
		bodyModel.isRightLegHot = true;
	if(sensors_->joint_temperatures_[LKneePitch] > 100 || sensors_->joint_temperatures_[LHipPitch] > 100)
		bodyModel.isLeftLegHot = true;

	// 3.2 Sensors
	for (int ut_ind=0; ut_ind<bumperRR + 1; ut_ind++){
		int rs_ind = utSensorToRSSensor[ut_ind];
		sensors.sensors[rs_ind] = sensors_->values_[ut_ind];
	}

	
	// 4. If robot is remaining still, calbrate x and y gyroscopes
	// 4.1 Calibrate gyroX:
	double cur_gyroX = sensors.sensors[RSSensors::InertialSensor_GyrX];
//        cout << cur_gyroX << " " << sensors.sensors[RSSensors::InertialSensor_GyrY] << " " << sensors.sensors[RSSensors::InertialSensor_AccX] << " " << sensors.sensors[RSSensors::InertialSensor_AccY] << endl;
	double delta_gyroX = abs(cur_gyroX - last_gyroX);
	// maintain moving averages for gyro and delta_gyro
	avg_gyroX = avg_gyroX * (1.0 - window_size) + cur_gyroX * window_size;
	avg_delta_gyroX = avg_delta_gyroX * (1.0- window_size) + delta_gyroX * window_size;
        //cout << "gyroX: " << cur_gyroX << endl;
 	if (avg_delta_gyroX < delta_threshold) {
		// robot remains still, do calibration
		offsetX = avg_gyroX;
		// reset avg_delta so it does not keep recalibrating
		avg_delta_gyroX = reset;
		calX_count += 1; 
		if (calX_count == 1) {
                  cout << "(First calibration, may not be accurate) A GyroX calibration was triggered, offsetX set to: " << offsetX << endl;
		}
		else {
		  cout << "A GyroX calibration was triggered, offsetX set to: " << offsetX << endl;
		}
		last_gyroX_time = frame_info_-> seconds_since_start;
	}
	else {
		if (DEBUG_OUTPUT) cout << "avg_delta_gyroX is: " << avg_delta_gyroX <<  " GyroX not stable, no calibration" << endl;
	}

      	last_gyroX = cur_gyroX;

	// 4.2 Calibrate gyroY:
	double cur_gyroY = sensors.sensors[RSSensors::InertialSensor_GyrY];
	double delta_gyroY = abs(cur_gyroY - last_gyroY);
	// maintain moving averages for gyro and delta_gyro
	avg_gyroY = avg_gyroY * (1.0 - window_size) + cur_gyroY * window_size;
	avg_delta_gyroY = avg_delta_gyroY * (1.0- window_size) + delta_gyroY * window_size;
 	if (avg_delta_gyroY < delta_threshold) {
		// robot remains still, do calibration
		offsetY = avg_gyroY;
		// reset avg_delta so it does not keep recalibrating
		avg_delta_gyroY = reset;
		calY_count += 1;
		if (calY_count == 1) {
                  cout << "(First calibration, may not be accurate) A GyroY calibration was triggered, offsetY set to: " << offsetY << endl;
		}
		else {     
	 	  cout << "A GyroY calibration was triggered, offsetY set to: " << offsetY << endl;
		}
		last_gyroY_time = frame_info_-> seconds_since_start;
	}
	else {
		if (DEBUG_OUTPUT) cout << "avg_delta_gyroY is: " << avg_delta_gyroY <<  " GyroY not stable, no calibration" << endl;
	}

	last_gyroY = cur_gyroY;
	

	// After each gyro has completed two cycles it is calibrated.
        //TODO: not sure whether need to put calZ_count here, since calibrating gyroZ does not affect walk;
	if (calX_count >= 2)
	  bodyModel.isGyroXCalibrated = true;
	if (calY_count >= 2)
	  bodyModel.isGyroYCalibrated = true;
//	if (calZ_count >= 2)
//	  bodyModel.isGyroZCalibrated = true;

	if ((calX_count >= 2 && calY_count >= 2) || (getSystemTime() - calibration_write_time < 600))
	{
	  odometry_->walkDisabled = false;
	  calX_count = 2;
	  calY_count = 2;
	  
	  if (last_gyroY_time  > last_calibration_write + 30 && last_gyroX_time > last_calibration_write + 30)
	  {
	    calibration_write_time = getSystemTime();
	    GyroConfig config;
	    config.offsetX = offsetX;
	    config.offsetY = offsetY;
	    config.calibration_write_time = calibration_write_time;
      auto path = util::format("%s/%i_xy.yaml", util::cfgpath(util::GyroConfigs), robot_state_->robot_id_);
	    config.save(path);
	    last_calibration_write = frame_info_->seconds_since_start;
	  }
	} else if ((calX_count < 2 || calY_count < 2 || calZ_count < 2)) {
		odometry_->walkDisabled = true;
	}

	// Apply offset and convert from rad/sec to rad /frame
	sensors.sensors[RSSensors::InertialSensor_GyrX] = (sensors.sensors[RSSensors::InertialSensor_GyrX] - offsetX) * 0.01;
	sensors.sensors[RSSensors::InertialSensor_GyrY] = (sensors.sensors[RSSensors::InertialSensor_GyrY] - offsetY) * 0.01;
        // V5s need to calibrate Z as well
        // cout << "gyroZ: " << sensors.sensors[RSSensors::InertialSensor_GyrRef] << endl;        
	// sensors.sensors[RSSensors::InertialSensor_GyrRef] = (sensors.sensors[RSSensors::InertialSensor_GyrRef] - offsetZ) * 0.01;
	
        // 5. Prepare BodyModel to pass to makeJoints

	// 5.1 Get Kinematics ready for bodyModel. We should figure out what parameter values should be.
	kinematics.setSensorValues(sensors); 

	// TODO apply camera calibrations to kinematics. For now we use default of 0
//	kinematics.parameters.cameraPitchTop=2.4;
//	kinematics.parameters.cameraYawTop=sensors_->values_[HeadYaw];
//	kinematics.parameters.cameraRollTop=0;
//	kinematics.parameters.cameraYawBottom=sensors_->values_[HeadPitch];
//	kinematics.parameters.cameraPitchBottom=sensors_->values_[HeadYaw];
//	kinematics.parameters.cameraRollBottom=0;
//	kinematics.parameters.bodyPitch=3.3;//sensors_->values_[angleY];

	kinematics.updateDHChain();

	// Resetting generators and moving to intermediate stance
        if (standing){

	  if (prev_command == WalkRequestBlock::LINE_UP || walk_request_->motion_ == WalkRequestBlock::LINE_UP) {
	    clipper->resetLinedUp();
	    standing = false;
	    if (walk_request_->motion_ == WalkRequestBlock::STAND || walk_request_->motion_ == WalkRequestBlock::STAND_STRAIGHT || walk_request_->motion_ == WalkRequestBlock::NONE)
		standing = true;
	  } 
          if(standing) {
            clipper->reset();
//          clipper->resetLinedUp();
	    if(DEBUG_OUTPUT) cout << "Resetting clipper" << endl;
            request.body = ActionCommand::Body::INITIAL;
            odo.clear();
	  }
        }


	// 5.2 update bodyModel
	bodyModel.kinematics = &kinematics;
	bodyModel.update(&odo, sensors);
	
	
        if (request.body.actionType == ActionCommand::Body::WALK && odometry_->walkDisabled){
		static int delay_ct = 0;
		if (delay_ct % 100 == 0)//prev_command != walk_request_->motion_)
		  speech_->say("Not Calibrated");
	        delay_ct ++;
		if(DEBUG_OUTPUT) cout << "Not calibrated" << endl;
		if (odometry_->standing)
			request.body = ActionCommand::Body::STAND;
		else
			request.body = ActionCommand::Body::NONE;
	}

	  if(DEBUG_OUTPUT) printf("motion request: %s, prev: %s, body request: %i\n", WalkRequestBlock::getName(walk_request_->motion_), WalkRequestBlock::getName(prev_command), static_cast<int>(request.body.actionType));
        prev_command = walk_request_->motion_;

	bool requestWalkKick = bodyModel.walkKick;
	// Call the clipped generator which calls the distributed generator to produce walks, stands, etc.
	JointValues joints = clipper->makeJoints(&request, &odo, sensors, bodyModel, ballX, ballY);
        if (requestWalkKick && requestWalkKick != bodyModel.walkKick && walk_request_->walk_control_status_ == WALK_CONTROL_SET) {//target_walk_is_active_) {
		cout << "Done with walk kick" << endl;
		walk_kick_finish_frame_ = frame_info_->frame_id; 
	        clipper->resetLinedUp();
//                walk_request_->target_walk_is_active_ = false;
		walk_request_->walk_control_status_ = WALK_CONTROL_DONE;
		target_walk_active_ = false;
        } //else if(not target_walk_active_) { walk_request_->target_walk_is_active_ = false;}
        	

//	static int point_id_ = 0;
	if (GSL_COLLECT_DATA && request.body.actionType == ActionCommand::Body::WALK)
        	writeDataToFile(sensors,joints);
//	point_id_ ++;

	// We aren't setting arms any more so this isn't important unless we go back to it
	// Setting arms behind back makes us get caught less but then we can't counter balance rswalk
	if ((frame_info_->seconds_since_start - last_walk_or_stand_) > 0.3) {
		arm_state_ = -1;
	}

        // Update odometry
	static double cum_f=0, cum_l=0, cum_t=0;
        odometry_->displacement.translation.x = odo.forward;
        odometry_->displacement.translation.y = odo.left;
        odometry_->displacement.rotation = odo.turn;
	//if (walk_request_->speed_.translation.x > 0.5)
		//odometry_->displacement.rotation -= (walk_params_->bh_params_.rs_turn_angle_offset / 100.0);
        odometry_->standing = clipper->isStanding();
	Odometry delta = Odometry(odo - prev);
	cum_f += delta.forward;
	cum_l += delta.left;
	cum_t += delta.turn;
	// For debugging odometry
/*	if (body.turn != 0){
		cout << "Odometry Prev: " << prev.turn;
		cout << " Odometry Delta: " << delta.turn;
		cout << " Odometry: "  << odo.turn;
		cout << " Cumulative Odometry: " << cum_t << endl;
	}
*/

      // Update walk_info_
        //walk_info_->walk_is_active_ = !(clipper->isStanding());
        walk_info_->instability_ = avg_gyroX;
        if (avg_gyroX < -2 or avg_gyroX > 2) {
          walk_info_->instable_ = true;
        } else {
          walk_info_->instable_ = false;
        }

        // Update
        wasKicking = kick_request_->kick_running_;

	if (request.body.actionType == ActionCommand::Body::ActionType::NONE){
		return;
	}

	// For setting arms
	last_walk_or_stand_ = frame_info_->seconds_since_start;

	// Convert RS joints to UT joints and write to commands memory block
	for (int ut_ind = BODY_JOINT_OFFSET; ut_ind < NUM_JOINTS; ut_ind++) {
		if (ut_ind != RHipYawPitch){ // RS does not have this joint
     			int rs_ind = utJointToRSJoint[ut_ind];
        		commands_->angles_[ut_ind] = robot_joint_signs[ut_ind] * joints.angles[rs_ind];
    	    		commands_->stiffness_[ut_ind] = joints.stiffnesses[rs_ind];
		} 
   	}

	// Ruohan: setting head stiffness was commented out in bhuman. Should ask Jake about this	
  // Jake: setting stiffness here is unnecessary and means we can never turn these off for testing.
	//commands_->stiffness_[HeadPitch] = 1.0;
	//commands_->stiffness_[HeadYaw] = 1.0;
	commands_->stiffness_[RHipYawPitch] = commands_->stiffness_[LHipYawPitch]; // RHYP will be overwritten with LHYP anyway but for completeness
	commands_->angles_[RHipYawPitch] = commands_->angles_[LHipYawPitch];

	// Walk keeps falling backwards when knees over 100. Leaning forward helps some
	if (request.body.actionType == ActionCommand::Body::STAND && (bodyModel.isLeftLegHot || bodyModel.isRightLegHot)){
		commands_->angles_[RHipPitch] -= DEG_T_RAD * 2;
		commands_->angles_[LHipPitch] -= DEG_T_RAD * 2;
	}
	
	// if the robot has walked, listen to arm command from WalkGenerator, else do stiffness
	if (request.body.actionType != ActionCommand::Body::WALK) {
		for (int ind = ARM_JOINT_FIRST; ind <= ARM_JOINT_LAST; ind ++)
			commands_->stiffness_[ind] = 1.0; //0.75;
	}
	
	selectivelySendStiffness(); // Only send stiffness if at least one joint has changed stiffness values by at least 0.01
//	setArms(commands_->angles_,0.01); // Arms getting stuck on robot front with rswalk. Needs tuning

	commands_->send_body_angles_ = true; // So commands will execute
	commands_->send_stiffness_ = true;

	if (request.body.actionType == ActionCommand::Body::STAND){
		commands_->body_angle_time_ = 10; // Increase these if standing?
		commands_->stiffness_time_ = 10; 
	} else {
		commands_->body_angle_time_ = 10;
		commands_->stiffness_time_ = 10;

	}

	// Not doing anything with these now. Could be used in future for smoothing
	prevForward = body.forward;
	prevLeft = body.left;
	prevTurn = body.turn;

	// Agent Effector code?
	// This is in rswalk agent effecter. Not sure what standing is but it works now.
	static bool kill_standing = false;
	standing = kill_standing;
	if (kill_standing) {
		kill_standing = false;
		standing = false;
	}
	else{
		kill_standing = true;
	}

}

void RSWalkModule2014::readOptions(std::string path)
{

	clipper->readOptions(path);

}

RSWalkModule2014::RSWalkModule2014():
/*    slow_stand_start(-1),
    slow_stand_end(-1),
    walk_requested_start_time(-1),
    prev_kick_active_(false), */
    arms_close_to_targets_(false),
    arm_state_(-1),
    arm_state_change_(-1),
    last_walk_or_stand_(-1),
    step_into_kick_state_(NONE),
    time_step_into_kick_finished_(0), 
    walk_kick_finish_frame_(-100),
    last_gyroY_time(-1),
    last_gyroX_time(-1),
    prevForward(0),
    prevLeft(0),
    prevTurn(0)
{
    utJointToRSJoint[HeadYaw] = RSJoints::HeadYaw;
    utJointToRSJoint[HeadPitch] = RSJoints::HeadPitch;

    utJointToRSJoint[LShoulderPitch] = RSJoints::LShoulderPitch;
    utJointToRSJoint[LShoulderRoll] = RSJoints::LShoulderRoll;
    utJointToRSJoint[LElbowYaw] = RSJoints::LElbowYaw;
    utJointToRSJoint[LElbowRoll] = RSJoints::LElbowRoll;

    utJointToRSJoint[RShoulderPitch] = RSJoints::RShoulderPitch;
    utJointToRSJoint[RShoulderRoll] = RSJoints::RShoulderRoll;
    utJointToRSJoint[RElbowYaw] = RSJoints::RElbowYaw;
    utJointToRSJoint[RElbowRoll] = RSJoints::RElbowRoll;

    utJointToRSJoint[LHipYawPitch] = RSJoints::LHipYawPitch;
    utJointToRSJoint[LHipRoll] = RSJoints::LHipRoll;
    utJointToRSJoint[LHipPitch] = RSJoints::LHipPitch;
    utJointToRSJoint[LKneePitch] = RSJoints::LKneePitch;
    utJointToRSJoint[LAnklePitch] = RSJoints::LAnklePitch;
    utJointToRSJoint[LAnkleRoll] = RSJoints::LAnkleRoll;

    utJointToRSJoint[RHipYawPitch] = -1;//RSJoints::RHipYawPitch;
    utJointToRSJoint[RHipRoll] = RSJoints::RHipRoll;
    utJointToRSJoint[RHipPitch] = RSJoints::RHipPitch;
    utJointToRSJoint[RKneePitch] = RSJoints::RKneePitch;
    utJointToRSJoint[RAnklePitch] = RSJoints::RAnklePitch;
    utJointToRSJoint[RAnkleRoll] = RSJoints::RAnkleRoll;

    // SENSOR VALUE CONVERSION
    utSensorToRSSensor[gyroX] = RSSensors::InertialSensor_GyrX;
    utSensorToRSSensor[gyroY] = RSSensors::InertialSensor_GyrY;   
    utSensorToRSSensor[gyroZ] = RSSensors::InertialSensor_GyrRef;   
    utSensorToRSSensor[accelX] = RSSensors::InertialSensor_AccX;
    utSensorToRSSensor[accelY] = RSSensors::InertialSensor_AccY;
    utSensorToRSSensor[accelZ] = RSSensors::InertialSensor_AccZ;   
    utSensorToRSSensor[angleX] = RSSensors::InertialSensor_AngleX;
    utSensorToRSSensor[angleY] = RSSensors::InertialSensor_AngleY;
    utSensorToRSSensor[angleZ] = RSSensors::InertialSensor_AngleZ;
    utSensorToRSSensor[battery] = RSSensors::Battery_Current;
    utSensorToRSSensor[fsrLFL] = RSSensors::LFoot_FSR_FrontLeft;
    utSensorToRSSensor[fsrLFR] = RSSensors::LFoot_FSR_FrontRight;   
    utSensorToRSSensor[fsrLRL] = RSSensors::LFoot_FSR_RearLeft;
    utSensorToRSSensor[fsrLRR] = RSSensors::LFoot_FSR_RearRight;
    utSensorToRSSensor[fsrRFL] = RSSensors::RFoot_FSR_FrontLeft;   
    utSensorToRSSensor[fsrRFR] = RSSensors::RFoot_FSR_FrontRight;
    utSensorToRSSensor[fsrRRL] = RSSensors::RFoot_FSR_RearLeft;
    utSensorToRSSensor[fsrRRR] = RSSensors::RFoot_FSR_RearRight;   
    utSensorToRSSensor[centerButton] = RSSensors::ChestBoard_Button;   
    utSensorToRSSensor[bumperLL] = RSSensors::LFoot_Bumper_Left;
    utSensorToRSSensor[bumperLR] = RSSensors::LFoot_Bumper_Right;
    utSensorToRSSensor[bumperRL] = RSSensors::RFoot_Bumper_Left;   
    utSensorToRSSensor[bumperRR] = RSSensors::RFoot_Bumper_Right;
    // No UT Sensors for RSSensors RFoot_FSR_CenterOfPressure_X/Y, Battery_Charge or US, not sure what these are - Josiah


}

RSWalkModule2014::~RSWalkModule2014() {
}

void RSWalkModule2014::specifyMemoryDependency() {
    requiresMemoryBlock("frame_info");
    requiresMemoryBlock("processed_joint_angles");
    requiresMemoryBlock("raw_joint_angles");
    requiresMemoryBlock("processed_joint_commands");
    requiresMemoryBlock("kick_request");
    requiresMemoryBlock("odometry");
    requiresMemoryBlock("robot_info");
    requiresMemoryBlock("raw_sensors");
    requiresMemoryBlock("walk_info");
    requiresMemoryBlock("walk_param");
    requiresMemoryBlock("walk_request");
    requiresMemoryBlock("body_model");
    requiresMemoryBlock("world_objects");//for rswalk2014
    requiresMemoryBlock("speech");
    requiresMemoryBlock("game_state");
    requiresMemoryBlock("robot_state");
}

void RSWalkModule2014::specifyMemoryBlocks() {
    getOrAddMemoryBlock(frame_info_,"frame_info");
    getOrAddMemoryBlock(joints_,"processed_joint_angles");
    getOrAddMemoryBlock(raw_joints_,"raw_joint_angles");
    getOrAddMemoryBlock(commands_,"processed_joint_commands");
    getOrAddMemoryBlock(kick_request_,"kick_request");
    getOrAddMemoryBlock(odometry_,"odometry");
    getOrAddMemoryBlock(robot_info_,"robot_info");
    getOrAddMemoryBlock(sensors_,"raw_sensors");
    getOrAddMemoryBlock(walk_info_,"walk_info");
    getOrAddMemoryBlock(walk_params_,"walk_param");
    getOrAddMemoryBlock(walk_request_,"walk_request");
    getOrAddMemoryBlock(body_model_,"body_model");
    getOrAddMemoryBlock(world_objects_,"world_objects");//for rswalk2014
    getOrAddMemoryBlock(speech_, "speech");
    memory_->getBlockByName(game_state_, "game_state", MemoryOwner::VISION);
    memory_->getBlockByName(robot_state_, "robot_state", MemoryOwner::VISION);
    memory_->getOrAddBlockByName(world_objects_,"world_objects",MemoryOwner::SHARED);//for rswalk2014
    memory_->getOrAddBlockByName(speech_,"speech",MemoryOwner::SHARED);
}

void RSWalkModule2014::initSpecificModule() {
    // For RSWalk
    generator = new WalkEnginePreProcessor(); 
    std::string config_path = memory_->data_path_;
    config_path += "/config/rswalk2014"; 
    clipper = new ClippedGenerator((Generator*) new DistributedGenerator(config_path));
    readOptions(config_path);
    standing = false;
    prev_command = WalkRequestBlock::NONE;
    x_target = -1;
    y_target = -1;
    wasKicking = false;


    auto path = util::format("%s/%i_xy.yaml", util::cfgpath(util::GyroConfigs), robot_state_->robot_id_);
    GyroConfig config;
    if(config.load(path)) {
      printf("Loaded XY gyro configuration for Robot %02i.\n", robot_state_->robot_id_);
      offsetX = config.offsetX;
      offsetY = config.offsetY;
      calibration_write_time = config.calibration_write_time;
    }
    if (GSL_COLLECT_DATA)
    	outfile.open("gsl_joint_data.csv");
}


void RSWalkModule2014::writeDataToFile(SensorValues sensors, JointValues joints) {
	outfile << "frameID=" << frame_info_->frame_id << ",";
        // Joints 
        for (int ut_ind=0; ut_ind<NUM_JOINTS; ut_ind++){
        	if (ut_ind != RHipYawPitch){ // RS does not have this joint because RHYP is same as LHYP on Nao
                	int rs_ind = utJointToRSJoint[ut_ind];
                        outfile << RSJoints::jointNames[rs_ind] << "State=" << sensors.joints.angles[rs_ind] << ",";
                }
        }
        //Sensors
        for (int ut_ind=0; ut_ind<bumperRR + 1; ut_ind++){
        	int rs_ind = utSensorToRSSensor[ut_ind];
                outfile << RSSensors::sensorNames[rs_ind] << "=" << sensors.sensors[rs_ind] << ",";
        }
        // Joint Commands
        for (int ut_ind = BODY_JOINT_OFFSET; ut_ind < NUM_JOINTS; ut_ind++) {
        	if (ut_ind != RHipYawPitch){ // RS does not have this joint - Josiah
                	int rs_ind = utJointToRSJoint[ut_ind];
                        	outfile << RSJoints::jointNames[rs_ind] << "Command=" << robot_joint_signs[ut_ind] * joints.angles[rs_ind] << ",";
                }
        }
	outfile << std::endl;
}

void RSWalkModule2014::getArmsForState(int state, Joints angles) {
    if (state <= 1) {
        angles[LShoulderPitch] = DEG_T_RAD * -116;
        angles[LShoulderRoll] = DEG_T_RAD * 15;//12;
        angles[LElbowYaw] = DEG_T_RAD * -85;
        angles[LElbowRoll] = DEG_T_RAD * -0;
        angles[RShoulderPitch] = DEG_T_RAD * -116;
        angles[RShoulderRoll] = DEG_T_RAD * 15;//12;
        angles[RElbowYaw] = DEG_T_RAD * -85;
        angles[RElbowRoll] = DEG_T_RAD * -0;
        if (state == 1) {
            angles[LElbowYaw] = DEG_T_RAD * 25;
            angles[RElbowYaw] = DEG_T_RAD * 25;
        }
    } else {
        angles[LShoulderPitch] = DEG_T_RAD * -116;
        angles[LShoulderRoll] = DEG_T_RAD * 8;//8;
        angles[LElbowYaw] = DEG_T_RAD * 25;
        angles[LElbowRoll] = DEG_T_RAD * -53;
        angles[RShoulderPitch] = DEG_T_RAD * -116;
        angles[RShoulderRoll] = DEG_T_RAD * 8;//8;
        angles[RElbowYaw] = DEG_T_RAD * 25;
        angles[RElbowRoll] = DEG_T_RAD * -53;
    }
}

void RSWalkModule2014::determineStartingArmState() {
    // start from the current joints
    for (int i = ARM_JOINT_FIRST; i <= ARM_JOINT_LAST; i++) {
        armStart[i] = joints_->values_[i];
    }

    // if arms are far out, start at 0
    //for (int i = 0; i < 2; i++) {
    //int shoulderPitch = LShoulderPitch;
    //int shoulderRoll = LShoulderRoll;
    //if (i == 1) {
    //shoulderPitch = RShoulderPitch;
    //shoulderRoll = RShoulderRoll;
    //}
    //if ((joints_->values_[shoulderRoll] > DEG_T_RAD * 45) || // arm is up (from cross)
    //(joints_->values_[shoulderPitch] > DEG_T_RAD * -90)) { // arm is in front
    //arm_state_ = 0;
    //return;
    //}
    //}

    for (int state = 2; state >= 1; state--) {
        Joints temp;
        getArmsForState(state,temp);
        bool acceptable = true;
        for (int i = ARM_JOINT_FIRST; i <= ARM_JOINT_LAST; i++) {
            if (fabs(joints_->values_[i] - temp[i]) > DEG_T_RAD * 10) {
                //std::cout << JointNames[i] << " is too far for state " << state << " sensed: " << RAD_T_DEG * joints_->values_[i] << " " << " desired: " << RAD_T_DEG * temp[i] << std::endl;
                acceptable = false;
                break;
            }
        }
        if (acceptable) {
            arm_state_ = state;
            //std::cout << "selected: " << arm_state_ << std::endl;
            return;
        }
    }
    // default to 0 if everything else has been bad
    arm_state_ = 0;
}

void RSWalkModule2014::setArms(Joints angles, float timeInSeconds) {

    float armStateTimes[3] = {1.0,0.5,0.5};
    
    if (timeInSeconds < 0.01)
        timeInSeconds = 0.01;

    float timePassed = frame_info_->seconds_since_start - arm_state_change_;

    int prevState = arm_state_;
    if (arm_state_ < 0) {
        determineStartingArmState();
    } else if (arm_state_ >= 2)
        arm_state_ = 2;
    else if (timePassed > armStateTimes[arm_state_]) {
        arm_state_ += 1;
    }
    // goal keeper only does state 0 ever
    if (walk_request_->keep_arms_out_){
        arm_state_ = 0;
    }

    if (arm_state_ != prevState) {
        //std::cout << frame_info_->frame_id << " changing state from " << prevState << " to " << arm_state_ << " after " << timePassed << " seconds" << std::endl;
        arm_state_change_ = frame_info_->seconds_since_start;
        timePassed = 0;
        // save previous commands as start
        if (prevState >= 0) {
            getArmsForState(prevState,armStart);
        }
    }

    // calculate the fraction we're into this state
    float frac = (timePassed + timeInSeconds) / armStateTimes[arm_state_];
    frac = crop(frac,0,1.0);

    // get desired angles
    getArmsForState(arm_state_,angles);

    // set the values
    for (int i = ARM_JOINT_FIRST; i <= ARM_JOINT_LAST; i++) {
        float des = angles[i];
        float orig = armStart[i];
        float val = frac * (des - orig) + orig;
        angles[i] = val;
    }

    // see if the arms are stable
    float maxDeltaDesired = 0;
    float maxDeltaDetected = 0;
    for (int i = LShoulderPitch; i <= RElbowRoll; i++) {
        float delta = angles[i] - joints_->values_[i];
        maxDeltaDesired = max(fabs(delta),maxDeltaDesired);
        maxDeltaDetected = max(fabs(joints_->getJointDelta(i)),maxDeltaDetected);
    }
    arms_close_to_targets_ = (maxDeltaDesired < DEG_T_RAD * 20) || (maxDeltaDetected < DEG_T_RAD * 0.35);
}

const float RSWalkModule2014::STAND_ANGLES[NUM_JOINTS] = {
    0,
    -0.366519,
    0,
    0.00669175,
    -0.548284,
    1.04734,
    -0.499061,
    -0.00669175,
    0,
    -0.00669175,
    -0.548284,
    1.04734,
    -0.499061,
    0.00669175,
    -1.5708,
    0.2,
    -1.5708,
    -0.2,
    -1.5708,
    0.2,
    -1.5708,
    -0.2
};

void RSWalkModule2014::selectivelySendStiffness() {
    for (int i = 0; i < NUM_JOINTS; i++) {
        if (fabs(joints_->stiffness_[i] - commands_->stiffness_[i]) > 0.01) {
            commands_->send_stiffness_ = true;
            commands_->stiffness_time_ = 10;
            return;
        }
    }
}

bool RSWalkModule2014::doingSlowStand() {
    return frame_info_->seconds_since_start < slow_stand_end;
}

void RSWalkModule2014::doSlowStand() {
    float dt = slow_stand_end - frame_info_->seconds_since_start;
    for (int i = BODY_JOINT_OFFSET; i < NUM_JOINTS; i++) {
        commands_->angles_[i] = STAND_ANGLES[i];
        commands_->stiffness_[i] = 1.0;
    }
    for (int ind = ARM_JOINT_FIRST; ind <= ARM_JOINT_LAST; ind ++)
	commands_->stiffness_[ind] = 0.75; //0.75

    setArms(commands_->angles_.data(),dt);
    commands_->send_body_angles_ = true;
    commands_->body_angle_time_ = 1000 * dt;
    selectivelySendStiffness();
}

bool RSWalkModule2014::readyToStartKickAfterStep() {
    return !(clipper->isStanding()) && (step_into_kick_state_ == PERFORMING);	
}

//TODO
void RSWalkModule2014::handleStepIntoKick() {
  //cout << "handleStepIntoKick"<<endl;
    if ((walk_request_->new_command_) && (walk_request_->perform_kick_) && (walk_request_->step_into_kick_) && (step_into_kick_state_ == NONE)) {
	//cout << "Kick: LINE UP" << endl;
        //std::cout << frame_info_->frame_id << " RECEIVED STEP_INTO_KICK" << std::endl;
        //WalkRequest &walk_request_BH = walk_engine_->theMotionRequest.walkRequest;
//        MotionRequest::Motion &motion = walk_engine_->theMotionRequest.motion;
        kick_request_->finished_with_step_ = false;
        step_into_kick_state_ = PERFORMING;
//        motion = MotionRequest::walk;
//        walk_request_BH.mode = WalkRequest::percentageSpeedMode;
//        walk_request_BH.speed = Pose2DBH(0,0,0);
//        if (walk_request_->kick_with_left_)
//            walk_request_BH.kickType = WalkRequest::stepForLeftKick;
//        else
//            walk_request_BH.kickType = WalkRequest::stepForRightKick;
    } 

    if (readyToStartKickAfterStep()) {
	//cout << "Kick: READY to KICK" << endl;
        step_into_kick_state_ = FINISHED_WITH_STEP;
        walk_request_->noWalk();
        kick_request_->finished_with_step_ = true;
        time_step_into_kick_finished_ = frame_info_->seconds_since_start;
        //std::cout << frame_info_->frame_id << " finished with step into kick" << std::endl;
        return;
    }

    if (step_into_kick_state_ == FINISHED_WITH_STEP) {
        if (kick_request_->kick_running_ || kick_request_->vision_kick_running_ || (frame_info_->seconds_since_start - time_step_into_kick_finished_ < 0.1)) {
            //std::cout << frame_info_->frame_id << " kick is running: " << kick_request_->kick_running_ << " " << kick_request_->vision_kick_running_ << std::endl;
        } else {
            //std::cout << frame_info_->frame_id << " done" << std::endl;
            step_into_kick_state_ = NONE;
        }
	//cout << "Kick: finished with step" << endl;
        walk_request_->noWalk();
        //walk_engine_->reset();
    }

    if (step_into_kick_state_ == PERFORMING) {
        kick_request_->setNoKick();
	//cout << "Kick: Performing" << endl;
    }

}
