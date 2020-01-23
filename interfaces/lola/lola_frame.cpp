#include "lola_frame.h"

#include <iostream>
#include <math/Geometry.h>
#include <stdio.h>
#include <cmath>

#include "stl_ext.h"
#include <assert.h>

/*
 * This file contains most of the code related to the LoLA connector.
 * 
 * The code in this file is used to recieve sensor values from LoLA and pass it
 * along to higher level parts of the codebase, and to send actuator commands
 * from higher levels to the actuators in the robot. Note that the camera is
 * the only sensor which is not covered by this file. Before sending joint
 * position and joint stiffness values to the robot, this file also performs
 * interpolation between initial and target values.
 */

using namespace std;



LolaFrameHandler::LolaFrameHandler(boost::asio::io_service& io_service) :
    // The file ~ /robocup.conf must exist at boot - up for this socket to exist.
    socket(io_service),
    head_yaw(0, 0, false),
    stiffness(0, NUM_JOINTS-1, false, false),
    head_pitch(1,1),
    body(BODY_JOINT_OFFSET, NUM_JOINTS-1, true)
{
    socket.connect("/tmp/robocup");
    // The indexing in LoLA is different from the indexing in the rest of our
    // codebase.
    initIxDictionary();
    initSensorFrame();
    initActuatorFrame();
}

LolaFrameHandler::~LolaFrameHandler() {
    /* The destructor must be called when LoLA is killed since it sets 
     * joint stiffnesses to -1, therefore saving battery and letting joints
     * cool down.
     * Note: in our codebase, the range of stiffness is 0 to 1, but in LoLA
     * it ranges from -1 to 1.
     */
    std::cout << "Destructing LolaFrameHandler" << std::endl;
    socket.receive(boost::asio::buffer(data, MAX_LEN));
    set_stiffness(-1.f, &actuator_frame.joints.legs);
    set_stiffness(-1.f, &actuator_frame.joints.arms);
    set_stiffness(-1.f, &actuator_frame.joints.head);
    set_led(-1.f, 0.f, 0.f, &actuator_frame.leds.chest);
    char* buffer;
    size_t size;
    std::tie(buffer, size) = pack();
    this->socket.send(boost::asio::buffer(buffer, size));
}


void LolaFrameHandler::initSensorFrame() {
    // This function sets up memory locations where readings from LoLA are
    // stored.
    auto& gyr = sensor_frame.imu.gyr;
    auto& accel = sensor_frame.imu.accel;
    auto& head = sensor_frame.joints.head;
    auto& arms = sensor_frame.joints.arms;
    auto& legs = sensor_frame.joints.legs;
    auto& fsr = sensor_frame.fsr;
    auto& battery = sensor_frame.battery;
    auto& angles = sensor_frame.imu.angles;
    auto& buttons = sensor_frame.buttons;
    auto& sonars = sensor_frame.sonars;
    // There might be joints missing here, which can be added in using
    // Softbank's documentation.
    sensor_frame_positions = {
        {"Angles", {&angles.x, &angles.y}},
        {"Touch", {&buttons.chest,
                   &buttons.head.front,
                   &buttons.head.middle,
                   &buttons.head.rear,
                   &buttons.left_foot.left,
                   &buttons.left_foot.right,
                   &buttons.left_hand.back,
                   &buttons.left_hand.left,
                   &buttons.left_hand.right,
                   &buttons.right_foot.left,
                   &buttons.right_foot.right,
                   &buttons.right_hand.back,
                   &buttons.right_hand.left,
                   &buttons.right_hand.right}},
        {"Accelerometer", {&accel.x, &accel.y, &accel.z}},
        {"Gyroscope", {&gyr.roll, &gyr.pitch, &gyr.yaw}},
        // The order for battery values in Softbank's documentation is wrong.
        {"Battery", {&battery.charge, &battery.status, &battery.current, &battery.temp}},
        {"FSR", {&fsr.left.fl, &fsr.left.fr, &fsr.left.rl, &fsr.left.rr,
                 &fsr.right.fl, &fsr.right.fr, &fsr.right.rl, &fsr.right.rr}},
        {"Position", {
             &head[Lola::HeadYaw].angle, &head[Lola::HeadPitch].angle,
             &arms[Lola::LShoulderPitch].angle, &arms[Lola::LShoulderRoll].angle, &arms[Lola::LElbowYaw].angle,
             &arms[Lola::LElbowRoll].angle, &arms[Lola::LWristYaw].angle,
             &legs[Lola::HipYawPitch].angle, &legs[Lola::LHipRoll].angle, &legs[Lola::LHipPitch].angle,
             &legs[Lola::LKneePitch].angle, &legs[Lola::LAnklePitch].angle, &legs[Lola::LAnkleRoll].angle,
             &legs[Lola::RHipRoll].angle,  &legs[Lola::RHipPitch].angle, &legs[Lola::RKneePitch].angle,
             &legs[Lola::RAnklePitch].angle, &legs[Lola::RAnkleRoll].angle,
             &arms[Lola::RShoulderPitch].angle, &arms[Lola::RShoulderRoll].angle, &arms[Lola::RElbowYaw].angle,
             &arms[Lola::RElbowRoll].angle, &arms[Lola::RWristYaw].angle,
             &arms[Lola::LHand].angle, &arms[Lola::RHand].angle}},
        {"Stiffness", {
             &head[Lola::HeadYaw].stiffness, &head[Lola::HeadPitch].stiffness,
             &arms[Lola::LShoulderPitch].stiffness, &arms[Lola::LShoulderRoll].stiffness, &arms[Lola::LElbowYaw].stiffness,
             &arms[Lola::LElbowRoll].stiffness, &arms[Lola::LWristYaw].stiffness,
             &legs[Lola::HipYawPitch].stiffness, &legs[Lola::LHipRoll].stiffness, &legs[Lola::LHipPitch].stiffness,
             &legs[Lola::LKneePitch].stiffness, &legs[Lola::LAnklePitch].stiffness, &legs[Lola::LAnkleRoll].stiffness,
             &legs[Lola::RHipRoll].stiffness,  &legs[Lola::RHipPitch].stiffness, &legs[Lola::RKneePitch].stiffness,
             &legs[Lola::RAnklePitch].stiffness, &legs[Lola::RAnkleRoll].stiffness,
             &arms[Lola::RShoulderPitch].stiffness, &arms[Lola::RShoulderRoll].stiffness, &arms[Lola::RElbowYaw].stiffness,
             &arms[Lola::RElbowRoll].stiffness, &arms[Lola::RWristYaw].stiffness,
             &arms[Lola::LHand].stiffness, &arms[Lola::RHand].stiffness}},
        {"Current", {
             &head[Lola::HeadYaw].current, &head[Lola::HeadPitch].current,
             &arms[Lola::LShoulderPitch].current, &arms[Lola::LShoulderRoll].current, &arms[Lola::LElbowYaw].current,
             &arms[Lola::LElbowRoll].current, &arms[Lola::LWristYaw].current,
             &legs[Lola::HipYawPitch].current, &legs[Lola::LHipRoll].current, &legs[Lola::LHipPitch].current,
             &legs[Lola::LKneePitch].current, &legs[Lola::LAnklePitch].current, &legs[Lola::LAnkleRoll].current,
             &legs[Lola::RHipRoll].current,  &legs[Lola::RHipPitch].current, &legs[Lola::RKneePitch].current,
             &legs[Lola::RAnklePitch].current, &legs[Lola::RAnkleRoll].current,
             &arms[Lola::RShoulderPitch].current, &arms[Lola::RShoulderRoll].current, &arms[Lola::RElbowYaw].current,
             &arms[Lola::RElbowRoll].current, &arms[Lola::RWristYaw].current,
             &arms[Lola::LHand].current, &arms[Lola::RHand].current}},
        {"Temperature", {
             &head[Lola::HeadYaw].temperature, &head[Lola::HeadPitch].temperature,
             &arms[Lola::LShoulderPitch].temperature, &arms[Lola::LShoulderRoll].temperature, &arms[Lola::LElbowYaw].temperature,
             &arms[Lola::LElbowRoll].temperature, &arms[Lola::LWristYaw].temperature,
             &legs[Lola::HipYawPitch].temperature, &legs[Lola::LHipRoll].temperature, &legs[Lola::LHipPitch].temperature,
             &legs[Lola::LKneePitch].temperature, &legs[Lola::LAnklePitch].temperature, &legs[Lola::LAnkleRoll].temperature,
             &legs[Lola::RHipRoll].temperature,  &legs[Lola::RHipPitch].temperature, &legs[Lola::RKneePitch].temperature,
             &legs[Lola::RAnklePitch].temperature, &legs[Lola::RAnkleRoll].temperature,
             &arms[Lola::RShoulderPitch].temperature, &arms[Lola::RShoulderRoll].temperature, &arms[Lola::RElbowYaw].temperature,
             &arms[Lola::RElbowRoll].temperature, &arms[Lola::RWristYaw].temperature,
             &arms[Lola::LHand].temperature, &arms[Lola::RHand].temperature}},
        {"Sonar", {&sonars.left, &sonars.right}}
    };
    sensor_integer_data = {
        {"Status", {
             &head[Lola::HeadYaw].status, &head[Lola::HeadPitch].status,
             &arms[Lola::LShoulderPitch].status, &arms[Lola::LShoulderRoll].status, &arms[Lola::LElbowYaw].status,
             &arms[Lola::LElbowRoll].status, &arms[Lola::LWristYaw].status,
             &legs[Lola::HipYawPitch].status, &legs[Lola::LHipRoll].status, &legs[Lola::LHipPitch].status,
             &legs[Lola::LKneePitch].status, &legs[Lola::LAnklePitch].status, &legs[Lola::LAnkleRoll].status,
             &legs[Lola::RHipRoll].status,  &legs[Lola::RHipPitch].status, &legs[Lola::RKneePitch].status,
             &legs[Lola::RAnklePitch].status, &legs[Lola::RAnkleRoll].status,
             &arms[Lola::RShoulderPitch].status, &arms[Lola::RShoulderRoll].status, &arms[Lola::RElbowYaw].status,
             &arms[Lola::RElbowRoll].status, &arms[Lola::RWristYaw].status,
             &arms[Lola::LHand].status, &arms[Lola::RHand].status}},
    };
}

void LolaFrameHandler::initActuatorFrame() {
    /* This sets up memory locations from which LoLA reads when sending data
     * to the LoLA connector socket.
     */
    auto& head = actuator_frame.joints.head;
    auto& arms = actuator_frame.joints.arms;
    auto& legs = actuator_frame.joints.legs;
    auto& ears = actuator_frame.leds.ears;
    auto& eyes = actuator_frame.leds.eyes;
    auto& chest = actuator_frame.leds.chest;
    auto& lfoot = actuator_frame.leds.left_foot;
    auto& rfoot = actuator_frame.leds.right_foot;
    auto& skull = actuator_frame.leds.skull;
    auto& sonars = actuator_frame.sonars;
    // Also not all actuators, you know the drill. ;)
    actuator_frame_positions = {
        {"LEar", {&ears.left[0], &ears.left[1], &ears.left[2], &ears.left[3], &ears.left[4],
                  &ears.left[5], &ears.left[6], &ears.left[7], &ears.left[8], &ears.left[9]}},
        {"REar", {&ears.right[9], &ears.right[8], &ears.right[7], &ears.right[6], &ears.right[5],
                  &ears.right[4], &ears.right[3], &ears.right[2], &ears.right[1], &ears.right[0]}},
        // What did they take to come up with the ordering of the eye LEDs?
        {"LEye", {&eyes.left[1].r, &eyes.left[0].r, &eyes.left[7].r, &eyes.left[6].r,
                  &eyes.left[5].r, &eyes.left[4].r, &eyes.left[3].r, &eyes.left[2].r,
                  &eyes.left[1].g, &eyes.left[0].g, &eyes.left[7].g, &eyes.left[6].g,
                  &eyes.left[5].g, &eyes.left[4].g, &eyes.left[3].g, &eyes.left[2].g,
                  &eyes.left[1].b, &eyes.left[0].b, &eyes.left[7].b, &eyes.left[6].b,
                  &eyes.left[5].b, &eyes.left[4].b, &eyes.left[3].b, &eyes.left[2].b}},
        {"REye", {&eyes.right[0].r, &eyes.right[1].r, &eyes.right[2].r, &eyes.right[3].r,
                  &eyes.right[4].r, &eyes.right[5].r, &eyes.right[6].r, &eyes.right[7].r,
                  &eyes.right[0].g, &eyes.right[1].g, &eyes.right[2].g, &eyes.right[3].g,
                  &eyes.right[4].g, &eyes.right[5].g, &eyes.right[6].g, &eyes.right[7].g,
                  &eyes.right[0].b, &eyes.right[1].b, &eyes.right[2].b, &eyes.right[3].b,
                  &eyes.right[4].b, &eyes.right[5].b, &eyes.right[6].b, &eyes.right[7].b}},
        {"Chest", {&chest.r, &chest.g, &chest.b}},
        {"LFoot", {&lfoot.r, &lfoot.g, &lfoot.b}},
        {"RFoot", {&rfoot.r, &rfoot.g, &rfoot.b}},
        {"Skull", {&skull.left[0], &skull.left[1], &skull.left[2], &skull.left[3], &skull.left[4], &skull.left[5],
                   &skull.right[0], &skull.right[1], &skull.right[2], &skull.right[3], &skull.right[4], &skull.right[5]}},
        {"Position", {
             &head[Lola::HeadYaw].angle, &head[Lola::HeadPitch].angle,
             &arms[Lola::LShoulderPitch].angle, &arms[Lola::LShoulderRoll].angle, &arms[Lola::LElbowYaw].angle,
             &arms[Lola::LElbowRoll].angle, &arms[Lola::LWristYaw].angle,
             &legs[Lola::HipYawPitch].angle, &legs[Lola::LHipRoll].angle, &legs[Lola::LHipPitch].angle,
             &legs[Lola::LKneePitch].angle, &legs[Lola::LAnklePitch].angle, &legs[Lola::LAnkleRoll].angle,
             &legs[Lola::RHipRoll].angle,  &legs[Lola::RHipPitch].angle, &legs[Lola::RKneePitch].angle,
             &legs[Lola::RAnklePitch].angle, &legs[Lola::RAnkleRoll].angle,
             &arms[Lola::RShoulderPitch].angle, &arms[Lola::RShoulderRoll].angle, &arms[Lola::RElbowYaw].angle,
             &arms[Lola::RElbowRoll].angle, &arms[Lola::RWristYaw].angle,
             &arms[Lola::LHand].angle, &arms[Lola::RHand].angle}},
        {"Stiffness", {
             &head[Lola::HeadYaw].stiffness, &head[Lola::HeadPitch].stiffness,
             &arms[Lola::LShoulderPitch].stiffness, &arms[Lola::LShoulderRoll].stiffness, &arms[Lola::LElbowYaw].stiffness,
             &arms[Lola::LElbowRoll].stiffness, &arms[Lola::LWristYaw].stiffness,
             &legs[Lola::HipYawPitch].stiffness, &legs[Lola::LHipRoll].stiffness, &legs[Lola::LHipPitch].stiffness,
             &legs[Lola::LKneePitch].stiffness, &legs[Lola::LAnklePitch].stiffness, &legs[Lola::LAnkleRoll].stiffness,
             &legs[Lola::RHipRoll].stiffness,  &legs[Lola::RHipPitch].stiffness, &legs[Lola::RKneePitch].stiffness,
             &legs[Lola::RAnklePitch].stiffness, &legs[Lola::RAnkleRoll].stiffness,
             &arms[Lola::RShoulderPitch].stiffness, &arms[Lola::RShoulderRoll].stiffness, &arms[Lola::RElbowYaw].stiffness,
             &arms[Lola::RElbowRoll].stiffness, &arms[Lola::RWristYaw].stiffness,
             &arms[Lola::LHand].stiffness, &arms[Lola::RHand].stiffness}},
    };
    sonar_actuator_frame_positions = {{"Sonar", {&sonars.left, &sonars.right}}};
}

void LolaFrameHandler::get_sensor_values(JointBlock* joint_angles, SensorBlock* sensors) {

    // unpack data recieved from the LoLA connector and pass it along to
    // higher levels of the codebase by setting values in the JointBlock and
    // SensorBlock memory blocks.
    this->unpack(this->data, this->socket.receive(boost::asio::buffer(data, MAX_LEN)));
    for(int i = 0; i < NUM_JOINTS; i++) {
        if (auto ptr = find(sensor_frame_positions, "Position")) {
            joint_angles->values_[i] = *(ptr.value())[utToLolaJointIx[i]];
        }
        if (auto ptr = find(sensor_frame_positions, "Stiffness")) {
            joint_angles->stiffness_[i] = *(ptr.value())[utToLolaJointIx[i]];
        }
        if (auto ptr = find(sensor_frame_positions, "Temperature")) {
            joint_angles->temperature_[i] = *(ptr.value())[utToLolaJointIx[i]];
        }
        if (auto ptr = find(sensor_frame_positions, "Current")) {
            joint_angles->current_[i] = *(ptr.value())[utToLolaJointIx[i]];
        }
        if (auto ptr = find(sensor_integer_data, "Status")) {
            joint_angles->status_[i] = *(ptr.value())[utToLolaJointIx[i]];
        }
    }

    // Look at the "Sensor" enum in core/common/RobotInfo.h in order to make
    // sense of this block of code. Basically we are searching for values in 
    // the recieved message and setting corresponding values in the sensor
    // block.
    std::string search_str;
    for(int i=0; i < NUM_SENSORS; i++) {
        if (i < accelX) search_str = "Gyroscope";
        else if (i < angleX) search_str = "Accelerometer";
        else if (i < battery) {
            if (i == angleZ) {
                sensors->values_[i] = 0;
                continue;
            }
            search_str = "Angles";
        } else if (i == battery) search_str = "Battery";
        else if (i < bumperLL) search_str = "FSR";
        else if (i < centerButton) {
            sensors->values_[i] = 0;
            continue;
        }
        else if (i <= headRear) {
            search_str = "Touch";
        }
        else {
            std::cout << i << " " << NUM_SENSORS << std::endl;
            assert(false);  // Should not get here.
        }
        if (auto ptr = find(sensor_frame_positions, search_str)) {
            sensors->values_[i] = *(ptr.value())[utToLolaSensorIx[i]];
        }
    }
    if(auto ptr = find(sensor_frame_positions, "Sonar")) {
        sensors->sonar_left_[0] = *(ptr.value())[0];
        sensors->sonar_right_[0] = *(ptr.value())[1];
    }
}


void LolaFrameHandler::set_actuator_values(JointCommandBlock *raw_joint_commands, JointBlock* raw_joint_angles_, LEDBlock* led_block) {
    /* This function takes actuator values from higher level code and passes
     * them along to the LoLA connector.
     */
    std::vector<float> interpolated_angle(NUM_JOINTS, 0);
    std::vector<float> interpolated_stiffness(NUM_JOINTS, -1);
    std::array<float, NUM_JOINTS> transformed_stiffness_;
 
    // The range of stiffnesses in our codebase is 0 to 1 whereas for LoLA
    // the range of stiffnesses is -1 to 1. We simply scale the values
    // recieved from higher levels of the codebase before passing them along.
    for(int i = 0; i < NUM_JOINTS; i++) {
        transformed_stiffness_[i] = raw_joint_commands->stiffness_[i] * 2 - 1;
    }
 
    // Gettng interpolated commands for head yaw, head pitch, body angles and
    // stiffnesses.

    head_yaw.nextMovement(
       raw_joint_commands->angles_,
       raw_joint_angles_->values_,
       raw_joint_commands->send_head_yaw_angle_,
       raw_joint_commands->head_yaw_angle_time_,
       interpolated_angle
    );
 
    head_pitch.nextMovement(
       raw_joint_commands->angles_,
       raw_joint_angles_->values_,
       raw_joint_commands->send_head_pitch_angle_,
       raw_joint_commands->head_pitch_angle_time_,
       interpolated_angle
   );
 
    body.nextMovement(
       raw_joint_commands->angles_,
       raw_joint_angles_->values_,
       raw_joint_commands->send_body_angles_,
       raw_joint_commands->body_angle_time_,
       interpolated_angle
    );
 
    stiffness.nextMovement(
       transformed_stiffness_,
       raw_joint_angles_->stiffness_,
       raw_joint_commands->send_stiffness_,
       raw_joint_commands->stiffness_time_,
       interpolated_stiffness
    );
  
    for(int i = 0; i < interpolated_angle.size(); i++) {
  
      if (auto ptr = find(actuator_frame_positions, "Position")) {
        *(ptr.value())[utToLolaJointIx[i]] = interpolated_angle[i];
      }
      if (auto ptr = find(actuator_frame_positions, "Stiffness")) {
        *(ptr.value())[utToLolaJointIx[i]] = interpolated_stiffness[i];
      }
    }
 
    send_leds(led_block);
    char* buffer;
    size_t size;
    tie(buffer, size) = pack();
    this->socket.send(boost::asio::buffer(buffer, size));
 
}


void LolaFrameHandler::send_leds(LEDBlock* led_block) {

  if ( not led_block->send_leds_ ) return;
  led_block->send_leds_ = false;  // reset variable

  std::string search_str;
  for(int i = 0; i < NUM_LEDS; i++) {
 
    if (i <= EarLeft324) search_str = "LEar";
    else if (i <= EarRight324) search_str = "REar";
    else if (i <= FaceBlueLeft315) search_str = "LEye";
    else if (i <= FaceBlueRight315) search_str = "REye";
    else if (i <= ChestBlue) search_str = "Chest";
    else if (i <= LFootBlue) search_str = "LFoot";
    else if (i <= RFootBlue) search_str = "RFoot";
    else search_str = "Skull";

    if (auto ptr = find(actuator_frame_positions, search_str)) {
       *(ptr.value())[utToLolaLed[i]] = led_block->values_[i];
    }
  }

}


const LolaSensorFrame& LolaFrameHandler::unpack(const char* const buffer, size_t size) {
    msgpack::unpacker pac;
    pac.reserve_buffer(size);
    memcpy(pac.buffer(), buffer, size);
    pac.buffer_consumed(size);
    msgpack::object_handle object_handler;
    if (!pac.next(object_handler)) {
        cerr << "No MsgPack message in LoLA message." << endl;
        return sensor_frame;
    }
    const auto& map = object_handler.get().via.map;
    auto* category = map.ptr;
    for (uint32_t i = 0; i < map.size; i++, category++) {
        if (auto ptr = find(sensor_frame_positions, category->key.as<string>())) {
            zip(*ptr, category->val.as<vector<float>>(), [](float* r, float f) { if (r != nullptr) *r = f; });
        } else if (auto ptr = find(sensor_integer_data, category->key.as<string>())) {
            zip(*ptr, category->val.as<vector<int>>(), [](int* r, int f) { if (r != nullptr) *r = f; });
        }
    }
    // These seem to be different between robots but consistent between reboots. Make sure you calibrate each robot otherwise most filters (e.g. Madgwick) won't work correctly!
    sensor_frame.imu.gyr.yaw *= -1.f; //z
    sensor_frame.imu.gyr.pitch *= 1.f; //y 
    sensor_frame.imu.gyr.roll *= 1.f; //x
    return sensor_frame;
}

pair<char*, size_t> LolaFrameHandler::pack() {
    buffer.clear();
    msgpack::packer<msgpack::sbuffer> pk(&buffer);
    pk.pack_map(actuator_frame_positions.size() +  sonar_actuator_frame_positions.size());
    for (const auto& kv : actuator_frame_positions) {
        pk.pack(kv.first);
        pk.pack_array(kv.second.size());
        for (float* val : kv.second) {
            pk.pack(*val);
        }
    }
    for (const auto& kv : sonar_actuator_frame_positions) {
        pk.pack(kv.first);
        pk.pack_array(kv.second.size());
        for (bool* val : kv.second) {
            pk.pack(*val);
        }
    }
    return {buffer.data(), buffer.size()};
}

void LolaFrameHandler::initIxDictionary() {
  // This list was made by looking at the "Position" vector inside sensor_frame_positions
  // 0. HeadYaw
  // 1. HeadPitch
  // 2. LShoulderPitch
  // 3. LShoulderRoll
  // 4. LElbowYaw
  // 5. LElbowRoll
  // 6. LWristYaw
  // 7. HipYawPitch
  // 8. LHipRoll
  // 9. LHipPitch
  // 10. LKneePitch
  // 11. LAnklePitch
  // 12. LAnkleRoll
  // 13. RHipRoll
  // 14. RHipPitch
  // 15. RKneePitch
  // 16. RAnklePitch
  // 17. RAnkleRoll
  // 18. RShoulderPitch
  // 19. RShoulderRoll
  // 20. RElbowYaw
  // 21. RElbowRoll
  // 22. RWristYaw
  // 23. LHand
  // 24. RHand

  this->utToLolaJointIx[HeadYaw] = 0;
  this->utToLolaJointIx[HeadPitch] = 1;
  this->utToLolaJointIx[LHipYawPitch] = 7;
  this->utToLolaJointIx[LHipRoll] = 8;
  this->utToLolaJointIx[LHipPitch] = 9;
  this->utToLolaJointIx[LKneePitch] = 10;
  this->utToLolaJointIx[LAnklePitch] = 11;
  this->utToLolaJointIx[LAnkleRoll] = 12;
  this->utToLolaJointIx[RHipYawPitch] = 7;
  this->utToLolaJointIx[RHipRoll] = 13;
  this->utToLolaJointIx[RHipPitch] = 14;
  this->utToLolaJointIx[RKneePitch] = 15;
  this->utToLolaJointIx[RAnklePitch] = 16;
  this->utToLolaJointIx[RAnkleRoll] = 17;
  this->utToLolaJointIx[LShoulderPitch] = 2;
  this->utToLolaJointIx[LShoulderRoll] = 3;
  this->utToLolaJointIx[LElbowYaw] = 4;
  this->utToLolaJointIx[LElbowRoll] = 5;
  this->utToLolaJointIx[RShoulderPitch] = 18;
  this->utToLolaJointIx[RShoulderRoll] = 19;
  this->utToLolaJointIx[RElbowYaw] = 20;
  this->utToLolaJointIx[RElbowRoll] = 21;

  utToLolaSensorIx[gyroX] = 0;
  utToLolaSensorIx[gyroY] = 1;
  utToLolaSensorIx[gyroZ] = 2;
  utToLolaSensorIx[accelX] = 0;
  utToLolaSensorIx[accelY] = 1;
  utToLolaSensorIx[accelZ] = 2;
  utToLolaSensorIx[angleX] = 0;
  utToLolaSensorIx[angleY] = 1;
  utToLolaSensorIx[angleZ] = -1;  // Not available yet
  utToLolaSensorIx[battery] = 0;
  utToLolaSensorIx[fsrLFL] = 0;
  utToLolaSensorIx[fsrLFR] = 1;
  utToLolaSensorIx[fsrLRL] = 2;
  utToLolaSensorIx[fsrLRR] = 3;
  utToLolaSensorIx[fsrRFL] = 4;
  utToLolaSensorIx[fsrRFR] = 5;
  utToLolaSensorIx[fsrRRL] = 6;
  utToLolaSensorIx[fsrRRR] = 7;
  utToLolaSensorIx[bumperLL] = 4;
  utToLolaSensorIx[bumperLR] = 5;
  utToLolaSensorIx[bumperRL] = 9;
  utToLolaSensorIx[bumperRR] = 10;
  utToLolaSensorIx[centerButton] = 0;
  utToLolaSensorIx[headFront] = 1;
  utToLolaSensorIx[headMiddle] = 2;
  utToLolaSensorIx[headRear] = 3;

  // Ear lights
  utToLolaLed[EarLeft0] = 0;
  utToLolaLed[EarLeft36] = 1;
  utToLolaLed[EarLeft72] = 2;
  utToLolaLed[EarLeft108] = 3;
  utToLolaLed[EarLeft144] = 4;
  utToLolaLed[EarLeft180] = 5;
  utToLolaLed[EarLeft216] = 6;
  utToLolaLed[EarLeft252] = 7;
  utToLolaLed[EarLeft288] = 8;
  utToLolaLed[EarLeft324] = 9;
  utToLolaLed[EarRight0] = 0;
  utToLolaLed[EarRight36] = 1;
  utToLolaLed[EarRight72] = 2;
  utToLolaLed[EarRight108] = 3;
  utToLolaLed[EarRight144] = 4;
  utToLolaLed[EarRight180] = 5;
  utToLolaLed[EarRight216] = 6;
  utToLolaLed[EarRight252] = 7;
  utToLolaLed[EarRight288] = 8;
  utToLolaLed[EarRight324] = 9;

  // Eye lights
  utToLolaLed[FaceRedLeft0] = 0;
  utToLolaLed[FaceRedLeft45] = 1;
  utToLolaLed[FaceRedLeft90] = 2;
  utToLolaLed[FaceRedLeft135] = 3;
  utToLolaLed[FaceRedLeft180] = 4;
  utToLolaLed[FaceRedLeft225] = 5;
  utToLolaLed[FaceRedLeft270] = 6;
  utToLolaLed[FaceRedLeft315] = 7;

  utToLolaLed[FaceGreenLeft0] = 8;
  utToLolaLed[FaceGreenLeft45] = 9;
  utToLolaLed[FaceGreenLeft90] = 10;
  utToLolaLed[FaceGreenLeft135] = 11;
  utToLolaLed[FaceGreenLeft180] = 12;
  utToLolaLed[FaceGreenLeft225] = 13;
  utToLolaLed[FaceGreenLeft270] = 14;
  utToLolaLed[FaceGreenLeft315] = 15;

  utToLolaLed[FaceBlueLeft0] = 16;
  utToLolaLed[FaceBlueLeft45] = 17;
  utToLolaLed[FaceBlueLeft90] = 18;
  utToLolaLed[FaceBlueLeft135] = 19;
  utToLolaLed[FaceBlueLeft180] = 20;
  utToLolaLed[FaceBlueLeft225] = 21;
  utToLolaLed[FaceBlueLeft270] = 22;
  utToLolaLed[FaceBlueLeft315] = 23;

  utToLolaLed[FaceRedRight0] = 0;
  utToLolaLed[FaceRedRight45] = 1;
  utToLolaLed[FaceRedRight90] = 2;
  utToLolaLed[FaceRedRight135] = 3;
  utToLolaLed[FaceRedRight180] = 4;
  utToLolaLed[FaceRedRight225] = 5;
  utToLolaLed[FaceRedRight270] = 6;
  utToLolaLed[FaceRedRight315] = 7;

  utToLolaLed[FaceGreenRight0] = 8;
  utToLolaLed[FaceGreenRight45] = 9;
  utToLolaLed[FaceGreenRight90] = 10;
  utToLolaLed[FaceGreenRight135] = 11;
  utToLolaLed[FaceGreenRight180] = 12;
  utToLolaLed[FaceGreenRight225] = 13;
  utToLolaLed[FaceGreenRight270] = 14;
  utToLolaLed[FaceGreenRight315] = 15;

  utToLolaLed[FaceBlueRight0] = 16;
  utToLolaLed[FaceBlueRight45] = 17;
  utToLolaLed[FaceBlueRight90] = 18;
  utToLolaLed[FaceBlueRight135] = 19;
  utToLolaLed[FaceBlueRight180] = 20;
  utToLolaLed[FaceBlueRight225] = 21;
  utToLolaLed[FaceBlueRight270] = 22;
  utToLolaLed[FaceBlueRight315] = 23;

  // Chest lights
  utToLolaLed[ChestRed] = 0;
  utToLolaLed[ChestGreen] = 1;
  utToLolaLed[ChestBlue] = 2;
  // Left foot lights
  utToLolaLed[LFootRed] = 0;
  utToLolaLed[LFootGreen] = 1;
  utToLolaLed[LFootBlue] = 2;
  // Right foot lights
  utToLolaLed[RFootRed] = 0;
  utToLolaLed[RFootGreen] = 1;
  utToLolaLed[RFootBlue] = 2;

  utToLolaLed[LHead0] = 0;
  utToLolaLed[LHead1] = 1;
  utToLolaLed[LHead2] = 2;
  utToLolaLed[LHead3] = 3;
  utToLolaLed[LHead4] = 4;
  utToLolaLed[LHead5] = 5;

  utToLolaLed[RHead5] = 6;
  utToLolaLed[RHead4] = 7;
  utToLolaLed[RHead3] = 8;
  utToLolaLed[RHead2] = 9;
  utToLolaLed[RHead1] = 10;
  utToLolaLed[RHead0] = 11;
  std::cout << "Loaded sensor and joint indices" << std::endl;
}
