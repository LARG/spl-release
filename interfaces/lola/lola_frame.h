#pragma once

#include <map>
#include <string>
#include <vector>
#include <array>

#include <msgpack.hpp>
#include <common/RobotInfo.h>

#include "battery.h"
#include "fsr.h"
#include "imu.h"
#include "joints.h"
#include "leds.h"
#include "buttons.h"

#include <memory/JointCommandBlock.h>
#include <memory/JointBlock.h>
#include <memory/SensorBlock.h>
#include <memory/LEDBlock.h>

#include "boost/asio.hpp"
#include "lola_interpolator.h"

const int MAX_LEN = 10000;

struct Sonars {
    float left = 4.0;
    float right = 4.0;
};

struct SonarCommand {
    bool left = true;
    bool right = true;
};

struct LolaSensorFrame {
    Lola::Joints joints;
    IMU imu;
    FSR fsr;
    Battery battery;
    Buttons buttons;
    Sonars sonars;
};


struct LolaActuatorFrame {
    Lola::Joints joints;
    Leds leds;
    SonarCommand sonars;
};

class LolaFrameHandler {
public:
    LolaFrameHandler(boost::asio::io_service& io_service);
    ~LolaFrameHandler();
    const LolaSensorFrame& unpack(const char* const buffer, size_t size);
    void get_sensor_values(JointBlock* joint_angles, SensorBlock* sensors);
    void set_actuator_values(JointCommandBlock *raw_joint_commands, JointBlock* raw_joint_angles_, LEDBlock* led_block);
    void send_leds(LEDBlock* led_block);
    void set_actuator_values(JointCommandBlock *raw_joint_commands, JointBlock* raw_joint_angles_);
    // void new_set_actuator_values(JointCommandBlock* raw_joint_commands);
    
    std::pair<char*, size_t> pack();

    LolaActuatorFrame actuator_frame;

private:
    void initSensorFrame();
    void initActuatorFrame();
    void initIxDictionary();

    boost::asio::local::stream_protocol::socket socket;
    char data[MAX_LEN] = {'\0'};
    int utToLolaJointIx[NUM_JOINTS];
    int utToLolaSensorIx[NUM_SENSORS];
    int utToLolaLed[NUM_LEDS];

    LolaSensorFrame sensor_frame;
    std::map<std::string, std::vector<float*>> sensor_frame_positions;
    std::map<std::string, std::vector<int*>> sensor_integer_data;
    std::map<std::string, std::vector<float*>> actuator_frame_positions;
    std::map<std::string, std::vector<bool*>> sonar_actuator_frame_positions;
    msgpack::sbuffer buffer;

    LolaInterpolator head_yaw;
    LolaInterpolator stiffness;
    LolaInterpolator head_pitch;
    LolaInterpolator body;
};
