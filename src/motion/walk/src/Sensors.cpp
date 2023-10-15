#include "walk/bodyV6.hpp"
#include "walk/JointValues.hpp"
#include "walk/Sensors.hpp"
#include "nao_sensor_msgs/msg/joint_positions.hpp"
#include "nao_sensor_msgs/msg/gyroscope.hpp"
#include "nao_sensor_msgs/msg/angle.hpp"
#include "nao_sensor_msgs/msg/fsr.hpp"
#include "nao_command_msgs/msg/joint_positions.hpp"
#include "nao_command_msgs/msg/joint_stiffnesses.hpp"


void SensorValues::populate_joint_positions(nao_sensor_msgs::msg::JointPositions::SharedPtr joint_positions)
{
    // Required Joint angles
    joints.angles[Joints::LKneePitch] = joint_positions->positions[10];
    joints.angles[Joints::RKneePitch] = joint_positions->positions[15];
    joints.angles[Joints::LHipRoll] = joint_positions->positions[8];
    joints.angles[Joints::RHipRoll] = joint_positions->positions[13]; 
}


void SensorValues::populate_gyro(nao_sensor_msgs::msg::Gyroscope::SharedPtr gyro)
{
    //Gyroscope
    sensors[Sensors::InertialSensor_GyroscopeX] = gyro->x;
    sensors[Sensors::InertialSensor_GyroscopeY] = gyro->y;
    sensors[Sensors::InertialSensor_GyroscopeZ] = gyro->z;    
}

void SensorValues::populate_angles(nao_sensor_msgs::msg::Angle::SharedPtr angles)
{
    //Angle
    sensors[Sensors::InertialSensor_AngleX] = angles->x;
    sensors[Sensors::InertialSensor_AngleY] = angles->y;
}


void SensorValues::populate_fsr(nao_sensor_msgs::msg::FSR::SharedPtr fsr)
{
    //FSR
    sensors[Sensors::LFoot_FSR_FrontLeft] = fsr->l_foot_front_left;
    sensors[Sensors::LFoot_FSR_FrontRight] = fsr->l_foot_front_right;
    sensors[Sensors::LFoot_FSR_RearLeft] = fsr->l_foot_back_left;
    sensors[Sensors::LFoot_FSR_RearRight] = fsr->l_foot_back_right;

    sensors[Sensors::RFoot_FSR_FrontLeft] = fsr->r_foot_front_left;
    sensors[Sensors::RFoot_FSR_FrontRight] = fsr->r_foot_front_right;
    sensors[Sensors::RFoot_FSR_RearLeft] = fsr->r_foot_back_left;
    sensors[Sensors::RFoot_FSR_RearRight] = fsr->r_foot_back_right;
}