#ifndef _SENSOR_VAL
#define _SENSOR_VAL

#include "walk/bodyV6.hpp"
#include "walk/JointValues.hpp"
#include "nao_sensor_msgs/msg/joint_positions.hpp"
#include "nao_sensor_msgs/msg/gyroscope.hpp"
#include "nao_sensor_msgs/msg/angle.hpp"
#include "nao_sensor_msgs/msg/fsr.hpp"
#include "nao_command_msgs/msg/joint_positions.hpp"
#include "nao_command_msgs/msg/joint_stiffnesses.hpp"

/**
 * A container for joint values, IMU values, FSR values, buttons and sonar
 * readings obtained from the Motion subsystem.
 */
struct SensorValues 
{
   SensorValues() 
   {
      for (int i = 0; i < Sensors::NUMBER_OF_SENSORS; ++i) {
         sensors[i] = NAN;
      }
   }

   SensorValues(bool zero) 
   {
      joints = JointValues(true);
      for (int i = 0; i < Sensors::NUMBER_OF_SENSORS; ++i) {
         sensors[i] = 0;
      }
   }

   JointValues joints;
   float sensors[Sensors::NUMBER_OF_SENSORS];

   void populate_joint_positions(nao_sensor_msgs::msg::JointPositions::SharedPtr joint_positions);
   void populate_gyro(nao_sensor_msgs::msg::Gyroscope::SharedPtr gyro);
   void populate_angles(nao_sensor_msgs::msg::Angle::SharedPtr angles);
   void populate_fsr(nao_sensor_msgs::msg::FSR::SharedPtr fsr);

   // void make_joint_msgs(const JointValues &joints, nao_command_msgs::msg::JointPositions &joint_angles,  nao_command_msgs::msg::JointStiffnesses &joint_stiffness);
};

#endif

