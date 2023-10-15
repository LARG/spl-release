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

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "keyframe_motions/GetupGenerator.hpp"
#include "keyframe_motions/JointValues.hpp"
#include "keyframe_motions/Sensors.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nao_sensor_msgs/msg/joint_positions.hpp"
#include "nao_sensor_msgs/msg/gyroscope.hpp"
#include "nao_sensor_msgs/msg/angle.hpp"
#include "nao_sensor_msgs/msg/fsr.hpp"
#include "nao_command_msgs/msg/joint_positions.hpp"
#include "nao_command_msgs/msg/joint_stiffnesses.hpp"
#include "nao_command_msgs/msg/joint_indexes.hpp"

#include "keyframe_motion_msg/msg/motion.hpp"


using namespace std::placeholders;

void make_joint_msgs(const JointValues &joints, nao_command_msgs::msg::JointPositions &joint_angles,  nao_command_msgs::msg::JointStiffnesses &joint_stiffness)
{
    joint_angles.indexes.clear();
    joint_angles.positions.clear();
    joint_stiffness.indexes.clear();
    joint_stiffness.stiffnesses.clear();
    for(int i = 0; i < Joints::NUMBER_OF_JOINTS; i++)
    {
        joint_stiffness.indexes.push_back(i);
        joint_stiffness.stiffnesses.push_back(joints.stiffnesses[i]);
        joint_angles.positions.push_back(joints.angles[i]); //TODO: This could cause problems if default values in joints.angles are bad
        joint_angles.indexes.push_back(i); //TODO: This could cause problems if default values in joints.angles are bad

    }
}

class KeyframemotionNode : public rclcpp::Node
{
public:
  KeyframemotionNode()
  : Node("KeyframemotionNode"), sensor_values(true), getup_generator("BACK",this)
  {

    RCLCPP_INFO(this->get_logger(), "Hello from KeyframemotionNode\n");

    sub_joint_states =
      create_subscription<nao_sensor_msgs::msg::JointPositions>(
      "sensors/joint_positions", 1,
      [this](nao_sensor_msgs::msg::JointPositions::SharedPtr sensor_joints) {

        sensor_values.populate_joint_positions(sensor_joints);

          if (curr_motion == "stop"){

            is_motion_done.data = true;
            pub_is_motion_done->publish(is_motion_done);
            getup_generator.stop();

          }  else if (curr_motion == "getup" || curr_motion == "stand" || curr_motion == "goaliesit"|| curr_motion == "sit" || curr_motion == "unstiff"){
          // RCLCPP_INFO(this->get_logger(), "Recived angle reading, x=%f, y = %f \n",angle->x, angle->y);

            if (!getup_generator.isActive() && !started_moiton){
              RCLCPP_INFO(this->get_logger(), "Resetting keyframe motion!\n");
              getup_generator.reset();
              getup_generator.setConfig(curr_motion, motion_speed, direction);
              started_moiton = true;
              is_motion_done.data = false;
            }
            if (getup_generator.isActive()){
              JointValues j = getup_generator.makeJoints(sensor_values);
              make_joint_msgs(j, joint_angles, joint_stiffness);
              pub_joint_angles->publish(joint_angles);
              pub_joint_stiffness->publish(joint_stiffness);
            }

            if (!getup_generator.isActive() && started_moiton && !is_motion_done.data){
              RCLCPP_INFO(this->get_logger(), "Finished keyframe motion \n");
              is_motion_done.data = true;
              pub_is_motion_done->publish(is_motion_done);
            }

            if (!is_motion_done.data){
              pub_is_motion_done->publish(is_motion_done);
            }

        }


      });


    sub_angle = create_subscription<nao_sensor_msgs::msg::Angle>(
      "sensors/angle", 1,
      [this](nao_sensor_msgs::msg::Angle::SharedPtr angle) {
        // RCLCPP_INFO(this->get_logger(), "Recieved angle: %f\n",angle->y);
        sensor_values.populate_angles(angle);

      }
    );


    sub_motion_start =
      create_subscription<keyframe_motion_msg::msg::Motion>(
      "motion/keyframe_motions", 10,
      [this](keyframe_motion_msg::msg::Motion::SharedPtr motion_command) {

        getup_generator.stop();

        motion_speed = motion_command->speed;
        curr_motion = motion_command->motion;
        direction = motion_command->direction;
        started_moiton = false; //indicates that we are not in progress of another motion

        RCLCPP_INFO(this->get_logger(), "Recieved keyframe motion command: %s\n",curr_motion.c_str());


      });
    pub_joint_angles = create_publisher<nao_command_msgs::msg::JointPositions>("effectors/joint_positions", 10);
    pub_joint_stiffness = create_publisher<nao_command_msgs::msg::JointStiffnesses>("effectors/joint_stiffnesses", 10);
    pub_is_motion_done = create_publisher<std_msgs::msg::Bool>("keyframe_motions/is_done", 10);

  }


private:

  GetupGenerator getup_generator;
  // ActionCommand action_command;
  SensorValues sensor_values;
  // BodyModel bodyModel;

  nao_command_msgs::msg::JointPositions joint_angles;
  nao_command_msgs::msg::JointStiffnesses joint_stiffness;
  std_msgs::msg::Bool is_motion_done;


  rclcpp::Subscription<nao_sensor_msgs::msg::Angle>::SharedPtr sub_angle;
  rclcpp::Subscription<keyframe_motion_msg::msg::Motion>::SharedPtr sub_motion_start;

  rclcpp::Publisher<nao_command_msgs::msg::JointPositions>::SharedPtr pub_joint_angles;
  rclcpp::Publisher<nao_command_msgs::msg::JointStiffnesses>::SharedPtr pub_joint_stiffness;
  rclcpp::Subscription<nao_sensor_msgs::msg::JointPositions>::SharedPtr sub_joint_states;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_is_motion_done;

  //rclcpp::Publisher<nao_command_msgs::msg::JointPositions>::SharedPtr pub_joint_states;
  rclcpp::TimerBase::SharedPtr timer_;

  //walk_msg::msg::Walk walk_command;
  // bool started_walk=false;
  std::string curr_motion;
  std::string motion_speed;
  std::string direction;
  bool started_moiton = false;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyframemotionNode>());
  rclcpp::shutdown();
  return 0;
}
