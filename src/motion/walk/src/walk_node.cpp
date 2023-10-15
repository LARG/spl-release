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
#include "rclcpp/rclcpp.hpp"
#include "walk/walk.hpp"
#include "walk/BodyModel.hpp"
#include "walk/JointValues.hpp"
#include "walk/Sensors.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nao_sensor_msgs/msg/joint_positions.hpp"
#include "nao_sensor_msgs/msg/gyroscope.hpp"
#include "nao_sensor_msgs/msg/angle.hpp"
#include "nao_sensor_msgs/msg/fsr.hpp"
#include "nao_command_msgs/msg/joint_positions.hpp"
#include "nao_command_msgs/msg/joint_stiffnesses.hpp"
#include "nao_command_msgs/msg/joint_indexes.hpp"
#include "walk_msg/msg/walk.hpp"

using std::placeholders::_1;
using nao_sensor_msgs::msg::JointPositions;
using nao_sensor_msgs::msg::Gyroscope;
using nao_sensor_msgs::msg::Angle;
using nao_sensor_msgs::msg::FSR;
using WalkMsg = walk_msg::msg::Walk;
using CommandPositions = nao_command_msgs::msg::JointPositions;
using nao_command_msgs::msg::JointStiffnesses;
using geometry_msgs::msg::Twist;

void make_joint_msgs(const JointValues &joints,
    CommandPositions &joint_angles,
    JointStiffnesses &joint_stiffness) {
  joint_angles.indexes.clear();
  joint_angles.positions.clear();
  joint_stiffness.indexes.clear();
  joint_stiffness.stiffnesses.clear();
  for (int i = Joints::LShoulderPitch; i < Joints::RWristYaw; i++) {
    joint_stiffness.indexes.push_back(i);
    joint_stiffness.stiffnesses.push_back(joints.stiffnesses[i]);
  }

  for (int i = Joints::LShoulderPitch; i <= Joints::RWristYaw; i++) {
    joint_angles.indexes.push_back(i);
    joint_angles.positions.push_back(joints.angles[i]);
  }
}

class WalkNode : public rclcpp::Node {
 public:
    WalkNode() : Node("WalkNode"), walk(this), sensor_values(true) {
      RCLCPP_INFO(this->get_logger(), "Hello from walknode\n");
      sub_joint_states =
        create_subscription<JointPositions>(
            "sensors/joint_positions", 1,
            [this](JointPositions::SharedPtr sensor_joints) {
              sensor_values.populate_joint_positions(sensor_joints);
              if (this->started_walk) {
                JointValues j = walk.notifyJoints(action_command,
                    sensor_values, bodyModel);
                make_joint_msgs(j, joint_angles, joint_stiffness);
                pub_joint_angles->publish(joint_angles);
                pub_joint_stiffness->publish(joint_stiffness);
                pub_odom->publish(walk.get_odom());
              } else {
                pub_odom->publish(Twist());
              }
            });


      sub_gyroscope = create_subscription<Gyroscope>(
          "sensors/gyroscope", 1,
          [this](Gyroscope::SharedPtr gyr) {
            sensor_values.populate_gyro(gyr);
          });

      sub_angle = create_subscription<Angle>(
          "sensors/angle", 1,
          [this](Angle::SharedPtr angle) {
            sensor_values.populate_angles(angle);
          });

      sub_fsr = create_subscription<FSR>(
          "sensors/fsr", 1,
          [this](FSR::SharedPtr fsr) {
            sensor_values.populate_fsr(fsr);
          });

      sub_walk_start = create_subscription<WalkMsg>(
            "motion/walk", 10,
            [this](WalkMsg::SharedPtr walk_command) {
              RCLCPP_DEBUG(this->get_logger(), "Recieved walk command\n");
              action_command.make_from_walk_command(walk_command);
              if (action_command.enabled == true) {
                if (this->started_walk == false) {
                  this->started_walk = true;
                  walk.start();
                }
              }
              else {
                this->started_walk = false;
              }

            });

      pub_joint_angles = create_publisher<CommandPositions>
        ("effectors/joint_positions", 10);
      pub_joint_stiffness = create_publisher<JointStiffnesses>
        ("effectors/joint_stiffnesses", 10);
      pub_odom = create_publisher<Twist>("motion/walk_odom", 10);
    }

 private:
    Walk walk;
    ActionCommand action_command;
    SensorValues sensor_values;
    BodyModel bodyModel;

    CommandPositions joint_angles;
    JointStiffnesses joint_stiffness;


    rclcpp::Subscription<JointPositions>::SharedPtr sub_joint_states;
    rclcpp::Subscription<Gyroscope>::SharedPtr sub_gyroscope;
    rclcpp::Subscription<Angle>::SharedPtr sub_angle;
    rclcpp::Subscription<FSR>::SharedPtr sub_fsr;
    rclcpp::Subscription<WalkMsg>::SharedPtr sub_walk_start;

    rclcpp::Publisher<CommandPositions>::SharedPtr pub_joint_angles;
    rclcpp::Publisher<JointStiffnesses>::SharedPtr pub_joint_stiffness;
    rclcpp::Publisher<Twist>::SharedPtr pub_odom;

    bool started_walk = false;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WalkNode>());
  rclcpp::shutdown();
  return 0;
}
