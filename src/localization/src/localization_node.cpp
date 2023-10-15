#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nao_sensor_msgs/msg/joint_positions.hpp"
#include "nao_sensor_msgs/msg/angle.hpp"
#include "nao_sensor_msgs/msg/fsr.hpp"
#include "common/msg/image_object.hpp"
#include "common/msg/image_object_list.hpp"
#include "common/msg/image_line.hpp"
#include "common/msg/image_line_list.hpp"
#include "common/msg/image_object_classes.hpp"
#include "localization/CameraMatrix.h"
#include "localization/ukf.hpp"
#include "localization/agent_ukf.hpp"
#include "localization/ball_ukf.hpp"
#include "localization/measurement.h"
#include "localization/ForwardKinematics.h"
#include "localization/RobotDimensions.h"
#include "localization/line_matching.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"

#define CALIBRATION_FRAMES 20

using Eigen::Vector3d;
using std::placeholders::_1;
using sensor_msgs::msg::Imu;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;
using nao_sensor_msgs::msg::JointPositions;
using nao_sensor_msgs::msg::Angle;
using nao_sensor_msgs::msg::FSR;
using common::msg::ImageObject;
using common::msg::ImageObjectList;
using common::msg::ImageLine;
using common::msg::ImageLineList;
using common::msg::ImageObjectClasses;

class LocalizationNode : public rclcpp::Node {
 public:
    LocalizationNode(
        const double agent_ukf_alpha = 0.001,
        const double agent_ukf_kappa = 1,
        const double agent_ukf_beta = 2,
        const double ball_ukf_alpha = 0.001,
        const double ball_ukf_kappa = 1,
        const double ball_ukf_beta = 2) :
      Node("localization"),
      top_camera_matrix(true, 1280, 960),
      bottom_camera_matrix(false, 1280, 960),
      current_stance_leg_left(true),
      robot_is_flying(true),
      accel_x_bias(0),
      accel_y_bias(0),
      accel_z_bias(0) {
      // Initialize UKFs
      agent_ukf.initialize(agent_ukf_alpha, agent_ukf_kappa, agent_ukf_beta);
      ball_ukf.initialize(ball_ukf_alpha, ball_ukf_kappa, ball_ukf_beta);
      rel_ball_ukf.initialize(ball_ukf_alpha, ball_ukf_kappa, ball_ukf_beta);
      MatrixXd ball_initial_covariance =
        MatrixXd::Identity(BALL_STATE_SIZE, BALL_STATE_SIZE);
      ball_initial_covariance(BallUkf::StateMemberX,
                              BallUkf::StateMemberX) = 1000;
      ball_initial_covariance(BallUkf::StateMemberY,
                              BallUkf::StateMemberY) = 1000;
      ball_ukf.setCovariance(ball_initial_covariance);
      rel_ball_ukf.setCovariance(ball_initial_covariance);
      // Initialize parameters
      this->declare_parameter("publish_lines", false);
      // Create subscriptions
      filtered_angle_subscription_ = this->create_subscription<Angle>(
        "sensors/angle", 10,
        std::bind(&LocalizationNode::filtered_angle_update, this, _1));
      imu_subscription_ = this->create_subscription<Imu>(
        "sensors/imu", 10,
        std::bind(&LocalizationNode::imu_update, this, _1));
      joint_subscription_ = this->create_subscription<JointPositions>(
        "sensors/joint_positions", 10,
        std::bind(&LocalizationNode::kinematics_update, this, _1));
      fsr_subscription_ = this->create_subscription<FSR>(
        "sensors/fsr", 10,
        std::bind(&LocalizationNode::fsr_update, this, _1));
      odom_subscription_ = this->create_subscription<Twist>(
          "motion/walk_odom", 10,
          std::bind(&LocalizationNode::walk_odom_update, this, _1));
      vision_objects_subscription_ = this->create_subscription<ImageObjectList>(
        "villa/vision/detected_objects", 10,
        std::bind(&LocalizationNode::vision_callback_objects, this, _1));
      vision_lines_subscription_ = this->create_subscription<ImageLineList>(
        "villa/vision/detected_lines", 10,
        std::bind(&LocalizationNode::vision_callback_lines, this, _1));
      spawn_subscription_ = this->create_subscription<Pose>(
        "localization/spawn", 10,
        std::bind(&LocalizationNode::spawn_callback, this, _1));
      // Initialize timers
      prev_predict_time = prev_ball_time = this->get_clock()->now();

      vision_invoke_publisher_ = this->create_publisher
        <std_msgs::msg::String>("/villa/vision/invoke", 10);
      invoke_timer_bottom_ = this->create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&LocalizationNode::invoke_vision_bottom_callback, this));
      invoke_timer_top_ = this->create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&LocalizationNode::invoke_vision_top_callback, this));

      pub_has_fallen_ = this->create_publisher<std_msgs::msg::Bool>("/has_fallen", 10);
      pub_last_ball_detection_timestamp_ = this->create_publisher<std_msgs::msg::Header>("/last_ball_detection_timestamp", 10);

      agent_pose_publisher_ = this->create_publisher<Odometry>
        ("/localization/agent_pose", 10);
      ball_rel_pose_publisher_ = this->create_publisher<Odometry>
        ("/localization/ball_rel_pose", 10);
      ball_pose_publisher_ = this->create_publisher<Odometry>
        ("/localization/ball_pose", 10);
      agent_pose_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&LocalizationNode::agent_pose_callback, this));
      ball_pose_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&LocalizationNode::ball_pose_callback, this));

      // Initialize visualization
      accepted_matched_lines_publisher_ = this->create_publisher
        <visualization_msgs::msg::Marker>
        ("/localization/accepted_matched_lines", 10);

      rejected_matched_lines_publisher_ = this->create_publisher
        <visualization_msgs::msg::Marker>
        ("/localization/rejected_matched_lines", 10);

      calibration_data = std::vector<VectorXd>();
      calibrated = false;

      first_ball_recv = true;
    }

 private:
    CameraMatrix top_camera_matrix;
    CameraMatrix bottom_camera_matrix;
    BallUkf ball_ukf;
    BallUkf rel_ball_ukf;
    AgentUkf agent_ukf;
    std::mutex ball_mutex;
    std::mutex agent_mutex;
    RobotDimensions robot_dimensions;

    bool current_stance_leg_left;
    bool robot_is_flying;
    bool first_ball_recv;

    float accel_x_bias;
    float accel_y_bias;
    float accel_z_bias;

    // Initialization
    std::vector<VectorXd> calibration_data;
    bool calibrated;

    // Sensor timestamps
    rclcpp::Time prev_predict_time;
    rclcpp::Time curr_predict_time;

    // Object timestamps
    rclcpp::Time prev_ball_time;
    rclcpp::Time curr_ball_time;

    rclcpp::Subscription<Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<JointPositions>::SharedPtr joint_subscription_;
    rclcpp::Subscription<FSR>::SharedPtr fsr_subscription_;
    rclcpp::Subscription<Twist>::SharedPtr odom_subscription_;
    rclcpp::Subscription<Angle>::SharedPtr filtered_angle_subscription_;
    rclcpp::Subscription<ImageObjectList>::SharedPtr
      vision_objects_subscription_;
    rclcpp::Subscription<ImageLineList>::SharedPtr
      vision_lines_subscription_;
    rclcpp::Subscription<Pose>::SharedPtr spawn_subscription_;

    rclcpp::TimerBase::SharedPtr invoke_timer_top_;
    rclcpp::TimerBase::SharedPtr invoke_timer_bottom_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      vision_invoke_publisher_;

    rclcpp::TimerBase::SharedPtr agent_pose_timer_;
    rclcpp::TimerBase::SharedPtr ball_pose_timer_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_has_fallen_;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr pub_last_ball_detection_timestamp_;

    std_msgs::msg::Bool has_fallen_;
    std_msgs::msg::Header last_ball_detection_timestamp_;
    rclcpp::Publisher<Odometry>::SharedPtr agent_pose_publisher_;
    rclcpp::Publisher<Odometry>::SharedPtr ball_rel_pose_publisher_;
    rclcpp::Publisher<Odometry>::SharedPtr ball_pose_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      accepted_matched_lines_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      rejected_matched_lines_publisher_;

    void predict_agent() {
      std::scoped_lock lock{agent_mutex};
      curr_predict_time = this->get_clock()->now();
      if ((curr_predict_time - prev_predict_time).seconds() < 0.001)
        return;  // No need to run predict too frequently
      agent_ukf.predict((curr_predict_time - prev_predict_time).seconds());
      prev_predict_time = curr_predict_time;
    }

    void predict_ball() {
      std::scoped_lock lock{ball_mutex};
      curr_ball_time = this->get_clock()->now();
      if ((curr_ball_time - prev_ball_time).seconds() < 0.001)
        return;  // No need to run predict too frequently
      ball_ukf.predict((curr_ball_time - prev_ball_time).seconds());
      rel_ball_ukf.predict((curr_ball_time - prev_ball_time).seconds());
      prev_ball_time = curr_ball_time;
    }

    void invoke_vision_bottom_callback() {
      std_msgs::msg::String msgbot;
      msgbot.data = "[{ \"name\" : \"nao_camera_bottom\"} , "
        "{\"name\" : \"ball_detector_bottom\"}]"
        "{\"name\" : \"cross_detector_top\"},";
      vision_invoke_publisher_->publish(msgbot);
    }

    void invoke_vision_top_callback() {
      std_msgs::msg::String msgtop;
      msgtop.data = "[{ \"name\" : \"nao_camera_top\"} , "
        "{\"name\" : \"ball_detector_top\"},"
        "{\"name\" : \"color_segmenter_top\"},"
        "{\"name\" : \"line_detector_top\"}]";
      vision_invoke_publisher_->publish(msgtop);
    }

    void agent_pose_callback() {
      std::scoped_lock lock{agent_mutex};
      Eigen::VectorXd state = agent_ukf.getState();
      Eigen::MatrixXd covariance = agent_ukf.getCovariance();
      Odometry agent_pose_msg;
      // Fill header
      agent_pose_msg.header.frame_id = "map";
      agent_pose_msg.header.stamp = this->get_clock()->now();
      agent_pose_msg.child_frame_id = "torso";
      // Fill position
      agent_pose_msg.pose.pose.position.x = state(AgentUkf::StateMemberX);
      agent_pose_msg.pose.pose.position.y = state(AgentUkf::StateMemberY);
      agent_pose_msg.pose.pose.position.z = state(AgentUkf::StateMemberZ);
      Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity() *
        Eigen::AngleAxisd(agent_ukf.getState()(AgentUkf::StateMemberYaw),
        Vector3d::UnitZ()) *
        Eigen::AngleAxisd(agent_ukf.getState()(AgentUkf::StateMemberPitch),
        Vector3d::UnitY()) *
        Eigen::AngleAxisd(agent_ukf.getState()(AgentUkf::StateMemberRoll),
        Vector3d::UnitX());
      agent_pose_msg.pose.pose.orientation.x = orientation.x();
      agent_pose_msg.pose.pose.orientation.y = orientation.y();
      agent_pose_msg.pose.pose.orientation.z = orientation.z();
      agent_pose_msg.pose.pose.orientation.w = orientation.w();
      std::array<int, 6> indexes = {
        AgentUkf::StateMemberX, AgentUkf::StateMemberY, AgentUkf::StateMemberZ,
        AgentUkf::StateMemberRoll, AgentUkf::StateMemberPitch,
        AgentUkf::StateMemberYaw};
      for (int i : indexes) {
        for (int j : indexes) {
          agent_pose_msg.pose.covariance[i * indexes.size() + j] =
            covariance(i, j);
        }
      }
      agent_pose_publisher_->publish(agent_pose_msg);
    }

    void ball_pose_callback() {
      predict_ball();
      std::scoped_lock lock{ball_mutex, agent_mutex};
      Eigen::VectorXd state = ball_ukf.getState();
      Eigen::MatrixXd covariance = ball_ukf.getCovariance();
      Eigen::VectorXd rel_state = rel_ball_ukf.getState();
      Eigen::MatrixXd rel_covariance = rel_ball_ukf.getCovariance();
      Odometry ball_pose_msg;
      Odometry rel_ball_pose_msg;
      // Fill headers
      ball_pose_msg.header.frame_id = "map";
      ball_pose_msg.header.stamp = this->get_clock()->now();
      ball_pose_msg.child_frame_id = "ball";
      rel_ball_pose_msg.header.frame_id = "map";
      rel_ball_pose_msg.header.stamp = this->get_clock()->now();
      ball_pose_msg.child_frame_id = "ball";
      // Fill position
      ball_pose_msg.pose.pose.position.x = state(BallUkf::StateMemberX);
      ball_pose_msg.pose.pose.position.y = state(BallUkf::StateMemberY);
      ball_pose_msg.pose.pose.position.z = 50;
      ball_pose_msg.pose.pose.orientation.x = 0;
      ball_pose_msg.pose.pose.orientation.y = 0;
      ball_pose_msg.pose.pose.orientation.z = 0;
      ball_pose_msg.pose.pose.orientation.w = 1;
      Eigen::Vector2d rel_ball_vect{rel_state(BallUkf::StateMemberX),
                                    rel_state(BallUkf::StateMemberY)};
      Eigen::Rotation2Dd rotation{agent_ukf.getState()
                                  (AgentUkf::StateMemberYaw)};
      Eigen::Vector2d agent_pose{agent_ukf.getState()(AgentUkf::StateMemberX),
                                 agent_ukf.getState()(AgentUkf::StateMemberY)};
      Eigen::Vector2d abs_ball_vect = rotation * rel_ball_vect + agent_pose;
      rel_ball_pose_msg.pose.pose.position.x = abs_ball_vect(0);
      rel_ball_pose_msg.pose.pose.position.y = abs_ball_vect(1);
      rel_ball_pose_msg.pose.pose.position.z = 50;
      rel_ball_pose_msg.pose.pose.orientation.x = 0;
      rel_ball_pose_msg.pose.pose.orientation.y = 0;
      rel_ball_pose_msg.pose.pose.orientation.z = 0;
      rel_ball_pose_msg.pose.pose.orientation.w = 1;
      // Fill covariance
      memset(ball_pose_msg.pose.covariance.data(), 0,
        sizeof(double) * ball_pose_msg.pose.covariance.size());
      ball_pose_msg.pose.covariance[0] = covariance(BallUkf::StateMemberX,
        BallUkf::StateMemberX);
      ball_pose_msg.pose.covariance[1] = covariance(BallUkf::StateMemberX,
        BallUkf::StateMemberY);
      ball_pose_msg.pose.covariance[6] = covariance(BallUkf::StateMemberY,
        BallUkf::StateMemberX);
      ball_pose_msg.pose.covariance[7] = covariance(BallUkf::StateMemberY,
        BallUkf::StateMemberY);
      memset(rel_ball_pose_msg.pose.covariance.data(), 0,
        sizeof(double) * rel_ball_pose_msg.pose.covariance.size());
      rel_ball_pose_msg.pose.covariance[0] = rel_covariance(BallUkf::StateMemberX,
        BallUkf::StateMemberX);
      rel_ball_pose_msg.pose.covariance[1] = rel_covariance(BallUkf::StateMemberX,
        BallUkf::StateMemberY);
      rel_ball_pose_msg.pose.covariance[6] = rel_covariance(BallUkf::StateMemberY,
        BallUkf::StateMemberX);
      rel_ball_pose_msg.pose.covariance[7] = rel_covariance(BallUkf::StateMemberY,
        BallUkf::StateMemberY);
      // Publish
      ball_pose_publisher_->publish(ball_pose_msg);
      ball_rel_pose_publisher_->publish(rel_ball_pose_msg);
    }

    void imu_update(const Imu &imu_msg) {
      predict_agent();
      std::scoped_lock lock{agent_mutex};
      Measurement measurement;
      measurement.measurement_ = VectorXd::Zero(AGENT_STATE_SIZE);
      // Fill in accelerometer values (input is in m/s^2)
      measurement.measurement_(AgentUkf::StateMemberAx) =
        imu_msg.linear_acceleration.x  * 1000 - accel_x_bias;
      measurement.measurement_(AgentUkf::StateMemberAy) =
        imu_msg.linear_acceleration.y * 1000 - accel_y_bias;
      measurement.measurement_(AgentUkf::StateMemberAz) =
        imu_msg.linear_acceleration.z * 1000 - accel_z_bias;
      // Fill in gyroscope values
      measurement.measurement_(AgentUkf::StateMemberVroll) =
        imu_msg.angular_velocity.x;
      measurement.measurement_(AgentUkf::StateMemberVpitch) =
        imu_msg.angular_velocity.y;
      measurement.measurement_(AgentUkf::StateMemberVyaw) =
        imu_msg.angular_velocity.z;
      // Fill in covariances
      measurement.covariance_ = MatrixXd::Zero(AGENT_STATE_SIZE,
                                               AGENT_STATE_SIZE);
      measurement.covariance_(AgentUkf::StateMemberAx,
                              AgentUkf::StateMemberAx) =
      measurement.covariance_(AgentUkf::StateMemberAy,
                              AgentUkf::StateMemberAy) =
      measurement.covariance_(AgentUkf::StateMemberAz,
                              AgentUkf::StateMemberAz) = 1e3;
      measurement.covariance_(AgentUkf::StateMemberVroll,
                              AgentUkf::StateMemberVroll) =
      measurement.covariance_(AgentUkf::StateMemberVpitch,
                              AgentUkf::StateMemberVpitch) =
      measurement.covariance_(AgentUkf::StateMemberVyaw,
                              AgentUkf::StateMemberVyaw) = 1e-5;
      // Update corresponding values
      measurement.update_vector_ = std::vector<bool>(AGENT_STATE_SIZE, false);
      measurement.update_vector_[AgentUkf::StateMemberAx] =
        measurement.update_vector_[AgentUkf::StateMemberAy] =
        measurement.update_vector_[AgentUkf::StateMemberAz] =
        measurement.update_vector_[AgentUkf::StateMemberVroll] =
        measurement.update_vector_[AgentUkf::StateMemberVpitch] =
        measurement.update_vector_[AgentUkf::StateMemberVyaw] = true;

      if (calibration_data.size() < CALIBRATION_FRAMES) {
        VectorXd cal_data = VectorXd(6);
        cal_data << measurement.measurement_(AgentUkf::StateMemberAx),
                    measurement.measurement_(AgentUkf::StateMemberAy),
                    measurement.measurement_(AgentUkf::StateMemberAz),
                    measurement.measurement_(AgentUkf::StateMemberVroll),
                    measurement.measurement_(AgentUkf::StateMemberVpitch),
                    measurement.measurement_(AgentUkf::StateMemberVyaw);
        calibration_data.push_back(cal_data);
        return;
      } else if (!calibrated) {
        std::array<double, 6> means = {0, 0, 0, 0, 0, 0};
        std::array<double, 6> variances = {0, 0, 0, 0, 0, 0};
        for (auto cal_data : calibration_data) {
          for (int i = 0; i < 6; i++) {
            means[i] += cal_data[i];
          }
        }
        for (int i = 0; i < 6; i++) {
          means[i] /= CALIBRATION_FRAMES;
        }
        for (auto cal_data : calibration_data) {
          for (int i = 0; i < 6; i++) {
            variances[i] += pow(cal_data[i] - means[i], 2);
          }
        }
        for (int i = 0; i < 6; i++) {
          variances[i] /= CALIBRATION_FRAMES;
        }
        accel_x_bias = means[0];
        accel_y_bias = means[1];
        accel_z_bias = means[2];
        RCLCPP_DEBUG_STREAM(get_logger(), "\nIMU mean/variances:\n: " <<
            "Ax: " << means[0] << "\t" << variances[0] << "\n" <<
            "Ay: " << means[1] << "\t" << variances[1] << "\n" <<
            "Az: " << means[2] << "\t" << variances[2] << "\n" <<
            "Vroll: " << means[3] << "\t" << variances[3] << "\n" <<
            "Vpitch: " << means[4] << "\t" << variances[4] << "\n" <<
            "Vyaw: " << means[5] << "\t" << variances[5] << "\n");
        calibrated = true;
      }

      agent_ukf.update(measurement);
    }

    void kinematics_update(const JointPositions &msg) {
      predict_agent();
      std::scoped_lock lock{agent_mutex};
      // Create a vector with joint values
      std::vector<float> joint_values(msg.positions.begin(),
                                      msg.positions.end());
      // Get pose of the bot
      std::vector<Pose3D> rel_parts = ForwardKinematics::calculateRelativePose(
          joint_values, robot_dimensions.values_);
      if (!robot_is_flying) {
        Pose3D torsoPos = rel_parts[BodyPart::torso].relativeTo(
            current_stance_leg_left ?  rel_parts[BodyPart::left_sole] :
                                       rel_parts[BodyPart::right_sole]);
        Measurement measurement;
        measurement.measurement_ = VectorXd::Zero(AGENT_STATE_SIZE);
        measurement.measurement_(AgentUkf::StateMemberZ) =
          torsoPos.translation[2];
        measurement.covariance_ = MatrixXd::Zero(AGENT_STATE_SIZE,
                                                AGENT_STATE_SIZE);
        measurement.covariance_(AgentUkf::StateMemberZ,
                                AgentUkf::StateMemberZ) = 100;
        measurement.update_vector_ = std::vector<bool>(AGENT_STATE_SIZE, false);
        measurement.update_vector_[AgentUkf::StateMemberZ] = true;
        agent_ukf.update(measurement);
      }

      // Update camera poses
      Eigen::VectorXd state = agent_ukf.getState();
      Pose3D torso_pos_from_ground {
        Vector3d{0, 0, state[AgentUkf::StateMemberZ]},
        Vector3d{state[AgentUkf::StateMemberRoll],
                 state[AgentUkf::StateMemberPitch], 0}
      };
      Pose3D origin = Pose3D().relativeTo(torso_pos_from_ground);
      top_camera_matrix.updateCameraPose(
          rel_parts[BodyPart::top_camera].relativeTo(origin));
      bottom_camera_matrix.updateCameraPose(
          rel_parts[BodyPart::bottom_camera].relativeTo(origin));
    }

    void fsr_update(const FSR &msg) {
      double total_force_left = msg.l_foot_front_left + msg.l_foot_front_right
                                + msg.l_foot_back_left + msg.l_foot_back_right;
      double total_force_right = msg.r_foot_front_left + msg.r_foot_front_right
                                 + msg.r_foot_back_left + msg.r_foot_back_right;
      if (total_force_left > total_force_right) {
        current_stance_leg_left = true;
      } else {
        current_stance_leg_left = false;
      }
      if (total_force_right + total_force_left < 1.3) {
        robot_is_flying = true;
      } else {
        robot_is_flying = false;
      }
    }

    void walk_odom_update(const Twist &msg) {
      predict_agent();
      std::scoped_lock lock{agent_mutex};
      Measurement measurement;
      measurement.measurement_ = VectorXd::Zero(AGENT_STATE_SIZE);
      measurement.measurement_(AgentUkf::StateMemberVx) = msg.linear.x;
      measurement.measurement_(AgentUkf::StateMemberVy) = msg.linear.y;
      measurement.measurement_(AgentUkf::StateMemberVyaw) = msg.angular.z;
      measurement.covariance_ = MatrixXd::Zero(AGENT_STATE_SIZE,
                                              AGENT_STATE_SIZE);
      measurement.covariance_(AgentUkf::StateMemberVx,
                              AgentUkf::StateMemberVx) =
      measurement.covariance_(AgentUkf::StateMemberVy,
                              AgentUkf::StateMemberVy) =
      measurement.covariance_(AgentUkf::StateMemberVyaw,
                              AgentUkf::StateMemberVyaw) = 100;
      measurement.update_vector_ = std::vector<bool>(AGENT_STATE_SIZE, false);
      measurement.update_vector_[AgentUkf::StateMemberVx] =
        measurement.update_vector_[AgentUkf::StateMemberVy] =
        measurement.update_vector_[AgentUkf::StateMemberVyaw] = true;
      agent_ukf.update(measurement);
    }

    void filtered_angle_update(const Angle &msg) {
      // RCLCPP_INFO(this->get_logger(), "Recived angle reading, x=%f, y = %f \n",msg.x, msg.y);

      if (msg.y >= 1.6 || msg.y <= -1.4){
        // RCLCPP_INFO(this->get_logger(), "NAO HAS FALLEN");
        this->has_fallen_.data = true;
        this->pub_has_fallen_->publish(has_fallen_);
      } else {
        this->has_fallen_.data = false;
        this->pub_has_fallen_->publish(has_fallen_);
      }

      predict_agent();
      std::scoped_lock lock{agent_mutex};
      // Update the orientation of the agent
      Measurement measurement;
      measurement.measurement_ = VectorXd::Zero(AGENT_STATE_SIZE);
      measurement.measurement_(AgentUkf::StateMemberRoll) = msg.x;
      measurement.measurement_(AgentUkf::StateMemberPitch) = msg.y;
      measurement.covariance_ = MatrixXd::Zero(AGENT_STATE_SIZE,
                                              AGENT_STATE_SIZE);
      measurement.covariance_(AgentUkf::StateMemberRoll,
                              AgentUkf::StateMemberRoll) =
      measurement.covariance_(AgentUkf::StateMemberPitch,
                              AgentUkf::StateMemberPitch) = 0.01;
      measurement.update_vector_ = std::vector<bool>(AGENT_STATE_SIZE, false);
      measurement.update_vector_[AgentUkf::StateMemberRoll] =
        measurement.update_vector_[AgentUkf::StateMemberPitch] = true;
      agent_ukf.update(measurement);


    }

    void vision_callback_objects(const ImageObjectList &detected_objects) {
      // Iterate through objects
      ImageObject ball;
      ball.confidence = 0.0;
      bool ball_found = false;
      for (auto object_itr = detected_objects.objects.begin();
           object_itr < detected_objects.objects.end(); ++object_itr) {
        // Find the ball detected with the most confidence
        if (object_itr->object_type == ImageObjectClasses::IO_BALL &&
            object_itr->confidence >= ball.confidence) {
          ball_found = true;
          ball = *(object_itr);
        }
      }

      if (ball_found) {

        this->last_ball_detection_timestamp_.stamp = this->get_clock()->now();
        this->pub_last_ball_detection_timestamp_->publish(this->last_ball_detection_timestamp_);

        // Have to lock since we're getting the camera matrix
        predict_ball();
        std::scoped_lock lock{agent_mutex, ball_mutex};
        // Transform ball from image to world coordinates
        // Check which camera it's from
        // Ideally want to read the names from vision config but hardcoding here
        Eigen::Vector3d ball_rel_pos;
        const float ball_radius = 50.0;
        if (ball.source == "ball_detector_top") {  // Top camera
          ball_rel_pos = top_camera_matrix.getWorldPosition(
            ball.center_x, ball.center_y, ball_radius);
        } else if (ball.source == "ball_detector_bottom") {  // Bottom camera
          ball_rel_pos = bottom_camera_matrix.getWorldPosition(
            ball.center_x, ball.center_y, ball_radius);
        } else {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
            "Unknown camera source: %s", ball.source.c_str());
          return;
        }

        // Estimate ball pose
        auto agent_state = agent_ukf.getState();
        Eigen::Vector2d agent_pos{agent_state[AgentUkf::StateMemberX],
                                  agent_state[AgentUkf::StateMemberY]};
        Eigen::Rotation2Dd rot(agent_state[AgentUkf::StateMemberYaw]);
        Eigen::Vector2d ball_position = agent_pos + rot * Eigen::Vector2d{
          ball_rel_pos[0], ball_rel_pos[1]};

        Measurement measurement;
        Measurement rel_measurement;

        measurement.measurement_ = VectorXd::Zero(BALL_STATE_SIZE);
        rel_measurement.measurement_ = VectorXd::Zero(BALL_STATE_SIZE);
        measurement.measurement_ << ball_position[0], ball_position[1], 0, 0;
        rel_measurement.measurement_ << ball_rel_pos[0], ball_rel_pos[1], 0, 0;

        measurement.covariance_ = MatrixXd::Zero(BALL_STATE_SIZE,
                                                 BALL_STATE_SIZE);
        rel_measurement.covariance_ = MatrixXd::Zero(BALL_STATE_SIZE,
                                                     BALL_STATE_SIZE);
        measurement.covariance_(BallUkf::StateMemberX,
                                BallUkf::StateMemberX) =
        measurement.covariance_(BallUkf::StateMemberY,
                                BallUkf::StateMemberY) = 100;
        rel_measurement.covariance_(BallUkf::StateMemberX,
                                    BallUkf::StateMemberX) =
        rel_measurement.covariance_(BallUkf::StateMemberY,
                                    BallUkf::StateMemberY) = 10;

        MatrixXd agent_cov = agent_ukf.getCovariance();
        MatrixXd agent_xy_cov(BALL_STATE_SIZE, BALL_STATE_SIZE);
        // TODO: agent velocities are in agent's refrence frame?
        // should velociy just be removed?
        std::array<float, BALL_STATE_SIZE> indexes = {
          AgentUkf::StateMemberX, AgentUkf::StateMemberY,
          AgentUkf::StateMemberVx, AgentUkf::StateMemberVy};
        for (int i = 0; i < BALL_STATE_SIZE; ++i) {
          for (int j = 0; j < BALL_STATE_SIZE; ++j) {
            agent_xy_cov(i, j) = agent_cov(indexes[i], indexes[j]);
          }
        }
        measurement.covariance_ += agent_xy_cov;

        measurement.update_vector_ = std::vector<bool>
          (BALL_STATE_SIZE, false);
        rel_measurement.update_vector_ = std::vector<bool>
          (BALL_STATE_SIZE, false);
        measurement.update_vector_[BallUkf::StateMemberX] =
          measurement.update_vector_[BallUkf::StateMemberY] =
          rel_measurement.update_vector_[BallUkf::StateMemberX] =
          rel_measurement.update_vector_[BallUkf::StateMemberY] = true;

        ball_ukf.update(measurement);
        rel_ball_ukf.update(rel_measurement);
      }
    }

    Line line_to_world_pos(const ImageLine &line,
                           const CameraMatrix &mat) const {
      WorldPosition p1_wp = mat.getWorldPosition(line.x1, line.y1);
      WorldPosition p2_wp = mat.getWorldPosition(line.x2, line.y2);
      Eigen::Vector2d p1_rel{p1_wp[0], p1_wp[1]};
      Eigen::Vector2d p2_rel{p2_wp[0], p2_wp[1]};
      Eigen::Rotation2Dd rot(agent_ukf.getState()(AgentUkf::StateMemberYaw));
      Eigen::Vector2d agent_pos{agent_ukf.getState()(AgentUkf::StateMemberX),
                                agent_ukf.getState()(AgentUkf::StateMemberY)};
      Eigen::Vector2d p1 = agent_pos + rot.toRotationMatrix() * p1_rel;
      Eigen::Vector2d p2 = agent_pos + rot.toRotationMatrix() * p2_rel;
      return Line(p1[0], p1[1], p2[0], p2[1]);
    }

    void vision_callback_lines(const ImageLineList &detected_lines) {
      predict_agent();
      std::scoped_lock lock{agent_mutex};
      // Iterate through lines
      std::vector<Line> lines_world_pos;
      for (ImageLine line : detected_lines.lines) {
        // Transform lines from image to world coordinates
        Line curr_line;
        // TODO: read source names from vision config
        if (line.source == "line_detector_top") {
          curr_line = line_to_world_pos(line, top_camera_matrix);
        } else {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
            "Unknown camera source: %s", line.source.c_str());
          return;
        }
        lines_world_pos.push_back(curr_line);
      }
      if (lines_world_pos.size() == 0) {
        return;
      }

      // Line matching
      std::vector<int> line_matches =
        line_matching::line_match(lines_world_pos);

      // Get correction/error vector (x, y, yaw)
      std::vector<bool> line_matches_accepted(line_matches.size(), false);
      Eigen::Vector3d correction_vec = line_matching::get_error_correction(
          line_matches, lines_world_pos, line_matches_accepted);

      // Send lines to rviz
      if (this->get_parameter("publish_lines").as_bool()) {
        visualization_msgs::msg::Marker accepted_marker;
        accepted_marker.header.frame_id = "map";
        accepted_marker.header.stamp = this->now();
        accepted_marker.ns = "accepted_lines";
        accepted_marker.id = 0;
        accepted_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        accepted_marker.scale.x = 50;
        accepted_marker.color.g = 1.0;
        accepted_marker.color.a = 1.0;

        visualization_msgs::msg::Marker rejected_marker;
        rejected_marker.header.frame_id = "map";
        rejected_marker.header.stamp = this->now();
        rejected_marker.ns = "rejected_lines";
        rejected_marker.id = 0;
        rejected_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        rejected_marker.scale.x = 50;
        rejected_marker.color.r = 1.0;
        rejected_marker.color.a = 1.0;

        for (size_t i = 0; i < line_matches.size(); i++) {
          if (line_matches_accepted[i]) {
            geometry_msgs::msg::Point p1;
            p1.x = lines_world_pos[i].a[0];
            p1.y = lines_world_pos[i].a[1];
            p1.z = 0;
            accepted_marker.points.push_back(p1);
            geometry_msgs::msg::Point p2;
            p2.x = lines_world_pos[i].b[0];
            p2.y = lines_world_pos[i].b[1];
            p2.z = 0;
            accepted_marker.points.push_back(p2);
          } else {
            geometry_msgs::msg::Point p1;
            p1.x = lines_world_pos[i].a[0];
            p1.y = lines_world_pos[i].a[1];
            p1.z = 0;
            rejected_marker.points.push_back(p1);
            geometry_msgs::msg::Point p2;
            p2.x = lines_world_pos[i].b[0];
            p2.y = lines_world_pos[i].b[1];
            p2.z = 0;
            rejected_marker.points.push_back(p2);
          }
        }
        accepted_matched_lines_publisher_->publish(accepted_marker);
        rejected_matched_lines_publisher_->publish(rejected_marker);
      }

      // Update Agent UKF
      if (correction_vec[0] == 0.0 &&
          correction_vec[1] == 0.0 &&
          correction_vec[2] == 0.0) {
        return;
      }
      Measurement measurement;
      measurement.measurement_ = agent_ukf.getState();
      measurement.measurement_(AgentUkf::StateMemberX) += correction_vec[0];
      measurement.measurement_(AgentUkf::StateMemberY) += correction_vec[1];
      measurement.measurement_(AgentUkf::StateMemberYaw) += -correction_vec[2];

      measurement.covariance_ = MatrixXd::Identity(AGENT_STATE_SIZE,
                                                   AGENT_STATE_SIZE);
      measurement.covariance_(AgentUkf::StateMemberX,
                              AgentUkf::StateMemberX) =
      measurement.covariance_(AgentUkf::StateMemberY,
                              AgentUkf::StateMemberY) = 10000;
      measurement.covariance_(AgentUkf::StateMemberYaw,
                              AgentUkf::StateMemberYaw) = 100;
      measurement.update_vector_ = std::vector<bool>(AGENT_STATE_SIZE, false);
      measurement.update_vector_[AgentUkf::StateMemberX] =
      measurement.update_vector_[AgentUkf::StateMemberY] =
      measurement.update_vector_[AgentUkf::StateMemberYaw] = true;
      agent_ukf.update(measurement);
    }

    void spawn_callback(const Pose &msg) {
      std::scoped_lock lock{agent_mutex};
      VectorXd spawn_state(AGENT_STATE_SIZE);
      spawn_state.setZero();
      spawn_state(AgentUkf::StateMemberX) = msg.position.x;
      spawn_state(AgentUkf::StateMemberY) = msg.position.y;
      spawn_state(AgentUkf::StateMemberZ) = msg.position.z;
      Eigen::Quaterniond orientation{msg.orientation.w, msg.orientation.x,
                                     msg.orientation.y, msg.orientation.z};
      Vector3d euler = orientation
        .toRotationMatrix()
        .eulerAngles(2, 1, 0);
      spawn_state(AgentUkf::StateMemberRoll) = euler[2];
      spawn_state(AgentUkf::StateMemberPitch) = euler[1];
      spawn_state(AgentUkf::StateMemberYaw) = euler[0];

      MatrixXd spawn_covariance(AGENT_STATE_SIZE, AGENT_STATE_SIZE);
      spawn_covariance.setIdentity();
      spawn_covariance *= 1e-9;
      spawn_covariance(AgentUkf::StateMemberX, AgentUkf::StateMemberX) = 100;
      spawn_covariance(AgentUkf::StateMemberY, AgentUkf::StateMemberY) = 100;
      spawn_covariance(AgentUkf::StateMemberYaw, AgentUkf::StateMemberYaw) = 1;

      agent_ukf.setState(spawn_state);
      agent_ukf.setCovariance(spawn_covariance);
    }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
