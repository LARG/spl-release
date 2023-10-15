#ifndef MEASUREMENT_HPP_
#define MEASUREMENT_HPP_

#include <Eigen/Dense>
#include <limits>
#include <vector>
#include <rclcpp/rclcpp.hpp>

struct Measurement {
  Measurement() :
    mahalanobis_thresh_(std::numeric_limits<double>::max()) {}

  // Threshold used to choose sigma points
  double mahalanobis_thresh_;
  // Vector for sensor measurement
  Eigen::VectorXd measurement_;
  // Measurement Covariance if available
  Eigen::MatrixXd covariance_;
  // Whether to update this value
  std::vector<bool> update_vector_;
};

#endif  // MEASUREMENT_HPP_
