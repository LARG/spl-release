/*
Unscented Kalman Filter based on ros robot localization implementation
customized for SPL
*/


#include "localization/ukf.hpp"
#include <Eigen/Cholesky>
#include <Eigen/Dense>
#include <angles/angles.h>
#include <chrono>
#include <ctime>
#include <vector>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "localization/measurement.h"

using std::placeholders::_1;

Ukf::Ukf(int size) :
  state_size(size),
  state_(size),
  transfer_function_(size, size),
  process_noise_covariance_(size, size),
  estimate_error_covariance_(size, size) {
  uncorrected_ = false;
  size_t sigma_count = (size << 1) + 1;
  sigma_points_.resize(sigma_count, VectorXd(size));
  state_weights_.resize(sigma_count);
  covar_weights_.resize(sigma_count);
  state_.setZero();
  estimate_error_covariance_.setIdentity();
}

Ukf::~Ukf() {}

void Ukf::initialize(double alpha, double kappa, double beta) {
  // Prepare initial sigma point weights
  size_t sigma_count = (state_size << 1) + 1;
  lambda_ = alpha * alpha * (state_size + kappa) - state_size;

  state_weights_[0] = lambda_ / (state_size + lambda_);
  covar_weights_[0] = state_weights_[0] + (1 - (alpha * alpha) + beta);
  sigma_points_[0].setZero();
  for (size_t i = 1; i < sigma_count; ++i) {
    sigma_points_[i].setZero();
    state_weights_[i] = 1 / (2 * (state_size + lambda_));
    covar_weights_[i] = state_weights_[i];
  }

  // Clear the state and predicted state
  state_.setZero();

  // Prepare the invariant parts of the transfer
  // function
  transfer_function_.setIdentity();

  // Set the estimate error covariance. We want our measurements
  // to be accepted rapidly when the filter starts, so we should
  // initialize the state's covariance with large values.
  estimate_error_covariance_.setIdentity();
  estimate_error_covariance_ *= 1e-6;

  // We need the identity for the update equations
  identity_.setIdentity();

  // These can be overridden via the launch parameters,
  // but we need default values.
  set_process_noise_covariance();
}

void Ukf::update(const Measurement &measurement) {
  if (!uncorrected_) {
    // Take the square root of a small fraction of the
    // estimate_error_covariance_ using LL' decomposition
    weighted_covar_sqrt_ =
      ((state_size + lambda_) * estimate_error_covariance_).llt().matrixL();

    // Compute sigma points.
    // First sigma point is the current state
    sigma_points_[0] = state_;
    // Next n sigma points are (state + weighted_covar_sqrt_[ith column])
    // followed by n that are (state - weighted_covar_sqrt_[ith column])
    // where n is state_size
    for (int sigma_ind = 0; sigma_ind < state_size; ++sigma_ind) {
      sigma_points_[sigma_ind + 1] =
        state_ + weighted_covar_sqrt_.col(sigma_ind);
      sigma_points_[sigma_ind + 1 + state_size] =
        state_ - weighted_covar_sqrt_.col(sigma_ind);
    }
  }

  std::vector<size_t> update_indices;
  for (size_t i = 0; i < measurement.update_vector_.size(); ++i) {
    if (measurement.update_vector_[i]) {
      // Exclude nan and inf values in measurements
      if (!std::isnan(measurement.measurement_(i)) &&
          !std::isinf(measurement.measurement_(i))) {
        update_indices.push_back(i);
      }
    }
  }

  size_t updateSize = update_indices.size();

  // Now set up the relevant matrices
  Eigen::VectorXd state_subset(updateSize);                    // x
  Eigen::VectorXd measurement_subset(updateSize);              // z
  Eigen::MatrixXd measurement_covariance_subset(
      updateSize, updateSize);                                 // P
  Eigen::MatrixXd state_to_measurement_subset(
      updateSize, state_size);                                 // H
  Eigen::MatrixXd kalman_gain_subset(state_size, updateSize);  // K
  Eigen::VectorXd innovation_subset(updateSize);               // z - Hx
  Eigen::VectorXd predicted_measurement(updateSize);
  Eigen::VectorXd sigma_diff(updateSize);
  Eigen::MatrixXd predicted_meas_covar(updateSize, updateSize);
  Eigen::MatrixXd cross_covar(state_size, updateSize);

  std::vector<Eigen::VectorXd> sigma_point_measurements(
    sigma_points_.size(), Eigen::VectorXd(updateSize));

  state_subset.setZero();
  measurement_subset.setZero();
  measurement_covariance_subset.setZero();
  state_to_measurement_subset.setZero();
  kalman_gain_subset.setZero();
  innovation_subset.setZero();
  predicted_measurement.setZero();
  predicted_meas_covar.setZero();
  cross_covar.setZero();

  // Now build the sub-matrices from the full-sized matrices
  for (size_t i = 0; i < updateSize; ++i) {
    measurement_subset(i) = measurement.measurement_(update_indices[i]);
    state_subset(i) = state_(update_indices[i]);

    for (size_t j = 0; j < updateSize; ++j) {
      measurement_covariance_subset(i, j) =
        measurement.covariance_(update_indices[i], update_indices[j]);
    }

    // Handle negative (read: bad) covariances in the measurement. Rather
    // than exclude the measurement or make up a covariance, just take
    // the absolute value.
    if (measurement_covariance_subset(i, i) < 0) {
      measurement_covariance_subset(i, i) =
        ::fabs(measurement_covariance_subset(i, i));
    }
    // If the measurement variance for a given variable is very
    // near 0 (as in e-50 or so) and the variance for that
    // variable in the covariance matrix is also near zero, then
    // the Kalman gain computation will blow up. Really, no
    // measurement can be completely without error, so add a small
    // amount in that case.
    if (measurement_covariance_subset(i, i) < 1e-9) {
      measurement_covariance_subset(i, i) = 1e-9;
    }
  }

  // The state-to-measurement function, h, will now be a measurement_size x
  // full_state_size matrix, with ones in the (i, i) locations of the values to
  // be updated
  for (size_t i = 0; i < updateSize; ++i) {
    state_to_measurement_subset(i, update_indices[i]) = 1;
  }

  // Compute the measurement sigma points  (Z_i_t = h(Y_i_t))
  for (size_t sigma_ind = 0; sigma_ind < sigma_points_.size(); ++sigma_ind) {
    sigma_point_measurements[sigma_ind] =
      state_to_measurement_subset * sigma_points_[sigma_ind];
    predicted_measurement.noalias() +=
      state_weights_[sigma_ind] * sigma_point_measurements[sigma_ind];
  }

  // Compute the the measurement covariance matrix (P_zz) and a
  // state/measurement cross-covariance matrix (P_xz).
  for (size_t sigma_ind = 0; sigma_ind < sigma_points_.size(); ++sigma_ind) {
    sigma_diff = sigma_point_measurements[sigma_ind] - predicted_measurement;
    predicted_meas_covar.noalias() +=
      covar_weights_[sigma_ind] * (sigma_diff * sigma_diff.transpose());
    cross_covar.noalias() +=
      covar_weights_[sigma_ind] *
      ((sigma_points_[sigma_ind] - state_) *
       sigma_diff.transpose());
  }

  // Compute the Kalman gain, making sure to use the actual measurement
  // covariance: K_t = P_xz * (P_zz + R)^-1
  MatrixXd inv_innov_cov = (predicted_meas_covar +
                            measurement_covariance_subset).inverse();
  kalman_gain_subset = cross_covar * inv_innov_cov;

  // Compute innovation: z - z_hat
  innovation_subset = (measurement_subset - predicted_measurement);

  // Check Mahalanobis distance of innovation
  if (checkMahalanobisThreshold(innovation_subset, inv_innov_cov,
                                measurement.mahalanobis_thresh_)) {
    // Clamp/wrap values if needed
    clampValues(&innovation_subset, update_indices);
    // Update state: x = x + K * (z - z_hat)
    state_.noalias() += kalman_gain_subset * innovation_subset;
    // Update covariance: P = P - (K * P_zz * K')
    estimate_error_covariance_.noalias() -=
      kalman_gain_subset * predicted_meas_covar *
      kalman_gain_subset.transpose();
    // Clamp/wrap state values if needed
    clampStateValues();
    // Mark that we need to re-compute sigma points for successive corrections
    uncorrected_ = false;
  }
}

void Ukf::predict(double delta_sec) {
  set_transfer_function(delta_sec);

  // Take the square root of a small fraction of the
  // estimate_error_covariance_ using LL' decomposition
  weighted_covar_sqrt_ =
    ((state_size + lambda_) * estimate_error_covariance_).llt().matrixL();

  // Compute sigma points *and* pass them through the transfer function to
  // save the extra loop

  // First sigma point is the current state
  sigma_points_[0] = transfer_function_ * state_;
  // Next n sigma points are state + weighted_covar_sqrt_[ith column]
  // and following n are state - weighted_covar_sqrt_[ith column]
  // where n is state size
  for (int sigma_ind = 0; sigma_ind < state_size; ++sigma_ind) {
    sigma_points_[sigma_ind + 1] = transfer_function_ *
      (state_ + weighted_covar_sqrt_.col(sigma_ind));
    sigma_points_[sigma_ind + 1 + state_size] = transfer_function_ *
      (state_ - weighted_covar_sqrt_.col(sigma_ind));
  }

  // Sum the weighted sigma points to generate a new state prediction
  state_.setZero();
  for (size_t sigma_ind = 0; sigma_ind < sigma_points_.size(); ++sigma_ind) {
    state_.noalias() += state_weights_[sigma_ind] * sigma_points_[sigma_ind];
  }

  // Now use the sigma points and the predicted state to compute a predicted
  // covariance
  estimate_error_covariance_.setZero();
  VectorXd sigma_diff(state_size);
  for (size_t sigma_ind = 0; sigma_ind < sigma_points_.size(); ++sigma_ind) {
    sigma_diff = (sigma_points_[sigma_ind] - state_);
    estimate_error_covariance_.noalias() +=
      covar_weights_[sigma_ind] * (sigma_diff * sigma_diff.transpose());
  }

  // Not strictly in the theoretical UKF formulation, but necessary here
  // to ensure that we actually incorporate the process_noise_covariance_
  estimate_error_covariance_.noalias() +=
    delta_sec * process_noise_covariance_;

  // Mark that we can keep these sigma points
  uncorrected_ = true;

  // Clamp/wrap state values if needed
  clampStateValues();
}


bool Ukf::checkMahalanobisThreshold(
  const Eigen::VectorXd & innovation,
  const Eigen::MatrixXd & innovation_covariance, const double n_sigmas) {
  double squared_mahalanobis =
    innovation.dot(innovation_covariance * innovation);
  double threshold = n_sigmas * n_sigmas;

  if (squared_mahalanobis >= threshold) {
    return false;
  }

  return true;
}
