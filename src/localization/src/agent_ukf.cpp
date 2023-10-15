#include <math.h>
#include "angles/angles.h"
#include "rclcpp/rclcpp.hpp"
#include "localization/agent_ukf.hpp"
#include "localization/measurement.h"
#include "localization/ukf.hpp"

using std::placeholders::_1;

AgentUkf::AgentUkf():Ukf(AGENT_STATE_SIZE) {}

AgentUkf::~AgentUkf() {}

void AgentUkf::set_process_noise_covariance() {
  process_noise_covariance_.setZero();
  process_noise_covariance_(StateMemberX, StateMemberX) = 500;
  process_noise_covariance_(StateMemberY, StateMemberY) = 500;
  process_noise_covariance_(StateMemberZ, StateMemberZ) = 500;
  process_noise_covariance_(StateMemberRoll, StateMemberRoll) = 0.03;
  process_noise_covariance_(StateMemberPitch, StateMemberPitch) = 0.03;
  process_noise_covariance_(StateMemberYaw, StateMemberYaw) = 0.06;
  process_noise_covariance_(StateMemberVx, StateMemberVx) = 250;
  process_noise_covariance_(StateMemberVy, StateMemberVy) = 250;
  process_noise_covariance_(StateMemberVz, StateMemberVz) = 250;
  process_noise_covariance_(StateMemberVroll, StateMemberVroll) = 0.01;
  process_noise_covariance_(StateMemberVpitch, StateMemberVpitch) = 0.01;
  process_noise_covariance_(StateMemberVyaw, StateMemberVyaw) = 0.02;
  process_noise_covariance_(StateMemberAx, StateMemberAx) = 100;
  process_noise_covariance_(StateMemberAy, StateMemberAy) = 100;
  process_noise_covariance_(StateMemberAz, StateMemberAz) = 100;
}

void AgentUkf::set_transfer_function(const double delta_sec) {
  double roll = state_(StateMemberRoll);
  double pitch = state_(StateMemberPitch);
  double yaw = state_(StateMemberYaw);

  // We'll need these trig calculations a lot.
  double sp = ::sin(pitch);
  double cp = ::cos(pitch);
  double cpi = 1.0 / cp;
  double tp = sp * cpi;

  double sr = ::sin(roll);
  double cr = ::cos(roll);

  double sy = ::sin(yaw);
  double cy = ::cos(yaw);

  transfer_function_(StateMemberX, StateMemberVx) = cy * cp * delta_sec;
  transfer_function_(StateMemberX, StateMemberVy) =
    (cy * sp * sr - sy * cr) * delta_sec;
  transfer_function_(StateMemberX, StateMemberVz) =
    (cy * sp * cr + sy * sr) * delta_sec;
  transfer_function_(StateMemberX, StateMemberAx) =
    0.5 * transfer_function_(StateMemberX, StateMemberVx) * delta_sec;
  transfer_function_(StateMemberX, StateMemberAy) =
    0.5 * transfer_function_(StateMemberX, StateMemberVy) * delta_sec;
  transfer_function_(StateMemberX, StateMemberAz) =
    0.5 * transfer_function_(StateMemberX, StateMemberVz) * delta_sec;

  transfer_function_(StateMemberY, StateMemberVx) = sy * cp * delta_sec;
  transfer_function_(StateMemberY, StateMemberVy) =
    (sy * sp * sr + cy * cr) * delta_sec;
  transfer_function_(StateMemberY, StateMemberVz) =
    (sy * sp * cr - cy * sr) * delta_sec;
  transfer_function_(StateMemberY, StateMemberAx) =
    0.5 * transfer_function_(StateMemberY, StateMemberVx) * delta_sec;
  transfer_function_(StateMemberY, StateMemberAy) =
    0.5 * transfer_function_(StateMemberY, StateMemberVy) * delta_sec;
  transfer_function_(StateMemberY, StateMemberAz) =
    0.5 * transfer_function_(StateMemberY, StateMemberVz) * delta_sec;

  transfer_function_(StateMemberZ, StateMemberVx) = -sp * delta_sec;
  transfer_function_(StateMemberZ, StateMemberVy) = cp * sr * delta_sec;
  transfer_function_(StateMemberZ, StateMemberVz) = cp * cr * delta_sec;
  transfer_function_(StateMemberZ, StateMemberAx) =
    0.5 * transfer_function_(StateMemberZ, StateMemberVx) * delta_sec;
  transfer_function_(StateMemberZ, StateMemberAy) =
    0.5 * transfer_function_(StateMemberZ, StateMemberVy) * delta_sec;
  transfer_function_(StateMemberZ, StateMemberAz) =
    0.5 * transfer_function_(StateMemberZ, StateMemberVz) * delta_sec;

  transfer_function_(StateMemberRoll, StateMemberVroll) = delta_sec;
  transfer_function_(StateMemberRoll, StateMemberVpitch) = sr * tp * delta_sec;
  transfer_function_(StateMemberRoll, StateMemberVyaw) = cr * tp * delta_sec;

  transfer_function_(StateMemberPitch, StateMemberVpitch) = cr * delta_sec;
  transfer_function_(StateMemberPitch, StateMemberVyaw) = -sr * delta_sec;

  transfer_function_(StateMemberYaw, StateMemberVpitch) = sr * cpi * delta_sec;
  transfer_function_(StateMemberYaw, StateMemberVyaw) = cr * cpi * delta_sec;

  transfer_function_(StateMemberVx, StateMemberAx) = delta_sec;

  transfer_function_(StateMemberVy, StateMemberAy) = delta_sec;

  transfer_function_(StateMemberVz, StateMemberAz) = delta_sec;
}

void AgentUkf::clampValues(VectorXd *state,
    std::vector<size_t> update_indices) {
  for (size_t i = 0; i < update_indices.size(); i++) {
    if (update_indices[i] == StateMemberRoll ||
        update_indices[i] == StateMemberPitch ||
        update_indices[i] == StateMemberYaw)
      (*state)(i) = clampRotation((*state)(i));
  }
}

void AgentUkf::clampStateValues() {
  state_(StateMemberRoll) =
    clampRotation(state_(StateMemberRoll));
  state_(StateMemberPitch) =
    clampRotation(state_(StateMemberPitch));
  state_(StateMemberYaw) =
    clampRotation(state_(StateMemberYaw));
}

double AgentUkf::clampRotation(const double rotation) const {
  return std::remainder(rotation, 2*M_PI);
}
