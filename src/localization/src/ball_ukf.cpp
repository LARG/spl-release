#include "localization/ball_ukf.hpp"
#include <vector>
#include "localization/measurement.h"
#include "localization/ukf.hpp"


using std::placeholders::_1;

BallUkf::BallUkf():Ukf(BALL_STATE_SIZE) {}


BallUkf::~BallUkf() {}

void BallUkf::set_process_noise_covariance() {
  process_noise_covariance_.setZero();
  process_noise_covariance_(StateMemberX, StateMemberX) = 50;
  process_noise_covariance_(StateMemberY, StateMemberY) = 50;
  process_noise_covariance_(StateMemberVx, StateMemberVx) = 2.5;
  process_noise_covariance_(StateMemberVy, StateMemberVy) = 2.5;
}

void BallUkf::set_transfer_function(const double delta_sec) {
  transfer_function_(StateMemberX, StateMemberVx) = delta_sec;
  transfer_function_(StateMemberY, StateMemberVy) = delta_sec;
  // friction
  transfer_function_(StateMemberVx, StateMemberVx) = -0.1 * delta_sec;
  transfer_function_(StateMemberVy, StateMemberVy) = -0.1 * delta_sec;
}
