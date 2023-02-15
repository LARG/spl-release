#include <localization/BallUKF.h>
#include <memory/TextLogMacros.h>

using BallMeas = BallUKF::BallMeas;
using Params = BallUKF::Params;
using Observation = BallUKF::Observation;

using namespace Eigen;

BallUKF::BallUKF(const Params& params) : UnscentedKalmanFilter(params), bmeas_(*this),
last_obs_seen_recently_(false) {
  state_ = StateVec::Zero();
  cov_ = StateCov::Identity();
  cov_.block<2,2>(BallX,BallX) = Matrix2f::Identity() * powf(1'000'000.0f,2.0f);
  cov_.block<2,2>(BallVelX,BallVelX) = Matrix2f::Identity() * powf(1.0f,2.0f);
  process_ = StateCov::Identity();
  process_.block<2,2>(BallX,BallX) = Matrix2f::Identity() * powf(90.0f,2.0f);
  process_.block<2,2>(BallVelX,BallVelX) = Matrix2f::Identity() * powf(50.0f,2.0f);
}

BallMeas::BallMeas(BaseUKF& ukf) : BaseMeas(ukf) {
}

BallMeas::MeasVec BallMeas::nonlinearPredictionTransform(const StateVec& state, const Observation& obs) {
  MeasVec meas(state[BallX], state[BallY]);
  return meas;
}

void BallMeas::update(const Observation& obs) {
  MeasVec meas(obs.relBall.x, obs.relBall.y);
  Matrix2f uncert = obs.relBallUncertainty;
  uncert *= max(powf(obs.relBall.getMagnitude() / 5, 2.0f), powf(200.0f, 2.0f));
  uncert = MatrixOperations::SanitizeCovariance(uncert);
  tlog(70, "Performing Ball UKF update with rel ball at %s, uncert: \n%s", obs.relBall, uncert);
  float innov = baseUpdate(meas, uncert, obs, true);
  tlog(70, "Ball measurement at %2.f,%2.f, Fmeas: %2.f,%2.f, Diff: %2.f,%2.f, adjust: %2.f,%2.f, Innov: %2.4f", meas[0], meas[1], fmeas_[0], fmeas_[1], diff_[0], diff_[1], adj_[BallX], adj_[BallY], innov);
}

BallUKF::StateVec BallUKF::nonlinearTimeTransform(const StateVec& v, const Observation& obs) {
  float timePassed = obs.timePassed;
  StateVec state = v;
  state[BallX] += v[BallVelX] * timePassed;
  state[BallY] += v[BallVelY] * timePassed;
  state[BallVelX] *= params_.BALL_DECAY_RATE;
  state[BallVelY] *= params_.BALL_DECAY_RATE;
  if(TextLogger::Enabled) {
    auto s0 = v.transpose();
    tlog(78, "Computing time transform on state: %s", s0);
    auto s1 = state.transpose();
    tlog(78, "Result: %s", s1);
  }
  return state;
}

void BallUKF::processOdometry(const Observation& obs) {
  tlog(70, "Processing relative odometry");
  Point2D ball(state_[BallX], state_[BallY]);
  Point2D vel(state_[BallVelX], state_[BallVelY]);
  Point2D disp(obs.odomX, obs.odomY);
  float theta = obs.odomTheta;
  if(obs.useGyro) {
    tlog(71, "Start:: pos: %s, vel: %s, disp: %s @ %2.2f <=  odom:%2.2f (%2.2f) + gyro:%2.2f (%2.2f)", 
      ball, vel, disp, theta, obs.odomTheta, params_.ODOM_THETA_WEIGHT, obs.gyroTheta, params_.GYRO_THETA_WEIGHT
    );
    theta = obs.odomTheta * params_.ODOM_THETA_WEIGHT + obs.gyroTheta * params_.GYRO_THETA_WEIGHT;
  } else {
    tlog(71, "Start:: pos: %s, vel: %s, disp: %s @ %2.2f [no gyro]", ball, vel, disp, theta);
  }
  ball -= disp;
  ball.rotate(-theta);
  vel.rotate(-theta);
  tlog(71, "Finish:: pos: %s, vel: %s", ball, vel);
  state_ << ball.x, ball.y, vel.x, vel.y;
}

void BallUKF::update(const Observation& obs) {
  tlog(71, "Updating ball model from observation:\n%s", obs);
  tlog(70, "Starting ball state: %s", *this);
  timeUpdate(obs);

  tlog(72, "After time update: %s", *this);
  processOdometry(obs);
  tlog(72, "After odom update: %s", *this);
  if(obs.hasBall) {
    if (not last_obs_seen_recently_){
      state_[BallX] = obs.relBall.x;
      state_[BallY] = obs.relBall.y;
      state_[BallVelX] = 0;
      state_[BallVelY] = 0;
    } else {
      bmeas_.update(obs);
    }
  } else if(!obs.ballSeenRecently) {
    tlog(72, "Ball hasn't been seen recently, bypassing standard update process");
    state_[BallX] = obs.relBall.x;
    state_[BallY] = obs.relBall.y;
    state_[BallVelX] = 0;
    state_[BallVelY] = 0;
  }
  last_obs_seen_recently_ = obs.ballSeenRecently;
  tlog(70, "Final ball state: %s", *this);
}

float BallUKF::distance(const BallUKF* other) const {
  auto diff = this->state() - other->state();
  return diff.norm();
}
  
std::ostream& operator<<(std::ostream& os, const BallObservation& obs) {
  os << "Ball Observation:\n";
  os << std::boolalpha;
  os.setf(std::ios_base::fixed, std::ios_base::floatfield);
  os.precision(2);
  os << VARLINE(obs.relBall) << ", ";
  os << VARLINE(obs.timePassed) << ", ";
  os << VARLINE(obs.odomX) << ", ";
  os << VARLINE(obs.odomY) << ", ";
  os << VARLINE(obs.odomTheta) << ", ";
  os << VARLINE(obs.gyroTheta) << ", ";
  os << VARLINE(obs.useGyro);
  return os;
}
    
std::ostream& operator<<(std::ostream& os, const BallUKF& ukf) {
  Point2D pos(ukf.state()[BallUKF::BallX], ukf.state()[BallUKF::BallY]);
  Point2D vel(ukf.state()[BallUKF::BallVelX], ukf.state()[BallUKF::BallVelY]);
  os << VARLINE(pos);
  os << ", ";
  os << VARLINE(vel);
  os << "\ncovariance:\n" << ukf.covariance() << "\n";
  return os;
}
