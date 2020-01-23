#include <audio/WhistleUKF.h>
#include <memory/TextLogMacros.h>
#include <common/Profiling.h>
#include <common/WorldObject.h>
#include <math/Common.h>

#define SQ(x) ((x) * (x))

using WhistleMeas = WhistleUKF::WhistleMeas;
using TeammateMeas = WhistleUKF::TeammateMeas;
using Params = WhistleUKF::Params;
using Observation = WhistleUKF::Observation;

using namespace Eigen;

WhistleUKF::WhistleUKF(const Params& params) : UnscentedKalmanFilter(params), wmeas_(*this), tmeas_(*this) {
  state_ = StateVec::Zero();
  cov_ = StateCov::Identity() * 1.0f;
  process_ = StateCov::Identity() * 1.0f;
}

WhistleUKF::WhistleUKF(const WhistleUKF& other) : WhistleUKF(other.params_) {
  state_ = other.state_;
  cov_ = other.cov_;
  process_ = other.process_;
  tlogger_ = other.tlogger_;
}

TeammateMeas::TeammateMeas(BaseUKF& ukf) : BaseMeas(ukf) {
}

// We have to scale incoming scores/sds to account for 1) only getting
// packets every 5 frames or so and 2) float underflow since the scores
// are from 0 to 1
#define TEAMMATE_SCALE 100

TeammateMeas::MeasVec TeammateMeas::nonlinearPredictionTransform(const StateVec& state, const Observation& obs) {
  MeasVec v;
  float score = crop(state[WhistleScore], 0.0f, 1.0f);
  v << score, score, score, score;
  v *= TEAMMATE_SCALE;
  return v;
}

void TeammateMeas::update(const Observation& obs) {
  MeasVec meas(obs.teammateScores.data());
  meas *= TEAMMATE_SCALE;
  MeasCov uncert = MeasCov::Identity();
  for(int i = 0; i < uncert.rows() && i < uncert.cols(); i++)
    uncert(i,i) = std::max(SQ(obs.teammateSds[i] * TEAMMATE_SCALE / 10), 0.1f);
  uncert = MatrixOperations::SanitizeCovariance(uncert);
  float innov = baseUpdate(meas, uncert, obs, true);
  tlog(41, "Teammate Measurement: meas: %s, uncert:\n%s\n", meas.transpose(), uncert);
  tlog(41, "Update innovation: %2.4f", innov);
}

WhistleMeas::WhistleMeas(BaseUKF& ukf) : BaseMeas(ukf) {
}

WhistleMeas::MeasVec WhistleMeas::nonlinearPredictionTransform(const StateVec& state, const Observation& obs) {
  MeasVec v;
  v << state[0];
  return v;
}

void WhistleMeas::update(const Observation& obs) {
  MeasVec meas(obs.scores.data());
  MeasCov uncert = MeasCov::Identity() * 0.5f;
  uncert = MatrixOperations::SanitizeCovariance(uncert);
  float innov = baseUpdate(meas, uncert, obs, true);
}

WhistleUKF::StateVec WhistleUKF::nonlinearTimeTransform(const StateVec& v, const Observation& obs) {
  StateVec s = 0.95 * v;
  return s;
}

void WhistleUKF::update(const Observation& obs) {
  tlog(40, "Updating whistle model from observation:\n%s", obs);
  tlog(40, "Starting whistle state: %s", *this);
  timeUpdate(obs);

  if(obs.scores.size() > 0)
    wmeas_.update(obs);
  else if(obs.teammateScores.size() > 0 && obs.teammateScores.size() == obs.teammateSds.size())
    tmeas_.update(obs);
  sanitize();
  tlog(40, "Final whistle state: %s", *this);
}

float WhistleUKF::distance(const WhistleUKF* other) const {
  auto diff = this->state() - other->state();
  return diff.norm();
}

void WhistleUKF::sanitize() {
  for(int i = 0; i < state_.size(); i++)
    state_[i] = sanitizeState(state_[i]);
  cov_ = MatrixOperations::SanitizeCovariance(cov_);
}

std::ostream& operator<<(std::ostream& os, const WhistleObservation& obs) {
  os.setf(std::ios_base::fixed, std::ios_base::floatfield);
  os.precision(2);
  if(obs.scores.size() > 0) {
    os << "Whistle Observation| ";
    os << obs.scores;
  } else {
    os << "Teammate Observation| ";
    os << "Scores: " << obs.teammateScores << ", ";
    os << "SDs: " << obs.teammateSds;
  }
  return os;
}
    
std::ostream& operator<<(std::ostream& os, const WhistleUKF& ukf) {
  os.setf(std::ios_base::fixed, std::ios_base::floatfield);
  os.precision(2);
  os << "Whistle score: " << ukf.score() << ", sd: " << ukf.sd();
  return os;
}
