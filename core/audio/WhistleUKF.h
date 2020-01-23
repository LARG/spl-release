#pragma once

#include <Eigen/Dense>

#include <math/UnscentedKalmanFilter.h>
#include <math/UnscentedKalmanFilterParams.h>
#include <math/Geometry.h>
#include <math/Common.h>

#include <audio/Logging.h>
#include <audio/KeypointGroup.h>

struct WhistleObservation {
  WhistleObservation(const KeypointGroup* config = nullptr) : config(config) { }
  const KeypointGroup* config = nullptr;
  std::vector<float> scores;
  std::vector<float> teammateScores, teammateSds;
  friend std::ostream& operator<<(std::ostream& os, const WhistleObservation& obs);
};

struct WhistleUKFParams : public UnscentedKalmanFilterParams { 
};

using BaseWhistleUKF = UnscentedKalmanFilter<float, 1, WhistleObservation, WhistleUKFParams>;
class WhistleUKF : public BaseWhistleUKF {
  public:
    using Base = BaseWhistleUKF;
    using Derived = WhistleUKF;
    using Observation = Base::Observation;
    using Params = Base::Params;
    enum Indexes {
      WhistleScore
    };
    DEFAULT_MEAS_DEFINITION(TeammateMeas,4);
    DEFAULT_MEAS_DEFINITION(WhistleMeas,1);
    
    WhistleUKF(const WhistleUKFParams& params);
    WhistleUKF(const WhistleUKF& other);
    virtual float distance(UnscentedKalmanFilter* right) const { return 0.0f; }
    virtual UnscentedKalmanFilter* copy() const { return new WhistleUKF(*this); }
    void update(const Observation& observation);
    float distance(const WhistleUKF* other) const;
    friend std::ostream& operator<<(std::ostream& os, const WhistleUKF& ukf);
    inline float score() const { return state_[WhistleScore]; }
    inline float sd() const { return cov_(WhistleScore,WhistleScore); }
    static constexpr float sanitizeState(float state) {
      if(std::isnan(state) || std::isinf(state)) return 0.0f;
      return static_math::crop(state, 0.0f, 1.0f);
    }
    static constexpr float sanitizeSd(float sd) {
      if(std::isnan(sd) || std::isinf(sd)) return 1.0f;
      if(sd <= 0.0f) return 1.0f;
      return sd;
    }
  
  private:
    StateVec nonlinearTimeTransform(const StateVec& state, const WhistleObservation& observation);
    void sanitize();
    
    WhistleMeas wmeas_;
    TeammateMeas tmeas_;
};
