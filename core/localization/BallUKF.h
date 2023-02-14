#pragma once

#include <Eigen/Dense>

#include <math/UnscentedKalmanFilter.h>
#include <math/UnscentedKalmanFilterParams.h>
#include <localization/Logging.h>

#include <math/Geometry.h>

struct BallObservation { 
  bool hasBall = false;
  bool ballSeenRecently = false;
  Point2D relBall;
  Eigen::Matrix2f relBallUncertainty;
  float timePassed = 0.0f;
  float odomX = 0.0f, odomY = 0.0f, odomTheta = 0.0f, gyroTheta = 0.0f;
  bool useGyro = false;
  friend std::ostream& operator<<(std::ostream& os, const BallObservation& obs);
};

struct BallUKFParams : public UnscentedKalmanFilterParams { 
  float ODOM_THETA_WEIGHT = 0.5f;
  float GYRO_THETA_WEIGHT = 0.5f;
  float BALL_DECAY_RATE = 0.97f;
};

using BaseBallUKF = UnscentedKalmanFilter<float, 4, BallObservation, BallUKFParams>;
class BallUKF : public BaseBallUKF {
  public:
    using Base = BaseBallUKF;
    using Derived = BallUKF;
    using Observation = Base::Observation;
    using Params = Base::Params;
    DEFAULT_MEAS_DEFINITION(BallMeas,2);

    enum Indexes {
      BallX = 0,
      BallY = 1,
      BallVelX = 2,
      BallVelY = 3,
      StateSize
    };
    
    BallUKF(const BallUKFParams& params);
    virtual float distance(UnscentedKalmanFilter* right) const { return 0.0f; }
    virtual UnscentedKalmanFilter* copy() const { return new BallUKF(*this); }
    void update(const Observation& observation);
    float distance(const BallUKF* other) const;
    void processOdometry(const Observation& observation);
    friend std::ostream& operator<<(std::ostream& os, const BallUKF& ukf);
  
  private:
    StateVec nonlinearTimeTransform(const StateVec& state, const BallObservation& observation);
    
    BallMeas bmeas_;
    bool last_obs_seen_recently_;
};
