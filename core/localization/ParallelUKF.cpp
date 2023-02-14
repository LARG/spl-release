#include <localization/ParallelUKF.h>
#include <memory/TextLogger.h>
#include <vision/Constants.h>
#include <common/Field.h>
#include <Eigen/Dense>
#include <localization/PoseEstimate.h>
#include <localization/Logging.h>

#define CROP(x,MIN,MAX) x = std::max(std::min(((float)x),((float)(MAX))),((float)(MIN)))
#define SQ(x) ((x)*(x))
#define logstate(msg) tlog(40, \
    "%s -> UKF State: Self: %2.f,%2.f @ %2.2f, Ball: %2.f,%2.f @ %2.f,%2.f," \
    "Alpha: %2.2f", msg, \
    state()[ParallelUKF::SelfX], state()[ParallelUKF::SelfY], state()[ParallelUKF::SelfTheta] * RAD_T_DEG, \
    state()[ParallelUKF::BallX], state()[ParallelUKF::BallY], \
    state()[ParallelUKF::BallVelX], state()[ParallelUKF::BallVelY], \
    alpha() \
    );


//TODO: reuse WorldObject code
#define isIntersection(type) ((type >= WO_OPP_PEN_LEFT_T && type <= WO_OWN_PEN_LEFT_T) || (type >= WO_OPP_FIELD_LEFT_L && type <= WO_OWN_FIELD_LEFT_L))
#define isGoal(type) (type >= WO_OWN_GOAL && type <= WO_UNKNOWN_GOALPOST)

using namespace std;
using namespace Eigen;

using BaseUKF = ParallelUKF::Base;
using Observation = ParallelObservation;
typedef ParallelUKF::ShortLineMeas ShortLineMeas;
typedef ParallelUKF::LongLineMeas LongLineMeas;
typedef ParallelUKF::SingleObjectMeas SingleMeas;
typedef ParallelUKF::BallMeas BallMeas;
typedef ParallelUKF::BallVelMeas BallVelMeas;
typedef ParallelUKF::SharedBallMeas SharedBallMeas;
typedef ParallelUKF::CoachMeas CoachMeas;

ParallelUKF::ParallelUKF(const ParallelParams& params) : UnscentedKalmanFilter(params), smeas_(*this), bmeas_(*this), bvmeas_(*this), llmeas_(*this), slmeas_(*this), sbmeas_(*this), cmeas_(*this) {
  normalizers_[SelfTheta] = normalizeAngle;
  init();
}

ParallelUKF::ParallelUKF(const ParallelUKF& other) : ParallelUKF(other.params_) {
  state_ = other.state_;
  cov_ = other.cov_;
  process_ = other.process_;
  alpha_ = other.alpha_;
}

ParallelUKF::ParallelUKF(const StateVec& memstate, const StateCov& memcov, const ParallelParams& params) : ParallelUKF(params) {
  state_ = memstate;
  cov_ = memcov;
}

ParallelUKF::ParallelUKF(const Pose2D& pose, const ParallelParams& params) : ParallelUKF(params) {
  state_[SelfX] = pose.translation.x;
  state_[SelfY] = pose.translation.y;
  state_[SelfTheta] = pose.rotation;
}

ParallelUKF::ParallelUKF(const PoseEstimate& estimate, const ParallelParams& params) : ParallelUKF(estimate.pose, params) {
}

ParallelUKF::ParallelUKF(const PoseEstimate& estimate, const Point2D& ball, const ParallelParams& params) : ParallelUKF(estimate.pose, ball, params) {
}

ParallelUKF::ParallelUKF(const Pose2D& pose, const Point2D& ball, const ParallelParams& params) : ParallelUKF(pose, params) {
  state_[BallX] = ball.x;
  state_[BallY] = ball.y;
}

ParallelUKF::ParallelUKF(const Pose2D& pose, const Point2D& ball, const StateCov& cov, const ParallelParams& params) : ParallelUKF(pose, ball, params) {
  cov_ = cov;
}

void ParallelUKF::init() {
  lifespan_ = 0;
  
  state_ = StateVec::Zero();
  
  StateVec process;
  process[SelfX] = process[SelfY] = params_.PROCESS_SELF_XY;
  process[SelfTheta] = params_.PROCESS_SELF_THETA;
  process[BallX] = process[BallY] = params_.PROCESS_BALL_XY;
  process[BallVelX] = process[BallVelY] = params_.PROCESS_BALL_VEL_XY;
  process_ = process.asDiagonal(); process_ *= process_;

  StateVec cov;
  cov[SelfX] = cov[SelfY] = params_.COV_SELF_XY;
  cov[SelfTheta] = params_.COV_SELF_THETA;
  cov[BallX] = cov[BallY] = params_.COV_BALL_XY;
  cov[BallVelX] = cov[BallVelY] = params_.COV_BALL_VEL_XY;
  cov_ = cov.asDiagonal(); cov_ *= cov_;
}

BallMeas::BallMeas(BaseUKF& ukf) : BaseMeas(ukf) { 
  maxInnovation_ = ukf.params().MAX_INNOVATION_SEENBALL;
}

BallMeas::MeasVec BallMeas::nonlinearPredictionTransform(const StateVec& state, const Observation& observation) {
  BallMeas::MeasVec meas;
  meas << state[BallX], state[BallY];
  return meas;
}

void BallMeas::update(const Observation& observation) {
  if(!observation.hasBall) return;
  BallMeas::MeasVec meas;
  meas << observation.ball.global.x, observation.ball.global.y;
  Matrix2f uncert = observation.ball.uncertainty;
  uncert *= max(SQ(observation.ball.relative.getMagnitude() / 5), SQ(200.0f));
  uncert = MatrixOperations::SanitizeCovariance(uncert);
  float innov = baseUpdate(meas, uncert, observation, true);
  tlog(45, "Ball measurement at %2.f,%2.f, Fmeas: %2.f,%2.f, Diff: %2.f,%2.f, adjust: %2.f,%2.f, Innov: %2.4f, uncert:\n %s", meas[0], meas[1], fmeas_[0], fmeas_[1], diff_[0], diff_[1], adj_[BallX], adj_[BallY], innov, uncert);
}

BallVelMeas::BallVelMeas(BaseUKF& ukf) : BaseMeas(ukf) { }  

BallVelMeas::MeasVec BallVelMeas::nonlinearPredictionTransform(const StateVec& state, const Observation& observation) {
  BallVelMeas::MeasVec meas;
  meas << state[BallVelX], state[BallVelY];
  return meas;
}

void BallVelMeas::update(const Observation& observation) {
  if(!observation.hasBall) return;
  Point2D self(state()[SelfX], state()[SelfY]);
  Point2D last(state()[BallX], state()[BallY]);
  last = last.globalToRelative(self, state()[SelfTheta]);
  Point2D relVel = (observation.ball.relative - last) / observation.timePassed;
  Point2D absVel = relVel.rotate(-state()[SelfTheta]);
  BallVelMeas::MeasVec meas;
  meas << absVel.x, absVel.y;
  Matrix2f uncert = observation.ball.uncertainty * 50000;
  uncert = MatrixOperations::SanitizeCovariance(uncert);
  float innov = baseUpdate(meas, uncert, observation);
  tlog(45, "Ball vel measurement at %2.f,%2.f, Innovation: %2.2f", meas[0], meas[1], innov);
}

SharedBallMeas::SharedBallMeas(BaseUKF& ukf) : BaseMeas(ukf) { }  

SharedBallMeas::MeasVec SharedBallMeas::nonlinearPredictionTransform(const StateVec& state, const Observation& observation) {
  MeasVec meas;
  if(observation.ballSeenRecently && observation.self.type != 1) { 

    // If we're wrong about the ball's global position, it's either
    // because we don't know where the ball is relative to us,
    // or because we don't know our own position. If we've seen the 
    // ball recently, we assume we don't know our own position,
    // so we perform a pose update.
    auto ball = observation.ball.relative;
    auto global = ball.relativeToGlobal(Point2D(state[SelfX],state[SelfY]),state[SelfTheta]);
    meas << global.x, global.y;
  } else {
    // Otherwise, just update the ball's position.
    meas << state[BallX], state[BallY];
  }
  return meas;
}

void SharedBallMeas::update(const Observation& observation) {
  auto obsCopy = observation;
  while(!obsCopy.teamballs.empty()) { 
    auto ball = obsCopy.teamballs.back();
    MeasVec meas;
    meas << ball.global.x, ball.global.y;
    MeasCov uncert;
    uncert = ball.uncertainty * 10;
    uncert = MatrixOperations::SanitizeCovariance(uncert);
    auto innov = baseUpdate(meas, uncert, obsCopy);
    tlog(45, 
      "Shared position meas: %2.f,%2.f, Fmeas: %2.f,%2.f, "
      "Diff: %2.f,%2.f, Adj: %2.f,%2.f, Innov: %2.4f, Uncert: %s",
      meas[0], meas[1], fmeas_[0], fmeas_[1], 
      diff_[0], diff_[1], adj_[SelfX], adj_[SelfY], innov, uncert
    );
    obsCopy.teamballs.pop_back();
  }
}

CoachMeas::CoachMeas(BaseUKF& ukf) : BaseMeas(ukf) {
  maxInnovation_ = ukf.params().MAX_INNOVATION_COACHBALL;
}

CoachMeas::MeasVec CoachMeas::nonlinearPredictionTransform(const StateVec& state, const Observation& observation) {
  MeasVec meas;
  if(IGNORE_COACH_DELAYS && observation.hasBall) {
    auto ball = observation.ball;
    Point2D self(state[SelfX], state[SelfY]);
    auto global = ball.relative.relativeToGlobal(self, state[SelfTheta]);
    meas << global.x, global.y;
  }
  else
    meas << state[BallX], state[BallY];
  return meas;
}

void CoachMeas::update(const Observation& observation) {
  if(!observation.hasCoachBall) return;
  auto ball = observation.coachball;
  MeasVec meas;
  meas << ball.global.x, ball.global.y;
  MeasCov uncert;
  uncert = ball.uncertainty;
  uncert = MatrixOperations::SanitizeCovariance(uncert);
  float innov = baseUpdate(meas, uncert, observation, true);
  tlog(45, "Coach measurement at %2.f,%2.f, Innovation: %2.2f (max %2.2f), Uncert: %s", meas[0], meas[1], innov, maxInnovation_, uncert);
}

LongLineMeas::LongLineMeas(BaseUKF& ukf) : BaseMeas(ukf) { 
  differencers_[4] = [](float left, float right) {
    float d = left - right;
    d = fmod(d, M_PI);
    if(d < 0) d += M_PI;
    if(d > M_PI/2) d = M_PI - d;
    return d;
  };
}

LongLineMeas::MeasVec LongLineMeas::nonlinearPredictionTransform(const StateVec& state, const Observation& observation) {
  auto self = Point2D(state[SelfX], state[SelfY]);
  auto obj = observation.longlines.back();
  auto gvis = obj.matchedLine;
  auto angle = obj.obsGlobLine.getSignedAngleToLine(gvis);
  auto rline = obj.obsGlobLine;
  tlog(46, "rline: %s, angle to glob: %2.2f", rline, angle * RAD_T_DEG);
  rline.rotate(self, angle);
  tlog(46, "rotated line %2.2f degrees about %2.f,%2.f: %s", angle * RAD_T_DEG, self.x, self.y, rline);
  gvis.crop(rline);
  auto vis = gvis.globalToRelative(self, state[SelfTheta]);
  auto direction = vis.getDirection();
  LongLineMeas::MeasVec meas;
  meas << vis.start.x, vis.start.y, vis.end.x, vis.end.y, direction;
  tlog(46, "Measurement from %2.f,%2.f @ %2.f: %s", self.x, self.y, state[SelfTheta] * RAD_T_DEG, gvis);
  return meas;
}

void LongLineMeas::update(const Observation& observation) {
  auto obsCopy = observation;
  while(!obsCopy.longlines.empty()) {
    auto obj = obsCopy.longlines.back();
    auto line = obj.obsRelLine;
    auto gline = obj.obsGlobLine;
    LongLineMeas::MeasVec meas;
    tlog(45, "Updating with line %s at %s, uncert:\n%s", getName(obj.type), gline, obj.uncertainty);
    meas << line.start.x, line.start.y, line.end.x, line.end.y, line.getDirection();
    MeasCov uncert = MeasCov::Identity();
    uncert.block<2,2>(0,0) = obj.uncertainty;
    uncert.block<2,2>(2,2) = obj.uncertainty;
    uncert(4,4) = SQ(0.1 * DEG_T_RAD);
    float ddiff = normalizeAngleNinety(gline.getDirection() - obj.matchedLine.getDirection());
    if(fabs(ddiff) > 70 * DEG_T_RAD) {
      obsCopy.longlines.pop_back();
      continue;
    }
    if (obj.relative.getMagnitude() > 2500){
      tlog(45, "Throwing out line due to distance %3.2f > 2500", obj.relative.getMagnitude());
      obsCopy.longlines.pop_back();
      continue;
    }
    if(obj.type == WO_CENTER_LINE && false) {
      maxInnovation_ = ukf_.params().MAX_INNOVATION_CENTERLINE;
      gline.computeCenter();
      auto gtDistance = obj.matchedLine.getDistanceToPoint(gline.center);
      uncert.block<4,4>(0,0) *= pow(200,2);
      uncert(4,4) *= 300;
      auto innov = baseUpdate(meas, uncert, obsCopy);
      auto self = Point2D(state()[SelfX],state()[SelfY]);
      auto fline = LineSegment(fmeas_[0], fmeas_[1], fmeas_[2], fmeas_[3]).relativeToGlobal(self, state()[SelfTheta]);
      tlog(45, "Center Line Fmeas: %s, Diff: %2.f,%2.f --> %2.f,%2.f, Adj: %2.f, %2.f @ %2.f, Innovation: %2.2f (max %2.2f), uncert:\n %s", 
          fline, diff_[0], diff_[1], diff_[2], diff_[3], adj_[0], adj_[1], adj_[2] * RAD_T_DEG, innov, maxInnovation_, uncert);
    } else {
      uncert.block<4,4>(0,0) *= pow(50,2);      // old: 100
      uncert(4,4) *= 800;                       // old: 400
      maxInnovation_ = ukf_.params().MAX_INNOVATION_LONGLINE;
      auto innov = baseUpdate(meas, uncert, obsCopy);
      auto self = Point2D(state()[SelfX],state()[SelfY]);
      auto fline = LineSegment(fmeas_[0], fmeas_[1], fmeas_[2], fmeas_[3]).relativeToGlobal(self, state()[SelfTheta]);
      tlog(45, "Long Line Fmeas: %s, Diff: %2.f,%2.f --> %2.f,%2.f,\n\tAdj: %2.f, %2.f @ %2.f, Innovation: %2.2f (max %2.2f), uncert:\n %s", 
          fline, diff_[0], diff_[1], diff_[2], diff_[3], adj_[0], adj_[1], adj_[2] * RAD_T_DEG, innov, maxInnovation_, uncert);
    }
    obsCopy.longlines.pop_back();
  }
}

ShortLineMeas::ShortLineMeas(BaseUKF& ukf) : BaseMeas(ukf) { 
  maxInnovation_ = ukf.params().MAX_INNOVATION_SHORTLINE;
}

ShortLineMeas::MeasVec ShortLineMeas::nonlinearPredictionTransform(const StateVec& state, const Observation& observation) {
  auto line = observation.shortlines.back().matchedLine;
  Point2D self(state[SelfX], state[SelfY]);
  auto vline = line.getVisiblePortion(self, state[SelfTheta] + observation.headPan, FOVx);
  auto point = vline.getPointOnSegmentClosestTo(self).globalToRelative(self, state[SelfTheta]);
  ShortLineMeas::MeasVec meas;
  meas << point.x, point.y;
  return meas;
}

void ShortLineMeas::update(const Observation& observation) {
  auto obsCopy = observation;
  while(!obsCopy.shortlines.empty()) {
    auto obj = obsCopy.shortlines.back();
    if (obj.relative.getMagnitude() > 2500){
      tlog(45, "Throwing out line due to distance %3.2f > 2500", obj.relative.getMagnitude());
      obsCopy.shortlines.pop_back();
      continue;
    }
    Point2D self(ukf_.state()[SelfX], ukf_.state()[SelfY]);
    Point2D closest = obj.obsGlobLine
      .getPointOnSegmentClosestTo(self)
      .globalToRelative(self, ukf_.state()[SelfTheta]);
    ShortLineMeas::MeasVec meas;
    meas << closest.x, closest.y;
    Matrix2f uncert = obj.uncertainty * 500;
    uncert = MatrixOperations::SanitizeCovariance(uncert);
    auto innov = baseUpdate(meas, uncert, obsCopy);
    obsCopy.shortlines.pop_back();
    tlog(45, "Short line Meas: %2.f,%2.f, Fmeas: %2.f,%2.f, Diff: %2.f,%2.f,\n\tAdj: %2.f, %2.f @ %2.f, Innovation: %2.2f (max %2.2f), uncert:\n %s", 
      meas[0], meas[1], fmeas_[0], fmeas_[1], diff_[0], diff_[1], adj_[0], adj_[1], adj_[2] * RAD_T_DEG, innov, maxInnovation_, uncert);
  }
}

SingleMeas::SingleObjectMeas(BaseUKF& ukf) : BaseMeas(ukf) {
  maxInnovation_ = ukf.params().MAX_INNOVATION_SINGLEOBJ;
}

SingleMeas::MeasVec SingleMeas::nonlinearPredictionTransform(const StateVec& state, const Observation& observation) {
  auto object = observation.objects.back();
  Point2D self(state[SelfX], state[SelfY]);
  Point2D relative = object.global.globalToRelative(self, state[SelfTheta]);
  Point2D global = object.relative.relativeToGlobal(self, state[SelfTheta]);
  SingleMeas::MeasVec meas;
  meas << relative.x, relative.y;
  float distance = relative.getMagnitude();
  if(isGoal(object.type) && distance > EPSILON)
    meas = meas * 1000 / relative.getMagnitude();
  float theta = relative.getDirection();
  tlog(46, "single meas from %2.f,%2.f @ %2.f: rel: %2.f, %2.f, glob: %2.f,%2.f, meas: %s",
    self.x,self.y,state[SelfTheta] * RAD_T_DEG, relative.x, relative.y, global.x, global.y, meas.transpose()
  );
  return meas;
}

void SingleMeas::update(const Observation& observation) {
  auto obsCopy = observation;
  while(!obsCopy.objects.empty()) {
    auto obj = obsCopy.objects.back();
    MeasVec meas;
    meas << obj.relative.x, obj.relative.y;
    float innov = -1;
    MeasCov uncert;
    maxInnovation_ = ukf_.params().MAX_INNOVATION_SINGLEOBJ;
    auto self = Point2D(state()[SelfX],state()[SelfY]);
    if(obj.type == WO_CENTER_CIRCLE) {
      auto gtDistance = obj.relative.relativeToGlobal(self, state()[SelfTheta]).getMagnitude();
      if(gtDistance > 2000) {
        obsCopy.objects.pop_back();
        tlog(46, "throwing out center cirle detection because of gt distance %2.f\n", gtDistance);
        ukf_.throwout();
//        printf("Throwing out circle!!!! GT distance: %3.2f Rel Magnitude: %3.2f\n", gtDistance, obj.relative.getMagnitude());
        continue;
      }
      auto bDistance = self.getMagnitude();
      tlog(46, "circle distance: %2.f (300 max), self distance: %2.f, (%2.f min)", gtDistance, bDistance, CIRCLE_RADIUS);
      if(gtDistance < 300 && bDistance < CIRCLE_RADIUS) {
        // Updating while we're inside of the circle is pretty noisy
        obsCopy.objects.pop_back();
//        printf("INSIDE CIRCLE!!!\n");
        continue;
      }
      else
        uncert = obj.uncertainty * SQ(50);
//        uncert = obj.uncertainty * 10 * std::max(250.0f, obj.relative.getMagnitude());
     
//      printf("Original %3.2f versus new %3.2f\n", SQ(50.0), std::max(2500.0f, obj.relative.getMagnitude()));
      
      tlog(45, "gtDistance: %2.4f, uncert:\n %s",gtDistance, uncert);
      uncert = MatrixOperations::SanitizeCovariance(uncert);
      maxInnovation_ = ukf_.params().MAX_INNOVATION_CENTERCIRCLE;
      innov = baseUpdate(meas, uncert, obsCopy);
    } else if(isGoal(obj.type)) {
      if(obj.relative.getMagnitude() < EPSILON) {
        ukf_.throwout();
        obsCopy.objects.pop_back();
        continue;
      }
      meas = meas * 1000 / obj.relative.getMagnitude();
      uncert = obj.uncertainty * std::max(200.0f, obj.relative.getMagnitude() / 10);
      uncert = MatrixOperations::SanitizeCovariance(uncert);
      innov = baseUpdate(meas, uncert, obsCopy);
    } else if(obj.type == WO_BALL) { 
      // If we're observing the ball it's because it's in a known position.
      uncert = MeasCov::Identity() * 10000;
      innov = baseUpdate(meas, uncert, obsCopy, true);
    } else if(isIntersection(obj.type)) {
      uncert = obj.uncertainty * 1000 * std::max(100.0f,obj.relative.getMagnitude());
      uncert = MatrixOperations::SanitizeCovariance(uncert);
      maxInnovation_ = .5;
      innov = baseUpdate(meas, uncert, obsCopy);
    } else if (obj.type == WO_OWN_PENALTY_CROSS || obj.type == WO_OPP_PENALTY_CROSS) {
      uncert = obj.uncertainty * 250 * std::max(100.0f, obj.relative.getMagnitude());
      uncert = MatrixOperations::SanitizeCovariance(uncert);
      innov = baseUpdate(meas, uncert, obsCopy);
    } else {
      uncert = obj.uncertainty * 50000 * obj.relative.getMagnitude();
      uncert = MatrixOperations::SanitizeCovariance(uncert);
      innov = baseUpdate(meas, uncert, obsCopy);
    }
    tlog(45, "Single Object '%s' Measurement: %2.f,%2.f, FMeas: %2.f,%2.f, Diff: %2.f,%2.f, Adj: %2.f,%2.f @ %2.2f, Innovation: %2.2f (max %2.2f), Uncert: %s", getName(obj.type), meas[0], meas[1], fmeas_[0], fmeas_[1], diff_[0], diff_[1], adj_[0], adj_[1], adj_[2] * RAD_T_DEG, innov, maxInnovation_, uncert);
    obsCopy.objects.pop_back();
  }
}

ParallelUKF::StateVec ParallelUKF::nonlinearTimeTransform(const StateVec& v, const Observation& observation) {
  float timePassed = observation.timePassed;
  StateVec state = v;
  state[BallX] += v[BallVelX] * timePassed;
  state[BallY] += v[BallVelY] * timePassed;
  state[BallVelX] *= params_.BALL_DECAY_RATE;
  state[BallVelY] *= params_.BALL_DECAY_RATE;
  return state;
}

/** Update the UKF using measurements, odometry, and time updates (which I'm guessing is like process noise?) */
void ParallelUKF::update(const Observation& observation) {
  lifespan_++;

  logstate("Start of updates");
  auto orig = process_;
  //updateProcessNoise(observation);
  timeUpdate(observation);
  process_ = orig;
  logstate("Time update");

  if(!observation.fallen)
    measurementUpdate(observation);
  processOdometry(observation);

  cropState();
  pruneAffinities();
}

void ParallelUKF::processOdometry(const Observation& observation) {
  float odomT = observation.odomTheta * params_.ODOM_FACTOR_THETA;
  float gyroT = observation.gyroTheta * params_.GYRO_FACTOR_THETA;
  float theta = odomT * params_.ODOM_THETA_WEIGHT + gyroT * params_.GYRO_THETA_WEIGHT;
  logstate("Pre-odometry");
  state_[SelfX] += observation.odomX * params_.ODOM_FACTOR_XY;
  state_[SelfY] += observation.odomY * params_.ODOM_FACTOR_XY;
  if (observation.useGyro)
    state_[SelfTheta] += theta;
  else
    state_[SelfTheta] += odomT;
  if(observation.ballKicked) {
    state_[BallVelX] = observation.kickVelocity * cosf(observation.kickHeading + state_[SelfTheta]);
    state_[BallVelY] = observation.kickVelocity * sinf(observation.kickHeading + state_[SelfTheta]);
  }
  logstate("Post-odometry");
}

void ParallelUKF::pruneAffinities() {
  if(lifespan_ > 300)
    affinities_.clear();
}

void ParallelUKF::measurementUpdate(const Observation& observation) {
  logstate("Measurement start");
  bmeas_.update(observation);
  logstate("Ball Meas");
  llmeas_.update(observation);
  logstate("Long Line Meas");
  slmeas_.update(observation);
  logstate("Short Line Meas");
  smeas_.update(observation);
  logstate("Single Object Meas");
  sbmeas_.update(observation);
  logstate("Shared Ball Meas");
  cmeas_.update(observation);
  logstate("Coach Meas");
}

void ParallelUKF::updateProcessNoise(const Observation& observation) {
  if(observation.bump) {
    process_(SelfTheta,SelfTheta) *= SQ(params_.PROCESS_BUMP_FACTOR);
  }

  // if fallen, less movement in x,y noise
  if(observation.fallen) {
    process_(SelfX,SelfX) /= SQ(params_.PROCESS_FALLEN_ROBOT_FACTOR);
    process_(SelfY,SelfY) /= SQ(params_.PROCESS_FALLEN_ROBOT_FACTOR);
    process_(SelfTheta,SelfTheta) *= SQ(params_.PROCESS_FALLEN_ROBOT_FACTOR);
    process_(BallX,BallX) /= SQ(params_.PROCESS_FALLEN_ROBOT_FACTOR);
    process_(BallY,BallY) /= SQ(params_.PROCESS_FALLEN_ROBOT_FACTOR);
  }

  // if standing, less movement
  if(observation.standing) {
    process_(SelfX,SelfX) /= SQ(params_.PROCESS_STANDING_ROBOT_FACTOR);
    process_(SelfY,SelfY) /= SQ(params_.PROCESS_STANDING_ROBOT_FACTOR);
    process_(SelfTheta,SelfTheta) /= SQ(params_.PROCESS_STANDING_ROBOT_FACTOR);
  }

  if(fabs(state_[BallVelX]) > 200 || fabs(state_[BallVelY]) > 200) {
    process_(BallX,BallX) *= params_.PROCESS_MOVING_BALL_FACTOR;
    process_(BallY,BallY) *= params_.PROCESS_MOVING_BALL_FACTOR;
  }
}

void ParallelUKF::cropState() {
  CROP(state_[SelfX], -GRASS_X/2, GRASS_X/2);
  CROP(state_[SelfY], -GRASS_Y/2, GRASS_Y/2);
  CROP(state_[BallX], -GRASS_X/2, GRASS_X/2);
  CROP(state_[BallY], -GRASS_Y/2, GRASS_Y/2);
  CROP(state_[BallVelX], -5000, 5000);
  CROP(state_[BallVelY], -5000, 5000);

  // Underflow can cause problems with the calculations
  for(int i = 0; i < StateSize; i++) {
    float minCov = 1;
    if(i == SelfTheta) minCov = 1 * DEG_T_RAD;
    cov_(i,i) = max(minCov, cov_(i,i));
  }
}

void ParallelUKF::setAffinities(const vector<WorldObjectType>& types) {
  affinities_.clear();
  for(auto t : types) affinities_.insert(t);
}

void ParallelUKF::setAffinities(const vector<WorldObject>& objects) {
  affinities_.clear();
  for(const auto& obj : objects)
    affinities_.insert(obj.type);
}

float ParallelUKF::distance(const ParallelUKF* other) const {
  auto lstate = this->state(), rstate = other->state();
  lstate[SelfTheta] *= 1000;
  rstate[SelfTheta] *= 1000;
  auto diff = lstate - rstate;
  return diff.norm();
}

std::vector<ParallelUKF*> ParallelUKF::mergeCount(std::vector<ParallelUKF*> models, int count) {
  return BaseUKF::mergeCount<ParallelUKF>(models, count);
}

std::vector<ParallelUKF*> ParallelUKF::mergeThreshold(std::vector<ParallelUKF*> models, float threshold) {
  return BaseUKF::mergeThreshold<ParallelUKF>(models, threshold);
}

void ParallelUKF::flip() {
  state_[SelfX] = -state_[SelfX];
  state_[SelfY] = -state_[SelfY];
  state_[SelfTheta] = normalizeAngle(state_[SelfTheta] + M_PI);
  state_[BallX] = -state_[BallX];
  state_[BallY] = -state_[BallY];
  state_[BallVelX] = -state_[BallVelX];
  state_[BallVelY] = -state_[BallVelY];
  affinities_.clear();
}

void ParallelUKF::movePlayer(const Point2D& self, float orientation) {
  state_[SelfX] = self.x;
  state_[SelfY] = self.y;
  state_[SelfTheta] = orientation;
}

void ParallelUKF::moveBall(const Point2D& ball) {
  state_[BallX] = ball.x;
  state_[BallY] = ball.y;
}
