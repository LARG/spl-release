#ifndef PARALLEL_UKF_H
#define PARALLEL_UKF_H

#include <common/WorldObject.h>
#include <math/UnscentedKalmanFilter.h>
#include <localization/ParallelParams.h>
#include <math/Pose2D.h>
#include <math/Geometry.h>

struct SeenObject {
  WorldObjectType type, utype;
  Point2D relative;
  Point2D global;
  LineSegment matchedLine, obsGlobLine, obsRelLine;
  float bearing;
  Eigen::MatrixXf uncertainty;
  bool disambiguated;
  SeenObject() : uncertainty(2,2), disambiguated(false) { }
  static std::vector<SeenObject> fromWorldObjects(const std::vector<WorldObject>& wobjects) {
    std::vector<SeenObject> sobjects;
    for(auto wo : wobjects) {
      if(wo.isBall() || wo.isT() || wo.isL()) continue;
      SeenObject so;
      so.type = wo.type;
      if(wo.type == WO_CENTER_LINE) {
        so.obsRelLine = wo.visionLine;
        so.matchedLine = wo.lineLoc;
      } else if (wo.isLine()) continue;
      else {
        so.global = wo.loc;
        so.relative = wo.relPos;
      }
      sobjects.push_back(so);
    }
    return sobjects;
  }
};

struct ParallelObservation {
  std::vector<SeenObject> objects;
  std::vector<SeenObject> longlines, shortlines;
  SeenObject ball;
  SeenObject self;
  std::vector<SeenObject> teamballs;
  SeenObject coachball;
  bool hasBall, hasCoachBall, ballSeenRecently;
  
  bool ballKicked;
  float kickVelocity, kickHeading;

  float timePassed;
  float headPan;
  float odomX, odomY, odomTheta, gyroTheta;
  bool useGyro;
  bool bump, fallen, standing;
  bool recentlyPenalized = false;
  int frameid;

  ParallelObservation() {
    odomX = odomY = odomTheta = gyroTheta = 0.0f;
    useGyro = false;
    timePassed = 1.0f/30.0f;
    headPan = 0.0f;
    bump = fallen = standing = false;
    hasBall = false;
    hasCoachBall = false;
    ball.type = WO_BALL;
    ballKicked = false;
    kickVelocity = 0.0f;
    kickHeading = 0.0f;
  }
};

class TextLogger;
class PoseEstimate;

using BaseParallelUKF = UnscentedKalmanFilter<float, 7, ParallelObservation, ParallelParams>;
class ParallelUKF : public BaseParallelUKF {
  public:
    using Base = BaseParallelUKF;
    using Derived = ParallelUKF;
    using Observation = Base::Observation;
    using Params = Base::Params;
    DEFAULT_MEAS_DEFINITION(ShortLineMeas,2);
    DEFAULT_MEAS_DEFINITION(LongLineMeas,5);
    DEFAULT_MEAS_DEFINITION(SingleObjectMeas,2);
    DEFAULT_MEAS_DEFINITION(MultiObjectMeas,3);
    DEFAULT_MEAS_DEFINITION(BallMeas,2);
    DEFAULT_MEAS_DEFINITION(BallVelMeas,2);
    DEFAULT_MEAS_DEFINITION(SharedBallMeas,2);
    DEFAULT_MEAS_DEFINITION(CoachMeas,2);

    enum Indexes {
      SelfX = 0,
      SelfY = 1,
      SelfTheta = 2,
      BallX = 3,
      BallY = 4,
      BallVelX = 5,
      BallVelY = 6,
      StateSize
    };
    ParallelUKF(const ParallelParams& params);
    ParallelUKF(const ParallelUKF& other);
    ParallelUKF(const StateVec& memstate, const StateCov& memcov, const ParallelParams& params);
    ParallelUKF(const Pose2D& pose, const ParallelParams& params);
    ParallelUKF(const PoseEstimate& estimate, const Point2D& ball, const ParallelParams& params);
    ParallelUKF(const PoseEstimate& estimate, const ParallelParams& params);
    ParallelUKF(const Pose2D& pose, const Point2D& ball, const ParallelParams& params);
    ParallelUKF(const Pose2D& pose, const Point2D& ball, const StateCov& cov, const ParallelParams& params);
    void update(const Observation& observation);
    void processOdometry(const Observation& observation);
    const std::set<WorldObjectType>& affinities() { return affinities_; }
    void setAffinities(const std::vector<WorldObject>& objects);
    void setAffinities(const std::vector<WorldObjectType>& types);
    void updateProcessNoise(const Observation& observation);
    void cropState();
    static std::vector<ParallelUKF*> mergeCount(std::vector<ParallelUKF*> models, int count);
    static std::vector<ParallelUKF*> mergeThreshold(std::vector<ParallelUKF*> models, float threshold);
    void init();
    void flip();
    inline int lifespan() const { return lifespan_; }
    inline int& minLifespan() { return min_lifespan_; }
    inline int minLifespan() const { return min_lifespan_; }
    void movePlayer(const Point2D& self, float orientation);
    void moveBall(const Point2D& ball);
    float distance(const ParallelUKF* other) const;
    UnscentedKalmanFilter* copy() const {
      return static_cast<UnscentedKalmanFilter*>(new ParallelUKF(*this));
    }
  private:
    StateVec nonlinearTimeTransform(const StateVec& state, const Observation& observation);
    void measurementUpdate(const Observation& observation);
    void pruneAffinities();

    SingleObjectMeas smeas_;
    BallMeas bmeas_;
    BallVelMeas bvmeas_;
    LongLineMeas llmeas_;
    ShortLineMeas slmeas_;
    SharedBallMeas sbmeas_;
    CoachMeas cmeas_;
    std::set<WorldObjectType> affinities_;
    int lifespan_, min_lifespan_ = 0;
};
#endif
