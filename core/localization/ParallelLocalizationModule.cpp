#include <localization/ParallelLocalizationModule.h>
#include <memory/WorldObjectBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/RobotStateBlock.h>
#include <memory/LocalizationBlock.h>
#include <memory/TeamPacketsBlock.h>
#include <memory/ProcessedSonarBlock.h>
#include <memory/OdometryBlock.h>
#include <memory/BehaviorBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/WalkRequestBlock.h>
#include <memory/JointBlock.h>
#include <memory/CalibrationBlock.h>
#include <Eigen/Dense>
#include <localization/Logging.h>
#include <localization/Spawn.h>
#include <localization/Disambiguator.h>
#include <localization/BallUKF.h>
#include <localization/ParallelUKF.h>
#include <memory/SensorBlock.h>
#include <VisionCore.h>
#include <localization/GyroConfig.h>

using Observation = ParallelObservation;
constexpr int StateSize = ParallelUKF::StateSize;
typedef ParallelLocalizationModule PLM;
using namespace Eigen;

#define REL_POINT(wo) Point2D(wo.visionDistance * cosf(wo.visionBearing), wo.visionDistance * sinf(wo.visionBearing))
#define getSelf(self) auto& self = cache_.world_object->objects_[cache_.robot_state->WO_SELF];
#define getBall(ball) auto& ball = cache_.world_object->objects_[WO_BALL];
#define LINE_DIST_THRESHOLD 1000.0f
#define LINE_ANGLE_THRESHOLD (30 * DEG_T_RAD)
#define MAX_MODELS 6
#define MIN_SPLIT_FRAMES 600
#define PARALLEL_FACTOR 1
#define LINE_LENGTH_THRESHOLD 400.0f
#define SQ(X) ((X)*(X))
// theta from gyroZ parameters (see rUNSWIFT 2015 code release)
#define ALPHA 0.25f
#define AVG_DELTA_GYRO_Z_RESET 10.0f
#define GYRO_DELTA_THRESHOLD 1 // rads/sec
#define GYRO_WINDOW_SIZE 0.005f

ParallelLocalizationModule::ParallelLocalizationModule() {
  loctypes_ = {
    WO_BALL,
    WO_UNKNOWN_PENALTY_CROSS,
    WO_CENTER_CIRCLE,
    WO_UNKNOWN_GOAL,
    WO_UNKNOWN_LEFT_GOALPOST,
    WO_UNKNOWN_RIGHT_GOALPOST,
    WO_UNKNOWN_GOALPOST,
    WO_UNKNOWN_L_1,
    WO_UNKNOWN_L_2,
    WO_UNKNOWN_T_1,
    WO_UNKNOWN_T_2,
    WO_UNKNOWN_FIELD_LINE_1,
    WO_UNKNOWN_FIELD_LINE_2,
    WO_UNKNOWN_FIELD_LINE_3,
    WO_UNKNOWN_FIELD_LINE_4
  };
  for(int i = LINE_OFFSET; i < LINE_OFFSET + NUM_LINES; i++)
    loctypes_.push_back((WorldObjectType)i);
  printf("Initialized parallel localization module.\n");
}

void PLM::loadParallelParams(ParallelParams params) {
  tlog(30, "Localization params loaded");
  params_ = params;
  for(auto model : models_) model->loadParams(params_);
}

void PLM::initSpecificModule() {
  reInit();
  initConfig();
  ball_params_ = std::make_unique<BallUKFParams>();
  ball_ukf_ = std::make_unique<BallUKF>(*ball_params_);
  disambiguator_ = std::make_unique<Disambiguator>(cache_, textlogger, config_);
  spawn_ = std::make_unique<Spawn>(cache_, textlogger, config_, models_, params_);
  if(cache_.robot_state->body_version_ >= 50) {
    GyroConfig config;
    int id = VisionCore::inst_->rconfig().robot_id;

    // First we load gyroscope configs. We try for robot specific values
    // and if none can be found we try for defaults.
    if(config.load(util::format("%s/%i_z.yaml", util::cfgpath(util::GyroConfigs), id))) {
      printf("Loaded Z gyro configuration for Robot %02i.\n", id);
    } else if(config.load(util::cfgpath(util::GyroConfigs) + "/default_z.yaml")) {
      printf("Loaded default Z gyro configuration.\n");
    } else {
      throw std::runtime_error("Error loading Z gyro configuration!");
    }
    cache_.calibration->slip_amount_ = config.slip_amount;
    cache_.calibration->pessimistic_scale_ = config.pessimistic_scale;
    cache_.calibration->gyro_z_offset_ = config.offsetZ;
    cache_.calibration->calibration_write_time_ = config.calibration_write_time;

    // The gyro offset is written to a different file during game play.
    // If values were written in the last 10 minutes then we will load from
    // this file.
    if (config.load("gyroZ_calibration.yaml")) {
      if (getSystemTime() - config.calibration_write_time) {
        cache_.calibration->gyro_z_offset_ = config.offsetZ;
        cache_.calibration->calibration_write_time_ = config.calibration_write_time;
      }
    }
  }
}

void PLM::reInit() {
  tlog(30, "Localization reinitialized");
  if(cache_.robot_state->WO_SELF == WO_TEAM_COACH) return;
  if(!params_.LOADED) return;
  initMemory();
  auto model = new ParallelUKF(params_);
  initModels(model);
}


void PLM::initFromWorld() {
  if(cache_.robot_state->WO_SELF == WO_TEAM_COACH) return;
  if(!params_.LOADED) return;
  getSelf(self);
  getBall(ball);
  auto model = new ParallelUKF(Pose2D(self.orientation, self.loc.x, self.loc.y), ball.loc, params_);
  initModels(model);
  tlog(30, "Localization initialized from world at %2.f,%2.f @ %2.f", self.loc.x, self.loc.y, self.orientation * RAD_T_DEG);
}

void PLM::initFromMemory() {
  if(cache_.robot_state->WO_SELF == WO_TEAM_COACH) return;
  if(!params_.LOADED) return;
  auto& mem = *cache_.localization_mem;
  if(mem.factor == PARALLEL_FACTOR) { // MemoryFrame has already been initialized
    for(auto m : models_) delete m;
    models_.clear();
    for(int i = 0; i < MAX_MODELS_IN_MEM; i++) {
      if(mem.alpha[i] < 0) continue;
      auto memstate = mem.state[i].block<StateSize,1>(0,0);
      auto memcov = mem.covariance[i].block<StateSize,StateSize>(0,0);
      auto model = new ParallelUKF(memstate, memcov, params_);
      model->alpha() = mem.alpha[i];
      models_.push_back(model);
      tlog(30, "Localization initialized from memory at %2.f,%2.f @ %2.f, Best? %i", memstate[ParallelUKF::SelfX], memstate[ParallelUKF::SelfY], memstate[ParallelUKF::SelfTheta] * RAD_T_DEG, i == mem.bestModel);
      if(i == mem.bestModel) {
        best_ = model;
      }
    }
    updateMemory();
    spawn_->initFromMemory();
  } else return;
}

void PLM::initModels(ParallelUKF* current) {
  if(!params_.LOADED) return;
  for(auto m : models_) {
    if(m != current) delete m; 
  }
  models_.clear();
  models_.push_back(current);
  best_ = current;
}

void PLM::initMemory() {
  auto& mem = *cache_.localization_mem;
  mem.useSR = false;
  mem.factor = PARALLEL_FACTOR;
  for(int i = 1; i < MAX_MODELS_IN_MEM; i++) {
    mem.alpha[i] = -1;
  }
}

void PLM::processFrame() {
#ifdef TOOL // Need to reinitialize these in the tool because the memory block locations change
  {
    if(best_ == nullptr) return;
    auto model = best_;
    auto& self = cache_.world_object->objects_[cache_.robot_state->WO_SELF];
    self.orientation = model->state()[ParallelUKF::SelfTheta];
    self.loc = Point2D(model->state()[ParallelUKF::SelfX], model->state()[ParallelUKF::SelfY]);
  }

#endif
  if(cache_.robot_state->WO_SELF == WO_TEAM_COACH) return;
  if(!params_.LOADED) return;
  auto modelsCopy = models_;
  getBall(ball);
  for(auto model : modelsCopy) {
    // Don't process models that have been deleted
    if(find(models_.begin(), models_.end(), model) == models_.end()) continue; 
    Observation obs;
    obs.frameid = cache_.frame_info->frame_id;
    obs.ballSeenRecently = obs.frameid - ball.frameLastSeen < 30;
    processPriorKnowledge(model);
    if(cache_.game_state->state() == PLAYING || cache_.game_state->state() == INITIAL || cache_.game_state->state() == TESTING)
      processCommunications(model, obs);
    processObservations(model, obs);
  }
  updateMemory();
}

void PLM::processPriorKnowledge(ParallelUKF* model) {
  if(cache_.game_state->state() == SET && !(cache_.game_state->isPenaltyKick or cache_.game_state->isFreeKickTypePenalty)) {
    float epsilon = 10;
    if(model->state()[ParallelUKF::SelfX] > epsilon && abs(model->state()[ParallelUKF::SelfTheta]) > M_PI / 2)
      model->flip();
  }
}

void PLM::processCommunications(ParallelUKF* model, Observation& obs) {
  int frame = cache_.frame_info->frame_id;
  // Look at each teammate message - if recent (within 5 frames), add the ball's
  // location to the list of observed balls (adjusted by uncertainy based on how
  // old the message is).
  for(int i = WO_TEAM_FIRST; i <= WO_TEAM_LAST; i++) {
    if(i == cache_.robot_state->WO_SELF) continue;
    int bframe = cache_.team_packets->ballUpdated[i];
    if(frame != bframe) {
      tlog(10, "ignore ball packet from teammate %i because it was received at frame %i, my frame %i, diff %i, max %i", i, bframe, frame, frame - bframe, 0);
      continue;
    }
    int rframe = cache_.team_packets->frameReceived[i]; 
    int maxframes = 30;
    if(rframe > frame || frame - rframe > maxframes) {
      tlog(10, "ignore ball packet from teammate because of frame %i, rframe %i, diff %i, max %i", frame, rframe, frame - rframe, maxframes);
      continue;
    }
    auto locData = cache_.team_packets->relayData[i].locData;
    auto bvrData = cache_.team_packets->relayData[i].bvrData;
    if(bvrData.ballMissed > 4) {
      tlog(10, "ignore message from robot %i who has missed ball for %i frames", i, bvrData.ballMissed);
      continue;
    }
    if(std::isnan(locData.ballPos().x) || std::isnan(locData.ballPos().y) || locData.ballCov(0,0) <= 0 || locData.ballCov(1,1) <= 0) {
      tlog(10, "Bad teammate ball packet from robot %i", i);
      continue;
    }
    // Disabled while we continue testing ball sharing - JM 06/01/15
    //if(cache_.game_state->state() != TESTING && cache_.game_state->state() != INITIAL && (bvrData.state != READY && bvrData.state != SET && bvrData.state != PLAYING && bvrData.state != TESTING)) {
      //tlog(10, "Ignored teammate packet because of state %i", bvrData.state);
      //continue;
    //}
    int adjust = frame - rframe + 1;
    SeenObject ball;
    ball.uncertainty = locData.ballCov.cast<float>() * adjust;
    ball.global = locData.ballPos();
    obs.teamballs.push_back(ball);
    tlog(46, "Received team packet (frame: %i, rframe %i, age %i) with ball at %2.f,%2.f, cov:\n\t%2.2f\t%2.2f\n\t%2.2f\t%2.2f", frame, rframe, frame - rframe, ball.global.x, ball.global.y, ball.uncertainty(0,0), ball.uncertainty(0,1), ball.uncertainty(1,0), ball.uncertainty(1,1));
  }
}

void PLM::clipToField(const Point2D& selfLoc, float selfOrientation, WorldObject& object) {
  static const auto endEdges = vector<LineSegment> {
    LineSegment(HALF_FIELD_X, HALF_FIELD_Y, HALF_FIELD_X, -HALF_FIELD_Y), // Forward
    LineSegment(-HALF_FIELD_X, -HALF_FIELD_Y, -HALF_FIELD_X, HALF_FIELD_Y), // Back
  };
  static const auto sideEdges = vector<LineSegment> {
    LineSegment(HALF_FIELD_X, -HALF_FIELD_Y, -HALF_FIELD_X, -HALF_FIELD_Y), // Right
    LineSegment(-HALF_FIELD_X, HALF_FIELD_Y, HALF_FIELD_X, HALF_FIELD_Y) // Left
  };
  vector<LineSegment> edges;
  if(object.isGoal())
    edges = endEdges;
  LineSegment s(selfLoc, object.loc);
  vector<Point2D> intersections;
  for(const auto& edge : edges) {
    if(s.intersects(edge)) {
      intersections.push_back(s.getSegmentIntersection(edge));
    }
  }
  std::sort(intersections.begin(), intersections.end(), [&](const auto& left, const auto& right) {
    return left.getDistanceTo(object.loc) < right.getDistanceTo(object.loc);
  });
  if(intersections.size() == 0) return;
  auto intersection = intersections[0];
  object.loc = intersection;
  object.relPos = object.loc.globalToRelative(selfLoc, selfOrientation);
  object.visionBearing = object.relPos.getDirection();
  object.visionDistance = object.relPos.getMagnitude();
}

/** Fills in an Observation object using odometry and gyros. Also adds some additional location information about seen vision objects to their WorldObjects. 
 
  The ukf model is used to calculate relative distances/angles to objects.
 */
void PLM::fillBaseObservations(ParallelUKF* model, Observation& obs) {
  tlog(40, "Filling in base observations");
  static int prevstate = cache_.game_state->state();
  static int stateStartFrame = cache_.frame_info->frame_id;
  if(prevstate != cache_.game_state->state()) {
    stateStartFrame = cache_.frame_info->frame_id;
  }
  prevstate = cache_.game_state->state();
  state_elapsed_frames_ = cache_.frame_info->frame_id - stateStartFrame;
  obs.recentlyPenalized = cache_.game_state->prevstate() == PENALISED && cache_.game_state->state() != PENALISED && state_elapsed_frames_ < 240;

  obs.timePassed = 1.0f/30.0f;
  obs.bump = cache_.sonar->bump_left_ || cache_.sonar->bump_right_ || cache_.odometry->getting_up_side_ != Getup::NONE;
  obs.fallen = cache_.odometry->getting_up_side_ != Getup::NONE || cache_.odometry->fall_direction_ != Fall::NONE;
  obs.standing = cache_.odometry->standing && cache_.walk_request->motion_ == WalkRequestBlock::STAND;
  obs.ballKicked = cache_.odometry->didKick;
  obs.kickVelocity = cache_.odometry->kickVelocity;
  obs.kickHeading = cache_.odometry->kickHeading;
  // Using gyroZ to determine turning angles for V5 robots (Body ID >= 50)
  if(cache_.robot_state->body_version_ >= 50) {
    obs.useGyro = true;
    double cur_gyroZ = cache_.sensor->getValue(gyroZ);
    double delta_gyroZ = abs(cur_gyroZ - cache_.calibration->last_gyro_z_);
    tlog(40, "cur gyro z: %2.2f, avg gyro z: %2.2f, delta thresh: %2.2f", cur_gyroZ, delta_gyroZ, GYRO_DELTA_THRESHOLD);
    // maintain moving averages for gyro and delta_gyro
    cache_.calibration->avg_gyro_z_ = cache_.calibration->avg_gyro_z_ * (1.0 - GYRO_WINDOW_SIZE) + cur_gyroZ * GYRO_WINDOW_SIZE;
    cache_.calibration->avg_delta_gyro_z_ = cache_.calibration->avg_delta_gyro_z_ * (1.0- GYRO_WINDOW_SIZE) + delta_gyroZ * GYRO_WINDOW_SIZE;
    if (cache_.calibration->avg_delta_gyro_z_ < GYRO_DELTA_THRESHOLD) {
      // robot remains still, do calibration
      cache_.calibration->gyro_z_offset_ = cache_.calibration->avg_gyro_z_;
      // reset avg_delta so it does not keep recalibrating
      cache_.calibration->avg_delta_gyro_z_ = AVG_DELTA_GYRO_Z_RESET;
      cache_.calibration->gyro_z_cal_count_ += 1;
      if (cache_.calibration->gyro_z_cal_count_ == 1) {
        tlog(40, "first calibration (maybe inaccurate), offset z: %2.4f", cache_.calibration->gyro_z_offset_);
        cout << "first calibration (maybe inaccurate), offset z: " <<  cache_.calibration->gyro_z_offset_ << endl;
      }
      else {     
        tlog(40, "later calibration, offset z: %2.4f", cache_.calibration->gyro_z_offset_);
        cout << "later calibration, offset z: " << cache_.calibration->gyro_z_offset_ << endl;
      }
      cache_.calibration->last_gyro_z_time_ = cache_.frame_info->seconds_since_start;
    }
    cache_.calibration->last_gyro_z_ = cur_gyroZ;

    static float slipAverage = 0.0f;
    static float gyroTheta = 0.0f;
    cache_.calibration->last_gyro_theta_ = gyroTheta;
    // Take out gyroZ offset for V6s
    // gyroTheta = (cache_.sensor->getValue(gyroZ) - cache_.calibration->gyro_z_offset_) / 30.0f; //VisionCore::inst_->camtimer_.avgrate(); // 30.0f;
    gyroTheta = cache_.sensor->getValue(gyroZ) / 30.0f; //VisionCore::inst_->camtimer_.avgrate(); // 30.0f;
    gyroTheta *=  cache_.calibration->pessimistic_scale_;
    float slipError = gyroTheta - cache_.odometry->displacement.rotation;
    tlog(40, "slip error: %2.4f, slip avg start: %2.4f", slipError, cache_.calibration->slip_average_);
    cache_.calibration->slip_average_ = slipAverage;
    cache_.calibration->slip_average_ = ALPHA * slipError + (1.f - ALPHA) * cache_.calibration->slip_average_;
    slipAverage = cache_.calibration->slip_average_;
    tlog(40, "slip error: %2.4f, slip avg end: %2.4f", slipError, cache_.calibration->slip_average_);
    tlog(40, "gyro theta: %2.4f, last gyro theta: %2.4f, slip error: %2.4f, slip avg: %2.4f, slip amount: %2.4f, cal count: %i", gyroTheta, cache_.calibration->last_gyro_theta_, slipError, cache_.calibration->slip_average_, cache_.calibration->slip_amount_, cache_.calibration->gyro_z_cal_count_); 
    if(fabs(cache_.calibration->slip_average_) > cache_.calibration->slip_amount_ && cache_.calibration->gyro_z_cal_count_ >= 2) {
      obs.gyroTheta = -normalizeAngle(gyroTheta);
      tlog(40, "gyro theta: %2.2f", obs.gyroTheta * RAD_T_DEG);
    }
    else {
      // TODO: we should just be setting obs.useGyro = false here, or better yet,
      // move all of this code into the walk engine where it belongs
      obs.gyroTheta = cache_.odometry->displacement.rotation;
      tlog(40, "setting gyro theta to odom rotation of %2.2f", obs.gyroTheta);
    }
    tlog(40, "gyro z calibration count: %i\n", cache_.calibration->gyro_z_cal_count_);
    if (cache_.calibration->gyro_z_cal_count_ >= 2 || getSystemTime() - cache_.calibration->calibration_write_time_ < 600) {
      cache_.calibration->gyro_z_cal_count_ = 2;
      tlog(40, "last gyro z time: %2.8f, last write: %2.8f, > +30? %i", cache_.calibration->last_gyro_z_time_, cache_.calibration->last_calibration_write_, cache_.calibration->last_gyro_z_time_ > cache_.calibration->last_calibration_write_ + 30);
      if (cache_.calibration->last_gyro_z_time_ > cache_.calibration->last_calibration_write_ + 30) {
        cache_.calibration->calibration_write_time_ = getSystemTime();
        GyroConfig config;
        config.offsetX = 0.0;
        config.offsetY = 0.0;
        config.offsetZ = cache_.calibration->gyro_z_offset_;
        config.slip_amount = cache_.calibration->slip_amount_;
        config.pessimistic_scale = cache_.calibration->pessimistic_scale_;
        config.calibration_write_time = cache_.calibration->calibration_write_time_;
        config.saveToFile("gyroZ_calibration.yaml");
        cache_.calibration->last_calibration_write_ = cache_.frame_info->seconds_since_start;
      }
    }
    total_turn += obs.gyroTheta * RAD_T_DEG;
    //cout << "gyroTheta accumulated from localization: " << total_turn << endl;

  }
  tlog(40, "Should we turn off the gyros? Fallen: %i, Recently Penalized: %i, Use Gyro: %i", obs.fallen, obs.recentlyPenalized, obs.useGyro);
  if(obs.fallen) obs.useGyro = false;
  if(obs.recentlyPenalized) obs.useGyro = false;
  tlog(40, "Final Gyro: %i", obs.useGyro);
  auto sloc = Point2D(model->state()[ParallelUKF::SelfX], model->state()[ParallelUKF::SelfY]);
  auto sorient = model->state()[ParallelUKF::SelfTheta];
  auto disp = cache_.odometry->displacement;
  tlog(40, "Local odometry displacement: %2.2f,%2.2f @ %2.2f, gyro(%s): %2.2f", disp.translation.x, disp.translation.y, disp.rotation * RAD_T_DEG, obs.useGyro ? "used" : "ignored", obs.gyroTheta * RAD_T_DEG);
  if(obs.standing) {
    tlog(40, "Standing, ignoring odometry: %2.f,%2.f @ %2.f", disp.translation.x, disp.translation.y, disp.rotation);
    disp.translation.x = disp.translation.y = disp.rotation = 0;
  }
  if (fabs(disp.rotation) > 1.0 || fabs(disp.translation.x) > 100 || fabs(disp.translation.y) > 100.0){
    tlog(40, "Bad odometry: %2.f,%2.f @ %2.f", disp.translation.x, disp.translation.y, disp.rotation);
    disp.translation.x = disp.translation.y = disp.rotation = 0;
  }
  if(!obs.standing && cache_.walk_request->walk_type_ == BHUMAN2011_WALK && disp.translation.x < 0 && disp.translation.x  > -3) {
    // Hack because the b-human's reverse walk odometry is bad - JM 04/23/15
    //disp.translation.x += 7.5;
  }
  Point2D pdisp(disp.translation.x, disp.translation.y);
  pdisp = pdisp.rotate(sorient);
  tlog(40, "Processing model %0x at %2.f,%2.f @ %2.f with displacement %2.6f,%2.6f @ %2.6f", model, sloc.x, sloc.y, sorient * RAD_T_DEG, pdisp.x, pdisp.y, disp.rotation * RAD_T_DEG);
  obs.odomX = pdisp.x, obs.odomY = pdisp.y, obs.odomTheta = disp.rotation;
  tlog(40, "Odom update: %2.f,%2.f @ %2.4f", obs.odomX, obs.odomY, obs.odomTheta * RAD_T_DEG);
  obs.self.type = static_cast<WorldObjectType>(cache_.robot_state->WO_SELF);
  obs.self.global = sloc;
  obs.self.relative = Point2D(0,0);
  obs.headPan = cache_.joint->values_[HeadPan];
  // Fills in location and position information about seen vision objects. Note that these do not go into obs. This is done later by fillSeenObjects.
  tlog(40, "Filling in vision observations");
  for(auto type : loctypes_) {
    auto& object = cache_.world_object->objects_[type];
    if(!object.seen) continue;
    object.relPos = REL_POINT(object);
    if(object.isAmbiguous())
      object.loc = object.relPos.relativeToGlobal(sloc, sorient);
    //if(object.isGoal()) {
      //tlog(40, "object %s is a goal so we're clipping it to the field from %s", getName(object.type), object.loc);
      //clipToField(sloc, sorient, object);
      //tlog(40, "object %s has new clipped location: %s", getName(object.type), object.loc);
    //}
    tlog(40, "Saw %s at dist %2.2f, bear %2.2f, rel: %2.f,%2.f, glob: %2.f,%2.f", getName(object.type), object.visionDistance, object.visionBearing * RAD_T_DEG, object.relPos.x, object.relPos.y, object.loc.x, object.loc.y);
    if(object.isLine() && object.isAmbiguous()) {
      object.lineLoc = object.visionLine.relativeToGlobal(sloc, sorient);
      tlog(40, "Object is line from %2.f,%2.f to %2.f,%2.f", object.lineLoc.start.x, object.lineLoc.start.y, object.lineLoc.end.x, object.lineLoc.end.y);
    }
  }
}

void PLM::processObservations(ParallelUKF* model, Observation& obs) {
  // Reset throwouts and attempts
  model->start();
  // Fill in observation object (updates obs with gyros and odom) 
  fillBaseObservations(model, obs);
  vector<WorldObject> ambiguousObjects;
  for(auto type : loctypes_) {
    auto& object = cache_.world_object->objects_[type];
    if(object.seen) ambiguousObjects.push_back(object);
  }
  disambiguator_->updateSelf(
    Point2D(model->state()[ParallelUKF::SelfX], model->state()[ParallelUKF::SelfY]),
    model->state()[ParallelUKF::SelfTheta]
  );
  auto objectSets = disambiguator_->disambiguate(ambiguousObjects, model->affinities());
  model->attempt(disambiguator_->thrownOut());
  model->throwout(disambiguator_->thrownOut());
  tlog(40, "throwing out %i items from the disambiguator (%i thrown out)", disambiguator_->thrownOut(), model->throwouts());
  // Update observations object with vision
  fillSeenObjects(model, obs, objectSets[0]);
  model->tlogger() = textlogger;
  // Update the UKF model with the observations
  model->update(obs);
  if(model == best_) {
    tlog(40, "attempting splits");
    attemptSplits(model, obs, objectSets);
    updateCloseBall(obs);
  }
}



void PLM::attemptSplits(ParallelUKF* model, const Observation& obs, vector<ObjectSet>& objectSets) {
  vector<Spawns::SpawnType> spawns;
#ifdef TOOL
  int stateElapsedTime = state_elapsed_frames_;
#else
  int stateElapsedTime = cache_.game_state->stateElapsedTime();
#endif
  tlog(40, "state %s, elapsed time %i, elapsed frames: %i", stateNames[cache_.game_state->state()].c_str(), stateElapsedTime, state_elapsed_frames_);
  if(cache_.game_state->state() == INITIAL)
    spawns = { Spawns::InitialState, Spawns::CenterCircle, Spawns::Throwouts, Spawns::GoalLines };
  if(cache_.game_state->state() == READY && (cache_.game_state->prevstate() == INITIAL || cache_.game_state->prevstate() == FINISHED) && stateElapsedTime <= 5) {
    spawns.push_back(Spawns::ReadyState);
    spawns.push_back(Spawns::CenterCircle);
  }
  else if((cache_.game_state->isPenaltyKick || cache_.game_state->isFreeKickTypePenalty) && cache_.game_state->state() == PLAYING && cache_.game_state->prevstate() != PLAYING && stateElapsedTime == 0) {
    spawns.push_back(Spawns::PenaltyKick);
  }
  else if((cache_.game_state->state() == PLAYING || cache_.game_state->state() == READY) && cache_.game_state->prevstate() == PENALISED && stateElapsedTime == 0 && cache_.game_state->spawnFromPenalty) {
    spawns.push_back(Spawns::PenalizedState);
  }
  else if((cache_.game_state->state() == PLAYING || cache_.game_state->state() == READY || cache_.game_state->state() == MANUAL_CONTROL) && obs.fallen)
    spawns.push_back(Spawns::Fallen);
  else if(cache_.game_state->state() == PLAYING || cache_.game_state->state() == MANUAL_CONTROL)
    spawns = { Spawns::Ambiguity, Spawns::BallFlip, Spawns::CenterCircle, Spawns::Throwouts, Spawns::GoalLines };
  if((cache_.game_state->prevstate() == INITIAL || cache_.game_state->prevstate() == FINISHED) && cache_.game_state->change())
    spawn_->reset();
  tlog(40, "attempting to spawn new models on %i types", spawns.size());
  auto newModels = spawn_->attempt(model, spawns, obs, objectSets);
  for(auto model : newModels) {
    tlog(40, "processing newly spawned model.");
    auto cobs = obs;
    fillBaseObservations(model.model, cobs);
    fillSeenObjects(model.model, cobs, model.objects);
    model.model->tlogger() = textlogger;
    model.model->update(cobs);
    models_.push_back(model.model);
  }
}

void PLM::checkErrors() {
  history_.push_back(dynamic_cast<ParallelUKF*>(best_->copy()));
  if(history_.size() > 10) {
    auto model = history_.front();
    delete model;
    history_.pop_front();
  }
  const auto& state = best_->state();
  const auto& cov = best_->covariance();
  bool reset = false;
  for(int i = 0; i < state.size(); i++)
    if(isnan(state[i])) reset = true;
  for(int i = 0; i < cov.rows(); i++)
    for(int j = 0; j < cov.cols(); j++)
      if(isnan(cov(i,j))) reset = true;
  if(reset) {
    if(history_.size() > 0)
      best_ = dynamic_cast<ParallelUKF*>(history_.front()->copy());
    else best_ = new ParallelUKF(params_);
    while(history_.size()) {
      delete history_.front();
      history_.pop_front();
    }
    models_ = { best_ };
  }
}

/** At this point, all objects that remain should be unambiguous. 
 
  This updates the Observation object with 
  It also fills in a base uncertainty which is scaled later by ParallelUKF.
 */
void PLM::fillSeenObjects(ParallelUKF* model, Observation& obs, vector<WorldObject>& dobjects) {
  tlog(40, "filling %i seen objects", dobjects.size());
  obs.objects.clear();
  obs.shortlines.clear();
  obs.longlines.clear();
  auto sloc = Point2D(model->state()[ParallelUKF::SelfX], model->state()[ParallelUKF::SelfY]);
  auto sorient = model->state()[ParallelUKF::SelfTheta];
  bool best = model == best_;
  for(auto dobject : dobjects) {
    model->attempt();
    if(best) {
      cache_.world_object->objects_[dobject.type].frameLastSeen = cache_.frame_info->frame_id;
    }
    tlog(40, "Processing disambiguated object %s", getName(dobject.type));
    if(dobject.isBall()) {
      obs.ball.bearing = dobject.visionBearing;
      obs.ball.relative = dobject.relPos;
      obs.ball.global = dobject.relPos.relativeToGlobal(sloc, sorient);
      // TODO: trim the ball's location instead of throwing it out
      if(fabs(obs.ball.global.x) > HALF_GRASS_X || fabs(obs.ball.global.y) > HALF_GRASS_Y) {
        model->throwout();
        tlog(40, "Throwing out ball because it's not on the field (%i thrown out).", model->throwouts());
        continue;
      }
      Matrix2f rotation; rotation = Rotation2Df(dobject.visionBearing + sorient);
      obs.ball.uncertainty <<
        25, 0,
        0, 1;
      obs.ball.uncertainty = rotation * obs.ball.uncertainty * rotation.transpose();
      obs.ball.uncertainty /= sqrtf(obs.ball.uncertainty.determinant());
      if(best) cache_.localization_mem->objectCov[WO_BALL] = obs.ball.uncertainty;
      obs.hasBall = true;
      // Hard code the position of the ball in SET
      if(cache_.game_state->state() == SET) {
        if(obs.ball.global.getMagnitude() > CIRCLE_RADIUS) model->degrade();
        SeenObject knownBall = obs.ball;
        if (cache_.game_state->isPenaltyKick || cache_.game_state->isFreeKickTypePenalty) {
          knownBall.global = Point2D(PENALTY_CROSS_X,0);
        } else {
          knownBall.global = Point2D(0,0);
        }
        obs.objects.push_back(knownBall);
        obs.ball.global = knownBall.global;
      }
    } else {
      // The keeper is stationary on a PK and doesn't need to update its position
      if((cache_.game_state->isPenaltyKick || cache_.game_state->isFreeKickTypePenalty)
              && cache_.robot_state->WO_SELF == KEEPER) continue;
      SeenObject sobj;
      sobj.disambiguated = !dobject.seen;
      sobj.bearing = dobject.visionBearing;
      sobj.relative = dobject.relPos;
      sobj.type = dobject.type;
      sobj.utype = dobject.utype;
      if(!sobj.disambiguated) sobj.utype = sobj.type;
      if(dobject.isLine()) {
        auto proj = sobj.relative.relativeToGlobal(sloc, sorient);
        auto cglobal = dobject.lineLoc.getNearestCircleIntersection(sloc, dobject.visionDistance, proj);
        auto cangle = sloc.getAngleBetweenPoints(proj, cglobal);
        sobj.global = cglobal;
        sobj.matchedLine = dobject.lineLoc;
        sobj.obsRelLine = dobject.visionLine;
        sobj.obsGlobLine = dobject.visionLine.relativeToGlobal(sloc, sorient);
        float dist = sobj.matchedLine.getDistanceTo(sobj.obsGlobLine.center);
	// TODO(sanmit): look at this later
        if(dist > LINE_DIST_THRESHOLD && cangle > LINE_ANGLE_THRESHOLD && sobj.utype != WO_CENTER_LINE) {
          tlog(40, "Object %s thrown out because of dist %2.2f (max %2.2f) and angle %2.2f (max %2.2f)", getName(sobj.utype), dist, LINE_DIST_THRESHOLD, cangle * RAD_T_DEG, LINE_ANGLE_THRESHOLD * RAD_T_DEG);
          model->throwout();
          continue;
        }
        sobj.uncertainty << 
          100, 0,
          0, 1;
        // Store the global version of the uncertainty
        Matrix2f srotation; srotation = Rotation2Df(sobj.obsGlobLine.getAngle());
        if(best) cache_.localization_mem->objectCov[sobj.utype] = srotation * sobj.uncertainty * srotation.transpose();
        // TODO: update logging calls to use ostream
        // Rotate line uncertainties because they're evaluated with relative coordinates
        tlog(40, "Set uncert for %s (u: %s) to %2.2f,%2.2f,%2.2f,%2.2f", getName(sobj.type), getName(sobj.utype),
          sobj.uncertainty(0,0), 
          sobj.uncertainty(0,1), 
          sobj.uncertainty(1,0), 
          sobj.uncertainty(1,1)
        );
        Matrix2f rotation; rotation = Rotation2Df(sobj.obsRelLine.getAngle());
        sobj.uncertainty = rotation * sobj.uncertainty * rotation.transpose();
        if(dobject.visionLine.length() < LINE_LENGTH_THRESHOLD)
          obs.shortlines.push_back(sobj);
        else
          obs.longlines.push_back(sobj);
        tlog(40, "Set rotated uncert for %s to %2.2f,%2.2f,%2.2f,%2.2f", getName(dobject.type), 
          sobj.uncertainty(0,0), 
          sobj.uncertainty(0,1), 
          sobj.uncertainty(1,0), 
          sobj.uncertainty(1,1)
        );
      }
      // Intersections, goals, etc.
      else {
        Matrix2f rotation; rotation = Rotation2Df(dobject.visionBearing + sorient);
        float base = std::max(SQ(dobject.visionDistance / 100),10000.0f);
        sobj.global = dobject.loc;
        sobj.uncertainty <<
          base, 0,
          0, 100;
        sobj.uncertainty = rotation * sobj.uncertainty * rotation.transpose();
        sobj.uncertainty /= sqrtf(sobj.uncertainty.determinant());
        obs.objects.push_back(sobj);
        if(best) cache_.localization_mem->objectCov[sobj.utype] = sobj.uncertainty;
      }
    }
  }
}

void PLM::updateCloseBall(const Observation& obs) {
  auto self = Point2D(best_->state()[ParallelUKF::SelfX], best_->state()[ParallelUKF::SelfY]);
  float sorient = best_->state()[ParallelUKF::SelfTheta];
  auto ball = Point2D(best_->state()[ParallelUKF::BallX], best_->state()[ParallelUKF::BallY]);
  ball = ball.globalToRelative(self, sorient);

  BallObservation bo;
  Matrix2f rotation; rotation = Rotation2Df(-sorient);
  bo.relBallUncertainty = rotation * obs.ball.uncertainty * rotation.transpose();
  bo.ballSeenRecently = obs.ballSeenRecently;
  bo.hasBall = obs.hasBall;
  if(obs.ballSeenRecently)
    bo.relBall = obs.ball.relative;
  else
    bo.relBall = ball;
  bo.timePassed = obs.timePassed;
  auto disp = cache_.odometry->displacement;
  bo.odomX = disp.translation.x, bo.odomY = disp.translation.y, bo.odomTheta = disp.rotation;
  bo.gyroTheta = obs.gyroTheta;
  bo.useGyro = obs.useGyro;
  tlog(43, "Created close ball observation:\n%s\n", bo);
  tlog(43, "Starting close ball update");
  ball_ukf_->tlogger() = textlogger;
  ball_ukf_->update(bo);
}

void PLM::normalizeModels() {
  if(models_.empty()) return;
  int origSize = models_.size();
  int maxModels = MAX_MODELS;
  if(cache_.robot_state->ignore_comms_) {
    maxModels = 8; // Allow more models since we have to spawn manual and penalized together.
  }
  tlog(43, "pre-merge %i models", models_.size());
#ifdef TOOL
  for(auto m1 : models_) {
    for(auto m2 : models_) {
      tlog(44, "distance between model at %2.f,%2.f @ %2.2f (alpha: %2.4f) and %2.f,%2.f @ %2.2f (alpha: %2.4f): %2.2f", 
          m1->state()[ParallelUKF::SelfX], m1->state()[ParallelUKF::SelfY], m1->state()[ParallelUKF::SelfTheta] * RAD_T_DEG, m1->alpha(),
          m2->state()[ParallelUKF::SelfX], m2->state()[ParallelUKF::SelfY], m2->state()[ParallelUKF::SelfTheta] * RAD_T_DEG, m2->alpha(),
          m1->distance(m2));
    }
  }
#endif
  models_ = ParallelUKF::mergeCount(models_, maxModels);
  int nextSize = models_.size();
  if(origSize > nextSize)
    tlog(44, "%i models removed from count merge", origSize - nextSize);
  models_ = ParallelUKF::mergeThreshold(models_, 10.0f);
  if(nextSize > models_.size())
    tlog(44, "%i models removed from distance merge", nextSize - models_.size());
  tlog(43, "post-merge %i models", models_.size());
#ifdef TOOL
  for(auto m1 : models_) {
    for(auto m2 : models_) {
      tlog(44, "distance between model at %2.f,%2.f @ %2.2f (alpha: %2.4f) and %2.f,%2.f @ %2.2f (alpha: %2.4f): %2.2f", 
          m1->state()[ParallelUKF::SelfX], m1->state()[ParallelUKF::SelfY], m1->state()[ParallelUKF::SelfTheta] * RAD_T_DEG, m1->alpha(),
          m2->state()[ParallelUKF::SelfX], m2->state()[ParallelUKF::SelfY], m2->state()[ParallelUKF::SelfTheta] * RAD_T_DEG, m2->alpha(),
          m1->distance(m2));
    }
  }
#endif

  std::sort(models_.begin(), models_.end(), 
    [](ParallelUKF* a, ParallelUKF* b) { 
      return a->alpha() > b->alpha(); 
  });
  auto maxAlpha = models_[0]->alpha();
  for(auto model : models_) {
    model->alpha() /= maxAlpha;
    model->finish();
  }
  vector<ParallelUKF*> goodModels;
  float threshold = 0.1f;
  for(auto model : models_)
    if(model->alpha() > threshold || model->lifespan() < 10 || model->minLifespan() > model->lifespan())
      goodModels.push_back(model);
    else
      delete model;
  tlog(43, "%i models removed from bad alphas", models_.size() - goodModels.size());
  models_ = goodModels;
}

void PLM::selectBestModel() {
  if(models_.size() == 0) {
    best_ = nullptr;
    fprintf(stderr, "ERROR: No models found in localization!\n");
    return;
  }
  if(models_.size() == 1) {
    best_ = models_[0];
    return;
  }
  // Set the best as the one with the highest alpha if it's been deleted
  if(find(models_.begin(), models_.end(), best_) == models_.end()) {
    best_ = models_[0];
  }
  if(models_[0]->alpha() * params_.MODEL_SWITCH_ALPHA_FACTOR > best_->alpha()) {
    tlog(40, "Switched models, from %2.f,%2.f @ %2.f A=%2.2f to %2.f,%2.f @ %2.f A=%2.2f",
      best_->state()[ParallelUKF::SelfX], best_->state()[ParallelUKF::SelfY], best_->state()[ParallelUKF::SelfTheta] * RAD_T_DEG, best_->alpha(),
      models_[0]->state()[ParallelUKF::SelfX], models_[0]->state()[ParallelUKF::SelfY], models_[0]->state()[ParallelUKF::SelfTheta] * RAD_T_DEG, models_[0]->alpha());
    best_ = models_[0];
  }
}

void PLM::movePlayer(const Point2D& position, float orientation) {
  for(auto m : models_) {
    m->movePlayer(position, orientation);
  }
}

void PLM::moveBall(const Point2D& position) {
  for(auto m : models_) {
    m->moveBall(position);
  }
}

/** This is where we check to see what the best model is, and update memory blocks with our position, ball position, distances/bearings to objects, etc. */
void PLM::updateMemory() {
  normalizeModels();
  selectBestModel();
  if(best_ == nullptr) return;
  checkErrors();
  getSelf(self);
  auto& mem = *cache_.localization_mem;

  for(int i = 0; i < MAX_MODELS_IN_MEM; i++) mem.alpha[i] = -1;
  for(int i = 0; i < MAX_MODELS_IN_MEM && i < models_.size(); i++) {
    auto& model = *models_[i];
    tlog(43, "model %i attempts: %i, throwouts: %i, rate: %2.2f, avg: %2.2f", i, model.attempts(), model.throwouts(), model.throwoutRate(), model.avgThrowoutRate());
    mem.modelNumber[i] = i;
    mem.alpha[i] = model.alpha();
    mem.state[i].block<StateSize,1>(0,0) = model.state();
    tlog(40, "model %i (%0x) mem: (%2.f,%2.f) @ %2.2f alpha %2.3f", i, &model,
      mem.state[i][ParallelUKF::SelfX], mem.state[i][ParallelUKF::SelfY],
      mem.state[i][ParallelUKF::SelfTheta] * RAD_T_DEG, mem.alpha[i]
    );
    mem.covariance[i].block<StateSize,StateSize>(0,0) = model.covariance();
    mem.numModels = models_.size();
    if(&model == best_) {
      mem.bestAlpha = model.alpha();
      mem.bestModel = i;
      
      self.loc = Point2D(model.state()[ParallelUKF::SelfX], model.state()[ParallelUKF::SelfY]);
      self.orientation = model.state()[ParallelUKF::SelfTheta];
      self.sd.x = sqrtf(model.covariance()(ParallelUKF::SelfX,ParallelUKF::SelfX));
      self.sd.y = sqrtf(model.covariance()(ParallelUKF::SelfY,ParallelUKF::SelfY));
      auto disp = cache_.odometry->displacement;
      self.relVel.x = disp.translation.x;
      self.relVel.y = disp.translation.y;
      
      getBall(ball);
      ball.loc = Point2D(model.state()[ParallelUKF::BallX], model.state()[ParallelUKF::BallY]);
      ball.relPos = ball.loc.globalToRelative(self.loc, self.orientation);
      ball.sd.x = sqrtf(model.covariance()(ParallelUKF::BallX,ParallelUKF::BallX));
      ball.sd.y = sqrtf(model.covariance()(ParallelUKF::BallY,ParallelUKF::BallY));
      ball.relVel = ball.absVel = Point2D(model.state()[ParallelUKF::BallVelX], model.state()[ParallelUKF::BallVelY]);
      ball.relVel.rotate(-self.orientation);
    }
  }
  for(int i = MAX_MODELS_IN_MEM; i < models_.size(); i++) {
    const auto& s = models_[i]->state();
    tlog(40, "skipped model %i (%0x) state: (%2.f,%2.f) @ %2.2f alpha %2.3f", i, models_[i],
      s[ParallelUKF::SelfX], s[ParallelUKF::SelfY],
      s[ParallelUKF::SelfTheta] * RAD_T_DEG, models_[i]->alpha()
    );
  }
  for(int i = 0; i < NUM_WORLD_OBJS; i++) {
    auto& obj = cache_.world_object->objects_[i];
    obj.distance = self.loc.getDistanceTo(obj.loc);
    obj.bearing = self.loc.getBearingTo(obj.loc, self.orientation);
  }
  if(ball_ukf_ != nullptr) {
    auto 
      X = BallUKF::BallX, Y = BallUKF::BallY,
      VX = BallUKF::BallVelX, VY = BallUKF::BallVelY;
    cache_.behavior->keeperRelBallPos.x = ball_ukf_->state()[X];
    cache_.behavior->keeperRelBallPos.y = ball_ukf_->state()[Y];
    cache_.behavior->keeperRelBallVel.x = ball_ukf_->state()[VX];
    cache_.behavior->keeperRelBallVel.y = ball_ukf_->state()[VY];
    cache_.behavior->keeperRelBallPosCov = ball_ukf_->covariance().block<2,2>(X,X);
    cache_.behavior->keeperRelBallVelCov = ball_ukf_->covariance().block<2,2>(VX,VX);
  }
}
