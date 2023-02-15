#include <localization/Spawn.h>

#include <memory/LocalizationBlock.h>
#include <memory/RobotStateBlock.h>
#include <memory/WorldObjectBlock.h>
#include <memory/GameStateBlock.h>
#include <memory/BehaviorBlock.h>
#include <memory/SensorBlock.h>

#include <Eigen/Dense>
#include <Eigen/SVD>

#include <localization/Logging.h>

using namespace Eigen;
using Observation = ParallelObservation;

#define LOG_SPAWN(type,model) \
  tlog(41, "Spawned: Type %s, at %2.f,%2.f @ %2.f", Spawns::getName(type), model->state()[ParallelUKF::SelfX], model->state()[ParallelUKF::SelfY], model->state()[ParallelUKF::SelfTheta] * RAD_T_DEG);

Spawn::Spawn(BASE_DECLARATIONS, vector<ParallelUKF*>& models, ParallelParams&
        params) : LocalizationBase(BASE_ARGS), models_(models), params_(params) {
  frames_ = minframes_ = {
    make_pair(Spawns::Ambiguity, 600),
    make_pair(Spawns::MultiObject, 600),
    make_pair(Spawns::InitialState, 600),
    make_pair(Spawns::ReadyState, 30),
    make_pair(Spawns::PenalizedState, 300),
    make_pair(Spawns::BallFlip, 900),
    make_pair(Spawns::Fallen, 450),
    make_pair(Spawns::PenaltyKick, 90),
    make_pair(Spawns::CenterCircle, 90),
    make_pair(Spawns::GoalLines, 600),
    make_pair(Spawns::Throwouts, 1800)
  };
}

void Spawn::initFromMemory() {
  for(int i = 0; i < (int)Spawns::NUM_SpawnTypes; i++)
    frames_[(Spawns::SpawnType)i] = cache_.localization_mem->spawnFrames[i];
}

vector<SpawnedModel> Spawn::attempt(ParallelUKF* model, vector<Spawns::SpawnType> types, const Observation& obs, const vector<ObjectSet>& objectSets) {
  vector<SpawnedModel> newModels;
  delete_ = false;
  for(auto type : types) {
    tlog(41, "attempting spawn of type %s, ready? %i", Spawns::getName(type), ready(type));
    auto nm = attempt(model, type, obs, objectSets);
    for(auto m : nm) {
      LOG_SPAWN(type,m.model);
      newModels.push_back(m);
    }
  }
  if(delete_) {
    for(auto model : models_) delete model; 
    models_.clear();
  }
  for(auto& kvp : frames_) {
    kvp.second++;
    cache_.localization_mem->spawnFrames[(int)kvp.first] = kvp.second;
  }
  return newModels;
}

vector<SpawnedModel> Spawn::attempt(ParallelUKF* model, Spawns::SpawnType type,
    const Observation& obs, const vector<ObjectSet>& objectSets) {
  vector<SpawnedModel> newModels;
  if(!ready(type)) return newModels;
  int player = cache_.robot_state->WO_SELF;
  bool kickoff = cache_.game_state->ourKickOff;
  auto sloc = Point2D(model->state()[ParallelUKF::SelfX], model->state()[ParallelUKF::SelfY]);
  auto sorient = model->state()[ParallelUKF::SelfTheta];
  switch(type) {
    case Spawns::GoalLines: {
      tlog(86, "spawning goal lines models");
      const auto& lgoal = cache_.world_object->objects_[WO_UNKNOWN_LEFT_GOALPOST];
      const auto& rgoal = cache_.world_object->objects_[WO_UNKNOWN_RIGHT_GOALPOST];
      Pose2D mpose(model->state()[ParallelUKF::SelfTheta],
                   model->state()[ParallelUKF::SelfX],
                   model->state()[ParallelUKF::SelfY]);
      Point2D ball(model->state()[ParallelUKF::BallX],
                   model->state()[ParallelUKF::BallY]);
      if(lgoal.seen && rgoal.seen) {
        tlog(86, "both goal posts seen");
        vector<std::tuple<Point2D,Point2D>> pairs = {std::make_tuple(
            cache_.world_object->objects_[WO_OPP_LEFT_GOALPOST].loc,
            cache_.world_object->objects_[WO_OPP_RIGHT_GOALPOST].loc
          ), std::make_tuple(
            cache_.world_object->objects_[WO_OWN_LEFT_GOALPOST].loc,
            cache_.world_object->objects_[WO_OWN_RIGHT_GOALPOST].loc
          )
        };
        for(auto& p : pairs) {
          const auto& gt_lgoal = std::get<0>(p);
          const auto& gt_rgoal = std::get<1>(p);
          tlog(86, "Considering lgoal at %s and rgoal at %s", gt_lgoal, gt_rgoal);
          float bearingDiff = 0.0f;
          bearingDiff += sloc.getBearingTo(gt_lgoal, sorient) - lgoal.visionBearing;
          bearingDiff += sloc.getBearingTo(gt_rgoal, sorient) - rgoal.visionBearing;
          bearingDiff /= 2;
          if(fabs(bearingDiff) > 90 * DEG_T_RAD) continue;
          tlog(86, "bearing diff is %2.2f", bearingDiff * RAD_T_DEG);
          auto rot_lgoal = lgoal.loc; rot_lgoal.rotate(sloc, bearingDiff);
          auto rot_rgoal = rgoal.loc; rot_rgoal.rotate(sloc, bearingDiff);
          float ldist = rot_lgoal.getDistanceTo(gt_lgoal); 
          float rdist = rot_rgoal.getDistanceTo(gt_rgoal); 
          tlog(86, "rotated observations to %s and %s (ldist: %2.2f, rdist: %2.2f)", rot_lgoal, rot_rgoal, ldist, rdist);
          if(ldist < 2000  && rdist < 2000 && sloc.getMagnitude() > 1000) {
            auto rpose = mpose;
            rpose.rotation = normalizeAngle(rpose.rotation + bearingDiff);
            tlog(86, "passed distance tests, adding spawned model: %s", rpose);
            auto newModel = new ParallelUKF(rpose, ball, params_);
            newModel->alpha() = .3;
            newModels.push_back(SpawnedModel(newModel, objectSets[0]));
          }
        }
      }
    }
    break;

    case Spawns::CenterCircle: {
        auto sobjects = SeenObject::fromWorldObjects(objectSets[0]);
        bool circleSeen = false, lineSeen = false;
        SeenObject circle, line;
        for(const auto& sobj : sobjects) {
          if(sobj.type == WO_CENTER_CIRCLE) {
            circleSeen = true;
            circle = sobj;
          }
          if(sobj.type == WO_CENTER_LINE) {
            lineSeen = true;
            line = sobj;
          }
        }
        if(circleSeen && lineSeen) {
          auto gline = line.obsRelLine.relativeToGlobal(sloc, sorient);
          auto rot = M_PI/2 - gline.getDirection();
          auto norient = sorient + rot;
          auto gcircle = circle.relative.relativeToGlobal(sloc, norient);
          // False Penalty Circle
          Point2D fpc(HALF_FIELD_X-PENALTY_X/2,HALF_FIELD_Y-PENALTY_X/2);
          vector<Point2D> badCircles = {
            Point2D(fpc.x, fpc.y),
            Point2D(fpc.x, -fpc.y),
            Point2D(-fpc.x, -fpc.y),
            Point2D(-fpc.x, fpc.y)
          };
          
          if (player == KEEPER && circle.relative.getMagnitude() > 2500){
            tlog(41, "Keeper throwing out circle spawn since distance too far");
            break;
          }

          tlog(41, "circle seen at %2.f,%2.f, center line at direction %2.2f", circle.relative.x, circle.relative.y, gline.getDirection() * RAD_T_DEG);
          for(const auto& bc : badCircles) {
            if((bc - gcircle).getMagnitude() < PENALTY_X/2) {
              tlog(41, "likely false positive, bailing out. gcircle: %2.f,%2.f, bc: %2.f,%2.f", gcircle.x, gcircle.y, bc.x, bc.y);
              return newModels;
            }
          }
          if(fabs(sloc.x) > HALF_FIELD_X - 200 && fabs(sloc.y) < GOAL_Y / 2) {
            tlog(41, "robot at %2.f,%2.f, we're probably just getting bad detections from the goal", sloc.x, sloc.y);
            break;
          }
          if(fabs(sloc.x) < 1000) {
            tlog(41, "robot is too close to the center");
            break;
          }
          auto nloc = sloc - gcircle;
          auto norient2 = normalizeAngle(norient - M_PI);
          auto nloc2 = -nloc;
          float magnitudeSum = 0.0f, magnitudeSum2 = 0.0f;
          for(auto model : models_) {
            auto modelLoc = Point2D(model->state()[ParallelUKF::SelfX],
                    model->state()[ParallelUKF::SelfY]);
            magnitudeSum += model->alpha() * (modelLoc - nloc).getSquaredMagnitude();
            magnitudeSum2 += model->alpha() * (modelLoc - nloc2).getSquaredMagnitude();
          }
          if(magnitudeSum > magnitudeSum2) {
            nloc = nloc2;
            norient = norient2;
          }
          tlog(41, "new model projected at %2.f,%2.f @ %2.2f", nloc.x, nloc.y, norient * RAD_T_DEG);
          auto odiff = normalizeAngle(sorient - norient), ldiff = (sloc -
                  nloc).getMagnitude();
          if(fabs(odiff) < 15 * DEG_T_RAD && ldiff < 1000) {
            tlog(41, "new model is too close to current model: dist %2.f, angle %2.2f", ldiff, odiff * RAD_T_DEG);
            break;
          }
          if(fabs(nloc.getMagnitude()) < 1000) {
            tlog(41, "new model is too close to the center circle: %2.4f", nloc.getMagnitude());
            break;
          }
          tlog(41, "new model is sufficiently different, odiff %2.2f, ldiff: %2.f", odiff * RAD_T_DEG, ldiff);
          auto pose = Pose2D(norient, nloc.x, nloc.y);
          Point2D ball(model->state()[ParallelUKF::BallX], model->state()[ParallelUKF::BallY]);
          auto newModel = new ParallelUKF(pose, ball, params_);
          newModels.push_back(SpawnedModel(newModel, objectSets[0]));
          complete(type);
        }
      }
      break;

    case Spawns::Ambiguity:
      if(objectSets.size() > 1) {
        for(int i = 0; i < objectSets.size(); i++) {
          auto newModel = new ParallelUKF(*model);
          newModel->setAffinities(objectSets[i]);
          newModels.push_back(SpawnedModel(newModel, objectSets[i]));
        }
        complete(type);
      }
      break;

    case Spawns::MultiObject: {
        auto sobjects = SeenObject::fromWorldObjects(objectSets[0]);
        if(sobjects.size() >= 3) {
          auto estimate = computePoseEstimate(sobjects);
          Point2D ball(model->state()[ParallelUKF::BallX], model->state()[ParallelUKF::BallY]);
          auto newModel = new ParallelUKF(estimate, ball, params_);
          newModel->alpha() = .05;
          newModels.push_back(SpawnedModel(newModel, objectSets[0]));
          complete(type);
        }
      }
      break;

    case Spawns::InitialState: {
        auto pose = cache_.robot_state->manual_pose_;
        auto newModel = new ParallelUKF(pose, params_);
        newModels.push_back(SpawnedModel(newModel, objectSets[0]));
        delete_ = true;
        complete(type);
      }
      break;

    case Spawns::ReadyState: {
        auto pose = RobotPositions::startingSidelinePoses[player];
        auto newModel = new ParallelUKF(pose, params_);
        newModels.push_back(SpawnedModel(newModel, objectSets[0]));
        delete_ = true;
        complete(type);
      }
      break;

    case Spawns::Fallen: {
        Pose2D mpose(model->state()[ParallelUKF::SelfTheta], model->state()[ParallelUKF::SelfX], model->state()[ParallelUKF::SelfY]);
        vector<float> rotations = { M_PI, M_PI / 2, -M_PI / 2 };
        for(auto rotation : rotations) {
          auto rpose = mpose;
          rpose.rotation = normalizeAngle(rpose.rotation + rotation);
          Point2D ball(model->state()[ParallelUKF::BallX], model->state()[ParallelUKF::BallY]);
          auto newModel = new ParallelUKF(rpose, ball, params_);
          newModel->alpha() = .2;
          newModels.push_back(SpawnedModel(newModel, objectSets[0]));
        }
        complete(type);
      }
      break;
    
    case Spawns::Throwouts: {
        tlog(41, "checking throwout models, attempts: %i, throwouts: %i, rate: %2.2f", model->attempts(), model->throwouts(), model->avgThrowoutRate());
        if(model->avgThrowoutRate() < .4) break;
        Pose2D mpose(model->state()[ParallelUKF::SelfTheta], model->state()[ParallelUKF::SelfX], model->state()[ParallelUKF::SelfY]);
        if(abs(mpose.translation.x) < 1000) {
          tlog(41, "we're too close to the center");
          break;
        }
        if(mpose.translation.abs() < CIRCLE_RADIUS * 1.2) break;
        vector<float> rotations = { M_PI / 2, -M_PI / 2 };
        for(auto rotation : rotations) {
          auto rpose = mpose;
          rpose.rotation = normalizeAngle(rpose.rotation + rotation);
          Point2D ball(model->state()[ParallelUKF::BallX], model->state()[ParallelUKF::BallY]);
          auto newModel = new ParallelUKF(rpose, ball, params_);
          newModel->alpha() = .2;
          newModels.push_back(SpawnedModel(newModel, objectSets[0]));
        }
        printf("THROWOUT SPAWN!!!\n");
        complete(type);
      }
      break;

    case Spawns::PenalizedState: {
        tlog(41, "attempting to spawn penalized models");
        vector<Pose2D> poses = { 
          Pose2D(M_PI/2, -PENALTY_CROSS_X, -FIELD_Y/2),
          Pose2D(-M_PI/2, -PENALTY_CROSS_X, FIELD_Y/2)
        };
        int minWhistleTime = 150; // 2.5 minutes
        for(auto pose : poses) {
          auto newModel = new ParallelUKF(pose, params_);
          newModels.push_back(SpawnedModel(newModel, objectSets[0]));
          if(cache_.game_state->whistleElapsedTime() < minWhistleTime && !cache_.game_state->spawned_whistle_positions_) {
            newModel->minLifespan() = 60;
          }

        }
        if(cache_.game_state->whistleElapsedTime() < minWhistleTime && !cache_.game_state->spawned_whistle_positions_) {
          if(cache_.game_state->ourKickOff || cache_.robot_state->ignore_comms_) { // Always spawn kickoff models if we're ignoring comms
            auto pose = RobotPositions::ourKickoffPosesDesired[cache_.robot_state->WO_SELF];
            auto newModel = new ParallelUKF(pose, params_);
            newModel->minLifespan() = 60;
            newModels.push_back(SpawnedModel(newModel, objectSets[0]));
          } else {
            auto pose = RobotPositions::theirKickoffPosesDesired[cache_.robot_state->WO_SELF];
            auto newModel = new ParallelUKF(pose, params_);
            newModel->minLifespan() = 60;
            newModels.push_back(SpawnedModel(newModel, objectSets[0]));
          }
          cache_.game_state->spawned_whistle_positions_ = true;
          cache_.behavior->timePlayingStarted = cache_.frame_info->seconds_since_start - 20;
        } else {
          delete_ = true;
        }
        complete(type);
      }
      break;

    case Spawns::BallFlip: {
        auto oppositeSides = [] (Point2D obj1, Point2D obj2) {
          Point2D opp1 = obj1;
          opp1.x *= -1;
          opp1.y *= -1;
          if((obj1.getDistanceTo(obj2) > 2000) &&
             (opp1.getDistanceTo(obj2) < 1000))
            return true;
          return false;
        };
        tlog(41, "attempting to spawn a flipped model due to ball. hasBall: %i, hasCoachBall: %i, team balls: %i", obs.hasBall, obs.hasCoachBall, obs.teamballs.size());
        Point2D mball = obs.ball.global;
        if(!obs.hasBall) break;
        if(!obs.hasCoachBall && obs.teamballs.size() < 2) break;
        if(obs.hasCoachBall && oppositeSides(mball, obs.coachball.global)) {
          auto nms = flip(model);
          for(auto nm : nms)
            newModels.push_back(SpawnedModel(nm, objectSets[0]));
          complete(type);
          break;
        }
        int opposites = 0;
        for(int i = 0; i < obs.teamballs.size(); i++) {
          if(oppositeSides(mball, obs.teamballs[i].global))
            opposites++;
        }
        if(opposites > 1) {
          auto nms = flip(model);
          for(auto nm : nms)
            newModels.push_back(SpawnedModel(nm, objectSets[0]));
          complete(type);
          break;
        }
      }
      break;

    case Spawns::PenaltyKick: {
        Pose2D pose;
        if(player == KEEPER) pose = Pose2D(0, -HALF_FIELD_X, 0);
        else pose = Pose2D(0, PENALTY_CROSS_X - 1000, 0);
        auto newModel = new ParallelUKF(pose, params_);
        newModels.push_back(SpawnedModel(newModel, objectSets[0]));
        delete_ = true;
        complete(type);
      }
      break;
  }
  return newModels;
}

bool Spawn::ready(Spawns::SpawnType type) {
  return frames_[type] >= minframes_[type];
}

void Spawn::complete(Spawns::SpawnType type) {
  frames_[type] = 0;
}

void Spawn::reset() {
  frames_ = minframes_;
}

vector<ParallelUKF*> Spawn::flip(ParallelUKF* model) {
  Pose2D mpose(model->state()[ParallelUKF::SelfTheta], model->state()[ParallelUKF::SelfX], model->state()[ParallelUKF::SelfY]);
  Pose2D fpose = mpose;
  fpose.translation *= -1;
  fpose.rotation = normalizeAngle(fpose.rotation + M_PI);

  Point2D mball(model->state()[ParallelUKF::BallX], model->state()[ParallelUKF::BallY]);
  Point2D fball = -mball;

  vector<WorldObjectType> affinities;
  for(auto affinity : model->affinities()) {
    switch(affinity) {
      case WO_OWN_LEFT_GOALPOST:
        affinities.push_back(WO_OPP_LEFT_GOALPOST);
        break;
      case WO_OWN_RIGHT_GOALPOST:
        affinities.push_back(WO_OPP_RIGHT_GOALPOST);
        break;
      case WO_OPP_LEFT_GOALPOST:
        affinities.push_back(WO_OWN_RIGHT_GOALPOST);
        break;
      case WO_OPP_RIGHT_GOALPOST:
        affinities.push_back(WO_OWN_RIGHT_GOALPOST);
        break;
    }
  }

  auto m1 = new ParallelUKF(fpose, mball, model->covariance(), params_);
  m1->setAffinities(affinities);
  auto m2 = new ParallelUKF(fpose, fball, model->covariance(), params_);
  m2->setAffinities(affinities);
  vector<ParallelUKF*> newModels = { m1, m2 };
  return newModels;
}

PoseEstimate Spawn::computePoseEstimate(const vector<SeenObject>& objects) {
  if(objects.size() == 1) {
    printf("ERROR: Need at least two seen objects to compute a pose estimate.");
    return PoseEstimate();
  }

  int n = objects.size();
  MatrixXf A(n * 2, 4);
  MatrixXf b(n * 2, 1);
  for(int i = 0; i < n; i++) {
    const auto& obj = objects[i];
    //                     | cos T |
    // | xi  -yi  1  0 |   | sin T |   | xi' |
    // | yi   xi  0  1 | x |  tx   | = | yi' |
    //                     |  ty   |
    //
    //     A matrix         b matrix   x matrix
    A.block<1,4>(2*i, 0) << obj.relative.x, -obj.relative.y, 1, 0;
    A.block<1,4>(2*i + 1, 0) << obj.relative.y, obj.relative.x, 0, 1;
    b.block<2,1>(2*i, 0) << obj.global.x, obj.global.y;
    if(tlogger_) {
      tlog(63, "Pose Estimate Input: %s, glob: %2.f,%2.f, rel: %2.f,%2.f", getName(obj.type), obj.global.x, obj.global.y, obj.relative.x, obj.relative.y
      );
    }
  }
  VectorXf x = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);
  float theta = atan2f(x[1], x[0]);
  Pose2D pose(theta, x[2], x[3]);
  float error = sqrtf((A * x - b).squaredNorm() / objects.size());
  tlog(63, "Computed pose estimate from %i objects.", n);
  tlog(63, "Solution: %2.2f,%2.2f,%2.2f,%2.2f", x[0], x[1], x[2], x[3]);
  tlog(63, "Pose: %2.f,%2.f @ %2.2f, error %2.2f", pose.translation.x, pose.translation.y, pose.rotation * RAD_T_DEG, error);
  return PoseEstimate(pose, error);
}
