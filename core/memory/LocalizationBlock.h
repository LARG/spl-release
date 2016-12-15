#pragma once

#include <Eigen/Core>
#include <common/Enum.h>
#include <memory/MemoryBlock.h>
#include <math/Geometry.h>
#include <math/Pose2D.h>
#include <memory/WorldObjectBlock.h>
#include <schema/gen/LocalizationBlock_generated.h>
#define STATE_SIZE 10

#define MAX_MODELS_IN_MEM 4
#define MODEL_STATE_SIZE (MAX_MODELS_IN_MEM * STATE_SIZE)
#define MODEL_COV_SIZE (MAX_MODELS_IN_MEM * STATE_SIZE * STATE_SIZE)
#define WO_COV_SIZE (4 * NUM_WORLD_OBJS)

struct Sides {
  ENUM(SideType,
    Undefined,
    BlueLeft,
    BlueRight
  );
};

struct Spawns {
  ENUM(SpawnType,
    Ambiguity,
    MultiObject,
    InitialState,
    ReadyState,
    SetState,
    PenalizedState,
    BallFlip,
    Fallen,
    PenaltyKick,
    CenterCircle,
    GoalLines,
    Throwouts
  );
};

DECLARE_INTERNAL_SCHEMA(struct LocalizationBlock : public MemoryBlock {
public:
  SCHEMA_METHODS(LocalizationBlock);
  LocalizationBlock();
  SCHEMA_FIELD(int blueSide);

  SCHEMA_FIELD(int kfType);
  SCHEMA_FIELD(int bestModel);
  SCHEMA_FIELD(float bestAlpha);

  // indicate if there are more with significant likelihood that think we're facing the opposite way
  SCHEMA_FIELD(bool oppositeModels);
  SCHEMA_FIELD(bool fallenModels);
  SCHEMA_FIELD(int numMateFlippedBalls);
  SCHEMA_FIELD(int numBadBallUpdates);

  SCHEMA_FIELD(bool useSR);
  SCHEMA_FIELD(float factor);
  
  SCHEMA_FIELD(std::array<int,Spawns::NUM_SpawnTypes> spawnFrames);
  SCHEMA_FIELD(std::array<int,MAX_MODELS_IN_MEM> modelNumber);
  SCHEMA_FIELD(std::array<float,MAX_MODELS_IN_MEM> alpha);
  mutable SCHEMA_FIELD(std::array<float, MODEL_STATE_SIZE> state_data);
  std::array<Eigen::Matrix<float, STATE_SIZE, 1, Eigen::DontAlign>, MAX_MODELS_IN_MEM> state;

  mutable SCHEMA_FIELD(std::array<float, MODEL_COV_SIZE> covariance_data);
  std::array<Eigen::Matrix<float, STATE_SIZE, STATE_SIZE, Eigen::DontAlign>, MAX_MODELS_IN_MEM> covariance;

  mutable SCHEMA_FIELD(std::array<float, 4> ballSR_data);
  Eigen::Matrix<float, 2, 2, Eigen::DontAlign> ballSR;

  mutable SCHEMA_FIELD(std::array<float, WO_COV_SIZE> objectCov_data);
  std::array<Eigen::Matrix<float, 2, 2, Eigen::DontAlign>, NUM_WORLD_OBJS> objectCov;

  SCHEMA_PRE_SERIALIZATION({
    for(int i = 0; i < MAX_MODELS_IN_MEM; i++)
      std::copy(
        __source_object__.state[i].data(), 
        __source_object__.state[i].data() + __source_object__.state[i].size(), 
        __source_object__.state_data.data() + i * __source_object__.state[i].size()
      );
    for(int i = 0; i < MAX_MODELS_IN_MEM; i++)
      std::copy(
        __source_object__.covariance[i].data(), 
        __source_object__.covariance[i].data() + __source_object__.covariance[i].size(), 
        __source_object__.covariance_data.data() + i * __source_object__.covariance[i].size()
      );
    std::copy(
      __source_object__.ballSR.data(), 
      __source_object__.ballSR.data() + __source_object__.ballSR.size(), 
      __source_object__.ballSR_data.data()
    );
    for(int i = 0; i < NUM_WORLD_OBJS; i++)
      std::copy(
        __source_object__.objectCov[i].data(), 
        __source_object__.objectCov[i].data() + __source_object__.objectCov[i].size(), 
        __source_object__.objectCov_data.data() + i * __source_object__.objectCov[i].size()
      );
  });
  SCHEMA_POST_DESERIALIZATION({
    for(int i = 0; i < MAX_MODELS_IN_MEM; i++)
      std::copy(
        __target_object__.state_data.data() + i * __target_object__.state[i].size(), 
        __target_object__.state_data.data() + (i + 1) * __target_object__.state[i].size(),
        __target_object__.state[i].data()
      );
    for(int i = 0; i < MAX_MODELS_IN_MEM; i++)
      std::copy(
        __target_object__.covariance_data.data() + i * __target_object__.covariance[i].size(), 
        __target_object__.covariance_data.data() + (i + 1) * __target_object__.covariance[i].size(), 
        __target_object__.covariance[i].data()
      );
    std::copy(
      __target_object__.ballSR_data.data(), 
      __target_object__.ballSR_data.data() + __target_object__.ballSR.size(), 
      __target_object__.ballSR.data()
    );
    for(int i = 0; i < NUM_WORLD_OBJS; i++)
      std::copy(
        __target_object__.objectCov_data.data() + i * __target_object__.objectCov[i].size(), 
        __target_object__.objectCov_data.data() + (i + 1) * __target_object__.objectCov[i].size(), 
        __target_object__.objectCov[i].data()
      );
  });

  Pose2D getPlayerPose(int model);
  Point2D getBallPosition(int model);
  Point2D getBallVelocity(int model);
  Eigen::Matrix2f getPlayerCov(int model);
  Eigen::Matrix2f getBallCov(int model);
  float getOrientationVar(int model);
});

