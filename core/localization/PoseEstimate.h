#pragma once

#include <math/Pose2D.h>

struct PoseEstimate {
  Pose2D pose;
  float error;
  PoseEstimate(Pose2D pose, float error) : pose(pose), error(error) { }
  PoseEstimate() : pose(0,0), error(0) { }
};
