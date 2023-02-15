#include "RobotPositions.h"

Pose2D* RobotPositions::startingSidelinePoses = new Pose2D[NUM_TEAM_POSES];
Pose2D* RobotPositions::ourKickoffPosesDesired = new Pose2D[NUM_TEAM_POSES];
Pose2D* RobotPositions::penaltyPoses = new Pose2D[NUM_PENALTY_POSES];
Pose2D* RobotPositions::theirKickoffPosesDesired = new Pose2D[NUM_TEAM_POSES];
Pose2D* RobotPositions::ourKickoffPosesPenalty = new Pose2D[NUM_TEAM_POSES];
Pose2D* RobotPositions::theirKickoffPosesPenalty = new Pose2D[NUM_TEAM_POSES];
