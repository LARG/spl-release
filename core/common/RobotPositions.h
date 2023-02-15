#ifndef ROBOTPOSITIONS_H_SBZQRKAM
#define ROBOTPOSITIONS_H_SBZQRKAM

#include <math/Pose2D.h>
#include <common/WorldObject.h>
#include <common/Field.h>

const unsigned int NUM_PENALTY_POSES = 2;
const unsigned int NUM_ROBOTS_ON_TEAM = WO_TEAM_LAST-WO_TEAM_FIRST+1;
const unsigned int NUM_TEAM_POSES = NUM_ROBOTS_ON_TEAM + 1; // + 1 for default (placed at ind 0)

class RobotPositions {
  public:
    static Pose2D* penaltyPoses;
    // starting poses of robot, before game starts
    static Pose2D* startingSidelinePoses;
    // poses during our kickoff
    static Pose2D* ourKickoffPosesDesired;
    // poses during their kickoff
    static Pose2D* theirKickoffPosesDesired;
    // poses during our penalty Kick
    static Pose2D* ourKickoffPosesPenalty;
    // poses during their penalty kick
    static Pose2D* theirKickoffPosesPenalty;
};

#endif /* end of include guard: ROBOTPOSITIONS_H_SBZQRKAM */

