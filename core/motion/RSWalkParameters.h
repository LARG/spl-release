#ifndef RSWALKPARAMETERS_BENPHJGZ
#define RSWALKPARAMETERS_BENPHJGZ

#include <math/Pose2D.h>

// only a subset of the full parameters, for now
struct RSWalkParameters {
  RSWalkParameters():
    set(false)
  {}

  bool set;
  Pose2D speedMax;
  Pose2D speedMaxChange;
  float speed;
};


#endif /* end of include guard: RSWALKPARAMETERS_BENPHJGZ */
