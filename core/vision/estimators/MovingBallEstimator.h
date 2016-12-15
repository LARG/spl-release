#ifndef MOVING_BALL_ESTIMATOR_H
#define MOVING_BALL_ESTIMATOR_H

#include <vision/estimators/LikelihoodEstimator.h>

/// @ingroup vision
class MovingBallEstimator : public LikelihoodEstimator<9> {
  public:
    MovingBallEstimator() { setNames(); }
  protected:
    void setNames() {
      names_ = {
        "ball white pixel \%",
        "ball undef pixel \%",
        "ball pixel \%",
        "g below \%",
        "circ dev",
        "height",
        "kwdisc",
        "field dist",
        "vel"
      };
    }
};

#endif
