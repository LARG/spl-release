#ifndef PENALTY_BALL_ESTIMATOR_H
#define PENALTY_BALL_ESTIMATOR_H

#include <vision/estimators/LikelihoodEstimator.h>

/// @ingroup vision
class PenaltyBallEstimator : public LikelihoodEstimator<6> {
  public:
    PenaltyBallEstimator() { setNames(); }
  protected:
    void setNames() {
      names_ = {
        "ball pixel \%",
        "g below \%",
        "circ dev",
        "height",
        "kwdisc",
        "field dist"
      //  "vel"
      };
    }
};

#endif
