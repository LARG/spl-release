#ifndef BALL_ESTIMATOR_H
#define BALL_ESTIMATOR_H

#include <vision/estimators/LikelihoodEstimator.h>

/// @ingroup vision
class BallEstimator : public LikelihoodEstimator<7> {
  public:
    BallEstimator() { setNames(); }
  protected:
    void setNames() {
      names_ = {
        "orange \%",
        "g/w \%",
        "circ dev",
        "height",
        "kwdisc",
        "field dist",
        "vel"
      };
    }
};

#endif
