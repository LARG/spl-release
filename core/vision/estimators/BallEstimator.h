#ifndef BALL_ESTIMATOR_H
#define BALL_ESTIMATOR_H

#include <vision/estimators/LikelihoodEstimator.h>

/// @ingroup vision
class BallEstimator : public LikelihoodEstimator<6> {
  public:
    BallEstimator() { setNames(); }
  protected:
    void setNames() {
      names_ = {
        "below green \%",
        "g/w \%",
        "height",
        "kw disc",
        "field dist",
        "vel"
      };
    }
};

#endif
