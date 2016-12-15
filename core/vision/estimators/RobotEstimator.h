#ifndef ROBOT_ESTIMATOR_H
#define ROBOT_ESTIMATOR_H

#include <vision/estimators/LikelihoodEstimator.h>

/// @ingroup vision
class RobotEstimator : public LikelihoodEstimator<10> {
  public:
    RobotEstimator() { setNames(); }
  protected:
    void setNames() {
      names_ = {
        "w/h",
        "h/w",
        "kdist/wdist",
        "kdist/hdist",
        "tf disc",
        "jersey \%",
        "gw \%",
        "w \%",
        "c \%",
        "wheight"
      };
    }
};

#endif
