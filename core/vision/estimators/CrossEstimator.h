#ifndef CROSS_ESTIMATOR_H
#define CROSS_ESTIMATOR_H

#include <vision/estimators/LikelihoodEstimator.h>

/// @ingroup vision
class CrossEstimator : public LikelihoodEstimator<11> {
  public:
    CrossEstimator() { setNames(); }
  protected:
    void setNames() {
      names_ = {
        "g TL",
        "g TM",
        "g TR",
        "g L",
        "g R",
        "g BL",
        "g BM",
        "g BR",
        "p dist",
        "w width",
        "w height"
      };
    }
};

#endif
