#pragma once

#include <vision/ml/Svm.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>

class OcvSvm : public Svm {
  public:
    bool train(const std::vector<FVec>& features, const std::vector<int>& labels) final;
    float predict(const FVec& feature) const final;
    void save(std::string file) const final;
    void load(std::string file) final;

  private:
    cv::SVM _cvsvm;
};
