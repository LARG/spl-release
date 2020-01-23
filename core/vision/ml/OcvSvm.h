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

// OpenCV2
// SN: In theory, we should just change our local install to also use OPENCV3, but this might affect large parts of the code base, so for now we are if-defing it
#ifdef USE_OPENCV2
    cv::SVM _cvsvm;
// OpenCV3    
#else
    cv::Ptr<cv::ml::SVM> _cvsvm;
#endif

};
