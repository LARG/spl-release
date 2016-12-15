#pragma once

#include <vision/ml/FeatureExtractor.h>

class SiftFeatureExtractor : public FeatureExtractor {
  public:
    SiftFeatureExtractor();
    FVec extractFeature(const cv::Mat&) const final;
  private:
    std::unique_ptr<cv::FeatureDetector> _detector;
    std::unique_ptr<cv::DescriptorExtractor> _extractor;
};
