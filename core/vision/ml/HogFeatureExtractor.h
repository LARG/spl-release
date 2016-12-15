#pragma once

#include <vision/ml/FeatureExtractor.h>
#include <opencv2/core/core.hpp>

class HogFeatureExtractor final : public FeatureExtractor {
  public:
    HogFeatureExtractor(cv::Size windowSize = cv::Size(64,64));
    FVec extractFeature(const cv::Mat& image) const final;

  protected:
    void deserialize(const YAML::Node& node) override;
    void serialize(YAML::Emitter& emitter) const override;

  private:
    void init();
    std::unique_ptr<cv::HOGDescriptor> _extractor;
    cv::Size _windowSize;
};
