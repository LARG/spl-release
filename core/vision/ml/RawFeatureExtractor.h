#pragma once

#include <vision/ml/FeatureExtractor.h>
#include <opencv2/core/core.hpp>

class RawFeatureExtractor final : public FeatureExtractor {
  public:
    RawFeatureExtractor(cv::Size windowSize = cv::Size(64,64));
    FVec extractFeature(const cv::Mat& image) const final;

  protected:
    void deserialize(const YAML::Node& node) override;
    void serialize(YAML::Emitter& emitter) const override;

  private:
    cv::Size _windowSize;
};
