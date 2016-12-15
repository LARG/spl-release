#include <vision/ml/RawFeatureExtractor.h>

RawFeatureExtractor::RawFeatureExtractor(cv::Size windowSize) : _windowSize(windowSize) {
  _quantized = false;
}

FVec RawFeatureExtractor::extractFeature(const cv::Mat& image) const {
  if(_windowSize == image.size())
    return image.reshape(1, image.channels() * image.rows * image.cols);
  cv::Mat rsimage;
  cv::resize(image, rsimage, _windowSize);
  return rsimage.reshape(1, rsimage.channels() * rsimage.rows * rsimage.cols);
}

void RawFeatureExtractor::serialize(YAML::Emitter& emitter) const {
  FeatureExtractor::serialize(emitter);
  {
    emitter << YAML::Key << "WindowSize" << YAML::Value;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "width" << YAML::Value << _windowSize.width;
    emitter << YAML::Key << "height" << YAML::Value << _windowSize.height;
    emitter << YAML::EndMap;
  }
}

void RawFeatureExtractor::deserialize(const YAML::Node& node) {
  FeatureExtractor::deserialize(node);
  if(const YAML::Node *s = node.FindValue("WindowSize")) {
    (*s)["width"] >> _windowSize.width;
    (*s)["height"] >> _windowSize.height;
  }
}
