#include <vision/ml/HogFeatureExtractor.h>

HogFeatureExtractor::HogFeatureExtractor(cv::Size windowSize) : _windowSize(windowSize) {
  init();
}

void HogFeatureExtractor::init() {
  _extractor = std::make_unique<cv::HOGDescriptor>(
      _windowSize,
      cv::Size(16,16), // block size
      cv::Size(8,8), // block stride
      cv::Size(8,8), // cell size
      9 // n bins
  );
  _vocabularySize = 25;
  _vocabularySamples = _vocabularySize * 50 / _extractor->nbins;
  _quantized = false;
}

FVec HogFeatureExtractor::extractFeature(const cv::Mat& image) const {
  std::vector<float> hogs;
  std::vector<cv::Point> points;
  assert(image.channels() == 1);

  cv::Mat resized;
  cv::resize(image, resized, _windowSize);

  _extractor->compute(resized, hogs, _extractor->cellSize, cv::Size(0,0), points);
  FVec feature(hogs.size(), 1);
  for(int i=0; i < hogs.size(); i++)
    feature(i,0) = hogs[i];
  if(quantized())
    feature = feature.reshape(1, _extractor->nbins);
  if(vocabularyBuilt())
    feature = quantize(feature);
  return feature;
}

void HogFeatureExtractor::serialize(YAML::Emitter& emitter) const {
  FeatureExtractor::serialize(emitter);
  {
    emitter << YAML::Key << "WindowSize" << YAML::Value;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "width" << YAML::Value << _windowSize.width;
    emitter << YAML::Key << "height" << YAML::Value << _windowSize.height;
    emitter << YAML::EndMap;
  }
}

void HogFeatureExtractor::deserialize(const YAML::Node& node) {
  FeatureExtractor::deserialize(node);
  if(const YAML::Node *s = node.FindValue("WindowSize")) {
    (*s)["width"] >> _windowSize.width;
    (*s)["height"] >> _windowSize.height;
  }
  init();
}
