#include <vision/ml/FeatureExtractor.h>
#include <common/Random.h>

FVec FeatureExtractor::processImage(const cv::Mat& image) {
  FVec feature = extractFeature(image);
  if(vocabularyBuilt()) {
    _qfeatures.push_back(feature);
  }
  else if (!quantized() || _features.size() < quantizePoint()) {
    _features.push_back(feature);
  }
  else {
    buildVocabulary();
    _qfeatures = quantize(_features);
    _features.clear();
    feature = quantize(feature);
    _qfeatures.push_back(feature);
  }
  return feature;
}

std::vector<FVec> FeatureExtractor::getFeatures() {
  if(quantized())
    return _qfeatures;
  return _features;
}

void FeatureExtractor::clearFeatures() {
  _qfeatures.clear();
  _features.clear();
}

std::vector<FVec> FeatureExtractor::combine(FeatureSet& fset) {
  assert(fset.size() > 0);
  std::vector<FVec> combined = fset[0];
  for(int i = 1; i < fset.size(); i++) {
    for(int j = 0; j < combined.size(); j++) {
      combined[j].push_back(fset[i][j]);
    }
  }
  return combined;
}

void FeatureExtractor::buildVocabulary() {
  printf("Building vocabulary...");
  fflush(stdout);
  Random::inst().shuffle_inplace(_features);
  FVec samples;
  int count = 0;
  for(int i = 0; i < _features.size(); i++) {
    if(_labels[i] == 1) {
      samples.push_back(_features[i]);
      count++;
    }
    if(count == _vocabularySamples) break;
  }
  cv::Mat l;
  cv::kmeans(samples, _vocabularySize, l, cv::TermCriteria(cv::TermCriteria::MAX_ITER, 1000, 0), 1, cv::KMEANS_RANDOM_CENTERS, _vocabulary);
  _index = std::make_unique<cv::flann::Index>(_vocabulary, cv::flann::KMeansIndexParams());
  printf("complete\n");
}

std::vector<FVec> FeatureExtractor::quantize(std::vector<FVec>& features) const {
  std::vector<FVec> qfeatures;
  for(int i = 0; i < features.size(); i++) {
    FVec& feature = features[i];
    feature = quantize(feature);
    qfeatures.push_back(feature);
  }
  return qfeatures;
}

FVec FeatureExtractor::quantize(FVec& feature) const {
  assert(_index != 0);
  FVec quantized(_vocabularySize, 1);
  quantized = 0;
  cv::Mat indices, dists;
  _index->knnSearch(feature, indices, dists, 1, cv::flann::SearchParams());
  for(int r = 0; r < indices.rows; r++) {
    int index = indices.at<int>(r,0);
    quantized(index,0)++;
  }
  return quantized;
}

void FeatureExtractor::serialize(YAML::Emitter& emitter) const {
  emitter << YAML::Key << "vocabulary" << YAML::Value;
  emitter << YAML::BeginSeq;
  for(int i = 0; i < _vocabulary.rows; i++) {
    emitter << YAML::BeginSeq;
    for(int j = 0; j < _vocabulary.cols; j++)
      emitter << _vocabulary(i, j);
    emitter << YAML::EndSeq;
  }
  emitter << YAML::EndSeq;
}

void FeatureExtractor::deserialize(const YAML::Node& node) {
  if(!quantized()) return;
  assert(node.size() > 0);
  const auto& vnode = node["vocabulary"];
  assert(vnode.size() > 0);
  assert(vnode[0].size() > 0);
  _vocabulary = FVec(vnode.size(), vnode[0].size());
  for(int i = 0; i < vnode.size(); i++) {
    const YAML::Node& vec = vnode[i];
    for(int j = 0; j < vec.size(); j++)
      vec[j] >> _vocabulary(i, j);
  }
  _index = std::make_unique<cv::flann::Index>(_vocabulary, cv::flann::KMeansIndexParams());
}
