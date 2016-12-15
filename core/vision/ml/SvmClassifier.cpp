#include <vision/ml/SvmClassifier.h>
#include <vision/ml/OcvSvm.h>
#include <common/Util.h>
#include <yuview/YUVImage.h>
#include <system_error>

#include <vision/ml/SiftFeatureExtractor.h>
#include <vision/ml/HogFeatureExtractor.h>
#include <vision/ml/RawFeatureExtractor.h>
#include <vision/ml/Util.h>

using namespace std;

SvmClassifier::SvmClassifier(const string& name, const string& directory, FeatureExtractors extractors, cv::Size windowSize)
  : Classifier(name, directory) {
  _svm = std::make_unique<OcvSvm>();
  if(extractors & FeatureExtractors::SIFT) {
    _siftExtractor = std::make_unique<SiftFeatureExtractor>();
    _siftExtractor->quantized() = false;
  }
  if(extractors & FeatureExtractors::QuantizedSIFT) {
    _siftExtractor = std::make_unique<SiftFeatureExtractor>();
    _siftExtractor->quantized() = true;
  }
  if(extractors & FeatureExtractors::HOG) {
    _hogExtractor = std::make_unique<HogFeatureExtractor>(windowSize);
    _hogExtractor->quantized() = false;
  }
  if(extractors & FeatureExtractors::QuantizedHOG) {
    _hogExtractor = std::make_unique<HogFeatureExtractor>(windowSize);
    _hogExtractor->quantized() = true;
  }
  if(extractors & FeatureExtractors::Raw) {
    _rawExtractor = std::make_unique<RawFeatureExtractor>(windowSize);
  }
}

SvmClassifier::SvmClassifier(SvmClassifier&& other) : Classifier(other) {
  _hogExtractor = std::move(other._hogExtractor);
  _siftExtractor = std::move(other._siftExtractor);
  _rawExtractor = std::move(other._rawExtractor);
  _svm = std::move(other._svm);

  _threshold = other._threshold;
  _trained = other._trained;
}

SvmClassifier::~SvmClassifier() {
}

void SvmClassifier::load() {
  _trained = true;
  _svm->load(directory() + "/" + name() + ".svm");
  
  string file = directory() + "/" + name() + ".vcb";
  ifstream fh(file.c_str());
  if(!fh.good()) {
    printf("WARNING: Attribute detector file %s doesn't exist.\n", file.c_str());
    throw std::system_error(std::make_error_code(std::errc::no_such_file_or_directory), "Attribute Detector configuration not found.");
  }
  YAML::Parser parser(fh);
  YAML::Node node;
  parser.GetNextDocument(node);
  if(const YAML::Node *sift = node.FindValue("SiftExtractor")) {
    _siftExtractor = std::make_unique<SiftFeatureExtractor>();
    *sift >> *_siftExtractor;
  }
  if(const YAML::Node *hog = node.FindValue("HOGExtractor")) {
    _hogExtractor = std::make_unique<HogFeatureExtractor>();
    *hog >> *_hogExtractor;
  }
  if(const YAML::Node *raw = node.FindValue("RawExtractor")) {
    _rawExtractor = std::make_unique<RawFeatureExtractor>();
    *raw >> *_rawExtractor;
  }
}

void SvmClassifier::save() const {
  if(!trained())
    throw std::runtime_error("Attempted saving an untrained attribute detector.");
  util::mkdir_recursive(directory());
  _svm->save(directory() + "/" + name() + ".svm");
  
  string file = directory() + "/" + name() + ".vcb";
  ofstream fh(file.c_str());
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;
  if(useSIFT())
    emitter << YAML::Key << "SiftExtractor" << YAML::Value << *_siftExtractor;
  if(useHOG())
    emitter << YAML::Key << "HOGExtractor" << YAML::Value << *_hogExtractor;
  if(useRaw())
    emitter << YAML::Key << "RawExtractor" << YAML::Value << *_rawExtractor;
  emitter << YAML::EndMap;
  fh << emitter.c_str();
}

bool SvmClassifier::train(const vector<string>& imageFiles, const vector<int>& labels) {
  vector<cv::Rect> boxes;
  for(int i = 0 ; i < imageFiles.size(); i++)
    boxes.push_back(cv::Rect(0,0,1000000,1000000));
  return train(imageFiles, boxes, labels);
}

bool SvmClassifier::train(const vector<string>& imageFiles, const vector<cv::Rect>& boxes, const vector<int>& labels) {
  int maxFile = 1000;
  if(maxFile >= imageFiles.size()) maxFile = imageFiles.size() - 1;
  if(useHOG()) {
    _hogExtractor->quantizePoint() = maxFile;
    _hogExtractor->labels() = labels;
  }
  if(useSIFT()) {
    _siftExtractor->quantizePoint() = maxFile;
    _siftExtractor->labels() = labels;
  }
  std::vector<cv::Mat> images;
  for(int i = 0; i < imageFiles.size(); i++) {
    printf("\rLoading training image %04i/%zu", i, imageFiles.size());
    string file = imageFiles[i];
    cv::Mat image = util::imread(file);
    images.push_back(image);
  }
  for(int i = 0; i < images.size(); i++) {
    printf("\rProcessing training image %04i/%zu", i, imageFiles.size());
    const auto& image = images[i];
    cv::Rect box = boxes[i];
    box = correctBoundingBox(box, image);
    cv::Mat roi = image(box);
    cv::Mat gray(roi.size(), CV_8U);
    if(useSIFT() || useHOG()) {
      if(roi.channels() != 1)
        cv::cvtColor(roi, gray, CV_RGB2GRAY);
      else
        gray = roi.clone();
    }
    try {
      if(useSIFT()) _siftExtractor->processImage(gray);
      if(useHOG()) _hogExtractor->processImage(gray);
      if(useRaw()) _rawExtractor->processImage(roi);
    } catch (std::exception& e) {
      fprintf(stderr, "failed training on file: %s\n", imageFiles[i].c_str());
      throw e;
    }
  }
  FeatureSet fset;
  if(useSIFT()) { 
    fset.push_back(_siftExtractor->getFeatures());
    _siftExtractor->clearFeatures();
  }
  if(useHOG()) { 
    fset.push_back(_hogExtractor->getFeatures());
    _hogExtractor->clearFeatures();
  }
  if(useRaw()) {
    fset.push_back(_rawExtractor->getFeatures());
    _rawExtractor->clearFeatures();
  }
  vector<FVec> combined = FeatureExtractor::combine(fset);
  assert(combined.size() == labels.size());
  printf("\nComputing support vectors\n");
  _trained = _svm->train(combined, labels);
  printf("Training complete\n");
  return _trained;
}

void SvmClassifier::normalize(vector<FVec>& features) const {
  for(auto& f : features)
    normalize(f);
}

void SvmClassifier::normalize(FVec& feature) const {
  float max = 0;
  for(int i = 0; i < feature.rows; i++) {
    for(int j = 0; j < feature.cols; j++) {
      float temp = feature(i,j);
      if(max < temp) max = temp;
    }
  }
  for(int i = 0; i < feature.rows; i++)
    for(int j = 0; j < feature.cols; j++)
      feature(i,j) /= max;
}

Classifier::Prediction SvmClassifier::classify(const cv::Mat& image, float& certainty) const {
  auto box = correctBoundingBox(cv::Rect(0, 0, image.cols, image.rows), image);
  cv::Mat roi = image(box);
  cv::Mat gray(roi.size(), CV_8U);
  if(useSIFT() || useHOG()) {
    if(roi.channels() != 1)
      cv::cvtColor(roi, gray, CV_RGB2GRAY);
    else
      gray = roi.clone();
  }
  FVec feature;
  if(useSIFT()) {
    assert(!_siftExtractor->quantized() || _siftExtractor->vocabularyBuilt());
    feature.push_back(_siftExtractor->extractFeature(gray));
  }
  if(useHOG()) {
    assert(!_hogExtractor->quantized() || _hogExtractor->vocabularyBuilt());
    feature.push_back(_hogExtractor->extractFeature(gray));
  }
  if(useRaw()) {
    feature.push_back(_rawExtractor->extractFeature(roi));
  }
  certainty = _svm->predict(feature);
  if(certainty > _threshold)
    return true;
  else
    return false;
}

cv::Rect SvmClassifier::correctBoundingBox(cv::Rect original, const cv::Mat& image) {
  if(original.x < 0) original.x = 0;
  if(original.y < 0) original.y = 0;
  if(original.x + original.width > image.cols) original.width = image.cols - original.x;
  if(original.y + original.height > image.rows) original.height = image.rows - original.y;
  return original;
}

