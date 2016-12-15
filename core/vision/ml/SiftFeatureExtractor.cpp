#include <vision/ml/SiftFeatureExtractor.h>

SiftFeatureExtractor::SiftFeatureExtractor() : FeatureExtractor() {
  cv::initModule_nonfree();
  //_detector = cv::FeatureDetector::create("Dense");
  //_extractor = cv::DescriptorExtractor::create("SIFT");
  _detector = std::make_unique<cv::DenseFeatureDetector>();
  _extractor = std::make_unique<cv::SIFT>();
  _quantized = true;
}

FVec SiftFeatureExtractor::extractFeature(const cv::Mat& image) const {
  assert(image.channels() == 1);

  std::vector<cv::KeyPoint> keypoints;
  FVec sifts;

  _detector->detect(image, keypoints);
  _extractor->compute(image, keypoints, sifts);
  if(quantized())
    sifts = sifts.reshape(1, sifts.rows * sifts.cols);
  if(vocabularyBuilt())
    sifts = quantize(sifts);
  return sifts;
}
