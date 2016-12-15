#pragma once

#include <vision/ml/Classifier.h>
#include <vision/ml/Svm.h>
#include <vision/ml/FeatureExtractors.h>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>

class HogFeatureExtractor;
class SiftFeatureExtractor;
class RawFeatureExtractor;

typedef std::vector<std::vector<FVec>> FeatureSet;

class SvmClassifier : public Classifier {
  public:
    SvmClassifier(const std::string& name, const std::string& directory, FeatureExtractors extractors = FeatureExtractors::Undefined, cv::Size windowSize = cv::Size(64,64));
    SvmClassifier(SvmClassifier&&);
    ~SvmClassifier();
    using Classifier::classify;
    Prediction classify(const cv::Mat& image, float& certainty) const final;
    void load() final;
    void save() const final;
    bool train(const std::vector<std::string>& imageFiles, const std::vector<int>& labels);
    bool train(const std::vector<std::string>& imageFiles, const std::vector<cv::Rect>& boxes, const std::vector<int>& labels);
    inline bool useHOG() const { return _hogExtractor != nullptr; }
    inline bool useSIFT() const { return _siftExtractor != nullptr; }
    inline bool useRaw() const { return _rawExtractor != nullptr; }
    inline float& threshold() { return _threshold; }
    inline float threshold() const { return _threshold; }
    inline bool trained() const { return _trained; }

  protected:
    void normalize(FVec& feature) const;
    void normalize(std::vector<FVec>& features) const;
    static cv::Rect correctBoundingBox(cv::Rect original, const cv::Mat& image);

  private:
    std::unique_ptr<HogFeatureExtractor> _hogExtractor;
    std::unique_ptr<SiftFeatureExtractor> _siftExtractor;
    std::unique_ptr<RawFeatureExtractor> _rawExtractor;
    std::unique_ptr<Svm> _svm;

    float _threshold = 0.0;
    bool _trained = false;
};
