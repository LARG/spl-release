#pragma once

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/flann/flann.hpp>
#include <algorithm>

#include <common/YamlConfig.h>
#include <vision/ml/Typedefs.h>

class FeatureExtractor : public YamlConfig {
  public:
    virtual ~FeatureExtractor() = default;
    FVec processImage(const cv::Mat& image);
    virtual FVec extractFeature(const cv::Mat& image) const = 0;
    std::vector<FVec> quantize(std::vector<FVec>& features) const;
    FVec quantize(FVec& feature)const ;
    static std::vector<FVec> combine(FeatureSet& fset);

    void reset();
    std::vector<FVec> getFeatures();
    void clearFeatures();

    inline std::vector<int>& labels() { return _labels; }
    inline const std::vector<int>& labels() const { return _labels; }
    inline bool quantized() const { return _quantized; }
    inline bool& quantized() { return _quantized; }
    inline int quantizePoint() const { return _quantizePoint; }
    inline int& quantizePoint() { return _quantizePoint; }

    inline bool vocabularyBuilt() const { return _index != nullptr; }
    inline int vocabularySize() const { return _vocabularySize; }
    inline int vocabularySamples() const { return _vocabularySamples; }

  protected:
  
    int _vocabularySize = 100, _vocabularySamples = 10;
    bool _quantized = true;
    int _quantizePoint = 1000;
    
  protected:
    void buildVocabulary();
    
    void deserialize(const YAML::Node& node) override;
    void serialize(YAML::Emitter& emitter) const override;
   
  private:
    std::unique_ptr<cv::flann::Index> _index;
    std::vector<FVec> _features, _qfeatures;
    std::vector<int> _labels;
    FVec _vocabulary;
};
