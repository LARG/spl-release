#pragma once

#include <vision/ml/Classifier.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

namespace caffe { template<typename T> class Net; }

class DeepClassifier : public Classifier {
  public:
    DeepClassifier(const std::string& name, const std::string& directory);
    ~DeepClassifier();

    using Classifier::classify;
    Prediction classify(const cv::Mat& image, float& certainty) const final;
    std::vector<float> predict(const cv::Mat& img) const;
    void load() final;

  private:
    void setMean(const std::string& mean_file);
    void wrapInputLayer(std::vector<cv::Mat>* input_channels) const;
    void preprocess(const cv::Mat& img, std::vector<cv::Mat>* input_channels) const;

  private:
    std::unique_ptr<caffe::Net<float>> net_;
    cv::Size input_geometry_;
    int num_channels_;
    cv::Mat mean_;
};
