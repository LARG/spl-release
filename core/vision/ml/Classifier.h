#pragma once

#include <opencv2/core/core.hpp>

class Classifier {
  public:
    using Prediction = int;
    static constexpr int Nothing = 0;


    Classifier(const std::string& name, const std::string& directory);
    virtual ~Classifier() = default;

    Prediction classify(const cv::Mat& img) const {
      float certainty;
      return classify(img, certainty);
    }

    virtual std::vector<float> predict(const cv::Mat& img) const { return std::vector<float>(); }

    virtual Prediction classify(const cv::Mat& img, float& certainty) const = 0;
    virtual void load() = 0;
    virtual void save() const { }
    
    inline const std::string& name() const { return name_; }
    inline const std::string& directory() const { return directory_; }

  protected:
    std::string name_, directory_;
};
