#pragma once

#include <vision/ObjectDetector.h>
#include <vision/ColorSegmenter.h>

class BandSampler;
class SvmClassifier;
class ROIDetector;
class Band;
class Classifier;

/// The RAndomized Machine-learned Bias-Oriented Goal Detector
/// :)

class RamboGoalDetector : public ObjectDetector {
  public:
    RamboGoalDetector(DETECTOR_DECLARE_ARGS, const ROIDetector& roi_detector, const ColorSegmenter& segmenter);
    ~RamboGoalDetector();
    void processFrame();
    void init(TextLogger* tl) final;
    inline const BandSampler* sampler() { return sampler_.get(); }
  private:
    void setGoalObject(const Band& band, bool left);
    std::unique_ptr<BandSampler> sampler_;
    std::unique_ptr<Classifier> classifier_;
    const ROIDetector& roi_detector_;
    const ColorSegmenter& color_segmenter_;
};

