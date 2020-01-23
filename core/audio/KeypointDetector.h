#pragma once

#include <memory>
#include <memory/MemoryCache.h>
#include <iostream>
#include <cstdlib>
#include <vector>
#include <memory/AudioProcessingBlock.h>
#include <Eigen/Core>
#include <audio/AudioDetector.h>
#include <audio/WhistleUKF.h>
#include <audio/KeypointDetectorConfig.h>
#include <audio/Types.h>

class STFT;
class FFTConfig;
class KeypointGroup;

class KeypointDetector : public AudioDetector {
  public:
    KeypointDetector(MemoryCache& cache, TextLogger* tlogger, const FFTConfig& fftConfig);
    ~KeypointDetector();
    bool detect() final;
    inline float score() const final { return score_; }
    inline const auto& config() const { return whistle_config_; }
    inline float threshold() const {
      return whistle_config_.threshold;
    };
    void updateConfig() final;
    void reset() final;
    inline const std::unique_ptr<WhistleUKF>& filter() const { return filter_; }

  protected:
    float score_ = 0.0f;
    int spectrum_index_ = 0;
    std::vector<Eigen::MatrixXf> divideSpectra(const KeypointGroup& config);
    float processKeypoints(const KeypointGroup& config, const Eigen::MatrixXf& division);
    void processSpectrum(const float* spectrum, int length);
    void processSpectra(const Spectra& spectra);

    float computeKeypointScore(const KeypointGroup& kp, const Spectrum& spectrum);
    float computeKeypointScore(const Keypoint& kp, const Spectrum& spectrum, const SpectrumStats& stats);

    inline const decltype(KeypointDetectorConfig::keypoint_groups)& configs() const {
      return whistle_config_.keypoint_groups;
    };

    KeypointDetectorConfig whistle_config_;
    mutable Spectra spectra_;
    std::unique_ptr<WhistleUKF> filter_;
};
