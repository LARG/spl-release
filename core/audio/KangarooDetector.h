#pragma once

#include <memory>
#include <memory/MemoryCache.h>
#include <iostream>
#include <cstdlib>
#include <vector>
#include <audio/FFTConfig.h>
#include <memory/AudioProcessingBlock.h>
#include <audio/AudioDetector.h>

class STFT;

class KangarooDetector : public AudioDetector {
  public:
    KangarooDetector(MemoryCache& cache, TextLogger* tlogger, const FFTConfig& fftConfig);
    ~KangarooDetector();
    inline float score() { return score_; }
    bool detect() final;
    void updateConfig() final;

  protected:
    void initConfig();
    void validateConfig();
    void handleSpectrum(const float* spectrum, int length);
    void calcMeanDeviation(const float* data, int length, float& mean, float& dev) const;

    struct WhistleConfig {
      FFTConfig fft;
      int whistle_freq_begin, whistle_freq_end;
      float whistle_z_thresh, whistle_pos_frames, whistle_neg_frames;
    };

    WhistleConfig config_;
    float success_;
    int whistles_ = 0, whistle_misses_ = 0;
    bool whistle_done_ = false;
    float score_;
    std::vector<float> spectrum_data_;
};
