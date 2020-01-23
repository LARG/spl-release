#pragma once

#include <memory>

class MemoryCache;
class TextLogger;
class FFTConfig;
class FFT;

class AudioDetector {
  public:
    AudioDetector(MemoryCache& cache, TextLogger* tlogger, const FFTConfig& fftConfig);
    virtual ~AudioDetector();
    virtual bool detect() = 0;
    virtual float score() const = 0;
    virtual void updateConfig() = 0;
    virtual const FFT& fft() const { return *fft_; }
    inline bool detected() const { return detected_; }
    virtual void reset() { }
    virtual std::string type() const { return last_whistle_type_; }
  protected:
    bool detected_ = false;
    MemoryCache& cache_;
    TextLogger* tlogger_ = nullptr;
    const FFTConfig& fft_config_;
    std::unique_ptr<FFT> fft_;
    std::string last_whistle_type_;
};
