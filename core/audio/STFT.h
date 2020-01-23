/*!
 * \brief Short Time Fourier Transform.
 * \author Thomas Hamboeck, Austrian Kangaroos 2014
 */

#pragma once

#include <fftw3.h>
#include <functional>
#include <vector>
#include <limits>
#include <memory>
#include <audio/FFT.h>
#include <audio/FFTConfig.h>
#include <audio/Types.h>

class STFT : public FFT {
  public:
    using SpectrumHandler = std::function<void(const float* spectrum, int length)>;
    STFT(const FFTConfig& config, SpectrumHandler handler);
    STFT(const FFTConfig& config);
    STFT();
    virtual ~STFT();
    inline void processSamples(const int16_t* data, int length, short channels) const final {
      processSamples(data, length, channels, handle_spectrum_);
    }
    Spectra computeSpectra(const int16_t* data, int length, short channels) const final;

  protected:
    void processSamples(const int16_t* data, int length, short channels, SpectrumHandler handler) const;
    // Truncate the front channel - fast but inaccurate
    std::vector<int16_t> truncateChannels(const int16_t* data, int length, short channels) const;
    // Take the mean of the four channels - slow but accurate
    std::vector<int16_t> mergeChannels(const int16_t* data, int length, short channels) const;
    inline float convert(int16_t i) const { 
      return static_cast<float>(i) / (std::numeric_limits<int16_t>::max() + 1); 
    }

    mutable int n_overflow_ = 0;
    mutable std::vector<int16_t> overflown_data_;
    mutable float* input_ = nullptr;
    mutable fftwf_complex* output_ = nullptr;
    mutable std::vector<float> output_mag_;
    
    static std::unique_ptr<FFTConfig> DefaultConfig;
    fftwf_plan plan;

    FFTConfig config_;
    std::function<void (const float *spectrum, int length)> handle_spectrum_;
};
