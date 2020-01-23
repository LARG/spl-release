/*!
 * \brief Short Time Fourier Transform.
 * \author Thomas Hamboeck, Austrian Kangaroos 2014
 * Updated: Jake Menashe, UT Austin Villa 2016
 * Updated: Ishan Durugkar, UT Austin Villa 2019
 */
#include <audio/STFT.h>
#include <complex>
#include <common/Profiling.h>

std::unique_ptr<FFTConfig> STFT::DefaultConfig = std::make_unique<FFTConfig>();
STFT::SpectrumHandler DefaultHandler = [](const float*,int){};

STFT::STFT(const FFTConfig& config, SpectrumHandler handler) : 
    config_(config), handle_spectrum_(handler), overflown_data_(config.window_size) {
  
  input_  = static_cast<float*>(fftwf_malloc(sizeof(float) * config_.window_size));
  output_ = static_cast<fftwf_complex*>(fftwf_malloc(sizeof(fftwf_complex) * config_.window_size_half()));
  output_mag_ = std::vector<float>(config_.window_size_half());

  for(int i = 0; i < config_.window_size; ++i)
    input_[i] = 0.0f;

  plan = fftwf_plan_dft_r2c_1d(config_.window_size, input_, output_, FFTW_MEASURE);
}

STFT::STFT() : STFT(*DefaultConfig, DefaultHandler) {
}

STFT::STFT(const FFTConfig& config) : STFT(config, DefaultHandler) {
}

STFT::~STFT() {
  if(input_) {
    fftwf_free(input_);
  }
  if(output_) {
    fftwf_free(output_);
  }
  if(plan) {
    fftwf_destroy_plan(plan);
  }
}

Spectra STFT::computeSpectra(const int16_t* data, int length, short channels) const {
  Spectra spectra(config_.spectrum_count(), config_.spectrum_length());
  int i = 0;
  auto handler = [&](const float* spectrum, int length) {
    spectra.block(i++, 0, 1, spectra.cols()) = Spectrum::Map(spectrum, length);
  };
  processSamples(data, length, channels, handler);
  return spectra;
}

std::vector<int16_t> STFT::truncateChannels(const int16_t *data, int length, short channels) const {
  length = std::min<int>(length / channels, config_.maximum_samples);
  std::vector<int16_t> merged(length, 0);
  std::copy(data, data + merged.size(), merged.data());
  return merged;
}

std::vector<int16_t> STFT::mergeChannels(const int16_t *data, int length, short channels) const {
  constexpr int cblength = AudioProcessingBlock::SamplesPerChannelPerBuffer;
  std::vector<int16_t> merged(data, data + length / channels);
  for(int i = 1; i < channels; i++) {
    std::transform(merged.begin(), merged.end(), data + i * cblength, merged.begin(),
      [i](const int16_t& left, const int16_t& right) {
        int mean = left;
        mean *= i;
        mean = mean + right;
        mean /= i + 1;
        return static_cast<int16_t>(mean);
    });
  }
  return merged;
}

void STFT::processSamples(const int16_t *data, int length, short channels, SpectrumHandler handler) const {
  int iBuffer = 0;

  auto merged = truncateChannels(data, length, channels);
  data = merged.data();
  length = merged.size();
  // std::cout << "Number of samples we are processing: " << length << std::endl;

  /* for each overflown data */
  while(iBuffer < n_overflow_) {
    input_[iBuffer] = convert(overflown_data_[iBuffer]);
    ++iBuffer;
  }

  int iBegin = 0, iDataChannel;
  while(iBegin + config_.window_size <= length) {

    iDataChannel = iBegin + config_.channel_offset;
    while(iBuffer < config_.window_size) {
      input_[iBuffer] = convert(data[iDataChannel]);
      ++iBuffer;
      iDataChannel++;
    }
    /* and the rest is zero */

    fftwf_execute(plan);

    /* calc magnitude */
    for(int i = 0; i < config_.window_size_half(); ++i) {
      auto amp = std::abs(*reinterpret_cast<std::complex<float>*>(&output_[i]));
      /*
        The multiplicative constant here should be a feature of the microphone sensitivity.
        We used 20.0 for v5 where the sensitivity was 20mV/P.
        For the v6 it is 250mV/P. But that value seems to be too large.
        Setting with trial and error for now.
      */
      auto db = std::max(-100.0f, 160.0f * std::log10(amp));
      output_mag_[i] = db;
    }
    handler(output_mag_.data(), config_.window_size_half());

    /* next cycle */
    iBuffer = 0;
    iBegin  += config_.window_step;
  }

  n_overflow_ = 0;
  iDataChannel = iBegin + config_.channel_offset;
  while(iBegin < length) {
    /* copy to overflow buffer */
    overflown_data_[n_overflow_] = data[iDataChannel];
    ++n_overflow_;
    ++iBegin;
    iDataChannel++;
  }
}
