#include <audio/KangarooDetector.h>
#include <audio/STFT.h>
#include <memory/AudioProcessingBlock.h>
#include <memory/FrameInfoBlock.h>
#include <cmath>

constexpr int Rate = AudioProcessingBlock::SampleRate;
constexpr int NChannels = AudioProcessingBlock::NumChannels;

KangarooDetector::KangarooDetector(MemoryCache& cache, TextLogger* tlogger, const FFTConfig& fftConfig)
  : AudioDetector(cache, tlogger, fftConfig) {
  updateConfig();
}

KangarooDetector::~KangarooDetector() { 
}

void KangarooDetector::updateConfig() {
  initConfig();
  validateConfig();
  initConfig();
  validateConfig();
  fft_ = std::make_unique<STFT>(fft_config_,
    std::bind(&KangarooDetector::handleSpectrum, this, std::placeholders::_1, std::placeholders::_2)   
  );
}

void KangarooDetector::initConfig() {
  config_.whistle_freq_begin = 2000;
  config_.whistle_freq_end = 3500;

  config_.whistle_z_thresh = 5.0;
  config_.whistle_pos_frames = 16;
  config_.whistle_neg_frames = 4;
}

void KangarooDetector::validateConfig() {
  const float fWhistleBegin = config_.whistle_freq_begin;
  const float fWhistleEnd = config_.whistle_freq_end;
  if(fWhistleBegin < 0) {
    std::cerr << "Whistle begin is below zero!" << std::endl;
    exit(EXIT_FAILURE);
  }
  if(fWhistleEnd   < 0) {
    std::cerr << "Whistle end is below zero!" << std::endl;
    exit(EXIT_FAILURE);
  }
  if(fWhistleBegin > (Rate / 2)) {
    std::cerr << "Whistle begin is above Nyquist frequency!" << std::endl;
    exit(EXIT_FAILURE);
  }
  if(fWhistleEnd   > (Rate / 2)) {
    std::cerr << "Whistle end is above Nyquist frequency!" << std::endl;
    exit(EXIT_FAILURE);
  }
  if(fWhistleBegin > fWhistleEnd) {
    std::cerr << "Whistle begin is above Whistle end!" << std::endl;
    exit(EXIT_FAILURE);
  }
}

bool KangarooDetector::detect() {
  success_ = false;
  score_ = 0.0f;
  fft_->processSamples(cache_.audio_processing->buffer_.data(), cache_.audio_processing->buffer_.size(), NChannels);
  return success_;
}

void KangarooDetector::handleSpectrum(const float *spectrum, int length) {
  float mean, dev;
  calcMeanDeviation(spectrum, length, mean, dev);

  bool found;
  const float whistleThresh = mean + config_.whistle_z_thresh * dev;
  found = false;

  int i;
  auto wbegin = fft_config_.frequency_index(config_.whistle_freq_begin);
  auto wend = fft_config_.frequency_index(config_.whistle_freq_end);
  for(i = wbegin; i <= wend; ++i) {
    if(spectrum[i] > whistleThresh) {
      found = true;
      break;
    }
  }

  if(whistle_done_) {
    if(!found) {
      ++whistle_misses_;
      if(whistle_misses_ > config_.whistle_neg_frames) {
        whistles_ = 0;
        whistle_misses_ = 0;
        whistle_done_ = false;
      }
    }
  }
  else
  {
    if(found) {
      //printf("counter: %i\n", whistles_);
      ++whistles_;
      whistle_misses_ = 0;
    } else if(whistles_ > 0) {
      ++whistle_misses_;
      if(whistle_misses_ > config_.whistle_neg_frames) {
        whistles_ = 0;
        whistle_misses_ = 0;
        whistle_done_ = false;
      }
    }
    if(whistles_ >= config_.whistle_pos_frames) {
      success_ = true;
      score_ = 1.0f;
      whistles_ = 0;
      whistle_misses_ = 0;
      whistle_done_ = true;
    }
  }
  //printf("found: %i, whastles: %i, masses: %i\n", found, whistles_, whistle_misses_);
};

void KangarooDetector::calcMeanDeviation(const float *data, int length, float &mean, float &dev) const {
  mean = dev = 0;
  for(int i = 0; i < length; ++i) {
    mean    += data[i];
    dev     += data[i] * data[i];
  }

  dev = std::sqrt(length * dev - mean * mean) / length;
  mean /= length;
};
