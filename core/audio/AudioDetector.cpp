#include <audio/AudioDetector.h>
#include <audio/FFT.h>

AudioDetector::AudioDetector(MemoryCache& cache, TextLogger* tlogger, const FFTConfig& fftConfig) :
  cache_(cache), tlogger_(tlogger), fft_config_(fftConfig) { 
}

AudioDetector::~AudioDetector() {
}
