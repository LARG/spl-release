#include <audio/KeypointDetector.h>
#include <audio/STFT.h>
#include <audio/Logging.h>

#include <audio/FFTConfig.h>
#include <audio/KeypointGroup.h>
#include <audio/KeypointDetectorConfig.h>

#include <memory/AudioProcessingBlock.h>
#include <memory/FrameInfoBlock.h>
#include <math/MatrixOperations.h>
#include <common/Profiling.h>
#include <common/Util.h>

#include <VisionCore.h>
#include <cmath>

#include <stdio.h>

constexpr int NChannels = AudioProcessingBlock::NumChannels;

template <typename T, typename = std::enable_if_t<std::is_floating_point<T>::value>>
constexpr T normal_pdf(T x, T m = 0, T s = 1) {
  constexpr T inv_sqrt_2pi = 0.3989422804014327;
  T a = (x - m) / s;
  return inv_sqrt_2pi / s * std::exp(-T(0.5) * a * a);
}

KeypointDetector::KeypointDetector(MemoryCache& cache, TextLogger* tlogger, const FFTConfig& fftConfig) 
  : AudioDetector(cache, tlogger, fftConfig) {
  reset();
  updateConfig();
}

KeypointDetector::~KeypointDetector() { 
}

void KeypointDetector::updateConfig() {
  using namespace std::placeholders;
  fft_ = std::make_unique<STFT>(fft_config_, std::bind(&KeypointDetector::processSpectrum, this, _1, _2));
  bool loaded = whistle_config_.load(util::format("%s/whistle_keypoint_config.yaml", util::cfgpath(util::Models)));
  if(!loaded)
    throw std::runtime_error("Unable to load whistle keypoint config.");
  printf("Loading whistle keypoints...%zu keypoints loaded.\n", whistle_config_.keypoint_groups.size());
}

void KeypointDetector::reset() {
  filter_ = std::make_unique<WhistleUKF>(WhistleUKFParams());
  filter_->tlogger() = tlogger_;
  detected_ = false;
}

int witer = 0;

bool KeypointDetector::detect() {
  detected_ = true;
  score_ = 0.0f;
  spectrum_index_ = 0;
  tlog(55, "Initial filter: %s", *filter_);
  // printf("\n\n\n Detecting\n");
  auto length = cache_.audio_processing->num_samples * NChannels;
  auto data = cache_.audio_processing->buffer_.data();
  // std::cout << "Num samples read: " << length << std::endl;
  // std::cout << "Max number is: " << std::max_element((int16_t*)data, (int16_t*)(data + length)) << std::endl;
  //fft_->processSamples(data, length, NChannels);
  //return score_ > threshold();
  spectra_ = fft_->computeSpectra(data, length, NChannels);
  // std::cout << "Got spectra" << std::endl;
  processSpectra(spectra_);
  // std::cout << "processed spectra. Score: " << score_ << " and Threshold: " << threshold() << std::endl << std::endl;
  if(witer == 0) {
      remove("whistle.txt");
      witer++;
  }
  FILE* fptr = fopen("whistle.txt", "a+");
  fprintf(fptr, "%f\n", score_);
  fclose(fptr);
  // if(score_ > threshold())
  //   std::cout << std::endl << "!!!! WHISTLE DETECTED !!!!" << std::endl;
  return score_ > threshold();
}

void KeypointDetector::processSpectra(const Spectra& spectra) {
  struct KeypointResult {
    KeypointResult(const KeypointGroup& kp) : kp(&kp), mean(0.0f) { }
    const KeypointGroup* kp;
    std::vector<float> scores;
    float mean;
  };
  std::vector<KeypointResult> results;
  results.reserve(configs().size());
  for(int j = 0; j < configs().size(); j++) {
    auto& config = configs()[j];
    results.emplace_back(config);
    auto& result = results.back();
    result.scores.reserve(spectra.rows());
    for(int i = 0; i < spectra.rows(); i++) {
      const auto& spectrum = spectra.block(i, 0, 1, spectra.cols());
      auto score = computeKeypointScore(config, spectrum);
      result.scores.push_back(score);
      if(i == 0) result.mean = score;
      else
        result.mean = (result.mean * i + score) / (i + 1);
    }
    tlog(55, "Config '%s' mean score: %2.2f, count: %i", config.name, result.mean, result.scores.size());
  }
  std::sort(results.begin(), results.end(), [](const auto& left, const auto& right) {
    return left.mean > right.mean;
  });
  auto& best = results[0];
  tlog(55, "Selected config: %s", best.kp->name);
#ifdef TOOL
  last_whistle_type_ = best.kp->name;
#endif
  for(int i = 0; i < best.scores.size(); i++) {
    auto score = best.scores[i];
    WhistleObservation obs;
    obs.scores.push_back(score);
    filter_->update(obs);
    tlog(55, "Processed spectrum %i:\n\t%s\n\t%s", i, obs, *filter_);
  }
  score_ = filter_->score();
}

void KeypointDetector::processSpectrum(const float* spectrum, int length) {
  Spectrum s(length);
  s = Spectrum::Map(spectrum, length);
  std::vector<float> scores;
  scores.reserve(configs().size());
  for(const auto& config : configs()) {
    tlog(42, "using config: %s", config.name);
    float score = processKeypoints(config,s);
    scores.push_back(score);
  }
  float score = std::accumulate(scores.begin(), scores.end(), 0.0f) / scores.size();
  WhistleObservation obs;
  obs.scores.push_back(score);
  filter_->update(obs);
  tlog(55, "Spectrum %i filter stats: %s, last obs: %s", spectrum_index_++, *filter_, obs);
  score_ = std::max(filter_->score(), score_);
}

std::vector<Eigen::MatrixXf> KeypointDetector::divideSpectra(const KeypointGroup& config) {
  std::vector<Eigen::MatrixXf> divisions;
  float period = 55.0f;
  int divSize = fft_config_.window_count(period);
  tlog(42, "got div size of %i from period of %2.2f", divSize, period);
  for(int i = 0; i < spectra_.size(); i++) {
    if(i + divSize >= spectra_.rows()) break;
    const auto& division = spectra_.block(i, 0, divSize, spectra_.cols());
    divisions.push_back(division);
  }
  tlog(42, "done creating divisions");
  return divisions;
}

float KeypointDetector::processKeypoints(const KeypointGroup& config, const Eigen::MatrixXf& division) {
  auto stats = MatrixOperations::ComputeStatistics(division);
  tlog(42, "-------------- div -------------");
  tlog(42, "stats: mean: %2.2f, stddev: %2.2f, max: %2.2f, min: %2.2f",
    stats.mean, stats.stddev, stats.max, stats.min
  );
  float weights = 0.0f, scores = 0.0f;
  tlog(43, "computing keypoints");
  for(const auto& kp : config.keypoints) {
    scores += computeKeypointScore(kp, division, stats) * kp.weight;
    weights += kp.weight;
  }
  float mscore = scores / weights;
  tlog(42, "Total score: %2.2f, Total Weight: %2.2f, Final Score: %2.2f", scores, weights, mscore);
  tlog(42, "--------------------------------");
  return mscore;
}
  
float KeypointDetector::computeKeypointScore(const KeypointGroup& config, const Spectrum& spectrum) {
  auto stats = MatrixOperations::ComputeStatistics(spectrum);
  float weights = 0.0f, scores = 0.0f;
  for(const auto& kp : config.keypoints) {
    scores += computeKeypointScore(kp, spectrum, stats) * kp.weight;
    weights += kp.weight;
  }
  return scores / weights;
}

float KeypointDetector::computeKeypointScore(const Keypoint& kp, const Spectrum& spectrum, const SpectrumStats& stats) {
  int fstart = fft_config_.frequency_index(kp.fstart);
  int fend = fft_config_.frequency_index(kp.fend);
  if(fstart > fend) {
    tlog(43, "invalid keypoint; fstart: %i, fend: %i, kp:\n%s", fstart, fend, kp);
    return 0.0f;
  }
  if(fend >= spectrum.cols()) {
    tlog(43, "invalid keypoint; fstart: %i, fend: %i (<= div cols %i), kp:\n%s", fstart, fend, spectrum.cols(), kp);
  }
  Eigen::MatrixXf m = spectrum.block(0, fstart, spectrum.rows(), fend - fstart + 1);
  auto kpstats = MatrixOperations::ComputeStatistics(m);
  tlog(43, "analyzing keypoint:\n%s", kp);
  tlog(49, "analyzing over spectrum:\n%s\n", m);
  tlog(43, "fstart: [%2.2f]->%i, fend: [%2.2f]->%i", kp.fstart, fstart, kp.fend, fend);
  tlog(43, "kpstats: mean: %2.2f, stddev: %2.2f, max: %2.2f, min: %2.2f",
    kpstats.mean, kpstats.stddev, kpstats.max, kpstats.min
  );
  float mean = kpstats.mean;
  float z = (kpstats.mean - stats.mean) / stats.stddev;
  if(kp.type == Keypoint::Type::Peak) { // Peaks that are higher than expected are ok
    if(mean > kp.db_mean) mean = kp.db_mean;
    if(z > kp.dbz_mean) z = kp.dbz_mean;
  } else {   // Valleys that are lower than expected are ok
    if(mean < kp.db_mean) mean = kp.db_mean;
    if(z < kp.dbz_mean) z = kp.dbz_mean;
  }
  float mz = (mean - kp.db_mean) / kp.db_stddev;
  float zz = (z - kp.dbz_mean) / kp.dbz_stddev;

  static constexpr float base = normal_pdf(0.0f);
  float mscore = normal_pdf(mz) / base;
  float zscore = normal_pdf(zz) / base;
  float score = mscore * zscore;
  tlog(42, "base: %2.2f, m: %2.2f, mz: %2.2f, mscore: %2.2f, z: %2.2f, zz: %2.2f, zscore: %2.2f, score: %2.2f",
    base, mean, mz, mscore, z, zz, zscore, score
  );
  return score;
};

